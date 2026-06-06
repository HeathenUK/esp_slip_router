#!/bin/bash
# USB-CDC OTA for the T-Dongle S3 — no bootloader, no esptool, no WiFi.
#
# Streams firmware.bin straight into the inactive OTA partition over the
# existing modem CDC link via the dongle's AT$OTASTART command:
#
#   host:   AT$OTASTART=<bytes>\r
#   dongle: \r\nOTA READY\r\n
#   host:   <bytes raw firmware.bin>
#   dongle: \r\nOTA OK\r\n          ← then esp_restart, new fw boots
#
# Usage:  ./flash.sh [firmware.bin]
#   defaults to ../tdongle-s3-idf/build/tdongle_s3.bin (the active IDF
#   build). The arduino-esp32 build at ../tdongle-s3/.pio/build/dos/
#   is no longer the production path -- if you need it, pass explicitly.
#   Note: firmware.bin (app only), NOT firmware.factory.bin — OTA can't
#   replace the bootloader from the running app.
#
# Fallback: if the dongle isn't responding to AT (running pre-OTA firmware
# or hung), use `./flash.sh --boot` to invoke the AT$BOOT path instead —
# reboots into ROM bootloader for esptool to take over.

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"

if [ "${1:-}" = "--boot" ]; then
    exec "$HERE/flash-bootloader.sh" "${2:-}"
fi

FW="${1:-$HERE/../tdongle-s3-idf/build/tdongle_s3.bin}"
[ -f "$FW" ] || { echo "flash: no firmware at $FW" >&2; exit 1; }

# Match either the original CDC-only build's tty (F412FA44AC4C1) or the
# composite CDC+MSC build's (...AC4C2 -- TinyUSB rebases the per-interface
# suffix when MSC joins).
DONGLE=$(ls /dev/cu.usbmodemF412FA44AC4C* 2>/dev/null | head -1)
if [ -z "$DONGLE" ] || [ ! -e "$DONGLE" ]; then
    echo "flash: dongle not present (/dev/cu.usbmodemF412FA44AC4C*)" >&2
    echo "flash: if it's in ROM bootloader (usbmodem123401), use: $0 --boot" >&2
    exit 1
fi

SIZE=$(wc -c < "$FW" | tr -d ' ')
echo "flash: $FW ($SIZE bytes) -> $DONGLE"

# Pre-flash firmware version (from ATI "ver=<hash>"). The ONLY trustworthy
# success signal is this version CHANGING after the reboot -- NOT an in-band
# "OTA OK" match: the 1 MB firmware contains the literal bytes "OTA OK"/"OTA
# READY" in .rodata, and a desynced OTA used to echo them back, faking success
# while nothing flashed. (Mirrors ota-http.sh's version-flip rule.) Pre-stream
# "OTA READY" is still safe to match -- no firmware bytes are in the stream yet.
PREVF=$(mktemp)

python3 - "$DONGLE" "$FW" "$SIZE" "$PREVF" <<'PY'
import sys, time, re, serial

dev, fw_path, sz, prevf = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
p = serial.Serial(dev, 115200, timeout=10)
time.sleep(0.3)
p.reset_input_buffer()

def read_for(secs):
    end = time.time() + secs; buf = b""
    while time.time() < end:
        b = p.read(p.in_waiting or 1)
        if b: buf += b
    return buf

# Capture the running version so the post-reboot check can confirm it changed.
p.write(b"ATI\r"); p.flush()
m = re.search(rb"ver=([0-9a-fA-F]+)", read_for(1.5))
prev = m.group(1).decode() if m else ""
open(prevf, "w").write(prev)
print(f"flash: pre-flash version: {prev or '(unknown)'}")

# Trigger the OTA receive loop on the dongle.
p.reset_input_buffer()
p.write(f"AT$OTASTART={sz}\r".encode())
p.flush()

def wait_line(needle, max_wait=8.0):
    end = time.time() + max_wait
    buf = b""
    while time.time() < end:
        b = p.read(p.in_waiting or 1)
        if b:
            buf += b
            if needle in buf:
                return True, buf
    return False, buf

# esp_ota_begin() erases the inactive partition before responding — that's
# 3-5s on flash. Safe to match in-band: no firmware bytes are in flight yet.
ok, txt = wait_line(b"OTA READY", max_wait=20.0)
if not ok:
    print("flash: dongle didn't enter OTA mode. Reply:", txt[:256], file=sys.stderr)
    sys.exit(2)
print(f"flash: OTA READY — streaming {sz} bytes...")

# Stream firmware in chunks; pyserial respects USB CDC backpressure (bulk-OUT
# NAKs from device when its RX queue is full).
sent = 0
t0 = time.time()
with open(fw_path, "rb") as f:
    while sent < sz:
        chunk = f.read(2048)
        if not chunk: break
        p.write(chunk)
        sent += len(chunk)
        if sent % (64 * 1024) == 0 or sent == sz:
            pct = sent * 100 // sz
            rate = sent / max(time.time() - t0, 0.001) / 1024
            print(f"  {sent:>8}/{sz} ({pct:3}%) @ {rate:.0f} KB/s", flush=True)
p.flush()
dt = time.time() - t0
print(f"flash: sent {sent} bytes in {dt:.1f}s ({sent/dt/1024:.0f} KB/s)")

# Fast-fail ONLY on an explicit device error code (OTA WRITE-FAIL / TIMEOUT /
# END-FAIL / SETBOOT-FAIL). We do NOT trust an in-band "OTA OK" -- success is
# proven by the version flip below. On a real failure the firmware now arms a
# parser drain and stays up on the OLD version (creds safe), so the version
# simply won't change.
txt = read_for(6.0)
for code in (b"WRITE-FAIL", b"TIMEOUT", b"END-FAIL", b"SETBOOT-FAIL", b"BADSIZE", b"NOPART", b"BEGIN-FAIL"):
    if b"OTA " + code in txt:
        print(f"flash: device reported OTA {code.decode()} — flash aborted on-device (still on old fw).", file=sys.stderr)
        p.close(); sys.exit(3)
p.close()
print("flash: stream sent; verifying by version change (authoritative)...")
PY
RC=$?
[ "$RC" -ne 0 ] && { rm -f "$PREVF"; exit "$RC"; }
PREV=$(cat "$PREVF" 2>/dev/null); rm -f "$PREVF"

# Post-reboot: the dongle USB-disappears for ~1-2s then reappears at the same
# path (TinyUSB serial is MAC-based, stable across reboots). Wait for the gap
# (so we don't open the stale tty), then wait for it to reappear, then verify
# by polling ATI until "ver=" differs from PREV. A failed OTA does NOT reboot
# (it drains and stays up), so a version that never flips == flash did not take.
echo "flash: waiting for dongle to reboot..."
for i in $(seq 1 30); do
    sleep 0.3
    [ ! -e "$DONGLE" ] && { echo "flash:  -> disappeared (rebooting)"; break; }
done
echo "flash: waiting for new firmware to enumerate..."
for i in $(seq 1 40); do
    sleep 0.3
    [ -e "$DONGLE" ] && { echo "flash:  -> back at $DONGLE"; break; }
done
[ -e "$DONGLE" ] || { echo "flash: dongle didn't reappear — manual replug?" >&2; exit 5; }
sleep 1.0    # let descriptors settle
python3 - "$DONGLE" "$PREV" <<'PY'
import serial, time, sys, re
dev, prev = sys.argv[1], sys.argv[2]
deadline = time.time() + 30          # generous: covers reboot + descriptor settle
while time.time() < deadline:
    try:
        p = serial.Serial(dev, 115200, timeout=2)
    except Exception:
        time.sleep(0.5); continue
    try:
        time.sleep(0.3); p.reset_input_buffer()
        p.write(b"ATI\r"); p.flush(); time.sleep(0.8)
        r = p.read(512)
        m = re.search(rb"ver=([0-9a-fA-F]+)", r)
        cur = m.group(1).decode() if m else ""
        p.close()
        if cur and cur != prev:
            print(f"flash: FLASHED OK — version {prev or '?'} -> {cur}")
            sys.exit(0)
        if cur and cur == prev:
            # device is up but unchanged; keep polling a little in case it's
            # still the pre-reboot instance answering, then conclude.
            pass
    except Exception:
        try: p.close()
        except Exception: pass
    time.sleep(1.0)
print(f"flash: version did NOT change from {prev or '?'} — flash did not take (device still on old fw, creds intact).", file=sys.stderr)
sys.exit(4)
PY
