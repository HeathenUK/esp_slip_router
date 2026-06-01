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

python3 - "$DONGLE" "$FW" "$SIZE" <<'PY'
import sys, time, serial

dev, fw_path, sz = sys.argv[1], sys.argv[2], int(sys.argv[3])
p = serial.Serial(dev, 115200, timeout=10)
time.sleep(0.3)
p.reset_input_buffer()

# Trigger the OTA receive loop on the dongle.
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
# 3-5s on flash. Give it a generous window before declaring failure.
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

ok, txt = wait_line(b"OTA OK", max_wait=15.0)
if not ok:
    print("flash: no OTA OK. Tail:", txt[-256:].decode('latin-1', 'replace'), file=sys.stderr)
    sys.exit(3)

print("flash: dongle reports OTA OK — rebooting into new firmware now")
p.close()
PY
RC=$?
[ "$RC" -ne 0 ] && exit "$RC"

# Post-reboot: the dongle USB-disappears for ~1-2s then reappears at the same
# path (TinyUSB serial is MAC-based, stable across reboots). Wait for the gap
# (so we don't open the stale tty), then wait for it to reappear, then verify.
echo "flash: waiting for dongle to reboot..."
for i in $(seq 1 30); do
    sleep 0.3
    [ ! -e "$DONGLE" ] && { echo "flash:  -> disappeared (rebooting)"; break; }
done
echo "flash: waiting for new firmware to enumerate..."
for i in $(seq 1 30); do
    sleep 0.3
    [ -e "$DONGLE" ] && { echo "flash:  -> back at $DONGLE"; break; }
done
[ -e "$DONGLE" ] || { echo "flash: dongle didn't reappear — manual replug?" >&2; exit 5; }
sleep 1.0    # let descriptors settle
python3 - "$DONGLE" <<'PY'
import serial, time, sys
for attempt in range(5):
    try:
        p = serial.Serial(sys.argv[1], 115200, timeout=2); break
    except Exception:
        time.sleep(0.5)
else:
    print("flash: couldn't open port after reboot", file=sys.stderr); sys.exit(6)
time.sleep(0.5)
p.reset_input_buffer(); p.write(b"AT\r"); p.flush(); time.sleep(0.5)
r = p.read(64)
print("flash: post-reboot AT ->", repr(r))
p.close()
sys.exit(0 if b"OK" in r else 4)
PY
