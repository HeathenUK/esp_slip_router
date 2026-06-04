#!/bin/bash
# Robust HTTP OTA + verify for the T-Dongle S3.
#
# WHY THIS EXISTS (lessons paid for in blood, 2026-06-03/04):
#  1. After an OTA reboot the dongle can take ~90 s+ to rejoin WiFi. A short
#     verify window makes a SUCCESSFUL flash look "failed/dark." We poll for up
#     to WINDOW seconds (default 180) before drawing ANY conclusion.
#  2. The only trustworthy success signal is the /status VERSION STRING
#     CHANGING. We do NOT trust the HTTP response body ("OTA OK; rebooting")
#     -- curl leaves a stale body file on a failed connection, which once made
#     a flash to a powered-OFF device look successful.
#  3. Single POST, never a retry loop (a retry loop re-flashes 3x and causes
#     reboot storms). rc=200 and rc=000-after-reboot are BOTH plausibly fine;
#     the version check decides, not rc.
#  4. Abort if the device isn't reachable BEFORE flashing (don't POST to a
#     dead device and invent a recovery).
#
# Usage:  ./ota-http.sh [firmware.bin] [host] [verify_secs]
#   defaults: ../tdongle-s3-idf/build/tdongle_s3.bin  dosongle.local  180
# CDC-OTA alternative (no WiFi): ./flash.sh
set -u
BIN="${1:-../tdongle-s3-idf/build/tdongle_s3.bin}"
HOST="${2:-dosongle.local}"
WINDOW="${3:-180}"

ver() { curl -sf --max-time 4 "http://$HOST/status" 2>/dev/null | grep -oE '"version":"[^"]*"' | sed 's/.*:"//; s/"$//'; }

[ -f "$BIN" ] || { echo "ABORT: no firmware at $BIN"; exit 1; }

# 1. Reachable + capture current version (abort if offline -- never flash blind).
PREV=$(ver)
[ -n "$PREV" ] || { echo "ABORT: $HOST not reachable (device off / not on network) -- NOT flashing."; exit 2; }
echo "pre-flash version: $PREV   firmware: $(du -h "$BIN" | cut -f1)"

# 2. Single POST. Fresh body file (never a stale one). rc is informational only.
BODY=$(mktemp)
rc=$(curl -s --max-time 150 -X POST --data-binary @"$BIN" "http://$HOST/ota" -w "%{http_code}" -o "$BODY" 2>/dev/null)
echo "OTA POST: rc=$rc  body=$(cat "$BODY" 2>/dev/null)"
rm -f "$BODY"

# 3. THE verdict comes from the version changing -- polled over a generous window.
echo "verifying via version change (device may take ~90 s+ to rejoin WiFi; window ${WINDOW}s)..."
deadline=$(( $(date +%s) + WINDOW ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  v=$(ver)
  if [ -n "$v" ] && [ "$v" != "$PREV" ]; then
    echo "FLASHED OK: $PREV -> $v"
    curl -sf --max-time 5 "http://$HOST/status" 2>/dev/null | grep -oE '"reset_reason":"[^"]*"|"free_heap":[0-9]*|"min_free_heap":[0-9]*' | tr '\n' ' '; echo
    exit 0
  fi
  sleep 4
done

# Timed out -- distinguish the cases, and NEVER assume plain "failed".
v=$(ver)
if [ -z "$v" ]; then
  echo "INCONCLUSIVE: still unreachable after ${WINDOW}s. Do NOT assume failed --"
  echo "  the reboot/WiFi-rejoin may just be slow. Re-run 'ver' in a minute, or check power."
  exit 4
else
  echo "DID NOT FLIP: device is back but still on OLD version $v after ${WINDOW}s -- flash did not take."
  exit 3
fi
