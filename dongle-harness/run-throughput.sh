#!/bin/bash
# T-Dongle S3 throughput harness — runs dos-throughput/throughput.exe in
# DOSBox over BNU FOSSIL on top of DOSBox's nullmodem serial bridged via
# socat to the dongle's USB CDC. Same skeleton as run.sh.
#
# This measures the DOS-side path you'd see on a real PC:
#   throughput.exe -> FOSSIL (BNU) -> COM1 -> nullmodem -> socat -> /dev/cu...
# The numbers will be DOSBox-CPU-limited, NOT the dongle's ceiling
# (use slip-throughput.py for that). What this proves: that throughput.exe
# actually links + runs against a real FOSSIL TSR, the BNU calls return
# real numbers, and the SLIP escape + restore works end-to-end.
#
# Usage:  ./run-throughput.sh [target-ip] [seconds]
#         defaults: 192.168.240.99 (sentinel) 5

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
TARGET="${1:-192.168.240.99}"
SECS="${2:-5}"

DOSBOX="${DOSBOX:-/Applications/dosbox.app/Contents/MacOS/DOSBox}"
SOCAT="${SOCAT:-$(command -v socat || true)}"
THRPUT="${THRPUT:-$HERE/dos-throughput/throughput.exe}"
BNU="${BNU:-$HOME/Downloads/bnu202/BNU.COM}"

for bin in "$DOSBOX" "$SOCAT" "$THRPUT" "$BNU"; do
    if [ -z "$bin" ] || [ ! -e "$bin" ]; then
        echo "harness: missing dependency: $bin" >&2; exit 1
    fi
done

DONGLE="$(ls /dev/cu.usbmodemF412FA44AC4C1 2>/dev/null | head -1)"
if [ -z "$DONGLE" ]; then
    DONGLE="$(ls /dev/cu.usbmodem* 2>/dev/null | grep -v '123401\|123301' | head -1)"
fi
[ -z "$DONGLE" ] && { echo "harness: no T-Dongle S3 found on USB." >&2; exit 1; }

WORK="$(mktemp -d -t throughput-harness.XXXXXX)"
trap 'rm -rf "$WORK"; pkill -P $$ socat 2>/dev/null || true' EXIT
cp "$BNU"    "$WORK/BNU.COM"
cp "$THRPUT" "$WORK/THRPUT.EXE"

cat > "$WORK/dosbox.conf" <<EOF
[sdl]
fullscreen=false
output=texture
[cpu]
core=dynamic
cputype=pentium_slow
cycles=500000
[serial]
serial1=nullmodem server:127.0.0.1 port:5555 transparent:1 rxdelay:0 txdelay:0
[autoexec]
mount c $WORK
c:
BNU
THRPUT $TARGET $SECS > RESULT.TXT
exit
EOF

"$SOCAT" -d TCP-LISTEN:5555,reuseaddr,fork \
         "$DONGLE",raw,echo=0 \
         >"$WORK/socat.log" 2>&1 &
SOCAT_PID=$!
sleep 1

SDL_VIDEODRIVER=dummy perl -e 'alarm 60; exec @ARGV' \
    "$DOSBOX" -conf "$WORK/dosbox.conf" \
    >"$WORK/dosbox.log" 2>&1
RC=$?

kill "$SOCAT_PID" 2>/dev/null || true
sleep 1

# Belt-and-braces: the in-program escape frame sometimes doesn't reach the
# dongle (BNU buffer state at DOSBox shutdown is fragile). Re-send from the
# host so the dongle is reliably in MODEM mode when we return.
python3 - "$DONGLE" <<'PY' 2>/dev/null || true
import sys, serial, time
p = serial.Serial(sys.argv[1], 115200, timeout=0.3)
p.write(b"\xC0MODE=MODEM\xC0"); p.flush(); time.sleep(0.5)
p.close()
PY

echo "=== throughput.exe stdout ==="
if [ -f "$WORK/RESULT.TXT" ]; then
    cat "$WORK/RESULT.TXT"
else
    echo "(no RESULT.TXT — check $WORK/dosbox.log and $WORK/socat.log)"
fi

if [ "$RC" -ne 0 ]; then
    echo "=== FAILED (rc=$RC) — work dir kept: $WORK ===" >&2
    trap - EXIT
fi
exit "$RC"
