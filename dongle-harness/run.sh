#!/bin/bash
# T-Dongle S3 AT-mode test harness.
#
# Drives the dongle's Hayes-modem firmware end-to-end through a real
# DOSBox + usbterm session, with the dongle wired through a socat TCP
# bridge so DOSBox's nullmodem backend can talk to it (directserial's
# CTS handshake is incompatible with USB CDC ACM; see HARNESS.md).
#
# Usage:
#   ./run.sh                        # uses default-at.txt
#   ./run.sh path/to/script.txt     # uses given AT script (CR-terminated lines)
#
# Outputs the captured RX log to stdout and leaves a copy in $WORK/rx.log.

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
SCRIPT="${1:-$HERE/default-at.txt}"

# --- locate the binaries we need --------------------------------------------
DOSBOX="${DOSBOX:-/Applications/dosbox.app/Contents/MacOS/DOSBox}"
SOCAT="${SOCAT:-$(command -v socat || true)}"
USBTERM="${USBTERM:-$HOME/CH375/usbterm.exe}"

for bin in "$DOSBOX" "$SOCAT" "$USBTERM" "$SCRIPT"; do
    if [ -z "$bin" ] || [ ! -e "$bin" ]; then
        echo "harness: missing dependency: $bin" >&2; exit 1
    fi
done

# --- find the dongle (TinyUSB CDC with MAC-based serial F412...) ------------
DONGLE="$(ls /dev/cu.usbmodemF412FA44AC4C1 2>/dev/null | head -1)"
if [ -z "$DONGLE" ]; then
    # Fallback: any usbmodemF* that isn't the ROM-bootloader serial (123401)
    DONGLE="$(ls /dev/cu.usbmodem* 2>/dev/null | grep -v '123401\|123301' | head -1)"
fi
if [ -z "$DONGLE" ]; then
    echo "harness: no T-Dongle S3 found on USB. Replug it (without holding BOOT)." >&2
    exit 1
fi

# --- working dir (fresh files mounted into DOSBox as C:) --------------------
WORK="$(mktemp -d -t dongle-harness.XXXXXX)"
trap 'rm -rf "$WORK"; pkill -P $$ socat 2>/dev/null || true' EXIT
cp "$USBTERM"   "$WORK/USBTERM.EXE"
cp "$SCRIPT"    "$WORK/AT.TXT"

cat > "$WORK/dosbox.conf" <<EOF
[sdl]
fullscreen=false
output=opengl
[serial]
serial1=nullmodem server:127.0.0.1 port:5555 transparent:1
[autoexec]
mount c $WORK
c:
usbterm -s at.txt -l rx.log com1 9600
exit
EOF

# --- start socat: TCP listener <-> dongle tty ------------------------------
"$SOCAT" -d TCP-LISTEN:5555,reuseaddr,fork \
         "$DONGLE",raw,echo=0 \
         >"$WORK/socat.log" 2>&1 &
SOCAT_PID=$!
sleep 1

# --- run DOSBox headless (SDL dummy driver), timeout via perl alarm --------
SDL_VIDEODRIVER=dummy perl -e 'alarm 30; exec @ARGV' \
    "$DOSBOX" -conf "$WORK/dosbox.conf" \
    >"$WORK/dosbox.log" 2>&1
RC=$?

kill "$SOCAT_PID" 2>/dev/null || true

# --- present the captured RX log -------------------------------------------
echo "=== rx ($([ -f "$WORK/RX.LOG" ] && wc -c <"$WORK/RX.LOG" || echo 0) bytes from $DONGLE) ==="
if [ -f "$WORK/RX.LOG" ]; then
    cat "$WORK/RX.LOG"
else
    echo "(no rx.log produced — check $WORK/dosbox.log and $WORK/socat.log)"
fi
if [ "$RC" -ne 0 ]; then
    echo "=== FAILED (rc=$RC) — work dir kept for inspection: $WORK ===" >&2
    trap - EXIT
fi
exit "$RC"
