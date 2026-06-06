#!/bin/bash
# T-Dongle S3 WGET end-to-end harness (HTTPS / redirects).
#
# Runs dos-throughput/wget.exe in DOSBox over BNU FOSSIL against a URL,
# through socat -> the real dongle CDC. Same caveat as run-httpget.sh:
# DOSBox's nullmodem paces at ~1 KB/s, so this is for CORRECTNESS only --
# but the TLS handshake, ATDS secure dial, status/header parse, body, the
# NO CARRIER strip, redirect-follow, and the Hayes restore all happen
# end-to-end on the dongle. Defaults to an HTTPS URL so it exercises the
# Phase-2 TLS path; pass any http(s) URL to test redirects etc.
#
# Usage:
#   ./run-wget.sh                         # https://example.com/  (TLS path)
#   ./run-wget.sh https://host/path       # arbitrary HTTPS URL
#   ./run-wget.sh http://host/that/302s   # exercise redirect-follow (http->https)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"

URL="${1:-https://example.com/}"

DOSBOX="${DOSBOX:-/Applications/dosbox.app/Contents/MacOS/DOSBox}"
SOCAT="${SOCAT:-$(command -v socat || true)}"
WGET="${WGET:-$HERE/dos-throughput/wget.exe}"
BNU="${BNU:-$HOME/Downloads/bnu202/BNU.COM}"

for bin in "$DOSBOX" "$SOCAT" "$WGET" "$BNU"; do
    if [ -z "$bin" ] || [ ! -e "$bin" ]; then
        echo "harness: missing dependency: $bin" >&2; exit 1
    fi
done

DONGLE="$(ls /dev/cu.usbmodemF412FA44AC4C1 2>/dev/null | head -1)"
if [ -z "$DONGLE" ]; then
    DONGLE="$(ls /dev/cu.usbmodem* 2>/dev/null | grep -v '123401\|123301' | head -1)"
fi
[ -z "$DONGLE" ] && { echo "harness: no T-Dongle S3 found on USB." >&2; exit 1; }

WORK="$(mktemp -d -t wget-harness.XXXXXX)"
cleanup() { rm -rf "$WORK"; pkill -P $$ socat 2>/dev/null || true; }
if [ -n "${KEEP_WORK:-}" ]; then
    trap 'pkill -P $$ socat 2>/dev/null || true; echo "WORK kept: $WORK"' EXIT
else
    trap cleanup EXIT
fi
cp "$BNU"  "$WORK/BNU.COM"
cp "$WGET" "$WORK/WGET.EXE"

cat > "$WORK/dosbox.conf" <<EOF
[sdl]
fullscreen=false
output=opengl
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
WGET $URL OUT.DAT > RESULT.TXT
exit
EOF

"$SOCAT" -d TCP-LISTEN:5555,reuseaddr,fork \
         "$DONGLE",raw,echo=0 \
         >"$WORK/socat.log" 2>&1 &
SOCAT_PID=$!
sleep 1

ALARM="${ALARM:-600}"
SDL_VIDEODRIVER=dummy perl -e "alarm $ALARM; exec @ARGV" \
    "$DOSBOX" -conf "$WORK/dosbox.conf" \
    >"$WORK/dosbox.log" 2>&1
RC=$?

kill "$SOCAT_PID" 2>/dev/null || true
sleep 1

# Belt-and-braces dongle recovery from host side (>1s silence before +++).
python3 - "$DONGLE" <<'PY' 2>/dev/null || true
import sys, serial, time
p = serial.Serial(sys.argv[1], 115200, timeout=0.3)
time.sleep(1.2); p.write(b"+++"); p.flush(); time.sleep(1.2)
p.write(b"ATH\r"); p.flush(); time.sleep(0.3)
p.close()
PY

echo "=== wget.exe stdout (RESULT.TXT) ==="
[ -f "$WORK/RESULT.TXT" ] && cat "$WORK/RESULT.TXT" || echo "(no RESULT.TXT)"
echo "=== downloaded OUT.DAT ==="
if [ -f "$WORK/OUT.DAT" ]; then
    echo "size: $(wc -c < "$WORK/OUT.DAT") bytes"
    echo "head:"; head -c 200 "$WORK/OUT.DAT"; echo
    echo "tail:"; tail -c 80 "$WORK/OUT.DAT"; echo
else
    echo "(no OUT.DAT)"
fi
echo "=== dosbox.log (last 20) ==="
tail -20 "$WORK/dosbox.log" 2>/dev/null || true

[ "$RC" -ne 0 ] && { echo "=== FAILED rc=$RC (WORK kept: $WORK) ===" >&2; trap - EXIT; }
exit "$RC"
