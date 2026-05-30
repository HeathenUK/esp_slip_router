#!/bin/bash
# T-Dongle S3 HTTP-download throughput harness.
#
# Runs dos-throughput/httpget.exe in DOSBox over BNU FOSSIL, against
# either a local Python HTTP server (default -- spun up automatically
# from $WORK/big.bin) or a user-supplied URL.
#
# Same caveat as run-throughput.sh: DOSBox's nullmodem serial paces at
# ~1 KB/s regardless of [bps]. On real DOS hardware with CHUSB, the
# same .exe will saturate the WiFi link. Use this harness for end-to-end
# CORRECTNESS validation -- the HTTP handshake, header parsing, body
# length, hang-up, and dongle Hayes restore all happen end-to-end.
#
# Usage:
#   ./run-httpget.sh                    # local: 1MB file via auto Python server
#   ./run-httpget.sh URL                # arbitrary URL -- skips local server
#   ./run-httpget.sh URL SIZE           # ditto, SIZE only affects local-server case

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"

MODE="${1:-local}"
case "$MODE" in
    local)    SIZE="${2:-131072}"; URL="";  ;;        # 128 KB default
    internet) URL="${2:-http://ipv4.download.thinkbroadband.com/1MB.zip}"; SIZE="";  ;;
    http*://*) URL="$MODE"; MODE="internet"; SIZE=""; ;;
    *)        echo "usage: $0 [local SIZE | internet [URL] | URL]" >&2; exit 1 ;;
esac

DOSBOX="${DOSBOX:-/Applications/dosbox.app/Contents/MacOS/DOSBox}"
SOCAT="${SOCAT:-$(command -v socat || true)}"
HTTPGET="${HTTPGET:-$HERE/dos-throughput/httpget.exe}"
BNU="${BNU:-$HOME/Downloads/bnu202/BNU.COM}"

for bin in "$DOSBOX" "$SOCAT" "$HTTPGET" "$BNU"; do
    if [ -z "$bin" ] || [ ! -e "$bin" ]; then
        echo "harness: missing dependency: $bin" >&2; exit 1
    fi
done

DONGLE="$(ls /dev/cu.usbmodemF412FA44AC4C1 2>/dev/null | head -1)"
if [ -z "$DONGLE" ]; then
    DONGLE="$(ls /dev/cu.usbmodem* 2>/dev/null | grep -v '123401\|123301' | head -1)"
fi
[ -z "$DONGLE" ] && { echo "harness: no T-Dongle S3 found on USB." >&2; exit 1; }

WORK="$(mktemp -d -t httpget-harness.XXXXXX)"
cleanup() {
    rm -rf "$WORK"
    pkill -P $$ socat 2>/dev/null || true
    [ -n "${HTTPD_PID:-}" ] && kill "$HTTPD_PID" 2>/dev/null || true
}
if [ -n "${KEEP_WORK:-}" ]; then
    trap 'pkill -P $$ socat 2>/dev/null || true; [ -n "${HTTPD_PID:-}" ] && kill "$HTTPD_PID" 2>/dev/null || true; echo "WORK kept: $WORK"' EXIT
else
    trap cleanup EXIT
fi
cp "$BNU"     "$WORK/BNU.COM"
cp "$HTTPGET" "$WORK/HTTPGET.EXE"

# --- spin up local server if local mode ---
HTTPD_PID=""
if [ "$MODE" = "local" ]; then
    MAC_IP="$(ifconfig | awk '/inet 192\.168\.1\./ {print $2; exit}')"
    [ -z "$MAC_IP" ] && { echo "no Mac IP on 192.168.1.0/24" >&2; exit 1; }
    pkill -f "http.server 8765\|python.*8765" 2>/dev/null || true
    sleep 0.3
    python3 - "$SIZE" 8765 >"$WORK/httpd.log" 2>&1 <<'PY' &
import http.server, sys, os
sz = int(sys.argv[1]); port = int(sys.argv[2])
data = os.urandom(sz)
class H(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200); self.send_header("Content-Length", str(len(data)))
        self.send_header("Connection", "close"); self.end_headers()
        self.wfile.write(data)
    def log_message(self, *a, **k): pass
http.server.ThreadingHTTPServer(("0.0.0.0", port), H).serve_forever()
PY
    HTTPD_PID=$!
    sleep 0.5
    URL="$MAC_IP:8765/X"
    echo "harness: local server up ($SIZE bytes at http://$URL)"
fi

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
HTTPGET $URL > RESULT.TXT
exit
EOF

"$SOCAT" -d TCP-LISTEN:5555,reuseaddr,fork \
         "$DONGLE",raw,echo=0 \
         >"$WORK/socat.log" 2>&1 &
SOCAT_PID=$!
sleep 1

ALARM="${ALARM:-600}"
T0=$(date +%s)
SDL_VIDEODRIVER=dummy perl -e "alarm $ALARM; exec @ARGV" \
    "$DOSBOX" -conf "$WORK/dosbox.conf" \
    >"$WORK/dosbox.log" 2>&1
RC=$?
T1=$(date +%s)
DOSBOX_S=$((T1 - T0))

kill "$SOCAT_PID" 2>/dev/null || true
sleep 1

# Belt-and-braces dongle recovery from host side.
python3 - "$DONGLE" <<'PY' 2>/dev/null || true
import sys, serial, time
p = serial.Serial(sys.argv[1], 115200, timeout=0.3)
p.write(b"+++"); p.flush(); time.sleep(1.2)
p.write(b"ATH\r"); p.flush(); time.sleep(0.3)
p.close()
PY

echo "=== httpget.exe stdout ==="
if [ -f "$WORK/RESULT.TXT" ]; then
    cat "$WORK/RESULT.TXT"
else
    echo "(no RESULT.TXT)"
fi

echo "=== dosbox.log (last 30 lines) ==="
tail -30 "$WORK/dosbox.log" 2>/dev/null || true

echo "=== socat.log ==="
cat "$WORK/socat.log" 2>/dev/null || true

if [ "$RC" -ne 0 ]; then
    echo "=== FAILED (rc=$RC) -- work dir kept: $WORK ===" >&2
    trap - EXIT
fi
exit "$RC"
