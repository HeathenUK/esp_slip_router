#!/bin/bash
# T-Dongle S3 FOSSIL + FOSSLIP + mTCP throughput harness.
#
# This is the REAL DOS production path -- the one we expect to deliver
# saturation throughput on hardware:
#
#   mTCP HTGET --INT 0x60--> FOSSLIP.EXE --INT 14h--> BNU --> COM1
#       --nullmodem--> socat --> /dev/cu... --> dongle (SLIP mode)
#       --> lwIP SLIP decode --> NAPT --> WiFi --> Internet
#
# Modes:
#   ./run-mtcp-throughput.sh                  # local mode, default 128 KB
#   ./run-mtcp-throughput.sh local SIZE       # local mode, custom size
#   ./run-mtcp-throughput.sh internet [URL]   # public HTTP URL, default 1 MB
#                                             #   (mTCP HTGET is HTTP-only;
#                                             #    proof.ovh.net is HTTPS-only;
#                                             #    default falls back to a
#                                             #    plain-HTTP thinkbroadband mirror)
#
# Throughput is computed harness-side from DOSBox wall-clock minus a small
# BNU+FOSSLIP setup fudge. DOSBox caps the serial-nullmodem at ~0.3 KB/s
# regardless of bps -- a 1 MB file will NOT complete inside DOSBox in a
# sane wall-clock. The same .exe + driver + config on real DOS hardware
# is bound only by the WiFi link.

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"

MODE="${1:-local}"
case "$MODE" in
    local)    SIZE="${2:-131072}";    URL="";       ;;   # 128 KB default
    internet) URL="${2:-http://ipv4.download.thinkbroadband.com/1MB.zip}"; SIZE="";   ;;
    *)        echo "usage: $0 [local SIZE | internet [URL]]" >&2; exit 1; ;;
esac

DOSBOX="${DOSBOX:-/Applications/dosbox.app/Contents/MacOS/DOSBox}"
SOCAT="${SOCAT:-$(command -v socat || true)}"
BNU="${BNU:-$HOME/Downloads/bnu202/BNU.COM}"
FOSSLIP="${FOSSLIP:-$HOME/FOSSLIP/FOSSLIP.EXE}"
HTGET="${HTGET:-$HERE/mtcp-bin/htget.exe}"

for bin in "$DOSBOX" "$SOCAT" "$BNU" "$FOSSLIP" "$HTGET"; do
    [ -z "$bin" -o ! -e "$bin" ] && { echo "harness: missing dependency: $bin" >&2; exit 1; }
done

DONGLE="$(ls /dev/cu.usbmodemF412FA44AC4C1 2>/dev/null | head -1)"
[ -z "$DONGLE" ] && DONGLE="$(ls /dev/cu.usbmodem* 2>/dev/null | grep -v '123401\|123301' | head -1)"
[ -z "$DONGLE" ] && { echo "harness: no T-Dongle S3 found." >&2; exit 1; }

WORK="$(mktemp -d -t mtcp-harness.XXXXXX)"
HTTPD_PID=""
cleanup() {
    [ -z "${KEEP_WORK:-}" ] && rm -rf "$WORK"
    pkill -P $$ socat 2>/dev/null || true
    [ -n "$HTTPD_PID" ] && kill "$HTTPD_PID" 2>/dev/null || true
    [ -n "${KEEP_WORK:-}" ] && echo "WORK kept: $WORK" >&2
}
trap cleanup EXIT
cp "$BNU"     "$WORK/BNU.COM"
cp "$FOSSLIP" "$WORK/FOSSLIP.EXE"
cp "$HTGET"   "$WORK/HTGET.EXE"

# --- spin up local server if needed ---
if [ "$MODE" = "local" ]; then
    MAC_IP="$(ifconfig | awk '/inet 192\.168\.1\./ {print $2; exit}')"
    [ -z "$MAC_IP" ] && { echo "no Mac IP on 192.168.1.0/24" >&2; exit 1; }
    python3 - "$SIZE" 8765 >"$WORK/httpd.log" 2>&1 <<'PY' &
import http.server, sys, os
sz = int(sys.argv[1]); port = int(sys.argv[2])
data = os.urandom(sz)
class H(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(data)
    def log_message(self, *a, **k): pass
http.server.ThreadingHTTPServer(("0.0.0.0", port), H).serve_forever()
PY
    HTTPD_PID=$!
    sleep 0.5
    URL="http://$MAC_IP:8765/X"
    echo "harness: local server up ($SIZE bytes at $URL)"
fi

# --- mTCP config (matches include/config.h SLIP IPs) ---
cat > "$WORK/MTCP.CFG" <<'EOF'
PACKETINT 0x60
HOSTNAME bolo
IPADDR 192.168.240.2
NETMASK 255.255.255.0
GATEWAY 192.168.240.1
NAMESERVER 1.1.1.1
MTU 1500
EOF

# --- switch dongle to SLIP ---
python3 - "$DONGLE" <<'PY' || { echo "SLIP-switch failed" >&2; exit 3; }
import sys, serial, time
p = serial.Serial(sys.argv[1], 115200, timeout=2); time.sleep(1)
p.reset_input_buffer()
p.write(b"+++"); p.flush(); time.sleep(1.2)
p.write(b"\xC0MODE=MODEM\xC0"); p.flush(); time.sleep(0.4)
p.reset_input_buffer()
p.write(b"AT\rAT$MODE=SLIP\r"); p.flush(); time.sleep(0.7)
r = p.read(4096).decode("latin-1","replace")
sys.exit(0 if "SLIP" in r else 2)
PY
echo "harness: dongle in SLIP"

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
SET MTCPCFG=C:\MTCP.CFG
BNU > L1.TXT
FOSSLIP.EXE 0x60 1 115200 > L2.TXT
echo M1 > M1.TXT
HTGET.EXE -o GOT.BIN $URL > L3.TXT
echo M2 > M2.TXT
exit
EOF

"$SOCAT" -d TCP-LISTEN:5555,reuseaddr,fork "$DONGLE",raw,echo=0 >"$WORK/socat.log" 2>&1 &
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

# Magic-frame restore to MODEM
python3 - "$DONGLE" <<'PY' 2>/dev/null || true
import sys, serial, time
p = serial.Serial(sys.argv[1], 115200, timeout=0.5)
p.write(b"\xC0MODE=MODEM\xC0"); p.flush(); time.sleep(0.5)
p.close()
PY

# Subtract a generous fudge for BNU+FOSSLIP install + mTCP TCP setup.
# Real HTGET wall-clock is approximately (DOSBOX_S - FUDGE).
FUDGE=10
HTGET_S=$((DOSBOX_S - FUDGE))
[ $HTGET_S -lt 1 ] && HTGET_S=1

echo "=== stage logs ==="
for f in L1 L2 M1 L3 M2; do
    if [ -f "$WORK/$f.TXT" ]; then
        echo "[$f]"; cat "$WORK/$f.TXT"
    else
        echo "[$f] (missing)"
    fi
done

echo "=== result ==="
if [ -f "$WORK/GOT.BIN" ]; then
    GOT=$(stat -f%z "$WORK/GOT.BIN")
    KBPS=$(awk "BEGIN { printf \"%.2f\", $GOT / $HTGET_S / 1024 }")
    echo "GOT.BIN:        $GOT bytes"
    echo "elapsed:        ${DOSBOX_S}s (DOSBox), ~${HTGET_S}s HTGET (minus ${FUDGE}s setup)"
    echo "throughput:     ${KBPS} KB/s"
    if [ "$MODE" = "local" ] && [ "$GOT" -eq "$SIZE" ]; then
        echo "completeness:   OK (got all $SIZE bytes)"
    elif [ "$MODE" = "local" ]; then
        echo "completeness:   PARTIAL ($GOT/$SIZE bytes -- alarm hit)"
    else
        REMOTE_MD5=$(curl -sL --max-time 30 "$URL" 2>/dev/null | md5 -q)
        LOCAL_MD5=$(md5 -q "$WORK/GOT.BIN")
        [ "$LOCAL_MD5" = "$REMOTE_MD5" ] && echo "md5:            OK" || echo "md5:            MISMATCH (likely partial)"
    fi
else
    echo "GOT.BIN:        (not created -- check $WORK/HTGET.LOG and $WORK/dosbox.log)"
fi

if [ "$RC" -ne 0 ] && [ "$RC" -ne 142 ]; then
    echo "=== DOSBox exited $RC ===" >&2
fi
exit 0
