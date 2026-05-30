# Throughput tests — the full picture

Four tests, three layers of "ceiling vs reality":

```
                                  WHAT IT MEASURES                          HARNESS
                                ────────────────────────────              ─────────────
slip-throughput.py              dongle USB-CDC ceiling                  Mac pyserial
                                (no DOS stack at all)
                                → currently 540 KB/s wire

run-throughput.sh               raw FOSSIL SLIP (UDP flood)             DOSBox + BNU
                                from the DOS side                       + socat → dongle
                                → DOSBox caps at ~1 KB/s

run-mtcp-throughput.sh          full SLIP TCP stack:                    DOSBox + BNU
                                mTCP → FOSSLIP → FOSSIL → SLIP          + FOSSLIP TSR
                                → dongle NAPT → WiFi                    + mTCP HTGET
                                → DOSBox caps at ~0.2 KB/s

run-httpget.sh                  Hayes mode (dongle terminates TCP):     DOSBox + BNU
                                FOSSIL → COM → dongle ATDT → TCP        + custom httpget.c
                                → DOSBox caps at ~1 KB/s
```

## Why so much DOSBox-capped junk

DOSBox 0.74's `nullmodem` serial paces bytes through its emulated 16550
at a fixed ~9600-baud-equivalent regardless of `bps:` (which 0.74's
parser silently drops) or CPU cycles. There's no flag to disable. The
modern dosbox-staging fork has better serial handling but crashes on
macOS with any headless video driver, so we're stuck with 0.74.

**The numbers in DOSBox are only useful for correctness validation.**
The dongle's actual ceiling is the `slip-throughput.py` number (540 KB/s
wire). The DOS-side binaries (`throughput.exe`, `httpget.exe`) and the
TSR path (`FOSSLIP.EXE` + mTCP `htget.exe`) all run the same byte-level
path on real DOS hardware that they do under DOSBox — they just don't
get throttled.

## Running them

```bash
# 1. Dongle ceiling -- Mac side, no DOS involved
./slip-throughput.py 5                  # 5 seconds, ~500 KB/s

# 2. Raw FOSSIL SLIP from DOS (UDP)
./run-throughput.sh 192.168.240.99 5    # sentinel IP, 5s

# 3. Full SLIP TCP stack (mTCP via FOSSLIP)
./run-mtcp-throughput.sh local 65536            # local Python server
./run-mtcp-throughput.sh internet               # default 1MB thinkbroadband

# 4. Hayes mode (dongle terminates TCP)
./run-httpget.sh local 65536                    # local Python server
./run-httpget.sh internet                       # default 1MB thinkbroadband
```

For (3) and (4), set `ALARM=600` (or higher) in env if you want DOSBox
to keep going past 600s — useful for full 1MB internet downloads which
would take ~3500s at DOSBox pace.

## On real DOS hardware

Same binaries. Skip DOSBox. Load CHUSB (which provides FOSSIL on top of
its CH375 USB host), then either:

```
REM raw FOSSIL SLIP UDP
C:\THRPUT.EXE 192.168.240.99 5

REM Hayes HTTP download
C:\HTTPGET.EXE http://ipv4.download.thinkbroadband.com/1MB.zip

REM mTCP HTTP via FOSSLIP
SET MTCPCFG=C:\MTCP.CFG
C:\FOSSLIP.EXE 0x60 1 115200
C:\MTCP\HTGET.EXE -o GOT.BIN http://ipv4.download.thinkbroadband.com/1MB.zip
```

Expected real-DOS numbers: somewhere between "host pyserial ceiling"
(540 KB/s) and "WiFi link saturation in the room" (-69 dBm gets you
around 330 KB/s outbound through NAPT in UDP, less in TCP).

## Note on `proof.ovh.net/files/1Mb.dat`

That URL is HTTPS-only (HTTP requests 301 to HTTPS). mTCP HTGET has no
TLS, and httpget.c doesn't either. Defaults fall back to
`http://ipv4.download.thinkbroadband.com/1MB.zip` -- plain HTTP, 1 MB,
no redirects. Override with any other HTTP URL.
