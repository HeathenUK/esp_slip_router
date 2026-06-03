# T-Dongle S3 — harness and developer guide

This directory is the host-side toolbox for the dongle: how you flash
it, how you exercise each of its data paths, and how you push or pull
files from it over WiFi. Everything in here is **macOS-side**, talking
to the dongle either over its USB CDC link, the new USB Mass Storage
class device, or its WiFi-side HTTP server.

## Mental model

The dongle is one ESP32-S3 with three USB roles and two runtime
personalities multiplexed onto one CDC byte stream:

```
                                                       ┌── /dev/cu.usbmodem… ── AT engine    (Hayes / MODEM)
                            ┌── USB CDC-ACM ───────────┤
                            │                          └── /dev/cu.usbmodem… ── SLIP framing (SLIP/router)
                            │      (one stream, switched at runtime via AT$MODE
   T-Dongle S3 ── USB ──────┤       or GPIO0 long-press; magic frame 0xC0
                            │       "MODE=MODEM" 0xC0 flips back from SLIP)
                            │
                            ├── USB MSC ─────────────── /dev/disk4 "DOSONGLE" (8 MB FAT12 dev disk)
                            │
                            └── (DFU during AT$BOOT: ROM bootloader for esptool)

                                            WiFi (STA, NAPT'd onto LAN)
                                              │
                                              └── HTTP :80 on dosongle.local
                                                  (GET /list, PUT /fs/X, /status, ...)
```

The two personalities share **the same CDC endpoint**. SLIP mode = lwIP
NAPT, host runs a packet driver. MODEM mode = WiFi232-style Hayes
engine, host runs any terminal. Switching:

* `AT$MODE=SLIP` / `AT$MODE=MODEM` (NVS-persisted)
* Long-press GPIO0
* SLIP → MODEM only: send the magic SLIP frame `0xC0 "MODE=MODEM" 0xC0`
  (used by `MAGICOUT.EXE` on the DOS side and by the harness scripts
  when they need to escape SLIP mid-test)

The MSC and WiFi/HTTP planes are **always-on** and orthogonal to which
personality the CDC plane is in. So you can be `MODE=SLIP` running a
mTCP download and at the same time drop a new binary onto the disk via
WiFi.

## Components

| File / dir | Role |
|------------|------|
| `flash.sh` | USB-CDC OTA (no buttons, no esptool, ~10 s). Default path. |
| `flash-bootloader.sh` | Escape-hatch: trip into ROM bootloader, esptool full-image flash. Use when the running firmware can't talk, or when the bootloader / partition table itself changed. `flash.sh --boot` forwards here. |
| `run.sh` | Adversarial Hayes-modem regression: drives the dongle's AT engine from DOSBox+`usbterm.exe` via socat. |
| `run-throughput.sh` | DOS-side throughput: `THROUGHPUT.EXE` (Hayes path) under DOSBox→BNU→nullmodem→socat→dongle. |
| `run-httpget.sh` | DOS-side HTTP download: `HTTPGET.EXE` (Hayes) under the same DOSBox+BNU stack. |
| `run-mtcp-throughput.sh` | End-to-end DOS-side SLIP via FOSSLIP+mTCP HTGET — the production path. |
| `slip-test.py` | One-shot SLIP ICMP-echo round trip (USB CDC SLIP framing → lwIP → reply). |
| `slip-tcp-test.py` | Synthesised TCP-over-SLIP integrity test: handshake, data, FIN. |
| `slip-throughput.py` | UDP-over-SLIP throughput (host → dongle → WiFi → Mac UDP server). |
| `slip-bench.py` | SLIP intake microbenchmark — bytes/sec the dongle accepts. |
| `datapath-test.py` | Hayes (MODEM mode) online-mode integrity + throughput: host → CDC → TCP → Mac echo. |
| `mtcpget-src/` | Clean-room mTCP-based DOS profiler (`PROFILE.EXE`) and downloader (`MTCPGET.EXE`). Checkpoint dir; real builds happen in `/tmp/mTCP-src_2025-01-10/APPS/*/` (see `mtcpget-src/BUILD.md`). |
| `mtcp-bin/` | Stock mTCP utilities shipped to the CF for ad-hoc DOS testing. |
| `dos-throughput/` | DOS-side throughput tools (THROUGHPUT.EXE, HTTPGET.EXE, MAGICOUT.EXE, ATQ.EXE) and the CF deploy tree. |
| `dosongle.sh` | FAT-over-HTTP toolkit (`ls`, `push`, `pull`, `cat`, `rm`, `swap`, `type`, `type-log`, `ota`, `format`, `reset`). Wraps the owner-swap + macOS-unmount + 8.3-validation + verify dance so you don't hand-roll the curl chain. Sourceable as a library. |

## The four planes

### 1. OTA

HTTP OTA is the preferred path when the dongle is plugged into a DOS
machine rather than the Mac:

```sh
pio run -e dos
curl -X POST --data-binary @../tdongle-s3/.pio/build/dos/firmware.bin \
  http://dosongle.local/ota
```

The endpoint accepts the raw app `firmware.bin` with `Content-Length`,
writes the inactive OTA slot, sets it as boot, replies `OTA OK`, then
reboots. It does not accept `firmware.factory.bin`; OTA cannot replace
the bootloader or partition table.

USB CDC OTA remains available when the dongle is attached locally:

```sh
./flash.sh                              # ../tdongle-s3/.pio/build/dos/firmware.bin
./flash.sh path/to/firmware.bin
./flash.sh --boot [firmware.factory.bin]   # ROM bootloader + esptool
```

The default OTA path streams `firmware.bin` straight into the inactive
OTA partition via `AT$OTASTART=<bytes>`:

```
host:    AT$OTASTART=<bytes>\r
dongle:  \r\nOTA READY\r\n
host:    <bytes raw firmware.bin>
dongle:  \r\nOTA OK\r\n          ← esp_ota_set_boot_partition + esp_restart
```

~187 KB/s, 1.2 MB app in ~7 s, fully autonomous including post-reboot
verify. Implemented in `tdongle-s3/src/modem.cpp` (the `AT$OTASTART`
handler) — see commit history for `f120652` / `fe0d1b0`.

OTA writes the app slot only. The `--boot` fallback flashes
`firmware.factory.bin` (bootloader + partition table + boot_app0 +
app) and is required whenever the partition table itself changes
(e.g. the MSC introduction added a `ffat` partition).

The dongle's CDC tty is matched by glob (`/dev/cu.usbmodemF412FA44AC4C*`) —
adding MSC made it a composite device, which bumps the per-interface
suffix.

### 2. Hayes / MODEM (USB CDC)

`AT$MODE=MODEM` (default at first boot). The dongle terminates TCP and
acts as a WiFi-aware Hayes modem:

```
ATDT example.com:80          → CONNECT
GET / HTTP/1.0\r\n\r\n        → response
+++ (1 s guard)               → returns to command mode
ATH                           → hang up
ATO                           → re-enter online mode
ATNET0 / ATNET1               → bare-TCP / telnet-protocol on the connection
```

Plus `AT$` extensions: `AT$WIFI=ssid,pw`, `AT$WIFI?`, `AT$MODE=…`,
`AT$SCAN`, `AT$STATS`, `AT$RESET`, `AT$BOOT`, `AT$OTASTART`, `AT$DISK?`
(see "MSC" below). Full list: `AT$HELP`.

The Hayes side is exercised three ways from here:

* **`run.sh`** drives it through the *actual* `usbterm.exe` inside
  DOSBox so we exercise the same DOS code path. AT scripts are
  CR-terminated (no LF) — the dongle's `feed_cmd` only commits on CR.

* **`run-throughput.sh`** runs `THROUGHPUT.EXE` (Hayes ATD to a Mac
  echo server) to measure user-mode DOS throughput. DOSBox CPU-paces
  the nullmodem serial, so the numbers aren't the dongle's ceiling —
  they're a regression detector for the Hayes pipeline.

* **`run-httpget.sh`** runs `HTTPGET.EXE` (Hayes ATD to an HTTP server,
  bare `GET / HTTP/1.0`). Same DOSBox-paced caveats.

* **`datapath-test.py`** is the pure-Mac equivalent — exercises Hayes
  online-mode via pyserial, no DOSBox in the chain.

Why DOSBox + nullmodem + socat instead of `directserial`: see
"Provenance" at the end. Short version: `directserial` doesn't
propagate DTR/RTS the way TinyUSB CDC needs, AND `INT 14h AH=01h`
waits for CTS/DSR that the dongle never asserts. `nullmodem` virtual
modem-status lines work around both.

### 3. SLIP (USB CDC)

`AT$MODE=SLIP`. The CDC stream is now RFC1055 SLIP frames; the dongle
strips them into IP packets, lwIP NAPTs onto WiFi.

```
host runs SLIP packet driver  ── CDC SLIP ──>  dongle ── NAPT ── WiFi
                              <── CDC SLIP ──  dongle <─ NAT ─── WiFi
```

DOS-side this lands on FOSSLIP (`~/FOSSLIP`) — a clean-room INT 0x60
packet driver TSR that talks INT 14h/FOSSIL to CHUSB. mTCP binds the
INT 0x60 packet driver like any other. The throughput target is real
TCP-on-DOS over the SLIP link.

Exercise it from the Mac (no DOS) via:

* **`slip-test.py`** — one-shot ICMP echo. Smoke test the framing
  + lwIP path. Hits `192.168.240.1` (the dongle's SLIP-side IP).
* **`slip-tcp-test.py`** — synthesises a full TCP handshake to a Mac
  TCP echo server. Validates NAPT and the dongle's host→Net direction.
* **`slip-throughput.py`** — UDP bytes/sec through the same path.
* **`slip-bench.py`** — how fast the dongle's USB→SLIP decode→lwIP
  intake can accept bytes. Doesn't depend on a peer.

Exercise it end-to-end as DOS would via:

* **`run-mtcp-throughput.sh`** — full DOS production path:
  `mTCP HTGET` → INT 0x60 → `FOSSLIP.EXE` → INT 14h → BNU → COM1 →
  DOSBox `nullmodem` → socat → dongle (SLIP) → WiFi.

After a SLIP test, escape back to MODEM with the magic frame
(`MAGICOUT.EXE` on DOS, or `python3 -c "import serial; serial.Serial('…',
115200).write(b'\\xc0MODE=MODEM\\xc0')"` on the Mac) so the next AT
command lands on the AT engine instead of the SLIP decoder.

### 4. USB MSC + WiFi HTTP (dev disk)

The dongle exposes the new `ffat` partition (8 MB FAT12, 512-byte
logical sectors) as a USB Mass Storage device. macOS / Windows / Linux
mount it natively; CHUSB on the DOS side already implements INT 13h
MSC, so the same drive appears in DOS without any extra driver work.

The same FAT is served on WiFi by an HTTP file server, with mDNS
registered as `dosongle.local`:

```
GET  /                  index (links to the JSON + the file endpoints)
GET  /status            JSON: WiFi, MSC, format state, logical block size
GET  /list              root-directory listing (also works in host-write)
GET  /fs/<NAME>         download a root 8.3 file (also works in host-write)
PUT  /fs/<NAME>         upload a root 8.3 file (device-write only)
DELETE /fs/<NAME>       delete a root 8.3 file (device-write only)
POST /device-write      device owns the block device; MSC medium not ready
POST /host-write        USB MSC owns the block device; host may mount/write it
POST /format            starts explicit FAT format; poll /status
POST /eject             alias for /device-write
POST /present           alias for /host-write
POST /type              body is a typing-DSL string (see dongle_kbd.h);
                        emitted as USB HID keystrokes to whatever host the
                        dongle is plugged into -- this is NOT a CDC write
                        and cannot escape a SLIP-stuck dongle (use /reset)
GET  /type-log          recent parsed HID typing events
POST /ota               HTTP OTA upload of firmware.bin; reboots on success
POST /reset             esp_restart -- also the recovery path for a dongle
                        stuck in SLIP mode (NVS-default personality is MODEM
                        so reboot drops CDC back into the AT engine)
```

**Recommended:** use `./dosongle.sh` for these — it wraps the
owner-swap + macOS-unmount + strict-8.3-validation + post-PUT verify
dance into a small CLI (and is sourceable as a library if you want
to script on top of it):

```sh
./dosongle.sh status                          # pretty-printed /status JSON
./dosongle.sh ls                              # file listing
./dosongle.sh pull PROFILE.LOG profile.log    # GET; no swap needed
./dosongle.sh cat  PROFILE.LOG                # GET to stdout
./dosongle.sh push ./SNAP.EXE                 # full owner-swap dance + verify
./dosongle.sh rm STALE.TXT -y
./dosongle.sh swap host                       # explicit owner-swap
./dosongle.sh type 'PROFILE<ENTER>'           # HID keystrokes -> attached host
./dosongle.sh type-log 20                     # what the HID parser last did
./dosongle.sh reset                           # also fixes a SLIP-stuck dongle
```

The same operations as raw HTTP (what `dosongle.sh` is actually doing
on the wire):

```sh
curl http://dosongle.local/status
curl -X POST http://dosongle.local/device-write
curl http://dosongle.local/list
curl http://dosongle.local/fs/PROFILE.LOG > local-profile.log
curl -X POST http://dosongle.local/host-write
```

**Coordination model.** The FAT partition has one writer at a time.
`host-write` is the normal host-visible state: USB MSC owns the block
device. HTTP `GET /list` and `GET /fs/<NAME>` can read the root
directory and root 8.3 files directly from the raw FAT blocks in this
mode, so the common harness path can fetch `SCREEN.TXT` after DOS has
closed/flushed it without taking write ownership. HTTP `PUT` and
`DELETE` return 409 in `host-write`.

Before switching to `device-write`, unmount/eject the host filesystem
view first (`diskutil unmount /Volumes/DOSONGLE` on macOS, or
finish/eject the drive on DOS). Then switch to `device-write`; the
firmware marks the MSC LUN not-ready so no host can write the FAT while
device-side raw FAT operations are active:

```sh
diskutil unmount /Volumes/DOSONGLE
curl -X POST http://dosongle.local/device-write
curl http://dosongle.local/list
curl -T SNAP.EXE http://dosongle.local/fs/SNAP.EXE
curl -X DELETE http://dosongle.local/fs/STALE.TXT
curl -X POST http://dosongle.local/host-write
diskutil mount disk4    # if macOS does not auto-mount it
```

While `device-write` is active, do not use any stale host mountpoint
that macOS may keep around; it is no longer the authoritative view and
may return cached directory entries or I/O errors. Switch back to
`host-write` after device-side writes so the device marks the MSC
medium ready again. The host then mounts/re-reads the FAT as the
authoritative view.

Do not mutate the FAT over HTTP while Finder, DOS, or any host OS has
the MSC filesystem mounted. macOS and DOS can cache FAT and directory
sectors, and later host writes can overwrite device-side changes. The
safe invariant is: host mounted means host owns writes; device-owned
mode is the only place firmware may mutate the FAT. The raw writer is
root-directory only and accepts strict DOS 8.3 names; it does not create
subdirectories or long filename entries.

Host-owned `GET /fs/<NAME>` is read-only and does not mount FATFS or
modify metadata. It is safe against FAT corruption, but it only sees
what the host has actually committed to the block device. For DOS screen
capture, run `SNAP`, wait for it to return to the prompt, then fetch:

```sh
curl http://dosongle.local/fs/SCREEN.TXT
```

Formatting is explicit only. Use `/format` or `AT$DISK=FORMAT` when
you deliberately want to create or rebuild the FAT volume; there is no
implicit format-on-mount anymore. HTTP `/format` returns immediately
with `202 Accepted`; poll `/status` until `format.running` is false and
`format.last_ok` is true. `AT$DISK=FORMAT` performs the same format from
CDC and returns when it is complete.

```sh
curl -X POST http://dosongle.local/format
curl http://dosongle.local/status
curl -X POST http://dosongle.local/host-write
```

`AT$DISK=DEVICE-WRITE`, `AT$DISK=HOST-WRITE`, and `AT$DISK=FORMAT`
perform the same operations from the CDC modem side. In SLIP mode,
magic frames with payload `DISK=DEVICE` or `DISK=USB` do the
ownership switch without leaving SLIP.

`AT$DISK?` shows status from the CDC side:

```
mdns=dosongle.local  http=up  mode=host-write  msc_present=yes  msc_writable=yes  fs_mounted=no
partition=8306688 bytes
```

If the FAT was never created, use `POST /format` or `AT$DISK=FORMAT`
once. Those commands take device ownership and then create the 512-byte
sector FAT12 volume; that is the only path that formats the partition.

The 8 MB partition came out of the OTA slots (shrunk to 3.94 MB each
from the stock 6.25 MB; still 3.5× headroom over the current 1.2 MB
firmware). Layout in `tdongle-s3/partitions_dongle.csv`. **Changing
this CSV requires a `--boot` flash** — the partition table can't be
OTA'd from the running app.

## Sequencing a typical dev loop

1. Edit firmware in `tdongle-s3/src/`.
2. `pio run -e dos` (builds `firmware.bin` + `firmware.factory.bin`).
3. `./flash.sh` (or `./flash.sh --boot` if you touched the partition
   table or the bootloader, or the dongle isn't responding to AT).
4. Drop new DOS-side artifacts to the dongle's FAT over WiFi:
   `curl -T mtcpget-src/PROFILE.EXE http://dosongle.local/fs/PROFILE.EXE`
5. Run on hardware (or in DOSBox via the run-* scripts).
6. Pull the log back:
   `curl http://dosongle.local/fs/PROFILE.LOG > tmp.log`
7. Repeat without unplugging anything.

## Prerequisites

* **macOS** with DOSBox 0.74-3 at `/Applications/dosbox.app` (override
  with `DOSBOX=…`). Only needed for the `run*.sh` scripts.
* **socat** in `$PATH` (`brew install socat`; override `SOCAT=…`).
* **`~/CH375/usbterm.exe`** built from commit `23ab543` or later
  (override `USBTERM=…`). Only for `run.sh`.
* **PlatformIO** in `$PATH` for `pio run`.
* **T-Dongle S3** enumerated as `/dev/cu.usbmodemF412FA44AC4C*`. The
  serial number is the chip MAC so the path is stable for a given
  unit; scripts glob the suffix so the composite USB descriptor's
  per-interface bump doesn't break them.

## Why all the DOSBox scaffolding (provenance)

The "obvious" approach — `serial1=directserial realport:cu.usbmodemF412…` —
silently fails. Two independent problems compound:

1. **DOSBox `directserial` doesn't propagate DTR/RTS** in a way that
   triggers TinyUSB CDC's `connected` state on the device side. The
   stock `USBCDC::write` gates on `tud_cdc_n_connected(itf)`, so every
   modem reply gets silently dropped at the device. **Fixed firmware-
   side** by routing TX through `tud_cdc_n_write` directly and by
   `Serial.enableReboot(false)`.

2. **DOSBox `INT 14h AH=01h` waits for CTS+DSR**, and TinyUSB CDC
   doesn't assert SERIAL_STATE notifications, so macOS reports them
   as 0 and `AH=01h` times out with `AH=80h` for every byte. **Worked
   around** by routing through DOSBox's `nullmodem` backend (always-
   asserted virtual modem-status lines) plus `socat` bridging to the
   real dongle.

Separately, `usbterm.exe` was written for the Pocket386 + CHUSB
target (full FOSSIL): its TX used `AH=0Bh` (FOSSIL nonblocking) and RX
used `AH=0Ch` / `AH=18h` (peek / batch). DOSBox implements none of
those. Commit `23ab543` of CH375 adds a runtime FOSSIL probe and
falls back to plain `AH=01h`/`AH=03h`/`AH=02h`, so the same binary
runs under DOSBox-on-Mac and on the Pocket386 with CHUSB.

## Limitations vs the Pocket386 target

The DOSBox harness **cannot** reproduce CHUSB-specific bugs: it
exercises the dongle through DOSBox's BIOS emulation → `nullmodem` →
`socat` → kernel CDC ACM. The Pocket386 exercises it through
`CHUSB.EXE` → CH375 silicon → USB. Different RX/TX queueing, different
timing, different concurrency. What the harness IS good for: regression-
testing the dongle's modem firmware (AT echo, OK, AT$ commands, dial
flow, telnet pump) and that `usbterm.exe` itself parses + drives a
serial port correctly.

For real DOS coverage, deploy to the CF (`dos-throughput/deploy-to-cf.sh`)
or push to the dev disk via HTTP (no CF needed) and run on the
Pocket386 directly.
