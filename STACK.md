# The dongle stack — ground truth across three repos

This document is the canonical view of the meta-project: **putting a
DOS machine on the internet via a USB dongle**, where the dongle is a
LilyGO T-Dongle S3 talking SLIP-over-USB-CDC to a Pocket386, and a
clean DOS-side packet-driver + USB-host TSR stack carries IP up to mTCP.

Three repositories cooperate. None of them is fully meaningful in
isolation. This file documents how they fit together, who owns what,
and where the seams are. Per-repo detail lives in each repo's own
`README` / `CLAUDE.md` / `PLAN.md`; this file does not duplicate that
detail, it just stitches them.

```
mTCP (DOS application — believes it is on Ethernet)
   │  INT 0x60  (packet-driver API, class-1 Ethernet)
   ▼
FOSSLIP.EXE                                    ── ~/FOSSLIP
   class-1 driver, fake MAC 02:53:4C:49:50:01
   answers ARP locally, strips eth header on TX
   RFC1055-SLIP-encodes IP onto a FOSSIL serial port
   INT 1Ch poll drains RX, fabricates eth hdr, upcalls
   │  INT 14h / FOSSIL  (AH=0B nb-tx, AH=18 block-rx, AH=0C peek)
   ▼
CHUSB.EXE                                      ── ~/CH375
   real-mode TSR over CH375B USB host on Pocket386 ISA
   exposes USB-CDC as virtual COM (INT 14h FOSSIL),
   USB-MSC as INT 13h fixed disk,
   USB-HID keyboards into INT 16h BIOS key buffer
   │  USB bulk OUT / IN  (CDC-ACM, MSC, HID — composite device)
   ▼
T-Dongle S3 firmware                           ── esp_slip_router/tdongle-s3
   one ESP32-S3, three USB roles, two CDC personalities:
     • CDC = MODEM (Hayes/AT)  or  SLIP (lwIP NAPT) — switchable
     • MSC = "DOSONGLE" FAT12 partition on internal flash
     • HID = keyboard typed by AT$TYPE / POST /type
   plus WiFi STA with HTTP+mDNS file server on dosongle.local
   │  WiFi STA (NAPT'd onto the LAN)
   ▼
internet
```

## The three repos

| Repo | Layer | Owns | Does NOT own |
|------|-------|------|--------------|
| `~/CH375` (CHUSB) | DOS-side USB host | CH375B silicon access; USB enumeration (incl. hubs + composite); INT 14h FOSSIL provider; INT 13h MSC provider; INT 16h HID keyboard injection; `usbterm.exe` | Networking. SLIP. mTCP. Anything above the byte stream. |
| `~/FOSSLIP` | DOS-side network | Packet-driver INT 0x60 API; class-1 Ethernet emulation + fake MAC; ARP responder; RFC1055 SLIP codec; INT 1Ch RX poll | The serial provider (uses CHUSB's FOSSIL unmodified). The far end of the link. |
| `esp_slip_router/tdongle-s3` | the dongle | USB-CDC↔SLIP framing; lwIP NAPT to WiFi; Hayes/AT engine; USB MSC + WiFi HTTP for the FAT partition; HID keyboard out; OTA over USB-CDC and HTTP | Anything DOS-side. |
| `esp_slip_router/dongle-harness` | macOS-side harness | DOSBox-based regression tests; Mac smoke tests (pyserial); flashing scripts; the DOS-side test programs (`dos-throughput/`, `mtcpget-src/`) | Production runtime. |

CHUSB's CLAUDE.md, FOSSLIP's PLAN.md/CONTEXT.md, the tdongle README,
and `dongle-harness/HARNESS.md` are the next-level-down references for
each layer. Read this file first to know which one you need.

## The four planes through the dongle

The dongle is the convergence point — every data path the meta-project
cares about either runs through it or out of it.

1. **CDC SLIP** (`AT$MODE=SLIP`). The RFC1055 stream FOSSLIP talks.
   lwIP NAPT on `192.168.240.1/24` masquerades onto WiFi. The
   production data path.
2. **CDC MODEM** (`AT$MODE=MODEM`). WiFi232-style Hayes engine on the
   same CDC byte stream. `ATDT host:port` opens a TCP socket; the
   dongle terminates TCP. No packet driver needed on the host — any
   terminal program works. Used by Phase 2 of PROFILE.EXE.
3. **USB MSC** (`DOSONGLE`, 8 MB FAT12 on internal flash). Always-on
   beside CDC. Mac/Win/Linux mount it natively; CHUSB's INT 13h MSC
   makes it appear as a DOS fixed-disk letter without extra drivers.
   The dev-loop closure: push a new `PROFILE.EXE` over WiFi, run it
   on DOS, pull `PROFILE.LOG` back over WiFi, no unplug.
4. **WiFi HTTP** (`dosongle.local:80`). The control plane.
   `GET /list`, `GET|PUT|DELETE /fs/<NAME>`, `POST /host-write` |
   `/device-write` | `/format` | `/ota` | `/type` | `/reset`,
   `GET /status`, `GET /type-log`. PUT and DELETE go through the raw
   FAT writer (no FATFS mount on the device side) and require
   `device-write` ownership; GET endpoints work in either mode.

CDC personalities are mutually exclusive (one byte stream); MSC and
HTTP are orthogonal to them. You can be in SLIP mode running an mTCP
download and at the same time drop a new binary onto the disk via
WiFi.

## PROFILE.EXE — the integration artefact

`dongle-harness/mtcpget-src/PROFILE.CPP`. Single DOS binary, built
against mTCP's TCPLIB `.obj` as a library (no mTCP source patches).
Replaces the older `dos-throughput/` multi-binary BAT orchestration.
Runs five phases in sequence; writes one `PROFILE.LOG` (committed via
INT 21h AH=68h every line so a crash doesn't lose the log):

| Phase | What it exercises | Stack layers touched |
|-------|-------------------|----------------------|
| 0 | `AT`, `AT$WIFI?`, `AT$NETIF`, `AT$STATS` | CHUSB FOSSIL + dongle MODEM |
| 1 | Raw FOSSIL SLIP UDP to sentinel `192.168.240.99` | CHUSB FOSSIL + dongle SLIP-decode + lwIP forward (drops at ARP — isolates the bottom path from WiFi) |
| 2 | Hayes ATDT HTTP/1.0 download | CHUSB FOSSIL + dongle MODEM (terminates TCP) |
| 3 | mTCP HTTP download, LAN target | CHUSB FOSSIL + FOSSLIP + dongle SLIP + lwIP NAPT (no internet) |
| 4 | same, internet target | full chain incl. WiFi NAPT to upstream |

PROFILE manages mode transitions itself (`AT$MODE=SLIP`, the magic
SLIP escape frame `0xC0 "MODE=MODEM" 0xC0` on the way out) and
loads/unloads FOSSLIP between the SLIP phases. **If PROFILE green,
the stack is green.**

The older standalone equivalents still ship under
`dongle-harness/dos-throughput/` (ATQ, MAGICOUT, THROUGHPUT, HTTPGET,
SNAP, TIMER) — useful when you want to exercise one piece without
running the whole sweep.

## The seams — and the scars at each one

This is the bit not obvious from any single repo.

### CHUSB ↔ FOSSLIP: INT 14h / FOSSIL

FOSSLIP uses **only standard FOSSIL calls** — `AH=0B` nb-tx,
`AH=0C` peek, `AH=02` read, and (since 2026-05-30) `AH=18` block
read. It does **not** modify CHUSB. The "block read" path is a
correctness/throughput compromise documented in FOSSLIP's `PLAN.md`
and the recent FOSSLIP commit `9a9a565`. INT 14h is reentered from
INT 1Ch — FOSSLIP's `in_driver` guard is what makes that safe.

The newest FOSSLIP commit (`ba45ff8`) drains RX inside `send_pkt` to
defeat 18.2 Hz INT 1Ch tick latency. The next perf step is a CHUSB-
side `AH=19` block-send extension; until that exists, TX is per-byte
INT 14h.

### FOSSLIP ↔ tdongle CDC: RFC1055 SLIP

Plain SLIP. No CSLIP, no compression. MTU 1500 both sides. The
dongle's escape hatch out of SLIP mode is the magic frame
`0xC0 "MODE=MODEM" 0xC0` (also `DISK=USB` / `DISK=DEVICE` for
ownership switching without leaving SLIP). FOSSLIP doesn't know about
that frame; PROFILE and the harness scripts emit it directly.

**Important non-obvious:** the magic SLIP-escape frame can only be
sent over **CDC**, not over WiFi-HTTP. The HTTP `POST /type` endpoint
is the USB **HID-keyboard** path — its body is parsed as a typing
DSL and emitted as keystrokes to whatever host the dongle is plugged
into. POSTing the magic byte sequence to `/type` would just type
control characters at the host, not escape SLIP. If something leaves
the dongle stuck in SLIP and you can't reach DOS to send the frame,
the recovery is **`POST /reset`** (esp_restart) — for instance via
`./dosongle.sh reset`. NVS-saved personality defaults to MODEM, so a
reset puts the CDC plane back into the AT engine. The same applies
to `AT$RESET` from CDC if AT is reachable.

### CHUSB ↔ tdongle USB: TinyUSB CDC composite

This is where the firmware has the most scars, all encoded in the
tdongle source as comments that name the symptom:

* **TX `connected` gate** (firmware bypassed in `cdc_write_raw` /
  `cdc_write`). `arduino-esp32`'s `USBCDC::write` gates on
  `tud_cdc_n_connected(itf)`, which needs `SET_CONTROL_LINE_STATE`
  with DTR=1 from the host. pyserial sends it; DOSBox `directserial`
  does not; CHUSB's CDC-ACM driver may or may not depending on path.
  Without the bypass, every modem reply / SLIP byte is silently
  dropped on hosts that don't set DTR. (commits `92d8033`, `96ca7e3`)

* **RX queue detach** (`Serial.end()` in `setup()`, then
  `tud_cdc_n_read` directly). `arduino-esp32`'s USBCDC layer copies
  every received byte through a FreeRTOS queue. Per-byte
  `xQueueSend` / `xQueueReceive` capped SLIP RX at ~50 KB/s.
  Detaching the layer and draining the TinyUSB CDC FIFO directly
  lifts the ceiling to "several hundred KB/s". (commit `eb90c34`)

* **`PBUF_IP` headroom**. `slip_deliver` allocates `PBUF_IP` not
  `PBUF_RAW`. lwIP's NAPT path forwards onto the WiFi netif via
  `etharp_output`, which prepends a 14-byte Ethernet header via
  `pbuf_header(-14)`. That fails silently on `PBUF_RAW` (zero
  headroom). Symptom: SLIP local-delivery (ping the dongle) works
  but anything-via-NAPT vanishes without a trace. (commit `047eefa`)

* **NAPT-on-internal-netif**. The ESP-IDF API is inverted vs. the
  obvious reading: `ip_napt_enable_netif()` flags the *input* netif
  to be source-rewritten on output. So NAPT belongs on the SLIP
  (internal) netif, not the WiFi (WAN) netif. (commit `047eefa`)

Each of these is a *silent* failure mode at the seam — the first
three drop bytes without errors; the fourth corrupts routing without
logs. They are why this stack works at all.

### tdongle CDC ↔ host (Mac harness path)

The Mac-side `dongle-harness/run-*.sh` scripts can't talk to a real
CH375 — they substitute DOSBox + BNU (FOSSIL) + virtual `nullmodem`
+ `socat` ↔ the real CDC tty. Two problems compound, both documented
in `dongle-harness/HARNESS.md` under "Provenance":

1. DOSBox `directserial` doesn't propagate DTR/RTS → fixed by the
   firmware-side `connected`-gate bypass above.
2. DOSBox `INT 14h AH=01h` waits for CTS/DSR. TinyUSB CDC doesn't
   assert SERIAL_STATE notifications so macOS reports them as 0
   → worked around with DOSBox's `nullmodem` backend (always-asserted
   virtual modem-status lines) plus socat bridging.

The harness is a regression detector for the dongle's firmware and
for `usbterm.exe` parsing. It cannot reproduce CHUSB-specific bugs
(different RX/TX queueing, different timing, different concurrency).
**Real DOS coverage requires the Pocket386 + CHUSB.**

## The iteration loop

The harness exists to make the iteration loop fast despite the
geographic split between the Mac (where development happens) and the
Pocket386 (where the only honest test runs).

```
edit firmware in tdongle-s3/src/
  └─► pio run -e dos
        └─► ./dongle-harness/flash.sh     (USB-CDC OTA, ~7s, 1.2 MB)
            └─► quick-loop on Mac:
                  - dongle-harness/run.sh           (Hayes regression)
                  - dongle-harness/slip-*.py        (SLIP unit-ish tests)
                  - dongle-harness/datapath-test.py (Hayes integrity)
              ── OR ── ship to Pocket386:
                  - curl -T PROFILE.EXE http://dosongle.local/fs/PROFILE.EXE
                  - run PROFILE on DOS via CHUSB MSC
                  - curl http://dosongle.local/fs/PROFILE.LOG > log
```

`./flash.sh --boot` is the bootloader-fallback path (esptool over the
ROM bootloader) — needed when the partition table changes or the
running firmware can't talk. HTTP OTA via `POST /ota` is the same
thing from anywhere on the LAN (DOS included), and is what HARNESS.md
now lists as the default OTA path.

**`dongle-harness/dosongle.sh`** is the FAT-over-HTTP toolkit used to
drive the dongle from the Mac side without hand-rolling curl chains:
`ls`, `push`, `pull`, `cat`, `rm`, `swap`, `type`, `type-log`, `ota`,
`format`, `reset`. It handles the owner-swap dance, macOS-side
`/Volumes/DOSONGLE` unmount, strict 8.3 client-side validation, and
post-PUT verification. Sourceable as a library if you want to script
on top of it. The PROFILE build path itself is documented in
`mtcpget-src/BUILD.md` (source lives in-repo; build happens in
`/tmp/mTCP-src_2025-01-10/APPS/PROFILE/`).

## Out-of-scope / hard rules

These are not negotiable across the meta-project:

* **FOSSLIP is MIT, clean-room.** Implemented from
  `crynwr.com/packet_driver.html`, RFC 1055, RFC 826 only. **Do not
  read the GPL EtherSlip / SLIP8250 source.** Every file carries an
  MIT header. See `~/FOSSLIP/CLAUDE.md`.
* **FOSSLIP does not modify CHUSB's FOSSIL provider.** Throughput
  features that need a new CHUSB extension (e.g. `AH=19` block send)
  are out of scope for FOSSLIP and require owner approval on the
  CHUSB side.
* **The tdongle's USB CDC byte stream stays binary-clean.** Debug
  output goes to `Serial0` (the hardware UART) — never to the USB
  CDC. SLIP framing assumes the channel.
* **CHUSB's CLAUDE.md "Mandatory rules" apply** when touching the
  DOS USB host: never guess byte offsets for DOS/BIOS/hardware
  structures; verify against an authoritative reference. The
  `CHUSB.LOG` on the CF is the primary diagnostic — read it before
  hypothesising about TSR failures.
* **The MSC FAT has one writer at a time.** USB-MSC owns it
  (`host-write`) by default; the device owns it (`device-write`) only
  for explicit operations. Both raw-FAT writes (HTTP `PUT` / `DELETE`)
  and formatting require `device-write`. Mixing readers/writers
  corrupts the FAT silently.

## Where to look for what

| Question | First place to look |
|----------|---------------------|
| "How do I flash the dongle?" | `dongle-harness/HARNESS.md` § OTA |
| "How does the FOSSLIP driver work?" | `~/FOSSLIP/PLAN.md` + `~/FOSSLIP/CONTEXT.md` |
| "How does CHUSB enumerate devices?" | `~/CH375/CLAUDE.md` + the dated `*.md` planning docs |
| "What AT commands does the dongle accept?" | `tdongle-s3/src/modem.cpp` (search for `AT$HELP` text) or run it |
| "How do I measure throughput?" | `dongle-harness/mtcpget-src/PROFILE.CPP` (current); `dos-throughput/THROUGHPUT.EXE` (Phase 1 standalone) |
| "Why is my SLIP traffic vanishing?" | The "seams — and the scars" section above + memory files |
| "How do I get a binary onto the Pocket386?" | `POST` over WiFi to the dongle's `/fs`, or `dos-throughput/deploy-to-cf.sh` for the CF route |
| "What does the dongle expose on WiFi?" | `tdongle-s3/src/dongle_disk.cpp` (URI table near the bottom) + `dongle-harness/HARNESS.md` § USB MSC + WiFi HTTP |
| "How do I push / pull / type-via-HID without curl?" | `dongle-harness/dosongle.sh help` |
| "How do I HTTP-OTA the firmware?" | `dongle-harness/dosongle.sh ota tdongle-s3/.pio/build/dos/firmware.bin` |
| "Dongle is stuck in SLIP / unresponsive on AT" | `dongle-harness/dosongle.sh reset` (esp_restart → NVS default MODEM) |

## Update protocol

When the seams move, this file moves with them. Specifically: any
change to the FOSSIL contract, the SLIP-escape sentinels, the raw-FAT
writer's mode gates, the AT$ command surface, or the HTTP API
surface, lands a paragraph in here in the same commit. Per-repo docs
stay the detailed reference; this file is the index.
