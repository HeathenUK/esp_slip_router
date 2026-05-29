# T-Dongle S3 — test harness + USB-CDC OTA

Two related tools in this directory:

* **`flash.sh`** — USB-CDC OTA. Stream firmware.bin straight into the
  inactive OTA partition via `AT$OTASTART`, then the dongle reboots
  into the new build. No bootloader trip, no esptool, no WiFi, no
  buttons. ~10s end-to-end for a 1.1 MB image. **This replaces
  every previous flashing workflow.** See "Flashing" below.

* **`run.sh`** — adversarial / regression test harness. Drives the
  dongle's Hayes-modem firmware via the *actual* `usbterm.exe`
  running inside DOSBox on macOS. Send a script of AT commands, get
  back the bytes the modem replied with. Useful for regression
  testing, reproducing user-reported issues, and demonstrating the
  modem path works without sitting in front of the Pocket386.

## Flashing (USB-CDC OTA)

```
$ ./flash.sh                              # uses ../tdongle-s3/.pio/build/dos/firmware.bin
$ ./flash.sh path/to/firmware.bin         # custom image
$ ./flash.sh --boot [factory.bin]         # escape hatch: AT$BOOT → ROM
                                          # bootloader → esptool (use this if
                                          # the running firmware can't talk
                                          # or you need to update the bootloader
                                          # / partition table itself)
```

The OTA path uses `AT$OTASTART=<bytes>` to stream the new app image
through the existing modem CDC link into the inactive OTA partition,
then `esp_ota_set_boot_partition` + `esp_restart`. Throughput ~110 KB/s,
1.1 MB image in ~10 s, fully autonomous including post-reboot verify.

**One-time prereq:** if the dongle isn't yet running a firmware
containing `AT$OTASTART` (i.e. anything before commit `f120652` on
`tdongle-s3-port`), do one manual download-mode flash first. After
that, `./flash.sh` handles every subsequent update.

`firmware.bin` (app only, ~1.1 MB) is what OTA writes. `firmware.factory.bin`
(bootloader + partitions + boot_app0 + app, ~1.16 MB) is what the
`--boot` escape-hatch writes via esptool. OTA can't replace the
bootloader from the running app.

## Testing (AT harness via DOSBox + usbterm)

```
$ ./run.sh
=== rx (89 bytes from /dev/cu.usbmodemF412FA44AC4C1) ===
AT
OK
ATI
FOSSLIP WiFi Modem

OK
AT$WIFI?
SSID:
status: not connected

OK
```

```
$ ./run.sh my-test.txt    # custom AT script (CR-terminated lines)
```

## What it actually does

```
   AT script (CR-terminated text file)
        │
        ▼
   usbterm.exe -s at.txt -l rx.log com1   (running under headless DOSBox)
        │  reads bytes from at.txt, sends via INT 14h, tees RX to rx.log
        ▼
   DOSBox INT 14h handler (BIOS path — AH=01 send, AH=03 status, AH=02 read)
        │
        ▼
   serial1=nullmodem to 127.0.0.1:5555
        │
        ▼
   socat TCP-LISTEN:5555 ── raw stream ── /dev/cu.usbmodemF412FA44AC4C1
        │
        ▼
   T-Dongle S3 (TinyUSB CDC → modem firmware → AT response)
```

The shell script (`run.sh`) sets up `socat` as a TCP-to-serial bridge, writes
a DOSBox `.conf` that uses `nullmodem` to connect to that bridge, copies
`usbterm.exe` + the AT script into a temp dir mounted as `C:`, runs DOSBox
headlessly (`SDL_VIDEODRIVER=dummy` + `perl alarm` for timeout), and prints
the captured `rx.log`.

## Prerequisites

* **macOS** with DOSBox 0.74-3 installed at `/Applications/dosbox.app`.
  Override with `DOSBOX=/path/to/dosbox`.
* **socat** in `$PATH` (`brew install socat`). Override with
  `SOCAT=/path/to/socat`.
* **`~/CH375/usbterm.exe`** built at HEAD or later (must include the
  `-s` / `-l` script-mode flags and the FOSSIL-fallback path — both
  shipped in commit `23ab543` of CH375). Override with
  `USBTERM=/path/to/usbterm.exe`.
* **T-Dongle S3** plugged in directly to the Mac (not via a marginal hub),
  enumerated as `/dev/cu.usbmodemF412FA44AC4C1`. The dongle's serial number
  is the chip MAC, so the path is stable across reboots for any given
  physical device. Other dongles need the path tweaked in `run.sh`.
* **Dongle firmware** at `tdongle-s3-port` branch tip or later (commit
  `92d8033` of esp_slip_router or newer — this contains the
  `tud_cdc_n_write` bypass that makes non-pyserial hosts work).

## Why all this scaffolding?

The "obvious" approach — `serial1=directserial realport:cu.usbmodemF412…`
— silently fails. The dongle responds, but the bytes never make it back
to DOS, because **two independent problems compound**:

1. **DOSBox `directserial` doesn't propagate DTR/RTS** in a way that
   triggers TinyUSB CDC's `connected` state on the device side. The
   stock `USBCDC::write` gates on `tud_cdc_n_connected(itf)` — so every
   modem reply gets silently dropped at the device. **Fixed firmware-
   side** by routing TX through `tud_cdc_n_write` directly
   (esp_slip_router `92d8033`) and by `Serial.enableReboot(false)` to
   disable a DTR-state-machine that interfered.

2. **DOSBox's BIOS `INT 14h AH=01h` send waits for CTS+DSR**, and the
   dongle's TinyUSB CDC doesn't assert SERIAL_STATE notifications, so
   macOS reports CTS/DSR as 0, and `AH=01h` times out with `AH=80h` for
   every byte. **Worked around** by routing through DOSBox's `nullmodem`
   backend (which provides virtual modem-status lines always asserted)
   plus a `socat` TCP-to-serial bridge to the actual dongle. `nullmodem`
   doesn't enforce CTS the way `directserial` does.

Separately, `usbterm.exe` was written for the Pocket386 + CHUSB target,
which provides full FOSSIL — so its TX path used `AH=0Bh` (FOSSIL
nonblocking send) and RX used `AH=0Ch`/`AH=18h` (FOSSIL peek / batch).
DOSBox implements none of those. Commit `23ab543` of CH375 adds a runtime
fossil-probe and falls back to plain BIOS `AH=01h`/`AH=03h`/`AH=02h` when
FOSSIL is absent — letting the same `usbterm.exe` binary work under
DOSBox-on-Mac *and* on the real Pocket386 with CHUSB.

## Limitations vs the Pocket386 target

This harness **cannot** reproduce CHUSB-specific bugs:

* The Pocket386 runs `CHUSB.EXE` as a TSR that talks to the CH375 USB
  host chip via I/O ports. DOSBox doesn't emulate CH375 hardware, so
  CHUSB can't load there.
* DOSBox + this harness exercises the dongle through DOSBox's BIOS
  emulation → `nullmodem` → `socat` → kernel CDC ACM driver. The
  Pocket386 exercises it through `CHUSB.EXE` → CH375 silicon → USB.
  Different RX/TX queueing, different timing, different concurrency
  surface. A bug in CHUSB's foreground/tick interaction will not
  reproduce here.
* `usbterm`'s FOSSIL path (`AH=0Bh` / `AH=18h`) only runs on the
  Pocket386. The harness exercises the BIOS-fallback path.

What the harness IS good for: confirming the **dongle-side** modem
firmware works (AT echo, OK, AT$ commands, dial flow, telnet pump), and
that `usbterm.exe` itself parses + drives a serial port correctly. Both
are common regression-test surfaces.

## Files

| File | Purpose |
|------|---------|
| `run.sh` | Driver script. Sets up socat + DOSBox + collects rx log. |
| `default-at.txt` | Default AT script: `AT\rATI\rAT$WIFI?\r` (CR-terminated, no LFs). |
| `HARNESS.md` | This file. |

## Writing your own AT script

The dongle's `feed_cmd` (in `tdongle-s3/src/modem.cpp`) commits a line on
`\r` (CR) only; LF is ignored. So your script file must use **CR
terminators**, NOT CR-LF or LF.

```sh
printf 'AT\rATDT example.com:23\r' > my-script.txt
./run.sh my-script.txt
```

Bytes after the last command get sent too — useful for stuffing payload
into a connected TCP session (Hayes online mode). The harness waits 5
seconds of RX silence after EOF before exiting, so make sure your final
expected reply lands inside that window.

## Troubleshooting

* `=== rx (0 bytes …)` — most often: the dongle's TFT shows `WiFi Modem H?`
  (red) rather than `H+`, meaning the host hasn't asserted CDC connect.
  Confirm the dongle is plugged directly into the Mac (not a marginal
  hub), and that `python3 -c "import serial; print(serial.Serial('/dev/cu.usbmodemF412FA44AC4C1', 115200, timeout=1).read(64))"`
  responds to AT.
* `harness: no T-Dongle S3 found on USB` — the device path
  `/dev/cu.usbmodemF412FA44AC4C1` doesn't exist. Either replug (without
  holding BOOT), or update the path in `run.sh` for a different physical
  unit.
* DOSBox prints `BIOS INT14: Unhandled call AH=…` — fine for `AH=04h`
  (the FOSSIL probe, which is expected to fail in DOSBox and trigger
  the fallback) and the one-time `AH=1Eh` init. Anything else means
  `usbterm.exe` is older than commit `23ab543` of CH375 (pre-fallback)
  — rebuild from HEAD.

## Provenance / forensic notes

Both compounding bugs were diagnosed empirically over a long session
(see `MEMORY.md` and the commit messages on `92d8033` /
`23ab543`). The short version: the `usbterm` author assumed FOSSIL, the
arduino-esp32 USBCDC author assumed pyserial-style DTR handling, and
neither held when you stacked DOSBox-on-macOS in front of a TinyUSB CDC
device. The dongle was innocent the whole time.
