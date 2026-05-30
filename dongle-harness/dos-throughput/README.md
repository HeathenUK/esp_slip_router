# THROUGHPUT.EXE — DOS-side SLIP throughput benchmark

A DOS C program that measures end-to-end TX bandwidth on the real
production path:

```
DOS app -> FOSSIL INT 14h -> CHUSB.EXE -> CH375 -> USB -> T-Dongle S3
                                                         -> SLIP decode
                                                         -> lwIP forward
                                                         -> WiFi TX (or drop)
```

This is the missing complement to `slip-throughput.py` (which measures the
same path from a Mac via pyserial). The Mac test proves the *dongle* is
fast; this test proves the *DOS host* can drive the dongle fast enough to
matter.

## What it does

1. Detects FOSSIL on the chosen COM port. Aborts with a clear error if
   FOSSIL is absent — this benchmark deliberately exercises the FOSSIL
   path, not the BIOS fallback.
2. Sends `AT\rAT$MODE=SLIP\r` to the dongle (modem mode) and waits 700ms
   for it to flip into SLIP mode.
3. Pre-builds **one** SLIP-framed IPv4/UDP packet in a static buffer:
   src=192.168.240.2, dst=*configurable*, sport=49152, dport=5611, 1024
   byte pattern payload (0..255 repeating). All IP and UDP checksums
   valid.
4. Captures BIOS tick count (`0040:006C`, 18.2 Hz).
5. Tight loop for the requested duration. Each iteration:
   - Bumps the IP `ident` field by 1 (so the upstream router / Mac
     doesn't dedupe).
   - Patches the IP header checksum incrementally (RFC 1624 ones-
     complement), staying in the SLIP-encoded buffer — no re-encoding
     cost per packet.
   - Sends the pre-encoded SLIP frame via FOSSIL `AH=0Bh` (non-blocking
     send), same call usbterm.exe uses.
6. Captures end ticks, computes seconds and KB/s.
7. Sends the raw escape frame `0xC0 'MODE=MODEM' 0xC0` so the dongle is
   back in modem mode for the next session. No power-cycle needed.

## Prerequisites

- **CHUSB loaded** on the target DOS box, with its FOSSIL providing
  `AH=0Bh` (non-blocking send) on whichever COM port the T-Dongle S3
  enumerated as.
- **Dongle firmware** with `AT$MODE=SLIP` support and WiFi already
  configured. If you haven't set WiFi yet:
  ```
  AT$WIFI=<ssid>,<password>
  ```
  …in a normal modem session (e.g. via `usbterm.exe`). WiFi creds are
  persisted in NVS so this is a one-time step.
- **Open Watcom v2** built/installed in `~/CH375/ow/` (same toolchain
  used to build `usbterm.exe`). `crtmm.lib` must already exist at
  `~/CH375/crtmm.lib`; if not, follow `~/CH375/BUILD-WATCOM.md` to
  rebuild it once.

## Build

```sh
./build.sh
```

Produces `throughput.exe` (~15 KB). The build script:
- runs `bwcl` with `-mm -0 -bt=dos -fo=throughput.obj` (medium model,
  matching usbterm)
- runs `bwlink` against a generated response file that links in
  `crtmm.lib`

The explicit `-fo=throughput.obj` matters — without it `bwcl` writes
`throughput.o` and the linker silently re-uses an older `.obj`. See the
big warning comment in `build.sh`.

The `W1008: cannot open clibm.lib` warning at link time is benign —
`crtmm.lib` already contains every CRT symbol the program references.

## Run

Copy `throughput.exe` to the DOS machine (floppy / CF / serial — your
preferred sneakernet). Then:

```
C:\> throughput.exe
```

That uses BDA auto-detect to pick the CHUSB COM port (same heuristic as
usbterm) and sends to the **sentinel address `192.168.240.99`** for 5
seconds.

### Why the sentinel address by default

`192.168.240.0/24` is the SLIP-side subnet. `.1` is the dongle, `.2` is
the DOS host, and `.99` is nothing. The dongle accepts the SLIP frame,
runs it through lwIP, fails the ARP, and drops it. That isolates the
benchmark to the **dongle-side decode + forward** path — exactly the
piece the user wants to validate without confounding it with WiFi link
quality, Mac UDP receive buffering, or the actual remote host's
responsiveness. If this number is anywhere near 300–400 KB/s, the
production path is fine.

### Sending to a real WiFi target

If you want to validate the whole pipe including WiFi TX, give it an IP
on the dongle's WiFi LAN (e.g. your Mac):

```
C:\> throughput.exe 192.168.1.226
C:\> throughput.exe 192.168.1.226 10        # 10-second run
C:\> throughput.exe com2 192.168.1.226 10   # explicit port
```

On the receive side, run something that drains UDP port 5611, e.g.:

```sh
# on the Mac
nc -ulk 5611 > /dev/null
```

Args are position-insensitive — anything matching `comN` is the port,
anything containing a `.` is an IPv4, anything numeric is the duration in
seconds.

## Expected output

```
THROUGHPUT: T-Dongle S3 SLIP benchmark (DOS / FOSSIL)
            port=COM1  dst=192.168.240.99  duration=5s
            switching dongle to SLIP mode...
            IP packet 1052 B, SLIP-encoded 1062 B
sent: 1872 pkts, 1988064 bytes in 5.04s = 385.2 KB/s
```

(Numbers fabricated; your mileage will depend on CHUSB, CH375 SPI clock,
the PC's ISA bus, and the dongle's USB-CDC ring servicing.)

## What "sent" means

The benchmark counts **bytes that FOSSIL accepted into its TX ring**, not
bytes that the dongle confirmed receiving. There's no ACK in SLIP. If
FOSSIL's TX ring backs up, `fossil_send` spins on `AH=0Bh` until it
accepts, so the throughput number naturally caps at the slowest link in
the chain (almost always the CH375 USB bulk OUT throughput, not the COM
emulation itself).

To cross-check against actual dongle RX rate, run `slip-throughput.py`
from the Mac side concurrently and compare. They should agree to within a
few percent when targeting the sentinel; they may diverge if the WiFi
path is the bottleneck.

## Caveats

- **No millisecond timer.** Duration is measured in BIOS ticks (~55 ms
  granularity). For 5-second runs that's <1.1% timing error; for 1-second
  runs it's >5%. Use 5+ seconds for a stable number.
- **Single-byte FOSSIL sends.** This deliberately matches what
  `usbterm.exe` does — no AH=19h "block send" fanciness — to stay on the
  same code path real apps use. If CHUSB later adds a block-send
  extension, this benchmark will need updating to use it.
- **The dongle is left in SLIP mode if the program crashes** before the
  cleanup `\xC0 MODE=MODEM \xC0` frame goes out. Ctrl-Break before
  completion = needs power-cycle or a manual escape frame via usbterm to
  recover. The program itself never aborts mid-run except on a wedged TX
  ring (in which case it prints whatever partial bytes/sec it achieved
  and then still tries to send the escape frame).
- **Max duration 600 seconds.** Beyond that, the unsigned-long byte
  counter math starts to overflow.

## Files

| File | Purpose |
|------|---------|
| `throughput.c` | Source. Medium-model Open Watcom DOS. |
| `build.sh` | bwcl + bwlink recipe. |
| `throughput.exe` | Compiled DOS executable (~15 KB). |
| `throughput.obj` | Intermediate. Safe to delete. |
| `throughput_mm.rsp` | Generated linker response file. |
| `crtmm.lib` | Copied from ~/CH375 at build time. |
