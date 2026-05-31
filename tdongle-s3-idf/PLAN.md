# T-Dongle S3 ESP-IDF native port — plan

This is the canonical plan for the `tdongle-s3-idf/` project: a
**rewrite of the LilyGO T-Dongle S3 firmware from arduino-esp32 to
ESP-IDF native** so we can get full control over the parts of the
toolchain the prebuilt arduino-esp32 libs blocked.

The dongle's role in the wider stack is documented in
`../STACK.md` (FOSSLIP / CHUSB / mtcpget integration). This file is
purely about the firmware.

The user has been collaborating on this stack for months; the user
already knows what CHUSB / FOSSLIP / mtcpget / Hayes AT / SLIP /
NAPT / the Pocket386 / the W25Q128JV are. Don't ask. If something's
unclear, search the repos or read the binary, don't fabricate.
**See memory `dont-fabricate-third-party-knowledge`**.

---

## Why this migration exists

The arduino-esp32 build was working on macOS hosts but failed
deterministically on the Pocket386's strict USB host. Root cause
chain:

1. ESP32-S3 SPI flash erase **disables the cache on both cores**
   for ~70 ms per 4 KB sector erase.
2. During cache-disable, **any code in flash-mapped memory page-
   faults**. The USB-OTG ISR was flash-resident in arduino-esp32 →
   ISR didn't fire during erase → device went silent on the USB bus
   for ~70 ms → strict hosts saw a disconnect.
3. Even patching the ISR location couldn't help, because
   arduino-esp32's prebuilt libs were shipped with
   `CONFIG_SPI_FLASH_AUTO_SUSPEND=n`. We can't change that without
   rebuilding the libs.
4. So: go to ESP-IDF native and set every Kconfig we want.

The three structural wins this migration unlocked, in priority
order:

- **`CONFIG_SPI_FLASH_AUTO_SUSPEND=y`** — flash chip suspends erase
  within microseconds of a cache miss. Cuts the cache-disable
  window from ~70 ms → ~1 ms. (Requires the chip be in IDF's
  Winbond-suspend whitelist. The W25Q128JV in this dongle is NOT
  in stock IDF's whitelist; we patch IDF — see memory
  `idf-winbond-suspend-patch`.)
- **IRAM-pinned OTG ISR.** Even with AUTO_SUSPEND, the residual
  ~1 ms cache-disable would page-fault a flash-resident ISR.
  `dcd_int_handler` and `dwc2_int_handler_wrap` are both forced
  into IRAM via patches applied at configure time
  (`tools/apply_iram_patches.sh`). See memory
  `idf-tinyusb-iram-patch`.
- **TinyUSB task pinned to CPU1.** WiFi/lwIP/HTTPD live on CPU0
  by IDF default; without isolation, a burst of WiFi RX or lwIP
  work could preempt SCSI CBW processing past CHUSB's tolerance.
  `CONFIG_TINYUSB_TASK_AFFINITY_CPU1=y`,
  `CONFIG_TINYUSB_TASK_PRIORITY=18` (just below WiFi's 23).

---

## Phase status

### Phase 0 — done (commit `507f168`)

Minimum-viable baseline: NVS, WiFi STA, HTTP server, mDNS as
`dosongle.local`, `/ota` / `/status` / `/reset` / `/disk-log`.

WiFi creds bootstrap from `main/wifi_creds.h` (gitignored,
local-only) and **auto-persist to NVS** on first
`IP_EVENT_STA_GOT_IP` so subsequent builds don't need the header.
See memory `wifi-creds-belong-in-nvs`.

`disk_logf()` writes to a 64-line × 160-char ring buffer,
exposed at `GET /disk-log`. This is the long-haul firmware
diagnostic channel — survives any number of host crashes,
only lost on dongle reboot.

### Phase 0.5 — done (commit `e2cbb98`)

The IRAM patch infrastructure. `tools/apply_iram_patches.sh` runs
at every configure pass (wired from top-level `CMakeLists.txt`
`execute_process`) and patches three things in
managed_components:

- `dwc2_esp32.h`: `esp_intr_alloc` flag gets `| ESP_INTR_FLAG_IRAM`
- `dwc2_esp32.h`: `dwc2_int_handler_wrap` gets `IRAM_ATTR`
- `dcd_dwc2.c`: `dcd_int_handler` gets `IRAM_ATTR`
- `tusb_msc_storage.c`: all `tud_msc_*_cb` get
  `__attribute__((weak))` so disk.c's strong overrides win

Script is idempotent; exits non-zero (fails build) if the
upstream files drift in a way the regex can't find — intentional,
so we never silently lose IRAM placement.

### Phase 1a — done (commit `c962c98`)

Composite TinyUSB device (MSC + HID + CDC) with hand-rolled
descriptors in `main/usb.c`. Stock esp_tinyusb's descriptors
don't include HID; ours do.

USB device init **deferred to `POST /usb-start`** so a panic in
TinyUSB init can't take WiFi/HTTP down with it. This safety
scaffold has earned its retirement now that we trust the init
path; flipping back to USB-on-boot is a one-line change in
`main.c` `app_main()`. **OPEN: do it after async eviction proves
stable in the field.**

The hard-won part of 1a: **the JTAG→OTG PHY mux switch.** When
USB-Serial-JTAG was the boot console, IDF's `usb_new_phy()`
claimed to switch the internal FSLS PHY to USB-OTG but the bits
in `RTC_CNTL_USB_CONF_REG` weren't sticking. Symptom: TinyUSB
reported OK, host saw no device. The working sequence (in
`usb.c` `switch_phy_jtag_to_otg()`):

1. Disable USJ pad + bus clock via
   `usb_serial_jtag_ll_phy_enable_pad(false)` + `_enable_bus_clock(false)`
2. Explicitly route internal PHY to USB Wrap via
   `usb_wrap_ll_phy_enable_external(&USB_WRAP, false)`
3. `periph_module_reset(PERIPH_USB_MODULE)` + `_enable()`
4. 20 ms delay so the host registers the JTAG disconnect before
   OTG asserts its pull-up
5. THEN `tinyusb_driver_install()`

See memory `esp32s3-usb-phy-mux-jtag-to-otg` for the diagnostic
signature (device appears in `ioreg -p IOUSB` with correct
VID/PID but no `kUSBCurrentConfiguration` and no child interface
nodes).

Other 1a quirks worth remembering:

- **`sdkconfig` does NOT regenerate from `sdkconfig.defaults` on
  subsequent builds.** Stale 0-valued options (like
  `CONFIG_TINYUSB_HID_COUNT=0`) persist silently. When you change
  a default, **delete `sdkconfig` before rebuilding**.
- ESP32-S3 stays in Download mode after `idf.py flash`; the
  RTS-toggled "Hard reset" doesn't power-cycle the strap pins on
  the T-Dongle S3. **User must physically unplug + replug** to
  exit DL mode. See memory `esp32-download-mode-needs-power-cycle`.

### Phase 1b — done (commits `54e0a4e` auto-format, `e23d3ee`
custom MSC + cache, `11c795f` task pinning, `870d4f5` instrumentation,
`0238028` 4-slot LRU; 32-slot async-eviction commit pending after
this test)

MSC backend in `main/disk.c`, owning the `ffat` partition through
the wear-levelling layer. Highlights:

- **Hand-built FAT12 superfloppy BPB** (drive_num=0x00, no MBR).
  Ported from arduino-esp32's `dongle_disk.cpp:format_for_device_locked`,
  which proved out on real DOS hosts. FatFs's `f_mkfs` produced
  4096-byte-sector BPB (because that's the WL sector size) which
  conflicted with our 512-byte MSC LBAs; macOS rejected that
  layout as "the disk is not readable."
- **Volume label `DOSONGLE`** in both the BPB `BS_VolLab` and the
  root-directory `VOLUME_ID` entry.
- **First-boot auto-format**: `partition_looks_like_fat()` checks
  for a valid BPB signature at sector 0; if missing, we format.
  Existing user data survives firmware updates.
- **Async write-back cache** (32 slots × 4 KB = 128 KB RAM).
  Worker task `wb_worker_task` pinned to CPU1 at priority 17
  (below TinyUSB's 18 so MSC callbacks preempt). Slots have a
  state machine: `EMPTY → DIRTY → FLUSHING → CLEAN` (or back to
  `DIRTY` if a writer touches a slot mid-flush).
  - SCSI write callback: takes mutex briefly, finds-or-allocates
    a slot, memcpys host data in, marks DIRTY, signals worker.
    Returns in microseconds.
  - Worker: snapshots a DIRTY slot's data under brief mutex,
    releases mutex, does the slow `wl_erase + wl_write` (~67 ms)
    outside the mutex.
  - On eject (`START_STOP_UNIT load_eject=1`): synchronously
    drains all DIRTY slots so data isn't lost.
- **Strong overrides for all `tud_msc_*_cb`** (made possible by
  the weak-link patch in 0.5). Custom inquiry strings,
  capacity, read10, write10, scsi (unsupported -> ILLEGAL_REQUEST),
  start_stop. No code path in the SCSI hot loop calls `disk_logf`
  — instrumentation is counter-based (see below).

### Phase 1b instrumentation — done

Per-callback counters + per-write timing, ~1 ns hot-path cost
(single `uint32_t` increment, no locks/strings). Exposed via:

- `GET /usb-stats` — JSON snapshot of counts, timings, last MSC
  op, cache slot states.
- `GET /disk-log` — ring buffer. Bus-event callbacks
  (`tud_mount/umount/suspend/resume_cb`) `disk_logf` here; those
  fire only on state changes, never the hot path.

Use this when CHUSB or any other host appears to fail: leave the
dongle plugged in (don't power-cycle it) and pull both endpoints
from the Mac at `192.168.1.174` (mDNS: `dosongle.local`).

**Signal interpretation**:
- `umount > 0` during a session = host disconnected us (the
  smoking gun for true USB drop-out)
- `slow_writes > 0` with `umount == 0` = host gave up at the SCSI
  protocol layer (CHUSB's per-CBW timeout), but the USB device
  stayed mounted
- `dirty + flushing > clean + empty` for an extended time = the
  cache is saturating; worker isn't keeping up with the host
- `reset_reason != SW` (specifically `PANIC`, `WDT`, etc.) on
  `GET /status` = firmware crashed

---

## Phase forward plan

### Phase 1c — CDC AT modem (next)

Port `tdongle-s3/src/modem.cpp` (Hayes AT command set) onto our
TinyUSB CDC interface. Wire `tud_cdc_rx_cb` + `tud_cdc_tx_complete_cb`
into the AT parser. The CDC interface is already enumerated and
exposes as `/dev/cu.usbmodemF412FA44AC4C1` on macOS; on DOS the
matching USB-CDC driver is whatever the host stack uses (the user
knows; check `~/FOSSLIP` and `dongle-harness/`).

AT commands of interest (from `modem.cpp`):
- `ATZ`, `ATI`, `ATE0/1` — boilerplate
- `AT$WIFI=<ssid>,<psk>` — re-provision WiFi creds at runtime
- `ATD<host:port>` — open a TCP socket to host:port (the SLIP
  dial-out path)

CDC RX is the dominant cost on the SLIP intake path; the
arduino-esp32 build had a known issue where its USBCDC layer
ran a per-byte `xQueueSend` from an ISR that capped throughput
at ~50 KB/s. See memory `arduino-esp32-usbcdc-rx-queue-is-slow`.
The ESP-IDF native CDC should bypass this; we'll see real
numbers when 1c is up.

### Phase 2 — Raw-FAT HTTP handlers — done

GET `/list`, GET / PUT / DELETE `/fs/<name>` (8.3 names). FATFS-backed
file ops on the same WL handle MSC uses, with stability improvements
over the arduino-esp32 design:

- Reads (/list, GET) don't take MSC ownership -- they pass through
  the custom 512-byte FATFS diskio direct to `wl_read`; DOS can
  write concurrently.
- Writes (PUT, DELETE) take ownership briefly, after pre-flushing
  the cache while MSC is still usable. Cache invalidated on
  release; UNIT_ATTENTION sense data signals DOS to re-read FAT.
- 429 "try again" if DOS wrote in the last 500 ms -- defers HTTP
  writes during host bursts.

The custom 512-byte FATFS diskio (`fat.c`) was needed because IDF's
stock `diskio_wl` exposes wl_sector_size (4096) to FATFS, which then
rejects our 512-byte BPB (matched to MSC's 512-byte LBAs for DOS
compat). Wrapper does pass-through reads + RMW writes at the 4 KB wl
granularity. `CONFIG_FATFS_SECTOR_512=y` to make FATFS accept 512.
GET `/lba` (raw block diagnostic) NOT yet added; trivial if we need
it later.

`/lba` (raw block diagnostic) not yet ported -- trivial when needed.
`AT$DISK=device|usb` from old build also not ported -- the new
ownership model is per-op, no persistent "device-side" state.

### Phase 3 — SLIP framing + lwIP NAPT (after 1c)

Port `tdongle-s3/src/main.cpp`'s SLIP intake/output paths. Use
lwIP's SLIP netif. Enable NAPT on the INTERNAL netif (the SLIP
side), NOT the WAN-side STA. See memory `esp-idf-napt-api-inverted`.
`pbuf_alloc(PBUF_RAW, ...)` silently breaks forwarding to
Ethernet; use `PBUF_IP`. See memory `slip-pbuf-headroom-forwarding`.

### Phase 4 — HID typing DSL

Port `tdongle-s3/src/dongle_kbd.cpp`. The HID interface is
already in the composite descriptor; just need the report
descriptor + typing engine + `POST /type` endpoint.

### Phase 5 — ST7735 LCD (defer indefinitely)

The dongle has a small ST7735 LCD on SPI. Useful for showing
WiFi state + IP. Lowest priority.

---

## Open decisions

1. **Re-enable USB-on-boot?** Phase 1a deferred USB init to
   `POST /usb-start` as a safety scaffold during bring-up. Now
   that we trust the init path (multiple successful OTAs and
   DISKTEST runs), this could go back to auto-start. One-line
   change in `main.c` `app_main()`. Do this after async eviction
   proves stable in field testing — once we've gone a week or two
   without needing to recover via `/usb-start` manually.

2. **Periodic idle-flush timer?** Currently the async worker
   drains dirty slots as fast as it can but only when notified by
   a writer. If the host writes a small amount then stops, the
   dirty data sits in RAM until either (a) the worker's
   already-pending drain completes (it will), or (b) eject. A
   periodic timer (~200 ms) that drains stale dirty slots would
   tighten data-loss exposure under hard power-off. Low priority;
   user's workflow is "copy files, eject, unplug."

3. **`/wifi-set` endpoint or AT$WIFI=** Currently if the wifi_creds
   header is wrong and the dongle can't join, recovery is
   reflashing via Download mode. After Phase 1c, AT$WIFI= on CDC
   gives a host-side recovery path. Could also add `POST /wifi-set`
   over a temporary AP-mode fallback, but that's complex.

---

## File and endpoint cheat-sheet

**Repo paths** (relative to `tdongle-s3-idf/`):

```
main/main.c            -- app_main, HTTP handlers, WiFi
main/usb.c, usb.h      -- composite descriptors, PHY switch, tinyusb_driver_install
main/disk.c, disk.h    -- MSC backend, async worker, FAT12 mkfs, instrumentation
main/wifi_creds.h      -- gitignored bootstrap creds (NVS supersedes on first boot)
sdkconfig.defaults     -- all the Kconfig knobs (AUTO_SUSPEND, task pinning, etc.)
partitions.csv         -- 16 MB layout (OTA slots + 8 MB ffat)
tools/apply_iram_patches.sh  -- idempotent patches to managed_components
CMakeLists.txt         -- top-level; runs apply_iram_patches via execute_process
PLAN.md                -- this file
```

**HTTP endpoints** (`http://dosongle.local/` or `http://192.168.1.174/`):

```
GET  /status     -- JSON: WiFi/heap/version/reset_reason
GET  /disk-log   -- ring buffer (human-readable)
GET  /usb-stats  -- JSON: per-callback counts + timings + cache state
POST /ota        -- body = raw firmware.bin, reboots into new image
POST /reset      -- esp_restart()
POST /usb-start  -- bring up USB device (temporary; will be auto on boot eventually)
```

**Diagnostic recipe when a host fails**:

```bash
# Don't unplug the dongle! Pull these from the Mac:
curl http://dosongle.local/status
curl http://dosongle.local/usb-stats
curl http://dosongle.local/disk-log
```

**Host-side test patterns we use**:

- DISKTEST.EXE on Pocket386 = read/write smoke test from DOS
- 256 KB BIGFILE.BIN copy from Pocket386 = sustained-write stress
- macOS `dd bs=4k count=64` to `/Volumes/DOSONGLE/` = scatter-write
  stress (macOS interleaves FAT/dir/data updates → ~13× slower
  than `bs=256k count=1`; this is normal, not a regression)

---

## Critical reference memory entries

These are the conclusions from past debugging sessions, kept as
discrete memory files in
`~/.claude/projects/-Users-gadyke-esp-slip-router/memory/`:

- `never-blame-hardware` — Pocket386/hub/CH375/cables are trusted
- `dont-fabricate-third-party-knowledge` — say "I don't know"
  about CHUSB/mtcpget/FOSSLIP rather than inventing
- `build-then-verify-before-flash` — never chain build + flash
- `esp32-download-mode-needs-power-cycle` — RTS reset doesn't
  exit Download mode on T-Dongle S3
- `esp32s3-usb-phy-mux-jtag-to-otg` — the JTAG→OTG mux sequence
- `idf-winbond-suspend-patch` — W25Q128JV needs adding to IDF's
  whitelist
- `idf-tinyusb-iram-patch` — managed TinyUSB needs IRAM patches
- `wifi-creds-belong-in-nvs` — bootstrap header + NVS persistence
- `esp-idf-napt-api-inverted` — NAPT on internal, not WAN
- `slip-pbuf-headroom-forwarding` — use PBUF_IP not PBUF_RAW
- `arduino-esp32-usbcdc-rx-queue-is-slow` — for context when
  benchmarking Phase 1c CDC throughput
