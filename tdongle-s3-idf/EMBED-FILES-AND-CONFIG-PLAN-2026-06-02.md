# Plan: embed self-healing DOS tools + a config-file sync surface

**Date:** 2026-06-02
**Status:** designed, not started. Build + OTA on the dongle side.
**Author context:** written to survive context compaction — it is self-contained.
Read it top to bottom and you can execute without the chat history.

---

## 0. Background / why

The dongle exposes a FAT12 "superfloppy" (the `ffat` partition via wear-
levelling) as a USB MSC volume labelled `DOSONGLE`. DOS (via CHUSB on the
Pocket386) and macOS both mount it. Today the useful DOS-side tools
(`FOSSLIP.EXE`, `PROFILE.EXE`, etc.) are just copied onto the volume and
are lost if the user deletes them or reformats.

Goal of this work, two parts:

1. **Embed essential DOS tools in the firmware** so they self-heal: if the
   volume is wiped/formatted/deleted, they reappear. Chosen set:
   `FOSSLIP.EXE`, `HTTPGET.EXE`, a new `MODE.EXE` (mode-toggle tool), and a
   self-healing `DONGLE.CFG`.
2. **A `DONGLE.CFG` config file** the user can edit to set SSID / PSK /
   IP mode / etc., reconciled with NVS at boot.

### Key facts about the current firmware (verified this session)

- App partition `app0`/`app1` = `0x3F0000` (3.94 MB) each; app binary
  ~960 KB → ~3 MB free. Embedding ~110 KB of tools is negligible.
- `disk.c`: owns the WL handle + write-back cache. Has
  `format_partition_superfloppy()` (hand-builds an empty FAT12 — does NOT
  use `f_mkfs`), `disk_init()` (mounts WL, formats only if
  `partition_looks_like_fat()` fails), `disk_format()` (runtime reformat,
  added this session — `POST /format`), and `disk_fatfs_lock(ms)` /
  `disk_fatfs_unlock()` (the shared `s_io_mutex` the MSC read10/write10
  path also holds — diskio takes it per-sector).
- `fat.c`: FATFS glue. `fat_read(name,out_cb,ctx)`, `fat_write(name,
  in_cb,total,ctx)` (callback data source), `fat_list(cb,ctx)`. diskio
  `d_read`/`d_write` take `disk_fatfs_lock` per sector. `fat_mount()` /
  `fat_unmount()` wrap `f_mount`. Files are root 8.3 only.
- WiFi creds live in **NVS namespace `"slip-router"`, keys `"ssid"` /
  `"pass"`** (see `modem.c` `save_wifi_creds`, `cmd_wifi_set`, and the
  `handle_dollar` AT$SSID/AT$PASS/AT$WIFI handlers). A gitignored
  `wifi_creds.h` only seeds NVS on first flash.
- HTTP endpoints already present: `/ota /status /reset /usb-start
  /usb-stats /list /fs/* (GET/PUT/DELETE) /mode /slip-stats /partitions
  /type /eject /mount /format /usb-reconnect /disk-log`.
- `slip.c` magic-frame handling: a SLIP frame `0xC0 "MODE=MODEM" 0xC0`
  switches to MODEM; `"MODE=SLIP"` switches to SLIP. `slip_get_mode()` /
  `slip_set_mode()` / `LinkMode {MODE_MODEM, MODE_SLIP}`.
- Build env: ESP-IDF at `/private/tmp/esp32-arduino-lib-builder/esp-idf`;
  `. $IDF/export.sh` then `idf.py build` from `tdongle-s3-idf/`. Deploy via
  HTTP `curl -sf -X POST --data-binary @build/tdongle_s3.bin http://<ip>/ota`
  (CDC OTA via `dongle-harness/flash.sh` is the alternative). **Never
  Download-mode flash unless unavoidable** (user rule).
- DOS toolchain: Open Watcom at `~/CH375/ow`; mTCP source unpacked at
  `/tmp/mTCP-src_2025-01-10`. Existing DOS tool sources:
  `dongle-harness/dos-throughput/{magicout.c,atq.c,httpget.c,...}` and
  `dongle-harness/mtcpget-src/` (PROFILE/MTCPGET). `MAGICOUT.EXE` sends the
  `0xC0 'MODE=MODEM' 0xC0` frame via FOSSIL INT 14h; `ATQ.EXE` does AT
  queries via INT 14h. These two are the basis for `MODE.EXE`.

### Hard rules / lessons banked this session

- **Never write the FAT device-side while the host has the volume mounted**
  — that dual-access write corrupted the FAT and started a multi-hour
  recovery. Therefore all embedded-file writes and config writes must
  happen **at boot before the host enumerates / mounts**, or right after
  `disk_format()` (host not mounted then). Never from a live `/fs` path
  while mounted.
- Device-side FATFS access must hold `disk_fatfs_lock()` (already true for
  the diskio layer; any new write path must go through `fat_*` which uses
  it).
- macOS keys disk arbitration to the USB serial; once it rejects/ejects a
  volume it won't re-mount without `mount_msdos` / a Mac reboot. Irrelevant
  to this plan but explains why we test FAT changes via `/list` + DOS, not
  the macOS Finder mount.

---

## 1. Embedding mechanism (shared by all embedded files)

Use ESP-IDF `EMBED_FILES`. In `main/CMakeLists.txt` `idf_component_register(...)`:

```cmake
    EMBED_FILES
        "embed/FOSSLIP.EXE"
        "embed/HTTPGET.EXE"
        "embed/MODE.EXE"
        "embed/DONGLE.CFG"   # the default/template config
```

Put the files in `tdongle-s3-idf/main/embed/`. Each yields linker symbols
`_binary_FOSSLIP_EXE_start` / `_binary_FOSSLIP_EXE_end` etc. (dots become
underscores). Access in C:

```c
extern const uint8_t fosslip_start[] asm("_binary_FOSSLIP_EXE_start");
extern const uint8_t fosslip_end[]   asm("_binary_FOSSLIP_EXE_end");
/* size = fosslip_end - fosslip_start */
```

(If the symbol names are awkward, the cleaner idiom is per-file vars in a
small table; see restore code below.)

### Restore-if-missing

New function in `disk.c` (or a small `embed.c`), called:
- at the end of `disk_init()` after the FAT is mounted+verified, and
- at the end of `disk_format()` (after the fresh empty FAT is built).

```c
struct embed_file { const char *name; const uint8_t *start; const uint8_t *end; };
static const struct embed_file s_embeds[] = {
    { "FOSSLIP.EXE", fosslip_start, fosslip_end },
    { "HTTPGET.EXE", httpget_start, httpget_end },
    { "MODE.EXE",    mode_start,    mode_end    },
    /* DONGLE.CFG handled specially -- see §3 */
};

void disk_restore_embedded(void) {
    /* MUST run with no host mounted (boot / post-format). Uses FATFS,
     * which writes the standard FAT dir entry + cluster chain -- no
     * manual FAT table work. */
    if (fat_mount() != ESP_OK) return;
    for (each e in s_embeds) {
        FILINFO fi;
        if (f_stat(e->name, &fi) == FR_NO_FILE) {
            FIL f;
            if (f_open(&f, e->name, FA_WRITE|FA_CREATE_ALWAYS) == FR_OK) {
                UINT bw; f_write(&f, e->start, e->end - e->start, &bw);
                f_close(&f);
                disk_logf("embed: restored %s (%u B)", e->name, (unsigned)(e->end-e->start));
            }
        }
    }
    fat_unmount();
}
```

Implementation note: `fat.c` currently exposes `fat_write` (callback
source). Either add `fat_write_buf(name, ptr, len)` (thin wrapper around
`f_open`/`f_write`/`f_close`) and call that, or do the `f_*` calls inside
`disk_restore_embedded` directly (it's in the same translation-unit family
that already includes `ff.h`). Prefer a `fat_write_buf()` helper in `fat.c`
so all FATFS access stays in one place and inherits the diskio mutex.

**Self-heal semantics:** files reappear on next boot if deleted, and
immediately after `/format`. NOT "undeletable" (a block-level overlay
would be, but it's overkill/fragile — explicitly out of scope).

---

## 2. `MODE.EXE` — mode-toggle tool (NEW DOS binary)

A small DOS program that flips the dongle's link mode and prints the
result. Build with Watcom; base on `dos-throughput/magicout.c` (sends the
magic frame via INT 14h) + `atq.c` (AT query via INT 14h).

### Logic
```
1. Probe: send "AT\r" over FOSSIL (INT 14h AH=01 per byte), then read
   (AH=02/03) for up to ~300 ms looking for "OK".
2. If "OK" seen  -> we are in MODEM mode:
      send "AT$MODE=SLIP\r"; print "DOSONGLE: was MODEM, now SLIP".
3. If no "OK"     -> assume SLIP mode:
      send the magic frame  0xC0 'M''O''D''E''=''M''O''D''E''M' 0xC0;
      re-probe "AT\r" -> expect "OK";
      print "DOSONGLE: was SLIP, now MODEM" (or a warning if AT still silent).
```

### Build
```
cp dos-throughput/magicout.c (as a base) -> new mode.c in dos-throughput/
# add the AT-probe from atq.c
cd dongle-harness/dos-throughput && ./build.sh   # or the wpp/wlink invocation used for magicout/atq
# output MODE.EXE -> copy to tdongle-s3-idf/main/embed/MODE.EXE
```
Confirm against `dongle-harness/dos-throughput/build.sh` for the exact
Watcom flags (small model, INT 14h FOSSIL helpers). 8.3 name: `MODE.EXE`.

Detection caveat: in MODEM mode while *online* (ATDT connected), "AT" goes
to TCP not the parser, so the probe could misread. Acceptable — MODE.EXE
is for the command-mode/SLIP case; document "run from the DOS prompt, not
mid-call."

---

## 3. `DONGLE.CFG` — config file as a boot-time NVS sync surface

### Decision (settled in discussion)
- **NVS stays the source of truth at runtime.** Not a virtual live mirror
  (impossible on FAT — no edit notification, polling fights dual-access),
  and not an NVS replacement (file is deletable/format-wipeable and would
  leave the PSK in cleartext permanently).
- The file is a **boot-time bidirectional sync surface**: read user edits
  in → apply to NVS; regenerate from NVS out → so it shows current state.
- **Format: INI-style `key = value`**, NOT TOML. A flat line parser is
  ~30 lines and `EDIT.COM`-friendly; a real TOML parser is multiple KB of
  code + array/table/quoting edge cases we don't need.
- **PSK: ingest-and-blank.** The file has a `psk =` line; on boot the
  firmware reads it, writes to NVS, then **blanks that line in the file**
  so the password isn't left on the removable disk. NVS keeps it
  (not casually readable), consistent with the "creds belong in NVS" rule.
- **Change detection: content hash stored in NVS.** Distinguishes "user
  edited the file" from "this is the copy I wrote last boot," so we never
  loop or clobber, and a blanked-PSK file isn't read as "user wants no
  password."

### Boot reconcile flow (new `config.c`, called early in `app_main` AFTER
disk_init mounts the FAT but BEFORE wifi_start, and before USB enum):

```
1. fat_mount(); read DONGLE.CFG into a buffer (if absent -> write the
   embedded default template from NVS values, store its hash, done).
2. hash = fnv1a(file_bytes).
3. stored_hash = nvs_get_u32("slip-router","cfghash").
4. if hash != stored_hash  -> USER EDITED:
      parse key=value lines; for each known key, write to NVS:
        ssid -> "ssid"
        psk  -> "pass"   (only if non-blank)
        ip_mode/ip/mask/gw/dns -> new NVS keys (see below)
        hostname -> "mdns" (new) ; lecho/naws/ttype -> telnet defaults
      apply to running config as far as possible (or just rely on reboot).
5. Regenerate DONGLE.CFG from current NVS values, PSK line blanked:
      "psk =            ; (set, hidden -- type a new value to change)"
   write file; recompute hash; nvs_set_u32("cfghash", new_hash); commit.
6. fat_unmount().
```

Because step 5 always rewrites the file, the next boot's hash matches
(no spurious "edited") unless the user actually changed something. The
PSK-blanking means after ingest the file's hash is stable.

### Proposed key set (INI)
```ini
; DONGLE.CFG -- edit, then reset the dongle (AT$RESET or replug) to apply.
; PSK is write-only: type a value to set it; it is absorbed into the
; dongle on next boot and this line is blanked. Current PSK is kept
; internally (not shown here).

ssid     = JELLING
psk      =                 ; set a value to change; blanked after ingest
ip_mode  = dhcp            ; dhcp | static
ip       =                 ; used only when ip_mode = static
mask     =
gw       =
dns      =
hostname = dosongle        ; mDNS name -> <hostname>.local
; --- telnet defaults (data mode) ---
lecho    = auto            ; auto | on | off
ttype    = ANSI
naws     = 80,24
```

New NVS keys needed (namespace `"slip-router"`): `pass` (exists),
`ssid` (exists), plus `ipmode`,`ip`,`mask`,`gw`,`dns`,`mdns`,`lecho`,
`ttype`,`naws`,`cfghash`. WiFi static/DHCP path: today the firmware does
DHCP via `esp_netif` defaults; static support is NEW work — set
`esp_netif_dhcpc_stop` + `esp_netif_set_ip_info` when `ip_mode=static`.
(Scope flag: static-IP support is the biggest *new* firmware surface here;
if it's not needed day one, ship `ip_mode=dhcp` only and stub static.)

### Apply model
- Simple: settings apply on **reboot** (`AT$RESET` or replug). Document it.
- Optional later: an `AT$RELOAD` / `POST /reload` that re-runs the reconcile
  + re-applies WiFi live (reuse the AT$WIFI reconnect path in `modem.c`).

### Safety
- All `DONGLE.CFG` read/write happens at boot (host not mounted) → no
  dual-access. If the user edits via the Mac mount or DOS, they then
  reboot → firmware ingests. Never touch the file from a live `/fs`
  handler while a host is mounted.

---

## 4. Build / deploy / verify

1. Stage `embed/` files: copy current `FOSSLIP.EXE` (`~/FOSSLIP/FOSSLIP.EXE`,
   md5 `e4a1a8ed…`), `HTTPGET.EXE` (`dongle-harness/dos-throughput/` or CF
   `TDONGLE/LEGACY/HTTPGET.EXE`), build `MODE.EXE`, and create the default
   `DONGLE.CFG` template.
2. `EMBED_FILES` in `main/CMakeLists.txt`.
3. Implement `disk_restore_embedded()` + `fat_write_buf()` + call sites
   (`disk_init` tail, `disk_format` tail).
4. Implement `config.c` reconcile + call from `app_main` (after disk_init,
   before wifi_start). Add NVS keys + static-IP path (or stub).
5. Build: `cd tdongle-s3-idf && . /private/tmp/esp32-arduino-lib-builder/esp-idf/export.sh && idf.py build`.
6. Deploy: `curl -sf -X POST --data-binary @build/tdongle_s3.bin http://<dongle-ip>/ota`.
7. **Verify (no host mounted — use HTTP `/list`, not the Mac Finder):**
   - After OTA reboot: `GET /list` shows FOSSLIP.EXE, HTTPGET.EXE,
     MODE.EXE, DONGLE.CFG.
   - `POST /format` then `GET /list` → all four reappear (self-heal).
   - `GET /fs/DONGLE.CFG` → shows current NVS values, PSK blank.
   - Edit DONGLE.CFG (set `psk=`, change `ssid=`), reboot, confirm
     `AT$WIFI?` reflects the change and `GET /fs/DONGLE.CFG` shows PSK
     re-blanked + new ssid.
   - MD5 a `GET /fs/FOSSLIP.EXE` against `~/FOSSLIP/FOSSLIP.EXE`.
8. On the Pocket386: `MODE` toggles + prints; `HTTPGET` works; PROFILE via
   `RUN <drive>` still works (FOSSLIP present).

---

## 5. Scope / sequencing recommendation

Ship in this order so each piece is independently testable:

1. **Embed + restore-on-boot/format for FOSSLIP.EXE + HTTPGET.EXE.**
   Lowest risk, immediate value, reuses proven `fat_write`. (Add a
   `fat_write_buf` helper.)
2. **MODE.EXE** (write + Watcom build + embed). Independent DOS work.
3. **DONGLE.CFG reconcile** — the biggest new surface. Do `ssid/psk/
   hostname/lecho/ttype/naws` + `ip_mode=dhcp` first; add **static IP**
   only if needed (it's the one genuinely new firmware capability).

A generated `README.TXT` (firmware version + AT$ command list + drive-letter
+ recovery notes, self-healing like the others) was discussed as a high-
value cheap add — optional, fold into step 1's embed table if wanted.

## 6. Open decisions to confirm before coding
- Static-IP support needed in v1, or DHCP-only with static stubbed?
- Include the generated `README.TXT`?
- `AT$RELOAD`/`POST /reload` for live apply, or reboot-only for v1?
- Embed `PROFILE.EXE` too (churns) or leave it `/fs`-pushed?
