// dongle_disk: USB MSC + HTTP file server backed by a FAT partition on
// internal flash. See dongle_disk.h for the architecture summary.
//
// Coordination: the FAT partition is owned EITHER by the USB MSC side
// (host sees the drive and can write it) OR by device-side raw FAT
// operations. Ownership is explicit and persistent; never let the host
// and device write the same FAT at once.

#include "dongle_disk.h"

#include <Arduino.h>
#include <WiFi.h>
#include <USB.h>
#include <USBMSC.h>
#include <ESPmDNS.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "esp_partition.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "wear_levelling.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

extern "C" {
#include "class/msc/msc.h"
#include "class/msc/msc_device.h"
}

#include "dongle_kbd.h"  /* POST /type -> HID keyboard */

#define TAG               "disk"
#define FAT_PART_LABEL    "ffat"
#define MDNS_HOSTNAME     "dosongle"
#define HTTP_DISK_LOCK_MS 250
#define OWNER_LOCK_MS     1000
#define MSC_BLOCK_SIZE    512U
#define FAT_CLUSTER_SECS  8U
#define FAT_ROOT_ENTRIES  512U

// Debug to the hardware UART (Serial0) so we don't pollute the USB CDC
// data link. Match the DBG macro style used elsewhere in this firmware.
#define DBG(...) do { Serial0.printf(__VA_ARGS__); } while (0)

static USBMSC          s_msc;
static wl_handle_t     s_wl = WL_INVALID_HANDLE;
static const esp_partition_t *s_part = nullptr;
// USB MSC must expose 512-byte logical sectors for DOS. The ESP wear-levelled
// flash sector remains 4 KB, so writes RMW the enclosing WL sector below.
static uint32_t        s_block_size = MSC_BLOCK_SIZE;
static uint32_t        s_block_count = 0;
static SemaphoreHandle_t s_lock = nullptr;     // owner mutex
static portMUX_TYPE    s_io_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t          s_msc_io_active = 0;
static volatile bool   s_raw_http_read_active = false;
static volatile bool   s_dongle_owns = false;  // false = USB owns, true = device-side raw FAT ops own it
static volatile bool   s_msc_present = true;   // current mediaPresent flag
static volatile bool   s_msc_writable = true;  // current host-side writable flag
static httpd_handle_t  s_httpd = nullptr;
static volatile bool   s_mdns_up = false;
static volatile bool   s_format_running = false;
static volatile bool   s_format_ok = false;
static volatile bool   s_ota_running = false;
static volatile bool   s_ota_ok = false;

static void put16(uint8_t *p, uint16_t v);
static void put32(uint8_t *p, uint32_t v);
static esp_err_t raw_write_bytes(uint32_t off, const void *buf, size_t len);
static esp_err_t raw_zero_bytes(uint32_t off, size_t len);

// ---- RAM ring-buffer log (observable via GET /disk-log) ------------------
// The T-Dongle S3 doesn't expose UART pads, so any Serial0.printf() from
// the firmware is invisible. This ring buffer captures disk-side diagnostic
// lines (MSC write failures, raw-FAT writer errors) in RAM where they
// survive any number of DOS / host crashes -- only an actual dongle reset
// loses them. Pull via the dosongle.sh disk-log subcommand or curl /disk-log.
// Pattern mirrors dongle_kbd.cpp's kbd_logf.

#define DISK_LOG_LINES    64
#define DISK_LOG_LINE_LEN 128

static portMUX_TYPE s_disk_log_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_disk_log[DISK_LOG_LINES][DISK_LOG_LINE_LEN];
static uint32_t s_disk_log_seq = 0;

// Sibling ring for USB device-state events (mount / unmount / suspend /
// resume / re-enumeration). Same pattern as disk_logf but called from
// main.cpp's loop poll. Exposed for C linkage so main.cpp can drive it
// without dragging in the rest of dongle_disk.cpp.
#define USB_LOG_LINES    32
#define USB_LOG_LINE_LEN 96
static portMUX_TYPE s_usb_log_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_usb_log[USB_LOG_LINES][USB_LOG_LINE_LEN];
static uint32_t s_usb_log_seq = 0;

extern "C" void usb_logf(const char *fmt, ...) {
    char line[USB_LOG_LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);

    portENTER_CRITICAL(&s_usb_log_mux);
    uint32_t seq = ++s_usb_log_seq;
    snprintf(s_usb_log[seq % USB_LOG_LINES], USB_LOG_LINE_LEN,
             "%lu %lu %s", (unsigned long)seq, (unsigned long)millis(), line);
    portEXIT_CRITICAL(&s_usb_log_mux);
}

static void disk_logf(const char *fmt, ...) {
    char line[DISK_LOG_LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);

    portENTER_CRITICAL(&s_disk_log_mux);
    uint32_t seq = ++s_disk_log_seq;
    snprintf(s_disk_log[seq % DISK_LOG_LINES], DISK_LOG_LINE_LEN,
             "%lu %s", (unsigned long)seq, line);
    portEXIT_CRITICAL(&s_disk_log_mux);
    // No Serial0 mirror: UART0 TX (GPIO43) is not exposed on the T-Dongle
    // S3, so printf'ing to it just burns cycles and risks blocking on
    // FIFO drain under a flood of failure logs.
}

// ---- write-back coalescing cache ----------------------------------------
// Why: DOS sends 8 consecutive SCSI WRITE(10)s of 512B each to fill a 4KB
// host write. Without coalescing each one does a full WL-sector RMW (read
// 4KB, modify 512B, erase 4KB, write 4KB). The back-to-back erase current
// spikes were severe enough to brown-out / POWERON-reset the dongle under
// sustained writes (DISKTEST WRITE+TIMING). With this single-sector
// write-back cache, 8 same-sector writes merge in RAM and become 1 erase +
// 1 write -- 8x fewer flash ops, 8x lower peak erase rate, 8x faster.
//
// Invariants (all updates inside msc_on_{read,write}, the idle timer, or
// the ownership-swap path, all serialized via s_io_mux + the active-IO
// counter):
//   s_wb_have     -> s_wb_cache holds a current snapshot of the WL sector
//                    starting at s_wb_sect_base. Reads within this sector
//                    MUST consult the cache because it may carry dirty
//                    data not yet flushed to flash.
//   s_wb_dirty    -> cache differs from flash; flush owed at s_wb_sect_base.
//
// Loss model: dirty cache is RAM-only. A dongle reset between the host's
// write and our flush loses the last <=4KB of writes. Mitigated by the
// idle-flush timer firing ~WB_IDLE_MS after the last write, plus an
// explicit flush at every ownership swap / eject / mediaPresent(false).

#define WB_CACHE_BYTES 4096

static uint8_t   s_wb_cache[WB_CACHE_BYTES];
static uint32_t  s_wb_sect_base = 0;
static bool      s_wb_have  = false;
static bool      s_wb_dirty = false;
static uint32_t  s_wb_merge_count = 0;   // writes that hit the cached sector
static uint32_t  s_wb_evict_count = 0;   // flushes triggered by sector change

// NB: an async eviction worker was attempted (Step 1 of the CHUSB-agent
// dark-window plan) but produced a crash cascade under macOS mount load --
// the host's burst of metadata writes (.Spotlight-V100, .fseventsd,
// AppleDouble forks) interacted with the worker + write-back cache + cache
// disable in some way that triggered a reboot loop. Each reboot lost the
// dirty cache, corrupting the FAT incrementally until the volume failed
// to mount entirely (at which point the loop stabilised). Rolled back
// to synchronous evictions on the TinyUSB task; the dongle still goes
// dark on the bus for ~70 ms per erase, which CHUSB doesn't tolerate,
// but macOS does. The proper fix is Step 2 -- replace the arduino-esp32
// USB init with ESP-IDF native tinyusb so we can IRAM-flag the OTG ISR
// and let CONFIG_SPI_FLASH_AUTO_SUSPEND preempt the cache-off window.
// Until that lands, we run synchronously.

// Erase+write the cache back to flash if dirty. Caller must hold the
// active-IO count (i.e. be inside an msc_*_begin/end window) or otherwise
// guarantee no concurrent flash op. Returns ESP_OK if nothing to do.
static esp_err_t wb_flush_to_flash(void) {
    if (!s_wb_have || !s_wb_dirty) return ESP_OK;
    if (s_wl == WL_INVALID_HANDLE) return ESP_ERR_INVALID_STATE;
    size_t sect = wl_sector_size(s_wl);
    if (sect > WB_CACHE_BYTES) return ESP_ERR_INVALID_SIZE;
    esp_err_t e = wl_erase_range(s_wl, s_wb_sect_base, sect);
    if (e != ESP_OK) {
        disk_logf("[wb] flush erase fail base=0x%x sect=%u err=%d (%s)\n",
                  (unsigned)s_wb_sect_base, (unsigned)sect, e, esp_err_to_name(e));
        return e;
    }
    e = wl_write(s_wl, s_wb_sect_base, s_wb_cache, sect);
    if (e != ESP_OK) {
        disk_logf("[wb] flush write fail base=0x%x sect=%u err=%d (%s)\n",
                  (unsigned)s_wb_sect_base, (unsigned)sect, e, esp_err_to_name(e));
        return e;
    }
    s_wb_dirty = false;
    return ESP_OK;
}

static void wb_invalidate(void) {
    s_wb_have      = false;
    s_wb_dirty     = false;
    s_wb_sect_base = 0;
}

// Load a fresh sector into the cache. Caller must have flushed any prior
// dirty contents first; we do NOT silently overwrite dirty data.
static esp_err_t wb_load_sector(uint32_t sect_base) {
    if (s_wb_have && s_wb_dirty && s_wb_sect_base != sect_base) {
        disk_logf("[wb] BUG: wb_load_sector(0x%x) while dirty at 0x%x\n",
                  (unsigned)sect_base, (unsigned)s_wb_sect_base);
        return ESP_ERR_INVALID_STATE;
    }
    size_t sect = wl_sector_size(s_wl);
    if (sect > WB_CACHE_BYTES) return ESP_ERR_INVALID_SIZE;
    esp_err_t e = wl_read(s_wl, sect_base, s_wb_cache, sect);
    if (e != ESP_OK) {
        disk_logf("[wb] load fail base=0x%x sect=%u err=%d (%s)\n",
                  (unsigned)sect_base, (unsigned)sect, e, esp_err_to_name(e));
        return e;
    }
    s_wb_have      = true;
    s_wb_dirty     = false;
    s_wb_sect_base = sect_base;
    return ESP_OK;
}

// Flush + invalidate. Used at ownership swaps and eject -- the cache must
// not survive into a context where the other side might write to flash.
static esp_err_t wb_flush_and_invalidate(void) {
    esp_err_t e = wb_flush_to_flash();
    wb_invalidate();
    return e;
}

// ---- low-level: raw flash via WL, 512-byte logical blocks ----

static bool msc_read_begin(void) {
    portENTER_CRITICAL(&s_io_mux);
    bool ok = s_msc_present && s_wl != WL_INVALID_HANDLE;
    if (ok) s_msc_io_active++;
    portEXIT_CRITICAL(&s_io_mux);
    return ok;
}

static bool msc_write_begin(void) {
    portENTER_CRITICAL(&s_io_mux);
    bool ok = !s_dongle_owns && !s_raw_http_read_active && s_msc_present && s_wl != WL_INVALID_HANDLE;
    if (ok) s_msc_io_active++;
    portEXIT_CRITICAL(&s_io_mux);
    return ok;
}

static void msc_io_end(void) {
    portENTER_CRITICAL(&s_io_mux);
    if (s_msc_io_active) s_msc_io_active--;
    portEXIT_CRITICAL(&s_io_mux);
}

static bool msc_io_idle(void) {
    portENTER_CRITICAL(&s_io_mux);
    bool idle = s_msc_io_active == 0;
    portEXIT_CRITICAL(&s_io_mux);
    return idle;
}

static void wait_for_msc_idle(void) {
    uint32_t start = millis();
    while (!msc_io_idle() && millis() - start < 1000U) delay(1);
}

static void raw_http_read_set(bool active) {
    portENTER_CRITICAL(&s_io_mux);
    s_raw_http_read_active = active;
    portEXIT_CRITICAL(&s_io_mux);
}

static void owner_set_device_side(bool owns) {
    portENTER_CRITICAL(&s_io_mux);
    s_dongle_owns = owns;
    portEXIT_CRITICAL(&s_io_mux);
}

// Read path. Cache hits return from RAM (carries dirty bytes that haven't
// been flushed yet); misses go straight to wl_read.
static int32_t msc_on_read(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    if (!msc_read_begin()) return -1;
    size_t addr = (size_t)lba * s_block_size + offset;
    if ((uint64_t)addr + bufsize > (uint64_t)s_block_count * s_block_size) {
        disk_logf("[msc] READ OOB lba=%u off=%u bsz=%u addr=0x%x limit=0x%x\n",
            (unsigned)lba, (unsigned)offset, (unsigned)bufsize,
            (unsigned)addr, (unsigned)((uint64_t)s_block_count * s_block_size));
        msc_io_end();
        return -1;
    }
    size_t sect = wl_sector_size(s_wl);
    size_t remaining = bufsize;
    uint8_t *dst = (uint8_t *)buffer;
    while (remaining > 0) {
        size_t sect_off  = addr & (sect - 1);
        size_t sect_base = addr - sect_off;
        size_t chunk     = sect - sect_off;
        if (chunk > remaining) chunk = remaining;
        if (s_wb_have && s_wb_sect_base == sect_base) {
            memcpy(dst, s_wb_cache + sect_off, chunk);
        } else {
            esp_err_t e = wl_read(s_wl, addr, dst, chunk);
            if (e != ESP_OK) {
                disk_logf("[msc] read wl_read fail lba=%u addr=0x%x bsz=%u err=%d (%s)\n",
                    (unsigned)lba, (unsigned)addr, (unsigned)chunk, e, esp_err_to_name(e));
                msc_io_end();
                return -1;
            }
        }
        addr += chunk; dst += chunk; remaining -= chunk;
    }
    msc_io_end();
    return (int32_t)bufsize;
}

// Write path: stages every byte into the write-back cache. A run of writes
// to the same WL sector collapses into a single erase+write at eviction
// (or idle-timer flush) -- the 8x current-draw and time saving that lets
// sustained writes run without browning out the chip. All failure paths
// log to disk_logf and survive into /disk-log.
static int32_t msc_on_write(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    if (!msc_write_begin()) {
        disk_logf("[msc] WRITE REJECTED lba=%u off=%u bsz=%u  owns=%d http_rd=%d present=%d wl=%d\n",
            (unsigned)lba, (unsigned)offset, (unsigned)bufsize,
            (int)s_dongle_owns, (int)s_raw_http_read_active,
            (int)s_msc_present, s_wl != WL_INVALID_HANDLE);
        return -1;
    }
    size_t sect = wl_sector_size(s_wl);
    size_t addr = (size_t)lba * s_block_size + offset;
    if ((uint64_t)addr + bufsize > (uint64_t)s_block_count * s_block_size) {
        disk_logf("[msc] WRITE OOB lba=%u off=%u bsz=%u addr=0x%x limit=0x%x\n",
            (unsigned)lba, (unsigned)offset, (unsigned)bufsize,
            (unsigned)addr, (unsigned)((uint64_t)s_block_count * s_block_size));
        msc_io_end();
        return -1;
    }
    if (sect > WB_CACHE_BYTES) {
        disk_logf("[msc] WB cache too small: sect=%u cache=%u (build defect)\n",
            (unsigned)sect, (unsigned)WB_CACHE_BYTES);
        msc_io_end();
        return -1;
    }

    size_t remaining = bufsize;
    uint8_t *src = buffer;
    while (remaining > 0) {
        size_t sect_off  = addr & (sect - 1);
        size_t sect_base = addr - sect_off;
        size_t chunk     = sect - sect_off;
        if (chunk > remaining) chunk = remaining;

        if (s_wb_have && s_wb_sect_base == sect_base) {
            // Same cached sector -- pure RAM merge, no flash op.
            memcpy(s_wb_cache + sect_off, src, chunk);
            s_wb_dirty = true;
            s_wb_merge_count++;
        } else {
            // Different sector. Flush the existing one if dirty, then load
            // the new one (skip the load on a full-sector overwrite).
            if (s_wb_have && s_wb_dirty) {
                esp_err_t e = wb_flush_to_flash();
                if (e != ESP_OK) {
                    msc_io_end();
                    return -1;
                }
                s_wb_evict_count++;
            }
            if (sect_off == 0 && chunk == sect) {
                memcpy(s_wb_cache, src, sect);
                s_wb_have      = true;
                s_wb_dirty     = true;
                s_wb_sect_base = sect_base;
            } else {
                wb_invalidate();   // clean state for wb_load_sector's BUG check
                esp_err_t e = wb_load_sector(sect_base);
                if (e != ESP_OK) {
                    msc_io_end();
                    return -1;
                }
                memcpy(s_wb_cache + sect_off, src, chunk);
                s_wb_dirty = true;
            }
        }

        addr += chunk; src += chunk; remaining -= chunk;
    }
    msc_io_end();
    return (int32_t)bufsize;
}

static bool msc_on_start_stop(uint8_t power_condition, bool start, bool load_eject) {
    // Host SCSI 1Bh START_STOP_UNIT. We accept all; eject just flips our
    // mediaPresent flag so the host treats the volume as gone.
    if (load_eject && !start) {
        // Flush any dirty write-back cache before the medium disappears --
        // an eject with dirty cache and no subsequent eviction would lose
        // the last <=4KB of host writes on dongle reset. We are on the
        // TinyUSB task here, same as msc_on_write, so no race with cache
        // state; safe to call wb_flush_and_invalidate inline.
        esp_err_t e = wb_flush_and_invalidate();
        if (e != ESP_OK) {
            disk_logf("[wb] eject flush err=%d (%s) -- data may be lost\n",
                      e, esp_err_to_name(e));
        }
        s_msc.mediaPresent(false);
        s_msc_present = false;
        s_msc.isWritable(false);
        s_msc_writable = false;
    }
    return true;
}

// ---- ownership: flip USB <-> device-side raw FAT operations atomically ----

static esp_err_t format_for_device_locked(void) {
    if (!s_dongle_owns || s_wl == WL_INVALID_HANDLE) return ESP_ERR_INVALID_STATE;
    uint32_t total_secs = s_block_count;
    uint16_t reserved = 1;
    uint8_t fats = 2;
    uint16_t root_entries = FAT_ROOT_ENTRIES;
    uint32_t root_secs = (root_entries * 32U + MSC_BLOCK_SIZE - 1U) / MSC_BLOCK_SIZE;
    uint8_t spc = FAT_CLUSTER_SECS;
    uint16_t spf = 1;

    for (;;) {
        uint32_t data_secs = total_secs - reserved - root_secs - (uint32_t)fats * spf;
        uint32_t clusters = data_secs / spc;
        uint32_t fat_bytes = ((clusters + 2U) * 3U + 1U) / 2U;
        uint16_t need_spf = (uint16_t)((fat_bytes + MSC_BLOCK_SIZE - 1U) / MSC_BLOCK_SIZE);
        if (need_spf == spf) break;
        spf = need_spf;
    }

    uint8_t sec[MSC_BLOCK_SIZE];
    memset(sec, 0, sizeof sec);
    sec[0] = 0xEB; sec[1] = 0x3C; sec[2] = 0x90;
    memcpy(sec + 3, "MSDOS5.0", 8);
    put16(sec + 11, MSC_BLOCK_SIZE);
    sec[13] = spc;
    put16(sec + 14, reserved);
    sec[16] = fats;
    put16(sec + 17, root_entries);
    if (total_secs <= 0xFFFFU) put16(sec + 19, (uint16_t)total_secs);
    else put32(sec + 32, total_secs);
    sec[21] = 0xF8;
    put16(sec + 22, spf);
    put16(sec + 24, 32);      // sectors/track, conventional geometry only
    put16(sec + 26, 64);      // heads
    // drive number: 0x00 = first removable / "super-floppy" (no MBR
    // partition table). With drive=0x80 macOS treats the 0x55AA signature
    // at the BPB's end as an MBR marker, looks for a partition table in
    // the BPB padding area, finds all zeros, and declares "uninitialized".
    sec[36] = 0x00;
    sec[38] = 0x29;
    put32(sec + 39, 0x444F5301UL);
    memcpy(sec + 43, "DOSONGLE   ", 11);
    memcpy(sec + 54, "FAT12   ", 8);
    sec[510] = 0x55; sec[511] = 0xAA;
    esp_err_t err = raw_write_bytes(0, sec, sizeof sec);
    if (err != ESP_OK) return err;

    memset(sec, 0, sizeof sec);
    sec[0] = 0xF8; sec[1] = 0xFF; sec[2] = 0xFF;
    uint32_t fat0 = (uint32_t)reserved * MSC_BLOCK_SIZE;
    for (uint8_t f = 0; f < fats; ++f) {
        uint32_t base = fat0 + (uint32_t)f * spf * MSC_BLOCK_SIZE;
        err = raw_write_bytes(base, sec, sizeof sec);
        if (err != ESP_OK) return err;
        if (spf > 1) {
            err = raw_zero_bytes(base + MSC_BLOCK_SIZE, (uint32_t)(spf - 1U) * MSC_BLOCK_SIZE);
            if (err != ESP_OK) return err;
        }
    }

    uint32_t root = fat0 + (uint32_t)fats * spf * MSC_BLOCK_SIZE;
    err = raw_zero_bytes(root, root_secs * MSC_BLOCK_SIZE);
    if (err != ESP_OK) return err;

    uint8_t label[32];
    memset(label, 0, sizeof label);
    memcpy(label, "DOSONGLE   ", 11);
    label[11] = 0x08;
    return raw_write_bytes(root, label, sizeof label);
}

static void msc_signal_medium_changed(void) {
    // Only one MSC LUN is created in this firmware, so LUN 0 is the disk.
    // 0x28/0x00 is "not ready to ready change, medium may have changed".
    tud_msc_set_sense(0, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
}

static void set_msc_present(bool present) {
    s_msc.mediaPresent(present);
    s_msc_present = present;
}

static void set_msc_writable(bool writable) {
    s_msc.isWritable(writable);
    s_msc_writable = writable;
}

static void msc_take_offline(bool writable_when_back) {
    set_msc_writable(false);
    // Make REQUEST SENSE report "medium not present" while TEST UNIT READY
    // is false. The Arduino USBMSC wrapper returns false for TUR when
    // mediaPresent is false, but does not populate a NOT READY sense itself.
    tud_msc_set_sense(0, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
    set_msc_present(false);
    vTaskDelay(pdMS_TO_TICKS(150));
    wait_for_msc_idle();
    // Cache must not survive into the device-side or post-eject context:
    // the FAT might be modified by the other side, and we'd be holding a
    // stale snapshot. Flush any dirty data back to flash and invalidate.
    esp_err_t e = wb_flush_and_invalidate();
    if (e != ESP_OK) {
        disk_logf("[wb] take_offline flush err=%d (%s) -- data may be lost\n",
                  e, esp_err_to_name(e));
    }
    set_msc_writable(writable_when_back);
}

static void msc_bring_online(bool writable) {
    set_msc_writable(writable);
    set_msc_present(true);
    msc_signal_medium_changed();
    vTaskDelay(pdMS_TO_TICKS(100));
}

static esp_err_t take_for_device_locked(void) {
    if (s_dongle_owns) return ESP_OK;
    msc_take_offline(false);
    owner_set_device_side(true);
    return ESP_OK;
}

bool dongle_disk_format(void) {
    if (!s_lock) return false;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(OWNER_LOCK_MS)) != pdTRUE) return false;
    esp_err_t err = s_dongle_owns ? ESP_OK : take_for_device_locked();
    if (err == ESP_OK) {
        err = format_for_device_locked();
    }
    xSemaphoreGive(s_lock);
    return err == ESP_OK;
}

static esp_err_t give_to_usb_locked(void) {
    msc_take_offline(true);
    if (s_dongle_owns) {
        owner_set_device_side(false);
    }
    msc_bring_online(true);
    return ESP_OK;
}

static esp_err_t set_owner(dongle_disk_owner_t owner) {
    if (!s_lock) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(OWNER_LOCK_MS)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t err = (owner == DONGLE_DISK_OWNER_DEVICE)
                    ? take_for_device_locked()
                    : give_to_usb_locked();
    xSemaphoreGive(s_lock);
    return err;
}

bool dongle_disk_set_owner(dongle_disk_owner_t owner) {
    return set_owner(owner) == ESP_OK;
}

dongle_disk_owner_t dongle_disk_get_owner(void) {
    return s_dongle_owns ? DONGLE_DISK_OWNER_DEVICE : DONGLE_DISK_OWNER_USB;
}

const char *dongle_disk_owner_name(dongle_disk_owner_t owner) {
    return owner == DONGLE_DISK_OWNER_DEVICE ? "device-write" : "host-write";
}

// ---- HTTP helpers ----

static esp_err_t send_text(httpd_req_t *req, const char *status, const char *ct, const char *body) {
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, ct);
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static const char *INDEX_HTML =
    "<!doctype html><meta charset=utf-8>"
    "<title>dosongle</title>"
    "<style>body{font:13px/1.5 system-ui;max-width:48em;margin:2em auto;padding:0 1em}"
    "code{background:#f3f3f3;padding:.1em .4em;border-radius:3px}"
    "h1{font-size:1.4em}li{margin:.3em 0}</style>"
    "<h1>dosongle.local</h1>"
    "<p>HTTP gateway to the T-Dongle S3 dev disk.</p>"
    "<ul>"
    "<li><a href=/status>/status</a> &mdash; JSON: WiFi, MSC, mount, partition</li>"
    "<li><a href=/list>/list</a> &mdash; files on the FAT partition</li>"
    "<li><code>GET /fs/&lt;path&gt;</code> &mdash; download a file</li>"
    "<li><code>PUT /fs/&lt;path&gt;</code> &mdash; upload root 8.3 file in device-write</li>"
    "<li><code>DELETE /fs/&lt;path&gt;</code> &mdash; delete root 8.3 file in device-write</li>"
    "<li><code>POST /device-write</code> &mdash; device owns the disk; MSC medium not ready</li>"
    "<li><code>POST /host-write</code> &mdash; USB MSC owns the disk; host may mount/write it</li>"
    "<li><code>POST /format</code> &mdash; start explicit 512-byte-sector FAT format</li>"
    "<li><code>POST /eject</code> / <code>/present</code> &mdash; aliases for device / host write</li>"
    "<li><code>POST /type</code> &mdash; HID keyboard input (body is text, "
    "tokens like <code>&lt;ENTER&gt;</code>, <code>&lt;F1&gt;</code>, "
    "<code>&lt;CTRL+C&gt;</code>, <code>&lt;DELAY=200&gt;</code>)</li>"
    "<li><a href=/type-log>/type-log</a> &mdash; recent parsed keyboard events</li>"
    "<li><a href=/disk-log>/disk-log</a> &mdash; MSC + raw-FAT diagnostic ring buffer "
    "(survives DOS crashes; lost only on dongle reset)</li>"
    "<li><a href=/usb-log>/usb-log</a> &mdash; USB device-state events (mount, "
    "unmount, suspend, resume, deferred-WiFi markers)</li>"
    "<li><code>POST /ota</code> &mdash; HTTP OTA upload of firmware.bin</li>"
    "<li><code>POST /reset</code> &mdash; reboot dongle</li>"
    "</ul>"
    "<p><code>GET /fs</code> and <code>/list</code> read raw FAT blocks in either "
    "ownership mode. HTTP writes require device-write.</p>";

static esp_err_t h_root(httpd_req_t *req) { return send_text(req, "200 OK", "text/html; charset=utf-8", INDEX_HTML); }

static esp_err_t h_status(httpd_req_t *req) {
    char buf[512];
    dongle_disk_owner_t owner = dongle_disk_get_owner();
    int n = snprintf(buf, sizeof buf,
        "{\"mdns\":\"%s.local\",\"wifi\":{\"connected\":%s,\"ip\":\"%s\",\"rssi\":%d},"
        "\"mode\":\"%s\","
        "\"msc\":{\"present\":%s,\"writable\":%s,\"block_size\":%u,\"block_count\":%u},"
        "\"format\":{\"running\":%s,\"last_ok\":%s},"
        "\"ota\":{\"running\":%s,\"last_ok\":%s},"
        "\"fs\":{\"mounted\":false,\"info_valid\":false,\"total\":0,\"free\":0}}",
        MDNS_HOSTNAME,
        WiFi.status() == WL_CONNECTED ? "true" : "false",
        WiFi.localIP().toString().c_str(),
        (int)WiFi.RSSI(),
        dongle_disk_owner_name(owner),
        s_msc_present ? "true" : "false",
        s_msc_writable ? "true" : "false",
        (unsigned)s_block_size, (unsigned)s_block_count,
        s_format_running ? "true" : "false",
        s_format_ok ? "true" : "false",
        s_ota_running ? "true" : "false",
        s_ota_ok ? "true" : "false");
    if (n < 0) n = 0;
    return send_text(req, "200 OK", "application/json", buf);
}

struct raw_fat_info_t {
    uint16_t bps;
    uint8_t  spc;
    uint8_t  fats;
    uint16_t root_entries;
    uint32_t total_sectors;
    uint32_t sectors_per_fat;
    uint32_t fat_start;
    uint32_t root_start;
    uint32_t root_bytes;
    uint32_t data_start;
    uint32_t cluster_count;
    bool     fat12;
};

static uint32_t le32(const uint8_t *p);
static esp_err_t raw_read_bytes(uint32_t off, void *buf, size_t len);
static bool raw_fat_mount_info(raw_fat_info_t *fi, char *why, size_t whysz);
static bool raw_fat_get_cluster(const raw_fat_info_t *fi, uint16_t cluster, uint16_t *value);
static esp_err_t raw_fat_set_cluster(const raw_fat_info_t *fi, uint16_t cluster, uint16_t value);
static esp_err_t raw_fat_free_chain(const raw_fat_info_t *fi, uint16_t first);
static bool raw_fat_find_dir_entry(const raw_fat_info_t *fi, const char name83[11],
                                   uint8_t *ent, uint32_t *entry_off);
static bool raw_fat_find_free_dir_entry(const raw_fat_info_t *fi, uint32_t *entry_off);
static esp_err_t raw_fat_write_dir_entry(const raw_fat_info_t *fi, uint32_t entry_off,
                                         const uint8_t ent[32]);

static esp_err_t h_list(httpd_req_t *req) {
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(HTTP_DISK_LOCK_MS)) != pdTRUE)
        return send_text(req, "503 Service Unavailable", "text/plain", "lock failed\n");

    raw_http_read_set(true);
    wait_for_msc_idle();

    raw_fat_info_t fi;
    char why[96];
    if (!raw_fat_mount_info(&fi, why, sizeof why)) {
        raw_http_read_set(false);
        xSemaphoreGive(s_lock);
        char rsp[128];
        snprintf(rsp, sizeof rsp, "raw FAT parse failed: %s\n", why);
        return send_text(req, "500 Internal Server Error", "text/plain", rsp);
    }

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");

    uint8_t ent[32];
    char line[160];
    bool ok = true;
    for (uint32_t off = 0; off < fi.root_bytes; off += 32) {
        if (raw_read_bytes(fi.root_start + off, ent, sizeof ent) != ESP_OK) {
            ok = false;
            break;
        }
        if (ent[0] == 0x00) break;
        if (ent[0] == 0xE5) continue;
        uint8_t attr = ent[11];
        if (attr == 0x0F || (attr & 0x08)) continue; // LFN or volume label

        char name[13];
        int ni = 0;
        for (int i = 0; i < 8 && ent[i] != ' '; ++i) name[ni++] = (char)ent[i];
        if (!(attr & 0x10)) {
            int has_ext = 0;
            for (int i = 8; i < 11; ++i) if (ent[i] != ' ') has_ext = 1;
            if (has_ext) {
                name[ni++] = '.';
                for (int i = 8; i < 11 && ent[i] != ' '; ++i) name[ni++] = (char)ent[i];
            }
        }
        name[ni] = 0;

        uint32_t sz = (attr & 0x10) ? 0 : le32(ent + 28);
        int n = snprintf(line, sizeof line, "%10lu  %s%s\n",
                         (unsigned long)sz, name, (attr & 0x10) ? "/" : "");
        if (n > 0 && httpd_resp_send_chunk(req, line, n) != ESP_OK) {
            ok = false;
            break;
        }
    }
    raw_http_read_set(false);
    httpd_resp_send_chunk(req, nullptr, 0);
    xSemaphoreGive(s_lock);
    return ok ? ESP_OK : ESP_FAIL;
}

static uint16_t le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static bool uri_to_83_name(const char *uri, char out[11]) {
    if (strncmp(uri, "/fs/", 4) != 0) return false;
    const char *p = uri + 4;
    if (!*p || strchr(p, '/') || strstr(p, "..")) return false;
    memset(out, ' ', 11);

    int base = 0, ext = 0;
    bool in_ext = false;
    for (; *p; ++p) {
        char c = *p;
        if (c == '.') {
            if (in_ext) return false;
            in_ext = true;
            continue;
        }
        if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
        if ((unsigned char)c <= ' ' || strchr("\"*+,/:;<=>?[\\]|", c)) return false;
        if (!in_ext) {
            if (base >= 8) return false;
            out[base++] = c;
        } else {
            if (ext >= 3) return false;
            out[8 + ext++] = c;
        }
    }
    return base > 0;
}

static esp_err_t raw_read_bytes(uint32_t off, void *buf, size_t len) {
    if (s_wl == WL_INVALID_HANDLE) return ESP_ERR_INVALID_STATE;
    if ((uint64_t)off + len > (uint64_t)s_block_count * s_block_size) return ESP_ERR_INVALID_SIZE;
    // STATIC -- this function is called transitively from h_fs_put / h_list /
    // h_fs_get_raw_host on the HTTPD task whose stack is only 8 KB. A 4 KB
    // stack local plus the wl_* call chain (each layer adds another ~512 B)
    // was a real stack-overflow / PANIC vector under sustained writes. The
    // dongle's MSC IO is single-threaded at the wl_* layer (s_lock and
    // s_io_mux serialize the callers that reach here), so a shared static
    // buffer is safe. Matches the pattern used by raw_write_bytes /
    // raw_zero_bytes already.
    static uint8_t sector[4096];
    if (s_block_size > sizeof sector) return ESP_ERR_INVALID_SIZE;
    uint8_t *dst = (uint8_t *)buf;
    while (len > 0) {
        uint32_t base = off - (off % s_block_size);
        uint32_t pos = off - base;
        size_t n = s_block_size - pos;
        if (n > len) n = len;
        esp_err_t err = wl_read(s_wl, base, sector, s_block_size);
        if (err != ESP_OK) return err;
        memcpy(dst, sector + pos, n);
        off += n;
        dst += n;
        len -= n;
    }
    return ESP_OK;
}

static esp_err_t raw_write_bytes(uint32_t off, const void *buf, size_t len) {
    if (s_wl == WL_INVALID_HANDLE) return ESP_ERR_INVALID_STATE;
    if ((uint64_t)off + len > (uint64_t)s_block_count * s_block_size) return ESP_ERR_INVALID_SIZE;
    size_t sect = wl_sector_size(s_wl);
    static uint8_t sbuf[4096];
    if (sect > sizeof sbuf || (sect & (sect - 1)) != 0) return ESP_ERR_INVALID_SIZE;

    const uint8_t *src = (const uint8_t *)buf;
    while (len > 0) {
        size_t sect_off = off & (sect - 1);
        size_t sect_base = off - sect_off;
        size_t chunk = sect - sect_off;
        if (chunk > len) chunk = len;
        if (wl_read(s_wl, sect_base, sbuf, sect) != ESP_OK) return ESP_FAIL;
        memcpy(sbuf + sect_off, src, chunk);
        if (wl_erase_range(s_wl, sect_base, sect) != ESP_OK) return ESP_FAIL;
        if (wl_write(s_wl, sect_base, sbuf, sect) != ESP_OK) return ESP_FAIL;
        off += chunk;
        src += chunk;
        len -= chunk;
    }
    return ESP_OK;
}

static esp_err_t raw_zero_bytes(uint32_t off, size_t len) {
    static uint8_t zero[4096];
    size_t sect = wl_sector_size(s_wl);
    if (sect > sizeof zero || (sect & (sect - 1)) != 0) return ESP_ERR_INVALID_SIZE;

    while (len > 0) {
        if ((off & (sect - 1)) == 0 && len >= sect) {
            if (wl_erase_range(s_wl, off, sect) != ESP_OK) return ESP_FAIL;
            if (wl_write(s_wl, off, zero, sect) != ESP_OK) return ESP_FAIL;
            off += sect;
            len -= sect;
            continue;
        }

        size_t align = sect - (off & (sect - 1));
        size_t n = len < align ? len : align;
        esp_err_t err = raw_write_bytes(off, zero, n);
        if (err != ESP_OK) return err;
        off += n;
        len -= n;
    }
    return ESP_OK;
}

static bool raw_fat_mount_info(raw_fat_info_t *fi, char *why, size_t whysz) {
    uint8_t bpb[64];
    if (raw_read_bytes(0, bpb, sizeof bpb) != ESP_OK) {
        snprintf(why, whysz, "boot read failed");
        return false;
    }
    if (bpb[510 % sizeof bpb] == 0) {
        // The signature is outside this small BPB read when bytes/sector > 512;
        // validate below after parsing the sector size.
    }
    memset(fi, 0, sizeof *fi);
    fi->bps = le16(bpb + 11);
    fi->spc = bpb[13];
    uint16_t reserved = le16(bpb + 14);
    fi->fats = bpb[16];
    fi->root_entries = le16(bpb + 17);
    uint16_t total16 = le16(bpb + 19);
    uint16_t spf16 = le16(bpb + 22);
    uint32_t total32 = le32(bpb + 32);
    if (fi->bps == 0 || fi->spc == 0 || fi->fats == 0 || reserved == 0 || spf16 == 0) {
        snprintf(why, whysz, "bad bpb bps=%u spc=%u fats=%u reserved=%u spf=%u",
                 fi->bps, fi->spc, fi->fats, reserved, spf16);
        return false;
    }
    if (fi->bps > 4096 || (fi->bps & (fi->bps - 1)) != 0) {
        snprintf(why, whysz, "bad sector size %u", fi->bps);
        return false;
    }
    fi->total_sectors = total16 ? total16 : total32;
    fi->sectors_per_fat = spf16;
    fi->fat_start = (uint32_t)reserved * fi->bps;
    fi->root_start = (uint32_t)(reserved + fi->fats * fi->sectors_per_fat) * fi->bps;
    fi->root_bytes = (uint32_t)fi->root_entries * 32U;
    uint32_t root_sectors = (fi->root_bytes + fi->bps - 1) / fi->bps;
    // Defend against corrupt BPB: if total_sectors is smaller than the
    // header overhead, data_sectors would underflow into a huge uint32_t
    // and downstream FAT scans (e.g. raw_fat_find_free_clusters walking
    // billions of entries) would hang the dongle. Bail early instead.
    uint32_t overhead = (uint32_t)reserved + (uint32_t)fi->fats * fi->sectors_per_fat + root_sectors;
    if (fi->total_sectors <= overhead) {
        snprintf(why, whysz, "bad total_sectors=%u (overhead=%u)",
                 (unsigned)fi->total_sectors, (unsigned)overhead);
        return false;
    }
    fi->data_start = (uint32_t)overhead * fi->bps;
    uint32_t data_sectors = fi->total_sectors - overhead;
    uint32_t clusters = (fi->spc > 0) ? (data_sectors / fi->spc) : 0;
    fi->cluster_count = clusters;
    fi->fat12 = clusters < 4085;

    uint8_t sig[2];
    if (raw_read_bytes(510, sig, sizeof sig) != ESP_OK) {
        snprintf(why, whysz, "signature read failed bps=%u", fi->bps);
        return false;
    }
    if (sig[0] != 0x55 || sig[1] != 0xAA) {
        snprintf(why, whysz, "bad signature %02x %02x bps=%u", sig[0], sig[1], fi->bps);
        return false;
    }
    return true;
}

static uint32_t raw_fat_cluster_bytes(const raw_fat_info_t *fi) {
    return (uint32_t)fi->spc * fi->bps;
}

static uint32_t raw_fat_cluster_offset(const raw_fat_info_t *fi, uint16_t cluster) {
    return fi->data_start + (uint32_t)(cluster - 2) * raw_fat_cluster_bytes(fi);
}

static uint16_t raw_fat_eoc(const raw_fat_info_t *fi) {
    return fi->fat12 ? 0x0FFF : 0xFFFF;
}

static bool raw_fat_valid_cluster(const raw_fat_info_t *fi, uint16_t cluster) {
    return cluster >= 2 && (uint32_t)(cluster - 2) < fi->cluster_count;
}

static bool raw_fat_get_cluster(const raw_fat_info_t *fi, uint16_t cluster, uint16_t *value) {
    if (!raw_fat_valid_cluster(fi, cluster)) return false;
    if (fi->fat12) {
        uint32_t off = fi->fat_start + cluster + (cluster / 2);
        uint8_t pair[2];
        if (raw_read_bytes(off, pair, sizeof pair) != ESP_OK) return false;
        uint16_t v = (uint16_t)pair[0] | ((uint16_t)pair[1] << 8);
        *value = (cluster & 1) ? (v >> 4) : (v & 0x0FFF);
    } else {
        uint8_t pair[2];
        if (raw_read_bytes(fi->fat_start + (uint32_t)cluster * 2U, pair, sizeof pair) != ESP_OK) return false;
        *value = le16(pair);
    }
    return true;
}

static esp_err_t raw_fat_set_one(const raw_fat_info_t *fi, uint32_t fat_start, uint16_t cluster, uint16_t value) {
    if (!raw_fat_valid_cluster(fi, cluster)) return ESP_ERR_INVALID_ARG;
    if (fi->fat12) {
        uint32_t off = fat_start + cluster + (cluster / 2);
        uint8_t pair[2];
        esp_err_t err = raw_read_bytes(off, pair, sizeof pair);
        if (err != ESP_OK) return err;
        uint16_t v = (uint16_t)pair[0] | ((uint16_t)pair[1] << 8);
        value &= 0x0FFF;
        if (cluster & 1) v = (uint16_t)((v & 0x000F) | (value << 4));
        else v = (uint16_t)((v & 0xF000) | value);
        pair[0] = (uint8_t)v;
        pair[1] = (uint8_t)(v >> 8);
        return raw_write_bytes(off, pair, sizeof pair);
    }
    uint8_t pair[2];
    put16(pair, value);
    return raw_write_bytes(fat_start + (uint32_t)cluster * 2U, pair, sizeof pair);
}

static esp_err_t raw_fat_set_cluster(const raw_fat_info_t *fi, uint16_t cluster, uint16_t value) {
    esp_err_t err = ESP_OK;
    for (uint8_t f = 0; f < fi->fats; ++f) {
        uint32_t fat_start = fi->fat_start + (uint32_t)f * fi->sectors_per_fat * fi->bps;
        err = raw_fat_set_one(fi, fat_start, cluster, value);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static esp_err_t raw_fat_free_chain(const raw_fat_info_t *fi, uint16_t first) {
    uint16_t cluster = first;
    uint32_t guard = 0;
    while (raw_fat_valid_cluster(fi, cluster) && guard++ < fi->cluster_count) {
        uint16_t next = 0;
        if (!raw_fat_get_cluster(fi, cluster, &next)) return ESP_FAIL;
        esp_err_t err = raw_fat_set_cluster(fi, cluster, 0);
        if (err != ESP_OK) return err;
        if (fi->fat12 ? (next >= 0x0FF8) : (next >= 0xFFF8)) break;
        cluster = next;
    }
    return ESP_OK;
}

static bool raw_fat_next_cluster(const raw_fat_info_t *fi, uint16_t cluster, uint16_t *next) {
    if (cluster < 2) return false;
    return raw_fat_get_cluster(fi, cluster, next);
}

static bool raw_fat_find_dir_entry(const raw_fat_info_t *fi, const char name83[11],
                                   uint8_t *ent, uint32_t *entry_off) {
    uint8_t tmp[32];
    for (uint32_t off = 0; off < fi->root_bytes; off += 32) {
        if (raw_read_bytes(fi->root_start + off, tmp, sizeof tmp) != ESP_OK) return false;
        if (tmp[0] == 0x00) break;
        if (tmp[0] == 0xE5) continue;
        uint8_t attr = tmp[11];
        if (attr == 0x0F || (attr & 0x18)) continue;
        if (!memcmp(tmp, name83, 11)) {
            if (ent) memcpy(ent, tmp, sizeof tmp);
            if (entry_off) *entry_off = fi->root_start + off;
            return true;
        }
    }
    return false;
}

static bool raw_fat_find_free_dir_entry(const raw_fat_info_t *fi, uint32_t *entry_off) {
    uint8_t ent[32];
    uint32_t deleted_off = 0;
    bool have_deleted = false;
    for (uint32_t off = 0; off < fi->root_bytes; off += 32) {
        if (raw_read_bytes(fi->root_start + off, ent, sizeof ent) != ESP_OK) return false;
        if (ent[0] == 0xE5 && !have_deleted) {
            deleted_off = fi->root_start + off;
            have_deleted = true;
        }
        if (ent[0] == 0x00) {
            *entry_off = have_deleted ? deleted_off : fi->root_start + off;
            return true;
        }
    }
    if (have_deleted) {
        *entry_off = deleted_off;
        return true;
    }
    return false;
}

static esp_err_t raw_fat_write_dir_entry(const raw_fat_info_t *fi, uint32_t entry_off,
                                         const uint8_t ent[32]) {
    if (entry_off < fi->root_start || entry_off + 32 > fi->root_start + fi->root_bytes)
        return ESP_ERR_INVALID_ARG;
    return raw_write_bytes(entry_off, ent, 32);
}

static bool raw_fat_find_file(const raw_fat_info_t *fi, const char name83[11], uint16_t *cluster, uint32_t *size) {
    uint8_t ent[32];
    if (raw_fat_find_dir_entry(fi, name83, ent, nullptr)) {
        *cluster = le16(ent + 26);
        *size = le32(ent + 28);
        return true;
    }
    return false;
}

static esp_err_t h_fs_get_raw_host(httpd_req_t *req) {
    char name83[11];
    if (!uri_to_83_name(req->uri, name83))
        return send_text(req, "400 Bad Request", "text/plain", "bad 8.3 path\n");

    raw_http_read_set(true);
    wait_for_msc_idle();

    raw_fat_info_t fi;
    char why[96];
    if (!raw_fat_mount_info(&fi, why, sizeof why)) {
        raw_http_read_set(false);
        char rsp[128];
        snprintf(rsp, sizeof rsp, "raw FAT parse failed: %s\n", why);
        return send_text(req, "500 Internal Server Error", "text/plain", rsp);
    }

    uint16_t cluster = 0;
    uint32_t remaining = 0;
    if (!raw_fat_find_file(&fi, name83, &cluster, &remaining)) {
        raw_http_read_set(false);
        return send_text(req, "404 Not Found", "text/plain", "no such file\n");
    }

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Connection", "close");

    uint8_t buf[1024];
    bool ok = true;
    while (remaining > 0 && cluster >= 2) {
        uint32_t cluster_bytes = (uint32_t)fi.spc * fi.bps;
        uint32_t base = fi.data_start + (uint32_t)(cluster - 2) * cluster_bytes;
        uint32_t in_cluster = remaining < cluster_bytes ? remaining : cluster_bytes;
        for (uint32_t pos = 0; pos < in_cluster; ) {
            uint32_t n = in_cluster - pos;
            if (n > sizeof buf) n = sizeof buf;
            if (raw_read_bytes(base + pos, buf, n) != ESP_OK ||
                httpd_resp_send_chunk(req, (const char *)buf, n) != ESP_OK) {
                ok = false;
                break;
            }
            pos += n;
            remaining -= n;
        }
        if (!ok || remaining == 0) break;
        uint16_t next = 0;
        if (!raw_fat_next_cluster(&fi, cluster, &next)) {
            ok = false;
            break;
        }
        if (fi.fat12 ? (next >= 0x0FF8) : (next >= 0xFFF8)) break;
        cluster = next;
    }
    raw_http_read_set(false);
    httpd_resp_send_chunk(req, nullptr, 0);
    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t h_fs_get(httpd_req_t *req) {
    char name83[11];
    if (!uri_to_83_name(req->uri, name83))
        return send_text(req, "400 Bad Request", "text/plain", "bad path\n");

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(HTTP_DISK_LOCK_MS)) != pdTRUE)
        return send_text(req, "503 Service Unavailable", "text/plain", "lock failed\n");
    esp_err_t raw_err = h_fs_get_raw_host(req);
    xSemaphoreGive(s_lock);
    return raw_err;
}

static bool raw_fat_find_free_clusters(const raw_fat_info_t *fi, uint16_t *clusters, uint32_t need) {
    uint32_t found = 0;
    for (uint32_t i = 0; i < fi->cluster_count && found < need; ++i) {
        uint16_t cluster = (uint16_t)(i + 2);
        uint16_t value = 0;
        if (!raw_fat_get_cluster(fi, cluster, &value)) return false;
        if (value == 0) clusters[found++] = cluster;
    }
    return found == need;
}

static esp_err_t raw_fat_write_file_data(httpd_req_t *req, const raw_fat_info_t *fi,
                                         const uint16_t *clusters, uint32_t nclusters,
                                         uint32_t total) {
    uint8_t *buf = (uint8_t *)malloc(1024);
    if (!buf) return ESP_ERR_NO_MEM;
    uint32_t cluster_bytes = raw_fat_cluster_bytes(fi);
    uint32_t got = 0;
    esp_err_t err = ESP_OK;

    for (uint32_t ci = 0; ci < nclusters && err == ESP_OK; ++ci) {
        uint32_t base = raw_fat_cluster_offset(fi, clusters[ci]);
        uint32_t in_cluster = 0;
        while (in_cluster < cluster_bytes && got < total) {
            int want = (int)(cluster_bytes - in_cluster);
            if (want > 1024) want = 1024;
            if ((uint32_t)want > total - got) want = (int)(total - got);
            int r = httpd_req_recv(req, (char *)buf, want);
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            if (r <= 0) {
                err = ESP_FAIL;
                break;
            }
            err = raw_write_bytes(base + in_cluster, buf, (size_t)r);
            if (err != ESP_OK) break;
            in_cluster += (uint32_t)r;
            got += (uint32_t)r;
        }
        if (err == ESP_OK && in_cluster < cluster_bytes) {
            err = raw_zero_bytes(base + in_cluster, cluster_bytes - in_cluster);
        }
    }
    free(buf);
    if (err != ESP_OK) return err;
    return got == total ? ESP_OK : ESP_FAIL;
}

static esp_err_t raw_fat_link_clusters(const raw_fat_info_t *fi, const uint16_t *clusters, uint32_t nclusters) {
    for (uint32_t i = 0; i < nclusters; ++i) {
        uint16_t next = (i + 1 < nclusters) ? clusters[i + 1] : raw_fat_eoc(fi);
        esp_err_t err = raw_fat_set_cluster(fi, clusters[i], next);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static void raw_fat_make_file_entry(uint8_t ent[32], const char name83[11], uint16_t first_cluster, uint32_t size) {
    memset(ent, 0, 32);
    memcpy(ent, name83, 11);
    ent[11] = 0x20;          // archive
    put16(ent + 14, 0);      // create time
    put16(ent + 16, 0x0021); // create date: 1980-01-01
    put16(ent + 18, 0x0021); // access date
    put16(ent + 22, 0);      // write time
    put16(ent + 24, 0x0021); // write date
    put16(ent + 26, first_cluster);
    put32(ent + 28, size);
}

static esp_err_t h_fs_put(httpd_req_t *req) {
    char name83[11];
    if (!uri_to_83_name(req->uri, name83))
        return send_text(req, "400 Bad Request", "text/plain", "bad path\n");
    if (!s_dongle_owns)
        return send_text(req, "409 Conflict", "text/plain", "PUT requires device-write\n");
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(OWNER_LOCK_MS)) != pdTRUE)
        return send_text(req, "503 Service Unavailable", "text/plain", "lock failed\n");
    if (!s_dongle_owns) {
        xSemaphoreGive(s_lock);
        return send_text(req, "409 Conflict", "text/plain", "PUT requires device-write\n");
    }

    esp_err_t err = ESP_OK;
    raw_fat_info_t fi;
    char why[96];
    if (!raw_fat_mount_info(&fi, why, sizeof why)) {
        xSemaphoreGive(s_lock);
        char rsp[128];
        snprintf(rsp, sizeof rsp, "raw FAT parse failed: %s\n", why);
        return send_text(req, "500 Internal Server Error", "text/plain", rsp);
    }

    uint8_t old_ent[32];
    uint32_t entry_off = 0;
    uint16_t old_first = 0;
    bool replacing = raw_fat_find_dir_entry(&fi, name83, old_ent, &entry_off);
    if (replacing) {
        old_first = le16(old_ent + 26);
    } else if (!raw_fat_find_free_dir_entry(&fi, &entry_off)) {
        xSemaphoreGive(s_lock);
        return send_text(req, "507 Insufficient Storage", "text/plain", "root directory full\n");
    }

    // Guard against absent / negative / wildly oversized Content-Length:
    // (uint32_t)negative wraps to huge; uncapped sizes would walk billions
    // of FAT entries in raw_fat_find_free_clusters and lock up the dongle.
    if (req->content_len < 0) {
        xSemaphoreGive(s_lock);
        return send_text(req, "411 Length Required", "text/plain", "Content-Length required\n");
    }
    uint64_t partition_bytes = (uint64_t)s_block_count * s_block_size;
    if ((uint64_t)req->content_len > partition_bytes) {
        xSemaphoreGive(s_lock);
        return send_text(req, "413 Payload Too Large", "text/plain", "exceeds partition size\n");
    }
    uint32_t size = (uint32_t)req->content_len;
    uint32_t cluster_bytes = raw_fat_cluster_bytes(&fi);
    uint32_t nclusters = size ? ((size + cluster_bytes - 1U) / cluster_bytes) : 0;
    uint16_t *clusters = nullptr;
    if (nclusters) {
        clusters = (uint16_t *)malloc(nclusters * sizeof(uint16_t));
        if (!clusters) {
            xSemaphoreGive(s_lock);
            return send_text(req, "500 Internal Server Error", "text/plain", "oom\n");
        }
        if (!raw_fat_find_free_clusters(&fi, clusters, nclusters)) {
            free(clusters);
            xSemaphoreGive(s_lock);
            return send_text(req, "507 Insufficient Storage", "text/plain", "not enough free clusters\n");
        }
        err = raw_fat_write_file_data(req, &fi, clusters, nclusters, size);
        if (err == ESP_OK) err = raw_fat_link_clusters(&fi, clusters, nclusters);
    }

    if (err == ESP_OK) {
        uint8_t ent[32];
        raw_fat_make_file_entry(ent, name83, nclusters ? clusters[0] : 0, size);
        err = raw_fat_write_dir_entry(&fi, entry_off, ent);
    }
    if (err == ESP_OK && replacing && raw_fat_valid_cluster(&fi, old_first)) {
        err = raw_fat_free_chain(&fi, old_first);
    }

    if (err != ESP_OK && clusters) {
        for (uint32_t i = 0; i < nclusters; ++i) {
            raw_fat_set_cluster(&fi, clusters[i], 0);
        }
    }
    free(clusters);
    xSemaphoreGive(s_lock);

    if (err != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain", "PUT failed\n");
    char rsp[64];
    snprintf(rsp, sizeof rsp, "wrote %lu bytes\n", (unsigned long)size);
    return send_text(req, "200 OK", "text/plain", rsp);
}

static esp_err_t h_fs_delete(httpd_req_t *req) {
    char name83[11];
    if (!uri_to_83_name(req->uri, name83))
        return send_text(req, "400 Bad Request", "text/plain", "bad path\n");
    if (!s_dongle_owns)
        return send_text(req, "409 Conflict", "text/plain", "DELETE requires device-write\n");

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(OWNER_LOCK_MS)) != pdTRUE)
        return send_text(req, "503 Service Unavailable", "text/plain", "lock failed\n");
    if (!s_dongle_owns) {
        xSemaphoreGive(s_lock);
        return send_text(req, "409 Conflict", "text/plain", "DELETE requires device-write\n");
    }

    raw_fat_info_t fi;
    char why[96];
    if (!raw_fat_mount_info(&fi, why, sizeof why)) {
        xSemaphoreGive(s_lock);
        char rsp[128];
        snprintf(rsp, sizeof rsp, "raw FAT parse failed: %s\n", why);
        return send_text(req, "500 Internal Server Error", "text/plain", rsp);
    }

    uint8_t ent[32];
    uint32_t entry_off = 0;
    if (!raw_fat_find_dir_entry(&fi, name83, ent, &entry_off)) {
        xSemaphoreGive(s_lock);
        return send_text(req, "404 Not Found", "text/plain", "no such file\n");
    }
    uint16_t first = le16(ent + 26);
    ent[0] = 0xE5;
    esp_err_t err = raw_fat_write_dir_entry(&fi, entry_off, ent);
    if (err == ESP_OK && raw_fat_valid_cluster(&fi, first)) {
        err = raw_fat_free_chain(&fi, first);
    }
    xSemaphoreGive(s_lock);
    if (err != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain", "DELETE failed\n");
    return send_text(req, "200 OK", "text/plain", "deleted\n");
}

static esp_err_t h_owner_device(httpd_req_t *req) {
    esp_err_t err = set_owner(DONGLE_DISK_OWNER_DEVICE);
    if (err != ESP_OK)
        return send_text(req, "503 Service Unavailable", "text/plain", "could not take disk for device\n");
    return send_text(req, "200 OK", "text/plain", "mode=device-write; MSC medium not ready\n");
}

static esp_err_t h_owner_usb(httpd_req_t *req) {
    esp_err_t err = set_owner(DONGLE_DISK_OWNER_USB);
    if (err != ESP_OK)
        return send_text(req, "503 Service Unavailable", "text/plain", "could not present disk to usb\n");
    return send_text(req, "200 OK", "text/plain", "mode=host-write; MSC medium ready and writable\n");
}

static void format_task(void *arg) {
    (void)arg;
    bool ok = dongle_disk_format();
    s_format_ok = ok;
    s_format_running = false;
    vTaskDelete(nullptr);
}

static esp_err_t h_format(httpd_req_t *req) {
    portENTER_CRITICAL(&s_io_mux);
    bool busy = s_format_running;
    if (!busy) {
        s_format_running = true;
        s_format_ok = false;
    }
    portEXIT_CRITICAL(&s_io_mux);

    if (busy)
        return send_text(req, "409 Conflict", "text/plain", "format already running\n");

    if (xTaskCreate(format_task, "disk_format", 4096, nullptr, 1, nullptr) != pdPASS) {
        s_format_running = false;
        return send_text(req, "503 Service Unavailable", "text/plain", "could not start format task\n");
    }
    return send_text(req, "202 Accepted", "text/plain", "format started; poll /status\n");
}

static esp_err_t h_type(httpd_req_t *req) {
    // Body is the typing string (see dongle_kbd.h DSL). Use Content-Length;
    // chunked encoding isn't supported here.
    int total = req->content_len;
    if (total <= 0 || total > 4096)
        return send_text(req, "400 Bad Request", "text/plain", "need Content-Length 1..4096\n");
    char *body = (char *)malloc(total + 1);
    if (!body) return send_text(req, "500 Internal Server Error", "text/plain", "oom\n");
    int got = 0;
    while (got < total) {
        int r = httpd_req_recv(req, body + got, total - got);
        if (r <= 0) { if (r == HTTPD_SOCK_ERR_TIMEOUT) continue; break; }
        got += r;
    }
    body[got] = 0;
    int n = dongle_kbd_type(body, got);
    free(body);
    if (n < 0) return send_text(req, "400 Bad Request", "text/plain", "parse error\n");
    char rsp[64];
    snprintf(rsp, sizeof rsp, "typed %d bytes\n", n);
    return send_text(req, "200 OK", "text/plain", rsp);
}

static esp_err_t h_type_log(httpd_req_t *req) {
    char *buf = (char *)malloc(6144);
    if (!buf) return send_text(req, "500 Internal Server Error", "text/plain", "oom\n");
    int n = dongle_kbd_log_dump(buf, 6144);
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");
    esp_err_t err = httpd_resp_send(req, buf, n);
    free(buf);
    return err;
}

static esp_err_t h_ota(httpd_req_t *req) {
    int total = req->content_len;
    if (total <= 0)
        return send_text(req, "411 Length Required", "text/plain", "need Content-Length firmware.bin body\n");

    portENTER_CRITICAL(&s_io_mux);
    bool busy = s_ota_running;
    if (!busy) {
        s_ota_running = true;
        s_ota_ok = false;
    }
    portEXIT_CRITICAL(&s_io_mux);
    if (busy)
        return send_text(req, "409 Conflict", "text/plain", "OTA already running\n");

    const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
    if (!next) {
        s_ota_running = false;
        return send_text(req, "500 Internal Server Error", "text/plain", "no OTA partition\n");
    }
    if ((size_t)total > next->size) {
        s_ota_running = false;
        return send_text(req, "413 Payload Too Large", "text/plain", "firmware too large for OTA slot\n");
    }

    esp_ota_handle_t h = 0;
    esp_err_t err = esp_ota_begin(next, (size_t)total, &h);
    if (err != ESP_OK) {
        s_ota_running = false;
        return send_text(req, "500 Internal Server Error", "text/plain", "OTA begin failed\n");
    }

    uint8_t *buf = (uint8_t *)malloc(2048);
    if (!buf) {
        esp_ota_abort(h);
        s_ota_running = false;
        return send_text(req, "500 Internal Server Error", "text/plain", "oom\n");
    }

    int got = 0;
    bool ok = true;
    while (got < total) {
        int want = total - got;
        if (want > 2048) want = 2048;
        int r = httpd_req_recv(req, (char *)buf, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) {
            ok = false;
            break;
        }
        if (esp_ota_write(h, buf, (size_t)r) != ESP_OK) {
            ok = false;
            break;
        }
        got += r;
    }
    free(buf);

    if (!ok || got != total) {
        esp_ota_abort(h);
        s_ota_running = false;
        return send_text(req, "500 Internal Server Error", "text/plain", "OTA receive/write failed\n");
    }
    if (esp_ota_end(h) != ESP_OK) {
        s_ota_running = false;
        return send_text(req, "400 Bad Request", "text/plain", "OTA end failed; bad image?\n");
    }
    if (esp_ota_set_boot_partition(next) != ESP_OK) {
        s_ota_running = false;
        return send_text(req, "500 Internal Server Error", "text/plain", "OTA set boot failed\n");
    }

    s_ota_ok = true;
    s_ota_running = false;
    send_text(req, "200 OK", "text/plain", "OTA OK; rebooting\n");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

static esp_err_t h_reset(httpd_req_t *req) {
    send_text(req, "200 OK", "text/plain", "rebooting\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;
}

// GET /usb-log -- dump the USB device-state event ring (mount / unmount /
// suspend / resume / re-enumeration markers + WiFi-deferral events).
static esp_err_t h_usb_log(httpd_req_t *req) {
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");

    portENTER_CRITICAL(&s_usb_log_mux);
    uint32_t seq = s_usb_log_seq;
    portEXIT_CRITICAL(&s_usb_log_mux);
    uint32_t first = (seq > USB_LOG_LINES) ? (seq - USB_LOG_LINES + 1) : 1;

    char line[USB_LOG_LINE_LEN + 2];
    for (uint32_t cur = first; cur <= seq; ++cur) {
        portENTER_CRITICAL(&s_usb_log_mux);
        int w = snprintf(line, sizeof line, "%s\n", s_usb_log[cur % USB_LOG_LINES]);
        portEXIT_CRITICAL(&s_usb_log_mux);
        if (w <= 0) continue;
        if (httpd_resp_send_chunk(req, line, (size_t)w) != ESP_OK) break;
    }
    httpd_resp_send_chunk(req, nullptr, 0);
    return ESP_OK;
}

// GET /lba/<N> -- raw 512-byte sector dump (octet-stream). Diagnostic
// endpoint for inspecting the on-flash FAT layout when a host (macOS,
// CHUSB) won't mount it. Bypasses the cache so we see exactly what's
// on flash, not what we have buffered.
static esp_err_t h_lba(httpd_req_t *req) {
    const char *uri = req->uri;
    // Skip "/lba/"
    const char *p = strrchr(uri, '/');
    if (!p) return send_text(req, "400 Bad Request", "text/plain", "bad path\n");
    p++;
    uint32_t lba = (uint32_t)strtoul(p, nullptr, 0);
    if (s_wl == WL_INVALID_HANDLE)
        return send_text(req, "503 Service Unavailable", "text/plain", "wl not mounted\n");
    if (lba >= s_block_count)
        return send_text(req, "416 Range Not Satisfiable", "text/plain", "lba OOB\n");
    uint8_t buf[MSC_BLOCK_SIZE];
    if (wl_read(s_wl, (size_t)lba * MSC_BLOCK_SIZE, buf, MSC_BLOCK_SIZE) != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain", "wl_read failed\n");
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, (const char *)buf, MSC_BLOCK_SIZE);
}

// GET /disk-log -- dump the disk-side ring buffer (MSC + raw-FAT diagnostics).
// Streamed chunked so we don't need a big buffer; each line snapshot under
// the spinlock to avoid tearing if disk_logf appends concurrently.
static esp_err_t h_disk_log(httpd_req_t *req) {
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");

    portENTER_CRITICAL(&s_disk_log_mux);
    uint32_t seq = s_disk_log_seq;
    portEXIT_CRITICAL(&s_disk_log_mux);
    uint32_t first = (seq > DISK_LOG_LINES) ? (seq - DISK_LOG_LINES + 1) : 1;

    char line[DISK_LOG_LINE_LEN + 2];
    for (uint32_t cur = first; cur <= seq; ++cur) {
        portENTER_CRITICAL(&s_disk_log_mux);
        int w = snprintf(line, sizeof line, "%s\n", s_disk_log[cur % DISK_LOG_LINES]);
        portEXIT_CRITICAL(&s_disk_log_mux);
        if (w <= 0) continue;
        if (httpd_resp_send_chunk(req, line, (size_t)w) != ESP_OK) break;
    }
    httpd_resp_send_chunk(req, nullptr, 0);
    return ESP_OK;
}

static void httpd_start_once(void) {
    if (s_httpd) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 20;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 2;
    cfg.send_wait_timeout = 2;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        DBG("[disk] httpd_start failed\n");
        s_httpd = nullptr;
        return;
    }
    static const httpd_uri_t routes[] = {
        { "/",          HTTP_GET,    h_root,      nullptr },
        { "/status",    HTTP_GET,    h_status,    nullptr },
        { "/list",      HTTP_GET,    h_list,      nullptr },
        { "/fs/*",      HTTP_GET,    h_fs_get,    nullptr },
        { "/fs/*",      HTTP_PUT,    h_fs_put,    nullptr },
        { "/fs/*",      HTTP_DELETE, h_fs_delete, nullptr },
        { "/device-write", HTTP_POST, h_owner_device, nullptr },
        { "/host-write", HTTP_POST,    h_owner_usb,    nullptr },
        { "/owner/device", HTTP_POST, h_owner_device, nullptr },
        { "/owner/usb", HTTP_POST,    h_owner_usb,    nullptr },
        { "/format",    HTTP_POST,   h_format,      nullptr },
        { "/eject",     HTTP_POST,   h_owner_device, nullptr },
        { "/present",   HTTP_POST,   h_owner_usb,    nullptr },
        { "/type",      HTTP_POST,   h_type,      nullptr },
        { "/type-log",  HTTP_GET,    h_type_log,  nullptr },
        { "/disk-log",  HTTP_GET,    h_disk_log,  nullptr },
        { "/lba/*",     HTTP_GET,    h_lba,       nullptr },
        { "/usb-log",   HTTP_GET,    h_usb_log,   nullptr },
        { "/ota",       HTTP_POST,   h_ota,       nullptr },
        { "/reset",     HTTP_POST,   h_reset,     nullptr },
    };
    for (auto &r : routes) httpd_register_uri_handler(s_httpd, &r);
    DBG("[disk] httpd listening on :80\n");
}

static void mdns_start_once(void) {
    if (s_mdns_up) return;
    if (!MDNS.begin(MDNS_HOSTNAME)) {
        DBG("[disk] mDNS begin failed\n");
        return;
    }
    MDNS.addService("http", "tcp", 80);
    MDNS.addServiceTxt("http", "tcp", "device", "tdongle-s3");
    s_mdns_up = true;
    DBG("[disk] mDNS up as %s.local\n", MDNS_HOSTNAME);
}

// Bring up HTTP + mDNS once WiFi is up. Polled from loop() via a small
// hook; cheaper than another task.
static void wifi_event_cb(WiFiEvent_t ev, WiFiEventInfo_t info) {
    if (ev == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
        httpd_start_once();
        mdns_start_once();
    } else if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        // leave httpd running; mDNS will rebind on next IP
    }
}

void dongle_disk_init(void) {
    s_lock = xSemaphoreCreateMutex();

    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_DATA_FAT,
                                      FAT_PART_LABEL);
    if (!s_part) {
        DBG("[disk] no '%s' partition found -- check partitions_dongle.csv\n", FAT_PART_LABEL);
        return;
    }

    if (wl_mount(s_part, &s_wl) != ESP_OK) {
        DBG("[disk] wl_mount failed\n");
        return;
    }

    s_block_count = (uint32_t)(wl_size(s_wl) / s_block_size);
    // Capture WHY the dongle last reset. If a write test crashed the dongle,
    // this is the field that tells us the cause: ESP_RST_BROWNOUT is a power
    // dip during sustained flash erases; ESP_RST_TASK_WDT means msc_on_write
    // starved the scheduler; ESP_RST_INT_WDT means IRQs were masked too long
    // (likely flash bus); ESP_RST_PANIC is a code defect (stack overflow,
    // assertion). ESP_RST_POWERON / ESP_RST_SW are benign.
    const char *rr;
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:  rr = "POWERON";  break;
        case ESP_RST_EXT:      rr = "EXT";      break;
        case ESP_RST_SW:       rr = "SW";       break;
        case ESP_RST_PANIC:    rr = "PANIC";    break;
        case ESP_RST_INT_WDT:  rr = "INT_WDT";  break;
        case ESP_RST_TASK_WDT: rr = "TASK_WDT"; break;
        case ESP_RST_WDT:      rr = "WDT";      break;
        case ESP_RST_BROWNOUT: rr = "BROWNOUT"; break;
        case ESP_RST_DEEPSLEEP:rr = "DEEPSLEEP";break;
        case ESP_RST_SDIO:     rr = "SDIO";     break;
        default:               rr = "UNKNOWN";  break;
    }
    // One-time capability dump for the CHUSB-tolerance decision: flash chip
    // JEDEC ID (vendor byte tells us if SPI_FLASH_AUTO_SUSPEND is safe to
    // enable -- GD/Winbond/ISSI ok, XMC-C explicitly de-qualified by
    // Espressif) and PSRAM size (0 if absent or unconfigured -- if 0 we
    // can't host a RAM-backed MSC volume).
    {
        uint32_t jedec = 0;
        if (esp_flash_default_chip)
            esp_flash_read_id(esp_flash_default_chip, &jedec);
        const char *vendor;
        switch ((jedec >> 16) & 0xFF) {
            case 0xC8: vendor = "GD";      break;
            case 0xEF: vendor = "Winbond"; break;
            case 0x9D: vendor = "ISSI";    break;
            case 0xC2: vendor = "MXIC";    break;
            case 0x20: vendor = "XMC";     break;
            case 0x68: vendor = "BOYA";    break;
            case 0x85: vendor = "PUYA";    break;
            case 0xCD: vendor = "TH";      break;
            default:   vendor = "unknown"; break;
        }
        size_t psram = 0;
#ifdef CONFIG_SPIRAM
        psram = esp_psram_get_size();
#endif
        disk_logf("hw: flash_jedec=0x%06lx vendor=%s psram=%lu bytes\n",
                  (unsigned long)jedec, vendor, (unsigned long)psram);
    }
    disk_logf("init: reset_reason=%s wl_sector=%u msc_block=%u count=%u total=%lu\n",
              rr,
              (unsigned)wl_sector_size(s_wl), (unsigned)s_block_size,
              (unsigned)s_block_count,
              (unsigned long)((uint64_t)s_block_count * s_block_size));

    s_msc.vendorID("DOSONGLE");      // <=8 chars
    s_msc.productID("DEV-DISK");     // <=16
    s_msc.productRevision("1.0");    // <=4
    s_msc.onStartStop(msc_on_start_stop);
    s_msc.onRead(msc_on_read);
    s_msc.onWrite(msc_on_write);
    s_msc.mediaPresent(true);
    s_msc.isWritable(true);
    s_msc.begin(s_block_count, s_block_size);
    s_msc_present = true;
    s_msc_writable = true;
    owner_set_device_side(false);

    WiFi.onEvent(wifi_event_cb);
    // If WiFi happens to already be up by the time we get here, kick it
    // immediately rather than waiting for an event that won't fire again.
    if (WiFi.status() == WL_CONNECTED) {
        httpd_start_once();
        mdns_start_once();
    }

    DBG("[disk] ffat=%u blocks*%u = %u bytes\n",
                 (unsigned)s_block_count, (unsigned)s_block_size,
                 (unsigned)(s_block_count * s_block_size));
}

void dongle_disk_get_status(dongle_disk_status_t *out) {
    if (!out) return;
    out->mounted        = s_dongle_owns;
    out->msc_present    = s_msc_present;
    out->msc_writable   = s_msc_writable;
    out->owner          = dongle_disk_get_owner();
    out->partition_bytes = s_block_count * s_block_size;
    out->used_bytes     = 0;            // not cheap to compute -- skip for AT
    out->http_ready     = s_httpd != nullptr;
    out->mdns_host      = MDNS_HOSTNAME;
}
