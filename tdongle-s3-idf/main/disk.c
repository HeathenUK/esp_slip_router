/* disk.c -- MSC backing store with first-boot mkfs + write-back cache.
 *
 * Owns the ffat partition. On first boot the partition is blank, so we
 * mount it through the FAT VFS with format_if_mount_failed=true, set
 * the volume label to "DOSONGLE", then unmount the VFS and keep the
 * wear-levelling handle for direct WL access. The wl_handle stays live
 * for the rest of the firmware's runtime.
 *
 * The SCSI read/write callbacks the TinyUSB MSC class invokes
 * (tud_msc_read10_cb / tud_msc_write10_cb) are STRONG implementations
 * here; esp_tinyusb's tusb_msc_storage.c versions are weak-linked by
 * tools/apply_iram_patches.sh so ours win.
 *
 * Why the write-back cache: wear_levelling's natural erase granularity
 * is one WL sector (4 KB on the S3). A FAT/MSC host writes in 512-byte
 * chunks; without batching, each chunk triggers a 4 KB erase+write,
 * which is ~8x more flash wear and ~8x more cache-disabled time than
 * necessary. We cache one WL sector in RAM and evict on sector change,
 * collapsing the common burst-of-small-writes-to-the-same-sector
 * pattern into a single flash op.
 */

#include "disk.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "wear_levelling.h"

#include "tusb.h"
#include "class/msc/msc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define TAG "disk"

/* MSC reports 512-byte logical blocks. The underlying WL sector is
 * larger (4 KB on the S3), and we batch writes into that. */
#define MSC_BLOCK_SIZE   512
#define WB_CACHE_BYTES   4096

#define MSC_VOLUME_LABEL "DOSONGLE"

/* FAT12 geometry for our 8 MB superfloppy. 8 sectors/cluster keeps the
 * cluster count comfortably inside FAT12's 12-bit range and 512 root
 * entries is the conventional choice for a removable FAT volume. */
#define FAT_CLUSTER_SECS 8U
#define FAT_ROOT_ENTRIES 512U

static wl_handle_t s_wl = WL_INVALID_HANDLE;
static uint32_t    s_block_count = 0;   /* in 512-byte logical blocks */
static SemaphoreHandle_t s_io_mutex = NULL;

/* Write-back cache: N slots, each holds one WL sector. The host's
 * typical write pattern (FAT update + data cluster + dir entry) hits
 * 3-4 distinct sectors interleaved; with multiple slots none of those
 * forces a flash eviction on the critical SCSI path. With one slot
 * (the previous design), eviction fired on almost every write, each
 * taking the full ~67 ms erase+program. CHUSB's per-CBW timeout
 * fires below that, causing DOS-side hangs.
 *
 * 4 slots × 4 KB = 16 KB RAM. Free heap is ~225 KB; cost is trivial. */
#define WB_SLOTS 4
typedef struct {
    uint8_t  data[WB_CACHE_BYTES];
    uint32_t sect_base;
    uint64_t last_use_us;   /* esp_timer_get_time() of last hit -- for LRU */
    bool     have;
    bool     dirty;
} wb_slot_t;

static wb_slot_t s_wb[WB_SLOTS];
static uint32_t  s_wb_merge_count = 0;
static uint32_t  s_wb_evict_count = 0;

/* Per-callback counters. Single-writer (TinyUSB task on CPU1), readers
 * (HTTP handler on CPU0) get an atomic uint32_t load; values may be
 * slightly stale but never torn. No lock in the hot path. */
static volatile uint32_t s_cb_tur        = 0;
static volatile uint32_t s_cb_read       = 0;
static volatile uint32_t s_cb_write      = 0;
static volatile uint32_t s_cb_scsi       = 0;
static volatile uint32_t s_cb_start_stop = 0;
static volatile uint32_t s_cb_mount      = 0;
static volatile uint32_t s_cb_umount     = 0;
static volatile uint32_t s_cb_suspend    = 0;
static volatile uint32_t s_cb_resume     = 0;
static volatile uint32_t s_last_lba      = 0;   /* last READ or WRITE LBA */
static volatile uint32_t s_last_size     = 0;
static volatile char     s_last_op       = '-'; /* 'R', 'W', 'T', 'S' */

/* Per-callback latency tracking. Hypothesis: a single slow flash op
 * pushes past CHUSB's per-CBW timeout and the DOS-side state machine
 * aborts. The hot-path cost is two esp_timer_get_time() calls and a
 * compare per callback -- ~150 ns total, well below the noise floor
 * for a SCSI command rate. */
static volatile uint32_t s_max_write_us       = 0;
static volatile uint32_t s_max_write_lba      = 0;
static volatile uint32_t s_slow_write_count   = 0; /* writes > 50 ms */
static volatile uint32_t s_max_read_us        = 0;
#define SLOW_WRITE_THRESHOLD_US 50000U

/* ---- cache helpers (multi-slot) ---- */

/* Flush one specific slot to flash, if it's dirty. */
static esp_err_t wb_flush_slot(wb_slot_t *s) {
    if (!s->have || !s->dirty) return ESP_OK;
    if (s_wl == WL_INVALID_HANDLE) return ESP_ERR_INVALID_STATE;
    size_t sect = wl_sector_size(s_wl);
    if (sect > WB_CACHE_BYTES) return ESP_ERR_INVALID_SIZE;

    esp_err_t e = wl_erase_range(s_wl, s->sect_base, sect);
    if (e != ESP_OK) {
        disk_logf("[wb] flush erase fail base=0x%x sect=%u err=%d (%s)",
                  (unsigned)s->sect_base, (unsigned)sect, e, esp_err_to_name(e));
        return e;
    }
    e = wl_write(s_wl, s->sect_base, s->data, sect);
    if (e != ESP_OK) {
        disk_logf("[wb] flush write fail base=0x%x sect=%u err=%d (%s)",
                  (unsigned)s->sect_base, (unsigned)sect, e, esp_err_to_name(e));
        return e;
    }
    s->dirty = false;
    return ESP_OK;
}

static void wb_slot_invalidate(wb_slot_t *s) {
    s->have      = false;
    s->dirty     = false;
    s->sect_base = 0;
}

/* Find a slot that currently holds `sect_base`, or NULL. */
static wb_slot_t *wb_find(uint32_t sect_base) {
    for (int i = 0; i < WB_SLOTS; ++i) {
        if (s_wb[i].have && s_wb[i].sect_base == sect_base) return &s_wb[i];
    }
    return NULL;
}

/* Allocate a slot to hold `sect_base`: prefer an empty slot; otherwise
 * pick the least-recently-used one (flushing it if dirty). The chosen
 * slot is left empty -- caller is responsible for populating it. */
static esp_err_t wb_alloc_slot(uint32_t sect_base, wb_slot_t **out) {
    (void)sect_base;
    /* Empty slot wins. */
    for (int i = 0; i < WB_SLOTS; ++i) {
        if (!s_wb[i].have) { *out = &s_wb[i]; return ESP_OK; }
    }
    /* Evict the LRU (smallest last_use_us). */
    wb_slot_t *victim = &s_wb[0];
    for (int i = 1; i < WB_SLOTS; ++i) {
        if (s_wb[i].last_use_us < victim->last_use_us) victim = &s_wb[i];
    }
    esp_err_t e = wb_flush_slot(victim);
    if (e != ESP_OK) return e;
    s_wb_evict_count++;
    wb_slot_invalidate(victim);
    *out = victim;
    return ESP_OK;
}

/* Read one full WL sector from flash into the given slot. */
static esp_err_t wb_load_into_slot(wb_slot_t *s, uint32_t sect_base) {
    size_t sect = wl_sector_size(s_wl);
    if (sect > WB_CACHE_BYTES) return ESP_ERR_INVALID_SIZE;
    esp_err_t e = wl_read(s_wl, sect_base, s->data, sect);
    if (e != ESP_OK) {
        disk_logf("[wb] load fail base=0x%x sect=%u err=%d (%s)",
                  (unsigned)sect_base, (unsigned)sect, e, esp_err_to_name(e));
        return e;
    }
    s->have      = true;
    s->dirty     = false;
    s->sect_base = sect_base;
    return ESP_OK;
}

/* Flush all dirty slots. Called on eject (START_STOP_UNIT with eject=1). */
static esp_err_t wb_flush_all(void) {
    for (int i = 0; i < WB_SLOTS; ++i) {
        esp_err_t e = wb_flush_slot(&s_wb[i]);
        if (e != ESP_OK) return e;
    }
    return ESP_OK;
}

/* ---- format detection + first-boot mkfs ---- */

/* The BPB at sector 0 has a FAT12/16 signature at offset 510 (0x55AA)
 * and a small set of values that have to be sensible. We only treat
 * the partition as needing reformat when this check fails -- so user
 * data on a previously-good volume isn't wiped by a firmware update. */
static bool partition_looks_like_fat(void) {
    uint8_t boot[512];
    esp_err_t e = wl_read(s_wl, 0, boot, sizeof(boot));
    if (e != ESP_OK) return false;
    if (boot[510] != 0x55 || boot[511] != 0xAA) return false;
    /* BPB_BytsPerSec = 512 (anything else trips this) */
    uint16_t bps = (uint16_t)boot[11] | ((uint16_t)boot[12] << 8);
    if (bps != 512) return false;
    /* Reserved-sector count > 0 and BPB_NumFATs in {1,2}. */
    uint16_t resv = (uint16_t)boot[14] | ((uint16_t)boot[15] << 8);
    if (resv == 0) return false;
    if (boot[16] != 1 && boot[16] != 2) return false;
    return true;
}

/* Byte-level WL helpers used when we hand-build the BPB / FATs / root
 * directory. Working sector-aligned writes go through a read-modify-
 * write of the underlying WL sector since wl_erase_range erases the
 * whole sector. */

static inline void put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static inline void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static esp_err_t raw_write_bytes(uint32_t off, const void *buf, size_t len) {
    if (s_wl == WL_INVALID_HANDLE) return ESP_ERR_INVALID_STATE;
    size_t sect = wl_sector_size(s_wl);
    static uint8_t sbuf[4096];
    if (sect > sizeof sbuf || (sect & (sect - 1)) != 0) return ESP_ERR_INVALID_SIZE;

    const uint8_t *src = (const uint8_t *)buf;
    while (len > 0) {
        size_t sect_off  = off & (sect - 1);
        size_t sect_base = off - sect_off;
        size_t chunk     = sect - sect_off;
        if (chunk > len) chunk = len;
        if (wl_read(s_wl, sect_base, sbuf, sect) != ESP_OK) return ESP_FAIL;
        memcpy(sbuf + sect_off, src, chunk);
        if (wl_erase_range(s_wl, sect_base, sect) != ESP_OK) return ESP_FAIL;
        if (wl_write(s_wl, sect_base, sbuf, sect) != ESP_OK) return ESP_FAIL;
        off += chunk; src += chunk; len -= chunk;
    }
    return ESP_OK;
}

static esp_err_t raw_zero_bytes(uint32_t off, size_t len) {
    static uint8_t zero[4096];
    size_t sect = wl_sector_size(s_wl);
    if (sect > sizeof zero || (sect & (sect - 1)) != 0) return ESP_ERR_INVALID_SIZE;
    memset(zero, 0, sect);

    while (len > 0) {
        if ((off & (sect - 1)) == 0 && len >= sect) {
            if (wl_erase_range(s_wl, off, sect) != ESP_OK) return ESP_FAIL;
            if (wl_write(s_wl, off, zero, sect) != ESP_OK) return ESP_FAIL;
            off += sect; len -= sect;
            continue;
        }
        size_t align = sect - (off & (sect - 1));
        size_t n = len < align ? len : align;
        esp_err_t err = raw_write_bytes(off, zero, n);
        if (err != ESP_OK) return err;
        off += n; len -= n;
    }
    return ESP_OK;
}

/* Hand-build a FAT12 superfloppy at the start of the partition.
 * Ported from the arduino-esp32 build (dongle_disk.cpp) which proved
 * out on real DOS hosts; FatFs's f_mkfs produced a 4096-byte-sector
 * BPB that conflicted with our 512-byte MSC LBAs, so we own the
 * layout ourselves. Drive number 0x00 = removable, no MBR. */
static esp_err_t format_partition_superfloppy(void) {
    uint32_t total_secs   = s_block_count;
    uint16_t reserved     = 1;
    uint8_t  fats         = 2;
    uint16_t root_entries = FAT_ROOT_ENTRIES;
    uint32_t root_secs    = (root_entries * 32U + MSC_BLOCK_SIZE - 1U) / MSC_BLOCK_SIZE;
    uint8_t  spc          = FAT_CLUSTER_SECS;
    uint16_t spf          = 1;

    /* Iteratively solve for sectors-per-FAT (FAT size depends on
     * cluster count which depends on FAT size). Converges in a few
     * iterations for any sane volume. */
    for (int i = 0; i < 8; ++i) {
        uint32_t data_secs = total_secs - reserved - root_secs - (uint32_t)fats * spf;
        uint32_t clusters  = data_secs / spc;
        uint32_t fat_bytes = ((clusters + 2U) * 3U + 1U) / 2U;
        uint16_t need_spf  = (uint16_t)((fat_bytes + MSC_BLOCK_SIZE - 1U) / MSC_BLOCK_SIZE);
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
    else                       put32(sec + 32, total_secs);
    sec[21] = 0xF8;
    put16(sec + 22, spf);
    put16(sec + 24, 32);     /* sectors/track -- conventional geometry only */
    put16(sec + 26, 64);     /* heads */
    /* drive_num = 0x00: removable / superfloppy. 0x80 makes macOS
     * interpret the 0x55AA as an MBR signature and look for partition
     * entries in the BPB padding, find none, and refuse to mount. */
    sec[36] = 0x00;
    sec[38] = 0x29;
    put32(sec + 39, 0x444F5301UL);
    memcpy(sec + 43, MSC_VOLUME_LABEL "   ", 11);  /* 11-byte label, space-padded */
    memcpy(sec + 54, "FAT12   ", 8);
    sec[510] = 0x55; sec[511] = 0xAA;
    esp_err_t err = raw_write_bytes(0, sec, sizeof sec);
    if (err != ESP_OK) return err;

    /* FAT tables: first two entries seed (0xFFFFF8 + 0xFFFFFF for FAT12). */
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

    /* Root directory: zero out, then write the volume label entry. */
    uint32_t root = fat0 + (uint32_t)fats * spf * MSC_BLOCK_SIZE;
    err = raw_zero_bytes(root, root_secs * MSC_BLOCK_SIZE);
    if (err != ESP_OK) return err;

    uint8_t label[32];
    memset(label, 0, sizeof label);
    memcpy(label, MSC_VOLUME_LABEL "   ", 11);
    label[11] = 0x08;   /* ATTR_VOLUME_ID */
    return raw_write_bytes(root, label, sizeof label);
}

/* ---- public entry ---- */

esp_err_t disk_init(void) {
    if (s_wl != WL_INVALID_HANDLE) return ESP_OK; /* idempotent */

    s_io_mutex = xSemaphoreCreateMutex();
    if (!s_io_mutex) return ESP_ERR_NO_MEM;

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ffat");
    if (!part) return ESP_ERR_NOT_FOUND;
    esp_err_t err = wl_mount(part, &s_wl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wl_mount: %s", esp_err_to_name(err));
        return err;
    }

    size_t total_bytes = wl_size(s_wl);
    s_block_count = (uint32_t)(total_bytes / MSC_BLOCK_SIZE);

    /* Format only if the existing FAT looks unsalvageable -- a stray
     * earlier write may have left a partial MBR with bogus partition
     * entries that confuse host filesystems. We don't preserve broken
     * volumes; on the other hand, a healthy FAT survives across
     * firmware updates and the user's files don't get wiped. */
    if (!partition_looks_like_fat()) {
        disk_logf("disk: no valid FAT found, formatting as superfloppy w/ label " MSC_VOLUME_LABEL);
        err = format_partition_superfloppy();
        if (err != ESP_OK) {
            disk_logf("disk: format failed err=%s", esp_err_to_name(err));
            return err;
        }
    }

    disk_logf("disk: wl ok, %u x 512B sectors (%u WL sectors of %u B), label=" MSC_VOLUME_LABEL,
              (unsigned)s_block_count,
              (unsigned)(total_bytes / wl_sector_size(s_wl)),
              (unsigned)wl_sector_size(s_wl));

    /* Dump the first 16 bytes of sector 0 + the FAT signature so we
     * can verify the BPB layout. */
    uint8_t s0[512];
    if (wl_read(s_wl, 0, s0, sizeof(s0)) == ESP_OK) {
        disk_logf("disk: sec0[0..15]=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  sig@510=%02x %02x",
                  s0[0],s0[1],s0[2],s0[3],s0[4],s0[5],s0[6],s0[7],
                  s0[8],s0[9],s0[10],s0[11],s0[12],s0[13],s0[14],s0[15],
                  s0[510], s0[511]);
        disk_logf("disk: BPB BytsPerSec=%u SecPerClus=%u RsvdSecCnt=%u NumFATs=%u",
                  (unsigned)(s0[11] | (s0[12]<<8)), (unsigned)s0[13],
                  (unsigned)(s0[14] | (s0[15]<<8)), (unsigned)s0[16]);
    } else {
        disk_logf("disk: sec0 read failed");
    }
    return ESP_OK;
}

/* ---- tud_msc_*_cb (strong overrides) ---- */

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;
    memcpy(vendor_id,  "DOSongle",        8);
    memcpy(product_id, "T-Dongle S3 Disk", 16);
    memcpy(product_rev,"1.0\0",            4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    s_cb_tur++;
    s_last_op = 'T';
    return s_wl != WL_INVALID_HANDLE;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void)lun;
    *block_count = s_block_count;
    *block_size  = MSC_BLOCK_SIZE;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
    (void)lun; (void)power_condition; (void)start;
    s_cb_start_stop++;
    s_last_op = 'S';
    if (load_eject && !start) {
        /* Eject: flush ALL dirty slots so the host doesn't lose any
         * data still in RAM if we reset before the next eviction. */
        xSemaphoreTake(s_io_mutex, portMAX_DELAY);
        esp_err_t e = wb_flush_all();
        xSemaphoreGive(s_io_mutex);
        if (e != ESP_OK) {
            disk_logf("[wb] eject flush err=%d (%s)", e, esp_err_to_name(e));
        }
    }
    return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    (void)lun;
    s_cb_read++;
    s_last_op = 'R';
    s_last_lba = lba;
    s_last_size = bufsize;
    int64_t t0 = esp_timer_get_time();
    if (s_wl == WL_INVALID_HANDLE) return -1;

    uint32_t addr = lba * (uint32_t)MSC_BLOCK_SIZE + offset;
    if ((uint64_t)addr + bufsize > (uint64_t)s_block_count * MSC_BLOCK_SIZE) {
        disk_logf("[msc] READ OOB lba=%u off=%u bsz=%u",
                  (unsigned)lba, (unsigned)offset, (unsigned)bufsize);
        return -1;
    }

    xSemaphoreTake(s_io_mutex, portMAX_DELAY);
    size_t sect = wl_sector_size(s_wl);
    size_t remaining = bufsize;
    uint8_t *dst = (uint8_t *)buffer;
    while (remaining > 0) {
        size_t sect_off  = addr & (sect - 1);
        size_t sect_base = addr - sect_off;
        size_t chunk     = sect - sect_off;
        if (chunk > remaining) chunk = remaining;

        wb_slot_t *hit = wb_find(sect_base);
        if (hit) {
            /* Cache hit: dirty data may not be on flash yet -- always
             * serve from RAM so the host sees a consistent view. */
            memcpy(dst, hit->data + sect_off, chunk);
            hit->last_use_us = (uint64_t)esp_timer_get_time();
        } else {
            esp_err_t e = wl_read(s_wl, addr, dst, chunk);
            if (e != ESP_OK) {
                disk_logf("[msc] read wl_read fail addr=0x%x bsz=%u err=%d (%s)",
                          (unsigned)addr, (unsigned)chunk, e, esp_err_to_name(e));
                xSemaphoreGive(s_io_mutex);
                return -1;
            }
        }
        addr += chunk; dst += chunk; remaining -= chunk;
    }
    xSemaphoreGive(s_io_mutex);
    {
        uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
        if (dt > s_max_read_us) s_max_read_us = dt;
    }
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    (void)lun;
    s_cb_write++;
    s_last_op = 'W';
    s_last_lba = lba;
    s_last_size = bufsize;
    int64_t t0 = esp_timer_get_time();
    if (s_wl == WL_INVALID_HANDLE) return -1;

    uint32_t addr = lba * (uint32_t)MSC_BLOCK_SIZE + offset;
    if ((uint64_t)addr + bufsize > (uint64_t)s_block_count * MSC_BLOCK_SIZE) {
        disk_logf("[msc] WRITE OOB lba=%u off=%u bsz=%u",
                  (unsigned)lba, (unsigned)offset, (unsigned)bufsize);
        return -1;
    }

    xSemaphoreTake(s_io_mutex, portMAX_DELAY);
    size_t sect = wl_sector_size(s_wl);
    if (sect > WB_CACHE_BYTES) {
        disk_logf("[wb] cache too small: sect=%u cache=%u", (unsigned)sect, (unsigned)WB_CACHE_BYTES);
        xSemaphoreGive(s_io_mutex);
        return -1;
    }

    size_t remaining = bufsize;
    uint8_t *src = buffer;
    while (remaining > 0) {
        size_t sect_off  = addr & (sect - 1);
        size_t sect_base = addr - sect_off;
        size_t chunk     = sect - sect_off;
        if (chunk > remaining) chunk = remaining;

        wb_slot_t *slot = wb_find(sect_base);
        if (slot) {
            /* Hit: pure RAM merge, no flash op. The eviction (and the
             * 67 ms erase+program latency that goes with it) was
             * deferred to when this slot eventually gets reused. */
            memcpy(slot->data + sect_off, src, chunk);
            slot->dirty = true;
            slot->last_use_us = (uint64_t)esp_timer_get_time();
            s_wb_merge_count++;
        } else {
            /* Miss. Allocate a slot (may flush a dirty victim). */
            esp_err_t e = wb_alloc_slot(sect_base, &slot);
            if (e != ESP_OK) {
                xSemaphoreGive(s_io_mutex);
                return -1;
            }
            if (sect_off == 0 && chunk == sect) {
                /* Full-sector overwrite: skip the read. */
                memcpy(slot->data, src, sect);
                slot->have      = true;
                slot->dirty     = true;
                slot->sect_base = sect_base;
            } else {
                e = wb_load_into_slot(slot, sect_base);
                if (e != ESP_OK) {
                    xSemaphoreGive(s_io_mutex);
                    return -1;
                }
                memcpy(slot->data + sect_off, src, chunk);
                slot->dirty = true;
            }
            slot->last_use_us = (uint64_t)esp_timer_get_time();
        }

        addr += chunk; src += chunk; remaining -= chunk;
    }
    xSemaphoreGive(s_io_mutex);
    {
        uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
        if (dt > s_max_write_us) {
            s_max_write_us  = dt;
            s_max_write_lba = lba;
        }
        if (dt > SLOW_WRITE_THRESHOLD_US) s_slow_write_count++;
    }
    return (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize) {
    (void)lun; (void)buffer; (void)bufsize;
    s_cb_scsi++;
    /* Unsupported SCSI command. Setting sense data isn't strictly
     * required here -- TinyUSB stalls the endpoint on negative
     * return -- but mirroring the arduino-esp32 path keeps host
     * driver behavior consistent. */
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

/* ---- USB bus event callbacks ----
 *
 * These only fire on state changes (host configures the device, host
 * disconnects/re-enumerates, suspend/resume on the bus). Not on the
 * hot transfer path, so disk_logf cost is irrelevant. The umount/
 * suspend events are the smoking-gun signals for "host gave up": if
 * we see those during a DISKTEST run, the dongle stayed alive but
 * the host disconnected us. */

void tud_mount_cb(void) {
    s_cb_mount++;
    disk_logf("[usb] mount #%u (host configured device)", (unsigned)s_cb_mount);
}

void tud_umount_cb(void) {
    s_cb_umount++;
    disk_logf("[usb] UMOUNT #%u  last=%c lba=%u sz=%u  r=%u w=%u tur=%u",
              (unsigned)s_cb_umount, s_last_op,
              (unsigned)s_last_lba, (unsigned)s_last_size,
              (unsigned)s_cb_read, (unsigned)s_cb_write, (unsigned)s_cb_tur);
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    s_cb_suspend++;
    disk_logf("[usb] suspend #%u  last=%c lba=%u sz=%u  r=%u w=%u",
              (unsigned)s_cb_suspend, s_last_op,
              (unsigned)s_last_lba, (unsigned)s_last_size,
              (unsigned)s_cb_read, (unsigned)s_cb_write);
}

void tud_resume_cb(void) {
    s_cb_resume++;
    disk_logf("[usb] resume #%u", (unsigned)s_cb_resume);
}

/* ---- JSON snapshot for /usb-stats ---- */

size_t disk_stats_json(char *out, size_t cap) {
    unsigned dirty = 0;
    for (int i = 0; i < WB_SLOTS; ++i) if (s_wb[i].dirty) dirty++;
    int n = snprintf(out, cap,
        "{\"counts\":{\"tur\":%u,\"read\":%u,\"write\":%u,\"scsi\":%u,\"start_stop\":%u"
        ",\"mount\":%u,\"umount\":%u,\"suspend\":%u,\"resume\":%u}"
        ",\"last\":{\"op\":\"%c\",\"lba\":%u,\"size\":%u}"
        ",\"timing_us\":{\"max_write\":%u,\"max_write_lba\":%u,\"slow_writes\":%u,\"max_read\":%u}"
        ",\"wb\":{\"slots\":%u,\"merges\":%u,\"evicts\":%u,\"dirty_slots\":%u}}\n",
        (unsigned)s_cb_tur, (unsigned)s_cb_read, (unsigned)s_cb_write,
        (unsigned)s_cb_scsi, (unsigned)s_cb_start_stop,
        (unsigned)s_cb_mount, (unsigned)s_cb_umount,
        (unsigned)s_cb_suspend, (unsigned)s_cb_resume,
        s_last_op, (unsigned)s_last_lba, (unsigned)s_last_size,
        (unsigned)s_max_write_us, (unsigned)s_max_write_lba,
        (unsigned)s_slow_write_count, (unsigned)s_max_read_us,
        (unsigned)WB_SLOTS, (unsigned)s_wb_merge_count, (unsigned)s_wb_evict_count,
        dirty);
    return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}
