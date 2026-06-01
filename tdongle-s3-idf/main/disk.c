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

/* Write-back cache with ASYNC eviction.
 *
 * SCSI WRITE callbacks no longer block on flash. The hot path is just:
 *   1. lock mutex
 *   2. find or allocate a slot (memory-only)
 *   3. memcpy host data into the slot
 *   4. mark slot DIRTY
 *   5. unlock mutex
 *   6. signal worker semaphore
 * Total: ~1-10 microseconds per write callback.
 *
 * The wb_worker task (pinned to CPU1, priority just below TinyUSB)
 * drains DIRTY slots to flash in the background. It snapshots each
 * slot under brief mutex into a worker-private buffer, then releases
 * the mutex before the slow wl_erase + wl_write (~67 ms) so the SCSI
 * path stays unblocked.
 *
 * Slot states form a small state machine:
 *
 *   EMPTY  --writer-->  DIRTY  --worker-snapshot-->  FLUSHING
 *                          ^                              |
 *                          | (writer hits FLUSHING slot   v
 *                          |  -> updates data in-place,  CLEAN
 *                          |  marks DIRTY again)          |
 *                          +----------------------------- + (re-dirty before commit)
 *
 * Slot count is the headroom for the host to burst writes faster
 * than the worker can drain to flash (worker rate ~15 sectors/sec).
 *
 * SIZING HISTORY: 32 (128 KB) -> 16 (64 KB) -> 8 (32 KB) -> 10 (40 KB)
 *                  -> 16 (64 KB).
 *
 * Returned to 16 slots after WB_SLOTS=10 started crashing during
 * sustained MSC writes from CHUSB. The dial-path heap squeeze that
 * forced us off 16 originally is now fixed by (a) pre-allocating the
 * modem stream buffers at modem_init (boot, when heap is highest)
 * rather than first-dial, (b) halving TCP_WND_DEFAULT to 32 KB, and
 * (c) removing the spurious setsockopt(SO_RCVBUF, 32 KB) override
 * that was layering on top of TCP_WND. With those landed, the
 * dial-time low-water heap with 16 slots is comfortable, and CHUSB
 * gets its full historical 64 KB burst headroom back. */
#define WB_SLOTS 16

typedef enum {
    SLOT_EMPTY    = 0,
    SLOT_CLEAN    = 1,   /* data matches flash; safe to evict */
    SLOT_DIRTY    = 2,   /* data differs; worker needs to flush */
    SLOT_FLUSHING = 3,   /* worker is currently committing */
} slot_state_t;

typedef struct {
    uint8_t  data[WB_CACHE_BYTES];
    uint32_t sect_base;
    uint64_t last_use_us;       /* for LRU when evicting CLEAN slots */
    slot_state_t state;
} wb_slot_t;

static wb_slot_t s_wb[WB_SLOTS];
static uint8_t   s_wb_flush_buf[WB_CACHE_BYTES];  /* worker's snapshot buf */
static uint32_t  s_wb_merge_count = 0;
static uint32_t  s_wb_evict_count = 0;            /* dirty slot displacements */
static uint32_t  s_wb_async_flush_count = 0;      /* worker-completed flushes */
static SemaphoreHandle_t s_wb_dirty_sem = NULL;    /* worker wakes on this */
static SemaphoreHandle_t s_wb_clean_sem = NULL;    /* writer waits on this when all slots dirty */
static TaskHandle_t      s_wb_worker_task = NULL;

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
static volatile uint32_t s_max_alloc_wait_us  = 0; /* time blocked waiting on clean_sem */
#define SLOW_WRITE_THRESHOLD_US 50000U

/* Ownership state for the raw-FAT HTTP path. When s_dongle_owns is
 * true, tud_msc_write10_cb rejects new writes with -1 (host gets a
 * sense-data error). Reads still pass through the cache + flash like
 * normal. */
static volatile bool    s_dongle_owns = false;
static volatile int64_t s_last_msc_write_us = 0;

/* ---- cache helpers (multi-slot, async) ----
 *
 * IMPORTANT: every function in this block runs with s_io_mutex held
 * UNLESS it explicitly drops it (only the worker does that, for the
 * actual wl_erase + wl_write calls). State transitions are valid only
 * under the mutex. */

static void wb_slot_invalidate(wb_slot_t *s) {
    s->state     = SLOT_EMPTY;
    s->sect_base = 0;
}

/* Find a slot that currently holds `sect_base`, or NULL. Slots in
 * FLUSHING state still count as holding their sect_base -- the data
 * in them is fresh, the worker is just persisting it. */
static wb_slot_t *wb_find(uint32_t sect_base) {
    for (int i = 0; i < WB_SLOTS; ++i) {
        if (s_wb[i].state != SLOT_EMPTY && s_wb[i].sect_base == sect_base)
            return &s_wb[i];
    }
    return NULL;
}

/* Allocate a slot. Preference order:
 *   1. EMPTY slot
 *   2. CLEAN slot, LRU (no flush needed)
 *   3. block on s_wb_clean_sem until worker promotes a slot to CLEAN
 * Returns the slot in EMPTY state ready to be populated.
 * MUST be called with s_io_mutex held; may temporarily release it to
 * wait on the clean-slot semaphore. */
static esp_err_t wb_alloc_slot(wb_slot_t **out) {
    int64_t wait_t0 = esp_timer_get_time();
    bool waited = false;
    for (int spin = 0; spin < 100; ++spin) {     /* runaway guard */
        wb_slot_t *empty_slot = NULL;
        wb_slot_t *clean_lru  = NULL;
        for (int i = 0; i < WB_SLOTS; ++i) {
            if (s_wb[i].state == SLOT_EMPTY) {
                empty_slot = &s_wb[i]; break;
            }
            if (s_wb[i].state == SLOT_CLEAN) {
                if (!clean_lru || s_wb[i].last_use_us < clean_lru->last_use_us)
                    clean_lru = &s_wb[i];
            }
        }
        if (empty_slot) {
            *out = empty_slot;
            goto done;
        }
        if (clean_lru) {
            s_wb_evict_count++;          /* displacing a CLEAN slot; no flash op */
            wb_slot_invalidate(clean_lru);
            *out = clean_lru;
            goto done;
        }
        /* All slots are DIRTY or FLUSHING. Wake the worker (if it
         * isn't already busy) and wait for it to promote one to
         * CLEAN. Drop the mutex while we wait so the worker can
         * actually progress. */
        waited = true;
        xSemaphoreGive(s_wb_dirty_sem);
        xSemaphoreGive(s_io_mutex);
        xSemaphoreTake(s_wb_clean_sem, pdMS_TO_TICKS(200));
        xSemaphoreTake(s_io_mutex, portMAX_DELAY);
    }
    disk_logf("[wb] alloc_slot timeout -- all slots stuck?");
    return ESP_ERR_TIMEOUT;
done:
    if (waited) {
        uint32_t dt = (uint32_t)(esp_timer_get_time() - wait_t0);
        if (dt > s_max_alloc_wait_us) s_max_alloc_wait_us = dt;
    }
    return ESP_OK;
}

/* Read one full WL sector from flash into the given slot. Caller must
 * hold s_io_mutex; we drop and reacquire it around the wl_read since
 * the read can take ~ms. */
static esp_err_t wb_load_into_slot(wb_slot_t *s, uint32_t sect_base) {
    size_t sect = wl_sector_size(s_wl);
    if (sect > WB_CACHE_BYTES) return ESP_ERR_INVALID_SIZE;
    /* Mark the slot CLEAN with its target sect_base so concurrent
     * lookups (after we drop the mutex) find it -- the data will be
     * loaded by the time we return. */
    s->sect_base = sect_base;
    s->state     = SLOT_CLEAN;
    xSemaphoreGive(s_io_mutex);
    esp_err_t e = wl_read(s_wl, sect_base, s->data, sect);
    xSemaphoreTake(s_io_mutex, portMAX_DELAY);
    if (e != ESP_OK) {
        disk_logf("[wb] load fail base=0x%x sect=%u err=%d (%s)",
                  (unsigned)sect_base, (unsigned)sect, e, esp_err_to_name(e));
        wb_slot_invalidate(s);
        return e;
    }
    return ESP_OK;
}

/* The worker task. Runs forever, sleeps on s_wb_dirty_sem until a
 * writer signals there's work. */
static void wb_worker_task(void *arg) {
    (void)arg;
    while (1) {
        xSemaphoreTake(s_wb_dirty_sem, portMAX_DELAY);

        while (1) {
            /* Find the oldest DIRTY slot. */
            xSemaphoreTake(s_io_mutex, portMAX_DELAY);
            wb_slot_t *victim = NULL;
            for (int i = 0; i < WB_SLOTS; ++i) {
                if (s_wb[i].state == SLOT_DIRTY) {
                    if (!victim || s_wb[i].last_use_us < victim->last_use_us)
                        victim = &s_wb[i];
                }
            }
            if (!victim) {
                xSemaphoreGive(s_io_mutex);
                break;        /* go back to sleep */
            }
            /* Snapshot the data + mark FLUSHING so the writer knows
             * the slot is in transit. If the writer touches the slot
             * before we finish, it'll mark it DIRTY again -- we'll
             * see that on completion and re-flush in the next pass. */
            uint32_t base = victim->sect_base;
            memcpy(s_wb_flush_buf, victim->data, WB_CACHE_BYTES);
            victim->state = SLOT_FLUSHING;
            xSemaphoreGive(s_io_mutex);

            /* Slow flash op -- ~67 ms -- runs OUTSIDE the mutex. */
            size_t sect = wl_sector_size(s_wl);
            esp_err_t e = wl_erase_range(s_wl, base, sect);
            if (e == ESP_OK)
                e = wl_write(s_wl, base, s_wb_flush_buf, sect);

            xSemaphoreTake(s_io_mutex, portMAX_DELAY);
            if (e != ESP_OK) {
                disk_logf("[wb] async flush fail base=0x%x err=%d (%s)",
                          (unsigned)base, e, esp_err_to_name(e));
                /* Leave slot DIRTY so we retry. */
                victim->state = SLOT_DIRTY;
            } else if (victim->state == SLOT_FLUSHING) {
                /* No writer touched it during the flush: data on flash
                 * matches data in slot; promote to CLEAN. */
                victim->state = SLOT_CLEAN;
                s_wb_async_flush_count++;
                xSemaphoreGive(s_wb_clean_sem);
            }
            /* else: writer re-marked DIRTY mid-flush; loop will pick
             * it up again next iteration. */
            xSemaphoreGive(s_io_mutex);
        }
    }
}

/* Invalidate every cache slot (force EMPTY). Used after the raw-FAT
 * HTTP path mutates the volume so subsequent MSC reads pick up the
 * new flash contents instead of stale cache data. Caller holds
 * s_io_mutex. */
static void wb_invalidate_all(void) {
    for (int i = 0; i < WB_SLOTS; ++i) wb_slot_invalidate(&s_wb[i]);
}

/* Synchronously drain every dirty slot to flash. Called on eject so
 * we don't lose data if the host pulls the device. Caller must hold
 * s_io_mutex. The wl ops happen with the mutex released. */
static esp_err_t wb_flush_all_sync(void) {
    for (int i = 0; i < WB_SLOTS; ++i) {
        if (s_wb[i].state != SLOT_DIRTY && s_wb[i].state != SLOT_FLUSHING) continue;
        /* Wait for any worker-in-flight on this slot to complete. */
        while (s_wb[i].state == SLOT_FLUSHING) {
            xSemaphoreGive(s_io_mutex);
            vTaskDelay(pdMS_TO_TICKS(5));
            xSemaphoreTake(s_io_mutex, portMAX_DELAY);
        }
        if (s_wb[i].state != SLOT_DIRTY) continue;
        uint32_t base = s_wb[i].sect_base;
        memcpy(s_wb_flush_buf, s_wb[i].data, WB_CACHE_BYTES);
        s_wb[i].state = SLOT_FLUSHING;
        xSemaphoreGive(s_io_mutex);
        size_t sect = wl_sector_size(s_wl);
        esp_err_t e = wl_erase_range(s_wl, base, sect);
        if (e == ESP_OK) e = wl_write(s_wl, base, s_wb_flush_buf, sect);
        xSemaphoreTake(s_io_mutex, portMAX_DELAY);
        if (e != ESP_OK) {
            s_wb[i].state = SLOT_DIRTY;
            return e;
        }
        if (s_wb[i].state == SLOT_FLUSHING)
            s_wb[i].state = SLOT_CLEAN;
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

    /* Counting semaphores so multiple gives between worker passes
     * (e.g., back-to-back writes) don't pile up infinitely; cap at
     * WB_SLOTS since that's the worst case interesting. */
    s_wb_dirty_sem = xSemaphoreCreateCounting(WB_SLOTS, 0);
    s_wb_clean_sem = xSemaphoreCreateCounting(WB_SLOTS, 0);
    if (!s_wb_dirty_sem || !s_wb_clean_sem) return ESP_ERR_NO_MEM;

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

    /* Spawn the async write-back worker. Pinned to CPU1 (same as
     * TinyUSB) with priority below it so MSC callbacks preempt the
     * worker if both want CPU1 at once. Stack: the wl ops + a single
     * snapshot buffer don't need much. */
    BaseType_t bx = xTaskCreatePinnedToCore(
        wb_worker_task, "wb_worker", 4096, NULL,
        17 /* TinyUSB is 18 */, &s_wb_worker_task, 1 /* CPU1 */);
    if (bx != pdPASS) return ESP_ERR_NO_MEM;

    disk_logf("disk: wl ok, %u x 512B sectors (%u WL sectors of %u B), label=" MSC_VOLUME_LABEL ", slots=%u async",
              (unsigned)s_block_count,
              (unsigned)(total_bytes / wl_sector_size(s_wl)),
              (unsigned)wl_sector_size(s_wl),
              (unsigned)WB_SLOTS);

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
        /* Eject: synchronously drain ALL dirty slots so the host
         * doesn't lose any data still in RAM if we reset before the
         * worker would have flushed them. */
        xSemaphoreTake(s_io_mutex, portMAX_DELAY);
        esp_err_t e = wb_flush_all_sync();
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
             * serve from RAM so the host sees a consistent view.
             * memcpy is safe under mutex; hit pointer stays valid. */
            memcpy(dst, hit->data + sect_off, chunk);
            hit->last_use_us = (uint64_t)esp_timer_get_time();
        } else {
            /* Miss: pre-warm the cache. Allocate a slot, load the
             * FULL 4 KB sector (even if the host only asked for
             * part of it), then memcpy the requested portion out.
             *
             * Why: typical host pattern is read-modify-write on
             * FAT/dir sectors. Without pre-warm, the subsequent
             * write misses the cache and triggers wb_load_into_slot,
             * whose wl_read serializes against any in-flight worker
             * erase on the SPI bus (~67 ms). With pre-warm the
             * write finds a cached slot and merges in microseconds.
             *
             * The mutex is dropped around wl_read so concurrent
             * ops don't pile up. The slot is left CLEAN (data
             * matches flash). */
            wb_slot_t *new_slot = NULL;
            esp_err_t e = wb_alloc_slot(&new_slot);
            if (e != ESP_OK) {
                /* Fallback: direct read, no caching. */
                xSemaphoreGive(s_io_mutex);
                e = wl_read(s_wl, addr, dst, chunk);
                xSemaphoreTake(s_io_mutex, portMAX_DELAY);
                if (e != ESP_OK) {
                    disk_logf("[msc] read wl_read fail addr=0x%x bsz=%u err=%d (%s)",
                              (unsigned)addr, (unsigned)chunk, e, esp_err_to_name(e));
                    xSemaphoreGive(s_io_mutex);
                    return -1;
                }
            } else {
                new_slot->sect_base   = sect_base;
                new_slot->state       = SLOT_CLEAN;
                new_slot->last_use_us = (uint64_t)esp_timer_get_time();
                xSemaphoreGive(s_io_mutex);
                e = wl_read(s_wl, sect_base, new_slot->data, sect);
                xSemaphoreTake(s_io_mutex, portMAX_DELAY);
                if (e != ESP_OK) {
                    disk_logf("[msc] read prewarm fail base=0x%x err=%d (%s)",
                              (unsigned)sect_base, e, esp_err_to_name(e));
                    /* Don't strand a partly-populated slot in the cache. */
                    if (new_slot->state == SLOT_CLEAN && new_slot->sect_base == sect_base)
                        wb_slot_invalidate(new_slot);
                    xSemaphoreGive(s_io_mutex);
                    return -1;
                }
                memcpy(dst, new_slot->data + sect_off, chunk);
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
    s_last_msc_write_us = t0;
    if (s_wl == WL_INVALID_HANDLE) return -1;
    if (s_dongle_owns) {
        /* Firmware is mid-FAT-write via the raw-FAT HTTP path.
         * Host's next TUR will get UNIT_ATTENTION sense from
         * disk_release_to_usb, which makes DOS re-read the FAT
         * after we're done. */
        return -1;
    }

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
            /* Hit (any state including FLUSHING): pure RAM merge.
             * If it was FLUSHING the worker already snapshotted the
             * old data; we can safely overwrite live data. Marking
             * DIRTY tells the worker "redo this one when you're back". */
            memcpy(slot->data + sect_off, src, chunk);
            slot->state = SLOT_DIRTY;
            slot->last_use_us = (uint64_t)esp_timer_get_time();
            s_wb_merge_count++;
        } else {
            /* Miss. Allocate a slot (may evict a CLEAN one or block
             * briefly waiting for the worker to drain). */
            esp_err_t e = wb_alloc_slot(&slot);
            if (e != ESP_OK) {
                xSemaphoreGive(s_io_mutex);
                return -1;
            }
            if (sect_off == 0 && chunk == sect) {
                /* Full-sector overwrite: skip the read. */
                memcpy(slot->data, src, sect);
                slot->state     = SLOT_DIRTY;
                slot->sect_base = sect_base;
            } else {
                e = wb_load_into_slot(slot, sect_base);
                if (e != ESP_OK) {
                    xSemaphoreGive(s_io_mutex);
                    return -1;
                }
                memcpy(slot->data + sect_off, src, chunk);
                slot->state = SLOT_DIRTY;
            }
            slot->last_use_us = (uint64_t)esp_timer_get_time();
        }

        addr += chunk; src += chunk; remaining -= chunk;
    }
    xSemaphoreGive(s_wb_dirty_sem);   /* wake worker (if needed) */
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

/* ---- ownership transfer for raw-FAT HTTP (Phase 2) ----
 *
 * Two-phase take: first pre-flush the cache WHILE MSC writes are
 * still allowed (so the host stays usable up to the last possible
 * moment). Then set s_dongle_owns=true. Any MSC write that races
 * past the flush is held in the cache; the next pre-flush before
 * the FATFS op picks it up.
 *
 * Release: invalidate cache (slots may hold stale data vs. flash
 * after FATFS wrote), clear owns flag, signal SCSI UNIT_ATTENTION
 * so the next TUR makes DOS re-read the FAT. */

bool disk_dongle_owns(void) { return s_dongle_owns; }

wl_handle_t disk_get_wl_handle(void) { return s_wl; }

esp_err_t disk_take_for_firmware(void) {
    if (s_wl == WL_INVALID_HANDLE) return ESP_ERR_INVALID_STATE;

    /* Defer if the host has been actively writing very recently --
     * better to bounce the HTTP request than to fight DOS for the
     * FAT mid-burst. Caller (HTTP handler) returns 429 to retry. */
    int64_t now = esp_timer_get_time();
    if (now - s_last_msc_write_us < DISK_RECENT_MSC_WRITE_GUARD_US)
        return ESP_ERR_TIMEOUT;

    /* Pre-flush before claiming ownership. MSC writes remain allowed
     * during this so the host doesn't see a stall. */
    xSemaphoreTake(s_io_mutex, portMAX_DELAY);
    esp_err_t e = wb_flush_all_sync();
    if (e != ESP_OK) { xSemaphoreGive(s_io_mutex); return e; }

    s_dongle_owns = true;

    /* Drain any MSC write that landed between the flush and the
     * owns=true assignment. Same wb_flush_all_sync; any new dirty
     * slots from a racing tud_msc_write10_cb get committed. */
    e = wb_flush_all_sync();
    xSemaphoreGive(s_io_mutex);
    if (e != ESP_OK) {
        s_dongle_owns = false;
        return e;
    }
    return ESP_OK;
}

void disk_release_to_usb(void) {
    xSemaphoreTake(s_io_mutex, portMAX_DELAY);
    /* FATFS may have written to flash via wl_write while we owned;
     * our cache could now be stale. Drop everything. */
    wb_invalidate_all();
    s_dongle_owns = false;
    xSemaphoreGive(s_io_mutex);
    /* Next host TUR will see this sense data and re-read the FAT.
     * 28h/00 == "not-ready-to-ready transition, medium may have
     * changed" -- the right code for "I touched the volume." */
    tud_msc_set_sense(0, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
}

/* ---- JSON snapshot for /usb-stats ---- */

size_t disk_stats_json(char *out, size_t cap) {
    unsigned dirty = 0, flushing = 0, clean = 0, empty = 0;
    for (int i = 0; i < WB_SLOTS; ++i) {
        switch (s_wb[i].state) {
            case SLOT_EMPTY:    empty++;    break;
            case SLOT_CLEAN:    clean++;    break;
            case SLOT_DIRTY:    dirty++;    break;
            case SLOT_FLUSHING: flushing++; break;
        }
    }
    int n = snprintf(out, cap,
        "{\"counts\":{\"tur\":%u,\"read\":%u,\"write\":%u,\"scsi\":%u,\"start_stop\":%u"
        ",\"mount\":%u,\"umount\":%u,\"suspend\":%u,\"resume\":%u}"
        ",\"last\":{\"op\":\"%c\",\"lba\":%u,\"size\":%u}"
        ",\"timing_us\":{\"max_write\":%u,\"max_write_lba\":%u,\"slow_writes\":%u,\"max_read\":%u,\"max_alloc_wait\":%u}"
        ",\"wb\":{\"slots\":%u,\"merges\":%u,\"evicts\":%u,\"async_flushes\":%u"
        ",\"empty\":%u,\"clean\":%u,\"dirty\":%u,\"flushing\":%u}}\n",
        (unsigned)s_cb_tur, (unsigned)s_cb_read, (unsigned)s_cb_write,
        (unsigned)s_cb_scsi, (unsigned)s_cb_start_stop,
        (unsigned)s_cb_mount, (unsigned)s_cb_umount,
        (unsigned)s_cb_suspend, (unsigned)s_cb_resume,
        s_last_op, (unsigned)s_last_lba, (unsigned)s_last_size,
        (unsigned)s_max_write_us, (unsigned)s_max_write_lba,
        (unsigned)s_slow_write_count, (unsigned)s_max_read_us,
        (unsigned)s_max_alloc_wait_us,
        (unsigned)WB_SLOTS, (unsigned)s_wb_merge_count, (unsigned)s_wb_evict_count,
        (unsigned)s_wb_async_flush_count,
        empty, clean, dirty, flushing);
    return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}
