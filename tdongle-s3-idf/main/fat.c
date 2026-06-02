/* fat.c -- FATFS-backed file ops on the WL-mounted DOSONGLE volume.
 *
 * Mount/unmount around every call so we don't keep FATFS state live
 * across HTTP requests -- DOS may have mutated the FAT in between.
 * The take/release-for-firmware ownership transition lives in disk.c;
 * writes flush dirty MSC cache before FATFS sees the volume, then
 * invalidate the cache after so MSC reads pick up FATFS's changes.
 */

#include "fat.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "esp_err.h"
#include "esp_log.h"
#include "ff.h"
#include "diskio.h"
#include "diskio_impl.h"
#include "wear_levelling.h"
#include "esp_partition.h"

#include "disk.h"     /* for disk_take_for_firmware / disk_release_to_usb / disk_get_wl_handle */

#define TAG "fat"

/* MSC reports 512-byte logical blocks (DOS expects that); our
 * hand-built BPB matches. IDF's stock diskio_wl exposes wl_sector_size()
 * (= 4096) to FATFS, which then rejects our 512-byte BPB with
 * FR_NO_FILESYSTEM. This custom diskio wraps wl with 512-byte
 * semantics so FATFS sees the same geometry the BPB declares. */

#define FAT_SECTOR_SIZE  512U
#define WL_SECTOR_BYTES  4096U

static wl_handle_t s_wl_for_diskio = WL_INVALID_HANDLE;

static DSTATUS d_init(unsigned char pdrv)   { (void)pdrv; return 0; }
static DSTATUS d_status(unsigned char pdrv) { (void)pdrv; return 0; }

static DRESULT d_read(unsigned char pdrv, unsigned char *buff, uint32_t sector, unsigned count) {
    (void)pdrv;
    /* wl_read takes arbitrary byte ranges -- we just pass through.
     * Reads don't need RMW.
     *
     * Hold the shared I/O mutex for this sector op so it's mutually
     * exclusive with the MSC read10/write10 path (which also takes it
     * per op). Same per-op granularity as MSC, so neither side blocks
     * the other for long -- but the dongle's wl_read never races the
     * host's at the lower WL lock, which is what wedged the httpd
     * worker under a storming macOS host. 3 s bounded; on the (in
     * practice impossible) timeout we fail the sector -> FATFS read
     * error -> HTTP 500, rather than hang. */
    if (!disk_fatfs_lock(3000)) return RES_ERROR;
    size_t off = (size_t)sector * FAT_SECTOR_SIZE;
    size_t len = (size_t)count * FAT_SECTOR_SIZE;
    DRESULT r = (wl_read(s_wl_for_diskio, off, buff, len) == ESP_OK) ? RES_OK : RES_ERROR;
    disk_fatfs_unlock();
    return r;
}

static DRESULT d_write(unsigned char pdrv, const unsigned char *buff, uint32_t sector, unsigned count) {
    (void)pdrv;
    /* Read-modify-write at the 4 KB wl sector. For each 4 KB block
     * we touch: read full block, splice in the changed 512-byte
     * slices, erase, write full block. */
    static uint8_t rmw[WL_SECTOR_BYTES];
    size_t off = (size_t)sector * FAT_SECTOR_SIZE;
    size_t len = (size_t)count * FAT_SECTOR_SIZE;

    while (len > 0) {
        size_t wl_off  = off & ~(WL_SECTOR_BYTES - 1U);
        size_t in_wl   = off & (WL_SECTOR_BYTES - 1U);
        size_t chunk   = WL_SECTOR_BYTES - in_wl;
        if (chunk > len) chunk = len;

        /* Lock per 4 KB block so the read-modify-erase-write is atomic
         * against the MSC path (a host READ/WRITE mid-RMW would see or
         * cause a torn block). Same mutex MSC uses; released between
         * blocks so we don't starve the USB stack. */
        if (!disk_fatfs_lock(3000)) return RES_ERROR;
        if (wl_read(s_wl_for_diskio, wl_off, rmw, WL_SECTOR_BYTES) != ESP_OK) { disk_fatfs_unlock(); return RES_ERROR; }
        memcpy(rmw + in_wl, buff, chunk);
        if (wl_erase_range(s_wl_for_diskio, wl_off, WL_SECTOR_BYTES) != ESP_OK) { disk_fatfs_unlock(); return RES_ERROR; }
        if (wl_write(s_wl_for_diskio, wl_off, rmw, WL_SECTOR_BYTES) != ESP_OK)  { disk_fatfs_unlock(); return RES_ERROR; }
        disk_fatfs_unlock();

        off  += chunk;
        buff += chunk;
        len  -= chunk;
    }
    return RES_OK;
}

static DRESULT d_ioctl(unsigned char pdrv, unsigned char cmd, void *buff) {
    (void)pdrv;
    switch (cmd) {
        case CTRL_SYNC:
            return RES_OK;
        case GET_SECTOR_COUNT:
            *((uint32_t *)buff) = (uint32_t)(wl_size(s_wl_for_diskio) / FAT_SECTOR_SIZE);
            return RES_OK;
        case GET_SECTOR_SIZE:
            *((uint16_t *)buff) = FAT_SECTOR_SIZE;
            return RES_OK;
        case GET_BLOCK_SIZE:
            /* In FATFS-block units; one wl sector = 8 of our logical
             * sectors, so 8 is the natural erase block. */
            *((uint32_t *)buff) = WL_SECTOR_BYTES / FAT_SECTOR_SIZE;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}

static const ff_diskio_impl_t s_diskio_512 = {
    .init   = d_init,
    .status = d_status,
    .read   = d_read,
    .write  = d_write,
    .ioctl  = d_ioctl,
};

/* ---- name validation ---- */

bool fat_uri_to_name83(const char *uri, char out_name[13]) {
    /* Accept "/fs/NAME[.EXT]" or "NAME[.EXT]". Uppercases as we copy.
     * Rejects anything that isn't pure 8.3 (no slashes, no leading
     * '.', basename <= 8 chars, ext <= 3 chars, no double dots). */
    if (uri[0] == '/' && !strncmp(uri, "/fs/", 4)) uri += 4;
    if (!*uri) return false;
    if (strchr(uri, '/')) return false;
    if (strstr(uri, "..")) return false;

    char base[9] = {0};
    char ext[4]  = {0};
    const char *dot = strchr(uri, '.');
    size_t blen = dot ? (size_t)(dot - uri) : strlen(uri);
    if (blen == 0 || blen > 8) return false;
    if (dot) {
        size_t elen = strlen(dot + 1);
        if (elen == 0 || elen > 3) return false;
        if (strchr(dot + 1, '.')) return false;
        memcpy(ext, dot + 1, elen);
        for (size_t i = 0; i < elen; ++i) ext[i] = (char)toupper((unsigned char)ext[i]);
    }
    memcpy(base, uri, blen);
    for (size_t i = 0; i < blen; ++i) base[i] = (char)toupper((unsigned char)base[i]);

    /* Forbid characters illegal in FAT short names. Conservative set. */
    static const char *forbidden = " +,;=[]\"\\/:|<>?*";
    for (size_t i = 0; i < blen;        ++i) if (strchr(forbidden, base[i])) return false;
    for (size_t i = 0; i < strlen(ext); ++i) if (strchr(forbidden, ext[i]))  return false;

    if (ext[0]) snprintf(out_name, 13, "%s.%s", base, ext);
    else        snprintf(out_name, 13, "%s",    base);
    return true;
}

/* ---- mount / unmount helpers ---- */

static FATFS s_fs;     /* big -- ~600 bytes -- but static avoids
                        * blowing the HTTPD task stack. The take/
                        * release-for-firmware ownership guarantees
                        * only one HTTP op uses this at a time. */

static esp_err_t fat_mount(void) {
    wl_handle_t wl = disk_get_wl_handle();
    if (wl == WL_INVALID_HANDLE) {
        disk_logf("[fat] mount: WL not ready");
        return ESP_ERR_INVALID_STATE;
    }
    s_wl_for_diskio = wl;
    ff_diskio_register(0, &s_diskio_512);
    FRESULT fr = f_mount(&s_fs, "0:", 1);
    if (fr != FR_OK) {
        disk_logf("[fat] f_mount: FR=%d", (int)fr);
        ff_diskio_unregister(0);
        s_wl_for_diskio = WL_INVALID_HANDLE;
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void fat_unmount(void) {
    f_unmount("0:");
    ff_diskio_unregister(0);
    s_wl_for_diskio = WL_INVALID_HANDLE;
}

/* ---- ops ---- */

esp_err_t fat_list(fat_list_cb_t cb, void *ctx) {
    esp_err_t e = fat_mount();
    if (e != ESP_OK) return e;

    FF_DIR  dir;
    FILINFO fno;
    FRESULT fr = f_opendir(&dir, "0:/");
    if (fr != FR_OK) { fat_unmount(); return ESP_FAIL; }

    bool keep_going = true;
    while (keep_going) {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK || fno.fname[0] == 0) break;
        if (fno.fname[0] == '.') continue;   /* skip dotfiles + .. */
        bool is_dir = (fno.fattrib & AM_DIR) != 0;
        keep_going = cb(fno.fname, (uint32_t)fno.fsize, is_dir, ctx);
    }
    f_closedir(&dir);
    fat_unmount();
    return ESP_OK;
}

esp_err_t fat_read(const char *name83, fat_out_cb_t out, void *ctx) {
    esp_err_t e = fat_mount();
    if (e != ESP_OK) return e;

    char path[20];
    snprintf(path, sizeof path, "0:/%s", name83);

    FIL f;
    FRESULT fr = f_open(&f, path, FA_READ);
    if (fr != FR_OK) {
        fat_unmount();
        return (fr == FR_NO_FILE) ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    }

    /* Stream in chunks. 4 KB matches our WL sector + cache slot so
     * each f_read in the steady state translates to one wl_read. */
    static uint8_t buf[4096];
    while (1) {
        UINT br = 0;
        fr = f_read(&f, buf, sizeof buf, &br);
        if (fr != FR_OK) { e = ESP_FAIL; break; }
        if (br == 0) break;
        e = out(buf, br, ctx);
        if (e != ESP_OK) break;
    }
    f_close(&f);
    fat_unmount();
    return e;
}

esp_err_t fat_write(const char *name83, fat_in_cb_t in, size_t total, void *ctx) {
    esp_err_t e = disk_take_for_firmware();
    if (e != ESP_OK) return e;
    e = fat_mount();
    if (e != ESP_OK) { disk_release_to_usb(); return e; }

    char path[20];
    snprintf(path, sizeof path, "0:/%s", name83);

    FIL f;
    FRESULT fr = f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        disk_logf("[fat] f_open(%s, WRITE): FR=%d", path, (int)fr);
        fat_unmount(); disk_release_to_usb(); return ESP_FAIL;
    }

    static uint8_t buf[4096];
    size_t remaining = total;
    while (remaining > 0) {
        size_t want = remaining < sizeof buf ? remaining : sizeof buf;
        size_t got = 0;
        e = in(buf, want, &got, ctx);
        if (e != ESP_OK || got == 0) break;
        UINT bw = 0;
        fr = f_write(&f, buf, (UINT)got, &bw);
        if (fr != FR_OK || bw != got) { e = ESP_FAIL; break; }
        remaining -= got;
    }
    f_close(&f);
    fat_unmount();
    disk_release_to_usb();
    return e;
}

esp_err_t fat_delete(const char *name83) {
    esp_err_t e = disk_take_for_firmware();
    if (e != ESP_OK) return e;
    e = fat_mount();
    if (e != ESP_OK) { disk_release_to_usb(); return e; }

    char path[20];
    snprintf(path, sizeof path, "0:/%s", name83);

    FRESULT fr = f_unlink(path);
    fat_unmount();
    disk_release_to_usb();
    if (fr == FR_NO_FILE) return ESP_ERR_NOT_FOUND;
    if (fr != FR_OK)      return ESP_FAIL;
    return ESP_OK;
}
