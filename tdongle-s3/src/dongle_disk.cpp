// dongle_disk: USB MSC + HTTP file server backed by a FAT partition on
// internal flash. See dongle_disk.h for the architecture summary.
//
// Coordination: the FAT partition is owned EITHER by the USB MSC side
// (host sees the drive) OR by the dongle's FATFS mount (HTTP can list,
// read, write). Switching ownership cycles mediaPresent on MSC so the
// host re-reads after a write -- never both sides on the same wear-
// levelling handle at once.
//
// HTTP transactions self-arbitrate: each request that touches the disk
// briefly takes OWNER_DONGLE, runs, then returns to OWNER_USB. The
// host sees the disk eject and re-mount around each WiFi op; for the
// dev loop (occasional log fetch / binary push) that's acceptable.

#include "dongle_disk.h"

#include <Arduino.h>
#include <WiFi.h>
#include <USB.h>
#include <USBMSC.h>
#include <ESPmDNS.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/unistd.h>

#include "esp_partition.h"
#include "wear_levelling.h"
#include "esp_vfs_fat.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "ff.h"          /* f_setlabel for the FAT volume name */

#define TAG               "disk"
#define FAT_PART_LABEL    "ffat"
#define MOUNT_POINT       "/dongle"
#define MDNS_HOSTNAME     "dosongle"
#define VOLUME_LABEL      "DOSONGLE"      /* shown as the disk name on the host */

// Debug to the hardware UART (Serial0) so we don't pollute the USB CDC
// data link. Match the DBG macro style used elsewhere in this firmware.
#define DBG(...) do { Serial0.printf(__VA_ARGS__); } while (0)

static USBMSC          s_msc;
static wl_handle_t     s_wl = WL_INVALID_HANDLE;
static const esp_partition_t *s_part = nullptr;
// MSC logical block size = WL physical sector size (4 KB on this SDK
// build). Matching the two means:
//   - read/write callbacks are sector-aligned -- no RMW needed
//   - the FAT BPB (also formatted at WL sector size) agrees with what
//     the MSC layer reports to the host, so macOS / DOS see a valid FS
static uint32_t        s_block_size = 4096;
static uint32_t        s_block_count = 0;
static SemaphoreHandle_t s_lock = nullptr;     // owner mutex
static volatile bool   s_dongle_owns = false;  // false = USB owns, true = FATFS owns
static volatile bool   s_msc_present = true;   // current mediaPresent flag
static httpd_handle_t  s_httpd = nullptr;
static volatile bool   s_mdns_up = false;

// ---- low-level: raw flash via WL, 512-byte logical blocks ----

static int32_t msc_on_read(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    if (s_dongle_owns || s_wl == WL_INVALID_HANDLE) return -1;
    size_t addr = (size_t)lba * s_block_size + offset;
    if (wl_read(s_wl, addr, buffer, bufsize) != ESP_OK) return -1;
    return (int32_t)bufsize;
}

// Block size matches WL sector size, so writes arrive 4 KB-aligned at
// offset 0. Erase the sector, write it. (If the host ever issues a
// partial-sector write we fall back to RMW.)
static int32_t msc_on_write(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    if (s_dongle_owns || s_wl == WL_INVALID_HANDLE) return -1;
    size_t sect = wl_sector_size(s_wl);
    size_t addr = (size_t)lba * s_block_size + offset;

    if (offset == 0 && bufsize == sect) {
        if (wl_erase_range(s_wl, addr, sect) != ESP_OK) return -1;
        if (wl_write(s_wl, addr, buffer, sect) != ESP_OK) return -1;
        return (int32_t)bufsize;
    }

    // Partial-sector path: read whole sector, splice in, erase+write back.
    static uint8_t sbuf[4096];
    if (sect > sizeof sbuf) return -1;
    size_t remaining = bufsize;
    uint8_t *src = buffer;
    while (remaining > 0) {
        size_t sect_off = addr & (sect - 1);
        size_t sect_base = addr - sect_off;
        size_t chunk = sect - sect_off;
        if (chunk > remaining) chunk = remaining;
        if (wl_read(s_wl, sect_base, sbuf, sect) != ESP_OK) return -1;
        memcpy(sbuf + sect_off, src, chunk);
        if (wl_erase_range(s_wl, sect_base, sect) != ESP_OK) return -1;
        if (wl_write(s_wl, sect_base, sbuf, sect) != ESP_OK) return -1;
        addr += chunk; src += chunk; remaining -= chunk;
    }
    return (int32_t)bufsize;
}

static bool msc_on_start_stop(uint8_t power_condition, bool start, bool load_eject) {
    // Host SCSI 1Bh START_STOP_UNIT. We accept all; eject just flips our
    // mediaPresent flag so the host treats the volume as gone.
    if (load_eject && !start) {
        s_msc.mediaPresent(false);
        s_msc_present = false;
    }
    return true;
}

// ---- ownership: flip USB <-> FATFS atomically ----

// Mount FATFS for our use; before calling this the caller MUST hold s_lock
// and have flipped mediaPresent(false) so the host stops touching wl.
static esp_err_t mount_for_dongle(void) {
    if (s_wl == WL_INVALID_HANDLE) return ESP_ERR_INVALID_STATE;
    esp_vfs_fat_mount_config_t cfg = {};
    cfg.format_if_mount_failed = true;
    cfg.max_files = 4;
    cfg.allocation_unit_size = 0;        // default cluster size
    cfg.disk_status_check_enable = false;
    cfg.use_one_fat = false;
    // unmount the wl handle first so esp_vfs_fat_spiflash_mount_rw_wl can
    // mount it itself. wl_unmount, then re-handed via the mount helper.
    wl_unmount(s_wl);
    s_wl = WL_INVALID_HANDLE;
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_POINT, FAT_PART_LABEL, &cfg, &s_wl);
    if (err != ESP_OK) {
        DBG("[disk] mount_for_dongle: vfs_fat_mount err=0x%x\n", err);
        // try to recover the raw wl handle so MSC keeps working
        wl_mount(s_part, &s_wl);
    }
    return err;
}

static void unmount_from_dongle(void) {
    esp_vfs_fat_spiflash_unmount_rw_wl(MOUNT_POINT, s_wl);
    s_wl = WL_INVALID_HANDLE;
    // re-attach raw WL handle for MSC
    wl_mount(s_part, &s_wl);
}

// Take ownership for an HTTP transaction. Returns ESP_OK on success.
static esp_err_t lock_for_dongle(void) {
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(8000)) != pdTRUE) return ESP_ERR_TIMEOUT;
    // 1) tell host the medium is gone so it stops issuing READ_10/WRITE_10
    s_msc.mediaPresent(false);
    s_msc_present = false;
    // 2) wait briefly for any in-flight bulk to drain
    vTaskDelay(pdMS_TO_TICKS(150));
    // 3) flip ownership flag (MSC callbacks now refuse with -1)
    s_dongle_owns = true;
    // 4) mount FATFS
    esp_err_t err = mount_for_dongle();
    if (err != ESP_OK) {
        s_dongle_owns = false;
        s_msc.mediaPresent(true);
        s_msc_present = true;
        xSemaphoreGive(s_lock);
        return err;
    }
    return ESP_OK;
}

static void unlock_from_dongle(void) {
    unmount_from_dongle();
    s_dongle_owns = false;
    s_msc.mediaPresent(true);
    s_msc_present = true;
    xSemaphoreGive(s_lock);
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
    "<li><code>PUT /fs/&lt;path&gt;</code> &mdash; upload a file (raw body)</li>"
    "<li><code>DELETE /fs/&lt;path&gt;</code> &mdash; delete</li>"
    "<li><code>POST /eject</code> &mdash; eject from host</li>"
    "<li><code>POST /reset</code> &mdash; reboot dongle</li>"
    "</ul>"
    "<p>Each <code>/fs</code> and <code>/list</code> request briefly ejects the "
    "disk from USB while it runs, then re-presents it.</p>";

static esp_err_t h_root(httpd_req_t *req) { return send_text(req, "200 OK", "text/html; charset=utf-8", INDEX_HTML); }

static esp_err_t h_status(httpd_req_t *req) {
    char buf[512];
    uint64_t total = 0, freeb = 0;
    bool got_fs = false;
    if (lock_for_dongle() == ESP_OK) {
        if (esp_vfs_fat_info(MOUNT_POINT, &total, &freeb) == ESP_OK) got_fs = true;
        unlock_from_dongle();
    }
    int n = snprintf(buf, sizeof buf,
        "{\"mdns\":\"%s.local\",\"wifi\":{\"connected\":%s,\"ip\":\"%s\",\"rssi\":%d},"
        "\"msc\":{\"present\":%s,\"block_size\":%u,\"block_count\":%u},"
        "\"fs\":{\"total\":%llu,\"free\":%llu}}",
        MDNS_HOSTNAME,
        WiFi.status() == WL_CONNECTED ? "true" : "false",
        WiFi.localIP().toString().c_str(),
        (int)WiFi.RSSI(),
        s_msc_present ? "true" : "false",
        (unsigned)s_block_size, (unsigned)s_block_count,
        (unsigned long long)total, (unsigned long long)freeb);
    (void)got_fs;
    if (n < 0) n = 0;
    return send_text(req, "200 OK", "application/json", buf);
}

static esp_err_t h_list(httpd_req_t *req) {
    esp_err_t err = lock_for_dongle();
    if (err != ESP_OK) return send_text(req, "503 Service Unavailable", "text/plain", "lock failed\n");

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");

    DIR *d = opendir(MOUNT_POINT);
    if (!d) {
        unlock_from_dongle();
        return httpd_resp_send(req, "(opendir failed)\n", HTTPD_RESP_USE_STRLEN);
    }
    char line[160];
    struct dirent *e;
    while ((e = readdir(d)) != nullptr) {
        struct stat st;
        char path[256];
        snprintf(path, sizeof path, "%s/%s", MOUNT_POINT, e->d_name);
        if (stat(path, &st) != 0) continue;
        int n = snprintf(line, sizeof line, "%10ld  %s%s\n",
                         (long)st.st_size, e->d_name,
                         S_ISDIR(st.st_mode) ? "/" : "");
        if (n > 0) httpd_resp_send_chunk(req, line, n);
    }
    closedir(d);
    httpd_resp_send_chunk(req, nullptr, 0);
    unlock_from_dongle();
    return ESP_OK;
}

// Translate URI /fs/<path> -> MOUNT_POINT/<UPPER(path)>. Reject ".." and
// any subdirs (FAT root only, keeps things simple). Caller-provided buf
// must be >= 256.
static bool resolve_fs_path(const char *uri, char *out, size_t outsz) {
    const char *p = uri;
    if (strncmp(p, "/fs/", 4) != 0) return false;
    p += 4;
    if (!*p) return false;
    if (strchr(p, '/')) return false;          // root-only
    if (strstr(p, "..")) return false;
    size_t n = snprintf(out, outsz, "%s/", MOUNT_POINT);
    for (; *p && n + 1 < outsz; ++p) {
        char c = *p;
        if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';     // DOS 8.3 likes upper
        out[n++] = c;
    }
    out[n] = 0;
    return true;
}

static esp_err_t h_fs_get(httpd_req_t *req) {
    char path[256];
    if (!resolve_fs_path(req->uri, path, sizeof path))
        return send_text(req, "400 Bad Request", "text/plain", "bad path\n");

    if (lock_for_dongle() != ESP_OK)
        return send_text(req, "503 Service Unavailable", "text/plain", "lock failed\n");

    FILE *f = fopen(path, "rb");
    if (!f) {
        unlock_from_dongle();
        return send_text(req, "404 Not Found", "text/plain", "no such file\n");
    }

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Connection", "close");

    char buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof buf, f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) break;
    }
    fclose(f);
    httpd_resp_send_chunk(req, nullptr, 0);
    unlock_from_dongle();
    return ESP_OK;
}

static esp_err_t h_fs_put(httpd_req_t *req) {
    char path[256];
    if (!resolve_fs_path(req->uri, path, sizeof path))
        return send_text(req, "400 Bad Request", "text/plain", "bad path\n");

    if (lock_for_dongle() != ESP_OK)
        return send_text(req, "503 Service Unavailable", "text/plain", "lock failed\n");

    FILE *f = fopen(path, "wb");
    if (!f) {
        unlock_from_dongle();
        return send_text(req, "500 Internal Server Error", "text/plain", "fopen failed\n");
    }

    char buf[1024];
    int remaining = req->content_len;
    int got_total = 0;
    while (remaining > 0) {
        int want = remaining > (int)sizeof buf ? (int)sizeof buf : remaining;
        int r = httpd_req_recv(req, buf, want);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            break;
        }
        if (fwrite(buf, 1, r, f) != (size_t)r) {
            fclose(f);
            unlock_from_dongle();
            return send_text(req, "500 Internal Server Error", "text/plain", "fwrite failed\n");
        }
        got_total += r;
        remaining -= r;
    }
    fflush(f);
    fsync(fileno(f));            // FATFS f_sync via the VFS layer -- forces
                                 // the dir entry + FAT chain out to wear_levelling
    fclose(f);
    unlock_from_dongle();

    char rsp[96];
    snprintf(rsp, sizeof rsp, "wrote %d bytes\n", got_total);
    return send_text(req, "200 OK", "text/plain", rsp);
}

static esp_err_t h_fs_delete(httpd_req_t *req) {
    char path[256];
    if (!resolve_fs_path(req->uri, path, sizeof path))
        return send_text(req, "400 Bad Request", "text/plain", "bad path\n");
    if (lock_for_dongle() != ESP_OK)
        return send_text(req, "503 Service Unavailable", "text/plain", "lock failed\n");
    int rc = unlink(path);
    unlock_from_dongle();
    return send_text(req, rc == 0 ? "200 OK" : "404 Not Found",
                     "text/plain", rc == 0 ? "deleted\n" : "no such file\n");
}

static esp_err_t h_eject(httpd_req_t *req) {
    s_msc.mediaPresent(false);
    s_msc_present = false;
    return send_text(req, "200 OK", "text/plain", "ejected\n");
}

static esp_err_t h_present(httpd_req_t *req) {
    s_msc.mediaPresent(true);
    s_msc_present = true;
    return send_text(req, "200 OK", "text/plain", "presented\n");
}

static esp_err_t h_reset(httpd_req_t *req) {
    send_text(req, "200 OK", "text/plain", "rebooting\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;
}

static void httpd_start_once(void) {
    if (s_httpd) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 16;
    cfg.lru_purge_enable = true;
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
        { "/eject",     HTTP_POST,   h_eject,     nullptr },
        { "/present",   HTTP_POST,   h_present,   nullptr },
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

    // First boot: format the partition if it has no valid FAT. Easiest way
    // is to mount with format_if_mount_failed=true, then unmount and grab
    // the raw WL handle.
    esp_vfs_fat_mount_config_t cfg = {};
    cfg.format_if_mount_failed = true;
    cfg.max_files = 4;
    cfg.allocation_unit_size = 0;
    cfg.disk_status_check_enable = false;
    cfg.use_one_fat = false;
    wl_handle_t wl;
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_POINT, FAT_PART_LABEL, &cfg, &wl);
    if (err != ESP_OK) {
        DBG("[disk] initial FAT mount failed: 0x%x\n", err);
        return;
    }
    // Set the volume label so the host sees "DOSONGLE" instead of "NO NAME".
    // FF_STR_VOLUME_ID=0 in this build, so no "label:" prefix -- f_setlabel
    // targets the currently-mounted FATFS drive (the only one we have).
    {
        FRESULT fr = f_setlabel(VOLUME_LABEL);
        if (fr != FR_OK) DBG("[disk] f_setlabel: %d\n", fr);
    }
    esp_vfs_fat_spiflash_unmount_rw_wl(MOUNT_POINT, wl);
    if (wl_mount(s_part, &s_wl) != ESP_OK) {
        DBG("[disk] wl_mount failed\n");
        return;
    }

    s_block_count = (uint32_t)(wl_size(s_wl) / s_block_size);

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
    out->partition_bytes = s_block_count * s_block_size;
    out->used_bytes     = 0;            // not cheap to compute -- skip for AT
    out->http_ready     = s_httpd != nullptr;
    out->mdns_host      = MDNS_HOSTNAME;
}
