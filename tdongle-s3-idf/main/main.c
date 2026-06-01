/*
 * T-Dongle S3 -- ESP-IDF native main.
 *
 * Phase 0: minimum-viable firmware that exposes HTTP OTA so all future
 * iterations are OTA-driven (no Download-mode steps). Brings up NVS,
 * WiFi STA, HTTP server, mDNS as dosongle.local, and the disk_log ring
 * buffer that gives us remote diagnostics. Nothing USB-side yet -- this
 * is just enough to make the dongle reachable + reflashable over WiFi.
 *
 * Migration roadmap (kept here so future-me knows the plan):
 *   [phase 0] NVS, WiFi STA, HTTP server, /ota, /status, /reset,
 *             /disk-log, mDNS.                          <-- this file
 *   [phase 1] USB device init via esp_tinyusb (composite MSC+HID+CDC).
 *   [phase 2] MSC backend (wear_levelling + write-back cache).
 *   [phase 3] Raw-FAT HTTP handlers + /lba diagnostic.
 *   [phase 4] AT modem engine (port from modem.cpp).
 *   [phase 5] SLIP framing + lwIP NAPT (port from main.cpp).
 *   [phase 6] HID typing DSL (port from dongle_kbd.cpp).
 *   [phase 7] ST7735 display via esp_lcd_panel_st7735 (defer).
 *
 * Each phase is one commit so we can bisect regressions.
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_app_desc.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mdns.h"

#include "usb.h"
#include "disk.h"
#include "fat.h"
#include "slip.h"
#include "dns_forwarder.h"
#include "kbd.h"

/* First-flash bootstrap WiFi credentials. The file `wifi_creds.h` is
 * gitignored and locally created. NVS-stored creds always take
 * precedence; this header is only consulted when NVS is empty (i.e. on
 * the very first flash of a fresh dongle). Once we successfully connect
 * we copy the working creds INTO NVS, so subsequent OTA updates don't
 * depend on this header at all.
 */
#if __has_include("wifi_creds.h")
#  include "wifi_creds.h"
#endif
#ifndef WIFI_SSID_DEFAULT
#define WIFI_SSID_DEFAULT ""
#endif
#ifndef WIFI_PASS_DEFAULT
#define WIFI_PASS_DEFAULT ""
#endif

#define MDNS_HOSTNAME "dosongle"

static const char *TAG = "tdongle";

/* ===== boot-time capability dump =====================================
 * Logged once at boot so /disk-log always shows the hardware fingerprint
 * (flash chip vendor, PSRAM size, AUTO_SUSPEND state, reset reason). */

static const char *reset_reason_name(esp_reset_reason_t rr) {
    switch (rr) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

/* ===== disk_log ring buffer ==========================================
 * RAM-resident; persists across DOS/host crashes but lost on dongle
 * reset (which is exactly the diagnostic we want -- on reset, the new
 * init line includes the previous reset_reason). Pulled via GET
 * /disk-log so all events are inspectable without a UART. */

#define DISK_LOG_LINES        64
#define DISK_LOG_LINE_LEN     160      /* prefix "<seq> " + 128 msg + slack */
#define DISK_LOG_MSG_LEN      128

static portMUX_TYPE s_disk_log_mux = portMUX_INITIALIZER_UNLOCKED;
static char         s_disk_log[DISK_LOG_LINES][DISK_LOG_LINE_LEN];
static uint32_t     s_disk_log_seq = 0;

void disk_logf(const char *fmt, ...) {
    char msg[DISK_LOG_MSG_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);

    portENTER_CRITICAL(&s_disk_log_mux);
    uint32_t seq = ++s_disk_log_seq;
    snprintf(s_disk_log[seq % DISK_LOG_LINES], DISK_LOG_LINE_LEN,
             "%" PRIu32 " %s", seq, msg);
    portEXIT_CRITICAL(&s_disk_log_mux);

    /* Also mirror to the standard ESP log so it shows up in the
     * USB-Serial-JTAG console if a developer has it attached. */
    ESP_LOGI(TAG, "[log] %s", msg);
}

/* ===== HTTP handlers ================================================= */

static esp_err_t send_text(httpd_req_t *req, const char *status,
                           const char *ct, const char *body) {
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, ct);
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t h_root(httpd_req_t *req) {
    static const char *INDEX_HTML =
        "<!doctype html><html><head><title>T-Dongle S3 (idf)</title></head><body>"
        "<h1>T-Dongle S3 (ESP-IDF native)</h1>"
        "<ul>"
        "<li><a href=/status>/status</a> &mdash; JSON: WiFi state, IP, RSSI</li>"
        "<li><a href=/disk-log>/disk-log</a> &mdash; diagnostic ring buffer</li>"
        "<li><code>POST /ota</code> &mdash; firmware update via raw .bin body</li>"
        "<li><code>POST /reset</code> &mdash; reboot</li>"
        "</ul>"
        "</body></html>";
    return send_text(req, "200 OK", "text/html; charset=utf-8", INDEX_HTML);
}

static esp_err_t h_status(httpd_req_t *req) {
    wifi_ap_record_t ap = {0};
    bool wifi_up = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip = {0};
    if (netif) esp_netif_get_ip_info(netif, &ip);
    const esp_app_desc_t *app = esp_app_get_description();

    char buf[512];
    int n = snprintf(buf, sizeof buf,
        "{\"mdns\":\"" MDNS_HOSTNAME ".local\","
        "\"wifi\":{\"connected\":%s,\"ssid\":\"%.32s\",\"ip\":\"" IPSTR "\",\"rssi\":%d},"
        "\"app\":{\"name\":\"%.32s\",\"version\":\"%.32s\"},"
        "\"reset_reason\":\"%s\","
        "\"free_heap\":%lu,"
        "\"min_free_heap\":%lu}",
        wifi_up ? "true" : "false",
        (const char *)ap.ssid,
        IP2STR(&ip.ip),
        ap.rssi,
        app->project_name, app->version,
        reset_reason_name(esp_reset_reason()),
        (unsigned long)esp_get_free_heap_size(),
        (unsigned long)esp_get_minimum_free_heap_size());
    (void)n;
    return send_text(req, "200 OK", "application/json", buf);
}

/* POST /type -- body is the HID typing DSL string (see kbd.h). Plain
 * ASCII goes through, <TOKEN> for special keys, <CTRL+x>/<ALT+x>/<SHIFT+x>
 * for combos, <DELAY=ms> for inter-keystroke pauses, "<<" for literal '<'.
 * Returns "OK\n" + bytes consumed, or "ERROR\n" on parse failure. */
static esp_err_t h_type(httpd_req_t *req) {
    int total = req->content_len;
    if (total <= 0)
        return send_text(req, "411 Length Required", "text/plain",
                         "need Content-Length\n");
    if (total > 4096)
        return send_text(req, "413 Payload Too Large", "text/plain",
                         "type body capped at 4096 B\n");
    char *body = malloc((size_t)total + 1);
    if (!body)
        return send_text(req, "500 Internal Server Error", "text/plain",
                         "alloc fail\n");
    int got = 0;
    while (got < total) {
        int r = httpd_req_recv(req, body + got, (size_t)(total - got));
        if (r <= 0) { free(body); return ESP_FAIL; }
        got += r;
    }
    body[got] = 0;
    int n = kbd_type(body, got);
    free(body);
    if (n < 0)
        return send_text(req, "400 Bad Request", "text/plain",
                         "DSL parse error (see /type-log)\n");
    char ok[32];
    snprintf(ok, sizeof ok, "OK %d\n", n);
    return send_text(req, "200 OK", "text/plain", ok);
}

/* GET /type-log -- recent HID-typing events. Ring buffer; small. */
static esp_err_t h_type_log(httpd_req_t *req) {
    static char buf[6144];
    int n = kbd_log_dump(buf, sizeof buf);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

/* GET /partitions -- dump OTA partition layout + which one is running +
 * which one the bootloader will pick next + each app partition's state
 * (NEW / PENDING_VERIFY / VALID / INVALID / ABORTED / UNDEFINED).
 * Diagnostic for "OTA flash succeeded but old firmware still booting":
 * if the running partition isn't the boot partition, the bootloader
 * rolled back; the state column says why. */
static esp_err_t h_partitions(httpd_req_t *req) {
    const esp_partition_t *run  = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();

    char buf[1024];
    int n = 0;
    n += snprintf(buf+n, sizeof buf - n,
        "running: %s @ 0x%lx (subtype %d)\n"
        "boot:    %s @ 0x%lx (subtype %d)\n",
        run  ? run->label  : "?", (unsigned long)(run  ? run->address  : 0),
        run  ? run->subtype: -1,
        boot ? boot->label : "?", (unsigned long)(boot ? boot->address : 0),
        boot ? boot->subtype: -1);

    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP,
                                                     ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it) {
        const esp_partition_t *p = esp_partition_get(it);
        esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
        esp_err_t e = esp_ota_get_state_partition(p, &st);
        const char *sn;
        switch (st) {
            case ESP_OTA_IMG_NEW:            sn = "NEW";            break;
            case ESP_OTA_IMG_PENDING_VERIFY: sn = "PENDING_VERIFY"; break;
            case ESP_OTA_IMG_VALID:          sn = "VALID";          break;
            case ESP_OTA_IMG_INVALID:        sn = "INVALID";        break;
            case ESP_OTA_IMG_ABORTED:        sn = "ABORTED";        break;
            case ESP_OTA_IMG_UNDEFINED:      sn = "UNDEFINED";      break;
            default:                         sn = "?";              break;
        }
        n += snprintf(buf+n, sizeof buf - n,
            "  %-8s @ 0x%06lx size=0x%lx state=%s (rc=%d)%s%s\n",
            p->label, (unsigned long)p->address, (unsigned long)p->size,
            sn, (int)e,
            (run  && run->address  == p->address) ? "  <-running" : "",
            (boot && boot->address == p->address) ? " <-boot"     : "");
        it = esp_partition_next(it);
    }
    esp_partition_iterator_release(it);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static esp_err_t h_reset(httpd_req_t *req) {
    send_text(req, "200 OK", "text/plain", "rebooting\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;
}

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
        int w = snprintf(line, sizeof line, "%s\n",
                         s_disk_log[cur % DISK_LOG_LINES]);
        portEXIT_CRITICAL(&s_disk_log_mux);
        if (w <= 0) continue;
        if (httpd_resp_send_chunk(req, line, (size_t)w) != ESP_OK) break;
    }
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* /ota: POST raw firmware.bin as the body. We write it into the
 * inactive OTA slot, mark that slot bootable, reply, then esp_restart.
 * Matches the protocol the arduino-esp32 build exposed, so dosongle.sh
 * ota works against this firmware too. */
static esp_err_t h_ota(httpd_req_t *req) {
    disk_logf("ota: h_ota entered, content_len=%d", req->content_len);
    int total = req->content_len;
    if (total <= 0)
        return send_text(req, "411 Length Required", "text/plain",
                         "need Content-Length\n");

    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (!next)
        return send_text(req, "500 Internal Server Error", "text/plain",
                         "no OTA partition\n");
    disk_logf("ota: target partition %s @ 0x%lx size=0x%lx",
              next->label, (unsigned long)next->address, (unsigned long)next->size);
    if ((size_t)total > next->size)
        return send_text(req, "413 Payload Too Large", "text/plain",
                         "exceeds OTA slot size\n");

    int64_t t_begin = esp_timer_get_time();
    esp_ota_handle_t h = 0;
    esp_err_t be = esp_ota_begin(next, OTA_SIZE_UNKNOWN, &h);
    disk_logf("ota: esp_ota_begin -> %d (%lld us)",
              (int)be, (long long)(esp_timer_get_time() - t_begin));
    if (be != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain",
                         "esp_ota_begin failed\n");

    uint8_t *buf = malloc(2048);
    if (!buf) {
        esp_ota_abort(h);
        return send_text(req, "500 Internal Server Error", "text/plain", "oom\n");
    }

    int got = 0;
    bool ok = true;
    int next_log = 32 * 1024;
    int64_t t_start = esp_timer_get_time();
    disk_logf("ota: begin, total=%d", total);
    while (got < total) {
        int want = total - got;
        if (want > 2048) want = 2048;
        int r = httpd_req_recv(req, (char *)buf, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            disk_logf("ota: recv TIMEOUT at got=%d (continuing)", got);
            continue;
        }
        if (r <= 0) {
            disk_logf("ota: recv err r=%d at got=%d (fatal)", r, got);
            ok = false; break;
        }
        int64_t t0 = esp_timer_get_time();
        esp_err_t we = esp_ota_write(h, buf, (size_t)r);
        int64_t dt = esp_timer_get_time() - t0;
        if (we != ESP_OK) {
            disk_logf("ota: write fail rc=%d at got=%d", (int)we, got);
            ok = false; break;
        }
        got += r;
        if (got >= next_log) {
            int64_t elapsed = esp_timer_get_time() - t_start;
            disk_logf("ota: got=%d (%lld us elapsed; last write %lld us)",
                      got, (long long)elapsed, (long long)dt);
            next_log += 32 * 1024;
        }
    }
    disk_logf("ota: loop exit ok=%d got=%d/%d", ok ? 1 : 0, got, total);
    free(buf);

    if (!ok || got != total) {
        esp_ota_abort(h);
        return send_text(req, "500 Internal Server Error", "text/plain",
                         "OTA receive/write failed\n");
    }
    if (esp_ota_end(h) != ESP_OK)
        return send_text(req, "400 Bad Request", "text/plain",
                         "OTA end failed; bad image?\n");
    if (esp_ota_set_boot_partition(next) != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain",
                         "OTA set boot failed\n");

    send_text(req, "200 OK", "text/plain", "OTA OK; rebooting\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* /usb-start: bring up the composite USB device on demand. Deferred
 * out of boot so a panic in TinyUSB init can't take WiFi/HTTP down
 * with it -- the dongle stays reachable and we can OTA a fix. */
static esp_err_t h_usb_start(httpd_req_t *req) {
    disk_logf("usb-start: enter");
    esp_err_t e = usb_start();
    disk_logf("usb-start: ret=%s", esp_err_to_name(e));
    if (e == ESP_OK)
        return send_text(req, "200 OK", "text/plain", "USB up\n");
    char msg[64];
    snprintf(msg, sizeof msg, "usb_start failed: %s\n", esp_err_to_name(e));
    return send_text(req, "500 Internal Server Error", "text/plain", msg);
}

/* ---- /list, /fs/<name>, /lba (raw-FAT HTTP, Phase 2) ----
 *
 * Reads (/list, GET /fs, /lba) don't take dongle ownership -- they
 * go through FATFS mounted briefly on the same WL handle, accepting
 * the small chance of inconsistency vs. concurrent MSC writes.
 *
 * Writes (PUT, DELETE) call disk_take_for_firmware() / release in
 * fat.c, so MSC stays consistent while we mutate the FAT. */

static bool list_json_cb(const char *name, uint32_t size, bool is_dir, void *ctx) {
    httpd_req_t *req = (httpd_req_t *)ctx;
    char line[96];
    int n = snprintf(line, sizeof line,
                     "  {\"name\":\"%s\",\"size\":%u,\"dir\":%s}",
                     name, (unsigned)size, is_dir ? "true" : "false");
    if (n <= 0 || (size_t)n >= sizeof line) return true;
    /* Comma separator only after the first entry; track via a
     * stashed flag in the req's user_ctx slot. */
    if (req->user_ctx) httpd_resp_send_chunk(req, ",\n", 2);
    httpd_resp_send_chunk(req, line, (size_t)n);
    req->user_ctx = (void *)1;
    return true;
}

static esp_err_t h_list(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send_chunk(req, "[\n", 2);
    req->user_ctx = NULL;   /* re-used as "first-entry?" flag inside list_json_cb */
    esp_err_t e = fat_list(list_json_cb, req);
    httpd_resp_send_chunk(req, "\n]\n", 3);
    httpd_resp_send_chunk(req, NULL, 0);
    return e;
}

static esp_err_t fat_out_to_httpd(const void *in, size_t n, void *ctx) {
    httpd_req_t *req = (httpd_req_t *)ctx;
    return httpd_resp_send_chunk(req, (const char *)in, n);
}

static esp_err_t h_fs_get(httpd_req_t *req) {
    char name[13];
    if (!fat_uri_to_name83(req->uri, name))
        return send_text(req, "400 Bad Request", "text/plain", "bad name (8.3 only)\n");

    httpd_resp_set_type(req, "application/octet-stream");
    esp_err_t e = fat_read(name, fat_out_to_httpd, req);
    if (e == ESP_ERR_NOT_FOUND)
        return send_text(req, "404 Not Found", "text/plain", "no such file\n");
    if (e != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain", "read failed\n");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t fat_in_from_httpd(void *out, size_t cap, size_t *got, void *ctx) {
    httpd_req_t *req = (httpd_req_t *)ctx;
    int r = httpd_req_recv(req, (char *)out, cap);
    while (r == HTTPD_SOCK_ERR_TIMEOUT) r = httpd_req_recv(req, (char *)out, cap);
    if (r < 0) return ESP_FAIL;
    *got = (size_t)r;
    return ESP_OK;
}

static esp_err_t h_fs_put(httpd_req_t *req) {
    char name[13];
    if (!fat_uri_to_name83(req->uri, name))
        return send_text(req, "400 Bad Request", "text/plain", "bad name (8.3 only)\n");
    if (req->content_len <= 0)
        return send_text(req, "411 Length Required", "text/plain", "need Content-Length\n");

    esp_err_t e = fat_write(name, fat_in_from_httpd, (size_t)req->content_len, req);
    if (e == ESP_ERR_TIMEOUT)
        return send_text(req, "429 Too Many Requests", "text/plain",
                         "host (DOS) actively writing; retry shortly\n");
    if (e != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain", "write failed\n");
    return send_text(req, "200 OK", "text/plain", "OK\n");
}

static esp_err_t h_fs_delete(httpd_req_t *req) {
    char name[13];
    if (!fat_uri_to_name83(req->uri, name))
        return send_text(req, "400 Bad Request", "text/plain", "bad name (8.3 only)\n");

    esp_err_t e = fat_delete(name);
    if (e == ESP_ERR_NOT_FOUND)
        return send_text(req, "404 Not Found", "text/plain", "no such file\n");
    if (e == ESP_ERR_TIMEOUT)
        return send_text(req, "429 Too Many Requests", "text/plain",
                         "host (DOS) actively writing; retry shortly\n");
    if (e != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain", "delete failed\n");
    return send_text(req, "200 OK", "text/plain", "OK\n");
}

/* GET /slip-stats -- SLIP path counters as JSON. Pollable over WiFi
 * while a SLIP test is running on DOS; diff two snapshots to derive
 * the live byte/packet rate. pbuf_fails > 0 is a smoking gun (lwIP
 * pool exhausted by burst RX); tx_truncs > 0 means an IP packet
 * exceeded our SLIP TX buffer. */
static esp_err_t h_slip_stats(httpd_req_t *req) {
    char buf[256];
    int n = snprintf(buf, sizeof buf,
        "{\"mode\":\"%s\""
        ",\"pkts_to_host\":%u,\"pkts_from_host\":%u"
        ",\"bytes_to_host\":%u,\"bytes_from_host\":%u"
        ",\"pbuf_alloc_fails\":%u,\"tx_truncs\":%u}\n",
        slip_get_mode() == MODE_SLIP ? "SLIP" : "MODEM",
        (unsigned)slip_stat_pkts_to_host(),
        (unsigned)slip_stat_pkts_from_host(),
        (unsigned)slip_stat_bytes_to_host(),
        (unsigned)slip_stat_bytes_from_host(),
        (unsigned)slip_stat_pbuf_fails(),
        (unsigned)slip_stat_tx_truncs());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, (n > 0 && (size_t)n < sizeof buf) ? n : 0);
    return ESP_OK;
}

/* POST /mode?to=SLIP|MODEM -- runtime mode switch from the WiFi
 * side. Always available (independent of CDC) so a stuck SLIP mode
 * with no DOS-side recovery can be unstuck via curl. Persisted to
 * NVS by slip_set_mode. */
static esp_err_t h_mode(httpd_req_t *req) {
    char qbuf[64];
    char to[16] = {0};
    int qlen = httpd_req_get_url_query_len(req);
    if (qlen > 0 && (size_t)qlen < sizeof qbuf) {
        if (httpd_req_get_url_query_str(req, qbuf, sizeof qbuf) == ESP_OK)
            httpd_query_key_value(qbuf, "to", to, sizeof to);
    }
    if (!to[0]) {
        return send_text(req, "200 OK", "text/plain",
                         slip_get_mode() == MODE_SLIP ? "SLIP\n" : "MODEM\n");
    }
    LinkMode want = (LinkMode)-1;
    if      (!strcasecmp(to, "SLIP"))  want = MODE_SLIP;
    else if (!strcasecmp(to, "MODEM")) want = MODE_MODEM;
    else
        return send_text(req, "400 Bad Request", "text/plain", "to=SLIP|MODEM\n");

    if (slip_set_mode(want) != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain", "set_mode failed\n");

    return send_text(req, "200 OK", "text/plain",
                     want == MODE_SLIP ? "SLIP\n" : "MODEM\n");
}

/* /usb-stats: per-callback counters + last MSC op + write-back cache
 * state, as JSON. Hot path is just uint32_t increments in disk.c;
 * formatting cost is borne here on the slow GET path. */
static esp_err_t h_usb_stats(httpd_req_t *req) {
    char buf[512];
    size_t n = disk_stats_json(buf, sizeof buf);
    if (n == 0)
        return send_text(req, "500 Internal Server Error", "text/plain", "stats buffer overflow\n");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static httpd_handle_t s_httpd = NULL;

static void httpd_start_once(void) {
    if (s_httpd) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn     = httpd_uri_match_wildcard;
    cfg.stack_size       = 8192;
    cfg.max_uri_handlers = 24;
    cfg.lru_purge_enable = true;
    /* 2 s was the default; OTA upload kept stalling at ~168 KB because
     * Mac bursts a 64 KB window then waits for ACKs while the dongle is
     * busy in esp_ota_write, recv blocks longer than 2 s, httpd kills
     * the connection. 10 s leaves comfortable margin for slow flash
     * writes during OTA without affecting interactive endpoints (they
     * never block recv anywhere near this long). */
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;
    /* Pin httpd to CPU1 so esp_ota_write doesn't share CPU0 with
     * WiFi/lwIP. With them on the same core, OTA's flash erase
     * starves the lwIP task, ACKs back up, Mac retransmits/resets
     * at ~168 KB. (CDC/TinyUSB are also on CPU1 but mostly idle when
     * not transferring; httpd's bursts of flash-write CPU work are
     * isolated to CPU1, leaving CPU0 free for WiFi-driven ACKs.) */
    cfg.core_id = 1;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        s_httpd = NULL;
        return;
    }
    const httpd_uri_t routes[] = {
        { .uri = "/",          .method = HTTP_GET,  .handler = h_root,      .user_ctx = NULL },
        { .uri = "/status",    .method = HTTP_GET,  .handler = h_status,    .user_ctx = NULL },
        { .uri = "/disk-log",  .method = HTTP_GET,  .handler = h_disk_log,  .user_ctx = NULL },
        { .uri = "/ota",       .method = HTTP_POST, .handler = h_ota,       .user_ctx = NULL },
        { .uri = "/reset",     .method = HTTP_POST, .handler = h_reset,     .user_ctx = NULL },
        { .uri = "/usb-start", .method = HTTP_POST, .handler = h_usb_start, .user_ctx = NULL },
        { .uri = "/usb-stats", .method = HTTP_GET,  .handler = h_usb_stats, .user_ctx = NULL },
        { .uri = "/list",      .method = HTTP_GET,  .handler = h_list,      .user_ctx = NULL },
        { .uri = "/fs/*",      .method = HTTP_GET,    .handler = h_fs_get,    .user_ctx = NULL },
        { .uri = "/fs/*",      .method = HTTP_PUT,    .handler = h_fs_put,    .user_ctx = NULL },
        { .uri = "/fs/*",      .method = HTTP_DELETE, .handler = h_fs_delete, .user_ctx = NULL },
        { .uri = "/mode",      .method = HTTP_POST,   .handler = h_mode,      .user_ctx = NULL },
        { .uri = "/mode",      .method = HTTP_GET,    .handler = h_mode,      .user_ctx = NULL },
        { .uri = "/slip-stats",.method = HTTP_GET,    .handler = h_slip_stats,.user_ctx = NULL },
        { .uri = "/partitions",.method = HTTP_GET,    .handler = h_partitions,.user_ctx = NULL },
        { .uri = "/type",      .method = HTTP_POST,   .handler = h_type,      .user_ctx = NULL },
        { .uri = "/type-log",  .method = HTTP_GET,    .handler = h_type_log,  .user_ctx = NULL },
    };
    for (size_t i = 0; i < sizeof routes / sizeof routes[0]; ++i)
        httpd_register_uri_handler(s_httpd, &routes[i]);
    disk_logf("httpd listening on :80");
}

static void mdns_start_once(void) {
    static bool up = false;
    if (up) return;
    if (mdns_init() != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed");
        return;
    }
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    up = true;
    disk_logf("mdns up as " MDNS_HOSTNAME ".local");
}

/* ===== WiFi STA ====================================================== */

/* Copy at most `dstmax-1` bytes from src and always null-terminate.
 * Uses memcpy (not strncpy) so newer GCC doesn't trip
 * -Wstringop-truncation on the "exactly sizeof-1" idiom. */
static void copy_z(char *dst, size_t dstmax, const char *src) {
    if (dstmax == 0) return;
    size_t n = strnlen(src, dstmax - 1);
    memcpy(dst, src, n);
    dst[n] = 0;
}

static bool s_creds_from_nvs = false;   /* true if load_wifi_creds got
                                         * them from NVS (no need to
                                         * re-save on connect). */

static void load_wifi_creds(char ssid[33], char pass[65]) {
    nvs_handle_t h;
    size_t n;
    ssid[0] = 0; pass[0] = 0;
    s_creds_from_nvs = false;
    esp_err_t e_open = nvs_open("slip-router", NVS_READONLY, &h);
    esp_err_t e_ssid = ESP_FAIL, e_pass = ESP_FAIL;
    if (e_open == ESP_OK) {
        n = 33; e_ssid = nvs_get_str(h, "ssid", ssid, &n);
        n = 65; e_pass = nvs_get_str(h, "pass", pass, &n);
        nvs_close(h);
        if (ssid[0]) s_creds_from_nvs = true;
    }
    disk_logf("nvs/load: open=%d ssid=%d \"%s\" pass=%d (%zu B)",
              (int)e_open, (int)e_ssid, ssid, (int)e_pass, strlen(pass));
    if (!ssid[0]) {
        copy_z(ssid, 33, WIFI_SSID_DEFAULT);
        copy_z(pass, 65, WIFI_PASS_DEFAULT);
    }
}

/* Persist the working creds to NVS once we know they connect. After
 * this, future OTA updates that ship without wifi_creds.h still
 * connect on first boot -- the dongle remembers its network. Matches
 * the arduino-esp32 behaviour (Preferences-backed). Only writes if
 * the creds didn't already come from NVS (avoids needless flash
 * wear). */
static void save_wifi_creds_if_new(const char *ssid, const char *pass) {
    disk_logf("nvs/bootstrap-save: from_nvs=%d ssid[0]=%d \"%s\"",
              s_creds_from_nvs ? 1 : 0, (int)ssid[0], ssid);
    if (s_creds_from_nvs) return;
    if (!ssid[0]) return;
    nvs_handle_t h;
    if (nvs_open("slip-router", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "ssid", ssid);
    nvs_set_str(h, "pass", pass);
    nvs_commit(h);
    nvs_close(h);
    s_creds_from_nvs = true;
    disk_logf("nvs/bootstrap-save: WROTE ssid=\"%s\"", ssid);
}

/* The current creds, kept in static storage so the GOT_IP handler can
 * persist them without re-loading from header (which would lose us
 * the build-time-only freshness check). */
static char s_ssid[33];
static char s_pass[65];

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Throttle the "retrying" log to once every ~30s so the disk-log
         * ring buffer isn't eaten by a tight reconnect loop. */
        static int64_t s_last_retry_log_us = 0;
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_last_retry_log_us > 30LL * 1000 * 1000) {
            disk_logf("wifi disconnected; retrying");
            s_last_retry_log_us = now_us;
        }
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        disk_logf("wifi got IP " IPSTR, IP2STR(&ev->ip_info.ip));
        save_wifi_creds_if_new(s_ssid, s_pass);
        httpd_start_once();
        mdns_start_once();
    }
}

static void wifi_start(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    load_wifi_creds(s_ssid, s_pass);
    if (s_ssid[0] == 0) {
        disk_logf("wifi: no creds in NVS or build defaults; not starting STA");
        ESP_LOGW(TAG, "WiFi has no credentials -- provision via NVS before first OTA-only boot");
        return;
    }

    wifi_config_t wifi_cfg = {0};
    /* wifi_cfg.sta.ssid is uint8_t[32] (NOT null-terminated). Copy
     * up to that many bytes via bounded memcpy. */
    {
        size_t n = strnlen(s_ssid, sizeof wifi_cfg.sta.ssid);
        memcpy(wifi_cfg.sta.ssid, s_ssid, n);
    }
    {
        size_t n = strnlen(s_pass, sizeof wifi_cfg.sta.password);
        memcpy(wifi_cfg.sta.password, s_pass, n);
    }
    wifi_cfg.sta.scan_method        = WIFI_ALL_CHANNEL_SCAN;
    wifi_cfg.sta.sort_method        = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Max TX power -- same rationale as the arduino-esp32 build:
     * stub PCB antenna, sits far from APs. +19.5 dBm is the chip max. */
    esp_wifi_set_max_tx_power(78);   /* 78 * 0.25 dBm = 19.5 dBm */
    disk_logf("wifi: STA started, ssid=\"%s\"", s_ssid);
}

/* ===== app_main ====================================================== */

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Boot capability summary: flash chip, PSRAM, AUTO_SUSPEND state,
     * reset reason. Lives forever in disk-log so we always know how the
     * device booted last. */
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
            default:   vendor = "?";       break;
        }
        size_t psram = 0;
#if CONFIG_SPIRAM
        psram = esp_psram_get_size();
#endif
        disk_logf("init: reset=%s jedec=0x%06lx vendor=%s psram=%u auto_suspend=%s",
                  reset_reason_name(esp_reset_reason()),
                  (unsigned long)jedec, vendor, (unsigned)psram,
#if CONFIG_SPI_FLASH_AUTO_SUSPEND
                  "ENABLED"
#else
                  "DISABLED"
#endif
        );
    }

    wifi_start();

    /* SLIP netif + NAPT come up here. The netif is admin-down by
     * default; entering SLIP mode (HTTP /mode, AT$MODE=, magic frame)
     * flips it up. Needs to be called after wifi_start so the STA
     * netif exists for NAPT to route through. */
    {
        esp_err_t e = slip_init();
        if (e != ESP_OK)
            disk_logf("slip_init failed: %s", esp_err_to_name(e));
    }

    /* DNS forwarder on the SLIP netif IP. Bypasses NAPT for DNS so
     * mTCP queries don't depend on the UDP NAT mapping surviving. */
    dns_forwarder_init();

    /* USB up at boot. Phase 1a deferred this behind POST /usb-start
     * as a safety scaffold while the JTAG -> OTG PHY-mux switch and
     * TinyUSB init were unstable; that path is solid now. The
     * /usb-start endpoint remains for manual retry if init fails. */
    {
        esp_err_t e = usb_start();
        if (e != ESP_OK)
            disk_logf("usb_start at boot failed: %s", esp_err_to_name(e));
    }

    /* Idle forever. WiFi event handler will start HTTP + mDNS once
     * STA is connected. Subsequent phases will start additional tasks
     * (AT engine, SLIP polling) here. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
