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
#include "display.h"
#include "modem.h"

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
 * /disk-log so all events are inspectable without a UART.
 *
 * Mirrored to an RTC_NOINIT buffer so that pre-panic entries survive
 * a soft reset (which includes PANIC). On boot, if the RTC ring's
 * magic checks out, dump_rtc_log_on_boot() drains the survived
 * entries back into the live disk_log so they appear via /disk-log
 * before the new init line. Magic+seq are then cleared so a clean
 * subsequent reboot doesn't double-print. */

#define DISK_LOG_LINES        24       /* 64 -> 48 -> 32 -> 24 (SRAM trim 2026-06-02) */
#define DISK_LOG_LINE_LEN     128      /* prefix + 96-byte msg (kept 128: 96 trips format-truncation) */
#define DISK_LOG_MSG_LEN      96
#define RTC_LOG_LINES         32       /* smaller -- RTC SLOW is precious */
#define RTC_LOG_LINE_LEN      128      /* was 144 -- match DISK_LOG_LINE_LEN */
#define RTC_LOG_MAGIC         0x5044AB1Eu   /* "DiskAble", invented */

static portMUX_TYPE s_disk_log_mux = portMUX_INITIALIZER_UNLOCKED;
static char         s_disk_log[DISK_LOG_LINES][DISK_LOG_LINE_LEN];
static uint32_t     s_disk_log_seq = 0;

/* RTC_NOINIT_ATTR places the variable in RTC SLOW memory and skips
 * zero-init at boot, so it survives any soft reset (PANIC, SW). Lost
 * only on power cycle or hard reset (which we never use). Each entry
 * is a single line of formatted text matching the disk_log shape. */
static RTC_NOINIT_ATTR uint32_t s_rtc_log_magic;
static RTC_NOINIT_ATTR uint32_t s_rtc_log_seq;
static RTC_NOINIT_ATTR char     s_rtc_log[RTC_LOG_LINES][RTC_LOG_LINE_LEN];

/* Used during dump_rtc_log_on_boot() to suppress the RTC mirror -- if
 * we DID mirror the dumped lines back, the next boot would re-dump
 * them with another "[pre-reset]" prefix and so on indefinitely. */
static bool s_disk_log_suppress_rtc = false;

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
    if (!s_disk_log_suppress_rtc) {
        /* RTC mirror -- same critical section so seq order is consistent
         * if disk_logf is called concurrently from multiple cores. */
        s_rtc_log_magic = RTC_LOG_MAGIC;
        s_rtc_log_seq   = seq;
        snprintf(s_rtc_log[seq % RTC_LOG_LINES], RTC_LOG_LINE_LEN,
                 "%" PRIu32 " %s", seq, msg);
    }
    portEXIT_CRITICAL(&s_disk_log_mux);

    /* Also mirror to the standard ESP log so it shows up in the
     * USB-Serial-JTAG console if a developer has it attached. */
    ESP_LOGI(TAG, "[log] %s", msg);
}

/* Pull surviving disk_log entries out of the RTC mirror on boot. Call
 * this BEFORE the new init line so the post-mortem trail appears
 * first in the dumped log. RTC mirror is suppressed during the dump
 * so the replayed "[pre-reset]" lines don't write back to RTC and
 * cause an infinite re-prefix loop on subsequent boots. */
static void dump_rtc_log_on_boot(void) {
    if (s_rtc_log_magic != RTC_LOG_MAGIC) return;   /* cold boot or garbage */
    uint32_t last = s_rtc_log_seq;
    /* Clear FIRST so even if we crash mid-dump the next boot starts
     * clean -- no risk of repeated "[pre-reset]" wrapping. */
    s_rtc_log_magic = 0;
    s_rtc_log_seq   = 0;
    if (last == 0) return;
    uint32_t first = (last > RTC_LOG_LINES) ? (last - RTC_LOG_LINES + 1) : 1;
    s_disk_log_suppress_rtc = true;
    for (uint32_t i = first; i <= last; ++i) {
        const char *line = s_rtc_log[i % RTC_LOG_LINES];
        if (line[0]) disk_logf("[pre-reset] %s", line);
    }
    s_disk_log_suppress_rtc = false;
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
                         "DSL parse error (see /disk-log)\n");
    char ok[32];
    snprintf(ok, sizeof ok, "OK %d\n", n);
    return send_text(req, "200 OK", "text/plain", ok);
}

/* GET /otadata -- dump the two ota_data sectors raw. Lets us see
 * whether esp_ota_set_boot_partition actually wrote a new seq.
 * Each sector starts with esp_ota_select_entry_t: { uint32_t ota_seq;
 * uint8_t seq_label[20]; uint32_t ota_state; uint32_t crc; }. */
static esp_err_t h_otadata(httpd_req_t *req) {
    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
    if (!p) return send_text(req, "404 Not Found", "text/plain", "no otadata\n");

    uint8_t s0[32] = {0}, s1[32] = {0};
    esp_partition_read(p, 0, s0, 32);
    esp_partition_read(p, p->erase_size, s1, 32);

    uint32_t seq0, state0, crc0, seq1, state1, crc1;
    memcpy(&seq0,   s0,     4);
    memcpy(&state0, s0+24,  4);
    memcpy(&crc0,   s0+28,  4);
    memcpy(&seq1,   s1,     4);
    memcpy(&state1, s1+24,  4);
    memcpy(&crc1,   s1+28,  4);

    char buf[512];
    int n = snprintf(buf, sizeof buf,
        "otadata @ 0x%lx (erase_size=%lu)\n"
        "sect0: seq=0x%08lx state=0x%08lx crc=0x%08lx\n"
        "sect1: seq=0x%08lx state=0x%08lx crc=0x%08lx\n"
        "\n"
        "seq active selection: highest non-0xFFFFFFFF wins.\n"
        "Running partition is determined by (seq - 1) %% ota_app_count.\n",
        (unsigned long)p->address, (unsigned long)p->erase_size,
        (unsigned long)seq0, (unsigned long)state0, (unsigned long)crc0,
        (unsigned long)seq1, (unsigned long)state1, (unsigned long)crc1);
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

/* Worker: soft USB unplug/replug. Detached so the HTTP response (over
 * WiFi, unaffected by the USB drop) returns immediately. */
static void usb_reconnect_task(void *arg) {
    unsigned hold_ms = (unsigned)(uintptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(150));   /* let the HTTP 200 flush first */
    usb_soft_reconnect(hold_ms);
    vTaskDelete(NULL);
}

/* POST /usb-reconnect -- firmware-triggered USB unplug/replug. The
 * remote equivalent of physically replugging: forces the host to fully
 * tear down and re-enumerate the composite, which clears a stuck
 * host-side storage/arbitration state (macOS diskarbitrationd) that a
 * chip reset doesn't. Drops CDC + MSC + HID for ~2 s, then they come
 * back. Optional ?hold=<ms> (default 2000, clamped 500..5000). */
static esp_err_t h_usb_reconnect(httpd_req_t *req) {
    unsigned hold = 2000;
    char q[32];
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK) {
        char v[8];
        if (httpd_query_key_value(q, "hold", v, sizeof v) == ESP_OK) {
            long h = strtol(v, NULL, 10);
            if (h >= 500 && h <= 5000) hold = (unsigned)h;
        }
    }
    send_text(req, "200 OK", "text/plain",
              "usb reconnect: dropping USB ~2s then re-enumerating "
              "(CDC port will blip)\n");
    xTaskCreate(usb_reconnect_task, "usb_reconn", 3072,
                (void *)(uintptr_t)hold, 5, NULL);
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
    /* Use OTA_WITH_SEQUENTIAL_WRITES. Defers erase to esp_ota_write
     * which erases each sector lazily just before its first write.
     * - No 15 s upfront full-partition erase (which the OTA_SIZE_UNKNOWN
     *   path triggered, blowing the TCP window during recv).
     * - No risk of writing past an undersized erased range (which the
     *   explicit-size path risks: image > ALIGN_UP(declared,4096) bytes
     *   land on un-erased flash, NOR-AND-only corrupts the image,
     *   bootloader rejects it on next boot and falls back to the
     *   previous partition -- net effect: set_boot_partition returns
     *   ESP_OK but the slot never actually flips.)
     * This is what Espressif's own esp_https_ota uses as its default. */
    esp_err_t be = esp_ota_begin(next, OTA_WITH_SEQUENTIAL_WRITES, &h);
    disk_logf("ota: esp_ota_begin(SEQ) -> %d (%lld us)",
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
    int timeouts = 0;
    int64_t t_start = esp_timer_get_time();
    disk_logf("ota: begin, total=%d", total);
    while (got < total) {
        int want = total - got;
        if (want > 2048) want = 2048;
        int r = httpd_req_recv(req, (char *)buf, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            /* Bound the stall. A dropped link (common on weak WiFi) leaves a
             * half-open connection where recv times out every recv_wait_timeout
             * (~10 s) forever -- the old `continue` looped indefinitely, holding
             * the esp_ota handle + blocking httpd, and drove min_free to ~900 B.
             * After a few consecutive timeouts treat the upload as dead, abort,
             * free, and let httpd recover. (never-starve directive) */
            if (++timeouts >= 3) {
                disk_logf("ota: recv stalled (%d timeouts) at got=%d -- aborting", timeouts, got);
                ok = false; break;
            }
            disk_logf("ota: recv TIMEOUT #%d at got=%d (retry)", timeouts, got);
            continue;
        }
        timeouts = 0;
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

/* The /fs handlers no longer gate on host-activity heuristics. The
 * original dual-mount wedge (dongle FATFS read racing a storming host's
 * MSC reads at the WL lock, hanging the httpd worker) is now prevented
 * structurally: fat.c's diskio takes the shared I/O mutex per sector,
 * the same mutex the MSC read10/write10 path holds, so the two are
 * mutually exclusive and never contend at the lower WL layer. Write
 * coherency against a live host mount is handled by disk_take_for_firmware
 * (flush + ownership + recent-write 429) inside fat_write. The earlier
 * disk_host_mounted()/disk_host_active_io() 409 guards were a blunt
 * heuristic that spuriously blocked all /fs access whenever a host
 * merely polled the LUN (e.g. macOS diskarbitration probing a volume
 * it won't even mount) -- removed in favour of the structural lock. */

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

/* POST /eject -- simulate media removal so the USB host unmounts the
 * volume, clearing the dual-mount hazard for device-side /fs ops.
 * EFFECTIVE ON macOS (fskit honours medium-not-present). INERT ON
 * DOS/CHUSB (no post-enum TUR polling on a composite device) -- see
 * ~/CH375/DOS-EJECT-REMOUNT-IDEAS-2026-06-02.md. */
static esp_err_t h_eject(httpd_req_t *req) {
    disk_eject();
    return send_text(req, "200 OK", "text/plain",
                     "ejected (host should unmount; macOS only -- DOS needs replug/CHUSB cmd)\n");
}

/* POST /mount -- re-present the medium + arm UNIT_ATTENTION so the host
 * remounts. Same macOS-effective / DOS-inert caveat as /eject. */
static esp_err_t h_mount(httpd_req_t *req) {
    disk_mount();
    return send_text(req, "200 OK", "text/plain", "mounted (medium present)\n");
}

/* POST /format -- reformat the MSC volume to a clean FAT12 superfloppy.
 * WIPES ALL FILES. Recovery for a corrupted FAT. */
static esp_err_t h_format(httpd_req_t *req) {
    esp_err_t e = disk_format();
    if (e != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain",
                         "format failed\n");
    return send_text(req, "200 OK", "text/plain",
                     "formatted: clean FAT12 superfloppy, all files wiped\n");
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

/* /tasks: per-task stack high-water (bytes still free at the worst point
 * since boot) so we can trim over-provisioned stacks with DATA, not guesses.
 * stack_free near 0 = do NOT trim; large = reclaimable. Needs
 * CONFIG_FREERTOS_USE_TRACE_FACILITY (enabled). */
static esp_err_t h_tasks(httpd_req_t *req) {
    UBaseType_t n = uxTaskGetNumberOfTasks();
    TaskStatus_t *st = malloc(n * sizeof(TaskStatus_t));
    if (!st) return send_text(req, "500 Internal Server Error", "text/plain", "oom\n");
    n = uxTaskGetSystemState(st, n, NULL);
    httpd_resp_set_type(req, "text/plain");
    char line[80];
    for (UBaseType_t i = 0; i < n; ++i) {
        /* usStackHighWaterMark is the min free stack ever. On ESP-IDF
         * StackType_t is a byte, so this is already in BYTES (not words). */
        snprintf(line, sizeof line, "%-16s pri=%-2u stack_free=%u B\n",
                 st[i].pcTaskName,
                 (unsigned)st[i].uxCurrentPriority,
                 (unsigned)st[i].usStackHighWaterMark);
        httpd_resp_sendstr_chunk(req, line);
    }
    httpd_resp_sendstr_chunk(req, NULL);
    free(st);
    return ESP_OK;
}

/* Last/current call's byte accounting -- localizes any download truncation:
 * rx = bytes the dongle recv()'d from the TCP socket, cdc = bytes it pushed
 * to the CDC host. If rx==cdc==Content-Length but the host saved fewer, the
 * loss is host-side (CHUSB/WGET); if cdc<rx, it's the dongle's CDC write;
 * if rx<Content-Length, the TCP connection closed early. Counters reset at
 * each dial, so read this AFTER a transfer and BEFORE the next one. */
static esp_err_t h_modem_stats(httpd_req_t *req) {
    uint32_t rx = 0, cdc = 0; uint64_t blk_us = 0;
    bool online = modem_get_tput(&rx, &cdc, &blk_us);
    const char *peer = NULL;
    modem_online_peer(&peer);
    char buf[256];
    snprintf(buf, sizeof buf,
        "{\"online\":%s,\"peer\":\"%.64s\",\"rx_bytes\":%lu,\"cdc_bytes\":%lu,"
        "\"cdc_block_us\":%llu}\n",
        online ? "true" : "false", peer ? peer : "",
        (unsigned long)rx, (unsigned long)cdc, (unsigned long long)blk_us);
    return send_text(req, "200 OK", "application/json", buf);
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
        { .uri = "/usb-reconnect", .method = HTTP_POST, .handler = h_usb_reconnect, .user_ctx = NULL },
        { .uri = "/usb-start", .method = HTTP_POST, .handler = h_usb_start, .user_ctx = NULL },
        { .uri = "/usb-stats", .method = HTTP_GET,  .handler = h_usb_stats, .user_ctx = NULL },
        { .uri = "/tasks",     .method = HTTP_GET,  .handler = h_tasks,     .user_ctx = NULL },
        { .uri = "/modem-stats", .method = HTTP_GET, .handler = h_modem_stats, .user_ctx = NULL },
        { .uri = "/list",      .method = HTTP_GET,  .handler = h_list,      .user_ctx = NULL },
        { .uri = "/fs/*",      .method = HTTP_GET,    .handler = h_fs_get,    .user_ctx = NULL },
        { .uri = "/fs/*",      .method = HTTP_PUT,    .handler = h_fs_put,    .user_ctx = NULL },
        { .uri = "/fs/*",      .method = HTTP_DELETE, .handler = h_fs_delete, .user_ctx = NULL },
        { .uri = "/mode",      .method = HTTP_POST,   .handler = h_mode,      .user_ctx = NULL },
        { .uri = "/mode",      .method = HTTP_GET,    .handler = h_mode,      .user_ctx = NULL },
        { .uri = "/slip-stats",.method = HTTP_GET,    .handler = h_slip_stats,.user_ctx = NULL },
        { .uri = "/partitions",.method = HTTP_GET,    .handler = h_partitions,.user_ctx = NULL },
        { .uri = "/type",      .method = HTTP_POST,   .handler = h_type,      .user_ctx = NULL },
        { .uri = "/eject",     .method = HTTP_POST,   .handler = h_eject,     .user_ctx = NULL },
        { .uri = "/mount",     .method = HTTP_POST,   .handler = h_mount,     .user_ctx = NULL },
        { .uri = "/format",    .method = HTTP_POST,   .handler = h_format,    .user_ctx = NULL },
        /* /type-log removed -- HID events now go to /disk-log. */
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
static bool s_persist_on_connect = false; /* true when runtime-provisioned
                                         * creds (AT$WIFI=/AT$SSID=) are
                                         * applied but NOT yet written to
                                         * NVS -- we persist them only once
                                         * GOT_IP proves they actually work,
                                         * so a bad/stray command can never
                                         * clobber the working network. */

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
/* Unconditional NVS cred write. Only ever called once the creds are PROVEN
 * to connect (GOT_IP). Before this change AT$WIFI=/AT$SSID= wrote NVS
 * immediately, so a bad or stray command (or CDC-line garbage parsed as AT)
 * permanently clobbered the working network -- e.g. NVS ending up holding
 * ssid="ssid". Persisting only on a verified connect makes that impossible. */
static esp_err_t nvs_write_creds(const char *ssid, const char *pass) {
    if (!ssid || !ssid[0]) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    if (nvs_open("slip-router", NVS_READWRITE, &h) != ESP_OK) return ESP_FAIL;
    nvs_set_str(h, "ssid", ssid);
    nvs_set_str(h, "pass", pass ? pass : "");
    nvs_commit(h);
    nvs_close(h);
    disk_logf("nvs/creds: WROTE ssid=\"%s\" (verified-connected)", ssid);
    return ESP_OK;
}

static void save_wifi_creds_if_new(const char *ssid, const char *pass) {
    if (s_creds_from_nvs) return;
    if (!ssid[0]) return;
    if (nvs_write_creds(ssid, pass) == ESP_OK) s_creds_from_nvs = true;
}

/* The current creds, kept in static storage so the GOT_IP handler can
 * persist them without re-loading from header (which would lose us
 * the build-time-only freshness check). */
static char s_ssid[33];
static char s_pass[65];

/* WiFi link state for the relay's grace-period teardown (modem.c polls
 * wifi_down_us()). Updated only on the up<->down edges in on_wifi_event. */
static volatile bool    s_wifi_up = false;
static volatile int64_t s_wifi_down_us = 0;

/* Microseconds the WiFi link has been continuously down, or 0 if it is up
 * (or has never connected). The relay uses this for a grace-period teardown:
 * short blips are ridden out by TCP, longer drops trigger a clean NO CARRIER. */
int64_t wifi_down_us(void) {
    if (s_wifi_up || s_wifi_down_us == 0) return 0;
    return esp_timer_get_time() - s_wifi_down_us;
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Timestamp the up->down edge ONCE (a reconnect storm re-fires this
         * event; we must not keep resetting the clock or the grace never
         * expires). The relay's grace teardown reads wifi_down_us(). */
        if (s_wifi_up) s_wifi_down_us = esp_timer_get_time();
        s_wifi_up = false;
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
        s_wifi_up = true;   /* link back up -> relay grace clock clears */
        disk_logf("wifi got IP " IPSTR, IP2STR(&ev->ip_info.ip));
        if (s_persist_on_connect) {
            /* Runtime-provisioned creds (AT$WIFI=/AT$SSID=) just proved they
             * work -- NOW it is safe to persist them, overwriting whatever
             * was there. Until this point NVS was untouched, so a command
             * that never connected left the working creds intact. */
            nvs_write_creds(s_ssid, s_pass);
            s_persist_on_connect = false;
            s_creds_from_nvs = true;
        } else {
            save_wifi_creds_if_new(s_ssid, s_pass);
        }
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
    /* Disable WiFi power save BEFORE start. Default mode is
     * WIFI_PS_MIN_MODEM which sleeps the radio between beacons --
     * fine for low-power, costs throughput on TCP streams because
     * the AP buffers ACK-driven flows during sleep windows. We're
     * USB-powered, so the power savings aren't worth the latency. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Max TX power -- same rationale as the arduino-esp32 build:
     * stub PCB antenna, sits far from APs. +19.5 dBm is the chip max. */
    esp_wifi_set_max_tx_power(78);   /* 78 * 0.25 dBm = 19.5 dBm */
    disk_logf("wifi: STA started, ssid=\"%s\"", s_ssid);
}

/* Runtime (re)provision of WiFi creds from the modem (AT$WIFI=/AT$SSID=).
 * Applies the creds to the running radio and connects, but does NOT write
 * NVS -- persistence is deferred to the GOT_IP handler, so creds that never
 * associate cannot overwrite the working network. Called from modem.c. */
void wifi_provision(const char *ssid, const char *pass) {
    if (!ssid) ssid = "";
    if (!pass) pass = "";
    copy_z(s_ssid, sizeof s_ssid, ssid);
    copy_z(s_pass, sizeof s_pass, pass);
    s_persist_on_connect = true;     /* persist iff this actually connects */
    s_creds_from_nvs = false;

    wifi_config_t wc = {0};
    size_t n = strnlen(s_ssid, sizeof wc.sta.ssid);
    memcpy(wc.sta.ssid, s_ssid, n);
    n = strnlen(s_pass, sizeof wc.sta.password);
    memcpy(wc.sta.password, s_pass, n);
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
    disk_logf("wifi: provision ssid=\"%s\" (apply+connect, NVS deferred)", s_ssid);
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_connect();
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
        /* Drain any pre-reset disk_log entries from RTC NOINIT first
         * so the post-mortem trail lands before the new init line. */
        dump_rtc_log_on_boot();
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

    /* Status LCD (0.96" ST7735). Brought up last so the state accessors
     * it polls (wifi/IP, slip stats, modem online/peer, CDC) all exist.
     * Self-contained: spawns its own task and degrades gracefully if the
     * panel fails to init. */
    display_init();

    /* Idle forever. WiFi event handler will start HTTP + mDNS once
     * STA is connected. Subsequent phases will start additional tasks
     * (AT engine, SLIP polling) here. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
