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

static void disk_logf(const char *fmt, ...) {
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
    int total = req->content_len;
    if (total <= 0)
        return send_text(req, "411 Length Required", "text/plain",
                         "need Content-Length\n");

    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (!next)
        return send_text(req, "500 Internal Server Error", "text/plain",
                         "no OTA partition\n");
    if ((size_t)total > next->size)
        return send_text(req, "413 Payload Too Large", "text/plain",
                         "exceeds OTA slot size\n");

    esp_ota_handle_t h = 0;
    if (esp_ota_begin(next, OTA_SIZE_UNKNOWN, &h) != ESP_OK)
        return send_text(req, "500 Internal Server Error", "text/plain",
                         "esp_ota_begin failed\n");

    uint8_t *buf = malloc(2048);
    if (!buf) {
        esp_ota_abort(h);
        return send_text(req, "500 Internal Server Error", "text/plain", "oom\n");
    }

    int got = 0;
    bool ok = true;
    while (got < total) {
        int want = total - got;
        if (want > 2048) want = 2048;
        int r = httpd_req_recv(req, (char *)buf, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) { ok = false; break; }
        if (esp_ota_write(h, buf, (size_t)r) != ESP_OK) { ok = false; break; }
        got += r;
    }
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

static httpd_handle_t s_httpd = NULL;

static void httpd_start_once(void) {
    if (s_httpd) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn     = httpd_uri_match_wildcard;
    cfg.stack_size       = 8192;
    cfg.max_uri_handlers = 16;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 2;
    cfg.send_wait_timeout = 2;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        s_httpd = NULL;
        return;
    }
    const httpd_uri_t routes[] = {
        { .uri = "/",         .method = HTTP_GET,  .handler = h_root,     .user_ctx = NULL },
        { .uri = "/status",   .method = HTTP_GET,  .handler = h_status,   .user_ctx = NULL },
        { .uri = "/disk-log", .method = HTTP_GET,  .handler = h_disk_log, .user_ctx = NULL },
        { .uri = "/ota",      .method = HTTP_POST, .handler = h_ota,      .user_ctx = NULL },
        { .uri = "/reset",    .method = HTTP_POST, .handler = h_reset,    .user_ctx = NULL },
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
    if (nvs_open("slip-router", NVS_READONLY, &h) == ESP_OK) {
        n = 33; nvs_get_str(h, "ssid", ssid, &n);
        n = 65; nvs_get_str(h, "pass", pass, &n);
        nvs_close(h);
        if (ssid[0]) s_creds_from_nvs = true;
    }
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
    if (s_creds_from_nvs) return;
    if (!ssid[0]) return;
    nvs_handle_t h;
    if (nvs_open("slip-router", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "ssid", ssid);
    nvs_set_str(h, "pass", pass);
    nvs_commit(h);
    nvs_close(h);
    s_creds_from_nvs = true;
    disk_logf("wifi: bootstrap creds saved to NVS");
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
        disk_logf("wifi disconnected; retrying");
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

    /* Idle forever. WiFi event handler will start HTTP + mDNS once
     * STA is connected. Subsequent phases will start additional tasks
     * (USB, AT engine, SLIP polling) here. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
