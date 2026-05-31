/* modem.c -- Hayes AT modem on TinyUSB CDC interface 0.
 *
 * Phase 1c.0 scaffold. Receives bytes via tusb_cdc_acm's RX callback,
 * accumulates them into a command-line buffer, and dispatches on CR.
 * Replies and verbose / numeric per the ATV setting -- same wire-level
 * shape as the arduino-esp32 build (tdongle-s3/src/modem.cpp).
 *
 * Things NOT here yet (and the file they came from in the old build):
 *   - TCP dial (ATD)              -- modem.cpp:dial()
 *   - Online-mode data pump       -- modem.cpp:modem_poll() under online=true
 *   - Escape sequence (+++)       -- modem.cpp:modem_poll() one-second guard
 *   - DNS / PING / NETIF / SCAN   -- modem.cpp:handle_dollar()
 *   - HID typing relay (AT$TYPE)  -- modem.cpp:handle_dollar()
 *   - OTA over CDC (AT$OTASTART)  -- modem.cpp:handle_dollar()
 * Bring these in incrementally; the scaffold here is the spine.
 */

#include "modem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "tusb.h"
#include "tusb_cdc_acm.h"
#include "class/cdc/cdc_device.h"

#include "disk.h"   /* for disk_logf */

#define TAG "modem"

/* ---- terminal / line state ---- */
#define CMD_LINE_MAX 256
static char     s_cmd[CMD_LINE_MAX];
static uint16_t s_cmd_len      = 0;
static bool     s_cmd_overflow = false;   /* >= CMD_LINE_MAX before CR */

/* Hayes runtime knobs. Defaults match the AT&F state in the old
 * modem.cpp; AT&W persists these to NVS, AT&V dumps them. */
static bool s_echo    = true;             /* ATE -- echo command bytes back */
static bool s_verbose = true;             /* ATV -- "OK\r\n" vs "0\r\n" */
static bool s_quiet   = false;            /* ATQ -- suppress result codes  */

/* ---- CDC write helpers ----
 *
 * TinyUSB's CDC TX FIFO is bounded (CFG_TUD_CDC_TX_BUFSIZE = 512).
 * Long replies (AT$HELP) won't fit in one write -- loop until done. */

static size_t cdc_write(const void *buf, size_t n) {
    const uint8_t *p = (const uint8_t *)buf;
    size_t sent = 0;
    while (sent < n) {
        if (!tud_cdc_n_connected(0)) break;
        size_t avail = tud_cdc_n_write_available(0);
        if (avail == 0) {
            tud_cdc_n_write_flush(0);
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        size_t chunk = (n - sent) < avail ? (n - sent) : avail;
        size_t w = tud_cdc_n_write(0, p + sent, chunk);
        if (w == 0) {
            tud_cdc_n_write_flush(0);
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        sent += w;
    }
    tud_cdc_n_write_flush(0);
    return sent;
}

static inline void cdc_print(const char *s) { cdc_write(s, strlen(s)); }
static inline void cdc_byte(uint8_t b)      { cdc_write(&b, 1); }

/* ---- result codes ---- */
static void r_ok(void) {
    if (s_quiet) return;
    cdc_print(s_verbose ? "\r\nOK\r\n" : "0\r\n");
}
static void r_error(void) {
    if (s_quiet) return;
    cdc_print(s_verbose ? "\r\nERROR\r\n" : "4\r\n");
}

/* ---- parser helpers ---- */

/* Strip leading "AT" / "at", then uppercase the remainder in place
 * (except the parts inside quoted strings, which we don't actually
 * parse yet -- this is the scaffold). Returns NULL if the line
 * didn't start with AT. */
static char *strip_at(char *line) {
    while (*line == ' ') line++;
    if (line[0] != 'A' && line[0] != 'a') return NULL;
    if (line[1] != 'T' && line[1] != 't') return NULL;
    char *rest = line + 2;
    for (char *p = rest; *p; ++p) *p = (char)toupper((unsigned char)*p);
    return rest;
}

/* ---- AT$ commands ---- */

static void print_wifi_status(void) {
    cdc_print("\r\n");
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        char line[128];
        snprintf(line, sizeof line,
                 "SSID:    %.32s\r\n"
                 "RSSI:    %d dBm\r\n"
                 "Channel: %u\r\n",
                 (const char *)ap.ssid, (int)ap.rssi, (unsigned)ap.primary);
        cdc_print(line);

        esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (sta) {
            esp_netif_ip_info_t ip = {0};
            esp_netif_get_ip_info(sta, &ip);
            char ips[64];
            snprintf(ips, sizeof ips,
                     "IP:      " IPSTR "\r\n"
                     "GW:      " IPSTR "\r\n",
                     IP2STR(&ip.ip), IP2STR(&ip.gw));
            cdc_print(ips);
        }
        cdc_print("status:  connected\r\n");
    } else {
        cdc_print("status:  not connected\r\n");
    }
}

static void handle_dollar(char *s) {
    /* s points to the char after the "$", e.g. "HELP" or "WIFI?".  */
    if (!strcmp(s, "HELP")) {
        cdc_print(
            "\r\n"
            "AT          OK                          ATE0/1     echo off/on\r\n"
            "ATI         identity                    ATV0/1     numeric/verbose\r\n"
            "ATZ         soft reset settings         ATQ0/1     result codes on/off\r\n"
            "AT$WIFI?    show current WiFi state     AT$HELP    this help\r\n"
            "\r\n"
            "Not yet implemented (Phase 1c.1+): AT$WIFI=, ATD, ATO, +++, ATH,\r\n"
            "AT$DNS=, AT$PING=, AT$NETIF, AT$SCAN, AT$TYPE=, AT$RESET, AT$OTASTART=\r\n"
        );
        r_ok();
    } else if (!strcmp(s, "WIFI?")) {
        print_wifi_status();
        r_ok();
    } else {
        r_error();
    }
}

/* ---- exec one AT line ---- */

static void exec(char *line) {
    char *p = strip_at(line);
    if (!p) {
        /* No AT prefix -- ignore (matches Hayes "non-command line" silent
         * behaviour). The old firmware did the same. */
        return;
    }
    /* Bare "AT" -- just return OK. */
    if (!*p) { r_ok(); return; }

    /* Hayes single-character commands. The old firmware uses a switch
     * here, dispatching on the first letter of the suffix. Keep that
     * shape; it's familiar and obvious. */
    switch (*p) {
        case 'E': {
            /* ATE0 = echo off, ATE1 = echo on, bare ATE = ATE1 */
            s_echo = (p[1] != '0');
            r_ok();
        } return;
        case 'V': {
            s_verbose = (p[1] != '0');
            r_ok();
        } return;
        case 'Q': {
            s_quiet = (p[1] == '1');
            r_ok();
        } return;
        case 'I': {
            cdc_print("\r\nDOSongle Modem (Phase 1c.0)\r\n");
            r_ok();
        } return;
        case 'Z': {
            s_echo = true; s_verbose = true; s_quiet = false;
            r_ok();
        } return;
        case '$': {
            handle_dollar(p + 1);
        } return;
        default:
            r_error();
            return;
    }
}

/* ---- CDC RX callback (TinyUSB task context) ---- */

static void on_cdc_rx(int itf, cdcacm_event_t *event) {
    (void)event;
    uint8_t buf[64];
    size_t got = 0;
    if (tinyusb_cdcacm_read(itf, buf, sizeof buf, &got) != ESP_OK) return;
    for (size_t i = 0; i < got; ++i) {
        uint8_t b = buf[i];
        if (b == '\r' || b == '\n') {
            if (s_echo) cdc_print("\r\n");
            if (s_cmd_overflow) {
                r_error();
                s_cmd_overflow = false;
            } else {
                s_cmd[s_cmd_len] = '\0';
                exec(s_cmd);
            }
            s_cmd_len = 0;
            continue;
        }
        if (b == 0x08 || b == 0x7F) { /* BS / DEL */
            if (s_cmd_len > 0) {
                s_cmd_len--;
                if (s_echo) cdc_print("\b \b");
            }
            continue;
        }
        if (b < 0x20 || b > 0x7E) continue; /* drop control + non-ASCII */
        if (s_cmd_len + 1 < sizeof s_cmd) {
            s_cmd[s_cmd_len++] = (char)b;
            if (s_echo) cdc_byte(b);
        } else {
            s_cmd_overflow = true;
        }
    }
}

/* ---- public entry ---- */

esp_err_t modem_init(void) {
    esp_err_t e = tinyusb_cdcacm_register_callback(
        TINYUSB_CDC_ACM_0, CDC_EVENT_RX, on_cdc_rx);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "cdc rx cb register: %s", esp_err_to_name(e));
        return e;
    }
    disk_logf("modem: AT scaffold ready on CDC0");
    return ESP_OK;
}
