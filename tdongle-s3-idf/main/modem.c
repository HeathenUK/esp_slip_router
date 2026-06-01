/* modem.c -- Hayes AT modem on TinyUSB CDC interface 0.
 *
 * Phase 1c (full set ported from the arduino-esp32 build's
 * tdongle-s3/src/modem.cpp). The wire-level behaviour matches that
 * firmware exactly so the DOS-side host stack (FOSSLIP + CHUSB +
 * mtcpget) sees a drop-in replacement.
 *
 * Command-mode bytes arrive via the CDC RX callback (TinyUSB task
 * context). When a successful ATD opens a TCP socket, we flip into
 * online mode and spawn modem_data_task to pump socket -> CDC; the
 * RX callback keeps draining CDC -> socket. Either direction can
 * trigger return-to-command (a +++ escape from CDC, a socket close
 * from the peer).
 *
 * Telnet IAC handling (RFC 854 + a few common options) is on by
 * default; binary-mode hosts negotiate it off via DO/WILL BINARY.
 *
 * NVS layout (shared with main.c's bootstrap path):
 *   slip-router/ssid  (string, max 32)
 *   slip-router/pass  (string, max 64)
 */

#include "modem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"

#include "tusb.h"
#include "tusb_cdc_acm.h"
#include "class/cdc/cdc_device.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "disk.h"   /* disk_logf */
#include "slip.h"

#define TAG "modem"

/* ---- command-mode state ---- */
#define CMD_LINE_MAX 256
static char     s_cmd[CMD_LINE_MAX];
static uint16_t s_cmd_len      = 0;
static bool     s_cmd_overflow = false;

static bool s_echo    = true;
static bool s_verbose = true;
static bool s_quiet   = false;
static bool s_telnet  = true;     /* Telnet IAC handling on by default */

/* ---- NVS-persisted modem flags (E/V/N) ----
 *
 * Old build's AT&W packs E/V/N into a single byte; AT&F resets to
 * all-on (matching ATZ defaults). modem_load_evn() is called at
 * init; modem_save_evn() persists current state. */
static void modem_load_evn(void) {
    nvs_handle_t h;
    if (nvs_open("slip-router", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t b = 0x07;   /* default: all-on */
    nvs_get_u8(h, "evn", &b);
    nvs_close(h);
    s_echo    = (b & 1) != 0;
    s_verbose = (b & 2) != 0;
    s_telnet  = (b & 4) != 0;
}
static void modem_save_evn(void) {
    nvs_handle_t h;
    if (nvs_open("slip-router", NVS_READWRITE, &h) != ESP_OK) return;
    uint8_t b = (s_echo ? 1 : 0) | (s_verbose ? 2 : 0) | (s_telnet ? 4 : 0);
    nvs_set_u8(h, "evn", b);
    nvs_commit(h);
    nvs_close(h);
}

/* ---- online-mode state ---- */
static int       s_sock = -1;
static volatile bool s_online = false;
static char      s_peer[96] = {0};       /* "host:port" string for AT$STATUS */
static TaskHandle_t s_data_task = NULL;

/* +++ escape sequence detector. Hayes rule: 1 s of guard, then exactly
 * three '+' within 1 s, then 1 s of guard with no other data. */
#define GUARD_US 1000000
static volatile uint8_t  s_plus_count    = 0;
static volatile int64_t  s_last_data_us  = 0;
static volatile int64_t  s_plus_time_us  = 0;

/* ---- telnet ---- */
#define TN_IAC  0xFF
#define TN_WILL 0xFB
#define TN_WONT 0xFC
#define TN_DO   0xFD
#define TN_DONT 0xFE
#define TN_SB   0xFA
#define TN_SE   0xF0
#define OPT_BINARY 0
#define OPT_ECHO   1
#define OPT_SGA    3
#define OPT_TTYPE  24
#define OPT_NAWS   31
#define TT_IS      0
#define TT_SEND    1
#define TERM_TYPE  "ANSI"
#define TERM_COLS  80
#define TERM_ROWS  25

/* Telnet RX state machine -- this is what we feed bytes-from-TCP into
 * to strip the IAC negotiation noise before relaying payload to CDC. */
enum { T_DATA, T_IAC, T_OPT, T_SB_OPT, T_SB_DATA, T_SB_IAC };
static int     s_tstate = T_DATA;
static uint8_t s_tcmd   = 0;
static uint8_t s_sbopt  = 0;
static uint8_t s_sbbuf[16];
static uint8_t s_sblen  = 0;
/* Tracked option flags so we don't echo the same WILL/DO repeatedly. */
static uint8_t s_remote_on[32], s_local_on[32], s_local_offered[32];
static inline bool bget(uint8_t *a, uint8_t o) { return a[o >> 3] & (1 << (o & 7)); }
static inline void bset(uint8_t *a, uint8_t o) { a[o >> 3] |=  (1 << (o & 7)); }
static inline void bclr(uint8_t *a, uint8_t o) { a[o >> 3] &= ~(1 << (o & 7)); }

/* ---- CDC write helper ---- */

static size_t cdc_write(const void *buf, size_t n) {
    const uint8_t *p = (const uint8_t *)buf;
    size_t sent = 0;
    int64_t deadline = esp_timer_get_time() + 500000;  /* 500 ms */
    while (sent < n && esp_timer_get_time() < deadline) {
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
static void r_ok(void)         { if (!s_quiet) cdc_print(s_verbose ? "\r\nOK\r\n"         : "0\r\n"); }
static void r_error(void)      { if (!s_quiet) cdc_print(s_verbose ? "\r\nERROR\r\n"      : "4\r\n"); }
static void r_connect(void)    { if (!s_quiet) cdc_print(s_verbose ? "\r\nCONNECT\r\n"    : "1\r\n"); }
static void r_nocarrier(void)  { if (!s_quiet) cdc_print(s_verbose ? "\r\nNO CARRIER\r\n" : "3\r\n"); }

/* ---- telnet helpers ---- */

static void tn_send3(uint8_t cmd, uint8_t opt) {
    uint8_t b[3] = {TN_IAC, cmd, opt};
    if (s_sock >= 0) send(s_sock, b, 3, 0);
}
static bool tn_accept_remote(uint8_t o) { return o == OPT_BINARY || o == OPT_SGA || o == OPT_ECHO; }
static bool tn_offer_local(uint8_t o)   { return o == OPT_BINARY || o == OPT_SGA || o == OPT_TTYPE || o == OPT_NAWS; }
static void tn_offer_will(uint8_t opt) {
    if (!bget(s_local_offered, opt)) { bset(s_local_offered, opt); tn_send3(TN_WILL, opt); }
}
static void tn_send_naws(void) {
    uint8_t b[] = {TN_IAC, TN_SB, OPT_NAWS, 0, TERM_COLS, 0, TERM_ROWS, TN_IAC, TN_SE};
    if (s_sock >= 0) send(s_sock, b, sizeof b, 0);
}
static void tn_after_local_enable(uint8_t opt) {
    if (opt == OPT_NAWS) tn_send_naws();
}
static void tn_handle_neg(uint8_t c, uint8_t opt) {
    switch (c) {
        case TN_WILL:
            if (tn_accept_remote(opt)) {
                if (!bget(s_remote_on, opt)) { bset(s_remote_on, opt); tn_send3(TN_DO, opt); }
            } else {
                bclr(s_remote_on, opt); tn_send3(TN_DONT, opt);
            }
            break;
        case TN_WONT:
            if (bget(s_remote_on, opt)) { bclr(s_remote_on, opt); tn_send3(TN_DONT, opt); }
            break;
        case TN_DO:
            if (tn_offer_local(opt)) {
                if (!bget(s_local_on, opt)) {
                    bset(s_local_on, opt);
                    if (!bget(s_local_offered, opt)) { bset(s_local_offered, opt); tn_send3(TN_WILL, opt); }
                    tn_after_local_enable(opt);
                }
            } else {
                tn_send3(TN_WONT, opt);
            }
            break;
        case TN_DONT:
            if (bget(s_local_on, opt)) { bclr(s_local_on, opt); tn_send3(TN_WONT, opt); }
            break;
    }
}
static void tn_handle_sb(void) {
    if (s_sbopt == OPT_TTYPE && s_sblen >= 1 && s_sbbuf[0] == TT_SEND) {
        uint8_t hdr[] = {TN_IAC, TN_SB, OPT_TTYPE, TT_IS};
        if (s_sock >= 0) {
            send(s_sock, hdr, sizeof hdr, 0);
            send(s_sock, (const uint8_t *)TERM_TYPE, strlen(TERM_TYPE), 0);
            uint8_t tail[] = {TN_IAC, TN_SE};
            send(s_sock, tail, sizeof tail, 0);
        }
    }
}
static void tn_start(void) {
    memset(s_remote_on, 0, sizeof s_remote_on);
    memset(s_local_on, 0, sizeof s_local_on);
    memset(s_local_offered, 0, sizeof s_local_offered);
    s_sblen = 0; s_tstate = T_DATA;
    if (!s_telnet) return;
    /* Opener: offer the options that make a BBS session clean. */
    tn_offer_will(OPT_TTYPE);
    tn_offer_will(OPT_NAWS);
    tn_offer_will(OPT_SGA);    tn_send3(TN_DO, OPT_SGA);
    tn_offer_will(OPT_BINARY); tn_send3(TN_DO, OPT_BINARY);
}

/* ---- TCP -> CDC pump (drains payload bytes, runs telnet IAC machine
 * inline so option negotiation never reaches the user terminal) ---- */

static void modem_data_task(void *arg) {
    (void)arg;
    /* Loose 100 ms poll. The recv timeout means we wake periodically
     * to check for +++ guard expiry / socket close even when the peer
     * is silent. */
    struct timeval rcv_tv = { .tv_sec = 0, .tv_usec = 100 * 1000 };
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof rcv_tv);

    /* Sized to the CDC TX FIFO (2 KB) so one recv() worth of bytes fits
     * in a single FIFO drain without spinning on backpressure -- bumping
     * higher costs SRAM that WiFi heap needs more. outbuf only used on
     * the telnet path; binary path pushes inbuf directly. */
    static uint8_t inbuf[2048];
    static uint8_t outbuf[2048];
    bool peer_closed = false;

    while (s_online && s_sock >= 0) {
        int n = recv(s_sock, inbuf, sizeof inbuf, 0);
        if (n > 0 && !s_telnet) {
            /* Binary fast path: skip the per-byte IAC state machine when
             * telnet is off -- push inbuf straight to CDC. Phase 2 (HTTP)
             * hits this. */
            cdc_write(inbuf, (size_t)n);
        } else if (n > 0) {
            size_t outlen = 0;
            for (int i = 0; i < n; ++i) {
                uint8_t ch = inbuf[i];
                switch (s_tstate) {
                    case T_DATA:
                        if (ch == TN_IAC) s_tstate = T_IAC;
                        else {
                            outbuf[outlen++] = ch;
                            if (outlen >= sizeof outbuf) { cdc_write(outbuf, outlen); outlen = 0; }
                        }
                        break;
                    case T_IAC:
                        if (ch == TN_IAC) {
                            outbuf[outlen++] = TN_IAC;
                            if (outlen >= sizeof outbuf) { cdc_write(outbuf, outlen); outlen = 0; }
                            s_tstate = T_DATA;
                        } else if (ch == TN_WILL || ch == TN_WONT || ch == TN_DO || ch == TN_DONT) {
                            s_tcmd = ch; s_tstate = T_OPT;
                        } else if (ch == TN_SB) {
                            s_tstate = T_SB_OPT;
                        } else {
                            s_tstate = T_DATA;
                        }
                        break;
                    case T_OPT:
                        tn_handle_neg(s_tcmd, ch); s_tstate = T_DATA;
                        break;
                    case T_SB_OPT:
                        s_sbopt = ch; s_sblen = 0; s_tstate = T_SB_DATA;
                        break;
                    case T_SB_DATA:
                        if (ch == TN_IAC) s_tstate = T_SB_IAC;
                        else if (s_sblen < sizeof s_sbbuf) s_sbbuf[s_sblen++] = ch;
                        break;
                    case T_SB_IAC:
                        if (ch == TN_SE) { tn_handle_sb(); s_tstate = T_DATA; }
                        else { if (s_sblen < sizeof s_sbbuf) s_sbbuf[s_sblen++] = ch; s_tstate = T_SB_DATA; }
                        break;
                }
            }
            if (outlen) cdc_write(outbuf, outlen);
        } else if (n == 0) {
            peer_closed = true;
            break;
        } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            disk_logf("modem: recv err=%d", errno);
            peer_closed = true;
            break;
        }

        /* +++ guard expired -> command mode, socket stays open (ATO returns). */
        int64_t now = esp_timer_get_time();
        if (s_plus_count == 3 && (now - s_plus_time_us) > GUARD_US) {
            s_plus_count = 0;
            s_online = false;
            r_ok();
            break;
        }
    }

    if (peer_closed) {
        s_online = false;
        if (s_sock >= 0) { close(s_sock); s_sock = -1; }
        s_peer[0] = 0;
        r_nocarrier();
    }
    s_data_task = NULL;
    vTaskDelete(NULL);
}

/* ---- CDC -> TCP push (called from on_cdc_rx when online) ---- */

static void online_push_bytes(const uint8_t *buf, size_t n) {
    if (s_sock < 0) return;
    uint8_t txbuf[1024];  /* worst case for telnet IAC + CRLF doubling */
    size_t txlen = 0;
    int64_t now = esp_timer_get_time();

    for (size_t i = 0; i < n; ++i) {
        uint8_t ch = buf[i];
        if (ch == '+' && s_plus_count < 3 &&
            (s_plus_count > 0 || (now - s_last_data_us) > GUARD_US)) {
            /* Hold the +'s back until we know whether the escape
             * completes (3 +'s + 1 s silence) or breaks. */
            if (txlen) { send(s_sock, txbuf, txlen, 0); txlen = 0; }
            s_plus_count++;
            s_plus_time_us = now;
            continue;
        }
        /* Any non-'+' (or 4th '+', etc.) breaks the escape: flush the
         * pending +'s as real data, then process this byte normally. */
        while (s_plus_count > 0) { txbuf[txlen++] = '+'; s_plus_count--; }

        if (s_telnet && ch == 0x0D && !bget(s_local_on, OPT_BINARY)) {
            /* Telnet NVT CR -> CR LF */
            txbuf[txlen++] = 0x0D;
            txbuf[txlen++] = 0x0A;
        } else {
            if (s_telnet && ch == TN_IAC) txbuf[txlen++] = TN_IAC;  /* escape */
            txbuf[txlen++] = ch;
        }
        s_last_data_us = now;

        if (txlen >= sizeof txbuf - 2) {
            send(s_sock, txbuf, txlen, 0); txlen = 0;
        }
    }
    if (txlen) send(s_sock, txbuf, txlen, 0);
}

/* ---- AT$ command handlers ---- */

static esp_err_t save_wifi_creds(const char *ssid, const char *pass) {
    nvs_handle_t h;
    esp_err_t e = nvs_open("slip-router", NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    nvs_set_str(h, "ssid", ssid);
    nvs_set_str(h, "pass", pass);
    nvs_commit(h);
    nvs_close(h);
    return ESP_OK;
}

static void cmd_wifi_set(const char *arg) {
    /* AT$WIFI=ssid,password -- comma is the separator; ssids with
     * literal commas aren't supported (matches old firmware). */
    const char *comma = strchr(arg, ',');
    if (!comma) { r_error(); return; }
    char ssid[33] = {0};
    char pass[65] = {0};
    size_t sl = (size_t)(comma - arg);
    if (sl == 0 || sl >= sizeof ssid) { r_error(); return; }
    memcpy(ssid, arg, sl);
    strncpy(pass, comma + 1, sizeof pass - 1);

    if (save_wifi_creds(ssid, pass) != ESP_OK) { r_error(); return; }

    wifi_config_t wc = {0};
    size_t n = strnlen(ssid, sizeof wc.sta.ssid);
    memcpy(wc.sta.ssid, ssid, n);
    n = strnlen(pass, sizeof wc.sta.password);
    memcpy(wc.sta.password, pass, n);
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    cdc_print("\r\nconnecting...\r\n");
    esp_wifi_connect();
    r_ok();
}

static void cmd_wifi_query(void) {
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
            char ips[128];
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

static void cmd_dns(const char *host) {
    cdc_print("\r\n");
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, NULL, &hints, &res) != 0 || !res) {
        cdc_print("NXDOMAIN\r\n");
        r_error();
        return;
    }
    char ip[INET_ADDRSTRLEN];
    struct sockaddr_in *sin = (struct sockaddr_in *)res->ai_addr;
    inet_ntop(AF_INET, &sin->sin_addr, ip, sizeof ip);
    cdc_print(ip); cdc_print("\r\n");
    freeaddrinfo(res);
    r_ok();
}

static void cmd_ping(const char *host) {
    /* Lightweight TCP-handshake "ping" (port 80). Real ICMP needs
     * lwIP's raw socket setup which is heavier; the TCP variant is
     * what the old firmware shipped. */
    cdc_print("\r\n");
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, "80", &hints, &res) != 0 || !res) {
        cdc_print("NXDOMAIN\r\n");
        r_error();
        return;
    }
    int s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s < 0) { freeaddrinfo(res); r_error(); return; }
    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    int64_t t0 = esp_timer_get_time();
    int ok = connect(s, res->ai_addr, res->ai_addrlen);
    int64_t dt = esp_timer_get_time() - t0;
    close(s);
    freeaddrinfo(res);
    char line[64];
    if (ok == 0)
        snprintf(line, sizeof line, "reply %d ms\r\n", (int)(dt / 1000));
    else
        snprintf(line, sizeof line, "timeout (%d ms)\r\n", (int)(dt / 1000));
    cdc_print(line);
    r_ok();
}

static void cmd_scan(void) {
    cdc_print("\r\nscanning...\r\n");
    /* The wifi event handler reconnects-on-disconnect aggressively, so a
     * scan started while connect-retry is in flight returns ESP_ERR_WIFI_STATE.
     * Disconnect first and the next reconnect attempt will fire after we
     * finish; this lets scan succeed even when creds are wrong. */
    esp_wifi_disconnect();
    wifi_scan_config_t sc = {0};
    esp_err_t e = esp_wifi_scan_start(&sc, true);
    if (e != ESP_OK) {
        char b[64];
        snprintf(b, sizeof b, "scan failed (%s)\r\n", esp_err_to_name(e));
        cdc_print(b);
        r_error();
        return;
    }
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n > 24) n = 24;
    char line[128];
    snprintf(line, sizeof line, "%u networks:\r\n", (unsigned)n);
    cdc_print(line);
    if (n == 0) { r_ok(); return; }
    wifi_ap_record_t *ap = calloc(n, sizeof *ap);
    if (!ap) { cdc_print("calloc fail\r\n"); r_error(); return; }
    esp_wifi_scan_get_ap_records(&n, ap);
    for (uint16_t i = 0; i < n; ++i) {
        snprintf(line, sizeof line,
                 "  %-32.32s  ch=%-2u  rssi=%4d  auth=%u\r\n",
                 (const char *)ap[i].ssid,
                 (unsigned)ap[i].primary,
                 (int)ap[i].rssi,
                 (unsigned)ap[i].authmode);
        cdc_print(line);
    }
    free(ap);
    r_ok();
}

static void cmd_netif(void) {
    cdc_print("\r\n");
    esp_netif_t *netif = NULL;
    char buf[160];
    for (netif = esp_netif_next_unsafe(NULL); netif; netif = esp_netif_next_unsafe(netif)) {
        esp_netif_ip_info_t ip = {0};
        esp_netif_get_ip_info(netif, &ip);
        snprintf(buf, sizeof buf,
                 "%-12s  ip=" IPSTR "  gw=" IPSTR "  mask=" IPSTR "\r\n",
                 esp_netif_get_ifkey(netif),
                 IP2STR(&ip.ip), IP2STR(&ip.gw), IP2STR(&ip.netmask));
        cdc_print(buf);
    }
    r_ok();
}

/* ---- ATD: dial a host:port ---- */

static void cmd_dial(const char *arg) {
    /* Strip Hayes dial-type modifier (T/P/R). */
    while (*arg == ' ' || *arg == '\t') arg++;
    if (*arg == 'T' || *arg == 'P' || *arg == 'R') arg++;
    while (*arg == ' ' || *arg == '\t') arg++;
    if (!*arg) { r_error(); return; }

    char host[80];
    uint16_t port = 23;     /* Hayes default = telnet */
    const char *colon = strrchr(arg, ':');
    if (colon) {
        size_t n = (size_t)(colon - arg);
        if (n >= sizeof host) n = sizeof host - 1;
        memcpy(host, arg, n); host[n] = 0;
        int p = atoi(colon + 1);
        if (p > 0 && p < 65536) port = (uint16_t)p;
    } else {
        strncpy(host, arg, sizeof host - 1); host[sizeof host - 1] = 0;
    }

    /* Need WiFi up to resolve and dial. */
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) { r_nocarrier(); return; }

    char portstr[8];
    snprintf(portstr, sizeof portstr, "%u", (unsigned)port);
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res) {
        disk_logf("modem: dial DNS-fail '%s'", host);
        r_nocarrier();
        return;
    }

    int sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) { freeaddrinfo(res); r_nocarrier(); return; }
    struct timeval tv = { .tv_sec = 20, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);

    if (connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
        close(sock);
        freeaddrinfo(res);
        r_nocarrier();
        return;
    }
    freeaddrinfo(res);

    int yes = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof yes);
    /* Bigger socket RX buffer means lwIP can hold more in-flight TCP
     * data while the CDC pipe drains. lwIP clamps to its own ceilings,
     * but asking for 32 KB gets us as much as it'll give. */
    int rxbuf = 32 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rxbuf, sizeof rxbuf);

    s_sock = sock;
    snprintf(s_peer, sizeof s_peer, "%s:%u", host, (unsigned)port);
    s_plus_count = 0;
    s_last_data_us = esp_timer_get_time();
    s_online = true;
    tn_start();
    r_connect();

    /* Spawn the TCP -> CDC pump task. CPU1 with priority below
     * TinyUSB (so CDC RX callbacks preempt this when CDC has data). */
    xTaskCreatePinnedToCore(modem_data_task, "modem_data",
                            4096, NULL, 16, &s_data_task, 1);
}

/* ---- exec one AT line (command mode only) ---- */

static char *strip_at(char *line) {
    while (*line == ' ') line++;
    if (line[0] != 'A' && line[0] != 'a') return NULL;
    if (line[1] != 'T' && line[1] != 't') return NULL;
    char *rest = line + 2;
    /* Uppercase the command part only -- stop at '=' so case-sensitive
     * values (passwords, URLs, SSIDs) survive intact. Without this,
     * AT$PASS=Crusty jugglers landed in NVS as CRUSTY JUGGLERS and
     * WPA2 auth silently failed. */
    for (char *p = rest; *p && *p != '='; ++p)
        *p = (char)toupper((unsigned char)*p);
    return rest;
}

static void handle_dollar(char *s) {
    /* AT$<KEY>[=<value>|?] */
    char *eq  = strchr(s, '=');
    char *qm  = strchr(s, '?');
    char *key = s;
    char *val = NULL;
    if (eq) { *eq = 0; val = eq + 1; }
    else if (qm) { *qm = 0; val = qm; }   /* '?' query keeps key clean */

    if (!strcmp(key, "WIFI")) {
        if (val && eq)        cmd_wifi_set(val);
        else if (val && qm)   { cmd_wifi_query(); r_ok(); }
        else {
            /* Bare AT$WIFI -- reconnect with stored creds. */
            cdc_print("\r\nreconnecting...\r\n");
            esp_wifi_disconnect();
            esp_wifi_connect();
            r_ok();
        }
    } else if (!strcmp(key, "SSID")) {
        nvs_handle_t h;
        if (val && eq) {
            if (nvs_open("slip-router", NVS_READWRITE, &h) != ESP_OK) { r_error(); return; }
            nvs_set_str(h, "ssid", val);
            nvs_commit(h); nvs_close(h);
            /* Old build debounced SSID+PASS pair, but with separate
             * commands we just reconnect on next AT$WIFI (or AT$WIFI=). */
            r_ok();
        } else {
            char ssid[33] = {0};
            size_t n = sizeof ssid;
            if (nvs_open("slip-router", NVS_READONLY, &h) == ESP_OK) {
                nvs_get_str(h, "ssid", ssid, &n);
                nvs_close(h);
            }
            cdc_print("\r\n"); cdc_print(ssid); cdc_print("\r\n");
            r_ok();
        }
    } else if (!strcmp(key, "PASS")) {
        nvs_handle_t h;
        if (val && eq) {
            if (nvs_open("slip-router", NVS_READWRITE, &h) != ESP_OK) { r_error(); return; }
            nvs_set_str(h, "pass", val);
            nvs_commit(h); nvs_close(h);
            r_ok();
        } else {
            char pass[65] = {0};
            size_t n = sizeof pass;
            bool has = false;
            if (nvs_open("slip-router", NVS_READONLY, &h) == ESP_OK) {
                if (nvs_get_str(h, "pass", pass, &n) == ESP_OK && pass[0]) has = true;
                nvs_close(h);
            }
            cdc_print(has ? "\r\n(set)\r\n" : "\r\n(none)\r\n");
            r_ok();
        }
    } else if (!strcmp(key, "RSSI")) {
        wifi_ap_record_t ap = {0};
        char b[32];
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
            snprintf(b, sizeof b, "\r\n%d\r\n", (int)ap.rssi);
        else
            snprintf(b, sizeof b, "\r\nnoconn\r\n");
        cdc_print(b); r_ok();
    } else if (!strcmp(key, "STATS")) {
        if (val && eq) {
            /* AT$STATS=0 -- zero the SLIP counters. The harness
             * brackets each test with a clear-and-run-and-read so
             * before/after deltas isolate the test's own traffic. */
            if (!strcmp(val, "0")) {
                /* slip.c uses volatile uint32_ts; race-free single-
                 * store overwrite is fine. */
                slip_stats_clear();
                r_ok();
            } else {
                r_error();
            }
        } else {
            char line[160];
            snprintf(line, sizeof line,
                "\r\n"
                "slip.pkts_to_host    %u\r\n"
                "slip.pkts_from_host  %u\r\n"
                "slip.bytes_to_host   %u\r\n"
                "slip.bytes_from_host %u\r\n",
                (unsigned)slip_stat_pkts_to_host(),
                (unsigned)slip_stat_pkts_from_host(),
                (unsigned)slip_stat_bytes_to_host(),
                (unsigned)slip_stat_bytes_from_host());
            cdc_print(line);
            r_ok();
        }
    } else if (!strcmp(key, "MODE")) {
        if (val && eq) {
            LinkMode want = (LinkMode)-1;
            if      (!strcmp(val, "SLIP"))  want = MODE_SLIP;
            else if (!strcmp(val, "MODEM")) want = MODE_MODEM;
            if (want == (LinkMode)-1) { r_error(); return; }
            /* Reply BEFORE flipping mode, so the OK lands while the
             * AT parser still owns the CDC stream. Once SLIP is up
             * any further bytes from the host are interpreted as
             * SLIP frames. */
            r_ok();
            slip_set_mode(want);
        } else {
            cdc_print(slip_get_mode() == MODE_SLIP ? "\r\nSLIP\r\n" : "\r\nMODEM\r\n");
            r_ok();
        }
    } else if (!strcmp(key, "DNS") && val && eq) {
        cmd_dns(val);
    } else if (!strcmp(key, "PING") && val && eq) {
        cmd_ping(val);
    } else if (!strcmp(key, "SCAN")) {
        cmd_scan();
    } else if (!strcmp(key, "NETIF")) {
        cmd_netif();
    } else if (!strcmp(key, "RESET")) {
        cdc_print("\r\nRESETTING\r\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (!strcmp(key, "BOOT")) {
        /* AT$BOOT -- jump to ESP32-S3 ROM bootloader. Same fallback
         * the old build had: if AT$OTASTART can't run (firmware too
         * broken, locked, pre-OTA), we set the "force download mode"
         * RTC option and reset. Bootloader then enumerates as
         * cu.usbmodem123401 -- esptool can reflash via the same cable. */
        cdc_print("\r\nENTERING BOOTLOADER\r\n");
        vTaskDelay(pdMS_TO_TICKS(150));
        REG_WRITE(RTC_CNTL_OPTION1_REG, 0x1);   /* RTC_CNTL_FORCE_DOWNLOAD_BOOT */
        esp_restart();
    } else if (!strcmp(key, "OTASTART") && val && eq) {
        /* AT$OTASTART=<size> -- CDC-side OTA. Streams <size> bytes
         * of raw firmware.bin into the inactive OTA slot, then
         * commits + reboots. Recovery path when WiFi is unreachable.
         *
         *   host    -> AT$OTASTART=<size>\r
         *   dongle  -> \r\nOTA READY\r\n
         *   host    -> <size> bytes of fw.bin
         *   dongle  -> \r\nOTA OK\r\n  (then esp_restart)
         *           or \r\nOTA <code>\r\n + ERROR */
        long sz = strtol(val, NULL, 10);
        if (sz < 16384L || sz > 6L * 1024L * 1024L) {
            cdc_print("\r\nOTA BADSIZE\r\n"); r_error(); return;
        }
        const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
        if (!next) { cdc_print("\r\nOTA NOPART\r\n"); r_error(); return; }
        esp_ota_handle_t h = 0;
        if (esp_ota_begin(next, (size_t)sz, &h) != ESP_OK) {
            cdc_print("\r\nOTA BEGIN-FAIL\r\n"); r_error(); return;
        }
        cdc_print("\r\nOTA READY\r\n");
        long got = 0;
        int64_t last_rx_us = esp_timer_get_time();
        static uint8_t buf[2048];
        while (got < sz) {
            size_t want = (size_t)(sz - got);
            if (want > sizeof buf) want = sizeof buf;
            size_t n = 0;
            if (tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf, want, &n) == ESP_OK && n > 0) {
                if (esp_ota_write(h, buf, n) != ESP_OK) {
                    esp_ota_abort(h);
                    cdc_print("\r\nOTA WRITE-FAIL\r\n"); r_error(); return;
                }
                got += (long)n;
                last_rx_us = esp_timer_get_time();
            } else {
                if (esp_timer_get_time() - last_rx_us > 8 * 1000 * 1000LL) {
                    esp_ota_abort(h);
                    cdc_print("\r\nOTA TIMEOUT\r\n"); r_error(); return;
                }
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
        if (esp_ota_end(h) != ESP_OK) {
            cdc_print("\r\nOTA END-FAIL\r\n"); r_error(); return;
        }
        if (esp_ota_set_boot_partition(next) != ESP_OK) {
            cdc_print("\r\nOTA SETBOOT-FAIL\r\n"); r_error(); return;
        }
        cdc_print("\r\nOTA OK\r\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (!strcmp(key, "HELP")) {
        cdc_print(
            "\r\n"
            "ATE0/1  echo off/on        ATV0/1  numeric/verbose\r\n"
            "ATQ0/1  result codes       ATI     identity\r\n"
            "ATZ     reset settings     ATD<host>[:port]  dial out\r\n"
            "ATO     return online      ATH     hang up\r\n"
            "+++     escape to cmd      (1 s guard, 3 +'s, 1 s guard)\r\n"
            "AT$WIFI=ssid,pw    set wifi creds + reconnect\r\n"
            "AT$WIFI?           show wifi status\r\n"
            "AT$DNS=host        DNS lookup\r\n"
            "AT$PING=host       TCP-handshake ping\r\n"
            "AT$SCAN            list visible networks\r\n"
            "AT$NETIF           dump netif state\r\n"
            "AT$RESET           reboot the dongle\r\n"
            "AT$HELP            this help\r\n"
        );
        r_ok();
    } else {
        r_error();
    }
}

static void exec(char *line) {
    char *p = strip_at(line);
    if (!p) return;                 /* non-AT line: silent */
    if (!*p) { r_ok(); return; }    /* bare AT */

    switch (*p) {
        case 'E': s_echo    = (p[1] != '0'); r_ok(); return;
        case 'V': s_verbose = (p[1] != '0'); r_ok(); return;
        case 'Q': s_quiet   = (p[1] == '1'); r_ok(); return;
        case 'I': cdc_print("\r\nDOSongle Modem (Phase 1c)\r\n"); r_ok(); return;
        case 'N': {
            /* AT N<n> / AT NET<n> -- telnet IAC + CR-to-CRLF processing.
             * Old build accepts both "ATN0" and "ATNET0"; HTTPGET.EXE
             * sends the longer form. Skip over any non-digit letters
             * after N until we find the digit. */
            const char *q = p + 1;
            while (*q && (*q < '0' || *q > '9')) ++q;
            s_telnet = (*q != '0');
            r_ok();
        } return;
        case 'Z': s_echo = true; s_verbose = true; s_quiet = false; s_telnet = true; r_ok(); return;
        case 'D': cmd_dial(p + 1); return;
        case 'H': {
            if (s_sock >= 0) { close(s_sock); s_sock = -1; }
            s_online = false;
            s_peer[0] = 0;
            r_ok();
        } return;
        case 'O': {
            if (s_sock >= 0) {
                s_online = true;
                if (!s_data_task) {
                    xTaskCreatePinnedToCore(modem_data_task, "modem_data",
                                            4096, NULL, 16, &s_data_task, 1);
                }
                r_connect();
            } else {
                r_error();
            }
        } return;
        case '$': handle_dollar(p + 1); return;
        case '&': {
            /* AT&V dump config / AT&W save E/V/N / AT&F factory reset. */
            char c = (char)toupper((unsigned char)p[1]);
            if (c == 'V') {
                char b[160];
                snprintf(b, sizeof b,
                         "\r\nATE%d  ATV%d  ATQ%d  ATN%d\r\n"
                         "mode:  %s\r\n",
                         s_echo?1:0, s_verbose?1:0, s_quiet?1:0, s_telnet?1:0,
                         slip_get_mode() == MODE_SLIP ? "SLIP" : "MODEM");
                cdc_print(b);
                r_ok();
            } else if (c == 'W') {
                modem_save_evn();
                r_ok();
            } else if (c == 'F') {
                s_echo = true; s_verbose = true; s_quiet = false; s_telnet = true;
                modem_save_evn();
                r_ok();
            } else {
                r_error();
            }
        } return;
        default:
            /* Real modems are lenient: unknown single-letter commands
             * (AT&K0, ATS0=0, etc. that scripts commonly issue) return
             * OK so the script doesn't abort. We do the same. */
            r_ok();
            return;
    }
}

/* ---- CDC RX callback (TinyUSB task context) ---- */

static void on_cdc_rx(int itf, cdcacm_event_t *event) {
    (void)event;
    /* Drain in MTU-sized chunks. SLIP MTU is 1500; CDC RX FIFO is
     * sized to hold a full SLIP frame, so this loop typically does
     * one big read per callback. */
    static uint8_t buf[2048];
    size_t got = 0;
    if (tinyusb_cdcacm_read(itf, buf, sizeof buf, &got) != ESP_OK) return;
    if (got == 0) return;

    /* SLIP mode: bytes are SLIP-framed IP packets. Feed them to the
     * de-framer; nothing else touches them. */
    if (slip_get_mode() == MODE_SLIP) {
        slip_feed(buf, got);
        return;
    }

    if (s_online) {
        /* Online: pipe CDC bytes to TCP with +++ escape + telnet
         * IAC/CRLF handling. */
        online_push_bytes(buf, got);
        return;
    }

    /* Command mode: line-buffered AT parser. */
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
        if (b == 0x08 || b == 0x7F) {
            if (s_cmd_len > 0) {
                s_cmd_len--;
                if (s_echo) cdc_print("\b \b");
            }
            continue;
        }
        if (b < 0x20 || b > 0x7E) continue;
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
    modem_load_evn();      /* restore E/V/N from NVS (AT&W saved) */
    esp_err_t e = tinyusb_cdcacm_register_callback(
        TINYUSB_CDC_ACM_0, CDC_EVENT_RX, on_cdc_rx);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "cdc rx cb register: %s", esp_err_to_name(e));
        return e;
    }
    disk_logf("modem: AT engine ready on CDC0 (E%d V%d N%d)",
              s_echo?1:0, s_verbose?1:0, s_telnet?1:0);
    return ESP_OK;
}
