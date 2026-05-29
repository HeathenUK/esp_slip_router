#include "modem.h"
#include "wificfg.h"
#include <WiFi.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

extern "C" {
#include "esp32-hal-tinyusb.h"   /* tud_cdc_n_write / tud_cdc_n_write_flush */
#include "esp_ota_ops.h"          /* OTA partition write API (USB-CDC OTA) */
}

// Debug to the hardware UART only — USB CDC is the data link.
#define DBG(...) do { Serial0.printf(__VA_ARGS__); } while (0)

/* USBCDC's Serial.write() returns 0 when the framework's `connected` flag is
 * false — i.e. when the host hasn't sent SET_CONTROL_LINE_STATE with DTR=1.
 * pyserial does that on open; DOSBox's directserial does not. So every modem
 * echo / OK / banner is silently dropped under DOSBox. Talk to TinyUSB CDC
 * itf 0 directly — that path doesn't gate on the connected flag, the host
 * still gets the bytes via the bulk-IN endpoint as soon as it reads.
 *
 * Loops on partial writes (TinyUSB's TX buffer is ~256B; AT$HELP and a few
 * other responses exceed that), with a brief yield between attempts so
 * TinyUSB has time to push a packet. 500ms hard deadline so a dead host
 * can't hang the modem loop indefinitely. Returns bytes actually accepted. */
static size_t cdc_write(const void *buf, size_t n)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t total = 0;
    uint32_t start = millis();
    while (total < n) {
        size_t w = tud_cdc_n_write(0, p + total, n - total);
        total += w;
        if (total < n) {
            tud_cdc_n_write_flush(0);
            if (millis() - start > 500U) break;
            delay(1);
        }
    }
    tud_cdc_n_write_flush(0);
    return total;
}
static inline size_t cdc_print(const char *s) { return cdc_write(s, strlen(s)); }
static inline void   cdc_byte(uint8_t b)      { (void)cdc_write(&b, 1); }

static WiFiClient client;
static bool online   = false;   // in data (online) mode with a live socket
static bool echo     = true;    // ATE — local echo of AT command lines
static bool verbose  = true;    // ATV — text vs numeric result codes
static bool telnet   = true;    // ATNETn — telnet protocol on the connection

static char    cmd[128];
static uint8_t cmdlen = 0;
static bool    cmd_too_long = false;   /* set when typing past cmd[]; exec() skipped + ERROR */
static char    peer[80] = "";

// +++ escape (Hayes guard timing): 3 '+' bracketed by ~1s of silence.
static uint8_t  plus_count = 0;
static uint32_t plus_time  = 0;
static uint32_t last_tx_ms = 0;
static const uint32_t GUARD_MS = 1000;

// ---- Telnet (RFC854/855 + options) --------------------------------------
#define TN_SE   240
#define TN_SB   250
#define TN_WILL 251
#define TN_WONT 252
#define TN_DO   253
#define TN_DONT 254
#define TN_IAC  255
#define OPT_BINARY 0    // RFC856
#define OPT_ECHO   1    // RFC857
#define OPT_SGA    3    // RFC858 suppress go-ahead
#define OPT_TTYPE  24   // RFC1091 terminal type
#define OPT_NAWS   31   // RFC1073 window size
#define TT_IS   0
#define TT_SEND 1

static const char *TERM_TYPE = "ansi";
static const uint8_t TERM_COLS = 80, TERM_ROWS = 24;

enum { T_DATA, T_IAC, T_OPT, T_SB_OPT, T_SB_DATA, T_SB_IAC };
static uint8_t tstate = T_DATA;
static uint8_t tcmd = 0, sbopt = 0;
static uint8_t sbbuf[40];
static uint8_t sblen = 0;

// agreed/offered state, 1 bit per option (loop-free negotiation)
static uint8_t remote_on[32], local_on[32], local_offered[32];
static inline bool bget(uint8_t *a, uint8_t o) { return a[o >> 3] & (1 << (o & 7)); }
static inline void bset(uint8_t *a, uint8_t o) { a[o >> 3] |=  (1 << (o & 7)); }
static inline void bclr(uint8_t *a, uint8_t o) { a[o >> 3] &= ~(1 << (o & 7)); }

static void send3(uint8_t cmdb, uint8_t opt) { uint8_t b[3] = {TN_IAC, cmdb, opt}; client.write(b, 3); }
static bool accept_remote(uint8_t o) { return o == OPT_BINARY || o == OPT_SGA || o == OPT_ECHO; }
static bool offer_local(uint8_t o)   { return o == OPT_BINARY || o == OPT_SGA || o == OPT_TTYPE || o == OPT_NAWS; }

static void send_naws() {
    uint8_t b[] = {TN_IAC, TN_SB, OPT_NAWS, 0, TERM_COLS, 0, TERM_ROWS, TN_IAC, TN_SE};
    client.write(b, sizeof(b));
}
static void after_local_enable(uint8_t opt) { if (opt == OPT_NAWS) send_naws(); }

static void offer_will(uint8_t opt) {
    if (!bget(local_offered, opt)) { bset(local_offered, opt); send3(TN_WILL, opt); }
}

static void telnet_start() {
    memset(remote_on, 0, sizeof(remote_on));
    memset(local_on, 0, sizeof(local_on));
    memset(local_offered, 0, sizeof(local_offered));
    sblen = 0; tstate = T_DATA;
    if (!telnet) return;
    // Client opener: offer the options that make a BBS session clean & correct.
    offer_will(OPT_TTYPE);
    offer_will(OPT_NAWS);
    offer_will(OPT_SGA);    send3(TN_DO, OPT_SGA);
    offer_will(OPT_BINARY); send3(TN_DO, OPT_BINARY);
}

static void handle_neg(uint8_t c, uint8_t opt) {
    switch (c) {
        case TN_WILL:
            if (accept_remote(opt)) {
                if (!bget(remote_on, opt)) { bset(remote_on, opt); send3(TN_DO, opt); }
            } else {
                bclr(remote_on, opt); send3(TN_DONT, opt);
            }
            break;
        case TN_WONT:
            if (bget(remote_on, opt)) { bclr(remote_on, opt); send3(TN_DONT, opt); }
            break;
        case TN_DO:
            if (offer_local(opt)) {
                if (!bget(local_on, opt)) {
                    bset(local_on, opt);
                    if (!bget(local_offered, opt)) { bset(local_offered, opt); send3(TN_WILL, opt); }
                    after_local_enable(opt);
                }
            } else {
                send3(TN_WONT, opt);
            }
            break;
        case TN_DONT:
            if (bget(local_on, opt)) { bclr(local_on, opt); send3(TN_WONT, opt); }
            break;
    }
}

static void handle_sb() {
    if (sbopt == OPT_TTYPE && sblen >= 1 && sbbuf[0] == TT_SEND) {
        uint8_t hdr[] = {TN_IAC, TN_SB, OPT_TTYPE, TT_IS};
        client.write(hdr, sizeof(hdr));
        client.write((const uint8_t *)TERM_TYPE, strlen(TERM_TYPE));
        uint8_t tail[] = {TN_IAC, TN_SE};
        client.write(tail, sizeof(tail));
    }
}

// ---- result codes ---------------------------------------------------------
static void r_ok()        { cdc_print(verbose ? "\r\nOK\r\n"         : "0\r\n"); }
static void r_error()     { cdc_print(verbose ? "\r\nERROR\r\n"      : "4\r\n"); }
static void r_connect()   { cdc_print(verbose ? "\r\nCONNECT\r\n"    : "1\r\n"); }
static void r_nocarrier() { cdc_print(verbose ? "\r\nNO CARRIER\r\n" : "3\r\n"); }

void modem_begin() {}

void modem_enter() {
    cmdlen = 0; plus_count = 0; tstate = T_DATA;
    online = client.connected();
    cdc_print("\r\nWiFi Modem ready\r\n");
    if (!online) r_ok();
}

void modem_leave() {
    if (client.connected()) client.stop();
    online = false; cmdlen = 0; plus_count = 0;
}

bool modem_is_online()   { return online && client.connected(); }
const char *modem_peer() { return peer; }

static void dial(const char *a) {
    while (*a == ' ' || *a == '\t') a++;
    // Optional single dial-type modifier (T/P/R) — only strip it when it's
    // clearly standalone (followed by space/digit/end), so a hostname that
    // starts with t/p/r (e.g. "telehack.com") isn't truncated.
    if (*a == 'T' || *a == 't' || *a == 'P' || *a == 'p' || *a == 'R' || *a == 'r') {
        char n = a[1];
        if (n == ' ' || n == '\t' || n == 0 || isdigit((unsigned char)n)) a++;
    }
    while (*a == ' ' || *a == '\t') a++;
    if (*a == 0) { r_error(); return; }

    char host[80];
    uint16_t port = 23;
    const char *colon = strrchr(a, ':');
    if (colon) {
        size_t n = (size_t)(colon - a);
        if (n >= sizeof(host)) n = sizeof(host) - 1;
        memcpy(host, a, n); host[n] = 0;
        int p = atoi(colon + 1);
        if (p > 0 && p < 65536) port = (uint16_t)p;
    } else {
        strncpy(host, a, sizeof(host) - 1); host[sizeof(host) - 1] = 0;
    }

    if (WiFi.status() != WL_CONNECTED) { r_nocarrier(); return; }
    DBG("[modem] dial %s:%u\n", host, port);
    if (client.connect(host, port)) {
        client.setNoDelay(true);
        snprintf(peer, sizeof(peer), "%s:%u", host, port);
        online = true; plus_count = 0; last_tx_ms = millis();
        telnet_start();
        r_connect();
    } else {
        peer[0] = 0;
        r_nocarrier();
    }
}

static void print_wifi_status() {
    cdc_print("\r\nSSID: "); cdc_print(wifi_ssid());
    if (WiFi.status() == WL_CONNECTED) {
        cdc_print("\r\nIP: "); cdc_print(WiFi.localIP().toString().c_str());
        cdc_print("\r\nstatus: connected\r\n");
    } else {
        cdc_print("\r\nstatus: not connected\r\n");
    }
}

// Extended config commands: AT$SSID=/?  AT$PASS=/?  AT$WIFI[?]  AT$HELP
// `s` is the text after the '$'. The key is matched case-insensitively; the
// value (after '=') keeps its original case.
static void handle_dollar(char *s) {
    char key[16];
    int k = 0;
    char *p = s;
    while (*p && *p != '=' && *p != '?' && k < (int)sizeof(key) - 1)
        key[k++] = (char)toupper((unsigned char)*p++);
    key[k] = 0;
    char op = *p;                                  // '=', '?', or 0
    char *val = (op == '=') ? p + 1 : (char *)"";

    if (!strcmp(key, "SSID")) {
        // setting a credential auto-connects (debounced, so SSID+PASS coalesce)
        if (op == '=') { wifi_set_ssid(val); wifi_apply_soon(); r_ok(); }
        else { cdc_print("\r\n"); cdc_print(wifi_ssid()); cdc_print("\r\n"); r_ok(); }
    } else if (!strcmp(key, "PASS")) {
        if (op == '=') { wifi_set_pass(val); wifi_apply_soon(); r_ok(); }
        else { cdc_print(wifi_has_pass() ? "\r\n(set)\r\n" : "\r\n(none)\r\n"); r_ok(); }
    } else if (!strcmp(key, "WIFI")) {
        if (op == '?') { print_wifi_status(); r_ok(); }
        else if (op == '=') {
            // AT$WIFI=ssid,password  — set both atomically and connect now.
            // No comma => open network (empty password).
            char *comma = strchr(val, ',');
            if (comma) {
                *comma = 0;
                wifi_set_ssid(val);
                wifi_set_pass(comma + 1);
            } else {
                wifi_set_ssid(val);
                wifi_set_pass("");
            }
            cdc_print("\r\nconnecting...\r\n"); wifi_reconnect(); r_ok();
        } else {
            cdc_print("\r\nconnecting...\r\n"); wifi_reconnect(); r_ok();
        }
    } else if (!strcmp(key, "OTASTART")) {
        // AT$OTASTART=<size>  — USB-CDC OTA: stream <size> bytes of app image
        // straight into the inactive OTA partition (app0/app1), then commit
        // and reboot into the new firmware. No bootloader trip, no esptool,
        // no WiFi, no buttons; the existing CDC link IS the upload channel.
        // Protocol:
        //   host:    AT$OTASTART=<bytes>\r
        //   dongle:  \r\nOTA READY\r\n
        //   host:    <bytes> of raw firmware.bin
        //   dongle:  \r\nOTA OK\r\n     (then esp_restart — new fw boots)
        //           or  \r\nOTA <code>\r\n  followed by ERROR
        if (op != '=') { r_error(); return; }
        long sz = atol(val);
        if (sz < 16384L || sz > 6L * 1024L * 1024L) {  // 16K..6M sanity
            cdc_print("\r\nOTA BADSIZE\r\n"); r_error(); return;
        }
        const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
        if (!next) { cdc_print("\r\nOTA NOPART\r\n"); r_error(); return; }
        esp_ota_handle_t h = 0;
        if (esp_ota_begin(next, (size_t)sz, &h) != ESP_OK) {
            cdc_print("\r\nOTA BEGIN-FAIL\r\n"); r_error(); return;
        }
        cdc_print("\r\nOTA READY\r\n");
        unsigned long got = 0UL;
        unsigned long last_rx = millis();
        uint8_t buf[512];
        while (got < (unsigned long)sz) {
            int avail = Serial.available();
            if (avail > 0) {
                int want = (avail > (int)sizeof(buf)) ? (int)sizeof(buf) : avail;
                if ((unsigned long)want > (unsigned long)sz - got)
                    want = (int)((unsigned long)sz - got);
                int n = Serial.read(buf, want);
                if (n > 0) {
                    if (esp_ota_write(h, buf, (size_t)n) != ESP_OK) {
                        esp_ota_abort(h);
                        cdc_print("\r\nOTA WRITE-FAIL\r\n"); r_error(); return;
                    }
                    got += (unsigned long)n;
                    last_rx = millis();
                }
            } else if (millis() - last_rx > 8000UL) {
                esp_ota_abort(h);
                cdc_print("\r\nOTA TIMEOUT\r\n"); r_error(); return;
            }
            delay(0);   // feed wdt, yield
        }
        if (esp_ota_end(h) != ESP_OK) {
            cdc_print("\r\nOTA END-FAIL (bad image?)\r\n"); r_error(); return;
        }
        if (esp_ota_set_boot_partition(next) != ESP_OK) {
            cdc_print("\r\nOTA SETBOOT-FAIL\r\n"); r_error(); return;
        }
        cdc_print("\r\nOTA OK\r\n");
        delay(200);                                  // drain CDC TX
        esp_restart();
        /* unreachable */
    } else if (!strcmp(key, "BOOT")) {
        // AT$BOOT — escape hatch. Reboots into the ESP32-S3 ROM bootloader so
        // esptool can reflash via the same USB cable WITHOUT the BOOT-button
        // download-mode dance. Only needed if AT$OTASTART can't run (e.g.
        // current firmware is broken / pre-OTA / locked). Bootloader then
        // enumerates as cu.usbmodem123401; esptool's --after hard_reset flips
        // back to user firmware on completion.
        cdc_print("\r\nENTERING BOOTLOADER\r\n");
        delay(150);                          // let the bytes drain to host
        usb_persist_restart(RESTART_BOOTLOADER);
        /* unreachable — esp_restart() above */
    } else if (!strcmp(key, "HELP")) {
        cdc_print("\r\nAT$WIFI=<ssid>,<pw>  set both + connect (auto)\r\n"
                     "AT$SSID=<ssid>       set SSID  (auto-connects)\r\n"
                     "AT$PASS=<pw>         set pass  (auto-connects)\r\n"
                     "AT$WIFI              reconnect with stored creds\r\n"
                     "AT$WIFI?             status   AT$SSID?  AT$PASS?\r\n"
                     "AT$OTASTART=<size>   USB-CDC OTA: stream <size> B fw.bin\r\n"
                     "AT$BOOT              fallback: reboot into ROM bootloader\r\n");
        r_ok();
    } else {
        r_error();
    }
}

static void exec(char *line) {
    if (line[0] == 0) { r_ok(); return; }     // bare "AT"
    char c = (char)toupper((unsigned char)line[0]);
    char *rest = line + 1;
    switch (c) {
        case '$': handle_dollar(rest); return;
        case 'D': dial(rest); return;
        case 'H': modem_leave(); r_ok(); return;
        case 'O': if (client.connected()) { online = true; r_connect(); }
                  else r_nocarrier();
                  return;
        case 'E': echo    = !(rest[0] == '0'); r_ok(); return;
        case 'V': verbose = !(rest[0] == '0'); r_ok(); return;
        case 'Z': modem_leave(); echo = true; verbose = true; telnet = true;
                  r_ok(); return;
        case 'I': cdc_print("\r\nFOSSLIP WiFi Modem\r\n"); r_ok(); return;
        case 'N': {  // ATNETn — telnet protocol on/off
            const char *p = rest;
            while (*p && !isdigit((unsigned char)*p)) p++;
            telnet = !(*p == '0');
            r_ok(); return;
        }
        default:  r_ok(); return;             // lenient, like real modems
    }
}

static void feed_cmd(uint8_t ch) {
    if (ch == '\r') {
        if (echo) cdc_byte('\r');
        if (cmd_too_long) {
            r_error();              // line was discarded; report once on CR
            cmdlen = 0; cmd_too_long = false;
            return;
        }
        cmd[cmdlen] = 0;
        if ((cmd[0] == 'A' || cmd[0] == 'a') && (cmdlen >= 2) &&
            (cmd[1] == 'T' || cmd[1] == 't'))
            exec(cmd + 2);
        else if (cmdlen != 0)
            r_error();
        cmdlen = 0;
    } else if (ch == '\n') {
        /* ignore */
    } else if (ch == 8 || ch == 127) {        // BS or DEL
        if (cmd_too_long) return;             // can't recover; wait for CR
        if (cmdlen > 0) {
            cmdlen--;
            if (echo) cdc_print("\b \b");  // erase on screen: back, space, back
        }
    } else if (cmdlen < sizeof(cmd) - 1) {
        cmd[cmdlen++] = ch;
        if (echo) cdc_byte(ch);           // echo only what we actually buffer
    } else {
        // Buffer full — mark line as discarded so we error on CR rather than
        // silently exec-ing the truncated prefix. No echo (would lie about what
        // we accepted).
        cmd_too_long = true;
    }
}

static void flush_plus() {
    while (plus_count > 0) { client.write('+'); plus_count--; }
}

// online: host(USB) -> TCP, with +++ escape and telnet IAC-doubling.
static void pump_usb_to_tcp() {
    uint32_t now = millis();
    while (Serial.available() && client.connected()) {
        int ci = Serial.read(); if (ci < 0) break;
        uint8_t ch = (uint8_t)ci;
        if (ch == '+' && plus_count < 3 &&
            (plus_count > 0 || (now - last_tx_ms) > GUARD_MS)) {
            plus_count++; plus_time = now;
        } else {
            flush_plus();
            if (telnet && ch == 0x0DU && !bget(local_on, OPT_BINARY)) {
                /* NVT end-of-line: the DOS keyboard sends Enter as a bare CR,
                 * but a telnet server submits a line on CR LF. The dongle is
                 * the telnet client, so do the translation here. Skipped when
                 * telnet is off (ATNET0) or we're transmitting binary (raw). */
                client.write((uint8_t)0x0D);
                client.write((uint8_t)0x0A);
            } else {
                if (telnet && ch == TN_IAC) client.write((uint8_t)TN_IAC);  // double IAC
                client.write(ch);
            }
            last_tx_ms = now;
        }
    }
}

// online: TCP -> host(USB). Gated on USB write room (backpressure to the slow
// host); telnet negotiation handled inline. Negotiation replies go to the
// socket, only payload bytes go to USB.
static void pump_tcp_to_usb() {
    /* Bypass USBCDC::availableForWrite too — that gates on connected. Talk to
     * TinyUSB's TX FIFO directly so backpressure works even when the host
     * hasn't sent SET_CONTROL_LINE_STATE (DOSBox directserial case). */
    while (client.available() && tud_cdc_n_write_available(0) > 4) {
        int ci = client.read(); if (ci < 0) break;
        uint8_t ch = (uint8_t)ci;
        if (!telnet) { cdc_byte(ch); continue; }
        switch (tstate) {
            case T_DATA:
                if (ch == TN_IAC) tstate = T_IAC; else cdc_byte(ch);
                break;
            case T_IAC:
                if (ch == TN_IAC) { cdc_byte((uint8_t)TN_IAC); tstate = T_DATA; }
                else if (ch == TN_WILL || ch == TN_WONT || ch == TN_DO || ch == TN_DONT) {
                    tcmd = ch; tstate = T_OPT;
                } else if (ch == TN_SB) tstate = T_SB_OPT;
                else tstate = T_DATA;                  // standalone command
                break;
            case T_OPT:
                handle_neg(tcmd, ch); tstate = T_DATA;
                break;
            case T_SB_OPT:
                sbopt = ch; sblen = 0; tstate = T_SB_DATA;
                break;
            case T_SB_DATA:
                if (ch == TN_IAC) tstate = T_SB_IAC;
                else if (sblen < sizeof(sbbuf)) sbbuf[sblen++] = ch;
                break;
            case T_SB_IAC:
                if (ch == TN_SE) { handle_sb(); tstate = T_DATA; }
                else { if (sblen < sizeof(sbbuf)) sbbuf[sblen++] = ch; tstate = T_SB_DATA; }
                break;
        }
    }
}

void modem_poll() {
    if (!online) {
        // Cap per-poll so a sustained host flood can't starve button polling,
        // WiFi events, display refresh, etc. 512 chars/iter @ ~1kHz loop()
        // sustains > 500 kB/s of AT chatter — well above keyboard speed.
        int budget = 512;
        while (budget-- > 0 && Serial.available()) {
            int ci = Serial.read(); if (ci < 0) break;
            feed_cmd((uint8_t)ci);
        }
        return;
    }
    pump_usb_to_tcp();
    pump_tcp_to_usb();

    if (plus_count == 3 && (millis() - plus_time) > GUARD_MS) {
        plus_count = 0; online = false;
        r_ok();                       // command mode; socket stays open (ATO)
    }
    if (!client.connected()) {
        online = false;
        r_nocarrier();
    }
}
