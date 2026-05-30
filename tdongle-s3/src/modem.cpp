#include "modem.h"
#include "wificfg.h"
#include <WiFi.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

extern "C" {
#include "esp32-hal-tinyusb.h"   /* tud_cdc_n_write / tud_cdc_n_write_flush */
#include "esp_ota_ops.h"          /* OTA partition write API (USB-CDC OTA) */
#include "esp_system.h"           /* esp_restart() for AT$RESET */
#include "dongle_disk.h"          /* AT$DISK status */
#include "dongle_kbd.h"           /* AT$TYPE -- HID keyboard out */
#include "lwip/netif.h"           /* AT$NETIF dump */
#include "ping/ping_sock.h"       /* AT$PING -- WiFi outbound smoke test */
size_t cdc_read_raw(uint8_t *buf, size_t max);   /* defined in main.cpp */
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

void modem_begin() {
    // Restore AT&W'd settings from NVS (defaults to all-on if never saved).
    bool e, v, t;
    wifi_load_modem_settings(&e, &v, &t);
    echo = e; verbose = v; telnet = t;
}

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
    // Hayes dial-type modifier (T=tone, P=pulse, R=originate-only): the
    // standard says one optional letter right after the D. Strip
    // unconditionally -- a hostname that genuinely starts with T/P/R can
    // be dialled as "ATD<host>" (no modifier), but a hostname *after* a
    // modifier ("ATDTipv4.download...") was being mis-parsed as
    // "Tipv4.download..." which then NXDOMAIN'd into NO CARRIER.
    if (*a == 'T' || *a == 't' || *a == 'P' || *a == 'p' || *a == 'R' || *a == 'r')
        a++;
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
    // 3s default is too short -- DNS over a poor WiFi link via NAPT, plus
    // the TCP handshake, easily exceeds it. 20s matches a typical dial-up
    // modem's "dial then timeout" behaviour.
    client.setConnectionTimeout(20000);

    // Resolve manually then connect by IP -- cleaner error semantics than
    // letting NetworkClient::connect(host,...) eat both failure modes.
    IPAddress resolved;
    if (!resolved.fromString(host)) {
        if (WiFi.hostByName(host, resolved) != 1) {
            DBG("[modem] dial DNS-fail '%s'\n", host);
            peer[0] = 0;
            r_nocarrier();
            return;
        }
        DBG("[modem] dial %s -> %s\n", host, resolved.toString().c_str());
    }
    if (client.connect(resolved, port)) {
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
    char line[80];
    cdc_print("\r\nSSID: "); cdc_print(wifi_ssid());
    if (WiFi.status() == WL_CONNECTED) {
        cdc_print("\r\nIP: "); cdc_print(WiFi.localIP().toString().c_str());
        cdc_print("\r\nGW: "); cdc_print(WiFi.gatewayIP().toString().c_str());
        cdc_print("\r\nDNS: "); cdc_print(WiFi.dnsIP(0).toString().c_str());
        if ((uint32_t)WiFi.dnsIP(1) != 0) {
            cdc_print(", "); cdc_print(WiFi.dnsIP(1).toString().c_str());
        }
        snprintf(line, sizeof(line), "\r\nRSSI: %d dBm   ch: %d   BSSID: %s",
                 (int)WiFi.RSSI(), (int)WiFi.channel(),
                 WiFi.BSSIDstr().c_str());
        cdc_print(line);
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
        unsigned long last_yield = millis();
        uint8_t buf[2048];                       // bigger chunk -> fewer syscalls
        while (got < (unsigned long)sz) {
            int want = (int)sizeof(buf);
            if ((unsigned long)want > (unsigned long)sz - got)
                want = (int)((unsigned long)sz - got);
            int n = (int)cdc_read_raw(buf, (size_t)want);
            if (n > 0) {
                if (esp_ota_write(h, buf, (size_t)n) != ESP_OK) {
                    esp_ota_abort(h);
                    cdc_print("\r\nOTA WRITE-FAIL\r\n"); r_error(); return;
                }
                got += (unsigned long)n;
                last_rx = millis();
            } else {
                if (millis() - last_rx > 8000UL) {
                    esp_ota_abort(h);
                    cdc_print("\r\nOTA TIMEOUT\r\n"); r_error(); return;
                }
                delay(0);   // idle -> yield to TinyUSB / WDT
                last_yield = millis();
                continue;
            }
            // Busy path: only yield every 50ms to keep TinyUSB / WDT happy
            // without paying a full FreeRTOS context switch per iteration.
            if (millis() - last_yield > 50UL) { delay(0); last_yield = millis(); }
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
    } else if (!strcmp(key, "MODE")) {
        // AT$MODE?  -- report current personality
        // AT$MODE=SLIP / AT$MODE=MODEM  -- switch (persists to NVS)
        // Once switched to SLIP the modem AT engine stops being called; the
        // device starts framing SLIP on the same CDC link. Reverse the switch
        // by a long BOOT-button press (host-side AT is unreachable in SLIP).
        const char *result = modem_set_personality(op == '=' ? val : NULL);
        if (!result) { r_error(); return; }
        cdc_print("\r\n"); cdc_print(result); cdc_print("\r\n");
        r_ok();
    } else if (!strcmp(key, "DNS") && op == '=') {
        // AT$DNS=<host>  --  resolve via lwIP (same path WiFi.hostByName uses).
        // Returns the IP or "NXDOMAIN".
        IPAddress ip;
        cdc_print("\r\n");
        if (WiFi.hostByName(val, ip) == 1) {
            cdc_print(ip.toString().c_str()); cdc_print("\r\n"); r_ok();
        } else {
            cdc_print("NXDOMAIN\r\n"); r_error();
        }
    } else if (!strcmp(key, "PING") && op == '=') {
        // AT$PING=<ip> -- one ICMP echo from the dongle itself (not via SLIP).
        // Smoke test for the WiFi-out path. If this works but the SLIP NAPT
        // path doesn't, the bug is in forwarding, not in WiFi.
        ip_addr_t target;
        if (!ipaddr_aton(val, &target)) { r_error(); return; }
        esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
        cfg.count = 1; cfg.timeout_ms = 2000; cfg.target_addr = target;
        static volatile int s_ok, s_done;
        s_ok = 0; s_done = 0;
        esp_ping_callbacks_t cbs = {};
        cbs.on_ping_success = [](esp_ping_handle_t, void *)  { s_ok = 1; };
        cbs.on_ping_end     = [](esp_ping_handle_t, void *)  { s_done = 1; };
        esp_ping_handle_t h;
        if (esp_ping_new_session(&cfg, &cbs, &h) != ESP_OK) { r_error(); return; }
        esp_ping_start(h);
        uint32_t until = millis() + 3000;
        while (!s_done && (int32_t)(millis() - until) < 0) delay(20);
        esp_ping_delete_session(h);
        cdc_print(s_ok ? "\r\nreply\r\n" : "\r\ntimeout\r\n");
        s_ok ? r_ok() : r_error();
    } else if (!strcmp(key, "NETIF")) {
        // AT$NETIF -- dump every lwIP netif (admin/link state, addr, napt bit).
        // Diagnostic for the SLIP->WiFi forwarding path. Note: in SLIP mode
        // the AT engine isn't running, so this only reports the state seen
        // from MODEM mode (slip_nif will read DN). Toggle in/out via AT$MODE
        // to see SLIP-mode state would need a different path -- not built.
        cdc_print("\r\n");
        struct netif *n;
        char line[160];
        char a[20], nm[20], gw[20];
        NETIF_FOREACH(n) {
            ip4addr_ntoa_r(netif_ip4_addr(n),    a,  sizeof(a));
            ip4addr_ntoa_r(netif_ip4_netmask(n), nm, sizeof(nm));
            ip4addr_ntoa_r(netif_ip4_gw(n),      gw, sizeof(gw));
            int is_default = (n == netif_default) ? 1 : 0;
            snprintf(line, sizeof(line),
                     "%c%c%d %s%s%s napt=%d %s/%s gw=%s\r\n",
                     n->name[0], n->name[1], n->num,
                     netif_is_up(n) ? "UP" : "DN",
                     netif_is_link_up(n) ? "+L" : "-L",
                     is_default ? " DEF" : "",
                     n->napt, a, nm, gw);
            cdc_print(line);
        }
        r_ok();
    } else if (!strcmp(key, "RSSI")) {
        // AT$RSSI? -- just the dBm of the current association, scriptable.
        char b[32];
        if (WiFi.status() == WL_CONNECTED) snprintf(b, sizeof(b), "\r\n%d\r\n", (int)WiFi.RSSI());
        else                               snprintf(b, sizeof(b), "\r\nnoconn\r\n");
        cdc_print(b); r_ok();
    } else if (!strcmp(key, "STATS")) {
        // AT$STATS  -- print SLIP byte/pkt counters (also valid in modem mode --
        // they accumulate across both personalities). AT$STATS=0 clears.
        if (op == '=' && val[0] == '0') {
            modem_clear_slip_stats();
            r_ok();
        } else {
            struct slip_stats s;
            modem_get_slip_stats(&s);
            char b[160];
            snprintf(b, sizeof(b),
                     "\r\nhost->net: %u pkts / %u bytes"
                     "\r\nnet->host: %u pkts / %u bytes\r\n",
                     (unsigned)s.pkts_in,  (unsigned)s.bytes_in,
                     (unsigned)s.pkts_out, (unsigned)s.bytes_out);
            cdc_print(b);
            r_ok();
        }
    } else if (!strcmp(key, "TYPE")) {
        // AT$TYPE=<string> -- send <string> via the USB HID keyboard. See
        // dongle_kbd.h for the token DSL (<ENTER>, <F1>, <CTRL+C>, etc).
        if (op != '=') { r_error(); return; }
        int n = dongle_kbd_type(val, (int)strlen(val));
        if (n < 0) { cdc_print("\r\nTYPE PARSE-ERR\r\n"); r_error(); return; }
        r_ok();
    } else if (!strcmp(key, "DISK")) {
        // AT$DISK? / AT$DISK=HOST-WRITE|DEVICE-WRITE|FORMAT -- USB MSC / HTTP dev-disk.
        if (op == '=') {
            dongle_disk_owner_t owner;
            if (!strcasecmp(val, "FORMAT")) {
                if (!dongle_disk_format()) {
                    cdc_print("\r\nDISK FORMAT-ERR\r\n");
                    r_error();
                    return;
                }
                r_ok();
                return;
            } else if (!strcasecmp(val, "HOST-WRITE") || !strcasecmp(val, "USB") || !strcasecmp(val, "HOST")) {
                owner = DONGLE_DISK_OWNER_USB;
            } else if (!strcasecmp(val, "DEVICE-WRITE") || !strcasecmp(val, "DEVICE") || !strcasecmp(val, "DONGLE")) {
                owner = DONGLE_DISK_OWNER_DEVICE;
            } else {
                r_error();
                return;
            }
            if (!dongle_disk_set_owner(owner)) {
                cdc_print("\r\nDISK OWNER-ERR\r\n");
                r_error();
                return;
            }
            r_ok();
            return;
        }
        if (op && op != '?') { r_error(); return; }
        dongle_disk_status_t st;
        dongle_disk_get_status(&st);
        char b[240];
        snprintf(b, sizeof(b),
                 "\r\nmdns=%s.local  http=%s  mode=%s  msc_present=%s  msc_writable=%s  fs_mounted=%s\r\n"
                 "partition=%lu bytes\r\n",
                 st.mdns_host,
                 st.http_ready ? "up" : "down",
                 dongle_disk_owner_name(st.owner),
                 st.msc_present ? "yes" : "no",
                 st.msc_writable ? "yes" : "no",
                 st.mounted ? "yes" : "no",
                 (unsigned long)st.partition_bytes);
        cdc_print(b);
        r_ok();
    } else if (!strcmp(key, "RESET")) {
        // AT$RESET -- software reboot. Differs from AT$BOOT (which jumps into the
        // ROM bootloader for esptool) -- this just esp_restarts, which boots the
        // currently-active OTA partition. Useful for "kick the firmware" without
        // re-flashing.
        cdc_print("\r\nRESETTING\r\n");
        delay(150);
        esp_restart();
        /* unreachable */
    } else if (!strcmp(key, "SCAN")) {
        // AT$SCAN -- synchronous WiFi network scan. Blocks ~3-5s while scanning.
        cdc_print("\r\nscanning...\r\n");
        int n = WiFi.scanNetworks(false /*async*/, true /*show hidden*/);
        if (n < 0) {
            cdc_print("\r\nscan failed\r\n"); r_error(); return;
        }
        char b[120];
        snprintf(b, sizeof(b), "\r\n%d networks:\r\n", n); cdc_print(b);
        for (int i = 0; i < n; i++) {
            // SSID | RSSI(dBm) | channel | encryption | BSSID
            const char *enc = "open";
            switch (WiFi.encryptionType(i)) {
                case WIFI_AUTH_WEP:            enc = "WEP"; break;
                case WIFI_AUTH_WPA_PSK:        enc = "WPA"; break;
                case WIFI_AUTH_WPA2_PSK:       enc = "WPA2"; break;
                case WIFI_AUTH_WPA_WPA2_PSK:   enc = "WPA/2"; break;
                case WIFI_AUTH_WPA2_ENTERPRISE:enc = "WPA2-EAP"; break;
                case WIFI_AUTH_WPA3_PSK:       enc = "WPA3"; break;
                default: break;
            }
            snprintf(b, sizeof(b), "  %3d dBm  ch %2d  %-9s  %s  %s\r\n",
                     (int)WiFi.RSSI(i), (int)WiFi.channel(i), enc,
                     WiFi.BSSIDstr(i).c_str(), WiFi.SSID(i).c_str());
            cdc_print(b);
        }
        WiFi.scanDelete();
        r_ok();
    } else if (!strcmp(key, "HELP")) {
        cdc_print("\r\nAT$WIFI=<ssid>,<pw>  set both + connect (auto)\r\n"
                     "AT$SSID=<ssid>       set SSID  (auto-connects)\r\n"
                     "AT$PASS=<pw>         set pass  (auto-connects)\r\n"
                     "AT$WIFI              reconnect with stored creds\r\n"
                     "AT$WIFI?             status   AT$SSID?  AT$PASS?\r\n"
                     "AT$MODE? / =SLIP|MODEM   personality   (NVS-persisted)\r\n"
                     "AT$RSSI?             just the dBm value (scriptable)\r\n"
                     "AT$SCAN              list WiFi networks (~3-5 s)\r\n"
                     "AT$STATS  / =0       SLIP byte+pkt counters / clear\r\n"
                     "AT$DISK? / =HOST-WRITE|DEVICE-WRITE|FORMAT  USB MSC / HTTP disk ownership\r\n"
                     "AT$TYPE=<str>        send keystrokes via HID keyboard\r\n"
                     "                     (tokens: <ENTER> <F1> <CTRL+C> etc.)\r\n"
                     "AT$RESET             software reboot (esp_restart)\r\n"
                     "AT$OTASTART=<size>   USB-CDC OTA: stream <size> B fw.bin\r\n"
                     "AT$BOOT              fallback: reboot into ROM bootloader\r\n"
                     "AT&V / AT&W / AT&F   view config / save E,V,N / factory reset\r\n");
        r_ok();
    } else {
        r_error();
    }
}

static void handle_ampersand(char *s) {
    // AT&V -- view config.  AT&W -- save E/V/N to NVS.  AT&F -- factory defaults.
    char c = (char)toupper((unsigned char)s[0]);
    if (c == 'V') {
        char b[220];
        struct slip_stats st; modem_get_slip_stats(&st);
        snprintf(b, sizeof(b),
                 "\r\nATE%d  ATV%d  ATNET%d"
                 "\r\nWiFi: %s @ %s  RSSI %d dBm"
                 "\r\npeer: %s%s"
                 "\r\nstats: h->n %u pkts/%u B   n->h %u pkts/%u B\r\n",
                 echo?1:0, verbose?1:0, telnet?1:0,
                 wifi_ssid()[0] ? wifi_ssid() : "(none)",
                 WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString().c_str() : "no ip",
                 WiFi.status()==WL_CONNECTED ? (int)WiFi.RSSI() : 0,
                 peer[0] ? peer : "(none)",
                 online ? "  [online]" : "",
                 (unsigned)st.pkts_in,  (unsigned)st.bytes_in,
                 (unsigned)st.pkts_out, (unsigned)st.bytes_out);
        cdc_print(b);
        r_ok();
    } else if (c == 'W') {
        wifi_save_modem_settings(echo, verbose, telnet);
        r_ok();
    } else if (c == 'F') {
        echo = true; verbose = true; telnet = true;
        wifi_save_modem_settings(echo, verbose, telnet);
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
        case '&': handle_ampersand(rest); return;
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

// online: host(USB) -> TCP, with +++ escape + telnet IAC-doubling + CR->CRLF.
//
// Reads from Serial in batches and accumulates the post-transform bytes into
// txbuf, then ONE client.write per batch. With client.setNoDelay(true) every
// per-byte write was its own TCP segment — over lossy WiFi each one costs the
// full RTT to ACK, capping throughput at ~RTT^-1 bytes/sec. Batching pulls it
// back up to USB-CDC line rate and dramatically reduces per-byte overhead in
// poor-signal conditions where retransmits would otherwise be per-byte too.
static void pump_usb_to_tcp() {
    uint8_t inbuf[256];
    uint8_t txbuf[512];               // worst case: every byte is IAC -> 2x
    size_t  txlen = 0;
    uint32_t now = millis();

    while (client.connected()) {
        int n = (int)cdc_read_raw(inbuf, sizeof(inbuf));
        if (n <= 0) break;
        for (int i = 0; i < n; ++i) {
            uint8_t ch = inbuf[i];
            if (ch == '+' && plus_count < 3 &&
                (plus_count > 0 || (now - last_tx_ms) > GUARD_MS)) {
                if (txlen) { client.write(txbuf, txlen); txlen = 0; }
                plus_count++; plus_time = now;
                continue;
            }
            while (plus_count > 0) { txbuf[txlen++] = '+'; plus_count--; }
            if (telnet && ch == 0x0DU && !bget(local_on, OPT_BINARY)) {
                txbuf[txlen++] = 0x0D;
                txbuf[txlen++] = 0x0A;
            } else {
                if (telnet && ch == TN_IAC) txbuf[txlen++] = TN_IAC;
                txbuf[txlen++] = ch;
            }
            last_tx_ms = now;
            if (txlen >= sizeof(txbuf) - 2) {
                client.write(txbuf, txlen); txlen = 0;
            }
        }
    }
    if (txlen) client.write(txbuf, txlen);
}

// online: TCP -> host(USB). Gated on USB write room (host backpressure);
// telnet negotiation handled inline; only payload bytes reach the USB.
//
// Same batching shape as pump_usb_to_tcp: read a chunk from client, run the
// telnet IAC state machine over it accumulating non-IAC payload, single
// cdc_write per batch. Previously did cdc_byte per byte, which flushes the
// TinyUSB TX endpoint per call (~1 bulk-IN packet per byte = ~2 KB/s).
static void pump_tcp_to_usb() {
    uint8_t inbuf[512];
    uint8_t outbuf[512];
    size_t  outlen = 0;

    // No tud_cdc_n_write_available gate here: the FIFO is 64 bytes total, so
    // any "> 64" check is permanently false; "> 0" would be wrong because
    // cdc_write blocks anyway and we'd just burn CPU when the host is slow.
    // cdc_write's own ~500ms-deadline loop is the right place for backpressure.
    while (client.available()) {
        int avail = client.available();
        int want  = avail > (int)sizeof(inbuf) ? (int)sizeof(inbuf) : avail;
        int n = client.read(inbuf, want);
        if (n <= 0) break;
        for (int i = 0; i < n; ++i) {
            uint8_t ch = inbuf[i];
            if (!telnet) {
                outbuf[outlen++] = ch;
                if (outlen >= sizeof(outbuf)) { cdc_write(outbuf, outlen); outlen = 0; }
                continue;
            }
            switch (tstate) {
                case T_DATA:
                    if (ch == TN_IAC) tstate = T_IAC;
                    else {
                        outbuf[outlen++] = ch;
                        if (outlen >= sizeof(outbuf)) { cdc_write(outbuf, outlen); outlen = 0; }
                    }
                    break;
                case T_IAC:
                    if (ch == TN_IAC) {
                        outbuf[outlen++] = TN_IAC;
                        if (outlen >= sizeof(outbuf)) { cdc_write(outbuf, outlen); outlen = 0; }
                        tstate = T_DATA;
                    } else if (ch == TN_WILL || ch == TN_WONT || ch == TN_DO || ch == TN_DONT) {
                        tcmd = ch; tstate = T_OPT;
                    } else if (ch == TN_SB) {
                        tstate = T_SB_OPT;
                    } else {
                        tstate = T_DATA;
                    }
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
    if (outlen) cdc_write(outbuf, outlen);
}

void modem_poll() {
    if (!online) {
        // Bulk-read up to 256 chars/iter from TinyUSB CDC FIFO (max real depth
        // is CFG_TUD_CDC_RX_BUFSIZE=64), feed them through the AT parser. The
        // FIFO refills next iter; no per-byte queue traffic.
        uint8_t ibuf[256];
        size_t n = cdc_read_raw(ibuf, sizeof(ibuf));
        for (size_t i = 0; i < n; i++) feed_cmd(ibuf[i]);
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
