//
// esp_slip_router — T-Dongle S3 port
//
// Dual personality on one USB-CDC link, switchable at runtime with the GPIO0
// (BOOT) button (long-press), choice persisted to NVS:
//
//   SLIP  — WiFi STA + NAPT + a point-to-point SLIP netif. The host runs a
//           packet driver + TCP/IP stack (full IP). Plain RFC1055 SLIP.
//   MODEM — a WiFi232-style Hayes engine; `ATDT host:port` opens a TCP socket
//           bridged to the serial link. Driverless on the host (any terminal).
//
// The two are mutually exclusive at any instant (one byte stream, one host
// program). Debug always goes to the hardware UART (Serial0) so USB stays a
// clean data link in both modes.
//

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <string.h>

#include "config.h"
#include "display.h"
#include "modem.h"
#include "wificfg.h"

extern "C" {
#include "lwip/opt.h"
#include "lwip/netif.h"
#include "lwip/ip.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"
#include "lwip/lwip_napt.h"
#include "lwip/dns.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
}

#define DBG(...) do { Serial0.printf(__VA_ARGS__); } while (0)

#define BTN_PIN 0   // GPIO0 / BOOT button (active low); read post-boot only

// ---- SLIP framing (RFC1055) ----
static const uint8_t SLIP_END     = 0xC0;
static const uint8_t SLIP_ESC     = 0xDB;
static const uint8_t SLIP_ESC_END = 0xDC;
static const uint8_t SLIP_ESC_ESC = 0xDD;

static struct netif slip_nif;
static bool napt_enabled = false;
static uint32_t pkts_to_host = 0;
static uint32_t pkts_from_host = 0;
static uint32_t bytes_to_host = 0;    // net -> host (the DOS box's "download")
static uint32_t bytes_from_host = 0;  // host -> net ("upload")

// ---- personality ----
enum LinkMode { MODE_MODEM = 0, MODE_SLIP = 1 };
static LinkMode g_mode = MODE_MODEM;
static Preferences g_prefs;

// ---- WiFi credentials (NVS-backed, shared by both personalities) ----
static char g_ssid[33];
static char g_pass[65];

// Deferred reconfigure: disconnect now, then WiFi.begin() a beat later, so we
// don't call set_config while the STA is still mid-transition (ESP_ERR_WIFI_STATE).
static volatile bool g_wifi_reconfig = false;
static uint32_t g_wifi_reconfig_at = 0;

// Debounced auto-apply: setting SSID/PASS arms this so a quick SSID+PASS pair
// coalesces into one connect (instead of a doomed attempt after just the SSID).
static uint32_t g_wifi_apply_at = 0;

void wifi_load_and_begin() {
    String s = g_prefs.getString("ssid", WIFI_SSID);   // NVS, else config.h default
    String p = g_prefs.getString("pass", WIFI_PASS);
    strncpy(g_ssid, s.c_str(), sizeof(g_ssid) - 1); g_ssid[sizeof(g_ssid) - 1] = 0;
    strncpy(g_pass, p.c_str(), sizeof(g_pass) - 1); g_pass[sizeof(g_pass) - 1] = 0;
    WiFi.begin(g_ssid, g_pass);
    DBG("[wifi] connecting to \"%s\"\n", g_ssid);
}

void wifi_set_ssid(const char *s) {
    strncpy(g_ssid, s, sizeof(g_ssid) - 1); g_ssid[sizeof(g_ssid) - 1] = 0;
    g_prefs.putString("ssid", g_ssid);
    DBG("[wifi] ssid set \"%s\"\n", g_ssid);
}

void wifi_set_pass(const char *p) {
    strncpy(g_pass, p, sizeof(g_pass) - 1); g_pass[sizeof(g_pass) - 1] = 0;
    g_prefs.putString("pass", g_pass);
    DBG("[wifi] pass set (%u chars)\n", (unsigned)strlen(g_pass));
}

const char *wifi_ssid() { return g_ssid; }
bool wifi_has_pass()    { return g_pass[0] != 0; }

void wifi_reconnect() {
    napt_enabled = false;
    g_wifi_apply_at = 0;              // cancel any pending debounced apply
    WiFi.disconnect(true, false);     // wifioff=true: radio off, nothing can re-dial
    g_wifi_reconfig = true;
    g_wifi_reconfig_at = millis() + 500;   // bring it back up + begin() once settled
    DBG("[wifi] reconnect scheduled for \"%s\"\n", g_ssid);
}

void wifi_apply_soon() { g_wifi_apply_at = millis() + 2500; }

// ---------------------------------------------------------------------------
// SLIP: outbound (lwIP -> SLIP-encode -> USB CDC). tcpip-thread context.
// ---------------------------------------------------------------------------
static uint8_t txbuf[2 * (SLIP_MTU + 64) + 2];

static err_t slip_output(struct netif *nif, struct pbuf *p, const ip4_addr_t *ipaddr) {
    (void)nif; (void)ipaddr;
    size_t n = 0;
    txbuf[n++] = SLIP_END;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        const uint8_t *d = (const uint8_t *)q->payload;
        for (uint16_t i = 0; i < q->len; i++) {
            if (n + 2 >= sizeof(txbuf)) return ERR_BUF;
            uint8_t c = d[i];
            if (c == SLIP_END)      { txbuf[n++] = SLIP_ESC; txbuf[n++] = SLIP_ESC_END; }
            else if (c == SLIP_ESC) { txbuf[n++] = SLIP_ESC; txbuf[n++] = SLIP_ESC_ESC; }
            else                    { txbuf[n++] = c; }
        }
    }
    txbuf[n++] = SLIP_END;
    Serial.write(txbuf, n);
    pkts_to_host++;
    bytes_to_host += p->tot_len;   // count IP payload, not SLIP-encoded size
    return ERR_OK;
}

static err_t slip_if_init(struct netif *nif) {
    nif->name[0] = 's'; nif->name[1] = 'l';
    nif->output = slip_output;
    nif->mtu = SLIP_MTU;
    nif->flags = NETIF_FLAG_LINK_UP;   // point-to-point IP: no broadcast/ARP
    return ERR_OK;
}

static void slip_start() {
    ip4_addr_t ip, nm, gw;
    IP4_ADDR(&ip, SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D);
    IP4_ADDR(&nm, 255, 255, 255, 0);
    IP4_ADDR(&gw, SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D);
    LOCK_TCPIP_CORE();
    netif_add(&slip_nif, &ip, &nm, &gw, NULL, slip_if_init, tcpip_input);
    netif_set_up(&slip_nif);
    netif_set_link_up(&slip_nif);
    UNLOCK_TCPIP_CORE();
    DBG("[slip] netif up %d.%d.%d.%d/24\n",
        SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D);
}

// ---- SLIP: inbound (USB CDC -> de-frame -> lwIP). loop-task context. ----
static uint8_t rxbuf[SLIP_MTU + 64];
static size_t rxlen = 0;
static bool in_esc = false;

static void slip_deliver(const uint8_t *data, size_t len) {
    if (len == 0) return;
    struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (!p) { DBG("[slip] pbuf alloc fail (%u)\n", (unsigned)len); return; }
    pbuf_take(p, data, len);
    if (slip_nif.input(p, &slip_nif) != ERR_OK) {
        pbuf_free(p);
    } else {
        pkts_from_host++;
        bytes_from_host += len;
    }
}

static void slip_poll() {
    while (Serial.available() > 0) {
        int ci = Serial.read(); if (ci < 0) break;
        uint8_t c = (uint8_t)ci;
        if (in_esc) {
            if (c == SLIP_ESC_END) c = SLIP_END;
            else if (c == SLIP_ESC_ESC) c = SLIP_ESC;
            if (rxlen < sizeof(rxbuf)) rxbuf[rxlen++] = c;
            in_esc = false;
        } else if (c == SLIP_END) {
            slip_deliver(rxbuf, rxlen); rxlen = 0;
        } else if (c == SLIP_ESC) {
            in_esc = true;
        } else {
            if (rxlen < sizeof(rxbuf)) rxbuf[rxlen++] = c;
            else { rxlen = 0; in_esc = false; }   // overrun: resync on next END
        }
    }
}

// ---- NAPT on the WiFi STA interface ----
static void enable_napt() {
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!sta) { DBG("[napt] no STA netif handle\n"); return; }
    struct netif *lwip_sta = (struct netif *)esp_netif_get_netif_impl(sta);
    if (!lwip_sta) { DBG("[napt] no lwip netif\n"); return; }
    LOCK_TCPIP_CORE();
    ip_napt_enable_netif(lwip_sta, 1);
    UNLOCK_TCPIP_CORE();
    napt_enabled = true;
    DBG("[napt] enabled on STA\n");
}

static void on_wifi_event(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            DBG("[wifi] got IP %s\n", WiFi.localIP().toString().c_str());
            enable_napt();
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            napt_enabled = false;
            if (!g_wifi_reconfig) {       // a real drop, not an intentional reconfigure
                DBG("[wifi] disconnected, reconnecting\n");
                WiFi.reconnect();
            }
            break;
        default: break;
    }
}

// ---- personality switching ----
static void apply_mode() {
    if (g_mode == MODE_SLIP) {
        modem_leave();
        LOCK_TCPIP_CORE(); netif_set_link_up(&slip_nif); UNLOCK_TCPIP_CORE();
        DBG("[mode] SLIP\n");
    } else {
        // Drop the SLIP link so lwIP never emits SLIP bytes onto a modem stream.
        LOCK_TCPIP_CORE(); netif_set_link_down(&slip_nif); UNLOCK_TCPIP_CORE();
        modem_enter();
        DBG("[mode] MODEM\n");
    }
}

static void toggle_mode() {
    g_mode = (g_mode == MODE_SLIP) ? MODE_MODEM : MODE_SLIP;
    g_prefs.putUChar("mode", (uint8_t)g_mode);
    apply_mode();
}

static void poll_button() {
    static bool prev = false;
    static uint32_t t0 = 0;
    static bool handled = false;
    bool down = (digitalRead(BTN_PIN) == LOW);
    if (down && !prev) { t0 = millis(); handled = false; }
    if (down && !handled && (millis() - t0) > 1500) { toggle_mode(); handled = true; }
    prev = down;
}

void setup() {
    Serial.begin(115200);
    Serial.setRxBufferSize(2048);
    // NOTE: was Serial.setTxTimeoutMs(0) here — intended to make Serial.write
    // fire-and-forget, but USBCDC interprets a 0 timeout as "timeout expires
    // immediately = send nothing", so every write was silently dropped. Default
    // (250 ms) is fine for the modem path; SLIP throughput is well below it.

    // T-Dongle S3 has no auto-reset circuit, so the esptool DTR/RTS state
    // machine inside USBCDC::_onLineState can't do its job here — it can only
    // get in the way by gating `connected` on a specific 4-step DTR/RTS pattern
    // that some hosts (DOSBox directserial, USB-Serial-JTAG) don't replicate.
    // Disable it so connected = (dtr && rts) cleanly, unblocking those hosts.
    Serial.enableReboot(false);

    Serial0.begin(115200);      // human-readable debug, off the USB data link
    pinMode(BTN_PIN, INPUT);    // GPIO0 has an external pull-up; pressed = LOW
    DBG("\n[boot] esp_slip_router (T-Dongle S3)\n");

    WiFi.onEvent(on_wifi_event);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    g_prefs.begin("slip-router", false);
    wifi_load_and_begin();      // NVS creds (or config.h defaults) -> WiFi.begin

    display_init();
    slip_start();
    modem_begin();

    g_mode = (LinkMode)g_prefs.getUChar("mode", MODE_MODEM);  // default: modem
    apply_mode();
    DBG("[boot] setup complete, mode=%s\n", g_mode == MODE_SLIP ? "SLIP" : "MODEM");
}

void loop() {
    poll_button();

    if (g_mode == MODE_SLIP) slip_poll();
    else                     modem_poll();

    uint32_t now = millis();

    // Debounced auto-apply after a credential change.
    if (g_wifi_apply_at && (int32_t)(now - g_wifi_apply_at) >= 0) {
        g_wifi_apply_at = 0;
        wifi_reconnect();
    }

    // Deferred WiFi reconfigure (see wifi_reconnect): radio back on, then begin.
    if (g_wifi_reconfig && (int32_t)(now - g_wifi_reconfig_at) >= 0) {
        g_wifi_reconfig = false;
        WiFi.mode(WIFI_STA);
        WiFi.begin(g_ssid, g_pass);
        DBG("[wifi] begin \"%s\"\n", g_ssid);
    }

    static uint32_t t_disp = 0, last_dl = 0, last_ul = 0, last_rate_ms = 0;
    if (now - t_disp > 1000) {
        t_disp = now;
        bool w = (WiFi.status() == WL_CONNECTED);
        if (g_mode == MODE_SLIP) {
            // bytes/sec over the actual elapsed interval since the last sample
            uint32_t dt = now - last_rate_ms; if (dt == 0) dt = 1;
            uint32_t dl_bps = (uint32_t)((uint64_t)(bytes_to_host   - last_dl) * 1000 / dt);
            uint32_t ul_bps = (uint32_t)((uint64_t)(bytes_from_host - last_ul) * 1000 / dt);
            display_slip(w, WiFi.localIP(), napt_enabled, dl_bps, ul_bps);
        } else {
            display_modem(w, WiFi.localIP(), modem_is_online(), modem_peer());
        }
        last_dl = bytes_to_host; last_ul = bytes_from_host; last_rate_ms = now;
    }

    static uint32_t t_log = 0;
    if (now - t_log > 5000) {
        t_log = now;
        DBG("[stat] mode=%s wifi=%d napt=%d  host->net=%lu  net->host=%lu\n",
            g_mode == MODE_SLIP ? "SLIP" : "MODEM",
            WiFi.status() == WL_CONNECTED, napt_enabled,
            (unsigned long)pkts_from_host, (unsigned long)pkts_to_host);
    }
}
