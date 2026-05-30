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

extern "C" {
#include "esp32-hal-tinyusb.h"   /* tud_cdc_n_write / tud_cdc_n_write_flush */
}

// Direct TinyUSB CDC write that bypasses USBCDC's `connected` gate -- same
// rationale as modem.cpp's cdc_write. SLIP-mode lwIP traffic was hitting the
// gated Serial.write and being silently dropped on hosts that don't propagate
// DTR (DOSBox directserial, anything that didn't pass SET_CONTROL_LINE_STATE).
static size_t cdc_write_raw(const uint8_t *buf, size_t n) {
    size_t total = 0;
    uint32_t start = millis();
    while (total < n) {
        size_t w = tud_cdc_n_write(0, buf + total, n - total);
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

// AT&W -- persist modem E/V/N flags. Stored as a single byte for compactness.
void wifi_save_modem_settings(bool echo, bool verbose, bool telnet) {
    uint8_t b = (echo ? 1 : 0) | (verbose ? 2 : 0) | (telnet ? 4 : 0);
    g_prefs.putUChar("evn", b);
}
void wifi_load_modem_settings(bool *echo, bool *verbose, bool *telnet) {
    // Default is all-on (echo=verbose=telnet=true), matching the boot defaults.
    uint8_t b = g_prefs.getUChar("evn", 0x07);
    *echo    = (b & 1) != 0;
    *verbose = (b & 2) != 0;
    *telnet  = (b & 4) != 0;
}

// ---------------------------------------------------------------------------
// SLIP: outbound (lwIP -> SLIP-encode -> USB CDC). tcpip-thread context.
// ---------------------------------------------------------------------------
static uint8_t txbuf[2 * (SLIP_MTU + 64) + 2];

static err_t slip_output(struct netif *nif, struct pbuf *p, const ip4_addr_t *ipaddr) {
    (void)nif; (void)ipaddr;
    // Belt-and-braces: even if apply_mode missed bringing the netif fully down,
    // we MUST NOT emit SLIP-framed bytes onto a modem CDC stream. Quietly drop.
    if (g_mode != MODE_SLIP) return ERR_OK;
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
    // cdc_write_raw bypasses USBCDC's `connected` gate (same reason as modem.cpp).
    cdc_write_raw(txbuf, n);
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

static void apply_mode();    // fwd-decl: slip_deliver's magic-frame uses it

static void slip_deliver(const uint8_t *data, size_t len) {
    if (len == 0) return;
    // SLIP-mode escape hatch: a frame whose payload is exactly the ASCII
    // sentinel below flips the personality back to MODEM. No valid IPv4
    // packet matches (first byte would have to be 'M' = 0x4D, version nibble
    // 4 -- but the rest of the header bytes wouldn't be ASCII). Lets a host
    // get out of SLIP without holding BOOT or pulling power.
    static const char MAGIC[] = "MODE=MODEM";
    if (len == sizeof(MAGIC) - 1 && memcmp(data, MAGIC, sizeof(MAGIC) - 1) == 0) {
        DBG("[slip] magic escape frame -> MODEM\n");
        g_mode = MODE_MODEM;
        g_prefs.putUChar("mode", (uint8_t)g_mode);
        apply_mode();
        return;
    }
    // PBUF_IP reserves headroom for an L2 header below the IP layer. Critical
    // when the packet is FORWARDED to an Ethernet netif (WiFi STA via NAPT):
    // etharp_output prepends 14 bytes of L2 via pbuf_header(-14), which fails
    // silently on PBUF_RAW (zero headroom). Symptom: SLIP local-delivery works
    // but anything-via-NAPT vanishes without a trace.
    struct pbuf *p = pbuf_alloc(PBUF_IP, len, PBUF_POOL);
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
    // Bound the drain so a sustained host flood can't starve loop() (button
    // polling, WiFi events, display refresh). 1024 bytes/iter @ ~1kHz loop
    // sustains > 1 MB/s of SLIP throughput -- well above USB CDC line rate.
    int budget = 1024;
    while (budget-- > 0 && Serial.available() > 0) {
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

// ---- NAPT on the SLIP (internal) netif ----
// The ESP-IDF NAPT API is inverted vs typical NAT: ip_napt_forward() only
// translates when the INPUT netif has napt=1, then rewrites source to the
// OUTPUT netif's address. So napt belongs on the *internal* side (where
// our clients live), not on the WAN-side STA. Symptom of getting this
// backwards: SLIP packets to public IPs vanish silently — forwarded with
// source 192.168.240.2 unchanged, dropped by the upstream router.
static void enable_napt() {
    LOCK_TCPIP_CORE();
    int ok = ip_napt_enable_netif(&slip_nif, 1);
    UNLOCK_TCPIP_CORE();
    napt_enabled = (ok == 1);
    DBG("[napt] enable on SLIP -> %s\n", ok ? "ok" : "FAILED");
}

static void on_wifi_event(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            DBG("[wifi] got IP %s\n", WiFi.localIP().toString().c_str());
            // NAPT lives on the SLIP netif (enabled at boot) -- nothing to do here.
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
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
        LOCK_TCPIP_CORE();
        netif_set_up(&slip_nif);       // admin up + link up so lwIP routes here
        netif_set_link_up(&slip_nif);
        UNLOCK_TCPIP_CORE();
        DBG("[mode] SLIP\n");
    } else {
        // Take the SLIP netif fully out of lwIP routing in modem mode -- both
        // link_down and admin_down. slip_output also has a g_mode check as
        // defense in depth, but a properly down netif means lwIP never picks
        // it for output in the first place.
        LOCK_TCPIP_CORE();
        netif_set_link_down(&slip_nif);
        netif_set_down(&slip_nif);
        UNLOCK_TCPIP_CORE();
        modem_enter();
        DBG("[mode] MODEM\n");
    }
}

static void toggle_mode() {
    g_mode = (g_mode == MODE_SLIP) ? MODE_MODEM : MODE_SLIP;
    g_prefs.putUChar("mode", (uint8_t)g_mode);
    apply_mode();
}

// AT$MODE=SLIP|MODEM -- host-side personality switch (vs the 1.5s BOOT-button
// long-press). Persists to NVS just like toggle_mode does. Returns the new
// mode name. AT$MODE? reports the current one.
const char *modem_set_personality(const char *want) {
    LinkMode requested = g_mode;
    if (want) {
        if (!strcasecmp(want, "SLIP"))       requested = MODE_SLIP;
        else if (!strcasecmp(want, "MODEM")) requested = MODE_MODEM;
        else                                  return NULL;   // bad arg
    }
    if (requested != g_mode) {
        g_mode = requested;
        g_prefs.putUChar("mode", (uint8_t)g_mode);
        apply_mode();
    }
    return g_mode == MODE_SLIP ? "SLIP" : "MODEM";
}

// ---- SLIP stats exposed for AT$STATS / AT&V ----
void modem_get_slip_stats(struct slip_stats *s) {
    s->pkts_in   = pkts_from_host;
    s->bytes_in  = bytes_from_host;
    s->pkts_out  = pkts_to_host;
    s->bytes_out = bytes_to_host;
}
void modem_clear_slip_stats() {
    pkts_from_host = pkts_to_host = 0;
    bytes_from_host = bytes_to_host = 0;
}

static void poll_button() {
    static bool prev = false;
    static uint32_t t0 = 0;
    static bool handled = false;
    static bool armed = false;        // require release-from-boot before any toggle
    bool down = (digitalRead(BTN_PIN) == LOW);
    if (!armed) {                     // user might be holding BOOT from power-on
        if (!down) armed = true;       // released -> arm
        prev = down; return;
    }
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
    // Maximum TX power for the worst-signal case. The T-Dongle S3 has a stub
    // PCB antenna and tends to sit far from APs in the wild (Pocket386 USB
    // port). +19.5 dBm is the chip's documented max for WIFI_PROTOCOL_11G/N
    // and stays within FCC class B for unintentional radiators -- no reason
    // to throttle.
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    g_prefs.begin("slip-router", false);
    wifi_load_and_begin();      // NVS creds (or config.h defaults) -> WiFi.begin

    display_init();
    slip_start();
    enable_napt();   // SLIP netif is up now; flip the napt bit once at boot
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
