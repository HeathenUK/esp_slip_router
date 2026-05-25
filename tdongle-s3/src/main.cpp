//
// esp_slip_router — T-Dongle S3 port
//
// Role: WiFi STA + NAT router. The USB CDC serial link carries SLIP (RFC1055).
// A host on the other end (a DOS machine running a SLIP packet driver, in the
// target use case) gets transparent internet access; traffic is NAPT'd onto
// WiFi. lwIP supplies NAPT and DNS, so the link is plain SLIP with no modem
// handshake.
//
// Data path:
//   Host --USB CDC (SLIP)--> [slip_poll de-frames] --> tcpip_input --> lwIP
//   lwIP --routes to STA, NAPT rewrites src--> WiFi --> internet
//   replies --NAPT reverse--> slip_nif.output --> [SLIP encode] --> USB CDC --> Host
//
// This first cut hardcodes config (see include/config.h); the TCP console and
// NVS-backed config come next.
//

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "display.h"

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

// Debug output normally goes to the hardware UART (GPIO TX/RX pads), NOT the
// USB CDC, because USB CDC is the SLIP data link and must stay binary-clean.
// For hardware bring-up, -DDEBUG_TO_USB routes it to USB so it's readable
// without wiring up the UART pins (safe while there's no SLIP traffic yet).
#ifdef DEBUG_TO_USB
#define DBG(...) do { Serial.printf(__VA_ARGS__); } while (0)
#else
#define DBG(...) do { Serial0.printf(__VA_ARGS__); } while (0)
#endif

// ---- SLIP framing (RFC1055) ----
static const uint8_t SLIP_END     = 0xC0;
static const uint8_t SLIP_ESC     = 0xDB;
static const uint8_t SLIP_ESC_END = 0xDC;
static const uint8_t SLIP_ESC_ESC = 0xDD;

static struct netif slip_nif;
static bool napt_enabled = false;

// Stats (mirrors the original's Bytes_in / Bytes_out feel). Plain counters —
// they're written from one task each and only read for the debug heartbeat, so
// exact atomicity doesn't matter.
static uint32_t pkts_to_host = 0;
static uint32_t pkts_from_host = 0;

// ---------------------------------------------------------------------------
// Outbound: lwIP hands us an IP packet -> SLIP-encode -> USB CDC.
// Runs in the tcpip thread (single-threaded), so a static scratch buffer is OK.
// ---------------------------------------------------------------------------
static uint8_t txbuf[2 * (SLIP_MTU + 64) + 2];

static err_t slip_output(struct netif *nif, struct pbuf *p, const ip4_addr_t *ipaddr) {
    (void)nif;
    (void)ipaddr;
    size_t n = 0;
    txbuf[n++] = SLIP_END;  // leading END flushes any line noise on the host side
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        const uint8_t *d = (const uint8_t *)q->payload;
        for (uint16_t i = 0; i < q->len; i++) {
            if (n + 2 >= sizeof(txbuf)) return ERR_BUF;  // oversized; drop
            uint8_t c = d[i];
            if (c == SLIP_END) {
                txbuf[n++] = SLIP_ESC;
                txbuf[n++] = SLIP_ESC_END;
            } else if (c == SLIP_ESC) {
                txbuf[n++] = SLIP_ESC;
                txbuf[n++] = SLIP_ESC_ESC;
            } else {
                txbuf[n++] = c;
            }
        }
    }
    txbuf[n++] = SLIP_END;
    Serial.write(txbuf, n);  // setTxTimeoutMs(0) keeps this from blocking lwIP
    pkts_to_host++;
    return ERR_OK;
}

static err_t slip_if_init(struct netif *nif) {
    nif->name[0] = 's';
    nif->name[1] = 'l';
    nif->output = slip_output;       // ip4 output
    nif->mtu = SLIP_MTU;
    // Point-to-point IP link: no broadcast, no ARP/ethernet. (lwIP dropped the
    // old NETIF_FLAG_POINTTOPOINT; the absence of BROADCAST/ETHARP is enough.)
    nif->flags = NETIF_FLAG_LINK_UP;
    return ERR_OK;
}

static void slip_start() {
    ip4_addr_t ip, nm, gw;
    IP4_ADDR(&ip, SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D);
    IP4_ADDR(&nm, 255, 255, 255, 0);
    IP4_ADDR(&gw, SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D);

    LOCK_TCPIP_CORE();
    // input = tcpip_input: thread-safe hand-off from the Arduino loop task.
    netif_add(&slip_nif, &ip, &nm, &gw, NULL, slip_if_init, tcpip_input);
    netif_set_up(&slip_nif);
    netif_set_link_up(&slip_nif);
    // NOTE: intentionally NOT the default netif — the WiFi STA owns the default
    // route so outbound packets head to the internet and get NAPT'd.
    UNLOCK_TCPIP_CORE();

    DBG("[slip] netif up %d.%d.%d.%d/24\n",
        SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D);
}

// ---------------------------------------------------------------------------
// Inbound: read USB CDC, de-frame SLIP, deliver IP packets to lwIP.
// Runs in the Arduino loop task.
// ---------------------------------------------------------------------------
static uint8_t rxbuf[SLIP_MTU + 64];
static size_t rxlen = 0;
static bool in_esc = false;

static void slip_deliver(const uint8_t *data, size_t len) {
    if (len == 0) return;
    struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (p == NULL) {
        DBG("[slip] pbuf alloc fail (%u)\n", (unsigned)len);
        return;
    }
    pbuf_take(p, data, len);
    // slip_nif.input == tcpip_input; it takes ownership on ERR_OK.
    if (slip_nif.input(p, &slip_nif) != ERR_OK) {
        pbuf_free(p);
    } else {
        pkts_from_host++;
    }
}

static void slip_poll() {
    while (Serial.available() > 0) {
        int ci = Serial.read();
        if (ci < 0) break;
        uint8_t c = (uint8_t)ci;

        if (in_esc) {
            if (c == SLIP_ESC_END) c = SLIP_END;
            else if (c == SLIP_ESC_ESC) c = SLIP_ESC;
            // else: protocol violation — keep the byte verbatim
            if (rxlen < sizeof(rxbuf)) rxbuf[rxlen++] = c;
            in_esc = false;
        } else if (c == SLIP_END) {
            slip_deliver(rxbuf, rxlen);
            rxlen = 0;
        } else if (c == SLIP_ESC) {
            in_esc = true;
        } else {
            if (rxlen < sizeof(rxbuf)) {
                rxbuf[rxlen++] = c;
            } else {
                // overrun: drop the frame, resync on next END
                rxlen = 0;
                in_esc = false;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// NAPT: enable masquerading on the WiFi STA interface once it has an IP.
// ---------------------------------------------------------------------------
static void enable_napt() {
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta == NULL) {
        DBG("[napt] no STA netif handle\n");
        return;
    }
    struct netif *lwip_sta = (struct netif *)esp_netif_get_netif_impl(sta);
    if (lwip_sta == NULL) {
        DBG("[napt] no lwip netif\n");
        return;
    }
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
            DBG("[wifi] disconnected, reconnecting\n");
            napt_enabled = false;
            WiFi.reconnect();
            break;
        default:
            break;
    }
}

void setup() {
    // USB CDC = SLIP data link. Don't block lwIP if the host isn't reading.
    Serial.begin(115200);
    Serial.setRxBufferSize(2048);
#ifndef DEBUG_TO_USB
    Serial.setTxTimeoutMs(0);  // (when debugging to USB, keep prints reliable)
#endif

    // Hardware UART for human-readable debug (keeps USB CDC binary-clean).
    Serial0.begin(115200);
#ifdef DEBUG_TO_USB
    delay(2000);  // give a USB serial reader time to attach during bring-up
#endif
    DBG("\n[boot] esp_slip_router (T-Dongle S3)\n");

    WiFi.onEvent(on_wifi_event);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    DBG("[wifi] connecting to \"%s\"\n", WIFI_SSID);

    DBG("[disp] init...\n");
    display_init();
    DBG("[disp] init returned\n");

    slip_start();
    DBG("[boot] setup complete\n");
}

void loop() {
    slip_poll();

    uint32_t now = millis();

    // Refresh the status screen ~1 Hz (draw_line only repaints what changed).
    static uint32_t t_disp = 0;
    if (now - t_disp > 1000) {
        t_disp = now;
        display_status(WiFi.status() == WL_CONNECTED, WiFi.localIP(),
                       napt_enabled, pkts_from_host, pkts_to_host);
    }

    // Lightweight status heartbeat on the debug UART.
    static uint32_t t_log = 0;
    if (now - t_log > 5000) {
        t_log = now;
        DBG("[stat] wifi=%d napt=%d  host->net=%lu  net->host=%lu\n",
            WiFi.status() == WL_CONNECTED, napt_enabled,
            (unsigned long)pkts_from_host, (unsigned long)pkts_to_host);
    }
}
