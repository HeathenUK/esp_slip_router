/* slip.c -- RFC 1055 SLIP on the CDC interface + lwIP netif + NAPT.
 *
 * RX path: bytes arrive via modem.c's tud_cdc_rx_cb, which forwards
 * to slip_feed() when slip_get_mode() == MODE_SLIP. slip_feed runs
 * the SLIP de-framer; complete frames go to slip_deliver, which
 * dispatches "magic" frames (mode/disk switches that DOS can send
 * to escape SLIP without an out-of-band channel) and hands the rest
 * to lwIP via slip_nif.input -> tcpip_input.
 *
 * TX path: lwIP's tcpip thread calls slip_output(p) when it wants
 * to send an IP packet down the SLIP link. We SLIP-encode and
 * tud_cdc_n_write to the CDC interface.
 *
 * NAPT: enabled on the SLIP netif (the INTERNAL side). ESP-IDF's
 * NAPT API is inverted vs typical NAT -- ip_napt_enable_netif on
 * the input side does the source-rewrite. See memory
 * esp-idf-napt-api-inverted for why getting this backwards drops
 * all outbound packets silently.
 *
 * Pbuf headroom: pbuf_alloc(PBUF_RAW, ...) silently breaks
 * forwarding to Ethernet (etharp_output's pbuf_header(-14) fails on
 * zero-headroom pbufs). Use PBUF_IP. See memory
 * slip-pbuf-headroom-forwarding.
 */

#include "slip.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs.h"

#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/ip4_addr.h"
#include "lwip/tcpip.h"
#include "lwip/lwip_napt.h"

#include "tusb.h"
#include "tusb_cdc_acm.h"
#include "class/cdc/cdc_device.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "disk.h"    /* disk_logf */

#define TAG "slip"

/* RFC 1055 framing. */
#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define SLIP_MTU 1500

#define SLIP_LOCAL_A 192
#define SLIP_LOCAL_B 168
#define SLIP_LOCAL_C 240
#define SLIP_LOCAL_D 1
#define SLIP_PEER_A  192
#define SLIP_PEER_B  168
#define SLIP_PEER_C  240
#define SLIP_PEER_D  2

/* lwIP state. */
static struct netif s_slip_nif;
static bool         s_nif_added = false;

/* RX de-framer state. */
static uint8_t  s_rxbuf[SLIP_MTU + 64];
static size_t   s_rxlen = 0;
static bool     s_in_esc = false;

/* TX encode buffer. Worst case: every byte needs escaping -> 2x.
 * Plus 2 framing END bytes. */
static uint8_t  s_txbuf[2 * (SLIP_MTU + 64) + 2];

/* Mode state. */
static volatile LinkMode s_mode = MODE_MODEM;

/* Stats (volatile so /status reads them without locking). */
static volatile uint32_t s_pkts_to_host    = 0;
static volatile uint32_t s_pkts_from_host  = 0;
static volatile uint32_t s_bytes_to_host   = 0;
static volatile uint32_t s_bytes_from_host = 0;

/* ---- TX: lwIP -> SLIP-encode -> CDC ---- */

static err_t slip_output(struct netif *nif, struct pbuf *p, const ip4_addr_t *ipaddr) {
    (void)nif; (void)ipaddr;
    /* Belt-and-braces: modem mode may have a brief window where the
     * netif is still admin-up but mode has flipped. Don't ever emit
     * SLIP-framed bytes onto a modem CDC stream. */
    if (s_mode != MODE_SLIP) return ERR_OK;

    /* Bulk-encode: scan for SLIP_END / SLIP_ESC, memcpy the runs of
     * normal bytes between them, escape the specials. memcpy on
     * Xtensa beats a per-byte loop by ~5-10x. The vast majority of
     * IP packet bytes are neither 0xC0 nor 0xDB, so most of the
     * encoded output comes from the bulk path. */
    size_t n = 0;
    s_txbuf[n++] = SLIP_END;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        const uint8_t *d = (const uint8_t *)q->payload;
        uint16_t i = 0;
        while (i < q->len) {
            uint16_t j = i;
            while (j < q->len && d[j] != SLIP_END && d[j] != SLIP_ESC) ++j;
            size_t run = (size_t)(j - i);
            if (n + run + 2 > sizeof s_txbuf) return ERR_BUF;
            memcpy(s_txbuf + n, d + i, run);
            n += run;
            if (j < q->len) {
                s_txbuf[n++] = SLIP_ESC;
                s_txbuf[n++] = (d[j] == SLIP_END) ? SLIP_ESC_END : SLIP_ESC_ESC;
                ++j;
            }
            i = j;
        }
    }
    s_txbuf[n++] = SLIP_END;

    /* Push bytes through TinyUSB CDC. tud_cdc_n_write FIFO is 512
     * bytes (CFG_TUD_CDC_TX_BUFSIZE), so we loop with flush. */
    size_t sent = 0;
    int64_t deadline_iters = 200;
    while (sent < n) {
        if (!tud_cdc_n_connected(0)) break;
        size_t avail = tud_cdc_n_write_available(0);
        if (avail == 0) {
            tud_cdc_n_write_flush(0);
            if (--deadline_iters <= 0) break;
            continue;
        }
        size_t chunk = (n - sent) < avail ? (n - sent) : avail;
        size_t w = tud_cdc_n_write(0, s_txbuf + sent, chunk);
        sent += w;
    }
    tud_cdc_n_write_flush(0);

    s_pkts_to_host++;
    s_bytes_to_host += p->tot_len;
    return ERR_OK;
}

static err_t slip_if_init(struct netif *nif) {
    nif->name[0] = 's'; nif->name[1] = 'l';
    nif->output = slip_output;
    nif->mtu    = SLIP_MTU;
    nif->flags  = NETIF_FLAG_LINK_UP;     /* point-to-point; no broadcast / ARP */
    return ERR_OK;
}

/* ---- RX: CDC -> SLIP-decode -> lwIP ---- */

static void apply_mode(void);   /* fwd-decl for magic-frame use */

static void slip_deliver(const uint8_t *data, size_t len) {
    if (len == 0) return;

    /* Magic frames: DOS-side escape hatches. None of these can match
     * a valid IP packet -- the first byte would need to be the
     * ASCII letter (M/D), with version nibble 4 -- not how IP works. */
    static const char MAGIC_MODEM[] = "MODE=MODEM";
    if (len == sizeof MAGIC_MODEM - 1 && memcmp(data, MAGIC_MODEM, sizeof MAGIC_MODEM - 1) == 0) {
        disk_logf("[slip] magic frame -> MODEM");
        slip_set_mode(MODE_MODEM);
        return;
    }
    static const char MAGIC_SLIP[] = "MODE=SLIP";
    if (len == sizeof MAGIC_SLIP - 1 && memcmp(data, MAGIC_SLIP, sizeof MAGIC_SLIP - 1) == 0) {
        disk_logf("[slip] magic frame -> SLIP (noop)");
        return;
    }

    /* Real IP packet. PBUF_IP reserves L2 headroom -- critical for
     * NAPT forwarding to Ethernet (etharp_output prepends 14 bytes
     * via pbuf_header(-14), which silently fails on PBUF_RAW). */
    struct pbuf *p = pbuf_alloc(PBUF_IP, len, PBUF_POOL);
    if (!p) { disk_logf("[slip] pbuf alloc fail (%u B)", (unsigned)len); return; }
    pbuf_take(p, data, len);
    if (s_slip_nif.input(p, &s_slip_nif) != ERR_OK) {
        pbuf_free(p);
    } else {
        s_pkts_from_host++;
        s_bytes_from_host += len;
    }
}

void slip_feed(const uint8_t *buf, size_t n) {
    /* Bulk-decode: scan for SLIP_END / SLIP_ESC, memcpy runs of
     * normal bytes into the rx buffer. Per-byte handling only when
     * one of the framing specials is encountered. */
    size_t k = 0;
    while (k < n) {
        if (s_in_esc) {
            uint8_t c = buf[k++];
            if      (c == SLIP_ESC_END) c = SLIP_END;
            else if (c == SLIP_ESC_ESC) c = SLIP_ESC;
            if (s_rxlen < sizeof s_rxbuf) s_rxbuf[s_rxlen++] = c;
            s_in_esc = false;
            continue;
        }
        /* Find the next special byte. */
        size_t j = k;
        while (j < n && buf[j] != SLIP_END && buf[j] != SLIP_ESC) ++j;
        /* Bulk-copy the run buf[k..j) into the frame buffer (clipped
         * to remaining capacity; an overrun forces a resync on the
         * next END). */
        size_t run = j - k;
        if (run) {
            size_t cap = sizeof s_rxbuf - s_rxlen;
            size_t copy = run < cap ? run : cap;
            memcpy(s_rxbuf + s_rxlen, buf + k, copy);
            s_rxlen += copy;
            if (run > cap) s_rxlen = sizeof s_rxbuf; /* mark overrun via full-buf */
        }
        k = j;
        if (k >= n) break;
        uint8_t c = buf[k++];
        if (c == SLIP_END) {
            /* Frame boundary -- if we overran, drop the frame. */
            if (s_rxlen < sizeof s_rxbuf) slip_deliver(s_rxbuf, s_rxlen);
            s_rxlen = 0;
            s_in_esc = false;
        } else { /* SLIP_ESC */
            s_in_esc = true;
        }
    }
}

/* ---- NAPT on the SLIP netif ----
 *
 * ip_napt_enable_netif() flags the netif as "translate source on
 * forward". Packets ENTERING this netif (i.e., from SLIP) get their
 * source rewritten to the OUTPUT netif's address when forwarded.
 * That's what we want for the DOS-host-via-WiFi-to-internet case.
 *
 * If we enabled NAPT on the STA netif instead, packets entering
 * from STA would get rewritten -- the wrong direction. */

static void enable_napt(void) {
    LOCK_TCPIP_CORE();
    int ok = ip_napt_enable_netif(&s_slip_nif, 1);
    UNLOCK_TCPIP_CORE();
    disk_logf("[slip] napt enable -> %s", ok == 1 ? "ok" : "FAILED");
}

/* ---- apply_mode: bring netif up or down based on s_mode ---- */

static void apply_mode(void) {
    /* Default route stays on STA. lwIP routes by subnet match: traffic
     * to 192.168.240.0/24 hits the SLIP netif by virtue of the netif's
     * IP/mask, no default-route games needed. NAPT on the SLIP netif
     * rewrites source on forward, so DOS-originated packets to the
     * internet exit through STA with the STA's IP. */
    LOCK_TCPIP_CORE();
    if (s_mode == MODE_SLIP) {
        netif_set_up(&s_slip_nif);
        netif_set_link_up(&s_slip_nif);
    } else {
        netif_set_link_down(&s_slip_nif);
        netif_set_down(&s_slip_nif);
    }
    UNLOCK_TCPIP_CORE();
    disk_logf("[slip] mode -> %s", s_mode == MODE_SLIP ? "SLIP" : "MODEM");
}

/* ---- public API ---- */

LinkMode slip_get_mode(void) { return s_mode; }

esp_err_t slip_set_mode(LinkMode mode) {
    if (mode != MODE_MODEM && mode != MODE_SLIP) return ESP_ERR_INVALID_ARG;
    s_mode = mode;
    /* Persist. Best-effort -- a failure here just means we won't
     * remember mode across reboot. */
    nvs_handle_t h;
    if (nvs_open("slip-router", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "mode", (uint8_t)mode);
        nvs_commit(h);
        nvs_close(h);
    }
    /* De-frame state reset so a partial frame from before the mode
     * flip doesn't get glued to the next one. */
    s_rxlen = 0; s_in_esc = false;
    apply_mode();
    return ESP_OK;
}

/* ---- BOOT-button (GPIO0) long-press mode toggle ----
 *
 * 1.5 s hold toggles the mode. Matches the old firmware's trigger
 * shape so muscle memory carries over. Pressed = LOW (external
 * pull-up on GPIO0). The button is GPIO0 -- same pin as the boot-
 * strap pin, but we read it only AFTER boot so the strap behaviour
 * isn't affected.
 *
 * Runs on a tiny 2 KB task that wakes 20 times/s. Negligible CPU. */
#define BTN_PIN          GPIO_NUM_0
#define BTN_LONGPRESS_MS 1500

static void btn_task(void *arg) {
    (void)arg;
    /* Wait for the button to be released after boot in case the user
     * was holding it for download mode. Otherwise we'd toggle mode
     * 1.5 s in. */
    while (gpio_get_level(BTN_PIN) == 0) vTaskDelay(pdMS_TO_TICKS(50));

    int  press_ms = 0;
    bool fired    = false;
    while (1) {
        if (gpio_get_level(BTN_PIN) == 0) {
            press_ms += 50;
            if (!fired && press_ms >= BTN_LONGPRESS_MS) {
                fired = true;
                LinkMode next = (s_mode == MODE_SLIP) ? MODE_MODEM : MODE_SLIP;
                disk_logf("[slip] BOOT long-press -> %s",
                          next == MODE_SLIP ? "SLIP" : "MODEM");
                slip_set_mode(next);
            }
        } else {
            press_ms = 0;
            fired    = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t slip_init(void) {
    /* Register the netif (admin-down initially). lwIP needs
     * tcpip_input as the input func because we'll call .input from
     * the modem CDC RX callback (TinyUSB task context), and we want
     * lwIP to process on its tcpip thread. */
    ip4_addr_t ip, nm, gw;
    IP4_ADDR(&ip, SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D);
    IP4_ADDR(&nm, 255, 255, 255, 0);
    IP4_ADDR(&gw, SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D);

    LOCK_TCPIP_CORE();
    netif_add(&s_slip_nif, &ip, &nm, &gw, NULL, slip_if_init, tcpip_input);
    /* Briefly admin-up the netif so ip_napt_enable_netif accepts it
     * (it checks netif_is_up and returns 0 otherwise). apply_mode
     * below restores the right up/down state based on saved mode. */
    netif_set_up(&s_slip_nif);
    netif_set_link_up(&s_slip_nif);
    UNLOCK_TCPIP_CORE();
    s_nif_added = true;

    enable_napt();

    /* Restore mode from NVS. */
    nvs_handle_t h;
    uint8_t saved = MODE_MODEM;
    if (nvs_open("slip-router", NVS_READONLY, &h) == ESP_OK) {
        size_t sz = sizeof saved;
        (void)sz;
        nvs_get_u8(h, "mode", &saved);
        nvs_close(h);
    }
    s_mode = (saved == MODE_SLIP) ? MODE_SLIP : MODE_MODEM;
    apply_mode();

    /* BOOT-button long-press mode toggle. Configure GPIO0 as input
     * with the internal pull-up enabled (T-Dongle S3 has an external
     * pull-up too, but belt-and-braces). */
    gpio_config_t bcfg = {
        .pin_bit_mask = (1ULL << BTN_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&bcfg);
    xTaskCreatePinnedToCore(btn_task, "boot_btn", 2048, NULL, 4, NULL, 0);

    disk_logf("[slip] init: nif up, local=%d.%d.%d.%d/24, mode=%s",
              SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D,
              s_mode == MODE_SLIP ? "SLIP" : "MODEM");
    return ESP_OK;
}

uint32_t slip_stat_pkts_to_host(void)    { return s_pkts_to_host; }
uint32_t slip_stat_pkts_from_host(void)  { return s_pkts_from_host; }
uint32_t slip_stat_bytes_to_host(void)   { return s_bytes_to_host; }
uint32_t slip_stat_bytes_from_host(void) { return s_bytes_from_host; }
