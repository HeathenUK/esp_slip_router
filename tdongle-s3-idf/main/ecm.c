/* ecm.c -- CDC-ECM network function bridged to WiFi STA via lwIP/NAPT.
 *
 * RX path (host -> world): TinyUSB's net driver delivers whole Ethernet
 * frames via tud_network_recv_cb (TinyUSB task, CPU1). We copy into a
 * pbuf and post to the tcpip thread (netif.input == tcpip_input, which
 * dispatches to ethernet_input for ETHARP netifs). NAPT on this netif
 * rewrites the source on forward, so host-originated packets exit the
 * STA with the STA's IP -- same inverted-API rule as the SLIP netif
 * (see memory esp-idf-napt-api-inverted).
 *
 * TX path (world -> host): lwIP's tcpip thread calls ecm_linkoutput.
 * TinyUSB's driver owns one in-flight frame (can_xmit); if the host
 * isn't draining (CH375 polls slowly), we wait briefly then DROP --
 * never wedge the tcpip thread (the plan's backpressure rule: drops
 * land at lwIP, TCP retransmits).
 *
 * Addressing: the host NIC's MAC comes from the iMACAddress string
 * descriptor (tud_network_mac_address); the bridge netif uses the same
 * MAC with the last byte flipped, exactly like TinyUSB's
 * net_lwip_webserver example (MIT, Peter Lawrence) that this glue is
 * modeled on.
 *
 * DHCP: vendored dhserver (see dhserver.h) leases 192.168.241.2+ with
 * router + DNS = 192.168.241.1. DNS is the dongle's dns_forwarder
 * (bypasses NAPT UDP-timeout problems -- proven on the SLIP path).
 */

#include "ecm.h"

#include <string.h>

#include "esp_err.h"
#include "esp_mac.h"

#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/ip4_addr.h"
#include "lwip/tcpip.h"
#include "lwip/etharp.h"
#include "lwip/lwip_napt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "tusb.h"
#include "class/net/net_device.h"
#include "device/usbd_pvt.h"   /* usbd_defer_func -- TX serialization */

#include "dhserver.h"
#include "dns_forwarder.h"
#include "disk.h"    /* disk_logf */
#include "modem.h"   /* modem_session_busy -- heap truce pacing */

#define ECM_IP_A 192
#define ECM_IP_B 168
#define ECM_IP_C 241

/* TX pump: lwIP's linkoutput must NEVER block the tcpip thread (measured
 * 2026-06-12: a bounded in-line wait throttled the bridge to ~9 KB/s --
 * every TCP window burst overran the one-in-flight USB frame, the drops put
 * TCP into RTO crawl, and the stalled tcpip thread backed WiFi RX up until
 * min_free grazed 3.6 K). Instead linkoutput enqueues a pbuf reference and
 * returns immediately; a small dedicated task absorbs the USB drain latency.
 * Queue depth bounds pinned pbufs (WiFi RX pbufs are ~1.6 K each, and the
 * dynamic pool is capped at 12, so 8 here can never pin more than the pool
 * allows); overflow drops at the queue -- cheap, and TCP paces to the link. */
#define ECM_TXQ_DEPTH        8
/* Per-frame drain bound inside the pump: a dead/unplugged host must not
 * wedge the pump holding a pbuf forever. */
#define ECM_TX_DRAIN_MS      200

/* The host adapter's MAC (served via the iMACAddress string descriptor).
 * Declared extern by TinyUSB's net driver; we own the definition. */
uint8_t tud_network_mac_address[6];

static struct netif s_ecm_nif;
static bool         s_started = false;
static QueueHandle_t s_txq    = NULL;

static volatile uint32_t s_tx_drops      = 0;
static volatile uint32_t s_rx_pbuf_fails = 0;
static volatile uint32_t s_rx_frames     = 0;
static volatile uint32_t s_tx_frames     = 0;
static volatile uint32_t s_rx_bytes      = 0;
static volatile uint32_t s_tx_bytes      = 0;

uint32_t ecm_stat_tx_drops(void)      { return s_tx_drops; }
uint32_t ecm_stat_rx_pbuf_fails(void) { return s_rx_pbuf_fails; }
uint32_t ecm_stat_rx_frames(void)     { return s_rx_frames; }
uint32_t ecm_stat_tx_frames(void)     { return s_tx_frames; }
uint32_t ecm_stat_rx_bytes(void)      { return s_rx_bytes; }
uint32_t ecm_stat_tx_bytes(void)      { return s_tx_bytes; }

void ecm_mac_init(void) {
    /* Locally-administered variant of the STA MAC: same OUI-ish bytes so
     * it's recognizably "this dongle", local bit set so it can never
     * collide with the factory-assigned space. */
    esp_read_mac(tud_network_mac_address, ESP_MAC_WIFI_STA);
    tud_network_mac_address[0] |= 0x02;   /* locally administered */
    tud_network_mac_address[0] &= (uint8_t)~0x01; /* unicast */
}

/* ---- TinyUSB net-driver callbacks (TinyUSB task, CPU1) ---- */

/** @brief One Ethernet frame arrived from the host. Copy to a pbuf and
 *  post to the tcpip thread. Returning false makes the driver re-arm the
 *  OUT endpoint itself (frame dropped). */
bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
    if (size == 0) return true;
    struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);
    if (!p) {
        s_rx_pbuf_fails++;
        return false;          /* driver renews on our behalf */
    }
    pbuf_take(p, src, size);
    /* netif.input == tcpip_input: thread-safe mailbox post; dispatches to
     * ethernet_input on the tcpip thread (netif has NETIF_FLAG_ETHARP). */
    if (s_ecm_nif.input(p, &s_ecm_nif) != ERR_OK) {
        pbuf_free(p);          /* mbox full -> drop, TCP retransmits */
    } else {
        s_rx_frames++;
        s_rx_bytes += size;
    }
    tud_network_recv_renew();  /* re-arm for the next frame */
    return true;
}

/** @brief Driver asks us to serialize the frame we handed tud_network_xmit. */
uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
    struct pbuf *p = (struct pbuf *)ref;
    (void)arg;
    return pbuf_copy_partial(p, dst, p->tot_len, 0);
}

/** @brief Reset network state (driver hook; RNDIS-initiated -- nothing to do
 *  for our ECM-only descriptor, but the symbol is required at link). */
void tud_network_init_cb(void) {
}

/* ---- lwIP netif glue ---- */

/* The actual usbd_edpt_xfer submission MUST run in the USB task: calling
 * tud_network_xmit from another task races the dcd event processing on the
 * same endpoint (the ECM driver doesn't claim like CDC does) -- measured
 * 2026-06-12 as an IN endpoint stuck busy=1 forever after a few thousand
 * frames at 416 KB/s. usbd_defer_func serializes us into the USB task.
 * Handoff is race-free: can_xmit is SET only by the USB task (completion)
 * and CLEARED only by our own xmit, so once the pump observes it true it
 * stays true until the deferred call consumes it. */
static SemaphoreHandle_t s_tx_done = NULL;

static void ecm_xmit_in_usbtask(void *param) {
    tud_network_xmit((struct pbuf *)param, 0);  /* serializes via xmit_cb NOW */
    xSemaphoreGive(s_tx_done);
}

/** @brief TX pump task: drain the queue into the USB driver, absorbing the
 *  one-frame-in-flight USB latency that linkoutput must not block on. */
static void ecm_tx_task(void *arg) {
    (void)arg;
    struct pbuf *p;
    for (;;) {
        if (xQueueReceive(s_txq, &p, portMAX_DELAY) != pdTRUE) continue;
        /* Heap truce with the modem's secure sessions: a libssh2/TLS session
         * spike plus full-rate bridging stacks to a ~zero heap floor (measured
         * 2026-06-12: concurrent SFTP get + 449 KB/s bridge -> min_free 104 B).
         * While a dial/session worker is active, pace the pump to ~60 KB/s --
         * TCP self-paces to the slower link and in-flight pbufs shrink. The
         * real DOS host never exceeds this during a session anyway. */
        if (modem_session_busy()) vTaskDelay(pdMS_TO_TICKS(25));
        int waited = 0;
        while (!(tud_ready() && tud_network_can_xmit(p->tot_len))) {
            if (++waited > ECM_TX_DRAIN_MS) break;    /* host gone/stalled */
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (waited <= ECM_TX_DRAIN_MS) {
            usbd_defer_func(ecm_xmit_in_usbtask, p, false);
            if (xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
                /* Deferred call hasn't run (USB task starved/dying): do NOT
                 * free the pbuf it still references -- deliberate counted
                 * leak in preference to a use-after-free. */
                s_tx_drops++;
                continue;
            }
            s_tx_frames++;
            s_tx_bytes += p->tot_len;
        } else {
            s_tx_drops++;
        }
        pbuf_free(p);                  /* drop the reference linkoutput took */
    }
}

/** @brief linkoutput (tcpip thread): enqueue a reference and return -- the
 *  pump owns the USB latency. Overflow drops here, without blocking lwIP. */
static err_t ecm_linkoutput(struct netif *nif, struct pbuf *p) {
    (void)nif;
    if (!s_txq || !tud_ready()) return ERR_IF;  /* not enumerated / unplugged */
    /* Heap truce, intake half: while a secure session runs, also cap the
     * QUEUE to 2 frames -- each queued pbuf pins ~1.6 K of WiFi RX buffer,
     * and 8 of them (~12 K) was most of the remaining gap to the floor
     * (pacing alone: min_free 4.4 K; still under the 5 K guard). */
    if (modem_session_busy() && uxQueueMessagesWaiting(s_txq) >= 2) {
        s_tx_drops++;
        return ERR_OK;                 /* drop early; TCP paces */
    }
    pbuf_ref(p);
    if (xQueueSend(s_txq, &p, 0) != pdTRUE) {
        pbuf_free(p);                  /* queue full: genuine overload */
        s_tx_drops++;
    }
    return ERR_OK;
}

/** @brief netif init callback (runs inside netif_add). */
static err_t ecm_if_init(struct netif *nif) {
    nif->mtu        = 1500;            /* CFG_TUD_NET_MTU (1514) minus eth header */
    nif->hwaddr_len = 6;
    memcpy(nif->hwaddr, tud_network_mac_address, 6);
    nif->hwaddr[5] ^= 0x01;            /* both ends of a link need distinct MACs */
    nif->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
    nif->output     = etharp_output;
    nif->linkoutput = ecm_linkoutput;
    nif->name[0]    = 'u';
    nif->name[1]    = 'e';
    return ERR_OK;
}

/* ---- DHCP lease table: tiny -- one DOS host (plus slack for testing) ---- */
static dhcp_entry_t s_dhcp_entries[] = {
    { {0}, { PP_HTONL(LWIP_MAKEU32(ECM_IP_A, ECM_IP_B, ECM_IP_C, 2)) }, 24 * 3600 },
    { {0}, { PP_HTONL(LWIP_MAKEU32(ECM_IP_A, ECM_IP_B, ECM_IP_C, 3)) }, 24 * 3600 },
    { {0}, { PP_HTONL(LWIP_MAKEU32(ECM_IP_A, ECM_IP_B, ECM_IP_C, 4)) }, 24 * 3600 },
};

static dhcp_config_t s_dhcp_config = {
    .router    = { PP_HTONL(LWIP_MAKEU32(ECM_IP_A, ECM_IP_B, ECM_IP_C, 1)) },
    .port      = 67,
    .dns       = { PP_HTONL(LWIP_MAKEU32(ECM_IP_A, ECM_IP_B, ECM_IP_C, 1)) },
    .domain    = "dosongle",
    .num_entry = sizeof(s_dhcp_entries) / sizeof(s_dhcp_entries[0]),
    .entries   = s_dhcp_entries,
    .netif     = &s_ecm_nif,           /* NEVER serve the WiFi side */
};

esp_err_t ecm_start(void) {
    if (s_started) return ESP_ERR_INVALID_STATE;

    /* TX pump before the netif goes up. */
    s_txq = xQueueCreate(ECM_TXQ_DEPTH, sizeof(struct pbuf *));
    if (!s_txq) return ESP_ERR_NO_MEM;
    s_tx_done = xSemaphoreCreateBinary();
    if (!s_tx_done) { vQueueDelete(s_txq); s_txq = NULL; return ESP_ERR_NO_MEM; }
    if (xTaskCreatePinnedToCore(ecm_tx_task, "ecm_tx", 2560, NULL, 17,
                                NULL, 1) != pdPASS) {
        vQueueDelete(s_txq); s_txq = NULL;
        return ESP_ERR_NO_MEM;
    }

    ip4_addr_t ip, nm, gw;
    IP4_ADDR(&ip, ECM_IP_A, ECM_IP_B, ECM_IP_C, 1);
    IP4_ADDR(&nm, 255, 255, 255, 0);
    IP4_ADDR(&gw, ECM_IP_A, ECM_IP_B, ECM_IP_C, 1);

    LOCK_TCPIP_CORE();
    netif_add(&s_ecm_nif, &ip, &nm, &gw, NULL, ecm_if_init, tcpip_input);
    netif_set_up(&s_ecm_nif);
    netif_set_link_up(&s_ecm_nif);
    /* NAPT on the INTERNAL (ECM) netif: packets entering from the host get
     * their source rewritten when forwarded out the STA. Inverted vs typical
     * NAT APIs -- see slip.c's enable_napt + memory esp-idf-napt-api-inverted. */
    int napt_ok = ip_napt_enable_netif(&s_ecm_nif, 1);
    err_t dh = dhserv_init(&s_dhcp_config);
    UNLOCK_TCPIP_CORE();

    disk_logf("[ecm] up %d.%d.%d.1/24 napt=%s dhcp=%s mac=%02x%02x%02x%02x%02x%02x",
              ECM_IP_A, ECM_IP_B, ECM_IP_C,
              napt_ok == 1 ? "ok" : "FAILED",
              dh == ERR_OK ? "ok" : "FAILED",
              tud_network_mac_address[0], tud_network_mac_address[1],
              tud_network_mac_address[2], tud_network_mac_address[3],
              tud_network_mac_address[4], tud_network_mac_address[5]);

    /* DNS forwarder on the ECM netif IP (same engine the SLIP path uses;
     * DHCP hands 192.168.241.1 to the client as its nameserver). */
    dns_forwarder_init_ip((ECM_IP_A << 24) | (ECM_IP_B << 16) | (ECM_IP_C << 8) | 1U);

    s_started = true;
    return ESP_OK;
}
