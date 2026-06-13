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
#include "esp_system.h"   /* esp_get_free_heap_size / minimum -- stall logging */
#include "esp_timer.h"    /* 1 Hz bridge heartbeat -> /disk-log */

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
/* Consecutive FULL drain timeouts (the host drained ZERO frames for that long)
 * we treat as a stuck IN endpoint rather than slow-host backpressure. An alive
 * CH375 host drains a 1500 B frame in ~15 ms even when TCP-window-limited, so
 * ECM_STUCK_FRAMES * ECM_TX_DRAIN_MS = ~1 s of NOTHING draining is the
 * can_xmit-stuck wedge (lost IN completion / failed submit), not pacing. We
 * can't read AT$STATS while the DOS host owns the CDC port, so log it to the
 * RAM ring (GET /disk-log over WiFi) at onset and heartbeat while it persists. */
#define ECM_STUCK_FRAMES     5
#define ECM_STUCK_HEARTBEAT  50    /* re-log every N further drains (~10 s) */

/* The host adapter's MAC (served via the iMACAddress string descriptor).
 * Declared extern by TinyUSB's net driver; we own the definition. */
uint8_t tud_network_mac_address[6];

static struct netif s_ecm_nif;
static bool         s_started = false;
static QueueHandle_t s_txq    = NULL;
static esp_timer_handle_t s_hb_timer = NULL;   /* 1 Hz bridge heartbeat */

/* NAPT-layer diagnostics (patched into IDF lwip ip4_napt.c): TCP mappings
 * garbage-collected, and inbound TCP packets dropped for want of a mapping
 * (black-holed server replies after an eviction). These sit UPSTREAM of every
 * ecm.c counter -- the missing-link for the "both directions flat" freeze. */
extern volatile uint32_t g_napt_tcp_evict;
extern volatile uint32_t g_napt_recv_nomatch;

static volatile uint32_t s_dbg_drain_ms  = 0;   /* debug: simulate a slow host */
static volatile bool     s_flood_active  = false;

static volatile uint32_t s_tx_drops      = 0;
static volatile uint32_t s_rx_pbuf_fails = 0;
static volatile uint32_t s_rx_mbox_drops = 0;   /* host->stack drops: tcpip mbox full */
static volatile uint32_t s_tx_drops_q    = 0;   /* TX dropped at linkoutput: queue full / session cap */
static volatile uint32_t s_tx_drops_pump = 0;   /* TX dropped in pump: host didn't drain in ECM_TX_DRAIN_MS */
static volatile uint32_t s_rx_frames     = 0;
static volatile uint32_t s_tx_frames     = 0;
static volatile uint32_t s_rx_bytes      = 0;
static volatile uint32_t s_tx_bytes      = 0;

uint32_t ecm_stat_tx_drops(void)      { return s_tx_drops; }
uint32_t ecm_stat_rx_pbuf_fails(void) { return s_rx_pbuf_fails; }
uint32_t ecm_stat_rx_mbox_drops(void) { return s_rx_mbox_drops; }
uint32_t ecm_stat_tx_drops_q(void)    { return s_tx_drops_q; }
uint32_t ecm_stat_tx_drops_pump(void) { return s_tx_drops_pump; }
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
        s_rx_mbox_drops++;     /* uplink/ACK loss suspect for the freeze */
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
    struct pbuf *p = (struct pbuf *)param;
    tud_network_xmit(p, 0);  /* serializes via xmit_cb NOW */
    pbuf_free(p);            /* the deferred call OWNS this reference and frees
                              * it -- never the pump. This is what makes a stale
                              * give (from a previously timed-out submission)
                              * harmless: the pump can wake early but it never
                              * frees a pbuf the USB task still holds. */
    xSemaphoreGive(s_tx_done);
}

/** @brief TX pump task: drain the queue into the USB driver, absorbing the
 *  one-frame-in-flight USB latency that linkoutput must not block on. */
static void ecm_tx_task(void *arg) {
    (void)arg;
    struct pbuf *p;
    uint32_t s_stuck_run = 0;          /* consecutive full-drain timeouts */
    for (;;) {
        if (xQueueReceive(s_txq, &p, portMAX_DELAY) != pdTRUE) continue;
        /* Heap truce with the modem's secure sessions: a libssh2/TLS session
         * spike plus full-rate bridging stacks to a ~zero heap floor (measured
         * 2026-06-12: concurrent SFTP get + 449 KB/s bridge -> min_free 104 B).
         * While a dial/session worker is active, pace the pump to ~60 KB/s --
         * TCP self-paces to the slower link and in-flight pbufs shrink. The
         * real DOS host never exceeds this during a session anyway. */
        if (modem_session_busy()) vTaskDelay(pdMS_TO_TICKS(25));
        /* Debug knob (AT$NETSLOW): delay each frame to mimic a slow-draining
         * host (the CH375's ~100 KB/s poll) so the DOS wedge can be recreated
         * on a fast host. 0 = off (the normal path). */
        if (s_dbg_drain_ms) vTaskDelay(pdMS_TO_TICKS(s_dbg_drain_ms));
        int waited = 0;
        while (!(tud_ready() && tud_network_can_xmit(p->tot_len))) {
            if (++waited > ECM_TX_DRAIN_MS) break;    /* host gone/stalled */
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (waited <= ECM_TX_DRAIN_MS) {
            s_stuck_run = 0;               /* the endpoint drained: not wedged */
            s_tx_frames++;
            s_tx_bytes += p->tot_len;
            /* Hand the pbuf to the USB task, which owns and frees it. The pump
             * must NOT free it (use-after-free), nor treat the semaphore as
             * tracking THIS frame -- a give left from a prior timed-out submit
             * can wake us early, but since the deferred call owns every pbuf an
             * early wake only loosens serialization (the can_xmit gate above
             * re-tightens it next iteration), it never corrupts memory. */
            usbd_defer_func(ecm_xmit_in_usbtask, p, false);
            xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(1000));
            continue;                      /* p is owned by the deferred call */
        }
        /* Full ECM_TX_DRAIN_MS timeout: the host drained NO frame in that long. */
        s_tx_drops++;
        s_tx_drops_pump++;             /* host poll-gap exceeded the drain bound */
        pbuf_free(p);                  /* never deferred -- pump still owns it */
        /* A long unbroken run of these is the can_xmit-stuck wedge: a lost IN
         * completion or a failed usbd_edpt_xfer leaves can_xmit false with
         * nothing actually in flight, and the pump has no way out. Surface it on
         * /disk-log (readable over WiFi while DOS owns CDC) at onset, then
         * heartbeat so its (non-)recovery is visible too. rdy/cx distinguish an
         * unplugged host (rdy=0) from a stuck endpoint (rdy=1,cx=0). */
        if (++s_stuck_run == ECM_STUCK_FRAMES ||
            (s_stuck_run > ECM_STUCK_FRAMES &&
             (s_stuck_run - ECM_STUCK_FRAMES) % ECM_STUCK_HEARTBEAT == 0)) {
            /* Try to unstick: only acts if the IN endpoint is idle (a lost
             * completion left can_xmit false with nothing in flight). alt=0/ep=00
             * means the host has the data interface INACTIVE -- not recoverable
             * here, the frames are correctly dropped. busy=1 means a transfer is
             * genuinely outstanding (host not draining) -- normal backpressure. */
            bool rec = tud_network_xmit_recover();
            disk_logf("[ecm] TX STALL run=%u rdy=%d cx=%d alt=%d ep=%02x busy=%d "
                      "rec=%d tx=%u drop=%u qd=%u heap=%u",
                      (unsigned)s_stuck_run, (int)tud_ready(),
                      (int)tud_network_can_xmit(64), (int)tud_network_data_alt(),
                      tud_network_ep_in(), (int)tud_network_ep_in_busy(),
                      (int)rec, (unsigned)s_tx_frames, (unsigned)s_tx_drops,
                      (unsigned)uxQueueMessagesWaiting(s_txq),
                      (unsigned)esp_get_free_heap_size());
            if (rec) s_stuck_run = 0;       /* recovered: re-arm the detector */
        }
    }
}

/** @brief 1 Hz bridge heartbeat -> /disk-log. The pump's TX-STALL line only
 *  fires when the device HAS frames it cannot send (topology A: the ECM-TX
 *  can_xmit-stuck wedge). The other freeze topology -- the host->server uplink
 *  (TCP ACKs) failing so the server stops sending -- leaves the pump idle on an
 *  empty queue and is invisible to it. This samples BOTH directions so a stall
 *  shows WHICH leg stopped: at the freeze the last line reads either
 *  qd=8,cx=0 (A: device wedged with frames waiting) or qd=0,cx=1 with tx AND rx
 *  flat (B: device fed everything, nothing more arrived to forward). Logs only
 *  while traffic moves, plus one trailing snapshot when it stops, so the
 *  24-line ring isn't flooded at idle. */
static void ecm_hb_cb(void *arg) {
    (void)arg;
    static uint32_t l_rx = 0, l_tx = 0, l_ev = 0, l_nm = 0, l_dr = 0;
    static bool was_active = false;
    uint32_t rx = s_rx_frames, tx = s_tx_frames;
    uint32_t rxd = rx - l_rx, txd = tx - l_tx;
    uint32_t ev = g_napt_tcp_evict, nm = g_napt_recv_nomatch;
    uint32_t dr = s_tx_drops_q + s_tx_drops_pump;
    /* Fire on DATA progress OR on a NAPT/drop counter moving. Progress alone
     * would silence the heartbeat exactly during a freeze -- but a NAPT
     * eviction (ev++) and the black-holed server retransmits that follow (nm++)
     * happen DURING that silence, and they are the whole point. So a freeze
     * that NAPT is killing logs "IDLE ... napt=e1/nm<climbing>" every second
     * (cumulative, so e1 shows in every retained line); a TRUE upstream
     * silence (nothing arriving) logs one IDLE snapshot then goes quiet. */
    bool data = rxd || txd;
    bool moved = data || (ev != l_ev) || (nm != l_nm) || (dr != l_dr);
    bool active = data;            /* the IDLE tag tracks DATA, not counters */
    if (moved || was_active) {
        disk_logf("[ecm] hb%s rx=%u+%u tx=%u+%u qd=%u cx=%d txdr=q%u/p%u rxdr=%u napt=e%u/nm%u",
                  active ? "" : " IDLE",
                  (unsigned)rx, (unsigned)rxd, (unsigned)tx, (unsigned)txd,
                  (unsigned)(s_txq ? uxQueueMessagesWaiting(s_txq) : 0),
                  (int)tud_network_can_xmit(64),
                  (unsigned)s_tx_drops_q, (unsigned)s_tx_drops_pump,
                  (unsigned)(s_rx_pbuf_fails + s_rx_mbox_drops),
                  (unsigned)g_napt_tcp_evict, (unsigned)g_napt_recv_nomatch);
    }
    l_rx = rx; l_tx = tx; l_ev = ev; l_nm = nm; l_dr = dr;
    was_active = moved;
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
        s_tx_drops_q++;
        return ERR_OK;                 /* drop early; TCP paces */
    }
    pbuf_ref(p);
    if (xQueueSend(s_txq, &p, 0) != pdTRUE) {
        pbuf_free(p);                  /* queue full: genuine overload */
        s_tx_drops++;
        s_tx_drops_q++;                /* burst outran the 8-deep queue */
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

int ecm_test_emit(uint16_t size, uint16_t count, uint32_t *drops_out) {
    if (!s_started || size < 14 || size > 1514 || count == 0 || count > 64)
        return -1;
    uint32_t drops0 = s_tx_drops;
    for (uint16_t n = 0; n < count; n++) {
        struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_RAM);
        if (!p) return -1;
        uint8_t *d = (uint8_t *)p->payload;
        memset(d, 0xFF, 6);                          /* dst: broadcast */
        memcpy(d + 6, s_ecm_nif.hwaddr, 6);          /* src: bridge MAC */
        d[12] = 0x88; d[13] = 0xB5;                  /* ethertype: IEEE local-experimental */
        for (uint16_t i = 14; i < size; i++)         /* payload: counting pattern + frame idx */
            d[i] = (uint8_t)(i + n);
        ecm_linkoutput(&s_ecm_nif, p);               /* refs the pbuf like real traffic */
        pbuf_free(p);
        vTaskDelay(pdMS_TO_TICKS(5));                /* paced: a chip test, not a flood */
    }
    if (drops_out) *drops_out = s_tx_drops - drops0;
    return 0;
}

void ecm_dbg_set_drain_ms(uint32_t ms) { s_dbg_drain_ms = ms; }

/** @brief Flood the ECM TX path at full rate for a bounded time -- a device-side
 *  repro of the download wedge with NO host client and NO route changes. Injects
 *  max-size unicast frames (to the host NIC, ethertype 0x88B5 -- the host drops
 *  them, never routes them) straight into ecm_linkoutput, exactly the path real
 *  download traffic takes from NAPT. Self-paces to the pump's drain rate via the
 *  queue-full check, so it never busy-spins or starves the pump/USB tasks.
 *  Runs in its own task so the AT/CDC console stays responsive (read AT$STATS /
 *  GET /disk-log live during the flood). */
static void ecm_flood_task(void *arg) {
    uint32_t ms = (uint32_t)(uintptr_t)arg;
    uint32_t f0 = s_tx_frames, d0 = s_tx_drops, injected = 0;
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    disk_logf("[ecm] FLOOD start %ums slow=%ums", (unsigned)ms,
              (unsigned)s_dbg_drain_ms);
    while (esp_timer_get_time() < end) {
        struct pbuf *p = pbuf_alloc(PBUF_RAW, 1514, PBUF_RAM);
        if (!p) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
        uint8_t *d = (uint8_t *)p->payload;
        memcpy(d, tud_network_mac_address, 6);       /* dst: the host NIC */
        memcpy(d + 6, s_ecm_nif.hwaddr, 6);          /* src: bridge MAC */
        d[12] = 0x88; d[13] = 0xB5;                  /* ethertype: local-experimental */
        ecm_linkoutput(&s_ecm_nif, p);               /* refs + queues like real TX */
        pbuf_free(p);                                /* drop our own reference */
        injected++;
        if (uxQueueMessagesWaiting(s_txq) >= ECM_TXQ_DEPTH)
            vTaskDelay(1);                           /* self-pace to the drain rate */
    }
    disk_logf("[ecm] FLOOD done inj=%u sent=%u drops=%u cx=%d",
              (unsigned)injected, (unsigned)(s_tx_frames - f0),
              (unsigned)(s_tx_drops - d0), (int)tud_network_can_xmit(64));
    s_flood_active = false;
    vTaskDelete(NULL);
}

int ecm_test_flood(uint32_t ms) {
    if (!s_started || ms == 0 || ms > 30000 || s_flood_active) return -1;
    s_flood_active = true;
    if (xTaskCreatePinnedToCore(ecm_flood_task, "ecm_flood", 2560,
                                (void *)(uintptr_t)ms, 10, NULL, 1) != pdPASS) {
        s_flood_active = false;
        return -1;
    }
    return 0;
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

    /* Bridge heartbeat: samples both directions to /disk-log (non-fatal if it
     * can't start -- pure diagnostics, no effect on the data path). */
    const esp_timer_create_args_t hb_args = {
        .callback        = ecm_hb_cb,
        .name            = "ecm_hb",
        .dispatch_method = ESP_TIMER_TASK,
    };
    if (esp_timer_create(&hb_args, &s_hb_timer) == ESP_OK)
        esp_timer_start_periodic(s_hb_timer, 1000000);   /* 1 s */

    s_started = true;
    return ESP_OK;
}
