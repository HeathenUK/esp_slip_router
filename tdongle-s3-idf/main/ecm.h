/**
 * @file ecm.h
 * @brief CDC-ECM network function: lwIP netif + NAPT bridge + DHCP server.
 *
 * The device half of the "WiFi for DOS" design (ECM-BRIDGE-PLAN-2026-06-10.md):
 * the host plugs in and sees a USB Ethernet adapter; frames are bridged to the
 * WiFi STA uplink through the same lwIP/NAPT machinery the SLIP path used.
 * Whether the ECM function exists at all is a boot-time choice (NVS "usbnet",
 * AT$USBNET=1) -- see usb.c for the endpoint-budget reasoning.
 *
 * Netif glue modeled on TinyUSB's examples/device/net_lwip_webserver (MIT,
 * Peter Lawrence) -- adapted from its bare-metal NO_SYS loop to ESP-IDF's
 * full-OS lwIP (tcpip_input + core locking).
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Derive the USB-side MAC addresses from the efuse base MAC.
 *
 * Must run BEFORE the USB stack enumerates (usb_start calls it): the host
 * reads the MAC out of the iMACAddress string descriptor. The host NIC gets
 * the locally-administered variant of the STA MAC; the bridge netif uses the
 * same with the last byte flipped (the two ends of a link must differ).
 */
void ecm_mac_init(void);

/**
 * @brief Bring up the ECM bridge: netif + NAPT + DHCP server + DNS forwarder.
 *
 * Call once after wifi_start() (the STA netif must exist for NAPT to route
 * through) and only in NET mode. The netif is up immediately; DHCP serves
 * 192.168.241.2+ with router/DNS pointing at the bridge (192.168.241.1).
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if already started.
 */
esp_err_t ecm_start(void);

/** @brief Frames dropped on the USB TX path (host slow / not polling). */
uint32_t ecm_stat_tx_drops(void);
/** @brief RX frames dropped for want of a pbuf. */
uint32_t ecm_stat_rx_pbuf_fails(void);
/** @brief RX frames dropped because the tcpip mailbox was full (uplink/ACK loss). */
uint32_t ecm_stat_rx_mbox_drops(void);
/** @brief TX drops at linkoutput: queue full / secure-session cap (burst overrun). */
uint32_t ecm_stat_tx_drops_q(void);
/** @brief TX drops in the pump: host didn't drain within ECM_TX_DRAIN_MS (poll-gap). */
uint32_t ecm_stat_tx_drops_pump(void);
/** @brief Frames received from the host (delivered into lwIP). */
uint32_t ecm_stat_rx_frames(void);
/** @brief Frames transmitted to the host. */
uint32_t ecm_stat_tx_frames(void);
/** @brief Bytes received from the host (upload direction). */
uint32_t ecm_stat_rx_bytes(void);
/** @brief Bytes transmitted to the host (download direction). */
uint32_t ecm_stat_tx_bytes(void);

/**
 * @brief Emit raw Ethernet broadcast test frames of an EXACT size (AT$NETTEST).
 *
 * For the CHUSB-side E0 chip test: frames whose total size is an exact
 * multiple of 64 end in a ZLP on the wire -- the CH375's behavior there is
 * the host plan's "one true unknown". dst FF:..:FF, src = bridge MAC,
 * ethertype 0x88B5 (IEEE local-experimental), payload = counting pattern
 * offset by the frame index (integrity-checkable on the DOS side).
 * @param size  Total frame length, 14..1514 (e.g. 1472 = 23*64 -> ZLP).
 * @param count Frames to send (1..64), paced 5 ms apart.
 * @param drops_out Optional: TX drops incurred during the burst.
 * @return 0 on success, -1 on bad args / bridge not started / no memory.
 */
int ecm_test_emit(uint16_t size, uint16_t count, uint32_t *drops_out);

/**
 * @brief Flood the ECM TX path at full rate for @p ms (1..30000), device-side
 *  (AT$NETFLOOD). No host client or route change needed -- injects max-size
 *  frames straight into the bridge's TX pump, the exact path a download takes,
 *  to reproduce the freeze. Runs in a task; watch AT$STATS / GET /disk-log.
 * @return 0 started, -1 on bad args / not in NET mode / a flood already running.
 */
int ecm_test_flood(uint32_t ms);

/**
 * @brief Debug: delay the TX pump @p ms per frame to mimic a slow-draining host
 *  (AT$NETSLOW; 0 = off). Pair with ecm_test_flood to recreate the ~100 KB/s
 *  CH375 drain condition on a fast host.
 */
void ecm_dbg_set_drain_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
