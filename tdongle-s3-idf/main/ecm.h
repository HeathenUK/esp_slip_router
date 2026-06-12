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
/** @brief Frames received from the host (delivered into lwIP). */
uint32_t ecm_stat_rx_frames(void);
/** @brief Frames transmitted to the host. */
uint32_t ecm_stat_tx_frames(void);
/** @brief Bytes received from the host (upload direction). */
uint32_t ecm_stat_rx_bytes(void);
/** @brief Bytes transmitted to the host (download direction). */
uint32_t ecm_stat_tx_bytes(void);

#ifdef __cplusplus
}
#endif
