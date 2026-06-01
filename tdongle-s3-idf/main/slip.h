/* slip.h -- RFC 1055 SLIP framing on the CDC interface + lwIP
 * point-to-point netif + NAPT to the WiFi STA.
 *
 * Owns the LinkMode global. When MODE_MODEM, the AT modem in modem.c
 * services CDC RX/TX; when MODE_SLIP, modem.c routes RX bytes here
 * via slip_feed() and slip's lwIP `output` callback emits TX.
 *
 * The mode is persisted in the "slip-router" NVS namespace under key
 * "mode" (uint8_t, 0=MODEM, 1=SLIP). Default on first boot is MODEM
 * to match the old firmware.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODE_MODEM = 0,
    MODE_SLIP  = 1,
} LinkMode;

/* Bring up the lwIP SLIP netif (admin-down initially) + enable NAPT
 * on it. Restores mode from NVS and applies it. Call after WiFi STA
 * is initialized (we need the STA netif present for NAPT to route
 * to). */
esp_err_t slip_init(void);

LinkMode  slip_get_mode(void);
esp_err_t slip_set_mode(LinkMode mode);   /* persists to NVS + applies */

/* Feed CDC RX bytes to the SLIP decoder. Called from modem.c's CDC
 * RX callback when slip_get_mode() == MODE_SLIP. */
void slip_feed(const uint8_t *buf, size_t n);

/* Stats accessors for /status. */
uint32_t slip_stat_pkts_to_host(void);
uint32_t slip_stat_pkts_from_host(void);
uint32_t slip_stat_bytes_to_host(void);
uint32_t slip_stat_bytes_from_host(void);
/* pbuf pool exhaustion on RX -- non-zero means SLIP packets are
 * being dropped before reaching lwIP. */
uint32_t slip_stat_pbuf_fails(void);
/* tx_buf overrun on SLIP encode -- should be 0 in normal use. */
uint32_t slip_stat_tx_truncs(void);
void     slip_stats_clear(void);

#ifdef __cplusplus
}
#endif
