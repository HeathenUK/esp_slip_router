/* modem.h -- Hayes AT modem on the TinyUSB CDC interface.
 *
 * Phase 1c.0: just the scaffolding -- line parser, standard responses,
 * a small set of commands (ATE/V/Q/I/Z + AT$HELP + AT$WIFI?) so we
 * can verify the CDC pipeline + AT parser end-to-end before bringing
 * up the heavier pieces (TCP dial via lwIP, online-mode data pump,
 * escape sequence).
 *
 * Subsequent phases add:
 *   1c.1: AT$WIFI=<ssid>,<pw> (writes NVS + esp_wifi_connect)
 *   1c.2: ATD<host:port>, ATO, +++, ATH (TCP dial / online mode)
 *   1c.3: AT$DNS=, AT$PING=, AT$NETIF, AT$SCAN (network diagnostics)
 *   1c.4: AT$RESET, AT$OTASTART= (system commands)
 */
#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Register the modem on TinyUSB CDC interface 0 + initialise its
 * NVS-backed settings. Call after tusb_cdc_acm_init(). */
esp_err_t modem_init(void);

#ifdef __cplusplus
}
#endif
