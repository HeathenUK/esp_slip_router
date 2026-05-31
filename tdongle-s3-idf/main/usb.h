#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the composite USB device (MSC + HID + CDC) and redirect
 * stdout to the CDC interface. Call once after networking is up so
 * HTTP-based diagnostics survive even if anything here fails. */
esp_err_t usb_start(void);

#ifdef __cplusplus
}
#endif
