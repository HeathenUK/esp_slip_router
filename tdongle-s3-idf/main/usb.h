#pragma once
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the composite USB device (MSC + HID + CDC) and redirect
 * stdout to the CDC interface. Call once after networking is up so
 * HTTP-based diagnostics survive even if anything here fails. */
esp_err_t usb_start(void);

/* USB composite mode for this boot (NVS "usbnet", AT$USBNET=n + reboot):
 *   0 = normal:  CDC + MSC + HID keyboard
 *   1 = DOS net: CDC + MSC + ECM without notification EP (CHUSB tolerates;
 *                macOS will NOT publish the interface in this mode)
 *   2 = dev net: CDC + ECM with notification EP, no MSC (what macOS needs)
 * The DWC2's 5 IN-endpoint FIFO budget forces the either/or. */
uint8_t usb_net_mode(void);
/* True when this boot has the ECM function (mode 1 or 2). */
bool usb_net_enabled(void);

/* Soft USB re-enumeration: drop the D+ pullup (tud_disconnect), hold
 * long enough that the host fully tears down its device state, then
 * re-assert it (tud_connect). This is the remote equivalent of a
 * physical unplug/replug -- it makes the host (macOS) see a true
 * device removal + fresh insertion, which can clear a stuck
 * diskarbitration/IOKit state that a chip reset's abrupt sub-second
 * disconnect doesn't. Affects the whole composite (CDC drops + comes
 * back ~hold_ms later); intended as a deliberate recovery action.
 * Runs synchronously -- call from a worker task, not an ISR. */
void usb_soft_reconnect(unsigned hold_ms);

#ifdef __cplusplus
}
#endif
