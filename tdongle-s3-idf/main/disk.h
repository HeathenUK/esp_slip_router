/* disk.h -- MSC backing store. Mounts the ffat partition through the
 * wear-levelling layer, formats it on first boot with a recognisable
 * volume label, and serves SCSI READ/WRITE through a 4KB write-back
 * cache that batches small same-sector writes into single flash erases.
 *
 * Defines strong tud_msc_*_cb that override esp_tinyusb's stock
 * implementations -- those are weak-linked via tools/apply_iram_patches.sh.
 */
#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the WL-backed disk. Idempotent; safe to call once at the
 * top of usb_start() before tinyusb_driver_install. */
esp_err_t disk_init(void);

void disk_logf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif
