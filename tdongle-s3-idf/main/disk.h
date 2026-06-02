/* disk.h -- MSC backing store. Mounts the ffat partition through the
 * wear-levelling layer, formats it on first boot with a recognisable
 * volume label, and serves SCSI READ/WRITE through a 4KB write-back
 * cache that batches small same-sector writes into single flash erases.
 *
 * Defines strong tud_msc_*_cb that override esp_tinyusb's stock
 * implementations -- those are weak-linked via tools/apply_iram_patches.sh.
 */
#pragma once
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the WL-backed disk. Idempotent; safe to call once at the
 * top of usb_start() before tinyusb_driver_install. */
esp_err_t disk_init(void);

void disk_logf(const char *fmt, ...);

/* Dump per-transaction counters + write-back cache stats as JSON.
 * Reads volatile uint32_ts that the MSC callbacks increment without
 * a lock; values may be slightly stale relative to a particular
 * callback but never torn. */
size_t disk_stats_json(char *out, size_t cap);

/* ---- ownership transfer for raw-FAT HTTP handlers (Phase 2) ----
 *
 * disk_take_for_firmware(): block MSC writes, flush dirty cache, so
 * the firmware can mount FATFS on the same WL handle without
 * conflicting with host writes. Returns ESP_OK on success;
 * ESP_ERR_INVALID_STATE if the disk isn't ready; ESP_ERR_TIMEOUT
 * (HTTP layer should return 429) if a recent MSC write means the
 * host is still actively writing.
 *
 * disk_release_to_usb(): drop ownership, invalidate the cache (so
 * MSC reads pick up FATFS-side changes from flash), set
 * UNIT_ATTENTION sense data so the next host TUR triggers a FAT
 * re-read. Idempotent.
 *
 * For read-only HTTP ops (GET /list, GET /fs/<name>, GET /lba) do
 * NOT call disk_take_for_firmware -- just go through wl_read /
 * the cache directly. Concurrent MSC writes can change the FAT
 * under FATFS, so reads use a lightweight path that doesn't mount
 * FATFS at all; see fat_*_via_wl in fat.c. */
#define DISK_RECENT_MSC_WRITE_GUARD_US 500000  /* 500 ms */

esp_err_t disk_take_for_firmware(void);
void      disk_release_to_usb(void);
bool      disk_dongle_owns(void);

/* True if the USB host currently appears to have the MSC volume
 * mounted (it polled Test Unit Ready or did any SCSI op within the
 * last few seconds). Device-side FAT access (HTTP /fs read, write,
 * list, delete) must refuse while this is true: a second FATFS mount
 * on the dongle side while the host's kernel also has the volume
 * mounted corrupts the FAT and wedges the HTTP worker on the
 * contended flash. The HTTP layer returns 409 Conflict so the caller
 * unmounts the volume first (the dosongle.sh helper does this). */
bool      disk_host_mounted(void);

/* True if the host is actively transferring data (READ10/WRITE10
 * within the last second), vs merely mounted-and-idle. Device-side
 * FAT *reads* gate on this (not disk_host_mounted) so a mounted-idle
 * DOS host at the prompt can still serve HTTP log pulls, while a host
 * mid-transfer (macOS fskit prefetch) is refused to avoid the
 * dual-mount wedge. */
bool      disk_host_active_io(void);

/* Software eject / mount. disk_eject() flushes the cache and reports
 * medium-not-present on the next Test Unit Ready, so the USB host
 * unmounts the volume (clears the dual-mount hazard without a manual
 * host-side eject). disk_mount() re-presents the medium + arms
 * UNIT_ATTENTION so the host remounts. disk_medium_present() reports
 * the current state. Exposed over HTTP as POST /eject and POST /mount. */
void      disk_eject(void);
void      disk_mount(void);
bool      disk_medium_present(void);

/* Reformat the volume to a clean FAT12 superfloppy at runtime (wipes
 * all files). Recovery for a corrupted FAT. Exposed as POST /format. */
esp_err_t disk_format(void);

/* Serialize a device-side FATFS op against the MSC SCSI path by holding
 * the shared I/O mutex across the whole mount+read/write. Eliminates
 * WL-layer contention with a storming host (the original /fs wedge) and
 * replaces the host-activity heuristic guard. Bounded acquire -> false
 * (HTTP 409) only on genuine timeout. Lock/unlock must be balanced. */
bool      disk_fatfs_lock(uint32_t timeout_ms);
void      disk_fatfs_unlock(void);

/* fat.c uses this to register the WL handle with FATFS' diskio
 * layer. Returns WL_INVALID_HANDLE before disk_init() completes. */
#include "wear_levelling.h"
wl_handle_t disk_get_wl_handle(void);

#ifdef __cplusplus
}
#endif
