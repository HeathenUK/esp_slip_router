// dongle_disk: the FAT partition that backs the USB MSC dev-disk and is
// served over HTTP on the WiFi side. Coordination model: USB has the disk
// by default; HTTP requests briefly take ownership (eject from host, mount
// FATFS locally, do the op, unmount, present back) so neither side
// corrupts the FAT under the other.
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mount the FAT partition (formatting on first boot), bring USB MSC up
// alongside the existing CDC, start the HTTP server, and register mDNS.
// Safe to call after WiFi.begin(); HTTP/mDNS will activate when STA is up.
void dongle_disk_init(void);

// Status snapshot for AT$DISK?.
typedef struct {
    bool     mounted;          // FAT mounted on dongle side right now
    bool     msc_present;      // mediaPresent flag asserted to host
    uint32_t partition_bytes;  // total FAT partition size
    uint32_t used_bytes;       // bytes used in FAT (best effort)
    uint32_t http_ready;       // HTTP server listening
    const char *mdns_host;     // "dongle" (so "dongle.local")
} dongle_disk_status_t;

void dongle_disk_get_status(dongle_disk_status_t *out);

#ifdef __cplusplus
}
#endif
