// dongle_disk: the FAT partition that backs the USB MSC dev-disk and is
// served over HTTP on the WiFi side. Coordination model: exactly one side
// owns write access at a time. USB MSC owns the disk by default; explicit
// commands switch ownership to the device for raw FAT operations, then
// switch it back to USB MSC.
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bring the FAT partition up as a raw USB MSC LUN alongside the existing
// CDC, start the HTTP server, and register mDNS.
// Safe to call after WiFi.begin(); HTTP/mDNS will activate when STA is up.
void dongle_disk_init(void);

typedef enum {
    DONGLE_DISK_OWNER_USB = 0,
    DONGLE_DISK_OWNER_DEVICE = 1,
} dongle_disk_owner_t;

// Status snapshot for AT$DISK?.
typedef struct {
    bool     mounted;          // device currently owns the disk
    bool     msc_present;      // mediaPresent flag asserted to host
    bool     msc_writable;     // host sees MSC as writable
    dongle_disk_owner_t owner;
    uint32_t partition_bytes;  // total FAT partition size
    uint32_t used_bytes;       // bytes used in FAT (best effort)
    uint32_t http_ready;       // HTTP server listening
    const char *mdns_host;     // "dosongle" (so "dosongle.local")
} dongle_disk_status_t;

bool dongle_disk_set_owner(dongle_disk_owner_t owner);
dongle_disk_owner_t dongle_disk_get_owner(void);
const char *dongle_disk_owner_name(dongle_disk_owner_t owner);
bool dongle_disk_format(void);
void dongle_disk_get_status(dongle_disk_status_t *out);

#ifdef __cplusplus
}
#endif
