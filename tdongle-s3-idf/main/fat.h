/* fat.h -- FATFS-backed file ops for the raw-FAT HTTP path.
 *
 * Used by main.c's /list, /fs/<name>, /lba HTTP handlers. Writes
 * (PUT, DELETE) call disk_take_for_firmware() to block MSC writes
 * while FATFS mutates the FAT; reads do not, accepting the small
 * chance of inconsistency vs. concurrent DOS writes (HTTP layer
 * surfaces this via size mismatch -> client retries).
 *
 * Filenames are 8.3 only -- FAT12 root dir constraint. Caller is
 * responsible for upper-casing + length-checking BEFORE call.
 */
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Listing callback. Return false to stop iteration. Sizes in bytes;
 * is_dir true means a directory entry (we still report it but the
 * caller may want to filter). */
typedef bool (*fat_list_cb_t)(const char *name83, uint32_t size, bool is_dir, void *ctx);

/* Read/write streaming callbacks. The "in" form sends bytes to us
 * (used by PUT body recv); "out" form gives us bytes (used by GET
 * response chunks). Return ESP_OK to continue, anything else aborts.
 */
typedef esp_err_t (*fat_in_cb_t)(void *out, size_t cap, size_t *got, void *ctx);
typedef esp_err_t (*fat_out_cb_t)(const void *in, size_t n, void *ctx);

/* All four take a freshly-mounted FATFS session: each call mounts,
 * does its work, unmounts. Write-path ones call
 * disk_take_for_firmware() / disk_release_to_usb() internally so MSC
 * stays consistent.
 *
 * Returns:
 *   ESP_OK on success.
 *   ESP_ERR_NOT_FOUND if a file doesn't exist.
 *   ESP_ERR_TIMEOUT (from take_for_firmware) if DOS is actively
 *     writing -- HTTP should reply 429.
 *   ESP_ERR_INVALID_ARG for malformed name.
 *   ESP_FAIL for everything else. */

esp_err_t fat_list  (fat_list_cb_t cb, void *ctx);
esp_err_t fat_read  (const char *name83, fat_out_cb_t out, void *ctx);
esp_err_t fat_write (const char *name83, fat_in_cb_t in, size_t total, void *ctx);
esp_err_t fat_delete(const char *name83);

/* Validate + canonicalise a URI tail into an 8.3 uppercase name.
 * Returns true if the result fits "NAME[.EXT]" within 8.3, false
 * otherwise. Strips a leading "/fs/" if present. */
bool fat_uri_to_name83(const char *uri, char out_name[13]);

#ifdef __cplusplus
}
#endif
