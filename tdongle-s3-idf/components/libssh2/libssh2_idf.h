/* IDF consumer shim for the vendored libssh2.
 *
 * The vendored headers (from Zimodem's Arduino-ESP32 port) gate their entire
 * bodies on `#if defined(ESP32)`, which Arduino-ESP32 defines but pure ESP-IDF
 * does not. Define it here so IDF translation units can use the libssh2 API by
 * including THIS header instead of <libssh2.h> directly. See CREDITS.md.
 */
#pragma once
#ifndef ESP32
#define ESP32 1
#endif
#include "libssh2.h"
