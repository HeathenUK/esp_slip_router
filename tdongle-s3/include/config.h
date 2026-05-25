#pragma once
//
// Compile-time configuration for the T-Dongle S3 SLIP router.
// Anything here can be overridden from platformio.ini build_flags.
//

// ---- WiFi (STA) credentials ----
// Normally set at runtime via the modem's AT$ commands (AT$SSID=, AT$PASS=) and
// stored in NVS. These compile-time values are only the fallback when NVS is
// empty; leave them blank so no credentials live in the source tree.
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif

// ---- SLIP-side IP addressing ----
// The dongle's address on the serial link; the host (DOS box) is SLIP_PEER and
// uses SLIP_LOCAL as its default gateway. /24 keeps it simple.
#ifndef SLIP_LOCAL_A
#define SLIP_LOCAL_A 192
#define SLIP_LOCAL_B 168
#define SLIP_LOCAL_C 240
#define SLIP_LOCAL_D 1
#endif
#ifndef SLIP_PEER_A
#define SLIP_PEER_A 192
#define SLIP_PEER_B 168
#define SLIP_PEER_C 240
#define SLIP_PEER_D 2
#endif

// ---- SLIP MTU ----
// Classic SLIP/dialup default is 1006; 1500 matches the original esp_slip_router.
#ifndef SLIP_MTU
#define SLIP_MTU 1500
#endif
