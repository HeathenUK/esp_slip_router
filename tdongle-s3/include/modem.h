#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// WiFi-modem personality: a WiFi232-style Hayes/AT engine that bridges
// `ATDT host:port` to a TCP socket over the same USB-CDC link the SLIP router
// uses. Driverless on the host side (any terminal program over the COM port).

void modem_begin();          // one-time init
void modem_enter();          // entering modem personality (reset to command mode)
void modem_leave();          // leaving it (drop any open socket)
void modem_poll();           // pump USB<->TCP / parse AT; call each loop in modem mode

bool modem_is_online();      // true while a call is connected and in data mode
const char *modem_peer();    // "host:port" of the current/last call, or ""

// AT$STATS / AT&V need access to the SLIP byte/pkt counters that live in main.cpp.
struct slip_stats { uint32_t pkts_in, bytes_in, pkts_out, bytes_out; };
void modem_get_slip_stats(struct slip_stats *s);
void modem_clear_slip_stats();

// AT$MODE host-side personality switch. NULL `want` = query only. Returns the
// resulting mode name ("SLIP"/"MODEM") or NULL on bad arg.
const char *modem_set_personality(const char *want);
