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
