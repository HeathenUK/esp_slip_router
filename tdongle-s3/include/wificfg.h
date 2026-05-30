#pragma once
#include <Arduino.h>

// WiFi credentials live in NVS (falling back to the config.h defaults when
// unset) and are shared by both personalities. Implemented in main.cpp (which
// owns WiFi + the Preferences handle); used by the modem's AT$ commands.

void wifi_load_and_begin();        // load NVS-or-default creds and start STA
void wifi_set_ssid(const char *s); // store SSID (RAM + NVS)
void wifi_set_pass(const char *p); // store password (RAM + NVS)
const char *wifi_ssid();           // currently configured SSID
bool wifi_has_pass();              // is a password set?
void wifi_reconnect();             // apply stored creds now
void wifi_apply_soon();            // (re)connect shortly — coalesces rapid set's

// AT&W -- persist the modem's runtime echo/verbose/telnet flags to NVS so they
// survive reboot. The next modem_begin()/setup() rehydrates them.
void wifi_save_modem_settings(bool echo, bool verbose, bool telnet);
void wifi_load_modem_settings(bool *echo, bool *verbose, bool *telnet);
