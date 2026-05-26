#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// Status display on the T-Dongle S3's 0.96" ST7735 (160x80 landscape).
// All no-ops unless ENABLE_DISPLAY is defined.

void display_init();

// SLIP-router personality status. dl_bps/ul_bps are live throughput in
// bytes/sec from the DOS host's perspective (download = net->host).
void display_slip(bool wifi_up, IPAddress sta_ip, bool napt,
                  uint32_t dl_bps, uint32_t ul_bps);

// WiFi-modem personality status.
void display_modem(bool wifi_up, IPAddress sta_ip, bool online, const char *peer);
