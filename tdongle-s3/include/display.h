#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// Status display on the T-Dongle S3's 0.96" TFT (160x80 in landscape).
// All no-ops unless ENABLE_DISPLAY is defined.

void display_init();

// Called periodically; only redraws lines that actually changed (no flicker).
void display_status(bool wifi_up, IPAddress sta_ip, bool napt,
                    uint32_t pkts_from_host, uint32_t pkts_to_host);
