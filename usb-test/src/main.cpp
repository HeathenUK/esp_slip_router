// Bare-minimum USB CDC smoke test for T-Dongle S3.
// No display, no WiFi, no NVS, no GFX, no modem code.
// Just spam recognisable lines so we can see if ANYTHING reaches the host.

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    // No setRxBufferSize / setTxTimeoutMs / enableReboot. Stock defaults.
}

void loop() {
    static uint32_t cnt = 0;
    Serial.printf("USBTEST cnt=%lu millis=%lu free=%u\r\n",
                  (unsigned long)cnt++, (unsigned long)millis(),
                  (unsigned)ESP.getFreeHeap());
    delay(500);
}
