# T-Dongle S3 main component.
#
# Composite USB device (MSC + HID + CDC) over the OTG controller, WiFi
# STA, lwIP NAPT for the SLIP gateway role, HTTP server for control
# plane, plus the wear-levelled FAT12 backing store. Most logic is the
# same as the arduino-esp32 version it replaces; the swap is purely
# at the API-wrapper layer.

idf_component_register(
    SRCS
        "main.c"
        "usb.c"
        "disk.c"
        "modem.c"
        "fat.c"
        "slip.c"
        "dns_forwarder.c"
        "kbd.c"
        "display.c"
        "ssh_spike.c"
        "ssh.c"
    INCLUDE_DIRS "."
    REQUIRES
        # Networking / WiFi
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
        # TLS termination (HTTPS / secure relay) -- esp-tls wraps IDF's mbedTLS.
        esp-tls
        # SSH termination -- vendored libssh2 (see components/libssh2, BSD-3).
        libssh2
        # HTTP server (already used in the arduino-esp32 build via
        # esp_http_server -- same component here).
        esp_http_server
        # OTA -- in-app firmware updates served from POST /ota.
        app_update
        # esp_timer_get_time() -- microsecond timer for MSC callback timing.
        esp_timer
        # Storage -- wear-levelled FAT partition for the MSC dev disk.
        wear_levelling
        spi_flash
        esp_partition
        # esp_psram -- pulled only for the boot-time capability dump.
        # The board has no PSRAM; the include is conditionally used.
        esp_psram
        # USB device class. The managed component `esp_tinyusb` wraps
        # TinyUSB's tud_* callbacks; we pull it via idf_component.yml.
        esp_tinyusb
        # mDNS for dosongle.local.
        mdns
        # GPIO for the BOOT-button mode toggle.
        esp_driver_gpio
        # Status LCD (display.c): ST7735 over SPI via esp_lcd panel-io,
        # SPI2 bus, and LEDC PWM for the backlight.
        esp_lcd
        esp_driver_spi
        esp_driver_ledc
)
