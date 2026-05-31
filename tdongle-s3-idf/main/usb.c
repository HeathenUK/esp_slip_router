/* usb.c -- composite USB device for the T-Dongle S3 (MSC + HID + CDC).
 *
 * Stock esp_tinyusb supports CDC + MSC out of the box but not HID, so the
 * configuration descriptor is hand-rolled here. The whole stack is
 * brought up via tinyusb_driver_install() with this descriptor; MSC is
 * backed by the `ffat` wear-levelled partition through the helper
 * `tinyusb_msc_storage_init_spiflash()`; CDC uses esp_tinyusb's high-
 * level tusb_cdcacm helper; HID uses raw tud_hid_* callbacks (no
 * Espressif wrapper exists).
 *
 * Once enumerated, stdout/printf is redirected to CDC via
 * esp_tusb_init_console() so we have a console again (the JTAG console
 * is gone, since USB-OTG and USB-Serial-JTAG share GPIO 19/20).
 */

#include "usb.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_partition.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_msc_storage.h"
#include "tusb_console.h"
#include "wear_levelling.h"
#include "class/hid/hid_device.h"

#include "hal/usb_serial_jtag_ll.h"
#include "hal/usb_wrap_ll.h"
#include "soc/usb_wrap_struct.h"
#include "esp_private/periph_ctrl.h"
#include "soc/periph_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "usb"

/* Device descriptor -- composite (IAD-style) class, custom PID, default
 * VID picked up from Kconfig (Espressif's 0x303A). */
static const tusb_desc_device_t s_dev_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,
    .idProduct          = 0x4023,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

/* HID report descriptor: standard boot keyboard (8 bytes in, 1 byte
 * LED out -- we ignore LEDs in the callback). */
static const uint8_t s_hid_report_desc[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(),
};

/* Interface / endpoint allocation:
 *   ITF 0  CDC control     EP 0x81 (notif IN, interrupt)
 *   ITF 1  CDC data        EP 0x02 OUT / 0x82 IN  (bulk)
 *   ITF 2  MSC             EP 0x03 OUT / 0x83 IN  (bulk)
 *   ITF 3  HID             EP 0x84 IN             (interrupt)
 */
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_MSC,
    ITF_NUM_HID,
    ITF_NUM_TOTAL,
};

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82
#define EPNUM_MSC_OUT   0x03
#define EPNUM_MSC_IN    0x83
#define EPNUM_HID_IN    0x84

#define CFG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t s_cfg_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CFG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),

    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 6, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(s_hid_report_desc), EPNUM_HID_IN, 8, 10),
};

/* String descriptors. Serial is derived from the WiFi-station MAC on
 * the fly in usb_start() so each dongle reports a stable unique id. */
static char        s_serial_str[13]; /* 12 hex chars + NUL */
static const char *s_strings[] = {
    (const char[]){0x09, 0x04}, /* 0: en-US */
    "DOSongle",                 /* 1: manufacturer */
    "DOSongle T-Dongle S3",     /* 2: product */
    s_serial_str,               /* 3: serial -- filled at runtime */
    "DOSongle CDC",             /* 4: CDC interface */
    "DOSongle Disk",            /* 5: MSC interface */
    "DOSongle Keyboard",        /* 6: HID interface */
};

/* ---- HID callbacks (TinyUSB asks us for these, even if we never
 * actually send a report). Stubbed for Phase 1a; the keyboard typing
 * engine lives in a later phase. */

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return s_hid_report_desc;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen) {
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)bufsize;
}

/* ---- MSC backing store -- mount the ffat partition through the
 * wear-levelling layer, then hand the WL handle to esp_tinyusb's MSC
 * helper. Phase 1a uses the stock SCSI plumbing; the write-back-cache
 * pattern from the arduino-esp32 build gets re-applied in 1b. */

static esp_err_t mount_msc_partition(void) {
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ffat");
    if (!part) {
        ESP_LOGE(TAG, "ffat partition not found");
        return ESP_ERR_NOT_FOUND;
    }

    wl_handle_t wl = WL_INVALID_HANDLE;
    esp_err_t err = wl_mount(part, &wl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wl_mount: %s", esp_err_to_name(err));
        return err;
    }

    const tinyusb_msc_spiflash_config_t cfg = {
        .wl_handle = wl,
    };
    err = tinyusb_msc_storage_init_spiflash(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "msc storage init: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "MSC backing store mounted (%u sectors x %u bytes)",
             (unsigned)tinyusb_msc_storage_get_sector_count(),
             (unsigned)tinyusb_msc_storage_get_sector_size());
    return ESP_OK;
}

/* ---- public entry point ---- */

/* Switch the USB phy from USB-Serial-JTAG to USB-OTG. ESP-IDF's
 * usb_new_phy() doesn't tear down JTAG state itself, so without this
 * sequence the OTG init "succeeds" but the host sees no enumeration --
 * the JTAG controller is still gripping D+/D- and the dwc2 controller
 * is in a stale state from JTAG mode. The exact sequence is the one
 * arduino-esp32 uses (esp32-hal-tinyusb.c around tinyusb_init), which
 * has been shipping on this hardware reliably for years.
 *
 * Order matters:
 *   1. Disable JTAG pad + clock so JTAG releases its grip.
 *   2. Reset the whole USB peripheral so the dwc2 controller starts
 *      from a clean state when usb_new_phy claims it for OTG.
 *   3. Brief delay so the host sees the disconnect transient before
 *      OTG re-asserts the pull-up.
 *
 * Boot-time JTAG console stops working at step 1; that's expected. */
static void switch_phy_jtag_to_otg(void) {
    {
        int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
        usb_serial_jtag_ll_phy_enable_pad(false);
        usb_serial_jtag_ll_enable_bus_clock(false);
    }
    /* Route the internal FSLS phy to the USB Wrap (OTG controller).
     * usb_new_phy() supposedly does this via usb_wrap_hal_phy_set_external
     * but the bits don't appear to stick when called from a context where
     * JTAG was just driving the same pins -- so we explicitly drive the
     * RTC_CNTL mux bits here and verify on the disk-log. */
    usb_wrap_ll_phy_enable_external(&USB_WRAP, false);
    periph_module_reset(PERIPH_USB_MODULE);
    periph_module_enable(PERIPH_USB_MODULE);
    vTaskDelay(pdMS_TO_TICKS(20));
}

esp_err_t usb_start(void) {
    /* Fill in the serial-number string from the STA MAC. */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_serial_str, sizeof(s_serial_str), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    esp_err_t err = mount_msc_partition();
    if (err != ESP_OK) return err;

    switch_phy_jtag_to_otg();

    const tinyusb_config_t cfg = {
        .device_descriptor        = &s_dev_desc,
        .string_descriptor        = s_strings,
        .string_descriptor_count  = (int)(sizeof(s_strings) / sizeof(s_strings[0])),
        .external_phy             = false,
        .configuration_descriptor = s_cfg_desc,
    };
    err = tinyusb_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tinyusb_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    /* Bring CDC up. RX/TX callbacks land in Phase 1c (AT modem); for
     * now we just need the interface present so the host enumerates
     * the whole composite. */
    const tinyusb_config_cdcacm_t cdc_cfg = {
        .usb_dev   = TINYUSB_USBDEV_0,
        .cdc_port  = TINYUSB_CDC_ACM_0,
    };
    err = tusb_cdc_acm_init(&cdc_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tusb_cdc_acm_init: %s", esp_err_to_name(err));
        return err;
    }

    /* Redirect stdout/printf/ESP_LOG to the CDC interface so we have a
     * console once the host attaches. */
    esp_tusb_init_console(TINYUSB_CDC_ACM_0);

    ESP_LOGI(TAG, "composite USB up: MSC + HID + CDC, serial=%s", s_serial_str);
    return ESP_OK;
}
