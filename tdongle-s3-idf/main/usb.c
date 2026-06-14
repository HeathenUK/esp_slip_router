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
#include "esp_system.h"        /* esp_restart -- BOOT-button mode toggle */
#include "driver/gpio.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "class/hid/hid_device.h"
#include "class/net/net_device.h"   /* CFG_TUD_NET_* + tud_network_mac_address */

#include "nvs.h"

#include "disk.h"
#include "modem.h"
#include "ecm.h"

#include "hal/usb_serial_jtag_ll.h"
#include "hal/usb_wrap_ll.h"
#include "soc/usb_wrap_struct.h"
#include "esp_private/periph_ctrl.h"
#include "soc/periph_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "usb"

/* Device descriptor -- composite (IAD-style) class, custom PID, default
 * VID picked up from Kconfig (Espressif's 0x303A). Non-const: NET mode
 * (AT$USBNET=1) swaps idProduct to 0x4024 so hosts that cache device
 * configs by VID/PID (macOS, Windows) never reuse the HID-shaped config
 * for the ECM-shaped one. */
static tusb_desc_device_t s_dev_desc = {
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

/* ===== NET mode (AT$USBNET=1): CDC + MSC + ECM, no HID ==================
 *
 * The S3's DWC2 core has only 5 IN-endpoint TX FIFOs (EP0 included) and the
 * normal composite uses all of them, so the ECM data-IN endpoint takes the
 * HID keyboard's slot -- the two functions are a boot-time either/or.
 *
 * The ECM function also runs WITHOUT its notification endpoint (that would
 * be IN #6): TinyUSB's driver parses it as optional, the CHUSB host plan
 * explicitly tolerates its absence, and Linux's cdc_ether assumes link-up
 * without a status endpoint. apply_iram_patches.sh guards netd_report()
 * against the missing endpoint. Descriptor below is TUD_CDC_ECM_DESCRIPTOR
 * (usbd.h) minus the 7-byte notification endpoint.
 *
 * Whole config blob stays well under the CH375 host's 512-byte parse buffer.
 */
enum {
    ITF_NET_CDC = 0,
    ITF_NET_CDC_DATA,
    ITF_NET_MSC,
    ITF_NET_ECM,
    ITF_NET_ECM_DATA,
    ITF_NET_TOTAL,
};

#define EPNUM_ECM_OUT  0x04   /* HID's endpoint number, reused for ECM data */
#define EPNUM_ECM_IN   0x84

#define ECM_NONOTIF_DESC_LEN (8+9+5+5+13+9+9+7+7)   /* 72: ECM template minus notif EP */

/* CDC-ECM function, notification endpoint omitted (see block comment).
 * Layout per TUD_CDC_ECM_DESCRIPTOR (TinyUSB usbd.h, MIT):
 * IAD, comm interface (02h/06h), Header + Union + Ethernet Networking
 * functional descriptors, data interface alt0 (no EPs) + alt1 (bulk pair). */
#define ECM_NONOTIF_DESCRIPTOR(_itfnum, _desc_stridx, _mac_stridx, _epout, _epin, _epsize, _maxsegmentsize) \
  /* Interface Association */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ETHERNET_CONTROL_MODEL, 0, 0,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ETHERNET_CONTROL_MODEL, 0, _desc_stridx,\
  /* CDC-ECM Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),\
  /* CDC-ECM Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* CDC-ECM Functional Descriptor */\
  13, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ETHERNET_NETWORKING, _mac_stridx, 0, 0, 0, 0, U16_TO_U8S_LE(_maxsegmentsize), U16_TO_U8S_LE(0), 0,\
  /* CDC Data Interface (alt 0: inactive, no endpoints) */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 0, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* CDC Data Interface (alt 1: active) */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 1, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

/* Mode 1, "DOS": CDC + MSC + ECM-without-notif (PID 0x4024). CHUSB's plan
 * explicitly tolerates a missing notification endpoint and Linux's cdc_ether
 * assumes link-up without one, so the real target keeps MSC. macOS will NOT
 * publish the interface in this mode (measured 2026-06-11: AppleUserECMData
 * attaches but never registers an interface without the notif EP). */
#define CFG_TOTAL_LEN_NET_DOS (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN + ECM_NONOTIF_DESC_LEN)

static const uint8_t s_cfg_desc_net_dos[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NET_TOTAL, 0, CFG_TOTAL_LEN_NET_DOS,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    TUD_CDC_DESCRIPTOR(ITF_NET_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    TUD_MSC_DESCRIPTOR(ITF_NET_MSC, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),

    ECM_NONOTIF_DESCRIPTOR(ITF_NET_ECM, 6, 7 /* iMACAddress string */,
                           EPNUM_ECM_OUT, EPNUM_ECM_IN,
                           CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};

/* Mode 2, "dev": CDC + ECM-with-notif, no MSC (PID 0x4025). macOS requires
 * the notification endpoint before AppleUserECMData publishes an Ethernet
 * interface (proven 2026-06-11: this shape -> en8 + DHCP lease + active);
 * affording it means giving up MSC's IN endpoint for the boot. Used for
 * Mac/Linux-side validation and bridge development. */
enum {
    ITF_DEV_CDC = 0,
    ITF_DEV_CDC_DATA,
    ITF_DEV_ECM,
    ITF_DEV_ECM_DATA,
    ITF_DEV_TOTAL,
};
#define EPNUM_ECM_NOTIF 0x83   /* MSC's IN slot, reused in dev mode */

#if CFG_TUD_NCM
/* NCM variant of Mode 2 (build-time, CONFIG_TINYUSB_NET_MODE_NCM=y). Same
 * interface/endpoint shape as the ECM dev descriptor -- CDC + a network class
 * with notif EP, no MSC -- so the 5-IN-FIFO budget and the hardened macOS
 * service binding (kept on PID 0x4024) are unchanged; only the network class
 * (and thus the host driver: AppleUSBNCM vs AppleUserECMData) differs. NCM
 * batches multiple datagrams per USB transfer (NTB) -- the throughput
 * experiment against ECM's cadence ceiling. ECM and NCM are mutually exclusive
 * in one TinyUSB build, so this whole config is compiled in only for the NCM
 * image; the ECM build below is the shipping one. */
#define CFG_TOTAL_LEN_NET_DEV (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

static const uint8_t s_cfg_desc_net_dev[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_DEV_TOTAL, 0, CFG_TOTAL_LEN_NET_DEV,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    TUD_CDC_DESCRIPTOR(ITF_DEV_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    TUD_CDC_NCM_DESCRIPTOR(ITF_DEV_ECM, 6, 7 /* iMACAddress string */,
                           EPNUM_ECM_NOTIF, 64,
                           EPNUM_ECM_OUT, EPNUM_ECM_IN,
                           CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};
#else
#define CFG_TOTAL_LEN_NET_DEV (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_CDC_ECM_DESC_LEN)

static const uint8_t s_cfg_desc_net_dev[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_DEV_TOTAL, 0, CFG_TOTAL_LEN_NET_DEV,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    TUD_CDC_DESCRIPTOR(ITF_DEV_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    TUD_CDC_ECM_DESCRIPTOR(ITF_DEV_ECM, 6, 7 /* iMACAddress string */,
                           EPNUM_ECM_NOTIF, 64,
                           EPNUM_ECM_OUT, EPNUM_ECM_IN,
                           CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};
#endif

/* Boot-time mode (NVS "slip-router"/"usbnet"): 0 = normal (HID),
 * 1 = DOS net (MSC + ECM no-notif), 2 = dev net (ECM + notif, no MSC). */
static uint8_t s_usbnet_mode = 0;
uint8_t usb_net_mode(void)    { return s_usbnet_mode; }
bool    usb_net_enabled(void) { return s_usbnet_mode != 0; }

/* String descriptors. Serial is derived from the WiFi-station MAC on
 * the fly in usb_start() so each dongle reports a stable unique id.
 *
 * esp_tinyusb caps the table at 8 entries (USB_STRING_DESCRIPTOR_ARRAY_SIZE
 * -- exceeding it fails tinyusb_driver_install with ESP_ERR_NOT_SUPPORTED),
 * and HID/ECM never coexist, so NET mode REUSES slot 6 for the ECM interface
 * name and adds the iMACAddress at slot 7: exactly 12 hex digits the host
 * parses into the 6-byte MAC its network adapter will use (the esp_tinyusb
 * wrapper converts the ASCII to the UTF-16LE the spec wants). */
static char        s_serial_str[13]; /* 12 hex chars + NUL */
static char        s_mac_str[13];    /* 12 hex chars + NUL (iMACAddress) */
static const char *s_strings[8] = {
    (const char[]){0x09, 0x04}, /* 0: en-US */
    "DOSongle",                 /* 1: manufacturer */
    "DOSongle T-Dongle S3",     /* 2: product */
    s_serial_str,               /* 3: serial -- filled at runtime */
    "DOSongle CDC",             /* 4: CDC interface */
    "DOSongle Disk",            /* 5: MSC interface */
    "DOSongle Keyboard",        /* 6: HID itf; NET mode: "DOSongle Net" */
    NULL,                       /* 7: NET mode only: iMACAddress */
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

/* MSC backing store lives in disk.c -- it owns the wl_handle and
 * provides strong tud_msc_*_cb implementations (the esp_tinyusb stock
 * ones are weak-linked via tools/apply_iram_patches.sh). usb_start
 * just calls disk_init() here; the SCSI plumbing wires itself up at
 * link time. */

/* ---- BOOT-button mode toggle ----
 *
 * Successor to the old SLIP-mode long-press (retired with slip.c): holding
 * the BOOT button (GPIO0, active-low, ~2 s) toggles AT$USBNET between 1
 * (DOS net: Ethernet, no HID) and 0 (HID composite) and reboots to apply --
 * a no-terminal-needed escape hatch on the DOS machine itself. From dev
 * mode 2 a hold lands on 1 (the toggle targets the two deployment modes). */
#define BTN_PIN     GPIO_NUM_0
#define BTN_HOLD_MS 2000

static void usb_btn_task(void *arg) {
    (void)arg;
    gpio_config_t bcfg = {
        .pin_bit_mask  = (1ULL << BTN_PIN),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&bcfg);
    uint32_t held_ms = 0;
    bool     fired   = false;
    for (;;) {
        if (gpio_get_level(BTN_PIN) == 0) {          /* pressed */
            held_ms += 50;
            if (!fired && held_ms >= BTN_HOLD_MS) {
                fired = true;
                uint8_t next = (s_usbnet_mode == 1) ? 0 : 1;
                nvs_handle_t h;
                if (nvs_open("slip-router", NVS_READWRITE, &h) == ESP_OK) {
                    esp_err_t we = nvs_set_u8(h, "usbnet", next);
                    if (we == ESP_OK) we = nvs_commit(h);
                    nvs_close(h);
                    if (we == ESP_OK) {
                        disk_logf("btn: usbnet %u -> %u, rebooting",
                                  (unsigned)s_usbnet_mode, (unsigned)next);
                        vTaskDelay(pdMS_TO_TICKS(150));  /* let the log land */
                        esp_restart();
                    }
                }
            }
        } else {
            held_ms = 0;
            fired   = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
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

    /* NET modes (AT$USBNET, NVS-persisted): swap functions within the DWC2's
     * 5 IN-EP FIFO budget (see the descriptor block comments). Each shape
     * gets its own PID so hosts never reuse a cached config across shapes.
     * DEFAULT (NVS unset) is mode 1 -- the DOS deployment shape (Ethernet,
     * no HID keyboard): the right out-of-box behavior on the Pocket386, and
     * provably inert on a Mac (without the notif EP macOS never publishes
     * the interface). AT$USBNET=0 restores the HID composite. */
    {
        nvs_handle_t h;
        uint8_t v = 1;
        if (nvs_open("slip-router", NVS_READONLY, &h) == ESP_OK) {
            nvs_get_u8(h, "usbnet", &v);
            nvs_close(h);
        }
        s_usbnet_mode = (v <= 2) ? v : 1;
    }
    if (s_usbnet_mode != 0) {
        ecm_mac_init();    /* host reads the MAC during enumeration */
        snprintf(s_mac_str, sizeof(s_mac_str), "%02X%02X%02X%02X%02X%02X",
                 tud_network_mac_address[0], tud_network_mac_address[1],
                 tud_network_mac_address[2], tud_network_mac_address[3],
                 tud_network_mac_address[4], tud_network_mac_address[5]);
        s_strings[6] = "DOSongle Net";   /* HID is absent; slot 6 = ECM itf */
        s_strings[7] = s_mac_str;        /* iMACAddress */
        /* PID per shape (hosts cache configs by VID/PID). Mode 2 keeps 0x4024:
         * that's the identity the Mac's hardened "DOSongle T-Dongle S3"
         * network service (manual IP, NO router, v6 off) is bound to -- a new
         * PID would mint a fresh macOS service defaulting to DHCP and re-open
         * the route-hijack hole. Mode 1 is Mac-inert anyway (without the notif
         * EP macOS never publishes the interface), so it takes the new PID. */
        s_dev_desc.idProduct = (s_usbnet_mode == 1) ? 0x4025 : 0x4024;
    }

    esp_err_t err = disk_init();
    if (err != ESP_OK) return err;

    switch_phy_jtag_to_otg();

    const tinyusb_config_t cfg = {
        .device_descriptor        = &s_dev_desc,
        .string_descriptor        = s_strings,
        /* esp_tinyusb hard-caps this at 8; normal mode passes 7 (slot 7 is
         * NULL outside NET modes). */
        .string_descriptor_count  = s_usbnet_mode ? 8 : 7,
        .external_phy             = false,
        .configuration_descriptor = (s_usbnet_mode == 1) ? s_cfg_desc_net_dos :
                                    (s_usbnet_mode == 2) ? s_cfg_desc_net_dev :
                                                           s_cfg_desc,
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

    /* The Hayes AT modem owns CDC0. esp_log output would interleave
     * with AT responses on the same interface, so we DO NOT call
     * esp_tusb_init_console here -- diagnostics go through the disk
     * log ring buffer (GET /disk-log) instead. */
    err = modem_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "modem_init: %s", esp_err_to_name(err));
        return err;
    }

    /* BOOT-button USBNET toggle (works in every mode; see usb_btn_task). */
    xTaskCreate(usb_btn_task, "usb_btn", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "composite USB up: CDC + %s, serial=%s",
             (s_usbnet_mode == 1) ? "MSC + ECM(no-notif)" :
             (s_usbnet_mode == 2) ? "ECM(notif)"          : "MSC + HID",
             s_serial_str);
    return ESP_OK;
}

void usb_soft_reconnect(unsigned hold_ms) {
    disk_logf("usb: soft reconnect -- tud_disconnect, hold %ums", hold_ms);
    /* Drop the D+ pullup: host sees a clean USB device removal. */
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    /* Re-assert: host sees a fresh insertion and re-enumerates from
     * scratch -- new descriptors fetch, fresh storage probe, fresh
     * diskarbitration session. */
    tud_connect();
    disk_logf("usb: soft reconnect -- tud_connect (re-enumerating)");
}
