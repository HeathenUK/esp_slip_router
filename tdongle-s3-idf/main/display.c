/* display.c -- 0.96" ST7735 status LCD, hand-rolled on esp_lcd_panel_io_spi.
 *
 * Port of the old arduino-esp32 display.cpp (Arduino_GFX). We use esp_lcd
 * only for the SPI + D/C plumbing (esp_lcd_panel_io_tx_param/tx_color); the
 * ST7735 init sequence, MADCTL/orientation, CASET/RASET windowing and the
 * text rendering are all ours, so there is no third-party panel driver to
 * trust and the orientation is deterministic rather than guessed.
 *
 * Panel: LilyGO T-Dongle S3, ST7735 GreenTab, 80x160 native, driven landscape
 * (160x80). Pins + offsets + IPS-inversion + BGR copied from the old firmware
 * (which matched the LilyGO factory esp_lcd init). The init command table is
 * transcribed verbatim from Arduino_ST7735's st7735_init_operations[].
 *
 * Layout: 5 rows of 8x16 text (80/16 = 5), reproducing display_slip /
 * display_modem. A low-priority task polls firmware state ~5 Hz and repaints
 * only rows whose text/colour changed (anti-flicker, as the old code did).
 */
#include "display.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "tusb.h"

#include "usb.h"
#include "ecm.h"
#include "modem.h"
#include "font8x16.h"

static const char *TAG = "display";

/* --- T-Dongle S3 ST7735 wiring (from old display.cpp) --- */
#define LCD_DC     2
#define LCD_CS     4
#define LCD_SCK    5
#define LCD_MOSI   3
#define LCD_RST    1
#define LCD_BL     38          /* backlight, ACTIVE LOW (LOW = on) */
#define LCD_BL_DUTY 192        /* 8-bit LEDC duty; active-low -> ~quarter brightness */

#define LCD_SPI_HOST   SPI2_HOST
#define LCD_PCLK_HZ    (20 * 1000 * 1000)   /* conservative; ST7735 is happy here */

/* Landscape geometry. ST7735 RAM is 132x162; this 80x160 panel sits at a
 * column offset of 26 and row offset of 1 in portrait. Driven landscape
 * (MADCTL MV set) those offsets swap: x(CASET)=1, y(RASET)=26. rotation-3
 * MADCTL = MX|MV|BGR (matches old Arduino_ST7735 rotation 3, ips, bgr). */
#define LCD_W        160
#define LCD_H        80
#define LCD_X_OFFSET 1
#define LCD_Y_OFFSET 26
#define LCD_MADCTL   (0x40 /*MX*/ | 0x20 /*MV*/ | 0x08 /*BGR*/)   /* = 0x68 */
#define LCD_IPS      1     /* IPS panel -> send INVON */

/* ST7735 command bytes (subset we use). */
#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT  0x11
#define ST7735_NORON   0x13
#define ST7735_INVON   0x21
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

/* Explicit RGB565 colours (same values the old firmware used; the BGR panel
 * bit makes them render as their names). */
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_CYAN    0x07FF
#define C_YELLOW  0xFFE0
#define C_MAGENTA 0xF81F
#define C_GREY    0x7BEF

#define ROW_H   16
#define NROWS   5
#define TEXT_X  2
#define MAX_COLS ((LCD_W - TEXT_X) / FONT_W)   /* glyphs that fit per row */

/* A full-row pixel buffer (160x16x2 = 5 KB) is a lot of heap on a board with
 * almost no headroom (the WiFi/lwIP path starves first). Render each row in
 * vertical BANDs instead, so the DMA buffer is only BAND_W*ROW_H*2 bytes.
 * BAND_W divides LCD_W evenly. */
#define BAND_W  40       /* divides LCD_W(160); 4 draws/row, 1.25 KB buffer */
#define BANDBUF_PX (BAND_W * ROW_H)

static spi_device_handle_t s_spi;
static uint16_t *s_bandbuf;                    /* BAND_W*ROW_H, big-endian RGB565, DMA-capable */
static char     s_last[NROWS][40];
static uint16_t s_lastc[NROWS];

/* ST7735 expects big-endian RGB565 (high byte first in memory). */
static inline uint16_t be16(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }

/* Hand-rolled SPI (no esp_lcd panel-io layer -- it cost ~11 KB of heap we
 * can't spare). DC is driven manually: LOW for the command byte, HIGH for
 * data. Polling transfers are synchronous, so DC is stable for the whole
 * transaction and the band buffer is safe to reuse immediately after. CS is
 * auto-managed by the spi_master driver (spics_io_num). */
static inline void wr_cmd(uint8_t cmd) {
    gpio_set_level(LCD_DC, 0);
    spi_transaction_t t = { .length = 8, .flags = SPI_TRANS_USE_TXDATA };
    t.tx_data[0] = cmd;
    spi_device_polling_transmit(s_spi, &t);
}
static void wr_data(const uint8_t *d, size_t n) {
    if (!n) return;
    gpio_set_level(LCD_DC, 1);
    spi_transaction_t t = { .length = 8 * n, .tx_buffer = d };
    spi_device_polling_transmit(s_spi, &t);
}
static inline void wr_cmd_data(uint8_t cmd, const uint8_t *d, size_t n) {
    wr_cmd(cmd);
    wr_data(d, n);
}

/* Verbatim from Arduino_ST7735 st7735_init_operations[] gamma tables. */
static const uint8_t GMCTRP1[16] = {
    0x09, 0x16, 0x09, 0x20, 0x21, 0x1B, 0x13, 0x19,
    0x17, 0x15, 0x1E, 0x2B, 0x04, 0x05, 0x02, 0x0E };
static const uint8_t GMCTRN1[16] = {
    0x0B, 0x14, 0x08, 0x1E, 0x22, 0x1D, 0x18, 0x1E,
    0x1B, 0x1A, 0x24, 0x2B, 0x06, 0x06, 0x02, 0x0F };

static void panel_init_seq(void) {
    /* Hardware reset (old code toggled RST when defined). */
    gpio_set_direction(LCD_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_RST, 0); vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_RST, 1); vTaskDelay(pdMS_TO_TICKS(120));

    wr_cmd(ST7735_SWRESET);            vTaskDelay(pdMS_TO_TICKS(120));
    wr_cmd(ST7735_SLPOUT);             vTaskDelay(pdMS_TO_TICKS(120));
    wr_cmd_data(ST7735_COLMOD, (uint8_t[]){0x05}, 1);   /* 16-bit colour */
    wr_cmd_data(ST7735_GMCTRP1, GMCTRP1, sizeof GMCTRP1);
    wr_cmd_data(ST7735_GMCTRN1, GMCTRN1, sizeof GMCTRN1);
    vTaskDelay(pdMS_TO_TICKS(10));
    wr_cmd(ST7735_NORON);              vTaskDelay(pdMS_TO_TICKS(10));
    wr_cmd_data(ST7735_MADCTL, (uint8_t[]){LCD_MADCTL}, 1);
#if LCD_IPS
    wr_cmd(ST7735_INVON);
#endif
    wr_cmd(ST7735_DISPON);             vTaskDelay(pdMS_TO_TICKS(10));
}

static void backlight_init(void) {
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num  = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);
    ledc_channel_config_t c = {
        .gpio_num = LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = LCD_BL_DUTY,           /* active-low: this is pin-HIGH time */
        .hpoint = 0,
    };
    ledc_channel_config(&c);
}

/* Blit one text row (anti-flicker: skip if text+colour unchanged). */
static void draw_row(int idx, const char *s, uint16_t color) {
    if (idx < 0 || idx >= NROWS) return;
    if (s_lastc[idx] == color && strcmp(s_last[idx], s) == 0) return;
    s_lastc[idx] = color;
    strncpy(s_last[idx], s, sizeof(s_last[idx]) - 1);
    s_last[idx][sizeof(s_last[idx]) - 1] = 0;

    const uint16_t fg = be16(color);
    int y_start = LCD_Y_OFFSET + idx * ROW_H;
    int y_end   = y_start + ROW_H - 1;

    /* Paint the row one BAND_W-wide vertical band at a time. */
    for (int bx = 0; bx < LCD_W; bx += BAND_W) {
        memset(s_bandbuf, 0, (size_t)BANDBUF_PX * sizeof(uint16_t));   /* black bg */

        for (int i = 0; s[i] && i < MAX_COLS; i++) {
            unsigned char ch = (unsigned char)s[i];
            if (ch < FONT_FIRST || ch > FONT_LAST) ch = ' ';
            int gx0 = TEXT_X + i * FONT_W;          /* glyph left edge (full row x) */
            if (gx0 >= bx + BAND_W || gx0 + FONT_W <= bx) continue;  /* not in band */
            const uint8_t *glyph = font8x16[ch - FONT_FIRST];
            for (int gy = 0; gy < FONT_H && gy < ROW_H; gy++) {
                uint8_t bits = glyph[gy];
                uint16_t *line = &s_bandbuf[gy * BAND_W];
                for (int gx = 0; gx < FONT_W; gx++) {
                    if (bits & (0x80 >> gx)) {
                        int x = gx0 + gx - bx;       /* x within band */
                        if (x >= 0 && x < BAND_W) line[x] = fg;
                    }
                }
            }
        }

        int x_start = LCD_X_OFFSET + bx;
        int x_end   = x_start + BAND_W - 1;
        wr_cmd_data(ST7735_CASET, (uint8_t[]){x_start >> 8, x_start & 0xFF,
                                              x_end   >> 8, x_end   & 0xFF}, 4);
        wr_cmd_data(ST7735_RASET, (uint8_t[]){y_start >> 8, y_start & 0xFF,
                                              y_end   >> 8, y_end   & 0xFF}, 4);
        wr_cmd_data(ST7735_RAMWR, (const uint8_t *)s_bandbuf,
                    (size_t)BANDBUF_PX * sizeof(uint16_t));
    }
}

static void clear_rows(void) {
    for (int i = 0; i < NROWS; i++) { s_last[i][0] = '\x01'; s_last[i][1] = 0; }
}

/* Compact bytes/sec with a single-letter unit (old rate_short). Integer-only
 * (no float printf) so the display task can run on a small stack. */
static void rate_short(uint32_t bps, char *out, size_t n) {
    if (bps < 1024) {
        snprintf(out, n, "%uB", (unsigned)bps);
    } else if (bps < 1024UL * 1024) {
        uint32_t k10 = (bps * 10) / 1024;                 /* tenths of KiB */
        snprintf(out, n, "%u.%uK", (unsigned)(k10 / 10), (unsigned)(k10 % 10));
    } else {
        uint32_t m10 = (uint32_t)(((uint64_t)bps * 10) / (1024UL * 1024)); /* tenths of MiB */
        snprintf(out, n, "%u.%uM", (unsigned)(m10 / 10), (unsigned)(m10 % 10));
    }
}

/* Format the STA IPv4 into out; "no ip" semantics handled by caller. */
static bool sta_ip_str(char *out, size_t n) {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (!netif || esp_netif_get_ip_info(netif, &ip) != ESP_OK || ip.ip.addr == 0)
        return false;
    snprintf(out, n, IPSTR, IP2STR(&ip.ip));
    return true;
}

static void display_task(void *arg) {
    char ip[20], r1[12], r2[12], buf[40];
    uint32_t prev_dl = 0, prev_ul = 0;
    uint32_t dl_ewma = 0, ul_ewma = 0;   /* smoothed display rate (EWMA) */
    int64_t prev_us = esp_timer_get_time();

    for (;;) {
        /* ONE screen: the CDC modem exists in every mode and the ECM bridge
         * is just an additional fact about this boot -- no more SLIP-era
         * either/or. Rows: mode+CDC / WiFi / IP / modem state / bridge rate. */
        wifi_ap_record_t ap;
        bool wifi_up = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
        bool have_ip = sta_ip_str(ip, sizeof ip);
        bool cdc_up  = tud_cdc_n_connected(0);
        const char *peer = NULL;
        bool online = modem_online_peer(&peer);

        const char *mode = (usb_net_mode() == 1) ? "NET" :
                           (usb_net_mode() == 2) ? "DEV" : "HID";
        snprintf(buf, sizeof buf, "DOSongle %s %s", mode, cdc_up ? "H+" : "H?");
        draw_row(0, buf, cdc_up ? C_MAGENTA : C_RED);
        draw_row(1, wifi_up ? "WiFi: up" : "WiFi: ...", wifi_up ? C_GREEN : C_YELLOW);
        draw_row(2, have_ip ? ip : "no ip", C_WHITE);

        if (online && peer && peer[0]) {
            draw_row(3, peer, C_GREEN);
        } else {
            draw_row(3, "command mode", C_GREY);
        }

        if (usb_net_enabled()) {
            /* bridge throughput deltas over the elapsed interval */
            int64_t now = esp_timer_get_time();
            uint32_t dl = ecm_stat_tx_bytes();   /* net -> host (download) */
            uint32_t ul = ecm_stat_rx_bytes();   /* host -> net (upload) */
            int64_t dt_us = now - prev_us;
            uint32_t dl_bps = (dt_us > 0) ? (uint32_t)(((uint64_t)(dl - prev_dl) * 1000000) / dt_us) : 0;
            uint32_t ul_bps = (dt_us > 0) ? (uint32_t)(((uint64_t)(ul - prev_ul) * 1000000) / dt_us) : 0;
            prev_dl = dl; prev_ul = ul; prev_us = now;
            /* EWMA low-pass on the displayed rate. The raw 200 ms sample swings
             * between a burst (~150 KB/s) and 0 because the ECM bridge carries
             * one frame in flight at a time, so the unfiltered number flickers.
             * A weighted moving average (new 1/8, prior 7/8 -> ~1.6 s time
             * constant) shows the sustained rate. Integer-only, no buffer; the
             * same exponential smoother TCP uses for RTT (RFC 6298). */
            dl_ewma = (uint32_t)(((uint64_t)dl_ewma * 7 + dl_bps) >> 3);
            ul_ewma = (uint32_t)(((uint64_t)ul_ewma * 7 + ul_bps) >> 3);
            rate_short(dl_ewma, r1, sizeof r1);
            rate_short(ul_ewma, r2, sizeof r2);
            snprintf(buf, sizeof buf, "D%s U%s", r1, r2);
            draw_row(4, buf, C_GREEN);
        } else {
            draw_row(4, "hold BTN: net mode", C_GREY);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void display_init(void) {
    s_bandbuf = heap_caps_malloc((size_t)BANDBUF_PX * sizeof(uint16_t),
                                 MALLOC_CAP_DMA);
    if (!s_bandbuf) { ESP_LOGE(TAG, "bandbuf alloc failed"); return; }

    spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BANDBUF_PX * sizeof(uint16_t) + 16,
    };
    esp_err_t err = spi_bus_initialize(LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) { ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err)); return; }

    /* DC is a plain GPIO we drive ourselves (command vs data). */
    gpio_set_direction(LCD_DC, GPIO_MODE_OUTPUT);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = LCD_PCLK_HZ,
        .mode = 0,
        .spics_io_num = LCD_CS,        /* driver auto-toggles CS per transaction */
        .queue_size = 1,
    };
    err = spi_bus_add_device(LCD_SPI_HOST, &dev, &s_spi);
    if (err != ESP_OK) { ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err)); return; }

    panel_init_seq();
    clear_rows();

    /* Clear all five rows to black before lighting the backlight, so the
     * uninitialised GRAM doesn't flash noise. Use a unique sentinel colour
     * so the first real poll (which uses C_* values) always repaints. */
    for (int i = 0; i < NROWS; i++) draw_row(i, "", C_BLACK);
    clear_rows();

    backlight_init();

    /* Integer-only task (no float printf) -> small stack is safe. */
    BaseType_t ok = xTaskCreatePinnedToCore(display_task, "display", 2560, NULL,
                                            2, NULL, 0 /* CPU0 */);
    if (ok != pdPASS) ESP_LOGE(TAG, "display task create failed");
    else ESP_LOGI(TAG, "ST7735 up (%dx%d landscape)", LCD_W, LCD_H);
}
