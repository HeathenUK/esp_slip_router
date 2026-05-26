#include "display.h"

#ifdef ENABLE_DISPLAY

#include <Arduino_GFX_Library.h>
#include "config.h"

// --- T-Dongle S3 0.96" ST7735 (from LilyGO factory esp_lcd init) ---
#define LCD_DC    2
#define LCD_CS    4
#define LCD_SCK   5
#define LCD_MOSI  3
#define LCD_RST   1
#define LCD_BL    38   // backlight, ACTIVE LOW (LOW = on)
// Brightness via LEDC PWM. Active-low, so duty = pin HIGH time and brightness =
// (255 - duty)/255:  0 = full, 128 = ~half, 192 = ~quarter, 255 = off.
#define LCD_BL_DUTY 192

static Arduino_DataBus *bus =
    new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);

// Offsets passed UNswapped (26,1,26,1): Arduino_TFT::setRotation swaps axes for
// landscape itself. rotation 3 = 180° from 1, ips=true (color inversion), BGR.
static Arduino_GFX *gfx =
    new Arduino_ST7735(bus, LCD_RST, 3, true /*ips*/,
                       80, 160, 26, 1, 26, 1, true /*bgr*/);

// Explicit RGB565 colours (don't rely on library colour-name macros).
static const uint16_t C_BLACK   = 0x0000;
static const uint16_t C_WHITE   = 0xFFFF;
static const uint16_t C_RED     = 0xF800;
static const uint16_t C_GREEN   = 0x07E0;
static const uint16_t C_CYAN    = 0x07FF;
static const uint16_t C_YELLOW  = 0xFFE0;
static const uint16_t C_MAGENTA = 0xF81F;
static const uint16_t C_GREY    = 0x7BEF;

static const int ROW_H = 16;   // 8px font * size 2
static const int NROWS = 5;
static String last_line[NROWS];

static void draw_line(int idx, const String &s, uint16_t color) {
    if (idx < 0 || idx >= NROWS) return;
    if (s == last_line[idx]) return;   // unchanged -> no redraw, no flicker
    last_line[idx] = s;
    int y = idx * ROW_H;
    gfx->fillRect(0, y, gfx->width(), ROW_H, C_BLACK);
    gfx->setTextColor(color);
    gfx->setTextSize(2);
    gfx->setCursor(2, y);
    gfx->print(s);
}

static void clear_rows() {
    for (int i = 0; i < NROWS; i++) last_line[i] = "\x01";  // force first redraw
}

void display_init() {
    ledcAttach(LCD_BL, 1000, 8);   // 1 kHz, 8-bit PWM
    ledcWrite(LCD_BL, LCD_BL_DUTY);
    gfx->begin();
    gfx->fillScreen(C_BLACK);
    clear_rows();
}

// Compact bytes/sec with a single-letter unit, so both directions fit one row.
static String rate_short(uint32_t bps) {
    char b[12];
    if (bps < 1024)                 snprintf(b, sizeof(b), "%uB", (unsigned)bps);
    else if (bps < 1024UL * 1024)   snprintf(b, sizeof(b), "%.1fK", bps / 1024.0);
    else                            snprintf(b, sizeof(b), "%.1fM", bps / (1024.0 * 1024.0));
    return String(b);
}

void display_slip(bool wifi_up, IPAddress sta_ip, bool napt,
                  uint32_t dl_bps, uint32_t ul_bps) {
    draw_line(0, "SLIP Router", C_CYAN);
    // WiFi: yellow until both associated and NAT is enabled, then green.
    draw_line(1, String("WiFi: ") + (wifi_up ? "up" : "..."),
              (wifi_up && napt) ? C_GREEN : C_YELLOW);
    draw_line(2, wifi_up ? sta_ip.toString() : String("no ip"), C_WHITE);
    // Down (net->host) and Up (host->net) on one row: "D12.3K U0.8K"
    draw_line(3, "D" + rate_short(dl_bps) + " U" + rate_short(ul_bps), C_GREEN);
    draw_line(4, "", C_GREY);   // blank (also clears any stale row from modem mode)
}

void display_modem(bool wifi_up, IPAddress sta_ip, bool online, const char *peer) {
    draw_line(0, "WiFi Modem", C_MAGENTA);
    draw_line(1, String("WiFi: ") + (wifi_up ? "up" : "..."),
              wifi_up ? C_GREEN : C_YELLOW);
    draw_line(2, wifi_up ? sta_ip.toString() : String("no ip"), C_WHITE);
    if (online && peer && peer[0]) {
        draw_line(3, "online", C_GREEN);
        draw_line(4, String(peer), C_WHITE);
    } else {
        draw_line(3, "command mode", C_GREY);
        draw_line(4, "ATDT host:port", C_GREY);
    }
}

#else  // !ENABLE_DISPLAY

void display_init() {}
void display_slip(bool, IPAddress, bool, uint32_t, uint32_t) {}
void display_modem(bool, IPAddress, bool, const char *) {}

#endif
