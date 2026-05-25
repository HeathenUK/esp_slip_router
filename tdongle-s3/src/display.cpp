#include "display.h"

#ifdef ENABLE_DISPLAY

#include <Arduino_GFX_Library.h>
#include "config.h"

#ifdef DEBUG_TO_USB
#define DBGD(...) do { Serial.printf(__VA_ARGS__); } while (0)
#else
#define DBGD(...) do {} while (0)
#endif

// --- T-Dongle S3 0.96" ST7735 (from LilyGO factory esp_lcd init) ---
#define LCD_DC    2
#define LCD_CS    4
#define LCD_SCK   5
#define LCD_MOSI  3
#define LCD_RST   1
#define LCD_BL    38   // backlight, ACTIVE LOW (LOW = on)

// SPI bus (FSPI on S3). DC, CS, SCK, MOSI, MISO=unused.
static Arduino_DataBus *bus =
    new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);

// ST7735 panel: rotation 1 (160x80 landscape), ips=true -> color inversion ON
// (factory does invert_color(true)), native 80x160, offsets 26/1 (portrait) and
// 1/26 (landscape, swapped), BGR colour order.
// Offsets passed UNswapped (26,1,26,1): Arduino_TFT::setRotation already swaps
// axes for landscape (rot 1 -> _xStart=ROW_OFFSET1=1, _yStart=COL_OFFSET2=26).
static Arduino_GFX *gfx =
    new Arduino_ST7735(bus, LCD_RST, 3 /*rotation: 180° from 1*/, true /*ips/invert*/,
                       80 /*w*/, 160 /*h*/,
                       26 /*col_off1*/, 1 /*row_off1*/,
                       26 /*col_off2*/, 1 /*row_off2*/,
                       true /*bgr*/);

// Explicit RGB565 colours (don't rely on library colour-name macros).
static const uint16_t C_BLACK  = 0x0000;
static const uint16_t C_WHITE  = 0xFFFF;
static const uint16_t C_RED    = 0xF800;
static const uint16_t C_GREEN  = 0x07E0;
static const uint16_t C_CYAN   = 0x07FF;
static const uint16_t C_YELLOW = 0xFFE0;
static const uint16_t C_GREY   = 0x7BEF;

static const int ROW_H = 16;   // 8px font * size 2
static const int NROWS = 5;
static String last_line[NROWS];

static void draw_line(int idx, const String &s, uint16_t color) {
    if (idx < 0 || idx >= NROWS) return;
    if (s == last_line[idx]) return;  // unchanged -> no redraw, no flicker
    last_line[idx] = s;
    int y = idx * ROW_H;
    gfx->fillRect(0, y, gfx->width(), ROW_H, C_BLACK);
    gfx->setTextColor(color);
    gfx->setTextSize(2);
    gfx->setCursor(2, y);
    gfx->print(s);
}

void display_init() {
    // Backlight ON first (active LOW), so a dark panel vs lit-but-blank is
    // distinguishable even if the SPI/init misbehaves.
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, LOW);
    DBGD("[disp] backlight %d -> LOW (on)\n", LCD_BL);

    DBGD("[disp] gfx->begin()...\n");
    if (!gfx->begin()) {
        DBGD("[disp] gfx->begin() FAILED\n");
    } else {
        DBGD("[disp] gfx->begin() ok\n");
    }

    // Bring-up sentinel: solid red proves the panel is being driven, even if
    // offsets/colours need tuning. Remove once confirmed.
    gfx->fillScreen(C_RED);
    DBGD("[disp] fill RED, w=%d h=%d\n", gfx->width(), gfx->height());
    delay(600);

    gfx->fillScreen(C_BLACK);
    for (int i = 0; i < NROWS; i++) last_line[i] = "";
    draw_line(0, "SLIP Router", C_CYAN);
    DBGD("[disp] init complete\n");
}

void display_status(bool wifi_up, IPAddress sta_ip, bool napt,
                    uint32_t pkts_from_host, uint32_t pkts_to_host) {
    char buf[40];

    draw_line(0, "SLIP Router", C_CYAN);
    draw_line(1, String("WiFi: ") + (wifi_up ? "up" : "..."),
              wifi_up ? C_GREEN : C_YELLOW);
    draw_line(2, wifi_up ? sta_ip.toString() : String("no ip"), C_WHITE);

    snprintf(buf, sizeof(buf), "SLIP %d.%d.%d.%d",
             SLIP_LOCAL_A, SLIP_LOCAL_B, SLIP_LOCAL_C, SLIP_LOCAL_D);
    draw_line(3, buf, napt ? C_WHITE : C_GREY);

    snprintf(buf, sizeof(buf), "rx%lu tx%lu",
             (unsigned long)pkts_from_host, (unsigned long)pkts_to_host);
    draw_line(4, buf, C_GREY);
}

#else  // !ENABLE_DISPLAY

void display_init() {}
void display_status(bool, IPAddress, bool, uint32_t, uint32_t) {}

#endif
