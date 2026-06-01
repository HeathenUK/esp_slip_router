/* kbd.c -- USB HID keyboard typing DSL.
 *
 * Port of the arduino-esp32 dongle_kbd.cpp, rewritten on TinyUSB's
 * tud_hid_keyboard_report() instead of the USBHIDKeyboard wrapper.
 * HID descriptor (8-byte boot keyboard report) is declared in usb.c.
 */

#include "kbd.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "class/hid/hid.h"
#include "class/hid/hid_device.h"

#include "disk.h"

/* ---- in-memory typing log (small ring, like disk-log but separate) ---- */

#define KBD_LOG_LINES    48
#define KBD_LOG_LINE_LEN 128   /* prefix "<seq> " + 96 msg + slack */
#define KBD_LOG_MSG_LEN  96

static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static char         s_log[KBD_LOG_LINES][KBD_LOG_LINE_LEN];
static uint32_t     s_log_seq = 0;

static void kbd_logf(const char *fmt, ...) {
    char line[KBD_LOG_MSG_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);

    portENTER_CRITICAL(&s_log_mux);
    uint32_t seq = ++s_log_seq;
    snprintf(s_log[seq % KBD_LOG_LINES], KBD_LOG_LINE_LEN, "%lu %s",
             (unsigned long)seq, line);
    portEXIT_CRITICAL(&s_log_mux);
}

static char printable_char(uint8_t c) {
    return (c >= 32 && c < 127) ? (char)c : '.';
}

/* ---- HID report emission ---- */

/* Inter-keystroke gap. Hosts (CHUSB on the DOS side, the Mac kernel CDC
 * stack) need a moment between press and release to register the event.
 * 5 ms each is a reasonable balance between throughput and missed keys. */
#define KBD_STEP_MS 5

static void hid_send(uint8_t modifier, uint8_t keycode) {
    /* Boot-keyboard report: modifier byte, then 6 keycodes. We only ever
     * press one non-modifier key at a time, so slots 1..5 stay zero. */
    uint8_t keys[6] = { keycode, 0, 0, 0, 0, 0 };
    if (!tud_hid_ready()) return;
    tud_hid_keyboard_report(0 /* report_id */, modifier, keys);
}

static void emit_press_release(uint8_t modifier, uint8_t keycode) {
    hid_send(modifier, keycode);
    vTaskDelay(pdMS_TO_TICKS(KBD_STEP_MS));
    hid_send(0, 0);
    vTaskDelay(pdMS_TO_TICKS(KBD_STEP_MS));
}

/* Emit one ASCII byte via the standard US-layout ascii-to-HID table. */
static void emit_ascii(uint8_t c) {
    static const uint8_t conv[128][2] = { HID_ASCII_TO_KEYCODE };
    if (c >= 128) return;
    uint8_t shift = conv[c][0];
    uint8_t key   = conv[c][1];
    if (key == 0) return;
    emit_press_release(shift ? KEYBOARD_MODIFIER_LEFTSHIFT : 0, key);
}

/* ---- <TOKEN> table ---- */

struct keyname { const char *name; uint8_t hid_code; };

static const struct keyname KEY_NAMES[] = {
    { "ENTER",     HID_KEY_ENTER },
    { "RETURN",    HID_KEY_ENTER },
    { "CR",        HID_KEY_ENTER },
    { "TAB",       HID_KEY_TAB },
    { "ESC",       HID_KEY_ESCAPE },
    { "ESCAPE",    HID_KEY_ESCAPE },
    { "BS",        HID_KEY_BACKSPACE },
    { "BACKSPACE", HID_KEY_BACKSPACE },
    { "SPACE",     HID_KEY_SPACE },
    { "UP",        HID_KEY_ARROW_UP },
    { "DOWN",      HID_KEY_ARROW_DOWN },
    { "LEFT",      HID_KEY_ARROW_LEFT },
    { "RIGHT",     HID_KEY_ARROW_RIGHT },
    { "HOME",      HID_KEY_HOME },
    { "END",       HID_KEY_END },
    { "PGUP",      HID_KEY_PAGE_UP },
    { "PGDN",      HID_KEY_PAGE_DOWN },
    { "INS",       HID_KEY_INSERT },
    { "DEL",       HID_KEY_DELETE },
    { "F1",  HID_KEY_F1 },  { "F2",  HID_KEY_F2 },
    { "F3",  HID_KEY_F3 },  { "F4",  HID_KEY_F4 },
    { "F5",  HID_KEY_F5 },  { "F6",  HID_KEY_F6 },
    { "F7",  HID_KEY_F7 },  { "F8",  HID_KEY_F8 },
    { "F9",  HID_KEY_F9 },  { "F10", HID_KEY_F10 },
    { "F11", HID_KEY_F11 }, { "F12", HID_KEY_F12 },
};

/* Map a token body (e.g. "ENTER", "F7", or a single ASCII char like "a")
 * to an HID usage code. Returns true on success. For single-char bodies
 * the ASCII byte is returned as-is and the caller decides whether to
 * shift it via the ASCII table. */
static bool keyname_lookup(const char *name, size_t namelen, uint8_t *out, bool *is_ascii) {
    *is_ascii = false;
    for (size_t i = 0; i < sizeof KEY_NAMES / sizeof KEY_NAMES[0]; ++i) {
        const struct keyname *k = &KEY_NAMES[i];
        if (strlen(k->name) == namelen &&
            strncasecmp(k->name, name, namelen) == 0) {
            *out = k->hid_code;
            return true;
        }
    }
    if (namelen == 1) {
        *out = (uint8_t)name[0];
        *is_ascii = true;
        return true;
    }
    return false;
}

/* Emit `key` (either an ASCII byte or an HID usage code) with `modifier`
 * held. For ASCII letters we lowercase first so e.g. <CTRL+C> produces
 * the raw Ctrl-c scancode the way DOS apps expect. */
static void emit_combo(uint8_t modifier, uint8_t key, bool key_is_ascii) {
    if (!key_is_ascii) {
        emit_press_release(modifier, key);
        return;
    }
    if (key >= 'A' && key <= 'Z') key = key - 'A' + 'a';
    static const uint8_t conv[128][2] = { HID_ASCII_TO_KEYCODE };
    if (key >= 128) return;
    uint8_t hid_key = conv[key][1];
    if (hid_key == 0) return;
    /* Ignore the ASCII-table's shift bit when a modifier was specified --
     * the caller asked for the bare scancode under the modifier, not
     * Shift+modifier+key. */
    emit_press_release(modifier, hid_key);
}

/* ---- public API ---- */

int kbd_type(const char *s, int len) {
    if (!tud_hid_ready()) {
        kbd_logf("type rejected hid not ready len=%d", len);
        return -1;
    }
    kbd_logf("type begin len=%d", len);
    int i = 0;
    while (i < len) {
        char c = s[i];
        if (c == '<' && i + 1 < len && s[i + 1] == '<') {
            kbd_logf("literal escaped '<' at=%d", i);
            emit_ascii('<');
            i += 2;
            continue;
        }
        if (c != '<') {
            kbd_logf("literal at=%d ch=0x%02x '%c'", i, (uint8_t)c,
                     printable_char((uint8_t)c));
            emit_ascii((uint8_t)c);
            i++;
            continue;
        }
        int end = i + 1;
        while (end < len && s[end] != '>') end++;
        if (end >= len) {
            kbd_logf("parse error unterminated token at=%d", i);
            return -1;
        }
        const char *body = s + i + 1;
        size_t bodylen = (size_t)(end - (i + 1));
        kbd_logf("token at=%d '%.*s'", i, (int)bodylen, body);

        /* <DELAY=ms> */
        if (bodylen > 6 && strncasecmp(body, "DELAY=", 6) == 0) {
            int ms = atoi(body + 6);
            if (ms < 1) ms = 1;
            if (ms > 5000) ms = 5000;
            kbd_logf("delay ms=%d", ms);
            vTaskDelay(pdMS_TO_TICKS(ms));
            i = end + 1;
            continue;
        }

        /* <CTRL+x> / <ALT+x> / <SHIFT+x> */
        uint8_t mod = 0;
        const char *plus = (const char *)memchr(body, '+', bodylen);
        if (plus) {
            size_t mlen = (size_t)(plus - body);
            if      (mlen == 4 && strncasecmp(body, "CTRL", 4)  == 0) mod = KEYBOARD_MODIFIER_LEFTCTRL;
            else if (mlen == 3 && strncasecmp(body, "ALT", 3)   == 0) mod = KEYBOARD_MODIFIER_LEFTALT;
            else if (mlen == 5 && strncasecmp(body, "SHIFT", 5) == 0) mod = KEYBOARD_MODIFIER_LEFTSHIFT;
        }

        if (mod) {
            const char *kname = plus + 1;
            size_t klen = (size_t)((body + bodylen) - kname);
            uint8_t key;
            bool is_ascii;
            if (!keyname_lookup(kname, klen, &key, &is_ascii)) {
                kbd_logf("parse error unknown combo key '%.*s'", (int)klen, kname);
                return -1;
            }
            emit_combo(mod, key, is_ascii);
        } else {
            uint8_t key;
            bool is_ascii;
            if (!keyname_lookup(body, bodylen, &key, &is_ascii)) {
                kbd_logf("parse error unknown token '%.*s'", (int)bodylen, body);
                return -1;
            }
            if (is_ascii) emit_ascii(key);
            else          emit_press_release(0, key);
        }
        i = end + 1;
    }
    kbd_logf("type end len=%d", len);
    return len;
}

int kbd_log_dump(char *out, int outsz) {
    if (!out || outsz <= 0) return 0;

    portENTER_CRITICAL(&s_log_mux);
    uint32_t seq = s_log_seq;
    uint32_t first = (seq > KBD_LOG_LINES) ? (seq - KBD_LOG_LINES + 1) : 1;
    int n = 0;
    for (uint32_t cur = first; cur <= seq && n < outsz - 1; ++cur) {
        int wrote = snprintf(out + n, (size_t)(outsz - n), "%s\n",
                             s_log[cur % KBD_LOG_LINES]);
        if (wrote < 0) break;
        if (wrote >= outsz - n) { n = outsz - 1; break; }
        n += wrote;
    }
    portEXIT_CRITICAL(&s_log_mux);
    out[n] = 0;
    return n;
}
