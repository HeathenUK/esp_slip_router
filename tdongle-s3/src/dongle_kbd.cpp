// dongle_kbd: USB HID keyboard. Composes alongside the existing CDC
// (Hayes/SLIP) and MSC (DOSONGLE dev disk) on the same USB device.
// See dongle_kbd.h for the typing-string DSL.

#include "dongle_kbd.h"

#include <Arduino.h>
#include <USB.h>
#include <USBHIDKeyboard.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#define DBG(...) do { Serial0.printf(__VA_ARGS__); } while (0)

static USBHIDKeyboard s_kbd;
static bool s_kbd_ready = false;

#define KBD_LOG_LINES 48
#define KBD_LOG_LINE_LEN 96

static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_log[KBD_LOG_LINES][KBD_LOG_LINE_LEN];
static uint32_t s_log_seq = 0;

static void kbd_logf(const char *fmt, ...) {
    char line[KBD_LOG_LINE_LEN];
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

void dongle_kbd_init(void) {
    s_kbd.begin();
    // USB.begin() has already been called by the USBMSC init path; HID
    // attaches to the same composite. arduino-esp32 manages descriptors.
    USB.begin();
    s_kbd_ready = true;
    kbd_logf("init ready");
    DBG("[kbd] HID keyboard up\n");
}

// ---- key-name table for <TOKEN> handling ----

struct KeyName { const char *name; uint8_t key; };

static const KeyName KEY_NAMES[] = {
    { "ENTER",     KEY_RETURN },
    { "RETURN",    KEY_RETURN },
    { "CR",        KEY_RETURN },
    { "TAB",       KEY_TAB },
    { "ESC",       KEY_ESC },
    { "ESCAPE",    KEY_ESC },
    { "BS",        KEY_BACKSPACE },
    { "BACKSPACE", KEY_BACKSPACE },
    { "SPACE",     ' ' },           // ASCII, USBHIDKeyboard handles it
    { "UP",        KEY_UP_ARROW },
    { "DOWN",      KEY_DOWN_ARROW },
    { "LEFT",      KEY_LEFT_ARROW },
    { "RIGHT",     KEY_RIGHT_ARROW },
    { "HOME",      KEY_HOME },
    { "END",       KEY_END },
    { "PGUP",      KEY_PAGE_UP },
    { "PGDN",      KEY_PAGE_DOWN },
    { "INS",       KEY_INSERT },
    { "DEL",       KEY_DELETE },
    { "F1",  KEY_F1 },  { "F2",  KEY_F2 },  { "F3",  KEY_F3 },  { "F4",  KEY_F4 },
    { "F5",  KEY_F5 },  { "F6",  KEY_F6 },  { "F7",  KEY_F7 },  { "F8",  KEY_F8 },
    { "F9",  KEY_F9 },  { "F10", KEY_F10 }, { "F11", KEY_F11 }, { "F12", KEY_F12 },
};

static bool keyname_lookup(const char *name, size_t namelen, uint8_t *out) {
    for (auto &k : KEY_NAMES) {
        if (strlen(k.name) == namelen && strncasecmp(k.name, name, namelen) == 0) {
            *out = k.key;
            return true;
        }
    }
    if (namelen == 1) {                  // single char -> ASCII
        *out = (uint8_t)name[0];
        return true;
    }
    return false;
}

// Emit one keystroke. `key` is either an ASCII byte (write() through the
// layout map) or a USBHIDKeyboard KEY_* macro (press/release).
static void emit_key(uint8_t key) {
    kbd_logf("emit key=0x%02x '%c'", key, printable_char(key));
    if (key < 0x80) {
        s_kbd.write(key);
    } else {
        s_kbd.press(key);
        s_kbd.release(key);
    }
}

// Emit one keystroke with a modifier held down. `mod` is one of
// KEY_LEFT_CTRL/SHIFT/ALT.
static void emit_combo(uint8_t mod, uint8_t key) {
    kbd_logf("emit combo mod=0x%02x key=0x%02x '%c'", mod, key, printable_char(key));
    s_kbd.press(mod);
    if (key < 0x80) {
        // Lowercase the ASCII so Ctrl+x doesn't get a shift implicitly;
        // CHUSB / DOS expect bare scancodes for control combinations.
        if (key >= 'A' && key <= 'Z') key = key - 'A' + 'a';
        // Map via raw press for ASCII letters under modifier
        // (USBHIDKeyboard.write would re-emit the shift table)
        if (key >= 'a' && key <= 'z') {
            s_kbd.press(key);
            s_kbd.release(key);
        } else {
            s_kbd.write(key);
        }
    } else {
        s_kbd.press(key);
        s_kbd.release(key);
    }
    s_kbd.release(mod);
}

int dongle_kbd_type(const char *s, int len) {
    if (!s_kbd_ready) {
        kbd_logf("type rejected not-ready len=%d", len);
        return -1;
    }
    kbd_logf("type begin len=%d", len);
    int i = 0;
    while (i < len) {
        char c = s[i];
        if (c == '<' && i + 1 < len && s[i + 1] == '<') {
            // "<<" -> literal '<'
            kbd_logf("literal escaped '<' at=%d", i);
            s_kbd.write('<');
            i += 2;
            continue;
        }
        if (c != '<') {
            kbd_logf("literal at=%d ch=0x%02x '%c'", i, (uint8_t)c, printable_char((uint8_t)c));
            s_kbd.write((uint8_t)c);
            i++;
            continue;
        }
        // <TOKEN>
        int end = i + 1;
        while (end < len && s[end] != '>') end++;
        if (end >= len) {
            kbd_logf("parse error unterminated token at=%d", i);
            return -1;
        }
        const char *body = s + i + 1;
        size_t bodylen = end - (i + 1);
        kbd_logf("token at=%d '%.*s'", i, (int)bodylen, body);

        // <DELAY=ms>
        if (bodylen > 6 && strncasecmp(body, "DELAY=", 6) == 0) {
            int ms = atoi(body + 6);
            if (ms < 1) ms = 1;
            if (ms > 5000) ms = 5000;
            kbd_logf("delay ms=%d", ms);
            delay(ms);
            i = end + 1;
            continue;
        }
        // <CTRL+x> / <ALT+x> / <SHIFT+x>
        uint8_t mod = 0;
        const char *plus = (const char *)memchr(body, '+', bodylen);
        if (plus) {
            size_t mlen = plus - body;
            if      (mlen == 4 && strncasecmp(body, "CTRL", 4) == 0)  mod = KEY_LEFT_CTRL;
            else if (mlen == 3 && strncasecmp(body, "ALT", 3) == 0)   mod = KEY_LEFT_ALT;
            else if (mlen == 5 && strncasecmp(body, "SHIFT", 5) == 0) mod = KEY_LEFT_SHIFT;
        }
        if (mod) {
            const char *kname = plus + 1;
            size_t klen = (body + bodylen) - kname;
            uint8_t key;
            if (!keyname_lookup(kname, klen, &key)) {
                kbd_logf("parse error unknown combo key '%.*s'", (int)klen, kname);
                return -1;
            }
            emit_combo(mod, key);
        } else {
            uint8_t key;
            if (!keyname_lookup(body, bodylen, &key)) {
                kbd_logf("parse error unknown token '%.*s'", (int)bodylen, body);
                return -1;
            }
            emit_key(key);
        }
        i = end + 1;
    }
    kbd_logf("type end len=%d", len);
    return len;
}

int dongle_kbd_log_dump(char *out, int outsz) {
    if (!out || outsz <= 0) return 0;

    portENTER_CRITICAL(&s_log_mux);
    uint32_t seq = s_log_seq;
    uint32_t first = (seq > KBD_LOG_LINES) ? (seq - KBD_LOG_LINES + 1) : 1;
    int n = 0;
    for (uint32_t cur = first; cur <= seq && n < outsz - 1; ++cur) {
        const char *line = s_log[cur % KBD_LOG_LINES];
        int wrote = snprintf(out + n, outsz - n, "%s\n", line);
        if (wrote < 0) break;
        if (wrote >= outsz - n) {
            n = outsz - 1;
            break;
        }
        n += wrote;
    }
    portEXIT_CRITICAL(&s_log_mux);
    out[n] = 0;
    return n;
}
