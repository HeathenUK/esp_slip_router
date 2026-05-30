// dongle_kbd: USB HID keyboard. Composes alongside the existing CDC
// (Hayes/SLIP) and MSC (DOSONGLE dev disk) on the same USB device.
// See dongle_kbd.h for the typing-string DSL.

#include "dongle_kbd.h"

#include <Arduino.h>
#include <USB.h>
#include <USBHIDKeyboard.h>
#include <string.h>
#include <ctype.h>

#define DBG(...) do { Serial0.printf(__VA_ARGS__); } while (0)

static USBHIDKeyboard s_kbd;
static bool s_kbd_ready = false;

void dongle_kbd_init(void) {
    s_kbd.begin();
    // USB.begin() has already been called by the USBMSC init path; HID
    // attaches to the same composite. arduino-esp32 manages descriptors.
    USB.begin();
    s_kbd_ready = true;
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
    if (!s_kbd_ready) return -1;
    int i = 0;
    while (i < len) {
        char c = s[i];
        if (c == '<' && i + 1 < len && s[i + 1] == '<') {
            // "<<" -> literal '<'
            s_kbd.write('<');
            i += 2;
            continue;
        }
        if (c != '<') {
            s_kbd.write((uint8_t)c);
            i++;
            continue;
        }
        // <TOKEN>
        int end = i + 1;
        while (end < len && s[end] != '>') end++;
        if (end >= len) return -1;       // unterminated <...>
        const char *body = s + i + 1;
        size_t bodylen = end - (i + 1);

        // <DELAY=ms>
        if (bodylen > 6 && strncasecmp(body, "DELAY=", 6) == 0) {
            int ms = atoi(body + 6);
            if (ms < 1) ms = 1;
            if (ms > 5000) ms = 5000;
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
            if (!keyname_lookup(kname, klen, &key)) return -1;
            emit_combo(mod, key);
        } else {
            uint8_t key;
            if (!keyname_lookup(body, bodylen, &key)) return -1;
            emit_key(key);
        }
        i = end + 1;
    }
    return len;
}
