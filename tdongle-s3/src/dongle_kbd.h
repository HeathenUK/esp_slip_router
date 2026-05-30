// dongle_kbd: USB HID keyboard composed into the dongle so it can type
// into DOS (or any host) on demand. Used by AT$TYPE and POST /type to
// drive DOS sessions from WiFi without a human at the keyboard.
//
// The host needs an HID-keyboard-aware USB driver to receive these
// keystrokes -- on the DOS side that's CHUSB.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void dongle_kbd_init(void);

// Type a string into the host. Plain ASCII bytes go through unchanged
// (translated by the keyboard layout). Tokens in angle brackets emit
// special keys:
//   <ENTER> <CR>      Return
//   <TAB>             Tab
//   <ESC>             Escape
//   <BS> <BACKSPACE>  Backspace
//   <SPACE>           Space (handy when string is whitespace-trimmed)
//   <F1>..<F12>       Function keys
//   <UP> <DOWN> <LEFT> <RIGHT>   Arrows
//   <HOME> <END> <PGUP> <PGDN> <INS> <DEL>
//   <CTRL+x> <ALT+x> <SHIFT+x>   Single-key combo (x is one char or a token)
//   <DELAY=ms>        Sleep N milliseconds (1..5000) between keystrokes
//   <<                Literal '<'
//
// Returns the number of source bytes consumed (i.e. len on success), or
// a negative value on parse error.
int dongle_kbd_type(const char *s, int len);

// Copy the recent in-memory typing debug log into `out` as plain text.
// Returns the number of bytes written, excluding the terminating NUL.
int dongle_kbd_log_dump(char *out, int outsz);

#ifdef __cplusplus
}
#endif
