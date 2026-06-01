/* kbd: USB HID keyboard typing DSL.
 *
 * Composes with CDC + MSC on the same composite USB device (descriptor
 * already declares HID; see usb.c ITF_NUM_HID).
 *
 * Use kbd_type() to send a string to the host. Plain ASCII bytes go
 * through unchanged (translated by the host's keyboard layout via
 * HID_ASCII_TO_KEYCODE). Tokens in angle brackets emit special keys:
 *
 *   <ENTER> <CR>      Return
 *   <TAB>             Tab
 *   <ESC>             Escape
 *   <BS> <BACKSPACE>  Backspace
 *   <SPACE>           Space
 *   <F1>..<F12>       Function keys
 *   <UP> <DOWN> <LEFT> <RIGHT>   Arrows
 *   <HOME> <END> <PGUP> <PGDN> <INS> <DEL>
 *   <CTRL+x> <ALT+x> <SHIFT+x>   Single-key combo (x is one char or a token)
 *   <DELAY=ms>        Sleep N milliseconds (1..5000) between keystrokes
 *   <<                Literal '<'
 *
 * Returns the number of source bytes consumed on success, or a
 * negative value on parse error.
 *
 * The host needs an HID-keyboard-aware USB driver to receive these
 * keystrokes -- on the DOS side that's CHUSB.
 */
#pragma once
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int kbd_type(const char *s, int len);

/* HID typing events are logged via disk_logf with a "kbd:" prefix and
 * surface through /disk-log -- there is no longer a separate kbd log
 * ring or /type-log endpoint (SRAM trim 2026-06-01). */

#ifdef __cplusplus
}
#endif
