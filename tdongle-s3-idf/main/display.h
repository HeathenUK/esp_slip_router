/* display.h -- 0.96" ST7735 status LCD (T-Dongle S3).
 *
 * Port of the old arduino-esp32 display.cpp. Self-contained: display_init()
 * brings up SPI2 + the panel + backlight and spawns a low-priority task that
 * polls firmware state ~5 Hz and paints one of two views (SLIP / MODEM),
 * reproducing the old layout. Nothing else to call from app_main.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the LCD and start the status task. Safe to call once, late in
 * app_main (after wifi + slip + modem init so the accessors it polls exist).
 * On panel-bringup failure it logs and returns without starting the task --
 * the rest of the firmware is unaffected. */
void display_init(void);

#ifdef __cplusplus
}
#endif
