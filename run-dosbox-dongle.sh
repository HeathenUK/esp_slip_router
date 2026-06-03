#!/bin/sh
# DOSBox launcher: dongle on COM1, CF as C:, autoexec runs USBTERM.
exec /Applications/dosbox.app/Contents/MacOS/DOSBox \
    -conf "/Users/gadyke/esp_slip_router/dosbox-dongle.conf"
