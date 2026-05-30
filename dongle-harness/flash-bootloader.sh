#!/bin/bash
# Fallback flash path: AT$BOOT -> ROM bootloader -> esptool.
# Use this when AT$OTASTART can't run (firmware bricked, pre-OTA build, etc.)
# or you need to update the bootloader/partition table itself.
#
# Usage: ./flash-bootloader.sh [firmware.factory.bin]
#   defaults to ../tdongle-s3/.pio/build/dos/firmware.factory.bin
#
# Note: this writes firmware.factory.bin (full image with bootloader +
# partitions + app). The OTA path writes firmware.bin (app only).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
FW="${1:-$HERE/../tdongle-s3/.pio/build/dos/firmware.factory.bin}"
ESPTOOL_PY="${ESPTOOL_PY:-$HOME/.platformio/packages/tool-esptoolpy/esptool.py}"
ESPTOOL_PYTHON="${ESPTOOL_PYTHON:-$HOME/.platformio/penv/bin/python}"

[ -f "$FW" ] || { echo "flash-bl: no firmware at $FW" >&2; exit 1; }

FW_PORT=$(ls /dev/cu.usbmodemF412FA44AC4C* 2>/dev/null | head -1)
DL_PORT=/dev/cu.usbmodem123401

if [ ! -e "$DL_PORT" ] && [ -e "$FW_PORT" ]; then
    echo "flash-bl: triggering AT\$BOOT on $FW_PORT to enter ROM bootloader..."
    python3 - "$FW_PORT" <<'PY'
import sys, time, serial
p = serial.Serial(sys.argv[1], 115200, timeout=2); time.sleep(0.3)
p.reset_input_buffer(); p.write(b"AT$BOOT\r"); p.flush(); time.sleep(0.3)
print("dongle:", p.read(p.in_waiting or 1).decode('latin-1','replace').strip())
p.close()
PY
    echo "flash-bl: waiting up to 8s for $DL_PORT..."
    for i in $(seq 1 16); do
        sleep 0.5; [ -e "$DL_PORT" ] && break
    done
fi
[ -e "$DL_PORT" ] || { echo "flash-bl: bootloader port never appeared" >&2; exit 1; }

echo "flash-bl: esptool write_flash -> $DL_PORT"
"$ESPTOOL_PYTHON" "$ESPTOOL_PY" --chip esp32s3 --port "$DL_PORT" --baud 921600 \
    --after hard_reset write_flash 0x0 "$FW" 2>&1 | tail -6
RC=${PIPESTATUS[0]}
[ "$RC" -ne 0 ] && exit "$RC"

echo "flash-bl: waiting up to 10s for firmware $FW_PORT..."
for i in $(seq 1 20); do sleep 0.5; [ -e "$FW_PORT" ] && break; done
[ -e "$FW_PORT" ] || { echo "flash-bl: firmware port didn't return — unplug+replug" >&2; exit 2; }
echo "flash-bl: ok"
