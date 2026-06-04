#!/usr/bin/env bash
# Patch the ACTIVE ESP-IDF tree (not the project) for the T-Dongle S3.
#
# Unlike tools/apply_iram_patches.sh (which patches the project's managed_
# components/), this patches the IDF tree itself. It exists so we never again
# depend on a pre-patched IDF artifact: point IDF_PATH at a clean ESP-IDF, run
# this once, build.
#
#   Winbond W25Q128JV auto-suspend whitelist
#   ----------------------------------------
#   spi_flash_chip_winbond_get_caps() only enables SPI_FLASH_CHIP_CAP_SUSPEND
#   for 0xEF4017 (W25Q64JV). The T-Dongle S3 carries a W25Q128JV (0xEF4018).
#   With CONFIG_SPI_FLASH_AUTO_SUSPEND=y (which we need so the non-IRAM OTG ISR
#   keeps running through flash ops) an un-whitelisted chip asserts at boot.
#   So add 0xEF4018 alongside 0xEF4017.
#
# Idempotent. Usage:  IDF_PATH=~/esp/esp-idf ./tools/apply_idf_patches.sh
set -euo pipefail

IDF="${IDF_PATH:-${1:-}}"
[ -n "${IDF}" ] || { echo "apply_idf_patches: set IDF_PATH (or pass the IDF dir as \$1)"; exit 1; }
WB="${IDF}/components/spi_flash/spi_flash_chip_winbond.c"
[ -f "${WB}" ] || { echo "apply_idf_patches: ${WB} not found"; exit 1; }

if grep -q '0xEF4018' "${WB}"; then
    echo "apply_idf_patches: winbond already patched (0xEF4018 present)"
else
    grep -q 'case 0xEF4017:' "${WB}" || {
        echo "ERROR: 'case 0xEF4017:' not found in ${WB} -- IDF layout changed, patch by hand"; exit 1; }
    # BSD/macOS-compatible append (same style as apply_iram_patches.sh).
    sed -i.bak '/case 0xEF4017:/a\
    case 0xEF4018:
' "${WB}"
    rm -f "${WB}.bak"
    echo "apply_idf_patches: winbond patched (added 0xEF4018 W25Q128JV suspend)"
fi
