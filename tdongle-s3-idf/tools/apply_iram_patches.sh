#!/usr/bin/env bash
# Patch managed_components/ TinyUSB so the DWC2 (USB-OTG) ISR survives
# SPI-flash cache disables. Idempotent: safe to re-run after every
# `idf.py reconfigure` (which re-extracts managed components).
#
# What we touch:
#   1. dwc2_esp32.h -- esp_intr_alloc flag: add ESP_INTR_FLAG_IRAM so
#      the ROM trampoline uses the IRAM-resident dispatcher.
#   2. dcd_dwc2.c   -- attribute on dcd_int_handler() so its body lives
#      in IRAM (otherwise the IRAM trampoline would jump to flash and
#      stall during cache-disable windows).
#
# Why we don't fork the component: tracking upstream is easier with a
# small patch applied at configure time than maintaining a vendor copy.
# If the upstream changes the file layout in a way this patch can't
# find, the script exits non-zero so the build fails loudly instead of
# silently leaving the ISR in flash.

set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-$(cd "$(dirname "$0")/.." && pwd)}"
TUSB="${PROJECT_DIR}/managed_components/espressif__tinyusb/src/portable/synopsys/dwc2"
ESP32_H="${TUSB}/dwc2_esp32.h"
DCD_C="${TUSB}/dcd_dwc2.c"
MSC_C="${PROJECT_DIR}/managed_components/espressif__esp_tinyusb/tusb_msc_storage.c"

if [[ ! -f "${ESP32_H}" ]]; then
  echo "apply_iram_patches: managed component not present yet (${ESP32_H} missing)."
  echo "apply_iram_patches: run \`idf.py reconfigure\` first to fetch managed components."
  exit 0   # not an error -- first configure pass before deps are fetched
fi

patched=0

# --- Patch 1: dwc2_esp32.h ---
#   1a. esp_intr_alloc flag: OR in ESP_INTR_FLAG_IRAM so the ROM
#       trampoline dispatches via the IRAM-resident path.
#   1b. dwc2_int_handler_wrap: IRAM_ATTR so the wrapper the trampoline
#       jumps into is itself in IRAM (otherwise it page-faults during
#       cache-disable BEFORE reaching dcd_int_handler).
if grep -q 'ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM' "${ESP32_H}" \
   && grep -qE 'IRAM_ATTR static void dwc2_int_handler_wrap' "${ESP32_H}"; then
  echo "apply_iram_patches: dwc2_esp32.h already patched"
else
  if ! grep -q 'esp_intr_alloc(_dwc2_controller\[rhport\].irqnum, ESP_INTR_FLAG_LOWMED,' "${ESP32_H}" 2>/dev/null \
     && ! grep -q 'ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM' "${ESP32_H}"; then
    echo "ERROR: dwc2_esp32.h doesn't have the expected esp_intr_alloc call. Upstream changed."
    exit 1
  fi
  if ! grep -qE '^static void dwc2_int_handler_wrap\(' "${ESP32_H}" \
     && ! grep -qE '^IRAM_ATTR static void dwc2_int_handler_wrap\(' "${ESP32_H}"; then
    echo "ERROR: dwc2_esp32.h doesn't have the expected dwc2_int_handler_wrap signature. Upstream changed."
    exit 1
  fi
  # Pull in esp_attr.h so IRAM_ATTR resolves.
  if ! grep -q '#include "esp_attr.h"' "${ESP32_H}"; then
    sed -i.bak '/#include "esp_intr_alloc.h"/a\
#include "esp_attr.h"
' "${ESP32_H}"
    rm -f "${ESP32_H}.bak"
  fi
  # Idempotent: only run the substitution if the target isn't already present.
  if ! grep -q 'ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM' "${ESP32_H}"; then
    sed -i.bak 's#esp_intr_alloc(_dwc2_controller\[rhport\].irqnum, ESP_INTR_FLAG_LOWMED,#esp_intr_alloc(_dwc2_controller[rhport].irqnum, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM,#' "${ESP32_H}"
    rm -f "${ESP32_H}.bak"
  fi
  if ! grep -qE '^IRAM_ATTR static void dwc2_int_handler_wrap' "${ESP32_H}"; then
    sed -i.bak 's#^static void dwc2_int_handler_wrap(#IRAM_ATTR static void dwc2_int_handler_wrap(#' "${ESP32_H}"
    rm -f "${ESP32_H}.bak"
  fi
  echo "apply_iram_patches: dwc2_esp32.h patched (ESP_INTR_FLAG_IRAM + IRAM_ATTR on wrapper)"
  patched=1
fi

# --- Patch 2: dcd_dwc2.c dcd_int_handler IRAM_ATTR ---
if grep -qE '^IRAM_ATTR void dcd_int_handler\(' "${DCD_C}"; then
  echo "apply_iram_patches: dcd_dwc2.c already patched"
else
  if ! grep -qE '^void dcd_int_handler\(uint8_t rhport\) \{' "${DCD_C}"; then
    echo "ERROR: dcd_dwc2.c doesn't have the expected dcd_int_handler signature. Upstream changed."
    exit 1
  fi
  # Inject `#include "esp_attr.h"` near the top if not already there.
  if ! grep -q '#include "esp_attr.h"' "${DCD_C}"; then
    sed -i.bak '1a\
#include "esp_attr.h"
' "${DCD_C}"
    rm -f "${DCD_C}.bak"
  fi
  sed -i.bak 's#^void dcd_int_handler(uint8_t rhport) {#IRAM_ATTR void dcd_int_handler(uint8_t rhport) {#' "${DCD_C}"
  rm -f "${DCD_C}.bak"
  echo "apply_iram_patches: dcd_dwc2.c patched (added IRAM_ATTR to dcd_int_handler)"
  patched=1
fi

# --- Patch 3: tusb_msc_storage.c -- weak-link tud_msc_*_cb so our own
# implementations in disk.c can override them. Without this we get a
# duplicate-symbol link error; with this our strong symbols win.
if [[ -f "${MSC_C}" ]]; then
  if grep -qE '^__attribute__\(\(weak\)\)[[:space:]]+(void|bool|int32_t) tud_msc_' "${MSC_C}"; then
    echo "apply_patches: tusb_msc_storage.c already patched"
  else
    if ! grep -qE '^(void|bool|int32_t) tud_msc_' "${MSC_C}"; then
      echo "ERROR: tusb_msc_storage.c doesn't have the expected tud_msc_*_cb signatures. Upstream changed."
      exit 1
    fi
    sed -i.bak -E 's/^(void|bool|int32_t)( tud_msc_[a-z_0-9]+_cb)/__attribute__((weak)) \1\2/' "${MSC_C}"
    rm -f "${MSC_C}.bak"
    echo "apply_patches: tusb_msc_storage.c patched (weak-linked tud_msc_*_cb)"
    patched=1
  fi
fi

if [[ ${patched} -eq 0 ]]; then
  echo "apply_iram_patches: nothing to do"
fi
