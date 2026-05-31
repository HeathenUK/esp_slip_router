# T-Dongle S3 -- ESP-IDF native build
#
# Migrated off arduino-esp32 in order to get full Kconfig control --
# specifically CONFIG_SPI_FLASH_AUTO_SUSPEND=y, which the prebuilt
# arduino-esp32 framework libs ship with disabled. With AUTO_SUSPEND on,
# the W25Q128JV flash chip on this target suspends long erases whenever
# the cache misses, keeping the OTG ISR responsive throughout. That's
# the fix for strict USB hosts (CHUSB on Pocket386) that previously
# saw the device go silent for ~70 ms per erase.

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tdongle_s3)

# After managed_components/ is populated, patch the TinyUSB DWC2 ISR
# allocation + handler so it lives in IRAM. Without this, even with
# AUTO_SUSPEND active, the residual cache-disable window during flash
# erase will silence the OTG controller because the ROM trampoline
# can't reach a flash-resident handler. See
# tools/apply_iram_patches.sh and project memory `idf-tinyusb-iram-patch`.
execute_process(
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/apply_iram_patches.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE _iram_patch_rc
    OUTPUT_VARIABLE _iram_patch_out
    ERROR_VARIABLE  _iram_patch_err
)
if(NOT _iram_patch_rc EQUAL 0)
    message(FATAL_ERROR "tools/apply_iram_patches.sh failed (rc=${_iram_patch_rc}):\n${_iram_patch_out}${_iram_patch_err}")
endif()
message(STATUS "${_iram_patch_out}")
