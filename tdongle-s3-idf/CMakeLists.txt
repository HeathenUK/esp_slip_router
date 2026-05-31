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
