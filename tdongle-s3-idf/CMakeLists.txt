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

# lwIP NAPT UDP entries default to a 2-second timeout, which is short
# enough that a DNS response with mild latency can arrive after the
# reverse-mapping entry has been GC'd. Bump it for any UDP traffic
# (DNS, NTP, etc.) routed via SLIP+NAPT. lwIP doesn't expose this
# constant as Kconfig, so we override via global compile-defs that
# propagate to every component (including lwip).
idf_build_set_property(COMPILE_DEFINITIONS "-DIP_NAPT_TIMEOUT_MS_UDP=30000" APPEND)

# Bump lwIP's pbuf pool so burst RX from the SLIP host has headroom.
# Default IDF is 16; 32 strikes the right balance for this target.
# SIZING HISTORY: 16 (default) -> 64 (was starving heap on the 1 MB
# Hayes transfer, dropping free heap to <1 KB) -> 32. PBUF_POOL is
# heap-allocated by memp_init -- each entry is ~PBUF_POOL_BUFSIZE
# (TCP_MSS + lwIP header ≈ 1.5 KB), so 32 entries cost ~48 KB heap
# at init versus ~96 KB at 64. PBUF_POOL_SIZE is not exposed as
# Kconfig in IDF, hence the -D override.
idf_build_set_property(COMPILE_DEFINITIONS "-DPBUF_POOL_SIZE=32" APPEND)

# CDC bulk endpoint buffer. Default for full-speed USB is 64 bytes
# (matches MaxPacketSize). Bumping to 512 lets the TinyUSB driver
# queue 8 max-sized USB packets internally before needing the host
# to drain them, reducing IRQ pressure and the per-packet ACK-cycle
# latency that was capping CDC throughput. Reported as ~3x gain on
# upstream TinyUSB / esp_tinyusb (esp-usb #54 / #212). Not exposed
# as Kconfig in esp_tinyusb 1.4.5; the override is safe because the
# TinyUSB CDC header uses #ifndef before its default.
idf_build_set_property(COMPILE_DEFINITIONS "-DCFG_TUD_CDC_EP_BUFSIZE=512" APPEND)

# DWC2 buffer-DMA (2026-06-12). The default slave/PIO mode has a fatal
# TXFE-refill race under sustained ECM TX (~1 in 800 frames at 416 KB/s):
# the IN endpoint ends up armed (EPENA=1, PKTCNT remaining), FIFO empty,
# ITTXFE+NAK latched, DAINT clear -- NAKing forever (tinyusb #1618 matches).
# Buffer DMA hands whole transfers to the core (no per-packet refill) and
# fixed it completely: 10 MB through the bridge at 406 KB/s, no wedge.
#
# HISTORY/CAVEAT: DMA was briefly convicted of breaking MSC -- that turned
# out to be confounded: macOS had begun mount-blocking the dev-disk after a
# day of re-enumeration churn (a known macOS behavior on this project), and
# MSC "failed" identically on PIO and in every mode. The IOKit chain stays
# fully attached and reads are served; only the volume mount is withheld
# host-side. MSC-under-DMA still deserves a clean-day verification (or the
# authoritative consumer: CHUSB on the DOS host). If genuine DMA-mode bugs
# do surface, tinyusb master (post-0.19) carries DMA fixes that aren't in
# any registry release and can't be cherry-picked (core API drift) -- the
# path then is vendoring the whole tree.
idf_build_set_property(COMPILE_DEFINITIONS "-DCFG_TUD_DWC2_DMA_ENABLE=1" APPEND)
