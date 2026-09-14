# SPDX-License-Identifier: BSD-3-Clause-Clear
# CMakeLists.txt for Renesas FPB-RA0E2 HAL

# Directory paths (three-tier scalable structure)
set(CMSIS_DIR ${MCU_DRIVERS_DIR}/cmsis)
set(FSP_COMMON_DIR ${MCU_DRIVERS_DIR}/renesas_fsp_common)
set(RA0E2_DIR ${MCU_DRIVERS_DIR}/renesas_ra0e2)

# FSP HAL library (Renesas BSP + generated code)
add_library(fsp_hal OBJECT
    # Common FSP BSP sources (Tier 1: shared across all RA families)
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_clocks.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_common.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_delay.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_io.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_irq.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_register_protection.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_security.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_guard.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_group_irq.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_sbrk.c

    # Common FSP drivers (Tier 1: shared across all RA families)
    ${FSP_COMMON_DIR}/fsp/src/r_ioport/r_ioport.c
    ${FSP_COMMON_DIR}/fsp/src/r_uarta/r_uarta.c
    ${FSP_COMMON_DIR}/fsp/src/r_rtc_c/r_rtc_c.c
    ${FSP_COMMON_DIR}/fsp/src/r_tml/r_tml.c
    ${FSP_COMMON_DIR}/fsp/src/r_tau/r_tau.c
    ${FSP_COMMON_DIR}/fsp/src/r_sau_spi/r_sau_spi.c
    ${FSP_COMMON_DIR}/fsp/src/r_icu/r_icu.c
    ${FSP_COMMON_DIR}/fsp/src/r_flash_lp/r_flash_lp.c
    ${FSP_COMMON_DIR}/fsp/src/r_lpm/r_lpm.c

    # RA0E2-specific FSP drivers (Tier 2: RA0E2 MCU family)
    ${RA0E2_DIR}/fsp/r_iwdt/r_iwdt.c

    # RA0E2-specific BSP sources (Tier 2: RA0E2 MCU family)
    ${RA0E2_DIR}/bsp/mcu/ra0e2/bsp_linker.c

    # Renesas CMSIS Device sources
    ${RA0E2_DIR}/cmsis/Device/RENESAS/Source/startup.c
    ${RA0E2_DIR}/cmsis/Device/RENESAS/Source/system.c

    # FSP generated sources (from e2studio)
    ${RA0E2_DIR}/ra_gen/common_data.c
    ${RA0E2_DIR}/ra_gen/hal_data.c
    ${RA0E2_DIR}/ra_gen/pin_data.c
    ${RA0E2_DIR}/ra_gen/vector_data.c

    # Board-specific sources (Tier 3: FPB-RA0E2 board)
    ${RA0E2_DIR}/board/ra0e2_fpb/board_init.c
    ${RA0E2_DIR}/board/ra0e2_fpb/board_leds.c
)

target_compile_definitions(fsp_hal PRIVATE
    _RENESAS_RA_
    _RA
    FPB_RA0E2
)

# Suppress pedantic warnings for Renesas FSP vendor code
# (FSP uses 64-bit enum values not allowed in C99/C11)
target_compile_options(fsp_hal PRIVATE
    -Wno-pedantic
)

target_include_directories(fsp_hal PUBLIC
    # Tier 1: Common CMSIS
    ${CMSIS_DIR}/Core/Include

    # Tier 1: Common FSP BSP and API
    ${FSP_COMMON_DIR}/bsp/mcu/all
    ${FSP_COMMON_DIR}/fsp/inc
    ${FSP_COMMON_DIR}/fsp/inc/api
    ${FSP_COMMON_DIR}/fsp/inc/instances
    ${FSP_COMMON_DIR}/fsp/src/bsp/mcu/all
    ${FSP_COMMON_DIR}/fsp/src/bsp/cmsis/Device/RENESAS/Include

    # Tier 2: RA0E2-specific
    ${RA0E2_DIR}/bsp/mcu/ra0e2
    ${RA0E2_DIR}/cmsis/Device/RENESAS/Include
    ${RA0E2_DIR}/ra_gen
    ${RA0E2_DIR}/ra_cfg/fsp_cfg/bsp
    ${RA0E2_DIR}/ra_cfg/fsp_cfg
    ${RA0E2_DIR}/fsp/r_iwdt

    # Tier 3: Board-specific
    ${RA0E2_DIR}/board/ra0e2_fpb
)

# Linker script
set(LINKER_SCRIPT ${RA0E2_DIR}/core/fsp.ld)
target_link_options(fsp_hal INTERFACE
    -T${LINKER_SCRIPT}
    -L${RA0E2_DIR}/core
)

# SMTC HAL abstraction layer
# NOTE: Includes FSP HAL sources directly (matches e2studio approach)
# This ensures FSP object files are linked directly to executable,
# not through a static library, avoiding --gc-sections issues
add_library(smtc_hal OBJECT
    # SMTC HAL implementation
    smtc_hal_gpio.c
    smtc_hal_lp_timer.c
    smtc_hal_libc_stub.c
    smtc_hal_mcu.c
    smtc_hal_rng.c
    smtc_hal_rtc.c
    smtc_hal_spi.c
    smtc_hal_tau.c
    smtc_hal_flash.c
    smtc_hal_trace.c
    smtc_hal_uart.c
    smtc_hal_watchdog.c
    smtc_hal_irq.c
    smtc_hal_button.c
    smtc_hal_led.c

    # FSP peripheral callbacks
    fsp_callbacks.c

    # FSP HAL sources (from fsp_hal) - linked directly like e2studio
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_clocks.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_common.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_delay.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_io.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_irq.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_register_protection.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_security.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_guard.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_group_irq.c
    ${FSP_COMMON_DIR}/bsp/mcu/all/bsp_sbrk.c
    ${FSP_COMMON_DIR}/fsp/src/r_ioport/r_ioport.c
    ${FSP_COMMON_DIR}/fsp/src/r_uarta/r_uarta.c
    ${FSP_COMMON_DIR}/fsp/src/r_rtc_c/r_rtc_c.c
    ${FSP_COMMON_DIR}/fsp/src/r_tml/r_tml.c
    ${FSP_COMMON_DIR}/fsp/src/r_tau/r_tau.c
    ${FSP_COMMON_DIR}/fsp/src/r_sau_spi/r_sau_spi.c
    ${FSP_COMMON_DIR}/fsp/src/r_icu/r_icu.c
    ${FSP_COMMON_DIR}/fsp/src/r_flash_lp/r_flash_lp.c
    ${FSP_COMMON_DIR}/fsp/src/r_lpm/r_lpm.c
    ${RA0E2_DIR}/fsp/r_iwdt/r_iwdt.c
    ${RA0E2_DIR}/bsp/mcu/ra0e2/bsp_linker.c
    ${RA0E2_DIR}/cmsis/Device/RENESAS/Source/startup.c
    ${RA0E2_DIR}/cmsis/Device/RENESAS/Source/system.c
    ${RA0E2_DIR}/ra_gen/common_data.c
    ${RA0E2_DIR}/ra_gen/hal_data.c
    ${RA0E2_DIR}/ra_gen/pin_data.c
    ${RA0E2_DIR}/ra_gen/vector_data.c
    ${RA0E2_DIR}/board/ra0e2_fpb/board_init.c
    ${RA0E2_DIR}/board/ra0e2_fpb/board_leds.c
)

target_compile_definitions(smtc_hal PRIVATE
    _RENESAS_RA_
    _RA
    FPB_RA0E2
)

# Suppress pedantic warnings for both SMTC and FSP code
target_compile_options(smtc_hal PRIVATE
    -Wno-pedantic
)

# Include directories from fsp_hal
target_include_directories(smtc_hal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    # FSP includes
    ${CMSIS_DIR}/Core/Include
    ${FSP_COMMON_DIR}/bsp/mcu/all
    ${FSP_COMMON_DIR}/fsp/inc
    ${FSP_COMMON_DIR}/fsp/inc/api
    ${FSP_COMMON_DIR}/fsp/inc/instances
    ${FSP_COMMON_DIR}/fsp/src/bsp/mcu/all
    ${FSP_COMMON_DIR}/fsp/src/bsp/cmsis/Device/RENESAS/Include
    ${RA0E2_DIR}/bsp/mcu/ra0e2
    ${RA0E2_DIR}/cmsis/Device/RENESAS/Include
    ${RA0E2_DIR}/ra_gen
    ${RA0E2_DIR}/ra_cfg/fsp_cfg/bsp
    ${RA0E2_DIR}/ra_cfg/fsp_cfg
    ${RA0E2_DIR}/fsp/r_iwdt
    ${RA0E2_DIR}/board/ra0e2_fpb
)

# Link fsp_hal for linker script and flags
target_link_libraries(smtc_hal PUBLIC fsp_hal)

# Optional: Disable HAL debug trace (saves ~5.7KB flash)
# Enable with -DHAL_DBG_TRACE=OFF
option(HAL_DBG_TRACE "Enable HAL debug trace output" ON)
if(NOT HAL_DBG_TRACE)
    target_compile_definitions(smtc_hal PUBLIC HAL_DBG_TRACE=0)
    message(STATUS "HAL_DBG_TRACE: OFF (saves ~5.7KB flash)")
endif()

# Low power mode and debug probe settings are hardcoded in smtc_hal_mcu.c
# (matches STM32 pattern). Override via compiler flags if needed:
#   -DCMAKE_C_FLAGS="-DHW_DEBUG_PROBE=1"    # Keep peripherals for debug
#   -DCMAKE_C_FLAGS="-DLOW_POWER_MODE=0"    # Disable software standby
