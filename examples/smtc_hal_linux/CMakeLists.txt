# SPDX-License-Identifier: BSD-3-Clause-Clear

cmake_minimum_required(VERSION 3.10)

################################################################################
# Linux HAL Library
################################################################################

add_library(smtc_hal OBJECT
    smtc_hal_button.c
    smtc_hal_gpio.c
    smtc_hal_led.c
    smtc_hal_spi.c
    smtc_hal_mcu.c
    smtc_hal_rtc.c
    smtc_hal_lp_timer.c
    smtc_hal_rng.c
    smtc_hal_watchdog.c
    smtc_hal_trace.c
    smtc_hal_flash.c
)

target_include_directories(smtc_hal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..  # For modem_pinout.h
)

# For smtc_modem_hal.h, which declares the RAC critical-section hooks
# (smtc_modem_hal_protect_api_call / _unprotect_api_call) entered by the IRQ/timer
# delivery threads to serialize with the RAC engine.
target_include_directories(smtc_hal PRIVATE
    ${EXAMPLES_DIR}/../smtc_rac_lib/smtc_modem_hal
)

target_compile_definitions(smtc_hal PUBLIC
    LINUX_PLATFORM
    _DEFAULT_SOURCE
    _POSIX_C_SOURCE=199309L
)

# Find required packages
find_package(Threads REQUIRED)

# Link required Linux libraries
# Use INTERFACE to avoid target constraints
target_link_libraries(smtc_hal INTERFACE
    Threads::Threads  # For threading support
    $<LINK_ONLY:-lrt>  # For POSIX timers
)

# Compiler options
target_compile_options(smtc_hal PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
)

# Optional: log LP-timer arm->fire latency/jitter (for timer-accuracy validation)
option(LP_TIMER_LATENCY_LOG "Log LP-timer arm->fire latency/jitter" OFF)
if(LP_TIMER_LATENCY_LOG)
    target_compile_definitions(smtc_hal PRIVATE LP_TIMER_LATENCY_LOG)
    message(STATUS "Linux HAL: LP-timer latency logging ENABLED")
endif()

message(STATUS "Linux HAL configured")

