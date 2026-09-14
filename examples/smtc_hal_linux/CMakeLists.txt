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

message(STATUS "Linux HAL configured")

