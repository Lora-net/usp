# SPDX-License-Identifier: BSD-3-Clause-Clear
# One-Channel-Hub Example (Linux only) -- see README.md

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "1-channel LoRaWAN hub: gateway + client sharing one radio (Linux only)")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(one_channel_hub LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")

# LBM is used in this sample
set(INCLUDE_LBM ON CACHE BOOL "")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Platform compatibility check
#
# The hub is a Linux application (POSIX threads, SPI/GPIO char devices, UDP packet
# forwarder). Skip it on any non-Linux board so the all_examples build on MCU
# targets does not attempt to build it.

if(NOT (BOARD STREQUAL "LINUX" OR BOARD STREQUAL "LINUX_ARM" OR BOARD STREQUAL "LINUX_ARM64"))
    message(STATUS "Skipping one_channel_hub example (Linux only, got BOARD=${BOARD})")
    return()
endif()

################################################################################
# Radio compatibility check
#
# The hub drives a physical radio through libloragw/RAC, so it supports the
# SX126x, LR11xx and LR20xx families. The virtual radio (udp_pf) is not supported.
# (Radio family lists are defined in common.cmake.)

if(NOT (RAC_RADIO IN_LIST sx126x_radios OR
        RAC_RADIO IN_LIST lr11xx_radios OR
        RAC_RADIO IN_LIST lr20xx_radios))
    message(STATUS "Skipping one_channel_hub example (needs sx126x/lr11xx/lr20xx radio, got ${RAC_RADIO})")
    return()
endif()

################################################################################
# Sources

set(ONE_CHANNEL_HUB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_one_channel_hub.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lorawan_client_conf.c

    # RAC serialization lock: strong override of the weak smtc_modem_hal protect
    # hooks (recursive pthread mutex) so all threads serialize on the radio planner.
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_modem_hal_protect.c

    # libloragw (gateway HAL implemented on top of RAC)
    ${CMAKE_CURRENT_SOURCE_DIR}/libloragw/src/loragw_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libloragw/src/loragw_aux.c

    # packet_forwarder (Semtech UDP packet forwarder + jitqueue + parson JSON)
    ${CMAKE_CURRENT_SOURCE_DIR}/packet_forwarder/src/lora_pkt_fwd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/packet_forwarder/src/jitqueue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/packet_forwarder/src/parson.c
    ${CMAKE_CURRENT_SOURCE_DIR}/packet_forwarder/src/base64.c
)

################################################################################
# Target

add_executable(one_channel_hub ${ONE_CHANNEL_HUB_SOURCES})
setup_example_target(one_channel_hub)
register_example(one_channel_hub "${DESCRIPTION}")

################################################################################
# Example-specific configuration

# The hub is multi-threaded (packet-forwarder up/down/jit threads plus the Linux
# HAL radio-IRQ and LP-timer threads). common.cmake sets
# CMAKE_LINK_LIBRARIES_ONLY_TARGETS, so we must link the Threads:: imported target
# rather than a bare -lpthread.
find_package(Threads REQUIRED)
target_link_libraries(one_channel_hub PUBLIC Threads::Threads)

add_library(toolchain::m INTERFACE IMPORTED)
set_property(TARGET toolchain::m PROPERTY IMPORTED_LIBNAME "m")
target_link_libraries(one_channel_hub PRIVATE toolchain::m)

target_include_directories(one_channel_hub PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/libloragw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/packet_forwarder/inc
)

# common.cmake applies LINUX_PLATFORM narrowly (to smtc_ral only). If the hub or its
# libloragw/packet-forwarder code relies on `#if defined(LINUX_PLATFORM)`, define it
# for this target as well. Uncomment if needed:
# target_compile_definitions(one_channel_hub PRIVATE LINUX_PLATFORM)
