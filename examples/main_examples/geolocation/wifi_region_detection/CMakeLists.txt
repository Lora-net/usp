# SPDX-License-Identifier: BSD-3-Clause-Clear
# WiFi Region Detection Example (LR11XX only)

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "WiFi region detection (LR11XX only)")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../toolchain.cmake)
project(wifi_region_detection LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")

################################################################################
# Example Requirements (set BEFORE including common.cmake)

set(LBM_GEOLOCATION ON CACHE BOOL "")

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../common.cmake)

################################################################################
# Radio compatibility check

if(NOT RAC_RADIO IN_LIST lr11xx_radios)
    message(STATUS "Skipping wifi_region_detection example (requires LR11XX radio, got ${RAC_RADIO})")
    return()
endif()

################################################################################
# Create target

set(WIFI_REGION_DETECTION_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_wifi_region_detection.c
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi_region_finder.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../geoloc_example/lr11xx_board/lr1110_board_mb1lxks.c
)

add_executable(wifi_region_detection ${WIFI_REGION_DETECTION_SOURCES})

setup_example_target(wifi_region_detection)
register_example(wifi_region_detection "${DESCRIPTION}")

################################################################################
# Example-specific configuration

target_include_directories(wifi_region_detection PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../geoloc_example/lr11xx_board
)