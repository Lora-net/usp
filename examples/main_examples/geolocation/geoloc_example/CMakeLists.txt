# SPDX-License-Identifier: BSD-3-Clause-Clear
# Geolocation Example (LR11XX only)

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Geolocation example (LR11XX only)")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../toolchain.cmake)
project(geolocation LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")

################################################################################
# Example Requirements (set BEFORE including common.cmake)

# Geolocation requires full LBM features
set(ENABLE_FULL_LBM_FEATURES ON CACHE BOOL "")
set(LBM_GEOLOCATION ON CACHE BOOL "")
set(LBM_ALMANAC OFF CACHE BOOL "")

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../common.cmake)

################################################################################
# Radio compatibility check

if(NOT RAC_RADIO IN_LIST lr11xx_radios)
    message(STATUS "Skipping geolocation example (requires LR11XX radio, got ${RAC_RADIO})")
    return()
endif()

################################################################################
# Create target

set(GEOLOCATION_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_geolocation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lr11xx_board/lr1110_board_mb1lxks.c
)

add_executable(geolocation ${GEOLOCATION_SOURCES})

setup_example_target(geolocation)
register_example(geolocation "${DESCRIPTION}")

################################################################################
# Example-specific configuration

target_include_directories(geolocation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lr11xx_board
)

# Enable GNSS feature in HAL and radio_hal
target_compile_definitions(radio_hal PUBLIC ENABLE_MODEM_GNSS_FEATURE)
target_compile_definitions(smtc_hal PUBLIC ENABLE_MODEM_GNSS_FEATURE)