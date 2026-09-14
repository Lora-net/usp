# SPDX-License-Identifier: BSD-3-Clause-Clear
# Full Almanac Update Example (LR11XX only)

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Full almanac update (LR11XX only)")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../toolchain.cmake)
project(full_almanac_update LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")

################################################################################
# Example Requirements (set BEFORE including common.cmake)

set(LBM_GEOLOCATION ON CACHE BOOL "")

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../common.cmake)

################################################################################
# Radio compatibility check

if(NOT RAC_RADIO IN_LIST lr11xx_radios)
    message(STATUS "Skipping full_almanac_update example (requires LR11XX radio, got ${RAC_RADIO})")
    return()
endif()

################################################################################
# Create target

set(FULL_ALMANAC_UPDATE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_full_almanac_update.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_full_almanac_update.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../geoloc_example/lr11xx_board/lr1110_board_mb1lxks.c
)

add_executable(full_almanac_update ${FULL_ALMANAC_UPDATE_SOURCES})

setup_example_target(full_almanac_update)
register_example(full_almanac_update "${DESCRIPTION}")

################################################################################
# Example-specific configuration

target_include_directories(full_almanac_update PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../geoloc_example/lr11xx_board
)