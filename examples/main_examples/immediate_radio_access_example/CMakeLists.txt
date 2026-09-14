# SPDX-License-Identifier: BSD-3-Clause-Clear
# Immediate Radio Access Example (LR20XX only)

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Immediate radio access (LR20XX only)")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(immediate_radio_access LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Radio compatibility check

if(NOT RAC_RADIO IN_LIST lr20xx_radios)
    message(STATUS "Skipping immediate_radio_access example (requires LR20XX radio, got ${RAC_RADIO})")
    return()
endif()

################################################################################
# Create target

set(IMMEDIATE_RADIO_ACCESS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_immediate_radio_access.c
    ${CMAKE_CURRENT_SOURCE_DIR}/immediate_radio_access.c
)

add_executable(immediate_radio_access ${IMMEDIATE_RADIO_ACCESS_SOURCES})
setup_example_target(immediate_radio_access)
register_example(immediate_radio_access "${DESCRIPTION}")

target_include_directories(immediate_radio_access PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})