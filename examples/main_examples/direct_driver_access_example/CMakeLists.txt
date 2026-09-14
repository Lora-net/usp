# SPDX-License-Identifier: BSD-3-Clause-Clear
# Direct Driver Access Example (LR20XX only)

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Direct radio driver access (LR20XX only)")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(direct_driver_access LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Radio compatibility check

if(NOT RAC_RADIO IN_LIST lr20xx_radios)
    message(STATUS "Skipping direct_driver_access example (requires LR20XX radio, got ${RAC_RADIO})")
    return()
endif()

################################################################################
# Create target

set(DIRECT_DRIVER_ACCESS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_direct_driver_access.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_direct_driver_access.c
)

add_executable(direct_driver_access ${DIRECT_DRIVER_ACCESS_SOURCES})
setup_example_target(direct_driver_access)
register_example(direct_driver_access "${DESCRIPTION}")

target_include_directories(direct_driver_access PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})