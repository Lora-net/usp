# SPDX-License-Identifier: BSD-3-Clause-Clear
# FHSS Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "LR20xx LoRa intra-packet frequency hopping ping-pong example")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(fhss_example LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Check radio compatibility

if(NOT RAC_RADIO IN_LIST lr20xx_radios)
    message(STATUS "Skipping fhss_example (requires LR20XX radio, got ${RAC_RADIO})")
    return()
endif()

################################################################################
# Create target

set(FHSS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_fhss_example.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_fhss_example.c
)

add_executable(fhss_example ${FHSS_SOURCES})
setup_example_target(fhss_example)
register_example(fhss_example "${DESCRIPTION}")

target_include_directories(fhss_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
