# SPDX-License-Identifier: BSD-3-Clause-Clear
# Spectral Scan Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Spectral scan analysis example")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(spectral_scan LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create target

set(SPECTRAL_SCAN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_spectral_scan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_spectral_scan.c
)

add_executable(spectral_scan ${SPECTRAL_SCAN_SOURCES})
setup_example_target(spectral_scan)
register_example(spectral_scan "${DESCRIPTION}")

target_include_directories(spectral_scan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})