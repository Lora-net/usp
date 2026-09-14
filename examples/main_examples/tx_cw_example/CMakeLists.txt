# SPDX-License-Identifier: BSD-3-Clause-Clear
# TX CW (Continuous Wave) Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Continuous wave (TX CW) transmission")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(tx_cw LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Options

option(INFINITE_PREAMBLE "Enable INFINITE_PREAMBLE" OFF)

################################################################################
# Create target

set(TX_CW_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_tx_cw.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_tx_cw.c
)

add_executable(tx_cw ${TX_CW_SOURCES})
setup_example_target(tx_cw)
register_example(tx_cw "${DESCRIPTION}")

target_include_directories(tx_cw PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(INFINITE_PREAMBLE)
    target_compile_definitions(tx_cw PRIVATE INFINITE_PREAMBLE)
    message(STATUS "tx_cw sample - INFINITE PREAMBLE ENABLED")
else()
    message(STATUS "tx_cw sample - CONTINUOUS WAVE ENABLED")
endif()