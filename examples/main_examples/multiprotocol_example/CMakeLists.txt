# SPDX-License-Identifier: BSD-3-Clause-Clear
# Multiprotocol Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Multiprotocol (LoRaWAN + ranging) example")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(multiprotocol LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create target

set(MULTIPROTOCOL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_multiprotocol.c
    ${EXAMPLES_DIR}/main_examples/ranging_demo/app_ranging_hopping.c
)

add_executable(multiprotocol ${MULTIPROTOCOL_SOURCES})
setup_example_target(multiprotocol)
register_example(multiprotocol "${DESCRIPTION}")

target_include_directories(multiprotocol PRIVATE
    ${EXAMPLES_DIR}/main_examples/ranging_demo
)