# SPDX-License-Identifier: BSD-3-Clause-Clear
# LR-FHSS Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "LR-FHSS transmission example")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(lrfhss LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create target

set(LRFHSS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_lrfhss_example.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_lrfhss_example.c
)

add_executable(lrfhss_tx ${LRFHSS_SOURCES})
setup_example_target(lrfhss_tx)
register_example(lrfhss_tx "${DESCRIPTION}")

target_include_directories(lrfhss_tx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Generate lrfhss.bin file (alias for lrfhss_tx.bin)
add_custom_command(
    TARGET lrfhss_tx POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/lrfhss_tx.bin" "${CMAKE_BINARY_DIR}/lrfhss.bin"
    COMMENT "Creating lrfhss.bin..."
)