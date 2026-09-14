# SPDX-License-Identifier: BSD-3-Clause-Clear
# CAD (Channel Activity Detection) Example (LR20XX only)

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Channel Activity Detection (LR20XX only)")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(cad LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Radio compatibility check

if(NOT RAC_RADIO IN_LIST lr20xx_radios)
    message(STATUS "Skipping cad example (requires LR20XX radio, got ${RAC_RADIO})")
    return()
endif()

################################################################################
# Options

set(TYPES_OF_CAD
    "RAL_LORA_CAD_ONLY"
    "RAL_LORA_CAD_RX"
    "RAL_LORA_CAD_LBT"
)
set(TYPE_OF_CAD "RAL_LORA_CAD_LBT" CACHE STRING "Action after CAD is finished (${TYPES_OF_CAD})")
if(NOT TYPE_OF_CAD IN_LIST TYPES_OF_CAD)
    message(FATAL_ERROR "Invalid TYPE_OF_CAD: ${TYPE_OF_CAD} (possible values: ${TYPES_OF_CAD})")
endif()

################################################################################
# Create target

set(CAD_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_cad.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_cad.c
)

add_executable(cad ${CAD_SOURCES})
setup_example_target(cad)
register_example(cad "${DESCRIPTION}")

target_include_directories(cad PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(cad PRIVATE TYPE_OF_CAD=${TYPE_OF_CAD})