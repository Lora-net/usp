# SPDX-License-Identifier: BSD-3-Clause-Clear
# RTToF (Ranging) Examples

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Ranging (RTToF)")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(rttof LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Options

option(PERIODIC_UPLINK "Enable the periodic uplink" ON)

################################################################################
# Create targets

set(RTTOF_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_ranging_demo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_ranging_hopping.c
)

# Manager
add_executable(rttof_manager ${RTTOF_SOURCES})
setup_example_target(rttof_manager)
register_example(rttof_manager "${DESCRIPTION} manager")
target_compile_definitions(rttof_manager PRIVATE
    RANGING_DEVICE_MODE=RANGING_DEVICE_MODE_MANAGER
)

# Subordinate
add_executable(rttof_subordinate ${RTTOF_SOURCES})
setup_example_target(rttof_subordinate)
register_example(rttof_subordinate "${DESCRIPTION} subordinate")
target_compile_definitions(rttof_subordinate PRIVATE
    RANGING_DEVICE_MODE=RANGING_DEVICE_MODE_SUBORDINATE
)

if(PERIODIC_UPLINK)
    target_compile_definitions(rttof_manager PRIVATE ENABLE_PERIODIC_UPLINK)
    target_compile_definitions(rttof_subordinate PRIVATE ENABLE_PERIODIC_UPLINK)
endif()

# Build both targets
add_custom_target(rttof DEPENDS rttof_manager rttof_subordinate)