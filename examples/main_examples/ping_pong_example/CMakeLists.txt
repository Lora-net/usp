# SPDX-License-Identifier: BSD-3-Clause-Clear
# Ping Pong Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Ping-pong communication example")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(ping_pong LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Options

option(PERIODIC_UPLINK "Enable the periodic uplink" ON)

################################################################################
# Create target

set(PING_PONG_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_ping_pong.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_ping_pong.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_periodic_uplink.c
)

add_executable(ping_pong ${PING_PONG_SOURCES})
setup_example_target(ping_pong)
register_example(ping_pong "${DESCRIPTION}")

if(PERIODIC_UPLINK)
    target_compile_definitions(ping_pong PRIVATE ENABLE_PERIODIC_UPLINK)
endif()