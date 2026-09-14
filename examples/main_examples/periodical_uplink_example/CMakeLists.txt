# SPDX-License-Identifier: BSD-3-Clause-Clear
# Periodical Uplink Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Periodical LoRaWAN uplink example")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(periodical_uplink LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create target

add_executable(periodical_uplink
    ${CMAKE_CURRENT_SOURCE_DIR}/main_periodical_uplink.c
)

setup_example_target(periodical_uplink)
register_example(periodical_uplink "${DESCRIPTION}")