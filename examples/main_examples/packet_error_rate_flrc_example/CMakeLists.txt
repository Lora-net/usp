# SPDX-License-Identifier: BSD-3-Clause-Clear
# FLRC PER (Packet Error Rate) Examples

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Packet error rate - FLRC")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(per_flrc LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create targets

set(PER_FLRC_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_per_flrc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_per_flrc.c
)

# Transmitter
add_executable(per_flrc_tx ${PER_FLRC_SOURCES})
setup_example_target(per_flrc_tx)
register_example(per_flrc_tx "${DESCRIPTION} (transmitter)")
target_compile_definitions(per_flrc_tx PRIVATE ROLE=TRANSMITTER)
target_include_directories(per_flrc_tx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Receiver
add_executable(per_flrc_rx ${PER_FLRC_SOURCES})
setup_example_target(per_flrc_rx)
register_example(per_flrc_rx "${DESCRIPTION} (receiver)")
target_compile_definitions(per_flrc_rx PRIVATE ROLE=RECEIVER)
target_include_directories(per_flrc_rx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Build both targets
add_custom_target(per_flrc DEPENDS per_flrc_tx per_flrc_rx)