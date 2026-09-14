# SPDX-License-Identifier: BSD-3-Clause-Clear
# FSK PER (Packet Error Rate) Examples

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Packet error rate - FSK")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(per_fsk LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create targets

set(PER_FSK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_per_fsk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_per_fsk.c
)

# Transmitter
add_executable(per_fsk_tx ${PER_FSK_SOURCES})
setup_example_target(per_fsk_tx)
register_example(per_fsk_tx "${DESCRIPTION} (transmitter)")
target_compile_definitions(per_fsk_tx PRIVATE ROLE=TRANSMITTER)
target_include_directories(per_fsk_tx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Receiver
add_executable(per_fsk_rx ${PER_FSK_SOURCES})
setup_example_target(per_fsk_rx)
register_example(per_fsk_rx "${DESCRIPTION} (receiver)")
target_compile_definitions(per_fsk_rx PRIVATE ROLE=RECEIVER)
target_include_directories(per_fsk_rx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Build both targets
add_custom_target(per_fsk DEPENDS per_fsk_tx per_fsk_rx)