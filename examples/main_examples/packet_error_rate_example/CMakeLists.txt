# SPDX-License-Identifier: BSD-3-Clause-Clear
# PER (Packet Error Rate) Examples - LoRa

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Packet error rate - LoRa")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(per LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create targets

set(PER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_per.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_per.c
)

# Transmitter
add_executable(per_tx ${PER_SOURCES})
setup_example_target(per_tx)
register_example(per_tx "${DESCRIPTION} (transmitter)")
target_compile_definitions(per_tx PRIVATE ROLE=TRANSMITTER)

# Receiver
add_executable(per_rx ${PER_SOURCES})
setup_example_target(per_rx)
register_example(per_rx "${DESCRIPTION} (receiver)")
target_compile_definitions(per_rx PRIVATE ROLE=RECEIVER)

# Build both targets
add_custom_target(per DEPENDS per_tx per_rx)