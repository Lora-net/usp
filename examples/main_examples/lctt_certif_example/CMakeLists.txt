# SPDX-License-Identifier: BSD-3-Clause-Clear
# LCTT Certification Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "LCTT LoRaWAN certification example")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(lctt_certif LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create target

add_executable(lctt_certif
    ${CMAKE_CURRENT_SOURCE_DIR}/main_lctt_certif.c
)

setup_example_target(lctt_certif)
register_example(lctt_certif "${DESCRIPTION}")