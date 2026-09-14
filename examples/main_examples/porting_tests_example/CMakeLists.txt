# SPDX-License-Identifier: BSD-3-Clause-Clear
# Porting Tests Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "HAL porting verification tests")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(porting_tests LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create target

add_executable(porting_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/main_porting_tests.c
)

setup_example_target(porting_tests)
register_example(porting_tests "${DESCRIPTION}")