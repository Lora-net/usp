# SPDX-License-Identifier: BSD-3-Clause-Clear
#
# toolchain.cmake - Board configuration and toolchain selection
#
# Include this file BEFORE project() command

# Include guard - only process once
if(TOOLCHAIN_CMAKE_INCLUDED)
    return()
endif()
set(TOOLCHAIN_CMAKE_INCLUDED ON CACHE INTERNAL "")

# Get the directory where this file is located
get_filename_component(EXAMPLES_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
set(EXAMPLES_DIR ${EXAMPLES_DIR} CACHE INTERNAL "Path to examples directory")

################################################################################
# Board Selection, Toolchain, and HAL Paths

set(BOARD "" CACHE STRING "Target board")
set(KNOWN_BOARDS "NUCLEO_L073" "NUCLEO_L476" "LINUX" "LINUX_ARM" "LINUX_ARM64" "FPB_RA0E2")
set_property(CACHE BOARD PROPERTY STRINGS "${KNOWN_BOARDS}")

if(NOT DEFINED BOARD OR BOARD STREQUAL "")
    message(FATAL_ERROR "BOARD must be defined. Valid values: ${KNOWN_BOARDS}")
elseif(BOARD STREQUAL "NUCLEO_L073")
    set(CMAKE_TOOLCHAIN_FILE ${EXAMPLES_DIR}/smtc_hal_l0_LL/cmake_stm32l0_toolchain.cmake)
    set(SMTC_HAL_DIR ${EXAMPLES_DIR}/smtc_hal_l0_LL)
    set(MCU_DRIVERS_DIR ${EXAMPLES_DIR}/mcu_drivers)
elseif(BOARD STREQUAL "NUCLEO_L476")
    set(CMAKE_TOOLCHAIN_FILE ${EXAMPLES_DIR}/smtc_hal_l4/cmake_stm32l4_toolchain.cmake)
    set(SMTC_HAL_DIR ${EXAMPLES_DIR}/smtc_hal_l4)
    set(MCU_DRIVERS_DIR ${EXAMPLES_DIR}/mcu_drivers)
elseif(BOARD STREQUAL "LINUX")
    message(STATUS "Native Linux build (no cross-compilation)")
    set(SMTC_HAL_DIR ${EXAMPLES_DIR}/smtc_hal_linux)
elseif(BOARD STREQUAL "LINUX_ARM")
    set(CMAKE_TOOLCHAIN_FILE ${EXAMPLES_DIR}/smtc_hal_linux/cmake_linux_arm_toolchain.cmake)
    set(SMTC_HAL_DIR ${EXAMPLES_DIR}/smtc_hal_linux)
    message(STATUS "ARM Linux 32-bit cross-compilation enabled")
elseif(BOARD STREQUAL "LINUX_ARM64")
    set(CMAKE_TOOLCHAIN_FILE ${EXAMPLES_DIR}/smtc_hal_linux/cmake_linux_aarch64_toolchain.cmake)
    set(SMTC_HAL_DIR ${EXAMPLES_DIR}/smtc_hal_linux)
    message(STATUS "AArch64 Linux 64-bit cross-compilation enabled")
elseif(BOARD STREQUAL "FPB_RA0E2")
    set(CMAKE_TOOLCHAIN_FILE ${EXAMPLES_DIR}/smtc_hal_ra0e2/cmake_ra0e2_toolchain.cmake)
    set(SMTC_HAL_DIR ${EXAMPLES_DIR}/smtc_hal_ra0e2)
    set(MCU_DRIVERS_DIR ${EXAMPLES_DIR}/mcu_drivers)
else()
    message(FATAL_ERROR "Unknown BOARD: '${BOARD}'. Valid values: ${KNOWN_BOARDS}")
endif()

message(STATUS "BOARD is set to: '${BOARD}'")
message(STATUS "SMTC_HAL_DIR set to: ${SMTC_HAL_DIR}")
message(STATUS "MCU_DRIVERS_DIR set to: ${MCU_DRIVERS_DIR}")