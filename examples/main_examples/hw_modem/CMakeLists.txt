# SPDX-License-Identifier: BSD-3-Clause-Clear
# HW Modem Example

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "Hardware modem with serial interface")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(hw_modem LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")

################################################################################
# Example Requirements (set BEFORE including common.cmake)

# HW Modem requires full LBM features
set(ENABLE_FULL_LBM_FEATURES ON CACHE BOOL "")
set(LBM_ALMANAC OFF CACHE BOOL "")

# HW Modem uses the FLRC protocol (disabled by default & include a 20KB buffer for the payload)
set(USE_FLRC_PROTOCOL OFF CACHE BOOL "")

include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create target

# Select hw_modem transport layer: PTY-based on Linux, GPIO+UART+DMA on embedded
if("${BOARD}" STREQUAL "LINUX" OR "${BOARD}" STREQUAL "LINUX_ARM" OR "${BOARD}" STREQUAL "LINUX_ARM64")
    set(HW_MODEM_TRANSPORT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/hw_modem_linux.c")
else()
    set(HW_MODEM_TRANSPORT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/hw_modem.c")
endif()

set(HW_MODEM_SOURCES
    ${HW_MODEM_TRANSPORT_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/git_version.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cmd_parser.c
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/generated/pb_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/generated/pb_decode.c
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/generated/pb_encode.c
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/generated/smtc_rac_context.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/helpers/rac_context_converter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_hw_modem.c
)

if(LBM_GEOLOCATION)
    list(APPEND HW_MODEM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../geolocation/geoloc_example/lr11xx_board/lr1110_board_mb1lxks.c
    )
endif()

add_executable(hw_modem ${HW_MODEM_SOURCES})

setup_example_target(hw_modem)
register_example(hw_modem "${DESCRIPTION}")

################################################################################
# Example-specific configuration

# HW modem requires this HAL definition
target_compile_definitions(smtc_hal PUBLIC HW_MODEM_ENABLED)

if(LBM_CRYPTO MATCHES "^LR11XX")
    target_compile_definitions(hw_modem PRIVATE USE_LR11XX_CE)
endif()

if(USE_FLRC_PROTOCOL)
    target_compile_definitions(hw_modem PRIVATE USE_FLRC_PROTOCOL)
    target_link_libraries(hw_modem PUBLIC smtc_flrc_protocol)
endif()

target_include_directories(hw_modem PRIVATE
    ${EXAMPLES_DIR}/../protocols/lbm_lib/smtc_modem_core/geolocation_services
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/nanopb
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/generated
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/helpers
)
