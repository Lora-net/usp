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

# LBM is used in this sample
set(INCLUDE_LBM ON CACHE BOOL "")

# HW Modem uses the FLRC protocol (disabled by default & include a 20KB buffer for the payload)
set(USE_FLRC_PROTOCOL OFF CACHE BOOL "")

include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create target

option(HW_MODEM_USE_BRIDGE "Use GPIO+UART4 physical bridge transport" OFF)

# Select hw_modem transport layer:
#   Linux           -> PTY (set BOARD to a linux variant) (see hw_modem_linux.c)
#   Software bridge -> single USART2 (set HW_MODEM_USE_BRIDGE=OFF) (see hw_modem_serial.c)
#   Physical bridge -> GPIO + UART4 (set HW_MODEM_USE_BRIDGE=ON) (see hw_modem.c)
if("${BOARD}" STREQUAL "LINUX" OR "${BOARD}" STREQUAL "LINUX_ARM" OR "${BOARD}" STREQUAL "LINUX_ARM64")
    set(HW_MODEM_TRANSPORT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/hw_modem_linux.c")
elseif(HW_MODEM_USE_BRIDGE)
    set(HW_MODEM_TRANSPORT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/hw_modem.c")
else()
    set(HW_MODEM_TRANSPORT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/hw_modem_serial.c")
    # Single-UART transport multiplexes hw_modem data with traces on the trace UART: the HAL runs
    # the circular DMA RX into hw_modem's own ring buffer and only exposes the producer write index
    # (hal_uart_rx_start/_write_index); hw_modem owns the ring buffer, consume logic and framing.
    # The physical-bridge transport (hw_modem.c) doesn't need it.
    target_compile_definitions(smtc_hal PUBLIC HAL_UART_RX_ENABLED)
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

# WFI only — keep USART2/DMA running during hal_mcu_set_sleep_for_ms()
set(LOW_POWER_MODE 0 CACHE STRING "HAL low power: 0=WFI, 1=STOP2")
if(LOW_POWER_MODE EQUAL 1)
    message(FATAL_ERROR "hw_modem requires LOW_POWER_MODE != 1")
endif()
target_compile_definitions(smtc_hal PUBLIC LOW_POWER_MODE=${LOW_POWER_MODE})

if(LBM_CRYPTO MATCHES "^LR11XX")
    target_compile_definitions(hw_modem PRIVATE USE_LR11XX_CE)
endif()

if(USE_FLRC_PROTOCOL)
    target_compile_definitions(hw_modem PRIVATE USE_FLRC_PROTOCOL)
endif()

target_include_directories(hw_modem PRIVATE
    ${EXAMPLES_DIR}/../protocols/lbm_lib/smtc_modem_core/geolocation_services
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/nanopb
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/generated
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization/helpers
)
