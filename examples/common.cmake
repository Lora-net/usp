# SPDX-License-Identifier: BSD-3-Clause-Clear
#
# common.cmake - Shared configuration for all examples
#
# Include this file AFTER project() command
#
# Usage in example CMakeLists.txt:
#   cmake_minimum_required(VERSION 3.25)
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
#   project(my_example LANGUAGES C ASM)
#   set(LBM_GEOLOCATION ON CACHE BOOL "")  # Optional: set requirements
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

# Include guard - only process once
if(COMMON_CMAKE_INCLUDED)
    return()
endif()
set(COMMON_CMAKE_INCLUDED ON CACHE INTERNAL "")

# EXAMPLES_DIR should be set by toolchain.cmake, but set it here too for safety
if(NOT DEFINED EXAMPLES_DIR)
    get_filename_component(EXAMPLES_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
    set(EXAMPLES_DIR ${EXAMPLES_DIR} CACHE INTERNAL "Path to examples directory")
endif()

################################################################################
# Compiler Settings

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

# Board-specific compile definitions
if(BOARD STREQUAL "FPB_RA0E2")
    add_compile_definitions(FPB_RA0E2)
endif()

# Set default build type
set(default_build_type "MinSizeRel")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Disable fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)

# Enable ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################################################
# Project Options

# LoRa Basics Modem Options
option(ENABLE_FULL_LBM_FEATURES "Enable all LBM features" OFF)

# FLRC protocol
option(USE_FLRC_PROTOCOL "Use the FLRC protocol" OFF)

# Radio Planner Options
set(RP_MARGIN_DELAY "8" CACHE STRING "Radio Planner margin delay in ms")
message(STATUS "RP_MARGIN_DELAY: ${RP_MARGIN_DELAY}")

################################################################################
# Logging Options

option(RAC_LOG_ENABLE "Enable logging system" ON)

# RAC logging controls
option(RAC_CORE_LOG_ERROR_ENABLE "Enable RAC Core ERROR logs" ON)
option(RAC_CORE_LOG_CONFIG_ENABLE "Enable RAC Core CONFIG logs" ON)
option(RAC_CORE_LOG_INFO_ENABLE "Enable RAC Core INFO logs" OFF)
option(RAC_CORE_LOG_WARN_ENABLE "Enable RAC Core WARN logs" OFF)
option(RAC_CORE_LOG_DEBUG_ENABLE "Enable RAC Core DEBUG logs" OFF)
option(RAC_CORE_LOG_API_ENABLE "Enable RAC Core API logs" OFF)
option(RAC_CORE_LOG_RADIO_ENABLE "Enable RAC Core RADIO logs" OFF)

# LoRa logging controls
option(RAC_LORA_LOG_TX_ENABLE "Enable RAC LoRa TX logs" ON)
option(RAC_LORA_LOG_RX_ENABLE "Enable RAC LoRa RX logs" ON)
option(RAC_LORA_LOG_CONFIG_ENABLE "Enable RAC LoRa CONFIG logs" ON)
option(RAC_LORA_LOG_ERROR_ENABLE "Enable RAC LoRa ERROR logs" ON)
option(RAC_LORA_LOG_INFO_ENABLE "Enable RAC LoRa INFO logs" OFF)
option(RAC_LORA_LOG_WARN_ENABLE "Enable RAC LoRa WARN logs" OFF)
option(RAC_LORA_LOG_DEBUG_ENABLE "Enable RAC LoRa DEBUG logs" OFF)

# Predefined logging profiles
set(RAC_LOG_PROFILE "DEFAULT" CACHE STRING "logging profile")
set_property(CACHE RAC_LOG_PROFILE PROPERTY STRINGS "MINIMAL;DEFAULT;VERBOSE;DEBUG;ALL")

# Apply logging profile settings
if(RAC_LOG_PROFILE STREQUAL "MINIMAL")
    message(STATUS "Logging Profile: MINIMAL (only ERROR logs)")
    set(RAC_CORE_LOG_ERROR_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_ERROR_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_CORE_LOG_CONFIG_ENABLE OFF CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_CONFIG_ENABLE OFF CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_TX_ENABLE OFF CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_RX_ENABLE OFF CACHE BOOL "" FORCE)
elseif(RAC_LOG_PROFILE STREQUAL "VERBOSE")
    message(STATUS "Logging Profile: VERBOSE (ERROR, CONFIG, TX, RX, INFO)")
    set(RAC_CORE_LOG_INFO_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_INFO_ENABLE ON CACHE BOOL "" FORCE)
elseif(RAC_LOG_PROFILE STREQUAL "DEBUG")
    message(STATUS "Logging Profile: DEBUG (ERROR, CONFIG, TX, RX, INFO, WARN, DEBUG)")
    set(RAC_CORE_LOG_INFO_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_CORE_LOG_WARN_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_CORE_LOG_DEBUG_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_INFO_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_WARN_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_DEBUG_ENABLE ON CACHE BOOL "" FORCE)
elseif(RAC_LOG_PROFILE STREQUAL "ALL")
    message(STATUS "Logging Profile: ALL (enable all log types)")
    set(RAC_CORE_LOG_INFO_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_CORE_LOG_WARN_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_CORE_LOG_DEBUG_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_CORE_LOG_API_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_CORE_LOG_RADIO_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_INFO_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_WARN_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_DEBUG_ENABLE ON CACHE BOOL "" FORCE)
else()
    message(STATUS "Logging Profile: DEFAULT (ERROR, CONFIG, TX, RX)")
endif()

################################################################################
# Radio Selection

# Set target radio families
set(sx126x_radios sx1261 sx1262 sx1268 CACHE INTERNAL "SX126x radio variants")
set(lr11xx_radios lr1110 lr1120 lr1121 CACHE INTERNAL "LR11xx radio variants")
set(lr20xx_radios lr2012 lr2021 lr2022 CACHE INTERNAL "LR20xx radio variants")
set(udp_pf_radios udp_pf CACHE INTERNAL "UDP packet forwarder (virtual radio)")
set(known_radios ${sx126x_radios} ${lr11xx_radios} ${lr20xx_radios} ${udp_pf_radios})

set(RAC_RADIO "" CACHE STRING "The radio to target")
set_property(CACHE RAC_RADIO PROPERTY STRINGS ${known_radios})
if(NOT DEFINED RAC_RADIO OR RAC_RADIO STREQUAL "")
    message(FATAL_ERROR "You must select the radio with -DRAC_RADIO=")
endif()
if(NOT RAC_RADIO IN_LIST known_radios)
    message(FATAL_ERROR "Unknown radio ${RAC_RADIO}")
endif()

# Set target radio family
if(RAC_RADIO IN_LIST sx126x_radios)
    set(RADIO_FAMILY sx126x)
elseif(RAC_RADIO IN_LIST lr11xx_radios)
    set(RADIO_FAMILY lr11xx)
elseif(RAC_RADIO IN_LIST lr20xx_radios)
    set(RADIO_FAMILY lr20xx)
elseif(RAC_RADIO IN_LIST udp_pf_radios)
    set(RADIO_FAMILY udp_pf)
endif()
string(TOUPPER ${RADIO_FAMILY} RADIO_FAMILY_UPPER)

# Radio-specific options
if(RAC_RADIO IN_LIST lr20xx_radios)
    option(LEGACY_EVK_LR20XX "Enable LR20XX shield" OFF)
    message(STATUS "Legacy (mbed) LR20XX shield: ${LEGACY_EVK_LR20XX}")
endif()
if(RAC_RADIO IN_LIST lr11xx_radios)
    option(USE_LR11XX_CRC_SPI "Enable CRC over SPI" OFF)
endif()

message(STATUS "Selected radio ${RAC_RADIO} in family ${RADIO_FAMILY}\n")

################################################################################
# Apply LBM features if requested (before including libraries)

if(NOT DEFINED LBM_RADIO)
    set(LBM_RADIO ${RAC_RADIO})
endif()
if(NOT LBM_RADIO STREQUAL ${RAC_RADIO})
    message(FATAL_ERROR "LBM_RADIO (${LBM_RADIO}) must match RAC_RADIO (${RAC_RADIO})")
endif()

if(ENABLE_FULL_LBM_FEATURES)
    set(LBM_CMAKE_CONFIG_AUTO ON CACHE BOOL "")
    set(LBM_ALC_SYNC ON CACHE BOOL "")
    set(LBM_CLASS_B ON CACHE BOOL "")
    set(LBM_CLASS_C ON CACHE BOOL "")
    set(LBM_MULTICAST ON CACHE BOOL "")
    set(LBM_CSMA ON CACHE BOOL "")
    set(LBM_RELAY_RX ON CACHE BOOL "")
    set(LBM_RELAY_TX ON CACHE BOOL "")
    set(LBM_STREAM ON CACHE BOOL "")
    set(LBM_DEVICE_MANAGEMENT ON CACHE BOOL "")
    set(LBM_LFU ON CACHE BOOL "")
    set(LBM_FUOTA ON CACHE BOOL "")
    set(LBM_FUOTA_MPA ON CACHE BOOL "")
    set(LBM_STORE_AND_FORWARD ON CACHE BOOL "")
    # Geolocation is only supported on LR11XX radios
    if(RAC_RADIO IN_LIST lr11xx_radios)
        set(LBM_GEOLOCATION ON CACHE BOOL "")
    endif()
endif()

################################################################################
# Include HAL and Libraries

add_subdirectory(${SMTC_HAL_DIR} smtc_hal_build)
add_subdirectory(${EXAMPLES_DIR}/../smtc_rac_lib smtc_rac_lib)

# Apply logging configuration to smtc_rac target
if(TARGET smtc_rac)
    # Add LINUX_PLATFORM definition for ral.h when building for Linux
    if("${BOARD}" STREQUAL "LINUX" OR "${BOARD}" STREQUAL "LINUX_ARM" OR "${BOARD}" STREQUAL "LINUX_ARM64")
        if(TARGET smtc_ral_${RADIO_FAMILY})
            target_compile_definitions(smtc_ral_${RADIO_FAMILY} PUBLIC LINUX_PLATFORM)
        endif()
    endif()

    # Apply Radio Planner options
    target_compile_definitions(smtc_rac PRIVATE RP_MARGIN_DELAY=${RP_MARGIN_DELAY})

    # Apply RAC logging options
    if(RAC_CORE_LOG_ERROR_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_CORE_LOG_ERROR_ENABLE=1)
    endif()
    if(RAC_CORE_LOG_CONFIG_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_CORE_LOG_CONFIG_ENABLE=1)
    endif()
    if(RAC_CORE_LOG_INFO_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_CORE_LOG_INFO_ENABLE=0)
    endif()
    if(RAC_CORE_LOG_WARN_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_CORE_LOG_WARN_ENABLE=0)
    endif()
    if(RAC_CORE_LOG_DEBUG_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_CORE_LOG_DEBUG_ENABLE=0)
    endif()
    if(RAC_CORE_LOG_API_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_CORE_LOG_API_ENABLE=0)
    endif()
    if(RAC_CORE_LOG_RADIO_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_CORE_LOG_RADIO_ENABLE=0)
    endif()

    # Apply RAC LoRa logging options
    if(RAC_LORA_LOG_TX_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_LORA_LOG_TX_ENABLE=1)
    endif()
    if(RAC_LORA_LOG_RX_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_LORA_LOG_RX_ENABLE=1)
    endif()
    if(RAC_LORA_LOG_CONFIG_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_LORA_LOG_CONFIG_ENABLE=0)
    endif()
    if(RAC_LORA_LOG_ERROR_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_LORA_LOG_ERROR_ENABLE=1)
    endif()
    if(RAC_LORA_LOG_INFO_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_LORA_LOG_INFO_ENABLE=0)
    endif()
    if(RAC_LORA_LOG_WARN_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_LORA_LOG_WARN_ENABLE=0)
    endif()
    if(RAC_LORA_LOG_DEBUG_ENABLE)
        target_compile_definitions(smtc_rac PRIVATE RAC_LORA_LOG_DEBUG_ENABLE=0)
    endif()

    message(STATUS "Applied logging configuration to smtc_rac target")
endif()

add_subdirectory(${EXAMPLES_DIR}/smtc_modem_hal smtc_modem_hal_build)
add_subdirectory(${EXAMPLES_DIR}/radio_hal radio_hal_build)

# Radio-specific compile definitions
# SX127x
if (RAC_RADIO STREQUAL "sx1272")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC SX1272)
    target_compile_definitions(radio_hal PUBLIC SX1272)
endif()
if (RAC_RADIO STREQUAL "sx1276")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC SX1276)
    target_compile_definitions(radio_hal PUBLIC SX1272)
endif()

# SX126x
# Not needed in smtc_modem_hal_implem - yet.
if (RAC_RADIO STREQUAL "sx1261")
    target_compile_definitions(radio_hal PUBLIC SX1261)
endif()
if (RAC_RADIO STREQUAL "sx1262")
    target_compile_definitions(radio_hal PUBLIC SX1262)
endif()
if (RAC_RADIO STREQUAL "sx1268")
    target_compile_definitions(radio_hal PUBLIC SX1268)
endif()

# LR11xx
if(RAC_RADIO STREQUAL "lr1110")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR1110)
endif()
if(RAC_RADIO STREQUAL "lr1120")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR1120)
endif()
if(RAC_RADIO STREQUAL "lr1121")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR1121)
endif()
if(RAC_RADIO IN_LIST lr11xx_radios)
    target_compile_definitions(smtc_hal PUBLIC LR11XX_TRANSCEIVER)
    target_compile_definitions(radio_hal PUBLIC LR11XX_TRANSCEIVER)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR11XX_TRANSCEIVER)
    if(USE_LR11XX_CRC_SPI)
        target_compile_definitions(radio_hal PRIVATE USE_LR11XX_CRC_OVER_SPI=1)
    endif()
endif()

# LR20xx
if(RAC_RADIO STREQUAL "lr2012")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR2012)
    target_compile_definitions(smtc_hal PUBLIC LR2012)
    target_compile_definitions(radio_hal PUBLIC LR2012)
    target_compile_definitions(smtc_ral_lr20xx PRIVATE LR2012)
endif()
if(RAC_RADIO STREQUAL "lr2021")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR2021)
    target_compile_definitions(smtc_hal PUBLIC LR2021)
    target_compile_definitions(radio_hal PUBLIC LR2021)
    target_compile_definitions(smtc_ral_lr20xx PRIVATE LR2021)
endif()
if(RAC_RADIO STREQUAL "lr2022")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR2022)
    target_compile_definitions(smtc_hal PUBLIC LR2022)
    target_compile_definitions(radio_hal PUBLIC LR2022)
    target_compile_definitions(smtc_ral_lr20xx PRIVATE LR2022)
endif()
if(RAC_RADIO IN_LIST lr20xx_radios)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR20XX)
    target_compile_definitions(smtc_hal PUBLIC LR20XX)
    target_compile_definitions(radio_hal PUBLIC LR20XX)
    if(LEGACY_EVK_LR20XX)
        target_compile_definitions(radio_hal PRIVATE LEGACY_EVK_LR20XX)
        target_compile_definitions(smtc_modem_hal_implem PRIVATE LEGACY_EVK_LR20XX)
        target_compile_definitions(smtc_hal PRIVATE LEGACY_EVK_LR20XX)
    endif()
endif()

# Optimized SPI for STM32 L4 (not managed by L0, and ra0e2)
if(NOT DEFINED OPTIMIZED_SPI)
  set(SPI_OPTIMIZED_BOARDS NUCLEO_L476)
  if(BOARD IN_LIST SPI_OPTIMIZED_BOARDS)
    set(OPTIMIZED_SPI 1)
  else()
    set(OPTIMIZED_SPI 0)
  endif()
endif()

target_compile_definitions(radio_hal PRIVATE
  OPTIMIZED_SPI=${OPTIMIZED_SPI}
)

if(APP_DEBUG)
    target_compile_definitions(smtc_hal PRIVATE HW_DEBUG_PROBE=1)
endif()

# LBM-related compile definitions to HAL targets
if(LBM_GEOLOCATION)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC ADD_APP_GEOLOCATION)
endif()
if(LBM_RELAY_TX)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC USE_RELAY_TX)
endif()
if(LBM_RELAY_RX)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC USE_RELAY_RX)
endif()
if(LBM_FUOTA)
    target_compile_definitions(smtc_hal PUBLIC USE_FUOTA USE_FLASH_READ_MODIFY_WRITE)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC USE_FUOTA USE_FLASH_READ_MODIFY_WRITE)
endif()
if(LBM_STORE_AND_FORWARD)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC ADD_SMTC_STORE_AND_FORWARD)
endif()

################################################################################
# LoRa Basics Modem library

add_subdirectory(${EXAMPLES_DIR}/../protocols/lbm_lib lbm_lib)
target_compile_definitions(lora_basics_modem PRIVATE USE_SMTC_RAC)

################################################################################
# FLRC protocol

if (USE_FLRC_PROTOCOL)
    add_subdirectory(${EXAMPLES_DIR}/../protocols/smtc_flrc_protocol smtc_flrc_protocol)
    # FLRP needs a longer PA ramp time on LR20xx for best sensitivity (see ral_lr20xx_bsp.c).
    target_compile_definitions(radio_hal PRIVATE USE_FLRC_PROTOCOL)
endif()

################################################################################
# Helper Functions

# Initialize the list of registered examples (used for all_examples target)
set(REGISTERED_EXAMPLES "" CACHE INTERNAL "List of registered example targets")

# Function to register an example (for all_examples target and status messages)
function(register_example TARGET_NAME EXAMPLE_DESCRIPTION)
    set(REGISTERED_EXAMPLES ${REGISTERED_EXAMPLES} ${TARGET_NAME} CACHE INTERNAL "")
    set(EXAMPLE_${TARGET_NAME}_DESCRIPTION "${EXAMPLE_DESCRIPTION}" CACHE INTERNAL "")
endfunction()

# Function to set up common example target configuration
# Call this after add_executable()
function(setup_example_target TARGET_NAME)
    # Set compile definitions
    target_compile_definitions(${TARGET_NAME} PRIVATE
        RP_MARGIN_DELAY=${RP_MARGIN_DELAY}
    )
    if(RAC_RADIO STREQUAL "lr2012")
        target_compile_definitions(${TARGET_NAME} PUBLIC LR2012)
    endif()
    if(RAC_RADIO STREQUAL "lr2021")
        target_compile_definitions(${TARGET_NAME} PUBLIC LR2021)
    endif()
    if(RAC_RADIO STREQUAL "lr2022")
        target_compile_definitions(${TARGET_NAME} PUBLIC LR2022)
    endif()
    if(RAC_RADIO IN_LIST lr20xx_radios)
        target_compile_definitions(${TARGET_NAME} PUBLIC LR20XX)
        if(LEGACY_EVK_LR20XX)
            target_compile_definitions(${TARGET_NAME} PRIVATE LEGACY_EVK_LR20XX)
        endif()
    endif()

    # For FPB_RA0E2: Add warmstart hook
    if(BOARD STREQUAL "FPB_RA0E2")
        target_sources(${TARGET_NAME} PRIVATE
            ${SMTC_HAL_DIR}/hal_warmstart.c
        )
    endif()

    # Add common sources
    target_sources(${TARGET_NAME} PRIVATE
        $<TARGET_OBJECTS:smtc_hal>
    )

    # Add geolocation BSP (weak default implementations, guarded by ADD_LBM_GEOLOCATION)
    # Board-specific code can override these with strong symbols
    target_sources(${TARGET_NAME} PRIVATE
        ${EXAMPLES_DIR}/main_examples/hw_modem/geoloc_bsp.c
    )

    # Add common includes
    target_include_directories(${TARGET_NAME} PUBLIC
        ${EXAMPLES_DIR}/main_examples
        ${EXAMPLES_DIR}/examples
        ${EXAMPLES_DIR}/main_examples/ranging_demo
        ${EXAMPLES_DIR}/smtc_sw_platform
        ${EXAMPLES_DIR}/../smtc_rac_lib/smtc_rac_api
        ${EXAMPLES_DIR}/../smtc_rac_lib/smtc_modem_hal
        ${EXAMPLES_DIR}/../smtc_rac_lib/radio_planner/src
        ${EXAMPLES_DIR}/../smtc_rac_lib/smtc_ral/src
        ${EXAMPLES_DIR}/../smtc_rac_lib/smtc_ralf/src
        ${EXAMPLES_DIR}/../smtc_rac_lib/radio_drivers
        ${EXAMPLES_DIR}/../protocols/network_discovery/network_discovery_api
        ${EXAMPLES_DIR}/../protocols/network_discovery/network_discovery_core
    )

    # FLRC protocol includes
    if(USE_FLRC_PROTOCOL)
        target_include_directories(${TARGET_NAME} PUBLIC
            ${EXAMPLES_DIR}/../protocols/smtc_flrc_protocol
            ${EXAMPLES_DIR}/../protocols/smtc_flrc_protocol/smtc_flrp_api
            ${EXAMPLES_DIR}/../protocols/smtc_flrc_protocol/smtc_flrp_core
        )
    endif()

    # For FPB_RA0E2: Add FSP include directories
    if(BOARD STREQUAL "FPB_RA0E2")
        target_include_directories(${TARGET_NAME} PRIVATE
            ${EXAMPLES_DIR}/mcu_drivers/cmsis/Core/Include
            ${EXAMPLES_DIR}/mcu_drivers/renesas_ra0e2/ra_gen
            ${EXAMPLES_DIR}/mcu_drivers/renesas_ra0e2/ra_cfg/fsp_cfg
            ${EXAMPLES_DIR}/mcu_drivers/renesas_ra0e2/ra_cfg/fsp_cfg/bsp
            ${EXAMPLES_DIR}/mcu_drivers/renesas_ra0e2/cmsis/Device/RENESAS/Include
            ${EXAMPLES_DIR}/mcu_drivers/renesas_ra0e2/bsp/mcu/ra0e2
            ${EXAMPLES_DIR}/mcu_drivers/renesas_fsp_common/bsp/mcu/all
            ${EXAMPLES_DIR}/mcu_drivers/renesas_fsp_common/fsp/inc
            ${EXAMPLES_DIR}/mcu_drivers/renesas_fsp_common/fsp/inc/api
            ${EXAMPLES_DIR}/mcu_drivers/renesas_fsp_common/fsp/inc/instances
        )
        target_compile_options(${TARGET_NAME} PRIVATE -Wno-pedantic)
    endif()

    # Link libraries
    target_link_libraries(${TARGET_NAME} PUBLIC
        smtc_modem_hal_implem
        smtc_rac
        LoRaBasicsModemApi
        lora_basics_modem
        smtc_hal
        radio_hal
    )

    # Generate hex file
    add_custom_command(
        TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${TARGET_NAME}>" "${CMAKE_BINARY_DIR}/${TARGET_NAME}.hex"
        COMMENT "Creating ${TARGET_NAME}.hex..."
    )

    # Generate bin file
    add_custom_command(
        TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary -S "$<TARGET_FILE:${TARGET_NAME}>" "${CMAKE_BINARY_DIR}/${TARGET_NAME}.bin"
        COMMENT "Creating ${TARGET_NAME}.bin..."
    )

    # Add flash target
    add_custom_target(flash_${TARGET_NAME}
        DEPENDS ${TARGET_NAME}
        COMMAND st-flash --connect-under-reset write "${CMAKE_BINARY_DIR}/${TARGET_NAME}.bin" 0x8000000
        USES_TERMINAL
    )
endfunction()
