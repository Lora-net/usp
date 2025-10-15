# SPDX-License-Identifier: BSD-3-Clause-Clear

cmake_minimum_required(VERSION 3.25)

if(BOARD STREQUAL NUCLEO_L073)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/smtc_hal_l0_LL/cmake_stm32l0_toolchain.cmake)
elseif(BOARD STREQUAL NUCLEO_L476)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/smtc_hal_l4/cmake_stm32l4_toolchain.cmake)
else()
    message(FATAL_ERROR "You have to define a -DBOARD=NUCLEO_L476 or NUCLEO_L073")
endif()
set_property(CACHE BOARD PROPERTY STRINGS "NUCLEO_L476;NUCLEO_L073")

project(examples
    DESCRIPTION "LoRa Basics Modem examples"
    LANGUAGES C ASM
)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

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

# Generate a compile_commands.json for IDEs / clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################################################
# Project Options

################################################################################
# Radio Planner Options
set(RP_MARGIN_DELAY "8" CACHE STRING "Radio Planner margin delay in ms")
message(STATUS "RP_MARGIN_DELAY: ${RP_MARGIN_DELAY}")

# LR2021 LoRa Plus Exapansion Board vs LR2021 shield : IRQ is not mapped to the same PIN
if(${RAC_RADIO} MATCHES "lr2021")
  option(LEGACY_EVK_LR2021 "Enable LR2021 shield" OFF)
  message(STATUS "LR2021 shield: ${LEGACY_EVK_LR2021}")
endif()

# Activation of CRC over SPI on LR11XX
if(${RAC_RADIO} MATCHES "^(lr1110|lr1120|lr1121)$")
option(USE_LR11XX_CRC_SPI "Enable " OFF)
endif()

################################################################################
# Logging Options
option(RAC_LOGGING_ENABLE "Enable logging system" ON)

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
# First build the HAL that might set useful variables

message(STATUS "SMTC_HAL_DIR: ${SMTC_HAL_DIR}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR:${CMAKE_CURRENT_SOURCE_DIR}")

# Set target radio
set(sx126x_radios sx1261 sx1262 sx1268)
set(lr11xx_radios lr1110 lr1120 lr1121)
set(lr20xx_radios lr2021)
set(known_radios ${sx126x_radios} ${lr11xx_radios} ${lr20xx_radios})

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
endif()

# Set target board
string(TOUPPER ${BOARD} BOARD_UPPER)
string(TOUPPER ${RADIO_FAMILY} RADIO_FAMILY_UPPER)

# Define SMTC_HAL_DIR
if(BOARD STREQUAL NUCLEO_L073)
    set(SMTC_HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/smtc_hal_l0_LL)
elseif(BOARD STREQUAL NUCLEO_L476)
    set(SMTC_HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/smtc_hal_l4)
endif()

message(STATUS "Selected radio ${RAC_RADIO} in family ${RADIO_FAMILY}\n")
if(DEFINED LBM_RADIO AND NOT LBM_RADIO STREQUAL ${RAC_RADIO})
    message(FATAL_ERROR "LBM_RADIO (${LBM_RADIO}) must match RAC_RADIO (${RAC_RADIO})")
endif()
set(LBM_RADIO ${RAC_RADIO})

add_subdirectory(${SMTC_HAL_DIR})
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib smtc_rac_lib)

# Apply logging configuration to smtc_rac target
if(TARGET smtc_rac)
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

    # Apply RAC LoRa logging options to the same target (both are in smtc_rac)
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

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/smtc_modem_hal)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/radio_hal)
if(${RAC_RADIO} MATCHES "lr1110")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR1110)
endif()
if(${RAC_RADIO} MATCHES "lr1120")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR1120)
endif()
if(${RAC_RADIO} MATCHES "lr1121")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR1121)
endif()
if(${RAC_RADIO} MATCHES "^(lr1110|lr1120|lr1121)$")
    if(USE_LR11XX_CRC_SPI)
      target_compile_definitions(radio_hal PRIVATE USE_LR11XX_CRC_OVER_SPI=1)
    endif()
endif()
if(${RAC_RADIO} MATCHES "lr2021")
  target_compile_definitions(smtc_modem_hal_implem PUBLIC LR2021)
  target_compile_definitions(smtc_hal PUBLIC LR2021)
  target_compile_definitions(radio_hal PUBLIC LR2021)
    if(LEGACY_EVK_LR2021)
      target_compile_definitions(radio_hal PRIVATE LEGACY_EVK_LR2021)
      target_compile_definitions(smtc_modem_hal_implem PRIVATE LEGACY_EVK_LR2021)
      target_compile_definitions(smtc_hal PRIVATE LEGACY_EVK_LR2021)
    endif()
endif()
if(LBM_FUOTA)
    target_compile_definitions(smtc_hal PUBLIC USE_FUOTA USE_FLASH_READ_MODIFY_WRITE)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC USE_FUOTA USE_FLASH_READ_MODIFY_WRITE)
endif()
if(LBM_STORE_AND_FORWARD)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC USE_STORE_AND_FORWARD)
endif()
################################################################################
# Original example target (keep for compatibility)

add_executable(example main.c)

target_sources(example PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/main_lctt_certif.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/main_periodical_uplink.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/multiprotocol_example/main_multiprotocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo/main_ranging_demo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo/app_ranging_hopping.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ping_pong_example/main_ping_pong.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ping_pong_example/app_ping_pong.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ping_pong_example/app_periodic_uplink.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_example/main_per.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_example/app_per.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_flrc_example/main_per_flrc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_flrc_example/app_per_flrc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/lrfhss_example/main_lrfhss_example.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/lrfhss_example/app_lrfhss_example.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/spectral_scan_example/main_spectral_scan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/spectral_scan_example/app_spectral_scan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/main_rf_certification.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/app_rf_certification.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/app_notification.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_sw_platform/smtc_sw_platform_helper.c
)

target_include_directories(example PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples
    ${CMAKE_CURRENT_SOURCE_DIR}/examples
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/spectral_scan_example
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_sw_platform
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/smtc_rac_api
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/smtc_modem_hal
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/radio_planner/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/smtc_ral/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/smtc_ralf/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/radio_drivers
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../protocols/lbm_lib lbm_lib)
target_link_libraries(example PUBLIC
    smtc_modem_hal_implem
    smtc_rac
    LoRaBasicsModemApi
    lora_basics_modem
    smtc_hal
    radio_hal
)

# Compile options for original target
if(DEFINED APP)
    string(TOUPPER ${APP} APP_UPPER)
    target_compile_definitions(example PRIVATE
        MAKEFILE_APP=${APP_UPPER}
        RP_MARGIN_DELAY=${RP_MARGIN_DELAY}
    )

    if(${RAC_RADIO} MATCHES "lr2021")
      target_compile_definitions(example PUBLIC LR2021)
        if(LEGACY_EVK_LR2021)
            target_compile_definitions(example PRIVATE LEGACY_EVK_LR2021)
        endif()
    endif()
    set(APPS_WITH_OPTIONAL_PERIODIC_UPLINK RTTOF PING_PONG)
    option(PERIODIC_UPLINK "Enable the periodic uplink" ON)
    if(PERIODIC_UPLINK)
        if(NOT APP_UPPER IN_LIST APPS_WITH_OPTIONAL_PERIODIC_UPLINK)
            message(WARNING "unused variable: PERIODIC_UPLINK")
        endif()
        target_compile_definitions(example PRIVATE ENABLE_PERIODIC_UPLINK)
    endif()

    set(RANGING_DEVICE_MODE "" CACHE STRING "RTTOF mode of the device (MANAGER or SUBORDINATE)")
    if(NOT RANGING_DEVICE_MODE)
        if(APP_UPPER MATCHES "RTTOF")
            message(WARNING "RANGING_DEVICE_MODE not defined, using default value")
        endif()
        target_compile_definitions(example PRIVATE RANGING_DEVICE_MODE=RANGING_DEVICE_MODE_MANAGER)
    else()
        if(NOT APP_UPPER MATCHES "RTTOF")
            message(WARNING "unused variable: RANGING_DEVICE_MODE")
        endif()
        target_compile_definitions(example PRIVATE RANGING_DEVICE_MODE=RANGING_DEVICE_MODE_${RANGING_DEVICE_MODE})
    endif()

    set(ROLE "" CACHE STRING "Role of the device (TRANSMITTER or RECEIVER)")
    if(NOT ROLE)
        if(APP_UPPER MATCHES "PER")
            message(WARNING "ROLE not defined, using default value")
        endif()
        target_compile_definitions(example PRIVATE ROLE=TRANSMITTER)
    else()
        if(NOT APP_UPPER MATCHES "PER")
            message(WARNING "unused variable: ROLE")
        endif()
        target_compile_definitions(example PRIVATE ROLE=${ROLE})
    endif()

    target_sources(example PRIVATE
        main.c
        $<TARGET_OBJECTS:smtc_hal>
    )

    add_custom_command(
        TARGET example POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:example>" "$<TARGET_FILE:example>.hex"
        COMMENT "Creating hex..."
    )

    add_custom_command(
        TARGET example POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary -S "$<TARGET_FILE:example>" "$<TARGET_FILE:example>.bin"
        COMMENT "Creating bin..."
    )

    add_custom_target(flash
        DEPENDS example
        COMMAND st-flash --connect-under-reset write $<TARGET_FILE:example>.bin 0x8000000
        USES_TERMINAL
    )

    add_custom_target(flash_copy
        DEPENDS example
        COMMAND cp $<TARGET_FILE:example>.bin /media/$ENV{USER}/NODE_L476RG/.
        USES_TERMINAL
    )
endif()

################################################################################
# Function to create example targets

function(add_example TARGET_NAME APP_DEFINE EXAMPLE_SOURCES)
    # Create executable
    add_executable(${TARGET_NAME} ${EXAMPLE_SOURCES})

    # Set compile definitions
    string(TOUPPER ${APP_DEFINE} APP_UPPER)
    target_compile_definitions(${TARGET_NAME} PRIVATE
        MAKEFILE_APP=${APP_UPPER}
        RP_MARGIN_DELAY=${RP_MARGIN_DELAY}
    )

    # Add common sources
    target_sources(${TARGET_NAME} PRIVATE
        main.c
        $<TARGET_OBJECTS:smtc_hal>
        ${CMAKE_CURRENT_SOURCE_DIR}/smtc_sw_platform/smtc_sw_platform_helper.c
    )

    # Add includes
    target_include_directories(${TARGET_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/main_examples
        ${CMAKE_CURRENT_SOURCE_DIR}/examples
        ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo
        ${CMAKE_CURRENT_SOURCE_DIR}/smtc_sw_platform
        ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/smtc_rac_api
        ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/smtc_modem_hal
        ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/radio_planner/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/smtc_ral/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/smtc_ralf/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_lib/radio_drivers
    )

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

################################################################################
# Define all examples



# RTToF (Ranging) Examples
add_example(rttof_manager "RTTOF"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo/main_ranging_demo.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo/app_ranging_hopping.c"
)
target_compile_definitions(rttof_manager PRIVATE
    RANGING_DEVICE_MODE=RANGING_DEVICE_MODE_MANAGER
    ENABLE_PERIODIC_UPLINK
)

add_example(rttof_subordinate "RTTOF"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo/main_ranging_demo.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo/app_ranging_hopping.c"
)
target_compile_definitions(rttof_subordinate PRIVATE
    RANGING_DEVICE_MODE=RANGING_DEVICE_MODE_SUBORDINATE
    ENABLE_PERIODIC_UPLINK
)

# Ping Pong Example
add_example(ping_pong "PING_PONG"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ping_pong_example/main_ping_pong.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ping_pong_example/app_ping_pong.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ping_pong_example/app_periodic_uplink.c"
)
target_compile_definitions(ping_pong PRIVATE ENABLE_PERIODIC_UPLINK)

# PER (Packet Error Rate) Examples
add_example(per_tx "PER"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_example/main_per.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_example/app_per.c"
)
target_compile_definitions(per_tx PRIVATE ROLE=2) # TRANSMITTER

add_example(per_rx "PER"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_example/main_per.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_example/app_per.c"
)
target_compile_definitions(per_rx PRIVATE ROLE=1) # RECEIVER

# FSK PER (Packet Error Rate) Examples
add_example(per_fsk_tx "PER_FSK"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_fsk_example/main_per_fsk.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_fsk_example/app_per_fsk.c"
)
target_compile_definitions(per_fsk_tx PRIVATE ROLE=2) # TRANSMITTER
target_include_directories(per_fsk_tx PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_fsk_example
)

add_example(per_fsk_rx "PER_FSK"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_fsk_example/main_per_fsk.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_fsk_example/app_per_fsk.c"
)
target_compile_definitions(per_fsk_rx PRIVATE ROLE=1) # RECEIVER
target_include_directories(per_fsk_rx PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_fsk_example
)

# FLRC PER (Packet Error Rate) Examples
add_example(per_flrc_tx "PER_FLRC"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_flrc_example/main_per_flrc.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_flrc_example/app_per_flrc.c"
)
target_compile_definitions(per_flrc_tx PRIVATE ROLE=2) # TRANSMITTER
target_include_directories(per_flrc_tx PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_flrc_example
)

add_example(per_flrc_rx "PER_FLRC"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_flrc_example/main_per_flrc.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_flrc_example/app_per_flrc.c"
)
target_compile_definitions(per_flrc_rx PRIVATE ROLE=1) # RECEIVER
target_include_directories(per_flrc_rx PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/packet_error_rate_flrc_example
)

# LR-FHSS Example
add_example(lrfhss_tx "LR_FHSS"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/lrfhss_example/main_lrfhss_example.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/lrfhss_example/app_lrfhss_example.c"
)
target_include_directories(lrfhss_tx PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/lrfhss_example
)

# Generate lrfhss.bin file (alias for lrfhss_tx.bin)
add_custom_command(
    TARGET lrfhss_tx POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/lrfhss_tx.bin" "${CMAKE_BINARY_DIR}/lrfhss.bin"
    COMMENT "Creating lrfhss.bin..."
)

# Spectral Scan Example
add_example(spectral_scan "SPECTRAL_SCAN"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/spectral_scan_example/main_spectral_scan.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/spectral_scan_example/app_spectral_scan.c"
)
target_include_directories(spectral_scan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/spectral_scan_example
)
# RF Certification Examples (one for each region)
set(NOTIFICATION_MODE "NOTIFICATIONS_OFF" CACHE STRING "Notification mode (ON or OFF)")
set(APP_MODE "APP_MODE_CERTIFICATION" CACHE STRING "App mode (CERTIFICATION or NOTIFICATION)")

# ETSI Region
add_example(rf_certification_etsi "RF_CERTIFICATION"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/main_rf_certification.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/app_rf_certification.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/app_notification.c"
)
target_include_directories(rf_certification_etsi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example
)
target_compile_definitions(rf_certification_etsi PRIVATE RF_CERT_REGION=RF_CERT_REGION_ETSI)
target_compile_definitions(rf_certification_etsi PRIVATE NOTIFICATION_MODE=${NOTIFICATION_MODE})
target_compile_definitions(rf_certification_etsi PRIVATE APP_MODE=${APP_MODE})

# ARIB Region
add_example(rf_certification_arib "RF_CERTIFICATION"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/main_rf_certification.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/app_rf_certification.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/app_notification.c"
)
target_include_directories(rf_certification_arib PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example
)
target_compile_definitions(rf_certification_arib PRIVATE RF_CERT_REGION=RF_CERT_REGION_ARIB)
target_compile_definitions(rf_certification_arib PRIVATE NOTIFICATION_MODE=${NOTIFICATION_MODE})
target_compile_definitions(rf_certification_arib PRIVATE APP_MODE=${APP_MODE})

# FCC Region
add_example(rf_certification_fcc "RF_CERTIFICATION"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/main_rf_certification.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/app_rf_certification.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example/app_notification.c"
)
target_include_directories(rf_certification_fcc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/rf_certification_example
)
target_compile_definitions(rf_certification_fcc PRIVATE RF_CERT_REGION=RF_CERT_REGION_FCC)
target_compile_definitions(rf_certification_fcc PRIVATE NOTIFICATION_MODE=${NOTIFICATION_MODE})
target_compile_definitions(rf_certification_fcc PRIVATE APP_MODE=${APP_MODE})

# Periodical Uplink Example
add_example(periodical_uplink "PERIODICAL_UPLINK"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/main_periodical_uplink.c"
)
target_compile_definitions(periodical_uplink PRIVATE ENABLE_PERIODIC_UPLINK)

# Multiprotocol Example
add_example(multiprotocol "MULTIPROTOCOL"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/multiprotocol_example/main_multiprotocol.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo/app_ranging_hopping.c"
)
target_include_directories(multiprotocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/ranging_demo
)




# Hardware Modem Test Example
add_example(hw_modem "HW_MODEM"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/main_porting_tests.c"
)

# LCTT Certification Example
add_example(lctt_certif "LCTT_CERTIF"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/main_lctt_certif.c"
)

# TX CW (Continuous Wave) Example
add_example(tx_cw "TX_CW"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/tx_cw_example/main_tx_cw.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/tx_cw_example/app_tx_cw.c"
)
target_include_directories(tx_cw PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/tx_cw_example
)
option(INFINITE_PREAMBLE "Enable INFINITE_PREAMBLE" OFF)

if(INFINITE_PREAMBLE)
  target_compile_definitions(tx_cw PRIVATE INFINITE_PREAMBLE)
  message(STATUS "INFINITE PREAMBLE ENABLED")
else()
  message(STATUS "CONTINUOUS WAVE ENABLED")
endif()

# Direct Driver Access Example
if(RAC_RADIO STREQUAL "lr2021")
add_example(direct_driver_access "DIRECT_DRIVER_ACCESS"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_examples/direct_driver_access_example/main_direct_driver_access.c;${CMAKE_CURRENT_SOURCE_DIR}/main_examples/direct_driver_access_example/app_direct_driver_access.c"
)
target_include_directories(direct_driver_access PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main_examples/direct_driver_access_example
)
endif()



################################################################################
# Create "all_examples" target to build everything

add_custom_target(all_examples
    DEPENDS rttof_manager rttof_subordinate ping_pong per_tx per_rx per_fsk_tx per_fsk_rx per_flrc_tx per_flrc_rx lrfhss_tx spectral_scan periodical_uplink hw_modem lctt_certif multiprotocol rf_certification_etsi rf_certification_arib rf_certification_fcc tx_cw direct_driver_access
    COMMENT "Building all examples..."
)

################################################################################
# Display information

message(STATUS "")
message(STATUS "Available examples:")
message(STATUS "  - rttof_manager    : ranging (RTToF) manager")
message(STATUS "  - rttof_subordinate: ranging (RTToF) subordinate")
message(STATUS "  - ping_pong        : ping-pong communication example")
message(STATUS "  - per_tx           : packet error rate - LoRa (transmitter)")
message(STATUS "  - per_rx           : packet error rate - LoRa (receiver)")
message(STATUS "  - per_fsk_tx       : packet error rate - FSK (transmitter)")
message(STATUS "  - per_fsk_rx       : packet error rate - FSK (receiver)")
message(STATUS "  - per_flrc_tx      : packet error rate - FLRC (transmitter)")
message(STATUS "  - per_flrc_rx      : packet error rate - FLRC (receiver)")
message(STATUS "  - lrfhss_tx        : LR-FHSS transmission example")
message(STATUS "  - spectral_scan    : spectral scan analysis example")
message(STATUS "  - periodical_uplink      : Periodical uplink transmission example")
message(STATUS "  - multiprotocol          : Multiprotocol example")
message(STATUS "  - hw_modem               : Hardware modem porting test")
message(STATUS "  - lctt_certif            : LCTT certification example")

message(STATUS "  - rf_certification_etsi : RF certification continuous LoRa TX example (ETSI region)")
message(STATUS "  - rf_certification_arib : RF certification continuous LoRa TX example (ARIB region)")
message(STATUS "  - rf_certification_fcc  : RF certification continuous LoRa TX example (FCC region)")
message(STATUS "  - tx_cw            : continuous transmission (TX CW) example")
message(STATUS "  - direct_driver_access : direct radio driver access example")
message(STATUS "")
message(STATUS "Build targets:")
message(STATUS "  - ninja -C build all_examples    : Build all examples")
message(STATUS "  - ninja -C build <example_name>  : Build specific example")
message(STATUS "  - ninja -C build flash_<example> : Flash specific example to device")
message(STATUS "")
message(STATUS "Generated files will be in ${CMAKE_BINARY_DIR}/:")
message(STATUS "  - <example_name>.bin")
message(STATUS "  - <example_name>.hex")
message(STATUS "")
message(STATUS "Logging Configuration:")
message(STATUS "  Profile: ${RAC_LOG_PROFILE}")
message(STATUS "  Core - ERROR:${RAC_CORE_LOG_ERROR_ENABLE} CONFIG:${RAC_CORE_LOG_CONFIG_ENABLE} INFO:${RAC_CORE_LOG_INFO_ENABLE} DEBUG:${RAC_CORE_LOG_DEBUG_ENABLE}")
message(STATUS "  LoRa - TX:${RAC_LORA_LOG_TX_ENABLE} RX:${RAC_LORA_LOG_RX_ENABLE} ERROR:${RAC_LORA_LOG_ERROR_ENABLE} CONFIG:${RAC_LORA_LOG_CONFIG_ENABLE}")
message(STATUS "")
message(STATUS "To change logging profile: -DRAC_LOG_PROFILE=<MINIMAL|DEFAULT|VERBOSE|DEBUG|ALL>")
message(STATUS "To enable specific logs: -DRAC_CORE_LOG_INFO_ENABLE=ON -DRAC_LORA_LOG_DEBUG_ENABLE=ON")
message(STATUS "")
