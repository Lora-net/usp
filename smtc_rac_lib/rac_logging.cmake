################################################################################
# RAC Library Logging Configuration
################################################################################

# RAC library logging controls - independent from examples logging
option(RAC_FSK_LOG_ENABLE "Enable FSK module logs" OFF)
option(RAC_LORA_LOG_ENABLE "Enable LoRa module logs" OFF)
option(RAC_LRFHSS_LOG_ENABLE "Enable LR-FHSS module logs" OFF)

# Predefined logging profiles for library
set(RAC_LIB_LOG_PROFILE "DEFAULT" CACHE STRING "RAC library logging profile")
set_property(CACHE RAC_LIB_LOG_PROFILE PROPERTY STRINGS "MINIMAL;DEFAULT;VERBOSE;ALL;OFF")

# Apply library logging profile settings
if(RAC_LIB_LOG_PROFILE STREQUAL "OFF")
    message(STATUS "RAC Library Logging Profile: OFF (no logs)")
    set(RAC_FSK_LOG_ENABLE OFF CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_ENABLE OFF CACHE BOOL "" FORCE)
    set(RAC_LRFHSS_LOG_ENABLE OFF CACHE BOOL "" FORCE)
elseif(RAC_LIB_LOG_PROFILE STREQUAL "MINIMAL")
    message(STATUS "RAC Library Logging Profile: MINIMAL (only critical modules)")
    set(RAC_FSK_LOG_ENABLE OFF CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LRFHSS_LOG_ENABLE OFF CACHE BOOL "" FORCE)
elseif(RAC_LIB_LOG_PROFILE STREQUAL "VERBOSE")
    message(STATUS "RAC Library Logging Profile: VERBOSE (all modules)")
    set(RAC_FSK_LOG_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LRFHSS_LOG_ENABLE ON CACHE BOOL "" FORCE)
elseif(RAC_LIB_LOG_PROFILE STREQUAL "ALL")
    message(STATUS "RAC Library Logging Profile: ALL (all modules with debug)")
    set(RAC_FSK_LOG_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LORA_LOG_ENABLE ON CACHE BOOL "" FORCE)
    set(RAC_LRFHSS_LOG_ENABLE ON CACHE BOOL "" FORCE)
else()
    message(STATUS "RAC Library Logging Profile: DEFAULT (FSK, LoRa, LR-FHSS)")
endif()