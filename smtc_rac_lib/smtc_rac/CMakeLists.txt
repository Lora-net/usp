# Configuration for the smtc_rac library

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_rac.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_rac_lora.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_rac_fsk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_rac_lrfhss.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_rac_lbt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_rac_flrc.c
)

add_library(smtc_rac STATIC ${SOURCES})

string(TOUPPER ${RADIO_FAMILY} RADIO_FAMILY_UPPER)
target_compile_definitions(smtc_rac PUBLIC
    ${RADIO_FAMILY_UPPER}
    RP_MARGIN_DELAY=${RP_MARGIN_DELAY}
)

# Apply RAC library logging configuration to smtc_rac target
if(RAC_FSK_LOG_ENABLE)
    target_compile_definitions(smtc_rac PRIVATE RAC_FSK_LOG_ENABLE=1)
    message(STATUS "  - FSK logs: ENABLED")
else()
    target_compile_definitions(smtc_rac PRIVATE RAC_FSK_LOG_ENABLE=0)
    message(STATUS "  - FSK logs: DISABLED")
endif()

if(RAC_LORA_LOG_ENABLE)
    target_compile_definitions(smtc_rac PRIVATE RAC_LORA_LOG_ENABLE=1)
    message(STATUS "  - LoRa logs: ENABLED")
else()
    target_compile_definitions(smtc_rac PRIVATE RAC_LORA_LOG_ENABLE=0)
    message(STATUS "  - LoRa logs: DISABLED")
endif()

if(RAC_LRFHSS_LOG_ENABLE)
    target_compile_definitions(smtc_rac PRIVATE RAC_LRFHSS_LOG_ENABLE=1)
    message(STATUS "  - LR-FHSS logs: ENABLED")
else()
    target_compile_definitions(smtc_rac PRIVATE RAC_LRFHSS_LOG_ENABLE=0)
    message(STATUS "  - LR-FHSS logs: DISABLED")
endif()

message(STATUS "Applied RAC library logging configuration")

# Include necessary directories
target_include_directories(smtc_rac PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_rac_api
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_ral/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_ralf/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../radio_planner/src
)

# Add radio driver includes conditionally for workarounds
if(${RADIO_FAMILY} STREQUAL "lr20xx")
    target_include_directories(smtc_rac PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../radio_drivers/lr20xx_driver/inc
    )
endif()

# Link required dependencies
target_link_libraries(smtc_rac PUBLIC
    smtc_hal
    smtc_ral_${RADIO_FAMILY}
    smtc_ralf_${RADIO_FAMILY}
    radio_planner
)

# Set C standards
set_target_properties(smtc_rac PROPERTIES C_STANDARD 11)

# Install the library
install(TARGETS smtc_rac
    EXPORT RACCoreTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
