# Configuration for the radio_planner library

add_library(radio_planner STATIC
    src/duty_cycle.c
    src/radio_planner.c
)

# Set preprocessor macros
if(NOT DEFINED RP_MARGIN_DELAY)
    set(RP_MARGIN_DELAY "8")
endif()
message(STATUS "radio_planner: RP_MARGIN_DELAY=${RP_MARGIN_DELAY}")
target_compile_definitions(radio_planner PRIVATE RP_MARGIN_DELAY=${RP_MARGIN_DELAY} )


# Include necessary directories
target_include_directories(radio_planner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../smtc_modem_hal
)

# Add LR11xx include if geoloc (geoloc implies LR1110 / LR1120 chip so no need to check radio aswell)
if(LBM_GEOLOCATION)
    target_include_directories(radio_planner PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../radio_drivers/lr11xx_driver/src 
    )
endif()

# Link the dependency to the ralf library
target_link_libraries(radio_planner PUBLIC
    smtc_ralf_${RADIO_FAMILY}
    smtc_ral_${RADIO_FAMILY}
)
target_link_libraries(radio_planner PUBLIC lora_basics_modem)
# Set C standards
set_target_properties(radio_planner PROPERTIES C_STANDARD 11)

# Install the library
install(TARGETS radio_planner
    EXPORT RadioPlannerTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install the header files
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/radio_planner.h
    DESTINATION include/radio_planner
)
