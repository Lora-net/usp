cmake_minimum_required(VERSION 3.25)

project(smtc_rac_lib VERSION 1.0.0 LANGUAGES C)

include(rac_logging.cmake)

add_subdirectory(smtc_core_hal_interface)


# Handles chosen radio verification, alongside w/ corresponding ralf, ral, and radio drivers
set(SMTC_RADIO ${RAC_RADIO}) # smtc_radio_interface expects SMTC_RADIO as input
add_subdirectory(smtc_radio_interface)
# smtc_radio_interface exposes RADIO_UPPER, RADIO_FAMILY_UPPER, RADIO_FAMILY
# These might need to be exposed to the caller CMakeLists.txt
if(NOT PROJECT_IS_TOP_LEVEL)
    set(RADIO_FAMILY ${RADIO_FAMILY} PARENT_SCOPE)
    set(RADIO_UPPER ${RADIO_UPPER} PARENT_SCOPE)
    set(RADIO_FAMILY_UPPER ${RADIO_FAMILY_UPPER} PARENT_SCOPE)
endif()


add_subdirectory(radio_planner)

# Compile definition for radio_planner
# PUBLIC so the consumers (smtc_rac and possibly LBM) inherit this property
target_compile_definitions(radio_planner PUBLIC USE_SMTC_RAC=1)

add_subdirectory(smtc_rac)

# If sample includes LBM, handle it here
if(INCLUDE_LBM)
    message(STATUS "Including LBM - RAC_RADIO = ${RAC_RADIO}")
    set(LBM_RADIO ${RAC_RADIO})
    add_subdirectory("../protocols/lbm_lib" lbm_lib)

    target_compile_definitions(radio_planner PUBLIC USE_LBM=1)

    # smtc_rac needs to access smtc_modem to set context pointers
    target_link_libraries(smtc_rac PRIVATE lora_basics_modem)
endif()


# Link driver only if not UDP virtual radio (which has no real driver)
# if(NOT ${RADIO_FAMILY} STREQUAL "udp_pf")
#     target_link_libraries(smtc_ral_${RADIO_FAMILY} PRIVATE
#         ${RADIO_FAMILY}_driver
#     )
# endif()

# if(${RADIO_FAMILY} MATCHES "lr20xx")
#     target_link_libraries(smtc_ral_lr20xx PRIVATE
#         lr20xx_driver_rttof
#     )
# endif()

# target_link_libraries(smtc_ralf_${RADIO_FAMILY} PRIVATE
#     smtc_ral_${RADIO_FAMILY}
# )
