# SPDX-License-Identifier: BSD-3-Clause-Clear
# FLRC Protocol API Examples

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "FLRP API")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(flrp_api LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")

################################################################################
# FLRC Protocol Library
set(USE_FLRC_PROTOCOL ON CACHE BOOL "")

include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Create targets

set(FLRP_API_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_flrp_api.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_flrp_api.c
)

# Initiator
add_executable(flrp_api_initiator ${FLRP_API_SOURCES})
setup_example_target(flrp_api_initiator)
register_example(flrp_api_initiator "${DESCRIPTION} (initiator)")
target_compile_definitions(flrp_api_initiator PRIVATE FLRP_API_ROLE=1)
target_include_directories(flrp_api_initiator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(flrp_api_initiator PUBLIC smtc_flrc_protocol)

# Slave
add_executable(flrp_api_slave ${FLRP_API_SOURCES})
setup_example_target(flrp_api_slave)
register_example(flrp_api_slave "${DESCRIPTION} (slave)")
target_compile_definitions(flrp_api_slave PRIVATE FLRP_API_ROLE=0)
target_include_directories(flrp_api_slave PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(flrp_api_slave PUBLIC smtc_flrc_protocol)

# Build all targets
add_custom_target(flrp_api DEPENDS
    flrp_api_initiator
    flrp_api_slave
)