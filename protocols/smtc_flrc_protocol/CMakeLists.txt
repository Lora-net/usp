# Configuration for the smtc_flrc_protocol library

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_core/smtc_flrp_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_crypto/smtc_flrp_crypto.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_crypto/soft_secure_element/aes_fast.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_crypto/soft_secure_element/cmac_fast.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_crypto/soft_secure_element/soft_se_fast.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_mac_layer/src/smtc_flrp_mac_layer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_mac_layer/src/smtc_flrp_mac_serde.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_mac_layer/src/smtc_flrp_mac_adaptive_link.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_mac_layer/src/smtc_flrp_mac_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_wor/src/smtc_flrp_wor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_wor/src/smtc_wor_rx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_wor/src/smtc_wor_tx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_wor/src/smtc_wor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_utils.c
)

add_library(smtc_flrc_protocol STATIC ${SOURCES})

# Include necessary directories
target_include_directories(smtc_flrc_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_api
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_core
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_mac_layer/src
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_wor/src
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_crypto
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_flrp_crypto/soft_secure_element
    ${CMAKE_CURRENT_SOURCE_DIR}/../../smtc_rac_lib/smtc_rac_api
    ${CMAKE_CURRENT_SOURCE_DIR}/../../smtc_rac_lib/smtc_rac
)


# add_subdirectory(smtc_flrp_crypto)
include(smtc_flrp_crypto.cmake)

# Link required dependencies
target_link_libraries(smtc_flrc_protocol PUBLIC
    smtc_rac
)

# Set C standards
set_target_properties(smtc_flrc_protocol PROPERTIES C_STANDARD 11)

# Set preprocessor macros

target_compile_definitions(smtc_flrc_protocol PRIVATE NUMBER_OF_STACKS=1)


if(TEST_LONG_PAYLOAD)
target_compile_definitions(smtc_flrc_protocol PRIVATE TEST_LONG_PAYLOAD=1)
endif()

if(DEBUG_LOG)
target_compile_definitions(smtc_flrc_protocol PRIVATE DEBUG_LOG=1)
endif()

# Install the library
install(TARGETS smtc_flrc_protocol
    EXPORT FLRCCoreTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
