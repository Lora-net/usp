# SPDX-License-Identifier: BSD-3-Clause-Clear

# Select HAL implementation based on platform
if("${BOARD}" STREQUAL "LINUX" OR "${BOARD}" STREQUAL "LINUX_ARM" OR "${BOARD}" STREQUAL "LINUX_ARM64")
    # Linux (native or ARM) uses direct spidev/GPIO implementation
    # UDP virtual radio has no hardware HAL - only needs BSP
    if(RADIO_FAMILY STREQUAL "udp_pf")
        add_library(radio_hal OBJECT
            ral_${RADIO_FAMILY}_bsp.c
            radio_utilities.c
        )
    else()
        add_library(radio_hal OBJECT
            ${RADIO_FAMILY}_hal_linux.c
            ral_${RADIO_FAMILY}_bsp.c
            radio_utilities.c
        )
    endif()
else()
    # STM32 uses standard smtc_hal_* abstraction
    add_library(radio_hal OBJECT
        ${RADIO_FAMILY}_hal.c
        ral_${RADIO_FAMILY}_bsp.c
        radio_utilities.c
    )
endif()

target_link_libraries(radio_hal PUBLIC
    smtc_hal
    smtc_modem_hal_implem
    smtc_ral::smtc_ral_${RADIO_FAMILY}
)

# Link driver only if not UDP virtual radio (which has no real driver)
if(NOT RADIO_FAMILY STREQUAL "udp_pf")
    target_link_libraries(radio_hal PUBLIC
        ${RADIO_FAMILY}_driver
    )
endif()

target_include_directories(radio_hal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
