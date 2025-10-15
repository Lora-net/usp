# SPDX-License-Identifier: BSD-3-Clause-Clear

set(SMTC_MODEM_API_DIR ${CMAKE_CURRENT_LIST_DIR})

add_library(smtc_modem_api_internal INTERFACE)
target_include_directories(smtc_modem_api_internal INTERFACE
    ${SMTC_MODEM_API_DIR}
    ${LORA_BASICS_MODEM_DIR}/smtc_modem_core/smtc_ral/src
)
