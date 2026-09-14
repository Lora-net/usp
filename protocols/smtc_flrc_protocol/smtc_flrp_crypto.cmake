# SPDX-License-Identifier: BSD-3-Clause-Clear

set(SMTC_FLRP_CRYPTO_DIR ${CMAKE_CURRENT_LIST_DIR}/smtc_flrp_crypto)

add_library(smtc_flrp_crypto OBJECT)

target_include_directories(smtc_flrp_crypto PUBLIC
    ${SMTC_FLRP_CRYPTO_DIR}/soft_secure_element
)

target_sources(smtc_flrp_crypto PRIVATE
    ${SMTC_FLRP_CRYPTO_DIR}/smtc_flrp_crypto.c
)

target_sources(smtc_flrp_crypto PRIVATE
    ${SMTC_FLRP_CRYPTO_DIR}/soft_secure_element/aes_fast.c
    ${SMTC_FLRP_CRYPTO_DIR}/soft_secure_element/cmac_fast.c
    ${SMTC_FLRP_CRYPTO_DIR}/soft_secure_element/soft_se_fast.c
)

# Compile aes.c with -O2 for speed (T-table code is aliasing-safe via memcpy)
set_source_files_properties(
    ${SMTC_FLRP_CRYPTO_DIR}/soft_secure_element/aes.c
    PROPERTIES COMPILE_FLAGS "-O2 -fno-strict-aliasing"
)

