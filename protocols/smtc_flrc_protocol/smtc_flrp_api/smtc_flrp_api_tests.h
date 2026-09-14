/**
 * @file      smtc_flrp_api_tests.h
 *
 * @brief     SMTC FLRC Protocol (FLRP) API for Semtech tests
 *
 * This header contains the prototypes for the SMTC FLRC Protocol (FLRP) API.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SMTC_FLRP_API_TESTS_H
#define SMTC_FLRP_API_TESTS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_flrp_api.h"
#include "../flrp_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct smtc_flrp_flrc_advanced_radio_config_s
{
    ral_flrc_cr_t                 cr;
    ral_flrc_pulse_shape_t        pulse_shape;
    ral_flrc_preamble_length_t    preambule_len;
    ral_flrc_sync_word_len_t      sync_word_len;
    ral_flrc_tx_syncword_t        tx_syncword_index;
    ral_flrc_rx_match_sync_word_t rx_match_sync_word;
    bool                          pld_is_fix;
    ral_flrc_crc_type_t           crc_type;
    uint32_t                      crc_seed;
    uint32_t                      crc_polynomial;
    uint8_t                       sync_word[3][SMTC_FLRP_SIZE_SYNC_WORD_MAX];

} smtc_flrp_flrc_advanced_radio_config_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Get the current advanced flrc radio configuration
 *
 */
smtc_flrp_flrc_advanced_radio_config_t smtc_flrp_get_current_advanced_flrc_radio_config( void );

/*!
 * \brief Set a new advanced flrc radio configuration
 *
 * \param [out] radio_config        The advanced flrc radio configuration structure
 *
 * \return SMTC_FLRP_RC_NOT_INIT if the protocol is not initialized yet.
 * SMTC_FLRP_RC_BUSY in case a transmission/reception is in progress.
 * SMTC_FLRP_RC_INVALID_PARAMS if the config is invalid.
 * SMTC_FLRP_RC_OK in case of success.
 */
smtc_flrp_return_code_t smtc_flrp_set_new_advanced_flrc_radio_config(
    smtc_flrp_flrc_advanced_radio_config_t radio_config );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_FLRP_API_TESTS_H

/* --- EOF ------------------------------------------------------------------ */
