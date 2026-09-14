/**
 * @file      smtc_flrp_mac_config.c
 *
 * @brief    smtc_flrp_mac_config implementation to set the radio configuration
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "smtc_flrp_mac_config.h"
#include "flrp_configuration.h"
#include "flrp_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_flrp_mac_set_radio_params_flrc_mod( smtc_rac_context_t*           transaction,
                                              smtc_flrp_mac_radio_config_t* mac_radio_config, uint32_t freq,
                                              int32_t freq_offset_hz, ral_flrc_raw_bit_rate_t raw_bit_rate,
                                              ral_flrc_cr_t cr )
{
    transaction->modulation_type = SMTC_RAC_MODULATION_FLRC;

    transaction->radio_params.flrc.frequency_in_hz           = freq;
    transaction->radio_params.flrc.rx_frequency_offset_in_hz = freq_offset_hz;
    transaction->radio_params.flrc.raw_bit_rate              = raw_bit_rate;
    transaction->radio_params.flrc.tx_power_in_dbm           = mac_radio_config->tx_power_in_dbm;
    transaction->radio_params.flrc.cr                        = cr;
    transaction->radio_params.flrc.pulse_shape               = mac_radio_config->pulse_shape;
    transaction->radio_params.flrc.preamble_len              = mac_radio_config->preambule_len;
    transaction->radio_params.flrc.sync_word_len             = mac_radio_config->sync_word_len;
    transaction->radio_params.flrc.tx_syncword_index         = mac_radio_config->tx_syncword_index;
    transaction->radio_params.flrc.match_sync_word           = mac_radio_config->rx_match_sync_word;
    transaction->radio_params.flrc.pld_is_fix                = mac_radio_config->pld_is_fix;
    transaction->radio_params.flrc.crc_type                  = mac_radio_config->crc_type;

    transaction->radio_params.flrc.sync_word[0] = &( mac_radio_config->sync_word[0] )[0];
    transaction->radio_params.flrc.sync_word[1] = &( mac_radio_config->sync_word[1] )[0];
    transaction->radio_params.flrc.sync_word[2] = &( mac_radio_config->sync_word[2] )[0];

    transaction->radio_params.flrc.crc_seed       = mac_radio_config->crc_seed;
    transaction->radio_params.flrc.crc_polynomial = mac_radio_config->crc_polynomial;
}

void smtc_flrp_mac_set_radio_params_flrc_burst_mod( smtc_rac_context_t*           transaction,
                                                    smtc_flrp_mac_radio_config_t* mac_radio_config, uint32_t freq,
                                                    int32_t freq_offset_hz, ral_flrc_raw_bit_rate_t raw_bit_rate,
                                                    ral_flrc_cr_t cr, uint32_t burst_interframe_delay_us,
                                                    bool crc_disabled )
{
    transaction->modulation_type = SMTC_RAC_MODULATION_FLRC_BURST;

    transaction->radio_params.flrc_burst.frequency_in_hz           = freq;
    transaction->radio_params.flrc_burst.rx_frequency_offset_in_hz = freq_offset_hz;
    transaction->radio_params.flrc_burst.raw_bit_rate              = raw_bit_rate;
    transaction->radio_params.flrc_burst.tx_power_in_dbm           = mac_radio_config->tx_power_in_dbm;
    transaction->radio_params.flrc_burst.cr                        = cr;
    transaction->radio_params.flrc_burst.pulse_shape               = mac_radio_config->pulse_shape;
    transaction->radio_params.flrc_burst.preamble_len              = mac_radio_config->preambule_len;
    transaction->radio_params.flrc_burst.sync_word_len             = mac_radio_config->sync_word_len;
    transaction->radio_params.flrc_burst.tx_syncword_index         = mac_radio_config->tx_syncword_index;
    transaction->radio_params.flrc_burst.match_sync_word           = mac_radio_config->rx_match_sync_word;
    transaction->radio_params.flrc_burst.pld_is_fix                = mac_radio_config->pld_is_fix;
    if( crc_disabled )
    {
        transaction->radio_params.flrc_burst.crc_type = RAL_FLRC_CRC_OFF;
    }
    else
    {
        transaction->radio_params.flrc_burst.crc_type = mac_radio_config->crc_type;
    }

    transaction->radio_params.flrc_burst.min_interframe_delay_us = burst_interframe_delay_us;

    transaction->radio_params.flrc_burst.sync_word[0] = &mac_radio_config->sync_word[0][0];
    transaction->radio_params.flrc_burst.sync_word[1] = &mac_radio_config->sync_word[1][0];
    transaction->radio_params.flrc_burst.sync_word[2] = &mac_radio_config->sync_word[2][0];

    transaction->radio_params.flrc_burst.crc_seed       = mac_radio_config->crc_seed;
    transaction->radio_params.flrc_burst.crc_polynomial = mac_radio_config->crc_polynomial;
}

uint8_t smtc_flrp_mac_get_freq_channel( const smtc_flrp_mac_radio_config_t* mac_radio_config, uint32_t frequency )
{
    uint8_t nb_freq = mac_radio_config->nb_frequencies;
    //for( uint8_t i = 0; i < mac_radio_config->nb_frequencies; i++ )
    for( uint8_t i = 0; i < nb_freq; i++ )
    {
        if( mac_radio_config->frequencies_hz[i] == frequency )
        {
            return i;
        }
    }
    return MAC_INVALID_CHANNEL;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
