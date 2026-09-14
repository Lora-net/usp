/**
 * @file      smtc_flrp_mac_config.h
 *
 * @brief     SMTC FLRC MAC radio configuration
 *
 * This header contains the prototypes for the SMTC FLRC MAC radio configuration.
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

#ifndef SMTC_FLRP_MAC_CONFIG_H
#define SMTC_FLRP_MAC_CONFIG_H

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#define MAC_INVALID_CHANNEL UINT8_MAX

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Configure the radio parameters for the FLRC modulation in smtc_rac_context_t
 *
 * \param [inout] transaction           Structure of smtc_rac_context_t
 * \param [in] mac_radio_config         Radio configuration
 * \param [in] freq                     Frequency used (in Hz)
 * \param [in] freq_offset_hz           Frequency offsed (in Hz)
 * \param [in] raw_bit_rate             Data rate used
 * \param [in] cr                       Coding rate used
 */
void smtc_flrp_mac_set_radio_params_flrc_mod( smtc_rac_context_t*           transaction,
                                              smtc_flrp_mac_radio_config_t* mac_radio_config, uint32_t freq,
                                              int32_t freq_offset_hz, ral_flrc_raw_bit_rate_t raw_bit_rate,
                                              ral_flrc_cr_t cr );

/*!
 * \brief Configure the radio parameters for the FLRC BURST  modulation in smtc_rac_context_t
 *
 * \param [inout] transaction               Structure of smtc_rac_context_t
 * \param [in] mac_radio_config             Radio configuration
 * \param [in] freq                         Frequency used (in Hz)
 * \param [in] freq_offset_hz               Frequency offsed (in Hz)
 * \param [in] raw_bit_rate                 Data rate used
 * \param [in] cr                           Coding rate used
 * \param [in] burst_interframe_delay_us    Delay of interframe is us during the burst
 * \param [in] crc_disabled                 If CRC check is disabled
 */
void smtc_flrp_mac_set_radio_params_flrc_burst_mod( smtc_rac_context_t*           transaction,
                                                    smtc_flrp_mac_radio_config_t* mac_radio_config, uint32_t freq,
                                                    int32_t freq_offset_hz, ral_flrc_raw_bit_rate_t raw_bit_rate,
                                                    ral_flrc_cr_t cr, uint32_t burst_interframe_delay_us,
                                                    bool crc_disabled );

/*!
 * \brief Get the channel index corresponding to the frequency set.
 *
 * \param [in] mac_radio_config             Radio configuration
 * \param [in] frequency                    Frequency (in Hz)
 *
 * \return The channel corresponding to the frequency. Return MAC_INVALID_CHANNEL if the frequency is not set.
 */
uint8_t smtc_flrp_mac_get_freq_channel( const smtc_flrp_mac_radio_config_t* mac_radio_config, uint32_t frequency );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_FLRP_MAC_CONFIG_H

/* --- EOF ------------------------------------------------------------------ */
