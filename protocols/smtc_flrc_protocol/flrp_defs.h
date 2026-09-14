/*!
 * @file      flrp_defs.h
 *
 * @brief     Common definitions header for FLRC Protocol parameters.
 *
 * This file centralizes all the key configuration macros and parameters used by the application,
 * especially for FLRC operations. It provides default values for frequency, power,
 * payload length, modulation parameters, and other radio settings. These macros ensure that all
 * parts of the application use consistent radio settings and make it easy to adapt the configuration
 * for different regions, hardware, or use cases.
 *
 * 
 * The Clear BSD License
 * Copyright Semtech Corporation 2026. All rights reserved.
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

#ifndef FLRP_DEFS_H
#define FLRP_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 * Includes standard integer types and the radio abstraction layer definitions.
 */
#include <stdint.h>

#include "ral_defs.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 * These macros define the default configuration for the radio and LoRa modulation.
 */
#define SMTC_FLRP_EUI_LENGTH 8

#define SMTC_FLRP_NB_CHANNELS_MAX 16

#define SMTC_FLRP_PACKETS_IN_BURST_MAX 255

#define SMTC_FLRP_NB_BURSTS_MAX 255

#define SMTC_FLRP_TX_DELAY_MARGIN_MS 2  // With radio planner precision

#define SMTC_FLRP_MAC_DELAY_ADAPTIVE_LINK_INTERFRAME_MS 8  // MAX 25 (convert in 100e of us in an uint8_t)

#ifdef DEBUG_LOG
#define SMTC_FLRP_MAC_DELAY_START_BURST_MS 25  // MAX 25 (convert in 100e of us in an uint8_t)
#else
#define SMTC_FLRP_MAC_DELAY_START_BURST_MS 10  // MAX 25 (convert in 100e of us in an uint8_t)
#endif

#define SMTC_FLRP_MAC_DELAY_START_BURST_ACK_MS 8  // MAX 25 (convert in 100e of us in an uint8_t)

#define SMTC_FLRP_ADAPTIVE_LINK_DURING_BURST_ENABLED 0

#define SMTC_FLRP_SIZE_SYNC_WORD_MAX 4

#if( SMTC_FLRP_PACKETS_IN_BURST_MAX % 8 ) == 0
#define SMTC_FLRP_BITFIELD_PACKETS_IN_BURST_LENGTH ( SMTC_FLRP_PACKETS_IN_BURST_MAX / 8 )
#else
#define SMTC_FLRP_BITFIELD_PACKETS_IN_BURST_LENGTH ( ( SMTC_FLRP_PACKETS_IN_BURST_MAX / 8 ) + 1 )
#endif

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [in] a 1st value
 * \param [in] b 2nd value
 * \retval minValue Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*!
 * \brief Returns the maximum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#ifndef MAX
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif

/**
 * @brief Returns the least integer greater than or equal to integer of the division
 */
#ifndef CEIL_DIVISION
#define CEIL_DIVISION( a, b ) ( ( a % b == 0 ) ? ( a / b ) : ( ( a / b ) + 1 ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPE -----------------------------------------------------------
 */

typedef struct smtc_flrp_mac_radio_config_s
{
    uint32_t                      frequencies_hz[SMTC_FLRP_NB_CHANNELS_MAX];
    uint8_t                       nb_frequencies;
    uint8_t                       default_channel;
    ral_flrc_raw_bit_rate_t       raw_bit_rate;
    ral_flrc_cr_t                 cr;
    ral_flrc_pulse_shape_t        pulse_shape;
    ral_flrc_preamble_length_t    preambule_len;
    ral_flrc_sync_word_len_t      sync_word_len;
    ral_flrc_tx_syncword_t        tx_syncword_index;
    ral_flrc_rx_match_sync_word_t rx_match_sync_word;
    bool                          pld_is_fix;
    ral_flrc_crc_type_t           crc_type;
    int8_t                        tx_power_in_dbm; /*!< Transmission power in dBm. */

    uint32_t crc_seed;
    uint32_t crc_polynomial;
    uint8_t  sync_word[3][SMTC_FLRP_SIZE_SYNC_WORD_MAX];

    /*!< Protocol delays */
    uint16_t interframe_delay_us;               /*!< Delay between frames in the burst */
    uint16_t start_burst_delay_us;              /*!< Delay before starting a burst */
    uint16_t start_ack_delay_us;                /*!< Delay before starting a burst ack */
    uint16_t adaptive_link_interframe_delay_us; /*!< Delay before starting a FLRC REQ or FLRC ACK frames */

    uint8_t burst_target_per; /*!<  the maximal PER to consider a burst valid (in %) */

} smtc_flrp_mac_radio_config_t;

#endif  // FLRP_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
