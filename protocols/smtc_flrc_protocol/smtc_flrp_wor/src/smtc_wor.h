/**
 * @file      smtc_wor.h
 *
 * @brief     SMTC FLRC WOR
 *
 * This header contains the prototypes for the SMTC FLRC WOR.
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

#ifndef SMTC_WOR_H
#define SMTC_WOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_flrp_wor.h"
#include "flrp_defs.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define SMTC_WOR_PAYLOAD_HEADER_SIZE 2
#define SMTC_WOR_PAYLOAD_MAX_DATA_SIZE 25
#define SMTC_WOR_PAYLOAD_SIGN_SIZE 64

#define SMTC_WOR_PAYLOAD_MAX_SIZE \
    ( SMTC_WOR_PAYLOAD_HEADER_SIZE + SMTC_WOR_PAYLOAD_MAX_DATA_SIZE + SMTC_WOR_PAYLOAD_SIGN_SIZE )

#define SMTC_WOR_TX_RX_BUFFER_MAX_SIZE ( 255 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief SMTC WOR status
 */
typedef enum smtc_wor_status_e
{
    SMTC_WOR_STATUS_SUCCESS = 0,
    SMTC_WOR_STATUS_FAILED, /* Radio error (Timeout, CRC error, transfert refused ...)*/
    SMTC_WOR_STATUS_ABORT,
    SMTC_WOR_STATUS_BUSY,
    SMTC_WOR_STATUS_ERROR,
    SMTC_WOR_STATUS_INVALID_PARAMETER,
    SMTC_WOR_STATUS_NOT_SUPPORTED,
    SMTC_WOR_STATUS_NOT_INITIALIZED,
} smtc_wor_status_t;

/**
 * @brief SMTC WOR state
 */
typedef enum smtc_wor_state_e
{
    SMTC_WOR_STATE_IDLE,
    SMTC_WOR_STATE_WOR,
    SMTC_WOR_STATE_WOR_ACK,
} smtc_wor_state_t;

/**
 * @brief SMTC WOR type
 */
typedef enum smtc_wor_type_e
{
    SMTC_WOR_TYPE_SATELLITE_BEACON = 0,
    SMTC_WOR_TYPE_SATELLITE_EPHEMERIS_BROADCAST = 1,
    SMTC_WOR_TYPE_SATELLITE_DATA_BROADCAST = 2,
    SMTC_WOR_TYPE_DRIVE_BY = 3,
    SMTC_WOR_TYPE_LOW_POWER_NETWORK_DISCOVERY = 4,
    SMTC_WOR_TYPE_CERTIFICATE_UPDATE = 5,
    
    //RFU - From 6 to 55

    //PORPRIETARY From 56 to 64
    SMTC_WOR_TYPE_FLRC = 56,
    SMTC_WOR_TYPE_FLRS = 57,

    //From 65 it is out of range
    SMTC_WOR_TYPE_UNKNOWN = 255,    //Out of range
} smtc_wor_type_t;

typedef struct smtc_wor_header_s
{
    uint8_t         cipher_id;
    uint8_t         version;
    smtc_wor_type_t beacon_type;
} smtc_wor_header_t;

typedef struct smtc_wor_data_s
{
    smtc_wor_type_t type;
    union
    {
        smtc_flrp_wor_data_t flrc;
    } u;
    uint8_t signature[SMTC_WOR_PAYLOAD_SIGN_SIZE];

} smtc_wor_data_t;

typedef struct smtc_wor_ack_data_s
{
    smtc_wor_type_t type;
    union
    {
        smtc_flrp_wor_ack_data_t flrc;
    } u;

} smtc_wor_ack_data_t;

typedef struct smtc_wor_rx_info_for_ack_s
{
    union
    {
        smtc_flrp_wor_rx_info_for_ack_t flrc;
    } u;

} smtc_wor_rx_info_for_ack_t;

typedef struct smtc_wor_rx_measurements_s
{
    int32_t freq_offset_hz;
} smtc_wor_rx_measurements_t;

typedef void ( *smtc_wor_done_f )( smtc_wor_status_t status, smtc_flrp_wor_rx_stats_t rx_metrics );

typedef struct smtc_wor_static_radio_config_s
{
    uint8_t                  tx_power_dbm;
    uint16_t                 time_preamble_ms;
    uint16_t                 wor_ack_preamble_len_symb;
    uint32_t                 listen_period_ms;
    ral_lora_cr_t            cr;               //!< LoRa Coding Rate
    ral_lora_pkt_len_modes_t header_type;      //!< LoRa Header Type
    bool                     invert_iq_is_on;  //!< LoRa IQ polarity setup
    uint8_t                  crc_is_on;
    smtc_rac_lora_syncword_t sync_word;
    uint16_t                 wor_ack_delay_ms;
    uint32_t                 rx_timeout;
    uint32_t                 wor_ack_rx_timeout;
} smtc_wor_static_radio_config_t;

typedef struct smtc_wor_s
{
    smtc_wor_state_t               state;
    smtc_rac_context_t*            transaction;
    uint8_t                        radio_access_id;
    smtc_wor_static_radio_config_t smtc_wor_radio_config;
    smtc_wor_done_f                smtc_wor_done_cb;
    smtc_wor_data_t                wor_data_trx;
    smtc_wor_rx_measurements_t     wor_rx_measurements;
    smtc_wor_ack_data_t            wor_ack_data_trx;

    uint32_t crystal_error;

    uint8_t         tx_payload[SMTC_WOR_PAYLOAD_MAX_SIZE];
    uint8_t         rx_payload[SMTC_WOR_TX_RX_BUFFER_MAX_SIZE];
    smtc_wor_type_t type;
    bool            ack_required;
    bool            data_ready_to_be_sent;
    uint8_t*        dev_addr;

    // For WOR RX
    smtc_wor_rx_info_for_ack_t info_wor_ack;

    smtc_flrp_wor_rx_stats_t rx_metrics;

} smtc_wor_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Initialize the WOR.
 * \param [in] smtc_wor_obj     WOR structure
 * \param [in] wor_config       WOR radio configuration
 * \param [in] hook_id          Radio planner hook ID
 * \param [in] wor_done_cb      Function called when a WOR transfer is done
 */
void smtc_wor_init( smtc_wor_t* smtc_wor_obj, smtc_wor_static_radio_config_t wor_config, uint8_t hook_id,
                    smtc_wor_done_f wor_done_cb );

/*!
 * \brief Enter the state IDLE and call the callback set in the initialization.
 * \param [in] smtc_wor_obj     WOR structure
 * \param [in] status           Status of the transaction
 */
void enter_idle_state_and_send_wor_cb( smtc_wor_t* smtc_wor_obj, smtc_wor_status_t status );

/*!
 * \brief Convert RAC status to WOR status
 * \param [in] rac_status    RAC status
 *
 * \return The WOR status corresponding
 */
smtc_wor_status_t convert_rac_status_to_wor_status( smtc_rac_return_code_t rac_status );

/*!
 * \brief Set the changing parameters of the radio config, in the rac context for the next transfer.
 * \param [in] smtc_wor_obj     WOR structure
 * \param [in] radio_config     Structure of the part of the radio config that can be changed by user.
 */
void smtc_wor_set_dynamic_radio_config( smtc_wor_t* smtc_wor_obj, smtc_flrp_wor_radio_config_t radio_config );

/*!
 * \brief Compute the time (in us) of a symbol (depending on SF and BW)
 * \param [in] bw    Lora bandwidth
 * \param [in] sf    Lora spreading factor
 *
 * \return The time of one symbol in us
 */
uint32_t smtc_wor_get_single_symbol_time_us( ral_lora_bw_t bw, ral_lora_sf_t sf );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_WOR_H

/* --- EOF ------------------------------------------------------------------ */
