/**
 * @file      smtc_flrp_mac_layer.h
 *
 * @brief     SMTC FLRC MAC Layer
 *
 * This header contains the prototypes for the SMTC FLRC MAC Layer.
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

#ifndef SMTC_FLRP_MAC_LAYER_H
#define SMTC_FLRP_MAC_LAYER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_rac.h"
#include "smtc_rac_api.h"
#include "smtc_flrp_api.h"
#include "flrp_defs.h"
#include "smtc_flrp_mac_adaptive_link.h"
#include "smtc_flrp_mac_serde.h"

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

typedef enum smtc_flrp_mac_layer_status_e
{
    SMTC_FLRP_MAC_LAYER_STATUS_OK = 0,
    SMTC_FLRP_MAC_LAYER_STATUS_INVALID_PARAMS,
    SMTC_FLRP_MAC_LAYER_STATUS_NOT_FOUND,
    SMTC_FLRP_MAC_LAYER_STATUS_TIMEOUT,
    SMTC_FLRP_MAC_LAYER_STATUS_BUSY,
    SMTC_FLRP_MAC_LAYER_STATUS_REFUSED,
    SMTC_FLRP_MAC_LAYER_STATUS_ERROR,
    SMTC_FLRP_MAC_LAYER_STATUS_ERROR_MEMORY,
} smtc_flrp_mac_layer_status_t;

typedef enum smtc_flrp_mac_layer_state_e
{
    SMTC_FLRP_MAC_LAYER_STATE_IDLE = 0,
    SMTC_FLRP_MAC_LAYER_STATE_FLRC_REQ,
    SMTC_FLRP_MAC_LAYER_STATE_FLRC_ACK,
    SMTC_FLRP_MAC_LAYER_STATE_BURST,
    SMTC_FLRP_MAC_LAYER_STATE_ACK,
} smtc_flrp_mac_layer_state_t;

typedef enum smtc_flrp_mac_layer_radio_event_e
{
    SMTC_FLRP_MAC_RADIO_EVENT_NONE = 0,
    SMTC_FLRP_MAC_RADIO_EVENT_TRX_OK,
    SMTC_FLRP_MAC_RADIO_EVENT_TRX_FAILED,
    SMTC_FLRP_MAC_RADIO_EVENT_ABORTED,
} smtc_flrp_mac_layer_radio_event_t;

typedef enum smtc_flrp_mac_transaction_type_e
{
    SMTC_FLRP_MAC_TX = 0,
    SMTC_FLRP_MAC_RX,
} smtc_flrp_mac_transaction_type_t;

typedef void ( *smtc_flrp_mac_layer_rx_done_f )( smtc_flrp_mac_layer_status_t status, uint32_t data_size,
                                                 uint32_t timestamp_ms, uint8_t* src_addr );
typedef void ( *smtc_flrp_mac_layer_tx_done_f )( smtc_flrp_mac_layer_status_t status, uint32_t timestamp_ms,
                                                 uint8_t* dest_addr );

typedef struct smtc_flrp_mac_burst_stats_s
{
    uint16_t nb_packets_received_ok;  /*!< Number of packets of the burst received and decoded successfully. */
    uint16_t nb_packets_check_error;  /*!< Number of packets of the burst received with a CRC or MIC error. */
    uint16_t nb_packets_received_nok; /*!< Number of wrong packets of the burst received. */
    int32_t  rssi_mean;               /*!< Received Signal Strength Indicator (RSSI) mean value of the burst. */
} smtc_flrp_mac_burst_stats_t;

typedef struct smtc_flrp_mac_layer_s
{
    smtc_flrp_mac_layer_state_t        state;
    smtc_flrp_mac_layer_radio_event_t  radio_event;
    smtc_flrp_mac_transaction_type_t   trx_type;
    uint8_t                            radio_access_id;
    smtc_rac_context_t*                transaction;
    smtc_rac_radio_flrc_burst_params_t rac_radio_flrc_burst_params;
    smtc_flrp_mac_radio_config_t       mac_radio_config;
    smtc_flrp_mac_layer_rx_done_f      mac_rx_done_cb;
    smtc_flrp_mac_layer_tx_done_f      mac_tx_done_cb;
    smtc_flrp_mac_burst_stats_t        burst_stats;
    smtc_flrp_burst_rx_stats_t         payload_stats;

    bool    crypto_enabled;
    uint8_t dev_eui[SMTC_FLRP_EUI_LENGTH];
    uint8_t destination_dev_eui[SMTC_FLRP_EUI_LENGTH];
    uint8_t rx_frame_dev_eui_filter_len; /*!< Number of dev_eui MSB bits to be used for selectivity
                                                     (0-63) */
    int32_t frequency_offset_hz;

    uint32_t number_of_packets; /*!< Number of packets to send all the payload*/
    uint8_t  packet_number;     /*!< Packet sequence number in this burst*/
    uint8_t  burst_number;      /*!< Burst number*/
    bool     burst_ack_enabled;

    /*!< Tx data */
    uint8_t* payload;

    /*!< Rx data */
    uint8_t* rx_data_buffer;       // Buffer to store the received data
    uint32_t rx_data_buffer_size;  // Size of the received data buffer
    uint16_t nb_packets_received;  // Number of packets received in all the payload
    uint32_t nb_bytes_received;    // Number of bytes received in all the payload

    /*!< Tx radio buffer */
    uint8_t  tx_payload_buffer[2][SMTC_FLRP_MAC_DATA_PACKET_LENGTH_MAX]; /*!< Pointer to the payload data buffer. */
    uint16_t tx_payload_buffer_size[2]; /*!< Size of the payload data user buffer in bytes. */
    bool     tx_payload_toggle;

    /*!< Rx radio buffer */
    uint8_t rx_payload_buffer[SMTC_FLRP_MAC_DATA_PACKET_LENGTH_MAX];  // Buffer to store the received from radio Fifo

    /*!< Adaptive link config */
    smtc_flrp_mac_adaptive_link_t               adaptive_link_config;
    smtc_flrp_mac_burst_info_in_adaptive_link_t burst_info;

    uint8_t burst_missed_packets_bitfield[SMTC_FLRP_BITFIELD_PACKETS_IN_BURST_LENGTH]; /*!< Bitfield of the packets
                                                                                              missed in the burst */
    bool     burst_retry_needed;
    uint32_t timestamp_start_burst_ms;
    uint32_t next_burst_start_delay_ms; /*!< Delay for the next burst (first, retry or multiburst)*/
    uint32_t
        burst_transmission_timeout; /*!< Time in ms to send a burst, after this timeout the transmission is aborted */

    uint32_t radio_end_timestamp_ms;

} smtc_flrp_mac_layer_t;

typedef struct smtc_flrp_mac_config_s
{
    uint8_t* dest_dev_eui;
    uint8_t  filter_len; /*!< Number of Slave DevEUI MSB bits to be used for selectivity (0-63) */
    uint16_t enabled_channels;
    smtc_flrp_link_adaptation_mode_t link_adaptation_mode;
    bool                             burst_ack_enabled;
    smtc_flrp_mac_radio_config_t     radio_config;
    int32_t                          freq_offset_hz;
} smtc_flrp_mac_config_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Initialize the mac layer module
 *
 * \param [inout] smtc_flrp_mac_layer          Structure of the mac layer
 * \param [in] dev_eui                         Device EUI
 * \param [in] priority                        Priority of the module (for radio planner)
 * \param [in] crypto_enabled                  If crypto in enabled
 * \param [in] tx_done_cb                      Callback for TX transactions
 * \param [in] rx_done_cb                      Callback for RX transactions
 *
 * \return SMTC_FLRP_MAC_LAYER_STATUS_ERROR if the crypto initialization failed,
 * SMTC_FLRP_MAC_LAYER_STATUS_OK otherwise.
 */
smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_init( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t* dev_eui,
                                                       uint8_t hook_id, bool crypto_enabled,
                                                       smtc_flrp_mac_layer_tx_done_f tx_done_cb,
                                                       smtc_flrp_mac_layer_rx_done_f rx_done_cb );

/*!
 * \brief Function to call periodically to process the radio events on hold.
 *
 * \param [inout] smtc_flrp_mac_layer          Structure of the mac layer
 */

void smtc_flrp_mac_layer_process( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );

/*!
 * \brief Start the processus to send data
 *
 * \param [inout] smtc_flrp_mac_layer          Structure of the mac layer
 * \param [in] start_time                      Timestamp (in ms) of the start of the transmission
 * \param [in] data_buffer                     Pointer to the data to send
 * \param [in] buffer_size                     Size of the data to send
 * \param [in] mac_config                      Mac radio configuration for this transaction
 *
 * \return SMTC_FLRP_MAC_LAYER_STATUS_BUSY if a transfer is already in progress
 * SMTC_FLRP_MAC_LAYER_STATUS_INVALID_PARAMS if the size of the buffer to send is too big or the radio configuration is
 * invalid.
 * SMTC_FLRP_MAC_LAYER_STATUS_ERROR if radio planner failed to program the TX.
 * SMTC_FLRP_MAC_LAYER_STATUS_OK otherwise.
 */
smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_start_tx_transaction( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                       uint32_t start_time, uint8_t* data_buffer,
                                                                       uint32_t               buffer_size,
                                                                       const smtc_flrp_mac_config_t* mac_config );

/*!
 * \brief Start the processus to receive data
 *
 * \param [inout] smtc_flrp_mac_layer          Structure of the mac layer
 * \param [in] start_time                      Timestamp (in ms) of the start of the reception
 * \param [in] data_buffer                     Pointer to the data to receive
 * \param [in] buffer_size                     Size of the data to receive
 * \param [in] mac_config                      Mac radio configuration for this transaction
 *
 * \return SMTC_FLRP_MAC_LAYER_STATUS_BUSY if a transfer is already in progress
 * SMTC_FLRP_MAC_LAYER_STATUS_INVALID_PARAMS if the size of the buffer to send is too big or the radio configuration is
 * invalid.
 * SMTC_FLRP_MAC_LAYER_STATUS_ERROR if radio planner failed to program the TX.
 * SMTC_FLRP_MAC_LAYER_STATUS_OK otherwise.
 */

smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_start_rx_transaction( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                       uint32_t start_time, uint8_t* data_buffer,
                                                                       uint32_t               buffer_size,
                                                                       const smtc_flrp_mac_config_t* mac_config );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_FLRP_MAC_LAYER_H

/* --- EOF ------------------------------------------------------------------ */
