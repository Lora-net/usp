/**
 * @file      smtc_flrp_mac_layer.c
 *
 * @brief     smtc_flrp_mac_layer api implementation
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

#include "flrp_configuration.h"
#include "smtc_flrp_utils.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_rac.h"
#include "smtc_rac_api.h"
#include "smtc_flrp_mac_layer.h"
#include "smtc_flrp_core.h"
#include "smtc_flrp_mac_config.h"
#include "smtc_flrp_mac_adaptive_link.h"
#include "smtc_flrp_mac_serde.h"
#include "smtc_flrp_crypto.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define RAL_RADIO_POINTER &( smtc_rac_get_rp( )->radio->ral )

#define MAC_BURST_ACK_DATA_RATE_DIVIDE 4

#define MAC_BURST_INTERFRAME_DURATION_INCERTITUDE_PERCENTAGE 10
#define MAC_BURST_TIMEOUT_NB_TOA 3
#define MAC_BURST_TIMEOUT_MS_MIN 100

#define MAC_DATA_SIZE_MAX 0xFFFFFF

#define MAC_BURST_MAX_TIME_MS 400
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

static void smtc_flrp_mac_reset_flrc_stats( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );

static void add_bit_to_rx_mask( smtc_flrp_mac_layer_t* mac, smtc_flrp_exchange_phase_success_bit_t bit );
static void remove_bit_from_rx_mask( smtc_flrp_mac_layer_t* mac, smtc_flrp_exchange_phase_success_bit_t bit );

static smtc_flrp_mac_layer_status_t convert_rac_status_to_mac_status( smtc_rac_return_code_t rac_status );

static uint32_t get_this_burst_duration_ms( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );
static uint32_t get_burst_duration_ms( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t nb_packets_in_burst,
                                       ral_flrc_raw_bit_rate_t raw_bit_rate );
static uint32_t get_packet_duration_us( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint16_t size_payload,
                                        ral_flrc_raw_bit_rate_t raw_bit_rate );
static uint8_t  get_nb_packets_in_burst( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );
static uint16_t get_nb_bursts( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );
static bool is_the_last_packet_in_payload( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t packet_number_in_burst );
static uint16_t get_packet_idx_in_payload( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t packet_number_in_burst );

static uint16_t get_packet_payload_size( ral_flrc_cr_t cr );

static uint32_t get_burst_timeout( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_set_common_config( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                     const smtc_flrp_mac_config_t* mac_config );

static bool is_burst_transmission_possible_before_timeout( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                           uint32_t               radio_end_timestamp_ms );

static ral_flrc_raw_bit_rate_t divide_raw_bit_rate( ral_flrc_raw_bit_rate_t raw_bit_rate, uint8_t divider );
static ral_flrc_raw_bit_rate_t get_raw_bit_rate_for_burst_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );

static void missed_packets_bitfield_set_rx_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t packet_number );
static void missed_packets_bitfield_reset( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t number_of_packets );
static uint8_t missed_packets_bitfield_get_nb_missed_packets( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );
static smtc_flrp_mac_layer_status_t missed_packets_bitfield_get_next_missed_packet(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t* missed_packet_idx );

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_construct_data_packet(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t packet_counter, uint8_t* payload, uint32_t payload_length );
static void smtc_flrp_mac_layer_construct_missing_data_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                               uint8_t* payload, uint32_t payload_length );

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_compute_packet_payload_size(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint32_t buffer_size );
static void reset_tx_burst_param( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );
static void reset_rx_burst_param( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );
static void stats_increment_packet_received( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, bool packet_check_error,
                                             bool packet_invalid );

static void smtc_flrp_mac_set_channels_to_test_in_adaptive_link_phase( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                       uint16_t               enabled_channels_mask,
                                                                       uint8_t*               nb_freq_to_test,
                                                                       uint32_t*              frequencies_to_test_hz );

static uint32_t smtc_flrp_mac_channel_get_frequency( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t channel_idx );
static void     compute_burst_info_in_flrc_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );

static void smtc_flrp_mac_prepare_for_flrc_req( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                uint16_t               enable_channels_mask );

/********** TX transaction functions **********/
/**********************************************/

static void smtc_flrp_mac_layer_prepare_tx_data_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                        uint8_t* data_buffer, uint32_t buffer_size,
                                                        uint8_t nb_packets_in_burst );
static void smtc_flrp_mac_layer_prepare_tx_burst_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_send_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                     uint32_t               timestamp_ms );

/********** RX transaction functions **********/
/**********************************************/

static void smtc_flrp_mac_layer_prepare_rx_data_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                        uint8_t* data_buffer, uint32_t buffer_size,
                                                        uint32_t packet_size, uint8_t nb_packets_in_burst );
static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_listen_rx_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                          uint32_t               timestamp_ms,
                                                                          bool                   transaction_asap );
static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_listen_rx_packet_asap(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint32_t timestamp_ms );
static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_listen_rx_packet_scheduled(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint32_t timestamp_ms );
static void smtc_flrp_mac_layer_prepare_rx_burst_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t* data_buffer,
                                                      uint32_t radio_end_timestamp_ms );

/********** Functions handling rx frame *******/
/**********************************************/
static bool smtc_flrp_mac_handle_burst_data( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t* rx_payload,
                                             uint16_t rx_payload_size );
static smtc_flrp_mac_layer_status_t smtc_flrp_mac_handle_burst_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );

/********** FSM functions **********/
/***********************************/

static void go_to_burst_state_if_remaining_burst( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );

static void go_to_mac_state( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, smtc_flrp_mac_layer_state_t state );

static void tx_fsm_go_to_state( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, smtc_flrp_mac_layer_state_t state );
static void rx_fsm_go_to_state( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, smtc_flrp_mac_layer_state_t state );

static smtc_flrp_mac_layer_status_t enter_state_flrc_req_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                             uint32_t               start_timestamp_ms );
static smtc_flrp_mac_layer_status_t enter_state_flrc_req_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                             uint32_t               start_timestamp_ms );
static smtc_flrp_mac_layer_status_t enter_state_flrc_ack_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                             uint32_t               radio_end_timestamp_ms );
static smtc_flrp_mac_layer_status_t enter_state_flrc_ack_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                             uint32_t               radio_end_timestamp_ms );
static smtc_flrp_mac_layer_status_t enter_state_burst_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                          uint32_t               start_timestamp_ms );
static smtc_flrp_mac_layer_status_t enter_state_burst_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                          uint32_t               start_timestamp_ms );
static smtc_flrp_mac_layer_status_t enter_state_burst_retry_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                uint32_t               radio_end_timestamp_ms );
static smtc_flrp_mac_layer_status_t enter_state_burst_retry_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                uint32_t               radio_end_timestamp_ms );
static smtc_flrp_mac_layer_status_t enter_state_burst_ack_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                              uint32_t               radio_end_timestamp_ms );
static smtc_flrp_mac_layer_status_t enter_state_burst_ack_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                              uint32_t               radio_end_timestamp_ms );

static void enter_state_idle( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer );

static void go_to_state_idle_and_return_status( smtc_flrp_mac_layer_t*       smtc_flrp_mac_layer,
                                                smtc_flrp_mac_layer_status_t mac_status, uint32_t data_size );

/********** Callback functions **********/
/****************************************/

static void pre_tx_transaction_callback( void );
static void post_tx_transaction_callback( rp_status_t status );

static void pre_rx_transaction_callback( void );
static void post_rx_transaction_callback( rp_status_t status );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_init( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t* dev_eui,
                                                       uint8_t hook_id, bool crypto_enabled,
                                                       smtc_flrp_mac_layer_tx_done_f tx_done_cb,
                                                       smtc_flrp_mac_layer_rx_done_f rx_done_cb )
{
    memset( smtc_flrp_mac_layer, 0, sizeof( smtc_flrp_mac_layer_t ) );

    smtc_flrp_mac_layer->mac_rx_done_cb = rx_done_cb;
    smtc_flrp_mac_layer->mac_tx_done_cb = tx_done_cb;

    smtc_flrp_mac_layer->crypto_enabled  = crypto_enabled;
    smtc_flrp_mac_layer->radio_access_id = smtc_rac_open_radio( ( smtc_rac_priority_t ) hook_id );
    smtc_flrp_mac_layer->transaction     = smtc_rac_get_context( smtc_flrp_mac_layer->radio_access_id );
    memcpy( smtc_flrp_mac_layer->dev_eui, dev_eui, SMTC_FLRP_EUI_LENGTH );

    smtc_rac_set_context_private( smtc_flrp_mac_layer->radio_access_id, smtc_flrp_mac_layer );

    if( smtc_flrp_mac_layer->crypto_enabled )
    {
        smtc_flrp_crypto_init( );
        smtc_flrp_crypto_return_code_t crypto_return_code =
            smtc_flrp_crypto_set_key( SMTC_SE_APP_KEY, flrc_default_key, 0 );
        if( crypto_return_code != SMTC_FLRP_CRYPTO_RC_SUCCESS )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set key (error %u)\n", crypto_return_code );
            return SMTC_FLRP_MAC_LAYER_STATUS_ERROR;
        }
    }

    go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_IDLE );

    return SMTC_FLRP_MAC_LAYER_STATUS_OK;
}

void smtc_flrp_mac_layer_process( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    smtc_flrp_mac_layer_state_t       current_state       = smtc_flrp_mac_layer->state;
    smtc_flrp_mac_layer_radio_event_t current_radio_event = smtc_flrp_mac_layer->radio_event;

    if( smtc_flrp_mac_layer->radio_event != SMTC_FLRP_MAC_RADIO_EVENT_NONE )
    {
        smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_NONE;

        switch( current_state )
        {
        case SMTC_FLRP_MAC_LAYER_STATE_IDLE:
            break;
        case SMTC_FLRP_MAC_LAYER_STATE_FLRC_REQ:
            if( smtc_flrp_mac_layer->adaptive_link_config.frequency_test_idx <
                smtc_flrp_mac_layer->adaptive_link_config.frequencies_to_test_size - 1 )
            {
                smtc_flrp_mac_layer->adaptive_link_config.frequency_test_idx++;
                go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_FLRC_REQ );
            }
            else
            {
                if( smtc_flrp_mac_layer->adaptive_link_config.flrc_ack_enabled )
                {
                    smtc_flrp_mac_layer->adaptive_link_config.frequency_test_idx = 0;
                    add_bit_to_rx_mask( smtc_flrp_mac_layer, SMTC_FLRP_EXCHANGE_ADAPTIVE_FLRC_REQ_SUCCESS );
                    go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_FLRC_ACK );
                }
                else
                {
                    if( ( smtc_flrp_mac_layer->trx_type == SMTC_FLRP_MAC_RX ) &&
                        !smtc_flrp_mac_at_least_one_flrc_req_was_received(
                            smtc_flrp_mac_layer->adaptive_link_config.flrc_req_rssi,
                            smtc_flrp_mac_layer->adaptive_link_config.frequencies_to_test_size ) )
                    {
                        go_to_state_idle_and_return_status( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATUS_TIMEOUT,
                                                            0 );
                    }
                    else
                    {
                        add_bit_to_rx_mask( smtc_flrp_mac_layer, SMTC_FLRP_EXCHANGE_ADAPTIVE_FLRC_REQ_SUCCESS );
                        go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_BURST );
                    }
                }
            }
            break;
        case SMTC_FLRP_MAC_LAYER_STATE_FLRC_ACK:
            if( smtc_flrp_mac_layer->adaptive_link_config.frequency_test_idx <
                smtc_flrp_mac_layer->adaptive_link_config.frequencies_to_test_size - 1 )
            {
                smtc_flrp_mac_layer->adaptive_link_config.frequency_test_idx++;
                go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_FLRC_ACK );
            }
            else
            {
                smtc_flrp_mac_adaptive_link_status_t adap_link_status =
                    smtc_flrp_mac_layer->adaptive_link_config.last_flrc_ack_transfer_status;
                if( adap_link_status == SMTC_FLRP_MAC_ADAPTIVE_LINK_ACCEPTED )
                {
                    add_bit_to_rx_mask( smtc_flrp_mac_layer, SMTC_FLRP_EXCHANGE_ADAPTIVE_FLRC_ACK_SUCCESS );
                    go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_BURST );
                }
                else if( ( adap_link_status == SMTC_FLRP_MAC_ADAPTIVE_LINK_RETRY ) &&
                         smtc_flrp_mac_layer->adaptive_link_config.retry_enabled )  // Only one retry allowed
                {
                    // Retry on all channels
                    uint16_t enable_all_channels_mask =
                        ( 1 << smtc_flrp_mac_layer->mac_radio_config.nb_frequencies ) - 1;
                    smtc_flrp_mac_layer->adaptive_link_config.retry_enabled = false;
                    smtc_flrp_mac_prepare_for_flrc_req( smtc_flrp_mac_layer, enable_all_channels_mask );

                    remove_bit_from_rx_mask( smtc_flrp_mac_layer, SMTC_FLRP_EXCHANGE_ADAPTIVE_FLRC_REQ_SUCCESS );
                    remove_bit_from_rx_mask( smtc_flrp_mac_layer, SMTC_FLRP_EXCHANGE_ADAPTIVE_FLRC_ACK_SUCCESS );

                    go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_FLRC_REQ );
                }
                else
                {
                    go_to_state_idle_and_return_status( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATUS_REFUSED, 0 );
                }
            }
            break;
        case SMTC_FLRP_MAC_LAYER_STATE_BURST:
            if( smtc_flrp_mac_layer->burst_ack_enabled )
            {
                go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_ACK );
            }
            else
            {
                go_to_burst_state_if_remaining_burst( smtc_flrp_mac_layer );
            }
            break;
        case SMTC_FLRP_MAC_LAYER_STATE_ACK:
            if( current_radio_event == SMTC_FLRP_MAC_RADIO_EVENT_TRX_FAILED )
            {
                go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_ACK );
            }
            else
            {
                go_to_burst_state_if_remaining_burst( smtc_flrp_mac_layer );
            }
            break;
        default:
            break;
        }
    }
}

smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_start_tx_transaction( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                       uint32_t start_time, uint8_t* data_buffer,
                                                                       uint32_t               buffer_size,
                                                                       const smtc_flrp_mac_config_t* mac_config )
{
    if( smtc_flrp_mac_layer->state != SMTC_FLRP_MAC_LAYER_STATE_IDLE )
    {
        return SMTC_FLRP_MAC_LAYER_STATUS_BUSY;
    }

    if( buffer_size > MAC_DATA_SIZE_MAX )
    {
        return SMTC_FLRP_MAC_LAYER_STATUS_INVALID_PARAMS;
    }

    smtc_flrp_mac_layer->trx_type                  = SMTC_FLRP_MAC_TX;
    smtc_flrp_mac_layer->payload                   = data_buffer;
    smtc_flrp_mac_layer->burst_info.payload_length = buffer_size;

    smtc_flrp_mac_layer_status_t status = smtc_flrp_mac_set_common_config( smtc_flrp_mac_layer, mac_config );
    if( status != SMTC_FLRP_MAC_LAYER_STATUS_OK )
    {
        return status;
    }
    reset_tx_burst_param( smtc_flrp_mac_layer );

    bit_mask_t* mp = get_mask( );
    if( mp != NULL )
    {
        *mp = 0;
    }

    if( smtc_flrp_mac_layer->adaptive_link_config.mode == SMTC_FLRP_LINK_ADAPTATION_DISABLED )
    {
        smtc_flrp_mac_layer->state = SMTC_FLRP_MAC_LAYER_STATE_BURST;
        status                     = enter_state_burst_tx( smtc_flrp_mac_layer, start_time );
    }
    else
    {
        smtc_flrp_mac_layer->state = SMTC_FLRP_MAC_LAYER_STATE_FLRC_REQ;
        status                     = enter_state_flrc_req_tx( smtc_flrp_mac_layer, start_time );
    }

    return status;
}

smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_start_rx_transaction( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                       uint32_t start_time, uint8_t* data_buffer,
                                                                       uint32_t               buffer_size,
                                                                       const smtc_flrp_mac_config_t* mac_config )
{
    if( smtc_flrp_mac_layer->state != SMTC_FLRP_MAC_LAYER_STATE_IDLE )
    {
        return SMTC_FLRP_MAC_LAYER_STATUS_BUSY;
    }

    if( buffer_size > MAC_DATA_SIZE_MAX )
    {
        return SMTC_FLRP_MAC_LAYER_STATUS_INVALID_PARAMS;
    }

    smtc_flrp_mac_layer->trx_type            = SMTC_FLRP_MAC_RX;
    smtc_flrp_mac_layer->rx_data_buffer      = data_buffer;
    smtc_flrp_mac_layer->rx_data_buffer_size = buffer_size;
    smtc_flrp_mac_layer->burst_info.payload_length =
        smtc_flrp_mac_layer->rx_data_buffer_size;  // Default, will the modified if adaptive link is enabled
    smtc_flrp_mac_layer->rx_frame_dev_eui_filter_len = mac_config->filter_len;

    smtc_flrp_mac_layer_status_t status = smtc_flrp_mac_set_common_config( smtc_flrp_mac_layer, mac_config );
    if( status != SMTC_FLRP_MAC_LAYER_STATUS_OK )
    {
        return status;
    }
    reset_rx_burst_param( smtc_flrp_mac_layer );

    // Reset payload stats
    memset( &smtc_flrp_mac_layer->payload_stats, 0, sizeof( smtc_flrp_burst_rx_stats_t ) );
    smtc_flrp_mac_layer->payload_stats.payload_size_expected = smtc_flrp_mac_layer->burst_info.payload_length;
    smtc_flrp_mac_layer->payload_stats.nb_packets_expected   = smtc_flrp_mac_layer->number_of_packets;

    bit_mask_t  phase = 0;
    bit_mask_t* mp    = get_mask( );
    if( mp != NULL )
    {
        phase = *mp;
        *mp   = 0;
    }
    if( smtc_flrp_mac_layer->adaptive_link_config.mode != SMTC_FLRP_LINK_ADAPTATION_DISABLED )
    {
        phase |= ( bit_mask_t ) SMTC_FLRP_EXCHANGE_ADAPTIVE_LINK_ACTIVE;
    }
    smtc_flrp_mac_layer->payload_stats.exchange_phase_success_mask = phase;

    if( smtc_flrp_mac_layer->adaptive_link_config.mode == SMTC_FLRP_LINK_ADAPTATION_DISABLED )
    {
        smtc_flrp_mac_layer->state = SMTC_FLRP_MAC_LAYER_STATE_BURST;
        status                     = enter_state_burst_rx( smtc_flrp_mac_layer, start_time );
    }
    else
    {
        smtc_flrp_mac_layer->state = SMTC_FLRP_MAC_LAYER_STATE_FLRC_REQ;
        status                     = enter_state_flrc_req_rx( smtc_flrp_mac_layer, start_time );
    }
    return status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION ---------------------------------------------
 */

static void smtc_flrp_mac_reset_flrc_stats( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    for( uint16_t i = 0; i < SMTC_FLRP_NB_CHANNELS_MAX; i++ )
    {
        smtc_flrp_mac_layer->adaptive_link_config.flrc_req_rssi[i] = MAC_INVALID_RSSI;
    }
}

static void add_bit_to_rx_mask( smtc_flrp_mac_layer_t* mac, smtc_flrp_exchange_phase_success_bit_t bit )
{
    if( mac->trx_type == SMTC_FLRP_MAC_RX )
    {
        mac->payload_stats.exchange_phase_success_mask |= ( bit_mask_t ) bit;
    }
}

static void remove_bit_from_rx_mask( smtc_flrp_mac_layer_t* mac, smtc_flrp_exchange_phase_success_bit_t bit )
{
    if( mac->trx_type == SMTC_FLRP_MAC_RX )
    {
        mac->payload_stats.exchange_phase_success_mask &= ~( ( bit_mask_t ) bit );
    }
}

static smtc_flrp_mac_layer_status_t convert_rac_status_to_mac_status( smtc_rac_return_code_t rac_status )
{
    switch( rac_status )
    {
    case SMTC_RAC_SUCCESS:
        return SMTC_FLRP_MAC_LAYER_STATUS_OK;

    case SMTC_RAC_BUSY:
        return SMTC_FLRP_MAC_LAYER_STATUS_BUSY;
    case SMTC_RAC_TIMEOUT:
        return SMTC_FLRP_MAC_LAYER_STATUS_TIMEOUT;
    case SMTC_RAC_INVALID_PARAMETER:
        return SMTC_FLRP_MAC_LAYER_STATUS_INVALID_PARAMS;
    case SMTC_RAC_ERROR:
    case SMTC_RAC_NOT_INITIALIZED:
    case SMTC_RAC_NOT_IMPLEMENTED:
    case SMTC_RAC_NOT_SUPPORTED:
    default:
        return SMTC_FLRP_MAC_LAYER_STATUS_ERROR;
    }
}

static uint32_t get_this_burst_duration_ms( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    uint32_t interframes_duration_ms = ( ( get_nb_packets_in_burst( smtc_flrp_mac_layer ) - 1 ) *
                                         smtc_flrp_mac_layer->burst_info.interframe_delay_us ) /
                                       1000;

    return smtc_flrp_mac_layer->transaction->scheduler_config.duration_time_ms + interframes_duration_ms;
}

static uint32_t get_burst_duration_ms( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t nb_packets_in_burst,
                                       ral_flrc_raw_bit_rate_t raw_bit_rate )
{
    uint32_t interframes_duration_us =
        ( ( nb_packets_in_burst - 1 ) * smtc_flrp_mac_layer->burst_info.interframe_delay_us );

    return ( ( get_packet_duration_us(
                   smtc_flrp_mac_layer,
                   ( smtc_flrp_mac_layer->burst_info.packet_uniformed_size + SMTC_FLRP_MAC_DATA_HEADER_FOOTER_LENGTH ),
                   raw_bit_rate ) *
               nb_packets_in_burst ) +
             interframes_duration_us ) /
           1000;
}

static uint32_t get_packet_duration_us( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint16_t size_payload,
                                        ral_flrc_raw_bit_rate_t raw_bit_rate )
{
    ral_flrc_mod_params_t mod_params = {
        .cr           = smtc_flrp_mac_layer->mac_radio_config.cr,
        .raw_bit_rate = raw_bit_rate,
        .pulse_shape  = smtc_flrp_mac_layer->mac_radio_config.pulse_shape,
    };
    ral_flrc_pkt_params_t pkt_params = {
        .preamble_len     = smtc_flrp_mac_layer->mac_radio_config.preambule_len,
        .sync_word_len    = smtc_flrp_mac_layer->mac_radio_config.sync_word_len,
        .tx_syncword      = smtc_flrp_mac_layer->mac_radio_config.tx_syncword_index,
        .match_sync_word  = smtc_flrp_mac_layer->mac_radio_config.rx_match_sync_word,
        .pld_is_fix       = smtc_flrp_mac_layer->mac_radio_config.pld_is_fix,
        .crc_type         = smtc_flrp_mac_layer->mac_radio_config.crc_type,
        .pld_len_in_bytes = size_payload,

    };

    return ral_get_flrc_time_on_air_in_us( RAL_RADIO_POINTER, &pkt_params, &mod_params );
}

static uint8_t get_nb_packets_in_burst( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    return MIN( smtc_flrp_mac_layer->number_of_packets -
                    ( smtc_flrp_mac_layer->burst_number * smtc_flrp_mac_layer->burst_info.nb_packets_burst_max ),
                smtc_flrp_mac_layer->burst_info.nb_packets_burst_max );
}

static uint16_t get_nb_bursts( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    return ( smtc_flrp_mac_layer->number_of_packets ) / ( smtc_flrp_mac_layer->burst_info.nb_packets_burst_max ) +
           ( ( ( smtc_flrp_mac_layer->number_of_packets ) % ( smtc_flrp_mac_layer->burst_info.nb_packets_burst_max ) !=
               0 )
                 ? 1
                 : 0 );
}

static uint16_t get_packet_idx_in_payload( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t packet_number_in_burst )
{
    return ( smtc_flrp_mac_layer->burst_number * smtc_flrp_mac_layer->burst_info.nb_packets_burst_max ) +
           packet_number_in_burst;
}

static bool is_the_last_packet_in_payload( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t packet_number_in_burst )
{
    return ( get_packet_idx_in_payload( smtc_flrp_mac_layer, packet_number_in_burst ) ==
             ( smtc_flrp_mac_layer->number_of_packets - 1 ) );
}

static uint16_t get_packet_payload_size( ral_flrc_cr_t cr )
{
    uint16_t packet_size = SMTC_FLRP_MAC_DATA_PACKET_LENGTH_MAX;
    if( cr == RAL_FLRC_CR_1_2 )
    {
        packet_size = 300;
    }
    return packet_size - SMTC_FLRP_MAC_DATA_HEADER_FOOTER_LENGTH;
}

static uint32_t get_burst_timeout( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    uint32_t burst_transmission_timeout = get_this_burst_duration_ms( smtc_flrp_mac_layer ) * MAC_BURST_TIMEOUT_NB_TOA;
    if( burst_transmission_timeout < MAC_BURST_TIMEOUT_MS_MIN )
    {
        burst_transmission_timeout = MAC_BURST_TIMEOUT_MS_MIN;
    }
    return burst_transmission_timeout;
}

static bool is_burst_transmission_possible_before_timeout( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                           uint32_t               radio_end_timestamp_ms )
{
    uint32_t next_burst_duration_ms = get_burst_duration_ms(
        smtc_flrp_mac_layer, missed_packets_bitfield_get_nb_missed_packets( smtc_flrp_mac_layer ),
        smtc_flrp_mac_layer->burst_info.raw_bit_rate );
    uint32_t next_ack_duration_us =
        get_packet_duration_us( smtc_flrp_mac_layer, SMTC_FLRP_MAC_BURST_ACK_PACKET_MAX_LENGTH,
                                get_raw_bit_rate_for_burst_ack( smtc_flrp_mac_layer ) );
    uint32_t next_ack_duration_ms = CEIL_DIVISION( next_ack_duration_us, 1000 );
    uint32_t timestamp_end_burst_transmission =
        radio_end_timestamp_ms + smtc_flrp_mac_layer->next_burst_start_delay_ms + SMTC_FLRP_TX_DELAY_MARGIN_MS +
        next_burst_duration_ms + smtc_flrp_mac_layer->burst_info.burst_ack_start_delay_ms +
        SMTC_FLRP_TX_DELAY_MARGIN_MS + next_ack_duration_ms;

    return ( timestamp_end_burst_transmission - smtc_flrp_mac_layer->timestamp_start_burst_ms ) <
           smtc_flrp_mac_layer->burst_transmission_timeout;
}

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_set_common_config( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                     const smtc_flrp_mac_config_t* mac_config )
{
    smtc_flrp_mac_layer_status_t status = SMTC_FLRP_MAC_LAYER_STATUS_OK;

    smtc_flrp_mac_layer->mac_radio_config = mac_config->radio_config;

    smtc_flrp_mac_layer->adaptive_link_config.mode             = mac_config->link_adaptation_mode;
    smtc_flrp_mac_layer->adaptive_link_config.flrc_ack_enabled = mac_config->burst_ack_enabled;
    smtc_flrp_mac_layer->burst_ack_enabled                     = mac_config->burst_ack_enabled;

    smtc_flrp_mac_layer->frequency_offset_hz = mac_config->freq_offset_hz;

    // Configure burst info structure, will be updated during the flrc config exchanges if exists
    smtc_flrp_mac_layer->burst_info.raw_bit_rate             = mac_config->radio_config.raw_bit_rate;
    smtc_flrp_mac_layer->burst_info.coding_rate              = mac_config->radio_config.cr;
    smtc_flrp_mac_layer->burst_info.interframe_delay_us      = mac_config->radio_config.interframe_delay_us;
    smtc_flrp_mac_layer->burst_info.burst_ack_start_delay_ms = mac_config->radio_config.start_ack_delay_us / 1000;

    status = smtc_flrp_mac_layer_compute_packet_payload_size( smtc_flrp_mac_layer,
                                                              smtc_flrp_mac_layer->burst_info.payload_length );

    memcpy( smtc_flrp_mac_layer->destination_dev_eui, mac_config->dest_dev_eui, SMTC_FLRP_EUI_LENGTH );

    if( !smtc_flrp_mac_layer->adaptive_link_config.flrc_ack_enabled &&
        !( ( smtc_flrp_mac_layer->adaptive_link_config.mode == SMTC_FLRP_LINK_ADAPTATION_DISABLED ) ||
           ( smtc_flrp_mac_layer->adaptive_link_config.mode == SMTC_FLRP_LINK_ADAPTATION_ENABLED_WITHOUT_SELECTION ) ) )
    {
        // SMTC_MODEM_HAL_TRACE_WARNING(
        //     "FLRC MAC Layer: Adaptive link enabled with selection but ack disabled. So set adaptive link without "
        //     "selection \n" );
        smtc_flrp_mac_layer->adaptive_link_config.mode = SMTC_FLRP_LINK_ADAPTATION_ENABLED_WITHOUT_SELECTION;
    }

    if( smtc_flrp_mac_layer->adaptive_link_config.mode != SMTC_FLRP_LINK_ADAPTATION_DISABLED )
    {
        smtc_flrp_mac_layer->adaptive_link_config.retry_enabled = true;
        smtc_flrp_mac_layer->adaptive_link_config.channel_interframe_delay_ms =
            ( mac_config->radio_config.adaptive_link_interframe_delay_us / 1000 );

        uint16_t channel_mask;
        bool     channel_selection_active =
            ( smtc_flrp_mac_layer->adaptive_link_config.mode == SMTC_FLRP_LINK_ADAPTATION_FULLY_ENABLED ) ||
            ( smtc_flrp_mac_layer->adaptive_link_config.mode == SMTC_FLRP_LINK_ADAPTATION_CHANNEL_SELECTION_ONLY );

        if( channel_selection_active )
        {
            channel_mask = mac_config->enabled_channels;
        }
        else
        {
            channel_mask = ( 1 << mac_config->radio_config.default_channel );
            // Set default frequency
            smtc_flrp_mac_layer->burst_info.frequency_hz =
                smtc_flrp_mac_channel_get_frequency( smtc_flrp_mac_layer, mac_config->radio_config.default_channel );
        }

        smtc_flrp_mac_prepare_for_flrc_req( smtc_flrp_mac_layer, channel_mask );
    }
    else
    {
        // Set default values
        smtc_flrp_mac_layer->burst_info.frequency_hz =
            smtc_flrp_mac_channel_get_frequency( smtc_flrp_mac_layer, mac_config->radio_config.default_channel );
    }

    return status;
}

static ral_flrc_raw_bit_rate_t divide_raw_bit_rate( ral_flrc_raw_bit_rate_t raw_bit_rate, uint8_t divider )
{
    ral_flrc_raw_bit_rate_t ret = RAL_FLRC_RAW_BIT_RATE_0_260_MBPS;

    if( raw_bit_rate > divider )
    {
        ret = raw_bit_rate - divider;
    }

    return ret;
}

static ral_flrc_raw_bit_rate_t get_raw_bit_rate_for_burst_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    return divide_raw_bit_rate( smtc_flrp_mac_layer->mac_radio_config.raw_bit_rate, MAC_BURST_ACK_DATA_RATE_DIVIDE );
}

static void missed_packets_bitfield_set_rx_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t packet_number )
{
    uint8_t packet_number_index = packet_number / 8;
    smtc_flrp_mac_layer->burst_missed_packets_bitfield[packet_number_index] &=
        ~( ( 1 << ( packet_number % 8 ) ) & 0xFF );
}

static void missed_packets_bitfield_reset( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t number_of_packets )
{
    // Set all expected packets to missed
    memset( smtc_flrp_mac_layer->burst_missed_packets_bitfield, 0, SMTC_FLRP_BITFIELD_PACKETS_IN_BURST_LENGTH );
    for( uint8_t i = 0; i < number_of_packets; i++ )
    {
        uint8_t packet_number_index = i / 8;
        smtc_flrp_mac_layer->burst_missed_packets_bitfield[packet_number_index] |= ( 1 << ( i % 8 ) );
    }
}

static uint8_t missed_packets_bitfield_get_nb_missed_packets( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    uint8_t number_of_packets_missed = 0;
    for( uint8_t i = 0; i < get_nb_packets_in_burst( smtc_flrp_mac_layer ); i++ )
    {
        uint8_t packet_number_index = i / 8;
        if( ( smtc_flrp_mac_layer->burst_missed_packets_bitfield[packet_number_index] >> ( i % 8 ) ) & 0x1 )
        {
            number_of_packets_missed++;
        }
    }
    return number_of_packets_missed;
}

static smtc_flrp_mac_layer_status_t missed_packets_bitfield_get_next_missed_packet(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t* missed_packet_idx )
{
    uint8_t nb_packets_in_burst = get_nb_packets_in_burst( smtc_flrp_mac_layer );

    if( smtc_flrp_mac_layer->packet_number < nb_packets_in_burst )
    {
        for( uint8_t i = smtc_flrp_mac_layer->packet_number; i < nb_packets_in_burst; i++ )
        {
            uint8_t packet_number_index = i / 8;
            if( ( smtc_flrp_mac_layer->burst_missed_packets_bitfield[packet_number_index] >> ( i % 8 ) ) & 0x1 )
            {
                *missed_packet_idx = i;
                return SMTC_FLRP_MAC_LAYER_STATUS_OK;
            }
        }
    }
    return SMTC_FLRP_MAC_LAYER_STATUS_NOT_FOUND;
}

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_construct_data_packet(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t packet_counter, uint8_t* payload, uint32_t payload_length )
{
    if( packet_counter >= get_nb_packets_in_burst( smtc_flrp_mac_layer ) )
    {
        smtc_flrp_mac_layer->tx_payload_buffer_size[smtc_flrp_mac_layer->tx_payload_toggle] = 0;
    }
    else
    {
        uint32_t payload_index = get_packet_idx_in_payload( smtc_flrp_mac_layer, packet_counter ) *
                                 smtc_flrp_mac_layer->burst_info.packet_uniformed_size;
        uint32_t copy_size =
            MIN( smtc_flrp_mac_layer->burst_info.packet_uniformed_size, payload_length - payload_index );

        smtc_flrp_mac_data_packet_t data_packet = {
            .is_last_burst        = true,
            .packet_seq           = packet_counter,
            .burst_seq            = smtc_flrp_mac_layer->burst_number,
            .payload_size         = copy_size,
            .payload_padding_size = smtc_flrp_mac_layer->burst_info.packet_uniformed_size - copy_size,
#ifdef TEST_LONG_PAYLOAD
            .payload = payload,
#else
            .payload = payload + payload_index,
#endif
            .mic = 0,
        };
        memcpy( data_packet.receiver_dev_eui, smtc_flrp_mac_layer->destination_dev_eui, SMTC_FLRP_EUI_LENGTH );
        memcpy( data_packet.initiator_dev_eui, smtc_flrp_mac_layer->dev_eui, SMTC_FLRP_EUI_LENGTH );

        if( smtc_flrp_mac_layer->crypto_enabled )
        {
            uint32_t devaddr = smtc_flrp_mac_layer->dev_eui[SMTC_FLRP_EUI_LENGTH - 1] |
                               ( uint32_t ) smtc_flrp_mac_layer->dev_eui[SMTC_FLRP_EUI_LENGTH - 2] << 8 |
                               ( uint32_t ) smtc_flrp_mac_layer->dev_eui[SMTC_FLRP_EUI_LENGTH - 3] << 16 |
                               ( uint32_t ) smtc_flrp_mac_layer->dev_eui[SMTC_FLRP_EUI_LENGTH - 4] << 24;
            smtc_flrp_crypto_return_code_t rc =
                smtc_flrp_crypto_compute_mic( data_packet.payload, data_packet.payload_size, SMTC_SE_APP_KEY, devaddr,
                                              1, packet_counter, &data_packet.mic, 0 );
            if( rc != SMTC_FLRP_CRYPTO_RC_SUCCESS )
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "FLRC Rx MAC Layer: Failed to set payload to compute mic (error %u)\n",
                                              rc );
            }
        }

        if( smtc_flrp_mac_serialize_data_packet(
                &smtc_flrp_mac_layer->tx_payload_buffer[smtc_flrp_mac_layer->tx_payload_toggle][0],
                &smtc_flrp_mac_layer->tx_payload_buffer_size[smtc_flrp_mac_layer->tx_payload_toggle],
                data_packet ) != SMTC_FLRP_MAC_SERDE_STATUS_OK )
        {
            return SMTC_FLRP_MAC_LAYER_STATUS_ERROR;
        }
    }

    smtc_flrp_mac_layer->tx_payload_toggle = !smtc_flrp_mac_layer->tx_payload_toggle;

    return SMTC_FLRP_MAC_LAYER_STATUS_OK;
}

static void smtc_flrp_mac_layer_construct_missing_data_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                               uint8_t* payload, uint32_t payload_length )
{
    if( missed_packets_bitfield_get_next_missed_packet( smtc_flrp_mac_layer, &smtc_flrp_mac_layer->packet_number ) ==
        SMTC_FLRP_MAC_LAYER_STATUS_OK )
    {
        smtc_flrp_mac_layer_construct_data_packet( smtc_flrp_mac_layer, smtc_flrp_mac_layer->packet_number,
                                                   smtc_flrp_mac_layer->payload,
                                                   smtc_flrp_mac_layer->burst_info.payload_length );
    }
    else
    {
        //  No more missed packets
        smtc_flrp_mac_layer->tx_payload_buffer_size[smtc_flrp_mac_layer->tx_payload_toggle] = 0;
    }
}

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_compute_packet_payload_size(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint32_t buffer_size )
{
    uint16_t payload_size = get_packet_payload_size( smtc_flrp_mac_layer->mac_radio_config.cr );
    smtc_flrp_mac_layer->number_of_packets =
        ( buffer_size ) / ( payload_size ) + ( ( ( buffer_size ) % ( payload_size ) != 0 ) ? 1 : 0 );

    smtc_flrp_mac_layer->burst_info.packet_uniformed_size = payload_size;

    // Compute optimized size to minimize padding
    if( smtc_flrp_mac_layer->number_of_packets > 1 )
    {
        uint32_t nb_packets_for_packet_size = buffer_size / smtc_flrp_mac_layer->burst_info.packet_uniformed_size;
        while( nb_packets_for_packet_size < smtc_flrp_mac_layer->number_of_packets )
        {
            smtc_flrp_mac_layer->burst_info.packet_uniformed_size--;
            nb_packets_for_packet_size = buffer_size / smtc_flrp_mac_layer->burst_info.packet_uniformed_size;
        }
        smtc_flrp_mac_layer->burst_info.packet_uniformed_size++;
    }

    // Info sent in FLRC ACK for the initiator
    compute_burst_info_in_flrc_ack( smtc_flrp_mac_layer );

    if( get_nb_bursts( smtc_flrp_mac_layer ) > SMTC_FLRP_NB_BURSTS_MAX )
    {
        SMTC_MODEM_HAL_TRACE_ERROR(
            "FLRC MAC Layer: Size payload too big to fit in the max of bursts with this radio configuration" );
        return SMTC_FLRP_MAC_LAYER_STATUS_INVALID_PARAMS;
    }

    return SMTC_FLRP_MAC_LAYER_STATUS_OK;
}

static void reset_tx_burst_param( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    smtc_flrp_mac_layer->tx_payload_toggle         = 0;
    smtc_flrp_mac_layer->packet_number             = 0;
    smtc_flrp_mac_layer->next_burst_start_delay_ms = smtc_flrp_mac_layer->mac_radio_config.start_burst_delay_us / 1000;
}

static void reset_rx_burst_param( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    smtc_flrp_mac_layer->packet_number             = 0;
    smtc_flrp_mac_layer->next_burst_start_delay_ms = smtc_flrp_mac_layer->mac_radio_config.start_burst_delay_us / 1000;
    memset( &smtc_flrp_mac_layer->burst_stats, 0, sizeof( smtc_flrp_mac_burst_stats_t ) );
    remove_bit_from_rx_mask( smtc_flrp_mac_layer, SMTC_FLRP_EXCHANGE_LAST_BURST_SUCCESS );
    remove_bit_from_rx_mask( smtc_flrp_mac_layer, SMTC_FLRP_EXCHANGE_LAST_BURST_ACK_SUCCESS );
}

static void stats_increment_packet_received( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, bool packet_check_error,
                                             bool packet_invalid )
{
    if( !packet_check_error && !packet_invalid )
    {
        int64_t rssi_mean_payload = ( int64_t ) smtc_flrp_mac_layer->payload_stats.rssi_mean *
                                    smtc_flrp_mac_layer->payload_stats.nb_packets_received_ok;
        smtc_flrp_mac_layer->burst_stats.rssi_mean *= smtc_flrp_mac_layer->burst_stats.nb_packets_received_ok;

        smtc_flrp_mac_layer->burst_stats.rssi_mean +=
            smtc_flrp_mac_layer->transaction->smtc_rac_data_result.rssi_result;
        rssi_mean_payload += ( int64_t ) smtc_flrp_mac_layer->transaction->smtc_rac_data_result.rssi_result;

        smtc_flrp_mac_layer->burst_stats.nb_packets_received_ok++;
        smtc_flrp_mac_layer->payload_stats.nb_packets_received_ok++;

        smtc_flrp_mac_layer->burst_stats.rssi_mean /= smtc_flrp_mac_layer->burst_stats.nb_packets_received_ok;
        smtc_flrp_mac_layer->payload_stats.rssi_mean =
            ( int32_t ) ( rssi_mean_payload / smtc_flrp_mac_layer->payload_stats.nb_packets_received_ok );
    }
    else if( packet_invalid )
    {
        smtc_flrp_mac_layer->burst_stats.nb_packets_received_nok++;
        smtc_flrp_mac_layer->payload_stats.nb_packets_received_nok++;
    }
    else if( packet_check_error )
    {
        smtc_flrp_mac_layer->burst_stats.nb_packets_check_error++;
        smtc_flrp_mac_layer->payload_stats.nb_packets_check_error++;
    }
}

static void smtc_flrp_mac_set_channels_to_test_in_adaptive_link_phase( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                       uint16_t               enabled_channels_mask,
                                                                       uint8_t*               nb_freq_to_test,
                                                                       uint32_t*              frequencies_to_test_hz )
{
    uint8_t nb_freq = 0;
    for( uint8_t i = 0; i < smtc_flrp_mac_layer->mac_radio_config.nb_frequencies; i++ )
    {
        if( ( enabled_channels_mask >> i ) & 0x1 )
        {
            frequencies_to_test_hz[i] = smtc_flrp_mac_channel_get_frequency( smtc_flrp_mac_layer, i );
            nb_freq++;
        }
    }

    // Enable all channels if enabled_channels_mask is invalid
    if( nb_freq == 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "No channel set for adaptive link configuration -> configure all channels\n" );
        for( uint8_t i = 0; i < smtc_flrp_mac_layer->mac_radio_config.nb_frequencies; i++ )
        {
            frequencies_to_test_hz[i] = smtc_flrp_mac_channel_get_frequency( smtc_flrp_mac_layer, i );
            nb_freq++;
        }
    }
    *nb_freq_to_test = nb_freq;
}

static uint32_t smtc_flrp_mac_channel_get_frequency( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t channel_idx )
{
    if( channel_idx > smtc_flrp_mac_layer->mac_radio_config.nb_frequencies )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Mac channel requested %u is invalid \n", channel_idx );
        return smtc_flrp_mac_layer->mac_radio_config.frequencies_hz[0];
    }
    return smtc_flrp_mac_layer->mac_radio_config.frequencies_hz[channel_idx];
}

static void compute_burst_info_in_flrc_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    smtc_flrp_mac_layer->burst_info.nb_packets_burst_max =
        MIN( smtc_flrp_mac_layer->number_of_packets, SMTC_FLRP_PACKETS_IN_BURST_MAX );

    uint32_t burst_duration =
        get_burst_duration_ms( smtc_flrp_mac_layer, smtc_flrp_mac_layer->burst_info.nb_packets_burst_max,
                               smtc_flrp_mac_layer->mac_radio_config.raw_bit_rate );
    if( burst_duration > MAC_BURST_MAX_TIME_MS )
    {
        smtc_flrp_mac_layer->burst_info.nb_packets_burst_max =
            ( smtc_flrp_mac_layer->burst_info.nb_packets_burst_max * MAC_BURST_MAX_TIME_MS ) / burst_duration;
    }

    smtc_flrp_mac_layer->burst_info.multi_burst_mode =
        ( smtc_flrp_mac_layer->number_of_packets > smtc_flrp_mac_layer->burst_info.nb_packets_burst_max );
}

static void smtc_flrp_mac_prepare_for_flrc_req( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                uint16_t               enable_channels_mask )
{
    // Reset parameters
    smtc_flrp_mac_layer->adaptive_link_config.frequency_test_idx = 0;

    smtc_flrp_mac_reset_flrc_stats( smtc_flrp_mac_layer );
    smtc_flrp_mac_set_channels_to_test_in_adaptive_link_phase(
        smtc_flrp_mac_layer, enable_channels_mask,
        &( smtc_flrp_mac_layer->adaptive_link_config.frequencies_to_test_size ),
        smtc_flrp_mac_layer->adaptive_link_config.frequencies_to_test_hz );
}

/********** TX transaction functions **********/
/**********************************************/

static void smtc_flrp_mac_layer_prepare_tx_data_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                        uint8_t* data_buffer, uint32_t buffer_size,
                                                        uint8_t nb_packets_in_burst )
{
    smtc_flrp_mac_set_radio_params_flrc_burst_mod(
        smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
        smtc_flrp_mac_layer->burst_info.frequency_hz, smtc_flrp_mac_layer->frequency_offset_hz,
        smtc_flrp_mac_layer->burst_info.raw_bit_rate, smtc_flrp_mac_layer->burst_info.coding_rate,
        smtc_flrp_mac_layer->burst_info.interframe_delay_us, ( smtc_flrp_mac_layer->crypto_enabled == true ) );

    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.is_tx = true;

    smtc_flrp_mac_layer_construct_missing_data_packet( smtc_flrp_mac_layer, data_buffer, buffer_size );
    smtc_flrp_mac_layer->packet_number++;
    smtc_flrp_mac_layer_construct_missing_data_packet( smtc_flrp_mac_layer, data_buffer, buffer_size );

    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.burst_tx_size =
        nb_packets_in_burst *
        ( smtc_flrp_mac_layer->burst_info.packet_uniformed_size + SMTC_FLRP_MAC_DATA_HEADER_FOOTER_LENGTH );

    // Prepare the tx fifo payload buffers
    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.tx_fifo_payload_buffer[0] =
        &smtc_flrp_mac_layer->tx_payload_buffer[0][0];
    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.tx_fifo_payload_buffer[1] =
        &smtc_flrp_mac_layer->tx_payload_buffer[1][0];

    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.tx_fifo_payload_length[0] =
        &smtc_flrp_mac_layer->tx_payload_buffer_size[0];
    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.tx_fifo_payload_length[1] =
        &smtc_flrp_mac_layer->tx_payload_buffer_size[1];

    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.size_of_tx_fifo_payload_buffer[0] =
        sizeof( smtc_flrp_mac_layer->tx_payload_buffer[0] );
    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.size_of_tx_fifo_payload_buffer[1] =
        sizeof( smtc_flrp_mac_layer->tx_payload_buffer[1] );
}

static void smtc_flrp_mac_layer_prepare_tx_burst_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    smtc_flrp_mac_set_radio_params_flrc_mod(
        smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
        smtc_flrp_mac_layer->burst_info.frequency_hz, smtc_flrp_mac_layer->frequency_offset_hz,
        get_raw_bit_rate_for_burst_ack( smtc_flrp_mac_layer ), smtc_flrp_mac_layer->burst_info.coding_rate );
    smtc_flrp_mac_layer->transaction->radio_params.flrc.is_tx = true;

    uint8_t nb_packets_in_burst = get_nb_packets_in_burst( smtc_flrp_mac_layer );
    uint8_t nb_packets_missed_accepted =
        ( nb_packets_in_burst * smtc_flrp_mac_layer->mac_radio_config.burst_target_per / 100 );
    if( ( nb_packets_missed_accepted == 0 ) && ( smtc_flrp_mac_layer->mac_radio_config.burst_target_per != 0 ) )
    {
        nb_packets_missed_accepted++;
    }
    smtc_flrp_mac_burst_ack_packet_t burst_ack = { 0 };
    burst_ack.burst_seq                        = smtc_flrp_mac_layer->burst_number;
    burst_ack.link_adaptation_req              = SMTC_FLRP_ADAPTIVE_LINK_DURING_BURST_ENABLED;
    burst_ack.is_missing_packets =
        ( missed_packets_bitfield_get_nb_missed_packets( smtc_flrp_mac_layer ) > nb_packets_missed_accepted );
    burst_ack.next_window_timing_us = smtc_flrp_mac_layer->mac_radio_config.start_burst_delay_us;
    memcpy( burst_ack.initiator_dev_eui, smtc_flrp_mac_layer->destination_dev_eui, SMTC_FLRP_EUI_LENGTH );
    memcpy( burst_ack.receiver_dev_eui, smtc_flrp_mac_layer->dev_eui, SMTC_FLRP_EUI_LENGTH );

    if( burst_ack.is_missing_packets )
    {
        smtc_flrp_mac_layer->burst_retry_needed = true;
        if( burst_ack.link_adaptation_req )
        {
            burst_ack.recommended_channel = smtc_flrp_mac_get_freq_channel( 
                (const smtc_flrp_mac_radio_config_t*) &(smtc_flrp_mac_layer->mac_radio_config), smtc_flrp_mac_layer->burst_info.frequency_hz );
            if( burst_ack.recommended_channel == MAC_INVALID_CHANNEL )
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "FLRC Rx MAC Layer: Current frequency is invalid\n" );
            }
        }
        burst_ack.missing_packets_data_len = ( nb_packets_in_burst / 8 ) + ( ( nb_packets_in_burst % 8 != 0 ) ? 1 : 0 );
        burst_ack.missing_packets_data     = smtc_flrp_mac_layer->burst_missed_packets_bitfield;
    }
    if( burst_ack.link_adaptation_req )
    {
        burst_ack.recommended_datarate    = smtc_flrp_mac_layer->burst_info.raw_bit_rate;
        burst_ack.recommended_coding_rate = smtc_flrp_mac_layer->burst_info.coding_rate;
    }

    smtc_flrp_mac_layer->next_burst_start_delay_ms = burst_ack.next_window_timing_us / 1000;

    uint16_t packet_size;
    if( smtc_flrp_mac_serialize_burst_ack_packet( smtc_flrp_mac_layer->tx_payload_buffer[0], &packet_size,
                                                  burst_ack ) != SMTC_FLRP_MAC_SERDE_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FLRC Rx MAC Layer: Failed to serialize Burst ACK packet\n" );
    }

    smtc_flrp_mac_layer->transaction->radio_params.flrc.tx_size = packet_size;
    smtc_flrp_mac_layer->transaction->smtc_rac_data_buffer_setup.tx_payload_buffer =
        smtc_flrp_mac_layer->tx_payload_buffer[0];
    smtc_flrp_mac_layer->transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = packet_size;
}

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_send_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                     uint32_t               timestamp_ms )
{
    smtc_flrp_mac_layer->transaction->scheduler_config.callback_pre_radio_transaction  = pre_tx_transaction_callback;
    smtc_flrp_mac_layer->transaction->scheduler_config.callback_post_radio_transaction = post_tx_transaction_callback;
    smtc_flrp_mac_layer->transaction->scheduler_config.scheduling                      = SMTC_RAC_SCHEDULED_TRANSACTION;
    smtc_flrp_mac_layer->transaction->scheduler_config.start_time_ms = timestamp_ms + SMTC_FLRP_TX_DELAY_MARGIN_MS;

    return convert_rac_status_to_mac_status(
        smtc_rac_submit_radio_transaction( smtc_flrp_mac_layer->radio_access_id ) );
}

/********** RX transaction functions **********/
/**********************************************/

static void smtc_flrp_mac_layer_prepare_rx_data_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                        uint8_t* data_buffer, uint32_t buffer_size,
                                                        uint32_t packet_size, uint8_t nb_packets_in_burst )
{
    smtc_flrp_mac_set_radio_params_flrc_burst_mod(
        smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
        smtc_flrp_mac_layer->burst_info.frequency_hz, smtc_flrp_mac_layer->frequency_offset_hz,
        smtc_flrp_mac_layer->burst_info.raw_bit_rate, smtc_flrp_mac_layer->burst_info.coding_rate,
        smtc_flrp_mac_layer->burst_info.interframe_delay_us, ( smtc_flrp_mac_layer->crypto_enabled == true ) );

    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.is_tx = false;
    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.max_rx_size =
        ( packet_size + SMTC_FLRP_MAC_DATA_HEADER_FOOTER_LENGTH );

    smtc_flrp_mac_layer->transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = data_buffer;
    smtc_flrp_mac_layer->transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = buffer_size;

    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.burst_rx_size =
        nb_packets_in_burst * ( packet_size + SMTC_FLRP_MAC_DATA_HEADER_FOOTER_LENGTH );

    uint32_t interframes_incertitude_duration_ms = ( ( nb_packets_in_burst - 1 ) *
                                                     ( smtc_flrp_mac_layer->burst_info.interframe_delay_us *
                                                       MAC_BURST_INTERFRAME_DURATION_INCERTITUDE_PERCENTAGE ) /
                                                     100 ) /
                                                   1000;
    if( interframes_incertitude_duration_ms == 0 )
    {
        interframes_incertitude_duration_ms++;
    }
    smtc_flrp_mac_layer->transaction->radio_params.flrc_burst.rx_burst_timeout_margin_ms =
        interframes_incertitude_duration_ms + SMTC_FLRP_TX_DELAY_MARGIN_MS;
}

static void smtc_flrp_mac_layer_prepare_rx_burst_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t* data_buffer,
                                                      uint32_t radio_end_timestamp_ms )
{
    smtc_flrp_mac_set_radio_params_flrc_mod(
        smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
        smtc_flrp_mac_layer->burst_info.frequency_hz, smtc_flrp_mac_layer->frequency_offset_hz,
        get_raw_bit_rate_for_burst_ack( smtc_flrp_mac_layer ), smtc_flrp_mac_layer->burst_info.coding_rate );

    smtc_flrp_mac_layer->transaction->radio_params.flrc.is_tx       = false;
    smtc_flrp_mac_layer->transaction->radio_params.flrc.max_rx_size = SMTC_FLRP_MAC_BURST_ACK_PACKET_MAX_LENGTH;
    smtc_flrp_mac_layer->transaction->radio_params.flrc.rx_timeout_ms =
        smtc_flrp_mac_layer->burst_transmission_timeout -
        ( radio_end_timestamp_ms - smtc_flrp_mac_layer->timestamp_start_burst_ms );

    smtc_flrp_mac_layer->transaction->smtc_rac_data_buffer_setup.rx_payload_buffer = data_buffer;
    smtc_flrp_mac_layer->transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer =
        SMTC_FLRP_MAC_BURST_ACK_PACKET_MAX_LENGTH;
}

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_listen_rx_packet( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                          uint32_t timestamp_ms, bool transaction_asap )
{
    smtc_flrp_mac_layer->transaction->scheduler_config.callback_pre_radio_transaction  = pre_rx_transaction_callback;
    smtc_flrp_mac_layer->transaction->scheduler_config.callback_post_radio_transaction = post_rx_transaction_callback;
    smtc_flrp_mac_layer->transaction->scheduler_config.scheduling =
        transaction_asap ? SMTC_RAC_ASAP_TRANSACTION : SMTC_RAC_SCHEDULED_TRANSACTION;
    smtc_flrp_mac_layer->transaction->scheduler_config.start_time_ms = timestamp_ms;

    return convert_rac_status_to_mac_status(
        smtc_rac_submit_radio_transaction( smtc_flrp_mac_layer->radio_access_id ) );
}

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_listen_rx_packet_asap(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint32_t timestamp_ms )
{
    return smtc_flrp_mac_layer_listen_rx_packet( smtc_flrp_mac_layer, timestamp_ms, true );
}

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_layer_listen_rx_packet_scheduled(
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint32_t timestamp_ms )
{
    return smtc_flrp_mac_layer_listen_rx_packet( smtc_flrp_mac_layer, timestamp_ms, false );
}

/********** Functions handling rx frame *******/
/**********************************************/

static bool smtc_flrp_mac_handle_burst_data( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, uint8_t* rx_payload,
                                             uint16_t rx_payload_size )
{
    static smtc_flrp_mac_data_packet_t data_packet;
    bool                               mic_check_ok = true;

    if( rx_payload_size !=
        smtc_flrp_mac_layer->burst_info.packet_uniformed_size + SMTC_FLRP_MAC_DATA_HEADER_FOOTER_LENGTH )
    {
        // Packet has invalid size
        return false;
    }

    if( smtc_flrp_mac_deserialize_data_packet_header( rx_payload, rx_payload_size, &data_packet ) !=
        SMTC_FLRP_MAC_SERDE_STATUS_OK )
    {
        //  Wrong packet received
        return false;
    }

    if( memcmp( smtc_flrp_mac_layer->destination_dev_eui, data_packet.initiator_dev_eui, SMTC_FLRP_EUI_LENGTH ) ||
        !smtc_flrp_is_frame_addressed_to_this_device( smtc_flrp_mac_layer->dev_eui, data_packet.receiver_dev_eui,
                                                      smtc_flrp_mac_layer->rx_frame_dev_eui_filter_len ) )
    {
        //  Packet is for/from an other device
        return false;
    }

    bool last_packet_all_bursts = is_the_last_packet_in_payload( smtc_flrp_mac_layer, data_packet.packet_seq );
    if( last_packet_all_bursts )
    {
        data_packet.payload_size =
            smtc_flrp_mac_layer->burst_info.payload_length -
            ( ( smtc_flrp_mac_layer->number_of_packets - 1 ) * smtc_flrp_mac_layer->burst_info.packet_uniformed_size );

        SMTC_MODEM_HAL_PANIC_ON_FAILURE( data_packet.payload_size <=
                                         smtc_flrp_mac_layer->burst_info.packet_uniformed_size );

        data_packet.payload_padding_size =
            smtc_flrp_mac_layer->burst_info.packet_uniformed_size - data_packet.payload_size;
    }
    else
    {
        data_packet.payload_size         = smtc_flrp_mac_layer->burst_info.packet_uniformed_size;
        data_packet.payload_padding_size = 0;
    }

    if( smtc_flrp_mac_deserialize_data_packet_footer(
            &rx_payload[SMTC_FLRP_MAC_DATA_HEADER_LENGTH + data_packet.payload_size], rx_payload_size, &data_packet ) !=
        SMTC_FLRP_MAC_SERDE_STATUS_OK )
    {
        //  Wrong packet received
        return false;
    }

    if( smtc_flrp_mac_layer->crypto_enabled )
    {
        uint32_t devaddr = data_packet.initiator_dev_eui[SMTC_FLRP_EUI_LENGTH - 1] |
                           ( uint32_t ) data_packet.initiator_dev_eui[SMTC_FLRP_EUI_LENGTH - 2] << 8 |
                           ( uint32_t ) data_packet.initiator_dev_eui[SMTC_FLRP_EUI_LENGTH - 3] << 16 |
                           ( uint32_t ) data_packet.initiator_dev_eui[SMTC_FLRP_EUI_LENGTH - 4] << 24;
        if( smtc_flrp_crypto_verify_mic( &rx_payload[SMTC_FLRP_MAC_DATA_HEADER_LENGTH], data_packet.payload_size,
                                         SMTC_SE_APP_KEY, devaddr, 1, data_packet.packet_seq, data_packet.mic,
                                         0 ) != SMTC_FLRP_CRYPTO_RC_SUCCESS )
        {
            mic_check_ok = false;
        }
    }

    if( mic_check_ok )
    {
#ifndef TEST_LONG_PAYLOAD
        uint16_t packet_number =
            ( smtc_flrp_mac_layer->burst_number * smtc_flrp_mac_layer->burst_info.nb_packets_burst_max ) +
            data_packet.packet_seq;

        uint32_t rx_data_buffer_index = ( packet_number * data_packet.payload_size );

        if( ( rx_data_buffer_index + data_packet.payload_size ) <= smtc_flrp_mac_layer->rx_data_buffer_size )
#endif
        {
#ifdef TEST_LONG_PAYLOAD
            data_packet.payload = smtc_flrp_mac_layer->rx_data_buffer;
#else
            data_packet.payload = smtc_flrp_mac_layer->rx_data_buffer +
                                  ( packet_number * smtc_flrp_mac_layer->burst_info.packet_uniformed_size );
#endif

            if( smtc_flrp_mac_deserialize_data_packet_payload( rx_payload + SMTC_FLRP_MAC_DATA_HEADER_LENGTH,
                                                               rx_payload_size - SMTC_FLRP_MAC_DATA_HEADER_LENGTH,
                                                               &data_packet ) != SMTC_FLRP_MAC_SERDE_STATUS_OK )
            {
                // packet received too small
                return false;
            }

            missed_packets_bitfield_set_rx_packet( smtc_flrp_mac_layer, data_packet.packet_seq );
            smtc_flrp_mac_layer->packet_number = data_packet.packet_seq;
            smtc_flrp_mac_layer->burst_number  = data_packet.burst_seq;
            smtc_flrp_mac_layer->nb_bytes_received += data_packet.payload_size;
            stats_increment_packet_received( smtc_flrp_mac_layer, false, false );
        }
#ifndef TEST_LONG_PAYLOAD
        else
        {
            // SMTC_MODEM_HAL_TRACE_WARNING( "FLRC Rx MAC Layer: RX buffer is full %u + %u > %u \n",
            // rx_data_buffer_index, rx_payload_size, smtc_flrp_mac_layer->rx_data_buffer_size );
            return false;
        }
#endif
    }
    else
    {
        stats_increment_packet_received( smtc_flrp_mac_layer, true, false );
    }
    return true;
}

static smtc_flrp_mac_layer_status_t smtc_flrp_mac_handle_burst_ack( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    smtc_flrp_mac_layer_status_t     status    = SMTC_FLRP_MAC_LAYER_STATUS_OK;
    smtc_flrp_mac_burst_ack_packet_t burst_ack = { 0 };
    burst_ack.missing_packets_data             = &smtc_flrp_mac_layer->burst_missed_packets_bitfield[0];
    if( smtc_flrp_mac_deserialize_burst_ack_packet( smtc_flrp_mac_layer->rx_payload_buffer,
                                                    smtc_flrp_mac_layer->transaction->smtc_rac_data_result.rx_size,
                                                    &burst_ack ) != SMTC_FLRP_MAC_SERDE_STATUS_OK )
    {
        // SMTC_MODEM_HAL_TRACE_ERROR( "FLRC Rx MAC Layer: Failed to deserialize Burst ACK packet\n" );
        status = SMTC_FLRP_MAC_LAYER_STATUS_ERROR;
    }
    else
    {
        // ACK is for only one device, the receiver dev eui must equals the destination dev eui (no filter len)
        if( memcmp( smtc_flrp_mac_layer->dev_eui, burst_ack.initiator_dev_eui, SMTC_FLRP_EUI_LENGTH ) ||
            memcmp( smtc_flrp_mac_layer->destination_dev_eui, burst_ack.receiver_dev_eui, SMTC_FLRP_EUI_LENGTH ) )
        {
            //  Packet is for/from an other device
            status = SMTC_FLRP_MAC_LAYER_STATUS_ERROR;
        }
        else
        {
            // The initiator can miss an ack and receive the next one (with all packets missing) but not the next ones
            if( ( burst_ack.burst_seq < smtc_flrp_mac_layer->burst_number ) ||
                ( burst_ack.burst_seq > smtc_flrp_mac_layer->burst_number + 1 ) )
            {
                status = SMTC_FLRP_MAC_LAYER_STATUS_ERROR;
            }
            else
            {
                smtc_flrp_mac_layer->burst_number = burst_ack.burst_seq;
                if( burst_ack.link_adaptation_req )
                {
                    smtc_flrp_mac_layer->burst_info.raw_bit_rate = burst_ack.recommended_datarate;
                    smtc_flrp_mac_layer->burst_info.coding_rate  = burst_ack.recommended_coding_rate;
                    smtc_flrp_mac_layer->burst_info.frequency_hz =
                        smtc_flrp_mac_channel_get_frequency( smtc_flrp_mac_layer, burst_ack.recommended_channel );
                }

                smtc_flrp_mac_layer->next_burst_start_delay_ms = burst_ack.next_window_timing_us / 1000;
                smtc_flrp_mac_layer->burst_retry_needed        = burst_ack.is_missing_packets;
            }
        }
    }
    return status;
}

/********** FSM functions **********/
/***********************************/
static void go_to_burst_state_if_remaining_burst( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    if( smtc_flrp_mac_layer->burst_retry_needed )
    {
        go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_BURST );
    }
    else if( ( smtc_flrp_mac_layer->burst_info.multi_burst_mode ) &&
             ( smtc_flrp_mac_layer->burst_number < ( get_nb_bursts( smtc_flrp_mac_layer ) - 1 ) ) )
    {
        // Send next burst
        smtc_flrp_mac_layer->burst_number++;
        go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_BURST );
    }
    else
    {
        add_bit_to_rx_mask( smtc_flrp_mac_layer, SMTC_FLRP_EXCHANGE_LAST_BURST_SUCCESS );
        if( smtc_flrp_mac_layer->burst_ack_enabled )
        {
            add_bit_to_rx_mask( smtc_flrp_mac_layer, SMTC_FLRP_EXCHANGE_LAST_BURST_ACK_SUCCESS );
        }
        go_to_state_idle_and_return_status( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATUS_OK,
                                            ( smtc_flrp_mac_layer->trx_type == SMTC_FLRP_MAC_TX )
                                                ? smtc_flrp_mac_layer->burst_info.payload_length
                                                : smtc_flrp_mac_layer->nb_bytes_received );
    }
}

static void go_to_mac_state( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, smtc_flrp_mac_layer_state_t state )
{
    smtc_flrp_mac_layer->state = state;

    if( state == SMTC_FLRP_MAC_LAYER_STATE_IDLE )
    {
        enter_state_idle( smtc_flrp_mac_layer );
    }
    else
    {
        if( smtc_flrp_mac_layer->trx_type == SMTC_FLRP_MAC_TX )
        {
            tx_fsm_go_to_state( smtc_flrp_mac_layer, state );
        }
        else
        {
            rx_fsm_go_to_state( smtc_flrp_mac_layer, state );
        }
    }
}

static void tx_fsm_go_to_state( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, smtc_flrp_mac_layer_state_t state )
{
    smtc_flrp_mac_layer_status_t mac_status = SMTC_FLRP_MAC_LAYER_STATUS_OK;

    switch( state )
    {
    case SMTC_FLRP_MAC_LAYER_STATE_IDLE:
        enter_state_idle( smtc_flrp_mac_layer );
        break;
    case SMTC_FLRP_MAC_LAYER_STATE_FLRC_REQ:
        mac_status = enter_state_flrc_req_tx(
            smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms +
                                     smtc_flrp_mac_layer->adaptive_link_config.channel_interframe_delay_ms );
        break;
    case SMTC_FLRP_MAC_LAYER_STATE_FLRC_ACK:
        mac_status = enter_state_flrc_ack_rx( smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms );
        break;
    case SMTC_FLRP_MAC_LAYER_STATE_BURST:
        if( smtc_flrp_mac_layer->burst_retry_needed )
        {
            mac_status = enter_state_burst_retry_tx( smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms );
            smtc_flrp_mac_layer->burst_retry_needed = false;
        }
        else
        {
            mac_status =
                enter_state_burst_tx( smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms +
                                                               smtc_flrp_mac_layer->next_burst_start_delay_ms );
        }
        break;
    case SMTC_FLRP_MAC_LAYER_STATE_ACK:
        mac_status = enter_state_burst_ack_rx( smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms );
        break;
    default:
        break;
    }

    if( mac_status != SMTC_FLRP_MAC_LAYER_STATUS_OK )
    {
        go_to_state_idle_and_return_status( smtc_flrp_mac_layer, mac_status, 0 );
    }
}

static void rx_fsm_go_to_state( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer, smtc_flrp_mac_layer_state_t state )
{
    smtc_flrp_mac_layer_status_t mac_status = SMTC_FLRP_MAC_LAYER_STATUS_OK;

    switch( state )
    {
    case SMTC_FLRP_MAC_LAYER_STATE_IDLE:
        enter_state_idle( smtc_flrp_mac_layer );
        break;
    case SMTC_FLRP_MAC_LAYER_STATE_FLRC_REQ:
        mac_status = enter_state_flrc_req_rx(
            smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms +
                                     smtc_flrp_mac_layer->adaptive_link_config.channel_interframe_delay_ms );
        break;
    case SMTC_FLRP_MAC_LAYER_STATE_FLRC_ACK:
        mac_status = enter_state_flrc_ack_tx( smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms );
        break;
    case SMTC_FLRP_MAC_LAYER_STATE_BURST:
        if( smtc_flrp_mac_layer->burst_retry_needed )
        {
            mac_status = enter_state_burst_retry_rx( smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms );
            smtc_flrp_mac_layer->burst_retry_needed = false;
        }
        else
        {
            mac_status =
                enter_state_burst_rx( smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms +
                                                               smtc_flrp_mac_layer->next_burst_start_delay_ms );
        }
        break;
    case SMTC_FLRP_MAC_LAYER_STATE_ACK:
        mac_status = enter_state_burst_ack_tx( smtc_flrp_mac_layer, smtc_flrp_mac_layer->radio_end_timestamp_ms );
        break;
    default:
        break;
    }

    if( mac_status != SMTC_FLRP_MAC_LAYER_STATUS_OK )
    {
        go_to_state_idle_and_return_status( smtc_flrp_mac_layer, mac_status, 0 );
    }
}

static smtc_flrp_mac_layer_status_t enter_state_flrc_req_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                             uint32_t               start_timestamp_ms )
{
    smtc_flrp_mac_prepare_rx_flrc_req(
        smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
        smtc_flrp_mac_layer->rx_payload_buffer,
        smtc_flrp_mac_layer->adaptive_link_config
            .frequencies_to_test_hz[smtc_flrp_mac_layer->adaptive_link_config.frequency_test_idx],
        smtc_flrp_mac_layer->frequency_offset_hz );

    return smtc_flrp_mac_layer_listen_rx_packet_scheduled( smtc_flrp_mac_layer, start_timestamp_ms );
}

static smtc_flrp_mac_layer_status_t enter_state_flrc_req_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                             uint32_t               start_timestamp_ms )
{
    smtc_flrp_mac_adaptive_link_t* adap_link_config = &smtc_flrp_mac_layer->adaptive_link_config;

    // Prepare the packet only once, change only the frequency for next FLRC REQ
    if( adap_link_config->frequency_test_idx == 0 )
    {
        smtc_flrp_mac_prepare_tx_flrc_req(
            smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
            adap_link_config->frequencies_to_test_hz[adap_link_config->frequency_test_idx],
            smtc_flrp_mac_layer->frequency_offset_hz, smtc_flrp_mac_layer->dev_eui,
            smtc_flrp_mac_layer->destination_dev_eui, smtc_flrp_mac_layer->burst_info,
            smtc_flrp_mac_layer->adaptive_link_config, smtc_flrp_mac_layer->tx_payload_buffer[0] );
    }
    else
    {
        smtc_flrp_mac_set_radio_params_flrc_mod(
            smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
            adap_link_config->frequencies_to_test_hz[adap_link_config->frequency_test_idx],
            smtc_flrp_mac_layer->frequency_offset_hz, smtc_flrp_mac_layer->mac_radio_config.raw_bit_rate,
            smtc_flrp_mac_layer->mac_radio_config.cr );
    }
    return smtc_flrp_mac_layer_send_packet( smtc_flrp_mac_layer, start_timestamp_ms );
}

static smtc_flrp_mac_layer_status_t enter_state_flrc_ack_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                             uint32_t               radio_end_timestamp_ms )
{
    smtc_flrp_mac_prepare_rx_flrc_ack(
        smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
        smtc_flrp_mac_layer->rx_payload_buffer,
        smtc_flrp_mac_layer->adaptive_link_config
            .frequencies_to_test_hz[smtc_flrp_mac_layer->adaptive_link_config.frequency_test_idx],
        smtc_flrp_mac_layer->frequency_offset_hz );

    return smtc_flrp_mac_layer_listen_rx_packet_scheduled(
        smtc_flrp_mac_layer,
        radio_end_timestamp_ms + smtc_flrp_mac_layer->adaptive_link_config.channel_interframe_delay_ms );
}

static smtc_flrp_mac_layer_status_t enter_state_flrc_ack_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                             uint32_t               radio_end_timestamp_ms )
{
    smtc_flrp_mac_adaptive_link_t* adap_link_config = &smtc_flrp_mac_layer->adaptive_link_config;

    // Prepare the packet only once, change only the frequency for next FLRC ACK
    if( adap_link_config->frequency_test_idx == 0 )
    {
        compute_burst_info_in_flrc_ack( smtc_flrp_mac_layer );

        smtc_flrp_mac_prepare_tx_flrc_ack(
            smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
            adap_link_config->frequencies_to_test_hz[adap_link_config->frequency_test_idx],
            smtc_flrp_mac_layer->frequency_offset_hz, smtc_flrp_mac_layer->dev_eui,
            smtc_flrp_mac_layer->destination_dev_eui, &smtc_flrp_mac_layer->adaptive_link_config,
            &smtc_flrp_mac_layer->burst_info, smtc_flrp_mac_layer->tx_payload_buffer[0] );
    }
    else
    {
        smtc_flrp_mac_set_radio_params_flrc_mod(
            smtc_flrp_mac_layer->transaction, &smtc_flrp_mac_layer->mac_radio_config,
            adap_link_config->frequencies_to_test_hz[adap_link_config->frequency_test_idx],
            smtc_flrp_mac_layer->frequency_offset_hz, smtc_flrp_mac_layer->mac_radio_config.raw_bit_rate,
            smtc_flrp_mac_layer->mac_radio_config.cr );
    }
    return smtc_flrp_mac_layer_send_packet( smtc_flrp_mac_layer,
                                            radio_end_timestamp_ms + adap_link_config->channel_interframe_delay_ms );
}

static smtc_flrp_mac_layer_status_t enter_state_burst_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                          uint32_t               start_timestamp_ms )
{
    smtc_flrp_mac_layer_status_t mac_status;

    reset_rx_burst_param( smtc_flrp_mac_layer );

    uint8_t number_of_packets_in_burst            = get_nb_packets_in_burst( smtc_flrp_mac_layer );
    smtc_flrp_mac_layer->timestamp_start_burst_ms = start_timestamp_ms;

    missed_packets_bitfield_reset( smtc_flrp_mac_layer, number_of_packets_in_burst );

    smtc_flrp_mac_layer_prepare_rx_data_packet(
        smtc_flrp_mac_layer, smtc_flrp_mac_layer->rx_payload_buffer, sizeof( smtc_flrp_mac_layer->rx_payload_buffer ),
        smtc_flrp_mac_layer->burst_info.packet_uniformed_size, number_of_packets_in_burst );

    mac_status = smtc_flrp_mac_layer_listen_rx_packet_scheduled( smtc_flrp_mac_layer,
                                                                 smtc_flrp_mac_layer->timestamp_start_burst_ms );

    smtc_flrp_mac_layer->burst_transmission_timeout = get_burst_timeout( smtc_flrp_mac_layer );

    // SMTC_MODEM_HAL_TRACE_INFO( " RX burst %u\n", smtc_flrp_mac_layer->burst_number );
    return mac_status;
}

static smtc_flrp_mac_layer_status_t enter_state_burst_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                          uint32_t               start_timestamp_ms )
{
    smtc_flrp_mac_layer_status_t status;
    smtc_flrp_mac_layer->timestamp_start_burst_ms = start_timestamp_ms;
    uint8_t nb_packets_in_burst                   = get_nb_packets_in_burst( smtc_flrp_mac_layer );

    reset_tx_burst_param( smtc_flrp_mac_layer );
    missed_packets_bitfield_reset( smtc_flrp_mac_layer, nb_packets_in_burst );

    smtc_flrp_mac_layer_prepare_tx_data_packet( smtc_flrp_mac_layer, smtc_flrp_mac_layer->payload,
                                                smtc_flrp_mac_layer->burst_info.payload_length, nb_packets_in_burst );
    status = smtc_flrp_mac_layer_send_packet( smtc_flrp_mac_layer, smtc_flrp_mac_layer->timestamp_start_burst_ms );

    smtc_flrp_mac_layer->burst_transmission_timeout = get_burst_timeout( smtc_flrp_mac_layer );

    // SMTC_MODEM_HAL_TRACE_INFO( " TX burst %u\n", smtc_flrp_mac_layer->burst_number );

    return status;
}

static smtc_flrp_mac_layer_status_t enter_state_burst_retry_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                uint32_t               radio_end_timestamp_ms )
{
    if( !is_burst_transmission_possible_before_timeout( smtc_flrp_mac_layer, radio_end_timestamp_ms ) )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "FLRC Rx MAC Layer: Burst retry timeout \n" );
        return SMTC_FLRP_MAC_LAYER_STATUS_TIMEOUT;
    }

    reset_rx_burst_param( smtc_flrp_mac_layer );

    smtc_flrp_mac_layer_prepare_rx_data_packet( smtc_flrp_mac_layer, smtc_flrp_mac_layer->rx_payload_buffer,
                                                smtc_flrp_mac_layer->burst_info.payload_length,
                                                smtc_flrp_mac_layer->burst_info.packet_uniformed_size,
                                                missed_packets_bitfield_get_nb_missed_packets( smtc_flrp_mac_layer ) );

    return smtc_flrp_mac_layer_listen_rx_packet_scheduled(
        smtc_flrp_mac_layer, radio_end_timestamp_ms + smtc_flrp_mac_layer->next_burst_start_delay_ms );
}

static smtc_flrp_mac_layer_status_t enter_state_burst_retry_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                                uint32_t               radio_end_timestamp_ms )
{
    if( !is_burst_transmission_possible_before_timeout( smtc_flrp_mac_layer, radio_end_timestamp_ms ) )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "FLRC Tx MAC Layer: Burst retry timeout \n" );
        return SMTC_FLRP_MAC_LAYER_STATUS_TIMEOUT;
    }
    reset_tx_burst_param( smtc_flrp_mac_layer );

    smtc_flrp_mac_layer_prepare_tx_data_packet( smtc_flrp_mac_layer, smtc_flrp_mac_layer->payload,
                                                smtc_flrp_mac_layer->burst_info.payload_length,
                                                missed_packets_bitfield_get_nb_missed_packets( smtc_flrp_mac_layer ) );

    return smtc_flrp_mac_layer_send_packet( smtc_flrp_mac_layer,
                                            radio_end_timestamp_ms + smtc_flrp_mac_layer->next_burst_start_delay_ms );
}

static smtc_flrp_mac_layer_status_t enter_state_burst_ack_rx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                              uint32_t               radio_end_timestamp_ms )
{
    if( ( radio_end_timestamp_ms - smtc_flrp_mac_layer->timestamp_start_burst_ms ) >
        smtc_flrp_mac_layer->burst_transmission_timeout )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "FLRC Tx MAC Layer: Timeout: No time to receive the ACK\n" );
        return SMTC_FLRP_MAC_LAYER_STATUS_TIMEOUT;
    }

    smtc_flrp_mac_layer_prepare_rx_burst_ack( smtc_flrp_mac_layer, smtc_flrp_mac_layer->rx_payload_buffer,
                                              radio_end_timestamp_ms );
    return smtc_flrp_mac_layer_listen_rx_packet_asap( smtc_flrp_mac_layer, radio_end_timestamp_ms );
}

static smtc_flrp_mac_layer_status_t enter_state_burst_ack_tx( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer,
                                                              uint32_t               radio_end_timestamp_ms )
{
    if( ( radio_end_timestamp_ms - smtc_flrp_mac_layer->timestamp_start_burst_ms ) >
        smtc_flrp_mac_layer->burst_transmission_timeout )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "FLRC Rx MAC Layer: Timeout: No time to send the ACK\n" );
        return SMTC_FLRP_MAC_LAYER_STATUS_TIMEOUT;
    }

    smtc_flrp_mac_layer_prepare_tx_burst_ack( smtc_flrp_mac_layer );
    return smtc_flrp_mac_layer_send_packet(
        smtc_flrp_mac_layer, radio_end_timestamp_ms + smtc_flrp_mac_layer->burst_info.burst_ack_start_delay_ms );
}

static void go_to_state_idle_and_return_status( smtc_flrp_mac_layer_t*       smtc_flrp_mac_layer,
                                                smtc_flrp_mac_layer_status_t mac_status, uint32_t data_size )
{
    go_to_mac_state( smtc_flrp_mac_layer, SMTC_FLRP_MAC_LAYER_STATE_IDLE );

    if( smtc_flrp_mac_layer->trx_type == SMTC_FLRP_MAC_TX )
    {
        if( smtc_flrp_mac_layer->mac_tx_done_cb != NULL )
        {
            smtc_flrp_mac_layer->mac_tx_done_cb( mac_status, smtc_flrp_mac_layer->radio_end_timestamp_ms,
                                                 smtc_flrp_mac_layer->destination_dev_eui );
        }
    }
    else
    {
        if( smtc_flrp_mac_layer->mac_rx_done_cb != NULL )
        {
            smtc_flrp_mac_layer->mac_rx_done_cb( mac_status, data_size, smtc_flrp_mac_layer->radio_end_timestamp_ms,
                                                 smtc_flrp_mac_layer->destination_dev_eui );
        }
    }
}

static void enter_state_idle( smtc_flrp_mac_layer_t* smtc_flrp_mac_layer )
{
    smtc_flrp_mac_layer->burst_transmission_timeout              = 0;
    smtc_flrp_mac_layer->burst_retry_needed                      = false;
    smtc_flrp_mac_layer->adaptive_link_config.frequency_test_idx = 0;
    smtc_flrp_mac_layer->adaptive_link_config.last_flrc_ack_transfer_status =
        SMTC_FLRP_MAC_ADAPTIVE_LINK_RETRY;  // By default retry if 0 flrc ack are received
    smtc_flrp_mac_layer->burst_number      = 0;
    smtc_flrp_mac_layer->nb_bytes_received = 0;

    if( smtc_flrp_mac_layer->trx_type == SMTC_FLRP_MAC_TX )
    {
        reset_tx_burst_param( smtc_flrp_mac_layer );
    }
    else
    {
        reset_rx_burst_param( smtc_flrp_mac_layer );
    }
}

/********** Callback functions **********/
/****************************************/

static void pre_tx_transaction_callback( void )
{
}

static void post_tx_transaction_callback( rp_status_t status )
{
    uint8_t                radio_access_id = smtc_rac_get_callback_radio_id( );
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer =
        ( smtc_flrp_mac_layer_t* ) smtc_rac_get_context_private( radio_access_id );

    if( smtc_flrp_mac_layer == NULL )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "FLRC Tx MAC Layer: Private context for radio access ID %u is NULL!\n",
                                      radio_access_id );
        return;
    }

    smtc_flrp_mac_layer->radio_end_timestamp_ms =
        smtc_flrp_mac_layer->transaction->smtc_rac_data_result.radio_end_timestamp_ms;

    switch( status )
    {
    case RP_STATUS_RADIO_LOCKED:
    {
        break;
    }
    case RP_STATUS_REQUEST_NEXT_TX_PAYLOAD:
    {
        if( smtc_flrp_mac_layer->state == SMTC_FLRP_MAC_LAYER_STATE_BURST )
        {
            smtc_flrp_mac_layer->packet_number++;

            smtc_flrp_mac_layer_construct_missing_data_packet( smtc_flrp_mac_layer, smtc_flrp_mac_layer->payload,
                                                               smtc_flrp_mac_layer->burst_info.payload_length );
        }
        break;
    }
    case RP_STATUS_TASK_ABORTED:
    {
        SMTC_MODEM_HAL_TRACE_INFO( "FLRC Tx MAC Layer: Task aborted\n" );
        smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_ABORTED;
        break;
    }
    case RP_STATUS_TX_DONE:

#ifdef DEBUG_LOG
        if( smtc_flrp_mac_layer->state == SMTC_FLRP_MAC_LAYER_STATE_ACK )
        {
            SMTC_MODEM_HAL_TRACE_INFO(
                "Burst %u -- RX packets OK: %u (rssi mean: %d), RX crc/mic error: %u, RX wrong packets: %u\n",
                smtc_flrp_mac_layer->burst_number, smtc_flrp_mac_layer->burst_stats.nb_packets_received_ok,
                smtc_flrp_mac_layer->burst_stats.rssi_mean, smtc_flrp_mac_layer->burst_stats.nb_packets_check_error,
                smtc_flrp_mac_layer->burst_stats.nb_packets_received_nok );
            if( missed_packets_bitfield_get_nb_missed_packets( smtc_flrp_mac_layer ) > 0 )
            {
                SMTC_MODEM_HAL_TRACE_ARRAY( "Bitfield RX packets (1 = missed)",
                                            smtc_flrp_mac_layer->burst_missed_packets_bitfield,
                                            SMTC_FLRP_BITFIELD_PACKETS_IN_BURST_LENGTH );
            }
        }
#endif
        smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_TRX_OK;
        break;
    case RP_STATUS_RADIO_UNLOCKED:
        if( smtc_flrp_mac_layer->state == SMTC_FLRP_MAC_LAYER_STATE_BURST )
        {
            smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_TRX_OK;
        }
        break;
    default:
        SMTC_MODEM_HAL_TRACE_WARNING( "FLRC Tx MAC Layer: Unknown transaction status: 0x%02x\n", status );
        smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_TRX_FAILED;
        break;
    }
}

static void pre_rx_transaction_callback( void )
{
}

static void post_rx_transaction_callback( rp_status_t status )
{
    uint8_t                radio_access_id = smtc_rac_get_callback_radio_id( );
    smtc_flrp_mac_layer_t* smtc_flrp_mac_layer =
        ( smtc_flrp_mac_layer_t* ) smtc_rac_get_context_private( radio_access_id );

    if( smtc_flrp_mac_layer == NULL )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "FLRC Rx MAC Layer: Private context for radio access ID %u is NULL!\n",
                                      radio_access_id );
        return;
    }

    smtc_flrp_mac_layer->radio_end_timestamp_ms =
        smtc_flrp_mac_layer->transaction->smtc_rac_data_result.radio_end_timestamp_ms;

    switch( status )
    {
    case RP_STATUS_RADIO_LOCKED:
    {
        SMTC_MODEM_HAL_TRACE_INFO( "FLRC Rx MAC Layer: Radio locked\n" );
        break;
    }
    case RP_STATUS_RX_PACKET:
    {
        if( smtc_flrp_mac_layer->transaction->smtc_rac_data_result.rx_size == 0 )
        {
            if( smtc_flrp_mac_layer->state != SMTC_FLRP_MAC_LAYER_STATE_BURST )
            {
                smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_TRX_FAILED;
            }
            return;
        }

        if( smtc_flrp_mac_layer->state == SMTC_FLRP_MAC_LAYER_STATE_BURST )
        {
            bool                   frame_header_is_valid = false;
            smtc_flrp_mac_header_t mac_header;
            smtc_flrp_mac_deserialize_header( smtc_flrp_mac_layer->rx_payload_buffer, &mac_header );
            if( mac_header.message_type == SMTC_FLRP_MAC_FRAME_TYPE_DATA )
            {
                frame_header_is_valid =
                    smtc_flrp_mac_handle_burst_data( smtc_flrp_mac_layer, smtc_flrp_mac_layer->rx_payload_buffer,
                                                     smtc_flrp_mac_layer->transaction->smtc_rac_data_result.rx_size );
            }
            if( frame_header_is_valid )
            {
                uint8_t missed_packet_idx;
                if( missed_packets_bitfield_get_next_missed_packet( smtc_flrp_mac_layer, &missed_packet_idx ) ==
                    SMTC_FLRP_MAC_LAYER_STATUS_NOT_FOUND )  // No more packets expected (all packets are
                                                            // received or the packet just received is the last
                                                            // expected)
                {
                    smtc_rac_flrc_burst_rx_done( smtc_flrp_mac_layer->radio_access_id );
                    // Program the send of the ACK once the radio is stopped (UNLOCKED)
                }
            }
            else
            {
                stats_increment_packet_received( smtc_flrp_mac_layer, false, true );
            }
        }
        else if( smtc_flrp_mac_layer->state == SMTC_FLRP_MAC_LAYER_STATE_ACK )
        {
            bool                   frame_ok   = false;
            smtc_flrp_mac_header_t mac_header = { 0 };
            smtc_flrp_mac_deserialize_header( smtc_flrp_mac_layer->rx_payload_buffer, &mac_header );
            if( mac_header.message_type == SMTC_FLRP_MAC_FRAME_TYPE_BURST_ACK )
            {
                frame_ok = ( smtc_flrp_mac_handle_burst_ack( smtc_flrp_mac_layer ) == SMTC_FLRP_MAC_LAYER_STATUS_OK );
            }
            else
            {
                frame_ok = false;
            }
            smtc_flrp_mac_layer->radio_event =
                frame_ok ? SMTC_FLRP_MAC_RADIO_EVENT_TRX_OK : SMTC_FLRP_MAC_RADIO_EVENT_TRX_FAILED;
        }
        else if( smtc_flrp_mac_layer->state == SMTC_FLRP_MAC_LAYER_STATE_FLRC_REQ )
        {
            bool                   frame_ok = false;
            smtc_flrp_mac_header_t mac_header;
            smtc_flrp_mac_deserialize_header( smtc_flrp_mac_layer->rx_payload_buffer, &mac_header );
            if( mac_header.message_type == SMTC_FLRP_MAC_FRAME_TYPE_FLRC_REQ )
            {
                smtc_flrp_mac_burst_info_in_adaptive_link_t* burst_info = &smtc_flrp_mac_layer->burst_info;

                frame_ok = smtc_flrp_mac_handle_flrc_req_packet(
                    smtc_flrp_mac_layer->rx_payload_buffer,
                    smtc_flrp_mac_layer->transaction->smtc_rac_data_result.rx_size,
                    smtc_flrp_mac_layer->transaction->smtc_rac_data_result.rssi_result, smtc_flrp_mac_layer->dev_eui,
                    smtc_flrp_mac_layer->rx_frame_dev_eui_filter_len, smtc_flrp_mac_layer->destination_dev_eui,
                    &smtc_flrp_mac_layer->adaptive_link_config, burst_info );

                if( frame_ok )
                {
#ifndef TEST_LONG_PAYLOAD
                    if( burst_info->payload_length > smtc_flrp_mac_layer->rx_data_buffer_size )
                    {
                        SMTC_MODEM_HAL_TRACE_WARNING(
                            "FLRC Rx MAC: payload length received in FLRC REQ is bigger than the rx buffer size\n" );
                    }
#endif
                    smtc_flrp_mac_layer->number_of_packets =
                        ( burst_info->payload_length ) / ( burst_info->packet_uniformed_size ) +
                        ( ( ( burst_info->payload_length ) % ( burst_info->packet_uniformed_size ) != 0 ) ? 1 : 0 );

                    smtc_flrp_mac_layer->payload_stats.payload_size_expected =
                        smtc_flrp_mac_layer->burst_info.payload_length;
                    smtc_flrp_mac_layer->payload_stats.nb_packets_expected = smtc_flrp_mac_layer->number_of_packets;
                }
            }
            smtc_flrp_mac_layer->radio_event =
                frame_ok ? SMTC_FLRP_MAC_RADIO_EVENT_TRX_OK : SMTC_FLRP_MAC_RADIO_EVENT_TRX_FAILED;
        }
        else if( smtc_flrp_mac_layer->state == SMTC_FLRP_MAC_LAYER_STATE_FLRC_ACK )
        {
            bool                   frame_ok = false;
            smtc_flrp_mac_header_t mac_header;
            smtc_flrp_mac_deserialize_header( smtc_flrp_mac_layer->rx_payload_buffer, &mac_header );
            if( mac_header.message_type == SMTC_FLRP_MAC_FRAME_TYPE_FLRC_ACK )
            {
                frame_ok = smtc_flrp_mac_handle_flrc_ack_packet(
                    smtc_flrp_mac_layer->rx_payload_buffer,
                    smtc_flrp_mac_layer->transaction->smtc_rac_data_result.rx_size, smtc_flrp_mac_layer->dev_eui,
                    smtc_flrp_mac_layer->destination_dev_eui, &smtc_flrp_mac_layer->adaptive_link_config,
                    &smtc_flrp_mac_layer->burst_info, &smtc_flrp_mac_layer->mac_radio_config );

                if( frame_ok )
                {
                    smtc_flrp_mac_layer->next_burst_start_delay_ms =
                        smtc_flrp_mac_layer->burst_info.first_burst_start_delay_ms;
                }
            }
            smtc_flrp_mac_layer->radio_event =
                frame_ok ? SMTC_FLRP_MAC_RADIO_EVENT_TRX_OK : SMTC_FLRP_MAC_RADIO_EVENT_TRX_FAILED;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "FLRC Rx MAC Layer: Received frame in IDLE state -> ignore it \n" );
        }
        break;
    }
    case RP_STATUS_TASK_ABORTED:
        SMTC_MODEM_HAL_TRACE_INFO( "FLRC Rx MAC Layer: Task aborted\n" );
        smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_ABORTED;
        break;
    case RP_STATUS_RADIO_UNLOCKED:
        if( smtc_flrp_mac_layer->state == SMTC_FLRP_MAC_LAYER_STATE_BURST )
        {
            memcpy( smtc_flrp_mac_layer->payload_stats.last_burst_missed_packets_bitfield,
                    smtc_flrp_mac_layer->burst_missed_packets_bitfield, SMTC_FLRP_BITFIELD_PACKETS_IN_BURST_LENGTH );

#ifdef DEBUG_LOG
            if( !smtc_flrp_mac_layer->burst_ack_enabled )
            {
                SMTC_MODEM_HAL_TRACE_INFO(
                    "Burst %u -- RX packets OK: %u (rssi mean: %d), RX crc/mic error: %u, RX wrong packets: %u \n",
                    smtc_flrp_mac_layer->burst_number, smtc_flrp_mac_layer->burst_stats.nb_packets_received_ok,
                    smtc_flrp_mac_layer->burst_stats.rssi_mean, smtc_flrp_mac_layer->burst_stats.nb_packets_check_error,
                    smtc_flrp_mac_layer->burst_stats.nb_packets_received_nok );
                if( missed_packets_bitfield_get_nb_missed_packets( smtc_flrp_mac_layer ) > 0 )
                {
                    SMTC_MODEM_HAL_TRACE_ARRAY( "Bitfield RX packets (1 = missed)",
                                                smtc_flrp_mac_layer->burst_missed_packets_bitfield,
                                                SMTC_FLRP_BITFIELD_PACKETS_IN_BURST_LENGTH );
                }
            }
#endif
            smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_TRX_OK;
        }
        break;
    case RP_STATUS_RX_TIMEOUT:
    case RP_STATUS_RX_CRC_ERROR:
    {
        smtc_flrp_mac_layer->radio_end_timestamp_ms =
            smtc_flrp_mac_layer->transaction->smtc_rac_data_result.radio_start_timestamp_ms +
            SMTC_FLRP_TX_DELAY_MARGIN_MS + smtc_flrp_mac_layer->transaction->scheduler_config.duration_time_ms;

        if( smtc_flrp_mac_layer->state == SMTC_FLRP_MAC_LAYER_STATE_BURST )
        {
            // RP_STATUS_RX_TIMEOUT not used (always RP_STATUS_RADIO_UNLOCKED)
            if( status == RP_STATUS_RX_CRC_ERROR )
            {
                stats_increment_packet_received( smtc_flrp_mac_layer, true, false );
            }
        }
        else
        {
            smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_TRX_FAILED;
        }
        break;
    }
    default:
        SMTC_MODEM_HAL_TRACE_WARNING( "FLRC Rx MAC Layer: Unknown transaction status: 0x%02x\n", status );
        smtc_flrp_mac_layer->radio_event = SMTC_FLRP_MAC_RADIO_EVENT_TRX_FAILED;
        break;
    }
}

/* --- EOF ------------------------------------------------------------------ */