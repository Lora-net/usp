/**
 * @file      smtc_wor_tx.c
 *
 * @brief     smtc_wor transmission implementation
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

#include "smtc_wor_tx.h"
#include "smtc_wor_tests.h"

#include "flrp_defs.h"
#include "flrp_configuration.h"
#include "smtc_flrp_api.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_flrp_utils.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define SMTC_WOR_HEADER_CIPHER_ID_MASK 0x000F
#define SMTC_WOR_HEADER_CIPHER_ID_SHIFT 0
#define SMTC_WOR_HEADER_VERSION_MASK 0x0030
#define SMTC_WOR_HEADER_VERSION_SHIFT 4
#define SMTC_WOR_HEADER_BEACON_TYPE_MASK 0x0FC0
#define SMTC_WOR_HEADER_BEACON_TYPE_SHIFT 6

#define DEV_EUI_FILTER_LEN_MAX ( ( SMTC_FLRP_EUI_LENGTH * 8 ) - 1 )

#define SMTC_WOR_PREAMBLE_DURATION_MIN_MS 10
#define SMTC_WOR_NB_SYM_TIMEOUT 16
#define MAX_RX_WINDOW_SYMB 248  // open rx window at max 248 symbol hardware limitation

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static uint16_t get_lora_symb_timeout( smtc_wor_t* smtc_wor_obj );

static void smtc_wor_serialize_header( smtc_wor_header_t header, uint8_t* packet );

static void prepare_wor_packet( smtc_wor_t* smtc_wor_obj, smtc_flrp_wor_radio_config_t radio_config,
                                smtc_wor_data_t wor_data );
static void prepare_wor_ack_rx_packet( smtc_wor_t* smtc_wor_obj );

static smtc_wor_status_t send_wor_packet( smtc_wor_t* smtc_wor_obj );
static smtc_wor_status_t listen_wor_ack_packet( smtc_wor_t* smtc_wor_obj, uint32_t timestamp_ms );

static void post_transaction_callback( rp_status_t status );
static void pre_transaction_callback( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_wor_tx_init( smtc_wor_t* smtc_wor_obj, smtc_wor_static_radio_config_t wor_config, uint8_t* dev_eui,
                       uint8_t hook_id, smtc_wor_done_f wor_done_cb, uint32_t crystal_error )
{
    smtc_wor_init( smtc_wor_obj, wor_config, hook_id, wor_done_cb );

    smtc_wor_obj->dev_addr      = dev_eui;
    smtc_wor_obj->crystal_error = crystal_error;

    smtc_wor_obj->transaction->scheduler_config.callback_pre_radio_transaction  = pre_transaction_callback;
    smtc_wor_obj->transaction->scheduler_config.callback_post_radio_transaction = post_transaction_callback;
}

smtc_wor_status_t smtc_wor_tx_start( smtc_wor_t* smtc_wor_obj, smtc_flrp_wor_radio_config_t radio_config,
                                     smtc_wor_data_t wor_data )
{
    // Transfer direction = 1
    if( smtc_wor_obj->state != SMTC_WOR_STATE_IDLE )
    {
        return SMTC_WOR_STATUS_BUSY;
    }
    else
    {
        memset( &smtc_wor_obj->rx_metrics, 0, sizeof( smtc_flrp_wor_rx_stats_t ) );

        smtc_wor_obj->state        = SMTC_WOR_STATE_WOR;
        smtc_wor_obj->type         = wor_data.type;
        smtc_wor_obj->ack_required = wor_data.u.flrc.wor_ack_required;
        smtc_wor_obj->wor_data_trx = wor_data;
        smtc_wor_set_dynamic_radio_config( smtc_wor_obj, radio_config );
        prepare_wor_packet( smtc_wor_obj, radio_config, smtc_wor_obj->wor_data_trx );
        return send_wor_packet( smtc_wor_obj );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION FOR TESTS -----------------------------------
 */

void smtc_wor_tests_serialize_header( smtc_wor_header_t header, uint8_t* packet )
{
    smtc_wor_serialize_header( header, packet );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION ---------------------------------------------
 */

static uint16_t get_lora_symb_timeout( smtc_wor_t* smtc_wor_obj )
{
    uint32_t time_symbol_us = smtc_wor_get_single_symbol_time_us( smtc_wor_obj->transaction->radio_params.lora.bw,
                                                                  smtc_wor_obj->transaction->radio_params.lora.sf );
    uint32_t rx_timeout_ms  = MAX( ( ( ( ( WOR_ACK_DELAY_MS * 2 * smtc_wor_obj->crystal_error ) / 1000 ) +
                                      ( SMTC_WOR_NB_SYM_TIMEOUT * time_symbol_us ) ) /
                                    1000 ),
                                   SMTC_WOR_PREAMBLE_DURATION_MIN_MS );

    uint16_t rx_window_symb =
        MIN( MAX( ( ( rx_timeout_ms * 1000 ) / time_symbol_us ), SMTC_WOR_NB_SYM_TIMEOUT ), MAX_RX_WINDOW_SYMB );

    // Because the hardware allows an even number of symbols
    if( ( ( rx_window_symb % 2 ) == 1 ) && ( rx_window_symb != MAX_RX_WINDOW_SYMB ) )
    {
        rx_window_symb += 1;
    }
    return rx_window_symb;
}

static void smtc_wor_serialize_header( smtc_wor_header_t header, uint8_t* packet )
{
    uint16_t header_packet;
    header_packet = ( header.cipher_id << SMTC_WOR_HEADER_CIPHER_ID_SHIFT ) & SMTC_WOR_HEADER_CIPHER_ID_MASK;
    header_packet |= ( header.version << SMTC_WOR_HEADER_VERSION_SHIFT ) & SMTC_WOR_HEADER_VERSION_MASK;
    header_packet |= ( header.beacon_type << SMTC_WOR_HEADER_BEACON_TYPE_SHIFT ) & SMTC_WOR_HEADER_BEACON_TYPE_MASK;

    SERIALIZE_16BITS( packet, header_packet );
}

static void prepare_wor_packet( smtc_wor_t* smtc_wor_obj, smtc_flrp_wor_radio_config_t radio_config,
                                smtc_wor_data_t wor_data )
{
    uint16_t payload_size = 0;
    uint16_t packet_size = 0;

    // configure transaction context
    smtc_wor_obj->transaction->radio_params.lora.is_tx = true;
    smtc_wor_obj->transaction->radio_params.lora.preamble_len_in_symb =
        ( uint16_t ) ( ( uint32_t ) smtc_wor_obj->smtc_wor_radio_config.time_preamble_ms * 1000 /
                       smtc_wor_get_single_symbol_time_us( radio_config.bw, radio_config.sf ) );
    smtc_wor_obj->transaction->radio_params.lora.invert_iq_is_on = smtc_wor_obj->smtc_wor_radio_config.invert_iq_is_on;

    smtc_wor_obj->transaction->lbt_context.lbt_enabled = false;

    smtc_wor_header_t header = {
        .beacon_type = wor_data.type,
        .cipher_id   = 0,
        .version     = 0,
    };
    smtc_wor_serialize_header( header, smtc_wor_obj->tx_payload );

    if( wor_data.type == SMTC_WOR_TYPE_FLRC )
    {
        smtc_flrp_wor_serialize_payload( wor_data.u.flrc, &( smtc_wor_obj->tx_payload )[SMTC_WOR_PAYLOAD_HEADER_SIZE],
                                         &payload_size );
    }

    memcpy( &smtc_wor_obj->tx_payload[SMTC_WOR_PAYLOAD_HEADER_SIZE + payload_size], wor_data.signature,
            SMTC_WOR_PAYLOAD_SIGN_SIZE );

    packet_size = SMTC_WOR_PAYLOAD_HEADER_SIZE + payload_size + SMTC_WOR_PAYLOAD_SIGN_SIZE;

    smtc_wor_obj->transaction->radio_params.lora.tx_size = packet_size;

    smtc_wor_obj->transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = smtc_wor_obj->tx_payload;
    smtc_wor_obj->transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = packet_size;
}

static void prepare_wor_ack_rx_packet( smtc_wor_t* smtc_wor_obj )
{
    smtc_wor_obj->transaction->radio_params.lora.is_tx = false;
    smtc_wor_obj->transaction->radio_params.lora.preamble_len_in_symb =
        smtc_wor_obj->smtc_wor_radio_config.wor_ack_preamble_len_symb;
    smtc_wor_obj->transaction->radio_params.lora.invert_iq_is_on = !smtc_wor_obj->smtc_wor_radio_config.invert_iq_is_on;
    smtc_wor_obj->transaction->radio_params.lora.rx_timeout_ms = smtc_wor_obj->smtc_wor_radio_config.wor_ack_rx_timeout;
    smtc_wor_obj->transaction->radio_params.lora.symb_nb_timeout = get_lora_symb_timeout( smtc_wor_obj );

    smtc_wor_obj->transaction->radio_params.lora.max_rx_size = SMTC_WOR_TX_RX_BUFFER_MAX_SIZE;

    smtc_wor_obj->transaction->smtc_rac_data_buffer_setup.rx_payload_buffer = smtc_wor_obj->rx_payload;
    smtc_wor_obj->transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer =
        sizeof( smtc_wor_obj->rx_payload );
}

static smtc_wor_status_t send_wor_packet( smtc_wor_t* smtc_wor_obj )
{
    smtc_wor_obj->transaction->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    smtc_wor_obj->transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( );
    return convert_rac_status_to_wor_status( smtc_rac_submit_radio_transaction( smtc_wor_obj->radio_access_id ) );
    // SMTC_MODEM_HAL_TRACE_INFO( "WOR packet request scheduled\n" );
}

static smtc_wor_status_t listen_wor_ack_packet( smtc_wor_t* smtc_wor_obj, uint32_t timestamp_ms )
{
    uint32_t time_rx_margin_us = ( smtc_wor_obj->transaction->radio_params.lora.symb_nb_timeout / 2 ) *
                                 smtc_wor_get_single_symbol_time_us( smtc_wor_obj->transaction->radio_params.lora.bw,
                                                                     smtc_wor_obj->transaction->radio_params.lora.sf );
    if( time_rx_margin_us > WOR_ACK_DELAY_MS * 1000 )
    {
        time_rx_margin_us = WOR_ACK_DELAY_MS * 1000;
    }
    smtc_wor_obj->transaction->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    smtc_wor_obj->transaction->scheduler_config.start_time_ms = timestamp_ms - CEIL_DIVISION( time_rx_margin_us, 1000 );
    return convert_rac_status_to_wor_status( smtc_rac_submit_radio_transaction( smtc_wor_obj->radio_access_id ) );
    // SMTC_MODEM_HAL_TRACE_INFO( "listen WOR ACK packet\n" );
}

static void post_transaction_callback( rp_status_t status )
{
    uint8_t     radio_access_id = smtc_rac_get_callback_radio_id( );
    smtc_wor_t* smtc_wor        = ( smtc_wor_t* ) smtc_rac_get_context_private( radio_access_id );
    bool        wor_failed      = false;

    switch( status )
    {
    case RP_STATUS_TX_DONE:
    {
        if( smtc_wor->state == SMTC_WOR_STATE_WOR )
        {
            set_mask_bit( SMTC_FLRP_EXCHANGE_WOR_TX_SUCCESS, true );
            if( smtc_wor->ack_required )
            {
                smtc_wor->state = SMTC_WOR_STATE_WOR_ACK;
                prepare_wor_ack_rx_packet( smtc_wor );

                smtc_wor_status_t wor_ack_status = listen_wor_ack_packet(
                    smtc_wor, smtc_wor->transaction->smtc_rac_data_result.radio_end_timestamp_ms +
                                  smtc_wor->smtc_wor_radio_config.wor_ack_delay_ms );
                if( wor_ack_status != SMTC_WOR_STATUS_SUCCESS )
                {
                    SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-TX] ACK RX submit failed status=%u\n",
                                                  ( unsigned ) wor_ack_status );
                    enter_idle_state_and_send_wor_cb( smtc_wor, wor_ack_status );
                }
            }
            else
            {
                enter_idle_state_and_send_wor_cb( smtc_wor, SMTC_WOR_STATUS_SUCCESS );
            }
        }
        break;
    }
    case RP_STATUS_RX_PACKET:
    {
        if( smtc_wor->state == SMTC_WOR_STATE_WOR_ACK )
        {
            //  Frequency offset measurement
            smtc_wor->wor_rx_measurements.freq_offset_hz =
                smtc_wor->transaction->smtc_rac_data_result.lora_freq_offset_hz;

            if( smtc_wor->type == SMTC_WOR_TYPE_FLRC )
            {
                wor_failed = true;

                smtc_wor->wor_ack_data_trx.u.flrc.has_min_interframe_delay =
                    ( smtc_wor->wor_data_trx.u.flrc.initiator_send_burst );
                if( !smtc_flrp_wor_ack_deserialize_payload( &smtc_wor->wor_ack_data_trx.u.flrc, smtc_wor->rx_payload,
                                                            smtc_wor->transaction->smtc_rac_data_result.rx_size ) )
                {
                    SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-TX] ACK deserialize failed\n" );
                }
                else
                {
                    if( smtc_flrp_is_frame_addressed_to_this_device( smtc_wor->dev_addr,
                                                                     smtc_wor->wor_ack_data_trx.u.flrc.receiver_dev_eui,
                                                                     DEV_EUI_FILTER_LEN_MAX ) )
                    {
                        smtc_wor->rx_metrics.rssi = smtc_wor->transaction->smtc_rac_data_result.rssi_result;
                        smtc_wor->rx_metrics.snr  = smtc_wor->transaction->smtc_rac_data_result.snr_result;

                        if( smtc_wor->wor_ack_data_trx.u.flrc.ack_status == SMTC_FLRP_WOR_ACCEPTED )
                        {
                            set_mask_bit( SMTC_FLRP_EXCHANGE_WOR_ACK_SUCCESS, true );
                            enter_idle_state_and_send_wor_cb( smtc_wor, SMTC_WOR_STATUS_SUCCESS );
                            wor_failed = false;
                        }
                        else
                        {
                            SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-TX] ACK declined by slave\n" );
                        }
                    }
                    else
                    {
                        SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-TX] ACK receiver mismatch\n" );
                    }
                }
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-TX] WOR type not handled\n" );
                wor_failed = true;
            }
        }
        break;
    }
    case RP_STATUS_RX_TIMEOUT:
    {
        // SMTC_MODEM_HAL_TRACE_WARNING( "Reception timeout\n" );
        wor_failed = true;
        break;
    }
    case RP_STATUS_RX_CRC_ERROR:
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-TX] CRC ERROR on ACK packet (preamble seen but data corrupted)\n" );
        wor_failed = true;
        break;
    }
    case RP_STATUS_TASK_ABORTED:
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-TX] Task aborted by radio planner\n" );
        enter_idle_state_and_send_wor_cb( smtc_wor, SMTC_WOR_STATUS_ABORT );
        break;
    }

    default:
    {
        wor_failed = true;
        SMTC_MODEM_HAL_TRACE_ERROR( "[WOR-TX] Unknown transaction status: %d\n", status );
        break;
    }
    }

    if( wor_failed )
    {
        enter_idle_state_and_send_wor_cb( smtc_wor, SMTC_WOR_STATUS_FAILED );
    }
}

static void pre_transaction_callback( void )
{
}

/* --- EOF ------------------------------------------------------------------ */
