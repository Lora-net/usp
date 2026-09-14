/**
 * @file      smtc_wor_rx.c
 *
 * @brief     smtc_wor reception implementation
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

#include "smtc_wor_rx.h"
#include "smtc_wor_tests.h"

#include "flrp_defs.h"
#include "smtc_flrp_utils.h"
#include "smtc_flrp_api.h"

#include "flrp_configuration.h"

#include "smtc_modem_hal_dbg_trace.h"

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

static smtc_wor_status_t smtc_wor_deserialize_header( smtc_wor_header_t* header, uint8_t* packet,
                                                      uint16_t packet_size );

static void prepare_wor_ack_tx_packet( smtc_wor_t* smtc_wor_obj, smtc_wor_ack_data_t wor_ack_data );
static void prepare_rx_wor_packet( smtc_wor_t* smtc_wor_obj, smtc_flrp_wor_radio_config_t radio_config );

static smtc_wor_status_t send_wor_ack_packet( smtc_wor_t* smtc_wor_obj, uint32_t timestamp_ms );
static smtc_wor_status_t listen_wor_packet( smtc_wor_t* smtc_wor_obj, uint32_t timestamp_ms );

static void post_transaction_callback( rp_status_t status );
static void pre_transaction_callback( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_wor_rx_init( smtc_wor_t* smtc_wor_obj, smtc_wor_static_radio_config_t wor_config, uint8_t* dev_eui,
                       uint8_t hook_id, smtc_wor_done_f wor_done_cb )
{
    smtc_wor_init( smtc_wor_obj, wor_config, hook_id, wor_done_cb );

    smtc_wor_obj->dev_addr = dev_eui;

    smtc_wor_obj->transaction->scheduler_config.callback_pre_radio_transaction  = pre_transaction_callback;
    smtc_wor_obj->transaction->scheduler_config.callback_post_radio_transaction = post_transaction_callback;
}

smtc_wor_status_t smtc_wor_rx_start( smtc_wor_t* smtc_wor_obj, smtc_flrp_wor_radio_config_t radio_config,
                                     uint32_t timestamp_ms, bool data_to_send, smtc_wor_ack_data_t wor_ack,
                                     smtc_wor_rx_info_for_ack_t info_wor_ack )
{
    if( smtc_wor_obj->state != SMTC_WOR_STATE_IDLE )
    {
        return SMTC_WOR_STATUS_BUSY;
    }
    else
    {
        memset( &smtc_wor_obj->rx_metrics, 0, sizeof( smtc_flrp_wor_rx_stats_t ) );

        // SMTC_MODEM_HAL_TRACE_INFO( "smtc_wor_rx_start\n" );
        smtc_wor_obj->data_ready_to_be_sent = data_to_send;
        smtc_wor_obj->wor_ack_data_trx      = wor_ack;
        smtc_wor_obj->state                 = SMTC_WOR_STATE_WOR;
        smtc_wor_obj->info_wor_ack          = info_wor_ack;

        smtc_wor_set_dynamic_radio_config( smtc_wor_obj, radio_config );

        prepare_rx_wor_packet( smtc_wor_obj, radio_config );
        return listen_wor_packet( smtc_wor_obj, timestamp_ms );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION FOR TESTS -----------------------------------
 */

smtc_wor_status_t smtc_wor_tests_deserialize_header( smtc_wor_header_t* header, uint8_t* packet, uint16_t packet_size )
{
    return smtc_wor_deserialize_header( header, packet, packet_size );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION ---------------------------------------------
 */

static smtc_wor_status_t smtc_wor_deserialize_header( smtc_wor_header_t* header, uint8_t* packet, uint16_t packet_size )
{
    if( packet_size < SMTC_WOR_PAYLOAD_HEADER_SIZE )
    {
        return SMTC_WOR_STATUS_ERROR;
    }
    uint16_t header_packet;
    DESERIALIZE_16BITS( packet, header_packet );

    header->cipher_id   = ( header_packet & SMTC_WOR_HEADER_CIPHER_ID_MASK ) >> SMTC_WOR_HEADER_CIPHER_ID_SHIFT;
    header->version     = ( header_packet & SMTC_WOR_HEADER_VERSION_MASK ) >> SMTC_WOR_HEADER_VERSION_SHIFT;
    header->beacon_type = ( smtc_wor_type_t ) ( ( header_packet & SMTC_WOR_HEADER_BEACON_TYPE_MASK ) >>
                                                SMTC_WOR_HEADER_BEACON_TYPE_SHIFT );

    return SMTC_WOR_STATUS_SUCCESS;
}

static void prepare_wor_ack_tx_packet( smtc_wor_t* smtc_wor_obj, smtc_wor_ack_data_t wor_ack_data )
{
    uint16_t payload_size = 0;
    
    if( wor_ack_data.type == SMTC_WOR_TYPE_FLRC )
    {
        smtc_flrp_wor_ack_serialize_payload( wor_ack_data.u.flrc, smtc_wor_obj->tx_payload, &payload_size );
    }

    smtc_wor_obj->transaction->radio_params.lora.is_tx = true;
    smtc_wor_obj->transaction->radio_params.lora.preamble_len_in_symb =
        smtc_wor_obj->smtc_wor_radio_config.wor_ack_preamble_len_symb;
    smtc_wor_obj->transaction->radio_params.lora.invert_iq_is_on = !smtc_wor_obj->smtc_wor_radio_config.invert_iq_is_on;
    smtc_wor_obj->transaction->radio_params.lora.tx_size         = payload_size;

    smtc_wor_obj->transaction->lbt_context.lbt_enabled = false;

    smtc_wor_obj->transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = smtc_wor_obj->tx_payload;
    smtc_wor_obj->transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = payload_size;
}

static void prepare_rx_wor_packet( smtc_wor_t* smtc_wor_obj, smtc_flrp_wor_radio_config_t radio_config )
{
    // configure transaction context
    smtc_wor_obj->transaction->radio_params.lora.is_tx = false;
    smtc_wor_obj->transaction->radio_params.lora.preamble_len_in_symb =
        ( uint16_t ) ( ( uint32_t ) smtc_wor_obj->smtc_wor_radio_config.time_preamble_ms * 1000 /
                       smtc_wor_get_single_symbol_time_us( radio_config.bw, radio_config.sf ) );
    smtc_wor_obj->transaction->radio_params.lora.invert_iq_is_on = smtc_wor_obj->smtc_wor_radio_config.invert_iq_is_on;
    smtc_wor_obj->transaction->radio_params.lora.rx_timeout_ms   = smtc_wor_obj->smtc_wor_radio_config.rx_timeout;
    smtc_wor_obj->transaction->radio_params.lora.symb_nb_timeout = WOR_SYM_NB_TIMEOUT;
    smtc_wor_obj->transaction->radio_params.lora.max_rx_size     = SMTC_WOR_TX_RX_BUFFER_MAX_SIZE;

    smtc_wor_obj->transaction->smtc_rac_data_buffer_setup.rx_payload_buffer = smtc_wor_obj->rx_payload;
    smtc_wor_obj->transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer =
        sizeof( smtc_wor_obj->rx_payload );
}

static smtc_wor_status_t send_wor_ack_packet( smtc_wor_t* smtc_wor_obj, uint32_t timestamp_ms )
{
    smtc_wor_obj->transaction->scheduler_config.scheduling    = SMTC_RAC_SCHEDULED_TRANSACTION;
    smtc_wor_obj->transaction->scheduler_config.start_time_ms = timestamp_ms + SMTC_FLRP_TX_DELAY_MARGIN_MS;
    return convert_rac_status_to_wor_status( smtc_rac_submit_radio_transaction( smtc_wor_obj->radio_access_id ) );
}

static smtc_wor_status_t listen_wor_packet( smtc_wor_t* smtc_wor_obj, uint32_t timestamp_ms )
{
    smtc_wor_obj->transaction->scheduler_config.callback_post_radio_transaction = post_transaction_callback;
    smtc_wor_obj->transaction->scheduler_config.callback_pre_radio_transaction  = pre_transaction_callback;
    smtc_wor_obj->transaction->scheduler_config.start_time_ms                   = timestamp_ms;
    smtc_wor_obj->transaction->scheduler_config.scheduling                      = SMTC_RAC_ASAP_TRANSACTION;

    return convert_rac_status_to_wor_status( smtc_rac_submit_radio_transaction( smtc_wor_obj->radio_access_id ) );
    // SMTC_MODEM_HAL_TRACE_INFO( "listen WOR packet\n" );
}

static void post_transaction_callback( rp_status_t status )
{
    uint8_t     radio_access_id = smtc_rac_get_callback_radio_id( );
    smtc_wor_t* smtc_wor        = ( smtc_wor_t* ) smtc_rac_get_context_private( radio_access_id );
    bool        wor_not_found   = false;

    switch( status )
    {
    case RP_STATUS_TX_DONE:
    {
        if( smtc_wor->state == SMTC_WOR_STATE_WOR_ACK )
        {
            if( smtc_wor->wor_ack_data_trx.u.flrc.ack_status == SMTC_FLRP_WOR_ACCEPTED )
            {
                set_mask_bit( SMTC_FLRP_EXCHANGE_WOR_ACK_SUCCESS, true );
            }
            enter_idle_state_and_send_wor_cb( smtc_wor,
                                              ( smtc_wor->wor_ack_data_trx.u.flrc.ack_status == SMTC_FLRP_WOR_ACCEPTED )
                                                  ? SMTC_WOR_STATUS_SUCCESS
                                                  : SMTC_WOR_STATUS_FAILED );
        }
        break;
    }
    case RP_STATUS_RX_PACKET:
    {
        if( smtc_wor->state == SMTC_WOR_STATE_WOR )
        {
            smtc_wor_header_t header     = { 0 };
            smtc_wor_status_t wor_status = smtc_wor_deserialize_header(
                &header, smtc_wor->rx_payload, smtc_wor->transaction->smtc_rac_data_result.rx_size );

            //  Frequency offset measurement
            smtc_wor->wor_rx_measurements.freq_offset_hz =
                smtc_wor->transaction->smtc_rac_data_result.lora_freq_offset_hz;

            if( ( wor_status == SMTC_WOR_STATUS_SUCCESS ) && ( header.beacon_type == SMTC_WOR_TYPE_FLRC ) )
            {
                smtc_wor->wor_data_trx.type = SMTC_WOR_TYPE_FLRC;
                if( !smtc_flrp_wor_deserialize_payload(
                        &smtc_wor->wor_data_trx.u.flrc, &smtc_wor->rx_payload[SMTC_WOR_PAYLOAD_HEADER_SIZE],
                        smtc_wor->transaction->smtc_rac_data_result.rx_size - SMTC_WOR_PAYLOAD_HEADER_SIZE ) )
                {
                    SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-RX] payload deserialize failed\n" );
                    wor_not_found = true;
                }
                else
                {
                    if( smtc_flrp_is_frame_addressed_to_this_device( smtc_wor->dev_addr,
                                                                     smtc_wor->wor_data_trx.u.flrc.slave_dev_eui,
                                                                     smtc_wor->wor_data_trx.u.flrc.filter_len ) )
                    {
                        bool send_ack;

                        smtc_wor->wor_ack_data_trx.type = SMTC_WOR_TYPE_FLRC;
                        smtc_flrp_wor_handle_payload(
                            smtc_wor->wor_data_trx.u.flrc, smtc_wor->transaction->smtc_rac_data_result,
                            smtc_wor->data_ready_to_be_sent, &send_ack, &smtc_wor->wor_ack_data_trx.u.flrc,
                            smtc_wor->info_wor_ack.u.flrc );
                        set_mask_bit( SMTC_FLRP_EXCHANGE_WOR_TX_SUCCESS, true );

                        smtc_wor->rx_metrics.rssi = smtc_wor->transaction->smtc_rac_data_result.rssi_result;
                        smtc_wor->rx_metrics.snr  = smtc_wor->transaction->smtc_rac_data_result.snr_result;

                        if( send_ack )
                        {
                            smtc_wor->state = SMTC_WOR_STATE_WOR_ACK;
                            prepare_wor_ack_tx_packet( smtc_wor, smtc_wor->wor_ack_data_trx );
                            smtc_wor_status_t wor_ack_status = send_wor_ack_packet(
                                smtc_wor, smtc_wor->transaction->smtc_rac_data_result.radio_end_timestamp_ms +
                                              ( smtc_wor->wor_data_trx.u.flrc.next_phase_start_delay_us / 1000 ) );
                            if( wor_ack_status != SMTC_WOR_STATUS_SUCCESS )
                            {
                                SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-RX] ACK TX submit failed status=%u\n",
                                                              ( unsigned ) wor_ack_status );
                                enter_idle_state_and_send_wor_cb( smtc_wor, wor_ack_status );
                            }
                        }
                        else
                        {
                            enter_idle_state_and_send_wor_cb( smtc_wor, SMTC_WOR_STATUS_SUCCESS );
                        }
                    }
                    else
                    {
                        SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-RX] address mismatch\n" );
                        wor_not_found = true;
                    }
                }
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-RX] invalid header (status=%u type=%u)\n", ( unsigned ) wor_status,
                                              ( unsigned ) header.beacon_type );
                wor_not_found = true;
            }
        }
        break;
    }
    case RP_STATUS_RX_TIMEOUT:
    {
        // SMTC_MODEM_HAL_TRACE_WARNING( "Reception timeout\n" );
        wor_not_found = true;
        break;
    }
    case RP_STATUS_RX_CRC_ERROR:
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-RX] CRC ERROR on RX packet (preamble seen but data corrupted)\n" );
        wor_not_found = true;
        break;
    }
    case RP_STATUS_TASK_ABORTED:
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "[WOR-RX] Task aborted by radio planner\n" );
        enter_idle_state_and_send_wor_cb( smtc_wor, SMTC_WOR_STATUS_ABORT );
        break;
    }

    default:
    {
        wor_not_found = true;
        SMTC_MODEM_HAL_TRACE_ERROR( "[WOR-RX] Unknown transaction status: %d\n", status );
        break;
    }
    }

    if( wor_not_found )
    {
        enter_idle_state_and_send_wor_cb( smtc_wor, SMTC_WOR_STATUS_FAILED );
    }
}

static void pre_transaction_callback( void )
{
}

/* --- EOF ------------------------------------------------------------------ */
