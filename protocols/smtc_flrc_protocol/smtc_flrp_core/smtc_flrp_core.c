/**
 * @file      smtc_flrp_core.c
 *
 * @brief     smtc_flrp_core api implementation
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

#include "smtc_flrp_mac_layer.h"
#include "smtc_rac.h"
#include "smtc_rac_api.h"
#include "smtc_flrp_api.h"
#include "smtc_flrp_core.h"
#include "smtc_flrp_mac_config.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_wor.h"

#include "smtc_flrp_api_tests.h"
#include "smtc_flrp_utils.h"

#include "flrp_configuration.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define SANITY_CHECK_FLRC_API_INITIALIZED( flrc_core ) \
    if( !flrc_core.initialized )                       \
    {                                                  \
        return SMTC_FLRP_RC_NOT_INIT;                  \
    }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define SMTC_FLRP_DELAYS_MAX \
    ( UINT8_MAX * 100 )  // Values have a 100us step and are transmitted in an uint8_t in frames
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static smtc_flrp_core_t smtc_flrp_core_obj;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static bool                    is_periodic_cad_enabled( smtc_flrp_core_t* smtc_flrp_core );
static smtc_flrp_return_code_t convert_wor_status_to_flrc_error_code( smtc_wor_status_t wor_status );
static smtc_flrp_return_code_t convert_mac_status_to_flrc_error_code( smtc_flrp_mac_layer_status_t mac_status );

static void smtc_flrp_configure_wor_data( smtc_flrp_core_t* smtc_flrp_core, smtc_wor_data_t* wor_data,
                                          smtc_flrp_com_config_t com_config );
static void smtc_flrp_configure_wor_ack_data( smtc_wor_ack_data_t* wor_ack_data );

static void smtc_flrp_core_return_error( smtc_flrp_core_t* smtc_flrp_core, smtc_flrp_return_code_t status,
                                         uint8_t* dest_addr, bool is_tx );
static void smtc_flrp_core_wor_tx_done_cb( smtc_wor_status_t status, smtc_flrp_wor_rx_stats_t rx_metrics );
static void smtc_flrp_core_wor_rx_done_cb( smtc_wor_status_t status, smtc_flrp_wor_rx_stats_t rx_metrics );
static void smtc_flrp_mac_tx_done_cb( smtc_flrp_mac_layer_status_t status, uint32_t timestamp_ms, uint8_t* dest_addr );
static void smtc_flrp_mac_rx_done_cb( smtc_flrp_mac_layer_status_t status, uint32_t data_size, uint32_t timestamp_ms,
                                      uint8_t* dest_addr );

static smtc_wor_status_t smtc_flrp_start_wor_rx( smtc_flrp_core_t* smtc_flrp_core_obj, uint32_t timestamp_start_rx_ms );
static void smtc_flrp_restart_reception( smtc_flrp_core_t* smtc_flrp_core_obj, uint32_t radio_end_timestamp_ms );

static void smtc_flrp_init_and_get_config( smtc_flrp_mac_radio_config_t* flrc_radio_config,
                                           smtc_flrp_wor_radio_config_t* wor_rx_radio_config,
                                           smtc_flrp_wor_radio_config_t* wor_tx_radio_config,
                                           smtc_flrp_frequency_plan_t    freq_plan );

static void smtc_flrp_core_start_mac( smtc_flrp_core_t* smtc_flrp_core, bool device_is_initiator );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bit_mask_t* get_mask( void )
{
    if( !smtc_flrp_core_obj.initialized )
    {
        return NULL;
    }
    return &smtc_flrp_core_obj.wor_exchange_phase_bits;
}

smtc_flrp_return_code_t smtc_flrp_init( smtc_flrp_api_config_t config, smtc_flrp_tx_done_f tx_callback,
                                        smtc_flrp_rx_done_f rx_callback, void* user_context )
{
    if( smtc_flrp_core_obj.initialized )
    {
        return SMTC_FLRP_RC_UNSUPPORTED_FEATURE;
    }

    memset( &smtc_flrp_core_obj, 0, sizeof( smtc_flrp_core_t ) );
    smtc_flrp_bind_wor_exchange_phase_mask( &smtc_flrp_core_obj.wor_exchange_phase_bits );

    memcpy( smtc_flrp_core_obj.dev_eui, config.dev_eui, SMTC_FLRP_EUI_LENGTH );

    smtc_wor_static_radio_config_t wor_static_radio_config = {
        .cr                        = WOR_LORA_CODING_RATE,
        .time_preamble_ms          = WOR_LORA_LONG_PREAMBLE_MS,
        .wor_ack_preamble_len_symb = WOR_ACK_LORA_PREAMBLE_LENGTH,
        .invert_iq_is_on           = WOR_LORA_IQ,
        .crc_is_on                 = WOR_LORA_CRC,
        .rx_timeout                = WOR_RX_TIMEOUT,
        .wor_ack_rx_timeout        = WOR_ACK_RX_TIMEOUT,
        .wor_ack_delay_ms          = WOR_ACK_DELAY_MS,
        .header_type               = LORA_PKT_LEN_MODE,
        .sync_word                 = LORA_SYNCWORD,
    };

    smtc_wor_tx_init( &smtc_flrp_core_obj.smtc_wor_tx_obj, wor_static_radio_config, smtc_flrp_core_obj.dev_eui,
                      RP_HOOK_RAC_FLRP_WOR_TX, smtc_flrp_core_wor_tx_done_cb, config.crystal_error );
    smtc_wor_rx_init( &smtc_flrp_core_obj.smtc_wor_rx_obj, wor_static_radio_config, smtc_flrp_core_obj.dev_eui,
                      RP_HOOK_RAC_FLRP_WOR_RX, smtc_flrp_core_wor_rx_done_cb );

    smtc_flrp_init_and_get_config( &smtc_flrp_core_obj.flrc_radio_config, &smtc_flrp_core_obj.wor_rx_radio_config,
                                   &smtc_flrp_core_obj.wor_tx_radio_config, config.freq_plan );

    if( smtc_flrp_mac_layer_init( &smtc_flrp_core_obj.smtc_flrp_mac_layer_obj, smtc_flrp_core_obj.dev_eui,
                                  RP_HOOK_RAC_FLRP_MAC, config.crypto_enabled, smtc_flrp_mac_tx_done_cb,
                                  smtc_flrp_mac_rx_done_cb ) != SMTC_FLRP_MAC_LAYER_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "SMTC FLRC CORE : Failed to init mac layer\n" );
        return SMTC_FLRP_RC_ERROR;
    }

    smtc_flrp_core_obj.tx_user_callback = tx_callback;
    smtc_flrp_core_obj.rx_user_callback = rx_callback;
    smtc_flrp_core_obj.user_context     = user_context;

    smtc_flrp_core_obj.state       = SMTC_FLRP_CORE_STATE_IDLE;
    smtc_flrp_core_obj.initialized = true;

    SMTC_MODEM_HAL_TRACE_INFO( "SMTC FLRC CORE : initialized (device %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X) \n",
                               smtc_flrp_core_obj.dev_eui[0], smtc_flrp_core_obj.dev_eui[1],
                               smtc_flrp_core_obj.dev_eui[2], smtc_flrp_core_obj.dev_eui[3],
                               smtc_flrp_core_obj.dev_eui[4], smtc_flrp_core_obj.dev_eui[5],
                               smtc_flrp_core_obj.dev_eui[6], smtc_flrp_core_obj.dev_eui[7] );

    return SMTC_FLRP_RC_OK;
}

smtc_flrp_return_code_t smtc_flrp_initiate_transmission( uint8_t* payload, uint32_t payload_size,
                                                         smtc_flrp_com_config_t com_config )
{
    SANITY_CHECK_FLRC_API_INITIALIZED( smtc_flrp_core_obj );

    if( ( payload_size == 0 ) || ( payload == NULL ) )
    {
        return SMTC_FLRP_RC_INVALID_PARAMS;
    }

    SMTC_MODEM_HAL_TRACE_INFO( "SMTC FLRC CORE : initiate transmission\n" );

    smtc_flrp_core_obj.wor_exchange_phase_bits = 0;

    smtc_wor_data_t tx_wor_data;
    smtc_flrp_configure_wor_data( &smtc_flrp_core_obj, &tx_wor_data, com_config );
    tx_wor_data.u.flrc.initiator_send_burst = true;

    smtc_flrp_core_obj.tx_initiator_data_buffer      = payload;
    smtc_flrp_core_obj.tx_initiator_data_buffer_size = payload_size;
    smtc_wor_status_t wor_status =
        smtc_wor_tx_start( &smtc_flrp_core_obj.smtc_wor_tx_obj, smtc_flrp_core_obj.wor_tx_radio_config, tx_wor_data );
    if( wor_status == SMTC_WOR_STATUS_SUCCESS )
    {
        smtc_flrp_core_obj.state = SMTC_FLRP_CORE_STATE_WOR_TX;
    }

    return convert_wor_status_to_flrc_error_code( wor_status );
}

smtc_flrp_return_code_t smtc_flrp_initiate_reception( uint8_t* payload, uint32_t payload_size,
                                                      smtc_flrp_com_config_t com_config )
{
    SANITY_CHECK_FLRC_API_INITIALIZED( smtc_flrp_core_obj );

    if( ( payload_size == 0 ) || ( payload == NULL ) || ( com_config.com_mode == SMTC_FLRP_COM_ONE_WAY ) )
    {
        return SMTC_FLRP_RC_INVALID_PARAMS;
    }

    SMTC_MODEM_HAL_TRACE_INFO( "SMTC FLRC CORE : initiate reception\n" );

    smtc_flrp_core_obj.wor_exchange_phase_bits = 0;

    smtc_flrp_core_obj.rx_initiator_data_buffer      = payload;
    smtc_flrp_core_obj.rx_initiator_data_buffer_size = payload_size;

    smtc_wor_data_t tx_wor_data;
    smtc_flrp_configure_wor_data( &smtc_flrp_core_obj, &tx_wor_data, com_config );
    tx_wor_data.u.flrc.initiator_send_burst = false;

    smtc_wor_status_t wor_status =
        smtc_wor_tx_start( &smtc_flrp_core_obj.smtc_wor_tx_obj, smtc_flrp_core_obj.wor_tx_radio_config, tx_wor_data );
    if( wor_status == SMTC_WOR_STATUS_SUCCESS )
    {
        smtc_flrp_core_obj.state = SMTC_FLRP_CORE_STATE_WOR_TX;
    }

    return convert_wor_status_to_flrc_error_code( wor_status );
}

smtc_flrp_return_code_t smtc_flrp_start_periodic_listening( uint8_t* payload, uint32_t payload_size )
{
    SANITY_CHECK_FLRC_API_INITIALIZED( smtc_flrp_core_obj );

    if( ( payload_size == 0 ) || ( payload == NULL ) )
    {
        return SMTC_FLRP_RC_INVALID_PARAMS;
    }

    SMTC_MODEM_HAL_TRACE_INFO( "SMTC FLRC CORE : start periodic RX\n" );

    smtc_flrp_core_obj.wor_exchange_phase_bits = 0;

    smtc_flrp_core_obj.rx_slave_data_buffer      = payload;
    smtc_flrp_core_obj.rx_slave_data_buffer_size = payload_size;

    smtc_flrp_core_obj.cad_period_ms = RESTART_WOR_RX_DELAY_MS;

    smtc_wor_status_t wor_status = smtc_flrp_start_wor_rx(
        &smtc_flrp_core_obj, smtc_modem_hal_get_time_in_ms( ) + smtc_flrp_core_obj.cad_period_ms );
    if( wor_status == SMTC_WOR_STATUS_SUCCESS )
    {
        smtc_flrp_core_obj.state = SMTC_FLRP_CORE_STATE_WOR_RX;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "SMTC FLRC CORE : Failed to start WOR reception\n" );
    }

    return convert_wor_status_to_flrc_error_code( wor_status );
}

smtc_flrp_return_code_t smtc_flrp_stop_periodic_listening( void )
{
    SMTC_MODEM_HAL_TRACE_INFO( "SMTC FLRC CORE : stop periodic RX\n" );

    smtc_flrp_core_obj.cad_period_ms = 0;

    return SMTC_FLRP_RC_OK;
}

smtc_flrp_return_code_t smtc_flrp_slave_prepare_data_to_send( uint8_t* payload, uint32_t payload_size )
{
    SANITY_CHECK_FLRC_API_INITIALIZED( smtc_flrp_core_obj );

    if( smtc_flrp_core_obj.state == SMTC_FLRP_CORE_STATE_DATA )
    {
        // Do not change the buffer if it is currently being sent
        return SMTC_FLRP_RC_BUSY;
    }

    smtc_flrp_core_obj.tx_slave_data_buffer        = payload;
    smtc_flrp_core_obj.tx_slave_data_buffer_size   = payload_size;
    smtc_flrp_core_obj.data_slave_ready_to_be_sent = ( payload_size > 0 );

    return SMTC_FLRP_RC_OK;
}

smtc_flrp_radio_config_t smtc_flrp_get_current_radio_config( void )
{
    smtc_flrp_radio_config_t flrp_radio_config = {
        .wor_rx                    = smtc_flrp_core_obj.wor_rx_radio_config,
        .wor_tx                    = smtc_flrp_core_obj.wor_tx_radio_config,
        .flrc.nb_channels          = smtc_flrp_core_obj.flrc_radio_config.nb_frequencies,
        .flrc.default_channel      = smtc_flrp_core_obj.flrc_radio_config.default_channel,
        .flrc.raw_bit_rate         = smtc_flrp_core_obj.flrc_radio_config.raw_bit_rate,
        .flrc.tx_power_in_dbm      = smtc_flrp_core_obj.flrc_radio_config.tx_power_in_dbm,
        .flrc.interframe_delay_us  = smtc_flrp_core_obj.flrc_radio_config.interframe_delay_us,
        .flrc.start_burst_delay_us = smtc_flrp_core_obj.flrc_radio_config.start_burst_delay_us,
        .flrc.start_ack_delay_us   = smtc_flrp_core_obj.flrc_radio_config.start_ack_delay_us,
        .flrc.adaptive_link_interframe_delay_us =
            smtc_flrp_core_obj.flrc_radio_config.adaptive_link_interframe_delay_us,
        .flrc.burst_target_per = smtc_flrp_core_obj.flrc_radio_config.burst_target_per,
    };
    memcpy( flrp_radio_config.flrc.channels_freq_hz, smtc_flrp_core_obj.flrc_radio_config.frequencies_hz,
            SMTC_FLRP_NB_CHANNELS_MAX );

    return flrp_radio_config;
}

smtc_flrp_return_code_t smtc_flrp_set_new_radio_config( smtc_flrp_radio_config_t radio_config )
{
    SANITY_CHECK_FLRC_API_INITIALIZED( smtc_flrp_core_obj );

    if( smtc_flrp_core_obj.state != SMTC_FLRP_CORE_STATE_IDLE )
    {
        return SMTC_FLRP_RC_BUSY;
    }

    if( ( radio_config.flrc.nb_channels > SMTC_FLRP_NB_CHANNELS_MAX ) ||
        ( radio_config.flrc.default_channel > SMTC_FLRP_NB_CHANNELS_MAX ) ||
        ( radio_config.flrc.interframe_delay_us > SMTC_FLRP_DELAYS_MAX ) ||
        ( radio_config.flrc.adaptive_link_interframe_delay_us > SMTC_FLRP_DELAYS_MAX ) ||
        ( radio_config.flrc.start_burst_delay_us > SMTC_FLRP_DELAYS_MAX ) ||
        ( radio_config.flrc.start_ack_delay_us > SMTC_FLRP_DELAYS_MAX ) ||
        ( radio_config.flrc.burst_target_per > 100 ) )
    {
        return SMTC_FLRP_RC_INVALID_PARAMS;
    }

    smtc_flrp_core_obj.wor_rx_radio_config = radio_config.wor_rx;
    smtc_flrp_core_obj.wor_tx_radio_config = radio_config.wor_tx;
    memcpy( smtc_flrp_core_obj.flrc_radio_config.frequencies_hz, radio_config.flrc.channels_freq_hz,
            SMTC_FLRP_NB_CHANNELS_MAX );
    smtc_flrp_core_obj.flrc_radio_config.nb_frequencies       = radio_config.flrc.nb_channels;
    smtc_flrp_core_obj.flrc_radio_config.default_channel      = radio_config.flrc.default_channel;
    smtc_flrp_core_obj.flrc_radio_config.raw_bit_rate         = radio_config.flrc.raw_bit_rate;
    smtc_flrp_core_obj.flrc_radio_config.tx_power_in_dbm      = radio_config.flrc.tx_power_in_dbm;
    smtc_flrp_core_obj.flrc_radio_config.interframe_delay_us  = radio_config.flrc.interframe_delay_us;
    smtc_flrp_core_obj.flrc_radio_config.start_burst_delay_us = radio_config.flrc.start_burst_delay_us;
    smtc_flrp_core_obj.flrc_radio_config.start_ack_delay_us   = radio_config.flrc.start_ack_delay_us;
    smtc_flrp_core_obj.flrc_radio_config.adaptive_link_interframe_delay_us =
        radio_config.flrc.adaptive_link_interframe_delay_us;
    smtc_flrp_core_obj.flrc_radio_config.burst_target_per = radio_config.flrc.burst_target_per;

    return SMTC_FLRP_RC_OK;
}

smtc_flrp_flrc_advanced_radio_config_t smtc_flrp_get_current_advanced_flrc_radio_config( void )
{
    smtc_flrp_flrc_advanced_radio_config_t flrc_advanced_radio_config = {
        .cr                 = smtc_flrp_core_obj.flrc_radio_config.cr,
        .pulse_shape        = smtc_flrp_core_obj.flrc_radio_config.pulse_shape,
        .preambule_len      = smtc_flrp_core_obj.flrc_radio_config.preambule_len,
        .sync_word_len      = smtc_flrp_core_obj.flrc_radio_config.sync_word_len,
        .tx_syncword_index  = smtc_flrp_core_obj.flrc_radio_config.tx_syncword_index,
        .rx_match_sync_word = smtc_flrp_core_obj.flrc_radio_config.rx_match_sync_word,
        .pld_is_fix         = smtc_flrp_core_obj.flrc_radio_config.pld_is_fix,
        .crc_type           = smtc_flrp_core_obj.flrc_radio_config.crc_type,
        .crc_seed           = smtc_flrp_core_obj.flrc_radio_config.crc_seed,
        .crc_polynomial     = smtc_flrp_core_obj.flrc_radio_config.crc_polynomial,
    };

    memcpy( flrc_advanced_radio_config.sync_word, smtc_flrp_core_obj.flrc_radio_config.sync_word,
            3 * SMTC_FLRP_SIZE_SYNC_WORD_MAX );

    return flrc_advanced_radio_config;
}

smtc_flrp_return_code_t smtc_flrp_set_new_advanced_flrc_radio_config(
    smtc_flrp_flrc_advanced_radio_config_t radio_config )
{
    SANITY_CHECK_FLRC_API_INITIALIZED( smtc_flrp_core_obj );

    if( smtc_flrp_core_obj.state != SMTC_FLRP_CORE_STATE_IDLE )
    {
        return SMTC_FLRP_RC_BUSY;
    }

    smtc_flrp_core_obj.flrc_radio_config.cr                 = radio_config.cr;
    smtc_flrp_core_obj.flrc_radio_config.pulse_shape        = radio_config.pulse_shape;
    smtc_flrp_core_obj.flrc_radio_config.preambule_len      = radio_config.preambule_len;
    smtc_flrp_core_obj.flrc_radio_config.sync_word_len      = radio_config.sync_word_len;
    smtc_flrp_core_obj.flrc_radio_config.tx_syncword_index  = radio_config.tx_syncword_index;
    smtc_flrp_core_obj.flrc_radio_config.rx_match_sync_word = radio_config.rx_match_sync_word;
    smtc_flrp_core_obj.flrc_radio_config.pld_is_fix         = radio_config.pld_is_fix;
    smtc_flrp_core_obj.flrc_radio_config.crc_type           = radio_config.crc_type;
    smtc_flrp_core_obj.flrc_radio_config.crc_seed           = radio_config.crc_seed;
    smtc_flrp_core_obj.flrc_radio_config.crc_polynomial     = radio_config.crc_polynomial;
    memcpy( smtc_flrp_core_obj.flrc_radio_config.sync_word, radio_config.sync_word, 3 * SMTC_FLRP_SIZE_SYNC_WORD_MAX );

    return SMTC_FLRP_RC_OK;
}

void smtc_flrp_run_engine( void )
{
    smtc_flrp_core_state_t current_state = smtc_flrp_core_obj.state;

    switch( current_state )
    {
    case SMTC_FLRP_CORE_STATE_IDLE:
        if( is_periodic_cad_enabled( &smtc_flrp_core_obj ) )
        {
            smtc_flrp_core_obj.state = SMTC_FLRP_CORE_STATE_WOR_RX;
            smtc_flrp_restart_reception( &smtc_flrp_core_obj, smtc_flrp_core_obj.radio_end_timestamp_ms );
        }
        break;

    case SMTC_FLRP_CORE_STATE_WOR_RX:
        if( smtc_flrp_core_obj.wor_event == SMTC_FLRP_CORE_WOR_SUCCESS )
        {
            smtc_flrp_core_start_mac( &smtc_flrp_core_obj, false );
            smtc_flrp_core_obj.state = SMTC_FLRP_CORE_STATE_DATA;
        }
        else if( ( smtc_flrp_core_obj.wor_event == SMTC_FLRP_CORE_WOR_FAILED ) ||
                 ( smtc_flrp_core_obj.wor_event == SMTC_FLRP_CORE_WOR_RX_ABORT ) )
        {
            smtc_flrp_core_obj.wor_exchange_phase_bits = 0;
            smtc_flrp_core_obj.state                   = SMTC_FLRP_CORE_STATE_IDLE;
        }
        smtc_flrp_core_obj.wor_event = SMTC_FLRP_CORE_WOR_IDLE;
        break;
    case SMTC_FLRP_CORE_STATE_WOR_TX:

        if( smtc_flrp_core_obj.wor_event == SMTC_FLRP_CORE_WOR_SUCCESS )
        {
            smtc_flrp_core_start_mac( &smtc_flrp_core_obj, true );
            smtc_flrp_core_obj.state = SMTC_FLRP_CORE_STATE_DATA;
        }
        else if( smtc_flrp_core_obj.wor_event == SMTC_FLRP_CORE_WOR_FAILED )
        {
            smtc_flrp_core_obj.wor_exchange_phase_bits = 0;
            smtc_flrp_core_obj.state                   = SMTC_FLRP_CORE_STATE_IDLE;
        }
        smtc_flrp_core_obj.wor_event = SMTC_FLRP_CORE_WOR_IDLE;
        break;

    case SMTC_FLRP_CORE_STATE_DATA:
        smtc_flrp_mac_layer_process( &smtc_flrp_core_obj.smtc_flrp_mac_layer_obj );
        break;
    }
}

bool smtc_flrp_call_run( void )
{
    bool ret = false;

    switch( smtc_flrp_core_obj.state )
    {
    case SMTC_FLRP_CORE_STATE_IDLE:
        if( is_periodic_cad_enabled( &smtc_flrp_core_obj ) == true )
        {
            ret = true;
        }
        break;
    case SMTC_FLRP_CORE_STATE_WOR_TX:
    case SMTC_FLRP_CORE_STATE_WOR_RX:
        ret = false;
        break;
    case SMTC_FLRP_CORE_STATE_DATA:
        ret = true;
        break;
    default:
        break;
    }

    return ret;
}

uint8_t smtc_flrp_core_get_mac_radio_access_id( void )
{
    return smtc_flrp_core_obj.smtc_flrp_mac_layer_obj.radio_access_id;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION ---------------------------------------------
 */
static bool is_periodic_cad_enabled( smtc_flrp_core_t* smtc_flrp_core_obj )
{
    return ( smtc_flrp_core_obj->cad_period_ms > 0 );
}

static smtc_flrp_return_code_t convert_wor_status_to_flrc_error_code( smtc_wor_status_t wor_status )
{
    switch( wor_status )
    {
    case SMTC_WOR_STATUS_SUCCESS:
    case SMTC_WOR_STATUS_FAILED:
        return SMTC_FLRP_RC_OK;

    case SMTC_WOR_STATUS_BUSY:
        return SMTC_FLRP_RC_BUSY;

    case SMTC_WOR_STATUS_INVALID_PARAMETER:
        return SMTC_FLRP_RC_INVALID_PARAMS;

    case SMTC_WOR_STATUS_NOT_INITIALIZED:
        return SMTC_FLRP_RC_NOT_INIT;

    case SMTC_WOR_STATUS_NOT_SUPPORTED:
        return SMTC_FLRP_RC_UNSUPPORTED_FEATURE;

    case SMTC_WOR_STATUS_ERROR:
    default:
        return SMTC_FLRP_RC_ERROR;
    }
}

static smtc_flrp_return_code_t convert_mac_status_to_flrc_error_code( smtc_flrp_mac_layer_status_t mac_status )
{
    smtc_flrp_return_code_t flrc_rc = SMTC_FLRP_RC_ERROR;
    switch( mac_status )
    {
    case SMTC_FLRP_MAC_LAYER_STATUS_OK:
    case SMTC_FLRP_MAC_LAYER_STATUS_TIMEOUT:
    case SMTC_FLRP_MAC_LAYER_STATUS_REFUSED:
        flrc_rc = SMTC_FLRP_RC_OK;
        break;
    case SMTC_FLRP_MAC_LAYER_STATUS_BUSY:
        flrc_rc = SMTC_FLRP_RC_BUSY;
        break;
    case SMTC_FLRP_MAC_LAYER_STATUS_INVALID_PARAMS:
        return SMTC_FLRP_RC_INVALID_PARAMS;
    case SMTC_FLRP_MAC_LAYER_STATUS_ERROR_MEMORY:
    case SMTC_FLRP_MAC_LAYER_STATUS_ERROR:
    default:
        flrc_rc = SMTC_FLRP_RC_ERROR;
        break;
    }
    return flrc_rc;
}

static void smtc_flrp_init_and_get_config( smtc_flrp_mac_radio_config_t* flrc_radio_config,
                                           smtc_flrp_wor_radio_config_t* wor_rx_radio_config,
                                           smtc_flrp_wor_radio_config_t* wor_tx_radio_config,
                                           smtc_flrp_frequency_plan_t    freq_plan )
{
    flrc_radio_config->nb_frequencies = DATA_FLRC_NB_FREQ;
    uint32_t freq_ref =
        ( freq_plan == SMTC_FLRP_FREQ_2GHz4 ) ? DATA_FLRC_2GHz4_RF_FREQ_IN_HZ : DATA_FLRC_865MHz_RF_FREQ_IN_HZ;
    uint32_t channel_step =
        ( freq_plan == SMTC_FLRP_FREQ_2GHz4 ) ? DATA_FLRC_CHANNEL_STEP_2GHz4_HZ : DATA_FLRC_CHANNEL_STEP_865MHz_HZ;
    for( uint8_t i = 0; i < DATA_FLRC_NB_FREQ; i++ )
    {
        flrc_radio_config->frequencies_hz[i] = freq_ref + channel_step * i;
    }
    flrc_radio_config->default_channel                   = 0;
    flrc_radio_config->tx_power_in_dbm                   = TX_OUTPUT_POWER_DBM;
    flrc_radio_config->raw_bit_rate                      = DATA_FLRC_RAW_BIT_RATE;
    flrc_radio_config->cr                                = DATA_FLRC_CR;
    flrc_radio_config->pulse_shape                       = DATA_FLRC_PULSE_SHAPE;
    flrc_radio_config->preambule_len                     = DATA_FLRC_PREAMBLE_BITS;
    flrc_radio_config->sync_word_len                     = DATA_FLRC_SYNCWORD_LEN;
    flrc_radio_config->tx_syncword_index                 = DATA_FLRC_SYNCWORD;
    flrc_radio_config->rx_match_sync_word                = DATA_FLRC_MATCH_SYNCWORD;
    flrc_radio_config->pld_is_fix                        = DATA_FLRC_PLD_IS_FIX;
    flrc_radio_config->crc_type                          = DATA_FLRC_CRC;
    flrc_radio_config->interframe_delay_us               = DATA_FLRC_MIN_INTERFRAME_DURATION_US;
    flrc_radio_config->start_ack_delay_us                = SMTC_FLRP_MAC_DELAY_START_BURST_ACK_MS * 1000;
    flrc_radio_config->start_burst_delay_us              = SMTC_FLRP_MAC_DELAY_START_BURST_MS * 1000;
    flrc_radio_config->adaptive_link_interframe_delay_us = SMTC_FLRP_MAC_DELAY_ADAPTIVE_LINK_INTERFRAME_MS * 1000;
    flrc_radio_config->burst_target_per                  = DATA_FLRC_BURST_TARGET_PER_PERCENTAGE;
    flrc_radio_config->crc_seed                          = DATA_FLRC_CRC_SEED;
    flrc_radio_config->crc_polynomial                    = DATA_FLRC_CRC_POLYNOMIAL;
    memcpy( flrc_radio_config->sync_word[0], &flrc_default_syncword_1[0], SMTC_FLRP_SIZE_SYNC_WORD_MAX );
    memcpy( flrc_radio_config->sync_word[1], &flrc_default_syncword_2[0], SMTC_FLRP_SIZE_SYNC_WORD_MAX );
    memcpy( flrc_radio_config->sync_word[2], &flrc_default_syncword_3[0], SMTC_FLRP_SIZE_SYNC_WORD_MAX );

    wor_rx_radio_config->frequency_hz =
        ( freq_plan == SMTC_FLRP_FREQ_2GHz4 ) ? WOR_2GHz4_RF_FREQ_IN_HZ : WOR_865MHz_RF_FREQ_IN_HZ;
    wor_rx_radio_config->sf              = WOR_LORA_SPREADING_FACTOR;
    wor_rx_radio_config->bw              = WOR_LORA_BANDWIDTH;
    wor_rx_radio_config->tx_power_in_dbm = TX_OUTPUT_POWER_DBM;

    memcpy( wor_tx_radio_config, wor_rx_radio_config, sizeof( smtc_flrp_wor_radio_config_t ) );
}

static void smtc_flrp_core_start_mac( smtc_flrp_core_t* smtc_flrp_core, bool device_is_initiator )
{
    smtc_flrp_mac_layer_status_t mac_status = SMTC_FLRP_MAC_LAYER_STATUS_OK;
    smtc_wor_t* wor_obj = device_is_initiator ? &smtc_flrp_core->smtc_wor_tx_obj : &smtc_flrp_core->smtc_wor_rx_obj;
    uint8_t*    dest_dev_eui = device_is_initiator ? smtc_flrp_core->smtc_wor_tx_obj.wor_data_trx.u.flrc.slave_dev_eui
                                                   : smtc_flrp_core->smtc_wor_rx_obj.wor_data_trx.u.flrc.initiator_dev_eui;
    uint32_t    timestamp_start_next_phase_ms = smtc_flrp_core_obj.radio_end_timestamp_ms;

    smtc_flrp_mac_config_t mac_cfg_wor = {
        .dest_dev_eui         = dest_dev_eui,
        .filter_len           = wor_obj->wor_data_trx.u.flrc.filter_len,
        .enabled_channels     = wor_obj->wor_data_trx.u.flrc.enabled_channels,
        .link_adaptation_mode = wor_obj->wor_data_trx.u.flrc.link_adaptation_mode,
        .burst_ack_enabled    = wor_obj->wor_data_trx.u.flrc.burst_ack_required,
        .freq_offset_hz       = wor_obj->wor_rx_measurements.freq_offset_hz,
    };

    mac_cfg_wor.radio_config                 = smtc_flrp_core->flrc_radio_config;
    mac_cfg_wor.radio_config.default_channel = wor_obj->wor_data_trx.u.flrc.default_channel;
    if( wor_obj->wor_data_trx.u.flrc.wor_ack_required )
    {
        mac_cfg_wor.radio_config.raw_bit_rate = wor_obj->wor_ack_data_trx.u.flrc.selected_flrc_dr;
        mac_cfg_wor.radio_config.cr           = wor_obj->wor_ack_data_trx.u.flrc.selected_coding_rate;
        timestamp_start_next_phase_ms += ( wor_obj->wor_ack_data_trx.u.flrc.next_phase_start_delay_us / 1000 );
    }
    else
    {
        mac_cfg_wor.radio_config.raw_bit_rate = wor_obj->wor_data_trx.u.flrc.default_flrc_dr;
        mac_cfg_wor.radio_config.cr           = wor_obj->wor_data_trx.u.flrc.default_coding_rate;
        timestamp_start_next_phase_ms += ( wor_obj->wor_data_trx.u.flrc.next_phase_start_delay_us / 1000 );
    }
    mac_cfg_wor.radio_config.adaptive_link_interframe_delay_us =
        ( wor_obj->wor_data_trx.u.flrc.wor_ack_required && wor_obj->wor_ack_data_trx.u.flrc.has_min_interframe_delay )
            ? wor_obj->wor_ack_data_trx.u.flrc.min_interframe_delay_us
            : wor_obj->wor_data_trx.u.flrc.channel_interframe_delay_us;

    bool is_tx =
        ( device_is_initiator && smtc_flrp_core_obj.smtc_wor_tx_obj.wor_data_trx.u.flrc.initiator_send_burst ) ||
        ( !device_is_initiator && !smtc_flrp_core_obj.smtc_wor_rx_obj.wor_data_trx.u.flrc.initiator_send_burst );
    if( is_tx )
    {
        mac_status = smtc_flrp_mac_layer_start_tx_transaction(
            &smtc_flrp_core->smtc_flrp_mac_layer_obj, timestamp_start_next_phase_ms,
            device_is_initiator ? smtc_flrp_core->tx_initiator_data_buffer : smtc_flrp_core->tx_slave_data_buffer,
            device_is_initiator ? smtc_flrp_core->tx_initiator_data_buffer_size
                                : smtc_flrp_core->tx_slave_data_buffer_size,
            ( const smtc_flrp_mac_config_t* ) &mac_cfg_wor );
    }
    else
    {
        mac_status = smtc_flrp_mac_layer_start_rx_transaction(
            &smtc_flrp_core->smtc_flrp_mac_layer_obj, timestamp_start_next_phase_ms,
            device_is_initiator ? smtc_flrp_core->rx_initiator_data_buffer : smtc_flrp_core->rx_slave_data_buffer,
            device_is_initiator ? smtc_flrp_core->rx_initiator_data_buffer_size
                                : smtc_flrp_core->rx_slave_data_buffer_size,
            ( const smtc_flrp_mac_config_t* ) &mac_cfg_wor );
    }

    if( mac_status != SMTC_FLRP_MAC_LAYER_STATUS_OK )
    {
        smtc_flrp_core_return_error( smtc_flrp_core, convert_mac_status_to_flrc_error_code( mac_status ), dest_dev_eui,
                                     is_tx );
    }
}

static void smtc_flrp_configure_wor_data( smtc_flrp_core_t* smtc_flrp_core, smtc_wor_data_t* wor_data,
                                          smtc_flrp_com_config_t com_config )
{
    smtc_flrp_mac_radio_config_t flrc_radio_config = smtc_flrp_core->flrc_radio_config;

    // Init tx wor data
    wor_data->type                        = SMTC_WOR_TYPE_FLRC;
    wor_data->u.flrc.link_adaptation_mode = com_config.link_adaptation_mode;
    memcpy( wor_data->u.flrc.initiator_dev_eui, smtc_flrp_core->dev_eui, SMTC_FLRP_EUI_LENGTH );

    wor_data->u.flrc.wor_ack_required    = ( com_config.com_mode != SMTC_FLRP_COM_ONE_WAY );
    wor_data->u.flrc.burst_ack_required  = ( com_config.com_mode == SMTC_FLRP_BIDIRECTIONAL );
    wor_data->u.flrc.default_flrc_dr     = flrc_radio_config.raw_bit_rate;
    wor_data->u.flrc.default_channel     = flrc_radio_config.default_channel;
    wor_data->u.flrc.default_coding_rate = flrc_radio_config.cr;
    memcpy( wor_data->u.flrc.slave_dev_eui, com_config.slave_dev_eui, SMTC_FLRP_EUI_LENGTH );
    if( com_config.com_mode != SMTC_FLRP_COM_ONE_WAY )
    {
        wor_data->u.flrc.filter_len = 0x3F;
    }
    else
    {
        wor_data->u.flrc.filter_len = com_config.filter_len;
    }

    if( wor_data->u.flrc.wor_ack_required )
    {
        wor_data->u.flrc.next_phase_start_delay_us = WOR_ACK_DELAY_MS * 1000;
    }
    else if( wor_data->u.flrc.link_adaptation_mode != SMTC_FLRP_LINK_ADAPTATION_DISABLED )
    {
        wor_data->u.flrc.next_phase_start_delay_us = flrc_radio_config.adaptive_link_interframe_delay_us;
    }
    else
    {
        wor_data->u.flrc.next_phase_start_delay_us = flrc_radio_config.start_burst_delay_us;
    }

    wor_data->u.flrc.enabled_channels            = 0x7;
    wor_data->u.flrc.channel_interframe_delay_us = flrc_radio_config.adaptive_link_interframe_delay_us;

    for( uint16_t i = 0; i < SMTC_WOR_PAYLOAD_SIGN_SIZE; i++ )
    {
        wor_data->signature[i] = i;
    }
}

static void smtc_flrp_configure_wor_ack_data( smtc_wor_ack_data_t* wor_ack_data )
{
    smtc_flrp_radio_config_t radio_config        = smtc_flrp_get_current_radio_config( );
    wor_ack_data->u.flrc.min_interframe_delay_us = radio_config.flrc.adaptive_link_interframe_delay_us;
    wor_ack_data->type                           = SMTC_WOR_TYPE_FLRC;
}

static smtc_wor_status_t smtc_flrp_start_wor_rx( smtc_flrp_core_t* smtc_flrp_core_obj, uint32_t timestamp_start_rx_ms )
{
    smtc_wor_ack_data_t wor_ack = {
        .type = SMTC_WOR_TYPE_UNKNOWN,
    };
    smtc_wor_rx_info_for_ack_t wor_ack_info;

    smtc_flrp_configure_wor_ack_data( &wor_ack );

    wor_ack_info.u.flrc.delay_until_flrc_req_us =
        smtc_flrp_core_obj->flrc_radio_config.adaptive_link_interframe_delay_us;
    wor_ack_info.u.flrc.delay_until_burst_us = smtc_flrp_core_obj->flrc_radio_config.start_burst_delay_us;

    return smtc_wor_rx_start( &smtc_flrp_core_obj->smtc_wor_rx_obj, smtc_flrp_core_obj->wor_rx_radio_config,
                              timestamp_start_rx_ms, smtc_flrp_core_obj->data_slave_ready_to_be_sent, wor_ack,
                              wor_ack_info );
}

static void smtc_flrp_restart_reception( smtc_flrp_core_t* smtc_flrp_core_obj, uint32_t radio_end_timestamp_ms )
{
    smtc_flrp_core_obj->wor_exchange_phase_bits = 0;

    uint32_t timestamp_start_rx_ms = 0;
    uint32_t current_time_ms       = smtc_modem_hal_get_time_in_ms( );
    if( radio_end_timestamp_ms + smtc_flrp_core_obj->cad_period_ms <= current_time_ms )
    {
        timestamp_start_rx_ms = current_time_ms + 10;
    }
    else
    {
        timestamp_start_rx_ms = radio_end_timestamp_ms + smtc_flrp_core_obj->cad_period_ms;
    }

    // Restart reception
    smtc_wor_status_t wor_status = smtc_flrp_start_wor_rx( smtc_flrp_core_obj, timestamp_start_rx_ms );
    if( wor_status != SMTC_WOR_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "SMTC FLRC CORE : Failed to start WOR reception (status %u)\n", wor_status );
        smtc_flrp_rx_stats_t stats = { 0 };
        if( smtc_flrp_core_obj->rx_user_callback != NULL )
        {
            smtc_flrp_core_obj->rx_user_callback( smtc_flrp_core_obj->user_context,
                                                  convert_wor_status_to_flrc_error_code( wor_status ), 0, NULL, stats );
        }
    }
}

static void smtc_flrp_core_return_error( smtc_flrp_core_t* smtc_flrp_core, smtc_flrp_return_code_t status,
                                         uint8_t* dest_addr, bool is_tx )
{
    if( is_tx )
    {
        if( smtc_flrp_core->tx_user_callback != NULL )
        {
            smtc_flrp_core->tx_user_callback( smtc_flrp_core->user_context, status, false, dest_addr );
        }
    }
    else
    {
        if( smtc_flrp_core->rx_user_callback != NULL )
        {
            smtc_flrp_rx_stats_t stats = { 0 };
            stats.wor                  = smtc_flrp_core->wor_rx_stats;
            smtc_flrp_core->rx_user_callback( smtc_flrp_core->user_context, status, 0, dest_addr, stats );
        }
    }
}

static void smtc_flrp_core_wor_tx_done_cb( smtc_wor_status_t status, smtc_flrp_wor_rx_stats_t rx_metrics )
{
    if( smtc_flrp_core_obj.state != SMTC_FLRP_CORE_STATE_WOR_TX )
    {
        /* silent: callback ignored, not in WOR_TX state (transient case) */
        return;
    }

    smtc_flrp_core_obj.radio_end_timestamp_ms =
        smtc_flrp_core_obj.smtc_wor_tx_obj.transaction->smtc_rac_data_result.radio_end_timestamp_ms;
    smtc_flrp_core_obj.wor_rx_stats = rx_metrics;

    if( status == SMTC_WOR_STATUS_SUCCESS )
    {
        smtc_flrp_core_obj.wor_event = SMTC_FLRP_CORE_WOR_SUCCESS;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "[FLRP] WOR TX failed (no ACK?) wor_status=%u\n", ( unsigned ) status );
        smtc_flrp_core_return_error( &smtc_flrp_core_obj, convert_wor_status_to_flrc_error_code( status ),
                                     smtc_flrp_core_obj.smtc_wor_tx_obj.wor_data_trx.u.flrc.slave_dev_eui,
                                     smtc_flrp_core_obj.smtc_wor_tx_obj.wor_data_trx.u.flrc.initiator_send_burst );
        smtc_flrp_core_obj.wor_event = SMTC_FLRP_CORE_WOR_FAILED;
    }
    smtc_modem_hal_user_lbm_irq( );

    // SMTC_MODEM_HAL_TRACE_INFO( "smtc_flrp_core_wor_done_cb %u\n", is_successful );
}

static void smtc_flrp_core_wor_rx_done_cb( smtc_wor_status_t status, smtc_flrp_wor_rx_stats_t rx_metrics )
{
    if( smtc_flrp_core_obj.state != SMTC_FLRP_CORE_STATE_WOR_RX )
    {
        /* silent: callback ignored, not in WOR_RX state (transient case) */
        return;
    }

    if( status != SMTC_WOR_STATUS_ABORT )
    {
        smtc_flrp_core_obj.radio_end_timestamp_ms =
            smtc_flrp_core_obj.smtc_wor_rx_obj.transaction->smtc_rac_data_result.radio_end_timestamp_ms;
        smtc_flrp_core_obj.wor_rx_stats = rx_metrics;

        if( status == SMTC_WOR_STATUS_SUCCESS )
        {
            smtc_flrp_core_obj.wor_event = SMTC_FLRP_CORE_WOR_SUCCESS;
        }
        else
        {
            /* silent on plain FAILED (= periodic RX timeout). Only log hard errors below. */
            smtc_flrp_core_obj.wor_event = SMTC_FLRP_CORE_WOR_FAILED;

            if( ( status == SMTC_WOR_STATUS_INVALID_PARAMETER ) || ( status == SMTC_WOR_STATUS_NOT_SUPPORTED ) ||
                ( status == SMTC_WOR_STATUS_NOT_INITIALIZED ) )
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "[FLRP] WOR RX hard error wor_status=%u\n", ( unsigned ) status );
                smtc_flrp_core_return_error(
                    &smtc_flrp_core_obj, convert_wor_status_to_flrc_error_code( status ), NULL,
                    !smtc_flrp_core_obj.smtc_wor_rx_obj.wor_data_trx.u.flrc.initiator_send_burst );
            }
        }
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "[FLRP] WOR RX ABORTED\n" );
        smtc_flrp_core_obj.wor_event = SMTC_FLRP_CORE_WOR_RX_ABORT;
    }
    smtc_modem_hal_user_lbm_irq( );
}

static void smtc_flrp_mac_tx_done_cb( smtc_flrp_mac_layer_status_t status, uint32_t timestamp_ms, uint8_t* dest_addr )
{
    if( smtc_flrp_core_obj.state != SMTC_FLRP_CORE_STATE_DATA )
    {
        return;
    }

    smtc_flrp_core_obj.state = SMTC_FLRP_CORE_STATE_IDLE;

    smtc_flrp_core_obj.radio_end_timestamp_ms =
        smtc_flrp_core_obj.smtc_flrp_mac_layer_obj.transaction->smtc_rac_data_result.radio_end_timestamp_ms;

    if( smtc_flrp_core_obj.tx_user_callback != NULL )
    {
        smtc_flrp_core_obj.tx_user_callback( smtc_flrp_core_obj.user_context,
                                             convert_mac_status_to_flrc_error_code( status ),
                                             ( status == SMTC_FLRP_MAC_LAYER_STATUS_OK ), dest_addr );
    }
}

static void smtc_flrp_mac_rx_done_cb( smtc_flrp_mac_layer_status_t status, uint32_t data_size, uint32_t timestamp_ms,
                                      uint8_t* dest_addr )
{
    if( smtc_flrp_core_obj.state != SMTC_FLRP_CORE_STATE_DATA )
    {
        return;
    }

    smtc_flrp_core_obj.state = SMTC_FLRP_CORE_STATE_IDLE;

    smtc_flrp_core_obj.radio_end_timestamp_ms =
        smtc_flrp_core_obj.smtc_flrp_mac_layer_obj.transaction->smtc_rac_data_result.radio_end_timestamp_ms;
    if( smtc_flrp_core_obj.rx_user_callback != NULL )
    {
        smtc_flrp_rx_stats_t stats = { 0 };
        stats.wor                  = smtc_flrp_core_obj.wor_rx_stats;
        stats.burst                = smtc_flrp_core_obj.smtc_flrp_mac_layer_obj.payload_stats;
        smtc_flrp_core_obj.rx_user_callback( smtc_flrp_core_obj.user_context,
                                             convert_mac_status_to_flrc_error_code( status ), data_size, dest_addr,
                                             stats );
    }
}

/* --- EOF ------------------------------------------------------------------ */
