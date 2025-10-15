/**
 * @file      app_notification.c
 *
 * @brief     RF Certification LoRa TX example for LR1110 or LR1120 chip
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

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_notification.h"
#include "app_rf_certification.h"
#include "smtc_rac_api.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "RF-CERT-NOTIF"
#include "smtc_rac_log.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static const smtc_rac_rttof_params_t UNUSED_RTTOF = {
    .request_address        = 0,
    .delay_indicator        = 0,
    .response_symbols_count = 0,
    .bw_ranging             = LORA_BANDWIDTH,
};

static const smtc_rac_lbt_context_t UNUSED_LBT = {
    .lbt_enabled        = false,
    .listen_duration_ms = 0,
    .threshold_dbm      = 0,
    .bandwidth_hz       = 0,
    .rssi_inst_dbm      = 0,
    .channel_busy       = false,
};

static const smtc_rac_cw_context_t UNUSED_CW = {
    .cw_enabled        = false,
    .infinite_preamble = false,
};

static const uint32_t PROCESSING_TIME = 10;  // ms

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct rf_notification_s
{
    uint8_t             radio_access_id;          // store result of `smtc_rac_open_radio`
    smtc_rac_context_t* notification_context;     // store result of `smtc_rac_get_context`
    smtc_rac_context_t* certification_context;    // values sent in the notification
    void ( *on_notification_sent )( void );       // called after a notification has been sent
    void ( *on_notification_received )( char* );  // called after a notification has been received
    rf_certification_region_t region;             // region to use
} rf_notification_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static char notification_string[NOTIFICATION_STRING_MAX_LEN + 1] = { 0 };

static char half_dbm_string[6] = { 0 };

static rf_notification_t rf_notification = {
    .radio_access_id          = RAC_INVALID_RADIO_ID,
    .notification_context     = NULL,
    .certification_context    = NULL,
    .on_notification_sent     = NULL,
    .on_notification_received = NULL,
    #ifdef RF_CERT_REGION
    .region = RF_CERT_REGION,
#else
    .region = RF_CERT_REGION_ETSI,
#endif
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void notification_init_context( void );
static void notification_start_tx( void );
static void notification_start_rx( void );
static void notification_pre_callback( void );
static void notification_post_callback( rp_status_t status );
static void dbm_to_half_dbm_string( uint8_t dbm );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void notification_init_certification( smtc_rac_context_t* context )
{
    if( NOTIFICATIONS_ENABLED == false )
    {
        return;
    }
    notification_init_context( );
    rf_notification.certification_context = context;
}

void notification_wrap( void ( *function )( void ) )
{
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( function != NULL );
    if( NOTIFICATIONS_ENABLED == false )
    {
        function( );
    }
    else
    {
        rf_notification.on_notification_sent = function;
        notification_start_tx( );
    }
}

void notification_init_rx_only( void ( *function )( char* notification ) )
{
    if( NOTIFICATIONS_ENABLED == false )
    {
        return;
    }
    notification_init_context( );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( function != NULL );
    rf_notification.on_notification_received = function;
    notification_start_rx( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void notification_init_context( void )
{
    if( rf_notification.radio_access_id != RAC_INVALID_RADIO_ID )
    {
        SMTC_HAL_TRACE_WARNING( "'notification_init_context' has already been called\n" );
        return;
    }

    rf_notification.radio_access_id = smtc_rac_open_radio( RAC_VERY_HIGH_PRIORITY );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( rf_notification.radio_access_id != RAC_INVALID_RADIO_ID );

    rf_notification.notification_context = smtc_rac_get_context( rf_notification.radio_access_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( rf_notification.notification_context != NULL );

    rf_notification.notification_context->modulation_type         = SMTC_RAC_MODULATION_LORA;
    rf_notification.notification_context->radio_params.lora.is_tx = false;  // must be set before each transaction
    rf_notification.notification_context->radio_params.lora.is_ranging_exchange  = false;
    rf_notification.notification_context->radio_params.lora.rttof                = UNUSED_RTTOF;
    rf_notification.notification_context->radio_params.lora.sf                   = LORA_SPREADING_FACTOR;
    rf_notification.notification_context->radio_params.lora.bw                   = LORA_BANDWIDTH;
    rf_notification.notification_context->radio_params.lora.cr                   = LORA_CODING_RATE;
    rf_notification.notification_context->radio_params.lora.preamble_len_in_symb = LORA_PREAMBLE_LENGTH;
    rf_notification.notification_context->radio_params.lora.header_type          = LORA_PKT_LEN_MODE;
    rf_notification.notification_context->radio_params.lora.invert_iq_is_on      = LORA_IQ;
    rf_notification.notification_context->radio_params.lora.crc_is_on            = LORA_CRC;
    rf_notification.notification_context->radio_params.lora.sync_word            = LORA_PRIVATE_NETWORK_SYNCWORD;
    rf_notification.notification_context->radio_params.lora.rx_timeout_ms        = 0;
    rf_notification.notification_context->radio_params.lora.symb_nb_timeout      = 0;
    rf_notification.notification_context->lbt_context                            = UNUSED_LBT;
    rf_notification.notification_context->cw_context                             = UNUSED_CW;
    rf_notification.notification_context->smtc_rac_data_buffer_setup.tx_payload_buffer =
        ( uint8_t* ) notification_string;
    rf_notification.notification_context->smtc_rac_data_buffer_setup.rx_payload_buffer =
        ( uint8_t* ) notification_string;
    rf_notification.notification_context->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = NOTIFICATION_STRING_MAX_LEN;
    rf_notification.notification_context->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = NOTIFICATION_STRING_MAX_LEN;
    rf_notification.notification_context->radio_params.lora.max_rx_size         = NOTIFICATION_STRING_MAX_LEN;
    rf_notification.notification_context->scheduler_config.start_time_ms    = 0;  // must be set before each transaction
    rf_notification.notification_context->scheduler_config.duration_time_ms = 0;  // must be set before each transaction
    rf_notification.notification_context->scheduler_config.scheduling       = SMTC_RAC_ASAP_TRANSACTION;
    rf_notification.notification_context->scheduler_config.callback_pre_radio_transaction  = notification_pre_callback;
    rf_notification.notification_context->scheduler_config.callback_post_radio_transaction = notification_post_callback;

    switch( rf_notification.region )
    {
    case RF_CERT_REGION_ETSI:
        rf_notification.notification_context->radio_params.lora.frequency_in_hz = 867300000u;
        rf_notification.notification_context->radio_params.lora.tx_power_in_dbm = EXPECTED_PWR_14_DBM;
        break;
    case RF_CERT_REGION_FCC:
        rf_notification.notification_context->radio_params.lora.frequency_in_hz = 902300000u;
        rf_notification.notification_context->radio_params.lora.tx_power_in_dbm = EXPECTED_PWR_22_DBM;
        break;
    case RF_CERT_REGION_ARIB:
        rf_notification.notification_context->radio_params.lora.frequency_in_hz = 921000000u;
        rf_notification.notification_context->radio_params.lora.tx_power_in_dbm = EXPECTED_PWR_9_DBM;
        break;
    }
}

static void notification_start_tx( void )
{
    if( rf_notification.radio_access_id == RAC_INVALID_RADIO_ID )
    {
        SMTC_HAL_TRACE_ERROR( "'notification_init_context' was never called\n" );
        return;
    }

    ( void ) memset( notification_string, 0, sizeof( notification_string ) );
    int written = 0;

    switch( rf_notification.certification_context->modulation_type )
    {
    case SMTC_RAC_MODULATION_LORA:
        dbm_to_half_dbm_string( rf_notification.certification_context->radio_params.lora.tx_power_in_dbm );
        written = snprintf( notification_string, sizeof( notification_string ),
                            "LORA TX AT FREQUENCY: %" PRIu32 " HZ, BANDWIDTH: %s KHZ, SF %s, POWER: %s DBM",
                            rf_notification.certification_context->radio_params.lora.frequency_in_hz,
                            ral_lora_bw_to_str( rf_notification.certification_context->radio_params.lora.bw ),
                            ral_lora_sf_to_str( rf_notification.certification_context->radio_params.lora.sf ),
                            half_dbm_string );
        break;

    case SMTC_RAC_MODULATION_FSK:
        dbm_to_half_dbm_string( rf_notification.certification_context->radio_params.fsk.tx_power_in_dbm );
        written = snprintf( notification_string, sizeof( notification_string ),
                            "FSK TX AT FREQUENCY: %" PRIu32 " HZ, BANDWIDTH: %" PRIu32 " KHZ, BITRATE: %" PRIu32
                            " BPS, POWER: %s DBM",
                            rf_notification.certification_context->radio_params.fsk.frequency_in_hz,
                            rf_notification.certification_context->radio_params.fsk.bw_dsb_in_hz,
                            rf_notification.certification_context->radio_params.fsk.br_in_bps, half_dbm_string );

        break;

    case SMTC_RAC_MODULATION_LRFHSS:
        dbm_to_half_dbm_string( rf_notification.certification_context->radio_params.lrfhss.tx_power_in_dbm );
        written =
            snprintf( notification_string, sizeof( notification_string ),
                      "LR-FHSS TX AT FREQUENCY: %" PRIu32 " HZ, BITRATE 488 BPS, POWER: %s DBM",
                      rf_notification.certification_context->radio_params.lrfhss.frequency_in_hz, half_dbm_string );
        break;

    case SMTC_RAC_MODULATION_FLRC:
        dbm_to_half_dbm_string( rf_notification.certification_context->radio_params.flrc.tx_power_in_dbm );
        written = snprintf( notification_string, sizeof( notification_string ),
                            "FLRC TX AT FREQUENCY: %" PRIu32 " HZ, BANDWIDTH: %" PRIu32 ", POWER: %s DBM",
                            rf_notification.certification_context->radio_params.flrc.frequency_in_hz,
                            rf_notification.certification_context->radio_params.flrc.bw_dsb_in_hz, half_dbm_string );
        break;

    default:
        break;
    }

    SMTC_HAL_TRACE_INFO( "%s\n", notification_string );

    rf_notification.notification_context->radio_params.lora.is_tx = true;
    rf_notification.notification_context->radio_params.lora.tx_size   = ( written < 256 ) ? ( ( uint8_t ) written ) : 255;
    rf_notification.notification_context->scheduler_config.start_time_ms =
        smtc_modem_hal_get_time_in_ms( ) + PROCESSING_TIME;

    smtc_rac_return_code_t error_code = smtc_rac_submit_radio_transaction( rf_notification.radio_access_id );
    if( error_code != SMTC_RAC_SUCCESS )
    {
        SMTC_HAL_TRACE_ERROR( "call to 'smtc_rac_submit_radio_transaction' failed\n" );
    }
}

static void notification_start_rx( void )
{
    rf_notification.notification_context->radio_params.lora.is_tx = false;
    rf_notification.notification_context->scheduler_config.start_time_ms =
        smtc_modem_hal_get_time_in_ms( ) + PROCESSING_TIME;

    smtc_rac_return_code_t error_code = smtc_rac_submit_radio_transaction( rf_notification.radio_access_id );
    if( error_code != SMTC_RAC_SUCCESS )
    {
        SMTC_HAL_TRACE_ERROR( "call to 'smtc_rac_submit_radio_transaction' failed\n" );
    }
}

static void notification_pre_callback( void )
{
    // does nothing
}

static void notification_post_callback( rp_status_t status )
{
    if( ( RX_ONLY == true ) && ( status == RP_STATUS_RX_PACKET ) )
    {
        notification_string[rf_notification.notification_context->smtc_rac_data_result.rx_size] = '\0';
        ( rf_notification.on_notification_received )( notification_string );
        notification_start_rx( );
        return;
    }

    if( ( RX_ONLY == false ) && ( status == RP_STATUS_TX_DONE ) )
    {
        ( rf_notification.on_notification_sent )( );
        return;
    }

    SMTC_HAL_TRACE_WARNING( "Invalid notification callback status: %d\n", ( ( int ) status ) );
}

static void dbm_to_half_dbm_string( uint8_t dbm )
{
    int half_dbm = ( int ) ( dbm / 2 );
    int decimal  = ( dbm % 2 == 0 ) ? 0 : 5;
    ( void ) snprintf( half_dbm_string, sizeof( half_dbm_string ), "%d.%d", half_dbm, decimal );
}

/* --- EOF ------------------------------------------------------------------ */
