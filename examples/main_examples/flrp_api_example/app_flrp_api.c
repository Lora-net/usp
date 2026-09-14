/*!
 * \file      app_flrp_api.c
 *
 * \brief     application program for FLRP API example
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdio.h>
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <string.h>

#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "APP-FLRP-API"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"
#include "smtc_rac_log.h"

#include "smtc_rac_api.h"
#include "smtc_flrp_api.h"
#include "smtc_flrp_crypto.h"
#include "app_flrp_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#if ( ( FLRP_API_ROLE != FLRP_API_ROLE_INITIATOR ) && ( FLRP_API_ROLE != FLRP_API_ROLE_SLAVE ) )
#error "Please define FLRP_API_ROLE as either FLRP_API_ROLE_INITIATOR or FLRP_API_ROLE_SLAVE"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static uint16_t           flrp_api_tx_counter = 0;
static flrp_api_message_t data_flrp           = { 0 };
static uint8_t            flrp_api_key[16]    = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

#if ( FLRP_API_ROLE == FLRP_API_ROLE_INITIATOR )
static uint8_t flrp_api_device_eui[SMTC_FLRP_EUI_LENGTH]        = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
static uint8_t flrp_api_target_device_eui[SMTC_FLRP_EUI_LENGTH] = { 0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
#elif ( FLRP_API_ROLE == FLRP_API_ROLE_SLAVE )
static uint8_t flrp_api_device_eui[SMTC_FLRP_EUI_LENGTH]        = { 0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
static uint8_t flrp_api_target_device_eui[SMTC_FLRP_EUI_LENGTH] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
#if ( FLRP_API_IS_LISTENING == false )
#error ( "A slave device not in listening mode cannot receive any message, please set FLRP_API_IS_LISTENING to true" )
#endif
#else
#error ( "Invalid FLRP_API_ROLE value" )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint32_t flrp_api_rx_cb_count    = 0;
static uint32_t flrp_api_rx_cb_ok_count = 0;
static uint32_t flrp_api_tx_cb_count    = 0;
static uint32_t flrp_api_tx_cb_ok_count = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static const char* flrp_status_to_str( smtc_flrp_return_code_t status );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bool flrp_api_init( bool crypto_enabled, bool is_low_frequency, bool listening )
{
    smtc_flrp_api_config_t  flrp_config;
    smtc_flrp_return_code_t return_code;
    bool                    ret = true;

    SMTC_HAL_TRACE_INFO( "=== FLRP init: role=%s crypto=%u band=%s listening=%u ===\n",
                         ( FLRP_API_ROLE == FLRP_API_ROLE_INITIATOR ) ? "initiator" : "slave",
                         ( unsigned ) crypto_enabled, is_low_frequency ? "SubGig" : "2.4GHz", ( unsigned ) listening );
    SMTC_HAL_TRACE_INFO( "FLRP local  EUI : %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", flrp_api_device_eui[0],
                         flrp_api_device_eui[1], flrp_api_device_eui[2], flrp_api_device_eui[3], flrp_api_device_eui[4],
                         flrp_api_device_eui[5], flrp_api_device_eui[6], flrp_api_device_eui[7] );
    SMTC_HAL_TRACE_INFO( "FLRP target EUI : %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", flrp_api_target_device_eui[0],
                         flrp_api_target_device_eui[1], flrp_api_target_device_eui[2], flrp_api_target_device_eui[3],
                         flrp_api_target_device_eui[4], flrp_api_target_device_eui[5], flrp_api_target_device_eui[6],
                         flrp_api_target_device_eui[7] );

    flrp_config.crypto_enabled = crypto_enabled;
    flrp_config.freq_plan      = is_low_frequency ? SMTC_FLRP_FREQ_865MHz : SMTC_FLRP_FREQ_2GHz4;
    flrp_config.crystal_error  = FLRP_API_CRYSTAL_ERROR;
    memcpy( ( void* ) flrp_config.dev_eui, ( void* ) flrp_api_device_eui, SMTC_FLRP_EUI_LENGTH );
    return_code = smtc_flrp_init( flrp_config, user_tx_callback, user_rx_callback, NULL );
    SMTC_HAL_TRACE_INFO( "FLRP smtc_flrp_init -> %s (%u)\n", flrp_status_to_str( return_code ),
                         ( unsigned ) return_code );

    smtc_flrp_crypto_init( );
    smtc_flrp_crypto_return_code_t crypto_rc = smtc_flrp_crypto_set_key( SMTC_SE_APP_KEY, flrp_api_key, 0 );
    ret                                      = ( crypto_rc == SMTC_FLRP_CRYPTO_RC_SUCCESS ) ? true : false;
    SMTC_HAL_TRACE_INFO( "FLRP smtc_flrp_crypto_set_key -> %u (ok=%u)\n", ( unsigned ) crypto_rc, ( unsigned ) ret );

    if( listening == true )
    {
        smtc_flrp_return_code_t rc_listen =
            smtc_flrp_start_periodic_listening( ( uint8_t* ) &data_flrp, sizeof( data_flrp ) );
        SMTC_HAL_TRACE_INFO( "FLRP smtc_flrp_start_periodic_listening -> %s (%u)\n", flrp_status_to_str( rc_listen ),
                             ( unsigned ) rc_listen );
        return_code |= rc_listen;

        memset( ( void* ) &data_flrp.data, 0x00, sizeof( data_flrp.data ) );
        for( uint32_t i = 0; i < sizeof( data_flrp.data ); i++ )
        {
            data_flrp.data[i] = ( uint8_t ) i;
        }
        smtc_flrp_return_code_t rc_prep =
            smtc_flrp_slave_prepare_data_to_send( ( uint8_t* ) &data_flrp, sizeof( data_flrp ) );
        SMTC_HAL_TRACE_INFO( "FLRP smtc_flrp_slave_prepare_data_to_send -> %s (%u)\n", flrp_status_to_str( rc_prep ),
                             ( unsigned ) rc_prep );
        return_code |= rc_prep;
    }

    if( ( return_code != SMTC_FLRP_RC_OK ) || ( ret != true ) )
    {
        ret = false;
        SMTC_HAL_TRACE_ERROR( "FLRP init failed (return_code=%s/%u, crypto ok=%u)\n", flrp_status_to_str( return_code ),
                              ( unsigned ) return_code, ( unsigned ) ret );
    }
    else
    {
        SMTC_HAL_TRACE_INFO( "=== FLRP init OK ===\n" );
    }

    return ret;
}

bool flrp_api_initiate_transfer( bool is_tx )
{
    if( is_tx == true )
    {
        if( flrp_api_initiate_tx( flrp_api_target_device_eui ) == false )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to initiate transmission\n" );
        }
    }
    else
    {
        if( flrp_api_initiate_rx( flrp_api_target_device_eui ) == false )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to initiate reception\n" );
        }
    }
    return ( !( is_tx ) );
}

bool flrp_api_initiate_tx( uint8_t* target_device_eui )
{
    bool ret = true;

    smtc_flrp_com_config_t com_config = {
        .com_mode             = SMTC_FLRP_BIDIRECTIONAL,
        .link_adaptation_mode = SMTC_FLRP_LINK_ADAPTATION_CHANNEL_SELECTION_ONLY,
    };
    memcpy( ( void* ) com_config.slave_dev_eui, ( void* ) target_device_eui, SMTC_FLRP_EUI_LENGTH );
    flrp_api_setup_tx_buffer( );

    smtc_flrp_return_code_t return_code =
        smtc_flrp_initiate_transmission( ( uint8_t* ) &data_flrp, sizeof( data_flrp ), com_config );
    if( return_code != SMTC_FLRP_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to initiate transmission (error %u)\n", return_code );
        ret = false;
    }

    return ret;
}

bool flrp_api_initiate_rx( uint8_t* target_device_eui )
{
    bool ret = true;

    smtc_flrp_com_config_t com_config = {
        .com_mode             = SMTC_FLRP_BIDIRECTIONAL,
        .link_adaptation_mode = SMTC_FLRP_LINK_ADAPTATION_CHANNEL_SELECTION_ONLY,
    };
    memcpy( ( void* ) com_config.slave_dev_eui, ( void* ) target_device_eui, SMTC_FLRP_EUI_LENGTH );
    memset( ( void* ) &data_flrp, 0x00, sizeof( data_flrp ) );

    smtc_flrp_return_code_t return_code =
        smtc_flrp_initiate_reception( ( uint8_t* ) &data_flrp, sizeof( data_flrp ), com_config );
    if( return_code != SMTC_FLRP_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to initiate reception (error %u)\n", return_code );
        ret = false;
    }

    return ret;
}

void user_rx_callback( const void* context, smtc_flrp_return_code_t status, uint32_t payload_size, uint8_t* src_addr,
                       smtc_flrp_rx_stats_t flrp_stats )
{
    flrp_api_rx_cb_count++;

    if( ( status == SMTC_FLRP_RC_OK ) && ( payload_size > 0 ) )
    {
        flrp_api_rx_cb_ok_count++;
        SMTC_HAL_TRACE_INFO( "[RX #%u OK] size=%u ok/err/nok=%u/%u/%u rssi=%d dBm wor_rssi=%d dBm wor_snr=%d dB\n",
                             ( unsigned ) flrp_api_rx_cb_ok_count, ( unsigned ) payload_size,
                             ( unsigned ) flrp_stats.burst.nb_packets_received_ok,
                             ( unsigned ) flrp_stats.burst.nb_packets_check_error,
                             ( unsigned ) flrp_stats.burst.nb_packets_received_nok, ( int ) flrp_stats.burst.rssi_mean,
                             ( int ) flrp_stats.wor.rssi, ( int ) flrp_stats.wor.snr );
        if( payload_size > sizeof( data_flrp.counter ) )
        {
            SMTC_HAL_TRACE_ARRAY( "[RX] data preview", data_flrp.data,
                                  MIN( FLRP_API_DISPLAY_BYTE_SIZE, payload_size - sizeof( data_flrp.counter ) ) );
        }
    }
    else
    {
        SMTC_HAL_TRACE_ERROR(
            "[RX #%u KO] status=%s size=%u ok/err/nok=%u/%u/%u expected=%u rssi=%d dBm "
            "wor_rssi=%d dBm wor_snr=%d dB\n",
            ( unsigned ) flrp_api_rx_cb_count, flrp_status_to_str( status ), ( unsigned ) payload_size,
            ( unsigned ) flrp_stats.burst.nb_packets_received_ok, ( unsigned ) flrp_stats.burst.nb_packets_check_error,
            ( unsigned ) flrp_stats.burst.nb_packets_received_nok, ( unsigned ) flrp_stats.burst.payload_size_expected,
            ( int ) flrp_stats.burst.rssi_mean, ( int ) flrp_stats.wor.rssi, ( int ) flrp_stats.wor.snr );
    }
    flrp_api_setup_tx_buffer( );
}

void user_tx_callback( const void* context, smtc_flrp_return_code_t err_code, bool send_successful, uint8_t* dest_addr )
{
    flrp_api_tx_cb_count++;

    if( ( err_code == SMTC_FLRP_RC_OK ) && ( send_successful == true ) )
    {
        flrp_api_tx_cb_ok_count++;
        SMTC_HAL_TRACE_INFO( "[TX #%u OK] counter=%u\n", ( unsigned ) flrp_api_tx_cb_ok_count,
                             ( unsigned ) data_flrp.counter );
        SMTC_HAL_TRACE_ARRAY( "[TX] data preview", data_flrp.data,
                              MIN( FLRP_API_DISPLAY_BYTE_SIZE, sizeof( data_flrp.data ) ) );
        flrp_api_tx_counter++;
    }
    else
    {
        SMTC_HAL_TRACE_ERROR( "[TX #%u KO] err_code=%s send_successful=%u\n", ( unsigned ) flrp_api_tx_cb_count,
                              flrp_status_to_str( err_code ), ( unsigned ) send_successful );
    }
    flrp_api_setup_tx_buffer( );
}

void flrp_api_setup_tx_buffer( void )
{
    memset( ( void* ) &data_flrp.data, 0x00, sizeof( data_flrp.data ) );
    data_flrp.counter = flrp_api_tx_counter;

    for( uint32_t i = 0; i < sizeof( data_flrp.data ); i++ )
    {
        data_flrp.data[i] = ( uint8_t ) i;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static const char* flrp_status_to_str( smtc_flrp_return_code_t status )
{
    switch( status )
    {
    case SMTC_FLRP_RC_OK:
        return "OK";
    case SMTC_FLRP_RC_NOT_INIT:
        return "NOT_INIT";
    case SMTC_FLRP_RC_INVALID_PARAMS:
        return "INVALID_PARAMS";
    case SMTC_FLRP_RC_UNSUPPORTED_FEATURE:
        return "UNSUPPORTED_FEATURE";
    case SMTC_FLRP_RC_BUSY:
        return "BUSY";
    case SMTC_FLRP_RC_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

/* --- EOF ------------------------------------------------------------------ */
