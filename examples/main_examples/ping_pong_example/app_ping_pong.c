/**
 * @file      app_ping_pong.c
 *
 * @brief     Simple ping-ping example for LR1110 or LR1120 chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
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

#include <string.h>

#include "app_ping_pong.h"
#include "main_ping_pong.h"

#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "PING-PONG"
#include "smtc_rac_log.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// Backward compatibility alias for existing code
#define PING_PONG_PRINT( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )

/**
 * @brief payload composition
 *
 * |      PREFIX      |  SEPARATOR  |      COUNTER      |
 * | "PING" or "PONG" | (uint8_t) 0 | ping_pong.counter |
 *
 */

#define PREFIX_SIZE ( 4 )
#define SEPARATOR_SIZE ( 1 )
#define COUNTER_SIZE ( 1 )
#define PAYLOAD_SIZE ( PREFIX_SIZE + SEPARATOR_SIZE + COUNTER_SIZE )

#define PREFIX_OFFSET ( 0 )
#define SEPARATOR_OFFSET ( PREFIX_OFFSET + PREFIX_SIZE )
#define COUNTER_OFFSET ( SEPARATOR_OFFSET + SEPARATOR_SIZE )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// Configurable processing time - can be overridden at build time for hw_modem compatibility
// Usage: -DSUBORDINATE_PROCESSING_TIME_MS=5000
#ifndef SUBORDINATE_PROCESSING_TIME_MS
#define SUBORDINATE_PROCESSING_TIME_MS 10  // Default: original 10ms (fast ping-pong)
#endif

static const smtc_rac_scheduling_t    ASAP                      = SMTC_RAC_ASAP_TRANSACTION;       // (alias)
static const smtc_rac_scheduling_t    SCHEDULED                 = SMTC_RAC_SCHEDULED_TRANSACTION;  // (alias)
static const smtc_rac_lora_syncword_t SYNC_WORD                 = LORA_PRIVATE_NETWORK_SYNCWORD;   // (alias)
static const uint32_t                 NO_DELAY                  = 0;                               // ms
static const uint32_t                 DELAY                     = 250;                             // ms
static const uint32_t                 RX_PROCESSING_TIME        = 10;                              // ms (configurable)
static const uint32_t                 TX_SUBORDINATE_TIME       = SUBORDINATE_PROCESSING_TIME_MS;  // ms (configurable)
static const uint32_t                 MANAGER_RX_TIMEOUT        = 500;                             // ms
static const uint32_t                 SUBORDINATE_RX_TIMEOUT    = 30000;                           // ms
static const uint8_t                  MAX_EXCHANGE_COUNT        = 25;                              // (no unit)
static const uint8_t                  MAX_RETRY_COUNT           = 5;                               // (no unit)
static const uint8_t                  PING_MESSAGE[PREFIX_SIZE] = "PING";                          // bytes
static const uint8_t                  PONG_MESSAGE[PREFIX_SIZE] = "PONG";                          // bytes

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct ping_pong_s
{
    bool                is_manager;             // true iff device start exchange and adds delay between transactions
    bool                tx_requested;           // true iff the next transaction must be tx
    uint8_t             consecutive_fails;      // number of failed exchanges
    uint8_t             counter;                // incremented after each successful exchange
    uint8_t             payload[PAYLOAD_SIZE];  // bytes buffer
    uint8_t             radio_access_id;        // store result of `smtc_rac_open_radio`
    smtc_rac_context_t* transaction;            // associated transaction
} ping_pong_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static ping_pong_t ping_pong = {
    .is_manager        = false,
    .tx_requested      = false,
    .consecutive_fails = 0,
    .counter           = 0,
    .payload           = { 0 },
    .radio_access_id   = 0,  // set in `ping_pong_init`

};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/** @brief reset `ping_pong` structure */
static void restart_ping_pong( void );

/** @brief set `ping_pong.transaction` and start the transmission */
static void ping_pong_tx( smtc_rac_scheduling_t scheduling, uint32_t delay );

/** @brief set `ping_pong.transaction` and start the reception */
static void ping_pong_rx( smtc_rac_scheduling_t scheduling, uint32_t delay );

/** @brief switch on `SMTC_PF_LED_TX` and `SMTC_PF_LED_RX` according to `ping_pong.transaction` */
static void pre_ping_pong_callback( void );

/** @brief call `ping_pong_tx` or `ping_pong_rx` according to the current state of `ping_pong` */
static void post_ping_pong_callback( rp_status_t status );

/** @brief retry the last transaction */
static void retry_last_transaction( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void ping_pong_init( void )
{
    // initialize static struct

    ping_pong.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );

    ping_pong.transaction = smtc_rac_get_context( ping_pong.radio_access_id );

    ping_pong.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = &ping_pong.payload[0];
    ping_pong.transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = &ping_pong.payload[0];
    ping_pong.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( ping_pong.payload );
    ping_pong.transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = sizeof( ping_pong.payload );

    ping_pong.transaction->modulation_type                        = SMTC_RAC_MODULATION_LORA;
    ping_pong.transaction->radio_params.lora.is_ranging_exchange  = false;
    ping_pong.transaction->radio_params.lora.frequency_in_hz      = RF_FREQ_IN_HZ;
    ping_pong.transaction->radio_params.lora.tx_power_in_dbm      = TX_OUTPUT_POWER_DBM;
    ping_pong.transaction->radio_params.lora.sf                   = LORA_SPREADING_FACTOR;
    ping_pong.transaction->radio_params.lora.bw                   = LORA_BANDWIDTH;
    ping_pong.transaction->radio_params.lora.cr                   = LORA_CODING_RATE;
    ping_pong.transaction->radio_params.lora.preamble_len_in_symb = LORA_PREAMBLE_LENGTH;
    ping_pong.transaction->radio_params.lora.header_type          = LORA_PKT_LEN_MODE;
    ping_pong.transaction->radio_params.lora.invert_iq_is_on      = LORA_IQ;
    ping_pong.transaction->radio_params.lora.crc_is_on            = LORA_CRC;
    ping_pong.transaction->radio_params.lora.sync_word            = SYNC_WORD;
    ping_pong.transaction->radio_params.lora.tx_size              = PAYLOAD_SIZE;
    ping_pong.transaction->radio_params.lora.max_rx_size          = sizeof( ping_pong.payload );

    ping_pong.transaction->scheduler_config.start_time_ms                   = 0;     // set at each transaction
    ping_pong.transaction->scheduler_config.scheduling                      = ASAP;  // set at each transaction
    ping_pong.transaction->scheduler_config.callback_post_radio_transaction = post_ping_pong_callback;
    ping_pong.transaction->scheduler_config.callback_pre_radio_transaction  = pre_ping_pong_callback;

    // start application
    restart_ping_pong( );
}

void ping_pong_on_button_press( void )
{
    PING_PONG_PRINT( "this device is now manager\n" );
    ping_pong.is_manager = true;

    PING_PONG_PRINT( "aborting current transaction\n" );
    smtc_rac_return_code_t abort_code = SMTC_SW_PLATFORM( smtc_rac_abort_radio_submit( ping_pong.radio_access_id ) );
    if( abort_code != SMTC_RAC_SUCCESS )
    {
        PING_PONG_PRINT( "usp/rac: ERROR: failed to abort: %d\n", ( int ) abort_code );
    }
    else
    {
        PING_PONG_PRINT( "requesting PING\n" );
        ping_pong.tx_requested = true;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void restart_ping_pong( void )
{
    PING_PONG_PRINT( "resetting parameters and restarting loop\n" );
    PING_PONG_PRINT( "this device is now subordinate\n" );
    ping_pong.is_manager        = false;
    ping_pong.consecutive_fails = 0;
    ping_pong.counter           = 0;
    ping_pong_rx( ASAP, NO_DELAY );
}

static void ping_pong_tx( smtc_rac_scheduling_t scheduling, uint32_t delay )
{
    // configure transaction
    ping_pong.transaction->radio_params.lora.is_tx        = true;
    ping_pong.transaction->scheduler_config.scheduling    = scheduling;
    ping_pong.transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + delay;

    // fill payload
    const uint8_t* message = ( ping_pong.is_manager ) ? PING_MESSAGE : PONG_MESSAGE;
    ( void ) memcpy( ping_pong.payload + PREFIX_OFFSET, message, PREFIX_SIZE );
    ( void ) memset( ping_pong.payload + SEPARATOR_OFFSET, 0, SEPARATOR_SIZE );
    ( void ) memcpy( ping_pong.payload + COUNTER_OFFSET, &ping_pong.counter, COUNTER_SIZE );

    // call the rac API
    PING_PONG_PRINT( "sending (value=%d, delay=%dms)\n", ping_pong.counter, delay );
    smtc_rac_return_code_t error_code =
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ping_pong.radio_access_id ) );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( error_code == SMTC_RAC_SUCCESS );
}

static void ping_pong_rx( smtc_rac_scheduling_t scheduling, uint32_t delay )
{
    // configure transaction
    ping_pong.transaction->radio_params.lora.is_tx = false;
    ping_pong.transaction->radio_params.lora.rx_timeout_ms =
        ( ping_pong.is_manager ) ? MANAGER_RX_TIMEOUT : SUBORDINATE_RX_TIMEOUT;
    ping_pong.transaction->scheduler_config.scheduling    = scheduling;
    ping_pong.transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + delay;

    // clear payload
    memset( ping_pong.payload, 0, sizeof( ping_pong.payload ) );

    // call the rac API
    PING_PONG_PRINT( "receiving...\n" );
    smtc_rac_return_code_t error_code =
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ping_pong.radio_access_id ) );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( error_code == SMTC_RAC_SUCCESS );
}

static void pre_ping_pong_callback( void )
{
    PING_PONG_PRINT( "usp/rac: transaction is starting\n" );
    const smtc_led_pin_e led = ( ping_pong.transaction->radio_params.lora.is_tx ) ? SMTC_PF_LED_TX : SMTC_PF_LED_RX;
    set_led( led, true );
}

static void post_ping_pong_callback( rp_status_t status )
{
    bool payload_is_correct = true;
    // will be used to hold the received payload data
    uint8_t prefix_received[PREFIX_SIZE] = { 0 };
    uint8_t separator_received           = 0;
    uint8_t counter_received             = 0;

    set_led( SMTC_PF_LED_TX, false );
    set_led( SMTC_PF_LED_RX, false );

    switch( status )
    {
    case RP_STATUS_TX_DONE:
        PING_PONG_PRINT( "usp/rac: new event: transaction (transmission) has ended (success)\n" );
        if( ping_pong.is_manager == true )
        {
            ping_pong_rx( SCHEDULED, RX_PROCESSING_TIME );
        }
        else
        {
            ping_pong_rx( ASAP, NO_DELAY );
        }
        break;

    case RP_STATUS_RX_PACKET:
        if( ping_pong.transaction->smtc_rac_data_result.rx_size == PAYLOAD_SIZE )
        {
            PING_PONG_PRINT( "usp/rac: new event: transaction (reception) has ended (success)\n" );
            ( void ) memcpy( prefix_received, ping_pong.payload + PREFIX_OFFSET, PREFIX_SIZE );
            ( void ) memcpy( &separator_received, ping_pong.payload + SEPARATOR_OFFSET, SEPARATOR_SIZE );
            ( void ) memcpy( &counter_received, ping_pong.payload + COUNTER_OFFSET, COUNTER_SIZE );
        }
        else
        {
            payload_is_correct = false;
        }

        // check payload
        payload_is_correct &= ( separator_received == 0 );
        if( ping_pong.is_manager == true )
        {
            payload_is_correct &= ( memcmp( prefix_received, PONG_MESSAGE, PREFIX_SIZE ) == 0 );
            payload_is_correct &= ( counter_received == ping_pong.counter );
        }
        else
        {
            payload_is_correct &= ( memcmp( prefix_received, PING_MESSAGE, PREFIX_SIZE ) == 0 );
        }
        if( payload_is_correct == false )
        {
            PING_PONG_PRINT( "ERROR: unexpected payload\n" );
            PING_PONG_PRINT( "prefix_received='%d', separator_received=%d, counter_received=%d\n", ping_pong.payload[0],
                             separator_received, counter_received );

            ping_pong.consecutive_fails += 1;
        }
        else
        {
            PING_PONG_PRINT( "payload is correct, continuing\n" );
            ping_pong.counter += 1;
            ping_pong.consecutive_fails = 0;
        }

        // if subordinate, send back the value received
        if( ping_pong.is_manager == false )
        {
            ping_pong.counter = counter_received;
            ping_pong_tx( SCHEDULED, TX_SUBORDINATE_TIME );
            break;
        }
        // from this line to the end of the case, device is manager

        if( ping_pong.counter >= MAX_EXCHANGE_COUNT )
        {
            PING_PONG_PRINT( "max exchange count reached, restarting\n" );
            restart_ping_pong( );
            break;
        }

        if( ping_pong.consecutive_fails >= MAX_RETRY_COUNT )
        {
            PING_PONG_PRINT( "max retry count reached, restarting\n" );
            restart_ping_pong( );
            break;
        }

        if( ping_pong.consecutive_fails > 0 )
        {
            PING_PONG_PRINT( "retrying (fails=%d, max=%d)\n", ping_pong.consecutive_fails, MAX_RETRY_COUNT );
        }

        ping_pong_tx( ASAP, DELAY );
        break;

    case RP_STATUS_RX_TIMEOUT:
        PING_PONG_PRINT( "usp/rac: new event: transaction (reception) has ended (timeout)\n" );
        ping_pong.consecutive_fails += 1;

        if( ping_pong.is_manager == false )
        {
            PING_PONG_PRINT( "device is subordinate, continuing\n" );
            // subordinate does not use `ping_pong.consecutive_fails`, it starts a new rx every time
            ping_pong_rx( ASAP, NO_DELAY );
            break;
        }

        if( ping_pong.consecutive_fails >= MAX_RETRY_COUNT )
        {
            PING_PONG_PRINT( "max retry count reached, restarting\n" );
            restart_ping_pong( );
            break;
        }

        PING_PONG_PRINT( "retrying (fails=%d, max=%d)\n", ping_pong.consecutive_fails, MAX_RETRY_COUNT );
        ping_pong_tx( ASAP, DELAY );
        break;

    case RP_STATUS_TASK_ABORTED:
        PING_PONG_PRINT( "usp/rac: new event: transaction aborted\n" );
        ping_pong.consecutive_fails += 1;

        if( ping_pong.tx_requested == true )
        {
            PING_PONG_PRINT( "PING requested, continuing\n" );
            ping_pong.tx_requested      = false;
            ping_pong.consecutive_fails = 0;
            ping_pong.counter           = 0;
            ping_pong_tx( ASAP, NO_DELAY );
            break;
        }
        else
        {
            retry_last_transaction( );
        }

        break;

    case RP_STATUS_RX_CRC_ERROR:  // single error case that could occur
        PING_PONG_PRINT( "usp/rac: new event: ERROR: RX_CRC_ERROR\n" );
        ping_pong.consecutive_fails += 1;
        retry_last_transaction( );
        break;

    default:  // other error cases should not occur
        PING_PONG_PRINT( "usp/rac: new event: ERROR: unexpected transaction status: %d\n", ( int ) status );
        SMTC_MODEM_HAL_PANIC( );
        break;
    }
}

static void retry_last_transaction( void )
{
    PING_PONG_PRINT( "trying last transaction again\n" );
    ping_pong.transaction->scheduler_config.scheduling    = ASAP;
    ping_pong.transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + RX_PROCESSING_TIME;
    smtc_rac_return_code_t error_code =
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ping_pong.radio_access_id ) );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( error_code == SMTC_RAC_SUCCESS );
}
