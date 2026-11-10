/**
 * @file      app_fhss_example.c
 *
 * @brief     LR20xx LoRa intra-packet frequency hopping ping-pong example
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

#include <string.h>

#ifndef LR20XX
#error "fhss_example requires an LR20xx radio"
#endif

#include "app_fhss_example.h"
#include "main_fhss_example.h"

#include "lr20xx_radio_lora.h"
#include "smtc_hal_led.h"
#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_hal_dbg_trace.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "FHSS"
#include "smtc_rac_log.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// Backward compatibility alias for existing code
#define FHSS_PRINT( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )

/**
 * @brief payload composition
 *
 * |      PREFIX      |  SEPARATOR  |      COUNTER      |
 * | "PING" or "PONG" | (uint8_t) 0 | fhss_example.counter |
 *
 */

#define PREFIX_SIZE ( 4 )
#define SEPARATOR_SIZE ( 1 )
#define COUNTER_SIZE ( 1 )

#define FHSS_PADDING_SIZE ( 200 )
#define PAYLOAD_SIZE ( PREFIX_SIZE + SEPARATOR_SIZE + COUNTER_SIZE + FHSS_PADDING_SIZE )

#define PREFIX_OFFSET ( 0 )
#define SEPARATOR_OFFSET ( PREFIX_OFFSET + PREFIX_SIZE )
#define COUNTER_OFFSET ( SEPARATOR_OFFSET + SEPARATOR_SIZE )
#define PADDING_OFFSET ( COUNTER_OFFSET + COUNTER_SIZE )

#define FHSS_TARGET_NAME "LR20xx"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#ifndef SUBORDINATE_PROCESSING_TIME_MS
#define SUBORDINATE_PROCESSING_TIME_MS 10  // Default: original 10ms (fast ping-pong)
#endif

static const smtc_rac_scheduling_t    ASAP                      = SMTC_RAC_ASAP_TRANSACTION;       // (alias)
static const smtc_rac_scheduling_t    SCHEDULED                 = SMTC_RAC_SCHEDULED_TRANSACTION;  // (alias)
static const smtc_rac_lora_syncword_t SYNC_WORD                 = LORA_PRIVATE_NETWORK_SYNCWORD;   // (alias)
static const uint32_t                 NO_DELAY                  = 0;                               // ms
static const uint32_t                 DELAY                     = 250;                             // ms
static const uint32_t                 RX_PROCESSING_TIME        = 10;                              // ms
static const uint32_t                 TX_SUBORDINATE_TIME       = SUBORDINATE_PROCESSING_TIME_MS;  // ms
static const uint32_t                 MANAGER_RX_TIMEOUT        = 500;                             // ms
static const uint32_t                 SUBORDINATE_RX_TIMEOUT    = 30000;                           // ms
static const uint8_t                  MAX_EXCHANGE_COUNT        = 25;                              // (no unit)
static const uint8_t                  MAX_RETRY_COUNT           = 5;                               // (no unit)
static const uint8_t                  PING_MESSAGE[PREFIX_SIZE] = "PING";                          // bytes
static const uint8_t                  PONG_MESSAGE[PREFIX_SIZE] = "PONG";                          // bytes

static uint32_t fhss_hop_freq_table[] = {
    490000000,
    491000000,
    492000000,
    493000000,
    494000000,
    495000000,
    496000000,
    497000000,
    498000000,
    499000000,
    500000000,
    501000000,
    502000000,
    503000000,
    504000000,
    505000000,
    506000000,
    507000000,
    508000000,
    509000000,
    510000000,
    511000000,
    512000000,
    513000000,
    514000000,
    515000000,
    516000000,
    517000000,
    518000000,
    519000000,
};

static const uint8_t FHSS_HOP_FREQ_COUNT =
    ( uint8_t ) ( sizeof( fhss_hop_freq_table ) / sizeof( fhss_hop_freq_table[0] ) );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct fhss_example_s
{
    bool                is_manager;             // true if device start exchange and adds delay between transactions
    bool                tx_requested;           // true if the next transaction must be tx
    uint8_t             consecutive_fails;      // number of failed exchanges
    uint8_t             counter;                // incremented after each successful exchange
    uint8_t             payload[PAYLOAD_SIZE];  // bytes buffer
    uint8_t             radio_access_id;        // store result of `smtc_rac_open_radio`
    smtc_rac_context_t* transaction;            // associated transaction
} fhss_example_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static fhss_example_t fhss_example = {
    .is_manager        = false,
    .tx_requested      = false,
    .consecutive_fails = 0,
    .counter           = 0,
    .payload           = { 0 },
    .radio_access_id   = 0,  // set in `fhss_example_init`
};

static uint32_t fhss_hop_irq_count = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/** @brief reset `fhss_example` structure */
static void restart_fhss_example( void );

/** @brief set `fhss_example.transaction` and start the transmission */
static void fhss_example_tx( smtc_rac_scheduling_t scheduling, uint32_t delay );

/** @brief set `fhss_example.transaction` and start the reception */
static void fhss_example_rx( smtc_rac_scheduling_t scheduling, uint32_t delay );

/** @brief configure LR20xx LoRa intra-packet frequency hopping */
static void configure_lora_hopping( void );

/** @brief switch on `SMTC_PF_LED_TX` and `SMTC_PF_LED_RX` according to `fhss_example.transaction` */
static void pre_fhss_example_callback( void );

/** @brief call `fhss_example_tx` or `fhss_example_rx` according to the current state of `fhss_example` */
static void post_fhss_example_callback( rp_status_t status );

/** @brief retry the last transaction */
static void retry_last_transaction( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void fhss_example_init( void )
{
    fhss_example.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );

    fhss_example.transaction = smtc_rac_get_context( fhss_example.radio_access_id );

    fhss_example.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = &fhss_example.payload[0];
    fhss_example.transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = &fhss_example.payload[0];
    fhss_example.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( fhss_example.payload );
    fhss_example.transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = sizeof( fhss_example.payload );

    fhss_example.transaction->modulation_type                        = SMTC_RAC_MODULATION_LORA;
    fhss_example.transaction->radio_params.lora.is_ranging_exchange  = false;
    fhss_example.transaction->radio_params.lora.frequency_in_hz      = fhss_hop_freq_table[0];
    fhss_example.transaction->radio_params.lora.tx_power_in_dbm      = TX_OUTPUT_POWER_DBM;
    fhss_example.transaction->radio_params.lora.sf                   = LORA_SPREADING_FACTOR;
    fhss_example.transaction->radio_params.lora.bw                   = LORA_BANDWIDTH;
    fhss_example.transaction->radio_params.lora.cr                   = LORA_CODING_RATE;
    fhss_example.transaction->radio_params.lora.preamble_len_in_symb = LORA_PREAMBLE_LENGTH;
    fhss_example.transaction->radio_params.lora.header_type          = LORA_PKT_LEN_MODE;
    fhss_example.transaction->radio_params.lora.invert_iq_is_on      = LORA_IQ;
    fhss_example.transaction->radio_params.lora.crc_is_on            = LORA_CRC;
    fhss_example.transaction->radio_params.lora.sync_word            = SYNC_WORD;
    fhss_example.transaction->radio_params.lora.tx_size              = PAYLOAD_SIZE;
    fhss_example.transaction->radio_params.lora.max_rx_size          = sizeof( fhss_example.payload );

    fhss_example.transaction->scheduler_config.start_time_ms                   = 0;     // set at each transaction
    fhss_example.transaction->scheduler_config.scheduling                      = ASAP;  // set at each transaction
    fhss_example.transaction->scheduler_config.callback_post_radio_transaction = post_fhss_example_callback;
    fhss_example.transaction->scheduler_config.callback_pre_radio_transaction  = pre_fhss_example_callback;

    FHSS_PRINT( "LoRa FHSS initialized: target=%s, initial=%lu Hz, hop_period=%u symbols, channels=%u\n",
                FHSS_TARGET_NAME, ( unsigned long ) fhss_hop_freq_table[0], ( unsigned int ) FHSS_HOP_PERIOD_IN_SYMBOLS,
                ( unsigned int ) FHSS_HOP_FREQ_COUNT );

    // start application
    restart_fhss_example( );
}

void fhss_example_on_button_press( void )
{
    FHSS_PRINT( "this device is now manager\n" );
    fhss_example.is_manager = true;

    FHSS_PRINT( "aborting current transaction\n" );
    smtc_rac_return_code_t abort_code = SMTC_SW_PLATFORM( smtc_rac_abort_radio_submit( fhss_example.radio_access_id ) );
    if( abort_code != SMTC_RAC_SUCCESS )
    {
        FHSS_PRINT( "usp/rac: ERROR: failed to abort: %d\n", ( int ) abort_code );
    }
    else
    {
        FHSS_PRINT( "requesting PING\n" );
        fhss_example.tx_requested = true;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void restart_fhss_example( void )
{
    FHSS_PRINT( "resetting parameters and restarting FHSS loop\n" );
    fhss_example.consecutive_fails = 0;
    fhss_example.counter           = 0;

    FHSS_PRINT( "this device is now subordinate\n" );
    fhss_example.is_manager = false;
    fhss_example_rx( ASAP, NO_DELAY );
}

static void fhss_example_tx( smtc_rac_scheduling_t scheduling, uint32_t delay )
{
    // configure transaction
    fhss_example.transaction->radio_params.lora.is_tx        = true;
    fhss_example.transaction->scheduler_config.scheduling    = scheduling;
    fhss_example.transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + delay;

    const uint8_t* message = ( fhss_example.is_manager ) ? PING_MESSAGE : PONG_MESSAGE;
    ( void ) memcpy( fhss_example.payload + PREFIX_OFFSET, message, PREFIX_SIZE );
    ( void ) memset( fhss_example.payload + SEPARATOR_OFFSET, 0, SEPARATOR_SIZE );
    ( void ) memcpy( fhss_example.payload + COUNTER_OFFSET, &fhss_example.counter, COUNTER_SIZE );
    ( void ) memset( fhss_example.payload + PADDING_OFFSET, fhss_example.counter, FHSS_PADDING_SIZE );

    // call the rac API
    FHSS_PRINT( "sending FHSS packet (value=%d, delay=%dms)\n", fhss_example.counter, delay );
    smtc_rac_return_code_t error_code =
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( fhss_example.radio_access_id ) );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( error_code == SMTC_RAC_SUCCESS );
}

static void fhss_example_rx( smtc_rac_scheduling_t scheduling, uint32_t delay )
{
    // configure transaction
    fhss_example.transaction->radio_params.lora.is_tx = false;
    fhss_example.transaction->radio_params.lora.rx_timeout_ms =
        ( fhss_example.is_manager ) ? MANAGER_RX_TIMEOUT : SUBORDINATE_RX_TIMEOUT;
    fhss_example.transaction->scheduler_config.scheduling    = scheduling;
    fhss_example.transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + delay;

    // clear payload
    memset( fhss_example.payload, 0, sizeof( fhss_example.payload ) );

    // call the rac API
    FHSS_PRINT( "receiving FHSS packet...\n" );
    smtc_rac_return_code_t error_code =
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( fhss_example.radio_access_id ) );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( error_code == SMTC_RAC_SUCCESS );
}

static void configure_lora_hopping( void )
{
    void* radio_driver_context = smtc_rac_get_radio_driver_context( );

    lr20xx_radio_lora_hopping_cfg_t hopping_cfg = {
        .hop_ctrl    = LR20XX_RADIO_LORA_HOPPING_CTRL_ENABLED,
        .hop_period  = FHSS_HOP_PERIOD_IN_SYMBOLS,
        .freq_hop    = fhss_hop_freq_table,
        .nb_freq_hop = FHSS_HOP_FREQ_COUNT,
    };

    lr20xx_status_t lr20xx_status = lr20xx_radio_lora_set_freq_hop( radio_driver_context, &hopping_cfg );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( lr20xx_status == LR20XX_STATUS_OK );

}

static void pre_fhss_example_callback( void )
{
    FHSS_PRINT( "usp/rac: transaction is starting with LoRa FHSS enabled\n" );
    configure_lora_hopping( );

    const hal_led_id_t led = ( fhss_example.transaction->radio_params.lora.is_tx ) ? HAL_LED_TX : HAL_LED_RX;
    hal_led_set( led, true );
}

static void post_fhss_example_callback( rp_status_t status )
{
    bool payload_is_correct = true;
    // will be used to hold the received payload data
    uint8_t prefix_received[PREFIX_SIZE] = { 0 };
    uint8_t separator_received           = 0;
    uint8_t counter_received             = 0;

    hal_led_set( HAL_LED_TX, false );
    hal_led_set( HAL_LED_RX, false );

    switch( status )
    {
    case RP_STATUS_LR_FHSS_HOP:
        fhss_hop_irq_count += 1;
        FHSS_PRINT( "usp/rac: new event: intra-packet hop irq #%lu\n", ( unsigned long ) fhss_hop_irq_count );
        break;

    case RP_STATUS_TX_DONE:
        FHSS_PRINT( "usp/rac: new event: FHSS transmission has ended (success)\n" );
        if( fhss_example.is_manager == true )
        {
            fhss_example_rx( SCHEDULED, RX_PROCESSING_TIME );
        }
        else
        {
            fhss_example_rx( ASAP, NO_DELAY );
        }
        break;

    case RP_STATUS_RX_PACKET:
        if( fhss_example.transaction->smtc_rac_data_result.rx_size == PAYLOAD_SIZE )
        {
            FHSS_PRINT( "usp/rac: new event: FHSS reception has ended (success)\n" );
            ( void ) memcpy( prefix_received, fhss_example.payload + PREFIX_OFFSET, PREFIX_SIZE );
            ( void ) memcpy( &separator_received, fhss_example.payload + SEPARATOR_OFFSET, SEPARATOR_SIZE );
            ( void ) memcpy( &counter_received, fhss_example.payload + COUNTER_OFFSET, COUNTER_SIZE );
        }
        else
        {
            payload_is_correct = false;
        }

        // check payload
        payload_is_correct &= ( separator_received == 0 );
        if( fhss_example.is_manager == true )
        {
            payload_is_correct &= ( memcmp( prefix_received, PONG_MESSAGE, PREFIX_SIZE ) == 0 );
            payload_is_correct &= ( counter_received == fhss_example.counter );
        }
        else
        {
            payload_is_correct &= ( memcmp( prefix_received, PING_MESSAGE, PREFIX_SIZE ) == 0 );
        }
        if( payload_is_correct == false )
        {
            FHSS_PRINT( "ERROR: unexpected payload\n" );
            FHSS_PRINT( "prefix_received='%d', separator_received=%d, counter_received=%d\n", fhss_example.payload[0],
                        separator_received, counter_received );

            fhss_example.consecutive_fails += 1;
        }
        else
        {
            FHSS_PRINT( "payload is correct, continuing\n" );
            fhss_example.counter += 1;
            fhss_example.consecutive_fails = 0;
        }

        // if subordinate, send back the value received
        if( fhss_example.is_manager == false )
        {
            fhss_example.counter = counter_received;
            fhss_example_tx( SCHEDULED, TX_SUBORDINATE_TIME );
            break;
        }
        // from this line to the end of the case, device is manager

        if( fhss_example.counter >= MAX_EXCHANGE_COUNT )
        {
            FHSS_PRINT( "max exchange count reached, restarting\n" );
            restart_fhss_example( );
            break;
        }

        if( fhss_example.consecutive_fails >= MAX_RETRY_COUNT )
        {
            FHSS_PRINT( "max retry count reached, restarting\n" );
            restart_fhss_example( );
            break;
        }

        if( fhss_example.consecutive_fails > 0 )
        {
            FHSS_PRINT( "retrying (fails=%d, max=%d)\n", fhss_example.consecutive_fails, MAX_RETRY_COUNT );
        }

        fhss_example_tx( ASAP, DELAY );
        break;

    case RP_STATUS_RX_TIMEOUT:
        FHSS_PRINT( "usp/rac: new event: FHSS reception has ended (timeout)\n" );
        fhss_example.consecutive_fails += 1;

        if( fhss_example.is_manager == false )
        {
            FHSS_PRINT( "device is subordinate, continuing\n" );
            // subordinate does not use `fhss_example.consecutive_fails`, it starts a new rx every time
            fhss_example_rx( ASAP, NO_DELAY );
            break;
        }

        if( fhss_example.consecutive_fails >= MAX_RETRY_COUNT )
        {
            FHSS_PRINT( "max retry count reached, restarting\n" );
            restart_fhss_example( );
            break;
        }

        FHSS_PRINT( "retrying (fails=%d, max=%d)\n", fhss_example.consecutive_fails, MAX_RETRY_COUNT );
        fhss_example_tx( ASAP, DELAY );
        break;

    case RP_STATUS_TASK_ABORTED:
        FHSS_PRINT( "usp/rac: new event: transaction aborted\n" );
        fhss_example.consecutive_fails += 1;

        if( fhss_example.tx_requested == true )
        {
            FHSS_PRINT( "PING requested, continuing\n" );
            fhss_example.tx_requested      = false;
            fhss_example.consecutive_fails = 0;
            fhss_example.counter           = 0;
            fhss_example_tx( ASAP, NO_DELAY );
            break;
        }
        else
        {
            retry_last_transaction( );
        }

        break;

    case RP_STATUS_RX_CRC_ERROR:  // single error case that could occur
        FHSS_PRINT( "usp/rac: new event: ERROR: RX_CRC_ERROR\n" );
        fhss_example.consecutive_fails += 1;
        retry_last_transaction( );
        break;

    default:  // other error cases should not occur
        FHSS_PRINT( "usp/rac: new event: ERROR: unexpected transaction status: %d\n", ( int ) status );
        SMTC_MODEM_HAL_PANIC( );
        break;
    }
}

static void retry_last_transaction( void )
{
    FHSS_PRINT( "trying last transaction again\n" );
    fhss_example.transaction->scheduler_config.scheduling    = ASAP;
    fhss_example.transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + RX_PROCESSING_TIME;
    smtc_rac_return_code_t error_code =
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( fhss_example.radio_access_id ) );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( error_code == SMTC_RAC_SUCCESS );
}
