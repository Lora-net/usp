/**
 * @file      app_per.c
 *
 * @brief     Simple PER (Packet Error Rate) example
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

#include "app_per.h"
#include "main_per.h"

#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "PER"
#include "smtc_rac_log.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#ifndef LORA_SYNCWORD
#define LORA_SYNCWORD LORA_PUBLIC_NETWORK_SYNCWORD
#endif

#define RECEIVER 1
#define TRANSMITTER 2

// Backward compatibility aliases for existing code
#define PER_LOG_INFO( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )
#define PER_LOG_WARN( ... ) SMTC_HAL_TRACE_WARNING( __VA_ARGS__ )
#define PER_LOG_ERROR( ... ) SMTC_HAL_TRACE_ERROR( __VA_ARGS__ )
#define PER_LOG_STATS( ... ) SMTC_HAL_TRACE_INFO( "STATS " __VA_ARGS__ )
#define PER_LOG_CONFIG( ... ) SMTC_HAL_TRACE_INFO( "CONFIG " __VA_ARGS__ )
#define PER_LOG_TX( ... ) SMTC_HAL_TRACE_INFO( "TX " __VA_ARGS__ )
#define PER_LOG_RX( ... ) SMTC_HAL_TRACE_INFO( "RX " __VA_ARGS__ )

// Helper macros for calculations
#define CALCULATE_PER_PERCENT( )                                                                                \
    ( ( packet_error_rate.exchange_count > 0 )                                                                  \
          ? ( ( float ) packet_error_rate.failure_count * 100.0f / ( float ) packet_error_rate.exchange_count ) \
          : 0.0f )
#define CALCULATE_SUCCESS_RATE( )                                                                         \
    ( ( packet_error_rate.exchange_count > 0 )                                                            \
          ? ( ( float ) ( packet_error_rate.exchange_count - packet_error_rate.failure_count ) * 100.0f / \
              ( float ) packet_error_rate.exchange_count )                                                \
          : 0.0f )

/**
 * @brief payload composition
 *
 * | HEADER |  SEPARATOR  |    COUNTER     |
 * | "PER"  | (uint8_t) 0 | exchange_count |
 *
 */

#define HEADER_SIZE ( 3 )
#define SEPARATOR_SIZE ( 1 )
#define COUNTER_SIZE ( 4 )  // sizeof(uint32_t)
#define PAYLOAD_SIZE ( HEADER_SIZE + SEPARATOR_SIZE + COUNTER_SIZE )

#define HEADER_OFFSET ( 0 )
#define SEPARATOR_OFFSET ( HEADER_OFFSET + HEADER_SIZE )
#define COUNTER_OFFSET ( SEPARATOR_OFFSET + SEPARATOR_SIZE )

#define LBT_SNIFF_DURATION_MS_DEFAULT ( 5 )
#define LBT_THRESHOLD_DBM_DEFAULT ( int16_t ) ( -80 )
#define LBT_BW_HZ__DEFAULT ( 200000 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if ( ROLE == RECEIVER )
static const bool IS_TRANSMITTER = false;
#elif ( ROLE == TRANSMITTER )
static const bool IS_TRANSMITTER = true;
#else
#error "Please define ROLE as either RECEIVER or TRANSMITTER"
#endif

static const uint32_t RX_TIMEOUT           = 30000;  // ms
static const uint32_t INTER_EXCHANGE_DELAY = 20;     // ms
static const uint32_t INTER_SERIES_DELAY   = 10000;  // ms
static const uint32_t MAX_EXCHANGE_COUNT   = 100;    // (no unit)
static const uint8_t  HEADER[HEADER_SIZE]  = "PER";  // bytes

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct packet_error_rate_s
{
    uint32_t            exchange_count;         // number of sent/received messages
    bool                exchange_started;       // true iff a packet has correctly been received
    uint32_t            failure_count;          // number of failed receptions after the exchange has started
    uint8_t             payload[PAYLOAD_SIZE];  // bytes buffer
    uint8_t             radio_access_id;        // store the result of `smtc_rac_request_radio_access`
    smtc_rac_context_t* transaction;            // associated transaction
} packet_error_rate_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static packet_error_rate_t packet_error_rate = {
    .exchange_count   = 0,
    .exchange_started = false,
    .failure_count    = 0,
    .payload          = { 0 },
    .radio_access_id  = 0,     // set in `per_init`
    .transaction      = NULL,  // set in `per_init`
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void pre_transaction_callback( void );

static void post_transaction_callback( rp_status_t status );

static void start_new_transaction( uint32_t delay );

static const char* rp_status_to_str( const rp_status_t status );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void per_init( void )
{
    // Print banner and device role
    PER_LOG_INFO( "===== SEMTECH PER TEST INITIALIZED =====\n" );
    PER_LOG_INFO( "Firmware: SMTC LoRa RAC Library\n" );
    PER_LOG_INFO( "Application: Packet Error Rate (PER) Measurement\n" );

    if( IS_TRANSMITTER )
    {
        PER_LOG_CONFIG( "Device Role: TRANSMITTER\n" );
        PER_LOG_CONFIG( "Mode: Continuous packet transmission for PER testing\n" );
    }
    else
    {
        PER_LOG_CONFIG( "Device Role: RECEIVER\n" );
        PER_LOG_CONFIG( "Mode: Continuous packet reception for PER measurement\n" );
    }

    memset( &packet_error_rate, 0, sizeof( packet_error_rate ) );

    // initialize radio access
    PER_LOG_INFO( "Initializing radio access with HIGH priority...\n" );
    packet_error_rate.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );
    PER_LOG_INFO( "Radio access ID: %u\n", packet_error_rate.radio_access_id );

    // get transaction context pointer
    packet_error_rate.transaction = smtc_rac_get_context( packet_error_rate.radio_access_id );

    PER_LOG_INFO( "Transaction context obtained successfully\n" );

    // configure transaction context
    packet_error_rate.transaction->scheduler_config.callback_post_radio_transaction = post_transaction_callback;
    packet_error_rate.transaction->scheduler_config.callback_pre_radio_transaction  = pre_transaction_callback;
    packet_error_rate.transaction->scheduler_config.start_time_ms                   = 0;  // set at each transaction
    packet_error_rate.transaction->scheduler_config.scheduling                      = SMTC_RAC_ASAP_TRANSACTION;

    packet_error_rate.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer = packet_error_rate.payload;
    packet_error_rate.transaction->smtc_rac_data_buffer_setup.rx_payload_buffer = packet_error_rate.payload;
    packet_error_rate.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer =
        sizeof( packet_error_rate.payload );
    packet_error_rate.transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer =
        sizeof( packet_error_rate.payload );

    packet_error_rate.transaction->modulation_type                        = SMTC_RAC_MODULATION_LORA;
    packet_error_rate.transaction->radio_params.lora.is_tx                = IS_TRANSMITTER;
    packet_error_rate.transaction->radio_params.lora.is_ranging_exchange  = false;
    packet_error_rate.transaction->radio_params.lora.frequency_in_hz      = RF_FREQ_IN_HZ;
    packet_error_rate.transaction->radio_params.lora.tx_power_in_dbm      = TX_OUTPUT_POWER_DBM;
    packet_error_rate.transaction->radio_params.lora.sf                   = LORA_SPREADING_FACTOR;
    packet_error_rate.transaction->radio_params.lora.bw                   = LORA_BANDWIDTH;
    packet_error_rate.transaction->radio_params.lora.cr                   = LORA_CODING_RATE;
    packet_error_rate.transaction->radio_params.lora.preamble_len_in_symb = LORA_PREAMBLE_LENGTH;
    packet_error_rate.transaction->radio_params.lora.header_type          = LORA_PKT_LEN_MODE;
    packet_error_rate.transaction->radio_params.lora.invert_iq_is_on      = LORA_IQ;
    packet_error_rate.transaction->radio_params.lora.crc_is_on            = LORA_CRC;
    packet_error_rate.transaction->radio_params.lora.sync_word            = LORA_SYNCWORD;
    packet_error_rate.transaction->radio_params.lora.tx_size              = PAYLOAD_SIZE;
    packet_error_rate.transaction->radio_params.lora.max_rx_size          = sizeof( packet_error_rate.payload );
    packet_error_rate.transaction->radio_params.lora.rx_timeout_ms        = RX_TIMEOUT;

    packet_error_rate.transaction->lbt_context.lbt_enabled        = true;
    packet_error_rate.transaction->lbt_context.listen_duration_ms = LBT_SNIFF_DURATION_MS_DEFAULT;
    packet_error_rate.transaction->lbt_context.threshold_dbm      = LBT_THRESHOLD_DBM_DEFAULT;
    packet_error_rate.transaction->lbt_context.bandwidth_hz       = LBT_BW_HZ__DEFAULT;
    // Log radio configuration
    PER_LOG_CONFIG( "===== RADIO CONFIGURATION =====\n" );
    PER_LOG_CONFIG( "Modulation: LoRa\n" );
    PER_LOG_CONFIG( "Frequency: %lu Hz (%.1f MHz)\n", RF_FREQ_IN_HZ, ( float ) RF_FREQ_IN_HZ / 1000000.0f );
    PER_LOG_CONFIG( "TX Power: %d dBm\n", TX_OUTPUT_POWER_DBM );
    PER_LOG_CONFIG( "Spreading Factor: SF%d\n", LORA_SPREADING_FACTOR );
    PER_LOG_CONFIG( "Bandwidth: %d kHz\n", LORA_BANDWIDTH == 0 ? 125 : ( LORA_BANDWIDTH == 1 ? 250 : 500 ) );
    PER_LOG_CONFIG( "Coding Rate: 4/%d\n", LORA_CODING_RATE + 5 );
    PER_LOG_CONFIG( "Preamble Length: %d symbols\n", LORA_PREAMBLE_LENGTH );
    PER_LOG_CONFIG( "Header Type: %s\n", LORA_PKT_LEN_MODE ? "Explicit" : "Implicit" );
    PER_LOG_CONFIG( "CRC: %s\n", LORA_CRC ? "Enabled" : "Disabled" );
    PER_LOG_CONFIG( "IQ Inversion: %s\n", LORA_IQ ? "Enabled" : "Disabled" );
    PER_LOG_CONFIG( "Sync Word: 0x%02X\n", LORA_SYNCWORD );
    if( !IS_TRANSMITTER )
    {
        PER_LOG_CONFIG( "RX Timeout: %lu ms\n", RX_TIMEOUT );
    }
    PER_LOG_CONFIG( "Payload Size: %d bytes\n", PAYLOAD_SIZE );
    PER_LOG_CONFIG( "Max Exchanges: %ld\n", MAX_EXCHANGE_COUNT );
    PER_LOG_CONFIG( "================================\n" );

    // start application
    PER_LOG_INFO( "Starting PER test sequence...\n" );
    start_new_transaction( 0 );
}

void per_on_button_press( void )
{
    // nothing to do
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void pre_transaction_callback( void )
{
    if( IS_TRANSMITTER )
    {
        set_led( SMTC_PF_LED_TX, true );
    }
    else
    {
        set_led( SMTC_PF_LED_RX, true );
    }
}

static void post_transaction_callback( rp_status_t status )
{
    bool payload_is_correct = true;
    // will be used to hold the received payload data
    uint8_t  header_received[HEADER_SIZE] = { 0 };
    uint8_t  separator_received           = 0;
    uint32_t counter_received             = 0;

    set_led( SMTC_PF_LED_TX, false );
    set_led( SMTC_PF_LED_RX, false );

    PER_LOG_INFO( "===== Transaction #%ld Complete =====\n", packet_error_rate.exchange_count + 1 );

    switch( status )
    {
    case RP_STATUS_TX_DONE:
    {
        PER_LOG_TX( "Transmission completed successfully\n" );
        PER_LOG_STATS( "TX Progress: %ld/%ld packets sent\n", packet_error_rate.exchange_count + 1,
                       MAX_EXCHANGE_COUNT );
        break;
    }

    case RP_STATUS_RX_PACKET:
    {
        PER_LOG_RX( "Packet received successfully\n" );
        PER_LOG_RX( "Packet size: %d bytes, RSSI: %d dBm, SNR: %d dB\n",
                    packet_error_rate.transaction->smtc_rac_data_result.rx_size,
                    packet_error_rate.transaction->smtc_rac_data_result.rssi_result,
                    packet_error_rate.transaction->smtc_rac_data_result.snr_result );

        packet_error_rate.exchange_started = true;
        if( packet_error_rate.transaction->smtc_rac_data_result.rx_size == PAYLOAD_SIZE )
        {
            ( void ) memcpy( header_received, packet_error_rate.payload + HEADER_OFFSET, HEADER_SIZE );
            ( void ) memcpy( &separator_received, packet_error_rate.payload + SEPARATOR_OFFSET, SEPARATOR_SIZE );
            ( void ) memcpy( &counter_received, packet_error_rate.payload + COUNTER_OFFSET, COUNTER_SIZE );

            payload_is_correct &= ( memcmp( header_received, HEADER, HEADER_SIZE ) == 0 );
            payload_is_correct &= ( separator_received == 0 );
        }
        else
        {
            payload_is_correct = false;
        }

        if( payload_is_correct == false )
        {
            PER_LOG_ERROR( "Corrupted payload detected - header or separator invalid\n" );
            PER_LOG_ERROR( "Expected header: '%.3s', received: '%.3s'\n", HEADER, header_received );
            PER_LOG_ERROR( "Expected separator: 0, received: %d\n", separator_received );
            packet_error_rate.failure_count += 1;
            break;
        }

        PER_LOG_RX( "Valid packet received with counter: %ld\n", counter_received );
        if( counter_received == packet_error_rate.exchange_count )
        {
            PER_LOG_STATS( "RX Progress: %ld/%ld packets received successfully\n", packet_error_rate.exchange_count + 1,
                           MAX_EXCHANGE_COUNT );
            break;
        }

        PER_LOG_ERROR( "Packet sequence error - expected: %ld, received: %ld\n", packet_error_rate.exchange_count,
                       counter_received );

        // Count missed packets
        uint32_t missed_packets = 0;
        while( packet_error_rate.exchange_count < counter_received )
        {
            packet_error_rate.exchange_count += 1;
            packet_error_rate.failure_count += 1;
            missed_packets++;
        }
        PER_LOG_WARN( "Detected %ld missed packet(s)\n", missed_packets );
        break;
    }

    default:
    {
        PER_LOG_ERROR( "Transaction failed with status: %s\n", rp_status_to_str( status ) );
        if( packet_error_rate.exchange_started == true )
        {
            packet_error_rate.failure_count += 1;
            PER_LOG_ERROR( "Failure counted (total failures: %ld)\n", packet_error_rate.failure_count );
        }
        else
        {
            PER_LOG_WARN( "Failure before exchange started - not counted in PER\n" );
        }
        break;
    }
    }

    packet_error_rate.exchange_count += 1;

    // Display current statistics after each transaction
    PER_LOG_STATS( "Current Progress: %ld/%ld | Failures: %ld | PER: %.2f%% | Success Rate: %.2f%%\n",
                   packet_error_rate.exchange_count, MAX_EXCHANGE_COUNT, packet_error_rate.failure_count,
                   CALCULATE_PER_PERCENT( ), CALCULATE_SUCCESS_RATE( ) );

    if( packet_error_rate.exchange_count < MAX_EXCHANGE_COUNT )
    {
        start_new_transaction( INTER_EXCHANGE_DELAY );
        return;
    }

    // Series completed - display final results
    PER_LOG_INFO( "===== PER TEST SERIES COMPLETED =====\n" );
    PER_LOG_STATS( "Final Results:\n" );
    PER_LOG_STATS( "  Total Packets: %ld\n", packet_error_rate.exchange_count );
    PER_LOG_STATS( "  Failed Packets: %ld\n", packet_error_rate.failure_count );
    PER_LOG_STATS( "  Successful Packets: %ld\n", packet_error_rate.exchange_count - packet_error_rate.failure_count );
    PER_LOG_STATS( "  Packet Error Rate (PER): %.4f%% (%ld/%ld)\n", CALCULATE_PER_PERCENT( ),
                   packet_error_rate.failure_count, packet_error_rate.exchange_count );
    PER_LOG_STATS( "  Success Rate: %.4f%%\n", CALCULATE_SUCCESS_RATE( ) );
    PER_LOG_INFO( "=====================================\n" );

    // reset for next series
    packet_error_rate.exchange_count   = 0;
    packet_error_rate.exchange_started = false;
    packet_error_rate.failure_count    = 0;

    // wait before starting a new series of exchanges
    PER_LOG_INFO( "Waiting %ld ms before starting new test series...\n", INTER_SERIES_DELAY );
    start_new_transaction( INTER_SERIES_DELAY );
}

static void start_new_transaction( uint32_t delay )
{
    uint32_t start_time = smtc_modem_hal_get_time_in_ms( ) + delay;

    if( delay > 0 )
    {
        PER_LOG_INFO( "Scheduling next transaction in %ld ms (at %lu ms)\n", delay, start_time );
    }

    PER_LOG_INFO( "Preparing transaction #%ld\n", packet_error_rate.exchange_count + 1 );

    if( IS_TRANSMITTER )
    {
        // Prepare TX payload
        ( void ) memcpy( packet_error_rate.payload + HEADER_OFFSET, HEADER, HEADER_SIZE );
        ( void ) memset( packet_error_rate.payload + SEPARATOR_OFFSET, 0, SEPARATOR_SIZE );
        ( void ) memcpy( packet_error_rate.payload + COUNTER_OFFSET, &( packet_error_rate.exchange_count ),
                         COUNTER_SIZE );

        PER_LOG_TX( "TX payload prepared: '%.3s'[0]%ld (%d bytes)\n", HEADER, packet_error_rate.exchange_count,
                    PAYLOAD_SIZE );
    }
    else
    {
        // Clear RX buffer
        ( void ) memset( packet_error_rate.payload, 0, PAYLOAD_SIZE );
        PER_LOG_RX( "RX buffer cleared, ready to receive\n" );
    }

    packet_error_rate.transaction->scheduler_config.start_time_ms = start_time;

    smtc_rac_return_code_t error_code =
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( packet_error_rate.radio_access_id ) );

    if( error_code == SMTC_RAC_SUCCESS )
    {
        PER_LOG_INFO( "Transaction scheduled successfully\n" );
    }
    else
    {
        PER_LOG_ERROR( "Failed to schedule transaction (error: %d)\n", error_code );
    }

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( error_code == SMTC_RAC_SUCCESS );
}

static const char* rp_status_to_str( const rp_status_t status )
{
    switch( status )
    {
    case RP_STATUS_RX_CRC_ERROR:
        return "RP_STATUS_RX_CRC_ERROR";
    case RP_STATUS_CAD_POSITIVE:
        return "RP_STATUS_CAD_POSITIVE";
    case RP_STATUS_CAD_NEGATIVE:
        return "RP_STATUS_CAD_NEGATIVE";
    case RP_STATUS_TX_DONE:
        return "RP_STATUS_TX_DONE";
    case RP_STATUS_RX_PACKET:
        return "RP_STATUS_RX_PACKET";
    case RP_STATUS_RX_TIMEOUT:
        return "RP_STATUS_RX_TIMEOUT";
    case RP_STATUS_LBT_FREE_CHANNEL:
        return "RP_STATUS_LBT_FREE_CHANNEL";
    case RP_STATUS_LBT_BUSY_CHANNEL:
        return "RP_STATUS_LBT_BUSY_CHANNEL";
    case RP_STATUS_WIFI_SCAN_DONE:
        return "RP_STATUS_WIFI_SCAN_DONE";
    case RP_STATUS_GNSS_SCAN_DONE:
        return "RP_STATUS_GNSS_SCAN_DONE";
    case RP_STATUS_TASK_ABORTED:
        return "RP_STATUS_TASK_ABORTED";
    case RP_STATUS_TASK_INIT:
        return "RP_STATUS_TASK_INIT";
    case RP_STATUS_LR_FHSS_HOP:
        return "RP_STATUS_LR_FHSS_HOP";
    case RP_STATUS_RTTOF_REQ_DISCARDED:
        return "RP_STATUS_RTTOF_REQ_DISCARDED";
    case RP_STATUS_RTTOF_RESP_DONE:
        return "RP_STATUS_RTTOF_RESP_DONE";
    case RP_STATUS_RTTOF_EXCH_VALID:
        return "RP_STATUS_RTTOF_EXCH_VALID";
    case RP_STATUS_RTTOF_TIMEOUT:
        return "RP_STATUS_RTTOF_TIMEOUT";
    }
    return "UNKNOW_RP_STATUS";
}
