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
#define PER_LOG_ARRAY( ... ) SMTC_HAL_TRACE_ARRAY( __VA_ARGS__ )

// Helper macros for calculations (integer-only, no float)
#define CALCULATE_PER_PERCENT( )                                                             \
    ( ( packet_error_rate.exchange_count > 0 )                                               \
          ? ( ( packet_error_rate.failure_count * 100 ) / packet_error_rate.exchange_count ) \
          : 0 )
#define CALCULATE_SUCCESS_RATE( )                                                                \
    ( ( packet_error_rate.exchange_count > 0 )                                                   \
          ? ( ( ( packet_error_rate.exchange_count - packet_error_rate.failure_count ) * 100 ) / \
              packet_error_rate.exchange_count )                                                 \
          : 0 )

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
#define LBT_THRESHOLD_DBM_DEFAULT ( int16_t )( -80 )
#define LBT_BW_HZ__DEFAULT ( 200000 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if( ROLE == RECEIVER )
static const bool     IS_TRANSMITTER       = false;
static const uint32_t INTER_EXCHANGE_DELAY = 0;  // ms
#elif( ROLE == TRANSMITTER )
static const bool     IS_TRANSMITTER       = true;
static const uint32_t INTER_EXCHANGE_DELAY = 300;  // ms
#else
#error "Please define ROLE as either RECEIVER or TRANSMITTER"
#endif

static const uint32_t RX_TIMEOUT = 30000;  // ms

static const uint32_t INTER_SERIES_DELAY_RECEIVER    = 0;                  // ms
static const uint32_t INTER_SERIES_DELAY_TRANSMITTER = 10000;              // ms
static const uint32_t MAX_EXCHANGE_COUNT             = 100;                // (no unit)
static const uint8_t  HEADER[HEADER_SIZE]            = { 'P', 'E', 'R' };  // bytes

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
    uint8_t             radio_access_id;        // store the result of `smtc_rac_open_radio`
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

static void        pre_transaction_callback( void );
static void        post_transaction_callback( rp_status_t status );
static void        start_new_transaction( uint32_t delay );
static const char* rp_status_to_str( const rp_status_t status );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void per_init( void )
{
    PER_LOG_INFO( "========================================\n" );
    PER_LOG_INFO( "     PACKET ERROR RATE (PER) TEST\n" );
    PER_LOG_INFO( "========================================\n" );
    PER_LOG_INFO( "Role: %s\n", IS_TRANSMITTER ? "TRANSMITTER" : "RECEIVER" );

    memset( &packet_error_rate, 0, sizeof( packet_error_rate ) );

    // initialize radio access
    packet_error_rate.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );

    // get transaction context pointer
    packet_error_rate.transaction = smtc_rac_get_context( packet_error_rate.radio_access_id );

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
    PER_LOG_CONFIG( "Frequency: %lu Hz (%lu MHz)\n", RF_FREQ_IN_HZ, RF_FREQ_IN_HZ / 1000000 );
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
        PER_LOG_CONFIG( "RX Timeout: %" PRIu32 " ms\n", RX_TIMEOUT );
    }
    PER_LOG_CONFIG( "Payload Size: %d bytes\n", PAYLOAD_SIZE );
    PER_LOG_CONFIG( "Max Exchanges: %" PRIu32 "\n", MAX_EXCHANGE_COUNT );
    PER_LOG_CONFIG( "================================\n" );

    // start application
    PER_LOG_INFO( "Starting PER test sequence...\n" );
    start_new_transaction( 500 );
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

    switch( status )
    {
    case RP_STATUS_TX_DONE:
    {
        PER_LOG_TX( "Transmission completed successfully\n" );

        uint32_t tx_counter = 0;
        memcpy( &tx_counter,
                &packet_error_rate.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer[COUNTER_OFFSET],
                COUNTER_SIZE );
        PER_LOG_INFO( "  Counter: %" PRIu32 "\n", tx_counter );

        PER_LOG_STATS( "TX Progress: %" PRIu32 " packets sent\n", packet_error_rate.exchange_count );
        break;
    }

    case RP_STATUS_RX_PACKET:
    {
        PER_LOG_RX( "Packet received successfully\n" );
        PER_LOG_RX( "RSSI: %d dBm, SNR: %d dB\n", packet_error_rate.transaction->smtc_rac_data_result.rssi_result,
                    packet_error_rate.transaction->smtc_rac_data_result.snr_result );

        packet_error_rate.exchange_started = true;
        if( packet_error_rate.transaction->smtc_rac_data_result.rx_size == PAYLOAD_SIZE )
        {
            ( void ) memcpy( header_received, packet_error_rate.payload + HEADER_OFFSET, HEADER_SIZE );
            ( void ) memcpy( &separator_received, packet_error_rate.payload + SEPARATOR_OFFSET, SEPARATOR_SIZE );
            ( void ) memcpy( &counter_received, packet_error_rate.payload + COUNTER_OFFSET, COUNTER_SIZE );

            PER_LOG_INFO( "  Counter received: %" PRIu32 " (expected: %" PRIu32 ")\n", counter_received,
                          packet_error_rate.exchange_count );

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
            PER_LOG_ARRAY( "Expected header", HEADER, HEADER_SIZE );
            PER_LOG_ARRAY( "Received header", header_received, HEADER_SIZE );
            PER_LOG_ERROR( "Expected separator: 0, received: %d\n", separator_received );
            packet_error_rate.failure_count += 1;
            break;
        }

        if( counter_received == packet_error_rate.exchange_count )
        {
            PER_LOG_STATS( "RX Progress: %" PRIu32 " packets received successfully\n",
                           packet_error_rate.exchange_count );
            break;
        }

        // Count missed packets
        uint32_t missed_packets = 0;
        while( packet_error_rate.exchange_count < counter_received )
        {
            packet_error_rate.exchange_count += 1;
            packet_error_rate.failure_count += 1;
            missed_packets++;
        }
        PER_LOG_WARN( "Lost packets detected: expected counter %" PRIu32 ", received %" PRIu32 " (%" PRIu32
                      " packets lost)\n",
                      packet_error_rate.exchange_count - missed_packets, counter_received, missed_packets );
        PER_LOG_STATS( "Packet loss: %" PRIu32 " consecutive packets lost\n", missed_packets );
        break;
    }

    default:
    {
        PER_LOG_ERROR( "Transaction failed with status: %s\n", rp_status_to_str( status ) );
        if( packet_error_rate.exchange_started == true )
        {
            packet_error_rate.failure_count += 1;
        }
        else
        {
            PER_LOG_INFO( "Waiting for first packet...\n" );
        }
        break;
    }
    }

    packet_error_rate.exchange_count += 1;

    // Display real-time PER statistics
    if( packet_error_rate.exchange_started )
    {
        PER_LOG_STATS( "RT: Total Pkts: %" PRIu32 ", Failed Pkts: %" PRIu32 "\n", packet_error_rate.exchange_count,
                       packet_error_rate.failure_count );
    }

    if( packet_error_rate.exchange_count < MAX_EXCHANGE_COUNT )
    {
        start_new_transaction( INTER_EXCHANGE_DELAY );
        return;
    }

    // Series completed - display final results
    uint32_t final_per          = CALCULATE_PER_PERCENT( );
    uint32_t final_success_rate = CALCULATE_SUCCESS_RATE( );

    PER_LOG_INFO( "========================================\n" );
    PER_LOG_INFO( "       PER TEST COMPLETED!\n" );
    PER_LOG_INFO( "========================================\n" );
    PER_LOG_STATS( "FINAL PER RESULTS:\n" );
    PER_LOG_STATS( "Total Packets Processed: %" PRIu32 "\n", packet_error_rate.exchange_count );
    PER_LOG_STATS( "Successful Packets: %" PRIu32 "\n",
                   packet_error_rate.exchange_count - packet_error_rate.failure_count );
    PER_LOG_STATS( "Failed Packets: %" PRIu32 "\n", packet_error_rate.failure_count );
    PER_LOG_STATS( "Final Packet Error Rate: %" PRIu32 "%%\n", final_per );
    PER_LOG_STATS( "Final Success Rate: %" PRIu32 "%%\n", final_success_rate );

    if( final_per < 1 )
    {
        PER_LOG_INFO( "Excellent link quality!\n" );
    }
    else if( final_per < 5 )
    {
        PER_LOG_INFO( "Good link quality.\n" );
    }
    else if( final_per < 10 )
    {
        PER_LOG_WARN( "Moderate link quality.\n" );
    }
    else
    {
        PER_LOG_WARN( "Poor link quality!\n" );
    }
    PER_LOG_INFO( "========================================\n" );

    // reset for next series
    packet_error_rate.exchange_count   = 0;
    packet_error_rate.exchange_started = false;
    packet_error_rate.failure_count    = 0;

    PER_LOG_INFO( "*** RESTARTING NEW PER SERIES ***\n" );
    start_new_transaction( IS_TRANSMITTER ? INTER_SERIES_DELAY_TRANSMITTER : INTER_SERIES_DELAY_RECEIVER );
}

static void start_new_transaction( uint32_t delay )
{
    uint32_t start_time = 0;
    if( packet_error_rate.transaction->smtc_rac_data_result.radio_start_timestamp_ms != 0 )
    {
        start_time = packet_error_rate.transaction->smtc_rac_data_result.radio_start_timestamp_ms + delay;
    }
    else
    {
        start_time = smtc_modem_hal_get_time_in_ms( ) + delay;
    }

    if( IS_TRANSMITTER )
    {
        PER_LOG_TX( "===== Starting transmission #%" PRIu32 "\n", packet_error_rate.exchange_count );

        // Prepare TX payload
        ( void ) memcpy( packet_error_rate.payload + HEADER_OFFSET, HEADER, HEADER_SIZE );
        ( void ) memset( packet_error_rate.payload + SEPARATOR_OFFSET, 0, SEPARATOR_SIZE );
        ( void ) memcpy( packet_error_rate.payload + COUNTER_OFFSET, &( packet_error_rate.exchange_count ),
                         COUNTER_SIZE );
    }
    else
    {
        PER_LOG_RX( "===== Starting reception #%" PRIu32 "\n", packet_error_rate.exchange_count );

        // Clear RX buffer
        ( void ) memset( packet_error_rate.payload, 0, PAYLOAD_SIZE );
    }

    packet_error_rate.transaction->scheduler_config.start_time_ms = start_time;

    smtc_rac_return_code_t error_code =
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( packet_error_rate.radio_access_id ) );

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
    case RP_STATUS_RADIO_LOCKED:
        return "RP_STATUS_RADIO_LOCKED";
    case RP_STATUS_RADIO_UNLOCKED:
        return "RP_STATUS_RADIO_UNLOCKED";
    }
    return "UNKNOW_RP_STATUS";
}
