/**
 * @file      app_per_flrc.c
 *
 * @brief     Simple PER (Packet Error Rate) example with FLRC modulation
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

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "app_per_flrc.h"
#include "main_per_flrc.h"

#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

#define RAC_LOG_APP_PREFIX "PER-FLRC"
#include "smtc_rac_log.h"
#include "smtc_hal_dbg_trace.h"

#define RECEIVER 1
#define TRANSMITTER 2

#define RX_BUFFER_MAX_SIZE ( 255 )

#define HEADER_SIZE ( 4 )
#define SEPARATOR_SIZE ( 1 )
#define COUNTER_SIZE ( 4 )
#define TX_BUFFER_MAX_SIZE ( 255 )  //( HEADER_SIZE + SEPARATOR_SIZE + COUNTER_SIZE )

#define HEADER_OFFSET ( 0 )
#define SEPARATOR_OFFSET ( HEADER_OFFSET + HEADER_SIZE )
#define COUNTER_OFFSET ( SEPARATOR_OFFSET + SEPARATOR_SIZE )

#if( ROLE == RECEIVER )
static const bool IS_TRANSMITTER = false;
#elif( ROLE == TRANSMITTER )
static const bool IS_TRANSMITTER = true;
#else
#error "Please define ROLE as either RECEIVER or TRANSMITTER"
#endif

static const uint8_t  HEADER[HEADER_SIZE]  = { 'F', 'L', 'R', 'C' };        // bytes
static const uint32_t INTER_EXCHANGE_DELAY = 200;                           // ms
static const uint32_t RX_TIMEOUT           = ( INTER_EXCHANGE_DELAY * 2 );  // ms
static const uint32_t INTER_SERIES_DELAY   = 10000;                         // ms
static const uint32_t MAX_EXCHANGE_COUNT   = 100;

// Helper macros for calculations (integer-only, no float)
#define CALCULATE_PER_PERCENT( ) \
    ( ( flrc.exchange_count > 0 ) ? ( ( flrc.payload_failure_count * 100 ) / flrc.exchange_count ) : 0 )
#define CALCULATE_SUCCESS_RATE( )                                                                    \
    ( ( flrc.exchange_count > 0 )                                                                    \
          ? ( ( ( flrc.exchange_count - flrc.payload_failure_count ) * 100 ) / flrc.exchange_count ) \
          : 0 )

typedef struct packet_error_rate_flrc_s
{
    uint32_t            exchange_count;
    uint32_t            packet_counter;  // counter for TX packets (separate from exchange_count)
    bool                exchange_started;
    uint32_t            payload_failure_count;  // number of failed receptions after the exchange has started
    uint32_t            crc_failure_count;
    uint8_t             tx_payload[TX_BUFFER_MAX_SIZE];
    uint8_t             rx_payload[RX_BUFFER_MAX_SIZE];
    uint8_t             radio_access_id;
    smtc_rac_context_t* transaction;
} packet_error_rate_flrc_t;

static packet_error_rate_flrc_t flrc = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void pre_transaction_callback( void );
static void unified_transaction_callback( rp_status_t status );
static void start_new_transaction( uint32_t rtc, uint32_t delay );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void per_flrc_init( void )
{
    SMTC_HAL_TRACE_INFO( "========================================\n" );
    SMTC_HAL_TRACE_INFO( "     FLRC PACKET ERROR RATE (PER) TEST\n" );
    SMTC_HAL_TRACE_INFO( "========================================\n" );
    SMTC_HAL_TRACE_INFO( "Role: %s\n", IS_TRANSMITTER ? "TRANSMITTER" : "RECEIVER" );

    flrc.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );

    flrc.transaction                                                   = smtc_rac_get_context( flrc.radio_access_id );
    flrc.transaction->scheduler_config.callback_post_radio_transaction = unified_transaction_callback;
    // Configure rac FLRC context
    flrc.transaction->modulation_type = SMTC_RAC_MODULATION_FLRC;

    flrc.transaction->scheduler_config.callback_pre_radio_transaction = pre_transaction_callback;
    flrc.transaction->scheduler_config.scheduling                     = SMTC_RAC_SCHEDULED_TRANSACTION;

    flrc.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = flrc.tx_payload;
    flrc.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( flrc.tx_payload );
    flrc.transaction->radio_params.flrc.tx_size                            = TX_BUFFER_MAX_SIZE;
    flrc.transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = flrc.rx_payload;
    flrc.transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = sizeof( flrc.rx_payload );
    flrc.transaction->radio_params.flrc.max_rx_size                        = RX_BUFFER_MAX_SIZE;

    // Buffer and size are set per-transaction in start_new_transaction()

    flrc.transaction->radio_params.flrc.is_tx                = IS_TRANSMITTER;
    flrc.transaction->radio_params.flrc.frequency_in_hz      = RF_FREQ_IN_HZ;
    flrc.transaction->radio_params.flrc.tx_power_in_dbm      = TX_OUTPUT_POWER_DBM;
    flrc.transaction->radio_params.flrc.br_in_bps            = FLRC_BR_BPS;
    flrc.transaction->radio_params.flrc.bw_dsb_in_hz         = FLRC_BW_HZ;
    flrc.transaction->radio_params.flrc.cr                   = FLRC_CR;
    flrc.transaction->radio_params.flrc.pulse_shape          = FLRC_PULSE_SHAPE;
    flrc.transaction->radio_params.flrc.preamble_len_in_bits = FLRC_PREAMBLE_BITS;
    flrc.transaction->radio_params.flrc.sync_word_len        = FLRC_SYNCWORD_LEN;
    flrc.transaction->radio_params.flrc.tx_syncword          = FLRC_TX_SYNCWORD;
    flrc.transaction->radio_params.flrc.match_sync_word      = FLRC_MATCH_SYNCWORD;
    flrc.transaction->radio_params.flrc.pld_is_fix           = FLRC_PLD_IS_FIX;

    static const uint8_t default_syncword[4]           = { 0x90, 0x56, 0x34, 0x12 };
    flrc.transaction->radio_params.flrc.sync_word      = default_syncword;
    flrc.transaction->radio_params.flrc.crc_type       = FLRC_CRC;
    flrc.transaction->radio_params.flrc.crc_seed       = 0x00000000;  // Left default value as is
    flrc.transaction->radio_params.flrc.crc_polynomial = 0x00000000;  // Left default value as is
    flrc.transaction->radio_params.flrc.rx_timeout_ms  = RX_TIMEOUT;

    // Log radio configuration
    SMTC_HAL_TRACE_INFO( "CONFIG ===== RADIO CONFIGURATION =====\n" );
    SMTC_HAL_TRACE_INFO( "CONFIG Modulation: FLRC\n" );
    SMTC_HAL_TRACE_INFO( "CONFIG Frequency: %u Hz (%u MHz)\n", RF_FREQ_IN_HZ, RF_FREQ_IN_HZ / 1000000 );
    SMTC_HAL_TRACE_INFO( "CONFIG TX Power: %d dBm\n", TX_OUTPUT_POWER_DBM );
    SMTC_HAL_TRACE_INFO( "CONFIG Bitrate: %u bps (%u kbps)\n", FLRC_BR_BPS, FLRC_BR_BPS / 1000 );
    SMTC_HAL_TRACE_INFO( "CONFIG Bandwidth: %u Hz (%u kHz)\n", FLRC_BW_HZ, FLRC_BW_HZ / 1000 );
    SMTC_HAL_TRACE_INFO( "CONFIG Coding Rate: %d\n", FLRC_CR );
    SMTC_HAL_TRACE_INFO( "CONFIG Preamble Length: %d bits\n", FLRC_PREAMBLE_BITS );
    if( !IS_TRANSMITTER )
    {
        SMTC_HAL_TRACE_INFO( "CONFIG RX Timeout: %" PRIu32 " ms\n", RX_TIMEOUT );
    }
    SMTC_HAL_TRACE_INFO( "CONFIG Payload Size: %d bytes\n", TX_BUFFER_MAX_SIZE );
    SMTC_HAL_TRACE_INFO( "CONFIG Max Exchanges: %" PRIu32 "\n", MAX_EXCHANGE_COUNT );
    SMTC_HAL_TRACE_INFO( "CONFIG ================================\n" );

    // start application
    SMTC_HAL_TRACE_INFO( "Starting FLRC PER test sequence...\n" );
    start_new_transaction( smtc_modem_hal_get_time_in_ms( ), 150 );
}

void per_flrc_on_button_press( void )
{
    // nothing
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void unified_transaction_callback( rp_status_t status )
{
    bool     payload_is_correct           = true;
    uint8_t  header_received[HEADER_SIZE] = { 0 };
    uint8_t  separator_received           = 0;
    uint32_t counter_received             = 0;
    uint32_t delay_ms                     = 10;
    uint32_t tmp_delayed_cnt              = 1;
    int32_t  add_delay_ms                 = 0;

    set_led( SMTC_PF_LED_TX, false );
    set_led( SMTC_PF_LED_RX, false );

    switch( status )
    {
    case RP_STATUS_TX_DONE:
    {
        // increment counters after successful transmission
        flrc.exchange_count += 1;
        flrc.packet_counter += 1;  // increment packet counter for next TX
        SMTC_HAL_TRACE_INFO( "TX Transmission completed successfully\n" );

        // Debug: show transmitted packet details
        uint32_t tx_counter = 0;
        memcpy( &tx_counter, &flrc.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer[COUNTER_OFFSET],
                COUNTER_SIZE );
        SMTC_HAL_TRACE_INFO( "Counter: %" PRIu32 "\n", tx_counter );

        SMTC_HAL_TRACE_INFO(
            "STATS "
            "TX Progress: %" PRIu32 " packets sent\n",
            flrc.exchange_count );
        break;
    }
    case RP_STATUS_RX_PACKET:
    {
        SMTC_HAL_TRACE_INFO( "RX FLRC packet received successfully\n" );
        SMTC_HAL_TRACE_INFO( "RX RSSI: %d dBm\n", flrc.transaction->smtc_rac_data_result.rssi_result );

        flrc.exchange_started = true;
        ( void ) memcpy( header_received, flrc.rx_payload + HEADER_OFFSET, HEADER_SIZE );
        ( void ) memcpy( &separator_received, flrc.rx_payload + SEPARATOR_OFFSET, SEPARATOR_SIZE );
        ( void ) memcpy( &counter_received, flrc.rx_payload + COUNTER_OFFSET, COUNTER_SIZE );

        // Debug: show parsed packet structure
        SMTC_HAL_TRACE_INFO( "  Counter received: %" PRIu32 " (expected: %" PRIu32 ")\n", counter_received,
                             flrc.exchange_count );

        payload_is_correct &= ( memcmp( header_received, HEADER, HEADER_SIZE ) == 0 );
        payload_is_correct &= ( separator_received == 0 );
        if( payload_is_correct == false )
        {
            SMTC_HAL_TRACE_ERROR( "Corrupted payload detected - header or separator invalid\n" );
            SMTC_HAL_TRACE_ARRAY( "Header expected", HEADER, HEADER_SIZE );
            SMTC_HAL_TRACE_ARRAY( "Header received", header_received, HEADER_SIZE );
            SMTC_HAL_TRACE_ERROR( "Expected separator: 0, received: %d\n", separator_received );
            flrc.payload_failure_count += 1;
            flrc.exchange_count += 1;  // Increment even for corrupted packets
            break;
        }

        add_delay_ms = ( INTER_EXCHANGE_DELAY / 10 );

        // Handle packet sequence: sync with received counter + detect lost packets
        uint32_t expected_counter   = flrc.exchange_count;
        uint32_t new_exchange_count = counter_received + 1;  // counter starts at 0, exchange_count at 1

        if( counter_received < expected_counter )
        {
            SMTC_HAL_TRACE_WARNING( "Out-of-order packet: received counter %" PRIu32 ", expected %" PRIu32 "\n",
                                    counter_received, expected_counter );
            // Don't update exchange_count for out-of-order packets
            flrc.payload_failure_count += 1;
        }
        else if( counter_received > expected_counter )
        {
            // Lost packets detected - update failure count
            uint32_t lost_packets = counter_received - expected_counter;
            flrc.payload_failure_count += lost_packets;
            flrc.exchange_count = new_exchange_count;

            SMTC_HAL_TRACE_WARNING( "Lost packets detected: expected counter %" PRIu32 ", received %" PRIu32
                                    " (%" PRIu32 " packets lost)\n",
                                    expected_counter, counter_received, lost_packets );
            SMTC_HAL_TRACE_INFO(
                "STATS "
                "Packet loss: %" PRIu32 " consecutive packets lost\n",
                lost_packets );
        }
        else
        {
            // Perfect sequence - update normally
            flrc.exchange_count = new_exchange_count;
            SMTC_HAL_TRACE_INFO( "Perfect sequence: counter %" PRIu32 " as expected\n", counter_received );
        }

        SMTC_HAL_TRACE_INFO(
            "STATS "
            "RX Progress: %" PRIu32 " FLRC packets received successfully\n",
            flrc.exchange_count - flrc.payload_failure_count );
        break;
    }
    case RP_STATUS_RX_TIMEOUT:
    {
        SMTC_HAL_TRACE_WARNING( "FLRC reception timeout after %" PRIu32 " ms\n", RX_TIMEOUT );
        SMTC_HAL_TRACE_INFO( "=== RX TIMEOUT DEBUG ===\n" );
        SMTC_HAL_TRACE_INFO( "Expected packet #%" PRIu32 "\n", flrc.exchange_count );
        SMTC_HAL_TRACE_INFO( "Exchange started: %s\n", flrc.exchange_started ? "YES" : "NO" );
        SMTC_HAL_TRACE_INFO( "========================\n" );

        if( flrc.exchange_started == true )
        {
            flrc.payload_failure_count += 1;
            flrc.exchange_count += 1;  // Advance counter even on timeout
            SMTC_HAL_TRACE_INFO(
                "STATS "
                "RX Timeout: %" PRIu32 "/%" PRIu32 " FLRC packets lost\n",
                flrc.payload_failure_count, flrc.exchange_count );
        }
        else
        {
            SMTC_HAL_TRACE_INFO( "Waiting for first packet...\n" );
        }

        add_delay_ms = RX_TIMEOUT;

        break;
    }
    case RP_STATUS_RX_CRC_ERROR:
    {
        SMTC_HAL_TRACE_ERROR( "FLRC packet received with CRC error\n" );
        SMTC_HAL_TRACE_INFO( "=== RX CRC ERROR DEBUG ===\n" );
        SMTC_HAL_TRACE_INFO( "Expected packet #%" PRIu32 "\n", flrc.exchange_count );
        SMTC_HAL_TRACE_INFO( "Exchange started: %s\n", flrc.exchange_started ? "YES" : "NO" );
        // Try to show received data even if CRC is bad (might be corrupted)

        SMTC_HAL_TRACE_INFO( "==========================\n" );

        if( flrc.exchange_started == true )
        {
            flrc.crc_failure_count += 1;
            flrc.exchange_count += 1;  // Advance counter even on CRC error
            SMTC_HAL_TRACE_INFO(
                "STATS "
                "RX CRC Error: %" PRIu32 "/%" PRIu32 " FLRC packets failed\n",
                flrc.payload_failure_count, flrc.exchange_count );
        }

        add_delay_ms = ( INTER_EXCHANGE_DELAY / 10 );

        break;
    }
    case RP_STATUS_TASK_ABORTED:
    {
        SMTC_HAL_TRACE_WARNING( "Task aborted\n" );
        add_delay_ms = ( INTER_EXCHANGE_DELAY / 10 );
        break;
    }
    default:
    {
        SMTC_HAL_TRACE_ERROR( "Unknown transaction status: %d\n", status );
        break;
    }
    }

    // Display real-time PER statistics
    if( flrc.exchange_started )
    {
        SMTC_HAL_TRACE_INFO(
            "Stats "
            "RT: Total Pkts: %" PRIu32 ", Failed Pkts: %" PRIu32 ", CRC Pkts: %" PRIu32 " \n",
            flrc.exchange_count, flrc.payload_failure_count, flrc.crc_failure_count );
    }

    uint32_t now_ms = smtc_modem_hal_get_time_in_ms( );

    do
    {
        delay_ms = ( INTER_EXCHANGE_DELAY * tmp_delayed_cnt ) -
                   ( now_ms - flrc.transaction->smtc_rac_data_result.radio_end_timestamp_ms ) - add_delay_ms -
                   smtc_modem_hal_get_board_delay_ms( );
        tmp_delayed_cnt++;

    } while( ( int32_t ) delay_ms <= 0 );

    // check if more to do
    if( flrc.exchange_count < MAX_EXCHANGE_COUNT )
    {
        start_new_transaction( now_ms, delay_ms );
    }
    else
    {
        // Final statistics
        uint32_t final_per          = CALCULATE_PER_PERCENT( );
        uint32_t final_success_rate = CALCULATE_SUCCESS_RATE( );

        SMTC_HAL_TRACE_INFO( "========================================\n" );
        SMTC_HAL_TRACE_INFO( "       FLRC PER TEST COMPLETED!\n" );
        SMTC_HAL_TRACE_INFO( "========================================\n" );
        SMTC_HAL_TRACE_INFO( "STATS FINAL FLRC PER RESULTS:\n" );
        SMTC_HAL_TRACE_INFO( "STATS Total Packets Processed: %" PRIu32 "\n", flrc.exchange_count );
        SMTC_HAL_TRACE_INFO( "STATS Successful Packets: %" PRIu32 "\n",
                             flrc.exchange_count - flrc.payload_failure_count );
        SMTC_HAL_TRACE_INFO( "STATS Failed Packets: %" PRIu32 "\n", flrc.payload_failure_count );
        SMTC_HAL_TRACE_INFO( "STATS Final Packet Error Rate: %" PRIu32 "%%\n", final_per );
        SMTC_HAL_TRACE_INFO( "STATS Final Success Rate: %" PRIu32 "%%\n", final_success_rate );

        if( final_per < 1 )
        {
            SMTC_HAL_TRACE_INFO( "Excellent link quality!\n" );
        }
        else if( final_per < 5 )
        {
            SMTC_HAL_TRACE_INFO( "Good link quality.\n" );
        }
        else if( final_per < 10 )
        {
            SMTC_HAL_TRACE_WARNING( "Moderate link quality.\n" );
        }
        else
        {
            SMTC_HAL_TRACE_WARNING( "Poor link quality!\n" );
        }
        SMTC_HAL_TRACE_INFO( "========================================\n" );

        // restart a new series - reset counters
        flrc.exchange_count        = 0;
        flrc.packet_counter        = 0;  // Reset packet counter for new series
        flrc.payload_failure_count = 0;
        flrc.crc_failure_count     = 0;
        flrc.exchange_started      = false;

        SMTC_HAL_TRACE_INFO( "*** RESTARTING NEW PER SERIES ***\n" );

        now_ms = smtc_modem_hal_get_time_in_ms( );

        tmp_delayed_cnt = 1;
        do
        {
            delay_ms = ( INTER_SERIES_DELAY * tmp_delayed_cnt ) -
                       ( now_ms - flrc.transaction->smtc_rac_data_result.radio_end_timestamp_ms ) - add_delay_ms -
                       smtc_modem_hal_get_board_delay_ms( );
            tmp_delayed_cnt++;

        } while( ( int32_t ) delay_ms <= 0 );
        start_new_transaction( now_ms, delay_ms );
    }
}

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

static void start_new_transaction( uint32_t rtc, uint32_t delay )
{
    uint32_t start_time = rtc + delay;

    if( IS_TRANSMITTER )
    {
        SMTC_HAL_TRACE_INFO( "TX ===== Starting transmission #%" PRIu32 "\n", flrc.exchange_count );

        ( void ) memcpy( flrc.tx_payload + HEADER_OFFSET, HEADER, HEADER_SIZE );
        ( void ) memset( flrc.tx_payload + SEPARATOR_OFFSET, 0, SEPARATOR_SIZE );
        ( void ) memcpy( flrc.tx_payload + COUNTER_OFFSET, &( flrc.exchange_count ), COUNTER_SIZE );

        for( uint16_t i = COUNTER_OFFSET + COUNTER_SIZE + 1; i < ( RX_BUFFER_MAX_SIZE - 1 ); i++ )
        {
            flrc.tx_payload[i] = i;
        }
    }
    else
    {
        SMTC_HAL_TRACE_INFO( "RX ===== Starting reception #%" PRIu32 "\n", flrc.exchange_count );

        ( void ) memset( flrc.rx_payload, 0, RX_BUFFER_MAX_SIZE );
    }
    flrc.transaction->scheduler_config.start_time_ms = start_time;

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_rac_submit_radio_transaction( flrc.radio_access_id ) == SMTC_RAC_SUCCESS );
}
