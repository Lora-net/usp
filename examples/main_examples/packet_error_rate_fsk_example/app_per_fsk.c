/**
 * @file      app_per_fsk.c
 *
 * @brief     Simple PER (Packet Error Rate) example with FSK modulation
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

#include "app_per_fsk.h"
#include "main_per_fsk.h"

#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "PER-FSK"
#include "smtc_rac_log.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define RECEIVER 1
#define TRANSMITTER 2

// Backward compatibility aliases for existing code
#define PER_FSK_LOG_INFO( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )
#define PER_FSK_LOG_WARN( ... ) SMTC_HAL_TRACE_WARNING( __VA_ARGS__ )
#define PER_FSK_LOG_ERROR( ... ) SMTC_HAL_TRACE_ERROR( __VA_ARGS__ )
#define PER_FSK_LOG_STATS( ... ) SMTC_HAL_TRACE_INFO( "STATS " __VA_ARGS__ )
#define PER_FSK_LOG_CONFIG( ... ) SMTC_HAL_TRACE_INFO( "CONF " __VA_ARGS__ )
#define PER_FSK_LOG_TX( ... ) SMTC_HAL_TRACE_INFO( "TX "__VA_ARGS__ )
#define PER_FSK_LOG_RX( ... ) SMTC_HAL_TRACE_INFO( "RX " __VA_ARGS__ )
#define PER_FSK_LOG_DEBUG( ... ) SMTC_HAL_TRACE_INFO( "DEBUG " __VA_ARGS__ )
#define PER_FSK_LOG_ARRAY( ... ) SMTC_HAL_TRACE_ARRAY( __VA_ARGS__ )

// Helper macros for calculations (integer-only, no float)
#define CALCULATE_PER_PERCENT( )                                                                         \
    ( ( packet_error_rate_fsk.exchange_count > 0 )                                                       \
          ? ( ( packet_error_rate_fsk.crc_failure_count * 100 ) / packet_error_rate_fsk.exchange_count ) \
          : 0 )
#define CALCULATE_SUCCESS_RATE( )                                                                            \
    ( ( packet_error_rate_fsk.exchange_count > 0 )                                                           \
          ? ( ( ( packet_error_rate_fsk.exchange_count - packet_error_rate_fsk.crc_failure_count ) * 100 ) / \
              packet_error_rate_fsk.exchange_count )                                                         \
          : 0 )

/**
 * @brief payload composition
 *
 * | HEADER |  SEPARATOR  |    COUNTER     |
 * | "FSK"  | (uint8_t) 0 | exchange_count |
 *
 */

#define HEADER_SIZE ( 3 )
#define SEPARATOR_SIZE ( 1 )
#define COUNTER_SIZE ( 4 )  // sizeof(uint32_t)
#define PAYLOAD_SIZE ( HEADER_SIZE + SEPARATOR_SIZE + COUNTER_SIZE )

#define HEADER_OFFSET ( 0 )
#define SEPARATOR_OFFSET ( HEADER_OFFSET + HEADER_SIZE )
#define COUNTER_OFFSET ( SEPARATOR_OFFSET + SEPARATOR_SIZE )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if( ROLE == RECEIVER )
static const bool     IS_TRANSMITTER       = false;
static const uint32_t INTER_EXCHANGE_DELAY = 0;  // ms
#elif( ROLE == TRANSMITTER )
static const bool     IS_TRANSMITTER       = true;
static const uint32_t INTER_EXCHANGE_DELAY = 500;  // ms
#else
#error "Please define ROLE as either RECEIVER or TRANSMITTER"
#endif

static const uint32_t RX_TIMEOUT = 30000;  // ms

static const uint32_t INTER_SERIES_DELAY  = 10000;  // ms
static const uint32_t MAX_EXCHANGE_COUNT  = 100;    // (no unit)
static const uint8_t  HEADER[HEADER_SIZE] = "FSK";  // bytes

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct packet_error_rate_fsk_s
{
    uint32_t exchange_count;          // number of sent/received messages
    uint32_t packet_counter;          // counter for TX packets (separate from exchange_count)
    bool     exchange_started;        // true iff a packet has correctly been received
    uint32_t payload_failure_count;   // number of failed receptions (corrupted payload) after the exchange has started
    uint32_t crc_failure_count;       // number of failed receptions (invalid crc) after the exchange has started
    uint8_t  payload[PAYLOAD_SIZE];   // bytes buffer
    uint8_t  radio_access_id;         // store the result of `smtc_rac_request_radio_access`
    smtc_rac_context_t* transaction;  // associated transaction
} packet_error_rate_fsk_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static packet_error_rate_fsk_t packet_error_rate_fsk = {
    .exchange_count        = 0,
    .packet_counter        = 0,
    .exchange_started      = false,
    .payload_failure_count = 0,
    .crc_failure_count     = 0,
    .payload               = { 0 },
    .radio_access_id       = 0,     // set in `per_fsk_init`
    .transaction           = NULL,  // set in `per_fsk_init`
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void empty_callback( void );

static void unified_transaction_callback( rp_status_t status );

static void start_new_transaction( uint32_t delay );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void per_fsk_init( void )
{
    // Professional initialization banner
    PER_FSK_LOG_INFO( "========================================\n" );
    PER_FSK_LOG_INFO( "     FSK PACKET ERROR RATE (PER) TEST\n" );
    PER_FSK_LOG_INFO( "========================================\n" );
    PER_FSK_LOG_INFO( "Role: %s\n", IS_TRANSMITTER ? "TRANSMITTER" : "RECEIVER" );
    PER_FSK_LOG_INFO( "System Time: %" PRIu32 " ms\n", smtc_modem_hal_get_time_in_ms( ) );

    // Open radio access with unified callback
    packet_error_rate_fsk.radio_access_id = smtc_rac_open_radio( RAC_HIGH_PRIORITY );
    if( packet_error_rate_fsk.radio_access_id == RAC_INVALID_RADIO_ID )
    {
        PER_FSK_LOG_ERROR( "Failed to open radio access!\n" );
        return;
    }
    PER_FSK_LOG_CONFIG( "Radio access ID: %d\n", packet_error_rate_fsk.radio_access_id );

    // get the rac context
    packet_error_rate_fsk.transaction = smtc_rac_get_context( packet_error_rate_fsk.radio_access_id );
    if( packet_error_rate_fsk.transaction == NULL )
    {
        PER_FSK_LOG_ERROR( "Failed to get RAC context!\n" );
        return;
    }

    // rac_context configuration
    packet_error_rate_fsk.transaction->modulation_type                  = SMTC_RAC_MODULATION_FSK;
    packet_error_rate_fsk.transaction->radio_params.fsk.tx_power_in_dbm = TX_OUTPUT_POWER_DBM;
    packet_error_rate_fsk.transaction->radio_params.fsk.frequency_in_hz = RF_FREQ_IN_HZ;
    packet_error_rate_fsk.transaction->radio_params.fsk.br_in_bps       = FSK_BITRATE;
    packet_error_rate_fsk.transaction->radio_params.fsk.fdev_in_hz      = FSK_FDEV;
    packet_error_rate_fsk.transaction->radio_params.fsk.bw_dsb_in_hz    = FSK_BANDWIDTH;
    packet_error_rate_fsk.transaction->radio_params.fsk.preamble_len_in_bits =
        FSK_PREAMBLE_LENGTH * 8;  // Convert bytes to bits
    packet_error_rate_fsk.transaction->radio_params.fsk.sync_word_len_in_bits =
        FSK_SYNC_WORD_LENGTH * 8;                                               // Convert bytes to bits
    packet_error_rate_fsk.transaction->radio_params.fsk.sync_word      = NULL;  // Will use default
    packet_error_rate_fsk.transaction->radio_params.fsk.crc_type       = FSK_CRC;
    packet_error_rate_fsk.transaction->radio_params.fsk.whitening_seed = FSK_WHITENING ? FSK_WHITENING_SEED : 0x0000;
    packet_error_rate_fsk.transaction->radio_params.fsk.header_type    = FSK_PACKET_TYPE;
    packet_error_rate_fsk.transaction->radio_params.fsk.is_tx          = IS_TRANSMITTER;
    packet_error_rate_fsk.transaction->radio_params.fsk.rx_timeout_ms  = RX_TIMEOUT;

    // Set defaults for advanced parameters
    packet_error_rate_fsk.transaction->radio_params.fsk.pulse_shape       = RAL_GFSK_PULSE_SHAPE_BT_1;
    packet_error_rate_fsk.transaction->radio_params.fsk.preamble_detector = RAL_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
    packet_error_rate_fsk.transaction->radio_params.fsk.dc_free           = RAL_GFSK_DC_FREE_WHITENING;
    packet_error_rate_fsk.transaction->radio_params.fsk.crc_seed          = FSK_CRC_SEED;
    packet_error_rate_fsk.transaction->radio_params.fsk.crc_polynomial    = FSK_CRC_POLYNOMIAL;
    packet_error_rate_fsk.transaction->radio_params.fsk.tx_size           = PAYLOAD_SIZE;
    packet_error_rate_fsk.transaction->radio_params.fsk.max_rx_size       = PAYLOAD_SIZE;
    packet_error_rate_fsk.transaction->scheduler_config.start_time_ms     = 0;  // set at each transaction
    packet_error_rate_fsk.transaction->scheduler_config.scheduling        = SMTC_RAC_ASAP_TRANSACTION;
    packet_error_rate_fsk.transaction->scheduler_config.callback_pre_radio_transaction  = empty_callback;
    packet_error_rate_fsk.transaction->scheduler_config.callback_post_radio_transaction = unified_transaction_callback;
    packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer     = packet_error_rate_fsk.payload;
    packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.rx_payload_buffer     = packet_error_rate_fsk.payload;
    packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = PAYLOAD_SIZE;
    packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = PAYLOAD_SIZE;

    // Log radio configuration
    PER_FSK_LOG_CONFIG( "===== RADIO CONFIGURATION =====\n" );
    PER_FSK_LOG_CONFIG( "Modulation: FSK\n" );
    PER_FSK_LOG_CONFIG( "Frequency: %u Hz (%.1f MHz)\n", RF_FREQ_IN_HZ, ( float ) RF_FREQ_IN_HZ / 1000000.0f );
    PER_FSK_LOG_CONFIG( "TX Power: %d dBm\n", TX_OUTPUT_POWER_DBM );
    PER_FSK_LOG_CONFIG( "Bitrate: %u bps (%.1f kbps)\n", FSK_BITRATE, ( float ) FSK_BITRATE / 1000.0f );
    PER_FSK_LOG_CONFIG( "Frequency Deviation: %u Hz (%.1f kHz)\n", FSK_FDEV, ( float ) FSK_FDEV / 1000.0f );
    PER_FSK_LOG_CONFIG( "Bandwidth: %.1f kHz\n", ( float ) FSK_BANDWIDTH / 1000.0f );
    PER_FSK_LOG_CONFIG( "Preamble Length: %d bytes (%d bits)\n", FSK_PREAMBLE_LENGTH, FSK_PREAMBLE_LENGTH * 8 );
    PER_FSK_LOG_CONFIG( "Sync Word: Default (%d bytes)\n", FSK_SYNC_WORD_LENGTH );
    PER_FSK_LOG_CONFIG( "CRC: %s\n", ( FSK_CRC == RAL_GFSK_CRC_2_BYTES ) ? "2-byte CRC" : "Other CRC" );
    PER_FSK_LOG_CONFIG( "Whitening: %s\n", FSK_WHITENING ? "Enabled" : "Disabled" );
    PER_FSK_LOG_CONFIG( "Packet Type: %s\n",
                        ( FSK_PACKET_TYPE == RAL_GFSK_PKT_VAR_LEN ) ? "Variable Length" : "Fixed Length" );
    if( !IS_TRANSMITTER )
    {
        PER_FSK_LOG_CONFIG( "RX Timeout: %lu ms\n", RX_TIMEOUT );
    }
    PER_FSK_LOG_CONFIG( "Payload Size: %d bytes\n", PAYLOAD_SIZE );
    PER_FSK_LOG_CONFIG( "Max Exchanges: %" PRIu32 "\n", MAX_EXCHANGE_COUNT );
    PER_FSK_LOG_CONFIG( "================================\n" );

    // start application
    PER_FSK_LOG_INFO( "Starting FSK PER test sequence...\n" );
    start_new_transaction( 0 );
}

void per_fsk_on_button_press( void )
{
    // nothing to do
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void empty_callback( void )
{
    // nothing to do
}

static void unified_transaction_callback( rp_status_t status )
{
    bool payload_is_correct = true;
    // will be used to hold the received payload data
    uint8_t  header_received[HEADER_SIZE] = { 0 };
    uint8_t  separator_received           = 0;
    uint32_t counter_received             = 0;

    // Turn off LEDs after transaction
    set_led( SMTC_PF_LED_TX, false );
    set_led( SMTC_PF_LED_RX, false );

    PER_FSK_LOG_INFO( "===== FSK Transaction #%" PRIu32 " Complete =====\n", packet_error_rate_fsk.exchange_count );

    switch( status )
    {
    case RP_STATUS_TX_DONE:
    {
        // increment counters after successful transmission
        packet_error_rate_fsk.exchange_count += 1;
        packet_error_rate_fsk.packet_counter += 1;  // increment packet counter for next TX
        PER_FSK_LOG_TX( "FSK transmission completed successfully\n" );

        // Debug: show transmitted packet details
        PER_FSK_LOG_DEBUG( "=== TRANSMITTED PACKET DEBUG ===\n" );
        PER_FSK_LOG_ARRAY( "TX PACKET", packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer,
                           PAYLOAD_SIZE );
        PER_FSK_LOG_DEBUG( "TX Packet Structure:\n" );
        PER_FSK_LOG_DEBUG(
            "  Header: '%.3s'\n",
            ( char* ) &packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer[HEADER_OFFSET] );
        PER_FSK_LOG_DEBUG(
            "  Separator: 0x%02X\n",
            packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer[SEPARATOR_OFFSET] );
        uint32_t tx_counter = 0;
        memcpy( &tx_counter,
                &packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer[COUNTER_OFFSET],
                COUNTER_SIZE );
        PER_FSK_LOG_DEBUG( "  Counter: %" PRIu32 "\n", tx_counter );
        PER_FSK_LOG_DEBUG( "=================================\n" );

        PER_FSK_LOG_STATS( "TX Progress: %" PRIu32 "/%" PRIu32 " FSK packets sent\n",
                           packet_error_rate_fsk.exchange_count, MAX_EXCHANGE_COUNT );
        break;
        }
    case RP_STATUS_RX_PACKET:
    {
        PER_FSK_LOG_RX( "FSK packet received successfully\n" );
        PER_FSK_LOG_RX( "RSSI: %d dBm\n", packet_error_rate_fsk.transaction->smtc_rac_data_result.rssi_result );

        // Debug: show received packet details BEFORE parsing
        PER_FSK_LOG_DEBUG( "=== RECEIVED PACKET DEBUG ===\n" );
        PER_FSK_LOG_ARRAY( "RX PACKET", packet_error_rate_fsk.payload, PAYLOAD_SIZE );
        PER_FSK_LOG_DEBUG( "RX Packet Size: %d bytes\n",
                           packet_error_rate_fsk.transaction->smtc_rac_data_result.rx_size );

        packet_error_rate_fsk.exchange_started = true;
        if( packet_error_rate_fsk.transaction->smtc_rac_data_result.rx_size == PAYLOAD_SIZE )
        {
            ( void ) memcpy( header_received, packet_error_rate_fsk.payload + HEADER_OFFSET, HEADER_SIZE );
            ( void ) memcpy( &separator_received, packet_error_rate_fsk.payload + SEPARATOR_OFFSET, SEPARATOR_SIZE );
            ( void ) memcpy( &counter_received, packet_error_rate_fsk.payload + COUNTER_OFFSET, COUNTER_SIZE );

            // Debug: show parsed packet structure
            PER_FSK_LOG_DEBUG( "RX Parsed Structure:\n" );
            PER_FSK_LOG_DEBUG( "  Header received: '%.3s'\n", header_received );
            PER_FSK_LOG_DEBUG( "  Header expected: '%.3s'\n", HEADER );
            PER_FSK_LOG_DEBUG( "  Separator received: 0x%02X (expected: 0x00)\n", separator_received );
            PER_FSK_LOG_DEBUG( "  Counter received: %" PRIu32 " (expected: %" PRIu32 ")\n", counter_received,
                               packet_error_rate_fsk.exchange_count );
            PER_FSK_LOG_DEBUG( "=============================\n" );
        }
        else
        {
            payload_is_correct = false;
        }

        payload_is_correct &= ( memcmp( header_received, HEADER, HEADER_SIZE ) == 0 );
        payload_is_correct &= ( separator_received == 0 );
        if( payload_is_correct == false )
        {
            PER_FSK_LOG_ERROR( "Corrupted FSK payload detected - header or separator invalid\n" );
            PER_FSK_LOG_ERROR( "Expected header: '%.3s', received: '%.3s'\n", HEADER, header_received );
            PER_FSK_LOG_ERROR( "Expected separator: 0, received: %d\n", separator_received );
            packet_error_rate_fsk.payload_failure_count += 1;
            packet_error_rate_fsk.exchange_count += 1;  // Increment even for corrupted packets
            break;
        }

        // Handle packet sequence: sync with received counter + detect lost packets
        uint32_t expected_counter   = packet_error_rate_fsk.exchange_count;
        uint32_t new_exchange_count = counter_received + 1;  // counter starts at 0, exchange_count at 1

        if( counter_received < expected_counter )
        {
            PER_FSK_LOG_WARN( "Out-of-order packet: received counter %" PRIu32 ", expected %" PRIu32 "\n",
                              counter_received, expected_counter );
            // Don't update exchange_count for out-of-order packets
            packet_error_rate_fsk.payload_failure_count += 1;
        }
        else if( counter_received > expected_counter )
        {
            // Lost packets detected - update failure count
            uint32_t lost_packets = counter_received - expected_counter;
            packet_error_rate_fsk.payload_failure_count += lost_packets;
            packet_error_rate_fsk.exchange_count = new_exchange_count;

            PER_FSK_LOG_WARN( "Lost packets detected: expected counter %" PRIu32 ", received %" PRIu32 " (%" PRIu32
                              " packets lost)\n",
                              expected_counter, counter_received, lost_packets );
            PER_FSK_LOG_STATS( "Packet loss: %" PRIu32 " consecutive packets lost\n", lost_packets );
        }
        else
        {
            // Perfect sequence - update normally
            packet_error_rate_fsk.exchange_count = new_exchange_count;
            PER_FSK_LOG_DEBUG( "Perfect sequence: counter %" PRIu32 " as expected\n", counter_received );
        }

        PER_FSK_LOG_RX( "FSK payload valid - Header: '%s', Counter: %" PRIu32 "\n", header_received, counter_received );
        PER_FSK_LOG_STATS( "RX Progress: %" PRIu32 "/%" PRIu32 " FSK packets received successfully\n",
                           packet_error_rate_fsk.exchange_count - packet_error_rate_fsk.payload_failure_count,
                           packet_error_rate_fsk.exchange_count );
        break;
        }
    case RP_STATUS_RX_TIMEOUT:
    {

        PER_FSK_LOG_WARN( "FSK reception timeout after %" PRIu32 " ms\n", RX_TIMEOUT );
        PER_FSK_LOG_DEBUG( "=== RX TIMEOUT DEBUG ===\n" );
        PER_FSK_LOG_DEBUG( "Expected packet #%" PRIu32 "\n", packet_error_rate_fsk.exchange_count );
        PER_FSK_LOG_DEBUG( "Exchange started: %s\n", packet_error_rate_fsk.exchange_started ? "YES" : "NO" );
        PER_FSK_LOG_DEBUG( "========================\n" );

        if( packet_error_rate_fsk.exchange_started == true )
        {
            packet_error_rate_fsk.payload_failure_count += 1;
            packet_error_rate_fsk.exchange_count += 1;  // Advance counter even on timeout
            PER_FSK_LOG_STATS( "RX Timeout: %" PRIu32 "/%" PRIu32 " FSK packets lost\n",
                               packet_error_rate_fsk.payload_failure_count, packet_error_rate_fsk.exchange_count );
        }
        else
        {
            PER_FSK_LOG_INFO( "Waiting for first FSK packet...\n" );
        }
        break;
        }
    case RP_STATUS_RX_CRC_ERROR:
    {
        PER_FSK_LOG_ERROR( "FSK packet received with CRC error\n" );
        PER_FSK_LOG_DEBUG( "=== RX CRC ERROR DEBUG ===\n" );
        PER_FSK_LOG_DEBUG( "Expected packet #%" PRIu32 "\n", packet_error_rate_fsk.exchange_count );
        PER_FSK_LOG_DEBUG( "Exchange started: %s\n", packet_error_rate_fsk.exchange_started ? "YES" : "NO" );
        // Try to show received data even if CRC is bad (might be corrupted)
        PER_FSK_LOG_ARRAY( "RX CRC ERROR", packet_error_rate_fsk.payload, PAYLOAD_SIZE );
        PER_FSK_LOG_DEBUG( "==========================\n" );

        if( packet_error_rate_fsk.exchange_started == true )
        {
            packet_error_rate_fsk.crc_failure_count += 1;
            packet_error_rate_fsk.exchange_count += 1;  // Advance counter even on CRC error
            PER_FSK_LOG_STATS( "RX CRC Error: %" PRIu32 "/%" PRIu32 " FSK packets failed\n",
                               packet_error_rate_fsk.payload_failure_count, packet_error_rate_fsk.exchange_count );
        }
        break;
        }
    default:
        PER_FSK_LOG_ERROR( "Unknown transaction status: %d\n", status );
        break;
    }

    // Display real-time PER statistics
    if( packet_error_rate_fsk.exchange_started )
    {
        uint32_t per_percent  = CALCULATE_PER_PERCENT( );
        uint32_t success_rate = CALCULATE_SUCCESS_RATE( );

        PER_FSK_LOG_STATS( "=== REAL-TIME FSK PER STATISTICS ===\n" );
        PER_FSK_LOG_STATS( "Total FSK Packets: %" PRIu32 "/%" PRIu32 "\n", packet_error_rate_fsk.exchange_count,
                           MAX_EXCHANGE_COUNT );
        PER_FSK_LOG_STATS( "Failed Packets: %" PRIu32 "\n", packet_error_rate_fsk.payload_failure_count );
        PER_FSK_LOG_STATS( "CRC Failed Packets: %" PRIu32 "\n", packet_error_rate_fsk.crc_failure_count );
        PER_FSK_LOG_STATS( "Packet Error Rate: %lu%%\n", per_percent );
        PER_FSK_LOG_STATS( "Success Rate: %" PRIu32 "%%\n", success_rate );
        PER_FSK_LOG_STATS( "====================================\n" );
    }

    // check if more to do
    if( packet_error_rate_fsk.exchange_count < MAX_EXCHANGE_COUNT )
    {
        start_new_transaction( INTER_EXCHANGE_DELAY );
    }
    else
    {
        // Final statistics
        uint32_t final_per          = CALCULATE_PER_PERCENT( );
        uint32_t final_success_rate = CALCULATE_SUCCESS_RATE( );

        PER_FSK_LOG_INFO( "========================================\n" );
        PER_FSK_LOG_INFO( "       FSK PER TEST COMPLETED!\n" );
        PER_FSK_LOG_INFO( "========================================\n" );
        PER_FSK_LOG_STATS( "FINAL FSK PER RESULTS:\n" );
        PER_FSK_LOG_STATS( "Total Packets Processed: %" PRIu32 "\n", packet_error_rate_fsk.exchange_count );
        PER_FSK_LOG_STATS( "Successful Packets: %" PRIu32 "\n",
                           packet_error_rate_fsk.exchange_count - packet_error_rate_fsk.payload_failure_count );
        PER_FSK_LOG_STATS( "Failed Packets: %" PRIu32 "\n", packet_error_rate_fsk.payload_failure_count );
        PER_FSK_LOG_STATS( "Final Packet Error Rate: %" PRIu32 "%%\n", final_per );
        PER_FSK_LOG_STATS( "Final Success Rate: %" PRIu32 "%%\n", final_success_rate );

        if( final_per < 1 )
        {
            PER_FSK_LOG_INFO( "Excellent FSK link quality!\n" );
        }
        else if( final_per < 5 )
        {
            PER_FSK_LOG_INFO( "Good FSK link quality.\n" );
        }
        else if( final_per < 10 )
        {
            PER_FSK_LOG_WARN( "Moderate FSK link quality.\n" );
        }
        else
        {
            PER_FSK_LOG_WARN( "Poor FSK link quality!\n" );
        }
        PER_FSK_LOG_INFO( "========================================\n" );

        // restart a new series - reset counters
        packet_error_rate_fsk.exchange_count        = 0;
        packet_error_rate_fsk.packet_counter        = 0;  // Reset packet counter for new series
        packet_error_rate_fsk.payload_failure_count = 0;
        packet_error_rate_fsk.crc_failure_count     = 0;
        packet_error_rate_fsk.exchange_started      = false;
        PER_FSK_LOG_INFO( "*** RESTARTING NEW FSK PER SERIES ***\n" );
        start_new_transaction( INTER_SERIES_DELAY );
    }
}

static void start_new_transaction( uint32_t delay )
{
    uint8_t* buffer = packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer;

    // Reset transaction state for clean operation
    packet_error_rate_fsk.transaction->radio_params.fsk.tx_size         = PAYLOAD_SIZE;
    packet_error_rate_fsk.transaction->smtc_rac_data_result.rssi_result = 0;
    packet_error_rate_fsk.transaction->smtc_rac_data_result.snr_result  = 0;

    // Clear payload buffer for clean state
    memset( packet_error_rate_fsk.payload, 0, PAYLOAD_SIZE );

    PER_FSK_LOG_DEBUG( "=== TRANSACTION RESET FOR #%" PRIu32 " ===\n", packet_error_rate_fsk.exchange_count + 1U );

    // Pre-transaction logging (moved from old pre_transaction_callback)
    if( IS_TRANSMITTER )
    {
        PER_FSK_LOG_TX( "Starting FSK transmission #%" PRIu32 "\n", packet_error_rate_fsk.exchange_count + 1U );
        PER_FSK_LOG_TX( "Payload: '%.3s' + counter=%" PRIu32 "\n", HEADER, packet_error_rate_fsk.packet_counter );
        set_led( SMTC_PF_LED_TX, true );

        // set payload for the next transmission
        // Use dedicated packet_counter that resets to 0 at each series
        ( void ) memcpy( buffer + HEADER_OFFSET, HEADER, HEADER_SIZE );
        ( void ) memset( buffer + SEPARATOR_OFFSET, 0, SEPARATOR_SIZE );
        ( void ) memcpy( buffer + COUNTER_OFFSET, &packet_error_rate_fsk.packet_counter, COUNTER_SIZE );

        // Debug: show packet being prepared for transmission
        PER_FSK_LOG_DEBUG( "=== PREPARING TX PACKET ===\n" );
        PER_FSK_LOG_ARRAY( "TX PREP", buffer, PAYLOAD_SIZE );
        PER_FSK_LOG_DEBUG( "TX Packet Structure:\n" );
        PER_FSK_LOG_DEBUG( "  Header: '%.3s'\n", HEADER );
        PER_FSK_LOG_DEBUG( "  Separator: 0x00\n" );
        PER_FSK_LOG_DEBUG( "  Counter: %" PRIu32 " (exchange_count: %" PRIu32 ")\n",
                           packet_error_rate_fsk.packet_counter, packet_error_rate_fsk.exchange_count );
        PER_FSK_LOG_DEBUG( "===========================\n" );
    }
    else
    {
        PER_FSK_LOG_RX( "Starting FSK reception #%" PRIu32 " (timeout: %" PRIu32 " ms)\n",
                        packet_error_rate_fsk.exchange_count + 1U, RX_TIMEOUT );
        PER_FSK_LOG_RX( "Awaiting FSK packet...\n" );
        set_led( SMTC_PF_LED_RX, true );

        // set the buffer where to store the received payload
        packet_error_rate_fsk.transaction->smtc_rac_data_buffer_setup.rx_payload_buffer = packet_error_rate_fsk.payload;

        PER_FSK_LOG_DEBUG( "RX Buffer prepared, expecting packet #%" PRIu32 "\n",
                           packet_error_rate_fsk.exchange_count );
    }

    // configure scheduler
    packet_error_rate_fsk.transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + delay;

    // request transaction
    smtc_rac_submit_radio_transaction( packet_error_rate_fsk.radio_access_id );
}
