/**
 * @file      app_ranging_hopping.c
 *
 * @brief     Ranging and frequency hopping for LR1110 or LR1120 chip
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

#include "app_ranging_hopping.h"
#include "apps_configuration.h"
#include "main_ranging_demo.h"
#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "RANGING"
#include "smtc_rac_log.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// Helper macros for logging
#ifndef DISABLE_RANGING_LOG
#define RANGING_LOG_INFO( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )
#define RANGING_LOG_WARN( ... ) SMTC_HAL_TRACE_WARNING( __VA_ARGS__ )
#define RANGING_LOG_ERROR( ... ) SMTC_HAL_TRACE_ERROR( __VA_ARGS__ )
#define RANGING_LOG_CONFIG( ... ) SMTC_HAL_TRACE_INFO( "CONF " __VA_ARGS__ )
#define RANGING_LOG_TX( ... ) SMTC_HAL_TRACE_INFO( "TX "__VA_ARGS__ )
#define RANGING_LOG_RX( ... ) SMTC_HAL_TRACE_INFO( "RX " __VA_ARGS__ )
#define RANGING_LOG_RESULT( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )
#define RANGING_LOG_PRINTF( ... ) SMTC_HAL_TRACE_PRINTF( __VA_ARGS__ )
#else
#define RANGING_LOG_INFO( ... )
#define RANGING_LOG_WARN( ... )
#define RANGING_LOG_ERROR( ... )
#define RANGING_LOG_CONFIG( ... )
#define RANGING_LOG_TX( ... )
#define RANGING_LOG_RX( ... )
#define RANGING_LOG_RESULT( ... )
#define RANGING_LOG_PRINTF( ... )
#endif

/*!
 * @brief Define preset ranging addresses
 */
#define RANGING_ADDR_1 0x32101222

/*!
 * @brief Total symbol numbers of a ranging process
 */
#define RANGING_ALL_SYMBOL ( 64.25f )
#define RANGING_DONE_PROCESSING_TIME 20  // ms
#define ASAP_TRANSACTION SMTC_RAC_ASAP_TRANSACTION
#define SCHEDULE_TRANSACTION SMTC_RAC_SCHEDULED_TRANSACTION

/*!
 * @brief Ranging related IRQs enabled on the Ranging manager device
 */
#define RANGING_MANAGER_IRQ_MASK ( LR11XX_SYSTEM_IRQ_RTTOF_EXCH_VALID | LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT )

/*!
 * @brief Ranging IRQs enabled on the Ranging subordinate device
 */
#define RANGING_SUBORDINATE_IRQ_MASK ( LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED | LR11XX_SYSTEM_IRQ_RTTOF_RESP_DONE )

#define LORA_IRQ_MASK                                                                          \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_HEADER_ERROR | \
      LR11XX_SYSTEM_IRQ_TIMEOUT | LR11XX_SYSTEM_IRQ_CRC_ERROR )

/*!
 * @brief Number of ranging address bytes the subordinate has to check upon reception of a ranging request
 */
#define RANGING_SUBORDINATE_CHECK_LENGTH_BYTES ( 4 )

/*!
 * @brief Length in byte of the ranging result
 */
#define LR11XX_RANGING_RESULT_LENGTH ( 4 )

/*!
 * @brief Number of symbols in ranging response
 */
#ifndef RANGING_RESPONSE_SYMBOLS_COUNT
#define RANGING_RESPONSE_SYMBOLS_COUNT UINT8_C( 15 )
#endif

#define BW_CNT 7
#define REPEAT_CNT 20

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
static uint32_t repeat_cnt = 0;

/*!
 * @brief Reference frequency hopping tables.
 * These arrays define the set of frequencies used for hopping during ranging.
 * Only one is active at a time, depending on the region.
 */
#if RF_FREQ_IN_HZ < 600000000
const char *region = "cn";
const uint32_t ranging_hopping_channels_array[RANGING_HOPPING_CHANNELS_MAX] = {
    490810000, 508940000, 496690000, 507470000, 504040000, 508450000, 505020000, 497670000, 497180000, 500610000,
    494240000, 493260000, 495710000, 491300000, 504530000, 501100000, 502080000, 501590000, 499140000, 494730000,
    505510000, 500120000, 503060000, 506000000, 506490000, 498650000, 491790000, 503550000, 502570000,
};

#elif RF_FREQ_IN_HZ < 900000000
const char *region = "eu";
const uint32_t ranging_hopping_channels_array[RANGING_HOPPING_CHANNELS_MAX] = {
    863750000, 865100000, 864800000, 868400000, 865250000, 867500000, 865550000, 867650000, 866150000, 864050000,
    //867800000, 863300000, 863450000, 867950000, 868550000, 868850000, 867200000, 867050000, 864650000, 863900000,
    864500000, 866450000, 865400000, 868700000, 863150000, 866750000, 866300000, 864950000, 864350000, 866000000,
    866900000, 868250000, 865850000, 865700000, 867350000, 868100000, 863600000, 866600000, 864200000,
};

#elif RF_FREQ_IN_HZ < 2000000000
const char *region = "us";
const uint32_t ranging_hopping_channels_array[RANGING_HOPPING_CHANNELS_MAX] = {
    907850000, 902650000, 914350000, 906550000, 905900000, 924750000, 926700000, 918250000, 921500000, 909150000,
    //907200000, 924100000, 903950000, 910450000, 917600000, 919550000, 923450000, 925400000, 909800000, 915000000,
    912400000, 904600000, 908500000, 911100000, 911750000, 916300000, 918900000, 905250000, 913700000, 927350000,
    926050000, 916950000, 913050000, 903300000, 920200000, 922800000, 915650000, 922150000, 920850000,
};

#else
const char *region = "2G4";
const uint32_t ranging_hopping_channels_array[RANGING_HOPPING_CHANNELS_MAX] = {
    2450000000, 2402000000, 2476000000, 2436000000, 2430000000, 2468000000, 2458000000, 2416000000,
    //2424000000, 2478000000, 2456000000, 2448000000, 2462000000, 2472000000, 2432000000, 2446000000,
    2422000000, 2442000000, 2460000000, 2474000000, 2414000000, 2464000000, 2454000000, 2444000000,
    2404000000, 2434000000, 2410000000, 2408000000, 2440000000, 2452000000, 2480000000, 2426000000,
    2428000000, 2466000000, 2418000000, 2412000000, 2406000000, // 2470000000, 2438000000,
};
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Ranging result comprising raw distance, distance meter and RSSI.
 * (Definition not shown here, assumed to be in a header)
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

// Results callback (to be extended with variance, and other metrics)
static void ( *results_callback )( smtc_rac_radio_lora_params_t*                    radio_lora_params,
                                   ranging_params_settings_t*                       ranging_params_settings,
                                   ranging_global_result_t*                         ranging_global_results,
                                   const char* region ) = NULL;

// Structure holding the current ranging settings
static ranging_params_settings_t ranging_settings = { 0 };
// Structure holding the global ranging results
static ranging_global_result_t ranging_results = { 0 };

/*!
 * @brief Radio payload buffer
 * Used to store the payload for radio transmissions
 */
static uint8_t radio_pl_buffer[PAYLOAD_LENGTH];

/*!
 * @brief Flag holding the current internal state of the ranging application
 */
static app_radio_internal_states_t ranging_internal_state;

/*!
 * @brief Current channel that is used for ranging
 */
static uint8_t current_channel;

/*!
 * @brief Count RANGING_HOPPING_CHANNELS_MAX that have been used
 * (Unused in this code, but could be used for statistics)
 */
// static uint16_t measured_channels;

// set a specific value to indicate the ranging value is invalid
int32_t invalid_value = 0xdeadbeef;

static uint8_t ranging_radio_access_id;

// Structure holding all radio and scheduler configuration for RAC API
static smtc_rac_context_t* rac_config = NULL;
// Flag to indicate if multiple data rates are activated
// This allows the application to use different spreading factors and bandwidths

#if defined( ACTIVATE_MULTIPLE_DATA_RATE ) && ( ACTIVATE_MULTIPLE_DATA_RATE == true )
// If multiple data rates are activated, set the extra delay for the radio
// This is used to ensure proper timing when using different data rates
static bool     activate_multiple_data_rate           = true;
static uint32_t extra_delay_ms_for_multiple_data_rate = 40;
#else
static bool     activate_multiple_data_rate           = false;
static uint32_t extra_delay_ms_for_multiple_data_rate = 0;
#endif

static ral_lora_sf_t current_sf = RAL_LORA_SF5;
static ral_lora_bw_t current_bw = RAL_LORA_BW_125_KHZ;
static uint8_t       multiple_data_rate_config;
#if defined( CONTINUOUS_RANGING ) && ( CONTINUOUS_RANGING == true )
static bool continuous_ranging = true;
#else
static bool continuous_ranging = false;
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
// Helper to get recommended delay indicator for ranging
static bool common_rttof_recommended_rx_tx_delay_indicator( uint32_t rf_freq_in_hz, ral_lora_bw_t bw, ral_lora_sf_t sf,
                                                            uint32_t* delay_indicator );
// Helper to compute the time of a single LoRa symbol in ms
static float get_single_symbol_time_ms( ral_lora_bw_t bw, ral_lora_sf_t sf );
// State machine for the manager node
static void ranging_state_machine_manager( rp_status_t status );
// State machine for the subordinate node
static void ranging_state_machine_subordinate( rp_status_t status );
// Compute the median of an array of uint32_t
static int32_t compute_median( int32_t* array, uint8_t size );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void app_radio_ranging_set_user_callback( void ( *results_callback_init )(
    smtc_rac_radio_lora_params_t* radio_lora_params, ranging_params_settings_t* ranging_params_settings,
    ranging_global_result_t* ranging_global_results, const char* region ) )
{
    results_callback = results_callback_init;
}

/*!
 * @brief Initialize ranging parameters and state machine
 * @param is_manager true if this node is the manager, false if subordinate
 */
void app_radio_ranging_params_init( bool is_manager, smtc_rac_priority_t priority )
{
    RANGING_LOG_INFO( "Ranging hopping demo started\n" );

    // Initialize the ranging settings and results structures
    memset( &ranging_settings, 0, sizeof( ranging_params_settings_t ) );
    memset( &ranging_results, 0, sizeof( ranging_global_result_t ) );

    // Set the initial state of the ranging application
    ranging_internal_state = APP_RADIO_IDLE;

    // Initialize the radio payload buffer
    memset( radio_pl_buffer, 0, sizeof( radio_pl_buffer ) );

    // Set the current channel to 0
    current_channel = 0;

    // Request radio access for the ranging state machine
    ranging_radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( priority ) );
    rac_config              = smtc_rac_get_context( ranging_radio_access_id );

    if( is_manager == true )
    {
        rac_config->scheduler_config.callback_post_radio_transaction = ranging_state_machine_manager;
    }
    else
    {
        rac_config->scheduler_config.callback_post_radio_transaction = ranging_state_machine_subordinate;
    }
    // Setup the radio parameters for ranging
    app_radio_ranging_setup( NULL );
}

/*!
 * @brief Setup the radio parameters for ranging
 * @param context Not used
 */
void app_radio_ranging_setup( const void* context )
{
    // Set all radio parameters for the ranging operation
    rac_config->modulation_type                        = SMTC_RAC_MODULATION_LORA;
    rac_config->radio_params.lora.frequency_in_hz      = RF_FREQ_IN_HZ;
    rac_config->radio_params.lora.tx_power_in_dbm      = TX_OUTPUT_POWER_DBM;
    rac_config->radio_params.lora.preamble_len_in_symb = LORA_PREAMBLE_LENGTH;
    rac_config->radio_params.lora.header_type          = LORA_PKT_LEN_MODE;
    rac_config->radio_params.lora.invert_iq_is_on      = LORA_IQ;
    rac_config->radio_params.lora.crc_is_on            = LORA_CRC;
    rac_config->radio_params.lora.sync_word            = LORA_PRIVATE_NETWORK_SYNCWORD;
    rac_config->radio_params.lora.sf                   = LORA_SPREADING_FACTOR;
    rac_config->radio_params.lora.bw                   = LORA_BANDWIDTH;
    rac_config->radio_params.lora.cr                   = LORA_CODING_RATE;

    rac_config->radio_params.lora.is_ranging_exchange          = false;
    rac_config->radio_params.lora.rttof.request_address        = RANGING_ADDR_1;
    rac_config->radio_params.lora.rttof.response_symbols_count = RANGING_RESPONSE_SYMBOLS_COUNT;
    rac_config->radio_params.lora.rttof.bw_ranging             = rac_config->radio_params.lora.bw;

    // Prepare the radio payload buffer with address and hopping info
    ranging_settings.rng_req_count = RANGING_HOPPING_CHANNELS_MAX;
    ranging_settings.rng_address   = RANGING_ADDR_1;
    radio_pl_buffer[0]             = ( ranging_settings.rng_address >> 24u ) & 0xFFu;
    radio_pl_buffer[1]             = ( ranging_settings.rng_address >> 16u ) & 0xFFu;
    radio_pl_buffer[2]             = ( ranging_settings.rng_address >> 8u ) & 0xFFu;
    radio_pl_buffer[3]             = ( ranging_settings.rng_address & 0xFFu );
    radio_pl_buffer[4]             = 0;                               // set the first channel to use
    radio_pl_buffer[5]             = ranging_settings.rng_req_count;  // set the number of frequency hopping
    radio_pl_buffer[6]             = 0;
    rac_config->smtc_rac_data_buffer_setup.tx_payload_buffer         = radio_pl_buffer;
    rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer         = radio_pl_buffer;
    rac_config->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( radio_pl_buffer );
    rac_config->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = sizeof( radio_pl_buffer );
    rac_config->radio_params.lora.max_rx_size                        = sizeof( radio_pl_buffer );
    rac_config->radio_params.lora.tx_size                            = sizeof( radio_pl_buffer );

    ranging_results.cnt_packet_rx_ok = 0u;
}

/*!
 * @brief State machine for the subordinate (slave) node.
 *
 * This function manages all the states and transitions for the subordinate node during the ranging and frequency
 * hopping process. It is called by the RAC scheduler when a radio event occurs (TX done, RX done, timeout, etc.).
 *
 * The subordinate is responsible for:
 *   - Waiting for the initial ranging configuration from the manager node.
 *   - Validating the received configuration (address and parameters).
 *   - Sending an acknowledgment back to the manager.
 *   - Participating in the ranging exchange on each hopping channel as instructed.
 *
 *   - Handling errors and timeouts by restarting the process or returning to idle.
 *
 * The function uses a switch-case on the internal state (ranging_internal_state) to determine the current step:
 *   - APP_RADIO_IDLE: Prepares the radio and LEDs for idle state.
 *   - APP_RADIO_RANGING_CONFIG: Handles reception and validation of the config from the manager, and sends
 * acknowledgment.
 *   - APP_RADIO_RANGING_START: Handles the actual ranging exchanges on each channel and schedules the next exchange.
 *   - Any error or unknown state restarts the ranging exchange.
 *
 * The context parameter is a pointer to the radio process status (rp_status_t), which indicates the last radio event.
 * The function updates the global state, schedules the next radio transaction, and manages the hopping sequence as a
 * subordinate.
 */
static void ranging_state_machine_subordinate( rp_status_t status )
{
    uint32_t current_radio_timestamp_ms = ( rac_config->smtc_rac_data_result.radio_end_timestamp_ms );
    switch( ranging_internal_state )
    {
    case APP_RADIO_IDLE:
        // Setup radio and LEDs for idle state
        //  app_radio_ranging_setup( NULL );
        set_led( SMTC_PF_LED_TX, false );
        set_led( SMTC_PF_LED_RX, true );
        break;
    case APP_RADIO_RANGING_CONFIG:
    {
        switch( status )
        {
        case RP_STATUS_TX_DONE:
            // After TX done, go to ranging start state and prepare for RX
            ranging_internal_state = APP_RADIO_RANGING_START;

            rac_config->radio_params.lora.is_tx               = false;
            rac_config->radio_params.lora.is_ranging_exchange = true;
            rac_config->radio_params.lora.frequency_in_hz     = ranging_hopping_channels_array[0];
            // rac_config->smtc_rac_data.ranging_result = ( rp_ranging_result_t* ) ( &ranging_results.rng_result[0] );
            rac_config->scheduler_config.scheduling = SCHEDULE_TRANSACTION;

            rac_config->scheduler_config.callback_pre_radio_transaction = &toggle_led;
            if( activate_multiple_data_rate == true )
            {
                rac_config->radio_params.lora.sf =
                    ( ral_lora_sf_t ) multiple_data_rate_config & 0x0F;  // Extract SF from payload
                rac_config->radio_params.lora.bw =
                    ( ral_lora_bw_t ) ( multiple_data_rate_config >> 4 );  // Extract BW from payload

                rac_config->radio_params.lora.rttof.bw_ranging = rac_config->radio_params.lora.bw;
            }
            common_rttof_recommended_rx_tx_delay_indicator(
                rac_config->radio_params.lora.frequency_in_hz, rac_config->radio_params.lora.bw,
                rac_config->radio_params.lora.sf, &rac_config->radio_params.lora.rttof.delay_indicator );
            ranging_settings.rng_req_delay =
                ( uint16_t ) ( get_single_symbol_time_ms( rac_config->radio_params.lora.bw,
                                                          rac_config->radio_params.lora.sf ) *
                               RANGING_ALL_SYMBOL ) +
                RANGING_DONE_PROCESSING_TIME + extra_delay_ms_for_multiple_data_rate;
            ranging_settings.rng_exch_timeout           = ranging_settings.rng_req_delay;
            rac_config->radio_params.lora.rx_timeout_ms = ranging_settings.rng_exch_timeout;
            rac_config->scheduler_config.start_time_ms =
                current_radio_timestamp_ms + ( ranging_settings.rng_req_delay >> 1 );
            SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ranging_radio_access_id ) );

            // Print radio parameters for debug
            RANGING_LOG_INFO( "ranging parameters: \r\n" );
            RANGING_LOG_INFO( "bw: %s\r\n", ral_lora_bw_to_str( rac_config->radio_params.lora.bw ) );
            RANGING_LOG_INFO( "sf: %s\r\n", ral_lora_sf_to_str( rac_config->radio_params.lora.sf ) );
            RANGING_LOG_INFO( "rng_req_delay           =%d\n", ranging_settings.rng_req_delay );
            RANGING_LOG_INFO( "starting ranging exchange\n" );
            break;

        case RP_STATUS_RX_PACKET:
            // Received a ranging config packet from the manager
            RANGING_LOG_RX( "Subordinate Ranging config rx done -> go to ack this config\n" );
            RANGING_LOG_RX( "Subordinate Ranging config rx done, payload size: %d\n",
                            rac_config->smtc_rac_data_result.rx_size );

            ranging_results.rssi_value = rac_config->smtc_rac_data_result.rssi_result;
            ranging_results.snr_value  = rac_config->smtc_rac_data_result.snr_result;

            // Check if the received address matches the expected address
            if( ( rac_config->smtc_rac_data_result.rx_size == sizeof( radio_pl_buffer ) ) &&
                ( rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer[0] ==
                  ( ( ranging_settings.rng_address >> 24 ) & 0xFF ) ) &&
                ( rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer[1] ==
                  ( ( ranging_settings.rng_address >> 16 ) & 0xFF ) ) &&
                ( rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer[2] ==
                  ( ( ranging_settings.rng_address >> 8 ) & 0xFF ) ) &&
                ( rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer[3] ==
                  ( ranging_settings.rng_address & 0xFF ) ) )
            {
                // If address matches, update channel and request count, prepare for TX
                current_channel                = rac_config->smtc_rac_data_buffer_setup.tx_payload_buffer[4];
                ranging_settings.rng_req_count = rac_config->smtc_rac_data_buffer_setup.tx_payload_buffer[5];
                multiple_data_rate_config      = rac_config->smtc_rac_data_buffer_setup.tx_payload_buffer[6];
                radio_pl_buffer[6]             = ( uint8_t ) rac_config->smtc_rac_data_result.rssi_result;

                rac_config->radio_params.lora.is_tx               = true;
                rac_config->radio_params.lora.is_ranging_exchange = false;
            }
            else
            {
                // If address does not match, restart RX to wait for the correct config
                RANGING_LOG_WARN(
                    "Subordinate Ranging config rx done -> received address does not match expected address\n" );
            }

            rac_config->scheduler_config.scheduling                     = ASAP_TRANSACTION;
            rac_config->scheduler_config.start_time_ms                  = smtc_modem_hal_get_time_in_ms( );
            rac_config->scheduler_config.callback_pre_radio_transaction = NULL;

            SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ranging_radio_access_id ) );

            break;
        case RP_STATUS_TASK_ABORTED:
        case RP_STATUS_RX_TIMEOUT:
        default:
            // On error or timeout, restart ranging exchange
            start_ranging_exchange( 0, false );
            break;
        }
        break;
    }
    case APP_RADIO_RANGING_START:
    {
        switch( status )
        {
        case RP_STATUS_RTTOF_RESP_DONE:
        case RP_STATUS_RTTOF_REQ_DISCARDED:
        case RP_STATUS_TASK_ABORTED:
        case RP_STATUS_RX_TIMEOUT:
            // After each ranging exchange, increment channel and schedule next exchange
            current_channel++;
            rac_config->scheduler_config.start_time_ms += ranging_settings.rng_req_delay;
            if( ( status == RP_STATUS_RX_TIMEOUT ) || ( status == RP_STATUS_TASK_ABORTED ) ||
                ( status == RP_STATUS_RTTOF_REQ_DISCARDED ) )
            {
                while( ( int ) ( rac_config->scheduler_config.start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) < 0 )
                {
                    rac_config->scheduler_config.start_time_ms += ranging_settings.rng_req_delay;
                    current_channel++;
                }
            }

            if( current_channel >= RANGING_HOPPING_CHANNELS_MAX )
            {
                if( results_callback != NULL )
                {
                    results_callback( &rac_config->radio_params.lora, &ranging_settings, &ranging_results, region );
                }

                // If all channels are done, reset state and LEDs, restart exchange
                RANGING_LOG_PRINTF( "\n" );
                current_channel = 0;
                set_led( SMTC_PF_LED_TX, false );
                set_led( SMTC_PF_LED_RX, true );
                start_ranging_exchange( 0, false );
                break;
            }
            else
            {
                RANGING_LOG_PRINTF( "." );
            }

            // Prepare next ranging exchange on the next channel
            rac_config->radio_params.lora.frequency_in_hz = ranging_hopping_channels_array[current_channel];
            //  rac_config->smtc_rac_data.ranging_result =
            //     ( rp_ranging_result_t* ) ( &ranging_results.rng_result[current_channel] );

            rac_config->scheduler_config.scheduling                     = ASAP_TRANSACTION;
            rac_config->scheduler_config.callback_pre_radio_transaction = &toggle_led;
            SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ranging_radio_access_id ) );
            break;

        default:
            // On error, restart ranging exchange
            start_ranging_exchange( 0, false );
            break;
        }
        break;
    }
    default:
        // On unknown state, restart ranging exchange
        start_ranging_exchange( 0, false );
        break;
    }
}

/*!
 * @brief State machine for the manager (master) node.
 *
 * This function handles all the states and transitions for the manager node during the ranging and frequency
 * hopping process. It is called by the RAC scheduler when a radio event occurs (TX done, RX done, timeout, etc.).
 *
 * The manager is responsible for:
 *   - Sending the initial ranging configuration to the subordinate node.
 *   - Waiting for the subordinate's acknowledgment.
 *   - Launching the ranging exchange on each hopping channel.
 *   - Collecting the ranging results (distance, RSSI) for each channel.
 *   - Computing the median distance at the end of the hopping sequence.
 *   - Handling errors and timeouts by restarting the process or returning to idle.
 *
 * The function uses a switch-case on the internal state (ranging_internal_state) to determine the current step:
 *   - APP_RADIO_IDLE: Prepares the radio and LEDs for idle state.
 *   - APP_RADIO_RANGING_CONFIG: Handles sending the config and waiting for the subordinate's response.
 *   - APP_RADIO_RANGING_START: Handles the actual ranging exchanges on each channel, collects results, and computes
 * the median.
 *   - Any error or unknown state returns to idle.
 *
 * The context parameter is a pointer to the radio process status (rp_status_t), which indicates the last radio
 * event. The function updates the global state, schedules the next radio transaction, and manages the hopping
 * sequence.
 */
static void ranging_state_machine_manager( rp_status_t status )
{
    uint32_t current_radio_timestamp_ms = ( rac_config->smtc_rac_data_result.radio_end_timestamp_ms );
    switch( ranging_internal_state )
    {
    case APP_RADIO_IDLE:
        // Setup radio and LEDs for idle state
        // app_radio_ranging_setup( NULL );
        set_led( SMTC_PF_LED_TX, true );
        set_led( SMTC_PF_LED_RX, false );
        break;
    case APP_RADIO_RANGING_CONFIG:
    {
        switch( status )
        {
        case RP_STATUS_TX_DONE:
            // After TX done, go to RX to wait for subordinate answer
            rac_config->radio_params.lora.is_tx                         = false;
            rac_config->radio_params.lora.rx_timeout_ms                 = 3000;
            rac_config->scheduler_config.scheduling                     = ASAP_TRANSACTION;
            rac_config->scheduler_config.start_time_ms                  = smtc_modem_hal_get_time_in_ms( );
            rac_config->scheduler_config.callback_pre_radio_transaction = NULL;
            SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ranging_radio_access_id ) );
            RANGING_LOG_INFO( "Manager Ranging config tx done  -> go to rx wait for subordinate answer\n" );
            break;

        case RP_STATUS_RX_PACKET:
            ranging_results.rssi_value = rac_config->smtc_rac_data_result.rssi_result;
            ranging_results.snr_value  = rac_config->smtc_rac_data_result.snr_result;

            // Received response from subordinate, start ranging exchange
            ranging_internal_state = APP_RADIO_RANGING_START;

            rac_config->radio_params.lora.is_tx               = true;
            rac_config->radio_params.lora.is_ranging_exchange = true;
            rac_config->radio_params.lora.frequency_in_hz     = ranging_hopping_channels_array[0];
            //  rac_config->smtc_rac_data.ranging_result = ( rp_ranging_result_t* ) ( &ranging_results.rng_result[0] );
            rac_config->scheduler_config.scheduling = SCHEDULE_TRANSACTION;

            rac_config->scheduler_config.callback_pre_radio_transaction = &toggle_led;
            if( activate_multiple_data_rate == true )
            {
                rac_config->radio_params.lora.sf = multiple_data_rate_config & 0x0F;  // Extract SF from payload
                rac_config->radio_params.lora.bw = multiple_data_rate_config >> 4;    // Extract BW from payload

                rac_config->radio_params.lora.rttof.bw_ranging = rac_config->radio_params.lora.bw;
            }
            common_rttof_recommended_rx_tx_delay_indicator(
                rac_config->radio_params.lora.frequency_in_hz, rac_config->radio_params.lora.bw,
                rac_config->radio_params.lora.sf, &rac_config->radio_params.lora.rttof.delay_indicator );
            ranging_settings.rng_req_delay =
                ( uint16_t ) ( get_single_symbol_time_ms( rac_config->radio_params.lora.bw,
                                                          rac_config->radio_params.lora.sf ) *
                               RANGING_ALL_SYMBOL ) +
                RANGING_DONE_PROCESSING_TIME + extra_delay_ms_for_multiple_data_rate;
            ranging_settings.rng_exch_timeout =
                ranging_settings.rng_req_delay >> 1;  // Set the timeout to half of the request delay
            rac_config->radio_params.lora.rx_timeout_ms = ranging_settings.rng_exch_timeout;
            rac_config->scheduler_config.start_time_ms  = current_radio_timestamp_ms + ranging_settings.rng_req_delay;
            SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ranging_radio_access_id ) );
            RANGING_LOG_INFO( "starting ranging exchange\n" );
            break;
        case RP_STATUS_TASK_ABORTED:
        case RP_STATUS_RX_TIMEOUT:
            // On timeout, restart config TX after a delay
            RANGING_LOG_WARN( "Manager Ranging config rx timeout  -> go to ranging config tx \n" );
            rac_config->radio_params.lora.is_tx                         = true;
            rac_config->scheduler_config.scheduling                     = ASAP_TRANSACTION;
            rac_config->scheduler_config.start_time_ms                  = smtc_modem_hal_get_time_in_ms( ) + 5000;
            rac_config->scheduler_config.callback_pre_radio_transaction = NULL;
            SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ranging_radio_access_id ) );
            break;
        default:
            // On error, go back to idle
            RANGING_LOG_ERROR(
                "Manager Ranging config error receive status = %d-> go to end of ranging exchange wait for press "
                "user "
                "button\n",
                status );
            ranging_internal_state = APP_RADIO_IDLE;
            break;
        }
        break;
    }
    case APP_RADIO_RANGING_START:
    {
        switch( status )
        {
        case RP_STATUS_TASK_ABORTED:
        case RP_STATUS_RTTOF_TIMEOUT:
        case RP_STATUS_RTTOF_EXCH_VALID:
            if( status == RP_STATUS_TASK_ABORTED )
            {
                RANGING_LOG_WARN( "APP_RADIO_RANGING_START aborted\n" );
            }
            else
            {
                RANGING_LOG_PRINTF( "." );
            }
            // After each ranging exchange, increment channel and schedule next exchange
            memcpy( ( uint8_t* ) &ranging_results.rng_result[current_channel],
                    ( uint8_t* ) &rac_config->smtc_rac_data_result.ranging_result, sizeof( rp_ranging_result_t ) );
            current_channel++;
            rac_config->scheduler_config.start_time_ms += ranging_settings.rng_req_delay;
            while( ( int ) ( rac_config->scheduler_config.start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) < 0 )
            {
                rac_config->scheduler_config.start_time_ms += ranging_settings.rng_req_delay;
                current_channel++;
            }
            if( current_channel >= RANGING_HOPPING_CHANNELS_MAX )
            {
                RANGING_LOG_PRINTF( "\n" );
                // If all channels are done, print last result, compute median, reset state and LEDs
                for( int i = 0; i < RANGING_HOPPING_CHANNELS_MAX; i++ )
                {
                    if( ranging_results.rng_result[i].distance_m == 0 && ranging_results.rng_result[i].rssi == 0 )
                    {
                        RANGING_LOG_RESULT( "Ranging result[%02d] freq = %u, distance = fail\n", ( i ),
                                            ranging_hopping_channels_array[i] );
                    }
                    else
                    {
                        RANGING_LOG_RESULT( "Ranging result[%02d] freq = %u, distance = %dm, rssi = %d\n", ( i ),
                                            ranging_hopping_channels_array[i], ranging_results.rng_result[i].distance_m,
                                            ranging_results.rng_result[i].rssi );
                    }
                }

                current_channel = 0;
                set_led( SMTC_PF_LED_TX, true );
                set_led( SMTC_PF_LED_RX, false );
                int32_t temp_distance[RANGING_HOPPING_CHANNELS_MAX] = { 0 };
                for( int i = 0; i < RANGING_HOPPING_CHANNELS_MAX; i++ )
                {
                    if( ranging_results.rng_result[i].distance_m == 0 && ranging_results.rng_result[i].rssi == 0 )
                    {
                        temp_distance[i] = invalid_value;
                    }
                    else
                    {
                        temp_distance[i] = ranging_results.rng_result[i].distance_m;
                        ranging_results.cnt_packet_rx_ok++;
                    }
                }
                int32_t distance             = compute_median( temp_distance, RANGING_HOPPING_CHANNELS_MAX );
                ranging_results.rng_distance = distance;
                ranging_results.rng_per =
                    ( int ) ( ( RANGING_HOPPING_CHANNELS_MAX - ranging_results.cnt_packet_rx_ok ) * 100 /
                              RANGING_HOPPING_CHANNELS_MAX );

                if( results_callback != NULL )
                {
                    results_callback( &rac_config->radio_params.lora, &ranging_settings,
                                      &ranging_results, region);
                }
                else
                {
                    RANGING_LOG_RESULT( " Distance Median computed: %" PRIi32 " m\n", distance >= 0 ? distance : 0 );
                }
                ranging_internal_state = APP_RADIO_IDLE;
                if( activate_multiple_data_rate == true || continuous_ranging == true )
                {
                    start_ranging_exchange( 0, true );
                }
                break;
            }

            // Prepare next ranging exchange on the next channel
            rac_config->radio_params.lora.frequency_in_hz = ranging_hopping_channels_array[current_channel];
            //  rac_config->smtc_rac_data.ranging_result =
            //    ( rp_ranging_result_t* ) ( &ranging_results.rng_result[current_channel] );

            rac_config->scheduler_config.scheduling                     = SCHEDULE_TRANSACTION;
            rac_config->scheduler_config.callback_pre_radio_transaction = &toggle_led;
            SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ranging_radio_access_id ) );
            /**
             * Calls the smtc_rac_lora function and then prints the result.
             *
             * Note: The print statement must be placed after the call to smtc_rac_lora
             * to avoid timing issues that may occur if printing is done before the function
             * completes its execution.
             */
            //  RANGING_LOG_PRINTF( "Ranging result[%d]  distance = %.4dm and rssi = %d\n", ( current_channel - 1
            //  ),
            //                         ranging_results.rng_result[current_channel - 1].distance_m,
            //                         ranging_results.rng_result[current_channel - 1].rssi );
            break;
        default:
            // On error, go back to idle
            RANGING_LOG_ERROR(
                "Manager Ranging APP_RADIO_RANGING_START config error receive status = %d-> go to end of ranging "
                "exchange wait for press user "
                "button\n",
                status );
            ranging_internal_state = APP_RADIO_IDLE;
            break;
        }
        break;
    }
    default:
        // On unknown state, go back to idle
        RANGING_LOG_ERROR(
            "Manager Ranging APP_RADIO_RANGING_START config error receive status = %d-> go to end of ranging "
            "exchange wait for press user "
            "button\n",
            status );
        ranging_internal_state = APP_RADIO_IDLE;
        break;
    }
}

/*!
 * @brief Get pointer to current radio settings (not implemented)
 */
ranging_params_settings_t* app_ranging_get_radio_settings( void )
{
    return NULL;
}

/*!
 * @brief Reset ranging parameters (not implemented)
 */
void app_ranging_params_reset( void )
{
}

/*!
 * @brief Get the frequency of a hopping channel by index (not implemented)
 */
uint32_t get_ranging_hopping_channels( uint8_t index )
{
    return ranging_hopping_channels_array[index];
}

/*!
 * @brief Set the current state of the ranging process (not implemented)
 */
void set_ranging_process_state( uint8_t state )
{
}

/*!
 * @brief Start a ranging exchange, either as manager or subordinate
 * @param delay_ms Delay before starting the exchange
 * @param is_manager true if this node is the manager, false if subordinate
 */

static void increase_current_bw( void )
{
    if( repeat_cnt % REPEAT_CNT == 0 )
        current_bw++;

    if( current_bw > RAL_LORA_BW_1000_KHZ )
        current_bw = RAL_LORA_BW_125_KHZ;
}

void start_ranging_exchange( uint16_t delay_ms, bool is_manager )
{
    app_radio_ranging_setup( NULL );
    if( is_manager == true )
    {
        if( activate_multiple_data_rate == true )
        {
            RANGING_LOG_INFO( "\n" );
            RANGING_LOG_INFO( "--------> SEQ: %d\n", ( repeat_cnt % REPEAT_CNT ) + 1 );
            repeat_cnt++;

            RANGING_LOG_CONFIG( "Manager Ranging exchange with multiple data rate enabled:\n" );
            RANGING_LOG_CONFIG( "  current_sf = %s, current_bw = %s\n", ral_lora_sf_to_str( current_sf ),
                                ral_lora_bw_to_str( current_bw ) );

            multiple_data_rate_config = current_sf + ( current_bw << 4 );
            radio_pl_buffer[6]        = multiple_data_rate_config;

            if( repeat_cnt > REPEAT_CNT * BW_CNT )
            {
                RANGING_LOG_INFO( "Tests finished.\n" );
                //results_callback( 1234567890, 128, rac_config->radio_params.lora.bw, 0, 0, region );
                while( 1 )
                    ;
            }
            increase_current_bw( );
        }
        else
        {
            RANGING_LOG_CONFIG( "Manager Ranging exchange with single data rate enabled\n" );
        }
        rac_config->radio_params.lora.is_tx                         = true;
        rac_config->scheduler_config.scheduling                     = ASAP_TRANSACTION;
        rac_config->scheduler_config.start_time_ms                  = smtc_modem_hal_get_time_in_ms( ) + delay_ms;
        rac_config->scheduler_config.callback_pre_radio_transaction = NULL;
    }
    else
    {
        rac_config->radio_params.lora.is_tx                         = false;
        rac_config->radio_params.lora.rx_timeout_ms                 = 64000;  // 64 seconds
        rac_config->scheduler_config.scheduling                     = ASAP_TRANSACTION;
        rac_config->scheduler_config.start_time_ms                  = smtc_modem_hal_get_time_in_ms( );
        rac_config->scheduler_config.callback_pre_radio_transaction = NULL;
        set_led( SMTC_PF_LED_TX, false );
        set_led( SMTC_PF_LED_RX, true );
    }

    SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( ranging_radio_access_id ) );

    ranging_internal_state = APP_RADIO_RANGING_CONFIG;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

/*!
 * @brief Lookup table for recommended delay indicator for frequencies below 600MHz
 */
static const uint32_t rttof_delay_indicator_table_below_600mhz[7][8] = {
    /* SF5,  SF6,   SF7,   SF8,   SF9,   SF10,  SF11,  SF12 */
    { 19701, 19669, 19576, 19397, 19043, 18323, 18323, 19104 },  // BW125
    { 23964, 24009, 24075, 24189, 24298, 24659, 24659, 19104 },  // BW203
    { 20186, 20180, 20118, 20040, 19838, 19470, 19470, 20312 },  // BW250
    { 24424, 24418, 24410, 24466, 24433, 24460, 20375, 20378 },  // BW406
    { 20605, 20642, 20606, 20605, 20528, 20428, 20375, 20378 },  // BW500
    { 25475, 25570, 25533, 25597, 25531, 25671, 20375, 20378 },  // BW812
    { 21685, 21747, 21721, 21685, 21657, 21584, 20375, 20378 },  // BW1000
};

/*!
 * @brief Lookup table for recommended delay indicator for frequencies from 600MHz to 2GHz
 */
static const uint32_t rttof_delay_indicator_table_from_600mhz_to_2ghz[7][8] = {
    /* SF5,  SF6,   SF7,   SF8,   SF9,   SF10,  SF11,  SF12 */
    { 19747, 19707, 19628, 19480, 19166, 18589, 19036, 19024 },  // BW125
    { 17498, 17502, 17515, 17606, 17722, 18024, 19036, 19024 },  // BW203
    { 20150, 20133, 20102, 20033, 19847, 19537, 20236, 20232 },  // BW250
    { 17768, 17791, 17868, 17997, 18123, 18456, 20295, 20298 },  // BW406
    { 20599, 20590, 20567, 20512, 20295, 19961, 20295, 20298 },  // BW500
    { 18681, 18738, 18763, 18874, 18737, 18824, 20295, 20298 },  // BW812
    { 21700, 21705, 21783, 21834, 21689, 21571, 20295, 20298 },  // BW1000
};

/*!
 * @brief Lookup table for recommended delay indicator for frequencies above 2GHz
 */
static const uint32_t rttof_delay_indicator_table_above_2ghz[7][8] = {
    /* SF5,  SF6,   SF7,   SF8,   SF9,   SF10,  SF11,  SF12 */
    { 19582, 19498, 19330, 19012, 18368, 17125, 19036, 19024 },  // BW125
    { 17173, 17262, 17335, 17554, 17828, 18557, 19036, 19024 },  // BW203
    { 19938, 19896, 19818, 19646, 19316, 18667, 20236, 20232 },  // BW250
    { 17767, 17822, 17869, 17937, 18119, 18442, 20295, 20298 },  // BW406
    { 20588, 20586, 20550, 20451, 20287, 19938, 20295, 20298 },  // BW500
    { 18698, 18777, 18848, 18981, 19047, 19449, 20295, 20298 },  // BW812
    { 21574, 21611, 21622, 20095, 21370, 21009, 20295, 20298 },  // BW1000
};

/*!
 * @brief Get the recommended RX/TX delay indicator for a given frequency, bandwidth, and spreading factor
 * @param rf_freq_in_hz Frequency in Hz
 * @param bw LoRa bandwidth
 * @param sf LoRa spreading factor
 * @param delay_indicator Output: pointer to the delay indicator value
 * @return true if successful, false if parameters are invalid
 */
static bool common_rttof_recommended_rx_tx_delay_indicator( uint32_t rf_freq_in_hz, ral_lora_bw_t bw, ral_lora_sf_t sf,
                                                            uint32_t* delay_indicator )
{
    uint8_t  row_index;
    uint8_t  column_index;
    uint32_t offset = 0;

    *delay_indicator = 0u;

    // Select row in table based on bandwidth
    switch( bw )
    {
    case RAL_LORA_BW_125_KHZ:
        row_index = 0;
        break;
    case RAL_LORA_BW_200_KHZ:
        row_index = 1;
        break;
    case RAL_LORA_BW_250_KHZ:
        row_index = 2;
        break;
    case RAL_LORA_BW_400_KHZ:
        row_index = 3;
        break;
    case RAL_LORA_BW_500_KHZ:
        row_index = 4;
        break;
    case RAL_LORA_BW_800_KHZ:
        row_index = 5;
        break;
    case RAL_LORA_BW_1000_KHZ:
        row_index = 6;
        break;
    default:
        return false;
    }

    // Select column in table based on spreading factor
    switch( sf )
    {
    case RAL_LORA_SF5:
        column_index = 0;
        break;
    case RAL_LORA_SF6:
        column_index = 1;
        break;
    case RAL_LORA_SF7:
        column_index = 2;
        break;
    case RAL_LORA_SF8:
        column_index = 3;
        break;
    case RAL_LORA_SF9:
        column_index = 4;
        break;
    case RAL_LORA_SF10:
        column_index = 5;
        break;
    case RAL_LORA_SF11:
        column_index = 6;
        break;
    case RAL_LORA_SF12:
        column_index = 7;
        break;
    default:
        return false;
    }

    // Select the correct table based on frequency
    if( rf_freq_in_hz < 600000000 )
    {
        *delay_indicator = rttof_delay_indicator_table_below_600mhz[row_index][column_index] + offset;
    }
    else if( ( 600000000 <= rf_freq_in_hz ) && ( rf_freq_in_hz < 2000000000 ) )
    {
        *delay_indicator = rttof_delay_indicator_table_from_600mhz_to_2ghz[row_index][column_index] + offset;
    }
    else
    {
        *delay_indicator = rttof_delay_indicator_table_above_2ghz[row_index][column_index] + offset;
    }

    return true;
}

/*!
 * @brief Compute the time of a single LoRa symbol in milliseconds
 * @param bw LoRa bandwidth
 * @param sf LoRa spreading factor
 * @return Symbol time in ms, or 0 if parameters are invalid
 */
static float get_single_symbol_time_ms( ral_lora_bw_t bw, ral_lora_sf_t sf )
{
    float bw_value;

    if( sf < RAL_LORA_SF5 || sf > RAL_LORA_SF12 )
    {
        return 0;
    }

    switch( bw )
    {
    case RAL_LORA_BW_125_KHZ:
        bw_value = 125;
        break;
    case RAL_LORA_BW_200_KHZ:
        bw_value = 200;
        break;
    case RAL_LORA_BW_250_KHZ:
        bw_value = 250;
        break;
    case RAL_LORA_BW_400_KHZ:
        bw_value = 400;
        break;
    case RAL_LORA_BW_500_KHZ:
        bw_value = 500;
        break;
    case RAL_LORA_BW_800_KHZ:
        bw_value = 800;
        break;
    case RAL_LORA_BW_1000_KHZ:
        bw_value = 1000;
        break;
    default:
        return 0;
    }

    return ( ( float ) ( 1 << sf ) / bw_value );
}

/*!
 * @brief Compute the median value of an array of int32_t
 * Ignores invalid values (invalid measurements)
 * @param array Pointer to the array of values
 * @param size Number of elements in the array
 * @return The median value
 */
static int32_t compute_median( int32_t* array, uint8_t size )
{
    int32_t median = 0;
    uint8_t i;

    if( size == 0 )
    {
        return 0;
    }

    // Sort the array in ascending order (simple bubble sort)
    for( i = 0; i < size - 1; i++ )
    {
        for( uint8_t j = i + 1; j < size; j++ )
        {
            if( array[i] > array[j] )  // Avoid swapping with zero
            {
                int32_t temp = array[i];
                array[i]     = array[j];
                array[j]     = temp;
            }
        }
    }
    // Remove invalid values from the sorted array
    uint8_t non_zero_count = 0;
    for( i = 0; i < size; i++ )
    {
        if( array[i] != invalid_value )
        {
            array[non_zero_count++] = array[i];
        }
    }
    size = non_zero_count;
    // Compute the median
    if( size == 0 )
    {
        return 0;  // No valid values
    }
    else if( size % 2 == 0 && size > 0 )
    {
        median = ( array[size / 2 - 1] + array[size / 2] ) / 2;
    }
    else if( size > 0 )
    {
        median = array[size / 2];
    }
    return median;
}
