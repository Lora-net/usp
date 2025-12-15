/**
 * @file      app_spectral_scan.c
 *
 * @brief     Spectral scan application implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer in the
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "app_spectral_scan.h"
#include "main_spectral_scan.h"

#include "radio_planner_types.h"
#include "smtc_rac_api.h"
#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"
// #include "smtc_hal_mcu.h"
#include "ral_defs.h"

// USP/RAC logging system
#define RAC_LOG_APP_PREFIX "SPECTRAL"
#include "smtc_rac_log.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

#define LOG_BANNER( app_name, version )                                \
    do                                                                 \
    {                                                                  \
        SMTC_HAL_TRACE_INFO( "================================\n" );   \
        SMTC_HAL_TRACE_INFO( "  %s v%s\n", app_name, version );        \
        SMTC_HAL_TRACE_INFO( "  Built: %s %s\n", __DATE__, __TIME__ ); \
        SMTC_HAL_TRACE_INFO( "================================\n" );   \
    } while( 0 )

/**
 * @brief Spectral Scan Configuration Parameters
 */

#ifndef FREQ_START_HZ
#define FREQ_START_HZ 866500000UL /*!< First channel frequency (Hz) */
#endif

#ifndef NB_CHAN
#define NB_CHAN 3 /*!< Number of channels to scan */
#endif

#ifndef NB_SCAN
#define NB_SCAN 50 /*!< RSSI samples per channel */
#endif

#ifndef WIDTH_CHAN_HZ
#define WIDTH_CHAN_HZ 1000000UL /*!< Channel spacing (Hz) */
#endif

#ifndef RSSI_TOP_LEVEL_DBM
#define RSSI_TOP_LEVEL_DBM 0 /*!< Highest RSSI value (dBm) */
#endif

#ifndef RSSI_BOTTOM_LEVEL_DBM
#define RSSI_BOTTOM_LEVEL_DBM -128 /*!< Lowest RSSI value (dBm) */
#endif

#ifndef RSSI_SCALE
#define RSSI_SCALE 4 /*!< RSSI scale (dBm per bin) */
#endif

#define RSSI_HISTOGRAM_BINS ( ( RSSI_TOP_LEVEL_DBM - RSSI_BOTTOM_LEVEL_DBM ) / RSSI_SCALE + 1 )
#define FREQ_END_HZ ( FREQ_START_HZ + ( NB_CHAN - 1 ) * WIDTH_CHAN_HZ )

#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH RAL_LORA_BW_125_KHZ
#endif

#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR 7
#endif

#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE 1  // CR 4/5
#endif

/**
 * @brief Operation delays
 */
#define RSSI_MEASUREMENT_DELAY_MS 100
#define CHANNEL_CHANGE_DELAY_MS 50
#define MAX_RADIO_RETRIES 3
#define LBT_BW_HZ_DEFAULT ( 200000 )
#define LAP_OF_TIME_TO_GET_A_RSSI_VALID 8
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Application version
 */
static const char* SPECTRAL_SCAN_APP_VERSION = "1.0.0";

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Spectral scan statistics for one channel
 */
typedef struct
{
    uint32_t frequency_hz;                   /*!< Channel frequency in Hz */
    uint32_t histogram[RSSI_HISTOGRAM_BINS]; /*!< RSSI histogram bins */
    uint16_t total_samples;                  /*!< Total number of samples */
    int16_t  min_rssi_dbm;                   /*!< Minimum RSSI measured */
    int16_t  max_rssi_dbm;                   /*!< Maximum RSSI measured */
    int16_t  avg_rssi_dbm;                   /*!< Average RSSI measured */
} spectral_scan_channel_stats_t;

/**
 * @brief Spectral scan context with RAC integration
 */
typedef struct
{
    uint32_t                      scan_cycle_count;       /*!< Total number of scan cycles completed */
    uint32_t                      current_channel;        /*!< Current channel being scanned */
    int16_t                       rssi_dbm_inst;          /*!< RSSI value in dBm */
    uint32_t                      current_scan_count;     /*!< Current scan count for this channel */
    bool                          scan_active;            /*!< Scan active flag */
    bool                          manual_scan_requested;  /*!< Manual scan requested flag */
    smtc_rac_context_t*           rac_transaction;        /*!< Pointer to RAC transaction context */
    spectral_scan_channel_stats_t channel_stats[NB_CHAN]; /*!< Statistics for all channels */
} spectral_scan_context_t;

/**
 * @brief Spectral scan application state
 */
typedef enum
{
    SPECTRAL_SCAN_IDLE,
    SPECTRAL_SCAN_MEASURING,
    SPECTRAL_SCAN_WAITING_BETWEEN_MEASUREMENTS,
    SPECTRAL_SCAN_WAITING_BETWEEN_CHANNELS,
    SPECTRAL_SCAN_CYCLE_COMPLETE,
    SPECTRAL_SCAN_ERROR
} spectral_scan_state_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Global spectral scan context
 */
static spectral_scan_context_t spectral_scan_ctx;

/**
 * @brief Current application state
 */
static spectral_scan_state_t current_state = SPECTRAL_SCAN_IDLE;

/**
 * @brief RAC radio access ID
 */
static uint8_t radio_access_id = 0;

/**
 * @brief Application initialized flag
 */
static bool app_initialized = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION ------------------------------------------
 */

/**
 * @brief Initialize RAC radio for spectral scan
 * @return true if initialization successful, false otherwise
 */
static bool spectral_scan_init_radio( void );

/**
 * @brief Start RSSI measurement for current channel using RAC API
 */
static void spectral_scan_start_channel_measurement( void );

/**
 * @brief Start next channel measurement or complete cycle
 */
static void spectral_scan_next_channel_or_complete( void );

/**
 * @brief Pre-transaction callback (called before radio operation starts)
 */
static void spectral_scan_pre_callback( void );

/**
 * @brief Post-transaction callback (called after radio operation completes)
 * @param [in] status Radio operation status
 */
static void spectral_scan_post_callback( rp_status_t status );

/**
 * @brief Initialize spectral scan context
 * @param [out] ctx Pointer to spectral scan context to initialize
 */
static void spectral_scan_init_context( spectral_scan_context_t* ctx );

/**
 * @brief Display spectral scan results
 * @param [in] ctx Pointer to spectral scan context with results
 */
static void spectral_scan_display_results( const spectral_scan_context_t* ctx );

/**
 * @brief Get histogram bin index for given RSSI value
 * @param [in] rssi_dbm RSSI value in dBm
 * @return uint32_t Histogram bin index
 */
static uint32_t spectral_scan_get_histogram_bin( int16_t rssi_dbm );

/**
 * @brief Display ASCII histogram for a single channel
 * @param [in] stats Pointer to channel statistics
 */
static void spectral_scan_display_ascii_histogram( const spectral_scan_channel_stats_t* stats );

/**
 * @brief Display overall spectral scan summary with ASCII graphics
 * @param [in] ctx Pointer to spectral scan context with results
 */
static void spectral_scan_display_summary( const spectral_scan_context_t* ctx );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION --------------------------------------------
 */

void spectral_scan_init( void )
{
    if( app_initialized )
    {
        SMTC_HAL_TRACE_WARNING( "Spectral scan already initialized\n" );
        return;
    }

    LOG_BANNER( "Spectral Scan Application", SPECTRAL_SCAN_APP_VERSION );

    SMTC_HAL_TRACE_INFO(
        "CONFIG "
        "Spectral Scan Configuration:\n" );
    SMTC_HAL_TRACE_INFO(
        "CONFIG "
        "  - Frequency range: %lu.%03lu - %lu.%03lu MHz\n",
        ( unsigned long ) ( FREQ_START_HZ / 1000000 ), ( unsigned long ) ( ( FREQ_START_HZ % 1000000 ) / 1000 ),
        ( unsigned long ) ( FREQ_END_HZ / 1000000 ), ( unsigned long ) ( ( FREQ_END_HZ % 1000000 ) / 1000 ) );
    SMTC_HAL_TRACE_INFO(
        "CONFIG "
        "  - Number of channels: %d\n",
        NB_CHAN );
    SMTC_HAL_TRACE_INFO(
        "CONFIG "
        "  - Channel spacing: %lu.%03lu MHz\n",
        ( unsigned long ) ( WIDTH_CHAN_HZ / 1000000 ), ( unsigned long ) ( ( WIDTH_CHAN_HZ % 1000000 ) / 1000 ) );
    SMTC_HAL_TRACE_INFO(
        "CONFIG "
        "  - Scans per channel: %d\n",
        NB_SCAN );

    SMTC_HAL_TRACE_INFO(
        "CONFIG "
        "  - RSSI range: %d to %d dBm\n",
        RSSI_BOTTOM_LEVEL_DBM, RSSI_TOP_LEVEL_DBM );
    SMTC_HAL_TRACE_INFO(
        "CONFIG "
        "  - RSSI scale: %d dBm/bin\n",
        RSSI_SCALE );
    SMTC_HAL_TRACE_INFO(
        "CONFIG "
        "  - Histogram bins: %d\n",
        RSSI_HISTOGRAM_BINS );

    // Initialize spectral scan context
    spectral_scan_init_context( &spectral_scan_ctx );

    // Initialize radio
    if( !spectral_scan_init_radio( ) )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to initialize radio\n" );
        current_state = SPECTRAL_SCAN_ERROR;
        return;
    }

    app_initialized = true;
    SMTC_HAL_TRACE_INFO( "Spectral scan application initialized successfully\n" );
}

void spectral_scan_start_cycle( void )
{
    if( !app_initialized )
    {
        SMTC_HAL_TRACE_ERROR( "Application not initialized\n" );
        return;
    }

    if( current_state != SPECTRAL_SCAN_IDLE )
    {
        SMTC_HAL_TRACE_WARNING( "Scan already in progress or error state\n" );
        return;
    }
    spectral_scan_start_channel_measurement( );
}

void spectral_scan_stop( void )
{
    if( !app_initialized )
    {
        return;
    }

    SMTC_HAL_TRACE_INFO( "Stopping spectral scan\n" );
    spectral_scan_ctx.scan_active           = false;
    spectral_scan_ctx.manual_scan_requested = false;
    current_state                           = SPECTRAL_SCAN_IDLE;
}

bool spectral_scan_is_active( void )
{
    return app_initialized && spectral_scan_ctx.scan_active;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION -------------------------------------------
 */

static bool spectral_scan_init_radio( void )
{
    // Open radio access with post callback
    radio_access_id = smtc_rac_open_radio( RAC_MEDIUM_PRIORITY );
    if( radio_access_id == RAC_INVALID_RADIO_ID )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to open radio access\n" );
        return false;
    }

    // Get the RAC transaction context
    spectral_scan_ctx.rac_transaction = smtc_rac_get_context( radio_access_id );

    spectral_scan_ctx.rac_transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = NULL;
    spectral_scan_ctx.rac_transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = NULL;
    spectral_scan_ctx.rac_transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = 0;
    spectral_scan_ctx.rac_transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = 0;

    SMTC_HAL_TRACE_INFO( "Radio initialized successfully (ID: %d)\n", radio_access_id );
    return true;
}

static void spectral_scan_start_channel_measurement( void )
{
    // Reset channel scan parameters
    spectral_scan_ctx.current_channel    = 0;
    spectral_scan_ctx.current_scan_count = 0;
    spectral_scan_ctx.scan_active        = true;
    current_state                        = SPECTRAL_SCAN_MEASURING;

    SMTC_HAL_TRACE_INFO(
        "Starting scan on channel %" PRIu32 " (%lu.%03lu MHz)\n", spectral_scan_ctx.current_channel,
        ( unsigned long ) ( ( FREQ_START_HZ + spectral_scan_ctx.current_channel * WIDTH_CHAN_HZ ) / 1000000 ),
        ( unsigned long ) ( ( ( FREQ_START_HZ + spectral_scan_ctx.current_channel * WIDTH_CHAN_HZ ) % 1000000 ) /
                            1000 ) );

    // Configure for first channel

    spectral_scan_ctx.rac_transaction->lbt_context.listen_duration_ms = RSSI_MEASUREMENT_DELAY_MS;
    spectral_scan_ctx.rac_transaction->lbt_context.bandwidth_hz       = LBT_BW_HZ_DEFAULT;

    // Schedule immediate execution

    // Schedule immediate execution
    spectral_scan_ctx.rac_transaction->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    spectral_scan_ctx.rac_transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( );
    spectral_scan_ctx.rac_transaction->scheduler_config.callback_post_radio_transaction = spectral_scan_post_callback;
    spectral_scan_ctx.rac_transaction->scheduler_config.callback_pre_radio_transaction  = spectral_scan_pre_callback;
    spectral_scan_ctx.rac_transaction->scheduler_config.duration_time_ms                = RSSI_MEASUREMENT_DELAY_MS;

    smtc_rac_return_code_t error_code =
        smtc_rac_lock_radio_access( radio_access_id, spectral_scan_ctx.rac_transaction->scheduler_config );

    if( error_code != SMTC_RAC_SUCCESS )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to start LoRa RX, error: %d\n", error_code );
        current_state                 = SPECTRAL_SCAN_ERROR;
        spectral_scan_ctx.scan_active = false;
    }
}

static void spectral_scan_next_channel_or_complete( void )
{
    spectral_scan_ctx.current_channel++;

    if( spectral_scan_ctx.current_channel >= NB_CHAN )
    {
        // All channels scanned, complete the cycle
        spectral_scan_ctx.scan_cycle_count++;
        spectral_scan_display_results( &spectral_scan_ctx );
        spectral_scan_ctx.scan_active = false;
        current_state                 = SPECTRAL_SCAN_IDLE;

        return;
    }

    // Start next channel
    spectral_scan_ctx.current_scan_count = 0;
    SMTC_HAL_TRACE_INFO(
        "Starting scan on channel %" PRIu32 " (%lu.%03lu MHz)\n", spectral_scan_ctx.current_channel,
        ( unsigned long ) ( ( FREQ_START_HZ + spectral_scan_ctx.current_channel * WIDTH_CHAN_HZ ) / 1000000 ),
        ( unsigned long ) ( ( ( FREQ_START_HZ + spectral_scan_ctx.current_channel * WIDTH_CHAN_HZ ) % 1000000 ) /
                            1000 ) );

    // Configure for first channel

    spectral_scan_ctx.rac_transaction->lbt_context.listen_duration_ms = RSSI_MEASUREMENT_DELAY_MS;
    spectral_scan_ctx.rac_transaction->lbt_context.bandwidth_hz       = LBT_BW_HZ_DEFAULT;

    // Schedule immediate execution
    spectral_scan_ctx.rac_transaction->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    spectral_scan_ctx.rac_transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( );
    spectral_scan_ctx.rac_transaction->scheduler_config.callback_post_radio_transaction = spectral_scan_post_callback;
    spectral_scan_ctx.rac_transaction->scheduler_config.callback_pre_radio_transaction  = spectral_scan_pre_callback;
    spectral_scan_ctx.rac_transaction->scheduler_config.duration_time_ms                = RSSI_MEASUREMENT_DELAY_MS;

    smtc_rac_return_code_t error_code =
        smtc_rac_lock_radio_access( radio_access_id, spectral_scan_ctx.rac_transaction->scheduler_config );

    if( error_code != SMTC_RAC_SUCCESS )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to start LoRa RX, error: %d\n", error_code );
        current_state                 = SPECTRAL_SCAN_ERROR;
        spectral_scan_ctx.scan_active = false;
    }
}

static void spectral_scan_pre_callback( void )
{
    // Prepare radio parameters for RSSI measurement
    ralf_params_gfsk_t gfsk_param;
    rp_radio_params_t  radio_params;

    memset( &radio_params, 0, sizeof( rp_radio_params_t ) );
    memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

    gfsk_param.rf_freq_in_hz      = FREQ_START_HZ + ( spectral_scan_ctx.current_channel * WIDTH_CHAN_HZ );
    gfsk_param.pkt_params.dc_free = RAL_GFSK_DC_FREE_WHITENING;

    // Use bandwidth to configure modulation parameters
    gfsk_param.mod_params.br_in_bps    = LBT_BW_HZ_DEFAULT >> 1;
    gfsk_param.mod_params.fdev_in_hz   = LBT_BW_HZ_DEFAULT >> 2;
    gfsk_param.mod_params.bw_dsb_in_hz = LBT_BW_HZ_DEFAULT;
    gfsk_param.mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;

    radio_params.pkt_type         = RAL_PKT_TYPE_GFSK;
    radio_params.rx.gfsk          = gfsk_param;
    radio_params.rx.timeout_in_ms = RSSI_MEASUREMENT_DELAY_MS;
    radio_params.lbt_threshold    = -130;

    radio_planner_t* rp = smtc_rac_get_rp( );
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_pkt_type( &( rp->radio->ral ), RAL_PKT_TYPE_GFSK ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rf_freq( &( rp->radio->ral ), radio_params.rx.gfsk.rf_freq_in_hz ) ==
                                     RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_gfsk_mod_params( &( rp->radio->ral ), &( radio_params.rx.gfsk.mod_params ) ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_NONE ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rx( &( rp->radio->ral ), RAL_RX_TIMEOUT_CONTINUOUS_MODE ) ==
                                     RAL_STATUS_OK );

    uint32_t carrier_sense_time = smtc_modem_hal_get_time_in_ms( );
    while( ( int32_t ) ( carrier_sense_time + LAP_OF_TIME_TO_GET_A_RSSI_VALID - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {  // delay LAP_OF_TIME_TO_GET_A_RSSI_VALID ms
    }

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_get_rssi_inst( &( rp->radio->ral ), &spectral_scan_ctx.rssi_dbm_inst ) ==
                                     RAL_STATUS_OK );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_standby( &( rp->radio->ral ), RAL_STANDBY_CFG_RC ) == RAL_STATUS_OK );

    smtc_rac_unlock_radio_access( radio_access_id );

    return;
}

static void spectral_scan_post_callback( rp_status_t status )
{
    switch( status )
    {
    case RP_STATUS_RADIO_UNLOCKED:
    {
        // Get RSSI measurement from radio
        int16_t rssi_dbm = spectral_scan_ctx.rssi_dbm_inst;

        // Update statistics for current channel
        if( spectral_scan_ctx.current_channel < NB_CHAN )
        {
            spectral_scan_channel_stats_t* stats = &spectral_scan_ctx.channel_stats[spectral_scan_ctx.current_channel];

            uint32_t bin = spectral_scan_get_histogram_bin( rssi_dbm );
            stats->histogram[bin]++;
            stats->total_samples++;

            // Update min/max/average
            if( rssi_dbm < stats->min_rssi_dbm )
            {
                stats->min_rssi_dbm = rssi_dbm;
            }
            if( rssi_dbm > stats->max_rssi_dbm )
            {
                stats->max_rssi_dbm = rssi_dbm;
            }

            // Calculate running average
            if( stats->total_samples == 1 )
            {
                stats->avg_rssi_dbm = rssi_dbm;
            }
            else
            {
                // Use double for precise running average to avoid floor/rounding errors
                double avg = ( ( double ) stats->avg_rssi_dbm * ( stats->total_samples - 1 ) + ( double ) rssi_dbm ) /
                             ( double ) stats->total_samples;
                stats->avg_rssi_dbm = ( int16_t ) ( avg + ( ( avg < 0 ) ? -0.5 : 0.5 ) );  // round to nearest int
            }

            SMTC_HAL_TRACE_INFO(
                "RX "
                "Ch%" PRIu32 ": RSSI %d dBm [bin %" PRIu32 "] Avg: %d dBm\n",
                spectral_scan_ctx.current_channel, rssi_dbm, bin, stats->avg_rssi_dbm );
        }

        // Increment scan count for this channel
        spectral_scan_ctx.current_scan_count++;

        if( spectral_scan_ctx.current_scan_count < NB_SCAN )
        {
            // More measurements needed for this channel
            current_state = SPECTRAL_SCAN_WAITING_BETWEEN_MEASUREMENTS;

            // Schedule next measurement with delay

            spectral_scan_ctx.rac_transaction->scheduler_config.start_time_ms =
                smtc_modem_hal_get_time_in_ms( ) + RSSI_MEASUREMENT_DELAY_MS;

            smtc_rac_return_code_t error_code =
                smtc_rac_lock_radio_access( radio_access_id, spectral_scan_ctx.rac_transaction->scheduler_config );
            if( error_code != SMTC_RAC_SUCCESS )
            {
                SMTC_HAL_TRACE_ERROR( "Failed to start next measurement, error: %d\n", error_code );
                current_state                 = SPECTRAL_SCAN_ERROR;
                spectral_scan_ctx.scan_active = false;
            }
        }
        else
        {
            // Channel measurements complete, move to next channel
            spectral_scan_next_channel_or_complete( );
        }
        break;
    }
    case RP_STATUS_TASK_ABORTED:
        SMTC_HAL_TRACE_ERROR( "Radio aborted\n" );
        current_state                 = SPECTRAL_SCAN_ERROR;
        spectral_scan_ctx.scan_active = false;
        break;
    default:
        SMTC_HAL_TRACE_ERROR( "Unexpected radio status: %d\n", status );
        current_state                 = SPECTRAL_SCAN_ERROR;
        spectral_scan_ctx.scan_active = false;
        break;
    }
}

static void spectral_scan_init_context( spectral_scan_context_t* ctx )
{
    if( ctx == NULL )
    {
        return;
    }

    memset( ctx, 0, sizeof( spectral_scan_context_t ) );

    // Initialize channel statistics
    for( uint32_t i = 0; i < NB_CHAN; i++ )
    {
        ctx->channel_stats[i].frequency_hz = FREQ_START_HZ + ( i * WIDTH_CHAN_HZ );
        ctx->channel_stats[i].min_rssi_dbm = RSSI_TOP_LEVEL_DBM;
        ctx->channel_stats[i].max_rssi_dbm = RSSI_BOTTOM_LEVEL_DBM;
    }

    ctx->scan_active           = false;
    ctx->scan_cycle_count      = 0;
    ctx->manual_scan_requested = false;
}

static void spectral_scan_display_results( const spectral_scan_context_t* ctx )
{
    if( ctx == NULL )
    {
        return;
    }

    SMTC_HAL_TRACE_INFO(
        "STATS "
        "=== Spectral Scan Results (Cycle %" PRIu32 ") ===\n",
        ctx->scan_cycle_count );

    for( uint32_t i = 0; i < NB_CHAN; i++ )
    {
        const spectral_scan_channel_stats_t* stats = &ctx->channel_stats[i];

        if( stats->total_samples == 0 )
        {
            continue;
        }

        // Display frequency and histogram on same line
        char histogram_str[RSSI_HISTOGRAM_BINS * 6 + 1] = { 0 };
        char temp_str[8];

        for( uint32_t bin = 0; bin < RSSI_HISTOGRAM_BINS; bin++ )
        {
            snprintf( temp_str, sizeof( temp_str ), "%" PRIu32 " ", stats->histogram[bin] );
            strcat( histogram_str, temp_str );
        }

        SMTC_HAL_TRACE_INFO( "%lu.%03lu MHz: %s\n", ( unsigned long ) ( stats->frequency_hz / 1000000 ),
                             ( unsigned long ) ( ( stats->frequency_hz % 1000000 ) / 1000 ), histogram_str );
        SMTC_HAL_TRACE_INFO( "  Stats: %u samples, Min: %d dBm, Max: %d dBm, Avg: %d dBm\n", stats->total_samples,
                             stats->min_rssi_dbm, stats->max_rssi_dbm, stats->avg_rssi_dbm );

        // Display ASCII histogram for detailed view
        spectral_scan_display_ascii_histogram( stats );
    }

    // Display overall summary with ASCII graphics
    spectral_scan_display_summary( ctx );

    SMTC_HAL_TRACE_INFO(
        "STATS "
        "=== End Spectral Scan Cycle %" PRIu32 " ===\n",
        ctx->scan_cycle_count );
}

static uint32_t spectral_scan_get_histogram_bin( int16_t rssi_dbm )
{
    if( rssi_dbm > RSSI_TOP_LEVEL_DBM )
    {
        return 0;
    }

    if( rssi_dbm < RSSI_BOTTOM_LEVEL_DBM )
    {
        return RSSI_HISTOGRAM_BINS - 1;
    }

    uint32_t bin = ( RSSI_TOP_LEVEL_DBM - rssi_dbm ) / RSSI_SCALE;

    return bin;
}

static void spectral_scan_display_ascii_histogram( const spectral_scan_channel_stats_t* stats )
{
    if( stats == NULL || stats->total_samples == 0 )
    {
        return;
    }

    // Find max value for scaling
    uint32_t max_count = 0;
    for( uint32_t i = 0; i < RSSI_HISTOGRAM_BINS; i++ )
    {
        if( stats->histogram[i] > max_count )
        {
            max_count = stats->histogram[i];
        }
    }

    if( max_count == 0 )
    {
        return;
    }

    // ASCII histogram height (number of lines)
    const uint32_t max_height = 8;
    // Buffer for each line: margin("    ") + histogram + terminator(\0)
    // the full block character ("█") is 3 bytes wide
    char line_buffer[4 + 3 * RSSI_HISTOGRAM_BINS + 1];

    SMTC_HAL_TRACE_INFO( "    ASCII Histogram for %lu.%03lu MHz:\n",
                         ( unsigned long ) ( stats->frequency_hz / 1000000 ),
                         ( unsigned long ) ( ( stats->frequency_hz % 1000000 ) / 1000 ) );

    // Display histogram from top to bottom
    for( int32_t height = max_height; height > 0; height-- )
    {
        strcpy( line_buffer, "    " );
        for( uint32_t bin = 0; bin < RSSI_HISTOGRAM_BINS; bin++ )
        {
            uint32_t bar_height = ( stats->histogram[bin] * max_height ) / max_count;
            if( bar_height >= ( uint32_t ) height )
            {
                strcat( line_buffer, "█" );  // Full block character
            }
            else
            {
                strcat( line_buffer, " " );
            }
        }
        SMTC_HAL_TRACE_INFO( "%s\n", line_buffer );
    }

    // Display RSSI axis
    strcpy( line_buffer, "    " );
    for( uint32_t bin = 0; bin < RSSI_HISTOGRAM_BINS; bin++ )
    {
        if( bin % 4 == 0 )
        {  // Show every 4th mark
            strcat( line_buffer, "|" );
        }
        else
        {
            strcat( line_buffer, "-" );
        }
    }
    SMTC_HAL_TRACE_INFO( "%s\n", line_buffer );

    // Display RSSI values axis using the same line_buffer as above for consistency
    // The left margin here matches the histogram ("    ")
    strcpy( line_buffer, "  " );
    for( uint32_t bin = 0; bin < RSSI_HISTOGRAM_BINS; bin++ )
    {
        if( bin % 8 == 0 )
        {  // Show every 8th value
            int16_t rssi_val = RSSI_TOP_LEVEL_DBM - ( bin * RSSI_SCALE );
            char    temp[8];
            snprintf( temp, sizeof( temp ), "%4d", rssi_val );
            strcat( line_buffer, temp );
            strcat( line_buffer, "    " );
        }
    }
    strcat( line_buffer, " dBm" );
    SMTC_HAL_TRACE_INFO( "%s\n", line_buffer );
}

static void spectral_scan_display_summary( const spectral_scan_context_t* ctx )
{
    if( ctx == NULL )
    {
        return;
    }

    SMTC_HAL_TRACE_INFO(
        "STATS "
        "\n=== SPECTRAL SCAN SUMMARY ===\n" );

    SMTC_HAL_TRACE_INFO( "Frequency Range: %lu - %lu kHz\n", ( unsigned long ) ( FREQ_START_HZ / 1000 ),
                         ( unsigned long ) ( FREQ_END_HZ / 1000 ) );
    SMTC_HAL_TRACE_INFO( "Channels Scanned: %d\n", NB_CHAN );
    SMTC_HAL_TRACE_INFO( "Scan Cycle: %" PRIu32 "\n", ctx->scan_cycle_count );

    // Display a visual heat map of the average RSSI for each channel

    SMTC_HAL_TRACE_INFO( "\nSpectrum Heat Map (Avg RSSI):\n" );
    SMTC_HAL_TRACE_INFO( "Freq(MHz)  Avg RSSI  Heat\n" );

    // Define thresholds for the heatmap (in dBm)
    // The higher the RSSI (closer to 0), the fuller the bar
    // Map the range [RSSI_BOTTOM_LEVEL_DBM, RSSI_TOP_LEVEL_DBM] to 20 levels
    const uint32_t bar_max  = 20;
    const int16_t  rssi_min = RSSI_BOTTOM_LEVEL_DBM;
    const int16_t  rssi_max = RSSI_TOP_LEVEL_DBM;

    for( uint32_t i = 0; i < NB_CHAN; i++ )
    {
        const spectral_scan_channel_stats_t* stats = &ctx->channel_stats[i];

        if( stats->total_samples == 0 )
        {
            continue;
        }

        int16_t avg_rssi = stats->avg_rssi_dbm;

        // Clamp avg_rssi within the range [rssi_min, rssi_max]
        if( avg_rssi < rssi_min )
            avg_rssi = rssi_min;
        if( avg_rssi > rssi_max )
            avg_rssi = rssi_max;

        // Calculate the bar length (the higher the RSSI, the longer the bar)
        // Invert the scale: -128dBm = low noise, 0dBm = high noise
        uint32_t bar_length = 0;
        if( rssi_max > rssi_min )
        {
            bar_length = ( uint32_t ) ( ( ( int32_t ) ( avg_rssi - rssi_min ) * bar_max ) / ( rssi_max - rssi_min ) );
        }

        // Display frequency
        uint32_t freq_khz = stats->frequency_hz / 1000;
        uint32_t mhz      = freq_khz / 1000;
        uint32_t dec      = ( freq_khz % 1000 ) / 100;  // tenths of MHz

        // Build the heatmap bar
        char bar_buffer[3 * bar_max + 2];  // "█" is 3 bytes wide, +2 for left margin and null terminator
        strcpy( bar_buffer, " " );
        for( uint32_t j = 0; j < bar_max; j++ )
        {
            if( j < bar_length )
            {
                strcat( bar_buffer, "█" );
            }
            else
            {
                strcat( bar_buffer, " " );
            }
        }

        SMTC_HAL_TRACE_INFO( "%4lu.%1lu    %4d dBm  [%s]  \n", ( unsigned long ) mhz, ( unsigned long ) dec,
                             ( int ) stats->avg_rssi_dbm, bar_buffer );
    }

    // Display legend
    SMTC_HAL_TRACE_INFO( "\nLegend:\n" );
    SMTC_HAL_TRACE_INFO( "  █ = High activity    (numbers) = total samples\n" );
    SMTC_HAL_TRACE_INFO( "  Each channel represents %lu.%01lu MHz bandwidth\n",
                         ( unsigned long ) ( WIDTH_CHAN_HZ / 1000000 ),
                         ( unsigned long ) ( ( WIDTH_CHAN_HZ % 1000000 ) / 100000 ) );
    SMTC_HAL_TRACE_INFO(
        "STATS "
        "=== END SUMMARY ===\n" );
}
