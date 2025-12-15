/**
 * @file      app_lrfhss_example.c
 *
 * @brief     LR-FHSS application implementation
 *
 * This example demonstrates how to use LR-FHSS modulation with the USP/RAC framework.
 * LR-FHSS is transmission-only, so this example only shows transmission functionality.
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
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
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

#include "app_lrfhss_example.h"
#include "smtc_rac_api.h"

#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "LRFHSS-APP"
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum lrfhss_state_e
{
    LRFHSS_STATE_IDLE = 0,
    LRFHSS_STATE_TRANSMITTING,
    LRFHSS_STATE_WAITING,
} lrfhss_state_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint8_t        tx_payload[LRFHSS_PAYLOAD_SIZE];
static uint8_t        radio_id       = RAC_INVALID_RADIO_ID;
static uint32_t       tx_count       = 0;
static uint32_t       next_tx_time   = 0;
static lrfhss_state_t current_state  = LRFHSS_STATE_IDLE;
static bool           is_initialized = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void lrfhss_radio_callback( rp_status_t status );
static void prepare_tx_payload( void );
static void schedule_next_transmission( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lrfhss_example_init( void )
{
    if( is_initialized )
    {
        SMTC_HAL_TRACE_WARNING( "LR-FHSS application already initialized\n" );
        return;
    }

    // Open radio access
    radio_id = smtc_rac_open_radio( RAC_HIGH_PRIORITY );

    if( radio_id == RAC_INVALID_RADIO_ID )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to open radio access\n" );
        return;
    }

    SMTC_HAL_TRACE_INFO( "Radio access opened with ID: %d\n", radio_id );
    SMTC_HAL_TRACE_INFO( "LR-FHSS Configuration:\n" );
    SMTC_HAL_TRACE_INFO( "  - Frequency: %u Hz\n", LRFHSS_FREQUENCY_HZ );
    SMTC_HAL_TRACE_INFO( "  - TX Power: %d dBm\n", LRFHSS_TX_POWER_DBM );
    SMTC_HAL_TRACE_INFO( "  - Payload Size: %d bytes\n", LRFHSS_PAYLOAD_SIZE );
    SMTC_HAL_TRACE_INFO( "  - TX Interval: %d ms\n", LRFHSS_TX_INTERVAL_MS );

    // Initialize state
    current_state = LRFHSS_STATE_IDLE;
    tx_count      = 0;
    next_tx_time  = 0;

    is_initialized = true;
    SMTC_HAL_TRACE_INFO( "LR-FHSS application initialized successfully\n" );
}

bool lrfhss_example_process( void )
{
    if( !is_initialized )
    {
        SMTC_HAL_TRACE_ERROR( "LR-FHSS application not initialized\n" );
        return false;
    }

    // Check if it's time for the next transmission
    if( ( current_state == LRFHSS_STATE_WAITING ) && ( smtc_modem_hal_get_time_in_ms( ) >= next_tx_time ) )
    {
        current_state = LRFHSS_STATE_TRANSMITTING;
        schedule_next_transmission( );
    }

    return true;
}

void lrfhss_example_start_cycle( void )
{
    if( !is_initialized )
    {
        SMTC_HAL_TRACE_ERROR( "LR-FHSS application not initialized\n" );
        return;
    }

    if( current_state == LRFHSS_STATE_IDLE )
    {
        SMTC_HAL_TRACE_INFO( "Starting LR-FHSS transmission cycle\n" );
        current_state = LRFHSS_STATE_TRANSMITTING;
        next_tx_time  = smtc_modem_hal_get_time_in_ms( ) + 1000;  // Start in 1 second
        schedule_next_transmission( );
    }
    else
    {
        SMTC_HAL_TRACE_INFO( "LR-FHSS transmission cycle already in progress\n" );
    }
}

void lrfhss_example_stop( void )
{
    if( !is_initialized )
    {
        return;
    }

    SMTC_HAL_TRACE_INFO( "Stopping LR-FHSS application\n" );
    current_state = LRFHSS_STATE_IDLE;

    // Abort any pending transmission
    if( radio_id != RAC_INVALID_RADIO_ID )
    {
        smtc_rac_abort_radio_submit( radio_id );
    }
}

bool lrfhss_example_is_active( void )
{
    return is_initialized && ( current_state != LRFHSS_STATE_IDLE );
}

uint32_t lrfhss_example_get_tx_count( void )
{
    return tx_count;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lrfhss_radio_callback( rp_status_t status )
{
    switch( status )
    {
    case RP_STATUS_TX_DONE:
        SMTC_HAL_TRACE_INFO(
            "TX : "
            "LR-FHSS TX #%" PRIu32 " completed successfully\n",
            tx_count );
        current_state = LRFHSS_STATE_WAITING;

        // Schedule next transmission
        next_tx_time = smtc_modem_hal_get_time_in_ms( ) + LRFHSS_TX_INTERVAL_MS;
        break;

    case RP_STATUS_TASK_ABORTED:
        SMTC_HAL_TRACE_WARNING( "LR-FHSS TX #%" PRIu32 " aborted\n", tx_count );
        current_state = LRFHSS_STATE_WAITING;

        // Retry after a short delay
        next_tx_time = smtc_modem_hal_get_time_in_ms( ) + 1000;
        break;

    default:
        SMTC_HAL_TRACE_INFO( "LR-FHSS status: %d\n", status );
        current_state = LRFHSS_STATE_WAITING;
        next_tx_time  = smtc_modem_hal_get_time_in_ms( ) + 1000;
        break;
    }
}

static void prepare_tx_payload( void )
{
    // Create a simple payload with incrementing counter and random data
    tx_payload[0] = 'L';  // LR-FHSS marker
    tx_payload[1] = 'R';
    tx_payload[2] = '-';
    tx_payload[3] = 'F';
    tx_payload[4] = 'H';
    tx_payload[5] = 'S';
    tx_payload[6] = 'S';
    tx_payload[7] = ':';

    // Add 32-bit counter (big-endian)
    tx_payload[8]  = ( tx_count >> 24 ) & 0xFF;
    tx_payload[9]  = ( tx_count >> 16 ) & 0xFF;
    tx_payload[10] = ( tx_count >> 8 ) & 0xFF;
    tx_payload[11] = tx_count & 0xFF;

    // Fill rest with random data
    for( int i = 12; i < LRFHSS_PAYLOAD_SIZE; i++ )
    {
        tx_payload[i] = smtc_modem_hal_get_random_nb_in_range( 0, 255 );
    }

    SMTC_HAL_TRACE_ARRAY( "TX Payload", tx_payload, LRFHSS_PAYLOAD_SIZE );
}

static void schedule_next_transmission( void )
{
    // Get RAC context
    smtc_rac_context_t* rac_context = smtc_rac_get_context( radio_id );
    if( rac_context == NULL )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to get RAC context\n" );
        current_state = LRFHSS_STATE_WAITING;
        next_tx_time  = smtc_modem_hal_get_time_in_ms( ) + 2000;
        return;
    }

    // Increment transmission counter
    tx_count++;

    // Prepare new payload
    prepare_tx_payload( );

    // Configure LR-FHSS parameters
    rac_context->modulation_type                     = SMTC_RAC_MODULATION_LRFHSS;
    rac_context->radio_params.lrfhss.is_tx           = true;  // LR-FHSS is transmission only
    rac_context->radio_params.lrfhss.frequency_in_hz = LRFHSS_FREQUENCY_HZ;
    rac_context->radio_params.lrfhss.tx_power_in_dbm = LRFHSS_TX_POWER_DBM;

    // LR-FHSS specific parameters - optimized for good range and reliability
    rac_context->radio_params.lrfhss.coding_rate     = LR_FHSS_V1_CR_1_3;  // Coding rate 1/3 for better sensitivity
    rac_context->radio_params.lrfhss.bandwidth       = LR_FHSS_V1_BW_136719_HZ;  // 136.719 kHz bandwidth
    rac_context->radio_params.lrfhss.grid            = LR_FHSS_V1_GRID_3906_HZ;  // 3.906 kHz grid step
    rac_context->radio_params.lrfhss.enable_hopping  = true;                     // Enable frequency hopping
    rac_context->radio_params.lrfhss.sync_word       = NULL;                     // Use default sync word
    rac_context->radio_params.lrfhss.device_offset   = 0;                        // No device offset
    rac_context->radio_params.lrfhss.hop_sequence_id = 0;                        // Auto-generate hop sequence

    // Configure payload data
    rac_context->smtc_rac_data_buffer_setup.tx_payload_buffer         = tx_payload;
    rac_context->radio_params.lrfhss.tx_size                          = LRFHSS_PAYLOAD_SIZE;
    rac_context->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( tx_payload );

    // Configure scheduler
    rac_context->scheduler_config.start_time_ms                   = next_tx_time;
    rac_context->scheduler_config.scheduling                      = SMTC_RAC_ASAP_TRANSACTION;
    rac_context->scheduler_config.callback_pre_radio_transaction  = NULL;
    rac_context->scheduler_config.callback_post_radio_transaction = lrfhss_radio_callback;

    // Schedule the transmission
    smtc_rac_return_code_t result = smtc_rac_submit_radio_transaction( radio_id );
    if( result != SMTC_RAC_SUCCESS )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to schedule LR-FHSS transmission: %d\n", result );
        current_state = LRFHSS_STATE_WAITING;
        // Retry after a delay
        next_tx_time = smtc_modem_hal_get_time_in_ms( ) + 2000;
    }
    else
    {
        SMTC_HAL_TRACE_INFO(
            "TX "
            "LR-FHSS TX #%" PRIu32 " scheduled for %" PRIu32 " ms\n",
            tx_count, next_tx_time );
    }
}
