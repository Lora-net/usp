/**
 * @file      app_cad.c
 *
 * @brief     CAD (Channel Activity Detection) example for LR20xx chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

#include "app_cad.h"

#include <stddef.h>
#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "CAD"
#include "smtc_rac_log.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// Backward compatibility alias for existing code
#define CAD_PRINT( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct cad_s
{
    uint8_t             radio_access_id;  // store result of `smtc_rac_open_radio`
    bool                is_running;       // indicates if CAD operation is active
    smtc_rac_context_t* transaction;      // associated transaction
} cad_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static cad_t cad = {
    .radio_access_id = 0,  // set in `cad_init`
    .transaction     = NULL,
    .is_running      = false,
};

// CAD statistics
static uint32_t cad_count          = 0;  // Total number of CAD operations
static uint32_t cad_positive_count = 0;  // Number of positive CAD results
static uint32_t cad_negative_count = 0;  // Number of negative CAD results
static uint32_t last_cad_positive  = 0;  // CAD number of last positive detection
static uint8_t  cad_buffer[255]    = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Schedule CAD actions in RAC
 */
static void schedule_cad( void );

/**
 * @brief Called after the radio transaction.
 *
 * This callback is called when the radio transaction is finished
 *
 * @param [in] status Result of radio operation
 */
static void post_cad_callback( rp_status_t status );

/**
 * @brief Callback to be called before radio transaction
 *
 * This callback is to be called when the radio is available, when direct radio access through RAL or driver calls are
 * possible
 * */
static void pre_cad_callback( void );

/* -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void cad_init( void )
{
    // Register direct radio access from RAC
    cad.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_VERY_HIGH_PRIORITY ) );
    cad.transaction     = smtc_rac_get_context( cad.radio_access_id );
    CAD_PRINT( "CAD initialized - press button to start CAD operation\n" );
    cad.transaction->scheduler_config.callback_post_radio_transaction = post_cad_callback;
    cad.transaction->scheduler_config.scheduling                      = SMTC_RAC_ASAP_TRANSACTION;
    cad.transaction->scheduler_config.callback_pre_radio_transaction  = pre_cad_callback;
    cad.transaction->modulation_type                                  = SMTC_RAC_MODULATION_LORA;
    cad.transaction->cad_context.cad_enabled                          = true;
    cad.transaction->cad_context.cad_symb_nb                          = RAL_LORA_CAD_04_SYMB;
    cad.transaction->cad_context.cad_exit_mode                        = TYPE_OF_CAD;
    cad.transaction->cad_context.cad_timeout_in_ms                    = CAD_DURATION_MS;
    cad.transaction->cad_context.sf                                   = LORA_SPREADING_FACTOR;
    cad.transaction->cad_context.bw                                   = LORA_BANDWIDTH;
    cad.transaction->cad_context.rf_freq_in_hz                        = FREQ_IN_HZ;
    cad.transaction->cad_context.invert_iq_is_on                      = LORA_IQ;

    cad.transaction->radio_params.lora.is_tx                      = ( TYPE_OF_CAD == RAL_LORA_CAD_RX ) ? false : true;
    cad.transaction->radio_params.lora.is_ranging_exchange        = false;
    cad.transaction->radio_params.lora.frequency_in_hz            = FREQ_IN_HZ;
    cad.transaction->radio_params.lora.tx_power_in_dbm            = TX_OUTPUT_POWER_DBM;
    cad.transaction->radio_params.lora.sf                         = LORA_SPREADING_FACTOR;
    cad.transaction->radio_params.lora.bw                         = LORA_BANDWIDTH;
    cad.transaction->radio_params.lora.cr                         = LORA_CODING_RATE;
    cad.transaction->radio_params.lora.preamble_len_in_symb       = LORA_PREAMBLE_LENGTH;
    cad.transaction->radio_params.lora.header_type                = LORA_PKT_LEN_MODE;
    cad.transaction->radio_params.lora.invert_iq_is_on            = LORA_IQ;
    cad.transaction->radio_params.lora.crc_is_on                  = LORA_CRC;
    cad.transaction->radio_params.lora.sync_word                  = LORA_SYNCWORD;
    cad.transaction->radio_params.lora.rx_timeout_ms              = 0;  // not used
    cad.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer = cad_buffer;
    cad.transaction->smtc_rac_data_buffer_setup.rx_payload_buffer = cad_buffer;
    cad.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( cad_buffer );
    cad.transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = sizeof( cad_buffer );
    cad.transaction->radio_params.lora.max_rx_size                        = sizeof( cad_buffer );
    cad.transaction->radio_params.lora.tx_size                            = PAYLOAD_SIZE;
    // Initialize statistics
    cad_count          = 0;
    cad_positive_count = 0;
    cad_negative_count = 0;
    last_cad_positive  = 0;
}

void cad_on_button_press( void )
{
    if( cad.is_running )
    {
        CAD_PRINT( "Radio is busy\n" );
    }
    else
    {
        schedule_cad( );
        cad.is_running = true;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void pre_cad_callback( void )
{
    set_led( SMTC_PF_LED_TX, true );
}

static void post_cad_callback( rp_status_t status )
{
    set_led( SMTC_PF_LED_TX, false );

    // Increment CAD counter
    cad_count++;

    switch( status )
    {
    case RP_STATUS_CAD_POSITIVE:
        cad_positive_count++;
        last_cad_positive = cad_count;
        CAD_PRINT( ">>> CAD #%lu: POSITIVE ( %lu positive %lu, %lu negative, Last positive: #%lu)\n", cad_count,
                   cad_positive_count, cad_negative_count, last_cad_positive );
        break;

    case RP_STATUS_CAD_NEGATIVE:
        cad_negative_count++;
        CAD_PRINT( ">>> CAD #%lu: NEGATIVE ( %lu positive, %lu negative, Last positive: #%lu)\n", cad_count,
                   cad_positive_count, cad_negative_count, last_cad_positive > 0 ? last_cad_positive : 0 );
        break;

    case RP_STATUS_TASK_ABORTED:
        CAD_PRINT( ">>> CAD #%lu: ABORTED ( %lu positive, %lu negative, Last positive: #%lu)\n", cad_count,
                   cad_positive_count, cad_negative_count, last_cad_positive > 0 ? last_cad_positive : 0 );
        break;
    case RP_STATUS_TX_DONE:  // when activate CAD_TO_TX feature
        cad_negative_count++;
        CAD_PRINT( ">>> CAD #%lu: TX DONE ( %lu positive, %lu negative, Last positive: #%lu)\n", cad_count,
                   cad_positive_count, cad_negative_count, last_cad_positive > 0 ? last_cad_positive : 0 );
        break;
    case RP_STATUS_RX_PACKET:  // when activate CAD_TO_RX feature
        cad_positive_count++;
        CAD_PRINT( ">>> CAD #%lu: RX PACKET ( %lu positive, %lu negative, Last positive: #%lu)\n", cad_count,
                   cad_positive_count, cad_negative_count, last_cad_positive > 0 ? last_cad_positive : 0 );
        break;
    case RP_STATUS_RX_TIMEOUT:
        cad_positive_count++;
        CAD_PRINT( ">>> CAD #%lu: RX TIMEOUT ( %lu positive, %lu negative, Last positive: #%lu)\n", cad_count,
                   cad_positive_count, cad_negative_count, last_cad_positive > 0 ? last_cad_positive : 0 );
        break;
    default:
        CAD_PRINT( ">>> CAD #%lu: ERROR ( %lu positive, %lu negative, Last positive: #%lu)\n", cad_count,
                   cad_positive_count, cad_negative_count, last_cad_positive > 0 ? last_cad_positive : 0 );
        break;
    }

    schedule_cad( );
}

void schedule_cad( void )
{
    cad.transaction->scheduler_config.start_time_ms    = smtc_modem_hal_get_time_in_ms( ) + CAD_DELAY_MS;
    cad.transaction->scheduler_config.duration_time_ms = CAD_DURATION_MS;

    smtc_rac_return_code_t return_code = SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( cad.radio_access_id ) );
    if( return_code != SMTC_RAC_SUCCESS )
    {
        CAD_PRINT( "Failed to schedule CAD operation: %d\n", return_code );
    }
}
