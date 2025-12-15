/**
 * @file      app_periodic_uplink.c
 *
 * @brief     Simple periodic uplink example for LR1110 or LR1120 chip
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

#include "app_periodic_uplink.h"
#include "main_ping_pong.h"

#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "PERIODIC"
#include "smtc_rac_log.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
// Backward compatibility alias for existing code
#define PERIODIC_PRINT( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#ifdef ENABLE_PERIODIC_UPLINK
static const bool ENABLED = true;
#else
static const bool ENABLED = false;
#endif

static const smtc_rac_lora_syncword_t SYNC_WORD = LORA_PUBLIC_NETWORK_SYNCWORD;  // (alias)
static const uint32_t                 DELAY     = 10000;                         // ms

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct periodic_s
{
    uint8_t             radio_access_id;         // store result of `smtc_rac_open_radio`
    smtc_rac_context_t* transaction;             // associated transaction
    uint8_t             periodic_tx_payload[4];  // Payload buffer for periodic uplink
} periodic_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static periodic_t periodic = { .radio_access_id     = 0,  // set in `periodic_uplink_init`
                               .transaction         = NULL,
                               .periodic_tx_payload = { 'L', 'o', 'R', 'a' } };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/** @brief set `periodic.transaction` and start the transmission */
static void periodic_tx( void );

/** @brief switch on `SMTC_LED_TX` */
static void pre_periodic_callback( void );

/** @brief switch off `SMTC_PF_LED_TX` and call `periodic_tx` again */
static void post_periodic_callback( rp_status_t status );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void periodic_uplink_init( void )
{
    // initialize static struct
    periodic.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_VERY_HIGH_PRIORITY ) );
    periodic.transaction     = smtc_rac_get_context( periodic.radio_access_id );

    periodic.transaction->modulation_type                                      = SMTC_RAC_MODULATION_LORA;
    periodic.transaction->radio_params.lora.is_tx                              = true;
    periodic.transaction->radio_params.lora.is_ranging_exchange                = false;
    periodic.transaction->radio_params.lora.frequency_in_hz                    = RF_FREQ_IN_HZ;
    periodic.transaction->radio_params.lora.tx_power_in_dbm                    = TX_OUTPUT_POWER_DBM;
    periodic.transaction->radio_params.lora.sf                                 = LORA_SPREADING_FACTOR;
    periodic.transaction->radio_params.lora.bw                                 = LORA_BANDWIDTH;
    periodic.transaction->radio_params.lora.cr                                 = LORA_CODING_RATE;
    periodic.transaction->radio_params.lora.preamble_len_in_symb               = LORA_PREAMBLE_LENGTH;
    periodic.transaction->radio_params.lora.header_type                        = LORA_PKT_LEN_MODE;
    periodic.transaction->radio_params.lora.invert_iq_is_on                    = LORA_IQ;
    periodic.transaction->radio_params.lora.crc_is_on                          = LORA_CRC;
    periodic.transaction->radio_params.lora.sync_word                          = SYNC_WORD;
    periodic.transaction->radio_params.lora.rx_timeout_ms                      = 0;  // not used
    periodic.transaction->radio_params.lora.tx_size                            = sizeof( periodic.periodic_tx_payload );
    periodic.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = periodic.periodic_tx_payload;
    periodic.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( periodic.periodic_tx_payload );
    periodic.transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = NULL;
    periodic.transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = 0;
    periodic.transaction->scheduler_config.start_time_ms                       = 0;  // set at each transaction
    periodic.transaction->scheduler_config.scheduling                          = SMTC_RAC_SCHEDULED_TRANSACTION;
    periodic.transaction->scheduler_config.callback_pre_radio_transaction      = pre_periodic_callback;
    periodic.transaction->scheduler_config.callback_post_radio_transaction     = post_periodic_callback;

    if( ENABLED )
    {
        // start application
        periodic_tx( );
    }
}

void periodic_uplink_on_button_press( void )
{
    // nothing to do
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void periodic_tx( void )
{
    PERIODIC_PRINT( "sending transmission with delay=%dms\n", DELAY );
    periodic.transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + DELAY;
    smtc_rac_return_code_t error_code =
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( periodic.radio_access_id ) );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( error_code == SMTC_RAC_SUCCESS );
}

static void pre_periodic_callback( void )
{
    PERIODIC_PRINT( "usp/rac: transaction is starting\n" );
    set_led( SMTC_PF_LED_TX, true );
}

static void post_periodic_callback( rp_status_t status )
{
    set_led( SMTC_PF_LED_TX, false );

    switch( status )
    {
    case RP_STATUS_TX_DONE:
        PERIODIC_PRINT( "usp/rac: new event: transaction (transmission) has ended (success)\n" );
        break;

    default:
        PERIODIC_PRINT( "usp/rac: new event: ERROR: unexpected transaction status: %d\n", ( int ) status );
        break;
    }

    periodic_tx( );
}
