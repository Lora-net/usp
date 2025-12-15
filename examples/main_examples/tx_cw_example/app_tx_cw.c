/**
 * @file      app_tx_cw.c
 *
 * @brief     Simple continuous transmission (TX CW) example for LR1110 or LR1120 chip
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

#include <inttypes.h>

#include "app_tx_cw.h"
#include "main_tx_cw.h"

#include <stddef.h>
#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "TX-CW"
#include "smtc_rac_log.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
// Backward compatibility alias for existing code
#define TX_CW_PRINT( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if defined( INFINITE_PREAMBLE )
static const bool IS_INFINITE_PREAMBLE_MODE = true;
#else
static const bool IS_INFINITE_PREAMBLE_MODE = false;
#endif

static const smtc_rac_lora_syncword_t SYNC_WORD = LORA_PUBLIC_NETWORK_SYNCWORD;  // (alias)
static const uint32_t                 TX_DELAY  = 1000;                          // ms (1 second between transmissions)

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct tx_cw_s
{
    uint8_t             radio_access_id;  // store result of `smtc_rac_open_radio`
    smtc_rac_context_t* transaction;      // associated transaction
    bool                is_running;       // indicates if continuous transmission is active
    uint32_t            packet_count;     // number of packets sent
} tx_cw_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static tx_cw_t tx_cw = {
    .radio_access_id = 0,  // set in `tx_cw_init`
    .transaction     = NULL,
    .is_running      = false,

};
static uint8_t payload[255] = { 0 };
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/** @brief set `tx_cw.transaction` and start the transmission */
static void tx_cw_send_packet( void );

/** @brief switch on `SMTC_LED_TX` */
static void pre_tx_callback( void );

/** @brief switch off `SMTC_LED_TX` and call `tx_cw_send_packet` again if running */
static void post_tx_callback( rp_status_t status );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void tx_cw_init( void )
{
    // initialize static struct
    tx_cw.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_VERY_HIGH_PRIORITY ) );
    tx_cw.transaction     = smtc_rac_get_context( tx_cw.radio_access_id );
    tx_cw.transaction->scheduler_config.callback_post_radio_transaction     = post_tx_callback;
    tx_cw.transaction->modulation_type                                      = PACKET_TYPE;
    tx_cw.transaction->radio_params.lora.is_tx                              = true;
    tx_cw.transaction->radio_params.lora.is_ranging_exchange                = false;
    tx_cw.transaction->radio_params.lora.frequency_in_hz                    = RF_FREQ_IN_HZ;
    tx_cw.transaction->radio_params.lora.tx_power_in_dbm                    = TX_OUTPUT_POWER_DBM;
    tx_cw.transaction->radio_params.lora.sf                                 = LORA_SPREADING_FACTOR;
    tx_cw.transaction->radio_params.lora.bw                                 = LORA_BANDWIDTH;
    tx_cw.transaction->radio_params.lora.cr                                 = LORA_CODING_RATE;
    tx_cw.transaction->radio_params.lora.preamble_len_in_symb               = LORA_PREAMBLE_LENGTH;
    tx_cw.transaction->radio_params.lora.header_type                        = LORA_PKT_LEN_MODE;
    tx_cw.transaction->radio_params.lora.invert_iq_is_on                    = LORA_IQ;
    tx_cw.transaction->radio_params.lora.crc_is_on                          = LORA_CRC;
    tx_cw.transaction->radio_params.lora.sync_word                          = SYNC_WORD;
    tx_cw.transaction->radio_params.lora.rx_timeout_ms                      = 0;  // not used
    tx_cw.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = payload;
    tx_cw.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( payload );
    tx_cw.transaction->radio_params.lora.tx_size                            = PAYLOAD_SIZE;
    tx_cw.transaction->scheduler_config.start_time_ms                       = 0;  // set at each transaction
    tx_cw.transaction->scheduler_config.scheduling                          = SMTC_RAC_SCHEDULED_TRANSACTION;
    tx_cw.transaction->scheduler_config.callback_pre_radio_transaction      = pre_tx_callback;
    tx_cw.transaction->cw_context.cw_enabled                                = true;
    tx_cw.transaction->cw_context.infinite_preamble                         = IS_INFINITE_PREAMBLE_MODE;

    TX_CW_PRINT( "TX CW initialized - press button to start/stop continuous transmission\n" );
}

void tx_cw_on_button_press( void )
{
    if( tx_cw.is_running )
    {
        tx_cw_stop( );
    }
    else
    {
        tx_cw_start( );
    }
}

void tx_cw_start( void )
{
    if( !tx_cw.is_running )
    {
        tx_cw.is_running   = true;
        tx_cw.packet_count = 0;
        TX_CW_PRINT( "Starting continuous transmission...\n" );
        if( tx_cw.transaction->cw_context.infinite_preamble == true )
        {
            TX_CW_PRINT( "Infinite preamble\n" );
        }
        else
        {
            TX_CW_PRINT( "Continuous wave\n" );
        }
        TX_CW_PRINT( "Modulation type: %s\n", PACKET_TYPE == SMTC_RAC_MODULATION_LORA ? "LORA" : "FSK" );
        TX_CW_PRINT( "Frequency: %u Hz\n", RF_FREQ_IN_HZ );
        TX_CW_PRINT( "Power: %d dBm\n", TX_OUTPUT_POWER_DBM );
        TX_CW_PRINT( "Spread factor: %s\n", LORA_SPREADING_FACTOR == RAL_LORA_SF7 ? "SF7" : "SF9" );
        TX_CW_PRINT( "Bandwidth: %s\n", LORA_BANDWIDTH == RAL_LORA_BW_125_KHZ ? "125 kHz" : "500 kHz" );
        TX_CW_PRINT( "Coding rate: %s\n", LORA_CODING_RATE == RAL_LORA_CR_4_5 ? "4/5" : "4/6" );
        TX_CW_PRINT( "Preamble length: %u\n", LORA_PREAMBLE_LENGTH );
        TX_CW_PRINT( "Packet length mode: %s\n", LORA_PKT_LEN_MODE == RAL_LORA_PKT_EXPLICIT ? "Explicit" : "Implicit" );
        tx_cw_send_packet( );
    }
}

void tx_cw_stop( void )
{
    if( tx_cw.is_running )
    {
        tx_cw.is_running = false;
        smtc_rac_abort_radio_submit( tx_cw.radio_access_id );
        TX_CW_PRINT( "Stopping continuous transmission. Total packets sent: %" PRIu32 "\n", tx_cw.packet_count );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void tx_cw_send_packet( void )
{
    if( !tx_cw.is_running )
    {
        return;  // Stop transmission if not running
    }

    tx_cw.transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + TX_DELAY;
    if( PACKET_TYPE == SMTC_RAC_MODULATION_LORA || PACKET_TYPE == SMTC_RAC_MODULATION_FSK )
    {
        SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( tx_cw.radio_access_id ) );
    }
}

static void pre_tx_callback( void )
{
    TX_CW_PRINT( "usp/rac: transmission #%" PRIu32 " starting\n", tx_cw.packet_count );
    set_led( SMTC_PF_LED_TX, true );
}

static void post_tx_callback( rp_status_t status )
{
    set_led( SMTC_PF_LED_TX, false );

    // Schedule next transmission if still running
    if( tx_cw.is_running )
    {
        tx_cw_send_packet( );
    }
}
