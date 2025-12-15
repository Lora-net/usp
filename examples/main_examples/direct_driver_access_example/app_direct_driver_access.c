/**
 * @file      app_direct_driver_access.c
 *
 * @brief     Direct radio driver access example for LR20xx chip
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

#include "radio_planner_types.h"
#ifndef LR20XX
#error \
    "this example is not supported for this radio, you have to implement it" \
    "yourself using specific radio driver,such as LR11XX driver and so on \n" );
#endif  // LR20XX

#include "app_direct_driver_access.h"

#include <stddef.h>
#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "DIRECT-DRIVER"
#include "smtc_rac_log.h"
#include "smtc_hal_dbg_trace.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_system.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_lora.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// Backward compatibility alias for existing code
#define DIRECT_DRIVER_PRINT( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static uint8_t payload_buffer[LORA_PAYLOAD_LENGTH] = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct direct_driver_access_s
{
    uint8_t                     radio_access_id;  // store result of `smtc_rac_open_radio`
    bool                        is_running;       // indicates if transmission is active
    smtc_rac_scheduler_config_t scheduler_config;
} direct_driver_access_t;
smtc_rac_scheduler_config_t scheduler_config;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static direct_driver_access_t direct_driver_access = {
    .radio_access_id  = 0,  // set in `direct_driver_access_init`
    .scheduler_config = { 0 },
    .is_running       = false,
};
static int8_t cpt_transmit_multiple_time_the_same_packet = TRANSMIT_MULTIPLE_TIME_THE_SAME_PACKET;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Schedule direct driver actions in RAC
 */
static void schedule_direct_driver_access( void );

/**
 * @brief Called after the radio transaction.
 *
 * This callback is called when the radio transaction is finished
 *
 * @param [in] status Result of radio operation
 */
static void post_tx_callback( rp_status_t status );

/**
 * @brief Callback to be called before radio transaction
 *
 * This callback is to be called when the radio is available, when direct radio access through RAL or driver calls are
 * possible
 * */
static void pre_tx_callback( void );

/* -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void direct_driver_access_init( void )
{
    // Register direct radio access from RAC
    direct_driver_access.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_VERY_HIGH_PRIORITY ) );

    DIRECT_DRIVER_PRINT( "Direct driver access initialized - press button to start/stop transmission\n" );
}

void direct_driver_access_on_button_press( void )
{
    if( direct_driver_access.is_running )
    {
        DIRECT_DRIVER_PRINT( "Radio is busy\n" );
    }
    else
    {
        schedule_direct_driver_access( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void pre_tx_callback( void )
{
    DIRECT_DRIVER_PRINT( "usp/rac: provide direct radio access \n" );
    void* radio_driver_context = smtc_rac_get_radio_driver_context( );
    lr20xx_radio_common_set_pkt_type( radio_driver_context, LR20XX_RADIO_COMMON_PKT_TYPE_LORA );
    lr20xx_radio_common_set_rf_freq( radio_driver_context, FREQ_IN_HZ );

    lr20xx_radio_common_set_rx_path( radio_driver_context, LR20XX_RADIO_COMMON_RX_PATH_LF,
                                     LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE );

    const lr20xx_radio_common_pa_cfg_t pa_cfg = {
        .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
        .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
        .pa_lf_duty_cycle = 5,
        .pa_lf_slices     = 4,
        .pa_hf_duty_cycle = 16,
    };

    lr20xx_radio_common_set_pa_cfg( radio_driver_context, &pa_cfg );
    lr20xx_radio_common_set_tx_params( radio_driver_context, 31, LR20XX_RADIO_COMMON_RAMP_48_US );

    lr20xx_radio_lora_mod_params_t radio_mod_params;
    radio_mod_params.sf  = LORA_SF;
    radio_mod_params.bw  = LORA_BW;
    radio_mod_params.cr  = LORA_CR;
    radio_mod_params.ppm = LORA_PPM;
    lr20xx_radio_lora_set_modulation_params( radio_driver_context, &radio_mod_params );

    lr20xx_radio_lora_pkt_params_t radio_pkt_params = { 0 };
    radio_pkt_params.preamble_len_in_symb           = LORA_PREAMBLE_LENGTH;
    radio_pkt_params.pld_len_in_bytes               = LORA_PAYLOAD_LENGTH;
    radio_pkt_params.pkt_mode                       = LORA_PKT_LEN_MODE;
    radio_pkt_params.crc                            = LORA_CRC;
    radio_pkt_params.iq                             = LORA_IQ;
    lr20xx_radio_lora_set_packet_params( radio_driver_context, &radio_pkt_params );
    lr20xx_radio_lora_set_syncword( radio_driver_context, LORA_SYNCWORD );
#if defined( LEGACY_EVK_LR20XX )
    lr20xx_system_set_dio_irq_cfg( radio_driver_context, LR20XX_SYSTEM_DIO_9,
                                   LR20XX_SYSTEM_IRQ_TX_DONE | LR20XX_SYSTEM_IRQ_TIMEOUT );
#else
    lr20xx_system_set_dio_irq_cfg( radio_driver_context, LR20XX_SYSTEM_DIO_8,
                                   LR20XX_SYSTEM_IRQ_TX_DONE | LR20XX_SYSTEM_IRQ_TIMEOUT );
#endif

    lr20xx_radio_fifo_write_tx( radio_driver_context, payload_buffer, LORA_PAYLOAD_LENGTH );
    lr20xx_radio_common_set_tx( radio_driver_context, 0 );
    cpt_transmit_multiple_time_the_same_packet = TRANSMIT_MULTIPLE_TIME_THE_SAME_PACKET;
    set_led( SMTC_PF_LED_TX, true );
}

static void post_tx_callback( rp_status_t status )
{
    void* radio_driver_context = smtc_rac_get_radio_driver_context( );
    if( status == RP_STATUS_RADIO_LOCKED )
    {
        lr20xx_system_irq_mask_t radio_irq = LR20XX_SYSTEM_IRQ_NONE;

        cpt_transmit_multiple_time_the_same_packet--;

        if( cpt_transmit_multiple_time_the_same_packet > 0 )
        {
            lr20xx_radio_fifo_write_tx( radio_driver_context, payload_buffer, LORA_PAYLOAD_LENGTH );
            lr20xx_radio_common_set_tx( radio_driver_context, 0 );
            lr20xx_system_get_and_clear_irq_status( radio_driver_context, &radio_irq );
        }
        else
        {
            smtc_rac_unlock_radio_access( direct_driver_access.radio_access_id );
            set_led( SMTC_PF_LED_TX, false );
            direct_driver_access.is_running = false;
        }
        DIRECT_DRIVER_PRINT( "usp/rac: transaction (transmission) has ended (success) %d\n",
                             cpt_transmit_multiple_time_the_same_packet );
    }
    else if( status == RP_STATUS_RADIO_UNLOCKED )
    {
        DIRECT_DRIVER_PRINT( "usp/rac: transaction (transmission) Unlocked \n" );
        set_led( SMTC_PF_LED_TX, false );
        direct_driver_access.is_running = false;
    }
    else if( status == RP_STATUS_TASK_ABORTED )
    {
        DIRECT_DRIVER_PRINT( "usp/rac: transaction (transmission) Aborted \n" );
        set_led( SMTC_PF_LED_TX, false );
        direct_driver_access.is_running = false;
    }
    else
    {
        DIRECT_DRIVER_PRINT( "usp/rac: transaction  (Error)\n" );
        direct_driver_access.is_running = false;
    }
}

void schedule_direct_driver_access( void )
{
    direct_driver_access.is_running = true;
    direct_driver_access.scheduler_config.start_time_ms =
        smtc_modem_hal_get_time_in_ms( ) + DIRECT_DRIVER_ACCESS_DELAY_MS;
    direct_driver_access.scheduler_config.duration_time_ms                = DIRECT_DRIVER_ACCESS_DURATION_MS;
    direct_driver_access.scheduler_config.callback_post_radio_transaction = post_tx_callback;
    direct_driver_access.scheduler_config.scheduling                      = SMTC_RAC_ASAP_TRANSACTION;
    direct_driver_access.scheduler_config.callback_pre_radio_transaction  = pre_tx_callback;
    smtc_rac_lock_radio_access( direct_driver_access.radio_access_id, direct_driver_access.scheduler_config );
}
