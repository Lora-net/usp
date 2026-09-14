/**
 * @file      immediate_radio_access.c
 *
 * @brief     Immediate radio access example for LR20xx chip
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

#include "immediate_radio_access.h"

#include <stddef.h>
#include "smtc_rac_api.h"
#include "smtc_hal_led.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "IMMEDIATE-RADIO"
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
#define IMMEDIATE_RADIO_PRINT( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static uint8_t payload_buffer[LORA_PAYLOAD_LENGTH] = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

uint8_t trig_tx_cpt = 0;  // count the number of times the radio is triggered to transmit

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Schedule immediate radio actions in RAC
 */
static void schedule_immediate_radio_access( void );

/**
 * @brief Called after the radio transaction.
 *
 * This callback is called when the radio transaction is finished
 *
 * @param [in] status Result of radio operation
 */
static void irq_callback( void );

/**
 * @brief Set the radio in sleep mode
 *
 * Without this call, the radio do not go automatically in sleep mode
 *
 * @param [in] status Result of radio operation
 */
static ral_status_t set_sleep( void );

/* -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void immediate_radio_access_init( void )
{
    // no specific init require for feature immediate radio access

    IMMEDIATE_RADIO_PRINT( "Immediate radio access initialized - press button to start/stop transmission\n" );
}

void immediate_radio_access_on_button_press( void )
{
    if( trig_tx_cpt > 0 )
    {
        IMMEDIATE_RADIO_PRINT( "Radio is busy\n" );
    }
    else
    {
        trig_tx_cpt                   = TRANSMIT_MULTIPLE_TIME_THE_SAME_PACKET;
        smtc_rac_return_code_t status = smtc_rac_immediate_radio_access( RAC_VERY_HIGH_PRIORITY, irq_callback );
        if( status != SMTC_RAC_SUCCESS )
        {
            IMMEDIATE_RADIO_PRINT( "Fail to suspend radio access with following error code: %x\n", status );
        }
        else
        {
            schedule_immediate_radio_access( );
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static ral_status_t set_sleep( void )
{
    void*                           radio_driver_context = smtc_rac_get_radio_driver_context( );
    const lr20xx_system_sleep_cfg_t radio_sleep_cfg      = {
             .is_ram_retention_enabled = true,
             .is_clk_32k_enabled       = false,
    };

    return ( ral_status_t ) lr20xx_system_set_sleep_mode( radio_driver_context, &radio_sleep_cfg, 0 );
}

static void irq_callback( void )
{
    lr20xx_system_irq_mask_t radio_irq            = LR20XX_SYSTEM_IRQ_NONE;
    void*                    radio_driver_context = smtc_rac_get_radio_driver_context( );
    lr20xx_system_get_and_clear_irq_status( radio_driver_context, &radio_irq );

    if( ( radio_irq & LR20XX_SYSTEM_IRQ_TX_DONE ) == LR20XX_SYSTEM_IRQ_TX_DONE )
    {
        IMMEDIATE_RADIO_PRINT( "usp/rac: TX sent\n" );
        trig_tx_cpt--;
    }
    else
    {
        IMMEDIATE_RADIO_PRINT( "usp/rac: transaction  (Error)\n" );
        ( void ) set_sleep( );
        smtc_rac_release_immediate_radio_access( );
        hal_led_set( HAL_LED_TX, false );
        trig_tx_cpt = 0;
        return;
    }
    if( trig_tx_cpt > 0 )
    {
        schedule_immediate_radio_access( );
    }
    else
    {
        ( void ) set_sleep( );
        smtc_rac_release_immediate_radio_access( );
        IMMEDIATE_RADIO_PRINT( "usp/rac:end of execution of immediate radio access \n" );
        hal_led_set( HAL_LED_TX, false );
        trig_tx_cpt = 0;
    }
}

static void schedule_immediate_radio_access( void )
{
    void* radio_driver_context = smtc_rac_get_radio_driver_context( );
    if( trig_tx_cpt ==
        TRANSMIT_MULTIPLE_TIME_THE_SAME_PACKET )  // set config only one time and after just trigger the radio
    {
        IMMEDIATE_RADIO_PRINT( "usp/rac: provide immediate radio access for first transmission \n" );
        lr20xx_radio_common_set_pkt_type( radio_driver_context, LR20XX_RADIO_COMMON_PKT_TYPE_LORA );
        lr20xx_radio_common_set_rf_freq( radio_driver_context, RF_FREQ_IN_HZ );

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

        // 20xx_radio_fifo_write_tx( radio_driver_context, payload_buffer, LORA_PAYLOAD_LENGTH );
        hal_led_set( HAL_LED_TX, true );
    }
    lr20xx_radio_fifo_write_tx( radio_driver_context, payload_buffer, LORA_PAYLOAD_LENGTH );
    lr20xx_radio_common_set_tx( radio_driver_context, 0 );
}
