/**
 * @file      smtc_hal_lp_timer.c
 *
 * @brief     Low Power Timer HAL for RA0E2
 *
 * Architecture:
 * - TML1 (16-bit @ 2048 Hz) provides LP timer callbacks
 * - TML1 runs from SOSC (32.768 kHz) with /16 divider = 2048 Hz
 * - ~0.5ms resolution, max delay ~32 seconds (clamped if longer)
 * - Uses compare match interrupt for callback
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

#include <stdint.h>
#include <stdbool.h>
#include "smtc_hal_lp_timer.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_rtc.h"
#include "hal_data.h"
#include "bsp_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

/* TML clock: SOSC 32.768 kHz / 16 = 2048 Hz (matches STM32 LPTIM resolution) */
#define TML_CLOCK_HZ 2048U

/* FDIV value for /16 divider */
#define TML_FDIV_DIV16 4U

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/* LP timer callback storage */
static hal_lp_timer_irq_t lp_timer_irq = { .context = NULL, .callback = NULL };

/* Track if LP timer is initialized */
static bool lp_timer_initialized = false;

/* Track if LP timer is currently running (for sleep wakeup coordination) */
static volatile bool lp_timer_running = false;

/* Flag set by callback when it updates systick - cleared by hal_lp_timer_did_update_systick() */
static volatile bool lp_timer_updated_systick = false;

/* Store the LP timer start time and delay for systick resync when callback fires */
static volatile uint32_t lp_timer_start_time_ms = 0;
static volatile uint32_t lp_timer_delay_ms      = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS --------------------------------------------------------
 */

void hal_lp_timer_init( hal_lp_timer_id_t id )
{
    fsp_err_t err;

    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }

    if( lp_timer_initialized )
    {
        return;
    }

    /* Set clock divider to /16 BEFORE opening TML */
    /* In 16-bit mode, channels 2+3 are used for TML1, so set FDIV2 and FDIV3 */
    R_TML->ITLFDIV01 = ( TML_FDIV_DIV16 << 4 ) | TML_FDIV_DIV16; /* FDIV3=4, FDIV2=4 */

    err = R_TML_Open( &g_timer_tml1_ctrl, &g_timer_tml1_cfg );
    if( FSP_SUCCESS != err )
    {
        mcu_panic( );
    }

    /* Configure NVIC for TML interrupt */
    NVIC_SetPriority( TML0_ITL_OR_IRQn, 2 );
    NVIC_EnableIRQ( TML0_ITL_OR_IRQn );

    lp_timer_irq         = ( hal_lp_timer_irq_t ){ .context = NULL, .callback = NULL };
    lp_timer_initialized = true;
}

void hal_lp_timer_start( hal_lp_timer_id_t id, const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    fsp_err_t err;
    uint32_t  delay_ticks;

    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }

    /* Convert milliseconds to ticks: ticks = ms * 1024 / 1000
     * Use ceiling (round UP) to ensure we don't open RX window too early.
     * The LoRaWAN stack accounts for timing margins, so being slightly late
     * (by up to ~1ms) is acceptable. Being too early causes the RX timeout
     * to expire before the downlink arrives.
     */
    delay_ticks = ( ( milliseconds * TML_CLOCK_HZ ) + 999U ) / 1000U;

    /* Clamp to 16-bit max if exceeds ~32 seconds */
    if( delay_ticks > 0xFFFFU )
    {
        delay_ticks = 0xFFFFU;
    }

    /* Ensure at least 1 tick */
    if( delay_ticks == 0 )
    {
        delay_ticks = 1;
    }

    /* Save callback, start time, and delay for resync in callback */
    lp_timer_irq           = *tmr_irq;
    lp_timer_start_time_ms = hal_rtc_get_time_ms( );
    lp_timer_delay_ms      = milliseconds;

    /* Stop timer if running */
    R_TML_Stop( &g_timer_tml1_ctrl );

    /* Set the period (compare value) */
    /* In 16-bit mode for channel 2, we write to ITLCMP01 */
    R_TML->ITLCMP01 = ( uint16_t ) ( delay_ticks - 1 );

    /* Clear any pending interrupt flag for channel 2 */
    R_TML->ITLS0 &= ~( R_TML_ITLS0_ITF02_Msk );

    /* Enable compare match interrupt */
    err = R_TML_Enable( &g_timer_tml1_ctrl );
    if( FSP_SUCCESS != err )
    {
        /* Non-fatal, continue anyway */
    }

    /* Start the timer */
    err = R_TML_Start( &g_timer_tml1_ctrl );
    if( FSP_SUCCESS != err )
    {
        mcu_panic( );
    }

    lp_timer_running = true;
}

void hal_lp_timer_stop( hal_lp_timer_id_t id )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }

    R_TML_Stop( &g_timer_tml1_ctrl );

    /* Clear callback */
    lp_timer_irq.callback = NULL;
    lp_timer_irq.context  = NULL;
    lp_timer_running      = false;
}

void hal_lp_timer_irq_enable( hal_lp_timer_id_t id )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }

    NVIC_EnableIRQ( TML0_ITL_OR_IRQn );
}

void hal_lp_timer_irq_disable( hal_lp_timer_id_t id )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }

    NVIC_DisableIRQ( TML0_ITL_OR_IRQn );
}

bool hal_lp_timer_is_running( hal_lp_timer_id_t id )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        return false;
    }

    return lp_timer_running;
}

bool hal_lp_timer_did_update_systick( hal_lp_timer_id_t id )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        return false;
    }

    /* Check and clear the flag atomically.
     * Returns true if LP timer callback updated systick since last check.
     * Clears the flag so next call returns false unless callback fires again.
     */
    bool updated             = lp_timer_updated_systick;
    lp_timer_updated_systick = false;
    return updated;
}

/*
 * -----------------------------------------------------------------------------
 * --- TML1 CALLBACK (called from FSP ISR) -------------------------------------
 */

/**
 * @brief TML1 callback - called when compare match occurs
 *
 * This is the callback registered with FSP in hal_data.c
 */
void tml1_lp_timer_callback( timer_callback_args_t* p_args )
{
    ( void ) p_args;

    /* Stop the timer after callback */
    R_TML_Stop( &g_timer_tml1_ctrl );
    lp_timer_running = false;

    /* Set systick to the correct absolute time.
     *
     * When LP timer fires:
     * - LP timer was started at lp_timer_start_time_ms
     * - It ran for lp_timer_delay_ms
     * - So current time = lp_timer_start_time_ms + lp_timer_delay_ms
     *
     * This is critical for LoRaWAN RX timing - the radio planner needs accurate
     * timestamps to decide if tasks are in the past or future.
     *
     * Note: We set the absolute time directly, not via hal_rtc_resync(), because
     * the LP timer start time may differ from sleep entry time.
     */
    hal_rtc_set_time_ms( lp_timer_start_time_ms + lp_timer_delay_ms );

    /* Mark that we updated systick - prevents calendar sync from overwriting */
    lp_timer_updated_systick = true;

    /* Call user callback if registered */
    if( lp_timer_irq.callback != NULL )
    {
        lp_timer_irq.callback( lp_timer_irq.context );
    }
}

/* --- EOF ------------------------------------------------------------------ */
