/**
 * @file      smtc_hal_tau.c
 *
 * @brief     TAU timer driver for precise microsecond delays
 *
 *       Implementation Notes:
 *       ---------------------
 *       TAU channel 0 is configured in interval timer mode at 1MHz (CK00 = HOCO/32).
 *       This provides 1 microsecond resolution with 16-bit counter (max ~65ms).
 *
 *       Uses NVIC pending bit polling for ISR-safe operation. This allows the
 *       delay function to be called from within ISR context without deadlock.
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

#include "smtc_hal_tau.h"
#include "hal_data.h"
#include "vector_data.h"
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/* Flag to track if TAU is initialized */
static bool tau_initialized = false;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_tau_init( void )
{
    if( tau_initialized )
    {
        return;
    }

    /* Open TAU timer instance (configured by FSP) */
    fsp_err_t err = R_TAU_Open( g_timer_tau0.p_ctrl, g_timer_tau0.p_cfg );
    if( err != FSP_SUCCESS && err != FSP_ERR_ALREADY_OPEN )
    {
        /* Initialization failed */
        return;
    }

    tau_initialized = true;
}

void hal_tau_delay_us( uint32_t microseconds )
{
    if( microseconds == 0 || !tau_initialized )
    {
        return;
    }

    /* Clamp to maximum supported delay */
    if( microseconds > HAL_TAU_MAX_DELAY_US )
    {
        microseconds = HAL_TAU_MAX_DELAY_US;
    }

    /* Disable NVIC to prevent ISR from firing - we poll instead */
    NVIC_DisableIRQ( TAU0_TMI00_IRQn );

    /* Clear any pending interrupt from previous use */
    NVIC_ClearPendingIRQ( TAU0_TMI00_IRQn );

    /* Set period (1 tick = 1us with CK00 @ 1MHz) */
    fsp_err_t err = R_TAU_PeriodSet( g_timer_tau0.p_ctrl, microseconds );
    if( err != FSP_SUCCESS )
    {
        NVIC_EnableIRQ( TAU0_TMI00_IRQn );
        return;
    }

    /* Start the timer */
    err = R_TAU_Start( g_timer_tau0.p_ctrl );
    if( err != FSP_SUCCESS )
    {
        NVIC_EnableIRQ( TAU0_TMI00_IRQn );
        return;
    }

    /* Poll NVIC pending bit - set by hardware when timer interval completes */
    while( NVIC_GetPendingIRQ( TAU0_TMI00_IRQn ) == 0 )
    {
        /* Busy wait - ISR-safe polling */
    }

    /* Stop the timer */
    R_TAU_Stop( g_timer_tau0.p_ctrl );

    /* Clear pending interrupt */
    NVIC_ClearPendingIRQ( TAU0_TMI00_IRQn );

    /* Re-enable NVIC */
    NVIC_EnableIRQ( TAU0_TMI00_IRQn );
}

/*
 * -----------------------------------------------------------------------------
 * --- FSP CALLBACK (unused - polling mode) ------------------------------------
 */

void hal_tau_irq_handler( void )
{
    /* Not used - polling mode via NVIC pending bit */
}

/* --- EOF ------------------------------------------------------------------ */
