/**
 * @file      smtc_hal_lp_timer.c
 *
 * @brief     Low Power Timer Hardware Abstraction Layer implementation for Linux
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <time.h>

#include "smtc_hal_lp_timer.h"
#include "smtc_hal_rtc.h"  // For time acceleration

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define MAX_TIMERS 2

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct
{
    timer_t            timer;
    hal_lp_timer_irq_t irq;
    bool               is_initialized;
    bool               is_active;
    bool               irq_enabled;
    bool irq_pending;  // latched on expiry while masked, replayed on enable (NVIC-like)
} lp_timer_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lp_timer_context_t timer_context[MAX_TIMERS] = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void timer_signal_handler( int sig, siginfo_t* si, void* uc );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init( const hal_lp_timer_id_t timer_id )
{
    if( timer_id >= MAX_TIMERS )
    {
        return;
    }

    // Set up signal handler for this timer
    struct sigaction sa;
    memset( &sa, 0, sizeof( sa ) );
    sa.sa_flags     = SA_SIGINFO;
    sa.sa_sigaction = timer_signal_handler;
    sigemptyset( &sa.sa_mask );

    int signal_num = SIGRTMIN + timer_id;
    if( sigaction( signal_num, &sa, NULL ) == -1 )
    {
        perror( "sigaction" );
        return;
    }

    // Create POSIX timer
    struct sigevent sev;
    memset( &sev, 0, sizeof( sev ) );
    sev.sigev_notify          = SIGEV_SIGNAL;
    sev.sigev_signo           = signal_num;
    sev.sigev_value.sival_int = timer_id;  // Pass timer ID to signal handler

    if( timer_create( CLOCK_MONOTONIC, &sev, &timer_context[timer_id].timer ) == -1 )
    {
        perror( "timer_create" );
        return;
    }

    timer_context[timer_id].is_initialized = true;
    timer_context[timer_id].is_active      = false;
    timer_context[timer_id].irq_enabled    = true;
    timer_context[timer_id].irq_pending    = false;
    timer_context[timer_id].irq.callback   = NULL;
    timer_context[timer_id].irq.context    = NULL;
}

void hal_lp_timer_start( const hal_lp_timer_id_t timer_id, const uint32_t milliseconds, const hal_lp_timer_irq_t* irq )
{
    if( timer_id >= MAX_TIMERS || irq == NULL || !timer_context[timer_id].is_initialized )
    {
        return;
    }

    // Store the callback and context
    timer_context[timer_id].irq = *irq;

    // Apply time acceleration (divide delay by acceleration factor)
    uint32_t accel_factor   = hal_rtc_get_time_acceleration( );
    uint32_t accelerated_ms = milliseconds / accel_factor;

    // Ensure minimum 1ms timer to avoid immediate expiry issues
    if( accelerated_ms == 0 && milliseconds > 0 )
    {
        accelerated_ms = 1;
    }

    // Configure timer as one-shot (non-repeating)
    struct itimerspec its;
    memset( &its, 0, sizeof( its ) );

    // Convert accelerated milliseconds to seconds and nanoseconds
    its.it_value.tv_sec  = accelerated_ms / 1000;
    its.it_value.tv_nsec = ( accelerated_ms % 1000 ) * 1000000;

    // Interval = 0 means one-shot (non-repeating)
    its.it_interval.tv_sec  = 0;
    its.it_interval.tv_nsec = 0;

    // Arm the timer
    if( timer_settime( timer_context[timer_id].timer, 0, &its, NULL ) == -1 )
    {
        perror( "timer_settime" );
        return;
    }

    timer_context[timer_id].is_active = true;
}

void hal_lp_timer_stop( const hal_lp_timer_id_t timer_id )
{
    if( timer_id >= MAX_TIMERS || !timer_context[timer_id].is_initialized )
    {
        return;
    }

    // Disarm the timer by setting it to zero
    struct itimerspec its;
    memset( &its, 0, sizeof( its ) );

    if( timer_settime( timer_context[timer_id].timer, 0, &its, NULL ) == -1 )
    {
        perror( "timer_settime (stop)" );
        return;
    }

    timer_context[timer_id].is_active = false;
}

void hal_lp_timer_irq_enable( const hal_lp_timer_id_t timer_id )
{
    if( timer_id >= MAX_TIMERS )
    {
        return;
    }

    timer_context[timer_id].irq_enabled = true;

    /* Replay an expiry that fired while the IRQ was masked (NVIC-pending-bit
     * semantics). */
    if( timer_context[timer_id].irq_pending && timer_context[timer_id].irq.callback != NULL )
    {
        timer_context[timer_id].irq_pending = false;
        timer_context[timer_id].irq.callback( timer_context[timer_id].irq.context );
    }
}

void hal_lp_timer_irq_disable( const hal_lp_timer_id_t timer_id )
{
    if( timer_id >= MAX_TIMERS )
    {
        return;
    }

    timer_context[timer_id].irq_enabled = false;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void timer_signal_handler( int sig, siginfo_t* si, void* uc )
{
    ( void ) sig;
    ( void ) uc;

    // Get the timer ID from the signal info
    int timer_id = si->si_value.sival_int;

    if( timer_id >= MAX_TIMERS )
    {
        return;
    }

    // Mark timer as inactive
    timer_context[timer_id].is_active = false;

    // Call the callback if IRQ is enabled and callback is set
    if( timer_context[timer_id].irq_enabled && timer_context[timer_id].irq.callback != NULL )
    {
        timer_context[timer_id].irq.callback( timer_context[timer_id].irq.context );
    }
    else
    {
        /* IRQ is masked: latch the expiry so hal_lp_timer_irq_enable() can
         * replay it. */
        timer_context[timer_id].irq_pending = true;
    }
}

/* --- EOF ------------------------------------------------------------------ */
