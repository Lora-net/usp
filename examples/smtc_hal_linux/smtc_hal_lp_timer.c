/**
 * @file      smtc_hal_lp_timer.c
 *
 * @brief     Low Power Timer Hardware Abstraction Layer implementation for Linux
 *
 * Timer expiry is delivered by a dedicated, persistent per-timer thread blocked on
 * a timerfd. Running in a normal thread context lets the callback take the RAC API
 * lock (smtc_modem_hal_protect_api_call) and serialize with the RAC engine and the
 * other HAL threads. After running the callback the thread calls
 * hal_mcu_wake_main_thread() so a main thread sleeping in nanosleep() re-checks
 * the IRQ flag immediately.
 *
 * Build with -DLP_TIMER_LATENCY_LOG to print per-expiry arm->fire latency/jitter.
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
#include <string.h>
#include <time.h>
#include <unistd.h>  // read, close
#include <pthread.h>
#include <signal.h>  // pthread_kill, SIGUSR2, pthread_sigmask
#include <sys/timerfd.h>

#include "smtc_hal_lp_timer.h"
#include "smtc_hal_rtc.h"  // For time acceleration
#include "smtc_hal_mcu.h"  // hal_mcu_wake_main_thread()

/* Declares smtc_modem_hal_protect_api_call() / _unprotect_api_call(): the RAC
 * critical-section hook (weak no-op default; an application may override it with a
 * real lock). The timer callback drives the radio planner, so it must run inside
 * this section to serialize with the RAC engine. */
#include "smtc_core_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define MAX_TIMERS 2

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct
{
    int                fd;      // timerfd descriptor
    pthread_t          thread;  // per-timer expiry-delivery thread
    bool               thread_running;
    hal_lp_timer_irq_t irq;
    bool               is_initialized;
    bool               is_active;
    bool               irq_enabled;
    bool               irq_pending;  // latched on expiry while masked, replayed on enable (NVIC-like)
#if defined( LP_TIMER_LATENCY_LOG )
    struct timespec arm_time;      // CLOCK_MONOTONIC time when the timer was armed
    uint32_t        requested_ms;  // armed duration after time acceleration
#endif
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

static void* timer_thread_func( void* arg );

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

    // One timerfd consumed by one dedicated, long-lived thread (created here). This
    // keeps timer delivery out of async-signal context and avoids spawning a thread
    // per expiry, giving low and stable latency for the radio planner.
    int fd = timerfd_create( CLOCK_MONOTONIC, 0 );
    if( fd == -1 )
    {
        perror( "timerfd_create" );
        return;
    }

    timer_context[timer_id].fd             = fd;
    timer_context[timer_id].is_initialized = true;
    timer_context[timer_id].is_active      = false;
    timer_context[timer_id].irq_enabled    = true;
    timer_context[timer_id].irq_pending    = false;
    timer_context[timer_id].irq.callback   = NULL;
    timer_context[timer_id].irq.context    = NULL;
    timer_context[timer_id].thread_running = true;

    if( pthread_create( &timer_context[timer_id].thread, NULL, timer_thread_func, ( void* ) ( intptr_t ) timer_id ) !=
        0 )
    {
        perror( "pthread_create (lp_timer)" );
        timer_context[timer_id].thread_running = false;
        timer_context[timer_id].is_initialized = false;
        close( fd );
    }
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
    its.it_value.tv_sec  = accelerated_ms / 1000;
    its.it_value.tv_nsec = ( accelerated_ms % 1000 ) * 1000000;
    // it_interval stays 0 => one-shot

#if defined( LP_TIMER_LATENCY_LOG )
    clock_gettime( CLOCK_MONOTONIC, &timer_context[timer_id].arm_time );
    timer_context[timer_id].requested_ms = accelerated_ms;
#endif

    // Arm the timer
    if( timerfd_settime( timer_context[timer_id].fd, 0, &its, NULL ) == -1 )
    {
        perror( "timerfd_settime" );
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

    if( timerfd_settime( timer_context[timer_id].fd, 0, &its, NULL ) == -1 )
    {
        perror( "timerfd_settime (stop)" );
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

    // Serialize with timer_thread_func, which mutates these fields and runs the
    // planner callback under the same lock.
    smtc_modem_hal_protect_api_call( );
    timer_context[timer_id].irq_enabled = true;

    // Replay an expiry that fired while the IRQ was masked (NVIC-pending-bit
    // semantics) so a timer event is never lost across a mask/unmask window.
    if( timer_context[timer_id].irq_pending && timer_context[timer_id].irq.callback != NULL )
    {
        timer_context[timer_id].irq_pending = false;
        timer_context[timer_id].irq.callback( timer_context[timer_id].irq.context );
    }
    smtc_modem_hal_unprotect_api_call( );
}

void hal_lp_timer_irq_disable( const hal_lp_timer_id_t timer_id )
{
    if( timer_id >= MAX_TIMERS )
    {
        return;
    }

    smtc_modem_hal_protect_api_call( );
    timer_context[timer_id].irq_enabled = false;
    smtc_modem_hal_unprotect_api_call( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void* timer_thread_func( void* arg )
{
    int                 timer_id = ( int ) ( intptr_t ) arg;
    lp_timer_context_t* ctx      = &timer_context[timer_id];

    // Block SIGUSR2 in this thread. hal_mcu_wake_main_thread() sends it
    // thread-directed via pthread_kill(main_tid, SIGUSR2), so it always reaches
    // the main thread on its own. This block is defensive: it guarantees a
    // *process-directed* SIGUSR2 (one sent to the whole process) can never be
    // handled by this worker thread nor interrupt its read() with EINTR.
    sigset_t block_mask;
    sigemptyset( &block_mask );
    sigaddset( &block_mask, SIGUSR2 );
    pthread_sigmask( SIG_BLOCK, &block_mask, NULL );

    while( ctx->thread_running )
    {
        uint64_t expirations;
        ssize_t  n = read( ctx->fd, &expirations, sizeof( expirations ) );
        if( n != ( ssize_t ) sizeof( expirations ) )
        {
            // EINTR or a short read (e.g. while the timer is disarmed): wait again.
            continue;
        }

#if defined( LP_TIMER_LATENCY_LOG )
        {
            struct timespec now;
            clock_gettime( CLOCK_MONOTONIC, &now );
            long elapsed_us =
                ( now.tv_sec - ctx->arm_time.tv_sec ) * 1000000L + ( now.tv_nsec - ctx->arm_time.tv_nsec ) / 1000L;
            long jitter_us = elapsed_us - ( long ) ctx->requested_ms * 1000L;
            printf( "LP_TIMER[%d]: req=%u ms actual=%ld.%03ld ms (jitter %+ld us)\n", timer_id, ctx->requested_ms,
                    elapsed_us / 1000, elapsed_us % 1000, jitter_us );
        }
#endif

        // Timer expired. Take the RAC lock first: hal_lp_timer_start()/stop() mutate
        // this context from the planner while holding the same lock, and the callback
        // itself runs the radio planner. So every access here (is_active / irq fields
        // / callback) must be under it.
        smtc_modem_hal_protect_api_call( );
        ctx->is_active = false;
        if( ctx->irq.callback != NULL )
        {
            if( ctx->irq_enabled )
            {
                ctx->irq.callback( ctx->irq.context );
            }
            else
            {
                // IRQ masked: latch the expiry so hal_lp_timer_irq_enable() can
                // replay it on unmask.
                ctx->irq_pending = true;
            }
        }
        smtc_modem_hal_unprotect_api_call( );

        // Wake the main thread: it may be sleeping in nanosleep() (hal_mcu_set_sleep
        // _for_ms). SIGUSR2 (no SA_RESTART) makes that sleep return EINTR so the main
        // loop re-runs the engine immediately.
        hal_mcu_wake_main_thread( );
    }

    return NULL;
}

/* --- EOF ------------------------------------------------------------------ */
