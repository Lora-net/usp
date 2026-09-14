/**
 * @file      smtc_hal_button.c
 *
 * @brief     Button Hardware Abstraction Layer implementation for Linux
 *
 * This implementation uses SIGUSR1 signal as a virtual button trigger.
 * Send the signal with: kill -SIGUSR1 <pid>
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
#include <signal.h>
#include <unistd.h>
#include <string.h>

#include "smtc_hal_button.h"
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * User callback and context
 */
static hal_button_callback_t button_callback = NULL;
static void*                 button_context  = NULL;

/*!
 * Button pressed flag (sig_atomic_t for signal handler safety)
 */
static volatile sig_atomic_t button_is_pressed = 0;

/*!
 * Process ID (cached at init for info display)
 */
static pid_t process_pid = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * Signal handler for SIGUSR1 (virtual button press)
 */
static void button_signal_handler( int signum );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_button_init( hal_button_callback_t callback, void* context )
{
    struct sigaction sa;

    // Store user callback and context
    button_callback = callback;
    button_context  = context;

    // Cache the process ID
    process_pid = getpid( );

    // Clear the sigaction structure
    memset( &sa, 0, sizeof( sa ) );

    // Set the handler function
    sa.sa_handler = button_signal_handler;

    // Clear signal mask (don't block any signals during handler execution)
    sigemptyset( &sa.sa_mask );

    // No special flags
    sa.sa_flags = 0;

    // Register the handler for SIGUSR1
    if( sigaction( SIGUSR1, &sa, NULL ) == -1 )
    {
        SMTC_HAL_TRACE_ERROR( "Button: Failed to register signal handler\n" );
        return;
    }

    // Print PID so user knows how to trigger the button
    SMTC_HAL_TRACE_INFO( "=================================================\n" );
    SMTC_HAL_TRACE_INFO( "Virtual button enabled (SIGUSR1)\n" );
    SMTC_HAL_TRACE_INFO( "Process PID: %d\n", process_pid );
    SMTC_HAL_TRACE_INFO( "Trigger button with: kill -SIGUSR1 %d\n", process_pid );
    SMTC_HAL_TRACE_INFO( "=================================================\n" );
}

bool hal_button_is_pressed( void )
{
    return ( button_is_pressed != 0 );
}

void hal_button_clear( void )
{
    button_is_pressed = 0;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void button_signal_handler( int signum )
{
    ( void ) signum;

    // Set the pressed flag (signal-safe operation)
    button_is_pressed = 1;

    // Note: We intentionally do NOT call the user callback from the signal handler
    // as it may not be signal-safe. The callback should be invoked from the main loop
    // after checking hal_button_is_pressed().
}

/* --- EOF ------------------------------------------------------------------ */
