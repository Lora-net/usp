/**
 * @file      smtc_hal_rtc.c
 *
 * @brief     RTC Hardware Abstraction Layer implementation for Linux
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

#define _POSIX_C_SOURCE 199309L  // For clock_gettime() and CLOCK_MONOTONIC

#include "smtc_hal_rtc.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>  // For getenv() and atoi()

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

static struct timespec rtc_start_time;
static uint32_t        offset_in_s              = 0;
static uint32_t        time_acceleration_factor = 1;  // 1 = real-time, N = Nx faster

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rtc_init( void )
{
    // Get monotonic time at initialization
    clock_gettime( CLOCK_MONOTONIC, &rtc_start_time );

    // Check for time acceleration environment variable
    const char* accel_env = getenv( "LBM_TIME_ACCELERATION" );
    if( accel_env != NULL )
    {
        int accel_val = atoi( accel_env );
        if( accel_val >= 1 )
        {
            time_acceleration_factor = ( uint32_t ) accel_val;
        }
    }

    printf( "RTC: Initialized with monotonic clock\n" );
    if( time_acceleration_factor > 1 )
    {
        printf( "RTC: Time acceleration factor set to %ux (LBM_TIME_ACCELERATION)\n", time_acceleration_factor );
    }
}

uint32_t hal_rtc_get_time_s( void )
{
    struct timespec current_time;
    clock_gettime( CLOCK_MONOTONIC, &current_time );

    uint32_t elapsed_s = ( uint32_t ) ( current_time.tv_sec - rtc_start_time.tv_sec );

    // Apply time acceleration
    return ( elapsed_s * time_acceleration_factor ) + offset_in_s;
}

uint32_t hal_rtc_get_time_ms( void )
{
    struct timespec current_time;
    clock_gettime( CLOCK_MONOTONIC, &current_time );

    uint64_t start_ms   = ( uint64_t ) rtc_start_time.tv_sec * 1000 + rtc_start_time.tv_nsec / 1000000;
    uint64_t current_ms = ( uint64_t ) current_time.tv_sec * 1000 + current_time.tv_nsec / 1000000;

    uint64_t elapsed_ms = current_ms - start_ms;

    // Apply time acceleration
    return ( uint32_t ) ( ( elapsed_ms * time_acceleration_factor ) + ( offset_in_s * 1000 ) );
}

void hal_rtc_set_offset_to_test_wrapping( const uint32_t offset_to_test_wrapping )
{
    offset_in_s = offset_to_test_wrapping;
}

uint32_t hal_rtc_get_time_acceleration( void )
{
    return time_acceleration_factor;
}

void hal_rtc_set_time_acceleration( uint32_t factor )
{
    if( factor >= 1 )
    {
        time_acceleration_factor = factor;
        printf( "RTC: Time acceleration factor set to %ux\n", time_acceleration_factor );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
