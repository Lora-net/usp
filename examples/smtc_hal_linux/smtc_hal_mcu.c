/**
 * @file      smtc_hal_mcu.c
 *
 * @brief     MCU Hardware Abstraction Layer implementation for Linux
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

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_rtc.h"
#include "smtc_hal_lp_timer.h"
#include "smtc_hal_rng.h"
#include "smtc_hal_watchdog.h"
#include "modem_pinout.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>  // for timespec, nanosleep

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

static bool hal_mcu_initialized = false;

jmp_buf       hal_reset_jmp_buf;
volatile bool hal_reset_jmp_set = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_mcu_critical_section_begin( uint32_t* mask )
{
    // No-op on Linux - placeholder for embedded implementation
    ( void ) mask;
}

void hal_mcu_critical_section_end( uint32_t* mask )
{
    // No-op on Linux - placeholder for embedded implementation
    ( void ) mask;
}

void hal_mcu_disable_irq( void )
{
    // No-op on Linux - placeholder for embedded implementation
}

void hal_mcu_enable_irq( void )
{
    // No-op on Linux - placeholder for embedded implementation
}

void hal_mcu_init( void )
{
    // On soft reset the HAL resources (GPIO thread, SPI fd, timers) are still
    // valid, so skip re-initialization.  Only the modem/RAC stack needs reinit.
    if( hal_mcu_initialized )
    {
        printf( "MCU: Linux HAL already initialized, skipping\n" );
        return;
    }

    printf( "MCU: Initializing Linux HAL\n" );

    // Initialize RTC
    hal_rtc_init( );

    // Initialize RNG
    hal_rng_init( );

    // Initialize GPIO subsystem
    hal_gpio_init( );

    // Configure radio GPIOs
    hal_gpio_init_out( RADIO_NRST, 1 );  // RESET pin as output, initially high

    hal_gpio_init_in( RADIO_BUSY_PIN, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_OFF,
                      NULL );  // BUSY pin as input, no IRQ

    hal_gpio_init_in( RADIO_DIO_MAIN, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_RISING,
                      NULL );  // DIOX pin as input with rising edge detection
                               // Note: IRQ callback will be attached later by
                               // smtc_modem_hal_irq_config_radio_irq()

    // Initialize Low Power Timer
    hal_lp_timer_init( HAL_LP_TIMER_ID_1 );

    // Initialize SPI
    hal_spi_init( );

    hal_mcu_initialized = true;
    printf( "MCU: Linux HAL initialized\n" );
}

void hal_mcu_reset( void )
{
    if( hal_reset_jmp_set )
    {
        printf( "MCU: Reset requested - performing soft reset via longjmp\n" );
        hal_reset_jmp_set = false;
        longjmp( hal_reset_jmp_buf, 1 );
    }
    else
    {
        printf( "MCU: Reset requested - exiting (no soft reset point set)\n" );
        exit( 1 );
    }
}

void hal_mcu_wait_us( const int32_t microseconds )
{
    if( microseconds > 0 )
    {
        struct timespec ts;
        ts.tv_sec  = microseconds / 1000000;
        ts.tv_nsec = ( microseconds % 1000000 ) * 1000L;
        nanosleep( &ts, NULL );
    }
}

void hal_mcu_set_sleep_for_ms( const int32_t milliseconds )
{
    if( milliseconds <= 0 )
    {
        return;
    }

    // MCU __WFI()-equivalent: sleep for the requested duration, but return early
    // as soon as a signal (radio DIO via SIGUSR2, lp_timer via SIGRTMIN, button
    // via SIGUSR1) arrives so the main loop can service the IRQ flag. SA_RESTART
    // is intentionally not set on those handlers (see smtc_hal_gpio.c /
    // smtc_hal_lp_timer.c) precisely so nanosleep() returns EINTR here.
    struct timespec ts = {
        .tv_sec  = milliseconds / 1000,
        .tv_nsec = ( milliseconds % 1000 ) * 1000000L,
    };
    nanosleep( &ts, NULL );
}

void hal_mcu_disable_low_power_wait( void )
{
    // No-op on Linux
}

void hal_mcu_enable_low_power_wait( void )
{
    // No-op on Linux
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
