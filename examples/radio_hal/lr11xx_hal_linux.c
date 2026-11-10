/**
 * @file      lr11xx_hal_linux.c
 *
 * @brief     Hardware Abstraction Layer for LR11XX on Linux
 *
 * This implementation provides SPI and GPIO functionality for the LR11XX radio
 * on Linux using the Linux SPI and GPIO subsystems.
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
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "lr11xx_hal.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_gpio.h"
#include "modem_pinout.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS
 * -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES
 * ------------------------------------------------------------
 */

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES
 * ------------------------------------------------------------
 */

static volatile radio_mode_t radio_mode = RADIO_AWAKE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS
 * --------------------------------------------------------
 */

static void wait_us( unsigned long delay_us )
{
    struct timespec dly;
    struct timespec rem;

    dly.tv_sec  = delay_us / 1000000;
    dly.tv_nsec = ( ( long ) delay_us % 1000000 ) * 1000;

    if( ( dly.tv_sec > 0 ) || ( ( dly.tv_sec == 0 ) && ( dly.tv_nsec > 100000 ) ) )
    {
        clock_nanosleep( CLOCK_MONOTONIC, 0, &dly, &rem );
    }
}

static void lr11xx_hal_wait_on_busy( void )
{
    while( hal_gpio_get_value( RADIO_BUSY_PIN ) == 1 )
    {
        wait_us( 10 );
    }
}

static void lr11xx_hal_check_device_ready( void )
{
    if( radio_mode != RADIO_SLEEP )
    {
        lr11xx_hal_wait_on_busy( );
    }
    else
    {
        // Keep NSS low for 100us to wake up the radio.
        hal_spi_nss_pulse( 100 );
        lr11xx_hal_wait_on_busy( );
        radio_mode = RADIO_AWAKE;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS --------------------------------------------------------
 */

lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    ( void ) context;

    lr11xx_hal_check_device_ready( );

    // Prepare single SPI transfer buffer: command + data
    uint8_t* tx_buf = malloc( command_length + data_length );

    if( tx_buf == NULL )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    memcpy( tx_buf, command, command_length );
    if( data != NULL && data_length > 0 )
    {
        memcpy( tx_buf + command_length, data, data_length );
    }

    int ret = hal_spi_transfer( tx_buf, NULL, command_length + data_length );

    free( tx_buf );

    if( ret < 0 )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    // Detect the SetSleep opcode (0x01 0x1B) and track radio state.
    // Add a small delay to let the chip fully enter sleep before any subsequent access.
    if( ( command_length >= 2 ) && ( command[0] == 0x01 ) && ( command[1] == 0x1B ) )
    {
        radio_mode = RADIO_SLEEP;
        wait_us( 500 );
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    ( void ) context;

    lr11xx_hal_check_device_ready( );

    // First SPI transaction - send command (CS auto-controlled by spidev)
    int ret = hal_spi_transfer( command, NULL, command_length );
    if( ret < 0 )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    // Wait for radio to be ready (BUSY low) after command
    lr11xx_hal_wait_on_busy( );

    if( data_length > 0 )
    {
        // Second SPI transaction - send 1 dummy byte + read data
        // Prepare buffer for second transaction: 1 dummy byte + data read
        uint8_t* tx_buf = malloc( 1 + data_length );
        uint8_t* rx_buf = malloc( 1 + data_length );

        if( tx_buf == NULL || rx_buf == NULL )
        {
            free( tx_buf );
            free( rx_buf );
            return LR11XX_HAL_STATUS_ERROR;
        }

        // Fill TX buffer: 1 dummy byte + zeros for data portion
        memset( tx_buf, 0x00, 1 + data_length );

        // Second SPI transaction (CS auto-controlled by spidev)
        ret = hal_spi_transfer( tx_buf, rx_buf, 1 + data_length );

        if( ret >= 0 )
        {
            // Copy received data (skip the 1 dummy response byte)
            memcpy( data, rx_buf + 1, data_length );
        }

        free( tx_buf );
        free( rx_buf );

        if( ret < 0 )
        {
            return LR11XX_HAL_STATUS_ERROR;
        }
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
    ( void ) context;

    lr11xx_hal_check_device_ready( );

    // Direct read with automatic CS control - send zeros and read response
    uint8_t* tx_buf = malloc( data_length );
    if( tx_buf == NULL )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    memset( tx_buf, 0x00, data_length );

    int ret = hal_spi_transfer( tx_buf, data, data_length );

    free( tx_buf );

    return ( ret >= 0 ) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
    ( void ) context;  // Suppress unused parameter warning

    // Pull reset pin low
    hal_gpio_set_value( RADIO_NRST, 0 );
    wait_us( 5000 );

    // Release reset pin (set high)
    hal_gpio_set_value( RADIO_NRST, 1 );
    wait_us( 5000 );

    // Wait 200ms until internal lr11xx fw is ready
    wait_us( 200000 );

    radio_mode = RADIO_AWAKE;
    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    ( void ) context;

    lr11xx_hal_check_device_ready( );
    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context )
{
    ( void ) context;  // Suppress unused parameter warning

    // Send 4 zero bytes to abort blocking command
    uint8_t command[4] = { 0 };

    int ret = hal_spi_transfer( command, NULL, sizeof( command ) );
    if( ret < 0 )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    // Wait for radio to be ready (BUSY low)
    lr11xx_hal_wait_on_busy( );

    return LR11XX_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
