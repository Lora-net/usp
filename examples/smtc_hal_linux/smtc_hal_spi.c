/**
 * @file      smtc_hal_spi.c
 *
 * @brief     SPI Hardware Abstraction Layer implementation for Linux
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

#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <unistd.h>

#include "smtc_hal_spi.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define SPI_DEVICE_PATH "/dev/spidev0.0"
#define SPI_SPEED_HZ 8000000
#define SPI_MODE SPI_MODE_0

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static int spi_fd = -1;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_spi_init( void )
{
    // Open SPI device
    spi_fd = open( SPI_DEVICE_PATH, O_RDWR );
    if( spi_fd < 0 )
    {
        printf( "Failed to open SPI device: %s\n", SPI_DEVICE_PATH );
        return;
    }

    // Configure SPI mode
    uint8_t mode = SPI_MODE;
    if( ioctl( spi_fd, SPI_IOC_WR_MODE, &mode ) < 0 )
    {
        printf( "Failed to set SPI mode\n" );
        close( spi_fd );
        return;
    }

    // Configure SPI speed
    uint32_t speed = SPI_SPEED_HZ;
    if( ioctl( spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed ) < 0 )
    {
        printf( "Failed to set SPI speed\n" );
        close( spi_fd );
        return;
    }

    printf( "SPI: Initialized\n" );
}

void hal_spi_de_init( void )
{
    if( spi_fd >= 0 )
    {
        close( spi_fd );
    }
}

int hal_spi_transfer( const uint8_t* tx_buf, uint8_t* rx_buf, size_t len )
{
    struct spi_ioc_transfer tr;
    memset( &tr, 0, sizeof( tr ) );

    tr.tx_buf        = ( unsigned long ) tx_buf;
    tr.rx_buf        = ( unsigned long ) rx_buf;
    tr.len           = len;
    tr.delay_usecs   = 0;
    tr.speed_hz      = SPI_SPEED_HZ;
    tr.bits_per_word = 8;
    // No cs_change needed since we use SPI_NO_CS mode

    return ioctl( spi_fd, SPI_IOC_MESSAGE( 1 ), &tr );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
