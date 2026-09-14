/**
 * @file      smtc_hal_spi.c
 *
 * @brief     SPI Hardware Abstraction Layer implementation for RA0E2
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

#include "smtc_hal_spi.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_mcu.h"

// FSP includes
#include "hal_data.h"
#include "r_sau_spi.h"
#include "r_spi_api.h"
#include "bsp_api.h"
#include "r_ioport_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * =============================================================================
 * POLLING MODE SPI
 * =============================================================================
 * Uses direct register access for ISR-safe SPI transfers.
 * This avoids the interrupt-driven FSP approach that hangs when called
 * from within an ISR (e.g., radio IRQ handler).
 *
 * SAU SPI Channel 0 registers:
 * - R_SAU0->SDR[0]: Serial Data Register (read/write data)
 * - R_SAU0->SSR[0]: Serial Status Register
 *   - BFF (bit 5): Buffer Full Flag - 1 when receive data available
 *   - TSF (bit 6): Transfer Status Flag - 1 when communication in progress
 * =============================================================================
 */

/* SAU SPI channel used (configured in e2studio as channel 0) */
#define SAU_SPI_CHANNEL 0

/* SSR register bit masks */
#define SAU_SSR_BFF_Msk ( 0x20U ) /* Buffer Full Flag - bit 5 */
#define SAU_SSR_TSF_Msk ( 0x40U ) /* Transfer Status Flag - bit 6 */

/* Polling timeout (iterations, not time-based for speed) */
#define SPI_POLL_TIMEOUT 10000U

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

static volatile bool spi_initialized = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_spi_init( const uint32_t id, const hal_gpio_pin_names_t mosi, const hal_gpio_pin_names_t miso,
                   const hal_gpio_pin_names_t sclk )
{
    ( void ) id;
    ( void ) mosi;
    ( void ) miso;
    ( void ) sclk;

    if( !spi_initialized )
    {
        /*
         * Restore SPI pins to peripheral mode (required after wakeup from standby).
         * hal_spi_de_init() configures these as GPIO for low-power, so we need to
         * reconfigure them back to SAU SPI peripheral function.
         *
         * Pin configuration from pin_data.c / e2studio:
         * - P500 (SCLK): IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SAU2_OUT
         * - P501 (MOSI): IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SAU2_OUT
         * - P502 (MISO): IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SAU2
         */
        R_BSP_PinAccessEnable( );

        /* SCLK (P500): SAU SPI clock output */
        R_BSP_PinCfg( BSP_IO_PORT_05_PIN_00, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SAU2_OUT );

        /* MOSI (P501): SAU SPI data output */
        R_BSP_PinCfg( BSP_IO_PORT_05_PIN_01, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SAU2_OUT );

        /* MISO (P502): SAU SPI data input */
        R_BSP_PinCfg( BSP_IO_PORT_05_PIN_02, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SAU2 );

        R_BSP_PinAccessDisable( );

        /* Open SAU SPI peripheral */
        fsp_err_t err = R_SAU_SPI_Open( &g_spi0_ctrl, &g_spi0_cfg );
        if( err == FSP_SUCCESS )
        {
            spi_initialized = true;
        }
    }
}

void hal_spi_de_init( const uint32_t id )
{
    ( void ) id;

    if( spi_initialized )
    {
        R_SAU_SPI_Close( &g_spi0_ctrl );
        spi_initialized = false;

        /*
         * Configure SPI pins for low-power mode (following STM32 HAL pattern):
         * - MISO: Input with pull-up - radio hi-Z when deselected, pull-up for defined state
         * - MOSI: Output LOW - equivalent to STM32's input with pull-down
         * - SCLK: Output LOW - equivalent to STM32's input with pull-down
         *
         * Note: P502 (MISO) doesn't have analog capability on RA0E2.
         * RA0E2 doesn't have internal pull-down, so we use output LOW for
         * MOSI/SCLK to match STM32's pull-down behavior.
         */
        R_BSP_PinAccessEnable( );

        /* MISO (P502): Input with pull-up (radio hi-Z when deselected) */
        R_BSP_PinCfg( BSP_IO_PORT_05_PIN_02, IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE );

        /* MOSI (P501): Output LOW (matches STM32 pull-down behavior) */
        R_BSP_PinCfg( BSP_IO_PORT_05_PIN_01, IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW );

        /* SCLK (P500): Output LOW (matches STM32 pull-down behavior) */
        R_BSP_PinCfg( BSP_IO_PORT_05_PIN_00, IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW );

        R_BSP_PinAccessDisable( );
    }
}

uint16_t hal_spi_in_out( const uint32_t id, const uint16_t out_data )
{
    ( void ) id;

    if( !spi_initialized )
    {
        return 0;
    }

    /*
     * Polling-mode SPI transfer (ISR-safe)
     *
     * For SAU SPI in master mode:
     * 1. Write TX data to SDR register (starts clock and transmission)
     * 2. Wait for BFF=1 (receive data ready - indicates transfer complete)
     * 3. Read RX data from SDR register
     *
     * Note: Writing to SDR while channel is active only affects DAT bits (0-8).
     * The STCLK bits are loaded from configuration when channel starts.
     */

    volatile uint16_t*       p_sdr = &R_SAU0->SDR[SAU_SPI_CHANNEL];
    volatile const uint16_t* p_ssr = &R_SAU0->SSR[SAU_SPI_CHANNEL];

    /* Write TX data (just the data byte, like FSP does) */
    *p_sdr = ( uint16_t ) ( out_data & 0x00FFU );

    /* Wait for receive data available (BFF=1 indicates transfer complete) */
    uint32_t timeout = SPI_POLL_TIMEOUT;
    while( timeout > 0 )
    {
        if( ( *p_ssr & SAU_SSR_BFF_Msk ) != 0 )
        {
            break;
        }
        timeout--;
    }

    if( timeout == 0 )
    {
        /* Timeout - transfer did not complete */
        return 0;
    }

    /* Read received data (lower 8 bits of SDR) */
    uint8_t rx_byte = ( uint8_t ) ( *p_sdr & 0x00FFU );

    return ( uint16_t ) rx_byte;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief SPI callback function (called from FSP ISR)
 *
 * Note: With polling-mode SPI, this callback is not used for data transfers.
 * It is kept to satisfy the FSP callback reference in hal_data.c, and may
 * still be called for error conditions if the FSP driver detects them.
 */
void sau_spi_callback( spi_callback_args_t* p_args )
{
    /* Polling mode - callback not used for normal transfers */
    ( void ) p_args;
}

/* --- EOF ------------------------------------------------------------------ */
