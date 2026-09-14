/**
 * @file      smtc_hal_uart.c
 *
 * @brief     UART Hardware Abstraction Layer implementation for RA0E2
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

#include "smtc_hal_uart.h"
#include "smtc_hal_mcu.h"
#include "hal_data.h"     // FSP generated: g_uart0
#include "vector_data.h"  // For UARTA0_TXI_IRQn
#include "bsp_api.h"
#include "r_ioport_api.h"

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

// UART register base (direct access for polling mode)
#define UARTA0_BASE R_UARTA0
#define TXBFA_FLAG ( 1 << 5 )  // Transmit Buffer Data Flag (0=empty, 1=full)
#define TXSFA_FLAG ( 1 << 4 )  // Transmit Shift Register Data Flag (0=empty, 1=busy)

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void trace_uart_init( void )
{
    /*
     * Restore UART pins to peripheral mode (required after wakeup from standby).
     * trace_uart_deinit() configures these as GPIO for low-power, so we need to
     * reconfigure them back to UARTA peripheral function.
     *
     * Pin configuration from pin_data.c / e2studio:
     * - P100 (RX): IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_UARTA5 (RXDA0)
     * - P101 (TX): IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_UARTA5_OUT (TXDA0)
     *
     * Note: These are the VCOM pins on FPB-RA0E2, directly connected to J-Link debugger.
     */
    R_BSP_PinAccessEnable( );

    /* RX (P100): UARTA input */
    R_BSP_PinCfg( BSP_IO_PORT_01_PIN_00, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_UARTA5 );

    /* TX (P101): UARTA output */
    R_BSP_PinCfg( BSP_IO_PORT_01_PIN_01, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_UARTA5_OUT );

    R_BSP_PinAccessDisable( );

    fsp_err_t err = g_uart0.p_api->open( g_uart0.p_ctrl, g_uart0.p_cfg );
    if( FSP_SUCCESS != err )
    {
        // UART initialization failed - halt system
        mcu_panic( );
    }

    // Disable TXI interrupt - we use polling mode for TX, so the TXI interrupt
    // is not needed. FSP enables it by default, and it causes spurious wakeups
    // from WFI if left enabled (hardware keeps re-setting pending bit).
    NVIC_DisableIRQ( UARTA0_TXI_IRQn );
}

void trace_uart_deinit( void )
{
    g_uart0.p_api->close( g_uart0.p_ctrl );

    /*
     * Configure UART pins for low-power mode:
     * - TX (P101): Input with pull-up - high-impedance, let debugger drive
     * - RX (P100): Input with pull-up - prevents floating (matches idle UART line)
     *
     * P100/P101 are the VCOM pins on FPB-RA0E2, directly connected to J-Link.
     * Using input mode avoids fighting the debugger's UART lines.
     */
    R_BSP_PinAccessEnable( );

    /* TX (P101): Input with pull-up - let debugger drive */
    R_BSP_PinCfg( BSP_IO_PORT_01_PIN_01, IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE );

    /* RX (P100): Input with pull-up to prevent floating */
    R_BSP_PinCfg( BSP_IO_PORT_01_PIN_00, IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE );

    R_BSP_PinAccessDisable( );
}

void trace_uart_tx( uint8_t* buff, uint8_t len )
{
    if( ( buff == NULL ) || ( len == 0 ) )
    {
        return;
    }

    // POLLING MODE: ISR-safe, can be called from any context
    // Transmit each byte with polling (no interrupts required)
    for( uint8_t i = 0; i < len; i++ )
    {
        // Wait for Transmit Buffer Empty (TXBFA=0 means buffer is empty)
        // At 115200 baud: ~87 µs per byte
        while( UARTA0_BASE->ASISAn & TXBFA_FLAG )
        {
            // Busy-wait for TX buffer ready (TXBFA=1 means full, wait until 0)
        }

        // Write byte to transmit buffer register
        UARTA0_BASE->TXBAn = buff[i];
    }

    // Wait for final byte transmission complete (TXSFA=0 means shift register empty)
    while( UARTA0_BASE->ASISAn & TXSFA_FLAG )
    {
        // Busy-wait for transmission end (TXSFA=1 means busy, wait until 0)
    }
}

/**
 * @brief FSP UART callback - called from interrupt context
 *
 * NOTE: Not used for TX (polling mode), but kept for future RX interrupt support
 *
 * @param p_args Callback arguments from FSP UART driver
 */
void uart0_callback( uart_callback_args_t* p_args )
{
    ( void ) p_args;  // Unused in polling mode
    // Future: RX interrupt handling can be added here
}

/*
 * -----------------------------------------------------------------------------
 * --- HW MODEM UART STUBS (Not needed for Iteration 2) -----------------------
 */

void hw_modem_uart_init( void )
{
    // Not implemented for RA0E2 - no modem UART support yet
}

void hw_modem_uart_deinit( void )
{
    // Not implemented for RA0E2
}

void hw_modem_uart_tx( uint8_t* buff, uint8_t len )
{
    ( void ) buff;
    ( void ) len;
    // Not implemented for RA0E2
}

void hw_modem_uart_dma_start_rx( uint8_t* buff, uint16_t size )
{
    ( void ) buff;
    ( void ) size;
    // Not implemented for RA0E2
}

void hw_modem_uart_dma_stop_rx( void )
{
    // Not implemented for RA0E2
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
