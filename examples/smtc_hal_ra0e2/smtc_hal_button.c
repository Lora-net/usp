/**
 * @file      smtc_hal_button.c
 *
 * @brief     Button Hardware Abstraction Layer implementation for STM32
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

#include "smtc_hal_button.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_rtc.h"
#include "smtc_hal_dbg_trace.h"
#include "modem_pinout.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * Debounce time in milliseconds
 */
#define BUTTON_DEBOUNCE_MS 500

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
 * Button pressed flag (volatile for ISR access)
 */
static volatile bool button_is_pressed = false;

/*!
 * Last press timestamp for debounce
 */
static uint32_t last_press_timestamp_ms = 0;

/*!
 * GPIO IRQ structure for button
 */
static hal_gpio_irq_t button_irq;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * Internal button callback for GPIO IRQ
 */
static void button_gpio_callback( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_button_init( hal_button_callback_t callback, void* context )
{
    // Store user callback and context
    button_callback = callback;
    button_context  = context;

    // Check if button pin is connected
    if( EXTI_BUTTON == NC )
    {
        SMTC_HAL_TRACE_WARNING( "Button: Pin not connected (NC)\n" );
        return;
    }

    // Configure button GPIO IRQ structure
    button_irq.pin      = EXTI_BUTTON;
    button_irq.context  = NULL;
    button_irq.callback = button_gpio_callback;

    // Initialize button GPIO as input with falling edge interrupt
    hal_gpio_init_in( EXTI_BUTTON, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &button_irq );

    SMTC_HAL_TRACE_INFO( "Button: Initialized (GPIO EXTI on pin 0x%02X)\n", EXTI_BUTTON );
}

bool hal_button_is_pressed( void )
{
    return button_is_pressed;
}

void hal_button_clear( void )
{
    button_is_pressed = false;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void button_gpio_callback( void* context )
{
    ( void ) context;

    // Debounce: ignore presses within debounce period
    uint32_t current_time_ms = hal_rtc_get_time_ms( );
    if( ( int32_t ) ( current_time_ms - last_press_timestamp_ms ) > BUTTON_DEBOUNCE_MS )
    {
        last_press_timestamp_ms = current_time_ms;
        button_is_pressed       = true;

        // Call user callback if registered
        if( button_callback != NULL )
        {
            button_callback( button_context );
        }
    }
}

/* --- EOF ------------------------------------------------------------------ */
