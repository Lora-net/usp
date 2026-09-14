/**
 * @file      smtc_hal_led.c
 *
 * @brief     LED Hardware Abstraction Layer implementation for Linux
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

#include "smtc_hal_led.h"
#include "smtc_hal_gpio.h"
#include "modem_pinout.h"

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
 * Mapping from LED identifiers to GPIO pins
 */
static const hal_gpio_pin_names_t led_pins[HAL_LED_COUNT] = {
    [HAL_LED_TX]   = SMTC_LED_TX,
    [HAL_LED_RX]   = SMTC_LED_RX,
    [HAL_LED_SCAN] = SMTC_LED_SCAN,
};

/*!
 * Current state of each LED (for toggle functionality)
 */
static bool led_states[HAL_LED_COUNT] = { false, false, false };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_led_init( void )
{
    for( int i = 0; i < HAL_LED_COUNT; i++ )
    {
        if( led_pins[i] != NC )
        {
            hal_gpio_init_out( led_pins[i], 0 );
            led_states[i] = false;
        }
    }
}

void hal_led_set( hal_led_id_t led, bool state )
{
    if( led >= HAL_LED_COUNT )
    {
        return;
    }

    if( led_pins[led] != NC )
    {
        hal_gpio_set_value( led_pins[led], state ? 1 : 0 );
        led_states[led] = state;
    }
}

void hal_led_toggle( hal_led_id_t led )
{
    if( led >= HAL_LED_COUNT )
    {
        return;
    }

    if( led_pins[led] != NC )
    {
        led_states[led] = !led_states[led];
        hal_gpio_set_value( led_pins[led], led_states[led] ? 1 : 0 );
    }
}

void hal_led_toggle_tx_rx( void )
{
    hal_led_toggle( HAL_LED_TX );
    hal_led_toggle( HAL_LED_RX );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
