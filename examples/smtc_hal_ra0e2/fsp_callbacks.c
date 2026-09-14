/**
 * @file      fsp_callbacks.c
 *
 * @brief     FSP peripheral callback implementations for RA0E2
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

#include "hal_data.h"
#include "common_data.h"
#include "smtc_hal_trace.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_tau.h"
#include "smtc_hal_rtc.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* Button debounce time in milliseconds */
#define BUTTON_DEBOUNCE_MS 50U

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/* Last button press timestamp for debounce */
static volatile uint32_t last_button_press_ms = 0;

/* =======================================================================================
 * RTC and TML Callbacks
 *
 * Note: rtc_callback is implemented in smtc_hal_rtc.c
 *       tml1_lp_timer_callback is implemented in smtc_hal_lp_timer.c
 *       TML0 has no callback (NULL in hal_data.c)
 * ======================================================================================= */

/* =======================================================================================
 * TAU (Timer Array Unit) Callback
 * ======================================================================================= */

/**
 * @brief TAU0 callback - for precise microsecond delays (hal_mcu_wait_us)
 *
 * TAU channel 0 is configured at 1 MHz (1 µs per tick) for hardware-timed
 * delays up to 65.535 ms. Used by radio HAL for precise timing operations.
 *
 * @param p_args Callback arguments from FSP TAU driver
 */
void tau0_callback( timer_callback_args_t* p_args )
{
    if( p_args == NULL )
    {
        return;
    }

    if( p_args->event == TIMER_EVENT_CYCLE_END )
    {
        /* Timer cycle ended - delay complete */
        hal_tau_irq_handler( );
    }
}

/* =======================================================================================
 * ICU (Interrupt Controller Unit) - External IRQ Callbacks
 * ======================================================================================= */

/**
 * @brief Radio DIO external interrupt callback (IRQ5 on P201)
 *
 * This callback is triggered when the radio asserts its DIO interrupt line
 * (active high, rising edge). The IRQ is configured on P201 (ARDUINO_D5).
 *
 * @param p_args External IRQ callback arguments from FSP ICU driver
 */
void radio_dio_irq_callback( external_irq_callback_args_t* p_args )
{
    if( p_args == NULL )
    {
        return;
    }

    // Verify this is IRQ5 (radio DIO on P201)
    if( p_args->channel == 5 )
    {
        // Dispatch to the registered GPIO IRQ handler
        // P201 = Port 2, Pin 1 = 0x0201
        hal_gpio_irq_dispatch( 0x0201 );
    }
}

/**
 * @brief User button external interrupt callback (IRQ0 on P200)
 *
 * This callback is triggered when the user button is pressed (active low, falling edge).
 * The button is connected to P200 with internal pull-up.
 *
 * Software debounce is implemented since RA0E2 does not have hardware digital
 * filtering on external IRQs (BSP_FEATURE_ICU_HAS_FILTER = 0).
 *
 * @param p_args External IRQ callback arguments from FSP ICU driver
 */
void user_button_irq_callback( external_irq_callback_args_t* p_args )
{
    if( p_args == NULL )
    {
        return;
    }

    /* Verify this is IRQ0 (user button on P200) */
    if( p_args->channel == 0 )
    {
        /* Software debounce - ignore bounces within BUTTON_DEBOUNCE_MS */
        uint32_t now = hal_rtc_get_time_ms( );
        if( ( now - last_button_press_ms ) < BUTTON_DEBOUNCE_MS )
        {
            return; /* Ignore bounce */
        }
        last_button_press_ms = now;

        /* Dispatch to the registered GPIO IRQ handler */
        /* P200 = Port 2, Pin 0 = 0x0200 */
        hal_gpio_irq_dispatch( 0x0200 );
    }
}

/* EOF */
