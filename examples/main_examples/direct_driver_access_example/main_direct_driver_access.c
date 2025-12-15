/**
 * @file      main_direct_driver_access.c
 *
 * @brief     Direct radio driver access example for LR20xx chip
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

#include "app_direct_driver_access.h"
#include "main_direct_driver_access.h"

#include "modem_pinout.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_watchdog.h"
#include "smtc_rac_api.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "MAIN-DIRECT-DRIVER"
#include "smtc_rac_log.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static const uint32_t SLEEP_DELAY          = 1000;  // ms
static const uint32_t BUTTON_TRIGGER_DELAY = 500;   // ms

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct user_button_s
{
    bool     is_pressed;            // is the button pressed
    uint32_t last_press_timestamp;  // last absolute time the button was pressed
} user_button_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile user_button_t user_button = {
    .is_pressed           = false,
    .last_press_timestamp = 0,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Update `user_button`
 *
 * This function is called in IRQ context
 *
 */
static void user_button_callback( void* context );

/**
 * @brief Get the and reset user button pressed status
 *
 * This function ensures the state is reset and returned protected from IRQ context.
 *
 * @return true The button has been pressed, its state is now reset
 * @return false The button has not been pressed
 */
static bool get_and_reset_user_button_state( void );

/**
 * @brief Initialise example HAL
 *
 * This function initialize example HAL:
 * - MCU
 * - user button
 * - LEDs
 */
static void init_hal( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void main_direct_driver_access( void )
{
    // Initialize example HAL
    init_hal( );

    // Initialize Radio Access Controller (RAC)
    smtc_rac_init( );

    // Initialize and start applications
    direct_driver_access_init( );

    // infinite loop
    while( true )
    {
        // tell the watchdog that the device is still running
        hal_watchdog_reload( );

        // periodic call to progress rac transactions
        smtc_rac_run_engine( );

        // handle logic
        if( get_and_reset_user_button_state( ) == true )
        {
            RAC_LOG_INFO( "button pressed\n" );
            direct_driver_access_on_button_press( );
        }

        // handle sleep
        hal_mcu_disable_irq( );
        if( ( user_button.is_pressed == false ) && ( smtc_rac_is_irq_flag_pending( ) == false ) )
        {
            hal_mcu_set_sleep_for_ms( SLEEP_DELAY );
            hal_watchdog_reload( );  // update watchdog after sleep
        }
        hal_mcu_enable_irq( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void user_button_callback( void* context )
{
    UNUSED( context );

    uint32_t timestamp = smtc_modem_hal_get_time_in_ms( );

    // avoid multiple triggers
    if( ( timestamp - user_button.last_press_timestamp ) > BUTTON_TRIGGER_DELAY )
    {
        user_button.last_press_timestamp = timestamp;
        user_button.is_pressed           = true;
    }
}

bool get_and_reset_user_button_state( void )
{
    hal_mcu_disable_irq( );
    const bool is_button_pressed = user_button.is_pressed;
    user_button.is_pressed       = false;
    hal_mcu_enable_irq( );
    return is_button_pressed;
}

void init_hal( void )
{
    hal_mcu_init( );

    // setup user button
    static hal_gpio_irq_t nucleo_blue_button = {
        .pin      = EXTI_BUTTON,
        .context  = 0,
        .callback = user_button_callback,
    };
    hal_gpio_init_in( EXTI_BUTTON, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &nucleo_blue_button );

    // initialize LEDs
    hal_gpio_init_out( SMTC_LED_TX, 0 );
    hal_gpio_init_out( SMTC_LED_RX, 0 );
}