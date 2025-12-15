/**
 * @file      main_per_flrc.c
 *
 * @brief     Simple PER (Packet Error Rate) example with FLRC modulation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
 */

#include "app_per_flrc.h"

#include "modem_pinout.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_watchdog.h"
#include "smtc_rac_api.h"

#define RAC_LOG_APP_PREFIX "MAIN-PER-FLRC"
#include "smtc_rac_log.h"

static const uint32_t SLEEP_DELAY          = 1000;  // ms
static const uint32_t BUTTON_TRIGGER_DELAY = 500;   // ms

typedef struct user_button_s
{
    bool     is_pressed;
    uint32_t last_press_timestamp;
} user_button_t;

static user_button_t user_button = {
    .is_pressed           = false,
    .last_press_timestamp = 0,
};

static void user_button_callback( void* context )
{
    UNUSED( context );
    uint32_t timestamp = smtc_modem_hal_get_time_in_ms( );
    if( ( timestamp - user_button.last_press_timestamp ) > BUTTON_TRIGGER_DELAY )
    {
        user_button.last_press_timestamp = timestamp;
        user_button.is_pressed           = true;
    }
}

void main_per_flrc( void )
{
    hal_mcu_init( );
    smtc_rac_init( );

    hal_gpio_irq_t nucleo_blue_button = {
        .pin      = EXTI_BUTTON,
        .context  = &user_button,
        .callback = user_button_callback,
    };
    hal_gpio_init_in( EXTI_BUTTON, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &nucleo_blue_button );

    hal_gpio_init_out( SMTC_LED_TX, false );
    hal_gpio_init_out( SMTC_LED_RX, false );

    per_flrc_init( );

    while( true )
    {
        hal_watchdog_reload( );
        smtc_rac_run_engine( );

        if( user_button.is_pressed == true )
        {
            RAC_LOG_INFO( "button pressed\n" );
            user_button.is_pressed = false;
            per_flrc_on_button_press( );
        }

        hal_mcu_disable_irq( );
        if( ( user_button.is_pressed == false ) && ( smtc_rac_is_irq_flag_pending( ) == false ) )
        {
            hal_mcu_set_sleep_for_ms( SLEEP_DELAY );
            hal_watchdog_reload( );
        }
        hal_mcu_enable_irq( );
    }
}
