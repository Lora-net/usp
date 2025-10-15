/**
 * @file      main_ranging_demo.c
 *
 * @brief     Main application file for the LR11xx ranging and frequency hopping demo.
 *
 * This file implements the entry point and main loop for the ranging demo application.
 * It configures the MCU, initializes the RAC, sets up the user button and LEDs,
 * and manages the main application flow for both manager (master) and subordinate (slave) modes.
 * Optionally, it can also schedule periodic uplink transmissions for demonstration or testing purposes.
 *
 * Key features:
 *   - Initializes all MCU peripherals, radio, and RAC.
 *   - Configures the device as either manager or subordinate based on compile-time macros.
 *   - Handles user button input to trigger ranging exchanges.
 *   - Manages LED indicators for TX, RX, and scan status.
 *   - Supports periodic uplink transmissions if enabled.
 *   - Enters low-power sleep mode between events, reloading the watchdog as needed.
 *
 * Usage:
 *   - Include this file in your project as the main application entry point.
 *   - Configure the device mode and periodic uplink options in main_ranging_demo.h.
 *   - Build and flash to a supported board with LR11xx or LR1120 radio.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 * Includes all required headers for radio abstraction, RAC API, hardware abstraction,
 * and application configuration.
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "ral_defs.h"
#include "ral.h"
#include "ralf_defs.h"
#include "ralf.h"
#include "smtc_rac_api.h"
#include "smtc_modem_hal.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"

#include "modem_pinout.h"
#include "smtc_sw_platform_helper.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "MAIN-RANGING"
#include "smtc_rac_log.h"

#include <string.h>
#include "app_ranging_hopping.h"
#include "apps_configuration.h"
#include "main_ranging_demo.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 * (No private macros defined in this file.)
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 * (No private constants defined in this file.)
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 * Defines a volatile flag for user button press detection.
 */
static volatile bool user_button_is_press = false;  // Flag for button status

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 * Defines application-level variables for manager/subordinate mode,
 * periodic transmission configuration, and payload buffer.
 */

// Set is_manager based on compile-time macro (see main_ranging_demo.h)
#if defined( RANGING_DEVICE_MODE ) && ( RANGING_DEVICE_MODE == RANGING_DEVICE_MODE_MANAGER )
static const bool is_manager = true;
#elif defined( RANGING_DEVICE_MODE ) && ( RANGING_DEVICE_MODE == RANGING_DEVICE_MODE_SUBORDINATE )
static const bool is_manager = false;
#else
#error Application must define RANGING_DEVICE_MODE
#endif

static smtc_rac_context_t* periodic_tx_config      = NULL;   // Context for periodic uplink
static uint8_t             periodic_tx_payload[33] = { 0 };  // Payload buffer for periodic uplink
static uint8_t             periodic_tx_handle;               // Radio access handle for periodic uplink

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 * Prototypes for internal helper functions.
 */
static void user_button_callback( void* context );
static void periodic_tx_handle_callback( rp_status_t status );
static void periodic_tx_handle_config( void );
static void periodic_tx_handle_start( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 *
 * Initializes the MCU, radio, and RAC, configures the user button and LEDs,
 * and enters the main application loop. Handles both manager and subordinate roles,
 * and optionally schedules periodic uplink transmissions.
 *
 * @return int (not used, as this is an embedded application)
 */
int main_ranging_demo( void )
{
    // Disable IRQ to avoid unwanted behavior during init
    hal_mcu_disable_irq( );

    // Configure all the µC peripherals (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Initialize the RAC
    SMTC_SW_PLATFORM( smtc_rac_init( ) );
    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "===== ranging and frequency hopping example =====\r\n" );

    set_led( SMTC_PF_LED_SCAN, true );
    // Configure Nucleo blue button as EXTI (external interrupt)
    hal_gpio_irq_t nucleo_blue_button = {
        .pin      = EXTI_BUTTON,
        .context  = NULL,                  // context passed to the callback - not used in this example
        .callback = user_button_callback,  // callback called when EXTI is triggered
    };
    hal_gpio_init_in( EXTI_BUTTON, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &nucleo_blue_button );

    hal_mcu_enable_irq( );
    if( is_manager == true )
    {
        set_led( SMTC_PF_LED_TX, true );
        set_led( SMTC_PF_LED_RX, false );
        SMTC_HAL_TRACE_INFO( "Running in ranging manager mode\n" );
        app_radio_ranging_params_init( is_manager, RAC_HIGH_PRIORITY );
#if defined( CONTINUOUS_RANGING ) && ( CONTINUOUS_RANGING == true )
        start_ranging_exchange( 0, is_manager );
#endif
    }
    else
    {
        set_led( SMTC_PF_LED_TX, false );
        set_led( SMTC_PF_LED_RX, true );
        SMTC_HAL_TRACE_INFO( "Running in ranging subordinate mode\n" );
        app_radio_ranging_params_init( is_manager, RAC_HIGH_PRIORITY );
        start_ranging_exchange( 0, is_manager );
    }
    // If periodic uplink is enabled, configure and start periodic transmissions
    if( PERIODIC_UPLINK_ENABLED == true )
    {
        periodic_tx_handle = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_VERY_HIGH_PRIORITY ) );
        periodic_tx_config = smtc_rac_get_context( periodic_tx_handle );
        periodic_tx_config->scheduler_config.callback_post_radio_transaction = periodic_tx_handle_callback;
        periodic_tx_handle_config( );
        periodic_tx_handle_start( );
    }
    // Main application loop
    while( 1 )
    {
        if( user_button_is_press == true )
        {
            user_button_is_press = false;
        }
        // Atomically check sleep conditions (button was not pressed and no modem flags pending)
        hal_mcu_disable_irq( );
        smtc_rac_run_engine( );

        if( ( user_button_is_press == false ) )
        {
            hal_watchdog_reload( );
            hal_mcu_set_sleep_for_ms( 1000 );
        }
        hal_watchdog_reload( );
        hal_mcu_enable_irq( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

/**
 * @brief Callback for user button press (EXTI interrupt).
 *
 * Debounces the button and sets the user_button_is_press flag.
 * Also triggers a new ranging exchange.
 *
 * @param context Not used in this example.
 */
static void user_button_callback( void* context )
{
    static uint32_t last_press_timestamp_ms = 0;
    if( ( int32_t ) ( smtc_modem_hal_get_time_in_ms( ) - last_press_timestamp_ms ) > 500 )
    {
        last_press_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
        user_button_is_press    = true;
    }
    start_ranging_exchange( 0, is_manager );
}

/**
 * @brief Callback for periodic uplink transmission.
 *
 * @param status Not used in this example.
 */
static void periodic_tx_handle_callback( rp_status_t status )
{
    UNUSED( status );
    periodic_tx_handle_start( );
}

/**
 * @brief Configures the periodic uplink transmission parameters.
 *
 * Sets up the radio parameters, payload, and scheduling for periodic transmissions.
 */
static void periodic_tx_handle_config( void )
{
    periodic_tx_config->modulation_type                                 = SMTC_RAC_MODULATION_LORA;
    periodic_tx_config->radio_params.lora.frequency_in_hz               = 868000000;
    periodic_tx_config->radio_params.lora.tx_power_in_dbm               = 14;
    periodic_tx_config->radio_params.lora.preamble_len_in_symb          = 8;
    periodic_tx_config->radio_params.lora.header_type                   = RAL_LORA_PKT_EXPLICIT;
    periodic_tx_config->radio_params.lora.invert_iq_is_on               = 0;
    periodic_tx_config->radio_params.lora.crc_is_on                     = 1;
    periodic_tx_config->radio_params.lora.sync_word                     = LORA_PUBLIC_NETWORK_SYNCWORD;
    periodic_tx_config->radio_params.lora.sf                            = RAL_LORA_SF12;
    periodic_tx_config->radio_params.lora.bw                            = RAL_LORA_BW_125_KHZ;
    periodic_tx_config->radio_params.lora.cr                            = RAL_LORA_CR_4_5;
    periodic_tx_config->radio_params.lora.is_tx                         = true;
    periodic_tx_config->smtc_rac_data_buffer_setup.tx_payload_buffer    = periodic_tx_payload;
    periodic_tx_config->radio_params.lora.tx_size                       = sizeof( periodic_tx_payload );
    periodic_tx_config->scheduler_config.scheduling                     = SMTC_RAC_ASAP_TRANSACTION;
    periodic_tx_config->scheduler_config.start_time_ms                  = smtc_modem_hal_get_time_in_ms( );
    periodic_tx_config->scheduler_config.callback_pre_radio_transaction = NULL;
}

/**
 * @brief Start the periodic uplink transmission.
 *
 * Schedules the next periodic uplink and sends the current payload.
 */
static void periodic_tx_handle_start( void )
{
    periodic_tx_config->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + TX_PERIODICITY_IN_MS;
    smtc_rac_submit_radio_transaction( periodic_tx_handle );
    SMTC_HAL_TRACE_INFO( "Periodic tx done\n" );
}

/* --- EOF ------------------------------------------------------------------ */
