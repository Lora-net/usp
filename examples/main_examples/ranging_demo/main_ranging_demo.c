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
 * Copyright Semtech Corporation 2026. All rights reserved.
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
#include "smtc_hal_led.h"
#include "smtc_hal_button.h"

#include "modem_pinout.h"

#include "smtc_sw_platform_helper.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "MAIN-RANGING"
#include "smtc_rac_log.h"

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
 */

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
int main( void )
{
    // Disable IRQ to avoid unwanted behavior during init
    hal_mcu_disable_irq( );

    // Configure all the µC peripherals (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Initialize LEDs and button
    hal_led_init( );
    hal_button_init( NULL, NULL );

    // Initialize the RAC
    SMTC_SW_PLATFORM( smtc_rac_init( ) );

    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "===== ranging and frequency hopping example =====\r\n" );

    if( is_manager == true )
    {
        hal_led_set( HAL_LED_TX, true );
        hal_led_set( HAL_LED_RX, false );
        SMTC_HAL_TRACE_INFO( "Running in ranging manager mode\n" );
        app_radio_ranging_params_init( is_manager, RAC_HIGH_PRIORITY );
#if defined( CONTINUOUS_RANGING ) && ( CONTINUOUS_RANGING == true )
        start_ranging_exchange( 0, is_manager );
#endif
    }
    else
    {
        hal_led_set( HAL_LED_TX, false );
        hal_led_set( HAL_LED_RX, true );
        SMTC_HAL_TRACE_INFO( "Running in ranging subordinate mode\n" );
        app_radio_ranging_params_init( is_manager, RAC_HIGH_PRIORITY );
        start_ranging_exchange( 0, is_manager );
    }

    // If periodic uplink is enabled, configure and start periodic transmissions
    SMTC_HAL_TRACE_INFO( "Periodic uplink enabled: %d\n", PERIODIC_UPLINK_ENABLED );
    if( PERIODIC_UPLINK_ENABLED == true )
    {
        periodic_tx_handle = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_VERY_HIGH_PRIORITY ) );
        periodic_tx_config = smtc_rac_get_context( periodic_tx_handle );
        periodic_tx_handle_config( );
        periodic_tx_handle_start( );
    }

    // Main application loop
    while( 1 )
    {
        if( hal_button_is_pressed( ) )
        {
            hal_button_clear( );
            start_ranging_exchange( 0, is_manager );
        }

        // Atomically check sleep conditions (button was not pressed and no modem flags pending)
        hal_mcu_disable_irq( );
        smtc_rac_run_engine( );
        if( !hal_button_is_pressed( ) )
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
    periodic_tx_config->modulation_type                                      = SMTC_RAC_MODULATION_LORA;
    periodic_tx_config->radio_params.lora.frequency_in_hz                    = 868000000;
    periodic_tx_config->radio_params.lora.tx_power_in_dbm                    = 14;
    periodic_tx_config->radio_params.lora.preamble_len_in_symb               = 8;
    periodic_tx_config->radio_params.lora.header_type                        = RAL_LORA_PKT_EXPLICIT;
    periodic_tx_config->radio_params.lora.invert_iq_is_on                    = 0;
    periodic_tx_config->radio_params.lora.crc_is_on                          = 1;
    periodic_tx_config->radio_params.lora.sync_word                          = LORA_PUBLIC_NETWORK_SYNCWORD;
    periodic_tx_config->radio_params.lora.sf                                 = RAL_LORA_SF12;
    periodic_tx_config->radio_params.lora.bw                                 = RAL_LORA_BW_125_KHZ;
    periodic_tx_config->radio_params.lora.cr                                 = RAL_LORA_CR_4_5;
    periodic_tx_config->radio_params.lora.is_tx                              = true;
    periodic_tx_config->smtc_rac_data_buffer_setup.tx_payload_buffer         = periodic_tx_payload;
    periodic_tx_config->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( periodic_tx_payload );
    periodic_tx_config->radio_params.lora.tx_size                            = sizeof( periodic_tx_payload );
    periodic_tx_config->scheduler_config.scheduling                          = SMTC_RAC_ASAP_TRANSACTION;
    periodic_tx_config->scheduler_config.start_time_ms                       = smtc_modem_hal_get_time_in_ms( );
    periodic_tx_config->scheduler_config.callback_pre_radio_transaction      = NULL;
    periodic_tx_config->scheduler_config.callback_post_radio_transaction     = periodic_tx_handle_callback;
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
