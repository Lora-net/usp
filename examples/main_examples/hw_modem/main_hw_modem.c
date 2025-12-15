/**
 * @file      main_hw_modem.c
 *
 * @brief     Application main
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

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#include <smtc_modem_utilities.h>

#include <smtc_rac_api.h>

#define RAC_LOG_APP_PREFIX "HW_MODEM"
#include "smtc_rac_log.h"

#include "hw_modem.h"
#include "cmd_parser.h"
#include "git_version.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_watchdog.h"
#include "smtc_sw_platform_helper.h"

#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/**
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog
 * period (here 32s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS 20000

/**
 * @brief Callback for modem hal
 */
static uint8_t prv_get_battery_level_cb( void )
{
    return 98;
}

/**
 * @brief Callback for modem hal
 */
static uint16_t prv_get_battery_voltage_cb( void )
{
    return 3300;
}

/**
 * @brief Callback for modem hal
 */
static int8_t prv_get_temperature_cb( void )
{
    return 25;
}

#ifdef ADD_FUOTA
static uint32_t prv_get_hw_version_for_fuota( void )
{
    return 1;
}

static uint32_t prv_get_fw_version_for_fuota( void )
{
    return 1;
}

static uint8_t prv_get_fw_status_available_for_fuota( void )
{
    return 1;
}

static uint32_t prv_get_next_fw_version_for_fuota( void )
{
    return 1;
}

static uint8_t prv_get_fw_delete_status_for_fuota( uint32_t version )
{
    return 0;
}

/* Callbacks for HAL implementation */
static struct lorawan_fuota_cb prv_fuota_cb = {
    .get_hw_version          = prv_get_hw_version_for_fuota,
    .get_fw_version          = prv_get_fw_version_for_fuota,
    .get_fw_status_available = prv_get_fw_status_available_for_fuota,
    .get_next_fw_version     = prv_get_next_fw_version_for_fuota,
    .get_fw_delete_status    = prv_get_fw_delete_status_for_fuota,
};
#endif /* CONFIG_LORA_BASICS_MODEM_FUOTA */

void main_hw_modem( void )
{
    // Disable IRQ to avoid unwanted behavior during init
    hal_mcu_disable_irq( );

    // Configure all the µC peripherals (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Initialize the RAC
    SMTC_SW_PLATFORM( smtc_rac_init( ) );

    // Initialize hw_modem
    hw_modem_init( );

    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "Modem is starting\n" );
    SMTC_HAL_TRACE_PRINTF( "Commit SHA1: %s\n", get_software_git_commit( ) );
    SMTC_HAL_TRACE_PRINTF( "Commit date: %s\n", get_software_git_date( ) );
    SMTC_HAL_TRACE_PRINTF( "Build date: %s\n", get_software_build_date( ) );

    uint32_t sleep_time_ms = 0;
    while( 1 )
    {
        // Check if a command is available
        if( hw_modem_is_a_cmd_available( ) == true )
        {
            // Command may generate work for the stack, so drop down to smtc_modem_run_engine().
            hw_modem_process_cmd( );
        }

        // Modem process launch
        sleep_time_ms = smtc_modem_run_engine( );

        // Atomically check sleep conditions (no command available and low power is possible)
        hal_mcu_disable_irq( );
        if( ( hw_modem_is_a_cmd_available( ) == false ) && ( hw_modem_is_low_power_ok( ) == true ) &&
            ( smtc_modem_is_irq_flag_pending( ) == false ) )
        {
            hal_watchdog_reload( );
            hal_mcu_set_sleep_for_ms( MIN( sleep_time_ms, WATCHDOG_RELOAD_PERIOD_MS ) );
        }
        hal_watchdog_reload( );
        hal_mcu_enable_irq( );
    }
}
