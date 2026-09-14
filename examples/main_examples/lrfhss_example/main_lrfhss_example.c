/**
 * @file      main_lrfhss_example.c
 *
 * @brief     Main file for LR-FHSS example using USP/RAC
 *
 * This example demonstrates how to use LR-FHSS modulation with the RAC framework.
 * LR-FHSS is transmission-only, so this example only shows transmission functionality.
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
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
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

#include "app_lrfhss_example.h"

#include "smtc_hal_button.h"
#include "smtc_hal_led.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_watchdog.h"
#include "smtc_rac_api.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "MAIN-LRFHSS"
#include "smtc_rac_log.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static const uint32_t SLEEP_DELAY = 100;  // ms

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

int main( void )
{
    // initialize device systems
    hal_mcu_init( );

    // Initialize LEDs and button
    hal_led_init( );
    hal_button_init( NULL, NULL );

    hal_watchdog_init( );
    smtc_rac_init( );

    // initialize LR-FHSS application
    lrfhss_example_init( );

    RAC_LOG_INFO( "LR-FHSS example started - press button to trigger immediate transmission cycle" );

    // infinite loop
    while( true )
    {
        // tell the watchdog that the device is still running
        hal_watchdog_reload( );

        // periodic call to progress rac transactions
        smtc_rac_run_engine( );

        // process LR-FHSS application
        if( !lrfhss_example_process( ) )
        {
            RAC_LOG_ERROR( "LR-FHSS application error" );
            break;
        }

        // handle button logic
        if( hal_button_is_pressed( ) )
        {
            RAC_LOG_INFO( "Button pressed - starting immediate transmission cycle" );
            hal_button_clear( );

            // Trigger immediate transmission cycle
            lrfhss_example_start_cycle( );
        }

        // handle sleep
        hal_mcu_disable_irq( );
        if( ( !hal_button_is_pressed( ) ) && ( smtc_rac_is_irq_flag_pending( ) == false ) )
        {
            hal_mcu_set_sleep_for_ms( SLEEP_DELAY );
            hal_watchdog_reload( );  // update watchdog after sleep
        }
        hal_mcu_enable_irq( );
    }

    return 0;
}
