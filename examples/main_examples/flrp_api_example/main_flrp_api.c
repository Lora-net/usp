/*!
 * \file      main_flrp_api.c
 *
 * \brief     main program for FLRP API example
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
 */
#include <stdio.h>
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <string.h>

#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "MAIN-FLRP-API"

#include "smtc_hal_mcu.h"
#include "smtc_hal_button.h"
#include "smtc_hal_led.h"
#include "smtc_hal_watchdog.h"

#include "smtc_rac_api.h"
#include "app_flrp_api.h"

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

#if( FLRP_API_ROLE == FLRP_API_ROLE_INITIATOR )
static uint32_t periodic_timestamp_ms = 0;
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
/**
 * @brief Example to send data using the FLRC protocol API
 *
 */
int main( void )
{
    static bool flrp_tx = true;

    hal_mcu_init( );
    hal_led_init( );
    hal_button_init( NULL, NULL );

    smtc_rac_init( );


    flrp_api_init( FLRP_API_CRYPTO_ENABLED, FLRP_API_IS_LOW_FREQUENCY, FLRP_API_IS_LISTENING );

    while( true )
    {
        hal_watchdog_reload( );
        smtc_rac_run_engine( );
        smtc_flrp_run_engine( );

        if ( hal_button_is_pressed( ) == true )
        {
            hal_button_clear( );
            SMTC_HAL_TRACE_INFO( "User button pressed\n" );
            if( smtc_flrp_call_run( ) == false )
            {
#if( FLRP_API_ROLE == FLRP_API_ROLE_INITIATOR )
                periodic_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
#endif
                flrp_tx = flrp_api_initiate_transfer( flrp_tx );
            }
            else
            {
                SMTC_HAL_TRACE_INFO( "FLRP API is busy, cannot initiate transfer\n" );
            }
        }
        else
        {
#if( FLRP_API_ROLE == FLRP_API_ROLE_INITIATOR )
        if( ( smtc_flrp_call_run( ) == false ) &&
        ( ( smtc_modem_hal_get_time_in_ms( ) - periodic_timestamp_ms ) > FLRP_API_INTIATOR_PERIODIC_TRANSFER ) )
        {
                periodic_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
                flrp_tx = flrp_api_initiate_transfer( flrp_tx );
        }
#endif
        }


        // handle sleep
        hal_mcu_disable_irq( );
        if( ( !hal_button_is_pressed( ) ) && ( smtc_rac_is_irq_flag_pending( ) == false ) && smtc_flrp_call_run( ) == false )
        {
            hal_mcu_set_sleep_for_ms( FLRP_API_INTIATOR_PERIODIC_TRANSFER>>1 );
            hal_watchdog_reload( );  // update watchdog after sleep
        }
        hal_mcu_enable_irq( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */


/* --- EOF ------------------------------------------------------------------ */
