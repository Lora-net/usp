/**
 * @file      main_per_flrc.c
 *
 * @brief     Simple PER (Packet Error Rate) example with FLRC modulation
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

#include "app_per_flrc.h"

#include "smtc_hal_button.h"
#include "smtc_hal_led.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_watchdog.h"
#include "smtc_rac_api.h"

#define RAC_LOG_APP_PREFIX "MAIN-PER-FLRC"
#include "smtc_rac_log.h"

static const uint32_t SLEEP_DELAY = 1000;  // ms

int main( void )
{
    hal_mcu_init( );

    // Initialize LEDs and button
    hal_led_init( );
    hal_button_init( NULL, NULL );

    smtc_rac_init( );

    per_flrc_init( );

    while( true )
    {
        hal_watchdog_reload( );
        smtc_rac_run_engine( );

        if( hal_button_is_pressed( ) )
        {
            hal_button_clear( );
            RAC_LOG_INFO( "button pressed\n" );
            per_flrc_on_button_press( );
        }

        hal_mcu_disable_irq( );
        if( ( !hal_button_is_pressed( ) ) && ( smtc_rac_is_irq_flag_pending( ) == false ) )
        {
            hal_mcu_set_sleep_for_ms( SLEEP_DELAY );
            hal_watchdog_reload( );
        }
        hal_mcu_enable_irq( );
    }
}
