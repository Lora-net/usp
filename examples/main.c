/**
 * @file      main.c
 *
 * @brief     main program
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
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "main.h"

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

/**
 * Modem application (must be chosen during build)
 * See @ref modem_application_t
 */
#ifndef MAKEFILE_APP
#error Please define MAKEFILE_APP
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

int main( void )
{
#if MAKEFILE_APP == PERIODICAL_UPLINK
    // This example show how to send data on an external event.
    main_periodical_uplink( );
#elif MAKEFILE_APP == HW_MODEM
    // This main is used to perform non regressions tests
    // TBD: main_hw_modem( );

#elif MAKEFILE_APP == RTTOF
    // This example show how to perform a ranging exchange (with periodic uplink)
    main_ranging_demo( );
#elif MAKEFILE_APP == PORTING_TESTS
    main_porting_tests( );
#elif MAKEFILE_APP == LCTT_CERTIF
    main_lctt_certif( );
#elif MAKEFILE_APP == MULTIPROTOCOL
    main_multiprotocol( );
#elif MAKEFILE_APP == PING_PONG
    // This example show a simple ping-pong exchange (with periodic uplink)
    main_ping_pong( );
#elif MAKEFILE_APP == PER
    // This main is used to measure a packet error rate
    main_per( );
#elif MAKEFILE_APP == PER_FSK
    // This main is used to measure a packet error rate with FSK
    main_per_fsk( );
#elif MAKEFILE_APP == PER_FLRC
    // This main is used to measure a packet error rate with FLRC
    main_per_flrc( );
#elif MAKEFILE_APP == LR_FHSS
    // This main is used to test LR-FHSS modulation
    main_lrfhss_example( );
#elif MAKEFILE_APP == SPECTRAL_SCAN
    // This main is used to perform spectral scan analysis
    main_spectral_scan( );
#elif MAKEFILE_APP == RF_CERTIFICATION
    // This main is used for RF certification with continuous LoRa transmission
    main_rf_certification( );
#elif MAKEFILE_APP == TX_CW
    // This example shows continuous transmission (TX CW) with user control
    main_tx_cw( );
#elif MAKEFILE_APP == DIRECT_DRIVER_ACCESS
    // This example shows direct radio driver access for LR1110/LR1120
    main_direct_driver_access( );
#else
#error "Unknown application" ## MAKEFILE_APP
#endif
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
