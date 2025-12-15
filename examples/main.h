/**
 * @file      main.h
 *
 * @brief     main program definitions
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

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Application examples
 */
#define UNDEFINED_APP 0
#define PERIODICAL_UPLINK 1
#define HW_MODEM 2
#define PORTING_TESTS 3
#define LCTT_CERTIF 4
#define MULTIPROTOCOL 5
#define RTTOF 6
#define PING_PONG 7
#define PER 8
#define PER_FSK 9
#define LR_FHSS 10
#define SPECTRAL_SCAN 11
#define RF_CERTIFICATION 12
#define PER_FLRC 13
#define TX_CW 14
#define DIRECT_DRIVER_ACCESS 15
#define CAD 16
#define GEOLOCATION 17
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

void main_periodical_uplink( void );
void main_hw_modem( void );
void main_geolocation( void );
void main_porting_tests( void );
void main_lctt_certif( void );
void main_multiprotocol( void );
void main_ranging_demo( void );
void main_ping_pong( void );
void main_per( void );
void main_per_fsk( void );
void main_per_flrc( void );
int  main_lrfhss_example( void );
int  main_spectral_scan( void );
void main_rf_certification( void );
void main_tx_cw( void );
void main_direct_driver_access( void );
void main_cad( void );
#ifdef __cplusplus
}
#endif

#endif  // MAIN_H

/* --- EOF ------------------------------------------------------------------ */
