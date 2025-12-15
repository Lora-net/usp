/**
 * @file      app_cad.h
 *
 * @brief     CAD (Channel Activity Detection) example for LR20xx chip
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

#ifndef APP_CAD_H
#define APP_CAD_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define CAD_DELAY_MS 1000
#define CAD_DURATION_MS 1000

#if !defined( TYPE_OF_CAD )
// #define TYPE_OF_CAD RAL_LORA_CAD_LBT // perform CAD and then TX
// #define TYPE_OF_CAD RAL_LORA_CAD_RX  // perform CAD and then RX
#define TYPE_OF_CAD RAL_LORA_CAD_ONLY  // perform CAD and stop the radio
#endif                                 // !defined( TYPE_OF_CAD )

#define FREQ_IN_HZ 868100000

#define LORA_SYNCWORD 0x34
#define LORA_SPREADING_FACTOR RAL_LORA_SF12
#define LORA_BANDWIDTH RAL_LORA_BW_125_KHZ
#define LORA_CODING_RATE RAL_LORA_CR_4_5
#define LORA_IQ false
#define TX_OUTPUT_POWER_DBM 14
#define PAYLOAD_SIZE 12
#define LORA_PREAMBLE_LENGTH 255
#define LORA_CRC true
#define LORA_PKT_LEN_MODE RAL_LORA_PKT_EXPLICIT

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize CAD example
 *
 * Register direct radio access in RAC
 */
void cad_init( void );

/**
 * @brief Determine and execute CAD actions upon button press
 *
 * Typical actions are:
 * - if radio not currently in used and has no scheduled tasks: scheduled a CAD operation
 * - otherwise: print a message indicating the radio is busy
 */
void cad_on_button_press( void );

#endif  // APP_CAD_H
