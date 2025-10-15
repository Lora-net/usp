/**
 * @file      app_direct_driver_access.h
 *
 * @brief     Direct radio driver access example for LR20xx chip
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

#ifndef APP_DIRECT_DRIVER_ACCESS_H
#define APP_DIRECT_DRIVER_ACCESS_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define DIRECT_DRIVER_ACCESS_DELAY_MS 100
#define DIRECT_DRIVER_ACCESS_DURATION_MS 2000

#define FREQ_IN_HZ 868100000
#define OUT_POWER_IN_DBM 14
#define LORA_PAYLOAD_LENGTH 51
#define LORA_PREAMBLE_LENGTH 12
#define LORA_SYNCWORD 0x34
#define LORA_PKT_LEN_MODE LR20XX_RADIO_LORA_PKT_EXPLICIT
#define LORA_CRC LR20XX_RADIO_LORA_CRC_ENABLED
#define LORA_IQ LR20XX_RADIO_LORA_IQ_STANDARD

#define LORA_SF LR20XX_RADIO_LORA_SF12;
#define LORA_BW LR20XX_RADIO_LORA_BW_125;
#define LORA_CR LR20XX_RADIO_LORA_CR_4_5;
#define LORA_PPM LR20XX_RADIO_LORA_NO_PPM;
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
 * @brief Initialize direct driver access example
 *
 * Register direct radio access in RAC
 */
void direct_driver_access_init( void );

/**
 * @brief Determine and execute direct radio actions upon button press
 *
 * Typical actions are:
 * - if radio not currently in used and has no scheduled tasks: scheduled a LoRa Tx
 * - otherwise: print a message indicating the radio is busy
 */
void direct_driver_access_on_button_press( void );

#endif  // APP_DIRECT_DRIVER_ACCESS_H
