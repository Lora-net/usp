/**
 * @file      main_one_channel_hub.h
 *
 * @brief     1-channel LoRaWAN gateway (hub) example based on RAC
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

#ifndef __MAIN_ONE_CHANNEL_HUB_H__
#define __MAIN_ONE_CHANNEL_HUB_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

#include "smtc_hal_dbg_trace.h"
#include "smtc_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Receive frequency in Hz
 *
 * Default: 868.1 MHz (EU868 common frequency)
 */
#ifndef RF_FREQ_IN_HZ
#define RF_FREQ_IN_HZ 868100000ul
#endif

/**
 * @brief LoRa spreading factor
 *
 * Default: SF7 (most commonly used for uplinks)
 */
#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR RAL_LORA_SF7
#endif

/**
 * @brief LoRa bandwidth
 *
 * Default: 125 kHz (standard LoRaWAN bandwidth)
 */
#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH RAL_LORA_BW_125_KHZ
#endif

/**
 * @brief LoRa coding rate
 *
 * Default: 4/5 (standard LoRaWAN coding rate)
 */
#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE RAL_LORA_CR_4_5
#endif

/**
 * @brief LoRa preamble length in symbols
 *
 * Default: 8 symbols (standard for LoRaWAN)
 */
#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 8
#endif

/**
 * @brief LoRa packet length mode
 *
 * Default: Explicit header (standard for LoRaWAN uplinks)
 */
#ifndef LORA_PKT_LEN_MODE
#define LORA_PKT_LEN_MODE RAL_LORA_PKT_EXPLICIT
#endif

/**
 * @brief LoRa IQ inversion
 *
 * Default: false (uplink standard - no inversion)
 */
#ifndef LORA_IQ
#define LORA_IQ false
#endif

/**
 * @brief LoRa CRC
 *
 * Default: true (CRC enabled for LoRaWAN)
 */
#ifndef LORA_CRC
#define LORA_CRC true
#endif

/**
 * @brief Maximum payload size for received packets
 */
#ifndef MAX_PAYLOAD_SIZE
#define MAX_PAYLOAD_SIZE 255
#endif

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

#endif  // __MAIN_ONE_CHANNEL_HUB_H__
