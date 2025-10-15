/**
 * @file      apps_configuration.h
 *
 * @brief     Common configuration header for application-level radio and LoRa parameters.
 *
 * This file centralizes all the key configuration macros and parameters used by the application,
 * especially for LoRa and ranging operations. It provides default values for frequency, power,
 * payload length, modulation parameters, and other radio settings. These macros ensure that all
 * parts of the application use consistent radio settings and make it easy to adapt the configuration
 * for different regions, hardware, or use cases.
 *
 * Key features:
 *   - Defines the default RF frequency and TX output power.
 *   - Sets the payload length for ranging initialization.
 *   - Specifies LoRa modulation parameters: spreading factor, bandwidth, coding rate, preamble length, packet mode, IQ,
 * CRC, and sync word.
 *   - Uses include guards to prevent multiple inclusion.
 *   - Provides C++ compatibility.
 *
 * Usage:
 *   - Include this header in any application file that needs access to radio or LoRa configuration.
 *   - Override the macros before including this file if custom values are needed.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 */

#ifndef APPS_CONFIGURATION_H
#define APPS_CONFIGURATION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 * Includes standard integer types and the radio abstraction layer definitions.
 */
#include <stdint.h>
#include "ral.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 * These macros define the default configuration for the radio and LoRa modulation.
 */

/*!
 * @brief Default RF frequency in Hz for the application.
 *        Can be overridden before including this header.
 */
#ifndef RF_FREQ_IN_HZ
#define RF_FREQ_IN_HZ 868100000  // 2450000000  // Default frequency set to 2.45 GHz (common for LoRa in 2.4GHz band)
#endif

/*!
 * @brief Default TX output power in dBm.
 *        Range: [-17, +22] for sub-GHz, [-18, 13] for 2.4GHz (HF_PA).
 */
#ifndef TX_OUTPUT_POWER_DBM
#define TX_OUTPUT_POWER_DBM 14
#endif

/*!
 * @brief Payload length used for ranging initialization for LoRa type.
 */
#ifndef PAYLOAD_LENGTH
#define PAYLOAD_LENGTH 7
#endif

/*!
 * @brief LoRa modulation parameters.
 *        These define the spreading factor, bandwidth, and coding rate.
 */
#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR RAL_LORA_SF9
#endif

#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH RAL_LORA_BW_500_KHZ
#endif

#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE RAL_LORA_CR_4_5
#endif

/*!
 * @brief LoRa preamble length in symbols.
 *        Please keep preamble length as 12, as it is related to the timing of ranging process.
 */
#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 12
#endif

/*!
 * @brief LoRa packet length mode (explicit or implicit).
 */
#ifndef LORA_PKT_LEN_MODE
#define LORA_PKT_LEN_MODE RAL_LORA_PKT_EXPLICIT
#endif

/*!
 * @brief LoRa IQ inversion setting.
 *        Please keep IQ standard, as all the available calibration tables are based on this.
 */
#ifndef LORA_IQ
#define LORA_IQ false
#endif

/*!
 * @brief LoRa CRC enable/disable.
 */
#ifndef LORA_CRC
#define LORA_CRC true
#endif

/*!
 * @brief LoRa sync word.
 */
#ifndef LORA_SYNCWORD
#define LORA_SYNCWORD LORA_PRIVATE_NETWORK_SYNCWORD
#endif

#ifdef __cplusplus
}
#endif

#endif  // APPS_CONFIGURATION_H

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 * (None defined in this file, but section reserved for future use.)
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 * (None defined in this file, but section reserved for future use.)
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 * (None defined in this file, but section reserved for future use.)
 */

/* --- EOF ------------------------------------------------------------------ */
