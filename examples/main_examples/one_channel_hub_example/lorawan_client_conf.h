/**
 * @file      lorawan_client_conf.h
 *
 * @brief     LoRaWAN client configuration parser for one-channel hub
 *
 * This module handles parsing of the optional LoRaWAN client configuration
 * from a JSON file. When enabled, the one-channel hub can also act as a
 * LoRaWAN end-device to send alerts (e.g., when UDP link to LNS is broken).
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

#ifndef LORAWAN_CLIENT_CONF_H
#define LORAWAN_CLIENT_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

#include "smtc_modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Default configuration file name for LoRaWAN client
 */
#define LORAWAN_CLIENT_CONF_DEFAULT_FILE "lorawan_client.json"

/**
 * @brief Environment variable to override config file path
 */
#define LORAWAN_CLIENT_CONF_ENV_VAR "LORAWAN_CLIENT_CONF"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Default values for optional configuration fields
 */
#define LORAWAN_CLIENT_DEFAULT_KEEP_ALIVE_S 3600 /* 1 hour */
#define LORAWAN_CLIENT_DEFAULT_ALERT_PORT 102

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief LoRaWAN client configuration structure
 */
typedef struct lorawan_client_conf_s
{
    bool enabled; /**< Enable/disable the LoRaWAN client */

    /* LoRaWAN credentials */
    uint8_t dev_eui[8];  /**< Device EUI (8 bytes) */
    uint8_t join_eui[8]; /**< Join EUI / App EUI (8 bytes) */
    uint8_t app_key[16]; /**< Application Key (16 bytes) - LoRaWAN 1.0.x */

    /* Region */
    smtc_modem_region_t region; /**< LoRaWAN region */

    /* Application parameters */
    uint32_t keep_alive_interval_s; /**< Periodic status update interval in seconds */
    uint8_t  alert_port;            /**< FPort for connection status messages */

} lorawan_client_conf_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Load LoRaWAN client configuration from JSON file
 *
 * This function attempts to load and parse the LoRaWAN client configuration
 * from a JSON file. If the file doesn't exist or parsing fails, the function
 * returns false and the LoRaWAN client should not be started.
 *
 * @param[in]  conf_file  Path to the JSON configuration file (can be NULL to use default)
 * @param[out] conf       Pointer to configuration structure to fill
 *
 * @return true if configuration was loaded successfully and client is enabled
 * @return false if file doesn't exist, parsing failed, or client is disabled
 */
bool lorawan_client_conf_load( const char* conf_file, lorawan_client_conf_t* conf );

/**
 * @brief Get the configuration file path
 *
 * Returns the path to use for the configuration file, checking:
 * 1. Environment variable LORAWAN_CLIENT_CONF
 * 2. Default file name "lorawan_client.json"
 *
 * @return Pointer to the configuration file path string
 */
const char* lorawan_client_conf_get_path( void );

/**
 * @brief Print configuration to debug output
 *
 * @param[in] conf Pointer to configuration structure to print
 */
void lorawan_client_conf_print( const lorawan_client_conf_t* conf );

/**
 * @brief Convert region string to smtc_modem_region_t
 *
 * @param[in] region_str Region string (e.g., "EU868", "US915", etc.)
 *
 * @return Corresponding smtc_modem_region_t value, or SMTC_MODEM_REGION_EU_868 as default
 */
smtc_modem_region_t lorawan_client_conf_str_to_region( const char* region_str );

#ifdef __cplusplus
}
#endif

#endif /* LORAWAN_CLIENT_CONF_H */

/* --- EOF ------------------------------------------------------------------ */
