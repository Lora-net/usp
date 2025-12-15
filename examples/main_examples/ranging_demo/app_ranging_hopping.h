/**
 * @file      app_ranging_hopping.h
 *
 * @brief     Ranging and frequency hopping for LR1110 or LR1120 chip
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

#ifndef APP_RANGING_HOPPING_H
#define APP_RANGING_HOPPING_H
#include "smtc_rac_api.h"
#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief Define min and max ranging channels count
 */
#define RANGING_HOPPING_CHANNELS_MAX 29
#define RANGING_HOPPING_CHANNELS_MIN 10
#define RANGING_CONFIG_TX_DELAY_MS 5000

/*!
 * @brief List of states for radio
 */
typedef enum app_radio_internal_states
{
    APP_RADIO_IDLE = 0,  // nothing to do (or wait a radio interrupt)
    APP_RADIO_RANGING_CONFIG,
    APP_RADIO_RANGING_START,
} app_radio_internal_states_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Ranging settings structure of structure
 */
typedef struct ranging_result_s
{
    uint32_t raw_distance;  // Raw distance value(hexadecimal)
    int32_t  distance_m;    // Distance obtained from ranging [m]
    int8_t   rssi;          // RSSI corresponding to manager-side response reception [dBm]
} ranging_result_t;
typedef struct
{
    uint8_t  rng_req_count;     // Ranging Request Count
    uint16_t rng_req_delay;     // Time between ranging request
    uint32_t rng_address;       // Ranging Address
    uint8_t  rng_status;        // Status of ranging
    uint32_t rng_exch_timeout;  // Computed global timeout based time on air of the current ranging packet
} ranging_params_settings_t;

typedef struct
{
    uint16_t         cnt_packet_rx_ok;
    ranging_result_t rng_result[RANGING_HOPPING_CHANNELS_MAX];

    uint8_t rng_per;       // Ranging PER
    int32_t rng_distance;  // Distance measured by ranging
    int8_t  rssi_value;    // RSSI Value
    int8_t  snr_value;     // SNR Value (only for LORA mode type)
} ranging_global_result_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize some parameters for ranging
 */
void app_radio_ranging_params_init( bool is_manager, smtc_rac_priority_t priority );

/*!
 * @brief Setup ranging and configure some parameters
 *
 * @param [in] context  Pointer to the radio context
 */
void app_radio_ranging_setup( const void* context );

/*!
 * @brief Set a callback to get distance
 *
 * @param [in] context  Pointer to the user_callback
 */
void app_radio_ranging_set_user_callback( void ( *results_callback_init )(
    smtc_rac_radio_lora_params_t* radio_lora_params, ranging_params_settings_t* ranging_params_settings,
    ranging_global_result_t* ranging_global_results, const char* region ) );

// void ranging_state_machine_manager( void* context );

/*!
 * @brief Compute the result distance and get the ranging results.
 *
 * @returns Pointer on the results.
 */
ranging_global_result_t* app_ranging_get_result( void );

/*!
 * @brief Get the current radio settings for ranging.
 *
 * @returns The pointer on the radio settings.
 */
ranging_params_settings_t* app_ranging_get_radio_settings( void );

/*!
 * @brief Reset the ranging parameters.
 */
void app_ranging_params_reset( void );

/*!
 * @brief Get the channel value from the predefine list
 *
 * @param [in] index  Index of array
 *
 * @returns  Frequency value
 */
uint32_t get_ranging_hopping_channels( uint8_t index );

/*!
 * @brief Set the current state for ranging process
 *
 * @param [in] state  Current state
 */
void set_ranging_process_state( uint8_t state );

void start_ranging_exchange( uint16_t delay_ms, bool );
#ifdef __cplusplus
}
#endif

#endif  // APP_RANGING_HOPPING_H

/* --- EOF ------------------------------------------------------------------ */
