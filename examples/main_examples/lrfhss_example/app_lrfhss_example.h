/**
 * @file      app_lrfhss_example.h
 *
 * @brief     LR-FHSS application header
 *
 * This example demonstrates how to use LR-FHSS modulation with the USP/RAC framework.
 * LR-FHSS is transmission-only, so this example only shows transmission functionality.
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
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __APP_LRFHSS_EXAMPLE_H__
#define __APP_LRFHSS_EXAMPLE_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Returns the minimum value between a and b
 *
 * @param [in] a 1st value
 * @param [in] b 2nd value
 * @retval Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define LRFHSS_FREQUENCY_HZ 866500000  // 866.5 MHz
#define LRFHSS_TX_POWER_DBM 14         // 14 dBm
#define LRFHSS_PAYLOAD_SIZE 50         // 50 bytes payload
#define LRFHSS_TX_INTERVAL_MS 5000     // 5 seconds between transmissions

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize the LR-FHSS application
 *
 * This function initializes the LR-FHSS state machine and prepares
 * the radio for transmission operations.
 */
void lrfhss_example_init( void );

/*!
 * @brief Process LR-FHSS state machine
 *
 * This function should be called periodically from the main loop to
 * advance the LR-FHSS state machine and perform transmission operations.
 *
 * @return true if the application is still running, false if an error occurred
 */
bool lrfhss_example_process( void );

/*!
 * @brief Start a new LR-FHSS transmission cycle
 *
 * This function triggers the start of a new complete LR-FHSS transmission cycle.
 * Can be called when a transmission cycle should be started immediately (e.g., on button press).
 */
void lrfhss_example_start_cycle( void );

/*!
 * @brief Stop the current LR-FHSS transmission
 *
 * This function stops the current LR-FHSS transmission operation and puts the
 * application in idle state.
 */
void lrfhss_example_stop( void );

/*!
 * @brief Check if LR-FHSS transmission is currently active
 *
 * @return true if a transmission is currently in progress, false otherwise
 */
bool lrfhss_example_is_active( void );

/*!
 * @brief Get the current transmission count
 *
 * @return Current number of transmissions completed
 */
uint32_t lrfhss_example_get_tx_count( void );

#endif  // __APP_LRFHSS_EXAMPLE_H__
