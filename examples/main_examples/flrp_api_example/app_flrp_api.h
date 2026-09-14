/**
 * @file      app_flrp_api.h
 *
 * @brief     FLRP API example definitions
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

#ifndef APP_FLRP_API_H
#define APP_FLRP_API_H

#include <stdint.h>
#include <stdbool.h>
#include "smtc_flrp_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @def FLRP_API_DISPLAY_BYTE_SIZE
 * @brief Define the byte size for display purposes
 */
#define FLRP_API_DISPLAY_BYTE_SIZE 64

/**
 * @def FLRP_API_ROLE_SLAVE
 * @brief Define the role of the device as a slave
 * @def FLRP_API_ROLE_INITIATOR
 * @brief Define the role of the device as an initiator
 */
#define FLRP_API_ROLE_SLAVE 0
#define FLRP_API_ROLE_INITIATOR 1

/**
 * @def FLRP_API_ROLE
 * @brief Define the role of the device (slave or initiator)
 */
#ifndef FLRP_API_ROLE
#define FLRP_API_ROLE FLRP_API_ROLE_SLAVE
#endif

/**
 * @def FLRP_API_DATA_SIZE
 * @brief Define the data size for the device
 */
#ifndef FLRP_API_DATA_SIZE
#define FLRP_API_DATA_SIZE ( 20 * 1024 )
#endif

/**
 * @def FLRP_API_CRYPTO_ENABLED
 * @brief Define whether cryptography is enabled for the device
 */
#ifndef FLRP_API_CRYPTO_ENABLED
#define FLRP_API_CRYPTO_ENABLED true
#endif

/**
 * @def FLRP_API_IS_LOW_FREQUENCY
 * @brief Define whether the device operates in 868MHz band or in 2.4GHz band
 * (if true the device operates in 868MHz, otherwise it operates in 2.4GHz)
 */
#ifndef FLRP_API_IS_LOW_FREQUENCY
#define FLRP_API_IS_LOW_FREQUENCY true
#endif

/**
 * @def FLRP_API_IS_LISTENING
 * @brief Define whether the device is in periodical listening mode
 */
#ifndef FLRP_API_IS_LISTENING
#define FLRP_API_IS_LISTENING true
#endif

/**
 * @def FLRP_API_INTIATOR_PERIODIC_TRANSFER
 * @brief Define the initiator periodic transfer delay in ms
 */
#ifndef FLRP_API_INTIATOR_PERIODIC_TRANSFER
#define FLRP_API_INTIATOR_PERIODIC_TRANSFER 20000
#endif

/**
 * @def FLRP_API_CRYSTAL_ERROR
 * @brief Define the crystal error for the device
 */
#ifndef FLRP_API_CRYSTAL_ERROR
#define FLRP_API_CRYSTAL_ERROR 10
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct flrp_api_message_s
{
    uint16_t counter;
    uint8_t  data[FLRP_API_DATA_SIZE];
} flrp_api_message_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

bool flrp_api_init( bool crypto_enabled, bool is_low_frequency, bool listening );
bool flrp_api_initiate_transfer( bool is_tx );
bool flrp_api_initiate_tx( uint8_t* target_device_eui );
bool flrp_api_initiate_rx( uint8_t* target_device_eui );
void user_rx_callback( const void* context, smtc_flrp_return_code_t status, uint32_t payload_size, uint8_t* src_addr,
                       smtc_flrp_rx_stats_t flrp_stats );
void user_tx_callback( const void* context, smtc_flrp_return_code_t err_code, bool send_successful,
                       uint8_t* dest_addr );
void flrp_api_setup_tx_buffer( void );

#endif  // APP_FLRP_API_H
