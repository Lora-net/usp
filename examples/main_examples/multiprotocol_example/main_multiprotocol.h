/**
 * @file      main_multiprotocol.h
 *
 * @brief     Multiprotocol example definitions
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

#ifndef MAIN_MULTIPROTOCOL_H
#define MAIN_MULTIPROTOCOL_H

#include <stdint.h>

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
 * @def KEEP_ALIVE_PORT
 * @brief Define the port used for keep-alive messages
 */
#ifndef KEEP_ALIVE_PORT
#define KEEP_ALIVE_PORT 101
#endif

/**
 * @def RANGING_UPLINK_PORT
 * @brief Define the port used for ranging uplink messages
 */
#ifndef RANGING_UPLINK_PORT
#define RANGING_UPLINK_PORT 102
#endif

/**
 * @def RANGING_UPLINK_MAX_RATE
 * @brief Define the maximum rate for ranging uplink messages (in ms)
 */
#ifndef RANGING_UPLINK_MAX_RATE
#define RANGING_UPLINK_MAX_RATE 60000
#endif

/**
 * @def PERIODICAL_UPLINK_DELAY_S
 * @brief Define the delay between two consecutive periodical uplink messages (in seconds) triggered by alarm
 */
#ifndef PERIODICAL_UPLINK_DELAY_S
#define PERIODICAL_UPLINK_DELAY_S 600
#endif

/**
 * @def DELAY_FIRST_MSG_AFTER_JOIN
 * @brief Define the delay before sending the first message after a join (in seconds)
 */
#ifndef DELAY_FIRST_MSG_AFTER_JOIN
#define DELAY_FIRST_MSG_AFTER_JOIN 60
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
/**
 * @struct multiprotocol_uplink_t
 * @brief Structure for multiprotocol uplink messages with ranging test result
 */
typedef struct __packed multiprotocol_uplink_s
{
    uint16_t distance; /**< Distance in meters */
    uint8_t  sf;       /**< Spreading factor */
    uint8_t  bw;       /**< Bandwidth */
} multiprotocol_uplink_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#endif  // MAIN_MULTIPROTOCOL_H
