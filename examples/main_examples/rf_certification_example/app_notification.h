/**
 * @file      app_notification.h
 *
 * @brief     RF Certification LoRa TX example for LR1110 or LR1120 chip
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

#ifndef __APP_NOTIFICATION_H__
#define __APP_NOTIFICATION_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>

#include "app_rf_certification.h"
#include "smtc_rac_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief The maximum length of the notification string
 */
#define NOTIFICATION_STRING_MAX_LEN ( 255 )

/**
 * @brief Values for the NOTIFICATION_MODE macro
 */
#define NOTIFICATIONS_ON 1
#define NOTIFICATIONS_OFF 2

/**
 * @brief Allow to enable/disable notifications at compile time
 *
 * @note compile with -DNOTIFICATION_MODE=NOTIFICATIONS_ON to enable them
 */
#ifndef NOTIFICATION_MODE
#define NOTIFICATION_MODE NOTIFICATIONS_OFF
#endif

/**
 * @brief Values for the APP_MODE macro
 */
#define APP_MODE_CERTIFICATION 1
#define APP_MODE_NOTIFICATION 2

/**
 * @brief Allow to choose the application mode at compile time
 *
 * @note compile with -DAPP_MODE=APP_MODE_NOTIFICATION to only receive notifications
 */
#ifndef APP_MODE
#define APP_MODE APP_MODE_CERTIFICATION
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#if( NOTIFICATION_MODE == NOTIFICATIONS_ON )
static const bool NOTIFICATIONS_ENABLED = true;
#elif( NOTIFICATION_MODE == NOTIFICATIONS_OFF )
static const bool NOTIFICATIONS_ENABLED = false;
#else
#error Invalid NOTIFICATION_MODE (possible values: NOTIFICATIONS_ON, NOTIFICATIONS_OFF)
#endif

#if( RF_CERT_REGION == RF_CERT_REGION_ETSI )
static const rf_certification_region_t REGION = RF_CERT_REGION_ETSI;
#elif( RF_CERT_REGION == RF_CERT_REGION_FCC )
static const rf_certification_region_t REGION = RF_CERT_REGION_FCC;
#elif( RF_CERT_REGION == RF_CERT_REGION_ARIB )
static const rf_certification_region_t REGION = RF_CERT_REGION_ARIB;
#else
#error Invalid RF_CERT_REGION (possible values: RF_CERT_REGION_ETSI, RF_CERT_REGION_FCC, RF_CERT_REGION_ARIB)
#endif

#if( APP_MODE == APP_MODE_CERTIFICATION )
static const bool RX_ONLY = false;
#elif( APP_MODE == APP_MODE_NOTIFICATION )
static const bool RX_ONLY = true;
#else
#error Invalid APP_MODE (possible values: APP_MODE_CERTIFICATION, APP_MODE_NOTIFICATION)
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DECLARATION --------------------------------------------
 */

/**
 * @brief Initialize notification system for APP_MODE_CERTIFICATION
 *
 * @param context: The RAC context of the rf_certification
 */
void notification_init_certification( smtc_rac_context_t* context );

/**
 * @brief Wrap the function that calls `smtc_rac_submit_radio_transaction`.
 *        Notify the receiver when invoked, and then call the wrapped function
 *
 * @param function: wrapped function
 *
 * @note Only relevant if APP_MODE == APP_MODE_CERTIFICATION
 * @note `notification_init_certification` must have been called before
 */
void notification_wrap( void ( *function )( void ) );

/**
 * @brief Start an infinite LoRa reception for a notification
 *
 * @param function: function to call upon receiving a notification
 */
void notification_init_rx_only( void ( *function )( char* notification ) );

#ifdef __cplusplus
}
#endif

#endif  // __APP_NOTIFICATION_H__

/* --- EOF ------------------------------------------------------------------ */
