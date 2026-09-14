/**
 * @file      smtc_flrp_core.h
 *
 * @brief     SMTC FLRC Core
 *
 * This header contains the prototypes for the SMTC FLRC Core.
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

#ifndef SMTC_FLRP_CORE_H
#define SMTC_FLRP_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_flrp_mac_layer.h"
#include "smtc_flrp_api.h"
#include "smtc_rac_api.h"
#include "flrp_defs.h"
#include "smtc_wor_rx.h"
#include "smtc_wor_tx.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief SMTC FLRC Core status
 */
typedef enum smtc_flrp_core_status_e
{
    SMTC_FLRP_CORE_STATUS_OK = 0,
    SMTC_FLRP_CORE_STATUS_ERROR,
} smtc_flrp_core_status_t;

/**
 * @brief SMTC FLRC Core state
 */
typedef enum smtc_flrp_core_state_e
{
    SMTC_FLRP_CORE_STATE_IDLE = 0,
    SMTC_FLRP_CORE_STATE_WOR_RX,
    SMTC_FLRP_CORE_STATE_WOR_TX,
    SMTC_FLRP_CORE_STATE_DATA,
} smtc_flrp_core_state_t;

/**
 * @brief SMTC FLRC Core wor event
 */
typedef enum smtc_flrp_core_wor_event_e
{
    SMTC_FLRP_CORE_WOR_IDLE,
    SMTC_FLRP_CORE_WOR_SUCCESS,
    SMTC_FLRP_CORE_WOR_FAILED,
    SMTC_FLRP_CORE_WOR_RX_ABORT,
} smtc_flrp_core_wor_event_t;

/**
 * @brief SMTC FLRC Core context
 */
typedef struct smtc_flrp_core_s
{
    smtc_flrp_core_state_t     state;
    smtc_flrp_core_wor_event_t wor_event;
    bit_mask_t                 wor_exchange_phase_bits;
    bool                       initialized;

    smtc_flrp_mac_layer_t smtc_flrp_mac_layer_obj;
    smtc_wor_t            smtc_wor_rx_obj;
    smtc_wor_t            smtc_wor_tx_obj;

    smtc_flrp_wor_rx_stats_t wor_rx_stats;

    uint32_t cad_period_ms;
    uint32_t radio_end_timestamp_ms;
    uint8_t  dev_eui[SMTC_FLRP_EUI_LENGTH];

    smtc_flrp_mac_radio_config_t flrc_radio_config;
    smtc_flrp_wor_radio_config_t wor_rx_radio_config;
    smtc_flrp_wor_radio_config_t wor_tx_radio_config;

    uint8_t* rx_initiator_data_buffer;
    uint32_t rx_initiator_data_buffer_size;
    uint8_t* rx_slave_data_buffer;
    uint32_t rx_slave_data_buffer_size;
    uint8_t* tx_initiator_data_buffer;
    uint32_t tx_initiator_data_buffer_size;
    uint8_t* tx_slave_data_buffer;
    uint32_t tx_slave_data_buffer_size;
    bool     data_slave_ready_to_be_sent;

    smtc_flrp_mac_layer_tx_done_f tx_done_cb;
    smtc_flrp_mac_layer_rx_done_f rx_done_cb;

    smtc_flrp_tx_done_f tx_user_callback;
    smtc_flrp_rx_done_f rx_user_callback;
    void*               user_context;

} smtc_flrp_core_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*! internal use only */
bit_mask_t* get_mask( void );

uint8_t smtc_flrp_core_get_mac_radio_access_id( void );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_FLRP_CORE_H

/* --- EOF ------------------------------------------------------------------ */
