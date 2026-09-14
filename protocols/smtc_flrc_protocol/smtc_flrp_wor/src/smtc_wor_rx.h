/**
 * @file      smtc_wor_rx.h
 *
 * @brief     SMTC FLRC WOR for reception
 *
 * This header contains the prototypes for the SMTC FLRC WOR.
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

#ifndef SMTC_WOR_RX_H
#define SMTC_WOR_RX_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_wor.h"
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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Initialize the WOR RX.
 * \param [in] smtc_wor_obj     WOR structure
 * \param [in] config           WOR radio configuration
 * \param [in] dev_eui          Device EUI
 * \param [in] hook_id          Radio planner hook ID
 * \param [in] wor_done_cb      Function called when a WOR transfer is done
 *
 */
void smtc_wor_rx_init( smtc_wor_t* smtc_wor_obj, smtc_wor_static_radio_config_t config, uint8_t* dev_eui,
                       uint8_t hook_id, smtc_wor_done_f wor_done_cb );
/*!
 * \brief Start a WOR reception procedure.
 * \param [in] smtc_wor_obj     WOR structure
 * \param [in] radio_config     Dynamic radio config
 * \param [in] timestamp_ms     Timestamp of reception start in ms
 * \param [in] data_to_send     Device has data to send
 * \param [in] wor_ack          WOR ack data to send
 * \param [in] info_wor_ack     Informations to configure the WOR ack (depending on WOR)
 *
 * \return SMTC_WOR_STATUS_BUSY if a transmission or reception is already ongoing.
 * SMTC_FLRP_RC_OK in case of success.
 */
smtc_wor_status_t smtc_wor_rx_start( smtc_wor_t* smtc_wor_obj, smtc_flrp_wor_radio_config_t radio_config,
                                     uint32_t timestamp_ms, bool data_to_send, smtc_wor_ack_data_t wor_ack,
                                     smtc_wor_rx_info_for_ack_t info_wor_ack );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_WOR_RX_H

/* --- EOF ------------------------------------------------------------------ */
