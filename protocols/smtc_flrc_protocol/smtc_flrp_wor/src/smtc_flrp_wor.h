/**
 * @file      smtc_flrp_wor.h
 *
 * @brief     SMTC FLRC WOR
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

#ifndef SMTC_FLRP_WOR_H
#define SMTC_FLRP_WOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "flrp_defs.h"
#include "../smtc_flrp_api/smtc_flrp_api.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#define SMTC_FLRP_WOR_PAYLOAD_SIZE 25
#define SMTC_FLRP_WOR_ACK_PAYLOAD_MIN_SIZE 12
#define SMTC_FLRP_WOR_ACK_PAYLOAD_MAX_SIZE 13
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief SMTC FLRC WOR ACK status
 */
typedef enum smtc_flrp_wor_ack_status_e
{
    SMTC_FLRP_WOR_ACCEPTED = 0,
    SMTC_FLRP_WOR_DECLINED_LINK_BUDGET_TOO_LOW,
    SMTC_FLRP_WOR_DECLINED_NO_DATA_TO_TRANSFER,
    SMTC_FLRP_WOR_DECLINED_BUSY_OR_OTHER
} smtc_flrp_wor_ack_status_t;

typedef struct smtc_flrp_wor_data_s
{
    uint8_t initiator_dev_eui[SMTC_FLRP_EUI_LENGTH]; /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                    DevEUI format) - identifies who is initiating the communication */
    uint8_t slave_dev_eui[SMTC_FLRP_EUI_LENGTH]; /*!< Extended Unique Identifier of the target slave device for wake-up
                                                        and communication (LoRaWAN DevEUI format) */
    bool                             initiator_send_burst; /*!<  transfer_direction */
    bool                             wor_ack_required;
    bool                             burst_ack_required;
    smtc_flrp_link_adaptation_mode_t link_adaptation_mode;
    uint8_t                 filter_len;       /*!< Number of Slave DevEUI MSB bits to be used for selectivity (0-63) */
    uint16_t                enabled_channels; /*!< 1 bit per channel */
    ral_flrc_raw_bit_rate_t default_flrc_dr;
    uint8_t                 default_channel;
    ral_flrc_cr_t           default_coding_rate;
    uint32_t                next_phase_start_delay_us; /*!< When WOR ACK Disable = 0: Delay until WOR ACK transmission.
             When WOR ACK Disable = 1: Delay until FLRC REQ procedure starts.
             Reference: Delay starts at WOR message Tx Done IRQ. */
    uint16_t channel_interframe_delay_us;              /*!< Minimum delay between FLRC REQ messages (and FLRC ACK
             messages) when transmitted on different channels.
             Reference: Delay starts at previous message Tx Done IRQ.*/
} smtc_flrp_wor_data_t;

typedef struct smtc_flrp_wor_ack_data_s
{
    uint8_t receiver_dev_eui[SMTC_FLRP_EUI_LENGTH]; /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                    DevEUI format) - identifies who has initiated the communication */
    smtc_flrp_wor_ack_status_t ack_status;
    ral_flrc_raw_bit_rate_t    selected_flrc_dr;
    ral_flrc_cr_t              selected_coding_rate;
    uint16_t                   next_phase_start_delay_us; /*!< Time until next phase starts.
                            If Link Adaptation enabled: time until FLRC REQ.
                            If Link Adaptation disabled: time until BURST.*/
    bool     has_min_interframe_delay;
    uint16_t min_interframe_delay_us;  // Only if slave receives from initiator
} smtc_flrp_wor_ack_data_t;

typedef struct smtc_flrp_wor_rx_info_for_ack_s
{
    uint16_t delay_until_flrc_req_us;
    uint16_t delay_until_burst_us;

} smtc_flrp_wor_rx_info_for_ack_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

void smtc_flrp_wor_serialize_payload( smtc_flrp_wor_data_t wor_data, uint8_t* payload, uint16_t* payload_size );
void smtc_flrp_wor_ack_serialize_payload( smtc_flrp_wor_ack_data_t wor_ack_data, uint8_t* payload,
                                          uint16_t* payload_size );

bool smtc_flrp_wor_deserialize_payload( smtc_flrp_wor_data_t* wor_data, uint8_t* payload, uint16_t payload_size );
bool smtc_flrp_wor_ack_deserialize_payload( smtc_flrp_wor_ack_data_t* wor_ack_data, uint8_t* payload,
                                            uint16_t payload_size );

/*!
 * \brief Handle the data received in the WOR and prepare the WOR ack data if requested.
 * \param [in] wor_data     Data of the WOR received
 * \param [in] data_result   WOR data results
 * \param [in] data_to_send  If data id ready to be sent
 * \param [out] ack_to_send  If WOR ack is requested
 * \param [out] wor_ack_data Pointer to the data of the WOR ack to send
 * \param [in] info_wor_ack  Informations to configure the WOR ack (depending on WOR)
 */
void smtc_flrp_wor_handle_payload( smtc_flrp_wor_data_t wor_data, smtc_rac_data_result_t data_result, bool data_to_send,
                                   bool* ack_to_send, smtc_flrp_wor_ack_data_t* wor_ack_data,
                                   smtc_flrp_wor_rx_info_for_ack_t info_wor_ack );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_FLRP_WOR_H

/* --- EOF ------------------------------------------------------------------ */
