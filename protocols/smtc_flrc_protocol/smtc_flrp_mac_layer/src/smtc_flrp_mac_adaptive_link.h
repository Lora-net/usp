/**
 * @file      smtc_flrp_mac_adaptive_link.h
 *
 * @brief     smtc_flrp_mac adaptive link implementation
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

#ifndef SMTC_FLRP_MAC_ADAPTIVE_LINK_H
#define SMTC_FLRP_MAC_ADAPTIVE_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_rac_api.h"
#include "smtc_flrp_api.h"
#include "smtc_flrp_mac_serde.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define MAC_INVALID_RSSI INT16_MIN

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
typedef struct smtc_flrp_mac_burst_info_in_adaptive_link_s
{
    bool                    multi_burst_mode;
    uint32_t                burst_ack_start_delay_ms;
    uint32_t                first_burst_start_delay_ms;
    ral_flrc_raw_bit_rate_t raw_bit_rate;
    ral_flrc_cr_t           coding_rate;
    uint32_t                frequency_hz;
    uint16_t                interframe_delay_us;
    uint32_t                payload_length;        /*!< Length of the full payload */
    uint16_t                packet_uniformed_size; /*!< Uniformed size of packets in the burst*/
    uint16_t                nb_packets_burst_max;  /*!< Number of packets max in a burst*/

} smtc_flrp_mac_burst_info_in_adaptive_link_t;
typedef struct smtc_flrp_mac_adaptive_link_s
{
    smtc_flrp_link_adaptation_mode_t     mode;
    uint32_t                             frequencies_to_test_hz[SMTC_FLRP_NB_CHANNELS_MAX];
    uint8_t                              frequencies_to_test_size;
    uint8_t                              frequency_test_idx;
    uint16_t                             channel_interframe_delay_ms;
    smtc_flrp_mac_adaptive_link_status_t last_flrc_ack_transfer_status;
    bool                                 retry_enabled;
    bool                                 flrc_ack_enabled;
    int16_t                              flrc_req_rssi[SMTC_FLRP_NB_CHANNELS_MAX];
} smtc_flrp_mac_adaptive_link_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Check if a FLRC REQ frame was received
 *
 * \param [in] flrc_req_rssi    RSSIs of the last flrc req packets if received
 * \param [in] nb_flrc_packets  Number of flrc req packets
 *
 * \return true is at least one packet was received, false if no flrc req was received.
 */
bool smtc_flrp_mac_at_least_one_flrc_req_was_received( int16_t* flrc_req_rssi, uint8_t nb_flrc_packets );

/*!
 * \brief Prepare the context to receive a FLRC Req frame.
 *
 * \param [inout] transaction       RAC context
 * \param [in] mac_radio_config     FLRC radio configuration
 * \param [in] rx_buffer            Pointer to the buffer receiving the data
 * \param [in] freq                 Frequency used for this frame (in Hz)
 * \param [in] freq_offset          Frequency offset (in Hz)
 */
void smtc_flrp_mac_prepare_rx_flrc_req( smtc_rac_context_t* transaction, smtc_flrp_mac_radio_config_t* mac_radio_config,
                                        uint8_t* rx_buffer, uint32_t freq, int32_t freq_offset );

/*!
 * \brief Prepare the context to receive a FLRC ACK frame.
 *
 * \param [inout] transaction       RAC context
 * \param [in] mac_radio_config     FLRC radio configuration
 * \param [in] rx_buffer            Pointer to the buffer receiving the data
 * \param [in] freq                 Frequency used for this frame (in Hz)
 * \param [in] freq_offset          Frequency offset (in Hz)
 */
void smtc_flrp_mac_prepare_rx_flrc_ack( smtc_rac_context_t* transaction, smtc_flrp_mac_radio_config_t* mac_radio_config,
                                        uint8_t* rx_buffer, uint32_t freq, int32_t freq_offset );

/*!
 * \brief Prepare the context to transmit a FLRC Req frame.
 *
 * \param [inout] transaction       RAC context
 * \param [in] mac_radio_config     FLRC radio configuration
 * \param [in] freq                 Frequency used for this frame (in Hz)
 * \param [in] freq_offset          Frequency offset (in Hz)
 * \param [in] dev_eui              Dev EUI of the this device
 * \param [in] dest_dev_eui         Dev EUI of the destination device
 * \param [in] burst_info           Information of the burst to send in the FLRC Req frame
 * \param [in] adaptive_link_config Strcture of the configuration of this adaptive link phase.
 * \param [in] tx_buffer            Pointer to the buffer of the data to transmit
 */
void smtc_flrp_mac_prepare_tx_flrc_req( smtc_rac_context_t* transaction, smtc_flrp_mac_radio_config_t* mac_radio_config,
                                        uint32_t freq, int32_t freq_offset, uint8_t* dev_eui, uint8_t* dest_dev_eui,
                                        smtc_flrp_mac_burst_info_in_adaptive_link_t burst_info,
                                        smtc_flrp_mac_adaptive_link_t adaptive_link_config, uint8_t* tx_buffer );

/*!
 * \brief Prepare the context to transmit a FLRC ACK frame.
 *
 * \param [inout] transaction           RAC context
 * \param [in] mac_radio_config         FLRC radio configuration
 * \param [in] freq                     Frequency used for this frame (in Hz)
 * \param [in] freq_offset              Frequency offset (in Hz)
 * \param [in] dev_eui                  Dev EUI of the this device
 * \param [in] dest_dev_eui             Dev EUI of the destination device
 * \param [inout] adaptive_link_config  Strcture of the configuration of this adaptive link phase.
 * \param [in] burst_info               Information of the burst to send in the FLRC Req frame
 * \param [in] tx_buffer                Pointer to the buffer of the data to transmit
 */
void smtc_flrp_mac_prepare_tx_flrc_ack( smtc_rac_context_t* transaction, smtc_flrp_mac_radio_config_t* mac_radio_config,
                                        uint32_t freq, int32_t freq_offset, uint8_t* dev_eui, uint8_t* dest_dev_eui,
                                        smtc_flrp_mac_adaptive_link_t*               adaptive_link_config,
                                        smtc_flrp_mac_burst_info_in_adaptive_link_t* burst_info, uint8_t* tx_buffer );

/*!
 * \brief Handle the FLRC Req frame received.
 *
 * \param [in] rx_packet                Packet received
 * \param [in] rx_size                  Size of the packet received
 * \param [in] rssi                     RSSI of the packet received
 * \param [in] dev_eui                  Dev EUI of this device
 * \param [in] dev_eui_filter_len       Length of the Dev EUI filter
 * \param [in] expected_src_dev_eui     Dev EUI of the expected initiator device
 * \param [inout] adaptive_link_config  Strcture of the configuration of this adaptive link phase.
 * \param [in] burst_info               Information of the burst to send in the FLRC Req frame
 *
 * \return true is the frame is valid and for this device, false otherwise.
 */
bool smtc_flrp_mac_handle_flrc_req_packet( uint8_t* rx_packet, uint32_t rx_size, int32_t rssi, uint8_t* dev_eui,
                                           uint8_t dev_eui_filter_len, uint8_t* expected_src_dev_eui,
                                           smtc_flrp_mac_adaptive_link_t*               adaptive_link_config,
                                           smtc_flrp_mac_burst_info_in_adaptive_link_t* burst_info );

/*!
 * \brief Handle the FLRC ACK frame received.
 *
 * \param [in] rx_packet                Packet received
 * \param [in] rx_size                  Size of the packet received
 * \param [in] dev_eui                  Dev EUI of this device
 * \param [in] expected_src_dev_eui     Dev EUI of the expected slave device
 * \param [inout] adaptive_link_config  Strcture of the configuration of this adaptive link phase.
 * \param [in] burst_info               Information of the burst to send in the FLRC Req frame
 * \param [in] mac_radio_config         FLRC radio configuration
 *
 * \return true is the frame is valid and for this device, false otherwise.
 */
bool smtc_flrp_mac_handle_flrc_ack_packet( uint8_t* rx_packet, uint32_t rx_size, uint8_t* dev_eui,
                                           uint8_t*                                     expected_src_dev_eui,
                                           smtc_flrp_mac_adaptive_link_t*               adaptive_link_config,
                                           smtc_flrp_mac_burst_info_in_adaptive_link_t* burst_info,
                                           smtc_flrp_mac_radio_config_t*                mac_radio_config );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_FLRP_MAC_LAYER_H

/* --- EOF ------------------------------------------------------------------ */
