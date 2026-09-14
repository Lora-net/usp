/**
 * @file      smtc_flrp_mac_serde.h
 *
 * @brief     smtc_flrp_mac serialization/deserialization implementation
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

#ifndef SMTC_FLRP_MAC_SERDE_H
#define SMTC_FLRP_MAC_SERDE_H

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
#include "smtc_rac.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#define SMTC_FLRP_MAC_DATA_PACKET_LENGTH_MAX ( RAC_FLRC_BURST_RADIO_PAYLOAD_MAX_LENGTH )
#define SMTC_FLRP_MAC_DATA_HEADER_LENGTH 19
#define SMTC_FLRP_MAC_DATA_MIC_LENGTH 4
#define SMTC_FLRP_MAC_DATA_HEADER_FOOTER_LENGTH ( SMTC_FLRP_MAC_DATA_HEADER_LENGTH + SMTC_FLRP_MAC_DATA_MIC_LENGTH )
#define SMTC_FLRP_MAC_DATA_PAYLOAD_MAX_LENGTH \
    ( SMTC_FLRP_MAC_DATA_PACKET_LENGTH_MAX - SMTC_FLRP_MAC_DATA_HEADER_FOOTER_LENGTH )

#define SMTC_FLRP_MAC_BURST_ACK_PACKET_MIN_LENGTH ( 22 )
#define SMTC_FLRP_MAC_BURST_ACK_PACKET_MAX_LENGTH ( 57 )

#define SMTC_FLRP_REQ_PACKET_LENGTH ( 24 )
#define SMTC_FLRP_ACK_PACKET_LENGTH ( 23 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum smtc_flrp_mac_serde_status_e
{
    SMTC_FLRP_MAC_SERDE_STATUS_OK = 0,
    SMTC_FLRP_MAC_SERDE_STATUS_ERROR,
} smtc_flrp_mac_serde_status_t;

typedef enum smtc_flrp_mac_frame_type_e
{
    SMTC_FLRP_MAC_FRAME_TYPE_FLRC_REQ  = 1,
    SMTC_FLRP_MAC_FRAME_TYPE_FLRC_ACK  = 2,
    SMTC_FLRP_MAC_FRAME_TYPE_BURST_ACK = 3,
    SMTC_FLRP_MAC_FRAME_TYPE_DATA      = 4,
} smtc_flrp_mac_frame_type_t;

typedef enum smtc_flrp_mac_adaptive_link_status_e
{
    SMTC_FLRP_MAC_ADAPTIVE_LINK_ACCEPTED = 0,
    SMTC_FLRP_MAC_ADAPTIVE_LINK_RETRY,
    SMTC_FLRP_MAC_ADAPTIVE_LINK_ABORT
} smtc_flrp_mac_adaptive_link_status_t;

typedef struct smtc_flrp_mac_header_s
{
    smtc_flrp_mac_frame_type_t message_type;
} smtc_flrp_mac_header_t;

typedef struct smtc_flrp_mac_data_packet_s
{
    bool    is_last_burst;                           /*!< Last burst in sequence */
    uint8_t burst_seq;                               /*!< Burst sequence */
    uint8_t packet_seq;                              /*!< Packet sequence */
    uint8_t initiator_dev_eui[SMTC_FLRP_EUI_LENGTH]; /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                    DevEUI format) - identifies the sender of this FLRC REQ
                                                 message */
    uint8_t receiver_dev_eui[SMTC_FLRP_EUI_LENGTH];  /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                     DevEUI format) - identifies the intended recipient of this FLRC REQ
                                                  message */
    uint8_t* payload;
    uint16_t payload_size;
    uint16_t payload_padding_size;
    uint32_t mic;
} smtc_flrp_mac_data_packet_t;

typedef struct smtc_flrp_mac_burst_ack_packet_s
{
    uint8_t initiator_dev_eui[SMTC_FLRP_EUI_LENGTH]; /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                    DevEUI format) - identifies the sender of this FLRC REQ
                                                 message */
    uint8_t receiver_dev_eui[SMTC_FLRP_EUI_LENGTH];  /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                     DevEUI format) - identifies the intended recipient of this FLRC REQ
                                                  message */
    bool    link_adaptation_req;                     /*!<  Adapt link parameters if true */
    uint8_t burst_seq;                               /*!< Burst sequence */
    bool    is_missing_packets;                      /*!< If packets are missing */
    uint8_t*
        missing_packets_data;          /*!< Bitmap encoding of missing packets
                                                                                          (format depends on encoding type) */
    uint8_t  missing_packets_data_len; /*!< Length of the missing_packets_data */
    uint8_t  reception_quality;        /*!< Average RSSI/SNR indicator for burst quality assessment */
    uint16_t next_window_timing_us;    /*!< Timing for next message (retry burst or next burst) */
    ral_flrc_raw_bit_rate_t
        recommended_datarate; /*!< Suggested FLRC datarate for retransmission (if link_adaptation_req is true) */
    ral_flrc_cr_t recommended_coding_rate;
    uint16_t      recommended_channel; /*!< Suggested channel for retransmission (if link_adaptation_req is true) */
} smtc_flrp_mac_burst_ack_packet_t;

typedef struct smtc_flrp_mac_flrc_req_packet_s
{
    uint8_t initiator_dev_eui[SMTC_FLRP_EUI_LENGTH]; /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                    DevEUI format) - identifies the sender of this FLRC REQ
                                                 message */
    uint8_t receiver_dev_eui[SMTC_FLRP_EUI_LENGTH];  /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                     DevEUI format) - identifies the intended recipient of this FLRC REQ
                                                  message */
    uint32_t      full_payload_size;                 /*!< Total application data size in bytes (max 16 MB)*/
    uint16_t      uniform_payload_size; /*!< Calculated by transmitter to minimize padding based on Full Payload Size */
    bool          flrc_ack_disabled;
    ral_flrc_cr_t coding_rate;
    uint16_t burst_interframe_delay_us; /*!< Minimum delay between individual data packets during burst transmission */
    uint32_t burst_ack_start_delay_us;  /*!< Delay before Burst ACK transmission */
} smtc_flrp_mac_flrc_req_packet_t;

typedef struct smtc_flrp_mac_flrc_ack_packet_s
{
    uint8_t initiator_dev_eui[SMTC_FLRP_EUI_LENGTH]; /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                    DevEUI format) - identifies the sender of this FLRC REQ
                                                 message */
    uint8_t receiver_dev_eui[SMTC_FLRP_EUI_LENGTH];  /*!<  Extended Unique Identifier of the initiator device (LoRaWAN
                                                     DevEUI format) - identifies the intended recipient of this FLRC REQ
                                                  message */
    smtc_flrp_mac_adaptive_link_status_t transfer_status;
    uint8_t                              best_channel;
    ral_flrc_raw_bit_rate_t              next_burst_data_rate;
    ral_flrc_cr_t                        selected_coding_rate;
    bool                                 multi_burst_mode;
    uint8_t                              nb_flrc_packets_per_burst;
    uint16_t                             burst_start_delay_us;
} smtc_flrp_mac_flrc_ack_packet_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

void smtc_flrp_mac_deserialize_header( uint8_t* packet, smtc_flrp_mac_header_t* mac_header );

smtc_flrp_mac_serde_status_t smtc_flrp_mac_serialize_data_packet( uint8_t* packet, uint16_t* packet_size,
                                                                  smtc_flrp_mac_data_packet_t data_packet );
smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_data_packet_header( uint8_t* packet, uint16_t packet_size,
                                                                           smtc_flrp_mac_data_packet_t* data_packet );

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_data_packet_payload( uint8_t* packet, uint16_t packet_size,
                                                                            smtc_flrp_mac_data_packet_t* data_packet );

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_data_packet_footer( uint8_t* packet, uint16_t packet_size,
                                                                           smtc_flrp_mac_data_packet_t* data_packet );

smtc_flrp_mac_serde_status_t smtc_flrp_mac_serialize_burst_ack_packet( uint8_t* packet, uint16_t* packet_size,
                                                                       smtc_flrp_mac_burst_ack_packet_t burst_ack );

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_burst_ack_packet( uint8_t* packet, uint16_t packet_size,
                                                                         smtc_flrp_mac_burst_ack_packet_t* burst_ack );

void smtc_flrp_mac_serialize_flrc_req_packet( uint8_t* packet, uint16_t* packet_size,
                                              smtc_flrp_mac_flrc_req_packet_t flrc_req );

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_flrc_req_packet( uint8_t* packet, uint16_t packet_size,
                                                                        smtc_flrp_mac_flrc_req_packet_t* flrc_req );

void smtc_flrp_mac_serialize_flrc_ack_packet( uint8_t* packet, uint16_t* packet_size,
                                              smtc_flrp_mac_flrc_ack_packet_t flrc_ack );

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_flrc_ack_packet( uint8_t* packet, uint16_t packet_size,
                                                                        smtc_flrp_mac_flrc_ack_packet_t* flrc_ack );
#ifdef __cplusplus
}
#endif

#endif  // SMTC_FLRP_MAC_LAYER_H

/* --- EOF ------------------------------------------------------------------ */
