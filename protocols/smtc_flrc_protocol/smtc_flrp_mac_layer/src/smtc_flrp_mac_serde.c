/**
 * @file      smtc_flrp_mac_serde.c
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "smtc_flrp_mac_serde.h"
#include "smtc_flrp_utils.h"
#include "smtc_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define FLRC_MESSAGE_TYPE_MASK 0x0F

#define BURST_LAST_BURST_MASK 0x10
#define BURST_LAST_BURST_SHIFT 4

#define BURST_ACK_LINK_ADAPTATION_REQ_MASK 0x10
#define BURST_ACK_LINK_ADAPTATION_REQ_SHIFT 4
#define BURST_ACK_MISSING_PACKETS_MASK 0x3
#define BURST_ACK_MISSING_PACKETS_SHIFT 0
#define BURST_ACK_MISSING_DATA_LEN_MASK 0xFC
#define BURST_ACK_MISSING_DATA_LEN_SHIFT 2
#define BURST_ACK_DATA_RATE_MASK 0x0F
#define BURST_ACK_DATA_RATE_SHIFT 0
#define BURST_ACK_CODING_RATE_MASK 0x30
#define BURST_ACK_CODING_RATE_SHIFT 4

#define FLRC_REQ_UNIFORM_PAYLOAD_SIZE_MASK 0x1FF
#define FLRC_REQ_UNIFORM_PAYLOAD_SIZE_SHIFT 0
#define FLRC_REQ_FLRC_ACK_DISABLE_MASK 0x200
#define FLRC_REQ_FLRC_ACK_DISABLE_SHIFT 9
#define FLRC_REQ_CODING_RATE_MASK 0xC00
#define FLRC_REQ_CODING_RATE_SHIFT 10

#define FLRC_ACK_DATA_RATE_MASK 0x0F
#define FLRC_ACK_DATA_RATE_SHIFT 0
#define FLRC_ACK_CODING_RATE_MASK 0x30
#define FLRC_ACK_CODING_RATE_SHIFT 4

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_flrp_mac_deserialize_header( uint8_t* packet, smtc_flrp_mac_header_t* mac_header )
{
    uint8_t byte_value;
    DESERIALIZE_8BITS( packet, byte_value );

    mac_header->message_type = ( smtc_flrp_mac_frame_type_t ) ( byte_value & FLRC_MESSAGE_TYPE_MASK );
}

smtc_flrp_mac_serde_status_t smtc_flrp_mac_serialize_data_packet( uint8_t* packet, uint16_t* packet_size,
                                                                  smtc_flrp_mac_data_packet_t data_packet )
{
    int packet_start_addr = ( intptr_t ) packet;

    uint8_t byte_value = SMTC_FLRP_MAC_FRAME_TYPE_DATA;
    byte_value |= ( !data_packet.is_last_burst << BURST_LAST_BURST_SHIFT ) & BURST_LAST_BURST_MASK;
    SERIALIZE_8BITS( packet, byte_value );

    SERIALIZE_8BITS( packet, data_packet.burst_seq );
    SERIALIZE_8BITS( packet, data_packet.packet_seq );

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( packet, data_packet.receiver_dev_eui[i] );
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( packet, data_packet.initiator_dev_eui[i] );
    }

    *packet_size = ( ( intptr_t ) packet - packet_start_addr );

    if( *packet_size + SMTC_FLRP_MAC_DATA_MIC_LENGTH + data_packet.payload_size > SMTC_FLRP_MAC_DATA_PACKET_LENGTH_MAX )
    {
        return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
    }

    for( uint16_t i = 0; i < data_packet.payload_size; i++ )
    {
        SERIALIZE_8BITS( packet, data_packet.payload[i] );
    }

    SERIALIZE_32BITS( packet, data_packet.mic );

    // Padding
    uint8_t byte_padding = 0;
    for( uint16_t i = 0; i < data_packet.payload_padding_size; i++ )
    {
        SERIALIZE_8BITS( packet, byte_padding );
    }

    *packet_size = ( ( intptr_t ) packet - packet_start_addr );

    return SMTC_FLRP_MAC_SERDE_STATUS_OK;
}

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_data_packet_header( uint8_t* packet, uint16_t packet_size,
                                                                           smtc_flrp_mac_data_packet_t* data_packet )
{
    if( packet_size < SMTC_FLRP_MAC_DATA_HEADER_LENGTH )
    {
        return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
    }

    uint8_t byte_value;
    DESERIALIZE_8BITS( packet, byte_value );
    data_packet->is_last_burst = !( ( byte_value & BURST_LAST_BURST_MASK ) >> BURST_LAST_BURST_SHIFT );

    DESERIALIZE_8BITS( packet, data_packet->burst_seq );
    DESERIALIZE_8BITS( packet, data_packet->packet_seq );

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( packet, data_packet->receiver_dev_eui[i] );
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( packet, data_packet->initiator_dev_eui[i] );
    }

    return SMTC_FLRP_MAC_SERDE_STATUS_OK;
}

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_data_packet_payload( uint8_t* packet, uint16_t packet_size,
                                                                            smtc_flrp_mac_data_packet_t* data_packet )
{
    if( packet_size < data_packet->payload_size )
    {
        return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
    }

    for( uint16_t i = 0; i < data_packet->payload_size; i++ )
    {
        DESERIALIZE_8BITS( packet, data_packet->payload[i] );
    }

    DESERIALIZE_32BITS( packet, data_packet->mic );

    return SMTC_FLRP_MAC_SERDE_STATUS_OK;
}

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_data_packet_footer( uint8_t* packet, uint16_t packet_size,
                                                                           smtc_flrp_mac_data_packet_t* data_packet )
{
    if( packet_size < SMTC_FLRP_MAC_DATA_MIC_LENGTH )
    {
        return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
    }
    DESERIALIZE_32BITS( packet, data_packet->mic );

    return SMTC_FLRP_MAC_SERDE_STATUS_OK;
}

smtc_flrp_mac_serde_status_t smtc_flrp_mac_serialize_burst_ack_packet( uint8_t* packet, uint16_t* packet_size,
                                                                       smtc_flrp_mac_burst_ack_packet_t burst_ack )
{
    int packet_start_addr = ( intptr_t ) packet;

    uint8_t type = SMTC_FLRP_MAC_FRAME_TYPE_BURST_ACK;
    SERIALIZE_8BITS( packet, type );

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( packet, burst_ack.initiator_dev_eui[i] );
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( packet, burst_ack.receiver_dev_eui[i] );
    }

    uint8_t byte_value = SMTC_FLRP_MAC_FRAME_TYPE_BURST_ACK;
    byte_value |=
        ( burst_ack.link_adaptation_req << BURST_ACK_LINK_ADAPTATION_REQ_SHIFT ) & BURST_ACK_LINK_ADAPTATION_REQ_MASK;
    SERIALIZE_8BITS( packet, byte_value );

    SERIALIZE_8BITS( packet, burst_ack.burst_seq );

    byte_value = ( ( uint8_t ) ( burst_ack.is_missing_packets ? 0x02 : 0 ) << BURST_ACK_MISSING_PACKETS_SHIFT ) &
                 BURST_ACK_MISSING_PACKETS_MASK;
    byte_value |=
        ( burst_ack.missing_packets_data_len << BURST_ACK_MISSING_DATA_LEN_SHIFT ) & BURST_ACK_MISSING_DATA_LEN_MASK;
    SERIALIZE_8BITS( packet, byte_value );

    *packet_size = ( ( intptr_t ) packet - packet_start_addr );

    if( burst_ack.is_missing_packets )
    {
        if( SMTC_FLRP_MAC_BURST_ACK_PACKET_MIN_LENGTH + burst_ack.missing_packets_data_len >
            SMTC_FLRP_MAC_BURST_ACK_PACKET_MAX_LENGTH )
        {
            return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
        }

        for( uint16_t i = 0; i < burst_ack.missing_packets_data_len; i++ )
        {
            SERIALIZE_8BITS( packet, burst_ack.missing_packets_data[i] );
        }
    }

    SERIALIZE_8BITS( packet, burst_ack.reception_quality );

    uint8_t next_window_timing_100us = ( uint8_t ) ( burst_ack.next_window_timing_us / 100 );
    SERIALIZE_8BITS( packet, next_window_timing_100us );

    if( burst_ack.link_adaptation_req )
    {
        byte_value =
            ( ( ( uint8_t ) burst_ack.recommended_datarate ) << BURST_ACK_DATA_RATE_SHIFT ) & BURST_ACK_DATA_RATE_MASK;
        byte_value |= ( ( ( uint8_t ) burst_ack.recommended_coding_rate ) << BURST_ACK_CODING_RATE_SHIFT ) &
                      BURST_ACK_CODING_RATE_MASK;
        SERIALIZE_8BITS( packet, byte_value );

        SERIALIZE_8BITS( packet, burst_ack.recommended_channel );

        // RFU
        packet++;
    }

    *packet_size = ( ( intptr_t ) packet - packet_start_addr );

    return SMTC_FLRP_MAC_SERDE_STATUS_OK;
}

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_burst_ack_packet( uint8_t* packet, uint16_t packet_size,
                                                                         smtc_flrp_mac_burst_ack_packet_t* burst_ack )
{
    uint8_t byte_value;

    if( packet_size < SMTC_FLRP_MAC_BURST_ACK_PACKET_MIN_LENGTH )
    {
        return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
    }

    // Ignore message type
    packet++;

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( packet, burst_ack->initiator_dev_eui[i] );
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( packet, burst_ack->receiver_dev_eui[i] );
    }

    DESERIALIZE_8BITS( packet, byte_value );
    burst_ack->link_adaptation_req =
        ( byte_value & BURST_ACK_LINK_ADAPTATION_REQ_MASK ) >> BURST_ACK_LINK_ADAPTATION_REQ_SHIFT;

    DESERIALIZE_8BITS( packet, burst_ack->burst_seq );

    DESERIALIZE_8BITS( packet, byte_value );
    burst_ack->is_missing_packets =
        ( ( byte_value & BURST_ACK_MISSING_PACKETS_MASK ) >> BURST_ACK_MISSING_PACKETS_SHIFT ) == 0x02;
    burst_ack->missing_packets_data_len =
        ( byte_value & BURST_ACK_MISSING_DATA_LEN_MASK ) >> BURST_ACK_MISSING_DATA_LEN_SHIFT;

    if( packet_size < burst_ack->missing_packets_data_len + SMTC_FLRP_MAC_BURST_ACK_PACKET_MIN_LENGTH )
    {
        return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
    }

    if( burst_ack->is_missing_packets )
    {
        for( uint16_t i = 0; i < burst_ack->missing_packets_data_len; i++ )
        {
            DESERIALIZE_8BITS( packet, burst_ack->missing_packets_data[i] );
        }
    }

    DESERIALIZE_8BITS( packet, burst_ack->reception_quality );

    uint8_t next_window_timing_100us;
    DESERIALIZE_8BITS( packet, next_window_timing_100us );
    burst_ack->next_window_timing_us = ( uint16_t ) next_window_timing_100us * 100;

    if( burst_ack->link_adaptation_req )
    {
        if( packet_size < burst_ack->missing_packets_data_len + SMTC_FLRP_MAC_BURST_ACK_PACKET_MIN_LENGTH + 2 )
        {
            return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
        }

        DESERIALIZE_8BITS( packet, byte_value );
        burst_ack->recommended_datarate =
            ( ral_flrc_raw_bit_rate_t ) ( ( byte_value & BURST_ACK_DATA_RATE_MASK ) >> BURST_ACK_DATA_RATE_SHIFT );
        burst_ack->recommended_coding_rate =
            ( ral_flrc_cr_t ) ( byte_value & BURST_ACK_CODING_RATE_MASK ) >> BURST_ACK_CODING_RATE_SHIFT;

        DESERIALIZE_8BITS( packet, burst_ack->recommended_channel );
    }

    return SMTC_FLRP_MAC_SERDE_STATUS_OK;
}

void smtc_flrp_mac_serialize_flrc_req_packet( uint8_t* packet, uint16_t* packet_size,
                                              smtc_flrp_mac_flrc_req_packet_t flrc_req )
{
    int packet_start_addr = ( intptr_t ) packet;

    uint8_t type = SMTC_FLRP_MAC_FRAME_TYPE_FLRC_REQ;
    SERIALIZE_8BITS( packet, type );

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( packet, flrc_req.initiator_dev_eui[i] );
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( packet, flrc_req.receiver_dev_eui[i] );
    }

    SERIALIZE_24BITS( packet, flrc_req.full_payload_size );

    uint16_t byte_value = ( ( flrc_req.uniform_payload_size ) << FLRC_REQ_UNIFORM_PAYLOAD_SIZE_SHIFT ) &
                          FLRC_REQ_UNIFORM_PAYLOAD_SIZE_MASK;
    byte_value |=
        ( ( flrc_req.flrc_ack_disabled ) << FLRC_REQ_FLRC_ACK_DISABLE_SHIFT ) & FLRC_REQ_FLRC_ACK_DISABLE_MASK;
    byte_value |= ( ( flrc_req.coding_rate ) << FLRC_REQ_CODING_RATE_SHIFT ) & FLRC_REQ_CODING_RATE_MASK;
    SERIALIZE_16BITS( packet, byte_value );

    uint8_t burst_interframe_delay_100us = ( uint8_t ) ( flrc_req.burst_interframe_delay_us / 100 );
    SERIALIZE_8BITS( packet, burst_interframe_delay_100us );

    uint8_t burst_ack_start_delay_100us = ( uint8_t ) ( flrc_req.burst_ack_start_delay_us / 100 );
    SERIALIZE_8BITS( packet, burst_ack_start_delay_100us );

    *packet_size = ( ( intptr_t ) packet - packet_start_addr );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( *packet_size == SMTC_FLRP_REQ_PACKET_LENGTH );
}

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_flrc_req_packet( uint8_t* packet, uint16_t packet_size,
                                                                        smtc_flrp_mac_flrc_req_packet_t* flrc_req )
{
    if( packet_size != SMTC_FLRP_REQ_PACKET_LENGTH )
    {
        return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
    }

    // Ignore message type
    packet++;

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( packet, flrc_req->initiator_dev_eui[i] );
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( packet, flrc_req->receiver_dev_eui[i] );
    }

    DESERIALIZE_24BITS( packet, flrc_req->full_payload_size );

    uint16_t byte_value;
    DESERIALIZE_16BITS( packet, byte_value );
    flrc_req->uniform_payload_size =
        ( byte_value & FLRC_REQ_UNIFORM_PAYLOAD_SIZE_MASK ) >> FLRC_REQ_UNIFORM_PAYLOAD_SIZE_SHIFT;
    flrc_req->flrc_ack_disabled = ( byte_value & FLRC_REQ_FLRC_ACK_DISABLE_MASK ) >> FLRC_REQ_FLRC_ACK_DISABLE_SHIFT;
    flrc_req->coding_rate       = ( byte_value & FLRC_REQ_CODING_RATE_MASK ) >> FLRC_REQ_CODING_RATE_SHIFT;

    uint8_t burst_interframe_delay_100us;
    DESERIALIZE_8BITS( packet, burst_interframe_delay_100us );
    flrc_req->burst_interframe_delay_us = burst_interframe_delay_100us * 100;

    uint8_t burst_ack_start_delay_100us;
    DESERIALIZE_8BITS( packet, burst_ack_start_delay_100us );
    flrc_req->burst_ack_start_delay_us = burst_ack_start_delay_100us * 100;

    return SMTC_FLRP_MAC_SERDE_STATUS_OK;
}

void smtc_flrp_mac_serialize_flrc_ack_packet( uint8_t* packet, uint16_t* packet_size,
                                              smtc_flrp_mac_flrc_ack_packet_t flrc_ack )
{
    int packet_start_addr = ( intptr_t ) packet;

    uint8_t byte_value = SMTC_FLRP_MAC_FRAME_TYPE_FLRC_ACK;
    SERIALIZE_8BITS( packet, byte_value );

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( packet, flrc_ack.initiator_dev_eui[i] );
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( packet, flrc_ack.receiver_dev_eui[i] );
    }

    SERIALIZE_8BITS( packet, flrc_ack.transfer_status );
    SERIALIZE_8BITS( packet, flrc_ack.best_channel );

    byte_value =
        ( ( ( uint8_t ) flrc_ack.next_burst_data_rate ) << FLRC_ACK_DATA_RATE_SHIFT ) & FLRC_ACK_DATA_RATE_MASK;
    byte_value |=
        ( ( ( uint8_t ) flrc_ack.selected_coding_rate ) << FLRC_ACK_CODING_RATE_SHIFT ) & FLRC_ACK_CODING_RATE_MASK;
    SERIALIZE_8BITS( packet, byte_value );

    SERIALIZE_8BITS( packet, ( uint8_t ) flrc_ack.multi_burst_mode );

    SERIALIZE_8BITS( packet, flrc_ack.nb_flrc_packets_per_burst );

    uint8_t burst_start_delay_100us = ( uint8_t ) ( flrc_ack.burst_start_delay_us / 100 );
    SERIALIZE_8BITS( packet, burst_start_delay_100us );

    *packet_size = ( ( intptr_t ) packet - packet_start_addr );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( *packet_size == SMTC_FLRP_ACK_PACKET_LENGTH );
}

smtc_flrp_mac_serde_status_t smtc_flrp_mac_deserialize_flrc_ack_packet( uint8_t* packet, uint16_t packet_size,
                                                                        smtc_flrp_mac_flrc_ack_packet_t* flrc_ack )
{
    if( packet_size != SMTC_FLRP_ACK_PACKET_LENGTH )
    {
        return SMTC_FLRP_MAC_SERDE_STATUS_ERROR;
    }

    // Ignore message type
    packet++;

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( packet, flrc_ack->initiator_dev_eui[i] );
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( packet, flrc_ack->receiver_dev_eui[i] );
    }

    DESERIALIZE_8BITS( packet, flrc_ack->transfer_status );

    DESERIALIZE_8BITS( packet, flrc_ack->best_channel );

    uint8_t byte_value;
    DESERIALIZE_8BITS( packet, byte_value );
    flrc_ack->next_burst_data_rate =
        ( ral_flrc_raw_bit_rate_t ) ( ( byte_value & FLRC_ACK_DATA_RATE_MASK ) >> FLRC_ACK_DATA_RATE_SHIFT );
    flrc_ack->selected_coding_rate =
        ( ral_flrc_cr_t ) ( byte_value & FLRC_ACK_CODING_RATE_MASK ) >> FLRC_ACK_CODING_RATE_SHIFT;

    DESERIALIZE_8BITS( packet, flrc_ack->multi_burst_mode );

    DESERIALIZE_8BITS( packet, flrc_ack->nb_flrc_packets_per_burst );

    uint8_t burst_start_delay_100us;
    DESERIALIZE_8BITS( packet, burst_start_delay_100us );
    flrc_ack->burst_start_delay_us = burst_start_delay_100us * 100;

    return SMTC_FLRP_MAC_SERDE_STATUS_OK;
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION ---------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */