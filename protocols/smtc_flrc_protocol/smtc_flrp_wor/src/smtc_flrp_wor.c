/**
 * @file      smtc_flrp_core.c
 *
 * @brief     smtc_flrp_core api implementation
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

#include "smtc_flrp_wor.h"
#include "smtc_flrp_utils.h"
#include "smtc_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define FLRC_WOR_TRANSFER_DIRECTION_MASK 0x01
#define FLRC_WOR_TRANSFER_DIRECTION_SHIFT 0
#define FLRC_WOR_DISABLE_WOR_ACK_MASK 0x02
#define FLRC_WOR_DISABLE_WOR_ACK_SHIFT 1
#define FLRC_WOR_LINK_ADAPTATION_MODE_MASK 0x1C
#define FLRC_WOR_LINK_ADAPTATION_MODE_SHIFT 2
#define FLRC_WOR_DISABLE_BURST_ACK_MASK 0x20
#define FLRC_WOR_DISABLE_BURST_ACK_SHIFT 5

#define FLRC_WOR_FILTER_LENGTH_MASK 0x3F
#define FLRC_WOR_FILTER_LENGTH_SHIFT 0

#define FLRC_WOR_DEFAULT_FLRC_DR_MASK 0x0F
#define FLRC_WOR_DEFAULT_FLRC_DR_SHIFT 0
#define FLRC_WOR_DEFAULT_CHANNEL_MASK 0xF0
#define FLRC_WOR_DEFAULT_CHANNEL_SHIFT 4

#define FLRC_WOR_DEFAULT_CODING_RATE_MASK 0x3
#define FLRC_WOR_DEFAULT_CODING_RATE_SHIFT 0

#define FLRC_WOR_ACK_REQUEST_ACCEPTED_MASK 0x3
#define FLRC_WOR_ACK_REQUEST_ACCEPTED_SHIFT 0
#define FLRC_WOR_ACK_SELECTED_FLRC_DR_MASK 0x3C
#define FLRC_WOR_ACK_SELECTED_FLRC_DR_SHIFT 2
#define FLRC_WOR_ACK_SELECTED_CR_MASK 0xC0
#define FLRC_WOR_ACK_SELECTED_CR_SHIFT 6
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

void smtc_flrp_wor_serialize_payload( smtc_flrp_wor_data_t wor_data, uint8_t* payload, uint16_t* payload_size )
{
    uint16_t data_size = 0;

    int payload_start_addr = ( intptr_t ) payload;

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( payload, wor_data.initiator_dev_eui[i] );
    }
    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( payload, wor_data.slave_dev_eui[i] );
    }

    uint8_t byte_value =
        ( wor_data.initiator_send_burst << FLRC_WOR_TRANSFER_DIRECTION_SHIFT ) & FLRC_WOR_TRANSFER_DIRECTION_MASK;
    byte_value |= ( !wor_data.wor_ack_required << FLRC_WOR_DISABLE_WOR_ACK_SHIFT ) & FLRC_WOR_DISABLE_WOR_ACK_MASK;
    byte_value |= ( ( uint8_t ) wor_data.link_adaptation_mode << FLRC_WOR_LINK_ADAPTATION_MODE_SHIFT ) &
                  FLRC_WOR_LINK_ADAPTATION_MODE_MASK;
    byte_value |=
        ( !wor_data.burst_ack_required << FLRC_WOR_DISABLE_BURST_ACK_SHIFT ) & FLRC_WOR_DISABLE_BURST_ACK_MASK;
    SERIALIZE_8BITS( payload, byte_value );

    byte_value = ( wor_data.filter_len << FLRC_WOR_FILTER_LENGTH_SHIFT ) & FLRC_WOR_FILTER_LENGTH_MASK;
    SERIALIZE_8BITS( payload, byte_value );

    SERIALIZE_16BITS( payload, wor_data.enabled_channels );

    byte_value =
        ( ( uint8_t ) wor_data.default_flrc_dr << FLRC_WOR_DEFAULT_FLRC_DR_SHIFT ) & FLRC_WOR_DEFAULT_FLRC_DR_MASK;
    byte_value |= ( wor_data.default_channel << FLRC_WOR_DEFAULT_CHANNEL_SHIFT ) & FLRC_WOR_DEFAULT_CHANNEL_MASK;
    SERIALIZE_8BITS( payload, byte_value );

    byte_value =
        ( wor_data.default_coding_rate << FLRC_WOR_DEFAULT_CODING_RATE_SHIFT ) & FLRC_WOR_DEFAULT_CODING_RATE_MASK;
    SERIALIZE_8BITS( payload, byte_value );

    uint16_t next_phase_start_delay_100us = ( uint16_t ) ( wor_data.next_phase_start_delay_us / 100 );
    SERIALIZE_16BITS( payload, next_phase_start_delay_100us );

    uint8_t channel_interframe_delay_100us = ( uint8_t ) ( wor_data.channel_interframe_delay_us / 100 );
    SERIALIZE_8BITS( payload, channel_interframe_delay_100us );

    data_size = ( ( intptr_t ) payload - payload_start_addr );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( data_size == SMTC_FLRP_WOR_PAYLOAD_SIZE );

    if( payload_size != NULL )
    {
        *payload_size = data_size;
    }
}

bool smtc_flrp_wor_deserialize_payload( smtc_flrp_wor_data_t* wor_data, uint8_t* payload, uint16_t payload_size )
{
    if( payload_size < SMTC_FLRP_WOR_PAYLOAD_SIZE )
    {
        return false;
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( payload, wor_data->initiator_dev_eui[i] );
    }
    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( payload, wor_data->slave_dev_eui[i] );
    }

    uint8_t byte_value;
    DESERIALIZE_8BITS( payload, byte_value );
    wor_data->initiator_send_burst =
        ( byte_value & FLRC_WOR_TRANSFER_DIRECTION_MASK ) >> FLRC_WOR_TRANSFER_DIRECTION_SHIFT;
    wor_data->wor_ack_required = !( ( byte_value & FLRC_WOR_DISABLE_WOR_ACK_MASK ) >> FLRC_WOR_DISABLE_WOR_ACK_SHIFT );
    wor_data->link_adaptation_mode =
        ( smtc_flrp_link_adaptation_mode_t ) ( ( byte_value & FLRC_WOR_LINK_ADAPTATION_MODE_MASK ) >>
                                               FLRC_WOR_LINK_ADAPTATION_MODE_SHIFT );
    wor_data->burst_ack_required =
        !( ( byte_value & FLRC_WOR_DISABLE_BURST_ACK_MASK ) >> FLRC_WOR_DISABLE_BURST_ACK_SHIFT );

    DESERIALIZE_8BITS( payload, byte_value );
    wor_data->filter_len = ( byte_value & FLRC_WOR_FILTER_LENGTH_MASK ) >> FLRC_WOR_FILTER_LENGTH_SHIFT;

    DESERIALIZE_16BITS( payload, wor_data->enabled_channels );

    DESERIALIZE_8BITS( payload, byte_value );
    wor_data->default_flrc_dr =
        ( ral_flrc_raw_bit_rate_t ) ( byte_value & FLRC_WOR_DEFAULT_FLRC_DR_MASK ) >> FLRC_WOR_DEFAULT_FLRC_DR_SHIFT;
    wor_data->default_channel = ( byte_value & FLRC_WOR_DEFAULT_CHANNEL_MASK ) >> FLRC_WOR_DEFAULT_CHANNEL_SHIFT;

    DESERIALIZE_8BITS( payload, byte_value );
    wor_data->default_coding_rate =
        ( byte_value & FLRC_WOR_DEFAULT_CODING_RATE_MASK ) >> FLRC_WOR_DEFAULT_CODING_RATE_SHIFT;

    uint16_t next_phase_start_delay_100us;
    DESERIALIZE_16BITS( payload, next_phase_start_delay_100us );
    wor_data->next_phase_start_delay_us = next_phase_start_delay_100us * 100;

    uint8_t channel_interframe_delay_100us;
    DESERIALIZE_8BITS( payload, channel_interframe_delay_100us );
    wor_data->channel_interframe_delay_us = channel_interframe_delay_100us * 100;

    return true;
}

void smtc_flrp_wor_ack_serialize_payload( smtc_flrp_wor_ack_data_t wor_ack_data, uint8_t* payload,
                                          uint16_t* payload_size )
{
    uint16_t data_size          = 0;
    int      payload_start_addr = ( intptr_t ) payload;

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        SERIALIZE_8BITS( payload, wor_ack_data.receiver_dev_eui[i] );
    }

    uint8_t first_byte =
        ( wor_ack_data.ack_status << FLRC_WOR_ACK_REQUEST_ACCEPTED_SHIFT ) & FLRC_WOR_ACK_REQUEST_ACCEPTED_MASK;
    first_byte |=
        ( wor_ack_data.selected_flrc_dr << FLRC_WOR_ACK_SELECTED_FLRC_DR_SHIFT ) & FLRC_WOR_ACK_SELECTED_FLRC_DR_MASK;
    first_byte |=
        ( wor_ack_data.selected_coding_rate << FLRC_WOR_ACK_SELECTED_CR_SHIFT ) & FLRC_WOR_ACK_SELECTED_CR_MASK;

    SERIALIZE_8BITS( payload, first_byte );

    uint8_t reserved_byte = 0x00;
    SERIALIZE_8BITS( payload, reserved_byte );

    uint16_t next_phase_start_delay_100us = ( uint16_t ) ( wor_ack_data.next_phase_start_delay_us / 100 );
    SERIALIZE_16BITS( payload, next_phase_start_delay_100us );

    if( wor_ack_data.has_min_interframe_delay )
    {
        uint8_t min_interframe_delay_100us = ( uint8_t ) ( wor_ack_data.min_interframe_delay_us / 100 );
        SERIALIZE_8BITS( payload, min_interframe_delay_100us );

        data_size = ( ( intptr_t ) payload - payload_start_addr );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( data_size == SMTC_FLRP_WOR_ACK_PAYLOAD_MAX_SIZE );
    }
    else
    {
        data_size = ( ( intptr_t ) payload - payload_start_addr );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( data_size == SMTC_FLRP_WOR_ACK_PAYLOAD_MIN_SIZE );
    }

    if( payload_size != NULL )
    {
        *payload_size = data_size;
    }
}

bool smtc_flrp_wor_ack_deserialize_payload( smtc_flrp_wor_ack_data_t* wor_ack_data, uint8_t* payload,
                                            uint16_t payload_size )
{
    if( ( wor_ack_data->has_min_interframe_delay && ( payload_size < SMTC_FLRP_WOR_ACK_PAYLOAD_MAX_SIZE ) ) ||
        ( !wor_ack_data->has_min_interframe_delay && ( payload_size < SMTC_FLRP_WOR_ACK_PAYLOAD_MIN_SIZE ) ) )
    {
        return false;
    }

    for( uint8_t i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        DESERIALIZE_8BITS( payload, wor_ack_data->receiver_dev_eui[i] );
    }

    uint8_t first_byte;
    DESERIALIZE_8BITS( payload, first_byte );

    wor_ack_data->ack_status = ( smtc_flrp_wor_ack_status_t ) ( ( first_byte & FLRC_WOR_ACK_REQUEST_ACCEPTED_MASK ) >>
                                                                FLRC_WOR_ACK_REQUEST_ACCEPTED_SHIFT );
    wor_ack_data->selected_flrc_dr =
        ( first_byte & FLRC_WOR_ACK_SELECTED_FLRC_DR_MASK ) >> FLRC_WOR_ACK_SELECTED_FLRC_DR_SHIFT;

    wor_ack_data->selected_coding_rate =
        ( first_byte & FLRC_WOR_ACK_SELECTED_CR_MASK ) >> FLRC_WOR_ACK_SELECTED_CR_SHIFT;

    // Reserved byte
    payload++;

    uint16_t next_phase_start_delay_100us;
    DESERIALIZE_16BITS( payload, next_phase_start_delay_100us );
    wor_ack_data->next_phase_start_delay_us = next_phase_start_delay_100us * 100;

    if( wor_ack_data->has_min_interframe_delay )
    {
        uint8_t min_interframe_delay_100us;
        DESERIALIZE_8BITS( payload, min_interframe_delay_100us );
        wor_ack_data->min_interframe_delay_us = min_interframe_delay_100us * 100;
    }

    return true;
}

void smtc_flrp_wor_handle_payload( smtc_flrp_wor_data_t wor_data, smtc_rac_data_result_t data_result, bool data_to_send,
                                   bool* ack_to_send, smtc_flrp_wor_ack_data_t* wor_ack_data,
                                   smtc_flrp_wor_rx_info_for_ack_t info_wor_ack )
{
    *ack_to_send = false;

    if( wor_data.wor_ack_required )
    {
        *ack_to_send = true;

        if( !wor_data.initiator_send_burst && !data_to_send )
        {
            wor_ack_data->ack_status = SMTC_FLRP_WOR_DECLINED_NO_DATA_TO_TRANSFER;
        }
        else
        {
            wor_ack_data->ack_status = SMTC_FLRP_WOR_ACCEPTED;
        }
    }

    wor_ack_data->has_min_interframe_delay = wor_data.initiator_send_burst;

    memcpy( wor_ack_data->receiver_dev_eui, wor_data.initiator_dev_eui, SMTC_FLRP_EUI_LENGTH );

    wor_ack_data->selected_flrc_dr     = wor_data.default_flrc_dr;
    wor_ack_data->selected_coding_rate = wor_data.default_coding_rate;

    if( wor_data.link_adaptation_mode != SMTC_FLRP_LINK_ADAPTATION_DISABLED )
    {
        wor_ack_data->next_phase_start_delay_us = info_wor_ack.delay_until_flrc_req_us;
    }
    else
    {
        wor_ack_data->next_phase_start_delay_us = info_wor_ack.delay_until_burst_us;
    }
}

/* --- EOF ------------------------------------------------------------------ */