/**
 * @file      smtc_flrp_mac_adaptive_link.c
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "smtc_flrp_mac_adaptive_link.h"

#include "smtc_flrp_mac_config.h"
#include "smtc_flrp_mac_serde.h"
#include "flrp_configuration.h"
#include "flrp_defs.h"
#include "smtc_flrp_utils.h"
#include "smtc_modem_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define MAC_RX_ADAPTIVE_LINK_TIMEOUT_MS ( 1 + SMTC_FLRP_TX_DELAY_MARGIN_MS + 2 )

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

static uint32_t smtc_flrp_mac_channel_get_frequency( smtc_flrp_mac_radio_config_t* mac_radio_config,
                                                     uint8_t                       channel_idx );

/*!
 * \brief Get the best channel from the last adaptive link configuration.
 *
 * Note: If two channels have the same RSSI, the first channel configured is returned.
 *
 * \param [in] flrc_req_rssi    RSSIs of the last flrc req packets received
 * \param [in] nb_flrc_packets  Number of flrc req packets received
 * \param [out] best_channel    The best channel obeserved
 *
 * \return true is a channel is found, false if no flrc req was received.
 */
static bool smtc_flrp_mac_get_flrc_req_best_channel( int16_t* flrc_req_rssi, uint8_t nb_flrc_packets,
                                                     uint8_t* best_channel );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bool smtc_flrp_mac_at_least_one_flrc_req_was_received( int16_t* flrc_req_rssi, uint8_t nb_flrc_packets )
{
    for( uint8_t i = 0; i < nb_flrc_packets; i++ )
    {
        if( flrc_req_rssi[i] != MAC_INVALID_RSSI )
        {
            return true;
        }
    }
    return false;
}

void smtc_flrp_mac_prepare_rx_flrc_req( smtc_rac_context_t* transaction, smtc_flrp_mac_radio_config_t* mac_radio_config,
                                        uint8_t* rx_buffer, uint32_t freq, int32_t freq_offset )
{
    smtc_flrp_mac_set_radio_params_flrc_mod( transaction, mac_radio_config, freq, freq_offset,
                                             mac_radio_config->raw_bit_rate, mac_radio_config->cr );
    transaction->radio_params.flrc.is_tx         = false;
    transaction->radio_params.flrc.max_rx_size   = SMTC_FLRP_REQ_PACKET_LENGTH;
    transaction->radio_params.flrc.rx_timeout_ms = MAC_RX_ADAPTIVE_LINK_TIMEOUT_MS;

    transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = rx_buffer;
    transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = SMTC_FLRP_REQ_PACKET_LENGTH;
}

void smtc_flrp_mac_prepare_rx_flrc_ack( smtc_rac_context_t* transaction, smtc_flrp_mac_radio_config_t* mac_radio_config,
                                        uint8_t* rx_buffer, uint32_t freq, int32_t freq_offset )
{
    smtc_flrp_mac_set_radio_params_flrc_mod( transaction, mac_radio_config, freq, freq_offset,
                                             mac_radio_config->raw_bit_rate, mac_radio_config->cr );
    transaction->radio_params.flrc.is_tx         = false;
    transaction->radio_params.flrc.max_rx_size   = SMTC_FLRP_ACK_PACKET_LENGTH;
    transaction->radio_params.flrc.rx_timeout_ms = MAC_RX_ADAPTIVE_LINK_TIMEOUT_MS;

    transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = rx_buffer;
    transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = SMTC_FLRP_ACK_PACKET_LENGTH;
}

void smtc_flrp_mac_prepare_tx_flrc_req( smtc_rac_context_t* transaction, smtc_flrp_mac_radio_config_t* mac_radio_config,
                                        uint32_t freq, int32_t freq_offset, uint8_t* dev_eui, uint8_t* dest_dev_eui,
                                        smtc_flrp_mac_burst_info_in_adaptive_link_t burst_info,
                                        smtc_flrp_mac_adaptive_link_t adaptive_link_config, uint8_t* tx_buffer )
{
    smtc_flrp_mac_set_radio_params_flrc_mod( transaction, mac_radio_config, freq, freq_offset,
                                             mac_radio_config->raw_bit_rate, mac_radio_config->cr );

    transaction->radio_params.flrc.is_tx = true;

    smtc_flrp_mac_flrc_req_packet_t flrc_req;
    memcpy( flrc_req.initiator_dev_eui, dev_eui, SMTC_FLRP_EUI_LENGTH );
    memcpy( flrc_req.receiver_dev_eui, dest_dev_eui, SMTC_FLRP_EUI_LENGTH );
    flrc_req.full_payload_size         = burst_info.payload_length;
    flrc_req.uniform_payload_size      = burst_info.packet_uniformed_size;
    flrc_req.flrc_ack_disabled         = !adaptive_link_config.flrc_ack_enabled;
    flrc_req.burst_ack_start_delay_us  = burst_info.burst_ack_start_delay_ms * 1000;
    flrc_req.burst_interframe_delay_us = burst_info.interframe_delay_us;
    flrc_req.coding_rate               = mac_radio_config->cr;

    uint16_t flrc_req_size;
    smtc_flrp_mac_serialize_flrc_req_packet( tx_buffer, &flrc_req_size, flrc_req );

    transaction->radio_params.flrc.tx_size                            = flrc_req_size;
    transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = tx_buffer;
    transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = flrc_req_size;
}

void smtc_flrp_mac_prepare_tx_flrc_ack( smtc_rac_context_t* transaction, smtc_flrp_mac_radio_config_t* mac_radio_config,
                                        uint32_t freq, int32_t freq_offset, uint8_t* dev_eui, uint8_t* dest_dev_eui,
                                        smtc_flrp_mac_adaptive_link_t*               adaptive_link_config,
                                        smtc_flrp_mac_burst_info_in_adaptive_link_t* burst_info, uint8_t* tx_buffer )
{
    smtc_flrp_mac_set_radio_params_flrc_mod( transaction, mac_radio_config, freq, freq_offset,
                                             mac_radio_config->raw_bit_rate, mac_radio_config->cr );
    transaction->radio_params.flrc.is_tx = true;

    smtc_flrp_mac_flrc_ack_packet_t flrc_ack;
    flrc_ack.transfer_status = SMTC_FLRP_MAC_ADAPTIVE_LINK_ACCEPTED;

    memcpy( flrc_ack.initiator_dev_eui, dest_dev_eui, SMTC_FLRP_EUI_LENGTH );
    memcpy( flrc_ack.receiver_dev_eui, dev_eui, SMTC_FLRP_EUI_LENGTH );

    uint8_t channel_idx    = MAC_INVALID_CHANNEL;
    bool    channel_config = ( adaptive_link_config->mode == SMTC_FLRP_LINK_ADAPTATION_CHANNEL_SELECTION_ONLY ) ||
                          ( adaptive_link_config->mode == SMTC_FLRP_LINK_ADAPTATION_FULLY_ENABLED );
    if( channel_config )
    {
        if( !smtc_flrp_mac_get_flrc_req_best_channel( adaptive_link_config->flrc_req_rssi,
                                                      adaptive_link_config->frequencies_to_test_size, &channel_idx ) )
        {
            // SMTC_MODEM_HAL_TRACE_ERROR( "FLRC Rx MAC Layer: Not able to found a best channel \n" );
            if( adaptive_link_config->retry_enabled )
            {
                flrc_ack.transfer_status = SMTC_FLRP_MAC_ADAPTIVE_LINK_RETRY;
            }
            else
            {
                flrc_ack.transfer_status = SMTC_FLRP_MAC_ADAPTIVE_LINK_ABORT;
            }
        }
        else
        {
            burst_info->frequency_hz = smtc_flrp_mac_channel_get_frequency( mac_radio_config, channel_idx );
        }
    }
    adaptive_link_config->last_flrc_ack_transfer_status = flrc_ack.transfer_status;

    flrc_ack.best_channel              = channel_idx;
    flrc_ack.next_burst_data_rate      = burst_info->raw_bit_rate;
    flrc_ack.multi_burst_mode          = burst_info->multi_burst_mode;
    flrc_ack.nb_flrc_packets_per_burst = burst_info->nb_packets_burst_max;
    flrc_ack.burst_start_delay_us      = mac_radio_config->start_burst_delay_us;
    flrc_ack.selected_coding_rate      = burst_info->coding_rate;

    uint16_t packet_size;
    smtc_flrp_mac_serialize_flrc_ack_packet( tx_buffer, &packet_size, flrc_ack );

    transaction->radio_params.flrc.tx_size                            = packet_size;
    transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = tx_buffer;
    transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = packet_size;
}

bool smtc_flrp_mac_handle_flrc_req_packet( uint8_t* rx_packet, uint32_t rx_size, int32_t rssi, uint8_t* dev_eui,
                                           uint8_t dev_eui_filter_len, uint8_t* expected_src_dev_eui,
                                           smtc_flrp_mac_adaptive_link_t*               adaptive_link_config,
                                           smtc_flrp_mac_burst_info_in_adaptive_link_t* burst_info )
{
    smtc_flrp_mac_flrc_req_packet_t flrc_req;
    bool                            frame_ok = false;

    if( smtc_flrp_mac_deserialize_flrc_req_packet( rx_packet, rx_size, &flrc_req ) != SMTC_FLRP_MAC_SERDE_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FLRC Rx MAC: Failed to deserialize FLRC Req \n" );
    }
    else
    {
        if( !memcmp( expected_src_dev_eui, flrc_req.initiator_dev_eui, SMTC_FLRP_EUI_LENGTH ) )
        {
            if( !smtc_flrp_is_frame_addressed_to_this_device( dev_eui, flrc_req.receiver_dev_eui, dev_eui_filter_len ) )
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "FLRC Rx MAC: Packet FLRC Req is for an other device\n" );
            }
            else
            {
                adaptive_link_config->flrc_req_rssi[adaptive_link_config->frequency_test_idx] = rssi;
                burst_info->burst_ack_start_delay_ms   = flrc_req.burst_ack_start_delay_us / 1000;
                burst_info->payload_length             = flrc_req.full_payload_size;
                burst_info->packet_uniformed_size      = flrc_req.uniform_payload_size;
                adaptive_link_config->flrc_ack_enabled = !flrc_req.flrc_ack_disabled;
                burst_info->interframe_delay_us        = flrc_req.burst_interframe_delay_us;
                burst_info->coding_rate                = flrc_req.coding_rate;
                frame_ok                               = true;
            }
        }
    }
    return frame_ok;
}

bool smtc_flrp_mac_handle_flrc_ack_packet( uint8_t* rx_packet, uint32_t rx_size, uint8_t* dev_eui,
                                           uint8_t*                                     expected_src_dev_eui,
                                           smtc_flrp_mac_adaptive_link_t*               adaptive_link_config,
                                           smtc_flrp_mac_burst_info_in_adaptive_link_t* burst_info,
                                           smtc_flrp_mac_radio_config_t*                mac_radio_config )
{
    smtc_flrp_mac_flrc_ack_packet_t flrc_ack;
    bool                            frame_ok = false;

    if( smtc_flrp_mac_deserialize_flrc_ack_packet( rx_packet, rx_size, &flrc_ack ) != SMTC_FLRP_MAC_SERDE_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FLRC Rx MAC: Failed to deserialize FLRC Req \n" );
    }
    else
    {
        // ACK is for only one device, the receiver dev eui must equals the targetted dev eui (no filter len)
        if( memcmp( flrc_ack.initiator_dev_eui, dev_eui, SMTC_FLRP_EUI_LENGTH ) ||
            memcmp( flrc_ack.receiver_dev_eui, expected_src_dev_eui, SMTC_FLRP_EUI_LENGTH ) )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "FLRC Rx MAC: Packet FLRC ACK is for/from an other device\n" );
        }
        else
        {
            adaptive_link_config->last_flrc_ack_transfer_status = flrc_ack.transfer_status;
            if( adaptive_link_config->last_flrc_ack_transfer_status == SMTC_FLRP_MAC_ADAPTIVE_LINK_ACCEPTED )
            {
                burst_info->frequency_hz =
                    smtc_flrp_mac_channel_get_frequency( mac_radio_config, flrc_ack.best_channel );
                burst_info->first_burst_start_delay_ms = flrc_ack.burst_start_delay_us / 1000;
                burst_info->raw_bit_rate               = flrc_ack.next_burst_data_rate;
                burst_info->multi_burst_mode           = flrc_ack.multi_burst_mode;
                burst_info->nb_packets_burst_max       = flrc_ack.nb_flrc_packets_per_burst;
                burst_info->coding_rate                = flrc_ack.selected_coding_rate;
                frame_ok                               = true;
            }
        }
    }
    return frame_ok;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION ---------------------------------------------
 */

static uint32_t smtc_flrp_mac_channel_get_frequency( smtc_flrp_mac_radio_config_t* mac_radio_config,
                                                     uint8_t                       channel_idx )
{
    if( channel_idx > mac_radio_config->nb_frequencies )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Mac channel requested %u is invalid \n", channel_idx );
        return mac_radio_config->frequencies_hz[0];
    }
    return mac_radio_config->frequencies_hz[channel_idx];
}

static bool smtc_flrp_mac_get_flrc_req_best_channel( int16_t* flrc_req_rssi, uint8_t nb_flrc_packets,
                                                     uint8_t* best_channel )
{
    int16_t best_rssi     = flrc_req_rssi[0];
    uint8_t idx_best_rssi = 0;
    for( uint8_t i = 1; i < nb_flrc_packets; i++ )
    {
        if( flrc_req_rssi[i] > best_rssi )
        {
            best_rssi     = flrc_req_rssi[i];
            idx_best_rssi = i;
        }
    }
    if( best_rssi != MAC_INVALID_RSSI )
    {
        *best_channel = idx_best_rssi;
        return 1;
    }
    return 0;
}
/* --- EOF ------------------------------------------------------------------ */