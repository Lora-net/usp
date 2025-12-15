/**
 * @file      rac_context_converter.c
 *
 * @brief     rac_context_converter implementation
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

#include "rac_context_converter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Forward declarations for enum conversion functions
static ral_lora_sf_t                convert_pb_sf_to_native( lora_spreading_factor_pb_t pb_sf );
static lora_spreading_factor_pb_t   convert_native_sf_to_pb( ral_lora_sf_t native_sf );
static ral_lora_bw_t                convert_pb_bw_to_native( lora_bandwidth_pb_t pb_bw );
static ral_lora_cr_t                convert_pb_cr_to_native( lora_coding_rate_pb_t pb_cr );
static lora_coding_rate_pb_t        convert_native_cr_to_pb( ral_lora_cr_t native_cr );
static ral_lora_pkt_len_modes_t     convert_pb_header_type_to_native( lora_packet_length_mode_pb_t pb_header );
static lora_packet_length_mode_pb_t convert_native_header_type_to_pb( ral_lora_pkt_len_modes_t native_header );
static smtc_rac_lora_syncword_t     convert_pb_sync_word_to_native( lora_syncword_pb_t pb_sync );
static lora_syncword_pb_t           convert_native_sync_word_to_pb( smtc_rac_lora_syncword_t native_sync );

// ========================================
// ENUM CONVERSION FUNCTIONS
// ========================================

smtc_rac_priority_pb_t rac_convert_priority_to_pb( smtc_rac_priority_t native_priority )
{
    switch( native_priority )
    {
    case RAC_VERY_HIGH_PRIORITY:
        return smtc_rac_priority_pb_t_RAC_VERY_HIGH_PRIORITY_PB;
    case RAC_HIGH_PRIORITY:
        return smtc_rac_priority_pb_t_RAC_HIGH_PRIORITY_PB;
    case RAC_MEDIUM_PRIORITY:
        return smtc_rac_priority_pb_t_RAC_MEDIUM_PRIORITY_PB;
    case RAC_LOW_PRIORITY:
        return smtc_rac_priority_pb_t_RAC_LOW_PRIORITY_PB;
    case RAC_VERY_LOW_PRIORITY:
        return smtc_rac_priority_pb_t_RAC_VERY_LOW_PRIORITY_PB;
    default:
        return smtc_rac_priority_pb_t_RAC_LOW_PRIORITY_PB;  // Safe default
    }
}

smtc_rac_priority_t rac_convert_priority_from_pb( smtc_rac_priority_pb_t pb_priority )
{
    switch( pb_priority )
    {
    case smtc_rac_priority_pb_t_RAC_VERY_HIGH_PRIORITY_PB:
        return RAC_VERY_HIGH_PRIORITY;
    case smtc_rac_priority_pb_t_RAC_HIGH_PRIORITY_PB:
        return RAC_HIGH_PRIORITY;
    case smtc_rac_priority_pb_t_RAC_MEDIUM_PRIORITY_PB:
        return RAC_MEDIUM_PRIORITY;
    case smtc_rac_priority_pb_t_RAC_LOW_PRIORITY_PB:
        return RAC_LOW_PRIORITY;
    case smtc_rac_priority_pb_t_RAC_VERY_LOW_PRIORITY_PB:
        return RAC_VERY_LOW_PRIORITY;
    default:
        return RAC_LOW_PRIORITY;  // Safe default
    }
}

smtc_rac_return_code_pb_t rac_convert_return_code_to_pb( smtc_rac_return_code_t native_code )
{
    switch( native_code )
    {
    case SMTC_RAC_SUCCESS:
        return smtc_rac_return_code_pb_t_SMTC_RAC_SUCCESS_PB;
    case SMTC_RAC_ERROR:
        return smtc_rac_return_code_pb_t_SMTC_RAC_ERROR_PB;
    case SMTC_RAC_BUSY:
        return smtc_rac_return_code_pb_t_SMTC_RAC_BUSY_PB;
    case SMTC_RAC_TIMEOUT:
        return smtc_rac_return_code_pb_t_SMTC_RAC_TIMEOUT_PB;
    case SMTC_RAC_INVALID_PARAMETER:
        return smtc_rac_return_code_pb_t_SMTC_RAC_INVALID_PARAMETER_PB;
    case SMTC_RAC_NOT_SUPPORTED:
        return smtc_rac_return_code_pb_t_SMTC_RAC_NOT_SUPPORTED_PB;
    case SMTC_RAC_NOT_INITIALIZED:
        return smtc_rac_return_code_pb_t_SMTC_RAC_NOT_INITIALIZED_PB;
    case SMTC_RAC_NOT_IMPLEMENTED:
        return smtc_rac_return_code_pb_t_SMTC_RAC_NOT_IMPLEMENTED_PB;
    default:
        return smtc_rac_return_code_pb_t_SMTC_RAC_ERROR_PB;  // Safe default
    }
}

smtc_rac_return_code_t rac_convert_return_code_from_pb( smtc_rac_return_code_pb_t pb_code )
{
    switch( pb_code )
    {
    case smtc_rac_return_code_pb_t_SMTC_RAC_SUCCESS_PB:
        return SMTC_RAC_SUCCESS;
    case smtc_rac_return_code_pb_t_SMTC_RAC_ERROR_PB:
        return SMTC_RAC_ERROR;
    case smtc_rac_return_code_pb_t_SMTC_RAC_BUSY_PB:
        return SMTC_RAC_BUSY;
    case smtc_rac_return_code_pb_t_SMTC_RAC_TIMEOUT_PB:
        return SMTC_RAC_TIMEOUT;
    case smtc_rac_return_code_pb_t_SMTC_RAC_INVALID_PARAMETER_PB:
        return SMTC_RAC_INVALID_PARAMETER;
    case smtc_rac_return_code_pb_t_SMTC_RAC_NOT_SUPPORTED_PB:
        return SMTC_RAC_NOT_SUPPORTED;
    case smtc_rac_return_code_pb_t_SMTC_RAC_NOT_INITIALIZED_PB:
        return SMTC_RAC_NOT_INITIALIZED;
    case smtc_rac_return_code_pb_t_SMTC_RAC_NOT_IMPLEMENTED_PB:
        return SMTC_RAC_NOT_IMPLEMENTED;
    default:
        return SMTC_RAC_ERROR;  // Safe default
    }
}

smtc_rac_scheduling_pb_t rac_convert_scheduling_to_pb( smtc_rac_scheduling_t native_scheduling )
{
    switch( native_scheduling )
    {
    case SMTC_RAC_SCHEDULED_TRANSACTION:
        return smtc_rac_scheduling_pb_t_SMTC_RAC_SCHEDULED_TRANSACTION_PB;
    case SMTC_RAC_ASAP_TRANSACTION:
        return smtc_rac_scheduling_pb_t_SMTC_RAC_ASAP_TRANSACTION_PB;
    default:
        return smtc_rac_scheduling_pb_t_SMTC_RAC_ASAP_TRANSACTION_PB;  // Safe default
    }
}

smtc_rac_scheduling_t rac_convert_scheduling_from_pb( smtc_rac_scheduling_pb_t pb_scheduling )
{
    switch( pb_scheduling )
    {
    case smtc_rac_scheduling_pb_t_SMTC_RAC_SCHEDULED_TRANSACTION_PB:
        return SMTC_RAC_SCHEDULED_TRANSACTION;
    case smtc_rac_scheduling_pb_t_SMTC_RAC_ASAP_TRANSACTION_PB:
        return SMTC_RAC_ASAP_TRANSACTION;
    default:
        return SMTC_RAC_ASAP_TRANSACTION;  // Safe default
    }
}

smtc_rac_modulation_type_pb_t rac_convert_modulation_type_to_pb( smtc_rac_modulation_type_t native_modulation )
{
    switch( native_modulation )
    {
    case SMTC_RAC_MODULATION_LORA:
        return smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_LORA_PB;
    case SMTC_RAC_MODULATION_FSK:
        return smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_FSK_PB;
    case SMTC_RAC_MODULATION_LRFHSS:
        return smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_LRFHSS_PB;
    case SMTC_RAC_MODULATION_FLRC:
        return smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_FLRC_PB;
    default:
        return smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_LORA_PB;  // Default to LoRa
    }
}

smtc_rac_modulation_type_t rac_convert_modulation_type_from_pb( smtc_rac_modulation_type_pb_t pb_modulation )
{
    switch( pb_modulation )
    {
    case smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_LORA_PB:
        return SMTC_RAC_MODULATION_LORA;
    case smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_FSK_PB:
        return SMTC_RAC_MODULATION_FSK;
    case smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_LRFHSS_PB:
        return SMTC_RAC_MODULATION_LRFHSS;
    case smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_FLRC_PB:
        return SMTC_RAC_MODULATION_FLRC;
    default:
        return SMTC_RAC_MODULATION_LORA;  // Default to LoRa
    }
}

// ========================================
// STRUCTURE CONVERSION FUNCTIONS
// ========================================

// Helper function: convert native RAL bandwidth to protobuf bandwidth
static lora_bandwidth_pb_t convert_native_bw_to_pb( ral_lora_bw_t native_bw )
{
    switch( native_bw )
    {
    case RAL_LORA_BW_007_KHZ:
        return lora_bandwidth_pb_t_BW_7_8_KHZ_PB;
    case RAL_LORA_BW_010_KHZ:
        return lora_bandwidth_pb_t_BW_10_4_KHZ_PB;
    case RAL_LORA_BW_015_KHZ:
        return lora_bandwidth_pb_t_BW_15_6_KHZ_PB;
    case RAL_LORA_BW_020_KHZ:
        return lora_bandwidth_pb_t_BW_20_8_KHZ_PB;
    case RAL_LORA_BW_031_KHZ:
        return lora_bandwidth_pb_t_BW_31_25_KHZ_PB;
    case RAL_LORA_BW_041_KHZ:
        return lora_bandwidth_pb_t_BW_41_7_KHZ_PB;
    case RAL_LORA_BW_062_KHZ:
        return lora_bandwidth_pb_t_BW_62_5_KHZ_PB;
    case RAL_LORA_BW_125_KHZ:
        return lora_bandwidth_pb_t_BW_125_KHZ_PB;
    case RAL_LORA_BW_250_KHZ:
        return lora_bandwidth_pb_t_BW_250_KHZ_PB;
    case RAL_LORA_BW_500_KHZ:
        return lora_bandwidth_pb_t_BW_500_KHZ_PB;
    default:
        return lora_bandwidth_pb_t_BW_125_KHZ_PB;  // Default to 125 kHz
    }
}

// Helper function: convert native RAL spreading factor to protobuf spreading factor
static lora_spreading_factor_pb_t convert_native_sf_to_pb( ral_lora_sf_t native_sf )
{
    switch( native_sf )
    {
    case RAL_LORA_SF5:
        return lora_spreading_factor_pb_t_SF5_PB;
    case RAL_LORA_SF6:
        return lora_spreading_factor_pb_t_SF6_PB;
    case RAL_LORA_SF7:
        return lora_spreading_factor_pb_t_SF7_PB;
    case RAL_LORA_SF8:
        return lora_spreading_factor_pb_t_SF8_PB;
    case RAL_LORA_SF9:
        return lora_spreading_factor_pb_t_SF9_PB;
    case RAL_LORA_SF10:
        return lora_spreading_factor_pb_t_SF10_PB;
    case RAL_LORA_SF11:
        return lora_spreading_factor_pb_t_SF11_PB;
    case RAL_LORA_SF12:
        return lora_spreading_factor_pb_t_SF12_PB;
    default:
        return lora_spreading_factor_pb_t_SF7_PB;  // Default to SF7
    }
}

void rac_convert_rttof_to_pb( const smtc_rac_rttof_params_t* native_rttof, rttof_params_pb_t* pb_rttof )
{
    if( !native_rttof || !pb_rttof )
    {
        return;
    }

    // Copy fields directly - native RTToF struct → protobuf
    pb_rttof->request_address        = native_rttof->request_address;
    pb_rttof->delay_indicator        = native_rttof->delay_indicator;
    pb_rttof->response_symbols_count = native_rttof->response_symbols_count;

    // Convert bandwidth enum: native RAL → protobuf
    pb_rttof->bw_ranging = convert_native_bw_to_pb( native_rttof->bw_ranging );
}

void rac_convert_rttof_from_pb( const rttof_params_pb_t* pb_rttof, smtc_rac_rttof_params_t* native_rttof )
{
    if( !pb_rttof || !native_rttof )
    {
        return;
    }

    // Copy fields directly - protobuf → native RTToF struct
    native_rttof->request_address        = pb_rttof->request_address;
    native_rttof->delay_indicator        = pb_rttof->delay_indicator;
    native_rttof->response_symbols_count = pb_rttof->response_symbols_count;

    // Convert bandwidth enum: protobuf → native RAL
    native_rttof->bw_ranging = convert_pb_bw_to_native( pb_rttof->bw_ranging );
}

void rac_convert_radio_params_to_pb( const smtc_rac_radio_lora_params_t* native_params,
                                     rac_radio_lora_params_pb_t*         pb_params )
{
    if( !native_params || !pb_params )
    {
        return;
    }

    // Set has_ field for rttof
    pb_params->has_rttof = true;

    pb_params->is_tx                = native_params->is_tx;
    pb_params->is_ranging_exchange  = native_params->is_ranging_exchange;
    pb_params->frequency_in_hz      = native_params->frequency_in_hz;
    pb_params->tx_power_in_dbm      = ( uint32_t ) native_params->tx_power_in_dbm;  // uint8_t -> uint32_t conversion
    pb_params->preamble_len_in_symb = native_params->preamble_len_in_symb;
    pb_params->invert_iq_is_on      = ( native_params->invert_iq_is_on != 0 );  // uint8_t -> bool conversion
    pb_params->crc_is_on            = ( native_params->crc_is_on != 0 );        // uint8_t -> bool conversion
    pb_params->rx_timeout_ms        = native_params->rx_timeout_ms;
    pb_params->symb_nb_timeout      = ( uint32_t ) native_params->symb_nb_timeout;
    pb_params->max_rx_size          = native_params->max_rx_size;
    pb_params->tx_size              = native_params->tx_size;

    // Convert RTToF params
    rac_convert_rttof_to_pb( &native_params->rttof, &pb_params->rttof );

    // Convert RAL enum types (now available):
    pb_params->sf          = convert_native_sf_to_pb( native_params->sf );
    pb_params->bw          = convert_native_bw_to_pb( native_params->bw );
    pb_params->cr          = convert_native_cr_to_pb( native_params->cr );
    pb_params->header_type = convert_native_header_type_to_pb( native_params->header_type );
    pb_params->sync_word   = convert_native_sync_word_to_pb( native_params->sync_word );
}

void rac_convert_radio_params_from_pb( const rac_radio_lora_params_pb_t* pb_params,
                                       smtc_rac_radio_lora_params_t*     native_params )
{
    if( !pb_params || !native_params )
    {
        return;
    }

    native_params->is_tx                = pb_params->is_tx;
    native_params->is_ranging_exchange  = pb_params->is_ranging_exchange;
    native_params->frequency_in_hz      = pb_params->frequency_in_hz;
    native_params->tx_power_in_dbm      = ( uint8_t ) pb_params->tx_power_in_dbm;  // uint32_t -> uint8_t conversion
    native_params->preamble_len_in_symb = pb_params->preamble_len_in_symb;
    native_params->invert_iq_is_on      = pb_params->invert_iq_is_on ? 1 : 0;  // bool -> uint8_t conversion
    native_params->crc_is_on            = pb_params->crc_is_on ? 1 : 0;        // bool -> uint8_t conversion
    native_params->rx_timeout_ms        = pb_params->rx_timeout_ms;
    native_params->symb_nb_timeout      = ( uint8_t ) pb_params->symb_nb_timeout;
    native_params->max_rx_size          = pb_params->max_rx_size;
    native_params->tx_size              = pb_params->tx_size;

    // Convert RTToF params
    rac_convert_rttof_from_pb( &pb_params->rttof, &native_params->rttof );

    // Convert enum types - critical for time-on-air calculations
    native_params->sf          = convert_pb_sf_to_native( pb_params->sf );
    native_params->bw          = convert_pb_bw_to_native( pb_params->bw );
    native_params->cr          = convert_pb_cr_to_native( pb_params->cr );
    native_params->header_type = convert_pb_header_type_to_native( pb_params->header_type );
    native_params->sync_word   = convert_pb_sync_word_to_native( pb_params->sync_word );
}

// Enum conversion functions
static ral_lora_sf_t convert_pb_sf_to_native( lora_spreading_factor_pb_t pb_sf )
{
    switch( pb_sf )
    {
    case lora_spreading_factor_pb_t_SF5_PB:
        return RAL_LORA_SF5;
    case lora_spreading_factor_pb_t_SF6_PB:
        return RAL_LORA_SF6;
    case lora_spreading_factor_pb_t_SF7_PB:
        return RAL_LORA_SF7;
    case lora_spreading_factor_pb_t_SF8_PB:
        return RAL_LORA_SF8;
    case lora_spreading_factor_pb_t_SF9_PB:
        return RAL_LORA_SF9;
    case lora_spreading_factor_pb_t_SF10_PB:
        return RAL_LORA_SF10;
    case lora_spreading_factor_pb_t_SF11_PB:
        return RAL_LORA_SF11;
    case lora_spreading_factor_pb_t_SF12_PB:
        return RAL_LORA_SF12;
    default:
        return RAL_LORA_SF7;  // Default to SF7
    }
}

static ral_lora_bw_t convert_pb_bw_to_native( lora_bandwidth_pb_t pb_bw )
{
    switch( pb_bw )
    {
    case lora_bandwidth_pb_t_BW_7_8_KHZ_PB:
        return RAL_LORA_BW_007_KHZ;
    case lora_bandwidth_pb_t_BW_10_4_KHZ_PB:
        return RAL_LORA_BW_010_KHZ;
    case lora_bandwidth_pb_t_BW_15_6_KHZ_PB:
        return RAL_LORA_BW_015_KHZ;
    case lora_bandwidth_pb_t_BW_20_8_KHZ_PB:
        return RAL_LORA_BW_020_KHZ;
    case lora_bandwidth_pb_t_BW_31_25_KHZ_PB:
        return RAL_LORA_BW_031_KHZ;
    case lora_bandwidth_pb_t_BW_41_7_KHZ_PB:
        return RAL_LORA_BW_041_KHZ;
    case lora_bandwidth_pb_t_BW_62_5_KHZ_PB:
        return RAL_LORA_BW_062_KHZ;
    case lora_bandwidth_pb_t_BW_125_KHZ_PB:
        return RAL_LORA_BW_125_KHZ;
    case lora_bandwidth_pb_t_BW_250_KHZ_PB:
        return RAL_LORA_BW_250_KHZ;
    case lora_bandwidth_pb_t_BW_500_KHZ_PB:
        return RAL_LORA_BW_500_KHZ;
    default:
        return RAL_LORA_BW_125_KHZ;  // Default to 125kHz
    }
}

static ral_lora_cr_t convert_pb_cr_to_native( lora_coding_rate_pb_t pb_cr )
{
    switch( pb_cr )
    {
    case lora_coding_rate_pb_t_CR_4_5_PB:
        return RAL_LORA_CR_4_5;
    case lora_coding_rate_pb_t_CR_4_6_PB:
        return RAL_LORA_CR_4_6;
    case lora_coding_rate_pb_t_CR_4_7_PB:
        return RAL_LORA_CR_4_7;
    case lora_coding_rate_pb_t_CR_4_8_PB:
        return RAL_LORA_CR_4_8;
    default:
        return RAL_LORA_CR_4_5;  // Default to 4/5
    }
}

static ral_lora_pkt_len_modes_t convert_pb_header_type_to_native( lora_packet_length_mode_pb_t pb_header )
{
    switch( pb_header )
    {
    case lora_packet_length_mode_pb_t_EXPLICIT_HEADER_PB:
        return RAL_LORA_PKT_EXPLICIT;
    case lora_packet_length_mode_pb_t_IMPLICIT_HEADER_PB:
        return RAL_LORA_PKT_IMPLICIT;
    default:
        return RAL_LORA_PKT_EXPLICIT;  // Default to explicit
    }
}

static smtc_rac_lora_syncword_t convert_pb_sync_word_to_native( lora_syncword_pb_t pb_sync )
{
    switch( pb_sync )
    {
    case lora_syncword_pb_t_LORA_PRIVATE_NETWORK_SYNCWORD_PB:
        return LORA_PRIVATE_NETWORK_SYNCWORD;
    case lora_syncword_pb_t_LORA_PUBLIC_NETWORK_SYNCWORD_PB:
        return LORA_PUBLIC_NETWORK_SYNCWORD;
    default:
        return LORA_PRIVATE_NETWORK_SYNCWORD;  // Default to private
    }
}

// Native to protobuf conversion functions
static lora_coding_rate_pb_t convert_native_cr_to_pb( ral_lora_cr_t native_cr )
{
    switch( native_cr )
    {
    case RAL_LORA_CR_4_5:
        return lora_coding_rate_pb_t_CR_4_5_PB;
    case RAL_LORA_CR_4_6:
        return lora_coding_rate_pb_t_CR_4_6_PB;
    case RAL_LORA_CR_4_7:
        return lora_coding_rate_pb_t_CR_4_7_PB;
    case RAL_LORA_CR_4_8:
        return lora_coding_rate_pb_t_CR_4_8_PB;
    default:
        return lora_coding_rate_pb_t_CR_4_5_PB;  // Default to 4/5
    }
}

static lora_packet_length_mode_pb_t convert_native_header_type_to_pb( ral_lora_pkt_len_modes_t native_header )
{
    switch( native_header )
    {
    case RAL_LORA_PKT_EXPLICIT:
        return lora_packet_length_mode_pb_t_EXPLICIT_HEADER_PB;
    case RAL_LORA_PKT_IMPLICIT:
        return lora_packet_length_mode_pb_t_IMPLICIT_HEADER_PB;
    default:
        return lora_packet_length_mode_pb_t_EXPLICIT_HEADER_PB;  // Default to explicit
    }
}

static lora_syncword_pb_t convert_native_sync_word_to_pb( smtc_rac_lora_syncword_t native_sync )
{
    switch( native_sync )
    {
    case LORA_PRIVATE_NETWORK_SYNCWORD:
        return lora_syncword_pb_t_LORA_PRIVATE_NETWORK_SYNCWORD_PB;
    case LORA_PUBLIC_NETWORK_SYNCWORD:
        return lora_syncword_pb_t_LORA_PUBLIC_NETWORK_SYNCWORD_PB;
    default:
        return lora_syncword_pb_t_LORA_PRIVATE_NETWORK_SYNCWORD_PB;  // Default to private
    }
}

/**
 * \brief Convert native rp_status_t to protobuf rp_status_pb_t
 *
 * \param [in] native_status Native rp_status_t value
 *
 * \return Corresponding protobuf rp_status_pb_t value
 */
rp_status_pb_t convert_native_rp_status_to_pb( rp_status_t native_status )
{
    switch( native_status )
    {
    case RP_STATUS_RX_CRC_ERROR:
        return rp_status_pb_t_RP_STATUS_RX_CRC_ERROR_PB;
    case RP_STATUS_CAD_POSITIVE:
        return rp_status_pb_t_RP_STATUS_CAD_POSITIVE_PB;
    case RP_STATUS_CAD_NEGATIVE:
        return rp_status_pb_t_RP_STATUS_CAD_NEGATIVE_PB;
    case RP_STATUS_TX_DONE:
        return rp_status_pb_t_RP_STATUS_TX_DONE_PB;
    case RP_STATUS_RX_PACKET:
        return rp_status_pb_t_RP_STATUS_RX_PACKET_PB;
    case RP_STATUS_RX_TIMEOUT:
        return rp_status_pb_t_RP_STATUS_RX_TIMEOUT_PB;
    case RP_STATUS_LBT_FREE_CHANNEL:
        return rp_status_pb_t_RP_STATUS_LBT_FREE_CHANNEL_PB;
    case RP_STATUS_LBT_BUSY_CHANNEL:
        return rp_status_pb_t_RP_STATUS_LBT_BUSY_CHANNEL_PB;
    case RP_STATUS_WIFI_SCAN_DONE:
        return rp_status_pb_t_RP_STATUS_WIFI_SCAN_DONE_PB;
    case RP_STATUS_GNSS_SCAN_DONE:
        return rp_status_pb_t_RP_STATUS_GNSS_SCAN_DONE_PB;
    case RP_STATUS_TASK_ABORTED:
        return rp_status_pb_t_RP_STATUS_TASK_ABORTED_PB;
    case RP_STATUS_TASK_INIT:
        return rp_status_pb_t_RP_STATUS_TASK_INIT_PB;
    case RP_STATUS_LR_FHSS_HOP:
        return rp_status_pb_t_RP_STATUS_LR_FHSS_HOP_PB;
    case RP_STATUS_RTTOF_REQ_DISCARDED:
        return rp_status_pb_t_RP_STATUS_RTTOF_REQ_DISCARDED_PB;
    case RP_STATUS_RTTOF_RESP_DONE:
        return rp_status_pb_t_RP_STATUS_RTTOF_RESP_DONE_PB;
    case RP_STATUS_RTTOF_EXCH_VALID:
        return rp_status_pb_t_RP_STATUS_RTTOF_EXCH_VALID_PB;
    case RP_STATUS_RTTOF_TIMEOUT:
        return rp_status_pb_t_RP_STATUS_RTTOF_TIMEOUT_PB;
    default:
        return rp_status_pb_t_RP_STATUS_TASK_INIT_PB;
    }
}

bool rac_convert_data_result_to_pb( const smtc_rac_data_result_t* native_result, smtc_rac_data_result_pb_t* pb_result )
{
    if( !native_result || !pb_result )
    {
        return false;
    }

    // Copy radio results
    pb_result->rx_size                  = native_result->rx_size;
    pb_result->rssi_result              = native_result->rssi_result;
    pb_result->snr_result               = native_result->snr_result;
    pb_result->radio_end_timestamp_ms   = native_result->radio_end_timestamp_ms;
    pb_result->radio_start_timestamp_ms = native_result->radio_start_timestamp_ms;

    // Convert ranging result (now a direct structure, not a pointer)
    pb_result->ranging_result.valid      = true;  // Always present as direct structure
    pb_result->ranging_result.distance_m = ( float ) native_result->ranging_result.distance_m;
    pb_result->ranging_result.rssi       = ( float ) native_result->ranging_result.rssi;
    pb_result->ranging_result.timestamp  = 0;  // Not available in native structure

    // Copy RX payload if present (requires access to the context for the buffer)
    // This will be handled by the caller with the full context
    pb_result->rx_payload_buffer.size = 0;  // Initialize as empty

    return true;
}

/**
 * \brief Copy RX payload from native buffer to protobuf result
 *
 * This function copies the RX payload from the native buffer to the protobuf result
 * if there is actual received data (rx_size > 0).
 *
 * \param [in] rx_payload_buffer Native RX payload buffer
 * \param [in] rx_size Size of received payload data
 * \param [out] pb_result Protobuf result structure to populate with RX payload
 *
 * \return true if copy successful, false otherwise
 */
bool rac_copy_rx_payload_to_result( const uint8_t* rx_payload_buffer, uint32_t rx_size,
                                    smtc_rac_data_result_pb_t* pb_result )
{
    if( !pb_result )
    {
        return false;
    }

    // Only copy if we have received data
    if( rx_size > 0 )
    {
        // Check buffer size limits
        if( rx_size > 255 )
        {
            return false;  // Payload too large for protobuf
        }

        // Check if we have a valid buffer
        if( !rx_payload_buffer )
        {
            return false;  // No payload buffer provided
        }

        // Copy the RX payload
        memcpy( pb_result->rx_payload_buffer.bytes, rx_payload_buffer, rx_size );
        pb_result->rx_payload_buffer.size = rx_size;
    }
    else
    {
        // No RX data
        pb_result->rx_payload_buffer.size = 0;
    }

    return true;
}

bool rac_convert_data_buffer_setup_from_pb( const smtc_rac_data_buffer_setup_pb_t* pb_setup,
                                            smtc_rac_data_buffer_setup_t*          native_setup )
{
    if( !pb_setup || !native_setup )
    {
        return false;
    }

    // Copy TX payload buffer if present - use existing TX payload buffer
    if( pb_setup->tx_payload_buffer.size > 0 )
    {
        if( !native_setup->tx_payload_buffer )
        {
            return false;  // No TX payload buffer pre-allocated
        }
        // Validate payload size using the bytes field size
        if( pb_setup->tx_payload_buffer.size > 255 )
        {
            return false;  // Buffer too large for existing buffer
        }
        memcpy( native_setup->tx_payload_buffer, pb_setup->tx_payload_buffer.bytes, pb_setup->tx_payload_buffer.size );
        // Update native structure size field
        // native_setup->size_of_tx_payload_buffer = pb_setup->tx_payload_buffer.size;
    }

    // Note: RX payload buffer is no longer in data_buffer_setup (moved to data_result)

    return true;
}

bool rac_convert_data_result_from_pb( const smtc_rac_data_result_pb_t* pb_result,
                                      smtc_rac_data_result_t*          native_result )
{
    if( !pb_result || !native_result )
    {
        return false;
    }

    // Copy radio results
    native_result->rx_size     = pb_result->rx_size;
    native_result->rssi_result = pb_result->rssi_result;
    native_result->snr_result  = pb_result->snr_result;

    // Copy timestamp values (direct values, no longer pointers)
    native_result->radio_end_timestamp_ms   = pb_result->radio_end_timestamp_ms;
    native_result->radio_start_timestamp_ms = pb_result->radio_start_timestamp_ms;

    // Copy ranging result (now a direct structure, not a pointer)
    if( pb_result->ranging_result.valid )
    {
        // Convert protobuf ranging result to native structure
        native_result->ranging_result.raw_distance = 0;  // Not available in protobuf version
        native_result->ranging_result.distance_m = ( int32_t ) pb_result->ranging_result.distance_m;  // float → int32_t
        native_result->ranging_result.rssi       = ( int8_t ) pb_result->ranging_result.rssi;         // float → int8_t
    }

    return true;
}

void rac_convert_scheduler_config_to_pb( const smtc_rac_scheduler_config_t* native_config,
                                         rac_scheduler_config_pb_t*         pb_config )
{
    if( !native_config || !pb_config )
    {
        return;
    }

    pb_config->start_time_ms    = native_config->start_time_ms;
    pb_config->scheduling       = rac_convert_scheduling_to_pb( native_config->scheduling );
    pb_config->duration_time_ms = native_config->duration_time_ms;

    // Note: callback functions are not serialized (runtime-only)
}

bool rac_convert_scheduler_config_from_pb( const rac_scheduler_config_pb_t* pb_config,
                                           smtc_rac_scheduler_config_t*     native_config )
{
    if( !pb_config || !native_config )
    {
        return false;
    }

    native_config->start_time_ms    = pb_config->start_time_ms;
    native_config->scheduling       = rac_convert_scheduling_from_pb( pb_config->scheduling );
    native_config->duration_time_ms = pb_config->duration_time_ms;

    // Note: callback functions remain as-is (runtime-only, not serialized)

    return true;
}

void rac_convert_lbt_context_to_pb( const smtc_rac_lbt_context_t* native_lbt, smtc_rac_lbt_context_pb_t* pb_lbt )
{
    if( !native_lbt || !pb_lbt )
    {
        return;
    }

    pb_lbt->lbt_enabled        = native_lbt->lbt_enabled;
    pb_lbt->listen_duration_ms = native_lbt->listen_duration_ms;
    pb_lbt->threshold_dbm      = ( int32_t ) native_lbt->threshold_dbm;
    pb_lbt->bandwidth_hz       = native_lbt->bandwidth_hz;
    pb_lbt->rssi_inst_dbm      = ( int32_t ) native_lbt->rssi_inst_dbm;
    pb_lbt->channel_busy       = native_lbt->channel_busy;
}

bool rac_convert_lbt_context_from_pb( const smtc_rac_lbt_context_pb_t* pb_lbt, smtc_rac_lbt_context_t* native_lbt )
{
    if( !pb_lbt || !native_lbt )
    {
        return false;
    }

    native_lbt->lbt_enabled        = pb_lbt->lbt_enabled;
    native_lbt->listen_duration_ms = pb_lbt->listen_duration_ms;
    native_lbt->threshold_dbm      = ( int16_t ) pb_lbt->threshold_dbm;
    native_lbt->bandwidth_hz       = pb_lbt->bandwidth_hz;
    native_lbt->rssi_inst_dbm      = ( int16_t ) pb_lbt->rssi_inst_dbm;
    native_lbt->channel_busy       = pb_lbt->channel_busy;

    return true;
}

void rac_convert_cw_context_to_pb( const smtc_rac_cw_context_t* native_cw, smtc_rac_cw_context_pb_t* pb_cw )
{
    if( !native_cw || !pb_cw )
    {
        return;
    }

    pb_cw->cw_enabled        = native_cw->cw_enabled;
    pb_cw->infinite_preamble = native_cw->infinite_preamble;
}

bool rac_convert_cw_context_from_pb( const smtc_rac_cw_context_pb_t* pb_cw, smtc_rac_cw_context_t* native_cw )
{
    if( !pb_cw || !native_cw )
    {
        return false;
    }

    native_cw->cw_enabled        = pb_cw->cw_enabled;
    native_cw->infinite_preamble = pb_cw->infinite_preamble;

    return true;
}

// ========================================
// CAD CONTEXT CONVERSION FUNCTIONS
// ========================================

bool rac_convert_cad_context_to_pb( const smtc_rac_cad_radio_params_t* native_cad, smtc_rac_cad_context_pb_t* pb_cad )
{
    if( !native_cad || !pb_cad )
    {
        return false;
    }

    // Convert CAD timeout
    pb_cad->cad_timeout_in_ms = native_cad->cad_timeout_in_ms;

    // Convert CAD exit mode
    switch (native_cad->cad_exit_mode) {
        case RAL_LORA_CAD_ONLY:
            pb_cad->cad_exit_mode = ral_lora_cad_exit_modes_pb_t_RAL_LORA_CAD_ONLY_PB;
            break;
        case RAL_LORA_CAD_RX:
            pb_cad->cad_exit_mode = ral_lora_cad_exit_modes_pb_t_RAL_LORA_CAD_RX_PB;
            break;
        case RAL_LORA_CAD_LBT:
            pb_cad->cad_exit_mode = ral_lora_cad_exit_modes_pb_t_RAL_LORA_CAD_LBT_PB;
            break;
        default:
            return false;
            break;
    }

    // Convert CAD symbol number
    switch (native_cad->cad_symb_nb) {
        case RAL_LORA_CAD_01_SYMB:
            pb_cad->cad_symb_nb = ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_01_SYMB_PB;
            break;
        case RAL_LORA_CAD_02_SYMB:
            pb_cad->cad_symb_nb = ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_02_SYMB_PB;
            break;
        case RAL_LORA_CAD_04_SYMB:
            pb_cad->cad_symb_nb = ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_04_SYMB_PB;
            break;
        case RAL_LORA_CAD_08_SYMB:
            pb_cad->cad_symb_nb = ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_08_SYMB_PB;
            break;
        case RAL_LORA_CAD_16_SYMB:
            pb_cad->cad_symb_nb = ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_16_SYMB_PB;
            break;
        default:
            return false;
            break;
    }

    // Convert timeout
    pb_cad->cad_timeout_in_ms = native_cad->cad_timeout_in_ms;

    // Convert spreading factor
    pb_cad->sf = convert_native_sf_to_pb(native_cad->sf);

    // Convert bandwidth
    pb_cad->bw = convert_native_bw_to_pb(native_cad->bw);

    // Copy other parameters
    pb_cad->rf_freq_in_hz = native_cad->rf_freq_in_hz;
    pb_cad->invert_iq_is_on = native_cad->invert_iq_is_on;

    return true;
}

bool rac_convert_cad_context_from_pb( const smtc_rac_cad_context_pb_t* pb_cad, smtc_rac_cad_radio_params_t* native_cad )
{
    if( !pb_cad || !native_cad )
    {
        return false;
    }

    // Convert CAD timeout
    native_cad->cad_timeout_in_ms = pb_cad->cad_timeout_in_ms;

    // Convert CAD exit mode
    switch (pb_cad->cad_exit_mode) {
        case ral_lora_cad_exit_modes_pb_t_RAL_LORA_CAD_ONLY_PB:
            native_cad->cad_exit_mode = RAL_LORA_CAD_ONLY;
            break;
        case ral_lora_cad_exit_modes_pb_t_RAL_LORA_CAD_RX_PB:
            native_cad->cad_exit_mode = RAL_LORA_CAD_RX;
            break;
        case ral_lora_cad_exit_modes_pb_t_RAL_LORA_CAD_LBT_PB:
            native_cad->cad_exit_mode = RAL_LORA_CAD_LBT;
            break;
        default:
            return false;
            break;
    }

    // Convert CAD symbol number
    switch (pb_cad->cad_symb_nb) {
        case ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_01_SYMB_PB:
            native_cad->cad_symb_nb = RAL_LORA_CAD_01_SYMB;
            break;
        case ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_02_SYMB_PB:
            native_cad->cad_symb_nb = RAL_LORA_CAD_02_SYMB;
            break;
        case ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_04_SYMB_PB:
            native_cad->cad_symb_nb = RAL_LORA_CAD_04_SYMB;
            break;
        case ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_08_SYMB_PB:
            native_cad->cad_symb_nb = RAL_LORA_CAD_08_SYMB;
            break;
        case ral_lora_cad_symbs_pb_t_RAL_LORA_CAD_16_SYMB_PB:
            native_cad->cad_symb_nb = RAL_LORA_CAD_16_SYMB;
            break;
        default:
            return false;
            break;
    }

    // Convert timeout
    native_cad->cad_timeout_in_ms = pb_cad->cad_timeout_in_ms;

    // Convert spreading factor
    native_cad->sf = convert_pb_sf_to_native(pb_cad->sf);

    // Convert bandwidth
    native_cad->bw = convert_pb_bw_to_native(pb_cad->bw);

    // Copy other parameters
    native_cad->rf_freq_in_hz = pb_cad->rf_freq_in_hz;
    native_cad->invert_iq_is_on = pb_cad->invert_iq_is_on;

    return true;
}

// ========================================
// MAIN CONTEXT CONVERSION FUNCTIONS
// ========================================

bool rac_convert_context_from_pb( const smtc_rac_context_pb_t* pb_context, smtc_rac_context_t* native_context )
{
    if( !pb_context || !native_context )
    {
        return false;
    }

    // Validate modulation type before conversion (only LoRa supported for now)
    if( pb_context->modulation_type != smtc_rac_modulation_type_pb_t_SMTC_RAC_MODULATION_LORA_PB )
    {
        // Unsupported modulation type - only LoRa is supported
        return false;
    }

    // Convert modulation type (validated above)
    native_context->modulation_type = rac_convert_modulation_type_from_pb( pb_context->modulation_type );

    // Convert radio parameters - direct access to .lora (no union)
    rac_convert_radio_params_from_pb( &pb_context->radio_params, &native_context->radio_params.lora );

    // Convert LBT context
    if( !rac_convert_lbt_context_from_pb( &pb_context->lbt_context, &native_context->lbt_context ) )
    {
        return false;
    }

    // Convert CW context
    if( !rac_convert_cw_context_from_pb( &pb_context->cw_context, &native_context->cw_context ) )
    {
        return false;
    }

    // Convert CAD context
    if( !rac_convert_cad_context_from_pb( &pb_context->cad_context, &native_context->cad_context ) )
    {
        return false;
    }

    // Convert data buffer setup - copy into existing buffers, don't replace pointers
    if( !rac_convert_data_buffer_setup_from_pb( &pb_context->smtc_rac_data_buffer_setup,
                                                &native_context->smtc_rac_data_buffer_setup ) )
    {
        return false;
    }

    // Convert data result
    if( !rac_convert_data_result_from_pb( &pb_context->smtc_rac_data_result, &native_context->smtc_rac_data_result ) )
    {
        return false;
    }

    // Convert scheduler config
    if( !rac_convert_scheduler_config_from_pb( &pb_context->scheduler_config, &native_context->scheduler_config ) )
    {
        return false;
    }

    return true;
}
