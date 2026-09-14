/**
 * @file      rac_context_converter.h
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

#ifndef RAC_CONTEXT_CONVERTER_H
#define RAC_CONTEXT_CONVERTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <smtc_rac_api.h>                                 // Original API structures
#include "serialization/generated/smtc_rac_context.pb.h"  // Generated protobuf structures

#if defined( USE_FLRC_PROTOCOL )
#include "smtc_flrp_api.h"        // FLRP protocol API structures
#include "smtc_flrp_api_tests.h"  // smtc_flrp_flrc_advanced_radio_config_t
#endif

// ========================================
// ENUM CONVERSION FUNCTIONS
// ========================================

/**
 * \brief Convert native priority to protobuf priority
 */
smtc_rac_priority_pb_t rac_convert_priority_to_pb( smtc_rac_priority_t native_priority );

/**
 * \brief Convert protobuf priority to native priority (with error checking)
 */
bool rac_convert_priority_from_pb( smtc_rac_priority_pb_t pb_priority, smtc_rac_priority_t* output );

/**
 * \brief Convert native modulation type to protobuf modulation type
 */
smtc_rac_modulation_type_pb_t rac_convert_modulation_type_to_pb( smtc_rac_modulation_type_t native_modulation );

/**
 * \brief Convert protobuf modulation type to native modulation type
 */
smtc_rac_modulation_type_t rac_convert_modulation_type_from_pb( smtc_rac_modulation_type_pb_t pb_modulation );

/**
 * \brief Convert native return code to protobuf return code
 */
smtc_rac_return_code_pb_t rac_convert_return_code_to_pb( smtc_rac_return_code_t native_code );

/**
 * \brief Convert protobuf return code to native return code
 */
smtc_rac_return_code_t rac_convert_return_code_from_pb( smtc_rac_return_code_pb_t pb_code );

/**
 * \brief Convert native scheduling to protobuf scheduling
 */
smtc_rac_scheduling_pb_t rac_convert_scheduling_to_pb( smtc_rac_scheduling_t native_scheduling );

/**
 * \brief Convert protobuf scheduling to native scheduling
 */
smtc_rac_scheduling_t rac_convert_scheduling_from_pb( smtc_rac_scheduling_pb_t pb_scheduling );

// ========================================
// STRUCTURE CONVERSION FUNCTIONS
// ========================================

/**
 * \brief Convert native RTToF params to protobuf RTToF params
 *
 * \param [in] native_rttof Native RTToF structure (from original API)
 * \param [out] pb_rttof Protobuf RTToF structure to populate
 */
void rac_convert_rttof_to_pb( const smtc_rac_rttof_params_t* native_rttof, rttof_params_pb_t* pb_rttof );

/**
 * \brief Convert protobuf RTToF params to native RTToF params
 *
 * \param [in] pb_rttof Protobuf RTToF structure
 * \param [out] native_rttof Native RTToF structure (smtc_rac_rttof_params_t)
 */
void rac_convert_rttof_from_pb( const rttof_params_pb_t* pb_rttof, smtc_rac_rttof_params_t* native_rttof );

/**
 * \brief Convert native radio params to protobuf radio params
 *
 * \param [in] native_params Native radio params structure
 * \param [out] pb_params Protobuf radio params structure to populate
 */
void rac_convert_radio_params_to_pb( const smtc_rac_radio_lora_params_t* native_params,
                                     rac_radio_lora_params_pb_t*         pb_params );

/**
 * \brief Convert protobuf LoRa radio params to native radio params
 *
 * \param [in] pb_params Protobuf radio params structure
 * \param [out] native_params Native radio params structure to populate
 */
void rac_convert_radio_params_from_pb( const rac_radio_lora_params_pb_t* pb_params,
                                       smtc_rac_radio_lora_params_t*     native_params );

#if defined( USE_FLRC_PROTOCOL )
/**
 * \brief Convert native FLRC radio params to protobuf FLRC radio params
 *
 * \param [in] native_params Native FLRC radio params structure
 * \param [out] pb_params Protobuf FLRC radio params structure to populate
 */
void rac_convert_flrc_radio_params_to_pb( const smtc_rac_radio_flrc_params_t* native_params,
                                          rac_radio_flrc_params_pb_t*         pb_params );

/**
 * \brief Convert protobuf FLRC radio params to native FLRC radio params
 *
 * \param [in] pb_params Protobuf FLRC radio params structure
 * \param [out] native_params Native FLRC radio params structure to populate
 * \param [out] sync_word_buffers Array of 3 buffers to store the sync words (each must be at least 4 bytes)
 */
void rac_convert_flrc_radio_params_from_pb( const rac_radio_flrc_params_pb_t* pb_params,
                                            smtc_rac_radio_flrc_params_t*     native_params,
                                            uint8_t*                          sync_word_buffers[3] );

/**
 * \brief Convert protobuf CMD_SET_FLRC_PROTOCOL_PARAMS payload to native FLRP radio config
 *
 * \param [in] pb_config flrc_protocol_radio_config_pb_t (flat FLRC + timing fields)
 * \param [out] native_config Native FLRP radio config structure to populate
 */
void convert_pb_flrc_protocol_radio_config_to_native( const flrc_protocol_radio_config_pb_t* pb_config,
                                                      smtc_flrp_radio_config_t*              native_config );

/**
 * \brief Map flrc_protocol_radio_config_pb_t to smtc_flrp_flrc_advanced_radio_config_t
 */
void convert_pb_flrc_protocol_radio_config_to_native_advanced(
    const flrc_protocol_radio_config_pb_t* pb_config, smtc_flrp_flrc_advanced_radio_config_t* advanced_config );

/**
 * \brief Map flrp_flrc_advanced_radio_config_pb_t (CMD_FLRP_SET_PARAMS_ADVANCED) to
 * smtc_flrp_flrc_advanced_radio_config_t
 */
void convert_pb_flrp_flrc_advanced_radio_config_to_native( const flrp_flrc_advanced_radio_config_pb_t* pb_config,
                                                           smtc_flrp_flrc_advanced_radio_config_t* advanced_config );

/**
 * \brief Convert protobuf FLRP radio config (nested flrc / wor_rx / wor_tx) to smtc_flrp_radio_config_t
 */
void convert_pb_flrp_radio_config( const flrp_radio_config_pb_t* pb_config,
                                   smtc_flrp_radio_config_t*     flrp_radio_config );

/**
 * \brief Convert native FLRP radio config to protobuf (inverse of convert_pb_flrp_radio_config)
 *
 * \return false if native_config or pb_config is NULL
 */
bool convert_native_flrp_radio_config_to_pb( const smtc_flrp_radio_config_t* native_config,
                                             flrp_radio_config_pb_t*         pb_config );

/**
 * \brief Convert protobuf FLRP com config to smtc_flrp_com_config_t
 *
 * \return false if slave_dev_eui is not exactly 8 bytes, filter_len > 63, or enums are not recognized
 */
bool convert_pb_flrp_com_config_to_native( const flrp_com_config_pb_t* pb_config,
                                           smtc_flrp_com_config_t*     native_config );

/**
 * \brief Convert protobuf FLRP API init config to smtc_flrp_api_config_t
 *
 * \return false if dev_eui size is not SMTC_FLRP_EUI_LENGTH or freq_plan is not a known enumerator
 */
bool convert_pb_flrp_api_config_to_native( const flrp_api_config_pb_t* pb_config,
                                           smtc_flrp_api_config_t*     native_config );
#endif  // USE_FLRC_PROTOCOL

/**
 * \brief Convert protobuf data buffer setup to native data buffer setup
 *
 * \param [in] pb_setup Protobuf data buffer setup structure
 * \param [out] native_setup Native data buffer setup structure to populate (uses existing buffers)
 *
 * \return true if conversion successful, false if payload too large or no pre-allocated buffers
 */
bool rac_convert_data_buffer_setup_from_pb( const smtc_rac_data_buffer_setup_pb_t* pb_setup,
                                            smtc_rac_data_buffer_setup_t*          native_setup );

/**
 * \brief Convert native data result to protobuf data result
 *
 * \param [in] native_result Native data result structure
 * \param [out] pb_result Protobuf data result structure to populate
 *
 * \return true if conversion successful, false otherwise
 */
bool rac_convert_data_result_to_pb( const smtc_rac_data_result_t* native_result, smtc_rac_data_result_pb_t* pb_result );

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
                                    smtc_rac_data_result_pb_t* pb_result );

/**
 * \brief Convert protobuf data result to native data result
 *
 * \param [in] pb_result Protobuf data result structure
 * \param [out] native_result Native data result structure to populate
 *
 * \return true if conversion successful, false otherwise
 */
bool rac_convert_data_result_from_pb( const smtc_rac_data_result_pb_t* pb_result,
                                      smtc_rac_data_result_t*          native_result );

/**
 * \brief Convert native scheduler config to protobuf scheduler config
 *
 * \param [in] native_config Native scheduler config structure
 * \param [out] pb_config Protobuf scheduler config structure to populate
 */
void rac_convert_scheduler_config_to_pb( const smtc_rac_scheduler_config_t* native_config,
                                         rac_scheduler_config_pb_t*         pb_config );

/**
 * \brief Convert protobuf scheduler config to native scheduler config
 *
 * \param [in] pb_config Protobuf scheduler config structure
 * \param [out] native_config Native scheduler config structure to populate
 *
 * \return true if conversion successful, false otherwise
 */
bool rac_convert_scheduler_config_from_pb( const rac_scheduler_config_pb_t* pb_config,
                                           smtc_rac_scheduler_config_t*     native_config );

/**
 * \brief Convert native LBT context to protobuf LBT context
 *
 * \param [in] native_lbt Native LBT context structure
 * \param [out] pb_lbt Protobuf LBT context structure to populate
 */
void rac_convert_lbt_context_to_pb( const smtc_rac_lbt_context_t* native_lbt, smtc_rac_lbt_context_pb_t* pb_lbt );

/**
 * \brief Convert protobuf LBT context to native LBT context
 *
 * \param [in] pb_lbt Protobuf LBT context structure
 * \param [out] native_lbt Native LBT context structure to populate
 *
 * \return true if conversion successful, false otherwise
 */
bool rac_convert_lbt_context_from_pb( const smtc_rac_lbt_context_pb_t* pb_lbt, smtc_rac_lbt_context_t* native_lbt );

/**
 * \brief Convert native CW context to protobuf CW context
 *
 * \param [in] native_cw Native CW context structure
 * \param [out] pb_cw Protobuf CW context structure to populate
 */
void rac_convert_cw_context_to_pb( const smtc_rac_cw_context_t* native_cw, smtc_rac_cw_context_pb_t* pb_cw );

/**
 * \brief Convert protobuf CW context to native CW context
 *
 * \param [in] pb_cw Protobuf CW context structure
 * \param [out] native_cw Native CW context structure to populate
 *
 * \return true if conversion successful, false otherwise
 */
bool rac_convert_cw_context_from_pb( const smtc_rac_cw_context_pb_t* pb_cw, smtc_rac_cw_context_t* native_cw );

// ========================================
// MAIN CONTEXT CONVERSION FUNCTIONS
// ========================================

/**
 * \brief Convert protobuf RAC context to native RAC context
 *
 * This is the main conversion function for deserialization.
 *
 * \param [in] pb_context Protobuf RAC context structure
 * \param [out] native_context Native RAC context structure to populate (uses existing buffers)
 *
 * \return true if conversion successful, false if error occurred
 */
bool rac_convert_context_from_pb( const smtc_rac_context_pb_t* pb_context, smtc_rac_context_t* native_context );

/**
 * \brief Convert native rp_status_t to protobuf rp_status_pb_t
 *
 * \param [in] native_status Native rp_status_t value
 *
 * \return Corresponding protobuf rp_status_pb_t value
 */
rp_status_pb_t convert_native_rp_status_to_pb( rp_status_t native_status );

/**
 * \brief Convert CAD context from native to protobuf
 *
 * \param [in] native_cad Native smtc_rac_cad_radio_params_t context
 * \param [out] pb_cad Protobuf smtc_rac_cad_context_pb_t context
 *
 * \return true if conversion successful, false otherwise
 */
bool rac_convert_cad_context_to_pb( const smtc_rac_cad_radio_params_t* native_cad, smtc_rac_cad_context_pb_t* pb_cad );

/**
 * \brief Convert CAD context from protobuf to native
 *
 * \param [in] pb_cad Protobuf smtc_rac_cad_context_pb_t context
 * \param [out] native_cad Native smtc_rac_cad_radio_params_t context
 *
 * \return true if conversion successful, false otherwise
 */
bool rac_convert_cad_context_from_pb( const smtc_rac_cad_context_pb_t* pb_cad,
                                      smtc_rac_cad_radio_params_t*     native_cad );

#ifdef __cplusplus
}
#endif

#endif  // RAC_CONTEXT_CONVERTER_H
