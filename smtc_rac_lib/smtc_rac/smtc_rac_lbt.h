/**
 * @file      smtc_rac_lbt.h
 *
 * @brief     LBT (Listen Before Talk) header file
 *
 * This header file provides the interface for the simplified LBT implementation
 * for the RAC framework, based on the original smtc_lbt.h but adapted to work
 * with radio planner and context management.
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

#ifndef SMTC_RAC_LBT_H
#define SMTC_RAC_LBT_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

#define LAP_OF_TIME_TO_GET_A_RSSI_VALID 8  // duration to stabilize the radio after rx cmd in ms
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief Listen to a channel to check if it's free before transmission
 *
 * This function implements the Listen Before Talk (LBT) mechanism by monitoring
 * a specific frequency channel for a given duration to detect if the channel
 * is busy or free for transmission.
 *
 * \param [in] radio_access_id    Radio access identifier
 * \param [in] freq_hz            Frequency in Hz to listen to
 * \param [in] listen_duration_ms Duration in milliseconds to listen to the channel
 * \param [in] threshold_dbm      RSSI threshold in dBm above which the channel is considered busy
 * \param [in] bandwidth_hz       Bandwidth in Hz for the RSSI measurement
 *
 * \retval void                   Function does not return a value, but updates the radio planner status
 */
uint16_t smtc_rac_lbt_listen_channel( uint8_t radio_access_id, uint32_t freq_hz, uint32_t listen_duration_ms,
                                      int16_t threshold_dbm, uint32_t bandwidth_hz );

#endif /* SMTC_RAC_LBT_H */
