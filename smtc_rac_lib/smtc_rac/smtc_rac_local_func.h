/**
 * @file      smtc_rac_local_func.h
 *
 * @brief     Internal (local) function declarations for SMTC radio planner tasks.
 *
 * This header contains the prototypes for internal functions used by the
 * SMTC RAC library to enqueue and manage radio tasks (LoRa, FSK, LR-FHSS, FLRC).
 * These functions are not part of the public API and are intended for use
 * within the RAC implementation.
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

#ifndef SMTC_RAC_LOCAL_FUNC_H
#define SMTC_RAC_LOCAL_FUNC_H

#define SMTC_RAC_CHECK_RADIO_ACCESS_ID( id )                           \
    do                                                                 \
    {                                                                  \
        if( ( id ) >= RP_HOOK_ID_MAX )                                 \
        {                                                              \
            SMTC_MODEM_HAL_PANIC( "radio_access_id is out of range" ); \
        }                                                              \
    } while( 0 )

#include "smtc_rac_api.h"
/*!
 * \brief Enqueue a LoRa or ranging task for execution by the radio planner.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_lora( uint8_t radio_access_id );

/*!
 * \brief Enqueue a FSK task for execution by the radio planner.
 *
 * This function configures and schedules a FSK radio task using the context
 * previously configured via smtc_rac_get_context(). The FSK parameters should
 * be configured in the context's radio_params.fsk field.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_fsk( uint8_t radio_access_id );

/*!
 * \brief Enqueue a LR-FHSS transmission task for execution by the radio
 * planner.
 *
 * This function configures and schedules a LR-FHSS transmission task using the
 * context previously configured via smtc_rac_get_context(). The LR-FHSS
 * parameters should be configured in the context's radio_params.lrfhss field.
 * Note: LR-FHSS is transmission-only. Reception is not supported.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_lrfhss( uint8_t radio_access_id );

/*!
 * \brief Enqueue a FLRC task for execution by the radio planner.
 *
 * This function configures and schedules a FLRC radio task using the context
 * previously configured via smtc_rac_get_context(). The FLRC parameters
 * should be configured in the context's radio_params.flrc field.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_flrc( uint8_t radio_access_id );

#endif
