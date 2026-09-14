/**
 * @file      smtc_wor.c
 *
 * @brief     smtc_wor implementation (common for RX and TX)
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

#include "smtc_wor.h"
#include "smtc_rac.h"
#include "smtc_rac_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
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
static void smtc_wor_set_static_radio_config( smtc_wor_t* smtc_wor_obj, smtc_wor_static_radio_config_t wor_config );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_wor_init( smtc_wor_t* smtc_wor_obj, smtc_wor_static_radio_config_t wor_config, uint8_t hook_id,
                    smtc_wor_done_f wor_done_cb )
{
    memset( smtc_wor_obj, 0, sizeof( smtc_wor_t ) );

    smtc_wor_obj->smtc_wor_done_cb      = wor_done_cb;
    smtc_wor_obj->smtc_wor_radio_config = wor_config;

    smtc_wor_obj->radio_access_id = smtc_rac_open_radio( ( smtc_rac_priority_t ) hook_id );

    smtc_wor_obj->transaction = smtc_rac_get_context( smtc_wor_obj->radio_access_id );

    smtc_wor_set_static_radio_config( smtc_wor_obj, wor_config );

    smtc_rac_set_context_private( smtc_wor_obj->radio_access_id, smtc_wor_obj );
}

void enter_idle_state_and_send_wor_cb( smtc_wor_t* smtc_wor_obj, smtc_wor_status_t status )
{
    smtc_wor_obj->state = SMTC_WOR_STATE_IDLE;
    if( smtc_wor_obj->smtc_wor_done_cb != NULL )
    {
        smtc_wor_obj->smtc_wor_done_cb( status, smtc_wor_obj->rx_metrics );
    }
}

smtc_wor_status_t convert_rac_status_to_wor_status( smtc_rac_return_code_t rac_status )
{
    switch( rac_status )
    {
    case SMTC_RAC_SUCCESS:
        return SMTC_WOR_STATUS_SUCCESS;

    case SMTC_RAC_BUSY:
        return SMTC_WOR_STATUS_BUSY;

    case SMTC_RAC_INVALID_PARAMETER:
        return SMTC_WOR_STATUS_INVALID_PARAMETER;

    case SMTC_RAC_NOT_INITIALIZED:
        return SMTC_WOR_STATUS_NOT_INITIALIZED;

    case SMTC_RAC_NOT_IMPLEMENTED:
    case SMTC_RAC_NOT_SUPPORTED:
        return SMTC_WOR_STATUS_NOT_SUPPORTED;

    case SMTC_RAC_ERROR:
    case SMTC_RAC_TIMEOUT:
    default:
        return SMTC_WOR_STATUS_ERROR;
    }
}

void smtc_wor_set_dynamic_radio_config( smtc_wor_t* smtc_wor_obj, smtc_flrp_wor_radio_config_t radio_config )
{
    smtc_wor_obj->transaction->radio_params.lora.bw              = radio_config.bw;
    smtc_wor_obj->transaction->radio_params.lora.sf              = radio_config.sf;
    smtc_wor_obj->transaction->radio_params.lora.frequency_in_hz = radio_config.frequency_hz;
    smtc_wor_obj->transaction->radio_params.lora.tx_power_in_dbm = radio_config.tx_power_in_dbm;
}

uint32_t smtc_wor_get_single_symbol_time_us( ral_lora_bw_t bw, ral_lora_sf_t sf )
{
    uint32_t bw_value;

    if( sf < RAL_LORA_SF5 || sf > RAL_LORA_SF12 )
    {
        return 0;
    }

    switch( bw )
    {
    case RAL_LORA_BW_125_KHZ:
        bw_value = 125;
        break;
    case RAL_LORA_BW_200_KHZ:
        bw_value = 200;
        break;
    case RAL_LORA_BW_250_KHZ:
        bw_value = 250;
        break;
    case RAL_LORA_BW_400_KHZ:
        bw_value = 400;
        break;
    case RAL_LORA_BW_500_KHZ:
        bw_value = 500;
        break;
    case RAL_LORA_BW_800_KHZ:
        bw_value = 800;
        break;
    case RAL_LORA_BW_1000_KHZ:
        bw_value = 1000;
        break;
    default:
        return 0;
    }

    return ( uint32_t ) ( ( 1 << sf ) * 1000 / bw_value );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION ---------------------------------------------
 */

static void smtc_wor_set_static_radio_config( smtc_wor_t* smtc_wor_obj, smtc_wor_static_radio_config_t wor_config )
{
    smtc_wor_obj->transaction->modulation_type                       = SMTC_RAC_MODULATION_LORA;
    smtc_wor_obj->transaction->radio_params.lora.is_ranging_exchange = false;
    smtc_wor_obj->transaction->radio_params.lora.cr                  = wor_config.cr;
    smtc_wor_obj->transaction->radio_params.lora.header_type         = wor_config.header_type;
    smtc_wor_obj->transaction->radio_params.lora.crc_is_on           = wor_config.crc_is_on;
    smtc_wor_obj->transaction->radio_params.lora.sync_word           = wor_config.sync_word;
}
/* --- EOF ------------------------------------------------------------------ */