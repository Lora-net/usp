/*!
 * @file      smtc_flrp_utils.c
 *
 * @brief     Common SMTC FLRC functions.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2026. All rights reserved.
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

#include "smtc_flrp_utils.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static bit_mask_t* wor_exchange_phase_mask = NULL;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_flrp_bind_wor_exchange_phase_mask( bit_mask_t* mask )
{
    wor_exchange_phase_mask = mask;
}

void set_mask_bit( smtc_flrp_exchange_phase_success_bit_t bit, bool value )
{
    if( wor_exchange_phase_mask == NULL )
    {
        return;
    }

    bit_mask_t b = ( bit_mask_t ) bit;
    if( value )
    {
        *wor_exchange_phase_mask |= b;
    }
    else
    {
        *wor_exchange_phase_mask &= ( bit_mask_t ) ~b;
    }
}

bool smtc_flrp_is_frame_addressed_to_this_device( uint8_t dev_eui[SMTC_FLRP_EUI_LENGTH],
                                                  uint8_t target_dev_eui[SMTC_FLRP_EUI_LENGTH], uint8_t filter_len )
{
    bool     device_eui_find = true;
    uint64_t mask_filter     = ( UINT64_MAX << ( ( SMTC_FLRP_EUI_LENGTH * 8 ) - 1 - filter_len ) );

    for( int i = 0; i < SMTC_FLRP_EUI_LENGTH; i++ )
    {
        uint8_t mask_filter_idx = ( uint8_t ) ( mask_filter >> ( 8 * ( SMTC_FLRP_EUI_LENGTH - 1 - i ) ) );
        if( ( dev_eui[i] & mask_filter_idx ) != ( target_dev_eui[i] & mask_filter_idx ) )
        {
            device_eui_find = false;
        }
    }

    return device_eui_find;
}

/* --- EOF ------------------------------------------------------------------ */