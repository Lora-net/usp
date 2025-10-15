/**
 * @file      smtc_rac_lbt.c
 *
 * @brief     USP/RAC LBT (Listen Before Talk) implementation
 *
 * This file provides a simplified LBT implementation for the RAC framework,
 * based on the original smtc_lbt.c but adapted to work with radio planner
 * and context management using the new LBT union in the RAC context.
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "smtc_rac_api.h"
#include "smtc_modem_hal.h"
#include "smtc_rac_lbt.h"
#include "radio_planner.h"
#include "ral.h"
#include "ralf.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_rac_local_func.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
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

uint16_t smtc_rac_lbt_listen_channel( uint8_t radio_access_id, uint32_t freq_hz, uint32_t listen_duration_ms,
                                      int16_t threshold_dbm, uint32_t bandwidth_hz )
{
    // Prepare radio parameters for RSSI measurement
    ralf_params_gfsk_t gfsk_param;
    rp_radio_params_t  radio_params;

    memset( &radio_params, 0, sizeof( rp_radio_params_t ) );
    memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

    gfsk_param.rf_freq_in_hz      = freq_hz;
    gfsk_param.pkt_params.dc_free = RAL_GFSK_DC_FREE_WHITENING;

    // Use bandwidth to configure modulation parameters
    gfsk_param.mod_params.br_in_bps    = bandwidth_hz >> 1;
    gfsk_param.mod_params.fdev_in_hz   = bandwidth_hz >> 2;
    gfsk_param.mod_params.bw_dsb_in_hz = bandwidth_hz;
    gfsk_param.mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;

    radio_params.pkt_type         = RAL_PKT_TYPE_GFSK;
    radio_params.rx.gfsk          = gfsk_param;
    radio_params.rx.timeout_in_ms = listen_duration_ms;
    radio_params.lbt_threshold    = threshold_dbm;

    uint8_t          id = radio_access_id;
    int16_t          rssi_tmp;
    radio_planner_t* rp = smtc_rac_get_rp( );
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_pkt_type( &( rp->radio->ral ), RAL_PKT_TYPE_GFSK ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rf_freq( &( rp->radio->ral ), radio_params.rx.gfsk.rf_freq_in_hz ) ==
                                     RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_gfsk_mod_params( &( rp->radio->ral ), &( radio_params.rx.gfsk.mod_params ) ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_NONE ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rx( &( rp->radio->ral ), RAL_RX_TIMEOUT_CONTINUOUS_MODE ) ==
                                     RAL_STATUS_OK );

    uint32_t carrier_sense_time = smtc_modem_hal_get_time_in_ms( );
    while( ( int32_t ) ( carrier_sense_time + LAP_OF_TIME_TO_GET_A_RSSI_VALID - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
       
    }
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
    carrier_sense_time = smtc_modem_hal_get_time_in_ms( );
    do
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_get_rssi_inst( &( rp->radio->ral ), &rssi_tmp ) == RAL_STATUS_OK );
        if( rssi_tmp >= threshold_dbm )
        {
            rp->status[id] = RP_STATUS_LBT_BUSY_CHANNEL;

            return rssi_tmp;
        }
    } while( ( int32_t ) ( carrier_sense_time + radio_params.rx.timeout_in_ms - smtc_modem_hal_get_time_in_ms( ) ) >
             0 );
    // SMTC_MODEM_HAL_TRACE_PRINTF( "channel is free : lbt rssi: %d thre= %d dBm\n", rssi_tmp, threshold_dbm );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_standby( &( rp->radio->ral ), RAL_STANDBY_CFG_RC ) == RAL_STATUS_OK );
    rp->status[id] = RP_STATUS_LBT_FREE_CHANNEL;
    return rssi_tmp;
}
