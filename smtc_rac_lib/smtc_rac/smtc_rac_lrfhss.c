/**
 * @file      smtc_rac_lrfhss.c
 *
 * @brief     smtc_rac_lrfhss api implementation
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
 *       notice, this list of conditions and the disclaimer in the
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <string.h>   // memset

#include "radio_planner.h"
#include "ral.h"
#include "ralf.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_rac_api.h"
#include "smtc_rac_lbt.h"
#include "smtc_rac_local_func.h"

// Conditional logging for LR-FHSS module
#if RAC_LRFHSS_LOG_ENABLE
#define RAC_LOG_APP_PREFIX "RAC-LRFHSS"
#include "smtc_rac_log.h"
#else
// Disabled logging - all macros become no-ops
#define RAC_LOG_ERROR( ... ) \
    do                       \
    {                        \
    } while( 0 )
#define RAC_LOG_WARN( ... ) \
    do                      \
    {                       \
    } while( 0 )
#define RAC_LOG_INFO( ... ) \
    do                      \
    {                       \
    } while( 0 )
#define RAC_LOG_DEBUG( ... ) \
    do                       \
    {                        \
    } while( 0 )
#define RAC_LOG_CONFIG( ... ) \
    do                        \
    {                         \
    } while( 0 )
#define RAC_LOG_TX( ... ) \
    do                    \
    {                     \
    } while( 0 )
#define RAC_LOG_RX( ... ) \
    do                    \
    {                     \
    } while( 0 )
#define RAC_LOG_STATS( ... ) \
    do                       \
    {                        \
    } while( 0 )
#endif

#if defined( SX128X )
#include "ralf_sx128x.h"
#elif defined( SX126X )
#include "ralf_sx126x.h"
#elif defined( LR11XX )
#include "ralf_lr11xx.h"
#elif defined( SX127X )
#include "ralf_sx127x.h"
#elif defined( LR20XX )
#include "ralf_lr20xx.h"
#endif
#define SMTC_RAC_LRFHSS_DEFAULT_SYNC_WORD { 0x2C, 0x0F, 0x79, 0x95 }
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define RALF_RADIO_POINTER smtc_rac_get_rp( )->radio
#define RAL_RADIO_POINTER &( smtc_rac_get_rp( )->radio->ral )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

// Default LR-FHSS sync word
static const uint8_t default_lrfhss_sync_word[] = SMTC_RAC_LRFHSS_DEFAULT_SYNC_WORD;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static rp_radio_params_t prepare_radio_params_for_lrfhss( smtc_rac_context_t* rac_config );
static void              smtc_rac_lrfhss_tx_callback( void* rp_void );
static uint8_t           get_lr_fhss_header_count( lr_fhss_v1_cr_t coding_rate );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_rac_return_code_t smtc_rac_lrfhss( uint8_t radio_access_id )
{
    /**
     * @brief [TODO] , a function will be introduced to check all input parameters inside `smtc_rac_context_t`.
     *
     * This function will validate the compatibility of radio parameters and return in case of any incompatible
     * configuration.
     */

    smtc_modem_hal_protect_api_call( );
    smtc_rac_context_t* rac_config = smtc_rac_get_context( radio_access_id );

    RAC_LOG_CONFIG( "LR-FHSS: Starting transmission for radio ID %d\n", radio_access_id );

    // Set modulation type to LR-FHSS
    rac_config->modulation_type = SMTC_RAC_MODULATION_LRFHSS;

    // LR-FHSS is transmission only
    if( rac_config->radio_params.lrfhss.is_tx != true )
    {
        RAC_LOG_ERROR( "LR-FHSS: Reception not supported - LR-FHSS is transmission only\n" );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_NOT_SUPPORTED;
    }

    RAC_LOG_TX( "LR-FHSS: Configuring TX - Freq:%lu Hz, Power:%d dBm, Payload:%d bytes\n",
                rac_config->radio_params.lrfhss.frequency_in_hz, rac_config->radio_params.lrfhss.tx_power_in_dbm,
                rac_config->radio_params.lrfhss.tx_size );

    rp_radio_params_t rp_radio_params = prepare_radio_params_for_lrfhss( rac_config );

    // Calculate time on air for LR-FHSS
    uint32_t time_on_air_ms = 0;
    if( ral_lr_fhss_get_time_on_air_in_ms( RAL_RADIO_POINTER, &( rp_radio_params.tx.lr_fhss.ral_lr_fhss_params ),
                                           rac_config->radio_params.lrfhss.tx_size, &time_on_air_ms ) != RAL_STATUS_OK )
    {
        RAC_LOG_ERROR( "LR-FHSS: Failed to calculate time on air\n" );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    const rp_task_t rp_task = {
        .hook_id = radio_access_id,
        .type    = RP_TASK_TYPE_TX_LR_FHSS,
        .state = ( rac_config->scheduler_config.scheduling == SMTC_RAC_SCHEDULED_TRANSACTION ) ? RP_TASK_STATE_SCHEDULE
                                                                                               : RP_TASK_STATE_ASAP,

        .schedule_task_low_priority = false,
        .duration_time_ms           = time_on_air_ms,
        .start_time_ms              = rac_config->scheduler_config.start_time_ms,
        .launch_task_callbacks      = smtc_rac_lrfhss_tx_callback,
    };

    if( rp_task_enqueue( smtc_rac_get_rp( ), &rp_task, rac_config->smtc_rac_data_buffer_setup.tx_payload_buffer,
                         rac_config->radio_params.lrfhss.tx_size, &rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        RAC_LOG_ERROR( "LR-FHSS: Error enqueueing task for radio access ID %d\n", radio_access_id );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    RAC_LOG_DEBUG( "LR-FHSS: Task successfully enqueued for radio ID %d, ToA: %lu ms\n", radio_access_id,
                   time_on_air_ms );
    smtc_modem_hal_unprotect_api_call( );
    return SMTC_RAC_SUCCESS;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static rp_radio_params_t prepare_radio_params_for_lrfhss( smtc_rac_context_t* rac_config )
{
    ralf_params_lr_fhss_t lr_fhss_param = { 0 };
    memset( &lr_fhss_param, 0, sizeof( ralf_params_lr_fhss_t ) );

    // Basic radio parameters
    lr_fhss_param.output_pwr_in_dbm = rac_config->radio_params.lrfhss.tx_power_in_dbm;

    // LR-FHSS modulation parameters
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488;
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.cr              = rac_config->radio_params.lrfhss.coding_rate;
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.grid            = rac_config->radio_params.lrfhss.grid;
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.enable_hopping  = rac_config->radio_params.lrfhss.enable_hopping;
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.bw              = rac_config->radio_params.lrfhss.bandwidth;

    // Calculate header count from coding rate
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.header_count =
        get_lr_fhss_header_count( rac_config->radio_params.lrfhss.coding_rate );

    // Use provided sync word or default
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.sync_word = ( rac_config->radio_params.lrfhss.sync_word != NULL )
                                                                    ? rac_config->radio_params.lrfhss.sync_word
                                                                    : default_lrfhss_sync_word;

    // Center frequency and device offset
    lr_fhss_param.ral_lr_fhss_params.center_frequency_in_hz = rac_config->radio_params.lrfhss.frequency_in_hz;
    lr_fhss_param.ral_lr_fhss_params.device_offset          = rac_config->radio_params.lrfhss.device_offset;

    // Generate random hop sequence ID if not provided
    if( rac_config->radio_params.lrfhss.hop_sequence_id == 0 )
    {
        unsigned int nb_max_hop_sequence = 0;
        if( ral_lr_fhss_get_hop_sequence_count( RAL_RADIO_POINTER, &( lr_fhss_param.ral_lr_fhss_params ),
                                                &nb_max_hop_sequence ) == RAL_STATUS_OK )
        {
            lr_fhss_param.hop_sequence_id =
                smtc_modem_hal_get_random_nb_in_range( 0, ( uint32_t ) nb_max_hop_sequence - 1 );
        }
        else
        {
            lr_fhss_param.hop_sequence_id = 0;  // Fallback
        }
    }
    else
    {
        lr_fhss_param.hop_sequence_id = rac_config->radio_params.lrfhss.hop_sequence_id;
    }

    // Configure radio parameters
    rp_radio_params_t rp_radio_params = { 0 };
    rp_radio_params.pkt_type          = RAL_PKT_TYPE_LORA;  // LR-FHSS uses LoRa packet type with special parameters
    rp_radio_params.tx.lr_fhss        = lr_fhss_param;

    return rp_radio_params;
}

static void smtc_rac_lrfhss_tx_callback( void* rp_void )
{
    radio_planner_t*    rp           = ( radio_planner_t* ) rp_void;
    uint8_t             id           = rp->radio_task_id;
    rp_radio_params_t*  radio_params = &rp->radio_params[id];
    smtc_rac_context_t* rac_config   = smtc_rac_get_context( id );
    if( rac_config->lbt_context.lbt_enabled )
    {
        smtc_rac_lbt_listen_channel( id, radio_params->tx.lora.rf_freq_in_hz,
                                     rac_config->lbt_context.listen_duration_ms, rac_config->lbt_context.threshold_dbm,
                                     rac_config->lbt_context.bandwidth_hz );

        if( rp->status[id] == RP_STATUS_LBT_BUSY_CHANNEL )
        {
            RAC_LOG_INFO( "LBT: Channel is busy, aborting transmission" );
            rp_radio_irq_callback( rp );

            return;
        }
        else if( rp->status[id] == RP_STATUS_LBT_FREE_CHANNEL )
        {
            RAC_LOG_INFO( "LBT: Channel is free, continuing transmission" );
        }
    }

    // Note: LR-FHSS setup is handled by the radio planner in this example
    // In a full implementation, this would call the appropriate LR-FHSS setup function
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_lr_fhss_init( &( rp->radio->ral ), &rp->radio_params[id].tx.lr_fhss.ral_lr_fhss_params ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_tx_cfg( &( rp->radio->ral ), rp->radio_params[id].tx.lr_fhss.output_pwr_in_dbm,
                        rp->radio_params[id].tx.lr_fhss.ral_lr_fhss_params.center_frequency_in_hz ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE | RAL_IRQ_LR_FHSS_HOP ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_lr_fhss_build_frame( &( rp->radio->ral ), &rp->radio_params[id].tx.lr_fhss.ral_lr_fhss_params,
                                 ( ral_lr_fhss_memory_state_t ) rp->radio_params[id].lr_fhss_state,
                                 rp->radio_params[id].tx.lr_fhss.hop_sequence_id, rp->payload[id],
                                 rp->payload_buffer_size[id] ) == RAL_STATUS_OK );
    // Wait the exact expected time (ie target - tcxo startup delay)
    while( ( int32_t ) ( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
        // Do nothing
    }
    // At this time only tcxo startup delay is remaining
    if( rac_config->lbt_context.lbt_enabled == false )
    {
        smtc_modem_hal_start_radio_tcxo( );
    }
    smtc_modem_hal_set_ant_switch( true );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );

    RAC_LOG_TX(
        "LR-FHSS Tx callback - Freq:%lu Hz, Power:%d dBm, CR:%u, BW:%u, Grid:%u, Hopping:%s, HopID:%u, length:%u\n",
        radio_params->tx.lr_fhss.ral_lr_fhss_params.center_frequency_in_hz, radio_params->tx.lr_fhss.output_pwr_in_dbm,
        radio_params->tx.lr_fhss.ral_lr_fhss_params.lr_fhss_params.cr,
        radio_params->tx.lr_fhss.ral_lr_fhss_params.lr_fhss_params.bw,
        radio_params->tx.lr_fhss.ral_lr_fhss_params.lr_fhss_params.grid,
        radio_params->tx.lr_fhss.ral_lr_fhss_params.lr_fhss_params.enable_hopping ? "ON" : "OFF",
        radio_params->tx.lr_fhss.hop_sequence_id, rp->payload_buffer_size[id] );
}

static uint8_t get_lr_fhss_header_count( lr_fhss_v1_cr_t coding_rate )
{
    switch( coding_rate )
    {
    case LR_FHSS_V1_CR_1_3:
        return 3;  // LR_FHSS_HC_3
    case LR_FHSS_V1_CR_2_3:
        return 2;  // LR_FHSS_HC_2
    default:
        return 3;  // Default to 3 for safety
    }
}
