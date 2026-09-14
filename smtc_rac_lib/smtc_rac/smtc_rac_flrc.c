/**
 * @file      smtc_rac_flrc.c
 *
 * @brief     smtc_rac_flrc api implementation
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

#include "radio_planner.h"
#include "ral.h"
#include "ralf.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_rac_api.h"
#include "smtc_rac_lbt.h"
#include "smtc_rac_local_func.h"

// Conditional logging for FLRC module
#ifndef RAC_FLRC_LOG_ENABLE
#define RAC_FLRC_LOG_ENABLE 0
#endif

#if RAC_FLRC_LOG_ENABLE
#define RAC_LOG_APP_PREFIX "RAC-FLRC"
#include "smtc_rac_log.h"
#else
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
#include "lr20xx_regmem.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define RALF_RADIO_POINTER smtc_rac_get_rp( )->radio
#define RAL_RADIO_POINTER &( smtc_rac_get_rp( )->radio->ral )

#if defined( LR20XX )
/* ShapeCfg = "Generic TX Modulator" (Genmod), base 0xF30900 + offset 0x04.
 * See dig_top_rif_1_1.html / LR20xx datasheet. */
#define LR20XX_GENMOD_SHAPECFG_REG_ADDR ( 0x00F30904UL )
/* DblRate = bit 8: interpolate by 16 instead of 8.
 * Datasheet: "Can only be enabled for symbol rate inferior or equal to 2Mb/s". */
#define LR20XX_GENMOD_SHAPECFG_DBLRATE_MASK ( 1UL << 8 )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static rp_radio_params_t prepare_radio_params_for_flrc( smtc_rac_context_t* rac_config );
static void              smtc_rac_flrc_tx_callback( void* rp_void );
static void              smtc_rac_flrc_rx_callback( void* rp_void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_rac_return_code_t smtc_rac_flrc( uint8_t radio_access_id )
{
    smtc_modem_hal_protect_api_call( );

    smtc_rac_context_t* rac_config = smtc_rac_get_context( radio_access_id );
    if( rac_config == NULL )
    {
        RAC_LOG_ERROR( "FLRC: invalid context for radio ID %u\n", radio_access_id );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    // Validate modulation type
    if( rac_config->modulation_type != SMTC_RAC_MODULATION_FLRC )
    {
        RAC_LOG_ERROR( "FLRC: invalid modulation type for radio ID %u\n", radio_access_id );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    RAC_LOG_CONFIG( "FLRC: Starting %s for radio ID %u\n", rac_config->radio_params.flrc.is_tx ? "TX" : "RX",
                    radio_access_id );

    rp_radio_params_t rp_radio_params = prepare_radio_params_for_flrc( rac_config );

    // Compute time on air using RAL FLRC API
    uint32_t time_on_air_us = ral_get_flrc_time_on_air_in_us(
        RAL_RADIO_POINTER,
        ( rac_config->radio_params.flrc.is_tx == true ) ? &( rp_radio_params.tx.flrc.pkt_params )
                                                        : &( rp_radio_params.rx.flrc.pkt_params ),
        ( rac_config->radio_params.flrc.is_tx == true ) ? &( rp_radio_params.tx.flrc.mod_params )
                                                        : &( rp_radio_params.rx.flrc.mod_params ) );

    const rp_task_t rp_task = {
        .hook_id = radio_access_id,
        .type    = ( rac_config->radio_params.flrc.is_tx == true ) ? RP_TASK_TYPE_TX_FLRC /* reuse slot */
                                                                   : RP_TASK_TYPE_RX_FLRC /* reuse slot */,
        .state = ( rac_config->scheduler_config.scheduling == SMTC_RAC_SCHEDULED_TRANSACTION ) ? RP_TASK_STATE_SCHEDULE
                                                                                               : RP_TASK_STATE_ASAP,
        .schedule_task_low_priority = false,
        .duration_time_ms           = time_on_air_us / 1000 + ( ( time_on_air_us % 1000 != 0 ) ? 1 : 0 ),
        .start_time_ms              = rac_config->scheduler_config.start_time_ms,
        .launch_task_callbacks =
            ( rac_config->radio_params.flrc.is_tx == true ) ? smtc_rac_flrc_tx_callback : smtc_rac_flrc_rx_callback,
    };

    rac_config->scheduler_config.duration_time_ms = rp_task.duration_time_ms;

    if( rp_task_enqueue( smtc_rac_get_rp( ), &rp_task,
                         ( rac_config->radio_params.flrc.is_tx == true )
                             ? rac_config->smtc_rac_data_buffer_setup.tx_payload_buffer
                             : rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer,
                         ( rac_config->radio_params.flrc.is_tx == true ) ? rac_config->radio_params.flrc.tx_size
                                                                         : rac_config->radio_params.flrc.max_rx_size,
                         &rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        RAC_LOG_ERROR( "FLRC: Error enqueueing task for radio access ID %u\n", radio_access_id );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    RAC_LOG_DEBUG( "FLRC: Task successfully enqueued for radio ID %u, ToA: %u us\n", radio_access_id, time_on_air_us );
    smtc_modem_hal_unprotect_api_call( );
    return SMTC_RAC_SUCCESS;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static rp_radio_params_t prepare_radio_params_for_flrc( smtc_rac_context_t* rac_config )
{
    ralf_params_flrc_t flrc_param = { 0 };

    // Basic radio parameters
    flrc_param.rf_freq_in_hz = rac_config->radio_params.flrc.frequency_in_hz;

    if( rac_config->radio_params.flrc.is_tx == false )
    {
        flrc_param.rf_freq_in_hz += rac_config->radio_params.flrc.rx_frequency_offset_in_hz;
    }

    flrc_param.output_pwr_in_dbm = rac_config->radio_params.flrc.tx_power_in_dbm;

    // Modulation parameters
    flrc_param.mod_params.raw_bit_rate = rac_config->radio_params.flrc.raw_bit_rate;
    flrc_param.mod_params.cr           = rac_config->radio_params.flrc.cr;
    flrc_param.mod_params.pulse_shape  = rac_config->radio_params.flrc.pulse_shape;

    // Packet parameters
    flrc_param.pkt_params.preamble_len    = rac_config->radio_params.flrc.preamble_len;
    flrc_param.pkt_params.sync_word_len   = rac_config->radio_params.flrc.sync_word_len;
    flrc_param.pkt_params.tx_syncword     = rac_config->radio_params.flrc.tx_syncword_index;
    flrc_param.pkt_params.match_sync_word = rac_config->radio_params.flrc.match_sync_word;
    flrc_param.pkt_params.pld_is_fix      = rac_config->radio_params.flrc.pld_is_fix;

    flrc_param.pkt_params.crc_type = rac_config->radio_params.flrc.crc_type;

    // Advanced
    for( uint8_t i = 0; i < ( sizeof( flrc_param.sync_word ) / sizeof( flrc_param.sync_word[0] ) ); i++ )
    {
        flrc_param.sync_word[i] = rac_config->radio_params.flrc.sync_word[i];
    }

    flrc_param.crc_seed       = rac_config->radio_params.flrc.crc_seed;
    flrc_param.crc_polynomial = rac_config->radio_params.flrc.crc_polynomial;

    // Configure radio params holder
    rp_radio_params_t rp_radio_params;
    memset( &rp_radio_params, 0, sizeof( rp_radio_params ) );
    rp_radio_params.pkt_type = RAL_PKT_TYPE_FLRC;

    if( rac_config->radio_params.flrc.is_tx == true )
    {
        rp_radio_params.tx.flrc.is_tx = true;
        // Note: FLRC shares union with lora/gfsk/lr_fhss in radio_planner_types
        // Use flrc union member shape to carry params where needed for ToA helper
        flrc_param.pkt_params.pld_len_in_bytes    = rac_config->radio_params.flrc.tx_size;
        rp_radio_params.tx.flrc.mod_params        = *( ( ral_flrc_mod_params_t* ) &( flrc_param.mod_params ) );
        rp_radio_params.tx.flrc.pkt_params        = *( ( ral_flrc_pkt_params_t* ) &( flrc_param.pkt_params ) );
        rp_radio_params.tx.flrc.rf_freq_in_hz     = flrc_param.rf_freq_in_hz;
        rp_radio_params.tx.flrc.output_pwr_in_dbm = flrc_param.output_pwr_in_dbm;

        rp_radio_params.tx.flrc.sync_word[flrc_param.pkt_params.tx_syncword - 1] =
            &flrc_param.sync_word[flrc_param.pkt_params.tx_syncword - 1][0];

        rp_radio_params.tx.flrc.crc_seed       = flrc_param.crc_seed;
        rp_radio_params.tx.flrc.crc_polynomial = flrc_param.crc_polynomial;
    }
    else
    {
        rp_radio_params.rx.flrc.is_tx          = false;
        flrc_param.pkt_params.pld_len_in_bytes = rac_config->radio_params.flrc.max_rx_size;
        rp_radio_params.rx.flrc.mod_params     = *( ( ral_flrc_mod_params_t* ) &( flrc_param.mod_params ) );
        rp_radio_params.rx.flrc.pkt_params     = *( ( ral_flrc_pkt_params_t* ) &( flrc_param.pkt_params ) );
        rp_radio_params.rx.flrc.rf_freq_in_hz  = flrc_param.rf_freq_in_hz;
        rp_radio_params.rx.timeout_in_ms       = rac_config->radio_params.flrc.rx_timeout_ms;
        for( uint8_t i = 0;
             i < ( sizeof( rp_radio_params.rx.flrc.sync_word ) / sizeof( rp_radio_params.rx.flrc.sync_word[0] ) ); i++ )
        {
            rp_radio_params.rx.flrc.sync_word[i] = flrc_param.sync_word[i];
        }
        rp_radio_params.rx.flrc.crc_seed       = flrc_param.crc_seed;
        rp_radio_params.rx.flrc.crc_polynomial = flrc_param.crc_polynomial;
    }

    // Store the true FLRC params into the gfsk/lora placeholders is not ideal; at
    // TX/RX callback we use ralf_setup_flrc directly, so we also keep a local
    // copy on stack for staging in callbacks via rp->radio_params. We repack from
    // the lora-shaped fields in callbacks, since ralf_setup_flrc expects
    // ralf_params_flrc_t. Alternatively, radio_planner_types.h would be extended
    // to include flrc unions.

    return rp_radio_params;
}

static void smtc_rac_flrc_tx_callback( void* rp_void )
{
    radio_planner_t*    rp           = ( radio_planner_t* ) rp_void;
    uint8_t             id           = rp->radio_task_id;
    rp_radio_params_t*  radio_params = &rp->radio_params[id];
    smtc_rac_context_t* rac_config   = smtc_rac_get_context( id );

    if( rac_config->lbt_context.lbt_enabled )
    {
        smtc_rac_lbt_listen_channel( id, rac_config->radio_params.flrc.frequency_in_hz,
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

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_flrc( rp->radio, &radio_params->tx.flrc ) == RAL_STATUS_OK );

#if defined( LR20XX )
    /* DblRate only applies to FLRC and for a bit rate <= 2 Mb/s (i.e. everything except 2.08 / 2.6 Mbps).
     * Written AFTER ralf_setup_flrc, because SetFlrcModulationParams rewrites ShapeCfg.
     * Masked read-modify-write: only bit 8 is touched, the rest of ShapeCfg is preserved.
     * Explicitly write 0 when the bit rate is >= 2 Mb/s to stay deterministic.
     * No leak to other modulations: GFSK clears it back to 0 on set_pkt_type, and LoRa ignores ShapeCfg. */
    {
        const bool enable_dbl_rate =
            ( radio_params->tx.flrc.mod_params.raw_bit_rate < RAL_FLRC_RAW_BIT_RATE_2_080_MBPS );

        lr20xx_regmem_write_regmem32_mask( rp->radio->ral.context, LR20XX_GENMOD_SHAPECFG_REG_ADDR,
                                           LR20XX_GENMOD_SHAPECFG_DBLRATE_MASK,
                                           enable_dbl_rate ? LR20XX_GENMOD_SHAPECFG_DBLRATE_MASK : 0U );
    }
#endif

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE ) == RAL_STATUS_OK );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_buffer_size[id] ) == RAL_STATUS_OK );

    if( rac_config->scheduler_config.callback_pre_radio_transaction != NULL )
    {
        rac_config->scheduler_config.callback_pre_radio_transaction( );
    }

    rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    if( rac_config->scheduler_config.scheduling == SMTC_RAC_SCHEDULED_TRANSACTION )
    {
        while( ( int32_t ) ( rp->tasks[id].start_time_ms - rac_config->smtc_rac_data_result.radio_start_timestamp_ms ) > 0 )
        {
            rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
        }
    }

    if( rac_config->lbt_context.lbt_enabled == false )
    {
        smtc_modem_hal_start_radio_tcxo( );
    }
    smtc_modem_hal_set_ant_switch( true );
    // Need in 2.4GHz at low datarate to avoid crc errors
    ral_set_fs( &( rp->radio->ral ) );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );

    RAC_LOG_TX( "FLRC Tx callback - Freq:%lu Hz, Power:%d dBm, BR_BW:%s Hz, length:%u\n",
                flrc_burst_rp_radio_params.tx.flrc.rf_freq_in_hz, flrc_burst_rp_radio_params.tx.flrc.output_pwr_in_dbm,
                ral_flrc_raw_bit_rate_to_str( flrc_burst_rp_radio_params.tx.flrc.mod_params.raw_bit_rate ),
                rp->payload_buffer_size[id] );
}

static void smtc_rac_flrc_rx_callback( void* rp_void )
{
    radio_planner_t*    rp           = ( radio_planner_t* ) rp_void;
    uint8_t             id           = rp->radio_task_id;
    rp_radio_params_t*  radio_params = &rp->radio_params[id];
    smtc_rac_context_t* rac_config   = smtc_rac_get_context( id );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_flrc( rp->radio, &radio_params->rx.flrc ) == RAL_STATUS_OK );

#if defined( LR20XX )
    // Disable IF correlator if the frequency offset was measured with à LoRa Rx

    if( rac_config->radio_params.flrc.rx_frequency_offset_in_hz != 0 )
    {
        uint32_t value = 0x000000;
        lr20xx_regmem_write_regmem32( rp->radio->ral.context, 0x00F30D18, &value, 1 );
    }
#endif

    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                         RAL_IRQ_RX_CRC_ERROR | RAL_IRQ_RX_LEN_ERROR ) ==
        RAL_STATUS_OK );

    if( rac_config->scheduler_config.callback_pre_radio_transaction != NULL )
    {
        rac_config->scheduler_config.callback_pre_radio_transaction( );
    }

    rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    if( rac_config->scheduler_config.scheduling == SMTC_RAC_SCHEDULED_TRANSACTION )
    {
        while( ( int32_t ) ( rp->tasks[id].start_time_ms - rac_config->smtc_rac_data_result.radio_start_timestamp_ms ) >
               0 )
        {
            rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
        }
    }

    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_rx( &( rp->radio->ral ), rac_config->radio_params.flrc.rx_timeout_ms ) == RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );

    RAC_LOG_RX( "FLRC Rx callback - Freq:%lu Hz, BR_BW:%s Hz\n", flrc_burst_rp_radio_params.rx.flrc.rf_freq_in_hz,
                ral_flrc_raw_bit_rate_to_str( flrc_burst_rp_radio_params.rx.flrc.mod_params.raw_bit_rate ) );
}
