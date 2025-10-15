/**
 * @file      smtc_rac_lora.c
 *
 * @brief     smtc_rac_lora api implementation
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "radio_planner.h"
#include "ral.h"
#include "ralf.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_rac_api.h"
#include "smtc_rac_lbt.h"
#include "smtc_rac_local_func.h"

#if defined( LR20XX )
#include <lr20xx_workarounds.h>
#endif

// Conditional logging for LoRa module
#if RAC_LOG_ENABLE
#define RAC_LOG_APP_PREFIX "RAC-LORA"
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

#define RTTOF_ADDRESS_LENGTH 4

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static rp_radio_params_t prepare_radio_params_for_lora( smtc_rac_context_t* rac_config );
static void update_radio_params_for_ranging( smtc_rac_context_t* rac_config, rp_radio_params_t* radio_params );
static void smtc_rac_tx_callback( void* rp_void );

static void smtc_rac_rx_callback( void* rp_void );

smtc_rac_return_code_t smtc_rac_lora( uint8_t radio_access_id )
{
    smtc_modem_hal_protect_api_call( );

    RAC_LOG_INFO( "===== LoRa Transaction Request =====" );
    RAC_LOG_INFO( "Radio Access ID: %u", radio_access_id );

    // Get the context for this radio access
    smtc_rac_context_t* rac_config = smtc_rac_get_context( radio_access_id );

    if( rac_config == NULL )
    {
        RAC_LOG_ERROR( "Failed to get context for radio access ID %u", radio_access_id );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    // Validate context is configured for LoRa
    if( rac_config->modulation_type != SMTC_RAC_MODULATION_LORA )
    {
        RAC_LOG_WARN( "Context modulation type was %d, setting to LoRa", rac_config->modulation_type );
        rac_config->modulation_type = SMTC_RAC_MODULATION_LORA;
    }

    // Log operation type and key parameters
    RAC_LOG_INFO( "Operation: %s", rac_config->radio_params.lora.is_tx ? "TRANSMISSION" : "RECEPTION" );
    RAC_LOG_CONFIG( "Frequency: %lu Hz (%.1f MHz)", rac_config->radio_params.lora.frequency_in_hz,
                    ( float ) rac_config->radio_params.lora.frequency_in_hz / 1000000.0f );
    RAC_LOG_CONFIG( "SF: %d, BW: %d, CR: 4/%d", rac_config->radio_params.lora.sf, rac_config->radio_params.lora.bw,
                    rac_config->radio_params.lora.cr + 5 );
    if( rac_config->radio_params.lora.is_tx )
    {
        RAC_LOG_TX( "TX Power: %d dBm, Payload Size: %d bytes", rac_config->radio_params.lora.tx_power_in_dbm,
                    rac_config->radio_params.lora.tx_size );
    }
    else
    {
        RAC_LOG_RX( "RX Timeout: %lu ms", rac_config->radio_params.lora.rx_timeout_ms );
    }

    if( rac_config->radio_params.lora.is_ranging_exchange )
    {
        RAC_LOG_INFO( "Ranging exchange enabled" );
    }

    // Setup timestamps

    RAC_LOG_DEBUG( "Preparing radio parameters..." );
    rp_radio_params_t rp_radio_params = prepare_radio_params_for_lora( rac_config );

    if( rac_config->radio_params.lora.is_ranging_exchange )
    {
        RAC_LOG_DEBUG( "Updating parameters for ranging..." );
        update_radio_params_for_ranging( rac_config, &rp_radio_params );
    }

    // Calculate time on air
    uint32_t time_on_air_ms = ral_get_lora_time_on_air_in_ms(
        RAL_RADIO_POINTER,
        ( rac_config->radio_params.lora.is_tx == true ) ? &( rp_radio_params.tx.lora.pkt_params )
                                                        : &( rp_radio_params.rx.lora.pkt_params ),
        ( rac_config->radio_params.lora.is_tx == true ) ? &( rp_radio_params.tx.lora.mod_params )
                                                        : &( rp_radio_params.rx.lora.mod_params ) );

    RAC_LOG_INFO( "Estimated time on air: %lu ms", time_on_air_ms );

    const rp_task_t rp_task = {
        .hook_id = radio_access_id,
        .type    = ( rac_config->radio_params.lora.is_tx == true ) ? RP_TASK_TYPE_TX_LORA : RP_TASK_TYPE_RX_LORA,
        .state = ( rac_config->scheduler_config.scheduling == SMTC_RAC_SCHEDULED_TRANSACTION ) ? RP_TASK_STATE_SCHEDULE
                                                                                               : RP_TASK_STATE_ASAP,
        .schedule_task_low_priority = false,
        .duration_time_ms           = time_on_air_ms,
        .start_time_ms              = rac_config->scheduler_config.start_time_ms,
        .launch_task_callbacks =
            ( rac_config->radio_params.lora.is_tx == true ) ? smtc_rac_tx_callback : smtc_rac_rx_callback,
    };

    RAC_LOG_INFO( "Scheduling: %s (start time: %lu ms)",
                  ( rac_config->scheduler_config.scheduling == SMTC_RAC_SCHEDULED_TRANSACTION ) ? "SCHEDULED" : "ASAP",
                  rac_config->scheduler_config.start_time_ms );

    RAC_LOG_DEBUG( "Enqueuing task to radio planner..." );

    if( rp_task_enqueue( smtc_rac_get_rp( ), &rp_task,
                         ( rac_config->radio_params.lora.is_tx == true )
                             ? rac_config->smtc_rac_data_buffer_setup.tx_payload_buffer
                             : rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer,
                         ( rac_config->radio_params.lora.is_tx == true ) ? rac_config->radio_params.lora.tx_size
                                                                         : rac_config->radio_params.lora.max_rx_size,
                         &rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        RAC_LOG_ERROR( "Failed to enqueue task for radio access ID %u", radio_access_id );
        RAC_LOG_ERROR( "Check radio planner status and resource availability" );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    RAC_LOG_INFO( "LoRa task successfully enqueued - transaction initiated" );
    RAC_LOG_INFO( "=====================================" );
    smtc_modem_hal_unprotect_api_call( );
    return SMTC_RAC_SUCCESS;
}

static rp_radio_params_t prepare_radio_params_for_lora( smtc_rac_context_t* rac_config )
{
    ralf_params_lora_t lora_param = { 0 };
    lora_param.rf_freq_in_hz                   = rac_config->radio_params.lora.frequency_in_hz;
    lora_param.output_pwr_in_dbm               = rac_config->radio_params.lora.tx_power_in_dbm;
    lora_param.sync_word                       = ( uint8_t ) rac_config->radio_params.lora.sync_word;
    lora_param.symb_nb_timeout                 = rac_config->radio_params.lora.symb_nb_timeout;
    lora_param.pkt_params.preamble_len_in_symb = rac_config->radio_params.lora.preamble_len_in_symb;
    lora_param.pkt_params.header_type          = rac_config->radio_params.lora.header_type;

    lora_param.pkt_params.crc_is_on       = rac_config->radio_params.lora.crc_is_on;
    lora_param.pkt_params.invert_iq_is_on = rac_config->radio_params.lora.invert_iq_is_on;
    lora_param.mod_params.sf              = rac_config->radio_params.lora.sf;
    lora_param.mod_params.bw              = rac_config->radio_params.lora.bw;
    lora_param.mod_params.cr              = rac_config->radio_params.lora.cr;
    lora_param.mod_params.ldro            = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

    // config radio parameters
    rp_radio_params_t rp_radio_params = { 0 };

    rp_radio_params.pkt_type =
        ( rac_config->radio_params.lora.is_ranging_exchange == false ) ? RAL_PKT_TYPE_LORA : RAL_PKT_TYPE_RTTOF;

    if( rac_config->radio_params.lora.is_tx == true )
    {
        lora_param.pkt_params.pld_len_in_bytes = rac_config->radio_params.lora.tx_size;
        rp_radio_params.tx.lora                = lora_param;
    }
    else
    {
        lora_param.pkt_params.pld_len_in_bytes = rac_config->radio_params.lora.max_rx_size;
        rp_radio_params.rx.lora                = lora_param;
        rp_radio_params.rx.timeout_in_ms       = rac_config->radio_params.lora.rx_timeout_ms;
    }
    return rp_radio_params;
}
static void update_radio_params_for_ranging( smtc_rac_context_t* rac_config, rp_radio_params_t* radio_params )
{
    if( rac_config->radio_params.lora.is_ranging_exchange == true )
    {
        radio_params->rttof.request_address        = rac_config->radio_params.lora.rttof.request_address;
        radio_params->rttof.delay_indicator        = rac_config->radio_params.lora.rttof.delay_indicator;
        radio_params->rttof.response_symbols_count = rac_config->radio_params.lora.rttof.response_symbols_count;
        radio_params->rttof.bw_ranging             = rac_config->radio_params.lora.rttof.bw_ranging;
        radio_params->rttof.ranging_result         = &rac_config->smtc_rac_data_result.ranging_result;
    }
}
static void smtc_rac_tx_callback( void* rp_void )
{
    if (rp_void == NULL) {
        RAC_LOG_ERROR("TX callback: NULL radio planner pointer");
        return;
    }
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
    if( radio_params->pkt_type == RAL_PKT_TYPE_RTTOF )
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rf_freq( &( rp->radio->ral ), radio_params->tx.lora.rf_freq_in_hz ) ==
                                         RAL_STATUS_OK );

        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_pkt_type( &( rp->radio->ral ), RAL_PKT_TYPE_RTTOF ) == RAL_STATUS_OK );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_set_lora_mod_params( &( rp->radio->ral ), &radio_params->tx.lora.mod_params ) == RAL_STATUS_OK );

#if defined( LR20XX )
        // Have to be done after ral_set_rf_freq, ral_set_pkt_type, and ral_set_lora_mod_params
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( lr20xx_workarounds_rttof_truncate_pll_freq_step( rp->radio->ral.context ) ==
                                         LR20XX_STATUS_OK );
#endif

        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx_cfg( &( rp->radio->ral ), radio_params->tx.lora.output_pwr_in_dbm,
                                                         radio_params->tx.lora.rf_freq_in_hz ) == RAL_STATUS_OK );

        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_set_lora_pkt_params( &( rp->radio->ral ), &radio_params->tx.lora.pkt_params ) == RAL_STATUS_OK );

        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_set_lora_sync_word( &( rp->radio->ral ), radio_params->tx.lora.sync_word ) == RAL_STATUS_OK );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RTTOF_EXCH_VALID | RAL_IRQ_RTTOF_TIMEOUT ) ==
            RAL_STATUS_OK );

        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_rttof_set_rx_tx_delay_indicator( &( rp->radio->ral ), radio_params->rttof.delay_indicator ) ==
            RAL_STATUS_OK );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_rttof_set_parameters( &( rp->radio->ral ), radio_params->rttof.response_symbols_count ) ==
            RAL_STATUS_OK );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_rttof_set_request_address( &( rp->radio->ral ), radio_params->rttof.request_address ) ==
            RAL_STATUS_OK );
    }

    else
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_lora( rp->radio, &rp->radio_params[id].tx.lora ) == RAL_STATUS_OK );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE ) ==
                                         RAL_STATUS_OK );
    }

    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_buffer_size[id] ) == RAL_STATUS_OK );

    if( smtc_rac_get_context( id )->scheduler_config.callback_pre_radio_transaction != NULL )
    {
        smtc_rac_get_context( id )->scheduler_config.callback_pre_radio_transaction( );
    }

    smtc_rac_get_context( id )->smtc_rac_data_result.radio_start_timestamp_ms = rp->tasks[id].start_time_ms;

    while( ( int32_t ) ( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
        // Precise timing wait
    }

    if( rac_config->lbt_context.lbt_enabled == false )
    {
        smtc_modem_hal_start_radio_tcxo( );
    }
    smtc_modem_hal_set_ant_switch( true );
    if( rac_config->cw_context.cw_enabled )
    {
        rp_disable_failsafe( rp, true );
        if( rac_config->cw_context.infinite_preamble )
        {
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx_infinite_preamble( &( rp->radio->ral ) ) == RAL_STATUS_OK );
        }
        else
        {
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx_cw( &( rp->radio->ral ) ) == RAL_STATUS_OK );
        }
    }
    else
    {
        if( ral_set_tx( &( rp->radio->ral ) ) != RAL_STATUS_OK )
        {
            RAC_LOG_ERROR( "Failed to start LoRa transmission" );
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( false );
        }
    }

    uint32_t tx_timestamp = smtc_modem_hal_get_time_in_ms( );
    rp_stats_set_tx_timestamp( &rp->stats, tx_timestamp );

    RAC_LOG_TX( "LoRa TX Started Successfully!" );
    RAC_LOG_TX( "  Frequency: %u Hz (%.1f MHz)", radio_params->tx.lora.rf_freq_in_hz,
                ( float ) radio_params->tx.lora.rf_freq_in_hz / 1000000.0f );
    RAC_LOG_TX( "  Power: %d dBm", radio_params->tx.lora.output_pwr_in_dbm );
    RAC_LOG_TX( "  SF: %s, BW: %s", ral_lora_sf_to_str( radio_params->tx.lora.mod_params.sf ),
                ral_lora_bw_to_str( radio_params->tx.lora.mod_params.bw ) );
    RAC_LOG_TX( "  Payload Length: %u bytes", rp->payload_buffer_size[id] );
    RAC_LOG_TX( "  Ranging: %s", ( radio_params->pkt_type == RAL_PKT_TYPE_RTTOF ) ? "ENABLED" : "DISABLED" );
    RAC_LOG_TX( "  TX Timestamp: %lu ms", tx_timestamp );
    RAC_LOG_TX( "===== LoRa TX Callback - Complete =====" );
}

static void smtc_rac_rx_callback( void* rp_void )
{
    if (rp_void == NULL) {
        RAC_LOG_ERROR("RX callback: NULL radio planner pointer");
        return;
    }
    radio_planner_t*   rp           = ( radio_planner_t* ) rp_void;
    uint8_t            id           = rp->radio_task_id;
    rp_radio_params_t* radio_params = &rp->radio_params[id];
    if( radio_params->pkt_type == RAL_PKT_TYPE_RTTOF )
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rf_freq( &( rp->radio->ral ), radio_params->rx.lora.rf_freq_in_hz ) ==
                                         RAL_STATUS_OK );

        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_pkt_type( &( rp->radio->ral ), RAL_PKT_TYPE_RTTOF ) == RAL_STATUS_OK );

        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_stop_timer_on_preamble( &( rp->radio->ral ), false ) == RAL_STATUS_OK );

        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_set_lora_symb_nb_timeout( &( rp->radio->ral ), radio_params->rx.lora.symb_nb_timeout ) ==
            RAL_STATUS_OK );

        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_set_lora_mod_params( &( rp->radio->ral ), &radio_params->rx.lora.mod_params ) == RAL_STATUS_OK );

#if defined( LR20XX )
        // Have to be done after ral_set_rf_freq, ral_set_pkt_type, and ral_set_lora_mod_params
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( lr20xx_workarounds_rttof_truncate_pll_freq_step( rp->radio->ral.context ) ==
                                         LR20XX_STATUS_OK );
#endif

        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_set_lora_sync_word( &( rp->radio->ral ), radio_params->rx.lora.sync_word ) == RAL_STATUS_OK );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RTTOF_REQ_DISCARDED |
                                                             RAL_IRQ_RTTOF_RESP_DONE ) == RAL_STATUS_OK );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_rttof_set_address( &( rp->radio->ral ), radio_params->rttof.request_address, RTTOF_ADDRESS_LENGTH ) == RAL_STATUS_OK );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_rttof_set_parameters( &( rp->radio->ral ), radio_params->rttof.response_symbols_count ) ==
            RAL_STATUS_OK );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_rttof_set_rx_tx_delay_indicator( &( rp->radio->ral ), radio_params->rttof.delay_indicator ) ==
            RAL_STATUS_OK );
    }
    else
    {
        if( ralf_setup_lora( rp->radio, &rp->radio_params[id].rx.lora ) != RAL_STATUS_OK )
        {
            RAC_LOG_ERROR( "Failed to setup LoRa radio for RX" );
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( false );
        }
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                             RAL_IRQ_RX_CRC_ERROR ) == RAL_STATUS_OK );
    }

    // Wait the exact expected time (ie target - tcxo startup delay)
    if( smtc_rac_get_context( id )->scheduler_config.callback_pre_radio_transaction != NULL )
    {
        smtc_rac_get_context( id )->scheduler_config.callback_pre_radio_transaction( );
    }
    smtc_rac_get_context( id )->smtc_rac_data_result.radio_start_timestamp_ms = rp->tasks[id].start_time_ms;

    while( ( int32_t ) ( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
    }

    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rx( &( rp->radio->ral ), rp->radio_params[id].rx.timeout_in_ms ) ==
                                     RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
    RAC_LOG_RX( "===== LoRa RX Callback - Starting =====" );
    RAC_LOG_RX( "Radio Task ID: %u", id );
    RAC_LOG_RX( "Frequency: %lu Hz (%.1f MHz)", radio_params->rx.lora.rf_freq_in_hz,
                ( float ) radio_params->rx.lora.rf_freq_in_hz / 1000000.0f );
}
