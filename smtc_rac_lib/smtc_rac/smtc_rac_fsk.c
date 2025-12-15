/**
 * @file      smtc_rac_fsk.c
 *
 * @brief     smtc_rac_fsk api implementation
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
#include <string.h>   // memset

#include "radio_planner.h"
#include "ral.h"
#include "ralf.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_rac_api.h"
#include "smtc_rac_lbt.h"
#include "smtc_rac_local_func.h"

// Conditional logging for FSK module
#if RAC_FSK_LOG_ENABLE
#define RAC_LOG_APP_PREFIX "RAC-FSK"
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
/*!
 * \brief Default FSK synchronization word
 */
#define SMTC_RAC_FSK_DEFAULT_SYNC_WORD \
    {                                  \
        0xC1, 0x94, 0xC1               \
    }
/*!
 * \brief Default FSK configuration constants
 */
#define SMTC_RAC_FSK_WHITENING_SEED ( 0x01FF )
#define SMTC_RAC_FSK_CRC_SEED ( 0x1D0F )
#define SMTC_RAC_FSK_CRC_POLYNOMIAL ( 0x1021 )
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

// Note: FSK contexts are now stored in the unified smtc_rac_context array in smtc_rac.c

// Default FSK sync word
static const uint8_t default_fsk_sync_word[] = SMTC_RAC_FSK_DEFAULT_SYNC_WORD;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static rp_radio_params_t prepare_radio_params_for_fsk( smtc_rac_context_t* rac_config );
static void              smtc_rac_fsk_tx_callback( void* rp_void );
static void              smtc_rac_fsk_rx_callback( void* rp_void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

// Removed: smtc_rac_get_fsk_context() - now use smtc_rac_get_context() instead

smtc_rac_return_code_t smtc_rac_fsk( uint8_t radio_access_id )
{
    /**
     * @brief [TODO] , a function will be introduced to check all input parameters inside `smtc_rac_context_t`.
     *
     * This function will validate the compatibility of radio parameters and return in case of any incompatible
     * configuration.
     */

    smtc_modem_hal_protect_api_call( );
    smtc_rac_context_t* rac_config = smtc_rac_get_context( radio_access_id );

    RAC_LOG_CONFIG( "FSK: Starting %s for radio ID %d\n",
                    rac_config->radio_params.fsk.is_tx ? "transmission" : "reception", radio_access_id );

    // Set modulation type to FSK
    rac_config->modulation_type = SMTC_RAC_MODULATION_FSK;

    rp_radio_params_t rp_radio_params = prepare_radio_params_for_fsk( rac_config );

    const rp_task_t rp_task = {
        .hook_id = radio_access_id,
        .type    = ( rac_config->radio_params.fsk.is_tx == true ) ? RP_TASK_TYPE_TX_FSK : RP_TASK_TYPE_RX_FSK,
        .state = ( rac_config->scheduler_config.scheduling == SMTC_RAC_SCHEDULED_TRANSACTION ) ? RP_TASK_STATE_SCHEDULE
                                                                                               : RP_TASK_STATE_ASAP,

        .schedule_task_low_priority = false,
        .duration_time_ms           = ral_get_gfsk_time_on_air_in_ms(
            RAL_RADIO_POINTER,
            ( rac_config->radio_params.fsk.is_tx == true ) ? &( rp_radio_params.tx.gfsk.pkt_params )
                                                                     : &( rp_radio_params.rx.gfsk.pkt_params ),
            ( rac_config->radio_params.fsk.is_tx == true ) ? &( rp_radio_params.tx.gfsk.mod_params )
                                                                     : &( rp_radio_params.rx.gfsk.mod_params ) ),
        .start_time_ms = rac_config->scheduler_config.start_time_ms,
        .launch_task_callbacks =
            ( rac_config->radio_params.fsk.is_tx == true ) ? smtc_rac_fsk_tx_callback : smtc_rac_fsk_rx_callback,
    };

    if( rac_config->radio_params.fsk.is_tx )
    {
        RAC_LOG_TX( "FSK: Configuring TX - Freq:%lu Hz, Power:%d dBm, Bitrate:%lu bps, Payload:%d bytes\n",
                    rp_radio_params.tx.gfsk.rf_freq_in_hz, rp_radio_params.tx.gfsk.output_pwr_in_dbm,
                    rp_radio_params.tx.gfsk.mod_params.br_in_bps, rac_config->radio_params.fsk.tx_size );
    }
    else
    {
        RAC_LOG_RX( "FSK: Configuring RX - Freq:%lu Hz, Bitrate:%lu bps\n", rp_radio_params.rx.gfsk.rf_freq_in_hz,
                    rp_radio_params.rx.gfsk.mod_params.br_in_bps );
    }

    if( rp_task_enqueue( smtc_rac_get_rp( ), &rp_task,
                         ( rac_config->radio_params.fsk.is_tx )
                             ? rac_config->smtc_rac_data_buffer_setup.tx_payload_buffer
                             : rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer,
                         ( rac_config->radio_params.fsk.is_tx ) ? rac_config->radio_params.fsk.tx_size
                                                                : rac_config->radio_params.fsk.max_rx_size,
                         &rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        RAC_LOG_ERROR( "FSK: Error enqueueing task for radio access ID %d\n", radio_access_id );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    RAC_LOG_DEBUG( "FSK: Task successfully enqueued for radio ID %d\n", radio_access_id );
    smtc_modem_hal_unprotect_api_call( );
    return SMTC_RAC_SUCCESS;
}

smtc_rac_return_code_t smtc_rac_start_radio_transaction( uint8_t radio_access_id )
{
    smtc_modem_hal_protect_api_call( );
    smtc_rac_context_t*    rac_config = smtc_rac_get_context( radio_access_id );
    smtc_rac_return_code_t result;

    // Route to appropriate function based on modulation type
    switch( rac_config->modulation_type )
    {
    case SMTC_RAC_MODULATION_LORA:
        smtc_modem_hal_unprotect_api_call( );
        result = smtc_rac_lora( radio_access_id );
        break;

    case SMTC_RAC_MODULATION_FSK:
        smtc_modem_hal_unprotect_api_call( );
        result = smtc_rac_fsk( radio_access_id );
        break;

    case SMTC_RAC_MODULATION_LRFHSS:
        smtc_modem_hal_unprotect_api_call( );
        result = smtc_rac_lrfhss( radio_access_id );
        break;

    default:
        RAC_LOG_ERROR( "Invalid modulation type %d\n", rac_config->modulation_type );
        smtc_modem_hal_unprotect_api_call( );
        result = SMTC_RAC_INVALID_PARAMETER;
        break;
    }

    return result;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static rp_radio_params_t prepare_radio_params_for_fsk( smtc_rac_context_t* rac_config )
{
    ralf_params_gfsk_t gfsk_param = { 0 };
    memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

    // Basic radio parameters
    gfsk_param.rf_freq_in_hz     = rac_config->radio_params.fsk.frequency_in_hz;
    gfsk_param.output_pwr_in_dbm = rac_config->radio_params.fsk.tx_power_in_dbm;

    // Use provided sync word or default
    gfsk_param.sync_word = ( rac_config->radio_params.fsk.sync_word != NULL ) ? rac_config->radio_params.fsk.sync_word
                                                                              : default_fsk_sync_word;

    // Advanced parameters with defaults if not set
    gfsk_param.whitening_seed = ( rac_config->radio_params.fsk.whitening_seed != 0 )
                                    ? rac_config->radio_params.fsk.whitening_seed
                                    : SMTC_RAC_FSK_WHITENING_SEED;
    gfsk_param.crc_seed =
        ( rac_config->radio_params.fsk.crc_seed != 0 ) ? rac_config->radio_params.fsk.crc_seed : SMTC_RAC_FSK_CRC_SEED;
    gfsk_param.crc_polynomial = ( rac_config->radio_params.fsk.crc_polynomial != 0 )
                                    ? rac_config->radio_params.fsk.crc_polynomial
                                    : SMTC_RAC_FSK_CRC_POLYNOMIAL;

    // Packet parameters
    gfsk_param.pkt_params.header_type           = rac_config->radio_params.fsk.header_type;
    gfsk_param.pkt_params.pld_len_in_bytes      = rac_config->radio_params.fsk.tx_size;
    gfsk_param.pkt_params.preamble_len_in_bits  = rac_config->radio_params.fsk.preamble_len_in_bits;
    gfsk_param.pkt_params.preamble_detector     = rac_config->radio_params.fsk.preamble_detector;
    gfsk_param.pkt_params.sync_word_len_in_bits = rac_config->radio_params.fsk.sync_word_len_in_bits;
    gfsk_param.pkt_params.crc_type              = rac_config->radio_params.fsk.crc_type;
    gfsk_param.pkt_params.dc_free               = rac_config->radio_params.fsk.dc_free;

    // Modulation parameters
    gfsk_param.mod_params.br_in_bps    = rac_config->radio_params.fsk.br_in_bps;
    gfsk_param.mod_params.fdev_in_hz   = rac_config->radio_params.fsk.fdev_in_hz;
    gfsk_param.mod_params.bw_dsb_in_hz = rac_config->radio_params.fsk.bw_dsb_in_hz;
    gfsk_param.mod_params.pulse_shape  = rac_config->radio_params.fsk.pulse_shape;

    // Configure radio parameters
    rp_radio_params_t rp_radio_params = { 0 };
    rp_radio_params.pkt_type          = RAL_PKT_TYPE_GFSK;

    if( rac_config->radio_params.fsk.is_tx == true )

    {
        gfsk_param.pkt_params.pld_len_in_bytes = rac_config->radio_params.fsk.tx_size;
        rp_radio_params.tx.gfsk                = gfsk_param;
    }
    else
    {
        gfsk_param.pkt_params.pld_len_in_bytes = rac_config->radio_params.fsk.max_rx_size;
        rp_radio_params.rx.gfsk                = gfsk_param;
        rp_radio_params.rx.timeout_in_ms       = rac_config->radio_params.fsk.rx_timeout_ms;
    }

    return rp_radio_params;
}

static void smtc_rac_fsk_tx_callback( void* rp_void )
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
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_gfsk( rp->radio, &rp->radio_params[id].tx.gfsk ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE ) == RAL_STATUS_OK );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_buffer_size[id] ) == RAL_STATUS_OK );

    // Wait the exact expected time (ie target - tcxo startup delay)
    smtc_rac_context_t* rac_context = smtc_rac_get_context( id );
    if( rac_context->scheduler_config.callback_pre_radio_transaction != NULL )
    {
        rac_context->scheduler_config.callback_pre_radio_transaction( );
    }

    rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    while( ( int32_t ) ( rp->tasks[id].start_time_ms - rac_config->smtc_rac_data_result.radio_start_timestamp_ms ) > 0 )
    {
        rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    }

    // At this time only tcxo startup delay is remaining
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
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    }
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );

    RAC_LOG_TX( "FSK Tx callback - Freq:%lu Hz, Power:%d dBm, Bitrate:%lu bps, Fdev:%lu Hz, BW:%lu Hz, length:%u\n",
                radio_params->tx.gfsk.rf_freq_in_hz, radio_params->tx.gfsk.output_pwr_in_dbm,
                radio_params->tx.gfsk.mod_params.br_in_bps, radio_params->tx.gfsk.mod_params.fdev_in_hz,
                radio_params->tx.gfsk.mod_params.bw_dsb_in_hz, rp->payload_buffer_size[id] );
}

static void smtc_rac_fsk_rx_callback( void* rp_void )
{
    radio_planner_t*    rp           = ( radio_planner_t* ) rp_void;
    uint8_t             id           = rp->radio_task_id;
    rp_radio_params_t*  radio_params = &rp->radio_params[id];
    smtc_rac_context_t* rac_config   = smtc_rac_get_context( id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_gfsk( rp->radio, &radio_params->rx.gfsk ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                         RAL_IRQ_RX_CRC_ERROR ) == RAL_STATUS_OK );

    // Wait the exact expected time (ie target - tcxo startup delay)
    smtc_rac_context_t* rac_context = smtc_rac_get_context( id );
    if( rac_context->scheduler_config.callback_pre_radio_transaction != NULL )
    {
        rac_context->scheduler_config.callback_pre_radio_transaction( );
    }

    rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    while( ( int32_t ) ( rp->tasks[id].start_time_ms - rac_config->smtc_rac_data_result.radio_start_timestamp_ms ) > 0 )
    {
        rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    }

    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rx( &( rp->radio->ral ), radio_params->rx.timeout_in_ms ) ==
                                     RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );

    RAC_LOG_RX( "FSK Rx callback - Freq:%lu Hz, Bitrate:%lu bps, Fdev:%lu Hz, BW:%lu Hz\n",
                radio_params->rx.gfsk.rf_freq_in_hz, radio_params->rx.gfsk.mod_params.br_in_bps,
                radio_params->rx.gfsk.mod_params.fdev_in_hz, radio_params->rx.gfsk.mod_params.bw_dsb_in_hz );
}
