/**
 * @file      smtc_rac_flrc_burst.c
 *
 * @brief     smtc_rac_flrc_burst api implementation
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
#include "ral_defs.h"
#include "ral_drv.h"
#include "ralf.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_rac_api.h"
#include "smtc_rac_lbt.h"
#include "smtc_rac_local_func.h"

// Conditional logging for FLRC module
#ifndef RAC_FLRC_BURST_LOG_ENABLE
#define RAC_FLRC_BURST_LOG_ENABLE 1
#endif

#if RAC_FLRC_BURST_LOG_ENABLE
#define RAC_LOG_APP_PREFIX "RAC-FLRC-BURST"
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
#include "lr20xx_system.h"
#include "lr20xx_radio_flrc.h"
#include "lr20xx_radio_common.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Returns the minimum value between a and b
 *
 * @param [in] a 1st value
 * @param [in] b 2nd value
 * @retval Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif
#ifndef MAX
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif

#define RALF_RADIO_POINTER smtc_rac_get_rp( )->radio
#define RAL_RADIO_POINTER &( smtc_rac_get_rp( )->radio->ral )

#if defined( LR20XX )
/* ShapeCfg = "Generic TX Modulator" (Genmod), base 0xF30900 + offset 0x04.
 * DblRate = bit 8: interpolate by 16 instead of 8, only allowed for a bit rate <= 2 Mb/s. */
#define LR20XX_GENMOD_SHAPECFG_REG_ADDR ( 0x00F30904UL )
#define LR20XX_GENMOD_SHAPECFG_DBLRATE_MASK ( 1UL << 8 )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -------------------------------------------------------
 */

static void ( *backup_user_callback_pre_radio_transaction )( ); /*!< Callback called just before the radio transaction
                                                            (can be NULL). */
static void ( *backup_user_callback_post_radio_transaction )(
    rp_status_t status ); /*!< Callback called just after the radio transaction (must not be NULL). */

typedef struct smtc_rac_flrc_buffer_data_s
{
    uint8_t** payload_buffer; /*!< Pointer to the payload data buffer. */
    uint16_t* payload_size;
    uint32_t  payload_index;
} smtc_rac_flrc_buffer_data_t;

typedef struct smtc_rac_flrc_tx_fragmented_data_s
{
    smtc_rac_flrc_buffer_data_t
        tx_fifo[RAC_FLRC_BURST_NUMBER_OF_TX_FIFO_PTR];  //!< buffer pointers to the tx payload pushed alternatively to
                                                        //!< the radio fifo

    uint8_t  tx_fifo_ptr_index;            //!< index of the tx payload buffer to use
    uint32_t tx_nb_bytes_pushed_to_radio;  //!< amount of bytes sent to the radio fifo during a burst)
    uint32_t tx_nb_bytes_sent;             //!< amount of bytes sent by the radio during a burst)

} smtc_rac_flrc_tx_fragmented_data_t;

typedef struct smtc_rac_flrc_rx_fragmented_data_s
{
    uint16_t rx_fifo_payload_index;
} smtc_rac_flrc_rx_fragmented_data_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static smtc_rac_flrc_rx_fragmented_data_t smtc_rac_flrc_rx_fragmented_data = { 0 };
static smtc_rac_flrc_tx_fragmented_data_t smtc_rac_flrc_tx_fragmented_data = { 0 };
static rp_radio_params_t                  flrc_burst_rp_radio_params       = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static rp_radio_params_t prepare_radio_params_for_flrc( smtc_rac_context_t* rac_config );

static void smtc_rac_flrc_tx_callback( void* rp_void );
static void smtc_rac_flrc_rx_callback( void* rp_void );

static void smtc_rac_flrc_tx_rac_lock_callback( void );
static void smtc_rac_flrc_rx_rac_lock_callback( void );

static void smtc_rac_flrc_post_tx_callback( rp_status_t status );
static void smtc_rac_flrc_post_rx_callback( rp_status_t status );

static void read_rx_fifo_payload( uint16_t read_n_bytes );
static void push_packet_to_radio_fifo( void );

static inline void smtc_flrc_tx_wo_setup_callback( void* rp_void );

static inline uint32_t ceil_div( uint32_t numer, uint32_t denom );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_rac_return_code_t smtc_rac_flrc_burst( uint8_t radio_access_id )
{
    smtc_rac_return_code_t rac_return_code = SMTC_RAC_SUCCESS;
    uint32_t               time_on_air_us  = 0;
    smtc_modem_hal_protect_api_call( );

    smtc_rac_context_t* rac_config = smtc_rac_get_context( radio_access_id );
    if( rac_config == NULL )
    {
        RAC_LOG_ERROR( "FLRC-BURST: invalid context for radio ID %u\n", radio_access_id );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    // Validate modulation type
    if( rac_config->modulation_type != SMTC_RAC_MODULATION_FLRC_BURST )
    {
        RAC_LOG_ERROR( "FLRC-BURST: invalid modulation type for radio ID %u\n", radio_access_id );
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;
    }

    // RAC_LOG_CONFIG( "FLRC: Starting %s for radio ID %u\n", rac_config->radio_params.flrc_burst.is_tx ? "TX" : "RX",
    //                 radio_access_id );

    memset( &flrc_burst_rp_radio_params, 0, sizeof( flrc_burst_rp_radio_params ) );
    memset( &rac_config->smtc_rac_data_result, 0, sizeof( smtc_rac_data_result_t ) );

    // Backup the user callback pre and post radio transaction
    backup_user_callback_pre_radio_transaction  = rac_config->scheduler_config.callback_pre_radio_transaction;
    backup_user_callback_post_radio_transaction = rac_config->scheduler_config.callback_post_radio_transaction;

    if( rac_config->radio_params.flrc_burst.is_tx == true )
    {
        memset( &smtc_rac_flrc_tx_fragmented_data, 0, sizeof( smtc_rac_flrc_tx_fragmented_data ) );

        for( uint8_t i = 0; i < RAC_FLRC_BURST_NUMBER_OF_TX_FIFO_PTR; i++ )
        {
            if( rac_config->radio_params.flrc_burst.tx_fifo_payload_buffer[i] == NULL )
            {
                SMTC_MODEM_HAL_PANIC( "FLRC-BURST: tx fifo payload buffer %u is NULL\n", i );
            }
            if( rac_config->radio_params.flrc_burst.tx_fifo_payload_length[i] == NULL )
            {
                SMTC_MODEM_HAL_PANIC( "FLRC-BURST: tx fifo payload length %u is NULL\n", i );
            }
            smtc_rac_flrc_tx_fragmented_data.tx_fifo[i].payload_buffer =
                &rac_config->radio_params.flrc_burst.tx_fifo_payload_buffer[i];

            smtc_rac_flrc_tx_fragmented_data.tx_fifo[i].payload_size =
                rac_config->radio_params.flrc_burst.tx_fifo_payload_length[i];

            smtc_rac_flrc_tx_fragmented_data.tx_fifo[i].payload_index = 0;
        }

        flrc_burst_rp_radio_params = prepare_radio_params_for_flrc( rac_config );

        // Compute time on air for one packet using RAL FLRC API
        time_on_air_us =
            ral_get_flrc_time_on_air_in_us( RAL_RADIO_POINTER, &( flrc_burst_rp_radio_params.tx.flrc.pkt_params ),
                                            &( flrc_burst_rp_radio_params.tx.flrc.mod_params ) );

        // Compute time on air for the burst
        // Note: this TOA does not include the interframe time
        time_on_air_us *= ceil_div( rac_config->radio_params.flrc_burst.burst_tx_size,
                                    flrc_burst_rp_radio_params.tx.flrc.pkt_params.pld_len_in_bytes );

        rac_config->scheduler_config.callback_pre_radio_transaction  = smtc_rac_flrc_tx_rac_lock_callback;
        rac_config->scheduler_config.callback_post_radio_transaction = smtc_rac_flrc_post_tx_callback;
        rac_config->scheduler_config.duration_time_ms                = time_on_air_us / 1000;

        rac_return_code = smtc_rac_lock_radio_access( radio_access_id, rac_config->scheduler_config );
    }
    else
    {
        flrc_burst_rp_radio_params = prepare_radio_params_for_flrc( rac_config );

        // Compute time on air for one packet using RAL FLRC API
        time_on_air_us =
            ral_get_flrc_time_on_air_in_us( RAL_RADIO_POINTER, &( flrc_burst_rp_radio_params.rx.flrc.pkt_params ),
                                            &( flrc_burst_rp_radio_params.rx.flrc.mod_params ) );

        // Compute time on air for the burst
        // Note: this TOA does not include the interframe time, it's added after for the RX timeout
        time_on_air_us *= ceil_div( rac_config->radio_params.flrc_burst.burst_rx_size,
                                    flrc_burst_rp_radio_params.rx.flrc.pkt_params.pld_len_in_bytes );

        memset( &smtc_rac_flrc_rx_fragmented_data, 0, sizeof( smtc_rac_flrc_rx_fragmented_data ) );

        rac_config->scheduler_config.callback_pre_radio_transaction  = smtc_rac_flrc_rx_rac_lock_callback;
        rac_config->scheduler_config.callback_post_radio_transaction = smtc_rac_flrc_post_rx_callback;
        rac_config->scheduler_config.duration_time_ms                = time_on_air_us / 1000;

        // Start timeout
        uint32_t nb_packets = ceil_div( rac_config->radio_params.flrc_burst.burst_rx_size,
                                        rac_config->radio_params.flrc_burst.max_rx_size );
        uint32_t interframes_duration_us =
            ( ( nb_packets - 1 ) * rac_config->radio_params.flrc_burst.min_interframe_delay_us );

        time_on_air_us = ( time_on_air_us + interframes_duration_us ) / 1000 +
                        rac_config->radio_params.flrc_burst.rx_burst_timeout_margin_ms;
        time_on_air_us = time_on_air_us + ( time_on_air_us * 10 ) / 100;
        time_on_air_us = MAX( 11, time_on_air_us ); // A burst cannot be smaller than 11ms (even for 1 packet). To be reworked
        smtc_rac_set_active_time_out( radio_access_id, time_on_air_us );
        rac_return_code = smtc_rac_lock_radio_access( radio_access_id, rac_config->scheduler_config );
    }

    // RAC_LOG_DEBUG( "FLRC: Task successfully enqueued for radio ID %u, ToA: %u us\n", radio_access_id, time_on_air_us
    // );
    smtc_modem_hal_unprotect_api_call( );

    return rac_return_code;
}

smtc_rac_return_code_t smtc_rac_flrc_burst_rx_done( uint8_t radio_access_id )
{
    smtc_rac_unlock_radio_access( radio_access_id );
    return SMTC_RAC_SUCCESS;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static inline uint32_t ceil_div( uint32_t numer, uint32_t denom )
{
    // assume (denom != 0) and ((numer + denom - 1U) < UINT32_MAX)
    return ( numer + denom - 1U ) / denom;
}

static rp_radio_params_t prepare_radio_params_for_flrc( smtc_rac_context_t* rac_config )
{
    ralf_params_flrc_t flrc_param = { 0 };

    // Basic radio parameters
    flrc_param.rf_freq_in_hz = rac_config->radio_params.flrc_burst.frequency_in_hz;

    if( rac_config->radio_params.flrc_burst.is_tx == false )
    {
        flrc_param.rf_freq_in_hz += rac_config->radio_params.flrc_burst.rx_frequency_offset_in_hz;
    }

    flrc_param.output_pwr_in_dbm = rac_config->radio_params.flrc_burst.tx_power_in_dbm;

    // Modulation parameters
    flrc_param.mod_params.raw_bit_rate = rac_config->radio_params.flrc_burst.raw_bit_rate;
    flrc_param.mod_params.cr           = rac_config->radio_params.flrc_burst.cr;
    flrc_param.mod_params.pulse_shape  = rac_config->radio_params.flrc_burst.pulse_shape;

    // Packet parameters
    flrc_param.pkt_params.preamble_len    = rac_config->radio_params.flrc_burst.preamble_len;
    flrc_param.pkt_params.sync_word_len   = rac_config->radio_params.flrc_burst.sync_word_len;
    flrc_param.pkt_params.tx_syncword     = rac_config->radio_params.flrc_burst.tx_syncword_index;
    flrc_param.pkt_params.match_sync_word = rac_config->radio_params.flrc_burst.match_sync_word;
    flrc_param.pkt_params.pld_is_fix      = rac_config->radio_params.flrc_burst.pld_is_fix;
    flrc_param.pkt_params.crc_type        = rac_config->radio_params.flrc_burst.crc_type;

    // Advanced
    for( uint8_t i = 0; i < ( sizeof( flrc_param.sync_word ) / sizeof( flrc_param.sync_word[0] ) ); i++ )
    {
        flrc_param.sync_word[i] = rac_config->radio_params.flrc_burst.sync_word[i];
    }

    flrc_param.crc_seed       = rac_config->radio_params.flrc_burst.crc_seed;
    flrc_param.crc_polynomial = rac_config->radio_params.flrc_burst.crc_polynomial;

    // Configure radio params holder
    rp_radio_params_t rp_radio_params = { 0 };

    rp_radio_params.pkt_type = RAL_PKT_TYPE_FLRC;

    if( rac_config->radio_params.flrc_burst.is_tx == true )
    {
        rp_radio_params.tx.flrc.is_tx = true;
        flrc_param.pkt_params.pld_len_in_bytes =
            *smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index].payload_size;
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
        flrc_param.pkt_params.pld_len_in_bytes = rac_config->radio_params.flrc_burst.max_rx_size;
        rp_radio_params.rx.flrc.mod_params     = *( ( ral_flrc_mod_params_t* ) &( flrc_param.mod_params ) );
        rp_radio_params.rx.flrc.pkt_params     = *( ( ral_flrc_pkt_params_t* ) &( flrc_param.pkt_params ) );
        rp_radio_params.rx.flrc.rf_freq_in_hz  = flrc_param.rf_freq_in_hz;
        rp_radio_params.rx.timeout_in_ms       = 0xFFFFFFFF;
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

static void smtc_rac_flrc_tx_rac_lock_callback( void )
{
    radio_planner_t* rp = smtc_rac_get_rp( );
    smtc_rac_flrc_tx_callback( rp );
}

static void smtc_rac_flrc_rx_rac_lock_callback( void )
{
    radio_planner_t* rp = smtc_rac_get_rp( );
    smtc_rac_flrc_rx_callback( rp );
}

static void smtc_rac_flrc_post_rx_callback( rp_status_t status )
{
    radio_planner_t*    rp                                  = smtc_rac_get_rp( );
    smtc_rac_context_t* rac_config                          = smtc_rac_get_context( rp->radio_task_id );
    rac_config->smtc_rac_data_result.radio_end_timestamp_ms = rp->irq_timestamp_ms[rp->radio_task_id];

    if( status == RP_STATUS_RADIO_LOCKED )
    {
        ral_irq_t radio_irq = RAL_IRQ_NONE;

        // Used get_and_clear_irq_status to avoid to mask last rx_done interrupt with small packet
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_get_and_clear_irq_status( &rp->radio->ral, &radio_irq ) == RAL_STATUS_OK );
        if( ( radio_irq & RAL_IRQ_ERROR ) == RAL_IRQ_ERROR )
        {
#if defined( LR20XX ) && defined( RAC_FLRC_BURST_DEBUG_ENABLE )
            uint16_t errors;
            lr20xx_system_get_errors( rp->radio->ral.context, &errors );
            RAC_LOG_ERROR( "rac flrc: RX Error %x \n", errors );
            if( errors != 0 )
            {
                lr20xx_system_clear_errors( rp->radio->ral.context );
            }
#endif  // LR20XX && RAC_FLRC_BURST_DEBUG_ENABLE
        }
        if( ( ( radio_irq & RAL_IRQ_RX_HDR_ERROR ) == RAL_IRQ_RX_HDR_ERROR ) ||
            ( ( radio_irq & RAL_IRQ_RX_CRC_ERROR ) == RAL_IRQ_RX_CRC_ERROR ) )
        {
            // RAC_LOG_ERROR( "rac flrc: CRC error \n" );
            ral_clear_rx_fifo( &rp->radio->ral );

            smtc_rac_flrc_rx_fragmented_data.rx_fifo_payload_index = 0;
            // Callback to upper layer to give the CRC error
            if( backup_user_callback_post_radio_transaction != NULL )
            {
                backup_user_callback_post_radio_transaction( RP_STATUS_RX_CRC_ERROR );
            }
        }
        else if( ( radio_irq & RAL_IRQ_RX_DONE ) == RAL_IRQ_RX_DONE )
        {
            ral_flrc_rx_pkt_status_t flrc_pkt_status = { 0 };
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_get_flrc_rx_pkt_status( &rp->radio->ral, &flrc_pkt_status ) ==
                                             RAL_STATUS_OK );

            rac_config->smtc_rac_data_result.rx_size     = flrc_pkt_status.packet_length_bytes;
            rac_config->smtc_rac_data_result.rssi_result = flrc_pkt_status.rssi_avg_in_dbm;

            uint16_t read_n_bytes =
                MIN( ( flrc_pkt_status.packet_length_bytes - smtc_rac_flrc_rx_fragmented_data.rx_fifo_payload_index ),
                     RAC_FLRC_BURST_RADIO_FIFO_LENGTH );

            read_rx_fifo_payload( read_n_bytes );

            // Callback to upper layer to give the packet
            if( backup_user_callback_post_radio_transaction != NULL )
            {
                backup_user_callback_post_radio_transaction( RP_STATUS_RX_PACKET );
            }
            smtc_rac_flrc_rx_fragmented_data.rx_fifo_payload_index = 0;
        }
    }
    else if( status == RP_STATUS_RADIO_UNLOCKED )
    {
        // RAC_LOG_INFO( "rac flrc unlocked: rx transaction (reception) Unlocked \n" );

#if defined( LR20XX ) && defined( RAC_FLRC_BURST_DEBUG_ENABLE )
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // TODO must be removed in production code
        // radio commands MUST be executed only on radio planner context
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        lr20xx_radio_flrc_rx_stats_t lr20xx_radio_flrc_rx_stats = { 0 };
        lr20xx_radio_flrc_get_rx_stats( rp->radio->ral.context, &lr20xx_radio_flrc_rx_stats );
        lr20xx_radio_common_reset_rx_stats(
            rp->radio->ral.context );  // Note: reset in smtc_rac_flrc_burst() instead if this part of code stays
                                       // because it counts FLRC (non burst) packets too

        RAC_LOG_INFO( "rac flrc: RX Stats received_packets:%u, crc_errors:%u, length_errors:%u, crc_ok:%u \n",
                      lr20xx_radio_flrc_rx_stats.received_packets, lr20xx_radio_flrc_rx_stats.crc_errors,
                      lr20xx_radio_flrc_rx_stats.length_errors, lr20xx_radio_flrc_rx_stats.crc_ok );
#endif  // LR20XX && RAC_FLRC_BURST_DEBUG_ENABLE

        rac_config->scheduler_config.callback_pre_radio_transaction  = backup_user_callback_pre_radio_transaction;
        rac_config->scheduler_config.callback_post_radio_transaction = backup_user_callback_post_radio_transaction;
        if( rac_config->scheduler_config.callback_post_radio_transaction != NULL )
        {
            rac_config->scheduler_config.callback_post_radio_transaction( status );
        }
    }
    else if( status == RP_STATUS_TASK_ABORTED )
    {
        rac_config->scheduler_config.callback_pre_radio_transaction  = backup_user_callback_pre_radio_transaction;
        rac_config->scheduler_config.callback_post_radio_transaction = backup_user_callback_post_radio_transaction;
        if( rac_config->scheduler_config.callback_post_radio_transaction != NULL )
        {
            rac_config->scheduler_config.callback_post_radio_transaction( status );
        }
        // DIRECT_DRIVER_PRINT( "usp/rac: transaction (transmission) Aborted \n" );
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "rac flrc: rx transaction (reception) Error\n" );
        // DIRECT_DRIVER_PRINT( "usp/rac: transaction  (Error)\n" );
    }
}
static void read_rx_fifo_payload( uint16_t read_n_bytes )
{
    radio_planner_t*    rp         = smtc_rac_get_rp( );
    smtc_rac_context_t* rac_config = smtc_rac_get_context( rp->radio_task_id );

    if( ( smtc_rac_flrc_rx_fragmented_data.rx_fifo_payload_index + read_n_bytes ) >
        rac_config->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer )
    {
        ral_clear_rx_fifo( &rp->radio->ral );
        smtc_rac_flrc_rx_fragmented_data.rx_fifo_payload_index = 0;
        return;
    }

    ral_get_data_rx_buffer( &rp->radio->ral,
                            rac_config->smtc_rac_data_buffer_setup.rx_payload_buffer +
                                smtc_rac_flrc_rx_fragmented_data.rx_fifo_payload_index,
                            read_n_bytes );

    smtc_rac_flrc_rx_fragmented_data.rx_fifo_payload_index += read_n_bytes;
}

static void smtc_rac_flrc_post_tx_callback( rp_status_t status )
{
    radio_planner_t*    rp                                  = smtc_rac_get_rp( );
    smtc_rac_context_t* rac_config                          = smtc_rac_get_context( rp->radio_task_id );
    rac_config->smtc_rac_data_result.radio_end_timestamp_ms = rp->irq_timestamp_ms[rp->radio_task_id];
    if( status == RP_STATUS_RADIO_LOCKED )
    {
        ral_irq_t radio_irq = RAL_IRQ_TX_TIMESTAMP;

        // SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_get_irq_status( &rp->radio->ral, &radio_irq ) == RAL_STATUS_OK );
        if( ( radio_irq & RAL_IRQ_ERROR ) == RAL_IRQ_ERROR )
        {
#if defined( LR20XX ) && defined( RAC_FLRC_BURST_DEBUG_ENABLE )
            uint16_t errors;
            lr20xx_system_get_errors( rp->radio->ral.context, &errors );
            RAC_LOG_ERROR( "rac flrc: TX Error %x \n", errors );
            if( errors != 0 )
            {
                lr20xx_system_clear_errors( rp->radio->ral.context );
            }
#endif  // LR20XX && RAC_FLRC_BURST_DEBUG_ENABLE
        }

        if( ( radio_irq & RAL_IRQ_TX_TIMESTAMP ) == RAL_IRQ_TX_TIMESTAMP )
        {
            smtc_rac_flrc_tx_fragmented_data.tx_nb_bytes_sent +=
                flrc_burst_rp_radio_params.tx.flrc.pkt_params.pld_len_in_bytes;
            if( smtc_rac_flrc_tx_fragmented_data.tx_nb_bytes_sent == rac_config->radio_params.flrc_burst.burst_tx_size )
            {
                smtc_rac_unlock_radio_access( rp->radio_task_id );
            }
            else if( rac_config->radio_params.flrc_burst.burst_tx_size <
                     smtc_rac_flrc_tx_fragmented_data.tx_nb_bytes_sent )
            {
                SMTC_MODEM_HAL_PANIC_ON_FAILURE( "RAC FLRC BURST: sent more bytes than expected" );
            }
            else
            {
                bool is_pld_len_in_bytes_need_update = false;

                if( flrc_burst_rp_radio_params.tx.flrc.pkt_params.pld_len_in_bytes !=
                    *smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index]
                         .payload_size )
                {
                    is_pld_len_in_bytes_need_update = true;
                    flrc_burst_rp_radio_params.tx.flrc.pkt_params.pld_len_in_bytes =
                        *smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index]
                             .payload_size;
                }

                if( is_pld_len_in_bytes_need_update == true )
                {
                    ral_flrc_pkt_params_t pkt_params = {
                        .preamble_len     = rac_config->radio_params.flrc_burst.preamble_len,
                        .sync_word_len    = rac_config->radio_params.flrc_burst.sync_word_len,
                        .tx_syncword      = rac_config->radio_params.flrc_burst.tx_syncword_index,
                        .match_sync_word  = rac_config->radio_params.flrc_burst.match_sync_word,
                        .pld_is_fix       = rac_config->radio_params.flrc_burst.pld_is_fix,
                        .crc_type         = rac_config->radio_params.flrc_burst.crc_type,
                        .pld_len_in_bytes = *smtc_rac_flrc_tx_fragmented_data
                                                 .tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index]
                                                 .payload_size,
                    };
                    flrc_burst_rp_radio_params.tx.flrc.pkt_params = pkt_params;

                    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
                        ral_set_flrc_pkt_params( &rp->radio->ral, &flrc_burst_rp_radio_params.tx.flrc.pkt_params ) ==
                        RAL_STATUS_OK );
                }

                smtc_flrc_tx_wo_setup_callback( rp );

                // Callback to upper layer to generate the next packet
                if( backup_user_callback_post_radio_transaction != NULL )
                {
                    backup_user_callback_post_radio_transaction( RP_STATUS_REQUEST_NEXT_TX_PAYLOAD );
                }
                push_packet_to_radio_fifo( );
            }
        }
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_clear_irq_status( &rp->radio->ral, radio_irq | RAL_IRQ_TX_DONE ) ==
                                         RAL_STATUS_OK );
    }
    else if( status == RP_STATUS_RADIO_UNLOCKED )
    {
        // RAC_LOG_INFO( "rac flrc: tx transaction (transmission) Unlocked \n" );
        rac_config->scheduler_config.callback_pre_radio_transaction  = backup_user_callback_pre_radio_transaction;
        rac_config->scheduler_config.callback_post_radio_transaction = backup_user_callback_post_radio_transaction;
        if( rac_config->scheduler_config.callback_post_radio_transaction != NULL )
        {
            rac_config->scheduler_config.callback_post_radio_transaction( status );
        }
        smtc_rac_release_active_time_out( rp->radio_task_id );
    }
    else if( status == RP_STATUS_TASK_ABORTED )
    {
        rac_config->scheduler_config.callback_pre_radio_transaction  = backup_user_callback_pre_radio_transaction;
        rac_config->scheduler_config.callback_post_radio_transaction = backup_user_callback_post_radio_transaction;
        if( rac_config->scheduler_config.callback_post_radio_transaction != NULL )
        {
            rac_config->scheduler_config.callback_post_radio_transaction( status );
        }
        smtc_rac_release_active_time_out( rp->radio_task_id );

        // DIRECT_DRIVER_PRINT( "usp/rac: transaction (transmission) Aborted \n" );
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "rac flrc: tx transaction (transmission) Error\n" );
    }
}

static inline void smtc_flrc_tx_wo_setup_callback( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

static void push_packet_to_radio_fifo( void )
{
    radio_planner_t*    rp         = smtc_rac_get_rp( );
    smtc_rac_context_t* rac_config = smtc_rac_get_context( rp->radio_task_id );

    if( smtc_rac_flrc_tx_fragmented_data.tx_nb_bytes_pushed_to_radio <
        rac_config->radio_params.flrc_burst.burst_tx_size )
    {
        // Manage fifo pointer index
        if( smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index]
                .payload_index ==
            *smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index].payload_size )
        {
            smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index].payload_index =
                0;

            smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index++;
            if( smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index >= RAC_FLRC_BURST_NUMBER_OF_TX_FIFO_PTR )
            {
                smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index = 0;
            }
        }

        // Manage copy size
        if( smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index].payload_index <
            *smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index].payload_size )
        {
            uint32_t copy_size =
                MIN( *smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index]
                             .payload_size -
                         smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index]
                             .payload_index,
                     RAC_FLRC_BURST_RADIO_FIFO_LENGTH );

            SMTC_MODEM_HAL_PANIC_ON_FAILURE(
                ral_set_pkt_payload(
                    &rp->radio->ral,
                    smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index]
                            .payload_buffer[0] +
                        smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index]
                            .payload_index,
                    copy_size ) == RAL_STATUS_OK );

            smtc_rac_flrc_tx_fragmented_data.tx_fifo[smtc_rac_flrc_tx_fragmented_data.tx_fifo_ptr_index]
                .payload_index += copy_size;
            smtc_rac_flrc_tx_fragmented_data.tx_nb_bytes_pushed_to_radio += copy_size;
        }
    }
}

static void smtc_rac_flrc_tx_callback( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;

    smtc_rac_context_t* rac_config = smtc_rac_get_context( id );
    if( rac_config->lbt_context.lbt_enabled )
    {
        smtc_rac_lbt_listen_channel( id, rac_config->radio_params.flrc_burst.frequency_in_hz,
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

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_flrc( rp->radio, &flrc_burst_rp_radio_params.tx.flrc ) ==
                                     RAL_STATUS_OK );

#if defined( LR20XX )
    /* DblRate only applies to FLRC and for a bit rate <= 2 Mb/s.
     * Written AFTER ralf_setup_flrc (which rewrites ShapeCfg), as a masked read-modify-write on bit 8.
     * Explicitly write 0 when the bit rate is >= 2 Mb/s to stay deterministic.
     * No leak to other modulations: GFSK clears it back to 0 on set_pkt_type, and LoRa ignores ShapeCfg. */
    {
        const bool enable_dbl_rate =
            ( flrc_burst_rp_radio_params.tx.flrc.mod_params.raw_bit_rate < RAL_FLRC_RAW_BIT_RATE_2_080_MBPS );

        lr20xx_regmem_write_regmem32_mask( rp->radio->ral.context, LR20XX_GENMOD_SHAPECFG_REG_ADDR,
                                           LR20XX_GENMOD_SHAPECFG_DBLRATE_MASK,
                                           enable_dbl_rate ? LR20XX_GENMOD_SHAPECFG_DBLRATE_MASK : 0U );
    }
#endif

    ral_irq_t irq = RAL_IRQ_TX_TIMESTAMP;

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), irq ) == RAL_STATUS_OK );
    ral_clear_tx_fifo( &rp->radio->ral );

    // Push 2 packets to radio fifo to have one in advance
    push_packet_to_radio_fifo( );
    push_packet_to_radio_fifo( );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rx_tx_fallback_mode( &( rp->radio->ral ), RAL_FALLBACK_FS ) ==
                                     RAL_STATUS_OK );

    if( backup_user_callback_pre_radio_transaction != NULL )
    {
        backup_user_callback_pre_radio_transaction( );
    }

    do
    {
        rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    } while( ( int32_t ) ( rp->tasks[id].start_time_ms - rac_config->smtc_rac_data_result.radio_start_timestamp_ms ) >
             0 );

    if( rac_config->lbt_context.lbt_enabled == false )
    {
        smtc_modem_hal_start_radio_tcxo( );
    }
    smtc_modem_hal_set_ant_switch( true );
    // Need in 2.4GHz at low datarate to avoid crc errors
    ral_set_fs( &( rp->radio->ral ) );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );

    // RAC_LOG_TX( "FLRC Tx callback - Freq:%lu Hz, Power:%d dBm, BR_BW:%s Hz, length:%u\n",
    //             flrc_burst_rp_radio_params.tx.flrc.rf_freq_in_hz,
    //             flrc_burst_rp_radio_params.tx.flrc.output_pwr_in_dbm, ral_flrc_raw_bit_rate_to_str(
    //             flrc_burst_rp_radio_params.tx.flrc.mod_params.raw_bit_rate ), rp->payload_buffer_size[id] );
}

static void smtc_rac_flrc_rx_callback( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;

    smtc_rac_context_t* rac_config = smtc_rac_get_context( id );
    ral_irq_t           irq = RAL_IRQ_RX_DONE | RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR | RAL_IRQ_RX_LEN_ERROR;

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_flrc( rp->radio, &flrc_burst_rp_radio_params.rx.flrc ) ==
                                     RAL_STATUS_OK );

#if defined( LR20XX )
    // Disable IF correlator if the frequency offset was measured with à LoRa Rx

    if( rac_config->radio_params.flrc_burst.rx_frequency_offset_in_hz != 0 )
    {
        uint32_t value = 0x000000;
        lr20xx_regmem_write_regmem32( rp->radio->ral.context, 0x00F30D18, &value, 1 );
    }

#endif

    ral_clear_rx_fifo( &rp->radio->ral );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), irq ) == RAL_STATUS_OK );

    if( backup_user_callback_pre_radio_transaction != NULL )
    {
        backup_user_callback_pre_radio_transaction( );
    }

    // reset the rx payload size for the new packet
    smtc_rac_flrc_rx_fragmented_data.rx_fifo_payload_index = 0;

    do
    {
        rac_config->smtc_rac_data_result.radio_start_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
    } while( ( int32_t ) ( rp->tasks[id].start_time_ms - rac_config->smtc_rac_data_result.radio_start_timestamp_ms ) >
             0 );

    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rx( &( rp->radio->ral ), 0xFFFFFFFF ) == RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );

    // RAC_LOG_RX( "FLRC Rx callback - Freq:%lu Hz, BR_BW:%s Hz\n", flrc_burst_rp_radio_params.rx.flrc.rf_freq_in_hz,
    //             ral_flrc_raw_bit_rate_to_str( flrc_burst_rp_radio_params.rx.flrc.mod_params.raw_bit_rate ) );
}
