/**
 * @file      loragw_hal.c
 *
 * @brief     LoRa Gateway Hardware Abstraction Layer - RAC-based implementation
 *
 * This HAL provides a gateway-like API implemented on top of RAC.
 * Received packets are stored in a queue and can be retrieved via lgw_receive().
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

#include "loragw_hal.h"

/* Standard C library */
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* System/POSIX headers */
#include <pthread.h>

/* HAL layer */
#include "smtc_hal_dbg_trace.h"
#include "smtc_modem_hal.h"

/* RAC layer */
#define RAC_LOG_APP_PREFIX "LGW-HAL"
#include "smtc_rac_api.h"
#include "smtc_rac_log.h"
#include "smtc_sw_platform_helper.h"

/* RAL layer */
#include "ralf.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

#define LGW_LOG_INFO( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )
#define LGW_LOG_WARN( ... ) SMTC_HAL_TRACE_WARNING( __VA_ARGS__ )
#define LGW_LOG_ERROR( ... ) SMTC_HAL_TRACE_ERROR( __VA_ARGS__ )

#define CHECK_NULL( a )       \
    if( a == NULL )           \
    {                         \
        return LGW_HAL_ERROR; \
    }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Size of the RX packet queue (circular buffer)
 */
#define RX_QUEUE_SIZE 8

/**
 * @brief Maximum payload size for received packets
 */
#define MAX_PAYLOAD_SIZE 256

/**
 * @brief RX timeout in ms
 *
 * The RX transaction is automatically re-submitted in the post-callback
 */
#define RX_TIMEOUT_MS 60000

/**
 * @brief LoRaWAN sync words
 */
#define LORA_SYNCWORD_PUBLIC 0x34
#define LORA_SYNCWORD_PRIVATE 0x12

/**
 * @brief Library version string
 */
#define HAL_VERSION_STRING "loragw_hal 1.0.0 (RAC-based one-channel hub)"

/**
 * @brief LoRa preamble lengths
 */
#define MIN_LORA_PREAMBLE 6
#define STD_LORA_PREAMBLE 8

/**
 * @brief LoRaWAN FSK parameters (per LoRaWAN Regional Parameters)
 */
#define FSK_SYNC_WORD_SIZE 3      /* 3 bytes = 24 bits */
#define FSK_PREAMBLE_SIZE_BITS 40 /* 5 bytes of preamble (0xAA pattern) */
#define FSK_FDEV_HZ 25000         /* 25 kHz frequency deviation */
#define FSK_BANDWIDTH_HZ 100000   /* ~100 kHz RX bandwidth */
#define FSK_WHITENING_SEED 0x01FF /* LoRaWAN whitening seed */
#define FSK_CRC_SEED 0x1D0F       /* CCITT CRC seed */
#define FSK_CRC_POLYNOMIAL 0x1021 /* CCITT CRC polynomial */

/**
 * @brief LoRaWAN FSK sync word: 0xC194C1 (MSB first)
 */
static const uint8_t fsk_sync_word[FSK_SYNC_WORD_SIZE] = { 0xC1, 0x94, 0xC1 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief HAL context structure
 */
typedef struct lgw_hal_context_s
{
    /* RAC handles for RX (low priority, continuous) */
    uint8_t             rx_radio_access_id;
    smtc_rac_context_t* rx_transaction;

    /* RAC handles for TX (high priority, on-demand) */
    uint8_t             tx_radio_access_id;
    smtc_rac_context_t* tx_transaction;

    /* RX packet queue (circular buffer) */
    struct lgw_pkt_rx_s rx_queue[RX_QUEUE_SIZE];
    volatile uint8_t    rx_queue_head; /* Write index (callback writes here) */
    volatile uint8_t    rx_queue_tail; /* Read index (lgw_receive reads here) */
    pthread_mutex_t     rx_queue_mutex;

    /* Temporary RX buffer (used by RAC callback) */
    uint8_t rx_buffer[MAX_PAYLOAD_SIZE];

    /* TX buffer (used by RAC TX callback) */
    uint8_t tx_buffer[MAX_PAYLOAD_SIZE];

    /* Configuration */
    struct lgw_conf_board_s board_conf;
    struct lgw_conf_rxrf_s  rxrf_conf[LGW_RF_CHAIN_NB];
    struct lgw_conf_rxif_s  rxif_conf[LGW_IF_CHAIN_NB];

    /* Status */
    bool    is_started;
    uint8_t rx_status;
    uint8_t tx_status;
    uint8_t rx_modulation; /* MOD_LORA or MOD_FSK - current RX mode */

} lgw_hal_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lgw_hal_context_t lgw_ctx = {
    .rx_radio_access_id = 0,
    .rx_transaction     = NULL,
    .tx_radio_access_id = 0,
    .tx_transaction     = NULL,
    .rx_queue_head      = 0,
    .rx_queue_tail      = 0,
    .is_started         = false,
    .rx_status          = RX_STATUS_UNKNOWN,
    .tx_status          = TX_STATUS_UNKNOWN,
    .rx_modulation      = MOD_LORA,
};

/*
 * Guards the mutable gateway state in lgw_ctx (is_started, rx_status, tx_status),
 * accessed both by the threads calling this API and by the radio completion
 * callbacks. Keep critical sections short and never hold it across a radio-layer
 * call, so it cannot deadlock with a lock held around those callbacks.
 */
static pthread_mutex_t lgw_state_mutex = PTHREAD_MUTEX_INITIALIZER;

/* All guarded-state access goes through these accessors (lock taken internally,
 * never held across a radio-layer call). */

static bool lgw_state_get_is_started( void )
{
    pthread_mutex_lock( &lgw_state_mutex );
    bool started = lgw_ctx.is_started;
    pthread_mutex_unlock( &lgw_state_mutex );
    return started;
}

static void lgw_state_set_is_started( bool started )
{
    pthread_mutex_lock( &lgw_state_mutex );
    lgw_ctx.is_started = started;
    pthread_mutex_unlock( &lgw_state_mutex );
}

static uint8_t lgw_state_get_tx_status( void )
{
    pthread_mutex_lock( &lgw_state_mutex );
    uint8_t status = lgw_ctx.tx_status;
    pthread_mutex_unlock( &lgw_state_mutex );
    return status;
}

/*
 * Set tx_status, returning true on success. Setting most values is
 * unconditional; setting TX_SCHEDULED is only allowed when tx_status is TX_FREE,
 * so that transition is guarded (tested atomically under the lock) and returns
 * false when it is not free.
 */
static bool lgw_state_set_tx_status( uint8_t status )
{
    bool state_updated = true;

    pthread_mutex_lock( &lgw_state_mutex );
    switch( status )
    {
    case TX_SCHEDULED:
        /* Only allowed when the transmitter is free */
        if( lgw_ctx.tx_status == TX_FREE )
        {
            lgw_ctx.tx_status = TX_SCHEDULED;
        }
        else
        {
            state_updated = false;
        }
        break;

    default:
        /* All other states are set unconditionally */
        lgw_ctx.tx_status = status;
        break;
    }
    pthread_mutex_unlock( &lgw_state_mutex );

    return state_updated;
}

static uint8_t lgw_state_get_rx_status( void )
{
    pthread_mutex_lock( &lgw_state_mutex );
    uint8_t status = lgw_ctx.rx_status;
    pthread_mutex_unlock( &lgw_state_mutex );
    return status;
}

static void lgw_state_set_rx_status( uint8_t status )
{
    pthread_mutex_lock( &lgw_state_mutex );
    lgw_ctx.rx_status = status;
    pthread_mutex_unlock( &lgw_state_mutex );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/* RX callbacks */
static void rx_pre_transaction_callback( void );
static void rx_post_transaction_callback( rp_status_t status );
static int  submit_rx_transaction( void );

/* TX callbacks */
static void tx_pre_transaction_callback( void );
static void tx_post_transaction_callback( rp_status_t status );

/* RX queue helpers */
static bool rx_queue_is_full( void );
static bool rx_queue_is_empty( void );
static void rx_queue_enqueue( const struct lgw_pkt_rx_s* pkt );
static bool rx_queue_dequeue( struct lgw_pkt_rx_s* pkt );

/* Conversion helpers */
static ral_lora_sf_t convert_sf_to_ral( uint32_t sf );
static ral_lora_bw_t convert_bw_to_ral( uint8_t bw );
static ral_lora_cr_t convert_cr_to_ral( uint8_t cr );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

int lgw_board_setconf( struct lgw_conf_board_s* conf )
{
    CHECK_NULL( conf );

    if( lgw_state_get_is_started( ) )
    {
        LGW_LOG_ERROR( "HAL is running, stop it before changing configuration\n" );
        return LGW_HAL_ERROR;
    }

    memcpy( &lgw_ctx.board_conf, conf, sizeof( struct lgw_conf_board_s ) );

    LGW_LOG_INFO( "Board config: lorawan_public=%d, com_type=%d, com_path=%s\n", conf->lorawan_public, conf->com_type,
                  conf->com_path );

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxrf_setconf( uint8_t rf_chain, struct lgw_conf_rxrf_s* conf )
{
    CHECK_NULL( conf );

    if( rf_chain >= LGW_RF_CHAIN_NB )
    {
        LGW_LOG_ERROR( "Invalid RF chain: %d\n", rf_chain );
        return LGW_HAL_ERROR;
    }

    if( lgw_state_get_is_started( ) )
    {
        LGW_LOG_ERROR( "HAL is running, stop it before changing configuration\n" );
        return LGW_HAL_ERROR;
    }

    memcpy( &lgw_ctx.rxrf_conf[rf_chain], conf, sizeof( struct lgw_conf_rxrf_s ) );

    LGW_LOG_INFO( "RF chain %d config: enable=%d, freq=%" PRIu32 " Hz, tx_enable=%d\n", rf_chain, conf->enable,
                  conf->freq_hz, conf->tx_enable );

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxif_setconf( uint8_t if_chain, struct lgw_conf_rxif_s* conf )
{
    CHECK_NULL( conf );

    if( if_chain >= LGW_IF_CHAIN_NB )
    {
        LGW_LOG_ERROR( "Invalid IF chain: %d\n", if_chain );
        return LGW_HAL_ERROR;
    }

    if( lgw_state_get_is_started( ) )
    {
        LGW_LOG_ERROR( "HAL is running, stop it before changing configuration\n" );
        return LGW_HAL_ERROR;
    }

    memcpy( &lgw_ctx.rxif_conf[if_chain], conf, sizeof( struct lgw_conf_rxif_s ) );

    LGW_LOG_INFO( "IF chain %d config: enable=%d, rf_chain=%d, freq=%d Hz, dr=%" PRIu32 "\n", if_chain, conf->enable,
                  conf->rf_chain, conf->freq_hz, conf->datarate );

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_demod_setconf( struct lgw_conf_demod_s* conf )
{
    ( void ) conf;

    printf( "lgw_demod_setconf not implemented\n" );
    return LGW_HAL_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_txgain_setconf( uint8_t rf_chain, struct lgw_tx_gain_lut_s* conf )
{
    CHECK_NULL( conf );
    ( void ) rf_chain;

    printf( "lgw_txgain_setconf not implemented\n" );
    return LGW_HAL_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_start( void )
{
    if( lgw_state_get_is_started( ) )
    {
        LGW_LOG_WARN( "HAL already started\n" );
        return LGW_HAL_SUCCESS;
    }

    LGW_LOG_INFO( "========================================\n" );
    LGW_LOG_INFO( "     LORAGW HAL (RAC-based) START\n" );
    LGW_LOG_INFO( "========================================\n" );

    /* Initialize mutex */
    pthread_mutex_init( &lgw_ctx.rx_queue_mutex, NULL );

    /* Reset queue */
    lgw_ctx.rx_queue_head = 0;
    lgw_ctx.rx_queue_tail = 0;

    /* Get the datarate from IF chain config */
    uint32_t datarate = lgw_ctx.rxif_conf[0].datarate;

    /*
     * -------------------------------------------------------------------------
     * RX Radio Access Setup (low priority, continuous reception)
     * -------------------------------------------------------------------------
     */
    lgw_ctx.rx_radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_LOW_PRIORITY ) );
    lgw_ctx.rx_transaction     = smtc_rac_get_context( lgw_ctx.rx_radio_access_id );

    /* Configure RX callbacks */
    lgw_ctx.rx_transaction->scheduler_config.callback_post_radio_transaction = rx_post_transaction_callback;
    lgw_ctx.rx_transaction->scheduler_config.callback_pre_radio_transaction  = rx_pre_transaction_callback;
    lgw_ctx.rx_transaction->scheduler_config.scheduling                      = SMTC_RAC_ASAP_TRANSACTION;

    /* Configure RX data buffers */
    lgw_ctx.rx_transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = NULL;
    lgw_ctx.rx_transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = lgw_ctx.rx_buffer;
    lgw_ctx.rx_transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = 0;
    lgw_ctx.rx_transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = sizeof( lgw_ctx.rx_buffer );

    /* Disable LBT for RX */
    lgw_ctx.rx_transaction->lbt_context.lbt_enabled = false;

    /* Determine modulation type based on datarate */
    if( IS_FSK_DR( datarate ) )
    {
        /*
         * ---------------------------------------------------------------------
         * FSK RX Configuration
         * ---------------------------------------------------------------------
         */
        lgw_ctx.rx_modulation = MOD_FSK;

        lgw_ctx.rx_transaction->modulation_type                  = SMTC_RAC_MODULATION_FSK;
        lgw_ctx.rx_transaction->radio_params.fsk.is_tx           = false;
        lgw_ctx.rx_transaction->radio_params.fsk.frequency_in_hz = lgw_ctx.rxrf_conf[0].freq_hz;
        lgw_ctx.rx_transaction->radio_params.fsk.tx_power_in_dbm = 0;

        /* Modulation parameters */
        lgw_ctx.rx_transaction->radio_params.fsk.br_in_bps    = datarate;
        lgw_ctx.rx_transaction->radio_params.fsk.fdev_in_hz   = FSK_FDEV_HZ;
        lgw_ctx.rx_transaction->radio_params.fsk.bw_dsb_in_hz = FSK_BANDWIDTH_HZ;
        lgw_ctx.rx_transaction->radio_params.fsk.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;

        /* Packet parameters */
        lgw_ctx.rx_transaction->radio_params.fsk.header_type           = RAL_GFSK_PKT_VAR_LEN;
        lgw_ctx.rx_transaction->radio_params.fsk.preamble_len_in_bits  = FSK_PREAMBLE_SIZE_BITS;
        lgw_ctx.rx_transaction->radio_params.fsk.preamble_detector     = RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
        lgw_ctx.rx_transaction->radio_params.fsk.sync_word_len_in_bits = FSK_SYNC_WORD_SIZE * 8;
        lgw_ctx.rx_transaction->radio_params.fsk.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;
        lgw_ctx.rx_transaction->radio_params.fsk.dc_free               = RAL_GFSK_DC_FREE_WHITENING;

        /* Sync word and CRC parameters */
        lgw_ctx.rx_transaction->radio_params.fsk.sync_word      = fsk_sync_word;
        lgw_ctx.rx_transaction->radio_params.fsk.whitening_seed = FSK_WHITENING_SEED;
        lgw_ctx.rx_transaction->radio_params.fsk.crc_seed       = FSK_CRC_SEED;
        lgw_ctx.rx_transaction->radio_params.fsk.crc_polynomial = FSK_CRC_POLYNOMIAL;

        /* RX parameters */
        lgw_ctx.rx_transaction->radio_params.fsk.tx_size       = 0;
        lgw_ctx.rx_transaction->radio_params.fsk.max_rx_size   = MAX_PAYLOAD_SIZE;
        lgw_ctx.rx_transaction->radio_params.fsk.rx_timeout_ms = RX_TIMEOUT_MS;

        LGW_LOG_INFO( "RX mode: FSK, Bitrate: %" PRIu32 " bps\n", datarate );
    }
    else if( IS_LORA_DR( datarate ) )
    {
        /*
         * ---------------------------------------------------------------------
         * LoRa RX Configuration
         * ---------------------------------------------------------------------
         */
        lgw_ctx.rx_modulation = MOD_LORA;

        lgw_ctx.rx_transaction->modulation_type                       = SMTC_RAC_MODULATION_LORA;
        lgw_ctx.rx_transaction->radio_params.lora.is_tx               = false;
        lgw_ctx.rx_transaction->radio_params.lora.is_ranging_exchange = false;
        lgw_ctx.rx_transaction->radio_params.lora.frequency_in_hz     = lgw_ctx.rxrf_conf[0].freq_hz;
        lgw_ctx.rx_transaction->radio_params.lora.tx_power_in_dbm     = 0;
        lgw_ctx.rx_transaction->radio_params.lora.sf                  = convert_sf_to_ral( datarate );
        lgw_ctx.rx_transaction->radio_params.lora.bw = convert_bw_to_ral( lgw_ctx.rxif_conf[0].bandwidth );
        lgw_ctx.rx_transaction->radio_params.lora.cr = RAL_LORA_CR_4_5;
        lgw_ctx.rx_transaction->radio_params.lora.preamble_len_in_symb = STD_LORA_PREAMBLE;
        lgw_ctx.rx_transaction->radio_params.lora.header_type          = RAL_LORA_PKT_EXPLICIT;
        lgw_ctx.rx_transaction->radio_params.lora.invert_iq_is_on      = false; /* Uplink = no inversion */
        lgw_ctx.rx_transaction->radio_params.lora.crc_is_on            = true;

        /* SF5/SF6 are not LoRaWAN standard, always use private syncword */
        if( datarate == DR_LORA_SF5 || datarate == DR_LORA_SF6 )
        {
            lgw_ctx.rx_transaction->radio_params.lora.sync_word = LORA_SYNCWORD_PRIVATE;
        }
        else
        {
            lgw_ctx.rx_transaction->radio_params.lora.sync_word =
                lgw_ctx.board_conf.lorawan_public ? LORA_SYNCWORD_PUBLIC : LORA_SYNCWORD_PRIVATE;
        }
        lgw_ctx.rx_transaction->radio_params.lora.tx_size       = 0;
        lgw_ctx.rx_transaction->radio_params.lora.max_rx_size   = MAX_PAYLOAD_SIZE;
        lgw_ctx.rx_transaction->radio_params.lora.rx_timeout_ms = RX_TIMEOUT_MS;

        LGW_LOG_INFO( "RX mode: LoRa, SF: %" PRIu32 ", BW: %d kHz\n", datarate,
                      ( lgw_ctx.rxif_conf[0].bandwidth == BW_500KHZ )   ? 500
                      : ( lgw_ctx.rxif_conf[0].bandwidth == BW_250KHZ ) ? 250
                                                                        : 125 );
    }
    else
    {
        LGW_LOG_ERROR( "Invalid datarate: %" PRIu32 "\n", datarate );
        return LGW_HAL_ERROR;
    }

    /*
     * -------------------------------------------------------------------------
     * TX Radio Access Setup (high priority, on-demand transmission)
     * -------------------------------------------------------------------------
     */
    if( lgw_ctx.rxrf_conf[0].tx_enable )
    {
        lgw_ctx.tx_radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );
        lgw_ctx.tx_transaction     = smtc_rac_get_context( lgw_ctx.tx_radio_access_id );

        /* Configure TX callbacks */
        lgw_ctx.tx_transaction->scheduler_config.callback_post_radio_transaction = tx_post_transaction_callback;
        lgw_ctx.tx_transaction->scheduler_config.callback_pre_radio_transaction  = tx_pre_transaction_callback;

        /* Configure TX data buffers */
        lgw_ctx.tx_transaction->smtc_rac_data_buffer_setup.tx_payload_buffer         = lgw_ctx.tx_buffer;
        lgw_ctx.tx_transaction->smtc_rac_data_buffer_setup.rx_payload_buffer         = NULL;
        lgw_ctx.tx_transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( lgw_ctx.tx_buffer );
        lgw_ctx.tx_transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = 0;

        /* TX LoRa parameters will be configured per-packet in lgw_send() */
        lgw_ctx.tx_transaction->modulation_type = SMTC_RAC_MODULATION_LORA;

        /* Disable LBT for TX (can be enabled per-packet if needed) */
        lgw_ctx.tx_transaction->lbt_context.lbt_enabled = false;

        LGW_LOG_INFO( "TX radio access opened (high priority)\n" );
    }

    /* Log configuration */
    LGW_LOG_INFO( "RX: RF chain %d, IF chain %d\n", 0, 0 );
    LGW_LOG_INFO( "RX Frequency: %" PRIu32 " Hz\n", lgw_ctx.rxrf_conf[0].freq_hz );
    LGW_LOG_INFO( "RX Queue size: %d packets\n", RX_QUEUE_SIZE );

    /* Update status */
    lgw_state_set_rx_status( RX_OFF );
    lgw_state_set_tx_status( lgw_ctx.rxrf_conf[0].tx_enable ? TX_FREE : TX_OFF );
    lgw_state_set_is_started( true );

    /* Start first RX transaction */
    if( submit_rx_transaction( ) != LGW_HAL_SUCCESS )
    {
        LGW_LOG_ERROR( "Failed to start RX reception\n" );
        lgw_state_set_is_started( false );
        return LGW_HAL_ERROR;
    }
    lgw_state_set_rx_status( RX_ON );

    LGW_LOG_INFO( "HAL started, RX continuous mode active\n" );
    LGW_LOG_INFO( "========================================\n" );

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_stop( void )
{
    if( !lgw_state_get_is_started( ) )
    {
        LGW_LOG_INFO( "HAL was not started\n" );
        return LGW_HAL_SUCCESS;
    }

    LGW_LOG_INFO( "Stopping HAL...\n" );

    /* Mark stopped first so the RX completion callback stops re-submitting. */
    lgw_state_set_is_started( false );
    uint8_t tx_status_snap = lgw_state_get_tx_status( );

    /* Abort any pending TX transaction */
    if( tx_status_snap == TX_SCHEDULED || tx_status_snap == TX_EMITTING )
    {
        smtc_rac_abort_radio_submit( lgw_ctx.tx_radio_access_id );
    }

    /* Abort RX transaction */
    smtc_rac_abort_radio_submit( lgw_ctx.rx_radio_access_id );

    /* Close radio accesses */
    smtc_rac_close_radio( lgw_ctx.rx_radio_access_id );
    if( lgw_ctx.tx_transaction != NULL )
    {
        smtc_rac_close_radio( lgw_ctx.tx_radio_access_id );
    }

    /* Update status */
    lgw_state_set_rx_status( RX_OFF );
    lgw_state_set_tx_status( TX_OFF );

    /* Reset contexts */
    lgw_ctx.rx_transaction = NULL;
    lgw_ctx.tx_transaction = NULL;

    /* rx_queue_mutex is intentionally not destroyed: a late RX completion callback
     * may still use it after stop (it is effectively static-lifetime). */

    LGW_LOG_INFO( "HAL stopped\n" );

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_receive( uint8_t max_pkt, struct lgw_pkt_rx_s* pkt_data )
{
    CHECK_NULL( pkt_data );

    if( !lgw_state_get_is_started( ) )
    {
        LGW_LOG_ERROR( "HAL is not started\n" );
        return LGW_HAL_ERROR;
    }

    int count = 0;

    pthread_mutex_lock( &lgw_ctx.rx_queue_mutex );

    while( count < max_pkt && !rx_queue_is_empty( ) )
    {
        if( rx_queue_dequeue( &pkt_data[count] ) )
        {
            count++;
        }
        else
        {
            break;
        }
    }

    pthread_mutex_unlock( &lgw_ctx.rx_queue_mutex );

    return count;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send( struct lgw_pkt_tx_s* pkt_data )
{
    CHECK_NULL( pkt_data );

    if( !lgw_state_get_is_started( ) )
    {
        LGW_LOG_ERROR( "HAL is not started\n" );
        return LGW_HAL_ERROR;
    }

    /* Validate RF chain */
    if( pkt_data->rf_chain >= LGW_RF_CHAIN_NB )
    {
        LGW_LOG_ERROR( "Invalid RF chain: %d\n", pkt_data->rf_chain );
        return LGW_HAL_ERROR;
    }

    /* Validate modulation */
    if( pkt_data->modulation != MOD_LORA && pkt_data->modulation != MOD_FSK )
    {
        LGW_LOG_ERROR( "Unsupported modulation: %d (only LoRa and FSK supported)\n", pkt_data->modulation );
        return LGW_HAL_ERROR;
    }

    /* Validate payload size */
    if( pkt_data->size > MAX_PAYLOAD_SIZE )
    {
        LGW_LOG_ERROR( "Payload too large: %d > %d\n", pkt_data->size, MAX_PAYLOAD_SIZE );
        return LGW_HAL_ERROR;
    }

    /* Validate TX mode up front so the only failure after setting TX_SCHEDULED
     * (below) is the radio submit, which keeps the rollback simple. */
    if( pkt_data->tx_mode != IMMEDIATE && pkt_data->tx_mode != TIMESTAMPED )
    {
        LGW_LOG_ERROR( "Invalid/unsupported TX mode: %d\n", pkt_data->tx_mode );
        return LGW_HAL_ERROR;
    }

    /* Set tx_status to TX_SCHEDULED (only allowed when the transmitter is free) */
    if( !lgw_state_set_tx_status( TX_SCHEDULED ) )
    {
        LGW_LOG_ERROR( "Failed to set TX status\n" );
        return LGW_HAL_ERROR;
    }

    /* Copy payload to TX buffer */
    memcpy( lgw_ctx.tx_buffer, pkt_data->payload, pkt_data->size );

    /* Configure TX parameters based on modulation type */
    if( pkt_data->modulation == MOD_FSK )
    {
        /*
         * ---------------------------------------------------------------------
         * FSK TX Configuration
         * ---------------------------------------------------------------------
         */
        lgw_ctx.tx_transaction->modulation_type = SMTC_RAC_MODULATION_FSK;

        lgw_ctx.tx_transaction->radio_params.fsk.is_tx           = true;
        lgw_ctx.tx_transaction->radio_params.fsk.frequency_in_hz = pkt_data->freq_hz;
        lgw_ctx.tx_transaction->radio_params.fsk.tx_power_in_dbm = pkt_data->rf_power;

        /* Modulation parameters */
        lgw_ctx.tx_transaction->radio_params.fsk.br_in_bps = pkt_data->datarate; /* Bitrate in bps */
        lgw_ctx.tx_transaction->radio_params.fsk.fdev_in_hz =
            pkt_data->f_dev * 1000; /* f_dev is in kHz, convert to Hz */
        lgw_ctx.tx_transaction->radio_params.fsk.bw_dsb_in_hz = FSK_BANDWIDTH_HZ;
        lgw_ctx.tx_transaction->radio_params.fsk.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;

        /* Packet parameters */
        lgw_ctx.tx_transaction->radio_params.fsk.header_type =
            pkt_data->no_header ? RAL_GFSK_PKT_FIX_LEN : RAL_GFSK_PKT_VAR_LEN;
        lgw_ctx.tx_transaction->radio_params.fsk.preamble_len_in_bits =
            ( pkt_data->preamble > 0 ) ? ( pkt_data->preamble * 8 ) : FSK_PREAMBLE_SIZE_BITS;
        lgw_ctx.tx_transaction->radio_params.fsk.preamble_detector     = RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
        lgw_ctx.tx_transaction->radio_params.fsk.sync_word_len_in_bits = FSK_SYNC_WORD_SIZE * 8;
        lgw_ctx.tx_transaction->radio_params.fsk.crc_type =
            pkt_data->no_crc ? RAL_GFSK_CRC_OFF : RAL_GFSK_CRC_2_BYTES_INV;
        lgw_ctx.tx_transaction->radio_params.fsk.dc_free = RAL_GFSK_DC_FREE_WHITENING;

        /* Sync word and CRC parameters */
        lgw_ctx.tx_transaction->radio_params.fsk.sync_word      = fsk_sync_word;
        lgw_ctx.tx_transaction->radio_params.fsk.whitening_seed = FSK_WHITENING_SEED;
        lgw_ctx.tx_transaction->radio_params.fsk.crc_seed       = FSK_CRC_SEED;
        lgw_ctx.tx_transaction->radio_params.fsk.crc_polynomial = FSK_CRC_POLYNOMIAL;

        /* TX parameters */
        lgw_ctx.tx_transaction->radio_params.fsk.tx_size       = pkt_data->size;
        lgw_ctx.tx_transaction->radio_params.fsk.max_rx_size   = 0;
        lgw_ctx.tx_transaction->radio_params.fsk.rx_timeout_ms = 0;
    }
    else
    {
        /*
         * ---------------------------------------------------------------------
         * LoRa TX Configuration
         * ---------------------------------------------------------------------
         */
        lgw_ctx.tx_transaction->modulation_type = SMTC_RAC_MODULATION_LORA;

        lgw_ctx.tx_transaction->radio_params.lora.is_tx               = true;
        lgw_ctx.tx_transaction->radio_params.lora.is_ranging_exchange = false;
        lgw_ctx.tx_transaction->radio_params.lora.frequency_in_hz     = pkt_data->freq_hz;
        lgw_ctx.tx_transaction->radio_params.lora.tx_power_in_dbm     = pkt_data->rf_power;
        lgw_ctx.tx_transaction->radio_params.lora.sf                  = convert_sf_to_ral( pkt_data->datarate );
        lgw_ctx.tx_transaction->radio_params.lora.bw                  = convert_bw_to_ral( pkt_data->bandwidth );
        lgw_ctx.tx_transaction->radio_params.lora.cr                  = convert_cr_to_ral( pkt_data->coderate );
        lgw_ctx.tx_transaction->radio_params.lora.preamble_len_in_symb =
            ( pkt_data->preamble > 0 ) ? pkt_data->preamble : STD_LORA_PREAMBLE;
        lgw_ctx.tx_transaction->radio_params.lora.header_type =
            pkt_data->no_header ? RAL_LORA_PKT_IMPLICIT : RAL_LORA_PKT_EXPLICIT;
        lgw_ctx.tx_transaction->radio_params.lora.invert_iq_is_on = pkt_data->invert_pol; /* Downlink = inverted IQ */
        lgw_ctx.tx_transaction->radio_params.lora.crc_is_on       = !pkt_data->no_crc;

        /* SF5/SF6 are not LoRaWAN standard, always use private syncword */
        if( pkt_data->datarate == DR_LORA_SF5 || pkt_data->datarate == DR_LORA_SF6 )
        {
            lgw_ctx.tx_transaction->radio_params.lora.sync_word = LORA_SYNCWORD_PRIVATE;
        }
        else
        {
            lgw_ctx.tx_transaction->radio_params.lora.sync_word =
                lgw_ctx.board_conf.lorawan_public ? LORA_SYNCWORD_PUBLIC : LORA_SYNCWORD_PRIVATE;
        }
        lgw_ctx.tx_transaction->radio_params.lora.tx_size       = pkt_data->size;
        lgw_ctx.tx_transaction->radio_params.lora.max_rx_size   = 0;
        lgw_ctx.tx_transaction->radio_params.lora.rx_timeout_ms = 0;
    }

    /* Configure scheduling based on TX mode */
    switch( pkt_data->tx_mode )
    {
    case IMMEDIATE:
    {
        /* Transmit as soon as possible */
        lgw_ctx.tx_transaction->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
        lgw_ctx.tx_transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( );
        if( pkt_data->modulation == MOD_FSK )
        {
            LGW_LOG_INFO( "TX IMMEDIATE FSK: freq=%" PRIu32 " Hz, br=%" PRIu32 " bps, pwr=%d dBm, size=%d\n",
                          pkt_data->freq_hz, pkt_data->datarate, pkt_data->rf_power, pkt_data->size );
        }
        else
        {
            LGW_LOG_INFO( "TX IMMEDIATE LoRa: freq=%" PRIu32 " Hz, SF=%" PRIu32 ", pwr=%d dBm, size=%d\n",
                          pkt_data->freq_hz, pkt_data->datarate, pkt_data->rf_power, pkt_data->size );
        }
        break;
    }

    case TIMESTAMPED:
    {
        /* Transmit at specified time.
         *
         * pkt_data->count_us lives in a wrapped µs domain: it is derived from
         * smtc_modem_hal_get_time_in_ms() * 1000, which is a uint32_t multiply
         * that overflows after ~71.6 minutes.  The JIT scheduler works correctly
         * in this domain using unsigned difference arithmetic, but the radio
         * planner compares start_time_ms against smtc_modem_hal_get_time_in_ms()
         * directly (which wraps only at ~49.7 days).  Dividing count_us / 1000
         * directly would give a start_time_ms that appears to be in the past
         * after the first overflow.
         *
         * Fix: express the target as an offset from the current real-ms time.
         * The signed delta in the wrapped µs domain is always correct for
         * targets within ±35 minutes, which covers all LoRaWAN RX windows.
         */
        uint32_t now_ms         = smtc_modem_hal_get_time_in_ms( );
        uint32_t now_us_wrapped = now_ms * 1000U; /* intentional wrap: same domain as count_us */
        int32_t  delta_us       = ( int32_t ) ( pkt_data->count_us - now_us_wrapped );

        lgw_ctx.tx_transaction->scheduler_config.scheduling    = SMTC_RAC_SCHEDULED_TRANSACTION;
        lgw_ctx.tx_transaction->scheduler_config.start_time_ms = now_ms + ( uint32_t ) ( delta_us / 1000 );
        if( pkt_data->modulation == MOD_FSK )
        {
            LGW_LOG_INFO( "TX TIMESTAMPED FSK: freq=%" PRIu32 " Hz, br=%" PRIu32
                          " bps, pwr=%d dBm, size=%d, time=%" PRIu32 " ms\n",
                          pkt_data->freq_hz, pkt_data->datarate, pkt_data->rf_power, pkt_data->size,
                          lgw_ctx.tx_transaction->scheduler_config.start_time_ms );
        }
        else
        {
            LGW_LOG_INFO( "TX TIMESTAMPED LoRa: freq=%" PRIu32 " Hz, SF=%" PRIu32 ", pwr=%d dBm, size=%d, time=%" PRIu32
                          " ms\n",
                          pkt_data->freq_hz, pkt_data->datarate, pkt_data->rf_power, pkt_data->size,
                          lgw_ctx.tx_transaction->scheduler_config.start_time_ms );
        }
        break;
    }

    case ON_GPS:
    {
        /* GPS-triggered TX not supported */
        LGW_LOG_ERROR( "TX mode ON_GPS is not supported\n" );
        return LGW_HAL_ERROR;
    }

    default:
    {
        LGW_LOG_ERROR( "Invalid TX mode: %d\n", pkt_data->tx_mode );
        return LGW_HAL_ERROR;
    }
    }

    /* Submit TX transaction (tx_status already set to TX_SCHEDULED) */
    smtc_rac_return_code_t rc = SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( lgw_ctx.tx_radio_access_id ) );

    if( rc != SMTC_RAC_SUCCESS )
    {
        LGW_LOG_ERROR( "Failed to submit TX transaction: %d\n", rc );
        lgw_state_set_tx_status( TX_FREE );
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_status( uint8_t rf_chain, uint8_t select, uint8_t* code )
{
    CHECK_NULL( code );

    if( rf_chain >= LGW_RF_CHAIN_NB )
    {
        LGW_LOG_ERROR( "Invalid RF chain: %d\n", rf_chain );
        return LGW_HAL_ERROR;
    }

    if( select == TX_STATUS )
    {
        *code = lgw_state_get_is_started( ) ? lgw_state_get_tx_status( ) : TX_OFF;
    }
    else if( select == RX_STATUS )
    {
        *code = lgw_state_get_is_started( ) ? lgw_state_get_rx_status( ) : RX_OFF;
    }
    else
    {
        LGW_LOG_ERROR( "Invalid status select: %d\n", select );
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_abort_tx( uint8_t rf_chain )
{
    if( rf_chain >= LGW_RF_CHAIN_NB )
    {
        LGW_LOG_ERROR( "Invalid RF chain: %d\n", rf_chain );
        return LGW_HAL_ERROR;
    }

    if( !lgw_state_get_is_started( ) )
    {
        LGW_LOG_ERROR( "HAL is not started\n" );
        return LGW_HAL_ERROR;
    }

    /* Check if there's a TX to abort */
    uint8_t tx_status_snap = lgw_state_get_tx_status( );
    if( tx_status_snap != TX_SCHEDULED && tx_status_snap != TX_EMITTING )
    {
        LGW_LOG_INFO( "No TX to abort (status=%d)\n", tx_status_snap );
        return LGW_HAL_SUCCESS;
    }

    /* Abort the TX (the completion callback also sets TX_FREE; doing it here too
     * is idempotent). */
    smtc_rac_return_code_t rc = smtc_rac_abort_radio_submit( lgw_ctx.tx_radio_access_id );

    if( rc != SMTC_RAC_SUCCESS )
    {
        LGW_LOG_ERROR( "Failed to abort TX transaction: %d\n", rc );
        return LGW_HAL_ERROR;
    }

    lgw_state_set_tx_status( TX_FREE );
    LGW_LOG_INFO( "TX aborted\n" );

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_trigcnt( uint32_t* trig_cnt_us )
{
    CHECK_NULL( trig_cnt_us );

    return LGW_HAL_ERROR; /* unsupported */
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_instcnt( uint32_t* inst_cnt_us )
{
    CHECK_NULL( inst_cnt_us );

    /* Get millisecond time and convert to microseconds */
    uint32_t time_ms = smtc_modem_hal_get_time_in_ms( );
    *inst_cnt_us     = time_ms * 1000;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

const char* lgw_version_info( void )
{
    return HAL_VERSION_STRING;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t lgw_time_on_air( const struct lgw_pkt_tx_s* packet )
{
    if( packet == NULL )
    {
        return 0;
    }

    /* Get the radio (RALF) to access RAL functions */
    ralf_t* radio = smtc_rac_get_radio( );
    if( radio == NULL )
    {
        return 0;
    }

    if( packet->modulation == MOD_FSK )
    {
        /* FSK time on air calculation */
        ral_gfsk_mod_params_t mod_params = {
            .br_in_bps    = packet->datarate,
            .fdev_in_hz   = ( uint32_t ) packet->f_dev * 1000,
            .bw_dsb_in_hz = FSK_BANDWIDTH_HZ,
            .pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1,
        };

        ral_gfsk_pkt_params_t pkt_params = {
            .preamble_len_in_bits  = ( packet->preamble > 0 ) ? ( packet->preamble * 8 ) : FSK_PREAMBLE_SIZE_BITS,
            .preamble_detector     = RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS,
            .sync_word_len_in_bits = FSK_SYNC_WORD_SIZE * 8,
            .address_filtering     = RAL_GFSK_ADDRESS_FILTERING_DISABLE,
            .header_type           = packet->no_header ? RAL_GFSK_PKT_FIX_LEN : RAL_GFSK_PKT_VAR_LEN,
            .pld_len_in_bytes      = packet->size,
            .crc_type              = packet->no_crc ? RAL_GFSK_CRC_OFF : RAL_GFSK_CRC_2_BYTES_INV,
            .dc_free               = RAL_GFSK_DC_FREE_WHITENING,
        };

        return ral_get_gfsk_time_on_air_in_ms( &radio->ral, &pkt_params, &mod_params );
    }
    else if( packet->modulation == MOD_LORA )
    {
        /* LoRa time on air calculation */
        ral_lora_sf_t sf = convert_sf_to_ral( packet->datarate );
        ral_lora_bw_t bw = convert_bw_to_ral( packet->bandwidth );
        ral_lora_cr_t cr = convert_cr_to_ral( packet->coderate );

        /* Calculate LDRO (Low Data Rate Optimization) */
        uint8_t ldro = SET_PPM_ON( packet->bandwidth, packet->datarate ) ? 1 : 0;

        ral_lora_mod_params_t mod_params = {
            .sf   = sf,
            .bw   = bw,
            .cr   = cr,
            .ldro = ldro,
        };

        ral_lora_pkt_params_t pkt_params = {
            .preamble_len_in_symb = ( packet->preamble > 0 ) ? packet->preamble : STD_LORA_PREAMBLE,
            .header_type          = packet->no_header ? RAL_LORA_PKT_IMPLICIT : RAL_LORA_PKT_EXPLICIT,
            .pld_len_in_bytes     = packet->size,
            .crc_is_on            = !packet->no_crc,
            .invert_iq_is_on      = packet->invert_pol,
        };

        return ral_get_lora_time_on_air_in_ms( &radio->ral, &pkt_params, &mod_params );
    }

    return 0;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- RX CALLBACKS ------------------------------------------------------------
 */

static void rx_pre_transaction_callback( void )
{
    /* Called when the RX transaction is about to start */
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void rx_post_transaction_callback( rp_status_t status )
{
    switch( status )
    {
    case RP_STATUS_RX_PACKET:
    {
        /* Packet received - add to queue */
        struct lgw_pkt_rx_s pkt;
        memset( &pkt, 0, sizeof( pkt ) );

        /* Fill common packet metadata */
        pkt.freq_hz     = lgw_ctx.rxrf_conf[0].freq_hz;
        pkt.freq_offset = 0;
        pkt.if_chain    = 0;
        pkt.rf_chain    = 0;
        pkt.status      = STAT_CRC_OK;
        pkt.modulation  = lgw_ctx.rx_modulation;
        pkt.datarate    = lgw_ctx.rxif_conf[0].datarate;
        pkt.rssic       = ( float ) lgw_ctx.rx_transaction->smtc_rac_data_result.rssi_result;
        pkt.rssis       = pkt.rssic;
        pkt.size        = lgw_ctx.rx_transaction->smtc_rac_data_result.rx_size;
        pkt.count_us    = lgw_ctx.rx_transaction->smtc_rac_data_result.radio_end_timestamp_ms * 1000;
        pkt.crc         = 0;

        /* Fill modulation-specific metadata */
        if( lgw_ctx.rx_modulation == MOD_FSK )
        {
            /* FSK-specific: no SNR, bandwidth not typically reported */
            pkt.bandwidth = BW_UNDEFINED;
            pkt.coderate  = CR_UNDEFINED;
            pkt.snr       = 0.0f; /* SNR not applicable for FSK */
            pkt.snr_min   = 0.0f;
            pkt.snr_max   = 0.0f;
        }
        else
        {
            /* LoRa-specific */
            pkt.bandwidth = lgw_ctx.rxif_conf[0].bandwidth;
            pkt.coderate  = CR_LORA_4_5;
            pkt.snr       = ( float ) lgw_ctx.rx_transaction->smtc_rac_data_result.snr_result;
            pkt.snr_min   = pkt.snr;
            pkt.snr_max   = pkt.snr;
        }

        /* Copy payload */
        if( pkt.size > 0 && pkt.size <= sizeof( pkt.payload ) )
        {
            memcpy( pkt.payload, lgw_ctx.rx_buffer, pkt.size );
        }

        /* Enqueue packet (with mutex) */
        pthread_mutex_lock( &lgw_ctx.rx_queue_mutex );

        if( rx_queue_is_full( ) )
        {
            LGW_LOG_WARN( "RX queue full, dropping oldest packet\n" );
            /* Advance tail to drop oldest */
            lgw_ctx.rx_queue_tail = ( lgw_ctx.rx_queue_tail + 1 ) % RX_QUEUE_SIZE;
        }

        rx_queue_enqueue( &pkt );

        pthread_mutex_unlock( &lgw_ctx.rx_queue_mutex );

        if( lgw_ctx.rx_modulation == MOD_FSK )
        {
            LGW_LOG_INFO( "FSK packet queued: %d bytes, RSSI=%d\n", pkt.size,
                          lgw_ctx.rx_transaction->smtc_rac_data_result.rssi_result );
        }
        else
        {
            LGW_LOG_INFO( "LoRa packet queued: %d bytes, RSSI=%d, SNR=%d\n", pkt.size,
                          lgw_ctx.rx_transaction->smtc_rac_data_result.rssi_result,
                          lgw_ctx.rx_transaction->smtc_rac_data_result.snr_result );
        }
        break;
    }
    case RP_STATUS_RX_CRC_ERROR:
    {
        /* Packet with CRC error - optionally queue with bad status */
        LGW_LOG_WARN( "Packet received with CRC error\n" );
        break;
    }
    case RP_STATUS_RX_TIMEOUT:
    {
        /* RX timeout - just restart */
        LGW_LOG_INFO( "RX timeout\n" );
        break;
    }
    case RP_STATUS_TASK_ABORTED:
    {
        LGW_LOG_WARN( "RX aborted\n" );
        break;
    }
    default:
    {
        LGW_LOG_WARN( "Unexpected RX RAC status: %d\n", status );
        break;
    }
    }

    /* Re-submit RX to keep listening; if it fails, RX is no longer scheduled */
    if( lgw_state_get_is_started( ) )
    {
        if( submit_rx_transaction( ) != LGW_HAL_SUCCESS )
        {
            lgw_state_set_rx_status( RX_OFF );
        }
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int submit_rx_transaction( void )
{
    /* Clear RX buffer */
    memset( lgw_ctx.rx_buffer, 0, sizeof( lgw_ctx.rx_buffer ) );

    /* Set start time to now (ASAP) */
    lgw_ctx.rx_transaction->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( );

    /* Submit RX transaction */
    smtc_rac_return_code_t rc = SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( lgw_ctx.rx_radio_access_id ) );

    if( rc != SMTC_RAC_SUCCESS )
    {
        LGW_LOG_ERROR( "Failed to submit RX transaction: %d\n", rc );
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/*
 * -----------------------------------------------------------------------------
 * --- TX CALLBACKS ------------------------------------------------------------
 */

static void tx_pre_transaction_callback( void )
{
    /* Called when the TX transaction is about to start */
    lgw_state_set_tx_status( TX_EMITTING );
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void tx_post_transaction_callback( rp_status_t status )
{
    switch( status )
    {
    case RP_STATUS_TX_DONE:
    {
        break;
    }

    case RP_STATUS_TASK_ABORTED:
    {
        LGW_LOG_WARN( "TX was aborted\n" );
        break;
    }

    default:
    {
        LGW_LOG_WARN( "Unexpected TX RAC status: %d\n", status );
        break;
    }
    }

    /* TX complete - set status back to free */
    lgw_state_set_tx_status( TX_FREE );
}

/*
 * -----------------------------------------------------------------------------
 * --- QUEUE FUNCTIONS ---------------------------------------------------------
 */

static bool rx_queue_is_full( void )
{
    return ( ( lgw_ctx.rx_queue_head + 1 ) % RX_QUEUE_SIZE ) == lgw_ctx.rx_queue_tail;
}

static bool rx_queue_is_empty( void )
{
    return lgw_ctx.rx_queue_head == lgw_ctx.rx_queue_tail;
}

static void rx_queue_enqueue( const struct lgw_pkt_rx_s* pkt )
{
    memcpy( &lgw_ctx.rx_queue[lgw_ctx.rx_queue_head], pkt, sizeof( struct lgw_pkt_rx_s ) );
    lgw_ctx.rx_queue_head = ( lgw_ctx.rx_queue_head + 1 ) % RX_QUEUE_SIZE;
}

static bool rx_queue_dequeue( struct lgw_pkt_rx_s* pkt )
{
    if( rx_queue_is_empty( ) )
    {
        return false;
    }

    memcpy( pkt, &lgw_ctx.rx_queue[lgw_ctx.rx_queue_tail], sizeof( struct lgw_pkt_rx_s ) );
    lgw_ctx.rx_queue_tail = ( lgw_ctx.rx_queue_tail + 1 ) % RX_QUEUE_SIZE;

    return true;
}

/*
 * -----------------------------------------------------------------------------
 * --- CONVERSION HELPERS ------------------------------------------------------
 */

static ral_lora_sf_t convert_sf_to_ral( uint32_t sf )
{
    switch( sf )
    {
    case DR_LORA_SF5:
        return RAL_LORA_SF5;
    case DR_LORA_SF6:
        return RAL_LORA_SF6;
    case DR_LORA_SF7:
        return RAL_LORA_SF7;
    case DR_LORA_SF8:
        return RAL_LORA_SF8;
    case DR_LORA_SF9:
        return RAL_LORA_SF9;
    case DR_LORA_SF10:
        return RAL_LORA_SF10;
    case DR_LORA_SF11:
        return RAL_LORA_SF11;
    case DR_LORA_SF12:
        return RAL_LORA_SF12;
    default:
        return RAL_LORA_SF7;
    }
}

static ral_lora_bw_t convert_bw_to_ral( uint8_t bw )
{
    switch( bw )
    {
    case BW_125KHZ:
        return RAL_LORA_BW_125_KHZ;
    case BW_250KHZ:
        return RAL_LORA_BW_250_KHZ;
    case BW_500KHZ:
        return RAL_LORA_BW_500_KHZ;
    default:
        return RAL_LORA_BW_125_KHZ;
    }
}

static ral_lora_cr_t convert_cr_to_ral( uint8_t cr )
{
    switch( cr )
    {
    case CR_LORA_4_5:
        return RAL_LORA_CR_4_5;
    case CR_LORA_4_6:
        return RAL_LORA_CR_4_6;
    case CR_LORA_4_7:
        return RAL_LORA_CR_4_7;
    case CR_LORA_4_8:
        return RAL_LORA_CR_4_8;
    default:
        return RAL_LORA_CR_4_5;
    }
}
