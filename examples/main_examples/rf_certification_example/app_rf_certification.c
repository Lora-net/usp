/**
 * @file      app_rf_certification.c
 *
 * @brief     RF Certification LoRa TX example for LR2021 chip
 *            Sends LoRa packets continuously until user button is pressed
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

#include "app_rf_certification.h"
#include "main_rf_certification.h"
#include "app_notification.h"

#include <stdint.h>
#include <string.h>
#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

// Use unified logging system
#define RAC_LOG_APP_PREFIX "RF-CERT"
#include "smtc_rac_log.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// Helper macros for logging
#define RF_CERT_LOG_INFO( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )
#define RF_CERT_LOG_WARN( ... ) SMTC_HAL_TRACE_WARNING( __VA_ARGS__ )
#define RF_CERT_LOG_ERROR( ... ) SMTC_HAL_TRACE_ERROR( __VA_ARGS__ )
#define RF_CERT_LOG_CONFIG( ... ) SMTC_HAL_TRACE_INFO( "CONFIG " __VA_ARGS__ )
#define RF_CERT_LOG_TX( ... ) SMTC_HAL_TRACE_INFO( "TX " __VA_ARGS__ )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static const smtc_rac_lora_syncword_t SYNC_WORD      = LORA_PUBLIC_NETWORK_SYNCWORD;  // (alias)
static const uint32_t                 INTER_TX_DELAY = 0;                             // ms between transmissions

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct rf_certification_s
{
    uint32_t                  packet_count;                     // number of sent packets
    bool                      transmission_active;              // true if transmission is ongoing
    uint8_t                   payload[PAYLOAD_SIZE];            // bytes buffer
    uint8_t                   radio_access_id;                  // store result of `smtc_rac_open_radio`
    rf_certification_state_t  rf_certification_state;           // state machine for rf certification
    smtc_rac_context_t*       transaction;                      // associated transaction
    rf_certification_region_t region;                           // region to use
    uint8_t                   fcc_sweep_fhss_counter;           // counter for FHSS sweep in FCC region
    uint32_t fcc_sweep_frequency[FCC_SWEEP_FHSS_CHANNELS_NUM];  // frequency for FHSS sweep in FCC region
    bool     fcc_sweep_started;                                 // true if FHSS sweep in FCC region is started
    bool     fcc_sweep_hybrid_started;                          // true if hybrid sweep in FCC region is started
} rf_certification_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static rf_certification_t rf_certification = {
    .packet_count           = 0,
    .transmission_active    = false,
    .payload                = { 0 },
    .radio_access_id        = 0,  // set in `rf_certification_init`
    .rf_certification_state = RF_CERT_STATE_IDLE,
    .transaction            = NULL,
#ifdef RF_CERT_REGION
    .region = RF_CERT_REGION,
#else
    .region = RF_CERT_REGION_ETSI,
#endif
    .fcc_sweep_fhss_counter   = 0,
    .fcc_sweep_frequency      = { 0 },
    .fcc_sweep_started        = false,
    .fcc_sweep_hybrid_started = false,
};
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void print_rf_certification_banner( void );
static void print_lora_configuration( void );
static void print_fsk_configuration( void );
static void print_lrfhss_configuration( void );
static void print_flrc_configuration( void );
static void print_notification( char* notification );

/** @brief set `rf_certification.transaction` and start the transmission */
static void rf_certification_tx( void );

/** @brief switch on `SMTC_LED_TX` */
static void pre_rf_certification_callback( void );

/** @brief switch off `SMTC_PF_LED_TX` and call `rf_certification_tx` again */
static void post_rf_certification_callback( rp_status_t status );

/** @brief change the LoRa parameters */
static void change_lora_parameters( uint32_t freq, uint8_t sf, uint8_t bw, uint8_t power );

/** @brief change the LoRa parameters with custom payload size */
static void change_lora_parameters_with_payload( uint32_t freq, uint8_t sf, uint8_t bw, uint8_t power,
                                                 uint16_t payload_size );

/** @brief change the FSK parameters */
static void change_fsk_parameters( uint32_t freq, uint8_t power );

/** @brief change the LR-FHSS parameters */
static void change_lrfhss_parameters( uint32_t freq, uint8_t power );

/** @brief change the FLRC parameters */
static void change_flrc_parameters( uint32_t freq, uint8_t power, uint32_t bitrate, uint32_t bw_dsb_in_hz );

/** @brief RF certification state machine for ETSI region */
static void rf_certification_etsi_state_machine( void );

/** @brief RF certification state machine for ARIB region */
static void rf_certification_arib_state_machine( void );

/** @brief RF certification state machine for FCC region */
static void rf_certification_fcc_state_machine( void );

/** @brief generate pseudo-random channel map for subband for FHSS sweep in FCC region*/
static void fcc_sweep_test_fhss( void );

/** @brief generate pseudo-random channel map for subband for hybrid sweep in FCC region*/
static void fcc_sweep_test_hybrid_fhss( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void rf_certification_init( void )
{
    // Print banner and configuration
    print_rf_certification_banner( );

    // Initialize payload with test pattern - create a recognizable pattern
    const char* header = "RF_CERT";
    memcpy( rf_certification.payload, header, strlen( header ) );
    for( uint8_t i = strlen( header ); i < PAYLOAD_SIZE; i++ )
    {
        rf_certification.payload[i] = i;
    }

    // Initialize static struct with RAC API
    rf_certification.radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );
    rf_certification.transaction     = smtc_rac_get_context( rf_certification.radio_access_id );
    rf_certification.transaction->scheduler_config.callback_post_radio_transaction = post_rf_certification_callback;
    rf_certification.transaction->modulation_type                                  = SMTC_RAC_MODULATION_LORA;
    rf_certification.transaction->radio_params.lora.is_tx                          = true;
    rf_certification.transaction->radio_params.lora.is_ranging_exchange            = false;
    rf_certification.transaction->radio_params.lora.frequency_in_hz                = RF_FREQ_IN_HZ;
    rf_certification.transaction->radio_params.lora.tx_power_in_dbm                = TX_OUTPUT_POWER_DBM;
    rf_certification.transaction->radio_params.lora.sf                             = LORA_SPREADING_FACTOR;
    rf_certification.transaction->radio_params.lora.bw                             = LORA_BANDWIDTH;
    rf_certification.transaction->radio_params.lora.cr                             = LORA_CODING_RATE;
    rf_certification.transaction->radio_params.lora.preamble_len_in_symb           = LORA_PREAMBLE_LENGTH;
    rf_certification.transaction->radio_params.lora.header_type                    = LORA_PKT_LEN_MODE;
    rf_certification.transaction->radio_params.lora.invert_iq_is_on                = LORA_IQ;
    rf_certification.transaction->radio_params.lora.crc_is_on                      = LORA_CRC;
    rf_certification.transaction->radio_params.lora.sync_word                      = SYNC_WORD;
    rf_certification.transaction->radio_params.lora.rx_timeout_ms                  = 0;  // not used
    rf_certification.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer     = rf_certification.payload;
    rf_certification.transaction->smtc_rac_data_buffer_setup.rx_payload_buffer     = rf_certification.payload;
    rf_certification.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer =
        sizeof( rf_certification.payload );
    rf_certification.transaction->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer =
        sizeof( rf_certification.payload );
    rf_certification.transaction->radio_params.lora.max_rx_size    = PAYLOAD_SIZE;
    rf_certification.transaction->radio_params.lora.tx_size        = PAYLOAD_SIZE;
    rf_certification.transaction->smtc_rac_data_result.rssi_result = 0;  // set by rac after each transaction
    rf_certification.transaction->smtc_rac_data_result.snr_result  = 0;  // set by rac after each transaction
    rf_certification.transaction->smtc_rac_data_result.radio_start_timestamp_ms =
        0;  // set by rac after each transaction

    rf_certification.transaction->scheduler_config.start_time_ms                  = 0;  // set at each transaction
    rf_certification.transaction->scheduler_config.scheduling                     = SMTC_RAC_ASAP_TRANSACTION;
    rf_certification.transaction->scheduler_config.callback_pre_radio_transaction = pre_rf_certification_callback;

    /* LBT config */
    if( rf_certification.region == RF_CERT_REGION_ARIB )
    {
        rf_certification.transaction->lbt_context.lbt_enabled        = true;
        rf_certification.transaction->lbt_context.listen_duration_ms = LBT_SNIFF_DURATION_MS_DEFAULT;
        rf_certification.transaction->lbt_context.threshold_dbm      = LBT_THRESHOLD_DBM_DEFAULT;
        rf_certification.transaction->lbt_context.bandwidth_hz       = LBT_BW_HZ__DEFAULT;
    }
    else
    {
        rf_certification.transaction->lbt_context.lbt_enabled = false;
    }

    RF_CERT_LOG_CONFIG( "RF Certification initialized successfully\n" );
    RF_CERT_LOG_INFO( "Press the blue button to start transmission\n" );

    // Initialize notifications
    if( NOTIFICATIONS_ENABLED == true )
    {
        char* app_mode = "";
        if( RX_ONLY == true )
        {
            app_mode = "RX_ONLY";
            notification_init_rx_only( print_notification );
        }
        else
        {
            app_mode = "CERTIFICATION";
            notification_init_certification( rf_certification.transaction );
        }
        RF_CERT_LOG_INFO( "Notifications enabled (%s)\n", app_mode );
    }

    // Start transmission
}
void rf_certification_init_fsk( void )
{
    // Print banner and configuration
    print_rf_certification_banner( );

    // Initialize payload with test pattern - create a recognizable pattern
    const char* header = "RF_CERT_FSK";
    memcpy( rf_certification.payload, header, strlen( header ) );
    for( uint8_t i = strlen( header ); i < PAYLOAD_SIZE; i++ )
    {
        rf_certification.payload[i] = i;
    }

    // Configure FSK modulation
    /*Same frequency plan as abovementioned LORA, with LoRaWAN-compliant FSK parameters
    (bw=117khz, bitrate=50kbps, fdev=25khz, syncword=0xc194c1, crc=crc-16-ccitt, gaussian filter=BT 1,0, dc free
    encoding= whitening encoding)*/
    static const uint8_t sync_word_gfsk[]                          = { 0xC1, 0x94, 0xC1 };
    rf_certification.transaction->modulation_type                  = SMTC_RAC_MODULATION_FSK;
    rf_certification.transaction->radio_params.fsk.is_tx           = true;
    rf_certification.transaction->radio_params.fsk.frequency_in_hz = RF_FREQ_IN_HZ;
    rf_certification.transaction->radio_params.fsk.tx_power_in_dbm = TX_OUTPUT_POWER_DBM;
    rf_certification.transaction->radio_params.fsk.br_in_bps       = FSK_BITRATE;
    rf_certification.transaction->radio_params.fsk.fdev_in_hz      = FSK_FDEV;
    rf_certification.transaction->radio_params.fsk.bw_dsb_in_hz    = FSK_BANDWIDTH;
    rf_certification.transaction->radio_params.fsk.preamble_len_in_bits =
        FSK_PREAMBLE_LENGTH * 8;  // Convert bytes to bits
    rf_certification.transaction->radio_params.fsk.sync_word_len_in_bits =
        FSK_SYNC_WORD_LENGTH * 8;  // Convert bytes to bits
    rf_certification.transaction->radio_params.fsk.sync_word      = ( uint8_t* ) sync_word_gfsk;
    rf_certification.transaction->radio_params.fsk.crc_type       = FSK_CRC;
    rf_certification.transaction->radio_params.fsk.whitening_seed = FSK_WHITENING ? FSK_WHITENING_SEED : 0x0000;
    rf_certification.transaction->radio_params.fsk.header_type    = FSK_PACKET_TYPE;
    rf_certification.transaction->radio_params.fsk.rx_timeout_ms  = 0;  // not used for TX

    // Set defaults for advanced parameters
    rf_certification.transaction->radio_params.fsk.pulse_shape       = RAL_GFSK_PULSE_SHAPE_BT_1;
    rf_certification.transaction->radio_params.fsk.preamble_detector = RAL_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
    rf_certification.transaction->radio_params.fsk.dc_free           = RAL_GFSK_DC_FREE_WHITENING;
    rf_certification.transaction->radio_params.fsk.crc_seed          = FSK_CRC_SEED;
    rf_certification.transaction->radio_params.fsk.crc_polynomial    = FSK_CRC_POLYNOMIAL;

    // Configure data and scheduler
    rf_certification.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer = rf_certification.payload;
    rf_certification.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer =
        sizeof( rf_certification.payload );
    rf_certification.transaction->radio_params.fsk.tx_size         = PAYLOAD_SIZE;
    rf_certification.transaction->smtc_rac_data_result.rssi_result = 0;  // set by rac after each transaction
    rf_certification.transaction->smtc_rac_data_result.snr_result  = 0;  // set by rac after each transaction

    rf_certification.transaction->smtc_rac_data_result.radio_start_timestamp_ms =
        0;  // set by rac after each transaction

    rf_certification.transaction->scheduler_config.start_time_ms                  = 0;  // set at each transaction
    rf_certification.transaction->scheduler_config.scheduling                     = SMTC_RAC_ASAP_TRANSACTION;
    rf_certification.transaction->scheduler_config.callback_pre_radio_transaction = pre_rf_certification_callback;

    RF_CERT_LOG_CONFIG( "RF Certification FSK initialized successfully\n" );
    RF_CERT_LOG_INFO( "Press the blue button to start FSK transmission\n" );
}
void rf_certification_init_lrfhss( void )
{
    // Print banner and configuration
    print_rf_certification_banner( );

    // Initialize payload with test pattern - create a recognizable pattern
    const char* header = "RF_CERT_LRFHSS";
    memcpy( rf_certification.payload, header, strlen( header ) );
    for( uint8_t i = strlen( header ); i < PAYLOAD_SIZE; i++ )
    {
        rf_certification.payload[i] = i;
    }

    // Configure LR-FHSS modulation
    rf_certification.transaction->modulation_type                     = SMTC_RAC_MODULATION_LRFHSS;
    rf_certification.transaction->radio_params.lrfhss.is_tx           = true;  // LR-FHSS is transmission only
    rf_certification.transaction->radio_params.lrfhss.frequency_in_hz = RF_FREQ_IN_HZ;
    rf_certification.transaction->radio_params.lrfhss.tx_power_in_dbm = TX_OUTPUT_POWER_DBM;

    // LR-FHSS specific parameters - optimized for good range and reliability
    rf_certification.transaction->radio_params.lrfhss.coding_rate     = LRFHSS_CODING_RATE;
    rf_certification.transaction->radio_params.lrfhss.bandwidth       = LRFHSS_BANDWIDTH;
    rf_certification.transaction->radio_params.lrfhss.grid            = LRFHSS_GRID;
    rf_certification.transaction->radio_params.lrfhss.enable_hopping  = LRFHSS_ENABLE_HOPPING;
    rf_certification.transaction->radio_params.lrfhss.sync_word       = NULL;  // Use default sync word
    rf_certification.transaction->radio_params.lrfhss.device_offset   = LRFHSS_DEVICE_OFFSET;
    rf_certification.transaction->radio_params.lrfhss.hop_sequence_id = LRFHSS_HOP_SEQUENCE_ID;

    // Configure data and scheduler
    rf_certification.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer = rf_certification.payload;
    rf_certification.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer =
        sizeof( rf_certification.payload );
    rf_certification.transaction->radio_params.lrfhss.tx_size      = LRFHSS_PAYLOAD_SIZE;
    rf_certification.transaction->smtc_rac_data_result.rssi_result = 0;  // set by rac after each transaction
    rf_certification.transaction->smtc_rac_data_result.snr_result  = 0;  // set by rac after each transaction

    rf_certification.transaction->smtc_rac_data_result.radio_start_timestamp_ms =
        0;  // set by rac after each transaction

    rf_certification.transaction->scheduler_config.start_time_ms                  = 0;  // set at each transaction
    rf_certification.transaction->scheduler_config.scheduling                     = SMTC_RAC_ASAP_TRANSACTION;
    rf_certification.transaction->scheduler_config.callback_pre_radio_transaction = pre_rf_certification_callback;

    RF_CERT_LOG_CONFIG( "RF Certification LR-FHSS initialized successfully\n" );
    RF_CERT_LOG_INFO( "Press the blue button to start LR-FHSS transmission\n" );
}

void rf_certification_init_flrc( void )
{
    // Print banner and configuration
    print_rf_certification_banner( );

    // Initialize payload with test pattern - create a recognizable pattern
    const char* header = "RF_CERT_FLRC";
    memcpy( rf_certification.payload, header, strlen( header ) );
    for( uint8_t i = strlen( header ); i < PAYLOAD_SIZE; i++ )
    {
        rf_certification.payload[i] = i;
    }

    // Configure FLRC modulation
    static const uint8_t sync_word_flrc[]                           = { 0x55, 0x55, 0x55, 0x55 };  // 4-byte sync word
    rf_certification.transaction->modulation_type                   = SMTC_RAC_MODULATION_FLRC;
    rf_certification.transaction->radio_params.flrc.is_tx           = true;
    rf_certification.transaction->radio_params.flrc.frequency_in_hz = RF_FREQ_IN_HZ;
    rf_certification.transaction->radio_params.flrc.tx_power_in_dbm = TX_OUTPUT_POWER_DBM;

    // FLRC modulation parameters
    rf_certification.transaction->radio_params.flrc.br_in_bps    = FLRC_BITRATE;
    rf_certification.transaction->radio_params.flrc.bw_dsb_in_hz = FLRC_BANDWIDTH;
    rf_certification.transaction->radio_params.flrc.cr           = FLRC_CODING_RATE;
    rf_certification.transaction->radio_params.flrc.pulse_shape  = FLRC_PULSE_SHAPE;

    // FLRC packet parameters
    rf_certification.transaction->radio_params.flrc.preamble_len_in_bits = FLRC_PREAMBLE_LENGTH;
    rf_certification.transaction->radio_params.flrc.sync_word_len        = FLRC_SYNC_WORD_LENGTH;
    rf_certification.transaction->radio_params.flrc.tx_syncword          = RAL_FLRC_TX_SYNCWORD_1;
    rf_certification.transaction->radio_params.flrc.match_sync_word      = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    rf_certification.transaction->radio_params.flrc.pld_is_fix           = FLRC_PACKET_TYPE;
    rf_certification.transaction->radio_params.flrc.crc_type             = FLRC_CRC_TYPE;

    // Advanced parameters
    rf_certification.transaction->radio_params.flrc.sync_word      = ( uint8_t* ) sync_word_flrc;
    rf_certification.transaction->radio_params.flrc.crc_seed       = 0;  // Default radio CRC seed
    rf_certification.transaction->radio_params.flrc.crc_polynomial = 0;  // Default radio CRC polynomial
    rf_certification.transaction->radio_params.flrc.rx_timeout_ms  = 0;  // not used for TX

    // Configure data and scheduler
    rf_certification.transaction->smtc_rac_data_buffer_setup.tx_payload_buffer = rf_certification.payload;
    rf_certification.transaction->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer =
        sizeof( rf_certification.payload );
    rf_certification.transaction->radio_params.flrc.tx_size        = PAYLOAD_SIZE;
    rf_certification.transaction->smtc_rac_data_result.rssi_result = 0;  // set by rac after each transaction
    rf_certification.transaction->smtc_rac_data_result.snr_result  = 0;  // set by rac after each transaction

    rf_certification.transaction->smtc_rac_data_result.radio_start_timestamp_ms =
        0;  // set by rac after each transaction

    rf_certification.transaction->scheduler_config.start_time_ms                  = 0;  // set at each transaction
    rf_certification.transaction->scheduler_config.scheduling                     = SMTC_RAC_ASAP_TRANSACTION;
    rf_certification.transaction->scheduler_config.callback_pre_radio_transaction = pre_rf_certification_callback;

    RF_CERT_LOG_CONFIG( "RF Certification FLRC initialized successfully\n" );
    RF_CERT_LOG_INFO( "Press the blue button to start FLRC transmission\n" );
}

void rf_certification_on_button_press( void )
{
    rf_certification.transmission_active = true;
    if( rf_certification.region == RF_CERT_REGION_ETSI )
    {
        rf_certification_etsi_state_machine( );
    }
    else if( rf_certification.region == RF_CERT_REGION_ARIB )
    {
        rf_certification_arib_state_machine( );
    }
    else if( rf_certification.region == RF_CERT_REGION_FCC )
    {
        rf_certification_fcc_state_machine( );
    }
}

bool rf_certification_is_transmission_active( void )
{
    return rf_certification.transmission_active;
}

void rf_certification_send_packet( void )
{
    // This function is now handled by the RAC scheduler and callbacks
    // No need to implement anything here as rf_certification_tx() handles the transmission
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
static void rf_certification_etsi_state_machine( void )
{
    switch( rf_certification.rf_certification_state )
    {
    case RF_CERT_STATE_IDLE:

        /*Lora modulation*/
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ETSI_863_1_SF7_BW_125_POWER_14_DBM;
        change_lora_parameters( RF_FREQUENCY_863_1, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_14_DBM );
        print_lora_configuration( );
        notification_wrap( rf_certification_tx );
        break;

    case RF_CERT_STATE_LORA_ETSI_863_1_SF7_BW_125_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ETSI_863_1_SF12_BW_125_POWER_14_DBM;
        change_lora_parameters( RF_FREQUENCY_863_1, RAL_LORA_SF12, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LORA_ETSI_863_1_SF12_BW_125_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ETSI_868_3_SF7_BW_125_POWER_14_DBM;
        change_lora_parameters( RF_FREQUENCY_868_3, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LORA_ETSI_868_3_SF7_BW_125_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ETSI_868_3_SF12_BW_125_POWER_14_DBM;
        change_lora_parameters( RF_FREQUENCY_868_3, RAL_LORA_SF12, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LORA_ETSI_868_3_SF12_BW_125_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ETSI_868_5_SF7_BW_125_POWER_14_DBM;
        change_lora_parameters( RF_FREQUENCY_868_5, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LORA_ETSI_868_5_SF7_BW_125_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ETSI_868_5_SF12_BW_125_POWER_14_DBM;
        change_lora_parameters( RF_FREQUENCY_868_5, RAL_LORA_SF12, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LORA_ETSI_868_5_SF12_BW_125_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ETSI_868_3_SF7_BW_250_POWER_14_DBM;
        change_lora_parameters( RF_FREQUENCY_868_3, RAL_LORA_SF7, RAL_LORA_BW_250_KHZ, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LORA_ETSI_868_3_SF7_BW_250_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ETSI_866_5_SF7_BW_125_POWER_14_DBM;
        change_lora_parameters( RF_FREQUENCY_866_5, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LORA_ETSI_866_5_SF7_BW_125_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ETSI_866_5_SF12_BW_125_POWER_14_DBM;
        change_lora_parameters( RF_FREQUENCY_866_5, RAL_LORA_SF12, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LORA_ETSI_866_5_SF12_BW_125_POWER_14_DBM:

        /*FSK modulation*/
        rf_certification.rf_certification_state = RF_CERT_STATE_FSK_ETSI_863_1_POWER_14_DBM;
        rf_certification_init_fsk( );
        change_fsk_parameters( RF_FREQUENCY_863_1, EXPECTED_PWR_14_DBM );
        print_fsk_configuration( );
        break;
    case RF_CERT_STATE_FSK_ETSI_863_1_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FSK_ETSI_866_5_POWER_14_DBM;
        change_fsk_parameters( RF_FREQUENCY_866_5, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_FSK_ETSI_866_5_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FSK_ETSI_868_1_POWER_14_DBM;
        change_fsk_parameters( RF_FREQUENCY_868_1, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_FSK_ETSI_868_1_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FSK_ETSI_868_3_POWER_14_DBM;
        change_fsk_parameters( RF_FREQUENCY_868_3, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_FSK_ETSI_868_3_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FSK_ETSI_868_5_POWER_14_DBM;
        change_fsk_parameters( RF_FREQUENCY_868_5, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_FSK_ETSI_868_5_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FSK_ETSI_869_9_POWER_14_DBM;
        change_fsk_parameters( RF_FREQUENCY_869_9, EXPECTED_PWR_14_DBM );
        break;
    case RF_CERT_STATE_FSK_ETSI_869_9_POWER_14_DBM:

        /*LR-FHSS modulation*/
        rf_certification.rf_certification_state = RF_CERT_STATE_LR_FHSS_ETSI_863_1_POWER_14_DBM;
        rf_certification_init_lrfhss( );
        change_lrfhss_parameters( RF_FREQUENCY_863_1, EXPECTED_PWR_14_DBM );
        print_lrfhss_configuration( );
        break;
    case RF_CERT_STATE_LR_FHSS_ETSI_863_1_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LR_FHSS_ETSI_866_5_POWER_14_DBM;
        change_lrfhss_parameters( RF_FREQUENCY_866_5, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LR_FHSS_ETSI_866_5_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LR_FHSS_ETSI_868_1_POWER_14_DBM;
        change_lrfhss_parameters( RF_FREQUENCY_868_1, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LR_FHSS_ETSI_868_1_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LR_FHSS_ETSI_868_3_POWER_14_DBM;
        change_lrfhss_parameters( RF_FREQUENCY_868_3, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LR_FHSS_ETSI_868_3_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LR_FHSS_ETSI_868_5_POWER_14_DBM;
        change_lrfhss_parameters( RF_FREQUENCY_868_5, EXPECTED_PWR_14_DBM );
        break;

    case RF_CERT_STATE_LR_FHSS_ETSI_868_5_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LR_FHSS_ETSI_869_9_POWER_14_DBM;
        change_lrfhss_parameters( RF_FREQUENCY_869_9, EXPECTED_PWR_14_DBM );
        break;
    case RF_CERT_STATE_LR_FHSS_ETSI_869_9_POWER_14_DBM:

        /*FLRC modulation*/
        rf_certification.rf_certification_state = RF_CERT_STATE_FLRC_ETSI_863_1_POWER_14_DBM;
        rf_certification_init_flrc( );
        change_flrc_parameters( RF_FREQUENCY_863_1, EXPECTED_PWR_14_DBM, FLRC_BITRATE, FLRC_BANDWIDTH );
        print_flrc_configuration( );
        break;
    case RF_CERT_STATE_FLRC_ETSI_863_1_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FLRC_ETSI_866_5_POWER_14_DBM;
        change_flrc_parameters( RF_FREQUENCY_866_5, EXPECTED_PWR_14_DBM, FLRC_BITRATE, FLRC_BANDWIDTH );
        break;
    case RF_CERT_STATE_FLRC_ETSI_866_5_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FLRC_ETSI_868_1_POWER_14_DBM;
        change_flrc_parameters( RF_FREQUENCY_868_1, EXPECTED_PWR_14_DBM, FLRC_BITRATE, FLRC_BANDWIDTH );
        break;
    case RF_CERT_STATE_FLRC_ETSI_868_1_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FLRC_ETSI_868_3_POWER_14_DBM;
        change_flrc_parameters( RF_FREQUENCY_868_3, EXPECTED_PWR_14_DBM, FLRC_BITRATE, FLRC_BANDWIDTH );
        break;
    case RF_CERT_STATE_FLRC_ETSI_868_3_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FLRC_ETSI_868_5_POWER_14_DBM;
        change_flrc_parameters( RF_FREQUENCY_868_5, EXPECTED_PWR_14_DBM, FLRC_BITRATE, FLRC_BANDWIDTH );
        break;
    case RF_CERT_STATE_FLRC_ETSI_868_5_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FLRC_ETSI_869_9_POWER_14_DBM;
        change_flrc_parameters( RF_FREQUENCY_869_9, EXPECTED_PWR_14_DBM, FLRC_BITRATE, FLRC_BANDWIDTH );
        break;
    case RF_CERT_STATE_FLRC_ETSI_869_9_POWER_14_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_IDLE;
        rf_certification.transmission_active    = false;
        break;
        break;
    default:
        rf_certification.rf_certification_state = RF_CERT_STATE_IDLE;
        rf_certification.transmission_active    = false;
        break;
    }
}
static void rf_certification_arib_state_machine( void )
{
    switch( rf_certification.rf_certification_state )
    {
    case RF_CERT_STATE_IDLE:
        /*LoRa modulation - ARIB region*/
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ARIB_920_6_SF7_BW_125_POWER_9_DBM;
        change_lora_parameters( RF_FREQUENCY_920_6, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_9_DBM );
        print_lora_configuration( );
        notification_wrap( rf_certification_tx );
        break;

    case RF_CERT_STATE_LORA_ARIB_920_6_SF7_BW_125_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ARIB_920_6_SF12_BW_125_POWER_9_DBM;
        change_lora_parameters( RF_FREQUENCY_920_6, RAL_LORA_SF12, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_9_DBM );
        break;

    case RF_CERT_STATE_LORA_ARIB_920_6_SF12_BW_125_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ARIB_922_0_SF7_BW_125_POWER_9_DBM;
        change_lora_parameters( RF_FREQUENCY_922_0, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_9_DBM );
        break;

    case RF_CERT_STATE_LORA_ARIB_922_0_SF7_BW_125_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ARIB_922_0_SF12_BW_125_POWER_9_DBM;
        change_lora_parameters( RF_FREQUENCY_922_0, RAL_LORA_SF12, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_9_DBM );
        break;

    case RF_CERT_STATE_LORA_ARIB_922_0_SF12_BW_125_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ARIB_923_4_SF7_BW_125_POWER_9_DBM;
        change_lora_parameters( RF_FREQUENCY_923_4, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_9_DBM );
        break;

    case RF_CERT_STATE_LORA_ARIB_923_4_SF7_BW_125_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LORA_ARIB_923_4_SF12_BW_125_POWER_9_DBM;
        change_lora_parameters( RF_FREQUENCY_923_4, RAL_LORA_SF12, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_9_DBM );
        break;

    case RF_CERT_STATE_LORA_ARIB_923_4_SF12_BW_125_POWER_9_DBM:
        /*FSK modulation - ARIB region*/
        rf_certification.rf_certification_state = RF_CERT_STATE_FSK_ARIB_920_6_POWER_9_DBM;
        rf_certification_init_fsk( );
        change_fsk_parameters( RF_FREQUENCY_920_6, EXPECTED_PWR_9_DBM );
        print_fsk_configuration( );
        break;

    case RF_CERT_STATE_FSK_ARIB_920_6_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FSK_ARIB_922_0_POWER_9_DBM;
        change_fsk_parameters( RF_FREQUENCY_922_0, EXPECTED_PWR_9_DBM );
        break;

    case RF_CERT_STATE_FSK_ARIB_922_0_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FSK_ARIB_923_4_POWER_9_DBM;
        change_fsk_parameters( RF_FREQUENCY_923_4, EXPECTED_PWR_9_DBM );
        break;

    case RF_CERT_STATE_FSK_ARIB_923_4_POWER_9_DBM:
        /*LR-FHSS modulation - ARIB region*/
        rf_certification.rf_certification_state = RF_CERT_STATE_LR_FHSS_ARIB_920_6_POWER_9_DBM;
        rf_certification_init_lrfhss( );
        change_lrfhss_parameters( RF_FREQUENCY_920_6, EXPECTED_PWR_9_DBM );
        print_lrfhss_configuration( );
        break;

    case RF_CERT_STATE_LR_FHSS_ARIB_920_6_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LR_FHSS_ARIB_922_0_POWER_9_DBM;
        change_lrfhss_parameters( RF_FREQUENCY_922_0, EXPECTED_PWR_9_DBM );
        break;

    case RF_CERT_STATE_LR_FHSS_ARIB_922_0_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_LR_FHSS_ARIB_923_4_POWER_9_DBM;
        change_lrfhss_parameters( RF_FREQUENCY_923_4, EXPECTED_PWR_9_DBM );
        break;

    case RF_CERT_STATE_LR_FHSS_ARIB_923_4_POWER_9_DBM:
        /*FLRC modulation - ARIB region*/
        rf_certification.rf_certification_state = RF_CERT_STATE_FLRC_ARIB_920_6_POWER_9_DBM;
        rf_certification_init_flrc( );
        change_flrc_parameters( RF_FREQUENCY_920_6, EXPECTED_PWR_9_DBM, FLRC_BITRATE, FLRC_BANDWIDTH );
        print_flrc_configuration( );
        break;
    case RF_CERT_STATE_FLRC_ARIB_920_6_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FLRC_ARIB_922_0_POWER_9_DBM;
        change_flrc_parameters( RF_FREQUENCY_922_0, EXPECTED_PWR_9_DBM, FLRC_BITRATE, FLRC_BANDWIDTH );
        break;
    case RF_CERT_STATE_FLRC_ARIB_922_0_POWER_9_DBM:
        rf_certification.rf_certification_state = RF_CERT_STATE_FLRC_ARIB_923_4_POWER_9_DBM;
        change_flrc_parameters( RF_FREQUENCY_923_4, EXPECTED_PWR_9_DBM, FLRC_BITRATE, FLRC_BANDWIDTH );
        break;
    case RF_CERT_STATE_FLRC_ARIB_923_4_POWER_9_DBM:
        /*Test sequence completed*/
        rf_certification.transmission_active    = false;
        rf_certification.rf_certification_state = RF_CERT_STATE_IDLE;
        break;

    default:
        rf_certification.rf_certification_state = RF_CERT_STATE_IDLE;
        rf_certification.transmission_active    = false;
        break;
    }
}

static void rf_certification_fcc_state_machine( void )
{
    switch( rf_certification.rf_certification_state )
    {
    case RF_CERT_STATE_IDLE:
        /*LoRa modulation - FCC region - DTS MODE*/
        rf_certification.rf_certification_state =
            RF_CERT_STATE_LORA_FCC_903_0_SF8_BW_500_POWER_22_DBM_DR4_PAYLOAD_LENGTH;
        change_lora_parameters_with_payload( RF_FREQUENCY_903_0, RAL_LORA_SF8, RAL_LORA_BW_500_KHZ, EXPECTED_PWR_22_DBM,
                                             LORAWAN_DR4_PAYLOAD_LENGTH );
        print_lora_configuration( );
        notification_wrap( rf_certification_tx );
        break;

    /**********************************************/
    /*                  DTS MODE                  */
    /**********************************************/
    case RF_CERT_STATE_LORA_FCC_903_0_SF8_BW_500_POWER_22_DBM_DR4_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state =
            RF_CERT_STATE_LORA_FCC_909_4_SF8_BW_500_POWER_22_DBM_DR4_PAYLOAD_LENGTH;
        change_lora_parameters_with_payload( RF_FREQUENCY_909_4, RAL_LORA_SF8, RAL_LORA_BW_500_KHZ, EXPECTED_PWR_22_DBM,
                                             LORAWAN_DR4_PAYLOAD_LENGTH );
        break;

    case RF_CERT_STATE_LORA_FCC_909_4_SF8_BW_500_POWER_22_DBM_DR4_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state =
            RF_CERT_STATE_LORA_FCC_914_2_SF8_BW_500_POWER_22_DBM_DR4_PAYLOAD_LENGTH;
        change_lora_parameters_with_payload( RF_FREQUENCY_914_2, RAL_LORA_SF8, RAL_LORA_BW_500_KHZ, EXPECTED_PWR_22_DBM,
                                             LORAWAN_DR4_PAYLOAD_LENGTH );
        break;

    /**********************************************/
    /*                 HYBRID MODE                */
    /**********************************************/
    case RF_CERT_STATE_LORA_FCC_914_2_SF8_BW_500_POWER_22_DBM_DR4_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state =
            RF_CERT_STATE_LORA_FCC_902_3_SF7_BW_125_POWER_22_DBM_DR3_PAYLOAD_LENGTH;
        change_lora_parameters_with_payload( RF_FREQUENCY_902_3, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_22_DBM,
                                             LORAWAN_DR3_PAYLOAD_LENGTH );
        break;

    case RF_CERT_STATE_LORA_FCC_902_3_SF7_BW_125_POWER_22_DBM_DR3_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state =
            RF_CERT_STATE_LORA_FCC_902_3_SF10_BW_125_POWER_22_DBM_DR0_PAYLOAD_LENGTH;
        change_lora_parameters_with_payload( RF_FREQUENCY_902_3, RAL_LORA_SF10, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR0_PAYLOAD_LENGTH );
        break;

    case RF_CERT_STATE_LORA_FCC_902_3_SF10_BW_125_POWER_22_DBM_DR0_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state =
            RF_CERT_STATE_LORA_FCC_908_7_SF7_BW_125_POWER_22_DBM_DR3_PAYLOAD_LENGTH;
        change_lora_parameters_with_payload( RF_FREQUENCY_908_7, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_22_DBM,
                                             LORAWAN_DR3_PAYLOAD_LENGTH );
        break;

    case RF_CERT_STATE_LORA_FCC_908_7_SF7_BW_125_POWER_22_DBM_DR3_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state =
            RF_CERT_STATE_LORA_FCC_908_7_SF10_BW_125_POWER_22_DBM_DR0_PAYLOAD_LENGTH;
        change_lora_parameters_with_payload( RF_FREQUENCY_908_7, RAL_LORA_SF10, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR0_PAYLOAD_LENGTH );
        break;

    case RF_CERT_STATE_LORA_FCC_908_7_SF10_BW_125_POWER_22_DBM_DR0_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state =
            RF_CERT_STATE_LORA_FCC_914_9_SF7_BW_125_POWER_22_DBM_DR3_PAYLOAD_LENGTH;
        change_lora_parameters_with_payload( RF_FREQUENCY_914_9, RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, EXPECTED_PWR_22_DBM,
                                             LORAWAN_DR3_PAYLOAD_LENGTH );
        break;

    case RF_CERT_STATE_LORA_FCC_914_9_SF7_BW_125_POWER_22_DBM_DR3_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state =
            RF_CERT_STATE_LORA_FCC_914_9_SF10_BW_125_POWER_22_DBM_DR0_PAYLOAD_LENGTH;
        change_lora_parameters_with_payload( RF_FREQUENCY_914_9, RAL_LORA_SF10, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR0_PAYLOAD_LENGTH );
        break;

    /**********************************************/
    /*                 FHSS SWEEP                 */
    /**********************************************/
    case RF_CERT_STATE_LORA_FCC_914_9_SF10_BW_125_POWER_22_DBM_DR0_PAYLOAD_LENGTH:
        rf_certification.fcc_sweep_started      = true;
        rf_certification.rf_certification_state = RF_CERT_STATE_FCC_SWEEP_FHSS_SF10_DR0_PAYLOAD_LENGTH;
        // FHSS sweep mode - SF10/DR0
        fcc_sweep_test_fhss( );
        change_lora_parameters_with_payload( RF_FREQUENCY_902_3, RAL_LORA_SF10, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR0_PAYLOAD_LENGTH );
        RF_CERT_LOG_CONFIG( "Starting FHSS SWEEP mode - SF10/DR0\n" );
        break;

    case RF_CERT_STATE_FCC_SWEEP_FHSS_SF10_DR0_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state = RF_CERT_STATE_FCC_SWEEP_FHSS_SF9_DR1_PAYLOAD_LENGTH;
        fcc_sweep_test_fhss( );
        change_lora_parameters_with_payload( rf_certification.fcc_sweep_frequency[0], RAL_LORA_SF9, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR1_PAYLOAD_LENGTH );
        RF_CERT_LOG_CONFIG( "FHSS SWEEP mode - SF9/DR1\n" );
        break;

    case RF_CERT_STATE_FCC_SWEEP_FHSS_SF9_DR1_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state = RF_CERT_STATE_FCC_SWEEP_FHSS_SF8_DR2_PAYLOAD_LENGTH;
        fcc_sweep_test_fhss( );
        change_lora_parameters_with_payload( rf_certification.fcc_sweep_frequency[0], RAL_LORA_SF8, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR2_PAYLOAD_LENGTH );
        RF_CERT_LOG_CONFIG( "FHSS SWEEP mode - SF8/DR2\n" );
        break;

    case RF_CERT_STATE_FCC_SWEEP_FHSS_SF8_DR2_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state = RF_CERT_STATE_FCC_SWEEP_FHSS_SF7_DR3_PAYLOAD_LENGTH;
        fcc_sweep_test_fhss( );
        change_lora_parameters_with_payload( rf_certification.fcc_sweep_frequency[0], RAL_LORA_SF7, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR3_PAYLOAD_LENGTH );
        RF_CERT_LOG_CONFIG( "FHSS SWEEP mode - SF7/DR3\n" );
        break;

    /**********************************************/
    /*                 HYBRID SWEEP               */
    /**********************************************/
    case RF_CERT_STATE_FCC_SWEEP_FHSS_SF7_DR3_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state   = RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF10_DR0_PAYLOAD_LENGTH;
        rf_certification.fcc_sweep_started        = false;
        rf_certification.fcc_sweep_hybrid_started = true;
        fcc_sweep_test_hybrid_fhss( );
        change_lora_parameters_with_payload( rf_certification.fcc_sweep_frequency[0], RAL_LORA_SF10,
                                             RAL_LORA_BW_125_KHZ, EXPECTED_PWR_22_DBM, LORAWAN_DR0_PAYLOAD_LENGTH );
        RF_CERT_LOG_CONFIG( "Starting HYBRID SWEEP mode - Subband 0 - SF10/DR0\n" );
        break;

    case RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF10_DR0_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state = RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF9_DR1_PAYLOAD_LENGTH;
        fcc_sweep_test_hybrid_fhss( );
        change_lora_parameters_with_payload( rf_certification.fcc_sweep_frequency[0], RAL_LORA_SF9, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR1_PAYLOAD_LENGTH );
        RF_CERT_LOG_CONFIG( "HYBRID SWEEP mode - Subband 0 - SF9/DR1\n" );
        break;

    case RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF9_DR1_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state = RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF8_DR2_PAYLOAD_LENGTH;
        fcc_sweep_test_hybrid_fhss( );
        change_lora_parameters_with_payload( rf_certification.fcc_sweep_frequency[0], RAL_LORA_SF8, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR2_PAYLOAD_LENGTH );
        RF_CERT_LOG_CONFIG( "HYBRID SWEEP mode - Subband 0 - SF8/DR2\n" );
        break;

    case RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF8_DR2_PAYLOAD_LENGTH:
        rf_certification.rf_certification_state = RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF7_DR3_PAYLOAD_LENGTH;
        fcc_sweep_test_hybrid_fhss( );
        change_lora_parameters_with_payload( rf_certification.fcc_sweep_frequency[0], RAL_LORA_SF7, RAL_LORA_BW_125_KHZ,
                                             EXPECTED_PWR_22_DBM, LORAWAN_DR3_PAYLOAD_LENGTH );
        RF_CERT_LOG_CONFIG( "HYBRID SWEEP mode - Subband 0 - SF7/DR3\n" );
        break;

    case RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF7_DR3_PAYLOAD_LENGTH:
        /*Test sequence completed*/
        rf_certification.transmission_active      = false;
        rf_certification.fcc_sweep_hybrid_started = false;
        rf_certification.rf_certification_state   = RF_CERT_STATE_IDLE;
        RF_CERT_LOG_CONFIG( "FCC certification test sequence completed\n" );
        break;

    default:
        rf_certification.rf_certification_state   = RF_CERT_STATE_IDLE;
        rf_certification.fcc_sweep_hybrid_started = false;
        rf_certification.fcc_sweep_started        = false;
        rf_certification.transmission_active      = false;
        break;
    }
}

static void rf_certification_tx( void )
{
    if( rf_certification.transmission_active )
    {
        // Update packet counter in payload
        rf_certification.payload[PAYLOAD_SIZE - 4] = ( rf_certification.packet_count >> 24 ) & 0xFF;
        rf_certification.payload[PAYLOAD_SIZE - 3] = ( rf_certification.packet_count >> 16 ) & 0xFF;
        rf_certification.payload[PAYLOAD_SIZE - 2] = ( rf_certification.packet_count >> 8 ) & 0xFF;
        rf_certification.payload[PAYLOAD_SIZE - 1] = rf_certification.packet_count & 0xFF;

        rf_certification.transaction->scheduler_config.start_time_ms =
            smtc_modem_hal_get_time_in_ms( ) + INTER_TX_DELAY;

        smtc_rac_return_code_t error_code;
        if( rf_certification.transaction->modulation_type == SMTC_RAC_MODULATION_FSK )
        {
            error_code = SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( rf_certification.radio_access_id ) );
            RF_CERT_LOG_TX( "Scheduling FSK transmission #%lu \n", rf_certification.packet_count + 1 );
        }
        else if( rf_certification.transaction->modulation_type == SMTC_RAC_MODULATION_LRFHSS )
        {
            error_code = SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( rf_certification.radio_access_id ) );
            RF_CERT_LOG_TX( "Scheduling LR-FHSS transmission #%lu \n", rf_certification.packet_count + 1 );
        }
        else if( rf_certification.transaction->modulation_type == SMTC_RAC_MODULATION_FLRC )
        {
            error_code = SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( rf_certification.radio_access_id ) );
            RF_CERT_LOG_TX( "Scheduling FLRC transmission #%lu \n", rf_certification.packet_count + 1 );
        }
        else
        {
            error_code = SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( rf_certification.radio_access_id ) );
            RF_CERT_LOG_TX( "Scheduling LoRa transmission #%lu \n", rf_certification.packet_count + 1 );
        }

        SMTC_MODEM_HAL_PANIC_ON_FAILURE( error_code == SMTC_RAC_SUCCESS );
    }
}

static void pre_rf_certification_callback( void )
{
    set_led( SMTC_PF_LED_TX, true );
}

static void post_rf_certification_callback( rp_status_t status )
{
    set_led( SMTC_PF_LED_TX, false );

    switch( status )
    {
    case RP_STATUS_TX_DONE:
        rf_certification.packet_count++;
        if( rf_certification.region == RF_CERT_REGION_FCC )
        {
            if( rf_certification.fcc_sweep_started )
            {
                if( rf_certification.fcc_sweep_fhss_counter > 0 )
                {
                    rf_certification.fcc_sweep_fhss_counter--;
                }
                if( rf_certification.fcc_sweep_fhss_counter == 0 )
                {
                    fcc_sweep_test_fhss( );
                }
                if( rf_certification.fcc_sweep_fhss_counter > 0 )
                {
                    rf_certification.transaction->radio_params.lora.frequency_in_hz =
                        rf_certification
                            .fcc_sweep_frequency[FCC_SWEEP_FHSS_CHANNELS_NUM - rf_certification.fcc_sweep_fhss_counter];
                    RF_CERT_LOG_INFO( "Frequency: %lu Hz\n",
                                      rf_certification.transaction->radio_params.lora.frequency_in_hz );
                }
            }
            if( rf_certification.fcc_sweep_hybrid_started )
            {
                rf_certification.fcc_sweep_fhss_counter--;

                if( rf_certification.fcc_sweep_fhss_counter == 0 )
                {
                    fcc_sweep_test_hybrid_fhss( );
                }
                rf_certification.transaction->radio_params.lora.frequency_in_hz =
                    rf_certification
                        .fcc_sweep_frequency[FCC_SWEEP_HYBRID_CHANNELS_NUM - rf_certification.fcc_sweep_fhss_counter];
                RF_CERT_LOG_INFO( "Frequency: %lu Hz\n",
                                  rf_certification.transaction->radio_params.lora.frequency_in_hz );
            }
        }

        break;

    default:
        RF_CERT_LOG_ERROR( "usp/rac: ERROR: unexpected transaction status: %d\n", ( int ) status );
        break;
    }

    // Continue transmission if still active
    if( rf_certification.transmission_active )
    {
        notification_wrap( rf_certification_tx );
    }
    else
    {
        RF_CERT_LOG_INFO( "===== Transmission stopped by user =====\n" );
        RF_CERT_LOG_INFO( "Total packets sent: %lu\n", rf_certification.packet_count );
        RF_CERT_LOG_INFO( "Transmission sequence completed\n" );
    }
}

static void change_lora_parameters( uint32_t freq, uint8_t sf, uint8_t bw, uint8_t power )
{
    rf_certification.transaction->radio_params.lora.frequency_in_hz = freq;
    rf_certification.transaction->radio_params.lora.sf              = sf;
    rf_certification.transaction->radio_params.lora.bw              = bw;
    rf_certification.transaction->radio_params.lora.tx_power_in_dbm = power;
    RF_CERT_LOG_CONFIG(
        "LoRa parameters changed: Frequency: %lu Hz, Spreading Factor: %s, Bandwidth: %s kHz, TX Power: %d dBm\n", freq,
        ral_lora_sf_to_str( sf ), ral_lora_bw_to_str( bw ), power );
}

static void change_lora_parameters_with_payload( uint32_t freq, uint8_t sf, uint8_t bw, uint8_t power,
                                                 uint16_t payload_size )
{
    rf_certification.transaction->radio_params.lora.frequency_in_hz = freq;
    rf_certification.transaction->radio_params.lora.sf              = sf;
    rf_certification.transaction->radio_params.lora.bw              = bw;
    rf_certification.transaction->radio_params.lora.tx_power_in_dbm = power;
    rf_certification.transaction->radio_params.lora.tx_size         = payload_size;
    RF_CERT_LOG_CONFIG(
        "LoRa parameters changed: Frequency: %lu Hz, Spreading Factor: %s, Bandwidth: %s kHz, TX Power: %d dBm, "
        "Payload: %d bytes\n",
        freq, ral_lora_sf_to_str( sf ), ral_lora_bw_to_str( bw ), power, payload_size );
}

static void change_fsk_parameters( uint32_t freq, uint8_t power )
{
    uint32_t freq_mhz_int = freq / 1000000;
    uint32_t freq_mhz_dec = ( freq % 1000000 ) / 100000;

    rf_certification.transaction->radio_params.fsk.frequency_in_hz = freq;

    rf_certification.transaction->radio_params.fsk.tx_power_in_dbm = power;
    RF_CERT_LOG_CONFIG( "FSK parameters changed: Frequency: %lu Hz (%lu.%lu MHz), TX Power: %d dBm\n", freq,
                        freq_mhz_int, freq_mhz_dec,

                        power );
}

static void print_rf_certification_banner( void )
{
    RF_CERT_LOG_CONFIG( "==============================\n" );
    RF_CERT_LOG_CONFIG( "   RF Certification Example   \n" );
    RF_CERT_LOG_CONFIG( "==============================\n" );
    RF_CERT_LOG_CONFIG( "Region: \n" );
    switch( rf_certification.region )
    {
    case RF_CERT_REGION_ETSI:
        RF_CERT_LOG_CONFIG( "ETSI (Europe)\n" );
        break;
    case RF_CERT_REGION_ARIB:
        RF_CERT_LOG_CONFIG( "ARIB (Japan)\n" );
        break;
    case RF_CERT_REGION_FCC:
        RF_CERT_LOG_CONFIG( "FCC (USA)\n" );
        break;
    default:
        RF_CERT_LOG_CONFIG( "Unknown\n" );
        break;
    }
    RF_CERT_LOG_CONFIG( "==============================\n" );
    RF_CERT_LOG_CONFIG( "\n" );
}

static void print_lora_configuration( void )
{
    RF_CERT_LOG_CONFIG( "===== LoRa Configuration =====\n" );
    RF_CERT_LOG_CONFIG( "Frequency:        %lu Hz (%lu.%lu MHz)\n",
                        rf_certification.transaction->radio_params.lora.frequency_in_hz,
                        rf_certification.transaction->radio_params.lora.frequency_in_hz / 1000000,
                        ( rf_certification.transaction->radio_params.lora.frequency_in_hz % 1000000 ) / 100000 );
    RF_CERT_LOG_CONFIG( "TX Power:         %d dBm\n", rf_certification.transaction->radio_params.lora.tx_power_in_dbm );
    RF_CERT_LOG_CONFIG( "Spreading Factor: SF%d\n", rf_certification.transaction->radio_params.lora.sf );
    RF_CERT_LOG_CONFIG( "Bandwidth:        %s kHz\n",
                        ral_lora_bw_to_str( rf_certification.transaction->radio_params.lora.bw ) );
    RF_CERT_LOG_CONFIG( "Coding Rate:      4/%d\n", rf_certification.transaction->radio_params.lora.cr + 4 );
    RF_CERT_LOG_CONFIG( "Payload Length:   %d bytes\n", rf_certification.transaction->radio_params.lora.tx_size );
    RF_CERT_LOG_CONFIG( "Inter-TX Delay:   %lu ms\n", INTER_TX_DELAY );
    RF_CERT_LOG_CONFIG( "\n" );
}
static void print_fsk_configuration( void )
{
    uint32_t freq_hz      = rf_certification.transaction->radio_params.fsk.frequency_in_hz;
    uint32_t freq_mhz_int = freq_hz / 1000000;
    uint32_t freq_mhz_dec = ( freq_hz % 1000000 ) / 100000;

    uint32_t bitrate          = rf_certification.transaction->radio_params.fsk.br_in_bps;
    uint32_t bitrate_kbps_int = bitrate / 1000;
    uint32_t bitrate_kbps_dec = ( bitrate % 1000 ) / 100;

    uint32_t fdev         = rf_certification.transaction->radio_params.fsk.fdev_in_hz;
    uint32_t fdev_khz_int = fdev / 1000;
    uint32_t fdev_khz_dec = ( fdev % 1000 ) / 100;

    uint32_t bw         = rf_certification.transaction->radio_params.fsk.bw_dsb_in_hz;
    uint32_t bw_khz_int = bw / 1000;
    uint32_t bw_khz_dec = ( bw % 1000 ) / 100;

    uint8_t preamble_length  = rf_certification.transaction->radio_params.fsk.preamble_len_in_bits;
    uint8_t sync_word_length = rf_certification.transaction->radio_params.fsk.sync_word_len_in_bits;
    uint8_t crc_type         = rf_certification.transaction->radio_params.fsk.crc_type;

    uint8_t  tx_power       = rf_certification.transaction->radio_params.fsk.tx_power_in_dbm;
    uint16_t payload_length = rf_certification.transaction->radio_params.fsk.tx_size;

    RF_CERT_LOG_CONFIG( "===== FSK Configuration =====\n" );
    RF_CERT_LOG_CONFIG( "Modulation:       FSK\n" );
    RF_CERT_LOG_CONFIG( "Frequency:        %lu Hz (%lu.%lu MHz)\n", freq_hz, freq_mhz_int, freq_mhz_dec );
    RF_CERT_LOG_CONFIG( "TX Power:         %d dBm\n", tx_power );
    RF_CERT_LOG_CONFIG( "Bitrate:          %lu bps (%lu.%lu kbps)\n", bitrate, bitrate_kbps_int, bitrate_kbps_dec );
    RF_CERT_LOG_CONFIG( "Freq Deviation:   %lu Hz (%lu.%lu kHz)\n", fdev, fdev_khz_int, fdev_khz_dec );
    RF_CERT_LOG_CONFIG( "Bandwidth:        %lu.%lu kHz\n", bw_khz_int, bw_khz_dec );
    RF_CERT_LOG_CONFIG( "Preamble Length:  %d bytes (%d bits)\n", preamble_length, preamble_length * 8 );
    RF_CERT_LOG_CONFIG( "Sync Word:        Default (%d bytes)\n", sync_word_length );
    RF_CERT_LOG_CONFIG( "CRC:              %s\n",
                        ( crc_type == RAL_GFSK_CRC_2_BYTES_INV ) ? "2-byte CRC" : "Other CRC" );

    RF_CERT_LOG_CONFIG( "Payload Length:   %d bytes\n", payload_length );
    RF_CERT_LOG_CONFIG( "Inter-TX Delay:   %lu ms\n", INTER_TX_DELAY );
    RF_CERT_LOG_CONFIG( "==============================\n" );
    RF_CERT_LOG_CONFIG( "\n" );
}

static void print_lrfhss_configuration( void )
{
    uint32_t freq_hz      = rf_certification.transaction->radio_params.lrfhss.frequency_in_hz;
    uint32_t freq_mhz_int = freq_hz / 1000000;
    uint32_t freq_mhz_dec = ( freq_hz % 1000000 ) / 100000;

    uint8_t  tx_power        = rf_certification.transaction->radio_params.lrfhss.tx_power_in_dbm;
    uint8_t  coding_rate     = rf_certification.transaction->radio_params.lrfhss.coding_rate;
    uint32_t bandwidth       = rf_certification.transaction->radio_params.lrfhss.bandwidth;
    uint32_t grid            = rf_certification.transaction->radio_params.lrfhss.grid;
    bool     enable_hopping  = rf_certification.transaction->radio_params.lrfhss.enable_hopping;
    int8_t   device_offset   = rf_certification.transaction->radio_params.lrfhss.device_offset;
    uint8_t  hop_sequence_id = rf_certification.transaction->radio_params.lrfhss.hop_sequence_id;
    uint16_t payload_length  = rf_certification.transaction->radio_params.lrfhss.tx_size;

    RF_CERT_LOG_CONFIG( "===== LR-FHSS Configuration =====\n" );
    RF_CERT_LOG_CONFIG( "Modulation:       LR-FHSS\n" );
    RF_CERT_LOG_CONFIG( "Frequency:        %lu Hz (%lu.%lu MHz)\n", freq_hz, freq_mhz_int, freq_mhz_dec );
    RF_CERT_LOG_CONFIG( "TX Power:         %d dBm\n", tx_power );
    RF_CERT_LOG_CONFIG( "Coding Rate:      1/%d\n", coding_rate );
    RF_CERT_LOG_CONFIG( "Bandwidth:        %lu.%03lu kHz\n", bandwidth / 1000, ( bandwidth % 1000 ) );
    RF_CERT_LOG_CONFIG( "Grid:             %lu.%03lu kHz\n", grid / 1000, ( grid % 1000 ) );
    RF_CERT_LOG_CONFIG( "Hopping:          %s\n", enable_hopping ? "Enabled" : "Disabled" );
    RF_CERT_LOG_CONFIG( "Device Offset:    %d\n", device_offset );
    RF_CERT_LOG_CONFIG( "Hop Sequence ID:  %d\n", hop_sequence_id );
    RF_CERT_LOG_CONFIG( "Payload Length:   %d bytes\n", payload_length );
    RF_CERT_LOG_CONFIG( "Inter-TX Delay:   %lu ms\n", INTER_TX_DELAY );
    RF_CERT_LOG_CONFIG( "==================================\n" );
    RF_CERT_LOG_CONFIG( "\n" );
}

static void print_flrc_configuration( void )
{
    uint32_t freq_hz      = rf_certification.transaction->radio_params.flrc.frequency_in_hz;
    uint32_t freq_mhz_int = freq_hz / 1000000;
    uint32_t freq_mhz_dec = ( freq_hz % 1000000 ) / 100000;

    uint32_t bitrate          = rf_certification.transaction->radio_params.flrc.br_in_bps;
    uint32_t bitrate_mbps_int = bitrate / 1000000;
    uint32_t bitrate_mbps_dec = ( bitrate % 1000000 ) / 100000;

    uint32_t bw         = rf_certification.transaction->radio_params.flrc.bw_dsb_in_hz;
    uint32_t bw_mhz_int = bw / 1000000;
    uint32_t bw_mhz_dec = ( bw % 1000000 ) / 100000;

    uint8_t  tx_power        = rf_certification.transaction->radio_params.flrc.tx_power_in_dbm;
    uint8_t  coding_rate     = rf_certification.transaction->radio_params.flrc.cr;
    uint8_t  pulse_shape     = rf_certification.transaction->radio_params.flrc.pulse_shape;
    uint16_t preamble_length = rf_certification.transaction->radio_params.flrc.preamble_len_in_bits;
    uint8_t  sync_word_len   = rf_certification.transaction->radio_params.flrc.sync_word_len;
    uint8_t  crc_type        = rf_certification.transaction->radio_params.flrc.crc_type;
    uint16_t payload_length  = rf_certification.transaction->radio_params.flrc.tx_size;

    RF_CERT_LOG_CONFIG( "===== FLRC Configuration =====\n" );
    RF_CERT_LOG_CONFIG( "Modulation:       FLRC\n" );
    RF_CERT_LOG_CONFIG( "Frequency:        %lu Hz (%lu.%lu MHz)\n", freq_hz, freq_mhz_int, freq_mhz_dec );
    RF_CERT_LOG_CONFIG( "TX Power:         %d dBm\n", tx_power );
    RF_CERT_LOG_CONFIG( "Bitrate:          %lu bps (%lu.%lu Mbps)\n", bitrate, bitrate_mbps_int, bitrate_mbps_dec );
    RF_CERT_LOG_CONFIG( "Bandwidth:        %lu.%lu MHz\n", bw_mhz_int, bw_mhz_dec );
    RF_CERT_LOG_CONFIG( "Coding Rate:      %s\n", ral_flrc_cr_to_str( coding_rate ) );
    RF_CERT_LOG_CONFIG( "Pulse Shape:      %s\n", ral_flrc_pulse_shape_to_str( pulse_shape ) );
    RF_CERT_LOG_CONFIG( "Preamble Length:  %d bits\n", preamble_length );
    RF_CERT_LOG_CONFIG( "Sync Word Length: %s\n", ral_flrc_sync_word_len_to_str( sync_word_len ) );
    RF_CERT_LOG_CONFIG( "CRC Type:         %s\n", ral_flrc_crc_type_to_str( crc_type ) );
    RF_CERT_LOG_CONFIG( "Payload Length:   %d bytes\n", payload_length );
    RF_CERT_LOG_CONFIG( "Inter-TX Delay:   %lu ms\n", INTER_TX_DELAY );
    RF_CERT_LOG_CONFIG( "===============================\n" );
    RF_CERT_LOG_CONFIG( "\n" );
}

static void change_lrfhss_parameters( uint32_t freq, uint8_t power )
{
    uint32_t freq_mhz_int = freq / 1000000;
    uint32_t freq_mhz_dec = ( freq % 1000000 ) / 100000;

    rf_certification.transaction->radio_params.lrfhss.frequency_in_hz = freq;
    rf_certification.transaction->radio_params.lrfhss.tx_power_in_dbm = power;
    RF_CERT_LOG_CONFIG( "LR-FHSS parameters changed: Frequency: %lu Hz (%lu.%lu MHz), TX Power: %d dBm\n", freq,
                        freq_mhz_int, freq_mhz_dec, power );
}
static void change_flrc_parameters( uint32_t freq, uint8_t power, uint32_t bitrate, uint32_t bw_dsb_in_hz )
{
    uint32_t freq_mhz_int = freq / 1000000;
    uint32_t freq_mhz_dec = ( freq % 1000000 ) / 100000;

    rf_certification.transaction->radio_params.flrc.frequency_in_hz = freq;
    rf_certification.transaction->radio_params.flrc.tx_power_in_dbm = power;

    rf_certification.transaction->radio_params.flrc.br_in_bps    = bitrate;
    rf_certification.transaction->radio_params.flrc.bw_dsb_in_hz = bw_dsb_in_hz;

    RF_CERT_LOG_CONFIG( "FLRC parameters changed: Frequency: %lu Hz (%lu.%lu MHz), TX Power: %d dBm\n", freq,
                        freq_mhz_int, freq_mhz_dec, power );
}
static void fcc_sweep_test_fhss( void )
{
    uint8_t i, j, temp = 0;

    uint8_t channel_index[FCC_SWEEP_FHSS_CHANNELS_NUM] = { 0 };

    // generate pseudo-random channel map for subband
    for( i = 0; i < FCC_SWEEP_FHSS_CHANNELS_NUM; i++ )
        channel_index[i] = i;

    for( i = ( FCC_SWEEP_FHSS_CHANNELS_NUM - 1 ); i > 0; i-- )
    {
        j                = smtc_modem_hal_get_random_nb_in_range( 0, i );
        temp             = channel_index[i];
        channel_index[i] = channel_index[j];
        channel_index[j] = temp;
    }
    for( i = 0; i < FCC_SWEEP_FHSS_CHANNELS_NUM; i++ )
    {
        rf_certification.fcc_sweep_frequency[i] =
            RF_FREQUENCY_902_3 + ( FCC_SWEEP_CHANNEL_WIDTH_HZ * channel_index[i] );
    }

    rf_certification.fcc_sweep_fhss_counter = FCC_SWEEP_FHSS_CHANNELS_NUM;
}

static void fcc_sweep_test_hybrid_fhss( void )
{
    uint8_t i, j, temp = 0;

    uint8_t channel_index[FCC_SWEEP_HYBRID_CHANNELS_NUM] = { 0 };

    // generate pseudo-random channel map for subband
    for( i = 0; i < FCC_SWEEP_HYBRID_CHANNELS_NUM; i++ )
        channel_index[i] = i;

    for( i = ( FCC_SWEEP_HYBRID_CHANNELS_NUM - 1 ); i > 0; i-- )
    {
        j                = smtc_modem_hal_get_random_nb_in_range( 0, i );
        temp             = channel_index[i];
        channel_index[i] = channel_index[j];
        channel_index[j] = temp;
    }
    for( i = 0; i < FCC_SWEEP_HYBRID_CHANNELS_NUM; i++ )
    {
        rf_certification.fcc_sweep_frequency[i] =
            RF_FREQUENCY_902_3 + ( FCC_SWEEP_CHANNEL_WIDTH_HZ * channel_index[i] );
    }

    rf_certification.fcc_sweep_fhss_counter = FCC_SWEEP_HYBRID_CHANNELS_NUM;
}

static void print_notification( char* notification )
{
    RF_CERT_LOG_INFO( "%s\n", notification );
}
/* --- EOF ------------------------------------------------------------------ */
