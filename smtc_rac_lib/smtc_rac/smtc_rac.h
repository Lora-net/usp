/*!
 * \file      smtc_rac.h
 *
 * \brief     SMTC Radio Abstraction Component (RAC) header file
 *
 * \copyright Semtech Corporation 2024. All rights reserved.
 *
 * \license   Revised BSD License, see section \ref LICENSE.
 *
 * \date      2024
 */

#ifndef SMTC_RAC_H
#define SMTC_RAC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "radio_planner.h"
#include "radio_planner_hook_id_defs.h"
#include "ral.h"
#include "ral_defs.h"
#include "ralf.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define RAC_INVALID_RADIO_ID 0xFF
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
/*!
 * \enum smtc_rac_return_code_t
 * \brief Return codes for RAC API functions.
 */
typedef enum smtc_rac_return_code_e
{
    SMTC_RAC_SUCCESS = 0,       /*!< Operation successful */
    SMTC_RAC_ERROR,             /*!< Generic error */
    SMTC_RAC_BUSY,              /*!< Radio or resource busy */
    SMTC_RAC_TIMEOUT,           /*!< Operation timed out */
    SMTC_RAC_INVALID_PARAMETER, /*!< Invalid parameter passed */
    SMTC_RAC_NOT_SUPPORTED,     /*!< Operation not supported */
    SMTC_RAC_NOT_INITIALIZED,   /*!< RAC not initialized */
    SMTC_RAC_NOT_IMPLEMENTED,   /*!< Function not implemented */
} smtc_rac_return_code_t;
/*!
 * \enum smtc_rac_priority_t
 * \brief Priority levels for radio access requests.
 *        Higher priority tasks can preempt lower priority ones.
 */
typedef enum smtc_rac_priority_e
{
    RAC_VERY_HIGH_PRIORITY = RP_HOOK_RAC_VERY_HIGH_PRIORITY, /*!< Highest priority */
    RAC_HIGH_PRIORITY      = RP_HOOK_RAC_HIGH_PRIORITY,
    RAC_MEDIUM_PRIORITY    = RP_HOOK_RAC_MEDIUM_PRIORITY,
    RAC_LOW_PRIORITY       = RP_HOOK_RAC_LOW_PRIORITY,
    RAC_VERY_LOW_PRIORITY  = RP_HOOK_RAC_VERY_LOW_PRIORITY /*!< Lowest priority */
} smtc_rac_priority_t;

/*!
 * \enum smtc_rac_scheduling_t
 * \brief Scheduling modes for the request
 */
typedef enum smtc_rac_scheduling_e
{
    SMTC_RAC_SCHEDULED_TRANSACTION, /*!< Transaction starts at scheduled time if
                                         possible, abort if not */
    SMTC_RAC_ASAP_TRANSACTION,      /*!< Transaction starts at scheduled time if
                                         possible, ASAP after it if not */
} smtc_rac_scheduling_t;
/*!
 * \enum smtc_rac_modulation_type_t
 * \brief Modulation types supported by RAC
 */
typedef enum smtc_rac_modulation_type_e
{
    SMTC_RAC_MODULATION_LORA   = 0, /*!< LoRa modulation */
    SMTC_RAC_MODULATION_FSK    = 1, /*!< FSK/GFSK modulation */
    SMTC_RAC_MODULATION_LRFHSS = 2, /*!< LR-FHSS modulation (transmission only) */
    SMTC_RAC_MODULATION_FLRC   = 3, /*!< FLRC modulation */
} smtc_rac_modulation_type_t;

/*!
 * \enum smtc_rac_lora_syncword_t
 * \brief Synchronization word for LoRa communication
 */
typedef enum smtc_rac_lora_syncword_e
{
    LORA_PRIVATE_NETWORK_SYNCWORD = 0x12, /*!< Synchronization word used for private network */
    LORA_PUBLIC_NETWORK_SYNCWORD  = 0x34, /*!< Synchronization word used for public network */
} smtc_rac_lora_syncword_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES - BASIC CONTEXT STRUCTURES --------------------------------
 */

/*!
 * \struct smtc_rac_rttof_params_t
 * \brief RTToF (Round Trip Time of Flight) parameters structure
 */
typedef struct smtc_rac_rttof_params_s
{
    uint32_t      request_address;        /*!< Ranging request address (used for RTToF ranging). */
    uint32_t      delay_indicator;        /*!< Delay indicator for ranging timing. */
    uint32_t      response_symbols_count; /*!< Number of symbols in the ranging response. */
    ral_lora_bw_t bw_ranging;             /*!< Bandwidth used for ranging. */
} smtc_rac_rttof_params_t;

/*!
 * \brief LBT context structure for RAC
 */
typedef struct
{
    bool     lbt_enabled;        /*!< Flag to indicate if LBT is enabled */
    uint32_t listen_duration_ms; /*!< Duration of listening period */
    int16_t  threshold_dbm;      /*!< RSSI threshold for busy/free detection */
    uint32_t bandwidth_hz;       /*!< Bandwidth for RSSI measurement */

    // Results
    int16_t rssi_inst_dbm; /*!< Last instantaneous RSSI */
    bool    channel_busy;  /*!< True if channel is busy */
} smtc_rac_lbt_context_t;

/*!
 * \struct smtc_rac_cw_context_t
 * \brief Structure holding all CW parameters for CW operations.
 *
 * This structure is used to configure the radio for CW transmission
 */
typedef struct smtc_rac_cw_context_s
{
    bool cw_enabled;        /*!< Flag to indicate if CW is enabled */
    bool infinite_preamble; /*!< Flag to indicate if infinite preamble is enabled */
} smtc_rac_cw_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES - LORA RADIO PARAMETERS -----------------------------------
 */

/*!
 * \struct smtc_rac_radio_lora_params_t
 * \brief Structure holding all radio parameters for LoRa and ranging operations.
 *
 * This structure is used to configure the radio for transmission or reception,
 * including frequency, power, modulation, and ranging-specific parameters.
 */
typedef struct smtc_rac_radio_lora_params_s
{
    // common lora parameters
    bool is_tx;                                    /*!< Flag to indicate if the operation is a transmission (true) or
                                                      reception (false). */
    bool is_ranging_exchange;                      /*!< Flag to indicate if the operation is a ranging
                                                      exchange (true) or standard LoRa (false). */
    smtc_rac_rttof_params_t  rttof;                /*!< RTToF parameters for ranging operations. */
    uint32_t                 frequency_in_hz;      /*!< Frequency for transmission/reception in Hertz. */
    ral_lora_sf_t            sf;                   /*!< Spreading factor for LoRa modulation. */
    ral_lora_bw_t            bw;                   /*!< Bandwidth for LoRa modulation. */
    ral_lora_cr_t            cr;                   /*!< Coding rate for LoRa modulation. */
    uint16_t                 preamble_len_in_symb; /*!< Length of the preamble in symbols. */
    ral_lora_pkt_len_modes_t header_type;          /*!< Packet header type (explicit or implicit). */
    uint8_t                  invert_iq_is_on;      /*!< Flag to enable (1) or disable (0) IQ inversion. */
    uint8_t                  crc_is_on;            /*!< Flag to enable (1) or disable (0) CRC. */
    smtc_rac_lora_syncword_t sync_word;            /*!< Synchronization word for LoRa communication. */

    // lora specific tx parameters
    uint8_t  tx_power_in_dbm; /*!< Transmission power in dBm. */
    uint16_t tx_size;         /*!< Size of the tx payload data in bytes. */
    // lora specific rx parameters
    uint32_t rx_timeout_ms;   /*!< Timeout for receiving data in milliseconds. */
    uint8_t  symb_nb_timeout; /*!< Rx only parameters: Number of symbols to wait for the preamble. */
    uint16_t max_rx_size;     /*!< Maximum size of the payload data in bytes received by the radio. when the received
    payload is larger than the max_rx_size, the radio will discard the remaining payload. */
} smtc_rac_radio_lora_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES - FSK RADIO PARAMETERS ------------------------------------
 */

/*!
 * \struct smtc_rac_radio_fsk_params_t
 * \brief Structure holding all radio parameters for FSK operations.
 *
 * This structure is used to configure the radio for FSK transmission or
 * reception, including frequency, power, modulation parameters, and packet
 * configuration.
 */
typedef struct smtc_rac_radio_fsk_params_s
{
    bool is_tx;               /*!< Flag to indicate if the operation is a transmission (true) or
                                 reception (false). */
    uint16_t tx_size;         /*!< Size of the tx payload data in bytes. */
    uint16_t max_rx_size;     /*!< Maximum size of the payload data in bytes received by the radio. when the received
                                 payload is larger than the max_rx_size, the radio will discard the remaining payload. */
    uint32_t frequency_in_hz; /*!< Frequency for transmission/reception in Hertz. */
    int8_t   tx_power_in_dbm; /*!< Transmission power in dBm. */

    // Modulation parameters
    uint32_t               br_in_bps;    /*!< Bit rate in bits per second. */
    uint32_t               fdev_in_hz;   /*!< Frequency deviation in Hz. */
    uint32_t               bw_dsb_in_hz; /*!< Bandwidth (double-sided) in Hz. */
    ral_gfsk_pulse_shape_t pulse_shape;  /*!< Pulse shaping filter. */

    // Packet parameters
    ral_gfsk_pkt_len_modes_t     header_type;           /*!< Packet header type (fixed or variable length). */
    uint16_t                     preamble_len_in_bits;  /*!< Length of the preamble in bits. */
    ral_gfsk_preamble_detector_t preamble_detector;     /*!< Preamble detector configuration. */
    uint8_t                      sync_word_len_in_bits; /*!< Synchronization word length in bits. */
    ral_gfsk_crc_type_t          crc_type;              /*!< CRC type configuration. */
    ral_gfsk_dc_free_t           dc_free;               /*!< DC-free encoding type. */

    // Advanced parameters
    const uint8_t* sync_word;      /*!< Pointer to synchronization word array. */
    uint16_t       whitening_seed; /*!< Whitening seed value. */
    uint16_t       crc_seed;       /*!< CRC seed value. */
    uint16_t       crc_polynomial; /*!< CRC polynomial value. */

    uint32_t rx_timeout_ms; /*!< Timeout for receiving data in milliseconds. */
} smtc_rac_radio_fsk_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES - FLRC RADIO PARAMETERS -----------------------------------
 */

typedef struct smtc_rac_radio_flrc_params_s
{
    bool is_tx;               /*!< Flag to indicate if the operation is a transmission (true) or
                                 reception (false). */
    uint16_t tx_size;         /*!< Size of the tx payload data in bytes. */
    uint16_t max_rx_size;     /*!< Maximum size of the payload data in bytes received by the radio. when the received
                                 payload is larger than the max_rx_size, the radio will discard the remaining payload. */
    uint32_t frequency_in_hz; /*!< Frequency for transmission/reception in Hertz. */
    int8_t   tx_power_in_dbm; /*!< Transmission power in dBm. */

    // Modulation parameters
    uint32_t               br_in_bps;    /*!< Bit rate in bits per second. */
    uint32_t               bw_dsb_in_hz; /*!< Bandwidth (double-sided) in Hz. */
    ral_flrc_cr_t          cr;           /*!< Coding rate. */
    ral_flrc_pulse_shape_t pulse_shape;  /*!< Pulse shaping filter. */

    // Packet parameters
    uint16_t                      preamble_len_in_bits; /*!< Length of the preamble in bits. */
    ral_flrc_sync_word_len_t      sync_word_len;        /*!< Sync word length configuration. */
    ral_flrc_tx_syncword_t        tx_syncword;          /*!< TX sync word index to use. */
    ral_flrc_rx_match_sync_word_t match_sync_word;      /*!< RX sync word match configuration. */
    bool                          pld_is_fix;           /*!< Fixed (true) or variable (false) length. */
    ral_flrc_crc_type_t           crc_type;             /*!< CRC type configuration. */

    // Advanced parameters
    const uint8_t* sync_word; /*!< Pointer to synchronization word array (expected
                                 4 bytes). */
    uint32_t crc_seed;        /*!< CRC seed value. */
    uint32_t crc_polynomial;  /*!< CRC polynomial value. */

    uint32_t rx_timeout_ms; /*!< Timeout for receiving data in milliseconds. */
} smtc_rac_radio_flrc_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES - LR-FHSS RADIO PARAMETERS --------------------------------
 */

typedef struct smtc_rac_radio_lrfhss_params_s
{
    bool is_tx;      /*!< Flag to indicate if the operation is a transmission (should
                        always be true for LR-FHSS). */
    uint8_t tx_size; /*!< Size of the tx payload data in bytes. */

    uint32_t frequency_in_hz; /*!< Center frequency for transmission in Hertz. */
    int8_t   tx_power_in_dbm; /*!< Transmission power in dBm. */

    // LR-FHSS specific parameters
    lr_fhss_v1_cr_t   coding_rate;    /*!< Coding rate for LR-FHSS modulation. */
    lr_fhss_v1_bw_t   bandwidth;      /*!< Bandwidth for LR-FHSS modulation. */
    lr_fhss_v1_grid_t grid;           /*!< Grid step for frequency hopping. */
    bool              enable_hopping; /*!< Enable frequency hopping. */

    // Advanced parameters
    const uint8_t* sync_word;     /*!< Pointer to synchronization word array (4 bytes). */
    uint32_t       device_offset; /*!< Device offset for frequency calculation. */

    // Internal parameters (auto-generated)
    uint8_t  header_count;    /*!< Header count (auto-calculated from coding rate). */
    uint32_t hop_sequence_id; /*!< Hop sequence ID (auto-generated). */
} smtc_rac_radio_lrfhss_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES - CAD PARAMETERS ------------------------------------------
 */

/*!
 * \struct smtc_rac_cad_radio_params_s
 * \brief Structure holding parameters for Channel Activity Detection (CAD).
 *
 * This structure is used to configure the radio for CAD operations, including
 * the number of symbols, detection thresholds, exit modes, and timeout.
 */
typedef struct smtc_rac_cad_radio_params_s
{
    bool                      cad_enabled; /*!< Flag to indicate if CAD is enabled */
    ral_lora_cad_symbs_t      cad_symb_nb;
    ral_lora_cad_exit_modes_t cad_exit_mode;
    uint32_t                  cad_timeout_in_ms;
    ral_lora_sf_t             sf;  //!< LoRa Spreading Factor
    ral_lora_bw_t             bw;  //!< LoRa Bandwidth
    uint32_t                  rf_freq_in_hz;
    bool                      invert_iq_is_on;  //!< LoRa IQ polarity setup
} smtc_rac_cad_radio_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES - DATA AND SCHEDULING STRUCTURES -------------------------
 */

/*!
 * \struct smtc_rac_data_t
 * \brief Structure holding payload and result data for a radio transaction.
 *
 * This structure is used to pass payloads, receive results, and access event
 * data such as RSSI, SNR, and timestamps.
 */
typedef struct smtc_rac_data_buffer_setup_s
{
    uint8_t* tx_payload_buffer;         /*!< Pointer to the payload data buffer. */
    uint16_t size_of_tx_payload_buffer; /*!< Size of the payload data user buffer in bytes. */
    uint8_t* rx_payload_buffer;         /*!< Pointer to the payload data buffer. */
    uint16_t size_of_rx_payload_buffer; /*!< Size of the payload data user buffer in bytes. */

} smtc_rac_data_buffer_setup_t;

typedef struct smtc_rac_data_result_s
{
    uint16_t rx_size;     /*!< Size of the rx payload data in bytes received by the radio */
    int16_t  rssi_result; /*!< Received Signal Strength Indicator (RSSI) value. */
    int8_t
        snr_result; /*!< Signal-to-Noise Ratio (SNR) value. WARNING : LoRa only. Not Applicable for other modulations */
    uint32_t            radio_end_timestamp_ms;   /*!< timestamp of the radio event in milliseconds. */
    uint32_t            radio_start_timestamp_ms; /*!< timestamp of the radio start event in milliseconds. */
    rp_ranging_result_t ranging_result;           /*!< Ranging result structure (for RTToF). */
} smtc_rac_data_result_t;

typedef struct smtc_rac_scheduler_config_s
{
    uint32_t start_time_ms; /*!< Scheduled start time in milliseconds (absolute time). For ASAP, 0 is not recommended;
                               set smtc_modem_hal_get_time_in_ms( ) if you want NOW, or smtc_modem_hal_get_time_in_ms( )
                               + the delay you want */
    uint32_t              duration_time_ms; /*!< Duration time in milliseconds (absolute time). */
    smtc_rac_scheduling_t scheduling;       /*!< Scheduling mode for the request. */
    void ( *callback_pre_radio_transaction )(
        void ); /*!< Callback called just before the radio transaction (can be NULL). */
    void ( *callback_post_radio_transaction )(
        rp_status_t status ); /*!< Callback called just after the radio transaction (must not be NULL). */
} smtc_rac_scheduler_config_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */
/*!
 * \brief Enqueue a LoRa or ranging task for execution by the radio planner.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_lora( uint8_t radio_access_id );

/*!
 * \brief Enqueue a FSK task for execution by the radio planner.
 *
 * This function configures and schedules a FSK radio task using the context
 * previously configured via smtc_rac_get_context(). The FSK parameters should
 * be configured in the context's radio_params.fsk field.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_fsk( uint8_t radio_access_id );

/*!
 * \brief Enqueue a LR-FHSS transmission task for execution by the radio
 * planner.
 *
 * This function configures and schedules a LR-FHSS transmission task using the
 * context previously configured via smtc_rac_get_context(). The LR-FHSS
 * parameters should be configured in the context's radio_params.lrfhss field.
 * Note: LR-FHSS is transmission-only. Reception is not supported.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_lrfhss( uint8_t radio_access_id );

/*!
 * \brief Enqueue a FLRC task for execution by the radio planner.
 *
 * This function configures and schedules a FLRC radio task using the context
 * previously configured via smtc_rac_get_context(). The FLRC parameters
 * should be configured in the context's radio_params.flrc field.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_flrc( uint8_t radio_access_id );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_RAC_H

/*
 * -----------------------------------------------------------------------------
 * --- EOF ---------------------------------------------------------------------
 */
