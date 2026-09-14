/**
 * @file      smtc_flrp_api.h
 *
 * @brief     SMTC FLRC Protocol (FLRP) API
 *
 * This header contains the prototypes for the SMTC FLRC Protocol (FLRP) API.
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

#ifndef SMTC_FLRP_API_H
#define SMTC_FLRP_API_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_rac_api.h"
#include "../flrp_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief SMTC FLRP Return code
 * @remark Command output values must not be read if the returned code differs from SMTC_FLRP_RC_OK
 */
typedef enum smtc_flrp_return_code_e
{
    SMTC_FLRP_RC_OK = 0x00,
    SMTC_FLRP_RC_NOT_INIT,
    SMTC_FLRP_RC_INVALID_PARAMS,
    SMTC_FLRP_RC_UNSUPPORTED_FEATURE,
    SMTC_FLRP_RC_BUSY,
    SMTC_FLRP_RC_ERROR,
} smtc_flrp_return_code_t;

/**
 * @brief SMTC FLRC communication mode
 */
typedef enum smtc_flrp_communication_mode_e
{
    SMTC_FLRP_BIDIRECTIONAL = 0x00, /*!< Standard protocol with ACK exchange. */
    SMTC_FLRP_BIDIRECTIONAL_STREAM, /*!< Standard protocol without burst ACK. */
    SMTC_FLRP_COM_ONE_WAY,          /*!< No WOR ACK response expected. */
} smtc_flrp_communication_mode_t;

/** @brief Bits for smtc_flrp_burst_rx_stats_t.exchange_phase_success_mask (OR together). */
typedef enum smtc_flrp_exchange_phase_success_bit_e
{
    SMTC_FLRP_EXCHANGE_WOR_TX_SUCCESS            = ( 1 << 0 ), /*!< Initiator: WOR TX ok. Slave: WOR beacon ok. */
    SMTC_FLRP_EXCHANGE_WOR_ACK_SUCCESS           = ( 1 << 1 ), /*!< WOR ACK leg ok. */
    SMTC_FLRP_EXCHANGE_ADAPTIVE_LINK_ACTIVE      = ( 1 << 2 ), /*!< Adaptive link (FLRC_REQ/ACK) is active. */
    SMTC_FLRP_EXCHANGE_ADAPTIVE_FLRC_REQ_SUCCESS = ( 1 << 3 ), /*!< FLRC_REQ adaptation round completed. */
    SMTC_FLRP_EXCHANGE_ADAPTIVE_FLRC_ACK_SUCCESS = ( 1 << 4 ), /*!< FLRC_ACK adaptation round completed. */
    SMTC_FLRP_EXCHANGE_LAST_BURST_SUCCESS        = ( 1 << 5 ), /*!< Final data burst phase ok. */
    SMTC_FLRP_EXCHANGE_LAST_BURST_ACK_SUCCESS    = ( 1 << 6 ), /*!< Final burst ACK ok. */
} smtc_flrp_exchange_phase_success_bit_t;

typedef uint32_t bit_mask_t;

/**
 * @brief SMTC FLRP burst rx statistics
 */
typedef struct smtc_flrp_burst_rx_stats_s
{
    uint32_t   nb_packets_received_ok;      /*!< Number of packets of the bursts received and decoded successfully. */
    uint32_t   nb_packets_check_error;      /*!< Number of packets of the bursts received with a CRC or MIC error. */
    uint32_t   nb_packets_received_nok;     /*!< Number of wrong packets of the burst received . */
    uint32_t   nb_packets_expected;         /*!< Number of packets the device is expecting. */
    uint32_t   payload_size_expected;       /*!< The total payload size the device is expecting. */
    int32_t    rssi_mean;                   /*!< Received Signal Strength Indicator (RSSI) mean value of the burst. */
    bit_mask_t exchange_phase_success_mask; /*!< OR of smtc_flrp_exchange_phase_success_bit_t flags. */

    /*!< Bitfield of the packets missed in the burst (last burst in case of retry or multiburst).
    bit to 1 = packet of this bit index is missed */
    uint8_t last_burst_missed_packets_bitfield[SMTC_FLRP_BITFIELD_PACKETS_IN_BURST_LENGTH];
} smtc_flrp_burst_rx_stats_t;

/**
 * @brief SMTC FLRP WOR rx statistics
 */
typedef struct smtc_flrp_wor_rx_stats_s
{
    int16_t rssi; /*!< Received Signal Strength Indicator (RSSI) value of the WOR or WOR ACK. */
    int8_t  snr;  /*!< SNR value of the WOR or WOR ACK. */
} smtc_flrp_wor_rx_stats_t;

/**
 * @brief SMTC FLRP rx statistics
 */
typedef struct smtc_flrp_rx_stats_s
{
    smtc_flrp_burst_rx_stats_t burst;
    smtc_flrp_wor_rx_stats_t   wor; /*!< WOR or WOR ACK stats depending on the device role. */
} smtc_flrp_rx_stats_t;

/**
 * @brief SMTC FLRC Link adaptation mode
 */
typedef enum smtc_flrp_link_adaptation_mode_e
{
    SMTC_FLRP_LINK_ADAPTATION_FULLY_ENABLED           = 0x0,  //  Fully enabled - channel + datarate selection
    SMTC_FLRP_LINK_ADAPTATION_DISABLED                = 0x1,
    SMTC_FLRP_LINK_ADAPTATION_CHANNEL_SELECTION_ONLY  = 0x2,  // Channel selection only (use default datarate)
    SMTC_FLRP_LINK_ADAPTATION_DATARATE_SELECTION_ONLY = 0x3,  // Datarate selection only (use default channel)
    SMTC_FLRP_LINK_ADAPTATION_ENABLED_WITHOUT_SELECTION =
        0x4,  // Run but use defaults - sequence performed but no selection
} smtc_flrp_link_adaptation_mode_t;

typedef void ( *smtc_flrp_tx_done_f )( const void* context, smtc_flrp_return_code_t status, bool send_successful,
                                       uint8_t* dest_addr );
typedef void ( *smtc_flrp_rx_done_f )( const void* context, smtc_flrp_return_code_t status, uint32_t payload_size,
                                       uint8_t* src_addr, smtc_flrp_rx_stats_t flrp_stats );

/**
 * @brief SMTC FLRC Frequency Plan
 */
typedef enum smtc_flrp_frequency_plan_e
{
    SMTC_FLRP_FREQ_865MHz = 0x0,
    SMTC_FLRP_FREQ_2GHz4,
} smtc_flrp_frequency_plan_t;
/**
 * @brief SMTC FLRP API configuration
 */
typedef struct smtc_flrp_api_config_s
{
    uint8_t                    dev_eui[SMTC_FLRP_EUI_LENGTH];
    bool                       crypto_enabled;  // Enable per-packet AES-CMAC MIC (integrity/authentication) in place of
                                                // the radio CRC. Integrity only: the payload is NOT encrypted.
    smtc_flrp_frequency_plan_t freq_plan;
    uint32_t                   crystal_error;
} smtc_flrp_api_config_t;

typedef struct smtc_flrp_flrc_radio_config_s
{
    uint32_t                channels_freq_hz[SMTC_FLRP_NB_CHANNELS_MAX]; /*!< Frequencies of the channels. */
    uint8_t                 nb_channels; /*!< Number of channels set in channels_freq_hz. */
    uint8_t                 default_channel;
    ral_flrc_raw_bit_rate_t raw_bit_rate;
    int8_t                  tx_power_in_dbm; /*!< Transmission power in dBm. */

    /*!< Protocol delays */
    uint16_t interframe_delay_us;               /*!< Delay between frames in the burst */
    uint16_t start_burst_delay_us;              /*!< Delay before starting a burst */
    uint16_t start_ack_delay_us;                /*!< Delay before starting a burst ack */
    uint16_t adaptive_link_interframe_delay_us; /*!< Delay before starting a FLRC REQ or FLRC ACK frames */

    uint8_t burst_target_per; /*!<  the maximal PER to consider a burst valid (in %) */

} smtc_flrp_flrc_radio_config_t;

typedef struct smtc_flrp_wor_radio_config_s
{
    uint32_t      frequency_hz;
    ral_lora_sf_t sf;              //!< LoRa Spreading Factor
    ral_lora_bw_t bw;              //!< LoRa Bandwidth
    int8_t        tx_power_in_dbm; /*!< Transmission power in dBm. */

} smtc_flrp_wor_radio_config_t;

typedef struct smtc_flrp_radio_config_s
{
    smtc_flrp_flrc_radio_config_t flrc;
    smtc_flrp_wor_radio_config_t  wor_rx;
    smtc_flrp_wor_radio_config_t  wor_tx;
} smtc_flrp_radio_config_t;

/**
 * @brief SMTC FLRC communication configuration
 */
typedef struct smtc_flrp_com_config_s
{
    smtc_flrp_communication_mode_t com_mode;
    uint8_t                        slave_dev_eui[SMTC_FLRP_EUI_LENGTH];  // Target device EUI
    uint8_t filter_len; /*!< Number of Slave DevEUI MSB bits to be used for selectivity (0-63) -- Used only in mode
                           SMTC_FLRP_COM_ONE_WAY */
    smtc_flrp_link_adaptation_mode_t link_adaptation_mode;

} smtc_flrp_com_config_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Initialize the FLRC protocol.
 *
 * \param [in] config           Device API configuration
 * \param [in] tx_callback      Function called when a transmission is done
 * \param [in] rx_callback      Function called when a message is received
 * \param [in] user_context     User context passed in the callback function
 *
 * \return SMTC_FLRP_RC_OK in case of success.
 */
smtc_flrp_return_code_t smtc_flrp_init( smtc_flrp_api_config_t config, smtc_flrp_tx_done_f tx_callback,
                                        smtc_flrp_rx_done_f rx_callback, void* user_context );

/*!
 * \brief Run the FLRC protocol.
 */
void smtc_flrp_run_engine( void );

/*!
 * \brief Start the periodic search of messages (the device has a slave role).
 *
 * \param [in] payload          Pointer of the buffer of the reception payload
 * \param [in] payload_size     Size of the buffer
 *
 * \return SMTC_FLRP_RC_NOT_INIT if the protocol is not initialized yet.
 * SMTC_FLRP_RC_BUSY in case a reception can't be programmed at the moment.
 * SMTC_FLRP_RC_OK in case of success.
 */
smtc_flrp_return_code_t smtc_flrp_start_periodic_listening( uint8_t* payload, uint32_t payload_size );

/*!
 * \brief Stop the periodic search of messages.
 *
 * \return SMTC_FLRP_RC_NOT_INIT if the protocol is not initialized yet.
 * SMTC_FLRP_RC_OK in case of success.
 */
smtc_flrp_return_code_t smtc_flrp_stop_periodic_listening( void );

/*!
 * \brief Send to the protocol the data to send next (the device has a slave role).
 *
 *      To unset the buffer to send, set the payload_size to 0. In this cas, the slave will inform the initiator
 * initiating the reception that it has no data to send.
 *
 *
 * \param [in] payload          Pointer of the payload to send
 * \param [in] payload_size     Size of the payload
 *
 * \return SMTC_FLRP_RC_NOT_INIT if the protocol is not initialized yet.
 * SMTC_FLRP_RC_OK in case of success.
 */
smtc_flrp_return_code_t smtc_flrp_slave_prepare_data_to_send( uint8_t* payload, uint32_t payload_size );

/*!
 * \brief Start a transmission (the device has an initiator role).
 *
 * \param [in] payload          Pointer of the payload to send
 * \param [in] payload_size     Size of the payload
 * \param [in] com_config       Communication configuration
 *
 * \return SMTC_FLRP_RC_NOT_INIT if the protocol is not initialized yet.
 * SMTC_FLRP_RC_BUSY in case a transmission can't be programmed at the moment.
 * SMTC_FLRP_RC_OK in case of success.
 */
smtc_flrp_return_code_t smtc_flrp_initiate_transmission( uint8_t* payload, uint32_t payload_size,
                                                         smtc_flrp_com_config_t com_config );

/*!
 * \brief Start the reception procedure (the device has an initiator role).
 *
 * \param [in] payload          Payload of reception
 * \param [in] payload_size     Size of the payload
 * \param [in] com_config       Communication configuration
 *
 * In this case, the broadcast / one-way mode is not supported. Only the com mode SMTC_FLRP_BIDIRECTIONAL is possible.
 *
 * \return SMTC_FLRP_RC_NOT_INIT if the protocol is not initialized yet.
 * SMTC_FLRP_RC_OK in case of success.
 */
smtc_flrp_return_code_t smtc_flrp_initiate_reception( uint8_t* payload, uint32_t payload_size,
                                                      smtc_flrp_com_config_t com_config );
/*!
 * \brief Get the current flrp radio configuration
 *
 */
smtc_flrp_radio_config_t smtc_flrp_get_current_radio_config( void );

/*!
 * \brief Set a new flrp radio configuration
 *
 * \param [out] radio_config        The radio configuration structure
 *
 * \return SMTC_FLRP_RC_NOT_INIT if the protocol is not initialized yet.
 * SMTC_FLRP_RC_BUSY in case a transmission/reception is in progress.
 * SMTC_FLRP_RC_INVALID_PARAMS if the config is invalid.
 * SMTC_FLRP_RC_OK in case of success.
 */
smtc_flrp_return_code_t smtc_flrp_set_new_radio_config( smtc_flrp_radio_config_t radio_config );

/*!
 * \brief Check if the protocol needs to be run (to process time critical operations such as starting a reception
 * window or sending a frame). \return true if the protocol needs to be run by calling smtc_flrp_run_engine(), false
 * otherwise.
 */
bool smtc_flrp_call_run( void );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_FLRP_API_H

/* --- EOF ------------------------------------------------------------------ */
