/**
 * @file      app_rf_certification.h
 *
 * @brief     RF Certification LoRa TX example for LR1110 or LR1120 chip
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

#ifndef __APP_RF_CERTIFICATION_H__
#define __APP_RF_CERTIFICATION_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * @brief RF frequency in Hz
 * Default value: 868.1 MHz (EU868 band)
 */
#ifndef RF_FREQ_IN_HZ
#define RF_FREQ_IN_HZ 868100000
#endif

/*!
 * @brief Payload length in bytes
 */
#ifndef PAYLOAD_SIZE
#define PAYLOAD_SIZE 255
#endif

/*!
 * @brief TX output power in dBm
 */
#ifndef TX_OUTPUT_POWER_DBM
#define TX_OUTPUT_POWER_DBM 14
#endif

/*!
 * @brief LoRa spreading factor
 */
#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR RAL_LORA_SF7
#endif

/*!
 * @brief LoRa bandwidth
 */
#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH RAL_LORA_BW_125_KHZ
#endif

/*!
 * @brief LoRa coding rate
 */
#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE RAL_LORA_CR_4_5
#endif

/*!
 * @brief LoRa preamble length
 */
#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 8
#endif

/*!
 * @brief LoRa packet length mode
 */
#ifndef LORA_PKT_LEN_MODE
#define LORA_PKT_LEN_MODE RAL_LORA_PKT_EXPLICIT
#endif

/*!
 * @brief LoRa IQ inversion
 */
#ifndef LORA_IQ
#define LORA_IQ false
#endif

/*!
 * @brief LoRa CRC enable
 */
#ifndef LORA_CRC
#define LORA_CRC true
#endif

/*!
 * @brief FSK modulation parameters
 */

#ifndef FSK_BITRATE
#define FSK_BITRATE 50000  // 50 kbps
#endif

#ifndef FSK_FDEV
#define FSK_FDEV 25000  // 25 kHz frequency deviation
#endif

#ifndef FSK_BANDWIDTH
#define FSK_BANDWIDTH 117000  // 117.0 kHz in Hz
#endif

#ifndef FSK_PREAMBLE_LENGTH
#define FSK_PREAMBLE_LENGTH 5  // bytes
#endif

#ifndef FSK_SYNC_WORD_LENGTH
#define FSK_SYNC_WORD_LENGTH 3  // bytes
#endif

#ifndef FSK_CRC
#define FSK_CRC RAL_GFSK_CRC_2_BYTES_INV  // 2-byte CRC
#endif

#ifndef FSK_WHITENING
#define FSK_WHITENING true
#endif

#ifndef FSK_PACKET_TYPE
#define FSK_PACKET_TYPE RAL_GFSK_PKT_VAR_LEN  // Variable length packet
#endif
/*!
 * \brief Default FSK configuration constants
 */
#define FSK_WHITENING_SEED ( 0x01FF )
#define FSK_CRC_SEED ( 0x1D0F )
#define FSK_CRC_POLYNOMIAL ( 0x1021 )
/*!
 * @brief LR-FHSS modulation parameters
 */
#ifndef LRFHSS_CODING_RATE
#define LRFHSS_CODING_RATE LR_FHSS_V1_CR_1_3  // Coding rate 1/3 for better sensitivity
#endif

#ifndef LRFHSS_BANDWIDTH
#define LRFHSS_BANDWIDTH LR_FHSS_V1_BW_136719_HZ  // 136.719 kHz bandwidth
#endif

#ifndef LRFHSS_GRID
#define LRFHSS_GRID LR_FHSS_V1_GRID_3906_HZ  // 3.906 kHz grid step
#endif

#ifndef LRFHSS_ENABLE_HOPPING
#define LRFHSS_ENABLE_HOPPING true  // Enable frequency hopping
#endif

#ifndef LRFHSS_DEVICE_OFFSET
#define LRFHSS_DEVICE_OFFSET 0  // No device offset
#endif

#ifndef LRFHSS_HOP_SEQUENCE_ID
#define LRFHSS_HOP_SEQUENCE_ID 0  // Auto-generate hop sequence
#endif

/*!
 * @brief FLRC modulation parameters
 */
#ifndef FLRC_BITRATE
#define FLRC_BITRATE 2600000
#endif

#ifndef FLRC_BANDWIDTH
#define FLRC_BANDWIDTH 2666000
#endif

#ifndef FLRC_CODING_RATE
#define FLRC_CODING_RATE RAL_FLRC_CR_1_1  // Coding rate 1/1
#endif

#ifndef FLRC_PULSE_SHAPE
#define FLRC_PULSE_SHAPE RAL_FLRC_PULSE_SHAPE_BT_05  // Gaussian BT 0.5
#endif

#ifndef FLRC_PREAMBLE_LENGTH
#define FLRC_PREAMBLE_LENGTH 16  // bits (2 bytes)
#endif

#ifndef FLRC_SYNC_WORD_LENGTH
#define FLRC_SYNC_WORD_LENGTH RAL_FLRC_SYNCWORD_LENGTH_4_BYTES  // 4 bytes
#endif

#ifndef FLRC_CRC_TYPE
#define FLRC_CRC_TYPE RAL_FLRC_CRC_2_BYTES  // 2-byte CRC
#endif

#ifndef FLRC_PACKET_TYPE
#define FLRC_PACKET_TYPE false  // Variable length packet
#endif

/**
 * @brief Frequencies used in FCC-regulated territories
 */
#define RF_FREQUENCY_902_3 902300000u  // Hz
#define RF_FREQUENCY_903_0 903000000u  // Hz
#define RF_FREQUENCY_908_7 908700000u  // Hz
#define RF_FREQUENCY_909_4 909400000u  // Hz
#define RF_FREQUENCY_914_2 914200000u  // Hz
#define RF_FREQUENCY_914_9 914900000u  // Hz
#define RF_FREQUENCY_927_7 927700000u  // Hz

/**
 * @brief The valid LoRaWAN subbands in hybrid mode
 */
#define LORAWAN_SUBBAND_0 0  // 902.3 - 903.7 MHz
#define LORAWAN_SUBBAND_1 1  // 903.9 - 905.3 MHz
#define LORAWAN_SUBBAND_2 2  // 905.5 - 906.9 MHz
#define LORAWAN_SUBBAND_3 3  // 907.1 - 908.5 MHz
#define LORAWAN_SUBBAND_4 4  // 908.7 - 910.1 MHz
#define LORAWAN_SUBBAND_5 5  // 910.3 - 911.7 MHz
#define LORAWAN_SUBBAND_6 6  // 911.9 - 913.3 MHz
#define LORAWAN_SUBBAND_7 7  // 913.5 - 914.9 MHz

/**
 * @brief The LoRaWAN DR0 (SF10/125 kHz) payload length
 */
#define LORAWAN_DR0_PAYLOAD_LENGTH 24

/**
 * @brief The LoRaWAN DR1 (SF9/125 kHz) payload length
 */
#define LORAWAN_DR1_PAYLOAD_LENGTH 66

/**
 * @brief The LoRaWAN DR2 (SF8/125 kHz) payload length
 */
#define LORAWAN_DR2_PAYLOAD_LENGTH 138

/**
 * @brief The LoRaWAN DR3 (SF7/125 kHz) payload length
 */
#define LORAWAN_DR3_PAYLOAD_LENGTH 235

/**
 * @brief The LoRaWAN DR4 (SF8/500 kHz) payload length
 */
#define LORAWAN_DR4_PAYLOAD_LENGTH 235

/**
 * @brief The LoRaWAN DR8 (SF12/500 kHz) payload length
 */
#define LORAWAN_DR8_PAYLOAD_LENGTH 66

/**
 * @brief The LoRaWAN DR9 (SF11/500 kHz) payload length
 */
#define LORAWAN_DR9_PAYLOAD_LENGTH 142

/**
 * @brief The LoRaWAN DR10 (SF10/500 kHz) payload length
 */
#define LORAWAN_DR10_PAYLOAD_LENGTH 255

/**
 * @brief The LoRaWAN DR11 (SF9/500 kHz) payload length
 */
#define LORAWAN_DR11_PAYLOAD_LENGTH 255

/**
 * @brief The LoRaWAN DR12 (SF8/500 kHz) payload length
 */
#define LORAWAN_DR12_PAYLOAD_LENGTH 255

/**
 * @brief The LoRaWAN DR13 (SF7/500 kHz) payload length
 */
#define LORAWAN_DR13_PAYLOAD_LENGTH 255

/**
 * @brief The number of channels that will be swept during the FCC FHSS sweep time
 */
#define FCC_SWEEP_FHSS_CHANNELS_NUM ( 64 )

/**
 * @brief The number of channels that will be swept during the FCC hybrid sweep time
 */
#define FCC_SWEEP_HYBRID_CHANNELS_NUM ( 8 )

/**
 * @brief The width in hz of a FCC channel during sweep mode
 */
#define FCC_SWEEP_CHANNEL_WIDTH_HZ 200000

/*!
 * @brief RF Certification Example State Machine
 */
typedef enum
{
    RF_CERT_STATE_IDLE = 0, /*!< Idle state, waiting for start */

    /*ETSI*/ RF_CERT_STATE_LORA_ETSI_863_1_SF7_BW_125_POWER_14_DBM = 1,  /*!< Actively transmitting packets */
    RF_CERT_STATE_LORA_ETSI_863_1_SF12_BW_125_POWER_14_DBM         = 2,  /*!< Actively transmitting packets */
    RF_CERT_STATE_LORA_ETSI_868_3_SF7_BW_125_POWER_14_DBM          = 3,  /*!< Actively transmitting packets */
    RF_CERT_STATE_LORA_ETSI_868_3_SF12_BW_125_POWER_14_DBM         = 4,  /*!< Actively transmitting packets */
    RF_CERT_STATE_LORA_ETSI_868_5_SF7_BW_125_POWER_14_DBM          = 5,  /*!< Actively transmitting packets */
    RF_CERT_STATE_LORA_ETSI_868_5_SF12_BW_125_POWER_14_DBM         = 6,  /*!< Actively transmitting packets */
    RF_CERT_STATE_LORA_ETSI_868_3_SF7_BW_250_POWER_14_DBM          = 7,  /*!< Actively transmitting packets */
    RF_CERT_STATE_LORA_ETSI_866_5_SF7_BW_125_POWER_14_DBM          = 8,  /*!< Actively transmitting packets */
    RF_CERT_STATE_LORA_ETSI_866_5_SF12_BW_125_POWER_14_DBM         = 9,  /*!< Actively transmitting packets */
    RF_CERT_STATE_FSK_ETSI_863_1_POWER_14_DBM                      = 10, /*!< Actively transmitting packets */
    RF_CERT_STATE_FSK_ETSI_866_5_POWER_14_DBM                      = 11, /*!< Actively transmitting packets */
    RF_CERT_STATE_FSK_ETSI_868_1_POWER_14_DBM                      = 12, /*!< Actively transmitting packets */
    RF_CERT_STATE_FSK_ETSI_868_3_POWER_14_DBM                      = 13, /*!< Actively transmitting packets */
    RF_CERT_STATE_FSK_ETSI_868_5_POWER_14_DBM                      = 14, /*!< Actively transmitting packets */
    RF_CERT_STATE_FSK_ETSI_869_9_POWER_14_DBM                      = 15, /*!< Actively transmitting packets */
    RF_CERT_STATE_LR_FHSS_ETSI_863_1_POWER_14_DBM                  = 16, /*!< Actively transmitting packets */
    RF_CERT_STATE_LR_FHSS_ETSI_866_5_POWER_14_DBM                  = 17, /*!< Actively transmitting packets */
    RF_CERT_STATE_LR_FHSS_ETSI_868_1_POWER_14_DBM                  = 18, /*!< Actively transmitting packets */
    RF_CERT_STATE_LR_FHSS_ETSI_868_3_POWER_14_DBM                  = 19, /*!< Actively transmitting packets */
    RF_CERT_STATE_LR_FHSS_ETSI_868_5_POWER_14_DBM                  = 20, /*!< Actively transmitting packets */
    RF_CERT_STATE_LR_FHSS_ETSI_869_9_POWER_14_DBM                  = 21, /*!< Actively transmitting packets */

    RF_CERT_STATE_FLRC_ETSI_863_1_POWER_14_DBM = 22, /*!< Actively transmitting packets */
    RF_CERT_STATE_FLRC_ETSI_866_5_POWER_14_DBM = 23, /*!< Actively transmitting packets */
    RF_CERT_STATE_FLRC_ETSI_868_1_POWER_14_DBM = 24, /*!< Actively transmitting packets */
    RF_CERT_STATE_FLRC_ETSI_868_3_POWER_14_DBM = 25, /*!< Actively transmitting packets */
    RF_CERT_STATE_FLRC_ETSI_868_5_POWER_14_DBM = 26, /*!< Actively transmitting packets */
    RF_CERT_STATE_FLRC_ETSI_869_9_POWER_14_DBM = 27, /*!< Actively transmitting packets */

    /*ARIB*/ RF_CERT_STATE_LORA_ARIB_920_6_SF7_BW_125_POWER_9_DBM = 28,
    RF_CERT_STATE_LORA_ARIB_920_6_SF12_BW_125_POWER_9_DBM         = 29,
    RF_CERT_STATE_LORA_ARIB_922_0_SF7_BW_125_POWER_9_DBM          = 30,
    RF_CERT_STATE_LORA_ARIB_922_0_SF12_BW_125_POWER_9_DBM         = 31,
    RF_CERT_STATE_LORA_ARIB_923_4_SF7_BW_125_POWER_9_DBM          = 32,
    RF_CERT_STATE_LORA_ARIB_923_4_SF12_BW_125_POWER_9_DBM         = 33,
    RF_CERT_STATE_FSK_ARIB_920_6_POWER_9_DBM                      = 34,
    RF_CERT_STATE_FSK_ARIB_922_0_POWER_9_DBM                      = 35,
    RF_CERT_STATE_FSK_ARIB_923_4_POWER_9_DBM                      = 36,
    RF_CERT_STATE_LR_FHSS_ARIB_920_6_POWER_9_DBM                  = 37,
    RF_CERT_STATE_LR_FHSS_ARIB_922_0_POWER_9_DBM                  = 38,
    RF_CERT_STATE_LR_FHSS_ARIB_923_4_POWER_9_DBM                  = 39,
    RF_CERT_STATE_FLRC_ARIB_920_6_POWER_9_DBM                     = 40, /*!< Actively transmitting packets */
    RF_CERT_STATE_FLRC_ARIB_922_0_POWER_9_DBM                     = 41, /*!< Actively transmitting packets */
    RF_CERT_STATE_FLRC_ARIB_923_4_POWER_9_DBM                     = 42, /*!< Actively transmitting packets */
    /*FCC*/

    /**********************************************/
    /*                  DTS MODE                  */
    /**********************************************/
    RF_CERT_STATE_LORA_FCC_903_0_SF8_BW_500_POWER_22_DBM_DR4_PAYLOAD_LENGTH = 43,
    RF_CERT_STATE_LORA_FCC_909_4_SF8_BW_500_POWER_22_DBM_DR4_PAYLOAD_LENGTH = 44,
    RF_CERT_STATE_LORA_FCC_914_2_SF8_BW_500_POWER_22_DBM_DR4_PAYLOAD_LENGTH = 45,

    /**********************************************/
    /*                 HYBRID MODE                */
    /**********************************************/
    RF_CERT_STATE_LORA_FCC_902_3_SF7_BW_125_POWER_22_DBM_DR3_PAYLOAD_LENGTH  = 46,
    RF_CERT_STATE_LORA_FCC_902_3_SF10_BW_125_POWER_22_DBM_DR0_PAYLOAD_LENGTH = 47,
    RF_CERT_STATE_LORA_FCC_908_7_SF7_BW_125_POWER_22_DBM_DR3_PAYLOAD_LENGTH  = 48,
    RF_CERT_STATE_LORA_FCC_908_7_SF10_BW_125_POWER_22_DBM_DR0_PAYLOAD_LENGTH = 49,
    RF_CERT_STATE_LORA_FCC_914_9_SF7_BW_125_POWER_22_DBM_DR3_PAYLOAD_LENGTH  = 50,
    RF_CERT_STATE_LORA_FCC_914_9_SF10_BW_125_POWER_22_DBM_DR0_PAYLOAD_LENGTH = 51,

    /**********************************************/
    /*                 FHSS SWEEP                 */
    /**********************************************/
    RF_CERT_STATE_FCC_SWEEP_FHSS_SF10_DR0_PAYLOAD_LENGTH = 52,
    RF_CERT_STATE_FCC_SWEEP_FHSS_SF9_DR1_PAYLOAD_LENGTH  = 53,
    RF_CERT_STATE_FCC_SWEEP_FHSS_SF8_DR2_PAYLOAD_LENGTH  = 54,
    RF_CERT_STATE_FCC_SWEEP_FHSS_SF7_DR3_PAYLOAD_LENGTH  = 55,

    /**********************************************/
    /*                 HYBRID SWEEP               */
    /**********************************************/
    RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF10_DR0_PAYLOAD_LENGTH = 56,
    RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF9_DR1_PAYLOAD_LENGTH  = 57,
    RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF8_DR2_PAYLOAD_LENGTH  = 58,
    RF_CERT_STATE_FCC_SWEEP_HYBRID_SUBBAND0_SF7_DR3_PAYLOAD_LENGTH  = 59,

} rf_certification_state_t;

typedef enum rf_certification_region_e
{
    RF_CERT_REGION_ETSI,
    RF_CERT_REGION_ARIB,
    RF_CERT_REGION_FCC
} rf_certification_region_t;

#define RF_FREQUENCY_863_1 863100000
#define RF_FREQUENCY_866_5 866500000
#define RF_FREQUENCY_868_1 868100000
#define RF_FREQUENCY_868_3 868300000
#define RF_FREQUENCY_868_5 868500000
#define RF_FREQUENCY_869_9 869900000

#define RF_FREQUENCY_920_6 920600000
#define RF_FREQUENCY_922_0 922000000
#define RF_FREQUENCY_923_4 923400000

#define EXPECTED_PWR_14_DBM 14
#define EXPECTED_PWR_9_DBM 9
#define EXPECTED_PWR_22_DBM 22

#define LBT_SNIFF_DURATION_MS_DEFAULT ( 5 )
#define LBT_THRESHOLD_DBM_DEFAULT ( int16_t )( -80 )
#define LBT_BW_HZ__DEFAULT ( 200000 )
#define LRFHSS_PAYLOAD_SIZE 7
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize the RF certification application (LoRa mode)
 */
void rf_certification_init( void );

/*!
 * @brief Initialize the RF certification application (FSK mode)
 */
void rf_certification_init_fsk( void );

/*!
 * @brief Initialize the RF certification application (LR-FHSS mode)
 */
void rf_certification_init_lrfhss( void );

/*!
 * @brief Initialize the RF certification application (FLRC mode)
 */
void rf_certification_init_flrc( void );

/*!
 * @brief Handle button press event
 */
void rf_certification_on_button_press( void );

/*!
 * @brief Check if transmission is active
 * @return true if transmission is active, false otherwise
 */
bool rf_certification_is_transmission_active( void );

/*!
 * @brief Send a single packet
 */
void rf_certification_send_packet( void );

#ifdef __cplusplus
}
#endif

#endif  // __APP_RF_CERTIFICATION_H__

/* --- EOF ------------------------------------------------------------------ */
