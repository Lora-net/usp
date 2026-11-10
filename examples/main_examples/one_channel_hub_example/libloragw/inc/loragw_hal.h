/**
 * @file      loragw_hal.h
 *
 * @brief     LoRa Gateway Hardware Abstraction Layer - RAC-based implementation
 *
 * This HAL provides a gateway-like API compatible with packet forwarders,
 * implemented on top of RAC (Radio Access Controller) for a 1-channel hub.
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

#ifndef _LORAGW_HAL_H
#define _LORAGW_HAL_H

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

#define IS_LORA_BW( bw ) ( ( bw == BW_125KHZ ) || ( bw == BW_250KHZ ) || ( bw == BW_500KHZ ) )

#define IS_LORA_DR( dr )                                                                                  \
    ( ( dr == DR_LORA_SF5 ) || ( dr == DR_LORA_SF6 ) || ( dr == DR_LORA_SF7 ) || ( dr == DR_LORA_SF8 ) || \
      ( dr == DR_LORA_SF9 ) || ( dr == DR_LORA_SF10 ) || ( dr == DR_LORA_SF11 ) || ( dr == DR_LORA_SF12 ) )

#define IS_LORA_CR( cr ) \
    ( ( cr == CR_LORA_4_5 ) || ( cr == CR_LORA_4_6 ) || ( cr == CR_LORA_4_7 ) || ( cr == CR_LORA_4_8 ) )

#define IS_FSK_BW( bw ) ( ( bw >= 1 ) && ( bw <= 7 ) )
#define IS_FSK_DR( dr ) ( ( dr >= DR_FSK_MIN ) && ( dr <= DR_FSK_MAX ) )

#define IS_TX_MODE( mode ) ( ( mode == IMMEDIATE ) || ( mode == TIMESTAMPED ) || ( mode == ON_GPS ) )

#define SET_PPM_ON( bw, dr )                                                             \
    ( ( ( bw == BW_125KHZ ) && ( ( dr == DR_LORA_SF11 ) || ( dr == DR_LORA_SF12 ) ) ) || \
      ( ( bw == BW_250KHZ ) && ( dr == DR_LORA_SF12 ) ) )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/* Return status codes */
#define LGW_HAL_SUCCESS 0
#define LGW_HAL_ERROR -1
#define LGW_LBT_NOT_ALLOWED 1

/* Radio-specific parameters */
#define LGW_XTAL_FREQU 32000000 /* frequency of the RF reference oscillator */
#define LGW_RF_CHAIN_NB 1       /* number of RF chains */
#define LGW_RF_RX_BANDWIDTH \
    {                       \
        1000000, 1000000    \
    } /* bandwidth of the radios */

/* Concentrator chipset-specific parameters */
#define LGW_IF_CHAIN_NB 1    /* number of IF+modem RX chains */
#define LGW_REF_BW 125000    /* typical bandwidth of data channel */
#define LGW_MULTI_NB 0       /* number of LoRa 'multi SF' chains */
#define LGW_MULTI_SF_EN 0x00 /* bitmask to enable/disable SF for multi-sf correlators (12 11 10 9 8 7 6 5) */

/* Values available for the 'modulation' parameters */
#define MOD_UNDEFINED 0
#define MOD_CW 0x08
#define MOD_LORA 0x10
#define MOD_FSK 0x20

/* Values available for the 'bandwidth' parameters (LoRa & FSK) */
#define BW_UNDEFINED 0
#define BW_500KHZ 0x06
#define BW_250KHZ 0x05
#define BW_125KHZ 0x04

/* Values available for the 'datarate' parameters */
#define DR_UNDEFINED 0
#define DR_LORA_SF5 5
#define DR_LORA_SF6 6
#define DR_LORA_SF7 7
#define DR_LORA_SF8 8
#define DR_LORA_SF9 9
#define DR_LORA_SF10 10
#define DR_LORA_SF11 11
#define DR_LORA_SF12 12
/* NOTE: for FSK directly use baudrate between 500 bauds and 250 kbauds */
#define DR_FSK_MIN 500
#define DR_FSK_MAX 250000

/* Values available for the 'coderate' parameters (LoRa only) */
#define CR_UNDEFINED 0
#define CR_LORA_4_5 0x01
#define CR_LORA_4_6 0x02
#define CR_LORA_4_7 0x03
#define CR_LORA_4_8 0x04

/* Values available for the 'status' parameter (RX packet CRC status) */
#define STAT_UNDEFINED 0x00
#define STAT_NO_CRC 0x01
#define STAT_CRC_BAD 0x11
#define STAT_CRC_OK 0x10

/* Values available for the 'tx_mode' parameter */
#define IMMEDIATE 0
#define TIMESTAMPED 1
#define ON_GPS 2

/* Values available for 'select' in the status function */
#define TX_STATUS 1
#define RX_STATUS 2

/* Status codes for TX_STATUS */
#define TX_STATUS_UNKNOWN 0
#define TX_OFF 1       /* TX modem disabled, it will ignore commands */
#define TX_FREE 2      /* TX modem is free, ready to receive a command */
#define TX_SCHEDULED 3 /* TX modem is loaded, ready to send the packet */
#define TX_EMITTING 4  /* TX modem is emitting */

/* Status codes for RX_STATUS */
#define RX_STATUS_UNKNOWN 0
#define RX_OFF 1       /* RX modem is disabled, it will ignore commands */
#define RX_ON 2        /* RX modem is receiving */
#define RX_SUSPENDED 3 /* RX is suspended while a TX is ongoing */

/* Maximum size of Tx gain LUT */
#define TX_GAIN_LUT_SIZE_MAX 16

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @enum lgw_com_type_t
 * @brief Type of COM interface to connect to the concentrator
 */
typedef enum
{
    LGW_COM_SPI,
    LGW_COM_UNKNOWN
} lgw_com_type_t;

/**
 * @struct lgw_conf_board_s
 * @brief Configuration structure for board specificities
 */
struct lgw_conf_board_s
{
    bool           lorawan_public; /* Enable ONLY for *public* networks using the LoRa MAC protocol */
    lgw_com_type_t com_type;       /* The COM interface (SPI/USB) to connect to the SX1302 */
    char           com_path[64];   /* Path to access the COM device to connect to the SX1302 */
};

/**
 * @struct lgw_rssi_tcomp_s
 * @brief Structure containing all coefficients for RSSI temperature compensation
 */
struct lgw_rssi_tcomp_s
{
    float coeff_a;
    float coeff_b;
    float coeff_c;
    float coeff_d;
    float coeff_e;
};

/**
 * @struct lgw_conf_rxrf_s
 * @brief Configuration structure for a RF chain
 */
struct lgw_conf_rxrf_s
{
    bool                    enable;      /* enable or disable that RF chain */
    uint32_t                freq_hz;     /* center frequency of the radio in Hz */
    float                   rssi_offset; /* Board-specific RSSI correction factor */
    struct lgw_rssi_tcomp_s rssi_tcomp;  /* Board-specific RSSI temperature compensation */
    bool                    tx_enable;   /* enable or disable TX on that RF chain */
};

/**
 * @struct lgw_conf_rxif_s
 * @brief Configuration structure for an IF chain
 */
struct lgw_conf_rxif_s
{
    bool     enable;                  /* enable or disable that IF chain */
    uint8_t  rf_chain;                /* to which RF chain is that IF chain associated */
    int32_t  freq_hz;                 /* center freq of the IF chain, relative to RF chain frequency */
    uint8_t  bandwidth;               /* RX bandwidth, 0 for default */
    uint32_t datarate;                /* RX datarate, 0 for default */
    uint8_t  sync_word_size;          /* size of FSK sync word (number of bytes, 0 for default) */
    uint64_t sync_word;               /* FSK sync word (ALIGN RIGHT, eg. 0xC194C1) */
    bool     implicit_hdr;            /* LoRa Service implicit header */
    uint8_t  implicit_payload_length; /* LoRa Service implicit header payload length */
    bool     implicit_crc_en;         /* LoRa Service implicit header CRC enable */
    uint8_t  implicit_coderate;       /* LoRa Service implicit header coding rate */
};

/**
 * @struct lgw_conf_demod_s
 * @brief Configuration structure for LoRa/FSK demodulators
 */
struct lgw_conf_demod_s
{
    uint8_t multisf_datarate; /* bitmask to enable spreading-factors for correlators (SF12-SF5) */
};

/**
 * @struct lgw_pkt_rx_s
 * @brief Structure containing the metadata of a received packet
 */
struct lgw_pkt_rx_s
{
    uint32_t freq_hz;      /* central frequency of the IF chain */
    int32_t  freq_offset;  /* frequency offset in Hz */
    uint8_t  if_chain;     /* by which IF chain was packet received */
    uint8_t  status;       /* status of the received packet */
    uint32_t count_us;     /* internal counter for timestamping, 1 microsecond resolution */
    uint8_t  rf_chain;     /* through which RF chain the packet was received */
    uint8_t  modulation;   /* modulation used by the packet */
    uint8_t  bandwidth;    /* modulation bandwidth (LoRa only) */
    uint32_t datarate;     /* RX datarate of the packet (SF for LoRa) */
    uint8_t  coderate;     /* error-correcting code of the packet (LoRa only) */
    float    rssic;        /* average RSSI of the channel in dB */
    float    rssis;        /* average RSSI of the signal in dB */
    float    snr;          /* average packet SNR, in dB (LoRa only) */
    float    snr_min;      /* minimum packet SNR, in dB (LoRa only) */
    float    snr_max;      /* maximum packet SNR, in dB (LoRa only) */
    uint16_t crc;          /* CRC that was received in the payload */
    uint16_t size;         /* payload size in bytes */
    uint8_t  payload[256]; /* buffer containing the payload */
};

/**
 * @struct lgw_pkt_tx_s
 * @brief Structure containing the configuration of a packet to send
 */
struct lgw_pkt_tx_s
{
    uint32_t freq_hz;      /* center frequency of TX */
    uint8_t  tx_mode;      /* select on what event/time the TX is triggered */
    uint32_t count_us;     /* timestamp or delay in microseconds for TX trigger */
    uint8_t  rf_chain;     /* through which RF chain will the packet be sent */
    int8_t   rf_power;     /* TX power, in dBm */
    uint8_t  modulation;   /* modulation to use for the packet */
    int8_t   freq_offset;  /* frequency offset from Radio Tx frequency (CW mode) */
    uint8_t  bandwidth;    /* modulation bandwidth (LoRa only) */
    uint32_t datarate;     /* TX datarate (baudrate for FSK, SF for LoRa) */
    uint8_t  coderate;     /* error-correcting code of the packet (LoRa only) */
    bool     invert_pol;   /* invert signal polarity, for orthogonal downlinks (LoRa only) */
    uint8_t  f_dev;        /* frequency deviation, in kHz (FSK only) */
    uint16_t preamble;     /* set the preamble length, 0 for default */
    bool     no_crc;       /* if true, do not send a CRC in the packet */
    bool     no_header;    /* if true, enable implicit header mode (LoRa), fixed length (FSK) */
    uint16_t size;         /* payload size in bytes */
    uint8_t  payload[256]; /* buffer containing the payload */
};

/**
 * @struct lgw_tx_gain_s
 * @brief Structure containing all gains of Tx chain
 */
struct lgw_tx_gain_s
{
    int8_t  rf_power; /* measured TX power at the board connector, in dBm */
    uint8_t dig_gain; /* 2 bits: control of the digital gain of SX1302 */
    uint8_t pa_gain;  /* 2 bits: control of the external PA (SX1302 I/O) */
    uint8_t dac_gain; /* 2 bits: control of the radio DAC */
    uint8_t mix_gain; /* 4 bits: control of the radio mixer */
    int8_t  offset_i; /* calibrated I offset */
    int8_t  offset_q; /* calibrated Q offset */
    uint8_t pwr_idx;  /* 6 bits: control the radio power index (SX1250) */
};

/**
 * @struct lgw_tx_gain_lut_s
 * @brief Structure defining the Tx gain LUT
 */
struct lgw_tx_gain_lut_s
{
    struct lgw_tx_gain_s lut[TX_GAIN_LUT_SIZE_MAX]; /* Array of Tx gain struct */
    uint8_t              size;                      /* Number of LUT indexes */
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Configure the gateway board
 *
 * @param conf Structure containing the configuration parameters
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_board_setconf( struct lgw_conf_board_s* conf );

/**
 * @brief Configure an RF chain (must configure before start)
 *
 * @param rf_chain Number of the RF chain to configure [0, LGW_RF_CHAIN_NB - 1]
 * @param conf Structure containing the configuration parameters
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_rxrf_setconf( uint8_t rf_chain, struct lgw_conf_rxrf_s* conf );

/**
 * @brief Configure an IF chain + modem (must configure before start)
 *
 * @param if_chain Number of the IF chain + modem to configure [0, LGW_IF_CHAIN_NB - 1]
 * @param conf Structure containing the configuration parameters
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_rxif_setconf( uint8_t if_chain, struct lgw_conf_rxif_s* conf );

/**
 * @brief Configure LoRa/FSK demodulators
 *
 * @param conf Structure containing the configuration parameters
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_demod_setconf( struct lgw_conf_demod_s* conf );

/**
 * @brief Configure the Tx gain LUT
 *
 * @param rf_chain RF chain index
 * @param conf Pointer to structure defining the LUT
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_txgain_setconf( uint8_t rf_chain, struct lgw_tx_gain_lut_s* conf );

/**
 * @brief Connect to the LoRa concentrator, reset it and configure it
 *
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_start( void );

/**
 * @brief Stop the LoRa concentrator and disconnect it
 *
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_stop( void );

/**
 * @brief Fetch received packets from the LoRa concentrator
 *
 * @param max_pkt Maximum number of packets to retrieve
 * @param pkt_data Pointer to an array that will receive the packet data
 * @return LGW_HAL_ERROR if failed, else the number of packets retrieved
 */
int lgw_receive( uint8_t max_pkt, struct lgw_pkt_rx_s* pkt_data );

/**
 * @brief Schedule a packet to be sent
 *
 * @param pkt_data Structure containing the data and metadata for the packet
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_send( struct lgw_pkt_tx_s* pkt_data );

/**
 * @brief Get the status of the TX or RX modem
 *
 * @param rf_chain RF chain index
 * @param select TX_STATUS or RX_STATUS
 * @param code Pointer to receive the status code
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_status( uint8_t rf_chain, uint8_t select, uint8_t* code );

/**
 * @brief Abort a currently scheduled or ongoing TX
 *
 * @param rf_chain RF chain index
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_abort_tx( uint8_t rf_chain );

/**
 * @brief Return value of internal counter when latest event was captured
 *
 * @param trig_cnt_us Pointer to receive timestamp value
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_get_trigcnt( uint32_t* trig_cnt_us );

/**
 * @brief Return instantaneous value of internal counter
 *
 * @param inst_cnt_us Pointer to receive timestamp value
 * @return LGW_HAL_ERROR if the operation failed, LGW_HAL_SUCCESS else
 */
int lgw_get_instcnt( uint32_t* inst_cnt_us );

/**
 * @brief Allow user to check the version/options of the library
 *
 * @return Pointer to a human-readable null terminated string
 */
const char* lgw_version_info( void );

/**
 * @brief Return time on air of given packet, in milliseconds
 *
 * @param packet Pointer to the packet structure
 * @return The packet time on air in milliseconds
 */
uint32_t lgw_time_on_air( const struct lgw_pkt_tx_s* packet );

#endif /* _LORAGW_HAL_H */
