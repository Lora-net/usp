/**
 * @file      app_per_fsk.h
 *
 * @brief     Simple PER (Packet Error Rate) example with FSK modulation for LR1110 or LR1120 chip
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

#ifndef __APP_PER_FSK_H__
#define __APP_PER_FSK_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#ifndef RF_FREQ_IN_HZ
#define RF_FREQ_IN_HZ 866500000
#endif

#ifndef TX_OUTPUT_POWER_DBM
#define TX_OUTPUT_POWER_DBM 14
#endif

#ifndef FSK_BITRATE
#define FSK_BITRATE 50000  // 50 kbps
#endif

#ifndef FSK_FDEV
#define FSK_FDEV 25000  // 25 kHz frequency deviation
#endif

#ifndef FSK_BANDWIDTH
#define FSK_BANDWIDTH 138000  // 138.0 kHz in Hz
#endif

#ifndef FSK_PREAMBLE_LENGTH
#define FSK_PREAMBLE_LENGTH 5  // bytes
#endif

#ifndef FSK_SYNC_WORD_LENGTH
#define FSK_SYNC_WORD_LENGTH 3  // bytes
#endif

#ifndef FSK_SYNC_WORD
#define FSK_SYNC_WORD NULL  // Use default sync word
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

void per_fsk_init( void );
void per_fsk_on_button_press( void );

#endif  // __APP_PER_FSK_H__
