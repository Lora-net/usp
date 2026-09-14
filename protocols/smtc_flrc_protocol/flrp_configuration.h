 /**
 * @file      flrp_configuration.h
 *
 * @brief     Common configuration header for FLRC Protocol parameters.
 *
 * This file centralizes all the key configuration macros and parameters used by the application,
 * especially for FLRC operations. It provides default values for frequency, power,
 * payload length, modulation parameters, and other radio settings. These macros ensure that all
 * parts of the application use consistent radio settings and make it easy to adapt the configuration
 * for different regions, hardware, or use cases.
 *
 * 
 * The Clear BSD License
 * Copyright Semtech Corporation 2026. All rights reserved.
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

#ifndef FLRP_CONFIGURATION_H
#define FLRP_CONFIGURATION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 * Includes standard integer types and the radio abstraction layer definitions.
 */
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 * These macros define the default configuration for the radio and LoRa modulation.
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPE -----------------------------------------------------------
 * These macros define the default configuration for the radio and LoRa modulation.
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 * (None defined in this file, but section reserved for future use.)
 */

/*!
 * @brief Default TX output power in dBm.
 *        Range: [-17, +22] for sub-GHz, [-18, 13] for 2.4GHz (HF_PA).
 */
#ifndef TX_OUTPUT_POWER_DBM
#define TX_OUTPUT_POWER_DBM 14
#endif

/*********************************
 * WOR LORA parameters
 *********************************/
#ifndef WOR_865MHz_RF_FREQ_IN_HZ
#define WOR_865MHz_RF_FREQ_IN_HZ 865100000
#endif

#ifndef WOR_2GHz4_RF_FREQ_IN_HZ
#define WOR_2GHz4_RF_FREQ_IN_HZ 2412000000
#endif

#ifndef WOR_LORA_SPREADING_FACTOR
#define WOR_LORA_SPREADING_FACTOR RAL_LORA_SF10
#endif

#ifndef WOR_LORA_BANDWIDTH
#define WOR_LORA_BANDWIDTH RAL_LORA_BW_500_KHZ
#endif

#ifndef WOR_LORA_CODING_RATE
#define WOR_LORA_CODING_RATE RAL_LORA_CR_LI_4_8
#endif

/*
 * ------------------------------------------------------------------------------
 * WOR preset selection (long preamble length + slave RX period)
 * ------------------------------------------------------------------------------
 *
 * `WOR_LORA_LONG_PREAMBLE_MS` and `RESTART_WOR_RX_DELAY_MS` MUST be set as a
 * coherent pair so that the slave's RX period stays smaller than the
 * initiator's TX preamble duration (with a safety margin).
 *
 * Preset A (default) - "Slow wake-up", power-efficient slave:
 *   WOR_LORA_LONG_PREAMBLE_MS = 1000  (1 s of long preamble)
 *   RESTART_WOR_RX_DELAY_MS   = 950   (~962 ms period with the 12.3 ms RX window at SF10/BW500, ~4% margin)
 *   --> Slave RX consumption percentage  ~1.3%
 *   --> Wake-up latency      up to ~1 s
 *
 * Preset B - "Fast wake-up", higher slave RX duty cycle, power efficient initiator:
 *   WOR_LORA_LONG_PREAMBLE_MS = 100   (100 ms of long preamble)
 *   RESTART_WOR_RX_DELAY_MS   = 80    (~92 ms period with the 12.3 ms RX window at SF10/BW500, ~8% margin)
 *   --> Slave RX consumption percentage  ~13%
 *   --> Wake-up latency      up to ~100 ms (10x faster than preset A)
 *
 * Custom preset: you can define your own pair instead of A/B. Recommendation:
 *   slave RX period = RESTART_WOR_RX_DELAY_MS + slave RX window (~12.3 ms at SF10/BW500)
 *   this period MUST stay < WOR_LORA_LONG_PREAMBLE_MS, keeping a ~5-8% margin for drift/jitter,
 *   otherwise the slave can miss the WOR.
 *     - longer preamble  -> lower slave RX duty cycle (lower slave power), higher latency & initiator cost
 *     - shorter preamble -> faster wake-up, higher slave RX duty cycle
 *   Wake-up latency ~= WOR_LORA_LONG_PREAMBLE_MS; slave RX duty cycle ~= RX window / slave RX period.
 *   See doc/FLRP_guidelines.md ("WOR wake-up preset") for the detailed recommendations.
 *
 * To switch presets: comment the active line, uncomment the alternative.
 * Both macros must be switched together.
 *
 * Independent of the preset (do not change for A<->B switch):
 *   WOR_SYM_NB_TIMEOUT    (current 6 = ~12.3 ms slave RX window at SF10/BW500)
 *   WOR_RX_TIMEOUT        (hard cap on slave RX window, ms)
 *   WOR_ACK_RX_TIMEOUT    (cap on initiator ACK RX window, ms)
 */
#ifndef WOR_LORA_LONG_PREAMBLE_MS
#define WOR_LORA_LONG_PREAMBLE_MS 1000      /* preset A: 1 s long preamble */
/* #define WOR_LORA_LONG_PREAMBLE_MS 100 */ /* preset B: 100 ms long preamble */
#endif

#ifndef WOR_LORA_IQ
#define WOR_LORA_IQ true
#endif

#ifndef WOR_LORA_CRC
#define WOR_LORA_CRC true
#endif

/*
 * Hard cap on slave RX window duration (ms). In practice the radio should
 * exit earlier via WOR_SYM_NB_TIMEOUT (symbols).
 *
 * Auto-derived: must cover the full WOR LoRa packet airtime AFTER preamble detection.
 *   = WOR_LORA_LONG_PREAMBLE_MS  (chip waits for preamble end)
 *   + LoRa ToA of a 91-byte WOR packet (header + payload + CRC)
 *   + safety margin (radio startup, scheduler jitter, crystal drift)
 *
 * The default +500 ms margin covers SF5..SF8 with BW >= 125 kHz (max post-preamble ToA ~270 ms).
 *
 * For slower modulations, override the margin by changing the +500 value below:
 *
 *   SF        | BW 125 kHz | BW 250 kHz | BW 500 kHz | BW 1000 kHz
 *   ----------|------------|------------|------------|------------
 *   SF5..SF8  | use +500   | use +500   | use +500   | use +500     (default OK)
 *   SF9       | use +600   | use +500   | use +500   | use +500
 *   SF10      | use +1000  | use +600   | use +500   | use +500
 *   SF11      | use +2000  | use +1100  | use +700   | use +500
 *   SF12      | use +3500  | use +1900  | use +1000  | use +600
 *
 * Above SF12/BW125 the WOR packet ToA exceeds 3 s -- at that point the WOR scheme itself
 * stops being efficient.
 *
 * Note: the radio still exits early via WOR_SYM_NB_TIMEOUT (symbols) when no preamble is
 * detected. WOR_RX_TIMEOUT only matters when a WOR is actually being received.
 */
#ifndef WOR_RX_TIMEOUT
#define WOR_RX_TIMEOUT ( WOR_LORA_LONG_PREAMBLE_MS + 500 )
#endif

/*
 * Cap on initiator ACK RX window duration (ms). Sized to cover
 * WOR_ACK_DELAY_MS + ACK preamble + ACK payload + drifts. Preset-independent.
 */
#ifndef WOR_ACK_RX_TIMEOUT
#define WOR_ACK_RX_TIMEOUT 300
#endif

/*
 * Number of LoRa symbols the slave radio waits before giving up on preamble
 * detection. T_RX_window = WOR_SYM_NB_TIMEOUT * (2^SF / BW).
 *
 * Current value: at SF10/BW500, 6 symb ~= 12.3 ms slave RX window when no signal.
 * Slave RX consumption percentage with preset A (gap 950 ms) is ~1.3%.
 *
 * Trade-off:
 *  - Lower values  -> shorter RX window, less power, but less robust preamble detection.
 *  - Higher values -> longer RX window, more power, more robust detection in noisy RF.
 *  Validated working values on LR2021 SF10/BW500: 6 symbols
 *
 * Preset-independent.
 */
#ifndef WOR_SYM_NB_TIMEOUT
#define WOR_SYM_NB_TIMEOUT 6
#endif

#ifndef WOR_ACK_DELAY_MS
#define WOR_ACK_DELAY_MS 5
#endif

#ifndef WOR_ACK_LORA_PREAMBLE_LENGTH
#define WOR_ACK_LORA_PREAMBLE_LENGTH 12
#endif

#ifndef LORA_PKT_LEN_MODE
#define LORA_PKT_LEN_MODE RAL_LORA_PKT_EXPLICIT
#endif

/*
 * Gap between the end of a slave RX window and the start of the next one
 * (i.e. start-to-start period = RESTART_WOR_RX_DELAY_MS + RX window duration).
 * Must be paired with WOR_LORA_LONG_PREAMBLE_MS (see preset comment above).
 */
#ifndef RESTART_WOR_RX_DELAY_MS
#define RESTART_WOR_RX_DELAY_MS 950      /* preset A: paired with WOR_LORA_LONG_PREAMBLE_MS = 1000 */
/* #define RESTART_WOR_RX_DELAY_MS 80 */ /* preset B: paired with WOR_LORA_LONG_PREAMBLE_MS = 100 (SF10/BW500) */
#endif

/*!
 * @brief LoRa sync word.
 */
#ifndef LORA_SYNCWORD
#define LORA_SYNCWORD LORA_PRIVATE_NETWORK_SYNCWORD
#endif

/*********************************
 * DATA FLRC parameters
 *********************************/

static const uint8_t flrc_default_syncword_1[4] = { 0x90, 0x56, 0x34, 0x12 };
static const uint8_t flrc_default_syncword_2[4] = { 0x00, 0x00, 0x00, 0x00 };
static const uint8_t flrc_default_syncword_3[4] = { 0x00, 0x00, 0x00, 0x00 };

#ifndef DATA_FLRC_865MHz_RF_FREQ_IN_HZ
#define DATA_FLRC_865MHz_RF_FREQ_IN_HZ WOR_865MHz_RF_FREQ_IN_HZ
#endif

#ifndef DATA_FLRC_2GHz4_RF_FREQ_IN_HZ
#define DATA_FLRC_2GHz4_RF_FREQ_IN_HZ WOR_2GHz4_RF_FREQ_IN_HZ
#endif

#ifndef DATA_FLRC_RAW_BIT_RATE
#define DATA_FLRC_RAW_BIT_RATE RAL_FLRC_RAW_BIT_RATE_2_600_MBPS
#endif

#ifndef DATA_FLRC_CR
#define DATA_FLRC_CR RAL_FLRC_CR_1_2
#endif

#ifndef DATA_FLRC_PULSE_SHAPE
#define DATA_FLRC_PULSE_SHAPE RAL_FLRC_PULSE_SHAPE_BT_05
#endif

#ifndef DATA_FLRC_PREAMBLE_BITS
#define DATA_FLRC_PREAMBLE_BITS RAL_FLRC_PREAMBLE_LENGTH_32_BITS
#endif

#ifndef DATA_FLRC_SYNCWORD_LEN
#define DATA_FLRC_SYNCWORD_LEN RAL_FLRC_SYNCWORD_LENGTH_4_BYTES
#endif

#ifndef DATA_FLRC_SYNCWORD
#define DATA_FLRC_SYNCWORD RAL_FLRC_TX_SYNCWORD_1
#endif

#ifndef DATA_FLRC_MATCH_SYNCWORD
#define DATA_FLRC_MATCH_SYNCWORD RAL_FLRC_RX_MATCH_SYNCWORD_1
#endif

#ifndef DATA_FLRC_PLD_IS_FIX
#define DATA_FLRC_PLD_IS_FIX false
#endif

#ifndef DATA_FLRC_CRC
#define DATA_FLRC_CRC RAL_FLRC_CRC_2_BYTES
#endif

#ifndef DATA_FLRC_CRC_SEED
#define DATA_FLRC_CRC_SEED 0xffffffff
#endif

#ifndef DATA_FLRC_CRC_POLYNOMIAL
#define DATA_FLRC_CRC_POLYNOMIAL 0x755B
#endif

#ifndef DATA_FLRC_NB_FREQ
#define DATA_FLRC_NB_FREQ 3
#endif

/*
 * Frequency hopping step between consecutive FLRC channels (Hz).
 * Channel N center = base_freq + DATA_FLRC_CHANNEL_STEP_*_HZ * N.
 * Adjust to keep all DATA_FLRC_NB_FREQ channels (with their bandwidth) inside
 * the regulatory sub-band of the target market.
 */
#ifndef DATA_FLRC_CHANNEL_STEP_865MHz_HZ
#define DATA_FLRC_CHANNEL_STEP_865MHz_HZ 2700000
#endif

#ifndef DATA_FLRC_CHANNEL_STEP_2GHz4_HZ
#define DATA_FLRC_CHANNEL_STEP_2GHz4_HZ 5000000
#endif

// 100 us step only (190 is the minimal interframe, pa-ramp-time is applied for up & down)
// 200 if LR20XX_RADIO_COMMON_RAMP_48_US (190 => 200)
// 400 if LR20XX_RADIO_COMMON_RAMP_128_US (190 + (128-48)*2 => 400)
// 700 if LR20XX_RADIO_COMMON_RAMP_272_US (190 + (272-48)*2 => 700)
#ifndef DATA_FLRC_MIN_INTERFRAME_DURATION_US
#define DATA_FLRC_MIN_INTERFRAME_DURATION_US 700
#endif

// 100 : no retry, less : max 3 retry
#ifndef DATA_FLRC_BURST_TARGET_PER_PERCENTAGE
#define DATA_FLRC_BURST_TARGET_PER_PERCENTAGE 0
#endif

static const uint8_t flrc_default_key[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                              0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

#ifdef __cplusplus
}
#endif

#endif  // FLRP_CONFIGURATION_H

/* --- EOF ------------------------------------------------------------------ */
