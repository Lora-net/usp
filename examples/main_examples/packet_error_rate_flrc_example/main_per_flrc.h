/**
 * @file      main_per_flrc.h
 *
 * @brief     Simple PER (Packet Error Rate) example with FLRC modulation
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

#ifndef __MAIN_PER_FLRC_H__
#define __MAIN_PER_FLRC_H__

#include <stdint.h>
#include "ral.h"

#ifndef RF_FREQ_IN_HZ
#define RF_FREQ_IN_HZ 866500000
#endif

#ifndef TX_OUTPUT_POWER_DBM
#define TX_OUTPUT_POWER_DBM 14
#endif

#ifndef FLRC_RAW_BIT_RATE
#define FLRC_RAW_BIT_RATE RAL_FLRC_RAW_BIT_RATE_2_600_MBPS
#endif

#ifndef FLRC_CR
#define FLRC_CR RAL_FLRC_CR_3_4
#endif

#ifndef FLRC_PULSE_SHAPE
#define FLRC_PULSE_SHAPE RAL_FLRC_PULSE_SHAPE_BT_05
#endif

#ifndef FLRC_PREAMBLE_BITS
#define FLRC_PREAMBLE_BITS RAL_FLRC_PREAMBLE_LENGTH_32_BITS
#endif

#ifndef FLRC_SYNCWORD_LEN
#define FLRC_SYNCWORD_LEN RAL_FLRC_SYNCWORD_LENGTH_4_BYTES
#endif

#ifndef FLRC_TX_SYNCWORD
#define FLRC_TX_SYNCWORD RAL_FLRC_TX_SYNCWORD_1
#endif

#ifndef FLRC_MATCH_SYNCWORD
#define FLRC_MATCH_SYNCWORD RAL_FLRC_RX_MATCH_SYNCWORD_1
#endif

#ifndef FLRC_PLD_IS_FIX
#define FLRC_PLD_IS_FIX false
#endif

#ifndef FLRC_CRC
#define FLRC_CRC RAL_FLRC_CRC_2_BYTES
#endif

#endif  // __MAIN_PER_FLRC_H__
