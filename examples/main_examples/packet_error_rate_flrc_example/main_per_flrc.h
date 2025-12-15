/**
 * @file      main_per_flrc.h
 *
 * @brief     Simple PER (Packet Error Rate) example with FLRC modulation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

#ifndef FLRC_BR_BPS
// #define FLRC_BR_BPS 520000
#define FLRC_BR_BPS 2600000
#endif

#ifndef FLRC_BW_HZ
// #define FLRC_BW_HZ 571000
#define FLRC_BW_HZ 2666000
#endif

#ifndef FLRC_CR
#define FLRC_CR RAL_FLRC_CR_3_4
#endif

#ifndef FLRC_PULSE_SHAPE
#define FLRC_PULSE_SHAPE RAL_FLRC_PULSE_SHAPE_BT_05
#endif

#ifndef FLRC_PREAMBLE_BITS
#define FLRC_PREAMBLE_BITS 32
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
