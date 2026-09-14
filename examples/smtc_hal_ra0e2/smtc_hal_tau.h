/**
 * @file      smtc_hal_tau.h
 *
 * @brief     TAU timer driver for precise microsecond delays
 *
 *       Implementation Notes:
 *       ---------------------
 *       TAU channel 0 is configured in interval timer mode at 1MHz (CK00 = HOCO/32).
 *       This provides 1 microsecond resolution with 16-bit counter (max ~65ms).
 *
 *       Uses NVIC pending bit polling for ISR-safe operation. This allows the
 *       delay function to be called from within ISR context without deadlock.
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

#ifndef SMTC_HAL_TAU_H
#define SMTC_HAL_TAU_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/* Maximum single delay in microseconds (16-bit counter @ 1MHz) */
#define HAL_TAU_MAX_DELAY_US 65000U

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize TAU timer for precise microsecond delays
 */
void hal_tau_init( void );

/**
 * @brief Blocking delay using TAU hardware timer (ISR-safe)
 *
 * Uses TAU channel 0 configured at 1MHz (1us per tick) for precise timing.
 * Polls NVIC pending bit for ISR-safe operation (no interrupt dependency).
 *
 * @param[in] microseconds  Delay duration in microseconds (max 65000)
 */
void hal_tau_delay_us( uint32_t microseconds );

/**
 * @brief TAU interrupt handler (called from fsp_callbacks.c)
 *
 * @note Not used in current polling implementation, kept for FSP compatibility.
 */
void hal_tau_irq_handler( void );

#ifdef __cplusplus
}
#endif

#endif /* SMTC_HAL_TAU_H */
