/**
 * @file      smtc_hal_irq.h
 *
 * @brief     IRQ Management for RA0E2 Low Power Mode
 *
 * Provides functions for clearing pending interrupts before entering
 * sleep mode to prevent immediate wakeup from spurious IRQs.
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

#ifndef SMTC_HAL_IRQ_H
#define SMTC_HAL_IRQ_H

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
 * --- PUBLIC FUNCTIONS --------------------------------------------------------
 */

/**
 * @brief Clear all pending interrupts before WFI
 *
 * Clears:
 * - NMISR flags (required per RA0E2 Section 11.8)
 * - All NVIC pending IRQs
 * - SysTick pending exception
 * - TML hardware interrupt flags
 */
void hal_irq_clear_all_pending( void );

/**
 * @brief Clear pending interrupts except specified wakeup sources
 *
 * @param preserve_tml0  Don't clear TML0/TML1 (wakeup/LP timer)
 * @param preserve_rtc   Don't clear RTC alarm
 * @param preserve_radio Don't clear radio DIO (ICU_IRQ5)
 */
void hal_irq_clear_all_pending_except( bool preserve_tml0, bool preserve_rtc, bool preserve_radio );

#ifdef __cplusplus
}
#endif

#endif /* SMTC_HAL_IRQ_H */
