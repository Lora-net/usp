/**
 * @file      smtc_hal_irq.c
 *
 * @brief     IRQ Management for RA0E2 Low Power Mode
 *
 * Implementation of functions for clearing pending interrupts before
 * entering sleep mode to prevent immediate wakeup from spurious IRQs.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_hal_irq.h"
#include "bsp_api.h"
#include "vector_data.h"
#include "R7FA0E209.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS --------------------------------------------------------
 */

void hal_irq_clear_all_pending( void )
{
    /*
     * Clear NMISR flags per Section 11.8 of RA0E2 User Manual:
     * "Whenever a WFI instruction is executed, confirm that all status
     * flags in the NMISR register are 0."
     *
     * Write 1 to NMICLR bits to clear corresponding NMISR flags.
     */
    R_ICU->NMICLR = ( uint16_t ) ( R_ICU_NMICLR_IWDTCLR_Msk | R_ICU_NMICLR_LVD1CLR_Msk | R_ICU_NMICLR_NMICLR_Msk |
                                   R_ICU_NMICLR_RPECLR_Msk );

    /*
     * Clear ALL NVIC pending bits, not just allocated IRQs.
     * This is necessary because spurious/uninitialized IRQs (like IRQ 11)
     * can be pending and cause immediate wakeup from WFI.
     *
     * RA0E2 has 42 IRQs (0-41), covered by ICPR[0] (IRQs 0-31) and ICPR[1] (IRQs 32-41).
     * Writing 1 to ICPR bit clears the corresponding pending bit.
     */
    NVIC->ICPR[0] = 0xFFFFFFFFU; /* Clear all pending IRQs 0-31 */
    NVIC->ICPR[1] = 0xFFFFFFFFU; /* Clear all pending IRQs 32-63 (only 32-41 used) */

    /* Clear SysTick pending (Cortex-M system exception, not in ICU) */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    /*
     * Clear TML hardware interrupt flags.
     * These are peripheral-level flags separate from ICU/NVIC.
     * If not cleared, they will re-trigger the interrupt.
     */
    R_TML->ITLS0 = 0x00U; /* Clear ITF00, ITF01, ITF02, ITF03 */

    /* Final barriers to ensure all clears complete before WFI */
    __DSB( );
    __ISB( );
}

void hal_irq_clear_all_pending_except( bool preserve_tml0, bool preserve_rtc, bool preserve_radio )
{
    /*
     * Clear NMISR flags per Section 11.8 (always required)
     */
    R_ICU->NMICLR = ( uint16_t ) ( R_ICU_NMICLR_IWDTCLR_Msk | R_ICU_NMICLR_LVD1CLR_Msk | R_ICU_NMICLR_NMICLR_Msk |
                                   R_ICU_NMICLR_RPECLR_Msk );

    /*
     * Clear ALL NVIC pending bits except specified wakeup sources.
     * Build mask of IRQs to preserve.
     */
    uint32_t preserve_mask_0 = 0;
    uint32_t preserve_mask_1 = 0;

    if( preserve_radio )
    {
        preserve_mask_0 |= ( 1UL << ICU_IRQ5_IRQn ); /* IRQ 7 - Radio DIO */
    }
    if( preserve_rtc )
    {
        preserve_mask_1 |= ( 1UL << ( RTC_ALARM_OR_PERIOD_IRQn - 32 ) ); /* IRQ 32 - RTC alarm */
    }
    if( preserve_tml0 )
    {
        preserve_mask_1 |= ( 1UL << ( TML0_ITL_OR_IRQn - 32 ) ); /* IRQ 33 - TML0/TML1 */
    }

    /* Clear all except preserved - write 1 to clear, 0 to preserve */
    NVIC->ICPR[0] = ~preserve_mask_0;
    NVIC->ICPR[1] = ~preserve_mask_1;

    /* Clear SysTick pending */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    /* Clear TML flags except if preserving TML0 */
    if( !preserve_tml0 )
    {
        R_TML->ITLS0 = 0x00U;
    }

    __DSB( );
    __ISB( );
}

/* --- EOF ------------------------------------------------------------------ */
