/**
 * @file      smtc_hal_lp_timer.h
 *
 * @brief     Low Power Timer Hardware Abstraction Layer definition for Linux
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
#ifndef __SMTC_HAL_LP_TIMER_H__
#define __SMTC_HAL_LP_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Low power timer identifiers
 */
typedef enum hal_lp_timer_id_e
{
    HAL_LP_TIMER_ID_1 = 0,
    HAL_LP_TIMER_ID_2 = 1,
} hal_lp_timer_id_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Low power timer IRQ data context
 */
typedef struct hal_lp_timer_irq_s
{
    void* context;
    void ( *callback )( void* context );
} hal_lp_timer_irq_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initializes the low power timer
 *
 * @param [in] timer_id Timer identifier
 */
void hal_lp_timer_init( const hal_lp_timer_id_t timer_id );

/**
 * @brief Starts the low power timer
 *
 * @param [in] timer_id     Timer identifier
 * @param [in] milliseconds Timer duration in milliseconds
 * @param [in] irq          IRQ data context
 */
void hal_lp_timer_start( const hal_lp_timer_id_t timer_id, const uint32_t milliseconds, const hal_lp_timer_irq_t* irq );

/**
 * @brief Stops the low power timer
 *
 * @param [in] timer_id Timer identifier
 */
void hal_lp_timer_stop( const hal_lp_timer_id_t timer_id );

/**
 * @brief Enable the timer IRQ
 *
 * @param [in] timer_id Timer identifier
 */
void hal_lp_timer_irq_enable( const hal_lp_timer_id_t timer_id );

/**
 * @brief Disable the timer IRQ
 *
 * @param [in] timer_id Timer identifier
 */
void hal_lp_timer_irq_disable( const hal_lp_timer_id_t timer_id );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_LP_TIMER_H__

/* --- EOF ------------------------------------------------------------------ */
