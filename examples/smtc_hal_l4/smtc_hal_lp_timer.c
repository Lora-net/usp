/**
 * @file      smtc_hal_lp_timer.c
 *
 * @brief     Implements Low Power Timer utilities functions.
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_hal_lp_timer.h"
#include "stm32l4xx_hal.h"
#include "smtc_hal_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define HAL_LP_TIMER_NB 2  //!< Number of supported low power timers

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static LPTIM_HandleTypeDef lptim_handle[HAL_LP_TIMER_NB];

static hal_lp_timer_irq_t lptim_tmr_irq[HAL_LP_TIMER_NB] = {
    {
        .context  = NULL,
        .callback = NULL,
    },
#if ( HAL_LP_TIMER_NB > 1 )
    {
        .context  = NULL,
        .callback = NULL,
    },
#endif
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init( hal_lp_timer_id_t id )
{
    lptim_handle[id].Instance             = ( id == 0 ) ? LPTIM1 : LPTIM2;
    lptim_handle[id].Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    lptim_handle[id].Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
    lptim_handle[id].Init.Trigger.Source  = LPTIM_TRIGSOURCE_SOFTWARE;
    lptim_handle[id].Init.OutputPolarity  = LPTIM_OUTPUTPOLARITY_HIGH;
    lptim_handle[id].Init.UpdateMode      = LPTIM_UPDATE_IMMEDIATE;
    lptim_handle[id].Init.CounterSource   = LPTIM_COUNTERSOURCE_INTERNAL;

    if( HAL_LPTIM_Init( &lptim_handle[id] ) != HAL_OK )
    {
        mcu_panic( );
    }
    lptim_tmr_irq[id] = ( hal_lp_timer_irq_t ) { .context = NULL, .callback = NULL };
}

void hal_lp_timer_start( hal_lp_timer_id_t id, const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    uint32_t delay_ms_2_tick = 0;

    // Remark LSE_VALUE / LPTIM_PRESCALER_DIV1
    delay_ms_2_tick = ( uint32_t ) ( ( ( uint64_t ) milliseconds * ( LSE_VALUE ) ) / 1000 );

    // Clamp value if greater than 0xFFFF
    if( delay_ms_2_tick > 0xFFFF )
    {
        delay_ms_2_tick = 0xFFFF;
    }

    lptim_tmr_irq[id] = *tmr_irq;

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE( &lptim_handle[id] );

    /* Clear flag */
    __HAL_LPTIM_CLEAR_FLAG( &lptim_handle[id], LPTIM_FLAG_CMPOK );

    /* Load the Timeout value in the compare register */
    __HAL_LPTIM_COMPARE_SET( &lptim_handle[id], delay_ms_2_tick );

    /* Wait for the completion of the write operation to the LPTIM_CMP register, without this the compare match
     * interrupt will not be triggered */
    while( !( __HAL_LPTIM_GET_FLAG( &lptim_handle[id], LPTIM_FLAG_CMPOK ) ) )
    {
    }

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE( &lptim_handle[id] );

    /* Enable Compare match interrupt */
    __HAL_LPTIM_ENABLE_IT( &lptim_handle[id], LPTIM_IT_CMPM );

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE( &lptim_handle[id] );

    // /* Start the LPTIM peripheral in single mode. */
    __HAL_LPTIM_START_SINGLE( &lptim_handle[id] );
}

void hal_lp_timer_stop( hal_lp_timer_id_t id )
{
    // HAL_LPTIM_TimeOut_Stop_IT( &lptim_handle[id] );
    // hal_lp_timer_irq_disable( id );

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE( &lptim_handle[id] );
}

void hal_lp_timer_irq_enable( hal_lp_timer_id_t id )
{
    HAL_NVIC_EnableIRQ( ( id == 0 ) ? LPTIM1_IRQn : LPTIM2_IRQn );
}

void hal_lp_timer_irq_disable( hal_lp_timer_id_t id )
{
    HAL_NVIC_DisableIRQ( ( id == 0 ) ? LPTIM1_IRQn : LPTIM2_IRQn );
}

void HAL_LPTIM_CompareMatchCallback( LPTIM_HandleTypeDef* hlptim )
{
    if( hlptim->Instance == LPTIM1 )
    {
        if( lptim_tmr_irq[HAL_LP_TIMER_ID_1].callback != NULL )
        {
            lptim_tmr_irq[HAL_LP_TIMER_ID_1].callback( lptim_tmr_irq[HAL_LP_TIMER_ID_1].context );
        }
    }
}
void HAL_LPTIM_AutoReloadMatchCallback( LPTIM_HandleTypeDef* hlptim )
{
    if( hlptim->Instance == LPTIM1 )
    {
        // HAL_LPTIM_Counter_Stop_IT( hlptim );
    }
}
void LPTIM1_IRQHandler( void )
{
    HAL_LPTIM_IRQHandler( &lptim_handle[HAL_LP_TIMER_ID_1] );

    // /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE( &lptim_handle[HAL_LP_TIMER_ID_1] );
    __HAL_LPTIM_DISABLE_IT( &lptim_handle[HAL_LP_TIMER_ID_1], LPTIM_IT_CMPM );
    __HAL_LPTIM_CLEAR_FLAG( &lptim_handle[HAL_LP_TIMER_ID_1], LPTIM_FLAG_CMPM );
}

void LPTIM2_IRQHandler( void )
{
    HAL_LPTIM_IRQHandler( &lptim_handle[HAL_LP_TIMER_ID_2] );
    HAL_LPTIM_TimeOut_Stop( &lptim_handle[HAL_LP_TIMER_ID_2] );

    if( lptim_tmr_irq[HAL_LP_TIMER_ID_2].callback != NULL )
    {
        lptim_tmr_irq[HAL_LP_TIMER_ID_2].callback( lptim_tmr_irq[HAL_LP_TIMER_ID_2].context );
    }
}

void HAL_LPTIM_MspInit( LPTIM_HandleTypeDef* lptimhandle )
{
    if( lptimhandle->Instance == LPTIM1 )
    {
        __HAL_RCC_LPTIM1_CLK_ENABLE( );
        HAL_NVIC_SetPriority( LPTIM1_IRQn, 0, 0 );
        HAL_NVIC_EnableIRQ( LPTIM1_IRQn );
    }
    if( lptimhandle->Instance == LPTIM2 )
    {
        __HAL_RCC_LPTIM2_CLK_ENABLE( );
        HAL_NVIC_SetPriority( LPTIM2_IRQn, 0, 0 );
        HAL_NVIC_EnableIRQ( LPTIM2_IRQn );
    }
}

void HAL_LPTIM_MspDeInit( LPTIM_HandleTypeDef* lptimhandle )
{
    if( lptimhandle->Instance == LPTIM1 )
    {
        __HAL_RCC_LPTIM1_CLK_DISABLE( );
        HAL_NVIC_DisableIRQ( LPTIM1_IRQn );
    }
    if( lptimhandle->Instance == LPTIM2 )
    {
        __HAL_RCC_LPTIM2_CLK_DISABLE( );
        HAL_NVIC_DisableIRQ( LPTIM2_IRQn );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
