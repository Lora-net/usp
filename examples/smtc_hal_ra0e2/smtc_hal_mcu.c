/**
 * @file      smtc_hal_mcu.c
 *
 * @brief     MCU Hardware Abstraction Layer implementation for RA0E2
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

#include "smtc_hal_mcu.h"
#include "smtc_hal_uart.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_tau.h"
#include "smtc_hal_rtc.h"
#include "smtc_hal_lp_timer.h"
#include "smtc_hal_watchdog.h"
#include "smtc_hal_irq.h"
#include "modem_pinout.h"
#include "bsp_api.h"
#include "hal_data.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* Low power mode: 1 = software standby, 0 = WFI only */
#ifndef LOW_POWER_MODE
#define LOW_POWER_MODE 1
#endif

/* Debug probe: 1 = keep peripherals initialized for debug, 0 = full de-init */
#ifndef HW_DEBUG_PROBE
#define HW_DEBUG_PROBE 0
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/* Flag to allow/disallow low power mode at runtime */
static volatile bool lp_mode_enabled = true;

/* Flag to track if LPM driver is initialized */
static volatile bool lpm_initialized = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void lpm_enter_sleep_mode( void );
static void lpm_exit_sleep_mode( void );

#if( LOW_POWER_MODE == 1 )
static void lpm_mcu_deinit( void );
static void lpm_mcu_reinit( void );
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_mcu_critical_section_begin( uint32_t* mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void hal_mcu_critical_section_end( uint32_t* mask )
{
    __set_PRIMASK( *mask );
}

void hal_mcu_disable_irq( void )
{
    __disable_irq( );
}

void hal_mcu_enable_irq( void )
{
    __enable_irq( );
}

void hal_mcu_init( void )
{
#if HAL_DBG_TRACE == HAL_FEATURE_ON
    trace_uart_init( );
#else
    /* When trace is disabled, reconfigure UART pins to low-power GPIO.
     * FSP pin_data.c sets P100/P101 as UART peripheral pins at startup,
     * but if UART is not opened, these pins float and cause leakage.
     * Config matches trace_uart_deinit() for consistency.
     * P100/P101 are VCOM pins - use input to avoid fighting J-Link. */
    R_BSP_PinAccessEnable( );
    R_BSP_PinCfg( BSP_IO_PORT_01_PIN_01, IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE );
    R_BSP_PinCfg( BSP_IO_PORT_01_PIN_00, IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE );
    R_BSP_PinAccessDisable( );
#endif

    hal_tau_init( );
    hal_rtc_init( );
    hal_rtc_wakeup_timer_init( );           /* TML0 - dedicated wakeup timer */
    hal_lp_timer_init( HAL_LP_TIMER_ID_1 ); /* TML1 - modem LP timer */
    hal_spi_init( RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );
    hal_watchdog_init( );

#if( LOW_POWER_MODE == 1 )
    /* Initialize LPM driver for software standby mode */
    fsp_err_t err = R_LPM_Open( &g_lpm0_ctrl, &g_lpm0_cfg );
    if( err == FSP_SUCCESS )
    {
        lpm_initialized = true;
    }
#endif
}

void hal_mcu_reset( void )
{
    NVIC_SystemReset( );
}

void hal_mcu_wait_us( const int32_t microseconds )
{
    if( microseconds <= 0 )
    {
        return;
    }

    uint32_t remaining = ( uint32_t ) microseconds;

    /* Chain multiple TAU delays for values exceeding single delay max */
    while( remaining > 0 )
    {
        uint32_t delay_chunk = ( remaining > HAL_TAU_MAX_DELAY_US ) ? HAL_TAU_MAX_DELAY_US : remaining;
        hal_tau_delay_us( delay_chunk );
        remaining -= delay_chunk;
    }
}

void hal_mcu_set_sleep_for_ms( const int32_t milliseconds )
{
    if( milliseconds <= 0 )
    {
        return;
    }

    if( milliseconds < 60000 )
    {
        /* ============================================================
         * SHORT SLEEP (< 60 seconds): Use TML0 only
         * This is the EXISTING code path - DO NOT MODIFY
         * ============================================================ */

        /* TML0 is a dedicated wakeup timer (separate from TML1 which is used by modem).
         * This simplifies the architecture - no need to coordinate with LP timer.
         */
        hal_rtc_wakeup_timer_set_ms( milliseconds );

        if( !lp_mode_enabled )
        {
            /* Low power mode disabled - just wait for interrupt.
             * Must still disable SysTick or its 1ms interrupt will wake CPU immediately.
             */
            SysTick->CTRL = 0U;
            __DSB( );
            __WFI( );
            SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
            hal_rtc_wakeup_timer_stop( );
            return;
        }

        /* Capture systick time at sleep entry for LP timer resync calculation */
        hal_rtc_capture_time_at_sleep_entry( );

        /* Stop SysTick before sleep - it uses CPU clock which stops in sleep */
        SysTick->CTRL = 0U;

        /* Clear pending interrupts EXCEPT wakeup sources.
         * This is CRITICAL for RA0E2 - prevents immediate wakeup from spurious IRQs.
         * We preserve TML0 (IRQ 33) for timer wakeup and Radio DIO (IRQ 7) for TX done.
         */
        hal_irq_clear_all_pending_except( true, false, true ); /* preserve_tml0=true, preserve_radio=true */

        /* Enter sleep mode (WFI or software standby depending on LOW_POWER_MODE)
         * TML0 (wakeup) and TML1 (LP timer) run on SOSC (32.768 kHz) and will wake us up
         */
        lpm_enter_sleep_mode( );
        lpm_exit_sleep_mode( );

        /* Woken up by TML0, TML1, or other interrupt */

        /* Stop wakeup timer */
        hal_rtc_wakeup_timer_stop( );

        /* Advance systick_ms_counter, but ONLY if LP timer didn't already set it.
         *
         * If TML1 (LP timer) woke us early for an RX window, its callback already
         * set systick_ms_counter to the correct absolute time via hal_rtc_set_time_ms().
         * Calling hal_rtc_resync() here would OVERWRITE that correct time with
         * (systick_at_sleep_entry + requested_sleep_duration), which is WRONG when
         * we woke early.
         *
         * Example of the bug this fixes:
         * - Modem requests 2000ms sleep, LP timer set for RX1 at 1000ms
         * - LP timer fires at 1000ms, sets systick = 11000ms (correct)
         * - hal_rtc_resync(2000) would set systick = 10000 + 2000 = 12000ms (WRONG!)
         * - Modem sees time as 1000ms ahead, schedules RX2 "in the past" -> MISSED
         *
         * hal_lp_timer_did_update_systick() returns true (and clears flag) if LP timer
         * callback updated systick. If true, we skip resync to preserve correct time.
         */
        if( !hal_lp_timer_did_update_systick( HAL_LP_TIMER_ID_1 ) )
        {
            hal_rtc_resync( ( uint32_t ) milliseconds );
        }

        /* Restart SysTick for ms timing */
        SysTick->VAL  = 0U;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    }
    else
    {
        /* ============================================================
         * LONG SLEEP (>= 60 seconds): Use RTC alarm as wakeup source
         * The modem's LP timer (TML1) will also wake us for RX windows.
         * We do a single WFI and return - modem recalculates sleep time.
         * ============================================================ */

        /* Set RTC alarm for target time (enables NVIC interrupt for wakeup) */
        ( void ) hal_rtc_alarm_set_for_sleep( ( uint32_t ) milliseconds );

        if( !lp_mode_enabled )
        {
            /* Low power mode disabled - just wait for interrupt.
             * Must still disable SysTick or its 1ms interrupt will wake CPU immediately.
             */
            SysTick->CTRL = 0U;
            __DSB( );
            __WFI( );
            SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
            hal_rtc_alarm_stop( );
            return;
        }

        /* NOTE: For long sleeps, we do NOT capture systick or resync ourselves.
         * The LP timer callback (TML1) handles timing via hal_rtc_set_time_ms().
         * We just need to stop SysTick during sleep and restart it after.
         */

        /* Stop SysTick before sleep - it uses CPU clock which stops in sleep */
        SysTick->CTRL = 0U;

        /* Clear pending interrupts EXCEPT wakeup sources.
         * This is CRITICAL for RA0E2 - prevents immediate wakeup from spurious IRQs.
         * We preserve TML0 (IRQ 33), RTC alarm (IRQ 32), and Radio DIO (IRQ 7).
         */
        hal_irq_clear_all_pending_except( true, true, true ); /* preserve all wakeup sources */

        /* Enter sleep mode (WFI or software standby depending on LOW_POWER_MODE) */
        lpm_enter_sleep_mode( );
        lpm_exit_sleep_mode( );

        /* Woken up - could be RTC alarm, TML1 (LP timer), radio IRQ, or other.
         * Stop the RTC alarm regardless - modem will request new sleep if needed.
         */
        hal_rtc_alarm_stop( );

        /* Sync systick_ms_counter to current RTC calendar time, but ONLY if
         * the LP timer (TML1) callback did NOT already set it.
         *
         * If TML1 callback fired:
         * - It already set systick to ms-accurate time via hal_rtc_set_time_ms()
         * - Overwriting with calendar time (second-level) would break RX timing
         * - hal_lp_timer_did_update_systick() returns true and clears the flag
         *
         * If RTC alarm or other IRQ woke us (or LP timer was never started):
         * - systick_ms_counter is stale from before sleep
         * - Need calendar sync for correct scheduler operation
         */
        if( !hal_lp_timer_did_update_systick( HAL_LP_TIMER_ID_1 ) )
        {
            /* LP timer didn't update systick - sync to calendar */
            hal_rtc_sync_systick_to_calendar( );
        }

        /* Restart SysTick for ms timing */
        SysTick->VAL  = 0U;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    }
}

void hal_mcu_disable_low_power_wait( void )
{
    lp_mode_enabled = false;
}

void hal_mcu_enable_low_power_wait( void )
{
    lp_mode_enabled = true;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief Either enters Low Power Standby Mode or calls WFI
 *
 * @note ARM exits the function when waking up
 */
static void lpm_enter_sleep_mode( void )
{
#if( LOW_POWER_MODE == 1 )
    /* De-init peripherals before entering standby */
    lpm_mcu_deinit( );

    /* Enter software standby mode using FSP LPM driver.
     * R_LPM_LowPowerModeEnter() sets up SBYCR register and calls WFI internally.
     * Wakeup sources configured in g_lpm0_cfg: IRQ0 (button), IRQ5 (radio),
     * RTC (alarm), ITL (TML interval timer).
     */
    if( lpm_initialized )
    {
        R_LPM_LowPowerModeEnter( &g_lpm0_ctrl );
    }
    else
    {
        /* Fallback to WFI if LPM driver not initialized */
        __WFI( );
    }
#else
    /* LOW_POWER_MODE disabled - just WFI */
    __WFI( );
#endif
}

/**
 * @brief Wakes from Low Power Standby Mode or from WFI
 */
static void lpm_exit_sleep_mode( void )
{
#if( LOW_POWER_MODE == 1 )
    /* Re-initialize peripherals after waking from standby */
    lpm_mcu_reinit( );
#endif
}

#if( LOW_POWER_MODE == 1 )

/**
 * @brief De-init peripherals before going in sleep mode
 *
 * @note RA0E2 automatically retains GPIO states in software standby
 *       (no SBYCR.OPE bit), so radio NSS/NRST stay configured.
 */
static void lpm_mcu_deinit( void )
{
#if( HW_DEBUG_PROBE == 0 )
    /* De-init SPI - SAU peripheral will be re-initialized on wakeup */
    hal_spi_de_init( RADIO_SPI_ID );

#if HAL_DBG_TRACE == HAL_FEATURE_ON
    /* De-init UART for trace output */
    trace_uart_deinit( );
#endif
#endif /* HW_DEBUG_PROBE == 0 */
}

/**
 * @brief Re-init MCU peripherals after waking from standby mode
 *
 * @note HOCO auto-restarts on wakeup (SBYCR.FWKUP=1 in FSP config).
 *       GPIO states are retained automatically by RA0E2.
 */
static void lpm_mcu_reinit( void )
{
#if( HW_DEBUG_PROBE == 0 )
#if HAL_DBG_TRACE == HAL_FEATURE_ON
    /* Re-init UART for trace output */
    trace_uart_init( );
#endif

    /* Re-init SPI for radio communication */
    hal_spi_init( RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );
#endif /* HW_DEBUG_PROBE == 0 */
}

#endif /* LOW_POWER_MODE == 1 */

/* --- EOF ------------------------------------------------------------------ */
