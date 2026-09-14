/**
 * @file      smtc_hal_rtc.c
 *
 * @brief     RTC HAL for RA0E2
 *
 * Architecture:
 * - RTC_C calendar provides seconds (SEC, MIN, HOUR, DAY, MONTH, YEAR)
 * - SysTick provides sub-second milliseconds (1ms interrupt)
 * - SysTick runs from CPU clock (32 MHz), interrupts every 1ms
 *
 * Synchronization:
 * - At init: Wait for RTC second boundary, then start SysTick aligned
 * - After sleep: Call hal_rtc_resync() to realign SysTick with RTC
 * - SysTick stops during sleep (CPU clock stops), so resync is needed
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "smtc_hal_rtc.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_lp_timer.h"
#include "hal_data.h"
#include "bsp_api.h"
#include "vector_data.h" /* For RTC_ALARM_OR_PERIOD_IRQn */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

/* RTC_C year base (RTC counts years 0-99, representing 2000-2099) */
#define RTC_YEAR_BASE 2000U

/* SysTick configuration */
#define SYSTICK_FREQ_HZ 1000U /* 1ms tick */

/* Days per month (non-leap year) */
static const uint8_t days_per_month[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/* SysTick millisecond counter - incremented every 1ms by SysTick ISR */
static volatile uint32_t systick_ms_counter = 0;

/* Offset for testing wrapping behavior */
static uint32_t rtc_offset_s = 0;

/* Systick value captured at sleep entry for resync calculation */
static volatile uint32_t systick_at_sleep_entry = 0;

/* Track if RTC is initialized */
static bool rtc_initialized = false;

/* Track if RTC alarm is pending (for long sleep) */
static volatile bool rtc_alarm_pending = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static uint32_t rtc_calendar_to_seconds( void );
static uint8_t  rtc_get_raw_seconds( void );
static bool     is_leap_year( uint16_t year );
static void     systick_start( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS --------------------------------------------------------
 */

void hal_rtc_init( void )
{
    fsp_err_t  err;
    rtc_time_t initial_time;
    uint8_t    start_sec;

    if( rtc_initialized )
    {
        return;
    }

    /* Configure SysTick for 1ms interrupts (but don't start yet) */
    /* SysTick runs from CPU clock (SystemCoreClock) */
    /* LOAD = (clock / tick_freq) - 1 = (32000000 / 1000) - 1 = 31999 */
    SysTick->LOAD = ( SystemCoreClock / SYSTICK_FREQ_HZ ) - 1U;
    SysTick->VAL  = 0U; /* Clear current value */
    SysTick->CTRL = 0U; /* Disabled for now */

    /* Set SysTick interrupt priority (lower than radio IRQ) */
    NVIC_SetPriority( SysTick_IRQn, 3 );

    /* Initialize RTC_C */
    err = R_RTC_C_Open( &g_rtc_ctrl, &g_rtc_cfg );
    if( FSP_SUCCESS != err )
    {
        mcu_panic( );
    }

    /* Set initial time to 2000-01-01 00:00:00 (epoch for our purposes) */
    initial_time.tm_sec  = 0;
    initial_time.tm_min  = 0;
    initial_time.tm_hour = 0;
    initial_time.tm_mday = 1;
    initial_time.tm_mon  = 0;   /* January (0-based in FSP) */
    initial_time.tm_year = 100; /* Years since 1900, so 100 = year 2000 */
    initial_time.tm_wday = 6;   /* Saturday (2000-01-01 was a Saturday) */

    err = R_RTC_C_CalendarTimeSet( &g_rtc_ctrl, &initial_time );
    if( FSP_SUCCESS != err )
    {
        mcu_panic( );
    }

    rtc_initialized = true;

    /* Synchronize SysTick with RTC second boundary */
    /* Wait for RTC seconds to change, then start SysTick at ms=0 */
    start_sec = rtc_get_raw_seconds( );
    while( rtc_get_raw_seconds( ) == start_sec )
    {
        /* Spin until second changes - max ~1 second wait */
    }

    /* Now at exact second boundary - start SysTick aligned with RTC time */
    systick_ms_counter = ( rtc_calendar_to_seconds( ) + rtc_offset_s ) * 1000U;
    systick_start( );
}

uint32_t hal_rtc_get_time_s( void )
{
    return rtc_calendar_to_seconds( ) + rtc_offset_s;
}

uint32_t hal_rtc_get_time_ms( void )
{
    /* Use systick_ms_counter directly.
     *
     * Design choice: systick_ms_counter is the authoritative millisecond time source.
     * It's updated by:
     *   - SysTick ISR: increments every 1ms when CPU is running
     *   - hal_rtc_resync(): aligned to RTC seconds after sleep
     *
     * This approach ensures elapsed time measurements are consistent:
     *   - Before/after operations within same wake period: accurate to 1ms
     *   - Across sleep: aligned to RTC second boundaries (up to ~1s granularity)
     *
     * For LoRaWAN timing (LP timer callbacks), the actual delay is timed by the
     * hardware LP timer at 1024 Hz. This function just provides timestamps for
     * scheduling decisions, which are tolerant of second-level alignment.
     */
    return systick_ms_counter;
}

/* TML0 clock: SOSC 32.768 kHz / 32 = 1024 Hz (same as TML1) */
#define TML0_CLOCK_HZ 1024U

/* TML0 FDIV value for /32 divider */
#define TML0_FDIV_DIV32 5U

/* Track if TML0 wakeup timer is running */
static volatile bool wakeup_timer_running = false;

/* Track if TML0 is initialized */
static bool tml0_initialized = false;

void hal_rtc_wakeup_timer_init( void )
{
    fsp_err_t err;

    if( tml0_initialized )
    {
        return;
    }

    /* Set clock divider to /32 BEFORE opening TML */
    /* In 16-bit mode, channels 0+1 are used for TML0, so set FDIV0 and FDIV1 */
    R_TML->ITLFDIV00 = ( TML0_FDIV_DIV32 << 4 ) | TML0_FDIV_DIV32; /* FDIV1=5, FDIV0=5 */

    err = R_TML_Open( &g_timer_tml0_ctrl, &g_timer_tml0_cfg );
    if( FSP_SUCCESS != err )
    {
        mcu_panic( );
    }

    /* TML0 and TML1 share the same IRQ (TML0_ITL_OR), already configured by LP timer init */

    tml0_initialized = true;
}

void hal_rtc_wakeup_timer_set_ms( const int32_t milliseconds )
{
    /* Use TML0 for dedicated wakeup timing.
     * TML0 runs on SOSC (32.768 kHz / 32 = 1024 Hz) and continues during sleep.
     * Max delay is ~64 seconds, which is sufficient for LoRaWAN RX windows.
     *
     * TML0 is dedicated to wakeup - it doesn't interfere with TML1 (LP timer/modem).
     */
    fsp_err_t err;
    uint32_t  delay_ticks;

    if( milliseconds <= 0 )
    {
        return;
    }

    /* Convert milliseconds to ticks: ticks = ms * 1024 / 1000
     * Use simple rounding - wakeup timer doesn't need to be super precise.
     */
    delay_ticks = ( ( milliseconds * TML0_CLOCK_HZ ) + 500U ) / 1000U;

    /* Clamp to 16-bit max if exceeds ~64 seconds */
    if( delay_ticks > 0xFFFFU )
    {
        delay_ticks = 0xFFFFU;
    }

    /* Ensure at least 1 tick */
    if( delay_ticks == 0 )
    {
        delay_ticks = 1;
    }

    /* Stop timer if running */
    R_TML_Stop( &g_timer_tml0_ctrl );

    /* Set the period (compare value) */
    /* In 16-bit mode for channel 0, we write to ITLCMP00 */
    R_TML->ITLCMP00 = ( uint16_t ) ( delay_ticks - 1 );

    /* Clear any pending interrupt flag for channel 0 */
    R_TML->ITLS0 &= ~( R_TML_ITLS0_ITF00_Msk );

    /* Enable compare match interrupt */
    err = R_TML_Enable( &g_timer_tml0_ctrl );
    if( FSP_SUCCESS != err )
    {
        /* Non-fatal, continue anyway */
    }

    /* Start the timer */
    err = R_TML_Start( &g_timer_tml0_ctrl );
    if( FSP_SUCCESS != err )
    {
        mcu_panic( );
    }

    wakeup_timer_running = true;
}

void hal_rtc_wakeup_timer_stop( void )
{
    if( wakeup_timer_running )
    {
        R_TML_Stop( &g_timer_tml0_ctrl );
        wakeup_timer_running = false;
    }
}

void hal_rtc_set_offset_to_test_wrapping( uint32_t offset )
{
    rtc_offset_s = offset;
}

void hal_rtc_capture_time_at_sleep_entry( void )
{
    systick_at_sleep_entry = systick_ms_counter;
}

void hal_rtc_set_time_ms( uint32_t time_ms )
{
    /* Set systick_ms_counter to an absolute value.
     * Used by LP timer callback to set correct time when timer fires.
     */
    systick_ms_counter = time_ms;
}

void hal_rtc_resync( uint32_t elapsed_ms )
{
    /* Called after wake from sleep to set SysTick to the correct time.
     *
     * Problem: SysTick stops during sleep but systick_ms_counter retains its value.
     * After waking, we need to update systick_ms_counter to reflect elapsed time.
     *
     * CRITICAL for LoRaWAN RX timing:
     * - Tasks are scheduled at specific millisecond times (e.g., RX1 at 14990ms)
     * - Radio planner ABORTS tasks if current_time > task_time (task "in past")
     * - If we overshoot, we risk aborting valid tasks
     *
     * Strategy: Set systick_ms_counter = time_at_sleep_entry + elapsed_ms
     *
     * This function may be called from:
     * 1. LP timer callback: elapsed_ms = LP timer delay (accurate)
     * 2. hal_mcu_set_sleep_for_ms: elapsed_ms = wakeup timer delay
     *
     * The LP timer callback runs first (before hal_mcu_set_sleep_for_ms continues),
     * so if LP timer woke us, it sets the correct time. Then when
     * hal_mcu_set_sleep_for_ms calls us again, we recalculate, which is harmless
     * since systick_at_sleep_entry stays the same.
     *
     * Note: If we were woken by something other than LP timer or wakeup timer
     * (e.g., radio IRQ), the caller's elapsed_ms may be wrong, but this is rare.
     */
    systick_ms_counter = systick_at_sleep_entry + elapsed_ms;

    /* Clear SysTick hardware counter to start fresh 1ms ticks */
    SysTick->VAL = 0U;
}

/*
 * -----------------------------------------------------------------------------
 * --- RTC ALARM FUNCTIONS (for long sleep > 60 seconds) -----------------------
 */

uint32_t hal_rtc_alarm_set_for_sleep( uint32_t sleep_ms )
{
    rtc_time_t       current_time;
    rtc_alarm_time_t alarm;
    uint32_t         current_sec_of_day;
    uint32_t         sleep_sec;
    uint32_t         sleep_ms_fraction;
    uint32_t         sleep_days;
    uint32_t         sleep_sec_in_day;
    uint32_t         target_sec_of_day;
    uint8_t          target_hour;
    uint8_t          target_min;
    uint8_t          target_sec;
    uint8_t          target_wday;
    uint32_t         remainder_ms;

    /* Get current RTC time */
    R_RTC_C_CalendarTimeGet( &g_rtc_ctrl, &current_time );

    /* Calculate current seconds since midnight */
    current_sec_of_day = ( uint32_t ) current_time.tm_hour * 3600U + ( uint32_t ) current_time.tm_min * 60U +
                         ( uint32_t ) current_time.tm_sec;

    /* Calculate sleep duration in seconds and milliseconds */
    sleep_sec         = sleep_ms / 1000U;
    sleep_ms_fraction = sleep_ms % 1000U;

    /* Split sleep into days and seconds-within-day */
    sleep_days       = sleep_sec / 86400U;
    sleep_sec_in_day = sleep_sec % 86400U;

    /* Calculate target time-of-day (seconds since midnight) */
    target_sec_of_day = current_sec_of_day + sleep_sec_in_day;

    /* Handle day rollover from time-of-day overflow */
    if( target_sec_of_day >= 86400U )
    {
        target_sec_of_day -= 86400U;
        sleep_days++;
    }

    /* Extract target hour, minute, second */
    target_hour = ( uint8_t ) ( target_sec_of_day / 3600U );
    target_min  = ( uint8_t ) ( ( target_sec_of_day % 3600U ) / 60U );
    target_sec  = ( uint8_t ) ( target_sec_of_day % 60U );

    /* Determine weekday matching strategy.
     *
     * RA0E2 RTC_C alarm only supports hour:minute:weekday matching (no day-of-month).
     * With weekday_match = 0x7F (all days), alarm fires at next matching hour:minute.
     *
     * PROBLEM: If sleep is exactly N*24 hours, target time-of-day equals current time,
     * and alarm would fire immediately instead of N days later.
     *
     * SOLUTION: Use single weekday bit to target specific day (up to 7 days ahead).
     * For sleeps > 7 days, we target 7 days ahead and rely on caller to re-request.
     *
     * tm_wday: 0=Sunday, 1=Monday, ..., 6=Saturday (standard C library convention)
     * weekday_match bits: bit0=Sunday, bit1=Monday, ..., bit6=Saturday
     */
    if( sleep_days > 0 )
    {
        /* Sleep crosses at least one day boundary - use specific weekday targeting */
        /* Clamp to 7 days max (weekday wraps after 7) */
        if( sleep_days > 7 )
        {
            sleep_days = 7;
        }
        target_wday         = ( uint8_t ) ( ( current_time.tm_wday + sleep_days ) % 7 );
        alarm.weekday_match = ( uint8_t ) ( 1U << target_wday );
    }
    else
    {
        /* Same day - use all-days match for next occurrence of target time */
        alarm.weekday_match = 0x7FU;
    }

    alarm.time_hour   = ( int ) target_hour;
    alarm.time_minute = ( int ) target_min;

    /* Mark alarm as pending before setting */
    rtc_alarm_pending = true;

    /* Set the alarm - this also sets WALE (alarm interrupt enable) */
    R_RTC_C_CalendarAlarmSet( &g_rtc_ctrl, &alarm );

    /* Clear any pending NVIC interrupt from previous alarm before enabling.
     * This prevents WFI from returning immediately due to stale pending IRQ.
     */
    NVIC_ClearPendingIRQ( RTC_ALARM_OR_PERIOD_IRQn );

    /* Enable alarm interrupt in NVIC so WFI can wake on alarm.
     * FSP has alarm_irq = FSP_INVALID_VECTOR, so we manually enable it.
     */
    NVIC_EnableIRQ( RTC_ALARM_OR_PERIOD_IRQn );

    /* Calculate remainder for TML0: seconds past the alarm minute + ms fraction
     * This is always < 60000ms (max 59999ms)
     */
    remainder_ms = ( uint32_t ) target_sec * 1000U + sleep_ms_fraction;

    return remainder_ms;
}

void hal_rtc_alarm_stop( void )
{
    /* Disable alarm interrupt in NVIC */
    NVIC_DisableIRQ( RTC_ALARM_OR_PERIOD_IRQn );

    /* Disable alarm by clearing WALE bit in RTCC1 register */
    R_RTC_C->RTCC1 &= ( uint8_t ) ~R_RTC_C_RTCC1_WALE_Msk;

    /* Also clear alarm flag if set */
    R_RTC_C->RTCC1 &= ( uint8_t ) ~R_RTC_C_RTCC1_WAFG_Msk;

    rtc_alarm_pending = false;
}

bool hal_rtc_alarm_is_pending( void )
{
    /* Check hardware alarm flag (WAFG) directly since alarm IRQ is not configured in FSP.
     * WAFG is set by hardware when alarm matches, cleared by software.
     * This polling approach works because we're in a WFI loop anyway.
     */
    if( R_RTC_C->RTCC1 & R_RTC_C_RTCC1_WAFG_Msk )
    {
        /* Alarm has fired - clear the flag and update pending state */
        rtc_alarm_pending = false;
    }

    return rtc_alarm_pending;
}

void hal_rtc_sync_systick_to_calendar( void )
{
    /* Sync systick_ms_counter to the current RTC calendar time.
     *
     * This is needed after long sleep (RTC alarm path) because:
     * - SysTick stops during sleep (CPU clock stops)
     * - LP timer callback may not have fired (if RTC alarm woke us)
     * - systick_ms_counter would be stale without this sync
     *
     * Resolution is seconds only - RTC doesn't provide sub-second time.
     * This is acceptable for long sleep scenarios where we're sleeping
     * for minutes at a time anyway.
     */
    systick_ms_counter = ( rtc_calendar_to_seconds( ) + rtc_offset_s ) * 1000U;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief Convert RTC calendar to seconds since epoch (2000-01-01 00:00:00)
 *
 * WARNING: This uses FSP API which has blocking waits. NOT ISR-safe.
 * Do not call from interrupt context.
 */
static uint32_t rtc_calendar_to_seconds( void )
{
    rtc_time_t time;
    uint32_t   total_seconds = 0;
    uint16_t   year;
    uint8_t    month;

    /* Get current time from RTC */
    R_RTC_C_CalendarTimeGet( &g_rtc_ctrl, &time );

    /* Convert tm_year (years since 1900) to actual year */
    year = ( uint16_t ) ( 1900 + time.tm_year );

    /* tm_mon is 0-based (0=January) */
    month = ( uint8_t ) ( time.tm_mon + 1 );

    /* Count seconds from years (since 2000) */
    for( uint16_t y = RTC_YEAR_BASE; y < year; y++ )
    {
        total_seconds += is_leap_year( y ) ? ( 366U * 86400U ) : ( 365U * 86400U );
    }

    /* Count seconds from months */
    for( uint8_t m = 0; m < ( month - 1 ); m++ )
    {
        uint32_t days = days_per_month[m];
        if( m == 1 && is_leap_year( year ) )
        {
            days = 29; /* February in leap year */
        }
        total_seconds += days * 86400U;
    }

    /* Add days (tm_mday is 1-based) */
    total_seconds += ( uint32_t ) ( time.tm_mday - 1 ) * 86400U;

    /* Add hours, minutes, seconds */
    total_seconds += ( uint32_t ) time.tm_hour * 3600U;
    total_seconds += ( uint32_t ) time.tm_min * 60U;
    total_seconds += ( uint32_t ) time.tm_sec;

    return total_seconds;
}

/**
 * @brief Check if a year is a leap year
 */
static bool is_leap_year( uint16_t year )
{
    return ( ( year % 4 == 0 ) && ( year % 100 != 0 ) ) || ( year % 400 == 0 );
}

/**
 * @brief Get raw RTC seconds value (0-59) without full calendar conversion
 *
 * Fast function for synchronization - only reads SEC register.
 */
static uint8_t rtc_get_raw_seconds( void )
{
    rtc_time_t time;
    R_RTC_C_CalendarTimeGet( &g_rtc_ctrl, &time );
    return ( uint8_t ) time.tm_sec;
}

/**
 * @brief Start SysTick timer
 */
static void systick_start( void )
{
    SysTick->VAL  = 0U;                          /* Clear current value */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | /* Use processor clock */
                    SysTick_CTRL_TICKINT_Msk |   /* Enable interrupt */
                    SysTick_CTRL_ENABLE_Msk;     /* Enable SysTick */
}

/*
 * -----------------------------------------------------------------------------
 * --- INTERRUPT HANDLERS ------------------------------------------------------
 */

/**
 * @brief SysTick interrupt handler - increments ms counter every 1ms
 */
void SysTick_Handler( void )
{
    systick_ms_counter++;
}

/* RTC callback - handles alarm interrupt for long sleep */
void rtc_callback( rtc_callback_args_t* p_args )
{
    if( p_args->event == RTC_EVENT_ALARM_IRQ )
    {
        /* Alarm fired - signal wakeup complete */
        rtc_alarm_pending = false;
    }
    /* RTC_EVENT_PERIODIC_IRQ not used */
}

/**
 * @brief TML0 wakeup callback - called when TML0 compare match occurs
 *
 * This is the callback registered with FSP in hal_data.c for TML0.
 * TML0 is used as a dedicated wakeup timer, separate from TML1 (LP timer/modem).
 */
void tml0_wakeup_callback( timer_callback_args_t* p_args )
{
    ( void ) p_args;

    /* Stop the timer */
    R_TML_Stop( &g_timer_tml0_ctrl );
    wakeup_timer_running = false;

    /* No callback needed - wakeup timer's only purpose is to wake from sleep */
}

/* --- EOF ------------------------------------------------------------------ */
