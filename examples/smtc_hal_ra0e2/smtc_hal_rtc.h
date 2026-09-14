/**
 * @file      smtc_hal_rtc.h
 *
 * @brief     RTC Hardware Abstraction Layer definition
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

#ifndef __RTC_UTILITIES_H__
#define __RTC_UTILITIES_H__

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 *  Initializes the MCU RTC peripheral
 */
void hal_rtc_init( void );

/*!
 * Returns the current RTC time in seconds
 *
 * \remark Used for scheduling autonomous retransmissions (i.e: NbTrans),
 *         transmitting MAC answers, basically any delay without accurate time
 *         constraints. It is also used to measure the time spent inside the
 *         LoRaWAN process for the integrated failsafe.
 *
 * retval rtc_time_s Current RTC time in seconds
 */
uint32_t hal_rtc_get_time_s( void );

/*!
 * Returns the current RTC time in milliseconds
 *
 * \remark Used to timestamp radio events (i.e: end of TX), will also be used
 * for ClassB
 *
 * retval rtc_time_ms Current RTC time in milliseconds wraps every 49 days
 */
uint32_t hal_rtc_get_time_ms( void );

/*!
 * Initialize the RTC wakeup timer (TML0)
 *
 * \remark Called during MCU init to set up TML0 as dedicated wakeup timer
 */
void hal_rtc_wakeup_timer_init( void );

/*!
 * Sets the rtc wakeup timer for milliseconds parameter. The RTC will generate
 * an IRQ to wakeup the MCU.
 *
 * \param[IN] milliseconds Number of seconds before wakeup
 */
void hal_rtc_wakeup_timer_set_ms( const int32_t milliseconds );

/*!
 * Stop the rtc wakeup timer
 */
void hal_rtc_wakeup_timer_stop( void );

/*!
 * hal to test wrapping (offset-based, doesn't affect actual RTC calendar)
 */
void hal_rtc_set_offset_to_test_wrapping( const uint32_t offset );

/*!
 * Capture systick time at sleep entry.
 *
 * \remark Call this just before entering sleep to record the current
 *         systick_ms_counter value. Used by hal_rtc_resync() to calculate
 *         actual elapsed time after sleep.
 */
void hal_rtc_capture_time_at_sleep_entry( void );

/*!
 * Set systick_ms_counter to an absolute value.
 *
 * \param[in] time_ms The absolute time in milliseconds.
 *
 * \remark Used by LP timer callback to set correct time when timer fires.
 */
void hal_rtc_set_time_ms( uint32_t time_ms );

/*!
 * Resync SysTick with RTC after wake from sleep.
 *
 * \param[in] sleep_duration_ms The requested sleep duration in milliseconds.
 *            This is added to systick_ms_counter to advance time by the
 *            amount we intended to sleep.
 *
 * \remark SysTick stops during sleep (CPU clock stops). Call this after
 *         waking from sleep to advance the millisecond counter by the
 *         intended sleep duration.
 */
void hal_rtc_resync( uint32_t sleep_duration_ms );

/*
 * -----------------------------------------------------------------------------
 * --- RTC ALARM FUNCTIONS (for long sleep > 60 seconds) -----------------------
 */

/*!
 * Set RTC alarm for long sleep operation.
 *
 * \param[in] sleep_ms Total sleep duration in milliseconds (must be >= 60000)
 *
 * \return Milliseconds remaining after alarm minute (unused, kept for compatibility)
 *
 * \remark Enables RTC alarm interrupt in NVIC so WFI can wake on alarm.
 *         The LP timer (TML1) may also wake us earlier for RX windows.
 */
uint32_t hal_rtc_alarm_set_for_sleep( uint32_t sleep_ms );

/*!
 * Stop/disable RTC alarm.
 *
 * \remark Call after alarm fires to prevent re-triggering.
 */
void hal_rtc_alarm_stop( void );

/*!
 * Check if RTC alarm is still pending.
 *
 * \return true if alarm has not yet fired, false if alarm has fired
 */
bool hal_rtc_alarm_is_pending( void );

/*!
 * Sync systick_ms_counter to current RTC calendar time.
 *
 * \remark Call after waking from long sleep (RTC alarm path) to ensure
 *         systick_ms_counter reflects the current RTC time.
 *         Resolution is seconds only (milliseconds will be 0).
 */
void hal_rtc_sync_systick_to_calendar( void );

#ifdef __cplusplus
}
#endif

#endif  // __RTC_UTILITIES_H__

/* --- EOF ------------------------------------------------------------------ */
