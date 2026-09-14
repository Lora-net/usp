/**
 * @file      smtc_hal_rtc.h
 *
 * @brief     RTC Hardware Abstraction Layer definition for Linux
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
#ifndef __SMTC_HAL_RTC_H__
#define __SMTC_HAL_RTC_H__

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

/**
 * @brief Initializes the MCU RTC peripheral
 */
void hal_rtc_init( void );

/**
 * @brief Returns the current RTC time in seconds
 *
 * @return uint32_t Current RTC time in seconds
 */
uint32_t hal_rtc_get_time_s( void );

/**
 * @brief Returns the current RTC time in milliseconds
 *
 * @return uint32_t Current RTC time in milliseconds
 */
uint32_t hal_rtc_get_time_ms( void );

/**
 * @brief Set offset for RTC time (for testing wrapping)
 *
 * @param offset_to_test_wrapping Offset value
 */
void hal_rtc_set_offset_to_test_wrapping( const uint32_t offset_to_test_wrapping );

/**
 * @brief Get the current time acceleration factor
 *
 * @return uint32_t Current time acceleration factor (1 = real-time)
 */
uint32_t hal_rtc_get_time_acceleration( void );

/**
 * @brief Set the time acceleration factor
 *
 * @remark Used for testing to speed up time-dependent operations.
 *         Can also be set via LBM_TIME_ACCELERATION environment variable.
 *         Example: LBM_TIME_ACCELERATION=10 makes time run 10x faster.
 *
 * @param factor Time acceleration factor (minimum 1)
 */
void hal_rtc_set_time_acceleration( uint32_t factor );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_RTC_H__

/* --- EOF ------------------------------------------------------------------ */
