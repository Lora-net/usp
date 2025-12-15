/**
 * @file      smtc_rac_log.h
 *
 * @brief     Unified logging library for RAC examples
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

#ifndef RAC_LOG_H
#define RAC_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <inttypes.h>

#include <stdio.h>

#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- LOG LEVEL CONFIGURATION ------------------------------------------------
 */

// Default configuration - can be overridden by CMake defines
#ifndef RAC_LOG_ERROR_ENABLE
#define RAC_LOG_ERROR_ENABLE 1
#endif

#ifndef RAC_LOG_WARN_ENABLE
#define RAC_LOG_WARN_ENABLE 1
#endif

#ifndef RAC_LOG_INFO_ENABLE
#define RAC_LOG_INFO_ENABLE 1
#endif

#ifndef RAC_LOG_DEBUG_ENABLE
#define RAC_LOG_DEBUG_ENABLE 1
#endif

#ifndef RAC_LOG_CONFIG_ENABLE
#define RAC_LOG_CONFIG_ENABLE 1
#endif

#ifndef RAC_LOG_TX_ENABLE
#define RAC_LOG_TX_ENABLE 1
#endif

#ifndef RAC_LOG_RX_ENABLE
#define RAC_LOG_RX_ENABLE 1
#endif

#ifndef RAC_LOG_STATS_ENABLE
#define RAC_LOG_STATS_ENABLE 1
#endif

/*
 * -----------------------------------------------------------------------------
 * --- APPLICATION PREFIX CONFIGURATION ---------------------------------------
 */

// Default application prefix - should be overridden by each example
#ifndef RAC_LOG_APP_PREFIX
#define RAC_LOG_APP_PREFIX "RAC"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

// Internal timestamp formatting
#define RAC_LOG_TIMESTAMP( ) smtc_modem_hal_get_time_in_ms( )

// Internal formatting helper
#define RAC_LOG_FORMAT( level, ... )                                                                 \
    do                                                                                               \
    {                                                                                                \
        char _rac_log_buf[220];                                                                      \
        int  _offset = snprintf( _rac_log_buf, sizeof( _rac_log_buf ), "[%s-%s] [%8" PRIu32 " ms] ", \
                                 RAC_LOG_APP_PREFIX, level, RAC_LOG_TIMESTAMP( ) );                  \
        snprintf( _rac_log_buf + _offset, sizeof( _rac_log_buf ) - _offset, __VA_ARGS__ );           \
        SMTC_MODEM_HAL_TRACE_PRINTF( "%s\n", _rac_log_buf );                                         \
    } while( 0 )
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

// ERROR level - critical errors, always enabled by default
#if RAC_LOG_ERROR_ENABLE
#define RAC_LOG_ERROR( ... )                    \
    do                                          \
    {                                           \
        RAC_LOG_FORMAT( "ERROR", __VA_ARGS__ ); \
    } while( 0 )
#else
#define RAC_LOG_ERROR( ... ) \
    do                       \
    {                        \
    } while( 0 )
#endif

// WARN level - warnings and non-critical issues
#if RAC_LOG_WARN_ENABLE
#define RAC_LOG_WARN( ... )                     \
    do                                          \
    {                                           \
        RAC_LOG_FORMAT( "WARN ", __VA_ARGS__ ); \
    } while( 0 )
#else
#define RAC_LOG_WARN( ... ) \
    do                      \
    {                       \
    } while( 0 )
#endif

// INFO level - general information
#if RAC_LOG_INFO_ENABLE
#define RAC_LOG_INFO( ... )                     \
    do                                          \
    {                                           \
        RAC_LOG_FORMAT( "INFO ", __VA_ARGS__ ); \
    } while( 0 )
#else
#define RAC_LOG_INFO( ... ) \
    do                      \
    {                       \
    } while( 0 )
#endif

// DEBUG level - detailed debugging information
#if RAC_LOG_DEBUG_ENABLE
#define RAC_LOG_DEBUG( ... )                    \
    do                                          \
    {                                           \
        RAC_LOG_FORMAT( "DEBUG", __VA_ARGS__ ); \
    } while( 0 )
#else
#define RAC_LOG_DEBUG( ... ) \
    do                       \
    {                        \
    } while( 0 )
#endif

// CONFIG level - configuration and initialization
#if RAC_LOG_CONFIG_ENABLE
#define RAC_LOG_CONFIG( ... )                   \
    do                                          \
    {                                           \
        RAC_LOG_FORMAT( "CONF ", __VA_ARGS__ ); \
    } while( 0 )
#else
#define RAC_LOG_CONFIG( ... ) \
    do                        \
    {                         \
    } while( 0 )
#endif

// TX level - transmission events
#if RAC_LOG_TX_ENABLE
#define RAC_LOG_TX( ... )                       \
    do                                          \
    {                                           \
        RAC_LOG_FORMAT( "TX   ", __VA_ARGS__ ); \
    } while( 0 )
#else
#define RAC_LOG_TX( ... ) \
    do                    \
    {                     \
    } while( 0 )
#endif

// RX level - reception events
#if RAC_LOG_RX_ENABLE
#define RAC_LOG_RX( ... )                       \
    do                                          \
    {                                           \
        RAC_LOG_FORMAT( "RX   ", __VA_ARGS__ ); \
    } while( 0 )
#else
#define RAC_LOG_RX( ... ) \
    do                    \
    {                     \
    } while( 0 )
#endif

// STATS level - statistics and performance metrics
#if RAC_LOG_STATS_ENABLE
#define RAC_LOG_STATS( ... )                    \
    do                                          \
    {                                           \
        RAC_LOG_FORMAT( "STATS", __VA_ARGS__ ); \
    } while( 0 )
#else
#define RAC_LOG_STATS( ... ) \
    do                       \
    {                        \
    } while( 0 )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- UTILITY MACROS ----------------------------------------------------------
 */

// Hex dump utility (conditional)
#if RAC_LOG_DEBUG_ENABLE
#define RAC_LOG_HEX_DUMP( prefix, data, size )             \
    do                                                     \
    {                                                      \
        RAC_LOG_DEBUG( "%s (%u bytes):\n", prefix, size ); \
        SMTC_HAL_TRACE_ARRAY( "", data, size );            \
    } while( 0 )
#else
#define RAC_LOG_HEX_DUMP( prefix, data, size ) \
    do                                         \
    {                                          \
    } while( 0 )
#endif

// Application banner
#define RAC_LOG_BANNER( app_name, version )                     \
    do                                                          \
    {                                                           \
        RAC_LOG_INFO( "================================\n" );   \
        RAC_LOG_INFO( "  %s v%s\n", app_name, version );        \
        RAC_LOG_INFO( "  Built: %s %s\n", __DATE__, __TIME__ ); \
        RAC_LOG_INFO( "================================\n" );   \
    } while( 0 )

// Simplified banner without version
#define RAC_LOG_SIMPLE_BANNER( app_name )                     \
    do                                                        \
    {                                                         \
        RAC_LOG_INFO( "================================\n" ); \
        RAC_LOG_INFO( "      %s\n", app_name );               \
        RAC_LOG_INFO( "================================\n" ); \
    } while( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- LEGACY COMPATIBILITY ---------------------------------------------------
 */

// Backward compatibility with existing examples
#ifndef DEBUG_PRINT
#define DEBUG_PRINT( ... ) RAC_LOG_INFO( __VA_ARGS__ )
#endif

#ifdef __cplusplus
}
#endif

#endif  // RAC_LOG_H
