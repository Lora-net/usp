/**
 * @file      smtc_hal_dbg_trace.h
 *
 * @brief     Debug Trace Hardware Abstraction Layer definition for Linux
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
#ifndef __SMTC_HAL_DBG_TRACE_H__
#define __SMTC_HAL_DBG_TRACE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief ANSI color codes for terminal output
 */
#define HAL_DBG_TRACE_COLOR_BLACK "\x1B[0;30m"
#define HAL_DBG_TRACE_COLOR_RED "\x1B[0;31m"
#define HAL_DBG_TRACE_COLOR_GREEN "\x1B[0;32m"
#define HAL_DBG_TRACE_COLOR_YELLOW "\x1B[0;33m"
#define HAL_DBG_TRACE_COLOR_BLUE "\x1B[0;34m"
#define HAL_DBG_TRACE_COLOR_MAGENTA "\x1B[0;35m"
#define HAL_DBG_TRACE_COLOR_CYAN "\x1B[0;36m"
#define HAL_DBG_TRACE_COLOR_WHITE "\x1B[0;37m"
#define HAL_DBG_TRACE_COLOR_DEFAULT "\x1B[0m"

/**
 * @brief Debug trace levels
 */
#define SMTC_HAL_TRACE_ERROR( ... ) \
    do                              \
    {                               \
        printf( "ERROR: " );        \
        printf( __VA_ARGS__ );      \
    } while( 0 )

#define SMTC_HAL_TRACE_WARNING( ... ) \
    do                                \
    {                                 \
        printf( "WARN: " );           \
        printf( __VA_ARGS__ );        \
    } while( 0 )

#define SMTC_HAL_TRACE_INFO( ... ) \
    do                             \
    {                              \
        printf( "INFO: " );        \
        printf( __VA_ARGS__ );     \
    } while( 0 )

#define SMTC_HAL_TRACE_PRINTF( ... ) \
    do                               \
    {                                \
        printf( __VA_ARGS__ );       \
    } while( 0 )

#define SMTC_HAL_TRACE_MSG( msg )        \
    do                                   \
    {                                    \
        printf( "%s\n", ( char* ) msg ); \
    } while( 0 )

#define SMTC_HAL_TRACE_MSG_COLOR( msg, color )                                   \
    do                                                                           \
    {                                                                            \
        printf( "%s%s%s\n", color, ( char* ) msg, HAL_DBG_TRACE_COLOR_DEFAULT ); \
    } while( 0 )

#define SMTC_HAL_TRACE_ARRAY( msg, array, len )                \
    do                                                         \
    {                                                          \
        printf( "%s - (%u bytes):\n", msg, ( uint32_t ) len ); \
        for( uint32_t i = 0; i < ( uint32_t ) len; i++ )       \
        {                                                      \
            printf( " %02X", array[i] );                       \
            if( ( ( i + 1 ) % 16 ) == 0 )                      \
            {                                                  \
                printf( "\n" );                                \
            }                                                  \
        }                                                      \
        printf( "\n" );                                        \
    } while( 0 )

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

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_DBG_TRACE_H__

/* --- EOF ------------------------------------------------------------------ */
