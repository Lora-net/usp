/**
 * @file      smtc_rac.c
 *
 * @brief     smtc_rac api implementation
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

#include "smtc_rac.h"
#include <stdbool.h>  // bool type
#include <stdint.h>   // C99 types

#include "radio_planner.h"
#include "ral.h"
#include "ralf.h"
#include "smtc_rac_api.h"
#include "smtc_rac_version.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_rac_local_func.h"
#if defined( SX128X )
#include "ralf_sx128x.h"
#elif defined( SX126X )
#include "ralf_sx126x.h"
#elif defined( LR11XX )
#include "ralf_lr11xx.h"
#elif defined( SX127X )
#include "ralf_sx127x.h"
#elif defined( LR20XX )
#include "ralf_lr20xx.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// RAC logging control - Configure which log types are enabled
//
// USAGE: To enable/disable specific log types, define the corresponding macro
// before compilation:
//   Example in CMakeLists.txt: target_compile_definitions(target_name PRIVATE
//   RAC_CORE_LOG_INFO_ENABLE=1) Example in compiler flags:
//   -DRAC_CORE_LOG_DEBUG_ENABLE=1 -DRAC_CORE_LOG_API_ENABLE=1
//
// Available controls:
//   RAC_CORE_LOG_ERROR_ENABLE   - Critical errors (default: 1)
//   RAC_CORE_LOG_CONFIG_ENABLE  - Configuration info (default: 1)
//   RAC_CORE_LOG_INFO_ENABLE    - General information (default: 0)
//   RAC_CORE_LOG_WARN_ENABLE    - Warnings (default: 0)
//   RAC_CORE_LOG_DEBUG_ENABLE   - Debug traces (default: 0)
//   RAC_CORE_LOG_API_ENABLE     - API call traces (default: 0)
//   RAC_CORE_LOG_RADIO_ENABLE   - Radio operations (default: 0)
//
// By default: only ERROR and CONFIG are enabled
#ifndef RAC_CORE_LOG_ERROR_ENABLE
#define RAC_CORE_LOG_ERROR_ENABLE 1  // Always enabled for critical errors
#endif
#ifndef RAC_CORE_LOG_CONFIG_ENABLE
#define RAC_CORE_LOG_CONFIG_ENABLE 1  // Enabled for configuration info
#endif
#ifndef RAC_CORE_LOG_INFO_ENABLE
#define RAC_CORE_LOG_INFO_ENABLE 0  // Disabled by default
#endif
#ifndef RAC_CORE_LOG_WARN_ENABLE
#define RAC_CORE_LOG_WARN_ENABLE 0  // Disabled by default
#endif
#ifndef RAC_CORE_LOG_DEBUG_ENABLE
#define RAC_CORE_LOG_DEBUG_ENABLE 0  // Disabled by default
#endif
#ifndef RAC_CORE_LOG_API_ENABLE
#define RAC_CORE_LOG_API_ENABLE 0  // Disabled by default
#endif
#ifndef RAC_CORE_LOG_RADIO_ENABLE
#define RAC_CORE_LOG_RADIO_ENABLE 0  // Disabled by default
#endif

// Professional logging macros for RAC - C99 compatible with individual
// control
#if RAC_CORE_LOG_INFO_ENABLE
#define RAC_CORE_LOG_INFO( ... )                                                        \
    do                                                                                  \
    {                                                                                   \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_CYAN "[RAC] [INFO ] " ); \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                     \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );          \
    } while( 0 )
#else
#define RAC_CORE_LOG_INFO( ... ) \
    do                           \
    {                            \
    } while( 0 )
#endif

#if RAC_CORE_LOG_WARN_ENABLE
#define RAC_CORE_LOG_WARN( ... )                                                          \
    do                                                                                    \
    {                                                                                     \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_YELLOW "[RAC] [WARN ] " ); \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                       \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );            \
    } while( 0 )
#else
#define RAC_CORE_LOG_WARN( ... ) \
    do                           \
    {                            \
    } while( 0 )
#endif

#if RAC_CORE_LOG_ERROR_ENABLE
#define RAC_CORE_LOG_ERROR( ... )                                                      \
    do                                                                                 \
    {                                                                                  \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_RED "[RAC] [ERROR] " ); \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                    \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );         \
    } while( 0 )
#else
#define RAC_CORE_LOG_ERROR( ... ) \
    do                            \
    {                             \
    } while( 0 )
#endif

#if RAC_CORE_LOG_DEBUG_ENABLE
#define RAC_CORE_LOG_DEBUG( ... )                                                          \
    do                                                                                     \
    {                                                                                      \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_MAGENTA "[RAC] [DEBUG] " ); \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                        \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );             \
    } while( 0 )
#else
#define RAC_CORE_LOG_DEBUG( ... ) \
    do                            \
    {                             \
    } while( 0 )
#endif

#if RAC_CORE_LOG_API_ENABLE
#define RAC_CORE_LOG_API( ... )                                                          \
    do                                                                                   \
    {                                                                                    \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_GREEN "[RAC] [API  ] " ); \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                      \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );           \
    } while( 0 )
#else
#define RAC_CORE_LOG_API( ... ) \
    do                          \
    {                           \
    } while( 0 )
#endif

#if RAC_CORE_LOG_RADIO_ENABLE
#define RAC_CORE_LOG_RADIO( ... )                                                       \
    do                                                                                  \
    {                                                                                   \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_BLUE "[RAC] [RADIO] " ); \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                     \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );          \
    } while( 0 )
#else
#define RAC_CORE_LOG_RADIO( ... ) \
    do                            \
    {                             \
    } while( 0 )
#endif

#if RAC_CORE_LOG_CONFIG_ENABLE
#define RAC_CORE_LOG_CONFIG( ... )                                                       \
    do                                                                                   \
    {                                                                                    \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_WHITE "[RAC] [CONF ] " ); \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                      \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );           \
    } while( 0 )
#else
#define RAC_CORE_LOG_CONFIG( ... ) \
    do                             \
    {                              \
    } while( 0 )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct smtc_rac_callback_e
{
    uint8_t argument;
    void ( *callback )( rp_status_t status );
} smtc_rac_callback_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

radio_planner_t smtc_rac_radio_planner;

#if defined( SX128X )
ralf_t smtc_rac_radio = RALF_SX128X_INSTANTIATE( NULL );
#elif defined( SX126X )
ralf_t smtc_rac_radio = RALF_SX126X_INSTANTIATE( NULL );
#elif defined( LR11XX )
ralf_t smtc_rac_radio = RALF_LR11XX_INSTANTIATE( NULL );
#elif defined( LR20XX )
ralf_t smtc_rac_radio = RALF_LR20XX_INSTANTIATE( NULL );
#elif defined( SX127X )
#include "sx127x.h"
static sx127x_t sx127x;
ralf_t          smtc_rac_radio = RALF_SX127X_INSTANTIATE( &sx127x );
#else
#error "Please select radio board.."
#endif
#define RALF_RADIO_POINTER smtc_rac_radio_planner.radio
#define RAL_RADIO_POINTER &( smtc_rac_radio_planner.radio->ral )

// TODO: HYT-129: hide these arrays in the rac_context
static smtc_rac_callback_t smtc_rac_callbacks[RP_HOOK_ID_MAX];
static smtc_rac_context_t  smtc_rac_context[RP_HOOK_ID_MAX];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void smtc_rac_empty_callback( void* context );
static void smtc_rac_lock_radio_access_callback( void* context );
static void smtc_rac_rp_callback( void* callback_void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/* ------------ Modem Utilities ------------*/

smtc_rac_return_code_t smtc_rac_get_version( smtc_usp_rac_version_t* version )
{
    if( NULL == version )
    {
        return SMTC_RAC_INVALID_PARAMETER;
    }

    version->major = USP_RAC_VERSION_MAJOR;
    version->minor = USP_RAC_VERSION_MINOR;
    version->patch = USP_RAC_VERSION_PATCH;

    return SMTC_RAC_SUCCESS;
}

void smtc_rac_init( void )
{
    smtc_modem_hal_protect_api_call( );

    RAC_CORE_LOG_INFO( "===== SMTC RAC INITIALIZATION =====" );
    RAC_CORE_LOG_INFO( "USP/RAC Version: %d.%d.%d", USP_RAC_VERSION_MAJOR, USP_RAC_VERSION_MINOR,
                       USP_RAC_VERSION_PATCH );

#if defined( SX128X )
    RAC_CORE_LOG_RADIO( "Radio Type: SX128X" );
#elif defined( SX126X )
    RAC_CORE_LOG_RADIO( "Radio Type: SX126X" );
#elif defined( LR11XX )
    RAC_CORE_LOG_RADIO( "Radio Type: LR11XX" );
#elif defined( LR20XX )
    RAC_CORE_LOG_RADIO( "Radio Type: LR20XX" );
#elif defined( SX127X )
    RAC_CORE_LOG_RADIO( "Radio Type: SX127X" );
#else
    RAC_CORE_LOG_ERROR( "Unknown radio type!" );
#endif

    RAC_CORE_LOG_INFO( "Initializing radio abstraction layer (RAL)..." );
    // init radio and put it in sleep mode
    if( ral_reset( &( smtc_rac_radio.ral ) ) != RAL_STATUS_OK )
    {
        RAC_CORE_LOG_ERROR( "RAL reset failed!" );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( false );
    }
    RAC_CORE_LOG_DEBUG( "RAL reset completed successfully" );

    if( ral_init( &( smtc_rac_radio.ral ) ) != RAL_STATUS_OK )
    {
        RAC_CORE_LOG_ERROR( "RAL initialization failed!" );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( false );
    }
    RAC_CORE_LOG_DEBUG( "RAL initialization completed successfully" );

    if( ral_set_sleep( &( smtc_rac_radio.ral ), true ) != RAL_STATUS_OK )
    {
        RAC_CORE_LOG_ERROR( "Failed to set radio to sleep mode!" );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( false );
    }
    RAC_CORE_LOG_DEBUG( "Radio set to sleep mode" );

    RAC_CORE_LOG_INFO( "Configuring antenna switch..." );
    smtc_modem_hal_set_ant_switch( false );
    RAC_CORE_LOG_DEBUG( "Antenna switch configured (disabled)" );

    // init radio planner and attach corresponding radio irq
    RAC_CORE_LOG_INFO( "Initializing radio planner..." );
    rp_init( &smtc_rac_radio_planner, &smtc_rac_radio );
    RAC_CORE_LOG_DEBUG( "Radio planner initialized successfully" );

    RAC_CORE_LOG_INFO( "Configuring radio IRQ handling..." );
    smtc_modem_hal_irq_config_radio_irq( rp_radio_irq_callback, &smtc_rac_radio_planner );
    RAC_CORE_LOG_DEBUG( "Radio IRQ callback configured" );

    RAC_CORE_LOG_INFO( "Setting up radio planner hooks..." );
    rp_hook_init( &smtc_rac_radio_planner, RP_HOOK_ID_SUSPEND, ( void ( * )( void* ) )( smtc_rac_empty_callback ),
                  &smtc_rac_radio_planner );
    RAC_CORE_LOG_DEBUG( "Radio planner suspend hook configured" );

    RAC_CORE_LOG_INFO( "===== RAC INITIALIZATION COMPLETE =====" );
    smtc_modem_hal_unprotect_api_call( );
}

void smtc_rac_set_radio_context( const void* radio_ctx )
{
    smtc_modem_hal_protect_api_call( );
    RAC_CORE_LOG_API( "Setting radio context (pointer: %p)", radio_ctx );

    if( radio_ctx == NULL )
    {
        RAC_CORE_LOG_WARN( "Radio context is NULL!" );
    }

#if defined( SX1272 ) || defined( SX1276 )
    // update smtc_rac_radio context with provided one
    ( ( sx127x_t* ) smtc_rac_radio.ral.context )->hal_context = radio_ctx;
    RAC_CORE_LOG_DEBUG( "SX127X HAL context updated" );
#else
    // update smtc_rac_radio context with provided one
    smtc_rac_radio.ral.context = radio_ctx;
    RAC_CORE_LOG_DEBUG( "RAL context updated" );
#endif

    RAC_CORE_LOG_API( "Radio context configuration complete" );
    smtc_modem_hal_unprotect_api_call( );
}

void smtc_rac_run_engine( void )
{
    smtc_modem_hal_protect_api_call( );
    RAC_CORE_LOG_DEBUG( "Running radio planner engine callback" );
    rp_callback( &smtc_rac_radio_planner );
    RAC_CORE_LOG_DEBUG( "Radio planner engine callback completed" );
    smtc_modem_hal_unprotect_api_call( );
}

bool smtc_rac_is_irq_flag_pending( void )
{
    smtc_modem_hal_protect_api_call( );
    bool temp = rp_get_irq_flag( &smtc_rac_radio_planner );
    RAC_CORE_LOG_DEBUG( "IRQ flag status: %s", temp ? "PENDING" : "CLEAR" );
    smtc_modem_hal_unprotect_api_call( );
    return temp;
}

radio_planner_t* smtc_rac_get_rp( void )
{
    smtc_modem_hal_protect_api_call( );
    RAC_CORE_LOG_API( "Retrieving radio planner instance" );
    radio_planner_t* temp = &smtc_rac_radio_planner;
    RAC_CORE_LOG_DEBUG( "Radio planner pointer: %p", temp );
    smtc_modem_hal_unprotect_api_call( );
    return temp;
}

ralf_t* smtc_rac_get_radio( void )
{
    smtc_modem_hal_protect_api_call( );
    RAC_CORE_LOG_API( "Retrieving ralf_t instance" );
    ralf_t* temp = &smtc_rac_radio;
    RAC_CORE_LOG_DEBUG( "Radio ralf pointer: %p", temp );
    smtc_modem_hal_unprotect_api_call( );
    return temp;
}

smtc_rac_context_t* smtc_rac_get_context( uint8_t radio_access_id )
{
    SMTC_RAC_CHECK_RADIO_ACCESS_ID( radio_access_id );
    smtc_modem_hal_protect_api_call( );
    RAC_CORE_LOG_API( "Retrieving context for radio access ID: %u", radio_access_id );

    if( radio_access_id >= RP_NB_HOOKS )
    {
        RAC_CORE_LOG_ERROR( "Invalid radio access ID: %u (max: %u)", radio_access_id, RP_NB_HOOKS - 1 );
        smtc_modem_hal_unprotect_api_call( );
        return NULL;
    }

    smtc_rac_context_t* temp = &smtc_rac_context[radio_access_id];
    RAC_CORE_LOG_DEBUG( "Context pointer for ID %u: %p", radio_access_id, temp );
    smtc_modem_hal_unprotect_api_call( );
    return temp;
}

uint8_t smtc_rac_open_radio( smtc_rac_priority_t priority )
{
    // Init the rac context to 0
    memset( &smtc_rac_context[priority], 0, sizeof( smtc_rac_context_t ) );
    memset( &smtc_rac_callbacks[priority], 0, sizeof( smtc_rac_callback_t ) );

    smtc_rac_callbacks[priority].argument = priority;

    smtc_modem_hal_protect_api_call( );
    rp_hook_status_t status =
        rp_hook_init( &smtc_rac_radio_planner, priority, ( void ( * )( void* ) )( smtc_rac_rp_callback ),
                      ( void* ) ( &( smtc_rac_callbacks[priority] ) ) );
    smtc_modem_hal_unprotect_api_call( );
    if( status != RP_HOOK_STATUS_OK )
    {
        return RAC_INVALID_RADIO_ID;
    }
    return ( uint8_t ) priority;
}
smtc_rac_return_code_t smtc_rac_close_radio( uint8_t radio_access_id )
{
    SMTC_RAC_CHECK_RADIO_ACCESS_ID( radio_access_id );
    smtc_modem_hal_protect_api_call( );

    if( rp_release_hook( &smtc_rac_radio_planner, radio_access_id ) == RP_HOOK_STATUS_OK )
    {
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_SUCCESS;
    }
    else
    {
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;  // Error releasing the radio access ID
    }
}
smtc_rac_return_code_t smtc_rac_abort_radio_submit( uint8_t radio_access_id )
{
    SMTC_RAC_CHECK_RADIO_ACCESS_ID( radio_access_id );
    smtc_modem_hal_protect_api_call( );

    if( rp_task_abort( &smtc_rac_radio_planner, radio_access_id ) == RP_HOOK_STATUS_OK )
    {
        smtc_modem_hal_unprotect_api_call( );  // @todo set here because the next will
                                               // call the callback which probably
                                               // will call the API again
        // @todo: check if the callback is called (at the next call to
        // `smtc_rac_run_engine`), which make the following redundant
        // smtc_rac_radio_planner.hook_callbacks[radio_access_id](&smtc_rac_radio_planner.status[radio_access_id]);
        return SMTC_RAC_SUCCESS;
    }
    else
    {
        smtc_modem_hal_unprotect_api_call( );
        return SMTC_RAC_ERROR;  // Error aborting the radio request
    }
}

smtc_rac_return_code_t smtc_rac_lock_radio_access( uint8_t                     radio_access_id,
                                                   smtc_rac_scheduler_config_t scheduler_config )
{
    SMTC_RAC_CHECK_RADIO_ACCESS_ID( radio_access_id );
    rp_radio_params_t fake_radio_params                = { 0 };
    uint8_t           fake_payload[1]                  = { 0 };
    smtc_rac_context[radio_access_id].scheduler_config = scheduler_config;
    smtc_rac_callbacks[radio_access_id].callback       = scheduler_config.callback_post_radio_transaction;

    rp_task_t rp_task = {
        .hook_id                    = radio_access_id,
        .type                       = RP_TASK_TYPE_LOCK_RADIO_ACCESS,
        .launch_task_callbacks      = smtc_rac_lock_radio_access_callback,
        .schedule_task_low_priority = false,
        .start_time_ms              = scheduler_config.start_time_ms,
        .duration_time_ms           = scheduler_config.duration_time_ms,
        .state =
            scheduler_config.scheduling == SMTC_RAC_SCHEDULED_TRANSACTION ? RP_TASK_STATE_SCHEDULE : RP_TASK_STATE_ASAP,

    };

    rp_hook_status_t status = rp_task_enqueue( smtc_rac_get_rp( ), &rp_task, fake_payload, 0, &fake_radio_params );

    if( status != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to suspend radio access with following error code: %x\n", status );
        return SMTC_RAC_ERROR;
    }
    return SMTC_RAC_SUCCESS;
}
smtc_rac_return_code_t smtc_rac_unlock_radio_access( uint8_t radio_access_id )
{
    return ( smtc_rac_abort_radio_submit( radio_access_id ) );
}
smtc_rac_return_code_t smtc_rac_submit_radio_transaction( uint8_t radio_access_id )
{
    SMTC_RAC_CHECK_RADIO_ACCESS_ID( radio_access_id );
    smtc_rac_callbacks[radio_access_id].callback =
        smtc_rac_context[radio_access_id].scheduler_config.callback_post_radio_transaction;
    switch( smtc_rac_context[radio_access_id].modulation_type )
    {
    case SMTC_RAC_MODULATION_LORA:
        if( ( smtc_rac_context[radio_access_id].radio_params.lora.is_tx == false ) &&
            ( smtc_rac_context[radio_access_id].smtc_rac_data_buffer_setup.size_of_rx_payload_buffer <
              smtc_rac_context[radio_access_id].radio_params.lora.max_rx_size ) )
        {
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( false );
        }
        else
        {
            return smtc_rac_lora( radio_access_id );
        }
        break;
    case SMTC_RAC_MODULATION_FSK:
        if( ( smtc_rac_context[radio_access_id].radio_params.fsk.is_tx == false ) &&
            ( smtc_rac_context[radio_access_id].smtc_rac_data_buffer_setup.size_of_rx_payload_buffer <
              smtc_rac_context[radio_access_id].radio_params.fsk.max_rx_size ) )
        {
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( false );
        }
        else
        {
            return smtc_rac_fsk( radio_access_id );
        }
        break;
    case SMTC_RAC_MODULATION_LRFHSS:
        return smtc_rac_lrfhss( radio_access_id );
        break;
    case SMTC_RAC_MODULATION_FLRC:
        if( ( smtc_rac_context[radio_access_id].radio_params.flrc.is_tx == false ) &&
            ( smtc_rac_context[radio_access_id].smtc_rac_data_buffer_setup.size_of_rx_payload_buffer <
              smtc_rac_context[radio_access_id].radio_params.flrc.max_rx_size ) )
        {
            SMTC_MODEM_HAL_PANIC_ON_FAILURE( false );
        }
        else
        {
            return smtc_rac_flrc( radio_access_id );
        }
        break;
    default:
        return SMTC_RAC_INVALID_PARAMETER;
    }
    return SMTC_RAC_SUCCESS;  // Never reached
}

static void smtc_rac_lock_radio_access_callback( void* context )
{
    radio_planner_t* rp              = ( radio_planner_t* ) context;
    uint8_t          radio_access_id = rp->radio_task_id;
    if( smtc_rac_context[radio_access_id].scheduler_config.callback_pre_radio_transaction != NULL )
    {
        smtc_rac_context[radio_access_id].scheduler_config.callback_pre_radio_transaction( );
    }
}

void* smtc_rac_get_radio_driver_context( void )
{
    return ( void* ) ( smtc_rac_get_rp( )->radio->ral.context );
}

/*
Private functions
*/

static void smtc_rac_empty_callback( void* context )
{
    UNUSED( context );
}

static void smtc_rac_rp_callback( void* callback_void )
{
    smtc_rac_callback_t* callback_wrapper = ( smtc_rac_callback_t* ) callback_void;
    smtc_rac_context[callback_wrapper->argument].smtc_rac_data_result.radio_end_timestamp_ms =
        ( smtc_rac_get_rp( )->irq_timestamp_ms[callback_wrapper->argument] );
    uint32_t    tmp_timestamp;
    rp_status_t tmp_status;
    rp_get_status( &smtc_rac_radio_planner, callback_wrapper->argument, &tmp_timestamp, &tmp_status );
    smtc_rac_context_t* rac_config = smtc_rac_get_context( callback_wrapper->argument );
    switch( tmp_status )
    {
    case RP_STATUS_RX_PACKET:
        if( smtc_rac_context[callback_wrapper->argument].modulation_type == SMTC_RAC_MODULATION_LORA )
        {
            smtc_rac_context[callback_wrapper->argument].smtc_rac_data_result.snr_result =
                ( &smtc_rac_radio_planner )->radio_params[callback_wrapper->argument].rx.lora_pkt_status.snr_pkt_in_db;
            smtc_rac_context[callback_wrapper->argument].smtc_rac_data_result.rssi_result =
                ( &smtc_rac_radio_planner )
                    ->radio_params[callback_wrapper->argument]
                    .rx.lora_pkt_status.rssi_pkt_in_dbm;
        }
        else if( smtc_rac_context[callback_wrapper->argument].modulation_type == SMTC_RAC_MODULATION_FSK )
        {
            smtc_rac_context[callback_wrapper->argument].smtc_rac_data_result.snr_result = 0;  // Not applicable for FSK
            smtc_rac_context[callback_wrapper->argument].smtc_rac_data_result.rssi_result =
                ( &smtc_rac_radio_planner )
                    ->radio_params[callback_wrapper->argument]
                    .rx.gfsk_pkt_status.rssi_avg_in_dbm;
        }

        else if( smtc_rac_context[callback_wrapper->argument].modulation_type == SMTC_RAC_MODULATION_FLRC )
        {
            smtc_rac_context[callback_wrapper->argument].smtc_rac_data_result.snr_result =
                0;  // Not applicable for FLRC
            smtc_rac_context[callback_wrapper->argument].smtc_rac_data_result.rssi_result =
                ( &smtc_rac_radio_planner )
                    ->radio_params[callback_wrapper->argument]
                    .rx.flrc_pkt_status.rssi_sync_in_dbm;
        }

        smtc_rac_context[callback_wrapper->argument].smtc_rac_data_result.rx_size =
            ( &smtc_rac_radio_planner )->rx_payload_size[callback_wrapper->argument];

        break;
    default:
        break;
    }
    if( ( rac_config->cad_context.cad_enabled == true ) &&
        ( rac_config->cad_context.cad_exit_mode == RAL_LORA_CAD_RX ) && ( tmp_status == RP_STATUS_CAD_POSITIVE ) )
    {
        return;  // don't call the callback
    }
    ( callback_wrapper->callback )( tmp_status );
}
