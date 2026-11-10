/**
 * @file      main_one_channel_hub.c
 *
 * @brief     1-channel LoRaWAN gateway (hub) example based on RAC
 *
 * This application integrates the LoRa packet forwarder with the RAC
 * (Radio Access Controller). It initializes the RAC, spawns the packet
 * forwarder in a separate thread, and runs the RAC engine in the main loop.
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
#include "main_one_channel_hub.h"

/* Standard C library */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/* System/POSIX headers */
#include <pthread.h>
#include <signal.h>

/* HAL layer */
#include "smtc_hal_button.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_hal_led.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_watchdog.h"

/* RAC API layer */
#include "smtc_rac_api.h"

/* LBM API layer */
#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"

/* Application layer */
#include "lora_pkt_fwd.h"
#include "lorawan_client_conf.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

#define MAIN_LOG_INFO( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )
#define MAIN_LOG_WARN( ... ) SMTC_HAL_TRACE_WARNING( __VA_ARGS__ )
#define MAIN_LOG_ERROR( ... ) SMTC_HAL_TRACE_ERROR( __VA_ARGS__ )

#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a

/*!
 * @brief Helper macro that returned a human-friendly message if a command does not return SMTC_MODEM_RC_OK
 *
 * @remark The macro is implemented to be used with functions returning a @ref smtc_modem_return_code_t
 *
 * @param[in] rc  Return code
 */

#define ASSERT_SMTC_MODEM_RC( rc_func )                                                         \
    do                                                                                          \
    {                                                                                           \
        smtc_modem_return_code_t rc = rc_func;                                                  \
        if( rc == SMTC_MODEM_RC_NOT_INIT )                                                      \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_NOT_INIT ) );                             \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_INVALID )                                                  \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_INVALID ) );                              \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_BUSY )                                                     \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_BUSY ) );                                 \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_FAIL )                                                     \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_FAIL ) );                                 \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_NO_TIME )                                                  \
        {                                                                                       \
            SMTC_HAL_TRACE_WARNING( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__, \
                                    xstr( SMTC_MODEM_RC_NO_TIME ) );                            \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_INVALID_STACK_ID )                                         \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_INVALID_STACK_ID ) );                     \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_NO_EVENT )                                                 \
        {                                                                                       \
            SMTC_HAL_TRACE_INFO( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,    \
                                 xstr( SMTC_MODEM_RC_NO_EVENT ) );                              \
        }                                                                                       \
    } while( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * Stack id value
 */
#define STACK_ID 0

/**
 * @brief Maximum payload size in byte of LoRaWAN payload
 */
#define SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH 242

/**
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog period (here 32s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS 20000

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief LoRaWAN client configuration (loaded from JSON file)
 */
static lorawan_client_conf_t lw_client_conf;
static bool                  lw_client_enabled = false;

static uint8_t                  rx_payload[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH] = { 0 };  // Buffer for rx payload
static uint8_t                  rx_payload_size = 0;      // Size of the payload in the rx_payload buffer
static smtc_modem_dl_metadata_t rx_metadata     = { 0 };  // Metadata of downlink
static uint8_t                  rx_remaining    = 0;      // Remaining downlink payload in modem

/**
 * @brief Alert flag for connection status changes
 * Value: PKT_FWD_CONN_EVENT_NONE, PKT_FWD_CONN_EVENT_OK or PKT_FWD_CONN_EVENT_LOST
 */
static struct
{
    pthread_mutex_t      mutex;
    pkt_fwd_conn_event_t status_value;
} alert_flag = { .mutex = PTHREAD_MUTEX_INITIALIZER, .status_value = PKT_FWD_CONN_EVENT_NONE };

/**
 * @brief Pending connection status to send after join
 */
static struct
{
    bool                 pending;
    pkt_fwd_conn_event_t status_value;
} pending_connection_status = { .pending = false, .status_value = PKT_FWD_CONN_EVENT_NONE };

/**
 * @brief Last known connection status
 */
static pkt_fwd_conn_event_t last_connection_status = PKT_FWD_CONN_EVENT_OK;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Packet forwarder thread
 *
 * This thread runs the packet forwarder's main function.
 * The packet forwarder handles:
 * - HAL configuration and start (lgw_start)
 * - Network socket setup to LNS
 * - Spawning worker threads (thread_up, thread_down, thread_jit)
 * - Statistics collection and display
 * - HAL shutdown (lgw_stop)
 *
 * @param arg Pointer to the configuration file path (const char*)
 */
static void* pkt_fwd_thread( void* arg );

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void modem_event_callback( void );

/**
 * @brief Connection status callback from packet forwarder
 *
 * This callback is invoked from thread_down when connection status changes.
 * It simply sets flags that will be checked in the main loop.
 *
 * @param event   Connection event type
 * @param context User context (unused)
 */
static void connection_status_callback( pkt_fwd_conn_event_t event, void* context );

/**
 * @brief Send connection status uplink via LBM
 *
 * @param status_value Status value (PKT_FWD_CONN_EVENT_OK or PKT_FWD_CONN_EVENT_LOST)
 */
static void send_connection_status_uplink( pkt_fwd_conn_event_t status_value );

/**
 * @brief Process pending connection status alerts (call after JOIN)
 */
static void process_pending_connection_status( void );

/**
 * @brief Turn off all status LEDs (registered via atexit).
 */
static void all_leds_off( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

int main( void )
{
    int       ret;
    pthread_t thrid_pkt_fwd;
    char*     conf_file     = NULL;
    uint32_t  sleep_time_ms = 0;

    /* Get configuration file from environment variable (optional) */
    conf_file = getenv( "GLOBAL_CONF" );

    /* Initialize device systems */
    hal_mcu_init( );

    // Initialize LEDs and button
    hal_led_init( );
    hal_button_init( NULL, NULL );

    /* Clear LEDs on any exit path. */
    atexit( all_leds_off );

    /* Initialize RAC (Radio Access Controller) */
    smtc_rac_init( );

    /*
     * -------------------------------------------------------------------------
     * Load LoRaWAN client configuration (optional)
     * If the config file exists and is valid, the LoRaWAN client will be enabled.
     * This allows the one-channel hub to also act as a LoRaWAN end-device for
     * sending alerts (e.g., when UDP link to LNS is broken).
     * -------------------------------------------------------------------------
     */
    lw_client_enabled = lorawan_client_conf_load( NULL, &lw_client_conf );

    if( lw_client_enabled )
    {
        lorawan_client_conf_print( &lw_client_conf );
        /* Initialize LBM (LoRa Basics Modem) */
        smtc_modem_init( &modem_event_callback );
        MAIN_LOG_INFO( "LoRaWAN client initialized\n" );
        MAIN_LOG_INFO( "Connection monitoring enabled - alerts will be sent on port %u\n", lw_client_conf.alert_port );
    }
    else
    {
        MAIN_LOG_INFO( "LoRaWAN client disabled - gateway-only mode\n" );
    }

    /* Register connection status callback (drives status LED, and alert uplinks when enabled). */
    pkt_fwd_register_connection_callback( connection_status_callback, NULL );

    MAIN_LOG_INFO( "========================================\n" );
    MAIN_LOG_INFO( "   ONE CHANNEL HUB - Packet Forwarder\n" );
    MAIN_LOG_INFO( "========================================\n" );
    if( conf_file != NULL )
    {
        MAIN_LOG_INFO( "Gateway configuration file: %s\n", conf_file );
    }

    /* Create packet forwarder thread, passing config file path */
    ret = pthread_create( &thrid_pkt_fwd, NULL, pkt_fwd_thread, ( void* ) conf_file );
    if( ret != 0 )
    {
        MAIN_LOG_ERROR( "Failed to create packet forwarder thread\n" );
        return EXIT_FAILURE;
    }

    MAIN_LOG_INFO( "Packet forwarder thread started\n" );
    MAIN_LOG_INFO( "RAC engine running in main loop...\n" );
    MAIN_LOG_INFO( "========================================\n" );

    /*
     * -------------------------------------------------------------------------
     * Main loop - runs RAC engine (and optionally LBM modem engine)
     *
     * This is critical: the RAC engine must run continuously to process
     * radio transactions and trigger callbacks for RX/TX events.
     * The packet forwarder's HAL functions (lgw_receive, lgw_send) rely on
     * the RAC engine being run regularly.
     *
     * If LoRaWAN client is enabled, we also run the LBM modem engine which
     * handles join, uplinks, and RX windows for the end-device functionality.
     * -------------------------------------------------------------------------
     */
    while( !exit_sig && !quit_sig )
    {
        /* Tell the watchdog that the device is still running */
        hal_watchdog_reload( );

        /* Check and handle connection alert flag. */
        pthread_mutex_lock( &alert_flag.mutex );
        pkt_fwd_conn_event_t connection_status = alert_flag.status_value;
        alert_flag.status_value                = PKT_FWD_CONN_EVENT_NONE; /* Clear flag */
        pthread_mutex_unlock( &alert_flag.mutex );

        /* Handle connection status change if any */
        if( connection_status != PKT_FWD_CONN_EVENT_NONE )
        {
            /* Update last known status */
            last_connection_status = connection_status;

            /* Drive the status LEDs: red (HAL_LED_TX) ON when the link to the LNS is
             * lost, green (HAL_LED_RX) ON when it is OK. */
            bool connection_ok = ( connection_status == PKT_FWD_CONN_EVENT_OK );
            hal_led_set( HAL_LED_TX, !connection_ok );
            hal_led_set( HAL_LED_RX, connection_ok );
            printf( "Connection status: %s\n", connection_ok ? "OK" : "LOST" );

            /* When acting as a LoRaWAN end-device, also report the status via uplink */
            if( lw_client_enabled )
            {
                /* Check if modem has joined before sending */
                smtc_modem_status_mask_t modem_status;
                ASSERT_SMTC_MODEM_RC( smtc_modem_get_status( STACK_ID, &modem_status ) );
                bool is_joined = ( modem_status & SMTC_MODEM_STATUS_JOINED ) != 0;

                if( is_joined )
                {
                    /* Send immediately */
                    const char* status_str = ( connection_status == PKT_FWD_CONN_EVENT_OK ) ? "OK" : "LOST";
                    MAIN_LOG_INFO( "Gateway connection %s - sending uplink\n", status_str );
                    send_connection_status_uplink( connection_status );
                }
                else
                {
                    /* Queue for later */
                    const char* status_str = ( connection_status == PKT_FWD_CONN_EVENT_OK ) ? "OK" : "LOST";
                    MAIN_LOG_INFO( "Gateway connection %s - queued until modem joins\n", status_str );
                    pending_connection_status.pending      = true;
                    pending_connection_status.status_value = connection_status;
                }
            }
        }

        /* Run modem engine if LoRaWAN client is enabled.
         *
         * smtc_modem_run_engine() drives the shared radio planner directly but
         * does NOT take the RAC API lock internally, so we must hold it here to
         * serialize against the worker threads (lgw_send/JIT) and the radio-IRQ /
         * LP-timer threads. (smtc_rac_run_engine() below already locks itself.) */
        if( lw_client_enabled )
        {
            smtc_modem_hal_protect_api_call( );
            sleep_time_ms = smtc_modem_run_engine( );
            smtc_modem_hal_unprotect_api_call( );
        }
        else
        {
            sleep_time_ms = WATCHDOG_RELOAD_PERIOD_MS;
        }

        /* Run RAC engine (processes radio transactions, triggers callbacks).
         * Self-protected: it takes the RAC API lock internally. */
        smtc_rac_run_engine( );

        /* Sleep to avoid busy-waiting */
        hal_mcu_disable_irq( );
        if( smtc_rac_is_irq_flag_pending( ) == false )
        {
            hal_mcu_set_sleep_for_ms( MIN( sleep_time_ms, WATCHDOG_RELOAD_PERIOD_MS ) );
            hal_watchdog_reload( );
        }
        hal_watchdog_reload( );
        hal_mcu_enable_irq( );
    }

    /* Send signal to packet forwarder thread to interrupt any blocking sleep */
    MAIN_LOG_INFO( "Signaling packet forwarder thread to exit...\n" );
    pthread_kill( thrid_pkt_fwd, SIGINT );

    /* Wait for packet forwarder thread to finish */
    MAIN_LOG_INFO( "Waiting for packet forwarder to shut down...\n" );
    pthread_join( thrid_pkt_fwd, NULL );

    MAIN_LOG_INFO( "========================================\n" );
    MAIN_LOG_INFO( "   ONE CHANNEL HUB - Shutdown complete\n" );
    MAIN_LOG_INFO( "========================================\n" );

    return EXIT_SUCCESS;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief Packet forwarder thread wrapper
 *
 * This thread runs the packet forwarder's main function.
 * The packet forwarder handles:
 * - HAL configuration and start (lgw_start)
 * - Network socket setup to LNS
 * - Spawning worker threads (thread_up, thread_down, thread_jit)
 * - Statistics collection and display
 * - HAL shutdown (lgw_stop)
 *
 * @param arg Pointer to the configuration file path (const char*)
 */
static void* pkt_fwd_thread( void* arg )
{
    const char* conf_file = ( const char* ) arg;

    MAIN_LOG_INFO( "[pkt_fwd_thread] Starting packet forwarder...\n" );

    /* Run the packet forwarder - this blocks until exit_sig is set */
    int ret = main_pkt_fwd( conf_file );

    if( ret != 0 )
    {
        MAIN_LOG_ERROR( "[pkt_fwd_thread] Packet forwarder returned error: %d\n", ret );
    }

    MAIN_LOG_INFO( "[pkt_fwd_thread] Packet forwarder exited\n" );

    return NULL;
}

static void modem_event_callback( void )
{
    SMTC_HAL_TRACE_MSG_COLOR( "Modem event callback\n", HAL_DBG_TRACE_COLOR_BLUE );

    smtc_modem_event_t current_event;
    uint8_t            event_pending_count;
    uint8_t            stack_id = STACK_ID;

    // Continue to read modem event until all event has been processed
    do
    {
        // Read modem event
        ASSERT_SMTC_MODEM_RC( smtc_modem_get_event( &current_event, &event_pending_count ) );

        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            SMTC_HAL_TRACE_INFO( "Event received: RESET\n" );

            /* Set credentials from configuration file */
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_deveui( stack_id, lw_client_conf.dev_eui ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_joineui( stack_id, lw_client_conf.join_eui ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_nwkkey( stack_id, lw_client_conf.app_key ) );

            /* Set region from configuration */
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_region( stack_id, lw_client_conf.region ) );

            /* Schedule a Join LoRaWAN network */
            ASSERT_SMTC_MODEM_RC( smtc_modem_join_network( stack_id ) );
            break;

        case SMTC_MODEM_EVENT_ALARM:
            SMTC_HAL_TRACE_INFO( "Event received: ALARM\n" );
            /* Send periodic connection status update (keep-alive) */
            MAIN_LOG_INFO( "Sending periodic connection status: %s\n",
                           ( last_connection_status == PKT_FWD_CONN_EVENT_OK ) ? "OK" : "LOST" );
            send_connection_status_uplink( last_connection_status );
            /* Restart periodical alarm */
            ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( lw_client_conf.keep_alive_interval_s ) );
            break;

        case SMTC_MODEM_EVENT_JOINED:
            SMTC_HAL_TRACE_INFO( "Event received: JOINED\n" );
            /* Send any pending connection status alert */
            process_pending_connection_status( );
            /* Start periodical uplink alarm */
            ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( lw_client_conf.keep_alive_interval_s ) );
            break;

        case SMTC_MODEM_EVENT_TXDONE:
            SMTC_HAL_TRACE_INFO( "Event received: TXDONE\n" );
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
            SMTC_HAL_TRACE_INFO( "Event received: DOWNDATA\n" );
            /* Get downlink data */
            ASSERT_SMTC_MODEM_RC(
                smtc_modem_get_downlink_data( rx_payload, &rx_payload_size, &rx_metadata, &rx_remaining ) );
            SMTC_HAL_TRACE_PRINTF( "Data received on port %u\n", rx_metadata.fport );
            SMTC_HAL_TRACE_ARRAY( "Received payload", rx_payload, rx_payload_size );
            break;

        case SMTC_MODEM_EVENT_JOINFAIL:
            SMTC_HAL_TRACE_INFO( "Event received: JOINFAIL\n" );
            break;

        case SMTC_MODEM_EVENT_LINK_CHECK:
            SMTC_HAL_TRACE_INFO( "Event received: LINK_CHECK\n" );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
            SMTC_HAL_TRACE_WARNING( "Event received: LORAWAN MAC TIME\n" );
            break;

        case SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE:
            SMTC_HAL_TRACE_INFO( "Event received: DUTY_CYCLE\n" );
            break;

        default:
            SMTC_HAL_TRACE_ERROR( "Unhandled event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
}

static void connection_status_callback( pkt_fwd_conn_event_t event, void* context )
{
    /* This callback runs in thread_down context - keep it minimal and non-blocking */
    pthread_mutex_lock( &alert_flag.mutex );

    switch( event )
    {
    case PKT_FWD_CONN_EVENT_OK:
        alert_flag.status_value = PKT_FWD_CONN_EVENT_OK;
        break;

    case PKT_FWD_CONN_EVENT_LOST:
        alert_flag.status_value = PKT_FWD_CONN_EVENT_LOST;
        break;

    default:
        break;
    }

    pthread_mutex_unlock( &alert_flag.mutex );
}

static void send_connection_status_uplink( pkt_fwd_conn_event_t status_value )
{
    uint8_t payload[1] = { status_value };

    smtc_modem_return_code_t rc = smtc_modem_request_uplink( STACK_ID, lw_client_conf.alert_port, false, payload, 1 );

    if( rc != SMTC_MODEM_RC_OK )
    {
        MAIN_LOG_ERROR( "Failed to queue connection status uplink: %d\n", rc );
    }
}

static void process_pending_connection_status( void )
{
    if( pending_connection_status.pending )
    {
        MAIN_LOG_INFO( "Sending queued connection status (%s)\n",
                       ( pending_connection_status.status_value == PKT_FWD_CONN_EVENT_OK ) ? "OK" : "LOST" );
        send_connection_status_uplink( pending_connection_status.status_value );
        pending_connection_status.pending = false;
    }
}

static void all_leds_off( void )
{
    for( hal_led_id_t led = 0; led < HAL_LED_COUNT; led++ )
    {
        hal_led_set( led, false );
    }
}