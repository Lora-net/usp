/**
 * @file      lora_pkt_fwd.h
 *
 * @brief     LoRa Packet Forwarder - Public API
 *
 * License: Revised BSD License, see LICENSE.TXT file include in the project
 */

#ifndef _LORA_PKT_FWD_H
#define _LORA_PKT_FWD_H

#include <stdbool.h>
#include <stdatomic.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/* atomic_bool: written from the signal handler and read by the worker threads */

/**
 * @brief Exit signal - set to true to terminate the packet forwarder cleanly
 */
extern volatile atomic_bool exit_sig;

/**
 * @brief Quit signal - set to true to terminate without cleanup
 */
extern volatile atomic_bool quit_sig;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Connection event types
 */
typedef enum pkt_fwd_conn_event_e
{
    PKT_FWD_CONN_EVENT_NONE = -1, /**< No event pending */
    PKT_FWD_CONN_EVENT_OK,        /**< Connection to LNS is OK (established or restored) */
    PKT_FWD_CONN_EVENT_LOST,      /**< Connection to LNS lost (threshold reached) */
} pkt_fwd_conn_event_t;

/**
 * @brief Connection status callback
 *
 * This callback is called from the packet forwarder's thread_down when the
 * connection status to the LNS changes. The implementation must be thread-safe
 * and non-blocking. Avoid calling blocking functions or performing I/O operations.
 *
 * @param event   Connection event type
 * @param context User context pointer passed during registration
 */
typedef void ( *pkt_fwd_connection_callback_t )( pkt_fwd_conn_event_t event, void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Main packet forwarder entry point
 *
 * This function:
 * - Parses configuration from the specified file
 * - Configures and starts the HAL (lgw_start)
 * - Opens network sockets to the LNS
 * - Spawns threads: thread_up, thread_down, thread_jit
 * - Runs the statistics loop
 * - Cleans up on exit (lgw_stop, close sockets)
 *
 * @param conf_file Path to the JSON configuration file
 * @return 0 on success, non-zero on error
 */
int main_pkt_fwd( const char* conf_file );

/**
 * @brief Register a connection status callback
 *
 * When a callback is registered and autoquit_threshold is configured (> 0),
 * the packet forwarder will invoke the callback when connection status changes
 * instead of terminating the application.
 *
 * The callback is invoked from thread_down context when:
 * - Connection OK: When first PULL_ACK is received (startup) or after connection loss
 * - Connection lost: After autoquit_threshold consecutive PULL_DATA without ACK
 *
 * @param callback Function to call on connection events (NULL to unregister)
 * @param context  User context pointer passed to callback
 */
void pkt_fwd_register_connection_callback( pkt_fwd_connection_callback_t callback, void* context );

#endif /* _LORA_PKT_FWD_H */
