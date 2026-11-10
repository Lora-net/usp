/**
 * @file      hw_modem_linux.c
 *
 * @brief     Linux PTY-based hw_modem implementation
 *
 * Replaces the embedded GPIO+UART+DMA transport with a PTY (pseudo-terminal)
 * for use on Linux when the radio is directly connected to the host.
 * Python test scripts connect to the PTY slave as a serial port, using the
 * same frame protocol as the embedded version.
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
 * -----------------------------------------------------------------------------
 */

/* Required for posix_openpt(), grantpt(), unlockpt(), ptsname() */
#define _XOPEN_SOURCE 600

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <termios.h>
#include <sys/stat.h>

#include "hw_modem.h"
#include "cmd_parser.h"
#include "radio_utilities.h"
#include "smtc_modem_utilities.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 * -----------------------------------------------------------------------------
 */

#define HW_MODEM_RX_BUFF_MAX_LENGTH 261
#define HW_MODEM_PTY_SYMLINK_DEFAULT "/tmp/ttyHWMODEM"

/**
 * @brief Bridge emulation header prepended to responses.
 *
 * The Python test scripts expect responses framed as the HW bridge would send
 * them: a 2-byte bridge header followed by the hw_modem response. Since the
 * bridge_header value is never validated by the Python side (only return_code
 * is checked), we use a fixed value matching the observed bridge behavior.
 */
#define BRIDGE_HEADER_BYTE0 0x01
#define BRIDGE_HEADER_BYTE1 0x00
#define BRIDGE_HEADER_SIZE 2

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 * -----------------------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 * -----------------------------------------------------------------------------
 */

static int  pty_master_fd = -1;
static bool pty_initialized = false;
static char pty_slave_name[256];
static char pty_symlink_path[256];

/* Kept open for the process lifetime so the master side never sees a
 * spurious POLLHUP just because no client is currently connected to the
 * slave (e.g. between test runs). Without this, pty_watch_thread_func()'s
 * poll() would report HUP and return immediately forever whenever no
 * client is attached, forcing a busy-spin.
 */
static int pty_slave_keepalive_fd = -1;

static uint8_t modem_response_buff[HW_MODEM_RX_BUFF_MAX_LENGTH];
static uint8_t modem_received_buff[HW_MODEM_RX_BUFF_MAX_LENGTH];
static size_t  response_length;

static volatile bool hw_cmd_available = false;

/**
 * @brief PTY byte stream assembly buffer.
 *
 * A single read() on the PTY has no relation to command-frame boundaries: it may
 * return only part of a frame, or several queued frames at once. Bytes are
 * accumulated here across calls until a complete frame (cmd_id + length byte +
 * length payload bytes) is available; any bytes beyond that frame are kept for
 * the next call instead of being discarded. File-scope statics so they survive
 * the longjmp-based soft reset (see hw_modem_process_cmd()).
 */
static uint8_t rx_assembly[HW_MODEM_RX_BUFF_MAX_LENGTH];
static size_t  rx_assembled_len = 0;

/* Dedicated thread that wakes the main loop as soon as a host command arrives
 * on the PTY - mirrors the GPIO IRQ thread / lp_timer thread pattern in
 * smtc_hal_linux (see hal_mcu_wake_main_thread()). Without this, a command
 * arriving while the main loop is asleep in hal_mcu_set_sleep_for_ms() would
 * only be noticed once that sleep (up to WATCHDOG_RELOAD_PERIOD_MS) elapses
 * on its own, since PTY input has no hardware-interrupt equivalent on Linux.
 */
static pthread_t     pty_watch_thread;
static volatile bool pty_watch_thread_running = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 * -----------------------------------------------------------------------------
 */

/**
 * @brief Called by the soft modem engine each time an async event is available
 */
static void hw_modem_event_handler( void );

/**
 * @brief Cleanup PTY resources on exit
 */
static void hw_modem_pty_cleanup( void );

/**
 * @brief Signal handler for clean shutdown (SIGTERM, SIGINT)
 */
static void hw_modem_signal_handler( int signum );

/**
 * @brief Wakes the main loop as soon as the PTY has data to read
 */
static void* pty_watch_thread_func( void* arg );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 * -----------------------------------------------------------------------------
 */

int hw_modem_init( void )
{
    bool is_soft_reset = pty_initialized;

    if( !pty_initialized )
    {
        /* First call: create the PTY and register cleanup handlers.
         * On soft reset the PTY fd and symlink remain valid, so we skip this.
         */
        pty_master_fd = posix_openpt( O_RDWR | O_NOCTTY );
        if( pty_master_fd < 0 )
        {
            SMTC_HAL_TRACE_ERROR( "PTY: posix_openpt failed: %s\n", strerror( errno ) );
            return -1;
        }

        if( grantpt( pty_master_fd ) < 0 )
        {
            SMTC_HAL_TRACE_ERROR( "PTY: grantpt failed: %s\n", strerror( errno ) );
            close( pty_master_fd );
            pty_master_fd = -1;
            return -1;
        }

        if( unlockpt( pty_master_fd ) < 0 )
        {
            SMTC_HAL_TRACE_ERROR( "PTY: unlockpt failed: %s\n", strerror( errno ) );
            close( pty_master_fd );
            pty_master_fd = -1;
            return -1;
        }

        char* slave_name = ptsname( pty_master_fd );
        if( slave_name == NULL )
        {
            SMTC_HAL_TRACE_ERROR( "PTY: ptsname failed: %s\n", strerror( errno ) );
            close( pty_master_fd );
            pty_master_fd = -1;
            return -1;
        }
        strncpy( pty_slave_name, slave_name, sizeof( pty_slave_name ) - 1 );
        pty_slave_name[sizeof( pty_slave_name ) - 1] = '\0';

        /* Raw mode: disable echo and canonical processing on the slave side.
         * This fd is intentionally kept open (not closed) for the process
         * lifetime - see pty_slave_keepalive_fd's declaration.
         */
        pty_slave_keepalive_fd = open( pty_slave_name, O_RDWR | O_NOCTTY );
        if( pty_slave_keepalive_fd >= 0 )
        {
            struct termios tty;
            tcgetattr( pty_slave_keepalive_fd, &tty );
            cfmakeraw( &tty );
            tcsetattr( pty_slave_keepalive_fd, TCSANOW, &tty );
        }

        /* Symlink at a well-known path (override via HW_MODEM_PTY_PATH env var) */
        const char* env_path = getenv( "HW_MODEM_PTY_PATH" );
        snprintf( pty_symlink_path, sizeof( pty_symlink_path ), "%s",
                  env_path ? env_path : HW_MODEM_PTY_SYMLINK_DEFAULT );

        unlink( pty_symlink_path );
        if( symlink( pty_slave_name, pty_symlink_path ) < 0 )
        {
            SMTC_HAL_TRACE_WARNING( "PTY: symlink creation failed: %s (non-fatal)\n", strerror( errno ) );
        }

        atexit( hw_modem_pty_cleanup );
        signal( SIGTERM, hw_modem_signal_handler );
        signal( SIGINT, hw_modem_signal_handler );

        pty_watch_thread_running = true;
        if( pthread_create( &pty_watch_thread, NULL, pty_watch_thread_func, NULL ) != 0 )
        {
            SMTC_HAL_TRACE_WARNING( "PTY: failed to start watch thread, host commands may be delayed\n" );
            pty_watch_thread_running = false;
        }
        else
        {
            pthread_detach( pty_watch_thread );
        }

        pty_initialized = true;
    }

    memset( modem_response_buff, 0, HW_MODEM_RX_BUFF_MAX_LENGTH );
    hw_cmd_available = false;
    smtc_modem_init( &hw_modem_event_handler );

    if( is_soft_reset && pty_master_fd >= 0 )
    {
        /* The longjmp-based soft reset skipped the response path in
         * hw_modem_process_cmd(), but the testbench is still waiting for the
         * 2-byte bridge header. Send it now so the stream stays in sync.
         */
        uint8_t bridge_hdr[BRIDGE_HEADER_SIZE] = { BRIDGE_HEADER_BYTE0, BRIDGE_HEADER_BYTE1 };
        if( write( pty_master_fd, bridge_hdr, BRIDGE_HEADER_SIZE ) < 0 )
        {
            SMTC_HAL_TRACE_ERROR( "PTY write failed after soft reset: %s\n", strerror( errno ) );
        }
    }

    SMTC_HAL_TRACE_INFO( "HW_MODEM (Linux PTY): %s -> %s\n", pty_symlink_path, pty_slave_name );
    SMTC_HAL_TRACE_INFO( "HW_MODEM: Connect Python scripts to: %s\n", pty_symlink_path );

#if defined( PERF_TEST_ENABLED )
    SMTC_HAL_TRACE_WARNING( "HARDWARE MODEM RUNNING PERF TEST MODE\n" );
#endif

    return 0;
}

void hw_modem_process_cmd( void )
{
    uint8_t              cmd_length = 0xFF;
    cmd_response_t       output;
    cmd_input_t          input;
    cmd_serial_rc_code_t rc_code;

    /* 0xFF means the buffer is still at its pre-fill value (no real command) */
    if( modem_received_buff[0] < 0xFF )
    {
        cmd_length  = modem_received_buff[1];
        uint8_t crc = 0;

        for( int i = 0; i < cmd_length + 2; i++ )
        {
            crc = crc ^ modem_received_buff[i];
        }
        uint8_t calculated_crc = crc;
        uint8_t cmd_crc        = modem_received_buff[cmd_length + 2];
        uint8_t cmd_id         = modem_received_buff[0];

        if( calculated_crc != cmd_crc )
        {
            rc_code         = CMD_RC_FRAME_ERROR;
            response_length = 0;
            SMTC_HAL_TRACE_ERROR( "Cmd with bad crc %x / %x\n", calculated_crc, cmd_crc );
        }
        else if( ( modem_received_buff[cmd_length + 3] != 0xFF ) && ( cmd_length != 0xFF ) )
        {
            rc_code         = CMD_RC_FRAME_ERROR;
            response_length = 0;
            SMTC_HAL_TRACE_WARNING( " Extra data after the command\n" );
        }
        else
        {
            SMTC_HAL_TRACE_ARRAY( "Cmd input", modem_received_buff, cmd_length + 2 );
            input.cmd_code = cmd_id;
            input.length   = cmd_length;
            input.buffer   = &modem_received_buff[2];
            output.buffer  = &modem_response_buff[2];
            parse_cmd( &input, &output );
            rc_code         = output.return_code;
            response_length = output.length;
        }

        modem_response_buff[0] = rc_code;
        modem_response_buff[1] = response_length;

        SMTC_HAL_TRACE_ARRAY( "Cmd output", modem_response_buff, response_length + 2 );
        hw_cmd_available = false;

        /* Response CRC carries over from input CRC per bridge protocol */
        for( size_t i = 0; i < response_length + 2; i++ )
        {
            crc = crc ^ modem_response_buff[i];
        }
        modem_response_buff[response_length + 2] = crc;

        /* Bridge emulation: prepend 2-byte header, single write for atomicity */
        if( pty_master_fd >= 0 )
        {
            size_t  modem_len = response_length + 3;
            uint8_t bridge_response[BRIDGE_HEADER_SIZE + HW_MODEM_RX_BUFF_MAX_LENGTH];
            bridge_response[0] = BRIDGE_HEADER_BYTE0;
            bridge_response[1] = BRIDGE_HEADER_BYTE1;
            memcpy( &bridge_response[BRIDGE_HEADER_SIZE], modem_response_buff, modem_len );

            size_t  total_len = BRIDGE_HEADER_SIZE + modem_len;
            ssize_t written   = write( pty_master_fd, bridge_response, total_len );
            if( written < 0 )
            {
                SMTC_HAL_TRACE_ERROR( "PTY write failed: %s\n", strerror( errno ) );
            }
            else if( ( size_t ) written != total_len )
            {
                SMTC_HAL_TRACE_WARNING( "PTY partial write: %zd/%zu\n", written, total_len );
            }
        }
    }
    else
    {
        hw_cmd_available = false;
    }
}

bool hw_modem_is_a_cmd_available( void )
{
    if( hw_cmd_available == true )
    {
        return true;
    }

    if( pty_master_fd < 0 )
    {
        return false;
    }

    struct pollfd pfd;
    pfd.fd     = pty_master_fd;
    pfd.events = POLLIN;

    /* Pull in whatever new bytes are ready, appending to whatever is already
     * assembled from a previous call (a prior read() may have returned more
     * than one frame's worth of bytes - see below).
     */
    int ret = poll( &pfd, 1, 0 );
    if( ret > 0 && ( pfd.revents & POLLIN ) )
    {
        size_t space = HW_MODEM_RX_BUFF_MAX_LENGTH - rx_assembled_len;
        if( space > 0 )
        {
            ssize_t n = read( pty_master_fd, &rx_assembly[rx_assembled_len], space );
            if( n > 0 )
            {
                rx_assembled_len += ( size_t ) n;
            }
        }
    }

    /* A read() on a PTY has no relation to frame boundaries: it may return
     * only part of a frame, or several queued frames at once. Only declare a
     * command available once a complete frame (cmd_id + length byte + length
     * payload bytes) has actually been assembled - regardless of whether that
     * completes just now or was already sitting here from a previous, bigger
     * read().
     */
    if( rx_assembled_len >= 2 )
    {
        uint8_t cmd_length = rx_assembly[1];
        size_t  frame_len  = ( size_t ) cmd_length + 2;

        if( rx_assembled_len >= frame_len )
        {
            /* Pre-fill with 0xFF to emulate the DMA-based reception on embedded */
            memset( modem_received_buff, 0xFF, HW_MODEM_RX_BUFF_MAX_LENGTH );
            memcpy( modem_received_buff, rx_assembly, frame_len );

            /* Bridge emulation: Python sends frames without CRC; the HW bridge
             * normally computes and appends it, so we replicate that here.
             */
            uint8_t crc = 0;
            for( size_t i = 0; i < frame_len; i++ )
            {
                crc ^= modem_received_buff[i];
            }
            modem_received_buff[frame_len] = crc;

            /* Keep any bytes beyond this frame (a second, already-queued
             * command) for the next call instead of discarding them.
             */
            size_t leftover = rx_assembled_len - frame_len;
            if( leftover > 0 )
            {
                memmove( rx_assembly, &rx_assembly[frame_len], leftover );
            }
            rx_assembled_len = leftover;

            hw_cmd_available = true;
            return true;
        }
    }

    return false;
}

bool hw_modem_is_low_power_ok( void )
{
    return true;
}

void hw_modem_unset_event_pin( void )
{
}

void hw_modem_disable_irq( void )
{
}

void hw_modem_set_tx_power_offset( const void* context, uint8_t tx_pwr_offset_db )
{
    ( void ) context;
    radio_utilities_set_tx_power_offset( tx_pwr_offset_db );
}

uint8_t hw_modem_get_tx_power_offset( const void* context )
{
    ( void ) context;
    return radio_utilities_get_tx_power_offset( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 * -----------------------------------------------------------------------------
 */

static void hw_modem_event_handler( void )
{
    SMTC_HAL_TRACE_MSG_COLOR( "Event available\n", HAL_DBG_TRACE_COLOR_BLUE );
}

static void hw_modem_pty_cleanup( void )
{
    if( pty_master_fd >= 0 )
    {
        close( pty_master_fd );
        pty_master_fd = -1;
    }
    if( pty_slave_keepalive_fd >= 0 )
    {
        close( pty_slave_keepalive_fd );
        pty_slave_keepalive_fd = -1;
    }
    unlink( pty_symlink_path );
}

static void hw_modem_signal_handler( int signum )
{
    ( void ) signum;
    exit( 0 );
}

static void* pty_watch_thread_func( void* arg )
{
    ( void ) arg;

    /* Block SIGUSR2 in this thread: the wake is sent thread-directed via
     * pthread_kill(main_tid, SIGUSR2) by hal_mcu_wake_main_thread(), so it
     * always reaches the main thread on its own. This guarantees a
     * process-directed SIGUSR2 can never interrupt this thread's poll().
     */
    sigset_t block_mask;
    sigemptyset( &block_mask );
    sigaddset( &block_mask, SIGUSR2 );
    pthread_sigmask( SIG_BLOCK, &block_mask, NULL );

    while( pty_watch_thread_running )
    {
        struct pollfd pfd;
        pfd.fd     = pty_master_fd;
        pfd.events = POLLIN;

        /* Blocks indefinitely until the PTY has data. pty_slave_keepalive_fd
         * keeps the slave side permanently open, so this never sees a
         * spurious POLLHUP (which would otherwise make poll() return
         * immediately forever, regardless of timeout, and busy-spin this
         * thread) just because no client is currently connected.
         */
        int ret = poll( &pfd, 1, -1 );
        if( ret > 0 && ( pfd.revents & POLLIN ) )
        {
            /* Just wake the main loop - hw_modem_is_a_cmd_available() (running
             * on the main thread) does the actual read()/parsing, so all PTY
             * buffer manipulation stays single-threaded.
             */
            hal_mcu_wake_main_thread( );

            /* Avoid spinning: wait for the main loop to actually drain the
             * PTY before polling again.
             */
            usleep( 1000 );
        }
    }

    return NULL;
}

/* --- EOF ------------------------------------------------------------------ */
