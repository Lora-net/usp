/**
 * @file      hw_modem_serial.c
 *
 * @brief     Single-UART hw_modem transport (no external bridge)
 *
 * Host sends [CMD][LEN][PAYLOAD] without CRC (same as hw_modem_linux.c).
 * Responses include a 2-byte bridge header for compatibility with rac_modem_api.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "hw_modem.h"
#include "cmd_parser.h"
#include "radio_utilities.h"
#include "smtc_modem_utilities.h"
#include "smtc_hal_uart.h"
#include "smtc_hal_trace.h"
#include "smtc_hal_dbg_trace.h"

#define HW_MODEM_RX_BUFF_MAX_LENGTH 261
#define BRIDGE_HEADER_BYTE0 0x01
#define BRIDGE_HEADER_BYTE1 0x00
#define BRIDGE_HEADER_SIZE 2

/*
 * Reassembly ring buffer, filled directly by the trace UART's circular DMA (see hal_uart_rx_start()).
 * The HAL is the producer (write cursor = hal_uart_rx_write_index()); this file is the consumer.
 * Must stay strictly larger than the largest hw_modem frame (HW_MODEM_RX_BUFF_MAX_LENGTH):
 * serial_poll_rx() waits for a full frame to be available before consuming any of it, otherwise the
 * circular DMA could wrap and overwrite the frame's own bytes before they are read. Sized to
 * NHM_REASSEMBLY_BUFFER_SIZE (cmd_parser.h) for headroom.
 */
static uint8_t  rx_ring[NHM_REASSEMBLY_BUFFER_SIZE];
static uint16_t rx_ring_tail;  // Read cursor, advanced only by the main loop (the HAL/DMA is the producer)

/**
 * @brief Message type multiplexed with the framing below, consumed by python_bridge.py's demux
 */
typedef enum unified_uart_message_type_e
{
    UNIFIED_UART_MSG_TRACE,
    UNIFIED_UART_MSG_DATA,
} unified_uart_message_type_t;

static uint8_t       modem_response_buff[HW_MODEM_RX_BUFF_MAX_LENGTH];
static uint8_t       modem_received_buff[HW_MODEM_RX_BUFF_MAX_LENGTH];
static size_t        response_length;
static volatile bool hw_cmd_available;
static bool          stay_awake_once;

static void hw_modem_event_handler( void );
static void serial_poll_rx( void );

/**
 * @brief Multiplex trace and data frames on the single UART: [type]['T'/'D'][len LO][len HI][payload]
 *
 * The 3-byte header lets python_bridge.py demux this single physical wire back into the
 * separate trace/data sockets it exposes to host tooling.
 */
static void unified_uart_tx( uint8_t* buff, uint16_t len, unified_uart_message_type_t type )
{
    uint8_t header[3];

    header[0] = ( type == UNIFIED_UART_MSG_TRACE ) ? ( uint8_t ) 'T' : ( uint8_t ) 'D';
    header[1] = ( uint8_t ) ( len & 0xFF );
    header[2] = ( uint8_t ) ( ( len >> 8 ) & 0xFF );

    trace_uart_tx( header, sizeof( header ) );
    if( len > 0 )
    {
        trace_uart_tx( buff, len );
    }
}

/**
 * @brief Adapter matching hal_trace_tx_fn_t, registered with hal_trace_set_tx_function()
 */
static void hw_modem_trace_tx( uint8_t* buff, uint16_t len )
{
    unified_uart_tx( buff, len, UNIFIED_UART_MSG_TRACE );
}

/**
 * @brief Number of unread bytes currently sitting in the reassembly ring buffer
 *
 * The producer cursor is the HAL/DMA write index; the consumer cursor is rx_ring_tail.
 */
static uint16_t hw_modem_rx_available( void )
{
    uint16_t head = hal_uart_rx_write_index( );
    return ( head >= rx_ring_tail ) ? ( uint16_t ) ( head - rx_ring_tail )
                                    : ( uint16_t ) ( sizeof( rx_ring ) - rx_ring_tail + head );
}

/**
 * @brief Look at a byte in the ring buffer without consuming it
 */
static bool hw_modem_rx_peek( uint16_t offset, uint8_t* byte )
{
    if( offset >= hw_modem_rx_available( ) )
    {
        return false;
    }
    *byte = rx_ring[( rx_ring_tail + offset ) % sizeof( rx_ring )];
    return true;
}

/**
 * @brief Pop the oldest unread byte from the ring buffer
 */
static bool hw_modem_rx_get_byte( uint8_t* byte )
{
    if( hw_modem_rx_available( ) == 0 )
    {
        return false;
    }
    *byte        = rx_ring[rx_ring_tail];
    rx_ring_tail = ( uint16_t ) ( ( rx_ring_tail + 1 ) % sizeof( rx_ring ) );
    return true;
}

static void serial_poll_rx( void )
{
    uint16_t avail = hw_modem_rx_available( );
    uint8_t  len;

    if( avail < 2 )
    {
        return;
    }

    if( !hw_modem_rx_peek( 1, &len ) )
    {
        return;
    }

    if( avail < ( uint16_t ) ( 2 + len ) )
    {
        return;
    }

    for( uint16_t i = 0; i < 2 + len; i++ )
    {
        hw_modem_rx_get_byte( &modem_received_buff[i] );
    }

    uint8_t crc = 0;
    for( uint16_t i = 0; i < 2 + len; i++ )
    {
        crc ^= modem_received_buff[i];
    }
    modem_received_buff[2 + len] = crc;

    hw_cmd_available = true;
}

int hw_modem_init( void )
{
    memset( modem_response_buff, 0, HW_MODEM_RX_BUFF_MAX_LENGTH );
    memset( modem_received_buff, 0xFF, HW_MODEM_RX_BUFF_MAX_LENGTH );
    hw_cmd_available = false;
    stay_awake_once  = false;
    rx_ring_tail     = 0;

    hal_uart_rx_start( rx_ring, sizeof( rx_ring ) );
    hal_trace_set_tx_function( hw_modem_trace_tx );

    smtc_modem_init( &hw_modem_event_handler );

#if defined( PERF_TEST_ENABLED )
    SMTC_HAL_TRACE_WARNING( "HARDWARE MODEM RUNNING PERF TEST MODE" );
#endif
    return 0;
}

void hw_modem_process_cmd( void )
{
    uint8_t              cmd_length = 0xFF;
    cmd_response_t       output;
    cmd_input_t          input;
    cmd_serial_rc_code_t rc_code;
    uint8_t              crc = 0;

    if( modem_received_buff[0] < 0xFF )
    {
        cmd_length = modem_received_buff[1];

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
            SMTC_HAL_TRACE_ERROR( "Cmd with bad crc %x / %x", calculated_crc, cmd_crc );
        }
        else if( ( modem_received_buff[cmd_length + 3] != 0xFF ) && ( cmd_length != 0xFF ) )
        {
            rc_code         = CMD_RC_FRAME_ERROR;
            response_length = 0;
            SMTC_HAL_TRACE_WARNING( " Extra data after the command" );
        }
        else
        {
            SMTC_HAL_TRACE_ARRAY( "Cmd input uart", modem_received_buff, cmd_length + 2 );
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

        SMTC_HAL_TRACE_ARRAY( "Cmd output on uart", modem_response_buff, response_length + 2 );
        hw_cmd_available = false;

        crc = 0;
        for( int i = 0; i < ( int ) ( response_length + 2 ); i++ )
        {
            crc = crc ^ modem_response_buff[i];
        }
        modem_response_buff[response_length + 2] = crc;

        {
            size_t  modem_len = response_length + 3;
            uint8_t bridge_response[BRIDGE_HEADER_SIZE + HW_MODEM_RX_BUFF_MAX_LENGTH];

            bridge_response[0] = BRIDGE_HEADER_BYTE0;
            bridge_response[1] = BRIDGE_HEADER_BYTE1;
            memcpy( &bridge_response[BRIDGE_HEADER_SIZE], modem_response_buff, modem_len );
            unified_uart_tx( bridge_response, BRIDGE_HEADER_SIZE + modem_len, UNIFIED_UART_MSG_DATA );
        }
    }
    else
    {
        hw_cmd_available = false;
    }

    stay_awake_once = true;
    memset( modem_received_buff, 0xFF, HW_MODEM_RX_BUFF_MAX_LENGTH );
}

bool hw_modem_is_a_cmd_available( void )
{
    if( !hw_cmd_available )
    {
        serial_poll_rx( );
    }
    return hw_cmd_available;
}

bool hw_modem_is_low_power_ok( void )
{
    if( hw_cmd_available || ( hw_modem_rx_available( ) > 0 ) )
    {
        return false;
    }

    if( stay_awake_once )
    {
        stay_awake_once = false;
        return false;
    }

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

static void hw_modem_event_handler( void )
{
    SMTC_HAL_TRACE_MSG_COLOR( "Event available\n", HAL_DBG_TRACE_COLOR_BLUE );
}
