/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa concentrator HAL common auxiliary functions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/

#ifndef _LORAGW_AUX_H
#define _LORAGW_AUX_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>  /* C99 types */
#include <stdbool.h> /* bool type */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define DEBUG_PERF 0 /* Debug timing performances: level [0..4] */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )

/**
@brief Get a particular bit value from a byte
@param b [in]   Any byte from which we want a bit value
@param p [in]   Position of the bit in the byte [0..7]
@param n [in]   Number of bits we want to get
@return The value corresponding the requested bits
*/
#define TAKE_N_BITS_FROM( b, p, n ) ( ( ( b ) >> ( p ) ) & ( ( 1 << ( n ) ) - 1 ) )

/**
@brief Substract struct timeval values
@param a [in]   struct timeval a
@param b [in]   struct timeval b
@param b [out]  struct timeval resulting from (a - b)
*/
#define TIMER_SUB( a, b, result )                              \
    do                                                         \
    {                                                          \
        ( result )->tv_sec  = ( a )->tv_sec - ( b )->tv_sec;   \
        ( result )->tv_usec = ( a )->tv_usec - ( b )->tv_usec; \
        if( ( result )->tv_usec < 0 )                          \
        {                                                      \
            --( result )->tv_sec;                              \
            ( result )->tv_usec += 1000000;                    \
        }                                                      \
    } while( 0 )

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Wait for a certain time (millisecond accuracy)
@param t number of milliseconds to wait.
*/
void wait_ms( unsigned long t );

/**
@brief Wait for a certain time (microsencond accuracy)
@param t number of microseconds to wait.
*/
void wait_us( unsigned long t );

#endif

/* --- EOF ------------------------------------------------------------------ */
