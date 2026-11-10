/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa concentrator HAL auxiliary functions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* Fix an issue between POSIX and C99 - must be before any includes */
#if __STDC_VERSION__ >= 199901L
#define _XOPEN_SOURCE 600
#else
#define _XOPEN_SOURCE 500
#endif

/* Own header */
#include "loragw_aux.h"

/* Standard C library */
#include <stdio.h>
#include <time.h>

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#if DEBUG_AUX == 1
#define DEBUG_MSG( str ) fprintf( stdout, str )
#define DEBUG_PRINTF( fmt, ... ) fprintf( stdout, "%s:%d: " fmt, __func__, __LINE__, __VA_ARGS__ )
#else
#define DEBUG_MSG( str )
#define DEBUG_PRINTF( fmt, ... )
#endif

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

void wait_us( unsigned long delay_us )
{
    struct timespec dly;
    struct timespec rem;

    dly.tv_sec  = delay_us / 1000000;
    dly.tv_nsec = ( delay_us % 1000000 ) * 1000;

    DEBUG_PRINTF( "NOTE dly: %ld sec %ld ns\n", dly.tv_sec, dly.tv_nsec );

    while( ( dly.tv_sec > 0 ) || ( dly.tv_nsec > 1000 ) )
    {
        /*
        rem is set ONLY if clock_nanosleep is interrupted (eg. by a signal).
        Must be zeroed each time or will get into an infinite loop after an IT.
        */
        rem.tv_sec  = 0;
        rem.tv_nsec = 0;
        clock_nanosleep( CLOCK_MONOTONIC, 0, &dly, &rem );
        DEBUG_PRINTF( "NOTE remain: %ld sec %ld ns\n", rem.tv_sec, rem.tv_nsec );
        dly = rem;
    }

    return;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void wait_ms( unsigned long delay_ms )
{
    struct timespec dly;
    struct timespec rem;

    dly.tv_sec  = delay_ms / 1000;
    dly.tv_nsec = ( ( long ) delay_ms % 1000 ) * 1000000;

    DEBUG_PRINTF( "NOTE dly: %ld sec %ld ns\n", dly.tv_sec, dly.tv_nsec );

    if( ( dly.tv_sec > 0 ) || ( ( dly.tv_sec == 0 ) && ( dly.tv_nsec > 100000 ) ) )
    {
        clock_nanosleep( CLOCK_MONOTONIC, 0, &dly, &rem );
        DEBUG_PRINTF( "NOTE remain: %ld sec %ld ns\n", rem.tv_sec, rem.tv_nsec );
    }
    return;
}

/* --- EOF ------------------------------------------------------------------ */
