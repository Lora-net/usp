/**
 * @file      smtc_hal_rng.c
 *
 * @brief     Random Number Generator Hardware Abstraction Layer implementation for RA0E2
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
 *
 * Implementation Notes:
 * ---------------------
 * Uses the RA0E2 hardware TRNG (True Random Number Generator) peripheral.
 * Polling mode is used - each call generates a fresh 32-bit random seed.
 *
 * TRNG Processing Flow (from datasheet):
 * 1. Clear MSTPCRC.MSTPC28 to cancel module-stop state
 * 2. Wait for PCLKB × 6 cycles
 * 3. Set TRNGSCR0.SGCEN = 1 to enable TRNG
 * 4. Set TRNGSCR0.SGSTART = 1 to start seed generation
 * 5. Poll TRNGSCR0.RDRDY until = 1
 * 6. Read TRNGSDR 4 times for 32-bit seed
 * 7. Set TRNGSCR0.SGCEN = 0 and SGSTART = 0 to stop
 * 8. Set MSTPCRC.MSTPC28 = 1 to enter module-stop state
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "smtc_hal_rng.h"
#include "bsp_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/* TRNG Module Stop bit in MSTPCRC */
#define TRNG_MSTP_BIT ( 1UL << 28 )

/* TRNGSCR0 bit definitions */
#define TRNGSCR0_SGSTART ( 1U << 2 ) /* SEED Generation Start */
#define TRNGSCR0_SGCEN ( 1U << 3 )   /* SEED Generation Circuit Enable */
#define TRNGSCR0_RDRDY ( 1U << 7 )   /* Read Ready flag */

/* Timeout for TRNG operations (in loop iterations) */
#define TRNG_TIMEOUT 100000U

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

uint32_t hal_rng_get_random( void )
{
    uint32_t random_value = 0;
    uint32_t timeout;

    /* Step 1: Cancel module-stop state for TRNG (clear MSTPC28) */
    R_MSTP->MSTPCRC &= ~TRNG_MSTP_BIT;

    /* Step 2: Wait for PCLKB × 6 cycles (short delay) */
    for( volatile int i = 0; i < 10; i++ )
    {
        __NOP( );
    }

    /* Step 3: Enable TRNG (SGCEN = 1) */
    R_TRNG->TRNGSCR0 = TRNGSCR0_SGCEN;

    /* Step 4: Disable interrupt (polling mode) */
    R_TRNG->TRNGSCR1 = 0;

    /* Step 5: Start seed generation (SGSTART = 1, keep SGCEN = 1) */
    R_TRNG->TRNGSCR0 = TRNGSCR0_SGCEN | TRNGSCR0_SGSTART;

    /* Step 6: Poll for RDRDY = 1 */
    timeout = TRNG_TIMEOUT;
    while( ( R_TRNG->TRNGSCR0 & TRNGSCR0_RDRDY ) == 0 )
    {
        if( --timeout == 0 )
        {
            /* Timeout - disable and return 0 */
            goto cleanup;
        }
    }

    /* Step 7: Read TRNGSDR 4 times for 32-bit value */
    random_value = ( uint32_t ) R_TRNG->TRNGSDR;
    random_value |= ( uint32_t ) R_TRNG->TRNGSDR << 8;
    random_value |= ( uint32_t ) R_TRNG->TRNGSDR << 16;
    random_value |= ( uint32_t ) R_TRNG->TRNGSDR << 24;

cleanup:
    /* Step 8: Stop TRNG (SGCEN = 0, SGSTART = 0) */
    R_TRNG->TRNGSCR0 = 0;

    /* Step 9: Enter module-stop state (set MSTPC28) */
    R_MSTP->MSTPCRC |= TRNG_MSTP_BIT;

    return random_value;
}

uint32_t hal_rng_get_random_in_range( const uint32_t val_1, const uint32_t val_2 )
{
    if( val_1 <= val_2 )
    {
        return ( uint32_t ) ( ( hal_rng_get_random( ) % ( val_2 - val_1 + 1 ) ) + val_1 );
    }
    else
    {
        return ( uint32_t ) ( ( hal_rng_get_random( ) % ( val_1 - val_2 + 1 ) ) + val_2 );
    }
}

int32_t hal_rng_get_signed_random_in_range( const int32_t val_1, const int32_t val_2 )
{
    uint32_t tmp_range = 0;

    if( val_1 <= val_2 )
    {
        tmp_range = ( val_2 - val_1 );
        return ( int32_t ) ( ( val_1 + hal_rng_get_random_in_range( 0, tmp_range ) ) );
    }
    else
    {
        tmp_range = ( val_1 - val_2 );
        return ( int32_t ) ( ( val_2 + hal_rng_get_random_in_range( 0, tmp_range ) ) );
    }
}

/* --- EOF ------------------------------------------------------------------ */
