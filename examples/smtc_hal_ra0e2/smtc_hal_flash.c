/**
 * @file      smtc_hal_flash.c
 *
 * @brief     Flash Hardware Abstraction Layer implementation for RA0E2
 *
 * RA0E2 Data Flash Specifications:
 * - Start address: 0x40100000
 * - Size: 2KB (2048 bytes)
 * - Block size (erase unit): 256 bytes
 * - Write size (minimum): 1 byte
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

#include "smtc_hal_flash.h"
#include "hal_data.h"
#include "bsp_api.h"
#include <string.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#ifndef SUCCESS
#define SUCCESS 1
#endif

#ifndef FAIL
#define FAIL 0
#endif

/* RA0E2 Data Flash parameters (from bsp_feature.h) */
#define RA0E2_DATA_FLASH_START 0x40100000UL
#define RA0E2_DATA_FLASH_SIZE 0x800UL       /* 2KB */
#define RA0E2_DATA_FLASH_BLOCK_SIZE 0x100UL /* 256 bytes per block (erase unit) */
#define RA0E2_DATA_FLASH_END ( RA0E2_DATA_FLASH_START + RA0E2_DATA_FLASH_SIZE )

/* Maximum retry count for flash operations */
#define FLASH_OPERATION_MAX_RETRY 4

/* Flash Ready Interrupt - fixed at IRQ 11 on RA0E2 (ICU_EVENT_FCU_FRDYI)
 * Hardware sets pending bit on flash operation completion even in blocking mode.
 * Not allocated in vector table, but must be cleared to prevent spurious wakeups.
 */
#define FCU_FRDYI_IRQn ( ( IRQn_Type ) 11 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/* Track if flash driver has been opened */
static bool flash_initialized = false;

#if defined( USE_FLASH_READ_MODIFY_WRITE ) || defined( MULTISTACK )
static uint8_t copy_page[RA0E2_DATA_FLASH_BLOCK_SIZE];
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Ensure flash driver is initialized
 * @return true if initialized successfully, false otherwise
 */
static bool flash_ensure_init( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

uint16_t hal_flash_get_page_size( void )
{
    return RA0E2_DATA_FLASH_BLOCK_SIZE;
}

uint8_t hal_flash_erase_page( uint32_t addr, uint8_t nb_page )
{
    fsp_err_t err;
    uint8_t   retry_count = 0;

    /* Validate address is within data flash region */
    if( addr < RA0E2_DATA_FLASH_START || addr >= RA0E2_DATA_FLASH_END )
    {
        return FAIL;
    }

    /* Ensure flash is initialized */
    if( !flash_ensure_init( ) )
    {
        return FAIL;
    }

    /* Erase with retry */
    do
    {
        err = R_FLASH_LP_Erase( &g_flash0_ctrl, addr, nb_page );
        retry_count++;
    } while( ( err != FSP_SUCCESS ) && ( retry_count < FLASH_OPERATION_MAX_RETRY ) );

    if( err != FSP_SUCCESS )
    {
        return FAIL;
    }

    /* Clear flash ready interrupt pending bit (set by hardware on completion) */
    NVIC_ClearPendingIRQ( FCU_FRDYI_IRQn );

    return SUCCESS;
}

uint32_t hal_flash_write_buffer( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    fsp_err_t err;
    uint8_t   retry_count = 0;

    /* Validate address is within data flash region */
    if( addr < RA0E2_DATA_FLASH_START || ( addr + size ) > RA0E2_DATA_FLASH_END )
    {
        return FAIL;
    }

    /* Ensure flash is initialized */
    if( !flash_ensure_init( ) )
    {
        return FAIL;
    }

    /* RA0E2 data flash can write 1 byte at a time, so no alignment needed.
     * However, FSP may optimize writes - pass buffer directly.
     */
    do
    {
        err = R_FLASH_LP_Write( &g_flash0_ctrl, ( uint32_t ) buffer, addr, size );
        retry_count++;
    } while( ( err != FSP_SUCCESS ) && ( retry_count < FLASH_OPERATION_MAX_RETRY ) );

    if( err != FSP_SUCCESS )
    {
        return FAIL;
    }

    /* Clear flash ready interrupt pending bit (set by hardware on completion) */
    NVIC_ClearPendingIRQ( FCU_FRDYI_IRQn );

    return size;
}

void hal_flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size )
{
    /* Data flash is memory-mapped - direct read is supported */
    memcpy( buffer, ( const void* ) addr, size );
}

#if defined( USE_FLASH_READ_MODIFY_WRITE ) || defined( MULTISTACK )
void hal_flash_read_modify_write( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    uint32_t remaining_size = size;

    do
    {
        memset( copy_page, 0xFF, sizeof( copy_page ) );

        /* Get the block number */
        uint32_t block_num = ( addr - RA0E2_DATA_FLASH_START ) / RA0E2_DATA_FLASH_BLOCK_SIZE;

        /* Get start address of this block */
        uint32_t start_addr_block = RA0E2_DATA_FLASH_START + ( RA0E2_DATA_FLASH_BLOCK_SIZE * block_num );

        /* Read data from this block */
        hal_flash_read_buffer( start_addr_block, copy_page, RA0E2_DATA_FLASH_BLOCK_SIZE );

        /* Compute the index where data need to be updated in RAM copy */
        uint32_t index = ( addr - start_addr_block ) % RA0E2_DATA_FLASH_BLOCK_SIZE;

        /* Compute the size of the data to copy without overflow to the next block */
        uint32_t cpy_size = MIN( remaining_size, RA0E2_DATA_FLASH_BLOCK_SIZE - index );
        memcpy( &copy_page[index], &buffer[size - remaining_size], cpy_size );

        /* Erase the block */
        hal_flash_erase_page( start_addr_block, 1 );

        /* Write the RAM buffer back to flash */
        hal_flash_write_buffer( start_addr_block, copy_page, RA0E2_DATA_FLASH_BLOCK_SIZE );

        /* Increment address to the next block */
        addr += RA0E2_DATA_FLASH_BLOCK_SIZE - index;

        /* Reduce the remaining data */
        remaining_size -= cpy_size;

    } while( remaining_size != 0 );
}
#endif /* USE_FLASH_READ_MODIFY_WRITE || MULTISTACK */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool flash_ensure_init( void )
{
    fsp_err_t err;

    if( flash_initialized )
    {
        return true;
    }

    /* Open the flash driver */
    err = R_FLASH_LP_Open( &g_flash0_ctrl, &g_flash0_cfg );
    if( err == FSP_SUCCESS )
    {
        flash_initialized = true;
        return true;
    }
    else if( err == FSP_ERR_ALREADY_OPEN )
    {
        /* Already initialized (e.g., by another module) */
        flash_initialized = true;
        return true;
    }

    return false;
}

/* --- EOF ------------------------------------------------------------------ */
