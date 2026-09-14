/**
 * @file      smtc_hal_flash.c
 *
 * @brief     Flash/Storage Hardware Abstraction Layer implementation for Linux
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
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/stat.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define FLASH_PAGE_SIZE 2048
#define FLASH_TOTAL_SIZE ( 256 * 1024 )  // 256KB total flash size
#define FLASH_FILE_NAME "flash_context.bin"
#define FLASH_ERASED_VALUE 0xFF  // Flash erased state

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Ensure flash file exists and is properly initialized
 *
 * @return true if file is ready, false on error
 */
static bool hal_flash_ensure_file_exists( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size )
{
    if( buffer == NULL || size == 0 )
    {
        return;
    }

    // Ensure address is within bounds
    if( addr + size > FLASH_TOTAL_SIZE )
    {
        memset( buffer, FLASH_ERASED_VALUE, size );
        return;
    }

    // Ensure flash file exists
    if( !hal_flash_ensure_file_exists( ) )
    {
        memset( buffer, FLASH_ERASED_VALUE, size );
        return;
    }

    // Open file for reading
    FILE* fp = fopen( FLASH_FILE_NAME, "rb" );
    if( fp == NULL )
    {
        memset( buffer, FLASH_ERASED_VALUE, size );
        return;
    }

    // Seek to address and read
    if( fseek( fp, addr, SEEK_SET ) != 0 )
    {
        memset( buffer, FLASH_ERASED_VALUE, size );
        fclose( fp );
        return;
    }

    size_t bytes_read = fread( buffer, 1, size, fp );
    if( bytes_read < size )
    {
        // Fill remaining with erased value if read was short
        memset( buffer + bytes_read, FLASH_ERASED_VALUE, size - bytes_read );
    }

    fclose( fp );
}

void hal_flash_write_buffer( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    if( buffer == NULL || size == 0 )
    {
        return;
    }

    // Ensure address is within bounds
    if( addr + size > FLASH_TOTAL_SIZE )
    {
        return;
    }

    // Ensure flash file exists
    if( !hal_flash_ensure_file_exists( ) )
    {
        return;
    }

    // Open file for reading and writing
    FILE* fp = fopen( FLASH_FILE_NAME, "r+b" );
    if( fp == NULL )
    {
        return;
    }

    // Seek to address and write
    if( fseek( fp, addr, SEEK_SET ) != 0 )
    {
        fclose( fp );
        return;
    }

    fwrite( buffer, 1, size, fp );
    fclose( fp );
}

void hal_flash_erase_page( uint32_t addr, uint8_t nb_page )
{
    if( nb_page == 0 )
    {
        return;
    }

    uint32_t erase_size = nb_page * FLASH_PAGE_SIZE;

    // Ensure address is within bounds
    if( addr + erase_size > FLASH_TOTAL_SIZE )
    {
        return;
    }

    // Ensure flash file exists
    if( !hal_flash_ensure_file_exists( ) )
    {
        return;
    }

    // Create buffer filled with erased value
    uint8_t erase_buffer[FLASH_PAGE_SIZE];
    memset( erase_buffer, FLASH_ERASED_VALUE, FLASH_PAGE_SIZE );

    // Open file for reading and writing
    FILE* fp = fopen( FLASH_FILE_NAME, "r+b" );
    if( fp == NULL )
    {
        return;
    }

    // Seek to address
    if( fseek( fp, addr, SEEK_SET ) != 0 )
    {
        fclose( fp );
        return;
    }

    // Erase page by page
    for( uint8_t i = 0; i < nb_page; i++ )
    {
        fwrite( erase_buffer, 1, FLASH_PAGE_SIZE, fp );
    }

    fclose( fp );
}

uint32_t hal_flash_get_page_size( void )
{
    return FLASH_PAGE_SIZE;
}

void hal_flash_read_modify_write( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    // For Linux file-based storage, we can simply write the buffer directly
    // since files don't require page erase before write like real flash.
    hal_flash_write_buffer( addr, buffer, size );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool hal_flash_ensure_file_exists( void )
{
    // Check if file exists and has correct size
    struct stat st;
    bool        file_exists = ( stat( FLASH_FILE_NAME, &st ) == 0 );

    if( file_exists && st.st_size == FLASH_TOTAL_SIZE )
    {
        // File exists and has correct size
        return true;
    }

    // Create or resize file
    FILE* fp = fopen( FLASH_FILE_NAME, "wb" );
    if( fp == NULL )
    {
        return false;
    }

    // Write erased flash (all 0xFF)
    uint8_t erase_buffer[FLASH_PAGE_SIZE];
    memset( erase_buffer, FLASH_ERASED_VALUE, FLASH_PAGE_SIZE );

    uint32_t pages_to_write = FLASH_TOTAL_SIZE / FLASH_PAGE_SIZE;
    for( uint32_t i = 0; i < pages_to_write; i++ )
    {
        if( fwrite( erase_buffer, 1, FLASH_PAGE_SIZE, fp ) != FLASH_PAGE_SIZE )
        {
            fclose( fp );
            return false;
        }
    }

    fclose( fp );
    return true;
}

/* --- EOF ------------------------------------------------------------------ */
