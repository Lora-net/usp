/**
 * @file      smtc_hal_flash.h
 *
 * @brief     Flash/Storage Hardware Abstraction Layer definition for Linux
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
#ifndef __SMTC_HAL_FLASH_H__
#define __SMTC_HAL_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>  // C99 types

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Reads a buffer from flash
 *
 * @param [in] addr   Address to read from
 * @param [out] buffer Buffer to store read data
 * @param [in] size   Number of bytes to read
 */
void hal_flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size );

/**
 * @brief Writes a buffer to flash
 *
 * @param [in] addr   Address to write to
 * @param [in] buffer Buffer containing data to write
 * @param [in] size   Number of bytes to write
 */
void hal_flash_write_buffer( uint32_t addr, const uint8_t* buffer, uint32_t size );

/**
 * @brief Erases flash pages
 *
 * @param [in] addr     Starting address
 * @param [in] nb_page  Number of pages to erase
 */
void hal_flash_erase_page( uint32_t addr, uint8_t nb_page );

/**
 * @brief Gets flash page size
 *
 * @return uint32_t Page size in bytes
 */
uint32_t hal_flash_get_page_size( void );

/**
 * @brief Reads a flash page, modifies it, and writes it back
 *
 * For Linux file-based storage, this simply writes the buffer directly
 * since files don't require page erase before write like real flash.
 *
 * @param [in] addr   Address to write to
 * @param [in] buffer Buffer containing data to write
 * @param [in] size   Number of bytes to write
 */
void hal_flash_read_modify_write( uint32_t addr, const uint8_t* buffer, uint32_t size );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_FLASH_H__

/* --- EOF ------------------------------------------------------------------ */
