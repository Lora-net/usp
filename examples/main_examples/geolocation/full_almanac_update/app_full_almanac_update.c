/**
 * @file      app_full_almanac_update.c
 *
 * @brief     LR11xx almanac update common functions
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
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

#include <time.h>
#include <string.h>

#include "smtc_modem_api.h"

#include "app_full_almanac_update.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_modem_geolocation_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Supported LR11XX radio firmware
 */
#define LR1110_FW_VERSION 0x0401
#define LR1120_FW_VERSION 0x0201
#define OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH ( 315964800 )

#define TIME_BUFFER_SIZE ( 80 )

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bool check_lr11xx_fw_version( const void* context )
{
    lr11xx_status_t         status;
    lr11xx_system_version_t lr11xx_fw_version;

    /* suspend modem to get access to the radio */
    smtc_modem_suspend_radio_communications( true );

    status = lr11xx_system_get_version( context, &lr11xx_fw_version );
    if( status != LR11XX_STATUS_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to get LR11XX firmware version\n" );
        smtc_modem_suspend_radio_communications( false );
        return false;
    }

    if( ( lr11xx_fw_version.type == LR11XX_SYSTEM_VERSION_TYPE_LR1110 ) &&
        ( lr11xx_fw_version.fw < LR1110_FW_VERSION ) )
    {
        SMTC_HAL_TRACE_ERROR( "Wrong LR1110 firmware version, expected 0x%04X, got 0x%04X\n", LR1110_FW_VERSION,
                              lr11xx_fw_version.fw );
        smtc_modem_suspend_radio_communications( false );
        return false;
    }
    if( ( lr11xx_fw_version.type == LR11XX_SYSTEM_VERSION_TYPE_LR1120 ) &&
        ( lr11xx_fw_version.fw < LR1120_FW_VERSION ) )
    {
        SMTC_HAL_TRACE_ERROR( "Wrong LR1120 firmware version, expected 0x%04X, got 0x%04X\n", LR1120_FW_VERSION,
                              lr11xx_fw_version.fw );
        smtc_modem_suspend_radio_communications( false );
        return false;
    }

    /* release radio to the modem */
    smtc_modem_suspend_radio_communications( false );
    SMTC_HAL_TRACE_INFO( "LR11XX FW: 0x%04X, type: 0x%02X\n", lr11xx_fw_version.fw, lr11xx_fw_version.type );

    return true;
}

bool almanac_full_update( uint8_t stack_id )
{
    smtc_modem_return_code_t rc;
    time_t                   almanac_date;
    uint16_t                 almanac_date_raw;
    bool                     ret = false;

    rc = smtc_modem_almanac_full_update( stack_id, full_almanac, sizeof( full_almanac ) );
    if( rc == SMTC_MODEM_RC_OK )
    {
        /* Convert raw almanac date to epoch time */
        almanac_date_raw = ( uint16_t ) ( ( full_almanac[2] << 8 ) | full_almanac[1] );
        almanac_date     = ( OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH + 24 * 3600 * ( 2048 * 7 + almanac_date_raw ) );
        /* Convert epoch time to human readbale format */
        char             buf[TIME_BUFFER_SIZE];
        const struct tm* time = localtime( &almanac_date );
        strftime( buf, TIME_BUFFER_SIZE, "%a %Y-%m-%d %H:%M:%S %Z", time );

        SMTC_HAL_TRACE_INFO( "Success: almanac source date %s\n", buf );
        ret = true;
    }
    else
    {
        SMTC_HAL_TRACE_ERROR( "Failed\n" );
    }

    return ret;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */
