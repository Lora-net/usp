/**
 * @file      lorawan_client_conf.c
 *
 * @brief     LoRaWAN client configuration parser implementation
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

#include "lorawan_client_conf.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "parson.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

#define CONF_LOG_INFO( ... ) SMTC_HAL_TRACE_INFO( __VA_ARGS__ )
#define CONF_LOG_WARN( ... ) SMTC_HAL_TRACE_WARNING( __VA_ARGS__ )
#define CONF_LOG_ERROR( ... ) SMTC_HAL_TRACE_ERROR( __VA_ARGS__ )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define CONF_OBJ_NAME "lorawan_client_conf"

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
 * @brief Parse a hex string into a byte array
 *
 * @param[in]  hex_str      Hex string (e.g., "70B3D57ED0074D50")
 * @param[out] bytes        Output byte array
 * @param[in]  expected_len Expected number of bytes
 *
 * @return true if parsing successful, false otherwise
 */
static bool parse_hex_string( const char* hex_str, uint8_t* bytes, size_t expected_len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

const char* lorawan_client_conf_get_path( void )
{
    const char* env_path = getenv( LORAWAN_CLIENT_CONF_ENV_VAR );
    if( env_path != NULL && strlen( env_path ) > 0 )
    {
        return env_path;
    }
    return LORAWAN_CLIENT_CONF_DEFAULT_FILE;
}

bool lorawan_client_conf_load( const char* conf_file, lorawan_client_conf_t* conf )
{
    JSON_Value*  root_val = NULL;
    JSON_Object* conf_obj = NULL;
    JSON_Value*  val      = NULL;
    const char*  str      = NULL;

    if( conf == NULL )
    {
        CONF_LOG_ERROR( "[LW_CONF] NULL configuration pointer\n" );
        return false;
    }

    /* Use default path if not provided */
    if( conf_file == NULL )
    {
        conf_file = lorawan_client_conf_get_path( );
    }

    /* Initialize configuration with defaults */
    memset( conf, 0, sizeof( lorawan_client_conf_t ) );
    conf->enabled               = false;
    conf->region                = SMTC_MODEM_REGION_EU_868;
    conf->keep_alive_interval_s = LORAWAN_CLIENT_DEFAULT_KEEP_ALIVE_S;
    conf->alert_port            = LORAWAN_CLIENT_DEFAULT_ALERT_PORT;

    /* Check if file exists */
    if( access( conf_file, R_OK ) != 0 )
    {
        CONF_LOG_INFO( "[LW_CONF] Configuration file '%s' not found - LoRaWAN client disabled\n", conf_file );
        return false;
    }

    /* Parse JSON file */
    root_val = json_parse_file_with_comments( conf_file );
    if( root_val == NULL )
    {
        CONF_LOG_ERROR( "[LW_CONF] '%s' is not a valid JSON file\n", conf_file );
        return false;
    }

    /* Get the lorawan_client_conf object */
    conf_obj = json_object_get_object( json_value_get_object( root_val ), CONF_OBJ_NAME );
    if( conf_obj == NULL )
    {
        CONF_LOG_ERROR( "[LW_CONF] '%s' does not contain '%s' object\n", conf_file, CONF_OBJ_NAME );
        json_value_free( root_val );
        return false;
    }

    CONF_LOG_INFO( "[LW_CONF] Parsing configuration from '%s'\n", conf_file );

    /*
     * -------------------------------------------------------------------------
     * Parse "enabled" field (optional, default: true if file exists)
     * -------------------------------------------------------------------------
     */
    val = json_object_get_value( conf_obj, "enabled" );
    if( json_value_get_type( val ) == JSONBoolean )
    {
        conf->enabled = ( bool ) json_value_get_boolean( val );
    }
    else
    {
        /* If file exists and has valid config, assume enabled */
        conf->enabled = true;
    }

    if( !conf->enabled )
    {
        CONF_LOG_INFO( "[LW_CONF] LoRaWAN client explicitly disabled in configuration\n" );
        json_value_free( root_val );
        return false;
    }

    /*
     * -------------------------------------------------------------------------
     * Parse "dev_eui" field (required)
     * -------------------------------------------------------------------------
     */
    str = json_object_get_string( conf_obj, "dev_eui" );
    if( str == NULL )
    {
        CONF_LOG_ERROR( "[LW_CONF] Missing required field 'dev_eui'\n" );
        json_value_free( root_val );
        return false;
    }
    if( !parse_hex_string( str, conf->dev_eui, 8 ) )
    {
        CONF_LOG_ERROR( "[LW_CONF] Invalid 'dev_eui' format (expected 16 hex chars)\n" );
        json_value_free( root_val );
        return false;
    }

    /*
     * -------------------------------------------------------------------------
     * Parse "join_eui" field (required)
     * -------------------------------------------------------------------------
     */
    str = json_object_get_string( conf_obj, "join_eui" );
    if( str == NULL )
    {
        CONF_LOG_ERROR( "[LW_CONF] Missing required field 'join_eui'\n" );
        json_value_free( root_val );
        return false;
    }
    if( !parse_hex_string( str, conf->join_eui, 8 ) )
    {
        CONF_LOG_ERROR( "[LW_CONF] Invalid 'join_eui' format (expected 16 hex chars)\n" );
        json_value_free( root_val );
        return false;
    }

    /*
     * -------------------------------------------------------------------------
     * Parse "app_key" field (required)
     * -------------------------------------------------------------------------
     */
    str = json_object_get_string( conf_obj, "app_key" );
    if( str == NULL )
    {
        CONF_LOG_ERROR( "[LW_CONF] Missing required field 'app_key'\n" );
        json_value_free( root_val );
        return false;
    }
    if( !parse_hex_string( str, conf->app_key, 16 ) )
    {
        CONF_LOG_ERROR( "[LW_CONF] Invalid 'app_key' format (expected 32 hex chars)\n" );
        json_value_free( root_val );
        return false;
    }

    /*
     * -------------------------------------------------------------------------
     * Parse "region" field (optional, default: EU868)
     * -------------------------------------------------------------------------
     */
    str = json_object_get_string( conf_obj, "region" );
    if( str != NULL )
    {
        conf->region = lorawan_client_conf_str_to_region( str );
    }

    /*
     * -------------------------------------------------------------------------
     * Parse "keep_alive_interval_s" field (optional)
     * -------------------------------------------------------------------------
     */
    val = json_object_get_value( conf_obj, "keep_alive_interval_s" );
    if( json_value_get_type( val ) == JSONNumber )
    {
        conf->keep_alive_interval_s = ( uint32_t ) json_value_get_number( val );
    }

    /*
     * -------------------------------------------------------------------------
     * Parse "alert_port" field (optional)
     * -------------------------------------------------------------------------
     */
    val = json_object_get_value( conf_obj, "alert_port" );
    if( json_value_get_type( val ) == JSONNumber )
    {
        conf->alert_port = ( uint8_t ) json_value_get_number( val );
    }

    json_value_free( root_val );

    CONF_LOG_INFO( "[LW_CONF] Configuration loaded successfully\n" );

    return true;
}

smtc_modem_region_t lorawan_client_conf_str_to_region( const char* region_str )
{
    if( region_str == NULL )
    {
        return SMTC_MODEM_REGION_EU_868;
    }

    /* Compare region strings (case-insensitive would be nicer, but keep it simple) */
    if( strcmp( region_str, "EU868" ) == 0 || strcmp( region_str, "EU_868" ) == 0 )
    {
        return SMTC_MODEM_REGION_EU_868;
    }
    else if( strcmp( region_str, "US915" ) == 0 || strcmp( region_str, "US_915" ) == 0 )
    {
        return SMTC_MODEM_REGION_US_915;
    }
    else if( strcmp( region_str, "AU915" ) == 0 || strcmp( region_str, "AU_915" ) == 0 )
    {
        return SMTC_MODEM_REGION_AU_915;
    }
    else if( strcmp( region_str, "AS923_GRP1" ) == 0 || strcmp( region_str, "AS923" ) == 0 )
    {
        return SMTC_MODEM_REGION_AS_923_GRP1;
    }
    else if( strcmp( region_str, "AS923_GRP2" ) == 0 )
    {
        return SMTC_MODEM_REGION_AS_923_GRP2;
    }
    else if( strcmp( region_str, "AS923_GRP3" ) == 0 )
    {
        return SMTC_MODEM_REGION_AS_923_GRP3;
    }
    else if( strcmp( region_str, "AS923_GRP4" ) == 0 )
    {
        return SMTC_MODEM_REGION_AS_923_GRP4;
    }
    else if( strcmp( region_str, "KR920" ) == 0 || strcmp( region_str, "KR_920" ) == 0 )
    {
        return SMTC_MODEM_REGION_KR_920;
    }
    else if( strcmp( region_str, "IN865" ) == 0 || strcmp( region_str, "IN_865" ) == 0 )
    {
        return SMTC_MODEM_REGION_IN_865;
    }
    else if( strcmp( region_str, "RU864" ) == 0 || strcmp( region_str, "RU_864" ) == 0 )
    {
        return SMTC_MODEM_REGION_RU_864;
    }
    else if( strcmp( region_str, "CN470" ) == 0 || strcmp( region_str, "CN_470" ) == 0 )
    {
        return SMTC_MODEM_REGION_CN_470;
    }
    else if( strcmp( region_str, "WW2G4" ) == 0 || strcmp( region_str, "WW_2G4" ) == 0 )
    {
        return SMTC_MODEM_REGION_WW_2G4;
    }
    else
    {
        CONF_LOG_WARN( "[LW_CONF] Unknown region '%s', defaulting to EU868\n", region_str );
        return SMTC_MODEM_REGION_EU_868;
    }
}

void lorawan_client_conf_print( const lorawan_client_conf_t* conf )
{
    if( conf == NULL )
    {
        return;
    }

    CONF_LOG_INFO( "[LW_CONF] ========================================\n" );
    CONF_LOG_INFO( "[LW_CONF]   LoRaWAN Client Configuration\n" );
    CONF_LOG_INFO( "[LW_CONF] ========================================\n" );
    CONF_LOG_INFO( "[LW_CONF] Enabled: %s\n", conf->enabled ? "yes" : "no" );
    CONF_LOG_INFO( "[LW_CONF] DevEUI:  %02X%02X%02X%02X%02X%02X%02X%02X\n", conf->dev_eui[0], conf->dev_eui[1],
                   conf->dev_eui[2], conf->dev_eui[3], conf->dev_eui[4], conf->dev_eui[5], conf->dev_eui[6],
                   conf->dev_eui[7] );
    CONF_LOG_INFO( "[LW_CONF] JoinEUI: %02X%02X%02X%02X%02X%02X%02X%02X\n", conf->join_eui[0], conf->join_eui[1],
                   conf->join_eui[2], conf->join_eui[3], conf->join_eui[4], conf->join_eui[5], conf->join_eui[6],
                   conf->join_eui[7] );
    CONF_LOG_INFO( "[LW_CONF] AppKey:  %02X%02X%02X%02X...%02X%02X%02X%02X (truncated)\n", conf->app_key[0],
                   conf->app_key[1], conf->app_key[2], conf->app_key[3], conf->app_key[12], conf->app_key[13],
                   conf->app_key[14], conf->app_key[15] );
    CONF_LOG_INFO( "[LW_CONF] Region:  %d\n", conf->region );
    CONF_LOG_INFO( "[LW_CONF] Status update interval: %u s\n", conf->keep_alive_interval_s );
    CONF_LOG_INFO( "[LW_CONF] Status port: %u\n", conf->alert_port );
    CONF_LOG_INFO( "[LW_CONF] ========================================\n" );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool parse_hex_string( const char* hex_str, uint8_t* bytes, size_t expected_len )
{
    if( hex_str == NULL || bytes == NULL )
    {
        return false;
    }

    size_t str_len = strlen( hex_str );

    /* Check length (2 hex chars per byte) */
    if( str_len != expected_len * 2 )
    {
        return false;
    }

    for( size_t i = 0; i < expected_len; i++ )
    {
        unsigned int byte_val;
        if( sscanf( hex_str + ( i * 2 ), "%02x", &byte_val ) != 1 )
        {
            return false;
        }
        bytes[i] = ( uint8_t ) byte_val;
    }

    return true;
}

/* --- EOF ------------------------------------------------------------------ */
