/**
 * @file      cmd_parser.c
 *
 * @brief     cmd_parser implementation
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>  //for memset
#include <inttypes.h>

#include "cmd_parser.h"

#include "smtc_modem_test_api.h"
#include "smtc_modem_api.h"
#include <smtc_rac_api.h>

// Protobuf includes for RAC Context serialization
#include "serialization/generated/smtc_rac_context.pb.h"
#include "serialization/helpers/rac_context_converter.h"
#include "serialization/nanopb/pb_decode.h"
#include "serialization/nanopb/pb_encode.h"

#if defined( ADD_APP_GEOLOCATION )
#include "smtc_modem_geolocation_api.h"
#include "lr11xx_hal.h"
#endif

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "modem_pinout.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_mcu.h"

#include "radio_utilities.h"

#if defined( USE_RELAY_TX )
#include "smtc_modem_relay_api.h"
#endif

#define STACK_ID 0
#if defined( STM32L073xx )
#define FILE_UPLOAD_MAX_SIZE 4096
#else
#define FILE_UPLOAD_MAX_SIZE 8192
#endif

#define MODEM_MAX_INFO_FIELD_SIZE 19
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Host command tab index
 */
typedef enum host_cmd_tab_idx_e
{
    HOST_CMD_TAB_IDX_AVAILABILITY = 0,
    HOST_CMD_TAB_IDX_MIN_LENGTH   = 1,
    HOST_CMD_TAB_IDX_MAX_LENGTH   = 2,
    HOST_CMD_TAB_IDX_COUNT,
} host_cmd_tab_idx_t;

/**
 * @brief Command length status
 */
typedef enum cmd_length_valid_e
{
    CMD_LENGTH_VALID,
    CMD_LENGTH_NOT_VALID,
} cmd_length_valid_t;

typedef enum upload_status_e
{
    UPLOAD_NOT_INIT,
    UPLOAD_INIT,
    UPLOAD_DATA_ON_GOING,
    UPLOAD_STARTED,
} upload_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static void* transceiver_context;

static bool                     modem_in_test_mode = false;
static smtc_modem_dl_metadata_t last_dl_metadata   = { 0 };

/* LFU handling */
static uint8_t         file_store[FILE_UPLOAD_MAX_SIZE] = { 0 };
static uint16_t        file_size                        = 0;
static uint16_t        upload_current_size              = 0;
static upload_status_t upload_status                    = UPLOAD_NOT_INIT;

#if defined( ADD_APP_GEOLOCATION )
/* Geolocation handling */
static smtc_modem_gnss_event_data_scan_done_t gnss_scan_data    = { 0 };
static cmd_serial_rc_code_t                   gnss_scan_done_rc = CMD_RC_FAIL;
#endif /* ADD_APP_GEOLOCATION */

/* ============================================================================ */
/* RAC CONTEXT                                                                  */
/* ============================================================================ */
typedef struct rac_context_data_s
{
    smtc_rac_context_t*    rac_context;
    uint8_t                radio_handle;
    uint8_t                payload[255];
    rp_status_t            status;
    bool                   pending_rac_event;
    smtc_rac_return_code_t last_rac_return_code;
    void ( *post_callback )( rp_status_t status );
} rac_context_data_t;
typedef struct rac_contexts_s
{
    smtc_rac_priority_t priority;
    rac_context_data_t  data;
} rac_contexts_t;

static void rac_post_callback( rp_status_t status, smtc_rac_priority_t priority );

static void rac_post_callback_very_high_priority( rp_status_t status )
{
    rac_post_callback( status, RAC_VERY_HIGH_PRIORITY );
}
static void rac_post_callback_high_priority( rp_status_t status )
{
    rac_post_callback( status, RAC_HIGH_PRIORITY );
}
static void rac_post_callback_medium_priority( rp_status_t status )
{
    rac_post_callback( status, RAC_MEDIUM_PRIORITY );
}
static void rac_post_callback_low_priority( rp_status_t status )
{
    rac_post_callback( status, RAC_LOW_PRIORITY );
}
static void rac_post_callback_very_low_priority( rp_status_t status )
{
    rac_post_callback( status, RAC_VERY_LOW_PRIORITY );
}

static rac_contexts_t rac_contexts[] = {
    { RAC_VERY_HIGH_PRIORITY,
      { NULL, 0, { 0 }, RP_STATUS_TASK_INIT, false, SMTC_RAC_SUCCESS, rac_post_callback_very_high_priority } },
    { RAC_HIGH_PRIORITY,
      { NULL, 0, { 0 }, RP_STATUS_TASK_INIT, false, SMTC_RAC_SUCCESS, rac_post_callback_high_priority } },
    { RAC_MEDIUM_PRIORITY,
      { NULL, 0, { 0 }, RP_STATUS_TASK_INIT, false, SMTC_RAC_SUCCESS, rac_post_callback_medium_priority } },
    { RAC_LOW_PRIORITY,
      { NULL, 0, { 0 }, RP_STATUS_TASK_INIT, false, SMTC_RAC_SUCCESS, rac_post_callback_low_priority } },
    { RAC_VERY_LOW_PRIORITY,
      { NULL, 0, { 0 }, RP_STATUS_TASK_INIT, false, SMTC_RAC_SUCCESS, rac_post_callback_very_low_priority } },
};

rac_context_data_t* get_rac_context_data_from_priority( smtc_rac_priority_t priority )
{
    for( size_t i = 0; i < sizeof( rac_contexts ) / sizeof( rac_contexts[0] ); i++ )
    {
        if( rac_contexts[i].priority == priority )
        {
            return &( rac_contexts[i].data );
        }
    }
    return NULL;
}

rac_context_data_t* get_rac_context_data_from_handle( uint8_t radio_handle )
{
    for( size_t i = 0; i < sizeof( rac_contexts ) / sizeof( rac_contexts[0] ); i++ )
    {
        if( rac_contexts[i].data.radio_handle == radio_handle )
        {
            return &( rac_contexts[i].data );
        }
    }
    return NULL;
}

void raz_rac_context_data( rac_context_data_t* rac_context_data )
{
    rac_context_data->rac_context  = NULL;
    rac_context_data->radio_handle = 0;
    // rac_context_data->payload = {0};
    rac_context_data->status               = RP_STATUS_TASK_INIT;
    rac_context_data->pending_rac_event    = false;
    rac_context_data->last_rac_return_code = SMTC_RAC_SUCCESS;
}

/* ============================================================================ */
/* NHM (New Hw Modem) Protocol Variables                                       */
/* ============================================================================ */

/* NHM segmentation buffer and state */
static uint8_t                      nhm_reassembly_buffer[NHM_REASSEMBLY_BUFFER_SIZE];
static nhm_segmentation_state_t     nhm_segmentation_state = { .cmd_id = 0, .current_pos = 0 };
static uint8_t                      nhm_reassembly_rsp_buffer[NHM_REASSEMBLY_BUFFER_SIZE];
static nhm_segmentation_rsp_state_t nhm_segmentation_rsp_state = { .cmd_id = 0, .current_pos = 0, .total_length = 0 };

/**
 * @brief Modem commands tab for availability, min length and max length
 *
 */
static const uint8_t host_cmd_tab[CMD_MAX][HOST_CMD_TAB_IDX_COUNT] = {
    /* [CMD_xxx] = {availability, len_min, len_max} */
    [CMD_RESET]                                = { 1, 0, 0 },
    [CMD_SET_REGION]                           = { 1, 1, 1 },
    [CMD_GET_REGION]                           = { 1, 0, 0 },
    [CMD_JOIN_NETWORK]                         = { 1, 0, 0 },
    [CMD_REQUEST_UPLINK]                       = { 1, 2, 244 },
    [CMD_GET_EVENT]                            = { 1, 0, 0 },
    [CMD_GET_DOWNLINK_DATA]                    = { 1, 0, 0 },
    [CMD_GET_DOWNLINK_METADATA]                = { 1, 0, 0 },
    [CMD_GET_JOIN_EUI]                         = { 1, 0, 0 },
    [CMD_SET_JOIN_EUI]                         = { 1, 8, 8 },
    [CMD_GET_DEV_EUI]                          = { 1, 0, 0 },
    [CMD_SET_DEV_EUI]                          = { 1, 8, 8 },
    [CMD_SET_NWKKEY]                           = { 1, 16, 16 },
    [CMD_GET_PIN]                              = { 1, 0, 0 },
    [CMD_GET_CHIP_EUI]                         = { 1, 0, 0 },
    [CMD_DERIVE_KEYS]                          = { 1, 0, 0 },
    [CMD_GET_MODEM_VERSION]                    = { 1, 0, 0 },
    [CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER]  = { 1, 0, 0 },
    [CMD_SET_CERTIFICATION_MODE]               = { 1, 1, 1 },
    [CMD_EMERGENCY_UPLINK]                     = { 1, 2, 244 },
    [CMD_REQUEST_EMPTY_UPLINK]                 = { 1, 3, 3 },
    [CMD_LEAVE_NETWORK]                        = { 1, 0, 0 },
    [CMD_ALARM_START_TIMER]                    = { 1, 4, 4 },
    [CMD_ALARM_CLEAR_TIMER]                    = { 1, 0, 0 },
    [CMD_ALARM_GET_REMAINING_TIME]             = { 1, 0, 0 },
    [CMD_GET_NEXT_TX_MAX_PAYLOAD]              = { 1, 0, 0 },
    [CMD_GET_DUTY_CYCLE_STATUS]                = { 1, 0, 0 },
    [CMD_SET_NETWORK_TYPE]                     = { 1, 1, 1 },
    [CMD_SET_JOIN_DR_DISTRIBUTION]             = { 1, 16, 16 },
    [CMD_SET_ADR_PROFILE]                      = { 1, 1, 17 },
    [CMD_SET_NB_TRANS]                         = { 1, 1, 1 },
    [CMD_GET_NB_TRANS]                         = { 1, 0, 0 },
    [CMD_GET_ENABLED_DATARATE]                 = { 1, 0, 0 },
    [CMD_SET_ADR_ACK_LIMIT_DELAY]              = { 1, 2, 2 },
    [CMD_GET_ADR_ACK_LIMIT_DELAY]              = { 1, 0, 0 },
    [CMD_SET_CRYSTAL_ERR]                      = { 1, 4, 4 },
    [CMD_LBT_SET_PARAMS]                       = { 1, 10, 10 },
    [CMD_LBT_GET_PARAMS]                       = { 1, 0, 0 },
    [CMD_LBT_SET_STATE]                        = { 1, 1, 1 },
    [CMD_LBT_GET_STATE]                        = { 1, 0, 0 },
    [CMD_GET_CHARGE]                           = { 1, 0, 0 },
    [CMD_RESET_CHARGE]                         = { 1, 0, 0 },
    [CMD_SET_CLASS]                            = { 1, 1, 1 },
    [CMD_CLASS_B_SET_PING_SLOT_PERIODICITY]    = { 1, 1, 1 },
    [CMD_CLASS_B_GET_PING_SLOT_PERIODICITY]    = { 1, 0, 0 },
    [CMD_MULTICAST_SET_GROUP_CONFIG]           = { 1, 37, 37 },
    [CMD_MULTICAST_GET_GROUP_CONFIG]           = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_C_START_SESSION]      = { 1, 6, 6 },
    [CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS] = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_C_STOP_SESSION]       = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS]  = { 1, 0, 0 },
    [CMD_MULTICAST_CLASS_B_START_SESSION]      = { 1, 7, 7 },
    [CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS] = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_B_STOP_SESSION]       = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS]  = { 1, 0, 0 },
    [CMD_START_ALCSYNC_SERVICE]                = { 1, 0, 0 },
    [CMD_STOP_ALCSYNC_SERVICE]                 = { 1, 0, 0 },
    [CMD_GET_ALCSYNC_TIME]                     = { 1, 0, 0 },
    [CMD_TRIG_ALCSYNC_REQUEST]                 = { 1, 0, 0 },
    [CMD_LORAWAN_MAC_REQUEST]                  = { 1, 1, 1 },
    [CMD_GET_LORAWAN_TIME]                     = { 1, 0, 0 },
    [CMD_GET_LINK_CHECK_DATA]                  = { 1, 0, 0 },
    [CMD_SET_DUTY_CYCLE_STATE]                 = { 1, 1, 1 },
    [CMD_DEBUG_CONNECT_WITH_ABP]               = { 1, 36, 36 },
    [CMD_TEST]                                 = { 1, 1, 255 },
    [CMD_GET_TX_POWER_OFFSET]                  = { 1, 0, 0 },
    [CMD_SET_TX_POWER_OFFSET]                  = { 1, 1, 1 },
    [CMD_CSMA_SET_STATE]                       = { 1, 1, 1 },
    [CMD_CSMA_GET_STATE]                       = { 1, 0, 0 },
    [CMD_CSMA_SET_PARAMETERS]                  = { 1, 3, 3 },
    [CMD_CSMA_GET_PARAMETERS]                  = { 1, 0, 0 },
    [CMD_STREAM_INIT]                          = { 1, 3, 3 },
    [CMD_STREAM_ADD_DATA]                      = { 1, 1, 255 },
    [CMD_STREAM_STATUS]                        = { 1, 0, 0 },
    [CMD_LFU_INIT]                             = { 1, 6, 6 },
    [CMD_LFU_DATA]                             = { 1, 0, 255 },
    [CMD_LFU_START]                            = { 1, 4, 4 },
    [CMD_LFU_RESET]                            = { 1, 0, 0 },
    [CMD_DM_ENABLE]                            = { 1, 1, 1 },
    [CMD_DM_GET_PORT]                          = { 1, 0, 0 },
    [CMD_DM_SET_PORT]                          = { 1, 1, 1 },
    [CMD_DM_GET_INFO_INTERVAL]                 = { 1, 0, 0 },
    [CMD_DM_SET_INFO_INTERVAL]                 = { 1, 1, 1 },
    [CMD_DM_GET_PERIODIC_INFO_FIELDS]          = { 1, 0, 0 },
    [CMD_DM_SET_PERIODIC_INFO_FIELDS]          = { 1, 0, MODEM_MAX_INFO_FIELD_SIZE },
    [CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS]     = { 1, 0, MODEM_MAX_INFO_FIELD_SIZE },
    [CMD_DM_SET_USER_DATA]                     = { 1, SMTC_MODEM_DM_USER_DATA_LENGTH, SMTC_MODEM_DM_USER_DATA_LENGTH },
    [CMD_DM_GET_USER_DATA]                     = { 1, 0, 0 },
    [CMD_GET_STATUS]                           = { 1, 0, 0 },
    [CMD_SUSPEND_RADIO_COMMUNICATIONS]         = { 1, 1, 1 },
    [CMD_GET_SUSPEND_RADIO_COMMUNICATIONS]     = { 1, 0, 0 },
    [CMD_DM_HANDLE_ALCSYNC]                    = { 1, 1, 1 },
    [CMD_SET_APPKEY]                           = { 1, 16, 16 },
    [CMD_GET_ADR_PROFILE]                      = { 1, 0, 0 },
    [CMD_GET_CERTIFICATION_MODE]               = { 1, 0, 0 },
    [CMD_STORE_AND_FORWARD_SET_STATE]          = { 1, 1, 1 },
    [CMD_STORE_AND_FORWARD_GET_STATE]          = { 1, 0, 0 },
    [CMD_STORE_AND_FORWARD_ADD_DATA]           = { 1, 2, 244 },
    [CMD_STORE_AND_FORWARD_CLEAR_DATA]         = { 1, 0, 0 },
    [CMD_STORE_AND_FORWARD_GET_FREE_SLOT]      = { 1, 0, 0 },
#if defined( ADD_APP_GEOLOCATION )
    [CMD_GNSS_SCAN]                             = { 1, 5, 5 },
    [CMD_GNSS_SCAN_CANCEL]                      = { 1, 0, 0 },
    [CMD_GNSS_GET_EVENT_DATA_SCAN_DONE]         = { 1, 0, 0 },
    [CMD_GNSS_GET_SCAN_DONE_RAW_DATA_LIST]      = { 1, 0, 0 },
    [CMD_GNSS_GET_SCAN_DONE_METADATA_LIST]      = { 1, 0, 0 },
    [CMD_GNSS_GET_SCAN_DONE_SCAN_SV]            = { 1, 0, 0 },
    [CMD_GNSS_GET_EVENT_DATA_TERMINATED]        = { 1, 0, 0 },
    [CMD_GNSS_SET_CONST]                        = { 1, 1, 1 },
    [CMD_GNSS_SET_PORT]                         = { 1, 1, 1 },
    [CMD_GNSS_SCAN_AGGREGATE]                   = { 1, 1, 1 },
    [CMD_GNSS_SEND_MODE]                        = { 1, 1, 1 },
    [CMD_GNSS_ALM_DEMOD_START]                  = { 1, 0, 0 },
    [CMD_GNSS_ALM_DEMOD_SET_CONSTEL]            = { 1, 1, 1 },
    [CMD_GNSS_ALM_DEMOD_GET_EVENT_DATA_ALM_UPD] = { 1, 0, 0 },
#if defined( ADD_ALMANAC )
    [CMD_CLOUD_ALMANAC_START] = { 1, 0, 0 },
    [CMD_CLOUD_ALMANAC_STOP]  = { 1, 0, 0 },
#endif
    [CMD_WIFI_SCAN_START]                = { 1, 4, 4 },
    [CMD_WIFI_SCAN_CANCEL]               = { 1, 0, 0 },
    [CMD_WIFI_GET_SCAN_DONE_SCAN_DATA]   = { 1, 0, 0 },
    [CMD_WIFI_GET_EVENT_DATA_TERMINATED] = { 1, 0, 0 },
    [CMD_WIFI_SET_PORT]                  = { 1, 1, 1 },
    [CMD_WIFI_SEND_MODE]                 = { 1, 1, 1 },
    [CMD_WIFI_SET_PAYLOAD_FORMAT]        = { 1, 1, 1 },
    [CMD_LR11XX_RADIO_READ]              = { 1, 0, 255 },
    [CMD_LR11XX_RADIO_WRITE]             = { 1, 0, 255 },
#endif /* ADD_APP_GEOLOCATION */

    [CMD_SET_RTC_OFFSET] = { 1, 4, 4 },

#if defined( USE_RELAY_TX )
    [CMD_SET_RELAY_CONFIG] = { 1, 10, 24 },
    [CMD_GET_RELAY_CONFIG] = { 1, 0, 0 },
#endif /* USE_RELAY_TX */

    [CMD_GET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF]     = { 1, 0, 0 },
    [CMD_SET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF]     = { 1, 1, 1 },
    [CMD_MODEM_GET_CRASHLOG]                     = { 1, 0, 0 },
    [CMD_MODEM_GET_REPORT_ALL_DOWNLINKS_TO_USER] = { 1, 0, 0 },
    [CMD_MODEM_SET_REPORT_ALL_DOWNLINKS_TO_USER] = { 1, 1, 1 },

    [CMD_USP_SUBMIT] = { 1, 1, 255 },  // Protobuf context: min 1 byte, max > 255 : SSA-TBD : how to manage
    [CMD_USP_CAD]    = { 1, 1, 255 },  // Protobuf context: min 1 byte, max > 255 : SSA-TBD : how to manage
    [CMD_USP_OPEN]   = { 1, 1, 1 },    // Parameter : radio priority
    [CMD_USP_CLOSE]  = { 1, 1, 1 },    // Parameter : radio ID
    [CMD_USP_ABORT]  = { 1, 1, 1 },    // Parameter : radio ID
    /* CMD_USP_GET_RESULTS removed - use NHM protocol instead */

    /* NHM (New Hw Modem) Protocol */
    [CMD_NHM_EXTENDED] = { 1, 4, 255 },  // NHM header (4 bytes) + payload (up to 251 bytes)

};

/**
 * @brief Test commands tab for availability, min length and max length
 *
 */
static const uint8_t host_cmd_test_tab[CMD_TST_MAX][HOST_CMD_TAB_IDX_COUNT] = {
    /* [CMD_xxx] = {availability, len_min, len_max} */
    [CMD_TST_START]            = { 1, 8, 8 },
    [CMD_TST_EXIT]             = { 1, 0, 0 },
    [CMD_TST_NOP]              = { 1, 0, 0 },
    [CMD_TST_TX_LORA]          = { 1, 25, 25 },
    [CMD_TST_TX_FSK]           = { 1, 14, 14 },
    [CMD_TST_TX_LRFHSS]        = { 1, 18, 18 },
    [CMD_TST_TX_CW]            = { 1, 5, 5 },
    [CMD_TST_RX_LORA]          = { 1, 16, 16 },
    [CMD_TST_RX_FSK_CONT]      = { 1, 4, 4 },
    [CMD_TST_READ_NB_PKTS_RX]  = { 1, 0, 0 },
    [CMD_TST_READ_LAST_RX_PKT] = { 1, 0, 0 },
    [CMD_TST_RSSI]             = { 1, 10, 10 },
    [CMD_TST_RSSI_GET]         = { 1, 0, 0 },
    [CMD_TST_RADIO_RST]        = { 1, 0, 0 },
    [CMD_TST_BUSYLOOP]         = { 1, 0, 0 },
    [CMD_TST_PANIC]            = { 1, 0, 0 },
    [CMD_TST_WATCHDOG]         = { 1, 0, 0 },
    [CMD_TST_RADIO_READ]       = { 1, 0, 255 },
    [CMD_TST_RADIO_WRITE]      = { 1, 0, 255 },
};

#if HAL_DBG_TRACE == HAL_FEATURE_ON
/**
 * @brief Host command string names for print purpose
 *
 */
static const char* host_cmd_str[CMD_MAX] = {
    [CMD_RESET]                                = "CMD_RESET",
    [CMD_SET_REGION]                           = "CMD_SET_REGION",
    [CMD_GET_REGION]                           = "CMD_GET_REGION",
    [CMD_JOIN_NETWORK]                         = "CMD_JOIN_NETWORK",
    [CMD_REQUEST_UPLINK]                       = "CMD_REQUEST_UPLINK",
    [CMD_GET_EVENT]                            = "CMD_GET_EVENT",
    [CMD_GET_DOWNLINK_DATA]                    = "CMD_GET_DOWNLINK_DATA",
    [CMD_GET_DOWNLINK_METADATA]                = "CMD_GET_DOWNLINK_METADATA",
    [CMD_GET_JOIN_EUI]                         = "CMD_GET_JOIN_EUI",
    [CMD_SET_JOIN_EUI]                         = "CMD_SET_JOIN_EUI",
    [CMD_GET_DEV_EUI]                          = "CMD_GET_DEV_EUI",
    [CMD_SET_DEV_EUI]                          = "CMD_SET_DEV_EUI",
    [CMD_SET_NWKKEY]                           = "CMD_SET_NWKKEY",
    [CMD_GET_PIN]                              = "CMD_GET_PIN",
    [CMD_GET_CHIP_EUI]                         = "CMD_GET_CHIP_EUI",
    [CMD_DERIVE_KEYS]                          = "CMD_DERIVE_KEYS",
    [CMD_GET_MODEM_VERSION]                    = "CMD_GET_MODEM_VERSION",
    [CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER]  = "CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER",
    [CMD_SET_CERTIFICATION_MODE]               = "CMD_SET_CERTIFICATION_MODE",
    [CMD_EMERGENCY_UPLINK]                     = "CMD_EMERGENCY_UPLINK",
    [CMD_REQUEST_EMPTY_UPLINK]                 = "CMD_REQUEST_EMPTY_UPLINK",
    [CMD_LEAVE_NETWORK]                        = "CMD_LEAVE_NETWORK",
    [CMD_ALARM_START_TIMER]                    = "CMD_ALARM_START_TIMER",
    [CMD_ALARM_CLEAR_TIMER]                    = "CMD_ALARM_CLEAR_TIMER",
    [CMD_ALARM_GET_REMAINING_TIME]             = "CMD_ALARM_GET_REMAINING_TIME",
    [CMD_GET_NEXT_TX_MAX_PAYLOAD]              = "CMD_GET_NEXT_TX_MAX_PAYLOAD",
    [CMD_GET_DUTY_CYCLE_STATUS]                = "CMD_GET_DUTY_CYCLE_STATUS",
    [CMD_SET_NETWORK_TYPE]                     = "CMD_SET_NETWORK_TYPE",
    [CMD_SET_JOIN_DR_DISTRIBUTION]             = "CMD_SET_JOIN_DR_DISTRIBUTION",
    [CMD_SET_ADR_PROFILE]                      = "CMD_SET_ADR_PROFILE",
    [CMD_SET_NB_TRANS]                         = "CMD_SET_NB_TRANS",
    [CMD_GET_NB_TRANS]                         = "CMD_GET_NB_TRANS",
    [CMD_GET_ENABLED_DATARATE]                 = "CMD_GET_ENABLED_DATARATE",
    [CMD_SET_ADR_ACK_LIMIT_DELAY]              = "CMD_SET_ADR_ACK_LIMIT_DELAY",
    [CMD_GET_ADR_ACK_LIMIT_DELAY]              = "CMD_GET_ADR_ACK_LIMIT_DELAY",
    [CMD_SET_CRYSTAL_ERR]                      = "CMD_SET_CRYSTAL_ERR",
    [CMD_LBT_SET_PARAMS]                       = "CMD_LBT_SET_PARAMS",
    [CMD_LBT_GET_PARAMS]                       = "CMD_LBT_GET_PARAMS",
    [CMD_LBT_SET_STATE]                        = "CMD_LBT_SET_STATE",
    [CMD_LBT_GET_STATE]                        = "CMD_LBT_GET_STATE",
    [CMD_GET_CHARGE]                           = "CMD_GET_CHARGE",
    [CMD_RESET_CHARGE]                         = "CMD_RESET_CHARGE",
    [CMD_SET_CLASS]                            = "CMD_SET_CLASS",
    [CMD_CLASS_B_SET_PING_SLOT_PERIODICITY]    = "CMD_CLASS_B_SET_PING_SLOT_PERIODICITY",
    [CMD_CLASS_B_GET_PING_SLOT_PERIODICITY]    = "CMD_CLASS_B_GET_PING_SLOT_PERIODICITY",
    [CMD_MULTICAST_SET_GROUP_CONFIG]           = "CMD_MULTICAST_SET_GROUP_CONFIG",
    [CMD_MULTICAST_GET_GROUP_CONFIG]           = "CMD_MULTICAST_GET_GROUP_CONFIG",
    [CMD_MULTICAST_CLASS_C_START_SESSION]      = "CMD_MULTICAST_CLASS_C_START_SESSION",
    [CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS] = "CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS",
    [CMD_MULTICAST_CLASS_C_STOP_SESSION]       = "CMD_MULTICAST_CLASS_C_STOP_SESSION",
    [CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS]  = "CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS",
    [CMD_MULTICAST_CLASS_B_START_SESSION]      = "CMD_MULTICAST_CLASS_B_START_SESSION",
    [CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS] = "CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS",
    [CMD_MULTICAST_CLASS_B_STOP_SESSION]       = "CMD_MULTICAST_CLASS_B_STOP_SESSION",
    [CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS]  = "CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS",
    [CMD_START_ALCSYNC_SERVICE]                = "CMD_START_ALCSYNC_SERVICE",
    [CMD_STOP_ALCSYNC_SERVICE]                 = "CMD_STOP_ALCSYNC_SERVICE",
    [CMD_GET_ALCSYNC_TIME]                     = "CMD_GET_ALCSYNC_TIME",
    [CMD_TRIG_ALCSYNC_REQUEST]                 = "CMD_TRIG_ALCSYNC_REQUEST",
    [CMD_LORAWAN_MAC_REQUEST]                  = "CMD_LORAWAN_MAC_REQUEST",
    [CMD_GET_LORAWAN_TIME]                     = "CMD_GET_LORAWAN_TIME",
    [CMD_GET_LINK_CHECK_DATA]                  = "CMD_GET_LINK_CHECK_DATA",
    [CMD_SET_DUTY_CYCLE_STATE]                 = "CMD_SET_DUTY_CYCLE_STATE",
    [CMD_DEBUG_CONNECT_WITH_ABP]               = "CMD_DEBUG_CONNECT_WITH_ABP",
    [CMD_TEST]                                 = "CMD_TEST",
    [CMD_GET_TX_POWER_OFFSET]                  = "CMD_GET_TX_POWER_OFFSET",
    [CMD_SET_TX_POWER_OFFSET]                  = "CMD_SET_TX_POWER_OFFSET",
    [CMD_CSMA_SET_STATE]                       = "CMD_CSMA_SET_STATE",
    [CMD_CSMA_GET_STATE]                       = "CMD_CSMA_GET_STATE",
    [CMD_CSMA_SET_PARAMETERS]                  = "CMD_CSMA_SET_PARAMETERS",
    [CMD_CSMA_GET_PARAMETERS]                  = "CMD_CSMA_GET_PARAMETERS",
    [CMD_STREAM_INIT]                          = "CMD_STREAM_INIT",
    [CMD_STREAM_ADD_DATA]                      = "CMD_STREAM_ADD_DATA",
    [CMD_STREAM_STATUS]                        = "CMD_STREAM_STATUS",
    [CMD_LFU_INIT]                             = "CMD_LFU_INIT",
    [CMD_LFU_DATA]                             = "CMD_LFU_DATA",
    [CMD_LFU_START]                            = "CMD_LFU_START",
    [CMD_LFU_RESET]                            = "CMD_LFU_RESET",
    [CMD_DM_ENABLE]                            = "CMD_DM_ENABLE",
    [CMD_DM_GET_PORT]                          = "CMD_DM_GET_PORT",
    [CMD_DM_SET_PORT]                          = "CMD_DM_SET_PORT",
    [CMD_DM_GET_INFO_INTERVAL]                 = "CMD_DM_GET_INFO_INTERVAL",
    [CMD_DM_SET_INFO_INTERVAL]                 = "CMD_DM_SET_INFO_INTERVAL",
    [CMD_DM_GET_PERIODIC_INFO_FIELDS]          = "CMD_DM_GET_PERIODIC_INFO_FIELDS",
    [CMD_DM_SET_PERIODIC_INFO_FIELDS]          = "CMD_DM_SET_PERIODIC_INFO_FIELDS",
    [CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS]     = "CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS",
    [CMD_DM_SET_USER_DATA]                     = "CMD_DM_SET_USER_DATA",
    [CMD_DM_GET_USER_DATA]                     = "CMD_DM_GET_USER_DATA",
    [CMD_GET_STATUS]                           = "CMD_GET_STATUS",
    [CMD_SUSPEND_RADIO_COMMUNICATIONS]         = "CMD_SUSPEND_RADIO_COMMUNICATIONS",
    [CMD_GET_SUSPEND_RADIO_COMMUNICATIONS]     = "CMD_GET_SUSPEND_RADIO_COMMUNICATIONS",
    [CMD_DM_HANDLE_ALCSYNC]                    = "CMD_DM_HANDLE_ALCSYNC",
    [CMD_SET_APPKEY]                           = "CMD_SET_APPKEY",
    [CMD_GET_ADR_PROFILE]                      = "CMD_GET_ADR_PROFILE",
    [CMD_GET_CERTIFICATION_MODE]               = "CMD_GET_CERTIFICATION_MODE",
    [CMD_STORE_AND_FORWARD_SET_STATE]          = "CMD_STORE_AND_FORWARD_SET_STATE",
    [CMD_STORE_AND_FORWARD_GET_STATE]          = "CMD_STORE_AND_FORWARD_GET_STATE",
    [CMD_STORE_AND_FORWARD_ADD_DATA]           = "CMD_STORE_AND_FORWARD_ADD_DATA",
    [CMD_STORE_AND_FORWARD_CLEAR_DATA]         = "CMD_STORE_AND_FORWARD_CLEAR_DATA",
    [CMD_STORE_AND_FORWARD_GET_FREE_SLOT]      = "CMD_STORE_AND_FORWARD_GET_FREE_SLOT",
#if defined( ADD_APP_GEOLOCATION )
    [CMD_GNSS_SCAN]                             = "CMD_GNSS_SCAN",
    [CMD_GNSS_SCAN_CANCEL]                      = "CMD_GNSS_SCAN_CANCEL",
    [CMD_GNSS_GET_EVENT_DATA_SCAN_DONE]         = "CMD_GNSS_GET_EVENT_DATA_SCAN_DONE",
    [CMD_GNSS_GET_SCAN_DONE_RAW_DATA_LIST]      = "CMD_GNSS_GET_SCAN_DONE_RAW_DATA_LIST",
    [CMD_GNSS_GET_SCAN_DONE_METADATA_LIST]      = "CMD_GNSS_GET_SCAN_DONE_METADATA_LIST",
    [CMD_GNSS_GET_SCAN_DONE_SCAN_SV]            = "CMD_GNSS_GET_SCAN_DONE_SCAN_SV",
    [CMD_GNSS_GET_EVENT_DATA_TERMINATED]        = "CMD_GNSS_GET_EVENT_DATA_TERMINATED",
    [CMD_GNSS_SET_CONST]                        = "CMD_GNSS_SET_CONST",
    [CMD_GNSS_SET_PORT]                         = "CMD_GNSS_SET_PORT",
    [CMD_GNSS_SCAN_AGGREGATE]                   = "CMD_GNSS_SCAN_AGGREGATE",
    [CMD_GNSS_SEND_MODE]                        = "CMD_GNSS_SEND_MODE",
    [CMD_GNSS_ALM_DEMOD_START]                  = "CMD_GNSS_ALM_DEMOD_START",
    [CMD_GNSS_ALM_DEMOD_SET_CONSTEL]            = "CMD_GNSS_ALM_DEMOD_SET_CONSTEL",
    [CMD_GNSS_ALM_DEMOD_GET_EVENT_DATA_ALM_UPD] = "CMD_GNSS_ALM_DEMOD_GET_EVENT_DATA_ALM_UPD",
#if defined( ADD_ALMANAC )
    [CMD_CLOUD_ALMANAC_START] = "CMD_CLOUD_ALMANAC_START",
    [CMD_CLOUD_ALMANAC_STOP]  = "CMD_CLOUD_ALMANAC_STOP",
#endif
    [CMD_WIFI_SCAN_START]                = "CMD_MODEM_WIFI_SCAN_START",
    [CMD_WIFI_SCAN_CANCEL]               = "CMD_MODEM_WIFI_SCAN_CANCEL",
    [CMD_WIFI_GET_SCAN_DONE_SCAN_DATA]   = "CMD_MODEM_WIFI_GET_SCAN_DONE_SCAN_DATA",
    [CMD_WIFI_GET_EVENT_DATA_TERMINATED] = "CMD_MODEM_WIFI_GET_EVENT_DATA_TERMINATED",
    [CMD_WIFI_SET_PORT]                  = "CMD_MODEM_WIFI_SET_PORT",
    [CMD_WIFI_SEND_MODE]                 = "CMD_MODEM_WIFI_SEND_MODE",
    [CMD_WIFI_SET_PAYLOAD_FORMAT]        = "CMD_MODEM_WIFI_SET_PAYLOAD_FORMAT",
    [CMD_LR11XX_RADIO_READ]              = "CMD_LR11XX_RADIO_READ",
    [CMD_LR11XX_RADIO_WRITE]             = "CMD_LR11XX_RADIO_WRITE",
#endif /* ADD_APP_GEOLOCATION */
    [CMD_SET_RTC_OFFSET] = "CMD_SET_RTC_OFFSET",
#if defined( USE_RELAY_TX )
    [CMD_SET_RELAY_CONFIG] = "CMD_SET_RELAY_CONFIG",
    [CMD_GET_RELAY_CONFIG] = "CMD_GET_RELAY_CONFIG",
#endif
    [CMD_GET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF]     = "CMD_GET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF",
    [CMD_SET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF]     = "CMD_SET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF",
    [CMD_MODEM_GET_CRASHLOG]                     = "CMD_GET_CRASHLOG",
    [CMD_MODEM_GET_REPORT_ALL_DOWNLINKS_TO_USER] = "CMD_MODEM_GET_REPORT_ALL_DOWNLINKS_TO_USER",
    [CMD_MODEM_SET_REPORT_ALL_DOWNLINKS_TO_USER] = "CMD_MODEM_SET_REPORT_ALL_DOWNLINKS_TO_USER",

    [CMD_USP_SUBMIT] = "CMD_USP_SUBMIT",
    [CMD_USP_CAD]    = "CMD_USP_CAD",
    [CMD_USP_OPEN]   = "CMD_USP_OPEN",
    [CMD_USP_CLOSE]  = "CMD_USP_CLOSE",
    [CMD_USP_ABORT]  = "CMD_USP_ABORT",
    /* CMD_USP_GET_RESULTS removed */

    [CMD_NHM_EXTENDED] = "CMD_NHM_EXTENDED",
};
#endif

#if HAL_DBG_TRACE == HAL_FEATURE_ON
/**
 * @brief Host test command names for print purpose
 *
 */
static const char* host_cmd_test_str[CMD_TST_MAX] = {
    [CMD_TST_START]            = "START",
    [CMD_TST_EXIT]             = "EXIT",
    [CMD_TST_NOP]              = "NOP",
    [CMD_TST_TX_LORA]          = "TX_LORA",
    [CMD_TST_TX_FSK]           = "TX_FSK",
    [CMD_TST_TX_LRFHSS]        = "TX_LRFHSS",
    [CMD_TST_TX_CW]            = "TX_CW",
    [CMD_TST_RX_LORA]          = "RX_LORA",
    [CMD_TST_RX_FSK_CONT]      = "RX_FSK_CONT",
    [CMD_TST_READ_NB_PKTS_RX]  = "READ_NB_PKTS_RX",
    [CMD_TST_READ_LAST_RX_PKT] = "READ_LAST_RX_PKT",
    [CMD_TST_RSSI]             = "RSSI",
    [CMD_TST_RSSI_GET]         = "TST_RSSI_GET",
    [CMD_TST_RADIO_RST]        = "RADIO_RST",
    [CMD_TST_BUSYLOOP]         = "BUSYLOOP",
    [CMD_TST_PANIC]            = "PANIC",
    [CMD_TST_WATCHDOG]         = "WATCHDOG",
    [CMD_TST_RADIO_READ]       = "RADIO_READ",
    [CMD_TST_RADIO_WRITE]      = "RADIO_WRITE",
};
#endif

/**
 * @brief Modem class conversion table from hw modem command format to modem api
 *
 */
static const smtc_modem_class_t cmd_modem_class_table[] = { SMTC_MODEM_CLASS_A, SMTC_MODEM_CLASS_C,
                                                            SMTC_MODEM_CLASS_B };

/**
 * @brief Look Up Table for hw modem event opcodes
 *
 */
static const uint8_t events_lut[SMTC_MODEM_EVENT_MAX] = {
    [SMTC_MODEM_EVENT_RESET]                             = 0x00,
    [SMTC_MODEM_EVENT_ALARM]                             = 0x01,
    [SMTC_MODEM_EVENT_JOINED]                            = 0x02,
    [SMTC_MODEM_EVENT_TXDONE]                            = 0x03,
    [SMTC_MODEM_EVENT_DOWNDATA]                          = 0x04,
    [SMTC_MODEM_EVENT_JOINFAIL]                          = 0x0A,
    [SMTC_MODEM_EVENT_ALCSYNC_TIME]                      = 0x0D,
    [SMTC_MODEM_EVENT_LINK_CHECK]                        = 0x10,
    [SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO]            = 0x13,
    [SMTC_MODEM_EVENT_CLASS_B_STATUS]                    = 0x14,
    [SMTC_MODEM_EVENT_LORAWAN_MAC_TIME]                  = 0x19,
    [SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE]                = 0x1A,
    [SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C] = 0x1B,
    [SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B] = 0x1C,
    [SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C]     = 0x1D,
    [SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B]     = 0x1E,
    [SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT]               = 0x1F,
    [SMTC_MODEM_EVENT_STREAM_DONE]                       = 0x08,
    [SMTC_MODEM_EVENT_UPLOAD_DONE]                       = 0x05,
    [SMTC_MODEM_EVENT_DM_SET_CONF]                       = 0x06,
    [SMTC_MODEM_EVENT_MUTE]                              = 0x07,
    [SMTC_MODEM_EVENT_GNSS_SCAN_DONE]                    = 0x20,
    [SMTC_MODEM_EVENT_GNSS_TERMINATED]                   = 0x21,
    [SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE]         = 0x22,
    [SMTC_MODEM_EVENT_WIFI_SCAN_DONE]                    = 0x23,
    [SMTC_MODEM_EVENT_WIFI_TERMINATED]                   = 0x24,
    [SMTC_MODEM_EVENT_RELAY_TX_DYNAMIC]                  = 0x30,
    [SMTC_MODEM_EVENT_RELAY_TX_MODE]                     = 0x31,
    [SMTC_MODEM_EVENT_RELAY_TX_SYNC]                     = 0x32,
    [SMTC_MODEM_EVENT_RELAY_RX_RUNNING]                  = 0x33,
    [SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE]               = 0x34,
    [SMTC_MODEM_EVENT_TEST_MODE]                         = 0x35, /*!< Test mode event */
    [SMTC_MODEM_EVENT_NO_DOWNLINK_THRESHOLD]             = 0x36,

};

/**
 * @brief Look Up Table for hw modem return code opcodes
 *
 */
static const cmd_serial_rc_code_t rc_lut[] = {
    [SMTC_MODEM_RC_OK]               = CMD_RC_OK,
    [SMTC_MODEM_RC_NOT_INIT]         = CMD_RC_NOT_INIT,
    [SMTC_MODEM_RC_INVALID]          = CMD_RC_INVALID,
    [SMTC_MODEM_RC_BUSY]             = CMD_RC_BUSY,
    [SMTC_MODEM_RC_FAIL]             = CMD_RC_FAIL,
    [SMTC_MODEM_RC_NO_TIME]          = CMD_RC_NO_TIME,
    [SMTC_MODEM_RC_INVALID_STACK_ID] = CMD_RC_INVALID_STACK_ID,
    [SMTC_MODEM_RC_NO_EVENT]         = CMD_RC_NO_EVENT,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Initialize Command Parser
 *
 */
void cmd_parser_update_rac_context( rac_context_data_t* rac_context_data );

/**
 * @brief Check command size
 *
 * @param [in] cmd_id Received command id
 * @param [in] length Received command length
 * @return cmd_length_valid_t
 */
static cmd_length_valid_t cmd_parser_check_cmd_size( host_cmd_id_t cmd_id, uint8_t length );

/**
 * @brief
 *
 * @param [in] test_id Received test command id
 * @param [in] length  Received test command length
 * @return cmd_length_valid_t
 */
static cmd_length_valid_t cmd_test_parser_check_cmd_size( host_cmd_test_id_t test_id, uint8_t length );

/**
 * @brief Crc function used for LFU (Large File Upload)
 *
 * @param [in] buf Payload buffer
 * @param [in] len Length of the payload
 * @return uint32_t The calculated CRC
 */
uint32_t cmd_parser_crc( const uint8_t* buf, int len );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void cmd_parser_set_transceiver_context( void* context )
{
    transceiver_context = context;
}

void cmd_parser_update_rac_context( rac_context_data_t* rac_context_data )
{
    // Initialize global rac_context with pre-allocated buffers
    // Note: payload buffer will be assigned to tx_payload_buffer or rx_payload_buffer based on operation
    rac_context_data->rac_context->smtc_rac_data_buffer_setup.tx_payload_buffer =
        rac_context_data->payload;  // Default TX buffer
    rac_context_data->rac_context->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer =
        sizeof( rac_context_data->payload );
    rac_context_data->rac_context->smtc_rac_data_buffer_setup.rx_payload_buffer =
        rac_context_data->payload;  // Default RX buffer
    rac_context_data->rac_context->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer =
        sizeof( rac_context_data->payload );

    // Initialize data results
    rac_context_data->rac_context->smtc_rac_data_result.rx_size                = 0;
    rac_context_data->rac_context->smtc_rac_data_result.radio_end_timestamp_ms = 0;  // Direct value (no longer pointer)
    rac_context_data->rac_context->smtc_rac_data_result.radio_start_timestamp_ms = 0;  // Direct value

    // Set default modulation type to LoRa
    rac_context_data->rac_context->modulation_type = SMTC_RAC_MODULATION_LORA;

    // Initialize radio params union for LoRa
    rac_context_data->rac_context->radio_params.lora.is_tx       = false;  // Default to RX
    rac_context_data->rac_context->radio_params.lora.max_rx_size = 255;    // Default max RX size
    rac_context_data->rac_context->radio_params.lora.tx_size     = 0;      // No TX data by default

    // Initialize LBT context to disabled by default
    rac_context_data->rac_context->lbt_context.lbt_enabled = false;

    // Initialize CAD context to default values
    memset( &rac_context_data->rac_context->cad_context, 0, sizeof( smtc_rac_cad_radio_params_t ) );

    // Initialize CW context to disabled by default
    rac_context_data->rac_context->cw_context.cw_enabled = false;
}

cmd_parse_status_t parse_cmd( cmd_input_t* cmd_input, cmd_response_t* cmd_output )
{
    cmd_parse_status_t ret = PARSE_OK;

    cmd_output->return_code = CMD_RC_OK;
    cmd_output->length      = 0;

    if( ( cmd_input->cmd_code >= CMD_MAX ) ||
        ( host_cmd_tab[cmd_input->cmd_code][HOST_CMD_TAB_IDX_AVAILABILITY] != 1 ) )
    {
        SMTC_HAL_TRACE_ERROR( "Unknown command (0x%x)\n", cmd_input->cmd_code );
        cmd_output->return_code = CMD_RC_UNKNOWN;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }

    if( cmd_parser_check_cmd_size( cmd_input->cmd_code, cmd_input->length ) == CMD_LENGTH_NOT_VALID )
    {
        cmd_output->return_code = CMD_RC_BAD_SIZE;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }
    SMTC_HAL_TRACE_WARNING( "CMD_%s (0x%02x)\n", host_cmd_str[cmd_input->cmd_code], cmd_input->cmd_code );
    switch( cmd_input->cmd_code )
    {
    case CMD_GET_EVENT:
    {
        smtc_modem_event_t current_event       = { 0 };
        uint8_t            event_pending_count = 0;

        cmd_output->return_code = rc_lut[smtc_modem_get_event( &current_event, &event_pending_count )];
        if( cmd_output->return_code == CMD_RC_NO_EVENT )
        {
            /* No event available */
            cmd_output->length = 0;
            break;
        }

        /* buffer[0]: event type */
        cmd_output->buffer[0] = events_lut[current_event.event_type];

        /* buffer[1]: missed event */
        cmd_output->buffer[1] = current_event.missed_events;

        /* buffer[2-N]; event data, depend on event_type */
        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            cmd_output->buffer[2] = ( uint8_t ) ( current_event.event_data.reset.count >> 8 );
            cmd_output->buffer[3] = ( uint8_t ) ( current_event.event_data.reset.count );
            cmd_output->length    = 4;
            break;
        case SMTC_MODEM_EVENT_TXDONE:
            cmd_output->buffer[2] = current_event.event_data.txdone.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_LINK_CHECK:
            cmd_output->buffer[2] = current_event.event_data.link_check.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            cmd_output->buffer[2] = current_event.event_data.class_b_ping_slot_info.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            cmd_output->buffer[2] = current_event.event_data.class_b_status.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
            cmd_output->buffer[2] = current_event.event_data.lorawan_mac_time.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE:
            cmd_output->buffer[2] = current_event.event_data.fuota_status.successful;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
            cmd_output->buffer[2] = current_event.event_data.new_multicast_class_c.group_id;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
            cmd_output->buffer[2] = current_event.event_data.new_multicast_class_b.group_id;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT:
            cmd_output->buffer[2] = current_event.event_data.fmp.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_UPLOAD_DONE:
            cmd_output->buffer[2] = current_event.event_data.uploaddone.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_DM_SET_CONF:
            cmd_output->buffer[2] = current_event.event_data.setconf.opcode;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_MUTE:
            cmd_output->buffer[2] = current_event.event_data.mute.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_DOWNDATA:
        case SMTC_MODEM_EVENT_ALCSYNC_TIME:
        case SMTC_MODEM_EVENT_ALARM:
        case SMTC_MODEM_EVENT_JOINED:
        case SMTC_MODEM_EVENT_JOINFAIL:
        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C:
        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B:
        case SMTC_MODEM_EVENT_STREAM_DONE:
        case SMTC_MODEM_EVENT_GNSS_SCAN_DONE:
        case SMTC_MODEM_EVENT_GNSS_TERMINATED:
        case SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE:
        case SMTC_MODEM_EVENT_WIFI_SCAN_DONE:
        case SMTC_MODEM_EVENT_WIFI_TERMINATED:
            cmd_output->length = 2;
            break;

        case SMTC_MODEM_EVENT_RELAY_TX_DYNAMIC:
        case SMTC_MODEM_EVENT_RELAY_TX_MODE:
        case SMTC_MODEM_EVENT_RELAY_TX_SYNC:
            cmd_output->buffer[2] = current_event.event_data.relay_tx.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_RELAY_RX_RUNNING:
            cmd_output->buffer[2] = current_event.event_data.relay_rx.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_TEST_MODE:
            cmd_output->buffer[2] = current_event.event_data.test_mode_status.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE:
            cmd_output->buffer[2] = current_event.event_data.regional_duty_cycle.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_NO_DOWNLINK_THRESHOLD:
            cmd_output->buffer[2] = current_event.event_data.no_downlink.status;
            cmd_output->length    = 3;
            break;
        default:
            cmd_output->length = 0;
            break;
        }

        /* Handle event_pending_count */
        if( event_pending_count == 0 )
        {
            /* de-assert hw_modem irq line to indicate host that all events have been
             * retrieved
             */
            hal_gpio_set_value( HW_MODEM_EVENT_PIN, 0 );
        }
        break;
    }
    case CMD_GET_DOWNLINK_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_downlink_data( &cmd_output->buffer[2], &cmd_output->buffer[1],
                                                                       &last_dl_metadata, &cmd_output->buffer[0] )];

        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 2 + cmd_output->buffer[1];
        }
        break;
    }
    case CMD_GET_DOWNLINK_METADATA:
    {
        cmd_output->return_code = CMD_RC_OK;

        cmd_output->buffer[0]  = last_dl_metadata.stack_id;
        cmd_output->buffer[1]  = last_dl_metadata.rssi;
        cmd_output->buffer[2]  = last_dl_metadata.snr;
        cmd_output->buffer[3]  = last_dl_metadata.window;
        cmd_output->buffer[4]  = last_dl_metadata.fport;
        cmd_output->buffer[5]  = last_dl_metadata.fpending_bit;
        cmd_output->buffer[6]  = ( last_dl_metadata.frequency_hz >> 24 ) & 0xff;
        cmd_output->buffer[7]  = ( last_dl_metadata.frequency_hz >> 16 ) & 0xff;
        cmd_output->buffer[8]  = ( last_dl_metadata.frequency_hz >> 8 ) & 0xff;
        cmd_output->buffer[9]  = ( last_dl_metadata.frequency_hz & 0xff );
        cmd_output->buffer[10] = last_dl_metadata.datarate;

        cmd_output->length = 11;
        break;
    }
    case CMD_RESET:
    {
        smtc_modem_hal_reset_mcu( );
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_RESET_CHARGE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_reset_charge( )];
        break;
    }
    case CMD_GET_CHARGE:
    {
        uint32_t charge = 0;

        cmd_output->return_code = rc_lut[smtc_modem_get_charge( &charge )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( charge >> 24 ) & 0xFF;
            cmd_output->buffer[1] = ( charge >> 16 ) & 0xFF;
            cmd_output->buffer[2] = ( charge >> 8 ) & 0xFF;
            cmd_output->buffer[3] = ( charge & 0xFF );
            cmd_output->length    = 4;
        }
        break;
    }
    case CMD_GET_TX_POWER_OFFSET:
    {
        int8_t offset = radio_utilities_get_tx_power_offset( );

        cmd_output->buffer[0]   = offset;
        cmd_output->length      = 1;
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_SET_TX_POWER_OFFSET:
    {
        radio_utilities_set_tx_power_offset( cmd_input->buffer[0] );
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_GET_ALCSYNC_TIME:
    {
        uint32_t gps_time_s = 0;

        cmd_output->return_code = rc_lut[smtc_modem_get_alcsync_time( STACK_ID, &gps_time_s )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( gps_time_s >> 24 ) & 0xFF;
            cmd_output->buffer[1] = ( gps_time_s >> 16 ) & 0xFF;
            cmd_output->buffer[2] = ( gps_time_s >> 8 ) & 0xFF;
            cmd_output->buffer[3] = ( gps_time_s & 0xFF );
            cmd_output->length    = 4;
        }
        break;
    }
    case CMD_ALARM_START_TIMER:
    {
        uint32_t alarm = 0;

        alarm |= cmd_input->buffer[0] << 24;
        alarm |= cmd_input->buffer[1] << 16;
        alarm |= cmd_input->buffer[2] << 8;
        alarm |= cmd_input->buffer[3];

        cmd_output->return_code = rc_lut[smtc_modem_alarm_start_timer( alarm )];
        break;
    }
    case CMD_GET_PIN:
    {
#if defined( CONFIG_LORA_BASICS_MODEM_CRYPTOGRAPHY_LR11XX_WITH_CREDENTIALS )
        uint8_t chip_pin[4];

        cmd_output->return_code = rc_lut[smtc_modem_get_pin( STACK_ID, chip_pin )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 4;
            /* reverse endianness */
            for( uint8_t i = 0; i < cmd_output->length; i++ )
            {
                cmd_output->buffer[i] = chip_pin[i];
            }
        }
#else
        cmd_output->return_code = CMD_RC_FAIL;
#endif
        break;
    }
    case CMD_GET_CHIP_EUI:
    {
#if defined( CONFIG_LORA_BASICS_MODEM_CRYPTOGRAPHY_LR11XX_WITH_CREDENTIALS )
        uint8_t chip_eui[8];

        cmd_output->return_code = rc_lut[smtc_modem_get_chip_eui( STACK_ID, chip_eui )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 8;
            /* reverse endianness */
            for( uint8_t i = 0; i < cmd_output->length; i++ )
            {
                cmd_output->buffer[i] = chip_eui[i];
            }
        }
#else
        cmd_output->return_code = CMD_RC_FAIL;
#endif
        break;
    }
    case CMD_GET_JOIN_EUI:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_joineui( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 8;
        }
        break;
    }
    case CMD_SET_JOIN_EUI:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_joineui( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_DEV_EUI:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_deveui( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 8;
        }
        break;
    }
    case CMD_SET_DEV_EUI:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_deveui( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_SET_NWKKEY:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_nwkkey( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_SET_CLASS:
    {
        if( cmd_input->buffer[0] > 2 )
        {
            cmd_output->return_code = CMD_RC_INVALID;
            cmd_output->length      = 0;
        }
        else
        {
            cmd_output->return_code =
                rc_lut[smtc_modem_set_class( STACK_ID, cmd_modem_class_table[cmd_input->buffer[0]] )];
        }
        break;
    }
    case CMD_GET_REGION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_region( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_SET_REGION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_region( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_SET_ADR_PROFILE:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_adr_set_profile( STACK_ID, cmd_input->buffer[0], &cmd_input->buffer[1] )];
        break;
    }
    case CMD_GET_ADR_PROFILE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_adr_get_profile( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_JOIN_NETWORK:
    {
        cmd_output->return_code = rc_lut[smtc_modem_join_network( STACK_ID )];
        break;
    }
    case CMD_LEAVE_NETWORK:
    {
        cmd_output->return_code = rc_lut[smtc_modem_leave_network( STACK_ID )];
        break;
    }
    case CMD_GET_NEXT_TX_MAX_PAYLOAD:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_next_tx_max_payload( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_REQUEST_UPLINK:
    {
        /* check if confirmed/not confirmed arg if different from 0/1
         * (modem api takes bool)
         */
        if( ( cmd_input->buffer[1] != 0x00 ) && ( cmd_input->buffer[1] != 0x01 ) )
        {
            cmd_output->return_code = CMD_RC_INVALID;
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_request_uplink(
                STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], &cmd_input->buffer[2], cmd_input->length - 2 )];
        }
        break;
    }
    case CMD_EMERGENCY_UPLINK:
    {
        /* check if confirmed/not confirmed arg if different from 0/1
         *(modem api takes bool)
         */
        if( ( cmd_input->buffer[1] != 0x00 ) && ( cmd_input->buffer[1] != 0x01 ) )
        {
            cmd_output->return_code = CMD_RC_INVALID;
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_request_emergency_uplink(
                STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], &cmd_input->buffer[2], cmd_input->length - 2 )];
        }
        break;
    }
    case CMD_DERIVE_KEYS:
    {
#if defined( CONFIG_LORA_BASICS_MODEM_CRYPTOGRAPHY_LR11XX_WITH_CREDENTIALS )
        cmd_output->return_code = rc_lut[smtc_modem_derive_keys( STACK_ID )];
#else
        cmd_output->return_code = CMD_RC_FAIL;
#endif
        break;
    }
    case CMD_SET_CERTIFICATION_MODE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_certification_mode( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_CERTIFICATION_MODE:
    {
        bool certification_enabled = false;

        cmd_output->return_code = rc_lut[smtc_modem_get_certification_mode( STACK_ID, &certification_enabled )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( uint8_t ) certification_enabled;
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_SET_RTC_OFFSET:
    {
        uint32_t rtc_offset = 0;

        rtc_offset |= cmd_input->buffer[0] << 24;
        rtc_offset |= cmd_input->buffer[1] << 16;
        rtc_offset |= cmd_input->buffer[2] << 8;
        rtc_offset |= cmd_input->buffer[3];
        SMTC_HAL_TRACE_PRINTF( " change rtc offset to test wrapping with value = %x\n", rtc_offset );
        smtc_modem_hal_set_offset_to_test_wrapping( rtc_offset );
        break;
    }

#if defined( USE_RELAY_TX )
    case CMD_SET_RELAY_CONFIG:
    {
        smtc_modem_relay_tx_config_t user_relay_config = { 0 };

        user_relay_config.second_ch.freq_hz |= cmd_input->buffer[0] << 24;
        user_relay_config.second_ch.freq_hz |= cmd_input->buffer[1] << 16;
        user_relay_config.second_ch.freq_hz |= cmd_input->buffer[2] << 8;
        user_relay_config.second_ch.freq_hz |= cmd_input->buffer[3];

        user_relay_config.second_ch.ack_freq_hz |= cmd_input->buffer[4] << 24;
        user_relay_config.second_ch.ack_freq_hz |= cmd_input->buffer[5] << 16;
        user_relay_config.second_ch.ack_freq_hz |= cmd_input->buffer[6] << 8;
        user_relay_config.second_ch.ack_freq_hz |= cmd_input->buffer[7];

        user_relay_config.second_ch.dr     = cmd_input->buffer[8];
        user_relay_config.second_ch_enable = cmd_input->buffer[9];

        user_relay_config.backoff                                         = cmd_input->buffer[10];
        user_relay_config.activation                                      = cmd_input->buffer[11];
        user_relay_config.smart_level                                     = cmd_input->buffer[12];
        user_relay_config.number_of_miss_wor_ack_to_switch_in_nosync_mode = cmd_input->buffer[13];
        smtc_modem_relay_tx_enable( STACK_ID, &user_relay_config );

        break;
    }
    case CMD_GET_RELAY_CONFIG:
    {
        smtc_modem_relay_tx_config_t user_relay_config = { 0 };

        smtc_modem_relay_tx_get_config( STACK_ID, &user_relay_config );
        cmd_output->buffer[0] = ( user_relay_config.second_ch.freq_hz >> 24 ) & 0xFF;
        cmd_output->buffer[1] = ( user_relay_config.second_ch.freq_hz >> 16 ) & 0xFF;
        cmd_output->buffer[2] = ( user_relay_config.second_ch.freq_hz >> 8 ) & 0xFF;
        cmd_output->buffer[3] = ( user_relay_config.second_ch.freq_hz & 0xFF );

        cmd_output->buffer[4] = ( user_relay_config.second_ch.ack_freq_hz >> 24 ) & 0xFF;
        cmd_output->buffer[5] = ( user_relay_config.second_ch.ack_freq_hz >> 16 ) & 0xFF;
        cmd_output->buffer[6] = ( user_relay_config.second_ch.ack_freq_hz >> 8 ) & 0xFF;
        cmd_output->buffer[7] = ( user_relay_config.second_ch.ack_freq_hz & 0xFF );
        cmd_output->buffer[8] = user_relay_config.second_ch.dr;
        cmd_output->buffer[9] = user_relay_config.second_ch_enable;

        cmd_output->buffer[10]  = user_relay_config.backoff;
        cmd_output->buffer[11]  = user_relay_config.activation;
        cmd_output->buffer[12]  = user_relay_config.smart_level;
        cmd_output->buffer[13]  = user_relay_config.number_of_miss_wor_ack_to_switch_in_nosync_mode;
        cmd_output->length      = 14;
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
#endif
    case CMD_TEST:
    {
        cmd_tst_input_t    cmd_tst_input;
        cmd_tst_response_t cmd_tst_output;

        cmd_tst_input.cmd_code = ( host_cmd_test_id_t ) cmd_input->buffer[0];
        cmd_tst_input.length   = cmd_input->length - 1;
        cmd_tst_input.buffer   = &cmd_input->buffer[1];
        cmd_tst_output.buffer  = &cmd_output->buffer[0];

        ret = cmd_test_parser( &cmd_tst_input, &cmd_tst_output );

        cmd_output->return_code = cmd_tst_output.return_code;
        cmd_output->length      = cmd_tst_output.length;
        break;
    }
    case CMD_GET_DUTY_CYCLE_STATUS:
    {
        int32_t next_free_dtc = 0;

        cmd_output->return_code = rc_lut[smtc_modem_get_duty_cycle_status( STACK_ID, &next_free_dtc )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( next_free_dtc >> 24 ) & 0xff;
            cmd_output->buffer[1] = ( next_free_dtc >> 16 ) & 0xff;
            cmd_output->buffer[2] = ( next_free_dtc >> 8 ) & 0xff;
            cmd_output->buffer[3] = ( next_free_dtc & 0xff );

            cmd_output->length = 4;
        }
        break;
    }
    case CMD_SET_DUTY_CYCLE_STATE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_debug_set_duty_cycle_state( cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_ENABLED_DATARATE:
    {
        uint16_t enabled_datarate = 0;

        cmd_output->return_code = rc_lut[smtc_modem_get_enabled_datarates( STACK_ID, &enabled_datarate )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( enabled_datarate >> 8 ) & 0xff;
            cmd_output->buffer[1] = ( enabled_datarate & 0xff );
            cmd_output->length    = 2;
        }
        break;
    }
    case CMD_SET_NETWORK_TYPE:
    {
        if( cmd_input->buffer[0] > 1 )
        {
            cmd_output->return_code = CMD_RC_INVALID;
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_set_network_type( STACK_ID, cmd_input->buffer[0] )];
        }
        break;
    }
    case CMD_SET_NB_TRANS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_nb_trans( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_NB_TRANS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_nb_trans( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_SET_CRYSTAL_ERR:
    {
        uint32_t crystal_error = 0;

        crystal_error |= cmd_input->buffer[0] << 24;
        crystal_error |= cmd_input->buffer[1] << 16;
        crystal_error |= cmd_input->buffer[2] << 8;
        crystal_error |= cmd_input->buffer[3];
        cmd_output->return_code = rc_lut[smtc_modem_set_crystal_error_ppm( crystal_error )];
        break;
    }
    case CMD_MULTICAST_SET_GROUP_CONFIG:
    {
        uint32_t addr = 0;

        addr = cmd_input->buffer[1] << 24;
        addr |= cmd_input->buffer[2] << 16;
        addr |= cmd_input->buffer[3] << 8;
        addr |= cmd_input->buffer[4];

        cmd_output->return_code = rc_lut[smtc_modem_multicast_set_grp_config(
            STACK_ID, cmd_input->buffer[0], addr, &cmd_input->buffer[5], &cmd_input->buffer[21] )];
        break;
    }
    case CMD_MULTICAST_GET_GROUP_CONFIG:
    {
        uint32_t addr = 0;

        cmd_output->return_code = rc_lut[smtc_modem_multicast_get_grp_config( STACK_ID, cmd_input->buffer[0], &addr )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( addr >> 24 ) & 0xff;
            cmd_output->buffer[1] = ( addr >> 16 ) & 0xff;
            cmd_output->buffer[2] = ( addr >> 8 ) & 0xff;
            cmd_output->buffer[3] = ( addr & 0xff );

            cmd_output->length = 4;
        }
        break;
    }
    case CMD_MULTICAST_CLASS_C_START_SESSION:
    {
        uint32_t freq = 0;

        freq = cmd_input->buffer[1] << 24;
        freq |= cmd_input->buffer[2] << 16;
        freq |= cmd_input->buffer[3] << 8;
        freq |= cmd_input->buffer[4];

        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_c_start_session( STACK_ID, cmd_input->buffer[0],
                                                                                     freq, cmd_input->buffer[5] )];
        break;
    }
    case CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS:
    {
        uint32_t freq               = 0;
        uint8_t  dr                 = 0xFF;
        bool     is_session_started = false;

        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_c_get_session_status(
            STACK_ID, cmd_input->buffer[0], &is_session_started, &freq, &dr )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = is_session_started;
            cmd_output->buffer[1] = ( freq >> 24 ) & 0xff;
            cmd_output->buffer[2] = ( freq >> 16 ) & 0xff;
            cmd_output->buffer[3] = ( freq >> 8 ) & 0xff;
            cmd_output->buffer[4] = ( freq & 0xff );
            cmd_output->buffer[5] = dr;

            cmd_output->length = 6;
        }
        break;
    }
    case CMD_MULTICAST_CLASS_C_STOP_SESSION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_c_stop_session( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_c_stop_all_sessions( STACK_ID )];
        break;
    }
    case CMD_GET_MODEM_VERSION:
    {
        smtc_modem_version_t modem_version = { 0 };

        cmd_output->return_code = rc_lut[smtc_modem_get_modem_version( &modem_version )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = modem_version.major;
            cmd_output->buffer[1] = modem_version.minor;
            cmd_output->buffer[2] = modem_version.patch;

            cmd_output->length = 3;
        }
        break;
    }
    case CMD_ALARM_CLEAR_TIMER:
    {
        cmd_output->return_code = rc_lut[smtc_modem_alarm_clear_timer( )];
        break;
    }
    case CMD_ALARM_GET_REMAINING_TIME:
    {
        uint32_t remaining_time = 0;

        cmd_output->return_code = rc_lut[smtc_modem_alarm_get_remaining_time( &remaining_time )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( remaining_time >> 24 ) & 0xff;
            cmd_output->buffer[1] = ( remaining_time >> 16 ) & 0xff;
            cmd_output->buffer[2] = ( remaining_time >> 8 ) & 0xff;
            cmd_output->buffer[3] = ( remaining_time & 0xff );
            cmd_output->length    = 4;
        }
        break;
    }
    case CMD_REQUEST_EMPTY_UPLINK:
    {
        cmd_output->return_code = rc_lut[smtc_modem_request_empty_uplink( STACK_ID, cmd_input->buffer[0],
                                                                          cmd_input->buffer[1], cmd_input->buffer[2] )];
        break;
    }
    case CMD_LBT_SET_PARAMS:
    {
        uint32_t listening_duration_ms = 0;
        int16_t  threshold_dbm         = 0;
        uint32_t bw_hz                 = 0;

        listening_duration_ms |= cmd_input->buffer[0] << 24;
        listening_duration_ms |= cmd_input->buffer[1] << 16;
        listening_duration_ms |= cmd_input->buffer[2] << 8;
        listening_duration_ms |= cmd_input->buffer[3];

        threshold_dbm |= cmd_input->buffer[4] << 8;
        threshold_dbm |= cmd_input->buffer[5];

        bw_hz |= cmd_input->buffer[6] << 24;
        bw_hz |= cmd_input->buffer[7] << 16;
        bw_hz |= cmd_input->buffer[8] << 8;
        bw_hz |= cmd_input->buffer[9];

        cmd_output->return_code =
            rc_lut[smtc_modem_lbt_set_parameters( STACK_ID, listening_duration_ms, threshold_dbm, bw_hz )];
        break;
    }
    case CMD_LBT_GET_PARAMS:
    {
        uint32_t listening_duration_ms = 0;
        int16_t  threshold_dbm         = 0;
        uint32_t bw_hz                 = 0;

        cmd_output->return_code =
            rc_lut[smtc_modem_lbt_get_parameters( STACK_ID, &listening_duration_ms, &threshold_dbm, &bw_hz )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( listening_duration_ms >> 24 ) & 0xff;
            cmd_output->buffer[1] = ( listening_duration_ms >> 16 ) & 0xff;
            cmd_output->buffer[2] = ( listening_duration_ms >> 8 ) & 0xff;
            cmd_output->buffer[3] = ( listening_duration_ms & 0xff );
            cmd_output->buffer[4] = ( threshold_dbm >> 8 ) & 0xff;
            cmd_output->buffer[5] = ( threshold_dbm & 0xff );
            cmd_output->buffer[6] = ( bw_hz >> 24 ) & 0xff;
            cmd_output->buffer[7] = ( bw_hz >> 16 ) & 0xff;
            cmd_output->buffer[8] = ( bw_hz >> 8 ) & 0xff;
            cmd_output->buffer[9] = ( bw_hz & 0xff );

            cmd_output->length = 10;
        }
        break;
    }
    case CMD_LBT_SET_STATE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_lbt_set_state( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_LBT_GET_STATE:
    {
        bool enabled = false;

        cmd_output->return_code = rc_lut[smtc_modem_lbt_get_state( STACK_ID, &enabled )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = enabled;
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_START_ALCSYNC_SERVICE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_start_alcsync_service( STACK_ID )];
        break;
    }
    case CMD_STOP_ALCSYNC_SERVICE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_stop_alcsync_service( STACK_ID )];
        break;
    }
    case CMD_TRIG_ALCSYNC_REQUEST:
    {
        cmd_output->return_code = rc_lut[smtc_modem_trigger_alcsync_request( STACK_ID )];
        break;
    }
    case CMD_CLASS_B_SET_PING_SLOT_PERIODICITY:
    {
        cmd_output->return_code = rc_lut[smtc_modem_class_b_set_ping_slot_periodicity(
            STACK_ID, ( smtc_modem_class_b_ping_slot_periodicity_t ) cmd_input->buffer[0] )];
        break;
    }
    case CMD_CLASS_B_GET_PING_SLOT_PERIODICITY:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_class_b_get_ping_slot_periodicity( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_MULTICAST_CLASS_B_START_SESSION:
    {
        uint32_t freq = 0;

        freq = cmd_input->buffer[1] << 24;
        freq |= cmd_input->buffer[2] << 16;
        freq |= cmd_input->buffer[3] << 8;
        freq |= cmd_input->buffer[4];

        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_b_start_session(
            STACK_ID, cmd_input->buffer[0], freq, cmd_input->buffer[5], cmd_input->buffer[6] )];
        break;
    }
    case CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS:
    {
        uint32_t                                   freq                          = 0;
        uint8_t                                    dr                            = 0xFF;
        bool                                       is_session_started            = false;
        bool                                       is_session_waiting_for_beacon = false;
        smtc_modem_class_b_ping_slot_periodicity_t ping_slot_periodicity         = SMTC_MODEM_CLASS_B_PINGSLOT_1_S;

        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_b_get_session_status(
            STACK_ID, cmd_input->buffer[0], &is_session_started, &is_session_waiting_for_beacon, &freq, &dr,
            &ping_slot_periodicity )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = is_session_started;
            cmd_output->buffer[1] = ( freq >> 24 ) & 0xff;
            cmd_output->buffer[2] = ( freq >> 16 ) & 0xff;
            cmd_output->buffer[3] = ( freq >> 8 ) & 0xff;
            cmd_output->buffer[4] = ( freq & 0xff );
            cmd_output->buffer[5] = dr;
            cmd_output->buffer[6] = is_session_waiting_for_beacon;
            cmd_output->buffer[7] = ping_slot_periodicity;

            cmd_output->length = 8;
        }
        break;
    }
    case CMD_MULTICAST_CLASS_B_STOP_SESSION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_b_stop_session( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_b_stop_all_sessions( STACK_ID )];
        break;
    }
    case CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER:
    {
        uint16_t lost_connection_cnt     = 0;
        uint32_t lost_connection_since_s = 0;

        cmd_output->return_code =
            rc_lut[smtc_modem_lorawan_get_lost_connection_counter( STACK_ID, &lost_connection_cnt )];

        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->return_code =
                rc_lut[smtc_modem_lorawan_get_lost_connection_counter_since_s( STACK_ID, &lost_connection_since_s )];
        }
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( lost_connection_cnt >> 8 ) & 0xff;
            cmd_output->buffer[1] = ( lost_connection_cnt & 0xff );

            cmd_output->buffer[2] = ( lost_connection_since_s >> 24 ) & 0xff;
            cmd_output->buffer[3] = ( lost_connection_since_s >> 16 ) & 0xff;
            cmd_output->buffer[4] = ( lost_connection_since_s >> 8 ) & 0xff;
            cmd_output->buffer[5] = ( lost_connection_since_s & 0xff );

            cmd_output->length = 6;
        }
        break;
    }
    case CMD_SET_ADR_ACK_LIMIT_DELAY:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_set_adr_ack_limit_delay( STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1] )];
        break;
    }
    case CMD_GET_ADR_ACK_LIMIT_DELAY:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_get_adr_ack_limit_delay( STACK_ID, &cmd_output->buffer[0], &cmd_output->buffer[1] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 2;
        }
        break;
    }
    case CMD_GET_LORAWAN_TIME:
    {
        uint32_t gps_time_s       = 0;
        uint32_t gps_fractional_s = 0;

        cmd_output->return_code = rc_lut[smtc_modem_get_lorawan_mac_time( STACK_ID, &gps_time_s, &gps_fractional_s )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( gps_time_s >> 24 ) & 0xFF;
            cmd_output->buffer[1] = ( gps_time_s >> 16 ) & 0xFF;
            cmd_output->buffer[2] = ( gps_time_s >> 8 ) & 0xFF;
            cmd_output->buffer[3] = ( gps_time_s & 0xFF );
            cmd_output->buffer[4] = ( gps_fractional_s >> 24 ) & 0xFF;
            cmd_output->buffer[5] = ( gps_fractional_s >> 16 ) & 0xFF;
            cmd_output->buffer[6] = ( gps_fractional_s >> 8 ) & 0xFF;
            cmd_output->buffer[7] = ( gps_fractional_s & 0xFF );
            cmd_output->length    = 8;
        }
        break;
    }
    case CMD_SET_JOIN_DR_DISTRIBUTION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_adr_set_join_distribution( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_LORAWAN_MAC_REQUEST:
    {
        cmd_output->return_code = rc_lut[smtc_modem_trig_lorawan_mac_request( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_LINK_CHECK_DATA:
    {
        uint8_t margin = 0;
        uint8_t gw_cnt = 0;

        cmd_output->return_code = rc_lut[smtc_modem_get_lorawan_link_check_data( STACK_ID, &margin, &gw_cnt )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = margin;
            cmd_output->buffer[1] = gw_cnt;
            cmd_output->length    = 2;
        }
        break;
    }
    case CMD_DEBUG_CONNECT_WITH_ABP:
    {
        uint32_t dev_addr = 0;

        dev_addr = cmd_input->buffer[0] << 24;
        dev_addr |= cmd_input->buffer[1] << 16;
        dev_addr |= cmd_input->buffer[2] << 8;
        dev_addr |= cmd_input->buffer[3];
        cmd_output->return_code = rc_lut[smtc_modem_debug_connect_with_abp( STACK_ID, dev_addr, &cmd_input->buffer[4],
                                                                            &cmd_input->buffer[20] )];
        break;
    }
    case CMD_CSMA_SET_STATE:
    {
#if defined( LR11XX ) || defined( SX126X ) || defined( LR20XX )
        cmd_output->return_code = rc_lut[smtc_modem_csma_set_state( STACK_ID, cmd_input->buffer[0] )];
#else
        cmd_output->return_code = CMD_RC_NOT_IMPLEMENTED;
#endif
        break;
    }
    case CMD_CSMA_GET_STATE:
    {
#if defined( LR11XX ) || defined( SX126X ) || defined( LR20XX )
        bool enable = false;

        cmd_output->return_code = rc_lut[smtc_modem_csma_get_state( STACK_ID, &enable )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = enable;
            cmd_output->length    = 1;
        }
#else
        cmd_output->return_code = CMD_RC_NOT_IMPLEMENTED;
#endif
        break;
    }
    case CMD_CSMA_SET_PARAMETERS:
    {
#if defined( LR11XX ) || defined( SX126X ) || defined( LR20XX )
        if( cmd_input->buffer[1] > 1 )
        {
            /* bo_enabled is a bool */
            cmd_output->return_code = CMD_RC_INVALID;
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_csma_set_parameters(
                STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], cmd_input->buffer[2] )];
        }
#else
        cmd_output->return_code = CMD_RC_NOT_IMPLEMENTED;
#endif
        break;
    }
    case CMD_CSMA_GET_PARAMETERS:
    {
#if defined( LR11XX ) || defined( SX126X ) || defined( LR20XX )
        uint8_t max_ch_change = 0;
        bool    bo_enabled    = false;
        uint8_t nb_bo_max     = 0;

        cmd_output->return_code =
            rc_lut[smtc_modem_csma_get_parameters( STACK_ID, &max_ch_change, &bo_enabled, &nb_bo_max )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = max_ch_change;
            cmd_output->buffer[1] = bo_enabled;
            cmd_output->buffer[2] = nb_bo_max;
            cmd_output->length    = 3;
        }
#else
        cmd_output->return_code = CMD_RC_NOT_IMPLEMENTED;
#endif
        break;
    }
    case CMD_STREAM_INIT:
    {
        uint8_t port             = cmd_input->buffer[0];
        uint8_t cipher           = cmd_input->buffer[1];
        uint8_t redundancy_ratio = cmd_input->buffer[2];

        cmd_output->return_code = rc_lut[smtc_modem_stream_init( STACK_ID, port, cipher, redundancy_ratio )];
        cmd_output->length      = 0;
        break;
    }
    case CMD_STREAM_ADD_DATA:
    {
        uint8_t* data     = &( cmd_input->buffer[0] );
        uint8_t  data_len = cmd_input->length;

        cmd_output->return_code = rc_lut[smtc_modem_stream_add_data( STACK_ID, data, data_len )];
        cmd_output->length      = 0;
        break;
    }
    case CMD_STREAM_STATUS:
    {
        uint16_t pending = 0;
        uint16_t free    = 0;

        cmd_output->return_code = rc_lut[smtc_modem_stream_status( STACK_ID, &pending, &free )];

        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length    = 4;
            cmd_output->buffer[0] = ( pending >> 8 ) & 0xff;
            cmd_output->buffer[1] = pending & 0xff;
            cmd_output->buffer[2] = ( free >> 8 ) & 0xff;
            cmd_output->buffer[3] = free & 0xff;
        }
        break;
    }
    case CMD_LFU_INIT:
    {
        uint16_t size          = 0;
        uint16_t average_delay = 0;

        size = cmd_input->buffer[2] << 8;
        size |= cmd_input->buffer[3];

        average_delay = cmd_input->buffer[4] << 8;
        average_delay |= cmd_input->buffer[5];

        file_size           = size;
        upload_status       = UPLOAD_NOT_INIT;
        upload_current_size = 0;
        /* empty the file_storage buffer */
        memset( file_store, 0, FILE_UPLOAD_MAX_SIZE );

        cmd_output->return_code = rc_lut[smtc_modem_file_upload_init(
            STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], file_store, file_size, average_delay )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            upload_status = UPLOAD_INIT;
        }
        break;
    }
    case CMD_LFU_DATA:
    {
        /* First check if modem is in test mode */
        if( modem_in_test_mode == true )
        {
            cmd_output->return_code = CMD_RC_BUSY;
        }
        else
        {
            /* here is a emulation of upload data to keep compliant with modem-e */
            if( ( upload_status != UPLOAD_INIT ) && ( upload_status != UPLOAD_DATA_ON_GOING ) )
            {
                cmd_output->return_code = CMD_RC_NOT_INIT;
                SMTC_HAL_TRACE_ERROR( "Upload file data, not init\n" );
            }
            else if( &cmd_input->buffer[0] == NULL )
            {
                cmd_output->return_code = CMD_RC_NOT_INIT;
                SMTC_HAL_TRACE_ERROR( "Upload file data, null\n" );
            }
            else if( ( upload_current_size + cmd_input->length ) > file_size )
            {
                cmd_output->return_code = CMD_RC_INVALID;
                SMTC_HAL_TRACE_ERROR( "Upload file data, size invalid\n" );
            }
            else if( upload_status == UPLOAD_STARTED )
            {
                cmd_output->return_code = CMD_RC_INVALID;
                SMTC_HAL_TRACE_ERROR( "Upload file still on going\n" );
            }
            else
            {
                memcpy( ( uint8_t* ) file_store + upload_current_size, &cmd_input->buffer[0], cmd_input->length );
                upload_current_size += cmd_input->length;

                upload_status = UPLOAD_DATA_ON_GOING;
            }
        }
        break;
    }
    case CMD_LFU_START:
    {
        uint32_t input_crc = 0;

        input_crc |= cmd_input->buffer[0] << 24;
        input_crc |= cmd_input->buffer[1] << 16;
        input_crc |= cmd_input->buffer[2] << 8;
        input_crc |= cmd_input->buffer[3];

        /* check if file_size defined at upload_init cmd is equal to the actual received
         * length
         */
        if( file_size != upload_current_size )
        {
            cmd_output->return_code = CMD_RC_BAD_SIZE;
            SMTC_HAL_TRACE_ERROR( "Data size uploaded does not correspond to what was defined\n" );
            smtc_modem_file_upload_reset( STACK_ID );
        }
        else if( input_crc != cmd_parser_crc( file_store, file_size ) )
        {
            cmd_output->return_code = CMD_RC_BAD_CRC;
            SMTC_HAL_TRACE_ERROR( "Bad crc after uploading file data\n" );
            smtc_modem_file_upload_reset( STACK_ID );
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_file_upload_start( STACK_ID )];
            if( cmd_output->return_code == CMD_RC_OK )
            {
                upload_status = UPLOAD_STARTED;
            }
        }
        break;
    }
    case CMD_LFU_RESET:
    {
        cmd_output->return_code = rc_lut[smtc_modem_file_upload_reset( STACK_ID )];
        break;
    }
    case CMD_DM_ENABLE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_enable( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_DM_GET_PORT:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_get_fport( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_DM_SET_PORT:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_set_fport( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_DM_GET_INFO_INTERVAL:
    {
        smtc_modem_dm_info_interval_format_t format   = SMTC_MODEM_DM_INFO_INTERVAL_IN_SECOND;
        uint8_t                              interval = 0;

        cmd_output->return_code = rc_lut[smtc_modem_dm_get_info_interval( STACK_ID, &format, &interval )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = interval & 0x3F;
            cmd_output->buffer[0] |= ( ( uint8_t ) format << 6 ) & 0xC0;
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_DM_SET_INFO_INTERVAL:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_set_info_interval(
            STACK_ID, ( cmd_input->buffer[0] >> 6 ) & 0x03, cmd_input->buffer[0] & 0x3F )];
        break;
    }
    case CMD_DM_GET_PERIODIC_INFO_FIELDS:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_dm_get_periodic_info_fields( STACK_ID, &cmd_output->buffer[0], &cmd_output->length )];
        if( cmd_output->return_code != CMD_RC_OK )
        {
            cmd_output->length = 0;
        }
        break;
    }
    case CMD_DM_SET_PERIODIC_INFO_FIELDS:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_dm_set_periodic_info_fields( STACK_ID, &cmd_input->buffer[0], cmd_input->length )];
        break;
    }
    case CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_dm_request_immediate_info_field( STACK_ID, &cmd_input->buffer[0], cmd_input->length )];
        break;
    }
    case CMD_DM_SET_USER_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_set_user_data( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_DM_GET_USER_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_get_user_data( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = SMTC_MODEM_DM_USER_DATA_LENGTH;
        }
        break;
    }
    case CMD_GET_STATUS:
    {
        smtc_modem_status_mask_t status_mask = { 0 };

        cmd_output->return_code = rc_lut[smtc_modem_get_status( STACK_ID, &status_mask )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( uint8_t ) ( status_mask );
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_GET_SUSPEND_RADIO_COMMUNICATIONS:
    {
        bool suspend = false;

        cmd_output->return_code = rc_lut[smtc_modem_get_suspend_radio_communications( STACK_ID, &suspend )];
        cmd_output->buffer[0]   = suspend;
        cmd_output->length      = 1;
        break;
    }
    case CMD_SUSPEND_RADIO_COMMUNICATIONS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_suspend_radio_communications( cmd_input->buffer[0] )];
        break;
    }
    case CMD_DM_HANDLE_ALCSYNC:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_handle_alcsync( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_SET_APPKEY:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_appkey( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF:
    {
        bool enabled = true;

        cmd_output->return_code = rc_lut[smtc_modem_get_join_duty_cycle_backoff_bypass( STACK_ID, &enabled )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = enabled;
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_SET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_set_join_duty_cycle_backoff_bypass( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_MODEM_GET_CRASHLOG:
    {
        uint8_t crash_string_length = 0;

        cmd_output->buffer[0] = smtc_modem_hal_crashlog_get_status( );
        smtc_modem_hal_crashlog_restore( &cmd_output->buffer[1], &crash_string_length );

        // Pad the remaining bytes so that cmd output is always same size
        memset( &cmd_output->buffer[crash_string_length + 1], 0, CRASH_LOG_SIZE - crash_string_length );

        /* clear status but not the data to be read if needed */
        smtc_modem_hal_crashlog_set_status( false );

        cmd_output->length      = CRASH_LOG_SIZE + 1; /* +1 crashlog status */
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_MODEM_GET_REPORT_ALL_DOWNLINKS_TO_USER:
    {
        bool report_all_downlinks = false;
        cmd_output->return_code =
            rc_lut[smtc_modem_get_report_all_downlinks_to_user( STACK_ID, &report_all_downlinks )];

        cmd_output->buffer[0] = report_all_downlinks;
        cmd_output->length    = 1;

        break;
    }
    case CMD_MODEM_SET_REPORT_ALL_DOWNLINKS_TO_USER:
    {
        if( cmd_input->buffer[0] < 2 )  // check Bool value
        {
            cmd_output->return_code =
                rc_lut[smtc_modem_set_report_all_downlinks_to_user( STACK_ID, cmd_input->buffer[0] )];
        }
        else
        {
            cmd_output->return_code = CMD_RC_INVALID;
        }

        break;
    }
#if defined( CONFIG_LORA_BASICS_MODEM_STORE_AND_FORWARD )
    case CMD_STORE_AND_FORWARD_SET_STATE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_store_and_forward_set_state( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_STORE_AND_FORWARD_GET_STATE:
    {
        smtc_modem_store_and_forward_state_t state = SMTC_MODEM_STORE_AND_FORWARD_DISABLE;

        cmd_output->return_code = rc_lut[smtc_modem_store_and_forward_get_state( STACK_ID, &state )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( uint8_t ) ( state );
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_STORE_AND_FORWARD_ADD_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_store_and_forward_flash_add_data(
            STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], &cmd_input->buffer[2], cmd_input->length - 2 )];
        break;
    }
    case CMD_STORE_AND_FORWARD_CLEAR_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_store_and_forward_flash_clear_data( STACK_ID )];
        break;
    }
    case CMD_STORE_AND_FORWARD_GET_FREE_SLOT:
    {
        uint32_t capacity  = 0;
        uint32_t free_slot = 0;

        cmd_output->return_code =
            rc_lut[smtc_modem_store_and_forward_flash_get_number_of_free_slot( STACK_ID, &capacity, &free_slot )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t idx = 0;

            cmd_output->buffer[idx++] = ( capacity >> 24 ) & 0xFF;
            cmd_output->buffer[idx++] = ( capacity >> 16 ) & 0xFF;
            cmd_output->buffer[idx++] = ( capacity >> 8 ) & 0xFF;
            cmd_output->buffer[idx++] = ( capacity & 0xFF );
            cmd_output->buffer[idx++] = ( free_slot >> 24 ) & 0xFF;
            cmd_output->buffer[idx++] = ( free_slot >> 16 ) & 0xFF;
            cmd_output->buffer[idx++] = ( free_slot >> 8 ) & 0xFF;
            cmd_output->buffer[idx++] = ( free_slot & 0xFF );

            cmd_output->length = idx;
        }
        break;
    }
#endif /* CONFIG_LORA_BASICS_MODEM_STORE_AND_FORWARD */

#if defined( ADD_APP_GEOLOCATION )
    case CMD_GNSS_SCAN:
    {
        smtc_modem_gnss_mode_t mode        = cmd_input->buffer[0];
        uint32_t               start_delay = 0;

        start_delay |= cmd_input->buffer[1] << 24;
        start_delay |= cmd_input->buffer[2] << 16;
        start_delay |= cmd_input->buffer[3] << 8;
        start_delay |= cmd_input->buffer[4];
        cmd_output->return_code = rc_lut[smtc_modem_gnss_scan( STACK_ID, mode, start_delay )];
        break;
    }
    case CMD_GNSS_SCAN_CANCEL:
    {
        cmd_output->return_code = rc_lut[smtc_modem_gnss_scan_cancel( STACK_ID )];
        break;
    }
    case CMD_GNSS_GET_EVENT_DATA_SCAN_DONE:
    {
        /* Reset value of static scan data saved struct */
        memset( &gnss_scan_data, 0, sizeof( smtc_modem_gnss_event_data_scan_done_t ) );

        /* Get the value of the struct */
        gnss_scan_done_rc       = rc_lut[smtc_modem_gnss_get_event_data_scan_done( STACK_ID, &gnss_scan_data )];
        cmd_output->return_code = gnss_scan_done_rc;

        if( gnss_scan_done_rc == CMD_RC_OK )
        {
            /* Fill the output buffer */
            uint8_t scan_done_offset = 0;

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.is_valid;

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.token;

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.nb_scans_valid;

            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.power_consumption_nah >> 24 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.power_consumption_nah >> 16 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.power_consumption_nah >> 8 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.power_consumption_nah & 0xff );

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.context.mode;

            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.context.almanac_crc >> 24 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.context.almanac_crc >> 16 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.context.almanac_crc >> 8 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.context.almanac_crc & 0xff );

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.indoor_detected;

            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.navgroup_duration_ms >> 24 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.navgroup_duration_ms >> 16 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.navgroup_duration_ms >> 8 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.navgroup_duration_ms ) & 0xff;

            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.timestamp >> 24 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.timestamp >> 16 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.timestamp >> 8 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.timestamp ) & 0xff;

            cmd_output->length = scan_done_offset;
        }
        break;
    }
    case CMD_GNSS_GET_SCAN_DONE_RAW_DATA_LIST:
    {
        cmd_output->return_code = gnss_scan_done_rc;
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t raw_data_offset = 0;

            for( uint8_t scan_index = 0; scan_index < gnss_scan_data.nb_scans_valid; scan_index++ )
            {
                cmd_output->buffer[raw_data_offset++] = ( gnss_scan_data.scans[scan_index].timestamp >> 24 ) & 0xff;
                cmd_output->buffer[raw_data_offset++] = ( gnss_scan_data.scans[scan_index].timestamp >> 16 ) & 0xff;
                cmd_output->buffer[raw_data_offset++] = ( gnss_scan_data.scans[scan_index].timestamp >> 8 ) & 0xff;
                cmd_output->buffer[raw_data_offset++] = ( gnss_scan_data.scans[scan_index].timestamp & 0xff );

                cmd_output->buffer[raw_data_offset++] = gnss_scan_data.scans[scan_index].nav_size;

                memcpy( &cmd_output->buffer[raw_data_offset], gnss_scan_data.scans[scan_index].nav,
                        gnss_scan_data.scans[scan_index].nav_size );

                raw_data_offset += gnss_scan_data.scans[scan_index].nav_size;
            }
            /* At the end of the scan loop the size of the buff is known */
            cmd_output->length = raw_data_offset;
        }
        break;
    }
    case CMD_GNSS_GET_SCAN_DONE_METADATA_LIST:
    {
        cmd_output->return_code = gnss_scan_done_rc;
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t metadata_offset = 0;

            for( uint8_t scan_index = 0; scan_index < gnss_scan_data.nb_scans_valid; scan_index++ )
            {
                uint32_t lat_x10000 =
                    ( uint32_t ) ( gnss_scan_data.scans[scan_index].aiding_position.latitude * 10000 );

                cmd_output->buffer[metadata_offset++] = ( lat_x10000 >> 24 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( lat_x10000 >> 16 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( lat_x10000 >> 8 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( lat_x10000 & 0xff );

                uint32_t long_x10000 =
                    ( uint32_t ) ( gnss_scan_data.scans[scan_index].aiding_position.longitude * 10000 );

                cmd_output->buffer[metadata_offset++] = ( long_x10000 >> 24 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( long_x10000 >> 16 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( long_x10000 >> 8 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( long_x10000 & 0xff );

                cmd_output->buffer[metadata_offset++] = gnss_scan_data.scans[scan_index].scan_mode_launched;

                cmd_output->buffer[metadata_offset++] =
                    ( gnss_scan_data.scans[scan_index].scan_duration_ms >> 24 ) & 0xff;
                cmd_output->buffer[metadata_offset++] =
                    ( gnss_scan_data.scans[scan_index].scan_duration_ms >> 16 ) & 0xff;
                cmd_output->buffer[metadata_offset++] =
                    ( gnss_scan_data.scans[scan_index].scan_duration_ms >> 8 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( gnss_scan_data.scans[scan_index].scan_duration_ms & 0xff );
            }
            /* At the end of the scan loop the size of the buff is known */
            cmd_output->length = metadata_offset;
        }
        break;
    }

    case CMD_GNSS_GET_SCAN_DONE_SCAN_SV:
    {
        cmd_output->return_code = gnss_scan_done_rc;
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t sv_index = 0;

            for( uint8_t scan_index = 0; scan_index < gnss_scan_data.nb_scans_valid; scan_index++ )
            {
                cmd_output->buffer[sv_index++] = gnss_scan_data.scans[scan_index].nb_svs;

                for( uint8_t nav_index = 0; nav_index < gnss_scan_data.scans[scan_index].nb_svs; nav_index++ )
                {
                    cmd_output->buffer[sv_index++] = gnss_scan_data.scans[scan_index].info_svs[nav_index].satellite_id;
                    cmd_output->buffer[sv_index++] = gnss_scan_data.scans[scan_index].info_svs[nav_index].cnr;
                    cmd_output->buffer[sv_index++] =
                        ( gnss_scan_data.scans[scan_index].info_svs[nav_index].doppler >> 8 ) & 0xff;
                    cmd_output->buffer[sv_index++] =
                        ( gnss_scan_data.scans[scan_index].info_svs[nav_index].doppler & 0xff );
                }
            }

            /* At the end of the scan loop the size of the buff is known */
            cmd_output->length = sv_index;
        }
        break;
    }
    case CMD_GNSS_GET_EVENT_DATA_TERMINATED:
    {
        smtc_modem_gnss_event_data_terminated_t gnss_data_terminated = { 0 };

        cmd_output->return_code = rc_lut[smtc_modem_gnss_get_event_data_terminated( STACK_ID, &gnss_data_terminated )];

        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = gnss_data_terminated.nb_scans_sent;
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_GNSS_SET_CONST:
    {
        cmd_output->return_code = rc_lut[smtc_modem_gnss_set_constellations( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GNSS_SET_PORT:
    {
        cmd_output->return_code = rc_lut[smtc_modem_gnss_set_port( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GNSS_SCAN_AGGREGATE:
    {
        smtc_modem_gnss_scan_aggregate( STACK_ID, cmd_input->buffer[0] );
        /* Above function do not have return code */
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_GNSS_SEND_MODE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_gnss_send_mode( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GNSS_ALM_DEMOD_START:
    {
        cmd_output->return_code = rc_lut[smtc_modem_almanac_demodulation_start( STACK_ID )];
        break;
    }
    case CMD_GNSS_ALM_DEMOD_SET_CONSTEL:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_almanac_demodulation_set_constellations( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GNSS_ALM_DEMOD_GET_EVENT_DATA_ALM_UPD:
    {
        smtc_modem_almanac_demodulation_event_data_almanac_update_t event_data_alm_update = { 0 };
        cmd_output->return_code =
            rc_lut[smtc_modem_almanac_demodulation_get_event_data_almanac_update( STACK_ID, &event_data_alm_update )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t index = 0;

            cmd_output->buffer[index++] = event_data_alm_update.status_gps;
            cmd_output->buffer[index++] = event_data_alm_update.status_beidou;
            cmd_output->buffer[index++] = event_data_alm_update.update_progress_gps;
            cmd_output->buffer[index++] = event_data_alm_update.update_progress_beidou;

            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_done >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_done >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_done >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_done & 0xff );

            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_success >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_success >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_success >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_success & 0xff );

            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_aborted_by_rp >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_aborted_by_rp >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_aborted_by_rp >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_aborted_by_rp & 0xff );

            cmd_output->buffer[index++] = ( event_data_alm_update.stat_cumulative_timings_s >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_cumulative_timings_s >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_cumulative_timings_s >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_cumulative_timings_s & 0xff );

            cmd_output->buffer[index++] = ( event_data_alm_update.power_consumption_nah >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.power_consumption_nah >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.power_consumption_nah >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.power_consumption_nah & 0xff );

            cmd_output->length = index;
        }
        break;
    }
#if defined( ADD_ALMANAC )
    case CMD_CLOUD_ALMANAC_START:
    {
        cmd_output->return_code = rc_lut[smtc_modem_almanac_start( STACK_ID )];
        break;
    }
    case CMD_CLOUD_ALMANAC_STOP:
    {
        cmd_output->return_code = rc_lut[smtc_modem_almanac_stop( STACK_ID )];
        break;
    }
#endif
    case CMD_WIFI_SCAN_START:
    {
        uint32_t start_delay = 0;

        start_delay |= cmd_input->buffer[0] << 24;
        start_delay |= cmd_input->buffer[1] << 16;
        start_delay |= cmd_input->buffer[2] << 8;
        start_delay |= cmd_input->buffer[3];
        cmd_output->return_code = rc_lut[smtc_modem_wifi_scan( STACK_ID, start_delay )];
        break;
    }
    case CMD_WIFI_SCAN_CANCEL:
    {
        cmd_output->return_code = rc_lut[smtc_modem_wifi_scan_cancel( STACK_ID )];
        break;
    }
    case CMD_WIFI_GET_SCAN_DONE_SCAN_DATA:
    {
        smtc_modem_wifi_event_data_scan_done_t wifi_scan_done_data = { 0 };

        cmd_output->return_code = rc_lut[smtc_modem_wifi_get_event_data_scan_done( STACK_ID, &wifi_scan_done_data )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t index = 0;

            cmd_output->buffer[index++] = wifi_scan_done_data.nbr_results;

            cmd_output->buffer[index++] = ( wifi_scan_done_data.power_consumption_nah >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.power_consumption_nah >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.power_consumption_nah >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.power_consumption_nah & 0xff );

            cmd_output->buffer[index++] = ( wifi_scan_done_data.scan_duration_ms >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.scan_duration_ms >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.scan_duration_ms >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.scan_duration_ms & 0xff );

            for( uint8_t scan_index = 0; scan_index < wifi_scan_done_data.nbr_results; scan_index++ )
            {
                /* copy LR11XX_WIFI_MAC_ADDRESS_LENGTH bytes of mac address */
                memcpy( &cmd_output->buffer[index], wifi_scan_done_data.results[scan_index].mac_address,
                        LR11XX_WIFI_MAC_ADDRESS_LENGTH );
                index += LR11XX_WIFI_MAC_ADDRESS_LENGTH;

                cmd_output->buffer[index++] = wifi_scan_done_data.results[scan_index].channel;
                cmd_output->buffer[index++] = wifi_scan_done_data.results[scan_index].type;
                cmd_output->buffer[index++] = wifi_scan_done_data.results[scan_index].rssi;
            }
            cmd_output->length = index;
        }
        break;
    }
    case CMD_WIFI_GET_EVENT_DATA_TERMINATED:
    {
        smtc_modem_wifi_event_data_terminated_t wifi_data_terminated = { 0 };

        cmd_output->return_code = rc_lut[smtc_modem_wifi_get_event_data_terminated( STACK_ID, &wifi_data_terminated )];

        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = wifi_data_terminated.nb_scans_sent;
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_WIFI_SET_PORT:
    {
        cmd_output->return_code = rc_lut[smtc_modem_wifi_set_port( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_WIFI_SEND_MODE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_wifi_send_mode( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_WIFI_SET_PAYLOAD_FORMAT:
    {
        smtc_modem_wifi_set_payload_format( STACK_ID, cmd_input->buffer[0] );
        /* Above function do not have return code */
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_LR11XX_RADIO_READ:
    {
        /* First check if modem is in test mode */
        if( modem_in_test_mode == true )
        {
            cmd_output->return_code = CMD_RC_BUSY;
        }
        else
        {
#if defined( CONFIG_SEMTECH_LR11XX )
            uint8_t command_length = cmd_input->buffer[0];
            uint8_t command[255]   = { 0 };

            memcpy( command, &cmd_input->buffer[1], command_length );

            uint8_t data_length = cmd_input->buffer[command_length + 1];
            uint8_t data[255]   = { 0 };

            if( lr11xx_hal_read( transceiver_context, command, command_length, data, data_length ) !=
                LR11XX_HAL_STATUS_OK )
            {
                cmd_output->return_code = CMD_RC_FAIL;
            }
            else
            {
                cmd_output->return_code = CMD_RC_OK;
            }
            memcpy( cmd_output->buffer, data, data_length );
            cmd_output->length = data_length;
#else
            cmd_output->return_code = CMD_RC_FAIL;
#endif
        }
        break;
    }
    case CMD_LR11XX_RADIO_WRITE:
    {
        /* First check if modem is in test mode */
        if( modem_in_test_mode == true )
        {
            cmd_output->return_code = CMD_RC_BUSY;
        }
        else
        {
#if defined( CONFIG_SEMTECH_LR11XX )
            uint8_t command_length = cmd_input->buffer[0];
            uint8_t command[255]   = { 0 };

            memcpy( command, &cmd_input->buffer[1], command_length );

            uint8_t data_length = cmd_input->buffer[command_length + 1];
            uint8_t data[255]   = { 0 };

            memcpy( data, &cmd_input->buffer[command_length + 2], data_length );
            if( lr11xx_hal_write( transceiver_context, command, command_length, data, data_length ) !=
                LR11XX_HAL_STATUS_OK )
            {
                cmd_output->return_code = CMD_RC_FAIL;
            }
            else
            {
                cmd_output->return_code = CMD_RC_OK;
            }
            cmd_output->length = 0;
#else
            cmd_output->return_code = CMD_RC_FAIL;
#endif
        }
        break;
    }

#endif /* ADD_APP_GEOLOCATION */

    /* CMD_USP_GET_RESULTS removed - use NHM_CMD_USP_GET_RESULTS via CMD_NHM_EXTENDED instead */
    case CMD_USP_SUBMIT:
    {
        SMTC_HAL_TRACE_INFO( "CMD_USP_SUBMIT: Received %d bytes\n", cmd_input->length );

        if( cmd_input->length == 0 )
        {
            SMTC_HAL_TRACE_ERROR( "CMD_USP_SUBMIT: No payload received\n" );
            cmd_output->return_code = CMD_RC_INVALID;
            cmd_output->length      = 0;
            break;
        }

        // unserialize pb context
        smtc_rac_lora_request_pb_t pb_rac_lora_call = smtc_rac_lora_request_pb_t_init_zero;
        pb_istream_t               stream           = pb_istream_from_buffer( cmd_input->buffer, cmd_input->length );

        if( !pb_decode( &stream, smtc_rac_lora_request_pb_t_fields, &pb_rac_lora_call ) )
        {
            SMTC_HAL_TRACE_ERROR( "CMD_USP_SUBMIT: Failed to decode protobuf request\n" );
            cmd_output->return_code = CMD_RC_INVALID;
            cmd_output->length      = 0;
            break;
        }

        SMTC_HAL_TRACE_INFO( "CMD_USP_SUBMIT: Protobuf decoded successfully\n" );
        SMTC_HAL_TRACE_INFO( "  Radio Id: %d\n", pb_rac_lora_call.radio_access_id );
        SMTC_HAL_TRACE_INFO( "  TX mode: %s\n", pb_rac_lora_call.rac_config.radio_params.is_tx ? "true" : "false" );
        SMTC_HAL_TRACE_INFO( "  Frequency: %u Hz\n", pb_rac_lora_call.rac_config.radio_params.frequency_in_hz );
        SMTC_HAL_TRACE_INFO( "  TX Power: %u dBm\n", pb_rac_lora_call.rac_config.radio_params.tx_power_in_dbm );
        SMTC_HAL_TRACE_INFO( "  TX Payload size: %u bytes\n",
                             ( uint32_t ) pb_rac_lora_call.rac_config.radio_params.tx_size );
        SMTC_HAL_TRACE_INFO( "  RX Max size: %u bytes\n", pb_rac_lora_call.rac_config.radio_params.max_rx_size );

        rac_context_data_t* rac_context_data = get_rac_context_data_from_handle( pb_rac_lora_call.radio_access_id );
        if( rac_context_data == NULL )
        {
            SMTC_HAL_TRACE_ERROR( "CMD_USP_SUBMIT: Failed to get rac context data\n" );
            cmd_output->return_code = CMD_RC_FAIL;
            cmd_output->length      = 0;
            break;
        }

        // Convert to native structure - use existing pre-allocated buffers
        if( !rac_convert_context_from_pb( &( pb_rac_lora_call.rac_config ), rac_context_data->rac_context ) )
        {
            SMTC_HAL_TRACE_ERROR( "CMD_USP_SUBMIT: Failed to convert protobuf to native context\n" );
            cmd_output->return_code = CMD_RC_FAIL;
            cmd_output->length      = 0;
            break;
        }

        SMTC_HAL_TRACE_INFO( "CMD_USP_SUBMIT: Context converted to native successfully\n" );

        // The first time, the python script do not know embedded side absolute time so we set it to the current time +
        // processing time
        if( rac_context_data->rac_context->scheduler_config.start_time_ms == 0 )
        {
            if( rac_context_data->rac_context->scheduler_config.scheduling == SMTC_RAC_ASAP_TRANSACTION )
            {
                rac_context_data->rac_context->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + 100;
            }
            else
            {
                rac_context_data->rac_context->scheduler_config.start_time_ms =
                    smtc_modem_hal_get_time_in_ms( ) + 100;  // 10 : processing time
            }
        }

        // TODO: Use the native context with RAC API
        // This would require a radio_id - for now we just validate the conversion worked
        rac_context_data->rac_context->scheduler_config.callback_pre_radio_transaction = NULL;
        SMTC_HAL_TRACE_INFO(
            "CMD_USP_SUBMIT: rac_context->scheduler_config.callback_post_radio_transaction (ret: 0x%p)\n",
            rac_context_data->rac_context->scheduler_config.callback_post_radio_transaction );
        smtc_rac_return_code_t ret = smtc_rac_submit_radio_transaction( pb_rac_lora_call.radio_access_id );

        // Store return code for CMD_USP_GET_RESULTS
        rac_context_data->last_rac_return_code = ret;

        SMTC_HAL_TRACE_INFO( "CMD_USP_SUBMIT: Context processing completed successfully (ret: %d)\n", ret );
        cmd_output->return_code =
            ( ret == SMTC_RAC_SUCCESS ) ? CMD_RC_OK : CMD_RC_FAIL;  // SSA-TBD : do this better : lut ?
        cmd_output->length = 0;
        break;
    }
    case CMD_USP_CAD:
    {
        // SSA-TBD
        cmd_output->return_code = CMD_RC_OK;
        cmd_output->length      = 0;
        break;
    }
    case CMD_USP_OPEN:
    {
        smtc_rac_priority_t priority         = ( smtc_rac_priority_t ) ( cmd_input->buffer[0] );
        rac_context_data_t* rac_context_data = get_rac_context_data_from_priority( priority );
        if( rac_context_data == NULL )
        {
            cmd_output->return_code = CMD_RC_FAIL;
            cmd_output->buffer[0]   = priority;
            cmd_output->length      = 1;
            break;
        }
        raz_rac_context_data( rac_context_data );
        rac_context_data->radio_handle = smtc_rac_open_radio( priority );
        if( rac_context_data->radio_handle == RAC_INVALID_RADIO_ID )
        {
            cmd_output->return_code = CMD_RC_FAIL;
            cmd_output->buffer[0]   = RAC_INVALID_RADIO_ID;
            cmd_output->length      = 1;
            break;
        }
        rac_context_data->rac_context = smtc_rac_get_context( rac_context_data->radio_handle );
        if( rac_context_data->rac_context == NULL )
        {
            cmd_output->return_code = CMD_RC_FAIL;
            cmd_output->buffer[0]   = rac_context_data->radio_handle;
            cmd_output->length      = 1;
            break;
        }
        rac_context_data->rac_context->scheduler_config.callback_post_radio_transaction =
            rac_context_data->post_callback;
        cmd_parser_update_rac_context( rac_context_data );
        cmd_output->return_code = CMD_RC_OK;
        cmd_output->buffer[0]   = rac_context_data->radio_handle;
        cmd_output->length      = 1;
        break;
    }
    case CMD_USP_CLOSE:
    {
        uint8_t                radio_id = ( uint8_t ) ( cmd_input->buffer[0] );
        smtc_rac_return_code_t ret      = smtc_rac_close_radio( radio_id );
        if( ret == SMTC_RAC_SUCCESS )
        {
            cmd_output->return_code = CMD_RC_OK;
        }
        else
        {
            // SSA-TBD : check what to do with true error code
            cmd_output->return_code = CMD_RC_FAIL;
        }
        cmd_output->length = 0;
        break;
    }
    case CMD_USP_ABORT:
    {
        uint8_t                radio_id = ( uint8_t ) ( cmd_input->buffer[0] );
        smtc_rac_return_code_t ret      = smtc_rac_abort_radio_submit( radio_id );
        if( ret == SMTC_RAC_SUCCESS )
        {
            cmd_output->return_code = CMD_RC_OK;
        }
        else
        {
            // SSA-TBD : check what to do with true error code
            cmd_output->return_code = CMD_RC_FAIL;
        }
        cmd_output->length = 0;
        break;
    }

    case CMD_NHM_EXTENDED:
    {
        SMTC_HAL_TRACE_INFO( "CMD_NHM_EXTENDED: Received %d bytes\n", cmd_input->length );

        if( cmd_input->length < NHM_HEADER_SIZE )
        {
            SMTC_HAL_TRACE_ERROR( "CMD_NHM_EXTENDED: Insufficient header data (%d < %d bytes)\n", cmd_input->length,
                                  NHM_HEADER_SIZE );
            cmd_output->return_code = CMD_RC_BAD_SIZE;
            cmd_output->length      = 0;
            break;
        }

        return parse_nhm_cmd( cmd_input, cmd_output );
    }

    default:
    {
        SMTC_HAL_TRACE_ERROR( "Unknown command (0x%x)\n", cmd_input->cmd_code );
        cmd_output->return_code = CMD_RC_UNKNOWN;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }
    }
    cmd_input->cmd_code = CMD_MAX;
    cmd_input->length   = 0;

    return ret;
}

cmd_parse_status_t cmd_test_parser( cmd_tst_input_t* cmd_tst_input, cmd_tst_response_t* cmd_tst_output )
{
    cmd_parse_status_t ret = PARSE_OK;

    if( ( cmd_tst_input->cmd_code >= CMD_TST_MAX ) ||
        ( host_cmd_test_tab[cmd_tst_input->cmd_code][HOST_CMD_TAB_IDX_AVAILABILITY] != 1 ) )
    {
        SMTC_HAL_TRACE_ERROR( "Unknown command test (0x%x)\n", cmd_tst_input->cmd_code );
        cmd_tst_output->return_code = CMD_RC_UNKNOWN;
        cmd_tst_output->length      = 0;
        return PARSE_ERROR;
    }

    if( cmd_test_parser_check_cmd_size( cmd_tst_input->cmd_code, cmd_tst_input->length ) == CMD_LENGTH_NOT_VALID )
    {
        SMTC_HAL_TRACE_ERROR( "Invalid size command test (0x%x)\n", cmd_tst_input->cmd_code );
        cmd_tst_output->return_code = CMD_RC_BAD_SIZE;
        cmd_tst_output->length      = 0;
        return PARSE_ERROR;
    }

    /* by default the return code is ok and length is 0 */
    cmd_tst_output->return_code = CMD_RC_OK;
    cmd_tst_output->length      = 0;

#if HAL_DBG_TRACE == HAL_FEATURE_ON
    SMTC_HAL_TRACE_WARNING( "\tCMD_TST_%s (0x%02x)\n", host_cmd_test_str[cmd_tst_input->cmd_code],
                            cmd_tst_input->cmd_code );
#endif
    switch( cmd_tst_input->cmd_code )
    {
    case CMD_TST_START:
    {
        if( strncmp( ( char* ) cmd_tst_input->buffer, "TESTTEST", 8 ) == 0 )
        {
            cmd_tst_output->return_code = rc_lut[smtc_modem_test_start( )];
            if( cmd_tst_output->return_code == CMD_RC_OK )
            {
                modem_in_test_mode = true;
            }
        }
        else
        {
            SMTC_HAL_TRACE_ERROR( "TST MODE: invalid enablement payload\n" );
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }

        break;
    }
    case CMD_TST_EXIT:
    {
        cmd_tst_output->return_code = rc_lut[smtc_modem_test_stop( )];
        if( cmd_tst_output->return_code == CMD_RC_OK )
        {
            modem_in_test_mode = false;
        }
        break;
    }
    case CMD_TST_NOP:
    {
        cmd_tst_output->return_code = rc_lut[smtc_modem_test_nop( true )];
        break;
    }
    case CMD_TST_TX_LORA:
    {
        uint32_t freq = 0;
        freq |= cmd_tst_input->buffer[0] << 24;
        freq |= cmd_tst_input->buffer[1] << 16;
        freq |= cmd_tst_input->buffer[2] << 8;
        freq |= cmd_tst_input->buffer[3];

        int8_t  pw  = cmd_tst_input->buffer[4];
        uint8_t len = cmd_tst_input->buffer[5];

        uint8_t sf = cmd_tst_input->buffer[6];
        uint8_t bw = cmd_tst_input->buffer[7];

        uint8_t cr = cmd_tst_input->buffer[8];

        uint8_t                  invert_iq   = cmd_tst_input->buffer[9] & 0x01;
        uint8_t                  crc_is_on   = cmd_tst_input->buffer[10] & 0x01;
        ral_lora_pkt_len_modes_t header_type = cmd_tst_input->buffer[11] & 0x01;

        uint32_t preamble_size = 0;
        preamble_size |= cmd_tst_input->buffer[12] << 24;
        preamble_size |= cmd_tst_input->buffer[13] << 16;
        preamble_size |= cmd_tst_input->buffer[14] << 8;
        preamble_size |= cmd_tst_input->buffer[15];

        uint32_t nb_of_tx = 0;
        nb_of_tx |= cmd_tst_input->buffer[16] << 24;
        nb_of_tx |= cmd_tst_input->buffer[17] << 16;
        nb_of_tx |= cmd_tst_input->buffer[18] << 8;
        nb_of_tx |= cmd_tst_input->buffer[19];

        uint32_t delay_ms = 0;
        delay_ms |= cmd_tst_input->buffer[20] << 24;
        delay_ms |= cmd_tst_input->buffer[21] << 16;
        delay_ms |= cmd_tst_input->buffer[22] << 8;
        delay_ms |= cmd_tst_input->buffer[23];

        uint8_t sync_word = cmd_tst_input->buffer[24];

        cmd_tst_output->return_code =
            rc_lut[smtc_modem_test_tx_lora( NULL, len, freq, pw, sf, bw, cr, sync_word, invert_iq, crc_is_on,
                                            header_type, preamble_size, nb_of_tx, delay_ms )];

        break;
    }
    case CMD_TST_TX_FSK:
    {
        uint32_t freq = 0;

        freq |= cmd_tst_input->buffer[0] << 24;
        freq |= cmd_tst_input->buffer[1] << 16;
        freq |= cmd_tst_input->buffer[2] << 8;
        freq |= cmd_tst_input->buffer[3];

        int8_t  pw  = cmd_tst_input->buffer[4];
        uint8_t len = cmd_tst_input->buffer[5];

        uint32_t nb_of_tx = 0;

        nb_of_tx |= cmd_tst_input->buffer[6] << 24;
        nb_of_tx |= cmd_tst_input->buffer[7] << 16;
        nb_of_tx |= cmd_tst_input->buffer[8] << 8;
        nb_of_tx |= cmd_tst_input->buffer[9];

        uint32_t delay_ms = 0;

        delay_ms |= cmd_tst_input->buffer[10] << 24;
        delay_ms |= cmd_tst_input->buffer[11] << 16;
        delay_ms |= cmd_tst_input->buffer[12] << 8;
        delay_ms |= cmd_tst_input->buffer[13];

        cmd_tst_output->return_code = rc_lut[smtc_modem_test_tx_fsk( NULL, len, freq, pw, nb_of_tx, delay_ms )];

        break;
    }
    case CMD_TST_TX_LRFHSS:
    {
        uint32_t freq = 0;

        freq |= cmd_tst_input->buffer[0] << 24;
        freq |= cmd_tst_input->buffer[1] << 16;
        freq |= cmd_tst_input->buffer[2] << 8;
        freq |= cmd_tst_input->buffer[3];

        int8_t  pw  = cmd_tst_input->buffer[4];
        uint8_t len = cmd_tst_input->buffer[5];

        uint8_t grid = cmd_tst_input->buffer[6];
        uint8_t bw   = cmd_tst_input->buffer[7];
        uint8_t cr   = cmd_tst_input->buffer[8];

        SMTC_HAL_TRACE_PRINTF( "grid:%u, bw:%u,cr:%u\n", grid, bw, cr );

        uint32_t nb_of_tx = 0;

        nb_of_tx |= cmd_tst_input->buffer[9] << 24;
        nb_of_tx |= cmd_tst_input->buffer[10] << 16;
        nb_of_tx |= cmd_tst_input->buffer[11] << 8;
        nb_of_tx |= cmd_tst_input->buffer[12];

        uint32_t delay_ms = 0;

        delay_ms |= cmd_tst_input->buffer[13] << 24;
        delay_ms |= cmd_tst_input->buffer[14] << 16;
        delay_ms |= cmd_tst_input->buffer[15] << 8;
        delay_ms |= cmd_tst_input->buffer[16];

        bool enable_hopping = cmd_tst_input->buffer[17] & 0x01;

        cmd_tst_output->return_code =
            rc_lut[smtc_modem_test_tx_lrfhss( NULL, len, freq, pw, cr, bw, grid, enable_hopping, nb_of_tx, delay_ms )];

        break;
    }
    case CMD_TST_TX_CW:
    {
        int8_t pw = cmd_tst_input->buffer[4];

        uint32_t freq = 0;

        freq |= cmd_tst_input->buffer[0] << 24;
        freq |= cmd_tst_input->buffer[1] << 16;
        freq |= cmd_tst_input->buffer[2] << 8;
        freq |= cmd_tst_input->buffer[3];

        cmd_tst_output->return_code = rc_lut[smtc_modem_test_tx_cw( freq, pw )];
        break;
    }
    case CMD_TST_RX_LORA:
    {
        uint32_t freq = 0;

        freq |= cmd_tst_input->buffer[0] << 24;
        freq |= cmd_tst_input->buffer[1] << 16;
        freq |= cmd_tst_input->buffer[2] << 8;
        freq |= cmd_tst_input->buffer[3];

        uint8_t sf = cmd_tst_input->buffer[4];
        uint8_t bw = cmd_tst_input->buffer[5];
        uint8_t cr = cmd_tst_input->buffer[6];

        uint8_t sync_word = cmd_tst_input->buffer[7];
        uint8_t invert_iq = cmd_tst_input->buffer[8] & 0x01;
        uint8_t crc_is_on = cmd_tst_input->buffer[9] & 0x01;

        ral_lora_pkt_len_modes_t header_type = cmd_tst_input->buffer[10] & 0x01;

        uint32_t preamble_size = 0;

        preamble_size |= cmd_tst_input->buffer[11] << 24;
        preamble_size |= cmd_tst_input->buffer[12] << 16;
        preamble_size |= cmd_tst_input->buffer[13] << 8;
        preamble_size |= cmd_tst_input->buffer[14];

        uint8_t nb_symb_timeout = cmd_tst_input->buffer[15];

        cmd_tst_output->return_code = rc_lut[smtc_modem_test_rx_lora( freq, sf, bw, cr, sync_word, invert_iq, crc_is_on,
                                                                      header_type, preamble_size, nb_symb_timeout )];

        break;
    }
    case CMD_TST_RX_FSK_CONT:
    {
        uint32_t freq = 0;

        freq |= cmd_tst_input->buffer[0] << 24;
        freq |= cmd_tst_input->buffer[1] << 16;
        freq |= cmd_tst_input->buffer[2] << 8;
        freq |= cmd_tst_input->buffer[3];

        cmd_tst_output->return_code = rc_lut[smtc_modem_test_rx_fsk_continuous( freq )];

        break;
    }
    case CMD_TST_READ_NB_PKTS_RX:
    {
        uint32_t nb_read_pkt = 0;

        cmd_tst_output->return_code = rc_lut[smtc_modem_test_get_nb_rx_packets( &nb_read_pkt )];
        cmd_tst_output->buffer[0]   = ( nb_read_pkt >> 24 ) & 0xFF;
        cmd_tst_output->buffer[1]   = ( nb_read_pkt >> 16 ) & 0xFF;
        cmd_tst_output->buffer[2]   = ( nb_read_pkt >> 8 ) & 0xFF;
        cmd_tst_output->buffer[3]   = ( nb_read_pkt & 0xFF );
        cmd_tst_output->length      = 4;
        break;
    }
    case CMD_TST_READ_LAST_RX_PKT:
    {
        int16_t rssi              = 0;
        int16_t snr               = 0;
        uint8_t rx_payload_length = 0;

        cmd_tst_output->return_code =
            rc_lut[smtc_modem_test_get_last_rx_packets( &rssi, &snr, &cmd_tst_output->buffer[4], &rx_payload_length )];
        cmd_tst_output->buffer[0] = ( snr >> 8 ) & 0xFF;
        cmd_tst_output->buffer[1] = ( snr & 0xFF );
        cmd_tst_output->buffer[2] = ( rssi >> 8 ) & 0xFF;
        cmd_tst_output->buffer[3] = ( rssi & 0xFF );
        cmd_tst_output->length    = 4 + rx_payload_length;
        break;
    }
    case CMD_TST_RSSI:
    {
        uint32_t freq = 0;

        freq |= cmd_tst_input->buffer[0] << 24;
        freq |= cmd_tst_input->buffer[1] << 16;
        freq |= cmd_tst_input->buffer[2] << 8;
        freq |= cmd_tst_input->buffer[3];

        uint16_t time_ms = 0;

        time_ms |= cmd_tst_input->buffer[4] << 8;
        time_ms |= cmd_tst_input->buffer[5];

        uint32_t bw = 0;

        bw |= cmd_tst_input->buffer[6] << 24;
        bw |= cmd_tst_input->buffer[7] << 16;
        bw |= cmd_tst_input->buffer[8] << 8;
        bw |= cmd_tst_input->buffer[9];

        cmd_tst_output->return_code = rc_lut[smtc_modem_test_rssi_lbt( freq, bw, time_ms )];
        break;
    }
    case CMD_TST_RSSI_GET:
    {
        int8_t rssi = 0;

        cmd_tst_output->return_code = rc_lut[smtc_modem_test_get_rssi( &rssi )];
        cmd_tst_output->buffer[0]   = rssi;
        cmd_tst_output->length      = 1;
        break;
    }
    case CMD_TST_RADIO_RST:
    {
        cmd_tst_output->return_code = rc_lut[smtc_modem_test_radio_reset( )];
        break;
    }
    case CMD_TST_BUSYLOOP:
    {
        /* First check if modem is in test mode */
        if( modem_in_test_mode == true )
        {
            /* Endless loop */
            while( 1 )
            {
            };
        }
        else
        {
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_PANIC:
    {
        /* First check if modem is in test mode */
        if( modem_in_test_mode == true )
        {
            SMTC_MODEM_HAL_PANIC( "TEST PANIC" );
        }
        else
        {
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_WATCHDOG:
    {
        /* First check if modem is in test mode */
        if( modem_in_test_mode == true )
        {
            hal_mcu_disable_irq( );
            /* Endless loop */
            while( 1 )
            {
            };
        }
        else
        {
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_RADIO_READ:
    {
        uint8_t command_length = cmd_tst_input->buffer[0];
        uint8_t command[255]   = { 0 };

        memcpy( command, &cmd_tst_input->buffer[1], command_length );

        uint8_t data_length = cmd_tst_input->buffer[command_length + 1];
        uint8_t data[255]   = { 0 };

        cmd_tst_output->return_code =
            rc_lut[smtc_modem_test_direct_radio_read( command, command_length, data, data_length )];
        memcpy( cmd_tst_output->buffer, data, data_length );
        cmd_tst_output->length = data_length;
        break;
    }
    case CMD_TST_RADIO_WRITE:
    {
        uint8_t command_length = cmd_tst_input->buffer[0];
        uint8_t command[255]   = { 0 };

        memcpy( command, &cmd_tst_input->buffer[1], command_length );

        uint8_t data_length = cmd_tst_input->buffer[command_length + 1];
        uint8_t data[255]   = { 0 };

        memcpy( data, &cmd_tst_input->buffer[command_length + 1], data_length );

        cmd_tst_output->return_code =
            rc_lut[smtc_modem_test_direct_radio_write( command, command_length, data, data_length )];
        cmd_tst_output->length = 0;
        break;
    }
    default:
    {
        cmd_tst_output->return_code = CMD_RC_UNKNOWN;
        cmd_tst_output->length      = 0;
        break;
    }
    }

    /* Erase test command content to avoid twice calls */
    cmd_tst_input->cmd_code = CMD_TST_MAX;
    cmd_tst_input->length   = 0;

    return ret;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void rac_post_callback( rp_status_t status, smtc_rac_priority_t priority )
{
    SMTC_HAL_TRACE_INFO( "RAC : %s\n", __func__ );

    rac_context_data_t* rac_context_data = get_rac_context_data_from_priority( priority );
    if( rac_context_data == NULL )
    {
        SMTC_HAL_TRACE_ERROR( "RAC: Invalid priority\n" );
        return;
    }

    rac_context_data->status            = status;
    rac_context_data->pending_rac_event = true;

    // Log payload info based on operation type
    if( rac_context_data->rac_context->radio_params.lora.is_tx )
    {
        // TX operation - log transmitted payload
        SMTC_HAL_TRACE_INFO( "TX size=%" PRIu32 "\n",
                             ( uint32_t ) rac_context_data->rac_context->radio_params.lora.tx_size );
        if( rac_context_data->rac_context->smtc_rac_data_buffer_setup.tx_payload_buffer &&
            rac_context_data->rac_context->radio_params.lora.tx_size > 0 )
        {
            SMTC_HAL_TRACE_ARRAY( "TX payload\n",
                                  rac_context_data->rac_context->smtc_rac_data_buffer_setup.tx_payload_buffer,
                                  ( uint32_t ) rac_context_data->rac_context->radio_params.lora.tx_size );
        }
    }
    else
    {
        // RX operation - log received payload
        SMTC_HAL_TRACE_INFO( "RX size=%" PRIu32 " (max=%" PRIu32 ")\n",
                             ( uint32_t ) rac_context_data->rac_context->smtc_rac_data_result.rx_size,
                             ( uint32_t ) rac_context_data->rac_context->radio_params.lora.max_rx_size );
        if( rac_context_data->rac_context->smtc_rac_data_buffer_setup.rx_payload_buffer &&
            rac_context_data->rac_context->smtc_rac_data_result.rx_size > 0 )
        {
            SMTC_HAL_TRACE_ARRAY( "RX payload\n",
                                  rac_context_data->rac_context->smtc_rac_data_buffer_setup.rx_payload_buffer,
                                  rac_context_data->rac_context->smtc_rac_data_result.rx_size );
        }
    }
}

static cmd_length_valid_t cmd_parser_check_cmd_size( host_cmd_id_t cmd_id, uint8_t length )
{
    /* cmd len too small */
    if( length < host_cmd_tab[cmd_id][HOST_CMD_TAB_IDX_MIN_LENGTH] )
    {
        SMTC_HAL_TRACE_ERROR( "Command size too small\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    /* cmd len too long */
    if( length > host_cmd_tab[cmd_id][HOST_CMD_TAB_IDX_MAX_LENGTH] )
    {
        SMTC_HAL_TRACE_ERROR( "Command size too long\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    return CMD_LENGTH_VALID;
}

static cmd_length_valid_t cmd_test_parser_check_cmd_size( host_cmd_test_id_t tst_id, uint8_t length )
{
    /* cmd len too small */
    if( length < host_cmd_test_tab[tst_id][HOST_CMD_TAB_IDX_MIN_LENGTH] )
    {
        SMTC_HAL_TRACE_ERROR( "Invalid command test size (too small)\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    /* cmd len too long */
    if( length > host_cmd_test_tab[tst_id][HOST_CMD_TAB_IDX_MAX_LENGTH] )
    {
        SMTC_HAL_TRACE_ERROR( "Invalid command test size (too long)\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    return CMD_LENGTH_VALID;
}

uint32_t cmd_parser_crc( const uint8_t* buf, int len )
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t mask;

    while( len-- > 0 )
    {
        crc = crc ^ *buf++;
        for( int i = 0; i < 8; i++ )
        {
            mask = -( crc & 1 );
            crc  = ( crc >> 1 ) ^ ( 0xEDB88320 & mask );
        }
    }
    return ~crc;
}

/* ============================================================================ */
/* NHM (New Hw Modem) Protocol Implementation                                  */
/* ============================================================================ */

/* Forward declarations for NHM RAC command handlers */
static cmd_serial_rc_code_t handle_nhm_rac_lora_cmd( uint8_t* cmd_payload, uint16_t cmd_length, uint16_t rsp_max_length,
                                                     uint8_t* rsp_payload, uint16_t* rsp_length );
static cmd_serial_rc_code_t handle_nhm_rac_cad_cmd( uint8_t* cmd_payload, uint16_t cmd_length, uint16_t rsp_max_length,
                                                    uint8_t* rsp_payload, uint16_t* rsp_length );
static cmd_serial_rc_code_t handle_nhm_rac_get_results_cmd( uint8_t* cmd_payload, uint16_t cmd_length,
                                                            uint16_t rsp_max_length, uint8_t* rsp_payload,
                                                            uint16_t* rsp_length );

/* Forward declarations for NHM segmentation handlers */
static void reset_nhm_segmentation_state( void );

/**
 * @brief Reset NHM segmentation state to initial values
 */
static void reset_nhm_segmentation_state( void )
{
    nhm_segmentation_state.cmd_id      = 0;
    nhm_segmentation_state.current_pos = 0;
}

cmd_parse_status_t parse_nhm_cmd( cmd_input_t* cmd_input, cmd_response_t* cmd_output )
{
    if( cmd_input->length < NHM_HEADER_SIZE )
    {
        SMTC_HAL_TRACE_ERROR( "NHM: Invalid header size (%d < %d)\n", cmd_input->length, NHM_HEADER_SIZE );
        reset_nhm_segmentation_state( );
        cmd_output->return_code = CMD_RC_BAD_SIZE;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }

    // Parse NHM header
    nhm_header_t* header = ( nhm_header_t* ) cmd_input->buffer;

    uint8_t  mt             = NHM_HEADER_GET_MT( header );
    uint8_t  pbf            = NHM_HEADER_GET_PBF( header );
    uint16_t nhm_cmd_id     = NHM_HEADER_GET_CMD_ID( header );
    uint8_t  payload_length = header->length;

    SMTC_HAL_TRACE_INFO( "NHM: MT=%d, PBF=%d, CMD_ID=0x%03x, Length=%d\n", mt, pbf, nhm_cmd_id, payload_length );

    // Verify payload length consistency
    if( payload_length != ( cmd_input->length - NHM_HEADER_SIZE ) || ( payload_length == 0 ) )
    {
        SMTC_HAL_TRACE_ERROR( "NHM: Payload length mismatch (header=%d, actual=%d)\n", payload_length,
                              cmd_input->length - NHM_HEADER_SIZE );
        reset_nhm_segmentation_state( );
        cmd_output->return_code = CMD_RC_INVALID;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }

    // Only handle commands for now (MT=1), responses and notifications will be handled later
    if( mt != NHM_MT_COMMAND )
    {
        SMTC_HAL_TRACE_ERROR( "NHM: Unsupported message type %d (only commands supported)\n", mt );
        reset_nhm_segmentation_state( );
        cmd_output->return_code = CMD_RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }

    uint8_t* payload = cmd_input->buffer + NHM_HEADER_SIZE;

    // Check cmd_id change (warning + reset if different)
    if( nhm_segmentation_state.current_pos > 0 && nhm_segmentation_state.cmd_id != nhm_cmd_id )
    {
        SMTC_HAL_TRACE_WARNING(
            "NHM: CMD_ID changed during segmentation (0x%03x -> 0x%03x) - starting new segmentation\n",
            nhm_segmentation_state.cmd_id, nhm_cmd_id );
        reset_nhm_segmentation_state( );
    }

    // Check buffer space
    if( nhm_segmentation_state.current_pos + payload_length > NHM_REASSEMBLY_BUFFER_SIZE )
    {
        SMTC_HAL_TRACE_ERROR( "NHM: Reassembly buffer overflow (%d + %d > %d)\n", nhm_segmentation_state.current_pos,
                              payload_length, NHM_REASSEMBLY_BUFFER_SIZE );
        reset_nhm_segmentation_state( );
        cmd_output->return_code = CMD_RC_BAD_SIZE;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }

    // Store cmd_id and append payload to buffer
    nhm_segmentation_state.cmd_id = nhm_cmd_id;
    memcpy( &nhm_reassembly_buffer[nhm_segmentation_state.current_pos], payload, payload_length );
    nhm_segmentation_state.current_pos += payload_length;

    SMTC_HAL_TRACE_PRINTF( "NHM: Segment stored (%d bytes, total=%d)\n", payload_length,
                           nhm_segmentation_state.current_pos );

    // Check if this is the last/complete segment
    if( pbf == NHM_PBF_COMPLETE_OR_LAST )
    {
        // Process complete message and reset
        SMTC_HAL_TRACE_PRINTF( "NHM: Processing complete message, CMD_ID=0x%03x, length=%d\n", nhm_cmd_id,
                               nhm_segmentation_state.current_pos );

        cmd_parse_status_t result = handle_nhm_complete_packet( nhm_cmd_id, nhm_reassembly_buffer,
                                                                nhm_segmentation_state.current_pos, cmd_output );

        reset_nhm_segmentation_state( );
        return result;
    }
    else
    {
        // Intermediate segment - acknowledge and wait for more
        cmd_output->return_code = CMD_RC_OK;
        cmd_output->length      = 0;
        return PARSE_OK;
    }
}

cmd_parse_status_t handle_nhm_complete_packet( uint16_t nhm_cmd_id, uint8_t* payload, uint16_t length,
                                               cmd_response_t* cmd_output )
{
    SMTC_HAL_TRACE_INFO( "NHM: Processing complete packet, CMD_ID=0x%03x, length=%d\n", nhm_cmd_id, length );

    switch( nhm_cmd_id )
    {
    case NHM_CMD_USP_SUBMIT:
        // Reset segmentation : macro or inline function
        nhm_segmentation_rsp_state.cmd_id       = nhm_cmd_id;
        nhm_segmentation_rsp_state.current_pos  = 0;
        nhm_segmentation_rsp_state.total_length = 0;
        cmd_output->return_code                 = handle_nhm_rac_lora_cmd( payload, length,

                                                                           NHM_REASSEMBLY_BUFFER_SIZE, nhm_reassembly_rsp_buffer,
                                                                           &( nhm_segmentation_rsp_state.total_length ) );
        break;
    case NHM_CMD_USP_CAD:
        // Reset segmentation : macro or inline function
        nhm_segmentation_rsp_state.cmd_id       = nhm_cmd_id;
        nhm_segmentation_rsp_state.current_pos  = 0;
        nhm_segmentation_rsp_state.total_length = 0;
        cmd_output->return_code =
            handle_nhm_rac_cad_cmd( payload, length, NHM_REASSEMBLY_BUFFER_SIZE, nhm_reassembly_rsp_buffer,
                                    &( nhm_segmentation_rsp_state.total_length ) );
        break;
    case NHM_CMD_USP_GET_RESULTS:
        // Reset segmentation : macro or inline function
        nhm_segmentation_rsp_state.cmd_id       = nhm_cmd_id;
        nhm_segmentation_rsp_state.current_pos  = 0;
        nhm_segmentation_rsp_state.total_length = 0;
        cmd_output->return_code =
            handle_nhm_rac_get_results_cmd( payload, length, NHM_REASSEMBLY_BUFFER_SIZE, nhm_reassembly_rsp_buffer,
                                            &( nhm_segmentation_rsp_state.total_length ) );
        break;
    case NHM_CMD_USP_GET_NEXT_SEGMENT:
        if( nhm_segmentation_rsp_state.total_length - nhm_segmentation_rsp_state.current_pos > 0 )
        {
            SMTC_HAL_TRACE_INFO( "NHM_CMD_USP_GET_NEXT_SEGMENT:  send remaining length (%u bytes)\n",
                                 nhm_segmentation_rsp_state.total_length - nhm_segmentation_rsp_state.current_pos );
            cmd_output->return_code = CMD_RC_OK;
        }
        else
        {
            SMTC_HAL_TRACE_ERROR( "NHM: No more segments to send\n" );
            cmd_output->return_code = CMD_RC_INVALID;
        }
        break;
    default:
        SMTC_HAL_TRACE_ERROR( "NHM: Unknown command ID 0x%03x\n", nhm_cmd_id );
        cmd_output->return_code = CMD_RC_UNKNOWN;
        // Reset segmentation : macro or inline function
        nhm_segmentation_rsp_state.cmd_id       = nhm_cmd_id;
        nhm_segmentation_rsp_state.current_pos  = 0;
        nhm_segmentation_rsp_state.total_length = 0;
    }

    // Parse NHM header
    nhm_header_t* header = ( nhm_header_t* ) cmd_output->buffer;
    NHM_HEADER_SET_MT( header, NHM_MT_RESPONSE );
    NHM_HEADER_SET_CMD_ID( header, nhm_cmd_id );

    if( cmd_output->return_code == CMD_RC_OK )
    {
        uint16_t remaining_length = nhm_segmentation_rsp_state.total_length - nhm_segmentation_rsp_state.current_pos;
        nhm_packet_boundary_t is_segmented =
            ( remaining_length < 245 ) ? NHM_PBF_COMPLETE_OR_LAST : NHM_PBF_NOT_LAST;  // SSA-TBD : value to calculate
        NHM_HEADER_SET_PBF( header, is_segmented );

        header->length = ( remaining_length < 245 ) ? remaining_length : 245;  // SSA-TBD :calculate value
        memcpy( cmd_output->buffer + NHM_HEADER_SIZE,
                nhm_reassembly_rsp_buffer + nhm_segmentation_rsp_state.current_pos, header->length );

        cmd_output->length = header->length + NHM_HEADER_SIZE;
        SMTC_HAL_TRACE_INFO( "return code OK : Send RSP :  remaining=%u, pktsize=%u, PBF = %u\n", remaining_length,
                             header->length, is_segmented );

        if( is_segmented == NHM_PBF_NOT_LAST )
        {
            nhm_segmentation_rsp_state.current_pos += header->length;
        }
        else
        {
            nhm_segmentation_rsp_state.current_pos  = 0;
            nhm_segmentation_rsp_state.total_length = 0;
        }

        return PARSE_OK;
    }
    else
    {
        SMTC_HAL_TRACE_INFO( "return code KO\n" );
        NHM_HEADER_SET_PBF( header, NHM_PBF_COMPLETE_OR_LAST );
        header->length     = 0;
        cmd_output->length = NHM_HEADER_SIZE;
        return PARSE_ERROR;
    }
}

/* NHM usp command handlers - Forward to existing implementations */
static cmd_serial_rc_code_t handle_nhm_rac_lora_cmd( uint8_t* cmd_payload, uint16_t cmd_length, uint16_t rsp_max_length,
                                                     uint8_t* rsp_payload, uint16_t* rsp_length )
{
    SMTC_HAL_TRACE_INFO( "NHM_CMD_USP_SUBMIT: Processing %d bytes\n", cmd_length );

    if( cmd_length == 0 )
    {
        SMTC_HAL_TRACE_ERROR( "NHM_CMD_USP_SUBMIT: No payload received\n" );
        return CMD_RC_INVALID;
    }

    // Deserialize protobuf context
    smtc_rac_lora_request_pb_t pb_rac_lora_call = smtc_rac_lora_request_pb_t_init_zero;
    pb_istream_t               stream           = pb_istream_from_buffer( cmd_payload, cmd_length );

    if( !pb_decode( &stream, smtc_rac_lora_request_pb_t_fields, &pb_rac_lora_call ) )
    {
        SMTC_HAL_TRACE_ERROR( "NHM_CMD_USP_SUBMIT: Failed to decode protobuf request\n" );
        return CMD_RC_INVALID;
    }

    SMTC_HAL_TRACE_INFO( "NHM_CMD_USP_SUBMIT: Protobuf decoded successfully\n" );
    SMTC_HAL_TRACE_INFO( "  Radio Id: %d\n", pb_rac_lora_call.radio_access_id );
    SMTC_HAL_TRACE_INFO( "  TX mode: %s\n", pb_rac_lora_call.rac_config.radio_params.is_tx ? "true" : "false" );
    SMTC_HAL_TRACE_INFO( "  Frequency: %u Hz\n", pb_rac_lora_call.rac_config.radio_params.frequency_in_hz );
    SMTC_HAL_TRACE_INFO( "  TX Power: %u dBm\n", pb_rac_lora_call.rac_config.radio_params.tx_power_in_dbm );
    SMTC_HAL_TRACE_INFO( "  TX Payload size: %u bytes\n",
                         ( uint32_t ) pb_rac_lora_call.rac_config.radio_params.tx_size );
    SMTC_HAL_TRACE_INFO( "  RX Max size: %u bytes\n", pb_rac_lora_call.rac_config.radio_params.max_rx_size );

    rac_context_data_t* rac_context_data = get_rac_context_data_from_handle( pb_rac_lora_call.radio_access_id );
    if( rac_context_data == NULL )
    {
        SMTC_HAL_TRACE_ERROR( "CMD_USP_SUBMIT: Failed to get rac context data\n" );
        return CMD_RC_FAIL;
    }

    // Convert to native structure - use existing pre-allocated buffers
    if( !rac_convert_context_from_pb( &( pb_rac_lora_call.rac_config ), rac_context_data->rac_context ) )
    {
        SMTC_HAL_TRACE_ERROR( "NHM_CMD_USP_SUBMIT: Failed to convert protobuf to native context\n" );
        return CMD_RC_FAIL;
    }

    SMTC_HAL_TRACE_PRINTF( "NHM_CMD_USP_SUBMIT: Context converted to native successfully\n" );

    // The first time, the python script do not know embedded side absolute time so we set it to the current time +
    // processing time
    if( rac_context_data->rac_context->scheduler_config.start_time_ms == 0 )
    {
        if( rac_context_data->rac_context->scheduler_config.scheduling == SMTC_RAC_ASAP_TRANSACTION )
        {
            rac_context_data->rac_context->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + 100;
        }
        else
        {
            rac_context_data->rac_context->scheduler_config.start_time_ms =
                smtc_modem_hal_get_time_in_ms( ) + 100;  // 10 : processing time
        }
    }

    // Call RAC API
    rac_context_data->rac_context->scheduler_config.callback_pre_radio_transaction = NULL;
    smtc_rac_return_code_t ret = smtc_rac_submit_radio_transaction( pb_rac_lora_call.radio_access_id );

    // Store return code for CMD_USP_GET_RESULTS
    rac_context_data->last_rac_return_code = ret;

    SMTC_HAL_TRACE_PRINTF( "NHM_CMD_USP_SUBMIT: Context processing completed successfully\n" );
    *rsp_length = 0;
    return ( ret == SMTC_RAC_SUCCESS ) ? CMD_RC_OK : CMD_RC_FAIL;  // SSA-TBD : do this better : lut ?
}

static cmd_serial_rc_code_t handle_nhm_rac_cad_cmd( uint8_t* cmd_payload, uint16_t cmd_length, uint16_t rsp_max_length,
                                                    uint8_t* rsp_payload, uint16_t* rsp_length )
{
    SMTC_HAL_TRACE_INFO( "NHM_CMD_USP_CAD: Processing %d bytes\n", cmd_length );
    // TODO: Implement CAD command processing similar to LORA
    *rsp_length = 0;
    return CMD_RC_NOT_IMPLEMENTED;
}

static cmd_serial_rc_code_t handle_nhm_rac_get_results_cmd( uint8_t* cmd_payload, uint16_t cmd_length,
                                                            uint16_t rsp_max_length, uint8_t* rsp_payload,
                                                            uint16_t* rsp_length )
{
    SMTC_HAL_TRACE_INFO( "NHM_CMD_USP_GET_RESULTS: Processing %d bytes\n", cmd_length );

    // Decode the request protobuf to get the radio_access_id
    rac_get_results_request_pb_t request = rac_get_results_request_pb_t_init_zero;

    if( cmd_length == 0 )
    {
        SMTC_HAL_TRACE_ERROR( "NHM_CMD_USP_GET_RESULTS: No payload received - radio_access_id is required\n" );
        return CMD_RC_INVALID;
    }

    pb_istream_t stream = pb_istream_from_buffer( cmd_payload, cmd_length );
    if( !pb_decode( &stream, rac_get_results_request_pb_t_fields, &request ) )
    {
        SMTC_HAL_TRACE_ERROR( "NHM_CMD_USP_GET_RESULTS: Failed to decode request protobuf\n" );
        return CMD_RC_INVALID;
    }

    SMTC_HAL_TRACE_INFO( "NHM_CMD_USP_GET_RESULTS: Radio handle = %d\n", request.radio_access_id );

    // Get the context corresponding to the radio_access_id
    rac_context_data_t* rac_context_data = get_rac_context_data_from_handle( request.radio_access_id );

    if( rac_context_data == NULL )
    {
        SMTC_HAL_TRACE_ERROR( "NHM_CMD_USP_GET_RESULTS: Invalid radio handle %d\n", request.radio_access_id );
        return CMD_RC_INVALID;
    }

    // Create optimized results message
    rac_results_pb_t results = rac_results_pb_t_init_zero;
    results.radio_access_id  = rac_context_data->radio_handle;

    // Determine transaction status based on pending_rac_event
    if( rac_context_data->pending_rac_event == true )
    {
        // Clear the pending event flag - results consumed
        rac_context_data->pending_rac_event = false;

        // Transaction completed - populate results
        results.transaction_status = rac_transaction_status_pb_t_RAC_TRANSACTION_COMPLETED_PB;
        results.return_code        = ( smtc_rac_return_code_pb_t ) rac_context_data->last_rac_return_code;
        results.rp_status          = convert_native_rp_status_to_pb( rac_context_data->status );

        // Convert native rac data result to protobuf
        if( !rac_convert_data_result_to_pb( &rac_context_data->rac_context->smtc_rac_data_result, &results.results ) )
        {
            SMTC_HAL_TRACE_ERROR( "NHM_CMD_USP_GET_RESULTS: Failed to convert results to protobuf\n" );
            return CMD_RC_FAIL;
        }

        // Copy RX payload if present
        if( !rac_copy_rx_payload_to_result( rac_context_data->rac_context->smtc_rac_data_buffer_setup.rx_payload_buffer,
                                            rac_context_data->rac_context->smtc_rac_data_result.rx_size,
                                            &results.results ) )
        {
            SMTC_HAL_TRACE_ERROR( "NHM_CMD_USP_GET_RESULTS: Failed to copy RX payload\n" );
            return CMD_RC_FAIL;
        }

        // FORCE nanopb to serialize payload field (critical for optional fields!)
        results.has_results                = true;  // Also ensure results field is serialized
        results.results.has_ranging_result = true;

        // Log results based on operation type (TX or RX)
        if( rac_context_data->rac_context->radio_params.lora.tx_size > 0 )
        {
            // TX operation results
            SMTC_HAL_TRACE_INFO(
                "NHM_CMD_USP_GET_RESULTS: TX Results - RSSI: %d dBm, SNR: %d dB, TX Payload: %d bytes\n",
                results.results.rssi_result, results.results.snr_result,
                ( uint32_t ) rac_context_data->rac_context->radio_params.lora.tx_size );
            SMTC_HAL_TRACE_ARRAY( "TX payload\n",
                                  rac_context_data->rac_context->smtc_rac_data_buffer_setup.tx_payload_buffer,
                                  ( uint32_t ) rac_context_data->rac_context->radio_params.lora.tx_size );
        }
        else
        {
            // RX operation results
            SMTC_HAL_TRACE_INFO(
                "NHM_CMD_USP_GET_RESULTS: RX Results - RSSI: %d dBm, SNR: %d dB, RX Payload: %d bytes\n",
                results.results.rssi_result, results.results.snr_result, ( uint32_t ) results.results.rx_size );
            if( results.results.rx_size > 0 &&
                rac_context_data->rac_context->smtc_rac_data_buffer_setup.rx_payload_buffer )
            {
                SMTC_HAL_TRACE_ARRAY( "RX payload\n",
                                      rac_context_data->rac_context->smtc_rac_data_buffer_setup.rx_payload_buffer,
                                      results.results.rx_size );
            }
        }
    }
    else
    {
        // No results available yet - transaction may be pending or no transaction started
        results.transaction_status = rac_transaction_status_pb_t_RAC_TRANSACTION_PENDING_PB;
        results.return_code        = smtc_rac_return_code_pb_t_SMTC_RAC_SUCCESS_PB;
        results.rp_status = convert_native_rp_status_to_pb( RP_STATUS_TASK_INIT );  // No operation completed yet

        SMTC_HAL_TRACE_INFO(
            "NHM_CMD_USP_GET_RESULTS: No results available for handle %d - transaction pending or not started\n",
            rac_context_data->radio_handle );
    }

    // Serialize the results message
    pb_ostream_t out_stream = pb_ostream_from_buffer( rsp_payload, rsp_max_length );
    if( !pb_encode( &out_stream, rac_results_pb_t_fields, &results ) )
    {
        SMTC_HAL_TRACE_ERROR( "NHM_CMD_USP_GET_RESULTS: Failed to encode results protobuf\n" );
        SMTC_HAL_TRACE_ERROR( "Encode failed: %s\n", PB_GET_ERROR( &out_stream ) );
        return CMD_RC_FAIL;
    }

    *rsp_length = out_stream.bytes_written;

    SMTC_HAL_TRACE_INFO( "NHM_CMD_USP_GET_RESULTS: Successfully encoded %u bytes for handle %d - Status: %d\n",
                         out_stream.bytes_written, rac_context_data->radio_handle, results.transaction_status );

    return CMD_RC_OK;
}

/* --- EOF ------------------------------------------------------------------ */
