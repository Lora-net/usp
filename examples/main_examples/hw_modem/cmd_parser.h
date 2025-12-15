/**
 * @file      cmd_parser.h
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

#ifndef CMD_PARSER_H__
#define CMD_PARSER_H__

#include <stdint.h>

#include "smtc_modem_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Host command opcode definition
 */
typedef enum host_cmd_id_e
{
    CMD_RESET                                = 0x00,
    CMD_SET_REGION                           = 0x01,
    CMD_GET_REGION                           = 0x02,
    CMD_JOIN_NETWORK                         = 0x03,
    CMD_REQUEST_UPLINK                       = 0x04,
    CMD_GET_EVENT                            = 0x05,
    CMD_GET_DOWNLINK_DATA                    = 0x06,
    CMD_GET_DOWNLINK_METADATA                = 0x07,
    CMD_GET_JOIN_EUI                         = 0x08,
    CMD_SET_JOIN_EUI                         = 0x09,
    CMD_GET_DEV_EUI                          = 0x0A,
    CMD_SET_DEV_EUI                          = 0x0B,
    CMD_SET_NWKKEY                           = 0x0C,
    CMD_GET_PIN                              = 0x0D,
    CMD_GET_CHIP_EUI                         = 0x0E,
    CMD_DERIVE_KEYS                          = 0x0F,
    CMD_GET_MODEM_VERSION                    = 0x10,
    CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER  = 0x11,
    CMD_SET_CERTIFICATION_MODE               = 0x12,
    CMD_EMERGENCY_UPLINK                     = 0x13,
    CMD_REQUEST_EMPTY_UPLINK                 = 0x14,
    CMD_LEAVE_NETWORK                        = 0x15,
    CMD_ALARM_START_TIMER                    = 0x16,
    CMD_ALARM_CLEAR_TIMER                    = 0x17,
    CMD_ALARM_GET_REMAINING_TIME             = 0x18,
    CMD_GET_NEXT_TX_MAX_PAYLOAD              = 0x19,
    CMD_GET_DUTY_CYCLE_STATUS                = 0x1A,
    CMD_SET_NETWORK_TYPE                     = 0x1B,
    CMD_SET_JOIN_DR_DISTRIBUTION             = 0x1C,
    CMD_SET_ADR_PROFILE                      = 0x1D,
    CMD_SET_NB_TRANS                         = 0x1E,
    CMD_GET_NB_TRANS                         = 0x1F,
    CMD_GET_ENABLED_DATARATE                 = 0x20,
    CMD_SET_ADR_ACK_LIMIT_DELAY              = 0x21,
    CMD_GET_ADR_ACK_LIMIT_DELAY              = 0x22,
    CMD_SET_CRYSTAL_ERR                      = 0x23,
    CMD_LBT_SET_PARAMS                       = 0x24,
    CMD_LBT_GET_PARAMS                       = 0x25,
    CMD_LBT_SET_STATE                        = 0x26,
    CMD_LBT_GET_STATE                        = 0x27,
    CMD_GET_CHARGE                           = 0x28,
    CMD_RESET_CHARGE                         = 0x29,
    CMD_SET_CLASS                            = 0x2A,
    CMD_CLASS_B_SET_PING_SLOT_PERIODICITY    = 0x2B,
    CMD_CLASS_B_GET_PING_SLOT_PERIODICITY    = 0x2C,
    CMD_MULTICAST_SET_GROUP_CONFIG           = 0x2D,
    CMD_MULTICAST_GET_GROUP_CONFIG           = 0x2E,
    CMD_MULTICAST_CLASS_C_START_SESSION      = 0x2F,
    CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS = 0x30,
    CMD_MULTICAST_CLASS_C_STOP_SESSION       = 0x31,
    CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS  = 0x32,
    CMD_MULTICAST_CLASS_B_START_SESSION      = 0x33,
    CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS = 0x34,
    CMD_MULTICAST_CLASS_B_STOP_SESSION       = 0x35,
    CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS  = 0x36,
    CMD_START_ALCSYNC_SERVICE                = 0x37,
    CMD_STOP_ALCSYNC_SERVICE                 = 0x38,
    CMD_GET_ALCSYNC_TIME                     = 0x39,
    CMD_TRIG_ALCSYNC_REQUEST                 = 0x3A,
    CMD_LORAWAN_MAC_REQUEST                  = 0x3B,
    CMD_GET_LORAWAN_TIME                     = 0x3C,
    CMD_GET_LINK_CHECK_DATA                  = 0x3D,
    CMD_SET_DUTY_CYCLE_STATE                 = 0x3E,
    CMD_DEBUG_CONNECT_WITH_ABP               = 0x3F,
    CMD_TEST                                 = 0x40,
    CMD_SET_TX_POWER_OFFSET                  = 0x41,
    CMD_GET_TX_POWER_OFFSET                  = 0x42,
    CMD_CSMA_SET_STATE                       = 0x43,
    CMD_CSMA_GET_STATE                       = 0x44,
    CMD_CSMA_SET_PARAMETERS                  = 0x45,
    CMD_CSMA_GET_PARAMETERS                  = 0x46,
    CMD_STREAM_INIT                          = 0x47,
    CMD_STREAM_ADD_DATA                      = 0x48,
    CMD_STREAM_STATUS                        = 0x49,
    CMD_LFU_INIT                             = 0x4A,
    CMD_LFU_DATA                             = 0x4B,
    CMD_LFU_START                            = 0x4C,
    CMD_LFU_RESET                            = 0x4D,
    CMD_DM_ENABLE                            = 0x4E,
    CMD_DM_GET_PORT                          = 0x4F,
    CMD_DM_SET_PORT                          = 0x50,
    CMD_DM_GET_INFO_INTERVAL                 = 0x51,
    CMD_DM_SET_INFO_INTERVAL                 = 0x52,
    CMD_DM_GET_PERIODIC_INFO_FIELDS          = 0x53,
    CMD_DM_SET_PERIODIC_INFO_FIELDS          = 0x54,
    CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS     = 0x55,
    CMD_DM_SET_USER_DATA                     = 0x56,
    CMD_DM_GET_USER_DATA                     = 0x57,
    CMD_GET_STATUS                           = 0x58,
    CMD_SUSPEND_RADIO_COMMUNICATIONS         = 0x59,
    CMD_DM_HANDLE_ALCSYNC                    = 0x5A,
    CMD_SET_APPKEY                           = 0x5B,
    CMD_GET_ADR_PROFILE                      = 0x5C,
    CMD_GET_CERTIFICATION_MODE               = 0x5D,
    CMD_STORE_AND_FORWARD_SET_STATE          = 0x5E,
    CMD_STORE_AND_FORWARD_GET_STATE          = 0x5F,
    CMD_STORE_AND_FORWARD_ADD_DATA           = 0x60,
    CMD_STORE_AND_FORWARD_CLEAR_DATA         = 0x61,
    CMD_STORE_AND_FORWARD_GET_FREE_SLOT      = 0x62,
#if defined( ADD_APP_GEOLOCATION )
    CMD_GNSS_SCAN                             = 0x70,
    CMD_GNSS_SCAN_CANCEL                      = 0x71,
    CMD_GNSS_GET_EVENT_DATA_SCAN_DONE         = 0x72,
    CMD_GNSS_GET_SCAN_DONE_RAW_DATA_LIST      = 0x73,
    CMD_GNSS_GET_SCAN_DONE_METADATA_LIST      = 0x74,
    CMD_GNSS_GET_SCAN_DONE_SCAN_SV            = 0x75,
    CMD_GNSS_GET_EVENT_DATA_TERMINATED        = 0x76,
    CMD_GNSS_SET_CONST                        = 0x77,
    CMD_GNSS_SET_PORT                         = 0x78,
    CMD_GNSS_SCAN_AGGREGATE                   = 0x79,
    CMD_GNSS_SEND_MODE                        = 0x7A,
    CMD_GNSS_ALM_DEMOD_START                  = 0x7B,
    CMD_GNSS_ALM_DEMOD_SET_CONSTEL            = 0x7C,
    CMD_GNSS_ALM_DEMOD_GET_EVENT_DATA_ALM_UPD = 0x7D,
#if defined( ADD_ALMANAC )
    CMD_CLOUD_ALMANAC_START = 0x7E,
    CMD_CLOUD_ALMANAC_STOP  = 0x7F,
#endif
    CMD_WIFI_SCAN_START                = 0x80,
    CMD_WIFI_SCAN_CANCEL               = 0x81,
    CMD_WIFI_GET_SCAN_DONE_SCAN_DATA   = 0x82,
    CMD_WIFI_GET_EVENT_DATA_TERMINATED = 0x83,
    CMD_WIFI_SET_PORT                  = 0x84,
    CMD_WIFI_SEND_MODE                 = 0x85,
    CMD_WIFI_SET_PAYLOAD_FORMAT        = 0x86,
    CMD_LR11XX_RADIO_READ              = 0x90,
    CMD_LR11XX_RADIO_WRITE             = 0x91,
#endif /* ADD_APP_GEOLOCATION */
    CMD_SET_RTC_OFFSET = 0x92,
#if defined( USE_RELAY_TX )
    CMD_SET_RELAY_CONFIG = 0x93,
    CMD_GET_RELAY_CONFIG = 0x94,
#endif /* USE_RELAY_TX */
    CMD_GET_SUSPEND_RADIO_COMMUNICATIONS       = 0x95,
    CMD_GET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF     = 0x96,
    CMD_SET_BYPASS_JOIN_DUTY_CYCLE_BACKOFF     = 0x97,
    CMD_MODEM_GET_CRASHLOG                     = 0x98,
    CMD_MODEM_GET_REPORT_ALL_DOWNLINKS_TO_USER = 0x99,
    CMD_MODEM_SET_REPORT_ALL_DOWNLINKS_TO_USER = 0x9A,

    CMD_USP_SUBMIT = 0xA0,
    CMD_USP_CAD    = 0xA1,
    CMD_USP_OPEN   = 0xA2,
    CMD_USP_CLOSE  = 0xA3,
    CMD_USP_ABORT  = 0xA4,
    /* CMD_USP_GET_RESULTS removed - use NHM_CMD_USP_GET_RESULTS via CMD_NHM_EXTENDED instead */

    /* NHM (New Hw Modem) Protocol - Extended commands */
    CMD_NHM_EXTENDED = 0xA6,

    CMD_MAX
} host_cmd_id_t;

/**
 * @brief Host test command opcode definition
 */
typedef enum host_cmd_test_id_e
{
    CMD_TST_START            = 0x00,
    CMD_TST_EXIT             = 0x01,
    CMD_TST_NOP              = 0x02,
    CMD_TST_TX_LORA          = 0x03,
    CMD_TST_TX_FSK           = 0x04,
    CMD_TST_TX_LRFHSS        = 0x05,
    CMD_TST_TX_CW            = 0x06,
    CMD_TST_RX_LORA          = 0x07,
    CMD_TST_RX_FSK_CONT      = 0x08,
    CMD_TST_READ_NB_PKTS_RX  = 0x09,
    CMD_TST_READ_LAST_RX_PKT = 0x0A,
    CMD_TST_RSSI             = 0x0B,
    CMD_TST_RSSI_GET         = 0x0C,
    CMD_TST_RADIO_RST        = 0x0D,
    CMD_TST_BUSYLOOP         = 0x0E,
    CMD_TST_PANIC            = 0x0F,
    CMD_TST_WATCHDOG         = 0x10,
    CMD_TST_RADIO_READ       = 0x11,
    CMD_TST_RADIO_WRITE      = 0x12,
    CMD_TST_MAX
} host_cmd_test_id_t;

/**
 * @brief Command parser serial return code
 */
typedef enum cmd_serial_rc_code_e
{
    CMD_RC_OK               = 0x00,
    CMD_RC_UNKNOWN          = 0x01,
    CMD_RC_NOT_IMPLEMENTED  = 0x02,
    CMD_RC_NOT_INIT         = 0x03,
    CMD_RC_INVALID          = 0x04,
    CMD_RC_BUSY             = 0x05,
    CMD_RC_FAIL             = 0x06,
    CMD_RC_BAD_CRC          = 0x08,
    CMD_RC_BAD_SIZE         = 0x0A,
    CMD_RC_FRAME_ERROR      = 0x0F,
    CMD_RC_NO_TIME          = 0x10,
    CMD_RC_INVALID_STACK_ID = 0x11,
    CMD_RC_NO_EVENT         = 0x12,
} cmd_serial_rc_code_t;

/**
 * @brief Input command structure
 */
typedef struct cmd_input_e
{
    host_cmd_id_t cmd_code;
    uint8_t       length;
    uint8_t*      buffer;
} cmd_input_t;

/**
 * @brief Command response struture
 */
typedef struct cmd_response_e
{
    cmd_serial_rc_code_t return_code;
    uint8_t              length;
    uint8_t*             buffer;
} cmd_response_t;

/**
 * @brief Input test command structure
 */
typedef struct cmd_tst_response_e
{
    cmd_serial_rc_code_t return_code;
    uint8_t              length;
    uint8_t*             buffer;
} cmd_tst_response_t;

/**
 * @brief Test command response struture
 */
typedef struct cmd_tst_input_e
{
    host_cmd_test_id_t cmd_code;
    uint8_t            length;
    uint8_t*           buffer;
} cmd_tst_input_t;

/**
 * @brief Command parser status
 */
typedef enum cmd_parse_status_e
{
    PARSE_ERROR,
    PARSE_OK,
} cmd_parse_status_t;

/* ============================================================================ */
/* NHM (New Hw Modem) Protocol Definitions                                     */
/* ============================================================================ */

/**
 * @brief NHM message types (3 bits - UCI compatible)
 */
typedef enum nhm_message_type_e
{
    NHM_MT_RFU          = 0, /*!< Reserved for future use */
    NHM_MT_COMMAND      = 1, /*!< Command message */
    NHM_MT_RESPONSE     = 2, /*!< Response message */
    NHM_MT_NOTIFICATION = 3, /*!< Notification message */
    /* 4-7 Reserved for future use */
} nhm_message_type_t;

/**
 * @brief NHM packet boundary flag for segmentation (1 bit - UCI compatible)
 */
typedef enum nhm_packet_boundary_e
{
    NHM_PBF_COMPLETE_OR_LAST = 0, /*!< Complete message OR last segment of fragmented message */
    NHM_PBF_NOT_LAST         = 1  /*!< Intermediate segment (not the last segment) */
} nhm_packet_boundary_t;

/**
 * @brief NHM extended command IDs (12-bit addressing = 4096 commands)
 */
typedef enum nhm_cmd_id_e
{
    NHM_CMD_USP_SUBMIT           = 0x100, /*!< USP/RAC submit command (replaces CMD_USP_SUBMIT for large packets) */
    NHM_CMD_USP_CAD              = 0x101, /*!< USP/RAC CAD command (replaces CMD_USP_CAD) */
    NHM_CMD_USP_GET_RESULTS      = 0x102, /*!< USP/RAC get results (replaces CMD_USP_GET_RESULTS) */
    NHM_CMD_USP_GET_NEXT_SEGMENT = 0x103, /*!< USP/RAC get next segment */
} nhm_cmd_id_t;

/**
 * @brief NHM protocol header structure (4 bytes) - UCI inspired format
 *
 * Format: [MT+PBF+ID_HIGH][ID_LOW][RFU][LENGTH]
 * - Byte 0: MT[7:5] + PBF[4] + ID_High[3:0] (UCI compatible)
 * - Byte 1: ID_Low[7:0] (total 12-bit command ID = 4096 commands)
 * - Byte 2: RFU (Reserved for Future Use)
 * - Byte 3: Payload length (0-251 bytes)
 *
 * Field details:
 * • MT (Message Type): 3 bits - Command(1)/Response(2)/Notification(3)
 * • PBF (Packet Boundary Flag): 1 bit - Complete/Last(0) vs Intermediate(1)
 * • ID_High: 4 bits - High part of 12-bit command identifier
 * • ID_Low: 8 bits - Low part of 12-bit command identifier
 * • RFU: 8 bits - Reserved for future protocol extensions
 * • Length: 8 bits - Payload size (max 251 bytes per packet)
 */
typedef struct nhm_header_e
{
    uint8_t mt_pbf_id_high; /*!< MT[7:5] + PBF[4] + ID_High[3:0] */
    uint8_t id_low;         /*!< ID_Low[7:0] - Command ID low 8 bits */
    uint8_t rfu;            /*!< Reserved for future use */
    uint8_t length;         /*!< Payload length (0-251 bytes) */
} nhm_header_t;

/**
 * @brief NHM packet structure
 */
typedef struct nhm_packet_e
{
    nhm_header_t header;  /*!< NHM header */
    uint8_t*     payload; /*!< Pointer to payload data */
} nhm_packet_t;

/**
 * @brief NHM segmentation state
 */
typedef struct nhm_segmentation_state_e
{
    uint16_t cmd_id;      /*!< Command ID being segmented (0 = no segmentation) */
    uint16_t current_pos; /*!< Current buffer position (0 = no segmentation) */
} nhm_segmentation_state_t;

typedef struct nhm_segmentation_rsp_state_e
{
    uint16_t cmd_id;       /*!< Command ID being segmented (0 = no segmentation) */
    uint16_t current_pos;  /*!< Current buffer position (0 = no segmentation) */
    uint16_t total_length; /*!< Total length of answer */
} nhm_segmentation_rsp_state_t;

/* Helper macros for NHM header manipulation (UCI compatible) */
#define NHM_HEADER_GET_MT( header ) ( ( ( header )->mt_pbf_id_high >> 5 ) & 0x07 )
#define NHM_HEADER_GET_PBF( header ) ( ( ( header )->mt_pbf_id_high >> 4 ) & 0x01 )
#define NHM_HEADER_GET_CMD_ID( header ) ( ( ( ( header )->mt_pbf_id_high & 0x0F ) << 8 ) | ( header )->id_low )

#define NHM_HEADER_SET_MT( header, mt ) \
    ( ( header )->mt_pbf_id_high = ( ( header )->mt_pbf_id_high & 0x1F ) | ( ( ( mt ) & 0x07 ) << 5 ) )
#define NHM_HEADER_SET_PBF( header, pbf ) \
    ( ( header )->mt_pbf_id_high = ( ( header )->mt_pbf_id_high & 0xEF ) | ( ( ( pbf ) & 0x01 ) << 4 ) )
#define NHM_HEADER_SET_CMD_ID_HIGH( header, cmd_id ) \
    ( ( header )->mt_pbf_id_high = ( ( header )->mt_pbf_id_high & 0xF0 ) | ( ( ( cmd_id ) >> 8 ) & 0x0F ) )
#define NHM_HEADER_SET_CMD_ID_LOW( header, cmd_id ) ( ( header )->id_low = ( cmd_id ) & 0xFF )
#define NHM_HEADER_SET_CMD_ID( header, cmd_id )       \
    do                                                \
    {                                                 \
        NHM_HEADER_SET_CMD_ID_HIGH( header, cmd_id ); \
        NHM_HEADER_SET_CMD_ID_LOW( header, cmd_id );  \
    } while( 0 )
#define NHM_HEADER_SET_MT_PBF_ID( mt, pbf, cmd_id ) \
    ( ( ( ( mt ) & 0x07 ) << 5 ) | ( ( ( pbf ) & 0x01 ) << 4 ) | ( ( ( cmd_id ) >> 8 ) & 0x0F ) )
#define NHM_HEADER_SET_ALL( header, mt, pbf, cmd_id, length )                     \
    do                                                                            \
    {                                                                             \
        ( header )->mt_pbf_id_high = NHM_HEADER_SET_MT_PBF_ID( mt, pbf, cmd_id ); \
        ( header )->id_low         = ( cmd_id ) & 0xFF;                           \
        ( header )->rfu            = 0;                                           \
        ( header )->length         = ( length );                                  \
    } while( 0 )

/* NHM Constants */
#define NHM_MAX_PAYLOAD_SIZE 251       /*!< Max payload per packet (255 - 4 header bytes) */
#define NHM_REASSEMBLY_BUFFER_SIZE 700 /*!< Max reassembled message size */
#define NHM_HEADER_SIZE 4              /*!< NHM header size in bytes */

/**
 * @brief Parse command received on serial link
 *
 * @param [in]  cmd_input  Contains the command received
 * @param [out] cmd_output Contains the response to the received command
 * @return cmd_parse_status_t
 */
cmd_parse_status_t parse_cmd( cmd_input_t* cmd_input, cmd_response_t* cmd_output );

/**
 * @brief Parse test command received on serial link
 *
 * @param [in] cmd_tst_input   Contains the command received
 * @param [out] cmd_tst_output Contains the response to the received command
 * @return cmd_parse_status_t
 */
cmd_parse_status_t cmd_test_parser( cmd_tst_input_t* cmd_tst_input, cmd_tst_response_t* cmd_tst_output );

/**
 * @brief Parse NHM (New Hw Modem) extended command
 *
 * @param [in]  cmd_input  Contains the NHM command received
 * @param [out] cmd_output Contains the response to the received NHM command
 * @return cmd_parse_status_t
 */
cmd_parse_status_t parse_nhm_cmd( cmd_input_t* cmd_input, cmd_response_t* cmd_output );

/**
 * @brief Handle NHM complete packet (no segmentation)
 *
 * @param [in]  nhm_cmd_id NHM command ID
 * @param [in]  payload    Pointer to payload data
 * @param [in]  length     Payload length
 * @param [out] cmd_output Contains the response
 * @return cmd_parse_status_t
 */
cmd_parse_status_t handle_nhm_complete_packet( uint16_t nhm_cmd_id, uint8_t* payload, uint16_t length,
                                               cmd_response_t* cmd_output );

/* Workaround for internal calls requiring a pointer to the transceiver context */

void cmd_parser_set_transceiver_context( void* context );

#ifdef __cplusplus
}
#endif

#endif /* CMD_PARSER_H__ */
