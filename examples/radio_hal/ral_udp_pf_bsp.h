/**
 * @file      ral_udp_pf_bsp.h
 *
 * @brief     Board Support Package for UDP Packet Forwarder (stub - no physical radio)
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

#ifndef RAL_UDP_PF_BSP_H
#define RAL_UDP_PF_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "ral_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief UDP PF BSP regulator mode (stub)
 */
typedef enum ral_udp_pf_bsp_reg_mode_e
{
    RAL_UDP_PF_BSP_REG_MODE_LDO  = 0x00,
    RAL_UDP_PF_BSP_REG_MODE_DCDC = 0x01,
} ral_udp_pf_bsp_reg_mode_t;

/**
 * @brief UDP PF BSP RF switch configuration (stub)
 */
typedef struct ral_udp_pf_bsp_rf_switch_cfg_s
{
    bool     enable;
    uint32_t standby;
    uint32_t rx;
    uint32_t tx;
    uint32_t tx_hp;
    uint32_t tx_hf;
    uint32_t gnss;
    uint32_t wifi;
} ral_udp_pf_bsp_rf_switch_cfg_t;

/**
 * @brief UDP PF BSP TX configuration input parameters (stub)
 */
typedef struct ral_udp_pf_bsp_tx_cfg_input_params_s
{
    int8_t   system_output_pwr_in_dbm;
    uint32_t freq_in_hz;
} ral_udp_pf_bsp_tx_cfg_input_params_t;

/**
 * @brief UDP PF BSP TX configuration output parameters (stub)
 */
typedef struct ral_udp_pf_bsp_tx_cfg_output_params_s
{
    int8_t chip_output_pwr_in_dbm_configured;
    int8_t chip_output_pwr_in_dbm_expected;
} ral_udp_pf_bsp_tx_cfg_output_params_t;

/**
 * @brief UDP PF BSP XOSC configuration (stub)
 */
typedef enum ral_udp_pf_bsp_xosc_cfg_e
{
    RAL_UDP_PF_BSP_XOSC_CFG_XTAL            = 0x00,
    RAL_UDP_PF_BSP_XOSC_CFG_TCXO_RADIO_CTRL = 0x01,
    RAL_UDP_PF_BSP_XOSC_CFG_TCXO_EXT_CTRL   = 0x02,
} ral_udp_pf_bsp_xosc_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Get RF switch configuration (stub for UDP virtual radio)
 */
void ral_udp_pf_bsp_get_rf_switch_cfg( const void* context, ral_udp_pf_bsp_rf_switch_cfg_t* rf_switch_cfg );

/**
 * @brief Get TX configuration (stub for UDP virtual radio)
 */
void ral_udp_pf_bsp_get_tx_cfg( const void* context, const ral_udp_pf_bsp_tx_cfg_input_params_t* input_params,
                                ral_udp_pf_bsp_tx_cfg_output_params_t* output_params );

/**
 * @brief Get regulator mode (stub for UDP virtual radio)
 */
void ral_udp_pf_bsp_get_reg_mode( const void* context, ral_udp_pf_bsp_reg_mode_t* reg_mode );

/**
 * @brief Get XOSC configuration (stub for UDP virtual radio)
 */
void ral_udp_pf_bsp_get_xosc_cfg( const void* context, ral_xosc_cfg_t* xosc_cfg,
                                  ral_udp_pf_bsp_xosc_cfg_t* tcxo_is_radio_controlled, uint32_t* supply_voltage,
                                  uint32_t* startup_time_in_tick );

/**
 * @brief Get trim capacitor values (stub for UDP virtual radio)
 */
void ral_udp_pf_bsp_get_trim_cap( const void* context, uint8_t* trimming_cap_xta, uint8_t* trimming_cap_xtb );

/**
 * @brief Get RX boost configuration (stub for UDP virtual radio)
 */
void ral_udp_pf_bsp_get_rx_boost_cfg( const void* context, bool* rx_boost_is_activated );

/**
 * @brief Get OCP value (stub for UDP virtual radio)
 */
void ral_udp_pf_bsp_get_ocp_value( const void* context, uint8_t* ocp_in_step_of_2_5_ma );

/**
 * @brief Get LoRa CAD detection peak (stub for UDP virtual radio)
 */
void ral_udp_pf_bsp_get_lora_cad_det_peak( ral_lora_sf_t sf, ral_lora_bw_t bw, ral_lora_cad_symbs_t nb_symbol,
                                           uint8_t* in_out_cad_det_peak );

#ifdef __cplusplus
}
#endif

#endif  // RAL_UDP_PF_BSP_H

/* --- EOF ------------------------------------------------------------------ */
