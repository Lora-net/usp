/**
 * @file      smtc_hal_watchdog.c
 *
 * @brief     Watchdog Hardware Abstraction Layer implementation for RA0E2
 *
 *       RA0E2 IWDT Configuration (via OFS0 option byte at 0x400):
 *       - Timeout: 2048 cycles × /256 divider = 32 seconds
 *       - Auto-start: Enabled (starts counting after reset)
 *       - Stop in sleep: Enabled (stops counting during sleep mode)
 *       - Reset on timeout: Enabled (generates reset, not NMI)
 *      
 *       The IWDT clock is 16.384 kHz (dedicated, not LOCO).
 *      
 *       NOTE: JLink must do a chip erase before programming to ensure OFS0
 *       option byte is written. Use: erase + loadfile in JLink script.
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

#include "smtc_hal_watchdog.h"
#include "hal_data.h"
#include "r_iwdt.h"

void hal_watchdog_init( void )
{
    /*
     * Open IWDT instance. Required before refresh even in auto-start mode.
     *
     * Configuration is set via OFS0 option byte in bsp_mcu_ofs_cfg.h:
     * - TOPS = 3 (2048 cycles)
     * - CKS = 5 (/256 divider)
     * - SLCSTP = 1 (stop counting in sleep)
     * - RSTIRQS = 1 (reset output on timeout)
     */
    R_IWDT_Open( &g_wdt0_ctrl, &g_wdt0_cfg );
}

void hal_watchdog_reload( void )
{
    /* Refresh IWDT counter */
    R_IWDT_Refresh( &g_wdt0_ctrl );
}

/* --- EOF ------------------------------------------------------------------ */
