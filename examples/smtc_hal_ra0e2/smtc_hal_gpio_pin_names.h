/**
 * @file      smtc_hal_gpio_pin_names.h
 *
 * @brief     Defines FPB-RA0E2 platform pin names
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

#ifndef __SMTC_HAL_GPIO_PIN_NAMES_H__
#define __SMTC_HAL_GPIO_PIN_NAMES_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

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

/**
 * @brief GPIO pin names for Renesas RA0E2 (FPB-RA0E2)
 *
 * Pin naming follows Renesas PnXX format:
 * - Upper byte: Port number (0x00-0x09)
 * - Lower byte: Pin number (0x00-0x0F)
 * Example: P103 = 0x0103 (Port 1, Pin 3)
 */
typedef enum gpio_pin_names_e
{
    // Port 0
    P000 = 0x0000,
    P001 = 0x0001,
    P002 = 0x0002,
    P003 = 0x0003,
    P004 = 0x0004,
    P008 = 0x0008,
    P009 = 0x0009,
    P010 = 0x000A,
    P011 = 0x000B,
    P012 = 0x000C,
    P013 = 0x000D,
    P014 = 0x000E,
    P015 = 0x000F,

    // Port 1
    P100 = 0x0100,
    P101 = 0x0101,
    P102 = 0x0102,  // LED2
    P103 = 0x0103,  // LED1
    P104 = 0x0104,
    P105 = 0x0105,
    P106 = 0x0106,
    P107 = 0x0107,
    P108 = 0x0108,  // SWDIO
    P109 = 0x0109,  // UART4 TX
    P110 = 0x010A,  // UART4 RX
    P111 = 0x010B,
    P112 = 0x010C,
    P113 = 0x010D,
    P114 = 0x010E,
    P115 = 0x010F,

    // Port 2
    P200 = 0x0200,
    P201 = 0x0201,
    P204 = 0x0204,
    P205 = 0x0205,
    P206 = 0x0206,
    P207 = 0x0207,
    P208 = 0x0208,
    P212 = 0x020C,
    P213 = 0x020D,
    P214 = 0x020E,
    P215 = 0x020F,

    // Port 3
    P300 = 0x0300,  // SWCLK
    P301 = 0x0301,
    P302 = 0x0302,
    P303 = 0x0303,
    P304 = 0x0304,

    // Port 4
    P400 = 0x0400,  // I2C1 SCL
    P401 = 0x0401,  // I2C1 SDA
    P402 = 0x0402,
    P403 = 0x0403,
    P407 = 0x0407,
    P408 = 0x0408,
    P409 = 0x0409,
    P410 = 0x040A,
    P411 = 0x040B,

    // Port 5
    P500 = 0x0500,
    P501 = 0x0501,
    P502 = 0x0502,

    // Port 9
    P913 = 0x090D,  // I2C1 SCL (alt)
    P914 = 0x090E,  // I2C1 SDA (alt)
    P915 = 0x090F,

    // Not connected
    NC = -1
} hal_gpio_pin_names_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_GPIO_PIN_NAMES_H__

/* --- EOF ------------------------------------------------------------------ */
