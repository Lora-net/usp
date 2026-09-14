/**
 * @file      smtc_hal_gpio_pin_names.h
 *
 * @brief     Defines NucleoL476 platform pin names
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
typedef enum gpio_pin_names_e
{
    // Raspberry Pi GPIO pins (BCM numbering) - 40-pin header
    RPI_GPIO0  = 0,   // Pin 27 - EEPROM SDA
    RPI_GPIO1  = 1,   // Pin 28 - EEPROM SCL
    RPI_GPIO2  = 2,   // Pin 3  - I2C1 SDA
    RPI_GPIO3  = 3,   // Pin 5  - I2C1 SCL
    RPI_GPIO4  = 4,   // Pin 7  - GPCLK0
    RPI_GPIO5  = 5,   // Pin 29
    RPI_GPIO6  = 6,   // Pin 31
    RPI_GPIO7  = 7,   // Pin 26 - SPI0 CE1
    RPI_GPIO8  = 8,   // Pin 24 - SPI0 CE0
    RPI_GPIO9  = 9,   // Pin 21 - SPI0 MISO
    RPI_GPIO10 = 10,  // Pin 19 - SPI0 MOSI
    RPI_GPIO11 = 11,  // Pin 23 - SPI0 SCLK
    RPI_GPIO12 = 12,  // Pin 32 - PWM0
    RPI_GPIO13 = 13,  // Pin 33 - PWM1
    RPI_GPIO14 = 14,  // Pin 8  - UART TX
    RPI_GPIO15 = 15,  // Pin 10 - UART RX
    RPI_GPIO16 = 16,  // Pin 36
    RPI_GPIO17 = 17,  // Pin 11
    RPI_GPIO18 = 18,  // Pin 12 - PCM CLK
    RPI_GPIO19 = 19,  // Pin 35 - PCM FS
    RPI_GPIO20 = 20,  // Pin 38 - PCM DIN
    RPI_GPIO21 = 21,  // Pin 40 - PCM DOUT
    RPI_GPIO22 = 22,  // Pin 15
    RPI_GPIO23 = 23,  // Pin 16
    RPI_GPIO24 = 24,  // Pin 18
    RPI_GPIO25 = 25,  // Pin 22
    RPI_GPIO26 = 26,  // Pin 37
    RPI_GPIO27 = 27,  // Pin 13

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
