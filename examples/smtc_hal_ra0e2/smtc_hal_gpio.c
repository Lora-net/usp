/**
 * @file      smtc_hal_gpio.c
 *
 * @brief     GPIO Hardware Abstraction Layer implementation for RA0E2
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

#include "smtc_hal_gpio.h"
#include "bsp_api.h"
#include "r_ioport_api.h"
#include "r_icu.h"
#include "r_external_irq_api.h"
#include "common_data.h"
#include "smtc_hal_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define GPIO_IRQ_MAX 16  // Maximum number of GPIO IRQ handlers

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static hal_gpio_irq_t* gpio_irq_handlers[GPIO_IRQ_MAX] = { NULL };

/**
 * @brief ICU instance registry entry
 *
 * Maps a GPIO pin to its corresponding FSP-generated ICU instance.
 * This allows dynamic lookup of ICU configuration for interrupt-capable pins.
 */
typedef struct
{
    hal_gpio_pin_names_t           pin;           // Pin this instance handles (e.g., 0x0201)
    const external_irq_instance_t* fsp_instance;  // FSP-generated instance (e.g., &g_external_radio_dio_irq5)
    bool                           is_open;       // Instance opened flag
} icu_instance_registry_t;

/**
 * @brief Registry of available ICU instances
 *
 * This table maps GPIO pins to their FSP-configured ICU instances.
 * Add new entries here when configuring additional external IRQs in e2studio.
 *
 * Current mappings:
 * - P201 (0x0201): Radio DIO interrupt on ICU channel 5 (rising edge)
 * - P200 (0x0200): User button on ICU channel 0 (falling edge)
 */
static icu_instance_registry_t icu_registry[] = {
    { .pin = 0x0201, .fsp_instance = &g_external_radio_dio_irq5, .is_open = false },  // Radio DIO
    { .pin = 0x0200, .fsp_instance = &g_external_button_irq0, .is_open = false },     // User button
};

#define ICU_REGISTRY_SIZE ( sizeof( icu_registry ) / sizeof( icu_registry[0] ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static bsp_io_port_pin_t        pin_name_to_bsp_pin( const hal_gpio_pin_names_t pin );
static icu_instance_registry_t* find_icu_for_pin( hal_gpio_pin_names_t pin );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_gpio_init_out( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    if( pin == NC )
    {
        return;
    }

    bsp_io_port_pin_t bsp_pin = pin_name_to_bsp_pin( pin );

    // Enable PFS register access
    R_BSP_PinAccessEnable( );

    // Configure as output with initial value
    uint32_t pin_cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT;

    if( value != 0 )
    {
        pin_cfg |= IOPORT_CFG_PORT_OUTPUT_HIGH;
    }
    else
    {
        pin_cfg |= IOPORT_CFG_PORT_OUTPUT_LOW;
    }

    R_BSP_PinCfg( bsp_pin, pin_cfg );

    // Disable PFS register access
    R_BSP_PinAccessDisable( );
}

void hal_gpio_init_in( const hal_gpio_pin_names_t pin, const hal_gpio_pull_mode_t pull_mode,
                       const hal_gpio_irq_mode_t irq_mode, hal_gpio_irq_t* irq )
{
    if( pin == NC )
    {
        return;
    }

    bsp_io_port_pin_t bsp_pin = pin_name_to_bsp_pin( pin );

    // Enable PFS register access
    R_BSP_PinAccessEnable( );

    // Configure as input with pull mode
    uint32_t pin_cfg = IOPORT_CFG_PORT_DIRECTION_INPUT;

    switch( pull_mode )
    {
    case BSP_GPIO_PULL_MODE_UP:
        pin_cfg |= IOPORT_CFG_PULLUP_ENABLE;
        break;
    case BSP_GPIO_PULL_MODE_DOWN:
        // RA0E2 doesn't have internal pull-down, use external resistor
        break;
    case BSP_GPIO_PULL_MODE_NONE:
    default:
        break;
    }

    // Enable IRQ on pin if IRQ mode is requested
    if( irq_mode != BSP_GPIO_IRQ_MODE_OFF )
    {
        pin_cfg |= IOPORT_CFG_IRQ_ENABLE;
    }

    R_BSP_PinCfg( bsp_pin, pin_cfg );

    // Disable PFS register access
    R_BSP_PinAccessDisable( );

    // Configure and attach IRQ if requested (matching STM32 HAL behavior)
    if( irq_mode != BSP_GPIO_IRQ_MODE_OFF && irq != NULL )
    {
        irq->pin = pin;
        hal_gpio_irq_attach( irq );
    }
}

void hal_gpio_irq_attach( const hal_gpio_irq_t* irq )
{
    if( irq == NULL )
    {
        return;
    }

    // Find ICU instance for this pin
    icu_instance_registry_t* icu_entry = find_icu_for_pin( irq->pin );
    if( icu_entry == NULL )
    {
        // Pin doesn't have ICU configured in e2studio
        // This should not happen if hal_gpio_init_in() was called properly
        return;
    }

    // Store handler in gpio_irq_handlers[] table for dispatch
    bool handler_stored = false;
    for( uint32_t i = 0; i < GPIO_IRQ_MAX; i++ )
    {
        if( gpio_irq_handlers[i] == NULL )
        {
            gpio_irq_handlers[i] = ( hal_gpio_irq_t* ) irq;
            handler_stored       = true;
            break;
        }
    }

    if( !handler_stored )
    {
        // GPIO IRQ handlers table is full
        return;
    }

    const external_irq_instance_t* instance = icu_entry->fsp_instance;

    // Close ICU instance if already open (clean state for reattach)
    if( icu_entry->is_open )
    {
        instance->p_api->disable( instance->p_ctrl );
        instance->p_api->close( instance->p_ctrl );
        icu_entry->is_open = false;
    }

    // Open ICU instance
    fsp_err_t err = instance->p_api->open( instance->p_ctrl, instance->p_cfg );
    if( err == FSP_SUCCESS )
    {
        icu_entry->is_open = true;
    }
    else
    {
        // Failed to open ICU instance - remove from handler table
        for( uint32_t i = 0; i < GPIO_IRQ_MAX; i++ )
        {
            if( gpio_irq_handlers[i] == irq )
            {
                gpio_irq_handlers[i] = NULL;
                break;
            }
        }
        return;
    }

    // Enable the interrupt
    instance->p_api->enable( instance->p_ctrl );
}

void hal_gpio_irq_deatach( const hal_gpio_irq_t* irq )
{
    if( irq == NULL )
    {
        return;
    }

    // Find ICU instance for this pin
    icu_instance_registry_t* icu_entry = find_icu_for_pin( irq->pin );
    if( icu_entry == NULL )
    {
        return;
    }

    // Disable and close the ICU instance
    const external_irq_instance_t* instance = icu_entry->fsp_instance;
    if( icu_entry->is_open )
    {
        instance->p_api->disable( instance->p_ctrl );
        instance->p_api->close( instance->p_ctrl );
        icu_entry->is_open = false;
    }

    // Remove IRQ handler from table
    for( uint32_t i = 0; i < GPIO_IRQ_MAX; i++ )
    {
        if( gpio_irq_handlers[i] == irq )
        {
            gpio_irq_handlers[i] = NULL;
            break;
        }
    }
}

void hal_gpio_irq_enable( void )
{
    // Enable all registered ICU channels
    for( uint32_t i = 0; i < GPIO_IRQ_MAX; i++ )
    {
        if( gpio_irq_handlers[i] != NULL )
        {
            icu_instance_registry_t* icu_entry = find_icu_for_pin( gpio_irq_handlers[i]->pin );
            if( icu_entry != NULL && icu_entry->is_open )
            {
                const external_irq_instance_t* instance = icu_entry->fsp_instance;
                instance->p_api->enable( instance->p_ctrl );
            }
        }
    }
}

void hal_gpio_irq_disable( void )
{
    // Disable all registered ICU channels
    for( uint32_t i = 0; i < GPIO_IRQ_MAX; i++ )
    {
        if( gpio_irq_handlers[i] != NULL )
        {
            icu_instance_registry_t* icu_entry = find_icu_for_pin( gpio_irq_handlers[i]->pin );
            if( icu_entry != NULL && icu_entry->is_open )
            {
                const external_irq_instance_t* instance = icu_entry->fsp_instance;
                instance->p_api->disable( instance->p_ctrl );
            }
        }
    }
}

void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    if( pin == NC )
    {
        return;
    }

    bsp_io_port_pin_t bsp_pin = pin_name_to_bsp_pin( pin );

    // Enable PFS register access
    R_BSP_PinAccessEnable( );

    // Write pin value
    R_BSP_PinWrite( bsp_pin, value ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW );

    // Disable PFS register access
    R_BSP_PinAccessDisable( );
}

uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin )
{
    if( pin == NC )
    {
        return 0;
    }

    bsp_io_port_pin_t bsp_pin = pin_name_to_bsp_pin( pin );

    // Read pin value
    bsp_io_level_t level = R_BSP_PinRead( bsp_pin );

    return ( level == BSP_IO_LEVEL_HIGH ) ? 1 : 0;
}

void hal_gpio_clear_pending_irq( const hal_gpio_pin_names_t pin )
{
    if( pin == NC )
    {
        return;
    }

    // FSP ICU driver automatically clears interrupt flags in the ISR handler
    // No manual flag clearing required for RA0E2 (unlike STM32 EXTI which requires __HAL_GPIO_EXTI_CLEAR_IT)
    // This function exists for API compatibility but is a no-op on RA0E2
    ( void ) pin;
}

void hal_gpio_enable_clock( const hal_gpio_pin_names_t pin )
{
    // On RA0E2, GPIO port clocks are managed by FSP BSP
    // No explicit clock enable needed for basic GPIO operations
    // This function is a no-op for FSP-based implementation
    ( void ) pin;
}

/**
 * @brief Dispatch GPIO IRQ to registered handler
 *
 * This function is called from FSP IRQ callbacks to route the interrupt
 * to the registered HAL GPIO IRQ handler.
 *
 * @param pin Pin number that triggered the IRQ
 */
void hal_gpio_irq_dispatch( hal_gpio_pin_names_t pin )
{
    // Find the handler registered for this pin
    for( uint32_t i = 0; i < GPIO_IRQ_MAX; i++ )
    {
        if( gpio_irq_handlers[i] != NULL && gpio_irq_handlers[i]->pin == pin )
        {
            // Call the registered callback if it exists
            if( gpio_irq_handlers[i]->callback != NULL )
            {
                gpio_irq_handlers[i]->callback( gpio_irq_handlers[i]->context );
            }
            break;
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bsp_io_port_pin_t pin_name_to_bsp_pin( const hal_gpio_pin_names_t pin )
{
    // Convert PnXX format (0xPPNN) to BSP pin format
    // Upper byte is port number, lower byte is pin number
    uint8_t port    = ( pin >> 8 ) & 0xFF;
    uint8_t pin_num = pin & 0xFF;

    // BSP_IO_PORT_XX_PIN_YY format: (port << 8) | pin
    return ( bsp_io_port_pin_t ) ( ( port << 8 ) | pin_num );
}

/**
 * @brief Find ICU instance for a pin
 *
 * Searches the ICU registry for the FSP-generated external IRQ instance
 * corresponding to the specified GPIO pin.
 *
 * @param pin GPIO pin number in 0xPPNN format
 * @return Pointer to ICU registry entry if found, NULL otherwise
 */
static icu_instance_registry_t* find_icu_for_pin( hal_gpio_pin_names_t pin )
{
    for( uint32_t i = 0; i < ICU_REGISTRY_SIZE; i++ )
    {
        if( icu_registry[i].pin == pin )
        {
            return &icu_registry[i];
        }
    }
    return NULL;  // Pin doesn't have ICU configured
}

/* --- EOF ------------------------------------------------------------------ */
