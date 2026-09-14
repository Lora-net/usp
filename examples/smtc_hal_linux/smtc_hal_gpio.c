/**
 * @file      smtc_hal_gpio.c
 *
 * @brief     GPIO Hardware Abstraction Layer implementation for Linux
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <pthread.h>
#include <poll.h>

#include "smtc_hal_gpio_pin_names.h"
#include "smtc_hal_gpio.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define GPIO_CHIP_PATH "/dev/gpiochip0"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static int gpio_chip_fd = -1;

#define MAX_GPIO_PINS 32  // Maximum number of GPIO pins we might use

typedef struct
{
    hal_gpio_pin_names_t pin;
    int                  line_fd;   // File descriptor from gpio_request_line/output
    int                  event_fd;  // File descriptor from gpio_request_event (for IRQ pins)
    bool                 is_initialized;
    bool                 is_output;
    hal_gpio_irq_t*      irq;  // IRQ callback and context (NULL if no IRQ)
} gpio_context_t;

static gpio_context_t gpio_contexts[MAX_GPIO_PINS];
static int            gpio_context_count = 0;

// IRQ thread management
static pthread_t       gpio_irq_thread         = 0;
static pthread_mutex_t gpio_irq_mutex          = PTHREAD_MUTEX_INITIALIZER;
static volatile bool   gpio_irq_thread_running = false;
static volatile bool   gpio_irq_enabled        = false;

// Main thread TID — captured at hal_gpio_init() time (which runs in the main
// thread). Used by the GPIO IRQ thread to send SIGUSR2 and interrupt nanosleep.
static pthread_t gpio_main_thread_tid = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static int             gpio_request_line( int gpio_chip_fd, unsigned int offset, const char* consumer );
static int             gpio_request_output( int gpio_chip_fd, unsigned int offset, const char* consumer, int value );
static int             gpio_request_event( int gpio_chip_fd, unsigned int offset, const char* consumer );
static gpio_context_t* find_gpio_context( hal_gpio_pin_names_t pin );
static gpio_context_t* add_gpio_context( hal_gpio_pin_names_t pin );
static void*           gpio_irq_thread_func( void* arg );
static void            gpio_sigusr2_handler( int sig );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_gpio_init( void )
{
    // Check if already initialized
    if( gpio_chip_fd >= 0 )
    {
        return;
    }

    // Capture the calling thread's TID. hal_gpio_init() is always called from
    // hal_mcu_init() which itself runs in the application's main thread.
    // The GPIO IRQ thread will use this TID to send SIGUSR2 and immediately
    // interrupt any nanosleep() the main thread is blocked in.
    gpio_main_thread_tid = pthread_self( );

    // Register a no-op SIGUSR2 handler so the signal interrupts nanosleep().
    //
    // Two design points:
    //   1. The handler is a real function, not SIG_IGN. SIG_IGN would suppress
    //      the signal entirely and nanosleep() would NOT be interrupted.
    //   2. SA_RESTART is intentionally absent. With SA_RESTART the kernel would
    //      transparently restart nanosleep() after the handler returns, defeating
    //      the purpose. Without it, nanosleep() returns -1/EINTR immediately,
    //      which causes the main loop to re-check irq_flag without delay.
    //
    // SIGUSR1 is already claimed by smtc_hal_button.c for the virtual button.
    // SIGUSR2 is reserved by this HAL for DIO edge wake-up. Application code
    // must not install its own SIGUSR2 handler.
    struct sigaction sa;
    memset( &sa, 0, sizeof( sa ) );
    sa.sa_handler = gpio_sigusr2_handler;
    sigemptyset( &sa.sa_mask );
    // no SA_RESTART
    if( sigaction( SIGUSR2, &sa, NULL ) != 0 )
    {
        printf( "GPIO: WARNING — failed to register SIGUSR2 handler, DIO wake-up disabled\n" );
        gpio_main_thread_tid = 0;
    }
    else
    {
        printf( "GPIO: SIGUSR2 registered for DIO edge wake-up (main tid=%lu)\n",
                ( unsigned long ) gpio_main_thread_tid );
    }

    // Open GPIO chip device
    gpio_chip_fd = open( GPIO_CHIP_PATH, O_RDWR );
    if( gpio_chip_fd < 0 )
    {
        printf( "GPIO: Failed to open GPIO chip: %s\n", GPIO_CHIP_PATH );
        return;
    }

    printf( "GPIO: Subsystem initialized (chip_fd=%d)\n", gpio_chip_fd );
}

void hal_gpio_de_init( void )
{
    // Stop IRQ thread if running
    if( gpio_irq_thread_running )
    {
        printf( "GPIO: Stopping IRQ thread\n" );
        gpio_irq_thread_running = false;

        // Wait for thread to terminate
        if( gpio_irq_thread != 0 )
        {
            pthread_join( gpio_irq_thread, NULL );
            gpio_irq_thread = 0;
        }
    }

    // Close all GPIO line file descriptors
    for( int i = 0; i < gpio_context_count; i++ )
    {
        if( gpio_contexts[i].line_fd >= 0 )
        {
            close( gpio_contexts[i].line_fd );
        }
        if( gpio_contexts[i].event_fd >= 0 )
        {
            close( gpio_contexts[i].event_fd );
        }
    }

    // Reset context
    gpio_context_count = 0;
    memset( gpio_contexts, 0, sizeof( gpio_contexts ) );

    // Close GPIO chip
    if( gpio_chip_fd >= 0 )
    {
        close( gpio_chip_fd );
        gpio_chip_fd = -1;
    }
}

void hal_gpio_init_out( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    if( pin == NC || gpio_chip_fd < 0 )
    {
        return;
    }

    // Check if already initialized
    gpio_context_t* ctx = find_gpio_context( pin );
    if( ctx != NULL )
    {
        printf( "GPIO: Pin %d already initialized\n", pin );
        return;
    }

    // Request output line using existing helper
    int line_fd = gpio_request_output( gpio_chip_fd, pin, "gpio_out", value );
    if( line_fd < 0 )
    {
        printf( "GPIO: Failed to init output pin %d\n", pin );
        return;
    }

    // Register in context
    ctx = add_gpio_context( pin );
    if( ctx )
    {
        ctx->line_fd        = line_fd;
        ctx->is_initialized = true;
        ctx->is_output      = true;
    }

    printf( "GPIO: Init output pin %d with value %u (fd=%d)\n", pin, value, line_fd );
}

void hal_gpio_init_in( const hal_gpio_pin_names_t pin, const hal_gpio_pull_mode_t pull_mode,
                       const hal_gpio_irq_mode_t irq_mode, hal_gpio_irq_t* irq )
{
    if( pin == NC || gpio_chip_fd < 0 )
    {
        return;
    }

    // Check if already initialized
    gpio_context_t* ctx = find_gpio_context( pin );
    if( ctx != NULL )
    {
        printf( "GPIO: Pin %d already initialized\n", pin );
        return;
    }

    int line_fd  = -1;
    int event_fd = -1;

    // Note: Linux GPIO character device doesn't support pull mode configuration
    // Pull-ups/downs must be configured at hardware or device tree level
    ( void ) pull_mode;

    // Configure based on IRQ mode
    // Note: IRQ callback can be NULL here and attached later via hal_gpio_irq_attach()
    if( irq_mode != BSP_GPIO_IRQ_MODE_OFF )
    {
        // Request event for IRQ-enabled input (for edge detection)
        event_fd = gpio_request_event( gpio_chip_fd, pin, "gpio_irq" );
        if( event_fd < 0 )
        {
            printf( "GPIO: Failed to request event for pin %d\n", pin );
            return;
        }
    }
    else
    {
        // Regular input (no IRQ)
        line_fd = gpio_request_line( gpio_chip_fd, pin, "gpio_in" );
        if( line_fd < 0 )
        {
            printf( "GPIO: Failed to init input pin %d\n", pin );
            return;
        }
    }

    // Register in context
    ctx = add_gpio_context( pin );
    if( ctx )
    {
        ctx->line_fd        = line_fd;
        ctx->event_fd       = event_fd;
        ctx->is_initialized = true;
        ctx->is_output      = false;
        ctx->irq            = irq;  // Store IRQ callback (can be NULL initially)
    }

    printf( "GPIO: Init input pin %d (pull:%d, irq_mode:%d, line_fd=%d, event_fd=%d)\n", pin, pull_mode, irq_mode,
            line_fd, event_fd );

    // If IRQ mode is enabled, start monitoring thread immediately
    // This prevents events from accumulating in kernel buffer before callback is attached
    if( irq_mode != BSP_GPIO_IRQ_MODE_OFF )
    {
        pthread_mutex_lock( &gpio_irq_mutex );

        // Start IRQ thread if not already running
        if( !gpio_irq_thread_running )
        {
            // If thread existed but exited, join it first
            if( gpio_irq_thread != 0 )
            {
                pthread_join( gpio_irq_thread, NULL );
                gpio_irq_thread = 0;
            }

            gpio_irq_thread_running = true;
            gpio_irq_enabled        = true;

            if( pthread_create( &gpio_irq_thread, NULL, gpio_irq_thread_func, NULL ) != 0 )
            {
                printf( "GPIO: Failed to create IRQ thread\n" );
                gpio_irq_thread_running = false;
            }
            else
            {
                printf( "GPIO: IRQ monitoring thread started for pin %d\n", pin );
            }
        }

        pthread_mutex_unlock( &gpio_irq_mutex );
    }
}

void hal_gpio_irq_attach( const hal_gpio_irq_t* irq )
{
    if( !irq )
    {
        return;
    }

    pthread_mutex_lock( &gpio_irq_mutex );

    // Find GPIO context for this pin
    gpio_context_t* ctx = find_gpio_context( irq->pin );
    if( ctx && ctx->event_fd >= 0 )
    {
        // Attach/update the callback pointer
        // The IRQ monitoring thread is already running (started in hal_gpio_init_in)
        ctx->irq = ( hal_gpio_irq_t* ) irq;

        // printf( "GPIO: IRQ callback attached to pin %d\n", irq->pin );
    }
    else
    {
        printf( "GPIO: Failed to attach IRQ - pin %d not configured for IRQ\n", irq->pin );
    }

    pthread_mutex_unlock( &gpio_irq_mutex );
}

void hal_gpio_irq_enable( void )
{
    pthread_mutex_lock( &gpio_irq_mutex );
    gpio_irq_enabled = true;
    pthread_mutex_unlock( &gpio_irq_mutex );
}

void hal_gpio_irq_disable( void )
{
    pthread_mutex_lock( &gpio_irq_mutex );
    gpio_irq_enabled = false;
    pthread_mutex_unlock( &gpio_irq_mutex );
}

void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    gpio_context_t* ctx = find_gpio_context( pin );
    if( !ctx || ctx->line_fd < 0 )
    {
        return;
    }

    struct gpiohandle_data data;
    data.values[0] = value ? 1 : 0;

    int ret = ioctl( ctx->line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data );
    if( ret < 0 )
    {
        printf( "GPIO: Failed to set value for pin %d\n", pin );
    }
}

uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin )
{
    gpio_context_t* ctx = find_gpio_context( pin );
    if( !ctx || ctx->line_fd < 0 )
    {
        return 0;
    }

    struct gpiohandle_data data;
    int                    ret = ioctl( ctx->line_fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data );
    if( ret < 0 )
    {
        printf( "GPIO: Failed to get value for pin %d\n", pin );
        return 0;
    }

    return data.values[0];
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void* gpio_irq_thread_func( void* arg )
{
    ( void ) arg;

    // Block SIGUSR2 in this thread. SIGUSR2 is only meant for the main thread.
    // Without this, pthread_kill() targeting the main thread could, under rare
    // scheduler conditions, be delivered here instead.
    sigset_t block_mask;
    sigemptyset( &block_mask );
    sigaddset( &block_mask, SIGUSR2 );
    pthread_sigmask( SIG_BLOCK, &block_mask, NULL );

    while( gpio_irq_thread_running )
    {
        // Build the poll array for all event_fds
        struct pollfd fds[MAX_GPIO_PINS];
        int           nfds = 0;

        pthread_mutex_lock( &gpio_irq_mutex );

        // Collect all event FDs (even if callback not attached yet)
        // This prevents events from accumulating in kernel buffer
        for( int i = 0; i < gpio_context_count; i++ )
        {
            if( gpio_contexts[i].is_initialized && gpio_contexts[i].event_fd >= 0 )
            {
                fds[nfds].fd     = gpio_contexts[i].event_fd;
                fds[nfds].events = POLLIN;
                nfds++;
            }
        }

        pthread_mutex_unlock( &gpio_irq_mutex );

        if( nfds == 0 )
        {
            // No IRQs to monitor, exit thread
            printf( "GPIO IRQ thread: No IRQs to monitor, exiting\n" );
            gpio_irq_thread_running = false;
            break;
        }

        // Poll for events with a timeout
        int ret = poll( fds, nfds, 100 );  // 100ms timeout

        if( ret > 0 )
        {
            // One or more file descriptors have events
            for( int i = 0; i < nfds; i++ )
            {
                if( fds[i].revents & POLLIN )
                {
                    // Read the event
                    struct gpioevent_data event;
                    ssize_t               bytes_read = read( fds[i].fd, &event, sizeof( event ) );

                    if( bytes_read == sizeof( event ) )
                    {
                        // Find the corresponding GPIO context
                        pthread_mutex_lock( &gpio_irq_mutex );

                        for( int j = 0; j < gpio_context_count; j++ )
                        {
                            if( gpio_contexts[j].event_fd == fds[i].fd )
                            {
                                // printf( "GPIO IRQ: Event received on pin %d (event_id=%u)\n", gpio_contexts[j].pin,
                                // event.id );

                                // Check if IRQs are enabled and callback is valid before invoking
                                if( gpio_irq_enabled && gpio_contexts[j].irq != NULL &&
                                    gpio_contexts[j].irq->callback != NULL )
                                {
                                    // printf( "GPIO IRQ: Calling callback for pin %d\n", gpio_contexts[j].pin );
                                    gpio_contexts[j].irq->callback( gpio_contexts[j].irq->context );

                                    // Wake the main thread immediately. The callback above has set
                                    // the radio planner IRQ flag. The main thread may be sleeping
                                    // inside nanosleep() — SIGUSR2 interrupts that sleep (returns
                                    // EINTR) so it re-checks the flag without waiting up to 1 ms.
                                    if( gpio_main_thread_tid != 0 )
                                    {
                                        pthread_kill( gpio_main_thread_tid, SIGUSR2 );
                                    }
                                }
                                else
                                {
                                    // Event consumed but not processed (callback not ready or IRQs disabled)
                                    // printf( "GPIO IRQ: Event on pin %d consumed but not processed\n",
                                    // gpio_contexts[j].pin );
                                }
                                break;
                            }
                        }

                        pthread_mutex_unlock( &gpio_irq_mutex );
                    }
                }
            }
        }
        else if( ret < 0 )
        {
            perror( "GPIO IRQ thread: poll error" );
            gpio_irq_thread_running = false;
            break;
        }
        // ret == 0 means timeout, just loop again
    }

    printf( "GPIO IRQ thread: stopped\n" );
    return NULL;
}

static int gpio_request_line( int gpio_chip_fd, unsigned int offset, const char* consumer )
{
    ( void ) consumer;  // Suppress unused parameter warning
    struct gpiohandle_request req;
    memset( &req, 0, sizeof( req ) );

    req.lineoffsets[0] = offset;
    req.flags          = GPIOHANDLE_REQUEST_INPUT;
    req.lines          = 1;

    int ret = ioctl( gpio_chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req );
    if( ret < 0 )
    {
        return ret;  // Return error code
    }

    return req.fd;  // Return the GPIO line file descriptor
}

static int gpio_request_output( int gpio_chip_fd, unsigned int offset, const char* consumer, int value )
{
    ( void ) consumer;  // Suppress unused parameter warning
    struct gpiohandle_request req;
    memset( &req, 0, sizeof( req ) );

    req.lineoffsets[0]    = offset;
    req.flags             = GPIOHANDLE_REQUEST_OUTPUT;
    req.lines             = 1;
    req.default_values[0] = value;

    int ret = ioctl( gpio_chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req );
    if( ret < 0 )
    {
        return ret;  // Return error code
    }

    return req.fd;  // Return the GPIO line file descriptor
}

static int gpio_request_event( int gpio_chip_fd, unsigned int offset, const char* consumer )
{
    ( void ) consumer;  // Suppress unused parameter warning
    struct gpioevent_request req;
    memset( &req, 0, sizeof( req ) );

    req.lineoffset  = offset;
    req.handleflags = GPIOHANDLE_REQUEST_INPUT;
    req.eventflags  = GPIOEVENT_REQUEST_RISING_EDGE;  // Trigger on rising edge

    int ret = ioctl( gpio_chip_fd, GPIO_GET_LINEEVENT_IOCTL, &req );
    if( ret < 0 )
    {
        return ret;  // Return error code
    }

    return req.fd;  // Return the event file descriptor
}

static gpio_context_t* find_gpio_context( hal_gpio_pin_names_t pin )
{
    if( pin == NC )
    {
        return NULL;
    }

    for( int i = 0; i < gpio_context_count; i++ )
    {
        if( gpio_contexts[i].pin == pin && gpio_contexts[i].is_initialized )
        {
            return &gpio_contexts[i];
        }
    }
    return NULL;
}

static gpio_context_t* add_gpio_context( hal_gpio_pin_names_t pin )
{
    if( pin == NC || gpio_context_count >= MAX_GPIO_PINS )
    {
        return NULL;
    }

    gpio_context_t* ctx = &gpio_contexts[gpio_context_count++];
    ctx->pin            = pin;
    ctx->line_fd        = -1;
    ctx->event_fd       = -1;
    ctx->is_initialized = false;
    ctx->is_output      = false;
    ctx->irq            = NULL;

    return ctx;
}

static void gpio_sigusr2_handler( int sig )
{
    ( void ) sig;
    // Intentionally empty. The sole purpose of this handler is to exist as a
    // non-SIG_IGN, non-SIG_DFL disposition so that nanosleep() returns EINTR
    // when SIGUSR2 is delivered to the main thread by the GPIO IRQ thread.
}

/* --- EOF ------------------------------------------------------------------ */
