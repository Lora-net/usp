/**
 * @file      smtc_modem_hal_protect.c
 *
 * @brief     One-channel-hub override of the RAC "critical section" hooks
 *            smtc_modem_hal_protect_api_call() / _unprotect_api_call().
 *
 * The shared modem HAL (examples/smtc_modem_hal/smtc_modem_hal.c) provides these
 * as weak no-ops, which is correct for the single-threaded examples. The
 * one-channel-hub, however, drives the RAC API from several threads at once
 * (the modem/RAC engine in the main loop, the packet-forwarder JIT thread, and
 * the GPIO radio-IRQ and LP-timer delivery threads). It therefore overrides the
 * hooks here with a real, process-wide recursive mutex so all of those contexts
 * serialize on the single radio planner / SPI radio.
 *
 * The mutex MUST be recursive: the engine holds the lock across rp_callback(),
 * which synchronously dispatches RAC/LBM callbacks that re-enter the protected
 * API on the same thread.
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

#include <pthread.h>

static pthread_mutex_t rac_api_mutex;
static pthread_once_t  rac_api_mutex_once = PTHREAD_ONCE_INIT;

static void rac_api_mutex_init( void )
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init( &attr );
    pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_RECURSIVE );
    pthread_mutex_init( &rac_api_mutex, &attr );
    pthread_mutexattr_destroy( &attr );
}

void smtc_modem_hal_protect_api_call( void )
{
    pthread_once( &rac_api_mutex_once, rac_api_mutex_init );
    pthread_mutex_lock( &rac_api_mutex );
}

void smtc_modem_hal_unprotect_api_call( void )
{
    pthread_mutex_unlock( &rac_api_mutex );
}

/* --- EOF ------------------------------------------------------------------ */
