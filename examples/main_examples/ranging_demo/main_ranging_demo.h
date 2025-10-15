/**
 * @file      main_ranging_demo.h
 *
 * @brief     Ranging and hopping frequency example for LR1110 or LR1120 chip.
 *
 * This header defines the main configuration macros and interface for the ranging demo application.
 * It provides compile-time options to select the device mode (manager or subordinate), periodic uplink activation,
 * and transmission periodicity. These macros allow the application to be easily configured for different roles
 * and behaviors without modifying the source code.
 *
 * Key features:
 *   - Defines device roles: subordinate (slave) or manager (master).
 *   - Allows selection of the device mode at compile time via RANGING_DEVICE_MODE.
 *   - Optionally enables periodic uplink transmissions for demo or test purposes.
 *   - Sets the periodicity of uplink transmissions in milliseconds.
 *   - Uses include guards and C++ compatibility.
 *
 * Usage:
 *   - Include this header in the main application file.
 *   - Set the desired macros (e.g., RANGING_DEVICE_MODE, PERIODIC_UPLINK_ENABLED, TX_PERIODICITY_IN_MS)
 *     before including this file to override the defaults.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
 */

#ifndef MAIN_RANGING_DEMO_H
#define MAIN_RANGING_DEMO_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 * Includes standard integer types for portability.
 */
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 * These macros control the main configuration of the ranging demo application.
 */

/**
 * @brief Ranging device subordinate (slave) mode.
 */
#define RANGING_DEVICE_MODE_SUBORDINATE 1

/**
 * @brief Ranging device manager (master) mode.
 */
#define RANGING_DEVICE_MODE_MANAGER 2

/**
 * @brief Mode of operation.
 *        Set to RANGING_DEVICE_MODE_SUBORDINATE by default.
 *        Can be overridden at compile time to select manager or subordinate mode.
 */
#ifndef RANGING_DEVICE_MODE
#define RANGING_DEVICE_MODE RANGING_DEVICE_MODE_SUBORDINATE
#endif

/**
 * @brief Enable or disable multiple data rate activation.
 *        Set to true to allow multiple data rates (for demo/test).
 *        This is useful for testing different data rates in the application.
 */
#ifndef ACTIVATE_MULTIPLE_DATA_RATE
#define ACTIVATE_MULTIPLE_DATA_RATE false
#endif

/**
 * @brief Periodicity of uplink transmissions in milliseconds.
 *        Default is 10 seconds (10000 ms).
 *        Can be overridden at compile time
 */
#ifndef TX_PERIODICITY_IN_MS
#define TX_PERIODICITY_IN_MS 200000  // 20 seconds
#endif

#ifndef CONTINUOUS_RANGING
#define CONTINUOUS_RANGING false
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Enable or disable periodic uplink transmissions.
 *        Set to true to activate periodic uplinks (for demo/test).
 *        Can be overridden at compile time
 */
#ifdef ENABLE_PERIODIC_UPLINK
static const bool PERIODIC_UPLINK_ENABLED = true;
#else
static const bool PERIODIC_UPLINK_ENABLED = false;
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 * (No public types defined in this file, section reserved for future use.)
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 * (No public function prototypes defined in this file, section reserved for future use.)
 */

#ifdef __cplusplus
}
#endif

#endif  // MAIN_RANGING_DEMO_H

/* --- EOF ------------------------------------------------------------------ */
