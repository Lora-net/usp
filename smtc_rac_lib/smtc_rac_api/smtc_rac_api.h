/**
 * @file      smtc_rac_api.h
 *
 * @brief     Header file for smtc_rac API
 *
 * This file provides the API for the RAC, which manages radio access,
 * scheduling, and configuration for LoRa and ranging operations. It defines the
 * main data structures, enums, and function prototypes for interacting with the
 * radio planner and hardware abstraction.
 *
 * The API allows applications to:
 *   - Initialize and run the RAC engine.
 *   - Request radio access with a given priority and callback.
 *   - Configure LoRa and ranging parameters for transmission and reception.
 *   - Schedule radio tasks with precise timing.
 *   - Access radio event data such as RSSI, SNR, and timestamps.
 *   - Enqueue LoRa or ranging tasks for execution by the radio planner.
 *
 * The API is designed to be hardware-agnostic and supports multiple radio
 * chipsets.
 *
 * The main structures are:
 *   - smtc_rac_radio_lora_params_t: Holds all radio parameters for LoRa and
 * ranging operations.
 *   - smtc_rac_data_t: Holds payload, result, and event data for a radio
 * transaction.
 *   - smtc_rac_scheduler_config_t: Holds scheduling and callback information
 * for a radio task.
 *   - smtc_rac_context_t: Groups all the above for a complete radio
 * transaction context.
 *
 * The API also defines enums for radio access priority and return codes.
 *
 * The RAC is intended to be used in embedded applications requiring
 * precise and efficient management of radio resources, such as LoRaWAN,
 * ranging, and frequency hopping protocols.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
 */

#ifndef RAC_H
#define RAC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include "smtc_rac.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES - CORE ENUMS ----------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES - MAIN CONTEXT STRUCTURE ----------------------------------
 */

/*!
 * \struct smtc_rac_context_t
 * \brief Complete context for a radio transaction.
 *
 * This structure groups all radio parameters, data, and scheduling information
 * for a single transaction. It is passed to the RAC API when enqueuing a
 * radio task.
 */
typedef struct smtc_rac_context_s
{
    smtc_rac_modulation_type_t modulation_type; /*!< Type of modulation (LoRa, FSK, or LR-FHSS). */
    union
    {
        smtc_rac_radio_lora_params_t   lora;                 /*!< LoRa radio parameters. */
        smtc_rac_radio_fsk_params_t    fsk;                  /*!< FSK radio parameters. */
        smtc_rac_radio_lrfhss_params_t lrfhss;               /*!< LR-FHSS radio parameters. */
        smtc_rac_radio_flrc_params_t   flrc;                 /*!< FLRC radio parameters. */
    } radio_params;                                          /*!< Radio parameters union for the transaction. */
    smtc_rac_data_buffer_setup_t smtc_rac_data_buffer_setup; /*!< Data buffer setup for the transaction. */
    smtc_rac_scheduler_config_t  scheduler_config;           /*!< Scheduling and callback configuration. */
    smtc_rac_data_result_t       smtc_rac_data_result;       /*!< Data result from the transaction. */

    // optional parameters
    smtc_rac_lbt_context_t      lbt_context; /*!< LBT context for Listen Before Talk operations. */
    smtc_rac_cad_radio_params_t cad_context; /*!< CAD context for Channel Activity Detection operations. */
    smtc_rac_cw_context_t       cw_context;  /*!< CW context for Continuous Wave operations. */

} smtc_rac_context_t;

/**
 * @brief USP RAC API version structure definition
 */
typedef struct smtc_usp_rac_version_s
{
    uint8_t major;  //!< Major value
    uint8_t minor;  //!< Minor value
    uint8_t patch;  //!< Patch value
} smtc_usp_rac_version_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC API - CORE FUNCTIONS --------------------------------------------
 */

/**
 * @brief Get the USP RAC API version
 *
 * @param [out] USP RAC API version
 *
 * @return return code as defined in @ref smtc_rac_return_code_t
 * @retval SMTC_RAC_SUCCESS                Command executed without errors
 * @retval SMTC_RAC_INVALID_PARAMETER       \p version is NULL
 */
smtc_rac_return_code_t smtc_rac_get_version( smtc_usp_rac_version_t* version );

/*!
 * \brief Initializes the RAC.
 *
 * This function must be called before any other RAC API function.
 * It initializes the radio hardware, puts it in sleep mode, and sets up the
 * radio planner.
 */
void smtc_rac_init( void );

/*!
 * \brief Runs the RAC engine.
 *
 * This function should be called regularly (e.g., in the main loop) to process
 * radio events and tasks.
 */
void smtc_rac_run_engine( void );

/*!
 * \brief Checks if an IRQ flag is pending.
 *
 * \return True if an IRQ flag is pending, false otherwise.
 */
bool smtc_rac_is_irq_flag_pending( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC API - RADIO ACCESS MANAGEMENT -----------------------------------
 */

/*!
 * \brief Requests radio access.
 *
 * Registers a callback to be called when the radio is available at the given
 * priority.
 *
 * \param [in] priority Priority of the request.
 *
 * \return Radio access ID (used for future transactions),
 * RAC_INVALID_RADIO_ID if the request fails.
 */
uint8_t smtc_rac_open_radio( smtc_rac_priority_t priority );

/*
 * \brief Closes the radio access.
 *
 * This function releases the radio access associated with the given radio
 * access ID. It should be called when the radio is no longer needed.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_close_radio( uint8_t radio_access_id );

/*
 * \brief Submit a radio transaction.
 *
 * This function submits a radio transaction for execution by the radio planner.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 */
smtc_rac_return_code_t smtc_rac_submit_radio_transaction( uint8_t radio_access_id );

/*
 * \brief Aborts a radio submit.
 *
 * This function aborts a pending radio request associated with the given radio
 * access ID. It can be used to cancel a scheduled or in-progress radio
 * transaction.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_abort_radio_submit( uint8_t radio_access_id );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC API - CONTEXT AND DRIVER ACCESS ---------------------------------
 */

/*!
 * \brief Gets the radio planner instance.
 *
 * \return Pointer to the radio planner instance.
 */
radio_planner_t* smtc_rac_get_rp( void );

/*!
 * \brief Gets the ralf_t instance.
 *
 * \return Pointer to the ralf_t instance.
 */
ralf_t* smtc_rac_get_radio( void );

/*!
 * \brief Gets the RAC context for a specific radio access ID.
 *
 * This functions returns the complete context of the radio transaction
 * associated to the radio access ID. All fields are initialized to 0.
 * The caller is responsible for setting:
 * - modulation_type and the associated radio_params
 * - smtc_rac_data_buffer_setup
 * - scheduler_config (except duration_time_ms)
 * - (optionally) lbt_context, cad_context, cw_context
 * before calling smtc_rac_submit_radio_transaction(radio_access_id).
 * The results of the transaction will be available for reading in:
 * - smtc_rac_data_buffer_setup
 * - smtc_rac_data_result
 * - scheduler_config.duration_time_ms
 * when scheduler_config.callback_post_radio_transaction() is invoked.
 *
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *
 * \return Pointer to the RAC context for the specified radio access ID.
 */
smtc_rac_context_t* smtc_rac_get_context( uint8_t radio_access_id );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC API - SPECIALIZED OPERATIONS ------------------------------------
 */

/*!
 * \brief Direct radio driver access.
 *
 * This function allows direct access to the radio driver.
 * \note The radio access is locked if your request is granted, when thge radio is lock first rac will call the
 * callback_pre_radio_transaction . In this function you can prepare the radio for the transaction.
 * When the transaction is finished, rac will call the callback_post_radio_transaction with the status
 * RP_STATUS_RADIO_LOCKED if the transaction is successful, RP_STATUS_TASK_ABORTED if the transaction
 * is aborted. At this stage the radio is still locked and you can continue to use the radio. If you want to unlock the
 radio,
 * you can call smtc_rac_unlock_radio_access.

 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 * \param [in] scheduler_config The scheduler configuration.
 *
 * \return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_lock_radio_access( uint8_t                     radio_access_id,
                                                   smtc_rac_scheduler_config_t scheduler_config );

/*!
 * \brief Unlocks the radio access.
 *
 * This function unlocks the radio access associated with the given radio
 * access ID. This function should be called when the radio access get with smtc_rac_lock_radio_access is no longer
 *needed.
 * \note  if you unlock the radio access without locking it first, the behavior is the same as if you have locked it
 *first and then unlocked it. as a consequence the callback_post_radio_transaction will called with the status
 *RP_STATUS_TASK_UNLOCKED.
 * \param [in] radio_access_id The radio access ID obtained from
 * smtc_rac_open_radio.
 *\return Return code indicating success or error.
 */
smtc_rac_return_code_t smtc_rac_unlock_radio_access( uint8_t radio_access_id );
/*!
 * \brief Gets the radio driver context instance.
 *
 * \return Pointer to the radio driver context instance.
 */
void* smtc_rac_get_radio_driver_context( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC API - SET RADIO CONTEXT -----------------------------------------
 */

/*!
 * \brief Set the Radio Context (will be used in radio implementation, for
 * examples gpio management or a reset). \note To be called before
 * smtc_rac_init()
 *
 * \param [in] radio_ctx pointer to transceiver implemented by OS/baremetal.
 */
void smtc_rac_set_radio_context( const void* radio_ctx );

#ifdef __cplusplus
}
#endif

#endif  // RAC_H
