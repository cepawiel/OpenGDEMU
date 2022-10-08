/**
 * @file pal_generic.h
 *
 * @brief PAL Generic APIs
 *
 * This header file declares prototypes of PAL APIs, enumerations
 * used by TAL and MAC.
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 */

/*
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef PAL_GENERIC_H
#define PAL_GENERIC_H

/*
 * NOTE:- Include 'return_val.h' before this file, as return_val.h has the
 *        all return value enums.
 */

/* === Includes ============================================================ */

#include "compiler.h"
#include "return_val.h"
/* === Macros =============================================================== */

/* === Types =============================================================== */

/**
 * @addtogroup group_pal
 * @{
 */

/**
 * \brief IDs for persistence storage type
 */
typedef enum ps_type_tag {
	/** MCU Internal EEPROM */
	INTERN_EEPROM,
	/** External EEPROM */
	EXTERN_EEPROM
} SHORTENUM ps_type_t;

/**
 * Timeout type
 */
typedef enum
		timeout_type_tag {
	/** The timeout is relative to the current time. */
	TIMEOUT_RELATIVE,
	/** The timeout is an absolute value. */
	TIMEOUT_ABSOLUTE
} SHORTENUM timeout_type_t;

/* === Externals ============================================================ */

/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Services timer and sio handler
 *
 * This function calls sio & timer handling functions.
 */
void pal_task(void);

/**
 * @brief Start regular timer
 *
 * This function starts a regular timer and installs the corresponding
 * callback function handle the timeout event.
 *
 * @param timer_id Timer identifier
 * @param timer_count Timeout in microseconds
 * @param timeout_type @ref TIMEOUT_RELATIVE or @ref TIMEOUT_ABSOLUTE
 * @param timer_cb Callback handler invoked upon timer expiry
 * @param param_cb Argument for the callback handler
 *
 * @return
 *          - @ref PAL_TMR_INVALID_ID  if the timer identifier is undefined,
 *          - @ref MAC_INVALID_PARAMETER if the callback function for this timer
 *                 is NULL,
 *          - @ref PAL_TMR_ALREADY_RUNNING if the timer is already running.
 *          - @ref MAC_SUCCESS if timer is started or
 *          - @ref PAL_TMR_INVALID_TIMEOUT if timeout is not within timeout
 * range.
 */
retval_t pal_timer_start(uint8_t timer_id,
		uint32_t timer_count,
		timeout_type_t timeout_type,
		FUNC_PTR timer_cb,
		void *param_cb);

/**
 * @brief Stops a running timer
 *
 * This function stops a running timer with specified timer_id
 *
 * @param timer_id Timer identifier
 *
 * @return
 *          - @ref MAC_SUCCESS if timer stopped successfully,
 *          - @ref PAL_TMR_NOT_RUNNING if specified timer is not running,
 *          - @ref PAL_TMR_INVALID_ID if the specifed timer id is undefined.
 */
retval_t pal_timer_stop(uint8_t timer_id);

/**
 * @brief Gets current time
 *
 * This function returns the current time.
 *
 * @param[out] current_time Returns current system time
 */
void pal_get_current_time(uint32_t *current_time);

#if defined(ENABLE_HIGH_PRIO_TMR) || defined(__DOXYGEN__)

/*
 * Internal prototypes for pal_timer module
 */

/**
 * @brief Starts high priority timer
 *
 * This function starts a high priority timer for the specified timeout.
 *
 * @param timer_id Timer identifier
 * @param timer_count Timeout in microseconds
 * @param timer_cb Callback handler invoked upon timer expiry
 * @param param_cb Argument for the callback handler
 *
 * @return
 * - @ref PAL_TMR_INVALID_ID if the identifier is undefined,
 * - @ref MAC_INVALID_PARAMETER if the callback function for this timer is NULL,
 * - @ref PAL_TMR_ALREADY_RUNNING if the timer is already running, or
 * - @ref MAC_SUCCESS if timer is started successfully.
 */
retval_t pal_start_high_priority_timer(uint8_t timer_id,
		uint16_t timer_count,
		FUNC_PTR timer_cb,
		void *param_cb);

/**
 * @brief Stops a high priority timer
 *
 * This function stops a high priority timer.
 *
 * @param timer_id Timer identifier
 *
 * @return
 * - @ref PAL_TMR_NOT_RUNNING if the timer id does not match with the high
 * priority
 * timer register, or
 * - @ref MAC_SUCCESS otherwise.
 */
retval_t pal_stop_high_priority_timer(uint8_t timer_id);

#endif

/*
 * Prototypes for persistence storage
 */

/**
 * \brief Get data from persistence storage
 *
 * \param[in]  ps_type Persistence storage type
 * \param[in]  start_addr Start offset within EEPROM
 * \param[in]  length Number of bytes to read from EEPROM
 * \param[out] value Data from persistence storage
 *
 * \return MAC_SUCCESS  if everything went OK else FAILURE
 */
retval_t pal_ps_get(ps_type_t ps_type, uint16_t start_addr, uint16_t length,
		void *value);

/**
 * \brief Write data to persistence storage
 *
 * \param[in]  start_addr Start address offset within EEPROM
 * \param[in]  length Number of bytes to be written to EEPROM
 * \param[in]  value Data to persistence storage
 *
 * \return MAC_SUCCESS  if everything went OK else FAILURE
 */
retval_t pal_ps_set(uint16_t start_addr, uint16_t length, void *value);

retval_t pal_timer_get_id(uint8_t *timer_id);

/**
 * @brief Checks if the timer of requested timer identifier is running
 *
 * This function checks if the timer of requested timer identifier is running.
 *
 * @param timer_id Timer identifier
 *
 * @return
 * - true if timer with requested timer id is running,
 * - false otherwise.
 */
bool pal_is_timer_running(uint8_t timer_id);

/** @cond DOXYGEN_PAL_DEBUG */
#if (_DEBUG_ > 0)
bool pal_are_all_timers_stopped(void);

#endif  /* (_DEBUG_ > 0) */
/** @endcond */

/**
 * @brief Generation of random seed for function srand() in case this
 *        is not supported by the transceiver (e.g. AT86RF230)
 *
 * @return uint16_t Random number to be used as seed for function srand()
 */
uint16_t pal_generate_rand_seed(void);

/* ! @} */
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /* PAL_GENERIC_H */
/* EOF */
