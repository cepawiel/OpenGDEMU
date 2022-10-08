/**
 * \file
 *
 * \brief SAM True Random Number Generator (TRNG) Driver
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
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
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "trng_callback.h"
#include "system_interrupt.h"

struct trng_module *_trng_instance;

/**
 * \brief Registers a callback
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref trng_enable_callback,
 * in order for the interrupt handler to call it when the conditions for the
 * callback type is met.
 *
 * \param[in]     module        Pointer to TC software instance struct
 * \param[in]     callback_func Pointer to callback function
 * \param[in]     callback_type Callback type given by an enum
 *
 * \retval STATUS_OK  The function exited successfully
 */
enum status_code trng_register_callback(
		struct trng_module *const module,
		trng_callback_t callback_func,
		const enum trng_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);
	Assert(callback_func);

	/* Register callback function */
	module->callback[callback_type] = callback_func;

	/* Set the bit corresponding to the callback_type */
	module->register_callback_mask |= (1 << callback_type);

	/* Enable interrupt for this TRNG module */
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_TRNG);

	return STATUS_OK;
}

/**
 * \brief Unregisters a callback
 *
 * Unregisters a callback function implemented by the user. The callback should be
 * disabled before it is unregistered.
 *
 * \param[in]     module        Pointer to TC software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \retval STATUS_OK  The function exited successfully
 */
enum status_code trng_unregister_callback(
		struct trng_module *const module,
		const enum trng_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);

	/* Unregister callback function */
	module->callback[callback_type] = NULL;

	/* Clear the bit corresponding to the callback_type */
	module->register_callback_mask &= ~(1 << callback_type);

	/* Disable interrupt for this TRNG module */
	if (module->register_callback_mask == 0) {
		system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_TRNG);
	}

	return STATUS_OK;
}

/**
 * \internal ISR handler for TRNG
 *
 * TRNG interrupt handler for random data ready.
 */
void TRNG_Handler(void)
{
	/* Temporary variable */
	uint8_t interrupt_and_callback_status_mask;

	/* Get device instance from the look-up table */
	struct trng_module *module = _trng_instance;

	/* Read and mask interrupt flag register */
	interrupt_and_callback_status_mask = module->hw->INTFLAG.reg &
			(module->register_callback_mask & module->enable_callback_mask);

	/* Check if data ready needs to be serviced */
	if (interrupt_and_callback_status_mask & TRNG_INTFLAG_DATARDY) {
		/* Store random result in job buffer (it will clear data ready flag) */
		*(module->job_buffer++) = module->hw->DATA.reg;

		module->remaining_number -= 1;
		if (module->remaining_number == 0) {
			if (module->job_status == STATUS_BUSY) {
				/* Job is complete. Update status, disable interrupt
				 * and call callback */
				module->job_status = STATUS_OK;
				module->hw->INTENCLR.reg = TRNG_INTENCLR_DATARDY;
				(module->callback[TRNG_CALLBACK_READ_BUFFER])(module);
			}
		}
	}

	/* Clear interrupt flag */
	module->hw->INTFLAG.reg = TRNG_INTFLAG_DATARDY;
}

/**
 * \brief Read multiple random data from TRNG
 *
 * As soon as the TRNG is enabled, the module provides a new 32-bits
 * random data for every 84 CLK_TRNG_APB clock cycles.
 *
 * \param[in]  module_inst  Pointer to the TRNG software instance struct
 * \param[in]  number       Number of random data to get
 * \param[out] buffer       Buffer to store the random data
 *
 * \return Status of the job start.
 * \retval STATUS_OK        The read job was started successfully and is
 *                          in progress
 * \retval STATUS_BUSY      The TRNG is already busy with another job
 */
enum status_code trng_read_buffer_job(
		struct trng_module *const module_inst,
		uint32_t *buffer,
		uint32_t number)
{
	Assert(module_inst);
	Assert(number);
	Assert(buffer);

	if (module_inst->remaining_number != 0 ||
			module_inst->job_status == STATUS_BUSY) {
		return STATUS_BUSY;
	}

	module_inst->job_status = STATUS_BUSY;
	module_inst->remaining_number = number;
	module_inst->job_buffer = buffer;

	/* Enable data ready interrupt */
	module_inst->hw->INTENSET.reg = TRNG_INTENSET_DATARDY;

	return STATUS_OK;
}

/**
 * \brief Gets the status of a job
 *
 * Gets the status of an ongoing or the last job.
 *
 * \param [in]  module_inst Pointer to the TRNG software instance struct
 * \param [in]  type        Type of job to abort
 *
 * \return Status of the job.
 */
enum status_code trng_get_job_status(
		struct trng_module *module_inst,
		enum trng_job_type type)
{
	/* Sanity check arguments */
	Assert(module_inst);

	if (type == TRNG_JOB_READ_BUFFER) {
		return module_inst->job_status;
	} else {
		return STATUS_ERR_INVALID_ARG;
	}
}

/**
 * \brief Aborts an ongoing job
 *
 * \param [in]  module_inst Pointer to the TRNG software instance struct
 * \param [in]  type        Type of job to abort
 */
void trng_abort_job(
		struct trng_module *module_inst,
		enum trng_job_type type)
{
	/* Sanity check arguments */
	Assert(module_inst);

	if (type == TRNG_JOB_READ_BUFFER) {
		/* Disable interrupt */
		module_inst->hw->INTENCLR.reg = TRNG_INTENCLR_DATARDY;
		/* Mark job as aborted */
		module_inst->job_status = STATUS_ABORTED;
		module_inst->remaining_number = 0;
	}
}

