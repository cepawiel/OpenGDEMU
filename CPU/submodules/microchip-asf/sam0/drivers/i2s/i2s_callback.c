/**
 * \file
 *
 * \brief SAM I<SUP>2</SUP>S - Inter-IC Sound Controller
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

#include "i2s_callback.h"

struct i2s_module *_i2s_instances[I2S_INST_NUM];

static void _i2s_interrupt_handler(const uint8_t instance)
{
	struct i2s_module *module = _i2s_instances[instance];
	struct i2s_serializer_module *data_module;

	/* Get interrupt flags */
	uint32_t intflag = module->hw->INTFLAG.reg;
	uint32_t inten = intflag & module->hw->INTENSET.reg;
	uint32_t run_flags = (I2S_INTFLAG_TXUR0 | I2S_INTFLAG_RXOR0);
	uint32_t ready_flags = (I2S_INTFLAG_TXRDY0 | I2S_INTFLAG_RXRDY0);
	uint32_t call_mask;
	uint8_t serializer;

	for (serializer = 0; serializer < 2; serializer ++) {
		data_module = &module->serializer[serializer];
		call_mask = data_module->registered_callback_mask &
				data_module->enabled_callback_mask;

		if (intflag & (run_flags | ready_flags)) {
			/* Serializer Tx ready */
			if ((I2S_INTFLAG_TXRDY0 << serializer) & inten) {

				if (data_module->transferred_words <
					data_module->requested_words) {

					/* Write data word */
					while (module->hw->SYNCBUSY.reg &
						(I2S_SYNCBUSY_DATA0 << serializer)) {
						/* Wait sync */
					}
					switch(data_module->data_size) {
					case I2S_DATA_SIZE_32BIT:
					case I2S_DATA_SIZE_24BIT:
					case I2S_DATA_SIZE_20BIT:
					case I2S_DATA_SIZE_18BIT:
						module->hw->DATA[serializer].reg =
							((uint32_t*)data_module->job_buffer) \
								[data_module->transferred_words];
						break;
					case I2S_DATA_SIZE_16BIT:
					case I2S_DATA_SIZE_16BIT_COMPACT:
						module->hw->DATA[serializer].reg =
							((uint16_t*)data_module->job_buffer) \
								[data_module->transferred_words];
						break;
					default:
						module->hw->DATA[serializer].reg =
							((uint8_t*)data_module->job_buffer) \
								[data_module->transferred_words];
					}
					/* Clear interrupt status */
					module->hw->INTFLAG.reg = I2S_INTFLAG_TXRDY0 << serializer;

					/* Count data */
					data_module->transferred_words ++;
				}

				/* Check if the buffer is done */
				if (data_module->transferred_words >=
					data_module->requested_words) {
					/* It's done */
					data_module->job_status = STATUS_OK;
					/* Disable interrupt */
					module->hw->INTENCLR.reg =
							I2S_INTFLAG_TXRDY0 << serializer;
					/* Invoke callback */
					if ((1 << I2S_SERIALIZER_CALLBACK_BUFFER_DONE) &
						call_mask) {
						(data_module->callback \
							[I2S_SERIALIZER_CALLBACK_BUFFER_DONE])(module);
					}
				}
				return;
			}
			/* Serializer Rx ready */
			if ((I2S_INTFLAG_RXRDY0 << serializer) & inten) {
				/* Read data word */
				switch(data_module->data_size) {
				case I2S_DATA_SIZE_32BIT:
				case I2S_DATA_SIZE_24BIT:
				case I2S_DATA_SIZE_20BIT:
				case I2S_DATA_SIZE_18BIT:
					((uint32_t*)data_module->job_buffer) \
							[data_module->transferred_words] =
									module->hw->DATA[serializer].reg;
					break;
				case I2S_DATA_SIZE_16BIT:
				case I2S_DATA_SIZE_16BIT_COMPACT:
					((uint16_t*)data_module->job_buffer) \
							[data_module->transferred_words] =
									(uint16_t)module->hw->DATA[serializer].reg;
					break;
				default:
					((uint8_t*)data_module->job_buffer) \
							[data_module->transferred_words] =
									(uint8_t)module->hw->DATA[serializer].reg;

				}
				/* Clear interrupt status */
				module->hw->INTFLAG.reg = I2S_INTFLAG_RXRDY0 << serializer;

				/* Count data */
				data_module->transferred_words ++;

				/* Check if the buffer is done */
				if (data_module->transferred_words >=
					data_module->requested_words) {
					if (data_module->job_status == STATUS_BUSY) {
						data_module->job_status = STATUS_OK;
						/* Disable interrupt */
						module->hw->INTENCLR.reg =
								I2S_INTFLAG_RXRDY0 << serializer;
						/* Invoke callback */
						if ((1 << I2S_SERIALIZER_CALLBACK_BUFFER_DONE) &
							call_mask) {
							(data_module->callback \
								[I2S_SERIALIZER_CALLBACK_BUFFER_DONE])(module);
						}
					}
				}
				return;
			}
			/* Serializer Tx undrerun or Rx overrun */
			if (run_flags & inten) {
				module->hw->INTFLAG.reg = I2S_INTFLAG_TXUR0 << serializer;
				if ((1 << I2S_SERIALIZER_CALLBACK_OVER_UNDER_RUN) &
					call_mask) {
					(data_module->callback \
						[I2S_SERIALIZER_CALLBACK_OVER_UNDER_RUN])(module);
				}
				return;
			}
		}
		run_flags <<= 1;
		ready_flags <<= 1;
	}

}

/** Interrupt handler for the I<SUP>2</SUP>S module */
void I2S_Handler(void)
{
	_i2s_interrupt_handler(0);
}

/**
 * \brief Write buffer to the specified Serializer of I<SUP>2</SUP>S module
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 * \param[in]  serializer    The serializer to write to
 * \param[in]  buffer        The data buffer to write
 * \param[in]  size          Number of data words to write
 *
 * \return Status of the initialization procedure.
 *
 * \retval STATUS_OK           The data was sent successfully
 * \retval STATUS_ERR_DENIED   The serializer is not in transmit mode
 * \retval STATUS_ERR_INVALID_ARG  An invalid buffer pointer was supplied
 */
enum status_code i2s_serializer_write_buffer_job(
		struct i2s_module *const module_inst,
		const enum i2s_serializer serializer,
		const void *buffer,
		const uint32_t size)
{
	struct i2s_serializer_module *data_module;

	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);
	Assert(serializer < I2S_SERIALIZER_N);
	
	data_module = &module_inst->serializer[serializer];

	/* Serializer must in transmit mode */
	if (data_module->mode != I2S_SERIALIZER_TRANSMIT) {
		return STATUS_ERR_DENIED;
	}

	/* Buffer should be aligned */
	switch(data_module->data_size) {
	case I2S_DATA_SIZE_32BIT:
	case I2S_DATA_SIZE_24BIT:
	case I2S_DATA_SIZE_20BIT:
	case I2S_DATA_SIZE_18BIT:
		if ((uint32_t)buffer & 0x3) {
			return STATUS_ERR_INVALID_ARG;
		}
		break;
	case I2S_DATA_SIZE_16BIT:
	case I2S_DATA_SIZE_16BIT_COMPACT:
		if ((uint32_t)buffer & 0x1) {
			return STATUS_ERR_INVALID_ARG;
		}
		break;
	default:
		break;
	}

	data_module = &module_inst->serializer[serializer];
	if (data_module->job_status == STATUS_BUSY) {
		return STATUS_BUSY;
	}

	data_module->job_status = STATUS_BUSY;
	data_module->requested_words = size;
	data_module->transferred_words = 0;
	data_module->job_buffer = (void*)buffer;

	module_inst->hw->INTENSET.reg = (I2S_INTENSET_TXRDY0 << serializer);

	return STATUS_OK;
}

/**
 * \brief Read from the specified Serializer of I<SUP>2</SUP>S module to a buffer
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 * \param[in]  serializer    The serializer to write to
 * \param[out] buffer        The buffer to fill read data
 * \param[in]  size          Number of data words to read
 *
 * \return Status of the initialization procedure.
 *
 * \retval STATUS_OK           The data was sent successfully
 * \retval STATUS_ERR_DENIED   The serializer is not in receive mode
 * \retval STATUS_ERR_INVALID_ARG  An invalid buffer pointer was supplied
 */
enum status_code i2s_serializer_read_buffer_job(
		struct i2s_module *const module_inst,
		const enum i2s_serializer serializer,
		void *buffer,
		const uint32_t size)
{
	struct i2s_serializer_module *data_module;

	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);
	Assert(serializer < I2S_SERIALIZER_N);

	data_module = &module_inst->serializer[serializer];

	/* Serializer must in receive mode */
	if (data_module->mode == I2S_SERIALIZER_TRANSMIT) {
		return STATUS_ERR_DENIED;
	}

	/* Data buffer must be aligned */
	switch(data_module->data_size) {
	case I2S_DATA_SIZE_32BIT:
	case I2S_DATA_SIZE_24BIT:
	case I2S_DATA_SIZE_20BIT:
	case I2S_DATA_SIZE_18BIT:
		if ((uint32_t)buffer & 0x3) {
			return STATUS_ERR_INVALID_ARG;
		}
		break;
	case I2S_DATA_SIZE_16BIT:
	case I2S_DATA_SIZE_16BIT_COMPACT:
		if ((uint32_t)buffer & 0x1) {
			return STATUS_ERR_INVALID_ARG;
		}
		break;
	default:
		break;
	}

	data_module = &module_inst->serializer[serializer];
	if (data_module->job_status == STATUS_BUSY) {
		return STATUS_BUSY;
	}

	data_module->job_status = STATUS_BUSY;
	data_module->requested_words = size;
	data_module->transferred_words = 0;
	data_module->job_buffer = (void*)buffer;

	module_inst->hw->INTENCLR.reg = (I2S_INTENSET_RXRDY0 << serializer);
	module_inst->hw->INTENSET.reg = (I2S_INTENSET_RXRDY0 << serializer);

	return STATUS_OK;
}

/**
 * \brief Aborts an ongoing job running on serializer
 *
 * Aborts an ongoing job.
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 * \param[in]  serializer    The serializer which runs the job
 * \param[in]  job_type      Type of job to abort
 */
void i2s_serializer_abort_job(
		struct i2s_module *const module_inst,
		const enum i2s_serializer serializer,
		const enum i2s_job_type job_type)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);
	Assert(serializer < I2S_SERIALIZER_N);

	if (job_type == I2S_JOB_WRITE_BUFFER) {
		/* Disable interrupt */
		module_inst->hw->INTENCLR.reg = (I2S_INTENCLR_TXRDY0 << serializer);
		/* Mark job as aborted */
		module_inst->serializer[serializer].job_status = STATUS_ABORTED;
	} else if (job_type == I2S_JOB_READ_BUFFER) {
		/* Disable interrupt */
		module_inst->hw->INTENCLR.reg = (I2S_INTENCLR_RXRDY0 << serializer);
		/* Mark job as aborted */
		module_inst->serializer[serializer].job_status = STATUS_ABORTED;
	}
}

/**
 * \brief Gets the status of a job running on serializer
 *
 * Gets the status of an ongoing or the last job.
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 * \param[in]  serializer    The serializer which runs the job
 * \param[in]  job_type      Type of job to abort
 *
 * \return Status of the job.
 */
enum status_code i2s_serializer_get_job_status(
		const struct i2s_module *const module_inst,
		const enum i2s_serializer serializer,
		const enum i2s_job_type job_type)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(serializer < I2S_SERIALIZER_N);

	if (job_type == I2S_JOB_WRITE_BUFFER || job_type == I2S_JOB_READ_BUFFER) {
		return module_inst->serializer[serializer].job_status;
	} else {
		return STATUS_ERR_INVALID_ARG;
	}
}
