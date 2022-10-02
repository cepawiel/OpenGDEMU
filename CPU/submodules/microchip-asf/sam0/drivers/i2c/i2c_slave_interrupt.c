/**
 * \file
 *
 * \brief I2C Master Interrupt Driver for SAMB
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
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

#include "i2c_slave_interrupt.h"

void *_i2c_instances;

/**
 * \internal
 * Reads next data. Used by interrupt handler to get next data byte from master.
 *
 * \param[in,out] module  Pointer to software module structure
 */
static void _i2c_slave_read(
		struct i2c_slave_module *const module)
{
	/* Sanity check */
	Assert(module);
	Assert(module->hw);
	
	I2c *const i2c_module = module->hw;

	/* Read byte from master and put in buffer. */
	*(module->buffer++) = i2c_module->RECEIVE_DATA.reg;

	/*Decrement remaining buffer length */
	module->buffer_remaining--;
}

/**
 * \internal
 * Writes next data. Used by interrupt handler to send next data byte to master.
 *
 * \param[in,out] module  Pointer to software module structure
 */
static void _i2c_slave_write(
		struct i2c_slave_module *const module)
{
	/* Sanity check */
	Assert(module);
	Assert(module->hw);
	
	I2c *const i2c_module = module->hw;

	/* Write byte from buffer to master */
	i2c_module->TRANSMIT_DATA.reg = *(module->buffer++);

	/*Decrement remaining buffer length */
	module->buffer_remaining--;
}

/**
 * \brief Registers callback for the specified callback type
 *
 * Associates the given callback function with the
 * specified callback type. To enable the callback, the
 * \ref i2c_slave_enable_callback function must be used.
 *
 * \param[in,out] module         Pointer to the software module struct
 * \param[in]     callback       Pointer to the function desired for the
 *                               specified callback
 * \param[in]     callback_type  Callback type to register
 */
void i2c_slave_register_callback(
		struct i2c_slave_module *const module,
		i2c_slave_callback_t callback,
		enum i2c_slave_callback callback_type)
{
	/* Sanity check. */
	Assert(module);
	Assert(module->hw);
	Assert(callback);

	/* Register callback. */
	module->callbacks[callback_type] = callback;

	/* Set corresponding bit to set callback as initiated. */
	module->registered_callback |= (1 << callback_type);
}

/**
 * \brief Unregisters callback for the specified callback type
 *
 * Removes the currently registered callback for the given callback
 * type.
 *
 * \param[in,out]  module         Pointer to the software module struct
 * \param[in]      callback_type  Callback type to unregister
 */
void i2c_slave_unregister_callback(
		struct i2c_slave_module *const module,
		enum i2c_slave_callback callback_type)
{
	/* Sanity check. */
	Assert(module);
	Assert(module->hw);

	/* Register callback. */
	module->callbacks[callback_type] = NULL;

	/* Set corresponding bit to set callback as initiated. */
	module->registered_callback &= ~(1 << callback_type);
}

/**
 * \brief Initiates a reads packet operation
 *
 * Reads a data packet from the master. A write request must be initiated by
 * the master before the packet can be read.
 *
 * The \ref I2C_SLAVE_CALLBACK_WRITE_REQUEST callback can be used to call this
 * function.
 *
 * \param[in,out] module  Pointer to software module struct
 * \param[in,out] packet  Pointer to I<SUP>2</SUP>C packet to transfer
 *
 * \return Status of starting asynchronously reading I<SUP>2</SUP>C packet.
 * \retval STATUS_OK    If reading was started successfully
 * \retval STATUS_BUSY  If module is currently busy with another transfer
 */
enum status_code i2c_slave_read_packet_job(
		struct i2c_slave_module *const module,
		struct i2c_slave_packet *const packet)
{
	/* Sanity check */
	Assert(module);
	Assert(module->hw);
	Assert(packet);

	I2c *const i2c_module = module->hw;
	
	/* Check if the I2C module is busy doing async operation. */
	if (module->buffer_remaining > 0) {
		return STATUS_BUSY;
	}

	/* Save packet to software module. */
	module->buffer           = packet->data;
	module->buffer_remaining = packet->data_length;
	module->buffer_length    = packet->data_length;
	module->status           = STATUS_BUSY;

	/* Enable interrupts */
	i2c_slave_rx_interrupt(i2c_module, true);

	return STATUS_OK;
}

/**
 * \brief Initiates a write packet operation
 *
 * Writes a data packet to the master. A read request must be initiated by
 * the master before the packet can be written.
 *
 * The \ref I2C_SLAVE_CALLBACK_READ_REQUEST callback can be used to call this
 * function.
 *
 * \param[in,out] module  Pointer to software module struct
 * \param[in,out] packet  Pointer to I<SUP>2</SUP>C packet to transfer
 *
 * \return Status of starting writing I<SUP>2</SUP>C packet.
 * \retval STATUS_OK   If writing was started successfully
 * \retval STATUS_BUSY If module is currently busy with another transfer
 */
enum status_code i2c_slave_write_packet_job(
		struct i2c_slave_module *const module,
		struct i2c_slave_packet *const packet)
{
	/* Sanity check */
	Assert(module);
	Assert(module->hw);
	Assert(packet);
	
	I2c *const i2c_module = module->hw;
	
	if (module->buffer_remaining > 0) {
		return STATUS_BUSY;
	}

	/* Save packet to software module. */
	module->buffer           = packet->data;
	module->buffer_remaining = packet->data_length;
	module->buffer_length    = packet->data_length;
	module->status			 = STATUS_BUSY;

	/* Enable interrupts */
	i2c_slave_tx_interrupt(i2c_module, true);

	return STATUS_OK;
}

/**
 * \internal Interrupt handler for I<SUP>2</SUP>C slave
 *
 * \param[in] instance  I2C instance that triggered the interrupt
 */
void _i2c_slave_rx_isr_handler(void)
{
	/* Get software module for callback handling. */
	struct i2c_slave_module *module =
			(struct i2c_slave_module*)_i2c_instances;

	Assert(module);

	I2c *const i2c_module = module->hw;

	/* Combine callback registered and enabled masks. */
	uint8_t callback_mask =
			module->enabled_callback & module->registered_callback;

	if (i2c_module->RECEIVE_STATUS.reg & I2C_RECEIVE_STATUS_RX_FIFO_NOT_EMPTY) {
		if (!module->buffer_length && (module->buffer_length == module->buffer_remaining)) {
			module->transfer_direction = I2C_TRANSFER_WRITE;
			if (callback_mask & (1 << I2C_SLAVE_CALLBACK_WRITE_REQUEST)) {
				/* Write to master complete */
				module->callbacks[I2C_SLAVE_CALLBACK_WRITE_REQUEST](module);
			}
		} 
		/* Continue buffer write/read */
		if (module->buffer_length > 0 && module->buffer_remaining > 0) {
				_i2c_slave_read(module);
		}
		if (!module->buffer_remaining) {
			module->status = STATUS_OK;
			module->buffer_length = 0;
			if (callback_mask & (1 << I2C_SLAVE_CALLBACK_WRITE_COMPLETE)) {
				/* Write to master complete */
				module->callbacks[I2C_SLAVE_CALLBACK_WRITE_COMPLETE](module);
			}
		}
	}
	if ((i2c_module->RECEIVE_STATUS.reg & I2C_RECEIVE_STATUS_NAK)) { //&&
			//module->transfer_direction == I2C_TRANSFER_READ) {
		/* Received NAK, master received completed. */
		i2c_module->RX_INTERRUPT_MASK.bit.NAK_MASK = 0;
		if (callback_mask & (1 << I2C_SLAVE_CALLBACK_READ_COMPLETE)) {
			module->callbacks[I2C_SLAVE_CALLBACK_READ_COMPLETE](module);
		}
	}
	
	if (module->hw == I2C0) {
		NVIC_ClearPendingIRQ(I2C0_RX_IRQn);
	} else if (module->hw == I2C1) {
		NVIC_ClearPendingIRQ(I2C1_RX_IRQn);
	}
}

void _i2c_slave_tx_isr_handler(void)
{
	/* Get software module for callback handling. */
	struct i2c_slave_module *module =
			(struct i2c_slave_module*)_i2c_instances;

	Assert(module);

	I2c *const i2c_module = module->hw;

	/* Combine callback registered and enabled masks. */
	uint8_t callback_mask =
			module->enabled_callback & module->registered_callback;
	
	if (!module->buffer_length && (module->buffer_length == module->buffer_remaining)) {
		/* First timer interrupt */
		module->transfer_direction = I2C_TRANSFER_READ;
		if (callback_mask & (1 << I2C_SLAVE_CALLBACK_READ_REQUEST)) {
			/* Write to master complete */
			module->callbacks[I2C_SLAVE_CALLBACK_READ_REQUEST](module);
		}
	} 
	if (module->buffer_length > 0 && module->buffer_remaining > 0) {
		_i2c_slave_write(module);
	}
	if (!module->buffer_remaining) {
		module->status = STATUS_OK;
		module->buffer_length = 0;
		i2c_module->RX_INTERRUPT_MASK.bit.NAK_MASK = 0;
		if (callback_mask & (1 << I2C_SLAVE_CALLBACK_READ_COMPLETE)) {
			module->callbacks[I2C_SLAVE_CALLBACK_READ_COMPLETE](module);
		}
	}
	
	if (module->hw == I2C0) {
		NVIC_ClearPendingIRQ(I2C0_TX_IRQn);
	} else if (module->hw == I2C1) {
		NVIC_ClearPendingIRQ(I2C1_TX_IRQn);
	}
}