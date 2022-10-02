/**
 * \file
 *
 * \brief This file controls the software Serial FIFO management.
 *
 * Copyright (c) 2017-2018 Microchip Technology Inc. and its subsidiaries.
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

#include "serial_fifo.h"

int ser_fifo_init(ser_fifo_desc_t *fifo_desc, void *buffer, uint16_t size)
{
	// Check the size parameter. It must be not null...
	Assert (size);

	// ... must be a 2-power ...
	Assert (!(size & (size - 1)));

	// ... and must fit in a uint16_t. Since the read and write indexes are using a
	// double-index range implementation, the max FIFO size is thus 32768 items.
	Assert (size <= 32768);

	// Serial Fifo starts empty.
	fifo_desc->read_index  = 0;
	fifo_desc->write_index = 0;

	// Save the size parameter.
	fifo_desc->size = size;

	// Create a mask to speed up the FIFO management (index swapping).
	fifo_desc->mask = (2 * (uint16_t)size) - 1;

	// Save the buffer pointer.
	fifo_desc->buffer.u8ptr = buffer;

	return SER_FIFO_OK;
}
