/**
* \file circularBuffer.c
*
* \brief Circular Buffer implementation
*
* Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
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
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
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

#include "circularBuffer.h"

void initCircularBuffer(circularBuffer_t *circularBuffer, uint8_t *buffer, uint16_t size)
{
	circularBuffer->buffer = buffer;
	circularBuffer->size = size;
	circularBuffer->head = 0;
	circularBuffer->tail = 0;
}

uint16_t readCircularBuffer(circularBuffer_t *circularBuffer, uint16_t maxReadLength, uint8_t *readBuffer)
{
	uint16_t data_received = 0;
	uint16_t readCount = 0;
	
	if (NULL == circularBuffer || NULL == circularBuffer->buffer)
	{
		return 0;
	}
	
	if(circularBuffer->tail >= circularBuffer->head)
	{
		readCount = circularBuffer->tail - circularBuffer->head;
	}
	else
	{
		readCount = circularBuffer->tail + (circularBuffer->size - circularBuffer->head);
	}
	
	if (0 == readCount) {
		return 0;
	}

	if (circularBuffer->size <= readCount) {
		/*
		 * Bytes between head and tail are overwritten by new data.
		 * The oldest data in buffer is the one to which the tail is
		 * pointing. So reading operation should start from the tail.
		 */
		circularBuffer->head = circularBuffer->tail;

		/*
		 * This is a buffer overflow case. But still only the number of
		 * bytes equivalent to
		 * full buffer size are useful.
		 */
		readCount = circularBuffer->size;

		/* Bytes received is more than or equal to buffer. */
		if (circularBuffer->size <= maxReadLength) {
			/*
			 * Requested receive length (max_length) is more than
			 * the
			 * max size of receive buffer, but at max the full
			 * buffer can be read.
			 */
			maxReadLength = circularBuffer->size;
		}
	} else {
		/* Bytes received is less than receive buffer maximum length. */
		if (maxReadLength > readCount) {
			/*
			 * Requested receive length (max_length) is more than
			 * the data
			 * present in receive buffer. Hence only the number of
			 * bytes
			 * present in receive buffer are read.
			 */
			maxReadLength = readCount;
		}
	}

	data_received = maxReadLength;
	while (maxReadLength > 0) {
		/* Start to copy from head. */
		*readBuffer = circularBuffer->buffer[circularBuffer->head];
		readBuffer++;
		maxReadLength--;
		if ((circularBuffer->size - 1) == circularBuffer->head) {
			circularBuffer->head = 0;
		}
		else
		{
			circularBuffer->head++;
		}
	}
	return data_received;
}

void writeCircularBuffer(circularBuffer_t *circularBuffer, uint16_t length, uint8_t *value)
{
	uint16_t bufferIndex = 0;

	if (NULL == circularBuffer || NULL == circularBuffer->buffer)
	{
		return;
	}
	
	while (length > 0)
	{
		circularBuffer->buffer[circularBuffer->tail] = value[bufferIndex];

		if ((circularBuffer->size - 1) == circularBuffer->tail) {
			/* Reached the end of buffer, revert back to beginning of
			 * buffer. */
			circularBuffer->tail = 0x00;
		} else {
			circularBuffer->tail++;
		}
		--length;
		++bufferIndex;
	}
}

