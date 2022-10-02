/**
* \file circularBuffer.h
*
* \brief Circular Buffer interface
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

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "compiler.h"

COMPILER_PACK_SET(1)

typedef struct {
	uint8_t *buffer;
	uint16_t size;
	uint16_t head;
	uint16_t tail;
}circularBuffer_t;

COMPILER_PACK_RESET()

void initCircularBuffer(circularBuffer_t *circularBuffer, uint8_t *buffer, uint16_t size);
uint16_t readCircularBuffer(circularBuffer_t *circularBuffer, uint16_t maxReadLength, uint8_t *readBuffer);
void writeCircularBuffer(circularBuffer_t *circularBuffer, uint16_t length, uint8_t *value);

#endif /* CIRCULAR_BUFFER_H */
