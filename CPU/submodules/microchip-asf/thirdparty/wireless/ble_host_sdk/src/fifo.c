/**
 * \file fifo.c
 *
 * \brief FIFO definitions
 *
 * Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
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
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

/* 
   lock-free FIFO, used to receive data in ISR context,
   and process it in Main context
   for optimization size MUST be power of 2, ie 2, 4, 8, 16, 32, ...
*/
#include "fifo.h"
#include <stddef.h>

uint8_t fifo_init(fifo_t *const fifo, uint8_t *const buffer, const uint32_t size)
{
    /* check buffer pointer, and buffer size
		buffer size MUST be 2 to the power n
	*/
    if (NULL != fifo && size > 0 && (size & (size - 1)) == 0)
    {
        fifo->buffer = buffer;
        fifo->size = size;
        /* valid indexes can be used to access array of size n is from 0 to n-1
			mask used to get the valid index to access fifo buffer from wr_idx and rd_idx
			for example size = 128 (dec) = 00000080 (hex) = 00000000000000000000000010000000 (bin)
            mask = 127 (dec) = 0000007F (hex) = 00000000000000000000000001111111 (bin)
		*/
        fifo->mask = size - 1;
        fifo->wr_idx = 0;
        fifo->rd_idx = 0;
        return 1;
    }
    return 0;
}
uint8_t fifo_empty(fifo_t *const fifo)
{
    return (fifo->wr_idx == fifo->rd_idx);
}
uint8_t fifo_full(fifo_t *const fifo)
{
    return ((fifo->wr_idx - fifo->rd_idx) == fifo->size);
}
uint8_t fifo_get(fifo_t *const fifo, uint8_t *const byte)
{
    if (!fifo_empty(fifo))
    {
        /* get the valid index from rd_idx by masking current rd_idx with pre calculated mask
			it will convert all bit to zeros except bits inside valid range of bits
		*/
        *byte = fifo->buffer[fifo->mask & fifo->rd_idx++];
        return 1;
    }
    return 0;
}
uint8_t fifo_put(fifo_t *const fifo, const uint8_t byte)
{
    if (!fifo_full(fifo))
    {
        /* get the valid index from rd_idx by masking current wr_idx with pre calculated mask
			it will convert all bit to zeros except bits inside valid range of bits
		*/
        fifo->buffer[fifo->mask & fifo->wr_idx++] = byte;
        return 1;
    }
    return 0;
}
