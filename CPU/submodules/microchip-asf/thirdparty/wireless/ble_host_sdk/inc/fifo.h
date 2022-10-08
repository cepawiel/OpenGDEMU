/**
 * \file fifo.h
 *
 * \brief FIFO declarations
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
#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>
#include <stdbool.h>
typedef struct fifo_handle_tag
{
    volatile uint32_t wr_idx; /* counter for FIFO input, inc in ISR only */
    volatile uint32_t rd_idx; /* counter for FIFO output, inc in MAIN only */
    uint32_t size;            /* FIFO buffer size */
    uint32_t mask;            /* Mask to extract buffer index from counters */
    uint8_t *buffer;          /* FIFO buffer */
} fifo_t;
uint8_t fifo_init(fifo_t *const fifo, uint8_t *const buffer, const uint32_t size);
uint8_t fifo_empty(fifo_t *const fifo);
uint8_t fifo_full(fifo_t *const fifo);
uint8_t fifo_get(fifo_t *const fifo, uint8_t *const byte);
uint8_t fifo_put(fifo_t *const fifo, const uint8_t byte);

#endif //__FIFO_H__
