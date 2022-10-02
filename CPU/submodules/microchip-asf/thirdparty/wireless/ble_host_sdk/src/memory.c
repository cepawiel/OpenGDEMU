/**
 * \file memory.c
 *
 * \brief Memory allocation for event
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

#include <stdint.h>
#include <stdbool.h>
#include "string.h"
#include "memory.h"

uint8_t *mem_pool_base_addr;
uint32_t mem_pool_size;
uint32_t max_count;
int32_t head;
int32_t tail;

void mem_init(uint8_t *base_addr, uint32_t size)
{
	mem_pool_base_addr = base_addr;
	mem_pool_size = size;
	head = -1;
	tail = -1;
}

void* mem_alloc(uint32_t size)
{
	uint8_t *mem;
	
	if((head == -1) && (tail == -1))
	{
		if(size > (mem_pool_size))
		{
			return NULL;
		}
		head = 0;
	}
	else if((head % mem_pool_size == (tail + 1) % mem_pool_size) ||
	(size > ((mem_pool_size -  (tail + 1)) + head) % mem_pool_size))
	{
		return NULL;
	}
	
	mem = mem_pool_base_addr + tail + 1;
	tail = (tail + size) % mem_pool_size;
	
	return mem;
}

uint8_t mem_free(void *ptr, uint32_t size)
{
	
	/* Pointer should not be NULL */
	if(NULL != ptr)
	{
		/* Deallocation should be in the same order in which it is allocated */
		if(ptr == mem_pool_base_addr + head)
		{
			/* Find next head */
			head = (head + size) % mem_pool_size;
			if(head == (int32_t)((tail + 1) % mem_pool_size))
			{
				head = tail = -1;
			}
			
			return MEM_DEALLOC_SUCCESS;
		}
		
		return MEM_INVALID_ADDRESS;
	}
	return MEM_DEALLOC_FAILURE;
}
