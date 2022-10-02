/**
 * \file event_mem.c
 *
 * \brief Event mechanism
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
#include "event_mem.h"
#include "bm_application_mode.h"

extern event_status_t interface_event_get(void);

static event_fifo_t event_fifo =
{
	.buffer = NULL,
	.size = 0,
	.head = 0,
	.tail = 0,
    .wrap_flag = false,
};

bool event_fifo_init(event_t* event_mem, uint16_t event_mem_size)
{
	if (event_mem != NULL && event_mem_size > 0)
	{
		event_fifo.buffer = event_mem;
		event_fifo.size = event_mem_size;
		event_fifo.head = 0;
		event_fifo.tail = 0;
        event_fifo.wrap_flag = false;
		return true;
	}
	return false;
}

bool event_fifo_empty(void)
{
	return ((event_fifo.tail == event_fifo.head) && !event_fifo.wrap_flag);
}

bool event_fifo_full(void)
{
	return ((event_fifo.tail == event_fifo.head) && event_fifo.wrap_flag);
}

event_status_t event_fifo_write(event_t *event)
{
	if (!event_fifo_full())
	{
		((event_t*)(event_fifo.buffer + event_fifo.head))->event_msg.data = mem_alloc(event->event_msg.data_len);
		if (((event_t*)(event_fifo.buffer + event_fifo.head))->event_msg.data == NULL)
		{
			return BLE_EVENT_Q_INSUFFICIENT_MEMORY;
		}
		
		memcpy(((event_t*)(event_fifo.buffer + event_fifo.head))->event_msg.data, event->event_msg.data, event->event_msg.data_len);
		((event_t*)(event_fifo.buffer + event_fifo.head))->event_msg.data_len = event->event_msg.data_len;
		((event_t*)(event_fifo.buffer + event_fifo.head++))->event_id = event->event_id;
		
		if(event_fifo.head >= event_fifo.size)
		{
            event_fifo.wrap_flag = true;
			event_fifo.head = 0;
		}
		return BLE_EVENT_SUCCESS;
	}
	
	return BLE_EVENT_Q_OVERFLOW;
}

event_status_t event_fifo_read(event_t* event)
{
	if (!event_fifo_empty())
	{
		*event = *(event_fifo.buffer + event_fifo.tail++);
        
		if(event_fifo.tail >= event_fifo.size)
		{
            event_fifo.wrap_flag = false;
			event_fifo.tail = 0;
		} 
		return BLE_EVENT_SUCCESS;
	}
	
	return BLE_EVENT_Q_EMPTY;
}

event_status_t ble_event_get(event_t *event_param)
{
	event_status_t status = BLE_EVENT_Q_EMPTY;
	event_t event = {0};
	
	event_param->event_id = 0;
	/* Check for event in Event Q */
	status = event_fifo_read(&event);
	
	if(BLE_EVENT_SUCCESS == status)
	{
		event_param->event_id = event.event_id;
		event_param->event_msg.data_len = event.event_msg.data_len;
		memcpy(event_param->event_msg.data, event.event_msg.data, event.event_msg.data_len);
		mem_free(event.event_msg.data, event.event_msg.data_len);
		BM_Application_EventParser(event_param->event_id, event_param->event_msg.data, &(event_param->event_msg.data_len));
	}
	else  /* BLE_EVENT_Q_EMPTY */
	{
		/* Check for oncoming event when Event Q is empty */
		if((status = interface_event_get()) == BLE_EVENT_SUCCESS)
		{
			status = event_fifo_read(&event);
			if(BLE_EVENT_SUCCESS == status)
			{
				event_param->event_id = event.event_id;
				event_param->event_msg.data_len = event.event_msg.data_len;
				memcpy(event_param->event_msg.data, event.event_msg.data, event.event_msg.data_len);
				mem_free(event.event_msg.data, event.event_msg.data_len);
				BM_Application_EventParser(event_param->event_id, event_param->event_msg.data, &(event_param->event_msg.data_len));
			}
		}
	}
	
	return status;
}
