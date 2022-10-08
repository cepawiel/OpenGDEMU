/**
 * \file event_mem.h
 *
 * \brief Event memory declarations
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

#ifndef __EVENT_MEM_H__
#define __EVENT_MEM_H__

typedef enum
{
	BLE_EVENT_SUCCESS,
	BLE_EVENT_Q_OVERFLOW,
	BLE_EVENT_Q_EMPTY,
	BLE_EVENT_Q_INSUFFICIENT_MEMORY,
}event_status_t;

typedef struct
{
	uint8_t *data;
	uint32_t data_len;
}event_msg_t;

typedef struct
{
	event_msg_t event_msg;
	uint8_t event_id;
}event_t;

typedef struct _event_fifo_t
{
	uint16_t head;
	uint16_t tail;
    bool     wrap_flag;
	uint16_t size;
	event_t* buffer;
} event_fifo_t;

bool event_fifo_init(event_t* event_mem, uint16_t event_mem_size);
bool event_fifo_empty(void);
bool event_fifo_full(void);
event_status_t event_fifo_write(event_t *event);
event_status_t event_fifo_read(event_t* event);
event_status_t ble_event_get(event_t *event_param);

#endif //__EVENT_MEM_H__
