/**
 * \file interface.c
 *
 * \brief BM module Interface definitions
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

#include "platform_files.h"
#include "bm_mode.h"
#include "bm_application_mode.h"
#include "bm_program_mode.h"
#include "bm_utils.h"
#include "fifo.h"
#include "event_mem.h"
#include "memory.h"
#include "ble_api.h"
#include "dfu_api.h"
#include "interface.h"


static fifo_t rx_fifo;
static uint8_t rx_fifo_buffer[1024];
static bool is_fifo_full = false;
static uint8_t resp_event_id = INVALID_EVENT_ID;
static void *platform_timer_handle = NULL;
static sw_timer_t sw_timers[INTERFACE_SW_TIMERS] = {0};
static bool hw_timer_status = 0;
static void* interface_uart_timer_handle = NULL;
static void* interface_event_timer_handle = NULL;
static bool interface_uart_timer_timeout = false;
static bool interface_event_timer_timeout = false;

ble_platform_api_list_t *platform;
uart_write_sync_cb_t ble_uart_send_sync;
uart_read_async_cb_t ble_uart_receive_async;

extern event_t appEvent;


static void interface_timer_callback(void *arg1)
{
    uint8_t index;
    platform->start_timer(platform_timer_handle, 1);
    hw_timer_status = 0;
    for (index = 0; index < INTERFACE_SW_TIMERS; index++)
    {
	    if (sw_timers[index].timer_usage == 1 && sw_timers[index].timer_counter > 0)
	    {
		    sw_timers[index].timer_counter--;
		    if (sw_timers[index].timer_counter == 0 && sw_timers[index].timer_cb != NULL)
		    {
			    sw_timers[index].timer_cb();
		    }
		    if (sw_timers[index].timer_counter > 0)
		    {
			    hw_timer_status = 1;
		    }
	    }
    }
    if (0 == hw_timer_status)
    {
	    platform->stop_timer(platform_timer_handle);
    }
	
	arg1 = arg1;
}

static void interface_uart_timer_cb(void)
{
	interface_uart_timer_timeout = true;
}

static void interface_event_timer_cb(void)
{
	interface_event_timer_timeout = true;
}

static void* interface_create_timer(timer_cb_t timer_cb)
{
	uint32_t index;
	for (index = 0; index < INTERFACE_SW_TIMERS; index++)
	{
		if (sw_timers[index].timer_usage == 0)
		{
			sw_timers[index].timer_usage = 1;
			sw_timers[index].timer_counter = 0;
			sw_timers[index].timer_cb = timer_cb;
			return &sw_timers[index];
		}
	}
	return NULL;
}

#if 0	/* Removed, just to avoid compiler error */
static void interface_delete_timer(void *timer_handle)
{
	uint32_t index;
	sw_timer_t *temp_sw_timer = (sw_timer_t *)timer_handle;
	temp_sw_timer->timer_usage = 0;
	temp_sw_timer->timer_counter = 0;
	temp_sw_timer->timer_cb = NULL;
	hw_timer_status = 0;
	for (index = 0; index < INTERFACE_SW_TIMERS; index++)
	{
		if (sw_timers[index].timer_usage == 1 && sw_timers[index].timer_counter > 0)
		{
			hw_timer_status = 1;
			break;
		}
	}
	if (0 == hw_timer_status)
	{
		platform->stop_timer(platform_timer_handle);
	}
}
#endif /* #if 0 */

static void interface_start_timer(void *timer_handle, uint32_t ms)
{
	sw_timer_t *temp_sw_timer = (sw_timer_t *)timer_handle;
	temp_sw_timer->timer_counter = ms;
	if (0 == hw_timer_status)
	{
		platform->start_timer(platform_timer_handle, 1);
		hw_timer_status = 1;
	}
}

static void interface_stop_timer(void *timer_handle)
{
	uint32_t index;
	sw_timer_t *temp_sw_timer = (sw_timer_t *)timer_handle;
	temp_sw_timer->timer_counter = 0;
	hw_timer_status = 0;
	for (index = 0; index < INTERFACE_SW_TIMERS; index++)
	{
		if (sw_timers[index].timer_usage == 1 && sw_timers[index].timer_counter > 0)
		{
			hw_timer_status = 1;
			break;
		}
	}
	if (0 == hw_timer_status)
	{
		platform->stop_timer(platform_timer_handle);
	}
}


/*
 * @brief Initialize the interface layer
 *
 * @param pointer to platform_init which contains platform API list and event memory
 *
 * @return status of interface initialization
*/
ble_status_t  interface_init(platform_init_t *platform_init)
{
	init_fifo();
	
	if((NULL == platform_init->event_mem.event_mem_pool) || (0 == platform_init->event_mem.event_mem_pool_size))
	{
		return BLE_FAILURE;
	}
	if((NULL == platform_init->event_mem.event_payload_mem) || (0 == platform_init->event_mem.event_payload_mem_size))
	{
		return BLE_FAILURE;
	}
	
	mem_init((uint8_t *)platform_init->event_mem.event_payload_mem, platform_init->event_mem.event_payload_mem_size);
	/* Initialize an event queue */
	if(false == event_fifo_init(platform_init->event_mem.event_mem_pool, platform_init->event_mem.event_mem_pool_size))
	{
		return BLE_FAILURE;
	}
	
	if(NULL == platform_init ||
		NULL == platform_init->platform_api_list.create_timer ||
		NULL == platform_init->platform_api_list.delete_timer ||
		NULL == platform_init->platform_api_list.start_timer ||
		NULL == platform_init->platform_api_list.stop_timer ||
		NULL == platform_init->platform_api_list.sleep_timer_ms ||
		NULL == platform_init->platform_api_list.gpio_set ||
		NULL == platform_init->platform_api_list.mode_set ||
		NULL == platform_init->platform_api_list.uart_rx_cb ||
		NULL == platform_init->platform_api_list.uart_tx_cb)
		{
			return BLE_INVALID_COMMAND_PARAMETERS;
		}
		
		platform = &platform_init->platform_api_list;
		
		/* Create platform timer */
		platform_timer_handle = platform_init->platform_api_list.create_timer(interface_timer_callback);
        if (NULL == platform_timer_handle)
        {
	        return BLE_FAILURE;
        }
		
		/* Creating timers used in interface layer */
		interface_uart_timer_handle = interface_create_timer(interface_uart_timer_cb);
        if (NULL == interface_uart_timer_handle)
        {
	        return BLE_FAILURE;
        }
		
		interface_event_timer_handle = interface_create_timer(interface_event_timer_cb);
        if (NULL == interface_event_timer_handle)
        {
	        return BLE_FAILURE;
        }
		
		/* Start reading UART data */
		platform_init->platform_api_list.uart_rx_cb(&store_uart_data);
		
		return BLE_SUCCESS;
}

/*
 * @brief Initialize the UART receive FIFO
 *
 * @param
 *
 * @return 
*/
void init_fifo(void)
{
	fifo_init(&rx_fifo, rx_fifo_buffer, sizeof(rx_fifo_buffer));
}

/*
 * @brief Send BM7x command through UART and not wait for response event
 *
 * @param[in] cmd BM7x command
 * @param[in] cmd_len BM7x command length
 *
 * @return ble_status_t
*/
ble_status_t interface_cmd_send_no_wait(uint8_t *cmd, uint32_t cmd_len)
{
	platform->uart_tx_cb(cmd, cmd_len);
	return BLE_SUCCESS;
}

/*
 * @brief Send BM7x command through UART and wait for response event
 *
 * @param[in] cmd BM7x command
 * @param[in] cmd_len BM7x command length
 * @param[in] event_id response event from BM7x for the command is being send
 *
 * @return ble_status_t BLE_SUCCESS, if received an expected event before timeout; BLE_UART_TIMEOUT, if timeout occurs
*/
ble_status_t interface_cmd_send_wait(uint8_t *cmd, uint32_t cmd_len, uint8_t event_id)
{
	resp_event_id = event_id;
	platform->uart_tx_cb(cmd, cmd_len);
	interface_start_timer(interface_uart_timer_handle, INTERFACE_CMD_COMPLETE_TIMEOUT);
	while(!interface_uart_timer_timeout && !interface_event_process());
	
	if(interface_uart_timer_timeout)
	{
		interface_uart_timer_timeout = false;
		return BLE_UART_TIMEOUT;
	}
	else
	{
		interface_stop_timer(interface_uart_timer_handle);
		return BLE_SUCCESS;
	}
}

/*
 * @brief Waiting (timeout) for a response event to a command
 *
 * @param[in] cmd BM7x command
 * @param[in] cmd_len BM7x command length
 * @param[in] event_id response event from BM7x for the command is being send
 * @param[in] timeout timeout to get a response
 *
 * @return ble_status_t BLE_SUCCESS, if received an event; BLE_UART_TIMEOUT, if timeout happens
*/
ble_status_t interface_cmd_send_wait_time(uint8_t *cmd, uint32_t cmd_len, uint8_t event_id, uint16_t timeout)
{
	resp_event_id = event_id;
	platform->uart_tx_cb(cmd, cmd_len);
	interface_start_timer(interface_uart_timer_handle, timeout);
	while(!interface_uart_timer_timeout && !interface_event_process());
	
	if(interface_uart_timer_timeout)
	{
		interface_uart_timer_timeout = false;
		return BLE_UART_TIMEOUT;
	}
	else
	{
		interface_stop_timer(interface_uart_timer_handle);
	}
	
	return BLE_SUCCESS;
}

/*
 * @brief Waiting for a response event to a command
 *
 * @param[in] event_id expected event ID
 *
 * @return ble_status_t BLE_SUCCESS, if received an event; BLE_UART_TIMEOUT, if timeout happens
*/
ble_status_t interface_event_wait(uint8_t event_id)
{
	resp_event_id = event_id;
	interface_start_timer(interface_uart_timer_handle, INTERFACE_CMD_COMPLETE_TIMEOUT);
	while(!interface_uart_timer_timeout && !interface_event_process());
	
	if(interface_uart_timer_timeout)
	{
		interface_uart_timer_timeout = false;
		return BLE_UART_TIMEOUT;
	}
	else
	{
		interface_stop_timer(interface_uart_timer_handle);
		return BLE_SUCCESS;
	}
}

/*
 * @brief Waiting (timeout) for a specific event
 *
 * @param[in] event_id Event ID
 * @param[in] timeout Wait timeout
 *
 * @return ble_status_t BLE_SUCCESS, if received an event; BLE_UART_TIMEOUT, if timeout happens
*/
ble_status_t interface_event_wait_time(uint8_t event_id, uint16_t timeout)
{
	resp_event_id = event_id;
	interface_start_timer(interface_uart_timer_handle, timeout);
	while(!interface_uart_timer_timeout && !interface_event_process());
	
	if(interface_uart_timer_timeout)
	{
		interface_uart_timer_timeout = false;
		return BLE_UART_TIMEOUT;
	}
	else
	{
		interface_stop_timer(interface_uart_timer_handle);
	}
	
	return BLE_SUCCESS;
}

/* [callback_funcs] */
void store_uart_data(uint16_t rx_data)
{
	uint8_t data = (uint8_t) rx_data;
	
	if (!fifo_put(&rx_fifo, data))
	{
		/* DBG_LOG("trying to put data in full fifo, wr_idx = %d, rd_idx = %d\r\n", (int)rx_fifo.wr_idx, (int)rx_fifo.rd_idx); */
	}
	/* We will stop receiving from the HW,
	 when fifo_full return zero we can start receiving from HW again */
	if (fifo_full(&rx_fifo))
	{
		/* stop receiver */
		is_fifo_full = true;
	}
	else
	{
		/* read the next byte */
		platform->uart_rx_cb(&store_uart_data);
        
	}
}

/*
 * @brief Check a received response event 
 *
 * @param[in] byte data fetched from UART FIFO
 *
 * @return bool true, if received a valid event completely; false, if CRC fails or partial event received
*/
bool ble_response_check(uint8_t byte)
{
	return (BM_APPLICATION_ResponseCheck(&byte, 1));
}

/*
 * @brief Reads fifo where data/commands from BMxx stored. It also parse commands and check CRC
 *
 * @param 
 *
 * @return
*/
uint8_t interface_process_fifo_data(void)
{
	uint8_t byte;
	bool result;
	
	while (!fifo_empty(&rx_fifo))
	{
		/* Initialize the mode variable to avoid warning */
		BM_MODE mode = BM_MODE_PROGRAM;
		
		fifo_get(&rx_fifo, &byte);
		
		if(true == is_fifo_full)
		{
			is_fifo_full = false;
			platform->uart_rx_cb(&store_uart_data);
		}
		
        platform->mode_get(&mode);
        if (mode == BM_MODE_APPLICATION)
            result = ble_response_check(byte);
        else if (mode == BM_MODE_PROGRAM)
            result = dfu_response_check(byte);

		if (result)
		{
			return INTERFACE_EVENT_READY;
		}
	}
	
	return INTERFACE_EVENT_WAITING;
}

/*
 * @brief Check whether the expected response has been received
 *
 * @param[in] event_id BM7x event ID
 * @return true, if cmd ID matched. false, otherwise
*/
bool is_waiting_for_this_response(uint8_t event_id)
{
	if((event_id == resp_event_id) && (INVALID_EVENT_ID != resp_event_id))
	{
		return true;
	}
	
	return false;
}

/*
 * @brief Check whether expecting a response
 *
 * @param[] 
 * @return true, if expecting a response. false, otherwise
*/
bool is_waiting_for_response(void)
{
	if(INVALID_EVENT_ID != resp_event_id)
	{
		return true;
	}
	
	return false;
}

/*
 * @brief Process events from BM7x
 *
 * @param
 *
 * @return true, if desired event is received otherwise false
*/
bool interface_event_process(void)
{
	uint8_t is_event_ready = interface_process_fifo_data();
	bool is_event_received = false;
	
	if(is_waiting_for_response())
	{
		/* Waiting for response from BM7x */
		do 
		{
			if(INTERFACE_EVENT_READY == is_event_ready)
			{
				if(is_waiting_for_this_response(appEvent.event_id))
				{
					/* Received the specific event? then return status to application */
					is_event_received = true;
				}
				else
				{
					/* If this is not the expected event then post event in event Q and look for specific event */
					is_event_received = false;
					appEvent.event_msg.data_len--; /* Subtract the length of event_id */
					/* Since this is not a specific event we are looking for, 
						not checking the status of event post */
					event_fifo_write(&appEvent);
					is_event_ready = interface_process_fifo_data();
				}
			}
			else
			{
				is_event_ready = interface_process_fifo_data();
			}
		} while ((false == is_event_received) && (!interface_uart_timer_timeout)); /* Is expected event received? or timeout happened*/
	}

	return is_event_received;
}

/*
 * @brief Process BM7x command, if any, and post it in Event Q
 *
 * @param
 *
 * @return Event Q status
*/
event_status_t interface_event_get(void)
{
	event_status_t status = BLE_EVENT_Q_EMPTY;
	uint8_t is_event_ready = interface_process_fifo_data();
	
	if(INTERFACE_EVENT_READY == is_event_ready)
	{
		/* Post response in event Q and process event */
		appEvent.event_msg.data_len--; /* Subtract the length of event_id */
		status = event_fifo_write(&appEvent);
	}
	
	return status;
}

/*
 * @brief Provides pointer to received command frame
 *
 * @param
 *
 * @return pointer to event_t
*/
event_t* get_received_cmd_frame(void)
{
	return &appEvent;
}
