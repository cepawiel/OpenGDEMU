/**
 * \file interface.h
 *
 * \brief BM module Interface declaration
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

#ifndef __INTERFACE_H__
#define __INTERFACE_H__

#define INVALID_EVENT_ID	0xFF

#define INTERFACE_SW_TIMERS	5

#define INTERFACE_CMD_COMPLETE_TIMEOUT		5000


enum
{
	INTERFACE_EVENT_WAITING,
	INTERFACE_EVENT_READY,
};

typedef struct
{
	uint32_t timer_usage;
	uint32_t timer_counter;
	void (*timer_cb)(void);
}sw_timer_t;

typedef void (*timer_cb_t)(void);
/*
 * @brief Initialize the interface layer
 *
 * @param pointer to platform_init which contains platform API list and event memory
 *
 * @return ble_status_t status of interface initialization
*/
ble_status_t interface_init(platform_init_t *platform_init);

/*
 * @brief Initialize the UART receive FIFO
 *
 * @param
 *
 * @return 
*/
void init_fifo(void);

/*
 * @brief Register a callback for BMxx UART TX
 *
 * @param[in] uart_tx_cb UART TX callback
 *
 * @return 
*/
void ble_api_register_uart_interface(uart_write_sync_cb_t uart_tx_cb, uart_read_async_cb_t uart_rx_cb);

/*
 * @brief Stores UART data in FIFO
 *
 * @param[in] rx_data UART received data
 *
 * @return
*/
void store_uart_data(uint16_t rx_data);

/*
 * @brief Send BM7x command through UART
 *
 * @param[in] cmd BM7x command
 * @param[in] cmd_len BM7x command length
 *
 * @return ble_status_t
*/
ble_status_t interface_cmd_send_no_wait(uint8_t *cmd, uint32_t cmd_len);

/*
 * @brief Send BM7x command through UART
 *
 * @param[in] cmd BM7x command
 * @param[in] cmd_len BM7x command length
 * @param[in] event_id response event from BM7x for the command is being send
 *
 * @return ble_status_t BLE_SUCCESS, if received an expected event before timeout; BLE_UART_TIMEOUT, if timeout occurs
*/
ble_status_t interface_cmd_send_wait(uint8_t *cmd, uint32_t cmd_len, uint8_t event_id);

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
ble_status_t interface_cmd_send_wait_time(uint8_t *cmd, uint32_t cmd_len, uint8_t event_id, uint16_t timeout);

/*
 * @brief Waiting (timeout) for a specific event
 *
 * @param[in] event_id Event ID
 * @param[in] timeout Wait timeout
 *
 * @return ble_status_t BLE_SUCCESS, if received an event; BLE_UART_TIMEOUT, if timeout happens
*/
ble_status_t interface_event_wait_time(uint8_t event_id, uint16_t timeout);

/*
 * @brief Check whether the expected response has been received
 *
 * @param[in] event_id BM7x event ID
 * @return true, if cmd ID matched. false, otherwise
*/
bool is_waiting_for_this_response(uint8_t event_id);

/*
 * @brief Check whether expecting a response
 *
 * @param[] 
 * @return true, if expecting a response. false, otherwise
*/
bool is_waiting_for_response(void);

/*
 * @brief Process events from BM7x
 *
 * @param
 *
 * @return true, if desired event is received otherwise false
*/
bool interface_event_process(void);

/*
 * @brief Process BM7x command, if any, and post it in Event Q
 *
 * @param
 *
 * @return Event Q status
*/
event_status_t interface_event_get(void);

/*
 * @brief Provides pointer to received command frame
 *
 * @param
 *
 * @return pointer to event_t
*/
event_t* get_received_cmd_frame(void);

/*
 * @brief Waiting (timeout) for a response event to a command
 *
 * @param[in] event_id expected event ID
 *
 * @return ble_status_t BLE_SUCCESS, if received an event; BLE_UART_TIMEOUT, if timeout happens
*/
ble_status_t interface_event_wait(uint8_t event_id);

/*
 * @brief Check a received response event 
 *
 * @param[in] byte data fetched from UART FIFO
 *
 * @return bool true, if received a valid event completely; false, if CRC fails or partial event received
*/
bool ble_response_check(uint8_t byte);

#endif //__INTERFACE_H__