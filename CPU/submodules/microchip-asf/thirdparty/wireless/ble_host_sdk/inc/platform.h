/**
 * \file platform.h
 *
 * \brief platform interface declarations
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

#ifndef __PLATFORM_H__
#define __PLATFORM_H__

/*
 * @brief Initialize platform layer
 *
 * @param
 *
 * @return 
*/
void platform_init(void);

/*
 * @brief Configure UART interface with BM7x
 *
 * @param 
 *
 * @return 
*/
void configure_usart(void);

/*
 * @brief Configure UART callback
 *
 * @param 
 *
 * @return 
*/
void configure_usart_callbacks(void);

/*
 * @brief UART receive interrupt callback function
 *
 * @param[in] usart_module UART module used to interface with BM7x
 *
 * @return 
*/
void usart_read_callback(struct usart_module *const usart_module);

/*
 * @brief UART write synchronously
 *
 * @param[in] data UART TX data
 * @param[in] length UART TX data length
 *
 * @return 
*/
void uart_write_sync(uint8_t *data, uint16_t length);

/*
 * @brief Read UART data
 *
 * @param[in] data UART RX data
 *
 * @return 
*/
void uart_read_data(uint16_t *data);

/*
 * @brief Start reading UART data asynchronously
 *
 * @param[in] data UART RX data
 *
 * @return 
*/
void read_data_async(void);

/*
 * @brief Send user data to remote device
 *
 * @param
 *
 * @return 
*/
void send_user_data(void);

void read_async_callback(uart_recv_async_cb_t recv_async_callback);

void *platform_create_timer(void (*timer_cb)(void *));
void platform_delete_timer(void *timer_handle);
void platform_start_timer(void *timer_handle, uint32_t ms);
void platform_stop_timer(void *timer_handle);
void platform_sleep(uint32_t ms);
void platform_gpio_set(gpio_pin_t pin, gpio_status_t status);
void platform_mode_set(BM_MODE mode);
void platform_mode_get(BM_MODE *mode);
void platform_send_sync(uint8_t *data, uint32_t length);
void platform_recv_async(uart_recv_async_cb_t recv_async_callback);
void platform_ble_mode_select(uint8_t mode);
bool platform_wakeup_pin_status(void);
void platform_process_rxdata(uint8_t t_rx_data);

#endif //__PLATFORM_H__
