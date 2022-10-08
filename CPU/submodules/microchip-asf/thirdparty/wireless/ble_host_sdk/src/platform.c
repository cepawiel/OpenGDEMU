/**
 * \file platform.c
 *
 * \brief platform interface definitions
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
#include "event_mem.h"
#include "bm_application_mode.h"
#include "ble_api.h"
#include "platform.h"

static uart_recv_async_cb_t recv_async_cb = NULL;

/* UART interface to BM7x */
static struct usart_module usart_instance;
/* UART receive asynchronous callback */
static uart_recv_async_cb_t uart_read_async_cb = NULL;
/* UART receive data */
static uint16_t rx_data = 0;
/* Is disconnection in progress */
extern bool disconnect_in_progress;


/*
 * @brief Initialize platform layer
 *
 * @param
 *
 * @return 
*/
void platform_init(void)
{
	/* Configure UART interface for BLE module */
	configure_serial_drv(115200);
	/* Configure REST, MODE and RX_IND */
	ble_configure_control_pin();
	/* Configure TX_IND */
	bm_host_wakeup_config();
	/* Enable global interrupt */
	system_interrupt_enable_global();
}

/*
 * @brief UART send synchronously
 *
 * @param[in] data UART TX data
 * @param[in] length UART TX data length
 *
 * @return 
*/
void platform_send_sync(uint8_t *data, uint32_t length)
{
	bm7x_wakeup_pin_set_low();
	delay_ms(5);
	serial_drv_send(data, length);
	bm7x_wakeup_pin_set_high();
}

void platform_recv_async(uart_recv_async_cb_t recv_async_callback)
{
	recv_async_cb = recv_async_callback;
	platform_start_rx();
}

void platform_process_rxdata(uint8_t t_rx_data)
{
	Assert((recv_async_cb != NULL));
	recv_async_cb(t_rx_data);
}

/*
 * @brief Start reading UART data asynchronously
 *
 * @param[in] data UART RX data
 *
 * @return 
*/
void read_data_async(void)
{
	uart_read_data(&rx_data);
}

/*
 * @brief Read UART data
 *
 * @param[in] data UART RX data
 *
 * @return 
*/
void uart_read_data(uint16_t *data)
{
	usart_read_job(&usart_instance, data);
}


void read_async_callback(uart_recv_async_cb_t recv_async_callback)
{
	uart_read_async_cb = recv_async_callback;
	read_data_async();
}

void platform_gpio_set(gpio_pin_t pin, gpio_status_t status)
{
	switch(pin)
	{
		case BM7X_PIN_RESET:
		{
			if(GPIO_HIGH == status)
			{
				bm7x_reset_pin_set_high();
			}
			else
			{
				bm7x_reset_pin_set_low();
			}
			break;
		}
		case BM7X_PIN_MODE:
		{
			if(GPIO_HIGH == status)
			{
				bm7x_mode_pin_set_high();
			}
			else
			{
				bm7x_mode_pin_set_low();
			}
			break;
		}
		case BM7X_PIN_RX_IND:
		{
			if(GPIO_HIGH == status)
			{
				bm7x_wakeup_pin_set_high();
			}
			else
			{
				bm7x_wakeup_pin_set_low();
			}
			break;
		}
		default:
		return;
	}
}

void platform_host_wake_interrupt_handler(void)
{
	/* Keep BM7x Wakeup line high */
}

void *platform_create_timer(void (*timer_cb)(void *))
{
	return (platform_create_hw_timer(timer_cb));
}

void platform_delete_timer(void *timer_handle)
{
	platform_delete_bus_timer(timer_handle);
}

void platform_start_timer(void *timer_handle, uint32_t ms)
{
	platform_enter_critical_section();
	platform_start_bus_timer(timer_handle, ms);
	platform_leave_critical_section();
}

void platform_stop_timer(void *timer_handle)
{
	platform_enter_critical_section();
	platform_stop_bus_timer(timer_handle);
	platform_leave_critical_section();
}

void platform_sleep(uint32_t ms)
{
	delay_ms(ms);
}

void platform_mode_set(BM_MODE mode)
{
	BM_MODE_Set(mode);
}

void platform_mode_get(BM_MODE *mode)
{
    BM_MODE_Get(*mode);
}

bool platform_wakeup_pin_status(void)
{
	return (bm7x_wakeup_pin_level());
}

void platform_ble_mode_select(uint8_t mode)
{
	/* Initialize mode pin */
	BM_MODE_Init();
	/* Set Application mode */
	BM_MODE_Set(mode);
}

