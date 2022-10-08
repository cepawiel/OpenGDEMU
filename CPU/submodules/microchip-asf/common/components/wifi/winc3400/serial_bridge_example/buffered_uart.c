/**
 *
 * \file
 *
 * \brief SAM UART API Implementations.
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
 *
 */

#include "asf.h"
#include <stdio.h>
#include <string.h>
#include "buffered_uart.h"
#include "conf_uart_serial.h"

static uint8_t serial_rx_buf[SERIAL_RX_BUF_SIZE_HOST];

static uint8_t serial_rx_buf_head;

static uint8_t serial_rx_buf_tail;

static usart_serial_options_t usart_serial_options = {
	.baudrate     = USART_HOST_BAUDRATE,
	.charlength   = USART_HOST_CHAR_LENGTH,
	.paritytype   = USART_HOST_PARITY,
	.stopbits     = USART_HOST_STOP_BITS
};

void buffered_uart_init(uint32_t baudrate)
{
	serial_rx_buf_head = 0;
	serial_rx_buf_tail = 0;

	/* Configure the UART console. */
	usart_serial_options.baudrate = baudrate;
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(USART_HOST, &usart_serial_options);
	USART_HOST_RX_ISR_ENABLE();
}

void buffered_uart_term(void)
{
	USART_HOST_RX_ISR_DISABLE();
}

void buffered_uart_putchar(uint8_t ch)
{
	#if SAMG55
	usart_serial_putchar(USART_HOST, ch);
	while(!usart_is_tx_buf_empty(USART_HOST));
	#else
	usart_serial_putchar(USART_HOST, ch);
	while(!uart_is_tx_buf_empty(USART_HOST));
	#endif
}

uint8_t buffered_uart_tx(uint8_t *data, uint32_t length)
{
	status_code_t status;

	do{
		status = usart_serial_write_packet(USART_HOST,(const uint8_t *)data,length);
	} while (status != STATUS_OK);
	return length;
}

uint8_t buffered_uart_rx(uint8_t *data, uint8_t max_len)
{
	uint8_t copy_len = 0;
	uint8_t data_len = 0;

	if (serial_rx_buf_tail != serial_rx_buf_head) {
		cpu_irq_disable();

		if (serial_rx_buf_tail >= serial_rx_buf_head) {
			copy_len = serial_rx_buf_tail - serial_rx_buf_head;
		} else {
			copy_len = serial_rx_buf_tail + (SERIAL_RX_BUF_SIZE_HOST - serial_rx_buf_head);
		}

		if (copy_len > max_len) {
			copy_len = max_len;
		}

		if (copy_len) {
			if (SERIAL_RX_BUF_SIZE_HOST < (copy_len + serial_rx_buf_head)) {
				data_len = SERIAL_RX_BUF_SIZE_HOST - serial_rx_buf_head;
				memcpy(data, &serial_rx_buf[serial_rx_buf_head], data_len);
				data += data_len;
				copy_len -= data_len;
				serial_rx_buf_head = 0;
			}
			memcpy(data, &serial_rx_buf[serial_rx_buf_head], copy_len);
			data_len += copy_len;
			serial_rx_buf_head += copy_len;
		}
		cpu_irq_enable();
	}
	return data_len;
}

int buffered_uart_getchar_nowait(void)
{
	int c = -1;
	cpu_irq_disable();
	if (serial_rx_buf_tail != serial_rx_buf_head) {
		c = serial_rx_buf[serial_rx_buf_head];
		serial_rx_buf_head++;
		if (serial_rx_buf_head == SERIAL_RX_BUF_SIZE_HOST) {
			serial_rx_buf_head = 0;
		}
	}
	cpu_irq_enable();
	return c;
}

uint8_t buffered_uart_getchar(void)
{
	int c;
	do {
		c = buffered_uart_getchar_nowait();
	} while (c < 0);
	return (uint8_t)c;
}


USART_HOST_ISR_VECT()
{
	while (usart_serial_is_rx_ready(USART_HOST)) {
		usart_serial_getchar(USART_HOST, &serial_rx_buf[serial_rx_buf_tail]);
		if ((SERIAL_RX_BUF_SIZE_HOST - 1) == serial_rx_buf_tail) {
			serial_rx_buf_tail = 0x00;
		} else {
			serial_rx_buf_tail++;
		}
	}
}
