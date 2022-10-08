/**
 * \file sio2ncp.c
 *
 * \brief Handles Serial I/O  Functionalities
 *
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
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

/* === INCLUDES ============================================================ */

#include "asf.h"
#include "sio2ncp.h"
#include "conf_sio2ncp.h"
#if SAMD || SAMR21
#include "stdio_serial.h"
#endif
/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === PROTOTYPES ========================================================== */

/* === GLOBALS ========================================================== */
#if SAMD || SAMR21
static struct usart_module uart_module;
#else
static usart_serial_options_t usart_serial_options = {
	.baudrate     = USART_NCP_BAUDRATE,
	.charlength   = USART_NCP_CHAR_LENGTH,
	.paritytype   = USART_NCP_PARITY,
	.stopbits     = USART_NCP_STOP_BITS
};
#endif

/**
 * Receive buffer
 * The buffer size is defined in sio2ncp.h
 */
static uint8_t serial_rx_buf[SERIAL_RX_BUF_SIZE_NCP];

/**
 * Receive buffer head
 */
static uint8_t serial_rx_buf_head;

/**
 * Receive buffer tail
 */
static uint8_t serial_rx_buf_tail;

/**
 * Number of bytes in receive buffer
 */
static uint8_t serial_rx_count;

/* === IMPLEMENTATION ====================================================== */

void sio2ncp_init(void)
{
#if SAMD || SAMR21
	struct usart_config ncp_uart_config;
	/* Configure USART for unit test output */
	usart_get_config_defaults(&ncp_uart_config);
	ncp_uart_config.mux_setting = NCP_SERCOM_MUX_SETTING;

	ncp_uart_config.pinmux_pad0 = NCP_SERCOM_PINMUX_PAD0;
	ncp_uart_config.pinmux_pad1 = NCP_SERCOM_PINMUX_PAD1;
	ncp_uart_config.pinmux_pad2 = NCP_SERCOM_PINMUX_PAD2;
	ncp_uart_config.pinmux_pad3 = NCP_SERCOM_PINMUX_PAD3;
	ncp_uart_config.baudrate    = USART_NCP_BAUDRATE;
	stdio_serial_init(&uart_module, USART_NCP, &ncp_uart_config);
	usart_enable(&uart_module);
	/* Enable transceivers */
	usart_enable_transceiver(&uart_module, USART_TRANSCEIVER_TX);
	usart_enable_transceiver(&uart_module, USART_TRANSCEIVER_RX);
#else
	usart_serial_init(USART_NCP, &usart_serial_options);
#endif

#if (BOARD == SAM4L_XPLAINED_PRO)
#if defined (NCP_RESET_GPIO)
	ioport_set_pin_dir(NCP_RESET_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(NCP_RESET_GPIO, IOPORT_PIN_LEVEL_HIGH);
#endif
#endif /* (BOARD == SAM4L_XPLAINED_PRO) */
	USART_NCP_RX_ISR_ENABLE();
}

uint8_t sio2ncp_tx(uint8_t *data, uint8_t length)
{
#if SAMD || SAMR21
	status_code_genare_t status;
#else
	status_code_t status;
#endif /* SAMD || SAMR21 */

	do {
#if SAMD || SAMR21
		status
			= usart_serial_write_packet(&uart_module,
				(const uint8_t *)data,
				length);
#else
		status = usart_serial_write_packet(USART_NCP,
				(const uint8_t *)data,
				length);
#endif
	} while (status != STATUS_OK);
	return length;
}

uint8_t sio2ncp_rx(uint8_t *data, uint8_t max_length)
{
	uint8_t data_received = 0;
	if(serial_rx_buf_tail >= serial_rx_buf_head)
	{
		serial_rx_count = serial_rx_buf_tail - serial_rx_buf_head;
	}
	else
	{
		serial_rx_count = serial_rx_buf_tail + (SERIAL_RX_BUF_SIZE_NCP - serial_rx_buf_head);
	}
	
	if (0 == serial_rx_count) {
		return 0;
	}

	if (SERIAL_RX_BUF_SIZE_NCP <= serial_rx_count) {
		/*
		 * Bytes between head and tail are overwritten by new data.
		 * The oldest data in buffer is the one to which the tail is
		 * pointing. So reading operation should start from the tail.
		 */
		serial_rx_buf_head = serial_rx_buf_tail;

		/*
		 * This is a buffer overflow case. But still only the number of
		 * bytes equivalent to
		 * full buffer size are useful.
		 */
		serial_rx_count = SERIAL_RX_BUF_SIZE_NCP;

		/* Bytes received is more than or equal to buffer. */
		if (SERIAL_RX_BUF_SIZE_NCP <= max_length) {
			/*
			 * Requested receive length (max_length) is more than
			 * the
			 * max size of receive buffer, but at max the full
			 * buffer can be read.
			 */
			max_length = SERIAL_RX_BUF_SIZE_NCP;
		}
	} else {
		/* Bytes received is less than receive buffer maximum length. */
		if (max_length > serial_rx_count) {
			/*
			 * Requested receive length (max_length) is more than
			 * the data
			 * present in receive buffer. Hence only the number of
			 * bytes
			 * present in receive buffer are read.
			 */
			max_length = serial_rx_count;
		}
	}

	data_received = max_length;
	while (max_length > 0) {
		/* Start to copy from head. */
		*data = serial_rx_buf[serial_rx_buf_head];
		data++;
		max_length--;
		if ((SERIAL_RX_BUF_SIZE_NCP - 1) == serial_rx_buf_head) {
			serial_rx_buf_head = 0;
		}
		else
		{
			serial_rx_buf_head++;
		}
	}
	return data_received;
}

uint8_t sio2ncp_getchar(void)
{
	uint8_t c;
	while (0 == sio2ncp_rx(&c, 1)) {
	}
	return c;
}

int sio2ncp_getchar_nowait(void)
{
	uint8_t c;
	int back = sio2ncp_rx(&c, 1);
	if (back >= 1) {
		return c;
	} else {
		return (-1);
	}
}

#if SAMD || SAMR21
void USART_NCP_ISR_VECT(uint8_t instance)
#else
USART_NCP_ISR_VECT()
#endif
{
	uint8_t temp;
#if SAMD || SAMR21
	usart_serial_read_packet(&uart_module, &temp, 1);
#else
	usart_serial_read_packet(USART_NCP, &temp, 1);
#endif

	/* Introducing critical section to avoid buffer corruption. */
	cpu_irq_disable();

	/* The number of data in the receive buffer is incremented and the
	 * buffer is updated. */

	serial_rx_buf[serial_rx_buf_tail] = temp;

	if ((SERIAL_RX_BUF_SIZE_NCP - 1) == serial_rx_buf_tail) {
		/* Reached the end of buffer, revert back to beginning of
		 * buffer. */
		serial_rx_buf_tail = 0x00;
	} else {
		serial_rx_buf_tail++;
	}

	cpu_irq_enable();
}

/* EOF */
