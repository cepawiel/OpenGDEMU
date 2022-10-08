/**
 * \file
 *
 * \brief UART functions
 *
 * Copyright (c) 2011-2018 Microchip Technology Inc. and its subsidiaries.
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
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>
#include "uart.h"
#include "main.h"
#include "ui.h"

static usart_options_t usart0_options;
static usart_options_t usart_options;

ISR(usart0_interrupt, USART0_IRQ_GROUP, 3)
{
	if (USART0->csr & AVR32_USART_CSR_RXRDY_MASK) {
		// Data received
		ui_com_tx_start();
		int value;
		bool b_error = (USART_SUCCESS != usart_read_char(USART0, &value));
		if (b_error) {
			usart_reset_status(USART0);
			udi_cdc_multi_signal_framing_error(0);
			ui_com_error();
		}
		// Transfer UART RX fifo to CDC TX
		if (!udi_cdc_multi_is_tx_ready(0)) {
			// Fifo full
			udi_cdc_multi_signal_overrun(0);
			ui_com_overflow();
		}else{
			udi_cdc_multi_putc(0,value);
		}
		ui_com_tx_stop();
		return;
	}

	if (USART0->csr & AVR32_USART_CSR_TXRDY_MASK) {
		// Data send
		if (udi_cdc_multi_is_rx_ready(0)) {
			// Transmit next data
			ui_com_rx_start();
			usart_write_char(USART0, udi_cdc_multi_getc(0));
		} else {
			// Fifo empty then Stop UART transmission
			USART0->idr = AVR32_USART_IER_TXRDY_MASK;
			ui_com_rx_stop();
		}
	}
}

ISR(usart_interrupt, USART_IRQ_GROUP, 3)
{
	if (USART->csr & AVR32_USART_CSR_RXRDY_MASK) {
		// Data received
		ui_com_tx_start();
		int value;
		bool b_error = (USART_SUCCESS != usart_read_char(USART, &value));
		if (b_error) {
			usart_reset_status(USART);
			udi_cdc_multi_signal_framing_error(1);
			ui_com_error();
		}
		// Transfer UART RX fifo to CDC TX
		if (!udi_cdc_multi_is_tx_ready(1)) {
			// Fifo full
			udi_cdc_multi_signal_overrun(1);
			ui_com_overflow();
		}else{
			udi_cdc_multi_putc(1,value);
		}
		ui_com_tx_stop();
		return;
	}

	if (USART->csr & AVR32_USART_CSR_TXRDY_MASK) {
		// Data send
		if (udi_cdc_multi_is_rx_ready(1)) {
			// Transmit next data
			ui_com_rx_start();
			usart_write_char(USART, udi_cdc_multi_getc(1));
		} else {
			// Fifo empty then Stop UART transmission
			USART->idr = AVR32_USART_IER_TXRDY_MASK;
			ui_com_rx_stop();
		}
	}
}

void uart_rx_notify(uint8_t port)
{
	if (port==0) {
		// If UART is open
		if (USART0->imr & AVR32_USART_IER_RXRDY_MASK) {
			// Enable UART TX interrupt to send a new value
			USART0->ier = AVR32_USART_IER_TXRDY_MASK;
		}
	}else{
		// If UART is open
		if (USART->imr & AVR32_USART_IER_RXRDY_MASK) {
			// Enable UART TX interrupt to send a new value
			USART->ier = AVR32_USART_IER_TXRDY_MASK;
		}
	}
}


void uart_config(uint8_t port, usb_cdc_line_coding_t * cfg)
{
	uint32_t stopbits, parity;
	uint32_t imr;

	switch (cfg->bCharFormat) {
	case CDC_STOP_BITS_2:
		stopbits = USART_2_STOPBITS;
		break;
	case CDC_STOP_BITS_1_5:
		stopbits = USART_1_5_STOPBITS;
		break;
	case CDC_STOP_BITS_1:
	default:
		// Default stop bit = 1 stop bit
		stopbits = USART_1_STOPBIT;
		break;
	}

	switch (cfg->bParityType) {
	case CDC_PAR_EVEN:
		parity = USART_EVEN_PARITY;
		break;
	case CDC_PAR_ODD:
		parity = USART_ODD_PARITY;
		break;
	case CDC_PAR_MARK:
		parity = USART_MARK_PARITY;
		break;
	case CDC_PAR_SPACE:
		parity = USART_SPACE_PARITY;
		break;
	default:
	case CDC_PAR_NONE:
		parity = USART_NO_PARITY;
		break;
	}

	// Options for USART
	if (port==0) {
		usart0_options.baudrate = LE32_TO_CPU(cfg->dwDTERate);
		usart0_options.charlength = cfg->bDataBits;
		usart0_options.paritytype = parity;
		usart0_options.stopbits = stopbits;
		usart0_options.channelmode = USART_NORMAL_CHMODE;
		imr = USART0->imr ;
		USART0->idr = 0xFFFFFFFF;
		usart_init_rs232(USART0, &usart0_options, sysclk_get_pba_hz());
		// Restore both RX and TX
		USART0->ier = imr;
	}else{
		usart_options.baudrate = LE32_TO_CPU(cfg->dwDTERate);
		usart_options.charlength = cfg->bDataBits;
		usart_options.paritytype = parity;
		usart_options.stopbits = stopbits;
		usart_options.channelmode = USART_NORMAL_CHMODE;
		imr = USART->imr ;
		USART->idr = 0xFFFFFFFF;
		usart_init_rs232(USART, &usart_options, sysclk_get_pba_hz());
		// Restore both RX and TX
		USART->ier = imr;
	}
}

void uart_open(uint8_t port)
{
	if (port==0) {
		// Enable interrupt with priority higher than USB
		irq_register_handler(usart0_interrupt, USART0_IRQ, 3);

		// Initialize it in RS232 mode.
		sysclk_enable_pba_module(USART_SYSCLK);
		if (USART_SUCCESS != usart_init_rs232(USART0, &usart0_options,
						sysclk_get_pba_hz())) {
			return;
		}
		// Enable both RX and TX
		USART0->ier = AVR32_USART_IER_TXRDY_MASK | AVR32_USART_IER_RXRDY_MASK;
	}else{
		// Enable interrupt with priority higher than USB
		irq_register_handler(usart_interrupt, USART_IRQ, 3);

		// Initialize it in RS232 mode.
		sysclk_enable_pba_module(USART_SYSCLK);
		if (USART_SUCCESS != usart_init_rs232(USART, &usart_options,
						sysclk_get_pba_hz())) {
			return;
		}
		// Enable both RX and TX
		USART->ier = AVR32_USART_IER_TXRDY_MASK | AVR32_USART_IER_RXRDY_MASK;
	}
}

void uart_close(uint8_t port)
{
	// Disable interrupts
	// Close RS232 communication
	if (port==0) {
		USART0->idr = 0xFFFFFFFF;
	}else{
		USART->idr = 0xFFFFFFFF;
	}
}
