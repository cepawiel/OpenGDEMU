#pragma once

/**
 * 
 * \file
 *
 * \brief Universal Asynchronous Receiver Transceiver (UART) driver for SAM.
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

#include <stdint.h>

#include <sam3u4e.h>

namespace ATSAM3U {
	class SAM_UART {
		protected:
			/* UART internal div factor for sampling */
			const uint32_t UART_MCK_DIV = 16;
			/* Div factor to get the maximum baud rate */
			const uint32_t UART_MCK_DIV_MIN_FACTOR = 1;
			/* Div factor to get the minimum baud rate */
			const uint32_t UART_MCK_DIV_MAX_FACTOR = 65535;
				
			Uart *p_uart;

		public:
			SAM_UART(Uart *p_uart);

			bool Init(uint32_t baudrate, uint32_t mode, uint32_t mck);

			void EnableTX();
			void DisableTX();
			void ResetTX();
			void EnableRX();
			void DisableRX();
			void ResetRX();
			void Enable();
			void Disable();
			void Reset();

			// void uart_enable_interrupt(Uart *p_uart, uint32_t ul_sources);
			// void uart_disable_interrupt(Uart *p_uart, uint32_t ul_sources);
			// uint32_t uart_get_interrupt_mask(Uart *p_uart);
			// uint32_t uart_get_status(Uart *p_uart);
			// void uart_reset_status(Uart *p_uart);
			bool IsTXReady();
			bool IsTXEmpty();
			// uint32_t uart_is_rx_ready(Uart *p_uart);
			// uint32_t uart_is_tx_buf_empty(Uart *p_uart);
			// void uart_set_clock_divisor(Uart *p_uart, uint16_t us_divisor);
			bool Write(const uint8_t uc_data);
			// uint32_t uart_read(Uart *p_uart, uint8_t *puc_data);
	};
}


// void uart_enable_interrupt(Uart *p_uart, uint32_t ul_sources);
// void uart_disable_interrupt(Uart *p_uart, uint32_t ul_sources);
// uint32_t uart_get_interrupt_mask(Uart *p_uart);
// uint32_t uart_get_status(Uart *p_uart);
// void uart_reset_status(Uart *p_uart);
// uint32_t uart_is_tx_ready(Uart *p_uart);
// uint32_t uart_is_tx_empty(Uart *p_uart);
// uint32_t uart_is_rx_ready(Uart *p_uart);
// uint32_t uart_is_tx_buf_empty(Uart *p_uart);
// void uart_set_clock_divisor(Uart *p_uart, uint16_t us_divisor);
// uint32_t uart_write(Uart *p_uart, const uint8_t uc_data);
// uint32_t uart_read(Uart *p_uart, uint8_t *puc_data);
// #if (!SAMV71 && !SAMV70 && !SAME70 && !SAMS70)
// uint32_t uart_is_rx_buf_end(Uart *p_uart);
// uint32_t uart_is_tx_buf_end(Uart *p_uart);
// uint32_t uart_is_rx_buf_full(Uart *p_uart);
// Pdc *uart_get_pdc_base(Uart *p_uart);
// #endif


