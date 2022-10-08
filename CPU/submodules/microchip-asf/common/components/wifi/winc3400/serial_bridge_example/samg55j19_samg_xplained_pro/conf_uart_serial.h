/**
 * \file
 *
 * \brief SAMG55 USART Serial Configuration
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

#ifndef CONF_USART_SERIAL_H
#define CONF_USART_SERIAL_H

/** UART Interface */
#define USART_HOST				   CONSOLE_UART
/** Baudrate setting */
#define USART_HOST_BAUDRATE		   115200
/** Character length setting */
#define USART_HOST_CHAR_LENGTH	   US_MR_CHRL_8_BIT
/** Parity setting */
#define USART_HOST_PARITY		   US_MR_PAR_NO
/** Stop bits setting */
#define USART_HOST_STOP_BITS	   US_MR_NBSTOP_1_BIT
/** Configuration for console uart IRQ handler */
#define USART_HOST_ISR_VECT()      ISR(FLEXCOM7_Handler)
/** UART Host IRQ Number */
#define USART_HOST_IRQn            FLEXCOM7_IRQn

#define SERIAL_RX_BUF_SIZE_HOST    256

#define USART_HOST_RX_ISR_ENABLE() \
do {\
	usart_enable_interrupt(USART_HOST, US_IER_RXRDY); \
	NVIC_EnableIRQ(USART_HOST_IRQn);\
} while (0)

#define USART_HOST_RX_ISR_DISABLE() \
do {\
	usart_disable_interrupt(USART_HOST, US_IER_RXRDY); \
	NVIC_DisableIRQ(USART_HOST_IRQn);\
} while (0)

#endif/* CONF_USART_SERIAL_H_INCLUDED */
