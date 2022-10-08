/**
* \file  conf_sio2ncp.h
*
* \brief Serial Input & Output configuration
*		
*
* Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries. 
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
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, 
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

#ifndef CONF_SIO2NCP_H_INCLUDED
#define CONF_SIO2NCP_H_INCLUDED

#define USART_NCP                 EXT1_UART_MODULE

#define NCP_SERCOM_MUX_SETTING    EXT1_UART_SERCOM_MUX_SETTING
#define NCP_SERCOM_PINMUX_PAD0    EXT1_UART_SERCOM_PINMUX_PAD0
#define NCP_SERCOM_PINMUX_PAD1    EXT1_UART_SERCOM_PINMUX_PAD1
#define NCP_SERCOM_PINMUX_PAD2    EXT1_UART_SERCOM_PINMUX_PAD2
#define NCP_SERCOM_PINMUX_PAD3    EXT1_UART_SERCOM_PINMUX_PAD3
/** Baudrate setting */
#define USART_NCP_BAUDRATE        115200

#define USART_NCP_RX_ISR_ENABLE()  _sercom_set_handler(4, USART_NCP_ISR_VECT); \
	USART_NCP->USART.INTENSET.reg = SERCOM_USART_INTFLAG_RXC; \
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SERCOM4);
#endif /* CONF_SIO2NCP_H_INCLUDED */
