/**
* \file  uartSerializer.c
*
* \brief Implementation of uart serialize interface.
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
/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
#include <uartSerializer.h>
#include "asf.h"
#include "conf_sio2host.h"
#if (USE_USART0 == 1) || (USE_USART1 == 1)

/******************************************************************************
                   Define(s) section
******************************************************************************/
/**************************************************************************//**
 \brief Sets USART U2Xn bit value.
******************************************************************************/
#ifndef USART_DOUBLE_SPEED
  #define USART_DOUBLE_SPEED 1ul
#endif

// usart channel
typedef SercomUsart* UsartChannel_t;


/******************************************************************************
                   Global variables section
******************************************************************************/
UsartChannel_t usartTty;

/******************************************************************************
                   Prototypes section
******************************************************************************/
static inline void hwInit(void);
static inline void hwUnInit(void);
struct usart_module host_uart_module;

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Startup initialization of the usart.
******************************************************************************/
static inline void hwInit(void)
{
}

/**************************************************************************//**
\brief Clear startup initialization parameters to start user application
******************************************************************************/
static inline void hwUnInit(void)
{
}

/**************************************************************************//**
\brief Receive byte on uart.

\param[out]
  p - pointer to byte memory;

\return
    true - there is received byte; \n
    false - there is not received byte;
******************************************************************************/
bool getByteUsart(uint8_t *p)
{
	uint16_t temp = 0;

	if(STATUS_OK != usart_read_wait(&host_uart_module, &temp))
	{
		return false;
	}
	else
	{
		*p = temp;	
		return true;
	}
	
}

/**************************************************************************//**
\brief Transmit byte to uart.

\param[in]
  len - number of bytes to transmit;
\param[in]
  p - pointer to byte memory;
******************************************************************************/
void setByteUsart(uint16_t length, uint8_t *data)
{
	
	while (length) {
		while(STATUS_OK !=usart_write_wait(&host_uart_module, (uint16_t)*data));
		length--;
		data++;
	}
		
}

#if (USE_USART0 == 1)
/**************************************************************************//**
\brief Startup initialization of the usart0.
******************************************************************************/
void hwInitUsart0(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	
	config_usart.baudrate    = USART_HOST_BAUDRATE;
	config_usart.mux_setting = HOST_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = HOST_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = HOST_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = HOST_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = HOST_SERCOM_PINMUX_PAD3;	
	
	while (usart_init(&host_uart_module,
	USART_HOST, &config_usart) != STATUS_OK) {
	}
	
	usart_enable(&host_uart_module);
	
	usart_enable_transceiver(&host_uart_module, USART_TRANSCEIVER_TX);
	usart_enable_transceiver(&host_uart_module, USART_TRANSCEIVER_RX);
        hwInit();
}

/**************************************************************************//**
\brief Clear startup initialization parameters to start user application
******************************************************************************/
void hwUnInitUsart0(void)
{

	usart_reset(&host_uart_module);

	usart_disable_transceiver(&host_uart_module, USART_TRANSCEIVER_TX);
	usart_disable_transceiver(&host_uart_module, USART_TRANSCEIVER_RX);
        hwUnInit();
}
void usart_disable0(void)
{
	hwUnInitUsart0();
	struct system_pinmux_config pin_conf;
	system_pinmux_get_config_defaults(&pin_conf);
		uint32_t pad_pinmuxes[] = {
		HOST_SERCOM_PINMUX_PAD0, HOST_SERCOM_PINMUX_PAD1,
		HOST_SERCOM_PINMUX_PAD2, HOST_SERCOM_PINMUX_PAD3
	};

	/* Configure the SERCOM pins according to the user configuration */
	for (uint8_t pad = 0; pad < 4; pad++) {
		uint32_t current_pinmux = pad_pinmuxes[pad];

		if (current_pinmux == PINMUX_DEFAULT) {
			current_pinmux = _sercom_get_default_pad(USART_HOST, pad);
		}

		if (current_pinmux != PINMUX_UNUSED) {
			pin_conf.mux_position = current_pinmux & 0xFFFF;
			system_pinmux_pin_set_config(current_pinmux >> 16, &pin_conf);
		}
	}
	//usart_disable(&host_uart_module);
}

#endif // (USE_USART0 == 1) || (USE_USART1 == 1)

#endif // (USE_USART0 == 1) || (USE_USART1 == 1)

// eof uartSerializer.c
