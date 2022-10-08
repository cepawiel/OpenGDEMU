/**
* \file  uartSerializer.h
*
* \brief Declaration of uart serialize interface.
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
#ifndef _UARTSERIALIZER_H
#define _UARTSERIALIZER_H
#include <types.h>
#include <gpio.h>
#include <fcpu.h>
#include <bmm.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/

#if RF_SERVER_MODE
#define BAUD_RATE 115200ul
#else
#define BAUD_RATE 115200ul
#endif

#ifdef BOARD_SAMR21_XPRO
/* USART0 Pin assignments */
HW_ASSIGN_PIN(USART0_TXD, A, 4);
HW_ASSIGN_PIN(USART0_RXD, A, 5);

HW_ASSIGN_PIN_FUNC(USART0_TXD, A, 4, 3);
HW_ASSIGN_PIN_FUNC(USART0_RXD, A, 5, 3);

#define USART0_RX_PAD 1
#define USART0_TX_PAD 0

#define USART_CHANNEL_0 &(SERCOM0->USART)

/* Define the sercom no. 0-5 */
#define USART0_SERCOM 0

/* USART1 Pin assignments */
HW_ASSIGN_PIN(USART1_TXD, A, 6);
HW_ASSIGN_PIN(USART1_RXD, A, 7);

HW_ASSIGN_PIN_FUNC(USART1_TXD, A, 6, 3);
HW_ASSIGN_PIN_FUNC(USART1_RXD, A, 7, 3);

#define USART1_RX_PAD 3
#define USART1_TX_PAD 1

#define USART_CHANNEL_1 &(SERCOM0->USART)

/* Define the sercom no. 0-5 */
#define USART1_SERCOM 0

#elif defined(BOARD_SAMR21_ZLLEK)
/* USART0 Pin assignments */
HW_ASSIGN_PIN(USART0_TXD, B, 2);
HW_ASSIGN_PIN(USART0_RXD, B, 3);

HW_ASSIGN_PIN_FUNC(USART0_TXD, B, 2, 3);
HW_ASSIGN_PIN_FUNC(USART0_RXD, B, 3, 3);

#define USART0_RX_PAD 1
#define USART0_TX_PAD 0

#define USART_CHANNEL_0 &(SERCOM5->USART)

/* Define the sercom no. 0-5 */
#define USART0_SERCOM 5
/* USART1 Pin assignments */
HW_ASSIGN_PIN(USART1_TXD, A, 27);
HW_ASSIGN_PIN(USART1_RXD, A, 28);

HW_ASSIGN_PIN_FUNC(USART1_TXD, A, 27, 5);
HW_ASSIGN_PIN_FUNC(USART1_RXD, A, 28, 5);

#define USART1_RX_PAD 1
#define USART1_TX_PAD 0

#define USART_CHANNEL_1 &(SERCOM3->USART)

/* Define the sercom no. 0-5 */
#define USART1_SERCOM 3




#elif defined(BOARD_SAMR21G18_MR210UA_MODULE)
/* USART0 Pin assignments */
HW_ASSIGN_PIN(USART0_TXD,     A, 8);
HW_ASSIGN_PIN(USART0_RXD,     A, 7);

HW_ASSIGN_PIN_FUNC(USART0_TXD, A, 8, 2);
HW_ASSIGN_PIN_FUNC(USART0_RXD, A, 7, 3);

/* USART1 Pin assignments */
HW_ASSIGN_PIN(USART1_TXD,     A, 14);
HW_ASSIGN_PIN(USART1_RXD,     A, 15);

HW_ASSIGN_PIN_FUNC(USART1_TXD, A, 14, 2);
HW_ASSIGN_PIN_FUNC(USART1_RXD, A, 15, 2);

#define USART0_RX_PAD 3
#define USART0_TX_PAD 0

#define USART1_RX_PAD 3
#define USART1_TX_PAD 2

#define USART_CHANNEL_0 &(SERCOM0->USART)
#define USART_CHANNEL_1 &(SERCOM2->USART)

/* Define the sercom no. 0-5 */
#define USART0_SERCOM 0
#define USART1_SERCOM 2

#elif defined(BOARD_SAMR21B18_MZ210PA_MODULE)
/* USART0 Pin assignments */
HW_ASSIGN_PIN(USART0_TXD,     A, 14);
HW_ASSIGN_PIN(USART0_RXD,     A, 15);

HW_ASSIGN_PIN_FUNC(USART0_TXD, A, 14, 2);
HW_ASSIGN_PIN_FUNC(USART0_RXD, A, 15, 2);

/* USART1 Pin assignments */
HW_ASSIGN_PIN(USART1_TXD,     A, 18);
HW_ASSIGN_PIN(USART1_RXD,     A, 19);

HW_ASSIGN_PIN_FUNC(USART1_TXD, A, 18, 2);
HW_ASSIGN_PIN_FUNC(USART1_RXD, A, 19, 2);

#define USART0_RX_PAD 3
#define USART0_TX_PAD 1

#define USART1_RX_PAD 3
#define USART1_TX_PAD 2

#define USART_CHANNEL_0 &(SERCOM2->USART)
#define USART_CHANNEL_1 &(SERCOM1->USART)

/* Define the sercom no. 0-5 */
#define USART0_SERCOM 2
#define USART1_SERCOM 1

#elif defined(BOARD_SAMR21_CUSTOM)
/* USART0 Pin assignments */
HW_ASSIGN_PIN(USART0_TXD,     A, 14);
HW_ASSIGN_PIN(USART0_RXD,     A, 15);

HW_ASSIGN_PIN_FUNC(USART0_TXD, A, 14, 2);
HW_ASSIGN_PIN_FUNC(USART0_RXD, A, 15, 2);

/* USART1 Pin assignments */
HW_ASSIGN_PIN(USART1_TXD,     A, 16);
HW_ASSIGN_PIN(USART1_RXD,     A, 17);

HW_ASSIGN_PIN_FUNC(USART1_TXD, A, 16, 2);
HW_ASSIGN_PIN_FUNC(USART1_RXD, A, 17, 2);

#define USART0_RX_PAD 3
#define USART0_TX_PAD 1

#define USART1_RX_PAD 1
#define USART1_TX_PAD 0

#define USART_CHANNEL_0 &(SERCOM2->USART)
#define USART_CHANNEL_1 &(SERCOM1->USART)

/* Define the sercom no. 0-5 */
#define USART0_SERCOM 2
#define USART1_SERCOM 1
#elif defined(BOARD_SAMR30_XPRO)
/* USART0 Pin assignments */
HW_ASSIGN_PIN(USART0_TXD, A, 4);
HW_ASSIGN_PIN(USART0_RXD, A, 5);

HW_ASSIGN_PIN_FUNC(USART0_TXD, A, 4, 3);
HW_ASSIGN_PIN_FUNC(USART0_RXD, A, 5, 3);

#define USART0_RX_PAD 1
#define USART0_TX_PAD 0

#define USART_CHANNEL_0 &(SERCOM0->USART)

/* Define the sercom no. 0-5 */
#define USART0_SERCOM 0

#elif defined(BOARD_SAMR30_MODULE_XPRO)
/* USART0 Pin assignments */
HW_ASSIGN_PIN(USART0_TXD, A, 24);
HW_ASSIGN_PIN(USART0_RXD, A, 25);

HW_ASSIGN_PIN_FUNC(USART0_TXD, A, 24, 2);
HW_ASSIGN_PIN_FUNC(USART0_RXD, A, 25, 2);

#define USART0_RX_PAD 3
#define USART0_TX_PAD 1

#define USART_CHANNEL_0 &(SERCOM3->USART)

/* Define the sercom no. 0-5 */
#define USART0_SERCOM 3
#else
#error 'undefined'
#endif

#if defined(ATSAMR21G18A) || defined(ATSAMR21E18A) || defined(ATSAMR21E19A)
/* 14 - SERCOM0_CLOCK  */
#define USART_SERCOM_CLOCK(x)  (0x14 + x)
#endif /* #ifdef BOARD_SAMR21_XPRO */

#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#define USART_SERCOM_CLOCK(x)  (0x12 + x)
#endif
/******************************************************************************
                   Prototypes section
******************************************************************************/
#if (USE_USART0 == 1)
/**************************************************************************//**
\brief Startup initialization of the usart0
******************************************************************************/
void hwInitUsart0(void);

/**************************************************************************//**
\brief Clear startup initialization parameters to start user application
******************************************************************************/
void hwUnInitUsart0(void);
#endif

#if (USE_USART1 == 1)
/**************************************************************************//**
\brief Startup initialization of the usart1
******************************************************************************/
void hwInitUsart1(void);

/**************************************************************************//**
\brief Clear startup initialization parameters to start user application
******************************************************************************/
void hwUnInitUsart1(void);
#endif

/**************************************************************************//**
\brief Receive byte on uart.

\param[out]
  p - pointer to byte memory;

\return
    true - there is received byte; \n
    false - there is not received byte;
******************************************************************************/
bool getByteUsart(uint8_t *p);

/**************************************************************************//**
\brief Transmit byte to uart.

\param[in]
  len - number of bytes to transmit;
\param[in]
  p - pointer to byte memory;
******************************************************************************/
void setByteUsart(uint16_t len, uint8_t *p);

#endif //_UARTSERIALIZER_H
