/**
* \file  spiMemInterface.h
*
* \brief Declaration of external spi memory interface.
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
#ifndef _SPIMEMINTERFACE_H
#define _SPIMEMINTERFACE_H

/******************************************************************************
                   Includes section
******************************************************************************/
#ifdef EXT_MEMORY
#include <types.h>
#include <gpio.h>
#include <fcpu.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
#if defined(BOARD_SAMR21_XPRO)
#if defined(ATSAMR21G18A) || defined(ATSAMR21E18A)
#if (USE_SPI == 1)
HW_ASSIGN_PIN(EXT_MEM_CS,    A, 27);
HW_ASSIGN_PIN(SPI_SCK,       A, 28);
HW_ASSIGN_PIN(SPI_MOSI,      A, 25);
HW_ASSIGN_PIN(SPI_MISO,      A, 24);
#endif
#endif

HW_ASSIGN_PIN_FUNC(SPI_SCK,  A, 28, 5);
HW_ASSIGN_PIN_FUNC(SPI_MOSI, A, 25, 2);
HW_ASSIGN_PIN_FUNC(SPI_MISO, A, 24, 2);

#define SPI_CTRLA_DOPO 2
#define SPI_CTRLA_DIPO 2

#define SPI_CHANNEL &(SERCOM3->SPI)

#define SPI_SERCOM 3

#elif defined(BOARD_SAMR21_ZLLEK)
#if defined(ATSAMR21G18A) && (USE_SPI == 1)
HW_ASSIGN_PIN(EXT_MEM_CS,    A, 14);
HW_ASSIGN_PIN(SPI_SCK,       A, 9);
HW_ASSIGN_PIN(SPI_MOSI,      A, 8);
HW_ASSIGN_PIN(SPI_MISO,      A, 15);
#endif

HW_ASSIGN_PIN_FUNC(SPI_SCK,  A, 9, 3);
HW_ASSIGN_PIN_FUNC(SPI_MOSI, A, 8, 3);
HW_ASSIGN_PIN_FUNC(SPI_MISO, A, 15, 2);

#define SPI_CTRLA_DOPO 0
#define SPI_CTRLA_DIPO 3

#define SPI_CHANNEL &(SERCOM2->SPI)

#define SPI_SERCOM 2

#elif defined(BOARD_SAMR21G18_MR210UA_MODULE)
#if defined(ATSAMR21G18A) && (USE_SPI == 1)
HW_ASSIGN_PIN(EXT_MEM_CS,    B, 2);
HW_ASSIGN_PIN(SPI_SCK,       B, 3);
HW_ASSIGN_PIN(SPI_MOSI,      B, 23);
HW_ASSIGN_PIN(SPI_MISO,      B, 22);
#endif

HW_ASSIGN_PIN_FUNC(SPI_SCK,  B, 3, 3);
HW_ASSIGN_PIN_FUNC(SPI_MOSI, B, 23, 3);
HW_ASSIGN_PIN_FUNC(SPI_MISO, B, 22, 3);

#define SPI_CTRLA_DOPO 2
#define SPI_CTRLA_DIPO 2

#define SPI_CHANNEL &(SERCOM5->SPI)

#define SPI_SERCOM 5
#elif defined(ATSAMR21E19A)
#if (USE_SPI == 1)
HW_ASSIGN_PIN(EXT_MEM_CS,    A, 23);
HW_ASSIGN_PIN(SPI_SCK,       B, 23);
HW_ASSIGN_PIN(SPI_MOSI,      A, 22);
HW_ASSIGN_PIN(SPI_MISO,      B, 22);
HW_ASSIGN_PIN(SPI_HOLD,      A, 0);
HW_ASSIGN_PIN(SPI_WP,      A, 12);
//HW_ASSIGN_PIN(SPI_MOSI,      B, 22);
//HW_ASSIGN_PIN(SPI_MISO,      A, 22);

#endif


HW_ASSIGN_PIN_FUNC(SPI_SCK,  B, 23, 3);
HW_ASSIGN_PIN_FUNC(SPI_MOSI, A, 22, 3);
HW_ASSIGN_PIN_FUNC(SPI_MISO, B, 22, 3);
//HW_ASSIGN_PIN_FUNC(SPI_MOSI, B, 22, 3);
//HW_ASSIGN_PIN_FUNC(SPI_MISO, A, 22, 3);
#define SPI_CTRLA_DOPO 3
#define SPI_CTRLA_DIPO 2
//#define SPI_CTRLA_DOPO 1
//#define SPI_CTRLA_DIPO 0
#define SPI_CHANNEL &(SERCOM5->SPI)

#define SPI_SERCOM 5
#elif defined( BOARD_SAMR21B18_MZ210PA_MODULE)
#if defined(ATSAMR21E18A) && (USE_SPI == 1)
#if (USE_SPI == 1)
HW_ASSIGN_PIN(EXT_MEM_CS,    A, 27);
HW_ASSIGN_PIN(SPI_SCK,       A, 28);
HW_ASSIGN_PIN(SPI_MOSI,      A, 25);
HW_ASSIGN_PIN(SPI_MISO,      A, 24);
#endif
#endif

HW_ASSIGN_PIN_FUNC(SPI_SCK,  A, 28, 5);
HW_ASSIGN_PIN_FUNC(SPI_MOSI, A, 25, 2);
HW_ASSIGN_PIN_FUNC(SPI_MISO, A, 24, 2);

#define SPI_CTRLA_DOPO 2
#define SPI_CTRLA_DIPO 2

#define SPI_CHANNEL &(SERCOM3->SPI)

#define SPI_SERCOM 3
#elif defined(BOARD_SAMR30_MODULE_XPRO)
#if defined(ATSAMR30E18A)
#if (USE_SPI == 1)
HW_ASSIGN_PIN(EXT_MEM_CS,    A, 27);
HW_ASSIGN_PIN(SPI_SCK,       A, 19);
HW_ASSIGN_PIN(SPI_MOSI,      A, 18);
HW_ASSIGN_PIN(SPI_MISO,      A, 16);
#endif
#endif

HW_ASSIGN_PIN_FUNC(SPI_SCK,  A, 19, 2);
HW_ASSIGN_PIN_FUNC(SPI_MOSI, A, 18, 2);
HW_ASSIGN_PIN_FUNC(SPI_MISO, A, 16, 2);

#define SPI_CTRLA_DOPO 1
#define SPI_CTRLA_DIPO 0

#define SPI_CHANNEL &(SERCOM1->SPI)

#define SPI_SERCOM 1
#else
#error 'undefined'
#endif /* #ifdef BOARD_SAMR21_XPRO */

#if defined(ATSAMR21G18A) || defined(ATSAMR21E18A) || defined(ATSAMR21E19A)
/* 14 - SERCOM0_CLOCK  */
#define SPI_SERCOM_CLOCK(x)  (0x14 + x)
#endif

#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#define SPI_SERCOM_CLOCK(x)  (0x12 + x)
#endif

#define SPI_CLOCK_RATE_2000  ((F_CPU / (2 * 2000000ul)) - 1)

/******************************************************************************
                   Types section
******************************************************************************/
typedef enum
{
  SPI_TRANSACTION_TYPE_READ,
  SPI_TRANSACTION_TYPE_WRITE
} TransactionType_t;

// spi channel
typedef SercomSpi* SpiChannel_t;

/******************************************************************************
                     Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Initializes spi.
******************************************************************************/
void spiMemInit(void);

/**************************************************************************//**
\brief Unintializies spi.
******************************************************************************/
void spiMemUnInit(void);

/**************************************************************************//**
\brief Writes or reads a length bytes from the external spi memory.

\param[in]
  type   -  transaction type;
\param[in]
  buffer -  pointer to the data buffer;
\param[in]
  length -  number bytes for transfer;
******************************************************************************/
void spiMemTransac(TransactionType_t type, uint8_t *buffer, uint16_t length);
#endif
#endif /* _SPIMEMINTERFACE_H */
