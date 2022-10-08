/**
* \file  abstractSerializer.h
*
* \brief Declaration of abstract serialize interface.
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
#ifndef _ABSTRACTSERIALIZER_H
#define _ABSTRACTSERIALIZER_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <hardwareInit.h>
#include <uartSerializer.h>

/******************************************************************************
                   Types section
******************************************************************************/
/* Some serial interface descriptor. */
typedef struct
{
  void (* hwInit)(void);
  void (* hwUnInit)(void);
  bool (* getByte)(uint8_t *p);
  void (* setByte)(uint16_t len, uint8_t *p);
} SerialInterface_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Clear interface setting.
******************************************************************************/
void bootUnInitSerializer(void);

/**************************************************************************//**
\brief Read byte from interface.

\param[out]
  p - pointer to data buffer;
\param[in]
  flag - flag about unlimited or limited reading time.

\return
  true - byte was read, false - byte was not read.
******************************************************************************/
bool bootGetChar(uint8_t *p, bool flag);

/**************************************************************************//**
\brief Write byte to interface.

\param[in]
  len - data length;
\param[in]
  p - pointer to data buffer.
******************************************************************************/
void bootSetChar(uint16_t len, uint8_t *p);

/**************************************************************************//**
\brief Perform search interface activity and handshake message.

\return
  true - handshake was read, false - there is no activity.
******************************************************************************/
bool bootScanInterfaces(void);

#endif //_ABSTRACTSERIALIZER_H
