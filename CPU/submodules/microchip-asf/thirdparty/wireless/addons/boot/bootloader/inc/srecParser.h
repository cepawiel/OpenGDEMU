/**
* \file  srecParser.h
*
* \brief Declaration of parser of the srec format interface.
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
#ifndef _SRECPARSER_H
#define _SRECPARSER_H

#include "parts.h"
/******************************************************************************
                   Define(s) section
******************************************************************************/
/******************************************************************************
                   Define(s) section
******************************************************************************/
#define S_SYMBOL                  0x53
#define HANDSHAKE_REQ             0x4b65a5b2
#define HANDSHAKE_CONF            0x26d2d369
#define ACK                       0xb49a5a4d
#define NACK                      0xb25a592d
#define CHECKSUM_AREA_SIZE        1
#define SRECORD_SIZE              128                        // The conventional data field length in S-Record

#if SAML21 || SAMR30 || SAMR34
  #define EEPROM_AREA_SIZE        0x1000                     // 4kbytes boot area size
  #define EEPROM_END_ADDRESS      0x3FFFF                    // End address of EEPROM area
  #define FLASH_END_ADDRESS      (0x3FFFF)        // End address of FLASH area      
    #define EEPROM_START_ADDRESS      0x00400000                   // Start address of EEPROM area
    #define FLASH_START_ADDRESS       0x000000
#else
  #error "Unsupported microcontroller."
#endif

/******************************************************************************
                   Types section
******************************************************************************/
// Type of record (S0, S1, S2, S3, S5, S7, S8, or S9).
typedef enum
{
  S0 = 0x5330,
  S1 = 0x5331,
  S2 = 0x5332,
  S3 = 0x5333,
  S5 = 0x5335,
  S7 = 0x5337,
  S19 = 0x5338,
  S9 = 0x5339
} SrecordType_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Handshake procedure.
******************************************************************************/
void srecHandshake(void);

/**************************************************************************//**
\brief Communication support between host and mcu
******************************************************************************/
void srecProtocol(void);

void convertStreamToNumbers(uint8_t *stream, uint8_t size);
bool readFromPreBuffer(uint8_t *data);
#endif // _SRECPARSER_H
