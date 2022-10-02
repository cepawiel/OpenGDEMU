/**
* \file  flashLoader.h
*
* \brief Declaration api of flash loader.
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
#ifndef _FLASHLOADER_H
#define _FLASHLOADER_H

#include <eepromLoader.h>
#include <extMemReader.h>
#include <srecParser.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
#define CMD_KEY                       0xA5
#define MCU_PAGE_SIZE                 64
#define ROW_SIZE                      (4 * MCU_PAGE_SIZE)
#define ROW_SIZE_IN_HALF_WORDS        (ROW_SIZE / 2)
#define FLASH_MAIN_ADDRESS_SPACE      (0x00)
#define NO_OF_PAGES_PER_ROW           4
#define NO_OF_HALF_WORDS_PER_PAGE     32



typedef enum flash_cmd_tag {
  ERASE_ROW = 0x02,
  WRITE_PAGE = 0x04,
  ERASE_AUXILARYROW = 0x05,
  WRITE_AUXILARYPAGE = 0x06,
  LOCK_REGION = 0x40,
  UNLOCK_REGION = 0x41,
  SET_POWERREDUCTIONMODE = 0x42,
  CLEAR_POWERREDUCTIONMODE = 0x43,
  PAGE_BUFFERCLEAR = 0x44,
  SET_SECURITYBIT = 0x45,
  INVALID_ALLCACHE = 0x46
 } flash_cmd_t;


/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Load received date to the flash.
******************************************************************************/
void flashPacketToPage(void);

/**************************************************************************//**
\brief Flush unloaded data before exit from bootloader.
******************************************************************************/
void flashFlushPage(void);

/**************************************************************************//**
  \brief Erases specified flash row.
  \param[in] pageStartAddress - address within the target row
******************************************************************************/
void hwEraseFlashRow(uint32_t Address);

/****************** ********************************************************//**
\brief Writes the input data to flash, with pre-erase, one-page at a time.
******************************************************************************/
void halFlashWrite(uint32_t address, uint8_t *data, uint32_t length);

/****************** ********************************************************//**
\brief Writes flash page buffer without erasing.
******************************************************************************/
void hwWriteFlashPage(uint32_t pageStartAddress, uint16_t* wordBuf);

/**************************************************************************//**
\brief Writes a page to the EEPROM.

\param[in]
  address - address of data byte.
\param[in]
  data - data for saving.
\param[in]
  length - No of bytes.
******************************************************************************/
 void hwFlashWriteEepromPage(HW_EepromParams_t *eeprom);
 


void hwExecuteCommand(flash_cmd_t cmd);
void flashUpdateAppStartAddr(void);
// void halFlashWrite(uint32_t address,uint8_t *data,uint16_t length);

extern uint32_t appStartAddr;

#endif // _FLASHLOADER_H
