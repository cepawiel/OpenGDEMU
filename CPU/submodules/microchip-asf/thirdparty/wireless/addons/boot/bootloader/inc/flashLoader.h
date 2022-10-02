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

/******************************************************************************
                   Define(s) section
******************************************************************************/
#define FLASH_PAGESIZE               256u
#define PAGE_NUMBER_SHIFT               8
#define PAGE_POSITION_OFFSET_MASK   0x000000FF
#define CMD_KEY                       0xA5
#define MCU_PAGE_SIZE                 64
#define NO_OF_PAGES_PER_ROW           4
#define NO_OF_HALF_WORDS_PER_PAGE     32
#define ROW_SIZE                      (4 * MCU_PAGE_SIZE)
#define ROW_SIZE_IN_HALF_WORDS        (ROW_SIZE / 2)
#define FLASH_MAIN_ADDRESS_SPACE      (0x00)
 #define MCU_EEPROM_START_ADDR                    0x3F000
 
 extern uint32_t appStartAddr;

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
\brief Writes a page to the EEPROM.

\param[in]
  address - address of data byte.
\param[in]
  data - data for saving.
\param[in]
  length - No of bytes.
******************************************************************************/
 void hwFlashWriteEepromPage(HW_EepromParams_t *eeprom);
 void hwFillFlashPageBuffer(uint32_t startOffset, uint16_t length, uint8_t *data);
 void hwEraseFlashPage(uint32_t pageStartAddress);
 void hwWriteFlashPage(uint32_t pageStartAddress);
 
 void flashUpdateAppStartAddr(void);
 /**************************************************************************//**
  \brief Reads byte from EEPROM.
  \param[in] address - address of a byte.
  \return read value.
******************************************************************************/
INLINE uint8_t hwFlashRead(uint16_t address)
{
  return *(uint8_t *)(MCU_EEPROM_START_ADDR + address);
}

void nvm_init(void);
#endif // _FLASHLOADER_H
