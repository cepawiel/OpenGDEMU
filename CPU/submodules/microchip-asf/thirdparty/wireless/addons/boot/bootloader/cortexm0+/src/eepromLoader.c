/**
* \file  eepromLoader.c
*
* \brief Implementation of internal eeprom loader.
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
#include <bootStructure.h>
#include <flashLoader.h>


/******************************************************************************
                   Define(s) section
******************************************************************************/
#define EEPROM_ADDRESS_MASK           0x0003FFFF

/******************************************************************************
                   External global variables section
******************************************************************************/
extern BootBuffer_t dataBuffer;

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************\\**
\brief Write eeprom page and counting address and length for next write
******************************************************************************/
void eepromPacketToPage(void)
{
  HW_EepromParams_t eepromParams;

  dataBuffer.address &= EEPROM_ADDRESS_MASK;

  eepromParams.address =  dataBuffer.address;
  eepromParams.data = dataBuffer.data;
  eepromParams.length = dataBuffer.dataSize;

  while (eepromParams.length > 0)
    hwFlashWriteEepromPage(&eepromParams);
}

/**************************************************************************//**
\brief Flush unloaded data before exit from bootloader.
******************************************************************************/
void eepromFlushPage(void)
{
}

/**************************************************************************//**
\brief Reads data from the EEPROM.

\param[in]
  address - address of data byte.
\return
  read byte
******************************************************************************/

// eof eepromLoader.c
