/**
* \file  srecParser.c
*
* \brief Implementation of real time srec parser.
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
#include <abstractSerializer.h>
#include <bootStructure.h>
#include <flashLoader.h>
#include <eepromLoader.h>
#include "asf.h"
#include "uartSerializer.h"
#include <appEntry.h>
#include "app_info.h"
/******************************************************************************
                   Prototypes section
******************************************************************************/
static void srecWaitForType(SrecordType_t *srecType);
static uint8_t srecWaitForAddress(uint8_t addressLength);
static bool srecParser(void);

/******************************************************************************
                   Global variables section
******************************************************************************/
BootBuffer_t dataBuffer;
static uint32_t srecACK  = ACK;
static uint32_t srecNACK = NACK;
static uint32_t previousAddress = 0xFFFFFFFF;
static uint32_t handshakeConf = HANDSHAKE_CONF;

extern app_info_t checkapp_info;
#define EEPROM_PAGE_SIZE 256u
/******************************************************************************
                    Implementation section
******************************************************************************/
/**************************************************************************//**
\brief Waiting for a field of the type from the serial interface.

\param[out]
  srecType - pointer to buffer;
******************************************************************************/
static void srecWaitForType(SrecordType_t *srecType)
{
  do
  {
    bootGetChar((uint8_t *)srecType, INFINITE);
  } while(S_SYMBOL != (uint8_t)*srecType);
  *srecType <<= 8;
  bootGetChar((uint8_t *)srecType, INFINITE);
}
void jumpToApplication(void)
{    flag=0;
	 uint32_t appEntryPoint;
	//uint32_t appEntryPoint = *(uint32_t *)(FLASH_END_ADDRESS - sizeof(uint32_t) + 1);
	 if(isCrcMatched==false)
	 {
		 appEntryPoint = (uint32_t )(0x4000);
	 }
	 else
	 { 
		 appEntryPoint = checkapp_info.appTable[0].app_start_addr;
	 }
	void(* startAddress)(void) =(void(*)(void))(*(uint32_t *)(appEntryPoint + sizeof(uint32_t)));
	//__set_MSP(*(uint32_t *)appEntryPoint);
	#ifdef  __IAR_SYSTEMS_ICC__
	__set_SP(*(uint32_t *)appEntryPoint);
	#elif __GNUC__
	__asm__ volatile ("mov r13, %0; \n\t"
	:
	: "r" (*(uint32_t *)appEntryPoint)
	);
	#endif

	// nulling extended indirect register for indirect call out memory
	CLEAR_EIND();
	startAddress();
}
/**************************************************************************//**
\brief Waiting for a field of the type from the internal address

\param[in]
  addressLength - length of address field;

\return
  checksum with address field
******************************************************************************/
static uint8_t srecWaitForAddress(uint8_t addressLength)
{
  uint8_t checksum = 0;

  while(addressLength)
  {
    addressLength--;
    bootGetChar((((uint8_t *)&dataBuffer.address) + addressLength), INFINITE);
    checksum += *(((uint8_t *)&dataBuffer.address) + addressLength);
  }

  return checksum;
}

/**************************************************************************//**
\brief Communication support between host and mcu
******************************************************************************/
void srecProtocol(void)
{ 
	 
  while(1)
  {
    if (srecParser())
    {
      if ((S0 == dataBuffer.recordType) || (S5 == dataBuffer.recordType))
      {
        bootSetChar(sizeof(srecACK), (uint8_t*)(&srecACK));
        continue;
      }

      if ((S7 == dataBuffer.recordType) || (S19 == dataBuffer.recordType) || (S9 == dataBuffer.recordType))
      {
        flashFlushPage();
        eepromFlushPage();
        bootSetChar(sizeof(srecACK), (uint8_t*)(&srecACK));
		bootUnInitSerializer();
		for(uint32_t i=0; i<64; i++)
		{
			nvm_erase_row(EEPROM_START_ADDRESS + (256u *i));
		}
		
       //jumpToApplication();
	   lowLevelUnInit();
		flag = RESET_FLAG;
		system_reset();
      }

      if (previousAddress == dataBuffer.address)
      { // host did not listen ack and repeat previous packet
        bootSetChar(sizeof(srecACK), (uint8_t*)(&srecACK));
        continue;
      }
      previousAddress = dataBuffer.address;

      if ((dataBuffer.address + dataBuffer.dataSize) <= (FLASH_END_ADDRESS + 1))
      { // flash section
        flashPacketToPage();
      }
      else if (dataBuffer.address < EEPROM_START_ADDRESS)
      {
        bootSetChar(sizeof(srecNACK), (uint8_t*)(&srecNACK));
        continue;
      }

      if ((dataBuffer.address >= EEPROM_START_ADDRESS) && (dataBuffer.address <= EEPROM_END_ADDRESS))
      { // eeprom section
        eepromPacketToPage();
      }

      // send ack
      bootSetChar(sizeof(srecACK), (uint8_t*)(&srecACK));
    }
    else
    {
      bootSetChar(sizeof(srecNACK), (uint8_t*)(&srecNACK));
    }
  }
}

/**************************************************************************//**
\brief Real time parser of srec format.

\return
  false - parse error
  true  - parse ok
******************************************************************************/
static bool srecParser(void)
{
  uint8_t addressLength;
  uint8_t packetLength;
  uint8_t checksum = 0;
  uint8_t checksumFromSerial;

  // start of packet is transmitted as ascii code.
  // the others are transmitted as hex code.
  // search begin of packet
  srecWaitForType(&dataBuffer.recordType);

  switch (dataBuffer.recordType)
  {
    case S0: // the address field is 2 bytes
    case S1:
    case S5:
    case S9:
        addressLength = 2;
      break;
    case S2: // the address field is 3 bytes
    case S19:
        addressLength = 3;
      break;
    case S3: // the address field is 4 bytes
    case S7:
        addressLength = 4;
      break;
    default: // unsupported type
      return false;
   }

  // get packet length
  bootGetChar(&packetLength, INFINITE);
  if (packetLength > (SREC_DATA_LENGTH + addressLength + CHECKSUM_AREA_SIZE))
    return false; // this length is not correct
  checksum += packetLength;
  dataBuffer.dataSize = packetLength - addressLength - CHECKSUM_AREA_SIZE;

  // get internal address
  checksum += srecWaitForAddress(addressLength);
  packetLength -= addressLength;

  // get data array
  uint8_t *tempPoiner = dataBuffer.data;
  while (--packetLength)
  {
    bootGetChar(tempPoiner, INFINITE);
    checksum += *tempPoiner++;
  }

  // get checksum
  bootGetChar(&checksumFromSerial, INFINITE);
  checksum += checksumFromSerial;
  if (0xFF != checksum)
    return false;

  return true;
}

/**************************************************************************//**
\brief Handshake procedure.
******************************************************************************/
void srecHandshake(void)
{
  if (bootScanInterfaces())
  {
    bootSetChar(sizeof(handshakeConf), (uint8_t *)(&handshakeConf)); // ack for handshake
    return;
  }

  bootUnInitSerializer();
  //jumpToApplication();
 lowLevelUnInit();
  flag = RESET_FLAG;
  system_reset();
}

// eof srecParser.c
