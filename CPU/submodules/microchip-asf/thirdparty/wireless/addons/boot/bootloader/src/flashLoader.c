/**
* \file  flashLoader.c
*
* \brief Implementation of internal flash loader.
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
#include <asf.h>
#include <flashLoader.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/


#define FLASH_READY() (NVMCTRL_INTFLAG_s.ready)

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

#define FIRST_PAGE             0
#define LAST_PAGE            1023

/******************************************************************************
                   Types section
******************************************************************************/
typedef union
{
  uint8_t  byteData[2];
  uint16_t wordData;
} FlashWord_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/



/******************************************************************************
                   External global variables section
******************************************************************************/
extern BootBuffer_t dataBuffer;

/******************************************************************************
                   Global variables section
******************************************************************************/
static uint16_t currentPage = 0;
static uint8_t temporaryPageBuffer[FLASH_PAGESIZE];
uint32_t appStartAddr;
static uint16_t pageBufferToFill[ROW_SIZE_IN_HALF_WORDS];

/******************************************************************************
                    Implementation section
******************************************************************************/
/**************************************************************************//**
\brief Load received date to the flash.
******************************************************************************/
void flashPacketToPage(void)
{
  uint16_t packetPage;
  uint16_t tableAddress;
  uint8_t *dataPointer;

  dataPointer = dataBuffer.data;
 
  while (dataBuffer.dataSize)
  {
    packetPage = dataBuffer.address / FLASH_PAGESIZE;

    if ( 0 == currentPage)
    {
      // first data of the first page of application image
      currentPage = dataBuffer.address / FLASH_PAGESIZE;
      appStartAddr = dataBuffer.address;
    }
    if (currentPage != packetPage)
    {
      hwEraseFlashPage(currentPage *FLASH_PAGESIZE);
      hwFillFlashPageBuffer(currentPage *FLASH_PAGESIZE, FLASH_PAGESIZE, &temporaryPageBuffer[0]);
      hwWriteFlashPage(currentPage *FLASH_PAGESIZE);
      memset(&temporaryPageBuffer[0], 0xFF, FLASH_PAGESIZE);
      currentPage = packetPage;
    }

    tableAddress = dataBuffer.address & PAGE_POSITION_OFFSET_MASK;
    temporaryPageBuffer[tableAddress] = *dataPointer++;
    dataBuffer.address++;
    dataBuffer.dataSize--;
  }
}

/**************************************************************************//**
\brief Flush unloaded data before exit from bootloader.
******************************************************************************/
void flashFlushPage(void)
{
  // Do a Erase-Write of the page.
  hwEraseFlashPage(currentPage *FLASH_PAGESIZE);
  hwFillFlashPageBuffer(currentPage *FLASH_PAGESIZE, FLASH_PAGESIZE, &temporaryPageBuffer[0]);
  hwWriteFlashPage(currentPage *FLASH_PAGESIZE);
  flashUpdateAppStartAddr();
}
/**************************************************************************//**
  \brief Writes data to the page buffer.
  \param[in] startOffset - start position for writing within the page
  \param[in] length - data to write length in bytes
  \param[in] data - pointer to data to write
******************************************************************************/
void hwFillFlashPageBuffer(uint32_t startOffset, uint16_t length, uint8_t *data)
{
  // Page size is 64 Bytes so that Row size becomes 4*64 = 256 Bytes
  // max. value of length may be 256 Bytes
  uint16_t unalignedWord;
  uint16_t alignedWord;
  uint8_t wordPos;
  
  if (startOffset % ROW_SIZE + length > ROW_SIZE)
  {
    // leaking into next row & abort the file operation
    return;
  }
  
  // Initialize the page buffer
  for (uint16_t index = 0; index < ROW_SIZE_IN_HALF_WORDS; index++)
    pageBufferToFill[index] = 0xFFFF;
  // Check, if first byte of current data is unaligned
  if (startOffset % 2)
  {
    /* Fill it as a MSB of a word */
    unalignedWord = 0x00FF | ((uint16_t)*data++ << 8);
    wordPos = (startOffset % ROW_SIZE - 1) / 2;
    pageBufferToFill[wordPos] = unalignedWord;
    startOffset++;
    length--;
  }

  // Handle if the last byte is not aligned
  if (length % 2)
  {
    unalignedWord = 0xFF00 | data[length - 1];
    wordPos = (startOffset % ROW_SIZE + length - 1) / 2 ;
    pageBufferToFill[wordPos] = unalignedWord;
    length--;
  }
  
  // Copy the remaining aligneed bytes to page buffer
  for (uint8_t index =0; index < length/2; index++)
  {
    wordPos = startOffset % ROW_SIZE / 2;
    alignedWord = *data++;
    alignedWord = alignedWord | (*data++ << 8);
    pageBufferToFill[wordPos] = alignedWord;
    startOffset += 2;
  }
}

/**************************************************************************//**
  \brief Erases specified flash page.
  \param[in] pageStartAddress - address within the target page
******************************************************************************/
void hwEraseFlashPage(uint32_t pageStartAddress)
{
	nvm_erase_row(pageStartAddress);

}

/**************************************************************************//**
  \brief Writes flash page buffer without erasing.
  \param[in] pageStartAddress - address within the target page
******************************************************************************/
void hwWriteFlashPage(uint32_t pageStartAddress)
{
  /* PDS page is mapped to row. so finding out the starting of row */
  if (pageStartAddress % ROW_SIZE)
    pageStartAddress = pageStartAddress - (pageStartAddress % ROW_SIZE);


  for (uint8_t pageCnt = 0; pageCnt < NO_OF_PAGES_PER_ROW; pageCnt++)
  {

	nvm_write_buffer(pageStartAddress,(((uint8_t *)pageBufferToFill)+(MCU_PAGE_SIZE*pageCnt)),MCU_PAGE_SIZE);

    pageStartAddress += MCU_PAGE_SIZE;
  }
}



/**************************************************************************//**
\brief Updates application start address in end of flash
******************************************************************************/
void flashUpdateAppStartAddr(void)
{
  uint32_t tempAddr;
  uint8_t tempPageData[FLASH_PAGESIZE];
  uint16_t PageNo;

  tempAddr = FLASH_END_ADDRESS - FLASH_PAGESIZE + 1;
  memcpy_P(&tempPageData[0], (void *)tempAddr, FLASH_PAGESIZE);
  memcpy(&tempPageData[FLASH_PAGESIZE-4], &appStartAddr, sizeof(uint32_t));

  PageNo = tempAddr / FLASH_PAGESIZE;
  hwEraseFlashPage(PageNo*FLASH_PAGESIZE);
  hwFillFlashPageBuffer(PageNo*FLASH_PAGESIZE, FLASH_PAGESIZE, &tempPageData[0]);
  hwWriteFlashPage(PageNo*FLASH_PAGESIZE);
  
}
/**************************************************************************//**
\brief Writes a page to the EEPROM.

\param[in]
  address - address of data byte.
\param[in]
  data - data for saving.
******************************************************************************/
 void hwFlashWriteEepromPage(HW_EepromParams_t *eeprom)
{
  uint16_t bytesToWrite;
  uint16_t pageNo;
  uint8_t pageBufferEeprom[ROW_SIZE];

  pageNo = eeprom->address / ROW_SIZE;

  bytesToWrite = ROW_SIZE - (eeprom->address % ROW_SIZE);
  memcpy_P(&pageBufferEeprom[0], (void *)(pageNo * ROW_SIZE), ROW_SIZE);
  
  if (eeprom->length < bytesToWrite)
    bytesToWrite = eeprom->length;

  memcpy(&pageBufferEeprom[eeprom->address % ROW_SIZE], eeprom->data, bytesToWrite);

  hwEraseFlashPage(pageNo*ROW_SIZE);
  hwFillFlashPageBuffer(pageNo*ROW_SIZE, ROW_SIZE, &pageBufferEeprom[0]);
  hwWriteFlashPage(pageNo*ROW_SIZE);
  
  eeprom->address += bytesToWrite;
  eeprom->length -= bytesToWrite;
  eeprom->data += bytesToWrite;
}

void nvm_init(void)
{
	/*struct nvm_config config_nvm;
	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);*/
	struct nvm_config config;
	/* Get the default configuration */
	nvm_get_config_defaults(&config);

	/* Enable automatic page write mode */
	config.manual_page_write = false;

	/* Set wait state to 1 */
	config.wait_states = 2;

	/* Set the NVM configuration */
	nvm_set_config(&config);
}
// eof flashLoader.c
