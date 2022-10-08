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
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
#include <atsamr21.h>
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#include <atsamr30.h>
#endif
#include <core_cm0plus.h>
#include <flashLoader.h>
#include <appEntry.h>
#include <srecParser.h>
#include <hardwareInit.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
#define ROW_MASK   0x000000FF
// #define FLASH_PAGE_SIZE               256u
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
#define IS_FLASH_READY() (NVMCTRL_INTFLAG_s.ready)
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#define IS_FLASH_READY() (NVMCTRL->INTFLAG.bit.READY)
#endif

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
// static void hwFillFlashPageBuffer(uint32_t startOffset, uint16_t length, uint8_t *data);


/******************************************************************************
                   External global variables section
******************************************************************************/
extern BootBuffer_t dataBuffer;

uint32_t appStartAddr;

/******************************************************************************
                   Global variables section
******************************************************************************/
static uint16_t currentPage = 0;
static uint16_t packetBuffer[ROW_SIZE_IN_HALF_WORDS];

// static uint16_t pageBufferToFill[ROW_SIZE_IN_HALF_WORDS];

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
  uint8_t* bufPtr;

  dataPointer = dataBuffer.data;
  bufPtr = (uint8_t*) packetBuffer;
 
  while (dataBuffer.dataSize)
  {
    packetPage = dataBuffer.address / ROW_SIZE;

    if ( 0 == currentPage)
    {
      // first data of the first page of application image
      currentPage = dataBuffer.address / ROW_SIZE;
      appStartAddr = dataBuffer.address;
    }

    if (currentPage != packetPage)
    {
      hwEraseFlashRow(currentPage *ROW_SIZE);
      hwWriteFlashPage(currentPage *ROW_SIZE, packetBuffer);
      memset(bufPtr, 0xFF, ROW_SIZE);
      currentPage = packetPage;
    }

    tableAddress = dataBuffer.address & ROW_MASK;
    bufPtr[tableAddress] = *dataPointer++;
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
  hwEraseFlashRow(currentPage *ROW_SIZE);
  hwWriteFlashPage(currentPage *ROW_SIZE, packetBuffer);
  flashUpdateAppStartAddr();
}


/**************************************************************************//**
  \brief Erases specified flash page.
  \param[in] pageStartAddress - address within the target page
******************************************************************************/
void hwEraseFlashRow(uint32_t Address)
{
  if (Address % 2)
    Address--;
  /* Loading ADDR register */
  *(uint16_t *)(FLASH_MAIN_ADDRESS_SPACE + Address) = 0xFFFF;
  hwExecuteCommand(ERASE_ROW);
}

/**************************************************************************//**
  \brief Writes flash page buffer without erasing.
  \param[in] pageStartAddress - address within the target page
******************************************************************************/
void hwWriteFlashPage(uint32_t pageStartAddress, uint16_t* wordBuf)
{
  /* PDS page is mapped to row. so finding out the starting of row */
  if (pageStartAddress % ROW_SIZE)
    pageStartAddress = pageStartAddress - (pageStartAddress % ROW_SIZE);

  for (uint8_t pageCnt = 0; pageCnt < NO_OF_PAGES_PER_ROW; pageCnt++)
  {
    for (uint8_t wordCnt = 0; wordCnt < NO_OF_HALF_WORDS_PER_PAGE; wordCnt++)
      *(uint16_t *)(FLASH_MAIN_ADDRESS_SPACE + pageStartAddress + wordCnt * 2) = wordBuf[pageCnt * NO_OF_HALF_WORDS_PER_PAGE + wordCnt];

    hwExecuteCommand(WRITE_PAGE);
    pageStartAddress += MCU_PAGE_SIZE;
  }
}

/**************************************************************************//**
  \brief Executes the specified command.
  \param[in] cmd - flash command
******************************************************************************/
void hwExecuteCommand(flash_cmd_t cmd)
{
  while (!IS_FLASH_READY());
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  NVMCTRL_CTRLA = NVMCTRL_CTRLA_CMDEX(CMD_KEY)| NVMCTRL_CTRLA_CMD(cmd);
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX(CMD_KEY)| NVMCTRL_CTRLA_CMD(cmd);
#endif

}

/**************************************************************************//**
\brief Updates application start address in end of flash
******************************************************************************/
void flashUpdateAppStartAddr(void)
{
  uint32_t tempAddr;

  tempAddr = (FLASH_END_ADDRESS + 1) - 4;  
  halFlashWrite (tempAddr, (uint8_t*) &appStartAddr, sizeof(uint32_t));
  
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

  bytesToWrite = ROW_SIZE - (eeprom->address % ROW_SIZE);
  
  if (eeprom->length < bytesToWrite)
    bytesToWrite = eeprom->length;
  
  halFlashWrite (eeprom->address, eeprom->data, bytesToWrite);

}

/**************************************************************************//**
\brief Writes the input data to flash, with pre-erase, one-page at a time.

\param[in]
  address - Destination address in flash.
\param[in]
  data - Pointer to the input data buffer.
\param[in]
  length - Size of the input data buffer.
******************************************************************************/
void halFlashWrite(uint32_t address, uint8_t *data, uint32_t length)
{
  uint16_t    temp_buffer[ROW_SIZE_IN_HALF_WORDS];
  uint32_t    page_start,page_address;
  uint8_t     page_offset;
  uint16_t    rem_len, curr_len;
  uint16_t    unalignedWord;
  uint16_t    alignedWord;
  uint8_t     wordPos;
  
  curr_len = rem_len = length;
  page_address = address;
  page_offset = page_address%ROW_SIZE;
  page_start = page_address-page_offset;
  
  while (rem_len)
  { // Loop to erase-write one row (= 4 pages = 256 bytes) at a time 
      
    if((page_offset!=0) && (page_offset + rem_len > ROW_SIZE))
    {
      /** Start address not aligned to page boundary
       Fill partial page **/
      curr_len = ROW_SIZE - page_offset;
    }
    
    else if ((page_offset == 0) && (rem_len >= ROW_SIZE))
    {
      curr_len = ROW_SIZE;
    }
    
    else
    {
      curr_len = rem_len;
    }
      
    // Page size is 64 Bytes so that Row size becomes 4*64 = 256 Bytes
    // max. value of curr_len may be 256 Bytes
          
    // Save existing data if writing something less than ROW_SIZE
    if (curr_len < ROW_SIZE)  
      memcpy(temp_buffer, (uint8_t*) page_start, ROW_SIZE);
  
    // Initialize temp_buffer with curr_len bytes of data
    
    // Check, if first byte of current data is unaligned
    if (page_offset % 2)
    {
      /* Odd start addr - Fill it as a MSB of the corresponding word */
      unalignedWord = 0x00FF | ((uint16_t)*data++ << 8);
      wordPos = (page_offset - 1) / 2;
      temp_buffer[wordPos] = unalignedWord;
      page_offset++;
      curr_len--;
    }

    // Handle if the last byte is not aligned
    if (curr_len % 2)
    {
      /* Odd end addr - Fill it as a LSB of next word */
      unalignedWord = 0xFF00 | data[curr_len - 1];
      wordPos = (page_offset + curr_len - 1) / 2 ;
      temp_buffer[wordPos] = unalignedWord;
      curr_len--;
    }
    
    // Copy the remaining aligned words to buffer
    for (uint8_t index =0; index < curr_len/2; index++)
    {
      wordPos = page_offset / 2;
      alignedWord = *data++;
      alignedWord = alignedWord | (*data++ << 8);
      temp_buffer[wordPos] = alignedWord;
      page_offset += 2;
    }
          
    hwEraseFlashRow(page_start);
    hwWriteFlashPage(page_start, temp_buffer);
    rem_len-=curr_len;
    page_start+= ROW_SIZE;
              
  }  // while (rem_len)

}  
  


// eof flashLoader.c
