/**
* \file  memAt25dd041b.c
*
* \brief Some commands for at25df041b implementation.
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
#if defined(EXT_MEMORY)
#include <types.h>
#include <extMemReader.h>
#include <spiMemInterface.h>

#if EXTERNAL_MEMORY == AT25DF041B

/******************************************************************************
                   Types section
******************************************************************************/
typedef enum
{
  FLASH_BUSY,
  FLASH_READY
} FlashBusyState_t;

/******************************************************************************
                   Constants section
******************************************************************************/
const uint32_t imageStartAddress[POSITION_MAX] = {IMAGE1_START_ADDRESS, IMAGE2_START_ADDRESS};
static inline FlashBusyState_t memCheckBusyState(void);
/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Check availability of the external flash. Reads vendor and chip ID
from the external flash.

\return
  true - correct memory, \n
  false - other
******************************************************************************/
bool memCheckMem(void)
{
  uint64_t manufacId = RDID;

  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_READ, (uint8_t *)&manufacId, sizeof(uint64_t)-3);
  GPIO_EXT_MEM_CS_set();

  if (MANUFACTURER_ID == (uint8_t)(manufacId >> 8))
    if ((DEVICE_ID_1 == (uint8_t)(manufacId >> 16)) &&
        (DEVICE_ID_2 == (uint8_t)(manufacId >> 24)) &&
        (EXT_STRING_LENGTH == (uint8_t)(manufacId >> 32)))
      return true;

  return false;
}

/**************************************************************************//**
\brief Reads data from memory.

\param[in]
  offset -  internal flash address;
\param[in]
  buffer -  pointer to the data buffer;
\param[in]
  length -  number bytes for reading;
******************************************************************************/
void memReadData(uint32_t offset, uint8_t *data, uint16_t size)
{
  uint8_t instruction = READ;

  offset = LITTLE_TO_BIG_ENDIAN(offset<<8);
  
  while (memCheckBusyState() == FLASH_BUSY);
  
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE , &instruction, sizeof(uint8_t));
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE , (uint8_t *)&offset, sizeof(uint32_t)-1);
  spiMemTransac(SPI_TRANSACTION_TYPE_READ, data, size);
  GPIO_EXT_MEM_CS_set();  // release spi cs
}

#if EXT_MEM_WRITE_API
/**************************************************************************//**
\brief Reads status register from the external flash.
\return status register
******************************************************************************/
static uint8_t memReadStatusRegister(void)
{
  uint16_t regStatus;
  uint8_t cmd = RDSR;

  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, &cmd, sizeof(uint8_t));
  spiMemTransac(SPI_TRANSACTION_TYPE_READ, (uint8_t *) &regStatus, sizeof(uint16_t));
  GPIO_EXT_MEM_CS_set();

  return (uint8_t)(regStatus >> 8);
}


/**************************************************************************//**
\brief Check if the external flash is busy.

\return 
  FLASH_BUSY  \n
  FLASH_READY
******************************************************************************/
static FlashBusyState_t memCheckBusyState(void)
{
  uint8_t statusReg = memReadStatusRegister();

  if (statusReg & RDY)
    return FLASH_BUSY;
 
  return FLASH_READY;
}


/**************************************************************************//**
\brief Sends "write enable" command to the external flash.

\return 
  None
******************************************************************************/
static void memSendWriteEnable(void)
{
  uint8_t wren = WREN;

  while (memCheckBusyState() == FLASH_BUSY);
  
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, &wren, sizeof(uint8_t));
  GPIO_EXT_MEM_CS_set();
}

///**************************************************************************//**
//\brief Starts entire external memory erasing.
//******************************************************************************/
//void memEraseChip(void)
//{
//  uint8_t eraseChip = CHIP_ERASE;
//
//  memSendWriteEnable();
//  
//  while (memCheckBusyState() == FLASH_BUSY);
//  GPIO_EXT_MEM_CS_clr();
//  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *)&eraseChip, sizeof(uint8_t));
//  GPIO_EXT_MEM_CS_set();
//}

/**************************************************************************//**
\brief Erases the specified sector in external memory.
******************************************************************************/
static void memSectorEraseChip(uint32_t offset)
{
  uint32_t address;
  
  // Address can't exceen three bytes
  offset &= 0xFFFFFF;
  
  address = LITTLE_TO_BIG_ENDIAN(offset);
  address |= SECTOR_ERASE;
  
  memSendWriteEnable();
  
  while (memCheckBusyState() == FLASH_BUSY);
  
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *)&address, sizeof(uint32_t));
  GPIO_EXT_MEM_CS_set();
}

/**************************************************************************//**
\brief Write within the specified page in the memory.
******************************************************************************/
static void memProgramPageData (uint32_t offset, uint8_t* data, uint16_t size)
{
  uint32_t address;
  
  // Address can't exceed three bytes
  offset &= 0xFFFFFF;
  
  // Used uint16_t for the "size" argument to handle the corner case 
  // when an entire page needs to be written since the page size = 0x100 
  // can't fit within uint8_t
  
  if (size == 0) 
    return;
  
  if (size > PAGE_SIZE) 
    size = PAGE_SIZE - (offset % PAGE_SIZE);  // Set to remaining area in the given page
    
  address = LITTLE_TO_BIG_ENDIAN(offset);
  address |= PROGRAM;
      
  memSendWriteEnable();
  
  while (memCheckBusyState() == FLASH_BUSY);
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *)&address, sizeof(uint32_t));
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *) data, size);
  GPIO_EXT_MEM_CS_set();  
}

/**************************************************************************//**
\brief pre-erase an image area before flashing
******************************************************************************/
void memPreEraseImageArea(uint32_t offset, uint32_t size)
{
  uint32_t sectorAddr;
  
  // Address can't exceed three bytes
  offset &= 0xFFFFFF;
  
  // Erase the sectors (64 KB) to which this image belongs
  for (sectorAddr = offset; (offset+size) >= (sectorAddr & 0xFF0000); sectorAddr+=SECTOR_SIZE) 
  {
    memSectorEraseChip (sectorAddr);
  }
  
}  

/**************************************************************************//**
\brief Writes data to the external memory.
******************************************************************************/
void memWriteData(uint32_t offset, uint8_t *data, uint16_t size)
{
  uint8_t *dataPtr;
  uint16_t currSize;
  uint32_t address;
  uint16_t pageNum;
  uint8_t pageOffset;
  
  // Address can't exceed three bytes
  offset &= 0xFFFFFF;
  
  if (!size)
    return;
  
  pageNum = (offset / PAGE_SIZE);  
  pageOffset = offset % PAGE_SIZE;
  dataPtr = data;
    
  address = (pageNum << 8) | pageOffset;
  
  if (pageOffset) {
    // Remaining space in the page 
    currSize = PAGE_SIZE - pageOffset;
    
    // Area to be flashed is within this remaining space
    if (size < currSize) {
      memProgramPageData (address, dataPtr, size);  
      return;
    }  
    
    memProgramPageData (address, dataPtr, currSize);
    dataPtr += currSize;
    pageNum++;
  } else {
    currSize = 0;
  }
    
  while (currSize < size) {
    // Loop to erase-write one page at a time
    address = (pageNum << 8);
    
    if ((size - currSize) < PAGE_SIZE) {
      // Last page
      memProgramPageData (address, dataPtr, (size - currSize));  
      return;
    }
      
    memProgramPageData (address, dataPtr, PAGE_SIZE);
      
    // Update loop counters
    dataPtr += PAGE_SIZE;
    pageNum++;
    currSize += PAGE_SIZE;   
  }  
    
}

#endif  // EXT_MEM_WRITE_API

#endif // EXTERNAL_MEMORY == AT25DF041B
#endif //EXT_MEM
// eof memAt25df041a.c
