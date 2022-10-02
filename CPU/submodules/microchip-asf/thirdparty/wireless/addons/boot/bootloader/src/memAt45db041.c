/**
* \file  memAt45db041.c
*
* \brief Some commands for at45db041 implementation.
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
#include <extMemReader.h>
#include <spiMemInterface.h>


/******************************************************************************
                   Define(s) section
******************************************************************************/
#if EXTERNAL_MEMORY == AT45DB041
#define ALL_DATA_HAS_BEEN_SAVED   0xFFFFFFFFul
#define ATOMIC_COUNTING           128

/******************************************************************************
                   Types section
******************************************************************************/
typedef enum
{
  FLASH_BUSY,
  FLASH_READY
} FlashBusyState_t;

/******************************************************************************
                   Globals section
******************************************************************************/
// const uint32_t imageStartAddress[POSITION_MAX] = {IMAGE1_START_ADDRESS, IMAGE2_START_ADDRESS};
// static uint32_t flushedPageAddr = ALL_DATA_HAS_BEEN_SAVED;

/******************************************************************************
                   Prototypes section
******************************************************************************/
#if EXT_MEM_WRITE_API
static uint8_t memReadStatusRegister(void);
static FlashBusyState_t memCheckBusyState(void);
static void memClearInternalBuffer(void);
static void memWriteInternalBuffer(uint8_t offset, uint8_t* data, uint16_t size);
static void memModifyWrite(uint32_t addr, uint8_t* data, uint8_t size);
static void memProgramBufferToPage (uint16_t pageNumber);
#endif

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Check availability of the external flash. Reads status register and
checks unchangable bits.

\return
  true - correct memory, \n
  false - other
******************************************************************************/
bool memCheckMem(void)
{
  uint16_t regStatus = RDSR;
  uint32_t pageMode = 0xA6802A3D;  // page size = 256 bytes
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_READ, (uint8_t *)&regStatus, sizeof(uint16_t));
  GPIO_EXT_MEM_CS_set();
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *)&pageMode, sizeof(uint32_t));
  GPIO_EXT_MEM_CS_set();
  if (STATUS_UNCHANGED_BITS == ((uint8_t)(regStatus >> 8) & STATUS_UNCHANGED_BIT_MASK))
    return true;
  
  return false;
}

/**************************************************************************//**
\brief Reads data from memory.

\param[in]
  offset -  internal flash address;
\param[in]
  buffer -  pointer to the output data buffer;
\param[in]
  size -  number bytes for reading;
******************************************************************************/
void memReadData(uint32_t offset, uint8_t *data, uint16_t size)
{
  uint32_t pageAddr;
  uint32_t byteAddr;
  
  // Address can't exceed three bytes
  offset &= 0xFFFFFF;

  pageAddr = offset / PAGE_SIZE;
  byteAddr = offset % PAGE_SIZE;
  offset = byteAddr | (pageAddr << PAGE_SHIFT) | ((uint32_t)READ << 24);
  offset = LITTLE_TO_BIG_ENDIAN(offset);
  
  while (memCheckBusyState() == FLASH_BUSY);
  
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE , (uint8_t *)&offset, sizeof(uint32_t));
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
  spiMemTransac(SPI_TRANSACTION_TYPE_READ, (uint8_t *)&regStatus, sizeof(uint16_t));
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
    return FLASH_READY;

  return FLASH_BUSY;
}

//#if (EXTERNAL_MEMORY == AT45DB041E)
///**************************************************************************//**
//\brief Erase an entire chip
//******************************************************************************/
//void memEraseChip(void)
//{
//    
//  uint32_t eraseChip = CHIP_ERASE;
//  
//  while (memCheckBusyState() == FLASH_BUSY);
//  
//  GPIO_EXT_MEM_CS_clr();
//  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *)&eraseChip, sizeof(uint32_t));
//  GPIO_EXT_MEM_CS_set();
//}
//#endif

/**************************************************************************//**
\brief pre-erase an image area before flashing
******************************************************************************/
void memPreEraseImageArea(uint32_t offset, uint32_t size)
{
  // Do nothing since we do built-in erase in this chip
  
}  

/**************************************************************************//**
\brief Clears internal flash buffer.
******************************************************************************/
static void memClearInternalBuffer(void)
{
  uint32_t address = 0ul;
  uint64_t data;
  uint8_t itr;

  while (FLASH_BUSY == memCheckBusyState());
  
  memset((uint8_t *)&data, 0xFF, sizeof(uint64_t));

  address |= WRITE_BUF1; 
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *)&address, sizeof(uint32_t));
  for (itr = 0; itr < (PAGE_SIZE / sizeof(uint64_t)); itr++)
     spiMemTransac(SPI_TRANSACTION_TYPE_WRITE , (uint8_t *)&data, sizeof(uint64_t));
  GPIO_EXT_MEM_CS_set();
}

/**************************************************************************//**
\brief Write to the page-sized internal flash buffer.
******************************************************************************/
static void memWriteInternalBuffer(uint8_t offset, uint8_t* data, uint16_t size)
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
  
  while (memCheckBusyState() == FLASH_BUSY);
  
  address = LITTLE_TO_BIG_ENDIAN(offset);
  address |= WRITE_BUF1;
      
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *)&address, sizeof(uint32_t));
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *) data, size);
  GPIO_EXT_MEM_CS_set();   
}

/**************************************************************************//**
\brief Read-modify-write to mem using buffer-1.
******************************************************************************/
static void memModifyWrite(uint32_t addr, uint8_t* data, uint8_t size)
{
  uint32_t address;
  
  while (memCheckBusyState() == FLASH_BUSY);
  
  address = LITTLE_TO_BIG_ENDIAN(addr & 0xFFFFFF);
  address |= MODIFY_BUF1;
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *)&address, sizeof(uint32_t));
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *) data, size);
  GPIO_EXT_MEM_CS_set();   
}

/**************************************************************************//**
\brief Program internal memory buffer to the specified page in mem, with built-in erase.
******************************************************************************/
void memProgramBufferToPage (uint16_t pageNumber)
{
  uint32_t address;
  while (memCheckBusyState() == FLASH_BUSY);
  
  address = LITTLE_TO_BIG_ENDIAN(pageNumber << PAGE_SHIFT);
  address |= PROGRAM_BUF1;
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE, (uint8_t *)&address, sizeof(uint32_t));
  GPIO_EXT_MEM_CS_set();
  
}

/**************************************************************************//**
\brief Writes data to memory.

\param[in]
  offset -  flash address;
\param[in]
  data -  pointer to the input data buffer;
\param[in]
  size -  number of bytes for writing;
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
      memModifyWrite (address, dataPtr, size);  
      return;
    }  
    
    memModifyWrite (address, dataPtr, currSize);
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
      memModifyWrite (address, dataPtr, (size - currSize));  
      return;
    }
    
    // Flash a full page  
    memClearInternalBuffer();
    memWriteInternalBuffer (address, dataPtr, PAGE_SIZE);
    memProgramBufferToPage (pageNum);
    
    // Update loop counters
    dataPtr += PAGE_SIZE;
    pageNum++;
    currSize += PAGE_SIZE;   
  }  
    
}
#endif

#endif  // EXTERNAL_MEMORY == AT45DB041

// eof memAt45db041.c
