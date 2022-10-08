/**
* \file  memAt25f2048.c
*
* \brief Some commands for at25f2048 implementation.
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

#if EXTERNAL_MEMORY == AT25F2048

/******************************************************************************
                   Constants section
******************************************************************************/
const uint32_t imageStartAddress[POSITION_MAX] = {IMAGE1_START_ADDRESS, IMAGE2_START_ADDRESS};

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
  uint32_t manufacId = RDID;

  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_READ, (uint8_t *)&manufacId, sizeof(uint32_t)-1);
  GPIO_EXT_MEM_CS_set();

  if (MANUFACTURER_ID == (uint8_t)(manufacId >> 8))
    if (DEVICE_ID == (uint8_t)(manufacId >> 16))
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
  GPIO_EXT_MEM_CS_clr();
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE , &instruction, sizeof(uint8_t));
  spiMemTransac(SPI_TRANSACTION_TYPE_WRITE , (uint8_t *)&offset, sizeof(uint32_t)-1);
  spiMemTransac(SPI_TRANSACTION_TYPE_READ, data, size);
  GPIO_EXT_MEM_CS_set();  // release spi cs
}

#endif // EXTERNAL_MEMORY == AT25F2048

// eof memAt25f2048.c
