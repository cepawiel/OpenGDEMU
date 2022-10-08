/**
* \file  preparser.c
*
* \brief Implementation of converting byte stream to string for common \n
*        bootloader part.
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
#include <srecParser.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
#define SIZE_PREBUFFER    64
#define ASCII_0          0x30
#define ASCII_8          0x38
#define ASCII_9          0x39
#define ASCII_A          0x41
#define DIFF_ASCII       0x37

/******************************************************************************
                   Prototypes section
******************************************************************************/
static uint8_t asciiToDig(uint8_t symbol);

/******************************************************************************
                   External global variables section
******************************************************************************/

/******************************************************************************
                   Global variables section
******************************************************************************/
static uint8_t preBuffer[SIZE_PREBUFFER];
static uint8_t placed;
static uint8_t poR;
bool srecFinished = false;

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Waiting for a field of the type from the internal address

\param[in]
  stream - byte stream from usb;
\param[in]
  size - size of the converted stream;
******************************************************************************/
void convertStreamToNumbers(uint8_t *stream, uint8_t size)
{
  static bool highNibble = true;
  static bool srecTypePart = false;
  uint8_t temp;
  uint8_t in, out = 0;

  for (in = 0; in < size; in++)
  {
    if (S_SYMBOL == stream[in])
    {
      preBuffer[out++] = stream[in];
      srecTypePart = true;
      continue;
    }

    if (srecTypePart)
    {
      if (ASCII_8 == stream[in])
      { // we've caught S8 string, change to S0 to avoid standard srec restart procedure
        stream[in] = ASCII_0;
        srecFinished = true;
      }
      preBuffer[out++] = stream[in];
      srecTypePart = false;
      continue;
    }

    temp = asciiToDig(stream[in]);
    if (highNibble)
    {
      preBuffer[out] = temp << 4;
      highNibble = false;
    }
    else
    {
      highNibble = true;
      preBuffer[out++] |= temp;
    }
  }

  placed = out;
}

/**************************************************************************//**
\brief Convert ascii to digital

\param[in]
  symbol - ascii code;

\return
  digital
******************************************************************************/
static uint8_t asciiToDig(uint8_t symbol)
{
  if (symbol <= ASCII_9)
    return symbol - ASCII_0;

  if (symbol >= ASCII_A)
    return symbol - DIFF_ASCII;

  return 0;
}

/**************************************************************************//**
\brief Read data byte from preparser buffer

\param[in]
  data - pointer to memory for data byte;

\return
  true - there is data byte;
  false - preparser buffer is empty
******************************************************************************/
bool readFromPreBuffer(uint8_t *data)
{
  if (!placed)
  {
    poR = 0;
    return false;
  }

  *data = preBuffer[poR++];
  placed--;
  return true;
}

// eof preparser.c
