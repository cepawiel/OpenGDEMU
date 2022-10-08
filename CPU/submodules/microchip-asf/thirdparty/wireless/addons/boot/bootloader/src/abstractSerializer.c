/**
* \file  abstractSerializer.c
*
* \brief Usage of all declared serial interfaces.
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
#include <srecParser.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
#define UPPER_BYTE_OFFSET          3

/******************************************************************************
                   Global variables section
******************************************************************************/
// interfaces list
static SerialInterface_t  serialInterface[] =
{
  {
    .hwInit   = hwInitUsart0,
    .hwUnInit = hwUnInitUsart0,
    .getByte  = getByteUsart,
    .setByte  = setByteUsart
  }
};

static uint8_t currentInterface;

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Clear interface setting.
******************************************************************************/
void bootUnInitSerializer(void)
{
  serialInterface[currentInterface].hwUnInit();
  lowLevelUnInit();
}

/**************************************************************************//**
\brief Perform search interface activity and handshake message.

\return
  true - handshake was read, false - there is no activity.
******************************************************************************/
bool bootScanInterfaces(void)
{
  bool result;
  uint8_t itr = sizeof(serialInterface) / sizeof(SerialInterface_t);
  uint32_t byteThread;

  //timerIntervalInit();
   uint16_t val = 0xFFFF;
  do {
    itr--;

    serialInterface[itr].hwInit();
   // timerIntervalStart();

    while (val) // wait for about 200 ms
    {
      result = serialInterface[itr].getByte((uint8_t *)&byteThread + UPPER_BYTE_OFFSET);

      if (result) // activity has been found
      {
        if (byteThread == HANDSHAKE_REQ)
          break;

        byteThread >>= 8;
        result = false;
      }
	  val--;
    }

    if (!result)
      serialInterface[itr].hwUnInit();

    currentInterface = itr;
   // timerIntervalStop();

  } while (itr && !result);

  // clear timer
 // timerIntervalUnInit();

  return result;
}

/**************************************************************************//**
\brief Read byte from interface.

\param[out]
  p - pointer to data buffer;
\param[in]
  flag - flag about unlimited or limited reading time.

\return
  true - byte was read, false - byte was not read.
******************************************************************************/
bool bootGetChar(uint8_t *p, bool flag)
{
  bool result;

  do {
    result = serialInterface[currentInterface].getByte(p);
  } while (!result && flag);

  return result;
}

/**************************************************************************//**
\brief Write byte to interface.

\param[in]
  len - data length;
\param[in]
  p - pointer to data buffer.
******************************************************************************/
void bootSetChar(uint16_t len, uint8_t *p)
{
  serialInterface[currentInterface].setByte(len, p);
}

// eof abstractSerializer.c
