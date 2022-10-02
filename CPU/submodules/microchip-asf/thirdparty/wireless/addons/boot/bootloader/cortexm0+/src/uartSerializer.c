/**
* \file  uartSerializer.c
*
* \brief Implementation of uart serialize interface.
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
#include <uartSerializer.h>
#include <wdt.h>
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
#include <atsamr21.h>
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#include <atsamr30.h>
#endif

#if (USE_USART0 == 1) || (USE_USART1 == 1)

/******************************************************************************
                   Define(s) section
******************************************************************************/
/**************************************************************************//**
 \brief Sets USART U2Xn bit value.
******************************************************************************/
#ifndef USART_DOUBLE_SPEED
  #define USART_DOUBLE_SPEED 1ul
#endif

// usart channel
typedef SercomUsart* UsartChannel_t;

/******************************************************************************
                   Global variables section
******************************************************************************/
UsartChannel_t usartTty;

/******************************************************************************
                   Prototypes section
******************************************************************************/
#ifndef BOARD_FAKE
static void hwInitCommon(uint8_t index);
//static void hwUnInitCommon(uint8_t index);
static void hwUartSwrstSync(UsartChannel_t usartTty);
static void hwUartEnableSync(UsartChannel_t usartTty);
static void hwUartCtrlbSync(UsartChannel_t usartTty);

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Startup initialization of the usart.
******************************************************************************/
static void hwInitCommon (uint8_t index)
{
  usartTty->CTRLA.bit.SWRST = 1; //reset the USART 
  hwUartSwrstSync(usartTty);

  usartTty->CTRLA.bit.MODE = 0x01; //USART with internal clk
  usartTty->CTRLA.bit.DORD = 0x01; //data order LSB first
  
  switch (index) {
    
#if (USE_USART0 == 1)
    case 0:
      usartTty->CTRLA.bit.RXPO = USART0_RX_PAD;
      usartTty->CTRLA.bit.TXPO = USART0_TX_PAD;
      break;
#endif

#if (USE_USART1 == 1)
    case 1:  
      usartTty->CTRLA.bit.RXPO = USART1_RX_PAD;
      usartTty->CTRLA.bit.TXPO = USART1_TX_PAD;
      break;
#endif

    default: 
      break;
  
  }
      

  usartTty->INTENCLR.reg = (uint8_t)0xFF;

  /* Set the baud rate */  
  usartTty->BAUD.reg = ((uint64_t)65536ul * (F_CPU - 16ul * BAUD_RATE) / F_CPU);

  /* Enable Transmitter and Receiver and set the data size */
  usartTty->CTRLB.bit.TXEN  = 1;
  usartTty->CTRLB.bit.RXEN  = 1;
  hwUartCtrlbSync(usartTty);

  /* Enable the Usart */
  usartTty->CTRLA.bit.ENABLE = 1;
  hwUartEnableSync(usartTty);
}

/**************************************************************************//**
\brief Clear startup initialization parameters to start user application
******************************************************************************/
//static void hwUnInitCommon (uint8_t index)
//{
//  UsartChannel_t tty;
//
//  // Select appropriate channel
//  switch (index) {
//    
//    case 0:
//      tty = USART_CHANNEL_0;
//      break;
//
//    case 1:  
//      tty = USART_CHANNEL_1;
//      break;
//
//    default: 
//      break;
//  
//  }
//    
//  // disable uart
//
//  tty->CTRLA.bit.SWRST = 1; //reset the USART 
//  hwUartSwrstSync(tty);
//  
//  tty->CTRLA.bit.ENABLE = 0;
//  hwUartEnableSync(tty);
// 
//  PM_APBCMASK &= ~(1 << (2 + USART0_SERCOM));
//  PM_APBCMASK &= ~(1 << (2 + USART1_SERCOM));
//}

/**************************************************************************//**
\brief Receive byte on uart.

\param[out]
  p - pointer to byte memory;

\return
    true - there is received byte; \n
    false - there is not received byte;
******************************************************************************/
bool getByteUsart(uint8_t *p)
{
  if (usartTty->INTFLAG.reg & SC1_USART_INTFLAG_RXC)
  {
    if (!(usartTty->INTFLAG.reg & SC1_USART_INTFLAG_ERROR))
    {
      *p = usartTty->DATA.reg;
      return true;
    }
  }
  return false;
}

/**************************************************************************//**
\brief Transmit byte to uart.

\param[in]
  len - number of bytes to transmit;
\param[in]
  p - pointer to byte memory;
******************************************************************************/
void setByteUsart(uint16_t len, uint8_t *p)
{
  //hwRestartWdt();

  while (len--)
  {
    /* prepare transmission */
    while(!(usartTty->INTFLAG.reg & SC1_USART_INTFLAG_DRE));
    usartTty->DATA.reg = *p++;
    /* wait for byte to send */
    while (!(usartTty->INTFLAG.reg & SC1_USART_INTFLAG_TXC));
    /* clear the flag */   
  }
}

#if (USE_USART0 == 1)
/**************************************************************************//**
\brief Startup initialization of the usart0.
******************************************************************************/
void hwInitUsart0(void)
{
  usartTty =  USART_CHANNEL_0;

  GPIO_USART0_TXD_make_out();
  GPIO_USART0_RXD_make_in();
  GPIO_USART0_TXD_pmuxen();
  GPIO_USART0_RXD_pmuxen();
    
  GPIO_USART0_TXD_config_pin();
  GPIO_USART0_RXD_config_pin();

#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  /* clk settings */
  GCLK_CLKCTRL_s.id = USART_SERCOM_CLOCK(USART0_SERCOM);
  GCLK_CLKCTRL_s.gen = 0;
  GCLK_CLKCTRL_s.clken = 1;

  /* enable the clock of USART */
  PM_APBCMASK |= (1 << (2 + USART0_SERCOM));
#endif

#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A)) 
  /* Clock Settings */
  GCLK->PCHCTRL[USART_SERCOM_CLOCK(USART0_SERCOM)].bit.GEN = 0;
  GCLK->PCHCTRL[USART_SERCOM_CLOCK(USART0_SERCOM)].bit.CHEN = 1;

  MCLK->APBCMASK.reg |= (1 << USART0_SERCOM);
#endif

  hwInitCommon(0);
}

/**************************************************************************//**
\brief Clear startup initialization parameters to start user application
******************************************************************************/
void hwUnInitUsart0(void)
{
//	GPIO_USART0_TXD_make_in();
//  GPIO_USART0_TXD_reset();
//  GPIO_USART0_RXD_reset();
////  GPIO_USART0_TXD_pmuxdis();
////  GPIO_USART0_RXD_pmuxdis();
//  GPIO_USART0_TXD_deconfig_pin();
//  GPIO_USART0_RXD_deconfig_pin();
//
//  hwUnInitCommon(0);
}
#endif // (USE_USART0 == 1)

#if (USE_USART1 == 1)
/**************************************************************************//**
\brief Startup initialization of the usart1.
******************************************************************************/
void hwInitUsart1(void)
{
  usartTty =  USART_CHANNEL_1;
  /* Configure Port Pins for USART0 */
  GPIO_USART1_TXD_make_out();
  GPIO_USART1_TXD_pmuxen();
  GPIO_USART1_RXD_make_in();
  GPIO_USART1_RXD_pmuxen();

  GPIO_USART1_TXD_config_pin();
  GPIO_USART1_RXD_config_pin();

#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  /* clk settings */
  GCLK_CLKCTRL_s.id = USART_SERCOM_CLOCK(USART1_SERCOM);
  GCLK_CLKCTRL_s.gen = 0;
  GCLK_CLKCTRL_s.clken = 1;

  /* enable the clock of USART */
  PM_APBCMASK |= (1 << (2 + USART1_SERCOM));
#endif

#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
  /* Clock Settings */
  GCLK->PCHCTRL[USART_SERCOM_CLOCK(USART1_SERCOM)].bit.GEN = 0;
  GCLK->PCHCTRL[USART_SERCOM_CLOCK(USART1_SERCOM)].bit.CHEN = 1;

  MCLK->APBCMASK.reg |= (1 << USART1_SERCOM);
#endif

  hwInitCommon(1);
}

/**************************************************************************//**
\brief Clear startup initialization parameters to start user application
******************************************************************************/
void hwUnInitUsart1(void)
{
//	GPIO_USART1_TXD_make_in();
//  GPIO_USART1_TXD_reset();
//  GPIO_USART1_RXD_reset();
////  GPIO_USART1_TXD_pmuxdis();
////  GPIO_USART1_RXD_pmuxdis();
//  GPIO_USART1_TXD_deconfig_pin();
//  GPIO_USART1_RXD_deconfig_pin();
//
//  hwUnInitCommon(1);
}
#endif // (USE_USART1 == 1)

/**************************************************************************//**
\brief Checking for the Sync. flag for USART register access.

  \param
    descriptor - USART module descriptor.
  \return
   none.
******************************************************************************/
static void hwUartSwrstSync(UsartChannel_t usartTty)
{
  while (usartTty->SYNCBUSY.reg & SC1_USART_SYNCBUSY_SWRST);
}
/**************************************************************************//**
  \brief Checking for the Sync. flag for USART register access.

  \param
    descriptor - USART module descriptor.
  \return
   none.
******************************************************************************/
static void hwUartEnableSync(UsartChannel_t usartTty)
{
  while (usartTty->SYNCBUSY.reg & SC1_USART_SYNCBUSY_ENABLE);
}
/**************************************************************************//**
  \brief Checking for the Sync. flag for USART register access.

  \param
    descriptor - USART module descriptor.
  \return
   none.
******************************************************************************/
static void hwUartCtrlbSync(UsartChannel_t usartTty)
{
  while (usartTty->SYNCBUSY.reg & SC1_USART_SYNCBUSY_CTRLB);
}

#elif defined BOARD_FAKE
void hwInitUsart0(void)
{
}
void hwInitUsart1(void)
{
}
void hwUnInitUsart0(void)
{
}
void hwUnInitUsart1(void)
{
}
void setByteUsart(uint16_t len, uint8_t *p)
{
  len = len;
  p= p;
}
bool getByteUsart(uint8_t *p)
{
  return p;
}
#else
#endif

#endif // (USE_USART0 == 1) || (USE_USART1 == 1)

// eof uartSerializer.c
