/**
* \file  spiMemInterface.c
*
* \brief Implementation of external spi memory interface.
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
#ifdef EXT_MEMORY
#include <atomic.h>
#include <spiMemInterface.h>
#include <wdt.h>

/******************************************************************************
                   global variables section
******************************************************************************/
SpiChannel_t spiTty;

/******************************************************************************
                   Implementations section
******************************************************************************/
static inline bool hwSpiCtrlBSyncing(SpiChannel_t tty)
{
  return (tty->SYNCBUSY.reg & SC4_SPI_SYNCBUSY_CTRLB);
}
/*****************************************************************************
Inline function to check the SPI ENABLE BUSY status
*****************************************************************************/
static inline bool hwSpiEnableSyncing(SpiChannel_t tty)
{
  return (tty->SYNCBUSY.reg & SC4_SPI_SYNCBUSY_ENABLE);
}
/*****************************************************************************
Inline function to check the SPI ENABLE BUSY status
*****************************************************************************/
static inline bool hwSpiSwrstSyncing(SpiChannel_t tty)
{
  return (tty->SYNCBUSY.reg & SC4_SPI_SYNCBUSY_SWRST);
}
/**************************************************************************//**
\brief Initializes spi.
******************************************************************************/
void spiMemInit(void)
{
  spiTty = SPI_CHANNEL;

  /* Configure PIOs for SPI */
  GPIO_EXT_MEM_CS_make_out();
  GPIO_SPI_MISO_make_in();
  GPIO_SPI_MOSI_make_out();
  GPIO_SPI_SCK_make_out();
  /*Write protect and hold pins are connected to gpio, This has to be pulled up to enable and access flash.*/
#ifdef ATSAMR21E19A
  GPIO_SPI_HOLD_make_out();
  GPIO_SPI_WP_make_out();
  GPIO_SPI_HOLD_set();
  GPIO_SPI_WP_set();
#endif
  GPIO_SPI_MISO_pmuxen();
  GPIO_SPI_MOSI_pmuxen();
  GPIO_SPI_SCK_pmuxen();

  GPIO_SPI_SCK_config_pin();
  GPIO_SPI_MISO_config_pin();
  GPIO_SPI_MOSI_config_pin();

#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  PM_APBCMASK |= (1 << (2 + SPI_SERCOM));

  /* clk settings */
  GCLK_CLKCTRL_s.id = SPI_SERCOM_CLOCK(SPI_SERCOM);
  GCLK_CLKCTRL_s.gen = 0;
  GCLK_CLKCTRL_s.clken = 1;
#endif

#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
  /* Clock Settings */
  GCLK->PCHCTRL[SPI_SERCOM_CLOCK(SPI_SERCOM)].bit.GEN = 0;
  GCLK->PCHCTRL[SPI_SERCOM_CLOCK(SPI_SERCOM)].bit.CHEN = 1;

  MCLK->APBCMASK.reg |= (1 << SPI_SERCOM);
#endif
  /* Reset */
  spiTty->CTRLA.bit.SWRST = 1;
  while(hwSpiSwrstSyncing(spiTty));

  spiTty->CTRLB.reg  = SC4_SPI_CTRLB_RXEN;
  while(hwSpiCtrlBSyncing(spiTty));

  spiTty->BAUD.reg = SPI_CLOCK_RATE_2000;

  /* setup spi mode master, pad & enable it */
  spiTty->CTRLA.reg |=  SC3_SPI_CTRLA_MODE(3) | SC3_SPI_CTRLA_DOPO(SPI_CTRLA_DOPO) | SC3_SPI_CTRLA_DIPO(SPI_CTRLA_DIPO) | SC3_SPI_CTRLA_ENABLE;
  while(hwSpiEnableSyncing(spiTty));

}

/**************************************************************************//**
\brief Writes or reads a length bytes from the external spi memory.

\param[in]
  type   -  transaction type;
\param[in]
  buffer -  pointer to the data buffer;
\param[in]
  length -  number bytes for transfer;
******************************************************************************/
void spiMemTransac(TransactionType_t type, uint8_t *buffer, uint16_t length)
{
  uint16_t i;
  uint8_t temp;

 // hwRestartWdt();

  for (i = 0; i < length; i++)
  {
    ATOMIC_SECTION_ENTER
    spiTty->INTFLAG.reg =0XFF;
    /*Checking data register empty flag to initiate data transfer via spi*/
    while (!(spiTty->INTFLAG.reg & SC4_SPI_INTFLAG_DRE));
    spiTty->DATA.reg = buffer[i];
    while (!(spiTty->INTFLAG.reg & SC4_SPI_INTFLAG_TXC));
    ATOMIC_SECTION_LEAVE

    if (SPI_TRANSACTION_TYPE_WRITE  == type)
    {
      spiTty->INTFLAG.reg =0XFF;
      while (!(spiTty->INTFLAG.reg & SC4_SPI_INTFLAG_DRE));
      temp = spiTty->DATA.reg;
    
    }
    else
    {
      buffer[i] = spiTty->DATA.reg;
    }
  }
  (void)temp;
}

/**************************************************************************//**
\brief Unintializies usart.
******************************************************************************/
void spiMemUnInit(void)
{
//#if (USE_SPI == 1)
//  GPIO_EXT_MEM_CS_set();
//  
//  // disable spi
//  spiTty->CTRLA.bit.ENABLE = 0x00; // disable
//  spiTty->CTRLA.bit.SWRST = 1; 
//  
//  // Disable clock for SPI_SERCOM
//  PM_APBCMASK &= ~(1 << (2 + SPI_SERCOM));
//  
//  
//  
//  
//#endif //(USE_SPI != 1)  
}
#endif //EXT_MEMORY

// eof spiMemInterface.c

