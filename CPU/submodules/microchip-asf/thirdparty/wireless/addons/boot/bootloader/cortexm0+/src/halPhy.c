/**
* \file  halPhy.c
*
* \brief ATSAMD21 PHY interface implementation
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


#if (USE_RF == 1)
/*- Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
// #include "halPhy.h"
#include "phy.h"
// #include "halTimer.h"

/*- Implementations --------------------------------------------------------*/
// spi channel
typedef SercomSpi* SpiChannel_t;

/*************************************************************************//**
*****************************************************************************/
uint8_t halPhySpiWriteByte(uint8_t value)
{
  return halPhySpiWriteByteInline(value);
}

/*************************************************************************//**
*****************************************************************************/
void halPhyReset(void)
{
  GPIO_PHY_RST_clr();
  halDelay(HAL_RESET_DELAY);
  GPIO_PHY_RST_set();
}

/*************************************************************************//**
*****************************************************************************/
static void halPhySpiSync(void)
{
  while (SERCOM4->SPI.SYNCBUSY.reg);
}

/*************************************************************************//**
*****************************************************************************/
void halPhyInit(void)
{
    
  // reset SPI
  halPhyReset ();
  
  // Configure PIOs for SPI
  GPIO_PHY_SLP_TR_make_out();
  GPIO_PHY_SLP_TR_clr();
  GPIO_PHY_RST_make_out();
  GPIO_PHY_IRQ_make_in();
  GPIO_PHY_IRQ_pmuxen();
  GPIO_PHY_CS_make_out();
  GPIO_PHY_MISO_make_in();
  GPIO_PHY_MISO_pmuxen();
  GPIO_PHY_MOSI_make_out();
  GPIO_PHY_MOSI_pmuxen();
  GPIO_PHY_SCK_make_out();
  GPIO_PHY_SCK_pmuxen();

  // Configure SPI in peripheral multiplexing
  PORTB_PMUX15 = PORTB_PMUX15_PMUXE(PORTB_PMUX15_PMUXE_VALUE);
  PORTC_PMUX9 = PORTC_PMUX9_PMUXE(PORTC_PMUX9_PMUXE_VALUE) | PORTC_PMUX9_PMUXO(PORTC_PMUX9_PMUXO_VALUE);

  // Enable the clock of SPI
  PM_APBCMASK |= PM_APBCMASK_SERCOM4;

  // Configure the generic clk settings
  GCLK_CLKCTRL_s.id    = CLKCTRL_SERCOM_ID;
  GCLK_CLKCTRL_s.gen   = CLKCTRL_GENERTR_VAL;
  GCLK_CLKCTRL_s.clken = CLKCTRL_GENERTR_CLKEN;
  
  // Reset
  SC4_SPI_CTRLA = SC4_SPI_CTRLA_SWRST;
  while(halSpiSwrstSyncing());

  SC4_SPI_CTRLB  = SC4_SPI_CTRLB_RXEN;
  while(halSpiCtrlBSyncing());

  // setup spi mode master, pad & enable it
  SC4_SPI_CTRLA |=  SC4_SPI_CTRLA_MODE(SERCOM4_SPI_CTRLA_MODE_VAL) | SC4_SPI_CTRLA_DOPO(SERCOM4_SPI_CTRLA_DOPO_VAL) | SC4_SPI_CTRLA_ENABLE;
  while(halSpiEnableSyncing());
  
/*    
#if F_CPU <= 16000000
  SC4_SPI_BAUD = 0;
#elif F_CPU <= 32000000
  SC4_SPI_BAUD = 1;
#elif F_CPU <= 48000000
  SC4_SPI_BAUD = 2;
#else
  #error Unsupported frequency
#endif
*/
  
  halPhySpiSync();
  
}


void halPhyUnInit(void)
{
  
  SERCOM4->SPI.CTRLA.reg = 0;
  halPhySpiSync();
  
  // Reset
  SC4_SPI_CTRLA = SC4_SPI_CTRLA_SWRST;
  while(halSpiSwrstSyncing());
  
  //PM_APBCMASK &= ~PM_APBCMASK_SERCOM4;
  
  //PM->APBCMASK.reg = 0;//&= ~(1<<PM_APBCMASK_SERCOM4);

}
#endif
/*************************************************************************//**
*****************************************************************************/

 




