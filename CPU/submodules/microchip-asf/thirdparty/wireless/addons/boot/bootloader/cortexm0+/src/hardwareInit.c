/**
* \file  hardwareInit.c
*
* \brief Hardware initialization routine.
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
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
#include <atsamr21.h>
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#include <atsamr30.h>
#endif
#include <core_cm0plus.h>
#include <types.h>
#include <hardwareInit.h>
#include <srecParser.h>
#include <wdt.h>
#include <flashLoader.h>
#ifdef SAM0
#include <app_info.h>
#endif
#if defined(EXT_MEMORY)
  #include <extMemReader.h>
#endif

/******************************************************************************
                   Define(s) section
******************************************************************************/
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
                   Prototypes section
******************************************************************************/
static void setFrequencyDefault(void);

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Set up default value
******************************************************************************/
static void setFrequencyDefault(void)
{
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  SYSCTRL_OSC8M_s.presc = 0x0; // Prescaler - 1
  SYSCTRL_OSC8M_s.enable = 0x1; // Enablign the oscillator

//  // Use generic clock generator 0(GCLKMAIN) - internal clock as input source
//  // configure the division for clock generator 0
//  GCLK_GENDIV_s.id = 0;
//  GCLK_GENDIV_s.div = 0;
//
//  // then configure internal clock as input source & division enabled
//  GCLK_GENCTRL_s.id = 0;
//  GCLK_GENCTRL_s.src = 0x06;  // OSC8M
//  GCLK_GENCTRL_s.divsel = 0;  // no division so 8MHz clock

  GCLK_GENDIV_s.id = 1;
  GCLK_GENDIV_s.div = 12;  // divide by 8192

  // then configure internal clock as input source & division enabled
  GCLK_GENCTRL_s.id = 1;
  GCLK_GENCTRL_s.src = 0x06;  // OSC8M
  GCLK_GENCTRL_s.divsel = 1;  // division of 8MHz clock
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))

  /* Clock Configuration */
  OSCCTRL->OSC16MCTRL.bit.FSEL = 0x01; //8Mhz selection
  OSCCTRL->OSC16MCTRL.reg |= OSCCTRL_OSC16MCTRL_ENABLE;

  /* Set Divide Register */
  GCLK->GENCTRL[1].bit.DIV = 12; // to get 8Mhz
  /* Configure internal clock as input source & division enabled */
  GCLK->GENCTRL[1].bit.SRC = 0x06;//OSC16M oscillator output
  GCLK->GENCTRL[1].bit.DIVSEL = 1; //division enable
#endif
}

/**************************************************************************//**
\brief Startup initialization (frequency, io, usb)
******************************************************************************/
void lowLevelInit(void)
{

  setFrequencyDefault();

  // enable watchdog for 8 sec.
  // hwWdtInit(WDT_INTERVAL_8000);

  //extLoadDataFromExtMem ();  

#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  /* enable manual write */
  NVMCTRL_CTRLB_s.manw = 1;
  /* configure power reduction mode - WAKEUPINSTANT */
  NVMCTRL_CTRLB_s.sleepprm = 0x01;
  /* Unlock all the regions */
  NVMCTRL_LOCK = 0xFFFF;
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
  NVMCTRL->CTRLB.bit.MANW = 1;
  NVMCTRL->CTRLB.bit.SLEEPPRM = 0x01;
  NVMCTRL->LOCK.reg = 0xFFFF;
#endif

  /* Enable interrupt */
  __enable_irq();
  
  
}

/**************************************************************************//**
\brief Clear startup initialization parameters to start user application
******************************************************************************/
void lowLevelUnInit(void)
{
  setFrequencyDefault();
  
  /* Disable interrupt */
  __disable_irq();
}

void system_reset(void)
{
  __DSB();                                                     /* Ensure all outstanding memory accesses included
                                                                  buffered write are completed before reset */
  SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
                SCB_AIRCR_SYSRESETREQ_Msk );
  __DSB();                                                     /* Ensure completion of memory access */
  while(1);  
}


// eof hardwareInit.c
