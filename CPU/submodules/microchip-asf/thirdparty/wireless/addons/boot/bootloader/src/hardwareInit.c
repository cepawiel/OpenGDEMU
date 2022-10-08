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
#include <asf.h>
#include <core_cm0plus.h>
#include <types.h>
#include <hardwareInit.h>
#include <srecParser.h>
#include <flashLoader.h>
#if defined(EXT_MEMORY)
  #include <extMemReader.h>
#endif
#include "uartSerializer.h"
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
 //OSCCTRL->DFLLCTRL.bit.ENABLE = 0x1;
 OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE;
 // SYSCTRL_OSC8M_s.enable = 0x1; // Enablign the oscillator
//
//  // Use generic clock generator 0(GCLKMAIN) - internal clock as input source
//  // configure the division for clock generator 0c
 // GCLK_GENDIV_s.id = 0;
  //GCLK_GENDIV_s.div = 0; 
  GCLK->GENCTRL[0].bit.DIV =0;
  GCLK->GENCTRL[0].bit.SRC=0x06;
  GCLK->GENCTRL[0].bit.DIVSEL=0;
//  // then configure internal clock as input source & division enabled

 
 
 GCLK->GENCTRL[1].bit.DIV=12;
 GCLK->GENCTRL[1].bit.DIVSEL=1;
 GCLK->GENCTRL[1].bit.SRC=0x06;
  
}

/**************************************************************************//**
\brief Startup initialization (frequency, io, usb)
******************************************************************************/
void lowLevelInit(void)
{
  //setFrequencyDefault();
  
// enable watchdog for 8 sec.
  //hwWdtInit(WDT_INTERVAL_8000);

/*
#if defined(EXT_MEMORY)
// Point of enter to loader from external memory.
  extLoadDataFromExtMem();
#endif*/


/*
Nvmctrl *const nvm_module = NVMCTRL;
nvm_module->CTRLB.bit.MANW =1;
nvm_module->CTRLB.bit.SLEEPPRM = 0x01;
nvm_module->LOCK.reg =0xFFFF;
*/

//__enable_irq();
	
	system_init();
	/* Initialize the board target resources */
	board_init();
	
	Enable_global_interrupt();
	
	nvm_init();
	
	cpu_irq_enable();
	
}

/**************************************************************************//**
\brief Clear startup initialization parameters to start user application
******************************************************************************/
void lowLevelUnInit(void)
{
 setFrequencyDefault();
 cpu_irq_disable();
 usart_disable0();
 
}

// eof hardwareInit.c
