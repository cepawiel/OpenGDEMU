/**
* \file  timerInterval.h
*
* \brief Implementation of timer interval interface.
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

#ifndef _TIMERINTERVAL_H
#define _TIMERINTERVAL_H

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
#include <fcpu.h>
#include <gpio.h>


/******************************************************************************
                   Define(s) section
******************************************************************************/
#define COUNTER_300MS   300ul
#define COUNTER_600MS   600ul
#define COUNTER_800MS   800ul
#define COUNTER_1000MS  1000ul
#define DIV_1024        1024
#define DIV_64          64

/** \brief frequency prescaler for system timer - 8MHz */
#define TIMER_CLOCK_SOURCE   6 /* OSC8M - Internal */

#define SERIAL_TIMER_FREQUENCY_PRESCALER        5     /*DIV_64*/
#define RF_TIMER_FREQUENCY_PRESCALER      7     /*DIV_1024*/
      
               
/** \brief timer counter top value */
#define SERIAL_TIMER_COUNTER_VALUE  ((F_CPU/1000ul) / DIV_64) * COUNTER_300MS

/** \brief timer counter top value */
#define RF_TIMER_COUNTER_VALUE  ((F_CPU/1000ul) / DIV_1024) * COUNTER_600MS

/******************************************************************************
                     Inline static functions section
******************************************************************************/
/******************************************************************************
 Polling the Sync. flag for register access
 Parameters:
   none
 Returns:
   none
 *****************************************************************************/
INLINE void halTimerSync(void)
{
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  while (TC3_16_STATUS_s.syncbusy);
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
  while (TC1->COUNT16.SYNCBUSY.bit.STATUS);
#endif
}
/**************************************************************************//**
\brief Timer initialization (interval 200ms).
******************************************************************************/
static void timerIntervalInit(uint16_t top_time, uint8_t prescalar)
{
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  /* Enable the TC3 in PM */
  PM_APBCMASK_s.tc3 = 1;

  GCLK_CLKCTRL_s.id = 0x1b;  // enabling clock for TC3
  GCLK_CLKCTRL_s.gen = 0;
  GCLK_CLKCTRL_s.clken = 1;

  /* configure the timer */
  TC3_16_CTRLA_s.mode = 0x00;  //16 bit mode
  halTimerSync();
  TC3_16_CTRLA_s.wavegen = 1;  // match frequency
  halTimerSync();
  TC3_16_CTRLA_s.prescaler = prescalar;
//  switch (prescalar)
//  {
//    case 1:
//      TC3_16_CTRLA_s.prescaler = 0;  
//      break;
//      
//    case 2:
//      TC3_16_CTRLA_s.prescaler = 1;  
//      break;
//      
//    case 4:
//      TC3_16_CTRLA_s.prescaler = 2;  
//      break;
//      
//    case 8:
//      TC3_16_CTRLA_s.prescaler = 3;  
//      break;
//      
//    case 16:
//      TC3_16_CTRLA_s.prescaler = 4;  
//      break;
//      
//    case 64:
//      TC3_16_CTRLA_s.prescaler = 5;  
//      break;
//      
//    case 256:
//      TC3_16_CTRLA_s.prescaler = 6;  
//      break;
//      
//    case 1024:
//      TC3_16_CTRLA_s.prescaler = 7;  
//      break;
//  }
  
  halTimerSync();
  TC3_16_CTRLA_s.prescsync = 0x01; // PRESC
  halTimerSync();
  TC3_16_CTRLBCLR_s.oneshot = 1; // clearing one shot mode
  halTimerSync();
  TC3_16_CTRLBCLR_s.dir = 1; 
  halTimerSync();

  TC3_16_COUNT = (uint16_t)0;
  halTimerSync();
  TC3_16_CC0 = (uint16_t)top_time;
  halTimerSync();
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A)) 
  /* Disable Timer */
  TC1->COUNT16.CTRLA.bit.ENABLE = 0;
  halTimerSync();
  
  /* Enabling clock for TC1 */
  MCLK->APBCMASK.bit.TC1_ = 1;  

  /* Disable Channel */
  GCLK->PCHCTRL[0x1b].bit.CHEN = 0;
  while (GCLK->PCHCTRL[0x1b].bit.CHEN);

  /* Set Generator and Enable Channel */
  GCLK->PCHCTRL[0x1b].bit.GEN = 0;
  GCLK->PCHCTRL[0x1b].bit.CHEN = 1;

  /* Configure the timer */
  TC1->COUNT16.CTRLA.bit.MODE = 0; //16 bit mode
  halTimerSync();
  TC1->COUNT16.WAVE.bit.WAVEGEN = 1; // match frequency
  halTimerSync();
  TC1->COUNT16.CTRLA.bit.PRESCALER = prescalar;
  halTimerSync();
  TC1->COUNT16.CTRLA.bit.PRESCSYNC = 0x01;
  halTimerSync();
  TC1->COUNT16.CTRLBCLR.bit.ONESHOT = 1; // disabling one shot mode
  halTimerSync();
  TC1->COUNT16.CTRLBCLR.bit.DIR = 1; //counting down 
  halTimerSync();
  TC1->COUNT16.COUNT.bit.COUNT = 0;
  halTimerSync();
  TC1->COUNT16.CC[0].bit.CC = top_time;
  halTimerSync();
#endif
}

/**************************************************************************//**
\brief Clear timer settings.
******************************************************************************/
//static void timerIntervalUnInit(void)
//{
//  /*  Reset the timer */
//  TC3_16_CTRLA_s.swrst = 0x00;  
//  halTimerSync();
// 
//  /* Disable the TC3 in PM */
//  PM_APBCMASK_s.tc3 = 0;
//}

/**************************************************************************//**
\brief Start timer interval.
******************************************************************************/
static void timerIntervalStart(void)
{
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  TC3_16_CTRLA_s.enable = 1; //after configuration enable the timer
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
  /* Clear Interrupt Flag */
  TC1->COUNT16.INTENCLR.bit.MC0 = 0x01;
  /* Enable Timer */
  TC1->COUNT16.CTRLA.bit.ENABLE = 1;
#endif
  halTimerSync();
}

/**************************************************************************//**
\brief Stop timer.
******************************************************************************/
static void timerIntervalStop(void)
{
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  /* Disable the clock */
  TC3_16_CTRLA_s.enable = 0;
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
  /* Disable Timer */
  TC1->COUNT16.CTRLA.bit.ENABLE = 0;
#endif
  halTimerSync();
}

/**************************************************************************//**
\brief Return expired state.

\return
  true - timer expired; \n
  false - otherwise.
******************************************************************************/
static bool timerReadIntervalState(void)
{
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
  if (TC3_16_INTFLAG_s.mc0)
  {
    TC3_16_INTFLAG_s.mc0 = 1;
    return true;
  }
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
  if (TC1->COUNT16.INTFLAG.bit.MC0)
  {
    /* Clear Interrupt Flag */
	TC1->COUNT16.INTENCLR.bit.MC0 = 0x01;
    return true;
  }
#endif
  return false;
}

#endif /* _TIMERINTERVAL_H */
