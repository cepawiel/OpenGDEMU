/**
* \file  wdt.c
*
* \brief Implementation of WDT interrupt handler.
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

///******************************************************************************
// *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
// *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
// ******************************************************************************/
///******************************************************************************
//                   Includes section
//******************************************************************************/
//#include <types.h>
//#include <atomic.h>
//#include <core_cm0plus.h>
//#include <wdt.h>
//
///******************************************************************************
//                   Global variables section
//******************************************************************************/
//void wdtTimerHandler(void);
//
///*******************************************************************************
//Starts WDT with interval.
//Parameters:
//  interval - interval.
//Returns:
//  none.
//*******************************************************************************/
//void hwWdtInit(wdtInterval_t interval)
//{
//  GCLK_CLKCTRL_s.id = 0x03; // wdt clock
//  GCLK_CLKCTRL_s.gen = 0x01; // Generic clock generator 1
//  GCLK_CLKCTRL_s.clken = 1;
//
//  ATOMIC_SECTION_ENTER
//  WDT_CTRL_s.enable = 0; // Disabling watchdog
//  WDT_CTRL_s.wen = 0; // disabling window mode
//  WDT_CTRL_s.alwayson = 0; // disabling alwayson mode
// 
//  WDT_INTENCLR_s.ew = 1;
//  WDT_CONFIG_s.per = interval;  
//
//  WDT_CTRL_s.enable = 1; // Enabling watchdog
//  ATOMIC_SECTION_LEAVE
//}
//
///*******************************************************************//**
//\brief Restart the Watch Dog timer
//
//Note Restart the watchdog timer.
//***********************************************************************/
//void hwRestartWdt(void)
//{
//  WDT_CLEAR_s.clear = 0xA5;
//}
//
//
///*******************************************************************************
//Interrupt handler.
//*******************************************************************************/
void wdtTimerHandler(void)
{
//  WDT_CTRL_s.enable = 0;
//  WDT_INTENCLR_s.ew = 0;
//  WDT_CONFIG_s.per = 0;
//  WDT_CTRL_s.enable = 1;
//  for (;;);
}
////eof wdt.c
