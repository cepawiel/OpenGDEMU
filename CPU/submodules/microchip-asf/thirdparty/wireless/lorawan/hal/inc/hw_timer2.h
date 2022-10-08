/**
* \file  hw_timer.h
*
* \brief Wrapper used by sw_timer utility using ASF timer api's
*		
*
* Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries. 
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

#ifndef HW_TIMER_H_INCLUDED
#define HW_TIMER_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/**************************************** INCLUDES*****************************/

#include "asf.h"
#include "stdint.h"

/**************************************** MACROS******************************/
#define HWTIMER_TO_USE_FOR_SYSTEMTIMER   TC0

/***************************************PROTOTYPES**************************/

void HwTimerInit(void);
void HwTimerStart(void);
void HwTimerStop(void);
void HwTimerSetCompare(uint16_t newCompare);
void HwTimerStopCompare(void);
void HwTimerSetOverflowCallback(void *cb);
void HwTimerSetCompareCallback(void *cb);
uint16_t HwTimerGetCount(void);
void HwTimerSetCount(uint16_t count);
#ifdef	__cplusplus
}
#endif
#endif /* HW_TIMER_H_INCLUDED */
