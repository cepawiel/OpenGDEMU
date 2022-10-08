/**
* \file  hw_timer.c
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

/**************************************** INCLUDES*****************************/
#include "hw_timer.h"

struct tc_module tcModule;

void HwTimerInit(void)
{
	struct tc_config tcConfig;
	tc_get_config_defaults(&tcConfig);

	/*
	* Set clock source and timer prescaler to generate 1us tick.
	*/
	tcConfig.clock_source = GCLK_GENERATOR_0;
	tcConfig.clock_prescaler = TC_CLOCK_PRESCALER_DIV8;

	/* this is a 16-bit timer */
	tcConfig.counter_size = TC_COUNTER_SIZE_16BIT;

	/*
	* this timer is running in a repeating mode
	* i.e, 0,1,2,...,UINT16_MAX,0,1,2,...
	*/
	tcConfig.oneshot = false;

	/* Use TC0 hardware timer for system timer */
	tc_init(&tcModule, HWTIMER_TO_USE_FOR_SYSTEMTIMER, &tcConfig);
}

void HwTimerStart(void)
{
	tc_enable(&tcModule);
}

void HwTimerStop(void)
{
	tc_disable(&tcModule);
}

uint16_t HwTimerGetCount(void)
{
	return tc_get_count_value(&tcModule);
}

void HwTimerSetCount(uint16_t count)
{
	tc_set_count_value(&tcModule, count);
}

void HwTimerSetCompare(uint16_t newCompare)
{
	tc_set_compare_value(&tcModule, TC_COMPARE_CAPTURE_CHANNEL_0, newCompare);
	tc_enable_callback(&tcModule, TC_CALLBACK_CC_CHANNEL0);
}

void HwTimerStopCompare(void)
{
	tc_disable_callback(&tcModule, TC_CALLBACK_CC_CHANNEL0);
}

void HwTimerSetOverflowCallback(void *cb)
{
	tc_register_callback(&tcModule, cb, TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tcModule, TC_CALLBACK_OVERFLOW);
}

inline void HwTimerSetCompareCallback(void *cb)
{
	tc_register_callback(&tcModule, cb, TC_CALLBACK_CC_CHANNEL0);
}

/* eof hw_timer.c */
