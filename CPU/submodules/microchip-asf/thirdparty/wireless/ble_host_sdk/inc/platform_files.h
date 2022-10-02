/**
 * \file platform_files.h
 *
 * \brief Platform specific header files inclusion
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
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
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
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef __PLATFORM_FILES_H__
#define __PLATFORM_FILES_H__

/* Platform list */
#define HOST_MCU_SAML21		1

#if (HOST_MCU_DEVICE == HOST_MCU_SAML21)
	#define PACKED		__attribute__((__packed__))
	
	#include "asf.h"
	#include "compiler.h"
	#include "console_serial.h"
	#include "timer_hw.h"
	#include "conf_extint.h"
	#include "ble_utils.h"
	#include "conf_serialdrv.h"
	#include "conf_gpio.h"
	#include "config.h"
	#include "serial_drv.h"
#elif (HOST_MCU_DEVICE == PIC_XX)
	#define PACKED   __attribute__ ((packed))
	#include "stdint.h"
	#include "stdbool.h"
	#include "stdlib.h"
	#include "string.h"
#else
	#error Select a Platform in platform_files.h
#endif /* (HOST_MCU_DEVICE)	*/

#endif /* __PLATFORM_FILES_H__ */