/**
* \file  bootloader.c
*
* \brief Implementation of bootloader state machine.
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
#include <bootStructure.h>
#include <abstractSerializer.h>
#include <hardwareInit.h>
#include "app_info.h"
#include "appEntry.h"

boot_info_t bInfo = {
	.crc =0,
	.majorRev = 0x0001,
	.minorRev = 0x0001,
	.reservedRev = 0x0000,
	.supported_features.mask = 0,
	.supported_features.bits.multiApps = FEATURE_MULTI_APPS,
	.supported_features.bits.imgSecurity = 0,
	.supported_features.bits.rfInterface = 0,
	.supported_features.bits.usart0Interface = 1,
	.supported_features.bits.usart1Interface = 0,
	.supported_features.bits.spiInterface = 0,
	.supported_features.bits.twiInterface = 0,
	.supported_features.bits.usbInterface = 0,
	.supported_features.bits.reserved = 0,
	.start_address = 0x00000000,
	.bprot_size = 0x0003,     // 8 KB
	.mcu_flash_size = TWO_POWER_EXPONENT_16(FLASH_SIZE_IN_KB),
	.ext_flash_size = TWO_POWER_EXPONENT_16(EXT_MEM_SIZE_IN_KB),
	.reserved = 0x00
};


uint64_t flag __attribute__ ((section (".noinit")));
/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief State machine in the main
******************************************************************************/
int Boot_main(void)
{

if(flag == RESET_FLAG)
{
	jumpToApplication();
}
lowLevelInit();
 //update_boot_info();
app_infoAction();
  srecHandshake();
  srecProtocol();
  lowLevelUnInit();
  
 flag = RESET_FLAG;
  system_reset();
 
 jumpToApplication();
  return 1;
}

// eof bootloader.c
