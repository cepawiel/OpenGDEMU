/**
* \file  main.c
*
* \brief Serial Bridge Application
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

/**
 * \mainpage Serial Bridge Application
 * \section preface Preface
 * This is the reference manual for Serial Bridge application
 * \section main_files Application Files
 * - main.c      Application main file.
 * \section intro Application Introduction
 * The serial Bridge Application is used in the host which acts as a bridge
 * between the Pc and the NCP device.
 * The serial Bridge application is used by Performance Analyzer application in
 * 2p approach
 * and for flashing image using Bootloader application,where it transfers data
 * from the Pc to the NCP and vice-versa.
 */

/* === INCLUDES ============================================================ */

#include <stdlib.h>
#include "asf.h"
#if SAMD || SAMR21 || SAML21
#include "system.h"
#endif
#include "bootStructure.h"
/* === PROTOTYPES
 *=============================================================== */


/* === GLOBALS ============================================================== */

/* === IMPLEMENTATION ====================================================== */

/**
 * \brief Main function of the Serial Bridge application
 */
int main(void)
{	   
	Boot_main();
}


/* EOF */
