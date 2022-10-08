/**
* \file  types.h
*
* \brief The header file describes global system types and pre-processor words
*        which depends on compiler or platform
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

#ifndef _TYPES_H
#define _TYPES_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "asf.h"
#if defined(__ICCAVR__) || defined(__ICCARM__)

#define INLINE static inline
 #define CLEAR_EIND()
     #define memcpy_P   memcpy
#elif defined(__GNUC__)


    #define PROGMEM_DECLARE(x) x
    #define FLASH_VAR
    #define FLASH_PTR
    #define memcpy_P   memcpy
    #define hw_platform_address_size_t    uint32_t
    #define BEGIN_PACK
    #define END_PACK
    #define PACK __attribute__ ((packed))
    #define CLEAR_EIND()
  #define INLINE static inline __attribute__ ((always_inline))
  #define NOP       asm volatile ("nop")
#else
  #error 'Compiler not supported.'
#endif /* Compilier types */

#endif /* _TYPES_H */
// eof types.h
