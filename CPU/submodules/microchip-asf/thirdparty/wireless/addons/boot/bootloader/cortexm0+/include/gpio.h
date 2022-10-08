/**
* \file  gpio.h
*
* \brief This module contains a set of functions to manipulate GPIO pins.
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
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/
#ifndef _GPIO_H
#define _GPIO_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
#include <atsamr21.h>
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#include <atsamr30.h>
#endif
/******************************************************************************
                   Define(s) section
******************************************************************************/
/******************************************************************************
* void gpioX_set() sets GPIOX pin to logical 1 level.
* void gpioX_clr() clears GPIOX pin to logical 0 level.
* void gpioX_make_in makes GPIOX pin as input.
* void gpioX_make_in makes GPIOX pin as output.
* uint8_t gpioX_read() returns logical level GPIOX pin.
* uint8_t gpioX_state() returns configuration of GPIOX port.
*******************************************************************************/
#define HW_ASSIGN_PIN(name, port, bit) \
INLINE void  GPIO_##name##_set()           {PORT##port##_OUTSET = (1 << bit); } \
INLINE void  GPIO_##name##_clr()           {PORT##port##_OUTCLR = (1 << bit);} \
INLINE uint8_t  GPIO_##name##_read()       {return (PORT##port##_IN & (1 << bit)) != 0;} \
INLINE uint8_t  GPIO_##name##_state()      {return (PORT##port##_DIR & (1 << bit)) != 0;} \
INLINE void  GPIO_##name##_make_out()      {PORT##port##_DIRSET = (1 << bit);\
                                            PORT##port##_PINCFG##bit |= PORTA_PINCFG##bit##_INEN; } \
INLINE void  GPIO_##name##_make_in()       {PORT##port##_DIRCLR = (1 << bit);   \
                                            PORT##port##_PINCFG##bit |= PORTA_PINCFG##bit##_INEN; \
                                            PORT##port##_PINCFG##bit &= ~PORTA_PINCFG##bit##_PULLEN;  } \
INLINE void  GPIO_##name##_make_pullup()   {PORT##port##_OUTSET = (1 << bit); \
                                            PORT##port##_PINCFG##bit |= PORTA_PINCFG##bit##_PULLEN; }  \
INLINE void  GPIO_##name##_make_pulldown() {PORT##port##_WRCONFIG_s.hwsel = (bit/16) &0x01; \
                                            PORT##port##_WRCONFIG_s.pinmask = (bit%16) & 0xF; \
                                            PORT##port##_WRCONFIG_s.pullen = 0; \
                                            PORT##port##_WRCONFIG_s.wrpincfg = 1; } \
INLINE void  GPIO_##name##_toggle()        {PORT##port##_OUTTGL = (1 << bit);} \
INLINE void  GPIO_##name##_pmuxen(void)    { PORT##port##_PINCFG##bit |= PORTA_PINCFG##bit##_PMUXEN; } \
INLINE void  GPIO_##name##_pmuxdis(void)   { PORT##port##_PINCFG##bit &= ~PORTA_PINCFG##bit##_PMUXEN; } \
INLINE void  GPIO_##name##_reset()         {PORT##port##_PINCFG##bit = 0; } 

/* port - port A, B, C
   bit - bit position 0-32
   funcionality - refer I/O multiplexing
*/
#define HW_ASSIGN_PIN_FUNC(name, port, bit, func) \
INLINE void GPIO_##name##_config_pin()    { \
                                          uint8_t bit_pos = bit/2; \
                                          uint8_t pmux_pos = (bit % 2) ? 4 : 0; \
                                          *(&PORT##port##_PMUX0 + bit_pos) |= (func << pmux_pos) ; \
                                          } \
INLINE void GPIO_##name##_deconfig_pin()    { \
                                          uint8_t bit_pos = bit/2; \
                                          uint8_t pmux_pos = (bit % 2) ? 4 : 0; \
                                          *(&PORT##port##_PMUX0 + bit_pos) &= ~((uint8_t)0xF << pmux_pos) ; \
                                          }

/******************************************************************************
                   Inline static functions section
******************************************************************************/
#endif
//eof gpio.h
