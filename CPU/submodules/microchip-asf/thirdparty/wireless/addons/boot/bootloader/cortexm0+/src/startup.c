/**
* \file  startup.c
*
* \brief Implementation of the start up code.
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

/*******************************************************************************
                   Includes section
*******************************************************************************/
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
#include <atsamr21.h>
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#include <atsamr30.h>
#endif
#include <core_cm0plus.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
#define START_ADDRESS_MASK    0x2FFFFFF8UL

/*******************************************************************************
                     External variables section
*******************************************************************************/
#ifdef  __IAR_SYSTEMS_ICC__
  extern uint32_t __ICFEDIT_vector_start__;
#elif __GNUC__
  extern uint32_t __data;
  extern uint32_t __data_end;
  extern uint32_t __data_load;
  extern uint32_t __bss;
  extern uint32_t __bss_end;
  extern uint32_t __vectors;
#endif

/*******************************************************************************
                   Extern functions prototypes section
*******************************************************************************/
#if __GNUC__
  extern int main(void) __attribute__((noreturn));
#endif

/******************************************************************************
                              Implementations section
******************************************************************************/
#ifdef  __IAR_SYSTEMS_ICC__
/**************************************************************************//**
\brief Specify function for low level initialization. It needs for IAR.
The function must return 1 to initialize variable section

\return
  1 - always.
******************************************************************************/
int __low_level_init(void)
{
  // Remap vector table to real location
  //SCB->VTOR = ((uint32_t)&__ICFEDIT_vector_start__) & START_ADDRESS_MASK;

  return 1; /*  if return 0, the data sections will not be initialized. */
}
#elif __GNUC__
/**************************************************************************//**
\brief Setup environment, do low-level initialization and proceed to main
******************************************************************************/
void gccReset(void)
{
  // Remap vector table to real location
  //SCB->VTOR = ((uint32_t)&__vectors) & START_ADDRESS_MASK;

  // Relocate .data section
  for (uint32_t *from = &__data_load, *to = &__data; to < &__data_end; from++, to++)
  {
    *to = *from;
  }

  // Clear .bss
  for (uint32_t *to = &__bss; to < &__bss_end; to++)
  {
    *to = 0;
  }

  main();
}
#endif

// eof startup.c
