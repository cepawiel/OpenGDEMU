/**
* \file  vectorTable.c
*
* \brief Interrupt vector table.
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
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
#include <atsamr21.h>
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#include <atsamr30.h>
#endif

/******************************************************************************
                   Types section
******************************************************************************/
//! Function prototype for exception table items (interrupt handler).
typedef void(* IntFunc_t)(void);

typedef union
{
  IntFunc_t __fun;
  void *__ptr;
} IntVector_t;

/******************************************************************************
                   Define(s) section
******************************************************************************/


/*******************************************************************************
                   Prototypes section
*******************************************************************************/
extern void lowLevelInit(void);
extern void __iar_program_start(void);
extern void gccReset(void);
static void irqHandlerNotUsed(void);
extern void wdtTimerHandler(void);

/*******************************************************************************
                     External variables section
*******************************************************************************/
extern uint32_t __main_stack_top;

/******************************************************************************
                              Constants section
******************************************************************************/

// IAR really wants this variable to be named __vector_table
#ifdef  __IAR_SYSTEMS_ICC__
  #pragma segment="CSTACK"
  #define STACK_TOP  __sfe("CSTACK")
  #define RESET_ADDR  __iar_program_start

  #pragma language=extended
  #pragma section = ".vectors"
  #pragma location = ".vectors"
#elif __GNUC__
  #define STACK_TOP   &__main_stack_top
  #define RESET_ADDR  gccReset

  __attribute__ ((section(".vectors"), used))
#else
  #error "Unsupported compiler"
#endif
const IntVector_t __vector_table[] =
{
    {.__ptr = STACK_TOP},
    {RESET_ADDR},

    {0},
    {0},
    {0}, {0}, {0}, {0}, {0}, {0}, {0},   /* Reserved */
    {0},
    {0}, {0},                      /*  Reserved */
    {0},
    {0},

    {irqHandlerNotUsed},        /*  0  POWER MANAGER (PM) */
    {irqHandlerNotUsed},        /*  1  SYSTEM CONTROL (SYSCTRL) */
    {wdtTimerHandler},          /*  2  WATCHDOG TIMER */
    {irqHandlerNotUsed},        /*  3  REAL TIME CLOCK */
    {irqHandlerNotUsed},        /*  4  EXTERNAL INTERRUPT CONTROLLER (EIC) */
    {irqHandlerNotUsed},        /*  5  NON-VOLATILE MEMORY CONTROLLER(NVMCTRL) */
    {irqHandlerNotUsed},        /*  6  DMAC_Handler(DMAC) */
    {irqHandlerNotUsed},        /*  7  USB_Handler(USB) */
    {irqHandlerNotUsed},        /*  8  EVENT SYSTEM INTERFACE (EVSYS) */
    {irqHandlerNotUsed},       /*  9  SERIAL COMMUICATION INTERFACE 0 (SERCOM0) */
    {irqHandlerNotUsed},       /*  10  SERIAL COMMUICATION INTERFACE 1 (SERCOM1) */
    {irqHandlerNotUsed},        /*  11  SERIAL COMMUICATION INTERFACE 2 (SERCOM2) */
    {irqHandlerNotUsed},      /*  12  SERIAL COMMUICATION INTERFACE 3 (SERCOM3) */
    {irqHandlerNotUsed},      /*  13  SERIAL COMMUICATION INTERFACE 4 (SERCOM4) */
    {irqHandlerNotUsed},        /*  14  SERIAL COMMUICATION INTERFACE 5 (SERCOM5) */
    {irqHandlerNotUsed},        /*  15 TCC0 */
    {irqHandlerNotUsed},        /*  16 TCC1 */
    {irqHandlerNotUsed},        /*  17 TCC2 */
    {irqHandlerNotUsed},                /*  18 Timer Counter 3 */
    {irqHandlerNotUsed},          /*  19 Timer Counter 4 */
    {irqHandlerNotUsed},        /*  20 Timer Counter 5 */
    {irqHandlerNotUsed},        /*  21 Timer Counter 6 */
    {irqHandlerNotUsed},        /*  22 Timer Counter 7 */
    {irqHandlerNotUsed},        /*  23 ADC controller */
    {irqHandlerNotUsed},        /*  24 Analog Comparator */
    {irqHandlerNotUsed},        /*  25 DAC controller */
    {irqHandlerNotUsed},        /*  26 PTC controller */
    {irqHandlerNotUsed},        /*  27 I2S controller */
    
    {irqHandlerNotUsed}        /*  28 not used */
    
};

/******************************************************************************
                              Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Default interrupt handler for not used irq.
******************************************************************************/
static void irqHandlerNotUsed(void)
{
  while(1);
}
// eof vectorTable.c
