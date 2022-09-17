/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) %copyright_year%, Atmel Corporation                                        */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

#include "sam3u.h"
#include <string.h>

/* Initialize segments */
extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _sstack;
extern uint32_t _estack;

/** \cond DOXYGEN_SHOULD_SKIP_THIS */
int main(void);
/** \endcond */

void __libc_init_array(void);

/* Default empty handler */
void Dummy_Handler(void);

/* Cortex-M3 core handlers */
void NMI_Handler        ( void ) {
        asm("bkpt");
        while(1) {};
};

void __attribute__((optimize("O0"))) HardFault_Handler () {

    uint32_t BFAR __attribute__((unused)) = (*((volatile unsigned long *)(0xE000ED38)));
    uint32_t CFSR __attribute__((unused)) = (*((volatile unsigned long *)(0xE000ED28)));
    uint32_t HFSR __attribute__((unused)) = (*((volatile unsigned long *)(0xE000ED2C)));
    uint32_t DFSR __attribute__((unused)) = (*((volatile unsigned long *)(0xE000ED30)));
    uint32_t AFSR __attribute__((unused)) = (*((volatile unsigned long *)(0xE000ED3C)));
//   printf ("SCB_SHCSR = %x\n", SCB->SHCSR);
  
  asm("bkpt");
  while (1);
}

void MemManage_Handler  ( void ) {
        asm("bkpt");
        while(1) {};
};

void BusFault_Handler   ( void ) {
        asm("bkpt");
        while(1) {};
};

void UsageFault_Handler ( void ) {
        asm("bkpt");
        while(1) {};
};

// void SVC_Handler        ( void ) {
//         asm("bkpt");
//         while(1) {};
// };

void DebugMon_Handler   ( void ) {
        asm("bkpt");
        while(1) {};
};

// void PendSV_Handler     ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
// void SysTick_Handler    ( void ) {
//         asm("bkpt");
//         while(1) {};
// };

/* Peripherals handlers */
void SUPC_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RSTC_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTT_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void WDT_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PMC_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EFC0_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#ifdef _SAM3U_EFC1_INSTANCE_
void EFC1_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif /* _SAM3U_EFC1_INSTANCE_ */
void UART_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PIOA_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PIOB_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#ifdef _SAM3U_PIOC_INSTANCE_
void PIOC_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif /* _SAM3U_PIOC_INSTANCE_ */
void USART0_Handler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USART1_Handler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USART2_Handler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#ifdef _SAM3U_USART3_INSTANCE_
void USART3_Handler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif /* _SAM3U_USART3_INSTANCE_ */
void HSMCI_Handler  ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TWI0_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TWI1_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SSC_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC0_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC1_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC2_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC12B_Handler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UDPHS_Handler  ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/* Exception Table */
__attribute__ ((section(".vectors")))
const DeviceVectors exception_table = {

        /* Configure Initial Stack Pointer, using linker-generated symbols */
        .pvStack = (void*) (&_estack),

        .pfnReset_Handler      = (void*) Reset_Handler,
        .pfnNMI_Handler        = (void*) NMI_Handler,
        .pfnHardFault_Handler  = (void*) HardFault_Handler,
        .pfnMemManage_Handler  = (void*) MemManage_Handler,
        .pfnBusFault_Handler   = (void*) BusFault_Handler,
        .pfnUsageFault_Handler = (void*) UsageFault_Handler,
        .pfnReserved1_Handler  = (void*) (0UL),          /* Reserved */
        .pfnReserved2_Handler  = (void*) (0UL),          /* Reserved */
        .pfnReserved3_Handler  = (void*) (0UL),          /* Reserved */
        .pfnReserved4_Handler  = (void*) (0UL),          /* Reserved */
        .pfnSVC_Handler        = (void*) SVC_Handler,
        .pfnDebugMon_Handler   = (void*) DebugMon_Handler,
        .pfnReserved5_Handler  = (void*) (0UL),          /* Reserved */
        .pfnPendSV_Handler     = (void*) PendSV_Handler,
        .pfnSysTick_Handler    = (void*) SysTick_Handler,

        /* Configurable interrupts */
        .pfnSUPC_Handler   = (void*) SUPC_Handler,   /* 0  Supply Controller */
        .pfnRSTC_Handler   = (void*) RSTC_Handler,   /* 1  Reset Controller */
        .pfnRTC_Handler    = (void*) RTC_Handler,    /* 2  Real Time Clock */
        .pfnRTT_Handler    = (void*) RTT_Handler,    /* 3  Real Time Timer */
        .pfnWDT_Handler    = (void*) WDT_Handler,    /* 4  Watchdog Timer */
        .pfnPMC_Handler    = (void*) PMC_Handler,    /* 5  Power Management Controller */
        .pfnEFC0_Handler   = (void*) EFC0_Handler,   /* 6  Enhanced Embedded Flash Controller 0 */
#ifdef _SAM3U_EFC1_INSTANCE_
        .pfnEFC1_Handler   = (void*) EFC1_Handler,   /* 7  Enhanced Embedded Flash Controller 1 */
#else
        .pvReserved7       = (void*) (0UL),          /* 7  Reserved */
#endif /* _SAM3U_EFC1_INSTANCE_ */
        .pfnUART_Handler   = (void*) UART_Handler,   /* 8  Universal Asynchronous Receiver Transmitter */
        .pvReserved9       = (void*) (0UL),          /* 9  Reserved */
        .pfnPIOA_Handler   = (void*) PIOA_Handler,   /* 10 Parallel I/O Controller A, */
        .pfnPIOB_Handler   = (void*) PIOB_Handler,   /* 11 Parallel I/O Controller B */
#ifdef _SAM3U_PIOC_INSTANCE_
        .pfnPIOC_Handler   = (void*) PIOC_Handler,   /* 12 Parallel I/O Controller C */
#else
        .pvReserved12      = (void*) (0UL),          /* 12 Reserved */
#endif /* _SAM3U_PIOC_INSTANCE_ */
        .pfnUSART0_Handler = (void*) USART0_Handler, /* 13 USART 0 */
        .pfnUSART1_Handler = (void*) USART1_Handler, /* 14 USART 1 */
        .pfnUSART2_Handler = (void*) USART2_Handler, /* 15 USART 2 */
#ifdef _SAM3U_USART3_INSTANCE_
        .pfnUSART3_Handler = (void*) USART3_Handler, /* 16 USART 3 */
#else
        .pvReserved16      = (void*) (0UL),          /* 16 Reserved */
#endif /* _SAM3U_USART3_INSTANCE_ */
        .pfnHSMCI_Handler  = (void*) HSMCI_Handler,  /* 17 High Speed Multimedia Card Interface */
        .pfnTWI0_Handler   = (void*) TWI0_Handler,   /* 18 Two-Wire Interface 0 */
        .pfnTWI1_Handler   = (void*) TWI1_Handler,   /* 19 Two-Wire Interface 1 */
        .pfnSPI_Handler    = (void*) SPI_Handler,    /* 20 Serial Peripheral Interface */
        .pfnSSC_Handler    = (void*) SSC_Handler,    /* 21 Synchronous Serial Controller */
        .pfnTC0_Handler    = (void*) TC0_Handler,    /* 22 Timer Counter 0 */
        .pfnTC1_Handler    = (void*) TC1_Handler,    /* 23 Timer Counter 1 */
        .pfnTC2_Handler    = (void*) TC2_Handler,    /* 24 Timer Counter 2 */
        .pfnPWM_Handler    = (void*) PWM_Handler,    /* 25 Pulse Width Modulation Controller */
        .pfnADC12B_Handler = (void*) ADC12B_Handler, /* 26 12-bit ADC Controller */
        .pfnADC_Handler    = (void*) ADC_Handler,    /* 27 10-bit ADC Controller */
        .pfnDMAC_Handler   = (void*) DMAC_Handler,   /* 28 DMA Controller */
        .pfnUDPHS_Handler  = (void*) UDPHS_Handler   /* 29 USB Device High Speed */
};

#ifdef UMM_ENABLE
extern void umm_init_heap(void *ptr, size_t size);
uint32_t heap_mem[4 * 1024];
#endif

/**
 * \brief This is the code that gets called on processor reset.
 * To initialize the device, and call the main() routine.
 */
void __attribute__((optimize("O0"))) __attribute__ ((__noreturn__)) Reset_Handler(void)
{
    asm("ldr sp, =_estack");

    // uint32_t *pSrc, *pDest;
 
    // // Copy Required Sections to RAM
    // uint32_t * src = &_etext;
    // uint32_t * dest = &_srelocate;
    uint32_t s = (uint32_t) &_srelocate;
    uint32_t e = (uint32_t) &_erelocate;
    uint32_t relocateSize = e - s;
    memcpy(&_srelocate, &_etext, relocateSize);
    // while(dest <= &_srelocate) {
    //     *dest++ = *src++;
    // }

    /* Clear the zero segment */
    for(uint32_t * pos = &_szero; pos < &_ezero; pos += sizeof(uint32_t)) {
        *pos = 0;
        if(*pos != 0) 
            asm("bkpt");
    }


    /* Set the vector table base address */
    uint32_t vecAddr = (uint32_t) &exception_table;
//     uint32_t vecAddr = (uint32_t) &_sfixed;
    SCB->VTOR = ((uint32_t) vecAddr & SCB_VTOR_TBLOFF_Msk);

#ifdef UMM_ENABLE
    umm_init_heap(&heap_mem, sizeof(heap_mem));
#endif

    // Initialize the C library, calls C++ constructors
    __libc_init_array();

    uint32_t resetReason __attribute__((unused)) = (RSTC->RSTC_SR >> 8) & 0xF;

    /* Branch to main function */
    main();

    /* Infinite loop */
    for(;;);
}

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Dummy_Handler(void)
{
    asm("bkpt");
    while (1) {}
}
