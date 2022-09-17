; * ---------------------------------------------------------------------------------------
; *  @file:    startup_MKL28T7_CORE1.s
; *  @purpose: CMSIS Cortex-M0P Core Device Startup File
; *            MKL28T7_CORE1
; *  @version: 1.7
; *  @date:    2015-5-16
; *  @build:   b151119
; * ---------------------------------------------------------------------------------------
; *
; * Copyright (c) 1997 - 2015 , Freescale Semiconductor, Inc.
; * All rights reserved.
; *
; * Redistribution and use in source and binary forms, with or without modification,
; * are permitted provided that the following conditions are met:
; *
; * o Redistributions of source code must retain the above copyright notice, this list
; *   of conditions and the following disclaimer.
; *
; * o Redistributions in binary form must reproduce the above copyright notice, this
; *   list of conditions and the following disclaimer in the documentation and/or
; *   other materials provided with the distribution.
; *
; * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
; *   contributors may be used to endorse or promote products derived from this
; *   software without specific prior written permission.
; *
; * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
; * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
; * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; *
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; *****************************************************************************/


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size
                IMPORT  |Image$$ARM_LIB_STACK$$ZI$$Limit|

__Vectors       DCD     |Image$$ARM_LIB_STACK$$ZI$$Limit| ; Top of Stack
                DCD     Reset_Handler  ; Reset Handler
                DCD     NMI_Handler                         ;NMI Handler
                DCD     HardFault_Handler                   ;Hard Fault Handler
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     SVC_Handler                         ;SVCall Handler
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     PendSV_Handler                      ;PendSV Handler
                DCD     SysTick_Handler                     ;SysTick Handler

                                                            ;External Interrupts
                DCD     DMA1_04_IRQHandler                  ;DMA1 channel 0/4 transfer complete
                DCD     DMA1_15_IRQHandler                  ;DMA1 channel 1/5 transfer complete
                DCD     DMA1_26_IRQHandler                  ;DMA1 channel 2/6 transfer complete
                DCD     DMA1_37_IRQHandler                  ;DMA1 channel 3/7 transfer complete
                DCD     CTI1_DMA1_Error_IRQHandler          ;CTI1 or DMA1 error
                DCD     FLEXIO0_IRQHandler                  ;FLEXIO0
                DCD     TPM0_IRQHandler                     ;TPM0 single interrupt vector for all sources
                DCD     TPM1_IRQHandler                     ;TPM1 single interrupt vector for all sources
                DCD     Reserved24_IRQHandler               ;Reserved interrupt
                DCD     LPIT1_IRQHandler                    ;LPIT1 interrupt
                DCD     LPSPI0_IRQHandler                   ;LPSPI0 single interrupt vector for all sources
                DCD     LPSPI1_IRQHandler                   ;LPSPI1 single interrupt vector for all sources
                DCD     LPUART0_IRQHandler                  ;LPUART0 status and error
                DCD     LPUART1_IRQHandler                  ;LPUART1 status and error
                DCD     LPI2C0_IRQHandler                   ;LPI2C0 interrupt
                DCD     LPI2C1_IRQHandler                   ;LPI2C1 interrupt
                DCD     MU0_B_IRQHandler                    ;MU 0 Side B interrupt
                DCD     PORTM_IRQHandler                    ;PORTM Pin detect
                DCD     TRNG_IRQHandler                     ;TRNG
                DCD     TSI0_IRQHandler                     ;TSI0
                DCD     DAC0_IRQHandler                     ;DAC0
                DCD     CMP1_IRQHandler                     ;CMP1
                DCD     LLWU1_IRQHandler                    ;Low leakage wakeup 1
                DCD     I2S0_IRQHandler                     ;I2S0 interrupt
                DCD     USB0_IRQHandler                     ;USB0 interrupt
                DCD     ADC0_IRQHandler                     ;ADC0 interrupt
                DCD     LPTMR1_IRQHandler                   ;LPTMR1 interrupt
                DCD     RTC_Seconds_IRQHandler              ;RTC seconds
                DCD     INTMUX1_0_IRQHandler                ;INTMUX1_0 interrupt
                DCD     INTMUX1_1_IRQHandler                ;INTMUX1_1 interrupt
                DCD     INTMUX1_2_IRQHandler                ;INTMUX1_2 interrupt
                DCD     INTMUX1_3_IRQHandler                ;INTMUX1_3 interrupt
                DCD     LPTMR0_IRQHandler                   ;LPTMR0 interrupt (INTMUX source IRQ0)
                DCD     LPIT0_IRQHandler                    ;LPIT0 interrupt (INTMUX source IRQ1)
                DCD     TPM2_IRQHandler                     ;TPM2 single interrupt vector for all sources (INTMUX source IRQ2)
                DCD     Reserved51_IRQHandler               ;Reserved interrupt
                DCD     LPSPI2_IRQHandler                   ;LPSPI2 single interrupt vector for all sources (INTMUX source IRQ4)
                DCD     LPUART2_IRQHandler                  ;LPUART2 status and error (INTMUX source IRQ5)
                DCD     EMVSIM0_IRQHandler                  ;EMVSIM0 interrupt (INTMUX source IRQ6)
                DCD     LPI2C2_IRQHandler                   ;LPI2C2 interrupt (INTMUX source IRQ7)
                DCD     Reserved56_IRQHandler               ;Reserved interrupt
                DCD     PMC_IRQHandler                      ;PMC interrupt (INTMUX source IRQ9)
                DCD     FTFA_IRQHandler                     ;FTFA interrupt (INTMUX source IRQ10)
                DCD     SCG_IRQHandler                      ;SCG interrupt (INTMUX source IRQ11)
                DCD     XRDC_IRQHandler                     ;XRDC interrupt (INTMUX source IRQ12)
                DCD     Reserved61_IRQHandler               ;Reserved interrupt
                DCD     Reserved62_IRQHandler               ;Reserved interrupt
                DCD     RCM_IRQHandler                      ;RCM interrupt (INTMUX source IRQ15)
                DCD     CMP0_IRQHandler                     ;CMP0 interrupt (INTMUX source IRQ16)
                DCD     Reserved65_IRQHandler               ;Reserved interrupt
                DCD     RTC_IRQHandler                      ;RTC Alarm interrupt (INTMUX source IRQ18)
                DCD     PORTA_IRQHandler                    ;PORTA Pin detect (INTMUX source IRQ19)
                DCD     PORTB_IRQHandler                    ;PORTB Pin detect (INTMUX source IRQ20)
                DCD     PORTC_IRQHandler                    ;PORTC Pin detect (INTMUX source IRQ21)
                DCD     PORTD_IRQHandler                    ;PORTD Pin detect (INTMUX source IRQ22)
                DCD     PORTE_IRQHandler                    ;PORTE Pin detect (INTMUX source IRQ23)
                DCD     DMA0_04_IRQHandler                  ;DMA0 channel 0/4 transfer complete (INTMUX source IRQ24)
                DCD     DMA0_15_IRQHandler                  ;DMA0 channel 1/5 transfer complete (INTMUX source IRQ25)
                DCD     DMA0_26_IRQHandler                  ;DMA0 channel 2/6 transfer complete (INTMUX source IRQ26)
                DCD     DMA0_37_IRQHandler                  ;DMA0 channel 3/7 transfer complete (INTMUX source IRQ27)
                DCD     DMA0_Error_IRQHandler               ;DMA0 error (INTMUX source IRQ28)
                DCD     Reserved77_IRQHandler               ;Reserved interrupt
                DCD     Reserved78_IRQHandler               ;Reserved interrupt
                DCD     Reserved79_IRQHandler               ;Reserved interrupt
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

; <h> Flash Configuration
;   <i> 16-byte flash configuration field that stores default protection settings (loaded on reset)
;   <i> and security information that allows the MCU to restrict access to the FTFL module.
;   <h> Backdoor Comparison Key
;     <o0>  Backdoor Comparison Key 0.  <0x0-0xFF:2>
;     <o1>  Backdoor Comparison Key 1.  <0x0-0xFF:2>
;     <o2>  Backdoor Comparison Key 2.  <0x0-0xFF:2>
;     <o3>  Backdoor Comparison Key 3.  <0x0-0xFF:2>
;     <o4>  Backdoor Comparison Key 4.  <0x0-0xFF:2>
;     <o5>  Backdoor Comparison Key 5.  <0x0-0xFF:2>
;     <o6>  Backdoor Comparison Key 6.  <0x0-0xFF:2>
;     <o7>  Backdoor Comparison Key 7.  <0x0-0xFF:2>
BackDoorK0      EQU     0xFF
BackDoorK1      EQU     0xFF
BackDoorK2      EQU     0xFF
BackDoorK3      EQU     0xFF
BackDoorK4      EQU     0xFF
BackDoorK5      EQU     0xFF
BackDoorK6      EQU     0xFF
BackDoorK7      EQU     0xFF
;   </h>
;   <h> Program flash protection bytes (FPROT)
;     <i> Each program flash region can be protected from program and erase operation by setting the associated PROT bit.
;     <i> Each bit protects a 1/32 region of the program flash memory.
;     <h> FPROT0
;       <i> Program Flash Region Protect Register 0
;       <i> 1/32 - 8/32 region
;       <o.0>   FPROT0.0
;       <o.1>   FPROT0.1
;       <o.2>   FPROT0.2
;       <o.3>   FPROT0.3
;       <o.4>   FPROT0.4
;       <o.5>   FPROT0.5
;       <o.6>   FPROT0.6
;       <o.7>   FPROT0.7
nFPROT0         EQU     0x00
FPROT0          EQU     nFPROT0:EOR:0xFF
;     </h>
;     <h> FPROT1
;       <i> Program Flash Region Protect Register 1
;       <i> 9/32 - 16/32 region
;       <o.0>   FPROT1.0
;       <o.1>   FPROT1.1
;       <o.2>   FPROT1.2
;       <o.3>   FPROT1.3
;       <o.4>   FPROT1.4
;       <o.5>   FPROT1.5
;       <o.6>   FPROT1.6
;       <o.7>   FPROT1.7
nFPROT1         EQU     0x00
FPROT1          EQU     nFPROT1:EOR:0xFF
;     </h>
;     <h> FPROT2
;       <i> Program Flash Region Protect Register 2
;       <i> 17/32 - 24/32 region
;       <o.0>   FPROT2.0
;       <o.1>   FPROT2.1
;       <o.2>   FPROT2.2
;       <o.3>   FPROT2.3
;       <o.4>   FPROT2.4
;       <o.5>   FPROT2.5
;       <o.6>   FPROT2.6
;       <o.7>   FPROT2.7
nFPROT2         EQU     0x00
FPROT2          EQU     nFPROT2:EOR:0xFF
;     </h>
;     <h> FPROT3
;       <i> Program Flash Region Protect Register 3
;       <i> 25/32 - 32/32 region
;       <o.0>   FPROT3.0
;       <o.1>   FPROT3.1
;       <o.2>   FPROT3.2
;       <o.3>   FPROT3.3
;       <o.4>   FPROT3.4
;       <o.5>   FPROT3.5
;       <o.6>   FPROT3.6
;       <o.7>   FPROT3.7
nFPROT3         EQU     0x00
FPROT3          EQU     nFPROT3:EOR:0xFF
;     </h>
;   </h>
;   <h> Flash nonvolatile option byte (FOPT)
;     <i> Allows the user to customize the operation of the MCU at boot time.
;     <o.0> LPBOOT0
;       <0=> Core and system clock divider (OUTDIV1) is 0x7 (divide by 8) when LPBOOT1=0 or 0x1 (divide by 2) when LPBOOT1=1.
;       <1=> Core and system clock divider (OUTDIV1) is 0x3 (divide by 4) when LPBOOT1=0 or 0x0 (divide by 1) when LPBOOT1=1.
;     <o.1> BOOTPIN_OPT
;       <0=> Force Boot from ROM if BOOTCFG0 asserted, where BOOTCFG0 is the boot config function which is muxed with NMI pin
;       <1=> Boot source configured by FOPT (BOOTSRC_SEL) bits
;     <o.2> NMI_DIS
;       <0=> NMI interrupts are always blocked
;       <1=> NMI_b pin/interrupts reset default to enabled
;     <o.3> RESET_PIN_CFG
;       <0=> RESET pin is disabled following a POR and cannot be enabled as reset function
;       <1=> RESET_b pin is dedicated
;     <o.4> LPBOOT1
;       <0=> Core and system clock divider (OUTDIV1) is 0x7 (divide by 8) when LPBOOT0=0 or 0x3 (divide by 4) when LPBOOT0=1.
;       <1=> Core and system clock divider (OUTDIV1) is 0x1 (divide by 2) when LPBOOT0=0 or 0x0 (divide by 1) when LPBOOT0=1.
;     <o.5> FAST_INIT
;       <0=> Slower initialization
;       <1=> Fast Initialization
;     <o.6..7> BOOTSRC_SEL
;       <0=> Boot from Flash
;       <2=> Boot from ROM
;       <3=> Boot from ROM
;         <i> Boot source selection
FOPT          EQU     0x3D
;   </h>
;   <h> Flash security byte (FSEC)
;     <i> WARNING: If SEC field is configured as "MCU security status is secure" and MEEN field is configured as "Mass erase is disabled",
;     <i> MCU's security status cannot be set back to unsecure state since Mass erase via the debugger is blocked !!!
;     <o.0..1> SEC
;       <2=> MCU security status is unsecure
;       <3=> MCU security status is secure
;         <i> Flash Security
;     <o.2..3> FSLACC
;       <2=> Freescale factory access denied
;       <3=> Freescale factory access granted
;         <i> Freescale Failure Analysis Access Code
;     <o.4..5> MEEN
;       <2=> Mass erase is disabled
;       <3=> Mass erase is enabled
;     <o.6..7> KEYEN
;       <2=> Backdoor key access enabled
;       <3=> Backdoor key access disabled
;         <i> Backdoor Key Security Enable
FSEC          EQU     0xFE
;   </h>
; </h>
                IF      :LNOT::DEF:RAM_TARGET
                AREA    FlashConfig, DATA, READONLY
__FlashConfig
                DCB     BackDoorK0, BackDoorK1, BackDoorK2, BackDoorK3
                DCB     BackDoorK4, BackDoorK5, BackDoorK6, BackDoorK7
                DCB     FPROT0    , FPROT1    , FPROT2    , FPROT3
                DCB     FSEC      , FOPT      , 0xFF      , 0xFF
                ENDIF


                AREA    |.text|, CODE, READONLY

; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                IF      :LNOT::DEF:RAM_TARGET
                REQUIRE FlashConfig
                ENDIF

                CPSID   I               ; Mask interrupts
                LDR     R0, =0xE000ED08
                LDR     R1, =__Vectors
                STR     R1, [R0]
                LDR     R0, =SystemInit
                BLX     R0
                CPSIE   i               ; Unmask interrupts
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)
NMI_Handler\
                PROC
                EXPORT  NMI_Handler         [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler\
                PROC
                EXPORT  SVC_Handler         [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler         [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler         [WEAK]
                B       .
                ENDP
DMA1_04_IRQHandler\
                PROC
                EXPORT  DMA1_04_IRQHandler         [WEAK]
                LDR     R0, =DMA1_04_DriverIRQHandler
                BX      R0
                ENDP

DMA1_15_IRQHandler\
                PROC
                EXPORT  DMA1_15_IRQHandler         [WEAK]
                LDR     R0, =DMA1_15_DriverIRQHandler
                BX      R0
                ENDP

DMA1_26_IRQHandler\
                PROC
                EXPORT  DMA1_26_IRQHandler         [WEAK]
                LDR     R0, =DMA1_26_DriverIRQHandler
                BX      R0
                ENDP

DMA1_37_IRQHandler\
                PROC
                EXPORT  DMA1_37_IRQHandler         [WEAK]
                LDR     R0, =DMA1_37_DriverIRQHandler
                BX      R0
                ENDP

CTI1_DMA1_Error_IRQHandler\
                PROC
                EXPORT  CTI1_DMA1_Error_IRQHandler         [WEAK]
                LDR     R0, =CTI1_DMA1_Error_DriverIRQHandler
                BX      R0
                ENDP

FLEXIO0_IRQHandler\
                PROC
                EXPORT  FLEXIO0_IRQHandler         [WEAK]
                LDR     R0, =FLEXIO0_DriverIRQHandler
                BX      R0
                ENDP

LPSPI0_IRQHandler\
                PROC
                EXPORT  LPSPI0_IRQHandler         [WEAK]
                LDR     R0, =LPSPI0_DriverIRQHandler
                BX      R0
                ENDP

LPSPI1_IRQHandler\
                PROC
                EXPORT  LPSPI1_IRQHandler         [WEAK]
                LDR     R0, =LPSPI1_DriverIRQHandler
                BX      R0
                ENDP

LPUART0_IRQHandler\
                PROC
                EXPORT  LPUART0_IRQHandler         [WEAK]
                LDR     R0, =LPUART0_DriverIRQHandler
                BX      R0
                ENDP

LPUART1_IRQHandler\
                PROC
                EXPORT  LPUART1_IRQHandler         [WEAK]
                LDR     R0, =LPUART1_DriverIRQHandler
                BX      R0
                ENDP

LPI2C0_IRQHandler\
                PROC
                EXPORT  LPI2C0_IRQHandler         [WEAK]
                LDR     R0, =LPI2C0_DriverIRQHandler
                BX      R0
                ENDP

LPI2C1_IRQHandler\
                PROC
                EXPORT  LPI2C1_IRQHandler         [WEAK]
                LDR     R0, =LPI2C1_DriverIRQHandler
                BX      R0
                ENDP

I2S0_IRQHandler\
                PROC
                EXPORT  I2S0_IRQHandler         [WEAK]
                LDR     R0, =I2S0_DriverIRQHandler
                BX      R0
                ENDP

INTMUX1_0_IRQHandler\
                PROC
                EXPORT  INTMUX1_0_IRQHandler         [WEAK]
                LDR     R0, =INTMUX1_0_DriverIRQHandler
                BX      R0
                ENDP

INTMUX1_1_IRQHandler\
                PROC
                EXPORT  INTMUX1_1_IRQHandler         [WEAK]
                LDR     R0, =INTMUX1_1_DriverIRQHandler
                BX      R0
                ENDP

INTMUX1_2_IRQHandler\
                PROC
                EXPORT  INTMUX1_2_IRQHandler         [WEAK]
                LDR     R0, =INTMUX1_2_DriverIRQHandler
                BX      R0
                ENDP

INTMUX1_3_IRQHandler\
                PROC
                EXPORT  INTMUX1_3_IRQHandler         [WEAK]
                LDR     R0, =INTMUX1_3_DriverIRQHandler
                BX      R0
                ENDP

LPSPI2_IRQHandler\
                PROC
                EXPORT  LPSPI2_IRQHandler         [WEAK]
                LDR     R0, =LPSPI2_DriverIRQHandler
                BX      R0
                ENDP

LPUART2_IRQHandler\
                PROC
                EXPORT  LPUART2_IRQHandler         [WEAK]
                LDR     R0, =LPUART2_DriverIRQHandler
                BX      R0
                ENDP

LPI2C2_IRQHandler\
                PROC
                EXPORT  LPI2C2_IRQHandler         [WEAK]
                LDR     R0, =LPI2C2_DriverIRQHandler
                BX      R0
                ENDP

DMA0_04_IRQHandler\
                PROC
                EXPORT  DMA0_04_IRQHandler         [WEAK]
                LDR     R0, =DMA0_04_DriverIRQHandler
                BX      R0
                ENDP

DMA0_15_IRQHandler\
                PROC
                EXPORT  DMA0_15_IRQHandler         [WEAK]
                LDR     R0, =DMA0_15_DriverIRQHandler
                BX      R0
                ENDP

DMA0_26_IRQHandler\
                PROC
                EXPORT  DMA0_26_IRQHandler         [WEAK]
                LDR     R0, =DMA0_26_DriverIRQHandler
                BX      R0
                ENDP

DMA0_37_IRQHandler\
                PROC
                EXPORT  DMA0_37_IRQHandler         [WEAK]
                LDR     R0, =DMA0_37_DriverIRQHandler
                BX      R0
                ENDP

DMA0_Error_IRQHandler\
                PROC
                EXPORT  DMA0_Error_IRQHandler         [WEAK]
                LDR     R0, =DMA0_Error_DriverIRQHandler
                BX      R0
                ENDP

Default_Handler\
                PROC
                EXPORT  DMA1_04_DriverIRQHandler         [WEAK]
                EXPORT  DMA1_15_DriverIRQHandler         [WEAK]
                EXPORT  DMA1_26_DriverIRQHandler         [WEAK]
                EXPORT  DMA1_37_DriverIRQHandler         [WEAK]
                EXPORT  CTI1_DMA1_Error_DriverIRQHandler         [WEAK]
                EXPORT  FLEXIO0_DriverIRQHandler         [WEAK]
                EXPORT  TPM0_IRQHandler         [WEAK]
                EXPORT  TPM1_IRQHandler         [WEAK]
                EXPORT  Reserved24_IRQHandler         [WEAK]
                EXPORT  LPIT1_IRQHandler         [WEAK]
                EXPORT  LPSPI0_DriverIRQHandler         [WEAK]
                EXPORT  LPSPI1_DriverIRQHandler         [WEAK]
                EXPORT  LPUART0_DriverIRQHandler         [WEAK]
                EXPORT  LPUART1_DriverIRQHandler         [WEAK]
                EXPORT  LPI2C0_DriverIRQHandler         [WEAK]
                EXPORT  LPI2C1_DriverIRQHandler         [WEAK]
                EXPORT  MU0_B_IRQHandler         [WEAK]
                EXPORT  PORTM_IRQHandler         [WEAK]
                EXPORT  TRNG_IRQHandler         [WEAK]
                EXPORT  TSI0_IRQHandler         [WEAK]
                EXPORT  DAC0_IRQHandler         [WEAK]
                EXPORT  CMP1_IRQHandler         [WEAK]
                EXPORT  LLWU1_IRQHandler         [WEAK]
                EXPORT  I2S0_DriverIRQHandler         [WEAK]
                EXPORT  USB0_IRQHandler         [WEAK]
                EXPORT  ADC0_IRQHandler         [WEAK]
                EXPORT  LPTMR1_IRQHandler         [WEAK]
                EXPORT  RTC_Seconds_IRQHandler         [WEAK]
                EXPORT  INTMUX1_0_DriverIRQHandler         [WEAK]
                EXPORT  INTMUX1_1_DriverIRQHandler         [WEAK]
                EXPORT  INTMUX1_2_DriverIRQHandler         [WEAK]
                EXPORT  INTMUX1_3_DriverIRQHandler         [WEAK]
                EXPORT  LPTMR0_IRQHandler         [WEAK]
                EXPORT  LPIT0_IRQHandler         [WEAK]
                EXPORT  TPM2_IRQHandler         [WEAK]
                EXPORT  Reserved51_IRQHandler         [WEAK]
                EXPORT  LPSPI2_DriverIRQHandler         [WEAK]
                EXPORT  LPUART2_DriverIRQHandler         [WEAK]
                EXPORT  EMVSIM0_IRQHandler         [WEAK]
                EXPORT  LPI2C2_DriverIRQHandler         [WEAK]
                EXPORT  Reserved56_IRQHandler         [WEAK]
                EXPORT  PMC_IRQHandler         [WEAK]
                EXPORT  FTFA_IRQHandler         [WEAK]
                EXPORT  SCG_IRQHandler         [WEAK]
                EXPORT  XRDC_IRQHandler         [WEAK]
                EXPORT  Reserved61_IRQHandler         [WEAK]
                EXPORT  Reserved62_IRQHandler         [WEAK]
                EXPORT  RCM_IRQHandler         [WEAK]
                EXPORT  CMP0_IRQHandler         [WEAK]
                EXPORT  Reserved65_IRQHandler         [WEAK]
                EXPORT  RTC_IRQHandler         [WEAK]
                EXPORT  PORTA_IRQHandler         [WEAK]
                EXPORT  PORTB_IRQHandler         [WEAK]
                EXPORT  PORTC_IRQHandler         [WEAK]
                EXPORT  PORTD_IRQHandler         [WEAK]
                EXPORT  PORTE_IRQHandler         [WEAK]
                EXPORT  DMA0_04_DriverIRQHandler         [WEAK]
                EXPORT  DMA0_15_DriverIRQHandler         [WEAK]
                EXPORT  DMA0_26_DriverIRQHandler         [WEAK]
                EXPORT  DMA0_37_DriverIRQHandler         [WEAK]
                EXPORT  DMA0_Error_DriverIRQHandler         [WEAK]
                EXPORT  Reserved77_IRQHandler         [WEAK]
                EXPORT  Reserved78_IRQHandler         [WEAK]
                EXPORT  Reserved79_IRQHandler         [WEAK]
                EXPORT  DefaultISR         [WEAK]
DMA1_04_DriverIRQHandler
DMA1_15_DriverIRQHandler
DMA1_26_DriverIRQHandler
DMA1_37_DriverIRQHandler
CTI1_DMA1_Error_DriverIRQHandler
FLEXIO0_DriverIRQHandler
TPM0_IRQHandler
TPM1_IRQHandler
Reserved24_IRQHandler
LPIT1_IRQHandler
LPSPI0_DriverIRQHandler
LPSPI1_DriverIRQHandler
LPUART0_DriverIRQHandler
LPUART1_DriverIRQHandler
LPI2C0_DriverIRQHandler
LPI2C1_DriverIRQHandler
MU0_B_IRQHandler
PORTM_IRQHandler
TRNG_IRQHandler
TSI0_IRQHandler
DAC0_IRQHandler
CMP1_IRQHandler
LLWU1_IRQHandler
I2S0_DriverIRQHandler
USB0_IRQHandler
ADC0_IRQHandler
LPTMR1_IRQHandler
RTC_Seconds_IRQHandler
INTMUX1_0_DriverIRQHandler
INTMUX1_1_DriverIRQHandler
INTMUX1_2_DriverIRQHandler
INTMUX1_3_DriverIRQHandler
LPTMR0_IRQHandler
LPIT0_IRQHandler
TPM2_IRQHandler
Reserved51_IRQHandler
LPSPI2_DriverIRQHandler
LPUART2_DriverIRQHandler
EMVSIM0_IRQHandler
LPI2C2_DriverIRQHandler
Reserved56_IRQHandler
PMC_IRQHandler
FTFA_IRQHandler
SCG_IRQHandler
XRDC_IRQHandler
Reserved61_IRQHandler
Reserved62_IRQHandler
RCM_IRQHandler
CMP0_IRQHandler
Reserved65_IRQHandler
RTC_IRQHandler
PORTA_IRQHandler
PORTB_IRQHandler
PORTC_IRQHandler
PORTD_IRQHandler
PORTE_IRQHandler
DMA0_04_DriverIRQHandler
DMA0_15_DriverIRQHandler
DMA0_26_DriverIRQHandler
DMA0_37_DriverIRQHandler
DMA0_Error_DriverIRQHandler
Reserved77_IRQHandler
Reserved78_IRQHandler
Reserved79_IRQHandler
DefaultISR
                LDR    R0, =DefaultISR
                BX     R0
                ENDP
                  ALIGN


                END
