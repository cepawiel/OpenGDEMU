/**
* \file  atsamr30.h
*
* \brief ATSAMR30 registers description
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

#ifndef _ATSAMR30_H_
#define _ATSAMR30_H_
#include <stdint.h>

#include <component_sercom.h>
#define MMIO_REG(mem_addr, type) (*(volatile type *)(mem_addr))

/** Interrupt Number Definition */
typedef enum IRQn
{
  /******  Cortex-M0+ Processor Exceptions Numbers ******************************/
  NonMaskableInt_IRQn      = -14,/**<  2 Non Maskable Interrupt                 */
  HardFault_IRQn           = -13,/**<  3 Cortex-M0+ Hard Fault Interrupt        */
  SVCall_IRQn              = -5, /**< 11 Cortex-M0+ SV Call Interrupt           */
  PendSV_IRQn              = -2, /**< 14 Cortex-M0+ Pend SV Interrupt           */
  SysTick_IRQn             = -1, /**< 15 Cortex-M0+ System Tick Interrupt       */
  /******  SAMR30G18A-specific Interrupt Numbers ***********************/
  SYSTEM_IRQn              =  0, /**<  0 SAMR30G18A System Interrupts */
  WDT_IRQn                 =  1, /**<  1 SAMR30G18A Watchdog Timer (WDT) */
  RTC_IRQn                 =  2, /**<  2 SAMR30G18A Real-Time Counter (RTC) */
  EIC_IRQn                 =  3, /**<  3 SAMR30G18A External Interrupt Controller (EIC) */
  NVMCTRL_IRQn             =  4, /**<  4 SAMR30G18A Non-Volatile Memory Controller (NVMCTRL) */
  DMAC_IRQn                =  5, /**<  5 SAMR30G18A Direct Memory Access Controller (DMAC) */
  USB_IRQn                 =  6, /**<  6 SAMR30G18A Universal Serial Bus (USB) */
  EVSYS_IRQn               =  7, /**<  7 SAMR30G18A Event System Interface (EVSYS) */
  SERCOM0_IRQn             =  8, /**<  8 SAMR30G18A Serial Communication Interface 0 (SERCOM0) */
  SERCOM1_IRQn             =  9, /**<  9 SAMR30G18A Serial Communication Interface 1 (SERCOM1) */
  SERCOM2_IRQn             = 10, /**< 10 SAMR30G18A Serial Communication Interface 2 (SERCOM2) */
  SERCOM3_IRQn             = 11, /**< 11 SAMR30G18A Serial Communication Interface 3 (SERCOM3) */
  SERCOM4_IRQn             = 12, /**< 12 SAMR30G18A Serial Communication Interface 4 (SERCOM4) */
  SERCOM5_IRQn             = 13, /**< 13 SAMR30G18A Serial Communication Interface 5 (SERCOM5) */
  TCC0_IRQn                = 14, /**< 14 SAMR30G18A Timer Counter Control 0 (TCC0) */
  TCC1_IRQn                = 15, /**< 15 SAMR30G18A Timer Counter Control 1 (TCC1) */
  TCC2_IRQn                = 16, /**< 16 SAMR30G18A Timer Counter Control 2 (TCC2) */
  TC0_IRQn                 = 17, /**< 17 SAMR30G18A Basic Timer Counter 0 (TC0) */
  TC1_IRQn                 = 18, /**< 18 SAMR30G18A Basic Timer Counter 1 (TC1) */
  TC4_IRQn                 = 21, /**< 21 SAMR30G18A Basic Timer Counter 4 (TC4) */
  ADC_IRQn                 = 22, /**< 22 SAMR30G18A Analog Digital Converter (ADC) */
  AC_IRQn                  = 23, /**< 23 SAMR30G18A Analog Comparators (AC) */
  PTC_IRQn                 = 25, /**< 25 SAMR30G18A Peripheral Touch Controller (PTC) */

  PERIPH_COUNT_IRQn        = 29  /**< Number of peripheral IDs */
} IRQn_Type;

typedef struct _DeviceVectors
{
  /* Stack pointer */
  void* pvStack;

  /* Cortex-M handlers */
  void* pfnReset_Handler;
  void* pfnNMI_Handler;
  void* pfnHardFault_Handler;
  void* pvReservedM12;
  void* pvReservedM11;
  void* pvReservedM10;
  void* pvReservedM9;
  void* pvReservedM8;
  void* pvReservedM7;
  void* pvReservedM6;
  void* pfnSVC_Handler;
  void* pvReservedM4;
  void* pvReservedM3;
  void* pfnPendSV_Handler;
  void* pfnSysTick_Handler;

  /* Peripheral handlers */
  void* pfnSYSTEM_Handler;                /*  0 Main Clock, 32k Oscillators Control, Oscillators Control, Peripheral Access Controller, Power Manager, Supply Controller, Trigger Allocator */
  void* pfnWDT_Handler;                   /*  1 Watchdog Timer */
  void* pfnRTC_Handler;                   /*  2 Real-Time Counter */
  void* pfnEIC_Handler;                   /*  3 External Interrupt Controller */
  void* pfnNVMCTRL_Handler;               /*  4 Non-Volatile Memory Controller */
  void* pfnDMAC_Handler;                  /*  5 Direct Memory Access Controller */
  void* pfnUSB_Handler;                   /*  6 Universal Serial Bus */
  void* pfnEVSYS_Handler;                 /*  7 Event System Interface */
  void* pfnSERCOM0_Handler;               /*  8 Serial Communication Interface 0 */
  void* pfnSERCOM1_Handler;               /*  9 Serial Communication Interface 1 */
  void* pfnSERCOM2_Handler;               /* 10 Serial Communication Interface 2 */
  void* pfnSERCOM3_Handler;               /* 11 Serial Communication Interface 3 */
  void* pfnSERCOM4_Handler;               /* 12 Serial Communication Interface 4 */
  void* pfnSERCOM5_Handler;               /* 13 Serial Communication Interface 5 */
  void* pfnTCC0_Handler;                  /* 14 Timer Counter Control 0 */
  void* pfnTCC1_Handler;                  /* 15 Timer Counter Control 1 */
  void* pfnTCC2_Handler;                  /* 16 Timer Counter Control 2 */
  void* pfnTC0_Handler;                   /* 17 Basic Timer Counter 0 */
  void* pfnTC1_Handler;                   /* 18 Basic Timer Counter 1 */
  void* pvReserved19;
  void* pvReserved20;
  void* pfnTC4_Handler;                   /* 21 Basic Timer Counter 4 */
  void* pfnADC_Handler;                   /* 22 Analog Digital Converter */
  void* pfnAC_Handler;                    /* 23 Analog Comparators */
  void* pvReserved24;
  void* pfnPTC_Handler;                   /* 25 Peripheral Touch Controller */
  void* pvReserved26;
  void* pvReserved27;
  void* pvReserved28;
} DeviceVectors;

/* Cortex-M0+ processor handlers */
void Reset_Handler               ( void );
void NMI_Handler                 ( void );
void HardFault_Handler           ( void );
void SVC_Handler                 ( void );
void PendSV_Handler              ( void );
void SysTick_Handler             ( void );

/* Peripherals handlers */
void SYSTEM_Handler              ( void );
void WDT_Handler                 ( void );
void RTC_Handler                 ( void );
void EIC_Handler                 ( void );
void NVMCTRL_Handler             ( void );
void DMAC_Handler                ( void );
void USB_Handler                 ( void );
void EVSYS_Handler               ( void );
void SERCOM0_Handler             ( void );
void SERCOM1_Handler             ( void );
void SERCOM2_Handler             ( void );
void SERCOM3_Handler             ( void );
void SERCOM4_Handler             ( void );
void SERCOM5_Handler             ( void );
void TCC0_Handler                ( void );
void TCC1_Handler                ( void );
void TCC2_Handler                ( void );
void TC0_Handler                 ( void );
void TC1_Handler                 ( void );
void TC4_Handler                 ( void );
void ADC_Handler                 ( void );
void AC_Handler                  ( void );
void PTC_Handler                 ( void );

/*
 * \brief Configuration of the Cortex-M0+ Processor and Core Peripherals
 */

#define LITTLE_ENDIAN          1
#define __CM0PLUS_REV          1         /*!< Core revision r0p1 */
#define __MPU_PRESENT          0         /*!< MPU present or not */
#define __NVIC_PRIO_BITS       2         /*!< Number of bits used for Priority Levels */
#define __VTOR_PRESENT         1         /*!< VTOR present or not */
#define __Vendor_SysTickConfig 0         /*!< Set to 1 if different SysTick Config is used */

/* ************************************************************************** */
/**  PERIPHERAL ID DEFINITIONS FOR SAML21G18A */
/* ************************************************************************** */
/** \defgroup SAML21G18A_id Peripheral Ids Definitions */
/*@{*/

// Peripheral instances on HPB0 bridge
#define ID_PM             0 /**< \brief Power Manager (PM) */
#define ID_MCLK           1 /**< \brief Main Clock (MCLK) */
#define ID_RSTC           2 /**< \brief Reset Controller (RSTC) */
#define ID_OSCCTRL        3 /**< \brief Oscillators Control (OSCCTRL) */
#define ID_OSC32KCTRL     4 /**< \brief 32k Oscillators Control (OSC32KCTRL) */
#define ID_SUPC           5 /**< \brief Supply Controller (SUPC) */
#define ID_GCLK           6 /**< \brief Generic Clock Generator (GCLK) */
#define ID_WDT            7 /**< \brief Watchdog Timer (WDT) */
#define ID_RTC            8 /**< \brief Real-Time Counter (RTC) */
#define ID_EIC            9 /**< \brief External Interrupt Controller (EIC) */
#define ID_PORT          10 /**< \brief Port Module (PORT) */
#define ID_TAL           11 /**< \brief Trigger Allocator (TAL) */

// Peripheral instances on HPB1 bridge
#define ID_USB           32 /**< \brief Universal Serial Bus (USB) */
#define ID_DSU           33 /**< \brief Device Service Unit (DSU) */
#define ID_NVMCTRL       34 /**< \brief Non-Volatile Memory Controller (NVMCTRL) */
#define ID_MTB           35 /**< \brief Cortex-M0+ Micro-Trace Buffer (MTB) */

// Peripheral instances on HPB2 bridge
#define ID_SERCOM0       64 /**< \brief Serial Communication Interface 0 (SERCOM0) */
#define ID_SERCOM1       65 /**< \brief Serial Communication Interface 1 (SERCOM1) */
#define ID_SERCOM2       66 /**< \brief Serial Communication Interface 2 (SERCOM2) */
#define ID_SERCOM3       67 /**< \brief Serial Communication Interface 3 (SERCOM3) */
#define ID_SERCOM4       68 /**< \brief Serial Communication Interface 4 (SERCOM4) */
#define ID_TCC0          69 /**< \brief Timer Counter Control 0 (TCC0) */
#define ID_TCC1          70 /**< \brief Timer Counter Control 1 (TCC1) */
#define ID_TCC2          71 /**< \brief Timer Counter Control 2 (TCC2) */
#define ID_TC0           72 /**< \brief Basic Timer Counter 0 (TC0) */
#define ID_TC1           73 /**< \brief Basic Timer Counter 1 (TC1) */
#define ID_DAC           76 /**< \brief Digital-to-Analog Converter (DAC) */
#define ID_AES           77 /**< \brief Advanced Encryption Standard (AES) */
#define ID_TRNG          78 /**< \brief True Random Generator (TRNG) */

// Peripheral instances on HPB3 bridge
#define ID_EVSYS         96 /**< \brief Event System Interface (EVSYS) */
#define ID_SERCOM5       97 /**< \brief Serial Communication Interface 5 (SERCOM5) */
#define ID_TC4           98 /**< \brief Basic Timer Counter 4 (TC4) */
#define ID_ADC           99 /**< \brief Analog Digital Converter (ADC) */
#define ID_AC           100 /**< \brief Analog Comparators (AC) */
#define ID_PTC          101 /**< \brief Peripheral Touch Controller (PTC) */
#define ID_OPAMP        102 /**< \brief Operational Amplifier (OPAMP) */
#define ID_CCL          103 /**< \brief Configurable Custom Logic (CCL) */

// Peripheral instances on HPB4 bridge
#define ID_PAC          128 /**< \brief Peripheral Access Controller (PAC) */
#define ID_DMAC         129 /**< \brief Direct Memory Access Controller (DMAC) */

#define ID_PERIPH_COUNT 130 /**< \brief Max number of peripheral IDs */
/*@}*/


#define PIN_PA00                           0  /**< \brief Pin Number for PA00 */
#define PORT_PA00                 (1ul <<  0) /**< \brief PORT Mask  for PA00 */
#define PIN_PA01                           1  /**< \brief Pin Number for PA01 */
#define PORT_PA01                 (1ul <<  1) /**< \brief PORT Mask  for PA01 */
#define PIN_PA04                           4  /**< \brief Pin Number for PA04 */
#define PORT_PA04                 (1ul <<  4) /**< \brief PORT Mask  for PA04 */
#define PIN_PA05                           5  /**< \brief Pin Number for PA05 */
#define PORT_PA05                 (1ul <<  5) /**< \brief PORT Mask  for PA05 */
#define PIN_PA06                           6  /**< \brief Pin Number for PA06 */
#define PORT_PA06                 (1ul <<  6) /**< \brief PORT Mask  for PA06 */
#define PIN_PA07                           7  /**< \brief Pin Number for PA07 */
#define PORT_PA07                 (1ul <<  7) /**< \brief PORT Mask  for PA07 */
#define PIN_PA08                           8  /**< \brief Pin Number for PA08 */
#define PORT_PA08                 (1ul <<  8) /**< \brief PORT Mask  for PA08 */
#define PIN_PA09                           9  /**< \brief Pin Number for PA09 */
#define PORT_PA09                 (1ul <<  9) /**< \brief PORT Mask  for PA09 */
#define PIN_PA10                          10  /**< \brief Pin Number for PA10 */
#define PORT_PA10                 (1ul << 10) /**< \brief PORT Mask  for PA10 */
#define PIN_PA11                          11  /**< \brief Pin Number for PA11 */
#define PORT_PA11                 (1ul << 11) /**< \brief PORT Mask  for PA11 */
#define PIN_PA12                          12  /**< \brief Pin Number for PA12 */
#define PORT_PA12                 (1ul << 12) /**< \brief PORT Mask  for PA12 */
#define PIN_PA13                          13  /**< \brief Pin Number for PA13 */
#define PORT_PA13                 (1ul << 13) /**< \brief PORT Mask  for PA13 */
#define PIN_PA14                          14  /**< \brief Pin Number for PA14 */
#define PORT_PA14                 (1ul << 14) /**< \brief PORT Mask  for PA14 */
#define PIN_PA15                          15  /**< \brief Pin Number for PA15 */
#define PORT_PA15                 (1ul << 15) /**< \brief PORT Mask  for PA15 */
#define PIN_PA16                          16  /**< \brief Pin Number for PA16 */
#define PORT_PA16                 (1ul << 16) /**< \brief PORT Mask  for PA16 */
#define PIN_PA17                          17  /**< \brief Pin Number for PA17 */
#define PORT_PA17                 (1ul << 17) /**< \brief PORT Mask  for PA17 */
#define PIN_PA18                          18  /**< \brief Pin Number for PA18 */
#define PORT_PA18                 (1ul << 18) /**< \brief PORT Mask  for PA18 */
#define PIN_PA19                          19  /**< \brief Pin Number for PA19 */
#define PORT_PA19                 (1ul << 19) /**< \brief PORT Mask  for PA19 */
#define PIN_PA20                          20  /**< \brief Pin Number for PA20 */
#define PORT_PA20                 (1ul << 20) /**< \brief PORT Mask  for PA20 */
#define PIN_PA22                          22  /**< \brief Pin Number for PA22 */
#define PORT_PA22                 (1ul << 22) /**< \brief PORT Mask  for PA22 */
#define PIN_PA23                          23  /**< \brief Pin Number for PA23 */
#define PORT_PA23                 (1ul << 23) /**< \brief PORT Mask  for PA23 */
#define PIN_PA24                          24  /**< \brief Pin Number for PA24 */
#define PORT_PA24                 (1ul << 24) /**< \brief PORT Mask  for PA24 */
#define PIN_PA25                          25  /**< \brief Pin Number for PA25 */
#define PORT_PA25                 (1ul << 25) /**< \brief PORT Mask  for PA25 */
#define PIN_PA27                          27  /**< \brief Pin Number for PA27 */
#define PORT_PA27                 (1ul << 27) /**< \brief PORT Mask  for PA27 */
#define PIN_PA28                          28  /**< \brief Pin Number for PA28 */
#define PORT_PA28                 (1ul << 28) /**< \brief PORT Mask  for PA28 */
#define PIN_PA30                          30  /**< \brief Pin Number for PA30 */
#define PORT_PA30                 (1ul << 30) /**< \brief PORT Mask  for PA30 */
#define PIN_PA31                          31  /**< \brief Pin Number for PA31 */
#define PORT_PA31                 (1ul << 31) /**< \brief PORT Mask  for PA31 */
#define PIN_PB00                          32  /**< \brief Pin Number for PB00 */
#define PORT_PB00                 (1ul <<  0) /**< \brief PORT Mask  for PB00 */
#define PIN_PB02                          34  /**< \brief Pin Number for PB02 */
#define PORT_PB02                 (1ul <<  2) /**< \brief PORT Mask  for PB02 */
#define PIN_PB03                          35  /**< \brief Pin Number for PB03 */
#define PORT_PB03                 (1ul <<  3) /**< \brief PORT Mask  for PB03 */
#define PIN_PB08                          40  /**< \brief Pin Number for PB08 */
#define PORT_PB08                 (1ul <<  8) /**< \brief PORT Mask  for PB08 */
#define PIN_PB09                          41  /**< \brief Pin Number for PB09 */
#define PORT_PB09                 (1ul <<  9) /**< \brief PORT Mask  for PB09 */
#define PIN_PB14                          46  /**< \brief Pin Number for PB14 */
#define PORT_PB14                 (1ul << 14) /**< \brief PORT Mask  for PB14 */
#define PIN_PB15                          47  /**< \brief Pin Number for PB15 */
#define PORT_PB15                 (1ul << 15) /**< \brief PORT Mask  for PB15 */
#define PIN_PB16                          48  /**< \brief Pin Number for PB16 */
#define PORT_PB16                 (1ul << 16) /**< \brief PORT Mask  for PB16 */
#define PIN_PB17                          49  /**< \brief Pin Number for PB17 */
#define PORT_PB17                 (1ul << 17) /**< \brief PORT Mask  for PB17 */
#define PIN_PB22                          54  /**< \brief Pin Number for PB22 */
#define PORT_PB22                 (1ul << 22) /**< \brief PORT Mask  for PB22 */
#define PIN_PB23                          55  /**< \brief Pin Number for PB23 */
#define PORT_PB23                 (1ul << 23) /**< \brief PORT Mask  for PB23 */
#define PIN_PB30                          62  /**< \brief Pin Number for PB30 */
#define PORT_PB30                 (1ul << 30) /**< \brief PORT Mask  for PB30 */
#define PIN_PB31                          63  /**< \brief Pin Number for PB31 */
#define PORT_PB31                 (1ul << 31) /**< \brief PORT Mask  for PB31 */
#define PIN_PC16                          80  /**< \brief Pin Number for PC16 */
#define PORT_PC16                 (1ul << 16) /**< \brief PORT Mask  for PC16 */
#define PIN_PC18                          82  /**< \brief Pin Number for PC18 */
#define PORT_PC18                 (1ul << 18) /**< \brief PORT Mask  for PC18 */
#define PIN_PC19                          83  /**< \brief Pin Number for PC19 */
#define PORT_PC19                 (1ul << 19) /**< \brief PORT Mask  for PC19 */
/* ========== PORT definition for RSTC peripheral ========== */
#define PIN_PA00A_RSTC_EXTWAKE0            0L  /**< \brief RSTC signal: EXTWAKE0 on PA00 mux A */
#define MUX_PA00A_RSTC_EXTWAKE0            0L
#define PINMUX_PA00A_RSTC_EXTWAKE0  ((PIN_PA00A_RSTC_EXTWAKE0 << 16) | MUX_PA00A_RSTC_EXTWAKE0)
#define PORT_PA00A_RSTC_EXTWAKE0   (1ul <<  0)
#define PIN_PA01A_RSTC_EXTWAKE1            1L  /**< \brief RSTC signal: EXTWAKE1 on PA01 mux A */
#define MUX_PA01A_RSTC_EXTWAKE1            0L
#define PINMUX_PA01A_RSTC_EXTWAKE1  ((PIN_PA01A_RSTC_EXTWAKE1 << 16) | MUX_PA01A_RSTC_EXTWAKE1)
#define PORT_PA01A_RSTC_EXTWAKE1   (1ul <<  1)
#define PIN_PA04A_RSTC_EXTWAKE4            4L  /**< \brief RSTC signal: EXTWAKE4 on PA04 mux A */
#define MUX_PA04A_RSTC_EXTWAKE4            0L
#define PINMUX_PA04A_RSTC_EXTWAKE4  ((PIN_PA04A_RSTC_EXTWAKE4 << 16) | MUX_PA04A_RSTC_EXTWAKE4)
#define PORT_PA04A_RSTC_EXTWAKE4   (1ul <<  4)
#define PIN_PA05A_RSTC_EXTWAKE5            5L  /**< \brief RSTC signal: EXTWAKE5 on PA05 mux A */
#define MUX_PA05A_RSTC_EXTWAKE5            0L
#define PINMUX_PA05A_RSTC_EXTWAKE5  ((PIN_PA05A_RSTC_EXTWAKE5 << 16) | MUX_PA05A_RSTC_EXTWAKE5)
#define PORT_PA05A_RSTC_EXTWAKE5   (1ul <<  5)
#define PIN_PA06A_RSTC_EXTWAKE6            6L  /**< \brief RSTC signal: EXTWAKE6 on PA06 mux A */
#define MUX_PA06A_RSTC_EXTWAKE6            0L
#define PINMUX_PA06A_RSTC_EXTWAKE6  ((PIN_PA06A_RSTC_EXTWAKE6 << 16) | MUX_PA06A_RSTC_EXTWAKE6)
#define PORT_PA06A_RSTC_EXTWAKE6   (1ul <<  6)
#define PIN_PA07A_RSTC_EXTWAKE7            7L  /**< \brief RSTC signal: EXTWAKE7 on PA07 mux A */
#define MUX_PA07A_RSTC_EXTWAKE7            0L
#define PINMUX_PA07A_RSTC_EXTWAKE7  ((PIN_PA07A_RSTC_EXTWAKE7 << 16) | MUX_PA07A_RSTC_EXTWAKE7)
#define PORT_PA07A_RSTC_EXTWAKE7   (1ul <<  7)
/* ========== PORT definition for SUPC peripheral ========== */
#define PIN_PB02H_SUPC_OUT1               34L  /**< \brief SUPC signal: OUT1 on PB02 mux H */
#define MUX_PB02H_SUPC_OUT1                7L
#define PINMUX_PB02H_SUPC_OUT1     ((PIN_PB02H_SUPC_OUT1 << 16) | MUX_PB02H_SUPC_OUT1)
#define PORT_PB02H_SUPC_OUT1       (1ul <<  2)
#define PIN_PB00H_SUPC_PSOK               32L  /**< \brief SUPC signal: PSOK on PB00 mux H */
#define MUX_PB00H_SUPC_PSOK                7L
#define PINMUX_PB00H_SUPC_PSOK     ((PIN_PB00H_SUPC_PSOK << 16) | MUX_PB00H_SUPC_PSOK)
#define PORT_PB00H_SUPC_PSOK       (1ul <<  0)
#define PIN_PB03H_SUPC_VBAT               35L  /**< \brief SUPC signal: VBAT on PB03 mux H */
#define MUX_PB03H_SUPC_VBAT                7L
#define PINMUX_PB03H_SUPC_VBAT     ((PIN_PB03H_SUPC_VBAT << 16) | MUX_PB03H_SUPC_VBAT)
#define PORT_PB03H_SUPC_VBAT       (1ul <<  3)
/* ========== PORT definition for GCLK peripheral ========== */
#define PIN_PB14H_GCLK_IO0                46L  /**< \brief GCLK signal: IO0 on PB14 mux H */
#define MUX_PB14H_GCLK_IO0                 7L
#define PINMUX_PB14H_GCLK_IO0      ((PIN_PB14H_GCLK_IO0 << 16) | MUX_PB14H_GCLK_IO0)
#define PORT_PB14H_GCLK_IO0        (1ul << 14)
#define PIN_PB22H_GCLK_IO0                54L  /**< \brief GCLK signal: IO0 on PB22 mux H */
#define MUX_PB22H_GCLK_IO0                 7L
#define PINMUX_PB22H_GCLK_IO0      ((PIN_PB22H_GCLK_IO0 << 16) | MUX_PB22H_GCLK_IO0)
#define PORT_PB22H_GCLK_IO0        (1ul << 22)
#define PIN_PA14H_GCLK_IO0                14L  /**< \brief GCLK signal: IO0 on PA14 mux H */
#define MUX_PA14H_GCLK_IO0                 7L
#define PINMUX_PA14H_GCLK_IO0      ((PIN_PA14H_GCLK_IO0 << 16) | MUX_PA14H_GCLK_IO0)
#define PORT_PA14H_GCLK_IO0        (1ul << 14)
#define PIN_PA27H_GCLK_IO0                27L  /**< \brief GCLK signal: IO0 on PA27 mux H */
#define MUX_PA27H_GCLK_IO0                 7L
#define PINMUX_PA27H_GCLK_IO0      ((PIN_PA27H_GCLK_IO0 << 16) | MUX_PA27H_GCLK_IO0)
#define PORT_PA27H_GCLK_IO0        (1ul << 27)
#define PIN_PA30H_GCLK_IO0                30L  /**< \brief GCLK signal: IO0 on PA30 mux H */
#define MUX_PA30H_GCLK_IO0                 7L
#define PINMUX_PA30H_GCLK_IO0      ((PIN_PA30H_GCLK_IO0 << 16) | MUX_PA30H_GCLK_IO0)
#define PORT_PA30H_GCLK_IO0        (1ul << 30)
#define PIN_PA28H_GCLK_IO0                28L  /**< \brief GCLK signal: IO0 on PA28 mux H */
#define MUX_PA28H_GCLK_IO0                 7L
#define PINMUX_PA28H_GCLK_IO0      ((PIN_PA28H_GCLK_IO0 << 16) | MUX_PA28H_GCLK_IO0)
#define PORT_PA28H_GCLK_IO0        (1ul << 28)
#define PIN_PB15H_GCLK_IO1                47L  /**< \brief GCLK signal: IO1 on PB15 mux H */
#define MUX_PB15H_GCLK_IO1                 7L
#define PINMUX_PB15H_GCLK_IO1      ((PIN_PB15H_GCLK_IO1 << 16) | MUX_PB15H_GCLK_IO1)
#define PORT_PB15H_GCLK_IO1        (1ul << 15)
#define PIN_PB23H_GCLK_IO1                55L  /**< \brief GCLK signal: IO1 on PB23 mux H */
#define MUX_PB23H_GCLK_IO1                 7L
#define PINMUX_PB23H_GCLK_IO1      ((PIN_PB23H_GCLK_IO1 << 16) | MUX_PB23H_GCLK_IO1)
#define PORT_PB23H_GCLK_IO1        (1ul << 23)
#define PIN_PA15H_GCLK_IO1                15L  /**< \brief GCLK signal: IO1 on PA15 mux H */
#define MUX_PA15H_GCLK_IO1                 7L
#define PINMUX_PA15H_GCLK_IO1      ((PIN_PA15H_GCLK_IO1 << 16) | MUX_PA15H_GCLK_IO1)
#define PORT_PA15H_GCLK_IO1        (1ul << 15)
#define PIN_PC16F_GCLK_IO1                80L  /**< \brief GCLK signal: IO1 on PC16 mux F */
#define MUX_PC16F_GCLK_IO1                 5L
#define PINMUX_PC16F_GCLK_IO1      ((PIN_PC16F_GCLK_IO1 << 16) | MUX_PC16F_GCLK_IO1)
#define PORT_PC16F_GCLK_IO1        (1ul << 16)
#define PIN_PB16H_GCLK_IO2                48L  /**< \brief GCLK signal: IO2 on PB16 mux H */
#define MUX_PB16H_GCLK_IO2                 7L
#define PINMUX_PB16H_GCLK_IO2      ((PIN_PB16H_GCLK_IO2 << 16) | MUX_PB16H_GCLK_IO2)
#define PORT_PB16H_GCLK_IO2        (1ul << 16)
#define PIN_PA16H_GCLK_IO2                16L  /**< \brief GCLK signal: IO2 on PA16 mux H */
#define MUX_PA16H_GCLK_IO2                 7L
#define PINMUX_PA16H_GCLK_IO2      ((PIN_PA16H_GCLK_IO2 << 16) | MUX_PA16H_GCLK_IO2)
#define PORT_PA16H_GCLK_IO2        (1ul << 16)
#define PIN_PA17H_GCLK_IO3                17L  /**< \brief GCLK signal: IO3 on PA17 mux H */
#define MUX_PA17H_GCLK_IO3                 7L
#define PINMUX_PA17H_GCLK_IO3      ((PIN_PA17H_GCLK_IO3 << 16) | MUX_PA17H_GCLK_IO3)
#define PORT_PA17H_GCLK_IO3        (1ul << 17)
#define PIN_PB17H_GCLK_IO3                49L  /**< \brief GCLK signal: IO3 on PB17 mux H */
#define MUX_PB17H_GCLK_IO3                 7L
#define PINMUX_PB17H_GCLK_IO3      ((PIN_PB17H_GCLK_IO3 << 16) | MUX_PB17H_GCLK_IO3)
#define PORT_PB17H_GCLK_IO3        (1ul << 17)
#define PIN_PA10H_GCLK_IO4                10L  /**< \brief GCLK signal: IO4 on PA10 mux H */
#define MUX_PA10H_GCLK_IO4                 7L
#define PINMUX_PA10H_GCLK_IO4      ((PIN_PA10H_GCLK_IO4 << 16) | MUX_PA10H_GCLK_IO4)
#define PORT_PA10H_GCLK_IO4        (1ul << 10)
#define PIN_PA20H_GCLK_IO4                20L  /**< \brief GCLK signal: IO4 on PA20 mux H */
#define MUX_PA20H_GCLK_IO4                 7L
#define PINMUX_PA20H_GCLK_IO4      ((PIN_PA20H_GCLK_IO4 << 16) | MUX_PA20H_GCLK_IO4)
#define PORT_PA20H_GCLK_IO4        (1ul << 20)
#define PIN_PA11H_GCLK_IO5                11L  /**< \brief GCLK signal: IO5 on PA11 mux H */
#define MUX_PA11H_GCLK_IO5                 7L
#define PINMUX_PA11H_GCLK_IO5      ((PIN_PA11H_GCLK_IO5 << 16) | MUX_PA11H_GCLK_IO5)
#define PORT_PA11H_GCLK_IO5        (1ul << 11)
#define PIN_PA22H_GCLK_IO6                22L  /**< \brief GCLK signal: IO6 on PA22 mux H */
#define MUX_PA22H_GCLK_IO6                 7L
#define PINMUX_PA22H_GCLK_IO6      ((PIN_PA22H_GCLK_IO6 << 16) | MUX_PA22H_GCLK_IO6)
#define PORT_PA22H_GCLK_IO6        (1ul << 22)
#define PIN_PA23H_GCLK_IO7                23L  /**< \brief GCLK signal: IO7 on PA23 mux H */
#define MUX_PA23H_GCLK_IO7                 7L
#define PINMUX_PA23H_GCLK_IO7      ((PIN_PA23H_GCLK_IO7 << 16) | MUX_PA23H_GCLK_IO7)
#define PORT_PA23H_GCLK_IO7        (1ul << 23)
/* ========== PORT definition for EIC peripheral ========== */
#define PIN_PA16A_EIC_EXTINT0             16L  /**< \brief EIC signal: EXTINT0 on PA16 mux A */
#define MUX_PA16A_EIC_EXTINT0              0L
#define PINMUX_PA16A_EIC_EXTINT0   ((PIN_PA16A_EIC_EXTINT0 << 16) | MUX_PA16A_EIC_EXTINT0)
#define PORT_PA16A_EIC_EXTINT0     (1ul << 16)
#define PIN_PB00A_EIC_EXTINT0             32L  /**< \brief EIC signal: EXTINT0 on PB00 mux A */
#define MUX_PB00A_EIC_EXTINT0              0L
#define PINMUX_PB00A_EIC_EXTINT0   ((PIN_PB00A_EIC_EXTINT0 << 16) | MUX_PB00A_EIC_EXTINT0)
#define PORT_PB00A_EIC_EXTINT0     (1ul <<  0)
#define PIN_PB16A_EIC_EXTINT0             48L  /**< \brief EIC signal: EXTINT0 on PB16 mux A */
#define MUX_PB16A_EIC_EXTINT0              0L
#define PINMUX_PB16A_EIC_EXTINT0   ((PIN_PB16A_EIC_EXTINT0 << 16) | MUX_PB16A_EIC_EXTINT0)
#define PORT_PB16A_EIC_EXTINT0     (1ul << 16)
#define PIN_PA00A_EIC_EXTINT0              0L  /**< \brief EIC signal: EXTINT0 on PA00 mux A */
#define MUX_PA00A_EIC_EXTINT0              0L
#define PINMUX_PA00A_EIC_EXTINT0   ((PIN_PA00A_EIC_EXTINT0 << 16) | MUX_PA00A_EIC_EXTINT0)
#define PORT_PA00A_EIC_EXTINT0     (1ul <<  0)
#define PIN_PA17A_EIC_EXTINT1             17L  /**< \brief EIC signal: EXTINT1 on PA17 mux A */
#define MUX_PA17A_EIC_EXTINT1              0L
#define PINMUX_PA17A_EIC_EXTINT1   ((PIN_PA17A_EIC_EXTINT1 << 16) | MUX_PA17A_EIC_EXTINT1)
#define PORT_PA17A_EIC_EXTINT1     (1ul << 17)
#define PIN_PB17A_EIC_EXTINT1             49L  /**< \brief EIC signal: EXTINT1 on PB17 mux A */
#define MUX_PB17A_EIC_EXTINT1              0L
#define PINMUX_PB17A_EIC_EXTINT1   ((PIN_PB17A_EIC_EXTINT1 << 16) | MUX_PB17A_EIC_EXTINT1)
#define PORT_PB17A_EIC_EXTINT1     (1ul << 17)
#define PIN_PA01A_EIC_EXTINT1              1L  /**< \brief EIC signal: EXTINT1 on PA01 mux A */
#define MUX_PA01A_EIC_EXTINT1              0L
#define PINMUX_PA01A_EIC_EXTINT1   ((PIN_PA01A_EIC_EXTINT1 << 16) | MUX_PA01A_EIC_EXTINT1)
#define PORT_PA01A_EIC_EXTINT1     (1ul <<  1)
#define PIN_PA18A_EIC_EXTINT2             18L  /**< \brief EIC signal: EXTINT2 on PA18 mux A */
#define MUX_PA18A_EIC_EXTINT2              0L
#define PINMUX_PA18A_EIC_EXTINT2   ((PIN_PA18A_EIC_EXTINT2 << 16) | MUX_PA18A_EIC_EXTINT2)
#define PORT_PA18A_EIC_EXTINT2     (1ul << 18)
#define PIN_PB02A_EIC_EXTINT2             34L  /**< \brief EIC signal: EXTINT2 on PB02 mux A */
#define MUX_PB02A_EIC_EXTINT2              0L
#define PINMUX_PB02A_EIC_EXTINT2   ((PIN_PB02A_EIC_EXTINT2 << 16) | MUX_PB02A_EIC_EXTINT2)
#define PORT_PB02A_EIC_EXTINT2     (1ul <<  2)
#define PIN_PA19A_EIC_EXTINT3             19L  /**< \brief EIC signal: EXTINT3 on PA19 mux A */
#define MUX_PA19A_EIC_EXTINT3              0L
#define PINMUX_PA19A_EIC_EXTINT3   ((PIN_PA19A_EIC_EXTINT3 << 16) | MUX_PA19A_EIC_EXTINT3)
#define PORT_PA19A_EIC_EXTINT3     (1ul << 19)
#define PIN_PB03A_EIC_EXTINT3             35L  /**< \brief EIC signal: EXTINT3 on PB03 mux A */
#define MUX_PB03A_EIC_EXTINT3              0L
#define PINMUX_PB03A_EIC_EXTINT3   ((PIN_PB03A_EIC_EXTINT3 << 16) | MUX_PB03A_EIC_EXTINT3)
#define PORT_PB03A_EIC_EXTINT3     (1ul <<  3)
#define PIN_PA04A_EIC_EXTINT4              4L  /**< \brief EIC signal: EXTINT4 on PA04 mux A */
#define MUX_PA04A_EIC_EXTINT4              0L
#define PINMUX_PA04A_EIC_EXTINT4   ((PIN_PA04A_EIC_EXTINT4 << 16) | MUX_PA04A_EIC_EXTINT4)
#define PORT_PA04A_EIC_EXTINT4     (1ul <<  4)
#define PIN_PA20A_EIC_EXTINT4             20L  /**< \brief EIC signal: EXTINT4 on PA20 mux A */
#define MUX_PA20A_EIC_EXTINT4              0L
#define PINMUX_PA20A_EIC_EXTINT4   ((PIN_PA20A_EIC_EXTINT4 << 16) | MUX_PA20A_EIC_EXTINT4)
#define PORT_PA20A_EIC_EXTINT4     (1ul << 20)
#define PIN_PA05A_EIC_EXTINT5              5L  /**< \brief EIC signal: EXTINT5 on PA05 mux A */
#define MUX_PA05A_EIC_EXTINT5              0L
#define PINMUX_PA05A_EIC_EXTINT5   ((PIN_PA05A_EIC_EXTINT5 << 16) | MUX_PA05A_EIC_EXTINT5)
#define PORT_PA05A_EIC_EXTINT5     (1ul <<  5)
#define PIN_PA06A_EIC_EXTINT6              6L  /**< \brief EIC signal: EXTINT6 on PA06 mux A */
#define MUX_PA06A_EIC_EXTINT6              0L
#define PINMUX_PA06A_EIC_EXTINT6   ((PIN_PA06A_EIC_EXTINT6 << 16) | MUX_PA06A_EIC_EXTINT6)
#define PORT_PA06A_EIC_EXTINT6     (1ul <<  6)
#define PIN_PA22A_EIC_EXTINT6             22L  /**< \brief EIC signal: EXTINT6 on PA22 mux A */
#define MUX_PA22A_EIC_EXTINT6              0L
#define PINMUX_PA22A_EIC_EXTINT6   ((PIN_PA22A_EIC_EXTINT6 << 16) | MUX_PA22A_EIC_EXTINT6)
#define PORT_PA22A_EIC_EXTINT6     (1ul << 22)
#define PIN_PB22A_EIC_EXTINT6             54L  /**< \brief EIC signal: EXTINT6 on PB22 mux A */
#define MUX_PB22A_EIC_EXTINT6              0L
#define PINMUX_PB22A_EIC_EXTINT6   ((PIN_PB22A_EIC_EXTINT6 << 16) | MUX_PB22A_EIC_EXTINT6)
#define PORT_PB22A_EIC_EXTINT6     (1ul << 22)
#define PIN_PA07A_EIC_EXTINT7              7L  /**< \brief EIC signal: EXTINT7 on PA07 mux A */
#define MUX_PA07A_EIC_EXTINT7              0L
#define PINMUX_PA07A_EIC_EXTINT7   ((PIN_PA07A_EIC_EXTINT7 << 16) | MUX_PA07A_EIC_EXTINT7)
#define PORT_PA07A_EIC_EXTINT7     (1ul <<  7)
#define PIN_PA23A_EIC_EXTINT7             23L  /**< \brief EIC signal: EXTINT7 on PA23 mux A */
#define MUX_PA23A_EIC_EXTINT7              0L
#define PINMUX_PA23A_EIC_EXTINT7   ((PIN_PA23A_EIC_EXTINT7 << 16) | MUX_PA23A_EIC_EXTINT7)
#define PORT_PA23A_EIC_EXTINT7     (1ul << 23)
#define PIN_PB23A_EIC_EXTINT7             55L  /**< \brief EIC signal: EXTINT7 on PB23 mux A */
#define MUX_PB23A_EIC_EXTINT7              0L
#define PINMUX_PB23A_EIC_EXTINT7   ((PIN_PB23A_EIC_EXTINT7 << 16) | MUX_PB23A_EIC_EXTINT7)
#define PORT_PB23A_EIC_EXTINT7     (1ul << 23)
#define PIN_PB08A_EIC_EXTINT8             40L  /**< \brief EIC signal: EXTINT8 on PB08 mux A */
#define MUX_PB08A_EIC_EXTINT8              0L
#define PINMUX_PB08A_EIC_EXTINT8   ((PIN_PB08A_EIC_EXTINT8 << 16) | MUX_PB08A_EIC_EXTINT8)
#define PORT_PB08A_EIC_EXTINT8     (1ul <<  8)
#define PIN_PA28A_EIC_EXTINT8             28L  /**< \brief EIC signal: EXTINT8 on PA28 mux A */
#define MUX_PA28A_EIC_EXTINT8              0L
#define PINMUX_PA28A_EIC_EXTINT8   ((PIN_PA28A_EIC_EXTINT8 << 16) | MUX_PA28A_EIC_EXTINT8)
#define PORT_PA28A_EIC_EXTINT8     (1ul << 28)
#define PIN_PA09A_EIC_EXTINT9              9L  /**< \brief EIC signal: EXTINT9 on PA09 mux A */
#define MUX_PA09A_EIC_EXTINT9              0L
#define PINMUX_PA09A_EIC_EXTINT9   ((PIN_PA09A_EIC_EXTINT9 << 16) | MUX_PA09A_EIC_EXTINT9)
#define PORT_PA09A_EIC_EXTINT9     (1ul <<  9)
#define PIN_PB09A_EIC_EXTINT9             41L  /**< \brief EIC signal: EXTINT9 on PB09 mux A */
#define MUX_PB09A_EIC_EXTINT9              0L
#define PINMUX_PB09A_EIC_EXTINT9   ((PIN_PB09A_EIC_EXTINT9 << 16) | MUX_PB09A_EIC_EXTINT9)
#define PORT_PB09A_EIC_EXTINT9     (1ul <<  9)
#define PIN_PA10A_EIC_EXTINT10            10L  /**< \brief EIC signal: EXTINT10 on PA10 mux A */
#define MUX_PA10A_EIC_EXTINT10             0L
#define PINMUX_PA10A_EIC_EXTINT10  ((PIN_PA10A_EIC_EXTINT10 << 16) | MUX_PA10A_EIC_EXTINT10)
#define PORT_PA10A_EIC_EXTINT10    (1ul << 10)
#define PIN_PA30A_EIC_EXTINT10            30L  /**< \brief EIC signal: EXTINT10 on PA30 mux A */
#define MUX_PA30A_EIC_EXTINT10             0L
#define PINMUX_PA30A_EIC_EXTINT10  ((PIN_PA30A_EIC_EXTINT10 << 16) | MUX_PA30A_EIC_EXTINT10)
#define PORT_PA30A_EIC_EXTINT10    (1ul << 30)
#define PIN_PA11A_EIC_EXTINT11            11L  /**< \brief EIC signal: EXTINT11 on PA11 mux A */
#define MUX_PA11A_EIC_EXTINT11             0L
#define PINMUX_PA11A_EIC_EXTINT11  ((PIN_PA11A_EIC_EXTINT11 << 16) | MUX_PA11A_EIC_EXTINT11)
#define PORT_PA11A_EIC_EXTINT11    (1ul << 11)
#define PIN_PA31A_EIC_EXTINT11            31L  /**< \brief EIC signal: EXTINT11 on PA31 mux A */
#define MUX_PA31A_EIC_EXTINT11             0L
#define PINMUX_PA31A_EIC_EXTINT11  ((PIN_PA31A_EIC_EXTINT11 << 16) | MUX_PA31A_EIC_EXTINT11)
#define PORT_PA31A_EIC_EXTINT11    (1ul << 31)
#define PIN_PA12A_EIC_EXTINT12            12L  /**< \brief EIC signal: EXTINT12 on PA12 mux A */
#define MUX_PA12A_EIC_EXTINT12             0L
#define PINMUX_PA12A_EIC_EXTINT12  ((PIN_PA12A_EIC_EXTINT12 << 16) | MUX_PA12A_EIC_EXTINT12)
#define PORT_PA12A_EIC_EXTINT12    (1ul << 12)
#define PIN_PA24A_EIC_EXTINT12            24L  /**< \brief EIC signal: EXTINT12 on PA24 mux A */
#define MUX_PA24A_EIC_EXTINT12             0L
#define PINMUX_PA24A_EIC_EXTINT12  ((PIN_PA24A_EIC_EXTINT12 << 16) | MUX_PA24A_EIC_EXTINT12)
#define PORT_PA24A_EIC_EXTINT12    (1ul << 24)
#define PIN_PA13A_EIC_EXTINT13            13L  /**< \brief EIC signal: EXTINT13 on PA13 mux A */
#define MUX_PA13A_EIC_EXTINT13             0L
#define PINMUX_PA13A_EIC_EXTINT13  ((PIN_PA13A_EIC_EXTINT13 << 16) | MUX_PA13A_EIC_EXTINT13)
#define PORT_PA13A_EIC_EXTINT13    (1ul << 13)
#define PIN_PA25A_EIC_EXTINT13            25L  /**< \brief EIC signal: EXTINT13 on PA25 mux A */
#define MUX_PA25A_EIC_EXTINT13             0L
#define PINMUX_PA25A_EIC_EXTINT13  ((PIN_PA25A_EIC_EXTINT13 << 16) | MUX_PA25A_EIC_EXTINT13)
#define PORT_PA25A_EIC_EXTINT13    (1ul << 25)
#define PIN_PB14A_EIC_EXTINT14            46L  /**< \brief EIC signal: EXTINT14 on PB14 mux A */
#define MUX_PB14A_EIC_EXTINT14             0L
#define PINMUX_PB14A_EIC_EXTINT14  ((PIN_PB14A_EIC_EXTINT14 << 16) | MUX_PB14A_EIC_EXTINT14)
#define PORT_PB14A_EIC_EXTINT14    (1ul << 14)
#define PIN_PB30A_EIC_EXTINT14            62L  /**< \brief EIC signal: EXTINT14 on PB30 mux A */
#define MUX_PB30A_EIC_EXTINT14             0L
#define PINMUX_PB30A_EIC_EXTINT14  ((PIN_PB30A_EIC_EXTINT14 << 16) | MUX_PB30A_EIC_EXTINT14)
#define PORT_PB30A_EIC_EXTINT14    (1ul << 30)
#define PIN_PA14A_EIC_EXTINT14            14L  /**< \brief EIC signal: EXTINT14 on PA14 mux A */
#define MUX_PA14A_EIC_EXTINT14             0L
#define PINMUX_PA14A_EIC_EXTINT14  ((PIN_PA14A_EIC_EXTINT14 << 16) | MUX_PA14A_EIC_EXTINT14)
#define PORT_PA14A_EIC_EXTINT14    (1ul << 14)
#define PIN_PA27A_EIC_EXTINT15            27L  /**< \brief EIC signal: EXTINT15 on PA27 mux A */
#define MUX_PA27A_EIC_EXTINT15             0L
#define PINMUX_PA27A_EIC_EXTINT15  ((PIN_PA27A_EIC_EXTINT15 << 16) | MUX_PA27A_EIC_EXTINT15)
#define PORT_PA27A_EIC_EXTINT15    (1ul << 27)
#define PIN_PB15A_EIC_EXTINT15            47L  /**< \brief EIC signal: EXTINT15 on PB15 mux A */
#define MUX_PB15A_EIC_EXTINT15             0L
#define PINMUX_PB15A_EIC_EXTINT15  ((PIN_PB15A_EIC_EXTINT15 << 16) | MUX_PB15A_EIC_EXTINT15)
#define PORT_PB15A_EIC_EXTINT15    (1ul << 15)
#define PIN_PB31A_EIC_EXTINT15            63L  /**< \brief EIC signal: EXTINT15 on PB31 mux A */
#define MUX_PB31A_EIC_EXTINT15             0L
#define PINMUX_PB31A_EIC_EXTINT15  ((PIN_PB31A_EIC_EXTINT15 << 16) | MUX_PB31A_EIC_EXTINT15)
#define PORT_PB31A_EIC_EXTINT15    (1ul << 31)
#define PIN_PA15A_EIC_EXTINT15            15L  /**< \brief EIC signal: EXTINT15 on PA15 mux A */
#define MUX_PA15A_EIC_EXTINT15             0L
#define PINMUX_PA15A_EIC_EXTINT15  ((PIN_PA15A_EIC_EXTINT15 << 16) | MUX_PA15A_EIC_EXTINT15)
#define PORT_PA15A_EIC_EXTINT15    (1ul << 15)
#define PIN_PA08A_EIC_NMI                  8L  /**< \brief EIC signal: NMI on PA08 mux A */
#define MUX_PA08A_EIC_NMI                  0L
#define PINMUX_PA08A_EIC_NMI       ((PIN_PA08A_EIC_NMI << 16) | MUX_PA08A_EIC_NMI)
#define PORT_PA08A_EIC_NMI         (1ul <<  8)
/* ========== PORT definition for TAL peripheral ========== */
#define PIN_PA27G_TAL_BRK                 27L  /**< \brief TAL signal: BRK on PA27 mux G */
#define MUX_PA27G_TAL_BRK                  6L
#define PINMUX_PA27G_TAL_BRK       ((PIN_PA27G_TAL_BRK << 16) | MUX_PA27G_TAL_BRK)
#define PORT_PA27G_TAL_BRK         (1ul << 27)
/* ========== PORT definition for USB peripheral ========== */
#define PIN_PA24G_USB_DM                  24L  /**< \brief USB signal: DM on PA24 mux G */
#define MUX_PA24G_USB_DM                   6L
#define PINMUX_PA24G_USB_DM        ((PIN_PA24G_USB_DM << 16) | MUX_PA24G_USB_DM)
#define PORT_PA24G_USB_DM          (1ul << 24)
#define PIN_PA25G_USB_DP                  25L  /**< \brief USB signal: DP on PA25 mux G */
#define MUX_PA25G_USB_DP                   6L
#define PINMUX_PA25G_USB_DP        ((PIN_PA25G_USB_DP << 16) | MUX_PA25G_USB_DP)
#define PORT_PA25G_USB_DP          (1ul << 25)
#define PIN_PA23G_USB_SOF_1KHZ            23L  /**< \brief USB signal: SOF_1KHZ on PA23 mux G */
#define MUX_PA23G_USB_SOF_1KHZ             6L
#define PINMUX_PA23G_USB_SOF_1KHZ  ((PIN_PA23G_USB_SOF_1KHZ << 16) | MUX_PA23G_USB_SOF_1KHZ)
#define PORT_PA23G_USB_SOF_1KHZ    (1ul << 23)
/* ========== PORT definition for SERCOM0 peripheral ========== */
#define PIN_PA04D_SERCOM0_PAD0             4L  /**< \brief SERCOM0 signal: PAD0 on PA04 mux D */
#define MUX_PA04D_SERCOM0_PAD0             3L
#define PINMUX_PA04D_SERCOM0_PAD0  ((PIN_PA04D_SERCOM0_PAD0 << 16) | MUX_PA04D_SERCOM0_PAD0)
#define PORT_PA04D_SERCOM0_PAD0    (1ul <<  4)
#define PIN_PA08C_SERCOM0_PAD0             8L  /**< \brief SERCOM0 signal: PAD0 on PA08 mux C */
#define MUX_PA08C_SERCOM0_PAD0             2L
#define PINMUX_PA08C_SERCOM0_PAD0  ((PIN_PA08C_SERCOM0_PAD0 << 16) | MUX_PA08C_SERCOM0_PAD0)
#define PORT_PA08C_SERCOM0_PAD0    (1ul <<  8)
#define PIN_PA05D_SERCOM0_PAD1             5L  /**< \brief SERCOM0 signal: PAD1 on PA05 mux D */
#define MUX_PA05D_SERCOM0_PAD1             3L
#define PINMUX_PA05D_SERCOM0_PAD1  ((PIN_PA05D_SERCOM0_PAD1 << 16) | MUX_PA05D_SERCOM0_PAD1)
#define PORT_PA05D_SERCOM0_PAD1    (1ul <<  5)
#define PIN_PA09C_SERCOM0_PAD1             9L  /**< \brief SERCOM0 signal: PAD1 on PA09 mux C */
#define MUX_PA09C_SERCOM0_PAD1             2L
#define PINMUX_PA09C_SERCOM0_PAD1  ((PIN_PA09C_SERCOM0_PAD1 << 16) | MUX_PA09C_SERCOM0_PAD1)
#define PORT_PA09C_SERCOM0_PAD1    (1ul <<  9)
#define PIN_PA06D_SERCOM0_PAD2             6L  /**< \brief SERCOM0 signal: PAD2 on PA06 mux D */
#define MUX_PA06D_SERCOM0_PAD2             3L
#define PINMUX_PA06D_SERCOM0_PAD2  ((PIN_PA06D_SERCOM0_PAD2 << 16) | MUX_PA06D_SERCOM0_PAD2)
#define PORT_PA06D_SERCOM0_PAD2    (1ul <<  6)
#define PIN_PA10C_SERCOM0_PAD2            10L  /**< \brief SERCOM0 signal: PAD2 on PA10 mux C */
#define MUX_PA10C_SERCOM0_PAD2             2L
#define PINMUX_PA10C_SERCOM0_PAD2  ((PIN_PA10C_SERCOM0_PAD2 << 16) | MUX_PA10C_SERCOM0_PAD2)
#define PORT_PA10C_SERCOM0_PAD2    (1ul << 10)
#define PIN_PA07D_SERCOM0_PAD3             7L  /**< \brief SERCOM0 signal: PAD3 on PA07 mux D */
#define MUX_PA07D_SERCOM0_PAD3             3L
#define PINMUX_PA07D_SERCOM0_PAD3  ((PIN_PA07D_SERCOM0_PAD3 << 16) | MUX_PA07D_SERCOM0_PAD3)
#define PORT_PA07D_SERCOM0_PAD3    (1ul <<  7)
#define PIN_PA11C_SERCOM0_PAD3            11L  /**< \brief SERCOM0 signal: PAD3 on PA11 mux C */
#define MUX_PA11C_SERCOM0_PAD3             2L
#define PINMUX_PA11C_SERCOM0_PAD3  ((PIN_PA11C_SERCOM0_PAD3 << 16) | MUX_PA11C_SERCOM0_PAD3)
#define PORT_PA11C_SERCOM0_PAD3    (1ul << 11)
/* ========== PORT definition for SERCOM1 peripheral ========== */
#define PIN_PA16C_SERCOM1_PAD0            16L  /**< \brief SERCOM1 signal: PAD0 on PA16 mux C */
#define MUX_PA16C_SERCOM1_PAD0             2L
#define PINMUX_PA16C_SERCOM1_PAD0  ((PIN_PA16C_SERCOM1_PAD0 << 16) | MUX_PA16C_SERCOM1_PAD0)
#define PORT_PA16C_SERCOM1_PAD0    (1ul << 16)
#define PIN_PA00D_SERCOM1_PAD0             0L  /**< \brief SERCOM1 signal: PAD0 on PA00 mux D */
#define MUX_PA00D_SERCOM1_PAD0             3L
#define PINMUX_PA00D_SERCOM1_PAD0  ((PIN_PA00D_SERCOM1_PAD0 << 16) | MUX_PA00D_SERCOM1_PAD0)
#define PORT_PA00D_SERCOM1_PAD0    (1ul <<  0)
#define PIN_PA17C_SERCOM1_PAD1            17L  /**< \brief SERCOM1 signal: PAD1 on PA17 mux C */
#define MUX_PA17C_SERCOM1_PAD1             2L
#define PINMUX_PA17C_SERCOM1_PAD1  ((PIN_PA17C_SERCOM1_PAD1 << 16) | MUX_PA17C_SERCOM1_PAD1)
#define PORT_PA17C_SERCOM1_PAD1    (1ul << 17)
#define PIN_PA01D_SERCOM1_PAD1             1L  /**< \brief SERCOM1 signal: PAD1 on PA01 mux D */
#define MUX_PA01D_SERCOM1_PAD1             3L
#define PINMUX_PA01D_SERCOM1_PAD1  ((PIN_PA01D_SERCOM1_PAD1 << 16) | MUX_PA01D_SERCOM1_PAD1)
#define PORT_PA01D_SERCOM1_PAD1    (1ul <<  1)
#define PIN_PA30D_SERCOM1_PAD2            30L  /**< \brief SERCOM1 signal: PAD2 on PA30 mux D */
#define MUX_PA30D_SERCOM1_PAD2             3L
#define PINMUX_PA30D_SERCOM1_PAD2  ((PIN_PA30D_SERCOM1_PAD2 << 16) | MUX_PA30D_SERCOM1_PAD2)
#define PORT_PA30D_SERCOM1_PAD2    (1ul << 30)
#define PIN_PA18C_SERCOM1_PAD2            18L  /**< \brief SERCOM1 signal: PAD2 on PA18 mux C */
#define MUX_PA18C_SERCOM1_PAD2             2L
#define PINMUX_PA18C_SERCOM1_PAD2  ((PIN_PA18C_SERCOM1_PAD2 << 16) | MUX_PA18C_SERCOM1_PAD2)
#define PORT_PA18C_SERCOM1_PAD2    (1ul << 18)
#define PIN_PA31D_SERCOM1_PAD3            31L  /**< \brief SERCOM1 signal: PAD3 on PA31 mux D */
#define MUX_PA31D_SERCOM1_PAD3             3L
#define PINMUX_PA31D_SERCOM1_PAD3  ((PIN_PA31D_SERCOM1_PAD3 << 16) | MUX_PA31D_SERCOM1_PAD3)
#define PORT_PA31D_SERCOM1_PAD3    (1ul << 31)
#define PIN_PA19C_SERCOM1_PAD3            19L  /**< \brief SERCOM1 signal: PAD3 on PA19 mux C */
#define MUX_PA19C_SERCOM1_PAD3             2L
#define PINMUX_PA19C_SERCOM1_PAD3  ((PIN_PA19C_SERCOM1_PAD3 << 16) | MUX_PA19C_SERCOM1_PAD3)
#define PORT_PA19C_SERCOM1_PAD3    (1ul << 19)
/* ========== PORT definition for SERCOM2 peripheral ========== */
#define PIN_PA08D_SERCOM2_PAD0             8L  /**< \brief SERCOM2 signal: PAD0 on PA08 mux D */
#define MUX_PA08D_SERCOM2_PAD0             3L
#define PINMUX_PA08D_SERCOM2_PAD0  ((PIN_PA08D_SERCOM2_PAD0 << 16) | MUX_PA08D_SERCOM2_PAD0)
#define PORT_PA08D_SERCOM2_PAD0    (1ul <<  8)
#define PIN_PA12C_SERCOM2_PAD0            12L  /**< \brief SERCOM2 signal: PAD0 on PA12 mux C */
#define MUX_PA12C_SERCOM2_PAD0             2L
#define PINMUX_PA12C_SERCOM2_PAD0  ((PIN_PA12C_SERCOM2_PAD0 << 16) | MUX_PA12C_SERCOM2_PAD0)
#define PORT_PA12C_SERCOM2_PAD0    (1ul << 12)
#define PIN_PA09D_SERCOM2_PAD1             9L  /**< \brief SERCOM2 signal: PAD1 on PA09 mux D */
#define MUX_PA09D_SERCOM2_PAD1             3L
#define PINMUX_PA09D_SERCOM2_PAD1  ((PIN_PA09D_SERCOM2_PAD1 << 16) | MUX_PA09D_SERCOM2_PAD1)
#define PORT_PA09D_SERCOM2_PAD1    (1ul <<  9)
#define PIN_PA13C_SERCOM2_PAD1            13L  /**< \brief SERCOM2 signal: PAD1 on PA13 mux C */
#define MUX_PA13C_SERCOM2_PAD1             2L
#define PINMUX_PA13C_SERCOM2_PAD1  ((PIN_PA13C_SERCOM2_PAD1 << 16) | MUX_PA13C_SERCOM2_PAD1)
#define PORT_PA13C_SERCOM2_PAD1    (1ul << 13)
#define PIN_PA10D_SERCOM2_PAD2            10L  /**< \brief SERCOM2 signal: PAD2 on PA10 mux D */
#define MUX_PA10D_SERCOM2_PAD2             3L
#define PINMUX_PA10D_SERCOM2_PAD2  ((PIN_PA10D_SERCOM2_PAD2 << 16) | MUX_PA10D_SERCOM2_PAD2)
#define PORT_PA10D_SERCOM2_PAD2    (1ul << 10)
#define PIN_PA14C_SERCOM2_PAD2            14L  /**< \brief SERCOM2 signal: PAD2 on PA14 mux C */
#define MUX_PA14C_SERCOM2_PAD2             2L
#define PINMUX_PA14C_SERCOM2_PAD2  ((PIN_PA14C_SERCOM2_PAD2 << 16) | MUX_PA14C_SERCOM2_PAD2)
#define PORT_PA14C_SERCOM2_PAD2    (1ul << 14)
#define PIN_PA11D_SERCOM2_PAD3            11L  /**< \brief SERCOM2 signal: PAD3 on PA11 mux D */
#define MUX_PA11D_SERCOM2_PAD3             3L
#define PINMUX_PA11D_SERCOM2_PAD3  ((PIN_PA11D_SERCOM2_PAD3 << 16) | MUX_PA11D_SERCOM2_PAD3)
#define PORT_PA11D_SERCOM2_PAD3    (1ul << 11)
#define PIN_PA15C_SERCOM2_PAD3            15L  /**< \brief SERCOM2 signal: PAD3 on PA15 mux C */
#define MUX_PA15C_SERCOM2_PAD3             2L
#define PINMUX_PA15C_SERCOM2_PAD3  ((PIN_PA15C_SERCOM2_PAD3 << 16) | MUX_PA15C_SERCOM2_PAD3)
#define PORT_PA15C_SERCOM2_PAD3    (1ul << 15)
/* ========== PORT definition for SERCOM3 peripheral ========== */
#define PIN_PA16D_SERCOM3_PAD0            16L  /**< \brief SERCOM3 signal: PAD0 on PA16 mux D */
#define MUX_PA16D_SERCOM3_PAD0             3L
#define PINMUX_PA16D_SERCOM3_PAD0  ((PIN_PA16D_SERCOM3_PAD0 << 16) | MUX_PA16D_SERCOM3_PAD0)
#define PORT_PA16D_SERCOM3_PAD0    (1ul << 16)
#define PIN_PA22C_SERCOM3_PAD0            22L  /**< \brief SERCOM3 signal: PAD0 on PA22 mux C */
#define MUX_PA22C_SERCOM3_PAD0             2L
#define PINMUX_PA22C_SERCOM3_PAD0  ((PIN_PA22C_SERCOM3_PAD0 << 16) | MUX_PA22C_SERCOM3_PAD0)
#define PORT_PA22C_SERCOM3_PAD0    (1ul << 22)
#define PIN_PA27F_SERCOM3_PAD0            27L  /**< \brief SERCOM3 signal: PAD0 on PA27 mux F */
#define MUX_PA27F_SERCOM3_PAD0             5L
#define PINMUX_PA27F_SERCOM3_PAD0  ((PIN_PA27F_SERCOM3_PAD0 << 16) | MUX_PA27F_SERCOM3_PAD0)
#define PORT_PA27F_SERCOM3_PAD0    (1ul << 27)
#define PIN_PA17D_SERCOM3_PAD1            17L  /**< \brief SERCOM3 signal: PAD1 on PA17 mux D */
#define MUX_PA17D_SERCOM3_PAD1             3L
#define PINMUX_PA17D_SERCOM3_PAD1  ((PIN_PA17D_SERCOM3_PAD1 << 16) | MUX_PA17D_SERCOM3_PAD1)
#define PORT_PA17D_SERCOM3_PAD1    (1ul << 17)
#define PIN_PA23C_SERCOM3_PAD1            23L  /**< \brief SERCOM3 signal: PAD1 on PA23 mux C */
#define MUX_PA23C_SERCOM3_PAD1             2L
#define PINMUX_PA23C_SERCOM3_PAD1  ((PIN_PA23C_SERCOM3_PAD1 << 16) | MUX_PA23C_SERCOM3_PAD1)
#define PORT_PA23C_SERCOM3_PAD1    (1ul << 23)
#define PIN_PA28F_SERCOM3_PAD1            28L  /**< \brief SERCOM3 signal: PAD1 on PA28 mux F */
#define MUX_PA28F_SERCOM3_PAD1             5L
#define PINMUX_PA28F_SERCOM3_PAD1  ((PIN_PA28F_SERCOM3_PAD1 << 16) | MUX_PA28F_SERCOM3_PAD1)
#define PORT_PA28F_SERCOM3_PAD1    (1ul << 28)
#define PIN_PA18D_SERCOM3_PAD2            18L  /**< \brief SERCOM3 signal: PAD2 on PA18 mux D */
#define MUX_PA18D_SERCOM3_PAD2             3L
#define PINMUX_PA18D_SERCOM3_PAD2  ((PIN_PA18D_SERCOM3_PAD2 << 16) | MUX_PA18D_SERCOM3_PAD2)
#define PORT_PA18D_SERCOM3_PAD2    (1ul << 18)
#define PIN_PA20D_SERCOM3_PAD2            20L  /**< \brief SERCOM3 signal: PAD2 on PA20 mux D */
#define MUX_PA20D_SERCOM3_PAD2             3L
#define PINMUX_PA20D_SERCOM3_PAD2  ((PIN_PA20D_SERCOM3_PAD2 << 16) | MUX_PA20D_SERCOM3_PAD2)
#define PORT_PA20D_SERCOM3_PAD2    (1ul << 20)
#define PIN_PA24C_SERCOM3_PAD2            24L  /**< \brief SERCOM3 signal: PAD2 on PA24 mux C */
#define MUX_PA24C_SERCOM3_PAD2             2L
#define PINMUX_PA24C_SERCOM3_PAD2  ((PIN_PA24C_SERCOM3_PAD2 << 16) | MUX_PA24C_SERCOM3_PAD2)
#define PORT_PA24C_SERCOM3_PAD2    (1ul << 24)
#define PIN_PA19D_SERCOM3_PAD3            19L  /**< \brief SERCOM3 signal: PAD3 on PA19 mux D */
#define MUX_PA19D_SERCOM3_PAD3             3L
#define PINMUX_PA19D_SERCOM3_PAD3  ((PIN_PA19D_SERCOM3_PAD3 << 16) | MUX_PA19D_SERCOM3_PAD3)
#define PORT_PA19D_SERCOM3_PAD3    (1ul << 19)
#define PIN_PA25C_SERCOM3_PAD3            25L  /**< \brief SERCOM3 signal: PAD3 on PA25 mux C */
#define MUX_PA25C_SERCOM3_PAD3             2L
#define PINMUX_PA25C_SERCOM3_PAD3  ((PIN_PA25C_SERCOM3_PAD3 << 16) | MUX_PA25C_SERCOM3_PAD3)
#define PORT_PA25C_SERCOM3_PAD3    (1ul << 25)
/* ========== PORT definition for SERCOM4 peripheral ========== */
#define PIN_PA12D_SERCOM4_PAD0            12L  /**< \brief SERCOM4 signal: PAD0 on PA12 mux D */
#define MUX_PA12D_SERCOM4_PAD0             3L
#define PINMUX_PA12D_SERCOM4_PAD0  ((PIN_PA12D_SERCOM4_PAD0 << 16) | MUX_PA12D_SERCOM4_PAD0)
#define PORT_PA12D_SERCOM4_PAD0    (1ul << 12)
#define PIN_PB08D_SERCOM4_PAD0            40L  /**< \brief SERCOM4 signal: PAD0 on PB08 mux D */
#define MUX_PB08D_SERCOM4_PAD0             3L
#define PINMUX_PB08D_SERCOM4_PAD0  ((PIN_PB08D_SERCOM4_PAD0 << 16) | MUX_PB08D_SERCOM4_PAD0)
#define PORT_PB08D_SERCOM4_PAD0    (1ul <<  8)
#define PIN_PC19F_SERCOM4_PAD0            83L  /**< \brief SERCOM4 signal: PAD0 on PC19 mux F */
#define MUX_PC19F_SERCOM4_PAD0             5L
#define PINMUX_PC19F_SERCOM4_PAD0  ((PIN_PC19F_SERCOM4_PAD0 << 16) | MUX_PC19F_SERCOM4_PAD0)
#define PORT_PC19F_SERCOM4_PAD0    (1ul << 19)
#define PIN_PA13D_SERCOM4_PAD1            13L  /**< \brief SERCOM4 signal: PAD1 on PA13 mux D */
#define MUX_PA13D_SERCOM4_PAD1             3L
#define PINMUX_PA13D_SERCOM4_PAD1  ((PIN_PA13D_SERCOM4_PAD1 << 16) | MUX_PA13D_SERCOM4_PAD1)
#define PORT_PA13D_SERCOM4_PAD1    (1ul << 13)
#define PIN_PB09D_SERCOM4_PAD1            41L  /**< \brief SERCOM4 signal: PAD1 on PB09 mux D */
#define MUX_PB09D_SERCOM4_PAD1             3L
#define PINMUX_PB09D_SERCOM4_PAD1  ((PIN_PB09D_SERCOM4_PAD1 << 16) | MUX_PB09D_SERCOM4_PAD1)
#define PORT_PB09D_SERCOM4_PAD1    (1ul <<  9)
#define PIN_PB31F_SERCOM4_PAD1            63L  /**< \brief SERCOM4 signal: PAD1 on PB31 mux F */
#define MUX_PB31F_SERCOM4_PAD1             5L
#define PINMUX_PB31F_SERCOM4_PAD1  ((PIN_PB31F_SERCOM4_PAD1 << 16) | MUX_PB31F_SERCOM4_PAD1)
#define PORT_PB31F_SERCOM4_PAD1    (1ul << 31)
#define PIN_PA14D_SERCOM4_PAD2            14L  /**< \brief SERCOM4 signal: PAD2 on PA14 mux D */
#define MUX_PA14D_SERCOM4_PAD2             3L
#define PINMUX_PA14D_SERCOM4_PAD2  ((PIN_PA14D_SERCOM4_PAD2 << 16) | MUX_PA14D_SERCOM4_PAD2)
#define PORT_PA14D_SERCOM4_PAD2    (1ul << 14)
#define PIN_PB14C_SERCOM4_PAD2            46L  /**< \brief SERCOM4 signal: PAD2 on PB14 mux C */
#define MUX_PB14C_SERCOM4_PAD2             2L
#define PINMUX_PB14C_SERCOM4_PAD2  ((PIN_PB14C_SERCOM4_PAD2 << 16) | MUX_PB14C_SERCOM4_PAD2)
#define PORT_PB14C_SERCOM4_PAD2    (1ul << 14)
#define PIN_PB30F_SERCOM4_PAD2            62L  /**< \brief SERCOM4 signal: PAD2 on PB30 mux F */
#define MUX_PB30F_SERCOM4_PAD2             5L
#define PINMUX_PB30F_SERCOM4_PAD2  ((PIN_PB30F_SERCOM4_PAD2 << 16) | MUX_PB30F_SERCOM4_PAD2)
#define PORT_PB30F_SERCOM4_PAD2    (1ul << 30)
#define PIN_PA15D_SERCOM4_PAD3            15L  /**< \brief SERCOM4 signal: PAD3 on PA15 mux D */
#define MUX_PA15D_SERCOM4_PAD3             3L
#define PINMUX_PA15D_SERCOM4_PAD3  ((PIN_PA15D_SERCOM4_PAD3 << 16) | MUX_PA15D_SERCOM4_PAD3)
#define PORT_PA15D_SERCOM4_PAD3    (1ul << 15)
#define PIN_PB15C_SERCOM4_PAD3            47L  /**< \brief SERCOM4 signal: PAD3 on PB15 mux C */
#define MUX_PB15C_SERCOM4_PAD3             2L
#define PINMUX_PB15C_SERCOM4_PAD3  ((PIN_PB15C_SERCOM4_PAD3 << 16) | MUX_PB15C_SERCOM4_PAD3)
#define PORT_PB15C_SERCOM4_PAD3    (1ul << 15)
#define PIN_PC18F_SERCOM4_PAD3            82L  /**< \brief SERCOM4 signal: PAD3 on PC18 mux F */
#define MUX_PC18F_SERCOM4_PAD3             5L
#define PINMUX_PC18F_SERCOM4_PAD3  ((PIN_PC18F_SERCOM4_PAD3 << 16) | MUX_PC18F_SERCOM4_PAD3)
#define PORT_PC18F_SERCOM4_PAD3    (1ul << 18)
/* ========== PORT definition for TCC0 peripheral ========== */
#define PIN_PA04E_TCC0_WO0                 4L  /**< \brief TCC0 signal: WO0 on PA04 mux E */
#define MUX_PA04E_TCC0_WO0                 4L
#define PINMUX_PA04E_TCC0_WO0      ((PIN_PA04E_TCC0_WO0 << 16) | MUX_PA04E_TCC0_WO0)
#define PORT_PA04E_TCC0_WO0        (1ul <<  4)
#define PIN_PA08E_TCC0_WO0                 8L  /**< \brief TCC0 signal: WO0 on PA08 mux E */
#define MUX_PA08E_TCC0_WO0                 4L
#define PINMUX_PA08E_TCC0_WO0      ((PIN_PA08E_TCC0_WO0 << 16) | MUX_PA08E_TCC0_WO0)
#define PORT_PA08E_TCC0_WO0        (1ul <<  8)
#define PIN_PB30E_TCC0_WO0                62L  /**< \brief TCC0 signal: WO0 on PB30 mux E */
#define MUX_PB30E_TCC0_WO0                 4L
#define PINMUX_PB30E_TCC0_WO0      ((PIN_PB30E_TCC0_WO0 << 16) | MUX_PB30E_TCC0_WO0)
#define PORT_PB30E_TCC0_WO0        (1ul << 30)
#define PIN_PA16F_TCC0_WO0                16L  /**< \brief TCC0 signal: WO0 on PA16 mux F */
#define MUX_PA16F_TCC0_WO0                 5L
#define PINMUX_PA16F_TCC0_WO0      ((PIN_PA16F_TCC0_WO0 << 16) | MUX_PA16F_TCC0_WO0)
#define PORT_PA16F_TCC0_WO0        (1ul << 16)
#define PIN_PA05E_TCC0_WO1                 5L  /**< \brief TCC0 signal: WO1 on PA05 mux E */
#define MUX_PA05E_TCC0_WO1                 4L
#define PINMUX_PA05E_TCC0_WO1      ((PIN_PA05E_TCC0_WO1 << 16) | MUX_PA05E_TCC0_WO1)
#define PORT_PA05E_TCC0_WO1        (1ul <<  5)
#define PIN_PA09E_TCC0_WO1                 9L  /**< \brief TCC0 signal: WO1 on PA09 mux E */
#define MUX_PA09E_TCC0_WO1                 4L
#define PINMUX_PA09E_TCC0_WO1      ((PIN_PA09E_TCC0_WO1 << 16) | MUX_PA09E_TCC0_WO1)
#define PORT_PA09E_TCC0_WO1        (1ul <<  9)
#define PIN_PB31E_TCC0_WO1                63L  /**< \brief TCC0 signal: WO1 on PB31 mux E */
#define MUX_PB31E_TCC0_WO1                 4L
#define PINMUX_PB31E_TCC0_WO1      ((PIN_PB31E_TCC0_WO1 << 16) | MUX_PB31E_TCC0_WO1)
#define PORT_PB31E_TCC0_WO1        (1ul << 31)
#define PIN_PA17F_TCC0_WO1                17L  /**< \brief TCC0 signal: WO1 on PA17 mux F */
#define MUX_PA17F_TCC0_WO1                 5L
#define PINMUX_PA17F_TCC0_WO1      ((PIN_PA17F_TCC0_WO1 << 16) | MUX_PA17F_TCC0_WO1)
#define PORT_PA17F_TCC0_WO1        (1ul << 17)
#define PIN_PA10F_TCC0_WO2                10L  /**< \brief TCC0 signal: WO2 on PA10 mux F */
#define MUX_PA10F_TCC0_WO2                 5L
#define PINMUX_PA10F_TCC0_WO2      ((PIN_PA10F_TCC0_WO2 << 16) | MUX_PA10F_TCC0_WO2)
#define PORT_PA10F_TCC0_WO2        (1ul << 10)
#define PIN_PA18F_TCC0_WO2                18L  /**< \brief TCC0 signal: WO2 on PA18 mux F */
#define MUX_PA18F_TCC0_WO2                 5L
#define PINMUX_PA18F_TCC0_WO2      ((PIN_PA18F_TCC0_WO2 << 16) | MUX_PA18F_TCC0_WO2)
#define PORT_PA18F_TCC0_WO2        (1ul << 18)
#define PIN_PA11F_TCC0_WO3                11L  /**< \brief TCC0 signal: WO3 on PA11 mux F */
#define MUX_PA11F_TCC0_WO3                 5L
#define PINMUX_PA11F_TCC0_WO3      ((PIN_PA11F_TCC0_WO3 << 16) | MUX_PA11F_TCC0_WO3)
#define PORT_PA11F_TCC0_WO3        (1ul << 11)
#define PIN_PA19F_TCC0_WO3                19L  /**< \brief TCC0 signal: WO3 on PA19 mux F */
#define MUX_PA19F_TCC0_WO3                 5L
#define PINMUX_PA19F_TCC0_WO3      ((PIN_PA19F_TCC0_WO3 << 16) | MUX_PA19F_TCC0_WO3)
#define PORT_PA19F_TCC0_WO3        (1ul << 19)
#define PIN_PA22F_TCC0_WO4                22L  /**< \brief TCC0 signal: WO4 on PA22 mux F */
#define MUX_PA22F_TCC0_WO4                 5L
#define PINMUX_PA22F_TCC0_WO4      ((PIN_PA22F_TCC0_WO4 << 16) | MUX_PA22F_TCC0_WO4)
#define PORT_PA22F_TCC0_WO4        (1ul << 22)
#define PIN_PB16F_TCC0_WO4                48L  /**< \brief TCC0 signal: WO4 on PB16 mux F */
#define MUX_PB16F_TCC0_WO4                 5L
#define PINMUX_PB16F_TCC0_WO4      ((PIN_PB16F_TCC0_WO4 << 16) | MUX_PB16F_TCC0_WO4)
#define PORT_PB16F_TCC0_WO4        (1ul << 16)
#define PIN_PA14F_TCC0_WO4                14L  /**< \brief TCC0 signal: WO4 on PA14 mux F */
#define MUX_PA14F_TCC0_WO4                 5L
#define PINMUX_PA14F_TCC0_WO4      ((PIN_PA14F_TCC0_WO4 << 16) | MUX_PA14F_TCC0_WO4)
#define PORT_PA14F_TCC0_WO4        (1ul << 14)
#define PIN_PA15F_TCC0_WO5                15L  /**< \brief TCC0 signal: WO5 on PA15 mux F */
#define MUX_PA15F_TCC0_WO5                 5L
#define PINMUX_PA15F_TCC0_WO5      ((PIN_PA15F_TCC0_WO5 << 16) | MUX_PA15F_TCC0_WO5)
#define PORT_PA15F_TCC0_WO5        (1ul << 15)
#define PIN_PA23F_TCC0_WO5                23L  /**< \brief TCC0 signal: WO5 on PA23 mux F */
#define MUX_PA23F_TCC0_WO5                 5L
#define PINMUX_PA23F_TCC0_WO5      ((PIN_PA23F_TCC0_WO5 << 16) | MUX_PA23F_TCC0_WO5)
#define PORT_PA23F_TCC0_WO5        (1ul << 23)
#define PIN_PB17F_TCC0_WO5                49L  /**< \brief TCC0 signal: WO5 on PB17 mux F */
#define MUX_PB17F_TCC0_WO5                 5L
#define PINMUX_PB17F_TCC0_WO5      ((PIN_PB17F_TCC0_WO5 << 16) | MUX_PB17F_TCC0_WO5)
#define PORT_PB17F_TCC0_WO5        (1ul << 17)
#define PIN_PA12F_TCC0_WO6                12L  /**< \brief TCC0 signal: WO6 on PA12 mux F */
#define MUX_PA12F_TCC0_WO6                 5L
#define PINMUX_PA12F_TCC0_WO6      ((PIN_PA12F_TCC0_WO6 << 16) | MUX_PA12F_TCC0_WO6)
#define PORT_PA12F_TCC0_WO6        (1ul << 12)
#define PIN_PA16F_TCC0_WO6                16L  /**< \brief TCC0 signal: WO6 on PA16 mux F */
#define MUX_PA16F_TCC0_WO6                 5L
#define PINMUX_PA16F_TCC0_WO6      ((PIN_PA16F_TCC0_WO6 << 16) | MUX_PA16F_TCC0_WO6)
#define PORT_PA16F_TCC0_WO6        (1ul << 16)
#define PIN_PA20F_TCC0_WO6                20L  /**< \brief TCC0 signal: WO6 on PA20 mux F */
#define MUX_PA20F_TCC0_WO6                 5L
#define PINMUX_PA20F_TCC0_WO6      ((PIN_PA20F_TCC0_WO6 << 16) | MUX_PA20F_TCC0_WO6)
#define PORT_PA20F_TCC0_WO6        (1ul << 20)
#define PIN_PA13F_TCC0_WO7                13L  /**< \brief TCC0 signal: WO7 on PA13 mux F */
#define MUX_PA13F_TCC0_WO7                 5L
#define PINMUX_PA13F_TCC0_WO7      ((PIN_PA13F_TCC0_WO7 << 16) | MUX_PA13F_TCC0_WO7)
#define PORT_PA13F_TCC0_WO7        (1ul << 13)
#define PIN_PA17F_TCC0_WO7                17L  /**< \brief TCC0 signal: WO7 on PA17 mux F */
#define MUX_PA17F_TCC0_WO7                 5L
#define PINMUX_PA17F_TCC0_WO7      ((PIN_PA17F_TCC0_WO7 << 16) | MUX_PA17F_TCC0_WO7)
#define PORT_PA17F_TCC0_WO7        (1ul << 17)
/* ========== PORT definition for TCC1 peripheral ========== */
#define PIN_PA06E_TCC1_WO0                 6L  /**< \brief TCC1 signal: WO0 on PA06 mux E */
#define MUX_PA06E_TCC1_WO0                 4L
#define PINMUX_PA06E_TCC1_WO0      ((PIN_PA06E_TCC1_WO0 << 16) | MUX_PA06E_TCC1_WO0)
#define PORT_PA06E_TCC1_WO0        (1ul <<  6)
#define PIN_PA10E_TCC1_WO0                10L  /**< \brief TCC1 signal: WO0 on PA10 mux E */
#define MUX_PA10E_TCC1_WO0                 4L
#define PINMUX_PA10E_TCC1_WO0      ((PIN_PA10E_TCC1_WO0 << 16) | MUX_PA10E_TCC1_WO0)
#define PORT_PA10E_TCC1_WO0        (1ul << 10)
#define PIN_PA30E_TCC1_WO0                30L  /**< \brief TCC1 signal: WO0 on PA30 mux E */
#define MUX_PA30E_TCC1_WO0                 4L
#define PINMUX_PA30E_TCC1_WO0      ((PIN_PA30E_TCC1_WO0 << 16) | MUX_PA30E_TCC1_WO0)
#define PORT_PA30E_TCC1_WO0        (1ul << 30)
#define PIN_PA07E_TCC1_WO1                 7L  /**< \brief TCC1 signal: WO1 on PA07 mux E */
#define MUX_PA07E_TCC1_WO1                 4L
#define PINMUX_PA07E_TCC1_WO1      ((PIN_PA07E_TCC1_WO1 << 16) | MUX_PA07E_TCC1_WO1)
#define PORT_PA07E_TCC1_WO1        (1ul <<  7)
#define PIN_PA11E_TCC1_WO1                11L  /**< \brief TCC1 signal: WO1 on PA11 mux E */
#define MUX_PA11E_TCC1_WO1                 4L
#define PINMUX_PA11E_TCC1_WO1      ((PIN_PA11E_TCC1_WO1 << 16) | MUX_PA11E_TCC1_WO1)
#define PORT_PA11E_TCC1_WO1        (1ul << 11)
#define PIN_PA31E_TCC1_WO1                31L  /**< \brief TCC1 signal: WO1 on PA31 mux E */
#define MUX_PA31E_TCC1_WO1                 4L
#define PINMUX_PA31E_TCC1_WO1      ((PIN_PA31E_TCC1_WO1 << 16) | MUX_PA31E_TCC1_WO1)
#define PORT_PA31E_TCC1_WO1        (1ul << 31)
#define PIN_PA08F_TCC1_WO2                 8L  /**< \brief TCC1 signal: WO2 on PA08 mux F */
#define MUX_PA08F_TCC1_WO2                 5L
#define PINMUX_PA08F_TCC1_WO2      ((PIN_PA08F_TCC1_WO2 << 16) | MUX_PA08F_TCC1_WO2)
#define PORT_PA08F_TCC1_WO2        (1ul <<  8)
#define PIN_PA24F_TCC1_WO2                24L  /**< \brief TCC1 signal: WO2 on PA24 mux F */
#define MUX_PA24F_TCC1_WO2                 5L
#define PINMUX_PA24F_TCC1_WO2      ((PIN_PA24F_TCC1_WO2 << 16) | MUX_PA24F_TCC1_WO2)
#define PORT_PA24F_TCC1_WO2        (1ul << 24)
#define PIN_PB30F_TCC1_WO2                62L  /**< \brief TCC1 signal: WO2 on PB30 mux F */
#define MUX_PB30F_TCC1_WO2                 5L
#define PINMUX_PB30F_TCC1_WO2      ((PIN_PB30F_TCC1_WO2 << 16) | MUX_PB30F_TCC1_WO2)
#define PORT_PB30F_TCC1_WO2        (1ul << 30)
#define PIN_PA09F_TCC1_WO3                 9L  /**< \brief TCC1 signal: WO3 on PA09 mux F */
#define MUX_PA09F_TCC1_WO3                 5L
#define PINMUX_PA09F_TCC1_WO3      ((PIN_PA09F_TCC1_WO3 << 16) | MUX_PA09F_TCC1_WO3)
#define PORT_PA09F_TCC1_WO3        (1ul <<  9)
#define PIN_PA25F_TCC1_WO3                25L  /**< \brief TCC1 signal: WO3 on PA25 mux F */
#define MUX_PA25F_TCC1_WO3                 5L
#define PINMUX_PA25F_TCC1_WO3      ((PIN_PA25F_TCC1_WO3 << 16) | MUX_PA25F_TCC1_WO3)
#define PORT_PA25F_TCC1_WO3        (1ul << 25)
#define PIN_PB31F_TCC1_WO3                63L  /**< \brief TCC1 signal: WO3 on PB31 mux F */
#define MUX_PB31F_TCC1_WO3                 5L
#define PINMUX_PB31F_TCC1_WO3      ((PIN_PB31F_TCC1_WO3 << 16) | MUX_PB31F_TCC1_WO3)
#define PORT_PB31F_TCC1_WO3        (1ul << 31)
/* ========== PORT definition for TCC2 peripheral ========== */
#define PIN_PA12E_TCC2_WO0                12L  /**< \brief TCC2 signal: WO0 on PA12 mux E */
#define MUX_PA12E_TCC2_WO0                 4L
#define PINMUX_PA12E_TCC2_WO0      ((PIN_PA12E_TCC2_WO0 << 16) | MUX_PA12E_TCC2_WO0)
#define PORT_PA12E_TCC2_WO0        (1ul << 12)
#define PIN_PA16E_TCC2_WO0                16L  /**< \brief TCC2 signal: WO0 on PA16 mux E */
#define MUX_PA16E_TCC2_WO0                 4L
#define PINMUX_PA16E_TCC2_WO0      ((PIN_PA16E_TCC2_WO0 << 16) | MUX_PA16E_TCC2_WO0)
#define PORT_PA16E_TCC2_WO0        (1ul << 16)
#define PIN_PA00E_TCC2_WO0                 0L  /**< \brief TCC2 signal: WO0 on PA00 mux E */
#define MUX_PA00E_TCC2_WO0                 4L
#define PINMUX_PA00E_TCC2_WO0      ((PIN_PA00E_TCC2_WO0 << 16) | MUX_PA00E_TCC2_WO0)
#define PORT_PA00E_TCC2_WO0        (1ul <<  0)
#define PIN_PA13E_TCC2_WO1                13L  /**< \brief TCC2 signal: WO1 on PA13 mux E */
#define MUX_PA13E_TCC2_WO1                 4L
#define PINMUX_PA13E_TCC2_WO1      ((PIN_PA13E_TCC2_WO1 << 16) | MUX_PA13E_TCC2_WO1)
#define PORT_PA13E_TCC2_WO1        (1ul << 13)
#define PIN_PA17E_TCC2_WO1                17L  /**< \brief TCC2 signal: WO1 on PA17 mux E */
#define MUX_PA17E_TCC2_WO1                 4L
#define PINMUX_PA17E_TCC2_WO1      ((PIN_PA17E_TCC2_WO1 << 16) | MUX_PA17E_TCC2_WO1)
#define PORT_PA17E_TCC2_WO1        (1ul << 17)
#define PIN_PA01E_TCC2_WO1                 1L  /**< \brief TCC2 signal: WO1 on PA01 mux E */
#define MUX_PA01E_TCC2_WO1                 4L
#define PINMUX_PA01E_TCC2_WO1      ((PIN_PA01E_TCC2_WO1 << 16) | MUX_PA01E_TCC2_WO1)
#define PORT_PA01E_TCC2_WO1        (1ul <<  1)
/* ========== PORT definition for TC0 peripheral ========== */
#define PIN_PA22E_TC0_WO0                 22L  /**< \brief TC0 signal: WO0 on PA22 mux E */
#define MUX_PA22E_TC0_WO0                  4L
#define PINMUX_PA22E_TC0_WO0       ((PIN_PA22E_TC0_WO0 << 16) | MUX_PA22E_TC0_WO0)
#define PORT_PA22E_TC0_WO0         (1ul << 22)
#define PIN_PB08E_TC0_WO0                 40L  /**< \brief TC0 signal: WO0 on PB08 mux E */
#define MUX_PB08E_TC0_WO0                  4L
#define PINMUX_PB08E_TC0_WO0       ((PIN_PB08E_TC0_WO0 << 16) | MUX_PB08E_TC0_WO0)
#define PORT_PB08E_TC0_WO0         (1ul <<  8)
#define PIN_PA23E_TC0_WO1                 23L  /**< \brief TC0 signal: WO1 on PA23 mux E */
#define MUX_PA23E_TC0_WO1                  4L
#define PINMUX_PA23E_TC0_WO1       ((PIN_PA23E_TC0_WO1 << 16) | MUX_PA23E_TC0_WO1)
#define PORT_PA23E_TC0_WO1         (1ul << 23)
#define PIN_PB09E_TC0_WO1                 41L  /**< \brief TC0 signal: WO1 on PB09 mux E */
#define MUX_PB09E_TC0_WO1                  4L
#define PINMUX_PB09E_TC0_WO1       ((PIN_PB09E_TC0_WO1 << 16) | MUX_PB09E_TC0_WO1)
#define PORT_PB09E_TC0_WO1         (1ul <<  9)
/* ========== PORT definition for TC1 peripheral ========== */
#define PIN_PA24E_TC1_WO0                 24L  /**< \brief TC1 signal: WO0 on PA24 mux E */
#define MUX_PA24E_TC1_WO0                  4L
#define PINMUX_PA24E_TC1_WO0       ((PIN_PA24E_TC1_WO0 << 16) | MUX_PA24E_TC1_WO0)
#define PORT_PA24E_TC1_WO0         (1ul << 24)
#define PIN_PB14E_TC1_WO0                 46L  /**< \brief TC1 signal: WO0 on PB14 mux E */
#define MUX_PB14E_TC1_WO0                  4L
#define PINMUX_PB14E_TC1_WO0       ((PIN_PB14E_TC1_WO0 << 16) | MUX_PB14E_TC1_WO0)
#define PORT_PB14E_TC1_WO0         (1ul << 14)
#define PIN_PA25E_TC1_WO1                 25L  /**< \brief TC1 signal: WO1 on PA25 mux E */
#define MUX_PA25E_TC1_WO1                  4L
#define PINMUX_PA25E_TC1_WO1       ((PIN_PA25E_TC1_WO1 << 16) | MUX_PA25E_TC1_WO1)
#define PORT_PA25E_TC1_WO1         (1ul << 25)
#define PIN_PB15E_TC1_WO1                 47L  /**< \brief TC1 signal: WO1 on PB15 mux E */
#define MUX_PB15E_TC1_WO1                  4L
#define PINMUX_PB15E_TC1_WO1       ((PIN_PB15E_TC1_WO1 << 16) | MUX_PB15E_TC1_WO1)
#define PORT_PB15E_TC1_WO1         (1ul << 15)
/* ========== PORT definition for SERCOM5 peripheral ========== */
#define PIN_PA22D_SERCOM5_PAD0            22L  /**< \brief SERCOM5 signal: PAD0 on PA22 mux D */
#define MUX_PA22D_SERCOM5_PAD0             3L
#define PINMUX_PA22D_SERCOM5_PAD0  ((PIN_PA22D_SERCOM5_PAD0 << 16) | MUX_PA22D_SERCOM5_PAD0)
#define PORT_PA22D_SERCOM5_PAD0    (1ul << 22)
#define PIN_PB02D_SERCOM5_PAD0            34L  /**< \brief SERCOM5 signal: PAD0 on PB02 mux D */
#define MUX_PB02D_SERCOM5_PAD0             3L
#define PINMUX_PB02D_SERCOM5_PAD0  ((PIN_PB02D_SERCOM5_PAD0 << 16) | MUX_PB02D_SERCOM5_PAD0)
#define PORT_PB02D_SERCOM5_PAD0    (1ul <<  2)
#define PIN_PB30D_SERCOM5_PAD0            62L  /**< \brief SERCOM5 signal: PAD0 on PB30 mux D */
#define MUX_PB30D_SERCOM5_PAD0             3L
#define PINMUX_PB30D_SERCOM5_PAD0  ((PIN_PB30D_SERCOM5_PAD0 << 16) | MUX_PB30D_SERCOM5_PAD0)
#define PORT_PB30D_SERCOM5_PAD0    (1ul << 30)
#define PIN_PB16C_SERCOM5_PAD0            48L  /**< \brief SERCOM5 signal: PAD0 on PB16 mux C */
#define MUX_PB16C_SERCOM5_PAD0             2L
#define PINMUX_PB16C_SERCOM5_PAD0  ((PIN_PB16C_SERCOM5_PAD0 << 16) | MUX_PB16C_SERCOM5_PAD0)
#define PORT_PB16C_SERCOM5_PAD0    (1ul << 16)
#define PIN_PA23D_SERCOM5_PAD1            23L  /**< \brief SERCOM5 signal: PAD1 on PA23 mux D */
#define MUX_PA23D_SERCOM5_PAD1             3L
#define PINMUX_PA23D_SERCOM5_PAD1  ((PIN_PA23D_SERCOM5_PAD1 << 16) | MUX_PA23D_SERCOM5_PAD1)
#define PORT_PA23D_SERCOM5_PAD1    (1ul << 23)
#define PIN_PB03D_SERCOM5_PAD1            35L  /**< \brief SERCOM5 signal: PAD1 on PB03 mux D */
#define MUX_PB03D_SERCOM5_PAD1             3L
#define PINMUX_PB03D_SERCOM5_PAD1  ((PIN_PB03D_SERCOM5_PAD1 << 16) | MUX_PB03D_SERCOM5_PAD1)
#define PORT_PB03D_SERCOM5_PAD1    (1ul <<  3)
#define PIN_PB31D_SERCOM5_PAD1            63L  /**< \brief SERCOM5 signal: PAD1 on PB31 mux D */
#define MUX_PB31D_SERCOM5_PAD1             3L
#define PINMUX_PB31D_SERCOM5_PAD1  ((PIN_PB31D_SERCOM5_PAD1 << 16) | MUX_PB31D_SERCOM5_PAD1)
#define PORT_PB31D_SERCOM5_PAD1    (1ul << 31)
#define PIN_PB17C_SERCOM5_PAD1            49L  /**< \brief SERCOM5 signal: PAD1 on PB17 mux C */
#define MUX_PB17C_SERCOM5_PAD1             2L
#define PINMUX_PB17C_SERCOM5_PAD1  ((PIN_PB17C_SERCOM5_PAD1 << 16) | MUX_PB17C_SERCOM5_PAD1)
#define PORT_PB17C_SERCOM5_PAD1    (1ul << 17)
#define PIN_PA24D_SERCOM5_PAD2            24L  /**< \brief SERCOM5 signal: PAD2 on PA24 mux D */
#define MUX_PA24D_SERCOM5_PAD2             3L
#define PINMUX_PA24D_SERCOM5_PAD2  ((PIN_PA24D_SERCOM5_PAD2 << 16) | MUX_PA24D_SERCOM5_PAD2)
#define PORT_PA24D_SERCOM5_PAD2    (1ul << 24)
#define PIN_PB00D_SERCOM5_PAD2            32L  /**< \brief SERCOM5 signal: PAD2 on PB00 mux D */
#define MUX_PB00D_SERCOM5_PAD2             3L
#define PINMUX_PB00D_SERCOM5_PAD2  ((PIN_PB00D_SERCOM5_PAD2 << 16) | MUX_PB00D_SERCOM5_PAD2)
#define PORT_PB00D_SERCOM5_PAD2    (1ul <<  0)
#define PIN_PB22D_SERCOM5_PAD2            54L  /**< \brief SERCOM5 signal: PAD2 on PB22 mux D */
#define MUX_PB22D_SERCOM5_PAD2             3L
#define PINMUX_PB22D_SERCOM5_PAD2  ((PIN_PB22D_SERCOM5_PAD2 << 16) | MUX_PB22D_SERCOM5_PAD2)
#define PORT_PB22D_SERCOM5_PAD2    (1ul << 22)
#define PIN_PA20C_SERCOM5_PAD2            20L  /**< \brief SERCOM5 signal: PAD2 on PA20 mux C */
#define MUX_PA20C_SERCOM5_PAD2             2L
#define PINMUX_PA20C_SERCOM5_PAD2  ((PIN_PA20C_SERCOM5_PAD2 << 16) | MUX_PA20C_SERCOM5_PAD2)
#define PORT_PA20C_SERCOM5_PAD2    (1ul << 20)
#define PIN_PA25D_SERCOM5_PAD3            25L  /**< \brief SERCOM5 signal: PAD3 on PA25 mux D */
#define MUX_PA25D_SERCOM5_PAD3             3L
#define PINMUX_PA25D_SERCOM5_PAD3  ((PIN_PA25D_SERCOM5_PAD3 << 16) | MUX_PA25D_SERCOM5_PAD3)
#define PORT_PA25D_SERCOM5_PAD3    (1ul << 25)
#define PIN_PB23D_SERCOM5_PAD3            55L  /**< \brief SERCOM5 signal: PAD3 on PB23 mux D */
#define MUX_PB23D_SERCOM5_PAD3             3L
#define PINMUX_PB23D_SERCOM5_PAD3  ((PIN_PB23D_SERCOM5_PAD3 << 16) | MUX_PB23D_SERCOM5_PAD3)
#define PORT_PB23D_SERCOM5_PAD3    (1ul << 23)
/* ========== PORT definition for TC4 peripheral ========== */
#define PIN_PA18E_TC4_WO0                 18L  /**< \brief TC4 signal: WO0 on PA18 mux E */
#define MUX_PA18E_TC4_WO0                  4L
#define PINMUX_PA18E_TC4_WO0       ((PIN_PA18E_TC4_WO0 << 16) | MUX_PA18E_TC4_WO0)
#define PORT_PA18E_TC4_WO0         (1ul << 18)
#define PIN_PA14E_TC4_WO0                 14L  /**< \brief TC4 signal: WO0 on PA14 mux E */
#define MUX_PA14E_TC4_WO0                  4L
#define PINMUX_PA14E_TC4_WO0       ((PIN_PA14E_TC4_WO0 << 16) | MUX_PA14E_TC4_WO0)
#define PORT_PA14E_TC4_WO0         (1ul << 14)
#define PIN_PA19E_TC4_WO1                 19L  /**< \brief TC4 signal: WO1 on PA19 mux E */
#define MUX_PA19E_TC4_WO1                  4L
#define PINMUX_PA19E_TC4_WO1       ((PIN_PA19E_TC4_WO1 << 16) | MUX_PA19E_TC4_WO1)
#define PORT_PA19E_TC4_WO1         (1ul << 19)
#define PIN_PA15E_TC4_WO1                 15L  /**< \brief TC4 signal: WO1 on PA15 mux E */
#define MUX_PA15E_TC4_WO1                  4L
#define PINMUX_PA15E_TC4_WO1       ((PIN_PA15E_TC4_WO1 << 16) | MUX_PA15E_TC4_WO1)
#define PORT_PA15E_TC4_WO1         (1ul << 15)
/* ========== PORT definition for ADC peripheral ========== */
#define PIN_PB08B_ADC_AIN2                40L  /**< \brief ADC signal: AIN2 on PB08 mux B */
#define MUX_PB08B_ADC_AIN2                 1L
#define PINMUX_PB08B_ADC_AIN2      ((PIN_PB08B_ADC_AIN2 << 16) | MUX_PB08B_ADC_AIN2)
#define PORT_PB08B_ADC_AIN2        (1ul <<  8)
#define PIN_PB09B_ADC_AIN3                41L  /**< \brief ADC signal: AIN3 on PB09 mux B */
#define MUX_PB09B_ADC_AIN3                 1L
#define PINMUX_PB09B_ADC_AIN3      ((PIN_PB09B_ADC_AIN3 << 16) | MUX_PB09B_ADC_AIN3)
#define PORT_PB09B_ADC_AIN3        (1ul <<  9)
#define PIN_PA04B_ADC_AIN4                 4L  /**< \brief ADC signal: AIN4 on PA04 mux B */
#define MUX_PA04B_ADC_AIN4                 1L
#define PINMUX_PA04B_ADC_AIN4      ((PIN_PA04B_ADC_AIN4 << 16) | MUX_PA04B_ADC_AIN4)
#define PORT_PA04B_ADC_AIN4        (1ul <<  4)
#define PIN_PA05B_ADC_AIN5                 5L  /**< \brief ADC signal: AIN5 on PA05 mux B */
#define MUX_PA05B_ADC_AIN5                 1L
#define PINMUX_PA05B_ADC_AIN5      ((PIN_PA05B_ADC_AIN5 << 16) | MUX_PA05B_ADC_AIN5)
#define PORT_PA05B_ADC_AIN5        (1ul <<  5)
#define PIN_PA06B_ADC_AIN6                 6L  /**< \brief ADC signal: AIN6 on PA06 mux B */
#define MUX_PA06B_ADC_AIN6                 1L
#define PINMUX_PA06B_ADC_AIN6      ((PIN_PA06B_ADC_AIN6 << 16) | MUX_PA06B_ADC_AIN6)
#define PORT_PA06B_ADC_AIN6        (1ul <<  6)
#define PIN_PA07B_ADC_AIN7                 7L  /**< \brief ADC signal: AIN7 on PA07 mux B */
#define MUX_PA07B_ADC_AIN7                 1L
#define PINMUX_PA07B_ADC_AIN7      ((PIN_PA07B_ADC_AIN7 << 16) | MUX_PA07B_ADC_AIN7)
#define PORT_PA07B_ADC_AIN7        (1ul <<  7)
#define PIN_PB00B_ADC_AIN8                32L  /**< \brief ADC signal: AIN8 on PB00 mux B */
#define MUX_PB00B_ADC_AIN8                 1L
#define PINMUX_PB00B_ADC_AIN8      ((PIN_PB00B_ADC_AIN8 << 16) | MUX_PB00B_ADC_AIN8)
#define PORT_PB00B_ADC_AIN8        (1ul <<  0)
#define PIN_PB02B_ADC_AIN10               34L  /**< \brief ADC signal: AIN10 on PB02 mux B */
#define MUX_PB02B_ADC_AIN10                1L
#define PINMUX_PB02B_ADC_AIN10     ((PIN_PB02B_ADC_AIN10 << 16) | MUX_PB02B_ADC_AIN10)
#define PORT_PB02B_ADC_AIN10       (1ul <<  2)
#define PIN_PB03B_ADC_AIN11               35L  /**< \brief ADC signal: AIN11 on PB03 mux B */
#define MUX_PB03B_ADC_AIN11                1L
#define PINMUX_PB03B_ADC_AIN11     ((PIN_PB03B_ADC_AIN11 << 16) | MUX_PB03B_ADC_AIN11)
#define PORT_PB03B_ADC_AIN11       (1ul <<  3)
#define PIN_PA08B_ADC_AIN16                8L  /**< \brief ADC signal: AIN16 on PA08 mux B */
#define MUX_PA08B_ADC_AIN16                1L
#define PINMUX_PA08B_ADC_AIN16     ((PIN_PA08B_ADC_AIN16 << 16) | MUX_PA08B_ADC_AIN16)
#define PORT_PA08B_ADC_AIN16       (1ul <<  8)
#define PIN_PA09B_ADC_AIN17                9L  /**< \brief ADC signal: AIN17 on PA09 mux B */
#define MUX_PA09B_ADC_AIN17                1L
#define PINMUX_PA09B_ADC_AIN17     ((PIN_PA09B_ADC_AIN17 << 16) | MUX_PA09B_ADC_AIN17)
#define PORT_PA09B_ADC_AIN17       (1ul <<  9)
#define PIN_PA10B_ADC_AIN18               10L  /**< \brief ADC signal: AIN18 on PA10 mux B */
#define MUX_PA10B_ADC_AIN18                1L
#define PINMUX_PA10B_ADC_AIN18     ((PIN_PA10B_ADC_AIN18 << 16) | MUX_PA10B_ADC_AIN18)
#define PORT_PA10B_ADC_AIN18       (1ul << 10)
#define PIN_PA11B_ADC_AIN19               11L  /**< \brief ADC signal: AIN19 on PA11 mux B */
#define MUX_PA11B_ADC_AIN19                1L
#define PINMUX_PA11B_ADC_AIN19     ((PIN_PA11B_ADC_AIN19 << 16) | MUX_PA11B_ADC_AIN19)
#define PORT_PA11B_ADC_AIN19       (1ul << 11)
#define PIN_PA04B_ADC_VREFP                4L  /**< \brief ADC signal: VREFP on PA04 mux B */
#define MUX_PA04B_ADC_VREFP                1L
#define PINMUX_PA04B_ADC_VREFP     ((PIN_PA04B_ADC_VREFP << 16) | MUX_PA04B_ADC_VREFP)
#define PORT_PA04B_ADC_VREFP       (1ul <<  4)
/* ========== PORT definition for AC peripheral ========== */
#define PIN_PA04B_AC_AIN0                  4L  /**< \brief AC signal: AIN0 on PA04 mux B */
#define MUX_PA04B_AC_AIN0                  1L
#define PINMUX_PA04B_AC_AIN0       ((PIN_PA04B_AC_AIN0 << 16) | MUX_PA04B_AC_AIN0)
#define PORT_PA04B_AC_AIN0         (1ul <<  4)
#define PIN_PA05B_AC_AIN1                  5L  /**< \brief AC signal: AIN1 on PA05 mux B */
#define MUX_PA05B_AC_AIN1                  1L
#define PINMUX_PA05B_AC_AIN1       ((PIN_PA05B_AC_AIN1 << 16) | MUX_PA05B_AC_AIN1)
#define PORT_PA05B_AC_AIN1         (1ul <<  5)
#define PIN_PA06B_AC_AIN2                  6L  /**< \brief AC signal: AIN2 on PA06 mux B */
#define MUX_PA06B_AC_AIN2                  1L
#define PINMUX_PA06B_AC_AIN2       ((PIN_PA06B_AC_AIN2 << 16) | MUX_PA06B_AC_AIN2)
#define PORT_PA06B_AC_AIN2         (1ul <<  6)
#define PIN_PA07B_AC_AIN3                  7L  /**< \brief AC signal: AIN3 on PA07 mux B */
#define MUX_PA07B_AC_AIN3                  1L
#define PINMUX_PA07B_AC_AIN3       ((PIN_PA07B_AC_AIN3 << 16) | MUX_PA07B_AC_AIN3)
#define PORT_PA07B_AC_AIN3         (1ul <<  7)
#define PIN_PA12H_AC_CMP0                 12L  /**< \brief AC signal: CMP0 on PA12 mux H */
#define MUX_PA12H_AC_CMP0                  7L
#define PINMUX_PA12H_AC_CMP0       ((PIN_PA12H_AC_CMP0 << 16) | MUX_PA12H_AC_CMP0)
#define PORT_PA12H_AC_CMP0         (1ul << 12)
#define PIN_PA18H_AC_CMP0                 18L  /**< \brief AC signal: CMP0 on PA18 mux H */
#define MUX_PA18H_AC_CMP0                  7L
#define PINMUX_PA18H_AC_CMP0       ((PIN_PA18H_AC_CMP0 << 16) | MUX_PA18H_AC_CMP0)
#define PORT_PA18H_AC_CMP0         (1ul << 18)
#define PIN_PA13H_AC_CMP1                 13L  /**< \brief AC signal: CMP1 on PA13 mux H */
#define MUX_PA13H_AC_CMP1                  7L
#define PINMUX_PA13H_AC_CMP1       ((PIN_PA13H_AC_CMP1 << 16) | MUX_PA13H_AC_CMP1)
#define PORT_PA13H_AC_CMP1         (1ul << 13)
#define PIN_PA19H_AC_CMP1                 19L  /**< \brief AC signal: CMP1 on PA19 mux H */
#define MUX_PA19H_AC_CMP1                  7L
#define PINMUX_PA19H_AC_CMP1       ((PIN_PA19H_AC_CMP1 << 16) | MUX_PA19H_AC_CMP1)
#define PORT_PA19H_AC_CMP1         (1ul << 19)
/* ========== PORT definition for CCL peripheral ========== */
#define PIN_PA04I_CCL_IN0                  4L  /**< \brief CCL signal: IN0 on PA04 mux I */
#define MUX_PA04I_CCL_IN0                  8L
#define PINMUX_PA04I_CCL_IN0       ((PIN_PA04I_CCL_IN0 << 16) | MUX_PA04I_CCL_IN0)
#define PORT_PA04I_CCL_IN0         (1ul <<  4)
#define PIN_PA16I_CCL_IN0                 16L  /**< \brief CCL signal: IN0 on PA16 mux I */
#define MUX_PA16I_CCL_IN0                  8L
#define PINMUX_PA16I_CCL_IN0       ((PIN_PA16I_CCL_IN0 << 16) | MUX_PA16I_CCL_IN0)
#define PORT_PA16I_CCL_IN0         (1ul << 16)
#define PIN_PB22I_CCL_IN0                 54L  /**< \brief CCL signal: IN0 on PB22 mux I */
#define MUX_PB22I_CCL_IN0                  8L
#define PINMUX_PB22I_CCL_IN0       ((PIN_PB22I_CCL_IN0 << 16) | MUX_PB22I_CCL_IN0)
#define PORT_PB22I_CCL_IN0         (1ul << 22)
#define PIN_PA05I_CCL_IN1                  5L  /**< \brief CCL signal: IN1 on PA05 mux I */
#define MUX_PA05I_CCL_IN1                  8L
#define PINMUX_PA05I_CCL_IN1       ((PIN_PA05I_CCL_IN1 << 16) | MUX_PA05I_CCL_IN1)
#define PORT_PA05I_CCL_IN1         (1ul <<  5)
#define PIN_PA17I_CCL_IN1                 17L  /**< \brief CCL signal: IN1 on PA17 mux I */
#define MUX_PA17I_CCL_IN1                  8L
#define PINMUX_PA17I_CCL_IN1       ((PIN_PA17I_CCL_IN1 << 16) | MUX_PA17I_CCL_IN1)
#define PORT_PA17I_CCL_IN1         (1ul << 17)
#define PIN_PB00I_CCL_IN1                 32L  /**< \brief CCL signal: IN1 on PB00 mux I */
#define MUX_PB00I_CCL_IN1                  8L
#define PINMUX_PB00I_CCL_IN1       ((PIN_PB00I_CCL_IN1 << 16) | MUX_PB00I_CCL_IN1)
#define PORT_PB00I_CCL_IN1         (1ul <<  0)
#define PIN_PA06I_CCL_IN2                  6L  /**< \brief CCL signal: IN2 on PA06 mux I */
#define MUX_PA06I_CCL_IN2                  8L
#define PINMUX_PA06I_CCL_IN2       ((PIN_PA06I_CCL_IN2 << 16) | MUX_PA06I_CCL_IN2)
#define PORT_PA06I_CCL_IN2         (1ul <<  6)
#define PIN_PA18I_CCL_IN2                 18L  /**< \brief CCL signal: IN2 on PA18 mux I */
#define MUX_PA18I_CCL_IN2                  8L
#define PINMUX_PA18I_CCL_IN2       ((PIN_PA18I_CCL_IN2 << 16) | MUX_PA18I_CCL_IN2)
#define PORT_PA18I_CCL_IN2         (1ul << 18)
#define PIN_PA08I_CCL_IN3                  8L  /**< \brief CCL signal: IN3 on PA08 mux I */
#define MUX_PA08I_CCL_IN3                  8L
#define PINMUX_PA08I_CCL_IN3       ((PIN_PA08I_CCL_IN3 << 16) | MUX_PA08I_CCL_IN3)
#define PORT_PA08I_CCL_IN3         (1ul <<  8)
#define PIN_PA30I_CCL_IN3                 30L  /**< \brief CCL signal: IN3 on PA30 mux I */
#define MUX_PA30I_CCL_IN3                  8L
#define PINMUX_PA30I_CCL_IN3       ((PIN_PA30I_CCL_IN3 << 16) | MUX_PA30I_CCL_IN3)
#define PORT_PA30I_CCL_IN3         (1ul << 30)
#define PIN_PA09I_CCL_IN4                  9L  /**< \brief CCL signal: IN4 on PA09 mux I */
#define MUX_PA09I_CCL_IN4                  8L
#define PINMUX_PA09I_CCL_IN4       ((PIN_PA09I_CCL_IN4 << 16) | MUX_PA09I_CCL_IN4)
#define PORT_PA09I_CCL_IN4         (1ul <<  9)
#define PIN_PA10I_CCL_IN5                 10L  /**< \brief CCL signal: IN5 on PA10 mux I */
#define MUX_PA10I_CCL_IN5                  8L
#define PINMUX_PA10I_CCL_IN5       ((PIN_PA10I_CCL_IN5 << 16) | MUX_PA10I_CCL_IN5)
#define PORT_PA10I_CCL_IN5         (1ul << 10)
#define PIN_PA22I_CCL_IN6                 22L  /**< \brief CCL signal: IN6 on PA22 mux I */
#define MUX_PA22I_CCL_IN6                  8L
#define PINMUX_PA22I_CCL_IN6       ((PIN_PA22I_CCL_IN6 << 16) | MUX_PA22I_CCL_IN6)
#define PORT_PA22I_CCL_IN6         (1ul << 22)
#define PIN_PA23I_CCL_IN7                 23L  /**< \brief CCL signal: IN7 on PA23 mux I */
#define MUX_PA23I_CCL_IN7                  8L
#define PINMUX_PA23I_CCL_IN7       ((PIN_PA23I_CCL_IN7 << 16) | MUX_PA23I_CCL_IN7)
#define PORT_PA23I_CCL_IN7         (1ul << 23)
#define PIN_PA24I_CCL_IN8                 24L  /**< \brief CCL signal: IN8 on PA24 mux I */
#define MUX_PA24I_CCL_IN8                  8L
#define PINMUX_PA24I_CCL_IN8       ((PIN_PA24I_CCL_IN8 << 16) | MUX_PA24I_CCL_IN8)
#define PORT_PA24I_CCL_IN8         (1ul << 24)
#define PIN_PB08I_CCL_IN8                 40L  /**< \brief CCL signal: IN8 on PB08 mux I */
#define MUX_PB08I_CCL_IN8                  8L
#define PINMUX_PB08I_CCL_IN8       ((PIN_PB08I_CCL_IN8 << 16) | MUX_PB08I_CCL_IN8)
#define PORT_PB08I_CCL_IN8         (1ul <<  8)
#define PIN_PB14I_CCL_IN9                 46L  /**< \brief CCL signal: IN9 on PB14 mux I */
#define MUX_PB14I_CCL_IN9                  8L
#define PINMUX_PB14I_CCL_IN9       ((PIN_PB14I_CCL_IN9 << 16) | MUX_PB14I_CCL_IN9)
#define PORT_PB14I_CCL_IN9         (1ul << 14)
#define PIN_PB15I_CCL_IN10                47L  /**< \brief CCL signal: IN10 on PB15 mux I */
#define MUX_PB15I_CCL_IN10                 8L
#define PINMUX_PB15I_CCL_IN10      ((PIN_PB15I_CCL_IN10 << 16) | MUX_PB15I_CCL_IN10)
#define PORT_PB15I_CCL_IN10        (1ul << 15)
#define PIN_PB16I_CCL_IN11                48L  /**< \brief CCL signal: IN11 on PB16 mux I */
#define MUX_PB16I_CCL_IN11                 8L
#define PINMUX_PB16I_CCL_IN11      ((PIN_PB16I_CCL_IN11 << 16) | MUX_PB16I_CCL_IN11)
#define PORT_PB16I_CCL_IN11        (1ul << 16)
#define PIN_PA07I_CCL_OUT0                 7L  /**< \brief CCL signal: OUT0 on PA07 mux I */
#define MUX_PA07I_CCL_OUT0                 8L
#define PINMUX_PA07I_CCL_OUT0      ((PIN_PA07I_CCL_OUT0 << 16) | MUX_PA07I_CCL_OUT0)
#define PORT_PA07I_CCL_OUT0        (1ul <<  7)
#define PIN_PA19I_CCL_OUT0                19L  /**< \brief CCL signal: OUT0 on PA19 mux I */
#define MUX_PA19I_CCL_OUT0                 8L
#define PINMUX_PA19I_CCL_OUT0      ((PIN_PA19I_CCL_OUT0 << 16) | MUX_PA19I_CCL_OUT0)
#define PORT_PA19I_CCL_OUT0        (1ul << 19)
#define PIN_PB02I_CCL_OUT0                34L  /**< \brief CCL signal: OUT0 on PB02 mux I */
#define MUX_PB02I_CCL_OUT0                 8L
#define PINMUX_PB02I_CCL_OUT0      ((PIN_PB02I_CCL_OUT0 << 16) | MUX_PB02I_CCL_OUT0)
#define PORT_PB02I_CCL_OUT0        (1ul <<  2)
#define PIN_PB23I_CCL_OUT0                55L  /**< \brief CCL signal: OUT0 on PB23 mux I */
#define MUX_PB23I_CCL_OUT0                 8L
#define PINMUX_PB23I_CCL_OUT0      ((PIN_PB23I_CCL_OUT0 << 16) | MUX_PB23I_CCL_OUT0)
#define PORT_PB23I_CCL_OUT0        (1ul << 23)
#define PIN_PA11I_CCL_OUT1                11L  /**< \brief CCL signal: OUT1 on PA11 mux I */
#define MUX_PA11I_CCL_OUT1                 8L
#define PINMUX_PA11I_CCL_OUT1      ((PIN_PA11I_CCL_OUT1 << 16) | MUX_PA11I_CCL_OUT1)
#define PORT_PA11I_CCL_OUT1        (1ul << 11)
#define PIN_PA31I_CCL_OUT1                31L  /**< \brief CCL signal: OUT1 on PA31 mux I */
#define MUX_PA31I_CCL_OUT1                 8L
#define PINMUX_PA31I_CCL_OUT1      ((PIN_PA31I_CCL_OUT1 << 16) | MUX_PA31I_CCL_OUT1)
#define PORT_PA31I_CCL_OUT1        (1ul << 31)
#define PIN_PA25I_CCL_OUT2                25L  /**< \brief CCL signal: OUT2 on PA25 mux I */
#define MUX_PA25I_CCL_OUT2                 8L
#define PINMUX_PA25I_CCL_OUT2      ((PIN_PA25I_CCL_OUT2 << 16) | MUX_PA25I_CCL_OUT2)
#define PORT_PA25I_CCL_OUT2        (1ul << 25)
#define PIN_PB09I_CCL_OUT2                41L  /**< \brief CCL signal: OUT2 on PB09 mux I */
#define MUX_PB09I_CCL_OUT2                 8L
#define PINMUX_PB09I_CCL_OUT2      ((PIN_PB09I_CCL_OUT2 << 16) | MUX_PB09I_CCL_OUT2)
#define PORT_PB09I_CCL_OUT2        (1ul <<  9)
#define PIN_PB17I_CCL_OUT3                49L  /**< \brief CCL signal: OUT3 on PB17 mux I */
#define MUX_PB17I_CCL_OUT3                 8L
#define PINMUX_PB17I_CCL_OUT3      ((PIN_PB17I_CCL_OUT3 << 16) | MUX_PB17I_CCL_OUT3)
#define PORT_PB17I_CCL_OUT3        (1ul << 17)
/* ========== PORT definition for RFCTRL peripheral ========== */
#define PIN_PA08F_RFCTRL_FECTRL0           8L  /**< \brief RFCTRL signal: FECTRL0 on PA08 mux F */
#define MUX_PA08F_RFCTRL_FECTRL0           5L
#define PINMUX_PA08F_RFCTRL_FECTRL0  ((PIN_PA08F_RFCTRL_FECTRL0 << 16) | MUX_PA08F_RFCTRL_FECTRL0)
#define PORT_PA08F_RFCTRL_FECTRL0  (1ul <<  8)
#define PIN_PA09F_RFCTRL_FECTRL1           9L  /**< \brief RFCTRL signal: FECTRL1 on PA09 mux F */
#define MUX_PA09F_RFCTRL_FECTRL1           5L
#define PINMUX_PA09F_RFCTRL_FECTRL1  ((PIN_PA09F_RFCTRL_FECTRL1 << 16) | MUX_PA09F_RFCTRL_FECTRL1)
#define PORT_PA09F_RFCTRL_FECTRL1  (1ul <<  9)
#define PIN_PA12F_RFCTRL_FECTRL2          12L  /**< \brief RFCTRL signal: FECTRL2 on PA12 mux F */
#define MUX_PA12F_RFCTRL_FECTRL2           5L
#define PINMUX_PA12F_RFCTRL_FECTRL2  ((PIN_PA12F_RFCTRL_FECTRL2 << 16) | MUX_PA12F_RFCTRL_FECTRL2)
#define PORT_PA12F_RFCTRL_FECTRL2  (1ul << 12)
#define PIN_PA13F_RFCTRL_FECTRL3          13L  /**< \brief RFCTRL signal: FECTRL3 on PA13 mux F */
#define MUX_PA13F_RFCTRL_FECTRL3           5L
#define PINMUX_PA13F_RFCTRL_FECTRL3  ((PIN_PA13F_RFCTRL_FECTRL3 << 16) | MUX_PA13F_RFCTRL_FECTRL3)
#define PORT_PA13F_RFCTRL_FECTRL3  (1ul << 13)
#define PIN_PA14F_RFCTRL_FECTRL4          14L  /**< \brief RFCTRL signal: FECTRL4 on PA14 mux F */
#define MUX_PA14F_RFCTRL_FECTRL4           5L
#define PINMUX_PA14F_RFCTRL_FECTRL4  ((PIN_PA14F_RFCTRL_FECTRL4 << 16) | MUX_PA14F_RFCTRL_FECTRL4)
#define PORT_PA14F_RFCTRL_FECTRL4  (1ul << 14)
#define PIN_PA15F_RFCTRL_FECTRL5          15L  /**< \brief RFCTRL signal: FECTRL5 on PA15 mux F */
#define MUX_PA15F_RFCTRL_FECTRL5           5L
#define PINMUX_PA15F_RFCTRL_FECTRL5  ((PIN_PA15F_RFCTRL_FECTRL5 << 16) | MUX_PA15F_RFCTRL_FECTRL5)
#define PORT_PA15F_RFCTRL_FECTRL5  (1ul << 15)

/*
// 0x41004000 NVMCTRL

#define NVMCTRL_CTRLA	MMIO_REG(0x41004000, uint16_t)
#define NVMCTRL_CTRLA_CMD(x)	((x) << 0)
#define NVMCTRL_CTRLA_CMD_MSK	0x0000007f
#define NVMCTRL_CTRLA_CMD_VAL(x)	(((x) >> 0) & 0x7f)
#define NVMCTRL_CTRLA_CMDEX(x)	((x) << 8)
#define NVMCTRL_CTRLA_CMDEX_MSK	0x0000ff00
#define NVMCTRL_CTRLA_CMDEX_VAL(x)	(((x) >> 8) & 0xff)

#define NVMCTRL_CTRLA_s    MMIO_REG(0x41004000, struct __struct_NVMCTRL_CTRLA)
struct __struct_NVMCTRL_CTRLA
{
  uint16_t cmd : 7;
  uint16_t  : 1;
  uint16_t cmdex : 8;
};

#define NVMCTRL_CTRLB	MMIO_REG(0x41004004, uint32_t)
#define NVMCTRL_CTRLB_RWS(x)	((x) << 1)
#define NVMCTRL_CTRLB_RWS_MSK	0x0000001e
#define NVMCTRL_CTRLB_RWS_VAL(x)	(((x) >> 1) & 0xf)
#define NVMCTRL_CTRLB_MANW	(1 << 7)
#define NVMCTRL_CTRLB_SLEEPPRM(x)	((x) << 8)
#define NVMCTRL_CTRLB_SLEEPPRM_MSK	0x00000300
#define NVMCTRL_CTRLB_SLEEPPRM_VAL(x)	(((x) >> 8) & 0x3)
#define NVMCTRL_CTRLB_READMODE(x)	((x) << 16)
#define NVMCTRL_CTRLB_READMODE_MSK	0x00030000
#define NVMCTRL_CTRLB_READMODE_VAL(x)	(((x) >> 16) & 0x3)
#define NVMCTRL_CTRLB_CACHEDIS	(1 << 18)

#define NVMCTRL_CTRLB_s    MMIO_REG(0x41004004, struct __struct_NVMCTRL_CTRLB)
struct __struct_NVMCTRL_CTRLB
{
  uint32_t  : 1;
  uint32_t rws : 4;
  uint32_t  : 2;
  uint32_t manw : 1;
  uint32_t sleepprm : 2;
  uint32_t  : 6;
  uint32_t readmode : 2;
  uint32_t cachedis : 1;
  uint32_t  : 13;
};

#define NVMCTRL_PARAM	MMIO_REG(0x41004008, uint32_t)
#define NVMCTRL_PARAM_NVMP(x)	((x) << 0)
#define NVMCTRL_PARAM_NVMP_MSK	0x0000ffff
#define NVMCTRL_PARAM_NVMP_VAL(x)	(((x) >> 0) & 0xffff)
#define NVMCTRL_PARAM_PSZ(x)	((x) << 16)
#define NVMCTRL_PARAM_PSZ_MSK	0x00070000
#define NVMCTRL_PARAM_PSZ_VAL(x)	(((x) >> 16) & 0x7)

#define NVMCTRL_PARAM_s    MMIO_REG(0x41004008, struct __struct_NVMCTRL_PARAM)
struct __struct_NVMCTRL_PARAM
{
  uint32_t nvmp : 16;
  uint32_t psz : 3;
  uint32_t  : 13;
};

#define NVMCTRL_INTENCLR	MMIO_REG(0x4100400c, uint8_t)
#define NVMCTRL_INTENCLR_READY	(1 << 0)
#define NVMCTRL_INTENCLR_ERROR	(1 << 1)

#define NVMCTRL_INTENCLR_s    MMIO_REG(0x4100400c, struct __struct_NVMCTRL_INTENCLR)
struct __struct_NVMCTRL_INTENCLR
{
  uint8_t ready : 1;
  uint8_t error : 1;
  uint8_t  : 6;
};

#define NVMCTRL_INTENSET	MMIO_REG(0x41004010, uint8_t)
#define NVMCTRL_INTENSET_READY	(1 << 0)
#define NVMCTRL_INTENSET_ERROR	(1 << 1)

#define NVMCTRL_INTENSET_s    MMIO_REG(0x41004010, struct __struct_NVMCTRL_INTENSET)
struct __struct_NVMCTRL_INTENSET
{
  uint8_t ready : 1;
  uint8_t error : 1;
  uint8_t  : 6;
};

#define NVMCTRL_INTFLAG	MMIO_REG(0x41004014, uint8_t)
#define NVMCTRL_INTFLAG_READY	(1 << 0)
#define NVMCTRL_INTFLAG_ERROR	(1 << 1)

#define NVMCTRL_INTFLAG_s    MMIO_REG(0x41004014, struct __struct_NVMCTRL_INTFLAG)
struct __struct_NVMCTRL_INTFLAG
{
  uint8_t ready : 1;
  uint8_t error : 1;
  uint8_t  : 6;
};

#define NVMCTRL_STATUS	MMIO_REG(0x41004018, uint16_t)
#define NVMCTRL_STATUS_PRM	(1 << 0)
#define NVMCTRL_STATUS_LOAD	(1 << 1)
#define NVMCTRL_STATUS_PROGE	(1 << 2)
#define NVMCTRL_STATUS_LOCKE	(1 << 3)
#define NVMCTRL_STATUS_NVME	(1 << 4)
#define NVMCTRL_STATUS_SB	(1 << 8)

#define NVMCTRL_STATUS_s    MMIO_REG(0x41004018, struct __struct_NVMCTRL_STATUS)
struct __struct_NVMCTRL_STATUS
{
  uint16_t prm : 1;
  uint16_t load : 1;
  uint16_t proge : 1;
  uint16_t locke : 1;
  uint16_t nvme : 1;
  uint16_t  : 3;
  uint16_t sb : 1;
  uint16_t  : 7;
};

#define NVMCTRL_ADDR	MMIO_REG(0x4100401c, uint32_t)
#define NVMCTRL_ADDR_ADDR(x)	((x) << 0)
#define NVMCTRL_ADDR_ADDR_MSK	0x003fffff
#define NVMCTRL_ADDR_ADDR_VAL(x)	(((x) >> 0) & 0x3fffff)

#define NVMCTRL_ADDR_s    MMIO_REG(0x4100401c, struct __struct_NVMCTRL_ADDR)
struct __struct_NVMCTRL_ADDR
{
  uint32_t addr : 22;
  uint32_t  : 10;
};

#define NVMCTRL_LOCK	MMIO_REG(0x41004020, uint16_t)
#define NVMCTRL_LOCK_LOCK(x)	((x) << 0)
#define NVMCTRL_LOCK_LOCK_MSK	0x0000ffff
#define NVMCTRL_LOCK_LOCK_VAL(x)	(((x) >> 0) & 0xffff)

#define NVMCTRL_LOCK_s    MMIO_REG(0x41004020, struct __struct_NVMCTRL_LOCK)
struct __struct_NVMCTRL_LOCK
{
  uint16_t lock : 16;
};
*/
// 0x40002800 PORTA

#define PORTA_DIR	MMIO_REG(0x40002800, int32_t)
#define PORTA_DIR_P0	(1 << 0)
#define PORTA_DIR_P1	(1 << 1)
#define PORTA_DIR_P2	(1 << 2)
#define PORTA_DIR_P3	(1 << 3)
#define PORTA_DIR_P4	(1 << 4)
#define PORTA_DIR_P5	(1 << 5)
#define PORTA_DIR_P6	(1 << 6)
#define PORTA_DIR_P7	(1 << 7)
#define PORTA_DIR_P8	(1 << 8)
#define PORTA_DIR_P9	(1 << 9)
#define PORTA_DIR_P10	(1 << 10)
#define PORTA_DIR_P11	(1 << 11)
#define PORTA_DIR_P12	(1 << 12)
#define PORTA_DIR_P13	(1 << 13)
#define PORTA_DIR_P14	(1 << 14)
#define PORTA_DIR_P15	(1 << 15)
#define PORTA_DIR_P16	(1 << 16)
#define PORTA_DIR_P17	(1 << 17)
#define PORTA_DIR_P18	(1 << 18)
#define PORTA_DIR_P19	(1 << 19)
#define PORTA_DIR_P20	(1 << 20)
#define PORTA_DIR_P21	(1 << 21)
#define PORTA_DIR_P22	(1 << 22)
#define PORTA_DIR_P23	(1 << 23)
#define PORTA_DIR_P24	(1 << 24)
#define PORTA_DIR_P25	(1 << 25)
#define PORTA_DIR_P26	(1 << 26)
#define PORTA_DIR_P27	(1 << 27)
#define PORTA_DIR_P28	(1 << 28)
#define PORTA_DIR_P29	(1 << 29)
#define PORTA_DIR_P30	(1 << 30)
#define PORTA_DIR_P31	(1 << 31)

#define PORTA_DIR_s    MMIO_REG(0x40002800, struct __struct_PORTA_DIR)
struct __struct_PORTA_DIR
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTA_DIRCLR	MMIO_REG(0x40002804, int32_t)
#define PORTA_DIRCLR_P0	(1 << 0)
#define PORTA_DIRCLR_P1	(1 << 1)
#define PORTA_DIRCLR_P2	(1 << 2)
#define PORTA_DIRCLR_P3	(1 << 3)
#define PORTA_DIRCLR_P4	(1 << 4)
#define PORTA_DIRCLR_P5	(1 << 5)
#define PORTA_DIRCLR_P6	(1 << 6)
#define PORTA_DIRCLR_P7	(1 << 7)
#define PORTA_DIRCLR_P8	(1 << 8)
#define PORTA_DIRCLR_P9	(1 << 9)
#define PORTA_DIRCLR_P10	(1 << 10)
#define PORTA_DIRCLR_P11	(1 << 11)
#define PORTA_DIRCLR_P12	(1 << 12)
#define PORTA_DIRCLR_P13	(1 << 13)
#define PORTA_DIRCLR_P14	(1 << 14)
#define PORTA_DIRCLR_P15	(1 << 15)
#define PORTA_DIRCLR_P16	(1 << 16)
#define PORTA_DIRCLR_P17	(1 << 17)
#define PORTA_DIRCLR_P18	(1 << 18)
#define PORTA_DIRCLR_P19	(1 << 19)
#define PORTA_DIRCLR_P20	(1 << 20)
#define PORTA_DIRCLR_P21	(1 << 21)
#define PORTA_DIRCLR_P22	(1 << 22)
#define PORTA_DIRCLR_P23	(1 << 23)
#define PORTA_DIRCLR_P24	(1 << 24)
#define PORTA_DIRCLR_P25	(1 << 25)
#define PORTA_DIRCLR_P26	(1 << 26)
#define PORTA_DIRCLR_P27	(1 << 27)
#define PORTA_DIRCLR_P28	(1 << 28)
#define PORTA_DIRCLR_P29	(1 << 29)
#define PORTA_DIRCLR_P30	(1 << 30)
#define PORTA_DIRCLR_P31	(1 << 31)

#define PORTA_DIRCLR_s    MMIO_REG(0x40002804, struct __struct_PORTA_DIRCLR)
struct __struct_PORTA_DIRCLR
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTA_DIRSET	MMIO_REG(0x40002808, int32_t)
#define PORTA_DIRSET_P0	(1 << 0)
#define PORTA_DIRSET_P1	(1 << 1)
#define PORTA_DIRSET_P2	(1 << 2)
#define PORTA_DIRSET_P3	(1 << 3)
#define PORTA_DIRSET_P4	(1 << 4)
#define PORTA_DIRSET_P5	(1 << 5)
#define PORTA_DIRSET_P6	(1 << 6)
#define PORTA_DIRSET_P7	(1 << 7)
#define PORTA_DIRSET_P8	(1 << 8)
#define PORTA_DIRSET_P9	(1 << 9)
#define PORTA_DIRSET_P10	(1 << 10)
#define PORTA_DIRSET_P11	(1 << 11)
#define PORTA_DIRSET_P12	(1 << 12)
#define PORTA_DIRSET_P13	(1 << 13)
#define PORTA_DIRSET_P14	(1 << 14)
#define PORTA_DIRSET_P15	(1 << 15)
#define PORTA_DIRSET_P16	(1 << 16)
#define PORTA_DIRSET_P17	(1 << 17)
#define PORTA_DIRSET_P18	(1 << 18)
#define PORTA_DIRSET_P19	(1 << 19)
#define PORTA_DIRSET_P20	(1 << 20)
#define PORTA_DIRSET_P21	(1 << 21)
#define PORTA_DIRSET_P22	(1 << 22)
#define PORTA_DIRSET_P23	(1 << 23)
#define PORTA_DIRSET_P24	(1 << 24)
#define PORTA_DIRSET_P25	(1 << 25)
#define PORTA_DIRSET_P26	(1 << 26)
#define PORTA_DIRSET_P27	(1 << 27)
#define PORTA_DIRSET_P28	(1 << 28)
#define PORTA_DIRSET_P29	(1 << 29)
#define PORTA_DIRSET_P30	(1 << 30)
#define PORTA_DIRSET_P31	(1 << 31)

#define PORTA_DIRSET_s    MMIO_REG(0x40002808, struct __struct_PORTA_DIRSET)
struct __struct_PORTA_DIRSET
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTA_DIRTGL	MMIO_REG(0x4000280c, int32_t)
#define PORTA_DIRTGL_P0	(1 << 0)
#define PORTA_DIRTGL_P1	(1 << 1)
#define PORTA_DIRTGL_P2	(1 << 2)
#define PORTA_DIRTGL_P3	(1 << 3)
#define PORTA_DIRTGL_P4	(1 << 4)
#define PORTA_DIRTGL_P5	(1 << 5)
#define PORTA_DIRTGL_P6	(1 << 6)
#define PORTA_DIRTGL_P7	(1 << 7)
#define PORTA_DIRTGL_P8	(1 << 8)
#define PORTA_DIRTGL_P9	(1 << 9)
#define PORTA_DIRTGL_P10	(1 << 10)
#define PORTA_DIRTGL_P11	(1 << 11)
#define PORTA_DIRTGL_P12	(1 << 12)
#define PORTA_DIRTGL_P13	(1 << 13)
#define PORTA_DIRTGL_P14	(1 << 14)
#define PORTA_DIRTGL_P15	(1 << 15)
#define PORTA_DIRTGL_P16	(1 << 16)
#define PORTA_DIRTGL_P17	(1 << 17)
#define PORTA_DIRTGL_P18	(1 << 18)
#define PORTA_DIRTGL_P19	(1 << 19)
#define PORTA_DIRTGL_P20	(1 << 20)
#define PORTA_DIRTGL_P21	(1 << 21)
#define PORTA_DIRTGL_P22	(1 << 22)
#define PORTA_DIRTGL_P23	(1 << 23)
#define PORTA_DIRTGL_P24	(1 << 24)
#define PORTA_DIRTGL_P25	(1 << 25)
#define PORTA_DIRTGL_P26	(1 << 26)
#define PORTA_DIRTGL_P27	(1 << 27)
#define PORTA_DIRTGL_P28	(1 << 28)
#define PORTA_DIRTGL_P29	(1 << 29)
#define PORTA_DIRTGL_P30	(1 << 30)
#define PORTA_DIRTGL_P31	(1 << 31)

#define PORTA_DIRTGL_s    MMIO_REG(0x4000280c, struct __struct_PORTA_DIRTGL)
struct __struct_PORTA_DIRTGL
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTA_OUT	MMIO_REG(0x40002810, int32_t)
#define PORTA_OUT_P0	(1 << 0)
#define PORTA_OUT_P1	(1 << 1)
#define PORTA_OUT_P2	(1 << 2)
#define PORTA_OUT_P3	(1 << 3)
#define PORTA_OUT_P4	(1 << 4)
#define PORTA_OUT_P5	(1 << 5)
#define PORTA_OUT_P6	(1 << 6)
#define PORTA_OUT_P7	(1 << 7)
#define PORTA_OUT_P8	(1 << 8)
#define PORTA_OUT_P9	(1 << 9)
#define PORTA_OUT_P10	(1 << 10)
#define PORTA_OUT_P11	(1 << 11)
#define PORTA_OUT_P12	(1 << 12)
#define PORTA_OUT_P13	(1 << 13)
#define PORTA_OUT_P14	(1 << 14)
#define PORTA_OUT_P15	(1 << 15)
#define PORTA_OUT_P16	(1 << 16)
#define PORTA_OUT_P17	(1 << 17)
#define PORTA_OUT_P18	(1 << 18)
#define PORTA_OUT_P19	(1 << 19)
#define PORTA_OUT_P20	(1 << 20)
#define PORTA_OUT_P21	(1 << 21)
#define PORTA_OUT_P22	(1 << 22)
#define PORTA_OUT_P23	(1 << 23)
#define PORTA_OUT_P24	(1 << 24)
#define PORTA_OUT_P25	(1 << 25)
#define PORTA_OUT_P26	(1 << 26)
#define PORTA_OUT_P27	(1 << 27)
#define PORTA_OUT_P28	(1 << 28)
#define PORTA_OUT_P29	(1 << 29)
#define PORTA_OUT_P30	(1 << 30)
#define PORTA_OUT_P31	(1 << 31)

#define PORTA_OUT_s    MMIO_REG(0x40002810, struct __struct_PORTA_OUT)
struct __struct_PORTA_OUT
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTA_OUTCLR	MMIO_REG(0x40002814, int32_t)
#define PORTA_OUTCLR_P0	(1 << 0)
#define PORTA_OUTCLR_P1	(1 << 1)
#define PORTA_OUTCLR_P2	(1 << 2)
#define PORTA_OUTCLR_P3	(1 << 3)
#define PORTA_OUTCLR_P4	(1 << 4)
#define PORTA_OUTCLR_P5	(1 << 5)
#define PORTA_OUTCLR_P6	(1 << 6)
#define PORTA_OUTCLR_P7	(1 << 7)
#define PORTA_OUTCLR_P8	(1 << 8)
#define PORTA_OUTCLR_P9	(1 << 9)
#define PORTA_OUTCLR_P10	(1 << 10)
#define PORTA_OUTCLR_P11	(1 << 11)
#define PORTA_OUTCLR_P12	(1 << 12)
#define PORTA_OUTCLR_P13	(1 << 13)
#define PORTA_OUTCLR_P14	(1 << 14)
#define PORTA_OUTCLR_P15	(1 << 15)
#define PORTA_OUTCLR_P16	(1 << 16)
#define PORTA_OUTCLR_P17	(1 << 17)
#define PORTA_OUTCLR_P18	(1 << 18)
#define PORTA_OUTCLR_P19	(1 << 19)
#define PORTA_OUTCLR_P20	(1 << 20)
#define PORTA_OUTCLR_P21	(1 << 21)
#define PORTA_OUTCLR_P22	(1 << 22)
#define PORTA_OUTCLR_P23	(1 << 23)
#define PORTA_OUTCLR_P24	(1 << 24)
#define PORTA_OUTCLR_P25	(1 << 25)
#define PORTA_OUTCLR_P26	(1 << 26)
#define PORTA_OUTCLR_P27	(1 << 27)
#define PORTA_OUTCLR_P28	(1 << 28)
#define PORTA_OUTCLR_P29	(1 << 29)
#define PORTA_OUTCLR_P30	(1 << 30)
#define PORTA_OUTCLR_P31	(1 << 31)

#define PORTA_OUTCLR_s    MMIO_REG(0x40002814, struct __struct_PORTA_OUTCLR)
struct __struct_PORTA_OUTCLR
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTA_OUTSET	MMIO_REG(0x40002818, int32_t)
#define PORTA_OUTSET_P0	(1 << 0)
#define PORTA_OUTSET_P1	(1 << 1)
#define PORTA_OUTSET_P2	(1 << 2)
#define PORTA_OUTSET_P3	(1 << 3)
#define PORTA_OUTSET_P4	(1 << 4)
#define PORTA_OUTSET_P5	(1 << 5)
#define PORTA_OUTSET_P6	(1 << 6)
#define PORTA_OUTSET_P7	(1 << 7)
#define PORTA_OUTSET_P8	(1 << 8)
#define PORTA_OUTSET_P9	(1 << 9)
#define PORTA_OUTSET_P10	(1 << 10)
#define PORTA_OUTSET_P11	(1 << 11)
#define PORTA_OUTSET_P12	(1 << 12)
#define PORTA_OUTSET_P13	(1 << 13)
#define PORTA_OUTSET_P14	(1 << 14)
#define PORTA_OUTSET_P15	(1 << 15)
#define PORTA_OUTSET_P16	(1 << 16)
#define PORTA_OUTSET_P17	(1 << 17)
#define PORTA_OUTSET_P18	(1 << 18)
#define PORTA_OUTSET_P19	(1 << 19)
#define PORTA_OUTSET_P20	(1 << 20)
#define PORTA_OUTSET_P21	(1 << 21)
#define PORTA_OUTSET_P22	(1 << 22)
#define PORTA_OUTSET_P23	(1 << 23)
#define PORTA_OUTSET_P24	(1 << 24)
#define PORTA_OUTSET_P25	(1 << 25)
#define PORTA_OUTSET_P26	(1 << 26)
#define PORTA_OUTSET_P27	(1 << 27)
#define PORTA_OUTSET_P28	(1 << 28)
#define PORTA_OUTSET_P29	(1 << 29)
#define PORTA_OUTSET_P30	(1 << 30)
#define PORTA_OUTSET_P31	(1 << 31)

#define PORTA_OUTSET_s    MMIO_REG(0x40002818, struct __struct_PORTA_OUTSET)
struct __struct_PORTA_OUTSET
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTA_OUTTGL	MMIO_REG(0x4000281c, int32_t)
#define PORTA_OUTTGL_P0	(1 << 0)
#define PORTA_OUTTGL_P1	(1 << 1)
#define PORTA_OUTTGL_P2	(1 << 2)
#define PORTA_OUTTGL_P3	(1 << 3)
#define PORTA_OUTTGL_P4	(1 << 4)
#define PORTA_OUTTGL_P5	(1 << 5)
#define PORTA_OUTTGL_P6	(1 << 6)
#define PORTA_OUTTGL_P7	(1 << 7)
#define PORTA_OUTTGL_P8	(1 << 8)
#define PORTA_OUTTGL_P9	(1 << 9)
#define PORTA_OUTTGL_P10	(1 << 10)
#define PORTA_OUTTGL_P11	(1 << 11)
#define PORTA_OUTTGL_P12	(1 << 12)
#define PORTA_OUTTGL_P13	(1 << 13)
#define PORTA_OUTTGL_P14	(1 << 14)
#define PORTA_OUTTGL_P15	(1 << 15)
#define PORTA_OUTTGL_P16	(1 << 16)
#define PORTA_OUTTGL_P17	(1 << 17)
#define PORTA_OUTTGL_P18	(1 << 18)
#define PORTA_OUTTGL_P19	(1 << 19)
#define PORTA_OUTTGL_P20	(1 << 20)
#define PORTA_OUTTGL_P21	(1 << 21)
#define PORTA_OUTTGL_P22	(1 << 22)
#define PORTA_OUTTGL_P23	(1 << 23)
#define PORTA_OUTTGL_P24	(1 << 24)
#define PORTA_OUTTGL_P25	(1 << 25)
#define PORTA_OUTTGL_P26	(1 << 26)
#define PORTA_OUTTGL_P27	(1 << 27)
#define PORTA_OUTTGL_P28	(1 << 28)
#define PORTA_OUTTGL_P29	(1 << 29)
#define PORTA_OUTTGL_P30	(1 << 30)
#define PORTA_OUTTGL_P31	(1 << 31)

#define PORTA_OUTTGL_s    MMIO_REG(0x4000281c, struct __struct_PORTA_OUTTGL)
struct __struct_PORTA_OUTTGL
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTA_IN	MMIO_REG(0x40002820, int32_t)
#define PORTA_IN_P0	(1 << 0)
#define PORTA_IN_P1	(1 << 1)
#define PORTA_IN_P2	(1 << 2)
#define PORTA_IN_P3	(1 << 3)
#define PORTA_IN_P4	(1 << 4)
#define PORTA_IN_P5	(1 << 5)
#define PORTA_IN_P6	(1 << 6)
#define PORTA_IN_P7	(1 << 7)
#define PORTA_IN_P8	(1 << 8)
#define PORTA_IN_P9	(1 << 9)
#define PORTA_IN_P10	(1 << 10)
#define PORTA_IN_P11	(1 << 11)
#define PORTA_IN_P12	(1 << 12)
#define PORTA_IN_P13	(1 << 13)
#define PORTA_IN_P14	(1 << 14)
#define PORTA_IN_P15	(1 << 15)
#define PORTA_IN_P16	(1 << 16)
#define PORTA_IN_P17	(1 << 17)
#define PORTA_IN_P18	(1 << 18)
#define PORTA_IN_P19	(1 << 19)
#define PORTA_IN_P20	(1 << 20)
#define PORTA_IN_P21	(1 << 21)
#define PORTA_IN_P22	(1 << 22)
#define PORTA_IN_P23	(1 << 23)
#define PORTA_IN_P24	(1 << 24)
#define PORTA_IN_P25	(1 << 25)
#define PORTA_IN_P26	(1 << 26)
#define PORTA_IN_P27	(1 << 27)
#define PORTA_IN_P28	(1 << 28)
#define PORTA_IN_P29	(1 << 29)
#define PORTA_IN_P30	(1 << 30)
#define PORTA_IN_P31	(1 << 31)

#define PORTA_IN_s    MMIO_REG(0x40002820, struct __struct_PORTA_IN)
struct __struct_PORTA_IN
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTA_CTRL	MMIO_REG(0x40002824, int32_t)

#define PORTA_WRCONFIG	MMIO_REG(0x40002828, int32_t)
#define PORTA_WRCONFIG_PINMASK(x)	((x) << 0)
#define PORTA_WRCONFIG_PINMASK_MSK	0x0000ffff
#define PORTA_WRCONFIG_PINMASK_VAL(x)	(((x) >> 0) & 0xffff)
#define PORTA_WRCONFIG_PMUXEN	(1 << 16)
#define PORTA_WRCONFIG_INEN	(1 << 17)
#define PORTA_WRCONFIG_PULLEN	(1 << 18)
#define PORTA_WRCONFIG_PMUX(x)	((x) << 24)
#define PORTA_WRCONFIG_PMUX_MSK	0x0f000000
#define PORTA_WRCONFIG_PMUX_VAL(x)	(((x) >> 24) & 0xf)
#define PORTA_WRCONFIG_WRPMUX	(1 << 28)
#define PORTA_WRCONFIG_WRPINCFG	(1 << 30)
#define PORTA_WRCONFIG_HWSEL	(1 << 31)

#define PORTA_WRCONFIG_s    MMIO_REG(0x40002828, struct __struct_PORTA_WRCONFIG)
struct __struct_PORTA_WRCONFIG
{
  uint32_t pinmask : 16;
  uint32_t pmuxen : 1;
  uint32_t inen : 1;
  uint32_t pullen : 1;
  uint32_t  : 5;
  uint32_t pmux : 4;
  uint32_t wrpmux : 1;
  uint32_t  : 1;
  uint32_t wrpincfg : 1;
  uint32_t hwsel : 1;
};

#define PORTA_PMUX0	MMIO_REG(0x40002830, uint8_t)
#define PORTA_PMUX0_PMUXE(x)	((x) << 0)
#define PORTA_PMUX0_PMUXE_MSK	0x0000000f
#define PORTA_PMUX0_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX0_PMUXO(x)	((x) << 4)
#define PORTA_PMUX0_PMUXO_MSK	0x000000f0
#define PORTA_PMUX0_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX0_s    MMIO_REG(0x40002830, struct __struct_PORTA_PMUX0)
struct __struct_PORTA_PMUX0
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX1	MMIO_REG(0x40002831, uint8_t)
#define PORTA_PMUX1_PMUXE(x)	((x) << 0)
#define PORTA_PMUX1_PMUXE_MSK	0x0000000f
#define PORTA_PMUX1_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX1_PMUXO(x)	((x) << 4)
#define PORTA_PMUX1_PMUXO_MSK	0x000000f0
#define PORTA_PMUX1_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX1_s    MMIO_REG(0x40002831, struct __struct_PORTA_PMUX1)
struct __struct_PORTA_PMUX1
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX2	MMIO_REG(0x40002832, uint8_t)
#define PORTA_PMUX2_PMUXE(x)	((x) << 0)
#define PORTA_PMUX2_PMUXE_MSK	0x0000000f
#define PORTA_PMUX2_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX2_PMUXO(x)	((x) << 4)
#define PORTA_PMUX2_PMUXO_MSK	0x000000f0
#define PORTA_PMUX2_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX2_s    MMIO_REG(0x40002832, struct __struct_PORTA_PMUX2)
struct __struct_PORTA_PMUX2
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX3	MMIO_REG(0x40002833, uint8_t)
#define PORTA_PMUX3_PMUXE(x)	((x) << 0)
#define PORTA_PMUX3_PMUXE_MSK	0x0000000f
#define PORTA_PMUX3_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX3_PMUXO(x)	((x) << 4)
#define PORTA_PMUX3_PMUXO_MSK	0x000000f0
#define PORTA_PMUX3_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX3_s    MMIO_REG(0x40002833, struct __struct_PORTA_PMUX3)
struct __struct_PORTA_PMUX3
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX4	MMIO_REG(0x40002834, uint8_t)
#define PORTA_PMUX4_PMUXE(x)	((x) << 0)
#define PORTA_PMUX4_PMUXE_MSK	0x0000000f
#define PORTA_PMUX4_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX4_PMUXO(x)	((x) << 4)
#define PORTA_PMUX4_PMUXO_MSK	0x000000f0
#define PORTA_PMUX4_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX4_s    MMIO_REG(0x40002834, struct __struct_PORTA_PMUX4)
struct __struct_PORTA_PMUX4
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX5	MMIO_REG(0x40002835, uint8_t)
#define PORTA_PMUX5_PMUXE(x)	((x) << 0)
#define PORTA_PMUX5_PMUXE_MSK	0x0000000f
#define PORTA_PMUX5_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX5_PMUXO(x)	((x) << 4)
#define PORTA_PMUX5_PMUXO_MSK	0x000000f0
#define PORTA_PMUX5_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX5_s    MMIO_REG(0x40002835, struct __struct_PORTA_PMUX5)
struct __struct_PORTA_PMUX5
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX6	MMIO_REG(0x40002836, uint8_t)
#define PORTA_PMUX6_PMUXE(x)	((x) << 0)
#define PORTA_PMUX6_PMUXE_MSK	0x0000000f
#define PORTA_PMUX6_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX6_PMUXO(x)	((x) << 4)
#define PORTA_PMUX6_PMUXO_MSK	0x000000f0
#define PORTA_PMUX6_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX6_s    MMIO_REG(0x40002836, struct __struct_PORTA_PMUX6)
struct __struct_PORTA_PMUX6
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX7	MMIO_REG(0x40002837, uint8_t)
#define PORTA_PMUX7_PMUXE(x)	((x) << 0)
#define PORTA_PMUX7_PMUXE_MSK	0x0000000f
#define PORTA_PMUX7_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX7_PMUXO(x)	((x) << 4)
#define PORTA_PMUX7_PMUXO_MSK	0x000000f0
#define PORTA_PMUX7_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX7_s    MMIO_REG(0x40002837, struct __struct_PORTA_PMUX7)
struct __struct_PORTA_PMUX7
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX8	MMIO_REG(0x40002838, uint8_t)
#define PORTA_PMUX8_PMUXE(x)	((x) << 0)
#define PORTA_PMUX8_PMUXE_MSK	0x0000000f
#define PORTA_PMUX8_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX8_PMUXO(x)	((x) << 4)
#define PORTA_PMUX8_PMUXO_MSK	0x000000f0
#define PORTA_PMUX8_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX8_s    MMIO_REG(0x40002838, struct __struct_PORTA_PMUX8)
struct __struct_PORTA_PMUX8
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX9	MMIO_REG(0x40002839, uint8_t)
#define PORTA_PMUX9_PMUXE(x)	((x) << 0)
#define PORTA_PMUX9_PMUXE_MSK	0x0000000f
#define PORTA_PMUX9_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX9_PMUXO(x)	((x) << 4)
#define PORTA_PMUX9_PMUXO_MSK	0x000000f0
#define PORTA_PMUX9_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX9_s    MMIO_REG(0x40002839, struct __struct_PORTA_PMUX9)
struct __struct_PORTA_PMUX9
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX10	MMIO_REG(0x4000283a, uint8_t)
#define PORTA_PMUX10_PMUXE(x)	((x) << 0)
#define PORTA_PMUX10_PMUXE_MSK	0x0000000f
#define PORTA_PMUX10_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX10_PMUXO(x)	((x) << 4)
#define PORTA_PMUX10_PMUXO_MSK	0x000000f0
#define PORTA_PMUX10_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX10_s    MMIO_REG(0x4000283a, struct __struct_PORTA_PMUX10)
struct __struct_PORTA_PMUX10
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX11	MMIO_REG(0x4000283b, uint8_t)
#define PORTA_PMUX11_PMUXE(x)	((x) << 0)
#define PORTA_PMUX11_PMUXE_MSK	0x0000000f
#define PORTA_PMUX11_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX11_PMUXO(x)	((x) << 4)
#define PORTA_PMUX11_PMUXO_MSK	0x000000f0
#define PORTA_PMUX11_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX11_s    MMIO_REG(0x4000283b, struct __struct_PORTA_PMUX11)
struct __struct_PORTA_PMUX11
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX12	MMIO_REG(0x4000283c, uint8_t)
#define PORTA_PMUX12_PMUXE(x)	((x) << 0)
#define PORTA_PMUX12_PMUXE_MSK	0x0000000f
#define PORTA_PMUX12_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX12_PMUXO(x)	((x) << 4)
#define PORTA_PMUX12_PMUXO_MSK	0x000000f0
#define PORTA_PMUX12_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX12_s    MMIO_REG(0x4000283c, struct __struct_PORTA_PMUX12)
struct __struct_PORTA_PMUX12
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX13	MMIO_REG(0x4000283d, uint8_t)
#define PORTA_PMUX13_PMUXE(x)	((x) << 0)
#define PORTA_PMUX13_PMUXE_MSK	0x0000000f
#define PORTA_PMUX13_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX13_PMUXO(x)	((x) << 4)
#define PORTA_PMUX13_PMUXO_MSK	0x000000f0
#define PORTA_PMUX13_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX13_s    MMIO_REG(0x4000283d, struct __struct_PORTA_PMUX13)
struct __struct_PORTA_PMUX13
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX14	MMIO_REG(0x4000283e, uint8_t)
#define PORTA_PMUX14_PMUXE(x)	((x) << 0)
#define PORTA_PMUX14_PMUXE_MSK	0x0000000f
#define PORTA_PMUX14_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX14_PMUXO(x)	((x) << 4)
#define PORTA_PMUX14_PMUXO_MSK	0x000000f0
#define PORTA_PMUX14_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX14_s    MMIO_REG(0x4000283e, struct __struct_PORTA_PMUX14)
struct __struct_PORTA_PMUX14
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PMUX15	MMIO_REG(0x4000283f, uint8_t)
#define PORTA_PMUX15_PMUXE(x)	((x) << 0)
#define PORTA_PMUX15_PMUXE_MSK	0x0000000f
#define PORTA_PMUX15_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTA_PMUX15_PMUXO(x)	((x) << 4)
#define PORTA_PMUX15_PMUXO_MSK	0x000000f0
#define PORTA_PMUX15_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTA_PMUX15_s    MMIO_REG(0x4000283f, struct __struct_PORTA_PMUX15)
struct __struct_PORTA_PMUX15
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTA_PINCFG0	MMIO_REG(0x40002840, uint8_t)
#define PORTA_PINCFG0_PMUXEN	(1 << 0)
#define PORTA_PINCFG0_INEN	(1 << 1)
#define PORTA_PINCFG0_PULLEN	(1 << 2)

#define PORTA_PINCFG0_s    MMIO_REG(0x40002840, struct __struct_PORTA_PINCFG0)
struct __struct_PORTA_PINCFG0
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG1	MMIO_REG(0x40002841, uint8_t)
#define PORTA_PINCFG1_PMUXEN	(1 << 0)
#define PORTA_PINCFG1_INEN	(1 << 1)
#define PORTA_PINCFG1_PULLEN	(1 << 2)

#define PORTA_PINCFG1_s    MMIO_REG(0x40002841, struct __struct_PORTA_PINCFG1)
struct __struct_PORTA_PINCFG1
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG2	MMIO_REG(0x40002842, uint8_t)
#define PORTA_PINCFG2_PMUXEN	(1 << 0)
#define PORTA_PINCFG2_INEN	(1 << 1)
#define PORTA_PINCFG2_PULLEN	(1 << 2)

#define PORTA_PINCFG2_s    MMIO_REG(0x40002842, struct __struct_PORTA_PINCFG2)
struct __struct_PORTA_PINCFG2
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG3	MMIO_REG(0x40002843, uint8_t)
#define PORTA_PINCFG3_PMUXEN	(1 << 0)
#define PORTA_PINCFG3_INEN	(1 << 1)
#define PORTA_PINCFG3_PULLEN	(1 << 2)

#define PORTA_PINCFG3_s    MMIO_REG(0x40002843, struct __struct_PORTA_PINCFG3)
struct __struct_PORTA_PINCFG3
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG4	MMIO_REG(0x40002844, uint8_t)
#define PORTA_PINCFG4_PMUXEN	(1 << 0)
#define PORTA_PINCFG4_INEN	(1 << 1)
#define PORTA_PINCFG4_PULLEN	(1 << 2)

#define PORTA_PINCFG4_s    MMIO_REG(0x40002844, struct __struct_PORTA_PINCFG4)
struct __struct_PORTA_PINCFG4
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG5	MMIO_REG(0x40002845, uint8_t)
#define PORTA_PINCFG5_PMUXEN	(1 << 0)
#define PORTA_PINCFG5_INEN	(1 << 1)
#define PORTA_PINCFG5_PULLEN	(1 << 2)

#define PORTA_PINCFG5_s    MMIO_REG(0x40002845, struct __struct_PORTA_PINCFG5)
struct __struct_PORTA_PINCFG5
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG6	MMIO_REG(0x40002846, uint8_t)
#define PORTA_PINCFG6_PMUXEN	(1 << 0)
#define PORTA_PINCFG6_INEN	(1 << 1)
#define PORTA_PINCFG6_PULLEN	(1 << 2)

#define PORTA_PINCFG6_s    MMIO_REG(0x40002846, struct __struct_PORTA_PINCFG6)
struct __struct_PORTA_PINCFG6
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG7	MMIO_REG(0x40002847, uint8_t)
#define PORTA_PINCFG7_PMUXEN	(1 << 0)
#define PORTA_PINCFG7_INEN	(1 << 1)
#define PORTA_PINCFG7_PULLEN	(1 << 2)

#define PORTA_PINCFG7_s    MMIO_REG(0x40002847, struct __struct_PORTA_PINCFG7)
struct __struct_PORTA_PINCFG7
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG8	MMIO_REG(0x40002848, uint8_t)
#define PORTA_PINCFG8_PMUXEN	(1 << 0)
#define PORTA_PINCFG8_INEN	(1 << 1)
#define PORTA_PINCFG8_PULLEN	(1 << 2)

#define PORTA_PINCFG8_s    MMIO_REG(0x40002848, struct __struct_PORTA_PINCFG8)
struct __struct_PORTA_PINCFG8
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG9	MMIO_REG(0x40002849, uint8_t)
#define PORTA_PINCFG9_PMUXEN	(1 << 0)
#define PORTA_PINCFG9_INEN	(1 << 1)
#define PORTA_PINCFG9_PULLEN	(1 << 2)

#define PORTA_PINCFG9_s    MMIO_REG(0x40002849, struct __struct_PORTA_PINCFG9)
struct __struct_PORTA_PINCFG9
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG10	MMIO_REG(0x4000284a, uint8_t)
#define PORTA_PINCFG10_PMUXEN	(1 << 0)
#define PORTA_PINCFG10_INEN	(1 << 1)
#define PORTA_PINCFG10_PULLEN	(1 << 2)

#define PORTA_PINCFG10_s    MMIO_REG(0x4000284a, struct __struct_PORTA_PINCFG10)
struct __struct_PORTA_PINCFG10
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG11	MMIO_REG(0x4000284b, uint8_t)
#define PORTA_PINCFG11_PMUXEN	(1 << 0)
#define PORTA_PINCFG11_INEN	(1 << 1)
#define PORTA_PINCFG11_PULLEN	(1 << 2)

#define PORTA_PINCFG11_s    MMIO_REG(0x4000284b, struct __struct_PORTA_PINCFG11)
struct __struct_PORTA_PINCFG11
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG12	MMIO_REG(0x4000284c, uint8_t)
#define PORTA_PINCFG12_PMUXEN	(1 << 0)
#define PORTA_PINCFG12_INEN	(1 << 1)
#define PORTA_PINCFG12_PULLEN	(1 << 2)

#define PORTA_PINCFG12_s    MMIO_REG(0x4000284c, struct __struct_PORTA_PINCFG12)
struct __struct_PORTA_PINCFG12
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG13	MMIO_REG(0x4000284d, uint8_t)
#define PORTA_PINCFG13_PMUXEN	(1 << 0)
#define PORTA_PINCFG13_INEN	(1 << 1)
#define PORTA_PINCFG13_PULLEN	(1 << 2)

#define PORTA_PINCFG13_s    MMIO_REG(0x4000284d, struct __struct_PORTA_PINCFG13)
struct __struct_PORTA_PINCFG13
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG14	MMIO_REG(0x4000284e, uint8_t)
#define PORTA_PINCFG14_PMUXEN	(1 << 0)
#define PORTA_PINCFG14_INEN	(1 << 1)
#define PORTA_PINCFG14_PULLEN	(1 << 2)

#define PORTA_PINCFG14_s    MMIO_REG(0x4000284e, struct __struct_PORTA_PINCFG14)
struct __struct_PORTA_PINCFG14
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG15	MMIO_REG(0x4000284f, uint8_t)
#define PORTA_PINCFG15_PMUXEN	(1 << 0)
#define PORTA_PINCFG15_INEN	(1 << 1)
#define PORTA_PINCFG15_PULLEN	(1 << 2)

#define PORTA_PINCFG15_s    MMIO_REG(0x4000284f, struct __struct_PORTA_PINCFG15)
struct __struct_PORTA_PINCFG15
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG16	MMIO_REG(0x40002850, uint8_t)
#define PORTA_PINCFG16_PMUXEN	(1 << 0)
#define PORTA_PINCFG16_INEN	(1 << 1)
#define PORTA_PINCFG16_PULLEN	(1 << 2)

#define PORTA_PINCFG16_s    MMIO_REG(0x40002850, struct __struct_PORTA_PINCFG16)
struct __struct_PORTA_PINCFG16
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG17	MMIO_REG(0x40002851, uint8_t)
#define PORTA_PINCFG17_PMUXEN	(1 << 0)
#define PORTA_PINCFG17_INEN	(1 << 1)
#define PORTA_PINCFG17_PULLEN	(1 << 2)

#define PORTA_PINCFG17_s    MMIO_REG(0x40002851, struct __struct_PORTA_PINCFG17)
struct __struct_PORTA_PINCFG17
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG18	MMIO_REG(0x40002852, uint8_t)
#define PORTA_PINCFG18_PMUXEN	(1 << 0)
#define PORTA_PINCFG18_INEN	(1 << 1)
#define PORTA_PINCFG18_PULLEN	(1 << 2)

#define PORTA_PINCFG18_s    MMIO_REG(0x40002852, struct __struct_PORTA_PINCFG18)
struct __struct_PORTA_PINCFG18
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG19	MMIO_REG(0x40002853, uint8_t)
#define PORTA_PINCFG19_PMUXEN	(1 << 0)
#define PORTA_PINCFG19_INEN	(1 << 1)
#define PORTA_PINCFG19_PULLEN	(1 << 2)

#define PORTA_PINCFG19_s    MMIO_REG(0x40002853, struct __struct_PORTA_PINCFG19)
struct __struct_PORTA_PINCFG19
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG20	MMIO_REG(0x40002854, uint8_t)
#define PORTA_PINCFG20_PMUXEN	(1 << 0)
#define PORTA_PINCFG20_INEN	(1 << 1)
#define PORTA_PINCFG20_PULLEN	(1 << 2)

#define PORTA_PINCFG20_s    MMIO_REG(0x40002854, struct __struct_PORTA_PINCFG20)
struct __struct_PORTA_PINCFG20
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG21	MMIO_REG(0x40002855, uint8_t)
#define PORTA_PINCFG21_PMUXEN	(1 << 0)
#define PORTA_PINCFG21_INEN	(1 << 1)
#define PORTA_PINCFG21_PULLEN	(1 << 2)

#define PORTA_PINCFG21_s    MMIO_REG(0x40002855, struct __struct_PORTA_PINCFG21)
struct __struct_PORTA_PINCFG21
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG22	MMIO_REG(0x40002856, uint8_t)
#define PORTA_PINCFG22_PMUXEN	(1 << 0)
#define PORTA_PINCFG22_INEN	(1 << 1)
#define PORTA_PINCFG22_PULLEN	(1 << 2)

#define PORTA_PINCFG22_s    MMIO_REG(0x40002856, struct __struct_PORTA_PINCFG22)
struct __struct_PORTA_PINCFG22
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG23	MMIO_REG(0x40002857, uint8_t)
#define PORTA_PINCFG23_PMUXEN	(1 << 0)
#define PORTA_PINCFG23_INEN	(1 << 1)
#define PORTA_PINCFG23_PULLEN	(1 << 2)

#define PORTA_PINCFG23_s    MMIO_REG(0x40002857, struct __struct_PORTA_PINCFG23)
struct __struct_PORTA_PINCFG23
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG24	MMIO_REG(0x40002858, uint8_t)
#define PORTA_PINCFG24_PMUXEN	(1 << 0)
#define PORTA_PINCFG24_INEN	(1 << 1)
#define PORTA_PINCFG24_PULLEN	(1 << 2)

#define PORTA_PINCFG24_s    MMIO_REG(0x40002858, struct __struct_PORTA_PINCFG24)
struct __struct_PORTA_PINCFG24
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG25	MMIO_REG(0x40002859, uint8_t)
#define PORTA_PINCFG25_PMUXEN	(1 << 0)
#define PORTA_PINCFG25_INEN	(1 << 1)
#define PORTA_PINCFG25_PULLEN	(1 << 2)

#define PORTA_PINCFG25_s    MMIO_REG(0x40002859, struct __struct_PORTA_PINCFG25)
struct __struct_PORTA_PINCFG25
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG26	MMIO_REG(0x4000285a, uint8_t)
#define PORTA_PINCFG26_PMUXEN	(1 << 0)
#define PORTA_PINCFG26_INEN	(1 << 1)
#define PORTA_PINCFG26_PULLEN	(1 << 2)

#define PORTA_PINCFG26_s    MMIO_REG(0x4000285a, struct __struct_PORTA_PINCFG26)
struct __struct_PORTA_PINCFG26
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG27	MMIO_REG(0x4000285b, uint8_t)
#define PORTA_PINCFG27_PMUXEN	(1 << 0)
#define PORTA_PINCFG27_INEN	(1 << 1)
#define PORTA_PINCFG27_PULLEN	(1 << 2)

#define PORTA_PINCFG27_s    MMIO_REG(0x4000285b, struct __struct_PORTA_PINCFG27)
struct __struct_PORTA_PINCFG27
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG28	MMIO_REG(0x4000285c, uint8_t)
#define PORTA_PINCFG28_PMUXEN	(1 << 0)
#define PORTA_PINCFG28_INEN	(1 << 1)
#define PORTA_PINCFG28_PULLEN	(1 << 2)

#define PORTA_PINCFG28_s    MMIO_REG(0x4000285c, struct __struct_PORTA_PINCFG28)
struct __struct_PORTA_PINCFG28
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG29	MMIO_REG(0x4000285d, uint8_t)
#define PORTA_PINCFG29_PMUXEN	(1 << 0)
#define PORTA_PINCFG29_INEN	(1 << 1)
#define PORTA_PINCFG29_PULLEN	(1 << 2)

#define PORTA_PINCFG29_s    MMIO_REG(0x4000285d, struct __struct_PORTA_PINCFG29)
struct __struct_PORTA_PINCFG29
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG30	MMIO_REG(0x4000285e, uint8_t)
#define PORTA_PINCFG30_PMUXEN	(1 << 0)
#define PORTA_PINCFG30_INEN	(1 << 1)
#define PORTA_PINCFG30_PULLEN	(1 << 2)

#define PORTA_PINCFG30_s    MMIO_REG(0x4000285e, struct __struct_PORTA_PINCFG30)
struct __struct_PORTA_PINCFG30
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTA_PINCFG31	MMIO_REG(0x4000285f, uint8_t)
#define PORTA_PINCFG31_PMUXEN	(1 << 0)
#define PORTA_PINCFG31_INEN	(1 << 1)
#define PORTA_PINCFG31_PULLEN	(1 << 2)

#define PORTA_PINCFG31_s    MMIO_REG(0x4100285f, struct __struct_PORTA_PINCFG31)
struct __struct_PORTA_PINCFG31
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

// 0x40002880 PORTB

#define PORTB_DIR	MMIO_REG(0x40002880, int32_t)
#define PORTB_DIR_P0	(1 << 0)
#define PORTB_DIR_P1	(1 << 1)
#define PORTB_DIR_P2	(1 << 2)
#define PORTB_DIR_P3	(1 << 3)
#define PORTB_DIR_P4	(1 << 4)
#define PORTB_DIR_P5	(1 << 5)
#define PORTB_DIR_P6	(1 << 6)
#define PORTB_DIR_P7	(1 << 7)
#define PORTB_DIR_P8	(1 << 8)
#define PORTB_DIR_P9	(1 << 9)
#define PORTB_DIR_P10	(1 << 10)
#define PORTB_DIR_P11	(1 << 11)
#define PORTB_DIR_P12	(1 << 12)
#define PORTB_DIR_P13	(1 << 13)
#define PORTB_DIR_P14	(1 << 14)
#define PORTB_DIR_P15	(1 << 15)
#define PORTB_DIR_P16	(1 << 16)
#define PORTB_DIR_P17	(1 << 17)
#define PORTB_DIR_P18	(1 << 18)
#define PORTB_DIR_P19	(1 << 19)
#define PORTB_DIR_P20	(1 << 20)
#define PORTB_DIR_P21	(1 << 21)
#define PORTB_DIR_P22	(1 << 22)
#define PORTB_DIR_P23	(1 << 23)
#define PORTB_DIR_P24	(1 << 24)
#define PORTB_DIR_P25	(1 << 25)
#define PORTB_DIR_P26	(1 << 26)
#define PORTB_DIR_P27	(1 << 27)
#define PORTB_DIR_P28	(1 << 28)
#define PORTB_DIR_P29	(1 << 29)
#define PORTB_DIR_P30	(1 << 30)
#define PORTB_DIR_P31	(1 << 31)

#define PORTB_DIR_s    MMIO_REG(0x40002880, struct __struct_PORTB_DIR)
struct __struct_PORTB_DIR
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTB_DIRCLR	MMIO_REG(0x40002884, int32_t)
#define PORTB_DIRCLR_P0	(1 << 0)
#define PORTB_DIRCLR_P1	(1 << 1)
#define PORTB_DIRCLR_P2	(1 << 2)
#define PORTB_DIRCLR_P3	(1 << 3)
#define PORTB_DIRCLR_P4	(1 << 4)
#define PORTB_DIRCLR_P5	(1 << 5)
#define PORTB_DIRCLR_P6	(1 << 6)
#define PORTB_DIRCLR_P7	(1 << 7)
#define PORTB_DIRCLR_P8	(1 << 8)
#define PORTB_DIRCLR_P9	(1 << 9)
#define PORTB_DIRCLR_P10	(1 << 10)
#define PORTB_DIRCLR_P11	(1 << 11)
#define PORTB_DIRCLR_P12	(1 << 12)
#define PORTB_DIRCLR_P13	(1 << 13)
#define PORTB_DIRCLR_P14	(1 << 14)
#define PORTB_DIRCLR_P15	(1 << 15)
#define PORTB_DIRCLR_P16	(1 << 16)
#define PORTB_DIRCLR_P17	(1 << 17)
#define PORTB_DIRCLR_P18	(1 << 18)
#define PORTB_DIRCLR_P19	(1 << 19)
#define PORTB_DIRCLR_P20	(1 << 20)
#define PORTB_DIRCLR_P21	(1 << 21)
#define PORTB_DIRCLR_P22	(1 << 22)
#define PORTB_DIRCLR_P23	(1 << 23)
#define PORTB_DIRCLR_P24	(1 << 24)
#define PORTB_DIRCLR_P25	(1 << 25)
#define PORTB_DIRCLR_P26	(1 << 26)
#define PORTB_DIRCLR_P27	(1 << 27)
#define PORTB_DIRCLR_P28	(1 << 28)
#define PORTB_DIRCLR_P29	(1 << 29)
#define PORTB_DIRCLR_P30	(1 << 30)
#define PORTB_DIRCLR_P31	(1 << 31)

#define PORTB_DIRCLR_s    MMIO_REG(0x40002884, struct __struct_PORTB_DIRCLR)
struct __struct_PORTB_DIRCLR
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTB_DIRSET	MMIO_REG(0x40002888, int32_t)
#define PORTB_DIRSET_P0	(1 << 0)
#define PORTB_DIRSET_P1	(1 << 1)
#define PORTB_DIRSET_P2	(1 << 2)
#define PORTB_DIRSET_P3	(1 << 3)
#define PORTB_DIRSET_P4	(1 << 4)
#define PORTB_DIRSET_P5	(1 << 5)
#define PORTB_DIRSET_P6	(1 << 6)
#define PORTB_DIRSET_P7	(1 << 7)
#define PORTB_DIRSET_P8	(1 << 8)
#define PORTB_DIRSET_P9	(1 << 9)
#define PORTB_DIRSET_P10	(1 << 10)
#define PORTB_DIRSET_P11	(1 << 11)
#define PORTB_DIRSET_P12	(1 << 12)
#define PORTB_DIRSET_P13	(1 << 13)
#define PORTB_DIRSET_P14	(1 << 14)
#define PORTB_DIRSET_P15	(1 << 15)
#define PORTB_DIRSET_P16	(1 << 16)
#define PORTB_DIRSET_P17	(1 << 17)
#define PORTB_DIRSET_P18	(1 << 18)
#define PORTB_DIRSET_P19	(1 << 19)
#define PORTB_DIRSET_P20	(1 << 20)
#define PORTB_DIRSET_P21	(1 << 21)
#define PORTB_DIRSET_P22	(1 << 22)
#define PORTB_DIRSET_P23	(1 << 23)
#define PORTB_DIRSET_P24	(1 << 24)
#define PORTB_DIRSET_P25	(1 << 25)
#define PORTB_DIRSET_P26	(1 << 26)
#define PORTB_DIRSET_P27	(1 << 27)
#define PORTB_DIRSET_P28	(1 << 28)
#define PORTB_DIRSET_P29	(1 << 29)
#define PORTB_DIRSET_P30	(1 << 30)
#define PORTB_DIRSET_P31	(1 << 31)

#define PORTB_DIRSET_s    MMIO_REG(0x40002888, struct __struct_PORTB_DIRSET)
struct __struct_PORTB_DIRSET
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTB_DIRTGL	MMIO_REG(0x4000288c, int32_t)
#define PORTB_DIRTGL_P0	(1 << 0)
#define PORTB_DIRTGL_P1	(1 << 1)
#define PORTB_DIRTGL_P2	(1 << 2)
#define PORTB_DIRTGL_P3	(1 << 3)
#define PORTB_DIRTGL_P4	(1 << 4)
#define PORTB_DIRTGL_P5	(1 << 5)
#define PORTB_DIRTGL_P6	(1 << 6)
#define PORTB_DIRTGL_P7	(1 << 7)
#define PORTB_DIRTGL_P8	(1 << 8)
#define PORTB_DIRTGL_P9	(1 << 9)
#define PORTB_DIRTGL_P10	(1 << 10)
#define PORTB_DIRTGL_P11	(1 << 11)
#define PORTB_DIRTGL_P12	(1 << 12)
#define PORTB_DIRTGL_P13	(1 << 13)
#define PORTB_DIRTGL_P14	(1 << 14)
#define PORTB_DIRTGL_P15	(1 << 15)
#define PORTB_DIRTGL_P16	(1 << 16)
#define PORTB_DIRTGL_P17	(1 << 17)
#define PORTB_DIRTGL_P18	(1 << 18)
#define PORTB_DIRTGL_P19	(1 << 19)
#define PORTB_DIRTGL_P20	(1 << 20)
#define PORTB_DIRTGL_P21	(1 << 21)
#define PORTB_DIRTGL_P22	(1 << 22)
#define PORTB_DIRTGL_P23	(1 << 23)
#define PORTB_DIRTGL_P24	(1 << 24)
#define PORTB_DIRTGL_P25	(1 << 25)
#define PORTB_DIRTGL_P26	(1 << 26)
#define PORTB_DIRTGL_P27	(1 << 27)
#define PORTB_DIRTGL_P28	(1 << 28)
#define PORTB_DIRTGL_P29	(1 << 29)
#define PORTB_DIRTGL_P30	(1 << 30)
#define PORTB_DIRTGL_P31	(1 << 31)

#define PORTB_DIRTGL_s    MMIO_REG(0x4000288c, struct __struct_PORTB_DIRTGL)
struct __struct_PORTB_DIRTGL
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTB_OUT	MMIO_REG(0x40002890, int32_t)
#define PORTB_OUT_P0	(1 << 0)
#define PORTB_OUT_P1	(1 << 1)
#define PORTB_OUT_P2	(1 << 2)
#define PORTB_OUT_P3	(1 << 3)
#define PORTB_OUT_P4	(1 << 4)
#define PORTB_OUT_P5	(1 << 5)
#define PORTB_OUT_P6	(1 << 6)
#define PORTB_OUT_P7	(1 << 7)
#define PORTB_OUT_P8	(1 << 8)
#define PORTB_OUT_P9	(1 << 9)
#define PORTB_OUT_P10	(1 << 10)
#define PORTB_OUT_P11	(1 << 11)
#define PORTB_OUT_P12	(1 << 12)
#define PORTB_OUT_P13	(1 << 13)
#define PORTB_OUT_P14	(1 << 14)
#define PORTB_OUT_P15	(1 << 15)
#define PORTB_OUT_P16	(1 << 16)
#define PORTB_OUT_P17	(1 << 17)
#define PORTB_OUT_P18	(1 << 18)
#define PORTB_OUT_P19	(1 << 19)
#define PORTB_OUT_P20	(1 << 20)
#define PORTB_OUT_P21	(1 << 21)
#define PORTB_OUT_P22	(1 << 22)
#define PORTB_OUT_P23	(1 << 23)
#define PORTB_OUT_P24	(1 << 24)
#define PORTB_OUT_P25	(1 << 25)
#define PORTB_OUT_P26	(1 << 26)
#define PORTB_OUT_P27	(1 << 27)
#define PORTB_OUT_P28	(1 << 28)
#define PORTB_OUT_P29	(1 << 29)
#define PORTB_OUT_P30	(1 << 30)
#define PORTB_OUT_P31	(1 << 31)

#define PORTB_OUT_s    MMIO_REG(0x40002890, struct __struct_PORTB_OUT)
struct __struct_PORTB_OUT
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTB_OUTCLR	MMIO_REG(0x40002894, int32_t)
#define PORTB_OUTCLR_P0	(1 << 0)
#define PORTB_OUTCLR_P1	(1 << 1)
#define PORTB_OUTCLR_P2	(1 << 2)
#define PORTB_OUTCLR_P3	(1 << 3)
#define PORTB_OUTCLR_P4	(1 << 4)
#define PORTB_OUTCLR_P5	(1 << 5)
#define PORTB_OUTCLR_P6	(1 << 6)
#define PORTB_OUTCLR_P7	(1 << 7)
#define PORTB_OUTCLR_P8	(1 << 8)
#define PORTB_OUTCLR_P9	(1 << 9)
#define PORTB_OUTCLR_P10	(1 << 10)
#define PORTB_OUTCLR_P11	(1 << 11)
#define PORTB_OUTCLR_P12	(1 << 12)
#define PORTB_OUTCLR_P13	(1 << 13)
#define PORTB_OUTCLR_P14	(1 << 14)
#define PORTB_OUTCLR_P15	(1 << 15)
#define PORTB_OUTCLR_P16	(1 << 16)
#define PORTB_OUTCLR_P17	(1 << 17)
#define PORTB_OUTCLR_P18	(1 << 18)
#define PORTB_OUTCLR_P19	(1 << 19)
#define PORTB_OUTCLR_P20	(1 << 20)
#define PORTB_OUTCLR_P21	(1 << 21)
#define PORTB_OUTCLR_P22	(1 << 22)
#define PORTB_OUTCLR_P23	(1 << 23)
#define PORTB_OUTCLR_P24	(1 << 24)
#define PORTB_OUTCLR_P25	(1 << 25)
#define PORTB_OUTCLR_P26	(1 << 26)
#define PORTB_OUTCLR_P27	(1 << 27)
#define PORTB_OUTCLR_P28	(1 << 28)
#define PORTB_OUTCLR_P29	(1 << 29)
#define PORTB_OUTCLR_P30	(1 << 30)
#define PORTB_OUTCLR_P31	(1 << 31)

#define PORTB_OUTCLR_s    MMIO_REG(0x40002894, struct __struct_PORTB_OUTCLR)
struct __struct_PORTB_OUTCLR
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTB_OUTSET	MMIO_REG(0x40002898, int32_t)
#define PORTB_OUTSET_P0	(1 << 0)
#define PORTB_OUTSET_P1	(1 << 1)
#define PORTB_OUTSET_P2	(1 << 2)
#define PORTB_OUTSET_P3	(1 << 3)
#define PORTB_OUTSET_P4	(1 << 4)
#define PORTB_OUTSET_P5	(1 << 5)
#define PORTB_OUTSET_P6	(1 << 6)
#define PORTB_OUTSET_P7	(1 << 7)
#define PORTB_OUTSET_P8	(1 << 8)
#define PORTB_OUTSET_P9	(1 << 9)
#define PORTB_OUTSET_P10	(1 << 10)
#define PORTB_OUTSET_P11	(1 << 11)
#define PORTB_OUTSET_P12	(1 << 12)
#define PORTB_OUTSET_P13	(1 << 13)
#define PORTB_OUTSET_P14	(1 << 14)
#define PORTB_OUTSET_P15	(1 << 15)
#define PORTB_OUTSET_P16	(1 << 16)
#define PORTB_OUTSET_P17	(1 << 17)
#define PORTB_OUTSET_P18	(1 << 18)
#define PORTB_OUTSET_P19	(1 << 19)
#define PORTB_OUTSET_P20	(1 << 20)
#define PORTB_OUTSET_P21	(1 << 21)
#define PORTB_OUTSET_P22	(1 << 22)
#define PORTB_OUTSET_P23	(1 << 23)
#define PORTB_OUTSET_P24	(1 << 24)
#define PORTB_OUTSET_P25	(1 << 25)
#define PORTB_OUTSET_P26	(1 << 26)
#define PORTB_OUTSET_P27	(1 << 27)
#define PORTB_OUTSET_P28	(1 << 28)
#define PORTB_OUTSET_P29	(1 << 29)
#define PORTB_OUTSET_P30	(1 << 30)
#define PORTB_OUTSET_P31	(1 << 31)

#define PORTB_OUTSET_s    MMIO_REG(0x40002898, struct __struct_PORTB_OUTSET)
struct __struct_PORTB_OUTSET
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTB_OUTTGL	MMIO_REG(0x4000289c, int32_t)
#define PORTB_OUTTGL_P0	(1 << 0)
#define PORTB_OUTTGL_P1	(1 << 1)
#define PORTB_OUTTGL_P2	(1 << 2)
#define PORTB_OUTTGL_P3	(1 << 3)
#define PORTB_OUTTGL_P4	(1 << 4)
#define PORTB_OUTTGL_P5	(1 << 5)
#define PORTB_OUTTGL_P6	(1 << 6)
#define PORTB_OUTTGL_P7	(1 << 7)
#define PORTB_OUTTGL_P8	(1 << 8)
#define PORTB_OUTTGL_P9	(1 << 9)
#define PORTB_OUTTGL_P10	(1 << 10)
#define PORTB_OUTTGL_P11	(1 << 11)
#define PORTB_OUTTGL_P12	(1 << 12)
#define PORTB_OUTTGL_P13	(1 << 13)
#define PORTB_OUTTGL_P14	(1 << 14)
#define PORTB_OUTTGL_P15	(1 << 15)
#define PORTB_OUTTGL_P16	(1 << 16)
#define PORTB_OUTTGL_P17	(1 << 17)
#define PORTB_OUTTGL_P18	(1 << 18)
#define PORTB_OUTTGL_P19	(1 << 19)
#define PORTB_OUTTGL_P20	(1 << 20)
#define PORTB_OUTTGL_P21	(1 << 21)
#define PORTB_OUTTGL_P22	(1 << 22)
#define PORTB_OUTTGL_P23	(1 << 23)
#define PORTB_OUTTGL_P24	(1 << 24)
#define PORTB_OUTTGL_P25	(1 << 25)
#define PORTB_OUTTGL_P26	(1 << 26)
#define PORTB_OUTTGL_P27	(1 << 27)
#define PORTB_OUTTGL_P28	(1 << 28)
#define PORTB_OUTTGL_P29	(1 << 29)
#define PORTB_OUTTGL_P30	(1 << 30)
#define PORTB_OUTTGL_P31	(1 << 31)

#define PORTB_OUTTGL_s    MMIO_REG(0x4000289c, struct __struct_PORTB_OUTTGL)
struct __struct_PORTB_OUTTGL
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTB_IN	MMIO_REG(0x400028a0, int32_t)
#define PORTB_IN_P0	(1 << 0)
#define PORTB_IN_P1	(1 << 1)
#define PORTB_IN_P2	(1 << 2)
#define PORTB_IN_P3	(1 << 3)
#define PORTB_IN_P4	(1 << 4)
#define PORTB_IN_P5	(1 << 5)
#define PORTB_IN_P6	(1 << 6)
#define PORTB_IN_P7	(1 << 7)
#define PORTB_IN_P8	(1 << 8)
#define PORTB_IN_P9	(1 << 9)
#define PORTB_IN_P10	(1 << 10)
#define PORTB_IN_P11	(1 << 11)
#define PORTB_IN_P12	(1 << 12)
#define PORTB_IN_P13	(1 << 13)
#define PORTB_IN_P14	(1 << 14)
#define PORTB_IN_P15	(1 << 15)
#define PORTB_IN_P16	(1 << 16)
#define PORTB_IN_P17	(1 << 17)
#define PORTB_IN_P18	(1 << 18)
#define PORTB_IN_P19	(1 << 19)
#define PORTB_IN_P20	(1 << 20)
#define PORTB_IN_P21	(1 << 21)
#define PORTB_IN_P22	(1 << 22)
#define PORTB_IN_P23	(1 << 23)
#define PORTB_IN_P24	(1 << 24)
#define PORTB_IN_P25	(1 << 25)
#define PORTB_IN_P26	(1 << 26)
#define PORTB_IN_P27	(1 << 27)
#define PORTB_IN_P28	(1 << 28)
#define PORTB_IN_P29	(1 << 29)
#define PORTB_IN_P30	(1 << 30)
#define PORTB_IN_P31	(1 << 31)

#define PORTB_IN_s    MMIO_REG(0x400028a0, struct __struct_PORTB_IN)
struct __struct_PORTB_IN
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTB_CTRL	MMIO_REG(0x400028a4, int32_t)

#define PORTB_WRCONFIG	MMIO_REG(0x400028a8, int32_t)
#define PORTB_WRCONFIG_PINMASK(x)	((x) << 0)
#define PORTB_WRCONFIG_PINMASK_MSK	0x0000ffff
#define PORTB_WRCONFIG_PINMASK_VAL(x)	(((x) >> 0) & 0xffff)
#define PORTB_WRCONFIG_PMUXEN	(1 << 16)
#define PORTB_WRCONFIG_INEN	(1 << 17)
#define PORTB_WRCONFIG_PULLEN	(1 << 18)
#define PORTB_WRCONFIG_PMUX(x)	((x) << 24)
#define PORTB_WRCONFIG_PMUX_MSK	0x0f000000
#define PORTB_WRCONFIG_PMUX_VAL(x)	(((x) >> 24) & 0xf)
#define PORTB_WRCONFIG_WRPMUX	(1 << 28)
#define PORTB_WRCONFIG_WRPINCFG	(1 << 30)
#define PORTB_WRCONFIG_HWSEL	(1 << 31)

#define PORTB_WRCONFIG_s    MMIO_REG(0x400028a8, struct __struct_PORTB_WRCONFIG)
struct __struct_PORTB_WRCONFIG
{
  uint32_t pinmask : 16;
  uint32_t pmuxen : 1;
  uint32_t inen : 1;
  uint32_t pullen : 1;
  uint32_t  : 5;
  uint32_t pmux : 4;
  uint32_t wrpmux : 1;
  uint32_t  : 1;
  uint32_t wrpincfg : 1;
  uint32_t hwsel : 1;
};

#define PORTB_PMUX0	MMIO_REG(0x400028b0, uint8_t)
#define PORTB_PMUX0_PMUXE(x)	((x) << 0)
#define PORTB_PMUX0_PMUXE_MSK	0x0000000f
#define PORTB_PMUX0_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX0_PMUXO(x)	((x) << 4)
#define PORTB_PMUX0_PMUXO_MSK	0x000000f0
#define PORTB_PMUX0_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX0_s    MMIO_REG(0x400028b0, struct __struct_PORTB_PMUX0)
struct __struct_PORTB_PMUX0
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX1	MMIO_REG(0x400028b1, uint8_t)
#define PORTB_PMUX1_PMUXE(x)	((x) << 0)
#define PORTB_PMUX1_PMUXE_MSK	0x0000000f
#define PORTB_PMUX1_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX1_PMUXO(x)	((x) << 4)
#define PORTB_PMUX1_PMUXO_MSK	0x000000f0
#define PORTB_PMUX1_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX1_s    MMIO_REG(0x400028b1, struct __struct_PORTB_PMUX1)
struct __struct_PORTB_PMUX1
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX2	MMIO_REG(0x400028b2, uint8_t)
#define PORTB_PMUX2_PMUXE(x)	((x) << 0)
#define PORTB_PMUX2_PMUXE_MSK	0x0000000f
#define PORTB_PMUX2_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX2_PMUXO(x)	((x) << 4)
#define PORTB_PMUX2_PMUXO_MSK	0x000000f0
#define PORTB_PMUX2_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX2_s    MMIO_REG(0x400028b2, struct __struct_PORTB_PMUX2)
struct __struct_PORTB_PMUX2
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX3	MMIO_REG(0x400028b3, uint8_t)
#define PORTB_PMUX3_PMUXE(x)	((x) << 0)
#define PORTB_PMUX3_PMUXE_MSK	0x0000000f
#define PORTB_PMUX3_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX3_PMUXO(x)	((x) << 4)
#define PORTB_PMUX3_PMUXO_MSK	0x000000f0
#define PORTB_PMUX3_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX3_s    MMIO_REG(0x400028b3, struct __struct_PORTB_PMUX3)
struct __struct_PORTB_PMUX3
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX4	MMIO_REG(0x400028b4, uint8_t)
#define PORTB_PMUX4_PMUXE(x)	((x) << 0)
#define PORTB_PMUX4_PMUXE_MSK	0x0000000f
#define PORTB_PMUX4_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX4_PMUXO(x)	((x) << 4)
#define PORTB_PMUX4_PMUXO_MSK	0x000000f0
#define PORTB_PMUX4_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX4_s    MMIO_REG(0x400028b4, struct __struct_PORTB_PMUX4)
struct __struct_PORTB_PMUX4
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX5	MMIO_REG(0x400028b5, uint8_t)
#define PORTB_PMUX5_PMUXE(x)	((x) << 0)
#define PORTB_PMUX5_PMUXE_MSK	0x0000000f
#define PORTB_PMUX5_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX5_PMUXO(x)	((x) << 4)
#define PORTB_PMUX5_PMUXO_MSK	0x000000f0
#define PORTB_PMUX5_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX5_s    MMIO_REG(0x400028b5, struct __struct_PORTB_PMUX5)
struct __struct_PORTB_PMUX5
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX6	MMIO_REG(0x400028b6, uint8_t)
#define PORTB_PMUX6_PMUXE(x)	((x) << 0)
#define PORTB_PMUX6_PMUXE_MSK	0x0000000f
#define PORTB_PMUX6_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX6_PMUXO(x)	((x) << 4)
#define PORTB_PMUX6_PMUXO_MSK	0x000000f0
#define PORTB_PMUX6_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX6_s    MMIO_REG(0x400028b6, struct __struct_PORTB_PMUX6)
struct __struct_PORTB_PMUX6
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX7	MMIO_REG(0x400028b7, uint8_t)
#define PORTB_PMUX7_PMUXE(x)	((x) << 0)
#define PORTB_PMUX7_PMUXE_MSK	0x0000000f
#define PORTB_PMUX7_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX7_PMUXO(x)	((x) << 4)
#define PORTB_PMUX7_PMUXO_MSK	0x000000f0
#define PORTB_PMUX7_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX7_s    MMIO_REG(0x400028b7, struct __struct_PORTB_PMUX7)
struct __struct_PORTB_PMUX7
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX8	MMIO_REG(0x400028b8, uint8_t)
#define PORTB_PMUX8_PMUXE(x)	((x) << 0)
#define PORTB_PMUX8_PMUXE_MSK	0x0000000f
#define PORTB_PMUX8_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX8_PMUXO(x)	((x) << 4)
#define PORTB_PMUX8_PMUXO_MSK	0x000000f0
#define PORTB_PMUX8_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX8_s    MMIO_REG(0x400028b8, struct __struct_PORTB_PMUX8)
struct __struct_PORTB_PMUX8
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX9	MMIO_REG(0x400028b9, uint8_t)
#define PORTB_PMUX9_PMUXE(x)	((x) << 0)
#define PORTB_PMUX9_PMUXE_MSK	0x0000000f
#define PORTB_PMUX9_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX9_PMUXO(x)	((x) << 4)
#define PORTB_PMUX9_PMUXO_MSK	0x000000f0
#define PORTB_PMUX9_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX9_s    MMIO_REG(0x400028b9, struct __struct_PORTB_PMUX9)
struct __struct_PORTB_PMUX9
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX10	MMIO_REG(0x400028ba, uint8_t)
#define PORTB_PMUX10_PMUXE(x)	((x) << 0)
#define PORTB_PMUX10_PMUXE_MSK	0x0000000f
#define PORTB_PMUX10_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX10_PMUXO(x)	((x) << 4)
#define PORTB_PMUX10_PMUXO_MSK	0x000000f0
#define PORTB_PMUX10_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX10_s    MMIO_REG(0x400028ba, struct __struct_PORTB_PMUX10)
struct __struct_PORTB_PMUX10
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX11	MMIO_REG(0x400028bb, uint8_t)
#define PORTB_PMUX11_PMUXE(x)	((x) << 0)
#define PORTB_PMUX11_PMUXE_MSK	0x0000000f
#define PORTB_PMUX11_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX11_PMUXO(x)	((x) << 4)
#define PORTB_PMUX11_PMUXO_MSK	0x000000f0
#define PORTB_PMUX11_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX11_s    MMIO_REG(0x400028bb, struct __struct_PORTB_PMUX11)
struct __struct_PORTB_PMUX11
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX12	MMIO_REG(0x400028bc, uint8_t)
#define PORTB_PMUX12_PMUXE(x)	((x) << 0)
#define PORTB_PMUX12_PMUXE_MSK	0x0000000f
#define PORTB_PMUX12_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX12_PMUXO(x)	((x) << 4)
#define PORTB_PMUX12_PMUXO_MSK	0x000000f0
#define PORTB_PMUX12_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX12_s    MMIO_REG(0x400028bc, struct __struct_PORTB_PMUX12)
struct __struct_PORTB_PMUX12
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX13	MMIO_REG(0x400028bd, uint8_t)
#define PORTB_PMUX13_PMUXE(x)	((x) << 0)
#define PORTB_PMUX13_PMUXE_MSK	0x0000000f
#define PORTB_PMUX13_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX13_PMUXO(x)	((x) << 4)
#define PORTB_PMUX13_PMUXO_MSK	0x000000f0
#define PORTB_PMUX13_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX13_s    MMIO_REG(0x400028bd, struct __struct_PORTB_PMUX13)
struct __struct_PORTB_PMUX13
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX14	MMIO_REG(0x400028be, uint8_t)
#define PORTB_PMUX14_PMUXE(x)	((x) << 0)
#define PORTB_PMUX14_PMUXE_MSK	0x0000000f
#define PORTB_PMUX14_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX14_PMUXO(x)	((x) << 4)
#define PORTB_PMUX14_PMUXO_MSK	0x000000f0
#define PORTB_PMUX14_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX14_s    MMIO_REG(0x400028be, struct __struct_PORTB_PMUX14)
struct __struct_PORTB_PMUX14
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PMUX15	MMIO_REG(0x400028bf, uint8_t)
#define PORTB_PMUX15_PMUXE(x)	((x) << 0)
#define PORTB_PMUX15_PMUXE_MSK	0x0000000f
#define PORTB_PMUX15_PMUXE_VAL(x)	(((x) >> 0) & 0xf)
#define PORTB_PMUX15_PMUXO(x)	((x) << 4)
#define PORTB_PMUX15_PMUXO_MSK	0x000000f0
#define PORTB_PMUX15_PMUXO_VAL(x)	(((x) >> 4) & 0xf)

#define PORTB_PMUX15_s    MMIO_REG(0x400028bf, struct __struct_PORTB_PMUX15)
struct __struct_PORTB_PMUX15
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTB_PINCFG0	MMIO_REG(0x400028c0, uint8_t)
#define PORTB_PINCFG0_PMUXEN	(1 << 0)
#define PORTB_PINCFG0_INEN	(1 << 1)
#define PORTB_PINCFG0_PULLEN	(1 << 2)

#define PORTB_PINCFG0_s    MMIO_REG(0x400028c0, struct __struct_PORTB_PINCFG0)
struct __struct_PORTB_PINCFG0
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG1	MMIO_REG(0x400028c1, uint8_t)
#define PORTB_PINCFG1_PMUXEN	(1 << 0)
#define PORTB_PINCFG1_INEN	(1 << 1)
#define PORTB_PINCFG1_PULLEN	(1 << 2)

#define PORTB_PINCFG1_s    MMIO_REG(0x400028c1, struct __struct_PORTB_PINCFG1)
struct __struct_PORTB_PINCFG1
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG2	MMIO_REG(0x400028c2, uint8_t)
#define PORTB_PINCFG2_PMUXEN	(1 << 0)
#define PORTB_PINCFG2_INEN	(1 << 1)
#define PORTB_PINCFG2_PULLEN	(1 << 2)

#define PORTB_PINCFG2_s    MMIO_REG(0x400028c2, struct __struct_PORTB_PINCFG2)
struct __struct_PORTB_PINCFG2
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG3	MMIO_REG(0x400028c3, uint8_t)
#define PORTB_PINCFG3_PMUXEN	(1 << 0)
#define PORTB_PINCFG3_INEN	(1 << 1)
#define PORTB_PINCFG3_PULLEN	(1 << 2)

#define PORTB_PINCFG3_s    MMIO_REG(0x400028c3, struct __struct_PORTB_PINCFG3)
struct __struct_PORTB_PINCFG3
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG4	MMIO_REG(0x400028c4, uint8_t)
#define PORTB_PINCFG4_PMUXEN	(1 << 0)
#define PORTB_PINCFG4_INEN	(1 << 1)
#define PORTB_PINCFG4_PULLEN	(1 << 2)

#define PORTB_PINCFG4_s    MMIO_REG(0x400028c4, struct __struct_PORTB_PINCFG4)
struct __struct_PORTB_PINCFG4
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG5	MMIO_REG(0x400028c5, uint8_t)
#define PORTB_PINCFG5_PMUXEN	(1 << 0)
#define PORTB_PINCFG5_INEN	(1 << 1)
#define PORTB_PINCFG5_PULLEN	(1 << 2)

#define PORTB_PINCFG5_s    MMIO_REG(0x400028c5, struct __struct_PORTB_PINCFG5)
struct __struct_PORTB_PINCFG5
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG6	MMIO_REG(0x400028c6, uint8_t)
#define PORTB_PINCFG6_PMUXEN	(1 << 0)
#define PORTB_PINCFG6_INEN	(1 << 1)
#define PORTB_PINCFG6_PULLEN	(1 << 2)

#define PORTB_PINCFG6_s    MMIO_REG(0x400028c6, struct __struct_PORTB_PINCFG6)
struct __struct_PORTB_PINCFG6
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG7	MMIO_REG(0x400028c7, uint8_t)
#define PORTB_PINCFG7_PMUXEN	(1 << 0)
#define PORTB_PINCFG7_INEN	(1 << 1)
#define PORTB_PINCFG7_PULLEN	(1 << 2)

#define PORTB_PINCFG7_s    MMIO_REG(0x400028c7, struct __struct_PORTB_PINCFG7)
struct __struct_PORTB_PINCFG7
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG8	MMIO_REG(0x400028c8, uint8_t)
#define PORTB_PINCFG8_PMUXEN	(1 << 0)
#define PORTB_PINCFG8_INEN	(1 << 1)
#define PORTB_PINCFG8_PULLEN	(1 << 2)

#define PORTB_PINCFG8_s    MMIO_REG(0x400028c8, struct __struct_PORTB_PINCFG8)
struct __struct_PORTB_PINCFG8
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG9	MMIO_REG(0x400028c9, uint8_t)
#define PORTB_PINCFG9_PMUXEN	(1 << 0)
#define PORTB_PINCFG9_INEN	(1 << 1)
#define PORTB_PINCFG9_PULLEN	(1 << 2)

#define PORTB_PINCFG9_s    MMIO_REG(0x400028c9, struct __struct_PORTB_PINCFG9)
struct __struct_PORTB_PINCFG9
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG10	MMIO_REG(0x400028ca, uint8_t)
#define PORTB_PINCFG10_PMUXEN	(1 << 0)
#define PORTB_PINCFG10_INEN	(1 << 1)
#define PORTB_PINCFG10_PULLEN	(1 << 2)

#define PORTB_PINCFG10_s    MMIO_REG(0x400028ca, struct __struct_PORTB_PINCFG10)
struct __struct_PORTB_PINCFG10
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG11	MMIO_REG(0x400028cb, uint8_t)
#define PORTB_PINCFG11_PMUXEN	(1 << 0)
#define PORTB_PINCFG11_INEN	(1 << 1)
#define PORTB_PINCFG11_PULLEN	(1 << 2)

#define PORTB_PINCFG11_s    MMIO_REG(0x400028cb, struct __struct_PORTB_PINCFG11)
struct __struct_PORTB_PINCFG11
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG12	MMIO_REG(0x400028cc, uint8_t)
#define PORTB_PINCFG12_PMUXEN	(1 << 0)
#define PORTB_PINCFG12_INEN	(1 << 1)
#define PORTB_PINCFG12_PULLEN	(1 << 2)

#define PORTB_PINCFG12_s    MMIO_REG(0x400028cc, struct __struct_PORTB_PINCFG12)
struct __struct_PORTB_PINCFG12
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG13	MMIO_REG(0x400028cd, uint8_t)
#define PORTB_PINCFG13_PMUXEN	(1 << 0)
#define PORTB_PINCFG13_INEN	(1 << 1)
#define PORTB_PINCFG13_PULLEN	(1 << 2)

#define PORTB_PINCFG13_s    MMIO_REG(0x400028cd, struct __struct_PORTB_PINCFG13)
struct __struct_PORTB_PINCFG13
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG14	MMIO_REG(0x400028ce, uint8_t)
#define PORTB_PINCFG14_PMUXEN	(1 << 0)
#define PORTB_PINCFG14_INEN	(1 << 1)
#define PORTB_PINCFG14_PULLEN	(1 << 2)

#define PORTB_PINCFG14_s    MMIO_REG(0x400028ce, struct __struct_PORTB_PINCFG14)
struct __struct_PORTB_PINCFG14
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG15	MMIO_REG(0x400028cf, uint8_t)
#define PORTB_PINCFG15_PMUXEN	(1 << 0)
#define PORTB_PINCFG15_INEN	(1 << 1)
#define PORTB_PINCFG15_PULLEN	(1 << 2)

#define PORTB_PINCFG15_s    MMIO_REG(0x400028cf, struct __struct_PORTB_PINCFG15)
struct __struct_PORTB_PINCFG15
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG16	MMIO_REG(0x400028d0, uint8_t)
#define PORTB_PINCFG16_PMUXEN	(1 << 0)
#define PORTB_PINCFG16_INEN	(1 << 1)
#define PORTB_PINCFG16_PULLEN	(1 << 2)

#define PORTB_PINCFG16_s    MMIO_REG(0x400028d0, struct __struct_PORTB_PINCFG16)
struct __struct_PORTB_PINCFG16
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG17	MMIO_REG(0x400028d1, uint8_t)
#define PORTB_PINCFG17_PMUXEN	(1 << 0)
#define PORTB_PINCFG17_INEN	(1 << 1)
#define PORTB_PINCFG17_PULLEN	(1 << 2)

#define PORTB_PINCFG17_s    MMIO_REG(0x400028d1, struct __struct_PORTB_PINCFG17)
struct __struct_PORTB_PINCFG17
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG18	MMIO_REG(0x400028d2, uint8_t)
#define PORTB_PINCFG18_PMUXEN	(1 << 0)
#define PORTB_PINCFG18_INEN	(1 << 1)
#define PORTB_PINCFG18_PULLEN	(1 << 2)

#define PORTB_PINCFG18_s    MMIO_REG(0x400028d2, struct __struct_PORTB_PINCFG18)
struct __struct_PORTB_PINCFG18
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG19	MMIO_REG(0x400028d3, uint8_t)
#define PORTB_PINCFG19_PMUXEN	(1 << 0)
#define PORTB_PINCFG19_INEN	(1 << 1)
#define PORTB_PINCFG19_PULLEN	(1 << 2)

#define PORTB_PINCFG19_s    MMIO_REG(0x400028d3, struct __struct_PORTB_PINCFG19)
struct __struct_PORTB_PINCFG19
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG20	MMIO_REG(0x400028d4, uint8_t)
#define PORTB_PINCFG20_PMUXEN	(1 << 0)
#define PORTB_PINCFG20_INEN	(1 << 1)
#define PORTB_PINCFG20_PULLEN	(1 << 2)

#define PORTB_PINCFG20_s    MMIO_REG(0x400028d4, struct __struct_PORTB_PINCFG20)
struct __struct_PORTB_PINCFG20
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG21	MMIO_REG(0x400028d5, uint8_t)
#define PORTB_PINCFG21_PMUXEN	(1 << 0)
#define PORTB_PINCFG21_INEN	(1 << 1)
#define PORTB_PINCFG21_PULLEN	(1 << 2)

#define PORTB_PINCFG21_s    MMIO_REG(0x400028d5, struct __struct_PORTB_PINCFG21)
struct __struct_PORTB_PINCFG21
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG22	MMIO_REG(0x400028d6, uint8_t)
#define PORTB_PINCFG22_PMUXEN	(1 << 0)
#define PORTB_PINCFG22_INEN	(1 << 1)
#define PORTB_PINCFG22_PULLEN	(1 << 2)

#define PORTB_PINCFG22_s    MMIO_REG(0x400028d6, struct __struct_PORTB_PINCFG22)
struct __struct_PORTB_PINCFG22
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG23	MMIO_REG(0x400028d7, uint8_t)
#define PORTB_PINCFG23_PMUXEN	(1 << 0)
#define PORTB_PINCFG23_INEN	(1 << 1)
#define PORTB_PINCFG23_PULLEN	(1 << 2)

#define PORTB_PINCFG23_s    MMIO_REG(0x400028d7, struct __struct_PORTB_PINCFG23)
struct __struct_PORTB_PINCFG23
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG24	MMIO_REG(0x400028d8, uint8_t)
#define PORTB_PINCFG24_PMUXEN	(1 << 0)
#define PORTB_PINCFG24_INEN	(1 << 1)
#define PORTB_PINCFG24_PULLEN	(1 << 2)

#define PORTB_PINCFG24_s    MMIO_REG(0x400028d8, struct __struct_PORTB_PINCFG24)
struct __struct_PORTB_PINCFG24
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG25	MMIO_REG(0x400028d9, uint8_t)
#define PORTB_PINCFG25_PMUXEN	(1 << 0)
#define PORTB_PINCFG25_INEN	(1 << 1)
#define PORTB_PINCFG25_PULLEN	(1 << 2)

#define PORTB_PINCFG25_s    MMIO_REG(0x400028d9, struct __struct_PORTB_PINCFG25)
struct __struct_PORTB_PINCFG25
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG26	MMIO_REG(0x400028da, uint8_t)
#define PORTB_PINCFG26_PMUXEN	(1 << 0)
#define PORTB_PINCFG26_INEN	(1 << 1)
#define PORTB_PINCFG26_PULLEN	(1 << 2)

#define PORTB_PINCFG26_s    MMIO_REG(0x400028da, struct __struct_PORTB_PINCFG26)
struct __struct_PORTB_PINCFG26
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG27	MMIO_REG(0x400028db, uint8_t)
#define PORTB_PINCFG27_PMUXEN	(1 << 0)
#define PORTB_PINCFG27_INEN	(1 << 1)
#define PORTB_PINCFG27_PULLEN	(1 << 2)

#define PORTB_PINCFG27_s    MMIO_REG(0x400028db, struct __struct_PORTB_PINCFG27)
struct __struct_PORTB_PINCFG27
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG28	MMIO_REG(0x400028dc, uint8_t)
#define PORTB_PINCFG28_PMUXEN	(1 << 0)
#define PORTB_PINCFG28_INEN	(1 << 1)
#define PORTB_PINCFG28_PULLEN	(1 << 2)

#define PORTB_PINCFG28_s    MMIO_REG(0x400028dc, struct __struct_PORTB_PINCFG28)
struct __struct_PORTB_PINCFG28
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG29	MMIO_REG(0x400028dd, uint8_t)
#define PORTB_PINCFG29_PMUXEN	(1 << 0)
#define PORTB_PINCFG29_INEN	(1 << 1)
#define PORTB_PINCFG29_PULLEN	(1 << 2)

#define PORTB_PINCFG29_s    MMIO_REG(0x400028dd, struct __struct_PORTB_PINCFG29)
struct __struct_PORTB_PINCFG29
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG30	MMIO_REG(0x400028de, uint8_t)
#define PORTB_PINCFG30_PMUXEN	(1 << 0)
#define PORTB_PINCFG30_INEN	(1 << 1)
#define PORTB_PINCFG30_PULLEN	(1 << 2)

#define PORTB_PINCFG30_s    MMIO_REG(0x400028de, struct __struct_PORTB_PINCFG30)
struct __struct_PORTB_PINCFG30
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTB_PINCFG31	MMIO_REG(0x400028df, uint8_t)
#define PORTB_PINCFG31_PMUXEN	(1 << 0)
#define PORTB_PINCFG31_INEN	(1 << 1)
#define PORTB_PINCFG31_PULLEN	(1 << 2)

#define PORTB_PINCFG31_s    MMIO_REG(0x400028df, struct __struct_PORTB_PINCFG31)
struct __struct_PORTB_PINCFG31
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

// 0x40002900 PORTC

#define PORTC_DIR MMIO_REG(0x40002900, uint32_t)
#define PORTC_DIR_P0  (1 << 0)
#define PORTC_DIR_P1  (1 << 1)
#define PORTC_DIR_P2  (1 << 2)
#define PORTC_DIR_P3  (1 << 3)
#define PORTC_DIR_P4  (1 << 4)
#define PORTC_DIR_P5  (1 << 5)
#define PORTC_DIR_P6  (1 << 6)
#define PORTC_DIR_P7  (1 << 7)
#define PORTC_DIR_P8  (1 << 8)
#define PORTC_DIR_P9  (1 << 9)
#define PORTC_DIR_P10 (1 << 10)
#define PORTC_DIR_P11 (1 << 11)
#define PORTC_DIR_P12 (1 << 12)
#define PORTC_DIR_P13 (1 << 13)
#define PORTC_DIR_P14 (1 << 14)
#define PORTC_DIR_P15 (1 << 15)
#define PORTC_DIR_P16 (1 << 16)
#define PORTC_DIR_P17 (1 << 17)
#define PORTC_DIR_P18 (1 << 18)
#define PORTC_DIR_P19 (1 << 19)
#define PORTC_DIR_P20 (1 << 20)
#define PORTC_DIR_P21 (1 << 21)
#define PORTC_DIR_P22 (1 << 22)
#define PORTC_DIR_P23 (1 << 23)
#define PORTC_DIR_P24 (1 << 24)
#define PORTC_DIR_P25 (1 << 25)
#define PORTC_DIR_P26 (1 << 26)
#define PORTC_DIR_P27 (1 << 27)
#define PORTC_DIR_P28 (1 << 28)
#define PORTC_DIR_P29 (1 << 29)
#define PORTC_DIR_P30 (1 << 30)
#define PORTC_DIR_P31 (1 << 31)

#define PORTC_DIR_s    MMIO_REG(0x40002900, struct __struct_PORTC_DIR)
struct __struct_PORTC_DIR
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTC_DIRCLR  MMIO_REG(0x40002904, uint32_t)
#define PORTC_DIRCLR_P0 (1 << 0)
#define PORTC_DIRCLR_P1 (1 << 1)
#define PORTC_DIRCLR_P2 (1 << 2)
#define PORTC_DIRCLR_P3 (1 << 3)
#define PORTC_DIRCLR_P4 (1 << 4)
#define PORTC_DIRCLR_P5 (1 << 5)
#define PORTC_DIRCLR_P6 (1 << 6)
#define PORTC_DIRCLR_P7 (1 << 7)
#define PORTC_DIRCLR_P8 (1 << 8)
#define PORTC_DIRCLR_P9 (1 << 9)
#define PORTC_DIRCLR_P10  (1 << 10)
#define PORTC_DIRCLR_P11  (1 << 11)
#define PORTC_DIRCLR_P12  (1 << 12)
#define PORTC_DIRCLR_P13  (1 << 13)
#define PORTC_DIRCLR_P14  (1 << 14)
#define PORTC_DIRCLR_P15  (1 << 15)
#define PORTC_DIRCLR_P16  (1 << 16)
#define PORTC_DIRCLR_P17  (1 << 17)
#define PORTC_DIRCLR_P18  (1 << 18)
#define PORTC_DIRCLR_P19  (1 << 19)
#define PORTC_DIRCLR_P20  (1 << 20)
#define PORTC_DIRCLR_P21  (1 << 21)
#define PORTC_DIRCLR_P22  (1 << 22)
#define PORTC_DIRCLR_P23  (1 << 23)
#define PORTC_DIRCLR_P24  (1 << 24)
#define PORTC_DIRCLR_P25  (1 << 25)
#define PORTC_DIRCLR_P26  (1 << 26)
#define PORTC_DIRCLR_P27  (1 << 27)
#define PORTC_DIRCLR_P28  (1 << 28)
#define PORTC_DIRCLR_P29  (1 << 29)
#define PORTC_DIRCLR_P30  (1 << 30)
#define PORTC_DIRCLR_P31  (1 << 31)

#define PORTC_DIRCLR_s    MMIO_REG(0x40002904, struct __struct_PORTC_DIRCLR)
struct __struct_PORTC_DIRCLR
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTC_DIRSET  MMIO_REG(0x40002908, uint32_t)
#define PORTC_DIRSET_P0 (1 << 0)
#define PORTC_DIRSET_P1 (1 << 1)
#define PORTC_DIRSET_P2 (1 << 2)
#define PORTC_DIRSET_P3 (1 << 3)
#define PORTC_DIRSET_P4 (1 << 4)
#define PORTC_DIRSET_P5 (1 << 5)
#define PORTC_DIRSET_P6 (1 << 6)
#define PORTC_DIRSET_P7 (1 << 7)
#define PORTC_DIRSET_P8 (1 << 8)
#define PORTC_DIRSET_P9 (1 << 9)
#define PORTC_DIRSET_P10  (1 << 10)
#define PORTC_DIRSET_P11  (1 << 11)
#define PORTC_DIRSET_P12  (1 << 12)
#define PORTC_DIRSET_P13  (1 << 13)
#define PORTC_DIRSET_P14  (1 << 14)
#define PORTC_DIRSET_P15  (1 << 15)
#define PORTC_DIRSET_P16  (1 << 16)
#define PORTC_DIRSET_P17  (1 << 17)
#define PORTC_DIRSET_P18  (1 << 18)
#define PORTC_DIRSET_P19  (1 << 19)
#define PORTC_DIRSET_P20  (1 << 20)
#define PORTC_DIRSET_P21  (1 << 21)
#define PORTC_DIRSET_P22  (1 << 22)
#define PORTC_DIRSET_P23  (1 << 23)
#define PORTC_DIRSET_P24  (1 << 24)
#define PORTC_DIRSET_P25  (1 << 25)
#define PORTC_DIRSET_P26  (1 << 26)
#define PORTC_DIRSET_P27  (1 << 27)
#define PORTC_DIRSET_P28  (1 << 28)
#define PORTC_DIRSET_P29  (1 << 29)
#define PORTC_DIRSET_P30  (1 << 30)
#define PORTC_DIRSET_P31  (1 << 31)

#define PORTC_DIRSET_s    MMIO_REG(0x40002908, struct __struct_PORTC_DIRSET)
struct __struct_PORTC_DIRSET
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTC_DIRTGL  MMIO_REG(0x4000290c, uint32_t)
#define PORTC_DIRTGL_P0 (1 << 0)
#define PORTC_DIRTGL_P1 (1 << 1)
#define PORTC_DIRTGL_P2 (1 << 2)
#define PORTC_DIRTGL_P3 (1 << 3)
#define PORTC_DIRTGL_P4 (1 << 4)
#define PORTC_DIRTGL_P5 (1 << 5)
#define PORTC_DIRTGL_P6 (1 << 6)
#define PORTC_DIRTGL_P7 (1 << 7)
#define PORTC_DIRTGL_P8 (1 << 8)
#define PORTC_DIRTGL_P9 (1 << 9)
#define PORTC_DIRTGL_P10  (1 << 10)
#define PORTC_DIRTGL_P11  (1 << 11)
#define PORTC_DIRTGL_P12  (1 << 12)
#define PORTC_DIRTGL_P13  (1 << 13)
#define PORTC_DIRTGL_P14  (1 << 14)
#define PORTC_DIRTGL_P15  (1 << 15)
#define PORTC_DIRTGL_P16  (1 << 16)
#define PORTC_DIRTGL_P17  (1 << 17)
#define PORTC_DIRTGL_P18  (1 << 18)
#define PORTC_DIRTGL_P19  (1 << 19)
#define PORTC_DIRTGL_P20  (1 << 20)
#define PORTC_DIRTGL_P21  (1 << 21)
#define PORTC_DIRTGL_P22  (1 << 22)
#define PORTC_DIRTGL_P23  (1 << 23)
#define PORTC_DIRTGL_P24  (1 << 24)
#define PORTC_DIRTGL_P25  (1 << 25)
#define PORTC_DIRTGL_P26  (1 << 26)
#define PORTC_DIRTGL_P27  (1 << 27)
#define PORTC_DIRTGL_P28  (1 << 28)
#define PORTC_DIRTGL_P29  (1 << 29)
#define PORTC_DIRTGL_P30  (1 << 30)
#define PORTC_DIRTGL_P31  (1 << 31)

#define PORTC_DIRTGL_s    MMIO_REG(0x4000290c, struct __struct_PORTC_DIRTGL)
struct __struct_PORTC_DIRTGL
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTC_OUT MMIO_REG(0x40002910, uint32_t)
#define PORTC_OUT_P0  (1 << 0)
#define PORTC_OUT_P1  (1 << 1)
#define PORTC_OUT_P2  (1 << 2)
#define PORTC_OUT_P3  (1 << 3)
#define PORTC_OUT_P4  (1 << 4)
#define PORTC_OUT_P5  (1 << 5)
#define PORTC_OUT_P6  (1 << 6)
#define PORTC_OUT_P7  (1 << 7)
#define PORTC_OUT_P8  (1 << 8)
#define PORTC_OUT_P9  (1 << 9)
#define PORTC_OUT_P10 (1 << 10)
#define PORTC_OUT_P11 (1 << 11)
#define PORTC_OUT_P12 (1 << 12)
#define PORTC_OUT_P13 (1 << 13)
#define PORTC_OUT_P14 (1 << 14)
#define PORTC_OUT_P15 (1 << 15)
#define PORTC_OUT_P16 (1 << 16)
#define PORTC_OUT_P17 (1 << 17)
#define PORTC_OUT_P18 (1 << 18)
#define PORTC_OUT_P19 (1 << 19)
#define PORTC_OUT_P20 (1 << 20)
#define PORTC_OUT_P21 (1 << 21)
#define PORTC_OUT_P22 (1 << 22)
#define PORTC_OUT_P23 (1 << 23)
#define PORTC_OUT_P24 (1 << 24)
#define PORTC_OUT_P25 (1 << 25)
#define PORTC_OUT_P26 (1 << 26)
#define PORTC_OUT_P27 (1 << 27)
#define PORTC_OUT_P28 (1 << 28)
#define PORTC_OUT_P29 (1 << 29)
#define PORTC_OUT_P30 (1 << 30)
#define PORTC_OUT_P31 (1 << 31)

#define PORTC_OUT_s    MMIO_REG(0x40002910, struct __struct_PORTC_OUT)
struct __struct_PORTC_OUT
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTC_OUTCLR  MMIO_REG(0x40002914, uint32_t)
#define PORTC_OUTCLR_P0 (1 << 0)
#define PORTC_OUTCLR_P1 (1 << 1)
#define PORTC_OUTCLR_P2 (1 << 2)
#define PORTC_OUTCLR_P3 (1 << 3)
#define PORTC_OUTCLR_P4 (1 << 4)
#define PORTC_OUTCLR_P5 (1 << 5)
#define PORTC_OUTCLR_P6 (1 << 6)
#define PORTC_OUTCLR_P7 (1 << 7)
#define PORTC_OUTCLR_P8 (1 << 8)
#define PORTC_OUTCLR_P9 (1 << 9)
#define PORTC_OUTCLR_P10  (1 << 10)
#define PORTC_OUTCLR_P11  (1 << 11)
#define PORTC_OUTCLR_P12  (1 << 12)
#define PORTC_OUTCLR_P13  (1 << 13)
#define PORTC_OUTCLR_P14  (1 << 14)
#define PORTC_OUTCLR_P15  (1 << 15)
#define PORTC_OUTCLR_P16  (1 << 16)
#define PORTC_OUTCLR_P17  (1 << 17)
#define PORTC_OUTCLR_P18  (1 << 18)
#define PORTC_OUTCLR_P19  (1 << 19)
#define PORTC_OUTCLR_P20  (1 << 20)
#define PORTC_OUTCLR_P21  (1 << 21)
#define PORTC_OUTCLR_P22  (1 << 22)
#define PORTC_OUTCLR_P23  (1 << 23)
#define PORTC_OUTCLR_P24  (1 << 24)
#define PORTC_OUTCLR_P25  (1 << 25)
#define PORTC_OUTCLR_P26  (1 << 26)
#define PORTC_OUTCLR_P27  (1 << 27)
#define PORTC_OUTCLR_P28  (1 << 28)
#define PORTC_OUTCLR_P29  (1 << 29)
#define PORTC_OUTCLR_P30  (1 << 30)
#define PORTC_OUTCLR_P31  (1 << 31)

#define PORTC_OUTCLR_s    MMIO_REG(0x40002914, struct __struct_PORTC_OUTCLR)
struct __struct_PORTC_OUTCLR
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTC_OUTSET  MMIO_REG(0x40002918, uint32_t)
#define PORTC_OUTSET_P0 (1 << 0)
#define PORTC_OUTSET_P1 (1 << 1)
#define PORTC_OUTSET_P2 (1 << 2)
#define PORTC_OUTSET_P3 (1 << 3)
#define PORTC_OUTSET_P4 (1 << 4)
#define PORTC_OUTSET_P5 (1 << 5)
#define PORTC_OUTSET_P6 (1 << 6)
#define PORTC_OUTSET_P7 (1 << 7)
#define PORTC_OUTSET_P8 (1 << 8)
#define PORTC_OUTSET_P9 (1 << 9)
#define PORTC_OUTSET_P10  (1 << 10)
#define PORTC_OUTSET_P11  (1 << 11)
#define PORTC_OUTSET_P12  (1 << 12)
#define PORTC_OUTSET_P13  (1 << 13)
#define PORTC_OUTSET_P14  (1 << 14)
#define PORTC_OUTSET_P15  (1 << 15)
#define PORTC_OUTSET_P16  (1 << 16)
#define PORTC_OUTSET_P17  (1 << 17)
#define PORTC_OUTSET_P18  (1 << 18)
#define PORTC_OUTSET_P19  (1 << 19)
#define PORTC_OUTSET_P20  (1 << 20)
#define PORTC_OUTSET_P21  (1 << 21)
#define PORTC_OUTSET_P22  (1 << 22)
#define PORTC_OUTSET_P23  (1 << 23)
#define PORTC_OUTSET_P24  (1 << 24)
#define PORTC_OUTSET_P25  (1 << 25)
#define PORTC_OUTSET_P26  (1 << 26)
#define PORTC_OUTSET_P27  (1 << 27)
#define PORTC_OUTSET_P28  (1 << 28)
#define PORTC_OUTSET_P29  (1 << 29)
#define PORTC_OUTSET_P30  (1 << 30)
#define PORTC_OUTSET_P31  (1 << 31)

#define PORTC_OUTSET_s    MMIO_REG(0x40002918, struct __struct_PORTC_OUTSET)
struct __struct_PORTC_OUTSET
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTC_OUTTGL  MMIO_REG(0x4000291c, uint32_t)
#define PORTC_OUTTGL_P0 (1 << 0)
#define PORTC_OUTTGL_P1 (1 << 1)
#define PORTC_OUTTGL_P2 (1 << 2)
#define PORTC_OUTTGL_P3 (1 << 3)
#define PORTC_OUTTGL_P4 (1 << 4)
#define PORTC_OUTTGL_P5 (1 << 5)
#define PORTC_OUTTGL_P6 (1 << 6)
#define PORTC_OUTTGL_P7 (1 << 7)
#define PORTC_OUTTGL_P8 (1 << 8)
#define PORTC_OUTTGL_P9 (1 << 9)
#define PORTC_OUTTGL_P10  (1 << 10)
#define PORTC_OUTTGL_P11  (1 << 11)
#define PORTC_OUTTGL_P12  (1 << 12)
#define PORTC_OUTTGL_P13  (1 << 13)
#define PORTC_OUTTGL_P14  (1 << 14)
#define PORTC_OUTTGL_P15  (1 << 15)
#define PORTC_OUTTGL_P16  (1 << 16)
#define PORTC_OUTTGL_P17  (1 << 17)
#define PORTC_OUTTGL_P18  (1 << 18)
#define PORTC_OUTTGL_P19  (1 << 19)
#define PORTC_OUTTGL_P20  (1 << 20)
#define PORTC_OUTTGL_P21  (1 << 21)
#define PORTC_OUTTGL_P22  (1 << 22)
#define PORTC_OUTTGL_P23  (1 << 23)
#define PORTC_OUTTGL_P24  (1 << 24)
#define PORTC_OUTTGL_P25  (1 << 25)
#define PORTC_OUTTGL_P26  (1 << 26)
#define PORTC_OUTTGL_P27  (1 << 27)
#define PORTC_OUTTGL_P28  (1 << 28)
#define PORTC_OUTTGL_P29  (1 << 29)
#define PORTC_OUTTGL_P30  (1 << 30)
#define PORTC_OUTTGL_P31  (1 << 31)

#define PORTC_OUTTGL_s    MMIO_REG(0x4000291c, struct __struct_PORTC_OUTTGL)
struct __struct_PORTC_OUTTGL
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTC_IN  MMIO_REG(0x40002920, uint32_t)
#define PORTC_IN_P0 (1 << 0)
#define PORTC_IN_P1 (1 << 1)
#define PORTC_IN_P2 (1 << 2)
#define PORTC_IN_P3 (1 << 3)
#define PORTC_IN_P4 (1 << 4)
#define PORTC_IN_P5 (1 << 5)
#define PORTC_IN_P6 (1 << 6)
#define PORTC_IN_P7 (1 << 7)
#define PORTC_IN_P8 (1 << 8)
#define PORTC_IN_P9 (1 << 9)
#define PORTC_IN_P10  (1 << 10)
#define PORTC_IN_P11  (1 << 11)
#define PORTC_IN_P12  (1 << 12)
#define PORTC_IN_P13  (1 << 13)
#define PORTC_IN_P14  (1 << 14)
#define PORTC_IN_P15  (1 << 15)
#define PORTC_IN_P16  (1 << 16)
#define PORTC_IN_P17  (1 << 17)
#define PORTC_IN_P18  (1 << 18)
#define PORTC_IN_P19  (1 << 19)
#define PORTC_IN_P20  (1 << 20)
#define PORTC_IN_P21  (1 << 21)
#define PORTC_IN_P22  (1 << 22)
#define PORTC_IN_P23  (1 << 23)
#define PORTC_IN_P24  (1 << 24)
#define PORTC_IN_P25  (1 << 25)
#define PORTC_IN_P26  (1 << 26)
#define PORTC_IN_P27  (1 << 27)
#define PORTC_IN_P28  (1 << 28)
#define PORTC_IN_P29  (1 << 29)
#define PORTC_IN_P30  (1 << 30)
#define PORTC_IN_P31  (1 << 31)

#define PORTC_IN_s    MMIO_REG(0x40002920, struct __struct_PORTC_IN)
struct __struct_PORTC_IN
{
  uint32_t p0 : 1;
  uint32_t p1 : 1;
  uint32_t p2 : 1;
  uint32_t p3 : 1;
  uint32_t p4 : 1;
  uint32_t p5 : 1;
  uint32_t p6 : 1;
  uint32_t p7 : 1;
  uint32_t p8 : 1;
  uint32_t p9 : 1;
  uint32_t p10 : 1;
  uint32_t p11 : 1;
  uint32_t p12 : 1;
  uint32_t p13 : 1;
  uint32_t p14 : 1;
  uint32_t p15 : 1;
  uint32_t p16 : 1;
  uint32_t p17 : 1;
  uint32_t p18 : 1;
  uint32_t p19 : 1;
  uint32_t p20 : 1;
  uint32_t p21 : 1;
  uint32_t p22 : 1;
  uint32_t p23 : 1;
  uint32_t p24 : 1;
  uint32_t p25 : 1;
  uint32_t p26 : 1;
  uint32_t p27 : 1;
  uint32_t p28 : 1;
  uint32_t p29 : 1;
  uint32_t p30 : 1;
  uint32_t p31 : 1;
};

#define PORTC_CTRL  MMIO_REG(0x40002924, uint32_t)

#define PORTC_WRCONFIG  MMIO_REG(0x40002928, uint32_t)
#define PORTC_WRCONFIG_PINMASK(x) ((x) << 0)
#define PORTC_WRCONFIG_PINMASK_MSK  0x0000ffff
#define PORTC_WRCONFIG_PINMASK_VAL(x) (((x) >> 0) & 0xffff)
#define PORTC_WRCONFIG_PMUXEN (1 << 16)
#define PORTC_WRCONFIG_INEN (1 << 17)
#define PORTC_WRCONFIG_PULLEN (1 << 18)
#define PORTC_WRCONFIG_PMUX(x)  ((x) << 24)
#define PORTC_WRCONFIG_PMUX_MSK 0x0f000000
#define PORTC_WRCONFIG_PMUX_VAL(x)  (((x) >> 24) & 0xf)
#define PORTC_WRCONFIG_WRPMUX (1 << 28)
#define PORTC_WRCONFIG_WRPINCFG (1 << 30)
#define PORTC_WRCONFIG_HWSEL  (1 << 31)

#define PORTC_WRCONFIG_s    MMIO_REG(0x40002928, struct __struct_PORTC_WRCONFIG)
struct __struct_PORTC_WRCONFIG
{
  uint32_t pinmask : 16;
  uint32_t pmuxen : 1;
  uint32_t inen : 1;
  uint32_t pullen : 1;
  uint32_t  : 5;
  uint32_t pmux : 4;
  uint32_t wrpmux : 1;
  uint32_t  : 1;
  uint32_t wrpincfg : 1;
  uint32_t hwsel : 1;
};

#define PORTC_PMUX0 MMIO_REG(0x40002930, uint8_t)
#define PORTC_PMUX0_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX0_PMUXE_MSK 0x0000000f
#define PORTC_PMUX0_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX0_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX0_PMUXO_MSK 0x000000f0
#define PORTC_PMUX0_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX0_s    MMIO_REG(0x40002930, struct __struct_PORTC_PMUX0)
struct __struct_PORTC_PMUX0
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX1 MMIO_REG(0x40002931, uint8_t)
#define PORTC_PMUX1_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX1_PMUXE_MSK 0x0000000f
#define PORTC_PMUX1_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX1_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX1_PMUXO_MSK 0x000000f0
#define PORTC_PMUX1_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX1_s    MMIO_REG(0x40002931, struct __struct_PORTC_PMUX1)
struct __struct_PORTC_PMUX1
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX2 MMIO_REG(0x40002932, uint8_t)
#define PORTC_PMUX2_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX2_PMUXE_MSK 0x0000000f
#define PORTC_PMUX2_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX2_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX2_PMUXO_MSK 0x000000f0
#define PORTC_PMUX2_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX2_s    MMIO_REG(0x40002932, struct __struct_PORTC_PMUX2)
struct __struct_PORTC_PMUX2
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX3 MMIO_REG(0x40002933, uint8_t)
#define PORTC_PMUX3_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX3_PMUXE_MSK 0x0000000f
#define PORTC_PMUX3_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX3_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX3_PMUXO_MSK 0x000000f0
#define PORTC_PMUX3_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX3_s    MMIO_REG(0x40002933, struct __struct_PORTC_PMUX3)
struct __struct_PORTC_PMUX3
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX4 MMIO_REG(0x40002934, uint8_t)
#define PORTC_PMUX4_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX4_PMUXE_MSK 0x0000000f
#define PORTC_PMUX4_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX4_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX4_PMUXO_MSK 0x000000f0
#define PORTC_PMUX4_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX4_s    MMIO_REG(0x40002934, struct __struct_PORTC_PMUX4)
struct __struct_PORTC_PMUX4
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX5 MMIO_REG(0x40002935, uint8_t)
#define PORTC_PMUX5_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX5_PMUXE_MSK 0x0000000f
#define PORTC_PMUX5_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX5_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX5_PMUXO_MSK 0x000000f0
#define PORTC_PMUX5_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX5_s    MMIO_REG(0x40002935, struct __struct_PORTC_PMUX5)
struct __struct_PORTC_PMUX5
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX6 MMIO_REG(0x40002936, uint8_t)
#define PORTC_PMUX6_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX6_PMUXE_MSK 0x0000000f
#define PORTC_PMUX6_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX6_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX6_PMUXO_MSK 0x000000f0
#define PORTC_PMUX6_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX6_s    MMIO_REG(0x40002936, struct __struct_PORTC_PMUX6)
struct __struct_PORTC_PMUX6
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX7 MMIO_REG(0x40002937, uint8_t)
#define PORTC_PMUX7_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX7_PMUXE_MSK 0x0000000f
#define PORTC_PMUX7_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX7_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX7_PMUXO_MSK 0x000000f0
#define PORTC_PMUX7_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX7_s    MMIO_REG(0x40002937, struct __struct_PORTC_PMUX7)
struct __struct_PORTC_PMUX7
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX8 MMIO_REG(0x40002938, uint8_t)
#define PORTC_PMUX8_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX8_PMUXE_MSK 0x0000000f
#define PORTC_PMUX8_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX8_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX8_PMUXO_MSK 0x000000f0
#define PORTC_PMUX8_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX8_s    MMIO_REG(0x40002938, struct __struct_PORTC_PMUX8)
struct __struct_PORTC_PMUX8
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX9 MMIO_REG(0x40002939, uint8_t)
#define PORTC_PMUX9_PMUXE(x)  ((x) << 0)
#define PORTC_PMUX9_PMUXE_MSK 0x0000000f
#define PORTC_PMUX9_PMUXE_VAL(x)  (((x) >> 0) & 0xf)
#define PORTC_PMUX9_PMUXO(x)  ((x) << 4)
#define PORTC_PMUX9_PMUXO_MSK 0x000000f0
#define PORTC_PMUX9_PMUXO_VAL(x)  (((x) >> 4) & 0xf)

#define PORTC_PMUX9_s    MMIO_REG(0x40002939, struct __struct_PORTC_PMUX9)
struct __struct_PORTC_PMUX9
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX10  MMIO_REG(0x4000293a, uint8_t)
#define PORTC_PMUX10_PMUXE(x) ((x) << 0)
#define PORTC_PMUX10_PMUXE_MSK  0x0000000f
#define PORTC_PMUX10_PMUXE_VAL(x) (((x) >> 0) & 0xf)
#define PORTC_PMUX10_PMUXO(x) ((x) << 4)
#define PORTC_PMUX10_PMUXO_MSK  0x000000f0
#define PORTC_PMUX10_PMUXO_VAL(x) (((x) >> 4) & 0xf)

#define PORTC_PMUX10_s    MMIO_REG(0x4000293a, struct __struct_PORTC_PMUX10)
struct __struct_PORTC_PMUX10
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX11  MMIO_REG(0x4000293b, uint8_t)
#define PORTC_PMUX11_PMUXE(x) ((x) << 0)
#define PORTC_PMUX11_PMUXE_MSK  0x0000000f
#define PORTC_PMUX11_PMUXE_VAL(x) (((x) >> 0) & 0xf)
#define PORTC_PMUX11_PMUXO(x) ((x) << 4)
#define PORTC_PMUX11_PMUXO_MSK  0x000000f0
#define PORTC_PMUX11_PMUXO_VAL(x) (((x) >> 4) & 0xf)

#define PORTC_PMUX11_s    MMIO_REG(0x4000293b, struct __struct_PORTC_PMUX11)
struct __struct_PORTC_PMUX11
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX12  MMIO_REG(0x4000293c, uint8_t)
#define PORTC_PMUX12_PMUXE(x) ((x) << 0)
#define PORTC_PMUX12_PMUXE_MSK  0x0000000f
#define PORTC_PMUX12_PMUXE_VAL(x) (((x) >> 0) & 0xf)
#define PORTC_PMUX12_PMUXO(x) ((x) << 4)
#define PORTC_PMUX12_PMUXO_MSK  0x000000f0
#define PORTC_PMUX12_PMUXO_VAL(x) (((x) >> 4) & 0xf)

#define PORTC_PMUX12_s    MMIO_REG(0x4000293c, struct __struct_PORTC_PMUX12)
struct __struct_PORTC_PMUX12
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX13  MMIO_REG(0x4000293d, uint8_t)
#define PORTC_PMUX13_PMUXE(x) ((x) << 0)
#define PORTC_PMUX13_PMUXE_MSK  0x0000000f
#define PORTC_PMUX13_PMUXE_VAL(x) (((x) >> 0) & 0xf)
#define PORTC_PMUX13_PMUXO(x) ((x) << 4)
#define PORTC_PMUX13_PMUXO_MSK  0x000000f0
#define PORTC_PMUX13_PMUXO_VAL(x) (((x) >> 4) & 0xf)

#define PORTC_PMUX13_s    MMIO_REG(0x4000293d, struct __struct_PORTC_PMUX13)
struct __struct_PORTC_PMUX13
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX14  MMIO_REG(0x4000293e, uint8_t)
#define PORTC_PMUX14_PMUXE(x) ((x) << 0)
#define PORTC_PMUX14_PMUXE_MSK  0x0000000f
#define PORTC_PMUX14_PMUXE_VAL(x) (((x) >> 0) & 0xf)
#define PORTC_PMUX14_PMUXO(x) ((x) << 4)
#define PORTC_PMUX14_PMUXO_MSK  0x000000f0
#define PORTC_PMUX14_PMUXO_VAL(x) (((x) >> 4) & 0xf)

#define PORTC_PMUX14_s    MMIO_REG(0x4000293e, struct __struct_PORTC_PMUX14)
struct __struct_PORTC_PMUX14
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PMUX15  MMIO_REG(0x4000293f, uint8_t)
#define PORTC_PMUX15_PMUXE(x) ((x) << 0)
#define PORTC_PMUX15_PMUXE_MSK  0x0000000f
#define PORTC_PMUX15_PMUXE_VAL(x) (((x) >> 0) & 0xf)
#define PORTC_PMUX15_PMUXO(x) ((x) << 4)
#define PORTC_PMUX15_PMUXO_MSK  0x000000f0
#define PORTC_PMUX15_PMUXO_VAL(x) (((x) >> 4) & 0xf)

#define PORTC_PMUX15_s    MMIO_REG(0x4000293f, struct __struct_PORTC_PMUX15)
struct __struct_PORTC_PMUX15
{
  uint8_t pmuxe : 4;
  uint8_t pmuxo : 4;
};

#define PORTC_PINCFG0 MMIO_REG(0x40002940, uint8_t)
#define PORTC_PINCFG0_PMUXEN  (1 << 0)
#define PORTC_PINCFG0_INEN  (1 << 1)
#define PORTC_PINCFG0_PULLEN  (1 << 2)

#define PORTC_PINCFG0_s    MMIO_REG(0x40002940, struct __struct_PORTC_PINCFG0)
struct __struct_PORTC_PINCFG0
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG1 MMIO_REG(0x40002941, uint8_t)
#define PORTC_PINCFG1_PMUXEN  (1 << 0)
#define PORTC_PINCFG1_INEN  (1 << 1)
#define PORTC_PINCFG1_PULLEN  (1 << 2)

#define PORTC_PINCFG1_s    MMIO_REG(0x40002941, struct __struct_PORTC_PINCFG1)
struct __struct_PORTC_PINCFG1
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG2 MMIO_REG(0x40002942, uint8_t)
#define PORTC_PINCFG2_PMUXEN  (1 << 0)
#define PORTC_PINCFG2_INEN  (1 << 1)
#define PORTC_PINCFG2_PULLEN  (1 << 2)

#define PORTC_PINCFG2_s    MMIO_REG(0x40002942, struct __struct_PORTC_PINCFG2)
struct __struct_PORTC_PINCFG2
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG3 MMIO_REG(0x40002943, uint8_t)
#define PORTC_PINCFG3_PMUXEN  (1 << 0)
#define PORTC_PINCFG3_INEN  (1 << 1)
#define PORTC_PINCFG3_PULLEN  (1 << 2)

#define PORTC_PINCFG3_s    MMIO_REG(0x40002943, struct __struct_PORTC_PINCFG3)
struct __struct_PORTC_PINCFG3
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG4 MMIO_REG(0x40002944, uint8_t)
#define PORTC_PINCFG4_PMUXEN  (1 << 0)
#define PORTC_PINCFG4_INEN  (1 << 1)
#define PORTC_PINCFG4_PULLEN  (1 << 2)

#define PORTC_PINCFG4_s    MMIO_REG(0x40002944, struct __struct_PORTC_PINCFG4)
struct __struct_PORTC_PINCFG4
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG5 MMIO_REG(0x40002945, uint8_t)
#define PORTC_PINCFG5_PMUXEN  (1 << 0)
#define PORTC_PINCFG5_INEN  (1 << 1)
#define PORTC_PINCFG5_PULLEN  (1 << 2)

#define PORTC_PINCFG5_s    MMIO_REG(0x40002945, struct __struct_PORTC_PINCFG5)
struct __struct_PORTC_PINCFG5
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG6 MMIO_REG(0x40002946, uint8_t)
#define PORTC_PINCFG6_PMUXEN  (1 << 0)
#define PORTC_PINCFG6_INEN  (1 << 1)
#define PORTC_PINCFG6_PULLEN  (1 << 2)

#define PORTC_PINCFG6_s    MMIO_REG(0x40002946, struct __struct_PORTC_PINCFG6)
struct __struct_PORTC_PINCFG6
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG7 MMIO_REG(0x40002947, uint8_t)
#define PORTC_PINCFG7_PMUXEN  (1 << 0)
#define PORTC_PINCFG7_INEN  (1 << 1)
#define PORTC_PINCFG7_PULLEN  (1 << 2)

#define PORTC_PINCFG7_s    MMIO_REG(0x40002947, struct __struct_PORTC_PINCFG7)
struct __struct_PORTC_PINCFG7
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG8 MMIO_REG(0x40002948, uint8_t)
#define PORTC_PINCFG8_PMUXEN  (1 << 0)
#define PORTC_PINCFG8_INEN  (1 << 1)
#define PORTC_PINCFG8_PULLEN  (1 << 2)

#define PORTC_PINCFG8_s    MMIO_REG(0x40002948, struct __struct_PORTC_PINCFG8)
struct __struct_PORTC_PINCFG8
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG9 MMIO_REG(0x40002949, uint8_t)
#define PORTC_PINCFG9_PMUXEN  (1 << 0)
#define PORTC_PINCFG9_INEN  (1 << 1)
#define PORTC_PINCFG9_PULLEN  (1 << 2)

#define PORTC_PINCFG9_s    MMIO_REG(0x40002949, struct __struct_PORTC_PINCFG9)
struct __struct_PORTC_PINCFG9
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG10  MMIO_REG(0x4000294a, uint8_t)
#define PORTC_PINCFG10_PMUXEN (1 << 0)
#define PORTC_PINCFG10_INEN (1 << 1)
#define PORTC_PINCFG10_PULLEN (1 << 2)

#define PORTC_PINCFG10_s    MMIO_REG(0x4000294a, struct __struct_PORTC_PINCFG10)
struct __struct_PORTC_PINCFG10
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG11  MMIO_REG(0x4000294b, uint8_t)
#define PORTC_PINCFG11_PMUXEN (1 << 0)
#define PORTC_PINCFG11_INEN (1 << 1)
#define PORTC_PINCFG11_PULLEN (1 << 2)

#define PORTC_PINCFG11_s    MMIO_REG(0x4000294b, struct __struct_PORTC_PINCFG11)
struct __struct_PORTC_PINCFG11
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG12  MMIO_REG(0x4000294c, uint8_t)
#define PORTC_PINCFG12_PMUXEN (1 << 0)
#define PORTC_PINCFG12_INEN (1 << 1)
#define PORTC_PINCFG12_PULLEN (1 << 2)

#define PORTC_PINCFG12_s    MMIO_REG(0x4000294c, struct __struct_PORTC_PINCFG12)
struct __struct_PORTC_PINCFG12
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG13  MMIO_REG(0x4000294d, uint8_t)
#define PORTC_PINCFG13_PMUXEN (1 << 0)
#define PORTC_PINCFG13_INEN (1 << 1)
#define PORTC_PINCFG13_PULLEN (1 << 2)

#define PORTC_PINCFG13_s    MMIO_REG(0x4000294d, struct __struct_PORTC_PINCFG13)
struct __struct_PORTC_PINCFG13
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG14  MMIO_REG(0x4000294e, uint8_t)
#define PORTC_PINCFG14_PMUXEN (1 << 0)
#define PORTC_PINCFG14_INEN (1 << 1)
#define PORTC_PINCFG14_PULLEN (1 << 2)

#define PORTC_PINCFG14_s    MMIO_REG(0x4000294e, struct __struct_PORTC_PINCFG14)
struct __struct_PORTC_PINCFG14
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG15  MMIO_REG(0x4000294f, uint8_t)
#define PORTC_PINCFG15_PMUXEN (1 << 0)
#define PORTC_PINCFG15_INEN (1 << 1)
#define PORTC_PINCFG15_PULLEN (1 << 2)

#define PORTC_PINCFG15_s    MMIO_REG(0x4000294f, struct __struct_PORTC_PINCFG15)
struct __struct_PORTC_PINCFG15
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG16  MMIO_REG(0x40002950, uint8_t)
#define PORTC_PINCFG16_PMUXEN (1 << 0)
#define PORTC_PINCFG16_INEN (1 << 1)
#define PORTC_PINCFG16_PULLEN (1 << 2)

#define PORTC_PINCFG16_s    MMIO_REG(0x40002950, struct __struct_PORTC_PINCFG16)
struct __struct_PORTC_PINCFG16
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG17  MMIO_REG(0x40002951, uint8_t)
#define PORTC_PINCFG17_PMUXEN (1 << 0)
#define PORTC_PINCFG17_INEN (1 << 1)
#define PORTC_PINCFG17_PULLEN (1 << 2)

#define PORTC_PINCFG17_s    MMIO_REG(0x40002951, struct __struct_PORTC_PINCFG17)
struct __struct_PORTC_PINCFG17
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG18  MMIO_REG(0x40002952, uint8_t)
#define PORTC_PINCFG18_PMUXEN (1 << 0)
#define PORTC_PINCFG18_INEN (1 << 1)
#define PORTC_PINCFG18_PULLEN (1 << 2)

#define PORTC_PINCFG18_s    MMIO_REG(0x40002952, struct __struct_PORTC_PINCFG18)
struct __struct_PORTC_PINCFG18
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG19  MMIO_REG(0x40002953, uint8_t)
#define PORTC_PINCFG19_PMUXEN (1 << 0)
#define PORTC_PINCFG19_INEN (1 << 1)
#define PORTC_PINCFG19_PULLEN (1 << 2)

#define PORTC_PINCFG19_s    MMIO_REG(0x40002953, struct __struct_PORTC_PINCFG19)
struct __struct_PORTC_PINCFG19
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG20  MMIO_REG(0x40002954, uint8_t)
#define PORTC_PINCFG20_PMUXEN (1 << 0)
#define PORTC_PINCFG20_INEN (1 << 1)
#define PORTC_PINCFG20_PULLEN (1 << 2)

#define PORTC_PINCFG20_s    MMIO_REG(0x40002954, struct __struct_PORTC_PINCFG20)
struct __struct_PORTC_PINCFG20
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG21  MMIO_REG(0x40002955, uint8_t)
#define PORTC_PINCFG21_PMUXEN (1 << 0)
#define PORTC_PINCFG21_INEN (1 << 1)
#define PORTC_PINCFG21_PULLEN (1 << 2)

#define PORTC_PINCFG21_s    MMIO_REG(0x40002955, struct __struct_PORTC_PINCFG21)
struct __struct_PORTC_PINCFG21
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG22  MMIO_REG(0x40002956, uint8_t)
#define PORTC_PINCFG22_PMUXEN (1 << 0)
#define PORTC_PINCFG22_INEN (1 << 1)
#define PORTC_PINCFG22_PULLEN (1 << 2)

#define PORTC_PINCFG22_s    MMIO_REG(0x40002956, struct __struct_PORTC_PINCFG22)
struct __struct_PORTC_PINCFG22
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG23  MMIO_REG(0x40002957, uint8_t)
#define PORTC_PINCFG23_PMUXEN (1 << 0)
#define PORTC_PINCFG23_INEN (1 << 1)
#define PORTC_PINCFG23_PULLEN (1 << 2)

#define PORTC_PINCFG23_s    MMIO_REG(0x40002957, struct __struct_PORTC_PINCFG23)
struct __struct_PORTC_PINCFG23
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG24  MMIO_REG(0x40002958, uint8_t)
#define PORTC_PINCFG24_PMUXEN (1 << 0)
#define PORTC_PINCFG24_INEN (1 << 1)
#define PORTC_PINCFG24_PULLEN (1 << 2)

#define PORTC_PINCFG24_s    MMIO_REG(0x40002958, struct __struct_PORTC_PINCFG24)
struct __struct_PORTC_PINCFG24
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG25  MMIO_REG(0x40002959, uint8_t)
#define PORTC_PINCFG25_PMUXEN (1 << 0)
#define PORTC_PINCFG25_INEN (1 << 1)
#define PORTC_PINCFG25_PULLEN (1 << 2)

#define PORTC_PINCFG25_s    MMIO_REG(0x40002959, struct __struct_PORTC_PINCFG25)
struct __struct_PORTC_PINCFG25
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG26  MMIO_REG(0x4000295a, uint8_t)
#define PORTC_PINCFG26_PMUXEN (1 << 0)
#define PORTC_PINCFG26_INEN (1 << 1)
#define PORTC_PINCFG26_PULLEN (1 << 2)

#define PORTC_PINCFG26_s    MMIO_REG(0x4000295a, struct __struct_PORTC_PINCFG26)
struct __struct_PORTC_PINCFG26
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG27  MMIO_REG(0x4000295b, uint8_t)
#define PORTC_PINCFG27_PMUXEN (1 << 0)
#define PORTC_PINCFG27_INEN (1 << 1)
#define PORTC_PINCFG27_PULLEN (1 << 2)

#define PORTC_PINCFG27_s    MMIO_REG(0x4000295b, struct __struct_PORTC_PINCFG27)
struct __struct_PORTC_PINCFG27
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG28  MMIO_REG(0x4000295c, uint8_t)
#define PORTC_PINCFG28_PMUXEN (1 << 0)
#define PORTC_PINCFG28_INEN (1 << 1)
#define PORTC_PINCFG28_PULLEN (1 << 2)

#define PORTC_PINCFG28_s    MMIO_REG(0x4000295c, struct __struct_PORTC_PINCFG28)
struct __struct_PORTC_PINCFG28
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG29  MMIO_REG(0x4000295d, uint8_t)
#define PORTC_PINCFG29_PMUXEN (1 << 0)
#define PORTC_PINCFG29_INEN (1 << 1)
#define PORTC_PINCFG29_PULLEN (1 << 2)

#define PORTC_PINCFG29_s    MMIO_REG(0x4000295d, struct __struct_PORTC_PINCFG29)
struct __struct_PORTC_PINCFG29
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG30  MMIO_REG(0x4000295e, uint8_t)
#define PORTC_PINCFG30_PMUXEN (1 << 0)
#define PORTC_PINCFG30_INEN (1 << 1)
#define PORTC_PINCFG30_PULLEN (1 << 2)

#define PORTC_PINCFG30_s    MMIO_REG(0x4000295e, struct __struct_PORTC_PINCFG30)
struct __struct_PORTC_PINCFG30
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};

#define PORTC_PINCFG31  MMIO_REG(0x4000295f, uint8_t)
#define PORTC_PINCFG31_PMUXEN (1 << 0)
#define PORTC_PINCFG31_INEN (1 << 1)
#define PORTC_PINCFG31_PULLEN (1 << 2)

#define PORTC_PINCFG31_s    MMIO_REG(0x4000295f, struct __struct_PORTC_PINCFG31)
struct __struct_PORTC_PINCFG31
{
  uint8_t pmuxen : 1;
  uint8_t inen : 1;
  uint8_t pullen : 1;
  uint8_t  : 5;
};


// 0x42000000 SC0_SPI

#define SC0_SPI_CTRLA	MMIO_REG(0x42000000, uint32_t)
#define SC0_SPI_CTRLA_SWRST	(1 << 0)
#define SC0_SPI_CTRLA_ENABLE	(1 << 1)
#define SC0_SPI_CTRLA_MODE(x)	((x) << 2)
#define SC0_SPI_CTRLA_MODE_MSK	0x0000001c
#define SC0_SPI_CTRLA_MODE_VAL(x)	(((x) >> 2) & 0x7)
#define SC0_SPI_CTRLA_RUNSTDBY	(1 << 7)
#define SC0_SPI_CTRLA_DOPO(x) ((x) << 16)
#define SC0_SPI_CTRLA_DOPO_MSK  0x00030000
#define SC0_SPI_CTRLA_DOPO_VAL(x) (((x) >> 16) & 0x3)
#define SC0_SPI_CTRLA_DIPO(x)	((x) << 20)
#define SC0_SPI_CTRLA_DIPO_MSK	0x00300000
#define SC0_SPI_CTRLA_DIPO_VAL(x)	(((x) >> 20) & 0x3)
#define SC0_SPI_CTRLA_FORM(x)	((x) << 24)
#define SC0_SPI_CTRLA_FORM_MSK	0x0f000000
#define SC0_SPI_CTRLA_FORM_VAL(x)	(((x) >> 24) & 0xf)
#define SC0_SPI_CTRLA_CPHA	(1 << 28)
#define SC0_SPI_CTRLA_CPOL	(1 << 29)
#define SC0_SPI_CTRLA_DORD	(1 << 30)

#define SC0_SPI_CTRLA_s    MMIO_REG(0x42000000, struct __struct_SC0_SPI_CTRLA)
struct __struct_SC0_SPI_CTRLA
{
  uint32_t swrst : 1;
  uint32_t enable : 1;
  uint32_t mode : 3;
  uint32_t  : 2;
  uint32_t runstdby : 1;
  uint32_t ibon : 1;
  uint32_t  : 7;
  uint32_t dopo : 2;
  uint32_t  : 2;
  uint32_t dipo : 2;
  uint32_t  : 2;
  uint32_t form : 4;
  uint32_t cpha : 1;
  uint32_t cpol : 1;
  uint32_t dord : 1;
  uint32_t  : 1;
};

#define SC0_SPI_CTRLB	MMIO_REG(0x42000004, uint32_t)
#define SC0_SPI_CTRLB_CHSIZE(x)	((x) << 0)
#define SC0_SPI_CTRLB_CHSIZE_MSK	0x00000007
#define SC0_SPI_CTRLB_CHSIZE_VAL(x)	(((x) >> 0) & 0x7)
#define SC0_SPI_CTRLB_PLOADEN	(1 << 6)
#define SC0_SPI_CTRLB_AMODE(x)	((x) << 14)
#define SC0_SPI_CTRLB_AMODE_MSK	0x0000c000
#define SC0_SPI_CTRLB_AMODE_VAL(x)	(((x) >> 14) & 0x3)
#define SC0_SPI_CTRLB_RXEN	(1 << 17)

#define SC0_SPI_CTRLB_s    MMIO_REG(0x42000004, struct __struct_SC0_SPI_CTRLB)
struct __struct_SC0_SPI_CTRLB
{
  uint32_t chsize : 3;
  uint32_t  : 3;
  uint32_t ploaden : 1;
  uint32_t  : 2;
  uint32_t ssde : 1;
  uint32_t  : 3;
  uint32_t mssen : 1;
  uint32_t amode : 2;
  uint32_t  : 1;
  uint32_t rxen : 1;
  uint32_t  : 14;
};

#define SC0_SPI_DBGCTRL	MMIO_REG(0x42000030, uint8_t)
#define SC0_SPI_DBGCTRL_DBGSTOP	(1 << 0)

#define SC0_SPI_DBGCTRL_s    MMIO_REG(0x42000030, struct __struct_SC0_SPI_DBGCTRL)
struct __struct_SC0_SPI_DBGCTRL
{
  uint8_t dbgstop : 1;
  uint8_t  : 7;
};

#define SC0_SPI_BAUD	MMIO_REG(0x4200000C, uint8_t)
#define SC0_SPI_BAUD_BAUD(x)	((x) << 0)
#define SC0_SPI_BAUD_BAUD_MSK	0x000000ff
#define SC0_SPI_BAUD_BAUD_VAL(x)	(((x) >> 0) & 0xff)

#define SC0_SPI_BAUD_s    MMIO_REG(0x4200000C, struct __struct_SC0_SPI_BAUD)
struct __struct_SC0_SPI_BAUD
{
  uint8_t baud : 8;
};

#define SC0_SPI_INTENCLR	MMIO_REG(0x42000014, uint8_t)
#define SC0_SPI_INTENCLR_DRE	(1 << 0)
#define SC0_SPI_INTENCLR_TXC	(1 << 1)
#define SC0_SPI_INTENCLR_RXC	(1 << 2)

#define SC0_SPI_INTENCLR_s    MMIO_REG(0x42000014, struct __struct_SC0_SPI_INTENCLR)
struct __struct_SC0_SPI_INTENCLR
{
  uint8_t dre : 1;
  uint8_t txc : 1;
  uint8_t rxc : 1;
  uint8_t ssl : 1;
  uint8_t  : 3;
  uint8_t error : 1;
};

#define SC0_SPI_INTENSET	MMIO_REG(0x42000016, uint8_t)
#define SC0_SPI_INTENSET_DRE	(1 << 0)
#define SC0_SPI_INTENSET_TXC	(1 << 1)
#define SC0_SPI_INTENSET_RXC	(1 << 2)

#define SC0_SPI_INTENSET_s    MMIO_REG(0x42000016, struct __struct_SC0_SPI_INTENSET)
struct __struct_SC0_SPI_INTENSET
{
  uint8_t dre : 1;
  uint8_t txc : 1;
  uint8_t rxc : 1;
  uint8_t ssl : 1;
  uint8_t  : 3;
  uint8_t error : 1;
};

#define SC0_SPI_INTFLAG	MMIO_REG(0x42000018, uint8_t)
#define SC0_SPI_INTFLAG_DRE	(1 << 0)
#define SC0_SPI_INTFLAG_TXC	(1 << 1)
#define SC0_SPI_INTFLAG_RXC	(1 << 2)

#define SC0_SPI_INTFLAG_s    MMIO_REG(0x42000018, struct __struct_SC0_SPI_INTFLAG)
struct __struct_SC0_SPI_INTFLAG
{
  uint8_t dre : 1;
  uint8_t txc : 1;
  uint8_t rxc : 1;
  uint8_t ssl : 1;
  uint8_t  : 3;
  uint8_t error : 1;
};

#define SC0_SPI_STATUS	MMIO_REG(0x4200001A, uint16_t)
#define SC0_SPI_STATUS_BUFOVF	(1 << 2)


#define SC0_SPI_STATUS_s    MMIO_REG(0x4200001A, struct __struct_SC0_SPI_STATUS)
struct __struct_SC0_SPI_STATUS
{
  uint16_t  : 2;
  uint16_t bufovf : 1;
  uint16_t  : 13;
};

#define SC0_SPI_SYNCBUSY  MMIO_REG(0x4200001C, uint32_t)
#define SC0_SPI_SYNCBUSY_SWRST (1 << 0)
#define SC0_SPI_SYNCBUSY_ENABLE (1 << 1)
#define SC0_SPI_SYNCBUSY_CTRLB (1 << 2)

#define SC0_SPI_SYNCBUSY_s    MMIO_REG(0x4200001C, struct __struct_SC0_SPI_STATUS)
struct __struct_SC0_SPI_SYNCBUSY
{
  uint32_t  swrst: 1;
  uint32_t enable : 1;
  uint32_t ctrlb : 1;
  uint32_t  : 29;
};

#define SC0_SPI_ADDR	MMIO_REG(0x42000024, uint32_t)
#define SC0_SPI_ADDR_ADDR(x)	((x) << 0)
#define SC0_SPI_ADDR_ADDR_MSK	0x000000ff
#define SC0_SPI_ADDR_ADDR_VAL(x)	(((x) >> 0) & 0xff)
#define SC0_SPI_ADDR_ADDRMASK(x)	((x) << 16)
#define SC0_SPI_ADDR_ADDRMASK_MSK	0x00ff0000
#define SC0_SPI_ADDR_ADDRMASK_VAL(x)	(((x) >> 16) & 0xff)

#define SC0_SPI_ADDR_s    MMIO_REG(0x42000024, struct __struct_SC0_SPI_ADDR)
struct __struct_SC0_SPI_ADDR
{
  uint32_t addr : 8;
  uint32_t  : 8;
  uint32_t addrmask : 8;
  uint32_t  : 8;
};

#define SC0_SPI_DATA	MMIO_REG(0x42000028, uint16_t)
#define SC0_SPI_DATA_DATA(x)	((x) << 0)
#define SC0_SPI_DATA_DATA_MSK	0x000001ff
#define SC0_SPI_DATA_DATA_VAL(x)	(((x) >> 0) & 0x1ff)

#define SC0_SPI_DATA_s    MMIO_REG(0x42000028, struct __struct_SC0_SPI_DATA)
struct __struct_SC0_SPI_DATA
{
  uint16_t data : 9;
  uint16_t  : 7;
};

// 0x42000400 SC1_USART

#define SC1_USART_CTRLA MMIO_REG(0x42000400, uint32_t)
#define SC1_USART_CTRLA_SWRST (1 << 0)
#define SC1_USART_CTRLA_ENABLE  (1 << 1)
#define SC1_USART_CTRLA_MODE(x) ((x) << 2)
#define SC1_USART_CTRLA_MODE_MSK  0x0000001c
#define SC1_USART_CTRLA_MODE_VAL(x) (((x) >> 2) & 0x7)
#define SC1_USART_CTRLA_RUNSTDBY  (1 << 7)
#define SC1_USART_CTRLA_TXPO  (1 << 16)
#define SC1_USART_CTRLA_RXPO(x) ((x) << 20)
#define SC1_USART_CTRLA_RXPO_MSK  0x00300000
#define SC1_USART_CTRLA_RXPO_VAL(x) (((x) >> 20) & 0x3)
#define SC1_USART_CTRLA_FORM(x) ((x) << 24)
#define SC1_USART_CTRLA_FORM_MSK  0x0f000000
#define SC1_USART_CTRLA_FORM_VAL(x) (((x) >> 24) & 0xf)
#define SC1_USART_CTRLA_CMODE (1 << 28)
#define SC1_USART_CTRLA_CPOL  (1 << 29)
#define SC1_USART_CTRLA_DORD  (1 << 30)

#define SC1_USART_CTRLA_s    MMIO_REG(0x42000400, struct __struct_SC1_USART_CTRLA)
struct __struct_SC1_USART_CTRLA
{
  uint32_t swrst : 1;
  uint32_t enable : 1;
  uint32_t mode : 3;
  uint32_t  : 2;
  uint32_t runstdby : 1;
  uint32_t ibon : 1;
  uint32_t  : 4;
  uint32_t sampr : 3;
  uint32_t txpo : 2;
  uint32_t  : 2;
  uint32_t rxpo : 2;
  uint32_t sampa : 2;
  uint32_t form : 4;
  uint32_t cmode : 1;
  uint32_t cpol : 1;
  uint32_t dord : 1;
  uint32_t  : 1;
};

#define SC1_USART_CTRLB MMIO_REG(0x42000404, uint32_t)
#define SC1_USART_CTRLB_CHSIZE(x) ((x) << 0)
#define SC1_USART_CTRLB_CHSIZE_MSK  0x00000007
#define SC1_USART_CTRLB_CHSIZE_VAL(x) (((x) >> 0) & 0x7)
#define SC1_USART_CTRLB_SBMODE  (1 << 6)
#define SC1_USART_CTRLB_PMODE (1 << 13)
#define SC1_USART_CTRLB_TXEN  (1 << 16)
#define SC1_USART_CTRLB_RXEN  (1 << 17)

#define SC1_USART_CTRLB_s    MMIO_REG(0x42000404, struct __struct_SC1_USART_CTRLB)
struct __struct_SC1_USART_CTRLB
{
  uint32_t chsize : 3;
  uint32_t  : 3;
  uint32_t sbmode : 1;
  uint32_t  : 1;
  uint32_t colden : 1;
  uint32_t sfde : 1;
  uint32_t enc : 1;
  uint32_t  : 2;
  uint32_t pmode : 1;
  uint32_t  : 2;
  uint32_t txen : 1;
  uint32_t rxen : 1;
  uint32_t  : 14;
};

#define SC1_USART_DBGCTRL MMIO_REG(0x42000430, uint8_t)
#define SC1_USART_DBGCTRL_DBGSTOP (1 << 0)

#define SC1_USART_DBGCTRL_s    MMIO_REG(0x42000430, struct __struct_SC1_USART_DBGCTRL)
struct __struct_SC1_USART_DBGCTRL
{
  uint8_t dbgstop : 1;
  uint8_t  : 7;
};

#define SC1_USART_BAUD  MMIO_REG(0x4200040c, uint16_t)
#define SC1_USART_BAUD_BAUD(x)  ((x) << 0)
#define SC1_USART_BAUD_BAUD_MSK 0x0000ffff
#define SC1_USART_BAUD_BAUD_VAL(x)  (((x) >> 0) & 0xffff)

#define SC1_USART_BAUD_s    MMIO_REG(0x4200040c, struct __struct_SC1_USART_BAUD)
struct __struct_SC1_USART_BAUD
{
  uint16_t baud : 16;
};

#define SC1_USART_INTENCLR  MMIO_REG(0x42000414, uint8_t)
#define SC1_USART_INTENCLR_DRE  (1 << 0)
#define SC1_USART_INTENCLR_TXC  (1 << 1)
#define SC1_USART_INTENCLR_RXC  (1 << 2)

#define SC1_USART_INTENCLR_s    MMIO_REG(0x42000414, struct __struct_SC1_USART_INTENCLR)
struct __struct_SC1_USART_INTENCLR
{
  uint8_t dre : 1;
  uint8_t txc : 1;
  uint8_t rxc : 1;
  uint8_t rxs : 1;
  uint8_t ctsic : 1;
  uint8_t rxbrk : 1;
  uint8_t  : 1;
  uint8_t error : 1;
};

#define SC1_USART_INTENSET  MMIO_REG(0x42000416, uint8_t)
#define SC1_USART_INTENSET_DRE  (1 << 0)
#define SC1_USART_INTENSET_TXC  (1 << 1)
#define SC1_USART_INTENSET_RXC  (1 << 2)

#define SC1_USART_INTENSET_s    MMIO_REG(0x42000416, struct __struct_SC1_USART_INTENSET)
struct __struct_SC1_USART_INTENSET
{
  uint8_t dre : 1;
  uint8_t txc : 1;
  uint8_t rxc : 1;
  uint8_t rxs : 1;
  uint8_t ctsic : 1;
  uint8_t rxbrk : 1;
  uint8_t  : 1;
  uint8_t error : 1;
};

#define SC1_USART_INTFLAG MMIO_REG(0x42000418, uint8_t)
#define SC1_USART_INTFLAG_ERROR (1 << 7)
#define SC1_USART_INTFLAG_DRE (1 << 0)
#define SC1_USART_INTFLAG_TXC (1 << 1)
#define SC1_USART_INTFLAG_RXC (1 << 2)

#define SC1_USART_INTFLAG_s    MMIO_REG(0x42000418, struct __struct_SC1_USART_INTFLAG)
struct __struct_SC1_USART_INTFLAG
{
  uint8_t dre : 1;
  uint8_t txc : 1;
  uint8_t rxc : 1;
  uint8_t rxs : 1;
  uint8_t ctsic : 1;
  uint8_t rxbrk : 1;
  uint8_t  : 1;
  uint8_t error : 1;
};

#define SC1_USART_STATUS  MMIO_REG(0x4200041a, uint16_t)
#define SC1_USART_STATUS_PERR (1 << 0)
#define SC1_USART_STATUS_FERR (1 << 1)
#define SC1_USART_STATUS_BUFOVF (1 << 2)
#define SC1_USART_STATUS_SYNCBUSY (1 << 15)

#define SC1_USART_STATUS_s    MMIO_REG(0x4200041a, struct __struct_SC1_USART_STATUS)
struct __struct_SC1_USART_STATUS
{
  uint16_t perr : 1;
  uint16_t ferr : 1;
  uint16_t bufovf : 1;
  uint16_t cts : 1;
  uint16_t isf : 1;
  uint16_t coll : 1;
  uint16_t  : 10;
};

#define SC1_USART_SYNCBUSY_SWRST (1 << 0)
#define SC1_USART_SYNCBUSY_ENABLE (1 << 1)
#define SC1_USART_SYNCBUSY_CTRLB (1 << 2)

#define SC1_USART_SYNCBUSY_s    MMIO_REG(0x4200041c, struct __struct_SC1_USART_SYNCBUSY)
struct __struct_SC1_USART_SYNCBUSY
{
  uint32_t swrst : 1;
  uint32_t enable : 1;
  uint32_t ctrlb : 1;
  uint32_t  : 29;
};


#define SC1_USART_DATA  MMIO_REG(0x42000428, uint16_t)
#define SC1_USART_DATA_DATA(x)  ((x) << 0)
#define SC1_USART_DATA_DATA_MSK 0x000001ff
#define SC1_USART_DATA_DATA_VAL(x)  (((x) >> 0) & 0x1ff)

#define SC1_USART_DATA_s    MMIO_REG(0x42000428, struct __struct_SC1_USART_DATA)
struct __struct_SC1_USART_DATA
{
  uint16_t data : 9;
  uint16_t  : 7;
};

// 0x42001400 SC3_SPI

#define SC3_SPI_CTRLA	MMIO_REG(0x42001400, uint32_t)
#define SC3_SPI_CTRLA_SWRST	(1 << 0)
#define SC3_SPI_CTRLA_ENABLE	(1 << 1)
#define SC3_SPI_CTRLA_MODE(x)	((x) << 2)
#define SC3_SPI_CTRLA_MODE_MSK	0x0000001c
#define SC3_SPI_CTRLA_MODE_VAL(x)	(((x) >> 2) & 0x7)
#define SC3_SPI_CTRLA_RUNSTDBY	(1 << 7)
#define SC3_SPI_CTRLA_DOPO(x) ((x) << 16)
#define SC3_SPI_CTRLA_DOPO_MSK  0x00030000
#define SC3_SPI_CTRLA_DOPO_VAL(x) (((x) >> 16) & 0x3)
#define SC3_SPI_CTRLA_DIPO(x)	((x) << 20)
#define SC3_SPI_CTRLA_DIPO_MSK	0x00300000
#define SC3_SPI_CTRLA_DIPO_VAL(x)	(((x) >> 20) & 0x3)
#define SC3_SPI_CTRLA_FORM(x)	((x) << 24)
#define SC3_SPI_CTRLA_FORM_MSK	0x0f000000
#define SC3_SPI_CTRLA_FORM_VAL(x)	(((x) >> 24) & 0xf)
#define SC3_SPI_CTRLA_CPHA	(1 << 28)
#define SC3_SPI_CTRLA_CPOL	(1 << 29)
#define SC3_SPI_CTRLA_DORD	(1 << 30)

#define SC3_SPI_CTRLA_s    MMIO_REG(0x42001400, struct __struct_SC3_SPI_CTRLA)
struct __struct_SC3_SPI_CTRLA
{
	uint32_t swrst : 1;
	uint32_t enable : 1;
	uint32_t mode : 3;
	uint32_t  : 2;
	uint32_t runstdby : 1;
	uint32_t ibon : 1;
	uint32_t  : 7;
	uint32_t dopo : 2;
	uint32_t  : 2;
	uint32_t dipo : 2;
	uint32_t  : 2;
	uint32_t form : 4;
	uint32_t cpha : 1;
	uint32_t cpol : 1;
	uint32_t dord : 1;
	uint32_t  : 1;
};

#define SC3_SPI_CTRLB	MMIO_REG(0x42001404, uint32_t)
#define SC3_SPI_CTRLB_CHSIZE(x)	((x) << 0)
#define SC3_SPI_CTRLB_CHSIZE_MSK	0x00000007
#define SC3_SPI_CTRLB_CHSIZE_VAL(x)	(((x) >> 0) & 0x7)
#define SC3_SPI_CTRLB_PLOADEN	(1 << 6)
#define SC3_SPI_CTRLB_AMODE(x)	((x) << 14)
#define SC3_SPI_CTRLB_AMODE_MSK	0x0000c000
#define SC3_SPI_CTRLB_AMODE_VAL(x)	(((x) >> 14) & 0x3)
#define SC3_SPI_CTRLB_RXEN	(1 << 17)

#define SC3_SPI_CTRLB_s    MMIO_REG(0x42001404, struct __struct_SC3_SPI_CTRLB)
struct __struct_SC3_SPI_CTRLB
{
	uint32_t chsize : 3;
	uint32_t  : 3;
	uint32_t ploaden : 1;
	uint32_t  : 2;
	uint32_t ssde : 1;
	uint32_t  : 3;
	uint32_t mssen : 1;
	uint32_t amode : 2;
	uint32_t  : 1;
	uint32_t rxen : 1;
	uint32_t  : 14;
};

#define SC3_SPI_DBGCTRL	MMIO_REG(0x42001430, uint8_t)
#define SC3_SPI_DBGCTRL_DBGSTOP	(1 << 0)

#define SC3_SPI_DBGCTRL_s    MMIO_REG(0x42001430, struct __struct_SC3_SPI_DBGCTRL)
struct __struct_SC3_SPI_DBGCTRL
{
	uint8_t dbgstop : 1;
	uint8_t  : 7;
};

#define SC3_SPI_BAUD	MMIO_REG(0x4200140C, uint8_t)
#define SC3_SPI_BAUD_BAUD(x)	((x) << 0)
#define SC3_SPI_BAUD_BAUD_MSK	0x000000ff
#define SC3_SPI_BAUD_BAUD_VAL(x)	(((x) >> 0) & 0xff)

#define SC3_SPI_BAUD_s    MMIO_REG(0x4200140C, struct __struct_SC3_SPI_BAUD)
struct __struct_SC3_SPI_BAUD
{
	uint8_t baud : 8;
};

#define SC3_SPI_INTENCLR	MMIO_REG(0x42001414, uint8_t)
#define SC3_SPI_INTENCLR_DRE	(1 << 0)
#define SC3_SPI_INTENCLR_TXC	(1 << 1)
#define SC3_SPI_INTENCLR_RXC	(1 << 2)

#define SC3_SPI_INTENCLR_s    MMIO_REG(0x42001414, struct __struct_SC3_SPI_INTENCLR)
struct __struct_SC3_SPI_INTENCLR
{
	uint8_t dre : 1;
	uint8_t txc : 1;
	uint8_t rxc : 1;
	uint8_t ssl : 1;
	uint8_t  : 3;
	uint8_t error : 1;
};

#define SC3_SPI_INTENSET	MMIO_REG(0x42001416, uint8_t)
#define SC3_SPI_INTENSET_DRE	(1 << 0)
#define SC3_SPI_INTENSET_TXC	(1 << 1)
#define SC3_SPI_INTENSET_RXC	(1 << 2)

#define SC3_SPI_INTENSET_s    MMIO_REG(0x42001416, struct __struct_SC3_SPI_INTENSET)
struct __struct_SC3_SPI_INTENSET
{
	uint8_t dre : 1;
	uint8_t txc : 1;
	uint8_t rxc : 1;
	uint8_t ssl : 1;
	uint8_t  : 3;
	uint8_t error : 1;
};

#define SC3_SPI_INTFLAG	MMIO_REG(0x42001418, uint8_t)
#define SC3_SPI_INTFLAG_DRE	(1 << 0)
#define SC3_SPI_INTFLAG_TXC	(1 << 1)
#define SC3_SPI_INTFLAG_RXC	(1 << 2)

#define SC3_SPI_INTFLAG_s    MMIO_REG(0x42001418, struct __struct_SC3_SPI_INTFLAG)
struct __struct_SC3_SPI_INTFLAG
{
	uint8_t dre : 1;
	uint8_t txc : 1;
	uint8_t rxc : 1;
	uint8_t ssl : 1;
	uint8_t  : 3;
	uint8_t error : 1;
};

#define SC3_SPI_STATUS	MMIO_REG(0x4200141A, uint16_t)
#define SC3_SPI_STATUS_BUFOVF	(1 << 2)


#define SC3_SPI_STATUS_s    MMIO_REG(0x4200141A, struct __struct_SC3_SPI_STATUS)
struct __struct_SC3_SPI_STATUS
{
	uint16_t  : 2;
	uint16_t bufovf : 1;
	uint16_t  : 13;
};

#define SC3_SPI_SYNCBUSY  MMIO_REG(0x4200141C, uint32_t)
#define SC3_SPI_SYNCBUSY_SWRST (1 << 0)
#define SC3_SPI_SYNCBUSY_ENABLE (1 << 1)
#define SC3_SPI_SYNCBUSY_CTRLB (1 << 2)

#define SC3_SPI_SYNCBUSY_s    MMIO_REG(0x4200141C, struct __struct_SC3_SPI_STATUS)
struct __struct_SC3_SPI_SYNCBUSY
{
	uint32_t  swrst: 1;
	uint32_t enable : 1;
	uint32_t ctrlb : 1;
	uint32_t  : 29;
};

#define SC3_SPI_ADDR	MMIO_REG(0x42001424, uint32_t)
#define SC3_SPI_ADDR_ADDR(x)	((x) << 0)
#define SC3_SPI_ADDR_ADDR_MSK	0x000000ff
#define SC3_SPI_ADDR_ADDR_VAL(x)	(((x) >> 0) & 0xff)
#define SC3_SPI_ADDR_ADDRMASK(x)	((x) << 16)
#define SC3_SPI_ADDR_ADDRMASK_MSK	0x00ff0000
#define SC3_SPI_ADDR_ADDRMASK_VAL(x)	(((x) >> 16) & 0xff)

#define SC3_SPI_ADDR_s    MMIO_REG(0x42001414, struct __struct_SC3_SPI_ADDR)
struct __struct_SC3_SPI_ADDR
{
	uint32_t addr : 8;
	uint32_t  : 8;
	uint32_t addrmask : 8;
	uint32_t  : 8;
};

#define SC3_SPI_DATA	MMIO_REG(0x42001428, uint16_t)
#define SC3_SPI_DATA_DATA(x)	((x) << 0)
#define SC3_SPI_DATA_DATA_MSK	0x000001ff
#define SC3_SPI_DATA_DATA_VAL(x)	(((x) >> 0) & 0x1ff)

#define SC3_SPI_DATA_s    MMIO_REG(0x42001428, struct __struct_SC3_SPI_DATA)
struct __struct_SC3_SPI_DATA
{
	uint16_t data : 9;
	uint16_t  : 7;
};

// 0x42001800 SC4_SPI

#define SC4_SPI_CTRLA	MMIO_REG(0x42001000, uint32_t)
#define SC4_SPI_CTRLA_SWRST	(1 << 0)
#define SC4_SPI_CTRLA_ENABLE	(1 << 1)
#define SC4_SPI_CTRLA_MODE(x)	((x) << 2)
#define SC4_SPI_CTRLA_MODE_MSK	0x0000001c
#define SC4_SPI_CTRLA_MODE_VAL(x)	(((x) >> 2) & 0x7)
#define SC4_SPI_CTRLA_RUNSTDBY	(1 << 7)
#define SC4_SPI_CTRLA_DOPO(x) ((x) << 16)
#define SC4_SPI_CTRLA_DOPO_MSK  0x00030000
#define SC4_SPI_CTRLA_DOPO_VAL(x) (((x) >> 16) & 0x3)
#define SC4_SPI_CTRLA_DIPO(x)	((x) << 20)
#define SC4_SPI_CTRLA_DIPO_MSK	0x00300000
#define SC4_SPI_CTRLA_DIPO_VAL(x)	(((x) >> 20) & 0x3)
#define SC4_SPI_CTRLA_FORM(x)	((x) << 24)
#define SC4_SPI_CTRLA_FORM_MSK	0x0f000000
#define SC4_SPI_CTRLA_FORM_VAL(x)	(((x) >> 24) & 0xf)
#define SC4_SPI_CTRLA_CPHA	(1 << 28)
#define SC4_SPI_CTRLA_CPOL	(1 << 29)
#define SC4_SPI_CTRLA_DORD	(1 << 30)

#define SC4_SPI_CTRLA_s    MMIO_REG(0x42001000, struct __struct_SC4_SPI_CTRLA)
struct __struct_SC4_SPI_CTRLA
{
  uint32_t swrst : 1;
  uint32_t enable : 1;
  uint32_t mode : 3;
  uint32_t  : 2;
  uint32_t runstdby : 1;
  uint32_t ibon : 1;
  uint32_t  : 7;
  uint32_t dopo : 2;
  uint32_t  : 2;
  uint32_t dipo : 2;
  uint32_t  : 2;
  uint32_t form : 4;
  uint32_t cpha : 1;
  uint32_t cpol : 1;
  uint32_t dord : 1;
  uint32_t  : 1;
};

#define SC4_SPI_CTRLB	MMIO_REG(0x42001004, uint32_t)
#define SC4_SPI_CTRLB_CHSIZE(x)	((x) << 0)
#define SC4_SPI_CTRLB_CHSIZE_MSK	0x00000007
#define SC4_SPI_CTRLB_CHSIZE_VAL(x)	(((x) >> 0) & 0x7)
#define SC4_SPI_CTRLB_PLOADEN	(1 << 6)
#define SC4_SPI_CTRLB_AMODE(x)	((x) << 14)
#define SC4_SPI_CTRLB_AMODE_MSK	0x0000c000
#define SC4_SPI_CTRLB_AMODE_VAL(x)	(((x) >> 14) & 0x3)
#define SC4_SPI_CTRLB_RXEN	(1 << 17)

#define SC4_SPI_CTRLB_s    MMIO_REG(0x42001004, struct __struct_SC4_SPI_CTRLB)
struct __struct_SC4_SPI_CTRLB
{
  uint32_t chsize : 3;
  uint32_t  : 3;
  uint32_t ploaden : 1;
  uint32_t  : 2;
  uint32_t ssde : 1;
  uint32_t  : 3;
  uint32_t mssen : 1;
  uint32_t amode : 2;
  uint32_t  : 1;
  uint32_t rxen : 1;
  uint32_t  : 14;
};

#define SC4_SPI_DBGCTRL	MMIO_REG(0x42001030, uint8_t)
#define SC4_SPI_DBGCTRL_DBGSTOP	(1 << 0)

#define SC4_SPI_DBGCTRL_s    MMIO_REG(0x42001030, struct __struct_SC4_SPI_DBGCTRL)
struct __struct_SC4_SPI_DBGCTRL
{
  uint8_t dbgstop : 1;
  uint8_t  : 7;
};

#define SC4_SPI_BAUD	MMIO_REG(0x4200100C, uint8_t)
#define SC4_SPI_BAUD_BAUD(x)	((x) << 0)
#define SC4_SPI_BAUD_BAUD_MSK	0x000000ff
#define SC4_SPI_BAUD_BAUD_VAL(x)	(((x) >> 0) & 0xff)

#define SC4_SPI_BAUD_s    MMIO_REG(0x4200100C, struct __struct_SC4_SPI_BAUD)
struct __struct_SC4_SPI_BAUD
{
  uint8_t baud : 8;
};

#define SC4_SPI_INTENCLR	MMIO_REG(0x42001014, uint8_t)
#define SC4_SPI_INTENCLR_DRE	(1 << 0)
#define SC4_SPI_INTENCLR_TXC	(1 << 1)
#define SC4_SPI_INTENCLR_RXC	(1 << 2)

#define SC4_SPI_INTENCLR_s    MMIO_REG(0x42001014, struct __struct_SC4_SPI_INTENCLR)
struct __struct_SC4_SPI_INTENCLR
{
  uint8_t dre : 1;
  uint8_t txc : 1;
  uint8_t rxc : 1;
  uint8_t ssl : 1;
  uint8_t  : 3;
  uint8_t error : 1;
};

#define SC4_SPI_INTENSET	MMIO_REG(0x42001016, uint8_t)
#define SC4_SPI_INTENSET_DRE	(1 << 0)
#define SC4_SPI_INTENSET_TXC	(1 << 1)
#define SC4_SPI_INTENSET_RXC	(1 << 2)

#define SC4_SPI_INTENSET_s    MMIO_REG(0x42001016, struct __struct_SC4_SPI_INTENSET)
struct __struct_SC4_SPI_INTENSET
{
  uint8_t dre : 1;
  uint8_t txc : 1;
  uint8_t rxc : 1;
  uint8_t ssl : 1;
  uint8_t  : 3;
  uint8_t error : 1;
};

#define SC4_SPI_INTFLAG	MMIO_REG(0x42001018, uint8_t)
#define SC4_SPI_INTFLAG_DRE	(1 << 0)
#define SC4_SPI_INTFLAG_TXC	(1 << 1)
#define SC4_SPI_INTFLAG_RXC	(1 << 2)

#define SC4_SPI_INTFLAG_s    MMIO_REG(0x42001018, struct __struct_SC4_SPI_INTFLAG)
struct __struct_SC4_SPI_INTFLAG
{
  uint8_t dre : 1;
  uint8_t txc : 1;
  uint8_t rxc : 1;
  uint8_t ssl : 1;
  uint8_t  : 3;
  uint8_t error : 1;
};

#define SC4_SPI_STATUS	MMIO_REG(0x4200101A, uint16_t)
#define SC4_SPI_STATUS_BUFOVF	(1 << 2)


#define SC4_SPI_STATUS_s    MMIO_REG(0x4200101A, struct __struct_SC4_SPI_STATUS)
struct __struct_SC4_SPI_STATUS
{
  uint16_t  : 2;
  uint16_t bufovf : 1;
  uint16_t  : 13;
};

#define SC4_SPI_SYNCBUSY  MMIO_REG(0x4200101C, uint32_t)
#define SC4_SPI_SYNCBUSY_SWRST (1 << 0)
#define SC4_SPI_SYNCBUSY_ENABLE (1 << 1)
#define SC4_SPI_SYNCBUSY_CTRLB (1 << 2)

#define SC4_SPI_SYNCBUSY_s    MMIO_REG(0x4200101C, struct __struct_SC4_SPI_STATUS)
struct __struct_SC4_SPI_SYNCBUSY
{
  uint32_t  swrst: 1;
  uint32_t enable : 1;
  uint32_t ctrlb : 1;
  uint32_t  : 29;
};

#define SC4_SPI_ADDR	MMIO_REG(0x42001024, uint32_t)
#define SC4_SPI_ADDR_ADDR(x)	((x) << 0)
#define SC4_SPI_ADDR_ADDR_MSK	0x000000ff
#define SC4_SPI_ADDR_ADDR_VAL(x)	(((x) >> 0) & 0xff)
#define SC4_SPI_ADDR_ADDRMASK(x)	((x) << 16)
#define SC4_SPI_ADDR_ADDRMASK_MSK	0x00ff0000
#define SC4_SPI_ADDR_ADDRMASK_VAL(x)	(((x) >> 16) & 0xff)

#define SC4_SPI_ADDR_s    MMIO_REG(0x42001014, struct __struct_SC4_SPI_ADDR)
struct __struct_SC4_SPI_ADDR
{
  uint32_t addr : 8;
  uint32_t  : 8;
  uint32_t addrmask : 8;
  uint32_t  : 8;
};

#define SC4_SPI_DATA	MMIO_REG(0x42001028, uint16_t)
#define SC4_SPI_DATA_DATA(x)	((x) << 0)
#define SC4_SPI_DATA_DATA_MSK	0x000001ff
#define SC4_SPI_DATA_DATA_VAL(x)	(((x) >> 0) & 0x1ff)

#define SC4_SPI_DATA_s    MMIO_REG(0x42001028, struct __struct_SC4_SPI_DATA)
struct __struct_SC4_SPI_DATA
{
  uint16_t data : 9;
  uint16_t  : 7;
};

// 0x42002400 TC3_16

#define TC3_16_CTRLA	MMIO_REG(0x42002400, uint16_t)
#define TC3_16_CTRLA_SWRST	(1 << 0)
#define TC3_16_CTRLA_ENABLE	(1 << 1)
#define TC3_16_CTRLA_MODE(x)	((x) << 2)
#define TC3_16_CTRLA_MODE_MSK	0x0000000c
#define TC3_16_CTRLA_MODE_VAL(x)	(((x) >> 2) & 0x3)
#define TC3_16_CTRLA_WAVEGEN(x)	((x) << 5)
#define TC3_16_CTRLA_WAVEGEN_MSK	0x00000060
#define TC3_16_CTRLA_WAVEGEN_VAL(x)	(((x) >> 5) & 0x3)
#define TC3_16_CTRLA_PRESCALER(x)	((x) << 8)
#define TC3_16_CTRLA_PRESCALER_MSK	0x00000700
#define TC3_16_CTRLA_PRESCALER_VAL(x)	(((x) >> 8) & 0x7)
#define TC3_16_CTRLA_RUNSTDBY	(1 << 11)
#define TC3_16_CTRLA_PRESCSYNC(x)	((x) << 12)
#define TC3_16_CTRLA_PRESCSYNC_MSK	0x00003000
#define TC3_16_CTRLA_PRESCSYNC_VAL(x)	(((x) >> 12) & 0x3)

#define TC3_16_CTRLA_s    MMIO_REG(0x42002400, struct __struct_TC3_16_CTRLA)
struct __struct_TC3_16_CTRLA
{
  uint16_t swrst : 1;
  uint16_t enable : 1;
  uint16_t mode : 2;
  uint16_t  : 1;
  uint16_t wavegen : 2;
  uint16_t  : 1;
  uint16_t prescaler : 3;
  uint16_t runstdby : 1;
  uint16_t prescsync : 2;
  uint16_t  : 2;
};

#define TC3_16_READREQ	MMIO_REG(0x42002402, uint16_t)
#define TC3_16_READREQ_ADDR(x)	((x) << 0)
#define TC3_16_READREQ_ADDR_MSK	0x0000001f
#define TC3_16_READREQ_ADDR_VAL(x)	(((x) >> 0) & 0x1f)
#define TC3_16_READREQ_RCONT	(1 << 14)
#define TC3_16_READREQ_RREQ	(1 << 15)

#define TC3_16_READREQ_s    MMIO_REG(0x42002402, struct __struct_TC3_16_READREQ)
struct __struct_TC3_16_READREQ
{
  uint16_t addr : 5;
  uint16_t  : 9;
  uint16_t rcont : 1;
  uint16_t rreq : 1;
};

#define TC3_16_CTRLBCLR	MMIO_REG(0x42002404, uint8_t)
#define TC3_16_CTRLBCLR_DIR	(1 << 0)
#define TC3_16_CTRLBCLR_ONESHOT	(1 << 2)
#define TC3_16_CTRLBCLR_CMD(x)	((x) << 6)
#define TC3_16_CTRLBCLR_CMD_MSK	0x000000c0
#define TC3_16_CTRLBCLR_CMD_VAL(x)	(((x) >> 6) & 0x3)

#define TC3_16_CTRLBCLR_s    MMIO_REG(0x42002404, struct __struct_TC3_16_CTRLBCLR)
struct __struct_TC3_16_CTRLBCLR
{
  uint8_t dir : 1;
  uint8_t  : 1;
  uint8_t oneshot : 1;
  uint8_t  : 3;
  uint8_t cmd : 2;
};

#define TC3_16_CTRLBSET	MMIO_REG(0x42002405, uint8_t)
#define TC3_16_CTRLBSET_DIR	(1 << 0)
#define TC3_16_CTRLBSET_ONESHOT	(1 << 2)
#define TC3_16_CTRLBSET_CMD(x)	((x) << 6)
#define TC3_16_CTRLBSET_CMD_MSK	0x000000c0
#define TC3_16_CTRLBSET_CMD_VAL(x)	(((x) >> 6) & 0x3)

#define TC3_16_CTRLBSET_s    MMIO_REG(0x42002405, struct __struct_TC3_16_CTRLBSET)
struct __struct_TC3_16_CTRLBSET
{
  uint8_t dir : 1;
  uint8_t  : 1;
  uint8_t oneshot : 1;
  uint8_t  : 3;
  uint8_t cmd : 2;
};

#define TC3_16_CTRLC	MMIO_REG(0x42002406, uint8_t)
#define TC3_16_CTRLC_INVEN0	(1 << 0)
#define TC3_16_CTRLC_INVEN1	(1 << 1)
#define TC3_16_CTRLC_CPTEN0	(1 << 4)
#define TC3_16_CTRLC_CPTEN1	(1 << 5)

#define TC3_16_CTRLC_s    MMIO_REG(0x42002406, struct __struct_TC3_16_CTRLC)
struct __struct_TC3_16_CTRLC
{
  uint8_t inven0 : 1;
  uint8_t inven1 : 1;
  uint8_t  : 2;
  uint8_t cpten0 : 1;
  uint8_t cpten1 : 1;
  uint8_t  : 2;
};

#define TC3_16_DBGCTRL	MMIO_REG(0x42002408, uint8_t)
#define TC3_16_DBGCTRL_DBGRUN	(1 << 0)

#define TC3_16_DBGCTRL_s    MMIO_REG(0x42002408, struct __struct_TC3_16_DBGCTRL)
struct __struct_TC3_16_DBGCTRL
{
  uint8_t dbgrun : 1;
  uint8_t  : 7;
};

#define TC3_16_EVCTRL	MMIO_REG(0x4200240a, uint16_t)
#define TC3_16_EVCTRL_EVACT(x)	((x) << 0)
#define TC3_16_EVCTRL_EVACT_MSK	0x00000007
#define TC3_16_EVCTRL_EVACT_VAL(x)	(((x) >> 0) & 0x7)
#define TC3_16_EVCTRL_TCINV	(1 << 4)
#define TC3_16_EVCTRL_TCEI	(1 << 5)
#define TC3_16_EVCTRL_OVFEO	(1 << 8)
#define TC3_16_EVCTRL_MCEO0	(1 << 12)
#define TC3_16_EVCTRL_MCEO1	(1 << 13)

#define TC3_16_EVCTRL_s    MMIO_REG(0x4200240a, struct __struct_TC3_16_EVCTRL)
struct __struct_TC3_16_EVCTRL
{
  uint16_t evact : 3;
  uint16_t  : 1;
  uint16_t tcinv : 1;
  uint16_t tcei : 1;
  uint16_t  : 2;
  uint16_t ovfeo : 1;
  uint16_t  : 3;
  uint16_t mceo0 : 1;
  uint16_t mceo1 : 1;
  uint16_t  : 2;
};

#define TC3_16_INTENCLR	MMIO_REG(0x4200240c, uint8_t)
#define TC3_16_INTENCLR_OVF	(1 << 0)
#define TC3_16_INTENCLR_ERR	(1 << 1)
#define TC3_16_INTENCLR_SYNCRDY	(1 << 3)
#define TC3_16_INTENCLR_MC0	(1 << 4)
#define TC3_16_INTENCLR_MC1	(1 << 5)

#define TC3_16_INTENCLR_s    MMIO_REG(0x4200240c, struct __struct_TC3_16_INTENCLR)
struct __struct_TC3_16_INTENCLR
{
  uint8_t ovf : 1;
  uint8_t err : 1;
  uint8_t  : 1;
  uint8_t syncrdy : 1;
  uint8_t mc0 : 1;
  uint8_t mc1 : 1;
  uint8_t  : 2;
};

#define TC3_16_INTENSET	MMIO_REG(0x4200240d, uint8_t)
#define TC3_16_INTENSET_OVF	(1 << 0)
#define TC3_16_INTENSET_ERR	(1 << 1)
#define TC3_16_INTENSET_SYNCRDY	(1 << 3)
#define TC3_16_INTENSET_MC0	(1 << 4)
#define TC3_16_INTENSET_MC1	(1 << 5)

#define TC3_16_INTENSET_s    MMIO_REG(0x4200240d, struct __struct_TC3_16_INTENSET)
struct __struct_TC3_16_INTENSET
{
  uint8_t ovf : 1;
  uint8_t err : 1;
  uint8_t  : 1;
  uint8_t syncrdy : 1;
  uint8_t mc0 : 1;
  uint8_t mc1 : 1;
  uint8_t  : 2;
};

#define TC3_16_INTFLAG	MMIO_REG(0x4200240e, uint8_t)
#define TC3_16_INTFLAG_OVF	(1 << 0)
#define TC3_16_INTFLAG_ERR	(1 << 1)
#define TC3_16_INTFLAG_SYNCRDY	(1 << 3)
#define TC3_16_INTFLAG_MC0	(1 << 4)
#define TC3_16_INTFLAG_MC1	(1 << 5)

#define TC3_16_INTFLAG_s    MMIO_REG(0x4200240e, struct __struct_TC3_16_INTFLAG)
struct __struct_TC3_16_INTFLAG
{
  uint8_t ovf : 1;
  uint8_t err : 1;
  uint8_t  : 1;
  uint8_t syncrdy : 1;
  uint8_t mc0 : 1;
  uint8_t mc1 : 1;
  uint8_t  : 2;
};

#define TC3_16_STATUS	MMIO_REG(0x4200240f, uint8_t)
#define TC3_16_STATUS_STOP	(1 << 3)
#define TC3_16_STATUS_SLAVE	(1 << 4)
#define TC3_16_STATUS_SYNCBUSY	(1 << 7)

#define TC3_16_STATUS_s    MMIO_REG(0x4200240f, struct __struct_TC3_16_STATUS)
struct __struct_TC3_16_STATUS
{
  uint8_t  : 3;
  uint8_t stop : 1;
  uint8_t slave : 1;
  uint8_t  : 2;
  uint8_t syncbusy : 1;
};

#define TC3_16_COUNT	MMIO_REG(0x42002410, uint16_t)
#define TC3_16_COUNT_COUNT(x)	((x) << 0)
#define TC3_16_COUNT_COUNT_MSK	0x0000ffff
#define TC3_16_COUNT_COUNT_VAL(x)	(((x) >> 0) & 0xffff)

#define TC3_16_COUNT_s    MMIO_REG(0x42002410, struct __struct_TC3_16_COUNT)
struct __struct_TC3_16_COUNT
{
  uint16_t count : 16;
};

#define TC3_16_CC0	MMIO_REG(0x42002418, uint16_t)
#define TC3_16_CC0_CC(x)	((x) << 0)
#define TC3_16_CC0_CC_MSK	0x0000ffff
#define TC3_16_CC0_CC_VAL(x)	(((x) >> 0) & 0xffff)

#define TC3_16_CC0_s    MMIO_REG(0x42002418, struct __struct_TC3_16_CC0)
struct __struct_TC3_16_CC0
{
  uint16_t cc : 16;
};

#define TC3_16_CC1	MMIO_REG(0x4200241a, uint16_t)
#define TC3_16_CC1_CC(x)	((x) << 0)
#define TC3_16_CC1_CC_MSK	0x0000ffff
#define TC3_16_CC1_CC_VAL(x)	(((x) >> 0) & 0xffff)

#define TC3_16_CC1_s    MMIO_REG(0x4200241a, struct __struct_TC3_16_CC1)
struct __struct_TC3_16_CC1
{
  uint16_t cc : 16;
};

// 0x40000C00 SYSCTRL

#define SYSCTRL_INTENCLR	MMIO_REG(0x40000C00, uint32_t)
#define SYSCTRL_INTENCLR_XOSCRDY	(1 << 0)
#define SYSCTRL_INTENCLR_XOSC32KRDY	(1 << 1)
#define SYSCTRL_INTENCLR_OSC32KRDY	(1 << 2)
#define SYSCTRL_INTENCLR_OSC8MRDY	(1 << 3)
#define SYSCTRL_INTENCLR_DFLLRDY	(1 << 4)
#define SYSCTRL_INTENCLR_DFLLOOB	(1 << 5)
#define SYSCTRL_INTENCLR_DFLLLCKF	(1 << 6)
#define SYSCTRL_INTENCLR_DFLLLCKC	(1 << 7)
#define SYSCTRL_INTENCLR_DFLLRCS	(1 << 8)
#define SYSCTRL_INTENCLR_BOD33RDY	(1 << 9)
#define SYSCTRL_INTENCLR_BOD33DET	(1 << 10)
#define SYSCTRL_INTENCLR_B33SRDY	(1 << 11)
#define SYSCTRL_INTENCLR_BOD12RDY	(1 << 12)
#define SYSCTRL_INTENCLR_BOD12DET	(1 << 13)
#define SYSCTRL_INTENCLR_B12SRDY	(1 << 14)

#define SYSCTRL_INTENCLR_s    MMIO_REG(0x40000C00, struct __struct_SYSCTRL_INTENCLR)
struct __struct_SYSCTRL_INTENCLR
{
  uint32_t xoscrdy : 1;
  uint32_t xosc32krdy : 1;
  uint32_t osc32krdy : 1;
  uint32_t osc8mrdy : 1;
  uint32_t dfllrdy : 1;
  uint32_t dflloob : 1;
  uint32_t dflllckf : 1;
  uint32_t dflllckc : 1;
  uint32_t dfllrcs : 1;
  uint32_t bod33rdy : 1;
  uint32_t bod33det : 1;
  uint32_t b33srdy : 1;
  uint32_t : 3;
  uint32_t dplllckr : 1;
  uint32_t dplllckf : 1;
  uint32_t dplllto : 1;
  uint32_t  : 14;
};

#define SYSCTRL_INTENSET	MMIO_REG(0x40000C04, uint32_t)
#define SYSCTRL_INTENSET_XOSCRDY	(1 << 0)
#define SYSCTRL_INTENSET_XOSC32KRDY	(1 << 1)
#define SYSCTRL_INTENSET_OSC32KRDY	(1 << 2)
#define SYSCTRL_INTENSET_OSC8MRDY	(1 << 3)
#define SYSCTRL_INTENSET_DFLLRDY	(1 << 4)
#define SYSCTRL_INTENSET_DFLLOOB	(1 << 5)
#define SYSCTRL_INTENSET_DFLLLCKF	(1 << 6)
#define SYSCTRL_INTENSET_DFLLLCKC	(1 << 7)
#define SYSCTRL_INTENSET_DFLLRCS	(1 << 8)
#define SYSCTRL_INTENSET_BOD33RDY	(1 << 9)
#define SYSCTRL_INTENSET_BOD33DET	(1 << 10)
#define SYSCTRL_INTENSET_B33SRDY	(1 << 11)
#define SYSCTRL_INTENSET_BOD12RDY	(1 << 12)
#define SYSCTRL_INTENSET_BOD12DET	(1 << 13)
#define SYSCTRL_INTENSET_B12SRDY	(1 << 14)

#define SYSCTRL_INTENSET_s    MMIO_REG(0x40000C04, struct __struct_SYSCTRL_INTENSET)
struct __struct_SYSCTRL_INTENSET
{
  uint32_t xoscrdy : 1;
  uint32_t xosc32krdy : 1;
  uint32_t osc32krdy : 1;
  uint32_t osc8mrdy : 1;
  uint32_t dfllrdy : 1;
  uint32_t dflloob : 1;
  uint32_t dflllckf : 1;
  uint32_t dflllckc : 1;
  uint32_t dfllrcs : 1;
  uint32_t bod33rdy : 1;
  uint32_t bod33det : 1;
  uint32_t b33srdy : 1;
  uint32_t : 3;
  uint32_t dplllckr : 1;
  uint32_t dplllckf : 1;
  uint32_t dplllto : 1;
  uint32_t  : 14;
};

#define SYSCTRL_INTFLAG	MMIO_REG(0x40000808, uint32_t)
#define SYSCTRL_INTFLAG_XOSCRDY	(1 << 0)
#define SYSCTRL_INTFLAG_XOSC32KRDY	(1 << 1)
#define SYSCTRL_INTFLAG_OSC32KRDY	(1 << 2)
#define SYSCTRL_INTFLAG_OSC8MRDY	(1 << 3)
#define SYSCTRL_INTFLAG_DFLLRDY	(1 << 4)
#define SYSCTRL_INTFLAG_DFLLOOB	(1 << 5)
#define SYSCTRL_INTFLAG_DFLLLCKF	(1 << 6)
#define SYSCTRL_INTFLAG_DFLLLCKC	(1 << 7)
#define SYSCTRL_INTFLAG_DFLLRCS	(1 << 8)
#define SYSCTRL_INTFLAG_BOD33RDY	(1 << 9)
#define SYSCTRL_INTFLAG_BOD33DET	(1 << 10)
#define SYSCTRL_INTFLAG_B33SRDY	(1 << 11)
#define SYSCTRL_INTFLAG_BOD12RDY	(1 << 12)
#define SYSCTRL_INTFLAG_BOD12DET	(1 << 13)
#define SYSCTRL_INTFLAG_B12SRDY	(1 << 14)

#define SYSCTRL_INTFLAG_s    MMIO_REG(0x40000C08, struct __struct_SYSCTRL_INTFLAG)
struct __struct_SYSCTRL_INTFLAG
{
  uint32_t xoscrdy : 1;
  uint32_t xosc32krdy : 1;
  uint32_t osc32krdy : 1;
  uint32_t osc8mrdy : 1;
  uint32_t dfllrdy : 1;
  uint32_t dflloob : 1;
  uint32_t dflllckf : 1;
  uint32_t dflllckc : 1;
  uint32_t dfllrcs : 1;
  uint32_t bod33rdy : 1;
  uint32_t bod33det : 1;
  uint32_t b33srdy : 1;
  uint32_t : 3;
  uint32_t dplllckr : 1;
  uint32_t dplllckf : 1;
  uint32_t dplllto : 1;
  uint32_t  : 14;
};

#define SYSCTRL_PCLKSR	MMIO_REG(0x40000C0c, uint32_t)
#define SYSCTRL_PCLKSR_XOSCRDY	(1 << 0)
#define SYSCTRL_PCLKSR_XOSC32KRDY	(1 << 1)
#define SYSCTRL_PCLKSR_OSC32KRDY	(1 << 2)
#define SYSCTRL_PCLKSR_OSC8MRDY	(1 << 3)
#define SYSCTRL_PCLKSR_DFLLRDY	(1 << 4)
#define SYSCTRL_PCLKSR_DFLLOOB	(1 << 5)
#define SYSCTRL_PCLKSR_DFLLLCKF	(1 << 6)
#define SYSCTRL_PCLKSR_DFLLLCKC	(1 << 7)
#define SYSCTRL_PCLKSR_DFLLRCS	(1 << 8)
#define SYSCTRL_PCLKSR_BOD33RDY	(1 << 9)
#define SYSCTRL_PCLKSR_BOD33DET	(1 << 10)
#define SYSCTRL_PCLKSR_B33SRDY	(1 << 11)
#define SYSCTRL_PCLKSR_BOD12RDY	(1 << 12)
#define SYSCTRL_PCLKSR_BOD12DET	(1 << 13)
#define SYSCTRL_PCLKSR_B12SRDY	(1 << 14)

#define SYSCTRL_PCLKSR_s    MMIO_REG(0x40000C0c, struct __struct_SYSCTRL_PCLKSR)
struct __struct_SYSCTRL_PCLKSR
{
  uint32_t xoscrdy : 1;
  uint32_t xosc32krdy : 1;
  uint32_t osc32krdy : 1;
  uint32_t osc8mrdy : 1;
  uint32_t dfllrdy : 1;
  uint32_t dflloob : 1;
  uint32_t dflllckf : 1;
  uint32_t dflllckc : 1;
  uint32_t dfllrcs : 1;
  uint32_t bod33rdy : 1;
  uint32_t bod33det : 1;
  uint32_t b33srdy : 1;
  uint32_t : 3;
  uint32_t dplllckr : 1;
  uint32_t dplllckf : 1;
  uint32_t dplllto : 1;
  uint32_t  : 14;
};

#define SYSCTRL_XOSC	MMIO_REG(0x40000C10, uint16_t)
#define SYSCTRL_XOSC_ENABLE	(1 << 1)
#define SYSCTRL_XOSC_XTALEN	(1 << 2)
#define SYSCTRL_XOSC_RUNSTDBY	(1 << 6)
#define SYSCTRL_XOSC_ONDEMAND	(1 << 7)
#define SYSCTRL_XOSC_GAIN(x)	((x) << 8)
#define SYSCTRL_XOSC_GAIN_MSK	0x00000700
#define SYSCTRL_XOSC_GAIN_VAL(x)	(((x) >> 8) & 0x7)
#define SYSCTRL_XOSC_AMPGC	(1 << 11)
#define SYSCTRL_XOSC_STARTUP(x)	((x) << 12)
#define SYSCTRL_XOSC_STARTUP_MSK	0x0000f000
#define SYSCTRL_XOSC_STARTUP_VAL(x)	(((x) >> 12) & 0xf)

#define SYSCTRL_XOSC_s    MMIO_REG(0x40000C10, struct __struct_SYSCTRL_XOSC)
struct __struct_SYSCTRL_XOSC
{
  uint16_t  : 1;
  uint16_t enable : 1;
  uint16_t xtalen : 1;
  uint16_t  : 3;
  uint16_t runstdby : 1;
  uint16_t ondemand : 1;
  uint16_t gain : 3;
  uint16_t ampgc : 1;
  uint16_t startup : 4;
};

#define SYSCTRL_XOSC32K	MMIO_REG(0x40000C14, uint16_t)
#define SYSCTRL_XOSC32K_ENABLE	(1 << 1)
#define SYSCTRL_XOSC32K_XTALEN	(1 << 2)
#define SYSCTRL_XOSC32K_EN32K	(1 << 3)
#define SYSCTRL_XOSC32K_EN1K	(1 << 4)
#define SYSCTRL_XOSC32K_AAMPEN	(1 << 5)
#define SYSCTRL_XOSC32K_RUNSTDBY	(1 << 6)
#define SYSCTRL_XOSC32K_ONDEMAND	(1 << 7)
#define SYSCTRL_XOSC32K_STARTUP(x)	((x) << 8)
#define SYSCTRL_XOSC32K_STARTUP_MSK	0x00000700
#define SYSCTRL_XOSC32K_STARTUP_VAL(x)	(((x) >> 8) & 0x7)
#define SYSCTRL_XOSC32K_WRTLOCK	(1 << 12)

#define SYSCTRL_XOSC32K_s    MMIO_REG(0x40000C14, struct __struct_SYSCTRL_XOSC32K)
struct __struct_SYSCTRL_XOSC32K
{
  uint16_t  : 1;
  uint16_t enable : 1;
  uint16_t xtalen : 1;
  uint16_t en32k : 1;
  uint16_t en1k : 1;
  uint16_t aampen : 1;
  uint16_t runstdby : 1;
  uint16_t ondemand : 1;
  uint16_t startup : 3;
  uint16_t  : 1;
  uint16_t wrtlock : 1;
  uint16_t  : 3;
};

#define SYSCTRL_OSC32K	MMIO_REG(0x40000C18, uint32_t)
#define SYSCTRL_OSC32K_ENABLE	(1 << 1)
#define SYSCTRL_OSC32K_EN32K	(1 << 2)
#define SYSCTRL_OSC32K_EN1K	(1 << 3)
#define SYSCTRL_OSC32K_RUNSTDBY	(1 << 6)
#define SYSCTRL_OSC32K_ONDEMAND	(1 << 7)
#define SYSCTRL_OSC32K_STARTUP(x)	((x) << 8)
#define SYSCTRL_OSC32K_STARTUP_MSK	0x00000700
#define SYSCTRL_OSC32K_STARTUP_VAL(x)	(((x) >> 8) & 0x7)
#define SYSCTRL_OSC32K_WRTLOCK	(1 << 12)
#define SYSCTRL_OSC32K_CALIB(x)	((x) << 16)
#define SYSCTRL_OSC32K_CALIB_MSK	0x007f0000
#define SYSCTRL_OSC32K_CALIB_VAL(x)	(((x) >> 16) & 0x7f)

#define SYSCTRL_OSC32K_s    MMIO_REG(0x40000C18, struct __struct_SYSCTRL_OSC32K)
struct __struct_SYSCTRL_OSC32K
{
  uint32_t  : 1;
  uint32_t enable : 1;
  uint32_t en32k : 1;
  uint32_t en1k : 1;
  uint32_t  : 2;
  uint32_t runstdby : 1;
  uint32_t ondemand : 1;
  uint32_t startup : 3;
  uint32_t  : 1;
  uint32_t wrtlock : 1;
  uint32_t  : 3;
  uint32_t calib : 7;
  uint32_t  : 9;
};

#define SYSCTRL_OSCULP32K	MMIO_REG(0x40000C1c, uint8_t)
#define SYSCTRL_OSCULP32K_CALIB(x)	((x) << 0)
#define SYSCTRL_OSCULP32K_CALIB_MSK	0x0000001f
#define SYSCTRL_OSCULP32K_CALIB_VAL(x)	(((x) >> 0) & 0x1f)
#define SYSCTRL_OSCULP32K_WRTLOCK	(1 << 7)

#define SYSCTRL_OSCULP32K_s    MMIO_REG(0x40000C1c, struct __struct_SYSCTRL_OSCULP32K)
struct __struct_SYSCTRL_OSCULP32K
{
  uint8_t calib : 5;
  uint8_t  : 2;
  uint8_t wrtlock : 1;
};

#define SYSCTRL_OSC8M	MMIO_REG(0x40000C20, uint32_t)
#define SYSCTRL_OSC8M_ENABLE	(1 << 1)
#define SYSCTRL_OSC8M_RUNSTDBY	(1 << 6)
#define SYSCTRL_OSC8M_ONDEMAND	(1 << 7)
#define SYSCTRL_OSC8M_PRESC(x)	((x) << 8)
#define SYSCTRL_OSC8M_PRESC_MSK	0x00000300
#define SYSCTRL_OSC8M_PRESC_VAL(x)	(((x) >> 8) & 0x3)
#define SYSCTRL_OSC8M_CALIB(x)	((x) << 16)
#define SYSCTRL_OSC8M_CALIB_MSK	0x0fff0000
#define SYSCTRL_OSC8M_CALIB_VAL(x)	(((x) >> 16) & 0xfff)
#define SYSCTRL_OSC8M_FRANGE(x)	((x) << 30)
#define SYSCTRL_OSC8M_FRANGE_MSK	0xc0000000
#define SYSCTRL_OSC8M_FRANGE_VAL(x)	(((x) >> 30) & 0x3)

#define SYSCTRL_OSC8M_s    MMIO_REG(0x40000C20, struct __struct_SYSCTRL_OSC8M)
struct __struct_SYSCTRL_OSC8M
{
  uint32_t  : 1;
  uint32_t enable : 1;
  uint32_t  : 4;
  uint32_t runstdby : 1;
  uint32_t ondemand : 1;
  uint32_t presc : 2;
  uint32_t  : 6;
  uint32_t calib : 12;
  uint32_t  : 2;
  uint32_t frange : 2;
};

#define SYSCTRL_DFLLCTRL	MMIO_REG(0x40000C24, uint16_t)
#define SYSCTRL_DFLLCTRL_ENABLE	(1 << 1)
#define SYSCTRL_DFLLCTRL_MODE	(1 << 2)
#define SYSCTRL_DFLLCTRL_STABLE	(1 << 3)
#define SYSCTRL_DFLLCTRL_LLAW	(1 << 4)
#define SYSCTRL_DFLLCTRL_RUNSTDBY	(1 << 6)
#define SYSCTRL_DFLLCTRL_ONDEMAND	(1 << 7)
#define SYSCTRL_DFLLCTRL_CCDIS	(1 << 8)
#define SYSCTRL_DFLLCTRL_QLDIS	(1 << 9)

#define SYSCTRL_DFLLCTRL_s    MMIO_REG(0x40000C24, struct __struct_SYSCTRL_DFLLCTRL)
struct __struct_SYSCTRL_DFLLCTRL
{
  uint16_t  : 1;
  uint16_t enable : 1;
  uint16_t mode : 1;
  uint16_t stable : 1;
  uint16_t llaw : 1;
  uint16_t usbcrm : 1;
  uint16_t runstdby : 1;
  uint16_t ondemand : 1;
  uint16_t ccdis : 1;
  uint16_t qldis : 1;
  uint16_t bplckc : 1;
  uint16_t waitlock : 1;
  uint16_t  : 4;
};

#define SYSCTRL_DFLLVAL	MMIO_REG(0x40000C28, uint32_t)
#define SYSCTRL_DFLLVAL_FINE(x)	((x) << 0)
#define SYSCTRL_DFLLVAL_FINE_MSK	0x000000ff
#define SYSCTRL_DFLLVAL_FINE_VAL(x)	(((x) >> 0) & 0xff)
#define SYSCTRL_DFLLVAL_COARSE(x)	((x) << 8)
#define SYSCTRL_DFLLVAL_COARSE_MSK	0x00001f00
#define SYSCTRL_DFLLVAL_COARSE_VAL(x)	(((x) >> 8) & 0x1f)
#define SYSCTRL_DFLLVAL_DIFF(x)	((x) << 16)
#define SYSCTRL_DFLLVAL_DIFF_MSK	0xffff0000
#define SYSCTRL_DFLLVAL_DIFF_VAL(x)	(((x) >> 16) & 0xffff)

#define SYSCTRL_DFLLVAL_s    MMIO_REG(0x40000C28, struct __struct_SYSCTRL_DFLLVAL)
struct __struct_SYSCTRL_DFLLVAL
{
  uint32_t fine : 10;
  uint32_t coarse : 6;
  uint32_t diff : 16;
};

#define SYSCTRL_DFLLMUL	MMIO_REG(0x40000C2c, uint32_t)
#define SYSCTRL_DFLLMUL_MUL(x)	((x) << 0)
#define SYSCTRL_DFLLMUL_MUL_MSK	0x0000ffff
#define SYSCTRL_DFLLMUL_MUL_VAL(x)	(((x) >> 0) & 0xffff)
#define SYSCTRL_DFLLMUL_FSTEP(x)	((x) << 16)
#define SYSCTRL_DFLLMUL_FSTEP_MSK	0x00ff0000
#define SYSCTRL_DFLLMUL_FSTEP_VAL(x)	(((x) >> 16) & 0xff)
#define SYSCTRL_DFLLMUL_CSTEP(x)	((x) << 24)
#define SYSCTRL_DFLLMUL_CSTEP_MSK	0x1f000000
#define SYSCTRL_DFLLMUL_CSTEP_VAL(x)	(((x) >> 24) & 0x1f)

#define SYSCTRL_DFLLMUL_s    MMIO_REG(0x40000C2c, struct __struct_SYSCTRL_DFLLMUL)
struct __struct_SYSCTRL_DFLLMUL
{
  uint32_t mul : 16;
  uint32_t fstep : 10;
  uint32_t cstep : 6;
};

#define SYSCTRL_DFLLSYNC	MMIO_REG(0x40000C30, uint8_t)
#define SYSCTRL_DFLLSYNC_READREQ	(1 << 7)

#define SYSCTRL_DFLLSYNC_s    MMIO_REG(0x40000C30, struct __struct_SYSCTRL_DFLLSYNC)
struct __struct_SYSCTRL_DFLLSYNC
{
  uint8_t  : 7;
  uint8_t readreq : 1;
};

#define SYSCTRL_BOD33	MMIO_REG(0x40000C34, uint32_t)
#define SYSCTRL_BOD33_ENABLE	(1 << 1)
#define SYSCTRL_BOD33_HYST	(1 << 2)
#define SYSCTRL_BOD33_ACTION(x)	((x) << 3)
#define SYSCTRL_BOD33_ACTION_MSK	0x00000018
#define SYSCTRL_BOD33_ACTION_VAL(x)	(((x) >> 3) & 0x3)
#define SYSCTRL_BOD33_RUNSTDBY	(1 << 6)
#define SYSCTRL_BOD33_MODE	(1 << 8)
#define SYSCTRL_BOD33_CEN	(1 << 9)
#define SYSCTRL_BOD33_PSEL(x)	((x) << 12)
#define SYSCTRL_BOD33_PSEL_MSK	0x0000f000
#define SYSCTRL_BOD33_PSEL_VAL(x)	(((x) >> 12) & 0xf)
#define SYSCTRL_BOD33_LEVEL(x)	((x) << 16)
#define SYSCTRL_BOD33_LEVEL_MSK	0x001f0000
#define SYSCTRL_BOD33_LEVEL_VAL(x)	(((x) >> 16) & 0x1f)

#define SYSCTRL_BOD33_s    MMIO_REG(0x40000C34, struct __struct_SYSCTRL_BOD33)
struct __struct_SYSCTRL_BOD33
{
  uint32_t  : 1;
  uint32_t enable : 1;
  uint32_t hyst : 1;
  uint32_t action : 2;
  uint32_t  : 1;
  uint32_t runstdby : 1;
  uint32_t  : 1;
  uint32_t mode : 1;
  uint32_t cen : 1;
  uint32_t  : 2;
  uint32_t psel : 4;
  uint32_t level : 6;
  uint32_t  : 10;
};

#define SYSCTRL_BOD12	MMIO_REG(0x40000C38, uint32_t)
#define SYSCTRL_BOD12_ENABLE	(1 << 1)
#define SYSCTRL_BOD12_HYST	(1 << 2)
#define SYSCTRL_BOD12_ACTION(x)	((x) << 3)
#define SYSCTRL_BOD12_ACTION_MSK	0x00000018
#define SYSCTRL_BOD12_ACTION_VAL(x)	(((x) >> 3) & 0x3)
#define SYSCTRL_BOD12_RUNSTDBY	(1 << 6)
#define SYSCTRL_BOD12_MODE	(1 << 8)
#define SYSCTRL_BOD12_CEN	(1 << 9)
#define SYSCTRL_BOD12_PSEL(x)	((x) << 12)
#define SYSCTRL_BOD12_PSEL_MSK	0x0000f000
#define SYSCTRL_BOD12_PSEL_VAL(x)	(((x) >> 12) & 0xf)
#define SYSCTRL_BOD12_LEVEL(x)	((x) << 16)
#define SYSCTRL_BOD12_LEVEL_MSK	0x001f0000
#define SYSCTRL_BOD12_LEVEL_VAL(x)	(((x) >> 16) & 0x1f)

#define SYSCTRL_BOD12_s    MMIO_REG(0x40000C38, struct __struct_SYSCTRL_BOD12)
struct __struct_SYSCTRL_BOD12
{
  uint32_t  : 1;
  uint32_t enable : 1;
  uint32_t hyst : 1;
  uint32_t action : 2;
  uint32_t  : 1;
  uint32_t runstdby : 1;
  uint32_t  : 1;
  uint32_t mode : 1;
  uint32_t cen : 1;
  uint32_t  : 2;
  uint32_t psel : 4;
  uint32_t level : 6;
  uint32_t  : 10;
};

#define SYSCTRL_VREG	MMIO_REG(0x40000C3c, uint16_t)
#define SYSCTRL_VREG_ENABLE	(1 << 1)
#define SYSCTRL_VREG_VDMON(x)	((x) << 4)
#define SYSCTRL_VREG_VDMON_MSK	0x00000030
#define SYSCTRL_VREG_VDMON_VAL(x)	(((x) >> 4) & 0x3)
#define SYSCTRL_VREG_RUNSTDBY	(1 << 6)
#define SYSCTRL_VREG_LEVEL(x)	((x) << 8)
#define SYSCTRL_VREG_LEVEL_MSK	0x00000700
#define SYSCTRL_VREG_LEVEL_VAL(x)	(((x) >> 8) & 0x7)
#define SYSCTRL_VREG_CALIB(x)	((x) << 12)
#define SYSCTRL_VREG_CALIB_MSK	0x00007000
#define SYSCTRL_VREG_CALIB_VAL(x)	(((x) >> 12) & 0x7)

#define SYSCTRL_VREG_s    MMIO_REG(0x40000C3c, struct __struct_SYSCTRL_VREG)
struct __struct_SYSCTRL_VREG
{
  uint16_t  : 1;
  uint16_t enable : 1;
  uint16_t  : 2;
  uint16_t vdmon : 2;
  uint16_t runstdby : 1;
  uint16_t  : 1;
  uint16_t level : 3;
  uint16_t  : 1;
  uint16_t calib : 3;
  uint16_t  : 1;
};

#define SYSCTRL_VREF	MMIO_REG(0x40000C40, uint32_t)
#define SYSCTRL_VREF_TSEN	(1 << 1)
#define SYSCTRL_VREF_BGOUTEN	(1 << 2)
#define SYSCTRL_VREF_CALIB(x)	((x) << 16)
#define SYSCTRL_VREF_CALIB_MSK	0x07ff0000
#define SYSCTRL_VREF_CALIB_VAL(x)	(((x) >> 16) & 0x7ff)

#define SYSCTRL_VREF_s    MMIO_REG(0x40000C40, struct __struct_SYSCTRL_VREF)
struct __struct_SYSCTRL_VREF
{
  uint32_t  : 1;
  uint32_t tsen : 1;
  uint32_t bgouten : 1;
  uint32_t  : 13;
  uint32_t calib : 11;
  uint32_t  : 5;
};
/*
// 0x40001800 GCLK

#define GCLK_CTRL	MMIO_REG(0x40001800, uint8_t)
#define GCLK_CTRL_SWRST	(1 << 0)

#define GCLK_CTRL_s    MMIO_REG(0x40001800, struct __struct_GCLK_CTRL)
struct __struct_GCLK_CTRL
{
  uint8_t swrst : 1;
  uint8_t  : 7;
};

#define GCLK_STATUS	MMIO_REG(0x40001801, uint8_t)
#define GCLK_STATUS_SYNCBUSY	(1 << 7)

#define GCLK_STATUS_s    MMIO_REG(0x40001801, struct __struct_GCLK_STATUS)
struct __struct_GCLK_STATUS
{
  uint8_t  : 7;
  uint8_t syncbusy : 1;
};

#define GCLK_CLKCTRL	MMIO_REG(0x40001802, uint16_t)
#define GCLK_CLKCTRL_ID(x)	((x) << 0)
#define GCLK_CLKCTRL_ID_MSK	0x0000003f
#define GCLK_CLKCTRL_ID_VAL(x)	(((x) >> 0) & 0x3f)
#define GCLK_CLKCTRL_GEN(x)	((x) << 8)
#define GCLK_CLKCTRL_GEN_MSK	0x00000f00
#define GCLK_CLKCTRL_GEN_VAL(x)	(((x) >> 8) & 0xf)
#define GCLK_CLKCTRL_CLKEN	(1 << 14)
#define GCLK_CLKCTRL_WRTLOCK	(1 << 15)

#define GCLK_CLKCTRL_s    MMIO_REG(0x40001802, struct __struct_GCLK_CLKCTRL)
struct __struct_GCLK_CLKCTRL
{
  uint16_t id : 6;
  uint16_t  : 2;
  uint16_t gen : 4;
  uint16_t  : 2;
  uint16_t clken : 1;
  uint16_t wrtlock : 1;
};

#define GCLK_GENCTRL	MMIO_REG(0x40001804, uint32_t)
#define GCLK_GENCTRL_ID(x)	((x) << 0)
#define GCLK_GENCTRL_ID_MSK	0x0000000f
#define GCLK_GENCTRL_ID_VAL(x)	(((x) >> 0) & 0xf)
#define GCLK_GENCTRL_SRC(x)	((x) << 8)
#define GCLK_GENCTRL_SRC_MSK	0x00001f00
#define GCLK_GENCTRL_SRC_VAL(x)	(((x) >> 8) & 0x1f)
#define GCLK_GENCTRL_GENEN	(1 << 16)
#define GCLK_GENCTRL_IDC	(1 << 17)
#define GCLK_GENCTRL_OOV	(1 << 18)
#define GCLK_GENCTRL_OE	(1 << 19)
#define GCLK_GENCTRL_DIVSEL	(1 << 20)
#define GCLK_GENCTRL_RUNSTDBY	(1 << 21)

#define GCLK_GENCTRL_s    MMIO_REG(0x40001804, struct __struct_GCLK_GENCTRL)
struct __struct_GCLK_GENCTRL
{
  uint32_t id : 4;
  uint32_t  : 4;
  uint32_t src : 5;
  uint32_t  : 3;
  uint32_t genen : 1;
  uint32_t idc : 1;
  uint32_t oov : 1;
  uint32_t oe : 1;
  uint32_t divsel : 1;
  uint32_t runstdby : 1;
  uint32_t  : 10;
};

#define GCLK_GENDIV	MMIO_REG(0x40001808, uint32_t)
#define GCLK_GENDIV_ID(x)	((x) << 0)
#define GCLK_GENDIV_ID_MSK	0x0000000f
#define GCLK_GENDIV_ID_VAL(x)	(((x) >> 0) & 0xf)
#define GCLK_GENDIV_DIV(x)	((x) << 8)
#define GCLK_GENDIV_DIV_MSK	0x00ffff00
#define GCLK_GENDIV_DIV_VAL(x)	(((x) >> 8) & 0xffff)

#define GCLK_GENDIV_s    MMIO_REG(0x40001808, struct __struct_GCLK_GENDIV)
struct __struct_GCLK_GENDIV
{
  uint32_t id : 4;
  uint32_t  : 4;
  uint32_t div : 16;
  uint32_t  : 8;
};
*/
// 0x40000000 PM

#define PM_CTRL	MMIO_REG(0x40000000, uint8_t)
#define PM_CTRL_CFDEN	(1 << 2)
#define PM_CTRL_BKUPCLK	(1 << 4)

#define PM_CTRL_s    MMIO_REG(0x40000000, struct __struct_PM_CTRL)
struct __struct_PM_CTRL
{
  uint8_t  : 2;
  uint8_t cfden : 1;
  uint8_t  : 1;
  uint8_t bkupclk : 1;
  uint8_t  : 3;
};

#define PM_SLEEP	MMIO_REG(0x40000001, uint8_t)
#define PM_SLEEP_IDLE(x)	((x) << 0)
#define PM_SLEEP_IDLE_MSK	0x00000003
#define PM_SLEEP_IDLE_VAL(x)	(((x) >> 0) & 0x3)

#define PM_SLEEP_s    MMIO_REG(0x40000001, struct __struct_PM_SLEEP)
struct __struct_PM_SLEEP
{
  uint8_t idle : 2;
  uint8_t  : 6;
};

#define PM_CPUSEL	MMIO_REG(0x40000008, uint8_t)
#define PM_CPUSEL_CPUDIV(x)	((x) << 0)
#define PM_CPUSEL_CPUDIV_MSK	0x00000007
#define PM_CPUSEL_CPUDIV_VAL(x)	(((x) >> 0) & 0x7)

#define PM_CPUSEL_s    MMIO_REG(0x40000008, struct __struct_PM_CPUSEL)
struct __struct_PM_CPUSEL
{
  uint8_t cpudiv : 3;
  uint8_t  : 5;
};

#define PM_APBASEL	MMIO_REG(0x40000009, uint8_t)
#define PM_APBASEL_APBADIV(x)	((x) << 0)
#define PM_APBASEL_APBADIV_MSK	0x00000007
#define PM_APBASEL_APBADIV_VAL(x)	(((x) >> 0) & 0x7)

#define PM_APBASEL_s    MMIO_REG(0x40000009, struct __struct_PM_APBASEL)
struct __struct_PM_APBASEL
{
  uint8_t apbadiv : 3;
  uint8_t  : 5;
};

#define PM_APBBSEL	MMIO_REG(0x4000000a, uint8_t)
#define PM_APBBSEL_APBBDIV(x)	((x) << 0)
#define PM_APBBSEL_APBBDIV_MSK	0x00000007
#define PM_APBBSEL_APBBDIV_VAL(x)	(((x) >> 0) & 0x7)

#define PM_APBBSEL_s    MMIO_REG(0x4000000a, struct __struct_PM_APBBSEL)
struct __struct_PM_APBBSEL
{
  uint8_t apbbdiv : 3;
  uint8_t  : 5;
};

#define PM_APBCSEL	MMIO_REG(0x4000000b, uint8_t)
#define PM_APBCSEL_APBCDIV(x)	((x) << 0)
#define PM_APBCSEL_APBCDIV_MSK	0x00000007
#define PM_APBCSEL_APBCDIV_VAL(x)	(((x) >> 0) & 0x7)

#define PM_APBCSEL_s    MMIO_REG(0x4000000b, struct __struct_PM_APBCSEL)
struct __struct_PM_APBCSEL
{
  uint8_t apbcdiv : 3;
  uint8_t  : 5;
};

#define PM_AHBMASK	MMIO_REG(0x40000014, uint32_t)
#define PM_AHBMASK_HPB0	(1 << 0)
#define PM_AHBMASK_HPB1	(1 << 1)
#define PM_AHBMASK_HPB2	(1 << 2)
#define PM_AHBMASK_DSU	(1 << 3)
#define PM_AHBMASK_NVMCTRL	(1 << 4)

#define PM_AHBMASK_s    MMIO_REG(0x40000014, struct __struct_PM_AHBMASK)
struct __struct_PM_AHBMASK
{
  uint32_t hpb0 : 1;
  uint32_t hpb1 : 1;
  uint32_t hpb2 : 1;
  uint32_t dsu : 1;
  uint32_t nvmctrl : 1;
  uint32_t dmac : 1;
  uint32_t usb : 1;
  uint32_t  : 25;
};

#define PM_APBAMASK	MMIO_REG(0x40000018, uint32_t)
#define PM_APBAMASK_PAC0	(1 << 0)
#define PM_APBAMASK_PM	(1 << 1)
#define PM_APBAMASK_SYSCTRL	(1 << 2)
#define PM_APBAMASK_GCLK	(1 << 3)
#define PM_APBAMASK_WDT	(1 << 4)
#define PM_APBAMASK_RTC	(1 << 5)
#define PM_APBAMASK_EIC	(1 << 6)

#define PM_APBAMASK_s    MMIO_REG(0x40000018, struct __struct_PM_APBAMASK)
struct __struct_PM_APBAMASK
{
  uint32_t pac0 : 1;
  uint32_t pm : 1;
  uint32_t sysctrl : 1;
  uint32_t gclk : 1;
  uint32_t wdt : 1;
  uint32_t rtc : 1;
  uint32_t eic : 1;
  uint32_t  : 25;
};

#define PM_APBBMASK	MMIO_REG(0x4000001c, uint32_t)
#define PM_APBBMASK_PAC1	(1 << 0)
#define PM_APBBMASK_DSU	(1 << 1)
#define PM_APBBMASK_NVMCTRL	(1 << 2)
#define PM_APBBMASK_PORT	(1 << 3)

#define PM_APBBMASK_s    MMIO_REG(0x4000001c, struct __struct_PM_APBBMASK)
struct __struct_PM_APBBMASK
{
  uint32_t pac1 : 1;
  uint32_t dsu : 1;
  uint32_t nvmctrl : 1;
  uint32_t port : 1;
  uint32_t dmac : 1;
  uint32_t usb : 1;
  uint32_t hmatrix : 1;
  uint32_t  : 25;
};

#define PM_APBCMASK	MMIO_REG(0x40000020, uint32_t)
#define PM_APBCMASK_PAC2	(1 << 0)
#define PM_APBCMASK_EVSYS	(1 << 1)
#define PM_APBCMASK_SERCOM0	(1 << 2)
#define PM_APBCMASK_SERCOM1	(1 << 3)
#define PM_APBCMASK_SERCOM2	(1 << 4)
#define PM_APBCMASK_SERCOM3	(1 << 5)
#define PM_APBCMASK_SERCOM4	(1 << 6)
#define PM_APBCMASK_SERCOM5	(1 << 7)
#define PM_APBCMASK_TCC0	(1 << 8)
#define PM_APBCMASK_TCC1	(1 << 9)
#define PM_APBCMASK_TC2	(1 << 10)
#define PM_APBCMASK_TC3	(1 << 11)
#define PM_APBCMASK_TC4	(1 << 12)
#define PM_APBCMASK_TC5	(1 << 13)
#define PM_APBCMASK_TC6	(1 << 14)
#define PM_APBCMASK_TC7	(1 << 15)
#define PM_APBCMASK_ADC	(1 << 16)
#define PM_APBCMASK_AC	(1 << 17)
#define PM_APBCMASK_DAC	(1 << 18)
#define PM_APBCMASK_PTC	(1 << 19)

#define PM_APBCMASK_s    MMIO_REG(0x40000020, struct __struct_PM_APBCMASK)
struct __struct_PM_APBCMASK
{
  uint32_t sercom0 : 1;
  uint32_t sercom1 : 1;
  uint32_t sercom2 : 1;
  uint32_t sercom3 : 1;
  uint32_t sercom4 : 1;
  uint32_t tcc0 : 1;
  uint32_t tcc1 : 1;
  uint32_t tcc2 : 1;
  uint32_t tc0 : 1;
  uint32_t tc1 : 1;
  uint32_t tc2 : 1;
  uint32_t dac : 1;
  uint32_t aes : 1;
  uint32_t trng : 1;
  uint32_t  : 15;
};

#define PM_INTENCLR	MMIO_REG(0x40000034, uint8_t)
#define PM_INTENCLR_CKRDY	(1 << 0)
#define PM_INTENCLR_CFD	(1 << 1)

#define PM_INTENCLR_s    MMIO_REG(0x40000034, struct __struct_PM_INTENCLR)
struct __struct_PM_INTENCLR
{
  uint8_t ckrdy : 1;
  uint8_t cfd : 1;
  uint8_t  : 6;
};

#define PM_INTENSET	MMIO_REG(0x40000035, uint8_t)
#define PM_INTENSET_CKRDY	(1 << 0)
#define PM_INTENSET_CFD	(1 << 1)

#define PM_INTENSET_s    MMIO_REG(0x40000035, struct __struct_PM_INTENSET)
struct __struct_PM_INTENSET
{
  uint8_t ckrdy : 1;
  uint8_t cfd : 1;
  uint8_t  : 6;
};

#define PM_INTFLAG	MMIO_REG(0x40000036, uint8_t)
#define PM_INTFLAG_CKRDY	(1 << 0)
#define PM_INTFLAG_CFD	(1 << 1)

#define PM_INTFLAG_s    MMIO_REG(0x40000036, struct __struct_PM_INTFLAG)
struct __struct_PM_INTFLAG
{
  uint8_t ckrdy : 1;
  uint8_t cfd : 1;
  uint8_t  : 6;
};

#define PM_RCAUSE	MMIO_REG(0x40000038, uint8_t)
#define PM_RCAUSE_POR	(1 << 0)
#define PM_RCAUSE_BOD12	(1 << 1)
#define PM_RCAUSE_BOD33	(1 << 2)
#define PM_RCAUSE_EXT	(1 << 4)
#define PM_RCAUSE_WDT	(1 << 5)
#define PM_RCAUSE_SYST	(1 << 6)

#define PM_RCAUSE_s    MMIO_REG(0x40000038, struct __struct_PM_RCAUSE)
struct __struct_PM_RCAUSE
{
  uint8_t por : 1;
  uint8_t bod12 : 1;
  uint8_t bod33 : 1;
  uint8_t  : 1;
  uint8_t ext : 1;
  uint8_t wdt : 1;
  uint8_t syst : 1;
  uint8_t  : 1;
};

/* ************************************************************************** */
/**  BASE ADDRESS DEFINITIONS FOR SAMR21G18A */
/* ************************************************************************** */
/** \defgroup SAMR21G18A_base Peripheral Base Address Definitions */
/*@{*/

#if defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)
#define AC                            (0x43001000UL) /**< \brief (AC) APB Base Address */
#define ADC                           (0x43000C00UL) /**< \brief (ADC) APB Base Address */
#define CCL                           (0x43001C00UL) /**< \brief (CCL) APB Base Address */
#define DMAC                          (0x44000400UL) /**< \brief (DMAC) APB Base Address */
#define DSU                           (0x41002000UL) /**< \brief (DSU) APB Base Address */
#define EIC                           (0x40002400UL) /**< \brief (EIC) APB Base Address */
#define EVSYS                         (0x43000000UL) /**< \brief (EVSYS) APB Base Address */
#define GCLK                          (0x40001800UL) /**< \brief (GCLK) APB Base Address */
#define MCLK                          (0x40000400UL) /**< \brief (MCLK) APB Base Address */
#define MTB                           (0x41006000UL) /**< \brief (MTB) APB Base Address */
#define NVMCTRL                       (0x41004000UL) /**< \brief (NVMCTRL) APB Base Address */
#define NVMCTRL_CAL                   (0x00800000UL) /**< \brief (NVMCTRL) CAL Base Address */
#define NVMCTRL_LOCKBIT               (0x00802000UL) /**< \brief (NVMCTRL) LOCKBIT Base Address */
#define NVMCTRL_OTP1                  (0x00806000UL) /**< \brief (NVMCTRL) OTP1 Base Address */
#define NVMCTRL_OTP2                  (0x00806008UL) /**< \brief (NVMCTRL) OTP2 Base Address */
#define NVMCTRL_OTP3                  (0x00806010UL) /**< \brief (NVMCTRL) OTP3 Base Address */
#define NVMCTRL_OTP4                  (0x00806018UL) /**< \brief (NVMCTRL) OTP4 Base Address */
#define NVMCTRL_OTP5                  (0x00806020UL) /**< \brief (NVMCTRL) OTP5 Base Address */
#define NVMCTRL_TEMP_LOG              (0x00806030UL) /**< \brief (NVMCTRL) TEMP_LOG Base Address */
#define NVMCTRL_USER                  (0x00804000UL) /**< \brief (NVMCTRL) USER Base Address */
#define OSCCTRL                       (0x40000C00UL) /**< \brief (OSCCTRL) APB Base Address */
#define OSC32KCTRL                    (0x40001000UL) /**< \brief (OSC32KCTRL) APB Base Address */
#define PAC                           (0x44000000UL) /**< \brief (PAC) APB Base Address */
#define PM                            (0x40000000UL) /**< \brief (PM) APB Base Address */
#define PORT                          (0x40002800UL) /**< \brief (PORT) APB Base Address */
#define PORT_IOBUS                    (0x60000000UL) /**< \brief (PORT) IOBUS Base Address */
#define RSTC                          (0x40000800UL) /**< \brief (RSTC) APB Base Address */
#define RTC                           (0x40002000UL) /**< \brief (RTC) APB Base Address */
#define RFCTRL                        (0x42003C00UL) /**< \brief (RFCTRL) APB Base Address */
#define SERCOM0                       (0x42000000UL) /**< \brief (SERCOM0) APB Base Address */
#define SERCOM1                       (0x42000400UL) /**< \brief (SERCOM1) APB Base Address */
#define SERCOM2                       (0x42000800UL) /**< \brief (SERCOM2) APB Base Address */
#define SERCOM3                       (0x42000C00UL) /**< \brief (SERCOM3) APB Base Address */
#define SERCOM4                       (0x42001000UL) /**< \brief (SERCOM4) APB Base Address */
#define SERCOM5                       (0x43000400UL) /**< \brief (SERCOM5) APB Base Address */
#define SUPC                          (0x40001400UL) /**< \brief (SUPC) APB Base Address */
#define TAL                           (0x40002C00UL) /**< \brief (TAL) APB Base Address */
#define TC0                           (0x42002000UL) /**< \brief (TC0) APB Base Address */
#define TC1                           (0x42002400UL) /**< \brief (TC1) APB Base Address */
#define TC4                           (0x43000800UL) /**< \brief (TC4) APB Base Address */
#define TCC0                          (0x42001400UL) /**< \brief (TCC0) APB Base Address */
#define TCC1                          (0x42001800UL) /**< \brief (TCC1) APB Base Address */
#define TCC2                          (0x42001C00UL) /**< \brief (TCC2) APB Base Address */
#define USB                           (0x41000000UL) /**< \brief (USB) APB Base Address */
#define WDT                           (0x40001C00UL) /**< \brief (WDT) APB Base Address */
#else
#define AC                ((Ac       *)0x43001000UL) /**< \brief (AC) APB Base Address */
#define AC_INST_NUM       1                          /**< \brief (AC) Number of instances */
#define AC_INSTS          { AC }                     /**< \brief (AC) Instances List */

#define ADC               ((Adc      *)0x43000C00UL) /**< \brief (ADC) APB Base Address */
#define ADC_INST_NUM      1                          /**< \brief (ADC) Number of instances */
#define ADC_INSTS         { ADC }                    /**< \brief (ADC) Instances List */

#define CCL               ((Ccl      *)0x43001C00UL) /**< \brief (CCL) APB Base Address */
#define CCL_INST_NUM      1                          /**< \brief (CCL) Number of instances */
#define CCL_INSTS         { CCL }                    /**< \brief (CCL) Instances List */

#define DMAC              ((Dmac     *)0x44000400UL) /**< \brief (DMAC) APB Base Address */
#define DMAC_INST_NUM     1                          /**< \brief (DMAC) Number of instances */
#define DMAC_INSTS        { DMAC }                   /**< \brief (DMAC) Instances List */

#define DSU               ((Dsu      *)0x41002000UL) /**< \brief (DSU) APB Base Address */
#define DSU_INST_NUM      1                          /**< \brief (DSU) Number of instances */
#define DSU_INSTS         { DSU }                    /**< \brief (DSU) Instances List */

#define EIC               ((Eic      *)0x40002400UL) /**< \brief (EIC) APB Base Address */
#define EIC_INST_NUM      1                          /**< \brief (EIC) Number of instances */
#define EIC_INSTS         { EIC }                    /**< \brief (EIC) Instances List */

#define EVSYS             ((Evsys    *)0x43000000UL) /**< \brief (EVSYS) APB Base Address */
#define EVSYS_INST_NUM    1                          /**< \brief (EVSYS) Number of instances */
#define EVSYS_INSTS       { EVSYS }                  /**< \brief (EVSYS) Instances List */

#define GCLK              ((Gclk     *)0x40001800UL) /**< \brief (GCLK) APB Base Address */
#define GCLK_INST_NUM     1                          /**< \brief (GCLK) Number of instances */
#define GCLK_INSTS        { GCLK }                   /**< \brief (GCLK) Instances List */

#define MCLK              ((Mclk     *)0x40000400UL) /**< \brief (MCLK) APB Base Address */
#define MCLK_INST_NUM     1                          /**< \brief (MCLK) Number of instances */
#define MCLK_INSTS        { MCLK }                   /**< \brief (MCLK) Instances List */

#define MTB               ((Mtb      *)0x41006000UL) /**< \brief (MTB) APB Base Address */
#define MTB_INST_NUM      1                          /**< \brief (MTB) Number of instances */
#define MTB_INSTS         { MTB }                    /**< \brief (MTB) Instances List */

#define NVMCTRL           ((Nvmctrl  *)0x41004000UL) /**< \brief (NVMCTRL) APB Base Address */
#define NVMCTRL_CAL                   (0x00800000UL) /**< \brief (NVMCTRL) CAL Base Address */
#define NVMCTRL_LOCKBIT               (0x00802000UL) /**< \brief (NVMCTRL) LOCKBIT Base Address */
#define NVMCTRL_OTP1                  (0x00806000UL) /**< \brief (NVMCTRL) OTP1 Base Address */
#define NVMCTRL_OTP2                  (0x00806008UL) /**< \brief (NVMCTRL) OTP2 Base Address */
#define NVMCTRL_OTP3                  (0x00806010UL) /**< \brief (NVMCTRL) OTP3 Base Address */
#define NVMCTRL_OTP4                  (0x00806018UL) /**< \brief (NVMCTRL) OTP4 Base Address */
#define NVMCTRL_OTP5                  (0x00806020UL) /**< \brief (NVMCTRL) OTP5 Base Address */
#define NVMCTRL_TEMP_LOG              (0x00806030UL) /**< \brief (NVMCTRL) TEMP_LOG Base Address */
#define NVMCTRL_USER                  (0x00804000UL) /**< \brief (NVMCTRL) USER Base Address */
#define NVMCTRL_INST_NUM  1                          /**< \brief (NVMCTRL) Number of instances */
#define NVMCTRL_INSTS     { NVMCTRL }                /**< \brief (NVMCTRL) Instances List */

#define OSCCTRL           ((Oscctrl  *)0x40000C00UL) /**< \brief (OSCCTRL) APB Base Address */
#define OSCCTRL_INST_NUM  1                          /**< \brief (OSCCTRL) Number of instances */
#define OSCCTRL_INSTS     { OSCCTRL }                /**< \brief (OSCCTRL) Instances List */

#define OSC32KCTRL        ((Osc32kctrl *)0x40001000UL) /**< \brief (OSC32KCTRL) APB Base Address */
#define OSC32KCTRL_INST_NUM 1                          /**< \brief (OSC32KCTRL) Number of instances */
#define OSC32KCTRL_INSTS  { OSC32KCTRL }             /**< \brief (OSC32KCTRL) Instances List */

#define PAC               ((Pac      *)0x44000000UL) /**< \brief (PAC) APB Base Address */
#define PAC_INST_NUM      1                          /**< \brief (PAC) Number of instances */
#define PAC_INSTS         { PAC }                    /**< \brief (PAC) Instances List */

#define PM                ((Pm       *)0x40000000UL) /**< \brief (PM) APB Base Address */
#define PM_INST_NUM       1                          /**< \brief (PM) Number of instances */
#define PM_INSTS          { PM }                     /**< \brief (PM) Instances List */

#define PORT              ((Port     *)0x40002800UL) /**< \brief (PORT) APB Base Address */
#define PORT_IOBUS        ((Port     *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */
#define PORT_INST_NUM     1                          /**< \brief (PORT) Number of instances */
#define PORT_INSTS        { PORT }                   /**< \brief (PORT) Instances List */

#define PTC_GCLK_ID       33
#define PTC_INST_NUM      1                          /**< \brief (PTC) Number of instances */
#define PTC_INSTS         { PTC }                    /**< \brief (PTC) Instances List */

#define RFCTRL            ((Rfctrl     *)0x42003C00U) /**< \brief (RSTC) APB Base Address */
#define RFCTRL_INST_NUM     1                          /**< \brief (RSTC) Number of instances */
#define RFCTRL_INSTS        { RFCTRL }                   /**< \brief (RSTC) Instances List */

#define RSTC              ((Rstc     *)0x40000800UL) /**< \brief (RSTC) APB Base Address */
#define RSTC_INST_NUM     1                          /**< \brief (RSTC) Number of instances */
#define RSTC_INSTS        { RSTC }                   /**< \brief (RSTC) Instances List */

#define RTC               ((Rtc      *)0x40002000UL) /**< \brief (RTC) APB Base Address */
#define RTC_INST_NUM      1                          /**< \brief (RTC) Number of instances */
#define RTC_INSTS         { RTC }                    /**< \brief (RTC) Instances List */

#define SERCOM0           ((Sercom   *)0x42000000UL) /**< \brief (SERCOM0) APB Base Address */
#define SERCOM1           ((Sercom   *)0x42000400UL) /**< \brief (SERCOM1) APB Base Address */
#define SERCOM2           ((Sercom   *)0x42000800UL) /**< \brief (SERCOM2) APB Base Address */
#define SERCOM3           ((Sercom   *)0x42000C00UL) /**< \brief (SERCOM3) APB Base Address */
#define SERCOM4           ((Sercom   *)0x42001000UL) /**< \brief (SERCOM4) APB Base Address */
#define SERCOM5           ((Sercom   *)0x43000400UL) /**< \brief (SERCOM5) APB Base Address */
#define SERCOM_INST_NUM   6                          /**< \brief (SERCOM) Number of instances */
#define SERCOM_INSTS      { SERCOM0, SERCOM1, SERCOM2, SERCOM3, SERCOM4, SERCOM5 } /**< \brief (SERCOM) Instances List */

#define SUPC              ((Supc     *)0x40001400UL) /**< \brief (SUPC) APB Base Address */
#define SUPC_INST_NUM     1                          /**< \brief (SUPC) Number of instances */
#define SUPC_INSTS        { SUPC }                   /**< \brief (SUPC) Instances List */

#define TAL               ((Tal      *)0x40002C00UL) /**< \brief (TAL) APB Base Address */
#define TAL_INST_NUM      1                          /**< \brief (TAL) Number of instances */
#define TAL_INSTS         { TAL }                    /**< \brief (TAL) Instances List */

#define TC0               ((Tc       *)0x42002000UL) /**< \brief (TC0) APB Base Address */
#define TC1               ((Tc       *)0x42002400UL) /**< \brief (TC1) APB Base Address */
#define TC4               ((Tc       *)0x43000800UL) /**< \brief (TC4) APB Base Address */
#define TC_INST_NUM       3                          /**< \brief (TC) Number of instances */
#define TC_INSTS          { TC0, TC1, TC4 }          /**< \brief (TC) Instances List */

#define TCC0              ((Tcc      *)0x42001400UL) /**< \brief (TCC0) APB Base Address */
#define TCC1              ((Tcc      *)0x42001800UL) /**< \brief (TCC1) APB Base Address */
#define TCC2              ((Tcc      *)0x42001C00UL) /**< \brief (TCC2) APB Base Address */
#define TCC_INST_NUM      3                          /**< \brief (TCC) Number of instances */
#define TCC_INSTS         { TCC0, TCC1, TCC2 }       /**< \brief (TCC) Instances List */

#define USB               ((Usb      *)0x41000000UL) /**< \brief (USB) APB Base Address */
#define USB_INST_NUM      1                          /**< \brief (USB) Number of instances */
#define USB_INSTS         { USB }                    /**< \brief (USB) Instances List */

#define WDT               ((Wdt      *)0x40001C00UL) /**< \brief (WDT) APB Base Address */
#define WDT_INST_NUM      1                          /**< \brief (WDT) Number of instances */
#define WDT_INSTS         { WDT }                    /**< \brief (WDT) Instances List */

#endif /* (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/*@}*/
#ifndef _SAMR30_TC_COMPONENT_
#define _SAMR30_TC_COMPONENT_

/* ========================================================================== */
/**  SOFTWARE API DEFINITION FOR TC */
/* ========================================================================== */
/** \addtogroup SAMR30_TC Basic Timer Counter */
/*@{*/

#define TC_U2249
#define REV_TC                      0x200

/* -------- TC_CTRLA : (TC Offset: 0x00) (R/W 32) Control A -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t SWRST:1;          /*!< bit:      0  Software Reset                     */
    uint32_t ENABLE:1;         /*!< bit:      1  Enable                             */
    uint32_t MODE:2;           /*!< bit:  2.. 3  Timer Counter Mode                 */
    uint32_t PRESCSYNC:2;      /*!< bit:  4.. 5  Prescaler and Counter Synchronization */
    uint32_t RUNSTDBY:1;       /*!< bit:      6  Run during Standby                 */
    uint32_t ONDEMAND:1;       /*!< bit:      7  Clock On Demand                    */
    uint32_t PRESCALER:3;      /*!< bit:  8..10  Prescaler                          */
    uint32_t ALOCK:1;          /*!< bit:     11  Auto Lock                          */
    uint32_t :4;               /*!< bit: 12..15  Reserved                           */
    uint32_t CAPTEN0:1;        /*!< bit:     16  Capture Channel 0 Enable           */
    uint32_t CAPTEN1:1;        /*!< bit:     17  Capture Channel 1 Enable           */
    uint32_t :2;               /*!< bit: 18..19  Reserved                           */
    uint32_t COPEN0:1;         /*!< bit:     20  Capture On Pin 0 Enable            */
    uint32_t COPEN1:1;         /*!< bit:     21  Capture On Pin 1 Enable            */
    uint32_t :10;              /*!< bit: 22..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  struct {
    uint32_t :16;              /*!< bit:  0..15  Reserved                           */
    uint32_t CAPTEN:2;         /*!< bit: 16..17  Capture Channel x Enable           */
    uint32_t :2;               /*!< bit: 18..19  Reserved                           */
    uint32_t COPEN:2;          /*!< bit: 20..21  Capture On Pin x Enable            */
    uint32_t :10;              /*!< bit: 22..31  Reserved                           */
  } vec;                       /*!< Structure used for vec  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} TC_CTRLA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_CTRLA_OFFSET             0x00         /**< \brief (TC_CTRLA offset) Control A */
#define TC_CTRLA_RESETVALUE         0x00000000ul /**< \brief (TC_CTRLA reset_value) Control A */

#define TC_CTRLA_SWRST_Pos          0            /**< \brief (TC_CTRLA) Software Reset */
#define TC_CTRLA_SWRST              (0x1ul << TC_CTRLA_SWRST_Pos)
#define TC_CTRLA_ENABLE_Pos         1            /**< \brief (TC_CTRLA) Enable */
#define TC_CTRLA_ENABLE             (0x1ul << TC_CTRLA_ENABLE_Pos)
#define TC_CTRLA_MODE_Pos           2            /**< \brief (TC_CTRLA) Timer Counter Mode */
#define TC_CTRLA_MODE_Msk           (0x3ul << TC_CTRLA_MODE_Pos)
#define TC_CTRLA_MODE(value)        (TC_CTRLA_MODE_Msk & ((value) << TC_CTRLA_MODE_Pos))
#define   TC_CTRLA_MODE_COUNT16_Val       0x0ul  /**< \brief (TC_CTRLA) Counter in 16-bit mode */
#define   TC_CTRLA_MODE_COUNT8_Val        0x1ul  /**< \brief (TC_CTRLA) Counter in 8-bit mode */
#define   TC_CTRLA_MODE_COUNT32_Val       0x2ul  /**< \brief (TC_CTRLA) Counter in 32-bit mode */
#define TC_CTRLA_MODE_COUNT16       (TC_CTRLA_MODE_COUNT16_Val     << TC_CTRLA_MODE_Pos)
#define TC_CTRLA_MODE_COUNT8        (TC_CTRLA_MODE_COUNT8_Val      << TC_CTRLA_MODE_Pos)
#define TC_CTRLA_MODE_COUNT32       (TC_CTRLA_MODE_COUNT32_Val     << TC_CTRLA_MODE_Pos)
#define TC_CTRLA_PRESCSYNC_Pos      4            /**< \brief (TC_CTRLA) Prescaler and Counter Synchronization */
#define TC_CTRLA_PRESCSYNC_Msk      (0x3ul << TC_CTRLA_PRESCSYNC_Pos)
#define TC_CTRLA_PRESCSYNC(value)   (TC_CTRLA_PRESCSYNC_Msk & ((value) << TC_CTRLA_PRESCSYNC_Pos))
#define   TC_CTRLA_PRESCSYNC_GCLK_Val     0x0ul  /**< \brief (TC_CTRLA) Reload or reset the counter on next generic clock */
#define   TC_CTRLA_PRESCSYNC_PRESC_Val    0x1ul  /**< \brief (TC_CTRLA) Reload or reset the counter on next prescaler clock */
#define   TC_CTRLA_PRESCSYNC_RESYNC_Val   0x2ul  /**< \brief (TC_CTRLA) Reload or reset the counter on next generic clock and reset the prescaler counter */
#define TC_CTRLA_PRESCSYNC_GCLK     (TC_CTRLA_PRESCSYNC_GCLK_Val   << TC_CTRLA_PRESCSYNC_Pos)
#define TC_CTRLA_PRESCSYNC_PRESC    (TC_CTRLA_PRESCSYNC_PRESC_Val  << TC_CTRLA_PRESCSYNC_Pos)
#define TC_CTRLA_PRESCSYNC_RESYNC   (TC_CTRLA_PRESCSYNC_RESYNC_Val << TC_CTRLA_PRESCSYNC_Pos)
#define TC_CTRLA_RUNSTDBY_Pos       6            /**< \brief (TC_CTRLA) Run during Standby */
#define TC_CTRLA_RUNSTDBY           (0x1ul << TC_CTRLA_RUNSTDBY_Pos)
#define TC_CTRLA_ONDEMAND_Pos       7            /**< \brief (TC_CTRLA) Clock On Demand */
#define TC_CTRLA_ONDEMAND           (0x1ul << TC_CTRLA_ONDEMAND_Pos)
#define TC_CTRLA_PRESCALER_Pos      8            /**< \brief (TC_CTRLA) Prescaler */
#define TC_CTRLA_PRESCALER_Msk      (0x7ul << TC_CTRLA_PRESCALER_Pos)
#define TC_CTRLA_PRESCALER(value)   (TC_CTRLA_PRESCALER_Msk & ((value) << TC_CTRLA_PRESCALER_Pos))
#define   TC_CTRLA_PRESCALER_DIV1_Val     0x0ul  /**< \brief (TC_CTRLA) Prescaler: GCLK_TC */
#define   TC_CTRLA_PRESCALER_DIV2_Val     0x1ul  /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/2 */
#define   TC_CTRLA_PRESCALER_DIV4_Val     0x2ul  /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/4 */
#define   TC_CTRLA_PRESCALER_DIV8_Val     0x3ul  /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/8 */
#define   TC_CTRLA_PRESCALER_DIV16_Val    0x4ul  /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/16 */
#define   TC_CTRLA_PRESCALER_DIV64_Val    0x5ul  /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/64 */
#define   TC_CTRLA_PRESCALER_DIV256_Val   0x6ul  /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/256 */
#define   TC_CTRLA_PRESCALER_DIV1024_Val  0x7ul  /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/1024 */
#define TC_CTRLA_PRESCALER_DIV1     (TC_CTRLA_PRESCALER_DIV1_Val   << TC_CTRLA_PRESCALER_Pos)
#define TC_CTRLA_PRESCALER_DIV2     (TC_CTRLA_PRESCALER_DIV2_Val   << TC_CTRLA_PRESCALER_Pos)
#define TC_CTRLA_PRESCALER_DIV4     (TC_CTRLA_PRESCALER_DIV4_Val   << TC_CTRLA_PRESCALER_Pos)
#define TC_CTRLA_PRESCALER_DIV8     (TC_CTRLA_PRESCALER_DIV8_Val   << TC_CTRLA_PRESCALER_Pos)
#define TC_CTRLA_PRESCALER_DIV16    (TC_CTRLA_PRESCALER_DIV16_Val  << TC_CTRLA_PRESCALER_Pos)
#define TC_CTRLA_PRESCALER_DIV64    (TC_CTRLA_PRESCALER_DIV64_Val  << TC_CTRLA_PRESCALER_Pos)
#define TC_CTRLA_PRESCALER_DIV256   (TC_CTRLA_PRESCALER_DIV256_Val << TC_CTRLA_PRESCALER_Pos)
#define TC_CTRLA_PRESCALER_DIV1024  (TC_CTRLA_PRESCALER_DIV1024_Val << TC_CTRLA_PRESCALER_Pos)
#define TC_CTRLA_ALOCK_Pos          11           /**< \brief (TC_CTRLA) Auto Lock */
#define TC_CTRLA_ALOCK              (0x1ul << TC_CTRLA_ALOCK_Pos)
#define TC_CTRLA_CAPTEN0_Pos        16           /**< \brief (TC_CTRLA) Capture Channel 0 Enable */
#define TC_CTRLA_CAPTEN0            (1 << TC_CTRLA_CAPTEN0_Pos)
#define TC_CTRLA_CAPTEN1_Pos        17           /**< \brief (TC_CTRLA) Capture Channel 1 Enable */
#define TC_CTRLA_CAPTEN1            (1 << TC_CTRLA_CAPTEN1_Pos)
#define TC_CTRLA_CAPTEN_Pos         16           /**< \brief (TC_CTRLA) Capture Channel x Enable */
#define TC_CTRLA_CAPTEN_Msk         (0x3ul << TC_CTRLA_CAPTEN_Pos)
#define TC_CTRLA_CAPTEN(value)      (TC_CTRLA_CAPTEN_Msk & ((value) << TC_CTRLA_CAPTEN_Pos))
#define TC_CTRLA_COPEN0_Pos         20           /**< \brief (TC_CTRLA) Capture On Pin 0 Enable */
#define TC_CTRLA_COPEN0             (1 << TC_CTRLA_COPEN0_Pos)
#define TC_CTRLA_COPEN1_Pos         21           /**< \brief (TC_CTRLA) Capture On Pin 1 Enable */
#define TC_CTRLA_COPEN1             (1 << TC_CTRLA_COPEN1_Pos)
#define TC_CTRLA_COPEN_Pos          20           /**< \brief (TC_CTRLA) Capture On Pin x Enable */
#define TC_CTRLA_COPEN_Msk          (0x3ul << TC_CTRLA_COPEN_Pos)
#define TC_CTRLA_COPEN(value)       (TC_CTRLA_COPEN_Msk & ((value) << TC_CTRLA_COPEN_Pos))
#define TC_CTRLA_MASK               0x00330FFFul /**< \brief (TC_CTRLA) MASK Register */

/* -------- TC_CTRLBCLR : (TC Offset: 0x04) (R/W  8) Control B Clear -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  DIR:1;            /*!< bit:      0  Counter Direction                  */
    uint8_t  LUPD:1;           /*!< bit:      1  Lock Update                        */
    uint8_t  ONESHOT:1;        /*!< bit:      2  One-Shot on Counter                */
    uint8_t  :2;               /*!< bit:  3.. 4  Reserved                           */
    uint8_t  CMD:3;            /*!< bit:  5.. 7  Command                            */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_CTRLBCLR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_CTRLBCLR_OFFSET          0x04         /**< \brief (TC_CTRLBCLR offset) Control B Clear */
#define TC_CTRLBCLR_RESETVALUE      0x00ul       /**< \brief (TC_CTRLBCLR reset_value) Control B Clear */

#define TC_CTRLBCLR_DIR_Pos         0            /**< \brief (TC_CTRLBCLR) Counter Direction */
#define TC_CTRLBCLR_DIR             (0x1ul << TC_CTRLBCLR_DIR_Pos)
#define TC_CTRLBCLR_LUPD_Pos        1            /**< \brief (TC_CTRLBCLR) Lock Update */
#define TC_CTRLBCLR_LUPD            (0x1ul << TC_CTRLBCLR_LUPD_Pos)
#define TC_CTRLBCLR_ONESHOT_Pos     2            /**< \brief (TC_CTRLBCLR) One-Shot on Counter */
#define TC_CTRLBCLR_ONESHOT         (0x1ul << TC_CTRLBCLR_ONESHOT_Pos)
#define TC_CTRLBCLR_CMD_Pos         5            /**< \brief (TC_CTRLBCLR) Command */
#define TC_CTRLBCLR_CMD_Msk         (0x7ul << TC_CTRLBCLR_CMD_Pos)
#define TC_CTRLBCLR_CMD(value)      (TC_CTRLBCLR_CMD_Msk & ((value) << TC_CTRLBCLR_CMD_Pos))
#define   TC_CTRLBCLR_CMD_NONE_Val        0x0ul  /**< \brief (TC_CTRLBCLR) No action */
#define   TC_CTRLBCLR_CMD_RETRIGGER_Val   0x1ul  /**< \brief (TC_CTRLBCLR) Force a start, restart or retrigger */
#define   TC_CTRLBCLR_CMD_STOP_Val        0x2ul  /**< \brief (TC_CTRLBCLR) Force a stop */
#define   TC_CTRLBCLR_CMD_UPDATE_Val      0x3ul  /**< \brief (TC_CTRLBCLR) Force update of double-buffered register */
#define   TC_CTRLBCLR_CMD_READSYNC_Val    0x4ul  /**< \brief (TC_CTRLBCLR) Force a read synchronization of COUNT */
#define   TC_CTRLBCLR_CMD_DMAOS_Val       0x5ul  /**< \brief (TC_CTRLBCLR) One-shot DMA trigger */
#define TC_CTRLBCLR_CMD_NONE        (TC_CTRLBCLR_CMD_NONE_Val      << TC_CTRLBCLR_CMD_Pos)
#define TC_CTRLBCLR_CMD_RETRIGGER   (TC_CTRLBCLR_CMD_RETRIGGER_Val << TC_CTRLBCLR_CMD_Pos)
#define TC_CTRLBCLR_CMD_STOP        (TC_CTRLBCLR_CMD_STOP_Val      << TC_CTRLBCLR_CMD_Pos)
#define TC_CTRLBCLR_CMD_UPDATE      (TC_CTRLBCLR_CMD_UPDATE_Val    << TC_CTRLBCLR_CMD_Pos)
#define TC_CTRLBCLR_CMD_READSYNC    (TC_CTRLBCLR_CMD_READSYNC_Val  << TC_CTRLBCLR_CMD_Pos)
#define TC_CTRLBCLR_CMD_DMAOS       (TC_CTRLBCLR_CMD_DMAOS_Val     << TC_CTRLBCLR_CMD_Pos)
#define TC_CTRLBCLR_MASK            0xE7ul       /**< \brief (TC_CTRLBCLR) MASK Register */

/* -------- TC_CTRLBSET : (TC Offset: 0x05) (R/W  8) Control B Set -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  DIR:1;            /*!< bit:      0  Counter Direction                  */
    uint8_t  LUPD:1;           /*!< bit:      1  Lock Update                        */
    uint8_t  ONESHOT:1;        /*!< bit:      2  One-Shot on Counter                */
    uint8_t  :2;               /*!< bit:  3.. 4  Reserved                           */
    uint8_t  CMD:3;            /*!< bit:  5.. 7  Command                            */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_CTRLBSET_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_CTRLBSET_OFFSET          0x05         /**< \brief (TC_CTRLBSET offset) Control B Set */
#define TC_CTRLBSET_RESETVALUE      0x00ul       /**< \brief (TC_CTRLBSET reset_value) Control B Set */

#define TC_CTRLBSET_DIR_Pos         0            /**< \brief (TC_CTRLBSET) Counter Direction */
#define TC_CTRLBSET_DIR             (0x1ul << TC_CTRLBSET_DIR_Pos)
#define TC_CTRLBSET_LUPD_Pos        1            /**< \brief (TC_CTRLBSET) Lock Update */
#define TC_CTRLBSET_LUPD            (0x1ul << TC_CTRLBSET_LUPD_Pos)
#define TC_CTRLBSET_ONESHOT_Pos     2            /**< \brief (TC_CTRLBSET) One-Shot on Counter */
#define TC_CTRLBSET_ONESHOT         (0x1ul << TC_CTRLBSET_ONESHOT_Pos)
#define TC_CTRLBSET_CMD_Pos         5            /**< \brief (TC_CTRLBSET) Command */
#define TC_CTRLBSET_CMD_Msk         (0x7ul << TC_CTRLBSET_CMD_Pos)
#define TC_CTRLBSET_CMD(value)      (TC_CTRLBSET_CMD_Msk & ((value) << TC_CTRLBSET_CMD_Pos))
#define   TC_CTRLBSET_CMD_NONE_Val        0x0ul  /**< \brief (TC_CTRLBSET) No action */
#define   TC_CTRLBSET_CMD_RETRIGGER_Val   0x1ul  /**< \brief (TC_CTRLBSET) Force a start, restart or retrigger */
#define   TC_CTRLBSET_CMD_STOP_Val        0x2ul  /**< \brief (TC_CTRLBSET) Force a stop */
#define   TC_CTRLBSET_CMD_UPDATE_Val      0x3ul  /**< \brief (TC_CTRLBSET) Force update of double-buffered register */
#define   TC_CTRLBSET_CMD_READSYNC_Val    0x4ul  /**< \brief (TC_CTRLBSET) Force a read synchronization of COUNT */
#define   TC_CTRLBSET_CMD_DMAOS_Val       0x5ul  /**< \brief (TC_CTRLBSET) One-shot DMA trigger */
#define TC_CTRLBSET_CMD_NONE        (TC_CTRLBSET_CMD_NONE_Val      << TC_CTRLBSET_CMD_Pos)
#define TC_CTRLBSET_CMD_RETRIGGER   (TC_CTRLBSET_CMD_RETRIGGER_Val << TC_CTRLBSET_CMD_Pos)
#define TC_CTRLBSET_CMD_STOP        (TC_CTRLBSET_CMD_STOP_Val      << TC_CTRLBSET_CMD_Pos)
#define TC_CTRLBSET_CMD_UPDATE      (TC_CTRLBSET_CMD_UPDATE_Val    << TC_CTRLBSET_CMD_Pos)
#define TC_CTRLBSET_CMD_READSYNC    (TC_CTRLBSET_CMD_READSYNC_Val  << TC_CTRLBSET_CMD_Pos)
#define TC_CTRLBSET_CMD_DMAOS       (TC_CTRLBSET_CMD_DMAOS_Val     << TC_CTRLBSET_CMD_Pos)
#define TC_CTRLBSET_MASK            0xE7ul       /**< \brief (TC_CTRLBSET) MASK Register */

/* -------- TC_EVCTRL : (TC Offset: 0x06) (R/W 16) Event Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint16_t EVACT:3;          /*!< bit:  0.. 2  Event Action                       */
    uint16_t :1;               /*!< bit:      3  Reserved                           */
    uint16_t TCINV:1;          /*!< bit:      4  TC Event Input Polarity            */
    uint16_t TCEI:1;           /*!< bit:      5  TC Event Enable                    */
    uint16_t :2;               /*!< bit:  6.. 7  Reserved                           */
    uint16_t OVFEO:1;          /*!< bit:      8  Event Output Enable                */
    uint16_t :3;               /*!< bit:  9..11  Reserved                           */
    uint16_t MCEO0:1;          /*!< bit:     12  MC Event Output Enable 0           */
    uint16_t MCEO1:1;          /*!< bit:     13  MC Event Output Enable 1           */
    uint16_t :2;               /*!< bit: 14..15  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  struct {
    uint16_t :12;              /*!< bit:  0..11  Reserved                           */
    uint16_t MCEO:2;           /*!< bit: 12..13  MC Event Output Enable x           */
    uint16_t :2;               /*!< bit: 14..15  Reserved                           */
  } vec;                       /*!< Structure used for vec  access                  */
  uint16_t reg;                /*!< Type      used for register access              */
} TC_EVCTRL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_EVCTRL_OFFSET            0x06         /**< \brief (TC_EVCTRL offset) Event Control */
#define TC_EVCTRL_RESETVALUE        0x0000ul     /**< \brief (TC_EVCTRL reset_value) Event Control */

#define TC_EVCTRL_EVACT_Pos         0            /**< \brief (TC_EVCTRL) Event Action */
#define TC_EVCTRL_EVACT_Msk         (0x7ul << TC_EVCTRL_EVACT_Pos)
#define TC_EVCTRL_EVACT(value)      (TC_EVCTRL_EVACT_Msk & ((value) << TC_EVCTRL_EVACT_Pos))
#define   TC_EVCTRL_EVACT_OFF_Val         0x0ul  /**< \brief (TC_EVCTRL) Event action disabled */
#define   TC_EVCTRL_EVACT_RETRIGGER_Val   0x1ul  /**< \brief (TC_EVCTRL) Start, restart or retrigger TC on event */
#define   TC_EVCTRL_EVACT_COUNT_Val       0x2ul  /**< \brief (TC_EVCTRL) Count on event */
#define   TC_EVCTRL_EVACT_START_Val       0x3ul  /**< \brief (TC_EVCTRL) Start TC on event */
#define   TC_EVCTRL_EVACT_STAMP_Val       0x4ul  /**< \brief (TC_EVCTRL) Time stamp capture */
#define   TC_EVCTRL_EVACT_PPW_Val         0x5ul  /**< \brief (TC_EVCTRL) Period captured in CC0, pulse width in CC1 */
#define   TC_EVCTRL_EVACT_PWP_Val         0x6ul  /**< \brief (TC_EVCTRL) Period captured in CC1, pulse width in CC0 */
#define   TC_EVCTRL_EVACT_PW_Val          0x7ul  /**< \brief (TC_EVCTRL) Pulse width capture */
#define TC_EVCTRL_EVACT_OFF         (TC_EVCTRL_EVACT_OFF_Val       << TC_EVCTRL_EVACT_Pos)
#define TC_EVCTRL_EVACT_RETRIGGER   (TC_EVCTRL_EVACT_RETRIGGER_Val << TC_EVCTRL_EVACT_Pos)
#define TC_EVCTRL_EVACT_COUNT       (TC_EVCTRL_EVACT_COUNT_Val     << TC_EVCTRL_EVACT_Pos)
#define TC_EVCTRL_EVACT_START       (TC_EVCTRL_EVACT_START_Val     << TC_EVCTRL_EVACT_Pos)
#define TC_EVCTRL_EVACT_STAMP       (TC_EVCTRL_EVACT_STAMP_Val     << TC_EVCTRL_EVACT_Pos)
#define TC_EVCTRL_EVACT_PPW         (TC_EVCTRL_EVACT_PPW_Val       << TC_EVCTRL_EVACT_Pos)
#define TC_EVCTRL_EVACT_PWP         (TC_EVCTRL_EVACT_PWP_Val       << TC_EVCTRL_EVACT_Pos)
#define TC_EVCTRL_EVACT_PW          (TC_EVCTRL_EVACT_PW_Val        << TC_EVCTRL_EVACT_Pos)
#define TC_EVCTRL_TCINV_Pos         4            /**< \brief (TC_EVCTRL) TC Event Input Polarity */
#define TC_EVCTRL_TCINV             (0x1ul << TC_EVCTRL_TCINV_Pos)
#define TC_EVCTRL_TCEI_Pos          5            /**< \brief (TC_EVCTRL) TC Event Enable */
#define TC_EVCTRL_TCEI              (0x1ul << TC_EVCTRL_TCEI_Pos)
#define TC_EVCTRL_OVFEO_Pos         8            /**< \brief (TC_EVCTRL) Event Output Enable */
#define TC_EVCTRL_OVFEO             (0x1ul << TC_EVCTRL_OVFEO_Pos)
#define TC_EVCTRL_MCEO0_Pos         12           /**< \brief (TC_EVCTRL) MC Event Output Enable 0 */
#define TC_EVCTRL_MCEO0             (1 << TC_EVCTRL_MCEO0_Pos)
#define TC_EVCTRL_MCEO1_Pos         13           /**< \brief (TC_EVCTRL) MC Event Output Enable 1 */
#define TC_EVCTRL_MCEO1             (1 << TC_EVCTRL_MCEO1_Pos)
#define TC_EVCTRL_MCEO_Pos          12           /**< \brief (TC_EVCTRL) MC Event Output Enable x */
#define TC_EVCTRL_MCEO_Msk          (0x3ul << TC_EVCTRL_MCEO_Pos)
#define TC_EVCTRL_MCEO(value)       (TC_EVCTRL_MCEO_Msk & ((value) << TC_EVCTRL_MCEO_Pos))
#define TC_EVCTRL_MASK              0x3137ul     /**< \brief (TC_EVCTRL) MASK Register */

/* -------- TC_INTENCLR : (TC Offset: 0x08) (R/W  8) Interrupt Enable Clear -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  OVF:1;            /*!< bit:      0  OVF Interrupt Disable              */
    uint8_t  ERR:1;            /*!< bit:      1  ERR Interrupt Disable              */
    uint8_t  :2;               /*!< bit:  2.. 3  Reserved                           */
    uint8_t  MC0:1;            /*!< bit:      4  MC Interrupt Disable 0             */
    uint8_t  MC1:1;            /*!< bit:      5  MC Interrupt Disable 1             */
    uint8_t  :2;               /*!< bit:  6.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  struct {
    uint8_t  :4;               /*!< bit:  0.. 3  Reserved                           */
    uint8_t  MC:2;             /*!< bit:  4.. 5  MC Interrupt Disable x             */
    uint8_t  :2;               /*!< bit:  6.. 7  Reserved                           */
  } vec;                       /*!< Structure used for vec  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_INTENCLR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_INTENCLR_OFFSET          0x08         /**< \brief (TC_INTENCLR offset) Interrupt Enable Clear */
#define TC_INTENCLR_RESETVALUE      0x00ul       /**< \brief (TC_INTENCLR reset_value) Interrupt Enable Clear */

#define TC_INTENCLR_OVF_Pos         0            /**< \brief (TC_INTENCLR) OVF Interrupt Disable */
#define TC_INTENCLR_OVF             (0x1ul << TC_INTENCLR_OVF_Pos)
#define TC_INTENCLR_ERR_Pos         1            /**< \brief (TC_INTENCLR) ERR Interrupt Disable */
#define TC_INTENCLR_ERR             (0x1ul << TC_INTENCLR_ERR_Pos)
#define TC_INTENCLR_MC0_Pos         4            /**< \brief (TC_INTENCLR) MC Interrupt Disable 0 */
#define TC_INTENCLR_MC0             (1 << TC_INTENCLR_MC0_Pos)
#define TC_INTENCLR_MC1_Pos         5            /**< \brief (TC_INTENCLR) MC Interrupt Disable 1 */
#define TC_INTENCLR_MC1             (1 << TC_INTENCLR_MC1_Pos)
#define TC_INTENCLR_MC_Pos          4            /**< \brief (TC_INTENCLR) MC Interrupt Disable x */
#define TC_INTENCLR_MC_Msk          (0x3ul << TC_INTENCLR_MC_Pos)
#define TC_INTENCLR_MC(value)       (TC_INTENCLR_MC_Msk & ((value) << TC_INTENCLR_MC_Pos))
#define TC_INTENCLR_MASK            0x33ul       /**< \brief (TC_INTENCLR) MASK Register */

/* -------- TC_INTENSET : (TC Offset: 0x09) (R/W  8) Interrupt Enable Set -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  OVF:1;            /*!< bit:      0  OVF Interrupt Enable               */
    uint8_t  ERR:1;            /*!< bit:      1  ERR Interrupt Enable               */
    uint8_t  :2;               /*!< bit:  2.. 3  Reserved                           */
    uint8_t  MC0:1;            /*!< bit:      4  MC Interrupt Enable 0              */
    uint8_t  MC1:1;            /*!< bit:      5  MC Interrupt Enable 1              */
    uint8_t  :2;               /*!< bit:  6.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  struct {
    uint8_t  :4;               /*!< bit:  0.. 3  Reserved                           */
    uint8_t  MC:2;             /*!< bit:  4.. 5  MC Interrupt Enable x              */
    uint8_t  :2;               /*!< bit:  6.. 7  Reserved                           */
  } vec;                       /*!< Structure used for vec  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_INTENSET_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_INTENSET_OFFSET          0x09         /**< \brief (TC_INTENSET offset) Interrupt Enable Set */
#define TC_INTENSET_RESETVALUE      0x00ul       /**< \brief (TC_INTENSET reset_value) Interrupt Enable Set */

#define TC_INTENSET_OVF_Pos         0            /**< \brief (TC_INTENSET) OVF Interrupt Enable */
#define TC_INTENSET_OVF             (0x1ul << TC_INTENSET_OVF_Pos)
#define TC_INTENSET_ERR_Pos         1            /**< \brief (TC_INTENSET) ERR Interrupt Enable */
#define TC_INTENSET_ERR             (0x1ul << TC_INTENSET_ERR_Pos)
#define TC_INTENSET_MC0_Pos         4            /**< \brief (TC_INTENSET) MC Interrupt Enable 0 */
#define TC_INTENSET_MC0             (1 << TC_INTENSET_MC0_Pos)
#define TC_INTENSET_MC1_Pos         5            /**< \brief (TC_INTENSET) MC Interrupt Enable 1 */
#define TC_INTENSET_MC1             (1 << TC_INTENSET_MC1_Pos)
#define TC_INTENSET_MC_Pos          4            /**< \brief (TC_INTENSET) MC Interrupt Enable x */
#define TC_INTENSET_MC_Msk          (0x3ul << TC_INTENSET_MC_Pos)
#define TC_INTENSET_MC(value)       (TC_INTENSET_MC_Msk & ((value) << TC_INTENSET_MC_Pos))
#define TC_INTENSET_MASK            0x33ul       /**< \brief (TC_INTENSET) MASK Register */

/* -------- TC_INTFLAG : (TC Offset: 0x0A) (R/W  8) Interrupt Flag Status and Clear -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { // __I to avoid read-modify-write on write-to-clear register
  struct {
    __I uint8_t  OVF:1;            /*!< bit:      0  OVF Interrupt Flag                 */
    __I uint8_t  ERR:1;            /*!< bit:      1  ERR Interrupt Flag                 */
    __I uint8_t  :2;               /*!< bit:  2.. 3  Reserved                           */
    __I uint8_t  MC0:1;            /*!< bit:      4  MC Interrupt Flag 0                */
    __I uint8_t  MC1:1;            /*!< bit:      5  MC Interrupt Flag 1                */
    __I uint8_t  :2;               /*!< bit:  6.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  struct {
    __I uint8_t  :4;               /*!< bit:  0.. 3  Reserved                           */
    __I uint8_t  MC:2;             /*!< bit:  4.. 5  MC Interrupt Flag x                */
    __I uint8_t  :2;               /*!< bit:  6.. 7  Reserved                           */
  } vec;                       /*!< Structure used for vec  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_INTFLAG_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_INTFLAG_OFFSET           0x0A         /**< \brief (TC_INTFLAG offset) Interrupt Flag Status and Clear */
#define TC_INTFLAG_RESETVALUE       0x00ul       /**< \brief (TC_INTFLAG reset_value) Interrupt Flag Status and Clear */

#define TC_INTFLAG_OVF_Pos          0            /**< \brief (TC_INTFLAG) OVF Interrupt Flag */
#define TC_INTFLAG_OVF              (0x1ul << TC_INTFLAG_OVF_Pos)
#define TC_INTFLAG_ERR_Pos          1            /**< \brief (TC_INTFLAG) ERR Interrupt Flag */
#define TC_INTFLAG_ERR              (0x1ul << TC_INTFLAG_ERR_Pos)
#define TC_INTFLAG_MC0_Pos          4            /**< \brief (TC_INTFLAG) MC Interrupt Flag 0 */
#define TC_INTFLAG_MC0              (1 << TC_INTFLAG_MC0_Pos)
#define TC_INTFLAG_MC1_Pos          5            /**< \brief (TC_INTFLAG) MC Interrupt Flag 1 */
#define TC_INTFLAG_MC1              (1 << TC_INTFLAG_MC1_Pos)
#define TC_INTFLAG_MC_Pos           4            /**< \brief (TC_INTFLAG) MC Interrupt Flag x */
#define TC_INTFLAG_MC_Msk           (0x3ul << TC_INTFLAG_MC_Pos)
#define TC_INTFLAG_MC(value)        (TC_INTFLAG_MC_Msk & ((value) << TC_INTFLAG_MC_Pos))
#define TC_INTFLAG_MASK             0x33ul       /**< \brief (TC_INTFLAG) MASK Register */

/* -------- TC_STATUS : (TC Offset: 0x0B) (R/W  8) Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  STOP:1;           /*!< bit:      0  Stop Status Flag                   */
    uint8_t  SLAVE:1;          /*!< bit:      1  Slave Status Flag                  */
    uint8_t  :1;               /*!< bit:      2  Reserved                           */
    uint8_t  PERBUFV:1;        /*!< bit:      3  Synchronization Busy Status        */
    uint8_t  CCBUFV0:1;        /*!< bit:      4  Compare channel buffer 0 valid     */
    uint8_t  CCBUFV1:1;        /*!< bit:      5  Compare channel buffer 1 valid     */
    uint8_t  :2;               /*!< bit:  6.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  struct {
    uint8_t  :4;               /*!< bit:  0.. 3  Reserved                           */
    uint8_t  CCBUFV:2;         /*!< bit:  4.. 5  Compare channel buffer x valid     */
    uint8_t  :2;               /*!< bit:  6.. 7  Reserved                           */
  } vec;                       /*!< Structure used for vec  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_STATUS_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_STATUS_OFFSET            0x0B         /**< \brief (TC_STATUS offset) Status */
#define TC_STATUS_RESETVALUE        0x01ul       /**< \brief (TC_STATUS reset_value) Status */

#define TC_STATUS_STOP_Pos          0            /**< \brief (TC_STATUS) Stop Status Flag */
#define TC_STATUS_STOP              (0x1ul << TC_STATUS_STOP_Pos)
#define TC_STATUS_SLAVE_Pos         1            /**< \brief (TC_STATUS) Slave Status Flag */
#define TC_STATUS_SLAVE             (0x1ul << TC_STATUS_SLAVE_Pos)
#define TC_STATUS_PERBUFV_Pos       3            /**< \brief (TC_STATUS) Synchronization Busy Status */
#define TC_STATUS_PERBUFV           (0x1ul << TC_STATUS_PERBUFV_Pos)
#define TC_STATUS_CCBUFV0_Pos       4            /**< \brief (TC_STATUS) Compare channel buffer 0 valid */
#define TC_STATUS_CCBUFV0           (1 << TC_STATUS_CCBUFV0_Pos)
#define TC_STATUS_CCBUFV1_Pos       5            /**< \brief (TC_STATUS) Compare channel buffer 1 valid */
#define TC_STATUS_CCBUFV1           (1 << TC_STATUS_CCBUFV1_Pos)
#define TC_STATUS_CCBUFV_Pos        4            /**< \brief (TC_STATUS) Compare channel buffer x valid */
#define TC_STATUS_CCBUFV_Msk        (0x3ul << TC_STATUS_CCBUFV_Pos)
#define TC_STATUS_CCBUFV(value)     (TC_STATUS_CCBUFV_Msk & ((value) << TC_STATUS_CCBUFV_Pos))
#define TC_STATUS_MASK              0x3Bul       /**< \brief (TC_STATUS) MASK Register */

/* -------- TC_WAVE : (TC Offset: 0x0C) (R/W  8) Waveform Generation Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  WAVEGEN:2;        /*!< bit:  0.. 1  Waveform Generation Mode           */
    uint8_t  :6;               /*!< bit:  2.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_WAVE_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_WAVE_OFFSET              0x0C         /**< \brief (TC_WAVE offset) Waveform Generation Control */
#define TC_WAVE_RESETVALUE          0x00ul       /**< \brief (TC_WAVE reset_value) Waveform Generation Control */

#define TC_WAVE_WAVEGEN_Pos         0            /**< \brief (TC_WAVE) Waveform Generation Mode */
#define TC_WAVE_WAVEGEN_Msk         (0x3ul << TC_WAVE_WAVEGEN_Pos)
#define TC_WAVE_WAVEGEN(value)      (TC_WAVE_WAVEGEN_Msk & ((value) << TC_WAVE_WAVEGEN_Pos))
#define   TC_WAVE_WAVEGEN_NFRQ_Val        0x0ul  /**< \brief (TC_WAVE) Normal frequency */
#define   TC_WAVE_WAVEGEN_MFRQ_Val        0x1ul  /**< \brief (TC_WAVE) Match frequency */
#define   TC_WAVE_WAVEGEN_NPWM_Val        0x2ul  /**< \brief (TC_WAVE) Normal PWM */
#define   TC_WAVE_WAVEGEN_MPWM_Val        0x3ul  /**< \brief (TC_WAVE) Match PWM */
#define TC_WAVE_WAVEGEN_NFRQ        (TC_WAVE_WAVEGEN_NFRQ_Val      << TC_WAVE_WAVEGEN_Pos)
#define TC_WAVE_WAVEGEN_MFRQ        (TC_WAVE_WAVEGEN_MFRQ_Val      << TC_WAVE_WAVEGEN_Pos)
#define TC_WAVE_WAVEGEN_NPWM        (TC_WAVE_WAVEGEN_NPWM_Val      << TC_WAVE_WAVEGEN_Pos)
#define TC_WAVE_WAVEGEN_MPWM        (TC_WAVE_WAVEGEN_MPWM_Val      << TC_WAVE_WAVEGEN_Pos)
#define TC_WAVE_MASK                0x03ul       /**< \brief (TC_WAVE) MASK Register */

/* -------- TC_DRVCTRL : (TC Offset: 0x0D) (R/W  8) Control C -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  INVEN0:1;         /*!< bit:      0  Output Waveform Invert Enable 0    */
    uint8_t  INVEN1:1;         /*!< bit:      1  Output Waveform Invert Enable 1    */
    uint8_t  :6;               /*!< bit:  2.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  struct {
    uint8_t  INVEN:2;          /*!< bit:  0.. 1  Output Waveform Invert Enable x    */
    uint8_t  :6;               /*!< bit:  2.. 7  Reserved                           */
  } vec;                       /*!< Structure used for vec  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_DRVCTRL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_DRVCTRL_OFFSET           0x0D         /**< \brief (TC_DRVCTRL offset) Control C */
#define TC_DRVCTRL_RESETVALUE       0x00ul       /**< \brief (TC_DRVCTRL reset_value) Control C */

#define TC_DRVCTRL_INVEN0_Pos       0            /**< \brief (TC_DRVCTRL) Output Waveform Invert Enable 0 */
#define TC_DRVCTRL_INVEN0           (1 << TC_DRVCTRL_INVEN0_Pos)
#define TC_DRVCTRL_INVEN1_Pos       1            /**< \brief (TC_DRVCTRL) Output Waveform Invert Enable 1 */
#define TC_DRVCTRL_INVEN1           (1 << TC_DRVCTRL_INVEN1_Pos)
#define TC_DRVCTRL_INVEN_Pos        0            /**< \brief (TC_DRVCTRL) Output Waveform Invert Enable x */
#define TC_DRVCTRL_INVEN_Msk        (0x3ul << TC_DRVCTRL_INVEN_Pos)
#define TC_DRVCTRL_INVEN(value)     (TC_DRVCTRL_INVEN_Msk & ((value) << TC_DRVCTRL_INVEN_Pos))
#define TC_DRVCTRL_MASK             0x03ul       /**< \brief (TC_DRVCTRL) MASK Register */

/* -------- TC_DBGCTRL : (TC Offset: 0x0F) (R/W  8) Debug Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  DBGRUN:1;         /*!< bit:      0  Run During Debug                   */
    uint8_t  :7;               /*!< bit:  1.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_DBGCTRL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_DBGCTRL_OFFSET           0x0F         /**< \brief (TC_DBGCTRL offset) Debug Control */
#define TC_DBGCTRL_RESETVALUE       0x00ul       /**< \brief (TC_DBGCTRL reset_value) Debug Control */

#define TC_DBGCTRL_DBGRUN_Pos       0            /**< \brief (TC_DBGCTRL) Run During Debug */
#define TC_DBGCTRL_DBGRUN           (0x1ul << TC_DBGCTRL_DBGRUN_Pos)
#define TC_DBGCTRL_MASK             0x01ul       /**< \brief (TC_DBGCTRL) MASK Register */

/* -------- TC_SYNCBUSY : (TC Offset: 0x10) (R/  32) Synchronization Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t SWRST:1;          /*!< bit:      0  swrst                              */
    uint32_t ENABLE:1;         /*!< bit:      1  enable                             */
    uint32_t CTRLB:1;          /*!< bit:      2  CTRLB                              */
    uint32_t STATUS:1;         /*!< bit:      3  STATUS                             */
    uint32_t COUNT:1;          /*!< bit:      4  Counter                            */
    uint32_t PER:1;            /*!< bit:      5  Period                             */
    uint32_t CC0:1;            /*!< bit:      6  Compare Channel 0                  */
    uint32_t CC1:1;            /*!< bit:      7  Compare Channel 1                  */
    uint32_t :24;              /*!< bit:  8..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  struct {
    uint32_t :6;               /*!< bit:  0.. 5  Reserved                           */
    uint32_t CC:2;             /*!< bit:  6.. 7  Compare Channel x                  */
    uint32_t :24;              /*!< bit:  8..31  Reserved                           */
  } vec;                       /*!< Structure used for vec  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} TC_SYNCBUSY_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_SYNCBUSY_OFFSET          0x10         /**< \brief (TC_SYNCBUSY offset) Synchronization Status */
#define TC_SYNCBUSY_RESETVALUE      0x00000000ul /**< \brief (TC_SYNCBUSY reset_value) Synchronization Status */

#define TC_SYNCBUSY_SWRST_Pos       0            /**< \brief (TC_SYNCBUSY) swrst */
#define TC_SYNCBUSY_SWRST           (0x1ul << TC_SYNCBUSY_SWRST_Pos)
#define TC_SYNCBUSY_ENABLE_Pos      1            /**< \brief (TC_SYNCBUSY) enable */
#define TC_SYNCBUSY_ENABLE          (0x1ul << TC_SYNCBUSY_ENABLE_Pos)
#define TC_SYNCBUSY_CTRLB_Pos       2            /**< \brief (TC_SYNCBUSY) CTRLB */
#define TC_SYNCBUSY_CTRLB           (0x1ul << TC_SYNCBUSY_CTRLB_Pos)
#define TC_SYNCBUSY_STATUS_Pos      3            /**< \brief (TC_SYNCBUSY) STATUS */
#define TC_SYNCBUSY_STATUS          (0x1ul << TC_SYNCBUSY_STATUS_Pos)
#define TC_SYNCBUSY_COUNT_Pos       4            /**< \brief (TC_SYNCBUSY) Counter */
#define TC_SYNCBUSY_COUNT           (0x1ul << TC_SYNCBUSY_COUNT_Pos)
#define TC_SYNCBUSY_PER_Pos         5            /**< \brief (TC_SYNCBUSY) Period */
#define TC_SYNCBUSY_PER             (0x1ul << TC_SYNCBUSY_PER_Pos)
#define TC_SYNCBUSY_CC0_Pos         6            /**< \brief (TC_SYNCBUSY) Compare Channel 0 */
#define TC_SYNCBUSY_CC0             (1 << TC_SYNCBUSY_CC0_Pos)
#define TC_SYNCBUSY_CC1_Pos         7            /**< \brief (TC_SYNCBUSY) Compare Channel 1 */
#define TC_SYNCBUSY_CC1             (1 << TC_SYNCBUSY_CC1_Pos)
#define TC_SYNCBUSY_CC_Pos          6            /**< \brief (TC_SYNCBUSY) Compare Channel x */
#define TC_SYNCBUSY_CC_Msk          (0x3ul << TC_SYNCBUSY_CC_Pos)
#define TC_SYNCBUSY_CC(value)       (TC_SYNCBUSY_CC_Msk & ((value) << TC_SYNCBUSY_CC_Pos))
#define TC_SYNCBUSY_MASK            0x000000FFul /**< \brief (TC_SYNCBUSY) MASK Register */

/* -------- TC_COUNT16_COUNT : (TC Offset: 0x14) (R/W 16) COUNT16 COUNT16 Count -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint16_t COUNT:16;         /*!< bit:  0..15  Counter Value                      */
  } bit;                       /*!< Structure used for bit  access                  */
  uint16_t reg;                /*!< Type      used for register access              */
} TC_COUNT16_COUNT_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT16_COUNT_OFFSET     0x14         /**< \brief (TC_COUNT16_COUNT offset) COUNT16 Count */
#define TC_COUNT16_COUNT_RESETVALUE 0x0000ul     /**< \brief (TC_COUNT16_COUNT reset_value) COUNT16 Count */

#define TC_COUNT16_COUNT_COUNT_Pos  0            /**< \brief (TC_COUNT16_COUNT) Counter Value */
#define TC_COUNT16_COUNT_COUNT_Msk  (0xFFFFul << TC_COUNT16_COUNT_COUNT_Pos)
#define TC_COUNT16_COUNT_COUNT(value) (TC_COUNT16_COUNT_COUNT_Msk & ((value) << TC_COUNT16_COUNT_COUNT_Pos))
#define TC_COUNT16_COUNT_MASK       0xFFFFul     /**< \brief (TC_COUNT16_COUNT) MASK Register */

/* -------- TC_COUNT32_COUNT : (TC Offset: 0x14) (R/W 32) COUNT32 COUNT32 Count -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t COUNT:32;         /*!< bit:  0..31  Counter Value                      */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} TC_COUNT32_COUNT_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT32_COUNT_OFFSET     0x14         /**< \brief (TC_COUNT32_COUNT offset) COUNT32 Count */
#define TC_COUNT32_COUNT_RESETVALUE 0x00000000ul /**< \brief (TC_COUNT32_COUNT reset_value) COUNT32 Count */

#define TC_COUNT32_COUNT_COUNT_Pos  0            /**< \brief (TC_COUNT32_COUNT) Counter Value */
#define TC_COUNT32_COUNT_COUNT_Msk  (0xFFFFFFFFul << TC_COUNT32_COUNT_COUNT_Pos)
#define TC_COUNT32_COUNT_COUNT(value) (TC_COUNT32_COUNT_COUNT_Msk & ((value) << TC_COUNT32_COUNT_COUNT_Pos))
#define TC_COUNT32_COUNT_MASK       0xFFFFFFFFul /**< \brief (TC_COUNT32_COUNT) MASK Register */

/* -------- TC_COUNT8_COUNT : (TC Offset: 0x14) (R/W  8) COUNT8 COUNT8 Count -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  COUNT:8;          /*!< bit:  0.. 7  Counter Value                      */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_COUNT8_COUNT_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT8_COUNT_OFFSET      0x14         /**< \brief (TC_COUNT8_COUNT offset) COUNT8 Count */
#define TC_COUNT8_COUNT_RESETVALUE  0x00ul       /**< \brief (TC_COUNT8_COUNT reset_value) COUNT8 Count */

#define TC_COUNT8_COUNT_COUNT_Pos   0            /**< \brief (TC_COUNT8_COUNT) Counter Value */
#define TC_COUNT8_COUNT_COUNT_Msk   (0xFFul << TC_COUNT8_COUNT_COUNT_Pos)
#define TC_COUNT8_COUNT_COUNT(value) (TC_COUNT8_COUNT_COUNT_Msk & ((value) << TC_COUNT8_COUNT_COUNT_Pos))
#define TC_COUNT8_COUNT_MASK        0xFFul       /**< \brief (TC_COUNT8_COUNT) MASK Register */

/* -------- TC_COUNT8_PER : (TC Offset: 0x1B) (R/W  8) COUNT8 COUNT8 Period -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  PER:8;            /*!< bit:  0.. 7  Period Value                       */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_COUNT8_PER_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT8_PER_OFFSET        0x1B         /**< \brief (TC_COUNT8_PER offset) COUNT8 Period */
#define TC_COUNT8_PER_RESETVALUE    0xFFul       /**< \brief (TC_COUNT8_PER reset_value) COUNT8 Period */

#define TC_COUNT8_PER_PER_Pos       0            /**< \brief (TC_COUNT8_PER) Period Value */
#define TC_COUNT8_PER_PER_Msk       (0xFFul << TC_COUNT8_PER_PER_Pos)
#define TC_COUNT8_PER_PER(value)    (TC_COUNT8_PER_PER_Msk & ((value) << TC_COUNT8_PER_PER_Pos))
#define TC_COUNT8_PER_MASK          0xFFul       /**< \brief (TC_COUNT8_PER) MASK Register */

/* -------- TC_COUNT16_CC : (TC Offset: 0x1C) (R/W 16) COUNT16 COUNT16 Compare and Capture -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint16_t CC:16;            /*!< bit:  0..15  Counter/Compare Value              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint16_t reg;                /*!< Type      used for register access              */
} TC_COUNT16_CC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT16_CC_OFFSET        0x1C         /**< \brief (TC_COUNT16_CC offset) COUNT16 Compare and Capture */
#define TC_COUNT16_CC_RESETVALUE    0x0000ul     /**< \brief (TC_COUNT16_CC reset_value) COUNT16 Compare and Capture */

#define TC_COUNT16_CC_CC_Pos        0            /**< \brief (TC_COUNT16_CC) Counter/Compare Value */
#define TC_COUNT16_CC_CC_Msk        (0xFFFFul << TC_COUNT16_CC_CC_Pos)
#define TC_COUNT16_CC_CC(value)     (TC_COUNT16_CC_CC_Msk & ((value) << TC_COUNT16_CC_CC_Pos))
#define TC_COUNT16_CC_MASK          0xFFFFul     /**< \brief (TC_COUNT16_CC) MASK Register */

/* -------- TC_COUNT32_CC : (TC Offset: 0x1C) (R/W 32) COUNT32 COUNT32 Compare and Capture -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t CC:32;            /*!< bit:  0..31  Counter/Compare Value              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} TC_COUNT32_CC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT32_CC_OFFSET        0x1C         /**< \brief (TC_COUNT32_CC offset) COUNT32 Compare and Capture */
#define TC_COUNT32_CC_RESETVALUE    0x00000000ul /**< \brief (TC_COUNT32_CC reset_value) COUNT32 Compare and Capture */

#define TC_COUNT32_CC_CC_Pos        0            /**< \brief (TC_COUNT32_CC) Counter/Compare Value */
#define TC_COUNT32_CC_CC_Msk        (0xFFFFFFFFul << TC_COUNT32_CC_CC_Pos)
#define TC_COUNT32_CC_CC(value)     (TC_COUNT32_CC_CC_Msk & ((value) << TC_COUNT32_CC_CC_Pos))
#define TC_COUNT32_CC_MASK          0xFFFFFFFFul /**< \brief (TC_COUNT32_CC) MASK Register */

/* -------- TC_COUNT8_CC : (TC Offset: 0x1C) (R/W  8) COUNT8 COUNT8 Compare and Capture -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  CC:8;             /*!< bit:  0.. 7  Counter/Compare Value              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_COUNT8_CC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT8_CC_OFFSET         0x1C         /**< \brief (TC_COUNT8_CC offset) COUNT8 Compare and Capture */
#define TC_COUNT8_CC_RESETVALUE     0x00ul       /**< \brief (TC_COUNT8_CC reset_value) COUNT8 Compare and Capture */

#define TC_COUNT8_CC_CC_Pos         0            /**< \brief (TC_COUNT8_CC) Counter/Compare Value */
#define TC_COUNT8_CC_CC_Msk         (0xFFul << TC_COUNT8_CC_CC_Pos)
#define TC_COUNT8_CC_CC(value)      (TC_COUNT8_CC_CC_Msk & ((value) << TC_COUNT8_CC_CC_Pos))
#define TC_COUNT8_CC_MASK           0xFFul       /**< \brief (TC_COUNT8_CC) MASK Register */

/* -------- TC_COUNT8_PERBUF : (TC Offset: 0x2F) (R/W  8) COUNT8 COUNT8 Period Buffer -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  PERBUF:8;         /*!< bit:  0.. 7  Period Buffer Value                */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_COUNT8_PERBUF_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT8_PERBUF_OFFSET     0x2F         /**< \brief (TC_COUNT8_PERBUF offset) COUNT8 Period Buffer */
#define TC_COUNT8_PERBUF_RESETVALUE 0xFFul       /**< \brief (TC_COUNT8_PERBUF reset_value) COUNT8 Period Buffer */

#define TC_COUNT8_PERBUF_PERBUF_Pos 0            /**< \brief (TC_COUNT8_PERBUF) Period Buffer Value */
#define TC_COUNT8_PERBUF_PERBUF_Msk (0xFFul << TC_COUNT8_PERBUF_PERBUF_Pos)
#define TC_COUNT8_PERBUF_PERBUF(value) (TC_COUNT8_PERBUF_PERBUF_Msk & ((value) << TC_COUNT8_PERBUF_PERBUF_Pos))
#define TC_COUNT8_PERBUF_MASK       0xFFul       /**< \brief (TC_COUNT8_PERBUF) MASK Register */

/* -------- TC_COUNT16_CCBUF : (TC Offset: 0x30) (R/W 16) COUNT16 COUNT16 Compare and Capture Buffer -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint16_t CCBUF:16;         /*!< bit:  0..15  Counter/Compare Buffer Value       */
  } bit;                       /*!< Structure used for bit  access                  */
  uint16_t reg;                /*!< Type      used for register access              */
} TC_COUNT16_CCBUF_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT16_CCBUF_OFFSET     0x30         /**< \brief (TC_COUNT16_CCBUF offset) COUNT16 Compare and Capture Buffer */
#define TC_COUNT16_CCBUF_RESETVALUE 0x0000ul     /**< \brief (TC_COUNT16_CCBUF reset_value) COUNT16 Compare and Capture Buffer */

#define TC_COUNT16_CCBUF_CCBUF_Pos  0            /**< \brief (TC_COUNT16_CCBUF) Counter/Compare Buffer Value */
#define TC_COUNT16_CCBUF_CCBUF_Msk  (0xFFFFul << TC_COUNT16_CCBUF_CCBUF_Pos)
#define TC_COUNT16_CCBUF_CCBUF(value) (TC_COUNT16_CCBUF_CCBUF_Msk & ((value) << TC_COUNT16_CCBUF_CCBUF_Pos))
#define TC_COUNT16_CCBUF_MASK       0xFFFFul     /**< \brief (TC_COUNT16_CCBUF) MASK Register */

/* -------- TC_COUNT32_CCBUF : (TC Offset: 0x30) (R/W 32) COUNT32 COUNT32 Compare and Capture Buffer -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t CCBUF:32;         /*!< bit:  0..31  Counter/Compare Buffer Value       */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} TC_COUNT32_CCBUF_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT32_CCBUF_OFFSET     0x30         /**< \brief (TC_COUNT32_CCBUF offset) COUNT32 Compare and Capture Buffer */
#define TC_COUNT32_CCBUF_RESETVALUE 0x00000000ul /**< \brief (TC_COUNT32_CCBUF reset_value) COUNT32 Compare and Capture Buffer */

#define TC_COUNT32_CCBUF_CCBUF_Pos  0            /**< \brief (TC_COUNT32_CCBUF) Counter/Compare Buffer Value */
#define TC_COUNT32_CCBUF_CCBUF_Msk  (0xFFFFFFFFul << TC_COUNT32_CCBUF_CCBUF_Pos)
#define TC_COUNT32_CCBUF_CCBUF(value) (TC_COUNT32_CCBUF_CCBUF_Msk & ((value) << TC_COUNT32_CCBUF_CCBUF_Pos))
#define TC_COUNT32_CCBUF_MASK       0xFFFFFFFFul /**< \brief (TC_COUNT32_CCBUF) MASK Register */

/* -------- TC_COUNT8_CCBUF : (TC Offset: 0x30) (R/W  8) COUNT8 COUNT8 Compare and Capture Buffer -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  CCBUF:8;          /*!< bit:  0.. 7  Counter/Compare Buffer Value       */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} TC_COUNT8_CCBUF_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TC_COUNT8_CCBUF_OFFSET      0x30         /**< \brief (TC_COUNT8_CCBUF offset) COUNT8 Compare and Capture Buffer */
#define TC_COUNT8_CCBUF_RESETVALUE  0x00ul       /**< \brief (TC_COUNT8_CCBUF reset_value) COUNT8 Compare and Capture Buffer */

#define TC_COUNT8_CCBUF_CCBUF_Pos   0            /**< \brief (TC_COUNT8_CCBUF) Counter/Compare Buffer Value */
#define TC_COUNT8_CCBUF_CCBUF_Msk   (0xFFul << TC_COUNT8_CCBUF_CCBUF_Pos)
#define TC_COUNT8_CCBUF_CCBUF(value) (TC_COUNT8_CCBUF_CCBUF_Msk & ((value) << TC_COUNT8_CCBUF_CCBUF_Pos))
#define TC_COUNT8_CCBUF_MASK        0xFFul       /**< \brief (TC_COUNT8_CCBUF) MASK Register */

/** \brief TC_COUNT8 hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct { /* 8-bit Counter Mode */
  __IO TC_CTRLA_Type             CTRLA;       /**< \brief Offset: 0x00 (R/W 32) Control A */
  __IO TC_CTRLBCLR_Type          CTRLBCLR;    /**< \brief Offset: 0x04 (R/W  8) Control B Clear */
  __IO TC_CTRLBSET_Type          CTRLBSET;    /**< \brief Offset: 0x05 (R/W  8) Control B Set */
  __IO TC_EVCTRL_Type            EVCTRL;      /**< \brief Offset: 0x06 (R/W 16) Event Control */
  __IO TC_INTENCLR_Type          INTENCLR;    /**< \brief Offset: 0x08 (R/W  8) Interrupt Enable Clear */
  __IO TC_INTENSET_Type          INTENSET;    /**< \brief Offset: 0x09 (R/W  8) Interrupt Enable Set */
  __IO TC_INTFLAG_Type           INTFLAG;     /**< \brief Offset: 0x0A (R/W  8) Interrupt Flag Status and Clear */
  __IO TC_STATUS_Type            STATUS;      /**< \brief Offset: 0x0B (R/W  8) Status */
  __IO TC_WAVE_Type              WAVE;        /**< \brief Offset: 0x0C (R/W  8) Waveform Generation Control */
  __IO TC_DRVCTRL_Type           DRVCTRL;     /**< \brief Offset: 0x0D (R/W  8) Control C */
       RoReg8                    Reserved1[0x1];
  __IO TC_DBGCTRL_Type           DBGCTRL;     /**< \brief Offset: 0x0F (R/W  8) Debug Control */
  __I  TC_SYNCBUSY_Type          SYNCBUSY;    /**< \brief Offset: 0x10 (R/  32) Synchronization Status */
  __IO TC_COUNT8_COUNT_Type      COUNT;       /**< \brief Offset: 0x14 (R/W  8) COUNT8 Count */
       RoReg8                    Reserved2[0x6];
  __IO TC_COUNT8_PER_Type        PER;         /**< \brief Offset: 0x1B (R/W  8) COUNT8 Period */
  __IO TC_COUNT8_CC_Type         CC[2];       /**< \brief Offset: 0x1C (R/W  8) COUNT8 Compare and Capture */
       RoReg8                    Reserved3[0x11];
  __IO TC_COUNT8_PERBUF_Type     PERBUF;      /**< \brief Offset: 0x2F (R/W  8) COUNT8 Period Buffer */
  __IO TC_COUNT8_CCBUF_Type      CCBUF[2];    /**< \brief Offset: 0x30 (R/W  8) COUNT8 Compare and Capture Buffer */
} TcCount8;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/** \brief TC_COUNT16 hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct { /* 16-bit Counter Mode */
  __IO TC_CTRLA_Type             CTRLA;       /**< \brief Offset: 0x00 (R/W 32) Control A */
  __IO TC_CTRLBCLR_Type          CTRLBCLR;    /**< \brief Offset: 0x04 (R/W  8) Control B Clear */
  __IO TC_CTRLBSET_Type          CTRLBSET;    /**< \brief Offset: 0x05 (R/W  8) Control B Set */
  __IO TC_EVCTRL_Type            EVCTRL;      /**< \brief Offset: 0x06 (R/W 16) Event Control */
  __IO TC_INTENCLR_Type          INTENCLR;    /**< \brief Offset: 0x08 (R/W  8) Interrupt Enable Clear */
  __IO TC_INTENSET_Type          INTENSET;    /**< \brief Offset: 0x09 (R/W  8) Interrupt Enable Set */
  __IO TC_INTFLAG_Type           INTFLAG;     /**< \brief Offset: 0x0A (R/W  8) Interrupt Flag Status and Clear */
  __IO TC_STATUS_Type            STATUS;      /**< \brief Offset: 0x0B (R/W  8) Status */
  __IO TC_WAVE_Type              WAVE;        /**< \brief Offset: 0x0C (R/W  8) Waveform Generation Control */
  __IO TC_DRVCTRL_Type           DRVCTRL;     /**< \brief Offset: 0x0D (R/W  8) Control C */
       RoReg8                    Reserved1[0x1];
  __IO TC_DBGCTRL_Type           DBGCTRL;     /**< \brief Offset: 0x0F (R/W  8) Debug Control */
  __I  TC_SYNCBUSY_Type          SYNCBUSY;    /**< \brief Offset: 0x10 (R/  32) Synchronization Status */
  __IO TC_COUNT16_COUNT_Type     COUNT;       /**< \brief Offset: 0x14 (R/W 16) COUNT16 Count */
       RoReg8                    Reserved2[0x6];
  __IO TC_COUNT16_CC_Type        CC[2];       /**< \brief Offset: 0x1C (R/W 16) COUNT16 Compare and Capture */
       RoReg8                    Reserved3[0x10];
  __IO TC_COUNT16_CCBUF_Type     CCBUF[2];    /**< \brief Offset: 0x30 (R/W 16) COUNT16 Compare and Capture Buffer */
} TcCount16;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/** \brief TC_COUNT32 hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct { /* 32-bit Counter Mode */
  __IO TC_CTRLA_Type             CTRLA;       /**< \brief Offset: 0x00 (R/W 32) Control A */
  __IO TC_CTRLBCLR_Type          CTRLBCLR;    /**< \brief Offset: 0x04 (R/W  8) Control B Clear */
  __IO TC_CTRLBSET_Type          CTRLBSET;    /**< \brief Offset: 0x05 (R/W  8) Control B Set */
  __IO TC_EVCTRL_Type            EVCTRL;      /**< \brief Offset: 0x06 (R/W 16) Event Control */
  __IO TC_INTENCLR_Type          INTENCLR;    /**< \brief Offset: 0x08 (R/W  8) Interrupt Enable Clear */
  __IO TC_INTENSET_Type          INTENSET;    /**< \brief Offset: 0x09 (R/W  8) Interrupt Enable Set */
  __IO TC_INTFLAG_Type           INTFLAG;     /**< \brief Offset: 0x0A (R/W  8) Interrupt Flag Status and Clear */
  __IO TC_STATUS_Type            STATUS;      /**< \brief Offset: 0x0B (R/W  8) Status */
  __IO TC_WAVE_Type              WAVE;        /**< \brief Offset: 0x0C (R/W  8) Waveform Generation Control */
  __IO TC_DRVCTRL_Type           DRVCTRL;     /**< \brief Offset: 0x0D (R/W  8) Control C */
       RoReg8                    Reserved1[0x1];
  __IO TC_DBGCTRL_Type           DBGCTRL;     /**< \brief Offset: 0x0F (R/W  8) Debug Control */
  __I  TC_SYNCBUSY_Type          SYNCBUSY;    /**< \brief Offset: 0x10 (R/  32) Synchronization Status */
  __IO TC_COUNT32_COUNT_Type     COUNT;       /**< \brief Offset: 0x14 (R/W 32) COUNT32 Count */
       RoReg8                    Reserved2[0x4];
  __IO TC_COUNT32_CC_Type        CC[2];       /**< \brief Offset: 0x1C (R/W 32) COUNT32 Compare and Capture */
       RoReg8                    Reserved3[0xC];
  __IO TC_COUNT32_CCBUF_Type     CCBUF[2];    /**< \brief Offset: 0x30 (R/W 32) COUNT32 Compare and Capture Buffer */
} TcCount32;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
       TcCount8                  COUNT8;      /**< \brief Offset: 0x00 8-bit Counter Mode */
       TcCount16                 COUNT16;     /**< \brief Offset: 0x00 16-bit Counter Mode */
       TcCount32                 COUNT32;     /**< \brief Offset: 0x00 32-bit Counter Mode */
} Tc;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/*@}*/

#endif /* _SAMR30_TC_COMPONENT_ */
#ifndef _SAMR30_MCLK_COMPONENT_
#define _SAMR30_MCLK_COMPONENT_

/* ========================================================================== */
/**  SOFTWARE API DEFINITION FOR MCLK */
/* ========================================================================== */
/** \addtogroup SAMR30_MCLK Main Clock */
/*@{*/

#define MCLK_U2234
#define REV_MCLK                    0x101

/* -------- MCLK_CTRLA : (MCLK Offset: 0x00) (R/W  8) Control A -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  uint8_t reg;                 /*!< Type      used for register access              */
} MCLK_CTRLA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_CTRLA_OFFSET           0x00         /**< \brief (MCLK_CTRLA offset) Control A */
#define MCLK_CTRLA_RESETVALUE       0x00ul       /**< \brief (MCLK_CTRLA reset_value) Control A */
#define MCLK_CTRLA_MASK             0x00ul       /**< \brief (MCLK_CTRLA) MASK Register */

/* -------- MCLK_INTENCLR : (MCLK Offset: 0x01) (R/W  8) Interrupt Enable Clear -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  CKRDY:1;          /*!< bit:      0  Clock Ready Interrupt Enable       */
    uint8_t  :7;               /*!< bit:  1.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} MCLK_INTENCLR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_INTENCLR_OFFSET        0x01         /**< \brief (MCLK_INTENCLR offset) Interrupt Enable Clear */
#define MCLK_INTENCLR_RESETVALUE    0x00ul       /**< \brief (MCLK_INTENCLR reset_value) Interrupt Enable Clear */

#define MCLK_INTENCLR_CKRDY_Pos     0            /**< \brief (MCLK_INTENCLR) Clock Ready Interrupt Enable */
#define MCLK_INTENCLR_CKRDY         (0x1ul << MCLK_INTENCLR_CKRDY_Pos)
#define MCLK_INTENCLR_MASK          0x01ul       /**< \brief (MCLK_INTENCLR) MASK Register */

/* -------- MCLK_INTENSET : (MCLK Offset: 0x02) (R/W  8) Interrupt Enable Set -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  CKRDY:1;          /*!< bit:      0  Clock Ready Interrupt Enable       */
    uint8_t  :7;               /*!< bit:  1.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} MCLK_INTENSET_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_INTENSET_OFFSET        0x02         /**< \brief (MCLK_INTENSET offset) Interrupt Enable Set */
#define MCLK_INTENSET_RESETVALUE    0x00ul       /**< \brief (MCLK_INTENSET reset_value) Interrupt Enable Set */

#define MCLK_INTENSET_CKRDY_Pos     0            /**< \brief (MCLK_INTENSET) Clock Ready Interrupt Enable */
#define MCLK_INTENSET_CKRDY         (0x1ul << MCLK_INTENSET_CKRDY_Pos)
#define MCLK_INTENSET_MASK          0x01ul       /**< \brief (MCLK_INTENSET) MASK Register */

/* -------- MCLK_INTFLAG : (MCLK Offset: 0x03) (R/W  8) Interrupt Flag Status and Clear -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { // __I to avoid read-modify-write on write-to-clear register
  struct {
    __I uint8_t  CKRDY:1;          /*!< bit:      0  Clock Ready                        */
    __I uint8_t  :7;               /*!< bit:  1.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} MCLK_INTFLAG_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_INTFLAG_OFFSET         0x03         /**< \brief (MCLK_INTFLAG offset) Interrupt Flag Status and Clear */
#define MCLK_INTFLAG_RESETVALUE     0x01ul       /**< \brief (MCLK_INTFLAG reset_value) Interrupt Flag Status and Clear */

#define MCLK_INTFLAG_CKRDY_Pos      0            /**< \brief (MCLK_INTFLAG) Clock Ready */
#define MCLK_INTFLAG_CKRDY          (0x1ul << MCLK_INTFLAG_CKRDY_Pos)
#define MCLK_INTFLAG_MASK           0x01ul       /**< \brief (MCLK_INTFLAG) MASK Register */

/* -------- MCLK_CPUDIV : (MCLK Offset: 0x04) (R/W  8) CPU Clock Division -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  CPUDIV:8;         /*!< bit:  0.. 7  CPU Clock Division Factor          */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} MCLK_CPUDIV_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_CPUDIV_OFFSET          0x04         /**< \brief (MCLK_CPUDIV offset) CPU Clock Division */
#define MCLK_CPUDIV_RESETVALUE      0x01ul       /**< \brief (MCLK_CPUDIV reset_value) CPU Clock Division */

#define MCLK_CPUDIV_CPUDIV_Pos      0            /**< \brief (MCLK_CPUDIV) CPU Clock Division Factor */
#define MCLK_CPUDIV_CPUDIV_Msk      (0xFFul << MCLK_CPUDIV_CPUDIV_Pos)
#define MCLK_CPUDIV_CPUDIV(value)   (MCLK_CPUDIV_CPUDIV_Msk & ((value) << MCLK_CPUDIV_CPUDIV_Pos))
#define   MCLK_CPUDIV_CPUDIV_DIV1_Val     0x1ul  /**< \brief (MCLK_CPUDIV) Divide by 1 */
#define   MCLK_CPUDIV_CPUDIV_DIV2_Val     0x2ul  /**< \brief (MCLK_CPUDIV) Divide by 2 */
#define   MCLK_CPUDIV_CPUDIV_DIV4_Val     0x4ul  /**< \brief (MCLK_CPUDIV) Divide by 4 */
#define   MCLK_CPUDIV_CPUDIV_DIV8_Val     0x8ul  /**< \brief (MCLK_CPUDIV) Divide by 8 */
#define   MCLK_CPUDIV_CPUDIV_DIV16_Val    0x10ul  /**< \brief (MCLK_CPUDIV) Divide by 16 */
#define   MCLK_CPUDIV_CPUDIV_DIV32_Val    0x20ul  /**< \brief (MCLK_CPUDIV) Divide by 32 */
#define   MCLK_CPUDIV_CPUDIV_DIV64_Val    0x40ul  /**< \brief (MCLK_CPUDIV) Divide by 64 */
#define   MCLK_CPUDIV_CPUDIV_DIV128_Val   0x80ul  /**< \brief (MCLK_CPUDIV) Divide by 128 */
#define MCLK_CPUDIV_CPUDIV_DIV1     (MCLK_CPUDIV_CPUDIV_DIV1_Val   << MCLK_CPUDIV_CPUDIV_Pos)
#define MCLK_CPUDIV_CPUDIV_DIV2     (MCLK_CPUDIV_CPUDIV_DIV2_Val   << MCLK_CPUDIV_CPUDIV_Pos)
#define MCLK_CPUDIV_CPUDIV_DIV4     (MCLK_CPUDIV_CPUDIV_DIV4_Val   << MCLK_CPUDIV_CPUDIV_Pos)
#define MCLK_CPUDIV_CPUDIV_DIV8     (MCLK_CPUDIV_CPUDIV_DIV8_Val   << MCLK_CPUDIV_CPUDIV_Pos)
#define MCLK_CPUDIV_CPUDIV_DIV16    (MCLK_CPUDIV_CPUDIV_DIV16_Val  << MCLK_CPUDIV_CPUDIV_Pos)
#define MCLK_CPUDIV_CPUDIV_DIV32    (MCLK_CPUDIV_CPUDIV_DIV32_Val  << MCLK_CPUDIV_CPUDIV_Pos)
#define MCLK_CPUDIV_CPUDIV_DIV64    (MCLK_CPUDIV_CPUDIV_DIV64_Val  << MCLK_CPUDIV_CPUDIV_Pos)
#define MCLK_CPUDIV_CPUDIV_DIV128   (MCLK_CPUDIV_CPUDIV_DIV128_Val << MCLK_CPUDIV_CPUDIV_Pos)
#define MCLK_CPUDIV_MASK            0xFFul       /**< \brief (MCLK_CPUDIV) MASK Register */

/* -------- MCLK_LPDIV : (MCLK Offset: 0x05) (R/W  8) Low-Power Clock Division -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  LPDIV:8;          /*!< bit:  0.. 7  Low-Power Clock Division Factor    */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} MCLK_LPDIV_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_LPDIV_OFFSET           0x05         /**< \brief (MCLK_LPDIV offset) Low-Power Clock Division */
#define MCLK_LPDIV_RESETVALUE       0x01ul       /**< \brief (MCLK_LPDIV reset_value) Low-Power Clock Division */

#define MCLK_LPDIV_LPDIV_Pos        0            /**< \brief (MCLK_LPDIV) Low-Power Clock Division Factor */
#define MCLK_LPDIV_LPDIV_Msk        (0xFFul << MCLK_LPDIV_LPDIV_Pos)
#define MCLK_LPDIV_LPDIV(value)     (MCLK_LPDIV_LPDIV_Msk & ((value) << MCLK_LPDIV_LPDIV_Pos))
#define   MCLK_LPDIV_LPDIV_DIV1_Val       0x1ul  /**< \brief (MCLK_LPDIV) Divide by 1 */
#define   MCLK_LPDIV_LPDIV_DIV2_Val       0x2ul  /**< \brief (MCLK_LPDIV) Divide by 2 */
#define   MCLK_LPDIV_LPDIV_DIV4_Val       0x4ul  /**< \brief (MCLK_LPDIV) Divide by 4 */
#define   MCLK_LPDIV_LPDIV_DIV8_Val       0x8ul  /**< \brief (MCLK_LPDIV) Divide by 8 */
#define   MCLK_LPDIV_LPDIV_DIV16_Val      0x10ul  /**< \brief (MCLK_LPDIV) Divide by 16 */
#define   MCLK_LPDIV_LPDIV_DIV32_Val      0x20ul  /**< \brief (MCLK_LPDIV) Divide by 32 */
#define   MCLK_LPDIV_LPDIV_DIV64_Val      0x40ul  /**< \brief (MCLK_LPDIV) Divide by 64 */
#define   MCLK_LPDIV_LPDIV_DIV128_Val     0x80ul  /**< \brief (MCLK_LPDIV) Divide by 128 */
#define MCLK_LPDIV_LPDIV_DIV1       (MCLK_LPDIV_LPDIV_DIV1_Val     << MCLK_LPDIV_LPDIV_Pos)
#define MCLK_LPDIV_LPDIV_DIV2       (MCLK_LPDIV_LPDIV_DIV2_Val     << MCLK_LPDIV_LPDIV_Pos)
#define MCLK_LPDIV_LPDIV_DIV4       (MCLK_LPDIV_LPDIV_DIV4_Val     << MCLK_LPDIV_LPDIV_Pos)
#define MCLK_LPDIV_LPDIV_DIV8       (MCLK_LPDIV_LPDIV_DIV8_Val     << MCLK_LPDIV_LPDIV_Pos)
#define MCLK_LPDIV_LPDIV_DIV16      (MCLK_LPDIV_LPDIV_DIV16_Val    << MCLK_LPDIV_LPDIV_Pos)
#define MCLK_LPDIV_LPDIV_DIV32      (MCLK_LPDIV_LPDIV_DIV32_Val    << MCLK_LPDIV_LPDIV_Pos)
#define MCLK_LPDIV_LPDIV_DIV64      (MCLK_LPDIV_LPDIV_DIV64_Val    << MCLK_LPDIV_LPDIV_Pos)
#define MCLK_LPDIV_LPDIV_DIV128     (MCLK_LPDIV_LPDIV_DIV128_Val   << MCLK_LPDIV_LPDIV_Pos)
#define MCLK_LPDIV_MASK             0xFFul       /**< \brief (MCLK_LPDIV) MASK Register */

/* -------- MCLK_BUPDIV : (MCLK Offset: 0x06) (R/W  8) Backup Clock Division -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  BUPDIV:8;         /*!< bit:  0.. 7  Backup Clock Division Factor       */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} MCLK_BUPDIV_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_BUPDIV_OFFSET          0x06         /**< \brief (MCLK_BUPDIV offset) Backup Clock Division */
#define MCLK_BUPDIV_RESETVALUE      0x01ul       /**< \brief (MCLK_BUPDIV reset_value) Backup Clock Division */

#define MCLK_BUPDIV_BUPDIV_Pos      0            /**< \brief (MCLK_BUPDIV) Backup Clock Division Factor */
#define MCLK_BUPDIV_BUPDIV_Msk      (0xFFul << MCLK_BUPDIV_BUPDIV_Pos)
#define MCLK_BUPDIV_BUPDIV(value)   (MCLK_BUPDIV_BUPDIV_Msk & ((value) << MCLK_BUPDIV_BUPDIV_Pos))
#define   MCLK_BUPDIV_BUPDIV_DIV1_Val     0x1ul  /**< \brief (MCLK_BUPDIV) Divide by 1 */
#define   MCLK_BUPDIV_BUPDIV_DIV2_Val     0x2ul  /**< \brief (MCLK_BUPDIV) Divide by 2 */
#define   MCLK_BUPDIV_BUPDIV_DIV4_Val     0x4ul  /**< \brief (MCLK_BUPDIV) Divide by 4 */
#define   MCLK_BUPDIV_BUPDIV_DIV8_Val     0x8ul  /**< \brief (MCLK_BUPDIV) Divide by 8 */
#define   MCLK_BUPDIV_BUPDIV_DIV16_Val    0x10ul  /**< \brief (MCLK_BUPDIV) Divide by 16 */
#define   MCLK_BUPDIV_BUPDIV_DIV32_Val    0x20ul  /**< \brief (MCLK_BUPDIV) Divide by 32 */
#define   MCLK_BUPDIV_BUPDIV_DIV64_Val    0x40ul  /**< \brief (MCLK_BUPDIV) Divide by 64 */
#define   MCLK_BUPDIV_BUPDIV_DIV128_Val   0x80ul  /**< \brief (MCLK_BUPDIV) Divide by 128 */
#define MCLK_BUPDIV_BUPDIV_DIV1     (MCLK_BUPDIV_BUPDIV_DIV1_Val   << MCLK_BUPDIV_BUPDIV_Pos)
#define MCLK_BUPDIV_BUPDIV_DIV2     (MCLK_BUPDIV_BUPDIV_DIV2_Val   << MCLK_BUPDIV_BUPDIV_Pos)
#define MCLK_BUPDIV_BUPDIV_DIV4     (MCLK_BUPDIV_BUPDIV_DIV4_Val   << MCLK_BUPDIV_BUPDIV_Pos)
#define MCLK_BUPDIV_BUPDIV_DIV8     (MCLK_BUPDIV_BUPDIV_DIV8_Val   << MCLK_BUPDIV_BUPDIV_Pos)
#define MCLK_BUPDIV_BUPDIV_DIV16    (MCLK_BUPDIV_BUPDIV_DIV16_Val  << MCLK_BUPDIV_BUPDIV_Pos)
#define MCLK_BUPDIV_BUPDIV_DIV32    (MCLK_BUPDIV_BUPDIV_DIV32_Val  << MCLK_BUPDIV_BUPDIV_Pos)
#define MCLK_BUPDIV_BUPDIV_DIV64    (MCLK_BUPDIV_BUPDIV_DIV64_Val  << MCLK_BUPDIV_BUPDIV_Pos)
#define MCLK_BUPDIV_BUPDIV_DIV128   (MCLK_BUPDIV_BUPDIV_DIV128_Val << MCLK_BUPDIV_BUPDIV_Pos)
#define MCLK_BUPDIV_MASK            0xFFul       /**< \brief (MCLK_BUPDIV) MASK Register */

/* -------- MCLK_AHBMASK : (MCLK Offset: 0x10) (R/W 32) AHB Mask -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t HPB0_:1;          /*!< bit:      0  HPB0 AHB Clock Mask                */
    uint32_t HPB1_:1;          /*!< bit:      1  HPB1 AHB Clock Mask                */
    uint32_t HPB2_:1;          /*!< bit:      2  HPB2 AHB Clock Mask                */
    uint32_t HPB3_:1;          /*!< bit:      3  HPB3 AHB Clock Mask                */
    uint32_t HPB4_:1;          /*!< bit:      4  HPB4 AHB Clock Mask                */
    uint32_t DSU_:1;           /*!< bit:      5  DSU AHB Clock Mask                 */
    uint32_t :2;               /*!< bit:  6.. 7  Reserved                           */
    uint32_t NVMCTRL_:1;       /*!< bit:      8  NVMCTRL AHB Clock Mask             */
    uint32_t HSRAM_:1;         /*!< bit:      9  HSRAM AHB Clock Mask               */
    uint32_t LPRAM_:1;         /*!< bit:     10  LPRAM AHB Clock Mask               */
    uint32_t DMAC_:1;          /*!< bit:     11  DMAC AHB Clock Mask                */
    uint32_t USB_:1;           /*!< bit:     12  USB AHB Clock Mask                 */
    uint32_t :1;               /*!< bit:     13  Reserved                           */
    uint32_t PAC_:1;           /*!< bit:     14  PAC AHB Clock Mask                 */
    uint32_t NVMCTRL_PICACHU_:1; /*!< bit:     15  NVMCTRL_PICACHU AHB Clock Mask     */
    uint32_t L2HBRIDGES_H_:1;  /*!< bit:     16  L2HBRIDGES_H AHB Clock Mask        */
    uint32_t H2LBRIDGES_H_:1;  /*!< bit:     17  H2LBRIDGES_H AHB Clock Mask        */
    uint32_t HMCRAMCHS_AHBSETUPKEEPER_:1; /*!< bit:     18  HMCRAMCHS_AHBSETUPKEEPER AHB Clock Mask */
    uint32_t HMCRAMCHS_HMATRIXLP2HMCRAMCHSBRIDGE_:1; /*!< bit:     19  HMCRAMCHS_HMATRIXLP2HMCRAMCHSBRIDGE AHB Clock Mask */
    uint32_t :12;              /*!< bit: 20..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCLK_AHBMASK_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_AHBMASK_OFFSET         0x10         /**< \brief (MCLK_AHBMASK offset) AHB Mask */
#define MCLK_AHBMASK_RESETVALUE     0x000FFFFFul /**< \brief (MCLK_AHBMASK reset_value) AHB Mask */

#define MCLK_AHBMASK_HPB0_Pos       0            /**< \brief (MCLK_AHBMASK) HPB0 AHB Clock Mask */
#define MCLK_AHBMASK_HPB0           (0x1ul << MCLK_AHBMASK_HPB0_Pos)
#define MCLK_AHBMASK_HPB1_Pos       1            /**< \brief (MCLK_AHBMASK) HPB1 AHB Clock Mask */
#define MCLK_AHBMASK_HPB1           (0x1ul << MCLK_AHBMASK_HPB1_Pos)
#define MCLK_AHBMASK_HPB2_Pos       2            /**< \brief (MCLK_AHBMASK) HPB2 AHB Clock Mask */
#define MCLK_AHBMASK_HPB2           (0x1ul << MCLK_AHBMASK_HPB2_Pos)
#define MCLK_AHBMASK_HPB3_Pos       3            /**< \brief (MCLK_AHBMASK) HPB3 AHB Clock Mask */
#define MCLK_AHBMASK_HPB3           (0x1ul << MCLK_AHBMASK_HPB3_Pos)
#define MCLK_AHBMASK_HPB4_Pos       4            /**< \brief (MCLK_AHBMASK) HPB4 AHB Clock Mask */
#define MCLK_AHBMASK_HPB4           (0x1ul << MCLK_AHBMASK_HPB4_Pos)
#define MCLK_AHBMASK_DSU_Pos        5            /**< \brief (MCLK_AHBMASK) DSU AHB Clock Mask */
#define MCLK_AHBMASK_DSU            (0x1ul << MCLK_AHBMASK_DSU_Pos)
#define MCLK_AHBMASK_NVMCTRL_Pos    8            /**< \brief (MCLK_AHBMASK) NVMCTRL AHB Clock Mask */
#define MCLK_AHBMASK_NVMCTRL        (0x1ul << MCLK_AHBMASK_NVMCTRL_Pos)
#define MCLK_AHBMASK_HSRAM_Pos      9            /**< \brief (MCLK_AHBMASK) HSRAM AHB Clock Mask */
#define MCLK_AHBMASK_HSRAM          (0x1ul << MCLK_AHBMASK_HSRAM_Pos)
#define MCLK_AHBMASK_LPRAM_Pos      10           /**< \brief (MCLK_AHBMASK) LPRAM AHB Clock Mask */
#define MCLK_AHBMASK_LPRAM          (0x1ul << MCLK_AHBMASK_LPRAM_Pos)
#define MCLK_AHBMASK_DMAC_Pos       11           /**< \brief (MCLK_AHBMASK) DMAC AHB Clock Mask */
#define MCLK_AHBMASK_DMAC           (0x1ul << MCLK_AHBMASK_DMAC_Pos)
#define MCLK_AHBMASK_USB_Pos        12           /**< \brief (MCLK_AHBMASK) USB AHB Clock Mask */
#define MCLK_AHBMASK_USB            (0x1ul << MCLK_AHBMASK_USB_Pos)
#define MCLK_AHBMASK_PAC_Pos        14           /**< \brief (MCLK_AHBMASK) PAC AHB Clock Mask */
#define MCLK_AHBMASK_PAC            (0x1ul << MCLK_AHBMASK_PAC_Pos)
#define MCLK_AHBMASK_NVMCTRL_PICACHU_Pos 15           /**< \brief (MCLK_AHBMASK) NVMCTRL_PICACHU AHB Clock Mask */
#define MCLK_AHBMASK_NVMCTRL_PICACHU (0x1ul << MCLK_AHBMASK_NVMCTRL_PICACHU_Pos)
#define MCLK_AHBMASK_L2HBRIDGES_H_Pos 16           /**< \brief (MCLK_AHBMASK) L2HBRIDGES_H AHB Clock Mask */
#define MCLK_AHBMASK_L2HBRIDGES_H   (0x1ul << MCLK_AHBMASK_L2HBRIDGES_H_Pos)
#define MCLK_AHBMASK_H2LBRIDGES_H_Pos 17           /**< \brief (MCLK_AHBMASK) H2LBRIDGES_H AHB Clock Mask */
#define MCLK_AHBMASK_H2LBRIDGES_H   (0x1ul << MCLK_AHBMASK_H2LBRIDGES_H_Pos)
#define MCLK_AHBMASK_HMCRAMCHS_AHBSETUPKEEPER_Pos 18           /**< \brief (MCLK_AHBMASK) HMCRAMCHS_AHBSETUPKEEPER AHB Clock Mask */
#define MCLK_AHBMASK_HMCRAMCHS_AHBSETUPKEEPER (0x1ul << MCLK_AHBMASK_HMCRAMCHS_AHBSETUPKEEPER_Pos)
#define MCLK_AHBMASK_HMCRAMCHS_HMATRIXLP2HMCRAMCHSBRIDGE_Pos 19           /**< \brief (MCLK_AHBMASK) HMCRAMCHS_HMATRIXLP2HMCRAMCHSBRIDGE AHB Clock Mask */
#define MCLK_AHBMASK_HMCRAMCHS_HMATRIXLP2HMCRAMCHSBRIDGE (0x1ul << MCLK_AHBMASK_HMCRAMCHS_HMATRIXLP2HMCRAMCHSBRIDGE_Pos)
#define MCLK_AHBMASK_MASK           0x000FDF3Ful /**< \brief (MCLK_AHBMASK) MASK Register */

/* -------- MCLK_APBAMASK : (MCLK Offset: 0x14) (R/W 32) APBA Mask -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t PM_:1;            /*!< bit:      0  PM APB Clock Enable                */
    uint32_t MCLK_:1;          /*!< bit:      1  MCLK APB Clock Enable              */
    uint32_t RSTC_:1;          /*!< bit:      2  RSTC APB Clock Enable              */
    uint32_t OSCCTRL_:1;       /*!< bit:      3  OSCCTRL APB Clock Enable           */
    uint32_t OSC32KCTRL_:1;    /*!< bit:      4  OSC32KCTRL APB Clock Enable        */
    uint32_t SUPC_:1;          /*!< bit:      5  SUPC APB Clock Enable              */
    uint32_t GCLK_:1;          /*!< bit:      6  GCLK APB Clock Enable              */
    uint32_t WDT_:1;           /*!< bit:      7  WDT APB Clock Enable               */
    uint32_t RTC_:1;           /*!< bit:      8  RTC APB Clock Enable               */
    uint32_t EIC_:1;           /*!< bit:      9  EIC APB Clock Enable               */
    uint32_t PORT_:1;          /*!< bit:     10  PORT APB Clock Enable              */
    uint32_t TAL_:1;           /*!< bit:     11  TAL APB Clock Enable               */
    uint32_t :20;              /*!< bit: 12..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCLK_APBAMASK_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_APBAMASK_OFFSET        0x14         /**< \brief (MCLK_APBAMASK offset) APBA Mask */
#define MCLK_APBAMASK_RESETVALUE    0x00001FFFul /**< \brief (MCLK_APBAMASK reset_value) APBA Mask */

#define MCLK_APBAMASK_PM_Pos        0            /**< \brief (MCLK_APBAMASK) PM APB Clock Enable */
#define MCLK_APBAMASK_PM            (0x1ul << MCLK_APBAMASK_PM_Pos)
#define MCLK_APBAMASK_MCLK_Pos      1            /**< \brief (MCLK_APBAMASK) MCLK APB Clock Enable */
#define MCLK_APBAMASK_MCLK          (0x1ul << MCLK_APBAMASK_MCLK_Pos)
#define MCLK_APBAMASK_RSTC_Pos      2            /**< \brief (MCLK_APBAMASK) RSTC APB Clock Enable */
#define MCLK_APBAMASK_RSTC          (0x1ul << MCLK_APBAMASK_RSTC_Pos)
#define MCLK_APBAMASK_OSCCTRL_Pos   3            /**< \brief (MCLK_APBAMASK) OSCCTRL APB Clock Enable */
#define MCLK_APBAMASK_OSCCTRL       (0x1ul << MCLK_APBAMASK_OSCCTRL_Pos)
#define MCLK_APBAMASK_OSC32KCTRL_Pos 4            /**< \brief (MCLK_APBAMASK) OSC32KCTRL APB Clock Enable */
#define MCLK_APBAMASK_OSC32KCTRL    (0x1ul << MCLK_APBAMASK_OSC32KCTRL_Pos)
#define MCLK_APBAMASK_SUPC_Pos      5            /**< \brief (MCLK_APBAMASK) SUPC APB Clock Enable */
#define MCLK_APBAMASK_SUPC          (0x1ul << MCLK_APBAMASK_SUPC_Pos)
#define MCLK_APBAMASK_GCLK_Pos      6            /**< \brief (MCLK_APBAMASK) GCLK APB Clock Enable */
#define MCLK_APBAMASK_GCLK          (0x1ul << MCLK_APBAMASK_GCLK_Pos)
#define MCLK_APBAMASK_WDT_Pos       7            /**< \brief (MCLK_APBAMASK) WDT APB Clock Enable */
#define MCLK_APBAMASK_WDT           (0x1ul << MCLK_APBAMASK_WDT_Pos)
#define MCLK_APBAMASK_RTC_Pos       8            /**< \brief (MCLK_APBAMASK) RTC APB Clock Enable */
#define MCLK_APBAMASK_RTC           (0x1ul << MCLK_APBAMASK_RTC_Pos)
#define MCLK_APBAMASK_EIC_Pos       9            /**< \brief (MCLK_APBAMASK) EIC APB Clock Enable */
#define MCLK_APBAMASK_EIC           (0x1ul << MCLK_APBAMASK_EIC_Pos)
#define MCLK_APBAMASK_PORT_Pos      10           /**< \brief (MCLK_APBAMASK) PORT APB Clock Enable */
#define MCLK_APBAMASK_PORT          (0x1ul << MCLK_APBAMASK_PORT_Pos)
#define MCLK_APBAMASK_TAL_Pos       11           /**< \brief (MCLK_APBAMASK) TAL APB Clock Enable */
#define MCLK_APBAMASK_TAL           (0x1ul << MCLK_APBAMASK_TAL_Pos)
#define MCLK_APBAMASK_MASK          0x00000FFFul /**< \brief (MCLK_APBAMASK) MASK Register */

/* -------- MCLK_APBBMASK : (MCLK Offset: 0x18) (R/W 32) APBB Mask -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t USB_:1;           /*!< bit:      0  USB APB Clock Enable               */
    uint32_t DSU_:1;           /*!< bit:      1  DSU APB Clock Enable               */
    uint32_t NVMCTRL_:1;       /*!< bit:      2  NVMCTRL APB Clock Enable           */
    uint32_t :29;              /*!< bit:  3..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCLK_APBBMASK_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_APBBMASK_OFFSET        0x18         /**< \brief (MCLK_APBBMASK offset) APBB Mask */
#define MCLK_APBBMASK_RESETVALUE    0x00000017ul /**< \brief (MCLK_APBBMASK reset_value) APBB Mask */

#define MCLK_APBBMASK_USB_Pos       0            /**< \brief (MCLK_APBBMASK) USB APB Clock Enable */
#define MCLK_APBBMASK_USB           (0x1ul << MCLK_APBBMASK_USB_Pos)
#define MCLK_APBBMASK_DSU_Pos       1            /**< \brief (MCLK_APBBMASK) DSU APB Clock Enable */
#define MCLK_APBBMASK_DSU           (0x1ul << MCLK_APBBMASK_DSU_Pos)
#define MCLK_APBBMASK_NVMCTRL_Pos   2            /**< \brief (MCLK_APBBMASK) NVMCTRL APB Clock Enable */
#define MCLK_APBBMASK_NVMCTRL       (0x1ul << MCLK_APBBMASK_NVMCTRL_Pos)
#define MCLK_APBBMASK_MASK          0x00000007ul /**< \brief (MCLK_APBBMASK) MASK Register */

/* -------- MCLK_APBCMASK : (MCLK Offset: 0x1C) (R/W 32) APBC Mask -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t SERCOM0_:1;       /*!< bit:      0  SERCOM0 APB Clock Enable           */
    uint32_t SERCOM1_:1;       /*!< bit:      1  SERCOM1 APB Clock Enable           */
    uint32_t SERCOM2_:1;       /*!< bit:      2  SERCOM2 APB Clock Enable           */
    uint32_t SERCOM3_:1;       /*!< bit:      3  SERCOM3 APB Clock Enable           */
    uint32_t SERCOM4_:1;       /*!< bit:      4  SERCOM4 APB Clock Enable           */
    uint32_t TCC0_:1;          /*!< bit:      5  TCC0 APB Clock Enable              */
    uint32_t TCC1_:1;          /*!< bit:      6  TCC1 APB Clock Enable              */
    uint32_t TCC2_:1;          /*!< bit:      7  TCC2 APB Clock Enable              */
    uint32_t TC0_:1;           /*!< bit:      8  TC0 APB Clock Enable               */
    uint32_t TC1_:1;           /*!< bit:      9  TC1 APB Clock Enable               */
    uint32_t :5;               /*!< bit: 10.. 14  Reserved                           */
	uint32_t RFCTRL_:1;        /*!< bit:      15  TC1 APB Clock Enable               */
	uint32_t :16;               /*!< bit: 16..32  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCLK_APBCMASK_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_APBCMASK_OFFSET        0x1C         /**< \brief (MCLK_APBCMASK offset) APBC Mask */
#define MCLK_APBCMASK_RESETVALUE    0x00007FFFul /**< \brief (MCLK_APBCMASK reset_value) APBC Mask */

#define MCLK_APBCMASK_SERCOM0_Pos   0            /**< \brief (MCLK_APBCMASK) SERCOM0 APB Clock Enable */
#define MCLK_APBCMASK_SERCOM0       (0x1ul << MCLK_APBCMASK_SERCOM0_Pos)
#define MCLK_APBCMASK_SERCOM1_Pos   1            /**< \brief (MCLK_APBCMASK) SERCOM1 APB Clock Enable */
#define MCLK_APBCMASK_SERCOM1       (0x1ul << MCLK_APBCMASK_SERCOM1_Pos)
#define MCLK_APBCMASK_SERCOM2_Pos   2            /**< \brief (MCLK_APBCMASK) SERCOM2 APB Clock Enable */
#define MCLK_APBCMASK_SERCOM2       (0x1ul << MCLK_APBCMASK_SERCOM2_Pos)
#define MCLK_APBCMASK_SERCOM3_Pos   3            /**< \brief (MCLK_APBCMASK) SERCOM3 APB Clock Enable */
#define MCLK_APBCMASK_SERCOM3       (0x1ul << MCLK_APBCMASK_SERCOM3_Pos)
#define MCLK_APBCMASK_SERCOM4_Pos   4            /**< \brief (MCLK_APBCMASK) SERCOM4 APB Clock Enable */
#define MCLK_APBCMASK_SERCOM4       (0x1ul << MCLK_APBCMASK_SERCOM4_Pos)
#define MCLK_APBCMASK_TCC0_Pos      5            /**< \brief (MCLK_APBCMASK) TCC0 APB Clock Enable */
#define MCLK_APBCMASK_TCC0          (0x1ul << MCLK_APBCMASK_TCC0_Pos)
#define MCLK_APBCMASK_TCC1_Pos      6            /**< \brief (MCLK_APBCMASK) TCC1 APB Clock Enable */
#define MCLK_APBCMASK_TCC1          (0x1ul << MCLK_APBCMASK_TCC1_Pos)
#define MCLK_APBCMASK_TCC2_Pos      7            /**< \brief (MCLK_APBCMASK) TCC2 APB Clock Enable */
#define MCLK_APBCMASK_TCC2          (0x1ul << MCLK_APBCMASK_TCC2_Pos)
#define MCLK_APBCMASK_TC0_Pos       8            /**< \brief (MCLK_APBCMASK) TC0 APB Clock Enable */
#define MCLK_APBCMASK_TC0           (0x1ul << MCLK_APBCMASK_TC0_Pos)
#define MCLK_APBCMASK_TC1_Pos       9            /**< \brief (MCLK_APBCMASK) TC1 APB Clock Enable */
#define MCLK_APBCMASK_TC1           (0x1ul << MCLK_APBCMASK_TC1_Pos)
#define MCLK_APBCMASK_RFCTRL_Pos    15            /**< \brief (MCLK_APBCMASK) TC1 APB Clock Enable */
#define MCLK_APBCMASK_RFCTRL        (0x1ul << MCLK_APBCMASK_RFCTRL_Pos)
#define MCLK_APBCMASK_MASK          0x000003FFul /**< \brief (MCLK_APBCMASK) MASK Register */

/* -------- MCLK_APBDMASK : (MCLK Offset: 0x20) (R/W 32) APBD Mask -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t EVSYS_:1;         /*!< bit:      0  EVSYS APB Clock Enable             */
    uint32_t SERCOM5_:1;       /*!< bit:      1  SERCOM5 APB Clock Enable           */
    uint32_t TC4_:1;           /*!< bit:      2  TC4 APB Clock Enable               */
    uint32_t ADC_:1;           /*!< bit:      3  ADC APB Clock Enable               */
    uint32_t AC_:1;            /*!< bit:      4  AC APB Clock Enable                */
    uint32_t PTC_:1;           /*!< bit:      5  PTC APB Clock Enable               */
    uint32_t :1;               /*!< bit:      6  Reserved                           */
    uint32_t CCL_:1;           /*!< bit:      7  CCL APB Clock Enable               */
    uint32_t :24;              /*!< bit:  8..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCLK_APBDMASK_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_APBDMASK_OFFSET        0x20         /**< \brief (MCLK_APBDMASK offset) APBD Mask */
#define MCLK_APBDMASK_RESETVALUE    0x000000FFul /**< \brief (MCLK_APBDMASK reset_value) APBD Mask */

#define MCLK_APBDMASK_EVSYS_Pos     0            /**< \brief (MCLK_APBDMASK) EVSYS APB Clock Enable */
#define MCLK_APBDMASK_EVSYS         (0x1ul << MCLK_APBDMASK_EVSYS_Pos)
#define MCLK_APBDMASK_SERCOM5_Pos   1            /**< \brief (MCLK_APBDMASK) SERCOM5 APB Clock Enable */
#define MCLK_APBDMASK_SERCOM5       (0x1ul << MCLK_APBDMASK_SERCOM5_Pos)
#define MCLK_APBDMASK_TC4_Pos       2            /**< \brief (MCLK_APBDMASK) TC4 APB Clock Enable */
#define MCLK_APBDMASK_TC4           (0x1ul << MCLK_APBDMASK_TC4_Pos)
#define MCLK_APBDMASK_ADC_Pos       3            /**< \brief (MCLK_APBDMASK) ADC APB Clock Enable */
#define MCLK_APBDMASK_ADC           (0x1ul << MCLK_APBDMASK_ADC_Pos)
#define MCLK_APBDMASK_AC_Pos        4            /**< \brief (MCLK_APBDMASK) AC APB Clock Enable */
#define MCLK_APBDMASK_AC            (0x1ul << MCLK_APBDMASK_AC_Pos)
#define MCLK_APBDMASK_PTC_Pos       5            /**< \brief (MCLK_APBDMASK) PTC APB Clock Enable */
#define MCLK_APBDMASK_PTC           (0x1ul << MCLK_APBDMASK_PTC_Pos)
#define MCLK_APBDMASK_CCL_Pos       7            /**< \brief (MCLK_APBDMASK) CCL APB Clock Enable */
#define MCLK_APBDMASK_CCL           (0x1ul << MCLK_APBDMASK_CCL_Pos)
#define MCLK_APBDMASK_MASK          0x000000BFul /**< \brief (MCLK_APBDMASK) MASK Register */

/* -------- MCLK_APBEMASK : (MCLK Offset: 0x24) (R/W 32) APBE Mask -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t PAC_:1;           /*!< bit:      0  PAC APB Clock Enable               */
    uint32_t :31;              /*!< bit:  1..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCLK_APBEMASK_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCLK_APBEMASK_OFFSET        0x24         /**< \brief (MCLK_APBEMASK offset) APBE Mask */
#define MCLK_APBEMASK_RESETVALUE    0x0000000Dul /**< \brief (MCLK_APBEMASK reset_value) APBE Mask */

#define MCLK_APBEMASK_PAC_Pos       0            /**< \brief (MCLK_APBEMASK) PAC APB Clock Enable */
#define MCLK_APBEMASK_PAC           (0x1ul << MCLK_APBEMASK_PAC_Pos)
#define MCLK_APBEMASK_MASK          0x00000001ul /**< \brief (MCLK_APBEMASK) MASK Register */

/** \brief MCLK hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO MCLK_CTRLA_Type           CTRLA;       /**< \brief Offset: 0x00 (R/W  8) Control A */
  __IO MCLK_INTENCLR_Type        INTENCLR;    /**< \brief Offset: 0x01 (R/W  8) Interrupt Enable Clear */
  __IO MCLK_INTENSET_Type        INTENSET;    /**< \brief Offset: 0x02 (R/W  8) Interrupt Enable Set */
  __IO MCLK_INTFLAG_Type         INTFLAG;     /**< \brief Offset: 0x03 (R/W  8) Interrupt Flag Status and Clear */
  __IO MCLK_CPUDIV_Type          CPUDIV;      /**< \brief Offset: 0x04 (R/W  8) CPU Clock Division */
  __IO MCLK_LPDIV_Type           LPDIV;       /**< \brief Offset: 0x05 (R/W  8) Low-Power Clock Division */
  __IO MCLK_BUPDIV_Type          BUPDIV;      /**< \brief Offset: 0x06 (R/W  8) Backup Clock Division */
       RoReg8                    Reserved1[0x9];
  __IO MCLK_AHBMASK_Type         AHBMASK;     /**< \brief Offset: 0x10 (R/W 32) AHB Mask */
  __IO MCLK_APBAMASK_Type        APBAMASK;    /**< \brief Offset: 0x14 (R/W 32) APBA Mask */
  __IO MCLK_APBBMASK_Type        APBBMASK;    /**< \brief Offset: 0x18 (R/W 32) APBB Mask */
  __IO MCLK_APBCMASK_Type        APBCMASK;    /**< \brief Offset: 0x1C (R/W 32) APBC Mask */
  __IO MCLK_APBDMASK_Type        APBDMASK;    /**< \brief Offset: 0x20 (R/W 32) APBD Mask */
  __IO MCLK_APBEMASK_Type        APBEMASK;    /**< \brief Offset: 0x24 (R/W 32) APBE Mask */
} Mclk;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/*@}*/

#endif /* _SAMR30_MCLK_COMPONENT_ */
#ifndef _SAMR30_GCLK_COMPONENT_
#define _SAMR30_GCLK_COMPONENT_

/* ========================================================================== */
/**  SOFTWARE API DEFINITION FOR GCLK */
/* ========================================================================== */
/** \addtogroup SAMR30_GCLK Generic Clock Generator */
/*@{*/

#define GCLK_U2122
#define REV_GCLK                    0x111

/* -------- GCLK_CTRLA : (GCLK Offset: 0x00) (R/W  8) Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  SWRST:1;          /*!< bit:      0  Software Reset                     */
    uint8_t  :7;               /*!< bit:  1.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} GCLK_CTRLA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define GCLK_CTRLA_OFFSET           0x00         /**< \brief (GCLK_CTRLA offset) Control */
#define GCLK_CTRLA_RESETVALUE       0x00ul       /**< \brief (GCLK_CTRLA reset_value) Control */

#define GCLK_CTRLA_SWRST_Pos        0            /**< \brief (GCLK_CTRLA) Software Reset */
#define GCLK_CTRLA_SWRST            (0x1ul << GCLK_CTRLA_SWRST_Pos)
#define GCLK_CTRLA_MASK             0x01ul       /**< \brief (GCLK_CTRLA) MASK Register */

/* -------- GCLK_SYNCBUSY : (GCLK Offset: 0x04) (R/  32) Synchronization Busy -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t SWRST:1;          /*!< bit:      0  Software Reset Synchroniation Busy bit */
    uint32_t :1;               /*!< bit:      1  Reserved                           */
    uint32_t GENCTRL:9;        /*!< bit:  2..10  Generic Clock Generator Control Synchronization Busy bits */
    uint32_t :21;              /*!< bit: 11..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} GCLK_SYNCBUSY_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define GCLK_SYNCBUSY_OFFSET        0x04         /**< \brief (GCLK_SYNCBUSY offset) Synchronization Busy */
#define GCLK_SYNCBUSY_RESETVALUE    0x00000000ul /**< \brief (GCLK_SYNCBUSY reset_value) Synchronization Busy */

#define GCLK_SYNCBUSY_SWRST_Pos     0            /**< \brief (GCLK_SYNCBUSY) Software Reset Synchroniation Busy bit */
#define GCLK_SYNCBUSY_SWRST         (0x1ul << GCLK_SYNCBUSY_SWRST_Pos)
#define GCLK_SYNCBUSY_GENCTRL_Pos   2            /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control Synchronization Busy bits */
#define GCLK_SYNCBUSY_GENCTRL_Msk   (0x1FFul << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_GENCTRL(value) (GCLK_SYNCBUSY_GENCTRL_Msk & ((value) << GCLK_SYNCBUSY_GENCTRL_Pos))
#define   GCLK_SYNCBUSY_GENCTRL_GCLK0_Val 0x0ul  /**< \brief (GCLK_SYNCBUSY) Generic clock generator 0 */
#define   GCLK_SYNCBUSY_GENCTRL_GCLK1_Val 0x1ul  /**< \brief (GCLK_SYNCBUSY) Generic clock generator 1 */
#define   GCLK_SYNCBUSY_GENCTRL_GCLK2_Val 0x2ul  /**< \brief (GCLK_SYNCBUSY) Generic clock generator 2 */
#define   GCLK_SYNCBUSY_GENCTRL_GCLK3_Val 0x3ul  /**< \brief (GCLK_SYNCBUSY) Generic clock generator 3 */
#define   GCLK_SYNCBUSY_GENCTRL_GCLK4_Val 0x4ul  /**< \brief (GCLK_SYNCBUSY) Generic clock generator 4 */
#define   GCLK_SYNCBUSY_GENCTRL_GCLK5_Val 0x5ul  /**< \brief (GCLK_SYNCBUSY) Generic clock generator 5 */
#define   GCLK_SYNCBUSY_GENCTRL_GCLK6_Val 0x6ul  /**< \brief (GCLK_SYNCBUSY) Generic clock generator 6 */
#define   GCLK_SYNCBUSY_GENCTRL_GCLK7_Val 0x7ul  /**< \brief (GCLK_SYNCBUSY) Generic clock generator 7 */
#define   GCLK_SYNCBUSY_GENCTRL_GCLK8_Val 0x8ul  /**< \brief (GCLK_SYNCBUSY) Generic clock generator 8 */
#define GCLK_SYNCBUSY_GENCTRL_GCLK0 (GCLK_SYNCBUSY_GENCTRL_GCLK0_Val << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_GENCTRL_GCLK1 (GCLK_SYNCBUSY_GENCTRL_GCLK1_Val << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_GENCTRL_GCLK2 (GCLK_SYNCBUSY_GENCTRL_GCLK2_Val << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_GENCTRL_GCLK3 (GCLK_SYNCBUSY_GENCTRL_GCLK3_Val << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_GENCTRL_GCLK4 (GCLK_SYNCBUSY_GENCTRL_GCLK4_Val << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_GENCTRL_GCLK5 (GCLK_SYNCBUSY_GENCTRL_GCLK5_Val << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_GENCTRL_GCLK6 (GCLK_SYNCBUSY_GENCTRL_GCLK6_Val << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_GENCTRL_GCLK7 (GCLK_SYNCBUSY_GENCTRL_GCLK7_Val << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_GENCTRL_GCLK8 (GCLK_SYNCBUSY_GENCTRL_GCLK8_Val << GCLK_SYNCBUSY_GENCTRL_Pos)
#define GCLK_SYNCBUSY_MASK          0x000007FDul /**< \brief (GCLK_SYNCBUSY) MASK Register */

/* -------- GCLK_GENCTRL : (GCLK Offset: 0x20) (R/W 32) Generic Clock Generator Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t SRC:4;            /*!< bit:  0.. 3  Source Select                      */
    uint32_t :4;               /*!< bit:  4.. 7  Reserved                           */
    uint32_t GENEN:1;          /*!< bit:      8  Generic Clock Generator Enable     */
    uint32_t IDC:1;            /*!< bit:      9  Improve Duty Cycle                 */
    uint32_t OOV:1;            /*!< bit:     10  Output Off Value                   */
    uint32_t OE:1;             /*!< bit:     11  Output Enable                      */
    uint32_t DIVSEL:1;         /*!< bit:     12  Divide Selection                   */
    uint32_t RUNSTDBY:1;       /*!< bit:     13  Run in Standby                     */
    uint32_t :2;               /*!< bit: 14..15  Reserved                           */
    uint32_t DIV:16;           /*!< bit: 16..31  Division Factor                    */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} GCLK_GENCTRL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define GCLK_GENCTRL_OFFSET         0x20         /**< \brief (GCLK_GENCTRL offset) Generic Clock Generator Control */
#define GCLK_GENCTRL_RESETVALUE     0x00000000ul /**< \brief (GCLK_GENCTRL reset_value) Generic Clock Generator Control */

#define GCLK_GENCTRL_SRC_Pos        0            /**< \brief (GCLK_GENCTRL) Source Select */
#define GCLK_GENCTRL_SRC_Msk        (0xFul << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_SRC(value)     (GCLK_GENCTRL_SRC_Msk & ((value) << GCLK_GENCTRL_SRC_Pos))
#define   GCLK_GENCTRL_SRC_XOSC_Val       0x0ul  /**< \brief (GCLK_GENCTRL) XOSC oscillator output */
#define   GCLK_GENCTRL_SRC_GCLKIN_Val     0x1ul  /**< \brief (GCLK_GENCTRL) Generator input pad */
#define   GCLK_GENCTRL_SRC_GCLKGEN1_Val   0x2ul  /**< \brief (GCLK_GENCTRL) Generic clock generator 1 output */
#define   GCLK_GENCTRL_SRC_OSCULP32K_Val  0x3ul  /**< \brief (GCLK_GENCTRL) OSCULP32K oscillator output */
#define   GCLK_GENCTRL_SRC_OSC32K_Val     0x4ul  /**< \brief (GCLK_GENCTRL) OSC32K oscillator output */
#define   GCLK_GENCTRL_SRC_XOSC32K_Val    0x5ul  /**< \brief (GCLK_GENCTRL) XOSC32K oscillator output */
#define   GCLK_GENCTRL_SRC_OSC16M_Val     0x6ul  /**< \brief (GCLK_GENCTRL) OSC16M oscillator output */
#define   GCLK_GENCTRL_SRC_DFLL48M_Val    0x7ul  /**< \brief (GCLK_GENCTRL) DFLL48M output */
#define   GCLK_GENCTRL_SRC_DPLL96M_Val    0x8ul  /**< \brief (GCLK_GENCTRL) DPLL96M output */
#define GCLK_GENCTRL_SRC_XOSC       (GCLK_GENCTRL_SRC_XOSC_Val     << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_SRC_GCLKIN     (GCLK_GENCTRL_SRC_GCLKIN_Val   << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_SRC_GCLKGEN1   (GCLK_GENCTRL_SRC_GCLKGEN1_Val << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_SRC_OSCULP32K  (GCLK_GENCTRL_SRC_OSCULP32K_Val << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_SRC_OSC32K     (GCLK_GENCTRL_SRC_OSC32K_Val   << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_SRC_XOSC32K    (GCLK_GENCTRL_SRC_XOSC32K_Val  << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_SRC_OSC16M     (GCLK_GENCTRL_SRC_OSC16M_Val   << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_SRC_DFLL48M    (GCLK_GENCTRL_SRC_DFLL48M_Val  << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_SRC_DPLL96M    (GCLK_GENCTRL_SRC_DPLL96M_Val  << GCLK_GENCTRL_SRC_Pos)
#define GCLK_GENCTRL_GENEN_Pos      8            /**< \brief (GCLK_GENCTRL) Generic Clock Generator Enable */
#define GCLK_GENCTRL_GENEN          (0x1ul << GCLK_GENCTRL_GENEN_Pos)
#define GCLK_GENCTRL_IDC_Pos        9            /**< \brief (GCLK_GENCTRL) Improve Duty Cycle */
#define GCLK_GENCTRL_IDC            (0x1ul << GCLK_GENCTRL_IDC_Pos)
#define GCLK_GENCTRL_OOV_Pos        10           /**< \brief (GCLK_GENCTRL) Output Off Value */
#define GCLK_GENCTRL_OOV            (0x1ul << GCLK_GENCTRL_OOV_Pos)
#define GCLK_GENCTRL_OE_Pos         11           /**< \brief (GCLK_GENCTRL) Output Enable */
#define GCLK_GENCTRL_OE             (0x1ul << GCLK_GENCTRL_OE_Pos)
#define GCLK_GENCTRL_DIVSEL_Pos     12           /**< \brief (GCLK_GENCTRL) Divide Selection */
#define GCLK_GENCTRL_DIVSEL         (0x1ul << GCLK_GENCTRL_DIVSEL_Pos)
#define GCLK_GENCTRL_RUNSTDBY_Pos   13           /**< \brief (GCLK_GENCTRL) Run in Standby */
#define GCLK_GENCTRL_RUNSTDBY       (0x1ul << GCLK_GENCTRL_RUNSTDBY_Pos)
#define GCLK_GENCTRL_DIV_Pos        16           /**< \brief (GCLK_GENCTRL) Division Factor */
#define GCLK_GENCTRL_DIV_Msk        (0xFFFFul << GCLK_GENCTRL_DIV_Pos)
#define GCLK_GENCTRL_DIV(value)     (GCLK_GENCTRL_DIV_Msk & ((value) << GCLK_GENCTRL_DIV_Pos))
#define GCLK_GENCTRL_MASK           0xFFFF3F0Ful /**< \brief (GCLK_GENCTRL) MASK Register */

/* -------- GCLK_PCHCTRL : (GCLK Offset: 0x80) (R/W 32) Peripheral Clock Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t GEN:4;            /*!< bit:  0.. 3  Generic Clock Generator            */
    uint32_t :2;               /*!< bit:  4.. 5  Reserved                           */
    uint32_t CHEN:1;           /*!< bit:      6  Channel Enable                     */
    uint32_t WRTLOCK:1;        /*!< bit:      7  Write Lock                         */
    uint32_t :24;              /*!< bit:  8..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} GCLK_PCHCTRL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define GCLK_PCHCTRL_OFFSET         0x80         /**< \brief (GCLK_PCHCTRL offset) Peripheral Clock Control */
#define GCLK_PCHCTRL_RESETVALUE     0x00000000ul /**< \brief (GCLK_PCHCTRL reset_value) Peripheral Clock Control */

#define GCLK_PCHCTRL_GEN_Pos        0            /**< \brief (GCLK_PCHCTRL) Generic Clock Generator */
#define GCLK_PCHCTRL_GEN_Msk        (0xFul << GCLK_PCHCTRL_GEN_Pos)
#define GCLK_PCHCTRL_GEN(value)     (GCLK_PCHCTRL_GEN_Msk & ((value) << GCLK_PCHCTRL_GEN_Pos))
#define   GCLK_PCHCTRL_GEN_GCLK0_Val      0x0ul  /**< \brief (GCLK_PCHCTRL) Generic clock generator 0 */
#define   GCLK_PCHCTRL_GEN_GCLK1_Val      0x1ul  /**< \brief (GCLK_PCHCTRL) Generic clock generator 1 */
#define   GCLK_PCHCTRL_GEN_GCLK2_Val      0x2ul  /**< \brief (GCLK_PCHCTRL) Generic clock generator 2 */
#define   GCLK_PCHCTRL_GEN_GCLK3_Val      0x3ul  /**< \brief (GCLK_PCHCTRL) Generic clock generator 3 */
#define   GCLK_PCHCTRL_GEN_GCLK4_Val      0x4ul  /**< \brief (GCLK_PCHCTRL) Generic clock generator 4 */
#define   GCLK_PCHCTRL_GEN_GCLK5_Val      0x5ul  /**< \brief (GCLK_PCHCTRL) Generic clock generator 5 */
#define   GCLK_PCHCTRL_GEN_GCLK6_Val      0x6ul  /**< \brief (GCLK_PCHCTRL) Generic clock generator 6 */
#define   GCLK_PCHCTRL_GEN_GCLK7_Val      0x7ul  /**< \brief (GCLK_PCHCTRL) Generic clock generator 7 */
#define GCLK_PCHCTRL_GEN_GCLK0      (GCLK_PCHCTRL_GEN_GCLK0_Val    << GCLK_PCHCTRL_GEN_Pos)
#define GCLK_PCHCTRL_GEN_GCLK1      (GCLK_PCHCTRL_GEN_GCLK1_Val    << GCLK_PCHCTRL_GEN_Pos)
#define GCLK_PCHCTRL_GEN_GCLK2      (GCLK_PCHCTRL_GEN_GCLK2_Val    << GCLK_PCHCTRL_GEN_Pos)
#define GCLK_PCHCTRL_GEN_GCLK3      (GCLK_PCHCTRL_GEN_GCLK3_Val    << GCLK_PCHCTRL_GEN_Pos)
#define GCLK_PCHCTRL_GEN_GCLK4      (GCLK_PCHCTRL_GEN_GCLK4_Val    << GCLK_PCHCTRL_GEN_Pos)
#define GCLK_PCHCTRL_GEN_GCLK5      (GCLK_PCHCTRL_GEN_GCLK5_Val    << GCLK_PCHCTRL_GEN_Pos)
#define GCLK_PCHCTRL_GEN_GCLK6      (GCLK_PCHCTRL_GEN_GCLK6_Val    << GCLK_PCHCTRL_GEN_Pos)
#define GCLK_PCHCTRL_GEN_GCLK7      (GCLK_PCHCTRL_GEN_GCLK7_Val    << GCLK_PCHCTRL_GEN_Pos)
#define GCLK_PCHCTRL_CHEN_Pos       6            /**< \brief (GCLK_PCHCTRL) Channel Enable */
#define GCLK_PCHCTRL_CHEN           (0x1ul << GCLK_PCHCTRL_CHEN_Pos)
#define GCLK_PCHCTRL_WRTLOCK_Pos    7            /**< \brief (GCLK_PCHCTRL) Write Lock */
#define GCLK_PCHCTRL_WRTLOCK        (0x1ul << GCLK_PCHCTRL_WRTLOCK_Pos)
#define GCLK_PCHCTRL_MASK           0x000000CFul /**< \brief (GCLK_PCHCTRL) MASK Register */

/** \brief GCLK hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO GCLK_CTRLA_Type           CTRLA;       /**< \brief Offset: 0x00 (R/W  8) Control */
       RoReg8                    Reserved1[0x3];
  __I  GCLK_SYNCBUSY_Type        SYNCBUSY;    /**< \brief Offset: 0x04 (R/  32) Synchronization Busy */
       RoReg8                    Reserved2[0x18];
  __IO GCLK_GENCTRL_Type         GENCTRL[9];  /**< \brief Offset: 0x20 (R/W 32) Generic Clock Generator Control */
       RoReg8                    Reserved3[0x3C];
  __IO GCLK_PCHCTRL_Type         PCHCTRL[36]; /**< \brief Offset: 0x80 (R/W 32) Peripheral Clock Control */
} Gclk;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/*@}*/

#endif /* _SAMR30_GCLK_COMPONENT_ */
#ifndef _SAMR30_OSCCTRL_COMPONENT_
#define _SAMR30_OSCCTRL_COMPONENT_

/* ========================================================================== */
/**  SOFTWARE API DEFINITION FOR OSCCTRL */
/* ========================================================================== */
/** \addtogroup SAMR30_OSCCTRL Oscillators Control */
/*@{*/

#define OSCCTRL_U2119
#define REV_OSCCTRL                 0x110

/* -------- OSCCTRL_INTENCLR : (OSCCTRL Offset: 0x00) (R/W 32) Interrupt Enable Clear -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t XOSCRDY:1;        /*!< bit:      0  XOSC Ready Interrupt Enable        */
    uint32_t :3;               /*!< bit:  1.. 3  Reserved                           */
    uint32_t OSC16MRDY:1;      /*!< bit:      4  OSC16M Ready Interrupt Enable      */
    uint32_t :3;               /*!< bit:  5.. 7  Reserved                           */
    uint32_t DFLLRDY:1;        /*!< bit:      8  DFLL Ready Interrupt Enable        */
    uint32_t DFLLOOB:1;        /*!< bit:      9  DFLL Out Of Bounds Interrupt Enable */
    uint32_t DFLLLCKF:1;       /*!< bit:     10  DFLL Lock Fine Interrupt Enable    */
    uint32_t DFLLLCKC:1;       /*!< bit:     11  DFLL Lock Coarse Interrupt Enable  */
    uint32_t DFLLRCS:1;        /*!< bit:     12  DFLL Reference Clock Stopped Interrupt Enable */
    uint32_t :3;               /*!< bit: 13..15  Reserved                           */
    uint32_t DPLLLCKR:1;       /*!< bit:     16  DPLL Lock Rise Interrupt Enable    */
    uint32_t DPLLLCKF:1;       /*!< bit:     17  DPLL Lock Fall Interrupt Enable    */
    uint32_t DPLLLTO:1;        /*!< bit:     18  DPLL Time Out Interrupt Enable     */
    uint32_t DPLLLDRTO:1;      /*!< bit:     19  DPLL Ratio Ready Interrupt Enable  */
    uint32_t :12;              /*!< bit: 20..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} OSCCTRL_INTENCLR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_INTENCLR_OFFSET     0x00         /**< \brief (OSCCTRL_INTENCLR offset) Interrupt Enable Clear */
#define OSCCTRL_INTENCLR_RESETVALUE 0x00000000ul /**< \brief (OSCCTRL_INTENCLR reset_value) Interrupt Enable Clear */

#define OSCCTRL_INTENCLR_XOSCRDY_Pos 0            /**< \brief (OSCCTRL_INTENCLR) XOSC Ready Interrupt Enable */
#define OSCCTRL_INTENCLR_XOSCRDY    (0x1ul << OSCCTRL_INTENCLR_XOSCRDY_Pos)
#define OSCCTRL_INTENCLR_OSC16MRDY_Pos 4            /**< \brief (OSCCTRL_INTENCLR) OSC16M Ready Interrupt Enable */
#define OSCCTRL_INTENCLR_OSC16MRDY  (0x1ul << OSCCTRL_INTENCLR_OSC16MRDY_Pos)
#define OSCCTRL_INTENCLR_DFLLRDY_Pos 8            /**< \brief (OSCCTRL_INTENCLR) DFLL Ready Interrupt Enable */
#define OSCCTRL_INTENCLR_DFLLRDY    (0x1ul << OSCCTRL_INTENCLR_DFLLRDY_Pos)
#define OSCCTRL_INTENCLR_DFLLOOB_Pos 9            /**< \brief (OSCCTRL_INTENCLR) DFLL Out Of Bounds Interrupt Enable */
#define OSCCTRL_INTENCLR_DFLLOOB    (0x1ul << OSCCTRL_INTENCLR_DFLLOOB_Pos)
#define OSCCTRL_INTENCLR_DFLLLCKF_Pos 10           /**< \brief (OSCCTRL_INTENCLR) DFLL Lock Fine Interrupt Enable */
#define OSCCTRL_INTENCLR_DFLLLCKF   (0x1ul << OSCCTRL_INTENCLR_DFLLLCKF_Pos)
#define OSCCTRL_INTENCLR_DFLLLCKC_Pos 11           /**< \brief (OSCCTRL_INTENCLR) DFLL Lock Coarse Interrupt Enable */
#define OSCCTRL_INTENCLR_DFLLLCKC   (0x1ul << OSCCTRL_INTENCLR_DFLLLCKC_Pos)
#define OSCCTRL_INTENCLR_DFLLRCS_Pos 12           /**< \brief (OSCCTRL_INTENCLR) DFLL Reference Clock Stopped Interrupt Enable */
#define OSCCTRL_INTENCLR_DFLLRCS    (0x1ul << OSCCTRL_INTENCLR_DFLLRCS_Pos)
#define OSCCTRL_INTENCLR_DPLLLCKR_Pos 16           /**< \brief (OSCCTRL_INTENCLR) DPLL Lock Rise Interrupt Enable */
#define OSCCTRL_INTENCLR_DPLLLCKR   (0x1ul << OSCCTRL_INTENCLR_DPLLLCKR_Pos)
#define OSCCTRL_INTENCLR_DPLLLCKF_Pos 17           /**< \brief (OSCCTRL_INTENCLR) DPLL Lock Fall Interrupt Enable */
#define OSCCTRL_INTENCLR_DPLLLCKF   (0x1ul << OSCCTRL_INTENCLR_DPLLLCKF_Pos)
#define OSCCTRL_INTENCLR_DPLLLTO_Pos 18           /**< \brief (OSCCTRL_INTENCLR) DPLL Time Out Interrupt Enable */
#define OSCCTRL_INTENCLR_DPLLLTO    (0x1ul << OSCCTRL_INTENCLR_DPLLLTO_Pos)
#define OSCCTRL_INTENCLR_DPLLLDRTO_Pos 19           /**< \brief (OSCCTRL_INTENCLR) DPLL Ratio Ready Interrupt Enable */
#define OSCCTRL_INTENCLR_DPLLLDRTO  (0x1ul << OSCCTRL_INTENCLR_DPLLLDRTO_Pos)
#define OSCCTRL_INTENCLR_MASK       0x000F1F11ul /**< \brief (OSCCTRL_INTENCLR) MASK Register */

/* -------- OSCCTRL_INTENSET : (OSCCTRL Offset: 0x04) (R/W 32) Interrupt Enable Set -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t XOSCRDY:1;        /*!< bit:      0  XOSC Ready Interrupt Enable        */
    uint32_t :3;               /*!< bit:  1.. 3  Reserved                           */
    uint32_t OSC16MRDY:1;      /*!< bit:      4  OSC16M Ready Interrupt Enable      */
    uint32_t :3;               /*!< bit:  5.. 7  Reserved                           */
    uint32_t DFLLRDY:1;        /*!< bit:      8  DFLL Ready Interrupt Enable        */
    uint32_t DFLLOOB:1;        /*!< bit:      9  DFLL Out Of Bounds Interrupt Enable */
    uint32_t DFLLLCKF:1;       /*!< bit:     10  DFLL Lock Fine Interrupt Enable    */
    uint32_t DFLLLCKC:1;       /*!< bit:     11  DFLL Lock Coarse Interrupt Enable  */
    uint32_t DFLLRCS:1;        /*!< bit:     12  DFLL Reference Clock Stopped Interrupt Enable */
    uint32_t :3;               /*!< bit: 13..15  Reserved                           */
    uint32_t DPLLLCKR:1;       /*!< bit:     16  DPLL Lock Rise Interrupt Enable    */
    uint32_t DPLLLCKF:1;       /*!< bit:     17  DPLL Lock Fall Interrupt Enable    */
    uint32_t DPLLLTO:1;        /*!< bit:     18  DPLL Time Out Interrupt Enable     */
    uint32_t DPLLLDRTO:1;      /*!< bit:     19  DPLL Ratio Ready Interrupt Enable  */
    uint32_t :12;              /*!< bit: 20..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} OSCCTRL_INTENSET_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_INTENSET_OFFSET     0x04         /**< \brief (OSCCTRL_INTENSET offset) Interrupt Enable Set */
#define OSCCTRL_INTENSET_RESETVALUE 0x00000000ul /**< \brief (OSCCTRL_INTENSET reset_value) Interrupt Enable Set */

#define OSCCTRL_INTENSET_XOSCRDY_Pos 0            /**< \brief (OSCCTRL_INTENSET) XOSC Ready Interrupt Enable */
#define OSCCTRL_INTENSET_XOSCRDY    (0x1ul << OSCCTRL_INTENSET_XOSCRDY_Pos)
#define OSCCTRL_INTENSET_OSC16MRDY_Pos 4            /**< \brief (OSCCTRL_INTENSET) OSC16M Ready Interrupt Enable */
#define OSCCTRL_INTENSET_OSC16MRDY  (0x1ul << OSCCTRL_INTENSET_OSC16MRDY_Pos)
#define OSCCTRL_INTENSET_DFLLRDY_Pos 8            /**< \brief (OSCCTRL_INTENSET) DFLL Ready Interrupt Enable */
#define OSCCTRL_INTENSET_DFLLRDY    (0x1ul << OSCCTRL_INTENSET_DFLLRDY_Pos)
#define OSCCTRL_INTENSET_DFLLOOB_Pos 9            /**< \brief (OSCCTRL_INTENSET) DFLL Out Of Bounds Interrupt Enable */
#define OSCCTRL_INTENSET_DFLLOOB    (0x1ul << OSCCTRL_INTENSET_DFLLOOB_Pos)
#define OSCCTRL_INTENSET_DFLLLCKF_Pos 10           /**< \brief (OSCCTRL_INTENSET) DFLL Lock Fine Interrupt Enable */
#define OSCCTRL_INTENSET_DFLLLCKF   (0x1ul << OSCCTRL_INTENSET_DFLLLCKF_Pos)
#define OSCCTRL_INTENSET_DFLLLCKC_Pos 11           /**< \brief (OSCCTRL_INTENSET) DFLL Lock Coarse Interrupt Enable */
#define OSCCTRL_INTENSET_DFLLLCKC   (0x1ul << OSCCTRL_INTENSET_DFLLLCKC_Pos)
#define OSCCTRL_INTENSET_DFLLRCS_Pos 12           /**< \brief (OSCCTRL_INTENSET) DFLL Reference Clock Stopped Interrupt Enable */
#define OSCCTRL_INTENSET_DFLLRCS    (0x1ul << OSCCTRL_INTENSET_DFLLRCS_Pos)
#define OSCCTRL_INTENSET_DPLLLCKR_Pos 16           /**< \brief (OSCCTRL_INTENSET) DPLL Lock Rise Interrupt Enable */
#define OSCCTRL_INTENSET_DPLLLCKR   (0x1ul << OSCCTRL_INTENSET_DPLLLCKR_Pos)
#define OSCCTRL_INTENSET_DPLLLCKF_Pos 17           /**< \brief (OSCCTRL_INTENSET) DPLL Lock Fall Interrupt Enable */
#define OSCCTRL_INTENSET_DPLLLCKF   (0x1ul << OSCCTRL_INTENSET_DPLLLCKF_Pos)
#define OSCCTRL_INTENSET_DPLLLTO_Pos 18           /**< \brief (OSCCTRL_INTENSET) DPLL Time Out Interrupt Enable */
#define OSCCTRL_INTENSET_DPLLLTO    (0x1ul << OSCCTRL_INTENSET_DPLLLTO_Pos)
#define OSCCTRL_INTENSET_DPLLLDRTO_Pos 19           /**< \brief (OSCCTRL_INTENSET) DPLL Ratio Ready Interrupt Enable */
#define OSCCTRL_INTENSET_DPLLLDRTO  (0x1ul << OSCCTRL_INTENSET_DPLLLDRTO_Pos)
#define OSCCTRL_INTENSET_MASK       0x000F1F11ul /**< \brief (OSCCTRL_INTENSET) MASK Register */

/* -------- OSCCTRL_INTFLAG : (OSCCTRL Offset: 0x08) (R/W 32) Interrupt Flag Status and Clear -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { // __I to avoid read-modify-write on write-to-clear register
  struct {
    __I uint32_t XOSCRDY:1;        /*!< bit:      0  XOSC Ready                         */
    __I uint32_t :3;               /*!< bit:  1.. 3  Reserved                           */
    __I uint32_t OSC16MRDY:1;      /*!< bit:      4  OSC16M Ready                       */
    __I uint32_t :3;               /*!< bit:  5.. 7  Reserved                           */
    __I uint32_t DFLLRDY:1;        /*!< bit:      8  DFLL Ready                         */
    __I uint32_t DFLLOOB:1;        /*!< bit:      9  DFLL Out Of Bounds                 */
    __I uint32_t DFLLLCKF:1;       /*!< bit:     10  DFLL Lock Fine                     */
    __I uint32_t DFLLLCKC:1;       /*!< bit:     11  DFLL Lock Coarse                   */
    __I uint32_t DFLLRCS:1;        /*!< bit:     12  DFLL Reference Clock Stopped       */
    __I uint32_t :3;               /*!< bit: 13..15  Reserved                           */
    __I uint32_t DPLLLCKR:1;       /*!< bit:     16  DPLL Lock Rise                     */
    __I uint32_t DPLLLCKF:1;       /*!< bit:     17  DPLL Lock Fall                     */
    __I uint32_t DPLLLTO:1;        /*!< bit:     18  DPLL Timeout                       */
    __I uint32_t DPLLLDRTO:1;      /*!< bit:     19  DPLL Ratio Ready                   */
    __I uint32_t :12;              /*!< bit: 20..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} OSCCTRL_INTFLAG_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_INTFLAG_OFFSET      0x08         /**< \brief (OSCCTRL_INTFLAG offset) Interrupt Flag Status and Clear */
#define OSCCTRL_INTFLAG_RESETVALUE  0x00000000ul /**< \brief (OSCCTRL_INTFLAG reset_value) Interrupt Flag Status and Clear */

#define OSCCTRL_INTFLAG_XOSCRDY_Pos 0            /**< \brief (OSCCTRL_INTFLAG) XOSC Ready */
#define OSCCTRL_INTFLAG_XOSCRDY     (0x1ul << OSCCTRL_INTFLAG_XOSCRDY_Pos)
#define OSCCTRL_INTFLAG_OSC16MRDY_Pos 4            /**< \brief (OSCCTRL_INTFLAG) OSC16M Ready */
#define OSCCTRL_INTFLAG_OSC16MRDY   (0x1ul << OSCCTRL_INTFLAG_OSC16MRDY_Pos)
#define OSCCTRL_INTFLAG_DFLLRDY_Pos 8            /**< \brief (OSCCTRL_INTFLAG) DFLL Ready */
#define OSCCTRL_INTFLAG_DFLLRDY     (0x1ul << OSCCTRL_INTFLAG_DFLLRDY_Pos)
#define OSCCTRL_INTFLAG_DFLLOOB_Pos 9            /**< \brief (OSCCTRL_INTFLAG) DFLL Out Of Bounds */
#define OSCCTRL_INTFLAG_DFLLOOB     (0x1ul << OSCCTRL_INTFLAG_DFLLOOB_Pos)
#define OSCCTRL_INTFLAG_DFLLLCKF_Pos 10           /**< \brief (OSCCTRL_INTFLAG) DFLL Lock Fine */
#define OSCCTRL_INTFLAG_DFLLLCKF    (0x1ul << OSCCTRL_INTFLAG_DFLLLCKF_Pos)
#define OSCCTRL_INTFLAG_DFLLLCKC_Pos 11           /**< \brief (OSCCTRL_INTFLAG) DFLL Lock Coarse */
#define OSCCTRL_INTFLAG_DFLLLCKC    (0x1ul << OSCCTRL_INTFLAG_DFLLLCKC_Pos)
#define OSCCTRL_INTFLAG_DFLLRCS_Pos 12           /**< \brief (OSCCTRL_INTFLAG) DFLL Reference Clock Stopped */
#define OSCCTRL_INTFLAG_DFLLRCS     (0x1ul << OSCCTRL_INTFLAG_DFLLRCS_Pos)
#define OSCCTRL_INTFLAG_DPLLLCKR_Pos 16           /**< \brief (OSCCTRL_INTFLAG) DPLL Lock Rise */
#define OSCCTRL_INTFLAG_DPLLLCKR    (0x1ul << OSCCTRL_INTFLAG_DPLLLCKR_Pos)
#define OSCCTRL_INTFLAG_DPLLLCKF_Pos 17           /**< \brief (OSCCTRL_INTFLAG) DPLL Lock Fall */
#define OSCCTRL_INTFLAG_DPLLLCKF    (0x1ul << OSCCTRL_INTFLAG_DPLLLCKF_Pos)
#define OSCCTRL_INTFLAG_DPLLLTO_Pos 18           /**< \brief (OSCCTRL_INTFLAG) DPLL Timeout */
#define OSCCTRL_INTFLAG_DPLLLTO     (0x1ul << OSCCTRL_INTFLAG_DPLLLTO_Pos)
#define OSCCTRL_INTFLAG_DPLLLDRTO_Pos 19           /**< \brief (OSCCTRL_INTFLAG) DPLL Ratio Ready */
#define OSCCTRL_INTFLAG_DPLLLDRTO   (0x1ul << OSCCTRL_INTFLAG_DPLLLDRTO_Pos)
#define OSCCTRL_INTFLAG_MASK        0x000F1F11ul /**< \brief (OSCCTRL_INTFLAG) MASK Register */

/* -------- OSCCTRL_STATUS : (OSCCTRL Offset: 0x0C) (R/  32) Power and Clocks Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t XOSCRDY:1;        /*!< bit:      0  XOSC Ready                         */
    uint32_t :3;               /*!< bit:  1.. 3  Reserved                           */
    uint32_t OSC16MRDY:1;      /*!< bit:      4  OSC16M Ready                       */
    uint32_t :3;               /*!< bit:  5.. 7  Reserved                           */
    uint32_t DFLLRDY:1;        /*!< bit:      8  DFLL Ready                         */
    uint32_t DFLLOOB:1;        /*!< bit:      9  DFLL Out Of Bounds                 */
    uint32_t DFLLLCKF:1;       /*!< bit:     10  DFLL Lock Fine                     */
    uint32_t DFLLLCKC:1;       /*!< bit:     11  DFLL Lock Coarse                   */
    uint32_t DFLLRCS:1;        /*!< bit:     12  DFLL Reference Clock Stopped       */
    uint32_t :3;               /*!< bit: 13..15  Reserved                           */
    uint32_t DPLLLCKR:1;       /*!< bit:     16  DPLL Lock Rise                     */
    uint32_t DPLLLCKF:1;       /*!< bit:     17  DPLL Lock Fall                     */
    uint32_t DPLLTO:1;         /*!< bit:     18  DPLL Timeout                       */
    uint32_t DPLLLDRTO:1;      /*!< bit:     19  DPLL Ratio Ready                   */
    uint32_t :12;              /*!< bit: 20..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} OSCCTRL_STATUS_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_STATUS_OFFSET       0x0C         /**< \brief (OSCCTRL_STATUS offset) Power and Clocks Status */
#define OSCCTRL_STATUS_RESETVALUE   0x00000000ul /**< \brief (OSCCTRL_STATUS reset_value) Power and Clocks Status */

#define OSCCTRL_STATUS_XOSCRDY_Pos  0            /**< \brief (OSCCTRL_STATUS) XOSC Ready */
#define OSCCTRL_STATUS_XOSCRDY      (0x1ul << OSCCTRL_STATUS_XOSCRDY_Pos)
#define OSCCTRL_STATUS_OSC16MRDY_Pos 4            /**< \brief (OSCCTRL_STATUS) OSC16M Ready */
#define OSCCTRL_STATUS_OSC16MRDY    (0x1ul << OSCCTRL_STATUS_OSC16MRDY_Pos)
#define OSCCTRL_STATUS_DFLLRDY_Pos  8            /**< \brief (OSCCTRL_STATUS) DFLL Ready */
#define OSCCTRL_STATUS_DFLLRDY      (0x1ul << OSCCTRL_STATUS_DFLLRDY_Pos)
#define OSCCTRL_STATUS_DFLLOOB_Pos  9            /**< \brief (OSCCTRL_STATUS) DFLL Out Of Bounds */
#define OSCCTRL_STATUS_DFLLOOB      (0x1ul << OSCCTRL_STATUS_DFLLOOB_Pos)
#define OSCCTRL_STATUS_DFLLLCKF_Pos 10           /**< \brief (OSCCTRL_STATUS) DFLL Lock Fine */
#define OSCCTRL_STATUS_DFLLLCKF     (0x1ul << OSCCTRL_STATUS_DFLLLCKF_Pos)
#define OSCCTRL_STATUS_DFLLLCKC_Pos 11           /**< \brief (OSCCTRL_STATUS) DFLL Lock Coarse */
#define OSCCTRL_STATUS_DFLLLCKC     (0x1ul << OSCCTRL_STATUS_DFLLLCKC_Pos)
#define OSCCTRL_STATUS_DFLLRCS_Pos  12           /**< \brief (OSCCTRL_STATUS) DFLL Reference Clock Stopped */
#define OSCCTRL_STATUS_DFLLRCS      (0x1ul << OSCCTRL_STATUS_DFLLRCS_Pos)
#define OSCCTRL_STATUS_DPLLLCKR_Pos 16           /**< \brief (OSCCTRL_STATUS) DPLL Lock Rise */
#define OSCCTRL_STATUS_DPLLLCKR     (0x1ul << OSCCTRL_STATUS_DPLLLCKR_Pos)
#define OSCCTRL_STATUS_DPLLLCKF_Pos 17           /**< \brief (OSCCTRL_STATUS) DPLL Lock Fall */
#define OSCCTRL_STATUS_DPLLLCKF     (0x1ul << OSCCTRL_STATUS_DPLLLCKF_Pos)
#define OSCCTRL_STATUS_DPLLTO_Pos   18           /**< \brief (OSCCTRL_STATUS) DPLL Timeout */
#define OSCCTRL_STATUS_DPLLTO       (0x1ul << OSCCTRL_STATUS_DPLLTO_Pos)
#define OSCCTRL_STATUS_DPLLLDRTO_Pos 19           /**< \brief (OSCCTRL_STATUS) DPLL Ratio Ready */
#define OSCCTRL_STATUS_DPLLLDRTO    (0x1ul << OSCCTRL_STATUS_DPLLLDRTO_Pos)
#define OSCCTRL_STATUS_MASK         0x000F1F11ul /**< \brief (OSCCTRL_STATUS) MASK Register */

/* -------- OSCCTRL_XOSCCTRL : (OSCCTRL Offset: 0x10) (R/W 16) External Multipurpose Crystal Oscillator (XOSC) Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint16_t :1;               /*!< bit:      0  Reserved                           */
    uint16_t ENABLE:1;         /*!< bit:      1  Oscillator Enable                  */
    uint16_t XTALEN:1;         /*!< bit:      2  Crystal Oscillator Enable          */
    uint16_t :3;               /*!< bit:  3.. 5  Reserved                           */
    uint16_t RUNSTDBY:1;       /*!< bit:      6  Run in Standby                     */
    uint16_t ONDEMAND:1;       /*!< bit:      7  On Demand Control                  */
    uint16_t GAIN:3;           /*!< bit:  8..10  Oscillator Gain                    */
    uint16_t AMPGC:1;          /*!< bit:     11  Automatic Amplitude Gain Control   */
    uint16_t STARTUP:4;        /*!< bit: 12..15  Start-Up Time                      */
  } bit;                       /*!< Structure used for bit  access                  */
  uint16_t reg;                /*!< Type      used for register access              */
} OSCCTRL_XOSCCTRL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_XOSCCTRL_OFFSET     0x10         /**< \brief (OSCCTRL_XOSCCTRL offset) External Multipurpose Crystal Oscillator (XOSC) Control */
#define OSCCTRL_XOSCCTRL_RESETVALUE 0x0080ul     /**< \brief (OSCCTRL_XOSCCTRL reset_value) External Multipurpose Crystal Oscillator (XOSC) Control */

#define OSCCTRL_XOSCCTRL_ENABLE_Pos 1            /**< \brief (OSCCTRL_XOSCCTRL) Oscillator Enable */
#define OSCCTRL_XOSCCTRL_ENABLE     (0x1ul << OSCCTRL_XOSCCTRL_ENABLE_Pos)
#define OSCCTRL_XOSCCTRL_XTALEN_Pos 2            /**< \brief (OSCCTRL_XOSCCTRL) Crystal Oscillator Enable */
#define OSCCTRL_XOSCCTRL_XTALEN     (0x1ul << OSCCTRL_XOSCCTRL_XTALEN_Pos)
#define OSCCTRL_XOSCCTRL_RUNSTDBY_Pos 6            /**< \brief (OSCCTRL_XOSCCTRL) Run in Standby */
#define OSCCTRL_XOSCCTRL_RUNSTDBY   (0x1ul << OSCCTRL_XOSCCTRL_RUNSTDBY_Pos)
#define OSCCTRL_XOSCCTRL_ONDEMAND_Pos 7            /**< \brief (OSCCTRL_XOSCCTRL) On Demand Control */
#define OSCCTRL_XOSCCTRL_ONDEMAND   (0x1ul << OSCCTRL_XOSCCTRL_ONDEMAND_Pos)
#define OSCCTRL_XOSCCTRL_GAIN_Pos   8            /**< \brief (OSCCTRL_XOSCCTRL) Oscillator Gain */
#define OSCCTRL_XOSCCTRL_GAIN_Msk   (0x7ul << OSCCTRL_XOSCCTRL_GAIN_Pos)
#define OSCCTRL_XOSCCTRL_GAIN(value) (OSCCTRL_XOSCCTRL_GAIN_Msk & ((value) << OSCCTRL_XOSCCTRL_GAIN_Pos))
#define OSCCTRL_XOSCCTRL_AMPGC_Pos  11           /**< \brief (OSCCTRL_XOSCCTRL) Automatic Amplitude Gain Control */
#define OSCCTRL_XOSCCTRL_AMPGC      (0x1ul << OSCCTRL_XOSCCTRL_AMPGC_Pos)
#define OSCCTRL_XOSCCTRL_STARTUP_Pos 12           /**< \brief (OSCCTRL_XOSCCTRL) Start-Up Time */
#define OSCCTRL_XOSCCTRL_STARTUP_Msk (0xFul << OSCCTRL_XOSCCTRL_STARTUP_Pos)
#define OSCCTRL_XOSCCTRL_STARTUP(value) (OSCCTRL_XOSCCTRL_STARTUP_Msk & ((value) << OSCCTRL_XOSCCTRL_STARTUP_Pos))
#define OSCCTRL_XOSCCTRL_MASK       0xFFC6ul     /**< \brief (OSCCTRL_XOSCCTRL) MASK Register */

/* -------- OSCCTRL_OSC16MCTRL : (OSCCTRL Offset: 0x14) (R/W  8) 16MHz Internal Oscillator (OSC16M) Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  :1;               /*!< bit:      0  Reserved                           */
    uint8_t  ENABLE:1;         /*!< bit:      1  Oscillator Enable                  */
    uint8_t  FSEL:2;           /*!< bit:  2.. 3  Oscillator Frequency Select        */
    uint8_t  :2;               /*!< bit:  4.. 5  Reserved                           */
    uint8_t  RUNSTDBY:1;       /*!< bit:      6  Run in Standby                     */
    uint8_t  ONDEMAND:1;       /*!< bit:      7  On Demand Control                  */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} OSCCTRL_OSC16MCTRL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_OSC16MCTRL_OFFSET   0x14         /**< \brief (OSCCTRL_OSC16MCTRL offset) 16MHz Internal Oscillator (OSC16M) Control */
#define OSCCTRL_OSC16MCTRL_RESETVALUE 0x82ul       /**< \brief (OSCCTRL_OSC16MCTRL reset_value) 16MHz Internal Oscillator (OSC16M) Control */

#define OSCCTRL_OSC16MCTRL_ENABLE_Pos 1            /**< \brief (OSCCTRL_OSC16MCTRL) Oscillator Enable */
#define OSCCTRL_OSC16MCTRL_ENABLE   (0x1ul << OSCCTRL_OSC16MCTRL_ENABLE_Pos)
#define OSCCTRL_OSC16MCTRL_FSEL_Pos 2            /**< \brief (OSCCTRL_OSC16MCTRL) Oscillator Frequency Select */
#define OSCCTRL_OSC16MCTRL_FSEL_Msk (0x3ul << OSCCTRL_OSC16MCTRL_FSEL_Pos)
#define OSCCTRL_OSC16MCTRL_FSEL(value) (OSCCTRL_OSC16MCTRL_FSEL_Msk & ((value) << OSCCTRL_OSC16MCTRL_FSEL_Pos))
#define   OSCCTRL_OSC16MCTRL_FSEL_4_Val   0x0ul  /**< \brief (OSCCTRL_OSC16MCTRL) 4MHz */
#define   OSCCTRL_OSC16MCTRL_FSEL_8_Val   0x1ul  /**< \brief (OSCCTRL_OSC16MCTRL) 8MHz */
#define   OSCCTRL_OSC16MCTRL_FSEL_12_Val  0x2ul  /**< \brief (OSCCTRL_OSC16MCTRL) 12MHz */
#define   OSCCTRL_OSC16MCTRL_FSEL_16_Val  0x3ul  /**< \brief (OSCCTRL_OSC16MCTRL) 16MHz */
#define OSCCTRL_OSC16MCTRL_FSEL_4   (OSCCTRL_OSC16MCTRL_FSEL_4_Val << OSCCTRL_OSC16MCTRL_FSEL_Pos)
#define OSCCTRL_OSC16MCTRL_FSEL_8   (OSCCTRL_OSC16MCTRL_FSEL_8_Val << OSCCTRL_OSC16MCTRL_FSEL_Pos)
#define OSCCTRL_OSC16MCTRL_FSEL_12  (OSCCTRL_OSC16MCTRL_FSEL_12_Val << OSCCTRL_OSC16MCTRL_FSEL_Pos)
#define OSCCTRL_OSC16MCTRL_FSEL_16  (OSCCTRL_OSC16MCTRL_FSEL_16_Val << OSCCTRL_OSC16MCTRL_FSEL_Pos)
#define OSCCTRL_OSC16MCTRL_RUNSTDBY_Pos 6            /**< \brief (OSCCTRL_OSC16MCTRL) Run in Standby */
#define OSCCTRL_OSC16MCTRL_RUNSTDBY (0x1ul << OSCCTRL_OSC16MCTRL_RUNSTDBY_Pos)
#define OSCCTRL_OSC16MCTRL_ONDEMAND_Pos 7            /**< \brief (OSCCTRL_OSC16MCTRL) On Demand Control */
#define OSCCTRL_OSC16MCTRL_ONDEMAND (0x1ul << OSCCTRL_OSC16MCTRL_ONDEMAND_Pos)
#define OSCCTRL_OSC16MCTRL_MASK     0xCEul       /**< \brief (OSCCTRL_OSC16MCTRL) MASK Register */

/* -------- OSCCTRL_DFLLCTRL : (OSCCTRL Offset: 0x18) (R/W 16) DFLL48M Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint16_t :1;               /*!< bit:      0  Reserved                           */
    uint16_t ENABLE:1;         /*!< bit:      1  DFLL Enable                        */
    uint16_t MODE:1;           /*!< bit:      2  Operating Mode Selection           */
    uint16_t STABLE:1;         /*!< bit:      3  Stable DFLL Frequency              */
    uint16_t LLAW:1;           /*!< bit:      4  Lose Lock After Wake               */
    uint16_t USBCRM:1;         /*!< bit:      5  USB Clock Recovery Mode            */
    uint16_t RUNSTDBY:1;       /*!< bit:      6  Run in Standby                     */
    uint16_t ONDEMAND:1;       /*!< bit:      7  On Demand Control                  */
    uint16_t CCDIS:1;          /*!< bit:      8  Chill Cycle Disable                */
    uint16_t QLDIS:1;          /*!< bit:      9  Quick Lock Disable                 */
    uint16_t BPLCKC:1;         /*!< bit:     10  Bypass Coarse Lock                 */
    uint16_t WAITLOCK:1;       /*!< bit:     11  Wait Lock                          */
    uint16_t :4;               /*!< bit: 12..15  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint16_t reg;                /*!< Type      used for register access              */
} OSCCTRL_DFLLCTRL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DFLLCTRL_OFFSET     0x18         /**< \brief (OSCCTRL_DFLLCTRL offset) DFLL48M Control */
#define OSCCTRL_DFLLCTRL_RESETVALUE 0x0080ul     /**< \brief (OSCCTRL_DFLLCTRL reset_value) DFLL48M Control */

#define OSCCTRL_DFLLCTRL_ENABLE_Pos 1            /**< \brief (OSCCTRL_DFLLCTRL) DFLL Enable */
#define OSCCTRL_DFLLCTRL_ENABLE     (0x1ul << OSCCTRL_DFLLCTRL_ENABLE_Pos)
#define OSCCTRL_DFLLCTRL_MODE_Pos   2            /**< \brief (OSCCTRL_DFLLCTRL) Operating Mode Selection */
#define OSCCTRL_DFLLCTRL_MODE       (0x1ul << OSCCTRL_DFLLCTRL_MODE_Pos)
#define OSCCTRL_DFLLCTRL_STABLE_Pos 3            /**< \brief (OSCCTRL_DFLLCTRL) Stable DFLL Frequency */
#define OSCCTRL_DFLLCTRL_STABLE     (0x1ul << OSCCTRL_DFLLCTRL_STABLE_Pos)
#define OSCCTRL_DFLLCTRL_LLAW_Pos   4            /**< \brief (OSCCTRL_DFLLCTRL) Lose Lock After Wake */
#define OSCCTRL_DFLLCTRL_LLAW       (0x1ul << OSCCTRL_DFLLCTRL_LLAW_Pos)
#define OSCCTRL_DFLLCTRL_USBCRM_Pos 5            /**< \brief (OSCCTRL_DFLLCTRL) USB Clock Recovery Mode */
#define OSCCTRL_DFLLCTRL_USBCRM     (0x1ul << OSCCTRL_DFLLCTRL_USBCRM_Pos)
#define OSCCTRL_DFLLCTRL_RUNSTDBY_Pos 6            /**< \brief (OSCCTRL_DFLLCTRL) Run in Standby */
#define OSCCTRL_DFLLCTRL_RUNSTDBY   (0x1ul << OSCCTRL_DFLLCTRL_RUNSTDBY_Pos)
#define OSCCTRL_DFLLCTRL_ONDEMAND_Pos 7            /**< \brief (OSCCTRL_DFLLCTRL) On Demand Control */
#define OSCCTRL_DFLLCTRL_ONDEMAND   (0x1ul << OSCCTRL_DFLLCTRL_ONDEMAND_Pos)
#define OSCCTRL_DFLLCTRL_CCDIS_Pos  8            /**< \brief (OSCCTRL_DFLLCTRL) Chill Cycle Disable */
#define OSCCTRL_DFLLCTRL_CCDIS      (0x1ul << OSCCTRL_DFLLCTRL_CCDIS_Pos)
#define OSCCTRL_DFLLCTRL_QLDIS_Pos  9            /**< \brief (OSCCTRL_DFLLCTRL) Quick Lock Disable */
#define OSCCTRL_DFLLCTRL_QLDIS      (0x1ul << OSCCTRL_DFLLCTRL_QLDIS_Pos)
#define OSCCTRL_DFLLCTRL_BPLCKC_Pos 10           /**< \brief (OSCCTRL_DFLLCTRL) Bypass Coarse Lock */
#define OSCCTRL_DFLLCTRL_BPLCKC     (0x1ul << OSCCTRL_DFLLCTRL_BPLCKC_Pos)
#define OSCCTRL_DFLLCTRL_WAITLOCK_Pos 11           /**< \brief (OSCCTRL_DFLLCTRL) Wait Lock */
#define OSCCTRL_DFLLCTRL_WAITLOCK   (0x1ul << OSCCTRL_DFLLCTRL_WAITLOCK_Pos)
#define OSCCTRL_DFLLCTRL_MASK       0x0FFEul     /**< \brief (OSCCTRL_DFLLCTRL) MASK Register */

/* -------- OSCCTRL_DFLLVAL : (OSCCTRL Offset: 0x1C) (R/W 32) DFLL48M Value -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t FINE:10;          /*!< bit:  0.. 9  Fine Value                         */
    uint32_t COARSE:6;         /*!< bit: 10..15  Coarse Value                       */
    uint32_t DIFF:16;          /*!< bit: 16..31  Multiplication Ratio Difference    */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} OSCCTRL_DFLLVAL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DFLLVAL_OFFSET      0x1C         /**< \brief (OSCCTRL_DFLLVAL offset) DFLL48M Value */
#define OSCCTRL_DFLLVAL_RESETVALUE  0x00000000ul /**< \brief (OSCCTRL_DFLLVAL reset_value) DFLL48M Value */

#define OSCCTRL_DFLLVAL_FINE_Pos    0            /**< \brief (OSCCTRL_DFLLVAL) Fine Value */
#define OSCCTRL_DFLLVAL_FINE_Msk    (0x3FFul << OSCCTRL_DFLLVAL_FINE_Pos)
#define OSCCTRL_DFLLVAL_FINE(value) (OSCCTRL_DFLLVAL_FINE_Msk & ((value) << OSCCTRL_DFLLVAL_FINE_Pos))
#define OSCCTRL_DFLLVAL_COARSE_Pos  10           /**< \brief (OSCCTRL_DFLLVAL) Coarse Value */
#define OSCCTRL_DFLLVAL_COARSE_Msk  (0x3Ful << OSCCTRL_DFLLVAL_COARSE_Pos)
#define OSCCTRL_DFLLVAL_COARSE(value) (OSCCTRL_DFLLVAL_COARSE_Msk & ((value) << OSCCTRL_DFLLVAL_COARSE_Pos))
#define OSCCTRL_DFLLVAL_DIFF_Pos    16           /**< \brief (OSCCTRL_DFLLVAL) Multiplication Ratio Difference */
#define OSCCTRL_DFLLVAL_DIFF_Msk    (0xFFFFul << OSCCTRL_DFLLVAL_DIFF_Pos)
#define OSCCTRL_DFLLVAL_DIFF(value) (OSCCTRL_DFLLVAL_DIFF_Msk & ((value) << OSCCTRL_DFLLVAL_DIFF_Pos))
#define OSCCTRL_DFLLVAL_MASK        0xFFFFFFFFul /**< \brief (OSCCTRL_DFLLVAL) MASK Register */

/* -------- OSCCTRL_DFLLMUL : (OSCCTRL Offset: 0x20) (R/W 32) DFLL48M Multiplier -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t MUL:16;           /*!< bit:  0..15  DFLL Multiply Factor               */
    uint32_t FSTEP:10;         /*!< bit: 16..25  Fine Maximum Step                  */
    uint32_t CSTEP:6;          /*!< bit: 26..31  Coarse Maximum Step                */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} OSCCTRL_DFLLMUL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DFLLMUL_OFFSET      0x20         /**< \brief (OSCCTRL_DFLLMUL offset) DFLL48M Multiplier */
#define OSCCTRL_DFLLMUL_RESETVALUE  0x00000000ul /**< \brief (OSCCTRL_DFLLMUL reset_value) DFLL48M Multiplier */

#define OSCCTRL_DFLLMUL_MUL_Pos     0            /**< \brief (OSCCTRL_DFLLMUL) DFLL Multiply Factor */
#define OSCCTRL_DFLLMUL_MUL_Msk     (0xFFFFul << OSCCTRL_DFLLMUL_MUL_Pos)
#define OSCCTRL_DFLLMUL_MUL(value)  (OSCCTRL_DFLLMUL_MUL_Msk & ((value) << OSCCTRL_DFLLMUL_MUL_Pos))
#define OSCCTRL_DFLLMUL_FSTEP_Pos   16           /**< \brief (OSCCTRL_DFLLMUL) Fine Maximum Step */
#define OSCCTRL_DFLLMUL_FSTEP_Msk   (0x3FFul << OSCCTRL_DFLLMUL_FSTEP_Pos)
#define OSCCTRL_DFLLMUL_FSTEP(value) (OSCCTRL_DFLLMUL_FSTEP_Msk & ((value) << OSCCTRL_DFLLMUL_FSTEP_Pos))
#define OSCCTRL_DFLLMUL_CSTEP_Pos   26           /**< \brief (OSCCTRL_DFLLMUL) Coarse Maximum Step */
#define OSCCTRL_DFLLMUL_CSTEP_Msk   (0x3Ful << OSCCTRL_DFLLMUL_CSTEP_Pos)
#define OSCCTRL_DFLLMUL_CSTEP(value) (OSCCTRL_DFLLMUL_CSTEP_Msk & ((value) << OSCCTRL_DFLLMUL_CSTEP_Pos))
#define OSCCTRL_DFLLMUL_MASK        0xFFFFFFFFul /**< \brief (OSCCTRL_DFLLMUL) MASK Register */

/* -------- OSCCTRL_DFLLSYNC : (OSCCTRL Offset: 0x24) (R/W  8) DFLL48M Synchronization -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  :7;               /*!< bit:  0.. 6  Reserved                           */
    uint8_t  READREQ:1;        /*!< bit:      7  Read Request                       */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} OSCCTRL_DFLLSYNC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DFLLSYNC_OFFSET     0x24         /**< \brief (OSCCTRL_DFLLSYNC offset) DFLL48M Synchronization */
#define OSCCTRL_DFLLSYNC_RESETVALUE 0x00ul       /**< \brief (OSCCTRL_DFLLSYNC reset_value) DFLL48M Synchronization */

#define OSCCTRL_DFLLSYNC_READREQ_Pos 7            /**< \brief (OSCCTRL_DFLLSYNC) Read Request */
#define OSCCTRL_DFLLSYNC_READREQ    (0x1ul << OSCCTRL_DFLLSYNC_READREQ_Pos)
#define OSCCTRL_DFLLSYNC_MASK       0x80ul       /**< \brief (OSCCTRL_DFLLSYNC) MASK Register */

/* -------- OSCCTRL_DPLLCTRLA : (OSCCTRL Offset: 0x28) (R/W  8) DPLL Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  :1;               /*!< bit:      0  Reserved                           */
    uint8_t  ENABLE:1;         /*!< bit:      1  Enable                             */
    uint8_t  :4;               /*!< bit:  2.. 5  Reserved                           */
    uint8_t  RUNSTDBY:1;       /*!< bit:      6  Run in Standby                     */
    uint8_t  ONDEMAND:1;       /*!< bit:      7  On Demand                          */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} OSCCTRL_DPLLCTRLA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DPLLCTRLA_OFFSET    0x28         /**< \brief (OSCCTRL_DPLLCTRLA offset) DPLL Control */
#define OSCCTRL_DPLLCTRLA_RESETVALUE 0x80ul       /**< \brief (OSCCTRL_DPLLCTRLA reset_value) DPLL Control */

#define OSCCTRL_DPLLCTRLA_ENABLE_Pos 1            /**< \brief (OSCCTRL_DPLLCTRLA) Enable */
#define OSCCTRL_DPLLCTRLA_ENABLE    (0x1ul << OSCCTRL_DPLLCTRLA_ENABLE_Pos)
#define OSCCTRL_DPLLCTRLA_RUNSTDBY_Pos 6            /**< \brief (OSCCTRL_DPLLCTRLA) Run in Standby */
#define OSCCTRL_DPLLCTRLA_RUNSTDBY  (0x1ul << OSCCTRL_DPLLCTRLA_RUNSTDBY_Pos)
#define OSCCTRL_DPLLCTRLA_ONDEMAND_Pos 7            /**< \brief (OSCCTRL_DPLLCTRLA) On Demand */
#define OSCCTRL_DPLLCTRLA_ONDEMAND  (0x1ul << OSCCTRL_DPLLCTRLA_ONDEMAND_Pos)
#define OSCCTRL_DPLLCTRLA_MASK      0xC2ul       /**< \brief (OSCCTRL_DPLLCTRLA) MASK Register */

/* -------- OSCCTRL_DPLLRATIO : (OSCCTRL Offset: 0x2C) (R/W 32) DPLL Ratio Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t LDR:12;           /*!< bit:  0..11  Loop Divider Ratio                 */
    uint32_t :4;               /*!< bit: 12..15  Reserved                           */
    uint32_t LDRFRAC:4;        /*!< bit: 16..19  Loop Divider Ratio Fractional Part */
    uint32_t :12;              /*!< bit: 20..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} OSCCTRL_DPLLRATIO_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DPLLRATIO_OFFSET    0x2C         /**< \brief (OSCCTRL_DPLLRATIO offset) DPLL Ratio Control */
#define OSCCTRL_DPLLRATIO_RESETVALUE 0x00000000ul /**< \brief (OSCCTRL_DPLLRATIO reset_value) DPLL Ratio Control */

#define OSCCTRL_DPLLRATIO_LDR_Pos   0            /**< \brief (OSCCTRL_DPLLRATIO) Loop Divider Ratio */
#define OSCCTRL_DPLLRATIO_LDR_Msk   (0xFFFul << OSCCTRL_DPLLRATIO_LDR_Pos)
#define OSCCTRL_DPLLRATIO_LDR(value) (OSCCTRL_DPLLRATIO_LDR_Msk & ((value) << OSCCTRL_DPLLRATIO_LDR_Pos))
#define OSCCTRL_DPLLRATIO_LDRFRAC_Pos 16           /**< \brief (OSCCTRL_DPLLRATIO) Loop Divider Ratio Fractional Part */
#define OSCCTRL_DPLLRATIO_LDRFRAC_Msk (0xFul << OSCCTRL_DPLLRATIO_LDRFRAC_Pos)
#define OSCCTRL_DPLLRATIO_LDRFRAC(value) (OSCCTRL_DPLLRATIO_LDRFRAC_Msk & ((value) << OSCCTRL_DPLLRATIO_LDRFRAC_Pos))
#define OSCCTRL_DPLLRATIO_MASK      0x000F0FFFul /**< \brief (OSCCTRL_DPLLRATIO) MASK Register */

/* -------- OSCCTRL_DPLLCTRLB : (OSCCTRL Offset: 0x30) (R/W 32) Digital Core Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t FILTER:2;         /*!< bit:  0.. 1  Proportional Integral Filter Selection */
    uint32_t LPEN:1;           /*!< bit:      2  Low-Power Enable                   */
    uint32_t WUF:1;            /*!< bit:      3  Wake Up Fast                       */
    uint32_t REFCLK:2;         /*!< bit:  4.. 5  Reference Clock Selection          */
    uint32_t :2;               /*!< bit:  6.. 7  Reserved                           */
    uint32_t LTIME:3;          /*!< bit:  8..10  Lock Time                          */
    uint32_t :1;               /*!< bit:     11  Reserved                           */
    uint32_t LBYPASS:1;        /*!< bit:     12  Lock Bypass                        */
    uint32_t :3;               /*!< bit: 13..15  Reserved                           */
    uint32_t DIV:11;           /*!< bit: 16..26  Clock Divider                      */
    uint32_t :5;               /*!< bit: 27..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} OSCCTRL_DPLLCTRLB_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DPLLCTRLB_OFFSET    0x30         /**< \brief (OSCCTRL_DPLLCTRLB offset) Digital Core Configuration */
#define OSCCTRL_DPLLCTRLB_RESETVALUE 0x00000000ul /**< \brief (OSCCTRL_DPLLCTRLB reset_value) Digital Core Configuration */

#define OSCCTRL_DPLLCTRLB_FILTER_Pos 0            /**< \brief (OSCCTRL_DPLLCTRLB) Proportional Integral Filter Selection */
#define OSCCTRL_DPLLCTRLB_FILTER_Msk (0x3ul << OSCCTRL_DPLLCTRLB_FILTER_Pos)
#define OSCCTRL_DPLLCTRLB_FILTER(value) (OSCCTRL_DPLLCTRLB_FILTER_Msk & ((value) << OSCCTRL_DPLLCTRLB_FILTER_Pos))
#define OSCCTRL_DPLLCTRLB_LPEN_Pos  2            /**< \brief (OSCCTRL_DPLLCTRLB) Low-Power Enable */
#define OSCCTRL_DPLLCTRLB_LPEN      (0x1ul << OSCCTRL_DPLLCTRLB_LPEN_Pos)
#define OSCCTRL_DPLLCTRLB_WUF_Pos   3            /**< \brief (OSCCTRL_DPLLCTRLB) Wake Up Fast */
#define OSCCTRL_DPLLCTRLB_WUF       (0x1ul << OSCCTRL_DPLLCTRLB_WUF_Pos)
#define OSCCTRL_DPLLCTRLB_REFCLK_Pos 4            /**< \brief (OSCCTRL_DPLLCTRLB) Reference Clock Selection */
#define OSCCTRL_DPLLCTRLB_REFCLK_Msk (0x3ul << OSCCTRL_DPLLCTRLB_REFCLK_Pos)
#define OSCCTRL_DPLLCTRLB_REFCLK(value) (OSCCTRL_DPLLCTRLB_REFCLK_Msk & ((value) << OSCCTRL_DPLLCTRLB_REFCLK_Pos))
#define OSCCTRL_DPLLCTRLB_LTIME_Pos 8            /**< \brief (OSCCTRL_DPLLCTRLB) Lock Time */
#define OSCCTRL_DPLLCTRLB_LTIME_Msk (0x7ul << OSCCTRL_DPLLCTRLB_LTIME_Pos)
#define OSCCTRL_DPLLCTRLB_LTIME(value) (OSCCTRL_DPLLCTRLB_LTIME_Msk & ((value) << OSCCTRL_DPLLCTRLB_LTIME_Pos))
#define OSCCTRL_DPLLCTRLB_LBYPASS_Pos 12           /**< \brief (OSCCTRL_DPLLCTRLB) Lock Bypass */
#define OSCCTRL_DPLLCTRLB_LBYPASS   (0x1ul << OSCCTRL_DPLLCTRLB_LBYPASS_Pos)
#define OSCCTRL_DPLLCTRLB_DIV_Pos   16           /**< \brief (OSCCTRL_DPLLCTRLB) Clock Divider */
#define OSCCTRL_DPLLCTRLB_DIV_Msk   (0x7FFul << OSCCTRL_DPLLCTRLB_DIV_Pos)
#define OSCCTRL_DPLLCTRLB_DIV(value) (OSCCTRL_DPLLCTRLB_DIV_Msk & ((value) << OSCCTRL_DPLLCTRLB_DIV_Pos))
#define OSCCTRL_DPLLCTRLB_MASK      0x07FF173Ful /**< \brief (OSCCTRL_DPLLCTRLB) MASK Register */

/* -------- OSCCTRL_DPLLPRESC : (OSCCTRL Offset: 0x34) (R/W  8) DPLL Prescaler -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  PRESC:2;          /*!< bit:  0.. 1  Output Clock Prescaler             */
    uint8_t  :6;               /*!< bit:  2.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} OSCCTRL_DPLLPRESC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DPLLPRESC_OFFSET    0x34         /**< \brief (OSCCTRL_DPLLPRESC offset) DPLL Prescaler */
#define OSCCTRL_DPLLPRESC_RESETVALUE 0x00ul       /**< \brief (OSCCTRL_DPLLPRESC reset_value) DPLL Prescaler */

#define OSCCTRL_DPLLPRESC_PRESC_Pos 0            /**< \brief (OSCCTRL_DPLLPRESC) Output Clock Prescaler */
#define OSCCTRL_DPLLPRESC_PRESC_Msk (0x3ul << OSCCTRL_DPLLPRESC_PRESC_Pos)
#define OSCCTRL_DPLLPRESC_PRESC(value) (OSCCTRL_DPLLPRESC_PRESC_Msk & ((value) << OSCCTRL_DPLLPRESC_PRESC_Pos))
#define   OSCCTRL_DPLLPRESC_PRESC_DIV1_Val 0x0ul  /**< \brief (OSCCTRL_DPLLPRESC) DPLL output is divided by 1 */
#define   OSCCTRL_DPLLPRESC_PRESC_DIV2_Val 0x1ul  /**< \brief (OSCCTRL_DPLLPRESC) DPLL output is divided by 2 */
#define   OSCCTRL_DPLLPRESC_PRESC_DIV4_Val 0x2ul  /**< \brief (OSCCTRL_DPLLPRESC) DPLL output is divided by 4 */
#define OSCCTRL_DPLLPRESC_PRESC_DIV1 (OSCCTRL_DPLLPRESC_PRESC_DIV1_Val << OSCCTRL_DPLLPRESC_PRESC_Pos)
#define OSCCTRL_DPLLPRESC_PRESC_DIV2 (OSCCTRL_DPLLPRESC_PRESC_DIV2_Val << OSCCTRL_DPLLPRESC_PRESC_Pos)
#define OSCCTRL_DPLLPRESC_PRESC_DIV4 (OSCCTRL_DPLLPRESC_PRESC_DIV4_Val << OSCCTRL_DPLLPRESC_PRESC_Pos)
#define OSCCTRL_DPLLPRESC_MASK      0x03ul       /**< \brief (OSCCTRL_DPLLPRESC) MASK Register */

/* -------- OSCCTRL_DPLLSYNCBUSY : (OSCCTRL Offset: 0x38) (R/   8) DPLL Synchronization Busy -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  :1;               /*!< bit:      0  Reserved                           */
    uint8_t  ENABLE:1;         /*!< bit:      1  DPLL Enable Synchronization Status */
    uint8_t  DPLLRATIO:1;      /*!< bit:      2  DPLL Ratio Synchronization Status  */
    uint8_t  DPLLPRESC:1;      /*!< bit:      3  DPLL Prescaler Synchronization Status */
    uint8_t  :4;               /*!< bit:  4.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} OSCCTRL_DPLLSYNCBUSY_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DPLLSYNCBUSY_OFFSET 0x38         /**< \brief (OSCCTRL_DPLLSYNCBUSY offset) DPLL Synchronization Busy */
#define OSCCTRL_DPLLSYNCBUSY_RESETVALUE 0x00ul       /**< \brief (OSCCTRL_DPLLSYNCBUSY reset_value) DPLL Synchronization Busy */

#define OSCCTRL_DPLLSYNCBUSY_ENABLE_Pos 1            /**< \brief (OSCCTRL_DPLLSYNCBUSY) DPLL Enable Synchronization Status */
#define OSCCTRL_DPLLSYNCBUSY_ENABLE (0x1ul << OSCCTRL_DPLLSYNCBUSY_ENABLE_Pos)
#define OSCCTRL_DPLLSYNCBUSY_DPLLRATIO_Pos 2            /**< \brief (OSCCTRL_DPLLSYNCBUSY) DPLL Ratio Synchronization Status */
#define OSCCTRL_DPLLSYNCBUSY_DPLLRATIO (0x1ul << OSCCTRL_DPLLSYNCBUSY_DPLLRATIO_Pos)
#define OSCCTRL_DPLLSYNCBUSY_DPLLPRESC_Pos 3            /**< \brief (OSCCTRL_DPLLSYNCBUSY) DPLL Prescaler Synchronization Status */
#define OSCCTRL_DPLLSYNCBUSY_DPLLPRESC (0x1ul << OSCCTRL_DPLLSYNCBUSY_DPLLPRESC_Pos)
#define OSCCTRL_DPLLSYNCBUSY_MASK   0x0Eul       /**< \brief (OSCCTRL_DPLLSYNCBUSY) MASK Register */

/* -------- OSCCTRL_DPLLSTATUS : (OSCCTRL Offset: 0x3C) (R/   8) DPLL Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  LOCK:1;           /*!< bit:      0  DPLL Lock Status                   */
    uint8_t  CLKRDY:1;         /*!< bit:      1  DPLL Clock Ready                   */
    uint8_t  :6;               /*!< bit:  2.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} OSCCTRL_DPLLSTATUS_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define OSCCTRL_DPLLSTATUS_OFFSET   0x3C         /**< \brief (OSCCTRL_DPLLSTATUS offset) DPLL Status */
#define OSCCTRL_DPLLSTATUS_RESETVALUE 0x00ul       /**< \brief (OSCCTRL_DPLLSTATUS reset_value) DPLL Status */

#define OSCCTRL_DPLLSTATUS_LOCK_Pos 0            /**< \brief (OSCCTRL_DPLLSTATUS) DPLL Lock Status */
#define OSCCTRL_DPLLSTATUS_LOCK     (0x1ul << OSCCTRL_DPLLSTATUS_LOCK_Pos)
#define OSCCTRL_DPLLSTATUS_CLKRDY_Pos 1            /**< \brief (OSCCTRL_DPLLSTATUS) DPLL Clock Ready */
#define OSCCTRL_DPLLSTATUS_CLKRDY   (0x1ul << OSCCTRL_DPLLSTATUS_CLKRDY_Pos)
#define OSCCTRL_DPLLSTATUS_MASK     0x03ul       /**< \brief (OSCCTRL_DPLLSTATUS) MASK Register */

/** \brief OSCCTRL hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO OSCCTRL_INTENCLR_Type     INTENCLR;    /**< \brief Offset: 0x00 (R/W 32) Interrupt Enable Clear */
  __IO OSCCTRL_INTENSET_Type     INTENSET;    /**< \brief Offset: 0x04 (R/W 32) Interrupt Enable Set */
  __IO OSCCTRL_INTFLAG_Type      INTFLAG;     /**< \brief Offset: 0x08 (R/W 32) Interrupt Flag Status and Clear */
  __I  OSCCTRL_STATUS_Type       STATUS;      /**< \brief Offset: 0x0C (R/  32) Power and Clocks Status */
  __IO OSCCTRL_XOSCCTRL_Type     XOSCCTRL;    /**< \brief Offset: 0x10 (R/W 16) External Multipurpose Crystal Oscillator (XOSC) Control */
       RoReg8                    Reserved1[0x2];
  __IO OSCCTRL_OSC16MCTRL_Type   OSC16MCTRL;  /**< \brief Offset: 0x14 (R/W  8) 16MHz Internal Oscillator (OSC16M) Control */
       RoReg8                    Reserved2[0x3];
  __IO OSCCTRL_DFLLCTRL_Type     DFLLCTRL;    /**< \brief Offset: 0x18 (R/W 16) DFLL48M Control */
       RoReg8                    Reserved3[0x2];
  __IO OSCCTRL_DFLLVAL_Type      DFLLVAL;     /**< \brief Offset: 0x1C (R/W 32) DFLL48M Value */
  __IO OSCCTRL_DFLLMUL_Type      DFLLMUL;     /**< \brief Offset: 0x20 (R/W 32) DFLL48M Multiplier */
  __IO OSCCTRL_DFLLSYNC_Type     DFLLSYNC;    /**< \brief Offset: 0x24 (R/W  8) DFLL48M Synchronization */
       RoReg8                    Reserved4[0x3];
  __IO OSCCTRL_DPLLCTRLA_Type    DPLLCTRLA;   /**< \brief Offset: 0x28 (R/W  8) DPLL Control */
       RoReg8                    Reserved5[0x3];
  __IO OSCCTRL_DPLLRATIO_Type    DPLLRATIO;   /**< \brief Offset: 0x2C (R/W 32) DPLL Ratio Control */
  __IO OSCCTRL_DPLLCTRLB_Type    DPLLCTRLB;   /**< \brief Offset: 0x30 (R/W 32) Digital Core Configuration */
  __IO OSCCTRL_DPLLPRESC_Type    DPLLPRESC;   /**< \brief Offset: 0x34 (R/W  8) DPLL Prescaler */
       RoReg8                    Reserved6[0x3];
  __I  OSCCTRL_DPLLSYNCBUSY_Type DPLLSYNCBUSY; /**< \brief Offset: 0x38 (R/   8) DPLL Synchronization Busy */
       RoReg8                    Reserved7[0x3];
  __I  OSCCTRL_DPLLSTATUS_Type   DPLLSTATUS;  /**< \brief Offset: 0x3C (R/   8) DPLL Status */
} Oscctrl;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/*@}*/

#endif /* _SAMR30_OSCCTRL_COMPONENT_ */
#ifndef _SAMR30_NVMCTRL_COMPONENT_
#define _SAMR30_NVMCTRL_COMPONENT_

/* ========================================================================== */
/**  SOFTWARE API DEFINITION FOR NVMCTRL */
/* ========================================================================== */
/** \addtogroup SAMR30_NVMCTRL Non-Volatile Memory Controller */
/*@{*/

#define NVMCTRL_U2207
#define REV_NVMCTRL                 0x302

/* -------- NVMCTRL_CTRLA : (NVMCTRL Offset: 0x00) (R/W 16) Control A -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint16_t CMD:7;            /*!< bit:  0.. 6  Command                            */
    uint16_t :1;               /*!< bit:      7  Reserved                           */
    uint16_t CMDEX:8;          /*!< bit:  8..15  Command Execution                  */
  } bit;                       /*!< Structure used for bit  access                  */
  uint16_t reg;                /*!< Type      used for register access              */
} NVMCTRL_CTRLA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define NVMCTRL_CTRLA_OFFSET        0x00         /**< \brief (NVMCTRL_CTRLA offset) Control A */
#define NVMCTRL_CTRLA_RESETVALUE    0x0000ul     /**< \brief (NVMCTRL_CTRLA reset_value) Control A */

#define NVMCTRL_CTRLA_CMD_Pos       0            /**< \brief (NVMCTRL_CTRLA) Command */
#define NVMCTRL_CTRLA_CMD_Msk       (0x7Ful << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD(value)    (NVMCTRL_CTRLA_CMD_Msk & ((value) << NVMCTRL_CTRLA_CMD_Pos))
#define   NVMCTRL_CTRLA_CMD_ER_Val        0x2ul  /**< \brief (NVMCTRL_CTRLA) Erase Row - Erases the row addressed by the ADDR register. */
#define   NVMCTRL_CTRLA_CMD_WP_Val        0x4ul  /**< \brief (NVMCTRL_CTRLA) Write Page - Writes the contents of the page buffer to the page addressed by the ADDR register. */
#define   NVMCTRL_CTRLA_CMD_EAR_Val       0x5ul  /**< \brief (NVMCTRL_CTRLA) Erase Auxiliary Row - Erases the auxiliary row addressed by the ADDR register. This command can be given only when the security bit is not set and only to the user configuration row. */
#define   NVMCTRL_CTRLA_CMD_WAP_Val       0x6ul  /**< \brief (NVMCTRL_CTRLA) Write Auxiliary Page - Writes the contents of the page buffer to the page addressed by the ADDR register. This command can be given only when the security bit is not set and only to the user configuration row. */
#define   NVMCTRL_CTRLA_CMD_SF_Val        0xAul  /**< \brief (NVMCTRL_CTRLA) Security Flow Command */
#define   NVMCTRL_CTRLA_CMD_WL_Val        0xFul  /**< \brief (NVMCTRL_CTRLA) Write lockbits */
#define   NVMCTRL_CTRLA_CMD_RWWEEER_Val   0x1Aul  /**< \brief (NVMCTRL_CTRLA) RWW EEPROM area Erase Row - Erases the row addressed by the ADDR register. */
#define   NVMCTRL_CTRLA_CMD_RWWEEWP_Val   0x1Cul  /**< \brief (NVMCTRL_CTRLA) RWW EEPROM Write Page - Writes the contents of the page buffer to the page addressed by the ADDR register. */
#define   NVMCTRL_CTRLA_CMD_LR_Val        0x40ul  /**< \brief (NVMCTRL_CTRLA) Lock Region - Locks the region containing the address location in the ADDR register. */
#define   NVMCTRL_CTRLA_CMD_UR_Val        0x41ul  /**< \brief (NVMCTRL_CTRLA) Unlock Region - Unlocks the region containing the address location in the ADDR register. */
#define   NVMCTRL_CTRLA_CMD_SPRM_Val      0x42ul  /**< \brief (NVMCTRL_CTRLA) Sets the power reduction mode. */
#define   NVMCTRL_CTRLA_CMD_CPRM_Val      0x43ul  /**< \brief (NVMCTRL_CTRLA) Clears the power reduction mode. */
#define   NVMCTRL_CTRLA_CMD_PBC_Val       0x44ul  /**< \brief (NVMCTRL_CTRLA) Page Buffer Clear - Clears the page buffer. */
#define   NVMCTRL_CTRLA_CMD_SSB_Val       0x45ul  /**< \brief (NVMCTRL_CTRLA) Set Security Bit - Sets the security bit by writing 0x00 to the first byte in the lockbit row. */
#define   NVMCTRL_CTRLA_CMD_INVALL_Val    0x46ul  /**< \brief (NVMCTRL_CTRLA) Invalidate all cache lines. */
#define NVMCTRL_CTRLA_CMD_ER        (NVMCTRL_CTRLA_CMD_ER_Val      << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_WP        (NVMCTRL_CTRLA_CMD_WP_Val      << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_EAR       (NVMCTRL_CTRLA_CMD_EAR_Val     << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_WAP       (NVMCTRL_CTRLA_CMD_WAP_Val     << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_SF        (NVMCTRL_CTRLA_CMD_SF_Val      << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_WL        (NVMCTRL_CTRLA_CMD_WL_Val      << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_RWWEEER   (NVMCTRL_CTRLA_CMD_RWWEEER_Val << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_RWWEEWP   (NVMCTRL_CTRLA_CMD_RWWEEWP_Val << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_LR        (NVMCTRL_CTRLA_CMD_LR_Val      << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_UR        (NVMCTRL_CTRLA_CMD_UR_Val      << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_SPRM      (NVMCTRL_CTRLA_CMD_SPRM_Val    << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_CPRM      (NVMCTRL_CTRLA_CMD_CPRM_Val    << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_PBC       (NVMCTRL_CTRLA_CMD_PBC_Val     << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_SSB       (NVMCTRL_CTRLA_CMD_SSB_Val     << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMD_INVALL    (NVMCTRL_CTRLA_CMD_INVALL_Val  << NVMCTRL_CTRLA_CMD_Pos)
#define NVMCTRL_CTRLA_CMDEX_Pos     8            /**< \brief (NVMCTRL_CTRLA) Command Execution */
#define NVMCTRL_CTRLA_CMDEX_Msk     (0xFFul << NVMCTRL_CTRLA_CMDEX_Pos)
#define NVMCTRL_CTRLA_CMDEX(value)  (NVMCTRL_CTRLA_CMDEX_Msk & ((value) << NVMCTRL_CTRLA_CMDEX_Pos))
#define   NVMCTRL_CTRLA_CMDEX_KEY_Val     0xA5ul  /**< \brief (NVMCTRL_CTRLA) Execution Key */
#define NVMCTRL_CTRLA_CMDEX_KEY     (NVMCTRL_CTRLA_CMDEX_KEY_Val   << NVMCTRL_CTRLA_CMDEX_Pos)
#define NVMCTRL_CTRLA_MASK          0xFF7Ful     /**< \brief (NVMCTRL_CTRLA) MASK Register */

/* -------- NVMCTRL_CTRLB : (NVMCTRL Offset: 0x04) (R/W 32) Control B -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t :1;               /*!< bit:      0  Reserved                           */
    uint32_t RWS:4;            /*!< bit:  1.. 4  NVM Read Wait States               */
    uint32_t :2;               /*!< bit:  5.. 6  Reserved                           */
    uint32_t MANW:1;           /*!< bit:      7  Manual Write                       */
    uint32_t SLEEPPRM:2;       /*!< bit:  8.. 9  Power Reduction Mode during Sleep  */
    uint32_t :1;               /*!< bit:     10  Reserved                           */
    uint32_t FWUP:1;           /*!< bit:     11  fast wake-up                       */
    uint32_t :4;               /*!< bit: 12..15  Reserved                           */
    uint32_t READMODE:2;       /*!< bit: 16..17  NVMCTRL Read Mode                  */
    uint32_t CACHEDIS:1;       /*!< bit:     18  Cache Disable                      */
    uint32_t :13;              /*!< bit: 19..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} NVMCTRL_CTRLB_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define NVMCTRL_CTRLB_OFFSET        0x04         /**< \brief (NVMCTRL_CTRLB offset) Control B */
#define NVMCTRL_CTRLB_RESETVALUE    0x00000080ul /**< \brief (NVMCTRL_CTRLB reset_value) Control B */

#define NVMCTRL_CTRLB_RWS_Pos       1            /**< \brief (NVMCTRL_CTRLB) NVM Read Wait States */
#define NVMCTRL_CTRLB_RWS_Msk       (0xFul << NVMCTRL_CTRLB_RWS_Pos)
#define NVMCTRL_CTRLB_RWS(value)    (NVMCTRL_CTRLB_RWS_Msk & ((value) << NVMCTRL_CTRLB_RWS_Pos))
#define   NVMCTRL_CTRLB_RWS_SINGLE_Val    0x0ul  /**< \brief (NVMCTRL_CTRLB) Single Auto Wait State */
#define   NVMCTRL_CTRLB_RWS_HALF_Val      0x1ul  /**< \brief (NVMCTRL_CTRLB) Half Auto Wait State */
#define   NVMCTRL_CTRLB_RWS_DUAL_Val      0x2ul  /**< \brief (NVMCTRL_CTRLB) Dual Auto Wait State */
#define NVMCTRL_CTRLB_RWS_SINGLE    (NVMCTRL_CTRLB_RWS_SINGLE_Val  << NVMCTRL_CTRLB_RWS_Pos)
#define NVMCTRL_CTRLB_RWS_HALF      (NVMCTRL_CTRLB_RWS_HALF_Val    << NVMCTRL_CTRLB_RWS_Pos)
#define NVMCTRL_CTRLB_RWS_DUAL      (NVMCTRL_CTRLB_RWS_DUAL_Val    << NVMCTRL_CTRLB_RWS_Pos)
#define NVMCTRL_CTRLB_MANW_Pos      7            /**< \brief (NVMCTRL_CTRLB) Manual Write */
#define NVMCTRL_CTRLB_MANW          (0x1ul << NVMCTRL_CTRLB_MANW_Pos)
#define NVMCTRL_CTRLB_SLEEPPRM_Pos  8            /**< \brief (NVMCTRL_CTRLB) Power Reduction Mode during Sleep */
#define NVMCTRL_CTRLB_SLEEPPRM_Msk  (0x3ul << NVMCTRL_CTRLB_SLEEPPRM_Pos)
#define NVMCTRL_CTRLB_SLEEPPRM(value) (NVMCTRL_CTRLB_SLEEPPRM_Msk & ((value) << NVMCTRL_CTRLB_SLEEPPRM_Pos))
#define   NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS_Val 0x0ul  /**< \brief (NVMCTRL_CTRLB) NVM block enters low-power mode when entering sleep.NVM block exits low-power mode upon first access. */
#define   NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT_Val 0x1ul  /**< \brief (NVMCTRL_CTRLB) NVM block enters low-power mode when entering sleep.NVM block exits low-power mode when exiting sleep. */
#define   NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val 0x3ul  /**< \brief (NVMCTRL_CTRLB) Auto power reduction disabled. */
#define NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS (NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS_Val << NVMCTRL_CTRLB_SLEEPPRM_Pos)
#define NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT (NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT_Val << NVMCTRL_CTRLB_SLEEPPRM_Pos)
#define NVMCTRL_CTRLB_SLEEPPRM_DISABLED (NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val << NVMCTRL_CTRLB_SLEEPPRM_Pos)
#define NVMCTRL_CTRLB_FWUP_Pos      11           /**< \brief (NVMCTRL_CTRLB) fast wake-up */
#define NVMCTRL_CTRLB_FWUP          (0x1ul << NVMCTRL_CTRLB_FWUP_Pos)
#define NVMCTRL_CTRLB_READMODE_Pos  16           /**< \brief (NVMCTRL_CTRLB) NVMCTRL Read Mode */
#define NVMCTRL_CTRLB_READMODE_Msk  (0x3ul << NVMCTRL_CTRLB_READMODE_Pos)
#define NVMCTRL_CTRLB_READMODE(value) (NVMCTRL_CTRLB_READMODE_Msk & ((value) << NVMCTRL_CTRLB_READMODE_Pos))
#define   NVMCTRL_CTRLB_READMODE_NO_MISS_PENALTY_Val 0x0ul  /**< \brief (NVMCTRL_CTRLB) The NVM Controller (cache system) does not insert wait states on a cache miss. Gives the best system performance. */
#define   NVMCTRL_CTRLB_READMODE_LOW_POWER_Val 0x1ul  /**< \brief (NVMCTRL_CTRLB) Reduces power consumption of the cache system, but inserts a wait state each time there is a cache miss. This mode may not be relevant if CPU performance is required, as the application will be stalled and may lead to increase run time. */
#define   NVMCTRL_CTRLB_READMODE_DETERMINISTIC_Val 0x2ul  /**< \brief (NVMCTRL_CTRLB) The cache system ensures that a cache hit or miss takes the same amount of time, determined by the number of programmed flash wait states. This mode can be used for real-time applications that require deterministic execution timings. */
#define NVMCTRL_CTRLB_READMODE_NO_MISS_PENALTY (NVMCTRL_CTRLB_READMODE_NO_MISS_PENALTY_Val << NVMCTRL_CTRLB_READMODE_Pos)
#define NVMCTRL_CTRLB_READMODE_LOW_POWER (NVMCTRL_CTRLB_READMODE_LOW_POWER_Val << NVMCTRL_CTRLB_READMODE_Pos)
#define NVMCTRL_CTRLB_READMODE_DETERMINISTIC (NVMCTRL_CTRLB_READMODE_DETERMINISTIC_Val << NVMCTRL_CTRLB_READMODE_Pos)
#define NVMCTRL_CTRLB_CACHEDIS_Pos  18           /**< \brief (NVMCTRL_CTRLB) Cache Disable */
#define NVMCTRL_CTRLB_CACHEDIS      (0x1ul << NVMCTRL_CTRLB_CACHEDIS_Pos)
#define NVMCTRL_CTRLB_MASK          0x00070B9Eul /**< \brief (NVMCTRL_CTRLB) MASK Register */

/* -------- NVMCTRL_PARAM : (NVMCTRL Offset: 0x08) (R/W 32) NVM Parameter -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t NVMP:16;          /*!< bit:  0..15  NVM Pages                          */
    uint32_t PSZ:3;            /*!< bit: 16..18  Page Size                          */
    uint32_t :1;               /*!< bit:     19  Reserved                           */
    uint32_t RWWEEP:12;        /*!< bit: 20..31  RWW EEPROM Pages                   */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} NVMCTRL_PARAM_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define NVMCTRL_PARAM_OFFSET        0x08         /**< \brief (NVMCTRL_PARAM offset) NVM Parameter */
#define NVMCTRL_PARAM_RESETVALUE    0x00000000ul /**< \brief (NVMCTRL_PARAM reset_value) NVM Parameter */

#define NVMCTRL_PARAM_NVMP_Pos      0            /**< \brief (NVMCTRL_PARAM) NVM Pages */
#define NVMCTRL_PARAM_NVMP_Msk      (0xFFFFul << NVMCTRL_PARAM_NVMP_Pos)
#define NVMCTRL_PARAM_NVMP(value)   (NVMCTRL_PARAM_NVMP_Msk & ((value) << NVMCTRL_PARAM_NVMP_Pos))
#define NVMCTRL_PARAM_PSZ_Pos       16           /**< \brief (NVMCTRL_PARAM) Page Size */
#define NVMCTRL_PARAM_PSZ_Msk       (0x7ul << NVMCTRL_PARAM_PSZ_Pos)
#define NVMCTRL_PARAM_PSZ(value)    (NVMCTRL_PARAM_PSZ_Msk & ((value) << NVMCTRL_PARAM_PSZ_Pos))
#define   NVMCTRL_PARAM_PSZ_8_Val         0x0ul  /**< \brief (NVMCTRL_PARAM) 8 bytes */
#define   NVMCTRL_PARAM_PSZ_16_Val        0x1ul  /**< \brief (NVMCTRL_PARAM) 16 bytes */
#define   NVMCTRL_PARAM_PSZ_32_Val        0x2ul  /**< \brief (NVMCTRL_PARAM) 32 bytes */
#define   NVMCTRL_PARAM_PSZ_64_Val        0x3ul  /**< \brief (NVMCTRL_PARAM) 64 bytes */
#define   NVMCTRL_PARAM_PSZ_128_Val       0x4ul  /**< \brief (NVMCTRL_PARAM) 128 bytes */
#define   NVMCTRL_PARAM_PSZ_256_Val       0x5ul  /**< \brief (NVMCTRL_PARAM) 256 bytes */
#define   NVMCTRL_PARAM_PSZ_512_Val       0x6ul  /**< \brief (NVMCTRL_PARAM) 512 bytes */
#define   NVMCTRL_PARAM_PSZ_1024_Val      0x7ul  /**< \brief (NVMCTRL_PARAM) 1024 bytes */
#define NVMCTRL_PARAM_PSZ_8         (NVMCTRL_PARAM_PSZ_8_Val       << NVMCTRL_PARAM_PSZ_Pos)
#define NVMCTRL_PARAM_PSZ_16        (NVMCTRL_PARAM_PSZ_16_Val      << NVMCTRL_PARAM_PSZ_Pos)
#define NVMCTRL_PARAM_PSZ_32        (NVMCTRL_PARAM_PSZ_32_Val      << NVMCTRL_PARAM_PSZ_Pos)
#define NVMCTRL_PARAM_PSZ_64        (NVMCTRL_PARAM_PSZ_64_Val      << NVMCTRL_PARAM_PSZ_Pos)
#define NVMCTRL_PARAM_PSZ_128       (NVMCTRL_PARAM_PSZ_128_Val     << NVMCTRL_PARAM_PSZ_Pos)
#define NVMCTRL_PARAM_PSZ_256       (NVMCTRL_PARAM_PSZ_256_Val     << NVMCTRL_PARAM_PSZ_Pos)
#define NVMCTRL_PARAM_PSZ_512       (NVMCTRL_PARAM_PSZ_512_Val     << NVMCTRL_PARAM_PSZ_Pos)
#define NVMCTRL_PARAM_PSZ_1024      (NVMCTRL_PARAM_PSZ_1024_Val    << NVMCTRL_PARAM_PSZ_Pos)
#define NVMCTRL_PARAM_RWWEEP_Pos    20           /**< \brief (NVMCTRL_PARAM) RWW EEPROM Pages */
#define NVMCTRL_PARAM_RWWEEP_Msk    (0xFFFul << NVMCTRL_PARAM_RWWEEP_Pos)
#define NVMCTRL_PARAM_RWWEEP(value) (NVMCTRL_PARAM_RWWEEP_Msk & ((value) << NVMCTRL_PARAM_RWWEEP_Pos))
#define NVMCTRL_PARAM_MASK          0xFFF7FFFFul /**< \brief (NVMCTRL_PARAM) MASK Register */

/* -------- NVMCTRL_INTENCLR : (NVMCTRL Offset: 0x0C) (R/W  8) Interrupt Enable Clear -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  READY:1;          /*!< bit:      0  NVM Ready Interrupt Enable         */
    uint8_t  ERROR:1;          /*!< bit:      1  Error Interrupt Enable             */
    uint8_t  :6;               /*!< bit:  2.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} NVMCTRL_INTENCLR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define NVMCTRL_INTENCLR_OFFSET     0x0C         /**< \brief (NVMCTRL_INTENCLR offset) Interrupt Enable Clear */
#define NVMCTRL_INTENCLR_RESETVALUE 0x00ul       /**< \brief (NVMCTRL_INTENCLR reset_value) Interrupt Enable Clear */

#define NVMCTRL_INTENCLR_READY_Pos  0            /**< \brief (NVMCTRL_INTENCLR) NVM Ready Interrupt Enable */
#define NVMCTRL_INTENCLR_READY      (0x1ul << NVMCTRL_INTENCLR_READY_Pos)
#define NVMCTRL_INTENCLR_ERROR_Pos  1            /**< \brief (NVMCTRL_INTENCLR) Error Interrupt Enable */
#define NVMCTRL_INTENCLR_ERROR      (0x1ul << NVMCTRL_INTENCLR_ERROR_Pos)
#define NVMCTRL_INTENCLR_MASK       0x03ul       /**< \brief (NVMCTRL_INTENCLR) MASK Register */

/* -------- NVMCTRL_INTENSET : (NVMCTRL Offset: 0x10) (R/W  8) Interrupt Enable Set -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint8_t  READY:1;          /*!< bit:      0  NVM Ready Interrupt Enable         */
    uint8_t  ERROR:1;          /*!< bit:      1  Error Interrupt Enable             */
    uint8_t  :6;               /*!< bit:  2.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} NVMCTRL_INTENSET_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define NVMCTRL_INTENSET_OFFSET     0x10         /**< \brief (NVMCTRL_INTENSET offset) Interrupt Enable Set */
#define NVMCTRL_INTENSET_RESETVALUE 0x00ul       /**< \brief (NVMCTRL_INTENSET reset_value) Interrupt Enable Set */

#define NVMCTRL_INTENSET_READY_Pos  0            /**< \brief (NVMCTRL_INTENSET) NVM Ready Interrupt Enable */
#define NVMCTRL_INTENSET_READY      (0x1ul << NVMCTRL_INTENSET_READY_Pos)
#define NVMCTRL_INTENSET_ERROR_Pos  1            /**< \brief (NVMCTRL_INTENSET) Error Interrupt Enable */
#define NVMCTRL_INTENSET_ERROR      (0x1ul << NVMCTRL_INTENSET_ERROR_Pos)
#define NVMCTRL_INTENSET_MASK       0x03ul       /**< \brief (NVMCTRL_INTENSET) MASK Register */

/* -------- NVMCTRL_INTFLAG : (NVMCTRL Offset: 0x14) (R/W  8) Interrupt Flag Status and Clear -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { // __I to avoid read-modify-write on write-to-clear register
  struct {
    __I uint8_t  READY:1;          /*!< bit:      0  NVM Ready                          */
    __I uint8_t  ERROR:1;          /*!< bit:      1  Error                              */
    __I uint8_t  :6;               /*!< bit:  2.. 7  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint8_t reg;                 /*!< Type      used for register access              */
} NVMCTRL_INTFLAG_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define NVMCTRL_INTFLAG_OFFSET      0x14         /**< \brief (NVMCTRL_INTFLAG offset) Interrupt Flag Status and Clear */
#define NVMCTRL_INTFLAG_RESETVALUE  0x00ul       /**< \brief (NVMCTRL_INTFLAG reset_value) Interrupt Flag Status and Clear */

#define NVMCTRL_INTFLAG_READY_Pos   0            /**< \brief (NVMCTRL_INTFLAG) NVM Ready */
#define NVMCTRL_INTFLAG_READY       (0x1ul << NVMCTRL_INTFLAG_READY_Pos)
#define NVMCTRL_INTFLAG_ERROR_Pos   1            /**< \brief (NVMCTRL_INTFLAG) Error */
#define NVMCTRL_INTFLAG_ERROR       (0x1ul << NVMCTRL_INTFLAG_ERROR_Pos)
#define NVMCTRL_INTFLAG_MASK        0x03ul       /**< \brief (NVMCTRL_INTFLAG) MASK Register */

/* -------- NVMCTRL_STATUS : (NVMCTRL Offset: 0x18) (R/W 16) Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint16_t PRM:1;            /*!< bit:      0  Power Reduction Mode               */
    uint16_t LOAD:1;           /*!< bit:      1  NVM Page Buffer Active Loading     */
    uint16_t PROGE:1;          /*!< bit:      2  Programming Error Status           */
    uint16_t LOCKE:1;          /*!< bit:      3  Lock Error Status                  */
    uint16_t NVME:1;           /*!< bit:      4  NVM Error                          */
    uint16_t :3;               /*!< bit:  5.. 7  Reserved                           */
    uint16_t SB:1;             /*!< bit:      8  Security Bit Status                */
    uint16_t :7;               /*!< bit:  9..15  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint16_t reg;                /*!< Type      used for register access              */
} NVMCTRL_STATUS_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define NVMCTRL_STATUS_OFFSET       0x18         /**< \brief (NVMCTRL_STATUS offset) Status */
#define NVMCTRL_STATUS_RESETVALUE   0x0000ul     /**< \brief (NVMCTRL_STATUS reset_value) Status */

#define NVMCTRL_STATUS_PRM_Pos      0            /**< \brief (NVMCTRL_STATUS) Power Reduction Mode */
#define NVMCTRL_STATUS_PRM          (0x1ul << NVMCTRL_STATUS_PRM_Pos)
#define NVMCTRL_STATUS_LOAD_Pos     1            /**< \brief (NVMCTRL_STATUS) NVM Page Buffer Active Loading */
#define NVMCTRL_STATUS_LOAD         (0x1ul << NVMCTRL_STATUS_LOAD_Pos)
#define NVMCTRL_STATUS_PROGE_Pos    2            /**< \brief (NVMCTRL_STATUS) Programming Error Status */
#define NVMCTRL_STATUS_PROGE        (0x1ul << NVMCTRL_STATUS_PROGE_Pos)
#define NVMCTRL_STATUS_LOCKE_Pos    3            /**< \brief (NVMCTRL_STATUS) Lock Error Status */
#define NVMCTRL_STATUS_LOCKE        (0x1ul << NVMCTRL_STATUS_LOCKE_Pos)
#define NVMCTRL_STATUS_NVME_Pos     4            /**< \brief (NVMCTRL_STATUS) NVM Error */
#define NVMCTRL_STATUS_NVME         (0x1ul << NVMCTRL_STATUS_NVME_Pos)
#define NVMCTRL_STATUS_SB_Pos       8            /**< \brief (NVMCTRL_STATUS) Security Bit Status */
#define NVMCTRL_STATUS_SB           (0x1ul << NVMCTRL_STATUS_SB_Pos)
#define NVMCTRL_STATUS_MASK         0x011Ful     /**< \brief (NVMCTRL_STATUS) MASK Register */

/* -------- NVMCTRL_ADDR : (NVMCTRL Offset: 0x1C) (R/W 32) Address -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ADDR:22;          /*!< bit:  0..21  NVM Address                        */
    uint32_t :10;              /*!< bit: 22..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} NVMCTRL_ADDR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define NVMCTRL_ADDR_OFFSET         0x1C         /**< \brief (NVMCTRL_ADDR offset) Address */
#define NVMCTRL_ADDR_RESETVALUE     0x00000000ul /**< \brief (NVMCTRL_ADDR reset_value) Address */

#define NVMCTRL_ADDR_ADDR_Pos       0            /**< \brief (NVMCTRL_ADDR) NVM Address */
#define NVMCTRL_ADDR_ADDR_Msk       (0x3FFFFFul << NVMCTRL_ADDR_ADDR_Pos)
#define NVMCTRL_ADDR_ADDR(value)    (NVMCTRL_ADDR_ADDR_Msk & ((value) << NVMCTRL_ADDR_ADDR_Pos))
#define NVMCTRL_ADDR_MASK           0x003FFFFFul /**< \brief (NVMCTRL_ADDR) MASK Register */

/* -------- NVMCTRL_LOCK : (NVMCTRL Offset: 0x20) (R/W 16) Lock Section -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint16_t LOCK:16;          /*!< bit:  0..15  Region Lock Bits                   */
  } bit;                       /*!< Structure used for bit  access                  */
  uint16_t reg;                /*!< Type      used for register access              */
} NVMCTRL_LOCK_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define NVMCTRL_LOCK_OFFSET         0x20         /**< \brief (NVMCTRL_LOCK offset) Lock Section */

#define NVMCTRL_LOCK_LOCK_Pos       0            /**< \brief (NVMCTRL_LOCK) Region Lock Bits */
#define NVMCTRL_LOCK_LOCK_Msk       (0xFFFFul << NVMCTRL_LOCK_LOCK_Pos)
#define NVMCTRL_LOCK_LOCK(value)    (NVMCTRL_LOCK_LOCK_Msk & ((value) << NVMCTRL_LOCK_LOCK_Pos))
#define NVMCTRL_LOCK_MASK           0xFFFFul     /**< \brief (NVMCTRL_LOCK) MASK Register */

/** \brief NVMCTRL APB hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO NVMCTRL_CTRLA_Type        CTRLA;       /**< \brief Offset: 0x00 (R/W 16) Control A */
       RoReg8                    Reserved1[0x2];
  __IO NVMCTRL_CTRLB_Type        CTRLB;       /**< \brief Offset: 0x04 (R/W 32) Control B */
  __IO NVMCTRL_PARAM_Type        PARAM;       /**< \brief Offset: 0x08 (R/W 32) NVM Parameter */
  __IO NVMCTRL_INTENCLR_Type     INTENCLR;    /**< \brief Offset: 0x0C (R/W  8) Interrupt Enable Clear */
       RoReg8                    Reserved2[0x3];
  __IO NVMCTRL_INTENSET_Type     INTENSET;    /**< \brief Offset: 0x10 (R/W  8) Interrupt Enable Set */
       RoReg8                    Reserved3[0x3];
  __IO NVMCTRL_INTFLAG_Type      INTFLAG;     /**< \brief Offset: 0x14 (R/W  8) Interrupt Flag Status and Clear */
       RoReg8                    Reserved4[0x3];
  __IO NVMCTRL_STATUS_Type       STATUS;      /**< \brief Offset: 0x18 (R/W 16) Status */
       RoReg8                    Reserved5[0x2];
  __IO NVMCTRL_ADDR_Type         ADDR;        /**< \brief Offset: 0x1C (R/W 32) Address */
  __IO NVMCTRL_LOCK_Type         LOCK;        /**< \brief Offset: 0x20 (R/W 16) Lock Section */
} Nvmctrl;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#ifdef __GNUC__
 #define SECTION_NVMCTRL_CAL          __attribute__ ((section(".flash")))
#elif defined(__ICCARM__)
 #define SECTION_NVMCTRL_CAL          @".flash"
#endif

#ifdef __GNUC__
 #define SECTION_NVMCTRL_LOCKBIT      __attribute__ ((section(".flash")))
#elif defined(__ICCARM__)
 #define SECTION_NVMCTRL_LOCKBIT      @".flash"
#endif

#ifdef __GNUC__
 #define SECTION_NVMCTRL_OTP1         __attribute__ ((section(".flash")))
#elif defined(__ICCARM__)
 #define SECTION_NVMCTRL_OTP1         @".flash"
#endif

#ifdef __GNUC__
 #define SECTION_NVMCTRL_OTP2         __attribute__ ((section(".flash")))
#elif defined(__ICCARM__)
 #define SECTION_NVMCTRL_OTP2         @".flash"
#endif

#ifdef __GNUC__
 #define SECTION_NVMCTRL_OTP3         __attribute__ ((section(".flash")))
#elif defined(__ICCARM__)
 #define SECTION_NVMCTRL_OTP3         @".flash"
#endif

#ifdef __GNUC__
 #define SECTION_NVMCTRL_OTP4         __attribute__ ((section(".flash")))
#elif defined(__ICCARM__)
 #define SECTION_NVMCTRL_OTP4         @".flash"
#endif

#ifdef __GNUC__
 #define SECTION_NVMCTRL_OTP5         __attribute__ ((section(".flash")))
#elif defined(__ICCARM__)
 #define SECTION_NVMCTRL_OTP5         @".flash"
#endif

#ifdef __GNUC__
 #define SECTION_NVMCTRL_TEMP_LOG     __attribute__ ((section(".flash")))
#elif defined(__ICCARM__)
 #define SECTION_NVMCTRL_TEMP_LOG     @".flash"
#endif

#ifdef __GNUC__
 #define SECTION_NVMCTRL_USER         __attribute__ ((section(".flash")))
#elif defined(__ICCARM__)
 #define SECTION_NVMCTRL_USER         @".flash"
#endif

/*@}*/

/* ************************************************************************** */
/**  SOFTWARE PERIPHERAL API DEFINITION FOR NON-VOLATILE FUSES */
/* ************************************************************************** */
/** \addtogroup fuses_api Peripheral Software API */
/*@{*/


#define ADC_FUSES_BIASCOMP_ADDR     NVMCTRL_OTP5
#define ADC_FUSES_BIASCOMP_Pos      3            /**< \brief (NVMCTRL_OTP5) ADC Comparator Scaling */
#define ADC_FUSES_BIASCOMP_Msk      (0x7ul << ADC_FUSES_BIASCOMP_Pos)
#define ADC_FUSES_BIASCOMP(value)   (ADC_FUSES_BIASCOMP_Msk & ((value) << ADC_FUSES_BIASCOMP_Pos))

#define ADC_FUSES_BIASREFBUF_ADDR   NVMCTRL_OTP5
#define ADC_FUSES_BIASREFBUF_Pos    0            /**< \brief (NVMCTRL_OTP5) ADC Bias Reference Buffer Scaling */
#define ADC_FUSES_BIASREFBUF_Msk    (0x7ul << ADC_FUSES_BIASREFBUF_Pos)
#define ADC_FUSES_BIASREFBUF(value) (ADC_FUSES_BIASREFBUF_Msk & ((value) << ADC_FUSES_BIASREFBUF_Pos))

#define FUSES_BOD12_DIS_ADDR        NVMCTRL_USER
#define FUSES_BOD12_DIS_Pos         23           /**< \brief (NVMCTRL_USER) BOD12 Disable */
#define FUSES_BOD12_DIS_Msk         (0x1ul << FUSES_BOD12_DIS_Pos)

#define FUSES_BOD12_HYST_ADDR       (NVMCTRL_USER + 4)
#define FUSES_BOD12_HYST_Pos        10           /**< \brief (NVMCTRL_USER) BOD12 Hysteresis */
#define FUSES_BOD12_HYST_Msk        (0x1ul << FUSES_BOD12_HYST_Pos)

#define FUSES_BOD33USERLEVEL_ADDR   NVMCTRL_USER
#define FUSES_BOD33USERLEVEL_Pos    8            /**< \brief (NVMCTRL_USER) BOD33 User Level */
#define FUSES_BOD33USERLEVEL_Msk    (0x3Ful << FUSES_BOD33USERLEVEL_Pos)
#define FUSES_BOD33USERLEVEL(value) (FUSES_BOD33USERLEVEL_Msk & ((value) << FUSES_BOD33USERLEVEL_Pos))

#define FUSES_BOD33_ACTION_ADDR     NVMCTRL_USER
#define FUSES_BOD33_ACTION_Pos      15           /**< \brief (NVMCTRL_USER) BOD33 Action */
#define FUSES_BOD33_ACTION_Msk      (0x3ul << FUSES_BOD33_ACTION_Pos)
#define FUSES_BOD33_ACTION(value)   (FUSES_BOD33_ACTION_Msk & ((value) << FUSES_BOD33_ACTION_Pos))

#define FUSES_BOD33_DIS_ADDR        NVMCTRL_USER
#define FUSES_BOD33_DIS_Pos         14           /**< \brief (NVMCTRL_USER) BOD33 Disable */
#define FUSES_BOD33_DIS_Msk         (0x1ul << FUSES_BOD33_DIS_Pos)

#define FUSES_BOD33_HYST_ADDR       (NVMCTRL_USER + 4)
#define FUSES_BOD33_HYST_Pos        9            /**< \brief (NVMCTRL_USER) BOD33 Hysteresis */
#define FUSES_BOD33_HYST_Msk        (0x1ul << FUSES_BOD33_HYST_Pos)

#define FUSES_HOT_ADC_VAL_ADDR      (NVMCTRL_TEMP_LOG + 4)
#define FUSES_HOT_ADC_VAL_Pos       20           /**< \brief (NVMCTRL_TEMP_LOG) 12-bit ADC conversion at hot temperature */
#define FUSES_HOT_ADC_VAL_Msk       (0xFFFul << FUSES_HOT_ADC_VAL_Pos)
#define FUSES_HOT_ADC_VAL(value)    (FUSES_HOT_ADC_VAL_Msk & ((value) << FUSES_HOT_ADC_VAL_Pos))

#define FUSES_HOT_INT1V_VAL_ADDR    (NVMCTRL_TEMP_LOG + 4)
#define FUSES_HOT_INT1V_VAL_Pos     0            /**< \brief (NVMCTRL_TEMP_LOG) 2's complement of the internal 1V reference drift at hot temperature (versus a 1.0 centered value) */
#define FUSES_HOT_INT1V_VAL_Msk     (0xFFul << FUSES_HOT_INT1V_VAL_Pos)
#define FUSES_HOT_INT1V_VAL(value)  (FUSES_HOT_INT1V_VAL_Msk & ((value) << FUSES_HOT_INT1V_VAL_Pos))

#define FUSES_HOT_TEMP_VAL_DEC_ADDR NVMCTRL_TEMP_LOG
#define FUSES_HOT_TEMP_VAL_DEC_Pos  20           /**< \brief (NVMCTRL_TEMP_LOG) Decimal part of hot temperature */
#define FUSES_HOT_TEMP_VAL_DEC_Msk  (0xFul << FUSES_HOT_TEMP_VAL_DEC_Pos)
#define FUSES_HOT_TEMP_VAL_DEC(value) (FUSES_HOT_TEMP_VAL_DEC_Msk & ((value) << FUSES_HOT_TEMP_VAL_DEC_Pos))

#define FUSES_HOT_TEMP_VAL_INT_ADDR NVMCTRL_TEMP_LOG
#define FUSES_HOT_TEMP_VAL_INT_Pos  12           /**< \brief (NVMCTRL_TEMP_LOG) Integer part of hot temperature in oC */
#define FUSES_HOT_TEMP_VAL_INT_Msk  (0xFFul << FUSES_HOT_TEMP_VAL_INT_Pos)
#define FUSES_HOT_TEMP_VAL_INT(value) (FUSES_HOT_TEMP_VAL_INT_Msk & ((value) << FUSES_HOT_TEMP_VAL_INT_Pos))

#define FUSES_ROOM_ADC_VAL_ADDR     (NVMCTRL_TEMP_LOG + 4)
#define FUSES_ROOM_ADC_VAL_Pos      8            /**< \brief (NVMCTRL_TEMP_LOG) 12-bit ADC conversion at room temperature */
#define FUSES_ROOM_ADC_VAL_Msk      (0xFFFul << FUSES_ROOM_ADC_VAL_Pos)
#define FUSES_ROOM_ADC_VAL(value)   (FUSES_ROOM_ADC_VAL_Msk & ((value) << FUSES_ROOM_ADC_VAL_Pos))

#define FUSES_ROOM_INT1V_VAL_ADDR   NVMCTRL_TEMP_LOG
#define FUSES_ROOM_INT1V_VAL_Pos    24           /**< \brief (NVMCTRL_TEMP_LOG) 2's complement of the internal 1V reference drift at room temperature (versus a 1.0 centered value) */
#define FUSES_ROOM_INT1V_VAL_Msk    (0xFFul << FUSES_ROOM_INT1V_VAL_Pos)
#define FUSES_ROOM_INT1V_VAL(value) (FUSES_ROOM_INT1V_VAL_Msk & ((value) << FUSES_ROOM_INT1V_VAL_Pos))

#define FUSES_ROOM_TEMP_VAL_DEC_ADDR NVMCTRL_TEMP_LOG
#define FUSES_ROOM_TEMP_VAL_DEC_Pos 8            /**< \brief (NVMCTRL_TEMP_LOG) Decimal part of room temperature */
#define FUSES_ROOM_TEMP_VAL_DEC_Msk (0xFul << FUSES_ROOM_TEMP_VAL_DEC_Pos)
#define FUSES_ROOM_TEMP_VAL_DEC(value) (FUSES_ROOM_TEMP_VAL_DEC_Msk & ((value) << FUSES_ROOM_TEMP_VAL_DEC_Pos))

#define FUSES_ROOM_TEMP_VAL_INT_ADDR NVMCTRL_TEMP_LOG
#define FUSES_ROOM_TEMP_VAL_INT_Pos 0            /**< \brief (NVMCTRL_TEMP_LOG) Integer part of room temperature in oC */
#define FUSES_ROOM_TEMP_VAL_INT_Msk (0xFFul << FUSES_ROOM_TEMP_VAL_INT_Pos)
#define FUSES_ROOM_TEMP_VAL_INT(value) (FUSES_ROOM_TEMP_VAL_INT_Msk & ((value) << FUSES_ROOM_TEMP_VAL_INT_Pos))

#define NVMCTRL_FUSES_BOOTPROT_ADDR NVMCTRL_USER
#define NVMCTRL_FUSES_BOOTPROT_Pos  0            /**< \brief (NVMCTRL_USER) Bootloader Size */
#define NVMCTRL_FUSES_BOOTPROT_Msk  (0x7ul << NVMCTRL_FUSES_BOOTPROT_Pos)
#define NVMCTRL_FUSES_BOOTPROT(value) (NVMCTRL_FUSES_BOOTPROT_Msk & ((value) << NVMCTRL_FUSES_BOOTPROT_Pos))

#define NVMCTRL_FUSES_EEPROM_SIZE_ADDR NVMCTRL_USER
#define NVMCTRL_FUSES_EEPROM_SIZE_Pos 4            /**< \brief (NVMCTRL_USER) EEPROM Size */
#define NVMCTRL_FUSES_EEPROM_SIZE_Msk (0x7ul << NVMCTRL_FUSES_EEPROM_SIZE_Pos)
#define NVMCTRL_FUSES_EEPROM_SIZE(value) (NVMCTRL_FUSES_EEPROM_SIZE_Msk & ((value) << NVMCTRL_FUSES_EEPROM_SIZE_Pos))

#define NVMCTRL_FUSES_REGION_LOCKS_ADDR (NVMCTRL_USER + 4)
#define NVMCTRL_FUSES_REGION_LOCKS_Pos 16           /**< \brief (NVMCTRL_USER) NVM Region Locks */
#define NVMCTRL_FUSES_REGION_LOCKS_Msk (0xFFFFul << NVMCTRL_FUSES_REGION_LOCKS_Pos)
#define NVMCTRL_FUSES_REGION_LOCKS(value) (NVMCTRL_FUSES_REGION_LOCKS_Msk & ((value) << NVMCTRL_FUSES_REGION_LOCKS_Pos))

#define USB_FUSES_TRANSN_ADDR       NVMCTRL_OTP5
#define USB_FUSES_TRANSN_Pos        13           /**< \brief (NVMCTRL_OTP5) USB pad Transn calibration */
#define USB_FUSES_TRANSN_Msk        (0x1Ful << USB_FUSES_TRANSN_Pos)
#define USB_FUSES_TRANSN(value)     (USB_FUSES_TRANSN_Msk & ((value) << USB_FUSES_TRANSN_Pos))

#define USB_FUSES_TRANSP_ADDR       NVMCTRL_OTP5
#define USB_FUSES_TRANSP_Pos        18           /**< \brief (NVMCTRL_OTP5) USB pad Transp calibration */
#define USB_FUSES_TRANSP_Msk        (0x1Ful << USB_FUSES_TRANSP_Pos)
#define USB_FUSES_TRANSP(value)     (USB_FUSES_TRANSP_Msk & ((value) << USB_FUSES_TRANSP_Pos))

#define USB_FUSES_TRIM_ADDR         NVMCTRL_OTP5
#define USB_FUSES_TRIM_Pos          23           /**< \brief (NVMCTRL_OTP5) USB pad Trim calibration */
#define USB_FUSES_TRIM_Msk          (0x7ul << USB_FUSES_TRIM_Pos)
#define USB_FUSES_TRIM(value)       (USB_FUSES_TRIM_Msk & ((value) << USB_FUSES_TRIM_Pos))

#define WDT_FUSES_ALWAYSON_ADDR     NVMCTRL_USER
#define WDT_FUSES_ALWAYSON_Pos      27           /**< \brief (NVMCTRL_USER) WDT Always On */
#define WDT_FUSES_ALWAYSON_Msk      (0x1ul << WDT_FUSES_ALWAYSON_Pos)

#define WDT_FUSES_ENABLE_ADDR       NVMCTRL_USER
#define WDT_FUSES_ENABLE_Pos        26           /**< \brief (NVMCTRL_USER) WDT Enable */
#define WDT_FUSES_ENABLE_Msk        (0x1ul << WDT_FUSES_ENABLE_Pos)

#define WDT_FUSES_EWOFFSET_ADDR     (NVMCTRL_USER + 4)
#define WDT_FUSES_EWOFFSET_Pos      4            /**< \brief (NVMCTRL_USER) WDT Early Warning Offset */
#define WDT_FUSES_EWOFFSET_Msk      (0xFul << WDT_FUSES_EWOFFSET_Pos)
#define WDT_FUSES_EWOFFSET(value)   (WDT_FUSES_EWOFFSET_Msk & ((value) << WDT_FUSES_EWOFFSET_Pos))

#define WDT_FUSES_PER_ADDR          NVMCTRL_USER
#define WDT_FUSES_PER_Pos           28           /**< \brief (NVMCTRL_USER) WDT Period */
#define WDT_FUSES_PER_Msk           (0xFul << WDT_FUSES_PER_Pos)
#define WDT_FUSES_PER(value)        (WDT_FUSES_PER_Msk & ((value) << WDT_FUSES_PER_Pos))

#define WDT_FUSES_WEN_ADDR          (NVMCTRL_USER + 4)
#define WDT_FUSES_WEN_Pos           8            /**< \brief (NVMCTRL_USER) WDT Window Mode Enable */
#define WDT_FUSES_WEN_Msk           (0x1ul << WDT_FUSES_WEN_Pos)

#define WDT_FUSES_WINDOW_ADDR       (NVMCTRL_USER + 4)
#define WDT_FUSES_WINDOW_Pos        0            /**< \brief (NVMCTRL_USER) WDT Window */
#define WDT_FUSES_WINDOW_Msk        (0xFul << WDT_FUSES_WINDOW_Pos)
#define WDT_FUSES_WINDOW(value)     (WDT_FUSES_WINDOW_Msk & ((value) << WDT_FUSES_WINDOW_Pos))

/*@}*/

#endif /* _SAMR30_NVMCTRL_COMPONENT_ */

#endif // _ATSAMR30_H_
