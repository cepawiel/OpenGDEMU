/**
* \file  configuration.h
*
* \brief Bootloader configuration header
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
#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#define NO 0x01
#define YES 0x02

#define AT25F2048  0x01
#define AT45DB041  0x02
#define AT25DF041A 0x03
#define AT25DF041B 0x04
#define W25X20CV 0x05
#define M25P40VMN6PB 0x06
#define MX25L2006E 0x07
#define MX25V4006E 0x08


#define CRYSTAL_12MHz     0x01
#define CRYSTAL_16MHz     0x02
#define CRYSTAL_18d432MHz 0x03

// Use external memory
#define EXTERNAL_MEMORY AT25DF041A
//#define EXTERNAL_MEMORY W25X20CV
//#define EXTERNAL_MEMORY AT25DF041B
//#define EXTERNAL_MEMORY M25P40VMN6PB
//#define EXTERNAL_MEMORY MX25L2006E
//#define EXTERNAL_MEMORY MX25V4006E
//#define EXTERNAL_MEMORY AT25F2048
//#define EXTERNAL_MEMORY AT45DB041

// Rcb_bb RS-232 controller
#define RCB_BB_RS232_CONTROLLER YES
//#define RCB_BB_RS232_CONTROLLER NO

// Key remote RS-232 controller
#define KEY_REMOTE_RS232_CONTROLLER NO
//#define KEY_REMOTE_RS232_CONTROLLER YES

// MCU crystal frequency for Sam3s
#define CRYSTAL_FREQUENCY CRYSTAL_12MHz
//#define CRYSTAL_FREQUENCY CRYSTAL_16MHz
//#define CRYSTAL_FREQUENCY CRYSTAL_18d432MHz

//-----------------------------------------------
//ATXMEGA256D3
//-----------------------------------------------
#ifdef ATXMEGA256D3
  // Use USARTD0
  #define USE_USARTD0 1
  //#define USE_USARTD0 0
  
  // Rcb_bb RS-232 controller
  #undef RCB_BB_RS232_CONTROLLER
  #define RCB_BB_RS232_CONTROLLER NO
  
  // Key remote RS-232 controller
  #undef KEY_REMOTE_RS232_CONTROLLER
  #define KEY_REMOTE_RS232_CONTROLLER NO
  
  // Use TWIC. USART shall not be used at the same time
  #define USE_TWIS_C 0
  //#define USE_TWIS_C 1
#endif

//-----------------------------------------------
//ATSAMR30E18A
//-----------------------------------------------
#ifdef ATSAMR30E18A
  // Use USART0
  #define USE_USART0 1
  
  // Use SPI
  #define USE_SPI 1
  
  // Use external memory
  #undef EXTERNAL_MEMORY
  #define EXTERNAL_MEMORY AT25DF041B
  
  // Supported boards
  #define BOARD_SAMR30_MODULE_XPRO
#endif

//-----------------------------------------------
//AT91SAM3S4C
//-----------------------------------------------
#ifdef AT91SAM3S4C
  // Use USB DFU class
  #define USE_USB_DFU 1
  //#define USE_USB_DFU 0
  
  // Key remote RS-232 controller
  #undef KEY_REMOTE_RS232_CONTROLLER
  #define KEY_REMOTE_RS232_CONTROLLER NO
  
  // Rcb_bb RS-232 controller
  #undef RCB_BB_RS232_CONTROLLER
  #define RCB_BB_RS232_CONTROLLER NO
#endif

//-----------------------------------------------
//ATXMEGA256A3
//-----------------------------------------------
#ifdef ATXMEGA256A3
  // Use USARTF0
  #define USE_USARTF0 1
  //#define USE_USARTF0 0
  
  // Use USE_SPIE
  #define USE_SPIE 0
  //#define USE_SPIE 1
  
  // Use TWIE. USART shall not be used at the same time
  #define USE_TWIS_E 0
  //#define USE_TWIS_E 1
  
  // Use TWIC. USART shall not be used at the same time
  #define USE_TWIS_C 0
  //#define USE_TWIS_C 1
  
  // Use USARTD0
  #define USE_USARTD0 1
  //#define USE_USARTD0 0
  
  // Key remote RS-232 controller
  #undef KEY_REMOTE_RS232_CONTROLLER
  #define KEY_REMOTE_RS232_CONTROLLER NO
  
  // Rcb_bb RS-232 controller
  #undef RCB_BB_RS232_CONTROLLER
  #define RCB_BB_RS232_CONTROLLER NO
#endif //

//-----------------------------------------------
//ATSAMR21E18A
//-----------------------------------------------
#ifdef ATSAMR21E18A
  // Use USART1
  #define USE_USART1 1
  //#define USE_USART1 0
  
  // Use RF
  #define USE_RF 1
  
  // Enable BL security
  #define USE_IMAGE_SECURITY 0
  //#define USE_IMAGE_SECURITY 1
  
  // Key remote RS-232 controller
  #undef KEY_REMOTE_RS232_CONTROLLER
  #define KEY_REMOTE_RS232_CONTROLLER NO
  
  // Use SPI
  #define USE_SPI 1
  
  // Use USART0
  #define USE_USART0 0
  //#define USE_USART0 1
  
  // Rcb_bb RS-232 controller
  #undef RCB_BB_RS232_CONTROLLER
  #define RCB_BB_RS232_CONTROLLER NO
  
  // Use external memory
  #undef EXTERNAL_MEMORY
  #define EXTERNAL_MEMORY MX25L2006E
  
  // Supported boards
  #define BOARD_SAMR21B18_MZ210PA_MODULE
#endif //

//-----------------------------------------------
//ATMEGA1281
//-----------------------------------------------
#ifdef ATMEGA1281
  // Use USB_FIFO
  #define USE_USB_FIFO 0
  //#define USE_USB_FIFO 1
  
  // Use TWID. USART shall not be used at the same time
  #define USE_TWIS_D 0
  //#define USE_TWIS_D 1
  
  // Use USART0
  #define USE_USART0 0
  //#define USE_USART0 1
  
  // Use USART1
  #define USE_USART1 1
  //#define USE_USART1 0
#endif

//-----------------------------------------------
//ATXMEGA128A1
//-----------------------------------------------
#ifdef ATXMEGA128A1
  // Use TWIF. USART shall not be used at the same time
  #define USE_TWIS_F 0
  //#define USE_TWIS_F 1
  
  // Use TWIC. USART shall not be used at the same time
  #define USE_TWIS_C 0
  //#define USE_TWIS_C 1
  
  // Use TWID. USART shall not be used at the same time
  #define USE_TWIS_D 0
  //#define USE_TWIS_D 1
  
  // Use TWIE. USART shall not be used at the same time
  #define USE_TWIS_E 0
  //#define USE_TWIS_E 1
  
  // Key remote RS-232 controller
  #undef KEY_REMOTE_RS232_CONTROLLER
  #define KEY_REMOTE_RS232_CONTROLLER NO
  
  // Use USARTF0
  #define USE_USARTF0 1
  //#define USE_USARTF0 0
  
  // Use USARTD0
  #define USE_USARTD0 1
  //#define USE_USARTD0 0
  
  // Rcb_bb RS-232 controller
  #undef RCB_BB_RS232_CONTROLLER
  #define RCB_BB_RS232_CONTROLLER NO
  
  // Use USE_SPIE
  #define USE_SPIE 0
  //#define USE_SPIE 1
#endif //

//-----------------------------------------------
//ATSAMR21E19A
//-----------------------------------------------
#ifdef ATSAMR21E19A
  // Key remote RS-232 controller
  #undef KEY_REMOTE_RS232_CONTROLLER
  #define KEY_REMOTE_RS232_CONTROLLER NO
  
  // Use SPI
  #define USE_SPI 1
  
  // Use RF
  #define USE_RF 1
  
  // Use USART0
  #define USE_USART0 0
  //#define USE_USART0 1
  
  // Use USART1
  #define USE_USART1 1
  //#define USE_USART1 0
  
  // Rcb_bb RS-232 controller
  #undef RCB_BB_RS232_CONTROLLER
  #define RCB_BB_RS232_CONTROLLER NO
  
  // Use external memory
  #undef EXTERNAL_MEMORY
  #define EXTERNAL_MEMORY MX25V4006E
  
  // Enable BL security
  #define USE_IMAGE_SECURITY 0
  //#define USE_IMAGE_SECURITY 1
  
  // Supported boards
  #define BOARD_FAKE
#endif //

//-----------------------------------------------
//ATSAMR21G18A
//-----------------------------------------------
#ifdef ATSAMR21G18A
  // RF_SERVER_MODE
  
  //#define RF_SERVER_MODE 1
  
  // Key remote RS-232 controller
  #undef KEY_REMOTE_RS232_CONTROLLER
  #define KEY_REMOTE_RS232_CONTROLLER NO
  
  // Use SPI
  #define USE_SPI 1
  
  // Use RF
  #define USE_RF 1
  
  // Use USART0
  #define USE_USART0 0
  //#define USE_USART0 1
  
  // Use USART1
  #define USE_USART1 1
  //#define USE_USART1 0
  
  // Rcb_bb RS-232 controller
  #undef RCB_BB_RS232_CONTROLLER
  #define RCB_BB_RS232_CONTROLLER NO
  
   
  // Supported boards
  #define BOARD_SAMR21G18_MR210UA_MODULE
  //#define BOARD_SAMR21_ZLLEK
  //#define BOARD_SAMR21_XPRO
#endif //

//-----------------------------------------------
//ATSAMR30G18A
//-----------------------------------------------
#ifdef ATSAMR30G18A
  // Use SPI
  #define USE_SPI 1
  
  // Use USART0
  #define USE_USART0 1
  
  // Supported boards
  #define BOARD_SAMR30_XPRO
#endif

//-----------------------------------------------
//ATMEGA256RFR2
//-----------------------------------------------
#ifdef ATMEGA256RFR2
  // Enable SPM interrupt
  #define RFR2_SPM 1
  //#define RFR2_SPM 0
  
  // Use TWID. USART shall not be used at the same time
  #define USE_TWIS_D 0
  //#define USE_TWIS_D 1
  
  // Use SPI
  #define USE_SPI 0
  //#define USE_SPI 1
  
  // Use USART0
  #define USE_USART0 0
  //#define USE_USART0 1
  
  // Use USART1
  #define USE_USART1 1
  //#define USE_USART1 0
  
  // Use USB_FIFO
  #define USE_USB_FIFO 0
  //#define USE_USB_FIFO 1
  
  // Enable BL security
  #define USE_IMAGE_SECURITY 0
  //#define USE_IMAGE_SECURITY 1
#endif //

//-----------------------------------------------
//ATMEGA2561
//-----------------------------------------------
#ifdef ATMEGA2561
  // Use TWID. USART shall not be used at the same time
  #define USE_TWIS_D 0
  //#define USE_TWIS_D 1
  
  // Use USART0
  #define USE_USART0 0
  //#define USE_USART0 1
  
  // Use USART1
  #define USE_USART1 1
  //#define USE_USART1 0
#endif

//-----------------------------------------------
//ATMEGA2564RFR2
//-----------------------------------------------
#ifdef ATMEGA2564RFR2
  // Use TWID. USART shall not be used at the same time
  #define USE_TWIS_D 0
  //#define USE_TWIS_D 1
  
  // Use SPI
  #define USE_SPI 0
  //#define USE_SPI 1
  
  // Use USART0
  #define USE_USART0 0
  //#define USE_USART0 1
  
  // Use USART1
  #define USE_USART1 1
  //#define USE_USART1 0
  
  // Use USB_FIFO
  #define USE_USB_FIFO 0
  //#define USE_USB_FIFO 1
  
  // Enable SPM interrupt
  #define RFR2_SPM 1
  //#define RFR2_SPM 0
  
  // Enable BL security
  #define USE_IMAGE_SECURITY 0
  //#define USE_IMAGE_SECURITY 1
#endif //

//-----------------------------------------------
//ATMEGA128RFA1
//-----------------------------------------------
#ifdef ATMEGA128RFA1
  // Use TWID. USART shall not be used at the same time
  #define USE_TWIS_D 0
  //#define USE_TWIS_D 1
  
  // Use SPI
  #define USE_SPI 0
  //#define USE_SPI 1
  
  // Use USART0
  #define USE_USART0 0
  //#define USE_USART0 1
  
  // Use USART1
  #define USE_USART1 1
  //#define USE_USART1 0
  
  // Use USB_FIFO
  #define USE_USB_FIFO 0
  //#define USE_USB_FIFO 1
#endif



//
// Defining SAM0 Macro for Supporting new bootloader features
#if defined(ATSAMR21G18A) || defined(ATSAMR21E18A) || defined(ATSAMR21E19A) || defined(ATSAMR30G18A) || defined(ATSAMR30E18A)
#define SAM0
#endif



#if RF_SERVER_MODE
#define USE_RF            1
#undef EXTERNAL_MEMORY
#endif

// Handle no external memory case
#ifndef EXTERNAL_MEMORY
#undef EXT_MEMORY
#endif

#endif // _CONFIGURATION_H_
