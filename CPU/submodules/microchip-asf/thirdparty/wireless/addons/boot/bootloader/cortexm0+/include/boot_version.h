/**
* \file  boot_version.h
*
* \brief bootloader information structure and parameters.
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
#ifndef _BOOT_VERSION_H
#define _BOOT_VERSION_H

#if defined(EXT_MEMORY)
  #include <extMemReader.h>
#endif

/******************************************************************************
                   Prototypes section
******************************************************************************/
#define TWO_POWER_EXPONENT_4(x)  ((x)&0x8?3:((x)&0x4?2:((x)&0x2?1:0)))

#define TWO_POWER_EXPONENT_8(x)  (((x)>>4)&0xF?TWO_POWER_EXPONENT_4(((x)>>4)&0xF)+4:((x)&0xF?TWO_POWER_EXPONENT_4((x)&0xF):0)) 

#define TWO_POWER_EXPONENT_16(x)   (((x)>>8)&0xFF?TWO_POWER_EXPONENT_8(((x)>>8)&0xFF)+8:((x)&0xFF?TWO_POWER_EXPONENT_8((x)&0xFF):0))   

#define FLASH_SIZE_IN_KB  ((EEPROM_END_ADDRESS + 1) >> 10)

#ifdef EXT_MEMORY
  #define EXT_MEM_SIZE_IN_KB  (IMAGE_SIZE >> 10)
#else
  #define EXT_MEM_SIZE_IN_KB  0
#endif


// Feature-mask definitions
#define FEATURE_MULTI_APPS	 	               1


#if USE_IMAGE_SECURITY
  #define FEATURE_IMG_SECURITY 	             1
#else
  #define FEATURE_IMG_SECURITY 	             0
#endif

#if USE_RF
  #define FEATURE_RF_INTERFACE       		     1
#else
  #define FEATURE_RF_INTERFACE               0
#endif

#if USE_USART0 || USE_USARTD0
  #define FEATURE_USART_0_INTERFACE 		     1
#else
  #define FEATURE_USART_0_INTERFACE 		     0
#endif

#if USE_USART1 || USE_USARTF0
  #define FEATURE_USART_1_INTERFACE 		     1
#else
  #define FEATURE_USART_1_INTERFACE 		     0
#endif

#if USE_SPIE
  #define FEATURE_SPI_INTERFACE		           1
#else
  #define FEATURE_SPI_INTERFACE	             0
#endif

#if (USE_TWIS_C == 1) || (USE_TWIS_D == 1) || (USE_TWIS_E == 1) || (USE_TWIS_F == 1)
  #define FEATURE_TWI_INTERFACE		           1
#else
  #define FEATURE_TWI_INTERFACE		           0
#endif

#if USE_USB_DFU || USE_USB_FIFO
  #define FEATURE_USB_INTERFACE		           1
#else
  #define FEATURE_USB_INTERFACE		           0
#endif

/* This structures needs to be alligned*/
typedef struct {
  uint16_t  multiApps       :1;   //  BIT_0
  uint16_t  imgSecurity     :1;   //  BIT_1
  uint16_t  rfInterface     :1;   //  BIT_2
  uint16_t  usart0Interface :1;   //  BIT_3
  uint16_t  usart1Interface :1;   //  BIT_4
  uint16_t  spiInterface    :1;   //  BIT_5
  uint16_t  twiInterface    :1;   //  BIT_6
  uint16_t  usbInterface    :1;   //  BIT_7
  uint16_t  reserved        :8;   //  BIT_8 to BIT_15
} featureBits_t;

typedef union {
  uint16_t        mask;
  featureBits_t   bits;
} featureMask_t;
  
typedef struct {
  uint8_t         crc;
  uint16_t        majorRev;     // Major revision of the bootloader release
  uint16_t        minorRev;     // Minor revision of the bootloader release
  uint16_t        reservedRev;  // Reserved for extending revision  
  featureMask_t   supported_features;  // Bit-mask indicating available features
  uint32_t        start_address;  // Start address of the bootloader in internal flash
  uint8_t         mcu_flash_size; // Flash size of the internal MCU (2^value KB, if non-zero) 
  uint8_t         ext_flash_size; // Flash size of the external mem (2^value KB, if non-zero)
  uint8_t         bprot_size;     // Size of the BOOT_PROT region (2^value KB, if non-zero)
  uint8_t         reserved;       // Future extension
} boot_info_t;
#define BOOT_INFO_START_ADDRESS 0x100
extern  boot_info_t bInfo;

#endif // _BOOT_VERSION_H

// eof boot_version.h
