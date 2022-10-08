/**
* \file  app_info.h
*
* \brief Application info table structure and parameters.
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
#ifndef _APP_INFO_H
#define _APP_INFO_H

#include "boot_version.h"

//
// typedefs and macros
//
COMPILER_PACK_SET(1)
typedef struct 
{
  uint16_t   memtype:1;            // Internal or External
  uint16_t   active:1;             // Whether this app is active
  uint16_t   reserved2:14;         // Place-holder
}ExistingImageInfo_t;

typedef struct 
{
  uint16_t   update_action:1;       // Whether an image update is pending
  uint16_t   src_memtype:1;         // Whether the downloaded image is in internal or external
  uint16_t   dest_memtype:1;        // Whether the executable location is in internal or external
  uint16_t   img_type:2;            // Type of the downloaded image: Encrypted app, Plain app, not-an-executable-app
  uint16_t   reserved:11;            // Place-holder
}BootInfo_t;

// App-specific fields in APP_INFO
typedef struct app_table_tag {
  char app_name[16];               // Description string for the app
  uint32_t  app_start_addr;        // Start address of the app image
  uint32_t  app_size;              // Size of the app image
  uint32_t  src_addr;              // Address where the image is downloaded
  uint32_t  dest_addr;             // Address where the image should be placed at the end of upgrade
  uint32_t  skip_pds_start;
  uint32_t  skip_pds_end;
  uint32_t  img_size;              // Size of the downloaded image
  uint8_t   imgcrc;                //CRC of the downloaded image
  ExistingImageInfo_t existingImageInfo;  // 2 Byte flag to indicate information abt existing image
  BootInfo_t bootInfo;             // 2 byte flag
  uint8_t   security_key [16];     // Security key for recovering the encryted image after download
} app_table_t;
COMPILER_PACK_RESET()

// APP_INFO table 
COMPILER_PACK_SET(1)
typedef struct {
  uint8_t         crc;                   // CRC over the entire app-info (excluding this field)
  /*
  Specifies default application index for the bootloader to jump to, 
  if the Active fields in application-related fields are in inconsistent state 
  (e.g. all set as non-active).
  */
     /*Application-configurable bit-mask indicating enabled bootloader features and interfaces:
    Bit 0: multi-image support 
    Bit 1: RF interface support
    Bit 2: USART 0 interface support
    Bit 3: USART 1 interface support
    Bit 4: SPI interface support
    Bit 5: TWI interface support
    Bit 6: USB interface support
    Bit 7-15: Reserved
  */
  uint8_t reservedbytes;
  uint8_t reservedbytes1;
  featureMask_t   required_features;     // Bit-mask for features requested by app (See boot_version.h for bit positions) 
  // Number of valid applications in internal flash. Shall be at least 1.
  uint8_t         reserved1[3];          // Place-holder
  app_table_t     appTable[1];           // App-specific sub-table for each app (upto 2)
} app_info_t;
COMPILER_PACK_RESET()

COMPILER_PACK_SET(1)
typedef struct _FotaImageHeader
{
	uint32_t magic;
	uint8_t  fotaImgHdrLen;
	uint8_t  pad1;
	uint8_t  pad2;
	uint8_t  pad3;
	uint32_t stackVersion;
	uint32_t appVersion;
	uint32_t hwVersion;
	uint32_t flags;
	uint32_t fotaImgSize;
	uint8_t  crc8;
	uint8_t  pad4;
	uint8_t  pad5;
	uint8_t  pad6;
} FotaImageHeader_t;
COMPILER_PACK_RESET()

#define TYPE_EXTERNAL 0x01      // Mem type for external flash
#define TYPE_INTERNAL 0x00      // Mem type for internal flash
#define TYPE_SECURED 0x01       // Image type for encrypted image 

#define IS_SKIP_ADDRESS_WITHIN_BUFFER(address, len, skipAddress)       \
  ((skipAddress >= address) && (skipAddress < (address+len)))
#define SKIP_ADDRESS_LENGTH             (4U)
//
// Extern variables
//

// Whether the CRC of APP_INFO is matched
// Requires the APP_COUNT field to be 1 or 2 (Otherwise APP_INFO is not valid)
extern bool isCrcMatched;

//
// Function prototypes
//
void app_infoAction(void);
void update_action(void);
void clear_updateaction(uint8_t app_index);
bool readapp_infotable(void);
void writeapp_infotable(void);
void memcopy(uint8_t dst_mem,uint32_t dst_addr,uint8_t src_mem,uint32_t src_addr,uint8_t img_type,uint32_t img_size,uint8_t app_index);
void Loadsecdatafromextmem(uint32_t dst_addr,uint32_t src_addr,uint32_t size);
uint8_t MemcalcCrc(uint8_t crc, uint8_t *pcBlock, uint8_t length);
void update_boot_info(void);
bool img_hdr_validate(void);

#endif // _APP_INFO_H

// eof app_info.h
