/**
* \file  fuota_nvm.h
*
* \brief Interface headers for accessign NVM to perform FUOTA image related storage
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

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include "status_codes.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
/*
 * Bootloader             =  16k max; (0x00000000 to 0x00003fFF)
 * Image_1 (active)       = 120k max; (0x00004000 to 0x00021fFF) (size = 0x1E000)
 * Image_2 (over-the-air) = 120k max; (0x00022000 to 0x0003ffFF) (size = 0x1E000)
 */

// After boot loader (16k)
#define FLASH_AREA_IMAGE_1_BASE_OFFSET          (0x00004000)

// After image_1 (136k)
#define FLASH_AREA_IMAGE_2_BASE_OFFSET          (0x00022000)       

// last 2 rows of FLASH
#define APP_INFO_START_ADDRESS                  (0x0003FE00)       

#define FLASH_AREA_IMAGE_HEADER_FILL            (32)

#define TYPE_INTERNAL                           (0)
#define TYPE_EXTERNAL                           (1)

// Type of flash memory
#define FUOTA_NVM_TYPE                          (TYPE_INTERNAL)

/************************************************************************/
/* TYPES                                                                */
/************************************************************************/
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

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/
/**
 * \brief Initialize the non volatile memory specified.
 */
status_code_t FUOTA_NVM_Init( void );

/**
 * \brief Read \a len number of bytes from address \a address in non volatile
 * memory \a mem and store it in the buffer \a buffer
 *
 * \param address Address to read
 * \param buffer Pointer to destination buffer
 * \param len Number of bytes to read
 */
status_code_t FUOTA_NVM_Read(uint32_t address, void *buffer, uint32_t len);

/**
 * \brief Write \a len number of bytes at address \a address in non volatile
 * memory \a mem from the buffer \a buffer
 *
 * \param address Address to write
 * \param buffer Pointer to source buffer
 * \param len Number of bytes to write
 */
status_code_t FUOTA_NVM_Write(uint32_t address, void *buffer, uint32_t len);

/* eof fuota_nvm.h */
