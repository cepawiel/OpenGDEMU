/**
 * \file
 *
 * \brief OTAU Flash Interface Driver
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
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
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
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
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Atmel
 * Support</a>
 */

#ifndef OFD_H
#define OFD_H

/** Flash header size to be read */
#define FLASH_DETECT_SECTION_NUM        0x8

/** Flash header start address */
#define FLASH_HEADER_START_ADDR         0x000000

/** Flash number of loadable section: 3 */
#define OFD_FLASH_LOADABLE_SECTION  0x3

/** FW Patch section index0 start address : 0x001000 */
#define OFD_FW_SECTION_INDEX0_START_ADDR  0x001000

/** Accessible meta data start address */
#define META_DATA_START_ADDRESS			0x03C000

/** Accessible meta data total size */
#define META_DATA_TOTAL_SIZE		    (uint16_t)FLASH_SECTOR_SIZE

/**@brief OTA operation status
*/
typedef enum
{
	AT_OFID_OP_SUCESS,
	AT_OFID_FLASH_EMPTY_IMAGE,
	AT_OFID_INVALID_SECTION_ID,
	AT_OFID_INVALID_ADDR,
	AT_OFID_INVALID_LENGTH,
} ofid_op_status_t;

/**
 * @brief ofid_section_info_t represent the section info
 */
typedef struct
{
	/** Section ID Value */
	uint8_t id;
	/** Section image count */
	uint8_t image_cnt;
	/** Section start address */
	uint32_t start_addr;
	/** Section end address */
	uint32_t end_addr;
	/** Section size */
	uint32_t size;	
}ofid_section_info_t; 

/** 
* @brief Flash and meta data related information
*/
typedef struct
{
	/** Total flash size */
	uint32_t             flash_size;
	/** Number of loadable section */
	uint8_t              num_section;
	/** Section ID info */
	ofid_section_info_t   section_info[OFD_FLASH_LOADABLE_SECTION];
	/** Meta data start address */
	uint32_t             metadata_addr;
	/** Meta data size */
	uint16_t              metadata_size;
	/** Page size */
	uint16_t			 page_size;
} ofid_flash_info_t;

/**
* @brief data related information
*/
typedef struct data_info
{
	/** Represent the section ID */
	uint8_t section_id;
	/** Pointer to flash info */
	ofid_flash_info_t *flash_info;
	/** Pointer to data buffer */
	uint8_t *data_buf;
	/** Flash Address */
	uint32_t flash_addr;
	/** Size of the data */
	uint32_t size;
}ofid_data_info_t;


/*Initialize the OTA flash interface layer*/
ofid_op_status_t ofid_init(void *param);

/*De initialize the OTA flash interface layer*/
ofid_op_status_t ofid_deinit(ofid_flash_info_t * flash_inst);

/*Get the flash info*/
ofid_op_status_t ofid_get_flashinfo(ofid_flash_info_t * flash_inst);

/*Validate section ID*/
ofid_op_status_t ofid_validate_id(ofid_data_info_t *data);

/*Validate flash address*/
ofid_op_status_t ofid_validate_addr(ofid_data_info_t *data);

/*Read SPI flash memory*/
ofid_op_status_t ofid_read_data(ofid_data_info_t *data);

/*Erase SPI flash sector memory*/
ofid_op_status_t ofid_erase_sector(ofid_data_info_t *data);

/* Erases the memory, here mostly sector size */
ofid_op_status_t ofid_erase_memory(uint32_t address, uint32_t size);

/*Write SPI flash memory*/
ofid_op_status_t ofid_write_data(ofid_data_info_t *data);

/*Write meta data info*/
ofid_op_status_t ofid_write_metadeta(ofid_data_info_t *data);

/*Erase meta data info*/
ofid_op_status_t ofid_erase_metadata(ofid_data_info_t *data);

/*Read meta data info*/
ofid_op_status_t ofid_read_metadeta(ofid_data_info_t *data);

/*Reset the device*/
void otau_device_reset(void);

#endif /* OFD_H */