/**
 * \file
 *
 * \brief Implementation of low level disk I/O module skeleton for FatFS.
 *
 * Copyright (c) 2012-2018 Microchip Technology Inc. and its subsidiaries.
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
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

// #include "compiler.h"
#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "sd_mmc.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <pio/pio.h>

#if (SAM3S || SAM3U || SAM3N || SAM3XA || SAM4S || SAM4N)
# include <rtc.h>
#endif

/**
 * \defgroup thirdparty_fatfs_port_group Port of low level driver for FatFS
 *
 * Low level driver for FatFS. The driver is based on the ctrl access module
 * of the specific MCU device.
 *
 * @{
 */

/** Default sector size */
#define SECTOR_SIZE_DEFAULT 512

/**
 * \brief Initialize a disk.
 *
 * \param drv Physical drive number (0..).
 *
 * \return 0 or disk status in combination of DSTATUS bits
 *         (STA_NOINIT, STA_PROTECT).
 */
DSTATUS disk_initialize(BYTE drv)
{
	// hsmci
    pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA4);    // MCCDA
    pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA3);    // MCCK
    pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA5);    // MCDA0
    pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA6);    // MCDA1
    pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA7);    // MCDA2
    pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA8);    // MCDA3

#if (SAM3S || SAM3U || SAM3N || SAM3XA || SAM4S)
	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);
#endif
	sd_mmc_init();

	// Slot 0
	uint8_t slot = sd_mmc_nb_slot() - 1;

	
	sd_mmc_err_t code = sd_mmc_check(slot);
	if(code == SD_MMC_ERR_NO_CARD) 
		return STA_NODISK | STA_NOINIT;

	// TODO: Handle more errors
	if(code != 0)
		asm("bkpt");

	//card_version_t card_version = sd_mmc_get_version(slot);

	uint32_t sizeKBytes = sd_mmc_get_capacity(slot);
	uint32_t sizeMBytes = sizeKBytes / 1000;
	uint32_t sizeGBytes = sizeMBytes / 1000;

	return 0;
}

/**
 * \brief  Return disk status.
 *
 * \param drv Physical drive number (0..).
 *
 * \return 0 or disk status in combination of DSTATUS bits
 *         (STA_NOINIT, STA_NODISK, STA_PROTECT).
 */
DSTATUS disk_status(BYTE drv)
{
	// asm("bkpt");
	// return STA_NODISK;

	return STA_PROTECT;
}

/**
 * \brief  Read sector(s).
 *
 * \param drv Physical drive number (0..).
 * \param buff Data buffer to store read data.
 * \param sector Sector address (LBA).
 * \param count Number of sectors to read (1..255).
 *
 * \return RES_OK for success, otherwise DRESULT error code.
 */
DRESULT disk_read(BYTE drv, BYTE *buff, LBA_t startSector, UINT sectorCount)
{
	if(drv == 0) {
		// for(LBA_t sec = 0; sec < sectorCount; sec++) {
		// 	sd_mmc_err_t ret = sd_mmc_init_read_blocks(0, startSector, sectorCount);
		// 	ret = sd_mmc_start_read_blocks(buff, sectorCount);
		// 	ret = sd_mmc_wait_end_of_read_blocks(true);
			
		// 	// bool ret = hsmci_start_read_blocks(buff + (sec * SECTOR_SIZE_DEFAULT), startSector + sec);
		// 	// if (!ret) {
		// 	// 	asm("bkpt");
		// 	// }
		// 	return RES_OK;
		// }
		
		sd_mmc_err_t ret = sd_mmc_init_read_blocks(0, startSector, sectorCount);
		ret = sd_mmc_start_read_blocks(buff, sectorCount);
		ret = sd_mmc_wait_end_of_read_blocks(true);
		return RES_OK;

	}
	return RES_ERROR;
}

/**
 * \brief  Write sector(s).
 *
 * The FatFs module will issue multiple sector transfer request (count > 1) to
 * the disk I/O layer. The disk function should process the multiple sector
 * transfer properly. Do not translate it into multiple sector transfers to the
 * media, or the data read/write performance may be drastically decreased.
 *
 * \param drv Physical drive number (0..).
 * \param buff Data buffer to store read data.
 * \param sector Sector address (LBA).
 * \param count Number of sectors to read (1..255).
 *
 * \return RES_OK for success, otherwise DRESULT error code.
 */
#if _READONLY == 0
DRESULT disk_write(BYTE drv, const BYTE *buff, LBA_t sector, UINT count)
{
	asm("bkpt");
	return RES_ERROR;
}

#endif /* _READONLY */

/**
 * \brief  Miscellaneous functions, which support the following commands:
 *
 * CTRL_SYNC    Make sure that the disk drive has finished pending write
 * process. When the disk I/O module has a write back cache, flush the
 * dirty sector immediately.
 * In read-only configuration, this command is not needed.
 *
 * GET_SECTOR_COUNT    Return total sectors on the drive into the DWORD variable
 * pointed by buffer.
 * This command is used only in f_mkfs function.
 *
 * GET_BLOCK_SIZE    Return erase block size of the memory array in unit
 * of sector into the DWORD variable pointed by Buffer.
 * When the erase block size is unknown or magnetic disk device, return 1.
 * This command is used only in f_mkfs function.
 *
 * GET_SECTOR_SIZE    Return sector size of the memory array.
 *
 * \param drv Physical drive number (0..).
 * \param ctrl Control code.
 * \param buff Buffer to send/receive control data.
 *
 * \return RES_OK for success, otherwise DRESULT error code.
 */
DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
	asm("bkpt");
	DRESULT res = RES_PARERR;

	switch (ctrl) {
	case GET_BLOCK_SIZE:
		*(DWORD *)buff = 1;
		res = RES_OK;
		break;

	case GET_SECTOR_COUNT:
	{
		uint32_t ul_last_sector_num = 0;
		res = RES_OK;
	}
	break;

	/* Get sectors on the disk (WORD) */
	case GET_SECTOR_SIZE:
	{
		// uint8_t uc_sector_size = mem_sector_size(drv);

		// if ((uc_sector_size != SECTOR_SIZE_512) &&
		// 		(uc_sector_size != SECTOR_SIZE_1024) &&
		// 		(uc_sector_size != SECTOR_SIZE_2048) &&
		// 		(uc_sector_size != SECTOR_SIZE_4096)) {
		// 	/* The sector size is not supported by the FatFS */
		// 	return RES_ERROR;
		// }

		// *(U8 *)buff = uc_sector_size * SECTOR_SIZE_DEFAULT;

		res = RES_OK;
	}
	break;

	/* Make sure that data has been written */
	case CTRL_SYNC:

		break;

	default:
		res = RES_PARERR;
	}

	return res;
}




/**
 * \brief Current time returned is packed into a DWORD value.
 *
 * The bit field is as follows:
 *
 * bit31:25  Year from 1980 (0..127)
 *
 * bit24:21  Month (1..12)
 *
 * bit20:16  Day in month(1..31)
 *
 * bit15:11  Hour (0..23)
 *
 * bit10:5   Minute (0..59)
 *
 * bit4:0    Second (0..29)
 *
 * \return Current time.
 */
uint32_t get_fattime(void)
{
	uint32_t ul_time;
	uint32_t ul_hour, ul_minute, ul_second;
	uint32_t ul_year, ul_month, ul_day, ul_week;

	/* Retrieve date and time */
	rtc_get_time(RTC, &ul_hour, &ul_minute, &ul_second);
	rtc_get_date(RTC, &ul_year, &ul_month, &ul_day, &ul_week);

	ul_time = ((ul_year - 1980) << 25)
			| (ul_month << 21)
			| (ul_day << 16)
			| (ul_hour << 11)
			| (ul_minute << 5)
			| ((ul_second >> 1) << 0);

	return ul_time;
}


//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
