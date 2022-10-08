/**
 * \file
 *
 * \brief
 *
 * Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
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
 *Support</a>
 */

#ifndef _EFUSE_H_
#define _EFUSE_H_

#define NUM_EFUSE_BANKS (6)

#ifdef _FIRMWARE_
#include "m2m_common.h"
#include "common.h"
#else
#include "bsp/include/nm_bsp.h"
#include "driver/source/nmbus.h"
#endif

#define	WIFI_EFUSE_0_CONTROL	(0x1014)
#define	WIFI_EFUSE_2_CONTROL	(0x1320)

#define EFUSE_ERR_CANT_LOAD_DATA		-2
#define EFUSE_ERR_INVALID_BANK_OR_DATA	-1
#define EFUSE_SUCCESS					 0

typedef struct {
	uint8 ver:3;
	uint8 bank_idx:3; 			/* bank number (index) */
	uint8 bank_used:1;			/* =1 if bank is used otherwise =0 */
	uint8 bank_invalid:1;		/* =1 if bank has INVALID data otherwise = 0 */
	uint8 MAC_addr_used:1;		/* =1 if mac address is used otherwise =0 */

	uint8 PATxGainCorr:7;
	uint8 PATxGainCorr_used:1;
	uint16 FreqOffset:15;		/* has frequency for tuner */
	uint16 FreqOffset_used:1;	/* =1 if frequency for tuner is used otherwise =0 */

	uint8 MAC_addr[6];			/* has The MAC address value */
} EFUSEProdStruct;

sint8 is_efuse_bank_loaded(uint8 bankIdx);
int load_efuse_to_regs(uint8 bankIdx);
sint8 read_efuse_struct(EFUSEProdStruct *efuse_struct, uint8 skip_bank_check);
void dump_efuse_struct(EFUSEProdStruct *efuse_struct);

extern EFUSEProdStruct 	g_efuse_struct;

#ifndef _FIRMWARE_

sint8 overwrite_efuse_struct(EFUSEProdStruct *efuse_struct, int bankIdx);

#endif /* _FIRMWARE_ */
#endif /* _EFUSE_H_ */
