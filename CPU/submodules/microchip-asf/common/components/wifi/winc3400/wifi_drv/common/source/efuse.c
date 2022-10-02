/**
 *
 * \file efuse.c
 *
 * \brief This module contains efuse APIs.
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
#include "common/include/efuse.h"

#define _EFUSE_ 1

#ifdef _FIRMWARE_
#define m2mmemset memset
#else
#define m2mmemset m2m_memset
#endif



/* Efuse variables  */

EFUSEProdStruct g_efuse_struct;


/* Efuse functions */

sint8 is_efuse_bank_loaded(uint8 bankIdx)
{
	uint32 bankAddr;
	uint32 val32;

	// Connect internal efuse_vddq to GND
	bankAddr = (bankIdx < 2) ? WIFI_EFUSE_0_CONTROL + bankIdx * 4 : WIFI_EFUSE_2_CONTROL + (bankIdx - 2) * 4;

	val32 = nm_read_reg(bankAddr);

	if((val32 & (1ul << 31)) == (1ul << 31)) {
		return 1;
	} else {
		return 0;
	}
}

int load_efuse_to_regs(uint8 bankIdx)
{
	int retry;
	uint32 bankAddr;
	uint32 val32;

	// Connect internal efuse_vddq to GND
	val32 = nm_read_reg(0x1428);
	nm_write_reg(0x1428, (val32 & (~(1 << 18))));

	bankAddr = (bankIdx < 2) ? WIFI_EFUSE_0_CONTROL + bankIdx * 4 : WIFI_EFUSE_2_CONTROL + (bankIdx - 2) * 4;
	nm_write_reg(bankAddr, 0x007c082d);

	retry = 100;
	do {
#ifdef _FIRMWARE_
		//add_delay(0xFFF);	//~ 10 ms
		// Diab: why do we need 10 ms ?
		// try 0x333
		add_delay(0xcc);
#else
		nm_bsp_sleep(10);
#endif
		val32 = nm_read_reg(bankAddr);
	} while ((retry-- > 0) && ((val32 & (1ul << 31)) != (1ul << 31)));

	if (retry <= 0) {
		return EFUSE_ERR_CANT_LOAD_DATA;
	}

	return EFUSE_SUCCESS;
}

//#define TEST_EFUSE
#ifdef TEST_EFUSE
static int read_from_efuse(uint8 bankIdx, uint8 *buf_in)
{
	uint32 * buf = (uint32 *)buf_in;
	uint32 test[NUM_EFUSE_BANKS][4] = {
			{0xc0000000, 0x00000000, 0x00000000, 0x00000000 },
			{0xc0000000, 0x00000000, 0x00000000, 0x00000000 },
			{0xc1aabbcc, 0xddeeff00, 0x00000000, 0x00000000 },
			{0xc0000000, 0x00000000, 0x00000000, 0x00000000 },
			{0xc0000000, 0x00000000, 0x00000000, 0x00000000 },
			{0xc1aabbcc, 0xddeeff00, 0x00000000, 0x00000000 },
	};
	if(bankIdx < NUM_EFUSE_BANKS) {
		buf[0] = test[bankIdx][0];
		buf[1] = test[bankIdx][1];
		buf[2] = test[bankIdx][2];
		buf[3] = test[bankIdx][3];
		return EFUSE_SUCCESS;
	} else {
		return EFUSE_ERR_CANT_LOAD_DATA;
	}
}
#else
static int read_from_efuse(uint8 bankIdx, uint8 *buf)
{
	uint32 bankAddr, val32;
	int i;
	bankAddr = (bankIdx < 2) ? 0x102c + bankIdx * 32 : 0x1380 + (bankIdx - 2) * 16;
	//rRegBurst(bankAddr, (uint8 *)&buf[0], 16);
	for (i = 0; i < 4; i++)
	{
		val32 = nm_read_reg(bankAddr+i*4);
		buf[i * 4] = (uint8) (val32 & 0xff);
		buf[i * 4 + 1] = (uint8) ((val32 >> 8) & 0xff);
		buf[i * 4 + 2] = (uint8) ((val32 >> 16) & 0xff);
		buf[i * 4 + 3] = (uint8) ((val32 >> 24) & 0xff);
	}
	return EFUSE_SUCCESS;
}
#endif

sint8 read_efuse_struct(EFUSEProdStruct *efuse_struct, uint8 skip_bank_check)
{
#ifdef _EFUSE_
	int bankIdx;
	uint8 skip_bank_5, skip_bank_4;
	uint32 bank_data[4];

	skip_bank_5 = 0;
	skip_bank_4 = 0;

	for (bankIdx = 0; bankIdx < NUM_EFUSE_BANKS; bankIdx++) {

		if (load_efuse_to_regs(bankIdx) < 0) {
			M2M_ERR("(Efuse)Error: Can't load data to registers.\n");
			/* Clear the output structure to remove any stale garbage data in
			 * efuse_struct before returning error.
			 */
			m2mmemset((uint8*) efuse_struct, 0, sizeof(EFUSEProdStruct));
			return EFUSE_ERR_CANT_LOAD_DATA; /* can't load data to registers*/
		}

		read_from_efuse(bankIdx, (uint8 *) &bank_data[0]);

		M2M_DBG("(Efuse) bank_data = %08x %08x %08x %08x\n",
			bank_data[0],
			bank_data[1],
			bank_data[2],
			bank_data[3]);

		if(skip_bank_check) {
			if(bankIdx == 2) { /* efuse 2 */
				if( (bank_data[3] & (0x7ul << 0)) != 0) {
					skip_bank_5 = 1;
				}
				if( (bank_data[3] & (0x7ul << 5)) != 0) {
					skip_bank_4 = 1;
				}
			}
			else if(
				((bankIdx == 4) && skip_bank_4) ||
				((bankIdx == 5) && skip_bank_5)) {
				M2M_INFO("(Efuse) Skip bank %d\n", bankIdx);
				continue;
			}
		}

		efuse_struct->bank_idx          = (uint8)bankIdx;
		efuse_struct->bank_used         = (uint8)(bank_data[0] >> 31) & 0x1;
		efuse_struct->bank_invalid      = (uint8)(bank_data[0] >> 30) & 0x1;

		if ((efuse_struct->bank_used == 1) && (efuse_struct->bank_invalid == 0))
		{
			efuse_struct->ver               = (uint8)(bank_data[0] >> 25) & 0x7;
			if(efuse_struct->ver == 0) {
				efuse_struct->MAC_addr_used     = (uint8)(bank_data[0] >> 24) & 0x1;
				efuse_struct->MAC_addr[0]       = (uint8)(bank_data[0] >> 16) & 0xff;
				efuse_struct->MAC_addr[1]       = (uint8)(bank_data[0] >> 8) & 0xff;
				efuse_struct->MAC_addr[2]       = (uint8)(bank_data[0] >> 0) & 0xff;
				efuse_struct->MAC_addr[3]       = (uint8)(bank_data[1] >> 24) & 0xff;
				efuse_struct->MAC_addr[4]       = (uint8)(bank_data[1] >> 16) & 0xff;
				efuse_struct->MAC_addr[5]       = (uint8)(bank_data[1] >> 8) & 0xff;
				efuse_struct->PATxGainCorr_used	= (uint8)(bank_data[1] >> 7) & 0x1;
				efuse_struct->PATxGainCorr      = (uint8)(bank_data[1] >> 0) & 0x7f;
				efuse_struct->FreqOffset_used   = (uint16)(bank_data[2] >> 31) & 0x1;
				efuse_struct->FreqOffset        = (uint16)(bank_data[2] >> 16) & 0x7fff;
				M2M_DBG("(Efuse) successfully loaded from bank %d.\n", bankIdx);
#ifndef _FIRMWARE_
				dump_efuse_struct(efuse_struct);
#endif
				return EFUSE_SUCCESS; /*Terminate this function as success procedure*/
			} else {
				//M2M_INFO("(Efuse) Bank %d: unsupported version %02x\n", bankIdx, efuse_struct->ver);
			}

		}
	}
	//M2M_INFO("(Efuse) No bank used or invalid data.\n");
#endif

	/* Clear the output structure to remove any stale garbage data in
	 * efuse_struct before returning error.
	 */
	m2mmemset((uint8*) efuse_struct, 0, sizeof(EFUSEProdStruct));
	return EFUSE_ERR_INVALID_BANK_OR_DATA;	/* Can't find a valid data in any bank*/
}

void dump_efuse_struct(EFUSEProdStruct *efuse_struct)
{
	M2M_INFO("----------- BEGIN EFUSE DUMP ----------------\n");
	M2M_INFO("(Efuse)Ver = %u,bank idx = %u,used = %u,invalid = %u\n",
		efuse_struct->ver,
		efuse_struct->bank_idx,
		efuse_struct->bank_used,
		efuse_struct->bank_invalid);
	M2M_INFO("(Efuse)Valid = %u,MAC = %02x:%02x:%02x:%02x:%02x:%02x\n",
		efuse_struct->MAC_addr_used,
		efuse_struct->MAC_addr[0],
		efuse_struct->MAC_addr[1],
		efuse_struct->MAC_addr[2],
		efuse_struct->MAC_addr[3],
		efuse_struct->MAC_addr[4],
		efuse_struct->MAC_addr[5]);

	M2M_INFO("(Efuse)Valid = %u,PATxGainCorr = %02x\n",
		efuse_struct->PATxGainCorr_used,
		efuse_struct->PATxGainCorr);

	M2M_INFO("(Efuse)Valid = %u,FreqOffset = %04x\n",
		efuse_struct->FreqOffset_used,
		efuse_struct->FreqOffset);
	M2M_INFO("------------- END EFUSE DUMP ----------------\n");
}

#ifndef _FIRMWARE_

static int write_to_efuse(uint8 bankIdx , uint8 *buf)
{
	int retry, i;
	uint32 bankAddr;
	uint32 val32;

	// Connect internal efuse_vddq to paldo
	val32 = nm_read_reg(0x1428);
	nm_write_reg(0x1428, (val32 | (1 << 18)));

	//Force Tx mode to Enable paldo
	val32 = nm_read_reg(0x1480);
	val32 &= ~0xffff0f00;
	val32 |= 0x13330300;
	nm_write_reg(0x1480, val32);

	val32 = nm_read_reg(0x1484);
	val32 &= ~0xff;
	val32 |= 0x11;
	nm_write_reg(0x1484, val32);


	bankAddr = (bankIdx<2)?0x101c+bankIdx*32:0x1340+(bankIdx-2)*16;
	//wRegBurst(bankAddr, (uint8 *)&buf[0], 16);
	for (i = 0; i < 4; i++)
	{
		val32 = buf[i*4] |
			(( buf[i*4+1] & 0xff) << 8) |
			(( buf[i*4+2] & 0xff) << 16) |
			(( buf[i*4+3] & 0xff) << 24);
		nm_write_reg(bankAddr+i*4 , val32);
	}


	bankAddr = (bankIdx<2)?0x1014+bankIdx*4:0x1320+(bankIdx-2)*4;
	// Clear the FUSE bit.
	nm_write_reg(bankAddr, 0x007c081c);
	// Set the FUSE bit.
	nm_write_reg(bankAddr, 0x007c081e);

	retry = 100;
	do {
		nm_bsp_sleep(10);
		val32 = nm_read_reg(bankAddr);
	} while( (retry-- > 0) &&
		((val32 & (1ul << 31)) != (1ul << 31)) );

	// Clear the FUSE bit.
	nm_write_reg(bankAddr, 0x007c081c);

	// Connect internal efuse_vddq to GND
	val32 = nm_read_reg(0x1428);
	nm_write_reg(0x1428, (val32 & (~(1 << 18))));

	val32 = nm_read_reg(0x1480);
	val32 &= ~0xffff0f00;
	nm_write_reg(0x1480, val32);

	val32 = nm_read_reg(0x1484);
	val32 &= ~0xff;
	nm_write_reg(0x1484, val32);

	if(retry <= 0) {
		return -1;
	}

	return 0;
}

sint8 overwrite_efuse_struct(EFUSEProdStruct *efuse_struct, int bankIdx)
{
	sint8 ret;
	uint32 bank_data[4];

	if (load_efuse_to_regs(bankIdx) < 0) {
		return -1;
	}


	m2mmemset((uint8*)&bank_data, 0, sizeof(uint32)*4);
	bank_data[0] |= (efuse_struct->bank_used << 31);
	bank_data[0] |= (efuse_struct->bank_invalid << 30);
	bank_data[0] |= ((efuse_struct->ver & 0x7 )	<< 25);
	bank_data[0] |= (efuse_struct->MAC_addr_used << 24);
	bank_data[0] |= (efuse_struct->MAC_addr[0] & 0xff) <<16;
	bank_data[0] |= (efuse_struct->MAC_addr[1] & 0xff) <<8;
	bank_data[0] |= (efuse_struct->MAC_addr[2] & 0xff) ;
	bank_data[1] |= (efuse_struct->MAC_addr[3] & 0xff) <<24;
	bank_data[1] |= (efuse_struct->MAC_addr[4] & 0xff) <<16;
	bank_data[1] |= (efuse_struct->MAC_addr[5] & 0xff) <<8;
	bank_data[1] |= (efuse_struct->PATxGainCorr_used << 7);
	bank_data[1] |= (efuse_struct->PATxGainCorr & 0x7f);
	bank_data[2] |= (efuse_struct->FreqOffset_used << 31);
	bank_data[2] |= (efuse_struct->FreqOffset & 0x7fff) << 16;

	/* Write to Efuse */
	ret = write_to_efuse(bankIdx , (uint8 *)&bank_data[0]);

	if(ret == 0) {
		return 0;
	} else {
		return -2;
	}
}
#endif /* _FIRMWARE_ */
