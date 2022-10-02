/**
 * \file *********************************************************************
 *
 * \brief Implementation of vendor-specific data handling
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
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
 */

/*
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

#ifdef VENDOR_DATA

/* === INCLUDES ============================================================ */

#include <stdio.h>

#include "rf4ce.h"
#include "zrc.h"
#include "vendor_data.h"
#include "pal.h"
#include "tal.h"
#ifdef FLASH_SUPPORT
#include "pal_flash.h"
#endif
#ifdef BOOT_FLASH
#include "pal_boot_flash.h"
#endif
#ifdef TFA_BAT_MON
#include "tfa.h"
#endif
#include "conf_board.h"
/* === MACROS ============================================================== */

/* Flash start address of the firmware image .*/
#define IMAGE_START_ADDR            ((uint32_t)60 * 1024)

/* Maximum size of the firmware image .*/
#define IMAGE_SIZE                  ((uint32_t)60 * 1024)

/* === EXTERNALS =========================================================== */

FLASH_EXTERN(uint16_t VendorIdentifier);
#ifdef RF4CE_CALLBACK_PARAM
#ifdef RF4CE_TARGET
extern void vendor_data_confirm(nwk_enum_t Status, uint8_t PairingRef, profile_id_t ProfileId
#ifdef NLDE_HANDLE
		, uint8_t Handle
#endif
		);

#else /* RF4CE_TARGET */
extern void nlme_rx_enable_confirm(nwk_enum_t Status);
static void vendor_data_confirm(nwk_enum_t Status, uint8_t PairingRef, profile_id_t ProfileId
#ifdef NLDE_HANDLE
		, uint8_t Handle
#endif
		);
void vendor_data_ind(uint8_t PairingRef, uint16_t VendorId,
		uint8_t nsduLength, uint8_t *nsdu, uint8_t RxLinkQuality,
		uint8_t RxFlags);

#endif /* RF4CE_TARGET */
#endif /* RF4CE_CALLBACK_PARAM */

/* === IMPLEMENTATION ====================================================== */

bool vendor_data_request(uint8_t PairingRef, profile_id_t ProfileId,
		uint16_t VendorId, uint8_t nsduLength, uint8_t *nsdu,
		uint8_t TxOptions)
{
	/* Keep compiler happy */
	ProfileId = ProfileId;

	return nlde_data_request(PairingRef, PROFILE_ID_ZRC, VendorId,
			nsduLength, nsdu, TxOptions
#ifdef NLDE_HANDLE
			, 1
#endif
#ifdef RF4CE_CALLBACK_PARAM
			, (FUNC_PTR)vendor_data_confirm
#endif
			);
}

#ifndef RF4CE_TARGET
void vendor_data_ind(uint8_t PairingRef, uint16_t VendorId,
		uint8_t nsduLength, uint8_t *nsdu, uint8_t RxLinkQuality,
		uint8_t RxFlags)
{
	/* Check if vendor id matches.
	 * Handle here only vendor data from same vendor */
	uint16_t v_id = PGM_READ_WORD(&VendorIdentifier);
	uint8_t nsduHandle = 1;
	if ((VendorId == v_id) && (RxFlags & RX_FLAG_WITH_SEC)) {
		switch (nsdu[0]) { /* vendor-specific command id */
#ifdef TFA_BAT_MON
		case BATTERY_STATUS_REQ:
		{
			uint16_t voltage = tfa_get_batmon_voltage();
			nsdu[0] = BATTERY_STATUS_RESP;
			nsdu[1] = (uint8_t)voltage; /* LSB */
			nsdu[2] = (uint8_t)(voltage >> 8); /* MSB */
			nsduLength = 3;
		}
		break;
#endif
		case ALIVE_REQ: /* Alive request */
			vendor_app_alive_req();
			/* Send alive response */
			nsdu[0] = ALIVE_RESP;
			nsduLength = 1;
			break;

		case FW_VERSION_REQ:
		{
			/* Send alive response */
			nsdu[0] = FW_VERSION_RESP;
			nsdu[1] = FW_VERSION_MAJOR; /* major version number */
			nsdu[2] = FW_VERSION_MINOR; /* minor version number */
			nsdu[3] = FW_VERSION_REV; /* revision version number */
			nsduLength = 4;
		}
		break;

		case RX_ON_REQ:
		{
			uint32_t duration = 0;

			memcpy(&duration, &nsdu[1], 3);
			if (!nlme_rx_enable_request(duration
#ifdef RF4CE_CALLBACK_PARAM
					, (FUNC_PTR)nlme_rx_enable_confirm
#endif
					)) {
				/*
				 * RX enable could not been added to the queue.
				 * Therefore do not send response message.
				 */
				return;
			}

			/* Send response */
			nsdu[0] = RX_ON_RESP;
			nsduLength = 1;
		}
		break;

#ifdef FLASH_SUPPORT
		case FW_DATA_REQ:
		{
			fw_data_frame_t *fw_frame;
			vendor_status_t status = VD_SUCCESS;

			fw_frame = (fw_data_frame_t *)nsdu;

			/* Verify data chunk size */
			uint8_t fw_data_size = nsduLength - 5; /* 5 = header len
				                                **/
			if (fw_data_size > 64) {
				status = VD_UNSUPPORTED_SIZE;
			} else {
				/* Fill temporary page buffer */
				uint16_t start_addr
					= (fw_frame->frame_cnt -
						1) % 4;
				flash_fill_page_buffer(start_addr *
						(SPM_PAGESIZE / 4),
						fw_data_size,
						&fw_frame->fw_data[0]);
				/* Write flash page */
				if ((fw_frame->frame_cnt % 4) == 0) {
					uint32_t page_start_addr;
					page_start_addr = IMAGE_START_ADDR +
							((uint32_t)
							SPM_PAGESIZE *
							((fw_frame->
							frame_cnt /
							4) - 1));
					flash_program_page(page_start_addr);
				} else if (fw_frame->frame_cnt ==
						fw_frame->total_num_frames) {
					uint32_t page_start_addr;
					page_start_addr = IMAGE_START_ADDR +
							((uint32_t)
							SPM_PAGESIZE *
							(fw_frame->
							frame_cnt / 4));
					flash_program_page(page_start_addr);
				}
			}

			/* Send response */
			nsdu[0] = FW_DATA_RESP;
			nsdu[1] = status;
			nsduLength = 2;
		}
		break;
#endif  /* #ifdef FLASH_SUPPORT */

#ifdef FLASH_SUPPORT
		case FW_SWAP_REQ:
		{
			/* Send response */
			nsdu[0] = FW_SWAP_RESP;
			nsduLength = 1;
			nsduHandle = 2;
		}
		break;
#endif  /* #ifdef FLASH_SUPPORT */

		default:
		{
			/* Send response */
			nsdu[0] = FW_DATA_RESP;
			nsdu[1] = VD_NOT_SUPPORTED_ATTRIBUTE;
			nsduLength = 2;
		}
		break;
		}

		/* Transmit response message */

		nlde_data_request(PairingRef, PROFILE_ID_ZRC, VendorId,
				nsduLength, nsdu,
				TXO_UNICAST | TXO_DST_ADDR_NET | TXO_ACK_REQ | TXO_SEC_REQ | TXO_MULTI_CH | TXO_CH_NOT_SPEC | TXO_VEND_SPEC
#ifdef NLDE_HANDLE
				, nsduHandle
#endif
#ifdef RF4CE_CALLBACK_PARAM
				, (FUNC_PTR)vendor_data_confirm
#endif
				);
		/* Keep compiler happy */
		RxLinkQuality = RxLinkQuality;
		RxFlags = RxFlags;
	}
}

#endif  /* #ifndef RF4CE_TARGET */

#ifndef RF4CE_TARGET
#ifdef RF4CE_CALLBACK_PARAM
static
#endif
void vendor_data_confirm(nwk_enum_t Status, uint8_t PairingRef, profile_id_t ProfileId
#ifdef NLDE_HANDLE
		, uint8_t Handle
#endif
		)
{
#ifdef FLASH_SUPPORT
	if (Handle == 2) {
		/* This is the confirm for SWAP_RSP
		 * so initiate flash swap for activiting the new image */
		flash_swap(IMAGE_START_ADDR, IMAGE_SIZE);
	}
#endif
	Status = Status;
	PairingRef = PairingRef;
#ifdef NLDE_HANDLE
	Handle = Handle;
	ProfileId = ProfileId;
#endif
}

#endif

#endif  /* #ifdef VENDOR_DATA */
