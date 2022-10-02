/**
 * @file mac_data_structures.h
 *
 * @brief This file contains MAC related data structures, types and enums.
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
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
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef MAC_DATA_STRUCTURES_H
#define MAC_DATA_STRUCTURES_H

/* === Includes ============================================================= */

#include "mac.h"

/* === Macros =============================================================== */

/* === Types ================================================================ */

/**
 * \addtogroup group_mac_ds
 * @{
 */
#ifdef GTS_SUPPORT

typedef enum gts_state_tag {
	GTS_STATE_IDLE = 0,
	GTS_STATE_REQ_SENT,
	GTS_STATE_ALLOCATED
} gts_state_t;

/**
 * @brief Device GTS table structure
 *
 * @ingroup apiMacTypes
 */
__PACK__DATA__
typedef struct mac_dev_gts_mgmt_tag {
	uint8_t *GtsReq_ptr;
	uint8_t GtsStartingSlot;
	uint8_t GtsLength;
	uint8_t GtsPersistCount;
	gts_state_t GtsState;
} mac_dev_gts_mgmt_t;

typedef struct mac_gts_spec_tag {
	uint8_t GtsDescCount : 3;
	uint8_t Reserved : 4;
	uint8_t GtsPermit : 1;
} mac_gts_spec_t;

typedef struct mac_gts_list_tag {
	uint8_t dev_addr[2];
	uint8_t starting_slot : 4;
	uint8_t length : 4;
} mac_gts_list_t;
#endif /* GTS_SUPPORT */

/**
 * Beacon Payload type
 */
typedef struct mac_beacon_payload_tag {
	uint16_t superframe_spec;
	uint8_t gts_spec;
#ifdef GTS_SUPPORT
	uint8_t gts_direction;
	mac_gts_list_t *gts_list;
#endif /* GTS_SUPPORT */
	uint8_t pending_addr_spec;
	uint8_t *pending_addr_list;
	uint8_t beacon_payload_len;
	uint8_t *beacon_payload;
} mac_beacon_payload_t;

/**
 * Data Payload type
 */
typedef struct mac_data_payload_tag {
	uint8_t *payload;
} mac_data_payload_t;

/**
 * Association Request type
 */
typedef struct mac_assoc_req_tag {
	uint8_t capability_info;
} mac_assoc_req_t;

/**
 * Association Response type
 */
typedef struct mac_assoc_response_tag {
	uint16_t short_addr;
	uint8_t assoc_status;
} mac_assoc_response_t;

/**
 * Disassociation Request type
 */
typedef struct mac_disassoc_req_tag {
	uint8_t disassoc_reason;
} mac_disassoc_req_t;

/**
 * Coordinator Realignment type
 */
typedef struct mac_coord_realign_tag {
	uint16_t pan_id;
	uint16_t coord_short_addr;
	uint8_t logical_channel;
	uint16_t short_addr;
	uint8_t channel_page;
} mac_coord_realign_t;

/**
 * General Command frame payload type
 */
typedef union {
	mac_beacon_payload_t beacon_data;
	mac_data_payload_t data;
	mac_assoc_req_t assoc_req_data;
	mac_assoc_response_t assoc_response_data;
	mac_disassoc_req_t disassoc_req_data;
	mac_coord_realign_t coord_realign_data;
#ifdef GTS_SUPPORT
	gts_char_t gts_req_data;
#endif /* GTS_SUPPORT */
} frame_payload_t;

/**
 * Structure containing auxiliary security header information
 */
typedef struct sec_ctrl_tag {
	uint8_t sec_level : 3;
	uint8_t key_id_mode : 2;
	uint8_t /* reserved */ : 3;
} sec_ctrl_t;

typedef struct parse_tag {
	uint16_t fcf;
	uint8_t frame_type;
	uint8_t mpdu_length;
	uint8_t sequence_number;
	uint8_t dest_addr_mode;
	uint16_t dest_panid;
	address_field_t dest_addr;
	uint8_t src_addr_mode;
	uint16_t src_panid;
	address_field_t src_addr;
#if ((defined MAC_SECURITY_ZIP)  || (defined MAC_SECURITY_2006))
	sec_ctrl_t sec_ctrl;
	uint8_t key_id_len;
	uint32_t frame_cnt;
	uint8_t key_id[MAX_KEY_ID_FIELD_LEN];
#endif  /* #if ((defined MAC_SECURITY_ZIP)  || (defined MAC_SECURITY_2006)) */
	uint8_t mac_command;
	uint8_t ppdu_link_quality;
#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP)

	/*
	 * The timestamping is only required for beaconing networks
	 * or if timestamping is explicitly enabled.
	 */
	uint32_t time_stamp;
#endif  /* #if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) */
	uint8_t mac_payload_length; /* Length of the MAC payload without FCS */
	frame_payload_t mac_payload_data;
} parse_t;
/* ! @} */

#ifdef GTS_SUPPORT
typedef struct mac_gts_desc_tag {
	uint8_t GtsStartingSlot;
	uint8_t GtsLength;
	bool GtsDirection;
} mac_gtsDesc_t;

/**
 * @brief PANC GTS table structure
 *
 * @ingroup apiMacTypes
 */
typedef struct mac_pan_gts_mgmt_tag {
	queue_t *gts_data_q;
	uint16_t DevShortAddr;
	uint16_t ExpiryCount;
	mac_gtsDesc_t GtsDesc;
	uint8_t PersistenceCount;
} mac_pan_gts_mgmt_t;
#endif /* GTS_SUPPORT */
__PACK__RST_DATA__
/* === Externals ============================================================ */

/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MAC_DATA_STRUCTURES_H */
/* EOF */
