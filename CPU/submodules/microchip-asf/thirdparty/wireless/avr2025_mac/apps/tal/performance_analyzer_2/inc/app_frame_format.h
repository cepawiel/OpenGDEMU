/**
 * \file app_frame_format.h
 *
 * \brief Defines various structure definitions, used to communicate with
 * Peer node over the air - Performance Analyzer application for AT86RF215
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
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
 * Copyright (c) 2015-2018, Microchip Technology Inc All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef APP_FRAME_FORMAT_H
#define APP_FRAME_FORMAT_H

/* === Includes ============================================================= */

/* === Macros =============================================================== */

/* === Types ================================================================ */
COMPILER_PACK_SET(1)

/**
 * \brief Peer request frame structure used in Peer Search process
 *
 */
typedef struct {
	uint8_t op_mode;
	uint16_t nwk_addr;
	uint8_t config_mode : 1;
} peer_req_t;

/**
 * \brief Peer response frame structure used in Peer Search process
 *
 */
typedef struct {
	uint16_t nwk_addr;
} peer_rsp_t;

/**
 * \brief Peer confirm frame structure used in Peer Search process
 *
 */
typedef struct {
	uint16_t nwk_addr;
} peer_conf_t;

/**
 * \brief Data packet frame structure to send data in Range Measurement mode
 *
 */
typedef struct {
	uint32_t pkt_count;
} data_pkt_range_test_t;

/**
 * \brief Set paramter Request frame structure to set the the parameters like
 * channel on peer node
 *
 */
typedef struct {
	uint8_t param_type;
	uint16_t param_value;
} set_parm_req_t;

/**
 * \brief Result request frame structure to request the results of the PER test
 *
 */
typedef struct {
	uint8_t cmd;
} result_req_t;

/**
 * \brief Result response frame structure to send the results of the PER test
 *
 */
typedef struct {
	uint32_t num_of_frames_rx;
	uint32_t lqi_avrg_rx;
	uint32_t rssi_avrg_rx;
	uint32_t frames_with_wrong_crc;
} result_rsp_t;

/**
 * \brief Antenna Diversity status request frame structure to request the peer
 * node's antenna diversity settings
 *
 */
typedef struct {
	uint8_t status;
} div_stat_req_t;

/**
 * \brief Antenna Diversity status response frame structure to send the peer
 * node's antenna diversity settings
 *
 */
typedef struct {
	uint8_t status    : 1;
	uint8_t ant_sel   : 2;
} div_stat_rsp_t;

/**
 * \brief Antenna Diversity set request frame structure to change the peer
 * node's
 * antenna diversity settings
 *
 */
typedef struct {
	uint8_t status    : 1;
	uint8_t ant_sel   : 2;
} div_set_req_t;

/**
 * \brief Antenna Diversity set response frame structure to send the status of
 * div_set_req
 *
 */
typedef struct {
	uint8_t status;
} div_set_rsp_t;

/**
 * \brief CRC status request frame structure to request the peer
 * node's CRC settings
 *
 */
typedef struct {
	uint8_t status;
} crc_stat_req_t;

/**
 * \brief CRC status response frame structure to send the peer
 * node's CRC settings
 *
 */
typedef struct {
	uint8_t status;
} crc_stat_rsp_t;

/**
 * \brief CRC set request frame structure to change the peer node's
 * CRC settings
 *
 */
typedef struct {
	uint8_t status;
} crc_set_req_t;

/**
 * \brief CRC set response frame structure to send the status of
 * crc_set_req
 *
 */
typedef struct {
	uint8_t status;
} crc_set_rsp_t;

/**
 * \brief Peer information response frame structure to send the peer details
 *
 */
typedef struct {
	uint8_t status;
	uint8_t ic_type;
	char soc_mcu_name[16];
	char trx_name[16];
	char board_name[25];
	uint64_t mac_address;
	float fw_version;
	uint32_t feature_mask;
} peer_info_rsp_t;

typedef struct {
	uint32_t frame_count;
	int8_t ed;
	uint8_t lqi;
} range_tx_t;

/**
 * \brief Union of all request and and response structures
 *
 */
typedef union {
	peer_req_t peer_req_data;
	peer_rsp_t peer_rsp_data;
	peer_conf_t peer_conf_data;
	data_pkt_range_test_t data_payload;
	set_parm_req_t set_parm_req_data;
	result_rsp_t test_result_rsp_data;
	div_stat_rsp_t div_stat_rsp_data;
	div_set_req_t div_set_req_data;
	crc_stat_rsp_t crc_stat_rsp_data;
	crc_set_req_t crc_set_req_data;
	peer_info_rsp_t peer_info_rsp_data;
	range_tx_t range_tx_data;
	result_req_t result_req_data;
	crc_stat_req_t crc_stat_req_data;
	phy_t sun_page_data;
} general_pkt_t;

/**
 * \brief application payload frame stucture
 *
 */
typedef struct {
	uint8_t cmd_id;
	uint8_t seq_num;
	general_pkt_t payload;
} app_payload_t;

COMPILER_PACK_RESET()

/* === Externals ============================================================ */

/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APP_FRAME_FORMAT_H */
/* EOF */
