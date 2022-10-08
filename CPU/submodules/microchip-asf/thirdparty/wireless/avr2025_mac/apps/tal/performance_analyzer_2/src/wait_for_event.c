/**
 * \file wait_for_event.c
 *
 * \brief Event handling functionalities to start Peer Search - Performance
 * Analyzer application for AT86RF215
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
/* === INCLUDES ============================================================ */

#include "app_init.h"
#include "ieee_const.h"
#include "app_frame_format.h"
#include "sio2host.h"
#include "sio2ncp.h"
#include "user_interface.h"
#include "app_peer_search.h"
/* === TYPES =============================================================== */

/* === MACROS ============================================================== */
/* ED threshold to accept a frame in the configuration mode  */
#define CONFIG_ED_THRESHOLD_RF09             (0x3a)

#define CONFIG_ED_THRESHOLD_RF24             (0x1f)
/* === PROTOTYPES ========================================================== */
static void configure_pibs(trx_id_t trx);

/* === GLOBALS ============================================================= */

/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Application task when the node is in the WAIT_FOR_EVENT state
 *
 * \param arg arguments for WAIT_FOR_EVENT state
 */
void wait_for_event_init(trx_id_t trx, void *arg)
{
	/* Configure the TAL PIBs; e.g. set short address */
	configure_pibs(trx);

	/* Leave the radio in RX mode in default channel */
	if (trx == RF09) {
		tal_rx_enable(RF09, PHY_RX_ON);
	} else {
		tal_rx_enable(RF24, PHY_RX_ON);
	}

	/* indicating application is started */
	app_led_event(LED_EVENT_POWER_ON);

	/* Keep compiler happy */
	arg = arg;
}

/*
 * \brief Application task handling user events like key press or
 * character on UART
 *
 * This function
 * - Implements the event handling in WAIT_FOR_EVENT state.
 */
void wait_for_event_task(trx_id_t trx)
{
	return;

	uint8_t key_press;

	/* Check for any key press */
	key_press = app_debounce_button();

	if (key_press != 0) {
		/* check->print event to be changed to sio */
		print_event(trx, PRINT_KEY_PRESS_PEER_SEARCH_INITIATOR);

		/* key press detected - so change to state PEER_SEARCH_RANGE_TX
		**/
		set_main_state(trx, PEER_SEARCH_RANGE_TX, NULL);
	}
}

/*
 * \brief Callback that is called if data has been received by trx
 * in WAIT_FOR_EVENT state. This allow the node to participate in
 * Peer Search process as receptor
 *
 * \param frame Pointer to received frame
 */
void wait_for_event_rx_cb(trx_id_t trx, frame_info_t *mac_frame_info)
{
	app_payload_t *msg;
	uint8_t ed_threshold;

	peer_search_receptor_arg_t peer_info;
	uint8_t expected_frame_size = FRAME_OVERHEAD_SRC_IEEE_ADDR
			+ ((sizeof(app_payload_t)
			- sizeof(general_pkt_t))
			+ sizeof(peer_req_t));

	if (trx == RF24) {
		ed_threshold = CONFIG_ED_THRESHOLD_RF24;
	} else {
		ed_threshold = CONFIG_ED_THRESHOLD_RF09;
	}

	/* Frame received on air: Processing the same */
	if ((mac_frame_info->len_no_crc) == expected_frame_size) {
		msg
			= (app_payload_t *)(mac_frame_info->mpdu +
				FRAME_OVERHEAD_SRC_IEEE_ADDR);

		/* Is this a peer request cmd */
		if ((msg->cmd_id) == PEER_REQUEST) {
			uint8_t frame_len = mac_frame_info->len_no_crc;
			uint8_t ed_val
				= mac_frame_info->mpdu[frame_len +
					LQI_LEN + ED_VAL_LEN];

			/* Check the threshold if the configuration mode is
			 * enabled, not otherwise */
			if (((msg->payload.peer_req_data.config_mode == true) &&
					(ed_val > ed_threshold)) ||
					(msg->payload.peer_req_data.config_mode
					== false)) {
				peer_info.my_short_addr
					= (msg->payload.peer_req_data.
						nwk_addr);
				memcpy(&(peer_info.peer_ieee_addr),
						(mac_frame_info->mpdu +
						OFFSET_FOR_SRC_IEEE_ADDR),
						sizeof(peer_info.peer_ieee_addr));

				switch (msg->payload.peer_req_data.op_mode) {
				case RANGE_MEASURE_MODE:

					/*
					 * Frame found to be peer search for
					 * range test
					 * so change to state
					 * PEER_SEARCH_RANGE_RX
					 */
					set_main_state(trx,
							PEER_SEARCH_RANGE_RX,
							&peer_info);
					print_event(trx,
							PRINT_KEY_PRESS_PEER_SEARCH_RECEPTOR);
					break;

				case PER_TEST_MODE:

					/*
					 * Frame found to be peer search for
					 * range test
					 * so change to state PEER_SEARCH_PER_RX
					 */
					set_main_state(trx, PEER_SEARCH_PER_RX,
							&peer_info);
					print_event(trx,
							PRINT_UART_CHAR_PEER_SEARCH_RECEPTOR);
					break;

				default:

					/* The node has got a wrong frame: No
					 * change of mode */
					print_event(trx,
							PRINT_PEER_SEARCH_RECEPTOR_BAD_FRAME);
					break;
				}
			}
		}
	}
}

/**
 * \brief Configure the TAL PIB's relevant to the Performance analyzer
 * application
 * \ingroup group_app_init
 */
static void configure_pibs(trx_id_t trx)
{
	uint16_t temp_word;

	/* Set Default address. */
	temp_word = CCPU_ENDIAN_TO_LE16(DEFAULT_ADDR);
	tal_pib_set(trx, macShortAddress, (pib_value_t *)&temp_word);

	/* Set PAN ID. */
	temp_word = CCPU_ENDIAN_TO_LE16(SRC_PAN_ID);
	tal_pib_set(trx, macPANId, (pib_value_t *)&temp_word);

	/* Set channel. */
	if (trx == RF09) {
		temp_word = CCPU_ENDIAN_TO_LE16(DEFAULT_CHANNEL_RF09);
	} else {
		temp_word = CCPU_ENDIAN_TO_LE16(DEFAULT_CHANNEL_RF24);
	}

	tal_pib_set(trx, phyCurrentChannel, (pib_value_t *)&temp_word);

	/* Set IEEE address - To make sure that trx registers written properly
	**/
	tal_pib_set(trx, macIeeeAddress,
			(pib_value_t *)&tal_pib[trx].IeeeAddress);
}
