/**
 * @file tal_pwr_mgmt.c
 *
 * @brief This file implements TAL power management functionality of
 *        the transceiver.
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

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "pal.h"
#include "return_val.h"
#include "ieee_const.h"
#include "tal.h"
#include "tal_config.h"
#include "tal_internal.h"
#include "tal_pib.h"

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === GLOBALS ============================================================= */

/* === PROTOTYPES ========================================================== */

/* === IMPLEMENTATION ====================================================== */

/**
 * @brief Sets the transceiver to SLEEP
 *
 * This function sets the entire device to state DEEP_SLEEP.
 *
 * @param trx_id Transceiver identifier
 * @param mode Defines sleep mode of transceiver (Not used for 215 trx)
 *
 * @return
 *      - @ref TAL_BUSY - The transceiver is busy in TX or RX
 *      - @ref MAC_SUCCESS - The transceiver is put to sleep
 *      - @ref TAL_TRX_ASLEEP - The transceiver is already asleep
 * @ingroup apiTalApi
 */
retval_t tal_trx_sleep(trx_id_t trx_id)
{
	Assert((trx_id >= 0) && (trx_id < NUM_TRX));

	if (tal_state[trx_id] == TAL_SLEEP) {
		return TAL_TRX_ASLEEP;
	}

	/* Device can be put to sleep only when the TAL is in IDLE state. */
	if (tal_state[trx_id] != TAL_IDLE) {
		return TAL_BUSY;
	}

	{
		uint16_t reg_offset = RF_BASE_ADDR_OFFSET * trx_id;
		trx_reg_write( reg_offset + RG_RF09_CMD, RF_TRXOFF);
#ifdef IQ_RADIO
		trx_reg_write(RF215_RF, reg_offset + RG_RF09_CMD, RF_TRXOFF);
#endif
	}
	trx_state[trx_id] = RF_TRXOFF;
	tal_state[trx_id] = TAL_SLEEP;

	/* Enter DEEP_SLEEP if both transceiver suppose to enter SLEEP */
#if ((defined RF215v1) || (defined RF215v3))
	if ((tal_state[RF09] == TAL_SLEEP) && (tal_state[RF24] == TAL_SLEEP)) {
		for (trx_id_t i = (trx_id_t)0; i < (trx_id_t)NUM_TRX; i++) {
			uint16_t reg_offset = RF_BASE_ADDR_OFFSET * i;
			trx_reg_write( reg_offset + RG_RF09_CMD, RF_SLEEP);
#ifdef IQ_RADIO
			trx_reg_write(RF215_RF, reg_offset + RG_RF09_CMD,
					RF_SLEEP);
#endif
			TAL_BB_IRQ_CLR_ALL(i);
			TAL_RF_IRQ_CLR_ALL(i);
			trx_state[i] = RF_SLEEP;
		}
	}

#endif

	/*
	 * Free TAL Rx buffer. During sleep no buffer is required.
	 * With tal_trx_wakeup() a new buffer gets allocated.
	 */
	bmm_buffer_free(tal_rx_buffer[trx_id]);
	tal_rx_buffer[trx_id] = NULL;
	tal_buf_shortage[trx_id] = false;

#ifdef ENABLE_FTN_PLL_CALIBRATION
	stop_ftn_timer(trx_id);
#endif  /* ENABLE_FTN_PLL_CALIBRATION */

	/* Keep compiler happy */

	return MAC_SUCCESS;
}

/**
 * @brief Wakes up the transceiver from SLEEP
 *
 * This function awakes the transceiver from state SLEEP.
 *
 * @param trx_id Transceiver identifier
 *
 * @return
 *      - @ref TAL_TRX_AWAKE - The transceiver is already awake
 *      - @ref MAC_SUCCESS - The transceiver is woken up from sleep
 *      - @ref FAILURE - The transceiver did not wake-up from sleep
 * @ingroup apiTalApi
 */
retval_t tal_trx_wakeup(trx_id_t trx_id)
{
	Assert((trx_id >= 0) && (trx_id < NUM_TRX));
	if (tal_state[trx_id] != TAL_SLEEP) {
		return TAL_TRX_AWAKE;
	}

#if (defined RF215v1) || (defined RF215v3)
	if ((tal_state[RF09] == TAL_SLEEP) && (tal_state[RF24] == TAL_SLEEP))
#endif
	{
		tal_state[trx_id] = TAL_WAKING_UP; /* Intermediate state to
		                                    *handle TRX IRQ */
		/* Write command to wake device up */
		uint16_t reg_offset = RF_BASE_ADDR_OFFSET * trx_id;
		trx_reg_write( reg_offset + RG_RF09_CMD, RF_TRXOFF);
#ifdef IQ_RADIO
		trx_reg_write(RF215_RF, reg_offset + RG_RF09_CMD, RF_TRXOFF);
#endif
		/* Wait for transceiver wakeup */
		uint32_t start_time;
		uint32_t current_time;
		pal_get_current_time(&start_time);
		pal_get_current_time(&current_time);
		while (1) {
			if (TAL_RF_IS_IRQ_SET(trx_id, RF_IRQ_WAKEUP)) {
#if (defined RF215v1) || (defined RF215v3)
				for (trx_id_t i = (trx_id_t)0;
						i < (trx_id_t)NUM_TRX; i++) {
					TAL_RF_IRQ_CLR(i, RF_IRQ_WAKEUP);
					trx_state[i] = RF_TRXOFF;
				}
#endif
				break;
			}

			pal_get_current_time(&current_time);
			/* @ToDo: Use no magic number for "1000" */
			if (pal_sub_time_us(current_time, start_time) > 1000) {
				return FAILURE;
			}
		}
	}

	tal_state[trx_id] = TAL_IDLE;

	/*
	 * Allocate a new buffer
	 * The previous buffer got freed with entering sleep mode.
	 */
	tal_rx_buffer[trx_id] = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

	/* Fill trx_id in new buffer */
	if (tal_rx_buffer[trx_id] != NULL) {
		frame_info_t *frm_info = (frame_info_t *)BMM_BUFFER_POINTER(
				tal_rx_buffer[trx_id]);
		frm_info->trx_id = trx_id;
	}

	trx_config(trx_id); /* see tal_init.c */
	write_all_tal_pib_to_trx(trx_id); /* see 'tal_pib.c' */
	config_phy(trx_id);
	return MAC_SUCCESS;
}

/* EOF */
