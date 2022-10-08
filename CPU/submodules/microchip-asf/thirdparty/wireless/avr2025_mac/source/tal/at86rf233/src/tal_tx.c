/**
 * @file tal_tx.c
 *
 * @brief This file handles the frame transmission within the TAL.
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
 */

/*
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === INCLUDES ============================================================ */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "pal.h"
#include "return_val.h"
#include "tal.h"
#include "ieee_const.h"
#include "tal_pib.h"
#include "tal_irq_handler.h"
#include "tal_constants.h"
#include "tal_tx.h"
#include "stack_config.h"
#include "bmm.h"
#include "qmm.h"
#include "tal_rx.h"
#include "tal_internal.h"
#include "at86rf233.h"
#ifdef BEACON_SUPPORT
#include "tal_slotted_csma.h"
#endif  /* BEACON_SUPPORT */
#include "mac_build_config.h"
#ifdef ENABLE_RTB
#include "rtb.h"
#endif  /* ENABLE_RTB */

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === GLOBALS ============================================================= */

static uint8_t tal_sw_retry_count;
static bool tal_sw_retry_no_csma_ca;

static trx_trac_status_t trx_trac_status;

/* === PROTOTYPES ========================================================== */

/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Requests to TAL to transmit frame
 *
 * This function is called by the MAC to deliver a frame to the TAL
 * to be transmitted by the transceiver.
 *
 * \param tx_frame Pointer to the frame_info_t structure updated by the MAC
 * layer
 * \param csma_mode Indicates mode of csma-ca to be performed for this frame
 * \param perform_frame_retry Indicates whether to retries are to be performed
 * for
 *                            this frame
 *
 * \return MAC_SUCCESS  if the TAL has accepted the data from the MAC for frame
 *                 transmission
 *         TAL_BUSY if the TAL is busy servicing the previous MAC request
 */
retval_t tal_tx_frame(frame_info_t *tx_frame, csma_mode_t csma_mode,
		bool perform_frame_retry)
{
	if (tal_state != TAL_IDLE) {
		return TAL_BUSY;
	}

	/*
	 * Store the pointer to the provided frame structure.
	 * This is needed for the callback function.
	 */
	mac_frame_ptr = tx_frame;

	/* Set pointer to actual mpdu to be downloaded to the transceiver. */
	tal_frame_to_tx = tx_frame->mpdu;
	last_frame_length = tal_frame_to_tx[0] - 1;

	/*
	 * In case the frame is too large, return immediately indicating
	 * invalid status.
	 */
	if (tal_frame_to_tx == NULL) {
		return MAC_INVALID_PARAMETER;
	}

#ifdef BEACON_SUPPORT
	/* Check if beacon mode is used */
	if (csma_mode == CSMA_SLOTTED) {
		if (!slotted_csma_start(perform_frame_retry)) {
			return MAC_CHANNEL_ACCESS_FAILURE;
		}
	} else {
#if (MAC_INDIRECT_DATA_FFD == 1)

		/*
		 * Check if frame is using indirect transmission, but do not use
		 * the
		 * indirect_in_transit flag. This flag is not set for null data
		 * frames.
		 */
		if ((tal_pib.BeaconOrder < NON_BEACON_NWK) &&
				(csma_mode == NO_CSMA_WITH_IFS) &&
				(perform_frame_retry == false)) {
			/*
			 * Check if indirect transmission can be completed
			 * before the next
			 * beacon transmission.
			 */
			uint32_t time_between_beacons_sym;
			uint32_t next_beacon_time_sym;
			uint32_t now_time_sym;
			uint32_t duration_before_beacon_sym;

			/* Calculate the entire transaction duration. Re-use
			 * function of slotted CSMA.
			 * The additional two backoff periods used for CCA are
			 * kept as a guard time.
			 */
			calculate_transaction_duration();

			/* Calculate the duration until the next beacon needs to
			 * be transmitted. */
			time_between_beacons_sym = TAL_GET_BEACON_INTERVAL_TIME(
					tal_pib.BeaconOrder);
			next_beacon_time_sym = tal_add_time_symbols(
					tal_pib.BeaconTxTime,
					time_between_beacons_sym);
			pal_get_current_time(&now_time_sym);
			now_time_sym = TAL_CONVERT_US_TO_SYMBOLS(now_time_sym);
			duration_before_beacon_sym = tal_sub_time_symbols(
					next_beacon_time_sym, now_time_sym);

			/* Check if transaction can be completed before next
			 * beacon transmission. */
			if ((now_time_sym >= next_beacon_time_sym) ||
					((transaction_duration_periods *
					aUnitBackoffPeriod) >
					duration_before_beacon_sym)) {
				/*
				 * Transaction will not be completed before next
				 * beacon transmission.
				 * Therefore the transmission is not executed.
				 */
				return MAC_CHANNEL_ACCESS_FAILURE;
			}
		}
#endif  /* #if (MAC_INDIRECT_DATA_FFD == 1) */
		send_frame(csma_mode, perform_frame_retry);
	}

#else   /* No BEACON_SUPPORT */
	send_frame(csma_mode, perform_frame_retry);
#endif  /* BEACON_SUPPORT / No BEACON_SUPPORT */

	return MAC_SUCCESS;
}

/*
 * \brief Implements the handling of the transmission end.
 *
 * This function handles the callback for the transmission end.
 */
void tx_done_handling(void)
{
	tal_state = TAL_IDLE;

#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP)
#if (DISABLE_TSTAMP_IRQ == 0)

	/*
	 * The Tx timestamp IRQ is enabled and provided via DIG2.
	 * The actual value is stored within trx_irq_timestamp_handler_cb();
	 * see file "tal_irq_handler.c".
	 * The value gets stored into the mac_frame_ptr structure during the
	 * tx end interrupt; see handle_tx_end_irq().
	 * So, here is nothing left to do.
	 */
#else   /* (DISABLE_TSTAMP_IRQ == 1) */

	/*
	 * The entire timestamp calculation is only required for beaconing
	 * networks
	 * or if timestamping is explicitly enabled.
	 */

	/* Calculate negative offset of timestamp */
	uint16_t offset;

	/* Calculate the tx time */
	offset
		= TAL_CONVERT_SYMBOLS_TO_US(
			(PHY_OVERHEAD + LENGTH_FIELD_LEN) * SYMBOLS_PER_OCTET)
			+ TAL_PSDU_US_PER_OCTET(*tal_frame_to_tx + FCS_LEN)
			+ TRX_IRQ_DELAY_US;
	if (mac_frame_ptr->mpdu[PL_POS_FCF_1] & FCF_ACK_REQUEST) {
		/* Tx timestamp needs to be reduced by ACK duration etc. */
		offset
			+= TAL_CONVERT_SYMBOLS_TO_US(
				(PHY_OVERHEAD +
				LENGTH_FIELD_LEN) * SYMBOLS_PER_OCTET) +
				TAL_PSDU_US_PER_OCTET(ACK_PAYLOAD_LEN +
				FCS_LEN);

#ifdef HIGH_DATA_RATE_SUPPORT
		if (tal_pib.CurrentPage == 0) {
			offset += TAL_CONVERT_SYMBOLS_TO_US(aTurnaroundTime);
		} else {
			offset += 32;
		}

#else
		offset += TAL_CONVERT_SYMBOLS_TO_US(aTurnaroundTime);
#endif  /* #ifdef HIGH_DATA_RATE_SUPPORT */
	}
	mac_frame_ptr->time_stamp -= offset;
#endif  /* #if (DISABLE_TSTAMP_IRQ == 0) */
#endif  /* #if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) */

	retval_t status;

	switch (trx_trac_status) {
	case TRAC_SUCCESS:
		status = MAC_SUCCESS;
		break;

	case TRAC_SUCCESS_DATA_PENDING:
		status = TAL_FRAME_PENDING;
		break;

	case TRAC_CHANNEL_ACCESS_FAILURE:
		status = MAC_CHANNEL_ACCESS_FAILURE;
		break;

	case TRAC_NO_ACK:
		status = MAC_NO_ACK;
		break;

	case TRAC_INVALID:
		status = FAILURE;
		break;

	default:
		Assert("Unexpected tal_tx_state" == 0);
		status = FAILURE;
		break;
	}

#ifdef ENABLE_RTB
	rtb_tx_frame_done_cb(status, mac_frame_ptr);
#else
	/* Regular handling without RTB */
	tal_tx_frame_done_cb(status, mac_frame_ptr);
#endif
} /* tx_done_handling() */

/*
 * \brief Sends frame
 *
 * \param use_csma Flag indicating if CSMA is requested
 * \param tx_retries Flag indicating if transmission retries are requested
 *                   by the MAC layer
 */
void send_frame(csma_mode_t csma_mode, bool tx_retries)
{
	tal_trx_status_t trx_status;

	/* Configure tx according to tx_retries */
	if (tx_retries) {
		trx_bit_write(SR_MAX_FRAME_RETRIES,
				tal_pib.MaxFrameRetries);
	} else {
		trx_bit_write(SR_MAX_FRAME_RETRIES, 0);
	}

	/* Configure tx according to csma usage */
	if ((csma_mode == NO_CSMA_NO_IFS) || (csma_mode == NO_CSMA_WITH_IFS)) {
		trx_bit_write(SR_MAX_CSMA_RETRIES, 7); /* immediate
		                                        * transmission */
		if (tx_retries) {
			tal_sw_retry_count = tal_pib.MaxFrameRetries;
			tal_sw_retry_no_csma_ca = true;
		}
	} else {
		trx_bit_write(SR_MAX_CSMA_RETRIES, tal_pib.MaxCSMABackoffs);
	}

	do {
		trx_status = set_trx_state(CMD_TX_ARET_ON);
	} while (trx_status != TX_ARET_ON);

	pal_trx_irq_dis();

	/* Handle interframe spacing */
	if (csma_mode == NO_CSMA_WITH_IFS) {
		if (last_frame_length > aMaxSIFSFrameSize) {
			pal_timer_delay(TAL_CONVERT_SYMBOLS_TO_US(
					macMinLIFSPeriod_def)
					- TRX_IRQ_DELAY_US -
					PRE_TX_DURATION_US);
			last_frame_length = 0;
		} else if (last_frame_length > 0) {
			pal_timer_delay(TAL_CONVERT_SYMBOLS_TO_US(
					macMinSIFSPeriod_def)
					- TRX_IRQ_DELAY_US -
					PRE_TX_DURATION_US);
			last_frame_length = 0;
		}
	}

	/* Toggle the SLP_TR pin triggering transmission. */
	TRX_SLP_TR_HIGH();
	PAL_WAIT_65_NS();
	TRX_SLP_TR_LOW();

	/*
	 * Send the frame to the transceiver.
	 * Note: The PhyHeader is the first byte of the frame to
	 * be sent to the transceiver and this contains the frame
	 * length.
	 * The actual length of the frame to be downloaded
	 * (parameter two of trx_frame_write)
	 * is
	 * 1 octet frame length octet
	 * + n octets frame (i.e. value of frame_tx[0])
	 * - 2 octets FCS
	 */
	trx_frame_write(tal_frame_to_tx, tal_frame_to_tx[0] - 1);

	tal_state = TAL_TX_AUTO;

#ifndef NON_BLOCKING_SPI
	pal_trx_irq_en();
#endif
}

/*
 * \brief Handles interrupts issued due to end of transmission
 *
 * \param underrun_occured  true if under-run has occurred
 */
void handle_tx_end_irq(bool underrun_occured)
{
#if ((MAC_START_REQUEST_CONFIRM == 1) && (defined BEACON_SUPPORT))
	if (tal_beacon_transmission) {
		tal_beacon_transmission = false;

		if (tal_csma_state == BACKOFF_WAITING_FOR_BEACON) {
			/* Slotted CSMA has been waiting for a beacon, now it
			 * can continue. */
			tal_csma_state = CSMA_HANDLE_BEACON;
		}
	} else
#endif /* ((MAC_START_REQUEST_CONFIRM == 1) && (defined BEACON_SUPPORT)) */
	{
#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP)

		/*
		 * Store tx timestamp to frame_info_t structure.
		 * The timestamping is only required for beaconing networks
		 * or if timestamping is explicitly enabled.
		 */
#if (DISABLE_TSTAMP_IRQ == 0)

		/*
		 * The Tx timestamp is stored during the timestamp interrupt
		 * at DIG2.
		 */
		mac_frame_ptr->time_stamp = tal_timestamp;
#else
		{
			uint32_t time_stamp_temp = 0;
			pal_trx_read_timestamp(&time_stamp_temp);
			mac_frame_ptr->time_stamp = time_stamp_temp;
		}
#endif
#endif  /* #if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) */

		/* Read trac status before enabling RX_AACK_ON. */
		if (underrun_occured) {
			trx_trac_status = TRAC_INVALID;
		} else {
			trx_trac_status = /*(trx_trac_status_t)*/ trx_bit_read(
					SR_TRAC_STATUS);
		}

#ifdef BEACON_SUPPORT
		if (tal_csma_state == FRAME_SENDING) { /* Transmission was
			                                * issued by slotted CSMA
			                                **/
			PIN_TX_END();
			tal_state = TAL_SLOTTED_CSMA;

			/* Map status message of transceiver to TAL constants.
			**/
			switch (trx_trac_status) {
			case TRAC_SUCCESS_DATA_PENDING:
				PIN_ACK_OK_START();
				tal_csma_state = TX_DONE_FRAME_PENDING;
				break;

			case TRAC_SUCCESS:
				PIN_ACK_OK_START();
				tal_csma_state = TX_DONE_SUCCESS;
				break;

			case TRAC_CHANNEL_ACCESS_FAILURE:
				PIN_NO_ACK_START();
				tal_csma_state = CSMA_ACCESS_FAILURE;
				break;

			case TRAC_NO_ACK:
				PIN_NO_ACK_START();
				tal_csma_state = TX_DONE_NO_ACK;
				break;

			case TRAC_INVALID: /* Handle this in the same way as
				            * default. */
			default:
				Assert("not handled trac status" == 0);
				tal_csma_state = CSMA_ACCESS_FAILURE;
				break;
			}
			PIN_ACK_OK_END();
			PIN_ACK_WAITING_END();
		} else
#endif  /* BEACON_SUPPORT */
		/* Trx has handled the entire transmission incl. CSMA */
		{
			if (tal_sw_retry_no_csma_ca && tal_sw_retry_count &&
					TRAC_NO_ACK == trx_trac_status) {
				tal_trx_status_t trx_status;
				do {
					trx_status = set_trx_state(
							CMD_TX_ARET_ON);
				} while (trx_status != TX_ARET_ON);

				/* Toggle the SLP_TR pin triggering
				 * transmission. */
				TRX_SLP_TR_HIGH();
				PAL_WAIT_65_NS();
				TRX_SLP_TR_LOW();
				if (--tal_sw_retry_count == 0) {
					tal_sw_retry_no_csma_ca = false;
				}
			} else {
				tal_state = TAL_TX_DONE; /* Further handling is
				                          * done by
				                          * tx_done_handling()
				                          **/
			}
		}
	}

	/*
	 * After transmission has finished, switch receiver on again.
	 * Check if receive buffer is available.
	 */
	if (NULL == tal_rx_buffer) {
		set_trx_state(CMD_PLL_ON);
		tal_rx_on_required = true;
	} else {
		set_trx_state(CMD_RX_AACK_ON);
	}
}

/*
 * \brief Beacon frame transmission
 */
#if ((MAC_START_REQUEST_CONFIRM == 1) && (defined BEACON_SUPPORT))
void tal_tx_beacon(frame_info_t *tx_frame)
{
	tal_trx_status_t trx_status;

	/* Set pointer to actual mpdu to be downloaded to the transceiver. */
	uint8_t *tal_beacon_to_tx = tx_frame->mpdu;

	/* Avoid that the beacon is transmitted while other transmision is
	 * on-going. */
	if (tal_state == TAL_TX_AUTO) {
		Assert(
				"trying to transmit beacon while ongoing transmission" ==
				0);
		return;
	}

	/* Send the pre-created beacon frame to the transceiver. */
	do {
		trx_status = set_trx_state(CMD_PLL_ON);
#if (_DEBUG_ > 1)
		if (trx_status != PLL_ON) {
			Assert("PLL_ON failed for beacon transmission" == 0);
		}
#endif
	} while (trx_status != PLL_ON);

	/* \TODO wait for talbeaconTxTime */

	pal_trx_irq_dis();

	/* Toggle the SLP_TR pin triggering transmission. */
	TRX_SLP_TR_HIGH();
	PAL_WAIT_65_NS();
	TRX_SLP_TR_LOW();

	/*
	 * Send the frame to the transceiver.
	 * Note: The PhyHeader is the first byte of the frame to
	 * be sent to the transceiver and this contains the frame
	 * length.
	 * The actual length of the frame to be downloaded
	 * (parameter two of trx_frame_write)
	 * is
	 * 1 octet frame length octet
	 * + n octets frame (i.e. value of frame_tx[0])
	 * - 2 octets FCS
	 */
	trx_frame_write(tal_beacon_to_tx, tal_beacon_to_tx[0] - 1);

	tal_beacon_transmission = true;

#ifndef NON_BLOCKING_SPI
	pal_trx_irq_en();
#endif
}

#endif /* ((MAC_START_REQUEST_CONFIRM == 1) && (defined BEACON_SUPPORT)) */

/* EOF */
