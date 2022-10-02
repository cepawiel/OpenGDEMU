/**
 * @file tal_internal.h
 *
 * @brief This header file contains types and variable definition that are used
 * within the TAL only.
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

/* Prevent double inclusion */
#ifndef TAL_INTERNAL_H
#define TAL_INTERNAL_H

/* === INCLUDES ============================================================ */

#include "bmm.h"
#include "qmm.h"
#if (defined MAC_SECURITY_ZIP || defined MAC_SECURITY_2006)
#include "tal.h"
#endif
#ifdef BEACON_SUPPORT
#include "tal_slotted_csma.h"
#endif  /* BEACON_SUPPORT */
#ifdef ENABLE_DEBUG_PINS
#include "pal_config.h"
#endif
#include "mac_build_config.h"

/**
 * \ingroup group_tal
 * \defgroup group_tal_212b AT86RF212B Transceiver Abstraction Layer
 * The AT86RF212B is a  low-power ,low voltage 700/800/900 MHz radio transceiver
 * designed for industrial
 * and consumer ZigBee/IEEE 802.15.4, 6LoWPAN, RF4CE and high data rate sub 1GHz
 * ISM band applications.
 * The Transceiver Abstraction Layer (TAL) implements the transceiver specific
 * functionalities and
 * provides interfaces to the upper layers (like IEEE 802.15.4 MAC )and  uses
 * the services of PAL.
 */

/**
 * \ingroup group_tal_212b
 * \defgroup group_tal_state_machine_212b  TAL State Machine
 * The different operating states of the Transceiver are controlled by the TAL
 * state machine.
 *
 */

/**
 * \ingroup group_tal_212b
 * \defgroup group_tal_init_212b  TAL Initialization and reset
 * Performs initialization and reset functionalities of the transceiver
 *
 */

/**
 * \ingroup group_tal_212b
 * \defgroup group_tal_ed_212b   TAL Energy Detection
 * Performs the ED scan functionalities.
 *
 */

/**
 * \ingroup group_tal_212b
 * \defgroup group_tal_irq_212b  Transceiver Interrupt Handling
 * Handles Transceiver related Interrupts.
 *
 */

/**
 * \ingroup group_tal_212b
 * \defgroup group_tal_pib_212b   TAL PIB Storage
 * The PIB(Pan Information Base) attributes related to the TAL are Stored and
 * handled  by the TAL PIB storage.
 *
 */

/**
 * \ingroup group_tal_212b
 * \defgroup group_tal_tx_212   TAL Frame Transmission Unit
 * The Frame Transmission Unit generates and transmits the frames using PAL .
 *
 */

/**
 * \ingroup group_tal_tx_212b
 * \defgroup group_tal_tx_csma_212b   TAL CSMA/CA Module
 * Performs channel access mechanism for frame transmission
 * For Detailed information refer  CSMA-CA algorithm section of IEEE Std
 * 802.15.4-2006
 *
 */

/**
 * \ingroup group_tal_212b
 * \defgroup group_tal_rx_212b   TAL Frame Reception Unit
 * The Frame Reception Unit reads/uploads the incoming frames .
 *
 */

/* === TYPES =============================================================== */

/** TAL states */
#if ((defined BEACON_SUPPORT) && (MAC_SCAN_ED_REQUEST_CONFIRM == 1))
typedef enum tal_state_tag {
	TAL_IDLE           = 0,
	TAL_TX_AUTO        = 1,
	TAL_TX_DONE        = 2,
	TAL_SLOTTED_CSMA   = 3,
	TAL_ED_RUNNING     = 4,
	TAL_ED_DONE        = 5
#ifdef SW_CONTROLLED_CSMA
	,
	TAL_BACKOFF,
	TAL_CCA,
	TAL_CSMA_CONTINUE,
	TAL_CCA_DONE
#endif
} SHORTENUM tal_state_t;
#endif

#if ((!defined BEACON_SUPPORT) && (MAC_SCAN_ED_REQUEST_CONFIRM == 1))
typedef enum tal_state_tag {
	TAL_IDLE           = 0,
	TAL_TX_AUTO        = 1,
	TAL_TX_DONE        = 2,
	TAL_ED_RUNNING     = 4,
	TAL_ED_DONE        = 5
#ifdef SW_CONTROLLED_CSMA
	,
	TAL_BACKOFF,
	TAL_CCA,
	TAL_CSMA_CONTINUE,
	TAL_CCA_DONE
#endif
} SHORTENUM tal_state_t;
#endif

#if ((defined BEACON_SUPPORT) && (MAC_SCAN_ED_REQUEST_CONFIRM == 0))
typedef enum tal_state_tag {
	TAL_IDLE           = 0,
	TAL_TX_AUTO        = 1,
	TAL_TX_DONE        = 2,
	TAL_SLOTTED_CSMA   = 3
#ifdef SW_CONTROLLED_CSMA
	,
	TAL_BACKOFF,
	TAL_CCA,
	TAL_CSMA_CONTINUE,
	TAL_CCA_DONE
#endif
} SHORTENUM tal_state_t;
#endif

#if ((!defined BEACON_SUPPORT) && (MAC_SCAN_ED_REQUEST_CONFIRM == 0))
typedef enum tal_state_tag {
	TAL_IDLE           = 0,
	TAL_TX_AUTO        = 1,
	TAL_TX_DONE        = 2
#ifdef SW_CONTROLLED_CSMA
	,
	TAL_BACKOFF,
	TAL_CCA,
	TAL_CSMA_CONTINUE,
	TAL_CCA_DONE
#endif
} SHORTENUM tal_state_t;
#endif

/* === EXTERNALS =========================================================== */

/* Global TAL variables */
extern tal_state_t tal_state;
extern tal_trx_status_t tal_trx_status;
extern frame_info_t *mac_frame_ptr;
extern queue_t tal_incoming_frame_queue;
extern uint8_t *tal_frame_to_tx;
extern buffer_t *tal_rx_buffer;
extern bool tal_rx_on_required;
extern uint8_t last_frame_length;
extern volatile bool tal_awake_end_flag;

#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP)
extern uint32_t tal_rx_timestamp;
#endif  /* #if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) */

#ifdef BEACON_SUPPORT
extern csma_state_t tal_csma_state;
#if (MAC_START_REQUEST_CONFIRM == 1)
extern uint8_t transaction_duration_periods;
#endif
#endif  /* BEACON_SUPPORT */

#if ((MAC_START_REQUEST_CONFIRM == 1) && (defined BEACON_SUPPORT))
extern bool tal_beacon_transmission;
#endif /* ((MAC_START_REQUEST_CONFIRM == 1) && (defined BEACON_SUPPORT)) */

/* === MACROS ============================================================== */

/**
 * Conversion of number of PSDU octets to duration in microseconds
 */
#ifdef HIGH_DATA_RATE_SUPPORT
#define TAL_PSDU_US_PER_OCTET(octets) \
	( \
		tal_pib.CurrentPage == 0 ? \
		(tal_pib.CurrentChannel == 0 ? (uint16_t)(octets) * \
		400 : (uint16_t)(octets) * 200) : \
		( \
			tal_pib.CurrentPage == 2 ? \
			(tal_pib.CurrentChannel == 0 ? (uint16_t)(octets) * \
			80 : (uint16_t)(octets) * 32) :	\
			( \
				tal_pib.CurrentPage == 5 ? \
				((uint16_t)(octets) * 32) : \
				( \
					tal_pib.CurrentPage == 16 ? \
					(tal_pib.CurrentChannel == \
					0 ? (uint16_t)(octets) * \
					40 : (uint16_t)(octets) * 16) :	\
					( \
                                                /* tal_pib.CurrentPage == 17 ?
                                                **/ \
						(tal_pib.CurrentChannel == \
						0 ? (uint16_t)(octets) * \
						20 : (uint16_t)(octets) * 8) \
					) \
				) \
			) \
		) \
	)
#else   /* #ifdef not HIGH_DATA_RATE_SUPPORT */
#define TAL_PSDU_US_PER_OCTET(octets) \
	( \
		tal_pib.CurrentPage == 0 ? \
		(tal_pib.CurrentChannel == 0 ? (uint16_t)(octets) * \
		400 : (uint16_t)(octets) * 200) : \
		( \
			tal_pib.CurrentPage == 2 ? \
			(tal_pib.CurrentChannel == 0 ? (uint16_t)(octets) * \
			80 : (uint16_t)(octets) * 32) :	\
			( \
                                /* tal_pib.CurrentPage == 5 ? */ \
				(uint16_t)(octets) * 32	\
			) \
		) \
	)
#endif

/*
 * Debug synonyms
 * These debug defines are only applicable if
 * the build switch "-DENABLE_DEBUG_PINS" is set.
 * The implementation of the debug pins is located in
 * pal_config.h
 */
#ifdef ENABLE_DEBUG_PINS
#define PIN_BEACON_START()              TST_PIN_0_HIGH()
#define PIN_BEACON_END()                TST_PIN_0_LOW()
#define PIN_CSMA_START()                TST_PIN_1_HIGH()
#define PIN_CSMA_END()                  TST_PIN_1_LOW()
#define PIN_BACKOFF_START()             TST_PIN_2_HIGH()
#define PIN_BACKOFF_END()               TST_PIN_2_LOW()
#define PIN_CCA_START()                 TST_PIN_3_HIGH()
#define PIN_CCA_END()                   TST_PIN_3_LOW()
#define PIN_TX_START()                  TST_PIN_4_HIGH()
#define PIN_TX_END()                    TST_PIN_4_LOW()
#define PIN_ACK_WAITING_START()         TST_PIN_5_HIGH()
#define PIN_ACK_WAITING_END()           TST_PIN_5_LOW()
#define PIN_WAITING_FOR_BEACON_START()  TST_PIN_6_HIGH()
#define PIN_WAITING_FOR_BEACON_END()    TST_PIN_6_LOW()
#define PIN_BEACON_LOSS_TIMER_START()
#define PIN_BEACON_LOSS_TIMER_END()
#define PIN_ACK_OK_START()              TST_PIN_7_HIGH()
#define PIN_ACK_OK_END()                TST_PIN_7_LOW()
#define PIN_NO_ACK_START()              TST_PIN_8_HIGH()
#define PIN_NO_ACK_END()                TST_PIN_8_LOW()
#else
#define PIN_BEACON_START()
#define PIN_BEACON_END()
#define PIN_CSMA_START()
#define PIN_CSMA_END()
#define PIN_BACKOFF_START()
#define PIN_BACKOFF_END()
#define PIN_CCA_START()
#define PIN_CCA_END()
#define PIN_TX_START()
#define PIN_TX_END()
#define PIN_ACK_WAITING_START()
#define PIN_ACK_WAITING_END()
#define PIN_WAITING_FOR_BEACON_START()
#define PIN_WAITING_FOR_BEACON_END()
#define PIN_BEACON_LOSS_TIMER_START()
#define PIN_BEACON_LOSS_TIMER_END()
#define PIN_ACK_OK_START()
#define PIN_ACK_OK_END()
#define PIN_NO_ACK_START()
#define PIN_NO_ACK_END()
#endif

#if ((defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP)) && \
	(DISABLE_TSTAMP_IRQ == 1)
#define TRX_IRQ_DEFAULT     TRX_IRQ_3_TRX_END | TRX_IRQ_2_RX_START
#else
#define TRX_IRQ_DEFAULT     TRX_IRQ_3_TRX_END
#endif

/* === PROTOTYPES ========================================================== */

/*
 * Prototypes from tal.c
 */

/**
 * \brief Sets transceiver state
 *
 * \param trx_cmd needs to be one of the trx commands
 *
 * \return current trx state
 * \ingroup group_tal_state_machine_212b
 */
tal_trx_status_t set_trx_state(trx_cmd_t trx_cmd);

#ifdef ENABLE_FTN_PLL_CALIBRATION

/**
 * \brief PLL calibration and filter tuning timer callback
 *
 * \param parameter Unused callback parameter
 */
void calibration_timer_handler_cb(void *parameter);

#endif  /* ENABLE_FTN_PLL_CALIBRATION */

/*
 * Prototypes from tal_ed.c
 */
#if (MAC_SCAN_ED_REQUEST_CONFIRM == 1)

/**
 * \brief Scan done
 *
 * This function updates the max_ed_level and invokes the callback function
 * tal_ed_end_cb().
 *
 * \param parameter unused callback parameter
 * \ingroup group_tal_ed_212b
 */
void ed_scan_done(void);

#endif /* (MAC_SCAN_ED_REQUEST_CONFIRM == 1) */

#endif /* TAL_INTERNAL_H */
