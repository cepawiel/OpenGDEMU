/**
 * \file nwkTx.c
 *
 * \brief Transmit routines implementation
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
 *
 *
 */

/*
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/*- Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "phy.h"
#include "sysConfig.h"
#include "sysTimer.h"
#include "nwk.h"
#include "nwkTx.h"
#include "nwkFrame.h"
#include "nwkRoute.h"
#include "nwkCommand.h"
#include "nwkSecurity.h"

/*- Definitions ------------------------------------------------------------*/
#define NWK_TX_ACK_WAIT_TIMER_INTERVAL    50 /* ms */
#define NWK_TX_DELAY_TIMER_INTERVAL       10 /* ms */
#define NWK_TX_DELAY_JITTER_MASK          0x07

/*- Types ------------------------------------------------------------------*/
enum {
	NWK_TX_STATE_ENCRYPT    = 0x10,
	NWK_TX_STATE_WAIT_DELAY = 0x11,
	NWK_TX_STATE_DELAY      = 0x12,
	NWK_TX_STATE_SEND       = 0x13,
	NWK_TX_STATE_WAIT_CONF  = 0x14,
	NWK_TX_STATE_SENT       = 0x15,
	NWK_TX_STATE_WAIT_ACK   = 0x16,
	NWK_TX_STATE_CONFIRM    = 0x17,
};

/*- Prototypes -------------------------------------------------------------*/
static void nwkTxAckWaitTimerHandler(SYS_Timer_t *timer);
static void nwkTxDelayTimerHandler(SYS_Timer_t *timer);

/*- Variables --------------------------------------------------------------*/
static NwkFrame_t *nwkTxPhyActiveFrame;
static SYS_Timer_t nwkTxAckWaitTimer;
static SYS_Timer_t nwkTxDelayTimer;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*  @brief Initializes the Tx module
*****************************************************************************/
void nwkTxInit(void)
{
	nwkTxPhyActiveFrame = NULL;

	nwkTxAckWaitTimer.interval = NWK_TX_ACK_WAIT_TIMER_INTERVAL;
	nwkTxAckWaitTimer.mode = SYS_TIMER_INTERVAL_MODE;
	nwkTxAckWaitTimer.handler = nwkTxAckWaitTimerHandler;

	nwkTxDelayTimer.interval = NWK_TX_DELAY_TIMER_INTERVAL;
	nwkTxDelayTimer.mode = SYS_TIMER_INTERVAL_MODE;
	nwkTxDelayTimer.handler = nwkTxDelayTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/
void nwkTxFrame(NwkFrame_t *frame)
{
	NwkFrameHeader_t *header = &frame->header;

	if (frame->tx.control & NWK_TX_CONTROL_ROUTING) {
		frame->state = NWK_TX_STATE_DELAY;
	} else {
  #ifdef NWK_ENABLE_SECURITY
		if (header->nwkFcf.security) {
			frame->state = NWK_TX_STATE_ENCRYPT;
		} else
  #endif
		frame->state = NWK_TX_STATE_DELAY;
	}

	frame->tx.status = NWK_SUCCESS_STATUS;

	if (frame->tx.control & NWK_TX_CONTROL_BROADCAST_PAN_ID) {
		header->macDstPanId = NWK_BROADCAST_PANID;
	} else {
		header->macDstPanId = nwkIb.panId;
	}

#ifdef NWK_ENABLE_ROUTING
	if (0 == (frame->tx.control & NWK_TX_CONTROL_DIRECT_LINK) &&
			0 ==
			(frame->tx.control & NWK_TX_CONTROL_BROADCAST_PAN_ID)) {
		nwkRoutePrepareTx(frame);
	} else
#endif
	header->macDstAddr = header->nwkDstAddr;

	header->macSrcAddr = nwkIb.addr;
	header->macSeq = ++nwkIb.macSeqNum;

	if (NWK_BROADCAST_ADDR == header->macDstAddr) {
		header->macFcf = 0x8841;
		frame->tx.timeout = (rand() & NWK_TX_DELAY_JITTER_MASK) + 1;
	} else {
		header->macFcf = 0x8861;
		frame->tx.timeout = 0;
	}
}

/*************************************************************************//**
*****************************************************************************/
void nwkTxBroadcastFrame(NwkFrame_t *frame)
{
	NwkFrame_t *newFrame;

	if (NULL == (newFrame = nwkFrameAlloc())) {
		return;
	}

	newFrame->state = NWK_TX_STATE_DELAY;
	newFrame->size = frame->size;
	newFrame->tx.status = NWK_SUCCESS_STATUS;
	newFrame->tx.timeout = (rand() & NWK_TX_DELAY_JITTER_MASK) + 1;
	newFrame->tx.confirm = NULL;
	memcpy(newFrame->data, frame->data, frame->size);

	newFrame->header.macFcf = 0x8841;
	newFrame->header.macDstAddr = NWK_BROADCAST_ADDR;
	newFrame->header.macDstPanId = frame->header.macDstPanId;
	newFrame->header.macSrcAddr = nwkIb.addr;
	newFrame->header.macSeq = ++nwkIb.macSeqNum;
}

/*************************************************************************//**
*****************************************************************************/
bool nwkTxAckReceived(NWK_DataInd_t *ind)
{
	NwkCommandAck_t *command = (NwkCommandAck_t *)ind->data;
	NwkFrame_t *frame = NULL;

	if (sizeof(NwkCommandAck_t) != ind->size) {
		return false;
	}

	while (NULL != (frame = nwkFrameNext(frame))) {
		if (NWK_TX_STATE_WAIT_ACK == frame->state &&
				frame->header.nwkSeq == command->seq) {
			frame->state = NWK_TX_STATE_CONFIRM;
			frame->tx.control = command->control;
			return true;
		}
	}

	return false;
}

/*************************************************************************//**
*****************************************************************************/
static void nwkTxAckWaitTimerHandler(SYS_Timer_t *timer)
{
	NwkFrame_t *frame = NULL;
	bool restart = false;

	while (NULL != (frame = nwkFrameNext(frame))) {
		if (NWK_TX_STATE_WAIT_ACK == frame->state) {
			restart = true;

			if (0 == --frame->tx.timeout) {
				nwkTxConfirm(frame, NWK_NO_ACK_STATUS);
			}
		}
	}

	if (restart) {
		SYS_TimerStart(timer);
	}
}

/*************************************************************************//**
*****************************************************************************/
void nwkTxConfirm(NwkFrame_t *frame, uint8_t status)
{
	frame->state = NWK_TX_STATE_CONFIRM;
	frame->tx.status = status;
}

#ifdef NWK_ENABLE_SECURITY

/*************************************************************************//**
*****************************************************************************/
void nwkTxEncryptConf(NwkFrame_t *frame)
{
	frame->state = NWK_TX_STATE_DELAY;
}

#endif

/*************************************************************************//**
*****************************************************************************/
static void nwkTxDelayTimerHandler(SYS_Timer_t *timer)
{
	NwkFrame_t *frame = NULL;
	bool restart = false;

	while (NULL != (frame = nwkFrameNext(frame))) {
		if (NWK_TX_STATE_WAIT_DELAY == frame->state) {
			restart = true;

			if (0 == --frame->tx.timeout) {
				frame->state = NWK_TX_STATE_SEND;
			}
		}
	}

	if (restart) {
		SYS_TimerStart(timer);
	}
}

/*************************************************************************//**
*****************************************************************************/
static uint8_t nwkTxConvertPhyStatus(uint8_t status)
{
	switch (status) {
	case PHY_STATUS_SUCCESS:
		return NWK_SUCCESS_STATUS;

	case PHY_STATUS_CHANNEL_ACCESS_FAILURE:
		return NWK_PHY_CHANNEL_ACCESS_FAILURE_STATUS;

	case PHY_STATUS_NO_ACK:
		return NWK_PHY_NO_ACK_STATUS;

	default:
		return NWK_ERROR_STATUS;
	}
}

/*************************************************************************//**
*****************************************************************************/
void PHY_DataConf(uint8_t status)
{
	nwkTxPhyActiveFrame->tx.status = nwkTxConvertPhyStatus(status);
	nwkTxPhyActiveFrame->state = NWK_TX_STATE_SENT;
	nwkTxPhyActiveFrame = NULL;
	nwkIb.lock--;
}

/*************************************************************************//**
*  @brief Tx Module task handler
*****************************************************************************/
void nwkTxTaskHandler(void)
{
	NwkFrame_t *frame = NULL;

	while (NULL != (frame = nwkFrameNext(frame))) {
		switch (frame->state) {
#ifdef NWK_ENABLE_SECURITY
		case NWK_TX_STATE_ENCRYPT:
		{
			nwkSecurityProcess(frame, true);
		}
		break;
#endif

		case NWK_TX_STATE_DELAY:
		{
			if (frame->tx.timeout > 0) {
				frame->state = NWK_TX_STATE_WAIT_DELAY;
				SYS_TimerStart(&nwkTxDelayTimer);
			} else {
				frame->state = NWK_TX_STATE_SEND;
			}
		}
		break;

		case NWK_TX_STATE_SEND:
		{
			if (NULL == nwkTxPhyActiveFrame) {
				nwkTxPhyActiveFrame = frame;
				frame->state = NWK_TX_STATE_WAIT_CONF;
				PHY_DataReq(&(frame->size));
				nwkIb.lock++;
			}
		}
		break;

		case NWK_TX_STATE_WAIT_CONF:
			break;

		case NWK_TX_STATE_SENT:
		{
			if (NWK_SUCCESS_STATUS == frame->tx.status) {
				if (frame->header.nwkSrcAddr == nwkIb.addr &&
						frame->header.nwkFcf.
						ackRequest) {
					frame->state = NWK_TX_STATE_WAIT_ACK;
					frame->tx.timeout = NWK_ACK_WAIT_TIME /
							NWK_TX_ACK_WAIT_TIMER_INTERVAL
							+ 1;
					SYS_TimerStart(&nwkTxAckWaitTimer);
				} else {
					frame->state = NWK_TX_STATE_CONFIRM;
				}
			} else {
				frame->state = NWK_TX_STATE_CONFIRM;
			}
		}
		break;

		case NWK_TX_STATE_WAIT_ACK:
			break;

		case NWK_TX_STATE_CONFIRM:
		{
#ifdef NWK_ENABLE_ROUTING
			nwkRouteFrameSent(frame);
#endif
			if (NULL == frame->tx.confirm) {
				nwkFrameFree(frame);
			} else {
				frame->tx.confirm(frame);
			}
		}
		break;

		default:
			break;
		}
	}
}
