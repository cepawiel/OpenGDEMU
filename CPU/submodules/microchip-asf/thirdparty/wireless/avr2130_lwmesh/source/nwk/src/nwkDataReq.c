/**
 * \file nwkDataReq.c
 *
 * \brief NWK_DataReq() implementation
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
#include "sysConfig.h"
#include "nwk.h"
#include "nwkTx.h"
#include "nwkFrame.h"
#include "nwkGroup.h"
#include "nwkDataReq.h"

/*- Types ------------------------------------------------------------------*/
enum {
	NWK_DATA_REQ_STATE_INITIAL,
	NWK_DATA_REQ_STATE_WAIT_CONF,
	NWK_DATA_REQ_STATE_CONFIRM,
};

/*- Prototypes -------------------------------------------------------------*/
static void nwkDataReqTxConf(NwkFrame_t *frame);

/*- Variables --------------------------------------------------------------*/
static NWK_DataReq_t *nwkDataReqQueue;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*  @brief Initializes the Data Request module
*****************************************************************************/
void nwkDataReqInit(void)
{
	nwkDataReqQueue = NULL;
}

/*************************************************************************//**
*  @brief Adds request @a req to the queue of outgoing requests
*  @param[in] req Pointer to the request parameters
*****************************************************************************/
void NWK_DataReq(NWK_DataReq_t *req)
{
	req->state = NWK_DATA_REQ_STATE_INITIAL;
	req->status = NWK_SUCCESS_STATUS;
	req->frame = NULL;

	nwkIb.lock++;

	if (NULL == nwkDataReqQueue) {
		req->next = NULL;
		nwkDataReqQueue = req;
	} else {
		req->next = nwkDataReqQueue;
		nwkDataReqQueue = req;
	}
}

/*************************************************************************//**
*  @brief Prepares and send outgoing frame based on the request @a req
* parameters
*  @param[in] req Pointer to the request parameters
*****************************************************************************/
static void nwkDataReqSendFrame(NWK_DataReq_t *req)
{
	NwkFrame_t *frame;

	if (NULL == (frame = nwkFrameAlloc())) {
		req->state = NWK_DATA_REQ_STATE_CONFIRM;
		req->status = NWK_OUT_OF_MEMORY_STATUS;
		return;
	}

	req->frame = frame;
	req->state = NWK_DATA_REQ_STATE_WAIT_CONF;

	frame->tx.confirm = nwkDataReqTxConf;
	frame->tx.control = req->options &
			NWK_OPT_BROADCAST_PAN_ID ?
			NWK_TX_CONTROL_BROADCAST_PAN_ID
			: 0;

	frame->header.nwkFcf.ackRequest = req->options &
			NWK_OPT_ACK_REQUEST ? 1 : 0;
	frame->header.nwkFcf.linkLocal = req->options &
			NWK_OPT_LINK_LOCAL ? 1 : 0;

#ifdef NWK_ENABLE_SECURITY
	frame->header.nwkFcf.security = req->options &
			NWK_OPT_ENABLE_SECURITY ? 1 : 0;
#endif

#ifdef NWK_ENABLE_MULTICAST
	frame->header.nwkFcf.multicast = req->options &
			NWK_OPT_MULTICAST ? 1 : 0;

	if (frame->header.nwkFcf.multicast) {
		NwkFrameMulticastHeader_t *mcHeader
			= (NwkFrameMulticastHeader_t *)frame->payload;

		mcHeader->memberRadius = req->memberRadius;
		mcHeader->maxMemberRadius = req->memberRadius;
		mcHeader->nonMemberRadius = req->nonMemberRadius;
		mcHeader->maxNonMemberRadius = req->nonMemberRadius;

		frame->payload += sizeof(NwkFrameMulticastHeader_t);
		frame->size += sizeof(NwkFrameMulticastHeader_t);
	}
#endif

	frame->header.nwkSeq = ++nwkIb.nwkSeqNum;
	frame->header.nwkSrcAddr = nwkIb.addr;
	frame->header.nwkDstAddr = req->dstAddr;
	frame->header.nwkSrcEndpoint = req->srcEndpoint;
	frame->header.nwkDstEndpoint = req->dstEndpoint;

	memcpy(frame->payload, req->data, req->size);
	frame->size += req->size;

	nwkTxFrame(frame);
}

/*************************************************************************//**
*  @brief Frame transmission confirmation handler
*  @param[in] frame Pointer to the sent frame
*****************************************************************************/
static void nwkDataReqTxConf(NwkFrame_t *frame)
{
	for (NWK_DataReq_t *req = nwkDataReqQueue; req; req = req->next) {
		if (req->frame == frame) {
			req->status = frame->tx.status;
			req->control = frame->tx.control;
			req->state = NWK_DATA_REQ_STATE_CONFIRM;
			break;
		}
	}

	nwkFrameFree(frame);
}

/*************************************************************************//**
*  @brief Confirms request @req to the application and remove it from the queue
*  @param[in] req Pointer to the request parameters
*****************************************************************************/
static void nwkDataReqConfirm(NWK_DataReq_t *req)
{
	if (nwkDataReqQueue == req) {
		nwkDataReqQueue = nwkDataReqQueue->next;
	} else {
		NWK_DataReq_t *prev = nwkDataReqQueue;
		while (prev->next != req) {
			prev = prev->next;
		}
		prev->next = ((NWK_DataReq_t *)prev->next)->next;
	}

	nwkIb.lock--;
	req->confirm(req);
}

/*************************************************************************//**
*  @brief Data Request module task handler
*****************************************************************************/
void nwkDataReqTaskHandler(void)
{
	for (NWK_DataReq_t *req = nwkDataReqQueue; req; req = req->next) {
		switch (req->state) {
		case NWK_DATA_REQ_STATE_INITIAL:
		{
			nwkDataReqSendFrame(req);
			return;
		}
		break;

		case NWK_DATA_REQ_STATE_WAIT_CONF:
			break;

		case NWK_DATA_REQ_STATE_CONFIRM:
		{
			nwkDataReqConfirm(req);
			return;
		}
		break;

		default:
			break;
		}
	}
}
