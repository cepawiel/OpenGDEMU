/**
* \file server_debug.c
*
* \brief Server debug implementation
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
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
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
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

#if defined(OTAU_ENABLED) && defined(OTAU_SERVER)

#include "stddef.h"
#include "string.h"
#include "stdint.h"
#include "server_debug.h"
#include "otau_debug.h"
#include "otau_parser.h"

void otauDebugInit(void)
{

}

void otauDebugRcvdFrame(addr_mode_t addr_mode, uint8_t *src_addr, uint16_t length, uint8_t *payload)
{
	uint8_t msg_code = *(payload + 1);
	payload += 2;
	length -= 2;
	switch (msg_code)
	{
		case OTA_READ_MEMORY_INDICATION:
		{
			send_server_data(DOMAIN_OTAU_DEBUG, addr_mode, src_addr, READ_MEMORY_INDICATION, payload, length);
			break;
		}
		case OTA_WRITE_MEMORY_INDICATION:
		{
			send_server_data(DOMAIN_OTAU_DEBUG, addr_mode, src_addr, WRITE_MEMORY_INDICATION, payload, length);
			break;
		}
		case OTA_LOG_INDICATION:
		{
			send_server_data(DOMAIN_OTAU_DEBUG, addr_mode, src_addr, LOG_INDICATION, payload, length);
			break;
		}
		case OTA_TRACE_INDICATION:
		{
			send_server_data(DOMAIN_OTAU_DEBUG, addr_mode, src_addr, TRACE_INDICATION, payload, length);
			break;
		}
	}
}

void otauDebugSentFrame(uint8_t messageId, uint8_t addr_mode, uint8_t *addr, uint8_t status)
{
	switch (messageId)
	{
		case OTA_READ_MEMORY_REQUEST:
		send_server_data(DOMAIN_OTAU_DEBUG, addr_mode, addr, READ_MEMORY_CONFIRM, &status, 1);
		break;
		case OTA_WRITE_MEMORY_REQUEST:
		send_server_data(DOMAIN_OTAU_DEBUG, addr_mode, addr, WRITE_MEMORY_CONFIRM, &status, 1);
		break;
		case OTA_LOG_REQUEST:
		send_server_data(DOMAIN_OTAU_DEBUG, addr_mode, addr, LOG_CONFIRM, &status, 1);
		break;
		case OTA_TRACE_REQUEST:
		send_server_data(DOMAIN_OTAU_DEBUG, addr_mode, addr, TRACE_CONFIRM, &status, 1);
		break;
		default:
		break;
		/*  */
	}
}

/**
 * \brief Parses the Received Data in the Buffer and Process the Commands
 *accordingly.
 */
void otauHandleDebugMsg(otau_domain_msg_t *otau_domain_msg)
{
	uint8_t *msg = &(otau_domain_msg->domain_msg);
	/* *msg is the mode specification for phy/app */
	uint8_t msg_id = *(uint8_t *)(msg + 1);
	msg += 2;

	uint8_t addr_mode = *msg;
	uint8_t *addr = msg + 1;
	if(NATIVE_ADDR_MODE == addr_mode)
	{
		msg += 1 + NATIVE_ADDR_SIZE;
	}
	else if(EXTENDED_ADDR_MODE == addr_mode)
	{
		msg += (1 + EXTENDED_ADDR_SIZE);
	}
	else
	{
		addr = NULL;
		msg++;
	}

	switch (msg_id)
	{ /* message type */
		case READ_MEMORY_REQUEST:
		{
			read_memory_req_t read_mem_req;
			read_mem_req.domainId = DOMAIN_OTAU_DEBUG;
			read_mem_req.msg_id = OTA_READ_MEMORY_REQUEST;
			memcpy(&read_mem_req.mem_type, msg, sizeof(read_memory_req_t) - 1);
			otauDataSend(addr_mode, addr, &read_mem_req, sizeof(read_memory_req_t));
			break;
		}
		case WRITE_MEMORY_REQUEST:
		{
			write_memory_req_t write_mem_req;
			write_mem_req.domainId = DOMAIN_OTAU_DEBUG;
			write_mem_req.msg_id = OTA_WRITE_MEMORY_REQUEST;
			memcpy(&write_mem_req.mem_type, msg, sizeof(write_memory_req_t) - 1);
			otauDataSend(addr_mode, addr, &write_mem_req, sizeof(write_memory_req_t) - (100 - write_mem_req.mem_len));
			break;
		}
		case LOG_REQUEST:
		{
			log_request_t log_req;
			log_req.domainId = DOMAIN_OTAU_DEBUG;
			log_req.msg_id = OTA_LOG_REQUEST;
			log_req.log_bitmap = *msg;
			otauDataSend(addr_mode, addr, &log_req, sizeof(log_request_t));
			break;
		}
		case TRACE_REQUEST:
		{
			trace_request_t trace_req;
			trace_req.domainId = DOMAIN_OTAU_DEBUG;
			trace_req.msg_id = OTA_TRACE_REQUEST;
			otauDataSend(addr_mode, addr, &trace_req, sizeof(trace_request_t));
			break;
		}
		default:
		{
			/* Do nothing...*/
		}
	}
}


void otauDebugTimerHandler(SYS_Timer_t *timer)
{

}


void otau_log(uint8_t log_type, module_id_t module_id, error_code_t error_code, uint8_t len, uint8_t* user_log)
{

}

void otau_trace(trace_type_t trace_type)
{

}
#endif //#if defined(OTAU_ENABLED) && defined(OTAU_SERVER)
