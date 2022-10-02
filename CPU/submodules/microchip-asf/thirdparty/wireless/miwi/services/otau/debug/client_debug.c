/**
* \file client_debug.c
*
* \brief OTAU Client implementation
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

#if defined(OTAU_ENABLED) && !defined(OTAU_SERVER)
 
#include "stddef.h"
#include "string.h"
#include "stdint.h"
#include "client_debug.h"
#include "common_nvm.h"
#include "otau_debug.h"
#include "circularBuffer.h"
#include "io.h"

uint8_t req_log_addr_mode = NATIVE_ADDR_MODE;
uint8_t log_bitmap = 0x00;
uint8_t curr_trace_mode = 0;
uint8_t debugTraceEnable = 0;

uint8_t logBufferMem[BUF_SIZE];
uint8_t traceBufferMem[BUF_SIZE];

circularBuffer_t logBuffer;
circularBuffer_t traceBuffer;

debug_timer_state_t curr_debug_timer_state = DEBUG_TIMER_IDLE;

uint8_t debug_confirm_wait = 0;

void otauDebugInit(void)
{	
	curr_debug_timer_state = DEBUG_TIMER_IDLE;
	initCircularBuffer(&logBuffer, logBufferMem, BUF_SIZE);
	initCircularBuffer(&traceBuffer, traceBufferMem, BUF_SIZE);
}

void otauDebugRcvdFrame(addr_mode_t addr_mode, uint8_t *src_addr, uint16_t length, uint8_t *payload)
{
	uint8_t msg_code = *(payload + 1);
	switch (msg_code)
	{
		case OTA_READ_MEMORY_REQUEST:
		{
			if(!debug_confirm_wait)
			{
				read_mem_indication_t read_mem_ind;
				read_memory_req_t *read_mem_req = (read_memory_req_t *)payload;
				read_mem_ind.domainId = DOMAIN_OTAU_DEBUG;
				read_mem_ind.msg_id = OTA_READ_MEMORY_INDICATION;
				read_mem_ind.mem_type = read_mem_req->mem_type;
				read_mem_ind.mem_len = read_mem_req->mem_len;
				memcpy((uint8_t *)&read_mem_ind.mem_addr, (uint8_t *)&read_mem_req->mem_addr, sizeof(uint32_t));

				read_mem_ind.status = OTAU_SUCCESS;
				memcpy(read_mem_ind.mem_content, (uint8_t *)read_mem_req->mem_addr, read_mem_req->mem_len);
				debug_confirm_wait = 1;
				otauDataSend(addr_mode, src_addr, &read_mem_ind.domainId, sizeof(read_mem_indication_t) - (100 - read_mem_req->mem_len));
			}
			break;
		}
		case OTA_WRITE_MEMORY_REQUEST:
		{
			if(!debug_confirm_wait)
			{
				write_mem_indication_t write_mem_ind;
				write_memory_req_t *write_mem_req = (write_memory_req_t *)payload;
				uint32_t addr;
				write_mem_ind.domainId = DOMAIN_OTAU_DEBUG;
				write_mem_ind.msg_id = OTA_WRITE_MEMORY_INDICATION;
				write_mem_ind.status = OTAU_SUCCESS;
				write_mem_ind.mem_type = write_mem_req->mem_type;
				write_mem_ind.mem_len = write_mem_req->mem_len;
				memcpy((uint8_t *)&write_mem_ind.mem_addr, (uint8_t *)&write_mem_req->mem_addr, sizeof(uint32_t));
				memcpy((uint8_t *)&addr, (uint8_t *)&write_mem_req->mem_addr, sizeof(uint32_t));
#if SAMR21
				if(HMCRAMC0_ADDR <= addr)
#elif SAMR30
				if(HSRAM_ADDR <= addr)
#endif
				{
					memcpy((uint8_t *)addr, (uint8_t *)&write_mem_req->mem_content, write_mem_req->mem_len);
				}
				else
				{
					nvm_write(INT_FLASH, addr,(uint8_t *)&write_mem_req->mem_content, write_mem_req->mem_len);
				}
				debug_confirm_wait = 1;
				otauDataSend(addr_mode, src_addr, &write_mem_ind.domainId, sizeof(write_mem_indication_t));
			}
			break;
		}
		case OTA_LOG_REQUEST:
		{
			req_log_addr_mode = addr_mode;
			log_bitmap = *(payload + 2);
			log_bitmap &= LOG_MASK;
			if(log_bitmap)
			{
				curr_debug_timer_state = DEBUG_LOG_STATE;
				initCircularBuffer(&logBuffer, logBufferMem, BUF_SIZE);
				otauTimerStart(DOMAIN_OTAU_DEBUG, LOG_INTERVAL_MS, TIMER_MODE_PERIODIC);
			}
			break;
		}
		case OTA_TRACE_REQUEST:
		{
			debugTraceEnable = 0;
			if (EXTENDED_ADDR_MODE == addr_mode)
			{
				curr_trace_mode = 1;
			}
			else
			{
				curr_trace_mode = 0;
			}
			curr_debug_timer_state = DEBUG_TRACE_STATE;
			otauTimerStart(DOMAIN_OTAU_DEBUG, TRACE_INTERVAL_MS, TIMER_MODE_PERIODIC);
			break;
		}
		default:
		{
			break;
		}
	}
}

void otauDebugSentFrame(uint8_t messageId, addr_mode_t addr_mode, uint8_t *addr, uint8_t status)
{
	debug_confirm_wait = 0;
}

void otauDebugTimerHandler(SYS_Timer_t *timer)
{
	switch (curr_debug_timer_state)
	{
		case DEBUG_LOG_STATE:
		{
			uint8_t temp_len;
			uint8_t *dst_addr = NULL;
			log_indication_t log_ind;
			log_ind.domainId = DOMAIN_OTAU_DEBUG;
			log_ind.msg_id = OTA_LOG_INDICATION;
			if (EXTENDED_ADDR_MODE == req_log_addr_mode)
			{
				temp_len = readCircularBuffer(&logBuffer, PHY_MAX_PAYLOAD_SIZE, (uint8_t *)&log_ind.log_content);
			}
			else
			{
				temp_len = readCircularBuffer(&logBuffer, APP_MAX_PAYLOAD_SIZE, (uint8_t *)&log_ind.log_content);
			}
			if(temp_len && !debug_confirm_wait)
			{
				debug_confirm_wait = 1;
				if (EXTENDED_ADDR_MODE == req_log_addr_mode)
				{
					otauDataSend(EXTENDED_ADDR_MODE, dst_addr, &log_ind.domainId, temp_len + 2);
				}
				else
				{
					otauDataSend(NATIVE_ADDR_MODE, dst_addr, &log_ind.domainId, temp_len + 2);
				}
			}
			break;
		}
		case DEBUG_TRACE_STATE:
		{
			uint8_t temp_len;
			uint8_t max_len = 0;
			uint8_t *dst_addr = NULL;
			trace_indication_t trace_ind;
			trace_ind.domainId = DOMAIN_OTAU_DEBUG;
			trace_ind.msg_id = OTA_TRACE_INDICATION;
			if (curr_trace_mode)
			{
				temp_len = readCircularBuffer(&traceBuffer, PHY_MAX_PAYLOAD_SIZE - 1, (uint8_t *)&trace_ind.trace_content);
				max_len = PHY_MAX_PAYLOAD_SIZE - 1;
			}
			else
			{
				temp_len = readCircularBuffer(&traceBuffer, APP_MAX_PAYLOAD_SIZE - 1, (uint8_t *)&trace_ind.trace_content);
				max_len = APP_MAX_PAYLOAD_SIZE - 1;
			}
			if (temp_len < max_len)
			{
				trace_ind.trace_end = 1;
				debugTraceEnable = 1;
				if(log_bitmap)
				{
					curr_debug_timer_state = DEBUG_LOG_STATE;
					otauTimerStart(DOMAIN_OTAU_DEBUG, LOG_INTERVAL_MS, TIMER_MODE_PERIODIC);
				}
				else
				{
					otauTimerStop(DOMAIN_OTAU_DEBUG);
				}
			}
			else
			{
				trace_ind.trace_end = 0;
			}
			if(temp_len && !debug_confirm_wait)
			{
				debug_confirm_wait = 1;
				if (curr_trace_mode)
				{
					//get_server_details(EXTENDED_ADDR_MODE, dst_addr);
					otauDataSend(EXTENDED_ADDR_MODE, dst_addr, &trace_ind.domainId, temp_len + 3);
				}
				else
				{
					//get_server_details(NATIVE_ADDR_MODE, dst_addr);
					otauDataSend(NATIVE_ADDR_MODE, dst_addr, &trace_ind.domainId, temp_len + 3);
				}
			}
			break;
		}
		default:
		break;
	}
}

void otau_log(uint8_t log_type, module_id_t module_id, error_code_t error_code, uint8_t len, uint8_t* user_log)
{
	if((log_type & log_bitmap))
	{
#ifdef __ICCARM__
		void* temp = (void*)__get_LR();
#else
		void* temp = (void*)__builtin_return_address(0);
#endif
		uint8_t eof = 0xFF;
		log_header_t log_header;
		log_header.sof = 0xF0;
		log_header.log_len = 7 + len;
		log_header.log_type = (log_type_t)log_type;
		log_header.module_id = module_id;
		log_header.addr = (uint32_t) temp;
		log_header.error_code = error_code;
		writeCircularBuffer(&logBuffer, sizeof(log_header_t), (uint8_t *)&log_header);
		if (0 != len && NULL != user_log)
		{
			writeCircularBuffer(&logBuffer, len, user_log);
		} 
		writeCircularBuffer(&logBuffer, 1, &eof);
	}
}

void otau_trace(trace_type_t trace_type)
{
	trace_t trace;
#ifdef __ICCARM__
	void* temp = (void*)__get_LR();
#else
	void* temp = (void*)__builtin_return_address(0);
#endif
	trace.sof = 0xF0;
	trace.length = 0x05;
	trace.addr = (uint32_t) temp;
	trace.trace_type = trace_type;
	trace.eof = 0xFF;
	writeCircularBuffer(&traceBuffer, sizeof(trace_t), (uint8_t *)&trace);
}
#endif //#if defined(OTAU_ENABLED) && !defined(OTAU_SERVER)
