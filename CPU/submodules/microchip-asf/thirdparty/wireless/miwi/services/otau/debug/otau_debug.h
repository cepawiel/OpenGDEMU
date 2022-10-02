/**
* \file otau_debug.h
*
* \brief OTAU Debug interface
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

#ifndef OTAU_DEBUG_H
#define OTAU_DEBUG_H

#include "otau.h"
#include "sysTimer.h"

#define LOG_INTERVAL_MS            (3000)

#define TRACE_INTERVAL_MS          (200)

#define LOG_MASK					(0x0F)

COMPILER_PACK_SET(1)
typedef enum {
	OTA_READ_MEMORY_REQUEST = 0,
	OTA_READ_MEMORY_INDICATION,
	OTA_WRITE_MEMORY_REQUEST,
	OTA_WRITE_MEMORY_INDICATION,
	OTA_LOG_REQUEST,
	OTA_LOG_INDICATION,
	OTA_TRACE_REQUEST,
	OTA_TRACE_INDICATION
}otau_debug_msg_code_t;

typedef struct {
	uint8_t domainId;
	uint8_t msg_id;
	uint8_t mem_type;
	uint16_t mem_len;
	uint32_t mem_addr;
}read_memory_req_t;

typedef struct {
	uint8_t domainId;
	uint8_t msg_id;
	uint8_t status;
	uint8_t mem_type;
	uint16_t mem_len;
	uint32_t mem_addr;
	uint8_t mem_content[100];
}read_mem_indication_t;

typedef struct {
	uint8_t domainId;
	uint8_t msg_id;
	uint8_t mem_type;
	uint16_t mem_len;
	uint32_t mem_addr;
	uint8_t mem_content[100];
}write_memory_req_t;

typedef struct {
	uint8_t domainId;
	uint8_t msg_id;
	uint8_t status;
	uint8_t mem_type;
	uint16_t mem_len;
	uint32_t mem_addr;
}write_mem_indication_t;

typedef struct {
	uint8_t domainId;
	uint8_t msg_id;
	uint8_t log_bitmap;
}log_request_t;

typedef struct {
	uint8_t domainId;
	uint8_t msg_id;
}trace_request_t;
COMPILER_PACK_RESET()

void otauHandleDebugMsg(otau_domain_msg_t *otau_domain_msg);
void otauDebugTimerHandler(SYS_Timer_t *timer);;
void otauDebugInit(void);
void otauDebugSentFrame(uint8_t messageId, addr_mode_t addr_mode, uint8_t *addr, uint8_t status);
void otauDebugRcvdFrame(addr_mode_t addr_mode, uint8_t *src_addr, uint16_t length, uint8_t *payload);

#endif /* OTAU_DEBUG_H */
