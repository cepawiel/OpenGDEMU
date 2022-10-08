/**
* \file client_debug.h
*
* \brief OTAU Client interface
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


#ifndef CLIENT_DEBUG_H
#define CLIENT_DEBUG_H

#include "otau.h"
#if SAMD || SAMR21
#include "samr21.h"
#endif

#define BUF_SIZE 500

typedef enum {
	DEBUG_TIMER_IDLE,
	DEBUG_LOG_STATE,
	DEBUG_TRACE_STATE
}debug_timer_state_t;

COMPILER_PACK_SET(1)
typedef struct {
	uint8_t sof;
	uint8_t log_len;
	log_type_t log_type;
	module_id_t module_id;
	uint32_t addr;
	error_code_t error_code;
}log_header_t;

typedef struct {
	uint8_t sof;
	uint8_t length;
	uint32_t addr;
	trace_type_t trace_type;
	uint8_t eof;
}trace_t;

typedef struct {
	uint8_t domainId;
	uint8_t msg_id;
	uint8_t log_content[100];
}log_indication_t;

typedef struct {
	uint8_t domainId;
	uint8_t msg_id;
	uint8_t trace_end;
	uint8_t trace_content[99];
}trace_indication_t;

COMPILER_PACK_RESET()

#endif /* CLIENT_DEBUG_H */