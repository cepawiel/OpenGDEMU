/**
 * \file dfu_api.h
 *
 * \brief DFU API declarations
 *
 * Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
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
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef __DFU_API_H__
#define __DFU_API_H__

#include "bm_program_mode.h"
#include "ble_api.h"

#define DFU_MAX_DATA_LENGTH 192

typedef enum
{
	DFU_SUCCESS  = 0x00,
	DFU_FAIL
} dfu_status_t;

typedef enum
{
	DFU_WRITE_CONTINUE_START,
	DFU_WRITE_CONTINUE_PROCEED,
	DFU_WRITE_CONTINUE_STOP,
} dfu_write_continue_mode_t;

dfu_status_t dfu_init(platform_init_t *platform_init);
dfu_status_t dfu_deinit(platform_init_t *platform_init);
dfu_status_t dfu_program_memory_erase(uint32_t address, uint8_t length);
dfu_status_t dfu_program_memory_write(uint32_t address, uint8_t* data, uint8_t length, bool write_continue);
dfu_status_t dfu_program_memory_write_continue(uint32_t address, uint8_t* data, uint8_t length, uint8_t continue_mode, uint32_t continue_length);
dfu_status_t dfu_program_memory_read(uint32_t address, uint8_t* data, uint8_t length);
bool dfu_response_check(uint8_t byte);

#endif /* __DFU_API_H__ */