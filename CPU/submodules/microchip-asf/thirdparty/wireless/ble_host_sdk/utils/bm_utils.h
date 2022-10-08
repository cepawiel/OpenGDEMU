/**
 * \file bm_utils.h
 *
 * \brief BM utility declarations
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

#ifndef __BM_UTILS_H__
#define __BM_UTILS_H__

#include "stdint.h"

uint8_t set_to_upper(uint8_t lower);
uint8_t get_hex_byte(uint8_t * str, uint8_t * val);
uint8_t get_hex(uint8_t * str, uint16_t * val);
uint8_t get_hex_24bit(uint8_t * str, uint32_t * val);
uint8_t get_hex_dword(uint8_t * str, uint32_t * val);
uint8_t get_hex_bdaddrs(uint8_t * str, uint64_t * val);
uint8_t get_hex_long(uint8_t * str, uint64_t * val);

void format_hex_byte(uint8_t * str, uint8_t val);
void format_hex_byte2(uint8_t * str, uint8_t val);
void format_hex(uint8_t * str, uint16_t val);
void format_hex_24bit(uint8_t * str, uint32_t val);
void format_hex_dword(uint8_t * str, uint32_t val);
void format_hex_bdaddrs(uint8_t * str, uint64_t val);
void format_hex_bdaddrs2(uint8_t * str, uint8_t* val);
void format_hex_long(uint8_t * str, uint64_t val);

void memcpy_ascii_reorder(uint8_t* a, uint8_t* b, int len);
void memcpy_ascii_order(uint8_t* a, uint8_t* b, int len);
void memcpy_reorder(uint8_t* a, uint8_t* b, int len);
void memcpy_nibble_reorder(uint8_t* a, uint8_t* b, int len);
uint8_t* memcpy_inplace_reorder(uint8_t* data, uint16_t len);
uint8_t* memcpy_nibble_inplace_reorder(uint8_t* data, uint16_t len);

uint8_t calc_chksum_8bit(uint8_t* data, uint16_t length);
#endif //__BM_UTILS_H__
