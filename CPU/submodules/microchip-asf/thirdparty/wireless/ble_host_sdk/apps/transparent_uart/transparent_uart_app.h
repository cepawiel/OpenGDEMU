/**
 * \file transparent_uart_app.h
 *
 * \brief Transparent UART application
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

#ifndef __TRANSPARENT_UART_APP_H__
#define __TRANSPARENT_UART_APP_H__

#define EVENT_BUFFER_LENGTH		512

/* Beacon Advertisement data */
#define ADV_TYPE_MANUFACTURER_SPECIFIC_DATA			0xff
#define BEACON_ADV_TYPE								ADV_TYPE_MANUFACTURER_SPECIFIC_DATA
#define COMPANY_IDENTIFIER_CODE						0x4c, 0x00
#define BEACON_ADV_FLAG								0x02, 0x01, 0x05
/* iBeacon parameters */
#define BEACON_TYPE									0x02, 0x15
#define PROXIMITY_UUID								0x21, 0x8a, 0xf6, 0x52, 0x73, 0xe3, 0x40, 0xb3, 0xb4, 0x1c, 0x19, 0x53, 0x24, 0x2c, 0x72, 0xf4
#define MAJOR										0x00, 0xbb
#define MINOR										0x00, 0x45
#define MEASURED_POWER								0xc5

#define ADDTIONAL_MANUFACTURER_SPECIFIC_DATA		BEACON_TYPE, PROXIMITY_UUID, MAJOR, MINOR, MEASURED_POWER
#define ADDTIONAL_MANUFACTURER_SPECIFIC_DATA_LENGTH	0x17
#define BEACON_ADV_LENGTH							0x1a

#endif /* __TRANSPARENT_UART_APP_H__ */
