/**
 * \file multilink_multirole_app.h
 *
 * \brief Multilink-Multirole application
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

#ifndef __MULTILINK_MULTIROLE_H__
#define __MULTILINK_MULTIROLE_H__

/* Length of event buffer */
#define EVENT_BUFFER_LENGTH		512

/* Total number of sensors added in the system */
#define NUMBER_OF_SENSORS		2

/* Total number of remote device supported */
#define MAX_REMOTE_DEVICE				2
/* Maximum remote peripheral device supported */
#define MAX_REMOTE_PERIPHERAL_DEVICE	1
/* Maximum remote central device supported */
#define MAX_REMOTE_CENTRAL_DEVICE		1


enum
{
	APP_DEVICE_ROLE_PERIPHERAL = 0x00,
	APP_DEVICE_ROLE_CENTRAL,
};

enum
{
	APP_STATE_DISCOVERY_PENDING = 0x01,
	APP_STATE_READY,
};

typedef struct PACKED
{
	ble_handle_t char_attr_handle;
	uint8_t property;
	ble_handle_t char_value_attr_handle;
	ble_uuid_t	uuid;
	ble_handle_t cccd_handle;
	uint16_t cccd_value;
}sensor_char_t;

typedef struct PACKED
{
	ble_uuid_t service_uuid;
	uint8_t num_of_sensor_chars;
	sensor_char_t sensor_char_list[2];
}sensor_service_t;

typedef struct PACKED
{
	uint8_t in_use;
	uint8_t conn_handle;
	uint8_t dev_role;
	uint8_t state;
	ble_addr_t peer_addr;
	ble_conn_param_t conn_param;
	sensor_service_t sensor_service;
}app_ble_remote_device_info_t;


#endif /* __MULTILINK_MULTIROLE_H__ */
