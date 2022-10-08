/**
 * \file acc_gyro_sensor_service.c
 *
 * \brief Accelerometer Gyroscope sensor service functionalities
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

#include "platform_files.h"
#include "bm_mode.h"
#include "bm_application_mode.h"
#include "bm_utils.h"
#include "event_mem.h"
#include "ble_api.h"
#include "ble_manager.h"
#include "bno055_port.h"
#include "bno055.h"
#include "acc_gyro_sensor_service.h"

ble_char_t sensor_char[2] = {0};
ble_service_t sensor_service = {0};

ble_service_t * acc_gyro_sensor_service_init(acc_sensor_t *acc_sensor_char, gyro_sensor_t *gyro_sensor_char)
{
	uint8_t acce_gyro_service_uuid[BLE_UUID_128B_LEN] = {ACCE_GYRO_SENSOR_SERVICE_UUID};
	uint8_t acce_char_uuid[BLE_UUID_128B_LEN] = {ACCE_UUID};
	uint8_t gyro_char_uuid[BLE_UUID_128B_LEN] = {GYRO_UUID};
	
	sensor_service.type = PRIMARY_SERVICE;
	sensor_service.perm = BLE_PM_WRITABLE;
	sensor_service.handle = 0;
	sensor_service.uuid.type = BLE_UUID_128B;
	memcpy(sensor_service.uuid.uuid.uuid_128b, acce_gyro_service_uuid, BLE_UUID_128B_LEN);
	memcpy_inplace_reorder(sensor_service.uuid.uuid.uuid_128b, BLE_UUID_128B_LEN);
	sensor_service.char_count = 2;
	sensor_service.char_list = sensor_char;
	
	/* Accelerometer characteristic */
	sensor_service.char_list[0].char_val.char_handle = 0;
	sensor_service.char_list[0].char_val.properties = (BLE_CHAR_READ | BLE_CHAR_NOTIFY);
	sensor_service.char_list[0].char_val.permissions = BLE_PM_WRITABLE;
	sensor_service.char_list[0].char_val.value_handle = 0;
	sensor_service.char_list[0].char_val.uuid.type = BLE_UUID_128B;
	memcpy(sensor_service.char_list[0].char_val.uuid.uuid.uuid_128b, acce_char_uuid, BLE_UUID_128B_LEN);
	sensor_service.char_list[0].char_val.max_len = sizeof(acc_sensor_value_t);
	sensor_service.char_list[0].char_val.len = sizeof(acc_sensor_value_t);
	sensor_service.char_list[0].char_val.init_value = (uint8_t *)acc_sensor_char;
	/* Client Characteristic Configuration Descriptor */
	sensor_service.char_list[0].client_config_desc.ccd_included = true;
	sensor_service.char_list[0].client_config_desc.handle = 0;
	sensor_service.char_list[0].client_config_desc.perm = BLE_PM_WRITABLE;
	sensor_service.char_list[0].client_config_desc.ccd_value = 0;
	
	/* Gyroscope characteristic */
	sensor_service.char_list[1].char_val.char_handle = 0;
	sensor_service.char_list[1].char_val.properties = (BLE_CHAR_READ | BLE_CHAR_NOTIFY);
	sensor_service.char_list[1].char_val.permissions = BLE_PM_WRITABLE;
	sensor_service.char_list[1].char_val.value_handle = 0;
	sensor_service.char_list[1].char_val.uuid.type = BLE_UUID_128B;
	memcpy(sensor_service.char_list[1].char_val.uuid.uuid.uuid_128b, gyro_char_uuid, BLE_UUID_128B_LEN);
	sensor_service.char_list[1].char_val.max_len = sizeof(gyro_sensor_value_t);
	sensor_service.char_list[1].char_val.len = sizeof(gyro_sensor_value_t);
	sensor_service.char_list[1].char_val.init_value = (uint8_t *)gyro_sensor_char;
	/* Client Characteristic Configuration Descriptor */
	sensor_service.char_list[1].client_config_desc.ccd_included = true;
	sensor_service.char_list[1].client_config_desc.handle = 0;
	sensor_service.char_list[1].client_config_desc.perm = BLE_PM_WRITABLE;
	sensor_service.char_list[1].client_config_desc.ccd_value = 0;
	
	return &sensor_service;
}

ble_status_t acce_sensor_data_send(acc_sensor_t *acce_data, uint8_t conn_handle)
{
	memcpy_inplace_reorder((uint8_t *)&acce_data->x, sizeof(acce_data->x));
	memcpy_inplace_reorder((uint8_t *)&acce_data->y, sizeof(acce_data->y));
	memcpy_inplace_reorder((uint8_t *)&acce_data->z, sizeof(acce_data->z));
	DBG_LOG("Acce data x = %d || y = %d || z = %d", acce_data->x, acce_data->y, acce_data->z);
	/* Notify the accelerometer data */
	return ble_characteristic_value_send(conn_handle, sensor_service.char_list[0].char_val.value_handle, (uint8_t *)acce_data, sizeof(acc_sensor_t));
}

ble_status_t gyro_sensor_data_send(gyro_sensor_t *gyro_data, uint8_t conn_handle)
{
	memcpy_inplace_reorder((uint8_t *)&gyro_data->x, sizeof(gyro_data->x));
	memcpy_inplace_reorder((uint8_t *)&gyro_data->y, sizeof(gyro_data->y));
	memcpy_inplace_reorder((uint8_t *)&gyro_data->z, sizeof(gyro_data->z));
	DBG_LOG("Gyro data x = %d || y = %d || z = %d", gyro_data->x, gyro_data->y, gyro_data->z);
	/* Notify the gyroscope data */
	return ble_characteristic_value_send(conn_handle, sensor_service.char_list[1].char_val.value_handle, (uint8_t *)gyro_data, sizeof(gyro_sensor_t));
}