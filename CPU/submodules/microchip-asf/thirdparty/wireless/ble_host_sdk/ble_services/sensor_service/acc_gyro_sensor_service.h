/**
 * \file acc_gyro_sensor_service.h
 *
 * \brief Accelerometer Gyroscope sensor service
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

#ifndef __ACC_GYRO_SENSOR_SERVICE_H__
#define __ACC_GYRO_SENSOR_SERVICE_H__

#define ACCE_GYRO_SENSOR_SERVICE_UUID	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xa6, 0x87, 0xe5, 0x11, 0x36, 0x39, 0xc1, 0xba, 0x5a, 0xf0
#define ACCE_UUID						0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xa6, 0x87, 0xe5, 0x11, 0x36, 0x39, 0xd7, 0xba, 0x5a, 0xf0
#define GYRO_UUID						0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xa6, 0x87, 0xe5, 0x11, 0x36, 0x39, 0xd4, 0xba, 0x5a, 0xf0

#define NUMBER_OF_SENSORS		2

typedef enum
{
	SENSOR_1_ACCELEROMETER,
	SENSOR_2_GYROSCOPE,
}sensor_list_t;

typedef struct acc_sensor_value_t_
{
	uint16_t handle;
	int16_t x;
	int16_t y;
	int16_t z;
}acc_sensor_value_t;

typedef acc_sensor_value_t gyro_sensor_value_t;

typedef struct bno055_accel_t acc_sensor_t;
typedef struct bno055_gyro_t gyro_sensor_t;

ble_service_t * acc_gyro_sensor_service_init(acc_sensor_t *acc_sensor_char, gyro_sensor_t *gyro_sensor_char);
ble_status_t acce_sensor_data_send(acc_sensor_t *acce_data, uint8_t conn_handle);
ble_status_t gyro_sensor_data_send(gyro_sensor_t *gyro_data, uint8_t conn_handle);

#endif //__ACC_GYRO_SENSOR_SERVICE_H__