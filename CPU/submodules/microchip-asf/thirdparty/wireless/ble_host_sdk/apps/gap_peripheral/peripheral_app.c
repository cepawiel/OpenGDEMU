/**
 * \file peripheral_app.c
 *
 * \brief GAP-Peripheral application
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
/**
 * \mainpage GAP_Peripheral Example
 * \section Introduction
 * **************************** Introduction *********************************
 * + The GAP_Peripheral Application is an example application that bring-up Bluetooth GAP-Peripheral role and GATT-Server role. 
 * + As a GAP-Peripheral, it starts advertise itself and connect with remote GAP-Central device.  
 * + As GATT-Server, it notifies accelerometer and gyroscope sensor data to GATT-Client.
 *- Supported Evolution Kit -
 *	+ ATSAML21-XPRO-B + BM71 XPRO
 */
/*- Includes ---------------------------------------------------------------*/
#include "platform_files.h"
#include "bm_mode.h"
#include "bm_application_mode.h"
#include "bm_utils.h"
#include "event_mem.h"
#include "ble_api.h"
#include "platform.h"
#include "ble_manager.h"
#include "peripheral_app.h"
#include "bno055_port.h"
#include "bno055.h"
#include "acc_gyro_sensor_service.h"


static void app_init(void);
static ble_status_t app_adv_report_cb(event_msg_t* msg);
static ble_status_t app_connected_cb(event_msg_t* msg);
static ble_status_t app_disconnected_cb(event_msg_t* msg);
static ble_status_t app_conn_param_update_cb(event_msg_t* msg);
static ble_status_t app_char_value_write_cb(event_msg_t* msg);
static ble_status_t app_service_disc_resp_cb(event_msg_t* msg);
static ble_status_t app_char_disc_resp_cb(event_msg_t* msg);
static ble_status_t app_char_descriptor_disc_resp_cb(event_msg_t* msg);
static ble_status_t app_char_value_received_cb(event_msg_t* msg);
static ble_status_t app_cmd_complete_cb(event_msg_t* msg);
static ble_status_t app_status_report_cb(event_msg_t* msg);
static ble_status_t app_le_end_test_result_cb(event_msg_t* msg);
static ble_status_t app_config_mode_status_cb(event_msg_t* msg);
static ble_status_t app_passkey_entry_req_cb(event_msg_t* msg);
static ble_status_t app_pairing_complete_cb(event_msg_t* msg);
static ble_status_t app_passkey_confirm_req_cb(event_msg_t* msg);
static ble_status_t app_trans_data_received_cb(event_msg_t* msg);

static const ble_gap_event_cb_t app_gap_event_handle = {
	.adv_report = app_adv_report_cb,
	.connected = app_connected_cb,
	.disconnected = app_disconnected_cb,
	.conn_param_update = app_conn_param_update_cb,
};

static const ble_gatt_server_event_cb_t app_gatt_server_event_handle = {
	.char_value_write = app_char_value_write_cb,
};

static const ble_gatt_client_event_cb_t app_gatt_client_event_handle = {
	.service_disc_resp = app_service_disc_resp_cb,
	.char_disc_resp = app_char_disc_resp_cb,
	.char_descriptor_disc_resp = app_char_descriptor_disc_resp_cb,
	.char_value_received = app_char_value_received_cb,
};

static const ble_common_event_cb_t app_common_event_handle = {
	.cmd_complete = app_cmd_complete_cb,
	.status_report = app_status_report_cb,
	.le_end_test_result = app_le_end_test_result_cb,
	.config_mode_status = app_config_mode_status_cb,
};

static const ble_pairing_event_cb_t app_pairing_event_handle = {
	.passkey_entry_req = app_passkey_entry_req_cb,
	.pairing_complete = app_pairing_complete_cb,
	.passkey_confirm_req = app_passkey_confirm_req_cb,
};

static const ble_gatt_transparent_event_cb_t app_transparent_event_handle = {
	.trans_data_received = app_trans_data_received_cb,
};

/* Event parameters */
uint8_t event_buf[EVENT_BUFFER_LENGTH];
event_t evt_param = {.event_msg.data = event_buf, .event_msg.data_len = 0, .event_id = 0};

/* Advertisement data */
static const uint8_t adv_data[]	= {	0x02,ADV_FLAGS,0x05,          /* Flags */
	0x15, ADV_MANUFATURER_SPECIFIC_DATA, 0xCD, 0x00, 0xFE, 0x14, 0xAD, 0x11, 0xCF, 0x40, 0x06, 0x3F, 0x11, 0xE5, 0xBE, 0x3E, 0x00, 0x02, 0xA5, 0xD5, 0xC5, 0x2C,
};

static const uint8_t scan_resp_data[] = {0x0C, 0x09, 0x53, 0x65, 0x6E, 0x73, 0x6F, 0x72, 0x2D, 0x44, 0x65, 0x6D, 0x6F};

app_ble_remote_device_info_t remote_device_info;
bool app_notify_timer_expired = false;
bool acce_notify_enabled = false;
bool gyro_notify_enabled = false;

acc_sensor_t acc_sensor_value = {0};
gyro_sensor_t gyro_sensor_value = {0};
static ble_service_t *ble_service = NULL;

static ble_status_t app_adv_report_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_connected_cb(event_msg_t* msg)
{
	ble_conn_complete_event_t *connected = (ble_conn_complete_event_t *)msg->data;
	
	if(BLE_SUCCESS == connected->status)
	{
		remote_device_info.conn_handle = connected->conn_handle;
		remote_device_info.peer_addr.type = connected->peer_addr.type;
		memcpy(remote_device_info.peer_addr.addr, connected->peer_addr.addr, BLE_ADDR_LEN);
		memcpy(&remote_device_info.conn_param, &connected->conn_param, sizeof(ble_conn_param_t));
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_disconnected_cb(event_msg_t* msg)
{
	ble_disconnect_complete_event_t *disconnectd = (ble_disconnect_complete_event_t *)msg->data;
	
	DBG_LOG("Device disconnected reason = 0x%02X", disconnectd->reason);
	acce_notify_enabled = false;
	gyro_notify_enabled = false;
	hw_timer_stop();
	ble_mgr_adv_start(adv_data, sizeof(adv_data), scan_resp_data, sizeof(scan_resp_data));
	return BLE_SUCCESS;
}

static ble_status_t app_conn_param_update_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_char_value_write_cb(event_msg_t* msg)
{
	ble_write_char_value_event_t *write_char_value = (ble_write_char_value_event_t *)msg->data;
	uint8_t acce_uuid[BLE_UUID_128B_LEN] = {ACCE_UUID};
	
	for(uint8_t sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
	{
		if(ble_service->char_list[sensor_index].client_config_desc.handle == write_char_value->char_value_handle)
		{
			ble_service->char_list[sensor_index].client_config_desc.ccd_value = (uint16_t)write_char_value->char_value[1] << 8;
			ble_service->char_list[sensor_index].client_config_desc.ccd_value |= (uint16_t)write_char_value->char_value[0];
		
			if(ble_service->char_list[sensor_index].client_config_desc.ccd_value & BLE_CCCD_NOTIFICATION_ENABLED)
			{
				DBG_LOG("Notification enabled!!!");
				if(!memcmp(ble_service->char_list[sensor_index].char_val.uuid.uuid.uuid_128b, acce_uuid, BLE_UUID_128B_LEN))
				{
					acce_notify_enabled = true;
				}
				else  //If more sensors included in future then need to check the UUID against each of them
				{
					gyro_notify_enabled = true;
				}
				/* Start timer to send sensor data periodically */
				hw_timer_start(1);
			}
			else
			{
				DBG_LOG("Notification disabled!!!");
				if(!memcmp(ble_service->char_list[sensor_index].char_val.uuid.uuid.uuid_128b, acce_uuid, BLE_UUID_128B_LEN))
				{
					acce_notify_enabled = false;
				}
				else  //If more sensors included in future then need to check the UUID against each of them
				{
					gyro_notify_enabled = false;
				}
				
				if(!acce_notify_enabled && !gyro_notify_enabled)
				{
					hw_timer_stop();
				}
			}
		}
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_service_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_char_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_char_descriptor_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_char_value_received_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_cmd_complete_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_status_report_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_le_end_test_result_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_config_mode_status_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_passkey_entry_req_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_pairing_complete_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_passkey_confirm_req_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_trans_data_received_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

/* timer callback function */
static void timer_callback_fn(void)
{
	/* Add timer callback functionality here */
	app_notify_timer_expired = true;
}

void button_cb(void)
{
	/* Add button callback functionality here */
}

static void app_init(void)
{
	/* BLE event callback registration */
	ble_mgr_events_register_callback(BLE_GAP_EVENT_TYPE, &app_gap_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_SERVER_EVENT_TYPE, &app_gatt_server_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_CLIENT_EVENT_TYPE, &app_gatt_client_event_handle);
	ble_mgr_events_register_callback(BLE_COMMON_EVENT_TYPE, &app_common_event_handle);
	ble_mgr_events_register_callback(BLE_PAIRING_EVENT_TYPE, &app_pairing_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_TP_EVENT_TYPE, &app_transparent_event_handle);
}


int main(void)
{
	ble_status_t status;
	acc_sensor_t accel_xyz_data = {0};
	gyro_sensor_t gyro_xyz_data = {0};
	
	system_init();
	/* Hardware timer */
	hw_timer_init();
	hw_timer_register_callback(timer_callback_fn);
	/* Initialize serial console */
	serial_console_init();
	/* button initialization */
	button_init();
	
	DBG_LOG("Initializing BLE Application");
	ble_mgr_device_init();
	app_init();
	
	/* Initialize accelerometer and gyroscope sensor service */
	ble_service = acc_gyro_sensor_service_init(&acc_sensor_value, &gyro_sensor_value);
	/* Create GATT service for accelerometer and gyro */
	status = ble_gatt_service_create(ble_service);
	if(BLE_SUCCESS != status)
	{
		DBG_LOG("GATT-Service creation fails = 0x%02X", status);
	}
	
	/* Start advertisement */
	ble_mgr_adv_start(adv_data, sizeof(adv_data), scan_resp_data, sizeof(scan_resp_data));
	
	/* Initialize BNO055 */
	bno055_initialize();
	
	while(true)
	{
		/* Write application task */
		memset(event_buf, 0x00, 100);
		ble_mgr_get_event(&evt_param);
		
		if(app_notify_timer_expired)
		{
			app_notify_timer_expired = false;
			
			if(acce_notify_enabled)
			{
				int8_t sta = bno055_read_accel_xyz(&accel_xyz_data);
				DBG_LOG("Status of Accelerometer sensor data read = %d", sta);
				/* Notify the accelerometer data */
				status = acce_sensor_data_send(&accel_xyz_data, remote_device_info.conn_handle);
				DBG_LOG("Acce - value send status = 0x%02X ", status);
			}
			
			if(gyro_notify_enabled)
			{
				int8_t sta = bno055_read_gyro_xyz(&gyro_xyz_data);
				DBG_LOG("Status of Gyroscope sensor data read = %d", sta);
				/* Notify the gyroscope data */
				status = gyro_sensor_data_send(&gyro_xyz_data, remote_device_info.conn_handle);
				DBG_LOG("Gyro - value send status = 0x%02X ", status);
			}
		}
	}
}

