/**
 * \file multilink_central_app.c
 *
 * \brief Multilink central application
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
 * \mainpage Multilink_Central Example
 * \section Introduction
 * **************************** Introduction *********************************
 * + The Multilink_Central Application is an example application that bring-up Bluetooth GAP-Central role and GATT-Client role. 
 * + As a Multilink-Central, it connects with multiple remote GAP-Peripheral device; 
 * + and as GATT-Client, it discovers services and characteristics in remote GATT-Server. 
 * + It enables notification in remote GATT-Server characteristics and receives accelerometer and gyroscope sensor data. 
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
#include "multilink_central_app.h"


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
/*Scan response data */
static const uint8_t scan_resp_data[] = {0x0C, 0x09, 0x53, 0x65, 0x6E, 0x73, 0x6F, 0x72, 0x2D, 0x44, 0x65, 0x6D, 0x6F};
/* Remote device information */
app_ble_remote_device_info_t remote_device_info[MAX_REMOTE_DEVICE] = {0};
/* Indicates number of remote peripheral device */
uint8_t remote_peripheral_device_count = 0;
/* Indicates number of remote central device */
uint8_t remote_central_device_count = 0;
/* Time to send notification */
bool app_notify_timer_expired = false;
/* Indicates status of accelerometer characteristic notification */
bool acce_notify_enabled = false;
/* Indicates status of gyroscope characteristic notification */
bool gyro_notify_enabled = false;
/* Sensor data send over BLE */
acc_sensor_value_t sonsor_data = {0};
/* The found_cccd_index will be used as flag as well as index */
uint8_t found_cccd_index = 0;
/* Indicates service discover is done or not */
bool service_disc_pending = 0;


static ble_status_t app_adv_report_cb(event_msg_t* msg)
{
	ble_adv_report_event_t *adv_report = (ble_adv_report_event_t *)msg->data;
		
	DBG_LOG("Adv_Report - Adv type = 0x%02X", adv_report->adv_event_type);
	DBG_LOG("Adv_Report - Addr type = 0x%02X", adv_report->addr.type);
	DBG_LOG("Adv_Report - Adv payload len = 0x%02X", adv_report->data_len);
	DBG_LOG("Adv_Report - RSSI = %d", adv_report->rssi);
		
	return BLE_SUCCESS;
}

static ble_status_t app_connected_cb(event_msg_t* msg)
{
	ble_conn_complete_event_t *connected = (ble_conn_complete_event_t *)msg->data;
	uint8_t index;
	
	if(BLE_SUCCESS == connected->status)
	{

		for(index = 0; index < MAX_REMOTE_DEVICE; index++)
		{
			if(!remote_device_info[index].in_use)
			{
				remote_device_info[index].conn_handle = connected->conn_handle;
				remote_device_info[index].peer_addr.type = connected->peer_addr.type;
				memcpy(remote_device_info[index].peer_addr.addr, connected->peer_addr.addr, BLE_ADDR_LEN);
				memcpy(&remote_device_info[index].conn_param, &connected->conn_param, sizeof(ble_conn_param_t));
				remote_device_info[index].in_use = true;
				
				/* Local device initiates a connection with remote device */
				if(APP_DEVICE_ROLE_PERIPHERAL == connected->role)
				{
					remote_device_info[index].dev_role = APP_DEVICE_ROLE_PERIPHERAL;
					remote_device_info[index].state = APP_STATE_DISCOVERY_PENDING;
					remote_peripheral_device_count++;
					service_disc_pending = true;
				}
				else
				{
					remote_device_info[index].dev_role = APP_DEVICE_ROLE_CENTRAL;
					remote_device_info[index].state = APP_STATE_READY;
					remote_central_device_count++;
					
					if(remote_central_device_count < MAX_REMOTE_CENTRAL_DEVICE)
					{
						/* Start advertisement */
						ble_status_t status = ble_adv_start();
						if(BLE_SUCCESS == status)
						{
							DBG_LOG("Adv started");
						}
						else
						{
							DBG_LOG("Advertisement start status = 0x%02X", status);
						}
					}
				}

				break;
			}
		}
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_disconnected_cb(event_msg_t* msg)
{
	ble_disconnect_complete_event_t *disconnectd = (ble_disconnect_complete_event_t *)msg->data;
	uint8_t index;
	
	DBG_LOG("Device disconnected reason = 0x%02X", disconnectd->reason);
	
	for(index = 0; index < MAX_REMOTE_DEVICE; index++)
	{
		if(remote_device_info[index].in_use && (remote_device_info[index].conn_handle == disconnectd->conn_handle))
		{
			if(remote_device_info[index].dev_role == APP_DEVICE_ROLE_PERIPHERAL)
			{
				memset(&remote_device_info[index], 0, sizeof(app_ble_remote_device_info_t));
				remote_peripheral_device_count--;
				
				/* Stop any ongoing scan */
				ble_mgr_scan_stop();
				/* Start scanning */
				ble_status_t status = ble_scan_start(BLE_SCAN_ENABLED, BLE_SCAN_DUPLICATE_FILTER_ENABLED);
				if(BLE_SUCCESS == status)
				{
					DBG_LOG("Scan started");
				}
				else
				{
					DBG_LOG("Scan start status = 0x%02X", status);
				}
				break;
			}
			else
			{
				memset(&remote_device_info[index], 0, sizeof(app_ble_remote_device_info_t));
				remote_central_device_count--;
				
				/* Start advertisement */
				ble_status_t status = ble_adv_start();
				if(BLE_SUCCESS == status)
				{
					DBG_LOG("Adv started");
				}
				else
				{
					DBG_LOG("Advertisement start status = 0x%02X", status);
				}
				break;
			}
		}
	}
	
	if(0 == remote_central_device_count)
	{
		acce_notify_enabled = false;
		gyro_notify_enabled = false;
		hw_timer_stop();
	}
	
	return BLE_SUCCESS;
}


static ble_status_t app_conn_param_update_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_char_value_write_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_service_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_char_disc_resp_cb(event_msg_t* msg)
{
	primary_service_char_discovery_resp_t *char_disc_resp = (primary_service_char_discovery_resp_t *)msg->data;
	uint8_t num_of_attrib = (msg->data_len - (SERVICE_DISC_CHAR_RESP_CONN_HANDLE_LEN + SERVICE_DISC_CHAR_RESP_LENGTH_LEN)) / char_disc_resp->length;
	uint8_t index;
	
	DBG_LOG("*** app_char_disc_resp_cb ***");
	DBG_LOG("Conn handle = 0x%02X", char_disc_resp->conn_handle);
	DBG_LOG("Attrib length = 0x%02X", char_disc_resp->length);
	DBG_LOG("num_of_attrib = 0x%02X", num_of_attrib);
	
	for(index = 0; index < MAX_REMOTE_DEVICE; index++)
	{
		if(remote_device_info[index].conn_handle == char_disc_resp->conn_handle)
		{
			if(char_disc_resp->length == 21)
			{
				for(uint8_t attrib_index = 0; attrib_index < num_of_attrib; attrib_index++)
				{
					/* Reverse the UUID in incoming parameter */
					memcpy_inplace_reorder(char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START, BLE_UUID_128B_LEN);
					for(uint8_t sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
					{
						if(BLE_UUID_128B == remote_device_info[index].sensor_service.sensor_char_list[sensor_index].uuid.type)
						{
							if(!memcmp(remote_device_info[index].sensor_service.sensor_char_list[sensor_index].uuid.uuid.uuid_128b,
							char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START,
							BLE_UUID_128B_LEN))
							{
								memcpy(&remote_device_info[index].sensor_service.sensor_char_list[sensor_index].char_attr_handle, 
									char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_ATTRIB_HANDLE_START, 5);
								break;
							}
						}
					}
				}
			}
			else if(char_disc_resp->length == 7)
			{
				for(uint8_t attrib_index = 0; attrib_index < num_of_attrib; attrib_index++)
				{
					/* Reverse the UUID in incoming parameter */
					memcpy_inplace_reorder(char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START, BLE_UUID_16B_LEN);
					for(uint8_t sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
					{
						if(BLE_UUID_16B == remote_device_info[index].sensor_service.sensor_char_list[sensor_index].uuid.type)
						{
							if(!memcmp(remote_device_info[index].sensor_service.sensor_char_list[sensor_index].uuid.uuid.uuid_16b,
							char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_CHAR_UUID_START,
							BLE_UUID_16B_LEN))
							{
								memcpy(&remote_device_info[index].sensor_service.sensor_char_list[sensor_index].char_attr_handle, 
									char_disc_resp->attrib_data + (attrib_index * char_disc_resp->length) + SERVICE_DISC_CHAR_RESP_ATTRIB_HANDLE_START, 5);
								break;
							}
						}
					}
				}
			}
		}
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_char_descriptor_disc_resp_cb(event_msg_t* msg)
{
	char_desc_discovery_resp_t *desc_discovery_resp = (char_desc_discovery_resp_t *)msg->data;
	uint8_t num_of_attrib = 0, index, sensor_index;
	uint8_t cccd_uuid[BLE_UUID_16B_LEN] = {0x02, 0x29};
	uint16_t cccd_handle = 0x0000;
	
	DBG_LOG("*** app_char_descriptor_disc_resp_cb ***");
	
	for(index = 0; index < MAX_REMOTE_DEVICE; index++)
	{
		if(remote_device_info[index].conn_handle == desc_discovery_resp->conn_handle)
		{
			if(1 == desc_discovery_resp->format)
			{
				num_of_attrib = (msg->data_len - (DESC_DISC_RESP_CONN_HANDLE_LEN + DESC_DISC_RESP_FORMAT_LEN)) / 4;
		
				for(uint8_t attrib_index = 0; attrib_index < num_of_attrib; attrib_index++)
				{
					if(!memcmp(desc_discovery_resp->desc_attrib_data + (attrib_index * 4) + DESC_DISC_RESP_UUID_START, cccd_uuid, BLE_UUID_16B_LEN))
					{
						memcpy(&cccd_handle, desc_discovery_resp->desc_attrib_data + (attrib_index * 4) + DESC_DISC_RESP_ATTRIB_HANDLE_START, 2);
				
						for(sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
						{
							if(cccd_handle == remote_device_info[index].sensor_service.sensor_char_list[sensor_index].char_value_attr_handle + 1)
							{
								remote_device_info[index].sensor_service.sensor_char_list[sensor_index].cccd_handle = cccd_handle;
								remote_device_info[index].sensor_service.sensor_char_list[sensor_index].cccd_value = 0;
								/* Increase the "found_cccd_index" value by 1. So that the "found_cccd_index" can be used 
									as flag and also used as an index. The value of "found_cccd_index" should be decreased 
									by 1 before used as index */
								found_cccd_index = index + 1;
								break;
							}
						}
					}
				}
			}
		}
	}
	
	return BLE_SUCCESS;
}

static ble_status_t app_char_value_received_cb(event_msg_t* msg)
{
	uint8_t *p_data = (uint8_t *)msg->data;
	uint8_t index, conn_handle = (uint8_t)msg->data[0];
	
	for(index = 0; index < MAX_REMOTE_DEVICE; index++)
	{
		if(remote_device_info[index].conn_handle == conn_handle)
		{
			memcpy(&sonsor_data, &p_data[1], 8);
			memcpy_inplace_reorder((uint8_t *)&sonsor_data.handle, sizeof(uint16_t));
	
			if(sonsor_data.handle == remote_device_info[index].sensor_service.sensor_char_list[SENSOR_1_ACCELEROMETER].char_value_attr_handle)
			{
				DBG_LOG("Conn_handle: 0x%02X || Accel data received X = %d || Y = %d || Z = %d", conn_handle, sonsor_data.x, sonsor_data.y, sonsor_data.z);
			}
			else if(sonsor_data.handle == remote_device_info[index].sensor_service.sensor_char_list[SENSOR_2_GYROSCOPE].char_value_attr_handle)
			{
				DBG_LOG("Conn_handle: 0x%02X || Gyro data received X = %d || Y = %d || Z = %d", conn_handle, sonsor_data.x, sonsor_data.y, sonsor_data.z);
			}
		}
	}
	
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

static ble_status_t app_adv_param_set(const uint8_t *ad_data, uint8_t ad_data_length, const uint8_t * scan_data, uint8_t scan_data_len)
{
	ble_status_t status;
	
	if(ad_data && ad_data_length)
	{
		status = ble_adv_data_set(adv_data, ad_data_length);
		if(BLE_SUCCESS != status)
		{
			DBG_LOG("Advertisement data set status = 0x%02X", status);
			return status;
		}
	}
	
	if(scan_data && scan_data_len)
	{
		status = ble_scan_resp_data_set(scan_data, scan_data_len);
		if(BLE_SUCCESS != status)
		{
			DBG_LOG("Scan response data set status = 0x%02X", status);
			return status;
		}
	}
	
	status = ble_adv_param_set(BM_ADV_CONNECTABLE_UNDIRECTED, NULL, 1600);
	if(BLE_SUCCESS != status)
	{
		DBG_LOG("Advertisement param set status = 0x%02X", status);
		return status;
	}
	
	return status;
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
	uint8_t adv_data_match[] = {0x15, 0xFF, 0xCD, 0x00, 0xFE, 0x14, 0xAD, 0x11, 0xCF, 0x40, 0x06, 0x3F, 0x11, 0xE5, 0xBE, 0x3E, 0x00, 0x02, 0xA5, 0xD5, 0xC5, 0x2C};
	ble_mgr_adv_report_match_param_t match_param;
	ble_set_conn_param_t conn_params = {
		.min_conn_interval = 0x0018,
		.max_conn_interval = 0x0018,
		.conn_latency = 0x0010,
		.link_sv_to = 0x0100
	};
	
	ble_uuid_t service_uuid = {
		.type = BLE_UUID_128B,
		.uuid.uuid_128b = {ACCE_GYRO_SENSOR_SERVICE_UUID}
	};
	ble_uuid_t acce_char_uuid = {
		.type = BLE_UUID_128B,
		.uuid.uuid_128b = {ACCE_UUID}
	};
	ble_uuid_t gyro_char_uuid = {
		.type = BLE_UUID_128B,
		.uuid.uuid_128b = {GYRO_UUID}
	};
	
	/* Initialize system */
	system_init();
	/* Hardware timer */
	hw_timer_init();
	hw_timer_register_callback(timer_callback_fn);
	/* Initialize serial console */
	serial_console_init();
	/* Initialize button */
	button_init();
	
	DBG_LOG("Initializing BLE Application");
	ble_mgr_device_init();
	app_init();
	
	/* Set GAP connection parameters */
	status = ble_set_connection_params(&conn_params);
	if(BLE_SUCCESS != status)
	{
		DBG_LOG("ble_set_connection_params->Status = 0x%02X", status);
	}
	
	/* Set advertisement parameters */
	app_adv_param_set(adv_data, sizeof(adv_data), scan_resp_data, sizeof(scan_resp_data));
	
	/*******   Central Init   *******/
	/* Set a expected matching parameters from remote device, to connect with */
	match_param.matching_param = BLE_MATCHING_PARAM_PAYLOAD;
	match_param.data_start = 0x03;
	match_param.data_len = sizeof(adv_data_match);
	memcpy(match_param.data, adv_data_match, sizeof(adv_data_match));
	ble_mgr_peripheral_device_match_params(&match_param);
	
	#if(MAX_REMOTE_PERIPHERAL_DEVICE)
		/* Start scanning */
		ble_mgr_scan_start();
	#elif(MAX_REMOTE_CENTRAL_DEVICE)
		/* Start advertisement */
		ble_adv_start();
	#endif
	
	while(true)
	{
		/* Write application task */
		memset(event_buf, 0x00, 100);
		ble_mgr_get_event(&evt_param);
		
		if(service_disc_pending)
		{
			uint8_t index;
			for(index = 0; index < MAX_REMOTE_DEVICE; index++)
			{
				if(remote_device_info[index].state == APP_STATE_DISCOVERY_PENDING)
				{
					/* Update the service table with expected service and characteristic UUID */
					remote_device_info[index].sensor_service.service_uuid.type = BLE_UUID_128B;
					memcpy(remote_device_info[index].sensor_service.service_uuid.uuid.uuid_128b, service_uuid.uuid.uuid_16b, BLE_UUID_128B_LEN);
					remote_device_info[index].sensor_service.sensor_char_list[0].uuid.type = BLE_UUID_128B;
					memcpy(remote_device_info[index].sensor_service.sensor_char_list[0].uuid.uuid.uuid_128b, acce_char_uuid.uuid.uuid_16b, BLE_UUID_128B_LEN);
					remote_device_info[index].sensor_service.sensor_char_list[1].uuid.type = BLE_UUID_128B;
					memcpy(remote_device_info[index].sensor_service.sensor_char_list[1].uuid.uuid.uuid_128b, gyro_char_uuid.uuid.uuid_16b, BLE_UUID_128B_LEN);
					
					status = ble_primary_service_characteristics_discover(remote_device_info[index].conn_handle, &service_uuid);
					
					if(BLE_SUCCESS == status)
					{
						remote_device_info[index].state = APP_STATE_READY;
						service_disc_pending = false;
					}
					else
					{
						DBG_LOG("***Discover characteristics in sensor service*** = 0x%02X", status);
					}
				}
			}
		}
		
		if(found_cccd_index)
		{
			uint8_t sensor_index;
			
			/* Revert back to original index value by subtracting 1 */
			found_cccd_index--;
			
			for(sensor_index = 0; sensor_index < NUMBER_OF_SENSORS; sensor_index++)
			{
				if(remote_device_info[found_cccd_index].sensor_service.sensor_char_list[sensor_index].cccd_handle && 
					(remote_device_info[found_cccd_index].sensor_service.sensor_char_list[sensor_index].cccd_value == 0))
				{
					ble_handle_t cccd_handle = remote_device_info[found_cccd_index].sensor_service.sensor_char_list[sensor_index].cccd_handle;
					
					remote_device_info[found_cccd_index].sensor_service.sensor_char_list[sensor_index].cccd_value = 0x0001;
					status = ble_mgr_characteristic_notify_set(remote_device_info[found_cccd_index].conn_handle, cccd_handle, true);
					
					if(sensor_index == SENSOR_1_ACCELEROMETER)
					{
						DBG_LOG("***Accelerometer notification enabled status*** = 0x%02X", status);
					}
					else if(sensor_index == SENSOR_2_GYROSCOPE)
					{
						DBG_LOG("***Gyroscope notification enabled status*** = 0x%02X", status);
					}
					break;
				}
			}
			
			found_cccd_index = 0;
			
			/* Start scan, if number of remote peripheral device limit is not reached */
			if(remote_peripheral_device_count < MAX_REMOTE_PERIPHERAL_DEVICE)
			{
				/* Stop any ongoing scan */
				ble_mgr_scan_stop();
				/* Start scanning */
				status = ble_scan_start(BLE_SCAN_ENABLED, BLE_SCAN_DUPLICATE_FILTER_ENABLED);
				
				if(BLE_SUCCESS == status)
				{
					DBG_LOG("Scan started");
				}
				else
				{
					DBG_LOG("Scan start status = 0x%02X", status);
				}
			}
			/* This device will start advertise as peripheral, only after reaching max connection as central */
			else if(remote_central_device_count < MAX_REMOTE_CENTRAL_DEVICE)
			{
				ble_mgr_adv_stop();
				/* Start advertisement, if number of remote peripheral device limit is not reached */
				status = ble_adv_start();
				if(BLE_SUCCESS == status)
				{
				DBG_LOG("Adv started");
				}
				else
				{
				DBG_LOG("Advertisement start status = 0x%02X", status);
				}
			}
		}
	}
}

