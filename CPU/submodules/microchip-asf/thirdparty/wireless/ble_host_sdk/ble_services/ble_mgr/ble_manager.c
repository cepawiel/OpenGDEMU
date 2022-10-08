/**
 * \file ble_manager.c
 *
 * \brief BLE manager
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
#include "event_mem.h"
#include "bm_application_mode.h"
#include "ble_api.h"
#include "platform.h"
#include "ble_manager.h"

event_t event_mem[MAX_EVENT_SIZE];
uint32_t event_payload[MAX_EVENT_PAYLOAD_SIZE/4];
ble_mgr_state_t ble_mgr_state = BLE_MGR_STATE_INIT;

static void ble_mgr_init(void);
static void ble_mgr_event_manager(event_t *event_param);
static ble_status_t ble_mgr_adv_report_cb(event_msg_t* msg);
static ble_status_t ble_mgr_connected_cb(event_msg_t* msg);
static ble_status_t ble_mgr_disconnected_cb(event_msg_t* msg);
static ble_status_t ble_mgr_conn_param_update_cb(event_msg_t* msg);
static ble_status_t ble_mgr_char_value_write_cb(event_msg_t* msg);
static ble_status_t ble_mgr_prepare_write_request_cb(event_msg_t* msg);
static ble_status_t ble_mgr_execute_write_request_cb(event_msg_t* msg);
static ble_status_t ble_mgr_service_disc_resp_cb(event_msg_t* msg);
static ble_status_t ble_mgr_char_disc_resp_cb(event_msg_t* msg);
static ble_status_t ble_mgr_char_descriptor_disc_resp_cb(event_msg_t* msg);
static ble_status_t ble_mgr_char_value_received_cb(event_msg_t* msg);
static ble_status_t ble_mgr_prepare_write_repose_cb(event_msg_t* msg);
static ble_status_t ble_mgr_execute_write_repose_cb(event_msg_t* msg);
static ble_status_t ble_mgr_cmd_complete_cb(event_msg_t* msg);
static ble_status_t ble_mgr_status_report_cb(event_msg_t* msg);
static ble_status_t ble_mgr_le_end_test_result_cb(event_msg_t* msg);
static ble_status_t ble_mgr_config_mode_status_cb(event_msg_t* msg);
static ble_status_t ble_mgr_passkey_entry_req_cb(event_msg_t* msg);
static ble_status_t ble_mgr_pairing_complete_cb(event_msg_t* msg);
static ble_status_t ble_mgr_passkey_confirm_req_cb(event_msg_t* msg);
static ble_status_t ble_mgr_trans_data_received_cb(event_msg_t* msg);

static ble_mgr_adv_report_event_t ble_mgr_adv_report_list = {0};
static ble_mgr_adv_report_match_param_t ble_mgr_adv_report_match_param = {0};

platform_init_t pf_cfg = {
	/* Register Platform callback API's */
	.platform_api_list.create_timer = platform_create_timer,
	.platform_api_list.delete_timer = platform_delete_timer,
	.platform_api_list.start_timer = platform_start_timer,
	.platform_api_list.stop_timer = platform_stop_timer,
	.platform_api_list.sleep_timer_ms = platform_sleep,
	.platform_api_list.gpio_set = platform_gpio_set,
	.platform_api_list.mode_set = platform_mode_set,
    .platform_api_list.mode_get = platform_mode_get,
	.platform_api_list.uart_tx_cb = platform_send_sync,
	.platform_api_list.uart_rx_cb = platform_recv_async,
	.event_mem.event_mem_pool = event_mem,
	.event_mem.event_mem_pool_size = MAX_EVENT_SIZE,
	.event_mem.event_payload_mem = event_payload,
	.event_mem.event_payload_mem_size = MAX_EVENT_PAYLOAD_SIZE,
};

ble_gap_event_cb_t *ble_mgr_gap_event_cb[MAX_GAP_EVENT_SUBSCRIBERS] = {NULL, };
ble_gatt_server_event_cb_t *ble_mgr_gatt_server_event_cb[MAX_GATT_SERVER_EVENT_SUBSCRIBERS] = {NULL, };
ble_gatt_client_event_cb_t *ble_mgr_gatt_client_event_cb[MAX_GATT_CLIENT_EVENT_SUBSCRIBERS] = {NULL, };
ble_common_event_cb_t *ble_mgr_common_event_cb[MAX_COMMON_EVENT_SUBSCRIBERS] = {NULL, };
ble_pairing_event_cb_t *ble_mgr_pairing_event_cb[MAX_PAIRING_SUBSCRIBERS] = {NULL, };
ble_gatt_transparent_event_cb_t *ble_mgr_gatt_tp_event_cb[MAX_GATT_TRANSPARENT_SUBSCRIBERS] = {NULL, };


static const ble_gap_event_cb_t ble_mgr_gap_event_handle = {
	.adv_report = ble_mgr_adv_report_cb,
	.connected = ble_mgr_connected_cb,
	.disconnected = ble_mgr_disconnected_cb,
	.conn_param_update = ble_mgr_conn_param_update_cb,
};

static const ble_gatt_server_event_cb_t ble_mgr_gatt_server_event_handle = {
	.char_value_write = ble_mgr_char_value_write_cb,
    .prepare_write_request = ble_mgr_prepare_write_request_cb,
    .execute_write_request = ble_mgr_execute_write_request_cb,
};

static const ble_gatt_client_event_cb_t ble_mgr_gatt_client_event_handle = {
	.service_disc_resp = ble_mgr_service_disc_resp_cb,
	.char_disc_resp = ble_mgr_char_disc_resp_cb,
	.char_descriptor_disc_resp = ble_mgr_char_descriptor_disc_resp_cb,
	.char_value_received = ble_mgr_char_value_received_cb,
    .prepare_write_response = ble_mgr_prepare_write_repose_cb,
    .execute_write_response = ble_mgr_execute_write_repose_cb,
};

static const ble_common_event_cb_t ble_mgr_common_event_handle = {
	.cmd_complete = ble_mgr_cmd_complete_cb,
	.status_report = ble_mgr_status_report_cb,
	.le_end_test_result = ble_mgr_le_end_test_result_cb,
	.config_mode_status = ble_mgr_config_mode_status_cb,
};

static const ble_pairing_event_cb_t ble_mgr_pairing_event_handle = {
	.passkey_entry_req = ble_mgr_passkey_entry_req_cb,
	.pairing_complete = ble_mgr_pairing_complete_cb,
	.passkey_confirm_req = ble_mgr_passkey_confirm_req_cb,
};

static const ble_gatt_transparent_event_cb_t ble_mgr_transparent_event_handle = {
	.trans_data_received = ble_mgr_trans_data_received_cb,
};

/*! \fn bool ble_mgr_events_register_callback(ble_mgr_event_t event_type, const void *ble_event_handler)
 *  \brief Register callback functions for BLE events.
 *  \param event_type Type of event, like GAP, GATT-Client, GATT-Server... etc.
 *  \param ble_event_handler Function pointer to group of event handler callbacks.
 *  \param scan_resp_data Scan response data.
 *  \param scan_reap_data_len Scan response data length.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of setting advertisement data, scan response data, advertisement parameters or start advertisement operation.
 */
bool ble_mgr_events_register_callback(ble_mgr_event_t event_type, const void *ble_event_handler)
{
	uint8_t index;
	bool status = false;
	
	if(ble_event_handler != NULL)
	{
		switch(event_type)
		{
			case BLE_GAP_EVENT_TYPE:
			{
				ble_gap_event_cb_t *ble_gap_event_cb = (ble_gap_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_GAP_EVENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_gap_event_cb[index])
					{
						ble_mgr_gap_event_cb[index] = ble_gap_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_GATT_SERVER_EVENT_TYPE:
			{
				ble_gatt_server_event_cb_t *ble_gatt_server_event_cb = (ble_gatt_server_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_GATT_SERVER_EVENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_gatt_server_event_cb[index])
					{
						ble_mgr_gatt_server_event_cb[index] = ble_gatt_server_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_GATT_CLIENT_EVENT_TYPE:
			{
				ble_gatt_client_event_cb_t *ble_gatt_client_event_cb = (ble_gatt_client_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_GATT_CLIENT_EVENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_gatt_client_event_cb[index])
					{
						ble_mgr_gatt_client_event_cb[index] = ble_gatt_client_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_COMMON_EVENT_TYPE:
			{
				ble_common_event_cb_t *ble_common_event_cb = (ble_common_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_COMMON_EVENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_common_event_cb[index])
					{
						ble_mgr_common_event_cb[index] = ble_common_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_PAIRING_EVENT_TYPE:
			{
				ble_pairing_event_cb_t *ble_pairing_event_cb = (ble_pairing_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_PAIRING_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_pairing_event_cb[index])
					{
						ble_mgr_pairing_event_cb[index] = ble_pairing_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			case BLE_GATT_TP_EVENT_TYPE:
			{
				ble_gatt_transparent_event_cb_t *ble_transparent_event_cb = (ble_gatt_transparent_event_cb_t *)ble_event_handler;
				
				for(index = 0; index < MAX_GATT_TRANSPARENT_SUBSCRIBERS; index++)
				{
					if(NULL == ble_mgr_gatt_tp_event_cb[index])
					{
						ble_mgr_gatt_tp_event_cb[index] = ble_transparent_event_cb;
						status = true;
						break;
					}
				}
				break;
			}
			default:
				break;
		}
	}
	
	return status;
}

static void ble_mgr_event_manager(event_t *event_param)
{
	switch(event_param->event_id)
	{
		case BM_ADVERTISING_REPORT:
		case BM_LE_CONNECT_COMPLETE:
		case BM_DISCONNECT_COMPLETE:
		case BM_CONNECTION_PARAMTER_UPDATE:
		{
			/* GAP events */
			uint8_t index;
			
			event_param->event_id -= BM_ADVERTISING_REPORT;
			
			for(index = 0; index < MAX_GAP_EVENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_gap_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_gap_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_SERVER_CHARACTERISTIC_VALUE_READ:
		case BM_SERVER_BLOB_READ_REQUEST:
		{
			break;
		}
		case BM_SERVER_CHARACTERICTIC_VALUE_WRITE:
        case BM_SERVER_PREPARE_WRITE_REQUEST:
        case BM_SERVER_EXECUTE_WRITE_REQUEST:
		{
			/* GATT Server events */
			uint8_t index;
			
			event_param->event_id -= BM_SERVER_CHARACTERICTIC_VALUE_WRITE;
			
			for(index = 0; index < MAX_GATT_SERVER_EVENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_gatt_server_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_gatt_server_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_CLIENT_DISCOVER_ALL_SERVICES_RESULT:
		case BM_CLIENT_DISCOVER_CHARACTERISTICS_RESULT:
		case BM_CLIENT_DISCOVER_CHARACTERISTICS_DESCRIPTORS_RESULT:
		case BM_CLIENT_CHARACTERISTIC_VALUE_RECEIVED:
        case BM_CLIENT_PREPARE_WRITE_RESPONSE:
        case BM_CLIENT_EXECUTE_WRITE_RESPONSE:
		{
			/* GATT Client events */
			uint8_t index;
			
			event_param->event_id -= BM_CLIENT_DISCOVER_ALL_SERVICES_RESULT;
			
			for(index = 0; index < MAX_GATT_CLIENT_EVENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_gatt_client_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_gatt_client_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_COMMAND_COMPLETE:
		case BM_STATUS_REPORT:
		case BM_LE_END_TEST_RESULT:
		case BM_CONFIGURE_MODE_STATUS:
		{
			/* Common events */
			uint8_t index;
			
			event_param->event_id -= BM_COMMAND_COMPLETE;
			
			for(index = 0; index < MAX_COMMON_EVENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_common_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_common_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_PASSKEY_REQUEST:
		case BM_PAIR_COMPLETE:
		case BM_PASSKEY_YESNO_REQUEST:
		{
			/* Pairing events */
			uint8_t index;
			
			event_param->event_id -= BM_PASSKEY_REQUEST;
			
			for(index = 0; index < MAX_PAIRING_SUBSCRIBERS; index++)
			{
				if(ble_mgr_pairing_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_pairing_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_TRANSPARENT_DATA_RECEIVED:
		{
			/* GATT Transparent events */
			uint8_t index;
			
			event_param->event_id -= BM_TRANSPARENT_DATA_RECEIVED;
			
			for(index = 0; index < MAX_GATT_TRANSPARENT_SUBSCRIBERS; index++)
			{
				if(ble_mgr_gatt_tp_event_cb[index] != NULL)
				{
					const ble_event_callback_t *event_cb_fn = (ble_event_callback_t *)ble_mgr_gatt_tp_event_cb[index];
					
					if(event_cb_fn[event_param->event_id] != NULL)
					{
						event_cb_fn[event_param->event_id](&event_param->event_msg);
					}
				}
			}
			break;
		}
		case BM_EVENT_NONE:
		case BM_ERROR:
		{
			/* Just to avoid compiler error */
			break;
		}
	}
}

/*! \fn ble_status_t ble_mgr_device_init(void)
 *  \brief Initialize platform, callback events and BLE interface.
 *  \param None.
 *  \pre None.
 *  \return ble_status_t Status of the BLE interface initialization.
 */
ble_status_t ble_mgr_device_init(void)
{
	ble_status_t status = BLE_SUCCESS;
	
	platform_init();
	ble_mgr_init();
    /* Initialize mode pin */
    BM_MODE_Init();
	
	/* Initialize platform */
	status = ble_init(&pf_cfg);
	
	if(BLE_SUCCESS == status)
	{
		DBG_LOG("BLE init succeed");
	}
	else
	{
		DBG_LOG("BLE init failed");
	}
	
	return status;
}

/*! \fn ble_status_t ble_mgr_adv_start(const uint8_t *adv_data, uint8_t adv_data_length, const uint8_t * scan_resp_data, uint8_t scan_reap_data_len)
 *  \brief Set advertisement data, scan response data, advertisement parameters and start advertisement.
 *  \param adv_data Advertisement data.
 *  \param adv_data_length Advertisement data length.
 *  \param scan_resp_data Scan response data.
 *  \param scan_reap_data_len Scan response data length.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of setting advertisement data, scan response data, advertisement parameters or start advertisement operation.
 */
ble_status_t ble_mgr_adv_start(const uint8_t *adv_data, uint8_t adv_data_length, const uint8_t * scan_resp_data, uint8_t scan_reap_data_len)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	/* Advertisement data is mandatory */
	if(adv_data && adv_data_length)
	{
		status = ble_adv_data_set(adv_data, adv_data_length);
		if(BLE_SUCCESS != status)
		{
			DBG_LOG("Advertisement data set status = 0x%02X", status);
			return status;
		}
	}
	else
	{
		return status;
	}
	
	/* Scan response data is optional */
	if(scan_resp_data && scan_reap_data_len)
	{
		status = ble_scan_resp_data_set(scan_resp_data, scan_reap_data_len);
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
	
	status = ble_adv_start();
	if(BLE_SUCCESS == status)
	{
		ble_mgr_state = BLE_MGR_STATE_ADVERTISING;
		DBG_LOG("Adv started");
	}
	else
	{
		DBG_LOG("Advertisement start status = 0x%02X", status);
	}
	
	return status;
}

/*! \fn ble_status_t ble_mgr_adv_stop(void)
 *  \brief Stops ongoing advertisement.
 *  \param None.
 *  \pre Advertisement has to be started already.
 *  \return ble_status_t Status of stop advertisement operation.
 */
ble_status_t ble_mgr_adv_stop(void)
{
	ble_status_t status;
	
	status = ble_adv_stop();
	if(BLE_SUCCESS == status)
	{
		DBG_LOG("Adv stopped");
	}
	else
	{
		DBG_LOG("Adv stop status = 0x%02X", status);
	}
	
	return status;
}

/*! \fn ble_status_t ble_mgr_scan_start(void)
 *  \brief Sets BLE scan parameters and starts scanning.
 *  \param None.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of the setting scan parameter or scan start operation.
 */
ble_status_t ble_mgr_scan_start(void)
{
	ble_status_t status;
	
	status = ble_scan_param_set(DEFAULT_SCAN_INTERVAL, DEFAULT_SCAN_WINDOW, BLE_SCAN_ACTIVE);
	
	if(BLE_SUCCESS != status)
	{
		DBG_LOG("Scan parameter set status = 0x%02X", status);
		return status;
	}
	
	status = ble_scan_start(BLE_SCAN_ENABLED, BLE_SCAN_DUPLICATE_FILTER_ENABLED);
	
	if(BLE_SUCCESS == status)
	{
		ble_mgr_state = BLE_MGR_STATE_SCANNING;
		DBG_LOG("Scan started");
	}
	else
	{
		DBG_LOG("Scan start status = 0x%02X", status);
	}
	
	return status;
}

/*! \fn ble_status_t ble_mgr_scan_stop(void)
 *  \brief Stops the ongoing BLE scan.
 *  \param None.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of the scan stop operation.
 */
ble_status_t ble_mgr_scan_stop(void)
{
	ble_status_t status = ble_scan_start(BLE_SCAN_DISABLED, BLE_SCAN_DUPLICATE_FILTER_ENABLED);
	
	if(BLE_SUCCESS == status)
	{
		ble_mgr_state = BLE_MGR_STATE_INIT;
		DBG_LOG("Scan stopped");
	}
	else
	{
		DBG_LOG("Scan stop status = 0x%02X", status);
	}
	
	return status;
}

/*! \fn ble_status_t ble_mgr_start_connection(ble_addr_t* address)
 *  \brief Initiate a connection based on remote device address.
 *  \param address Address of remote device.
 *  \pre None.
 *  \return bool Status of connection request.
 */
ble_status_t ble_mgr_start_connection(ble_addr_t* address)
{
    ble_status_t status = ble_create_connection(BLE_CONN_WHITELIST_FILTER_DISABLED, address);
    if(BLE_SUCCESS == status)
	{
		DBG_LOG("Connect initiated");
	}
	else
	{
		DBG_LOG("Connect failed");
	}
	
	return status;
}

ble_status_t ble_mgr_characteristic_notify_set(ble_handle_t conn_handle, uint16_t desc_handle, bool enabled)
{
    uint16_t notify_val = BLE_GATT_NONE;

    if(enabled)
    {
        notify_val = BLE_GATT_NOTIFY;
    }
    ble_status_t status = ble_characteristic_write_with_response(conn_handle, desc_handle, (uint8_t*)&notify_val, 2);
	return status;
}

ble_status_t ble_mgr_characteristic_indicate_set(ble_handle_t conn_handle, uint16_t desc_handle, bool enabled)
{
    uint16_t indicate_val = BLE_GATT_NONE;

    if(enabled)
    {
        indicate_val = BLE_GATT_INDICATE;
    }
    ble_status_t status = ble_characteristic_write_with_response(conn_handle, desc_handle, (uint8_t*)&indicate_val, 2);
	return status;
}


/*! \fn ble_status_t ble_mgr_transparent_uart_enable(ble_handle_t conn_handle)
 *  \brief Enables transparentUART with remote device.
 *  \param conn_handle Connection handle of remote device.
 *  \pre Connection is established with remote device.
 *  \return ble_status_t Status of transparentUART enable operation.
 */
ble_status_t ble_mgr_transparent_uart_enable(ble_handle_t conn_handle)
{
	ble_enable_transparent_uart_t enable_transparent_uart = {
		.conn_handle = conn_handle,
		.server_transparent_cntrl = 0x01,
		.client_transparent_mode = 0x00};
	ble_status_t status = ble_transparent_uart_enable(&enable_transparent_uart);
	if(BLE_SUCCESS == status)
	{
		DBG_LOG("Enable Transparent UART succeed");
	}
	else
	{
		DBG_LOG("Enable Transparent UART failed");
	}
	
	return status;
}

/*! \fn ble_status_t ble_mgr_transparent_uart_disable(ble_handle_t conn_handle)
 *  \brief Disables transparentUART with remote device.
 *  \param conn_handle Connection handle of remote device.
 *  \pre Connection is established with remote device and transparentUART is enabled.
 *  \return ble_status_t Status of transparentUART enable operation.
 */
ble_status_t ble_mgr_transparent_uart_disable(ble_handle_t conn_handle)
{
	ble_enable_transparent_uart_t enable_transparent_uart = {
		.conn_handle = conn_handle,
		.server_transparent_cntrl = 0x00,
		.client_transparent_mode = 0x00};
	ble_status_t status = ble_transparent_uart_enable(&enable_transparent_uart);
	if(BLE_SUCCESS == status)
	{
		DBG_LOG("Disable Transparent UART succeed");
	}
	else
	{
		DBG_LOG("Disable Transparent UART failed");
	}
	
	return status;
}

/*! \fn ble_status_t ble_mgr_transparent_uart_send(ble_handle_t conn_handle, const uint8_t *data, uint8_t datalen)
 *  \brief Sends transparentUART data to remote device.
 *  \param conn_handle Connection handle of remote device.
 *  \param data Data that needs to be sent to remote device.
 *  \param datalen Length of data that needs to be sent to remote device.
 *  \pre TransparentUART is enabled with remote device using ble_mgr_transparent_uart_enable.
 *  \return ble_status_t Status of transparentUART send data operation.
 */
ble_status_t ble_mgr_transparent_uart_send(ble_handle_t conn_handle, const uint8_t *data, uint8_t datalen)
{
	ble_status_t status = ble_transparent_uart_data_send(conn_handle, (uint8_t *)data, datalen);
	if(BLE_SUCCESS == status)
	{
		/*Print the data back on console */
		DBG_LOG_CONT("%c", *data);
	}
	else
	{
		DBG_LOG("Transparent UART send failed");
	}
	
	return status;
}

/*! \fn void ble_mgr_peripheral_device_match_params(ble_mgr_adv_report_match_param_t *match_param)
 *  \brief Sets a matching parameter to find and connect with the device.
 *  \param match_param Matching parameter, it could be BLE address, RSSI threshold or advertisement payload.
 *  \pre None.
 *  \return None.
 */
void ble_mgr_peripheral_device_match_params(ble_mgr_adv_report_match_param_t *match_param)
{
	memset(&ble_mgr_adv_report_match_param, 0, sizeof(ble_mgr_adv_report_match_param_t));
	memcpy(&ble_mgr_adv_report_match_param, match_param, sizeof(ble_mgr_adv_report_match_param_t));
}

/*! \fn bool ble_mgr_check_match_param(ble_mgr_adv_report_event_t *adv_report)
 *  \brief Check against the matching parameter set by the user, it could be BLE address, RSSI threshold or advertisement payload.
 *  \param adv_report Advertisement report info got from scanning operation.
 *  \pre Peripheral matching parameter has to be set using ble_mgr_peripheral_device_match_params.
 *  \return bool Status of checking against matching parameter.
 */
bool ble_mgr_check_match_param(ble_mgr_adv_report_event_t *adv_report)
{
	bool match = false;
	
	switch(ble_mgr_adv_report_match_param.matching_param)
	{
		case BLE_MATCHING_PARAM_ADDRESS:
		{
			if(!memcmp(&ble_mgr_adv_report_match_param.addr, &adv_report->addr, sizeof(ble_addr_t)))
			{
				match = true;
			}
			break;
		}
		case BLE_MATCHING_PARAM_PAYLOAD:
		{
			if(!memcmp(ble_mgr_adv_report_match_param.data, adv_report->data + ble_mgr_adv_report_match_param.data_start, ble_mgr_adv_report_match_param.data_len))
			{
				match = true;
			}
			break;
		}
		case BLE_MATCHING_PARAM_RSSI:
		{
			if(ble_mgr_adv_report_match_param.rssi >= adv_report->rssi)
			{
				match = true;
			}
			break;
		}
		default:
			return match;
	}
	
	return match;
}

/*! \fn event_status_t ble_mgr_get_event(event_t *event)
 *  \brief Get BLE events and process them.
 *  \param event Carries event ID, parameters and length.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return event_status_t Status of event get operation.
 */
event_status_t ble_mgr_get_event(event_t *event)
{
	event_status_t status = ble_event_get(event); 
    
	if(BLE_EVENT_SUCCESS == status)
	{
		ble_mgr_event_manager(event);
	}
	
	return status;
}

/*! \fn void ble_mgr_init(void)
 *  \brief Registers BLE manager event callbacks.
 *  \param None.
 *  \pre BLE manager event callbacks have to be defined in initialized.
 *  \return None.
 */
static void ble_mgr_init(void)
{
	/* BLE event callback registration */
	ble_mgr_events_register_callback(BLE_GAP_EVENT_TYPE, &ble_mgr_gap_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_SERVER_EVENT_TYPE, &ble_mgr_gatt_server_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_CLIENT_EVENT_TYPE, &ble_mgr_gatt_client_event_handle);
	ble_mgr_events_register_callback(BLE_COMMON_EVENT_TYPE, &ble_mgr_common_event_handle);
	ble_mgr_events_register_callback(BLE_PAIRING_EVENT_TYPE, &ble_mgr_pairing_event_handle);
	ble_mgr_events_register_callback(BLE_GATT_TP_EVENT_TYPE, &ble_mgr_transparent_event_handle);
}

static ble_status_t ble_mgr_adv_report_cb(event_msg_t* msg)
{
	ble_adv_report_event_t *adv_report = (ble_adv_report_event_t *)msg->data;
	bool match = false;
	
	memcpy(&ble_mgr_adv_report_list.addr, &adv_report->addr, sizeof(ble_addr_t));
	ble_mgr_adv_report_list.adv_event_type = adv_report->adv_event_type;
	ble_mgr_adv_report_list.rssi = adv_report->rssi;
	ble_mgr_adv_report_list.data_len = adv_report->data_len;
	memcpy(ble_mgr_adv_report_list.data, adv_report->data, adv_report->data_len);
		
	if(ble_mgr_adv_report_match_param.matching_param != BLE_MATCHING_PARAM_NONE)
	{
		if((adv_report->adv_event_type == ADV_IND) || (adv_report->adv_event_type == ADV_DIRECT_IND))
		{
			match = ble_mgr_check_match_param(&ble_mgr_adv_report_list);
			
			if(match)
			{
				ble_mgr_state = BLE_MGRSTATE_CONNECTING;
				ble_create_connection(BLE_CONN_WHITELIST_FILTER_DISABLED, &ble_mgr_adv_report_list.addr);
				return match;
			}	
		}
	}
		
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_connected_cb(event_msg_t* msg)
{
	ble_conn_complete_event_t *connected = (ble_conn_complete_event_t *) msg->data;
	uint8_t index;
    
	if(BLE_SUCCESS == connected->status)
	{
		ble_mgr_state = BLE_MGR_STATE_CONNECTED;
	}
	else
	{
		ble_mgr_state = BLE_MGR_STATE_INIT;
	}
	
	DBG_LOG("Device connected");
	DBG_LOG("Status = 0x%02X", connected->status);
	DBG_LOG("Conn_handle = 0x%02X", connected->conn_handle);
	DBG_LOG("Conn_interval = 0x%04X", connected->conn_param.conn_interval);
	DBG_LOG("Conn_latency = 0x%04X", connected->conn_param.conn_latency);
	DBG_LOG("Conn_sv_timeout = 0x%04X", connected->conn_param.link_sv_to);
	DBG_LOG("Remote device Address :");
	for(index = 0; index < BLE_ADDR_LEN; index++)
	{
		DBG_LOG_CONT(" 0x%02X", connected->peer_addr.addr[index]);
	}
	
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_disconnected_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_conn_param_update_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_char_value_write_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_prepare_write_request_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_execute_write_request_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_service_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_char_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_char_descriptor_disc_resp_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_char_value_received_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_prepare_write_repose_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_execute_write_repose_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_cmd_complete_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_status_report_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_le_end_test_result_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_config_mode_status_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_passkey_entry_req_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_pairing_complete_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_passkey_confirm_req_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t ble_mgr_trans_data_received_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}
