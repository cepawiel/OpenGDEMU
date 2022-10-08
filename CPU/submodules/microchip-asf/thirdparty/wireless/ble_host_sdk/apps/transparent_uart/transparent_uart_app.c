/**
 * \file transparent_uart_app.c
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
/**
 * \mainpage TransparentUART Example
 * \section Introduction
 * **************************** Introduction *********************************
 * + The TransparentUART Application is an example application that opens up serial pipe between remote device and sends out stream of data. 
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
#include "sio2host.h"
#include "platform.h"
#include "ble_manager.h"
#include "transparent_uart_app.h"


static void app_init(void);
static void app_send_transparent_uart_data(void);
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

/* Event buffer */
uint8_t event_buf[EVENT_BUFFER_LENGTH];
/* Event parameter */
event_t evt_param = {.event_msg.data = event_buf, .event_msg.data_len = 0, .event_id = 0};
/* Connection handle */
static uint8_t app_conn_handle;
/* Advertisement data */
static const uint8_t adv_data[]	= {	0x02,ADV_FLAGS,0x05,
	0x14, ADV_COMPLETE_NAME, 'T', 'r', 'a', 'n', 's', 'p', 'a', 'r', 'e', 'n', 't', 'U', 'A', 'R', 'T', 'D', 'e', 'm', 'o',
};

static ble_status_t app_adv_report_cb(event_msg_t* msg)
{
	return BLE_SUCCESS;
}

static ble_status_t app_connected_cb(event_msg_t* msg)
{
	ble_conn_complete_event_t *connected = (ble_conn_complete_event_t*)msg->data;
	
	app_conn_handle = connected->conn_handle;
	/* Enable transparentUART with remote device */
	ble_mgr_transparent_uart_enable(app_conn_handle);
	return BLE_SUCCESS;
}

static ble_status_t app_disconnected_cb(event_msg_t* msg)
{
	ble_disconnect_complete_event_t *disconnectd = (ble_disconnect_complete_event_t *)msg->data;
	
	DBG_LOG("Device disconnected reason = 0x%02X", disconnectd->reason);
	ble_mgr_transparent_uart_disable(app_conn_handle);
	ble_mgr_adv_start(adv_data, sizeof(adv_data), NULL, 0);
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
	ble_transparent_uart_data_received_event_t *transparentUART_data = (ble_transparent_uart_data_received_event_t *)msg->data;
	uint8_t data_len = msg->data_len - 1; /* total length - connection handle length */
	uint8_t index;
	
	DBG_LOG("app_trans_data_received_cb: ");
	for(index = 0; index < data_len; index++)
	{
		DBG_LOG_CONT("%c", transparentUART_data->data[index]);
	}
	DBG_LOG_CONT("\r\n");
	
	return BLE_SUCCESS;
}

void button_cb(void)
{
	/* Add button callback functionality here */
}

/* Function used for send data */
static void app_send_transparent_uart_data(void)
{
	uint16_t len = 0;
	uint8_t buff = 0;
	
	len = sio2host_rx(&buff, 1);
	if(len)
	{
		ble_mgr_transparent_uart_send(app_conn_handle, &buff, 1);
	}
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
	/* Initialize system */
	system_init();
	/* Initialize button */
	button_init();
	/* Initialize serial console */
	sio2host_init();
		
	DBG_LOG("Initializing BLE Application");
	/* Initialize the platform and BLE interface */
	ble_mgr_device_init();
	/* Initialize BLE event callbacks for application */
	app_init();
	/* Initialize and start advertisement */
	ble_mgr_adv_start(adv_data, sizeof(adv_data), NULL, 0);
	
	while(true)
	{
		/* Write application task */
		memset(event_buf, 0x00, 100);
		ble_mgr_get_event(&evt_param);
		app_send_transparent_uart_data();
	}
}

