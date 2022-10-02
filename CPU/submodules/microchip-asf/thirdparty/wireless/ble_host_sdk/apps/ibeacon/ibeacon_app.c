/**
 * \file ibeacon_app.c
 *
 * \brief iBeacon application
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
 * \mainpage iBeacon Example
 * \section Introduction
 * **************************** Introduction *********************************
 * + The iBeacon Application is an example application that sends out iBeacon periodically. 
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
#include "ibeacon_app.h"

static void app_init(void);
static ble_status_t app_adv_report_cb(event_msg_t* msg);
static ble_status_t app_cmd_complete_cb(event_msg_t* msg);
static ble_status_t app_status_report_cb(event_msg_t* msg);
static ble_status_t app_le_end_test_result_cb(event_msg_t* msg);
static ble_status_t app_config_mode_status_cb(event_msg_t* msg);


static const ble_gap_event_cb_t app_gap_event_handle = {
	.adv_report = app_adv_report_cb,
};

static const ble_common_event_cb_t app_common_event_handle = {
	.cmd_complete = app_cmd_complete_cb,
	.status_report = app_status_report_cb,
	.le_end_test_result = app_le_end_test_result_cb,
	.config_mode_status = app_config_mode_status_cb,
};


/* Event buffer */
uint8_t event_buf[EVENT_BUFFER_LENGTH];
/* Event parameter */
event_t evt_param = {.event_msg.data = event_buf, .event_msg.data_len = 0, .event_id = 0};

static ble_status_t app_adv_report_cb(event_msg_t* msg)
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

void button_cb(void)
{
	/* Add button callback functionality here */
}

static void app_ibeacon_init(void)
{
	ble_status_t status;
	const uint8_t ibeacon_data[] = {BEACON_ADV_FLAG, BEACON_ADV_LENGTH, BEACON_ADV_TYPE, COMPANY_IDENTIFIER_CODE, ADDTIONAL_MANUFACTURER_SPECIFIC_DATA};
	
	status = ble_adv_data_set(ibeacon_data, sizeof(ibeacon_data));
	if(BLE_SUCCESS != status)
	{
		DBG_LOG("iBeacon data set status = 0x%02X", status);
	}
	
	status = ble_adv_param_set(BM_ADV_NONCONNECTABLE_UNDIRECTED, NULL, 160);
	if(BLE_SUCCESS != status)
	{
		DBG_LOG("iBeacon param set status = 0x%02X", status);
	}
	
	status = ble_adv_start();
	if(BLE_SUCCESS == status)
	{
		DBG_LOG("iBeacon started");
	}
	else
	{
		DBG_LOG("iBeacon start status = 0x%02X", status);
	}
}

static void app_init(void)
{
	/* BLE event callback registration */
	ble_mgr_events_register_callback(BLE_GAP_EVENT_TYPE, &app_gap_event_handle);
	ble_mgr_events_register_callback(BLE_COMMON_EVENT_TYPE, &app_common_event_handle);
	
	/* Initialize and start iBeacon */
	app_ibeacon_init();
}


int main(void)
{
	/* Initialize system */
	system_init();
	/* Initialize serial console */
	serial_console_init();
	/* Initialize button */
	button_init();
	
	DBG_LOG("Initializing BLE Application");
	/* Initialize BLE */
	ble_mgr_device_init();
	/* Initialize application and start iBeacon */
	app_init();
	
	while(true)
	{	
		/* Write application task */
		memset(event_buf, 0x00, 100);
		ble_mgr_get_event(&evt_param);
	}
}

