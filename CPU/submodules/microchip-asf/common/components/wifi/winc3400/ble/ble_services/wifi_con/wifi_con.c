/**
 * \file
 *
 * \brief wifi_con Service
 *
 * Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
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
 *
 */

/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Atmel
 *Support</a>
 */

/****************************************************************************************
*                                  Includes                                             *
*****************************************************************************************/
#include "wifi_con.h"
/****************************************************************************************
*                                   Global                                              *
****************************************************************************************/
#define STATE_INITIAL_VALUE		(0)
#define APPARAM_INITIAL_VALUE	(0)

#ifdef ENTERPRISE_SECURITY
#define APPARAM_MAX_LEN			(180)
#else
#define APPARAM_MAX_LEN		    (98)
#endif

uint8_t init_connect_state = 0;
uint8_t connect_state = 0;
uint8_t init_ap_param = 0;;
static const at_ble_char_presentation_t wificon_char_pres_fmt_uint8 =
{
    .unit = 0x2700,		//ATT_UNIT_UNITLESS
    .description = 0,
    .format = 0x04,		//ATT_FORMAT_UINT8
    .exponent = 0,
    .name_space = 0x01
};
static const at_ble_char_presentation_t wificon_char_pres_fmt_struct =
{
    .unit = 0x2700,		//ATT_UNIT_UNITLESS
	.description = 0,
	.format = 0x1b,		//ATT_FORMAT_STRUCT,
	.exponent = 0,
	.name_space = 0x01
};
extern at_ble_connected_t ble_connected_dev_info[MAX_DEVICE_CONNECTED];
at_ble_status_t wificon_connect_noti(wificon_gatt_service_handler_t *wificon_handler, uint8_t s)
{
	connect_state=s;
	if ((at_ble_characteristic_value_set(wificon_handler->serv_chars[0].char_val_handle, &connect_state, sizeof(uint8_t))) == AT_BLE_FAILURE)
	{
		printf("# wificon service: updating state failed");
		return AT_BLE_FAILURE;
	}

	if((at_ble_notification_send(ble_connected_dev_info[0].handle, wificon_handler->serv_chars[0].char_val_handle)) == AT_BLE_FAILURE)
	{
		printf("# wificon service: sending con state notification to the peer failed");
		return AT_BLE_FAILURE;
	}
	return AT_BLE_SUCCESS;
}
uint8_t wificon_char_change_state_client_cfg(at_ble_characteristic_changed_t *param, wificon_gatt_service_handler_t *wificon_handle)
{
	if (param->char_handle == wificon_handle->serv_chars[0].client_config_handle)
	{
		if ((at_ble_characteristic_value_set(param->char_handle, (uint8_t *) &(param->char_new_value[0]), param->char_len)) == AT_BLE_FAILURE)
			return FAILED_WIFICON_CHANGE_PARAM;
		else
			return VALID_WIFICON_CHANGE_PARAM;
	}

	return INVALID_WIFICON_CHANGE_PARAM;
}
uint8_t wificon_char_change_apparam(at_ble_characteristic_changed_t *param, wificon_gatt_service_handler_t *wificon_handle)
{
	if (param->char_handle == wificon_handle->serv_chars[1].char_val_handle)
	{
		if ((at_ble_characteristic_value_set(param->char_handle, (uint8_t *) &(param->char_new_value[0]), param->char_len)) == AT_BLE_FAILURE)
			return FAILED_WIFICON_CHANGE_PARAM;
		else
			return VALID_WIFICON_CHANGE_PARAM;
	}

	return INVALID_WIFICON_CHANGE_PARAM;
}
/** @brief wifi_con service initialization
  *
  */
void init_wifi_con_service(wificon_gatt_service_handler_t *wificon_handle)
{
	/* Configure wifi_con service */
	wificon_handle->serv_handle = 0;
	wificon_handle->serv_uuid.type = AT_BLE_UUID_128;
	memcpy(&(wificon_handle->serv_uuid.uuid[0]), WIFI_CON_SERVICE_UUID, WIFICON_UUID_128_LEN);

	/* Characteristic Info: (1) state (2) apparam */

	/* (1) state */
	/* handle */
	wificon_handle->serv_chars[0].char_val_handle = 0;
	/* UUID */
	wificon_handle->serv_chars[0].uuid.type = AT_BLE_UUID_128;
	memcpy(&(wificon_handle->serv_chars[0].uuid.uuid[0]), WIFI_CON_STATE_CHAR_UUID, WIFICON_UUID_128_LEN);
	 /* Properties */
	wificon_handle->serv_chars[0].properties = (AT_BLE_CHAR_READ | AT_BLE_CHAR_WRITE | AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_INDICATE);
	/* value */
	wificon_handle->serv_chars[0].init_value = &init_connect_state;
	wificon_handle->serv_chars[0].value_init_len = sizeof(int8_t);
	wificon_handle->serv_chars[0].value_max_len = sizeof(int8_t);
	/* permissions */
	wificon_handle->serv_chars[0].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	/* user defined name */
	wificon_handle->serv_chars[0].user_desc = CHAR_USER_STR_WIFICON_STATE;
	wificon_handle->serv_chars[0].user_desc_len = CHAR_USER_STR_WIFICON_STATE_LEN;
	wificon_handle->serv_chars[0].user_desc_max_len = CHAR_USER_STR_WIFICON_STATE_LEN;
	/*user description permissions*/
//	wificon_handle->serv_chars[0].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*client config permissions*/
	wificon_handle->serv_chars[0].client_config_permissions = AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR;
	/*server config permissions*/
//	wificon_handle->serv_chars[0].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	 /*user desc handles*/
	wificon_handle->serv_chars[0].user_desc_handle = 0;
	/*client config handles*/
	wificon_handle->serv_chars[0].client_config_handle = 0;
	/*server config handles*/
	wificon_handle->serv_chars[0].server_config_handle = 0;
	/* presentation format */
	wificon_handle->serv_chars[0].presentation_format = &wificon_char_pres_fmt_uint8;

	/* (2) apparam */
	/* handle */
	wificon_handle->serv_chars[1].char_val_handle = 0;
	/* UUID */
	wificon_handle->serv_chars[1].uuid.type = AT_BLE_UUID_128;
	memcpy(&(wificon_handle->serv_chars[1].uuid.uuid[0]), WIFI_CON_APPARAM_CHAR_UUID, WIFICON_UUID_128_LEN);
	/* Properties */
	wificon_handle->serv_chars[1].properties = AT_BLE_CHAR_WRITE;
	/* value */
	wificon_handle->serv_chars[1].init_value = &init_ap_param;
	wificon_handle->serv_chars[1].value_init_len = sizeof(int8_t);
	wificon_handle->serv_chars[1].value_max_len = APPARAM_MAX_LEN;
	/* permissions */
	wificon_handle->serv_chars[1].value_permissions = AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR;
	/* user defined name */
	wificon_handle->serv_chars[1].user_desc			= CHAR_USER_STR_WIFICON_APPARAM;
	wificon_handle->serv_chars[1].user_desc_len		= CHAR_USER_STR_WIFICON_APPARAM_LEN;
	wificon_handle->serv_chars[1].user_desc_max_len	= CHAR_USER_STR_WIFICON_APPARAM_LEN;
	/*user description permissions*/
//	wificon_handle->serv_chars[1].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*client config permissions*/
//	wificon_handle->serv_chars[1].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*server config permissions*/
//	wificon_handle->serv_chars[1].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*user desc handles*/
	wificon_handle->serv_chars[1].user_desc_handle = 0;
	/*client config handles*/
	wificon_handle->serv_chars[1].client_config_handle = 0;
	/*server config handles*/
	wificon_handle->serv_chars[1].server_config_handle = 0;
	/* presentation format */
	wificon_handle->serv_chars[1].presentation_format = &wificon_char_pres_fmt_struct;
}
/** @brief wifi_con service definition
  *
  */
at_ble_status_t wificon_primary_service_define(wificon_gatt_service_handler_t *wificon_primary_service)
{
	return(at_ble_primary_service_define(&wificon_primary_service->serv_uuid,
											&wificon_primary_service->serv_handle,
											NULL, WIFICON_INCLUDED_SERVICE_COUNT,
											&wificon_primary_service->serv_chars, WIFICON_CHARACTERISTIC_COUNT));
}
