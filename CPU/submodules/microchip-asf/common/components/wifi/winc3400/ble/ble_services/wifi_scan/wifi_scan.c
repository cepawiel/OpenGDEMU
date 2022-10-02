/**
 * \file
 *
 * \brief wifi_scan Service
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
*							        Includes                                     		*
****************************************************************************************/
#include "wifi_scan.h"
/****************************************************************************************
*							        Global	                                     		*
****************************************************************************************/
#define APDETAILS_MAX_LEN			(36)
/** Initial value of scan mode characteristic value */
static uint8_t init_scan_mode=0;
/** Initial value of ap count characteristic value */
static uint8_t init_ap_count=0;
/** Initial value of ap details characteristic value */
static uint8_t init_ap_details=0;
static uint8_t num_ap;
static uint8_t scan_mode;
typedef struct {
	uint8_t   sec_type;
	uint8_t   rssi;
	uint8_t   ssidLen;
#ifdef WIFI_PROVISIONING
	uint8_t   ssid[WIFI_PROVISION_MAX_SSID_LENGTH];
#endif
} ap_details_t;

#ifdef WIFI_PROVISIONING
static ap_details_t apdetails[WIFI_PROVISION_MAX_AP_NUM];
#endif

static const at_ble_char_presentation_t wifiscan_char_pres_fmt_uint8 =
{
	.unit = 0x2700,		//ATT_UNIT_UNITLESS
	.description = 0,
	.format = 0x04,		//ATT_FORMAT_UINT8
	.exponent = 0,
	.name_space = 0x01
};
static const at_ble_char_presentation_t wifiscan_char_pres_fmt_struct =
{
	.unit = 0x2700,		//ATT_UNIT_UNITLESS
	.description = 0,
	.format = 0x1b,		//ATT_FORMAT_STRUCT,
	.exponent = 0,
	.name_space = 0x01
};
extern at_ble_connected_t ble_connected_dev_info[MAX_DEVICE_CONNECTED];
/**
  * @brief wifi_scan service initialization
  */
void init_wifi_scan_service(wifiscan_gatt_service_handler_t *wifiscan_handle)
{
	uint8_t i;
	/* Configure wifi_scan service */
	wifiscan_handle->serv_handle = 0;
	wifiscan_handle->serv_uuid.type = AT_BLE_UUID_128;
	memcpy(&(wifiscan_handle->serv_uuid.uuid[0]), WIFI_SCAN_SERVICE_UUID, WIFISCAN_UUID_128_LEN);
	/* Characteristic Info: (1) mode (2) apcount (3) apdetails */
	/* (1) mode */
	/* handle */
	wifiscan_handle->serv_chars[0].char_val_handle = 0;
	/* UUID */
	wifiscan_handle->serv_chars[0].uuid.type = AT_BLE_UUID_128;
	memcpy(&(wifiscan_handle->serv_chars[0].uuid.uuid[0]), WIFI_SCAN_MODE_CHAR_UUID, WIFISCAN_UUID_128_LEN);
	/* Properties */
	wifiscan_handle->serv_chars[0].properties = (AT_BLE_CHAR_READ | AT_BLE_CHAR_WRITE | AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_INDICATE);
	/* value */
	wifiscan_handle->serv_chars[0].init_value = &init_scan_mode;
	wifiscan_handle->serv_chars[0].value_init_len = sizeof(int8_t);
	wifiscan_handle->serv_chars[0].value_max_len = sizeof(int8_t);
	/* permissions */
	wifiscan_handle->serv_chars[0].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	/* user defined name */
	wifiscan_handle->serv_chars[0].user_desc			= CHAR_USER_STR_WIFISCAN_SCANMODE;
	wifiscan_handle->serv_chars[0].user_desc_len		= CHAR_USER_STR_WIFISCAN_SCANMODE_LEN;
	wifiscan_handle->serv_chars[0].user_desc_max_len	= CHAR_USER_STR_WIFISCAN_SCANMODE_LEN;
	/*user description permissions*/
//	wifiscan_handle->serv_chars[0].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*client config permissions*/
//	wifiscan_handle->serv_chars[0].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*server config permissions*/
//	wifiscan_handle->serv_chars[0].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*user desc handles*/
	wifiscan_handle->serv_chars[0].user_desc_handle = 0;
	/*client config handles*/
	wifiscan_handle->serv_chars[0].client_config_handle = 0;
	/*server config handles*/
	wifiscan_handle->serv_chars[0].server_config_handle = 0;
	/* presentation format */
	wifiscan_handle->serv_chars[0].presentation_format = &wifiscan_char_pres_fmt_uint8;
	/* (2) apcount */
	/* handle */
	wifiscan_handle->serv_chars[1].char_val_handle = 0;
	/* UUID */
	wifiscan_handle->serv_chars[1].uuid.type = AT_BLE_UUID_128;
	memcpy(&(wifiscan_handle->serv_chars[1].uuid.uuid[0]), WIFI_SCAN_APCOUNT_CHAR_UUID, WIFISCAN_UUID_128_LEN);
	/* Properties */
	wifiscan_handle->serv_chars[1].properties = AT_BLE_CHAR_READ;
	/* value */
	wifiscan_handle->serv_chars[1].init_value = &init_ap_count;
	wifiscan_handle->serv_chars[1].value_init_len = sizeof(int8_t);
	wifiscan_handle->serv_chars[1].value_max_len =  sizeof(int8_t);
	/* permissions */
	wifiscan_handle->serv_chars[1].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;
	/* user defined name */
//	wifiscan_handle->serv_chars[1].user_desc			= CHAR_USER_STR_WIFISCAN_APCOUNT;
//	wifiscan_handle->serv_chars[1].user_desc_len		= CHAR_USER_STR_WIFISCAN_APCOUNT_LEN;
//	wifiscan_handle->serv_chars[1].user_desc_max_len	= CHAR_USER_STR_WIFISCAN_APCOUNT_LEN;
	/*user description permissions*/
	wifiscan_handle->serv_chars[1].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*client config permissions*/
	wifiscan_handle->serv_chars[1].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*server config permissions*/
	wifiscan_handle->serv_chars[1].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
	/*user desc handles*/
	wifiscan_handle->serv_chars[1].user_desc_handle = 0;
	/*client config handles*/
	wifiscan_handle->serv_chars[1].client_config_handle = 0;
	/*server config handles*/
	wifiscan_handle->serv_chars[1].server_config_handle = 0;
	/* presentation format */
	wifiscan_handle->serv_chars[1].presentation_format = &wifiscan_char_pres_fmt_uint8;
#ifdef WIFI_PROVISIONING
	/* (3) ap details */
	if(WIFISCAN_CHARACTERISTIC_COUNT > WIFISCAN_BASIC_CHARACTERISTIC)
	{
		for (i=WIFISCAN_BASIC_CHARACTERISTIC; i < WIFISCAN_CHARACTERISTIC_COUNT; i++)
		{
			/* handle */
			wifiscan_handle->serv_chars[i].char_val_handle = 0;
			/* UUID */
			wifiscan_handle->serv_chars[i].uuid.type = AT_BLE_UUID_128;
			memcpy(&(wifiscan_handle->serv_chars[i].uuid.uuid[0]), WIFI_SCAN_APDETAILS_CHAR_UUID, WIFISCAN_UUID_128_LEN);
			/* Properties */
			wifiscan_handle->serv_chars[i].properties = AT_BLE_CHAR_READ ;
			/* value */
			wifiscan_handle->serv_chars[i].init_value = &init_ap_details;
			wifiscan_handle->serv_chars[i].value_init_len = APDETAILS_MAX_LEN;
			wifiscan_handle->serv_chars[i].value_max_len = APDETAILS_MAX_LEN;
			/* permissions */
			wifiscan_handle->serv_chars[i].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;
			/* user defined name */
			wifiscan_handle->serv_chars[i].user_desc			= CHAR_USER_STR_WIFISCAN_APDETAILS;
			wifiscan_handle->serv_chars[i].user_desc_len		= CHAR_USER_STR_WIFISCAN_APDETAILS_LEN;
			wifiscan_handle->serv_chars[i].user_desc_max_len	= CHAR_USER_STR_WIFISCAN_APDETAILS_LEN;
			/*user description permissions*/
//			wifiscan_handle->serv_chars[i].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
			/*client config permissions*/
//			wifiscan_handle->serv_chars[i].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
			/*server config permissions*/
//			wifiscan_handle->serv_chars[i].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;
			/*user desc handles*/
			wifiscan_handle->serv_chars[i].user_desc_handle = 0;
			/*client config handles*/
			wifiscan_handle->serv_chars[i].client_config_handle = 0;
			/*server config handles*/
			wifiscan_handle->serv_chars[i].server_config_handle = 0;
			/* presentation format */
			wifiscan_handle->serv_chars[i].presentation_format = &wifiscan_char_pres_fmt_struct;
		}
	}
#endif
}

#ifdef WIFI_PROVISIONING
/**
  * @brief wifi_scan service definition
  */
at_ble_status_t wifiscan_primary_service_define(wifiscan_gatt_service_handler_t *wifiscan_primary_service)
{

	return(at_ble_primary_service_define(&wifiscan_primary_service->serv_uuid,
											&wifiscan_primary_service->serv_handle,
											NULL, WIFISCAN_INCLUDED_SERVICE_COUNT,
											&wifiscan_primary_service->serv_chars, WIFISCAN_CHARACTERISTIC_COUNT));
}
#endif

/**
  * @brief utility function to retrieve and print out characteristics (scanmode and ap_num)
  */
void wifiscan_print_char(wifiscan_gatt_service_handler_t *wifiscan_handler)
{
	uint8_t val;
    uint16_t len = 1;
	at_ble_characteristic_value_get(wifiscan_handler->serv_chars[0].char_val_handle, &val,&len);
	printf("# Scan Service status: scan mode: %u\n",val);
	len = 1;
	at_ble_characteristic_value_get(wifiscan_handler->serv_chars[1].char_val_handle, &val,&len);
	printf("# ap_num: %u\n",val);
}
/**
  * @brief scan mode characteristics change
  */
uint8_t wifiscan_char_change_scanmode(at_ble_characteristic_changed_t *p, wifiscan_gatt_service_handler_t *wifiscan_handle)
{
	if (p->char_handle == wifiscan_handle->serv_chars[0].char_val_handle)
	{
		if ((at_ble_characteristic_value_set(p->char_handle, (uint8_t *) &(p->char_new_value[0]), p->char_len)) == AT_BLE_FAILURE)
			return FAILED_WIFISCAN_CHANGE_PARAM;
		else
			return VALID_WIFISCAN_CHANGE_PARAM;
	}
	return INVALID_WIFISCAN_CHANGE_PARAM;
}
/**
  * @brief application update of scan mode characteristics value
  */
at_ble_status_t wifiscan_update_scanmode_char_value (wifiscan_gatt_service_handler_t *wifiscan_serv , uint8_t char_data)
{
	scan_mode = char_data;
	// Updating scan mode characteristic value
	if ((at_ble_characteristic_value_set(wifiscan_serv->serv_chars[0].char_val_handle, &scan_mode, sizeof(uint8_t))) == AT_BLE_FAILURE)
	{
		printf("# wifiscan service: updating scan mode failed");
		return AT_BLE_FAILURE;
	}
	// sending scan mode notification
	if((at_ble_notification_send(ble_connected_dev_info[0].handle, wifiscan_serv->serv_chars[0].char_val_handle)) == AT_BLE_FAILURE) {
		printf("# wifiscan service: sending scan mode notification failed");
		return AT_BLE_FAILURE;
	}
	return AT_BLE_SUCCESS;
}

#ifdef WIFI_PROVISIONING
/**
  * @brief scan list receive from application
  */
at_ble_status_t wifiscan_scanlist_receive(wifiscan_gatt_service_handler_t *wifiscan_serv , wifi_provision_scanlist *param, uint8_t *num_ap_found)
{
	uint8_t i;
	uint8_t usable; //counting AP with usable ssid
	usable = 0;
	for (i=0;i<param->num_valid;i++)
	{
		uint8_t ssid_len = 0;

		while ((ssid_len < WIFI_PROVISION_MAX_SSID_LENGTH) && (param->scandetails[i].ssid[ssid_len] != 0))
			ssid_len++;
		if (ssid_len > 0)
		{
			memset(&apdetails[usable],0,sizeof(ap_details_t));
			apdetails[usable].ssidLen = ssid_len;
			apdetails[usable].sec_type = param->scandetails[i].sec_type;
			apdetails[usable].rssi = param->scandetails[i].rssi;
			memcpy(apdetails[usable].ssid, param->scandetails[i].ssid, ssid_len);
			//this AP has usable ssid, increment usable count
			usable++;
		}
	}
	//finalised usable number of AP
	num_ap = usable;
	*num_ap_found = usable;
	/* Updating apcount characteristic value */
	if ((at_ble_characteristic_value_set(wifiscan_serv->serv_chars[1].char_val_handle, &num_ap, sizeof(uint8_t))) == AT_BLE_FAILURE)
	{
		printf("# wifiscan service: updating apcount failed");
		return AT_BLE_FAILURE;
	}
	/* sending apcount notification */
	if((at_ble_notification_send(ble_connected_dev_info[0].handle, wifiscan_serv->serv_chars[1].char_val_handle)) == AT_BLE_FAILURE)
	{
		printf("# wifiscan service: sending scan apcount notification to the peer failed");
		return AT_BLE_FAILURE;
	}
	for (i=0;i<num_ap;i++)
	{
		if ((at_ble_characteristic_value_set(wifiscan_serv->serv_chars[i+WIFISCAN_BASIC_CHARACTERISTIC].char_val_handle, (uint8_t*) &apdetails[i], sizeof(ap_details_t))) == AT_BLE_FAILURE)
		{
			printf("# wifiscan service: updating apdetails ap_num:%u failed",i);
			return AT_BLE_FAILURE;
		}
	}
	return AT_BLE_SUCCESS;
}
#endif
