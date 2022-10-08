/**
 *
 * \file
 *
 * \brief WiFi Provisioning Implementations
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

#include "provisioning_app.h"
//#define PROVAPP_PRINTF(...)
//#define PROVAPP_PRINTF_CONT(...)
#define PROVAPP_PRINTF(...)			do{printf("$ "); printf(__VA_ARGS__);}while(0)
#define PROVAPP_PRINTF_CONT(...)	do{printf(__VA_ARGS__);}while(0)
#define LED_CLEAR					(0)
#define LED_FLASHING_FAST			(60)
#define LED_FLASHING_MEDIUM			(255)
#define LOCAL_STATE_NOT_CONNECTED	(0x00)
#define LOCAL_STATE_CONNECTED		(0x01)
#define LOCAL_STATE_IDLE			(0x02)
static uint8_t local_state;
static uint8_t led_state;
static wifi_provision_scanlist scan_list;
static credentials provisioned_credentials;
static credentials_mschapv2 provisioned_mschapv2_credentials;
static void led_clear(void);
static void led_toggle(void);
static led_flash(uint32_t interval_ms);
static void set_local_state(uint8_t s);
static void led_clear(void)
{
	nm_bsp_stop_timer();
	#if defined(__SAMD21J18A__)
	port_pin_set_output_level(LED0_PIN, true);
	#else
	LED_Off(LED0);
	#endif
}
static void led_toggle(void)
{
	#if defined(__SAMD21J18A__)
	port_pin_toggle_output_level(LED0_PIN);
	#else
	LED_Toggle(LED0);
	#endif
}
static led_flash(uint32_t interval_ms)
{
	nm_bsp_stop_timer();
	nm_bsp_start_timer(led_toggle, interval_ms);
}
static void set_local_state(uint8_t s)
{
	switch(s)
	{
		case LOCAL_STATE_NOT_CONNECTED:
			local_state = LOCAL_STATE_NOT_CONNECTED;
			led_state = LED_FLASHING_FAST;
			led_flash(LED_FLASHING_FAST);
		break;
		case LOCAL_STATE_CONNECTED:
			local_state = LOCAL_STATE_CONNECTED;
			led_state = LED_FLASHING_MEDIUM;
			led_flash(LED_FLASHING_MEDIUM);
		break;
		case LOCAL_STATE_IDLE:
		default:
			local_state = LOCAL_STATE_IDLE;
			led_state = LED_CLEAR;
			led_clear();
		break;
	}
}
uint8_t retrieve_mschapv2_credentials(credentials_mschapv2 *c)
{
	uint8_t v = CREDENTIALS_VALID;

	/*if (c->sec_type != 4){
		v = CREDENTIALS_NOT_VALID;
	}*/
	//printf("\n\rINSIDE retrieve_mschapv2_credentials");

	if (provisioned_mschapv2_credentials.ssid_length == 0)
	{
		v = CREDENTIALS_NOT_VALID;
	}
	else
	{
		c->sec_type = provisioned_mschapv2_credentials.sec_type;
		c->ssid_length = provisioned_mschapv2_credentials.ssid_length;
		c->passphrase_length = provisioned_mschapv2_credentials.passphrase_length;
		memcpy(c->ssid,provisioned_mschapv2_credentials.ssid, WIFI_PROVISION_MAX_SSID_LENGTH);
		memcpy(c->username,provisioned_mschapv2_credentials.username,WIFI_PROVISION_MAX_USERNAME_LENGTH);
		memcpy(c->passphrase,provisioned_mschapv2_credentials.passphrase,WIFI_PROVISION_MAX_PASS_LENGTH);

		v = CREDENTIALS_VALID;
	}
	return v;
}
uint8_t retrieve_credentials(credentials *c)
{
	uint8_t v = CREDENTIALS_VALID;
	if (provisioned_credentials.ssid_length == 0)
	{
		v = CREDENTIALS_NOT_VALID;
	}
	else
	{
		c->sec_type = provisioned_credentials.sec_type;
		c->ssid_length = provisioned_credentials.ssid_length;
		c->passphrase_length = provisioned_credentials.passphrase_length;
		memcpy(c->ssid,provisioned_credentials.ssid, WIFI_PROVISION_MAX_SSID_LENGTH);
		if (c->sec_type == M2M_WIFI_SEC_WEP)
		{
			// Convert WEP passphrase for m2m_wifi_connect friendly format.
			tstrM2mWifiWepParams *wepParams = (tstrM2mWifiWepParams *)c->passphrase;
			memset(wepParams, 0, sizeof(tstrM2mWifiWepParams));
			wepParams->u8KeyIndx = M2M_WIFI_WEP_KEY_INDEX_1;
			wepParams->u8KeySz  = (strlen((const char *)provisioned_credentials.passphrase)==WEP_40_KEY_STRING_SIZE)?
			WEP_40_KEY_STRING_SIZE + 1:
			WEP_104_KEY_STRING_SIZE + 1;
			memcpy((uint8*)(&wepParams->au8WepKey), provisioned_credentials.passphrase, wepParams->u8KeySz-1);
		}
		else
		{
			memcpy(c->passphrase,provisioned_credentials.passphrase,WIFI_PROVISION_MAX_PASS_LENGTH);
		}
		v = CREDENTIALS_VALID;
	}
	return v;
}
at_ble_status_t wifi_provision_app_credentials_update(credentials *c)
{
	credentials cred;
	at_ble_status_t s = AT_BLE_SUCCESS;
	provisioned_credentials.sec_type = c->sec_type;
	provisioned_credentials.ssid_length = c->ssid_length;
	provisioned_credentials.passphrase_length = c->passphrase_length;
	memcpy(&provisioned_credentials.ssid,c->ssid,WIFI_PROVISION_MAX_SSID_LENGTH);
	memcpy(&provisioned_credentials.passphrase,c->passphrase,WIFI_PROVISION_MAX_PASS_LENGTH);
	if (CREDENTIALS_VALID == retrieve_credentials(&cred))
	{
		set_local_state(LOCAL_STATE_IDLE);
		/*PROVAPP_PRINTF("Provisioned AP is\n");
		PROVAPP_PRINTF("    SSID: %s\n",c->ssid);
		PROVAPP_PRINTF("    PASS: %s\n",c->passphrase);
		PROVAPP_PRINTF("    SECU: %u\n",c->sec_type);*/
		s = AT_BLE_SUCCESS;
	}
	else
	{
		PROVAPP_PRINTF("Error: AP credentials are not valid.\n");
		s = AT_BLE_FAILURE;
	}
	return s;
}
at_ble_status_t wifi_provision_app_mschapv2_credentials_update(credentials_mschapv2 *c)
{
	credentials_mschapv2 mschapv2_cred;
	at_ble_status_t s = AT_BLE_SUCCESS;
	provisioned_mschapv2_credentials.sec_type = c->sec_type;
	provisioned_mschapv2_credentials.ssid_length = c->ssid_length;
	provisioned_mschapv2_credentials.passphrase_length = c->passphrase_length;
	provisioned_mschapv2_credentials.username_length = c->username_length;
	memcpy(&provisioned_mschapv2_credentials.ssid,c->ssid,WIFI_PROVISION_MAX_SSID_LENGTH);
	memcpy(&provisioned_mschapv2_credentials.passphrase,c->passphrase,WIFI_PROVISION_MAX_PASS_LENGTH);
	memcpy(&provisioned_mschapv2_credentials.username,c->username,WIFI_PROVISION_MAX_USERNAME_LENGTH);
	if (CREDENTIALS_VALID == retrieve_mschapv2_credentials(&mschapv2_cred))
	{
		set_local_state(LOCAL_STATE_IDLE);
		/*PROVAPP_PRINTF("\n\rProvisioned AP is");
		PROVAPP_PRINTF("\n\rSSID: %s",c->ssid);
		PROVAPP_PRINTF("\n\rPASS: %s",c->passphrase);
		PROVAPP_PRINTF("\n\rSECU: %u",c->sec_type);
		PROVAPP_PRINTF("\n\rUNAME: %s",c->username);*/
		s = AT_BLE_SUCCESS;
	}
	else
	{
		PROVAPP_PRINTF("Error: AP credentials are not valid.\n");
		s = AT_BLE_FAILURE;
	}
	return s;
}
void ble_app_disconnected_update(at_ble_handle_t connection_handle)
{
	set_local_state(LOCAL_STATE_IDLE);
	//tell the provision profile that ble is disconnected
	ble_disconnected();
	PROVAPP_PRINTF("BLE Disconnected.\n");
}
void ble_app_connected_update(at_ble_handle_t connection_handle)
{
	set_local_state(LOCAL_STATE_CONNECTED);
	//tell the provision profile that ble is connected
	ble_connected();
	PROVAPP_PRINTF("BLE Connected.\n");
}
void ble_app_paired_update(at_ble_handle_t paired_handle)
{
	//received pairing successful notification
}
at_ble_status_t wifi_provision_app_scanning_handler(void)
{
	uint8_t st = AT_BLE_FAILURE;
	memset(&scan_list, 0, sizeof(scan_list));
	if (m2m_wifi_request_scan(M2M_WIFI_CH_ALL) == M2M_SUCCESS)
		st = AT_BLE_SUCCESS;
	return st;
}
void send_scan_result(tstrM2mWifiscanResult* sr,uint8_t remain)
{
	if (scan_list.num_valid < WIFI_PROVISION_MAX_AP_NUM && sr->au8SSID[0])
	{
		uint8_t index = scan_list.num_valid;
		scan_list.scandetails[index].rssi = sr->s8rssi;
		scan_list.scandetails[index].sec_type = sr->u8AuthType;
		memcpy(scan_list.scandetails[index].ssid, sr->au8SSID, sizeof(scan_list.scandetails[index].ssid));
		scan_list.num_valid++;
	}
	if (remain==0)
	{
		wifi_provision_scanlist_receive(&scan_list);
	}
}
at_ble_status_t start_provisioning_app(void)
{
	at_ble_status_t s;
	s=provision_start();
	if (ble_is_connected())
		set_local_state(LOCAL_STATE_CONNECTED);
	else
		set_local_state(LOCAL_STATE_NOT_CONNECTED);
	return s;
}
void provision_app_ble_disconnect(void)
{
	disconnect_ble();
	set_local_state(LOCAL_STATE_IDLE);
}
void wifi_state_update(uint8_t s)
{
	inform_wifi_connection_state(s);
}
uint8_t provisioning_app_processing(void)
{
	uint8_t s;
	//get update on wifi provision profile's state
	s=wifi_provision_processing();
	switch(local_state)
	{
		case LOCAL_STATE_NOT_CONNECTED:
			if (led_state != LED_FLASHING_FAST)
			{
				led_flash(LED_FLASHING_FAST);
				led_state = LED_FLASHING_FAST;
			}
			break;
		case LOCAL_STATE_CONNECTED:
			if (led_state != LED_FLASHING_MEDIUM)
			{
				led_flash(LED_FLASHING_MEDIUM);
				led_state = LED_FLASHING_MEDIUM;
			}
			break;
		case LOCAL_STATE_IDLE:
		default:
			local_state = LOCAL_STATE_IDLE;
			if (led_state != LED_CLEAR)
			{
				led_clear();
				led_state = LED_CLEAR;
			}
			break;
	}
	//return wifi provision profile's state to main loop
	return s;
}
void initialise_provisioning_app(void)
{
	register_wifi_provision_credentials_handler(wifi_provision_app_mschapv2_credentials_update);
	register_wifi_provision_scanning_handler(wifi_provision_app_scanning_handler);
	register_ble_connected_event_cb(ble_app_connected_update);
	register_ble_disconnected_event_cb(ble_app_disconnected_update);
	register_ble_paired_event_cb(ble_app_paired_update);
	local_state = LOCAL_STATE_IDLE;
	led_state = LED_CLEAR;
}
