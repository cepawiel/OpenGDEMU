/**
 *
 * \file
 *
 * \brief WINC3400 WiFi-MSCHAPv2 BLE Provisioning Profile Demo Example.
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

/** \mainpage
 * \section intro Introduction
 * This example demonstrates the use of the WINC3400 with the SAM Xplained Pro
 * board to behave as a station, connect to AP with MSCHAPv2 using ble provisioning.<br>
 * It uses the following hardware:
 * - the SAM Xplained Pro.
 * - the WINC3400 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC3400 and connect AP as a station mode.
 *
 * \section usage Usage
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as the follows.
 * \code
 *    Baud Rate : 115200
 *    Data : 8bit
 *    Parity bit : none
 *    Stop bit : 1bit
 *    Flow control : none
 * \endcode
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 * \code
 *      -- WiFi BLE Provisioning for mschapv2 Profile demo --
 *		-- SAMD21_XPLAINED_PRO --
 *		-- Compiled: Mar 12 2019 11:27:30 --
 *    (APP)(INFO)Chip ID 3400d2
 *    (APP)(INFO)Curr driver ver: x.x.x
 *    (APP)(INFO)Curr driver HIF Level: (2) x.x
 *    (APP)(INFO)Fw HIF: 8104
 *    (APP)(INFO)Firmware HIF (2) : x.x
 *    (APP)(INFO)Firmware ver   : x.x.x
 *    (APP)(INFO)Firmware Build <Month> DD YYYY Time xx:xx:xx
 *	  (APP)(INFO)Ota HIF: 0000
 *	  (APP)(INFO)No valid Ota image
 *    (APP)(INFO)OTP MAC
 *	  (APP)(INFO)MAC Address: xx:xx:xx:xx:xx:xx
 *
 *      (APP)(INFO)OTP MAC
 *		(APP)(INFO)MAC Address: F8:F0:05:F6:79:FA
 *
 *		BLE is initializing
 *
 *		Device Name: MCHP_BLE
 *
 * 		This is WiFi Provisioning.
 *		(APP)(INFO)SW0 Long press   (APP)(INFO)Hold SW0 for 2 sec to start provisioning.
 *    	Connecting to WINC3400_ENTERPRISE
 *      SSID: test_ap
 *      Username:DEMO_USER
 *      Password:12345678
 *    	Wi-Fi connected
 *    	Wi-Fi IP is 192.168.100.105
 *    	Connection successfully completed.
 * \endcode
 *
 * \section compinfo Compilation Information
 * This software was written for the GNU GCC compiler using Atmel Studio 6.2
 * Other compilers are not guaranteed to work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.microchip.com">Microchip</A>.\n
 */

#include <asf.h>
#include "platform.h"
#include "m2m_ble.h"
#include "at_ble_api.h"
#include "provisioning_app.h"
#include "ble_utils.h"
#include "m2m_test_config.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "driver/include/m2m_periph.h"
#include "stdio_serial.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WiFi BLE Provisioning for mschapv2 Profile demo --"STRING_EOL \
"-- "BOARD_NAME" --"STRING_EOL \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

#define EVENT_BTN1_LONG_PRESS						NBIT0
#define EVENT_BTN1_SHORT_PRESS						NBIT1
#define APP_STATE_IDLE								0
#define APP_STATE_WAITING_FOR_WIFI_DISCONNECTION	1
#define APP_STATE_WAITING_FOR_CREDENTIALS_PROVISION	2
#define APP_STATE_WAITING_FOR_WIFICONNECT_PRESS		3
#define APP_STATE_WAITING_FOR_WIFI_CONNECTION		4
#define CONNECTION_ATTEMPT_CONNECTING				0
#define CONNECTION_ATTEMPT_CONNECTED				1
#define CONNECTION_ATTEMPT_PROVISIONED				2
#define CONNECTION_ATTEMPT_DISCONNECTED				3
#define CONNECTION_ATTEMPT_FAILED					4
#define WIFI_CONNECT_TIMEOUT						15000
static uint8_t gu8ConnectionAttempt;
static volatile uint8 gu8IsWiFiConnected = M2M_WIFI_DISCONNECTED;
static volatile uint8 gu8Event = 0;
static uint8 gu8ScanIndex = 0;
static at_ble_event_parameter_t gBLEP __aligned(4);
static void configure_console(void);
static void disable_pullups(void);
static void app_wifi_handle_event(uint8 u8MsgType, void * pvMsg);
static void	set_3400name(void);
static void inits(void);
static void wifi_connect_timeout(void);
static void btn_callback(uint8 btn, uint8 press);
static void btn_callback(uint8 btn, uint8 press)
{
	if (SW1 == btn)
	{
		gu8Event = (!press) ? (EVENT_BTN1_SHORT_PRESS) : (EVENT_BTN1_LONG_PRESS);
	}
}
/** UART module for debug. */
static struct usart_module cdc_uart_module;
#define CONF_STDIO_USART_MODULE  EDBG_CDC_MODULE
#define CONF_STDIO_MUX_SETTING   EDBG_CDC_SERCOM_MUX_SETTING
#define CONF_STDIO_PINMUX_PAD0   EDBG_CDC_SERCOM_PINMUX_PAD0
#define CONF_STDIO_PINMUX_PAD1   EDBG_CDC_SERCOM_PINMUX_PAD1
#define CONF_STDIO_PINMUX_PAD2   EDBG_CDC_SERCOM_PINMUX_PAD2
#define CONF_STDIO_PINMUX_PAD3   EDBG_CDC_SERCOM_PINMUX_PAD3
#define CONF_STDIO_BAUDRATE      115200
/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;
	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = CONF_STDIO_MUX_SETTING;
	usart_conf.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	usart_conf.baudrate    = CONF_STDIO_BAUDRATE;
	stdio_serial_init(&cdc_uart_module, CONF_STDIO_USART_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}
static void wifi_connect_timeout(void)
{
	nm_bsp_stop_timer();
	M2M_INFO("Timed out connecting to AP.");
	gu8ConnectionAttempt = CONNECTION_ATTEMPT_FAILED;
}
static void app_wifi_handle_event(uint8 u8MsgType, void * pvMsg)
{
	if (u8MsgType == M2M_WIFI_RESP_CON_STATE_CHANGED)
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged*) pvMsg;
		switch(gu8ConnectionAttempt)
		{
			case CONNECTION_ATTEMPT_CONNECTING:
				gu8ConnectionAttempt = pstrWifiState->u8CurrState ? CONNECTION_ATTEMPT_CONNECTED : CONNECTION_ATTEMPT_FAILED;
			break;
			case CONNECTION_ATTEMPT_PROVISIONED:
				if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED)
				{
					gu8ConnectionAttempt = CONNECTION_ATTEMPT_DISCONNECTED;
				}
				break;
		}
		if(pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED)
		{
			gu8IsWiFiConnected = M2M_WIFI_DISCONNECTED;
		}
	}
	else if (u8MsgType == M2M_WIFI_REQ_DHCP_CONF)
	{
		uint8 *pu8IPAddress = (uint8*) pvMsg;
		M2M_INFO("DHCP IP Address :: %u.%u.%u.%u ::\n", pu8IPAddress[0],
		pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);

		gu8IsWiFiConnected = M2M_WIFI_CONNECTED;
	}
	else if (u8MsgType == M2M_WIFI_RESP_SCAN_DONE)
	{
		tstrM2mScanDone *pstrInfo = (tstrM2mScanDone*) pvMsg;

		if (gu8IsWiFiConnected == M2M_WIFI_DISCONNECTED)
		{
			gu8ScanIndex = 0;

			if (pstrInfo->u8NumofCh >= 1)
			{
				m2m_wifi_req_scan_result(gu8ScanIndex);
				gu8ScanIndex++;
			}
		}
	}
	else if (u8MsgType == M2M_WIFI_RESP_SCAN_RESULT)
	{
		uint8 u8NumFoundAPs = m2m_wifi_get_num_ap_found();

		if (gu8IsWiFiConnected == M2M_WIFI_DISCONNECTED)
		{
			tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult*) pvMsg;
			send_scan_result(pstrScanResult, u8NumFoundAPs-gu8ScanIndex);
			if(gu8ScanIndex < u8NumFoundAPs)
			{
				m2m_wifi_req_scan_result(gu8ScanIndex);
				gu8ScanIndex++;
			}
		}
	}
	else if (u8MsgType == M2M_WIFI_RESP_CURRENT_RSSI)
	{
		sint8	*rssi = (sint8*)pvMsg;
		M2M_INFO("(%lu) rssi %d\n",NM_BSP_TIME_MSEC,*rssi);
	}
	else if(u8MsgType == M2M_WIFI_RESP_SET_GAIN_TABLE) {
		tstrM2MGainTableRsp *pstrRsp = (tstrM2MGainTableRsp *)pvMsg;
		M2M_ERR("Gain Table Load Fail %d\n", pstrRsp->s8ErrorCode);
	}
	else if (u8MsgType == M2M_WIFI_RESP_BLE_API_RECV)
	{
		tstrM2mBleApiMsg* rx = (tstrM2mBleApiMsg*)pvMsg;
#ifdef BLE_API_DBG
		M2M_INFO("BLE_API_RECV:%x:%x:%x:%x:%x:%x:%x:%x\n", rx->u16Len, rx->data[0], rx->data[1], rx->data[2], rx->data[3], rx->data[4], rx->data[5], rx->data[6]);
#endif
		platform_receive(rx->data, rx->u16Len);
	}
}
static void disable_pullups(void)
{
	uint32 pinmask;
	pinmask = (
	M2M_PERIPH_PULLUP_DIS_HOST_WAKEUP|
	M2M_PERIPH_PULLUP_DIS_SD_CMD_SPI_SCK|
	M2M_PERIPH_PULLUP_DIS_SD_DAT0_SPI_TXD);
	m2m_periph_pullup_ctrl(pinmask, 0);
}
static void set_3400name(void)
{
	uint8 mac_addr[6];
	uint8 u8IsMacAddrValid;
	sint8 deviceName[] = M2M_DEVICE_NAME;
	gu8IsWiFiConnected = M2M_WIFI_DISCONNECTED;
	gu8Event = 0;

	m2m_wifi_get_otp_mac_address(mac_addr, &u8IsMacAddrValid);
	if (!u8IsMacAddrValid) {
		uint8 DEFAULT_MAC[] = MAC_ADDRESS;
		M2M_INFO("Default MAC\n");
		m2m_wifi_set_mac_address(DEFAULT_MAC);
		} else {
		M2M_INFO("OTP MAC\n");
	}
	m2m_wifi_get_mac_address(mac_addr);
	M2M_INFO("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
	mac_addr[0],mac_addr[1],mac_addr[2],
	mac_addr[3],mac_addr[4],mac_addr[5]);
	// Name must be in the format WINC3400_00:00
	{
		#define HEX2ASCII(x) (((x)>=10)? (((x)-10)+'A') : ((x)+'0'))

		uint16 len;
		len = m2m_strlen(deviceName);
		if (len >= 5) {
			deviceName[len-1] = HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
			deviceName[len-2] = HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
			deviceName[len-4] = HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
			deviceName[len-5] = HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
		}
	}
	m2m_wifi_set_device_name((uint8*)deviceName, (uint8)m2m_strlen((uint8*)deviceName));
}
static void inits(void)
{
	sint8 ret;
	tstrWifiInitParam param;
	system_init();
	nm_bsp_init();
	nm_bsp_app_init();
	nm_bsp_btn_init(btn_callback);
	configure_console();
	puts(STRING_HEADER);
	//init WiFi driver
	m2m_memset((uint8*)&param, 0, sizeof(param));
	param.pfAppWifiCb = app_wifi_handle_event;
	param.GainTableIndex = GAIN_TABLE_FROM_FLASH;
	ret = m2m_ble_init_callback(&param);
	if (M2M_SUCCESS != ret)
	{
		M2M_ERR("Driver Init Failed <%d>\n",ret);
		M2M_ERR("Reseting\n");
		// Catastrophe - problem with booting. Nothing but to try and reset
		system_reset();
		while (1)
		{
		}
	}
	disable_pullups();
	set_3400name();
	//allow leeway for ble cortus to complete booting up
	nm_bsp_sleep(2000);
}
int main(void)
{
	uint8 btn_press;
	static at_ble_events_t evt;
	uint8_t app_state = APP_STATE_IDLE;
	gu8Event = 0;
	gu8IsWiFiConnected = M2M_WIFI_DISCONNECTED;
	gu8ConnectionAttempt = CONNECTION_ATTEMPT_DISCONNECTED;
	tstrNetworkId networkId;
	tstrAuth1xMschap2 mschapv2_credential;

	//system inits
	inits();
	//ble inits
	m2m_ble_init();
	ble_device_init(NULL);
	//provisioning app init
	initialise_provisioning_app();
	M2M_INFO("Hold SW0 for 2 sec to start provisioning.\r\n");

	//main_loop
	while (1)
	{
		evt = AT_BLE_UNDEFINED_EVENT;
		at_ble_event_get(&evt,&gBLEP,10);
		if (evt != AT_BLE_UNDEFINED_EVENT)
			//printf("\r\nEVENT:%d\r\n", evt);

		while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
			M2M_INFO("Error - WiFi Event not handled");
		}
		if(evt != AT_BLE_UNDEFINED_EVENT)
		{
			ble_event_manager(evt,&gBLEP);
		}
		btn_press = gu8Event;
		gu8Event = 0;
		if (btn_press == EVENT_BTN1_LONG_PRESS)
		{
			if (app_state==APP_STATE_WAITING_FOR_WIFICONNECT_PRESS)
			{
				M2M_INFO("Waiting for short press to connect to AP");
				M2M_INFO(".. now aborting to re-provision\n");
				if (gu8ConnectionAttempt == CONNECTION_ATTEMPT_PROVISIONED)
				{
					m2m_wifi_disconnect();
				}
				app_state = APP_STATE_WAITING_FOR_WIFI_DISCONNECTION;
				btn_press = 0;
			}
			else if (app_state!=APP_STATE_IDLE)
			{
				M2M_INFO("Provisioning in process, aborting..\n");
				provision_app_ble_disconnect();
				M2M_INFO("Press button again to start provisioning\n");
				gu8ConnectionAttempt = CONNECTION_ATTEMPT_DISCONNECTED;
				app_state = APP_STATE_IDLE;
				btn_press = 0;
			}
		}
		switch (app_state)
		{
			case APP_STATE_IDLE:
			{
				if (btn_press == EVENT_BTN1_LONG_PRESS)
				{
					btn_press = 0;
					M2M_INFO("SW0 Long press\r\n");
					if (gu8ConnectionAttempt == CONNECTION_ATTEMPT_PROVISIONED)
					{
						m2m_wifi_disconnect();
					}
					app_state = APP_STATE_WAITING_FOR_WIFI_DISCONNECTION;
				}
				break;
			}
			case APP_STATE_WAITING_FOR_WIFI_DISCONNECTION:
			{
				if (gu8ConnectionAttempt == CONNECTION_ATTEMPT_DISCONNECTED)
				{
					if (start_provisioning_app()== AT_BLE_SUCCESS)
					{
						app_state = APP_STATE_WAITING_FOR_CREDENTIALS_PROVISION;
					}
				}
				break;
			}
			case APP_STATE_WAITING_FOR_CREDENTIALS_PROVISION:
			{
				switch (provisioning_app_processing())
				{
					case WIFI_PROVISION_STATE_HAVE_CREDENTIAL:
					{
						M2M_INFO("\n\rProvisioning data received - press SW0 to attempt WiFi connect");
						app_state = APP_STATE_WAITING_FOR_WIFICONNECT_PRESS;
						break;
					}
					case WIFI_PROVISION_STATE_FAILED:
					{
						M2M_INFO("Press button again to re-start provisioning\r\n");
						app_state = APP_STATE_IDLE;
					}
					break;
				}
				break;
			}
			case APP_STATE_WAITING_FOR_WIFICONNECT_PRESS:
			{
				if (btn_press == EVENT_BTN1_SHORT_PRESS)
				{
					credentials_mschapv2 mycred;
					btn_press = 0;
					if (retrieve_mschapv2_credentials(&mycred) != CREDENTIALS_VALID)
					{
						M2M_INFO("WiFi Connect: using default ssid");
						m2m_wifi_connect(DEFAULT_SSID, sizeof(DEFAULT_SSID),DEFAULT_AUTH, (void *) DEFAULT_KEY, M2M_WIFI_CH_ALL);
					}
					else
					{
						networkId.pu8Bssid = NULL;
						networkId.pu8Ssid = mycred.ssid;
						networkId.u8SsidLen = strlen(mycred.ssid);
						networkId.enuChannel = M2M_WIFI_CH_ALL;

						m2m_wifi_set_system_time(1540293393);
						mschapv2_credential.pu8Domain = NULL;
						mschapv2_credential.pu8UserName = mycred.username;
						mschapv2_credential.pu8Password = mycred.passphrase;
						mschapv2_credential.u16UserNameLen = strlen(mycred.username);
						mschapv2_credential.u16PasswordLen = strlen(mycred.passphrase);
						mschapv2_credential.bUnencryptedUserName = false;
						mschapv2_credential.bPrependDomain = true;

						M2M_INFO("WiFi Connect: using provisioned AP");
						printf("\n\rnetworkId.pu8Ssid:%s", networkId.pu8Ssid);
						printf("\n\rmschapv2_credential.pu8UserName:%s", mschapv2_credential.pu8UserName);
						printf("\n\rmschapv2_credential.pu8Password:%s", mschapv2_credential.pu8Password);
						printf("\n\rmschapv2_credential.u16UserNameLen:%d", mschapv2_credential.u16UserNameLen);
						printf("\n\rmschapv2_credential.u16PasswordLen:%d", mschapv2_credential.u16PasswordLen);
						/*m2m_wifi_connect((char *)mycred.ssid, mycred.ssid_length,
						mycred.sec_type, mycred.passphrase, M2M_WIFI_CH_ALL);*/
						m2m_wifi_connect_1x_mschap2( WIFI_CRED_SAVE_ENCRYPTED, &networkId, &mschapv2_credential);
					}

					wifi_state_update(WIFI_CONNECTION_STATE_CONNECTING);
					app_state = APP_STATE_WAITING_FOR_WIFI_CONNECTION;
					gu8ConnectionAttempt = CONNECTION_ATTEMPT_CONNECTING;
					nm_bsp_stop_timer();
					nm_bsp_start_timer(wifi_connect_timeout, WIFI_CONNECT_TIMEOUT);
				}
				break;
			}
			case APP_STATE_WAITING_FOR_WIFI_CONNECTION:
			{
				switch(gu8ConnectionAttempt)
				{
					case CONNECTION_ATTEMPT_CONNECTED:
					{
						gu8ConnectionAttempt = CONNECTION_ATTEMPT_PROVISIONED;
						wifi_state_update(WIFI_CONNECTION_STATE_PROVISIONED);
						app_state = APP_STATE_IDLE;
						nm_bsp_sleep(1000);
						provision_app_ble_disconnect();
						nm_bsp_sleep(1000);
						M2M_INFO("Provisioning Complete.\r\n");
						break;
					}
					case CONNECTION_ATTEMPT_FAILED:
					{
						gu8ConnectionAttempt = CONNECTION_ATTEMPT_DISCONNECTED;
						wifi_state_update(WIFI_CONNECTION_STATE_PROVISIONFAILED);
						app_state = APP_STATE_IDLE;
						nm_bsp_sleep(1000);
						provision_app_ble_disconnect();
						nm_bsp_sleep(1000);
						M2M_INFO("Failed connecting to AP.\r\n");
						M2M_INFO("Hold SW0 for 2 sec for provisioning.\r\n");
						break;
					}
				}
				break;
			}
		}
	}
}


