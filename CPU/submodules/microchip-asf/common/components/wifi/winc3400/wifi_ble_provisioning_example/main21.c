/**
 * \file
 *
 * \brief WINC3400 WiFi BLE Provisioning Profile Demo Example.
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
 * board to behave as a station, connect to AP using ble provisioning.<br>
 * It uses the following hardware:
 * - the SAM Xplained Pro.
 * - the WINC3400 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC3400 and connect to AP in station mode using ble provisioning.
 *
 * \section usage Usage
 * -# Configure below code in the config main.h for AP information to be connected.
 * \code
 *    #define MAIN_WLAN_SSID         "DEMO_AP"
 *    #define MAIN_WLAN_AUTH         M2M_WIFI_SEC_WPA_PSK
 *    #define MAIN_WLAN_PSK          "12345678"
 * \endcode
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
 *	  -- WiFi BLE Provisioning Profile demo --
 *    -- SAMXXX_XPLAINED_PRO --
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
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
 *	   BLE is initializing
 *
 *	   Device Name: MCHP_BLE
 *
 * 	   This is WiFi Provisioning.
 *     BLE device is in Advertising Mode
 *     Advertising Device Name: 3400-DEMO
 *     Number of APs found: xx
 *
 *     Connected to peer device with address xx:xx:xx:xx:xx:xx
 *
 *     BLE Connected.
 *	   *******************************************
 *	   Receive complete.
 *	   SSID: xxxx
 *	   SSID LENGTH: xxxx
 *	   PASS: xxxx
 *	   PASS LENGTH: xxxx
 *	   SECU: xxxx
 *	   *******************************************
 *     (APP)(INFO)Provisioning data received
 *	   (APP)(INFO)WiFi Connect: using provisioned AP
 *	   WiFi state_CONNECTING.
 *	   (APP)(INFO)Wifi State :: CONNECTED ::
 *	   WiFi state_SUCCESS.
 *	   (APP)(INFO)DHCP IP Address :: xxx.xxx.xxx.xxx ::
 *
 *     BLE Disconnected.
 *
 *     Provisioning Complete.
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

/*- Includes -----------------------------------------------------------------------*/

#include <asf.h>
#include "main.h"
#include "ble_manager.h"
#include "driver/include/m2m_wifi.h"
#include "driver/include/m2m_periph.h"
#include "m2m_ble.h"
#include "at_ble_api.h"
#include "provisioning_app.h"
#include "ble_profiles/wifi_provisioning/wifi_provisioning.h"

extern uint32 nmi_inet_addr(char *pcIpAddr);


#define STRING_HEADER "-- Wifi BLE Provisioning --\r\n" \
                      "-- "BOARD_NAME" --\r\n" \
                      "-- Compiled: "__DATE__" "__TIME__" --\r\n"

#define APP_WIFI_PROV_DISPLAY_NAME  ("3400 DEMO")

#define APP_BTN_EVENT_BTN1_SHORT_PRESS              1
#define APP_BTN_EVENT_BTN1_LONG_PRESS               2
#define APP_BTN_EVENT_BTN2_SHORT_PRESS              4
#define APP_BTN_EVENT_BTN2_LONG_PRESS               8

#define APP_STATE_IDLE								0
#define APP_STATE_WAITING_FOR_WIFI_DISCONNECTION	1
#define APP_STATE_PROVISIONING						2
#define APP_STATE_WAITING_FOR_BUTTON_PRESS			3
#define APP_STATE_WAITING_FOR_WIFI_CONNECTION		4
#define APP_STATE_WAITING_FOR_PROFIFE_SWITCH		5
#define APP_STATE_COMPLETE							6

#define CONNECTION_ATTEMPT_CONNECTING				0
#define CONNECTION_ATTEMPT_CONNECTED				1
#define CONNECTION_ATTEMPT_PROVISIONED				2
#define CONNECTION_ATTEMPT_DISCONNECTED				3
#define CONNECTION_ATTEMPT_FAILED					4
#define WIFI_CONNECT_TIMEOUT						15000

uint8_t app_state = APP_STATE_WAITING_FOR_WIFI_DISCONNECTION;

static uint8_t gu8ConnectionAttempt;
static volatile uint8 gu8WiFiConnectionState = M2M_WIFI_UNDEF;
static volatile uint8 gu8BtnEvent;
static uint8 gu8ScanIndex;
static at_ble_event_parameter_t gu8BleParam __aligned(4);


bool volatile timer_cb_done;
static uint8_t led_toggle;

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

static void app_wifi_init(tpfAppWifiCb wifi_cb_func)
{
	tstrWifiInitParam param;
	uint32 pinmask = (
	M2M_PERIPH_PULLUP_DIS_HOST_WAKEUP|
	M2M_PERIPH_PULLUP_DIS_SD_CMD_SPI_SCK|
	M2M_PERIPH_PULLUP_DIS_SD_DAT0_SPI_TXD);

	sint8 ret;

	uint8 mac_addr[6];
	uint8 u8IsMacAddrValid;
	uint8 deviceName[] = M2M_DEVICE_NAME;

#ifdef _STATIC_PS_
	nm_bsp_register_wake_isr(wake_cb, PS_SLEEP_TIME_MS);
#endif

	m2m_memset((uint8*)&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb_func;
#ifdef ETH_MODE
	param.strEthInitParam.pfAppEthCb = ethernet_demo_cb;
	param.strEthInitParam.au8ethRcvBuf = gau8ethRcvBuf;
	param.strEthInitParam.u16ethRcvBufSize = sizeof(gau8ethRcvBuf);
#endif
	ret = m2m_ble_wifi_init(&param);

	if (M2M_SUCCESS != ret)
	{
		M2M_ERR("Driver Init Failed <%d>\n",ret);
		M2M_ERR("Resetting\n");
		// Catastrophe - problem with booting. Nothing but to try and reset
		system_reset();

		while (1)
		{
		}
	}

	m2m_periph_pullup_ctrl(pinmask, 0);

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


	/* Name must be in the format WINC3400_00:00 */
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

#ifdef _DYNAMIC_PS_
	{
		tstrM2mLsnInt strM2mLsnInt;
		M2M_INFO("M2M_PS_DEEP_AUTOMATIC\r\n");
		m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);
		strM2mLsnInt.u16LsnInt = M2M_LISTEN_INTERVAL;
		m2m_wifi_set_lsn_int(&strM2mLsnInt);
	}
#elif (defined _STATIC_PS_)
	M2M_INFO("M2M_PS_MANUAL\r\n");
	m2m_wifi_set_sleep_mode(M2M_PS_MANUAL, 1);
#else
	M2M_INFO("M2M_NO_PS\r\n");
	m2m_wifi_set_sleep_mode(M2M_NO_PS, 1);
#endif
}

static void app_wifi_handle_event(uint8 u8MsgType, void * pvMsg)
{
	if (u8MsgType == M2M_WIFI_RESP_CON_STATE_CHANGED)
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged*) pvMsg;

		M2M_INFO("Wifi State :: %s ::\r\n", pstrWifiState->u8CurrState ? "CONNECTED" : "DISCONNECTED");
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
			gu8WiFiConnectionState = M2M_WIFI_DISCONNECTED;
		}
	}
	else if (u8MsgType == M2M_WIFI_REQ_DHCP_CONF)
	{
		tstrM2MIPConfig* pstrM2MIpConfig = (tstrM2MIPConfig*) pvMsg;
		uint8 *pu8IPAddress = (uint8*) &pstrM2MIpConfig->u32StaticIP;

		M2M_INFO("DHCP IP Address :: %u.%u.%u.%u ::\n",
			pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);

		gu8WiFiConnectionState = M2M_WIFI_CONNECTED;

	}
	else if (u8MsgType == M2M_WIFI_RESP_SCAN_DONE)
	{
		tstrM2mScanDone *pstrInfo = (tstrM2mScanDone*) pvMsg;

		if (gu8WiFiConnectionState != M2M_WIFI_CONNECTED)
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

		if (gu8WiFiConnectionState != M2M_WIFI_CONNECTED)
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
}

static void app_button_press_callback(uint8 btn, uint8 press)
{
	if (SW1 == btn)
	{
		gu8BtnEvent = (!press) ? (APP_BTN_EVENT_BTN1_SHORT_PRESS) : (APP_BTN_EVENT_BTN1_LONG_PRESS);
	}
	else if (SW2 == btn)
	{
		gu8BtnEvent = (!press) ? (APP_BTN_EVENT_BTN2_SHORT_PRESS) : (APP_BTN_EVENT_BTN2_LONG_PRESS);
	}
}

// This is an example of using onchip_profile, ble_prov API.
static void app_ble_wifi_provisioning(void)
{
	static at_ble_events_t evt;
	gu8BtnEvent =0; //Button Event
	uint8_t btn_event;
	gu8WiFiConnectionState = M2M_WIFI_DISCONNECTED;
	gu8ConnectionAttempt = CONNECTION_ATTEMPT_DISCONNECTED;
	at_ble_events_t ble_event;

	m2m_ble_init();
	ble_device_init(NULL);
	initialise_provisioning_app();

		while (1)
		{

			/* Pump event to BLE application */
			if (m2m_ble_event_get(&ble_event, &gu8BleParam) == AT_BLE_SUCCESS)
			{
				ble_event_manager(ble_event, &gu8BleParam);
			}

			#if 0
					//
					// Handle external (Button/WiFi) events for provisioning app.
					// This state machine runs until app_state becomes APP_STATE_COMPLETE.
					//
					btn_event = gu8BtnEvent;
					gu8BtnEvent = 0;

					if (btn_event == APP_BTN_EVENT_BTN1_LONG_PRESS && app_state != APP_STATE_IDLE)
					{
						M2M_INFO("Provisioning in process, aborting..\r\n");
						ble_prov_stop();

						M2M_INFO("Press button again to start provisioning\r\n");
						app_state = APP_STATE_IDLE;
						wifi_con_state = M2M_WIFI_UNDEF;
						btn_event = 0;
					}
			#endif

			switch (app_state)
			{
				case APP_STATE_WAITING_FOR_WIFI_DISCONNECTION:
				{
					if (gu8ConnectionAttempt == CONNECTION_ATTEMPT_DISCONNECTED)
					{
						if (start_provisioning_app()== AT_BLE_SUCCESS)
						{
							app_state = APP_STATE_PROVISIONING;
						}
					}
					break;
				}
				case APP_STATE_PROVISIONING:
				{
					switch (provisioning_app_processing())
					{
						case WIFI_PROVISION_STATE_HAVE_CREDENTIAL:
						{
							credentials mycred;
							M2M_INFO("Provisioning data received \r\n");
							if (retrieve_credentials(&mycred) != CREDENTIALS_VALID)
							{
								M2M_INFO("WiFi Connect: using default ssid");
								m2m_wifi_connect(DEFAULT_SSID, sizeof(DEFAULT_SSID),DEFAULT_AUTH, (void *) DEFAULT_KEY, M2M_WIFI_CH_ALL);
							}
							else
							{
								M2M_INFO("WiFi Connect: using provisioned AP");
								m2m_wifi_connect((char *)mycred.ssid, mycred.ssid_length,
								mycred.sec_type, mycred.passphrase, M2M_WIFI_CH_ALL);
							}
							wifi_state_update(WIFI_CONNECTION_STATE_CONNECTING);
							app_state = APP_STATE_WAITING_FOR_WIFI_CONNECTION;
							gu8ConnectionAttempt = CONNECTION_ATTEMPT_CONNECTING;
							nm_bsp_stop_timer();
							nm_bsp_start_timer(wifi_connect_timeout, WIFI_CONNECT_TIMEOUT);

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
				case APP_STATE_WAITING_FOR_WIFI_CONNECTION:
				{
					switch(gu8ConnectionAttempt)
					{
						case CONNECTION_ATTEMPT_CONNECTED:
						{
							gu8ConnectionAttempt = CONNECTION_ATTEMPT_PROVISIONED;
							wifi_state_update(WIFI_CONNECTION_STATE_PROVISIONED);
							nm_bsp_sleep(1000);
							provision_app_ble_disconnect();
							nm_bsp_sleep(1000);
							app_state = APP_STATE_IDLE;
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
							M2M_INFO("Hold SW0 for 2 sec for provisioning.");
							break;
						}
					}
					break;
				}
			}
		}
}

static void app_main(void)
{
	/* Initialize WiFi interface first.
	 3400 WiFi HIF is used to convey BLE API primitives.*/
	app_wifi_init(app_wifi_handle_event);
	nm_bsp_btn_init(app_button_press_callback);

	/* Demo application using profile.*/
	app_ble_wifi_provisioning();

}

int main (void)
{
	/* Initialize the board. */
	system_init();

	configure_console();
	puts(STRING_HEADER);

	nm_bsp_init();
	nm_bsp_app_init();

	app_main();

	while (1);
}
