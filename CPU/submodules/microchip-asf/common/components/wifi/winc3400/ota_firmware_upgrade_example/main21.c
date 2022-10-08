/**
 *
 * \file
 *
 * \brief WINC3400 OTA firmware upgrade example.
 *
 * Copyright (c) 2017-2019 Microchip Technology Inc. and its subsidiaries.
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
 * board to perform an OTA Firmware upgrade.
 * It uses the following hardware:
 * - the SAM Xplained Pro.
 * - the WINC3400 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC3400.
 *
 * \section usage Usage
 * -# You need to setup an HTTP server accessible via your Wi-Fi network. This HTTP
 * server must have the "m2m_ota_3400.bin" OTA firmware accessible at this location:
 * http://SERVER_IP_ADDRESS/m2m_ota_3400.bin
 *
 * Note:
 * - the OTA firmware file is included in this project under the firmware
 * folder.
 * - the server IP address must be set via the MAIN_OTA_URL macro from main.h file.
 *
 * -# Configure below code in the config main.h for AP to connect to.
 * \code
 *     #define MAIN_WLAN_SSID        "DEMO_AP"
 *     #define MAIN_WLAN_AUTH        M2M_WIFI_SEC_WPA_PSK
 *     #define MAIN_WLAN_PSK         "12345678"
 *     #define MAIN_OTA_URL          "http://192.168.1.11/m2m_ota_3400.bin"
 * \endcode
 *
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as the follows.
 *
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
 *    -- WINC3400 OTA firmware upgrade example --
 *    -- SAMXXX_XPLAINED_PRO --
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    Wi-Fi connected
 *    Wi-Fi IP is 192.168.0.10
 *    OtaUpdateCb 1 0
 *    (APP)(INFO)OTA download succeeded
 *    OtaUpdateCb m2m_ota_switch_firmware start.
 *    OtaUpdateCb 2 0
 *    OTA Success. Press reset your board.
 *
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

#include "asf.h"
#include "main.h"
#include "driver/include/m2m_ota.h"
#include "driver/include/m2m_wifi.h"
#include "driver/source/nmasic.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC3400 OTA firmware upgrade example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/** UART module for debug. */
static struct usart_module cdc_uart_module;

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	usart_conf.baudrate    = 115200;

	stdio_serial_init(&cdc_uart_module, EDBG_CDC_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_REQ_DHCP_CONF](@ref M2M_WIFI_REQ_DHCP_CONF)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("Wi-Fi disconnected\r\n");

			/* Connect to defined AP. */
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (void *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("Wi-Fi connected\r\n");
		printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		/* Start OTA Firmware download. */
		m2m_ota_start_update((uint8_t *)MAIN_OTA_URL);
		break;
	}

	default:
	{
		break;
	}
	}
}

/**
 * \brief Callback to get the OTA update event.
 *
 * \param[in] u8OtaUpdateStatusType type of OTA update status notification. Possible types are:
 * - [DL_STATUS](@ref DL_STATUS)
 * - [SW_STATUS](@ref SW_STATUS)
 * - [RB_STATUS](@ref RB_STATUS)
 * \param[in] u8OtaUpdateStatus type of OTA update status detail. Possible types are:
 * - [OTA_STATUS_SUCCESS](@ref OTA_STATUS_SUCCESS)
 * - [OTA_STATUS_FAIL](@ref OTA_STATUS_FAIL)
 * - [OTA_STATUS_INVALID_ARG](@ref OTA_STATUS_INVALID_ARG)
 * - [OTA_STATUS_INVALID_RB_IMAGE](@ref OTA_STATUS_INVALID_RB_IMAGE)
 * - [OTA_STATUS_INVALID_FLASH_SIZE](@ref OTA_STATUS_INVALID_FLASH_SIZE)
 * - [OTA_STATUS_AlREADY_ENABLED](@ref OTA_STATUS_AlREADY_ENABLED)
 * - [OTA_STATUS_UPDATE_INPROGRESS](@ref OTA_STATUS_UPDATE_INPROGRESS)
 */
static void OtaUpdateCb(uint8 u8OtaUpdateStatusType ,uint8 u8OtaUpdateStatus)
{
	sint8 s8tmp;

	printf("OtaUpdateCb %d %d\r\n", u8OtaUpdateStatusType, u8OtaUpdateStatus);

	switch(u8OtaUpdateStatusType) {
		case DL_STATUS:
		 if(u8OtaUpdateStatus == OTA_STATUS_SUCCESS)
		 {
			M2M_INFO("OTA download succeeded\n");
			s8tmp = m2m_wifi_check_ota_rb();
			if(s8tmp == M2M_ERR_FW_VER_MISMATCH) {
				//	In this case the application SHOULD update the host driver before calling
				//	@ref m2m_ota_switch_firmware(). Switching firmware image and resetting without updating host
				//	driver would lead to severely limited functionality (i.e. OTA rollback only).

				/* Start Host Controller OTA HERE ... Before switching.... */
			}
			else if(s8tmp == M2M_SUCCESS) {
				//	In this case the application MAY WANT TO update the host driver before calling
				//	@ref m2m_ota_switch_firmware(). Switching firmware image and resetting without updating host
				//	driver may lead to suboptimal functionality.

				/* Start Host Controller OTA HERE ... Before switching.... */
			}
			else {
				printf("Cannot recognize downloaded image\n");
				//	In this case the application MUST NOT update the host driver if such an update would change the
				//	driver HIF Major field. Firmware switch @ref using m2m_ota_switch_firmware() is blocked.
				break;
			}
			printf("OtaUpdateCb m2m_ota_switch_firmware start.\r\n");
			m2m_ota_switch_firmware();
		}
		else {
			printf("OtaUpdateCb FAIL u8OtaUpdateStatus %d\r\n", u8OtaUpdateStatus);
		}
		break;
		case SW_STATUS:
		case RB_STATUS:
		if(u8OtaUpdateStatus == OTA_STATUS_SUCCESS) {
			printf("OTA Success. Press reset your board.\r\n");
			/* system_reset(); */
		}
		break;
	}
}

/**
 * \brief OTA notify callback.
 *
 * OTA notify callback typedef.
 */
static void OtaNotifCb(tstrOtaUpdateInfo *pv)
{
	printf("OtaNotifCb \r\n");
}

/**
 * \brief Main application function.
 *
 * Application entry point.
 *
 * \return program return value.
 */
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	bool rollback_required = false;

	/* Initialize the board. */
	system_init();

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	/* Initialize the BSP. */
	nm_bsp_init();

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);

	if(ret == M2M_ERR_FW_VER_MISMATCH) {
		ret = m2m_wifi_check_ota_rb();
		if(ret == M2M_SUCCESS) {
			//	In this case the image in the inactive partition has compatible HIF. We will switch/rollback to it
			//	after initializing the OTA module.
			rollback_required = true;
		}
	}
	else if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}

    /* Init ota function. */
    m2m_ota_init(OtaUpdateCb, OtaNotifCb);

	if(rollback_required) {
		//	We need to call either @ref m2m_ota_rollback() or @ref m2m_ota_switch_firmware() (functionally equivalent).
		m2m_ota_rollback();
	}
	else {
	    /* Connect to defined AP. */
	    m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (void *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
	}

	while (1) {
		/* Handle pending events from network controller. */
		while (m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
		}
	}

	return 0;
}
