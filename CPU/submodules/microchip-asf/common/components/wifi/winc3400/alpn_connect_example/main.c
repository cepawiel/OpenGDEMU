/**
 *
 * \file
 *
 * \brief ALPN Example.
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
 * This example demonstrates how to connect to a server via Application Layer Protocol
 * Negotiation (ALPN)
 * using the WINC3400 Wi-Fi module.<br>
 * It uses the following hardware:
 * - the SAM Xplained Pro.
 * - the WINC3400 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC3400 Wi-Fi module and connect to a server using SSL.
 *
 * \section usage Usage
 * -# The connection parameters can be configured in main.h.
 * \code
 *    #define MAIN_WLAN_SSID                  "DEMO_AP"
 *    #define MAIN_WLAN_AUTH                  M2M_WIFI_SEC_WPA_PSK
 *    #define MAIN_WLAN_PSK                   "12345678"
 * \endcode
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as following.
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
 *    -- ALPN example --
 *    -- SAMXXX_XPLAINED_PRO --
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    (APP)(INFO)Chip ID 3400d2
 *    (APP)(INFO)Curr driver ver: x.x.x
 *    (APP)(INFO)Curr driver HIF Level: (2) x.x
 *    (APP)(INFO)Fw HIF: 8104
 *    (APP)(INFO)Firmware HIF (2) : x.x
 *    (APP)(INFO)Firmware ver   : x.x.x
 *    (APP)(INFO)Firmware Build <Month> DD YYYY Time xx:xx:xx
 *    wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED
 *    wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is xxx.xxx.xxx.xxx
 *    Host IP is xxx.xxx.xxx.xxx
 *    Host Name is www.google.com
 *    Successfully connected.
 *    Negotiated HTTP/2.
 * \endcode
 *
 * \warning
 * \code
 *    For using SSL, the root certificate must be installed.
 *    Download the root certificate using the root_certificate_downloader from the
 *    firmware update project available in the ASF. (Refer to WINC3400 Software User Guide.)
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
#include <string.h>
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "driver/include/m2m_ssl.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- ALPN example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/** IP address of host. */
uint32_t gu32HostIp = 0;

uint8_t gu8SocketStatus = SocketInit;

/** TCP client socket handler. */
static SOCKET tcp_client_socket = -1;

/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;

/** Get host IP status variable. */
static bool gbHostIpByName = false;

/** Server host name. */
static char server_host_name[] = MAIN_HOST_NAME;

/** Secure socket connection start variable. */
static bool gbSocketConnectInit = false;

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief Creates and connects to an unsecure socket to be used for SSL.
 *
 * \param[in] None.
 *
 * \return SOCK_ERR_NO_ERROR if success, -1 if socket create error, SOCK_ERR_INVALID if socket connect error.
 */
static int8_t sslConnect(void)
{
	struct sockaddr_in addr_in;

	addr_in.sin_family = AF_INET;
	addr_in.sin_port = _htons(MAIN_HOST_PORT);
	addr_in.sin_addr.s_addr = gu32HostIp;

	/* Create secure socket */
	if (tcp_client_socket < 0) {
		tcp_client_socket = socket(AF_INET, SOCK_STREAM, SOCKET_FLAGS_SSL);
	}

	/* Check if socket was created successfully */
	if (tcp_client_socket == -1) {
		printf("socket error.\r\n");
		close(tcp_client_socket);
		return -1;
	}

	/*Configure the SNI for the socket */
	setsockopt(tcp_client_socket, SOL_SSL_SOCKET, SO_SSL_SNI, MAIN_HOST_NAME, sizeof(MAIN_HOST_NAME));

	/* Sets the protocol list to be used for ALPN */
	set_alpn_list(tcp_client_socket,"h2 http/1.1");

	/* If success, connect to socket */
	if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
		printf("connect error.\r\n");
		return SOCK_ERR_INVALID;
	}

	/* Success */
	return SOCK_ERR_NO_ERROR;
}

/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	gu32HostIp = hostIp;
	gbHostIpByName = true;
	printf("Host IP is %d.%d.%d.%d\r\n", (int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
			(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
	printf("Host Name is %s\r\n", hostName);
}

/**
 * \brief Callback function of TCP client socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg Type of Socket notification
 * \param[in] pvMsg Structure that contains notification information.
 *
 * \return None.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	/* Check for socket event on TCP socket. */
	if (sock == tcp_client_socket) {
		switch (u8Msg) {
		case SOCKET_MSG_CONNECT:
		{
			tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
			if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {

				printf("Successfully connected.\r\n");

				uint8  alpn_index = get_alpn_index(pstrConnect->sock);

				/* Check for ALPN negotiation type. */
				switch (alpn_index)
				{
					case 1:
					printf("Negotiated HTTP/2.\r\n");
					break;
					case 2:
					printf("Negotiated HTTP/1.1.\r\n");
					break;
					case 0:
					printf("Protocol negotiation did not occur.\r\n");
					break;
				}
			} else {
					if(get_error_detail(pstrConnect->sock,pstrConnect->s8Error) == SOCK_ERR_NO_ERROR)
					{
						printf("Connect error! code(%d)\r\n", pstrConnect->s8Error);
					}
					else
					{
						printf("Invalid argument is passed to socket function\r\n");
					}

					gu8SocketStatus = SocketError;

					close(pstrConnect->sock);
			}
		}
		break;

		default:
			break;
		}
	}
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
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			gbConnectedWifi = false;
			gbHostIpByName = false;
			gbSocketConnectInit = false;
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		/* Turn LED0 on to declare that IP address received. */
		printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		gbConnectedWifi = true;

		/* Obtain the IP Address by network name */
		gethostbyname((uint8_t *)server_host_name);
		break;
	}

	case M2M_WIFI_RESP_GET_SYS_TIME:
	{
		/* When the WINC connects to an AP it will automatically try to get the time
		 by contacting various NTP servers, and when it gets a response, the M2M_WIFI_RESP_GET_SYS_TIME
		 event is received on the host. You should only try to connect on a secure socket after receiving this event.*/
		gbSocketConnectInit = true;
	}

	default:
	{
		break;
	}
	}
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

	/* Initialize the board. */
	sysclk_init();
	board_init();

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
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}

	/* Initialize Socket module */
	socketInit();
	registerSocketCallback(socket_cb, resolve_cb);

	/* Connect to AP . */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
			MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	while (1) {
		m2m_wifi_handle_events(NULL);

		if (gbConnectedWifi && gbHostIpByName && gbSocketConnectInit) {
			if (gu8SocketStatus == SocketInit) {
				if (tcp_client_socket < 0) {
					gu8SocketStatus = SocketWaiting;
					if (sslConnect() != SOCK_ERR_NO_ERROR) {
						gu8SocketStatus = SocketInit;
					}
				}
			}
		}
	}

	return 0;
}
