/**
 *
 * \file
 *
 * \brief STA Task.
 *
 * Copyright (c) 2018-2020 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "asf.h"
#include "FreeRTOS.h"
#include "osprintf.h"
#include "sta.h"
#include "lwip/ip.h"
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/dns.h"
#include "os/include/net_init.h"
#include "os/include/m2m_wifi_ex.h"
#include "tinyservices.h"
#include "conf_wilc.h"
#include <string.h>
#include <stdio.h>

/** Using broadcast address for simplicity. */
#define HTTP_PORT					(80)

/** Delay on close error. */
#define webSHORT_DELAY				(10)

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** Wi-Fi status variable. */
static volatile bool gbConnectedWifi = false;

/** Get host IP status variable. */
static volatile bool gbHostIpByName = false;

/** Server host name. */
static char server_host_name[] = STA_WEATHER_SERVER_NAME;

/** Cities list. */
static const char *city_list[] = {
	"Paris", 
	"Rome", 
	"Berlin", 
	"Cairo", 
	"Windhoek", 
	"Chennai", 
	"Hyderabad", 
	"Mumbai", 
	"Bangkok", 
	"Shanghai", 
	"Tokyo"
};

/** Weather forecast server HTTP response. */
static char server_response[2048];

/** Built up report to display on HTML page for connected STA. */
char report[512];

/** Number of STA connected. */
uint32_t sta_connected = 0;
tstrM2MAPAssocInfo strApAssocInfo;

tstrM2mWifiWepParams  gstrSTAWepParam = WEP_CONN_PARAM;
tstrM2mWifiWepParams  gstrAPWepParam = WEP_CONN_PARAM;

/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(const char *hostName, ip_addr_t *ipaddr, void *callback_arg)
{
	gu32HostIp = ipaddr->addr;
	gbHostIpByName = true;
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType Type of Wi-Fi notification.
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters.
 *
 * \return None.
 */
static void wifi_cb(uint8 msg_type, void *msg)
{
	switch (msg_type) {
		case M2M_WIFI_RESP_CON_STATE_CHANGED : {
			tstrM2mWifiStateChanged *ctx = (tstrM2mWifiStateChanged*)msg;
			osprintf("wifi_cb: %s Ifc %s state %02x-%02x-%02x-%02x-%02x-%02x, error: %s, %d\r\n",
						ctx->u8IfcId == STATION_INTERFACE? "STA":
						ctx->u8IfcId == AP_INTERFACE? "AP":
						ctx->u8IfcId == P2P_INTERFACE? "P2P":"Unknown",
						ctx->u8CurrState == M2M_WIFI_DISCONNECTED ? "Disconnected":
						ctx->u8CurrState == M2M_WIFI_CONNECTED ? "Connected" : "Unknown",
							ctx->u8MAcAddr[0], ctx->u8MAcAddr[1], ctx->u8MAcAddr[2],
							ctx->u8MAcAddr[3], ctx->u8MAcAddr[4], ctx->u8MAcAddr[5],
							ctx->u8ErrCode == M2M_ERR_NONE ? "None":
							ctx->u8ErrCode == M2M_ERR_AP_NOT_FOUND ? "M2M_ERR_AP_NOT_FOUND":
							ctx->u8ErrCode == M2M_ERR_AUTH_FAIL ? "M2M_ERR_AUTH_FAIL":
							ctx->u8ErrCode == M2M_ERR_ASSOC_FAIL ? "M2M_ERR_ASSOC_FAIL":
							ctx->u8ErrCode == M2M_ERR_LINK_LOSS ? "M2M_ERR_LINK_LOSS":
							ctx->u8ErrCode == M2M_ERR_STATION_IS_LEAVING ? "M2M_ERR_STATION_IS_LEAVING": 							
							ctx->u8ErrCode == M2M_ERR_AP_OVERLOAD ? "M2M_ERR_AP_OVERLOAD":
							ctx->u8ErrCode == M2M_ERR_SEC_CNTRMSR ? "M2M_ERR_SEC_CNTR_MSR":"UNKNOWN", ctx->u8ErrCode);
							
			if (ctx->u8IfcId == STATION_INTERFACE) {
				if (ctx->u8CurrState == M2M_WIFI_CONNECTED) {
					osprintf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
					net_interface_up(NET_IF_STA);
					m2m_wifi_request_dhcp_client_ex();
					// Power save will not be enabled by the FW since AP will be running for this app
					//os_m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, true);
				}
				if(ctx->u8CurrState == M2M_WIFI_DISCONNECTED) {
					tuniM2MWifiAuth sta_auth_param;
					gbConnectedWifi = false;
					osprintf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
					osprintf("wifi_cb: reconnecting...\r\n");
					net_interface_down(NET_IF_STA);
					if (STA_WLAN_AUTH != M2M_WIFI_SEC_WEP) {
						strcpy((char*)sta_auth_param.au8PSK, STA_WLAN_PSK);
						os_m2m_wifi_connect((char *)STA_WLAN_SSID, sizeof(STA_WLAN_SSID),
							STA_WLAN_AUTH, &sta_auth_param, M2M_WIFI_CH_ALL);
					} else {
						sta_auth_param.strWepInfo = gstrSTAWepParam;
						os_m2m_wifi_connect((char *)STA_WLAN_SSID, sizeof(STA_WLAN_SSID),
							STA_WLAN_AUTH, &sta_auth_param, M2M_WIFI_CH_ALL);
					}
				}
			}
			else if (ctx->u8IfcId == AP_INTERFACE) {
				if (ctx->u8CurrState == M2M_WIFI_CONNECTED) {
					osprintf("wifi_cb: AP M2M_WIFI_CONNECTED %02x-%02x-%02x-%02x-%02x-%02x\r\n",
							ctx->u8MAcAddr[0], ctx->u8MAcAddr[1], ctx->u8MAcAddr[2],
							ctx->u8MAcAddr[3], ctx->u8MAcAddr[4], ctx->u8MAcAddr[5]);
					sta_connected += 1;
				}
				if (ctx->u8CurrState == M2M_WIFI_DISCONNECTED) {
					osprintf("wifi_cb: AP M2M_WIFI_DISCONNECTED %02x-%02x-%02x-%02x-%02x-%02x, reason: %s, %d\r\n",
							ctx->u8MAcAddr[0], ctx->u8MAcAddr[1], ctx->u8MAcAddr[2],
							ctx->u8MAcAddr[3], ctx->u8MAcAddr[4], ctx->u8MAcAddr[5],
							ctx->u8ErrCode == M2M_ERR_STATION_IS_LEAVING ? "M2M_ERR_STATION_IS_LEAVING": 
							ctx->u8ErrCode == M2M_ERR_LINK_LOSS ? "M2M_ERR_LINK_LOSS":
							ctx->u8ErrCode == M2M_ERR_AUTH_FAIL ? "M2M_ERR_AUTH_FAIL":"UNKNOWN", ctx->u8ErrCode);
					/* Check the reason for STA disconnection */ 
					switch (ctx->u8ErrCode) {
						case M2M_ERR_STATION_IS_LEAVING:
						case M2M_ERR_LINK_LOSS:
							lwip_dhcp_unregister_mac(ctx->u8MAcAddr);
							sta_connected -= 1;
							break;
						default:
							break;
					}
				}
			}	
			else if (ctx->u8IfcId == P2P_INTERFACE) {
				if (ctx->u8CurrState == M2M_WIFI_CONNECTED) {
					osprintf("wifi_cb: P2P: M2M_WIFI_CONNECTED\r\n");
					net_interface_up(NET_IF_STA);
					m2m_wifi_request_dhcp_client_ex();
				}
				if(ctx->u8CurrState == M2M_WIFI_DISCONNECTED) {
					gbConnectedWifi = false;
					osprintf("wifi_cb: P2P: M2M_WIFI_DISCONNECTED\r\n");
					net_interface_down(NET_IF_STA);
				}
			}
		}
		break;

		case NET_IF_REQ_DHCP_CONF : {
			tstrM2MIPConfig2 *strIpConfig = msg;
			uint16_t *a = (void *)strIpConfig->u8StaticIPv6;
			osprintf("wifi_cb: STA M2M_WIFI_REQ_DHCP_CONF\r\n");
			osprintf("wifi_cb: STA IPv4 addr: %d.%d.%d.%d\r\n", strIpConfig->u8StaticIP[0], strIpConfig->u8StaticIP[1],
					strIpConfig->u8StaticIP[2], strIpConfig->u8StaticIP[3]);
			osprintf("wifi_cb: STA IPv6 addr: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\r\n",
				htons(a[0]), htons(a[1]), htons(a[2]), htons(a[3]),
				htons(a[4]), htons(a[5]), htons(a[6]), htons(a[7]));
			gbConnectedWifi = true;
			
			/* Obtain the IP Address by network name. */
			static ip_addr_t resolve_addr;
			dns_gethostbyname(server_host_name, &resolve_addr, resolve_cb, 0);
		}
		break;

		case M2M_WIFI_RESP_AP_ASSOC_INFO:{
			tstrM2MAPAssocInfo *pstrApAssocInfo = (tstrM2MAPAssocInfo *)msg;
			memcpy(&strApAssocInfo, pstrApAssocInfo, sizeof(tstrM2MAPAssocInfo));
		}
		break;
		

		default:
		break;
	}
}

/**
 * \brief Parse weather server HTTP response.
 *
 * \param[in] buffer actual response.
 * \param[in] len response length.
 *
 * \return None.
 */
static void parse_response(char *buffer, uint32_t len)
{
	char *pcIndxPtr = 0;
	char *pcEndPtr = 0;
	char *pcEndPtr2 = 0;

	/* Check for error. */
	if (NULL == strstr(buffer, "200 OK")) {
		strcpy(report, "\r\nService not available at the moment.");
		return;
	}
	
	/* Get city. */
	strcpy(report, "\r\nCity: ");
	pcIndxPtr = strstr(buffer, "name=");
	if (NULL != pcIndxPtr) {
		pcIndxPtr = pcIndxPtr + strlen("name=") + 1;
		pcEndPtr = strstr(pcIndxPtr, "\">");
		if (NULL != pcEndPtr) {
			*pcEndPtr = 0;
		}
		strcat(report, pcIndxPtr);
	}
	else {
		strcat(report, "N/A\r\n");
	}

	/* Get temperature */
	pcIndxPtr = strstr(pcEndPtr + 1, "temperature value");
	strcat(report, "\r\nTemperature: ");
	if (NULL != pcIndxPtr) {
		pcIndxPtr = pcIndxPtr + strlen("temperature value") + 2;
		pcEndPtr = strstr(pcIndxPtr, "\"");
		pcEndPtr2 = strstr(pcIndxPtr, ".");
		if (pcEndPtr2 && pcEndPtr2 < pcEndPtr) {
			pcEndPtr = pcEndPtr2;
		}
		if (NULL != pcEndPtr) {
			*pcEndPtr = 0;
		}
		strcat(report, pcIndxPtr);
		strcat(report, "�C");
	}
	else {
		strcat(report, "N/A\r\n");
	}

	/* Get weather condition */
	pcIndxPtr = strstr(pcEndPtr + 1, "weather number");
	if (NULL != pcIndxPtr) {
		strcat(report, "\r\nConditions: ");
		pcIndxPtr = pcIndxPtr + strlen("weather number") + 14;
		pcEndPtr = strstr(pcIndxPtr, "\" ");
		if (NULL != pcEndPtr) {
			*pcEndPtr = 0;
		}
		strcat(report, pcIndxPtr);
	}
	else {
		strcat(report, "N/A\r\n");
	}
}

/**
 * \brief STA task main function.
 */
void sta_task(void *argument)
{
    xSemaphoreTake(wifi_bt_if_sem, portMAX_DELAY); /* Semaphore to synchronize BT &WiFi FW download */
	struct netconn *conn = NULL;
	struct ip_addr local_ip;
	struct ip_addr remote_ip;
	struct netbuf *rx_buf;
	char *str;
	char *tmp;
	uint16_t tot_len, len;
	uint8_t request[256];
	uint8_t index = 0;
	tuniM2MWifiAuth sta_auth_param;
	tstrM2MAPConfig cfg;

	/* Initialize the network stack. */
	net_init();

	/* Initialize the WILC1000 driver. */
	tstrWifiInitParam param;
	memset(&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb;
	os_m2m_wifi_init(&param);

	
	/* Enable AP mode. */
	memset(&cfg, 0, sizeof(cfg));
	strcpy((char *)cfg.au8SSID, AP_WLAN_SSID);
	cfg.u8ListenChannel = M2M_WIFI_CH_11;
	cfg.u16BeaconInterval = 0;
	cfg.u8SecType = AP_WLAN_AUTH;
	if (cfg.u8SecType == M2M_WIFI_SEC_WEP) {
		cfg.uniAuth.strWepInfo = gstrAPWepParam;
	} else {
		strcpy((char *)cfg.uniAuth.au8PSK, AP_WLAN_PSK);
	}
	os_m2m_wifi_enable_ap(&cfg);

	xSemaphoreGive(wifi_bt_if_sem);	/* Semaphore to synchronize BT &WiFi FW download */
#if 0
//	os_m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC,1);
os_m2m_wifi_set_device_name("Direct_WILC3000",strlen("Direct_WILC3000"));
os_m2m_wifi_set_p2p_control_ifc(P2P_AP_CONCURRENCY_INTERFACE);
os_m2m_wifi_p2p(M2M_WIFI_CH_11);
#endif

	/* Connect to station. */
	if (STA_WLAN_AUTH != M2M_WIFI_SEC_WEP) {
		strcpy((char*)sta_auth_param.au8PSK, STA_WLAN_PSK);
		os_m2m_wifi_connect((char *)STA_WLAN_SSID, sizeof(STA_WLAN_SSID),
			STA_WLAN_AUTH, &sta_auth_param, M2M_WIFI_CH_ALL);
	} else {
		sta_auth_param.strWepInfo = gstrSTAWepParam;
		os_m2m_wifi_connect((char *)STA_WLAN_SSID, sizeof(STA_WLAN_SSID),
			STA_WLAN_AUTH, &sta_auth_param, M2M_WIFI_CH_ALL);
	}
	
	//m2m_wifi_set_sleep_mode(M2M_NO_PS);
	//os_m2m_wifi_set_sleep_mode(M2M_NO_PS,0);


	while (1) {
		
		/* Ensure we are connected to AP. */
		if (gbConnectedWifi && gbHostIpByName) {

			/* Create TCP socket. */
			conn = netconn_new(NETCONN_TCP);
			if (conn == NULL) {
				osprint("sta_task: failed to create socket!\r\n");
				vTaskDelay(1000);
				continue;
			}
			
			/* Bind socket. */
			local_ip.addr = 0;
			if (netconn_bind(conn, &local_ip, 0) != ERR_OK) {
				osprint("sta_task: failed to bind socket!\r\n");
				netconn_delete(conn);
				vTaskDelay(1000);
				continue;
			}
						
			/* Connect socket. */
			remote_ip.addr = gu32HostIp;
			if (netconn_connect(conn, &remote_ip, HTTP_PORT) != ERR_OK) {
				osprint("sta_task: failed to connect socket!\r\n");
				netconn_delete(conn);
				vTaskDelay(1000);
				continue;
			}
			
			/* Send weather forecast request. */
			memset(request, 0, sizeof(request));
			sprintf((char *)request, "%s%s%s", STA_PREFIX_BUFFER, city_list[index], STA_POST_BUFFER);
			index = (index + 1) % (sizeof(city_list) / sizeof (char *));
			netconn_write(conn, (char *)request, strlen((char *)request), NETCONN_COPY);

			/* Wait for server response. */
			tmp = server_response;
			tot_len = 0;
			while (netconn_recv(conn, &rx_buf) != ERR_OK) {
				vTaskDelay(webSHORT_DELAY);
			}
			do {
				netbuf_data(rx_buf, (void *)&str, &len);
				memcpy(tmp, str, len);
				tmp += len;
				tot_len += len;
			}
			while(netbuf_next(rx_buf) >= 0 && tot_len < 2048);
			parse_response(server_response, tot_len);

			netbuf_delete(rx_buf);
			netconn_close(conn);
			netconn_delete(conn);

			os_m2m_wifi_ap_get_assoc_info();
			///* Trigger another request in few seconds. */
			//vTaskDelay(3000);
			vTaskDelay(3000);
		}
	}
}
