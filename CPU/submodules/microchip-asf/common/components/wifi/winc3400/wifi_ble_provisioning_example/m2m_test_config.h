/**
 *
 * \file m2m_test_config.h
 *
 * \brief Configurations and definitions of different test applications.
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

#ifndef __M2M_TEST_CONFIG_H__
#define __M2M_TEST_CONFIG_H__

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "common/include/nm_common.h"
#include "driver/include/m2m_wifi.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#define M2M_SEC_PARAM_MAX				M2M_MAX(M2M_PSK_MAX_LENGTH, sizeof(tstr1xAuthCredentials))

#ifdef ETH_MODE
#define LIST_CONNECT
#define CONF_STATIC_IP_ADDRESS
#endif

typedef struct{
	uint8	au8Ssid[M2M_MAX_SSID_LEN];
	uint8	*pu8AuthCred;
	uint8	u8AuthType;
} tstrM2mAp;

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifndef OTA_HTTPS
#define OTA_URL			"http://192.168.1.101:8080/m2m_ota_3a0.bin"
#else
#define OTA_URL			"https://192.168.1.100:4433/m2m_ota_3a0.bin"
#endif
/*************************
*    AP CONFIGURATIONS   *
**************************/


#define NMI_M2M_AP					"WINC3400_00:00"
#define NMI_M2M_AP_SEC				M2M_WIFI_SEC_OPEN
#define NMI_M2M_AP_WEP_KEY_INDEX	M2M_WIFI_WEP_KEY_INDEX_1
#define NMI_M2M_AP_WEP_KEY			"1234567890"
#define NMI_M2M_AP_SSID_MODE		SSID_MODE_VISIBLE
#define NMI_M2M_AP_CHANNEL			M2M_WIFI_CH_1

#define HTTP_PROV_SERVER_DOMAIN_NAME	"atmelconfig.com"
#define HTTP_PROV_SERVER_IP_ADDRESS		{192, 168, 1, 1}

/*************************
* STATION CONFIGURATIONS *
**************************/

#define M2M_DEVICE_NAME				"WINC3400_00:00"
#define MAC_ADDRESS					{0xf8, 0xf0, 0x05, 0x45, 0xD4, 0x84}
/*The BLE PIN is 6 digits, each one in the range 0-9 only*/
#define BLE_PIN                     {1, 2, 3, 4, 5, 6}
#define GAIN_TABLE_FROM_FLASH       0

/*********** Wi-Fi Enterprise Settings ***********
# RADIUS Server
	RADIUS IP Address    : 23.23.234.126
	RADIUS Port			 : 3670
	RADIUS Shared Secret : 8NXZg4DPq8QVqdxe1BUfcp

# Access Credentials
	User-Name : Nmi_M2m_Usr
	Password  : M2M_PAssworD
**************************************************/

#if 1
#define M2M_802_1X_USR_NAME						"Nmi_M2m_Usr"
#define M2M_802_1X_PWD							"M2M_PAssworD"
#else
#define M2M_802_1X_USR_NAME						"NewPortMediaInc"
#define M2M_802_1X_PWD							"M2m&!PAssWord#9267"
#endif

/***********************************************************************
* DEFAULT SSID CREDENTIALS CONFIGURATIONS OF THE PEER ACCESS POINT *
************************************************************************/
#define DEFAULT_SSID				"DEMO_AP"
#define DEFAULT_AUTH				M2M_WIFI_SEC_WPA_PSK
#define	DEFAULT_KEY					"12345678"

#define WEP_KEY_INDEX				M2M_WIFI_WEP_KEY_INDEX_1
#define WEP_KEY						"1234567890"
#define WEP_KEY_SIZE				sizeof(WEP_KEY)


#define WEP_CONN_PARAM     { WEP_KEY_INDEX, WEP_KEY_SIZE, WEP_KEY}

#define AUTH_CREDENTIALS   { M2M_802_1X_USR_NAME, M2M_802_1X_PWD }


#define AP_LIST													\
{																\
	{ DEFAULT_SSID, (uint8*)DEFAULT_KEY, DEFAULT_AUTH },		\
	{"DEMO_AP1", (uint8*)"",             M2M_WIFI_SEC_OPEN},	\
	{"DEMO_AP2", (uint8*)&gstrWepParams, M2M_WIFI_SEC_WEP},		\
	{"DEMO_AP3", (uint8*)"12345678",     M2M_WIFI_SEC_WPA_PSK},	\
	{"DEMO_AP4", (uint8*)&gstrCred1x,    M2M_WIFI_SEC_802_1X},	\
	/* Add more access points to the scan list here... */		\
}




#ifdef LIST_CONNECT
#define M2M_AP_LIST_SZ (sizeof(gastrPreferredAPList)/sizeof(tstrM2mAp))
#endif


/***************************
* GROWL APP CONFIGURATIONS *
****************************/
#ifdef GROWL
#undef UDP_TEST
#undef UDP_DEMO
#if (!defined NMA)&&(!defined PROWL)
#error "Please defined at least one app"
#endif

#define PROWL_API_KEY				"6ce3b9ff6c29e5c5b8960b28d9e987aec5ed603a"
#define NMA_API_KEY					"e6b71841e68eb03053dc9a45f006f2cdfe2048a6f4527e1c"

#define SSL_CONNECTION				1
#define NORMAL_CONNECTION			0

#ifdef __TLS_X509_BYPASS_TEST__
#define PROWL_CONNECTION_TYPE		SSL_CONNECTION
#else
#define PROWL_CONNECTION_TYPE		SSL_CONNECTION
#endif

#define NMA_CONNECTION_TYPE			SSL_CONNECTION


//#define AUTOMATED_TEST
#ifdef AUTOMATED_TEST
#ifdef _STATIC_PS_
#error "please undef _STATIC_PS_"
#endif
#define GROWL_N_TESTS				10
#endif /* AUTOMATED_TEST */

#endif /* GROWL */

/*******************************
* PS_SERVER APP CONFIGURATIONS *
********************************/
#define M2M_SERVER_CHANNEL		M2M_WIFI_CH_6

#ifdef _PS_SERVER_
#ifdef _PS_CLIENT_
#error "You can't defined _PS_SERVER_ & _PS_CLIENT_ together"
#endif /* _PS_CLIENT_ */
#endif /* _PS_SERVER_ */

/******************************
* UDP TEST APP CONFIGURATIONS *
*******************************/

#ifdef UDP_TEST
extern uint8 gbUdpTestActive;
#ifdef LED_BTN_DEMO
#undef LED_BTN_DEMO
#endif
#ifdef _STATIC_PS_
#error "please undef _STATIC_PS_"
#endif
#ifdef _DYNAMIC_PS_
#error "please undef _DYNAMIC_PS_"
#endif
#define UDP_SERVER_PORT				9000
#define UDP_CLIENT_PORT				9002
#define TCP_SERVER_PORT				20000

#ifdef WIN32
#define TEST_PACKET_COUNT			250
#else
#define TEST_PACKET_COUNT			1000
#endif
#endif /* UDP_TEST */


/**********************************
* UDP LED-TEMP APP CONFIGURATIONS *
**********************************/
#ifdef UDP_DEMO

#ifdef LED_BTN_DEMO
#undef LED_BTN_DEMO
#endif
#endif /* UDP_DEMO */

#define DEMO_PRODUCT_NAME					"WINC3400_DEMO"
#define BROADCAST							"255.255.255.255"
#define DEMO_SERVER_IP                      "192.168.2.3"
#define DEMO_SERVER_PORT					(6666)
#define DEMO_REPORT_INTERVAL				(1000)
#define CLIENT_KEEPALIVE					"0001,"DEMO_PRODUCT_NAME","
#define CLIENT_REPORT						"0002,"DEMO_PRODUCT_NAME",00.0,0,"
#define SERVER_REPORT						"0002,"DEMO_PRODUCT_NAME","



#if defined SAMD21_XPLAINED_PRO
#define CONF_DEMO_LED_PIN                   PIN_PB12
#else
//#error Please define CONF_DEMO_LED_PIN
#endif


#define CLIENT_RPOS_TEMP					(sizeof(CLIENT_REPORT) - 8)
#define CLIENT_RPOS_LED						(sizeof(CLIENT_REPORT) - 3)
#define SERVER_RPOS_LED						(sizeof(SERVER_REPORT) - 1)

#define GRWOL_REPORT						""DEMO_PRODUCT_NAME" = 00.0"
#define GROWL_RPOS_TEMP						(sizeof(GRWOL_REPORT) - 5)


/***************************
* STATIC IP CONFIGURATIONS *
****************************/

#ifdef CONF_STATIC_IP_ADDRESS
#define STATIC_IP_ADDRESS				"192.168.1.100"
#define DNS_ADDRESS						"192.168.1.1"
#define DEFAULT_GATEWAY_ADDRESS			"192.168.1.1"
#define SUBNET_MASK						"255.255.255.0"
#endif /* CONF_STATIC_IP_ADDRESS */


#define WPS_DISABLED				0
#define WPS_ENABLED					1

#define PROV_DISABLED				0
#define PROV_ENABLED				1


#define PS_WAKE       				0
#define PS_SLEEP      				1
#define PS_REQ_SLEEP				3
#define PS_REQ_CHECK				4
#define PS_SLEEP_TIME_MS			30000 /*30sec*/

#define TEST_BUS_BUF_SZ				100U
#define SHARED_PKT_MEM_BASE			0xd0000UL
#define DEFAULT_PREFIX				"NMI"

#define M2M_CLIENT_CMD_WAKE_FIRMWARE    ((uint8)10)
#define M2M_CLIENT_CMD_LED_ON 			((uint8)12)
#define M2M_CLIENT_CMD_LED_OFF 			((uint8)11)

/**/
#define M2M_CLIENT_RESP_MOVEMENT 		((uint8)20)
#define M2M_CLIENT_RESP_BTN1_PRESS 		((uint8)21)
#define M2M_CLIENT_RESP_BTN2_PRESS 		((uint8)22)
#define M2M_CLIENT_CHECK_STATE			((uint8)0)

#ifdef _STATIC_PS_
#ifdef _DYNAMIC_PS_
#error "you cant not define poth power save modes together"
#endif
#endif

#endif
