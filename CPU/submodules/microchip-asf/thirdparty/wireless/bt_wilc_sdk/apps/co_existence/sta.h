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

#ifndef APP_H_INCLUDED
#define APP_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/** Wi-Fi Settings. */
#define STA_WLAN_SSID                "DEMO_AP" /* < Destination SSID */
#define STA_WLAN_AUTH                M2M_WIFI_SEC_WPA_PSK /* < Security manner */
#define STA_WLAN_PSK                 "12345678" /* < Password for Destination SSID */

/* Semaphore to synchronize BT &WiFi FW download */
xSemaphoreHandle wifi_bt_if_sem;

/** WEP security setting for STA mode and/or AP mode */
#define WEP_KEY_INDEX            M2M_WIFI_WEP_KEY_INDEX_1
#define WEP_KEY                  "1234567890"
#define WEP_KEY_SIZE             sizeof(WEP_KEY)
#define WEP_AUTH_TYPE            WEP_ANY
#define WEP_CONN_PARAM           {WEP_KEY_INDEX, WEP_KEY_SIZE, WEP_KEY, WEP_AUTH_TYPE}
/* Note : In case of using WEP security on both STA and AP interfaces, the same
password should be used for both interfaces. In addition to that, the same
WEP authentication type (OPEN, SHARED, or ANY) should be used for both
STA and AP interfaces.*/

/** Wi-Fi Settings - AP mode. */
#define AP_WLAN_SSID                "DEMO_AP_CO" /* < WILC SSID */
#define AP_WLAN_AUTH                M2M_WIFI_SEC_OPEN/* < Security manner M2M_WIFI_SEC_OPEN, M2M_WIFI_SEC_WPA_PSK*/
#define AP_WLAN_PSK                 "1234567890" /* < Password for WILC SSID */

/** Send buffer of TCP socket. */
#define STA_PREFIX_BUFFER            "GET /data/2.5/weather?q="
#define STA_POST_BUFFER              "&appid=c592e14137c3471fa9627b44f6649db4&mode=xml&units=metric HTTP/1.1\r\nHost: api.openweathermap.org\r\nAccept: */*\r\n\r\n"

/** Weather information provider server. */
#define STA_WEATHER_SERVER_NAME      "api.openweathermap.org"

void sta_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_INCLUDED */
