/**
 * \file
 *
 * \brief wifi_con Service declarations
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
#ifndef __WIFI_CON_H__
#define __WIFI_CON_H__
#include "asf.h"
#include "ble_manager.h"

/**
@defgroup sdk Atmel BLE SDK

@{
*/

/** @brief WIFICON_UUID_LEN the size of WIFICON service uuid */
#define WIFICON_UUID_128_LEN					(16)
/** @brief WIFICON STATE NAME_LEN the  length of the device name */
#define CHAR_USER_STR_WIFICON_STATE_LEN			(18)
/* @brief WIFICON STATE NAME_DATA the actual name of device */
#define CHAR_USER_STR_WIFICON_STATE				("WiFi Connect State")
/** @brief WIFICON APPARAM NAME_LEN the  length of the device name */
#define CHAR_USER_STR_WIFICON_APPARAM_LEN		(13)
/* @brief WIFICON APPARAM NAME_DATA the actual name of device */
#define CHAR_USER_STR_WIFICON_APPARAM			("AP Parameters")
typedef struct wificon_gatt_service_handler
{
	/// service uuid
	at_ble_uuid_t	serv_uuid;
	/// service handle
	at_ble_handle_t	serv_handle;
	/// service characteristic
	at_ble_characteristic_t	serv_chars[2];
}wificon_gatt_service_handler_t;
/****************************************************************************************
*							        Macros	                                     		*
****************************************************************************************/
/** @brief count of included services in wifi_con service
  *
  */
#define WIFICON_INCLUDED_SERVICE_COUNT		(0)
/** @brief count of characteristics in wifi_con service
  *
  */
#define WIFICON_CHARACTERISTIC_COUNT		(2)
/** @brief status wifi_con of characteristics write
  *
  */
#define INVALID_WIFICON_CHANGE_PARAM		(0xff)
#define FAILED_WIFICON_CHANGE_PARAM			(0xee)
#define VALID_WIFICON_CHANGE_PARAM			(0xdd)
typedef struct gatt_wificon_char_handler
{
	at_ble_handle_t start_handle;
	at_ble_handle_t end_handle;
	at_ble_handle_t char_handle;
	at_ble_status_t char_discovery;
	uint8_t *char_data;
}gatt_wificon_char_handler_t;
/****************************************************************************************
*							        Function Declarations	                            *
****************************************************************************************/
/** @brief Initialize the wifi_con service with default values
  *
  *
  * @param[in] wificon_handle the service info which has handle (range,uuid and characteristic array fields)
  *
  * @pre Must be called before @ref wificon_primary_service_define
  *
  * @return void
  */
void init_wifi_con_service(wificon_gatt_service_handler_t *wificon_handle);

/** @brief Defining the wifi_con service to the attribute data base
  *
  * @param[in] wificon_primary_service  the service info which has handle (range,uuid and characteristic array fields)
  *
  * @pre Must be called after @ref init_wifi_con_service
  *
  * @return @ref AT_BLE_SUCCESS operation completed successfully
  * @return @ref AT_BLE_FAILURE Generic error.
  */
at_ble_status_t wificon_primary_service_define(wificon_gatt_service_handler_t *wificon_primary_service);

/** @brief notification on the wifi connecting progress
  * @param[in] wificon_handler the service info which has handle (range,uuid and characteristic array fields)
  * @param[in] s Connected state NOTIFY_STATE_PROVISIONFAILED, NOTIFY_STATE_WIFICONNECTING or NOTIFY_STATE_PROVISIONED
  * @return @ref AT_BLE_SUCCESS operation completed successfully
  * @return @ref AT_BLE_FAILURE Generic error.
  */
at_ble_status_t wificon_connect_noti(wificon_gatt_service_handler_t *wificon_handler, uint8_t s);

/** @brief wificon service state characteristics client config change
  * @param[in] param changed parameter
  * @param[in] wificon_handle the service info which has handle (range,uuid and characteristic array fields)
  * @return status of the change request
  */
uint8_t wificon_char_change_state_client_cfg(at_ble_characteristic_changed_t *param, wificon_gatt_service_handler_t *wificon_handle);

/** @brief wificon service apparam characteristics change
  * @param[in] param changed parameter
  * @param[in] wificon_handle the service info which has handle range,uuid and characteristic array fields
  * @return status of the change request
  */
uint8_t wificon_char_change_apparam(at_ble_characteristic_changed_t *param, wificon_gatt_service_handler_t *wificon_handle);

/** @}*/

#endif /* __WIFI_CON_H__ */
