/**
 * \file
 *
 * \brief wifi_scan service declarations
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
#ifndef __WIFI_SCAN_H__
#define __WIFI_SCAN_H__
#include "asf.h"
#include "ble_manager.h"
#ifdef WIFI_PROVISIONING
#include "wifi_provisioning.h"
#endif

/**
@defgroup sdk Atmel BLE SDK

@{
*/

/** @brief results of wifiscan characteristics write
  *
  */
#define INVALID_WIFISCAN_CHANGE_PARAM			(0xff)
#define FAILED_WIFISCAN_CHANGE_PARAM			(0xee)
#define VALID_WIFISCAN_CHANGE_PARAM				(0xdd)
/** @brief WIFISCAN_UUID_LEN the size of WIFISCAN service uuid */
#define WIFISCAN_UUID_128_LEN					(16)
/** @brief WIFISCAN SCANMODE NAME_LEN the  length of the device name */
#define CHAR_USER_STR_WIFISCAN_SCANMODE_LEN		(18)
/* @brief WIFISCAN SCANMODE NAME_DATA the actual name of device */
#define CHAR_USER_STR_WIFISCAN_SCANMODE			("WiFi Scanning Mode")
/** @brief WIFISCAN APCOUNT NAME_LEN the  length of the device name */
#define CHAR_USER_STR_WIFISCAN_APCOUNT_LEN		(8)
/* @brief WIFISCAN APCOUNT NAME_DATA the actual name of device */
#define CHAR_USER_STR_WIFISCAN_APCOUNT			("AP Count")
/** @brief WIFISCAN APDETAILS NAME_LEN the  length of the device name */
#define CHAR_USER_STR_WIFISCAN_APDETAILS_LEN	(10)
/* @brief WIFISCAN APDETAILS NAME_DATA the actual name of device */
#define CHAR_USER_STR_WIFISCAN_APDETAILS		("AP Details")
typedef struct wifiscan_gatt_service_handler
{
	/// service uuid
	at_ble_uuid_t	serv_uuid;
	/// service handle
	at_ble_handle_t	serv_handle;
	/// service characteristic
	at_ble_characteristic_t	serv_chars[17];
}wifiscan_gatt_service_handler_t;
/****************************************************************************************
*							        Macros	                                     		*
****************************************************************************************/
/** @brief count of included service in wifi_scan service
  *
  */
#define WIFISCAN_INCLUDED_SERVICE_COUNT		(0)
/** @brief the two basic characteristics are scan mode and apcount
  *
  */
#define WIFISCAN_BASIC_CHARACTERISTIC		(2)
/** @brief count of characteristics in wifi_scan service
  *
  */
#ifdef WIFI_PROVISIONING
#define WIFISCAN_CHARACTERISTIC_COUNT		(WIFISCAN_BASIC_CHARACTERISTIC+WIFI_PROVISION_MAX_AP_NUM)
#endif
typedef struct gatt_wifiscan_char_handler
{
	at_ble_handle_t start_handle;
	at_ble_handle_t end_handle;
	at_ble_handle_t char_handle;
	at_ble_status_t char_discovery;
	uint8_t *char_data;
}gatt_wifiscan_char_handler_t;
/****************************************************************************************
*							        Function Declarations	                            *
****************************************************************************************/
/** @brief Initialize the wifi_scan service with default values
  *
  *
  * @param[in] wifiscan_handle  the service info which has handle range,uuid and characteristic array fields
  *
  * @pre Must be called before @ref wifiscan_primary_service_define
  *
  * @return void
  */
void init_wifi_scan_service(wifiscan_gatt_service_handler_t *wifiscan_handle);
/** @brief Defining the wifi_scan service to the attribute data base
  *
  *
  * @param[in] wifiscan_primary_service the service info which has handle range,uuid and characteristic array fields
  *
  * @pre Must be called after @ref init_wifi_scan_service
  *
  * @return @ref AT_BLE_SUCCESS operation completed successfully
  * @return @ref AT_BLE_FAILURE Generic error.
  */
at_ble_status_t wifiscan_primary_service_define(wifiscan_gatt_service_handler_t *wifiscan_primary_service);
/** @brief update wifiscan scan mode
  * @param[in] wifiscan_serv wifiscan service
  * @param[in] char_data characteristics data
  * @return @ref AT_BLE_SUCCESS operation completed successfully
  * @return @ref AT_BLE_FAILURE Generic error.
  */
at_ble_status_t wifiscan_update_scanmode_char_value (wifiscan_gatt_service_handler_t *wifiscan_serv , uint8_t char_data);

#ifdef WIFI_PROVISIONING//WIFI_PROVISIONING
/** @brief update wifiscan scan list
  * @param[in] wifiscan_serv wifiscan service
  * @param[in] param pointer to variable to be updated with number of APs found
  * @param[in] num_ap_found number of APs found
  * @return @ref AT_BLE_SUCCESS operation completed successfully
  * @return @ref AT_BLE_FAILURE Generic error.
  */
at_ble_status_t wifiscan_scanlist_receive(wifiscan_gatt_service_handler_t *wifiscan_serv , wifi_provision_scanlist *param, uint8_t *num_ap_found);
#endif
/** @brief wifiscan service scan mode characteristics change
  * @param[in] p characteristic changed
  * @param[in] wifiscan_handle the wifi scan handle
  * @return status of the change request
  */
uint8_t wifiscan_char_change_scanmode(at_ble_characteristic_changed_t *p, wifiscan_gatt_service_handler_t *wifiscan_handle);
/** @brief utility function to retrieve and print out characteristics (scanmode and ap_num)
  * @param[in] wifiscan_handler the wifi scan handler
  */
void wifiscan_print_char(wifiscan_gatt_service_handler_t *wifiscan_handler);

/** @}*/

#endif /* __WIFI_SCAN_H__ */
