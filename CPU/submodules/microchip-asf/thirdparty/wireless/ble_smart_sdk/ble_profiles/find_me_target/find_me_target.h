/**
 * \file
 *
 * \brief Find Me Profile declarations
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
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
 * Support</a>
 */

// <<< Use Configuration Wizard in Context Menu >>>
// <h> Find Me Target Profile Configuration
// =======================

#ifndef __FIND_ME_TARGET_H__
#define __FIND_ME_TARGET_H__


/****************************************************************************************
*							        Includes	
*                                                                                       *
****************************************************************************************/


/****************************************************************************************
*							        Macros	
*                                                                                       *
****************************************************************************************/
/** @brief PATHLOSS which defines if the user wants pathloss functionality*/

/** @brief APP_FMP_FAST_ADV between 0x0020 and 0x4000 in 0.625 ms units (20ms to
 *10.24s). */
//	<o> Fast Advertisement Interval <100-1000:50>
//	<i> Defines inteval of Fast advertisement in ms.
//	<i> Default: 100
//	<id> fmp_sensor_fast_adv
#define APP_FMP_FAST_ADV				(1600) /* 1000 ms */

/** @brief APP_FMP_ADV_TIMEOUT Advertising time-out between 0x0001 and 0x028F in 
 *seconds, 0x0000 disables time-out.*/
//	<o> Advertisement Timeout <1-655>
//	<i> Defines interval at which advertisement timeout in sec.
//	<i> Default: 655
//	<id> fmp_adv_timeout
#define APP_FMP_ADV_TIMEOUT				(655) /* 10 min */

/** @brief scan_resp_len is the length of the scan response data */
//	<o> Scan Response Buffer <1-20>
//	<i> Defines size of buffer for scan response.
//	<i> Default: 10
//	<id> fmp_scan_resp_len
#define SCAN_RESP_LEN					(10)

/** @brief ADV_DATA_LEN */
#define ADV_DATA_LEN					(18)

/** @brief ADV_TYPE_LEN */
#define ADV_TYPE_LEN					(0x01)

/** @brief IAL_ADV_DATA_UUID_LEN the size of IAL service uuid */
#define IAL_ADV_DATA_UUID_LEN			(2)

/** @brief IAL_ADV_DATA_UUID_TYPE the total sizeof IAL service uuid*/
#define IAL_ADV_DATA_UUID_TYPE			(0x03)

/** @brief FMP_ADV_DATA_NAME_LEN the  length of the device name */
#define FMP_ADV_DATA_NAME_LEN			(9)

/** @brief FMP_ADV_DATA_NAME_TYPE the gap ad data type */
#define FMP_ADV_DATA_NAME_TYPE			(0x09)

/* @brief FMP_ADV_DATA_NAME_DATA the actual name of device */
//	<s.9>	Advertising String
//	<i>	String Descriptor describing in advertising packet.
//	<id> fmp_sensor_adv_data_name_data
#define FMP_ADV_DATA_NAME_DATA			("ATMEL-FMP")

/* @brief call back handler type  */
typedef void (*find_me_callback_t)(uint8_t);

/****************************************************************************************
*							        Function
*									Prototypes	                                        *
****************************************************************************************/

/** @brief Initialize the profile, includes all initializations and start up
 * routines of the profiles
 *
 *
 * @param[in] void
 *
 * @pre Must be called after @ref ble_init
 *
 * @return void
 */
void fmp_target_init(void *param);

/** @brief Initialize the services of the profile
 *
 * @param[in] void
 *
 * @pre Must be called before @ref fmp_target_service_define
 *
 * @return void
 */
void fmp_target_service_init(void);

/** @brief profile services definition to the attribute data base
 *
 * @param[in] void
 *
 * @pre Must be called after ref fmp_target_service_init
 *
 * @return @ref AT_BLE_SUCCESS operation completed successfully
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t fmp_target_service_define(void);

/** @brief sets the advertisement data and starts advertisement
 *
 * @param[in] void
 *
 * @pre Must be called after ref fmp_target_service_init
 *
 * @return void
 */
void fmp_target_adv(void);

/** @brief connection state handler
 * called when device goes on connected state
 * @param[in] at_ble_connected_t which includes the connection parameters
 *
 *
 * @return @ref AT_BLE_SUCCESS operation completed successfully
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t fmp_target_connected_state_handler(void *params);

/** @brief disconnection event handler
 *
 * @param[in] at_ble_disconencted_t which includes the connection parameters
 *like handle,reason for disconnection
 *
 * @return @ref AT_BLE_SUCCESS operation completed successfully
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t fmp_target_disconnect_event_handler(void *params);

/** @brief character changed handler
 *
 * @param[in] at_ble_characteristic_changed_t which includes handle,new value
 *
 * @pre Must be called when character change event occurred
 *
 * @return @ref AT_BLE_SUCCESS operation completed successfully
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t fmp_target_char_changed_handler(void *params);

/**
 * @brief function which register the application alerting function for
 *Immediate Alert to the profile
 *
 * @param[in] call back function of the application for immediate alert value
 *
 */
void register_find_me_handler(find_me_callback_t immediate_alert_fn);

#endif /* __FIND_ME_TARGET_H__ */
// </h>

// <<< end of configuration section >>>
