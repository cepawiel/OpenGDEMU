/**
 * \file
 *
 * \brief Heart Rate Sensor Profile declarations
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
 *Support</a>
 */


// <<< Use Configuration Wizard in Context Menu >>>
// <h> Heart Rate Sensor Configuration
// =======================
#ifndef __HR_SENSOR_H__
#define __HR_SENSOR_H__

/****************************************************************************************
*							        Macro                                               *
****************************************************************************************/


/** @brief APP_HR_SENSOR_FAST_ADV between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s). */
//	<o> Fast Advertisement Interval <100-1000:50>
//	<i> Defines inteval of Fast advertisement in ms.
//	<i> Default: 100
//	<id> hr_sensor_fast_adv
#define HR_SENSOR_FAST_ADV								(1600) //1000 ms

/** @brief APP_HR_SENSOR_ADV_TIMEOUT Advertising time-out between 0x0001 and 0x028F in 
 *seconds, 0x0000 disables time-out.*/
//	<o> Advertisement Timeout <1-655>
//	<i> Defines interval at which advertisement timeout in sec.
//	<i> Default: 655
//	<id> hr_sensor_adv_timeout
#define HR_SENSOR_ADV_TIMEOUT							(655) // 10 min

/** @brief scan_resp_len is the length of the scan response data */
//	<o> Scan Response Buffer <1-20>
//	<i> Defines size of buffer for scan response.
//	<i> Default: 10
//	<id> hr_sensor_scan_resp_len
#define SCAN_RESP_LEN									(10)
	
/** @brief ADV_DATA_LEN */
#define ADV_DATA_LEN									(18)

/** @brief ADV_TYPE_LEN */
#define ADV_TYPE_LEN									(0x01)

#define UUID_16_BIT_LEN									(0x02)


/** @brief HR_SENSOR_ADV_DATA_UUID_TYPE is complete 16 bit uuid type*/
#define HR_SENSOR_ADV_DATA_COMP_16_UUID_TYPE			(0x03)

/** @brief HR_SENSOR_ADV_DATA_UUID_LEN the total length for hr uuid and dis uuid */
#define HR_SENSOR_ADV_DATA_UUID_LEN						(4)

/** @brief DEVICE_INFORMATION_SERVICE_UUID **/
#define DEVICE_INFORMATION_SERVICE_UUID					(0x180A)

/** @brief HR_SENSOR_ADV_DATA_NAME_TYPE the gap ad data type */
#define HR_SENSOR_ADV_DATA_NAME_TYPE					(0x09)

/** @brief HR_SENSOR_ADV_DATA_NAME_LEN the  length of the device name */
#define HR_SENSOR_ADV_DATA_NAME_LEN						(9)

/* @brief HR_SENSOR_ADV_DATA_NAME_DATA the actual name of device */
//	<s.9>	Advertising String
//	<i>	String Descriptor describing in advertising packet.
//	<id> hr_sensor_adv_data_name_data
#define HR_SENSOR_ADV_DATA_NAME_DATA					("ATMEL-HRP")


/**
 * @brief APP_CONNECTION_STATE indicates application is in connected state
 */
#define HR_APP_CONNECTION_STATE							(1)

/**
 * @brief APP_DISCONNECT_STATE indicates application is in disconnected state
 */
#define HR_APP_DISCONNECT_STATE							(0)

#define HR_CONTROL_POINT_RESET_VAL						(1)

#define FLAG_ENERGY_EXP									(0x1 << 3)
/* @brief call back handler type  */
typedef void (*hr_notification_callback_t)(uint8_t);

/* @brief call back handler type  */
typedef void (*hr_reset_callback_t)(void);

/****************************************************************************************
*							        Function Prototypes	                                *                                                        *
****************************************************************************************/

/** @brief hr_sensor_init initializes and defines the services of the hr profile
 *
 *  @param[in] params are unused.
 *
 */
void hr_sensor_init(void *param);

/** @brief hr_sensor_service_init initializes the services of the profile
 *
 */
void hr_sensor_service_init(void);

/** @brief hr_sensor_service_define defines the services of the profile
 *
 */
void hr_sensor_service_define(void);

/** @brief hr_sensor_adv adds the advertisement data of the profile and starts
 * advertisement
 *
 */
void hr_sensor_adv(void);

/** @brief hr_sensor_disconnect disconnects with the peer device called by the
 * application
 *
 */
void hr_sensor_disconnect(void);

/** @brief hr_sensor_send_notification adds the new characteristic value and
 * sends the notification
 *  @param[in] hr_data the new hr characteristic value needs to be updated
 *  @param[in] length length of new characteristic value
 *  @return	   returns 1 on success and 0 on failure
 */
bool hr_sensor_send_notification(uint8_t *hr_data, uint8_t length);

/** @brief hr_sensor_notification_cfm_handler called on notification confirmation
 *  event by the ble manager
 *	@param[in] at_ble_status_t AT_BLE_SUCCESS on success AT_BLE_FAILURE on failure
 *  called
 */
at_ble_status_t hr_sensor_notification_cfm_handler(void * params);

/** @brief register_hr_notification_handler registers the notification handler
 *  passed by the application
 *  @param[in] hr_notification_callback_t address of the notification handler
 *  function to be called
 */
at_ble_status_t hr_notification_confirmation_handler(void * params);

/** @brief register_hr_reset_handler registers the reset handler passed by the
 * application
 *  @param[in]	hr_reset_callback_t address of the handler function to be called
 */
void register_hr_reset_handler(hr_reset_callback_t hr_reset_handler);

/** @brief hr_sensor_char_changed_handler called by the ble manager after a
 *  change in the characteristic
 *  @param[in] at_ble_characteristic_changed_t which contains handle of
 *  characteristic and new value
 *  @return AT_BLE_SUCCESS on success and AT_BLE_FAILURE on failure
 */
at_ble_status_t hr_sensor_char_changed_handler(
		void *char_params);

/** @brief hr_sensor_disconnect_event_handler called by ble manager after
 *  disconnection event recieved
 *	@param[in] at_ble_disconnected_t	which has connection handle and
 *  reason for disconnection
 */
at_ble_status_t hr_sensor_disconnect_event_handler(
		void *disconnect);

/** @brief hr_sensor_connected_state_handler called by ble manager after a
 * change in characteristic
 *  @param[in] at_ble_connected_t which has connection handle and the peer
 *device address
 */
at_ble_status_t hr_sensor_connected_state_handler(
							void *params);
							
							/** @brief register_hr_notification_handler registers the notification handler
 *	passed by the application
 *  param[in] hr_notification_callback_t address of the notification handler
 *	function to be called
 */
void register_hr_notification_handler(
		hr_notification_callback_t hr_notificaton_handler);

/** @brief hr_sensor_char_write_request handles the write request for heart rate 
 *  control point characteristic.
 *  @param[in]	at_ble_characteristic_write_request_t parameters containing the value written
 */
at_ble_status_t hr_sensor_char_write_request(void * params);
#endif /*__HR_SENSOR_H__ */
// </h>

// <<< end of configuration section >>>
