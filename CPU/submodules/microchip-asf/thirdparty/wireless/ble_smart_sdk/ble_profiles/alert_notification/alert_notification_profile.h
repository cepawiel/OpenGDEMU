/**
* \file
*
* \brief Alert Notification Profile declarations
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
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

/**
* 
* \section preface Preface
* This is the reference manual for the Time Information Profile
*/

// <<< Use Configuration Wizard in Context Menu >>>
// <h> Alert Notification Configuration
// =======================

#ifndef __ALERT_NOTIFICATION_PROFILE_H__
#define __ALERT_NOTIFICATION_PROFILE_H__
/***********************************************************************************
 *									Macros			                               *
 **********************************************************************************/
/**@brief Advertisement Interval*/
//	<o> Fast Advertisement Interval <100-1000:50>
//	<i> Defines inteval of Fast advertisement in ms.
//	<i> Default: 100
//	<id> anp_app_anp_fast_adv
#define APP_ANP_FAST_ADV							(1600) //1000 ms

/** @brief APP_ANP_ADV_TIMEOUT Advertising time-out between 0x0001 and 0x028F in 
 *seconds, 0x0000 disables time-out.*/
//	<o> Advertisement Timeout <1-655>
//	<i> Defines interval at which advertisement timeout in sec.
//	<i> Default: 655
//	<id> anp_app_anp_adv_timeout
#define APP_ANP_ADV_TIMEOUT							(655) // 10 min

/**@brief Scan Response length*/
//	<o> Scan Response Buffer <1-20>
//	<i> Defines size of buffer for scan response.
//	<i> Default: 10
//	<id> anp_scan_resp_len
#define SCAN_RESP_LEN								(10)

#define ADV_TYPE_LEN								(0x01)

/**@brief Appearance type & Length */
#define ANP_ADV_DATA_APPEARANCE_LEN					(2)
#define ANP_ADV_DATA_APPEARANCE_TYPE				(0x19)
#define ANP_ADV_DATA_APPEARANCE_DATA				("\x00\x40")

/**@brief Advertisement Name Type Length & data */
#define ANP_ADV_DATA_NAME_LEN						(9)
#define ANP_ADV_DATA_NAME_TYPE						(0x09)
//	<s.9>	Advertising String
//	<i>	String Descriptor describing in advertising packet.
//	<id> anp_adv_data_name_data
#define ANP_ADV_DATA_NAME_DATA						("ATMEL-ANS")

/**@brief ANCS Service Solicitation Info*/
#define ANP_ADV_DATA_UUID_LEN						(2)

#define ANP_ADV_DATA_UUID_TYPE						(0x03)


#define SUPPORT_NEW_ALERT_CHAR_UUID					(0x2A47)
#define NEW_ALERT_CHAR_UUID							(0x2A46)
#define SUPPORT_UNREAD_ALERT_CHAR_UUID				(0x2A48)
#define UNREAD_ALERT_STATUS_CHAR_UUID				(0x2A45)
#define ALERT_NOTI_CONTROL_CHAR_UUID				(0x2A44)
#define CLIENT_CHAR_CONF_DESC_UUID					(0x2902)

#define AT_BLE_DISCOVERY_SUCCESS					(10)
//#define AT_BLE_INSUFF_RESOURCE						AT_BLE_ATT_INSUFF_RESOURCE

/***********************************************************************************
 *									types			                               *
 **********************************************************************************/
/* @brief connected callback */
typedef void (*connected_callback_t) (bool);

/***********************************************************************************
 *									Functions		                               *
 **********************************************************************************/
/**
 * @brief Initialize the profile specific information
 * 
 */
void anp_info_init(void);

/**
 * @brief Function used for profile initialization and also enable the advertisement
 * \note Called by the ble_manager
 * 
 */
void anp_client_init( void *params);

/**
 * @brief does all the profile initialization and starts the advertisement
 * \note Called by the ble_manager
 */
void anp_client_adv(void);

/**
 * @brief Connection handler callback
 * @param[in] at_ble_connected_t which consists of connection handle
 * @return at_ble_status_t which return AT_BLE_SUCCESS on success
 */
at_ble_status_t anp_info_connect_handler(void *params);

/**
 * @brief Handler for disconnection event
 * @param[in] disconnected event parameter containing details like handle
 * \note Called by the ble_manager after receiving disconnection event
 */
at_ble_status_t anp_client_disconnected_event_handler(void *params);

/**
 * @brief Handler for service found event
 * @param[in] service found event parameter containing details like service handle,uuid
 * \note Called by the ble_manager after receiving service found event
 */
at_ble_status_t anp_client_service_found_handler(void *params);

/**
 * @brief Discovering the services of Alert Notification
 * @return at_ble_status_t which return AT_BLE_SUCCESS on success
 */
at_ble_status_t alert_service_discovery(void);

/**
 * @brief Handler for discovery complete event
 * @param[in] discovery complete event which contains result of discovery event
 * \note Called by the ble_manager after receiving discovery complete event
 */
at_ble_status_t anp_client_discovery_complete_handler(void *params);

/**
 * @brief Handler for characteristic found event
 * @param[in] characteristic found event parameter containing details like characteristic handle,uuid
 * \note Called by the ble_manager after receiving characteristic found event
 */
at_ble_status_t anp_client_characteristic_found_handler(void *params);

/**
 * @brief Handler for descriptor found event
 * @param[in] descriptor found event parameter containing details like descriptor handle,uuid
 * \note Called by the ble_manager after receiving descriptor found event
 */
at_ble_status_t anp_client_descriptor_found_handler(void *params);

/**
 * @brief Handler for char changed handler 
 * @param[in] characteristic changed event parameter containing details like characteristic handle,value
 * \note Called by the ble_manager after receiving characteristic change event
 */
void anp_client_char_changed_handler(at_ble_characteristic_changed_t *params);

/**
 * @brief Handler for write response 
 * @param[in] write response parameter contacting the result of write request
 * \note Called by the ble_manager after receiving write response event
 */
at_ble_status_t anp_client_write_response_handler(void *params);

/**
 * @brief Handler for notification event 
 * @param[in] notification received parameter containing the notification value
 * \note Called by the ble_manager after receiving the notification
 */
at_ble_status_t anp_client_notification_handler(void *params);

/**
 * @brief Handler for enabling the notification 
 * \note Called by the ble_manager for enabling the notification in the gatt server
 */
void anp_client_write_notification_handler(void );

/**
 * @brief Handler for read response handle 
 * \note Called by the ble_manager for read response in the gatt server
 */
at_ble_status_t anp_client_read_response_handler(void *params);

/**
 * @brief Discovering the services of Alert Notification
 * @param[in] at_ble_connected_t which consists of connection handle
 * @return at_ble_status_t which return AT_BLE_SUCCESS on success
 */
at_ble_status_t anp_info_service_discover(void *params);

/**
 * @brief register the call back for application state
 * @param[in]
 * @return none
 */
void register_connected_callback(connected_callback_t app_connected_cb);

/**
 * @brief invoked by app for disabling the notifications in gatt server
 */
void anp_client_disable_notification(void);

/**
 * @brief invoked by ble manager for setting the notification 
 */
at_ble_status_t anp_client_security_done_handler(void *param);

/**
 * @brief char changed handler invoked by application
 * @param[in] uint8_t array consists of value to be written to alert notification control point
 */
at_ble_status_t anp_write_to_ncp(uint8_t *value);

#endif /* __ALERT_NOTIFICATION_PROFILE_H__ */
// </h>

// <<< end of configuration section >>>
