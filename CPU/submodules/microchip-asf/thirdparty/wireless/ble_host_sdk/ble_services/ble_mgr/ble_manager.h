/**
 * \file ble_manager.h
 *
 * \brief BLE manager
 *
 * Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
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
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef __BLE_MANAGER_H__
#define __BLE_MANAGER_H__

#define MAX_EVENT_SIZE			10
#define MAX_EVENT_PAYLOAD_SIZE	1024

#define MAX_GAP_EVENT_SUBSCRIBERS			3
#define MAX_GATT_SERVER_EVENT_SUBSCRIBERS	3
#define MAX_GATT_CLIENT_EVENT_SUBSCRIBERS	3
#define MAX_COMMON_EVENT_SUBSCRIBERS		3
#define MAX_PAIRING_SUBSCRIBERS				3
#define MAX_GATT_TRANSPARENT_SUBSCRIBERS	3

#define DEFAULT_SCAN_INTERVAL				0x0010
#define DEFAULT_SCAN_WINDOW					0x0010

#define MAX_ADV_PAYLOAD_LEN					31
#define MAX_ADV_REPORT_EVENT_LIST			20

typedef enum
{
	BLE_MGR_STATE_INIT,
	BLE_MGR_STATE_ADVERTISING,
	BLE_MGR_STATE_SCANNING,
	BLE_MGRSTATE_CONNECTING,
	BLE_MGR_STATE_CONNECTED,
	BLE_MGR_STATE_DISCOVERING_SERVICE,
}ble_mgr_state_t;

typedef enum
{
	BLE_GATT_NONE,
	BLE_GATT_NOTIFY,
	BLE_GATT_INDICATE,
}ble_mgr_handle_value_operation_t;

/** \enum ble_mgr_event_t 
 *  \brief BLE Manager event type. This event types are used to
 *    subscribe the Group of BLE Event callbacks or 
 *    un-subscribe the group of BLE Event callbacks
*/
typedef enum {
	/* BLE GAP Events types */
	BLE_GAP_EVENT_TYPE,
	/* BLE GATT Server Event types */
	BLE_GATT_SERVER_EVENT_TYPE,
	/* BLE GATT Client Event types */
	BLE_GATT_CLIENT_EVENT_TYPE,
	/* Common Event types */
	BLE_COMMON_EVENT_TYPE,
	/* BLE Pairing Event types */
	BLE_PAIRING_EVENT_TYPE,
	/* BLE GATT Transparent Event types */
	BLE_GATT_TP_EVENT_TYPE,
}ble_mgr_event_t;

/**   \enum ble_mgr_match_param_t 
	\brief BLE Manager advertisement report matching parameter to initiate connection. 
		Upon matching parameter, the connection will be initiated to remote device.
*/
typedef enum
{
	/* None */
	BLE_MATCHING_PARAM_NONE,
	/* Address */
	BLE_MATCHING_PARAM_ADDRESS,
	/* Rssi */
	BLE_MATCHING_PARAM_RSSI,
	/* payload */
	BLE_MATCHING_PARAM_PAYLOAD,
}ble_mgr_match_param_t;

/**	\enum ble_mgr_match_param_t 
	\brief BLE advertisement data types. */
typedef enum Advertisement_Data_Type
{
	ADV_FLAGS = 0x01,
	ADV_INCOMPLETE_128B_SERVICE_UUIDS = 0x06,
	ADV_COMPLETE_128B_SERVICE_UUIDS,
	ADV_SHORT_NAME,
	ADV_COMPLETE_NAME,
	ADV_TX_POWER_LEVEL,
	ADV_CLASS_OF_DEVICE = 0x0D,
	ADV_SERVICE_DATA = 0x16,
	ADV_ADVERTISING_INTERVAL = 0x1A,
	ADV_MANUFATURER_SPECIFIC_DATA = 0xFF,
}ble_mgr_adv_data_type;

/**	\typedef ble_event_callback_t 
 *	\brief BLE Event callback generic type */
typedef ble_status_t (*ble_event_callback_t) (event_msg_t* message);

/**	\struct ble_gap_event_cb_t 
 *	\brief BLE GAP Event callback types
*/
typedef struct _ble_gap_event_cb_t {
	ble_event_callback_t adv_report;
	ble_event_callback_t connected;
	ble_event_callback_t disconnected;
	ble_event_callback_t conn_param_update;
}ble_gap_event_cb_t;

/**	\struct ble_gatt_server_event_cb_t 
 *	\brief All BLE GATT Server callback types
*/
typedef struct _ble_gatt_server_event_cb_t {
	ble_event_callback_t char_value_write;
    ble_event_callback_t prepare_write_request;
    ble_event_callback_t execute_write_request;
}ble_gatt_server_event_cb_t;

/**	\struct ble_gatt_client_event_cb_t 
 *	\brief All BLE GATT Client callback types
*/
typedef struct _ble_gatt_client_event_cb_t {
	ble_event_callback_t service_disc_resp;
	ble_event_callback_t char_disc_resp;
	ble_event_callback_t char_descriptor_disc_resp;
	ble_event_callback_t char_value_received;
    ble_event_callback_t prepare_write_response;
    ble_event_callback_t execute_write_response;
}ble_gatt_client_event_cb_t;

/**	\struct ble_common_event_cb_t 
 *	\brief All BLE Common callback types
*/
typedef struct _ble_common_event_cb_t {
	ble_event_callback_t cmd_complete;
	ble_event_callback_t status_report;
	ble_event_callback_t le_end_test_result;
	ble_event_callback_t config_mode_status;
}ble_common_event_cb_t;

/**	\struct ble_pairing_event_cb_t 
 *	\brief All BLE Pairing callback types
*/
typedef struct _ble_pairing_event_cb_t {
	ble_event_callback_t passkey_entry_req;
	ble_event_callback_t pairing_complete;
	ble_event_callback_t passkey_confirm_req;
}ble_pairing_event_cb_t;

/**	\struct ble_gatt_transparent_event_cb_t 
 *	\brief All BLE GATT Transparent callback types
*/
typedef struct _ble_gatt_transparent_event_cb_t {
	ble_event_callback_t trans_data_received;
}ble_gatt_transparent_event_cb_t;

/**	\struct ble_mgr_adv_report_event_t 
 *	\brief Advertisement report event data
*/
typedef struct
{
	ble_adv_event_type_t adv_event_type;
	ble_addr_t addr;
	uint8_t data_len;
	uint8_t data[MAX_ADV_PAYLOAD_LEN];
	int8_t rssi;
}ble_mgr_adv_report_event_t;

/**	\struct ble_mgr_adv_report_event_t 
 *	\brief Advertisement report event data
*/
typedef struct
{
	ble_mgr_match_param_t matching_param;
	ble_adv_event_type_t adv_event_type;
	ble_addr_t addr;
	uint8_t data_start;
	uint8_t data_len;
	uint8_t data[MAX_ADV_PAYLOAD_LEN];
	int8_t rssi;
}ble_mgr_adv_report_match_param_t;

/*! \fn ble_status_t ble_mgr_device_init(void)
 *  \brief Initialize platform, callback events and BLE interface.
 *  \param None.
 *  \pre None.
 *  \return ble_status_t Status of the BLE interface initialization.
 */
ble_status_t ble_mgr_device_init(void);

/*! \fn ble_status_t ble_mgr_adv_start(const uint8_t *adv_data, uint8_t adv_data_length, const uint8_t * scan_resp_data, uint8_t scan_reap_data_len)
 *  \brief Set advertisement data, scan response data, advertisement parameters and start advertisement.
 *  \param adv_data Advertisement data.
 *  \param adv_data_length Advertisement data length.
 *  \param scan_resp_data Scan response data.
 *  \param scan_reap_data_len Scan response data length.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of setting advertisement data, scan response data, advertisement parameters or start advertisement operation.
 */
ble_status_t ble_mgr_adv_start(const uint8_t *adv_data, uint8_t adv_data_length, const uint8_t * scan_data, uint8_t scan_data_len);

/*! \fn ble_status_t ble_mgr_adv_stop(void)
 *  \brief Stops ongoing advertisement.
 *  \param None.
 *  \pre Advertisement has to be started already.
 *  \return ble_status_t Status of stop advertisement operation.
 */
ble_status_t ble_mgr_adv_stop(void);

/*! \fn ble_status_t ble_mgr_scan_start(void)
 *  \brief Sets BLE scan parameters and starts scanning.
 *  \param None.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of the setting scan parameter or scan start operation.
 */
ble_status_t ble_mgr_scan_start(void);

/*! \fn ble_status_t ble_mgr_scan_stop(void)
 *  \brief Stops the ongoing BLE scan.
 *  \param None.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of the scan stop operation.
 */
ble_status_t ble_mgr_scan_stop(void);

/*! \fn event_status_t ble_mgr_get_event(event_t *event)
 *  \brief Get BLE events and process them.
 *  \param event Carries event ID, parameters and length.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return event_status_t Status of event get operation.
 */
event_status_t ble_mgr_get_event(event_t *event);

/*! \fn ble_status_t ble_mgr_transparent_uart_enable(ble_handle_t conn_handle)
 *  \brief Enables transparentUART with remote device.
 *  \param conn_handle Connection handle of remote device.
 *  \pre Connection is established with remote device.
 *  \return ble_status_t Status of transparentUART enable operation.
 */
ble_status_t ble_mgr_transparent_uart_enable(ble_handle_t conn_handle);

/*! \fn ble_status_t ble_mgr_transparent_uart_disable(ble_handle_t conn_handle)
 *  \brief Disables transparentUART with remote device.
 *  \param conn_handle Connection handle of remote device.
 *  \pre Connection is established with remote device and transparentUART is enabled.
 *  \return ble_status_t Status of transparentUART enable operation.
 */
ble_status_t ble_mgr_transparent_uart_disable(ble_handle_t conn_handle);

/*! \fn ble_status_t ble_mgr_transparent_uart_send(ble_handle_t conn_handle, const uint8_t *data, uint8_t datalen)
 *  \brief Sends transparentUART data to remote device.
 *  \param conn_handle Connection handle of remote device.
 *  \param data Data that needs to be sent to remote device.
 *  \param datalen Length of data that needs to be sent to remote device.
 *  \pre TransparentUART is enabled with remote device using ble_mgr_transparent_uart_enable.
 *  \return ble_status_t Status of transparentUART send data operation.
 */
ble_status_t ble_mgr_transparent_uart_send(ble_handle_t conn_handle, const uint8_t *data, uint8_t datalen);

/*! \fn bool ble_mgr_events_register_callback(ble_mgr_event_t event_type, const void *ble_event_handler)
 *  \brief Register callback functions for BLE events.
 *  \param event_type Type of event, like GAP, GATT-Client, GATT-Server... etc.
 *  \param ble_event_handler Function pointer to group of event handler callbacks.
 *  \param scan_resp_data Scan response data.
 *  \param scan_reap_data_len Scan response data length.
 *  \pre Platform and BLE interface has to be initialized using ble_mgr_device_init.
 *  \return ble_status_t Status of setting advertisement data, scan response data, advertisement parameters or start advertisement operation.
 */
bool ble_mgr_events_register_callback(ble_mgr_event_t event_type, const void *ble_event_handler);

/*! \fn void ble_mgr_peripheral_device_match_params(ble_mgr_adv_report_match_param_t *match_param)
 *  \brief Sets a matching parameter to find and connect with the device.
 *  \param match_param Matching parameter, it could be BLE address, RSSI threshold or advertisement payload.
 *  \pre None.
 *  \return None.
 */
void ble_mgr_peripheral_device_match_params(ble_mgr_adv_report_match_param_t *match_param);

/*! \fn bool ble_mgr_check_match_param(ble_mgr_adv_report_event_t *adv_report)
 *  \brief Check against the matching parameter set by the user, it could be BLE address, RSSI threshold or advertisement payload.
 *  \param adv_report Advertisement report info got from scanning operation.
 *  \pre Peripheral matching parameter has to be set using ble_mgr_peripheral_device_match_params.
 *  \return bool Status of checking against matching parameter.
 */
bool ble_mgr_check_match_param(ble_mgr_adv_report_event_t *adv_report);

/*! \fn ble_status_t ble_mgr_start_connection(ble_addr_t* address)
 *  \brief Initiate a connection based on remote device address.
 *  \param address Address of remote device.
 *  \pre None.
 *  \return bool Status of connection request.
 */
ble_status_t ble_mgr_start_connection(ble_addr_t* address);

/*! \fn ble_status_t ble_mgr_characteristic_notify_set(ble_handle_t conn_handle, uint16_t desc_handle, bool enabled)
 *  \brief Enable/disable characteristic notification.
 *  \param conn_handle Connection handle.
 *  \param desc_handle Client characteristic config descriptor handle.
 *  \param enabled Enable/disable the notification.
 *  \pre None.
 *  \return bool Status of enable/disable notification.
 */
ble_status_t ble_mgr_characteristic_notify_set(ble_handle_t conn_handle, uint16_t desc_handle, bool enabled);

/*! \fn ble_status_t ble_mgr_characteristic_indicate_set(ble_handle_t conn_handle, uint16_t desc_handle, bool enabled)
 *  \brief Enable/disable characteristic indication.
 *  \param conn_handle Connection handle.
 *  \param desc_handle Client characteristic config descriptor handle.
 *  \param enabled Enable/disable the indication.
 *  \pre None.
 *  \return bool Status of enable/disable indication.
 */
ble_status_t ble_mgr_characteristic_indicate_set(ble_handle_t conn_handle, uint16_t desc_handle, bool enabled);

#endif	/* __BLE_MANAGER_H__ */
