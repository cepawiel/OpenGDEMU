/**
 * \file
 *
 * \brief Proximity Monitor Profile
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
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

/**
* \mainpage
* \section preface Preface
* This is the reference manual for the Proximity Monitor Profile
*/
/*- Includes ---------------------------------------------------------------*/
#include <asf.h>
#include "platform.h"
#include "pxp_monitor.h"
#include "ble_manager.h"
#include "link_loss.h"

#if defined TX_POWER_SERVICE
#include "tx_power.h"
#endif

#if defined LINK_LOSS_SERVICE
#include "link_loss.h"
#endif

#if defined IMMEDIATE_ALERT_SERVICE
#include "immediate_alert.h"
#endif

/* pxp reporter device address to connect */
at_ble_addr_t pxp_reporter_address;

#if ((BLE_DEVICE_ROLE == BLE_CENTRAL) || (BLE_DEVICE_ROLE == BLE_CENTRAL_AND_PERIPHERAL) || (BLE_DEVICE_ROLE == BLE_OBSERVER))
uint8_t pxp_supp_scan_index[MAX_SCAN_DEVICE];
#endif
uint8_t scan_index = 0;


extern volatile uint8_t scan_response_count;
#if ((BLE_DEVICE_ROLE == BLE_CENTRAL) || (BLE_DEVICE_ROLE == BLE_CENTRAL_AND_PERIPHERAL) || (BLE_DEVICE_ROLE == BLE_OBSERVER))
extern at_ble_scan_info_t scan_info[MAX_SCAN_DEVICE];
#endif
extern volatile ble_device_ll_state_t ble_device_current_state;

volatile uint8_t pxp_connect_request_flag = PXP_DEV_UNCONNECTED;

#if defined TX_POWER_SERVICE
gatt_txps_char_handler_t txps_handle =
{0xff, 0, 0, 0, AT_BLE_INVALID_PARAM, NULL};
uint8_t tx_power_char_data[MAX_TX_POWER_CHAR_SIZE];
#endif

#if defined LINK_LOSS_SERVICE
gatt_lls_char_handler_t lls_handle =
{0xff,0, 0, 0, AT_BLE_INVALID_PARAM, NULL};
uint8_t lls_char_data[MAX_LLS_CHAR_SIZE];
#endif

#if defined IMMEDIATE_ALERT_SERVICE
gatt_ias_char_handler_t ias_handle =
{0xff,0, 0, 0, AT_BLE_INVALID_PARAM, AT_BLE_INVALID_PARAM, NULL};
uint8_t ias_char_data[MAX_IAS_CHAR_SIZE];
#endif

/* *@brief Initializes Proximity profile
* handler Pointer reference to respective variables
*
*/
void pxp_monitor_init(void *param)
{
	UNUSED(param);
	#if defined LINK_LOSS_SERVICE
	lls_handle.char_data = lls_char_data;
	#endif
	#if defined IMMEDIATE_ALERT_SERVICE
	ias_handle.char_data = ias_char_data;
	#endif
	#if defined TX_POWER_SERVICE
	txps_handle.char_data = tx_power_char_data;	
	#endif
	DBG_LOG("High Alert RSSI range: %ddBm and above", (PXP_HIGH_ALERT_RANGE-1));
	DBG_LOG("Mild Alert RSSI range: %ddBm to %ddBm", PXP_LOW_ALERT_RANGE, PXP_HIGH_ALERT_RANGE);
	DBG_LOG("No Alert RSSI range:   %ddBm and below", (PXP_LOW_ALERT_RANGE+1));
	
}

/**@brief Connect to a peer device
*
* Connecting to a peer device, implicitly starting the necessary scan operation
* then connecting if a device in the peers list is found.
*
* @param[in] scan_buffer a list of peers that the device will connect to one of
* them
* @param[in] index index of elements in peers, to initiate the connection
*
* @return @ref AT_BLE_SUCCESS operation programmed successfully
* @return @ref AT_BLE_INVALID_PARAM incorrect parameter.
* @return @ref AT_BLE_FAILURE Generic error.
*/
at_ble_status_t pxp_monitor_connect_request(at_ble_scan_info_t *scan_buffer,
uint8_t index)
{
	memcpy((uint8_t *)&pxp_reporter_address,
	(uint8_t *)&scan_buffer[index].dev_addr,
	sizeof(at_ble_addr_t));

#if ((BLE_DEVICE_ROLE == BLE_CENTRAL) || (BLE_DEVICE_ROLE == BLE_CENTRAL_AND_PERIPHERAL) || (BLE_DEVICE_ROLE == BLE_OBSERVER))
	if (gap_dev_connect(&pxp_reporter_address) == AT_BLE_SUCCESS) {
		DBG_LOG("PXP Connect request sent");
		pxp_connect_request_flag = PXP_DEV_CONNECTING;
		
		return AT_BLE_SUCCESS;
		} else {
		DBG_LOG("PXP Connect request send failed");
	}
#endif
	return AT_BLE_FAILURE;
}

/**@brief Search for a given AD type in a buffer, received from advertising
* packets starts search from the buffer, need to provide required
* search params
*
* @param[in] scan_buffer where all received advertising packet are stored
* @param[in] scanned_dev_count elements in scan_buffer
*
* @return @ref AT_BLE_SUCCESS operation programmed successfully
* @return @ref AT_BLE_INVALID_PARAM incorrect parameter.
* @return @ref AT_BLE_FAILURE Generic error.
*/
at_ble_status_t pxp_monitor_scan_data_handler(at_ble_scan_info_t *scan_info, uint8_t scan_count)
{
#if ((BLE_DEVICE_ROLE == BLE_CENTRAL) || (BLE_DEVICE_ROLE == BLE_CENTRAL_AND_PERIPHERAL) || (BLE_DEVICE_ROLE == BLE_OBSERVER))
	uint8_t scan_device[MAX_SCAN_DEVICE];
	uint8_t pxp_scan_device_count = 0;
	uint8_t scanned_dev_count = scan_count;
	scan_index = 0;
	uint8_t index;
	at_ble_scan_info_t *scan_buffer = scan_info;
	memset(scan_device, 0, MAX_SCAN_DEVICE);
	if (scanned_dev_count) {
		
		at_ble_uuid_t service_uuid;

		for (index = 0; index < scanned_dev_count; index++) {			
			/* Display only the connectible devices*/
			if((scan_buffer[index].type == AT_BLE_ADV_TYPE_DIRECTED) 
				|| (scan_buffer[index].type == AT_BLE_ADV_TYPE_UNDIRECTED)) {					
				scan_device[pxp_scan_device_count++] = index;
			}
		}
		
		if (pxp_scan_device_count) {		
			/* Service type to be searched */
			service_uuid.type = AT_BLE_UUID_16;

			/* Service UUID */
			service_uuid.uuid[1] = (LINK_LOSS_SERVICE_UUID >> 8);
			service_uuid.uuid[0] = (uint8_t)LINK_LOSS_SERVICE_UUID;
			
			for (index = 0; index < pxp_scan_device_count; index++) {
				DBG_LOG("Info: Device found address [%d]  0x%02X%02X%02X%02X%02X%02X ",
				index,
				scan_buffer[scan_device[index]].dev_addr.addr[5],
				scan_buffer[scan_device[index]].dev_addr.addr[4],
				scan_buffer[scan_device[index]].dev_addr.addr[3],
				scan_buffer[scan_device[index]].dev_addr.addr[2],
				scan_buffer[scan_device[index]].dev_addr.addr[1],
				scan_buffer[scan_device[index]].dev_addr.addr[0]);
				
				if (scan_info_parse(&scan_buffer[scan_device[index]], &service_uuid,
				AD_TYPE_COMPLETE_LIST_UUID) ==
				AT_BLE_SUCCESS) {
					/* Device Service UUID  matched */
					pxp_supp_scan_index[scan_index++] = index;
					DBG_LOG_CONT("---PXP");
				}
			}			
		}

		if (!scan_index)  {
			DBG_LOG("Proximity Profile supported device not found ");
		}		
		
		/* Stop the current scan active */
		at_ble_scan_stop();
		
		/*Updating the index pointer to connect */
		if(pxp_scan_device_count) {  
			/* Successful device found event*/
			uint8_t deci_index = pxp_scan_device_count;
			deci_index+=PXP_ASCII_TO_DECIMAL_VALUE;
			do {
				DBG_LOG("Select Index number to Connect or [s] to scan");
				index = getchar();
				DBG_LOG("%c", index);
			} while (!(((index < (deci_index)) && (index >='0')) || (index == 's')));	
			
			if(index == 's') {
				return gap_dev_scan();
			} else {
				index -= PXP_ASCII_TO_DECIMAL_VALUE;
				return pxp_monitor_connect_request(scan_buffer,	scan_device[index]);
			}			
		}			
	} else {  
		/* from no device found event*/
		do
		{
			DBG_LOG("Select [s] to scan again");
			index = getchar();
			DBG_LOG("%c", index);
		} while (!(index == 's')); 
		
		if(index == 's') {
			return gap_dev_scan();
		}
	}		
#endif

	return AT_BLE_FAILURE;
}

at_ble_status_t pxp_monitor_start_scan(void)
{

	at_ble_scan_stop();
	char index_value = 0xFF;

	DBG_LOG("Select [r] to Reconnect or [s] to Scan");
	
	do 
	{
		if(index_value != 0xFF)
		{
			DBG_LOG("Please give input either [r] to reconnect or [s] to scan ");
		}
		
		index_value = getchar();
		ble_event_task();
	} while ((index_value == 0xFF) ||
	          ((index_value != 0xFF) && !((index_value == 'r') || (index_value == 's'))));
		
	if (index_value != 0xFF)
	{
		DBG_LOG("%c", index_value);
	}
	
	if(index_value == 'r') {
#if ((BLE_DEVICE_ROLE == BLE_CENTRAL) || (BLE_DEVICE_ROLE == BLE_CENTRAL_AND_PERIPHERAL) || (BLE_DEVICE_ROLE == BLE_OBSERVER))
		if (gap_dev_connect(&pxp_reporter_address) == AT_BLE_SUCCESS) {
			DBG_LOG("PXP Re-Connect request sent");
			pxp_connect_request_flag = PXP_DEV_CONNECTING;
			return AT_BLE_SUCCESS;
			} else {
			DBG_LOG("PXP Re-Connect request send failed");
		}
#endif
	}
	else if(index_value == 's') {
#if ((BLE_DEVICE_ROLE == BLE_CENTRAL) || (BLE_DEVICE_ROLE == BLE_CENTRAL_AND_PERIPHERAL) || (BLE_DEVICE_ROLE == BLE_OBSERVER))
		return gap_dev_scan();
#endif
	}
	return AT_BLE_FAILURE;
}

/**@brief peer device connection terminated
*
* handler for disconnect notification
* try to send connect request for previously connect device.
*
* @param[in] available disconnect handler of peer and
* reason for disconnection
*
* @return @ref AT_BLE_SUCCESS Reconnect request sent to previously connected
*device
* @return @ref AT_BLE_FAILURE Reconnection fails.
*/
at_ble_status_t pxp_disconnect_event_handler(void *params)
{	
	at_ble_disconnected_t *disconnect;
	disconnect = (at_ble_disconnected_t *)params;
	pxp_connect_request_flag = PXP_DEV_UNCONNECTED;
	
// 	if((ble_device_current_state != CENTRAL_SCANNING_STATE) && (pxp_connect_request_flag == PXP_DEV_UNCONNECTED) && (ble_device_current_state != PERIPHERAL_ADVERTISING_STATE))
// 		{
// 			pxp_monitor_start_scan();
// 		}
		
	return AT_BLE_FAILURE;
}

#if ((BLE_DEVICE_ROLE == BLE_CENTRAL) || (BLE_DEVICE_ROLE == BLE_CENTRAL_AND_PERIPHERAL) || (BLE_DEVICE_ROLE == BLE_OBSERVER))
/**@brief Discover all services
 *
 * @param[in] connection handle.
 * @return @ref AT_BLE_SUCCESS operation programmed successfully.
 * @return @ref AT_BLE_INVALID_PARAM incorrect parameter.
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t pxp_monitor_service_discover(at_ble_handle_t handle)
{
	at_ble_status_t status;
	status = at_ble_primary_service_discover_all(
					handle,
					GATT_DISCOVERY_STARTING_HANDLE,
					GATT_DISCOVERY_ENDING_HANDLE);
	if (status == AT_BLE_SUCCESS) {
		DBG_LOG_DEV("GATT Discovery request started ");
	} else {
		DBG_LOG("GATT Discovery request failed");
	}
	
	return status;
}
#endif //((BLE_DEVICE_ROLE == BLE_CENTRAL) || (BLE_DEVICE_ROLE == BLE_CENTRAL_AND_PERIPHERAL) || (BLE_DEVICE_ROLE == BLE_OBSERVER))

at_ble_status_t pxp_monitor_pair_done_handler(void *params)
{
	at_ble_status_t status = AT_BLE_FAILURE;
	at_ble_pair_done_t *pair_done_val;
	pair_done_val = (at_ble_pair_done_t *)params;

	if (pair_done_val->status == AT_BLE_SUCCESS) {
		DBG_LOG("Pairing completed successfully\n\r");
		status = pxp_monitor_service_discover(pair_done_val->handle);
	} else {
		pxp_monitor_start_scan();
		return AT_BLE_FAILURE;
	}

	return status;
}

at_ble_status_t pxp_monitor_encryption_change_handler(void *params)
{
	at_ble_status_t status = AT_BLE_FAILURE;
	at_ble_encryption_status_changed_t *encryption_status;
	encryption_status = (at_ble_encryption_status_changed_t *)params;

	if (encryption_status->status == AT_BLE_SUCCESS) {
	#if BLE_PAIR_ENABLE == true
		pxp_connect_request_flag = PXP_DEV_CONNECTED;
	#endif
		status = pxp_monitor_service_discover(encryption_status->handle);
	} else {
		pxp_connect_request_flag = PXP_DEV_UNCONNECTED;
	}
	
	return status;
}

/**@brief Connected event state handle after connection request to peer device
*
* After connecting to the peer device start the GATT primary discovery
*
* @param[in] conn_params parameters of the established connection
*
* @return @ref AT_BLE_SUCCESS operation successfully.
* @return @ref AT_BLE_INVALID_PARAM if GATT discovery parameter are incorrect
*parameter.
* @return @ref AT_BLE_FAILURE Generic error.
*/
at_ble_status_t pxp_monitor_connected_state_handler(void *params)
{
	at_ble_connected_t *conn_params;
	conn_params = (at_ble_connected_t *)params;	

#if BLE_PAIR_ENABLE == false
		pxp_connect_request_flag = PXP_DEV_CONNECTED;
#endif

	return conn_params->conn_status;
}

/**@brief Discover the Proximity services
*
* Search will go from start_handle to end_handle, whenever a service is found
*and
* compare with proximity services and stores the respective handlers
* @ref PXP_MONITOR_CONNECTED_STATE_HANDLER event i.
*
* @param[in] at_ble_primary_service_found_t  Primary service parameter
*
*/
at_ble_status_t pxp_monitor_service_found_handler(void *params)
{
	at_ble_uuid_t *pxp_service_uuid;
	at_ble_status_t status = AT_BLE_SUCCESS;
	at_ble_primary_service_found_t *primary_service_params;
	primary_service_params = (at_ble_primary_service_found_t *)params;
	
	pxp_service_uuid = &primary_service_params->service_uuid;
	if (pxp_service_uuid->type == AT_BLE_UUID_16) {
		uint16_t service_uuid;
		service_uuid
		= ((pxp_service_uuid->uuid[1] <<
		8) | pxp_service_uuid->uuid[0]);
		switch (service_uuid) {
			/* for link loss service Handler */
			case LINK_LOSS_SERVICE_UUID:
			{
				#if defined LINK_LOSS_SERVICE
				lls_handle.conn_handle
				= primary_service_params->conn_handle;
				lls_handle.start_handle
				= primary_service_params->start_handle;
				lls_handle.end_handle
				= primary_service_params->end_handle;
				DBG_LOG("link loss service discovered");
				DBG_LOG_PTS("start_handle: %04X end_handle: %04X",
				primary_service_params->start_handle,
				primary_service_params->end_handle);				
				lls_handle.char_discovery=(at_ble_status_t)DISCOVER_SUCCESS;
				#endif
			}
			break;

			/* for Immediate Alert service Handler */
			case IMMEDIATE_ALERT_SERVICE_UUID:
			{
				#if defined IMMEDIATE_ALERT_SERVICE
				ias_handle.conn_handle
				= primary_service_params->conn_handle;
				ias_handle.start_handle
				= primary_service_params->start_handle;
				ias_handle.end_handle
				= primary_service_params->end_handle;
				DBG_LOG("Immediate Alert service discovered");
				DBG_LOG_PTS("start_handle: %04X end_handle: %04X ",
				primary_service_params->start_handle,
				primary_service_params->end_handle);				
				ias_handle.char_discovery=(at_ble_status_t)DISCOVER_SUCCESS;
				#endif
			}
			break;

			/* for Tx Power service Handler */
			case TX_POWER_SERVICE_UUID:
			{
				#if defined TX_POWER_SERVICE
				txps_handle.conn_handle
				= primary_service_params->conn_handle;
				txps_handle.start_handle
				= primary_service_params->start_handle;
				txps_handle.end_handle
				= primary_service_params->end_handle;
				DBG_LOG("Tx power service discovered");
				DBG_LOG_PTS("start_handle: %04X end_handle: %04X",
				primary_service_params->start_handle,
				primary_service_params->end_handle);
				txps_handle.char_discovery=(at_ble_status_t)DISCOVER_SUCCESS;
				#endif
			}
			break;

			default:
			status = AT_BLE_INVALID_PARAM; 
			break;
		}
	}
	return status;
}

/**@brief Discover all Characteristics supported for Proximity Service of a
* connected device
*  and handles discovery complete
* Search will go from start_handle to end_handle, whenever a characteristic is
*found
* After search and discovery completes will initialize the alert level and read
*the tx power value as defined
* @ref AT_BLE_CHARACTERISTIC_FOUND event is sent and @ref
*AT_BLE_DISCOVERY_COMPLETE is sent at end of discover operation.
*
* @param[in] discover_status discovery status of each handle
*
*/
at_ble_status_t pxp_monitor_discovery_complete_handler(void *params)
{
	bool discover_char_flag = true;
	at_ble_discovery_complete_t *discover_status;
	discover_status = (at_ble_discovery_complete_t *)params;

	
	DBG_LOG_DEV("discover complete operation %d",discover_status->status);
	if ((discover_status->status == DISCOVER_SUCCESS) || (discover_status->status == AT_BLE_SUCCESS)) {
		#if defined TX_POWER_SERVICE
		if ((txps_handle.char_discovery == DISCOVER_SUCCESS) && (discover_char_flag)) {
			if (at_ble_characteristic_discover_all(
			txps_handle.conn_handle,
			txps_handle.start_handle,
			txps_handle.end_handle) ==
			AT_BLE_SUCCESS) {
				DBG_LOG_DEV("Tx Characteristic Discovery Started");
			} else {
				DBG_LOG("Tx Characteristic Discovery Failed");
			}
			txps_handle.char_discovery = AT_BLE_FAILURE;
			discover_char_flag = false;
		} else if (txps_handle.char_discovery == AT_BLE_INVALID_PARAM) {
			DBG_LOG("Tx Power Service not Found");
			txps_handle.char_discovery = AT_BLE_INVALID_STATE;
			discover_char_flag = false;
		}

		#endif

		#if defined LINK_LOSS_SERVICE
		if ((lls_handle.char_discovery == DISCOVER_SUCCESS) &&
		(discover_char_flag)) {
			if (at_ble_characteristic_discover_all(
			lls_handle.conn_handle,
			lls_handle.start_handle,
			lls_handle.end_handle) ==
			AT_BLE_SUCCESS) 
			{
				DBG_LOG_DEV(
				"Link Loss Characteristic Discovery Started");
			} else {
				lls_handle.char_discovery = AT_BLE_FAILURE;
				DBG_LOG(
				"Link Loss Characteristic Discovery Failed");
			}
			lls_handle.char_discovery = AT_BLE_FAILURE;
			discover_char_flag = false;
		} else if(lls_handle.char_discovery==AT_BLE_INVALID_PARAM) {
			DBG_LOG("Link Loss Service not Available");
			lls_handle.char_discovery = AT_BLE_INVALID_STATE;
		}

		#endif

		#if defined IMMEDIATE_ALERT_SERVICE
		if ((ias_handle.char_discovery == DISCOVER_SUCCESS) &&
		(discover_char_flag)) {
			if (at_ble_characteristic_discover_all(
			ias_handle.conn_handle,
			ias_handle.start_handle,
			ias_handle.end_handle) ==
			AT_BLE_SUCCESS) {
				DBG_LOG_DEV(
				"Immediate Characteristic Discovery Started");
				} else {
				ias_handle.char_discovery = AT_BLE_FAILURE;
				DBG_LOG(
				"Immediate Characteristic Discovery Failed");
			}
			ias_handle.char_discovery = AT_BLE_FAILURE;
			discover_char_flag = false;
		} else if(ias_handle.char_discovery==AT_BLE_INVALID_PARAM) {
			DBG_LOG("Immediate Alert Service not Available");
			ias_handle.char_discovery = AT_BLE_INVALID_STATE;
		}

#endif
	
#if defined LINK_LOSS_SERVICE
		if(lls_handle.char_discovery == AT_BLE_INVALID_STATE) {
			DBG_LOG("PROXIMITY PROFILE NOT SUPPORTED");
			discover_char_flag = false;
			at_ble_disconnect(lls_handle.conn_handle, AT_BLE_TERMINATED_BY_USER);
		}
#endif
		
		if (discover_char_flag) {
			DBG_LOG_DEV("GATT characteristic discovery completed");
			#if defined LINK_LOSS_SERVICE
			/* set link loss profile to high alert upon connection */
			if (!(lls_alert_level_write(lls_handle.conn_handle, lls_handle.char_handle,
			LLS_ALERT_LEVEL) == AT_BLE_SUCCESS)) {
				DBG_LOG("Link Loss write characteristics failed");
			}

			#endif

			#if defined TX_POWER_SERVICE
			if (!(txps_power_read(txps_handle.conn_handle,
			txps_handle.char_handle) ==
			AT_BLE_SUCCESS)) {
				DBG_LOG("Characteristic Read Request failed");
			}

			#endif
		}
	}
	return AT_BLE_SUCCESS;
}

/**@brief Handles the read response from the peer/connected device
*
* if any read request send, response back event is handle.
* compare the read response characteristics with available service.
* and data is handle to the respective service.
*/
at_ble_status_t pxp_monitor_characteristic_read_response(void *params)
{
	at_ble_characteristic_read_response_t *char_read_resp;
	char_read_resp = (at_ble_characteristic_read_response_t *)params;
	
	
	#if defined TX_POWER_SERVICE
	txps_power_read_response(char_read_resp, &txps_handle);
	#endif

	#if defined LINK_LOSS_SERVICE
	lls_alert_read_response(char_read_resp, &lls_handle);
	#endif
		
	return AT_BLE_SUCCESS;
}

/**@brief Handles all Discovered characteristics of a given handler in a
* connected device
*
* Compare the characteristics UUID with proximity services whenever a
*characteristics is found
* if compare stores the characteristics handler of respective service
*
* @param[in] characteristic_found Discovered characteristics params of a
*connected device
*
*/
at_ble_status_t pxp_monitor_characteristic_found_handler(void *params)
{
	uint16_t charac_16_uuid;
	at_ble_characteristic_found_t *characteristic_found;
	characteristic_found = (at_ble_characteristic_found_t *)params;
	
	charac_16_uuid = (uint16_t)((characteristic_found->char_uuid.uuid[0]) |	\
	(characteristic_found->char_uuid.uuid[1] << 8));

	if (charac_16_uuid == TX_POWER_LEVEL_CHAR_UUID) {
		#if defined TX_POWER_SERVICE
		txps_handle.char_handle = characteristic_found->value_handle;
		DBG_LOG("Tx power characteristics: Attrib handle %x property %x handle: %x uuid : %x",
					characteristic_found->char_handle, characteristic_found->properties,
					txps_handle.char_handle, charac_16_uuid);
		#endif
	} else if ((charac_16_uuid == ALERT_LEVEL_CHAR_UUID)) {
     #if defined LINK_LOSS_SERVICE
		if ((characteristic_found->char_handle > lls_handle.start_handle) &&
				(characteristic_found->char_handle < lls_handle.end_handle)) {
			lls_handle.char_handle = characteristic_found->value_handle;
			DBG_LOG("link loss characteristics: Attrib handle %x property %x handle: %x uuid : %x",
					characteristic_found->char_handle, characteristic_found->properties,
					lls_handle.char_handle, charac_16_uuid);
			
		} else {
			#if defined IMMEDIATE_ALERT_SERVICE
			ias_handle.char_handle = characteristic_found->value_handle;
			DBG_LOG("Immediate alert characteristics: Attrib handle %x property %x handle: %x uuid : %x",
					characteristic_found->char_handle, characteristic_found->properties,
					ias_handle.char_handle, charac_16_uuid);
			#endif
		}
		#endif
	}

	return AT_BLE_SUCCESS;
}
