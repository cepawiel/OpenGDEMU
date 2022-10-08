/**
 * \file
 *
 * \brief Proximity Reporter Profile
 *
 * Copyright (c) 2017-2018 Microchip Technology Inc. and its subsidiaries.
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

/****************************************************************************************
*							        Includes	                                     	*
****************************************************************************************/
#include <asf.h>
#include <string.h>
#include "at_ble_api.h"
#include "ble_manager.h"
#include "pxp_reporter.h"
#include "immediate_alert.h"
#include "link_loss.h"
#include "tx_power.h"
#include "ble_utils.h"

/****************************************************************************************
*							        Globals	                                     		*
****************************************************************************************/

static const ble_gap_event_cb_t pxp_gap_handle = {
	.connected = pxp_reporter_connected_state_handler,
	.disconnected = pxp_reporter_disconnect_event_handler
};

static const ble_gatt_server_event_cb_t pxp_gatt_server_handle = {
	.characteristic_changed = pxp_reporter_char_changed_handler
};

#ifdef LINK_LOSS_SERVICE
gatt_service_handler_t txps_handle;
#endif // LINK_LOSS_SERVICE

#ifdef TX_POWER_SERVICE
gatt_service_handler_t lls_handle;
#endif // TX_POWER_SERVICE

#ifdef IMMEDIATE_ALERT_SERVICE
gatt_service_handler_t ias_handle;
#endif // IMMEDIATE_ALER_SERVICE




/** @brief Scan response data*/
uint8_t scan_rsp_data[SCAN_RESP_LEN] = {0x09, 0xff, 0x00, 0x06, 0xd6, 0xb2, 0xf0, 0x05, 0xf0, 0xf8};


/** @brief Alert value used for immediate alert service helps in pathloss */
uint8_t pathloss_alert_value = INVALID_IAS_PARAM ;

/** @brief Alert value used for Linkloss service*/
uint8_t linkloss_current_alert_level ;

/** @brief Callback handlers for linkloss and pathloss */
reporter_callback_t pathloss_cb;
reporter_callback_t linkloss_cb;
reporter_state_callback_t connected_cb;

/****************************************************************************************
*							        Implementation	                                     							*
****************************************************************************************/

/**
 * \brief Initializations of profile services based on pathloss option
*/
void pxp_service_init(void)
{
	/** Initializing the mandatory linkloss service of proximity reporter*/
	init_linkloss_service(&lls_handle);
	
	#if defined PATHLOSS	
	/** Initializing the optional services for pathloss feature of proximity reporter*/
	init_immediate_alert_service(&ias_handle);
	init_tx_power_service(&txps_handle);	
	#endif
}

/**
 * \brief registering the path loss handler of the application
*/
void register_pathloss_handler(reporter_callback_t pathloss_fn)
{
	pathloss_cb = pathloss_fn;
}

/**
 * \brief registering the linkloss handler of the application
*/
void register_linkloss_handler(reporter_callback_t linkloss_fn)
{
	linkloss_cb = linkloss_fn;
}

void register_state_handler(reporter_state_callback_t state_fn)
{
	connected_cb = state_fn;
}

/**
* \Definition of profile services to the attribute data base based on pathloss
*/
at_ble_status_t pxp_service_define (void)
{
	if (!(lls_primary_service_define(&lls_handle) == AT_BLE_SUCCESS))
	{
		DBG_LOG("LLS Service Define Failed");
	} else
	{
		DBG_LOG_DEV("lls handle %d",lls_handle.serv_handle);
	}
		
	#if defined PATHLOSS
	if (!(ias_primary_service_define(&ias_handle) == AT_BLE_SUCCESS))
	{
		DBG_LOG("IAS Service Define Failed");
	} else
	{
	DBG_LOG_DEV("ias handle %d",ias_handle.serv_handle);
	}
	
	if (!(txps_primary_service_define(&txps_handle) == AT_BLE_SUCCESS))
	{
		DBG_LOG("TXPS Service Define Failed");
	} else 
	{
		DBG_LOG_DEV("tx handle %d",txps_handle.serv_handle);
	}
	#endif
	
	DBG_LOG("The Supported Services in Proximity Reporter are:");
	DBG_LOG("  -> Link Loss Service");
	
	#if defined PATHLOSS
	DBG_LOG("  -> Immediate Alert Service");
	DBG_LOG("  -> Tx Power Service");
	
	#endif
	
	return AT_BLE_SUCCESS;
}



/**
* \Service Characteristic change handler function 
*/
at_ble_status_t pxp_reporter_char_changed_handler(void *params)
{
	int temp_val;
	at_ble_characteristic_changed_t change_params;
	
	at_ble_characteristic_changed_t *char_handle;
	char_handle = (at_ble_characteristic_changed_t *)params;
	
	memcpy((uint8_t *)&change_params, char_handle, sizeof(at_ble_characteristic_changed_t));

	temp_val = lls_set_alert_value(&change_params,&lls_handle);
	
	if (temp_val != INVALID_LLS_PARAM)
	{
		linkloss_current_alert_level = temp_val;
	}
	
	pathloss_alert_value = ias_set_alert_value(&change_params,&ias_handle);
	
	if (pathloss_alert_value != INVALID_IAS_PARAM)
	{
		pathloss_cb(pathloss_alert_value);
		
	}
	
	return AT_BLE_SUCCESS;
}

/**
* \Pxp reporter connected state handler */
at_ble_status_t pxp_reporter_connected_state_handler(void *params)
{
	at_ble_status_t status;
	uint16_t len = sizeof(uint8_t);
	at_ble_connected_t *conn_params;
	conn_params = (at_ble_connected_t *)params;
	
	connected_cb(true);

	if ((status = at_ble_characteristic_value_get(lls_handle.serv_chars.char_val_handle,&linkloss_current_alert_level,&len)))
	{
		DBG_LOG("Read of alert value for link loss service failed:reason %x",status);
	}
	
	if ((status = at_ble_characteristic_value_get(ias_handle.serv_chars.char_val_handle,&pathloss_alert_value,&len)))
	{
		DBG_LOG("Read of alert value for Immediate alert service failed:reason %x",status);
	}
	ALL_UNUSED(conn_params);
	return AT_BLE_SUCCESS;
}

/**
* \Pxp reporter disconnected state handler */
at_ble_status_t pxp_reporter_disconnect_event_handler(void *params)
{
	at_ble_disconnected_t *disconnect;
	disconnect = (at_ble_disconnected_t *)params;
	
	linkloss_cb(linkloss_current_alert_level);
	
	if(at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED, AT_BLE_ADV_GEN_DISCOVERABLE, NULL, AT_BLE_ADV_FP_ANY,
	APP_PXP_FAST_ADV, APP_PXP_ADV_TIMEOUT, 0) != AT_BLE_SUCCESS)
	{
		#ifdef DBG_LOG
		DBG_LOG("BLE Adv start Failed");
		#endif
	}
	else
	{
		DBG_LOG("Bluetooth Device is in Advertising Mode");
	}
	 ALL_UNUSED(&disconnect);
        return AT_BLE_SUCCESS;
}


/**
* \Pxp reporter advertisement initialization and adv start 
*/
void pxp_reporter_adv(void)
{
	/* Advertisement data set from ble_manager*/
	if(!(ble_advertisement_data_set() == AT_BLE_SUCCESS))
	{
		DBG_LOG("Fail to set Advertisement data");
	}
	
	/* Start of advertisement */
	if(at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED, AT_BLE_ADV_GEN_DISCOVERABLE, NULL, AT_BLE_ADV_FP_ANY, APP_PXP_FAST_ADV, APP_PXP_ADV_TIMEOUT, 0) == AT_BLE_SUCCESS)
	{
		#ifdef DBG_LOG
		DBG_LOG("Bluetooth device is in Advertising Mode");
		#endif
	}	
	else
	{
		#ifdef DBG_LOG
		DBG_LOG("BLE Adv start Failed");
		#endif
	}
}

/**
* \Pxp reporter Initialization which initializes service,defines and start adv
*/
void pxp_reporter_init(void *param)
{
	/* pxp services initialization*/
	pxp_service_init();	

	/* pxp services definition		*/
	pxp_service_define();	
	
	/* pxp services advertisement */
	pxp_reporter_adv();
	
	/* Callback registering for BLE-GAP Role */
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE, &pxp_gap_handle);
	
	/* Callback registering for BLE-GATT-Server Role */
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_SERVER_EVENT_TYPE, &pxp_gatt_server_handle);
	
    ALL_UNUSED(param);
}
