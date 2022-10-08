/**
 * \file
 *
 * \brief OTAU Service declarations
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

#ifndef __OTAU_SERVICE_H__
#define __OTAU_SERVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************/
/*								MACROS					   */
/*************************************************************************/

/**
 * @brief Total no of characteristics in OTAU service
 */
#define OTAU_MAX_CHARACTERISTICS				(3)

/* OTAU Service handle will be unknown before adding it to GATT Server database */
#define DEFAULT_OTAU_SERVICE_HANDLE				(0)

/* OTAU characteristics handle will be unknown before adding it to GATT Server database */
#define DEFAULT_OTAU_CHAR_HANDLE				(0)

/* OTAU End Point-1 Characteristics size */
#define OTAU_CHARACTERISTICS_ENDPOINT1_MAX_SIZE	(280)

/* OTAU End Point-2 Characteristics size */
#define OTAU_CHARACTERISTICS_ENDPOINT2_MAX_SIZE	(280)

/* OTAU End Point-3 Characteristics size */
#define OTAU_CHARACTERISTICS_ENDPOINT3_MAX_SIZE	(4)

/* OTAU included service list, no service included. Its desired to set to NULL */
#define OTAU_INCLUDED_SERVICE_LIST				(NULL)

/* OTAU included service count, no service included. Its desired to set to 0 */
#define OTAU_INCLUDED_SERVICE_COUNT				(0)

/* Invalid state or OTAU is not yet ready */
#define OTAU_CHARACTERISTICS_DEFAULT_VALUE		(0xFFFF)

/* OTAU Service UUID */
#define OTAU_SERVICE_UUID										("\x8B\x69\x8D\x5B\x04\xE1\x48\xB1\x96\x17\xAC\x80\xAA\x64\xE0\xD0")

/* OTAU End Point-1 Characteristics UUID */
#define OTAU_CHARACTERISTICS_ENDPOINT1_UUID						("\x8B\x69\x8D\x5B\x04\xE1\x48\xB1\x96\x17\xAC\x80\xAA\x64\xE0\xE0")

/* OTAU End Point-2 Characteristics UUID */
#define OTAU_CHARACTERISTICS_ENDPOINT2_UUID						("\x8B\x69\x8D\x5B\x04\xE1\x48\xB1\x96\x17\xAC\x80\xAA\x64\xE0\xE5")

/* OTAU End Point-3 Characteristics UUID */
#define OTAU_CHARACTERISTICS_ENDPOINT3_UUID						("\x8B\x69\x8D\x5B\x04\xE1\x48\xB1\x96\x17\xAC\x80\xAA\x64\xE0\xEA")

/* OTAU End Point-1 Characteristics index */
#define OTAU_INDICATION_CHAR_IDX						(0)

/* OTAU End Point-2 Characteristics index */
#define OTAU_WRITE_CHAR_IDX								(1)

/* OTAU End Point-3 Characteristics index */
#define OTAU_READ_CHAR_IDX								(2)

/* CCCD Write Unused */
#define CCCD_WRITE_UNUSED						(0xFFFF)

/* CCCD Write Failed */
#define CCCD_WRITE_FAILED						(0xFFFE)

/* CCCD Notification only Enabled */
#define CCCD_NOTIFICATION_ENABLED				(0x0001)

/* CCCD Indication only Enabled */
#define CCCD_INDICATION_ENABLED					(0x0002)

/* CCCD notification and indications are enabled */
#define CCCD_NOTIFICATION_INDICATION_ENABLED	(0x0003)

/* CCCD notification and indications are disabled */
#define CCCD_NOTIFICATION_INDICATION_DISABLED	(0xFFF4)

/* CCCD bits value length */
#define CCCD_VALUE_LENGTH						(2)

/* CCCD bits value offset */
#define CCCD_VALUE_OFFSET						(0)

/* Connection handle default value */
#define DEFAULT_CONNECTION_HANDLE				(0xFFFF)

/************************************************************************/
/*							Types										*/
/************************************************************************/

/**
 * @brief otau_gatt_service_handler is the service handler function
 */
typedef struct otau_gatt_service_handler {
	/** OTAU service connection Handle */
	at_ble_handle_t conn_hanle;
	/** OTAU service uuid */
	at_ble_uuid_t service_uuid;
	/** OTAU service handle */
	at_ble_handle_t service_handle;
	/** OTAU characteristic handle */
	at_ble_characteristic_t chars[OTAU_MAX_CHARACTERISTICS];
} otau_gatt_service_handler_t;

/************************************************************************/
/*						  Functions 			                        */
/************************************************************************/

/**@brief OTAU GATT service and characteristic initialization to default values
 *
 * @param[in] OTAU service instance pointer
 *
 * @return at_ble_status_t AT_BLE_SUCCESS or AT_BLE_FAILURE
 */
at_ble_status_t otau_service_get_default(otau_gatt_service_handler_t *otau_service);

/**@brief Register a OTAU service instance into database.
 *
 * @param[in] otau_service_instance OTAU service instance
 * @param[in] service_type  primary service or secondary service
 *
 * @return @ref AT_BLE_SUCCESS operation completed successfully
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t otau_gatt_service_define(otau_gatt_service_handler_t *otau_service_instance, at_ble_service_type_t service_type);

/**@brief otau_characteritics_changed_handler for characteristic CCCD write
 *
 * @param[in] otau_service_instance of type otau_gatt_service_handler_t
 * @param[in] params of type at_ble_characteristic_changed_t
 *
 * @return number representing the changed characteristic in case of failure CCCD_WRITE_UNUSED
 *			CCCD_WRITE_FAILED will be returned
 */
uint16_t otau_characteritics_changed_handler(otau_gatt_service_handler_t *otau_service_instance,
		at_ble_characteristic_changed_t *params);
		
/**@brief its an utility function to reverse the memory content of given length
 *
 * @param[in] buf pointer to memory location
 * @param[in] len length of the memory needs to be reversed
 *
 * @return at_ble_status_t AT_BLE_SUCCESS or AT_BLE_FAILURE
 */
void reverse_byte_buffer(uint8_t *buf, uint16_t len);
		
#ifdef __cplusplus
}
#endif

#endif /* __OTAU_SERVICE_H__ */

