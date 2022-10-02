/**
 * \file
 *
 * \brief Heart Rate Service declarations
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
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Atmel
 * Support</a>
 */
// <<< Use Configuration Wizard in Context Menu >>>
// <h> Blood Pressure Service Configuration
// =======================

#ifndef __BLOOD_PRESSURE_H__
#define __BLOOD_PRESSURE_H__

/************************************************************************/
/*                              Includes                                */
/************************************************************************/


/*************************************************************************/
/*								MACROS								     */
/*************************************************************************/

/**
 * @brief Blood Pressure service UUID
 */
//#define BLOOD_PRESSURE_SERVICE_UUID                                                 (0x1810)

/**
 * @brief Blood Pressure measurement characteristic UUID
 */
#define BLOOD_PRESSURE_MEASUREMENT_CHAR_UUID										(0x2A35)

/**
 * @brief Intermediate Cuff Pressure characteristic UUID
 */
#define INTERMEDIATE_CUFF_PRESSURE_CHAR_UUID										(0x2A36)

/**
 * @brief Blood Pressure Feature characteristic UUID
 */
#define BLOOD_PRESSURE_FEATURE_CHAR_UUID											(0x2A49)

/**
 * @brief Total no of characteristics in Blood Pressure service
 */
#define BLP_TOTAL_CHARATERISTIC_NUM													(3)


/**
 * @brief Notification Mask
 */
#define BLP_NOTIFICATION															(1)

/**
 * @brief Disabling the Indication
 */
#define BLP_INDICATION																(2)

/** 
 * @brief notification enable
 *
 */
#define BLP_NOTIFICATION_ENABLE														 (1)

#define BLP_NOTIFICATION_DISABLE													 (2)

#define BLP_INDICATION_ENABLE														 (3)

#define BLP_INDICATION_DISABLE														 (4)

/**
 * @brief maximum size of flags field of bp mm characteristic in bytes
 */
#define BLP_MM_FLAGS_SIZE                                                            (1)

/**
 * @brief maximum size of systolic  field of blp mm characteristic in bytes
 */
#define BLP_SYSTOLIC_VAL_SIZE                                                        (2)

/**
 * @brief maximum size of diastolic  field of blp mm characteristic in bytes
 */
#define BLP_DIASTOLIC_VAL_SIZE                                                       (2)

/**
 * @brief maximum size of map  field of blp mm characteristic in bytes
 */
#define BLP_MAP_VAL_SIZE		                                                     (2)

/**
 * @brief maximum size of time stamp field of  blp mm characteristic in bytes
 */
#define BLP_MAX_TIME_STAMP_SIZE                                                      (7)

/**
 * @brief maximum size of pulse rate in characteristic in bytes
 */
#define BLP_MAX_PULSE_RATE_SIZE                                                      (2)

/**
 * @brief maximum size of user id in characteristic in bytes
 */
#define BLP_MAX_USER_ID_SIZE	                                                     (1)

/**
 * @brief maximum size of measurement status in characteristic in bytes
 */
#define BLP_MAX_MM_STATUS_SIZE	                                                     (2)


#define DEFAULT_VALUE																 (0)
/************************************************************************/
/*							Types										*/
/************************************************************************/

/**
 * @brief hr_gatt_service_handler is the service handler function
 */
typedef struct hr_gatt_service_handler {
	/** service uuid */
	at_ble_uuid_t serv_uuid;
	/** service handle */
	at_ble_handle_t serv_handle;
	/** characteristic handle */
	at_ble_characteristic_t serv_chars[BLP_TOTAL_CHARATERISTIC_NUM];
} blp_gatt_service_handler_t;

/************************************************************************/
/*						  Functions 			                        */
/************************************************************************/

/**@brief Blood Pressure service and characteristic initialization(Called only once
 * by user).
 *
 * @param[in] Blood Pressure service instance
 *
 * @return none
 */
void blp_init_service(blp_gatt_service_handler_t *blood_pressure_serv);

/**@brief Register a blp service instance inside stack.
 *
 * @param[in] dis_primary_service blp service instance
 *
 * @return @ref AT_BLE_SUCCESS operation completed successfully
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t blp_primary_service_define(
		blp_gatt_service_handler_t *blp_primary_service);

/**@brief handler for characteristic write called by profile
 *
 * @param[in] hr_gatt_service_handler_t servcie handler
 *
 * @return number representing the changed characteristic
 */
uint8_t blp_char_change_handler(blp_gatt_service_handler_t *blp_primary_service,
		at_ble_characteristic_changed_t *params);

#endif /* __BLOOD_PRESSURE_H__ */
// </h>

// <<< end of configuration section >>>
