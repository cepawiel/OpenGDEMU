/**
 * \file bm_application_mode.h
 *
 * \brief BM application mode declarations
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

/** \file bm_application_mode.h
 * Defines application mode APIs.
 * Support for configure mode, pairing procedure and events.
 */

#ifndef __BM_APPLICATION_MODE_H__
#define __BM_APPLICATION_MODE_H__

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"


#define BM_PKT_LEN_MAX    0x01FF
#define BM_CFG_CMD_SOF_VAL    0xAA

#define BM_APPLICATION_MODE_CMD(x)	(appCmds[x])
#define BM_APPLICATION_MODE_CMD_LEN(x)	(appCmds[x].length)

#define BM_LE_ADV_INTERVAL_UNIT (640)    /* 0.625ms x 1024 */
#define BM_LE_ADV_INTERVAL_VAL(x)   (((uint32_t)x << 10)/BM_LE_ADV_INTERVAL_UNIT)
#define BM_LE_ADV_INTERVAL_MIN  (0x0020)
#define BM_LE_ADV_INTERVAL_MAX  (0x4000)
#define BM_LE_ADV_INTERVAL_MS(x)  (((uint32_t)x * BM_LE_ADV_INTERVAL_UNIT) >> 10)

/* Advertisement report */
#define ADV_REPORT_EVENT_TYPE_START			0
#define ADV_REPORT_EVENT_TYPE_LEN			1
#define ADV_REPORT_ADDRESS_TYPE_START		(ADV_REPORT_EVENT_TYPE_START + ADV_REPORT_EVENT_TYPE_LEN)
#define ADV_REPORT_ADDRESS_TYPE_LEN			1
#define ADV_REPORT_ADDRESS_START			(ADV_REPORT_ADDRESS_TYPE_START + ADV_REPORT_ADDRESS_TYPE_LEN)
#define ADV_REPORT_ADDRESS_LEN				6
#define ADV_REPORT_DATA_LENGTH_START		(ADV_REPORT_ADDRESS_START + ADV_REPORT_ADDRESS_LEN)
#define ADV_REPORT_DATA_LENGTH_LEN			1
#define ADV_REPORT_DATA_START				(ADV_REPORT_DATA_LENGTH_START + ADV_REPORT_DATA_LENGTH_LEN)
#define ADV_REPORT_RSSI_LEN					1

/* Discover specific primary service characteristic response */
#define SERVICE_DISC_CHAR_RESP_CONN_HANDLE_START				0
#define SERVICE_DISC_CHAR_RESP_CONN_HANDLE_LEN					1
#define SERVICE_DISC_CHAR_RESP_LENGTH_START						(SERVICE_DISC_CHAR_RESP_CONN_HANDLE_START + SERVICE_DISC_CHAR_RESP_CONN_HANDLE_LEN)
#define SERVICE_DISC_CHAR_RESP_LENGTH_LEN						1
#define SERVICE_DISC_CHAR_RESP_ATTRIB_DATA_START				(SERVICE_DISC_CHAR_RESP_LENGTH_START + SERVICE_DISC_CHAR_RESP_LENGTH_LEN)
#define SERVICE_DISC_CHAR_RESP_ATTRIB_HANDLE_START				0
#define SERVICE_DISC_CHAR_RESP_ATTRIB_HANDLE_LEN				2
#define SERVICE_DISC_CHAR_RESP_CHAR_PROPERTY_START				(SERVICE_DISC_CHAR_RESP_ATTRIB_HANDLE_START + SERVICE_DISC_CHAR_RESP_ATTRIB_HANDLE_LEN)
#define SERVICE_DISC_CHAR_RESP_CHAR_PROPERTY_LEN				1
#define SERVICE_DISC_CHAR_RESP_CHAR_VAL_ATTRIB_HANDLE_START		(SERVICE_DISC_CHAR_RESP_CHAR_PROPERTY_START + SERVICE_DISC_CHAR_RESP_CHAR_PROPERTY_LEN)
#define SERVICE_DISC_CHAR_RESP_CHAR_VAL_ATTRIB_HANDLE_LEN		2
#define SERVICE_DISC_CHAR_RESP_CHAR_UUID_START					(SERVICE_DISC_CHAR_RESP_CHAR_VAL_ATTRIB_HANDLE_START + SERVICE_DISC_CHAR_RESP_CHAR_VAL_ATTRIB_HANDLE_LEN)
#define SERVICE_DISC_CHAR_RESP_CHAR_UUID_16_LEN					2
#define SERVICE_DISC_CHAR_RESP_CHAR_UUID_128_LEN				16

/* Discover all characteristic descriptor response */
#define DESC_DISC_RESP_CONN_HANDLE_START		0
#define DESC_DISC_RESP_CONN_HANDLE_LEN			1
#define DESC_DISC_RESP_FORMAT_START				(DESC_DISC_RESP_CONN_HANDLE_START + DESC_DISC_RESP_CONN_HANDLE_LEN)
#define DESC_DISC_RESP_FORMAT_LEN				1
#define DESC_DISC_RESP_DATA_START				(DESC_DISC_RESP_FORMAT_START + DESC_DISC_RESP_FORMAT_LEN)
#define DESC_DISC_RESP_ATTRIB_HANDLE_START		0
#define DESC_DISC_RESP_ATTRIB_HANDLE_LEN		2
#define DESC_DISC_RESP_UUID_START				(DESC_DISC_RESP_ATTRIB_HANDLE_START + DESC_DISC_RESP_ATTRIB_HANDLE_LEN)
#define DESC_DISC_RESP_UUID_16B_LEN				2
#define DESC_DISC_RESP_UUID_128B_LEN			16

typedef enum _bm_application_event_opcode
{
	BM_EVENT_NONE = 0x00,
    BM_PASSKEY_REQUEST       	= 0x60,
    BM_PAIR_COMPLETE,
    BM_PASSKEY_YESNO_REQUEST,
    BM_ADVERTISING_REPORT	  	= 0x70,
    BM_LE_CONNECT_COMPLETE,
    BM_DISCONNECT_COMPLETE,
    BM_CONNECTION_PARAMTER_UPDATE,
    BM_COMMAND_COMPLETE       	= 0x80,
    BM_STATUS_REPORT,
	BM_LE_END_TEST_RESULT,
    BM_CONFIGURE_MODE_STATUS  	= 0x8F,
    BM_CLIENT_DISCOVER_ALL_SERVICES_RESULT 	= 0x90,
    BM_CLIENT_DISCOVER_CHARACTERISTICS_RESULT,
    BM_CLIENT_DISCOVER_CHARACTERISTICS_DESCRIPTORS_RESULT,
    BM_CLIENT_CHARACTERISTIC_VALUE_RECEIVED,
    BM_SERVER_CHARACTERICTIC_VALUE_WRITE = 0x98,
    BM_SERVER_CHARACTERISTIC_VALUE_READ,
    BM_TRANSPARENT_DATA_RECEIVED    	= 0x9A,
    BM_SERVER_PREPARE_WRITE_REQUEST,
    BM_SERVER_EXECUTE_WRITE_REQUEST,
    BM_SERVER_BLOB_READ_REQUEST = 0xA0,
    BM_CLIENT_PREPARE_WRITE_RESPONSE,
    BM_CLIENT_EXECUTE_WRITE_RESPONSE,
    BM_ERROR = 0xFF
} BM_APPLICATION_EVENT_OPCODE;

typedef enum _bm_status
{
    BM_STATUS_POWER_ON       = 0x00,
    BM_STATUS_SCANNING,
    BM_STATUS_CONNECTING,    
    BM_STATUS_STANDBY,
    BM_STATUS_BROADCAST,
    BM_STATUS_TRANSPARENT_UART,
    BM_STATUS_IDLE,
    BM_STATUS_SHUTDOWN,
    BM_STATUS_CONFIGURE,
    BM_STATUS_CONNECTED
} BM_STATUS;

/*! \enum BM_PASSKEY_ACTION_OPTION
 * Enumeration of the passkey entry actions during pairing procedure.
 */
enum BM_PASSKEY_ACTION_OPTION
{
	/*! Passkey digit enter. */
	BM_PASSKEY_DIGIT_ENTER = 1,
	/*! Passkey digit erase. */
	BM_PASSKEY_DIGIT_ERASE,
	/*! Passkey clear. */
	BM_PASSKEY_CLEAR,
	/*! Passkey entry complete. */
	BM_PASSKEY_ENTRY_COMPLETE
};

typedef enum _bm_application_cmd_index
{
	BM_LOCAL_INFORMATION_READ_INDEX = 0x00,
    BM_RESET_INDEX,
    BM_STATUS_READ_INDEX,
    BM_ADC_READ_INDEX,
    BM_SHUTDOWN_INDEX,
	BM_DEBUG_INDEX,
    BM_NAME_READ_INDEX,
    BM_NAME_WRITE_INDEX,
    BM_PDL_ERASE_INDEX,
    BM_PAIR_MODE_READ_INDEX,
    BM_PAIR_MODE_WRITE_INDEX,
    BM_PDL_READ_INDEX,
    BM_DEVICE_ERASE_INDEX,
	BM_DIO_CONTROL_INDEX,
	BM_PWM_CONTROL_INDEX,
    BM_RSSI_READ_INDEX,
	BM_ADV_DATA_WRITE_INDEX,
    BM_SCAN_DATA_WRITE_INDEX,
	BM_ADV_PARAM_SET_INDEX,
	BM_CONN_PARAM_SET_INDEX,
	BM_SCAN_PARAM_SET_INDEX,
	BM_SCAN_ENABLE_SET_INDEX,
	BM_CONNECT_INDEX,
	BM_CONNECT_CANCEL_INDEX,
	BM_CONNECTION_PARAM_UPDATE_INDEX,
    BM_DISCONNECT_INDEX,
	BM_SET_ADV_ENABLE_INDEX,
    BM_REMOTE_NAME_READ_INDEX,
    BM_CLIENT_WRITE_REQUEST_PREPARE_INDEX,
    BM_CLIENT_WRITE_REQUEST_EXECUTE_INDEX,
    BM_CLIENT_BLOB_REQUEST_READ_INDEX,
    BM_CLIENT_HANDLE_VALUE_CONFIRM_INDEX,
	BM_CLIENT_DISCOVER_ALL_SERVICES_INDEX,
	BM_CLIENT_DISCOVER_CHARACTERISTICS_INDEX,
	BM_CLIENT_CHARACTERISTIC_READ_INDEX,
	BM_CLIENT_CHARACTERISTIC_UUID_READ_INDEX,
	BM_CLIENT_CHARACTERISTIC_WRITE_INDEX,
	BM_TRANSPARENT_ENABLE_INDEX,
	BM_SERVER_CREATE_SERVICE_REQUEST_INDEX,
    BM_SERVER_WRITE_RESPONSE_PREPARE_INDEX,
    BM_SERVER_WRITE_RESPONSE_EXECUTE_INDEX,
    BM_SERVER_BLOB_RESPONSE_READ_INDEX,
    BM_SERVER_ERROR_RESPONSE_INDEX,
	BM_SERVER_CHARACTERISTIC_SEND_INDEX,
	BM_SERVER_CHARACTERISTIC_UPDATE_INDEX,
	BM_SERVER_CHARACTERISTIC_READ_INDEX,
	BM_SERVER_ALL_SERVICES_READ_INDEX,
	BM_SERVER_SERVICE_READ_INDEX,
	BM_SERVER_WRITE_RESPONSE_SEND_INDEX,
    BM_SERVER_READ_RESPONSE_SEND_INDEX,
	BM_TRANSPARENT_DATA_SEND_INDEX,
    BM_PASSKEY_ENTRY_INDEX,
    BM_PASSKEY_YESNO_CONFIRM_INDEX,
    BM_PAIRING_REQUEST_INDEX,
	BM_CONFIG_MODE_CLOSE_INDEX,
	BM_RECEIVER_TEST_INDEX,
	BM_TRANSMITTER_TEST_INDEX,
	BM_END_TEST_INDEX,
    BM_PATTERN_SET_INDEX,
} BM_APPLICATION_CMD_INDEX;

typedef enum _bm_application_cmd_opcode
{
	BM_LOCAL_INFORMATION_READ 	= 0x01,
    BM_RESET,
    BM_STATUS_READ,
    BM_ADC_READ,
    BM_SHUTDOWN             	= 0x05,
    BM_DEBUG,
    BM_NAME_READ,
    BM_NAME_WRITE,
    BM_PDL_ERASE,
    BM_PAIR_MODE_READ,
    BM_PAIR_MODE_WRITE,
    BM_PDL_READ,
    BM_DEVICE_ERASE,
    BM_DIO_CONTROL,
    BM_PWM_CONTROL,
    BM_RSSI_READ              	= 0x10,
    BM_ADV_DATA_WRITE,
    BM_SCAN_DATA_WRITE,
	BM_ADV_PARAM_SET            = 0x13,
	BM_CONN_PARAM_SET,
	BM_SCAN_PARAM_SET           = 0x15,
	BM_SCAN_ENABLE_SET,
	BM_CONNECT,
	BM_CONNECT_CANCEL,
	BM_CONNECTION_PARAM_UPDATE,
    BM_DISCONNECT             	= 0x1B,
	BM_SET_ADV_ENABLE           = 0x1C,
	BM_REMOTE_NAME_READ         = 0x1F,
    BM_CLIENT_WRITE_REQUEST_PREPARE = 0x20,
    BM_CLIENT_WRITE_REQUEST_EXECUTE,
    BM_CLIENT_BLOB_REQUEST_READ,
    BM_CLIENT_HANDLE_VALUE_CONFIRM = 0x2D,
	BM_CLIENT_DISCOVER_ALL_SERVICES	= 0x30,
	BM_CLIENT_DISCOVER_CHARACTERISTICS,
	BM_CLIENT_CHARACTERISTIC_READ,
	BM_CLIENT_CHARACTERISTIC_UUID_READ,
	BM_CLIENT_CHARACTERISTIC_WRITE,
	BM_TRANSPARENT_ENABLE,
	BM_SERVER_CREATE_SERVICE_REQUEST = 0x27,
    BM_SERVER_WRITE_RESPONSE_PREPARE = 0x28,
    BM_SERVER_WRITE_RESPONSE_EXECUTE,
    BM_SERVER_BLOB_RESPONSE_READ = 0x2A,
    BM_SERVER_ERROR_RESPONSE = 0x37,
	BM_SERVER_CHARACTERISTIC_SEND = 0x38,
	BM_SERVER_CHARACTERISTIC_UPDATE,
	BM_SERVER_CHARACTERISTIC_READ,
	BM_SERVER_ALL_SERVICES_READ,
	BM_SERVER_SERVICE_READ,
	BM_SERVER_WRITE_RESPONSE_SEND,
    BM_SERVER_READ_RESPONSE_SEND,        
	BM_TRANSPARENT_DATA_SEND 	= 0x3F,
    BM_PASSKEY_ENTRY          	= 0x40,
    BM_PASSKEY_YESNO_CONFIRM,
    BM_PAIRING_REQUEST,		
	BM_CONFIG_MODE_CLOSE		= 0x52,
	BM_RECEIVER_TEST,
	BM_TRANSMITTER_TEST,
	BM_END_TEST,
    BM_PATTERN_SET              = 0x57,
} BM_APPLICATION_CMD_OPCODE;

typedef enum _bm_application_cmd_state
{
	BM_CFG_CMD_SOF = 0x00,
    BM_CFG_CMD_LENH,
    BM_CFG_CMD_LENL,
    BM_CFG_CMD_OPCODE,
    BM_CFG_CMD_DATA,
    BM_CFG_CMD_CHKSUM
} BM_APPLICATION_CMD_STATE;

typedef enum _bm_adc_channel
{
    BM_ADC_CHANNEL_0 = 0x00,
    BM_ADC_CHANNEL_1,
    BM_ADC_CHANNEL_2,
    BM_ADC_CHANNEL_3,
    BM_ADC_CHANNEL_4,
    BM_ADC_CHANNEL_5,
    BM_ADC_CHANNEL_6,
    BM_ADC_CHANNEL_7,
    BM_ADC_CHANNEL_8,
    BM_ADC_CHANNEL_9,
    BM_ADC_CHANNEL_10,
    BM_ADC_CHANNEL_11,
    BM_ADC_CHANNEL_12,
    BM_ADC_CHANNEL_13,
    BM_ADC_CHANNEL_14,
    BM_ADC_CHANNEL_15,
    BM_BATTERY_VOLTAGE,
    BM_TEMPERATURE_VALUE
}BM_ADC_CHANNEL;


typedef struct _bm_application_cmd
{
	BM_APPLICATION_CMD_OPCODE opcode;
	uint16_t length;
} BM_APPLICATION_CMD;

/*! \enum BM_APPLICATION_RSP_RET
 * Enumeration of the Application mode return response.
 */
typedef enum _bm_application_rsp_ret
{
    /*! Error in response. */
    BM_CFG_RSP_ERROR = -1,
    /*! Unknown response. */
    BM_CFG_RSP_UNKNOWN = 0,
    /*! Response for successful command. */
    BM_CFG_RSP_AOK,
    /*! Response for unsuccessful command. */
    BM_CFG_RSP_ERR
}BM_APPLICATION_RSP_RET;

/*! \enum BM_PAIR_MODE_RSP
 * Enumeration of the Pair mode return response.
 */
typedef enum _bm_pair_mode_rsp
{
    /*! Display only. */
    BM_RSP_DISPLAY_ONLY = 0,
    /*! Display yes/no confirm.. */
    BM_RSP_DISPLAY_CONFIRM = 1,
    /*! Keyboard only. */
    BM_RSP_KEYBOARD_ONLY = 2,
    /*! No input no output. */
    BM_RSP_NO_INPUT_OUTPUT = 3,
    /*! Display keyboard. */
    BM_RSP_DISPLAY_KEYBOARD = 4
} BM_PAIR_MODE_RSP;

/*! \enum BM_ADV_TYPE_STORE
 * Enumeration of the Pair mode return response.
 */
typedef enum _bm_adv_type_store
{
    /*! Advertising data not stored to EEPROM. */
    BM_ADV_NO_EEPROM = 0x00,
    /*! Advertising data stored to EEPROM. */
    BM_ADV_EEPROM = 0x01,
    /*! Beacon data not stored to EEPROM. */
    BM_BEACON_NO_EEPROM = 0x80,
    /*! Beacon data stored to EEPROM. */
    BM_BEACON_EEPROM = 0x81
} BM_ADV_TYPE_STORE;

/*! \enum BM_ADV_TYPE
 * Enumeration of the Pair mode return response.
 */
typedef enum _bm_adv_type
{
    /*! Connectable undirected advertising. */
    BM_ADV_CONNECTABLE_UNDIRECTED = 0x00,
    /*! Connectable directed advertising. */
    BM_ADV_CONNECTABLE_DIRECTED,
    /*! Scannable undirected advertising. */
    BM_ADV_SCANNABLE_UNDIRECTED,
    /*! Non connectable undirected advertising. */
    BM_ADV_NONCONNECTABLE_UNDIRECTED,
    /*! Proprietary Beacon. */
    BM_ADV_PROPRIETARY_BEACON
} BM_ADV_TYPE;

/*! \enum BM_ADV_ADDRESS
 * Enumeration of the Pair mode return response.
 */
typedef enum _bm_adv_address
{
    /*! Public device address. */
    BM_ADV_ADDRESS_PUBLIC = 0x00,
    /*! Random device address. */
    BM_ADV_ADDRESS_RANDOM
} BM_ADV_ADDRESS;

typedef enum _bm_pair_mode
{
    /*! Display only. */
    BM_PAIR_DISPLAY_ONLY = 0,
    /*! Display yes/no confirm.. */
    BM_PAIR_DISPLAY_CONFIRM = 1,
    /*! Keyboard only. */
    BM_PAIR_KEYBOARD_ONLY = 2,
    /*! No input no output. */
    BM_PAIR_NO_INPUT_OUTPUT = 3,
    /*! Display keyboard. */
    BM_PAIR_DISPLAY_KEYBOARD = 4,
            
    BM_PAIR_INVALID_MODE = 5
} BM_PAIR_MODE;

/*! \enum BM_APPLICATION_ERROR_CODE
 * Enumeration of the Application mode error code return response.
 */
typedef enum _bm_application_error_code
{
	BM_COMMAND_SUCCESS = 0x00,
	BM_COMMAND_UNKNOWN,
	BM_CONNECTION_ID_UNKNOWN,
	BM_HARDWARE_FAIL,
	BM_AUTHENTICATION_FAIL = 0x05,
	BM_KEY_MISSING,
	BM_MEMORY_FULL,
	BM_CONNECTION_TIMEOUT,
	BM_CONNECTION_LIMIT,
	BM_ACL_EXISTS = 0x0B,
	BM_COMMAND_DISALLOWED,
	BM_CONNECTION_REJECT_RESOURCES,
	BM_CONNECTION_REJECT_SECURITY,
	BM_CONNECTION_REJECT_ADDRESS,
	BM_CONNECTION_ACCEPT_TIMEOUT,
	BM_FEATURE_UNKNOWN,
	BM_PARAMETERS_INVALID,
	BM_REMOTE_CONNECTION_END,
	BM_CONNECTION_END_RESOURCES,
	BM_CONNECTION_END_POWER,
	BM_CONNECTION_END_LOCAL,
	BM_PAIRING_DISALLOWED = 0x18,
	BM_ERROR_UNSPECIFIED = 0x1F,
	BM_INSTANT_PASS = 0x28,
	BM_PAIRING_KEY_DISALLOWED,
	BM_SECURITY_INSUFFICIENT = 0x2F,
	BM_CONNECTION_REJECT_CHANNEL = 0x39,
	BM_CONTROLLER_BUSY,
	BM_CONNECTION_INTERVAL_INVALID,
	BM_ADVERTISING_TIMEOUT,
	BM_CONNECTION_END_MIC,
	BM_CONNECTION_FAIL,
	BM_HANDLE_INVALID = 0x81,
	BM_READ_DISALLOWED,
	BM_WRITE_DISALLOWED,
	BM_PDU_INVALID,
	BM_AUTHENTICATION_INSUFFIECIENT,
	BM_REQUEST_INVALID,
	BM_OFFSET_INVALID,
	BM_AUTHORIZATION_INSUFFICIENT,
	BM_QUEUE_FULL,
	BM_ATTRIBUTE_UNSPECIFIED,
	BM_ATTRIBUTE_INSUFFICIENT,
	BM_ENCRYPTION_KEY_INSUFFICIENT,
	BM_ATTRIBUTE_LENGTH_INSUFFICIENT,
	BM_UNLIKELY_ERROR,
	BM_ENCRYPTION_INSUFFICIENT,
	BM_GROUP_INVALID,
	BM_RESOURCES_INSUFFICIENT,
	BM_UART_CHECKSUM_ERROR = 0xFF
} BM_APPLICATION_ERROR_CODE;

typedef struct _bm_application_event 
{
   	BM_APPLICATION_EVENT_OPCODE 	eventID;
    void*                           data;
    uint16_t                        length;
} BM_APPLICATION_EVENT;

typedef struct _primary_service_char_discovery_resp
{
	uint8_t conn_handle;
	uint8_t length;
	uint8_t attrib_data;	/* This field will have variable length data. Just a starting pointer will be used to invoke actual data */
}PRIMARY_SERVICE_CHAR_DISCOVERY_RESP;

typedef struct _all_desc_discovery_resp
{
	uint8_t conn_handle;
	uint8_t format;
	uint8_t data;	/* This field will have variable length data. Just a starting pointer will be used to invoke actual data */
}All_DESC_DISCOVERY_RESP;

/*! \struct BM_APPLICATION_CMDPKT
 * Application mode command packet. Consists of the length of and handle to command data.
 */
typedef struct _bm_application_cmdpkt
{
    /*! Length of the command data. */
    uint16_t length;
    /*! Handle to the command data. */
    uint8_t* cmdPkt;
} BM_APPLICATION_CMDPKT;

/*! \fn BM_APPLICATION_CMDPKT* BM_APPLICATION_Init(void)
 *  \brief Initializes application mode configuration library.
 *  \param bmxxDevice Bluetooth device type used. Refer BMxx_BLUETOOTH_DEVICE.
 *  \pre None.
 *  \return Handle to the application mode command packet.
 */
BM_APPLICATION_CMDPKT* BM_APPLICATION_Init(void);

/*! \fn void BM_APPLICATION_DeInit(BM_APPLICATION_CMDPKT* applicationCmdPkt)
 *  \brief De-initializes application mode configuration library.
 *  \param applicationCmdPkt Handle to the application mode command packet created by init function.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_DeInit(void);

/*! \fn bool BM_APPLICATION_ResponseCheck(uint8_t* data, uint8_t length)
 *  \brief De-initializes Application mode configuration library.
 *  \param data Handle to the response data received.
 *  \param length Length of the response data received.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return Operation status.
 */
bool BM_APPLICATION_ResponseCheck(uint8_t* data, uint8_t length);

/*! \fn void BM_APPLICATION_LocalInformationReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
 *  \brief Builds an application mode command packet which reads local information in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_LocalInformationReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);

/*! \fn void BM_APPLICATION_SystemDeviceNameReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
 *  \brief Builds an application mode command packet which reads device name in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_DeviceNameReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);

/*! \fn void BM_APPLICATION_SystemDeviceNameWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* bluetoothName, bool store)
 *  \brief Builds an application mode command packet which writes device name in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param bluetoothName Handle to the Bluetooth name.
 *  \param store EEPROM store option: TRUE - Store in EEPROM; FALSE - Do not store in EEPROM.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_DeviceNameWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* deviceName, uint8_t deviceNameLength);

void BM_APPLICATION_PatternSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t pattern);

void BM_APPLICATION_ResetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);

void BM_APPLICATION_StatusReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);

void BM_APPLICATION_ADCReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t adcChannel);

void BM_APPLICATION_ShutdownPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);

void BM_APPLICATION_DebugPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t debugOpcode, uint8_t* debugParams, uint8_t debugLength);

/*! \fn void BM_APPLICATION_PairDeviceListErasePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
 *  \brief Builds an application mode command packet to erase paired device list (PDL) in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairDeviceListErasePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);

/*! \fn void BM_APPLICATION_PairModeReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
 *  \brief Builds an application mode command packet to read pairing mode in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairModeReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);

/*! \fn void BM_APPLICATION_PairModeWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t pairMode, bool store)
 *  \brief Builds an application mode command packet which writes pairing mode in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param pairMode Pairing mode.
 *  \param store EEPROM store option: TRUE - Store in EEPROM; FALSE - Do not store in EEPROM.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairModeWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, BM_PAIR_MODE pairMode);

/*! \fn void BM_APPLICATION_PairDeviceListReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
 *  \brief Builds an application mode command packet to read paired device list in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairDeviceListReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);   

void BM_APPLICATION_DIOControlPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* dioControl);

void BM_APPLICATION_PWMControlPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* pwmControl);

void BM_APPLICATION_RSSIReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle);



/*! \fn void BM_APPLICATION_PairDeviceDeletePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t deviceIndex)
 *  \brief Builds an application mode command packet to delete paired device by index in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param deviceIndex Index for the device for which the pairing information is to be deleted.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairDeviceDeletePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t deviceIndex);

/*! \fn void BM_APPLICATION_AdvertiseDataWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* advertiseData, BM_ADV_TYPE_STORE store)
 *  \brief Builds an application mode command packet which writes the advertisement or Beacon data in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param advertiseData Handle to the advertisement or Beacon data.
 *  \param store Advertising type and EEPROM store options: BM_ADV_TYPE_STORE 
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_AdvertisementDataWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* advertisementData, uint8_t advertisementLength, BM_ADV_TYPE_STORE store);

/*! \fn void BM_APPLICATION_AdvertiseParameterSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* advertiseParameter)
 *  \brief Builds an application mode command packet which sets the advertising parameters in idle mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param advertiseParameter.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_AdvertisementParameterSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* advertiseParameter);

/*! \fn void BM_APPLICATION_AdvertiseEnableSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* advertiseEnable)
 *  \brief Builds an application mode command packet which enables advertising in idle mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param advertiseEnable.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_AdvertisementEnableSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t advertiseMode); 

/*! \fn void BM_APPLICATION_ScanDataWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* scanData, bool store)
 *  \brief Builds an application mode command packet which writes the advertise data in configure mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param advertiseData Handle to the advertise data.
 *  \param store EEPROM store option: TRUE - Store in EEPROM; FALSE - Do not store in EEPROM.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_ScanDataWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* scanData, uint8_t scanLength, bool store);

/*! \fn void BM_APPLICATION_ScanParameterSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* scanParameter)
 *  \brief Builds an application mode command packet which sets the scan parameters in idle mode.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param scanParameter.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_ScanParameterSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint16_t scanInterval, uint16_t scanWindow, uint8_t scanType);

void BM_APPLICATION_ScanEnableSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t scanEnable, uint8_t filterPolicy);

void BM_APPLICATION_ConnParameterSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint16_t min_conn_interval, uint16_t max_conn_interval, uint16_t slave_latency, uint16_t so_timeout);

void BM_APPLICATION_ConnectDevicePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* peerAddress, uint8_t peerAddressType, uint8_t filterPolicy);

void BM_APPLICATION_ConnectDeviceCancelPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);

void BM_APPLICATION_ConnectionParameterUpdatePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t connectionInterval, uint16_t connectionLatency, uint16_t supervisionTimeout);

void BM_APPLICATION_DisconnectDevicePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle);

void BM_APPLICATION_RemoteDeviceNameReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle);


void BM_APPLICATION_ClientAllServicesDiscoverPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle);

void BM_APPLICATION_ClientCharacteristicsDiscoverPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t* serviceUUID, uint8_t serviceUUIDLength);

void BM_APPLICATION_ClientCharacteristicHandleReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteriticHandle);

void BM_APPLICATION_ClientCharacteristicUUIDReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t* characteriticUUID, uint8_t characteriticUUIDLength);

void BM_APPLICATION_ClientCharacteristicHandleWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteristicHandle, uint8_t writeType, uint8_t* characteriticValue, uint8_t characteriticValueLength);

void BM_APPLICATION_ClientWriteRequestPreparePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteristicHandle, uint8_t writeType, uint16_t valueOffset, uint8_t* characteriticValue, uint8_t characteriticValueLength);

void BM_APPLICATION_ClientWriteRequestExecutePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t writeType, uint8_t flag);

void BM_APPLICATION_ClientBlobRequestReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteristicHandle, uint16_t valueOffset);

void BM_APPLICATION_ClientHandleValueConfirmPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle);

void BM_APPLICATION_EnableTransparentUartSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t serverControl, uint8_t clientMode);

void BM_APPLICATION_ServerCharacteristicSendPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteriticHandle, uint8_t* characteriticValue, uint8_t characteriticValueLength);

void BM_APPLICATION_ServerCharacteristicUpdatePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint16_t characteriticHandle, uint8_t* characteriticValue, uint8_t characteriticValueLength);

void BM_APPLICATION_ServerCharacteristicReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint16_t characteriticHandle);

void BM_APPLICATION_ServerPrimaryServicesReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt);

void BM_APPLICATION_ServerPrimaryServiceReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* serviceUUID, uint8_t serviceUUIDlength);

void BM_APPLICATION_ServerWriteResponseSendPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t requestOpcode, uint16_t attributeHandle, uint8_t errorCode);

void BM_APPLICATION_ServerReadResponseSendPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t responseType, uint8_t* characteriticValue, uint8_t characteriticValueLength);

void BM_APPLICATION_ServerWriteResponsePreparePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteristicHandle, uint8_t writeType, uint16_t valueOffset, uint8_t* characteriticValue, uint8_t characteriticValueLength);

void BM_APPLICATION_ServerWriteExecutePreparePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t writeType);

void BM_APPLICATION_ServerBlobResponseReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t responseOpcode, uint8_t* characteriticValue, uint8_t characteriticValueLength);

void BM_APPLICATION_ServerErrorResponsePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t requestOpcode, uint16_t characteristicHandle, uint8_t errorCode);

/*! \fn void BM_APPLICATION_SendTransparentDataPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* data, uint8_t datalen);
 *  \brief Enables transparent UART.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param data holds transparent uart data.
 *  \param datalen holds length of transparent uart data.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_SendTransparentDataPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t* data, uint8_t datalen);

/*! \fn void BM_APPLICATION_ModeClosePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, bool store)
 *  \brief Builds an application mode command packet which closes the configure mode. optionally the configure mode can be disabled from being invoked in future.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param store EEPROM store option: TRUE - Store in EEPROM; FALSE - Do not store in EEPROM.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_ConfigureModeClosePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, bool store);

void BM_APPLICATION_PairRequest(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle);

/*! \fn void BM_APPLICATION_PairPassKeyDigitEnterPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t passkeyDigit)
 *  \brief Builds an application mode command packet for entering a passkey digit in pairing procedure.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param passkeyDigit Passkey digit in pairing procedure.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairPassKeyDigitEnterPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t passkeyDigit);

/*! \fn void BM_APPLICATION_PairPassKeyDigitErasePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
 *  \brief Builds an application mode command packet for erasing the previously entered passkey digit in pairing procedure.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairPassKeyDigitErasePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle);

/*! \fn void BM_APPLICATION_PairPassKeyClearPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
 *  \brief Builds an application mode command packet for clearing the entered passkey in pairing procedure.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairPassKeyClearPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle);

/*! \fn void BM_APPLICATION_PairPassKeyEntryCompletePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
 *  \brief Builds an application mode command packet indicating passkey entry complete after entering all digits of passkey in pairing procedure.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairPassKeyEntryCompletePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle);

/*! \fn void BM_APPLICATION_PairYesNoConfirmPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, bool confirm)
 *  \brief Builds an application mode command packet for entering a yes/no confirmation in pairing procedure.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param confirm Passkey confirm option: TRUE - Passkey is correct; FALSE - Passkey is incorrect.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_PairYesNoConfirmPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, bool confirm);

/*! \fn void BM_APPLICATION_ServerCreateService(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t *service, uint8_t length)
 *  \brief Builds an application mode command packet for creating GATT-Server service.
 *  \param applicationCmdPkt Handle to the application mode configuration library created by init function.
 *  \param service Starting address to the GATT-Server service.
 *  \param length Length of the GATT-Server service.
 *  \param num_of_attrib Total number of attributes included.
 *  \pre Application mode library initialization function must be used to initialize the eepromCmdPkt.
 *  \return None.
 */
void BM_APPLICATION_ServerCreateService(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t *service, uint16_t length, uint8_t num_of_attrib);

/*! \fn void BM_Application_EventParser(BM_APPLICATION_EVENT_OPCODE event_id, void *data, uint32_t *dataLen)
 *  \brief Parse received events @ref ble_conn_complete_event_t.
 *  \param event_id event opcode.
 *  \param data event data.
 *  \param dataLen length of the data.
 *  \pre None.
 *  \return None.
 */
void BM_Application_EventParser(BM_APPLICATION_EVENT_OPCODE event_id, void *data, uint32_t *dataLen);

/*! \fn void BM_APPLICATION_AdvReportEventParser(void *data, uint32_t *dataLen)
 *  \brief Parse a advertisement report event @ref ble_adv_report_event_t.
 *  \param data event data.
 *  \param dataLen length of the data.
 *  \pre None.
 *  \return None.
 */
void BM_APPLICATION_AdvReportEventParser(void *data, uint32_t *dataLen);

/*! \fn void BM_APPLICATION_ConnCompleteEventParser(void *data, uint32_t *dataLen)
 *  \brief Parse a connection complete event @ref ble_conn_complete_event_t.
 *  \param data event data.
 *  \param dataLen length of the data.
 *  \pre None.
 *  \return None.
 */
void BM_APPLICATION_ConnCompleteEventParser(void *data, uint32_t *dataLen);

/*! \fn void BM_APPLICATION_ConnParamUpdateEventParser(void *data, uint32_t *dataLen)
 *  \brief Parse a connection parameter update event @ref ble_conn_param_update_event_t.
 *  \param data event data.
 *  \param dataLen length of the data.
 *  \pre None.
 *  \return None.
 */
void BM_APPLICATION_ConnParamUpdateEventParser(void *data, uint32_t *dataLen);

/*! \fn void BM_APPLICATION_LeEndTestResultEventParser(void *data, uint32_t *dataLen)
 *  \brief Parse a LE end test result event @ref ble_end_test_result_event_t.
 *  \param data event data.
 *  \param dataLen length of the data.
 *  \pre None.
 *  \return None.
 */
void BM_APPLICATION_LeEndTestResultEventParser(void *data, uint32_t *dataLen);

/*! \fn void BM_APPLICATION_WriteCharValueEventParser(void *data, uint32_t *dataLen)
 *  \brief Parse a Client write characteristic value event @ref ble_write_char_value_event_t.
 *  \param data event data.
 *  \param dataLen length of the data.
 *  \pre None.
 *  \return None.
 */
void BM_APPLICATION_WriteCharValueEventParser(void *data, uint32_t *dataLen);

/*! \fn void BM_APPLICATION_DiscoverCharRespEventParser(void *data, uint32_t *dataLen)
 *  \brief Parse a Discover primary service characteristic response event @ref PRIMARY_SERVICE_CHAR_DISCOVERY_RESP.
 *  \param data event data.
 *  \param dataLen length of the data.
 *  \pre None.
 *  \return None.
 */
void BM_APPLICATION_DiscoverCharRespEventParser(void *data, uint32_t *dataLen);
#endif /* __BM_APPLICATION_MODE_H__ */
