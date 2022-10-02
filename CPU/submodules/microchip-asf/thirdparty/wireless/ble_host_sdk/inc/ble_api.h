/**
 * \file ble_api.h
 *
 * \brief BLE API declarations
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

#ifndef __BLE_API_H__
#define __BLE_API_H__

/** DEFINES  ******************************************************/
#define BLE_CCCD_UUID			0x2902
#define BLE_CCCD_NOTIFICATION_ENABLED	0x0001
#define BLE_CCCD_INDICATION_ENABLED		0x0002

#define BLE_ADDR_LEN			6

#define BM77_BLUETOOTH_DEVICE	0
#define BM78_BLUETOOTH_DEVICE	1
#define BM70_BLUETOOTH_DEVICE	2

#define BMXX_DEVICE (BM70_BLUETOOTH_DEVICE)

/* Advertisement parameter */
#define ADV_PARAM_INTERVAL_INDEX_HIGH		0
#define ADV_PARAM_INTERVAL_INDEX_LOW		1
#define ADV_PARAM_TYPE_INDEX				2
#define ADV_PARAM_DIRECT_ADDR_TYPE_INDEX	3
#define ADV_PARAM_DIRECT_ADDR_INDEX			4

/* Advertisement interval */
#define ADV_INTERVAL_1_S					1600	/* (1600 * 625 us) */
#define ADV_INTERVAL_100_MS					160

/* GAP Advertising interval max. and min. */
#define AT_BLE_ADV_INTERVAL_MIN        0x0020 /**< Minimum Advertising interval in 625 us units, i.e. 20 ms. */
#define AT_BLE_ADV_INTERVAL_MAX        0x4000 /**< Maximum Advertising interval in 625 us units, i.e. 10.24 s. */

/* Scan parameters */
#define MIN_SCAN_INTERVAL		0x0004
#define MAX_SCAN_INTERVAL		0x4000
#define MIN_SCAN_WINDOW			0x0004
#define MAX_SCAN_WINDOW			0x4000

/* Connection parameters */
#define MIN_CONN_INTERVAL		0x0006
#define MAX_CONN_INTERVAL		0x0C80
#define MIN_SLAVE_LATENCY		0x0000
#define MAX_SLAVE_LATENCY		0x01F4
#define MIN_SV_TIMEOUT			0x000A
#define MAX_SV_TIMEOUT			0x0C80

#define BLE_PAIR_CONFIRM_YES true
#define BLE_PAIR_CONFIRM_NO false

#define BLE_UUID_128B_LEN	(1 << BLE_UUID_128B)
#define BLE_UUID_16B_LEN	(1 << BLE_UUID_16B)

#define MAX_CHAR_WRITE_VALUE	20

/**@brief BLE Attribute UUID lengths */
/**< 16-bit Bluetooth UUID. */
#define BLE_ATTRIB_UUID_LENGTH_2		2
/**< 32-bit Bluetooth UUID. */
#define BLE_ATTRIB_UUID_LENGTH_4		4
/**< 128-bit Bluetooth UUID. */
#define BLE_ATTRIB_UUID_LENGTH_16		16

/**< Characteristic properties */
#define BLE_CHAR_BROADCST				(0x01 << 0)
#define BLE_CHAR_READ					(0x01 << 1)
#define BLE_CHAR_WRITE_WITHOUT_RESPONSE (0x01 << 2)
#define BLE_CHAR_WRITE					(0x01 << 3)
#define BLE_CHAR_NOTIFY					(0x01 << 4)
#define BLE_CHAR_INDICATE				(0x01 << 5)
#define BLE_CHAR_SIGNED_WRITE			(0x01 << 6)
#define BLE_CHAR_EXT_PROPERTIES			(0x01 << 7)

/**< Characteristic extended properties */
#define BLE_CHAR_EXT_PROP_RELIABLE_WRITE	(0x0001 << 0)
#define BLE_CHAR_EXT_PROP_WRITABLE_AUX		(0x0001 << 1)

/* The following values inherited from BLEDK3. This needs to be updated whenever BLEDK3 update these values */

/**< Attribute Permissions. All attributes are readable as default. */
#define BLE_PM_WRITABLE					0x02	/**< Access Permission: Writeable.*/
#define BLE_PM_SECURITY_READ_ENABLE		0x04	/**< Encryption/Authentication Permission on READ property attribute. If enabled, it requires pairing to device to access the attribute. Note that Encryption or Authentication permission is based on IO capability of device. */
#define BLE_PM_SECURITY_WRITE_ENABLE	0x08	/**< Encryption/Authentication Permission on WRITE property attribute. If enabled, it requires pairing to device to access the attribute. Note that Encryption or Authentication permission is based on IO capability of device. */
#define BLE_PM_MANUAL_WRITE_RESP		0x40	/**< Authorization Permission: Manual sending write response configuration. Application can send write response manually. */
#define BLE_PM_MANUAL_READ_RESP			0x80	/**< Authorization Permission: Manual sending read response configuration. Application can send read response with data manually. */

#define BLE_ATT_ATTRIBUTE_VALUE_LEN			23		/**< The Maximum Length of Attribute Value. Refer to ATT default MTU size. */
#define BLE_ATT_DEFAULT_MTU_LENGTH			23		/**< ATT default MTU size. */

#define BLE_MAX_SERVICE_BUF_LEN    			0x01FF

typedef bool ble_pair_confirm_t;
typedef BM_APPLICATION_EVENT_OPCODE bm_app_event_opcode_t;
/****************************************************************************************
*										Enumerations									*
****************************************************************************************/
/**@brief BLE Host SDK status messages
*/
typedef enum 
{
	BLE_SUCCESS  = 0x00,
	BLE_UNKNOWN_COMMAND,
	BLE_UNKNOWN_CONNECTION_ID,
	BLE_HARDWARE_FAILURE,
	BLE_AUTHENTICATION_FAILURE = 0x05,
	BLE_PIN_KEY_MISSING,
	BLE_MEMORY_CAPACITY_EXCEEDED,
	BLE_CONNECTION_TIMEOUT,
	BLE_CONNECTION_LIMIT_EXCEEDED,
	BLE_ACL_CONNECTION_ALREADY_EXISTS = 0x0B,
	BLE_COMMAND_DISALLOWED,
	BLE_CONNECTION_REJECTED_DUE_TO_LIMITED_RESOURCES,
	BLE_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS,
	BLE_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR,
	BLE_CONNECTION_ACCEPT_TIMEOUT_EXCEEDED,
	BLE_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE,
	BLE_INVALID_COMMAND_PARAMETERS,
	BLE_REMOTE_USER_TERMINATED_CONNECTION,
	BLE_REMOTE_USER_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES,
	BLE_REMOTE_USER_TERMINATED_CONNECTION_DUE_TO_POWER_OFF,
	BLE_CONNECTION_TERMINATED_BY_LOCAL_HOST,
	BLE_PAIRING_NOT_ALLOWED = 0x18,
	BLE_UNSPECIFIED_ERROR = 0x1F,
	BLE_INSTANT_PASSED = 0x28,
	BLE_PAIRING_WITH_UINT_KEY_NOT_SUPPORTED = 0x29,
	BLE_INSUFFICIENT_SECURITY = 0x2F,
	BLE_CONNECTION_REJECTED_DUE_TO_NO_SUITABLE_CHANNEL_FOUND = 0x39,
	BLE_CONTROLLER_BUSY,
	BLE_UNACCEPTABLE_CONNECTION_INTERVAL,
	BLE_DIRECTED_ADVERTISING_TIMEOUT,
	BLE_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE,
	BLE_CONNECTION_FAILED_TO_BE_ESTABLISHED,
	BLE_INVALID_HANDLE = 0x81,
	BLE_READ_NOT_PERMITTED,
	BLE_WRITE_NOT_PERMITTED,
	BLE_INVALID_PDU,
	BLE_INSUFFICIENT_AUTHENTICATION,
	BLE_REQUEST_NOT_SUPPORTED,
	BLE_INVALID_OFFSET,
	BLE_INSUFFICIENT_AUTHORIZATION = 0x88,
	BLE_PREPARE_QUEUE_FULL,
	BLE_ATTRIBUTE_NOT_FOUND,
	BLE_ATTRIBUTE_NOT_LONG,
	BLE_INSUFFICIENT_ENCRYPTION_KEY_SIZE,
	BLE_INVALID_ATTRIBUTE_VALUE_LENGTH,
	BLE_UNLIKELY_ERROR,
	BLE_INSUFFICIENT_ENCRYPTION,
	BLE_UNSUPPORTED_GROUT_TYPE,
	BLE_INSUFFICIENT_RESOURCES,
	BLE_APPLICATION_DEFINED_ERROR = 0xF0,
	BLE_FAILURE,
	BLE_UART_TIMEOUT,
	BLE_UART_CRC_ERROR = 0xFF,
}ble_status_t;

/**@brief Advertising Types
*/
typedef enum
{
	BLE_ADV_TYPE_UNDIRECTED    = 0x00,   /**< Connectable undirected. */
	BLE_ADV_TYPE_DIRECTED,               /**< Connectable high duty cycle directed advertising. */
	BLE_ADV_TYPE_SCANNABLE_UNDIRECTED,   /**< Scannable undirected. */
	BLE_ADV_TYPE_NONCONN_UNDIRECTED,     /**< Non connectable undirected. */
	BLE_ADV_TYPE_DIRECTED_LDC,           /**< Connectable low duty cycle directed advertising. */
	BLE_ADV_TYPE_SCAN_RESPONSE           /** only used in @ref BLE_SCAN_INFO event to signify a scan response*/
} ble_adv_type_t;

/**@brief Advertising Mode
*/
typedef enum
{
	/* Mode in non-discoverable */
	BLE_ADV_NON_DISCOVERABLE,
	/* Mode in general discoverable, AD type general flag in Flags set to 1. */
	BLE_ADV_GEN_DISCOVERABLE,
	/* Mode in limited discoverable, AD type limited flag in Flags set to 1 (This mode is automatically stopped after 180 sec of activity) */
	BLE_ADV_LIM_DISCOVERABLE,
	/* Broadcaster mode which is a non discoverable and non connectable mode. */
	BLE_ADV_BROADCASTER_MODE
} ble_adv_mode_t;

/**@brief BLE can accept to kinds of addresses, either public or random addresses
*/
typedef enum
{
	/** a public static address */
	BLE_ADDRESS_PUBLIC,
	/** a random static address */
	BLE_ADDRESS_RANDOM_STATIC,
	/** resolvable private random address */
	BLE_ADDRESS_RANDOM_PRIVATE_RESOLVABLE,
	/** non-resolvable private random address */
	BLE_ADDRESS_RANDOM_PRIVATE_NON_RESOLVABLE ,

} ble_addr_type_t;

/**@brief Event Types
*/
typedef enum
{
	GAP_EVENT_HANDLERS,
	GATT_SERVER_EVENT_HANDLERS,
	GATT_CLIENT_EVENT_HANDLERS,
	COMMON_EVENT_HANDLERS,
	TRANSPARENT_EVENT_HANDLERS
}ble_event_types_t;

/**@brief BLEDK3 events
*/
enum
{
	PASSKEY_ENTRY_REQ			= 0x60,
	PAIRING_COMPLETE,
	PASSKEY_CONFIRM_REQ,
	ADV_REPORT					= 0x70,
	CONNECTION_COMPLETE,
	DISCONNECTION_COMPLETE,
	CONN_PARAM_UPDATE_NOTIFY,
	COMMAND_COMPLETE			= 0x80,
	STATUS_REPORT,
	CONFIG_MODE_STATUS,
	DISCOVER_ALL_PRIMARY_SERVICE_RESP = 0x90,
	DISCOVER_SPECIFIC_PRIMARY_SERVICE_CHAR_RESP,
	DISCOVER_ALL_CHAR_DESCRIPTOR_RESP,
	CHAR_VALUE_RECEIVED,
	CLIENT_WRITE_CHAR_VALUE		= 0x98,
	RECEIVED_TRANSPARENT_DATA	= 0x9A,
};

/**@brief BLEDK3 status
*/
typedef enum
{
	SCANNING_MODE = 0x01,
	CONNECTING_MODE,
	STANDBY_MODE,
	BROADCAST_MODE = 0x05,
	TRANSPARENT_SERVICE_ENABLED_MODE = 0x08,
	IDLE_MODE,
	SHUTDOWN_MODE,
	CONFIGURE_MODE,
	BLE_CONNECTED_MODE,
}bledk3_status_t;

/**@brief BLEDK3 status
*/
typedef enum
{
	BM7X_PIN_RESET,
	BM7X_PIN_MODE,
	BM7X_PIN_TX_IND,
	BM7X_PIN_RX_IND,
} gpio_pin_t;

typedef enum
{
	GPIO_LOW,
	GPIO_HIGH
} gpio_status_t;


/**@brief Scan types used at @ref ble_scan_param_set
*/
typedef enum
{
	/* No SCAN_REQ packets shall be sent */
	BLE_SCAN_PASSIVE,
	/* SCAN_REQ packets may be sent */
	BLE_SCAN_ACTIVE
}ble_scan_type_t;

/**@brief Scan enable used at @ref ble_scan_start
*/
typedef enum
{
	BLE_SCAN_DISABLED,
	BLE_SCAN_ENABLED,
}ble_scan_enable_t;

/**@brief Scan filter duplicate used at @ref ble_scan_start
*/
typedef enum
{
	BLE_SCAN_DUPLICATE_FILTER_DISABLED,
	BLE_SCAN_DUPLICATE_FILTER_ENABLED,
}ble_scan_duplicate_filter_t;

/**@brief Connection filter used at @ref ble_create_connection
*/
typedef enum
{
	BLE_CONN_WHITELIST_FILTER_DISABLED,
	BLE_CONN_WHITELIST_FILTER_ENABLED,
}ble_connection_filter_t;

/**@brief BLEDK3 advertisement event type used at @ref ble_adv_report_event_t
*/
typedef enum
{
	ADV_IND,
	ADV_DIRECT_IND,
	ADV_SCAN_IND,
	ADV_NONCONN_IND,
	SCAN_RESP,
}ble_adv_event_type_t;

typedef enum
{
	CONFIG_MODE_DISABLED,
	CONFIG_MODE_ENABLED,
}ble_config_mode_status_t;

typedef enum
{
	BLE_UUID_16B = 1,
	BLE_UUID_128B = 4,
	BLE_UUID_INVALID
} ble_uuid_type_t;

/**@brief GATT UUIDs defined by SIG
*/
typedef enum
{
/**< Primary Service Declaration.*/
UUID_PRIMARY_SERVICE = 0x2800,
/**< Secondary Service Declaration.*/
UUID_SECONDARY_SERVICE,
/**< Include Declaration.*/
UUID_INCLUDE,
/**< Characteristic Declaration.*/
UUID_CHARACTERISTIC
}ble_public_uuids_t;

/**@brief GATT Descriptor UUID defined by SIG
*/
typedef enum
{
	CHAR_EXTENDED_PROPERTIES = 0x2900,
	CHAR_USER_DESCRIPTION,
	CLIENT_CHAR_CONFIGURATION,
	SERVER_CHAR_CONFIGURATION,
	CHAR_PRESENTATION_FORMAT,
	CHAR_AGGREGATE_FORMAT,
	VALID_RANGE,
	EXTERNAL_REPORT_REFERENCE,
	REPORT_REFERENCE,
	NUMBER_OF_DIGITALS,
	VALUE_TRIGGER_SETTING,
	ENVIRONMENTAL_SENSING_CONFIGURATION,
	ENVIRONMENTAL_SENSING_MEASUREMENT,
	ENVIRONMENTAL_SENSING_TRIGGER_SETTING,
	TIME_TRIGGER_SETTING
}gatt_descriptors_uuid_t;

/**@brief Characteristic presentation format used at @ref ble_char_presentation_format_t
*/
typedef enum
{
	BLE_PRES_FORMAT_BOOLEAN = 0x01,
	BLE_PRES_FORMAT_2BIT = 0x02,
	BLE_PRES_FORMAT_NIBBLE = 0x03,
	BLE_PRES_FORMAT_UINT8 = 0x04,
	BLE_PRES_FORMAT_UINT12 = 0x05,
	BLE_PRES_FORMAT_UINT16 = 0x06,
	BLE_PRES_FORMAT_UINT24 = 0x07,
	BLE_PRES_FORMAT_UINT32 = 0x08,
	BLE_PRES_FORMAT_UINT48 = 0x09,
	BLE_PRES_FORMAT_UINT64 = 0x0A,
	BLE_PRES_FORMAT_UINT128 = 0x0B,
	BLE_PRES_FORMAT_SINT8 = 0x0C,
	BLE_PRES_FORMAT_SINT12 = 0x0D,
	BLE_PRES_FORMAT_SINT16 = 0x0E,
	BLE_PRES_FORMAT_SINT24 = 0x0F,
	BLE_PRES_FORMAT_SINT32 = 0x10,
	BLE_PRES_FORMAT_SINT48 = 0x11,
	BLE_PRES_FORMAT_SINT64 = 0x12,
	BLE_PRES_FORMAT_SINT128 = 0x13,
	BLE_PRES_FORMAT_FLOAT32 = 0x14,
	BLE_PRES_FORMAT_FLOAT64 = 0x15,
	BLE_PRES_FORMAT_SFLOAT = 0x16,
	BLE_PRES_FORMAT_FLOAT = 0x17,
	BLE_PRES_FORMAT_DUINT16 = 0x18,
	BLE_PRES_FORMAT_UTF8S = 0x19,
	BLE_PRES_FORMAT_UTF16S = 0x1A,
	BLE_PRES_FORMAT_STRUCT = 0x1B,
}ble_char_pres_format_t;

/**@brief Characteristic extended properties used at @ref ble_char_ext_properties_t
*/
typedef enum
{
	BLE_EXT_PROP_RELIABLE_WRITE = 0x0001,
	BLE_EXT_PROP_WRITABLE_AUX,
}ble_char_ext_prop_t;

/**@brief Service type used at @ref ble_service_t
*/
typedef enum
{
	SECONDARY_SERVICE,
	PRIMARY_SERVICE
}ble_service_type_t;

/**@brief UART receive asynchronous callback type. */
typedef void (*uart_recv_async_cb_t)(uint16_t);
/**@brief UART write synchronous callback type. */
typedef void (*uart_write_sync_cb_t)(uint8_t *, uint32_t);
/**@brief UART read asynchronous callback type. */
typedef void (*uart_read_async_cb_t) (void (*)(uint16_t));
/**@brief Create timer callback type. */
typedef void *(*create_timer_cb_t)(void (*)(void *));
/**@brief Delete timer callback type. */
typedef void (*delete_timer_cb_t)(void *);
/**@brief Start timer callback type. */
typedef void (*start_timer_cb_t)(void *, uint32_t);
/**@brief Stop timer callback type. */
typedef void (*stop_timer_cb_t)(void *);
/**@brief Sleep timer callback type. */
typedef void (*sleep_timer_cb_t)(uint32_t);
/**@brief GPIO set callback type. */
typedef void (*gpio_set_cb_t)(gpio_pin_t, gpio_status_t);
/**@brief Mode set callback type. */
typedef void (*mode_set_cb_t)(BM_MODE);
/**@brief Mode get callback type. */
typedef void (*mode_get_cb_t)(BM_MODE*);
/****************************************************************************************
*                                   Structures                                          *
****************************************************************************************/
typedef struct
{
	uint16_t frame_length;
	uint8_t  event_id;
	uint8_t  palyload[800];
	uint8_t  crc;
}bm_cmd_frame_t;

/**@brief Blue-tooth Low Energy address Type
*/
typedef struct PACKED
{
	BM_ADV_ADDRESS type;          /**< See @ref at_ble_addr_type_t */
	uint8_t addr[BLE_ADDR_LEN];    /**< 48-bit address, LSB format. */
} ble_addr_t, ble_rand_addr_changed_t;

/**@brief BLEDK3 Enable TransparentUART command parameters
*/
typedef struct PACKED
{
	uint8_t conn_handle;
	uint8_t server_transparent_cntrl;
	uint8_t client_transparent_mode;
}ble_enable_transparent_uart_t;

/**@brief BLEDK3 Set connection parameter command parameters
*/
typedef struct PACKED
{
	/* Minimum time between two connection events. Range is from 0x0006 to 0x0C80.
		This should be less than or equal to max_conn_interval */
	uint16_t min_conn_interval;
	/* Maximum time between two connection events. Range is from 0x0006 to 0x0C80. 
		This should be greater than or equal to min_conn_interval */
	uint16_t max_conn_interval;
	/* Slave latency for the connection in number of connection events. Range is from 0x0000 to 0x01F3 */
	uint16_t conn_latency;
	/* LE link supervision timeout. Range is from 0x000A to 0x0C80 */
	uint16_t link_sv_to;
}ble_set_conn_param_t;

/**@brief BLE Connection parameters
*/
typedef struct PACKED
{
	/* Time between two connection events. Range is from 0x0006 to 0x0C80 */
	uint16_t conn_interval;
	/* Slave latency for the connection in number of connection events. Range is from 0x0000 to 0x01F3 */
	uint16_t conn_latency;
	/* LE link supervision timeout. Range is from 0x000A to 0x0C80 */
	uint16_t link_sv_to;
}ble_conn_param_t;

/**@brief BLEDK3 Connection complete event parameters
*/
typedef struct PACKED
{
	uint8_t status;
	uint8_t conn_handle;
	uint8_t role;
	ble_addr_t peer_addr;
	ble_conn_param_t conn_param;
}ble_conn_complete_event_t;

/**@brief BLEDK3 Advertisement report event parameters
*/
typedef struct PACKED
{
	ble_adv_event_type_t adv_event_type;
	ble_addr_t addr;
	int8_t rssi;
	uint8_t data_len;
	uint8_t data[31];
}ble_adv_report_event_t;

/**@brief BLEDK3 Disconnect complete event parameters
*/
typedef struct PACKED
{
	uint8_t conn_handle;
	uint8_t reason;
}ble_disconnect_complete_event_t;

/**@brief BLEDK3 Connection parameter update event parameters
*/
typedef struct PACKED
{
	uint8_t conn_handle;
	ble_conn_param_t conn_param;
}ble_conn_param_update_event_t;

/**@brief BLEDK3 Command complete event parameters
*/
typedef struct PACKED
{
	uint8_t cmd_id;
	ble_status_t status;
	uint8_t data_len;
	uint8_t *data;
}ble_cmd_complete_event_t;

/**@brief BLEDK3 Status report event parameters
*/
typedef struct
{
	bledk3_status_t status;
}ble_status_report_event_t;

/**@brief BLEDK3 BLE end test result event parameters
*/
typedef struct
{
	uint16_t num_of_packets;
}ble_end_test_result_event_t;

/**@brief BLEDK3 BLE Config mode status event parameters
*/
typedef struct
{
	ble_config_mode_status_t status;
}ble_config_mode_status_event_t;

/**@brief UUID value
*/
typedef union
{
	uint8_t uuid_16b[BLE_UUID_16B_LEN];
	uint8_t uuid_128b[BLE_UUID_128B_LEN];
} ble_uuid_val_t;

/**@brief UUID
*/
typedef struct
{
	uint8_t type;
	ble_uuid_val_t uuid;
} ble_uuid_t;

/**@brief BLEDK3 Write characteristic value event parameters
*/
typedef struct PACKED
{
	uint8_t conn_handle;
	uint16_t char_value_handle;
	uint8_t char_value[MAX_CHAR_WRITE_VALUE];
}ble_write_char_value_event_t;

/**@brief BLEDK3 Service attribute data parameters used at @ref ble_primary_service_discover_all_event_t
*/
typedef struct PACKED
{
    uint16_t group_handle_start;
    uint16_t group_handle_end;
    ble_uuid_val_t service_uuid;
}ble_service_attribute_data;

/**@brief BLEDK3 Discover all primary service event parameters
*/
typedef struct
{
    uint8_t conn_handle;
    uint8_t attribute_length;
    ble_service_attribute_data attribute_data[];
}ble_primary_service_discover_all_event_t;

/**@brief BLEDK3 Characteristic attribute data parameters used at @ref ble_primary_service_characteristics_discover_event_t
*/
typedef struct PACKED
{
    uint16_t char_attribute_handle;
    uint8_t char_property;
    uint16_t char_value_handle;
    ble_uuid_val_t char_uuid;
}ble_characteristic_attribute_data;

/**@brief BLEDK3 Discover primary service/characteristic event parameters
*/
typedef struct
{
    uint8_t conn_handle;
    uint8_t char_attribute_length;
    ble_characteristic_attribute_data attribute_data[];
}ble_primary_service_characteristics_discover_event_t;

/**@brief BLEDK3 Descriptor attribute data parameters used at @ref ble_characteristic_descriptors_discover_event_t
*/
typedef struct
{
    uint16_t desc_handle;
    ble_uuid_val_t desc_uuid;
}ble_descriptor_attribute_data;

/**@brief BLEDK3 Discover all characteristic descriptors event parameters
*/
typedef struct
{
    uint8_t conn_handle;
    uint8_t desc_format;
    ble_descriptor_attribute_data attribute_data[];
}ble_characteristic_descriptors_discover_event_t;

/**@brief BLEDK3 Characteristic value received event parameters
*/
typedef struct  PACKED
{
    uint8_t conn_handle;
    uint16_t char_value_handle;
    uint8_t char_value_data[];
}ble_characteristic_value_received_event_t, ble_characteristic_value_write_event_t;

/**@brief BLEDK3 TransparentUART data received event parameters
*/
typedef struct  PACKED
{
    uint8_t conn_handle;
    uint8_t data[];
}ble_transparent_uart_data_received_event_t, ble_transparent_uart_data_write_event_t;

/**@brief Platform APIs registration list used at @ref platform_init_t
*/
typedef struct
{
    /* Pointer to function that creates new one-shot timer. 
		@return timer handle */
    create_timer_cb_t create_timer;
    /* Pointer to function that deletes the one-shot timer which was created earlier.
		@param timer handle */
    delete_timer_cb_t delete_timer;
    /* Pointer to function that starts the one-shot timer which was created earlier.
       @param timer handle
       @param time in milliseconds */
    start_timer_cb_t start_timer;
    /* Pointer to function that stops the one-shot timer which was started earlier.
       @param timer handle */
    stop_timer_cb_t stop_timer;
    /* Pointer to function that enables sleep mode for specified amount of time.
		@param duration in millisecond */
    sleep_timer_cb_t sleep_timer_ms;
    /* Pointer to function that set the specified GPIO in specified state.
		@param gpio_pin, the valid pins are RST, MODE, TX_IND and RX_IND
		@param gpio_status */
    gpio_set_cb_t gpio_set;
    /* Pointer to function that set the specified mode.
		@param mode, operation mode */
    mode_set_cb_t mode_set;
    /* Pointer to function that set the specified mode.
		@param mode, operation mode */
    mode_get_cb_t mode_get;
    /* Pointer to function that sends uart data synchronously to BM7x.
       The function should block until transmission done */
    uart_write_sync_cb_t uart_tx_cb;
    /* Pointer to function that receives asynchronously from BM7xto.
       The function should return immediately */
    uart_read_async_cb_t uart_rx_cb;
} ble_platform_api_list_t;

/**@brief Event memory init used at @ref platform_init_t
*/
typedef struct
{
	event_t *event_mem_pool;
	uint32_t event_mem_pool_size;
	uint32_t *event_payload_mem;
	uint32_t event_payload_mem_size;
}event_mem_t;

/**@brief Platform initialization, includes event memory init and platform APIs registration
*/
typedef struct 
{
	event_mem_t event_mem;
	ble_platform_api_list_t platform_api_list;
}platform_init_t;


typedef uint16_t ble_handle_t;
typedef uint8_t ble_attr_permissions_t;
typedef uint8_t ble_char_properties_t;

/**@brief Discover service, characteristic response event parameters
*/
typedef struct  PACKED
{
	uint8_t conn_handle;
	uint8_t length;
	uint8_t attrib_data[];
}primary_service_char_discovery_resp_t;

/**@brief Discover descriptor response event parameters
*/
typedef struct  
{
	uint8_t conn_handle;
	uint8_t format;
	uint8_t desc_attrib_data[];
}char_desc_discovery_resp_t;

/**
 * @brief The service attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Attribute start handle */
	ble_handle_t start_handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 2 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_2];
	/**< Attribute end handle */
	ble_handle_t end_handle;
	/**< Service UUID */
	ble_uuid_val_t uuid;
}attrib_service_t;

/**
 * @brief The characteristic attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Attribute start handle */
	ble_handle_t start_handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 2 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_2];
	/**< Characteristic properties */
	ble_char_properties_t properties;
	/**< Attribute value handle */
	ble_handle_t value_handle;
	/**< Service UUID */
	ble_uuid_val_t uuid;
}attrib_char_t;

/**
 * @brief The characteristic value attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Attribute start handle */
	ble_handle_t start_handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 2 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_16];
	/**< Characteristic value length */
	uint8_t value_length;
	/**< Characteristic value */
	uint8_t value[BLE_ATT_ATTRIBUTE_VALUE_LEN];
}attrib_char_value_t;

/**
 * @brief The Client Characteristic Configuration Descriptor attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Attribute start handle */
	ble_handle_t start_handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 2 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_2];
	/**< Client/Server Characteristic Configuration Descriptor */
	uint8_t ccd_value[2];
}attrib_ccd_desc_t;

/**
 * @brief The User descriptor attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Attribute start handle */
	ble_handle_t start_handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 2 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_2];
	/**< User descriptor length */
	uint8_t ud_length;
	/**< User descriptor */
	uint8_t user_desc[BLE_ATT_ATTRIBUTE_VALUE_LEN];
}attrib_user_desc_t;

/**
 * @brief The Extended property descriptor attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Attribute start handle */
	ble_handle_t start_handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 2 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_2];
	/**< Extended property descriptor */
	uint8_t extend_property[2];
}attrib_ext_prop_desc_t;

/**
 * @brief The Presentation format descriptor attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Attribute start handle */
	ble_handle_t start_handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 2 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_2];
	/**< Format */
	uint8_t format;
	/**< Exponent */
	uint8_t exponent;
	/**< Unit */
	uint16_t unit;
	/**< Description */
	uint16_t description;
	/**< Namespace */
	uint8_t name_space;
}attrib_pres_format_desc_t;

/**
 * @brief The Presentation format descriptor attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Attribute start handle */
	ble_handle_t start_handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 2 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_2];
	/**< Number of attribute handles */
	uint8_t num_of_handles;
	/**< Attribute handles list */
	uint16_t *handles_list;
}attrib_aggr_format_desc_t;

/**
 * @brief The public descriptor attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Here the stack will store the handle for future use */
	ble_handle_t handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 2 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_2];
	/**<  descriptor value length */
	uint16_t desc_val_length;
	/**< descriptor value */
	uint8_t desc_value[BLE_ATT_ATTRIBUTE_VALUE_LEN];
}ble_generic_att_public_desc_t;

/**
 * @brief The private descriptor attribute definition.
 * This will be used as part of GATT-Service Create request
 */
typedef struct PACKED
{
	/**< Length of the whole attribute. Including Header and Value */
	uint8_t	length;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Here the stack will store the handle for future use */
	ble_handle_t handle;
	/**< Length of the attribute type */
	uint8_t	type_length;
	/**< The 16 bytes attribute type (UUID) */
	uint8_t	type[BLE_ATTRIB_UUID_LENGTH_16];
	/**<  descriptor value length */
	uint16_t desc_val_length;
	/*< descriptor value*/
	uint8_t desc_value[BLE_ATT_ATTRIBUTE_VALUE_LEN];
}ble_generic_att_private_desc_t;

/**
 * @brief The client characteristic configuration descriptor 
 * attribute definition, used at @ref ble_char_t
 */
typedef struct
{
	/**< Here the stack will store the handle for future use */
	ble_handle_t handle;
	/**< characteristic configuration value */
	uint16_t ccd_value;
	/**<  descriptor permissions */
	ble_attr_permissions_t perm;
	/**< set this flag, if CCD used. Reset otherwise. */
	bool ccd_included;
}ble_client_char_config_desc_t;

/**
 * @brief The server characteristic configuration descriptor
 * attribute definition, used at @ref ble_char_t
 */
typedef ble_client_char_config_desc_t ble_server_char_config_desc_t;

/**
 * @brief The user descriptor attribute definition, used at @ref ble_char_t
 */
typedef struct PACKED
{
	/**< user descriptor handle */
	ble_handle_t handle;
	/**< the user friendly description length, this value will be stored in the relevant descriptor, if no user description is desired set to 0*/
	uint16_t len;
	/**< a user friendly description, this value will be stored in the relevant descriptor, if no user description is desired set to NULL */
	uint8_t *user_description;
	/**< user descriptor permissions */
	ble_attr_permissions_t permissions;
	/**< set this flag, if user descriptor used. Reset otherwise. */
	bool ud_included;
}ble_user_desc_t;

/**
 * @brief The characteristic presentation format descriptor
 * attribute definition, used at @ref ble_char_t
 */
typedef struct PACKED
{
	/**< presentation format handle */
	ble_handle_t handle;
	/**< value format */
	ble_char_pres_format_t format;
	/**< value exponent */
	int8_t exponent;
	/**<  as defined in GATT spec Part G, Section 3.3.3.5.4 */
	uint16_t unit;
	/**<  as defined in GATT spec Part G, Section 3.3.3.5.6 */
	uint16_t description;
	/**<  as defined in GATT spec Part G, Section 3.3.3.5.5 */
	uint8_t name_space;
	/**< presentation format permissions */
	ble_attr_permissions_t permissions;
	/**< set this flag, if presentation format used. Reset otherwise. */
	bool pf_included;
}ble_char_presentation_format_t;

/**
 * @brief The characteristic extended properties descriptor
 * attribute definition, used at @ref ble_char_t
 */
typedef struct PACKED
{
	/**< presentation format handle */
	ble_handle_t handle;
	/**< value format */
	ble_char_ext_prop_t ext_property;
	/**< presentation format permissions */
	ble_attr_permissions_t permissions;
	/**< set this flag, if extended properties used. Reset otherwise. */
	bool ep_included;
}ble_char_ext_properties_t;

/**
 * @brief The generic descriptor attribute definition, used at @ref ble_char_t
 */
typedef struct PACKED
{
	/**< Descriptor handle */
	ble_handle_t handle;
	/**< The attribute permission */
	ble_attr_permissions_t	permission;
	/**< Descriptor UUID */
	ble_uuid_t desc_uuid;
	/**<  Descriptor value length */
	uint16_t desc_val_length;
	/*< Descriptor value*/
	uint8_t desc_value[BLE_ATT_ATTRIBUTE_VALUE_LEN];
}ble_generic_desc_t;

/**
 * @brief The characteristic value attribute definition, used at @ref ble_char_t
 */
typedef struct PACKED
{
	/**< Characteristic handle */
	ble_handle_t char_handle;
	/**< Characteristic properties, values for Client Characteristic Configuration Descriptor and Server Characteristic Configuration Descriptor will be decided from this value*/
	ble_char_properties_t properties;
	/**< Value permissions */
	ble_attr_permissions_t permissions;
	/**< Characteristic value handle */
	ble_handle_t value_handle;
	/**< Characteristic UUID */
	ble_uuid_t uuid;
	/**<  initial value length */
	uint8_t len;
	/**< maximum possible length of the char. value */
	uint16_t max_len;
	/**< initial value of this characteristic  */
	uint8_t *init_value;
}ble_char_val_t;

/**
 * @brief The characteristic attribute definition, used at @ref ble_service_t
 */
typedef struct PACKED
{
	/**< characteristics value related info */
	ble_char_val_t char_val;
	/**< client config descriptor related info */
	ble_client_char_config_desc_t  client_config_desc;
	/**< server config descriptor related info */
	ble_server_char_config_desc_t  server_config_desc;
	/**< user descriptor related info */
	ble_user_desc_t user_desc;
	/**< Characteristic presentation format, if no presentation format is necessary then reset pf_included flag */
	ble_char_presentation_format_t presentation_format;
	/**< Characteristic extended properties, if no extended properties is necessary then reset ep_included flag */
	ble_char_ext_properties_t ext_properties;
	/**< Number of generic descriptors included in list */
	uint16_t additional_desc_count;
	/**< generic descriptor list */
	ble_generic_desc_t    *additional_desc_list;
}ble_char_t;

/**
 * @brief The service attribute definition
 */
typedef struct PACKED
{
	/**< Service type (PRIMARY or SECONDARY )*/
	ble_service_type_t type;
	/**< Service permissions*/
	ble_attr_permissions_t perm;
	/**< Here the stack will store the handle for future use */
	ble_handle_t handle;
	/**< UUID of the  Service*/
	ble_uuid_t uuid;
	/**< Characteristics count*/
	uint16_t char_count;
	/**< Characteristics list*/
	ble_char_t *char_list;
}ble_service_t;

/****************************************************************************************
*											APIs	                                    *
****************************************************************************************/
/*
 * @brief Resets BM7x module
 *
 * @param pointer to platform_init which contains platform API list and event memory
 * @param mode operation mode
 *
 * @return status of BLE initialization
*/
ble_status_t ble_init(platform_init_t *platform_init);

/*
 * @brief Set an advertisement data on BM7x
 *
 * @param[in] adv_data advertisement data
 * @param[in] adv_data_len advertisement data length
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_adv_data_set(uint8_t const *adv_data, uint8_t adv_data_len);

/*
 * @brief Set scan response data on BM7x
 *
 * @param[in] scan_data scan response data
 * @param[in] scan_data_len scan response data length
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_scan_resp_data_set(uint8_t const *scan_data, uint8_t scan_data_len);

/*
 * @brief Set an advertisement parameters on BM7x
 *
 * @param[in] type advertisement type @ref BM_ADV_TYPE
 * @param[in] peer_addr peer device address and its type. Only applicable in direct advertising mode
 * @param[in] interval advertisment interval. each unit is .625 ms
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_adv_param_set(BM_ADV_TYPE type, ble_addr_t *peer_addr, uint16_t interval);

/* @brief Start an advertisement on BM7x
 *
 * @param
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_adv_start(void);

/* @brief Stop an advertisement on BM7x
 *
 * @param
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_adv_stop(void);

/*
 * @brief Set device name for BM7x
 *
 * @param[in] device_name name of the device
 * @param[in] length length of device name
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_set_device_name(const uint8_t *device_name, uint8_t length);

/*
 * @brief Gives RSSI value for peer connection. This API is only valid after connection established with peer device
 *
 * @param[in] conn_handle connection handle
 * @param[out] rssi rssi value of peer connection
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_rssi_read(uint8_t conn_handle, uint8_t *rssi);

/*
 * @brief Set scan parameters
 *
 * @param[in] scan_interval time interval between subsequent scans. The scan interval should be in the range of 0x0004 to 0x4000
 * @param[in] scan_window is a scan duration and it should not be greater than scan_interval
 * @param[in] scan_type controls the type of scan @ref ble_scan_type_t
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_scan_param_set(uint16_t scan_interval, uint16_t scan_window, ble_scan_type_t scan_type);

/*
 * @brief Start scan
 *
 * @param[in] scan_enable enables/disables scan operation
 * @param[in] scan_dup_filter enables/disables scan duplicate filter
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_scan_start(ble_scan_enable_t scan_enable, ble_scan_duplicate_filter_t scan_dup_filter);

/*
 * @brief Set connection parameters
 *
 * @param[in] conn_params provides connection parameter for upcoming connections. @ref ble_set_conn_param_t
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_set_connection_params(ble_set_conn_param_t *conn_params);

/*
 * @brief Create connection
 *
 * @param[in] conn_filter determines the peer device info source. If @ref BLE_CONN_WHITELIST_FILTER_DISABLED is set 
 *				then whitelist is not used select the peer device. If @ref BLE_CONN_WHITELIST_FILTER_ENABLED is set
 *				then whitelist is used to select peer device.
 * @param[in] ble_addr provides peer device address and its type to connect
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_create_connection(ble_connection_filter_t conn_filter, ble_addr_t *address);

/*
 * @brief Cancel ongoing connection
 *
 * @param None
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_cancel_connection(void);

/*
 * @brief Update connection parameters
 *
 * @param[in] conn_handle identifies the connection 
 * @param[in] conn_param @ref ble_conn_param_t
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_update_connection_parameters(uint8_t conn_handle, ble_conn_param_t *conn_param);

/*
 * @brief Disconnect from remote device
 *
 * @param[in] conn_handle connection handle. 
 * 
 * @note Since current version supports single connection, the connection handle is not honored.
 *				In future versions when multi connection is supported, this will be honored.
 *
 * @return 
*/
void ble_disconnect_device(uint8_t conn_handle);

ble_status_t ble_pair_mode_set(BM_PAIR_MODE mode);

ble_status_t ble_pair_mode_get(void);

ble_status_t ble_pair_request(ble_handle_t conn_handle);

ble_status_t ble_pair_passkey_enter(ble_handle_t conn_handle, uint8_t* passkey, uint8_t length);

ble_status_t ble_pair_passkey_reply(ble_handle_t conn_handle);

ble_status_t ble_pair_passkey_erase_digits(ble_handle_t conn_handle, uint8_t length);

ble_status_t ble_pair_passkey_clear(ble_handle_t conn_handle);

ble_status_t ble_pair_passkey_confirm(ble_handle_t conn_handle, ble_pair_confirm_t confirm);

ble_status_t ble_pair_device_erase_all(void);

ble_status_t ble_pair_device_delete(uint8_t index);

/*
 * @brief Read remote device name
 *
 * @param[in] conn_handle connection handle
 * @param[out] name remote device name
 * @param[out] name_length length of remote device name
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
 */
/* ble_status_t ble_remote_device_name_read(uint8_t conn_handle, uint8_t *name, uint8_t *name_length); */
 
/*
 * @brief Enable transparent UART service
 *
 * @param[in] enable_transparent_uart transparent UART parameters @ref ble_enable_transparent_uart_t
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_transparent_uart_enable(ble_enable_transparent_uart_t * enable_transparent_uart);

/*
 * @brief Transparent UART data
 *
 * @param[in] data transparent UART data
 * @param[in] datalen length of transparent UART data
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_transparent_uart_data_send(ble_handle_t conn_handle, uint8_t *data, uint8_t datalen);

/*
 * @brief Register ble event callbacks
 *
 * @param[in] event_type type of event @ref ble_event_types_t
 * @param[in] event_handler callback function
 * 
 * @return 
*/
void register_ble_event_cb_handlers(ble_event_types_t event_type, const void *event_handler);

/*
 * @brief Process BMxx commands and invoke appropriate callbacks
 *
 * @param[in] bm_cmd command/data from BM7x device @ref bm_cmd_frame_t
 *
 * @return
*/
void ble_event_cb_handler(bm_cmd_frame_t *bm_cmd);

/*
 * @brief Reads fifo where data/commands from BMxx stored. It also parse commands and check CRC
 *
 * @param 
 *
 * @return
*/
uint8_t interface_process_fifo_data(void);

/*
 * @brief Create GATT-Service
 *
 * @param[in] GATT-Service parameters
 *
 * @pre 
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_gatt_service_create(ble_service_t *ble_service);

/*
 *  @brief Discovers all the primary services on the GATT server by GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_primary_service_discover_all(ble_handle_t conn_handle);

/*
 *  @brief Discovers all the characteristics of the service specified on the GATT server by GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] service_uuid Handle to the service UUID for which characteristics need to discovered.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_primary_service_characteristics_discover(ble_handle_t conn_handle, ble_uuid_t *service_uuid);

/*
 *  @brief Perform read operation of the characteristic specified by UUID on the GATT server by GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] characteristic_uuid Handle to the characteristic UUID for which read operation is to be performed.
 *  @param[out] data Pointer to the data buffer to copy to the read data.
 *  @param[out] data_len Pointer to the data length variable to copy the length of the read data.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_read_by_uuid(ble_handle_t conn_handle, ble_uuid_t *characteristic_uuid, void *data, uint16_t *data_len);

/*
 *  @brief Perform read operation of the characteristic specified by handle on the GATT server by GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] characteristic_handle Handle to the characteristic for which read operation is to be performed.
 *  @param[out] data Pointer to the data buffer to copy the read data.
 *  @param[out] data_len Pointer to the data length variable to copy the length of the read data.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_read(ble_handle_t conn_handle, ble_handle_t characteristic_handle, void *data, uint16_t *data_len);

/*
 *  @brief Perform write with response operation of the characteristic specified by handle on the GATT server by GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] characteristic_handle Handle to the characteristic for which write operation is to be performed.
 *  @param[in] data Pointer to the data buffer with write data.
 *  @param[in] length Length of the data to be written.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_write_with_response(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t* data, uint16_t length);

/*
 *  @brief Perform write without response operation of the characteristic specified by handle on the GATT server by GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] characteristic_handle Handle to the characteristic for which write operation is to be performed.
 *  @param[in] data Pointer to the data buffer with write data.
 *  @param[in] length Length of the data to be written.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_write_without_respose(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t* data, uint16_t length);

/*
 *  @brief Send prepare write request from the GATT client to the GATT server.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] characteristic_handle Handle of the characteristic value.
 *  @param[in] offset Offset of the first octet to be written.
 *  @param[in] data Pointer to the part of characteristic value data.
 *  @param[in] length Length of the part of characteristic value data.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_reliable_write_request_prepare(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint16_t offset, uint8_t* data, uint16_t length);

/*
 *  @brief Send execute write request from the GATT client to the GATT server.
 *  @param[in] conn_handle Connection handle.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_reliable_write_request_execute(ble_handle_t conn_handle);

/*
 *  @brief Send cancel write request from the GATT client to the GATT server.
 *  @param[in] conn_handle Connection handle.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_reliable_write_request_cancel(ble_handle_t conn_handle);

/*
 *  @brief Send read request from the GATT client to the GATT server.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] characteristic_handle Characteristic handle.
 *  @param[in] offset Offset of characteristic data.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_attribute_read_request(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint16_t offset);

/*
 *  @brief Perform characteristic value send to GATT client by GATT server.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] characteristic_handle Handle to the characteristic for which send operation is to be performed.
 *  @param[in] data Pointer to the data buffer with data.
 *  @param[in] length Length of the data to be sent.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_value_send(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t* data, uint16_t length);

/*
 *  @brief Perform local update of characteristic value send by GATT server.
 *  @param[in] characteristic_handle Handle to the characteristic for which send operation is to be performed.
 *  @param[in] data Pointer to the data buffer with data to be updated.
 *  @param[in] length Length of the data updated.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_value_update(ble_handle_t characteristic_handle, uint8_t* data, uint16_t length);

/*
 *  @brief Perform local read of characteristic value send by GATT server.
 *  @param[in] characteristic_handle Handle to the characteristic for which send operation is to be performed.
 *  @param[out] data Pointer to the data buffer to copy the read data.
 *  @param[in] data_len Handle to the data length variable to copy the length of the read data.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_value_read(ble_handle_t characteristic_handle, void *data, uint16_t *data_len);

/*
 *  @brief Read all local services by the GATT server.
 *  @param[] None.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_primary_service_read_all(void);

/*
 *  @brief Read all chracterictics of the local service by the GATT server.
 *  @param[in] service_uuid Handle to the service UUID for which characteristics are read.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_primary_service_characteristics_read(ble_uuid_t* service_uuid);

/*
 *  @brief Send response from the GATT server to the write operation requested by the GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] attr_handle Attribute handle to send the write response.
 *  @param[in] err_code Code of the response to be sent for the write request.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_write_response_send(ble_handle_t conn_handle, ble_handle_t attr_handle, uint8_t err_code);

/*
 *  @brief Send response from the GATT server to the read operation requested by the GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] rsp_type Type of the read response.
 *  @param[in] data Pointer to the data to be sent for the read request.
 *  @param[in] length Length of the data to be sent for the read request.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_read_response_send(ble_handle_t conn_handle, uint8_t rsp_type, uint8_t* data, uint16_t length);

/*
 *  @brief Send response from the GATT server to the Prepare Write Request by the GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] characteristic_handle Handle of the characteristic value.
 *  @param[in] offset Offset of the first octet to be written.
 *  @param[in] data Pointer to the part of characteristic value data.
 *  @param[in] length Length of the part of characteric value data.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_reliable_write_response_prepare(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint16_t offset, uint8_t* data, uint16_t length);

/*
 *  @brief Send response from the GATT server to the Prepare Write Execute by the GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_reliable_write_response_execute(ble_handle_t conn_handle);

/*
 *  @brief Send response from the GATT server to the Read Blob Request by the GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] data Pointer to the part of characteristic value data.
 *  @param[in] length Length of the part of characteristic value data.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_characteristic_attribute_read_response(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t* data, uint16_t length);

/*
 *  @brief Send error response from the GATT server to the GATT client.
 *  @param[in] conn_handle Connection handle.
 *  @param[in] characteristic_handle Charactertic handle to which the error response is to be sent.
 *  @param[in] req_opcode Opcode of the request for which the error response is to be sent.
 *  @param[in] err_code Error code response to be sent.
 *  @pre The BLE Central must have an active connection with the BLE Peripheral.
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_error_response_send(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t req_opcode, uint8_t err_code);

/*
 *  @brief This configures the GPIOs that can wakeup the device and puts the device into power saving mode.
 *         In the power saving mode, the cpu clock is switched to 32 KHz and only hardware Logic handling wakeup is powered on. On wakeup the device starts out of reset.
 *  @param[] 
 *  @return ble_status_t Status returned for the command issued.
 */
ble_status_t ble_shutdown(void);

#endif /* __BLE_API_H__ */
