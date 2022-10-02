/*******************************************************************************
Copyright (c) RivieraWaves 2009-2014
Copyright (c) 2017 released Microchip Technology Inc. All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
/*
 * gattc_task.h
 *
 */
#ifndef GATTC_TASK_H_
#define GATTC_TASK_H_

#include "cmn_defs.h"


at_ble_status_t gattc_write_cmd_ind_handler(uint8_t *msg, uint16_t len);
at_ble_status_t gattc_send_evt_cmd_handler(uint8_t u8ReqType, uint16_t u16AttHdl, uint16_t u16ConHdl);
at_ble_status_t gattc_disc_cmd_handler (uint8_t u8ReqType, uint8_t u8UUIDLen, uint16_t u16StartHdl,
                             uint16_t u16EndHdl, uint8_t* pu8UUID, uint16_t u16ConHdl);
at_ble_status_t gattc_reg_to_peer_handler(uint8_t u8ReqType, uint16_t u16StartHdl, uint16_t u16EndHdl,
                               uint16_t u16ConHdl);

at_ble_status_t gattc_execute_write_cmd_handler(uint8_t u8Execute, uint16_t u16ConHdl);


at_ble_status_t gattc_write_cmd_handler(uint8_t u8ReqType, uint8_t u8AutoExec, uint16_t u16AttHdl, uint16_t u16Offset,
                             uint16_t u16Length, uint8_t* pu8Val, uint16_t u16ConHdl);

at_ble_status_t gattc_read_cmd_handler_by_uuid(uint16_t u16StartHdl, uint16_t u16EndHdl,
                             uint8_t u8UUIDLen, uint8_t *pu8UUID, uint16_t u16ConHdl);

at_ble_status_t gattc_read_cmd_handler_simple_read(uint16_t u16Handle, uint16_t u16Offset,
                             uint16_t u16Len, uint16_t u16ConHdl);

at_ble_status_t gattc_read_cmd_handler_multible(uint16_t* u16Handle_list, uint8_t u8Count,
                             uint16_t* u16Length_list, uint16_t u16ConHdl);



at_ble_status_t gattc_write_cmd_ind(uint16_t src, uint8_t* data, at_ble_characteristic_changed_t *params);


at_ble_status_t gattc_disc_svc_ind_parser(uint16_t src, uint8_t* data, at_ble_primary_service_found_t* params);


at_ble_status_t gattc_disc_svc_incl_ind_parser(uint16_t src, uint8_t* data, at_ble_included_service_found_t* params);


at_ble_status_t gattc_disc_char_ind_parser(uint16_t src, uint8_t* data, at_ble_characteristic_found_t* params);


at_ble_status_t gattc_disc_char_desc_ind_parser(uint16_t src, uint8_t* data, at_ble_descriptor_found_t* params);


at_ble_status_t gattc_read_ind_parser(uint16_t src, uint8_t* data, at_ble_characteristic_read_response_t* params);

at_ble_status_t gattc_svc_changed_notification_cmd_handler(uint16_t conn_handle, uint16_t start_handle, uint16_t end_handle);

at_ble_events_t gattc_event_ind_parser(uint16_t src, uint8_t* data, void* params);

at_ble_events_t gattc_complete_evt_handler(uint16_t src, uint8_t* data, void* params);

at_ble_status_t gattc_mtu_cmd_handler(uint16_t u16ConHdl);

/// GATT Task messages
enum gattc_msg_id
{
    /* Default event */
    /// Command Complete event
    GATTC_CMP_EVT = 0x3000,

    /* ATTRIBUTE CLIENT */
    /// Server configuration request
    GATTC_EXC_MTU_CMD,

    /*Discover All Services */
    /*Discover Services by Service UUID*/
    /*Find Included Services*/
    /*Discover Characteristics by UUID*/
    /*Discover All Characteristics of a Service*/
    /*Discover All Characteristic Descriptors*/
    /// Discovery command
    GATTC_DISC_CMD,
    /* GATT -> HL: Events to Upper layer */
    /*Discover All Services*/
    /// Discovery services indication
    GATTC_DISC_SVC_IND,
    /*Find Included Services*/
    /// Discover included services indication
    GATTC_DISC_SVC_INCL_IND,
    /*Discover All Characteristics of a Service*/
    /// Discover characteristic indication
    GATTC_DISC_CHAR_IND,
    /*Discover All Characteristic Descriptors*/
    /// Discovery characteristic descriptor indication
    GATTC_DISC_CHAR_DESC_IND,

    /*Read Value*/
    /*Read Using UUID*/
    /*Read Long Value*/
    /*Read Multiple Values*/
    /// Read command
    GATTC_READ_CMD,
    /// Read response
    GATTC_READ_IND,

    /*Write without response*/
    /*Write without response with Authentication*/
    /*Write Characteristic Value*/
    /*Signed Write Characteristic Value*/
    /*Write Long Characteristic Value*/
    /*Characteristic Value Reliable Write*/
    /*Write Characteristic Descriptors*/
    /*Write Long Characteristic Descriptors*/
    /*Characteristic Value Reliable Write*/
    /// Write command request
    GATTC_WRITE_CMD,

    /* Cancel / Execute pending write operations */
    /// Execute write characteristic request
    GATTC_EXECUTE_WRITE_CMD,

    /* Reception of an indication or notification from peer device. */
    /// peer device triggers an event (indication or notification)
    GATTC_EVENT_IND,

    /// Registration to peer device events (Indication/Notification).
    GATTC_REG_TO_PEER_EVT_CMD,

    /* ATTRIBUTE SERVER */
    /*Notify Characteristic*/
    /*Indicate Characteristic*/
    /// send an event to peer device
    GATTC_SEND_EVT_CMD,

    /* Indicate that write operation is requested. */
    /// Write command indicated to upper layers.
    GATTC_WRITE_CMD_IND,

    /* Service Changed Characteristic Indication */
    /**
     * Send a Service Changed indication to a device
     * (message structure is struct gattm_svc_changed_ind_req)
     */
    GATTC_SEND_SVC_CHANGED_CMD,
    /**
     * Inform the application when sending of Service Changed indications has been
     * enabled or disabled
     */
    GATTC_SVC_CHANGED_CFG_IND,

    /* Confirm write command execution. */
    /// Write command confirmation from upper layers.
    GATTC_WRITE_CMD_CFM
};


/// request operation type - application interface
enum gattc_operation
{
    /*              Attribute Client Flags              */
    /* No Operation (if nothing has been requested)     */
    /* ************************************************ */
    /// No operation
    GATTC_NO_OP                                    = 0x00,

    /* Operation flags for MTU Exchange                 */
    /* ************************************************ */
    /// Perform MTU exchange
    GATTC_MTU_EXCH,

    /*      Operation flags for discovery operation     */
    /* ************************************************ */
    /// Discover all services
    GATTC_DISC_ALL_SVC,
    /// Discover services by UUID
    GATTC_DISC_BY_UUID_SVC,
    /// Discover included services
    GATTC_DISC_INCLUDED_SVC,
    /// Discover all characteristics
    GATTC_DISC_ALL_CHAR,
    /// Discover characteristic by UUID
    GATTC_DISC_BY_UUID_CHAR,
    /// Discover characteristic descriptor
    GATTC_DISC_DESC_CHAR,

    /* Operation flags for reading attributes           */
    /* ************************************************ */
    /// Read attribute
    GATTC_READ,
    /// Read long attribute
    GATTC_READ_LONG,
    /// Read attribute by UUID
    GATTC_READ_BY_UUID,
    /// Read multiple attribute
    GATTC_READ_MULTIPLE,

    /* Operation flags for writing/modifying attributes */
    /* ************************************************ */
    /// Write attribute
    GATTC_WRITE,
    /// Write no response
    GATTC_WRITE_NO_RESPONSE,
    /// Write signed
    GATTC_WRITE_SIGNED,
    /// Execute write
    GATTC_EXEC_WRITE,

    /* Operation flags for registering to peer device   */
    /* events                                           */
    /* ************************************************ */
    /// Register to peer device events
    GATTC_REGISTER,
    /// Unregister from peer device events
    GATTC_UNREGISTER,

    /* Operation flags for sending events to peer device*/
    /* ************************************************ */
    /// Send an attribute notification
    GATTC_NOTIFY,
    /// Send an attribute indication
    GATTC_INDICATE,
    /// Send a service changed indication
    GATTC_SVC_CHANGED,

    /// Last GATT flag
    GATTC_LAST
};

/// Attribute Error Code from BT 4.0 specs
typedef enum {
    /// 0x00: No error
    ATT_ERR_NO_ERROR            = 0x00,
    /// 0x01: Handle is invalid
    ATT_ERR_INVALID_HANDLE,
    /// 0x02: Read permission disabled
    ATT_ERR_READ_NOT_PERMITTED,
    /// 0x03: Write permission disabled
    ATT_ERR_WRITE_NOT_PERMITTED,
    /// 0x04: Incorrect PDU
    ATT_ERR_INVALID_PDU,
    /// 0x05: Authentication privilege not enough
    ATT_ERR_INSUFF_AUTHEN,
    /// 0x06: Request not supported or not understood
    ATT_ERR_REQUEST_NOT_SUPPORTED,
    /// 0x07: Incorrect offset value
    ATT_ERR_INVALID_OFFSET,
    /// 0x08: Authorization privilege not enough
    ATT_ERR_INSUFF_AUTHOR,
    /// 0x09: Capacity queue for reliable write reached
    ATT_ERR_PREPARE_QUEUE_FULL,
    /// 0x0A: Attribute requested not existing
    ATT_ERR_ATTRIBUTE_NOT_FOUND,
    /// 0x0B: Attribute requested not long
    ATT_ERR_ATTRIBUTE_NOT_LONG,
    /// 0x0C: Encryption size not sufficient
    ATT_ERR_INSUFF_ENC_KEY_SIZE,
    /// 0x0D: Invalid length of the attribute value
    ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN,
    /// 0x0E: Operation not fit to condition
    ATT_ERR_UNLIKELY_ERR,
    /// 0x0F: Attribute requires encryption before operation
    ATT_ERR_INSUFF_ENC,
    /// 0x10: Attribute grouping not supported
    ATT_ERR_UNSUPP_GRP_TYPE,
    /// 0x11: Resources not sufficient to complete the request
    ATT_ERR_INSUFF_RESOURCE,
    /// 0x80: Application Error
    ATT_ERR_APP_ERROR = 0x80,
    /// Maximum attribute defined errors
    ATT_ERR_LAST
} ATT_ERR;
#endif
