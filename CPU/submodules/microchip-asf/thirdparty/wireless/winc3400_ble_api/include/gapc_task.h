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
 * gapc_task.h
 *
 */
#ifndef GAPC_TASK_H_
#define GAPC_TASK_H_

#include "cmn_defs.h"

typedef struct
{
    /// RSSI value
    uint8_t rssi;
}gapc_con_rssi_ind;

typedef struct
{
    /// Indication conveying the maximum PA gain allowed
    uint8_t max_PA_gain;
} gapc_con_max_PA_gain_get_ind;

typedef struct
{
    /// Indication conveying the transmit power being used
    uint8_t tx_pow_lvl;
} gapc_con_tx_pow_get_ind;

typedef struct
{
    /// Indication conveying the status of the transmit power set request
    uint8_t status;
} gapc_con_tx_pow_set_ind;


at_ble_status_t gapc_connection_cfm_handler(uint8_t *pu8LocalKey,  uint32_t u32LocalSignCntr, uint8_t *pu8RemoteKey,
    uint32_t u32RemoteSignCntr, uint8_t u8Auth, uint8_t u8Authorize, uint16_t u16ConHdl);

at_ble_status_t  gapc_bond_cfm_handler_pair_resp(uint8_t u8Accept, uint8_t u8IoCap, uint8_t u8OOB, uint8_t u8Auth,
    uint8_t u8KeySize, uint8_t u8IkeyDist, uint8_t u8RkeyDist, uint8_t u8SecReq,
    uint16_t u16ConHdl);

at_ble_status_t gapc_get_tx_pwr_req_handler(uint16_t u16ConHdl, uint8_t get_type); //get_type: current or maximum

at_ble_status_t gapc_set_tx_pwr_req_handler(uint16_t u16ConHdl, uint8_t u8txpow);

at_ble_status_t gapc_disconnect_cmd_handler(uint8_t reason, uint16_t handle);

at_ble_status_t gapc_param_update_cmd_handler(uint16_t handle,
    uint16_t con_intv_min, uint16_t con_intv_max, uint16_t con_latency, uint16_t superv_to,
    uint16_t ce_len_min, uint16_t ce_len_max);

at_ble_status_t gapc_param_update_cfm_handler(uint16_t conn_handle,
    uint16_t ce_len_min, uint16_t ce_len_max);

at_ble_status_t gapc_bond_cmd_handler(uint16_t conn_handle, uint8_t io_capabilities,
    uint8_t oob_available, uint8_t auth_req, uint8_t max_key_size,
    uint8_t initiator_keys, uint8_t responder_keys, uint8_t desired_auth);

at_ble_status_t gapc_security_cmd_handler(uint16_t conn_handle, uint8_t auth_req);

at_ble_status_t gapc_bond_cfm_handler_key_exch(uint8_t u8Req,  uint8_t u8Accept, uint8_t* key,
    uint16_t u16ConHdl);

at_ble_status_t gapc_encrypt_cmd_handler(uint16_t conn_handle, uint8_t* key,
    uint16_t ediv, uint8_t* rand, uint8_t key_size , uint8_t auth);

at_ble_status_t gapc_encrypt_cfm_handler(uint16_t conn_handle, uint8_t auth ,uint8_t key_found,
    uint8_t* key, uint8_t key_size);


at_ble_status_t gapc_con_req_ind(uint8_t* data);

at_ble_status_t gapc_disconnect_ind(uint8_t* data, at_ble_disconnected_t* param);

at_ble_status_t gapc_param_updated_ind(uint16_t src, uint8_t* data, at_ble_conn_param_update_done_t* param);

at_ble_status_t gapc_param_update_req_ind(uint16_t src, uint8_t* data,
    at_ble_conn_param_update_request_t *param);

void gapc_bond_req(uint16_t src, at_ble_pair_request_t* param);

void gapc_key_exch(uint8_t u8Req, uint16_t ConHdl);

at_ble_events_t gapc_bond_req_ind(uint16_t src, uint8_t* data, void*param);

at_ble_events_t gapc_bond_ind(uint16_t src, uint8_t* data, at_ble_pair_done_t* param);

at_ble_events_t gapc_cmp_evt(uint16_t src, uint8_t* data,
    at_ble_encryption_status_changed_t* params);

at_ble_status_t gapc_encrypt_req_ind(uint16_t src, uint8_t* data,
    at_ble_encryption_request_t* params);

at_ble_status_t gapc_encrypt_ind(uint16_t src ,uint8_t* data ,at_ble_encryption_status_changed_t* params);

at_ble_status_t gapc_sec_req_ind(uint16_t src, uint8_t* data,at_ble_slave_sec_request_t* params);

at_ble_status_t gapc_get_info_cmd_handler(uint16_t conn_handle, uint8_t operation);

at_ble_status_t gapc_con_rssi_ind_parser(uint16_t src, uint8_t* data, gapc_con_rssi_ind* params);

at_ble_status_t gapc_con_max_PA_gain_get_ind_parser(uint16_t src, uint8_t* data, gapc_con_max_PA_gain_get_ind* params);
at_ble_status_t gapc_con_tx_pow_get_ind_parser(uint16_t src, uint8_t* data, gapc_con_tx_pow_get_ind* params);
at_ble_status_t gapc_con_tx_pow_set_ind_parser(uint16_t src, uint8_t* data, gapc_con_tx_pow_set_ind* params);

enum gapc_msg_id
{
    /* Default event */
    /// Command Complete event
    GAPC_CMP_EVT = 0x3800,

    /* Connection state information */
    /// Indicate that a connection has been established
    GAPC_CONNECTION_REQ_IND,
    /// Set specific link data configuration.
    GAPC_CONNECTION_CFM,

    /// Indicate that a link has been disconnected
    GAPC_DISCONNECT_IND,

    /* Link management command */
    /// Request disconnection of current link command.
    GAPC_DISCONNECT_CMD,

    /* Peer device info */
    /// Retrieve information command
    GAPC_GET_INFO_CMD,
    /// Name of peer device indication
    GAPC_PEER_NAME_IND,
    /// Indication of peer version info
    GAPC_PEER_VERSION_IND,
    /// Indication of peer features info
    GAPC_PEER_FEATURES_IND,

    /// Indication of ongoing connection RSSI
    GAPC_CON_RSSI_IND,
    /// Indication of peer privacy info
    GAPC_PRIVACY_IND,//380A
    /// Indication of peer reconnection address info
    GAPC_RECON_ADDR_IND,

    /* Privacy configuration */
    /// Set Privacy flag command.
    GAPC_SET_PRIVACY_CMD,
    /// Set Reconnection Address Value command.
    GAPC_SET_RECON_ADDR_CMD,

    /* Connection parameters update */
    /// Perform update of connection parameters command
    GAPC_PARAM_UPDATE_CMD,
    /// Request of updating connection parameters indication
    GAPC_PARAM_UPDATE_REQ_IND,
    /// Master confirm or not that parameters proposed by slave are accepted or not
    GAPC_PARAM_UPDATE_CFM,//3810
    /// Connection parameters updated indication
    GAPC_PARAM_UPDATED_IND,

    /* Bonding procedure */
    /// Start Bonding command procedure
    GAPC_BOND_CMD,
    /// Bonding requested by peer device indication message.
    GAPC_BOND_REQ_IND,
    /// Confirm requested bond information.
    GAPC_BOND_CFM,
    /// Bonding information indication message
    GAPC_BOND_IND,

    /* Encryption procedure */
    /// Start Encryption command procedure
    GAPC_ENCRYPT_CMD,
    /// Encryption requested by peer device indication message.
    GAPC_ENCRYPT_REQ_IND,
    /// Confirm requested Encryption information.
    GAPC_ENCRYPT_CFM,
    /// Encryption information indication message
    GAPC_ENCRYPT_IND,

    /* Security request procedure */
    /// Start Security Request command procedure
    GAPC_SECURITY_CMD,//381A
    /// Security requested by peer device indication message
    GAPC_SECURITY_IND,

    /* Signature procedure */
    /// Indicate the current sign counters to the application
    GAPC_SIGN_COUNTER_IND,

    /* Device information */
    /// Indication of ongoing connection Channel Map
    GAPC_CON_CHANNEL_MAP_IND,


    /* Internal messages for timer events, not part of API*/
    /// Parameter update procedure timeout indication
    GAPC_PARAM_UPDATE_TO_IND,
       /// Get Tx Power value
    GAPC_GET_TX_PWR_REQ,
    /// Get Tx Power value complete event
    GAPC_GET_TX_PWR_REQ_CMP_EVT,//3820
     /// SET Tx Power value
    GAPC_SET_TX_PWR_REQ,
    /// Set Tx Power value complete event
    GAPC_SET_TX_PWR_REQ_CMP_EVT,
    /// Indication of tx power get
    GAPC_CON_TX_POW_GET_IND,
    /// Indication of tx power set
    GAPC_CON_TX_POW_SET_IND,
   /// Indication of max PA gain get
    GAPC_CON_MAX_PA_GAIN_GET_IND,
 };


/// request operation type - application interface
enum gapc_operation
{
    /*                 Operation Flags                  */
    /* No Operation (if nothing has been requested)     */
    /* ************************************************ */
    /// No operation
    GAPC_NO_OP                                    = 0x00,

    /* Connection management */
    /// Disconnect link
    GAPC_DISCONNECT,

    /* Connection information */
    /// Retrieve name of peer device.
    GAPC_GET_PEER_NAME,
    /// Retrieve peer device version info.
    GAPC_GET_PEER_VERSION,
    /// Retrieve peer device features.
    GAPC_GET_PEER_FEATURES,
    /// Retrieve connection RSSI.
    GAPC_GET_CON_RSSI,
    /// Retrieve Privacy Info.
    GAPC_GET_PRIVACY,
    /// Retrieve Reconnection Address Value.
    GAPC_GET_RECON_ADDR,

    /* Privacy Configuration */
    /// Set Privacy flag.
    GAPC_SET_PRIVACY,
    /// Set Reconnection Address Value.
    GAPC_SET_RECON_ADDR,

    /* Connection parameters update */
    /// Perform update of connection parameters.
    GAPC_UPDATE_PARAMS,

    /* Security procedures */
    /// Start bonding procedure.
    GAPC_BOND,
    /// Start encryption procedure.
    GAPC_ENCRYPT,
    /// Start security request procedure
    GAPC_SECURITY_REQ,

    /* Connection information */
    /// Retrieve Connection Channel MAP.
    GAPC_GET_CON_CHANNEL_MAP,


    /// Last GAPC operation flag
    GAPC_LAST
};
/// Authentication mask
enum gap_auth_mask
{
    /// No Flag set
    GAP_AUTH_NONE = 0,
    /// Bond authentication
    GAP_AUTH_BOND = (1 << 0),
    /// Man In the middle protection
    GAP_AUTH_MITM = (1 << 2)
};
/// Authentication Requirements
enum gap_auth
{
    /// No MITM No Bonding
    GAP_AUTH_REQ_NO_MITM_NO_BOND = (GAP_AUTH_NONE),
    /// No MITM Bonding
    GAP_AUTH_REQ_NO_MITM_BOND    = (GAP_AUTH_BOND),
    /// MITM No Bonding
    GAP_AUTH_REQ_MITM_NO_BOND    = (GAP_AUTH_MITM),
    /// MITM and Bonding
    GAP_AUTH_REQ_MITM_BOND       = (GAP_AUTH_MITM | GAP_AUTH_BOND),
    GAP_AUTH_REQ_LAST
};


/// Bond event type.
enum gapc_bond
{
    /// Bond Pairing request
    GAPC_PAIRING_REQ,
    /// Respond to Pairing request
    GAPC_PAIRING_RSP,

    /// Pairing Finished information
    GAPC_PAIRING_SUCCEED,
    /// Pairing Failed information
    GAPC_PAIRING_FAILED,

    /// Used to retrieve pairing Temporary Key
    GAPC_TK_EXCH,
    /// Used for Identity Resolving Key exchange
    GAPC_IRK_EXCH,
    /// Used for Connection Signature Resolving Key exchange
    GAPC_CSRK_EXCH,
    /// Used for Long Term Key exchange
    GAPC_LTK_EXCH,

    /// Bond Pairing request issue, Repeated attempt
    GAPC_REPEATED_ATTEMPT
};


/// OOB Data Present Flag Values
enum gap_oob
{
    /// OOB Data not present
    GAP_OOB_AUTH_DATA_NOT_PRESENT = 0x00,
    /// OOB data present
    GAP_OOB_AUTH_DATA_PRESENT,
    GAP_OOB_AUTH_DATA_LAST
};

/// Key Distribution Flags
enum gap_kdist
{
    /// No Keys to distribute
    GAP_KDIST_NONE = 0x00,
    /// Encryption key in distribution
    GAP_KDIST_ENCKEY = (1 << 0),
    /// IRK (ID key)in distribution
    GAP_KDIST_IDKEY  = (1 << 1),
    /// CSRK(Signature key) in distribution
    GAP_KDIST_SIGNKEY= (1 << 2),

    GAP_KDIST_LAST =   (1 << 3)
};

/// Security Defines
enum gap_sec_req
{
    /// No security (no authentication and encryption)
    GAP_NO_SEC = 0x00,
    /// Unauthenticated pairing with encryption
    GAP_SEC1_NOAUTH_PAIR_ENC,
    /// Authenticated pairing with encryption
    GAP_SEC1_AUTH_PAIR_ENC,
    /// Unauthenticated pairing with data signing
    GAP_SEC2_NOAUTH_DATA_SGN,
    /// Authentication pairing with data signing
    GAP_SEC2_AUTH_DATA_SGN,
    /// Unrecognized security
    GAP_SEC_UNDEFINED
};

/// IO Capability Values
enum gap_io_cap
{
    /// Display Only
    GAP_IO_CAP_DISPLAY_ONLY = 0x00,
    /// Display Yes No
    GAP_IO_CAP_DISPLAY_YES_NO,
    /// Keyboard Only
    GAP_IO_CAP_KB_ONLY,
    /// No Input No Output
    GAP_IO_CAP_NO_INPUT_NO_OUTPUT,
    /// Keyboard Display
    GAP_IO_CAP_KB_DISPLAY,
    GAP_IO_CAP_LAST
};

/// TK Type
enum gap_tk_type
{
    ///  TK get from out of band method
    GAP_TK_OOB         = 0x00,
    /// TK generated and shall be displayed by local device
    GAP_TK_DISPLAY,
    /// TK shall be entered by user using device keyboard
    GAP_TK_KEY_ENTRY
};

#endif
