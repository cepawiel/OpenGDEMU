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
 * gapc_task.c
 *
 */
#include "at_ble_api.h"
#include "platform.h"
#include "interface.h"

#include "gapc_task.h"
#include "dbg_task.h"

#include "device.h"

typedef struct
{
    at_ble_handle_t handle;
    at_ble_LTK_t ltk;
    at_ble_CSRK_t csrk;
    at_ble_IRK_t irk;
}at_key_info_t;

static at_key_info_t peer_key_info;

at_ble_status_t gapc_bond_cfm_handler_pair_resp(uint8_t u8Accept, uint8_t u8IoCap, uint8_t u8OOB, uint8_t u8Auth,
                                 uint8_t u8KeySize, uint8_t u8IkeyDist, uint8_t u8RkeyDist, uint8_t u8SecReq,
                                 uint16_t u16ConHdl)
{
	at_ble_status_t status;
    INTERFACE_MSG_INIT(GAPC_BOND_CFM, KE_BUILD_ID(TASK_GAPC, u16ConHdl));
    INTERFACE_PACK_ARG_UINT8(GAPC_PAIRING_RSP);
    INTERFACE_PACK_ARG_UINT8(u8Accept);
    INTERFACE_PACK_ARG_UINT8(u8IoCap);
    INTERFACE_PACK_ARG_UINT8(u8OOB);
    INTERFACE_PACK_ARG_UINT8(u8Auth);
    INTERFACE_PACK_ARG_UINT8(u8KeySize);
    INTERFACE_PACK_ARG_UINT8(u8IkeyDist);
    INTERFACE_PACK_ARG_UINT8(u8RkeyDist);
    INTERFACE_PACK_ARG_UINT8(u8SecReq);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	return status;
}

at_ble_status_t gapc_bond_cfm_handler_key_exch(uint8_t u8Req,  uint8_t u8Accept, uint8_t* key,
                uint16_t u16ConHdl)
{
	at_ble_status_t status;
    INTERFACE_MSG_INIT(GAPC_BOND_CFM, KE_BUILD_ID(TASK_GAPC, u16ConHdl));
    INTERFACE_PACK_ARG_UINT8(u8Req);
    INTERFACE_PACK_ARG_UINT8(u8Accept);
    INTERFACE_PACK_ARG_BLOCK(key, KEY_LEN);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	return status;
}

at_ble_status_t gapc_encrypt_cmd_handler(uint16_t conn_handle, uint8_t* key,
    uint16_t ediv, uint8_t* rand, uint8_t key_size ,uint8_t auth)
{
	at_ble_status_t status;
    static uint8_t dummy_key[KEY_LEN] =
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    INTERFACE_MSG_INIT(GAPC_ENCRYPT_CMD, KE_BUILD_ID(TASK_GAPC, conn_handle));
    INTERFACE_PACK_ARG_UINT8(GAPC_ENCRYPT);
    //Added for structure padding
    INTERFACE_PACK_ARG_UINT8(0);
    INTERFACE_PACK_ARG_BLOCK(key, KEY_LEN);
    INTERFACE_PACK_ARG_UINT16(ediv);
    INTERFACE_PACK_ARG_BLOCK(rand,8);
    INTERFACE_PACK_ARG_UINT8(key_size);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	if (status == AT_BLE_SUCCESS)
	{
		status = gapc_connection_cfm_handler(dummy_key, 0, dummy_key, 0, auth,
                    GAP_AUTHZ_NOT_SET, conn_handle);
	}
	return status;
}

at_ble_status_t gapc_encrypt_cfm_handler(uint16_t conn_handle, uint8_t auth ,uint8_t key_found,
    uint8_t* key, uint8_t key_size)
{
	at_ble_status_t status;
    static uint8_t dummy_key[KEY_LEN] =
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    INTERFACE_MSG_INIT(GAPC_ENCRYPT_CFM, KE_BUILD_ID(TASK_GAPC, conn_handle));
    INTERFACE_PACK_ARG_UINT8(key_found);
    INTERFACE_PACK_ARG_BLOCK(key, KEY_LEN);
    //Encryption key size
    INTERFACE_PACK_ARG_UINT8(key_size);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();

	if (status == AT_BLE_SUCCESS)
	{
		status = gapc_connection_cfm_handler(dummy_key, 0, dummy_key, 0, auth,
						GAP_AUTHZ_NOT_SET, conn_handle);
	}
	return status;
}

at_ble_status_t gapc_get_tx_pwr_req_handler(uint16_t u16ConHdl, uint8_t get_type)
{
	at_ble_status_t status;
    INTERFACE_MSG_INIT(GAPC_GET_TX_PWR_REQ, KE_BUILD_ID(TASK_GAPC, u16ConHdl));
    INTERFACE_PACK_ARG_UINT16(u16ConHdl);
    INTERFACE_PACK_ARG_UINT8(get_type);
	status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	return status;
}

at_ble_status_t gapc_set_tx_pwr_req_handler(uint16_t u16ConHdl, uint8_t u8txpow)
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(GAPC_SET_TX_PWR_REQ, KE_BUILD_ID(TASK_GAPC, u16ConHdl));
    INTERFACE_PACK_ARG_UINT16(u16ConHdl);
    INTERFACE_PACK_ARG_UINT8(u8txpow);
	status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	
	return status;
}

at_ble_status_t gapc_disconnect_cmd_handler(uint8_t reason, uint16_t handle)
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(GAPC_DISCONNECT_CMD, KE_BUILD_ID(TASK_GAPC, handle));
    INTERFACE_PACK_ARG_UINT8(GAPC_DISCONNECT);
    INTERFACE_PACK_ARG_UINT8(reason);
	status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	
	return status;
}

at_ble_status_t gapc_param_update_cmd_handler(uint16_t handle,
    uint16_t con_intv_min, uint16_t con_intv_max, uint16_t con_latency, uint16_t superv_to,
    uint16_t ce_len_min, uint16_t ce_len_max )
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(GAPC_PARAM_UPDATE_CMD, KE_BUILD_ID(TASK_GAPC, handle));
    INTERFACE_PACK_ARG_UINT8(GAPC_UPDATE_PARAMS);
    INTERFACE_PACK_ARG_UINT8(0);
    INTERFACE_PACK_ARG_UINT16(con_intv_min);
    INTERFACE_PACK_ARG_UINT16(con_intv_max);
    INTERFACE_PACK_ARG_UINT16(con_latency);
    INTERFACE_PACK_ARG_UINT16(superv_to);
    INTERFACE_PACK_ARG_UINT16(ce_len_min);
    INTERFACE_PACK_ARG_UINT16(ce_len_max);
	status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	
	return status;
}

at_ble_status_t gapc_param_update_cfm_handler(uint16_t conn_handle,
        uint16_t ce_len_min, uint16_t ce_len_max )
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(GAPC_PARAM_UPDATE_CFM, KE_BUILD_ID(TASK_GAPC, conn_handle));
    INTERFACE_PACK_ARG_UINT8(1);
    INTERFACE_PACK_ARG_UINT16(ce_len_min);
    INTERFACE_PACK_ARG_UINT16(ce_len_max);
	status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	
	return status;
}

at_ble_status_t gapc_connection_cfm_handler(uint8_t *lcsrk,  uint32_t local_signcntr, uint8_t *rcsrk,
                                 uint32_t remote_signcntr, uint8_t auth, uint8_t authorize,
                                 uint16_t handle)
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(GAPC_CONNECTION_CFM,  KE_BUILD_ID(TASK_GAPC, handle));
    INTERFACE_PACK_ARG_BLOCK(lcsrk, KEY_LEN);
    INTERFACE_PACK_ARG_UINT32(local_signcntr);
    INTERFACE_PACK_ARG_BLOCK(rcsrk, KEY_LEN);
    INTERFACE_PACK_ARG_UINT32(remote_signcntr);
    INTERFACE_PACK_ARG_UINT8(auth);
    INTERFACE_PACK_ARG_UINT8(authorize);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	
	return status;
}

at_ble_status_t gapc_bond_cmd_handler(uint16_t conn_handle, uint8_t io_capabilities,
    uint8_t oob_available, uint8_t auth_req, uint8_t max_key_size,
    uint8_t initiator_keys, uint8_t responder_keys, uint8_t desired_auth)
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(GAPC_BOND_CMD, KE_BUILD_ID(TASK_GAPC, conn_handle));
    INTERFACE_PACK_ARG_UINT8(GAPC_BOND);
    INTERFACE_PACK_ARG_UINT8(io_capabilities);
    INTERFACE_PACK_ARG_UINT8(oob_available);
    INTERFACE_PACK_ARG_UINT8(auth_req);
    // To do check max and min key size (not supported)
    INTERFACE_PACK_ARG_UINT8(max_key_size);
    INTERFACE_PACK_ARG_UINT8(initiator_keys);
    INTERFACE_PACK_ARG_UINT8(responder_keys);
    INTERFACE_PACK_ARG_UINT8(desired_auth);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	
	return status;
}

at_ble_status_t gapc_security_cmd_handler(uint16_t conn_handle, uint8_t auth_req)
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(GAPC_SECURITY_CMD, KE_BUILD_ID(TASK_GAPC, conn_handle));
    INTERFACE_PACK_ARG_UINT8(GAPC_SECURITY_REQ);
    INTERFACE_PACK_ARG_UINT8(auth_req);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	
	return status;
}

at_ble_status_t gapc_con_req_ind(uint8_t* data)
{
    static uint8_t dummy_key[KEY_LEN] =
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT16(&(device.conn_handle));
    INTERFACE_UNPACK_SKIP(7); //we are not interested in those params
    INTERFACE_UNPACK_UINT8(&(device.peer_addr.type));
    INTERFACE_UNPACK_BLOCK(device.peer_addr.addr, AT_BLE_ADDR_LEN);
    INTERFACE_UNPACK_DONE();

    return gapc_connection_cfm_handler(dummy_key, 0, dummy_key, 0, GAP_AUTH_REQ_NO_MITM_NO_BOND,
                    GAP_AUTHZ_NOT_SET, device.conn_handle);
}

at_ble_status_t gapc_disconnect_ind(uint8_t* data, at_ble_disconnected_t* param)
{
    device.conn_handle = 0xFFFF;
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT16(&(param->handle));
    INTERFACE_UNPACK_UINT8(&(param->reason));
    INTERFACE_UNPACK_DONE();
	return AT_BLE_SUCCESS;
}

at_ble_status_t gapc_param_updated_ind(uint16_t src, uint8_t* data, at_ble_conn_param_update_done_t* param)
{
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT16(&(param->con_intv));
    INTERFACE_UNPACK_UINT16(&(param->con_latency));
    INTERFACE_UNPACK_UINT16(&(param->superv_to));
    INTERFACE_UNPACK_DONE();
    // To do check add connection parameters or not
    param->handle = KE_IDX_GET(src);
    param->status = AT_BLE_SUCCESS;
	return AT_BLE_SUCCESS;
}

at_ble_status_t gapc_param_update_req_ind(uint16_t src, uint8_t* data,
    at_ble_conn_param_update_request_t *param)
{
    // To do check ce_min and ce_max
    param->handle = KE_IDX_GET(src);
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT16(&(param->params.con_intv_max));
    INTERFACE_UNPACK_UINT16(&(param->params.con_intv_min));
    INTERFACE_UNPACK_UINT16(&(param->params.con_latency));
    INTERFACE_UNPACK_UINT16(&(param->params.superv_to));
    INTERFACE_UNPACK_DONE();
	return AT_BLE_SUCCESS;
}

void gapc_bond_req(uint16_t src, at_ble_pair_request_t* param)
{
    param->handle = KE_IDX_GET(src);
}

void gapc_key_exch(uint8_t u8Req, uint16_t ConHdl)
{
    //if(local_key.handle == ConHdl)
    {
        INTERFACE_MSG_INIT(GAPC_BOND_CFM, KE_BUILD_ID(TASK_GAPC, ConHdl));
        INTERFACE_PACK_ARG_UINT8(u8Req);
        INTERFACE_PACK_ARG_UINT8(1);
        switch(u8Req)
        {
            case GAPC_LTK_EXCH:
                INTERFACE_PACK_ARG_BLOCK(device.ltk.key, AT_BLE_MAX_KEY_LEN);
                INTERFACE_PACK_ARG_UINT16(device.ltk.ediv);
                INTERFACE_PACK_ARG_BLOCK(device.ltk.nb,8);
                INTERFACE_PACK_ARG_UINT8(device.ltk.key_size);
            break;
            case GAPC_CSRK_EXCH:
                INTERFACE_PACK_ARG_BLOCK(device.csrk.key, KEY_LEN);
        }
        INTERFACE_SEND_NO_WAIT();
        INTERFACE_MSG_DONE();
    }
}

at_ble_events_t gapc_bond_req_ind(uint16_t src, uint8_t* data, void*param)
{
    uint8_t u8Req , pair_data;
    at_ble_pair_key_request_t *pair_key_request;
    at_ble_events_t evt_num = AT_BLE_UNDEFINED_EVENT;
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&u8Req);
    INTERFACE_UNPACK_UINT8(&pair_data);
    INTERFACE_UNPACK_DONE();

    switch(u8Req)
    {
        case GAPC_PAIRING_REQ:
        {
            evt_num = AT_BLE_PAIR_REQUEST;
            gapc_bond_req(src, (at_ble_pair_request_t*)param);
        }
        break;
        case GAPC_TK_EXCH:
        {
            evt_num = AT_BLE_PAIR_KEY_REQUEST;
            pair_key_request = (at_ble_pair_key_request_t *)param;
            pair_key_request->handle = KE_IDX_GET(src);
            //To be modified out of bond or TK
            if(pair_data == GAP_TK_OOB)
            {
                pair_key_request->type = AT_BLE_PAIR_OOB;
            }
            else
            {
                pair_key_request->type = AT_BLE_PAIR_PASSKEY;
                if(pair_data == GAP_TK_KEY_ENTRY)
                {
                    pair_key_request->passkey_type = AT_BLE_PAIR_PASSKEY_ENTRY;
                }
                else if(pair_data == GAP_TK_DISPLAY)
                {
                    pair_key_request->passkey_type = AT_BLE_PAIR_PASSKEY_DISPLAY;
                }
            }
        }
        break;
        case GAPC_LTK_EXCH:
        case GAPC_CSRK_EXCH:
            gapc_key_exch(u8Req,KE_IDX_GET(src));
        break;
        default:
        break;
    }
    return evt_num;
}

at_ble_events_t gapc_bond_ind(uint16_t src, uint8_t* data, at_ble_pair_done_t* param)
{
    at_ble_events_t evt_num = AT_BLE_UNDEFINED_EVENT;
    uint8_t u8Info;
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&u8Info);
    INTERFACE_UNPACK_SKIP(1); //padding

    switch(u8Info)
    {
        case GAPC_PAIRING_FAILED:
        {
            evt_num = AT_BLE_PAIR_DONE;
            param->handle = KE_IDX_GET(src);
            /* To do add reason */
            param->status = AT_BLE_FAILURE;
        }
        break;
        case GAPC_PAIRING_SUCCEED:
        {
            evt_num = AT_BLE_PAIR_DONE;
            param->handle = KE_IDX_GET(src);
            param->status = AT_BLE_SUCCESS;
            param->auth = INTERFACE_UNPACK_UINT8(&u8Info);
            if(peer_key_info.irk.key != NULL)
            {
                memcpy(&(param->peer_irk),&(peer_key_info.irk),sizeof(at_ble_IRK_t));
            }
            if(peer_key_info.csrk.key != NULL)
            {
                memcpy(&(param->peer_csrk),&(peer_key_info.csrk),sizeof(at_ble_CSRK_t));
            }
            if(peer_key_info.ltk.key != NULL)
            {
                memcpy(&(param->peer_ltk),&(peer_key_info.ltk),sizeof(at_ble_LTK_t));
            }
        }
        break;
        case GAPC_IRK_EXCH:
        {
            INTERFACE_UNPACK_BLOCK(peer_key_info.irk.key ,KEY_LEN);
            INTERFACE_UNPACK_BLOCK((peer_key_info.irk.addr.addr),AT_BLE_ADDR_LEN);
            INTERFACE_UNPACK_UINT8(&(peer_key_info.irk.addr.type));
        }
        break;
        case GAPC_CSRK_EXCH:
            INTERFACE_UNPACK_BLOCK(peer_key_info.csrk.key ,KEY_LEN);
        break;
        case GAPC_LTK_EXCH:
        {
            INTERFACE_UNPACK_BLOCK(peer_key_info.ltk.key ,KEY_LEN);
            INTERFACE_UNPACK_UINT16(&(peer_key_info.ltk.ediv));
            INTERFACE_UNPACK_BLOCK(peer_key_info.ltk.nb,8);
            INTERFACE_UNPACK_UINT8(&(peer_key_info.ltk.key_size));
        }
        break;
        default:
            break;
    }
    INTERFACE_UNPACK_DONE();
    return evt_num;
}

at_ble_status_t gapc_encrypt_ind(uint16_t src ,uint8_t* data ,at_ble_encryption_status_changed_t* params)
{
    uint8_t auth;

    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&auth);
    INTERFACE_UNPACK_DONE();
    params->handle = KE_IDX_GET(src);
    params->status = AT_BLE_SUCCESS;
	return AT_BLE_SUCCESS;
}

at_ble_events_t gapc_cmp_evt(uint16_t src, uint8_t* data,
    at_ble_encryption_status_changed_t* params)
{
    uint8_t u8Operation,status;
    at_ble_events_t evt_num = AT_BLE_UNDEFINED_EVENT;
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&u8Operation);
    INTERFACE_UNPACK_UINT8(&status);
    INTERFACE_UNPACK_DONE();

    switch(u8Operation)
    {
        case GAPC_DISCONNECT:
            {
                if( AT_BLE_SUCCESS != status )
                {
                    at_ble_disconnected_t *p = (at_ble_disconnected_t *)params;
                    p->handle = KE_IDX_GET(src);
                    p->reason = 0;
                    evt_num = AT_BLE_DISCONNECTED;
                }
            }
            break;

        case GAPC_BOND:
            {
                //uint8_t role = ROLE_SLAVE;
                evt_num = AT_BLE_PAIR_DONE;
                ((at_ble_pair_done_t*)params)->handle = KE_IDX_GET(src);
                ((at_ble_pair_done_t*)params)->status = (at_ble_status_t)status;
            }
            break;

        case GAPC_ENCRYPT:
            {
                evt_num = AT_BLE_ENCRYPTION_STATUS_CHANGED;
                params->handle = KE_IDX_GET(src);
                if(status != AT_BLE_SUCCESS)
                {
                    params->status = AT_BLE_FAILURE;
                }
                else
                {
                    params->status = AT_BLE_SUCCESS;
                }
            }
            break;

        case GAPC_UPDATE_PARAMS:
            {
                if( AT_BLE_SUCCESS != status )
                {
                    at_ble_conn_param_update_done_t *evt = (at_ble_conn_param_update_done_t *)params;
                    evt->handle = KE_IDX_GET(src);
                    evt->status = (at_ble_status_t)status;
                    evt->con_intv = 0;
                    evt->con_latency = 0;
                    evt->superv_to = 0;
                    evt_num = AT_BLE_CONN_PARAM_UPDATE_DONE;
                }
            }
            break;

        default:
            break;
    }

    return evt_num;
}

at_ble_status_t gapc_encrypt_req_ind(uint16_t src, uint8_t* data, at_ble_encryption_request_t* params)
{
    params->handle = KE_IDX_GET(src);
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT16(&(params->ediv));
    INTERFACE_UNPACK_BLOCK((params->nb),8);
    INTERFACE_UNPACK_DONE();
	return AT_BLE_SUCCESS;
}

at_ble_status_t gapc_sec_req_ind(uint16_t src, uint8_t* data,at_ble_slave_sec_request_t* params)
{
    uint8_t auth;
    params->handle = KE_IDX_GET(src);
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&auth);
    INTERFACE_UNPACK_DONE();
    
	if((auth & 0x01) == 0x01)
    {
        params->bond = 1;
    }
    else
    {
        params->bond = 0;
    }

    if((auth & 0x04) == 0x04)
    {
        params->mitm_protection = 1;
    }
    else
    {
        params->mitm_protection = 0;
    }
	return AT_BLE_SUCCESS;
}

at_ble_status_t gapc_get_info_cmd_handler(uint16_t conn_handle, uint8_t operation)
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, conn_handle));
    INTERFACE_PACK_ARG_UINT8(operation);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	
	return status;
}

// Indication carrying status about whether transmit power has been set
// as requested. On occasions, such as when the request exceeds maximum
// permitted, the power will not be set.
at_ble_status_t gapc_con_tx_pow_set_ind_parser(uint16_t src, uint8_t* data, gapc_con_tx_pow_set_ind* params)
{
	(void)(src);
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&(params->status));
    INTERFACE_UNPACK_DONE();
	
	return AT_BLE_SUCCESS;
}

// Indication carrying the maximum allowed PA gain
at_ble_status_t gapc_con_max_PA_gain_get_ind_parser(uint16_t src, uint8_t* data, gapc_con_max_PA_gain_get_ind* params)
{
	(void)(src);

    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&(params->max_PA_gain));
    INTERFACE_UNPACK_DONE();
	
	return AT_BLE_SUCCESS;
}

// Indication carrying the transmit power being used
at_ble_status_t gapc_con_tx_pow_get_ind_parser(uint16_t src, uint8_t* data, gapc_con_tx_pow_get_ind* params)
{
	(void)(src);

    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&(params->tx_pow_lvl));
    INTERFACE_UNPACK_DONE();
	
	return AT_BLE_SUCCESS;
}

// Indication carrying the rssi of received frames
at_ble_status_t gapc_con_rssi_ind_parser(uint16_t src, uint8_t* data, gapc_con_rssi_ind* params)
{
	(void)(src);

    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&(params->rssi));
    INTERFACE_UNPACK_DONE();
	
	return AT_BLE_SUCCESS;
}

