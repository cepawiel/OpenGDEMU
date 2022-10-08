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
 * gap_task.c
 *
 */

#include "at_ble_api.h"
#include "platform.h"
#include "interface.h"

#include "gapm_task.h"
#include "dbg_task.h"
#include "gapc_task.h"

#include "error.h"
#include "device.h"

// original location : llm.h
enum
{
    LLM_ADV_CONN_UNDIR = 0x00,
    LLM_ADV_CONN_DIR,
    LLM_ADV_NONCONN_UNDIR,
    LLM_SCAN_REQ,
    LLM_SCAN_RSP,
    LLM_CONNECT_REQ,
    LLM_ADV_DISC_UNDIR,
    LLM_ADV_END
};

/*
 * gapm_get_address_type
 * The following function will parse addrerss type to at_ble_addr_type_t values according to
 * last byte of address and received type from FW.
 * Where received values from FW are PUBLIC or RANDOM only.
 */
static uint8_t gapm_get_address_type(uint8_t *pu8Addr, uint8_t u8AddrType)
{
    uint8_t u8RetValue = AT_BLE_ADDRESS_PUBLIC;

    if(0x00 != (u8AddrType & 0x000000FF))
    {
        if(0xC0 == (0xC0 & pu8Addr[AT_BLE_ADDR_LEN-1]))
        {
            u8RetValue = AT_BLE_ADDRESS_RANDOM_STATIC;
        }
        else if(0x40 == (0x40 & pu8Addr[AT_BLE_ADDR_LEN-1]))
        {
            u8RetValue = AT_BLE_ADDRESS_RANDOM_PRIVATE_RESOLVABLE;
        }
        else
        {
            u8RetValue= AT_BLE_ADDRESS_RANDOM_PRIVATE_NON_RESOLVABLE;
        }
    }

    return u8RetValue;
}

/*
* address on air is sent either 0:public or 1:random
*/
#define gapm_set_address_type(type) ((type>0 && type <4)?1:0)

at_ble_status_t gapm_reset_req_handler (void)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(GAPM_RESET_CMD, TASK_GAPM);
    INTERFACE_PACK_ARG_UINT8(GAPM_RESET);
    status = INTERFACE_SEND_WAIT(GAPM_CMP_EVT, TASK_GAPM);

	if (status == AT_BLE_SUCCESS)
	{
	    uint8_t u8Operation, u8Status;
		INTERFACE_UNPACK_UINT8(&u8Operation);
		INTERFACE_UNPACK_UINT8(&u8Status);
	    if (u8Status != 0 || u8Operation != GAPM_RESET)
		{
		    status = AT_BLE_FAILURE;
		}
	}
    INTERFACE_MSG_DONE();
    return status;
}

at_ble_status_t gapm_set_dev_name_handler(uint8_t len, uint8_t* name)
{
	at_ble_status_t	status;

    INTERFACE_MSG_INIT(GAPM_SET_DEV_NAME_CMD, TASK_GAPM);
    INTERFACE_PACK_ARG_UINT8(GAPM_SET_DEV_NAME);
    INTERFACE_PACK_ARG_UINT8(len);
    INTERFACE_PACK_ARG_BLOCK(name, len);
    status = INTERFACE_SEND_WAIT(GAPM_CMP_EVT, TASK_GAPM);
	
	if (status == AT_BLE_SUCCESS)
	{
	    uint8_t u8Operation, u8Status;
		INTERFACE_UNPACK_UINT8(&u8Operation);
		INTERFACE_UNPACK_UINT8(&u8Status);
		if(u8Status != 0 || u8Operation != GAPM_SET_DEV_NAME)
		{
			status = AT_BLE_FAILURE;
		}
	}
	INTERFACE_MSG_DONE();
    
	return status;
}

at_ble_status_t gapm_start_adv_cmd_handler (uint8_t u8OpCode, uint8_t u8AddrSrc, uint16_t u16RenewDur,
        uint8_t *pu8BdAddr,uint8_t peer_addr_type ,uint8_t *peerBdAddr, uint16_t u16MinIntv, uint16_t u16MaxIntv, uint8_t u8ChnlMap, uint8_t u8Mode ,
    uint8_t u8AdvFiltPolicy, uint8_t u8AdvDataLen, uint8_t *pu8AdvData, uint8_t u8ScnRespLen, uint8_t *pu8ScnRespData)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(GAPM_START_ADVERTISE_CMD, TASK_GAPM);
    INTERFACE_PACK_ARG_UINT8(u8OpCode);
    INTERFACE_PACK_ARG_UINT8(u8AddrSrc);
    // Dummy data use to retrieve internal operation state (should be set to 0).
    INTERFACE_PACK_ARG_UINT16(0);
    // To do check renew address duration
    INTERFACE_PACK_ARG_UINT16(u16RenewDur);
    // Local address
    if(pu8BdAddr != NULL)
    {
        INTERFACE_PACK_ARG_BLOCK(pu8BdAddr, AT_BLE_ADDR_LEN);
    }
    else
    {
        INTERFACE_PACK_ARG_DUMMY(AT_BLE_ADDR_LEN);
    }
    INTERFACE_PACK_ARG_UINT16(u16MinIntv);
    INTERFACE_PACK_ARG_UINT16(u16MaxIntv);
    INTERFACE_PACK_ARG_UINT8(u8ChnlMap);
    if( u8OpCode == GAPM_ADV_DIRECT)
    {
        if(peerBdAddr != NULL)
        {
            INTERFACE_PACK_ARG_BLOCK(peerBdAddr, AT_BLE_ADDR_LEN);
        }
        else
        {
            INTERFACE_PACK_ARG_DUMMY(AT_BLE_ADDR_LEN);
        }
        INTERFACE_PACK_ARG_UINT8(peer_addr_type);
    }
    else
    {
        INTERFACE_PACK_ARG_UINT8(u8Mode);
        INTERFACE_PACK_ARG_UINT8(u8AdvFiltPolicy);
        // advertising data
        INTERFACE_PACK_ARG_UINT8(u8AdvDataLen);
        INTERFACE_PACK_ARG_BLOCK(pu8AdvData, u8AdvDataLen);
        INTERFACE_PACK_ARG_DUMMY((ADV_DATA_MAX_SIZE - u8AdvDataLen));
        // scan response
        INTERFACE_PACK_ARG_UINT8(u8ScnRespLen);
        INTERFACE_PACK_ARG_BLOCK(pu8ScnRespData, u8ScnRespLen);
        INTERFACE_PACK_ARG_DUMMY((SCAN_RESP_DATA_MAX_SIZE - u8ScnRespLen));
    }

    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
	return status;
}

at_ble_status_t gapm_set_dev_config_cmd_handler(uint8_t u8Role, uint8_t *pu8Key,
                uint16_t u16Appearance, uint8_t u8AppWrPerm, uint8_t u8NameWrPerm, uint16_t u16MaxMTU,
                uint16_t u16ConIntMin, uint16_t u16ConIntMax, uint16_t u16ConLatency,
                uint16_t u16SupervTo, uint8_t u8Flags)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(GAPM_SET_DEV_CONFIG_CMD, TASK_GAPM);
    INTERFACE_PACK_ARG_UINT8(GAPM_SET_DEV_CONFIG);
    INTERFACE_PACK_ARG_UINT8(u8Role);
    INTERFACE_PACK_ARG_BLOCK(pu8Key, KEY_LEN);
    INTERFACE_PACK_ARG_UINT16(u16Appearance);
    INTERFACE_PACK_ARG_UINT8(u8AppWrPerm);
    INTERFACE_PACK_ARG_UINT8(u8NameWrPerm);
    INTERFACE_PACK_ARG_UINT16(u16MaxMTU);
    if(u8Role == GAP_PERIPHERAL_SLV)
    {
        INTERFACE_PACK_ARG_UINT16(u16ConIntMin);
        INTERFACE_PACK_ARG_UINT16(u16ConIntMax);
        INTERFACE_PACK_ARG_UINT16(u16ConLatency);
        INTERFACE_PACK_ARG_UINT16(u16SupervTo);
        INTERFACE_PACK_ARG_UINT8(u8Flags);
    }
   
    status = INTERFACE_SEND_WAIT(GAPM_CMP_EVT, TASK_GAPM);
	if (status == AT_BLE_SUCCESS)
	{
	    uint8_t u8Operation, u8Status;
		INTERFACE_UNPACK_UINT8(&u8Operation);
		INTERFACE_UNPACK_UINT8(&u8Status);	
		if(u8Status != 0 || u8Operation!=GAPM_SET_DEV_CONFIG)
		{
			status = AT_BLE_FAILURE;
		}
	}
	INTERFACE_MSG_DONE();

    return status;
}

at_ble_status_t gapm_start_scan_cmd_handler (uint8_t u8OpCode, uint8_t u8AddrType, uint16_t u16RenewDur,
        uint8_t *pu8BdAddr, uint16_t u16ScanInterval, uint16_t u16ScanWin, uint8_t u8ScanMode, uint8_t u8FiltPolicy,
        uint8_t u8FilterDuplic)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(GAPM_START_SCAN_CMD, TASK_GAPM);
    INTERFACE_PACK_ARG_UINT8(u8OpCode);
    INTERFACE_PACK_ARG_UINT8(u8AddrType);
    INTERFACE_PACK_ARG_UINT16(0);
    INTERFACE_PACK_ARG_UINT16(u16RenewDur);
    INTERFACE_PACK_ARG_BLOCK(pu8BdAddr, BD_ADDR_LEN);
    INTERFACE_PACK_ARG_UINT16(u16ScanInterval);
    INTERFACE_PACK_ARG_UINT16(u16ScanWin);
    INTERFACE_PACK_ARG_UINT8(u8ScanMode);
    INTERFACE_PACK_ARG_UINT8(u8FiltPolicy);
    INTERFACE_PACK_ARG_UINT8(u8FilterDuplic);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();

	return status;
}

at_ble_status_t gapm_start_connection_cmd_handler(uint8_t u8OpCode, uint8_t u8AddrType, uint16_t u16RenewDur,
            uint8_t *pu8BdAddr, uint16_t u16ScanInterval, uint16_t u16ScanWin, uint16_t u16ConIntvMin,
            uint16_t u16ConIntvMax, uint16_t u16ConLatency, uint16_t u16SupervTO, uint16_t u16CeMin,
            uint16_t u16CeMAx, uint8_t u8NbOfPeers, at_ble_addr_t *peers)
{
	at_ble_status_t status;
    uint8_t i = 0;

    INTERFACE_MSG_INIT(GAPM_START_CONNECTION_CMD, TASK_GAPM);
    INTERFACE_PACK_ARG_UINT8(u8OpCode);
    INTERFACE_PACK_ARG_UINT8(u8AddrType);
    INTERFACE_PACK_ARG_UINT16(0);
    INTERFACE_PACK_ARG_UINT16(u16RenewDur);
    INTERFACE_PACK_ARG_BLOCK(pu8BdAddr, BD_ADDR_LEN);
    INTERFACE_PACK_ARG_UINT16(u16ScanInterval);
    INTERFACE_PACK_ARG_UINT16(u16ScanWin);
    INTERFACE_PACK_ARG_UINT16(u16ConIntvMin);
    INTERFACE_PACK_ARG_UINT16(u16ConIntvMax);
    INTERFACE_PACK_ARG_UINT16(u16ConLatency);
    INTERFACE_PACK_ARG_UINT16(u16SupervTO);
    INTERFACE_PACK_ARG_UINT16(u16CeMin);
    INTERFACE_PACK_ARG_UINT16(u16CeMAx);
    INTERFACE_PACK_ARG_UINT8(u8NbOfPeers);
    for(i = 0; i < u8NbOfPeers; i++)
    {
        INTERFACE_PACK_ARG_BLOCK((peers->addr), BD_ADDR_LEN);
        INTERFACE_PACK_ARG_UINT8(gapm_set_address_type(peers->type));
        peers +=1;
    }
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();

	return status;
}

at_ble_status_t gapm_cancel_cmd_handler(void)
{
    at_ble_status_t status;

    INTERFACE_MSG_INIT(GAPM_CANCEL_CMD, TASK_GAPM);
    INTERFACE_PACK_ARG_UINT8(GAPM_CANCEL);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();

    return status;
}

at_ble_status_t gapm_connection_cfm_handler( uint8_t *pu8PeerBdAddr, uint8_t u8PeerAddrType,uint16_t u16ConIntvMin,
            uint16_t u16ConIntvMax, uint16_t u16ConLatency, uint16_t u16SupervTO, uint16_t u16CeMin,
            uint16_t u16CeMAx)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(GAPM_CONNECTION_CFM, TASK_GAPM);
    INTERFACE_PACK_ARG_BLOCK(pu8PeerBdAddr, BD_ADDR_LEN);
    INTERFACE_PACK_ARG_UINT8(u8PeerAddrType);
    INTERFACE_PACK_ARG_UINT8(0);//zero padding
    INTERFACE_PACK_ARG_UINT16(u16ConIntvMin);
    INTERFACE_PACK_ARG_UINT16(u16ConIntvMax);
    INTERFACE_PACK_ARG_UINT16(u16ConLatency);
    INTERFACE_PACK_ARG_UINT16(u16SupervTO);
    INTERFACE_PACK_ARG_UINT16(u16CeMin);
    INTERFACE_PACK_ARG_UINT16(u16CeMAx);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();

	return status;
}

at_ble_status_t gapm_resolv_addr_cmd_handler(uint8_t nb_key , uint8_t* rand_addr , uint8_t* irk)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(GAPM_RESOLV_ADDR_CMD,TASK_GAPM);
    INTERFACE_PACK_ARG_UINT8(GAPM_RESOLV_ADDR);
    INTERFACE_PACK_ARG_UINT8(nb_key);
    INTERFACE_PACK_ARG_BLOCK(rand_addr,AT_BLE_ADDR_LEN);
    INTERFACE_PACK_ARG_BLOCK(irk, (KEY_LEN *nb_key));
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();

	return status;
}

at_ble_status_t gapm_white_list_mgm_cmd(uint8_t operation, uint8_t addr_type, uint8_t* address)
{
    // To do check complete effect operation or not
    at_ble_status_t status;

	(void)(operation);

    INTERFACE_MSG_INIT(GAPM_WHITE_LIST_MGT_CMD, TASK_GAPM);
    INTERFACE_PACK_ARG_UINT8(GAPM_ADD_DEV_IN_WLIST);
    /// Number of device information present in command
    if(address != NULL)
    {
        INTERFACE_PACK_ARG_UINT8(1);
        INTERFACE_PACK_ARG_BLOCK(address, AT_BLE_ADDR_LEN);
        INTERFACE_PACK_ARG_UINT8(addr_type);
    }
    status = INTERFACE_SEND_WAIT(GAPM_CMP_EVT, TASK_GAPM);
    INTERFACE_MSG_DONE();

    return status;
}

at_ble_status_t gapm_adv_report_evt_handler(uint8_t* data, at_ble_scan_info_t* param)
{
    uint8_t evt_type;
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&evt_type);
    INTERFACE_UNPACK_UINT8(&(param->dev_addr.type));
    INTERFACE_UNPACK_BLOCK(param->dev_addr.addr, AT_BLE_ADDR_LEN);
    INTERFACE_UNPACK_UINT8(&(param->adv_data_len));
    INTERFACE_UNPACK_BLOCK(param->adv_data ,param->adv_data_len);
    INTERFACE_UNPACK_DONE();

    param->dev_addr.type = gapm_get_address_type((uint8_t *)(&(param->dev_addr.addr)), param->dev_addr.type);

    switch(evt_type)
    {
        case ADV_CONN_UNDIR:
            param->type = AT_BLE_ADV_TYPE_UNDIRECTED;
            break;

        case ADV_CONN_DIR:
            param->type = AT_BLE_ADV_TYPE_DIRECTED;
            break;

        case ADV_DISC_UNDIR:
            param->type = AT_BLE_ADV_TYPE_SCANNABLE_UNDIRECTED;
            break;

        case ADV_NONCONN_UNDIR:
            param->type = AT_BLE_ADV_TYPE_NONCONN_UNDIRECTED;
            break;

        case LLM_SCAN_RSP:
            param->type = AT_BLE_ADV_TYPE_SCAN_RESPONSE;
            break;
    }

	return AT_BLE_SUCCESS;
}

at_ble_status_t gapm_dev_bdaddr_ind_handler(uint8_t* data,at_ble_rand_addr_changed_t* param)
{
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_BLOCK(param->addr, AT_BLE_ADDR_LEN);
    INTERFACE_UNPACK_DONE();

	return AT_BLE_SUCCESS;
}

at_ble_events_t gapm_cmp_evt(uint8_t* data, void* params)
{
    uint8_t u8Operation , u8status;
    at_ble_events_t evt_num = AT_BLE_UNDEFINED_EVENT;
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&u8Operation);
    INTERFACE_UNPACK_UINT8(&u8status);
    INTERFACE_UNPACK_DONE();

    //if(u8Operation == GAPM_ADV_UNDIRECT && u8status ==
    if((u8Operation == GAPM_CONNECTION_AUTO)||
        (((u8Operation == GAPM_ADV_UNDIRECT) || (u8Operation == GAPM_ADV_DIRECT))&&(device.conn_handle != 0xFFFF)))
    {
        at_ble_connected_t *connected = (at_ble_connected_t*)params;
        connected->handle = device.conn_handle;
        connected->peer_addr.type = device.peer_addr.type;
        connected->conn_status = at_ble_gap_error(u8status);
        memcpy(connected->peer_addr.addr, device.peer_addr.addr, AT_BLE_ADDR_LEN);

        connected->peer_addr.type = gapm_get_address_type((uint8_t *)(&(connected->peer_addr.addr)), connected->peer_addr.type);

        evt_num = AT_BLE_CONNECTED;
    }

    if(u8Operation == GAPM_RESOLV_ADDR)
    {
        if(u8status != GAP_ERR_NO_ERROR)
        {
            at_ble_resolv_rand_addr_status_t * resolv_status =
                (at_ble_resolv_rand_addr_status_t*)params;
            evt_num = AT_BLE_RESOLV_RAND_ADDR_STATUS;
            resolv_status->status = at_ble_gap_error(u8status);
        }
    }

    if((u8Operation == GAPM_SCAN_PASSIVE)
    ||(u8Operation == GAPM_SCAN_ACTIVE))
    {
        at_ble_scan_report_t *scan_report = (at_ble_scan_report_t*) params;
        if(u8status == GAP_ERR_TIMEOUT)
        {
            evt_num = AT_BLE_SCAN_REPORT;
            scan_report->status = AT_BLE_SUCCESS;
        }
    }

    return evt_num;
}

at_ble_status_t gapm_addr_solved_ind_handler(uint8_t* data , at_ble_resolv_rand_addr_status_t* params)
{
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_SKIP(6);
    INTERFACE_UNPACK_BLOCK(params->irk,AT_BLE_MAX_KEY_LEN);
    INTERFACE_UNPACK_DONE();
    params->status = AT_BLE_SUCCESS;
	return AT_BLE_SUCCESS;
}

