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

#include "at_ble_api.h"
#include "platform.h"
#include "interface.h"

#include "dbg_task.h"

at_ble_status_t dbg_le_set_bd_addr_req_handler(uint8_t* addr)
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(DBG_LE_SET_BD_ADDR_REQ, TASK_DBG);
    INTERFACE_PACK_ARG_BLOCK(addr , AT_BLE_ADDR_LEN);
    status = INTERFACE_SEND_WAIT(DBG_LE_SET_BD_ADDR_CMP_EVT, TASK_DBG);
	if (status == AT_BLE_SUCCESS)
	{
		uint8_t u8Status;
	    INTERFACE_UNPACK_UINT8(&u8Status);
		if (u8Status != 0)
		{
			status = AT_BLE_FAILURE;
		}
	}
    INTERFACE_MSG_DONE();

    return status;
}

at_ble_status_t dbg_le_get_bd_addr_req_handler(uint8_t* addr)
{
	at_ble_status_t status;
    
	INTERFACE_MSG_INIT(DBG_LE_GET_BD_ADDR_REQ, TASK_DBG);
    status = INTERFACE_SEND_WAIT(DBG_LE_GET_BD_ADDR_CMP_EVT, TASK_DBG);
	if (status == AT_BLE_SUCCESS)
	{
	    INTERFACE_UNPACK_BLOCK(addr,AT_BLE_ADDR_LEN);
	}
    INTERFACE_MSG_DONE();
    return status;
}

at_ble_status_t dbg_set_tx_pw_req_handler(uint16_t conn_handle, uint8_t level)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(DBG_SET_TX_PW_REQ, TASK_DBG);
    INTERFACE_PACK_ARG_UINT16(conn_handle);
    INTERFACE_PACK_ARG_UINT8(level);
    status = INTERFACE_SEND_WAIT(DBG_SET_TX_PW_CMP_EVT, TASK_DBG);
	if (status == AT_BLE_SUCCESS)
	{
		uint8_t u8Status;
		INTERFACE_UNPACK_UINT8(&u8Status);
		if (u8Status != 0)
		{
			status = AT_BLE_FAILURE;		
		}
	}
    INTERFACE_MSG_DONE();

    return status;
}

at_ble_status_t dbg_wr_mem_req_handler(uint32_t memAddr,uint8_t* data,uint8_t size)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(DBG_WR_MEM_REQ, TASK_DBG);
    INTERFACE_PACK_ARG_UINT32(memAddr);
    INTERFACE_PACK_ARG_UINT8(8);
    INTERFACE_PACK_ARG_UINT8(size);
    INTERFACE_PACK_ARG_BLOCK(data,size);
	status = INTERFACE_SEND_WAIT(DBG_WR_MEM_CMP_EVT, TASK_DBG);
	if (status == AT_BLE_SUCCESS)
	{
		uint8_t u8Status;
		INTERFACE_UNPACK_UINT8(&u8Status);
		if (u8Status != 0)
		{
			status = AT_BLE_FAILURE;
		}
	}
    INTERFACE_MSG_DONE();

    return status;
}

at_ble_status_t dbg_wr_mem_req_handler32_reset(uint32_t memAddr,uint32_t * data,uint8_t size)
{
    at_ble_status_t status;

    INTERFACE_MSG_INIT(DBG_WR_MEM_REQ, TASK_DBG);
    INTERFACE_PACK_ARG_UINT32(memAddr);
    INTERFACE_PACK_ARG_UINT8(32);
    INTERFACE_PACK_ARG_UINT8(size);
    INTERFACE_PACK_ARG_BLOCK(data,size);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();

	return status;
}

at_ble_status_t dbg_wr_mem_req_handler32(uint32_t memAddr,uint32_t* data,uint8_t size)
{
    at_ble_status_t status;
    
	INTERFACE_MSG_INIT(DBG_WR_MEM_REQ, TASK_DBG);
    INTERFACE_PACK_ARG_UINT32(memAddr);
    INTERFACE_PACK_ARG_UINT8(32);
    INTERFACE_PACK_ARG_UINT8(size);
    INTERFACE_PACK_ARG_BLOCK(data,size);
    status = INTERFACE_SEND_WAIT(DBG_WR_MEM_CMP_EVT, TASK_DBG);
	if (status == AT_BLE_SUCCESS)
	{
		uint8_t u8Status;
		
		INTERFACE_UNPACK_UINT8(&u8Status);
		if (u8Status != 0)
		{
			status = AT_BLE_FAILURE;
		}
	}
    INTERFACE_MSG_DONE();

    return status;
}

at_ble_status_t dbg_rd_mem_req_handler(uint32_t memAddr,uint8_t* data ,uint8_t size)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(DBG_RD_MEM_REQ, TASK_DBG);
    INTERFACE_PACK_ARG_UINT32(memAddr);
    INTERFACE_PACK_ARG_UINT8(8);
    INTERFACE_PACK_ARG_UINT8(size);
	status = INTERFACE_SEND_WAIT(DBG_RD_MEM_CMP_EVT, TASK_DBG);
	if (status == AT_BLE_SUCCESS)
	{
		uint8_t u8Status;
		INTERFACE_UNPACK_UINT8(&u8Status);
		INTERFACE_UNPACK_BLOCK(data,size);
		if (u8Status != 0)
		{
			status = AT_BLE_FAILURE;
		}
    }
	INTERFACE_MSG_DONE();

    return status;
}

at_ble_status_t dbg_rd_mem_req_handler32(uint32_t memAddr,uint8_t* data ,uint8_t size)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(DBG_RD_MEM_REQ, TASK_DBG);
    INTERFACE_PACK_ARG_UINT32(memAddr);
    INTERFACE_PACK_ARG_UINT8(32);
    INTERFACE_PACK_ARG_UINT8(size);
	status = INTERFACE_SEND_WAIT(DBG_RD_MEM_CMP_EVT, TASK_DBG);
	if (status == AT_BLE_SUCCESS)
	{
	    uint8_t u8Status;
		INTERFACE_UNPACK_UINT8(&u8Status);
		INTERFACE_UNPACK_BLOCK(data,size);
		if (u8Status != 0)
		{
			status = AT_BLE_FAILURE;
		}
	}
    INTERFACE_MSG_DONE();

    return status;
}