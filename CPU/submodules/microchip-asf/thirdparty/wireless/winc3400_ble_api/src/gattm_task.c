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
 * gattm_task.c
 *
 */

#include "at_ble_api.h"
#include "platform.h"
#include "interface.h"
#include "gattc_task.h"
#include "gattm_task.h"

ATT_ERR gattm_add_svc_req_handler(uint16_t *start_hdl, uint16_t total_size, uint8_t nb_att_uuid_16,
        uint8_t nb_att_uuid_32, uint8_t nb_att_uuid_128)
{
    uint8_t u8Status;

    INTERFACE_MSG_INIT(GATTM_ADD_SVC_REQ, TASK_ATTM);
    INTERFACE_PACK_ARG_UINT16(*start_hdl);
    INTERFACE_PACK_ARG_UINT16(TASK_EXTERN);
    INTERFACE_PACK_ARG_UINT16(total_size);
    INTERFACE_PACK_ARG_UINT8(nb_att_uuid_16);
    INTERFACE_PACK_ARG_UINT8(nb_att_uuid_32);
    INTERFACE_PACK_ARG_UINT8(nb_att_uuid_128);
    if (INTERFACE_SEND_WAIT(GATTM_ADD_SVC_RSP, TASK_ATTM) != 0)
		return (ATT_ERR)-1;
    INTERFACE_UNPACK_UINT16(start_hdl);
    INTERFACE_UNPACK_UINT8(&u8Status);
    INTERFACE_MSG_DONE();

    return (ATT_ERR)u8Status;
}

ATT_ERR gattm_add_attribute_req_handler (uint16_t start_hdl, uint16_t max_data_size, uint16_t perm,
        uint8_t uuid_len, uint8_t *uuid, uint16_t *handle)
{
    uint8_t u8Status;

    INTERFACE_MSG_INIT(GATTM_ADD_ATTRIBUTE_REQ, TASK_ATTM);
    INTERFACE_PACK_ARG_UINT16(start_hdl);
    INTERFACE_PACK_ARG_UINT16(max_data_size);
    INTERFACE_PACK_ARG_UINT16(perm);
    INTERFACE_PACK_ARG_UINT8(uuid_len);
    INTERFACE_PACK_ARG_BLOCK(uuid, uuid_len);
    if (INTERFACE_SEND_WAIT(GATTM_ADD_ATTRIBUTE_RSP, TASK_ATTM) != 0)
		return (ATT_ERR)-1;
    INTERFACE_UNPACK_UINT16(handle);
    INTERFACE_UNPACK_UINT8(&u8Status);
    INTERFACE_MSG_DONE();

    return (ATT_ERR)u8Status;
}

ATT_ERR gattm_att_set_value_req_handler(uint16_t handle, uint16_t length, uint8_t *value)
{
    uint8_t u8Status;
    uint16_t u16RcvHandle;

    INTERFACE_MSG_INIT(GATTM_ATT_SET_VALUE_REQ, TASK_ATTM);
    INTERFACE_PACK_ARG_UINT16(handle);
    INTERFACE_PACK_ARG_UINT16(length);
    INTERFACE_PACK_ARG_BLOCK(value, length);
    if (INTERFACE_SEND_WAIT(GATTM_ATT_SET_VALUE_RSP, TASK_ATTM) != 0)
		return (ATT_ERR)-1;
    INTERFACE_UNPACK_UINT16(&u16RcvHandle);
    INTERFACE_UNPACK_UINT8(&u8Status);
    INTERFACE_MSG_DONE();

    return (ATT_ERR)u8Status;
}

ATT_ERR gattm_att_get_value_req_handler(uint16_t handle, uint16_t* length, uint8_t *value)
{
    uint16_t u16RcvHandle;
    uint8_t u8Status;

    INTERFACE_MSG_INIT(GATTM_ATT_GET_VALUE_REQ, TASK_ATTM);
    INTERFACE_PACK_ARG_UINT16(handle);
    if (INTERFACE_SEND_WAIT(GATTM_ATT_GET_VALUE_RSP, TASK_ATTM) != 0)
		return (ATT_ERR)-1;
    INTERFACE_UNPACK_UINT16(&u16RcvHandle);
    INTERFACE_UNPACK_UINT16(length);
    INTERFACE_UNPACK_UINT8(&u8Status);
    INTERFACE_UNPACK_BLOCK(value, *length);
    INTERFACE_MSG_DONE();
    if(handle != u16RcvHandle)
		return (ATT_ERR)-1;

    return (ATT_ERR)u8Status;
}