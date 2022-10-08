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
#include "gattc_task.h"
#include "device.h"

static uint8_t at_ble_uuid_type2len(at_ble_uuid_type_t type)
{
    switch(type)
    {
    case AT_BLE_UUID_16 :
        return AT_BLE_UUID_16_LEN;

    case AT_BLE_UUID_32 :
        return AT_BLE_UUID_32_LEN;

    default:
        return AT_BLE_UUID_128_LEN;
    }
}

at_ble_status_t at_ble_primary_service_discover_all(at_ble_handle_t conn_handle,
    at_ble_handle_t start_handle, at_ble_handle_t end_handle)
{
    // check if end handle is not smaller than start handle
    if((end_handle < start_handle) || start_handle == 0)
    {
		return  AT_BLE_INVALID_PARAM;
	}       
	
	return gattc_disc_cmd_handler(GATTC_DISC_ALL_SVC,0,
				start_handle, end_handle, NULL, conn_handle);
    
}

at_ble_status_t at_ble_primary_service_discover_by_uuid(at_ble_handle_t conn_handle,
    at_ble_handle_t start_handle, at_ble_handle_t end_handle, at_ble_uuid_t* uuid)
{
    uint32_t uuid_len = 0, valid_uuid=0;

    // check UUID is non zero
    uuid_len = at_ble_uuid_type2len(uuid->type);

    for(;uuid_len>0;uuid_len--)
    {
        valid_uuid += uuid->uuid[uuid_len - 1];
    }

    // check if end handle is not smaller than start handle

	if ((end_handle < start_handle) || (start_handle == 0) || (valid_uuid == 0))
    {
		return AT_BLE_INVALID_PARAM;
	}

	return gattc_disc_cmd_handler(GATTC_DISC_BY_UUID_SVC,
			 at_ble_uuid_type2len(uuid->type),
            start_handle, end_handle, uuid->uuid, conn_handle);
}

at_ble_status_t at_ble_included_service_discover_all(at_ble_handle_t conn_handle,
    at_ble_handle_t start_handle, at_ble_handle_t end_handle)
{
    // check if end handle is not smaller than start handle
    if (end_handle < start_handle)
    {
		return AT_BLE_INVALID_PARAM;
	}

    return gattc_disc_cmd_handler(GATTC_DISC_INCLUDED_SVC, 0,
            start_handle, end_handle, NULL, conn_handle);
}

at_ble_status_t at_ble_characteristic_discover_all(at_ble_handle_t conn_handle,
    at_ble_handle_t start_handle, at_ble_handle_t end_handle)
{
    // check if end handle is not smaller than start handle
    if ((end_handle < start_handle) || (start_handle == 0))
    {
	   return AT_BLE_INVALID_PARAM;
	}

	return gattc_disc_cmd_handler(GATTC_DISC_ALL_CHAR, 0,
            start_handle, end_handle, NULL, conn_handle);
}

at_ble_status_t at_ble_characteristic_discover_by_uuid(at_ble_handle_t conn_handle,
    at_ble_handle_t start_handle, at_ble_handle_t end_handle, at_ble_uuid_t* uuid)
{
    uint32_t uuid_len = 0, valid_uuid = 0;

    // check UUID is non zero
    uuid_len = at_ble_uuid_type2len(uuid->type);

    for(;uuid_len>0;uuid_len--)
    {
        valid_uuid += uuid->uuid[uuid_len - 1];
    }

    // check if end handle is not smaller than start handle
    if ((end_handle < start_handle) || (start_handle == 0) || (valid_uuid == 0))
    {
		return AT_BLE_INVALID_PARAM;    
	}
	
	return gattc_disc_cmd_handler(GATTC_DISC_BY_UUID_CHAR,
            at_ble_uuid_type2len(uuid->type),
            start_handle, end_handle, uuid->uuid, conn_handle);
    
}

at_ble_status_t at_ble_descriptor_discover_all(at_ble_handle_t conn_handle,
    at_ble_handle_t start_handle, at_ble_handle_t end_handle)
{
    // check if end handle is not smaller than start handle
    if((end_handle < start_handle) || (start_handle == 0))
    {
		return AT_BLE_INVALID_PARAM;
	} 

	return gattc_disc_cmd_handler(GATTC_DISC_DESC_CHAR,
			0, 
			start_handle, end_handle, NULL, conn_handle);
}

at_ble_status_t at_ble_characteristic_read_by_uuid(at_ble_handle_t conn_handle,
    at_ble_handle_t start_handle, at_ble_handle_t end_handle, at_ble_uuid_t* uuid)
{
    // check if end handle is not smaller than start handle
    if(end_handle < start_handle)
    {
        return AT_BLE_INVALID_PARAM;
	}

	return gattc_read_cmd_handler_by_uuid(start_handle, end_handle,
            at_ble_uuid_type2len(uuid->type), uuid->uuid, conn_handle);
}

at_ble_status_t at_ble_characteristic_read(at_ble_handle_t conn_handle, at_ble_handle_t char_handle, uint16_t offset, uint16_t length)
{
    return gattc_read_cmd_handler_simple_read(char_handle, offset, length, conn_handle);
}

at_ble_status_t at_ble_characteristic_read_multible(at_ble_handle_t conn_handle,  at_ble_handle_t* char_handle_list,
    uint16_t* char_val_lenght, uint8_t char_handle_count)
{
    return gattc_read_cmd_handler_multible(char_handle_list, char_handle_count, char_val_lenght, conn_handle);
}

at_ble_status_t at_ble_characteristic_write(at_ble_handle_t conn_handle, at_ble_handle_t char_handle,
            uint16_t offset, uint16_t length, uint8_t* data,
            bool signed_write, bool with_response )
{
    uint8_t req = (signed_write)?  GATTC_WRITE_SIGNED:
	              (with_response)? GATTC_WRITE:
								   GATTC_WRITE_NO_RESPONSE;

    return gattc_write_cmd_handler(req, 1, char_handle, offset, length, data, conn_handle);
}

at_ble_status_t at_ble_characteristic_reliable_write_prepare(at_ble_handle_t conn_handle,
        at_ble_handle_t char_handle,
        uint16_t offset, uint16_t length, uint8_t* data)
{
    return gattc_write_cmd_handler(GATTC_WRITE, 0, char_handle, offset, length, data, conn_handle);
}

at_ble_status_t at_ble_characteristic_reliable_write_execute(at_ble_handle_t conn_handle)
{
    return gattc_execute_write_cmd_handler(1, conn_handle);
}

at_ble_status_t at_ble_characteristic_reliable_write_cancel(at_ble_handle_t conn_handle)
{
	return gattc_execute_write_cmd_handler(0, conn_handle);
}

at_ble_status_t at_ble_exchange_mtu(at_ble_handle_t conn_handle)
{
    return gattc_mtu_cmd_handler(conn_handle);
}

