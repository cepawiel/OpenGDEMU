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
#include "gattm_task.h"
#include "gattc_task.h"
#include "string.h"
#include "error.h"

#include "cmn_defs.h"

static uint8_t primary_service_uuid[] = {0x00, 0x28};
static uint8_t secondary_service_uuid[] = {0x01, 0x28};
static uint8_t include_uuid[] = {0x02, 0x28};
static uint8_t char_uuid[] = {0x03, 0x28};

static uint8_t extended_props_uuid[] = {0x00, 0x29};
static uint8_t user_desc_uuid[] = {0x01, 0x29};
static uint8_t client_conf_uuid[] = {0x02, 0x29};
static uint8_t server_conf_uuid[] = {0x03, 0x29};
static uint8_t presentation_format_uuid[] = {0x04, 0x29};

static bool is_extended_properties_defined(
                        at_ble_characteristic_t *characteristic)
{
    /*
    an extended properties descriptor is needed if either
    "reliable write" or "writable aux" are defined
    */
    return (characteristic->properties
        & (AT_BLE_CHAR_RELIABLE_WRITE | AT_BLE_CHAR_WRITEABLE_AUX)) != 0;

}

static ATT_ERR extended_properties_define(at_ble_handle_t service_handle,
                       at_ble_characteristic_t *characteristic)
{
    at_ble_handle_t temp_handle;
	ATT_ERR att_status;
    uint8_t temp_buff[2] = {0x00, 0x00};

    if((characteristic->properties & AT_BLE_CHAR_RELIABLE_WRITE) == AT_BLE_CHAR_RELIABLE_WRITE)
    {
        temp_buff[0] |= 0x01;
    }
    if((characteristic->properties & AT_BLE_CHAR_WRITEABLE_AUX) == AT_BLE_CHAR_WRITEABLE_AUX)
    {
        temp_buff[0] |= 0x02;
    }

    att_status = gattm_add_attribute_req_handler(service_handle, 2, PERM(RD, ENABLE),
		 sizeof(extended_props_uuid), extended_props_uuid, &temp_handle);
	
	if (att_status == ATT_ERR_NO_ERROR)
	{
		att_status = gattm_att_set_value_req_handler(temp_handle, 2, temp_buff);
	}

    return att_status;
}

static bool is_user_desc_defined(
                        at_ble_characteristic_t *characteristic)
{
    /*
    a user desccription descriptor is defined if the user defines a decription
    */
    return characteristic->user_desc_len != 0;
}

static uint16_t att_permissions(at_ble_attr_permissions_t permissions,char Read,char Write)
{
    uint16_t perm = 0;

    if((permissions & AT_BLE_ATTR_READABLE_NO_AUTHN_REQ_AUTHR)
        == AT_BLE_ATTR_READABLE_NO_AUTHN_REQ_AUTHR)
    {
        perm = PERM(RD,AUTHZ);
    }
    else if((permissions & AT_BLE_ATTR_READABLE_REQ_AUTHN_REQ_AUTHR)
        == AT_BLE_ATTR_READABLE_REQ_AUTHN_REQ_AUTHR)
    {
        perm = PERM(RD,AUTHZ)| PERM(RD,AUTH);
    }
    else if((permissions & AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR)
        == AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR)
    {
        perm = PERM(RD,AUTH);
    }
    else
    {
        if(Read)
        perm = PERM(RD,ENABLE);
    }

    if((permissions & AT_BLE_ATTR_WRITABLE_NO_AUTHN_REQ_AUTHR)
        == AT_BLE_ATTR_WRITABLE_NO_AUTHN_REQ_AUTHR)
    {
        perm |= PERM(WR,AUTHZ);
    }
    else if((permissions & AT_BLE_ATTR_WRITABLE_REQ_AUTHN_REQ_AUTHR)
        == AT_BLE_ATTR_WRITABLE_REQ_AUTHN_REQ_AUTHR)
    {
        perm |= PERM(WR,AUTHZ)| PERM(WR,AUTH);;
    }
    else if((permissions & AT_BLE_ATTR_WRITABLE_REQ_AUTHN_NO_AUTHR)
        == AT_BLE_ATTR_WRITABLE_REQ_AUTHN_NO_AUTHR)
    {
        perm |= PERM(WR,AUTH);
    }
    else
    {
        if(Write)
        perm |= PERM(WR,ENABLE);
    }

    return perm;
}

static ATT_ERR user_description_define(at_ble_handle_t service_handle,
                       at_ble_characteristic_t *characteristic)
{
	ATT_ERR att_status;
    uint16_t perm = PERM(RD, ENABLE);

    if((characteristic->properties & AT_BLE_CHAR_WRITEABLE_AUX) == AT_BLE_CHAR_WRITEABLE_AUX)
    {
        perm = att_permissions(characteristic->user_desc_permissions,1,1);
    }else{
        perm = att_permissions(characteristic->user_desc_permissions,1,0);
    }

    att_status = gattm_add_attribute_req_handler(service_handle, characteristic->user_desc_len, perm,
					sizeof(user_desc_uuid), user_desc_uuid, &characteristic->user_desc_handle);

	if (att_status == ATT_ERR_NO_ERROR)
	{
		if (characteristic->user_desc != NULL)
		{
			att_status = gattm_att_set_value_req_handler(characteristic->user_desc_handle, characteristic->user_desc_len,
							characteristic->user_desc);
		}
	}

    return att_status;
}

static bool is_server_conf_defined(
                        at_ble_characteristic_t *characteristic)
{
    return (characteristic->properties & AT_BLE_CHAR_BROADCST) != 0;
}

static ATT_ERR server_conf_define(at_ble_handle_t service_handle,
                       at_ble_characteristic_t *characteristic)
{
	ATT_ERR att_status;
    uint8_t temp_buff[2] = {0x00, 0x00};

    att_status = gattm_add_attribute_req_handler(service_handle, 2, att_permissions(characteristic->server_config_permissions,1,0),
					sizeof(server_conf_uuid), server_conf_uuid, &characteristic->server_config_handle);
    if (att_status == ATT_ERR_NO_ERROR)
	{
		att_status = gattm_att_set_value_req_handler(characteristic->server_config_handle, 2, temp_buff);
	}

    return att_status;
}

static bool is_client_conf_defined(
                        at_ble_characteristic_t *characteristic)
{
    return (characteristic->properties
        & (AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_INDICATE)) != 0;
}

static ATT_ERR client_conf_define(at_ble_handle_t service_handle,
                       at_ble_characteristic_t *characteristic)
{
	ATT_ERR att_status;
    uint8_t temp_buff[2] = {0x00, 0x00};

    att_status = gattm_add_attribute_req_handler(service_handle, 2, att_permissions(characteristic->client_config_permissions,1,1),
					sizeof(client_conf_uuid), client_conf_uuid, &characteristic->client_config_handle);
    if (att_status == ATT_ERR_NO_ERROR)
	{
		att_status = gattm_att_set_value_req_handler(characteristic->client_config_handle, 2, temp_buff);
	}
    return att_status;
}

static bool is_presentation_format_defined(
                        at_ble_characteristic_t *characteristic)
{
    return (characteristic->presentation_format != NULL);
}

static ATT_ERR presentation_format_define(at_ble_handle_t service_handle,
                       at_ble_characteristic_t *characteristic)
{
	ATT_ERR att_status;
    at_ble_handle_t temp_handle;
    uint8_t temp_buff[7];

    att_status = gattm_add_attribute_req_handler(service_handle, 7, PERM(RD, ENABLE),
					sizeof(presentation_format_uuid), presentation_format_uuid, &temp_handle);
	
	if (att_status == ATT_ERR_NO_ERROR)
	{
		temp_buff[0]=characteristic->presentation_format->format;
		temp_buff[1]=characteristic->presentation_format->exponent;
		temp_buff[2]=characteristic->presentation_format->unit & 0xff;
		temp_buff[3]=characteristic->presentation_format->unit >> 8;
		temp_buff[4]=characteristic->presentation_format->name_space;
		temp_buff[5]=characteristic->presentation_format->description & 0xff;
		temp_buff[6]=characteristic->presentation_format->description >> 8;

		att_status = gattm_att_set_value_req_handler(temp_handle, 7, temp_buff);
	}

    return att_status;
}

static uint8_t char_properties(at_ble_characteristic_t *characteristic)
{
    /* there is 1:1 mapping for the first 7 bits of propertites beteween the standard
    and AT API*/
    uint8_t props = characteristic->properties & 0x7F;

    /* the last bit is set if there is extended properties defined */
    if (is_extended_properties_defined(characteristic))
    {
        props |= 0x80;
    }

    return props;
}

static uint16_t char_permissions(at_ble_characteristic_t *characteristic)
{
    uint16_t perm = 0;

    // TODO: proper permission mapping based on characteristic->value_permissions

    if((characteristic->properties & AT_BLE_CHAR_NOTIFY) == AT_BLE_CHAR_NOTIFY)
    {
        perm |= PERM(NTF, ENABLE);
    }

    if((characteristic->properties & AT_BLE_CHAR_INDICATE) == AT_BLE_CHAR_INDICATE)
    {
        perm |= PERM(IND, ENABLE);
    }

    if ((characteristic->properties & AT_BLE_CHAR_SIGNED_WRITE) == AT_BLE_CHAR_SIGNED_WRITE)
    {
        perm |= PERM(WR, ENABLE)| PERM(WRITE_SIGNED, ENABLE );
    }

    if ((characteristic->value_permissions & AT_BLE_ATTR_READABLE_NO_AUTHN_REQ_AUTHR)
        == AT_BLE_ATTR_READABLE_NO_AUTHN_REQ_AUTHR)
    {
        perm |= PERM(RD,AUTHZ);
    }
    else if ((characteristic->value_permissions & AT_BLE_ATTR_READABLE_REQ_AUTHN_REQ_AUTHR)
        == AT_BLE_ATTR_READABLE_REQ_AUTHN_REQ_AUTHR)
    {
        perm |= PERM(RD,AUTHZ)| PERM(RD,AUTH);
    }
    else if ((characteristic->value_permissions & AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR)
        == AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR)
    {
        perm |= PERM(RD,AUTH);
    } else {

        if((characteristic->properties & AT_BLE_CHAR_READ) == AT_BLE_CHAR_READ)
        {
            perm |= PERM(RD,ENABLE);
        }
    }

    if ((characteristic->value_permissions & AT_BLE_ATTR_WRITABLE_NO_AUTHN_REQ_AUTHR)
        == AT_BLE_ATTR_WRITABLE_NO_AUTHN_REQ_AUTHR)
    {
        perm |= PERM(WR,AUTHZ);
    }
    else if ((characteristic->value_permissions & AT_BLE_ATTR_WRITABLE_REQ_AUTHN_REQ_AUTHR)
        == AT_BLE_ATTR_WRITABLE_REQ_AUTHN_REQ_AUTHR)
    {
        perm |= PERM(WR,AUTHZ)| PERM(WR,AUTH);;
    }
    else if ((characteristic->value_permissions & AT_BLE_ATTR_WRITABLE_REQ_AUTHN_NO_AUTHR)
        == AT_BLE_ATTR_WRITABLE_REQ_AUTHN_NO_AUTHR)
    {
        perm |= PERM(WR,AUTH);
    }
    else
    {
        if ((characteristic->properties & AT_BLE_CHAR_RELIABLE_WRITE) ==  AT_BLE_CHAR_RELIABLE_WRITE)
        {
            perm |= PERM(WR,ENABLE);
        }
        if ((characteristic->properties & AT_BLE_CHAR_WRITE_WITHOUT_RESPONSE) == AT_BLE_CHAR_WRITE_WITHOUT_RESPONSE)
        {
            perm |= PERM(WR,ENABLE);
        }

        if ((characteristic->properties & AT_BLE_CHAR_WRITE) == AT_BLE_CHAR_WRITE)
        {
            perm |= PERM(WR,ENABLE);
        }
    }

    return perm;
}

static uint8_t uuid_size(at_ble_uuid_t* uuid)
{
    uint8_t size;
    switch(uuid->type)
    {
    case AT_BLE_UUID_16:
        size = 2;
        break;

    case AT_BLE_UUID_32:
        size = 4;
        break;

    default:
    case AT_BLE_UUID_128:
        size = 16;
        break;
    }

    return size;
}

static at_ble_status_t at_ble_service_define(at_ble_uuid_t* uuid, at_ble_handle_t* service_handle,
	at_ble_included_service_t * included_list, uint16_t included_count, at_ble_characteristic_t * charactristic_list, uint16_t char_count, bool primary)
{
	uint16_t total_size = 0;
	int i=0;
	uint16_t  start_hdl=0;
	uint8_t nb_att_uuid_16=0,nb_att_uuid_32=0,nb_att_uuid_128=0;
	uint16_t temp_handle;
	uint8_t temp_buff[AT_BLE_UUID_128_LEN + 4];
	ATT_ERR att_status;

	do{
		/* To allocate a service we need to know the number of attributes and total size */

		/* intially, we need one attribute for the service declaration itself */
		nb_att_uuid_16++; /* service decl UUID is 16 bit */
		total_size += uuid_size(uuid); /* data contained in the service decl is the UUID of service */

		/* then go through include services */
		for (i = 0; i < included_count; i++)
		{
			nb_att_uuid_16++; /* include decl UUID is 16 bit */
			total_size += uuid_size(&included_list[i].uuid) + 4; /* data contained is the UUID plus 2 handles */
		}

		/* Finally the characteristic list */
		for (i = 0; i < char_count; i++)
		{
			/* account for the char decl attribute */
			nb_att_uuid_16++; /* char decl UUID is 16 bit */
			total_size += uuid_size(&charactristic_list[i].uuid) + 3; /* data contained is the UUID plus a handle and properties */

			/* then the chartrisct-containing attribute itself */
			switch(charactristic_list[i].uuid.type)
			{
				case AT_BLE_UUID_16:
					nb_att_uuid_16++;
					break;
				case AT_BLE_UUID_32:
					nb_att_uuid_32++;
					break;
				case AT_BLE_UUID_128:
					nb_att_uuid_128++;
					break;
				case AT_BLE_UUID_INVALID:
					default:
					break;
			}
			total_size += charactristic_list[i].value_max_len;

			/* Finally the descriptors */
			if (is_extended_properties_defined(&charactristic_list[i]))
			{
				nb_att_uuid_16++;
				total_size += 2;
			}
			if (is_user_desc_defined(&charactristic_list[i]))
			{
				nb_att_uuid_16++;
				total_size += charactristic_list[i].user_desc_len;
			}
			if (is_server_conf_defined(&charactristic_list[i]))
			{
				nb_att_uuid_16++;
				total_size += 2;
			}
			if (is_client_conf_defined(&charactristic_list[i]))
			{
				nb_att_uuid_16++;
				total_size += 2;
			}
			if (is_presentation_format_defined(&charactristic_list[i]))
			{
				nb_att_uuid_16++;
				total_size += 7;
			}
		}

		att_status = gattm_add_svc_req_handler(&start_hdl, total_size,  nb_att_uuid_16,  nb_att_uuid_32,  nb_att_uuid_128);
		if (att_status != ATT_ERR_NO_ERROR)
			break;

		*service_handle = start_hdl;
		/* define "service decl" attribute */
		if (primary)
		{
			att_status = gattm_add_attribute_req_handler(start_hdl, uuid_size(uuid), PERM(RD, ENABLE),
				sizeof(primary_service_uuid), primary_service_uuid, &temp_handle);
		}
		else
		{
			att_status = gattm_add_attribute_req_handler(start_hdl, uuid_size(uuid), PERM(RD, ENABLE),
				sizeof(secondary_service_uuid), secondary_service_uuid, &temp_handle);
		}
		if (att_status != ATT_ERR_NO_ERROR)
			break;

		att_status = gattm_att_set_value_req_handler(temp_handle, uuid_size(uuid), uuid->uuid);
		if (att_status != ATT_ERR_NO_ERROR)
			break;

		/* define "include" attributes */
		for (i = 0; i < included_count; i++)
		{
			at_ble_included_service_t *included_service = &included_list[i];

			att_status = gattm_add_attribute_req_handler(start_hdl, uuid_size(&included_list[i].uuid) + 4, PERM(RD, ENABLE),
							sizeof(include_uuid), include_uuid, &temp_handle);
			if (att_status != ATT_ERR_NO_ERROR)
				break;

			temp_buff[0] = included_service->service_handle & 0xff;
			temp_buff[1] = included_service->service_handle >> 8;
			temp_buff[2] = included_service->end_group_handle & 0xff;
			temp_buff[3] = included_service->end_group_handle >> 8;
			memcpy(&temp_buff[4], &included_list[i].uuid.uuid, uuid_size(&included_list[i].uuid));
		
			att_status = gattm_att_set_value_req_handler(temp_handle, uuid_size(&included_list[i].uuid) + 4, temp_buff);
			if (att_status != ATT_ERR_NO_ERROR)
				break;
		}

		if (att_status != ATT_ERR_NO_ERROR)
			break;

		/* define characteristics */
		for (i = 0; i < char_count; i++)
		{
			at_ble_characteristic_t *charactaristic = &charactristic_list[i];

			/* define "char decl" attribute */
			att_status = gattm_add_attribute_req_handler(start_hdl, uuid_size(&charactaristic->uuid) + 3, PERM(RD, ENABLE),
							sizeof(char_uuid), char_uuid, &temp_handle);
			if(att_status != ATT_ERR_NO_ERROR)
				break;

			/* define the attribute holding the value */
			att_status = gattm_add_attribute_req_handler(start_hdl, charactaristic->value_max_len, char_permissions(charactaristic),
							uuid_size(&charactaristic->uuid), charactaristic->uuid.uuid, &charactaristic->char_val_handle);
			if(att_status != ATT_ERR_NO_ERROR)
				break;

			if (charactaristic->value_init_len > 0 && charactaristic->init_value != NULL)
			{
				att_status = gattm_att_set_value_req_handler(charactaristic->char_val_handle,
								charactaristic->value_init_len, charactaristic->init_value);
				if(att_status != ATT_ERR_NO_ERROR)
					break;
			}

			/* lets set the value of "char dec" attribute, this was defered until we got the handle from the above step */
			temp_buff[0] = char_properties(charactaristic);
			temp_buff[1] = charactaristic->char_val_handle & 0xff;
			temp_buff[2] =  charactaristic->char_val_handle >> 8;
			memcpy(&temp_buff[3], &charactaristic->uuid.uuid, uuid_size(&charactaristic->uuid));

			att_status = gattm_att_set_value_req_handler(temp_handle,  uuid_size(&charactaristic->uuid) + 3, temp_buff);
			if (att_status != ATT_ERR_NO_ERROR)
				break;

			/* define the descriptors */
			if (is_extended_properties_defined(charactaristic))
			{
				extended_properties_define(start_hdl, charactaristic);
			}

			if (is_user_desc_defined(charactaristic))
			{
				user_description_define(start_hdl, charactaristic);
			}

			if (is_server_conf_defined(charactaristic))
			{
				server_conf_define(start_hdl, charactaristic);
			}

			if (is_client_conf_defined(charactaristic))
			{
				client_conf_define(start_hdl, charactaristic);
			}

			if (is_presentation_format_defined(charactaristic))
			{
				presentation_format_define(start_hdl, charactaristic);
			}
		}
	} while(0);

	return at_ble_att_error(att_status);
}

at_ble_status_t at_ble_primary_service_define(at_ble_uuid_t* uuid, at_ble_handle_t* service_handle,
	at_ble_included_service_t * included_list, uint16_t included_count, at_ble_characteristic_t * charactristic_list, uint16_t char_count)
{
	return at_ble_service_define(uuid, service_handle, included_list, included_count, charactristic_list, char_count, true);
}


at_ble_status_t at_ble_secondary_service_define(at_ble_uuid_t* uuid, at_ble_handle_t* service_handle,
	at_ble_included_service_t * included_list, uint16_t included_count, at_ble_characteristic_t * charactristic_list, uint16_t char_count)
{
	return at_ble_service_define(uuid, service_handle, included_list, included_count, charactristic_list, char_count, false);
}

at_ble_status_t at_ble_characteristic_value_set(at_ble_handle_t handle,
    uint8_t* value, uint16_t len)
{
    ATT_ERR att_status = gattm_att_set_value_req_handler(handle, len, value);
    
	return at_ble_att_error(att_status);
}

at_ble_status_t at_ble_characteristic_value_get(at_ble_handle_t handle,
    uint8_t* value, uint16_t *actual_read_len)
{
    ATT_ERR att_status = gattm_att_get_value_req_handler(handle, actual_read_len, value);

    return at_ble_att_error(att_status);
}

at_ble_status_t at_ble_notification_send(at_ble_handle_t conn_handle,
    at_ble_handle_t attr_handle)
{
    return gattc_send_evt_cmd_handler(GATTC_NOTIFY, attr_handle, conn_handle);
}

at_ble_status_t at_ble_indication_send(at_ble_handle_t conn_handle,
    at_ble_handle_t attr_handle)
{
    return gattc_send_evt_cmd_handler(GATTC_INDICATE, attr_handle, conn_handle);
}

at_ble_status_t at_ble_service_changed_notification_send(at_ble_handle_t conn_handle, at_ble_handle_t start_handle, at_ble_handle_t end_handle)
{
	if( start_handle > end_handle )
    {
		return AT_BLE_INVALID_PARAM;
	}
    return gattc_svc_changed_notification_cmd_handler(conn_handle, start_handle, end_handle);
}