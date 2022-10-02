/*******************************************************************************
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
 * wifiprov_task.c
 *
 */
#include "at_ble_api.h"
#include "platform.h"
#include "interface.h"
#
#include "wifiprov_task.h"
#include "profiles.h"

/*
 * msgs from APP to wifiprov
 *
 *
 */
at_ble_status_t wifiprov_configure_provisioning(uint8_t* lname)
{
    //TODO: check on platform error
    uint8_t lname_len = 0;
	uint8_t u8Status;
    if (lname)
    {
        lname_len = strlen((const char*)lname);
        if (lname_len > MAX_LOCALNAME_LENGTH)
            lname_len = MAX_LOCALNAME_LENGTH;
    }


    INTERFACE_MSG_INIT(WIFIPROV_CONFIGURE_REQ, TASK_WIFIPROV);
    INTERFACE_PACK_ARG_UINT8(lname_len);
    INTERFACE_PACK_ARG_BLOCK(lname,lname_len);
    if(INTERFACE_SEND_WAIT(WIFIPROV_CONFIGURE_CFM, TASK_WIFIPROV) != 0)
		return AT_BLE_FAILURE;
    INTERFACE_UNPACK_UINT8(&u8Status);
    
    INTERFACE_MSG_DONE();
    return (at_ble_status_t)u8Status;
}

at_ble_status_t wifiprov_create_db(void)
{
    //TODO: check on platform error
	uint8_t u8Status;

    INTERFACE_MSG_INIT(WIFIPROV_CREATE_DB_REQ, TASK_WIFIPROV);
    INTERFACE_PACK_ARG_UINT8(1|2); // We support both scanning and connection
    if(INTERFACE_SEND_WAIT(WIFIPROV_CREATE_DB_CFM, TASK_WIFIPROV) != 0)
		return AT_BLE_FAILURE;
    INTERFACE_UNPACK_UINT8(&u8Status);
    //GTODO: Export status define from wifiprov to header file
    
    INTERFACE_MSG_DONE();
    return (at_ble_status_t)u8Status;
}

at_ble_status_t wifiprov_start(uint8_t *pin, uint8_t len)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(WIFIPROV_START_CMD, TASK_WIFIPROV);
    INTERFACE_PACK_ARG_BLOCK(pin,len);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
    return status;
}

at_ble_status_t wifiprov_disable(void)
{
	at_ble_status_t status;

    INTERFACE_MSG_INIT(WIFIPROV_DISABLE_CMD, TASK_WIFIPROV);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
    return status;
}

at_ble_status_t wifiprov_scan_mode_change_ind_send(uint8_t scanmode)
{
    at_ble_status_t status;

    INTERFACE_MSG_INIT(WIFIPROV_SCAN_MODE_CHANGE_IND, TASK_WIFIPROV);
    INTERFACE_PACK_ARG_UINT8(scanmode);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
    return status;
}

at_ble_status_t wifiprov_scan_list_ind_send(struct wifiprov_scanlist_ind *param)
{
    at_ble_status_t status;

    INTERFACE_MSG_INIT(WIFIPROV_SCANLIST_IND, TASK_WIFIPROV);
    INTERFACE_PACK_ARG_BLOCK(param, sizeof(struct wifiprov_scanlist_ind));  // GTODO: Does this need to be 'repacked' using pack macros?
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
    return status;
}

at_ble_status_t wifiprov_wifi_con_update(uint8_t conn_state)
{
    at_ble_status_t status;

    INTERFACE_MSG_INIT(WIFIPROV_CONNECTION_STATE_CHANGE_IND, TASK_WIFIPROV);
    INTERFACE_PACK_ARG_UINT8(conn_state);
    status = INTERFACE_SEND_NO_WAIT();
    INTERFACE_MSG_DONE();
    return status;
}

/*
 * events from WIFIPROV to APP
 *
 *
 */
void wifiprov_scan_mode_change_ind_handler(uint8_t *data, at_ble_wifiprov_scan_mode_change_ind_t *param)
{
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&param->scanmode);
    INTERFACE_UNPACK_DONE();
}

void wifiprov_complete_ind_handler(uint8_t *data, at_ble_wifiprov_complete_ind *param)
{
    INTERFACE_UNPACK_INIT(data);
    INTERFACE_UNPACK_UINT8(&param->status);
    INTERFACE_UNPACK_UINT8(&param->sec_type);
    INTERFACE_UNPACK_UINT8(&param->ssid_length);
    INTERFACE_UNPACK_BLOCK(&param->ssid, MAX_WIPROVTASK_SSID_LENGTH);
    INTERFACE_UNPACK_UINT8(&param->passphrase_length);
    INTERFACE_UNPACK_BLOCK(&param->passphrase, MAX_WIPROVTASK_PASS_LENGTH);
    INTERFACE_UNPACK_DONE();
}

