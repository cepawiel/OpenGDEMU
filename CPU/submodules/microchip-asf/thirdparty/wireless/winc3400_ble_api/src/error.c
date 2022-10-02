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
#include "gattc_task.h"
#include "gapm_task.h"
#include "error.h"

at_ble_status_t at_ble_att_error(ATT_ERR att_error_code)
{
    at_ble_status_t status;
    switch(att_error_code)
    {
    case  ATT_ERR_NO_ERROR:
        status = AT_BLE_SUCCESS;
        break;
    case ATT_ERR_INVALID_HANDLE:
        status = AT_BLE_INVALID_HANDLE;
        break;
    case ATT_ERR_INSUFF_RESOURCE:
        status = AT_BLE_INSUFF_RESOURCE;
        break;
    case ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN:
        status = AT_BLE_INVALID_ATTRIBUTE_VAL_LEN;
        break;
    default:
        status = AT_BLE_FAILURE;
        break;
    }
    return status;
}

at_ble_status_t at_ble_gap_error(uint8_t gap_error_code)
{
    at_ble_status_t status;
    switch(gap_error_code)
    {
    case   GAP_ERR_INVALID_PARAM:
        status = AT_BLE_INVALID_PARAM;
        break;
    case  GAP_ERR_COMMAND_DISALLOWED:
        status = AT_BLE_COMMAND_DISALLOWED;
        break;
    case GAP_ERR_NO_ERROR:
        status = AT_BLE_SUCCESS;
        break;
    default:
        status = AT_BLE_FAILURE;
        break;
    }
    return status;
} 