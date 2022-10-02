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
#ifndef __DEVICE_H__
#define __DEVICE_H__

/// Constant defining the role
typedef enum
{
    ///Master role
    ROLE_MASTER,
    ///Slave role
    ROLE_SLAVE,
    ///Enumeration end value for role value check
    ROLE_END
}at_ble_dev_role;

struct device_info
{
    // local device adv and scan data
    uint8_t ADVData[AT_BLE_ADV_MAX_SIZE];
    uint8_t SrData[AT_BLE_ADV_MAX_SIZE];
    uint8_t advLen;
    uint8_t srLen;

    // handle when the device is in a slave connection
    at_ble_handle_t conn_handle;

    // local device bt address
    bool at_addr_set;
    at_ble_addr_t at_dev_addr;
    bool addr_auto_gen;
    uint8_t privacy_flags;
    // peer device address
    at_ble_addr_t peer_addr;

    // the role in which the device is running in now
    at_ble_dev_role role;

    // device local keys
    at_ble_LTK_t ltk;
    at_ble_CSRK_t csrk;
    at_ble_IRK_t irk;
    uint16_t renew_dur;

    at_ble_spcp_t spcp_param;
    uint16_t appearance;
    uint8_t dev_name_write_perm;

};

extern struct device_info device;

#endif
