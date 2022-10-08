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
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include "platform.h"
extern uint32_t gu32Jiffies1ms;
#define TIMEOUT_VALUE 4000
static uint32_t gu32Timeout = TIMEOUT_VALUE;

static plf_params_t plf_params;

#define MINIMUM_FW_MAJ (1)
#define MINIMUM_FW_MIN (2)
#define MINIMUM_FW_PATCH  (2)

void platform_set_timeout(uint32_t timeout)
{
	gu32Timeout = timeout;
}

//Called from the app/wifi driver to inject a BLE blob received over the HIF into the bottom of the BLE API
//stack.
void platform_receive(uint8_t* data, uint32_t len)
{
    platform_interface_callback(data, len);
}

at_ble_status_t platform_init(plf_params_t* platform_params)
{
	at_ble_status_t ret = AT_BLE_SUCCESS;
	
	if (platform_params->mv == VERSION_FIELD_VALID)
	{
		ret = AT_BLE_FW_INCOMPAT;
		if (platform_params->fw_ver.major > MINIMUM_FW_MAJ)
		{
			ret = AT_BLE_SUCCESS;
		}
		else if (platform_params->fw_ver.major == MINIMUM_FW_MAJ)
		{
			if (platform_params->fw_ver.minor > MINIMUM_FW_MIN)
			{
				ret = AT_BLE_SUCCESS;
			}
			else if (platform_params->fw_ver.minor == MINIMUM_FW_MIN)
			{
				if (platform_params->fw_ver.patch >= MINIMUM_FW_PATCH)
				{
					ret = AT_BLE_SUCCESS;
				}
			}
		}
	}
	
	plf_params.ble_write_cb = platform_params->ble_write_cb;
	plf_params.plf_wait_cb = platform_params->plf_wait_cb;
	
	return ret;
}

//Call the specified callback which will handle sending the BLE API blob over the HIF
at_ble_status_t platform_interface_send(uint8_t* data, uint32_t len)
{
    if (plf_params.ble_write_cb && (plf_params.ble_write_cb(data, len) == 0))
		return AT_BLE_SUCCESS;
	return AT_BLE_FAILURE;		
}

static volatile uint32_t cmd_cmpl_flag = 0;
static volatile uint32_t event_flag = 0;

void platform_cmd_cmpl_signal()
{
    cmd_cmpl_flag = 1;
}

at_ble_status_t platform_cmd_cmpl_wait(void)
{
    uint32_t timer = gu32Jiffies1ms+gu32Timeout;
    do
    {
        //Allow the app/wifi driver to do things during this busy wait
        if (plf_params.plf_wait_cb)
            plf_params.plf_wait_cb();
    } while((cmd_cmpl_flag != 1) && ((gu32Jiffies1ms - timer ) & 0x80000000));

    if (cmd_cmpl_flag == 1)
        cmd_cmpl_flag = 0;
    else
        return AT_BLE_FAILURE;

    return AT_BLE_SUCCESS;
}

void platform_event_signal()
{
    event_flag = 1;
}

at_ble_status_t platform_event_wait(uint32_t timeout)
{
    uint32_t timer = gu32Jiffies1ms+timeout;
    at_ble_status_t status = AT_BLE_SUCCESS;

    do
    {
        //Allow the app/wifi driver to do things during this busy wait
        if (plf_params.plf_wait_cb)
            plf_params.plf_wait_cb();
    } while((event_flag != 1) && ((gu32Jiffies1ms - timer ) & 0x80000000));

    if (event_flag == 1)
        event_flag = 0;
    else
        status = AT_BLE_TIMEOUT;

    return status;
}

