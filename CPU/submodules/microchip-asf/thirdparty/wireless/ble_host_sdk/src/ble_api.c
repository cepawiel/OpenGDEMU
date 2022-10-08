/**
 * \file ble_api.c
 *
 * \brief BLE API definitions
 *
 * Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "platform_files.h"
#include "stdlib.h"
#include "string.h"
#include "bm_mode.h"
#include "bm_application_mode.h"
#include "bm_utils.h"
#include "event_mem.h"
#include "ble_api.h"
#include "interface.h"

BM_APPLICATION_CMDPKT* application_cmdpkt = NULL;

extern uart_write_sync_cb_t ble_uart_send_sync;
extern uart_read_async_cb_t ble_uart_receive_async;

static ble_status_t ble_command_complete_parser(BM_APPLICATION_CMD_OPCODE cmd_id);
static ble_status_t ble_status_report_parser(bledk3_status_t status_mode);
static ble_status_t ble_rssi_read_parser(uint8_t *rssi);
static ble_status_t ble_characteristic_value_read_parser(void *data, uint16_t *data_len);


static ble_status_t ble_command_complete_parser(BM_APPLICATION_CMD_OPCODE cmd_id)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
	
	if(cmd_id == cmd_frame->event_msg.data[0])
	{
		return cmd_frame->event_msg.data[1];
	}
	
	return BLE_COMMAND_DISALLOWED;
}

static ble_status_t ble_status_report_parser(bledk3_status_t status_mode)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
	
	if(status_mode == cmd_frame->event_msg.data[0])
	{
		return BLE_SUCCESS;
	}
	
	return BLE_FAILURE;
}

static ble_status_t ble_rssi_read_parser(uint8_t *rssi)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
	
	if(BM_RSSI_READ == cmd_frame->event_msg.data[0])
	{
		if(BLE_SUCCESS == cmd_frame->event_msg.data[1])
		{
			*rssi = cmd_frame->event_msg.data[2];
		}
		
		return cmd_frame->event_msg.data[1];
	}
	
	return BLE_FAILURE;
}

static ble_status_t ble_characteristic_value_read_parser(void *data, uint16_t *data_len)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
	
	if((BM_SERVER_CHARACTERISTIC_READ == cmd_frame->event_msg.data[0]) || (BM_CLIENT_CHARACTERISTIC_READ == cmd_frame->event_msg.data[0]))
	{
		if(BLE_SUCCESS == cmd_frame->event_msg.data[1])
		{
            uint8_t index = 0;
			*data_len = cmd_frame->event_msg.data_len - 3; /* command complete event_id + remote device name read cmd_id + status */
			for(index = 0; index < *data_len; index++)
			{
				*((uint8_t *)data + index) = cmd_frame->event_msg.data[index + 2];	/* remote device name read cmd_id + status */
			}
		}
		return cmd_frame->event_msg.data[1];
	}
	return BLE_FAILURE;
}

static ble_status_t ble_characteristic_value_read_by_uuid_parser(void *data, uint16_t *data_len)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
	
	if(BM_CLIENT_CHARACTERISTIC_UUID_READ == cmd_frame->event_msg.data[0])
	{
		if(BLE_SUCCESS == cmd_frame->event_msg.data[1])
		{
            uint8_t index = 0;
			*data_len = cmd_frame->event_msg.data_len - 3; /* command complete event_id + cmd_id for client characteristic read by uuid + status */
			for(index = 0; index < *data_len; index++)
			{
				*((uint8_t *)data + index) = cmd_frame->event_msg.data[index + 2];	/* cmd_id for client characteristic read by uuid + status */
			}
		}
		return cmd_frame->event_msg.data[1];
	}
	return BLE_FAILURE;
}

static ble_status_t ble_gatt_service_create_parser(uint16_t *handle)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
	
	if(BM_SERVER_CREATE_SERVICE_REQUEST == cmd_frame->event_msg.data[0])
	{
		if(BLE_SUCCESS == cmd_frame->event_msg.data[1])
		{
			*handle = ((uint16_t)cmd_frame->event_msg.data[2]) << 8;
			*handle |= cmd_frame->event_msg.data[3];
		}
		return cmd_frame->event_msg.data[1];
	}
	
	return BLE_FAILURE;
}

#if 0
static ble_status_t ble_remote_device_name_read_parser(uint8_t *name, uint8_t *name_length)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
	
	if(BM_REMOTE_NAME_READ == cmd_frame->event_msg.data[0])
	{
		if(BLE_SUCCESS == cmd_frame->event_msg.data[1])
		{
			*name_length = cmd_frame->event_msg.data_len - 3; /* command complete event_id + remote device name read cmd_id + status */
			for(uint8_t index = 0; index < *name_length; index++)
			{
				name[index] = cmd_frame->event_msg.data[index + 2];	/* remote device name read cmd_id + status */
			}
			
		}
		
		return cmd_frame->event_msg.data[1];
	}
	
	return BLE_FAILURE;
}
#endif /* #if 0 */

/*
 * @brief Resets BM7x module
 *
 * @param pointer to platform_init which contains platform API list and event memory
 *
 * @return status of BLE initialization
*/
ble_status_t ble_init(platform_init_t *platform_init)
{
	ble_status_t status = BLE_SUCCESS;
	uint8_t i;
	
	status = interface_init(platform_init);
	if(BLE_SUCCESS != status)
	{
		return status;
	}
	
	application_cmdpkt = (BM_APPLICATION_CMDPKT*)BM_APPLICATION_Init();
	
	/* Set operating mode */
	platform_init->platform_api_list.mode_set(BM_MODE_APPLICATION);
	
    for(i=0; i<0x55; i++)
    {
	    BM_APPLICATION_PatternSetPkt(application_cmdpkt, 0x01);
	    
	    status = interface_cmd_send_wait_time(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE, 25);
	    if(BLE_SUCCESS == status)
	    {
		    /* Check the status in status-report event */
		    status = ble_status_report_parser(BM_PATTERN_SET);
		    
		    status = interface_event_wait_time(STATUS_REPORT, 25);
		    if(BLE_SUCCESS == status)
		    {
			    /* Check the status in status-report event */
			    status = ble_status_report_parser(IDLE_MODE);
			    break;
		    }
	    }
    }
    
    if (status != BLE_SUCCESS)
    {
	    BM_APPLICATION_ResetPkt(application_cmdpkt);
	    status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, STATUS_REPORT);
	    if(BLE_SUCCESS == status)
	    {
		    /* Check the status in status-report event */
		    status = ble_status_report_parser(IDLE_MODE);
	    }
    }
		
	return status;
}

/*
 * @brief Set an advertisement data on BMxx
 *
 * @param[in] adv_data advertisement data
 * @param[in] adv_data_len advertisement data length
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_adv_data_set(uint8_t const *adv_data, uint8_t adv_data_len)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(NULL == adv_data)
	{
		return status;
	}
	
	if(adv_data_len > 31)
	{
		return status;
	}
	
	if(adv_data && adv_data_len)
	{
		BM_APPLICATION_AdvertisementDataWritePkt(application_cmdpkt, (uint8_t *) adv_data, adv_data_len, true);
		status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
		
		if(BLE_SUCCESS == status)
		{
			/* Check the status in status-report event */
			status = ble_command_complete_parser(BM_ADV_DATA_WRITE);
		}
	}
		
	return status;
}

/*
 * @brief Set scan response data on BMxx
 *
 * @param[in] scan_data scan response data
 * @param[in] scan_data_len scan response data length
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_scan_resp_data_set(uint8_t const *scan_data, uint8_t scan_data_len)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(NULL == scan_data)
	{
		return status;
	}
	
	if(scan_data_len > 31)
	{
		return status;
	}
	
	if(scan_data && scan_data_len)
	{
		BM_APPLICATION_ScanDataWritePkt(application_cmdpkt, (uint8_t *) scan_data, scan_data_len, true);
		status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
		
		if(BLE_SUCCESS == status)
		{
			/* Check the status in status-report event */
			status = ble_command_complete_parser(BM_SCAN_DATA_WRITE);
		}
	}
		
	return status;
}

/*
 * @brief Set an advertisement parameters on BMxx
 *
 * @param[in] type advertisement type @ref BM_ADV_TYPE
 * @param[in] peer_addr peer device address and its type. Only applicable in direct advertising mode
 * @param[in] interval advertisment interval. each unit is .625 ms
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_adv_param_set(BM_ADV_TYPE type, ble_addr_t *peer_addr, uint16_t interval)
{
	ble_status_t status = BLE_SUCCESS;
	
	uint8_t adv_param[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t index = 0;
	
	if((interval < AT_BLE_ADV_INTERVAL_MIN) || (interval > AT_BLE_ADV_INTERVAL_MAX))
	{
		return BLE_INVALID_COMMAND_PARAMETERS;
	}
	
	if(type > BM_ADV_PROPRIETARY_BEACON)
	{
		return BLE_INVALID_COMMAND_PARAMETERS;
	}
	
	adv_param[ADV_PARAM_TYPE_INDEX] = type;
	adv_param[ADV_PARAM_INTERVAL_INDEX_HIGH] = (uint8_t) (interval >> 8);
	adv_param[ADV_PARAM_INTERVAL_INDEX_LOW] = (uint8_t) interval;
	
	if(NULL != peer_addr)
	{
		adv_param[ADV_PARAM_DIRECT_ADDR_TYPE_INDEX] = peer_addr->type;
		for(index = 0; index < BLE_ADDR_LEN; index++)
		{
			adv_param[ADV_PARAM_DIRECT_ADDR_INDEX + index] = peer_addr->addr[index];
		}
	}
	
	/* Advertisement Type */
	BM_APPLICATION_AdvertisementParameterSetPkt(application_cmdpkt, adv_param);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_ADV_PARAM_SET);
	}
		
	return BLE_SUCCESS;
}

/* @brief Set an advertisement parameters on BMxx
 *
 * @param
 *
 * @return 
*/
ble_status_t ble_adv_start(void)
{
	ble_status_t status;
	uint8_t adv_enable = 0x81;
	
	BM_APPLICATION_AdvertisementEnableSetPkt(application_cmdpkt, adv_enable);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	
	if(BLE_SUCCESS == status)
	{
		status = ble_command_complete_parser(BM_SET_ADV_ENABLE);
	}
		
	return status;
}

ble_status_t ble_adv_stop(void)
{
	ble_status_t status;
	uint8_t adv_enable = 0x00;
	
	BM_APPLICATION_AdvertisementEnableSetPkt(application_cmdpkt, adv_enable);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	
	if(BLE_SUCCESS == status)
	{
		status = ble_command_complete_parser(BM_SET_ADV_ENABLE);
	}
		
	return status;
}

/*
 * @brief Set device name for BMxx
 *
 * @param[in] device_name name of the device
 * @param[in] length length of device name
 *
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_set_device_name(const uint8_t *device_name, uint8_t length)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if((NULL == device_name) || (!length))
	{
		return status;
	}
	
    BM_APPLICATION_DeviceNameWritePkt(application_cmdpkt, (uint8_t *)device_name, length);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_NAME_WRITE);
	}
		
	return status;
}

ble_status_t ble_pair_request(ble_handle_t conn_handle)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0)
	{
		return status;
	}
	BM_APPLICATION_PairRequest(application_cmdpkt, conn_handle);
    status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_PAIRING_REQUEST);
	}
	return status;
}

ble_status_t ble_pair_mode_set(BM_PAIR_MODE mode)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
    if(mode >= BM_PAIR_INVALID_MODE)
	{
		return status;
	}
    BM_APPLICATION_PairModeWritePkt(application_cmdpkt, mode);
    status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_PAIR_MODE_WRITE);
	}
	return status;
}

ble_status_t ble_pair_mode_get()
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
    BM_APPLICATION_PairModeReadPkt(application_cmdpkt);
    status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_PAIR_MODE_READ);
	}
	return status;
}

ble_status_t ble_pair_passkey_enter(ble_handle_t conn_handle, uint8_t* passkey, uint8_t length)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || passkey == NULL || length > 6)
	{
		return status;
	}

	{
		int i=0;
		for(i=0; i<length; i++)
		{
			uint8_t digit = *(passkey+i);
			if(digit>=0 && digit<=9)
			{
				digit += 0x30; 
			}
			BM_APPLICATION_PairPassKeyDigitEnterPkt(application_cmdpkt, conn_handle, digit);
			status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
            if(BLE_SUCCESS == status)
            {
                /* Check the status in status-report event */
                status = ble_command_complete_parser(BM_PASSKEY_ENTRY);
            }
		}
	}

	return status;
}

ble_status_t ble_pair_passkey_reply(ble_handle_t conn_handle)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0)
	{
		return status;
	}
	BM_APPLICATION_PairPassKeyEntryCompletePkt(application_cmdpkt, conn_handle);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
    if(BLE_SUCCESS == status)
    {
        /* Check the status in status-report event */
        status = ble_command_complete_parser(BM_PASSKEY_ENTRY);
    }	

	return status;
}	

ble_status_t ble_pair_passkey_erase_digits(ble_handle_t conn_handle, uint8_t length)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || length > 6)
	{
		return status;
	}

	{
		int i=0;
		for(i=0; i<length; i++)
		{
			BM_APPLICATION_PairPassKeyDigitErasePkt(application_cmdpkt, conn_handle);
			status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
            if(BLE_SUCCESS == status)
            {
                /* Check the status in status-report event */
                status = ble_command_complete_parser(BM_PASSKEY_ENTRY);
            }
		}
	}

	return status;
}

ble_status_t ble_pair_passkey_clear(ble_handle_t conn_handle)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0)
	{
		return status;
	}
	BM_APPLICATION_PairPassKeyClearPkt(application_cmdpkt, conn_handle);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
    if(BLE_SUCCESS == status)
    {
        /* Check the status in status-report event */
        status = ble_command_complete_parser(BM_PASSKEY_ENTRY);
    }	

	return status;
}

ble_status_t ble_pair_passkey_confirm(ble_handle_t conn_handle, ble_pair_confirm_t confirm)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0)
	{
		return status;
	}
	BM_APPLICATION_PairYesNoConfirmPkt(application_cmdpkt, conn_handle, confirm);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
    if(BLE_SUCCESS == status)
    {
        /* Check the status in status-report event */
        status = ble_command_complete_parser(BM_PASSKEY_YESNO_CONFIRM);
    }	

	return status;
}

ble_status_t ble_pair_device_erase_all(void)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
    BM_APPLICATION_PairDeviceListErasePkt(application_cmdpkt);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
    if(BLE_SUCCESS == status)
    {
        /* Check the status in status-report event */
        status = ble_command_complete_parser(BM_PDL_ERASE);
    }	

	return status;
}

ble_status_t ble_pair_device_delete(uint8_t index)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
    if(index > 7)
    {
        return status;
    }
    BM_APPLICATION_PairDeviceDeletePkt(application_cmdpkt, index);
    status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
    if(BLE_SUCCESS == status)
    {
        /* Check the status in status-report event */
        status = ble_command_complete_parser(BM_DEVICE_ERASE);
    }	

	return status;
}

/*
 * @brief Enable transparent UART service
 *
 * @param[in] enable_transparent_uart transparent UART parameters @ref ble_enable_transparent_uart_t
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_transparent_uart_enable(ble_enable_transparent_uart_t *enable_transparent_uart)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(NULL == enable_transparent_uart)
	{
		return status;
	}
	BM_APPLICATION_EnableTransparentUartSetPkt(application_cmdpkt, enable_transparent_uart->conn_handle, 
												enable_transparent_uart->server_transparent_cntrl, 
												enable_transparent_uart->client_transparent_mode);
	
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_TRANSPARENT_ENABLE);
	}
	
	return BLE_SUCCESS;
}

/*
 * @brief Transparent UART data
 *
 * @param[in] data transparent UART data
 * @param[in] datalen length of transparent UART data
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_transparent_uart_data_send(ble_handle_t conn_handle, uint8_t *data, uint8_t datalen)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || (NULL == data) || (!datalen))
	{
		return status;
	}
	
	BM_APPLICATION_SendTransparentDataPkt(application_cmdpkt, conn_handle, data, datalen);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_TRANSPARENT_DATA_SEND);
	}
	
	return status;
}

/*
 * @brief Gives RSSI value for peer connection. This API is only valid after connection established with peer device
 *
 * @param[in] conn_handle connection handle
 * @param[out] rssi rssi value of peer connection
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_rssi_read(uint8_t conn_handle, uint8_t *rssi)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(NULL == rssi)
	{
		return status;
	}
	
	BM_APPLICATION_RSSIReadPkt(application_cmdpkt, conn_handle);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_rssi_read_parser(rssi);
	}
		
	return status;
}

/*
 * @brief Set scan parameters
 *
 * @param[in] scan_interval time interval between subsequent scans. The scan interval should be in the range of 0x0004 to 0x4000
 * @param[in] scan_window is a scan duration and it should not be greater than scan_interval
 * @param[in] scan_type controls the type of scan @ref ble_scan_type_t
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_scan_param_set(uint16_t scan_interval, uint16_t scan_window, ble_scan_type_t scan_type)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(scan_interval < MIN_SCAN_INTERVAL || 
		scan_interval > MAX_SCAN_INTERVAL ||
		scan_window < MIN_SCAN_WINDOW || 
		scan_window > MAX_SCAN_WINDOW ||
		scan_type > BLE_SCAN_ACTIVE)
	{
		return status;
	}
	
	BM_APPLICATION_ScanParameterSetPkt(application_cmdpkt, scan_interval, scan_window, scan_type);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_SCAN_PARAM_SET);
	}
		
	return status;
}

/*
 * @brief Start scan
 *
 * @param[in] scan_enable enables/disables scan operation
 * @param[in] scan_dup_filter enables/disables scan duplicate filter
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_scan_start(ble_scan_enable_t scan_enable, ble_scan_duplicate_filter_t scan_dup_filter)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	BM_APPLICATION_ScanEnableSetPkt(application_cmdpkt, scan_enable, scan_dup_filter);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_SCAN_ENABLE_SET);
	}
		
	return status;
}

/*
 * @brief Set connection parameters
 *
 * @param[in] conn_params provides connection parameter for upcoming connections. @ref ble_set_conn_param_t
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_set_connection_params(ble_set_conn_param_t *conn_params)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(NULL == conn_params)
	{
		return status;
	}
	
	BM_APPLICATION_ConnParameterSetPkt(application_cmdpkt, conn_params->min_conn_interval, conn_params->max_conn_interval, conn_params->conn_latency, conn_params->link_sv_to);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_CONN_PARAM_SET);
	}
	
	return status;
}

/*
 * @brief Create connection
 *
 * @param[in] conn_filter determines the peer device info source. If @ref BLE_CONN_WHITELIST_FILTER_DISABLED is set 
 *				then whitelist is not used select the peer device. If @ref BLE_CONN_WHITELIST_FILTER_ENABLED is set
 *				then whitelist is used to select peer device.
 * @param[in] ble_addr provides peer device address and its type to connect
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_create_connection(ble_connection_filter_t conn_filter, ble_addr_t *address)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(NULL == address)
	{
		return status;
	}
	
	BM_APPLICATION_ConnectDevicePkt(application_cmdpkt, address->addr, address->type, conn_filter);
	status = interface_cmd_send_no_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length);
		
	return status;
}

/*
 * @brief Cancel ongoing connection
 *
 * @param None
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_cancel_connection(void)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	BM_APPLICATION_ConnectDeviceCancelPkt(application_cmdpkt);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_CONNECT_CANCEL);
	}
		
	return status;
}

/*
 * @brief Update connection parameters
 *
 * @param[in] conn_handle identifies the connection 
 * @param[in] conn_param @ref ble_conn_param_t
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
*/
ble_status_t ble_update_connection_parameters(uint8_t conn_handle, ble_conn_param_t *conn_param)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(NULL == conn_param)
	{
		return status;
	}
	
	if(conn_param->conn_interval > MAX_CONN_INTERVAL || 
		conn_param->conn_interval < MIN_CONN_INTERVAL ||
		conn_param->conn_latency > MAX_SLAVE_LATENCY ||		/* Minimum connection slave latency is 0 */
		conn_param->link_sv_to > MAX_SV_TIMEOUT || 
		conn_param->link_sv_to < MIN_SV_TIMEOUT)
	{
		return status;
	}
	
	BM_APPLICATION_ConnectionParameterUpdatePkt(application_cmdpkt, conn_handle, conn_param->conn_interval, conn_param->conn_latency, conn_param->link_sv_to);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_CONNECTION_PARAM_UPDATE);
	}
	
	return status;
}

/*
 * @brief Disconnect from remote device
 *
 * @param[in] conn_handle connection handle. 
 * 
 * @note Since current version supports single connection, the connection handle is not honored.
 *				In future versions when multi connection is supported, this will be honored.
 *
 * @return 
*/
void ble_disconnect_device(uint8_t conn_handle)
{
	BM_APPLICATION_DisconnectDevicePkt(application_cmdpkt, conn_handle);
	interface_cmd_send_no_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length);
		
	return;
}

#if 0
/* Length field along with name will help to improve the SDK. This requires firmware change. So this feature won't be supported, until firmware is upgraded */
/*
 * @brief Read remote device name
 *
 * @param[in] conn_handle connection handle
 * @param[out] name remote device name
 * @param[out] name_length length of remote device name
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
 */
ble_status_t ble_remote_device_name_read(uint8_t conn_handle, uint8_t *name, uint8_t *name_length)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(NULL == name)
	{
		return status;
	}
	
	BM_APPLICATION_RemoteDeviceNameReadPkt(application_cmdpkt, conn_handle);
	status = interface_cmd_send_no_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length);
	
	return status;
}
#endif /* #if 0 */


ble_status_t ble_primary_service_discover_all(ble_handle_t conn_handle)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(conn_handle == 0)
	{
		return status;
	}
	
	BM_APPLICATION_ClientAllServicesDiscoverPkt(application_cmdpkt, conn_handle);
	status = interface_cmd_send_no_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length);
	
	return status;
}

ble_status_t ble_primary_service_characteristics_discover(ble_handle_t conn_handle, ble_uuid_t *service_uuid)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(conn_handle == 0 || service_uuid == NULL || service_uuid->type >= BLE_UUID_INVALID)
	{
		return status;
	}
	
	BM_APPLICATION_ClientCharacteristicsDiscoverPkt(application_cmdpkt, conn_handle, (uint8_t*)&service_uuid->uuid, 1<<(service_uuid->type));
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_CLIENT_DISCOVER_CHARACTERISTICS);
	}
	
	return status;
}

ble_status_t ble_characteristic_read_by_uuid(ble_handle_t conn_handle, ble_uuid_t *characteristic_uuid, void *data, uint16_t *data_len)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(conn_handle == 0 || characteristic_uuid == NULL || characteristic_uuid->type >= BLE_UUID_INVALID)
	{
		return status;
	}
	
	BM_APPLICATION_ClientCharacteristicUUIDReadPkt(application_cmdpkt, conn_handle, (uint8_t*)&characteristic_uuid->uuid, 1<<(characteristic_uuid->type));
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_characteristic_value_read_by_uuid_parser(data, data_len);
	}
	
	return status;
}

ble_status_t ble_characteristic_read(ble_handle_t conn_handle, ble_handle_t characteristic_handle, void *data, uint16_t *data_len)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(conn_handle == 0 || characteristic_handle == 0)
	{
		return status;
	}
	
	BM_APPLICATION_ClientCharacteristicHandleReadPkt(application_cmdpkt, conn_handle, characteristic_handle);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_characteristic_value_read_parser(data, data_len);
	}
	
	return status;
}

ble_status_t ble_characteristic_write_with_response(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t* data, uint16_t length)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(conn_handle == 0 || characteristic_handle == 0 || data == NULL) 
	{
		return status;
	}
	BM_APPLICATION_ClientCharacteristicHandleWritePkt(application_cmdpkt, conn_handle, characteristic_handle, 0x00, data, length);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_CLIENT_CHARACTERISTIC_WRITE);
	}
	
	return status;
}

ble_status_t ble_characteristic_write_without_respose(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t* data, uint16_t length)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(conn_handle == 0 || characteristic_handle == 0 || data == NULL) 
	{
		return status;
	}
	
	BM_APPLICATION_ClientCharacteristicHandleWritePkt(application_cmdpkt, conn_handle, characteristic_handle, 0x01, data, length);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_CLIENT_CHARACTERISTIC_WRITE);
	}
	
	return status;
}

ble_status_t ble_characteristic_reliable_write_request_prepare(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint16_t offset, uint8_t* data, uint16_t length)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || characteristic_handle == 0 || data == NULL) 
	{
		return status;
	}
	BM_APPLICATION_ClientWriteRequestPreparePkt(application_cmdpkt, conn_handle, characteristic_handle, 0x03, offset, data, length);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_CLIENT_WRITE_REQUEST_PREPARE);
	}
	return BLE_SUCCESS;
}

ble_status_t ble_characteristic_reliable_write_request_execute(ble_handle_t conn_handle)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0) 
	{
		return status;
	}
	BM_APPLICATION_ClientWriteRequestExecutePkt(application_cmdpkt, conn_handle, 0x04, 0x01);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_CLIENT_WRITE_REQUEST_EXECUTE);
	}

	return BLE_SUCCESS;
}

ble_status_t ble_characteristic_reliable_write_request_cancel(ble_handle_t conn_handle)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0) 
	{
		return status;
	}
	BM_APPLICATION_ClientWriteRequestExecutePkt(application_cmdpkt, conn_handle, 0x04, 0x00);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_CLIENT_WRITE_REQUEST_EXECUTE);
	}

	return BLE_SUCCESS;
}

ble_status_t ble_characteristic_attribute_read_request(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint16_t offset)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || characteristic_handle == 0) 
	{
		return status;
	}
	BM_APPLICATION_ClientBlobRequestReadPkt(application_cmdpkt, conn_handle, characteristic_handle, offset);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_CLIENT_BLOB_REQUEST_READ);
	}

	return BLE_SUCCESS;
}

ble_status_t ble_characteristic_value_send(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t* data, uint16_t length)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(conn_handle == 0 || characteristic_handle == 0 || data == NULL) 
	{
		return status;
	}
	
	BM_APPLICATION_ServerCharacteristicSendPkt(application_cmdpkt, conn_handle, characteristic_handle, data, length);
	
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_SERVER_CHARACTERISTIC_SEND);
	}
	
	return status;
}

ble_status_t ble_characteristic_value_update(ble_handle_t characteristic_handle, uint8_t* data, uint16_t length)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(characteristic_handle == 0 || data == NULL) 
	{
		return status;
	}
	BM_APPLICATION_ServerCharacteristicUpdatePkt(application_cmdpkt, characteristic_handle, data, length);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_SERVER_CHARACTERISTIC_UPDATE);
	}
	
	return status;
}

ble_status_t ble_characteristic_value_read(ble_handle_t characteristic_handle, void *data, uint16_t *data_len)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(characteristic_handle == 0) 
	{
		return status;
	}
	
	BM_APPLICATION_ServerCharacteristicReadPkt(application_cmdpkt, characteristic_handle);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_characteristic_value_read_parser(data, data_len);
	}
	
	return status;
}

ble_status_t ble_primary_service_read_all(void)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	BM_APPLICATION_ServerPrimaryServicesReadPkt(application_cmdpkt);
    status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = ble_command_complete_parser(BM_SERVER_ALL_SERVICES_READ);
	}
	return status;
}

ble_status_t ble_primary_service_characteristics_read(ble_uuid_t* service_uuid)
{
	ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
	
	if(service_uuid == NULL || service_uuid->type >= BLE_UUID_INVALID)
	{
		return status;
	}
	BM_APPLICATION_ServerPrimaryServiceReadPkt(application_cmdpkt, (uint8_t*)&service_uuid->uuid, 1<<(service_uuid->type));
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the cmd_id in command complete event */
		status = ble_command_complete_parser(BM_SERVER_SERVICE_READ);
	}
	
	return status;
}

ble_status_t ble_write_response_send(ble_handle_t conn_handle, ble_handle_t attr_handle, uint8_t err_code)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || attr_handle == 0)
	{
		return status;
	}
	BM_APPLICATION_ServerWriteResponseSendPkt(application_cmdpkt, conn_handle, 0x12, attr_handle, err_code);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_SERVER_WRITE_RESPONSE_SEND);
	}

	return status;
}

ble_status_t ble_read_response_send(ble_handle_t conn_handle, uint8_t rsp_type, uint8_t* data, uint16_t length)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || data == 0)
	{
		return status;
	}
	BM_APPLICATION_ServerReadResponseSendPkt(application_cmdpkt, conn_handle, rsp_type, data, length);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_SERVER_READ_RESPONSE_SEND);
	}

	return BLE_SUCCESS;
}


ble_status_t ble_characteristic_reliable_write_response_prepare(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint16_t offset, uint8_t* data, uint16_t length)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || characteristic_handle == 0 || data == NULL) 
	{
		return status;
	}
	BM_APPLICATION_ServerWriteResponsePreparePkt(application_cmdpkt, conn_handle, characteristic_handle, 0x02, offset, data, length);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_SERVER_WRITE_RESPONSE_PREPARE);
	}

	return BLE_SUCCESS;
}

ble_status_t ble_characteristic_reliable_write_response_execute(ble_handle_t conn_handle)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0) 
	{
		return status;
	}
	BM_APPLICATION_ServerWriteExecutePreparePkt(application_cmdpkt, conn_handle, 0x03);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_SERVER_WRITE_RESPONSE_EXECUTE);
	}

	return BLE_SUCCESS;
}

ble_status_t ble_characteristic_attribute_read_response(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t* data, uint16_t length)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || characteristic_handle == 0 || data == NULL) 
	{
		return status;
	}
	BM_APPLICATION_ServerBlobResponseReadPkt(application_cmdpkt, conn_handle, characteristic_handle, data, length);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_SERVER_BLOB_RESPONSE_READ);
	}

	return status;
}

ble_status_t ble_error_response_send(ble_handle_t conn_handle, ble_handle_t characteristic_handle, uint8_t req_opcode, uint8_t err_code)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	if(conn_handle == 0 || characteristic_handle == 0) 
	{
		return status;
	}
	BM_APPLICATION_ServerErrorResponsePkt(application_cmdpkt, conn_handle, req_opcode, characteristic_handle, err_code);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_SERVER_ERROR_RESPONSE);
	}

	return status;
}

ble_status_t ble_shutdown(void)
{
    ble_status_t status = BLE_INVALID_COMMAND_PARAMETERS;
    
	BM_APPLICATION_ShutdownPkt(application_cmdpkt);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint16_t) application_cmdpkt->length, COMMAND_COMPLETE);
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_command_complete_parser(BM_CLIENT_BLOB_REQUEST_READ);
	}

	return status;
}

/*
 * @brief Create GATT-Service
 *
 * @param[in/out] ble_service service that needs to be created. The attribute handles will be updated upon successful creation of service
 * 
 * @return Upon successful completion of the function shall return @ref BLE_SUCCESS, otherwise it shall return @ref ble_status_t
 *
 * @note ble_gatt_service_create will take only one service at a time
 */
ble_status_t ble_gatt_service_create(ble_service_t *ble_service)
{
	uint8_t service_buf[BLE_MAX_SERVICE_BUF_LEN], num_of_attrib = 0;
	attrib_service_t *service_header = (attrib_service_t *)service_buf;
	attrib_char_t *char_header = NULL;
	attrib_char_value_t *char_val_header = NULL;
	uint16_t service_buf_len = 0, char_index, desc_index, handle = 0;
	ble_status_t status;
	
	/* There is no GATT-Service available */
	if(ble_service == NULL)
	{
		return BLE_INVALID_COMMAND_PARAMETERS;
	}
	
	/* Service handle defined by user */
	service_header->start_handle = ble_service->handle;
	service_header->end_handle = ble_service->handle;
	
	/* Attribute permissions */
	service_header->permission = ble_service->perm;
	/* Attribute type (UUID) length */
	service_header->type_length = BLE_ATTRIB_UUID_LENGTH_2;
	/* Unless specified by user, create it as primary service */
	if(ble_service->type == SECONDARY_SERVICE)
	{
		service_header->type[0] = (uint8_t) (UUID_SECONDARY_SERVICE >> 8);
		service_header->type[1] = (uint8_t) (UUID_SECONDARY_SERVICE);
	}
	else
	{
		service_header->type[0] = (uint8_t) (UUID_PRIMARY_SERVICE >> 8);
		service_header->type[1] = (uint8_t) (UUID_PRIMARY_SERVICE);
	}
	
	/* Calculate header length, based on UUID type */
	if(ble_service->uuid.type == BLE_UUID_16B)
	{
		/* Service uses 2 bytes UUID */
		service_header->length = (uint8_t)sizeof(attrib_service_t) - 14;
		memcpy(service_header->uuid.uuid_16b, ble_service->uuid.uuid.uuid_16b, BLE_UUID_16B_LEN);
	}
	else if(ble_service->uuid.type == BLE_UUID_128B)
	{
		/* Service uses 16 bytes UUID */
		service_header->length = (uint8_t)sizeof(attrib_service_t);
		memcpy(service_header->uuid.uuid_128b, ble_service->uuid.uuid.uuid_128b, BLE_UUID_128B_LEN);
	}
	
	service_buf_len = service_header->length;
	/* Exclude the length field in calculating overall length */
	service_header->length--;
	/* Counted a Service */
	num_of_attrib++;
	
	/* Include characteristics */
	for(char_index = 0; char_index < ble_service->char_count; char_index++)
	{
		if(service_buf_len > BLE_MAX_SERVICE_BUF_LEN)
		{
			return BLE_INSUFFICIENT_RESOURCES;
		}
		char_header = (attrib_char_t *)(service_buf + service_buf_len);
		
		char_header->permission = ble_service->char_list[char_index].char_val.permissions;
		char_header->properties = ble_service->char_list[char_index].char_val.properties;
		char_header->start_handle = ble_service->char_list[char_index].char_val.char_handle;
		/* Value handle needs to be added in */
		char_header->value_handle = ble_service->char_list[char_index].char_val.value_handle;
		char_header->type[0] = (uint8_t) (UUID_CHARACTERISTIC >> 8);
		char_header->type[1] = (uint8_t) (UUID_CHARACTERISTIC);
		char_header->type_length = BLE_ATTRIB_UUID_LENGTH_2;
		
		/* Calculate header length, based on UUID type */
		if(ble_service->char_list[char_index].char_val.uuid.type == BLE_UUID_16B)
		{
			/* Service uses 2 bytes UUID */
			char_header->length = sizeof(attrib_char_t) - 14;
			memcpy(char_header->uuid.uuid_16b, ble_service->char_list[char_index].char_val.uuid.uuid.uuid_16b, sizeof(char_header->uuid.uuid_16b));
		}
		else if(ble_service->char_list[char_index].char_val.uuid.type == BLE_UUID_128B)
		{
			/* Service uses 16 bytes UUID */
			char_header->length = sizeof(attrib_char_t);
			memcpy(char_header->uuid.uuid_128b, ble_service->char_list[char_index].char_val.uuid.uuid.uuid_128b, sizeof(char_header->uuid.uuid_128b));
		}
		
		service_buf_len += char_header->length;
		/* Exclude the length field in calculating overall length */
		char_header->length--;
		/* Counted a Characteristic */
		num_of_attrib++;
		
		/* Characteristic value */
		char_val_header = (attrib_char_value_t *)(service_buf + service_buf_len);
		char_val_header->permission = ble_service->char_list[char_index].char_val.permissions;
		char_val_header->start_handle = 0;
		if(ble_service->char_list[char_index].char_val.uuid.type == BLE_UUID_16B)
		{
			char_val_header->type_length = BLE_UUID_16B_LEN;
			memcpy(char_val_header->type, ble_service->char_list[char_index].char_val.uuid.uuid.uuid_16b, char_val_header->type_length);
		}
		else if(ble_service->char_list[char_index].char_val.uuid.type == BLE_UUID_128B)
		{
			char_val_header->type_length = BLE_UUID_128B_LEN;
			memcpy(char_val_header->type, ble_service->char_list[char_index].char_val.uuid.uuid.uuid_128b, char_val_header->type_length);
		}
		char_val_header->value_length = ble_service->char_list[char_index].char_val.len;
		memcpy(char_val_header->value, ble_service->char_list[char_index].char_val.init_value, char_val_header->value_length);
		/* Adjust the length based on actual length of value */
		char_val_header->length = sizeof(attrib_char_value_t) - (BLE_ATT_ATTRIBUTE_VALUE_LEN - char_val_header->value_length);
		
		service_buf_len += char_val_header->length;
		/* Exclude the length field in calculating overall length */
		char_val_header->length--;
		/* Counted a Characteristic value */
		num_of_attrib++;
		
		/* Client Characteristic Configuration Descriptor */
		if(ble_service->char_list[char_index].client_config_desc.ccd_included)
		{
			attrib_ccd_desc_t *ccd = (attrib_ccd_desc_t *)(service_buf + service_buf_len);
			if(service_buf_len > BLE_MAX_SERVICE_BUF_LEN)
			{
				return BLE_INSUFFICIENT_RESOURCES;
			}
			ccd->permission = ble_service->char_list[char_index].client_config_desc.perm;
			ccd->start_handle = ble_service->char_list[char_index].client_config_desc.handle;
			ccd->type_length = BLE_ATTRIB_UUID_LENGTH_2;
			ccd->type[0] = (uint8_t) (CLIENT_CHAR_CONFIGURATION >> 8);
			ccd->type[1] = (uint8_t) (CLIENT_CHAR_CONFIGURATION);
			ccd->ccd_value[0] = ble_service->char_list[char_index].client_config_desc.ccd_value >>8;
			ccd->ccd_value[1] = ble_service->char_list[char_index].client_config_desc.ccd_value;
			ccd->length = sizeof(attrib_ccd_desc_t);
			service_buf_len += ccd->length;
			/* Exclude the length field in calculating overall length */
			ccd->length--;
			num_of_attrib++;
		}
		
		/* Server Characteristic Configuration Descriptor */
		if(ble_service->char_list[char_index].server_config_desc.ccd_included)
		{
			attrib_ccd_desc_t *ccd = (attrib_ccd_desc_t *)(service_buf + service_buf_len);
			if(service_buf_len > BLE_MAX_SERVICE_BUF_LEN)
			{
				return BLE_INSUFFICIENT_RESOURCES;
			}
			ccd->permission = ble_service->char_list[char_index].server_config_desc.perm;
			ccd->start_handle = ble_service->char_list[char_index].server_config_desc.handle;
			ccd->type_length = BLE_ATTRIB_UUID_LENGTH_2;
			ccd->type[0] = (uint8_t) (SERVER_CHAR_CONFIGURATION >> 8);
			ccd->type[1] = (uint8_t) (SERVER_CHAR_CONFIGURATION);
			ccd->ccd_value[0] = ble_service->char_list[char_index].server_config_desc.ccd_value >>8;
			ccd->ccd_value[1] = ble_service->char_list[char_index].server_config_desc.ccd_value;
			ccd->length = sizeof(attrib_ccd_desc_t);
			service_buf_len += ccd->length;
			/* Exclude the length field in calculating overall length */
			ccd->length--;
			num_of_attrib++;
		}
		
		/* User Descriptor */
		if(ble_service->char_list[char_index].user_desc.ud_included)
		{
			attrib_user_desc_t *ud = (attrib_user_desc_t *)(service_buf + service_buf_len);
			if(service_buf_len > BLE_MAX_SERVICE_BUF_LEN)
			{
				return BLE_INSUFFICIENT_RESOURCES;
			}
			ud->start_handle = ble_service->char_list[char_index].user_desc.handle;
			ud->permission = ble_service->char_list[char_index].user_desc.permissions;
			ud->type_length = BLE_ATTRIB_UUID_LENGTH_2;
			ud->type[0] = (uint8_t) (CHAR_USER_DESCRIPTION >> 8);
			ud->type[1] = (uint8_t) (CHAR_USER_DESCRIPTION);
			ud->ud_length = ble_service->char_list[char_index].user_desc.len;
			memcpy(ud->user_desc, ble_service->char_list[char_index].user_desc.user_description, ble_service->char_list[char_index].user_desc.len);
			/* Calculate attribute length based on user descriptor length */
			ud->length = (sizeof(attrib_user_desc_t) - (BLE_ATT_ATTRIBUTE_VALUE_LEN - ud->ud_length));
			service_buf_len += ud->length;
			/* Exclude the length field in calculating overall length */
			ud->length--;
			num_of_attrib++;
		}
	
		/* Presentation Format */
		if(ble_service->char_list[char_index].presentation_format.pf_included)
		{
			attrib_pres_format_desc_t *pf = (attrib_pres_format_desc_t *)(service_buf + service_buf_len);
			if(service_buf_len > BLE_MAX_SERVICE_BUF_LEN)
			{
				return BLE_INSUFFICIENT_RESOURCES;
			}
			pf->type_length = BLE_ATTRIB_UUID_LENGTH_2;
			pf->type[0] = (uint8_t) (CHAR_PRESENTATION_FORMAT >> 8);
			pf->type[1] = (uint8_t) (CHAR_PRESENTATION_FORMAT);
			pf->start_handle = ble_service->char_list[char_index].presentation_format.handle;
			pf->permission = ble_service->char_list[char_index].presentation_format.permissions;
			pf->format = ble_service->char_list[char_index].presentation_format.format;
			pf->exponent = ble_service->char_list[char_index].presentation_format.exponent;
			pf->unit = ble_service->char_list[char_index].presentation_format.unit;
			pf->name_space = ble_service->char_list[char_index].presentation_format.name_space;
			pf->description = ble_service->char_list[char_index].presentation_format.description;
			pf->length = sizeof(attrib_pres_format_desc_t);
			service_buf_len += pf->length;
			/* Exclude the length field in calculating overall length */
			pf->length--;
			num_of_attrib++;
		}
		
		if(ble_service->char_list[char_index].ext_properties.ep_included)
		{
			attrib_ext_prop_desc_t *ep = (attrib_ext_prop_desc_t *)(service_buf + service_buf_len);
			if(service_buf_len > BLE_MAX_SERVICE_BUF_LEN)
			{
				return BLE_INSUFFICIENT_RESOURCES;
			}
			ep->start_handle = ble_service->char_list[char_index].ext_properties.handle;
			ep->permission = ble_service->char_list[char_index].ext_properties.permissions;
			ep->type_length = BLE_ATTRIB_UUID_LENGTH_2;
			ep->type[0] = (uint8_t) (CHAR_EXTENDED_PROPERTIES >> 8);
			ep->type[1] = (uint8_t) (CHAR_EXTENDED_PROPERTIES);
			ep->extend_property[0] = ble_service->char_list[char_index].ext_properties.ext_property >> 8;
			ep->extend_property[1] = ble_service->char_list[char_index].ext_properties.ext_property;
			ep->length = sizeof(attrib_ext_prop_desc_t);
			service_buf_len += ep->length;
			/* Exclude the length field in calculating overall length */
			ep->length--;
			num_of_attrib++;
		}
		
		
		for(desc_index = 0; desc_index < ble_service->char_list[char_index].additional_desc_count; desc_index++)
		{
			/* Other generic public descriptors */
			if(BLE_UUID_16B == ble_service->char_list[char_index].additional_desc_list[desc_index].desc_uuid.type)
			{
				ble_generic_att_public_desc_t *gen_att_public = (ble_generic_att_public_desc_t *)(service_buf + service_buf_len);
				if(service_buf_len > BLE_MAX_SERVICE_BUF_LEN)
				{
					return BLE_INSUFFICIENT_RESOURCES;
				}
				gen_att_public->handle = ble_service->char_list[char_index].additional_desc_list[desc_index].handle;
				gen_att_public->permission = ble_service->char_list[char_index].additional_desc_list[desc_index].permission;
				gen_att_public->type_length = BLE_UUID_16B_LEN;
				memcpy(gen_att_public->type, ble_service->char_list[char_index].additional_desc_list[desc_index].desc_uuid.uuid.uuid_16b, BLE_UUID_16B_LEN);
				
				if(ble_service->char_list[char_index].additional_desc_list[desc_index].desc_val_length > BLE_ATT_ATTRIBUTE_VALUE_LEN)
				{
					return BLE_INVALID_COMMAND_PARAMETERS;
				}
				
				gen_att_public->desc_val_length = ble_service->char_list[char_index].additional_desc_list[desc_index].desc_val_length;
				memcpy(gen_att_public->desc_value, ble_service->char_list[char_index].additional_desc_list[desc_index].desc_value, gen_att_public->desc_val_length);
				gen_att_public->length = sizeof(ble_generic_att_public_desc_t) - (BLE_ATT_ATTRIBUTE_VALUE_LEN - gen_att_public->desc_val_length);
				service_buf_len += gen_att_public->length;
				/* Exclude the length field in calculating overall length */
				gen_att_public->length--;
			}
			else if(BLE_UUID_128B == ble_service->char_list[char_index].additional_desc_list[desc_index].desc_uuid.type)
			{
				ble_generic_att_private_desc_t *gen_att_private = (ble_generic_att_private_desc_t *)(service_buf + service_buf_len);
				if(service_buf_len > BLE_MAX_SERVICE_BUF_LEN)
				{
					return BLE_INSUFFICIENT_RESOURCES;
				}
				gen_att_private->handle = ble_service->char_list[char_index].additional_desc_list[desc_index].handle;
				gen_att_private->permission = ble_service->char_list[char_index].additional_desc_list[desc_index].permission;
				gen_att_private->type_length = BLE_UUID_128B_LEN;
				memcpy(gen_att_private->type, ble_service->char_list[char_index].additional_desc_list[desc_index].desc_uuid.uuid.uuid_128b, BLE_UUID_128B_LEN);
				
				if(ble_service->char_list[char_index].additional_desc_list[desc_index].desc_val_length > BLE_ATT_ATTRIBUTE_VALUE_LEN)
				{
					return BLE_INVALID_COMMAND_PARAMETERS;
				}
				
				gen_att_private->desc_val_length = ble_service->char_list[char_index].additional_desc_list[desc_index].desc_val_length;
				memcpy(gen_att_private->desc_value, ble_service->char_list[char_index].additional_desc_list[desc_index].desc_value, gen_att_private->desc_val_length);
				gen_att_private->length = sizeof(ble_generic_att_private_desc_t) - (BLE_ATT_ATTRIBUTE_VALUE_LEN - gen_att_private->desc_val_length);
				service_buf_len += gen_att_private->length;
				/* Exclude the length field in calculating overall length */
				gen_att_private->length--;
			}
			else
			{
				return BLE_INVALID_COMMAND_PARAMETERS;
			}
			num_of_attrib++;
		}
	}
	
	if(service_buf_len > BLE_MAX_SERVICE_BUF_LEN)
	{
		return BLE_INSUFFICIENT_RESOURCES;
	}
	
	BM_APPLICATION_ServerCreateService(application_cmdpkt, (uint8_t *)service_header, service_buf_len, num_of_attrib);
	status = interface_cmd_send_wait(application_cmdpkt->cmdPkt, (uint32_t) application_cmdpkt->length, COMMAND_COMPLETE);
	
	if(BLE_SUCCESS == status)
	{
		/* Check the status in command complete event */
		status = ble_gatt_service_create_parser(&handle);
		/* Assign handles */
		if(BLE_SUCCESS == status)
		{
			/* Assign service handle */
			ble_service->handle = handle++;
			for(char_index = 0; char_index < ble_service->char_count; char_index++)
			{
				/* Assign characteristic handle */
				ble_service->char_list[char_index].char_val.char_handle = handle++;
				/* Assign characteristic value handle */
				ble_service->char_list[char_index].char_val.value_handle = handle++;
				/* Assign client configuration descriptor handle */
				if(ble_service->char_list[char_index].client_config_desc.ccd_included)
				{
					ble_service->char_list[char_index].client_config_desc.handle = handle++;
				}
				/* Assign server configuration descriptor handle */
				if(ble_service->char_list[char_index].server_config_desc.ccd_included)
				{
					ble_service->char_list[char_index].server_config_desc.handle = handle++;
				}
				/* Assign user descriptor handle */
				if(ble_service->char_list[char_index].user_desc.ud_included)
				{
					ble_service->char_list[char_index].user_desc.handle = handle++;
				}
				/* Assign presentation properties descriptor handle */
				if(ble_service->char_list[char_index].presentation_format.pf_included)
				{
					ble_service->char_list[char_index].presentation_format.handle = handle++;
				}
				/* Assign extended properties descriptor handle */
				if(ble_service->char_list[char_index].ext_properties.ep_included)
				{
					ble_service->char_list[char_index].ext_properties.handle = handle++;
				}
				
				/* Assign additional descriptor handles */
				for(desc_index = 0; desc_index < ble_service->char_list[char_index].additional_desc_count; desc_index++)
				{
					ble_service->char_list[char_index].additional_desc_list[desc_index].handle = handle++;
				}
			}
		}
	}
	
	return status;
}