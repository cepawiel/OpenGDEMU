/**
 * \file bm_application_mode.c
 *
 * \brief BM application mode definitions
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
#include "bm_utils.h"
#include "bm_mode.h"
#include "bm_application_mode.h"
#include "event_mem.h"
#include "ble_api.h"

static const BM_APPLICATION_CMD bm70ApplicationCmds[] =
{
    {BM_LOCAL_INFORMATION_READ,     	0x00},
    {BM_RESET,                          0x00},
    {BM_STATUS_READ,                	0x00},
    {BM_ADC_READ_INDEX,                 0x01},	
    {BM_SHUTDOWN,                       0x00},
    {BM_DEBUG,                          0xFF},
    {BM_NAME_READ,                  	0x00},
    {BM_NAME_WRITE,                     0x11},	
    {BM_PDL_ERASE,                  	0x00},
    {BM_PAIR_MODE_READ,             	0x00},
    {BM_PAIR_MODE_WRITE,            	0x02},    
    {BM_PDL_READ,                   	0x00},
    {BM_DEVICE_ERASE,                   0x01},    	
    {BM_DIO_CONTROL,                    0x0C},
    {BM_PWM_CONTROL,                    0x08},
    {BM_RSSI_READ,                      0x01},    
    {BM_ADV_DATA_WRITE,             	0x20},
    {BM_SCAN_DATA_WRITE,            	0x20},  
    {BM_ADV_PARAM_SET,                  0x0A},
	{BM_CONN_PARAM_SET,					0x08},
    {BM_SCAN_PARAM_SET,                 0x05},
    {BM_SCAN_ENABLE_SET,                0x02},
    {BM_CONNECT,                        0x08},
    {BM_CONNECT_CANCEL,                 0x00},
    {BM_CONNECTION_PARAM_UPDATE,        0x07},
    {BM_DISCONNECT,                     0x01},
    {BM_SET_ADV_ENABLE,                 0x01},		
    {BM_REMOTE_NAME_READ,               0x01},
    {BM_CLIENT_WRITE_REQUEST_PREPARE,   0xF8},
    {BM_CLIENT_WRITE_REQUEST_EXECUTE,   0x03},
    {BM_CLIENT_BLOB_REQUEST_READ,       0x05},
    {BM_CLIENT_HANDLE_VALUE_CONFIRM,    0x01},	
    {BM_CLIENT_DISCOVER_ALL_SERVICES,	0x01},
    {BM_CLIENT_DISCOVER_CHARACTERISTICS,0x11},
    {BM_CLIENT_CHARACTERISTIC_READ,     0x03},
    {BM_CLIENT_CHARACTERISTIC_UUID_READ,0x11},
    {BM_CLIENT_CHARACTERISTIC_WRITE,	0x18},
	{BM_TRANSPARENT_ENABLE,             0x03},
	{BM_SERVER_CREATE_SERVICE_REQUEST,  0x01FF},
    {BM_SERVER_WRITE_RESPONSE_PREPARE,  0xF8},
    {BM_SERVER_WRITE_RESPONSE_EXECUTE,  0x02},
    {BM_SERVER_BLOB_RESPONSE_READ,      0xF8},
    {BM_SERVER_ERROR_RESPONSE,          0x05},
    {BM_SERVER_CHARACTERISTIC_SEND,     0x17},
    {BM_SERVER_CHARACTERISTIC_UPDATE,	0x16},
    {BM_SERVER_CHARACTERISTIC_READ,     0x02},
    {BM_SERVER_ALL_SERVICES_READ,       0x00},
    {BM_SERVER_SERVICE_READ,            0x10},
    {BM_SERVER_WRITE_RESPONSE_SEND,     0x05},
    {BM_SERVER_READ_RESPONSE_SEND,      0xF8},
	{BM_TRANSPARENT_DATA_SEND,          0xFF},
    {BM_PASSKEY_ENTRY,              	0x03},
    {BM_PASSKEY_YESNO_CONFIRM,          0x02},
    {BM_PAIRING_REQUEST,                0x01},
    {BM_CONFIG_MODE_CLOSE,          	0x01},
    {BM_RECEIVER_TEST_INDEX,            0x00},
	{BM_TRANSMITTER_TEST_INDEX,         0x00},
	{BM_END_TEST_INDEX,                 0x00},
    {BM_PATTERN_SET,                    0x01},
};


static uint16_t                      appDataIndex = 0;
static BM_APPLICATION_CMD_STATE     appCmdState = BM_CFG_CMD_SOF;
static BM_APPLICATION_CMD*          appCmds;
uint8_t                             appRspData[BM_PKT_LEN_MAX];
uint8_t                             appDataCmdPkt[BM_PKT_LEN_MAX];
uint8_t                             appByteTimeout = 0;
event_t								appEvent = {0};
BM_APPLICATION_CMDPKT               appCmdPkt = {0, appDataCmdPkt};

static bool BM_APPLICATION_Response(void);
static void BM_APPLICATION_CmdPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, BM_APPLICATION_CMD* applicationCmd, uint8_t* data);


BM_APPLICATION_CMDPKT* BM_APPLICATION_Init()
{
    appCmdPkt.cmdPkt = appDataCmdPkt;
    appCmdPkt.length = 0;
    appEvent.event_id = BM_ERROR;
    appEvent.event_msg.data = NULL;
    appEvent.event_msg.data_len = 0;
    appCmds = (BM_APPLICATION_CMD*) bm70ApplicationCmds;
    return &appCmdPkt;
}

void BM_APPLICATION_DeInit()
{
}

static void BM_APPLICATION_CmdPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, BM_APPLICATION_CMD* applicationCmd, uint8_t* data)
{
	uint8_t* cmdPkt = applicationCmdPkt->cmdPkt;
	applicationCmdPkt->length = (5 + applicationCmd->length);

    *cmdPkt++ = 0xAA;
    *cmdPkt++ = (uint8_t)((applicationCmd->length + 1) >> 8);
    *cmdPkt++ = (uint8_t)(applicationCmd->length + 1);
    *cmdPkt++ = applicationCmd->opcode;
    if(data != NULL)
    {
        memcpy(cmdPkt, data, applicationCmd->length);
        cmdPkt += (applicationCmd->length);
    }
    *cmdPkt = calc_chksum_8bit(applicationCmdPkt->cmdPkt + 1, applicationCmd->length + 3);
}


bool BM_APPLICATION_ResponseCheck(uint8_t* data, uint8_t len)
{
	
	while(len--)
	{
		switch(appCmdState)
		{
			case BM_CFG_CMD_SOF:
			if(*data == BM_CFG_CMD_SOF_VAL)
			{
				appRspData[BM_CFG_CMD_SOF] = *data;
				appEvent.event_id = BM_ERROR;
				appEvent.event_msg.data = NULL;
				appEvent.event_msg.data_len = 0;
				appCmdState = BM_CFG_CMD_LENH;
			}
			break;

			case BM_CFG_CMD_LENH:
			{
				appRspData[BM_CFG_CMD_LENH] = *data;
				appEvent.event_msg.data_len = ((uint16_t)*data) << 8;
				appCmdState = BM_CFG_CMD_LENL;
			}
			break;

			case BM_CFG_CMD_LENL:
			{
				appRspData[BM_CFG_CMD_LENL] = *data;
				appEvent.event_msg.data_len |= (uint16_t)*data;
				appCmdState = BM_CFG_CMD_OPCODE;
			}
			break;

			case BM_CFG_CMD_OPCODE:
			{
				appRspData[BM_CFG_CMD_OPCODE] = *data;
				appEvent.event_id = *data;
				appCmdState = BM_CFG_CMD_DATA;
			}
			break;

			case BM_CFG_CMD_DATA:
			{
				appRspData[BM_CFG_CMD_DATA+appDataIndex] = *data;
				appDataIndex++;
				if(appDataIndex >= (appEvent.event_msg.data_len - 1))
				{
					appDataIndex = 0;
					appCmdState = BM_CFG_CMD_CHKSUM;
				}
			}
			break;

			case BM_CFG_CMD_CHKSUM:
			{
				bool result;
				appRspData[BM_CFG_CMD_DATA + (((appRspData[BM_CFG_CMD_LENH] << 8) | appRspData[BM_CFG_CMD_LENL]) - 1)] = *data;
				result = BM_APPLICATION_Response();
				appCmdState = BM_CFG_CMD_SOF;
				return result;
			}
			break;

			default:
			break;

		}
		if(len != 0)
		data++;
	}

	return false;
}

static bool BM_APPLICATION_Response(void)
{
    uint8_t chksum = calc_chksum_8bit(appRspData + BM_CFG_CMD_LENH, (((appRspData[BM_CFG_CMD_LENH] << 8) | appRspData[BM_CFG_CMD_LENL]) + 2));
    if(*(appRspData + BM_CFG_CMD_DATA + (((appRspData[BM_CFG_CMD_LENH] << 8) | appRspData[BM_CFG_CMD_LENL]) - 1)) != chksum)
    {
        appEvent.event_id = BM_ERROR;
        appEvent.event_msg.data = NULL;
        appEvent.event_msg.data_len = 0;
		return false;
    }
    else
    {
        appEvent.event_msg.data = (appRspData + BM_CFG_CMD_DATA);
    }
	return true;
}

void BM_APPLICATION_PatternSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t pattern)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PATTERN_SET_INDEX), &pattern);
}

void BM_APPLICATION_ResetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_RESET_INDEX), NULL);
}

void BM_APPLICATION_LocalInformationReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_LOCAL_INFORMATION_READ_INDEX), NULL);
}

void BM_APPLICATION_StatusReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_STATUS_READ_INDEX), NULL);
}

void BM_APPLICATION_ADCReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t adcChannel)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_ADC_READ_INDEX), &adcChannel);
}

void BM_APPLICATION_ShutdownPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_SHUTDOWN_INDEX), NULL);
}

void BM_APPLICATION_DebugPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t debugOpcode, uint8_t* debugParams, uint8_t debugLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_TRANSPARENT_DATA_SEND_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_DEBUG_INDEX)];
    *payload = debugOpcode;
    memcpy((void*)(payload+1), debugParams, debugLength);
    appCmd.length = debugLength + 1; 
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_DeviceNameReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_NAME_READ_INDEX), NULL);
}

void BM_APPLICATION_DeviceNameWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* deviceName, uint8_t deviceNameLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_NAME_WRITE_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_NAME_WRITE_INDEX)];
    *payload = 0x00;
    if (deviceNameLength >= BM_APPLICATION_MODE_CMD_LEN(BM_NAME_WRITE_INDEX)-1)
    {
        memcpy(payload+1, deviceName, BM_APPLICATION_MODE_CMD_LEN(BM_NAME_WRITE_INDEX)-1);
    } 
    else
    {
        memcpy(payload+1, deviceName, deviceNameLength);
        appCmd.length = deviceNameLength + 1;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_PairDeviceListErasePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PDL_ERASE_INDEX), NULL);
}

void BM_APPLICATION_PairModeReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PAIR_MODE_READ_INDEX), NULL);
}

void BM_APPLICATION_PairModeWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, BM_PAIR_MODE pairMode)
{
    uint16_t payload = (pairMode << 8);
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PAIR_MODE_WRITE_INDEX), (uint8_t*)&payload);
}

void BM_APPLICATION_PairDeviceListReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PDL_READ_INDEX), NULL);
}

void BM_APPLICATION_DIOControlPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* dioControl)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_DIO_CONTROL_INDEX), dioControl);
}

void BM_APPLICATION_PWMControlPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* pwmControl)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PWM_CONTROL_INDEX), pwmControl);
}

void BM_APPLICATION_PairDeviceDeletePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t deviceIndex)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_DEVICE_ERASE_INDEX), &deviceIndex);
}

void BM_APPLICATION_RSSIReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_RSSI_READ), &connectionHandle);
}

void BM_APPLICATION_AdvertisementDataWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* advertisementData, uint8_t advertisementLength, BM_ADV_TYPE_STORE store)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_ADV_DATA_WRITE_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_ADV_DATA_WRITE_INDEX)];
    *payload = store;
    if (advertisementLength >= BM_APPLICATION_MODE_CMD_LEN(BM_ADV_DATA_WRITE_INDEX)-1)
    {
        memcpy(payload+1, advertisementData, BM_APPLICATION_MODE_CMD_LEN(BM_ADV_DATA_WRITE_INDEX)-1);
    }
    else
    {
        memcpy(payload+1, advertisementData, advertisementLength);
        appCmd.length = advertisementLength + 1;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_AdvertisementParameterSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* advertisementParameter) 
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_ADV_PARAM_SET_INDEX), (uint8_t*)advertisementParameter);
}

void BM_APPLICATION_AdvertisementEnableSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t advertisementMode) 
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_SET_ADV_ENABLE_INDEX), &advertisementMode);
}

void BM_APPLICATION_ScanDataWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* scanData, uint8_t scanLength, bool store)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_SCAN_DATA_WRITE_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SCAN_DATA_WRITE_INDEX)];
    *payload = store;
    if (scanLength >= BM_APPLICATION_MODE_CMD_LEN(BM_SCAN_DATA_WRITE_INDEX)-1)
    {
        memcpy(payload+1, scanData, BM_APPLICATION_MODE_CMD_LEN(BM_SCAN_DATA_WRITE_INDEX)-1);
    }
    else
    {
        memcpy(payload+1, scanData, scanLength);
        appCmd.length = scanLength + 1;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ScanParameterSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint16_t scanInterval, uint16_t scanWindow, uint8_t scanType)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SCAN_PARAM_SET_INDEX)];
    memcpy_reorder(payload, (uint8_t*)&scanInterval, 2);
    memcpy_reorder(payload+2, (uint8_t*)&scanWindow, 2);
    *(payload+4) = scanType;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_SCAN_PARAM_SET_INDEX), payload);
}

void BM_APPLICATION_ScanEnableSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t scanEnable, uint8_t filterPolicy)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SCAN_ENABLE_SET_INDEX)];
    payload[0] = scanEnable;
    payload[1] = filterPolicy;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_SCAN_ENABLE_SET_INDEX), payload);
}

void BM_APPLICATION_ConnParameterSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint16_t min_conn_interval, uint16_t max_conn_interval, uint16_t slave_latency, uint16_t so_timeout)
{
	uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_CONN_PARAM_SET_INDEX)];
	payload[0] = min_conn_interval >> 8;
	payload[1] = min_conn_interval;
	payload[2] = max_conn_interval >> 8;
	payload[3] = max_conn_interval;
	payload[4] = slave_latency >> 8;
	payload[5] = slave_latency;
	payload[6] = so_timeout >> 8;
	payload[7] = so_timeout;
	BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CONN_PARAM_SET_INDEX), payload);
}

void BM_APPLICATION_ConnectDevicePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* peerAddress, uint8_t peerAddressType, uint8_t filterPolicy)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_CONNECT_INDEX)];
    payload[0] = filterPolicy;
    payload[1] = peerAddressType;
    memcpy(payload+2, peerAddress, 6);
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CONNECT_INDEX), payload);
}

void BM_APPLICATION_ConnectDeviceCancelPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CONNECT_CANCEL_INDEX), NULL);
}

void BM_APPLICATION_ConnectionParameterUpdatePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t connectionInterval, uint16_t connectionLatency, uint16_t supervisionTimeout)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_CONNECTION_PARAM_UPDATE_INDEX)];
    *payload = connectionHandle;
    memcpy_reorder(payload+1, (uint8_t*)&connectionInterval, 2);
    memcpy_reorder(payload+3, (uint8_t*)&connectionLatency, 2);    
    memcpy_reorder(payload+5, (uint8_t*)&supervisionTimeout, 2);  
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CONNECTION_PARAM_UPDATE_INDEX), payload);
}

void BM_APPLICATION_DisconnectDevicePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle)
{	
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_DISCONNECT_INDEX), &connectionHandle);
}

void BM_APPLICATION_RemoteDeviceNameReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle)
{	
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_REMOTE_NAME_READ_INDEX), &connectionHandle);
}

void BM_APPLICATION_ClientAllServicesDiscoverPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CLIENT_DISCOVER_ALL_SERVICES_INDEX), &connectionHandle);
}
    
void BM_APPLICATION_ClientCharacteristicsDiscoverPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t* serviceUUID, uint8_t serviceUUIDLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_CLIENT_DISCOVER_CHARACTERISTICS_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_DISCOVER_CHARACTERISTICS_INDEX)];
    *payload = connectionHandle;
    if (serviceUUIDLength >= BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_DISCOVER_CHARACTERISTICS_INDEX)-1)
    {
        memcpy(payload+1, serviceUUID, BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_DISCOVER_CHARACTERISTICS_INDEX)-1);
    }
    else
    {
        memcpy(payload+1, serviceUUID, serviceUUIDLength);
        appCmd.length = serviceUUIDLength + 1;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ClientCharacteristicHandleReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteriticHandle)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_CHARACTERISTIC_READ_INDEX)];
    *payload = connectionHandle;
    memcpy(payload+1, &characteriticHandle, BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_CHARACTERISTIC_READ_INDEX)-1);
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CLIENT_CHARACTERISTIC_READ_INDEX), payload);
}

void BM_APPLICATION_ClientCharacteristicUUIDReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t* characteriticUUID, uint8_t characteriticUUIDLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_CLIENT_CHARACTERISTIC_UUID_READ_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_CHARACTERISTIC_UUID_READ_INDEX)];
    *payload = connectionHandle;
    if (characteriticUUIDLength >= BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_CHARACTERISTIC_UUID_READ_INDEX)-1)
    {
        memcpy(payload+1, characteriticUUID, BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_CHARACTERISTIC_UUID_READ_INDEX)-1);
    }
    else 
    {
        memcpy(payload+1, characteriticUUID, characteriticUUIDLength);
        appCmd.length = characteriticUUIDLength + 1;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ClientCharacteristicHandleWritePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteristicHandle, uint8_t writeType, uint8_t* characteriticValue, uint8_t characteriticValueLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_CLIENT_CHARACTERISTIC_WRITE_INDEX);
    uint8_t payload[0x18]; /* BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_CHARACTERISTIC_WRITE_INDEX)]; */
    *payload = connectionHandle;
    memcpy(payload+1, &writeType, 1);
    memcpy_reorder(payload+2, (uint8_t*)&characteristicHandle, 2);
    if (characteriticValueLength >= /*BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_CHARACTERISTIC_WRITE_INDEX)*/0x18-4)
    {
        memcpy(payload+4, characteriticValue, /*BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_CHARACTERISTIC_WRITE_INDEX)*/0x18-4);
    }
    else
    {
        memcpy(payload+4, characteriticValue, characteriticValueLength);
        appCmd.length = characteriticValueLength + 4;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ClientWriteRequestPreparePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteristicHandle, uint8_t writeType, uint16_t valueOffset, uint8_t* characteriticValue, uint8_t characteriticValueLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_CLIENT_WRITE_REQUEST_PREPARE_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_WRITE_REQUEST_PREPARE_INDEX)];
    *payload = connectionHandle;
    memcpy(payload+1, &writeType, 1);
    memcpy(payload+2, &characteristicHandle, 2);
    if (characteriticValueLength >= BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_WRITE_REQUEST_PREPARE_INDEX)-4)
    {
        memcpy(payload+4, characteriticValue, BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_WRITE_REQUEST_PREPARE_INDEX)-4);
    }
    else
    {
        memcpy(payload+4, characteriticValue, characteriticValueLength);
        appCmd.length = characteriticValueLength + 4;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ClientWriteRequestExecutePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t writeType, uint8_t flag)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_WRITE_REQUEST_EXECUTE_INDEX)];
    payload[0] = connectionHandle;
    payload[1] = writeType;
    payload[2] = flag;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CLIENT_WRITE_REQUEST_EXECUTE_INDEX), payload);
}

void BM_APPLICATION_ClientBlobRequestReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteristicHandle, uint16_t valueOffset)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_CLIENT_BLOB_REQUEST_READ_INDEX)];
    *payload = connectionHandle;
    memcpy(payload+1, &characteristicHandle, 2);
    memcpy(payload+3, &valueOffset, 2);
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CLIENT_BLOB_REQUEST_READ_INDEX), payload);
}

void BM_APPLICATION_ClientHandleValueConfirmPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CLIENT_HANDLE_VALUE_CONFIRM_INDEX), &connectionHandle);
}

void BM_APPLICATION_ServerCharacteristicSendPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteriticHandle, uint8_t* characteriticValue, uint8_t characteriticValueLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_SERVER_CHARACTERISTIC_SEND_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_CHARACTERISTIC_SEND_INDEX)];
    payload[0] = connectionHandle;
    memcpy_reorder(payload+1, (uint8_t*)&characteriticHandle, 2);
    if (characteriticValueLength >= BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_CHARACTERISTIC_SEND_INDEX)-3)
    {
        memcpy(payload+3, characteriticValue, BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_CHARACTERISTIC_SEND_INDEX)-3);
    }
    else
    {
        memcpy(payload+3, characteriticValue, characteriticValueLength);
        appCmd.length = characteriticValueLength + 3;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ServerCharacteristicUpdatePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint16_t characteriticHandle, uint8_t* characteriticValue, uint8_t characteriticValueLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_SERVER_CHARACTERISTIC_UPDATE_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_CHARACTERISTIC_UPDATE_INDEX)];
    memcpy(payload, &characteriticHandle, 2);
    if (characteriticValueLength >= BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_CHARACTERISTIC_UPDATE_INDEX)-2)
    {
        memcpy(payload+2, characteriticValue, BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_CHARACTERISTIC_UPDATE_INDEX)-2);
    }
    else
    {
        memcpy(payload+2, characteriticValue, characteriticValueLength);
        appCmd.length = characteriticValueLength + 2;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ServerCharacteristicReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint16_t characteriticHandle)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_SERVER_CHARACTERISTIC_READ_INDEX), (uint8_t*)&characteriticHandle);
}

void BM_APPLICATION_ServerPrimaryServicesReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_SERVER_ALL_SERVICES_READ_INDEX), NULL);
}

void BM_APPLICATION_ServerPrimaryServiceReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t* serviceUUID, uint8_t serviceUUIDLength)
{  
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_SERVER_SERVICE_READ_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_SERVICE_READ_INDEX)];
    if (serviceUUIDLength >= BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_SERVICE_READ_INDEX))
    {
        memcpy_reorder(payload, serviceUUID, BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_SERVICE_READ_INDEX));
    }
    else
    {
        memcpy_reorder(payload, serviceUUID, serviceUUIDLength);
        appCmd.length = serviceUUIDLength;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ServerWriteResponseSendPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t requestOpcode, uint16_t attributeHandle, uint8_t errorCode)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_WRITE_RESPONSE_SEND_INDEX)];
    payload[0] = connectionHandle;
    payload[1] = requestOpcode;
    memcpy((void*)(payload+2), (const void*)&attributeHandle, 2);
    payload[4] = errorCode;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_SERVER_WRITE_RESPONSE_SEND_INDEX), payload);
}

void BM_APPLICATION_ServerReadResponseSendPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t responseType, uint8_t* characteriticValue, uint8_t characteriticValueLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_SERVER_READ_RESPONSE_SEND_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_READ_RESPONSE_SEND_INDEX)];
    payload[0] = connectionHandle;
    payload[1] = responseType;
    if (characteriticValueLength >= BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_READ_RESPONSE_SEND_INDEX)-2)
    {
        memcpy(payload+2, characteriticValue, BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_READ_RESPONSE_SEND_INDEX)-2);
    }
    else 
    {
        memcpy(payload+2, characteriticValue, characteriticValueLength);
        appCmd.length = characteriticValueLength + 2;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ServerWriteResponsePreparePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint16_t characteristicHandle, uint8_t writeType, uint16_t valueOffset, uint8_t* characteriticValue, uint8_t characteriticValueLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_CLIENT_WRITE_REQUEST_PREPARE_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_WRITE_RESPONSE_PREPARE_INDEX)];
    *payload = connectionHandle;
    memcpy(payload+1, &writeType, 1);
    memcpy(payload+2, &characteristicHandle, 2);
    if (characteriticValueLength >= BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_WRITE_RESPONSE_PREPARE_INDEX)-4)
    {
        memcpy(payload+4, characteriticValue, BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_WRITE_RESPONSE_PREPARE_INDEX)-4);
    }
    else 
    {
        memcpy(payload+4, characteriticValue, characteriticValueLength);
        appCmd.length = characteriticValueLength + 4;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ServerWriteExecutePreparePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t writeType)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_WRITE_RESPONSE_EXECUTE_INDEX)];
    payload[0] = connectionHandle;
    payload[1] = writeType;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_SERVER_WRITE_RESPONSE_EXECUTE_INDEX), payload);
}

void BM_APPLICATION_ServerBlobResponseReadPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t responseOpcode, uint8_t* characteriticValue, uint8_t characteriticValueLength)
{
    BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_SERVER_BLOB_RESPONSE_READ_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_BLOB_RESPONSE_READ_INDEX)];
    *payload = connectionHandle;
    *(payload+1) = responseOpcode;
    if (characteriticValueLength >= BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_BLOB_RESPONSE_READ_INDEX)-3)
    {
        memcpy(payload+2, characteriticValue, BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_BLOB_RESPONSE_READ_INDEX)-3);
    }
    else
    {
        memcpy(payload+2, characteriticValue, characteriticValueLength);
        appCmd.length = characteriticValueLength + 2;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ServerErrorResponsePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t requestOpcode, uint16_t characteristicHandle, uint8_t errorCode)
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_ERROR_RESPONSE_INDEX)];
    *payload = connectionHandle;
    *(payload+1) = requestOpcode;
    memcpy(payload+2, &characteristicHandle, 2);
    *(payload+4) = errorCode;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_SERVER_ERROR_RESPONSE_INDEX), payload);
}

void BM_APPLICATION_EnableTransparentUartSetPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t serverControl, uint8_t clientMode) 
{
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_TRANSPARENT_ENABLE_INDEX)];
    payload[0] = connectionHandle;
    payload[1] = serverControl;
    payload[2] = clientMode;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_TRANSPARENT_ENABLE_INDEX), payload);
}

void BM_APPLICATION_SendTransparentDataPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t* data, uint8_t datalen)
{
	BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_TRANSPARENT_DATA_SEND_INDEX);
    uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_TRANSPARENT_DATA_SEND_INDEX)];
	appCmd.length = datalen+1;
    payload[0] = connectionHandle;
    memcpy(payload+1, data, datalen);
	BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_APPLICATION_ConfigureModeClosePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, bool store)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_CONFIG_MODE_CLOSE_INDEX), (uint8_t*)&store);
}

void BM_APPLICATION_PairRequest(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle)
{
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PAIRING_REQUEST), (uint8_t*)&connectionHandle);
}

void BM_APPLICATION_PairPassKeyDigitEnterPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, uint8_t passkeyDigit)
{
    uint32_t param = 0;
    param = ((uint32_t)passkeyDigit << 16) | (BM_PASSKEY_DIGIT_ENTER << 8) | connectionHandle;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PASSKEY_ENTRY_INDEX), (uint8_t*)&param);
}

void BM_APPLICATION_PairPassKeyDigitErasePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle)
{
    uint16_t param = 0;
    param = (BM_PASSKEY_DIGIT_ERASE << 8) | connectionHandle;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PASSKEY_ENTRY_INDEX), (uint8_t*)&param);
}
void BM_APPLICATION_PairPassKeyClearPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle)
{
    uint16_t param = 0;
    param = (BM_PASSKEY_CLEAR << 8) | connectionHandle;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PASSKEY_ENTRY_INDEX), (uint8_t*)&param);
}

void BM_APPLICATION_PairPassKeyEntryCompletePkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle)
{
    uint16_t param = 0;
    param = (BM_PASSKEY_ENTRY_COMPLETE << 8) | connectionHandle;
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PASSKEY_ENTRY_INDEX), (uint8_t*)&param);
}

void BM_APPLICATION_PairYesNoConfirmPkt(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t connectionHandle, bool confirm)
{
    uint16_t param = 0x00;
    if(!confirm)
    {
    	param = (0x01 << 8) | connectionHandle;
    }
    else
    {
        param = connectionHandle;
    }
    BM_APPLICATION_CmdPkt(applicationCmdPkt, &BM_APPLICATION_MODE_CMD(BM_PASSKEY_YESNO_CONFIRM_INDEX), (uint8_t*)&param);
}

void BM_APPLICATION_ServerCreateService(BM_APPLICATION_CMDPKT* applicationCmdPkt, uint8_t *service, uint16_t length, uint8_t num_of_attrib)
{
	BM_APPLICATION_CMD appCmd = BM_APPLICATION_MODE_CMD(BM_SERVER_CREATE_SERVICE_REQUEST_INDEX);
	uint8_t payload[BM_APPLICATION_MODE_CMD_LEN(BM_SERVER_CREATE_SERVICE_REQUEST_INDEX)];
	
	/* Total number of attribute in this service */
	payload[0] = num_of_attrib;
	payload[1] = length >> 8;
	payload[2] = length;
	memcpy(&payload[3], service, length);
	appCmd.length = length + 3;
	BM_APPLICATION_CmdPkt(applicationCmdPkt, &appCmd, payload);
}

void BM_Application_EventParser(BM_APPLICATION_EVENT_OPCODE event_id, void *data, uint32_t *dataLen)
{
	switch(event_id)
	{
		/* GAP events */
		case BM_ADVERTISING_REPORT:
		{
			BM_APPLICATION_AdvReportEventParser(data, dataLen);
			break;
		}
		case BM_LE_CONNECT_COMPLETE:
		{
			BM_APPLICATION_ConnCompleteEventParser(data, dataLen);
			break;
		}
		case BM_DISCONNECT_COMPLETE:
		{
			/* No changes required */
			break;
		}
		case BM_CONNECTION_PARAMTER_UPDATE:
		{
			BM_APPLICATION_ConnParamUpdateEventParser(data, dataLen);
			break;
		}
		case BM_COMMAND_COMPLETE:
		{
			
			break;
		}
		case BM_STATUS_REPORT:
		{
			/* No changes required */
			break;
		}
		case BM_LE_END_TEST_RESULT:
		{
			BM_APPLICATION_LeEndTestResultEventParser(data, dataLen);
			break;
		}
		case BM_CONFIGURE_MODE_STATUS:
		{
			/* No changes required */
			break;
		}
		case BM_SERVER_CHARACTERICTIC_VALUE_WRITE:
		{
			BM_APPLICATION_WriteCharValueEventParser(data, dataLen);
			break;
		}
		case BM_CLIENT_DISCOVER_CHARACTERISTICS_RESULT:
		{
			
			break;
		}
		default:
			return;
	}
}

void BM_APPLICATION_AdvReportEventParser(void *data, uint32_t *dataLen)
{
	ble_adv_report_event_t adv_report = {0};
	uint8_t *data_ptr = (uint8_t *)data;
	/* Rearrange the data order, based on ble_adv_report_event_t */
	/* Copy advertisement event type */
	adv_report.adv_event_type = *((ble_adv_event_type_t*)data_ptr + ADV_REPORT_EVENT_TYPE_START);
	/* Copy address */
	memcpy(&adv_report.addr, data_ptr + ADV_REPORT_ADDRESS_TYPE_START, ADV_REPORT_ADDRESS_TYPE_LEN + ADV_REPORT_ADDRESS_LEN);
	/* Copy RSSI. Find the RSSI based on advertisement data len */
	adv_report.rssi = *(data_ptr + ADV_REPORT_DATA_START + *(data_ptr + ADV_REPORT_DATA_LENGTH_START));
	/* Copy advertisement data length */
	adv_report.data_len = *(data_ptr + ADV_REPORT_DATA_LENGTH_START);
	/* Copy advertisement data */
	memcpy(adv_report.data, (data_ptr + ADV_REPORT_DATA_START), *(data_ptr + ADV_REPORT_DATA_LENGTH_START));
	
	/* Copy the data back on original location */
	memcpy(data, &adv_report, *dataLen);
}

void BM_APPLICATION_ConnCompleteEventParser(void *data, uint32_t *dataLen)
{
	ble_conn_complete_event_t *conn_complete = (ble_conn_complete_event_t *)data;
	
	memcpy_inplace_reorder((uint8_t *)&conn_complete->conn_param.conn_interval, sizeof(conn_complete->conn_param.conn_interval));
	memcpy_inplace_reorder((uint8_t *)&conn_complete->conn_param.conn_latency, sizeof(conn_complete->conn_param.conn_latency));
	memcpy_inplace_reorder((uint8_t *)&conn_complete->conn_param.link_sv_to, sizeof(conn_complete->conn_param.link_sv_to));
	
	return;
}

void BM_APPLICATION_ConnParamUpdateEventParser(void *data, uint32_t *dataLen)
{
	ble_conn_param_update_event_t *conn_param_update = (ble_conn_param_update_event_t *)data;
	
	memcpy_inplace_reorder((uint8_t *)&conn_param_update->conn_param.conn_interval, sizeof(conn_param_update->conn_param.conn_interval));
	memcpy_inplace_reorder((uint8_t *)&conn_param_update->conn_param.conn_latency, sizeof(conn_param_update->conn_param.conn_latency));
	memcpy_inplace_reorder((uint8_t *)&conn_param_update->conn_param.link_sv_to, sizeof(conn_param_update->conn_param.link_sv_to));
	
	return;
}

void BM_APPLICATION_LeEndTestResultEventParser(void *data, uint32_t *dataLen)
{
	ble_end_test_result_event_t *le_end_test_result = (ble_end_test_result_event_t *)data;
	
	memcpy_inplace_reorder((uint8_t *)&le_end_test_result->num_of_packets, sizeof(le_end_test_result->num_of_packets));
	
	return;
}

void BM_APPLICATION_WriteCharValueEventParser(void *data, uint32_t *dataLen)
{
	ble_write_char_value_event_t *write_char_value = (ble_write_char_value_event_t *)data;
	
	memcpy_inplace_reorder((uint8_t *)&write_char_value->char_value_handle, sizeof(write_char_value->char_value_handle));
	
	return;
}
