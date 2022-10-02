/**
* \file otau.c
*
* \brief OTAU implementation
*
* Copyright (c) 2018 - 2019 Microchip Technology Inc. and its subsidiaries.
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
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
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
*
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

#if defined(OTAU_ENABLED)
#include "compiler.h"
#include "asf.h"
#include "otau.h"
#include "system.h"
#include "sysTimer.h"

#ifdef OTAU_SERVER
#include "otau_parser.h"
#else
#ifndef OTAU_USE_EXTERNAL_MEMORY
#include "common_nvm.h"
#else
#if (BOARD == SAMR30_MODULE_XPLAINED_PRO)
#include "at25dfx.h"
#include "conf_at25dfx.h"
#endif
#endif
#endif

#include "miwi_api.h"
#include "mimem.h"
#include "miqueue.h"

#ifdef OTAU_SERVER
#include "otau_parser.h"
#endif
#include "otau_notify.h"
#include "otau_upgrade.h"
#include "otau_debug.h"

#if defined(OTAU_PHY_MODE)
#include "phy.h"
#include "mimac_at86rf.h"
#endif

static void otauDataInd(RECEIVED_MESH_MESSAGE *ind);
static void otauDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer);

static void initDataHandleTable(void);
static bool addDataHandleTableEntry(uint8_t domainId, uint8_t messageId, addr_mode_t addrMode, uint8_t *addr, uint8_t handle);
static bool getAddressFromDataHandleTable(uint8_t handle, uint8_t *domainId, uint8_t *messageId, uint8_t *addrMode, uint8_t *addr);

#if defined(OTAU_PHY_MODE)
static void otauPhyDataInd(PHY_DataInd_t *ind);
static void otauPhyDataConf(uint8_t status);
#endif

static SYS_Timer_t otauNotifyTimer;
static SYS_Timer_t otauUpgradeTimer;
static SYS_Timer_t otauDebugTimer;

static bool otauInited = false;

uint16_t shortAddress;
uint16_t pan_id = MY_PAN_ID;

uint8_t seq_no = 0;

MiQueue_t networkFrame;

DataHandleTable_t dataHandleTable[DATA_HANDLE_TABLE_SIZE];

static uint8_t msgHandle = 0;

#ifndef OTAU_SERVER
node_address_t serverAddress;
#endif

#if defined(OTAU_PHY_MODE)
uint8_t otauPhyData[128];
static uint8_t storage_buffer[140];
#endif

#if !defined(OTAU_SERVER)
#ifdef OTAU_USE_EXTERNAL_MEMORY
#if (BOARD == SAMR30_MODULE_XPLAINED_PRO)

struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;

void serialFlashInit(void)
{
    /* configuration instances */
    struct at25dfx_chip_config at25dfx_chip_config_struct;
    struct spi_config at25dfx_spi_config;

    /* SPI Setup */
    at25dfx_spi_get_config_defaults(&at25dfx_spi_config);
    at25dfx_spi_config.mode_specific.master.baudrate = AT25DFX_CLOCK_SPEED;
    at25dfx_spi_config.mux_setting = AT25DFX_SPI_PINMUX_SETTING;
    at25dfx_spi_config.pinmux_pad0 = AT25DFX_SPI_PINMUX_PAD0;
    at25dfx_spi_config.pinmux_pad1 = AT25DFX_SPI_PINMUX_PAD1;
    at25dfx_spi_config.pinmux_pad2 = AT25DFX_SPI_PINMUX_PAD2;
    at25dfx_spi_config.pinmux_pad3 = AT25DFX_SPI_PINMUX_PAD3;

    /* SPI Initialization */
    spi_init(&at25dfx_spi, AT25DFX_SPI, &at25dfx_spi_config);

    /* SPI Enable */
    spi_enable(&at25dfx_spi);

    /* Chip Setup */
    at25dfx_chip_config_struct.type = AT25DFX_MEM_TYPE;
    at25dfx_chip_config_struct.cs_pin = AT25DFX_CS;

    /* Chip Initialization */
    at25dfx_chip_init(&at25dfx_chip, &at25dfx_spi, &at25dfx_chip_config_struct);

    /* Chip Wakeup */
    at25dfx_chip_wake(&at25dfx_chip);

    /* Erase Chip for storing the images */
    at25dfx_chip_erase(&at25dfx_chip);

    /* Disable global sector protect */
    at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
}
#endif
#endif
#endif

void otauInit(void)
{
	miQueueInit(&networkFrame);
	otauNotifyInit();
	otauUpgradeInit();
	otauDebugInit();
#ifdef OTAU_SERVER
	otauParserInit();
#else
#ifndef OTAU_USE_EXTERNAL_MEMORY
	nvm_init(INT_FLASH);
#else
#if (BOARD == SAMR30_MODULE_XPLAINED_PRO)
	serialFlashInit();
#else
#error "not supported "
#endif
#endif
#endif
	initDataHandleTable();
#if defined(OTAU_PHY_MODE)
	PHY_SubscribeReservedFrameIndicationCallback(otauPhyDataInd);
#endif
	MiApp_SubscribeManuSpecDataIndicationCallback(otauDataInd);
}

void otauTask(void)
{
	if (MiApp_IsConnected())
	{
		MiApp_Get(SHORT_ADDRESS, (uint8_t *)&shortAddress);
		if (!otauInited)
		{
			otauInit();
			otauInited = true;
		}
		if (0 != networkFrame.size)
		{
			uint8_t *event = (uint8_t *)miQueueRemove(&networkFrame, NULL);
			if (NULL != event)
			{
				/* + 4  to store the next pointer in Queue */
				otau_rcvd_frame_t *buff = (otau_rcvd_frame_t *)(event + 4);
				otauRcvdFrame(buff);
				MiMem_Free(event);
			}
		}
	}
	#ifdef OTAU_SERVER
	serialDataHandler();
	if (0 != dataFromTool.size)
	{
		uint8_t *event = (uint8_t *)miQueueRemove(&dataFromTool, NULL);
		if (NULL != event)
		{
			/* + 4  to store the next pointer in Queue */
			otau_domain_msg_t *buff = (otau_domain_msg_t *)(event + 4);
			otauHandleMsg(buff);
			MiMem_Free(event);
		}
	}
	#endif
}

void otauHandleMsg(otau_domain_msg_t *otau_domain_msg)
{
#ifdef OTAU_SERVER
	switch (otau_domain_msg->domainId)
	{
		case DOMAIN_OTAU_NOTIFY:
			otauHandleNotifyMsg(otau_domain_msg);
			break;
		case DOMAIN_OTAU_UPGRADE:
			otauHandleUpgradeMsg(otau_domain_msg);
			break;
		case DOMAIN_OTAU_DEBUG:
			otauHandleDebugMsg(otau_domain_msg);
			break;
	}
#endif
}

void otauRcvdFrame(otau_rcvd_frame_t *frame)
{
	uint8_t *domain = frame->frame_payload;
	if (DOMAIN_OTAU_NOTIFY == *domain)
	{
		otauNotifyRcvdFrame(frame->addr_mode, frame->src_addr, frame->frame_length, frame->frame_payload);
	}
	else if (DOMAIN_OTAU_UPGRADE == *domain)
	{
		otauUpgradeRcvdFrame(frame->addr_mode, frame->src_addr, frame->frame_length, frame->frame_payload);
	}
	else if (DOMAIN_OTAU_DEBUG == *domain)
	{
		otauDebugRcvdFrame(frame->addr_mode, frame->src_addr, frame->frame_length, frame->frame_payload);
	}
}

#if defined(OTAU_PHY_MODE)
static void phy_transmit_frame(uint8_t dst_addr_mode,uint8_t* dst_addr,uint8_t *src_addr,
	uint16_t panid, uint8_t payload_length, uint8_t *payload, bool ack_req)
{
	uint8_t i;
	uint8_t *frame_ptr;
	uint16_t fcf = 0;
	uint16_t temp_value;
	PHY_DataReq_t phyDataRequest;
	/* Get length of current frame. */
	frame_ptr = &storage_buffer[1] + FCF_LEN;

	/* Set DSN. */
	*frame_ptr = seq_no++;
	frame_ptr++;

	/* Destination address */
	if (FCF_LONG_ADDR == dst_addr_mode) {
		/* Destination PAN-Id */
		temp_value = panid;
		memcpy(frame_ptr, &temp_value, 2);
		frame_ptr += PAN_ID_LEN;
#ifndef OTAU_SERVER
		dst_addr = serverAddress.extended_addr;
#endif
		memcpy(frame_ptr, dst_addr, 8);
		frame_ptr += EXT_ADDR_LEN;

		fcf |= FCF_SET_DEST_ADDR_MODE(FCF_LONG_ADDR);

		/* Source address */
		memcpy(frame_ptr, src_addr, 8);
		frame_ptr += EXT_ADDR_LEN;

		fcf |= FCF_SET_SOURCE_ADDR_MODE(FCF_LONG_ADDR);
	}
	else
	{
		uint64_t tempSrcAddr;
		/* Destination PAN-Id */
		temp_value = macShortAddress_def;
		convert_16_bit_to_byte_array(temp_value, frame_ptr);
		frame_ptr += PAN_ID_LEN;

		convert_16_bit_to_byte_array(macShortAddress_def,
		frame_ptr);
		frame_ptr += SHORT_ADDR_LEN;

		fcf |= FCF_SET_DEST_ADDR_MODE(FCF_SHORT_ADDR);

		memcpy(&tempSrcAddr, src_addr, EXT_ADDR_LEN);
		/* Source address */
		convert_64_bit_to_byte_array(tempSrcAddr, frame_ptr);
		frame_ptr += EXT_ADDR_LEN;

		fcf |= FCF_SET_SOURCE_ADDR_MODE(FCF_LONG_ADDR);
	}
	/*
	* Payload is stored to the end of the buffer avoiding payload
	* copying.
	*/
	for (i = 0; i < payload_length; i++) {
		*frame_ptr++ = *(payload + i);
	}

	/* No source PAN-Id included, but FCF updated. */
	fcf |= FCF_PAN_ID_COMPRESSION;

	/* Set the FCF. */
	fcf |= FCF_SET_FRAMETYPE(0x07);
	if (ack_req) {
		fcf |= FCF_ACK_REQUEST;
	}
	memcpy(&storage_buffer[1], &fcf, 2);

	storage_buffer[0] = frame_ptr - &storage_buffer[1];

    phyDataRequest.polledConfirmation = true;
    phyDataRequest.confirmCallback = otauPhyDataConf;
    phyDataRequest.data = &storage_buffer[0];

	PHY_DataReq(&phyDataRequest);
}

static void otauPhyDataConf(uint8_t status)
{
	switch (status)
	{
		case PHY_STATUS_SUCCESS:
			status = OTAU_SUCCESS;
		break;
		case PHY_STATUS_NO_ACK:
			status = OTAU_NO_ACK;
		break;
		case PHY_STATUS_CHANNEL_ACCESS_FAILURE:
			status = OTAU_CCA_FAILURE;
		break;
		default:
			status = OTAU_ERROR;
	}
	if (DOMAIN_OTAU_NOTIFY == otauPhyData[0])
	{
		otauNotifySentFrame(otauPhyData[1], EXTENDED_ADDR_MODE, &storage_buffer[6], status);
	}
	else if (DOMAIN_OTAU_UPGRADE == otauPhyData[0])
	{
		otauUpgradeSentFrame(otauPhyData[1], EXTENDED_ADDR_MODE, &storage_buffer[6], status);
	}
	else if (DOMAIN_OTAU_DEBUG == otauPhyData[0])
	{
		otauDebugSentFrame(otauPhyData[1], EXTENDED_ADDR_MODE, &storage_buffer[6], status);
	}
}


static void otauPhyDataInd(PHY_DataInd_t *ind)
{
	uint8_t *frame_ptr = ind->data;
	uint8_t src_addr_mode;
	uint8_t dst_addr_mode;
	bool intra_pan;
	uint8_t addr_field_len = FRAME_OVERHEAD;
	uint16_t fcf;
	uint8_t *src_addr;

    ind->size -=2;
	fcf = convert_byte_array_to_16_bit(frame_ptr);
	
	src_addr_mode = FCF_GET_SOURCE_ADDR_MODE(fcf);
	dst_addr_mode = FCF_GET_DEST_ADDR_MODE(fcf);
	intra_pan = fcf & FCF_PAN_ID_COMPRESSION;

	if (0 != dst_addr_mode)
	{
		addr_field_len += PAN_ID_LEN;
		if(FCF_SHORT_ADDR == dst_addr_mode)
		{
			addr_field_len += SHORT_ADDR_LEN;
		}
		else if (FCF_LONG_ADDR == dst_addr_mode)
		{
			addr_field_len += EXT_ADDR_LEN;
		}
	}
	if (0 != src_addr_mode)
	{
		if (!intra_pan)
		{
			addr_field_len += PAN_ID_LEN;
		}
		src_addr = frame_ptr + addr_field_len;
		if (FCF_SHORT_ADDR == src_addr_mode)
		{
			addr_field_len += SHORT_ADDR_LEN;
		}
		else if (FCF_LONG_ADDR == src_addr_mode)
		{
			addr_field_len += EXT_ADDR_LEN;
		}
	}
	else
	{
		src_addr = NULL;
	}
	otau_rcvd_frame_t rcvd_frame;
	rcvd_frame.addr_mode = EXTENDED_ADDR_MODE;
	rcvd_frame.src_addr = src_addr;
	rcvd_frame.frame_length = ind->size - addr_field_len;
	rcvd_frame.frame_payload = frame_ptr + addr_field_len;
	otauRcvdFrame(&rcvd_frame);
}
#endif

void otauDataSend(addr_mode_t addr_mode, uint8_t *addr, void *payload, uint16_t len)
{
	uint16_t dstAddr;
	uint8_t domainId = *((uint8_t *)payload);
	uint8_t msgId = *(uint8_t *)((uint8_t *)payload + 1);
	if (EXTENDED_ADDR_MODE != addr_mode)
	{
		if (NATIVE_ADDR_MODE == addr_mode && NULL == addr)
		{
	#ifndef OTAU_SERVER
			addr = (uint8_t *)&serverAddress.native_addr;
	#endif
		}
		else if (BROADCAST_MODE == addr_mode)
		{
			dstAddr = MESH_BROADCAST_TO_ALL;
			addr = (uint8_t *)&dstAddr;
		}
		if (DOMAIN_OTAU_NOTIFY == domainId)
		{
			if (!MiApp_ManuSpecSendData(SHORT_ADDR_LEN, addr, len, payload, msgHandle, 1, otauDataConf))
			{
				otauNotifySentFrame(msgId, NATIVE_ADDR_MODE, addr, OTAU_ERROR);
			}
			else
			{
				addDataHandleTableEntry(domainId, msgId, NATIVE_ADDR_MODE, addr, msgHandle);
				msgHandle++;
			}		
		}
		else if (DOMAIN_OTAU_UPGRADE == domainId)
		{
			if (!MiApp_ManuSpecSendData(SHORT_ADDR_LEN, addr, len, payload, msgHandle, 1, otauDataConf))
			{
				otauUpgradeSentFrame(msgId, NATIVE_ADDR_MODE, addr, OTAU_ERROR);
			}
			else
			{
				addDataHandleTableEntry(domainId, msgId, NATIVE_ADDR_MODE, addr, msgHandle);
				msgHandle++;
			}
		}
		else if (DOMAIN_OTAU_DEBUG == domainId)
		{
			if (!MiApp_ManuSpecSendData(SHORT_ADDR_LEN, addr, len, payload, msgHandle, 1, otauDataConf))
			{
				otauDebugSentFrame(msgId, NATIVE_ADDR_MODE, addr, OTAU_ERROR);
			}
			else
			{
				addDataHandleTableEntry(domainId, msgId, NATIVE_ADDR_MODE, addr, msgHandle);
				msgHandle++;
			}
		}
	}
#if defined(OTAU_PHY_MODE)
	else
	{
		uint16_t panID;
		MiApp_Get(PANID, (uint8_t *)&panID);
#ifndef OTAU_SERVER
		addr = (uint8_t *)&serverAddress.extended_addr;
#endif
		memcpy(&otauPhyData[0], payload, len);
		phy_transmit_frame(FCF_LONG_ADDR, addr, get_node_address(EXTENDED_ADDR_MODE), panID, len, otauPhyData, 1);
	}
#endif
}

static void otauDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
	uint16_t dstAddr;
	uint8_t domainId = DOMAIN_OTAU_NOTIFY;
	uint8_t messageId = 0xFF;
	uint8_t addrMode;
        error_status_t otauStatus;
	getAddressFromDataHandleTable(msgConfHandle, &domainId, &messageId, &addrMode, (uint8_t *)&dstAddr);
	switch (status)
	{
		case SUCCESS:
		otauStatus = OTAU_SUCCESS;
		break;
		case NO_ACK:
		otauStatus = OTAU_NO_ACK;
		break;
		case CHANNEL_ACCESS_FAILURE:
		otauStatus = OTAU_CCA_FAILURE;
		break;
		default:
		otauStatus = OTAU_ERROR;
	}
	if (DOMAIN_OTAU_NOTIFY == domainId)
	{
		otauNotifySentFrame(messageId, NATIVE_ADDR_MODE, (uint8_t *)&dstAddr, otauStatus);
	}
	else if (DOMAIN_OTAU_UPGRADE == domainId)
	{
		otauUpgradeSentFrame(messageId, NATIVE_ADDR_MODE, (uint8_t *)&dstAddr, otauStatus);
	}
	else if (DOMAIN_OTAU_DEBUG == domainId)
	{
		otauDebugSentFrame(messageId, NATIVE_ADDR_MODE, (uint8_t *)&dstAddr, otauStatus);
	}
}

static void otauDataInd(RECEIVED_MESH_MESSAGE *ind)
{
	if((NULL != ind) && (NULL != ind->payload))
	{
		/* + 4  to store the next pointer in Queue */
		uint8_t *buffer_header = MiMem_Alloc(sizeof(otau_rcvd_frame_t) 
							+ NATIVE_ADDR_SIZE + ind->payloadSize + 4);
		if (NULL != buffer_header)
		{
			uint8_t *frame_ptr;
			uint16_t size;
			/* + 4  to store the next pointer in Queue */
			otau_rcvd_frame_t *rcvd_frame = (otau_rcvd_frame_t *)(buffer_header + 4);

			frame_ptr = (uint8_t *)rcvd_frame + sizeof(otau_rcvd_frame_t);
			memcpy(&(rcvd_frame->src_addr), &frame_ptr, 4);
			memcpy(frame_ptr, (uint8_t *)&(ind->sourceAddress), NATIVE_ADDR_SIZE);
			frame_ptr += NATIVE_ADDR_SIZE;

			memcpy(&(rcvd_frame->frame_payload), &frame_ptr, 4);
			memcpy(rcvd_frame->frame_payload, ind->payload, ind->payloadSize);

			size = ind->payloadSize;
			memcpy(&(rcvd_frame->frame_length), &size, 2);
			rcvd_frame->addr_mode = NATIVE_ADDR_MODE;

			miQueueAppend(&networkFrame, (void *)buffer_header);
		}
	}
}

void otauTimerStart(otau_domain_t domain_code, uint32_t interval, otau_timer_mode_t mode)
{
	switch (domain_code)
	{
		case DOMAIN_OTAU_NOTIFY:
			if (SYS_TimerStarted(&otauNotifyTimer))
			{
				SYS_TimerStop(&otauNotifyTimer);
			}
			otauNotifyTimer.interval = interval;
			otauNotifyTimer.handler = otauNotifyTimerHandler;
			if(TIMER_MODE_SINGLE == mode) {
				otauNotifyTimer.mode = SYS_TIMER_INTERVAL_MODE;
				} else {
				otauNotifyTimer.mode = SYS_TIMER_PERIODIC_MODE;
			}
			SYS_TimerStart(&otauNotifyTimer);
			break;
		case DOMAIN_OTAU_UPGRADE:
			if (SYS_TimerStarted(&otauUpgradeTimer))
			{
				SYS_TimerStop(&otauUpgradeTimer);
			}
			otauUpgradeTimer.interval = interval;
			otauUpgradeTimer.handler = otauUpgradeTimerHandler;
			if(TIMER_MODE_SINGLE == mode) {
				otauUpgradeTimer.mode = SYS_TIMER_INTERVAL_MODE;
				} else {
				otauUpgradeTimer.mode = SYS_TIMER_PERIODIC_MODE;
			}
			SYS_TimerStart(&otauUpgradeTimer);
			break;
		case DOMAIN_OTAU_DEBUG:
			if (SYS_TimerStarted(&otauDebugTimer))
			{
				SYS_TimerStop(&otauDebugTimer);
			}
			otauDebugTimer.interval = interval;
			otauDebugTimer.handler = otauDebugTimerHandler;
			if(TIMER_MODE_SINGLE == mode) {
				otauDebugTimer.mode = SYS_TIMER_INTERVAL_MODE;
				} else {
				otauDebugTimer.mode = SYS_TIMER_PERIODIC_MODE;
			}
			SYS_TimerStart(&otauDebugTimer);
			break;
		default:
			break;
	}
}

void otauTimerStop(otau_domain_t domain_code)
{
	switch (domain_code)
	{
		case DOMAIN_OTAU_NOTIFY:
			SYS_TimerStop(&otauNotifyTimer);
			break;
		case DOMAIN_OTAU_UPGRADE:
			SYS_TimerStop(&otauUpgradeTimer);
			break;
		case DOMAIN_OTAU_DEBUG:
			SYS_TimerStop(&otauDebugTimer);
			break;
		default:
			break;
	}
}

uint8_t *get_node_address(addr_mode_t addr_mode)
{
	if (NATIVE_ADDR_MODE == addr_mode)
	{
		return (uint8_t *)&shortAddress;
	}
	else
	{
		return (uint8_t *)&myLongAddress;
	}
}

void otauLed(otau_led_t led_state)
{
	if (OTAU_LED_ON == led_state)
	{
		LED_On(LED0);
	}
	else if (OTAU_LED_OFF == led_state)
	{
		LED_Off(LED0);
	}
	else
	{
		LED_Toggle(LED0);
	}
}


void otauResetDevice(void)
{
	NVIC_SystemReset();
}

void reverseMemcpy(uint8_t *dst, uint8_t *src, uint8_t len)
{
	uint8_t i;
	for (i=0; i < len; ++i)
	{
		dst[len-1-i] = src[i];
	}
}

static bool getAddressFromDataHandleTable(uint8_t handle, uint8_t *domainId, uint8_t *messageId, uint8_t *addrMode, uint8_t *addr)
{
	uint8_t loopIndex;
	for (loopIndex = 0; loopIndex < DATA_HANDLE_TABLE_SIZE; loopIndex++)
	{
		if (dataHandleTable[loopIndex].dataHandle == handle)
		{
			*domainId = dataHandleTable[loopIndex].domainId;
			*messageId = dataHandleTable[loopIndex].messageId;
			*addrMode = dataHandleTable[loopIndex].addrMode;
			memcpy(addr, &(dataHandleTable[loopIndex].nativeAddr), SHORT_ADDR_LEN);
			dataHandleTable[loopIndex].domainId = 0xFF;
			return true;
		}
	}
	return false;
}

static bool addDataHandleTableEntry(uint8_t domainId, uint8_t messageId, addr_mode_t addrMode, uint8_t *addr, uint8_t handle)
{
	uint8_t loopIndex;
	for (loopIndex = 0; loopIndex < DATA_HANDLE_TABLE_SIZE; loopIndex++)
	{
		if (dataHandleTable[loopIndex].domainId == 0xFF)
		{
			dataHandleTable[loopIndex].domainId = domainId;
			dataHandleTable[loopIndex].messageId = messageId;
			dataHandleTable[loopIndex].addrMode = addrMode;
			dataHandleTable[loopIndex].dataHandle = handle;
			memcpy(&(dataHandleTable[loopIndex].nativeAddr), addr, SHORT_ADDR_LEN);
			return true;
		}
	}
	return false;
}

static void initDataHandleTable(void)
{
	uint8_t loopIndex;
	for (loopIndex = 0; loopIndex < DATA_HANDLE_TABLE_SIZE; loopIndex++)
	{
		dataHandleTable[loopIndex].domainId = 0xFF;
		dataHandleTable[loopIndex].messageId = 0xFF;
	}
}

#ifndef OTAU_SERVER
void otauSetServerDetails(addr_mode_t addr_mode, uint8_t *addr)
{
	if (NATIVE_ADDR_MODE == addr_mode)
	{
		memcpy(&serverAddress.native_addr, addr, NATIVE_ADDR_SIZE);
	}
	else
	{
		memcpy(&serverAddress.extended_addr, addr, EXTENDED_ADDR_SIZE);
	}
}

void otauGetServerDetails(addr_mode_t addr_mode, uint8_t *addr)
{
	if (NATIVE_ADDR_MODE == addr_mode)
	{
		addr = (uint8_t *)&serverAddress.native_addr;
	}
	else
	{
		addr = (uint8_t *)&serverAddress.extended_addr;
	}
        /* Keep Compiler happy*/
        addr = addr;
}
#endif
#endif //#if defined(OTAU_ENABLED)
