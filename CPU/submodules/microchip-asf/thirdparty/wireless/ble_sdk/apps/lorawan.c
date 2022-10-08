/**
* \file  lorawan.c
*
* \brief LORAWAN Demo Application main file
*
*
* Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries. 
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
 
/****************************** INCLUDES **************************************/
/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include "console_serial.h"
#include "radio_driver_hal.h"
#include "sw_timer.h"
#include "aes_engine.h"
#include "pds_interface.h"
#include "delay.h"
#include "sleep_timer.h"
#include "pmm.h"
#include "conf_pmm.h"
#include "lorawan.h"
#include "ble_manager.h"



/* Application Timer ID */
uint8_t AppTimerID;

/* Callback Functions */
void appdata_callback(void *appHandle, appCbParams_t *appdata);
void joindata_callback(StackRetStatus_t status);

/* Sleep Wake up function */
void appWakeup(void);

void lorawan_main (void);

/* Application resources un-initialization function -
 * This function is used to un-initialize UART and SPI drivers 
 * before putting the SAM R34 to Sleep
 */
static void app_resources_uninit(void);

/* OTAA join parameters */
uint8_t demoDevEui[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x12 }; //{0x00, 0x04, 0x25, 0x19, 0x18, 0x01, 0xd4, 0xb6};
uint8_t demoAppEui[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0xB7, 0xFE }; //{0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x01, 0x27, 0x68};
uint8_t demoAppKey[16] = { 0xB0, 0x08, 0x24, 0x2E, 0xEE, 0x37, 0xA6, 0x8C, 0x3A, 0xD1, 0x80, 0x77, 0xA7, 0x68, 0x61, 0x12 }; //{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12};
	 
/* Device join status flag */
bool joined = false;
bool ble_init_start = false,ble_init_complete = false;

/* Application payload request */
LorawanSendReq_t lorawanSendReq;

/* Application buffer */
uint8_t app_buf[4] = {"LoRa"};
uint8_t bleappsTimerId = 0xFF;
uint8_t iTimerId = 0xFF;
extern int send_ble_data;
extern char ble_apps_buf[100];
/* App Task Handler Prototype */
SYSTEM_TaskStatus_t APP_TaskHandler(void);

void lorawan_main (void)
{
	StackRetStatus_t status;
	bool pds_status = false;
	
	/* Delay Init */
	delay_init();
	
	/* HAL Init */
	HAL_RadioInit();
	
	/*SW Timer Init */
	SystemTimerInit();
	
	/* AES Init */
	AESInit();
	
	/* PDS Init */
	PDS_Init();
	
	/* Sleep Init */
	SleepTimerInit();
	
	/* Interrupt Enable */
	INTERRUPT_GlobalInterruptEnable();
	
	//printf("LoRaWAN Sample Application\r\n");
	
	/* Initialize LoRaWAN Stack */
	LORAWAN_Init(appdata_callback,joindata_callback);
	
	/* LoRaWAN Stack RESET */
	LORAWAN_Reset(ISM_EU868);

	pds_status = PDS_IsRestorable();
	if(false == pds_status)
	{
		/* Set Join Parameters */
		/* Dev EUI */
		status = LORAWAN_SetAttr (DEV_EUI, demoDevEui);
		if (LORAWAN_SUCCESS == status)
		{
			//printf("Set DevEUI Success\r\n");
		
		}
		/* APP EUI */
		status = LORAWAN_SetAttr (APP_EUI, demoAppEui);
		if (LORAWAN_SUCCESS == status)
		{
			//printf("Set AppEUI Success\r\n");
		
		}
		status = LORAWAN_SetAttr (APP_KEY, demoAppKey);
		if (LORAWAN_SUCCESS == status)
		{
			//printf("Set AppKey Success\r\n");
		
		}
		
		/* Store all the parameters to PDS */
		PDS_StoreAll();
	}
	else
	{
		//printf("Restored data from PDS successfully.\r\n");
		/* Restore the stored parameters from PDS */
		PDS_RestoreAll();
	}	
	//SwTimerCreate(&iTimerId);
	SwTimerCreate(&bleappsTimerId);
	SwTimerStart(bleappsTimerId,MS_TO_US(1000),
						SW_TIMEOUT_RELATIVE,appWakeup,NULL);	

}

void joindata_callback(StackRetStatus_t status)
{
	printf("LORAWAN Join ");
	if(LORAWAN_SUCCESS == status)
	{
		printf(" Successful\r\n");
		/* Update Join status flag */
		joined = true;
		ble_init_start = true;
	}
	else if(LORAWAN_NO_CHANNELS_FOUND == status)
	{
		joined = false;
		printf(" No Free Channel found\r\n");
	}
	else
	{
		joined = false;
		printf(" Denied\r\n");
	}	
	SYSTEM_PostTask(APP_TASK_ID);
}

void appdata_callback(void *appHandle, appCbParams_t *appdata)
{
	if (LORAWAN_EVT_RX_DATA_AVAILABLE == appdata->evt)
	{
		printf("Rx data Received - Status: %d\r\n",appdata->param.rxData.status);
	}
	else if(LORAWAN_EVT_TRANSACTION_COMPLETE == appdata->evt)
	{
		printf("Transaction Complete - Status: %d\r\n",appdata->param.transCmpl.status);
	}

}

void appWakeup(void)
{
	HAL_Radio_resources_init();
	
	//printf("Wake up from Sleep %ld ms\r\n", sleptDuration);
	SYSTEM_PostTask(APP_TASK_ID);
}

SYSTEM_TaskStatus_t APP_TaskHandler(void)
{
	StackRetStatus_t status;
	
	//printf("App Task Handler\r\n");
	
	if(false == joined)
	{
		/* Send Join request */
		status = LORAWAN_Join(LORAWAN_OTAA);
		if (LORAWAN_SUCCESS == status)
		{
			joined = false;
			printf("\r\nLORAWAN Join Request Sent\n\r");
		}
	}
	else
	{
		/* Send application data */
		//lorawanSendReq.buffer = &app_buf;
		//lorawanSendReq.bufferLength = sizeof(app_buf);
		if(send_ble_data)
		{
			lorawanSendReq.buffer = &ble_apps_buf;
			lorawanSendReq.bufferLength = send_ble_data;
			send_ble_data = 0;			
		//}else
		//{
			//lorawanSendReq.buffer = &app_buf;
			//lorawanSendReq.bufferLength = sizeof(app_buf);
		//}

		
		lorawanSendReq.confirmed = LORAWAN_UNCNF;
		lorawanSendReq.port = 1;
		status = LORAWAN_Send(&lorawanSendReq);
		if (LORAWAN_SUCCESS == status)
		{
			//printf("Data Sent\r\n");
		}
		}
			SwTimerStart(bleappsTimerId,MS_TO_US(10000),
						SW_TIMEOUT_RELATIVE,appWakeup,NULL);
	}	

	return SYSTEM_TASK_SUCCESS;
}

#ifdef CONF_PMM_ENABLE
static void app_resources_uninit(void)
{
	/* Disable USART TX and RX Pins */
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	pin_conf.powersave  = true;
	#ifdef HOST_SERCOM_PAD0_PIN
	port_pin_set_config(HOST_SERCOM_PAD0_PIN, &pin_conf);
	#endif
	#ifdef HOST_SERCOM_PAD1_PIN
	port_pin_set_config(HOST_SERCOM_PAD1_PIN, &pin_conf);
	#endif
	/* Disable UART module */
	//sio2host_deinit();
	/* Disable Transceiver SPI Module */
	HAL_RadioDeInit();
}
#endif