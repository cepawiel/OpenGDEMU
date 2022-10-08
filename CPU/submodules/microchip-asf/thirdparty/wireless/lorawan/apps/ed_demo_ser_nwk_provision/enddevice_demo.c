/**
* \file  enddevice_demo.c
*
* \brief Serial Provisioning of LoRaWAN Demo Application
*
* Copyright (c) 2019-2020 Microchip Technology Inc. and its subsidiaries.
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

/**
* \mainpage
* \section preface Preface
* Serial Provisioning of LoRaWAN Demo Application  available from Atmel Studio examples,
* this is used to send the temperature sensor data through the LoRaWAN network to the network server.
* <P>• This example provides an option for the  user to configure the join types and provisioning paramters as per the join type chosen in run time.</P>
* <P>• User if chooses NA / AU band can choose the sub-band of operation depending on the LoRaWAN Gateway being used.</P>
* <P>• This example uses the commands sent over UART/TeraTerm to join/send data/sleep. </P>
*/
/****************************** INCLUDES **************************************/
#include "asf.h"
#include "lorawan.h"
#include "system_task_manager.h"
#include "enddevice_demo.h"
#include "conf_app.h"
#include "sio2host.h"
#include "resources.h"
#include "delay.h"
#include "sw_timer.h"
#include "LED.h"
#include "pmm.h"
#include "radio_driver_hal.h"
#include "conf_pmm.h"
#include "conf_sio2host.h"
#include "pds_interface.h"


#if (EDBG_EUI_READ == 1)
#include "edbg_eui.h"
#endif
#include "atomic.h"
#include <stdint.h>
/******************************** MACROS ***************************************/

/************************** GLOBAL VARIABLES ***********************************/
/************************** GLOBAL VARIABLES ***********************************/
static bool joinTypeSelection = false;
static bool keyConfigSelection = false;
static bool devEuiSelectionDone = false;
static bool joinEuiSelectionDone = false;
static bool appKeySelectionDone = false;
static bool devAddrSelectionDone = false;
static bool appSkeySelectionDone = false;
static bool nwkSkeySelectionDone = false;
static bool verifyDemoConfigSelection = false;
ActivationType_t demoJoinActivationType;

static bool joined = false;
static float cel_val;
static float fahren_val;
static char temp_sen_str[25];
static uint8_t data_len = 0;
bool certAppEnabled = false;

static uint8_t on = LON;
static uint8_t off = LOFF;
static uint8_t toggle = LTOGGLE;

static volatile uint8_t appTaskFlags = 0x00u;
/* Default Regional band start delay time */
volatile static uint8_t count = 5;

static uint8_t rxchar[11];
static bool startReceiving = false;
extern uint8_t demoTimerId;
extern uint8_t lTimerId;
static AppTaskState_t appTaskState;

static const char* bandStrings[] =
{
    "FactoryDefaultReset",
#if (EU_BAND == 1)
    "EU868",
#endif
#if (NA_BAND == 1)
    "NA915",
#endif
#if (AU_BAND == 1)
    "AU915",
#endif
#if (AS_BAND == 1)
    "AS923",
#endif
#if (JPN_BAND == 1)
    "JPN923",
#endif
#if (KR_BAND == 1)
    "KR920",
#endif
#if (IND_BAND == 1)
    "IND865",
#endif
    "Clear PDS",
    "Reset Board"
};


uint8_t bandTable[] =
{
    0xFF,
#if (EU_BAND == 1)
    ISM_EU868,
#endif
#if (NA_BAND == 1)
    ISM_NA915,
#endif
#if (AU_BAND == 1)
    ISM_AU915,
#endif
#if (AS_BAND == 1)
    ISM_THAI923,
#endif
#if (JPN_BAND == 1)
    ISM_JPN923,
#endif
#if (KR_BAND == 1)
    ISM_KR920,
#endif
#if (IND_BAND == 1)
    ISM_IND865,
#endif
    0xFF,
    0xFF
};

static uint8_t tempBuff[4];
#if (RANDOM_NW_ACQ == 0)
	/*Band Selection for NA and AU region */
	static uint8_t subBand = 2;
#endif
/*ABP Join Parameters */
static uint32_t demoDevAddr;
static uint8_t demoNwksKey[16];
static uint8_t demoAppsKey[16];
/* OTAA join parameters */
static uint8_t demoDevEui[8];
static uint8_t demoJoinEui[8];
static uint8_t demoAppKey[16];

static LorawanSendReq_t lorawanSendReq;
static char serialBuffer;

/* Muticast Parameters */
static bool demoMcastEnable = DEMO_APP_MCAST_ENABLE;
static uint32_t demoMcastDevAddr = DEMO_APP_MCAST_GROUP_ADDRESS;
static uint8_t demoMcastNwksKey[16] = DEMO_APP_MCAST_NWK_SESSION_KEY;
static uint8_t demoMcastAppsKey[16] = DEMO_APP_MCAST_APP_SESSION_KEY;
static uint8_t demoMcastGroupId = DEMO_APP_MCAST_GROUPID;
/************************** EXTERN VARIABLES ***********************************/
extern bool button_pressed;
extern bool factory_reset;
extern bool bandSelected;
extern uint32_t longPress;

static void demoAddr_keyconfig_handler(void);
static void sendData(void);
static void appPostTask(AppTaskIds_t id);
static void displaySelectedAppConfig(void);
static SYSTEM_TaskStatus_t (*appTaskHandlers[])(void);
static void demoTimerCb(void * cnt);
static void lTimerCb(void * data);
static SYSTEM_TaskStatus_t displayTask(void);
static SYSTEM_TaskStatus_t processTask(void);
static void processRunDemoCertApp(void);
static void processRunRestoreBand(void);
static void processJoinAndSend(void);
static void processAppConfig(void);
static void processRunDemoApp(void);
static void displayRunDemoCertApp(void);
static void displayRunRestoreBand(void);
static void displayAppConfigMenu(void);
static void displayJoinAndSend(void);
static void displayRunDemoApp(void);
#ifdef CONF_PMM_ENABLE
static void appWakeup(uint32_t sleptDuration);
static void app_resources_uninit(void);
#endif
/************************** FUNCTION PROTOTYPES ********************************/
SYSTEM_TaskStatus_t APP_TaskHandler(void);
static float convert_celsius_to_fahrenheit(float cel_val);
/*********************************************************************//*
 \brief      Function that processes the Rx data
 \param[in]  data - Rx data payload
 \param[in]  dataLen - The number of Rx bytes
 ************************************************************************/
static void demo_handle_evt_rx_data(void *appHandle, appCbParams_t *appdata);

/***************************** FUNCTIONS ***************************************/

static SYSTEM_TaskStatus_t (*appTaskHandlers[APP_TASKS_COUNT])(void) = {
    /* In the order of descending priority */
    displayTask,
    processTask
};

/*********************************************************************//**
\brief    Calls appropriate functions based on state variables
*************************************************************************/
static SYSTEM_TaskStatus_t displayTask(void)
{
	switch(appTaskState)
	{
		case RESTORE_BAND_STATE:
			displayRunRestoreBand();
			break;
		case DEMO_APP_CONFIG:
			displayAppConfigMenu();
			break;
		case DEMO_CERT_APP_STATE:
			displayRunDemoCertApp();
			break;
		case DEMO_APP_STATE:
			displayRunDemoApp();
			break;
		case JOIN_SEND_STATE:
			displayJoinAndSend();
			break;
		default:
			printf("Error STATE Entered\r\n");
			break;
	}

	return SYSTEM_TASK_SUCCESS;
}

/*********************************************************************//**
\brief    Pulls the data from UART when activated
*************************************************************************/
void serial_data_handler(void)
{
	int rxChar;
	char serialData;
	/* verify if there was any character received*/
	if (startReceiving == true)
	{
		if((-1) != (rxChar = sio2host_getchar_nowait()))
		{
			serialData = (char)rxChar;
			if((serialData != '\r') && (serialData != '\n') && (serialData != '\b'))
			{
				startReceiving = false;
  			    serialBuffer = rxChar;
			    appPostTask(PROCESS_TASK_HANDLER);
				printf("\r\n");
			}
		}
	}
}
uint8_t asciiToDecConverter(uint8_t input)
{
	uint8_t decVal = -1;
	if (input >= 48 && input <= 57)  decVal = input - 48; // decimal values between 1 and 9
	else if (input >= 65 && input <= 70) decVal = input - 55; // hex values between A and F
	else if (input >= 97 && input <= 102) decVal = input - 87;  // hex values between a and f
	return decVal;
}

void serial_read_appConfig(uint8_t *buffer, uint8_t noOfBytes)
{
	uint8_t i = 0;
	uint8_t temp = 0, rxDec;
	while (i != noOfBytes*2)
	{
		rxDec = asciiToDecConverter(sio2host_getchar());
		if (i%2 == 0) temp = rxDec << 4;
		else if (i%2 != 0) buffer[i/2] = temp + rxDec;
		if (rxDec != 255) i++; // increment count only when receiving a valid hex value from 0-F
	}
}

static void displaySelectedAppConfig(void)
{
	uint8_t i = 0;
	if (demoJoinActivationType == OVER_THE_AIR_ACTIVATION)
	{
		printf("Activation Type: OVER THE AIR ACTIVATION (OTAA)\r\n");
		printf("Device EUI: 0x");
		for (i=0;i<8;i++) printf("%x",demoDevEui[i]);
		printf("\r\n");
		printf("Join EUI: 0x");
		for (i=0;i<8;i++) printf("%x",demoJoinEui[i]);
		printf("\r\n");
		printf("Application Key: 0x");
		for (i=0;i<16;i++) printf("%x",demoAppKey[i]);
		printf("\r\n");


	}
	else if (demoJoinActivationType == ACTIVATION_BY_PERSONALIZATION)
	{
		printf("Activation Type: ACTIVATION BY PERSONALIZATION (ABP)\r\n");
		printf("Device Address: 0x%lx\r\n",demoDevAddr);
		printf("Application Session Key: 0x");
		for (i=0;i<16;i++) printf("%x",demoAppsKey[i]);
		printf("\r\n");
		printf("Network Session Key: 0x");
		for (i=0;i<16;i++) printf("%x",demoNwksKey[i]);
		printf("\r\n");
	}

}
/*********************************************************************//**
\brief    Handle the Key / addresses based on Join Mechanism Chosen
*************************************************************************/
static void demoAddr_keyconfig_handler(void)
{
	if (demoJoinActivationType == OVER_THE_AIR_ACTIVATION)
	{
		if (devEuiSelectionDone == false)
		{
			serial_read_appConfig(demoDevEui, 8);
			devEuiSelectionDone = true;
		}
		else if (joinEuiSelectionDone == false)
		{
			serial_read_appConfig(demoJoinEui, 8);
			joinEuiSelectionDone = true;
		}
		else if (appKeySelectionDone == false)
		{
			serial_read_appConfig(demoAppKey, 16);
			appKeySelectionDone = true;
		}
	}
	else if (demoJoinActivationType == ACTIVATION_BY_PERSONALIZATION)
	{
		if (devAddrSelectionDone == false)
		{
			serial_read_appConfig(tempBuff, 4);
			demoDevAddr = (uint32_t)tempBuff[0] << 24 | (uint32_t)tempBuff[1] << 16 | (uint32_t)tempBuff[2] << 8 | (uint32_t)tempBuff[3];
			devAddrSelectionDone = true;
		}
		else if (appSkeySelectionDone == false)
		{
			serial_read_appConfig(demoAppsKey, 16);
			appSkeySelectionDone = true;
		}
		else if (nwkSkeySelectionDone == false)
		{
			serial_read_appConfig(demoNwksKey, 16);
			nwkSkeySelectionDone = true;
		}
	}
}

/*********************************************************************//**
\brief    Calls appropriate functions based on state variables
*************************************************************************/
static SYSTEM_TaskStatus_t processTask(void)
{
	switch(appTaskState)
	{
		case RESTORE_BAND_STATE:
			processRunRestoreBand();
			break;
		case DEMO_APP_CONFIG:
			processAppConfig();
			break;
		case DEMO_CERT_APP_STATE:
			processRunDemoCertApp();
			break;
		case DEMO_APP_STATE:
			processRunDemoApp();
			break;
		case JOIN_SEND_STATE:
			processJoinAndSend();
			break;
		default:
			printf("Error STATE Entered\r\n");
			break;
	}

	return SYSTEM_TASK_SUCCESS;
}

/*********************************************************************//**
\brief    Activates demo application or certification application
*************************************************************************/
static void processRunDemoCertApp(void)
{
	if(serialBuffer == '1')
	{
		appTaskState = DEMO_APP_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
	else
	{
		printf("Please enter a valid choice\r\n");
		appTaskState = DEMO_CERT_APP_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
}

/*********************************************************************//**
\brief    Restores the previous band and runs
*************************************************************************/
static void processRunRestoreBand(void)
{
	StackRetStatus_t status = LORAWAN_SUCCESS;
	uint8_t prevBand = 0xff;
	uint8_t choice = 0xff;
	bool joinBackoffEnable = false;

	PDS_RestoreAll();
	LORAWAN_GetAttr(ISMBAND,NULL,&prevBand);
	for (uint32_t i = 0; i < sizeof(bandTable)-1; i++)
	{
		if(bandTable[i] == prevBand)
		{
			choice = i;
			break;
		}
	}
	if(choice >0 && choice < sizeof(bandTable)-1)
	{
		status = LORAWAN_Reset(bandTable[choice]);
	}

	 /*Disabled Join backoff in Demo application
	Needs to be enabled in Production Environment Ref Section */
    LORAWAN_SetAttr(JOIN_BACKOFF_ENABLE,&joinBackoffEnable);

	if(status == LORAWAN_SUCCESS && choice < sizeof(bandTable)-1)
	{
		uint32_t joinStatus = 0;
		PDS_RestoreAll();
		LORAWAN_GetAttr(LORAWAN_STATUS,NULL, &joinStatus);
		printf("\r\nPDS_RestorationStatus: Success\r\n" );
		if(joinStatus & LORAWAN_NW_JOINED)
		{
			joined = true;
			printf("joinStatus: Joined\r\n");
		}
		else
		{
			joined = false;
			printf("JoinStatus : Denied\r\n");
			set_LED_data(LED_AMBER,&on);
			SYSTEM_PostTask(APP_TASK_ID);
		}
		printf("Band: %s\r\n",bandStrings[choice]);

		print_application_config();
		appTaskState = JOIN_SEND_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
	else
	{
		printf("Restoration failed\r\n");
		appTaskState = DEMO_APP_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
}

/*********************************************************************//**
\brief    Sends Join request or Data to the network
*************************************************************************/
static void processJoinAndSend(void)
{
	StackRetStatus_t status = LORAWAN_SUCCESS;
	if(serialBuffer == '1')
	{
		status = LORAWAN_Join(demoJoinActivationType);
		if (LORAWAN_SUCCESS == (StackRetStatus_t)status)
		{
			set_LED_data(LED_GREEN,&on);
			printf("\nJoin Request Sent\n\r");

		}
		else
		{
			set_LED_data(LED_AMBER,&on);
			print_stack_status(status);
			appTaskState = JOIN_SEND_STATE;
			appPostTask(DISPLAY_TASK_HANDLER);
		}
	}
	else if(serialBuffer == '2' && joined == true)
	{
		sendData();
	}
	else if(serialBuffer == '2' && !joined)
	{
		set_LED_data(LED_AMBER,&on);
		printf("Device not joined to the network\r\n");
		appTaskState = JOIN_SEND_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
#ifdef CONF_PMM_ENABLE
	else if(serialBuffer == '3')
	{
		static bool deviceResetsForWakeup = false;
		PMM_SleepReq_t sleepReq;
		/* Put the application to sleep */
		sleepReq.sleepTimeMs = DEMO_CONF_DEFAULT_APP_SLEEP_TIME_MS;
		sleepReq.pmmWakeupCallback = appWakeup;
		sleepReq.sleep_mode = CONF_PMM_SLEEPMODE_WHEN_IDLE;
		if (CONF_PMM_SLEEPMODE_WHEN_IDLE == SLEEP_MODE_STANDBY)
		{
			deviceResetsForWakeup = false;
		}
		if (true == LORAWAN_ReadyToSleep(deviceResetsForWakeup))
		{
			app_resources_uninit();
			if (PMM_SLEEP_REQ_DENIED == PMM_Sleep(&sleepReq))
			{
				HAL_Radio_resources_init();
				sio2host_init();
				appTaskState = JOIN_SEND_STATE;
				appPostTask(DISPLAY_TASK_HANDLER);
				printf("\r\nsleep_not_ok\r\n");
			}
		}
		else
		{
			printf("\r\nsleep_not_ok\r\n");
			appTaskState = JOIN_SEND_STATE;
			appPostTask(DISPLAY_TASK_HANDLER);
		}
	}
	else if(serialBuffer == '4')
#else
	else if(serialBuffer == '3')
#endif
	{
		appTaskState = DEMO_APP_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
	else
	{
		set_LED_data(LED_AMBER,&on);
		printf("Invalid choice entered\r\n");
		appTaskState = JOIN_SEND_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
}

/*********************************************************************//**
\brief    Runs the Demo application
*************************************************************************/
static void processRunDemoApp(void)
{
	uint8_t num = serialBuffer - '0';
	if(num == sizeof(bandTable)-1)
	{
		NVIC_SystemReset();
	}
	else if(num == sizeof(bandTable)-2)
	{
		PDS_DeleteAll();
		appTaskState = DEMO_APP_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
	else if(num >0 && num < sizeof(bandTable) -2)
	{
		LORAWAN_Reset(bandTable[num]);
		#if (RANDOM_NW_ACQ == 0)
			if (bandTable[num] == ISM_NA915 || bandTable[num] == ISM_AU915)
			{
				printf("\r\nEnter SUBBAND(1-8): ");
				subBand = sio2host_getchar() - '0';
			}
		#endif
		mote_set_parameters(bandTable[num],num);
		set_LED_data(LED_GREEN,&on);
	}
	else
	{
		printf("Not a valid regional band choice\r\n");
		appTaskState = DEMO_APP_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
}

/*********************************************************************//**
\brief    configures the configuration for Demo application
*************************************************************************/
static void processAppConfig(void)
{
	uint8_t choice = serialBuffer - '0';
	if(joinTypeSelection == false && choice >= 1 && choice <= 2)
	{
		if (choice == 1) demoJoinActivationType = OVER_THE_AIR_ACTIVATION;
		else demoJoinActivationType = ACTIVATION_BY_PERSONALIZATION;
		joinTypeSelection = true;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
	else if (joinTypeSelection == false && choice < 0 && choice > 2)
	{
		printf("Not a valid join type\r\n");
		appPostTask(DISPLAY_TASK_HANDLER);
	}
	else if (joinTypeSelection == true &&  keyConfigSelection == false)
	{
		demoAddr_keyconfig_handler();
		if ((devEuiSelectionDone && joinEuiSelectionDone && appKeySelectionDone) ||
		(devAddrSelectionDone && appSkeySelectionDone && nwkSkeySelectionDone)) keyConfigSelection = true;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
	else if (joinTypeSelection == true &&  keyConfigSelection == true && verifyDemoConfigSelection == false)
	{
		if (choice == 1)
		{
			verifyDemoConfigSelection = true;
		}
		else if (choice == 2)
		{
			joinTypeSelection = false;
			keyConfigSelection = false;
			devEuiSelectionDone = false;
			joinEuiSelectionDone = false;
			appKeySelectionDone = false;
			devAddrSelectionDone = false;
			appSkeySelectionDone = false;
			nwkSkeySelectionDone = false;
		}
		appPostTask(DISPLAY_TASK_HANDLER);
	}
}
/*********************************************************************//**
\brief    Displays and activates LED's to choose between Demo
		  and Certification application
*************************************************************************/
static void displayRunDemoCertApp(void)
{
	//sio2host_rx(rxchar,10);
	set_LED_data(LED_AMBER,&off);
	set_LED_data(LED_GREEN,&off);
	printf("\r\n--Choose Application Type--\r\n");
	printf("1. Demo application\r\n");
	printf("\r\n Select Application : ");
	startReceiving = true;
}

/*********************************************************************//**
\brief    Activates LED's to indicate restoring of band
*************************************************************************/
static void displayRunRestoreBand(void)
{
	//sio2host_rx(rxchar,10);
	set_LED_data(LED_AMBER,&off);
	set_LED_data(LED_GREEN,&off);
	appPostTask(PROCESS_TASK_HANDLER);
}


/*********************************************************************//**
\brief    Display App Configuration Options Available
*************************************************************************/
static void displayAppConfigMenu(void)
{
	if (joinTypeSelection == false)
	{
		printf("\r\n---Join Type Selection---\r\n");
		printf("1. Over The Air Activation(OTAA)\r\n");
		printf("2. Activation By Personalization(ABP)\r\n");
		printf("\r\nEnter your choice: ");
		startReceiving = true;
	}
	else if (joinTypeSelection == true && keyConfigSelection == false)
	{
		if (demoJoinActivationType == OVER_THE_AIR_ACTIVATION)
		{
			if (devEuiSelectionDone == false) printf("\r\nEnter Device EUI(hex 0-F): ");
			else if (joinEuiSelectionDone == false) printf("\r\nEnter Join EUI(hex 0-F): ");
			else if (appKeySelectionDone == false) printf("\r\nEnter Application Key(hex 0-F): ");
		}
		else if (demoJoinActivationType == ACTIVATION_BY_PERSONALIZATION)
		{
			if (devAddrSelectionDone == false) printf("\r\nEnter Device Address(hex 0-F): ");
			else if (appSkeySelectionDone == false) printf("\r\nEnter Application Session Key(hex 0-F): ");
			else if (nwkSkeySelectionDone == false) printf("\r\nEnter Network Session Key(hex 0-F): ");
		}
		appPostTask(PROCESS_TASK_HANDLER);
		startReceiving = false;
	}
	else if (joinTypeSelection == true && keyConfigSelection == true && verifyDemoConfigSelection == false)
	{
		printf("\r\n\r\n---Configuration Selected ---\r\n");
		displaySelectedAppConfig();
		printf("\r\n1. Confirm App Configuration Selection\r\n");
		printf("2. Modify Configuration \r\n");
		printf("Enter your choice: ");
		startReceiving = true;
	}
	else if (joinTypeSelection == true && keyConfigSelection == true && verifyDemoConfigSelection == true)
	{
		appTaskState = DEMO_CERT_APP_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
	}
}

/*********************************************************************//**
\brief    Displays and activates LED's for joining to a network
		  and sending data to a network
*************************************************************************/
static void displayJoinAndSend(void)
{
    printf("\r\n1. Send Join Request\r\n");
    printf("2. Send Data\r\n");
#ifdef CONF_PMM_ENABLE
    printf("3. Sleep\r\n");
    printf("4. Select Band\r\n");
#else
    printf("3. Select Band\r\n");
#endif /* CONF_PMM_ENABLE */
    printf("\r\nEnter your choice: ");
    set_LED_data(LED_AMBER,&off);
    set_LED_data(LED_GREEN,&off);
	startReceiving = true;
}

/*********************************************************************//**
\brief    Displays and activates LED's for selecting Demo application
*************************************************************************/
static void displayRunDemoApp(void)
{
	uint8_t i = 0;

    set_LED_data(LED_AMBER,&off);
    set_LED_data(LED_GREEN,&off);

    printf("\r\nPlease select one of the band given below\r\n");
    for(i = 1;i < sizeof(bandTable); i++)
    {
	    printf("%d. %s\r\n",i,bandStrings[i]);
    }

    printf("Select Regional Band : ");
	startReceiving = true;
}

/*********************************************************************//**
\brief    Initialization the Demo application
*************************************************************************/
void mote_demo_init(void)
{
    bool status = false;
    /* Initialize the resources */
    resource_init();
	  startReceiving = false;
    /* Initialize the LORAWAN Stack */
    LORAWAN_Init(demo_appdata_callback, demo_joindata_callback);
    printf("\n\n\r*******************************************************\n\r");
    printf("\n\rMicrochip LoRaWAN Stack %s\r\n",STACK_VER);
    printf("\r\nInit - Successful\r\n");

    status = PDS_IsRestorable();
    if(status)
    {
        static uint8_t prevBand = 0xFF;
        uint8_t prevChoice = 0xFF;
        PDS_RestoreAll();
        LORAWAN_GetAttr(ISMBAND,NULL,&prevBand);
        for (uint32_t i = 0; i < sizeof(bandTable) -1; i++)
        {
            if(bandTable[i] == prevBand)
            {
                prevChoice = i;
                break;
            }
        }
        memset(rxchar,0,sizeof(rxchar));
        sio2host_rx(rxchar,10);
        printf ("Last configured Regional band %s\r\n",bandStrings[prevChoice]);
        printf("Press any key to change band\r\n Continuing in %s in ", bandStrings[prevChoice]);

        SwTimerStart(demoTimerId,MS_TO_US(1000),SW_TIMEOUT_RELATIVE,(void *)demoTimerCb,NULL);
    }
    else
    {
		appTaskState = DEMO_APP_CONFIG;
        appPostTask(DISPLAY_TASK_HANDLER);
    }
}

/*********************************************************************//*
 \brief      Function that processes the Rx data
 \param[in]  data - Rx data payload
 \param[in]  dataLen - The number of Rx bytes
 ************************************************************************/
static void demo_handle_evt_rx_data(void *appHandle, appCbParams_t *appdata)
{
    uint8_t *pData = appdata->param.rxData.pData;
    uint8_t dataLength = appdata->param.rxData.dataLength;
    uint32_t devAddress = appdata->param.rxData.devAddr;

    //Successful transmission
    if((dataLength > 0U) && (NULL != pData))
    {
        printf("*** Received DL Data ***\n\r");
        printf("\nFrame Received at port %d\n\r",pData[0]);
        printf("\nFrame Length - %d\n\r",dataLength);
        printf("\nAddress - 0x%lx\n\r", devAddress);
        printf ("\nPayload: ");
        for (uint8_t i =0; i<dataLength - 1; i++)
        {
            printf("%x",pData[i+1]);
        }
        printf("\r\n*************************\r\n");
    }
    else
    {
        printf("Received ACK for Confirmed data\r\n");
    }
}

/*********************************************************************//**
\brief Callback function for the ending of Bidirectional communication of
       Application data
 *************************************************************************/
void demo_appdata_callback(void *appHandle, appCbParams_t *appdata)
{
    StackRetStatus_t status = LORAWAN_INVALID_REQUEST;

    if (LORAWAN_EVT_RX_DATA_AVAILABLE == appdata->evt)
    {
        status = appdata->param.rxData.status;
        switch(status)
        {
            case LORAWAN_SUCCESS:
            {
                demo_handle_evt_rx_data(appHandle, appdata);
            }
            break;
            case LORAWAN_RADIO_NO_DATA:
            {
                printf("\n\rRADIO_NO_DATA \n\r");
            }
            break;
            case LORAWAN_RADIO_DATA_SIZE:
                printf("\n\rRADIO_DATA_SIZE \n\r");
            break;
            case LORAWAN_RADIO_INVALID_REQ:
                printf("\n\rRADIO_INVALID_REQ \n\r");
            break;
            case LORAWAN_RADIO_BUSY:
                printf("\n\rRADIO_BUSY \n\r");
            break;
            case LORAWAN_RADIO_OUT_OF_RANGE:
                printf("\n\rRADIO_OUT_OF_RANGE \n\r");
            break;
            case LORAWAN_RADIO_UNSUPPORTED_ATTR:
                printf("\n\rRADIO_UNSUPPORTED_ATTR \n\r");
            break;
            case LORAWAN_RADIO_CHANNEL_BUSY:
                printf("\n\rRADIO_CHANNEL_BUSY \n\r");
            break;
            case LORAWAN_NWK_NOT_JOINED:
                printf("\n\rNWK_NOT_JOINED \n\r");
            break;
            case LORAWAN_INVALID_PARAMETER:
                printf("\n\rINVALID_PARAMETER \n\r");
            break;
            case LORAWAN_KEYS_NOT_INITIALIZED:
                printf("\n\rKEYS_NOT_INITIALIZED \n\r");
            break;
            case LORAWAN_SILENT_IMMEDIATELY_ACTIVE:
                printf("\n\rSILENT_IMMEDIATELY_ACTIVE\n\r");
            break;
            case LORAWAN_FCNTR_ERROR_REJOIN_NEEDED:
                printf("\n\rFCNTR_ERROR_REJOIN_NEEDED \n\r");
            break;
            case LORAWAN_INVALID_BUFFER_LENGTH:
                printf("\n\rINVALID_BUFFER_LENGTH \n\r");
            break;
            case LORAWAN_MAC_PAUSED :
                printf("\n\rMAC_PAUSED  \n\r");
            break;
            case LORAWAN_NO_CHANNELS_FOUND:
                printf("\n\rNO_CHANNELS_FOUND \n\r");
            break;
            case LORAWAN_BUSY:
                printf("\n\rBUSY\n\r");
            break;
            case LORAWAN_NO_ACK:
                printf("\n\rNO_ACK \n\r");
            break;
            case LORAWAN_NWK_JOIN_IN_PROGRESS:
                printf("\n\rALREADY JOINING IS IN PROGRESS \n\r");
            break;
            case LORAWAN_RESOURCE_UNAVAILABLE:
                printf("\n\rRESOURCE_UNAVAILABLE \n\r");
            break;
            case LORAWAN_INVALID_REQUEST:
                printf("\n\rINVALID_REQUEST \n\r");
            break;
            case LORAWAN_FCNTR_ERROR:
                printf("\n\rFCNTR_ERROR \n\r");
            break;
            case LORAWAN_MIC_ERROR:
                printf("\n\rMIC_ERROR \n\r");
            break;
            case LORAWAN_INVALID_MTYPE:
                printf("\n\rINVALID_MTYPE \n\r");
            break;
            case LORAWAN_MCAST_HDR_INVALID:
                printf("\n\rMCAST_HDR_INVALID \n\r");
            break;
			case LORAWAN_INVALID_PACKET:
				printf("\n\rINVALID_PACKET \n\r");
			break;
            default:
                printf("UNKNOWN ERROR\n\r");
            break;
        }
    }
    else if(LORAWAN_EVT_TRANSACTION_COMPLETE == appdata->evt)
    {
        switch(status = appdata->param.transCmpl.status)
        {
            case LORAWAN_SUCCESS:
            {
                printf("Transmission Success\r\n");
            }
            break;
            case LORAWAN_RADIO_SUCCESS:
            {
                printf("Transmission Success\r\n");
            }
            break;
            case LORAWAN_RADIO_NO_DATA:
            {
                printf("\n\rRADIO_NO_DATA \n\r");
            }
            break;
            case LORAWAN_RADIO_DATA_SIZE:
                printf("\n\rRADIO_DATA_SIZE \n\r");
            break;
            case LORAWAN_RADIO_INVALID_REQ:
                printf("\n\rRADIO_INVALID_REQ \n\r");
            break;
            case LORAWAN_RADIO_BUSY:
                printf("\n\rRADIO_BUSY \n\r");
            break;
            case LORAWAN_TX_TIMEOUT:
                printf("\nTx Timeout\n\r");
            break;
            case LORAWAN_RADIO_OUT_OF_RANGE:
                printf("\n\rRADIO_OUT_OF_RANGE \n\r");
            break;
            case LORAWAN_RADIO_UNSUPPORTED_ATTR:
                printf("\n\rRADIO_UNSUPPORTED_ATTR \n\r");
            break;
            case LORAWAN_RADIO_CHANNEL_BUSY:
                printf("\n\rRADIO_CHANNEL_BUSY \n\r");
            break;
            case LORAWAN_NWK_NOT_JOINED:
                printf("\n\rNWK_NOT_JOINED \n\r");
            break;
            case LORAWAN_INVALID_PARAMETER:
                printf("\n\rINVALID_PARAMETER \n\r");
            break;
            case LORAWAN_KEYS_NOT_INITIALIZED:
                printf("\n\rKEYS_NOT_INITIALIZED \n\r");
            break;
            case LORAWAN_SILENT_IMMEDIATELY_ACTIVE:
                printf("\n\rSILENT_IMMEDIATELY_ACTIVE\n\r");
            break;
            case LORAWAN_FCNTR_ERROR_REJOIN_NEEDED:
                printf("\n\rFCNTR_ERROR_REJOIN_NEEDED \n\r");
            break;
            case LORAWAN_INVALID_BUFFER_LENGTH:
                printf("\n\rINVALID_BUFFER_LENGTH \n\r");
            break;
            case LORAWAN_MAC_PAUSED :
                printf("\n\rMAC_PAUSED  \n\r");
            break;
            case LORAWAN_NO_CHANNELS_FOUND:
                printf("\n\rNO_CHANNELS_FOUND \n\r");
            break;
            case LORAWAN_BUSY:
                printf("\n\rBUSY\n\r");
            break;
            case LORAWAN_NO_ACK:
                printf("\n\rNO_ACK \n\r");
            break;
            case LORAWAN_NWK_JOIN_IN_PROGRESS:
                printf("\n\rALREADY JOINING IS IN PROGRESS \n\r");
            break;
            case LORAWAN_RESOURCE_UNAVAILABLE:
                printf("\n\rRESOURCE_UNAVAILABLE \n\r");
            break;
            case LORAWAN_INVALID_REQUEST:
                printf("\n\rINVALID_REQUEST \n\r");
            break;
            case LORAWAN_FCNTR_ERROR:
                printf("\n\rFCNTR_ERROR \n\r");
            break;
            case LORAWAN_MIC_ERROR:
                printf("\n\rMIC_ERROR \n\r");
            break;
            case LORAWAN_INVALID_MTYPE:
                printf("\n\rINVALID_MTYPE \n\r");
            break;
            case LORAWAN_MCAST_HDR_INVALID:
                printf("\n\rMCAST_HDR_INVALID \n\r");
            break;
			case LORAWAN_INVALID_PACKET:
				printf("\n\rINVALID_PACKET \n\r");
			break;
            default:
                printf("\n\rUNKNOWN ERROR\n\r");
            break;

                    }
        printf("\n\r*************************************************\n\r");
    }

    SwTimerStop(lTimerId);
    set_LED_data(LED_GREEN,&off);
    if(status != LORAWAN_SUCCESS)
    {
        set_LED_data(LED_AMBER,&on);
    }
	appTaskState = JOIN_SEND_STATE;
    appPostTask(DISPLAY_TASK_HANDLER);
}

/*********************************************************************//*
\brief Callback function for the ending of Activation procedure
 ************************************************************************/
void demo_joindata_callback(StackRetStatus_t status)
{
    /* This is called every time the join process is finished */
    set_LED_data(LED_GREEN,&off);
    if(LORAWAN_SUCCESS == status)
    {
        uint32_t devAddress;
        bool mcastEnabled;

        joined = true;
        printf("\nJoining Successful\n\r");
        LORAWAN_GetAttr(DEV_ADDR, NULL, &devAddress);
        LORAWAN_GetAttr(MCAST_ENABLE, NULL, &mcastEnabled);

        if (devAddress != DEMO_APP_MCAST_GROUP_ADDRESS)
        {
            printf("\nDevAddr: 0x%lx\n\r", devAddress);
        }
        else if ((devAddress == DEMO_APP_MCAST_GROUP_ADDRESS) && (true == mcastEnabled))
        {
            printf("\nAddress conflict between Device Address and Multicast group address\n\r");
        }
        print_application_config();
        set_LED_data(LED_GREEN,&on);
    }
	else if(LORAWAN_NO_CHANNELS_FOUND == status)
	{
		joined = false;
		set_LED_data(LED_AMBER,&on);
		printf("\n No Free Channel found");
	}
  else if (LORAWAN_MIC_ERROR == status)
	{
		joined = false;
		set_LED_data(LED_AMBER,&on);
		printf("\n MIC Error");
	}
	else if (LORAWAN_TX_TIMEOUT == status)
	{
		joined = false;
		set_LED_data(LED_AMBER,&on);
		printf("\n Transmission Timeout");
	}
    else
    {
        joined = false;
        set_LED_data(LED_AMBER,&on);
        printf("\nJoining Denied\n\r");
    }
    printf("\n\r*******************************************************\n\r");
    PDS_StoreAll();

	appTaskState = JOIN_SEND_STATE;
    appPostTask(DISPLAY_TASK_HANDLER);
}

void lTimerCb(void *data)
{
    SwTimerStart(lTimerId,MS_TO_US(100),SW_TIMEOUT_RELATIVE,(void *)lTimerCb,NULL);
    set_LED_data(LED_GREEN,&toggle);
}

/*********************************************************************//*
 \brief      Function to send data from end device to application server
  ************************************************************************/
void sendData(void)
{
    int status = -1;
	uint8_t avail_payload;
    /* Read temperature sensor value */
    get_resource_data(TEMP_SENSOR,(uint8_t *)&cel_val);
    fahren_val = convert_celsius_to_fahrenheit(cel_val);
    printf("\nTemperature:");
    snprintf(temp_sen_str,sizeof(temp_sen_str),"%.1fC/%.1fF\n", cel_val, fahren_val);
    printf("%.1f\xf8 C/%.1f\xf8 F\n\r", cel_val, fahren_val);

    data_len = strlen(temp_sen_str);
    lorawanSendReq.buffer = &temp_sen_str;
    lorawanSendReq.bufferLength = data_len - 1;
    lorawanSendReq.confirmed = DEMO_APP_TRANSMISSION_TYPE;
    lorawanSendReq.port = DEMO_APP_FPORT;
	LORAWAN_GetAttr(NEXT_PAYLOAD_SIZE, NULL, &avail_payload);
	if (avail_payload < lorawanSendReq.bufferLength)
	{
		// At DR0 for NA and AU regions Max payload = 3 bytes or less, due to FHDR(7) and FPORT(1) byte 
		printf("\r\nSending %d bytes of payload - DR limitation\r\n", avail_payload);
		lorawanSendReq.bufferLength = avail_payload;
	}
    status = LORAWAN_Send(&lorawanSendReq);
    if (LORAWAN_SUCCESS == status)
    {
        printf("\nTx Data Sent \r\n");
        set_LED_data(LED_GREEN,&on);
        SwTimerStart(lTimerId,MS_TO_US(100),SW_TIMEOUT_RELATIVE,(void *)lTimerCb,NULL);
    }
    else
    {
        print_stack_status(status);
		appTaskState = JOIN_SEND_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
    }
}

#ifdef CONF_PMM_ENABLE
static void appWakeup(uint32_t sleptDuration)
{
    HAL_Radio_resources_init();
    sio2host_init();
	appTaskState = JOIN_SEND_STATE;
    appPostTask(DISPLAY_TASK_HANDLER);
    printf("\r\nsleep_ok %ld ms\r\n", sleptDuration);
}
#endif

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
    sio2host_deinit();
    /* Disable Transceiver SPI Module */
    HAL_RadioDeInit();
}
#endif

/*********************************************************************//*
 \brief      Timer callback for demo application.
             Used during the initial 5 sec wait period.
 \param[in]  cnt - not used
 ************************************************************************/
void demoTimerCb(void * cnt)
{
    uint8_t i = 10;
    int8_t rxdata = 0;
    printf("%d..",count);
    count--;
	startReceiving = false;
    sio2host_rx(rxchar,10);
    for(i = 0;i<=10;i++)
    {
        if(rxchar[i] != 13 && rxchar[i] != 10)
        {
            rxdata = rxchar[i];
            break;
        }
    }
    if(!count)
    {
        printf("\r\n");
    }
    /* No input so far. start timer till expiry */
    if(count > 0 && (!rxdata))
    {
        SwTimerStart(demoTimerId,MS_TO_US(1000),SW_TIMEOUT_RELATIVE,(void *)demoTimerCb,NULL);
    }
    /* user did not press any input */
    else if(count == 0 && (!rxdata))
    {
		appTaskState = RESTORE_BAND_STATE;
        appPostTask(DISPLAY_TASK_HANDLER);
    }
    /* User pressed a key */
    else if(rxdata)
    {
        printf("\r\n");
		appTaskState = DEMO_APP_CONFIG;
        appPostTask(DISPLAY_TASK_HANDLER);
    }

}

/*********************************************************************//*
 \brief      App Post Task
 \param[in]  Id of the application to be posted
 ************************************************************************/

void appPostTask(AppTaskIds_t id)
{
    ATOMIC_SECTION_ENTER
    appTaskFlags |= (1 << id);
    ATOMIC_SECTION_EXIT

    /* Also post a APP task to the system */
    SYSTEM_PostTask(APP_TASK_ID);
}

/*********************************************************************//*
 \brief      Application Task Handler
 ************************************************************************/

SYSTEM_TaskStatus_t APP_TaskHandler(void)
{

    if (appTaskFlags)
    {
        for (uint16_t taskId = 0; taskId < APP_TASKS_COUNT; taskId++)
        {
            if ((1 << taskId) & (appTaskFlags))
            {
                ATOMIC_SECTION_ENTER
                appTaskFlags &= ~(1 << taskId);
                ATOMIC_SECTION_EXIT

                appTaskHandlers[taskId]();

                if (appTaskFlags)
                {
                    SYSTEM_PostTask(APP_TASK_ID);
                }

                break;
            }
        }
    }

    return SYSTEM_TASK_SUCCESS;
}

/*********************************************************************//*
 \brief      Set join parameters function
 \param[in]  activation type - notifies the activation type (OTAA/ABP)
 \return     LORAWAN_SUCCESS, if successfully set the join parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t set_join_parameters(ActivationType_t activation_type)
{
    StackRetStatus_t status;

    printf("\n********************Join Parameters********************\n\r");

    if(ACTIVATION_BY_PERSONALIZATION == activation_type)
    {
        status = LORAWAN_SetAttr (DEV_ADDR, &demoDevAddr);
        if (LORAWAN_SUCCESS == status)
        {
            status = LORAWAN_SetAttr (APPS_KEY, demoAppsKey);
        }

        if (LORAWAN_SUCCESS == status)
        {
            printf("\nAppSessionKey : ");
            print_array((uint8_t *)&demoAppsKey, sizeof(demoAppsKey));
            status = LORAWAN_SetAttr (NWKS_KEY, demoNwksKey);
        }

        if (LORAWAN_SUCCESS == status)
        {
            printf("\nNwkSessionKey : ");
            print_array((uint8_t *)&demoNwksKey, sizeof(demoNwksKey));
        }

    }
    else
    {
        status = LORAWAN_SetAttr (DEV_EUI, demoDevEui);
        if (LORAWAN_SUCCESS == status)
        {
            printf("\nDevEUI : ");
            print_array((uint8_t *)&demoDevEui, sizeof(demoDevEui));
            status = LORAWAN_SetAttr (JOIN_EUI, demoJoinEui);
        }

        if (LORAWAN_SUCCESS == status)
        {
            printf("\nJoinEUI : ");
            print_array((uint8_t *)&demoJoinEui, sizeof(demoJoinEui));
            status = LORAWAN_SetAttr (APP_KEY, demoAppKey);
        }

        if (LORAWAN_SUCCESS == status)
        {
            printf("\nAppKey : ");
            print_array((uint8_t *)&demoAppKey, sizeof(demoAppKey));
        }
    }
    return status;
}

/*********************************************************************//*
 \brief      Function to Initialize the device type
 \param[in]  ed_class - notifies the device class (CLASS_A/CLASS_B/CLASS_C)
 \return     LORAWAN_SUCCESS, if successfully set the device class
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t set_device_type(EdClass_t ed_class)
{
    StackRetStatus_t status = LORAWAN_SUCCESS;

    status = LORAWAN_SetAttr(EDCLASS, &ed_class);

    if((LORAWAN_SUCCESS == status) && ((CLASS_C | CLASS_B) & ed_class) && (true == DEMO_APP_MCAST_ENABLE))
    {
        set_multicast_params();
    }

    return status;
}

/*********************************************************************//*
 \brief      Function to Initialize the Multicast parameters
 ************************************************************************/
void set_multicast_params (void)
{
    StackRetStatus_t status;
    LorawanMcastDevAddr_t dMcastDevAddr;
    LorawanMcastAppSkey_t mcastAppSKey;
    LorawanMcastNwkSkey_t mcastNwkSKey;
    LorawanMcastDlFreqeuncy_t mcastDlFreq;
	  LorawanMcastDatarate_t mcastDatarate;
    LorawanMcastStatus_t  mcastStatus;
    ReceiveWindow2Params_t receivewindow2param;

    printf("\n***************Multicast Parameters********************\n\r");

    dMcastDevAddr.groupId = demoMcastGroupId;
    mcastAppSKey.groupId  = demoMcastGroupId;
    mcastNwkSKey.groupId  = demoMcastGroupId;
    mcastDlFreq.groupId   = demoMcastGroupId;
	  mcastDatarate.groupId = demoMcastGroupId;
    mcastStatus.groupId   = demoMcastGroupId;

    memcpy(&(mcastAppSKey.mcastAppSKey), &demoMcastAppsKey,LORAWAN_SESSIONKEY_LENGTH);
    dMcastDevAddr.mcast_dev_addr = demoMcastDevAddr;
    memcpy(&(mcastNwkSKey.mcastNwkSKey), &demoMcastNwksKey,LORAWAN_SESSIONKEY_LENGTH);
    memcpy(&(mcastStatus.status),&demoMcastEnable,sizeof(demoMcastEnable));
    	LORAWAN_GetAttr(RX2_WINDOW_PARAMS ,NULL, &receivewindow2param);
	mcastDatarate.datarate = receivewindow2param.dataRate;
	mcastDlFreq.dlFrequency = receivewindow2param.frequency;

    status = LORAWAN_SetAttr(MCAST_APPS_KEY, &mcastAppSKey);
    if (status == LORAWAN_SUCCESS)
    {
	    printf("\nMcastAppSessionKey : ");
	    print_array((uint8_t *)&(mcastAppSKey.mcastAppSKey), LORAWAN_SESSIONKEY_LENGTH);
	    status = LORAWAN_SetAttr(MCAST_NWKS_KEY, &mcastNwkSKey);
    }

    if(status == LORAWAN_SUCCESS)
    {
	    printf("\nMcastNwkSessionKey : ");
	    print_array((uint8_t *)&(mcastNwkSKey.mcastNwkSKey), LORAWAN_SESSIONKEY_LENGTH);
	    status = LORAWAN_SetAttr(MCAST_GROUP_ADDR, &dMcastDevAddr);
    }
    if (status == LORAWAN_SUCCESS)
    {
	    printf("\nMcastGroupAddr : 0x%lx\n\r", dMcastDevAddr.mcast_dev_addr);
	    status = LORAWAN_SetAttr(MCAST_ENABLE, &mcastStatus);
    }
    if (status == LORAWAN_SUCCESS)
	  {
	    status = LORAWAN_SetMulticastParam(MCAST_DATARATE , &mcastDatarate);
	  }
	  if (status == LORAWAN_SUCCESS)
	  {
	   status = LORAWAN_SetMulticastParam(MCAST_FREQUENCY , &mcastDlFreq);
	  }
    else
    {
	    printf("\nMcastGroupAddrStatus : Failed\n\r");
    }

    if (status == LORAWAN_SUCCESS)
    {
	    printf("\nMulticastStatus : Enabled\n\r");
    }
    else
    {
	    printf("\nMulticastStatus : Failed\n\r");
    }

	 printf("\n********************************************************\n\r");
}

/***********************************************************************
 \brief      Function to Initialize set default parameters
 \param[in]  void
 \return     LORAWAN_SUCCESS, if successfully set all the parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t mote_set_parameters(IsmBand_t ismBand, const uint16_t index)
{
    StackRetStatus_t status;
    bool joinBackoffEnable = false;
    LORAWAN_Reset(ismBand);
#if (NA_BAND == 1 || AU_BAND == 1)
#if (RANDOM_NW_ACQ == 0)
    if ((ismBand == ISM_NA915) || (ismBand == ISM_AU915))
    {
        #define MAX_NA_CHANNELS 72
        #define MAX_SUBBAND_CHANNELS 8

        ChannelParameters_t ch_params;

        uint8_t allowed_min_125khz_ch,allowed_max_125khz_ch,allowed_500khz_channel;

        allowed_min_125khz_ch = (subBand-1)*MAX_SUBBAND_CHANNELS;

        allowed_max_125khz_ch = ((subBand-1)*MAX_SUBBAND_CHANNELS) + 7 ;

        allowed_500khz_channel = subBand+63;

        for (ch_params.channelId = 0; ch_params.channelId < MAX_NA_CHANNELS; ch_params.channelId++)
        {
            if((ch_params.channelId >= allowed_min_125khz_ch) && (ch_params.channelId <= allowed_max_125khz_ch))
            {
                ch_params.channelAttr.status = true;
            }
            else if(ch_params.channelId == allowed_500khz_channel)
            {
                ch_params.channelAttr.status = true;
            }
            else
            {
                ch_params.channelAttr.status = false;
            }

            LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
        }
    }
#endif
#endif
    /*Disabled Join backoff in Demo application
	Needs to be enabled in Production Environment Ref Section */
    LORAWAN_SetAttr(JOIN_BACKOFF_ENABLE,&joinBackoffEnable);

    /* Initialize the join parameters for Demo application */
    status = set_join_parameters(demoJoinActivationType);

    if (LORAWAN_SUCCESS != status)
    {
        printf("\nJoin parameters initialization failed\n\r");
        return status;
    }

    /* Set the device type */
    status = set_device_type(DEMO_APP_ENDDEVICE_CLASS);

    if (LORAWAN_SUCCESS != status)
    {
        printf("\nUnsupported Device Type\n\r");
        return status;
    }


    /* Send Join request for Demo application */
    status = LORAWAN_Join(demoJoinActivationType);

    if (LORAWAN_SUCCESS == status && index < sizeof(bandTable))
    {
        printf("\nJoin Request Sent for %s\n\r",bandStrings[index]);
    }
    else
    {
        print_stack_status(status);
		appTaskState = JOIN_SEND_STATE;
		appPostTask(DISPLAY_TASK_HANDLER);
    }

    return status;
}

/*********************************************************************//*
 \brief      Function to Print array of characters
 \param[in]  *array  - Pointer of the array to be printed
 \param[in]   length - Length of the array
 ************************************************************************/
void print_array (uint8_t *array, uint8_t length)
{
    printf("0x");
    for (uint8_t i =0; i < length; i++)
    {
        printf("%02x", *array);
        array++;
    }
    printf("\n\r");
}

/*********************************************************************//*
 \brief      Function to Print application configuration
 ************************************************************************/
void  print_application_config (void)
{
    EdClass_t edClass;
    printf("\n***************Application Configuration***************\n\r");
    LORAWAN_GetAttr(EDCLASS, NULL, &edClass);
    printf("\nDevType : ");

    if(edClass == CLASS_A)
    {
        printf("CLASS A\n\r");
    }
    else if(edClass == CLASS_C)
    {
        printf("CLASS C\n\r");
    }

    printf("\nActivationType : ");

    if(demoJoinActivationType == OVER_THE_AIR_ACTIVATION)
    {
        printf("OTAA\n\r");
    }
    else if(demoJoinActivationType == ACTIVATION_BY_PERSONALIZATION)
    {
        printf("ABP\n\r");
    }

    printf("\nTransmission Type - ");

    if(DEMO_APP_TRANSMISSION_TYPE == CONFIRMED)
    {
        printf("CONFIRMED\n\r");
    }
    else if(DEMO_APP_TRANSMISSION_TYPE == UNCONFIRMED)
    {
        printf("UNCONFIRMED\n\r");
    }

    printf("\nFPort - %d\n\r", DEMO_APP_FPORT);

    printf("\n*******************************************************\n\r");
}

/*********************************************************************//*
 \brief      Function to Print stack return status
 \param[in]  status - Status from the stack
 ************************************************************************/
void print_stack_status(StackRetStatus_t status)
{
    switch(status)
    {
        case LORAWAN_SUCCESS:
             printf("\nlorawan_success\n\r");
        break;

        case LORAWAN_BUSY:
             printf("\nlorawan_state : stack_Busy\n\r");
        break;

        case LORAWAN_NWK_NOT_JOINED:
            printf("\ndevice_not_joined_to_network\n\r");
        break;

        case LORAWAN_INVALID_PARAMETER:
            printf("\ninvalid_parameter\n\r");
        break;

        case LORAWAN_KEYS_NOT_INITIALIZED:
            printf("\nkeys_not_initialized\n\r");
        break;

        case LORAWAN_SILENT_IMMEDIATELY_ACTIVE:
            printf("\nsilent_immediately_active\n\r");
        break;

        case LORAWAN_FCNTR_ERROR_REJOIN_NEEDED:
            printf("\nframecounter_error_rejoin_needed\n\r");
        break;

        case LORAWAN_INVALID_BUFFER_LENGTH:
            printf("\ninvalid_buffer_length\n\r");
        break;

        case LORAWAN_MAC_PAUSED:
            printf("\nMAC_paused\n\r");
        break;

        case LORAWAN_NO_CHANNELS_FOUND:
            printf("\nno_free_channels_found\n\r");
        break;

        case LORAWAN_INVALID_REQUEST:
            printf("\nrequest_invalid\n\r");
        break;
        case LORAWAN_NWK_JOIN_IN_PROGRESS:
            printf("\nprev_join_request_in_progress\n\r");
        break;
        default:
           printf("\nrequest_failed %d\n\r",status);
        break;
    }
}

/*********************************************************************//*
 \brief      Function to convert Celsius value to Fahrenheit
 \param[in]  cel_val   - Temperature value in Celsius
 \param[out] fauren_val- Temperature value in Fahrenheit
 ************************************************************************/
static float convert_celsius_to_fahrenheit(float celsius_val)
{
    float fauren_val;
    /* T(°F) = T(°C) × 9/5 + 32 */
    fauren_val = (((celsius_val * 9)/5) + 32);

    return fauren_val;

}



/* eof demo_app.c */
