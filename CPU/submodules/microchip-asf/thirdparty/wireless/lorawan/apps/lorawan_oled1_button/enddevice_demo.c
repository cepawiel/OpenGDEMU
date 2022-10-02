/**
* \file  enddevice_demo.c
*
* \brief LORAWAN Getting Started  OLED1[Button] Demo Application
*
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
* LORAWAN Getting Started LoRaWAN OLED1[Button] Demo Application main file available from Atmel Studio examples,
* this is used to send the temperature sensor data through the LoRaWAN network to the network server.
* <P>• This example provides an option for the  user to configure the regional band at run time.</P>
* <P>• Using this example application, OLED display menu operation is supported.</P>
* <P>• This example also showcases sleep functionality of the LoRaWAN Stack, and the Hardware.</P>
* <P>• This example demonstrates storing stack parameters in NVM using PDS. </P>
* <P>• This example uses the SW0 pushbutton of the SAMR34 Xplained Pro/WLR089 Xplained Pro to initiate a manual confirmed message transmission and to  select DR and TX PWR. </P>
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
#include "radio_interface.h"
#include "conf_pmm.h"
#include "conf_sio2host.h"
#include "pds_interface.h"
#include "lorawan_reg_params.h"
#include "conf_regparams.h"
#include "lorawan_multiband.h"
#include "oled1_support.h"
#include "conf_menu.h"



#if (EDBG_EUI_READ == 1)
#include "edbg_eui.h"
#endif
#include "atomic.h"
#include <stdint.h>
/******************************** MACROS ***************************************/

/************************** GLOBAL VARIABLES ***********************************/
uint8_t oled_select = 0xFF;

static float cel_val;
static float fahren_val;
static char temp_sen_str[25];
static uint8_t data_len = 0;

static uint8_t on = LON;
static uint8_t off = LOFF;
static uint8_t toggle = LTOGGLE;

static volatile uint8_t appTaskFlags = 0x00u;

/* Default Regional band start delay time ( provides a time window to allow user to change previous ISM_Band setting*/
static uint8_t count = 5;

static uint8_t rxchar[11];
static bool startReceiving = false;
static bool try_join = false;
static uint8_t blinkcnt = 0xFF;
static bool greenBlink = true;
bool any_button = false;

static AppTaskState_t appTaskState;

extern uint8_t AppTimerId;
extern uint8_t demoITimerId;
extern uint8_t lTimerId;
extern uint8_t keycode;
// devAddr for ABP stored for display
uint8_t d_Addr[4];

//128x32 display
//5x7 font	(5 [x] ; 7 [y])
//up to 4 lines
//22 chars per line (max value)
typedef enum lineY {LINE1 = 0, LINE2 = 8, LINE3 = 17, LINE4 = 25} Ly;
#define OLED1_EXT_HEADER  EXT1
OLED1_CREATE_INSTANCE(oled1, OLED1_EXT_HEADER);

//Note: NOT CURRENTLY USED -- option to clear a line is to draw string with empty line
#define CLEAR_LINE1()	gfx_mono_draw_string("                      ", 0, LINE1, &sysfont) ;
#define CLEAR_LINE2()	gfx_mono_draw_string("                      ", 0, LINE2, &sysfont) ;
#define CLEAR_LINE3()	gfx_mono_draw_string("                      ", 0, LINE3, &sysfont) ;
#define CLEAR_LINE4()	gfx_mono_draw_string("                      ", 0, LINE4, &sysfont) ;
#define CLEAR_DISPLAY()	CLEAR_LINE1() ; CLEAR_LINE2() ; CLEAR_LINE3() ; CLEAR_LINE4() ;


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

/* ABP and OTAA settings are located in conf_app.h */
/*ABP Join Parameters */
static uint32_t demoDevAddr = DEMO_DEVICE_ADDRESS;
static uint8_t demoNwksKey[16] = DEMO_NETWORK_SESSION_KEY;
static uint8_t demoAppsKey[16] = DEMO_APPLICATION_SESSION_KEY;
/* OTAA join parameters */
static uint8_t demoDevEui[8] = DEMO_DEVICE_EUI;
static uint8_t demoJoinEui[8] = DEMO_JOIN_EUI;
static uint8_t demoAppKey[16] = DEMO_APPLICATION_KEY;

static LorawanSendReq_t lorawanSendReq;
static char serialBuffer;

/* Muticast Parameters */
static bool demoMcastEnable = DEMO_APP_MCAST_ENABLE;
static uint32_t demoMcastDevAddr = DEMO_APP_MCAST_GROUP_ADDRESS;
static uint8_t demoMcastNwksKey[16] = DEMO_APP_MCAST_NWK_SESSION_KEY;
static uint8_t demoMcastAppsKey[16] = DEMO_APP_MCAST_APP_SESSION_KEY;
static uint8_t demoMcastGroupId = DEMO_APP_MCAST_GROUPID;
/************************** EXTERN VARIABLES ***********************************/


/************************** FUNCTION PROTOTYPES ********************************/
static void sendData(TransmissionType_t type);
static void appPostTask(AppTaskIds_t id);
static SYSTEM_TaskStatus_t (*appTaskHandlers[])(void);
static void demoITimerCb(void * cnt);
static void lTimerCb(void * data);
static void appTimerCb(void *data);
void button_read (void);
static SYSTEM_TaskStatus_t processTask(void);
static SYSTEM_TaskStatus_t buttonTask(void);
static void processRunRestoreBand(void);
static void processJoinAndSend(void);
static void processBandSelection(void);
static void Select_Band(void);
static void dev_eui_read(void);
SYSTEM_TaskStatus_t APP_TaskHandler(void);
static float convert_celsius_to_fahrenheit(float cel_val);
void periodicSend(void);


/*********************************************************************//*
 \brief      Function that processes the Rx data
 \param[in]  data - Rx data payload
 \param[in]  dataLen - The number of Rx bytes
 ************************************************************************/
static void demo_handle_evt_rx_data(void *appHandle, appCbParams_t *appdata);

/***************************** FUNCTIONS ***************************************/

static SYSTEM_TaskStatus_t (*appTaskHandlers[APP_TASKS_COUNT])(void) = {
    /* In the order of descending priority */
    processTask,
	buttonTask
};

void periodicSend(void)
{
	appTaskState = JOIN_SEND_STATE;
	appPostTask(PROCESS_TASK_HANDLER);
	app_gflags.sleep_en = false;  // do not allow sleep until data transaction has completed
}

/* Adjustment parameters to define the durations of long button press and extended long button press */
#define longcnt 5
#define extlongcnt  longcnt * 2
#define longcntreset extlongcnt + longcnt

/* Interrupt handler for Read  button press */
void button_read (void)
{
	uint8_t cnt = 0;
	// if not joined to a network ignore SW0 pushbutton presses
	if(app_gflags.joined)
	{
		/* Read the button level */
		if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
		{
			// turn off leds on pushbutton press ( just in case previous error indication still active )
			set_LED_data(LED_AMBER,&off);
			set_LED_data(LED_GREEN,&off);
			app_gflags.button_press = true;

			#ifdef CONF_PMM_ENABLE
			PMM_Wakeup();
			HAL_Radio_resources_init();
			sio2host_init();
			#endif
			/* Wait for button debounce time */
			delay_ms(APP_DEBOUNCE_TIME);
			/* Check whether button is in active state */
			while(port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
			{
				delay_ms(500);
				cnt += 1;
				if(cnt > longcntreset)
				{
					LED_Off(LED_0_PIN);  // turn off Amber LED to indicate long cnt reset button press'
					LED_Off(LED_1_PIN);  // turn off Green LED to indicate long cnt reset button press'
				}
				else if(cnt > extlongcnt)
					LED_On(LED_0_PIN);  // turn on Amber LED to indicate extended long button press'
				else if(cnt > longcnt)
					LED_On(LED_1_PIN);  // turn on Green LED to indicate long button press'

			}
			if(cnt > longcntreset)
				appTaskState = LONGBUTTON_RESET_PRESS_STATE;
			else if(cnt > extlongcnt)  // if extended long button press
				appTaskState = EXTLONGBUTTON_PRESS_STATE;
			else if((cnt > longcnt) && (cnt <= extlongcnt))  // if long button press
				appTaskState = LONGBUTTON_PRESS_STATE;
			else
				appTaskState = BUTTON_PRESS_STATE;

			/* Post task to application handler on button press */

			appPostTask(BUTTON_TASK_HANDLER);

		}
	}
}

/*********************************************************************//**
\brief    Task handler for processing SW0  user button press
\ Multiple functions assigned to the SW0 button in this application
*************************************************************************/
static SYSTEM_TaskStatus_t buttonTask(void)
{
	StackRetStatus_t status;
	uint8_t rate;
	uint8_t pIndex;
	uint8_t band;
	char str[20];

	switch(appTaskState)
	{
		case BUTTON_PRESS_STATE:					// Manual initiate a sensor transmission upon short button press
			SwTimerStop(AppTimerId);				// Stop Periodic application timer
			sendData(CONFIRMED);					// send  frame and restart Periodic application timer
			break;

		case LONGBUTTON_PRESS_STATE:				// Long Button press to adjust the current DataRate setting
			status = LORAWAN_GetAttr(CURRENT_DATARATE, 0, &rate);
			rate++;			// adjust rate ( increment)
			// ck for NA region
			if(rate > RegParams.minDataRate)
				rate = 0;	// force rate back to DR0

			status = 0;
		    do
			{
				status = LORAWAN_SetAttr (CURRENT_DATARATE, &rate);  // set data rate (DR)  attribute

				if (LORAWAN_SUCCESS == status)
				{
					printf("Set DateRate = DR%d Success\r\n",rate);
					// Clear screen
					gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
					sprintf(str,"DataRate set to DR%d",rate);
					gfx_mono_draw_string(str, 0, LINE1, &sysfont) ;
				}
				else
				{ // added check for EU region where max data rate is based upon channel
					rate = RegParams.maxDataRate; // force back to lowest data rate setting
				}
			}while(status != LORAWAN_SUCCESS);

			set_LED_data(LED_AMBER,&off);
			set_LED_data(LED_GREEN,&off);
			blinkcnt = (rate << 1) + 1;
			app_gflags.sleep_en = false;
			appTaskState = IDLE_STATE;
			appPostTask(PROCESS_TASK_HANDLER);
			SwTimerStart(lTimerId,MS_TO_US(250),SW_TIMEOUT_RELATIVE,(void *)lTimerCb,NULL); // set blink timer for led
			SwTimerStop(AppTimerId);	// Stop Periodic application timer
			break;

		case EXTLONGBUTTON_PRESS_STATE:
			status = LORAWAN_GetAttr(TX_POWER, 0, &pIndex);
			pIndex++;		// adjust Tx power index
			if(pIndex > RegParams.maxTxPwrIndx)
			{
				LORAWAN_GetAttr(ISMBAND,NULL,&band);
				if(band == ISM_NA915)
					pIndex = 5;	// force Tx pwr Index back to 5(max)
				else
					pIndex = 0; // force Tx pwr Index back to zero (max)
			}
			status = LORAWAN_SetAttr (TX_POWER, &pIndex);  // set TX Pwr Index  attribute

			if (LORAWAN_SUCCESS == status)
			{
				printf("Set Tx Power Index = %d Success\r\n",pIndex);
				// Clear screen
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				sprintf(str,"TXPWR Idx set to %d",pIndex);
				gfx_mono_draw_string(str, 0, LINE1, &sysfont) ;
			}

			set_LED_data(LED_AMBER,&off);
			set_LED_data(LED_GREEN,&off);
			blinkcnt = ((RegParams.maxTxPwrIndx - pIndex) << 1) +1;
			greenBlink = false;
			app_gflags.sleep_en = false;
			SwTimerStart(lTimerId,MS_TO_US(250),SW_TIMEOUT_RELATIVE,(void *)lTimerCb,NULL); // set blink timer for led
			SwTimerStop(AppTimerId);	// Stop Periodic application timer
			break;

		case LONGBUTTON_RESET_PRESS_STATE:
			rate = 0;
			status = LORAWAN_SetAttr (CURRENT_DATARATE, &rate);  // set data rate (DR)  attribute
			if (LORAWAN_SUCCESS == status)
			{
				printf("Set DateRate to default  DR%d Success\r\n",rate);
				// Clear screen
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				sprintf(str,"Restore DR to %d",rate);
				gfx_mono_draw_string(str, 0, LINE1, &sysfont) ;
			}

			LORAWAN_GetAttr(ISMBAND,NULL,&band);
			if(band == ISM_NA915)
				pIndex = 5;	// force Tx pwr Index back to 5(max)
			else
				pIndex = 0; // force Tx pwr Index back to zero (max)

			status = LORAWAN_SetAttr (TX_POWER, &pIndex);  // set TX Pwr Index  attribute
			if (LORAWAN_SUCCESS == status)
			{
				printf("Set default Tx Power Index  %d Success\r\n",pIndex);
				sprintf(str,"Restore TX Pwr to %d",pIndex);
				gfx_mono_draw_string(str, 0, LINE2, &sysfont) ;
			}
			blinkcnt = 2;
			greenBlink = false;
			app_gflags.sleep_en = false;
			SwTimerStart(lTimerId,MS_TO_US(250),SW_TIMEOUT_RELATIVE,(void *)lTimerCb,NULL); // set blink timer for led
			SwTimerStop(AppTimerId);	// Stop Periodic application timer
			break;


		default:
			break;
	}
	app_gflags.button_press = false;
	app_gflags.oled_button_chk = true;
	return SYSTEM_TASK_SUCCESS;
}

/*********************************************************************//**
\brief    Obtains serial  data from the UART when  called from the main While() loop
\ used to select initial operating parameters like ISM_BAND
*************************************************************************/
void serial_data_handler(void)
{
	int rxChar;
	char serialData;
	uint8_t keystatus;

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
				appTaskState = PROCESS_BAND_STATE;
			    appPostTask(PROCESS_TASK_HANDLER);
				printf("\r\n");
			}
		}
		if(app_gflags.oled_button_chk == true)
		{
			check_oled1_buttons();
			if(any_button == true)
			{
				if(keycode != 0 )
				{
					startReceiving = false;
					keycode = 0xFE;
					any_button = false;
					app_gflags.oled_button_chk = false;
				}
			}
			else
			{
				keystatus =  gfx_mono_menu_process_key(&Bsel_menu, keycode);
				if(keystatus != GFX_MONO_MENU_EVENT_IDLE )
				{
					oled_select = keystatus;
					appTaskState = PROCESS_BAND_STATE;
					appPostTask(PROCESS_TASK_HANDLER);
				}
				keycode = 0;
			}

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

		case SELECT_BAND_STATE:
			Select_Band();
			break;

		case PROCESS_BAND_STATE:
			processBandSelection();
			break;

		case JOIN_SEND_STATE:
			processJoinAndSend();
			break;
		case IDLE_STATE:
			break;
		default:
			//printf("Error STATE Entered\r\n");
			break;
	}

	return SYSTEM_TASK_SUCCESS;
}


/*********************************************************************//**
\brief    Restores the previous band and runs
*************************************************************************/
static void processRunRestoreBand(void)
{
	StackRetStatus_t status = LORAWAN_SUCCESS;
	uint8_t prevBand = 0xff;
	uint8_t choice = 0xff;
	char str[20];

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
	if(status == LORAWAN_SUCCESS && choice < sizeof(bandTable)-1)
	{
		uint32_t joinStatus = 0;
		PDS_RestoreAll();

		LORAWAN_GetAttr(LORAWAN_STATUS,NULL, &joinStatus);
		printf("\r\nPDS_RestorationStatus: Success\r\n" );
		// Clear screen
		gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
		gfx_mono_draw_string("PDS Restore Success", 0, LINE1, &sysfont) ;
		if(joinStatus & LORAWAN_NW_JOINED)
		{
			app_gflags.joined = true;
			printf("joinStatus: Joined\r\n");
			gfx_mono_draw_string("joinStatus : Joined", 0, LINE2, &sysfont) ;
		}
		else
		{
			app_gflags.joined = false;
			printf("JoinStatus : Denied\r\n");
			gfx_mono_draw_string("joinStatus : Denied", 0, LINE2, &sysfont) ;
			set_LED_data(LED_AMBER,&on);
		}

		sprintf(str,"Band: %s",bandStrings[choice]);
		printf("%s\r\n",str);
		gfx_mono_draw_string(str, 0, LINE3, &sysfont) ;

		print_application_config();
		delay_s(3);
		try_join = true;
		appTaskState = JOIN_SEND_STATE;
		appPostTask(PROCESS_TASK_HANDLER);
	}
	else
	{
		printf("Restoration failed\r\n");
		appTaskState = SELECT_BAND_STATE;
		appPostTask(PROCESS_TASK_HANDLER);
	}
}

/*********************************************************************//**
\brief    Sends Join request or Data to the network
*************************************************************************/
static void processJoinAndSend(void)
{
	StackRetStatus_t status = LORAWAN_SUCCESS;

	if(app_gflags.joined == false)
	{
		if(try_join)
		{
			status = LORAWAN_Join(DEMO_APP_ACTIVATION_TYPE);
			if (LORAWAN_SUCCESS == (StackRetStatus_t)status)
			{
				set_LED_data(LED_GREEN,&on);
				printf("\nJoin Request Sent\n\r");
				// Clear screen
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Join Request Sent", 0, LINE1, &sysfont) ;


			}
			else
			{
				set_LED_data(LED_AMBER,&on);
				print_stack_status(status);
				appTaskState = JOIN_SEND_STATE;
				appPostTask(PROCESS_TASK_HANDLER);
			}
		}
	}
	else if(app_gflags.joined == true)
	{
		sendData(DEMO_APP_TRANSMISSION_TYPE);
	}

}

/*********************************************************************//**
\brief    handles selection of ISM band / Region of operation
*************************************************************************/
static void processBandSelection(void)
{
	uint8_t num;
	if(oled_select != GFX_MONO_MENU_EVENT_IDLE )
	{
		num = oled_select + 1;  // offset by one for bandTable alignment
		switch (num) {
			case 1:		// EU868
			case 2:		// NA915
			case 3:		// AU915
			case 4:		// AS923
			case 5:		// JPN923
			case 6:		// KR920
			case 7:		// IND865
				LORAWAN_Reset(bandTable[num]);
				mote_set_parameters(bandTable[num],num);
				set_LED_data(LED_GREEN,&on);
				break;
			case 8:		// Clear PDS
				PDS_DeleteAll();
				appTaskState = SELECT_BAND_STATE;
				appPostTask(PROCESS_TASK_HANDLER);
				break;
			case 9:		// Reset Board
				NVIC_SystemReset();
				break;
			default:
				break;
		}
	}
	else
	{
		num = serialBuffer - '0';
		if(num == sizeof(bandTable)-1)			// Reset device
		{
			NVIC_SystemReset();
		}
		else if(num == sizeof(bandTable)-2)		// Clear PDS
		{
			PDS_DeleteAll();
			appTaskState = SELECT_BAND_STATE;
			appPostTask(PROCESS_TASK_HANDLER);
		}
		else if(num >0 && num < sizeof(bandTable) -2) // Set selected ISM band
		{
			LORAWAN_Reset(bandTable[num]);
			mote_set_parameters(bandTable[num],num);
			set_LED_data(LED_GREEN,&on);
		}
		else
		{
			printf("Not a valid regional band choice\r\n");
			appTaskState = SELECT_BAND_STATE;
			appPostTask(PROCESS_TASK_HANDLER);
		}
	}

}



/*********************************************************************//**
\brief    Displays and activates LED's for selecting band in Demo application
*************************************************************************/
static void Select_Band(void)
{
	uint8_t i = 0;

    set_LED_data(LED_AMBER,&off);
    set_LED_data(LED_GREEN,&off);

    printf("\r\nPlease select one of the bands listed below\r\n");
    for(i = 1;i < sizeof(bandTable); i++)
    {
	    printf("%d. %s\r\n",i,bandStrings[i]);
    }

    printf("Select Regional Band : ");
	startReceiving = true;
	app_gflags.oled_button_chk = true;
	gfx_mono_menu_init(&Bsel_menu);

}

/*********************************************************************//**
\brief    Initialization the Demo application
*************************************************************************/
void mote_demo_init(void)
{
	/* Init global flags */
	app_gflags.button_press = false;
	app_gflags.joined = false;
	app_gflags.factory_reset = false;
	app_gflags.bandSelected = false;
	app_gflags.sleep_en = true;
	app_gflags.oled_button_chk = false;

    bool status = false;
	char str[20];

    /* Initialize the resources */
    resource_init();
	/* Read DEV EUI from EDBG */
	dev_eui_read();
	startReceiving = false;
    /* Initialize the LORAWAN Stack */
    LORAWAN_Init(demo_appdata_callback, demo_joindata_callback);
    printf("\r\n***********************************************\r\n");
    printf("\r\nMicrochip LoRaWAN Stack %s\r\n",STACK_VER);
    printf("\r\nInit - Successful\r\n");

	// Init OLED display and oled1 buttons and leds
	gfx_mono_init() ;
	// Clear screen
	gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
	gfx_mono_draw_string("*********************",0, LINE1, &sysfont) ;
	gfx_mono_draw_string(" LoRaWAN SAMR34 Demo ",0, LINE2, &sysfont) ;
	gfx_mono_draw_string("  Init - Successful  ",0, LINE3, &sysfont) ;
	gfx_mono_draw_string("*********************",0, LINE4, &sysfont) ;
	// Oled1 Interface Init. (buttons, leds)
	oled1_init(&oled1) ;
	oled1_set_led_state(&oled1, OLED1_LED1_ID, true) ;
	oled1_set_led_state(&oled1, OLED1_LED3_ID, true) ;
	delay_ms(1000) ;
	oled1_set_led_state(&oled1, OLED1_LED2_ID, true) ;
	delay_ms(2000) ;
	oled1_set_led_state(&oled1, OLED1_LED1_ID, false) ;
	oled1_set_led_state(&oled1, OLED1_LED2_ID, false) ;
	oled1_set_led_state(&oled1, OLED1_LED3_ID, false) ;

	button_splash();
	delay_s(5);

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
		printf("\r\n");

		gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
		sprintf(str,"Regional Band %s",bandStrings[prevChoice]);
		gfx_mono_draw_string(str,0, LINE1, &sysfont) ;
		gfx_mono_draw_string("Press any button to",0, LINE2, &sysfont) ;
		gfx_mono_draw_string("    change Band",0, LINE3, &sysfont) ;
		startReceiving = true;
		app_gflags.oled_button_chk = true;
		any_button = true;
        SwTimerStart(demoITimerId,MS_TO_US(5000),SW_TIMEOUT_RELATIVE,(void *)demoITimerCb,NULL);
    }
    else
    {
		appTaskState = SELECT_BAND_STATE;
        appPostTask(PROCESS_TASK_HANDLER);
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
	char str[20];
	uint8_t sz;

    //Successful reception
    if((dataLength > 0U) && (NULL != pData))
    {
        printf("\n\n*** Received DL Data ***\r\n");
        printf("Frame Received at port %d\r\n",pData[0]);
        printf("Frame Length:  %d\r\n",dataLength);
        printf("Address: 0x%lx\r\n", devAddress);
        printf ("Payload: ");
        for (uint8_t i =0; i<dataLength - 1; i++)
        {
            printf("%2.2X",pData[i+1]);
        }
        printf("\r\n\n*************************\r\n");

		// Clear screen
		gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
		gfx_mono_draw_string("Received DL Data", 0, LINE1, &sysfont) ;
		sprintf(str,"Port %d Length %d",pData[0],dataLength);
		gfx_mono_draw_string(str, 0, LINE2, &sysfont) ;
		sprintf(str,"Address: 0x%lx",devAddress);
		gfx_mono_draw_string(str, 0, LINE3, &sysfont) ;
		gfx_mono_draw_string("Payload: ", 0, LINE4, &sysfont) ;
		sz = sizeof(pData);
		if(sz > 4)
			sz = 4;
		oled1_print_array(pData+1, sz+1, 54, LINE4) ;
		delay_s(4);

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
	int8_t ch_ID;
	int8_t pwr;
	int8_t dr;
	char str[20];

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
				LORAWAN_GetAttr(LAST_CH_ID,0,&ch_ID);
				printf("Active Channel %d \r\n",ch_ID);
				LORAWAN_GetAttr(TX_POWER,0,&pwr);
				printf("Active TxPower Index = %02d \r\n",pwr);
				LORAWAN_GetAttr(CURRENT_DATARATE,0,&dr);
				printf("Active Data rate = DR%d \r\n",dr);

				// Clear screen
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Transmission Success", 0, LINE1, &sysfont) ;
				sprintf(str,"Active Channel %d",ch_ID);
				gfx_mono_draw_string(str, 0, LINE2, &sysfont) ;
				sprintf(str,"TxPwr: %d  DR%d",pwr,dr);
				gfx_mono_draw_string(str, 0, LINE3, &sysfont) ;

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
				// clear oled
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Transmission Failed", 0, LINE1, &sysfont) ;
				gfx_mono_draw_string("NO CHANNELS FOUND", 0, LINE2, &sysfont) ;
            break;
            case LORAWAN_BUSY:
                printf("\n\rBUSY\n\r");
            break;
            case LORAWAN_NO_ACK:
                printf("\n\rNO_ACK \n\r");
				// clear oled
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Transmission Failed", 0, LINE1, &sysfont) ;
				gfx_mono_draw_string("NO ACK", 0, LINE2, &sysfont) ;
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

        printf("***************************************************\n\r");
    }

    SwTimerStop(lTimerId);     // stop Green led blink timer
    set_LED_data(LED_GREEN,&off);
    if(status != LORAWAN_SUCCESS)
    {
        set_LED_data(LED_AMBER,&on);
    }
	else
	{
		set_LED_data(LED_AMBER,&off);
	}
	app_gflags.sleep_en = true;
	appTaskState = IDLE_STATE;
    appPostTask(PROCESS_TASK_HANDLER);

}

/*********************************************************************//*
\brief Callback function for the ending of Activation procedure
 ************************************************************************/
void demo_joindata_callback(StackRetStatus_t status)
{
	int8_t ch_ID;
	char str[20];
    /* This is called every time the join process is finished */
    set_LED_data(LED_GREEN,&off);
    if(LORAWAN_SUCCESS == status)
    {
        uint32_t devAddress;
        bool mcastEnabled;

        app_gflags.joined = true;
        printf("\nJoining Successful\n\r");

		LORAWAN_GetAttr(LAST_CH_ID,0,&ch_ID);
		printf("Joined on Channel %d \r\n",ch_ID);

		// Clear screen
		gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
		gfx_mono_draw_string("Joining Successful", 0, LINE1, &sysfont) ;
		sprintf(str,"Joined on CH%d",ch_ID);
		gfx_mono_draw_string(str, 0, LINE2, &sysfont) ;

        LORAWAN_GetAttr(DEV_ADDR, NULL, &devAddress);
        LORAWAN_GetAttr(MCAST_ENABLE, NULL, &mcastEnabled);

		sprintf(str, "Dev Addr: 0x%lx", devAddress) ;
		gfx_mono_draw_string(str, 0, LINE3, &sysfont) ;

        if (devAddress != DEMO_APP_MCAST_GROUP_ADDRESS)
        {
            printf("\nDevAddr: 0x%lx\n\r", devAddress);
        }
        else if ((devAddress == DEMO_APP_MCAST_GROUP_ADDRESS) && (true == mcastEnabled))
        {
            printf("\nAddress conflict between Device Address and Multicast group address\n\r");
        }
#if (EU_BAND == 1)
// if EU 868mhz region then limit channel selections to mandatory 125khz channels 0,1 and 2
	uint8_t band;
	LORAWAN_GetAttr(ISMBAND,NULL,&band);

    if (band == ISM_EU868)
    {
		UpdateChId_t update_chid;
		#define MAX_EU_CHANNELS 16

		uint8_t channelIndex;
		for (channelIndex = 0; channelIndex < MAX_EU_CHANNELS; channelIndex++)
		{
			// set channel number using update_chid.channelIndex
			update_chid.channelIndex = channelIndex;
			if(channelIndex < 3)
			{
				// Enable or Disable the channel using update_chid.statusNew
				update_chid.statusNew = ENABLED;
			}
			else
			{
				update_chid.statusNew = DISABLED;
			}

			//Call LORAREG_SetAttr CHANNEL_ID_STATUS to set the new channel parameters
			LORAREG_SetAttr (CHANNEL_ID_STATUS,&update_chid);
		}
	}
#endif
		PDS_StoreAll();
        print_application_config();
        set_LED_data(LED_GREEN,&on);
		set_LED_data(LED_AMBER,&off);
		app_gflags.sleep_en = false; // do not allow sleep until 1st sensor transmission occurs
		// set delay before 1st sensor transmission after initial join
		SwTimerStart(AppTimerId,MS_TO_US(5000),SW_TIMEOUT_RELATIVE,(void *)appTimerCb,NULL);
    }
    else
    {
       app_gflags.joined = false;
        set_LED_data(LED_AMBER,&on);
        printf("\nJoining Denied\n\r");
		LORAWAN_GetAttr(LAST_CH_ID,0,&ch_ID);
		printf("Join Attempt was on CH  %d \r\n",ch_ID);
		try_join = false;
		printf("Awaiting next Join attempt\r\n");

		// Clear screen
		gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
		gfx_mono_draw_string("Joining Denied", 0, LINE1, &sysfont) ;
		sprintf(str, "CH %d Join Failed", ch_ID);
		gfx_mono_draw_string(str,0,LINE2, &sysfont);

		SwTimerStart(AppTimerId,MS_TO_US(10000),SW_TIMEOUT_RELATIVE,(void *)appTimerCb,NULL); // set rejoin timer to 10000ms
		printf("\n\r***********************************************\n\r");
    }
}

// LED blink timer call back handler
void lTimerCb(void *data)
{
	if(blinkcnt == 0xFF)
	{
		SwTimerStart(lTimerId,MS_TO_US(100),SW_TIMEOUT_RELATIVE,(void *)lTimerCb,NULL);
		set_LED_data(LED_GREEN,&toggle);
	}
	else
	{
		if(blinkcnt != 0)
		{
			SwTimerStart(lTimerId,MS_TO_US(250),SW_TIMEOUT_RELATIVE,(void *)lTimerCb,NULL);
			if(greenBlink)
				set_LED_data(LED_GREEN,&toggle);
			else
				set_LED_data(LED_AMBER,&toggle);
			blinkcnt--;
		}
		else
		{
			 set_LED_data(LED_GREEN,&off);
			 set_LED_data(LED_AMBER,&off);
			 blinkcnt = 0xFF;		// restore to normal toggle ( used for TX transmission blink )
			 greenBlink = true;		// restore indicator to default green blink
			 app_gflags.sleep_en = true;

		}
	}
}

// Application Periodic timer callback handler
void appTimerCb(void *data)
{
	if(app_gflags.joined)
	{
		appTaskState = JOIN_SEND_STATE;
		appPostTask(PROCESS_TASK_HANDLER);
		app_gflags.sleep_en = false;
	}
	else
	{
		try_join = true;
		appTaskState = JOIN_SEND_STATE;
		appPostTask(PROCESS_TASK_HANDLER);
	}

}

/*********************************************************************//*
 \brief      Function to send data from end device to application server
  ************************************************************************/
void sendData(TransmissionType_t type)
{

    int status = -1;
	uint8_t avail_payload;
    /* Read temperature sensor value */
	int i;
	int celavg;
	celavg = 0;
	char str[24];
	#define avgcnt 20		// sets number of a/d samples to average to get stable value
	for(i=0;i<avgcnt;i++)
	{
		get_resource_data(TEMP_SENSOR,(uint8_t *)&cel_val);
		celavg = celavg + cel_val;
	}
	cel_val = celavg/avgcnt;

    fahren_val = convert_celsius_to_fahrenheit(cel_val);
	printf("***************************************************\r\n");
    printf("Temperature: ");
    snprintf(temp_sen_str,sizeof(temp_sen_str),"%.1fC/%.1fF\n", cel_val, fahren_val);
    printf("%.1f C  / %.1f F\n\r", cel_val, fahren_val);

    data_len = strlen(temp_sen_str);
    lorawanSendReq.buffer = &temp_sen_str;
    lorawanSendReq.bufferLength = data_len - 1;
    lorawanSendReq.confirmed = type;
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

		// Clear screen
		gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
		gfx_mono_draw_string("Tx Data Sent", 0, LINE1, &sysfont) ;
		gfx_mono_draw_string("Temperature:", 0, LINE2, &sysfont) ;
		sprintf(str,"%.1f C  / %.1f F", cel_val, fahren_val);
		gfx_mono_draw_string(str,0,LINE3, &sysfont);

        SwTimerStart(lTimerId,MS_TO_US(100),SW_TIMEOUT_RELATIVE,(void *)lTimerCb,NULL); // set blink timer for led

    }
    else
    {
        print_stack_status(status);
    }
}


/*********************************************************************//*
 \brief      Initial user input wait Timer callback handler.
             Used during the initial 5 sec waiting period allowing user to change ISM band selection
 \param[in]  cnt - not used
 ************************************************************************/
void demoITimerCb(void * cnt)
{
    uint8_t i = 10;
    int8_t rxdata = 0;
	char str[20];
	// count defined as global
    printf("%d..",count);
	sprintf(str,"%d..",count);
	gfx_mono_draw_string(str,0,LINE4,&sysfont);
    count--;

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
    /* if No input so far. then start timer till expiration */
    if(count > 0 && (!rxdata) && (keycode !=0xFE))
    {
        SwTimerStart(demoITimerId,MS_TO_US(1000),SW_TIMEOUT_RELATIVE,(void *)demoITimerCb,NULL);
    }
    /* user did not press any input  during the time alloted*/
    else if(count == 0 && (!rxdata))
    {
		appTaskState = RESTORE_BAND_STATE;
        appPostTask(PROCESS_TASK_HANDLER);
    }
    /* User pressed a key */
    else if(rxdata || (keycode == 0xFE))
    {
        printf("\r\n");
		appTaskState = SELECT_BAND_STATE;
        appPostTask(PROCESS_TASK_HANDLER);
		app_gflags.oled_button_chk = true;
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

    printf("\n****************Join Parameters****************\n\r");

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

	// Clear screen
	gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
	gfx_mono_draw_string("Join Request Sent", 0, LINE1, &sysfont) ;
	if (DEMO_APP_ACTIVATION_TYPE == OVER_THE_AIR_ACTIVATION)
	{
		gfx_mono_draw_string("OTAA Activation EUI:", 0, LINE2, &sysfont) ;
		oled1_print_array(demoDevEui, sizeof(demoDevEui), 0, LINE3) ;
		oled1_print_array(demoJoinEui, sizeof(demoJoinEui), 0, LINE4) ;
	}
	else
	{
		gfx_mono_draw_string("ABP devAddr: ", 0, LINE2, &sysfont) ;
		d_Addr[0] = (demoDevAddr & 0xFF000000) >> 24;
		d_Addr[1] = (demoDevAddr & 0x00FF0000) >> 16;
		d_Addr[2] = (demoDevAddr & 0x0000FF00) >> 8;
		d_Addr[3] = (demoDevAddr & 0x000000FF) >> 0; 
		//oled1_print_array((uint8_t*) demoDevAddr, sizeof(demoDevAddr), 0, LINE3) ;
		oled1_print_array((uint8_t*) d_Addr, sizeof(d_Addr), 0, LINE3) ;
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
	uint8_t pwr;
	uint8_t dr;
	uint8_t band;
	uint8_t retry;
	bool adr;

    LORAWAN_Reset(ismBand);
 
#if (NA_BAND == 1 || AU_BAND == 1)
#if (RANDOM_NW_ACQ == 0)
    if ((ismBand == ISM_NA915) || (ismBand == ISM_AU915))
    {
        #define MAX_NA_CHANNELS 72
        #define MAX_SUBBAND_CHANNELS 8

        ChannelParameters_t ch_params;

        uint8_t allowed_min_125khz_ch,allowed_max_125khz_ch,allowed_500khz_channel;

        allowed_min_125khz_ch = (SUBBAND-1)*MAX_SUBBAND_CHANNELS;

        allowed_max_125khz_ch = ((SUBBAND-1)*MAX_SUBBAND_CHANNELS) + 7 ;

        allowed_500khz_channel = SUBBAND+63;

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
    status = set_join_parameters(DEMO_APP_ACTIVATION_TYPE);

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

	LORAWAN_GetAttr(TX_POWER,0,&pwr);
	printf("\nTxPower Index     = %02d \r\n",pwr);
	LORAWAN_GetAttr(CURRENT_DATARATE,0,&dr);
	printf("Current Data rate = DR%d \r\n",dr);
	adr = ADRenable;
	LORAWAN_SetAttr(ADR, &adr);
	LORAWAN_GetAttr(ADR,0,&adr);
	printf("ADR               = ");
	printf(adr == true ? "ON" : "OFF");

	printf("\r\n");
	retry = 4;
	LORAWAN_SetAttr(CNF_RETRANSMISSION_NUM, &retry);
	LORAWAN_GetAttr(CNF_RETRANSMISSION_NUM,0,&retry);
	LORAWAN_GetAttr(ISMBAND,NULL,&band);
	if((band == ISM_NA915) || (band == ISM_AU915))
#if (RANDOM_NW_ACQ == 0)
		printf("SUBBAND           = %02d \r\n",SUBBAND);
#else
		printf("RAPID NWK ACQ ENABLED\r\n");
#endif

    try_join = true;
    /* Send Join request for Demo application */
    status = LORAWAN_Join(DEMO_APP_ACTIVATION_TYPE);

    if (LORAWAN_SUCCESS == status && index < sizeof(bandTable))
    {
        printf("\nJoin Request Sent for %s\n\r",bandStrings[index]);
    }
    else
    {
        print_stack_status(status);
		appTaskState = JOIN_SEND_STATE;
		appPostTask(PROCESS_TASK_HANDLER);
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
	uint8_t pwr;
	uint8_t band;
	uint8_t retry;

    printf("\n*************Application Configuration*************\n\r");
    LORAWAN_GetAttr(EDCLASS, NULL, &edClass);
    printf("\nDevType           : ");

    if(edClass == CLASS_A)
    {
        printf("CLASS A\n\r");
    }
    else if(edClass == CLASS_C)
    {
        printf("CLASS C\n\r");
    }

    printf("\nActivationType    : ");

    if(DEMO_APP_ACTIVATION_TYPE == OVER_THE_AIR_ACTIVATION)
    {
        printf("OTAA\n\r");
    }
    else if(DEMO_APP_ACTIVATION_TYPE == ACTIVATION_BY_PERSONALIZATION)
    {
        printf("ABP\n\r");
    }

    printf("\nTransmission Type : ");

    if(DEMO_APP_TRANSMISSION_TYPE == CONFIRMED)
    {
        printf("CONFIRMED\n\r");
    }
    else if(DEMO_APP_TRANSMISSION_TYPE == UNCONFIRMED)
    {
        printf("UNCONFIRMED\n\r");
    }

    printf("\nFPort             : %d\r\n", DEMO_APP_FPORT);

	LORAWAN_GetAttr(TX_POWER,0,&pwr);
	printf("\nTxPower Index     : %02d \r\n",pwr);

	LORAWAN_GetAttr(ISMBAND,NULL,&band);
	if((band == ISM_NA915) || (band == ISM_AU915))
#if (RANDOM_NW_ACQ == 0)
		printf("SUBBAND           = %02d \r\n",SUBBAND);
#else
		printf("RAPID NWK ACQ ENABLED\r\n");
#endif
	LORAWAN_GetAttr(CNF_RETRANSMISSION_NUM,0,&retry);
	printf("\nConfirmed Retries : %02d \r\n",retry);

	printf("\n***************************************************\r\n");
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

/*************************************************************************************************//*
 \brief      Reads the DEV EUI if it is flashed in EDBG MCU(SAMR34 Xplained Pro)/ Module(WLR089) 
 **************************************************************************************************/
static void dev_eui_read(void)
{
	#if (EDBG_EUI_READ == 1)
		uint8_t invalidEDBGDevEui[8];
		uint8_t EDBGDevEUI[8];
		edbg_eui_read_eui64((uint8_t *)&EDBGDevEUI);
		memset(&invalidEDBGDevEui, 0xFF, sizeof(invalidEDBGDevEui));
		/* If EDBG doesnot have DEV EUI, the read value will be of all 0xFF,
		   Set devEUI in conf_app.h in that case */
		if(0 != memcmp(&EDBGDevEUI, &invalidEDBGDevEui, sizeof(demoDevEui)))
		{
			/* Set EUI addr in EDBG if there */
			memcpy(demoDevEui, EDBGDevEUI, sizeof(demoDevEui));
		}
	#elif (MODULE_EUI_READ == 1)
		uint8_t i = 0, j = 0;
		uint8_t invalidMODULEDevEui[8];
		uint8_t moduleDevEUI[8];
		for (i = 0; i < 8; i += 2, j++)
		{
			moduleDevEUI[i] = (NVM_UID_ADDRESS[j] & 0xFF);
			moduleDevEUI[i + 1] = (NVM_UID_ADDRESS[j] >> 8);
		}
		memset(&invalidMODULEDevEui, 0xFF, sizeof(invalidMODULEDevEui));
		/* If Module doesnot have DEV EUI, the read value will be of all 0xFF,
		Set devEUI in conf_app.h in that case */
		if(0 != memcmp(&moduleDevEUI, &invalidMODULEDevEui, sizeof(demoDevEui)))
		{
			/* Set EUI addr in Module if there */
			memcpy(demoDevEui, moduleDevEUI, sizeof(demoDevEui));
		}
	#endif
}

void check_oled1_buttons(void)
{
	// Check buttons to see if user changed the selection
	if (oled1_get_button_state(&oled1, OLED1_BUTTON1_ID))
	{
		delay_ms(40);  // debounce delay
		oled1_set_led_state(&oled1, OLED1_LED1_ID, true) ;
		while(oled1_get_button_state(&oled1, OLED1_BUTTON1_ID)) ;
		oled1_set_led_state(&oled1, OLED1_LED1_ID, false) ;
		keycode = GFX_MONO_MENU_KEYCODE_DOWN;
	}
	else if (oled1_get_button_state(&oled1, OLED1_BUTTON2_ID))
	{
		delay_ms(40);  // debounce delay
		oled1_set_led_state(&oled1, OLED1_LED2_ID, true) ;
		while(oled1_get_button_state(&oled1, OLED1_BUTTON2_ID)) ;
		oled1_set_led_state(&oled1, OLED1_LED2_ID, false) ;
		keycode = GFX_MONO_MENU_KEYCODE_SELECT;
	}
	else if (oled1_get_button_state(&oled1, OLED1_BUTTON3_ID))
	{
		delay_ms(40);
		oled1_set_led_state(&oled1, OLED1_LED3_ID, true) ;
		while(oled1_get_button_state(&oled1, OLED1_BUTTON3_ID)) ;
		oled1_set_led_state(&oled1, OLED1_LED3_ID, false) ;
		keycode = GFX_MONO_MENU_KEYCODE_UP;
	}
}



/* eof  */
