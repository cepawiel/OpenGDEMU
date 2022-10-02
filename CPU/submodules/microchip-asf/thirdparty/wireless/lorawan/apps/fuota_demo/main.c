/**
* \file  main.c
*
* \brief FUOTA Demo Application
*
*
* Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
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
* <p>This is minimal application to join, communicate with LoRaWAN network.</p>
* <p>This application also demonstrates firmware upgrade over LoRaWAN</p>
*/
/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include <asf.h>
#include "radio_driver_hal.h"
#include "sw_timer.h"
#include "aes_engine.h"
#include "pds_interface.h"
#include "sio2host.h"
#include "delay.h"
#include "sleep_timer.h"
#include "pmm.h"
#include "conf_pmm.h"
#include "lorawan.h"
#include "edbg_eui.h"
#include "DMPackageCore.h"
#include "MCMPackageCore.h"
#include "FTMPackageCore.h"
#include "msg.h"
#include "conf_app.h"
#include "log.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
#define FACTORY_NEW_DEVICE     (false)
#define NON_FACTORY_NEW_DEVICE (true)

/* GREEN LED is device activity indicator; ON - ACTIVE, OFF - SLEEP */
#define GREEN_LED_ON()         port_pin_set_output_level(LED_1_PIN, false)
#define GREEN_LED_OFF()        port_pin_set_output_level(LED_1_PIN, true)

/* AMBER LED is new firmware indicator; ON - new firmware available */
#define AMBER_LED_ON()         port_pin_set_output_level(LED_0_PIN, false)
#define AMBER_LED_OFF()        port_pin_set_output_level(LED_0_PIN, true)

#define APP_FW_VERSION         (0x00000002)
#define APP_HW_VERSION         (0x00000001)

/************************************************************************/
/* TYPES                                                                */
/************************************************************************/
typedef union _AppFlags
{
    uint16_t value;
    struct {
        uint16_t joinInProgress : 1;
        uint16_t deviceJoined   : 1;
        uint16_t uplinkEnabled  : 1;
        uint16_t sendUplink     : 1;
    };
} AppFlags_t;

typedef struct _AppActivation
{
    uint32_t devAddr;
    uint8_t  devEui[sizeof(uint64_t)];
    uint8_t  appEui[sizeof(uint64_t)];
    uint8_t  appKey[LORAWAN_SESSIONKEY_LENGTH];
} AppActivation_t;

typedef struct _Application
{
    uint32_t        version;
    uint8_t         tmrUplink;
    AppActivation_t activation;
    AppFlags_t      flags;
} Application_t;

/************************************************************************/
/* EXTERNS                                                              */
/************************************************************************/


/************************************************************************/
/* STATIC FUNCTION PROTOTYPES                                           */
/************************************************************************/
static void appInit( void );
static void appConfig( void );

static PMM_Status_t appSleep( void );

static void appResourcesInit( void );
static void appResourcesUninit( void );

static void configXproButtonSW0( void );
static void xproButtonSW0Pressed( void );

static void appJoin( void );
static void appUplink( void );
static void appTransCmplCallback( appCbParams_t *params );
static void appRxDataAvailCallback( appCbParams_t *params );

static void appPrintIsmBand( IsmBand_t band );

/************************************************************************/
/* GLOBAL FUNCTION PROTOTYPES                                           */
/************************************************************************/
/* PMM wakeup callback */
void APP_WakeupCallback( uint32_t sleptDuration );

/* Timer callbacks */
void APP_UplinkTimerCallback( void );

/* LoRaWAN callbacks */
void APP_JoinCallback( StackRetStatus_t status );

/* FTMPackage callback */
void APP_FuotaImageReceivedCallback(void);

/* Task handler */
SYSTEM_TaskStatus_t APP_TaskHandler( void );

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
Application_t fuotaDemo;
uint32_t appData = 1;

/************************************************************************/
/* APPLICATION FUNCTIONS                                                */
/************************************************************************/

/*
* \brief main function
*/
int main( void )
{
    appInit();
    appConfig();      
    
    for ( ; ; )
    {
        GREEN_LED_ON();
        SYSTEM_RunTasks();
        GREEN_LED_OFF();

        appSleep();   
    }

    return 0;
}

/*
* \brief main function
*/
SYSTEM_TaskStatus_t APP_TaskHandler( void )
{
    SYSTEM_TaskStatus_t taskStatus = SYSTEM_TASK_SUCCESS;
    EdClass_t class;      
    LORAWAN_GetAttr(EDCLASS, NULL, &class);
    if ( !fuotaDemo.flags.deviceJoined )
    {
        appJoin();
        return taskStatus;
    }

    MSG_Process();

    if (CLASS_A == class)
    {
        if ( fuotaDemo.flags.uplinkEnabled )
        {
            if ( false == SwTimerIsRunning(fuotaDemo.tmrUplink) )
            {
                SwTimerStart( fuotaDemo.tmrUplink, MS_TO_US(APP_UPLINKINTERVAL_MS), SW_TIMEOUT_RELATIVE, APP_UplinkTimerCallback, NULL );
                /*LOGLINE_DEBUG("Uplink Timer started, timerCount=%ums", APP_UPLINKINTERVAL_MS);*/
            }            
        }
    }

    return taskStatus;
}

void APP_JoinCallback( StackRetStatus_t status )
{
    fuotaDemo.flags.joinInProgress = false;
    if ( LORAWAN_SUCCESS == status )
    {
        fuotaDemo.flags.deviceJoined = true;
        LORAWAN_GetAttr(DEV_ADDR, NULL, &fuotaDemo.activation.devAddr);
        LOGLINE_INFO("OTAA Join Success, DevAddr=0x%08X", (unsigned int)fuotaDemo.activation.devAddr);

        SwTimerStart( fuotaDemo.tmrUplink, MS_TO_US(APP_UPLINKINTERVAL_MS), SW_TIMEOUT_RELATIVE, APP_UplinkTimerCallback, NULL );

        if (APP_EDCLASS == CLASS_C)
        {
            EdClass_t class = CLASS_C;
            LORAWAN_SetAttr(EDCLASS, &class);
            LORAWAN_SetAttr(SEND_DEVICE_TIME_CMD, NULL);
            MSG_Send(LORAWAN_UNCNF, 1, NULL, 0, 0);
        }        
    }
    else
    {
        LOGLINE_ERROR("OTAA Join Failed, StatusCode=0x%04X", status);
    }
    SYSTEM_PostTask( APP_TASK_ID );
}

/************************************************************************/
/* APPLICATION TIMER CALLBACKS                                          */
/************************************************************************/
void APP_UplinkTimerCallback( void )
{    
    EdClass_t class;    
    
    /*LOGLINE_DEBUG("Uplink timer fired");*/
    
    LORAWAN_GetAttr(EDCLASS, NULL, &class);    

    if (CLASS_A == class)
    {
        appUplink();
    }
    else
    {
        LOGLINE_WARN("No timed uplinks in class other than class-A");
        LOGLINE_INFO("Device not in class-A, please use SW0 for uplinks");
    }

    SYSTEM_PostTask(APP_TASK_ID);
}

/************************************************************************/
/* APPLICATION SLEEP CALLBACK                                           */
/************************************************************************/
void APP_WakeupCallback( uint32_t sleptDuration )
{
    SYSTEM_PostTask(APP_TASK_ID);
    appResourcesInit();
}

/************************************************************************/
/* FTMPACKAGE CALLBACK                                                  */
/************************************************************************/
void APP_FuotaImageReceivedCallback(void)
{
    LOGLINE_INFO("Full image received, reset to upgrade to new image");
    AMBER_LED_ON();
}

/************************************************************************/
/* STATIC FUNCTIONS                                                     */
/************************************************************************/
static void appInit( void )
{
    /* Init base system */
    system_init();
    board_init();
    delay_init();

    /* Before peripherals initialize, enable interrupts */
    INTERRUPT_GlobalInterruptEnable();    

    /* Init UART */
    sio2host_init();
    delay_ms(1000);

    /* Init timers */
    SleepTimerInit();
    SystemTimerInit();

    /* Init persistent storage */
    PDS_Init();

    /* Init modules required for MAC and TAL */
    AESInit();
    HAL_RadioInit();

    /* External interrupts */
    configXproButtonSW0();
    if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
    {
        // If SW0 is pressed on XPRO during PoR, then device will factory-reset itself
        PDS_DeleteAll();
    }

    /* Init MAC - implicity TAL gets initialized */
    LORAWAN_Init(MSG_DataCallback, APP_JoinCallback);
    LORAWAN_Reset(APP_ISMBAND);

    /* Init app-specific stuff */
    fuotaDemo.version = APP_FW_VERSION;
    fuotaDemo.flags.uplinkEnabled = true;
    SwTimerCreate( &(fuotaDemo.tmrUplink) );
    
    /* Setup FUOTA packages */
    DMPackage_SetVersionInfo(APP_FW_VERSION, APP_HW_VERSION);
    MCMPackageCreateTimer();
    {
        uint8_t key[LORAWAN_SESSIONKEY_LENGTH] = APP_GENAPPKEY;
        MCMPackageSetGenAppKey(key);
    }
    FTMPackageSetFuotaDescriptor(APP_FTM_FUOTA_DESCRIPTOR);
    FTMPackageSetImageReceivedCallback(APP_FuotaImageReceivedCallback);    

    MSG_Init();
    MSG_SetDefaultRxDataAvailCallback( appRxDataAvailCallback );
    MSG_SetTransactionCompleteCallback( appTransCmplCallback );
    MSG_SetFPortRxDataAvailCallback( LORAWAN_DMPACKAGE_PORT, DMPackageCore );
    MSG_SetFPortRxDataAvailCallback( LORAWAN_MCMPACKAGE_PORT, MCMPackageCore );
    MSG_SetFPortRxDataAvailCallback( LORAWAN_FTMPACKAGE_PORT, FTMPackageCore );

    /* Kickstart the app */
    SYSTEM_PostTask( APP_TASK_ID );

    /* App info to console */
    LOGLINE_INFO("\r\n======================================================");
    LOGLINE_INFO("\r\nMicrochip LoRaWAN Stack - version %s", STACK_VER);    
    LOGLINE_INFO("\r\nfuota_demo app - version %08X", (unsigned int)fuotaDemo.version);
    LOGLINE_INFO("\r\n======================================================");
    {
        uint8_t tmp;
        LORAWAN_GetAttr( ISMBAND, NULL, &tmp );
        appPrintIsmBand( (IsmBand_t)tmp );
        LORAWAN_GetAttr( CURRENT_DATARATE, NULL , &tmp );
        LOGLINE_INFO("\r\nCurrent UPLINK Data Rate: %d", tmp);
    }

    AMBER_LED_OFF();
}

static void appConfig( void )
{
    /* Try restoring the previous configuration */
    bool deviceCondition = PDS_IsRestorable();
    uint8_t appEui[8] = APPLICATION_EUI;
    uint8_t devEui[8] = DEVICE_EUI;
    uint8_t appKey[16] = APPLICATION_KEY;

#if (EDBG_EUI_READ == 1)
    /* Reading DEV EUI from SAMR34 XPRO EDBG */
    edbg_eui_read_eui64((uint8_t *) &devEui);
#endif
    if(FACTORY_NEW_DEVICE == deviceCondition)
    {
        memcpy( &fuotaDemo.activation.devEui[0], devEui, sizeof(fuotaDemo.activation.devEui) );
        memcpy( &fuotaDemo.activation.appEui[0], appEui, sizeof(fuotaDemo.activation.appEui) );
        memcpy( &fuotaDemo.activation.appKey[0], appKey, sizeof(fuotaDemo.activation.appKey) );
        
        LORAWAN_SetAttr(DEV_EUI, &fuotaDemo.activation.devEui[0]);
        LORAWAN_SetAttr(JOIN_EUI, &fuotaDemo.activation.appEui[0]);
        LORAWAN_SetAttr(APP_KEY, &fuotaDemo.activation.appKey[0]);

        PDS_StoreAll();

        fuotaDemo.flags.deviceJoined = false;
        fuotaDemo.flags.joinInProgress = false;
    }
    else // NON_FACTORY_NEW_DEVICE
    {
        uint32_t tmp;
        EdClass_t class = CLASS_A;

        // Restore previous configuration from PDS
        PDS_RestoreAll();

        LORAWAN_GetAttr(LORAWAN_STATUS, NULL, &tmp);
        fuotaDemo.flags.deviceJoined = (bool) (tmp & 0x1);

        LORAWAN_GetAttr(DEV_ADDR, NULL, &tmp);
        fuotaDemo.activation.devAddr = tmp;

        LORAWAN_SetAttr(EDCLASS, &class);

        fuotaDemo.flags.joinInProgress = false;
        LORAWAN_SetAttr(SEND_DEVICE_TIME_CMD, NULL);
    }
    
    LOGLINE_INFO("Device is %s", (FACTORY_NEW_DEVICE == deviceCondition) ? "FACTORY-NEW" : "NON-FACTORY-NEW");
    LOGLINE_INFO("\r\n!!! Hold SW0 button during power-up to factory-reset !!!\r\n");

    LOGLINE_INFO("***** CURRENT DEVICE SETTINGS *****");
    {
        uint8_t eui[ 8 ];
        LOGLINE_INFO("Activation Method = OTAA");
        LORAWAN_GetAttr(DEV_EUI, NULL, &eui);
        LOGLINE_INFO("DevEUI = ");
        for (uint8_t i = 0; i < 8; i++)
        {
            LOGMSG_INFO("%02X", eui[i]);
        }
        LORAWAN_GetAttr(JOIN_EUI, NULL, &eui);
        LOGLINE_INFO("AppEUI = ");
        for (uint8_t i = 0; i < 8; i++)
        {
            LOGMSG_INFO("%02X", eui[i]);
        }
    }
    
    LOGLINE_INFO("\r\nEnd-device configured and ready\r\n");
}

static PMM_Status_t appSleep( void )
{
	PMM_SleepReq_t sleepReq = {
		.sleepTimeMs		= APP_SLEEPTIME_MS,
		.pmmWakeupCallback	= APP_WakeupCallback,
		.sleep_mode			= (HAL_SleepMode_t) CONF_PMM_SLEEPMODE_WHEN_IDLE
	};

    if ( LORAWAN_ReadyToSleep(false) )
    {
        appResourcesUninit();
   
        if (PMM_SLEEP_REQ_DENIED == PMM_Sleep(&sleepReq))
        {
            appResourcesInit();
            SYSTEM_PostTask(APP_TASK_ID);
            return PMM_SLEEP_REQ_DENIED;
        }
    }   

    return PMM_SLEEP_REQ_PROCESSED;
}

static void appResourcesInit( void )
{
    sio2host_init();
    HAL_RadioInit();
}

static void appResourcesUninit( void )
{
    struct port_config pin_conf;
    port_get_config_defaults(&pin_conf);
    pin_conf.powersave  = true;

#ifdef HOST_SERCOM_PAD0_PIN
    port_pin_set_config(HOST_SERCOM_PAD0_PIN, &pin_conf);
#endif

#ifdef HOST_SERCOM_PAD1_PIN
    port_pin_set_config(HOST_SERCOM_PAD1_PIN, &pin_conf);
#endif

    sio2host_deinit();
    HAL_RadioDeInit();
}

static void configXproButtonSW0(void)
{
    struct extint_chan_conf eint_chan_conf;
    extint_chan_get_config_defaults(&eint_chan_conf);

    eint_chan_conf.gpio_pin           = BUTTON_0_EIC_PIN;
    eint_chan_conf.gpio_pin_mux       = BUTTON_0_EIC_MUX;
    eint_chan_conf.detection_criteria = EXTINT_DETECT_FALLING;
    eint_chan_conf.filter_input_signal = true;
    extint_chan_set_config(BUTTON_0_EIC_LINE, &eint_chan_conf);
    extint_register_callback(
        xproButtonSW0Pressed,
        BUTTON_0_EIC_LINE,
        EXTINT_CALLBACK_TYPE_DETECT);
        extint_chan_enable_callback(BUTTON_0_EIC_LINE,
        EXTINT_CALLBACK_TYPE_DETECT
    );
}

static void xproButtonSW0Pressed(void)
{
    EdClass_t class;
    
    /* Read the button level */
    if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
    {
        /* Wait for button debounce time */
        delay_ms(50);
        /* Check whether button is in default state */
        if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
        {            
            /* Try waking up the device if it may be sleeping */
            PMM_Wakeup();
            
            LOGLINE_INFO("SW0 button press detected");

            LORAWAN_GetAttr(EDCLASS, NULL, &class);

            if (CLASS_A == class)
            {
                fuotaDemo.flags.uplinkEnabled = !fuotaDemo.flags.uplinkEnabled;
                if ( !fuotaDemo.flags.uplinkEnabled )
                {
                    SwTimerStop( fuotaDemo.tmrUplink );
                }
                LOGLINE_INFO("UPLINK timer is currently %s", (fuotaDemo.flags.uplinkEnabled ? "enabled" : "disabled"));
            }
            else if (CLASS_C == class)
            {
                appUplink();
            }

            /* Post task to application handler on button press */
            SYSTEM_PostTask(APP_TASK_ID);
        }
    }
}

static void appJoin( void )
{
    if ( !fuotaDemo.flags.joinInProgress )
    {
	    StackRetStatus_t status = LORAWAN_Join(LORAWAN_OTAA);
	    if (LORAWAN_SUCCESS == status)
	    {
            uint8_t channel;
            uint32_t chfreq;
            LORAWAN_GetAttr(LAST_CH_ID, NULL, &channel);
            LORAWAN_GetAttr(CH_PARAM_FREQUENCY, &channel, &chfreq);
    	    fuotaDemo.flags.joinInProgress = true;
    	    LOGLINE_INFO("OTAA JoinReq sent, ChannelID=%d, Frequency=%luHz", channel, chfreq);
	    }
    }
}

static void appUplink( void )
{
    AppResult_t result = APP_RESULT_OK; 
    uint32_t txTime;
    uint8_t channel;
    uint32_t chfreq;
    uint32_t mcmTmo = MCMPackageNextTimeoutDuration();
    bool send = MSG_IsTxPossible(sizeof(appData), &txTime);
    if ( 0 != mcmTmo )
    {
        mcmTmo = US_TO_MS( mcmTmo ) + 1;
        send = send && (txTime < mcmTmo);
    }

    if (send)
    {
        LORAWAN_GetAttr(LAST_CH_ID, NULL, &channel);
        LORAWAN_GetAttr(CH_PARAM_FREQUENCY, &channel, &chfreq);
        LOGLINE_INFO("UPLINK Channel=%d, Frequency=%luHz, Status=", channel, chfreq);
        result = MSG_Send(APP_TRANSMISSION_TYPE, APP_FPORT, (uint8_t*)&appData, sizeof(appData), 0);
        LOGMSG_INFO("%s", send ? ((result == APP_RESULT_OK) ? "OK" : "PREV_TRANSACTION_PENDING") : "TX_NOT_POSSIBLE");
    }
 
    (void)result;
}

static void appTransCmplCallback( appCbParams_t *params )
{
    LOGLINE_INFO("DONE");
    if (LORAWAN_SUCCESS == params->param.transCmpl.status)
    {
        LOGMSG_INFO(", Status = SUCCESS");
        LORAWAN_SetAttr(SEND_DEVICE_TIME_CMD, NULL);
        /*LOGLINE_DEBUG("!!! Added DeviceTimeReq in next uplink !!!");*/
    }
    else
    {
        LOGMSG_DEBUG(", Status = 0x%04X", params->param.transCmpl.status);
    }
}

static void appRxDataAvailCallback( appCbParams_t *params )
{
    LOGLINE_INFO("Data received");
    (void)params;
}

static void appPrintIsmBand( IsmBand_t band )
{
    LOGLINE_INFO("Current Region = ");
    if ( ISM_EU868 == band )
    {
        LOGMSG_INFO("EU868");
    }
    else if ( ISM_NA915 == band )
    {
        LOGMSG_INFO("NA915");
    }
    else if ( 0 != (ISM_ASBAND & (1 << band)) )
    {
        LOGMSG_INFO("AS923");
    }
    else if (ISM_IND865 == band )
    {
        LOGMSG_INFO("IND865");
    }
    else
    {
        LOGMSG_INFO("%d", (uint8_t)band);
    }
}

/* EOF main.c */
