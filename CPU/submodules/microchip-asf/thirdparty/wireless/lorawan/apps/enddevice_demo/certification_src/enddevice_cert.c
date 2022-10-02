/**
* \file  enddevice_cert.c
*
* \brief LORAWAN Certification Application
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
 

#if (CERT_APP == 1)

/****************************** INCLUDES **************************************/
#include "asf.h"
#include "lorawan.h"
#include "system_task_manager.h"
#include "enddevice_cert.h"
#include "enddevice_cpc.h"
#include "conf_certification.h"
#include "sio2host.h"
#include "sw_timer.h"
#if (ENABLE_PDS == 1)
#include "pds_interface.h"
#endif

/************************** GLOBAL VARIABLES ***********************************/
uint8_t testMode = OFF;
uint8_t sendData[100];
uint16_t sendDataLen;
uint16_t downlinkCtr = 0;
bool bTxCnf = false;
uint8_t uplinkTestNoResp = 0;
bool pktRxd = false;
uint8_t certAppTimerId;
uint32_t certAppTimeout = CERT_APP_TIMEOUT;
TransmissionType_t txType = CERT_APP_TRANSMISSION_TYPE;
LorawanSendReq_t lorawanSendReq;
IsmBand_t current_band;
bool clear_linkcheck = false;
uint8_t tx_cw_timer;
//bool reset_test_mode = false;
bool cnfTxInProgress = false;
uint8_t sendFport = CERT_APP_FPORT;

/************************** STATIC VARIABLES ***********************************/
/*ABP Join Parameters */
static uint32_t devAddr = CERT_DEVICE_ADDRESS;
static uint8_t nwksKey[16] = CERT_NETWORK_SESSION_KEY;
static uint8_t appsKey[16] = CERT_APPLICATION_SESSION_KEY;
/* OTAA join parameters */
uint8_t devEui[8] = CERT_DEVICE_EUI;
uint8_t joinEui[8] = CERT_APPLICATION_EUI;
uint8_t appKey[16] = CERT_APPLICATION_KEY;
static const char* bandStrings[] =
{
	"0xFF",
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
};


static uint8_t bandTable[] =
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
};

const char * status_message[] = {
    "radio_success",
    "radio_no_data",
    "radio_data_size",
    "radio_invalid_req",
    "radio_busy",
    "radio_out_of_range",
    "radio_unsupported_attr",
    "radio_channel_busy",
    "success",
    "nwk_not_joined",
    "invalid_parameter",
    "keys_not_initialized",
    "silent_immediately_active",
    "fcntr_error_rejoin_needed",
    "invalid_buffer_length",
    "mac_paused",
    "no_channels_found",
    "busy",
    "no_ack",
    "nwk_join_in_progress",
    "resource_unavailable",
    "invalid_request",
    "unsupported_band",
    "fcntr_error",
    "mic_error",
    "invalid_mtype,",
    "mcast_hdr_invalid",
    "tx_timeout",
    "radio_tx_timeout",
    "max_mcast_group_reached",
    "invalid_packet",
    "rxpkt_encryption_failed",
    "txpkt_encryption_failed",
    "skey_derivation_failed",
    "mic_calculation_failed",
    "skey_read_failed",
    "join_nonce_error"
};

/************************** EXTERN VARIABLES ***********************************/


/************************** FUNCTION PROTOTYPES ********************************/

/*********************************************************************//*
 \brief      Select Band to reset LORAWAN Stack
 \return     False, if a valid band is selected
             True, otherwise
 ************************************************************************/
static bool cert_select_band(void);
/*********************************************************************//*
 \brief      Enable channels if NA or AU band is selected
 \return     None
 ************************************************************************/
static void cert_enable_channels(void);
/*********************************************************************//*
 \brief      Function that processes the Rx data
 \param[in]  data - Rx data payload
 \param[in]  dataLen - The number of Rx bytes
 ************************************************************************/
static void cert_handle_cert_rx_data(uint8_t* data,uint8_t dataLen);

/*********************************************************************//**
\brief Callback function for the ending of Bidirectional communication of
       Application data
 *************************************************************************/
static void cert_appdata_callback(void *appHandle, appCbParams_t *appdata);

/*********************************************************************//*
\brief Callback function for the ending of Activation procedure
 ************************************************************************/
static void cert_joindata_callback(StackRetStatus_t status);

/*********************************************************************//*
 \brief      Set MAC  parameters function
 \return     LORAWAN_SUCCESS, if successfully set the join parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
static StackRetStatus_t cert_set_mac_parameters(void);

/*********************************************************************//*
 \brief    Activates Test Mode
 ************************************************************************/
static void cert_enter_test_mode(void);

/*********************************************************************//*
 \brief      Function to Print array of characters
 \param[in]  *array  - Pointer of the array to be printed
 \param[in]   length - Length of the array
 ************************************************************************/
static void print_array (uint8_t *array, uint8_t length);

/*********************************************************************//*
 \brief      Function to Print stack return status
 \param[in]  status - Status from the stack
 ************************************************************************/
static void print_stack_status(StackRetStatus_t status);

/***************************** FUNCTIONS ***************************************/

/*********************************************************************//**
\brief    Initialization the Demo application
*************************************************************************/
void cert_app_init(void)
{
    StackRetStatus_t status;

#if (ENABLE_PDS == 1)
    PDS_RestoreAll();
#endif

    status = SwTimerCreate(&certAppTimerId);

    if(status!=LORAWAN_SUCCESS)
    {
        printf("\r\nUnable to start certification timer. Pls check");
        while(1);
    }
    
    status = SwTimerCreate(&tx_cw_timer);
    if(status!=LORAWAN_SUCCESS)
    {
        printf("\r\nUnable to create CW TX timer. Pls check");
        while(1);
    }

    /* Initialize the LORAWAN Stack */
    LORAWAN_Init(cert_appdata_callback, cert_joindata_callback);

/*
    if (sizeof(bandTable) < 2)
    {
        printf("\nCertification cannot be run for the current regional band configuration\n\r");
        while(1);
    }
*/
    while(!cert_select_band());

    status = cert_set_mac_parameters();

    if (LORAWAN_SUCCESS != status)
    {
        printf("\nMAC parameters initialization failed\n\r");
    }
    /* Initialize the join parameters */
    status = cert_set_join_parameters(LORAWAN_ABP);
    if (LORAWAN_SUCCESS != status)
    {
        printf("\nJoin parameters initialization failed\n\r");
    }

    status = cert_set_join_parameters(LORAWAN_OTAA);
    if (LORAWAN_SUCCESS != status)
    {
        printf("\nOTAA Join parameters initialization failed\n\r");
    }
    /* Send Join request */
    status = LORAWAN_Join(CERT_APP_ACTIVATION_TYPE);
    if (status == LORAWAN_SUCCESS)
    {
        printf("\n%s Join Request Sent\n\r", (LORAWAN_OTAA == CERT_APP_ACTIVATION_TYPE) ? "OTAA" : "ABP");
    }
    else
    {
        print_stack_status(status);
    }
}

/*********************************************************************//*
 \brief      Select Band to reset LORAWAN Stack
 \return     true, if a valid band is selected
             false, otherwise
 ************************************************************************/
static bool cert_select_band(void)
{
    printf("\r\nPlease select one of the band given below\r\n");
    for(uint8_t i = 1;i < sizeof(bandTable); i++)
    {
        printf("%d. %s\r\n",i,bandStrings[i]);
    }
    printf("Select Regional Band : ");

    char rxChar;
    rxChar = sio2host_getchar();
    const char *charPtr = &rxChar;
    current_band = atoi(charPtr);
    if ( (current_band >= sizeof(bandTable)) || (current_band == 0) )
    {
        return false;
    }
    else
    {
      LORAWAN_Reset(bandTable[current_band]);
	  #if (NA_BAND == 1 || AU_BAND == 1)
	  if ((bandTable[current_band] == ISM_NA915) || (bandTable[current_band] == ISM_AU915))
	  {
		  cert_enable_channels();
	  }
	 #endif
    }

    printf("\n\n\r........................................\n\r");
//    printf("\n\rMicrochip LoRaWAN Stack %s\r\n",STACK_VER);
    printf("\r\nInit - Successful\r\n");
    return true;
}

/*********************************************************************//*
 \brief      Enable channels if NA or AU band is selected
 \return     None
 ************************************************************************/
static void cert_enable_channels(void)
{
	#if (RANDOM_NW_ACQ == 0)
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
	#endif	
}

/*********************************************************************//*
 \brief      Function to Print stack return status
 \param[in]  status - Status from the stack
 ************************************************************************/
static void print_stack_status(StackRetStatus_t status)
{
    printf("\n>> %s\n\r", status_message[status]);

}

/*********************************************************************//**
\brief Callback function for the ending of Bidirectional communication of
       Application data
 *************************************************************************/
static void cert_appdata_callback(void *appHandle, appCbParams_t *appdata)
{
    if (LORAWAN_EVT_RX_DATA_AVAILABLE == appdata->evt)
    {
        uint8_t *pData = appdata->param.rxData.pData;
        uint8_t dataLength = appdata->param.rxData.dataLength;
        StackRetStatus_t status = appdata->param.rxData.status;
        uint32_t downcounter;
        LORAWAN_GetAttr(DOWNLINK_COUNTER, NULL, &downcounter);

        switch(status)
        {
            case LORAWAN_SUCCESS:
            {
                 pktRxd = true;
                 cnfTxInProgress = false;
                 //Successful transmission
                 if((dataLength > 0U) && (NULL != pData))
                 {
                     printf("\n--------------- Downlink ---------------\n\r");
                     printf("\n  Fport : %d", pData[0]);
                     printf("\nPayload : ");
                     for (uint8_t i = 0; i < dataLength - 1; i++)
                     {
                         printf("%02X", pData[i+1]);
                     }
                     printf("\n\r");

                     if(pData[0] == TEST_PORT_NB)
                     {
                        cert_handle_cert_rx_data(pData+1,dataLength-1);
                     }
                 }
                 else
                 {
                     uplinkTestNoResp++;
                     printf("\nReceived ACK for Confirmed data\r\n");
                 }
                 uplinkTestNoResp = 0;

            }
            break;


            default:
                ;
            break;
        }
        print_stack_status(status);
    }
    else if(LORAWAN_EVT_TRANSACTION_COMPLETE == appdata->evt)
    {
        uint8_t lastchid;
        uint32_t upcounter;
        LORAWAN_GetAttr(UPLINK_COUNTER, NULL, &upcounter);
        LORAWAN_GetAttr(LAST_CH_ID, NULL, &lastchid);
        printf("\n--------- Transaction Complete ---------\n\r");
        printf("\nLast uplink channel: %d, fcntup: %04X\n\r", lastchid, (uint16_t)((upcounter-1) & 0x0000FFFF));
        switch(appdata->param.transCmpl.status)
        {
            case LORAWAN_SUCCESS:
            {
                if (cnfTxInProgress)
                {
                    if (false == pktRxd)
                    {
                        /*
                        * In case a CNF uplink receives ACK in empty frame
                        */
                        downlinkCtr++;
                    }                   
                    cnfTxInProgress = false;
                }
                if(pktRxd == false)
                {
                    //printf("\nTransmission Success\r\n");
                    // No answer received from the server side
                    // Increment no response counter
                    uplinkTestNoResp++;
                    //prepare the next data
                    sendData[0] = (uint8_t) (downlinkCtr >> 8);
                    sendData[1] = (uint8_t) (downlinkCtr);
                    sendDataLen = 2;
                }
            }
            break;

            default:
                ;//printf("\r\nMAC NOK! - %d", appdata->param.transCmpl.status );
            break;
        }
        print_stack_status(appdata->param.transCmpl.status);
        printf("\n========================================\n\r");
    }
}

/*********************************************************************//*
\brief Callback function for the ending of Activation procedure
 ************************************************************************/
static void cert_joindata_callback(StackRetStatus_t status)
{
    uint32_t devaddr;
    StackRetStatus_t stackRetStatus;
    /* This is called every time the join process is finished */
    if(LORAWAN_SUCCESS == status)
    {
        printf("\nJoin Successful\n\r");
        LORAWAN_GetAttr(DEV_ADDR, NULL, &devaddr);
        printf("\nNew DeviceAddr: %08X\n\r", (unsigned int)devaddr);
        /*Start the Timer to send data periodically*/
        stackRetStatus = SwTimerStart(certAppTimerId, MS_TO_US(certAppTimeout), SW_TIMEOUT_RELATIVE, (void*)cert_app_timer_callback, NULL);
        printf("\nStart certAppTimerId\n\r");
    }
    else
    {
        printf("\nJoin Denied\n\r");
        print_stack_status(status);
        stackRetStatus = LORAWAN_Join(LORAWAN_OTAA);
        printf("\nOTAA Join Request Sent\n\r");
        print_stack_status(stackRetStatus);
    }
}

/*********************************************************************//*
 \brief      Set join parameters function
 \param[in]  activation type - notifies the activation type (OTAA/ABP)
 \return     LORAWAN_SUCCESS, if successfully set the join parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t cert_set_join_parameters(ActivationType_t activation_type)
{
    StackRetStatus_t status;
#ifdef CRYPTO_DEV_ENABLED
	bool cryptoDevEnabled = true;
	LORAWAN_SetAttr(CRYPTODEVICE_ENABLED, &cryptoDevEnabled );
#endif /* #ifdef CRYPTO_DEV_ENABLED */
	uint8_t dataRate;
	
	if(bandTable[current_band] == ISM_THAI923 || bandTable[current_band] == ISM_JPN923)
	dataRate = DR2;
	else
	dataRate = DR0;
	
	status = LORAWAN_SetAttr (CURRENT_DATARATE, &dataRate);

    printf("\n----------- Join Parameters ------------\n\r");

    if(ACTIVATION_BY_PERSONALIZATION == activation_type)
    {
		if (LORAWAN_SUCCESS == status)
		{
			status = LORAWAN_SetAttr (DEV_ADDR, &devAddr);
		}
		
        if (LORAWAN_SUCCESS == status)
        {
            printf("\nDevice Addr - ");
            printf("0x%lx \n\r", devAddr);
            status = LORAWAN_SetAttr (APPS_KEY, appsKey);
        }

        if (LORAWAN_SUCCESS == status)
        {
            printf("\nApplication Session Key - ");
            print_array((uint8_t *)&appsKey, sizeof(appsKey));
            status = LORAWAN_SetAttr (NWKS_KEY, nwksKey);
        }

        if (LORAWAN_SUCCESS == status)
        {
            printf("\nNetwork Session Key - ");
            print_array((uint8_t *)&nwksKey, sizeof(nwksKey));
        }

    }
    else
    {
		if (LORAWAN_SUCCESS == status)
		{
			status = LORAWAN_SetAttr (DEV_EUI, devEui);
		}
		
        if (LORAWAN_SUCCESS == status)
        {
            printf("\nDevice EUI - ");
            print_array((uint8_t *)&devEui, sizeof(devEui));
            status = LORAWAN_SetAttr (JOIN_EUI, joinEui);
        }

        if (LORAWAN_SUCCESS == status)
        {
            printf("\nApplication EUI - ");
            print_array((uint8_t *)&joinEui, sizeof(joinEui));
            status = LORAWAN_SetAttr (APP_KEY, appKey);
        }

        if (LORAWAN_SUCCESS == status)
        {
            printf("\nApplication Key - ");
            print_array((uint8_t *)&appKey, sizeof(appKey));
        }
    }
    return status;
}

/*********************************************************************//*
 \brief      Function to Print array of characters
 \param[in]  *array  - Pointer of the array to be printed
 \param[in]   length - Length of the array
 ************************************************************************/
static void print_array (uint8_t *array, uint8_t length)
{     
    for (uint8_t i =0; i < length; i++)
    {
        printf("%02X", *array);
        array++;
    }
    printf("\n\r");
}

/*********************************************************************//*
 \brief    Certification Timer Callback
 ************************************************************************/
void cert_app_timer_callback(uint8_t param)
{
    StackRetStatus_t status;
    uint32_t upcounter;
    uint8_t datarate;
    SwTimerStart(certAppTimerId, MS_TO_US(certAppTimeout), SW_TIMEOUT_RELATIVE, (void *)cert_app_timer_callback, NULL);
    LORAWAN_GetAttr(UPLINK_COUNTER, NULL, &upcounter);
    LORAWAN_GetAttr(CURRENT_DATARATE, NULL, &datarate);
    
    if (clear_linkcheck)
    {
      uint16_t lcp = 0;
      clear_linkcheck = false;
      LORAWAN_SetAttr(LINK_CHECK_PERIOD, &lcp);
    }

    lorawanSendReq.buffer = sendData;
    lorawanSendReq.bufferLength = sendDataLen;
    lorawanSendReq.confirmed = bTxCnf;
    lorawanSendReq.port = sendFport;
    status = LORAWAN_Send(&lorawanSendReq);
    sendFport = CERT_APP_FPORT;
    
    printf("\n---------------- Uplink ----------------\n\r");
    printf("\n     DR : %d", datarate);
    printf("\n   Type : %s", lorawanSendReq.confirmed ? "Cnf" : "UnCnf");
    printf("\n  Fport : %d", lorawanSendReq.port);
    printf("\n FCntUp : %ld (%08X)", upcounter, (unsigned int)upcounter);
    printf("\nPayload : ");
    print_array(lorawanSendReq.buffer,lorawanSendReq.bufferLength);
    if (LORAWAN_SUCCESS == status)
    {
        pktRxd = false;
        if (lorawanSendReq.confirmed)
        {
            cnfTxInProgress = true;
        }
    }
    print_stack_status(status);
}

/*********************************************************************//*
 \brief    Activates Test Mode
 ************************************************************************/
static void cert_enter_test_mode(void)
{
    testMode = ON;
    downlinkCtr = 0;
    uplinkTestNoResp = 0;

    //prepare the first data
    sendData[0] = (uint8_t) (downlinkCtr >> 8);
    sendData[1] = (uint8_t) (downlinkCtr);
    sendDataLen = 2;
    printf("\r\nTest Mode Activated\r\n");
}

/*********************************************************************//*
 \brief      Set MAC  parameters function
 \return     LORAWAN_SUCCESS, if successfully set the join parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
static StackRetStatus_t cert_set_mac_parameters(void)
{
    StackRetStatus_t status = LORAWAN_SUCCESS;
    bool adrValue = true ;
    bool testModeEnable = true;
    status = LORAWAN_SetAttr(TEST_MODE_ENABLE,&testModeEnable);

    if(status == LORAWAN_SUCCESS)
    {
        status = LORAWAN_SetAttr(ADR,&adrValue);
    }

    return status;
}

/*********************************************************************//*
 \brief      Function that processes the Rx data
 \param[in]  data - Rx data payload
 \param[in]  dataLen - The number of Rx bytes
 ************************************************************************/
static void cert_handle_cert_rx_data(uint8_t* data,uint8_t dataLen)
{
    StackRetStatus_t status;

    if (dataLen > 0)
    {
        downlinkCtr++;
        uplinkTestNoResp = 0;

        //prepare the next data
        sendData[0] = (uint8_t) (downlinkCtr >> 8);
        sendData[1] = (uint8_t) (downlinkCtr);
        sendDataLen = 2;
        
        if (enddevice_cpc_is_payload_valid(data, dataLen))
        {
            enddevice_cpc_execute_command(data, dataLen);
        }
        else
        {
            switch (data[0])
            {
                case DEACTIVATE_MODE:
                {
                    if (dataLen == 1) {
                        //Deactivated test mode
                        testMode = OFF;
                    }
                }
                break;

                case ACTIVATE_MODE:
                {
                    bool bPayloadValid = false;
                    if (dataLen == 4)
                    {
                        for (uint8_t i = 0; i < 4; i++)
                        {
                            if (data[i] == 1)
                            {
                                bPayloadValid = true;
                            }
                            else
                            {
                                bPayloadValid = false;
                                break;
                            }
                        }
                    }

                    if (bPayloadValid)
                    {
                        //Activated test mode
                        cert_enter_test_mode();
                    }
                }
                break;

                case CNF_MODE:
                {
                    if (dataLen == 1)
                    {
                        //Confirmed frames
                        bTxCnf = true;
                    }
                }
                break;

                case UNCNF_MODE:
                {
                    if (dataLen == 1)
                    {
                        //Unconfirmed frames
                        bTxCnf = false;
                    }
                }
                break;

                case CRYPTO_MODE:
                {
                    if (dataLen <= 33) {
                        //Cryptography tests
                        sendData[0] = 0x04;

                        for (uint8_t i = 1; i < dataLen; i++)
                        {
                            sendData[i] = (((uint16_t) data[i]) + 1) % 256;
                        }
                        sendDataLen = dataLen;
                    }
                }
                break;

                case OTAA_TRIGGER_MODE:
                {
                    status = LORAWAN_Join(LORAWAN_OTAA);
                    if (status == LORAWAN_SUCCESS)
                    {
                        testMode = OFF;
                        SwTimerStop(certAppTimerId);
                        printf("\nOTAA Join Request Sent\n\r");
                    }
                    else
                    {
                        print_stack_status(status);
                    }
                }
                break;

                default:
                break;
            }
        }        
    }
}

#endif /* CERT_APP == 1 */
/* eof demo_app.c */

