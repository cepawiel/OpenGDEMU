/**
* \file  enddevice_cert.c
*
* \brief LORAWAN Certification Application
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


#if (CERT_APP == 1)

/****************************** INCLUDES **************************************/
#include "asf.h"
#include "lorawan.h"
#include "system_task_manager.h"
#include "enddevice_cert.h"
#include "conf_certification.h"
#include "sio2host.h"
#include "sw_timer.h"
#if (ENABLE_PDS == 1)
#include "pds_interface.h"
#endif

/************************** STATIC VARIABLES ***********************************/
static uint8_t testMode = OFF;
static uint8_t sendData[100];
static uint16_t sendDataLen;
static uint16_t downlinkCtr = 0;
static bool bTxCnf = false;
static uint8_t uplinkTestNoResp = 0;
static bool pktRxd = false;
static uint8_t certAppTimerId;
static LorawanSendReq_t lorawanSendReq;
/*ABP Join Parameters */
static uint32_t devAddr = CERT_DEVICE_ADDRESS;
static uint8_t nwksKey[16] = CERT_NETWORK_SESSION_KEY;
static uint8_t appsKey[16] = CERT_APPLICATION_SESSION_KEY;
/* OTAA join parameters */
static uint8_t devEui[8] = CERT_DEVICE_EUI;
static uint8_t appEui[8] = CERT_APPLICATION_EUI;
static uint8_t appKey[16] = CERT_APPLICATION_KEY;
static const char* bandStrings[] =
{
    #if (EU_BAND == 1)
    "EU868",
    #endif
};


static uint8_t bandTable[] =
{
    0xFF,
    #if (EU_BAND == 1)
    ISM_EU868,
    #endif
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
 \brief      Set join parameters function
 \param[in]  activation type - notifies the activation type (OTAA/ABP)
 \return     LORAWAN_SUCCESS, if successfully set the join parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
static StackRetStatus_t cert_set_join_parameters(ActivationType_t activation_type);

/*********************************************************************//*
 \brief    Certification Timer Callback
 ************************************************************************/
static void cert_app_timer_callback(uint8_t param);

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
    PDS_DeleteAll();

    PDS_UnInit();
#endif

    status = SwTimerCreate(&certAppTimerId);

    if(status!=LORAWAN_SUCCESS)
    {
        printf("\r\nUnable to start certification timer. Pls check");
        while(1);
    }

    /* Initialize the LORAWAN Stack */
    LORAWAN_Init(cert_appdata_callback, cert_joindata_callback);

    if (sizeof(bandTable) < 2)
    {
        printf("\nCertification cannot be run for the current regional band configuration\n\r");
        while(1);
    }

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
        printf("\nJoin Request Sent\n\r");
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
    uint8_t choice;
    rxChar = sio2host_getchar();
    const char *charPtr = &rxChar;
    choice = atoi(charPtr);
    if ( (choice >= sizeof(bandTable)) || (choice == 0) )
    {
        return false;
    }
    else
    {
        LORAWAN_Reset(bandTable[choice]);
    }

    printf("\n\n\r*******************************************************\n\r");
//    printf("\n\rMicrochip LoRaWAN Stack %s\r\n",STACK_VER);
    printf("\r\nInit - Successful\r\n");
    return true;
}

/*********************************************************************//*
 \brief      Function to Print stack return status
 \param[in]  status - Status from the stack
 ************************************************************************/
static void print_stack_status(StackRetStatus_t status)
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
        uint32_t devAddress = appdata->param.rxData.devAddr;

        switch(status)
        {
            case LORAWAN_SUCCESS:
            {
                 pktRxd = true;
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

                     if(pData[0] == TEST_PORT_NB)
                     {
                       cert_handle_cert_rx_data(pData+1,dataLength-1);
                     }
                 }
                 else
                 {
                     uplinkTestNoResp++;
                     printf("Received ACK for Confirmed data\r\n");
                 }
                 uplinkTestNoResp = 0;

            }
            break;


            default:
                printf("\r\nMAC NOK! - %d", status);
            break;
        }
    }
    else if(LORAWAN_EVT_TRANSACTION_COMPLETE == appdata->evt)
    {
        switch(appdata->param.transCmpl.status)
        {
            case LORAWAN_SUCCESS:
            {
                if(pktRxd == false)
                {
                    printf("Transmission Success\r\n");
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
                printf("\r\nMAC NOK! - %d", appdata->param.transCmpl.status );
            break;
        }
        printf("\n\r*************************************************\n\r");
    }

}

/*********************************************************************//*
\brief Callback function for the ending of Activation procedure
 ************************************************************************/
static void cert_joindata_callback(StackRetStatus_t status)
{
    StackRetStatus_t stackRetStatus;
    /* This is called every time the join process is finished */
    if(LORAWAN_SUCCESS == status)
    {
        printf("\nJoining Successful\n\r");

        /*Start the Timer to send data periodically*/
        stackRetStatus = SwTimerStart(certAppTimerId, MS_TO_US(CERT_APP_TIMEOUT), SW_TIMEOUT_RELATIVE, (void*)cert_app_timer_callback, NULL);
        if(stackRetStatus != LORAWAN_SUCCESS)
        {
            printf("ERROR : Unable to start Certification Timer\r\n");
        }
    }
    else
    {
        stackRetStatus = LORAWAN_Join(LORAWAN_OTAA);
        if (stackRetStatus == LORAWAN_SUCCESS)
        {
            printf("\nOTAA Join Request Sent\n\r");
        }
        else
        {
            print_stack_status(stackRetStatus);
        }
    }
    printf("\n\r*******************************************************\n\r");
}

/*********************************************************************//*
 \brief      Set join parameters function
 \param[in]  activation type - notifies the activation type (OTAA/ABP)
 \return     LORAWAN_SUCCESS, if successfully set the join parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
static StackRetStatus_t cert_set_join_parameters(ActivationType_t activation_type)
{
    StackRetStatus_t status;

	uint8_t dataRate = DR0;
	status = LORAWAN_SetAttr (CURRENT_DATARATE, &dataRate);

    printf("\n********************Join Parameters********************\n\r");

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
            status = LORAWAN_SetAttr (APP_EUI, appEui);
        }

        if (LORAWAN_SUCCESS == status)
        {
            printf("\nApplication EUI - ");
            print_array((uint8_t *)&appEui, sizeof(appEui));
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
    printf("0x");
    for (uint8_t i =0; i < length; i++)
    {
        printf("%02x", *array);
        array++;
    }
    printf("\n\r");
}

/*********************************************************************//*
 \brief    Certification Timer Callback
 ************************************************************************/
static void cert_app_timer_callback(uint8_t param)
{
    StackRetStatus_t status;
    SwTimerStart(certAppTimerId, MS_TO_US(CERT_APP_TIMEOUT), SW_TIMEOUT_RELATIVE, (void *)cert_app_timer_callback, NULL);

    if(testMode == OFF)
    {
        lorawanSendReq.buffer = sendData;
        lorawanSendReq.bufferLength = sendDataLen;
        lorawanSendReq.confirmed = CERT_APP_TRANSMISSION_TYPE;
        lorawanSendReq.port = CERT_APP_FPORT;
        status = LORAWAN_Send(&lorawanSendReq);
    }
    else
    {
        lorawanSendReq.buffer = sendData;
        lorawanSendReq.bufferLength = sendDataLen;
        lorawanSendReq.confirmed = bTxCnf;
        lorawanSendReq.port = TEST_PORT_NB;
        status = LORAWAN_Send(&lorawanSendReq);
    }

    if (LORAWAN_SUCCESS == status)
    {
        pktRxd = false;
        printf("\nFrame Sent:");
        print_array(lorawanSendReq.buffer,lorawanSendReq.bufferLength);
    }
    else
    {
        print_stack_status(status);
    }
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

#endif /* CERT_APP == 1 */
/* eof demo_app.c */

