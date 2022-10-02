/**
* \file  msg.c
*
* \brief LoRaWAN application messaging support implementation
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

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include "lorawan.h"
#include "radio_interface.h"
#include "sw_timer.h"
#include "msg.h"
#include "conf_app.h"
#include "log.h"

/************************************************************************/
/* STATIC FUNCTION PROTOTYPES                                           */
/************************************************************************/
static void messageReceiveProcess(void);
static void messageSendProcess(void);
static void messageTimerCb(void);
static void processTransactionComplete(void *appHandle, appCbParams_t *params);
static void processRxDataAvailable( void *appHandle, appCbParams_t *params );
static uint8_t messageGetIdByGroupAddr(uint32_t devaddr);
static bool messageIsDutyCyclePending(void);
static MessageState_t msgState;

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
uint8_t txBufMem[MSG_TX_BUFFER_SIZE];
uint8_t rxBufMem[MSG_TX_BUFFER_SIZE];
uint8_t tmrMsg;
uint8_t fportCallbackAlloc;
MsgCallback_t msgDefaultRxDataAvailCallback;
MsgCallback_t msgTransactionCompleteCallback;
FportCallbacks_t fportCallbacks[MSG_FPORT_CB_MAX];
MessageRx_t msgRx;
MessageTx_t msgTx;

/************************************************************************/
/* FUNCTION DEFINITIONS                                                 */
/************************************************************************/
void MSG_Init( void )
{
    StackRetStatus_t retval;
    msgState.value = 0;
    fportCallbackAlloc = 0;
    msgDefaultRxDataAvailCallback = NULL;
    msgTransactionCompleteCallback = NULL;
    
    retval = SwTimerCreate( &tmrMsg );
    if (retval != LORAWAN_SUCCESS)
    {
        LOGLINE_ERROR("%s: tmrMsg creation failed", __FUNCTION__);
    }
}

uint16_t MSG_GetState( void )
{
    return msgState.value;
}

AppResult_t MSG_Send( TransmissionType_t msgType, uint8_t port, uint8_t *data,
    uint8_t length, uint32_t timeout )
{
    if ( msgState.transactionInProgress )
    {
        LOGLINE_WARN("Previous transaction still in progress");
        return APP_RESULT_BUSY;
    }

    msgTx.lorawanReq.confirmed = (bool) msgType;
    msgTx.lorawanReq.port = port;
    if ( data )
    {
        memcpy(txBufMem, data, length);
    }
    msgTx.lorawanReq.buffer = txBufMem;
    msgTx.lorawanReq.bufferLength = length;
    msgTx.retries = APP_SENDRETRY_COUNT;
    msgTx.timeout = timeout;

    msgState.transactionInProgress = true;
    msgState.txWaiting = true;
    msgState.classRx2Dl = false;
    SYSTEM_PostTask( APP_TASK_ID );
    return APP_RESULT_OK;
}

void MSG_DataCallback( void *appHandle, appCbParams_t *params )
{
    switch ( params->evt )
    {
        case LORAWAN_EVT_TRANSACTION_COMPLETE:
        {
            processTransactionComplete( appHandle, params );
        }
        break;
        
        case LORAWAN_EVT_RX_DATA_AVAILABLE:
        {
            processRxDataAvailable( appHandle, params );
        }
        break;

        default:
        // keep compiler happy
        break;
    }
}

void MSG_SetFPortRxDataAvailCallback( uint8_t port, MsgCallback_t callback )
{
    if ( MSG_FPORT_CB_MAX > fportCallbackAlloc )
    {
        fportCallbacks[ fportCallbackAlloc ].port = port;
        fportCallbacks[ fportCallbackAlloc ].pFunc = callback;
        fportCallbackAlloc++;        
    }
    else
    {
        LOGLINE_WARN("%s: Exceeds maximum fport callback count", __FUNCTION__);
    }
}

void MSG_SetDefaultRxDataAvailCallback( MsgCallback_t callback )
{
    msgDefaultRxDataAvailCallback = callback;
}

void MSG_SetTransactionCompleteCallback( MsgCallback_t callback )
{
    msgTransactionCompleteCallback = callback;
}

void MSG_Process( void )
{
    messageReceiveProcess( );
    messageSendProcess( );
}

bool MSG_IsTxPossible( uint8_t len, uint32_t *txTime )
{
    uint8_t size;
    uint32_t duration = 0;
    TimeOnAirParams_t tp;
    EdClass_t class;
    uint32_t outputTxTime = 0;

    outputTxTime = 0;
    
    LORAWAN_GetAttr(NEXT_PAYLOAD_SIZE, NULL, &size);
    if (len > size)
    { /* if tx not possible return 0 */
        return false;
    }

    if ( messageIsDutyCyclePending() )
    {
        return false;
    }

    memset(&tp, 0x00, sizeof(TimeOnAirParams_t));
    tp.pktLen = (size + len);
    LORAWAN_GetAttr(CURRENT_DATARATE, NULL, &tp.dr);
    RADIO_GetAttr(CRC_ON, &tp.crcOn);
    RADIO_GetAttr(ERROR_CODING_RATE, &tp.cr);
    RADIO_GetAttr(PREAMBLE_LEN, &tp.preambleLen);
    LORAWAN_GetAttr(PACKET_TIME_ON_AIR, &tp, &outputTxTime);

    /* If payload size and duty cycle allows the tx then return the timeOnAir */
    outputTxTime = US_TO_MS(outputTxTime);
    LORAWAN_GetAttr(RX_DELAY2, NULL, &duration);
    outputTxTime += duration;

    *txTime = outputTxTime;
    
    LORAWAN_GetAttr(EDCLASS, NULL, &class);
    if (CLASS_C == class)
    {
        if (msgState.classRx2Dl) /* Means something is received just now */
        {
            if ( outputTxTime >= MSG_CLASS_C_INTER_DOWNLINK_TIME_MS  )
            {
                LOGLINE_DEBUG(
                    "%s: Uplink airtime < MSG_CLASS_C_INTER_DOWNLINK_TIME_MS",
                    __FUNCTION__);
                return false;
            }
        }
        else
        {
            LOGLINE_DEBUG(
                "In class-C mode, do uplink after receiving downlink");
            return false;
        }
    }
    else
    {
        outputTxTime += MSG_MAX_RX_WINDOW_TIME_MS;
        *txTime = outputTxTime;
    }
    return true;
}

/************************************************************************/
/* STATIC FUNCTION DEFINITIONS                                          */
/************************************************************************/
static void messageReceiveProcess( void )
{
    uint8_t fport, groupId;
    bool isMcast;
    uint32_t devaddr, fcnt;
    
    if ( ! msgState.rxPending )
    {
        return;
    }

    LORAWAN_GetAttr(DEV_ADDR, NULL, &devaddr);
    isMcast = (msgRx.params.param.rxData.devAddr != devaddr);

    
    if (isMcast)
    {
        groupId = messageGetIdByGroupAddr(msgRx.params.param.rxData.devAddr);
    }
    
    LOGLINE_DEBUG("%scast packet received", isMcast ? "Multi" : "Uni");        
                
    LORAWAN_GetAttr(isMcast ? MCAST_FCNT_DOWN :
                              DOWNLINK_COUNTER, &groupId, &fcnt);

    fport = *(msgRx.params.param.rxData.pData);
    
    LOGLINE_DEBUG("FPort   = %d", fport);
    LOGLINE_DEBUG("Length  = %d", msgRx.params.param.rxData.dataLength-1);
    LOGLINE_DEBUG("Fcnt    = %lu", fcnt);
    LOGLINE_DEBUG("Payload:\r\n");
    for (uint8_t i = 1; i < msgRx.params.param.rxData.dataLength; i++)
    {
        LOGMSG_DEBUG("%02x", msgRx.params.param.rxData.pData[i]);
    }
    
    for (uint8_t i = 0; i < 5; i++)
    {
        if ( fportCallbacks[i].port == fport )
        {
            if ( fportCallbacks[i].pFunc )
            {
                fportCallbacks[i].pFunc( &msgRx.params );
                msgState.rxPending = false;
                return;
            }
        }
    }
    
    if ( msgDefaultRxDataAvailCallback )
    {
        msgDefaultRxDataAvailCallback( &msgRx.params );        
    }

    msgState.rxPending = false;
}

static void messageSendProcess( void )
{
    if ( ! msgState.transactionInProgress )
    {
        return;
    }

    if ( msgTx.timeout )
    {        
        SwTimerStart(tmrMsg,
                     MS_TO_US( msgTx.timeout ),
                     SW_TIMEOUT_RELATIVE,
                     messageTimerCb,
                     NULL );
        LOGLINE_DEBUG("Send delay timeout started: %lums", msgTx.timeout);
        msgTx.timeout = 0;       
    }
    else if ( msgState.txWaiting )
    {
        if ( LORAWAN_SUCCESS == LORAWAN_Send( &msgTx.lorawanReq ) )
        {
            uint32_t fCntUp;
            uint8_t dr;
            
            msgState.txWaiting = false;
            
            LORAWAN_GetAttr(CURRENT_DATARATE, NULL, &dr);
            LORAWAN_GetAttr(UPLINK_COUNTER, NULL, &fCntUp);
            LOGLINE_DEBUG(
                "Sending... DR%u, FPort=%d, FcntUP=%lu, FRMPayloadLen=%d",
                dr, msgTx.lorawanReq.port, fCntUp,
                msgTx.lorawanReq.bufferLength);
        }
        else
        {
            if ( 0 != msgTx.retries )
            {
                msgTx.retries--;
                SwTimerStart(tmrMsg,
                            MS_TO_US( APP_SENDRETRYINTERVAL_MS ),
                            SW_TIMEOUT_RELATIVE,
                            messageTimerCb,
                            NULL );
            }
        }
    }
}

static void messageTimerCb( void )
{
    if ( msgState.transactionInProgress )
    {
        msgState.txWaiting = true;
        SYSTEM_PostTask( APP_TASK_ID );
    }
}

static void processTransactionComplete(void *appHandle, appCbParams_t *params)
{
    uint32_t dcycle;
    LORAWAN_GetAttr(PENDING_DUTY_CYCLE_TIME, NULL, &dcycle);
    
    switch ( params->param.transCmpl.status )
    {
        case LORAWAN_RADIO_CHANNEL_BUSY:
        case LORAWAN_NO_CHANNELS_FOUND:
        case LORAWAN_NO_ACK:
        {
            if ( (0 != msgTx.retries) )
            {
                uint32_t timeout = messageIsDutyCyclePending() ?
                                   dcycle : APP_SENDRETRYINTERVAL_MS;
                msgTx.retries--;
                LOGLINE_DEBUG("Send unsuccessful..! Retries left = %u",
                    (msgTx.retries + 1));                
                SwTimerStart(tmrMsg,
                             MS_TO_US(timeout),
                             SW_TIMEOUT_RELATIVE,
                             messageTimerCb,
                             NULL);
                break;
            }
            else
            { // fall through default
            }
        }
        default:
        {
            if ( msgTransactionCompleteCallback )
            {
                msgTransactionCompleteCallback( params );
            }
            msgState.transactionInProgress = false;
        }
        break;
    }
}

static void processRxDataAvailable( void *appHandle, appCbParams_t *params )
{
    EdClass_t class;
    memcpy( &msgRx.params, params, sizeof(appCbParams_t) );
    msgRx.params.param.rxData.pData = rxBufMem;
    memcpy(rxBufMem, (uint8_t *)params->param.rxData.pData,
        params->param.rxData.dataLength);  

    msgState.rxPending = true;

    LORAWAN_GetAttr(EDCLASS, NULL, &class);
    if (CLASS_C == class)
    {
        msgState.classRx2Dl = true;
        LOGLINE_DEBUG("DOWNLINK in class-C RXC_CONT");
    }
    
    SYSTEM_PostTask(APP_TASK_ID);
}

static uint8_t messageGetIdByGroupAddr(uint32_t devaddr)
{
    uint8_t id = UINT8_MAX;
    
    for (uint8_t i = 0; i < 4; i++)
    {
        uint32_t addr;
        LORAWAN_GetAttr(MCAST_GROUP_ADDR, &i, &addr);
        if (devaddr == addr)
        {
            id = i;
            break;
        }
    }

    return id;
}

static bool messageIsDutyCyclePending( void )
{
    uint32_t duration;
    if (LORAWAN_SUCCESS == \
        LORAWAN_GetAttr(PENDING_DUTY_CYCLE_TIME, NULL, &duration))
    {
        if ((duration > 0) && (duration < UINT32_MAX))
        {
            LOGLINE_DEBUG("Dutycycle pending: %lums", duration);
            return true;
        }
    }    
    return false;
}

/* eof msg.c */
