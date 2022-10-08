/**
* \file  msg.h
*
* \brief LoRaWAN application messaging support interfaces
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
#ifndef _MSG_H_
#define _MSG_H_

/************************************************************************/
/* CONSTANTS                                                            */
/************************************************************************/
#define MSG_RX_BUFFER_SIZE                  (255)
#define MSG_TX_BUFFER_SIZE                  (255)
#define MSG_FPORT_CB_MAX                    (5)
#define MSG_CLASS_C_INTER_DOWNLINK_TIME_MS  (3000)
#define MSG_MAX_RX_WINDOW_TIME_MS           (3000)

/************************************************************************/
/* TYPE DEFINES                                                         */
/************************************************************************/
/*
* Type of the function callback invoked by messaging interface
*/
typedef void (*MsgCallback_t)(appCbParams_t *params );

/*
* List of status codes returned by messaging interface
*/
typedef enum _MessageStatus_t
{
    /* if it is possible to post the request to MAC */
    APP_RESULT_OK = 0,
    /*
    * in all other cases such as duty cycle pending,
    * previous message retry pending etc.
    */
    APP_RESULT_BUSY
} AppResult_t;

/*
* Structure of the message transmit request
*/
typedef struct _MessageTx_t
{
    /* Info for the MAC layer send request */
    LorawanSendReq_t lorawanReq;
    /* Number of retries left */
    uint8_t retries;
    /* Delay to elapse before the first transmit */
    uint32_t timeout;
} MessageTx_t;

/*
* Structure of the message receive indication
*/
typedef struct _MessageRx_t
{
    /* Info from the MAC layer receive done */
    appCbParams_t params;
} MessageRx_t;

/*
* State of the messaging support
*/
typedef union _MessageState_t
{
    /* Status bitfield */
    uint16_t value;
    /* Definition of individual bits */
    struct {
        /*
        * 0 -> messaging interface is idle
        * 1 -> previous message transaction yet to complete
        */
        uint16_t transactionInProgress : 1;
        /*
        * 0 -> no message pending for transmission
        * 1 -> retry or new message is pending for transmission
        */
        uint16_t txWaiting  : 1;
        /*
        * 0 -> no new message is received
        * 1 -> new message received and waiting for processing
        */
        uint16_t rxPending  : 1;
        /*
        * 0 -> no new message received in class-C RXC_CONT window
        * 1 -> new message received in class-C RXC_CONT window and pending
        */
        uint16_t classRx2Dl : 1;
    };
} MessageState_t;

/*
* Structure of Fport callback info
*/
typedef struct _FportCallbacks_t
{
    /* Frame port number */
    uint8_t port;
    /* Callback to invoke when new message received in corresponding port */
    MsgCallback_t pFunc;
} FportCallbacks_t;

/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/
/*
* \brief Initializes the messaging support
*/
void MSG_Init( void );

/*
* \brief Returns the current state of messaging support
*
* \return State bitfield value (MessageState_t)
*/
uint16_t MSG_GetState( void );

/*
* \brief Requests the MAC layer to send the given data
*
* \param[in] msgType - (TransmissionType_t) confirmed or unconfirmed
* \param[in] port    - frame port number
* \param[in] * data  - pointer to payload data
* \param[in] length  - length of the payload data
* \param[in] timeout - delay to elapse before requesting MAC
*
* \return Status of the request
*           APP_RESULT_OK - if request posted to MAC
*           APP_RESULT_BUSY - if not possible to process the request
*/
AppResult_t MSG_Send( TransmissionType_t msgType, uint8_t port, uint8_t *data,
    uint8_t length, uint32_t timeout );

/*
* \brief Callback definition to process the received data from MAC
*
* \param[in] *appHandle - pointer to appHandle from MAC data callback
* \param[in]  *params   - pointer to appCbParams_t from MAC data callback
*/
void MSG_DataCallback( void *appHandle, appCbParams_t *params );

/*
* \brief Sets the callback to invoke for payload received in corresponding fport
*
* \param[in] port     - fport to map the callback
* \param[in] callback - pointer to callback function for mapping
*/
void MSG_SetFPortRxDataAvailCallback( uint8_t port, MsgCallback_t callback );

/*
* \brief Sets the default callback to invoke for payload received from MAC. If,
*        there is a fport-specific mapping available, then default will not be
*        called in that case.
*
* \param[in] callback - pointer to callback function to be invoked for payload
*/
void MSG_SetDefaultRxDataAvailCallback( MsgCallback_t callback );

/*
* \brief Sets the callback to invoke for transaction complete event
*
* \param[in] callback - pointer to callback function
*/
void MSG_SetTransactionCompleteCallback( MsgCallback_t callback );

/*
* \brief Function to run the messaging support. It shall be called from
*        application to keep this layer running.
*/
void MSG_Process( void );

/*
* \brief Returns if it is possible to transmit now
*
* \param[in] len      - length of the message to transmit
* \param[out] *txTime - time-on-air of the given message
*/
bool MSG_IsTxPossible( uint8_t len, uint32_t *txTime );

#endif // _MSG_H_

/* eof msg.h */
