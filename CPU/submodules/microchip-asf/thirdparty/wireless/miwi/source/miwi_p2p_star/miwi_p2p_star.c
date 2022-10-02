/**
* \file  miwi_p2p_star.c
*
* \brief MiWi P2P & STAR Protocol Implementation
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

/*********************************** HEADERS ******************************************/
#include "system.h"

/* MiWi Application layer configuration file */
#include "miwi_config.h"
/* MiWi Protocol layer configuration file */
#include "miwi_config_p2p.h"

#include "miwi_p2p_star.h"
#include "mimac_at86rf.h"
#include "mimem.h"
#include "delay.h"

#if defined(ENABLE_NETWORK_FREEZER)
#include "pdsDataServer.h"
#include "wlPdsTaskManager.h"
#endif

#include "miqueue.h"
#include "string.h"

/********************************* Macro Definitions *************************************/
#define PROTOCOL_TIMER_INTERVAL   1000
#if defined(PROTOCOL_STAR)
#define DATA_TIMER_INTERVAL       100
#endif
#if defined(ENABLE_SLEEP_FEATURE)
#define RECEIVE_ON_WHEN_IDLE     0x00
#else
#define RECEIVE_ON_WHEN_IDLE     0x01
#endif
#if defined(ENABLE_SECURITY)
#define SECURITY_INFO            0x80
#else
#define SECURITY_INFO            0x00
#endif
/* Maximum number of failure in transmitting link status before reporting failure link */
#define MAX_LINK_STATUS_FAILURES  3
/******************************* Type Definitions  *************************************/
typedef struct _gEstConnection
{
    uint8_t addrLen;
    miwi_status_t status;
    p2pStarState_t backupState;
    /* Connection Retry manipulation Variable */
    uint8_t connectionRetries;
    uint8_t address[LONG_ADDR_LEN];
    connectionConf_callback_t confCallback;
} gEstConnection_t;

typedef struct _resyncConnection
{
    miwi_status_t status;
    /* Resync Retry manipulation Variable */
    uint8_t resyncTimes;
    p2pStarState_t backupState;
    uint32_t channelMap;
    uint8_t connectionIndex;
    resyncConnection_callback_t confCallback;
} resyncConnection_t;

typedef struct _gSearchConnection
{
    uint8_t activeScanResultIndex;
    uint8_t scanDuration;
    uint32_t channelMap;
    SearchConnectionConf_callback_t gSearchConfCallback;
} gSearchConnection_t;

/* Frame Transmit Structures */
typedef struct _TxFrameEntry_t
{
    API_UINT64_UNION frameDstAddr;
    DataConf_callback_t frameConfCallback;
    uint8_t *frame;
    MAC_TRANS_PARAM frameParam;
    uint8_t frameLength;
    uint8_t frameHandle;
} TxFrameEntry_t;

typedef struct _TxFrame_t
{
    TxFrameEntry_t *nextFrame;
    TxFrameEntry_t txFrameEntry;
} TxFrame_t;

/********************* Global and Static Variables *************************************/

/* The PAN Identifier for the device */
API_UINT16_UNION  myPANID;

/* current operating channel for the device */
uint8_t  currentChannel = 0xFF;

/* Holds the current connection mode info of device */
uint8_t  ConnMode = DISABLE_ALL_CONN;

/* Capability Information of the device */
uint8_t  P2PCapacityInfo = RECEIVE_ON_WHEN_IDLE | SECURITY_INFO;

/* Indicates the status of P2P stack */
volatile P2P_STATUS P2PStatus;

/* State of the Mesh stack */
p2pStarState_t p2pStarCurrentState = INITIAL_STATE;

/* Establishment connection info */
gEstConnection_t gEstConnectionInfo;

/* Resync info */
resyncConnection_t resyncInfo;

/* Search connection info */
gSearchConnection_t gSearchConnectionInfo;

MAC_RECEIVED_PACKET MACRxPacket;

/* Long Address of the Device */
uint8_t myLongAddress[MY_ADDRESS_LENGTH] = {EUI_0,EUI_1,EUI_2,EUI_3, EUI_4, EUI_5,EUI_6,EUI_7};

/* Tx Frame Queue Parameters */
MiQueue_t frameTxQueue;
bool txCallbackReceived = true;
TxFrame_t *sentFrame;
static MIWI_TICK lastTxFrameTick;
static uint32_t transaction_duration_us = 0;
#ifdef ENABLE_ED_SCAN
static bool noiseDetectionInProgress = false;
#endif
static SYS_Timer_t protocolTimer;
uint8_t backupChannel = 0xFF;
/* Time interval parameter for active scan */
uint16_t connectionTimeInterval = 0;

defaultParametersRomOrRam_t *miwiDefaultRomOrRamParams;
defaultParametersRamOnly_t *miwiDefaultRamOnlyParams;

uint8_t MyindexinPC;
uint8_t myConnectionIndex_in_PanCo;
uint8_t LatestConnection;

#if defined(PROTOCOL_STAR)
 /* Role of Device in Star Network */
DeviceRole_t role;
/* Used by END_DEVICES to store total no of end_devices in network */
uint8_t end_nodes = 0;
END_DEVICES_Unique_Short_Address  END_DEVICES_Short_Address[CONNECTION_SIZE];
LinkFailureCallback_t linkFailureCallback;
#if defined(ENABLE_PERIODIC_CONNECTIONTABLE_SHARE)
/* Time interval parameter for broadcasting dev info */
uint16_t sharePeerDevInfoTimeInterval = 0;
#endif
#if defined(ENABLE_LINK_STATUS)
uint16_t inActiveDeviceCheckTimeInterval = 0;
uint16_t linkStatusTimeInterval = 0;
uint8_t linkStatusFailureCount = 0;
#endif
#endif
#ifdef ENABLE_SLEEP_FEATURE
SYS_Timer_t rfdDataWaitTimer;
#endif
#if defined(ENABLE_NETWORK_FREEZER)
ReconnectionCallback_t reconnectionCallback;
#endif
/* structure to store information for the received packet */
RECEIVED_MESSAGE  rxMessage;
/* total no of devices-In star used only by PAN COR to calculate no of END Devices */
uint8_t conn_size = 0;

#if defined(PROTOCOL_STAR)
/* Queue to store frame requiring application level ack from destination */
MiQueue_t appAckWaitDataQueue;
#endif
#if defined(ENABLE_ED_SCAN)
/* Time interval parameter for active scan */
uint16_t edScanDurationTimeInterval = 0;
#endif
#if defined(ENABLE_ACTIVE_SCAN)
/* Time interval parameter for active scan */
uint16_t activeScanDurationTimeInterval = 0;
#endif
/* Queue to store frame with MAC level ack from destination */
MiQueue_t macAckOnlyFrameQueue;

/* Queue to store frame destined for sleeping devices */
MiQueue_t indirectFrameQueue;
#ifdef ENABLE_SLEEP_FEATURE
uint16_t dataRequestInterval = 0;
#endif
#ifdef ENABLE_FREQUENCY_AGILITY
/* Optimial Channel Choosen for Channel Hopping */
uint8_t optimalChannel = 0xFF;
uint16_t freqAgilityBroadcastInterval = 0;
uint8_t freqAgilityRetries = 0;
bool channelChangeInProgress = false;
#endif
PacketIndCallback_t pktRxcallback = NULL;
#if defined(ENABLE_SECURITY)
API_UINT32_UNION IncomingFrameCounter[CONNECTION_SIZE];  // If authentication is used, IncomingFrameCounter can prevent replay attack
#endif
/************************************** Function Prototypes****************************************************/
bool frameTransmit(INPUT bool Broadcast,API_UINT16_UNION DestinationPANID,INPUT uint8_t *DestinationAddress,INPUT bool isCommand,INPUT bool SecurityEnabled,
                   INPUT uint8_t msgLen, INPUT uint8_t* msgPtr, INPUT uint8_t msghandle, INPUT bool ackReq, INPUT DataConf_callback_t ConfCallback);
static void CommandConfCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer);
static void frameTxCallback(uint8_t handle, miwi_status_t status, uint8_t* msgPointer);
static void frameParse(MAC_RECEIVED_PACKET *macRxPacket);
#ifndef ENABLE_SLEEP_FEATURE
static void connectionRespConfCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer);
#endif
static void connReqConfCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer);
uint8_t AddConnection(uint8_t capacityInfo);
static miwi_status_t sendConnectionRequest(void);
static void protocolTimerInit(void);
static void protocolTimerHandler(SYS_Timer_t *timer);
#ifdef ENABLE_ACTIVE_SCAN
static uint8_t ScanChannel(void);
#endif
#if defined(ENABLE_ED_SCAN) || defined(ENABLE_ACTIVE_SCAN) || defined(ENABLE_FREQUENCY_AGILITY)
static inline  uint32_t miwi_scan_duration_ticks(uint8_t scan_duration);
#endif
static void removeConnection(uint8_t index);

#if defined(PROTOCOL_STAR)
static SYS_Timer_t dataTimer;
static void dataTimerHandler(SYS_Timer_t *timer);
static void store_connection_tb(uint8_t *payload);
static uint8_t Find_Index (uint8_t *DestAddr);
static void startCompleteProcedure(bool timeronly);
static void startLinkStatusTimer(void);
static void sendLinkStatus(void);
static void findInActiveDevices(void);
static void handleLostConnection(void);
void appAckWaitDataCallback(uint8_t handle, miwi_status_t status, uint8_t* msgPointer);
static void MiApp_BroadcastConnectionTable(void);
#endif
#ifdef ENABLE_SLEEP_FEATURE
static void sendDataRequest(void);
static void dataRequestConfCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer);
static void rfdDataWaitTimerExpired(struct SYS_Timer_t *timer);
#endif
void macAckOnlyDataCallback(uint8_t handle, miwi_status_t status, uint8_t* msgPointer);
#ifdef ENABLE_FREQUENCY_AGILITY
static void StartChannelHopping(void);
static void channelHopCmdCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer);
#endif
/********************* Function Definitions *******************************************/
miwi_status_t MiApp_ProtocolInit(defaultParametersRomOrRam_t *defaultRomOrRamParams,
                                       defaultParametersRamOnly_t *defaultRamOnlyParams)
{
    uint8_t i = 0;
    uint16_t broadcastAddress = 0xFFFF;
    miwi_status_t initStatus = SUCCESS;

    MACINIT_PARAM initValue;

    /* Clear all status bits */
    P2PStatus.Val = 0;//TODO

    /* Store the ROmRAM parameter references */
    miwiDefaultRomOrRamParams = defaultRomOrRamParams;
    miwiDefaultRamOnlyParams = defaultRamOnlyParams;

    /* Initialize PANID */
    myPANID.Val = 0;
#if defined(ENABLE_NETWORK_FREEZER)
    /* Restore Network Information Base from Non Volatile Memory
       - If it fails, initialize the PDS Items */
    if (defaultRomOrRamParams->networkFreezerRestore)
    {
        if (!PDS_IsAbleToRestore(MIWI_ALL_MEMORY_MEM_ID) || !PDS_Restore(MIWI_ALL_MEMORY_MEM_ID))
        {
            PDS_InitItems();
        }
    }
    /* If Restored properly, valid PanId will be available */
    if (myPANID.Val != 0x0000 && myPANID.Val != 0xFFFF)
    {
        p2pStarCurrentState = IN_NETWORK_STATE;
        initStatus = RECONNECTED;

        /* Inform application reconnection success */
        if (NULL != reconnectionCallback)
            reconnectionCallback(SUCCESS);
    }
    else
#endif
    {
        /* Stack is initialized, change state to INIT_STATE */
        p2pStarCurrentState = INIT_STATE;

        /* Initialize PANID */
        myPANID.Val = MY_PAN_ID;

#if defined(ENABLE_NETWORK_FREEZER)
    for(i = 0; i < CONNECTION_SIZE; i++)
    {
        defaultRomOrRamParams->ConnectionTable[i].status.Val = 0;
        memset(&defaultRomOrRamParams->ConnectionTable[i].Address, 0, LONG_ADDR_LEN);
    }
#endif

#if defined(ENABLE_SECURITY)
    for(i = 0; i < CONNECTION_SIZE; i++)
    {
        IncomingFrameCounter[i].Val = 0;
    }
#endif
    }
    initValue.PAddress = myLongAddress;
    initValue.actionFlags.bits.CCAEnable = 1;
    initValue.actionFlags.bits.PAddrLength = MY_ADDRESS_LENGTH;
#if defined(ENABLE_NETWORK_FREEZER)
    initValue.actionFlags.bits.NetworkFreezer = 1;
#else
    initValue.actionFlags.bits.NetworkFreezer = 0;
#endif
    initValue.actionFlags.bits.RepeaterMode = 0;

    MiMAC_Init(initValue);

    if (currentChannel != 0xFF)
        MiApp_Set(CHANNEL, &currentChannel);

    MiMAC_SetAltAddress((uint8_t *)&broadcastAddress, (uint8_t *)&myPANID.Val);

    P2PCapacityInfo |= (ConnMode << 4);

    /* Initialize Protocol Timer */
    protocolTimerInit();
	
#if defined(PROTOCOL_STAR)
	miQueueInit(&appAckWaitDataQueue);
#endif
	miQueueInit(&macAckOnlyFrameQueue);
	miQueueInit(&indirectFrameQueue);

    if (IN_NETWORK_STATE == p2pStarCurrentState)
    {
#if defined(PROTOCOL_STAR)
        if (PAN_COORD == role)
        {
            /* Procedures for PAN Coordinator*/
            startCompleteProcedure(true);
        }
        else
        {
            /* start Link Status Timer */
            startLinkStatusTimer();
#ifdef ENABLE_SLEEP_FEATURE
            /* Start data request timer upon network freezer restore */
            dataRequestInterval = RFD_WAKEUP_INTERVAL;
#endif
        }
#else
#ifdef ENABLE_SLEEP_FEATURE
        /* Start data request timer upon network freezer restore */
        dataRequestInterval = RFD_WAKEUP_INTERVAL;
#endif
#endif
    }
    (void)i;
    return initStatus;
}

bool MiApp_Get(miwi_params_t id, uint8_t *value)
{
    switch(id)
    {
        case CHANNEL:
        {
            *value = currentChannel;
            return true;
        }
        break;

        case PANID:
        {
            value[0] = myPANID.Val;
            value[1] = myPANID.Val >> 8;
            return true;
        }
        break;

        default:
        break;
    }
    return false;
}
bool MiApp_Set(miwi_params_t id, uint8_t *value)
{
    switch(id)
    {
        case CHANNEL:
        {
            if( MiMAC_Set(MAC_CHANNEL, value))
            {
                currentChannel = *value;
#if defined(ENABLE_NETWORK_FREEZER)
                PDS_Store(PDS_CURRENT_CHANNEL_ID);
#endif
                return true;
            }
        }
        break;

        default:
        break;
    }
    return false;
}

/************************************************************************************
 * Function:
 *      void    MiApp_ConnectionMode(uint8_t Mode)
 *
 * Summary:
 *      This function set the current connection mode.
 *
 * Description:
 *      This is the primary user interface function for the application layer to
 *      configure the way that the host device accept connection request.
 *
 * PreCondition:
 *      Protocol initialization has been done.
 *
 * Parameters:
 *      uint8_t Mode -     The mode to accept connection request. The privilege for those modes
 *                      decreases gradually as defined. The higher privilege mode has all the
 *                      rights of the lower privilege modes.
 *                      The possible modes are
 *                      * ENABLE_ALL_CONN       Enable response to all connection request
 *                      * ENABLE_PREV_CONN      Enable response to connection request
 *                                              from device already in the connection
 *                                              table.
 *                      * ENABLE_ACTIVE_SCAN_RSP    Enable response to active scan only
 *                      * DISABLE_ALL_CONN      Disable response to connection request, including
 *                                              an acitve scan request.
 *
 * Returns:
 *      None
 *
 * Example:
 *      <code>
 *      // Enable all connection request
 *      MiApp_ConnectionMode(ENABLE_ALL_CONN);
 *      </code>
 *
 * Remarks:
 *      None
 *
 *****************************************************************************************/
void MiApp_ConnectionMode(INPUT uint8_t Mode)
{
    if( Mode > 3 )
    {
        return;
    }
    ConnMode = Mode;
    P2PCapacityInfo = (P2PCapacityInfo & 0x0F) | (ConnMode << 4);

#if defined(ENABLE_NETWORK_FREEZER)
        PDS_Store(PDS_CONNECTION_MODE_ID);
#endif
}

#ifdef ENABLE_ACTIVE_SCAN
static void sendScanConfirm(void)
{
    /* Revert back working channel */
    currentChannel = backupChannel;
    if(currentChannel!= 0xFF)
    {
    MiApp_Set(CHANNEL, &backupChannel);
    }

    if (p2pStarCurrentState == SEARCHING_NETWORK)
    {
        /* Change state while completing search */
        p2pStarCurrentState = SEARCH_COMPLETE;

        /* All channels were scanned. Send confirmation to the upper layer. */
        gSearchConnectionInfo.gSearchConfCallback(gSearchConnectionInfo.activeScanResultIndex, (uint8_t*)miwiDefaultRomOrRamParams->ActiveScanResults);
        gSearchConnectionInfo.gSearchConfCallback = NULL;
    }
    else if (p2pStarCurrentState == RESYNC_IN_PROGRESS)
    {
        if (resyncInfo.resyncTimes > 0)
        {
            --resyncInfo.resyncTimes;
            gSearchConnectionInfo.channelMap = resyncInfo.channelMap;
            ScanChannel();
        }
        else
        {
            /* Change state while completing search */
            p2pStarCurrentState = resyncInfo.backupState;

            resyncInfo.confCallback(backupChannel, FAILURE);
            resyncInfo.confCallback = NULL;
        }
    }
}
/************************************************************************************
* Function:
*      static void scanDurationExpired(void)
*
* Summary:
*      This callback function is called when scan timer is expired, so
*      it tries to scan the next channel if available
*
*****************************************************************************************/
static void scanDurationExpired(void)
{
    uint8_t status;

    /* Remove the scanned channel from unscanned channels list */
    gSearchConnectionInfo.channelMap &= ~(1UL << currentChannel);

    status = ScanChannel();
    if ( status != SUCCESS)
    {
        /* Send Scan Confirmation to upper layer */
        sendScanConfirm();
    }
}

/************************************************************************************
* Function:
*      static void ActiveScanReqConfcb(uint8_t handle, miwi_status_t status, uint8_t* msgPointer)
*
* Summary:
*      This callback function starts the search timer if beacon is sent successfully
*
* Parameters/Returns:
*      uint8_t handle, miwi_status_t status, uint8_t* msgPointer
*****************************************************************************************/
static void ActiveScanReqConfcb(uint8_t handle, miwi_status_t confstatus, uint8_t* msgPointer)
{
    MiMem_Free(msgPointer);

    if (SUCCESS == confstatus)
    {
        /* Start the timer for scan Duration time */
        activeScanDurationTimeInterval =  (miwi_scan_duration_ticks(gSearchConnectionInfo.scanDuration)/1000000) + 1;
    }
    else
    {
        /* Remove the scanned channel from unscanned channels list */
        gSearchConnectionInfo.channelMap &= ~(1UL << currentChannel);

        /* Beacon request is not sent successfully , then try again */
        uint8_t status = ScanChannel();
        if ( status != SUCCESS)
        {
            /* Send Scan Confirmation to upper layer */
            sendScanConfirm();
        }
    }
}

/************************************************************************************
* Function:
*      static uint8_t initiateBeaconReq(void)
*
* Summary:
*      This function sends the beacon request
*
* Parameters/Returns:
*      status of the request
*****************************************************************************************/
static uint8_t initiateActiveScanReq(void)
{
    uint8_t dataLen = 0;
    uint8_t *dataPtr = NULL;
    API_UINT16_UNION broadcastPANID;

    dataPtr = MiMem_Alloc(PACKETLEN_P2P_ACTIVE_SCAN_REQUEST);
    if (NULL == dataPtr)
        return MEMORY_UNAVAILABLE;

    /* Construct the P2P Active Scan Request */
    dataPtr[dataLen++] = CMD_P2P_ACTIVE_SCAN_REQUEST;
    dataPtr[dataLen++] = currentChannel;
    broadcastPANID.Val = 0xFFFF;

    /* Initiate the frame transmission */
    if (SEARCHING_NETWORK == p2pStarCurrentState)
    {
        frameTransmit(true, broadcastPANID, NULL, true, false, dataLen, dataPtr,0, true, ActiveScanReqConfcb);
    }
    else if (RESYNC_IN_PROGRESS == p2pStarCurrentState)
    {
        frameTransmit(false, myPANID, miwiDefaultRomOrRamParams->ConnectionTable[resyncInfo.connectionIndex].Address, 
                                      true, false, dataLen, dataPtr,0, true, ActiveScanReqConfcb);
    }
    else
    {
        return FAILURE;
    }

    return SUCCESS;
}

/************************************************************************************
* Function:
*      static uint8_t ScanChannel(searchConf_t* searchConfirm)
*
* Summary:
*      This function does the channel scanning by initiating beacon req
*
* Parameters:
*      searchConf_t* searchConfirm - confirm of search
* Returns:
*      uint8_t - status of operation
*****************************************************************************************/
static uint8_t ScanChannel(void)
{
    bool channelSetStatus = false;

    uint8_t curChannel = 0;
    /* Find the next unscanned channel to scan and initiate the scan, */
    for (curChannel = currentChannel; curChannel <= 26; curChannel++)
    {
        if ((gSearchConnectionInfo.channelMap & (1UL << curChannel)) != 0)
        {
            /* Set the selected channel */
            currentChannel = curChannel;
            channelSetStatus = MiApp_Set(CHANNEL, &currentChannel);

            if (channelSetStatus)
            {
                return (initiateActiveScanReq());
            }
        }
    }

    /* Send Scan Confirmation to upper layer */
    sendScanConfirm();

    return SUCCESS;
}


/************************************************************************************
* Function:
*      uint8_t    MiApp_SearchConnection(uint8_t ScanDuartion, uint32_t ChannelMap,
*                                             SearchConnectionConf_callback_t ConfCallback)
*
* Summary:
*      This function perform an active scan to locate operating PANs in the
*      neighborhood.
*
* Description:
*      This is the primary user interface function for the application layer to
*      perform an active scan. After this function call, all active scan response
*      will be stored in the global variable ActiveScanResults in the format of
*      structure ACTIVE_SCAN_RESULT. The return value indicates the total number
*      of valid active scan response in the active scan result array.
*
* PreCondition:
*      Protocol initialization has been done.
*
* Parameters:
*      uint8_t ScanDuration - The maximum time to perform scan on single channel. The
*                          value is from 5 to 14. The real time to perform scan can
*                          be calculated in following formula from IEEE 802.15.4
*                          specification
*                              960 * (2^ScanDuration + 1) * 10^(-6) second
*      uint32_t ChannelMap -  The bit map of channels to perform noise scan. The 32-bit
*                          double word parameter use one bit to represent corresponding
*                          channels from 0 to 31. For instance, 0x00000003 represent to
*                          scan channel 0 and channel 1.
*      SearchConnectionConf_callback_t ConfCallback - The callback routine which will be called upon
*                                               the initiated connection procedure is performed
*
* Returns:
*      The number of valid active scan response stored in the global variable ActiveScanResults.
*
* Example:
*      <code>
*      // Perform an active scan on all possible channels
*      NumOfActiveScanResponse = MiApp_SearchConnection(10, 0xFFFFFFFF, callback);
*      </code>
*
* Remarks:
*      None
*
*****************************************************************************************/
uint8_t MiApp_SearchConnection(INPUT uint8_t ScanDuration, INPUT uint32_t ChannelMap,
                 SearchConnectionConf_callback_t ConfCallback)
{
    uint32_t supportedChannelMap = 0;
    uint32_t index = 1;

    /* Check the given scan duration is within the range and callback is not NULL*/
    if ((ScanDuration > MAX_SCAN_DURATION) || (NULL == ConfCallback))
        return FAILURE;

    /* If state is not Initial state or search complete , don't process further */
    if ((p2pStarCurrentState == INITIAL_STATE))
    {
        return FAILURE;
    }

    /* Change state while processing start connection */
    p2pStarCurrentState = SEARCHING_NETWORK;

    /* Store scan duration */
    gSearchConnectionInfo.scanDuration = ScanDuration;

    /* Store scan channels */
    gSearchConnectionInfo.channelMap = ChannelMap;

    /* Allocate memory for scan and confirmation */
    gSearchConnectionInfo.gSearchConfCallback = ConfCallback;

    /* Initialize the scan confirm parameters */
    for(uint8_t i = 0; i < ACTIVE_SCAN_RESULT_SIZE; i++)
    {
            miwiDefaultRomOrRamParams->ActiveScanResults[i].Channel = 0xFF;
    }
    gSearchConnectionInfo.activeScanResultIndex = 0;

    /* Get the supported channel and find the first channel to start scan */
    supportedChannelMap = MiMAC_GetPHYChannelInfo();

    /* Backup currentChannel */
    backupChannel = currentChannel;

    currentChannel = 0;

    while (!(index & supportedChannelMap))
    {
        // Unset current bit and set the next bit in 'i'
        index = index << 1;

        // increment position
        ++currentChannel;
    }

    /* Start the scan procedure */
    return (ScanChannel());
}
#endif

#if defined(ENABLE_ED_SCAN) || defined(ENABLE_ACTIVE_SCAN) || defined(ENABLE_FREQUENCY_AGILITY)
/* Scan Duration formula for P2P Connection: 60 * 16 * (2 ^ n + 1) symbols */
static inline  uint32_t miwi_scan_duration_ticks(uint8_t scan_duration)
{
    uint32_t scan_symbols;
    scan_symbols =   ABASESUPERFRAMEDURATION *((1<<scan_duration) + 1);
    return SYMBOLS_TO_TICKS(scan_symbols);
}
#endif

#ifdef ENABLE_ED_SCAN
/************************************************************************************
* Function:
*      static void edScanDurationExpired(void)
*
* Summary:
*      This callback function is called when scan timer is expired, so
*      it tries to scan the next channel if available
*
*****************************************************************************************/
static void edScanDurationExpired(void)
{
    noiseDetectionInProgress = false;
}

uint8_t MiApp_NoiseDetection(uint32_t ChannelMap, uint8_t ScanDuration, uint8_t DetectionMode, OUTPUT uint8_t *NoiseLevel)
{
    uint8_t i;
    uint8_t OptimalChannel = 0xFF;
    uint8_t minRSSI = 0xFF;
    uint32_t channelMask = 0x00000001;

    noiseDetectionInProgress = false;
    if( DetectionMode != NOISE_DETECT_ENERGY )
    {
        return 0xFF;
    }
    i = 0;
    while( i < 32 )
    {
        if( ChannelMap & MiMAC_GetPHYChannelInfo() & (channelMask << i) )
        {
            uint8_t RSSIcheck;
            uint8_t maxRSSI = 0;

            /* choose appropriate channel */
            MiApp_Set(CHANNEL, &i);

            /* Start the timer for scan Duration time */
            edScanDurationTimeInterval = (miwi_scan_duration_ticks(ScanDuration)/1000000) + 1;

		    noiseDetectionInProgress = true;

            while(1)
            {
                RSSIcheck = MiMAC_ChannelAssessment(CHANNEL_ASSESSMENT_ENERGY_DETECT);
                if( RSSIcheck > maxRSSI )
                {
                    maxRSSI = RSSIcheck;
                }

                if( false == noiseDetectionInProgress)
                {
                    // if scan time exceed scan duration, prepare to scan the next channel
                    break;
                }
                SYS_TimerTaskHandler();
            }

            if( maxRSSI < minRSSI )
            {
                minRSSI = maxRSSI;
                OptimalChannel = i;
                if( NoiseLevel )
                {
                    *NoiseLevel = minRSSI;
                }
            }
        }
        i++;
    }

    return OptimalChannel;
}
#endif

static void startCompleteProcedure(bool timeronly)
{
    if (false == timeronly)
    {
#if defined (PROTOCOL_STAR)
        /* Set Role to PANC */
        role = PAN_COORD;
#endif

        /* Set the state to in network state*/
        p2pStarCurrentState = IN_NETWORK_STATE;

#if defined(ENABLE_NETWORK_FREEZER)
        /*Store Network Information in Persistent Data Server */
        PDS_Store(MIWI_ALL_MEMORY_MEM_ID);
#endif
    }

#if defined (PROTOCOL_STAR)
#if defined(ENABLE_LINK_STATUS)
    /* Start the timer for Finding in active devices and initiating remove connection
    if found any */
    inActiveDeviceCheckTimeInterval = FIND_INACTIVE_DEVICE_TIMEOUT;
#endif

#if defined(ENABLE_PERIODIC_CONNECTIONTABLE_SHARE)
    /* Start the timer for sharing the connection table periodically */
    sharePeerDevInfoTimeInterval = SHARE_PEER_DEVICE_INFO_TIMEOUT;
#endif
#endif

}
/************************************************************************************************
    * Function:
    *      bool    MiApp_StartConnection(uint8_t Mode, uint8_t ScanDuration, uint32_t ChannelMap,
    *                                                    connectionConf_callback_t ConfCallback)
    *
    * Summary:
    *      This function start a PAN without connected to any other devices
    *
    * Description:
    *      This is the primary user interface function for the application layer to
    *      a PAN. Usually, this function is called by the PAN Coordinator who is the
    *      first in the PAN. The PAN Coordinator may start the PAN after a noise scan
    *      if specified in the input mode.
    *
    * PreCondition:
    *      Protocol initialization has been done.
    *
    * Parameters:
    *      uint8_t Mode - Whether to start a PAN after a noise scan. Possible modes are
    *                  * START_CONN_DIRECT Start PAN directly without noise scan
    *                  * START_CONN_ENERGY_SCN Perform an energy scan first, then
    *                                          start the PAN on the channel with least
    *                                          noise.
    *                  * START_CONN_CS_SCN     Perform a carrier-sense scan first,
    *                                          then start the PAN on the channel with
    *                                          least noise.
    *      uint8_t ScanDuration - The maximum time to perform scan on single channel. The
    *                          value is from 5 to 14. The real time to perform scan can
    *                          be calculated in following formula from IEEE 802.15.4
    *                          specification:
    *                              960 * (2^ScanDuration + 1) * 10^(-6) second
    *                          ScanDuration is discarded if the connection mode is
    *                          START_CONN_DIRECT.
    *      uint32_t ChannelMap -  The bit map of channels to perform noise scan. The 32-bit
    *                          double word parameter use one bit to represent corresponding
    *                          channels from 0 to 31. For instance, 0x00000003 represent to
    *                          scan channel 0 and channel 1. ChannelMap is discarded if the
    *                          connection mode is START_CONN_DIRECT.
    *
    *      connectionConf_callback_t ConfCallback - The callback routine which will be called upon
    *                                               the initiated connection procedure is performed
    * Returns:
    *      a boolean to indicate if PAN has been started successfully.
    *
    * Example:
    *      <code>
    *      // start the PAN on the least noisy channel after scanning all possible channels.
    *      MiApp_StartConnection(START_CONN_ENERGY_SCN, 10, 0xFFFFFFFF, callback);
    *      </code>
    *
    * Remarks:
    *      None
    *
*************************************************************************************************/
bool MiApp_StartConnection(uint8_t Mode, uint8_t ScanDuration, uint32_t ChannelMap,
                                            connectionConf_callback_t ConfCallback)
{
    /* If state is not correct state , don't process further */
    if ((p2pStarCurrentState != INIT_STATE) && (p2pStarCurrentState != SEARCH_COMPLETE))
        return false;

    /* Change state while processing start connection */
    p2pStarCurrentState = STARTING_NETWORK;

    switch(Mode)
    {
        case START_CONN_DIRECT:
        {
            uint8_t channel = 0;
            uint32_t index = 1;
            uint16_t tmp = 0xFFFF;

#if MY_PAN_ID == 0xFFFF
            myPANID.Val = PHY_RandomReq();
#else
            myPANID.Val = MY_PAN_ID;
#endif

            MiMAC_SetAltAddress((uint8_t *)&tmp, (uint8_t *)&myPANID.Val);

            while (!(index & ChannelMap))
            {
                // Unset current bit and set the next bit in 'i'
                index = index << 1;

                // increment position
                ++channel;
            }

            /* Set the best channel */
            MiApp_Set(CHANNEL, &channel);

            /* Procedures if start operation is success */
            startCompleteProcedure(false);

            /* Send Confirmation to caller */
            ConfCallback(SUCCESS);

            return true;
        }
        break;

        case START_CONN_ENERGY_SCN:
#if defined(ENABLE_ED_SCAN)
        {
            uint8_t channel;
            uint8_t RSSIValue;
            uint16_t tmp = 0xFFFF;

#if MY_PAN_ID == 0xFFFF
            myPANID.Val = PHY_RandomReq();
#else
            myPANID.Val = MY_PAN_ID;
#endif
            /* Set Short Address and PAN ID */
            MiMAC_SetAltAddress((uint8_t *)&tmp, (uint8_t *)&myPANID.Val);

            /* Identify the best channel by doing energy detection scan */
            channel = MiApp_NoiseDetection(ChannelMap, ScanDuration, NOISE_DETECT_ENERGY, &RSSIValue);

            /* Set the best channel */
            MiApp_Set(CHANNEL, &channel);

            /* Procedures if start operation is success */
            startCompleteProcedure(false);

            /* Send Confirmation to caller */
            ConfCallback(SUCCESS);

            return true;
        }
#else
        /* Send Confirmation to caller */
        ConfCallback(FAILURE);
        return false;
#endif

        case START_CONN_CS_SCN:
        {
            /* Carrier sense scan is not supported for current available transceivers */
            ConfCallback(FAILURE);

            /* Set the state to Initial since failure to start network */
            p2pStarCurrentState = INIT_STATE;

            return false;
        }
        break;

        default:
        break;
    }
    ConfCallback(FAILURE);
    /* Set the state to Initial since failure to start network */
    p2pStarCurrentState = INIT_STATE;
    return false;
}

/************************************************************************************
* Function:
*      static void connReqConfCallback(uint8_t handle, miwi_status_t status, uint8_t* msgPointer)
*
* Summary:
*      This callback function frees the memory used for the command sent
*      and starts connection response timer if packet sent successfully otherwise it post confirm
*
* Parameters/Returns:
*      uint8_t handle, miwi_status_t status, uint8_t* msgPointer
*****************************************************************************************/
static void connReqConfCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
    MiMem_Free(msgPointer);
    --gEstConnectionInfo.connectionRetries;
    if (SUCCESS == status)
    {
        /* Start the timer to wait for connection response */
        connectionTimeInterval = CONNECTION_INTERVAL;
#ifdef ENABLE_SLEEP_FEATURE
        rfdDataWaitTimer.handler = rfdDataWaitTimerExpired;
        rfdDataWaitTimer.timeout = RFD_DATA_WAIT / 1000;
        rfdDataWaitTimer.interval = RFD_DATA_WAIT / 1000;
        rfdDataWaitTimer.mode = SYS_TIMER_INTERVAL_MODE;
#endif
    }
    else
    {
        sendConnectionRequest();
    }
}

/************************************************************************************
* Function:
*      miwi_status_t sendConnectionRequest(void)
*
* Summary:
*      This function sends a connection request
*
* Parameters/Returns:
*      miwi_status_t - the status of the send operation
*****************************************************************************************/
static miwi_status_t sendConnectionRequest(void)
{
    if ((gEstConnectionInfo.connectionRetries > 0) &&  (ESTABLISHING_NETWORK ==  p2pStarCurrentState))
    {
        uint8_t* dataPtr = NULL;
        uint8_t dataLen = 0;

        dataPtr = MiMem_Alloc(PACKETLEN_P2P_CONNECTION_REQUEST);

        if (NULL == dataPtr)
          return MEMORY_UNAVAILABLE;

        /* Construct the full frame with command id and payload */
        dataPtr[dataLen++] = CMD_P2P_CONNECTION_REQUEST;
        dataPtr[dataLen++] = currentChannel;
        dataPtr[dataLen++] = P2PCapacityInfo;

#if defined(PROTOCOL_STAR) && defined(MAKE_ENDDEVICE_PERMANENT)
        dataPtr[dataLen++] = 0xAA;
#endif

#if ADDITIONAL_NODE_ID_SIZE > 0
        for(uint8_t i = 0; i < ADDITIONAL_NODE_ID_SIZE; i++)
        {
            dataPtr[dataLen++] = miwiDefaultRomOrRamParams->AdditionalNodeID[i];
        }
#endif

#if defined(ENABLE_ACTIVE_SCAN)
        uint16_t DestinationAddress16 = ((gEstConnectionInfo.address[1] << 8) + gEstConnectionInfo.address[0]);
        if( DestinationAddress16 == 0xFFFF )
        {
            if(frameTransmit(true, myPANID, NULL, true, false, dataLen, dataPtr,0, true, connReqConfCallback))
                return SUCCESS;
            else
                return MEMORY_UNAVAILABLE;
        }
        else
        {
            uint8_t i,j;
            bool deviceFound = false;
            for(i = 0; i < ACTIVE_SCAN_RESULT_SIZE; i++)
            {
                deviceFound = true;
                for(j = 0; j < MY_ADDRESS_LENGTH; j++)
                {
                    if (gEstConnectionInfo.address[j] != miwiDefaultRomOrRamParams->ActiveScanResults[i].Address[j])
                    {
                        deviceFound = false;
                        break;
                    }
                }
                if (deviceFound)
                {
                    break;
                }
            }
            if (deviceFound)
            {
                if (frameTransmit(false, miwiDefaultRomOrRamParams->ActiveScanResults[i].PANID, miwiDefaultRomOrRamParams->ActiveScanResults[i].Address, true, false,
                dataLen, dataPtr,0, true, connReqConfCallback))
                    return SUCCESS;
                else
                    return MEMORY_UNAVAILABLE;
            }
            else
            {
                /* Free the allocated memory */
                MiMem_Free(dataPtr);

                /* Change back state  */
                p2pStarCurrentState = gEstConnectionInfo.backupState;

                /* Post Confirmation since address not found in active scan results list */
                gEstConnectionInfo.confCallback(ADDR_NOT_FOUND_IN_SCANNED_LIST);
                gEstConnectionInfo.confCallback = NULL;
                return ADDR_NOT_FOUND_IN_SCANNED_LIST;
            }
        }
#else
        if(frameTransmit(true, myPANID, NULL, true, false, dataLen, dataPtr,0, true, connReqConfCallback))
            return SUCCESS;
        else
            return MEMORY_UNAVAILABLE;
#endif
    }
    else
    {
        if ((gEstConnectionInfo.status == ALREADY_EXISTS) || (gEstConnectionInfo.status == SUCCESS))
        {
            /* Set the state to in network state*/
            p2pStarCurrentState = IN_NETWORK_STATE;
            /* Post Confirmation */
            gEstConnectionInfo.confCallback(gEstConnectionInfo.status);
        }
        else
        {
            /* Change back state  */
            p2pStarCurrentState = gEstConnectionInfo.backupState;
            /* Post Confirmation since no connection response received */
            gEstConnectionInfo.confCallback(FAILURE);
        }
        gEstConnectionInfo.confCallback = NULL;
        return TIMER_EXPIRED;
    }
}

uint8_t MiApp_EstablishConnection(uint8_t Channel, uint8_t addr_len, uint8_t *addr, uint8_t Capability_info,
                                            connectionConf_callback_t ConfCallback)
{
    /* If state is initial state , don't process further */
    if (p2pStarCurrentState == INITIAL_STATE)
        return 0;

    ConnMode = ENABLE_ALL_CONN;

    /* Store the Establish connection Informations */
    gEstConnectionInfo.confCallback = ConfCallback;
    gEstConnectionInfo.addrLen = addr_len;
    memcpy(gEstConnectionInfo.address, addr, addr_len);
    gEstConnectionInfo.backupState = p2pStarCurrentState;
	gEstConnectionInfo.status = SCAN_NO_BEACON;

    /* Assign connection retries */
    gEstConnectionInfo.connectionRetries = CONNECTION_RETRY_TIMES;

    /* Change state while processing establish connection */
    p2pStarCurrentState = ESTABLISHING_NETWORK;

    MiApp_Set(CHANNEL, &Channel);

    return sendConnectionRequest();
}

#if !defined(TARGET_SMALL)
static void removeConnection(uint8_t index)
{
    uint8_t* dataPtr = NULL;
    uint8_t dataLen = 0;

    dataPtr = MiMem_Alloc(PACKETLEN_P2P_CONNECTION_REMOVAL_REQUEST);

    if (NULL == dataPtr)
        return;

    dataPtr[dataLen++] = CMD_P2P_CONNECTION_REMOVAL_REQUEST;

    frameTransmit(false, myPANID, miwiDefaultRomOrRamParams->ConnectionTable[index].Address, true, false, dataLen, dataPtr,0, true, CommandConfCallback);
}
/*********************************************************************
* Function:
*      void MiApp_RemoveConnection(uint8_t ConnectionIndex)
*
* Summary:
*      This function remove connection(s) in connection table
*
* Description:
*      This is the primary user interface function to disconnect connection(s).
*      For a P2P protocol, it simply remove the connection. For a network protocol,
*      if the device referred by the input parameter is the parent of the device
*      calling this function, the calling device will get out of network along with
*      its children. If the device referred by the input parameter is children of
*      the device calling this function, the target device will get out of network.
*
* PreCondition:
*      Transceiver has been initialized. Node has establish
*      one or more connections
*
* Parameters:
*      uint8_t ConnectionIndex -  The index of the connection in the
*                              connection table to be removed
*
* Returns:
*      None
*
* Example:
*      <code>
*      MiApp_RemoveConnection(0x00);
*      </code>
*
* Remarks:
*      None
*
********************************************************************/
void MiApp_RemoveConnection(INPUT uint8_t ConnectionIndex)
{
    if( ConnectionIndex == 0xFF )
    {
        uint8_t i;
        for(i = 0; i < CONNECTION_SIZE; i++)
        {
            if( miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.isValid )
            {
                removeConnection(i);
            }
            miwiDefaultRomOrRamParams->ConnectionTable[i].status.Val = 0;
#if defined(ENABLE_NETWORK_FREEZER)
            PDS_Store(PDS_CONNECTION_TABLE_ID);
#endif
        }
    }
    else if( miwiDefaultRomOrRamParams->ConnectionTable[ConnectionIndex].status.bits.isValid )
    {
        removeConnection(ConnectionIndex);

        miwiDefaultRomOrRamParams->ConnectionTable[ConnectionIndex].status.Val = 0;

#if defined(ENABLE_NETWORK_FREEZER)
       PDS_Store(PDS_CONNECTION_TABLE_ID);
#endif
    }
}
#endif


bool MiApp_SubscribeDataIndicationCallback(PacketIndCallback_t callback)
{
    if (NULL != callback)
    {
	    pktRxcallback = callback;
	    return true;
    }
    return false;
}

#ifdef ENABLE_ACTIVE_SCAN
bool MiApp_ResyncConnection(INPUT uint8_t ConnectionIndex, INPUT uint32_t ChannelMap, resyncConnection_callback_t callback)
{
    /* Backup the current channel */
    backupChannel = currentChannel;

    /* Store the resync connection Informations */
    resyncInfo.confCallback = callback;
    resyncInfo.channelMap = ChannelMap;
    resyncInfo.connectionIndex = ConnectionIndex;
    resyncInfo.backupState = p2pStarCurrentState;

    /* Assign connection retries */
    resyncInfo.resyncTimes = RESYNC_TIMES;

    /* Change state while processing resync connection */
    p2pStarCurrentState = RESYNC_IN_PROGRESS;

    /* Store channelMap and scan duration in searchinfo used for manipulation */
    gSearchConnectionInfo.channelMap = ChannelMap;
    gSearchConnectionInfo.scanDuration = 9;

    --resyncInfo.resyncTimes;

    /* Start the scan procedure */
    return (ScanChannel());	

}
#endif

bool MiApp_SendData(uint8_t addr_len, uint8_t *addr, uint8_t msglen, uint8_t *msgpointer, uint8_t msghandle,
                                                              bool ackReq, DataConf_callback_t ConfCallback)
{
	P2PStarDataFrame_t *dataFramePtr = NULL;
	if (IN_NETWORK_STATE == p2pStarCurrentState &&  MAX_PAYLOAD >= msglen)
	{
	    bool broadcast = false;
		uint16_t DestinationAddress16 = ((addr[1] << 8) + addr[0]);
		/* Check if the frame is broadcast */
	    if(addr_len == 2 && (DestinationAddress16 == 0xFFFF))
	    {
		    broadcast = true;
#ifdef ENABLE_INDIRECT_MESSAGE
		    uint8_t i;
		    /* Add individual indirect entry for each sleeping device in connection table */
		    for(i = 0; i < CONNECTION_SIZE; i++)
		    {
			    if(miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.isValid && miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.RXOnWhenIdle == 0 
					&& 50 < MiMem_PercentageOfFreeBuffers())
			    {
					dataFramePtr = (P2PStarDataFrame_t *)MiMem_Alloc(sizeof(P2PStarDataFrame_t));
					if (NULL == dataFramePtr)
					{
						return false;
					}
					dataFramePtr->dataFrame.confCallback = ConfCallback;
					memcpy(&(dataFramePtr->dataFrame.destAddress), miwiDefaultRomOrRamParams->ConnectionTable[i].Address, MY_ADDRESS_LENGTH);
					dataFramePtr->dataFrame.msghandle = msghandle;
					dataFramePtr->dataFrame.msgLength = msglen;
					memcpy(&(dataFramePtr->dataFrame.msg), msgpointer, msglen);
					dataFramePtr->dataFrame.timeout = INDIRECT_MESSAGE_TIMEOUT;
					dataFramePtr->dataFrame.ackReq = 0;
					dataFramePtr->dataFrame.broadcast = 1;
					miQueueAppend(&indirectFrameQueue, (miQueueBuffer_t*)dataFramePtr);
			    }
		    }
#endif
			/* Also send the broadcast for all the Non-sleeping end devices */
			dataFramePtr = (P2PStarDataFrame_t *)MiMem_Alloc(sizeof(P2PStarDataFrame_t));
			if (NULL == dataFramePtr)
			{
				return false;
			}
			dataFramePtr->dataFrame.confCallback = ConfCallback;
			dataFramePtr->dataFrame.msghandle = msghandle;
			dataFramePtr->dataFrame.msgLength = msglen;
			dataFramePtr->dataFrame.timeout = 0;
			memcpy(&(dataFramePtr->dataFrame.msg), msgpointer, msglen);
			frameTransmit(broadcast, myPANID, addr, false, false, msglen, dataFramePtr->dataFrame.msg, msghandle, 0, macAckOnlyDataCallback);
			miQueueAppend(&macAckOnlyFrameQueue, (miQueueBuffer_t*)dataFramePtr);
			return true;
	    }
	    else
	    {
#ifdef ENABLE_INDIRECT_MESSAGE
		    uint8_t i;
		    
		    for(i = 0; i < CONNECTION_SIZE; i++)
		    {
			    // check if RX on when idle
			    if(miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.isValid && (miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.RXOnWhenIdle == 0) &&
			    isSameAddress(addr, miwiDefaultRomOrRamParams->ConnectionTable[i].Address) )
			    {
					dataFramePtr = (P2PStarDataFrame_t *)MiMem_Alloc(sizeof(P2PStarDataFrame_t));
					if (NULL == dataFramePtr)
					{
						return false;
					}
					dataFramePtr->dataFrame.confCallback = ConfCallback;
					memcpy(&(dataFramePtr->dataFrame.destAddress), addr, MY_ADDRESS_LENGTH);
					dataFramePtr->dataFrame.msghandle = msghandle;
					dataFramePtr->dataFrame.msgLength = msglen;
					memcpy(&(dataFramePtr->dataFrame.msg), msgpointer, msglen);
					dataFramePtr->dataFrame.timeout = INDIRECT_MESSAGE_TIMEOUT;
					dataFramePtr->dataFrame.ackReq = ackReq;
					
					miQueueAppend(&indirectFrameQueue, (miQueueBuffer_t*)dataFramePtr);
					return true;
			    }
		    }
#endif
	    }
	    
#if defined(ENABLE_ENHANCED_DATA_REQUEST) && defined(ENABLE_SLEEP_FEATURE)
	    if( P2PStatus.bits.Sleeping )
	    {
		    P2PStatus.bits.Enhanced_DR_SecEn = 0;
		    return true;
	    }
#endif
		
		dataFramePtr = (P2PStarDataFrame_t *)MiMem_Alloc(sizeof(P2PStarDataFrame_t));
		if (NULL == dataFramePtr)
		{
			return false;
		}
		dataFramePtr->dataFrame.confCallback = ConfCallback;
		memcpy(&(dataFramePtr->dataFrame.destAddress), addr, MY_ADDRESS_LENGTH);
		dataFramePtr->dataFrame.msghandle = msghandle;
		dataFramePtr->dataFrame.msgLength = msglen;
		dataFramePtr->dataFrame.timeout = 0;
		dataFramePtr->dataFrame.ackReq = ackReq;
#if defined(PROTOCOL_STAR)
		if (END_DEVICE == role)
		{
			if (MY_ADDRESS_LENGTH == addr_len && isSameAddress(addr, miwiDefaultRomOrRamParams->ConnectionTable[0].Address))
			{
				memcpy(&(dataFramePtr->dataFrame.msg), msgpointer, msglen);
				frameTransmit(broadcast, myPANID, addr, false, false, msglen, dataFramePtr->dataFrame.msg, msghandle, ackReq, macAckOnlyDataCallback);
				miQueueAppend(&macAckOnlyFrameQueue, (miQueueBuffer_t*)dataFramePtr);
			}
			else
			{
				// packet forward
				dataFramePtr->dataFrame.msg[0] = CMD_FORWRD_PACKET;
				dataFramePtr->dataFrame.msg[1] = addr[0];
				dataFramePtr->dataFrame.msg[2] = addr[1];
				dataFramePtr->dataFrame.msg[3] = addr[2];

				memcpy(&(dataFramePtr->dataFrame.msg[4]), msgpointer, msglen);
				dataFramePtr->dataFrame.msgLength = msglen + 4;
				if (ackReq)
				{
					dataFramePtr->dataFrame.timeout = SW_ACK_TIMEOUT + 1;
					frameTransmit(broadcast, myPANID, miwiDefaultRomOrRamParams->ConnectionTable[0].Address, true, false, dataFramePtr->dataFrame.msgLength, dataFramePtr->dataFrame.msg, msghandle, ackReq, appAckWaitDataCallback);
					miQueueAppend(&appAckWaitDataQueue, (miQueueBuffer_t*)dataFramePtr);
					SYS_TimerStart(&dataTimer);
				}
				else
				{
					frameTransmit(broadcast, myPANID, miwiDefaultRomOrRamParams->ConnectionTable[0].Address, true, false, dataFramePtr->dataFrame.msgLength, dataFramePtr->dataFrame.msg, msghandle, ackReq, macAckOnlyDataCallback);
					miQueueAppend(&macAckOnlyFrameQueue, (miQueueBuffer_t*)dataFramePtr);
				}
			}
		}
		else
		{
			memcpy(&(dataFramePtr->dataFrame.msg), msgpointer, msglen);
			frameTransmit(broadcast, myPANID, addr, false, false, msglen, dataFramePtr->dataFrame.msg, msghandle, ackReq, macAckOnlyDataCallback);
			miQueueAppend(&macAckOnlyFrameQueue, (miQueueBuffer_t*)dataFramePtr);
		}
#else
		memcpy(&(dataFramePtr->dataFrame.msg), msgpointer, msglen);
		frameTransmit(broadcast, myPANID, addr, false, false, msglen, dataFramePtr->dataFrame.msg, msghandle, ackReq, macAckOnlyDataCallback);
		miQueueAppend(&macAckOnlyFrameQueue, (miQueueBuffer_t*)dataFramePtr);
#endif
	}
	return true;
}

void CommandConfCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
    MiMem_Free(msgPointer);
}

static void frameTxCallback(uint8_t handle, miwi_status_t status, uint8_t* msgPointer)
{
    txCallbackReceived = true;
    DataConf_callback_t callback = sentFrame->txFrameEntry.frameConfCallback;
    if (NULL != callback)
    {
        callback(handle, status, msgPointer);
    }
    else
    {
        MiMem_Free(msgPointer);
    }
    MiMem_Free((uint8_t *)sentFrame);
}

/* Evaluate Total No of Peer Connection on a Node */
uint8_t Total_Connections(void)
{
    uint8_t count=0 , i;
    for (i=0;i<CONNECTION_SIZE;i++)
    {
#if defined(PROTOCOL_STAR)
        if (miwiDefaultRomOrRamParams->ConnectionTable[i].Address[0] != 0x00 || miwiDefaultRomOrRamParams->ConnectionTable[i].Address[1] != 0x00 || miwiDefaultRomOrRamParams->ConnectionTable[i].Address[2] != 0x00)
        {
            count++;
        }
#else
        if (miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.isValid)
        {
            count++;
        }
#endif
    }
    return count;
}


#if defined(ENABLE_HAND_SHAKE)
/*********************************************************************
* uint8_t AddConnection(void)
*
* Overview:        This function create a new P2P connection entry
*
* PreCondition:    A P2P Connection Request or Response has been
*                  received and stored in rxMessage structure
*
* Input:  None
*
* Output:
*          The index of the P2P Connection Entry for the newly added
*          connection
*
* Side Effects:    A new P2P Connection Entry is created. The search
*                  connection operation ends if an entry is added
*                  successfully
*
********************************************************************/
uint8_t AddConnection(uint8_t capacityInfo)
{
    uint8_t i;
    uint8_t status = STATUS_SUCCESS;
    uint8_t connectionSlot = 0xFF;

    /* if no peerinfo attached, this is only an active scan request,
       so do not save the source device's info */
#ifdef ENABLE_ACTIVE_SCAN
        if( rxMessage.PayloadSize < 3 )
        {
            return STATUS_ACTIVE_SCAN;
        }
#endif

    /* loop through all entry and locate an proper slot */
    for(i = 0; i < CONNECTION_SIZE; i++)
    {
        /* check if the entry is valid */
        if( miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.isValid )
        {
            // check if the entry address matches source address of current received packet
            if( isSameAddress(rxMessage.SourceAddress, miwiDefaultRomOrRamParams->ConnectionTable[i].Address) )
            {
                connectionSlot = i;
                status = STATUS_EXISTS;
                break;
            }
        }
        else if( connectionSlot == 0xFF )
        {
            /* store the first empty slot */
            connectionSlot = i;
        }
    }

    if( connectionSlot == 0xFF )
    {
        return STATUS_NOT_ENOUGH_SPACE;
    }
    else
    {
        if( ConnMode >= ENABLE_PREV_CONN )
        {
            return status;
        }
        MyindexinPC = connectionSlot;
        /* store the source address */
        for(i = 0; i < 8; i++)
        {
            miwiDefaultRomOrRamParams->ConnectionTable[connectionSlot].Address[i] = rxMessage.SourceAddress[i];
        }

        /* store the capacity info and validate the entry */
        miwiDefaultRomOrRamParams->ConnectionTable[connectionSlot].status.bits.isValid = 1;
        miwiDefaultRomOrRamParams->ConnectionTable[connectionSlot].status.bits.RXOnWhenIdle = (capacityInfo & 0x01);

        /* store possible additional connection payload */
#if ADDITIONAL_NODE_ID_SIZE > 0
            for(i = 0; i < ADDITIONAL_NODE_ID_SIZE; i++)
            {
                miwiDefaultRomOrRamParams->ConnectionTable[connectionSlot].PeerInfo[i] = rxMessage.Payload[3+i];
            }
#endif

#ifdef ENABLE_SECURITY
            /* if security is enabled, clear the incoming frame control */
            IncomingFrameCounter[connectionSlot].Val = 0;
#endif
        LatestConnection = connectionSlot;
    }
    conn_size = Total_Connections();
#if defined (ENABLE_NETWORK_FREEZER)
    PDS_Store(PDS_EDC_ID);
#endif
    return status;
}
#endif

/*********************************************************************
 * BOOL    isSameAddress(uint8_t *Address1, uint8_t *Address2)
 *
 * Overview:        This function compares two long addresses and returns
 *                  the boolean to indicate if they are the same
 *
 * PreCondition:
 *
 * Input:
 *          Address1    - Pointer to the first long address to be compared
 *          Address2    - Pointer to the second long address to be compared
 *
 * Output:
 *          If the two address are the same
 *
 * Side Effects:
 *
 ********************************************************************/
bool isSameAddress(INPUT uint8_t *Address1, INPUT uint8_t *Address2)
{
    uint8_t i;

    for(i = 0; i < MY_ADDRESS_LENGTH; i++)
    {
        if( Address1[i] != Address2[i] )
        {
            return false;
        }
    }
    return true;
}

#ifndef ENABLE_SLEEP_FEATURE
static void connectionRespConfCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
    /* Free the Frame Memory */
    MiMem_Free(msgPointer);

#if defined(PROTOCOL_STAR)
    /* Broadcast connection table upon a device join */
    MiApp_BroadcastConnectionTable();
#endif
}
#endif

#if defined(PROTOCOL_STAR)
#if defined(ENABLE_LINK_STATUS)
static void linkStatusConfCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
    MiMem_Free(msgPointer);

    if (SUCCESS != status)
    {
        if (linkStatusFailureCount >= MAX_LINK_STATUS_FAILURES)
        {
            /* Stop Timers */
            linkStatusTimeInterval = 0;
#ifdef ENABLE_SLEEP_FEATURE
            dataRequestInterval = 0;
#endif
            if ((NULL != linkFailureCallback) && (p2pStarCurrentState != DISCONNECTED))
            {
                linkFailureCallback();
            }
            p2pStarCurrentState = DISCONNECTED;	
        }
        ++linkStatusFailureCount;
    }
    else
    {
        linkStatusFailureCount = 0;
    }
}

static void sendLinkStatus(void)
{
    uint8_t* dataPtr = NULL;
    uint8_t dataLen = 0;

    /* Allocate memory for link status command */
    dataPtr = MiMem_Alloc(PACKETLEN_CMD_IAM_ALIVE);
    if (NULL == dataPtr)
        return;

    dataPtr[dataLen++] = CMD_IAM_ALIVE;
    /* Pan Co is @ index 0 of connection table of END_Device in a Star Network */
    frameTransmit(false, myPANID, miwiDefaultRomOrRamParams->ConnectionTable[0].Address, true, false,
    dataLen, dataPtr,0, true, linkStatusConfCallback);
}

static void findInActiveDevices(void)
{
    uint8_t i;

    for (i = 0;i < CONNECTION_SIZE; i++)
    {
        if (miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.isValid)
        {
            if (miwiDefaultRomOrRamParams->ConnectionTable[i].link_status == 0 && miwiDefaultRomOrRamParams->ConnectionTable[i].permanent_connections != 0xFF)
            {
                MiApp_RemoveConnection(i);
            }
            else
            {
                miwiDefaultRomOrRamParams->ConnectionTable[i].link_status = 0;
            }
        }
    }
}

static void startLinkStatusTimer(void)
{
    /* Start the timer for sending link status periodically  */
    linkStatusTimeInterval = LINK_STATUS_TIMEOUT;
}
#endif
#endif

#ifdef ENABLE_SLEEP_FEATURE
static void dataRequestConfCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
    MiMem_Free(msgPointer);
    rfdDataWaitTimer.handler = rfdDataWaitTimerExpired;
    rfdDataWaitTimer.timeout = RFD_DATA_WAIT / 1000;
    rfdDataWaitTimer.interval = RFD_DATA_WAIT / 1000;
    rfdDataWaitTimer.mode = SYS_TIMER_INTERVAL_MODE;
    SYS_TimerStart(&rfdDataWaitTimer);
}

static void sendDataRequest(void)
{
	uint8_t* dataPtr = NULL;
	uint8_t dataLen = 0;

	/* Allocate memory for link status command */
	dataPtr = MiMem_Alloc(PACKETLEN_MAC_DATA_REQUEST);
	if (NULL == dataPtr)
	return;

	dataPtr[dataLen++] = CMD_MAC_DATA_REQUEST;
	/* Pan Co is @ index 0 of connection table of END_Device in a Star Network */
	frameTransmit(false, myPANID, miwiDefaultRomOrRamParams->ConnectionTable[0].Address, true, false,
	dataLen, dataPtr,0, true, dataRequestConfCallback);
	
	P2PStatus.bits.DataRequesting = 1;
}
#endif

/******************************************************************************
* Function:
*      void frameParse(MAC_RECEIVED_PACKET *macRxPacket)
*
* Summary:
*      This function parses all the received frames
*
* Parameters:
*      MAC_RECEIVED_PACKET *macRxPacket - Pointer of the received MAC frame
*
* Returns:
*      None
******************************************************************************/
void frameParse(MAC_RECEIVED_PACKET *macRxPacket)
{
    uint8_t i;

    /* Reject if Source PANID is not from my PANID
    if (macRxPacket->SourcePANID.Val != MY_PAN_ID)
    {
        return;
    }
    */

    /* Parse the message and Fill the rxMessage structure */
    rxMessage.flags.Val = 0;
    rxMessage.flags.bits.broadcast = MACRxPacket.flags.bits.broadcast;
    rxMessage.flags.bits.secEn = MACRxPacket.flags.bits.secEn;
    rxMessage.flags.bits.command = (MACRxPacket.flags.bits.packetType == PACKET_TYPE_COMMAND) ? 1:0;
    rxMessage.flags.bits.srcPrsnt = MACRxPacket.flags.bits.sourcePrsnt;

    /* Read Source Address if source present bit is set in flags */
    if( MACRxPacket.flags.bits.sourcePrsnt )
    {
        rxMessage.SourceAddress = MACRxPacket.SourceAddress;
    }

    rxMessage.SourcePANID.Val = MACRxPacket.SourcePANID.Val;
    rxMessage.PayloadSize = MACRxPacket.PayloadLen;
    rxMessage.Payload = MACRxPacket.Payload;

    rxMessage.PacketLQI = MACRxPacket.LQIValue;
    rxMessage.PacketRSSI = MACRxPacket.RSSIValue;

    /* Command Frames Handling */
    if( rxMessage.flags.bits.command )
    {
        switch( rxMessage.Payload[0] )
        {
            case CMD_P2P_CONNECTION_REQUEST:
            {
#ifndef ENABLE_SLEEP_FEATURE
                uint8_t* dataPtr = NULL;
                uint8_t dataLen = 0;
#endif
#if defined(PROTOCOL_STAR)
                if(PAN_COORD != role)
                {
                    /* Ignore If not PANCoordinator -Important for star network */
                    return;
                }
#endif

#ifdef ENABLE_SLEEP_FEATURE
                /* If a device goes to sleep, it can only have one connection, as the result, it cannot accept new connection request */
                return;
#else
                uint8_t status = STATUS_SUCCESS;

                 /* if new connection is not allowed, ignore the request */
                if( ConnMode > ENABLE_PREV_CONN )
                {
                     return;
                }
                /* if channel does not math, it may be a sub-harmonics signal, ignore the request */
                if( currentChannel != rxMessage.Payload[1] )
                {
                    return;
                }

#if !defined(TARGET_SMALL) && defined(IEEE_802_15_4)
                /* if PANID does not match, ignore the request */
                if( (rxMessage.SourcePANID.Val != 0xFFFF) && (rxMessage.SourcePANID.Val != myPANID.Val) &&
                        (rxMessage.PayloadSize > 2))
                {
                    status = STATUS_NOT_SAME_PAN;
                }
                else
#endif
                {
                    /* Request accepted, try to add the requesting device into P2P Connection Entry */
                    status = AddConnection(rxMessage.Payload[2]);
                }
#if defined(PROTOCOL_STAR) && defined(ENABLE_LINK_STATUS)
                if (rxMessage.Payload[3] == 0xAA)
                {
                    for (uint8_t p = 0 ;p <CONNECTION_SIZE;p++)
                    {
                        if (isSameAddress(rxMessage.SourceAddress, miwiDefaultRomOrRamParams->ConnectionTable[p].Address) )
                        {
                             miwiDefaultRomOrRamParams->ConnectionTable[p].permanent_connections = 0xFF;
                        }
                    }
                }
#endif
                if( (ConnMode == ENABLE_PREV_CONN) && (status != STATUS_EXISTS && status != STATUS_ACTIVE_SCAN) )
                {
                    status = STATUS_NOT_PERMITTED;
                }

                if( (status == STATUS_SUCCESS || status == STATUS_EXISTS ) && MiApp_CB_AllowConnection(LatestConnection) == false )
                {
                    miwiDefaultRomOrRamParams->ConnectionTable[LatestConnection].status.Val = 0;
                    status = STATUS_NOT_PERMITTED;
                }

                /* Prepare the P2P_CONNECTION_RESPONSE command */
                dataPtr = MiMem_Alloc(TX_BUFFER_SIZE);

                if (NULL == dataPtr)
                    return;

                /* Fill Connection Response Structure */
                dataPtr[dataLen++] = CMD_P2P_CONNECTION_RESPONSE;
                dataPtr[dataLen++] = status;
#if defined(PROTOCOL_STAR)
                dataPtr[dataLen++] = MyindexinPC;
#endif
                if( status == STATUS_SUCCESS || status == STATUS_EXISTS )
                {
                    dataPtr[dataLen++] = P2PCapacityInfo;
#if ADDITIONAL_NODE_ID_SIZE > 0
                    for(i = 0; i < ADDITIONAL_NODE_ID_SIZE; i++)
                    {
                        dataPtr[dataLen++] = miwiDefaultRomOrRamParams->AdditionalNodeID[i];
                    }
#endif
                }

                /* Unicast the P2P_CONNECTION_RESPONSE to the requesting device */
#ifdef TARGET_SMALL
                frameTransmit(false, myPANID, rxMessage.SourceAddress, true, rxMessage.flags.bits.secEn,
                        dataLen, dataPtr, 0, true, connectionRespConfCallback);
#else
                frameTransmit(false, rxMessage.SourcePANID, rxMessage.SourceAddress, true, rxMessage.flags.bits.secEn,
                        dataLen, dataPtr, 0, true, connectionRespConfCallback);
#endif
#if defined(ENABLE_NETWORK_FREEZER)
                if( status == STATUS_SUCCESS )
                {
                     PDS_Store(PDS_CONNECTION_TABLE_ID);
                }
#endif
#endif
            }
            break;

            case CMD_P2P_CONNECTION_RESPONSE:
            {
                if(ESTABLISHING_NETWORK != p2pStarCurrentState)
                {
                    /* Ignore If not in establish state */
                    return;
                }

                switch( rxMessage.Payload[1] )
                {
                    case STATUS_SUCCESS:
                    case STATUS_EXISTS:
                    if( myPANID.Val == 0xFFFF )
                    {
                        uint16_t broadcastAddr = 0xFFFF;
                        myPANID.Val = rxMessage.SourcePANID.Val;
                        MiMAC_SetAltAddress((uint8_t *)&broadcastAddr, (uint8_t *)&myPANID.Val);
#if defined(ENABLE_NETWORK_FREEZER)
                        PDS_Store(PDS_PANID_ID);
#endif
                    }
                    if (rxMessage.Payload[1] == STATUS_EXISTS)
                    {
                        gEstConnectionInfo.status = ALREADY_EXISTS;
                    }
                    else if (rxMessage.Payload[1] == STATUS_SUCCESS)
                    {
                        gEstConnectionInfo.status = SUCCESS;
                    }
                    else
                    {
                        gEstConnectionInfo.status = FAILURE;
                    }
                    /* Retry is not needed since already response received */
                    gEstConnectionInfo.connectionRetries = 0;
                    {
                    /* Add Entry in the connection Table */
#if defined(PROTOCOL_STAR)
                    uint8_t status = AddConnection(rxMessage.Payload[3]);
#else
                    uint8_t status = AddConnection(rxMessage.Payload[2]);
#endif
                    if ((status == STATUS_SUCCESS) || (status == STATUS_EXISTS))
                    {
#if defined(PROTOCOL_STAR)
                        /* Role is end node */
                        role = END_DEVICE;
                        /* Initiate Link Status Timer */
                        startLinkStatusTimer();
#endif
#ifdef ENABLE_SLEEP_FEATURE
                        dataRequestInterval = RFD_WAKEUP_INTERVAL;
#endif
                    }
#if defined(PROTOCOL_STAR)
                    myConnectionIndex_in_PanCo = rxMessage.Payload[2];
#if defined(ENABLE_NETWORK_FREEZER)
                    PDS_Store(PDS_MYINDEX_ID);
#endif
#endif
                    }
#if defined(ENABLE_NETWORK_FREEZER)
                   PDS_Store(MIWI_ALL_MEMORY_MEM_ID);
#endif
                   break;

                    default:
                    break;
                }
            }
            break;

            case CMD_P2P_ACTIVE_SCAN_REQUEST:
            {
                uint8_t* dataPtr = NULL;
                uint8_t dataLen = 0;
                if(ConnMode > ENABLE_ACTIVE_SCAN_RSP)
                {
                    return;
                }
                if( currentChannel != rxMessage.Payload[1] )
                {
                    return;
                }

                /* Prepare Active Scan Response */
                dataPtr = MiMem_Alloc(PACKETLEN_P2P_ACTIVE_SCAN_RESPONSE);

                if (NULL == dataPtr)
                    return;

                dataPtr[dataLen++] = CMD_P2P_ACTIVE_SCAN_RESPONSE;
                dataPtr[dataLen++] = P2PCapacityInfo;

#if ADDITIONAL_NODE_ID_SIZE > 0
                for(i = 0; i < ADDITIONAL_NODE_ID_SIZE; i++)
                {
                    dataPtr[dataLen++] = (miwiDefaultRomOrRamParams->AdditionalNodeID[i]);
                }
#endif

                /* unicast the response to the requesting device */
#ifdef TARGET_SMALL
                frameTransmit(false, myPANID, rxMessage.SourceAddress, true, rxMessage.flags.bits.secEn,
                dataLen, dataPtr, 0, true, CommandConfCallback);
#else
                frameTransmit(false, rxMessage.SourcePANID, rxMessage.SourceAddress, true, rxMessage.flags.bits.secEn,
                dataLen, dataPtr, 0, true, CommandConfCallback);
#endif
            }
            break;

            case CMD_P2P_ACTIVE_SCAN_RESPONSE:
            {
                if(RESYNC_IN_PROGRESS == p2pStarCurrentState)
                {
                    resyncInfo.resyncTimes = 0;
#ifdef ENABLE_ACTIVE_SCAN
                    activeScanDurationTimeInterval = 0;
#endif
                    p2pStarCurrentState = IN_NETWORK_STATE;
                    resyncInfo.confCallback(currentChannel, SUCCESS);
                    resyncInfo.confCallback = NULL;
                }
#ifdef ENABLE_ACTIVE_SCAN
                else
                {
                    i = 0;
                    for(; i < gSearchConnectionInfo.activeScanResultIndex; i++)
                    {
                        if( (miwiDefaultRomOrRamParams->ActiveScanResults[i].Channel == currentChannel) &&
#if defined(IEEE_802_15_4)
                        (miwiDefaultRomOrRamParams->ActiveScanResults[i].PANID.Val == rxMessage.SourcePANID.Val) &&
#endif
                        isSameAddress(miwiDefaultRomOrRamParams->ActiveScanResults[i].Address, rxMessage.SourceAddress)
                        )
                        {
                            break;
                        }
                    }
                    if( i == gSearchConnectionInfo.activeScanResultIndex && (i < ACTIVE_SCAN_RESULT_SIZE))
                    {
                        miwiDefaultRomOrRamParams->ActiveScanResults[gSearchConnectionInfo.activeScanResultIndex].Channel = currentChannel;
                        miwiDefaultRomOrRamParams->ActiveScanResults[gSearchConnectionInfo.activeScanResultIndex].RSSIValue = rxMessage.PacketRSSI;
                        miwiDefaultRomOrRamParams->ActiveScanResults[gSearchConnectionInfo.activeScanResultIndex].LQIValue = rxMessage.PacketLQI;
                        miwiDefaultRomOrRamParams->ActiveScanResults[gSearchConnectionInfo.activeScanResultIndex].PANID.Val = rxMessage.SourcePANID.Val;

                        for(i = 0; i < MY_ADDRESS_LENGTH; i++)
                        {
                            miwiDefaultRomOrRamParams->ActiveScanResults[gSearchConnectionInfo.activeScanResultIndex].Address[i] = rxMessage.SourceAddress[i];
                        }
                        miwiDefaultRomOrRamParams->ActiveScanResults[gSearchConnectionInfo.activeScanResultIndex].Capability.Val = rxMessage.Payload[1];
#if ADDITIONAL_NODE_ID_SIZE > 0
                        for(i = 0; i < ADDITIONAL_NODE_ID_SIZE; i++)
                        {
                            miwiDefaultRomOrRamParams->ActiveScanResults[gSearchConnectionInfo.activeScanResultIndex].PeerInfo[i] = rxMessage.Payload[2+i];
                        }
#endif
                        gSearchConnectionInfo.activeScanResultIndex++;
                    }
                }
#endif
            }
            break;

#if defined (PROTOCOL_STAR)
            case CMD_SHARE_CONNECTION_TABLE:
            {
                if (END_DEVICE == role)
                {
                    /* END_devices FFD|| RFD process this Packet */
                    end_nodes = rxMessage.Payload[1];
                    store_connection_tb(rxMessage.Payload);
                }
            }
            break;
            case CMD_DATA_TO_ENDDEV_SUCCESS:
            {
				P2PStarDataFrame_t *dataFramePtr = NULL;
				dataFramePtr = (P2PStarDataFrame_t *) miQueueRemove(&appAckWaitDataQueue, NULL);

				if (NULL == dataFramePtr)
				{
					return;
				}
				DataConf_callback_t callback = dataFramePtr->dataFrame.confCallback;
				if (NULL != callback)
				{
					callback(dataFramePtr->dataFrame.msghandle, SUCCESS, dataFramePtr->dataFrame.msg);
				}
				MiMem_Free((uint8_t *)dataFramePtr);
            }
            break;
            case CMD_FORWRD_PACKET:
            {
				/* If the role is PANC, the data has to be forwarded to corresponding enddevice */
	            if (PAN_COORD == role)
	            {
					/* Based on the end device short address, the index in connection table is retrieved */
					uint8_t ed_index = Find_Index(&(rxMessage.Payload[1]));
					if (0xFF != ed_index)
					{
						/* Allocate buffer for data forward and update */
						P2PStarDataFrame_t* dataPtr = NULL;
						uint8_t dataLen = 0;
						dataPtr = (P2PStarDataFrame_t*)MiMem_Alloc(sizeof(P2PStarDataFrame_t));
						if (NULL == dataPtr)
						{
							return;
						}
						memcpy(dataPtr->dataFrame.destAddress, miwiDefaultRomOrRamParams->ConnectionTable[ed_index].Address, LONG_ADDR_LEN);
						/* first 3 bytes in payload is updated with short address of source end device */
						dataPtr->dataFrame.msg[dataLen++] = rxMessage.SourceAddress[0];    // Unique address of EDy (DEST ED)
						dataPtr->dataFrame.msg[dataLen++] = rxMessage.SourceAddress[1];    // Unique address of EDy (DEST ED)
						dataPtr->dataFrame.msg[dataLen++] = rxMessage.SourceAddress[2];    // Unique address of EDy (DEST ED)
						for(i = 4; i < rxMessage.PayloadSize; i++)
						{
							dataPtr->dataFrame.msg[dataLen++] = rxMessage.Payload[i];
						}
						dataPtr->dataFrame.msgLength = dataLen;
						dataPtr->dataFrame.fromEDToED = 1;
						/* If the destination end device is sleeping device, place the data in indirect queue or transmit directly */
						if(miwiDefaultRomOrRamParams->ConnectionTable[ed_index].status.bits.isValid && miwiDefaultRomOrRamParams->ConnectionTable[ed_index].status.bits.RXOnWhenIdle == 0)
						{
							if (50 < MiMem_PercentageOfFreeBuffers())
							{
								dataPtr->dataFrame.confCallback = NULL;
								dataPtr->dataFrame.timeout = INDIRECT_MESSAGE_TIMEOUT;
								dataPtr->dataFrame.ackReq = true;
								miQueueAppend(&indirectFrameQueue, (miQueueBuffer_t*)dataPtr);
							}
						}
						else
						{
							frameTransmit(false, myPANID, miwiDefaultRomOrRamParams->ConnectionTable[ed_index].Address, false, false, dataLen, dataPtr->dataFrame.msg, 1, true, appAckWaitDataCallback);
							miQueueAppend(&appAckWaitDataQueue, (miQueueBuffer_t*)dataPtr);
						}
					}
	            }
            }
            break;
#if defined(ENABLE_LINK_STATUS)
            case CMD_IAM_ALIVE:
            {
                if (PAN_COORD == role)
                {
                    // PAN CP processes this packet to qualify it as alive , increments the link stat
                    uint8_t p;
                    for (p=0  ; p < CONNECTION_SIZE ; p++)
                    {
                        if (miwiDefaultRomOrRamParams->ConnectionTable[p].Address[0] == rxMessage.SourceAddress[0] && miwiDefaultRomOrRamParams->ConnectionTable[p].Address[1] == rxMessage.SourceAddress[1]
                        && miwiDefaultRomOrRamParams->ConnectionTable[p].Address[2] == rxMessage.SourceAddress[2])
                        {
                            miwiDefaultRomOrRamParams->ConnectionTable[p].link_status++;
                            break;
                        }
                    }
                }
            }
            break;
#endif
#endif
#if defined(ENABLE_FREQUENCY_AGILITY)
            case CMD_CHANNEL_HOPPING:
            {
                if((true == channelChangeInProgress) || (rxMessage.Payload[1] != currentChannel))
                {
                    return;
                }
                channelChangeInProgress = true;
                freqAgilityRetries = 0;
                optimalChannel = rxMessage.Payload[2];
                StartChannelHopping();
            }
            break;
#endif
#if defined(ENABLE_INDIRECT_MESSAGE)
            case CMD_MAC_DATA_REQUEST:
            {
				if (indirectFrameQueue.size)
				{
					uint8_t loopIndex;
					for (loopIndex =0; loopIndex < indirectFrameQueue.size; loopIndex++)
					{
						P2PStarDataFrame_t *dataFramePtr = (P2PStarDataFrame_t *)miQueueRemove(&indirectFrameQueue, NULL);
						if (NULL != dataFramePtr)
						{
							if (isSameAddress(rxMessage.SourceAddress, dataFramePtr->dataFrame.destAddress))
							{
								frameTransmit(dataFramePtr->dataFrame.broadcast, myPANID, dataFramePtr->dataFrame.destAddress, false, false, dataFramePtr->dataFrame.msgLength, dataFramePtr->dataFrame.msg, 
									dataFramePtr->dataFrame.msghandle, dataFramePtr->dataFrame.ackReq, macAckOnlyDataCallback);
								miQueueAppend(&macAckOnlyFrameQueue, (miQueueBuffer_t *)dataFramePtr);
								break;
							}
							else
							{
								miQueueAppend(&indirectFrameQueue, (miQueueBuffer_t *)dataFramePtr);
							}
						}
					}
				}
            }
            break;
#endif
#ifndef TARGET_SMALL
            case CMD_P2P_CONNECTION_REMOVAL_REQUEST:
            {
                uint8_t* dataPtr = NULL;
                uint8_t dataLen = 0;

                dataPtr = MiMem_Alloc(PACKETLEN_P2P_CONNECTION_REMOVAL_RESPONSE);

                if (NULL == dataPtr)
                    return;

                dataPtr[dataLen++] = CMD_P2P_CONNECTION_REMOVAL_RESPONSE;

                for(i = 0; i < CONNECTION_SIZE; i++)
                {
                    /* if the record is valid */
                    if( miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.isValid )
                    {
                        /* if the record is the same as the requesting device */
                        if( isSameAddress(rxMessage.SourceAddress, miwiDefaultRomOrRamParams->ConnectionTable[i].Address) )
                        {
                            /* Find the record. disable the record and set status to be SUCCESS */
                            miwiDefaultRomOrRamParams->ConnectionTable[i].status.Val = 0;
#if defined(ENABLE_NETWORK_FREEZER)
                            PDS_Store(PDS_CONNECTION_TABLE_ID);
#endif
                            dataPtr[dataLen++] = STATUS_SUCCESS;

                            break;
                        }
                    }
                }

                if( i == CONNECTION_SIZE )
                {
                    /* not found, the requesting device is not my peer */
                    dataPtr[dataLen++] = STATUS_ENTRY_NOT_EXIST;
                }
#ifdef TARGET_SMALL
                frameTransmit(false, myPANID, rxMessage.SourceAddress, true, rxMessage.flags.bits.secEn,
                dataLen, dataPtr,0, true, CommandConfCallback);
#else
                frameTransmit(false, rxMessage.SourcePANID, rxMessage.SourceAddress, true, rxMessage.flags.bits.secEn,
                dataLen, dataPtr,0, true, CommandConfCallback);
#endif
            }
            break;

            case CMD_P2P_CONNECTION_REMOVAL_RESPONSE:
            {
                if( rxMessage.Payload[1] == STATUS_SUCCESS )
                {
                    for(i = 0; i < CONNECTION_SIZE; i++)
                    {
                        // if the record is valid
                        if( miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.isValid )
                        {
                            // if the record address is the same as the requesting device
                            if( isSameAddress(rxMessage.SourceAddress, miwiDefaultRomOrRamParams->ConnectionTable[i].Address) )
                            {
                                // invalidate the record
                                miwiDefaultRomOrRamParams->ConnectionTable[i].status.Val = 0;
#if defined(ENABLE_NETWORK_FREEZER)
                                PDS_Store(PDS_CONNECTION_TABLE_ID);
#endif
                                break;
                            }
                        }
                    }
                }
            }
            break;
#endif
            default:
            break;
        }
    }
	else
	{
		if (IN_NETWORK_STATE == p2pStarCurrentState)
		{
			pktRxcallback(&rxMessage);
		}
	}
}

void P2PTasks(void)
{
    MIWI_TICK  currentTick;

    currentTick.Val = MiWi_TickGet();

    /* Transmission Queue Handling */
    if (frameTxQueue.size && txCallbackReceived && (MiWi_TickGetDiff(currentTick, lastTxFrameTick) > (transaction_duration_us)))
    {
        TxFrame_t *txFramePtr = NULL;
        txFramePtr =  (TxFrame_t *)miQueueRemove(&frameTxQueue, NULL);
        if (NULL != txFramePtr)
        {
            uint16_t transaction_duration_sym = 0;
            sentFrame = txFramePtr;

            /* Calculate Transaction Duration Time which is based on Frame Size either minLIFSPeriod or minSIFSPeriod*/
            if ( (txFramePtr->txFrameEntry.frameLength + MAC_OVERHEAD + PHY_OVERHEAD) > aMaxSIFSFrameSize)
            {
                transaction_duration_sym = macMinLIFSPeriod_def;
            }
            else
            {
                transaction_duration_sym = macMinSIFSPeriod_def;
            }
            /* Turn around time and unit back off period added for MAC acknowledgment reception */
            transaction_duration_sym += aTurnaroundTime + aUnitBackoffPeriod;
            transaction_duration_us = MiMAC_SymbolToTicks(transaction_duration_sym);

            /* Store current transmitted frame tick information */
            lastTxFrameTick = currentTick;

            MiMAC_SendPacket(txFramePtr->txFrameEntry.frameParam, txFramePtr->txFrameEntry.frame,
            txFramePtr->txFrameEntry.frameLength, txFramePtr->txFrameEntry.frameHandle,
            frameTxCallback);
            txCallbackReceived = false;
        }
    }
    /* Check for New frame Reception, Parse and handle the frame if received  */
    if(MiMAC_ReceivedPacket())
    {
        frameParse(&MACRxPacket);
        MiMAC_DiscardPacket();
    }
    /* MiMAC Task Handler */
    MiMAC_Task();

    /* System Software Timer Handler */
    SYS_TimerTaskHandler();
#ifdef ENABLE_SLEEP_FEATURE
    if(!(P2PStatus.bits.DataRequesting || P2PStatus.bits.RxHasUserData || (frameTxQueue.size) || (!txCallbackReceived)) && (p2pStarCurrentState == IN_NETWORK_STATE))
    {
        MiMAC_PowerState(POWER_STATE_DEEP_SLEEP);
    }
#endif
}


/*********************************************************************
 * BOOL frameTransmit(BOOL Broadcast,
 *                 uint16_t_VAL DestinationPANID,
 *                 uint8_t *DestinationAddress,
 *                 BOOL isCommand,
 *                 BOOL SecurityEnabled)
 *
 * Overview:        This function sends the packet
 *
 * PreCondition:    Transceiver is initialized
 *
 * Input:
 *          BOOL        Broadcast           If packet to send needs to be broadcast
 *          uint16_t_VAL    DestinationPANID    Destination PAN Identifier
 *          uint8_t *      DestinationAddress  Pointer to destination long address
 *          BOOL        isCommand           If packet to send is a command packet
 *          BOOL        SecurityEnabled     If packet to send needs encryption
 *
 * Output:
 *          BOOL                            If operation successful
 *
 * Side Effects:    Transceiver is triggered to transmit a packet
 *
 ********************************************************************/
bool frameTransmit(INPUT bool Broadcast,
                API_UINT16_UNION DestinationPANID,
                INPUT uint8_t *DestinationAddress,
                INPUT bool isCommand,
                INPUT bool SecurityEnabled,
                INPUT uint8_t msgLen,
                INPUT uint8_t* msgPtr,
                INPUT uint8_t msghandle,
                INPUT bool ackReq,
                INPUT DataConf_callback_t ConfCallback)
{
    MAC_TRANS_PARAM *tParam;

    TxFrame_t *txFramePtr = NULL;

    txFramePtr = (TxFrame_t *) MiMem_Alloc(sizeof(TxFrame_t));

    if (NULL == txFramePtr)
    {
        return false;
    }
    tParam = &(txFramePtr->txFrameEntry.frameParam);
    tParam->flags.Val = 0;
    tParam->flags.bits.packetType = (isCommand) ? PACKET_TYPE_COMMAND : PACKET_TYPE_DATA;
    tParam->flags.bits.ackReq = (Broadcast) ? 0 : ackReq;
    tParam->flags.bits.broadcast = Broadcast;
    tParam->flags.bits.secEn = SecurityEnabled;
    #if defined(IEEE_802_15_4)
        tParam->altSrcAddr = 0;
        tParam->altDestAddr = (Broadcast) ? true : false;
    #endif

    #if defined(INFER_DEST_ADDRESS)
        tParam->flags.bits.destPrsnt = 0;
    #else
        tParam->flags.bits.destPrsnt = (Broadcast) ? 0:1;
    #endif

    #if defined(SOURCE_ADDRESS_ABSENT)
        if( tParam->flags.bits.packetType == PACKET_TYPE_COMMAND )
        {
            tParam->flags.bits.sourcePrsnt = 1;
        }
        else
        {
            tParam->flags.bits.sourcePrsnt = 0;
        }
    #else
        tParam->flags.bits.sourcePrsnt = 1;
    #endif

    tParam->DestAddress = DestinationAddress;

    #if defined(IEEE_802_15_4)
        tParam->DestPANID.Val = DestinationPANID.Val;
    #endif

    if (NULL != DestinationAddress)
    {
        if (false == tParam->flags.bits.broadcast)
        {
            memcpy(&(txFramePtr->txFrameEntry.frameDstAddr), DestinationAddress, LONG_ADDR_LEN);
        }
        else
        {
            memcpy(&(txFramePtr->txFrameEntry.frameDstAddr), DestinationAddress, SHORT_ADDR_LEN);
        }
    }

    txFramePtr->txFrameEntry.frame = msgPtr;

    tParam->DestAddress = (uint8_t*)&(txFramePtr->txFrameEntry.frameDstAddr);
    txFramePtr->txFrameEntry.frameConfCallback = ConfCallback;
    txFramePtr->txFrameEntry.frameHandle = msghandle;
    txFramePtr->txFrameEntry.frameLength = msgLen;

    miQueueAppend(&frameTxQueue, (miQueueBuffer_t *)txFramePtr);

    return true;
}

static void protocolTimerInit(void)
{
    protocolTimer.interval = PROTOCOL_TIMER_INTERVAL;
    protocolTimer.mode = SYS_TIMER_PERIODIC_MODE;
    protocolTimer.handler = protocolTimerHandler;
    SYS_TimerStart(&protocolTimer);

#if defined(PROTOCOL_STAR)
    dataTimer.interval = DATA_TIMER_INTERVAL;
    dataTimer.mode = SYS_TIMER_PERIODIC_MODE;
    dataTimer.handler = dataTimerHandler;
#endif
}

static void protocolTimerHandler(SYS_Timer_t *timer)
{
#ifdef ENABLE_SLEEP_FEATURE
    if((0 != dataRequestInterval) && ((--dataRequestInterval) == 0))
    {
        sendDataRequest();
    }
#endif
#ifdef ENABLE_ACTIVE_SCAN
    if((0 != activeScanDurationTimeInterval) && ((--activeScanDurationTimeInterval) == 0))
    {
        scanDurationExpired();
    }
#endif
#ifdef ENABLE_ED_SCAN
    if((0 != edScanDurationTimeInterval) && ((--edScanDurationTimeInterval) == 0))
    {
        edScanDurationExpired();
    }
#endif
    if((0 != connectionTimeInterval) && ((--connectionTimeInterval) == 0))
    {
        sendConnectionRequest();
    }
#ifdef ENABLE_FREQUENCY_AGILITY
    if((0 != freqAgilityBroadcastInterval) && ((--freqAgilityBroadcastInterval) == 0))
    {
        StartChannelHopping();
    }
#endif
#ifdef ENABLE_PERIODIC_CONNECTIONTABLE_SHARE
    if((0 != sharePeerDevInfoTimeInterval) && ((--sharePeerDevInfoTimeInterval) == 0))
    {
        /* Reload time interval since it is periodic timer */
        sharePeerDevInfoTimeInterval = SHARE_PEER_DEVICE_INFO_TIMEOUT;
        MiApp_BroadcastConnectionTable();
    }
#endif
#ifdef ENABLE_LINK_STATUS
    if((0 != inActiveDeviceCheckTimeInterval) && ((--inActiveDeviceCheckTimeInterval) == 0))
    {
        /* Reload time interval since it is periodic timer */
        inActiveDeviceCheckTimeInterval = FIND_INACTIVE_DEVICE_TIMEOUT;
        findInActiveDevices();
    }
    if((0 != linkStatusTimeInterval) && ((--linkStatusTimeInterval) == 0))
    {
        /* Reload time interval since it is periodic timer */
        linkStatusTimeInterval = LINK_STATUS_TIMEOUT;
        sendLinkStatus();
    }
#endif
#ifdef ENABLE_INDIRECT_MESSAGE
	if (indirectFrameQueue.size)
	{
		uint8_t loopIndex;
		for (loopIndex =0; loopIndex < indirectFrameQueue.size; loopIndex++)
		{
			P2PStarDataFrame_t *dataFramePtr = (P2PStarDataFrame_t *)miQueueRemove(&indirectFrameQueue, NULL);
			if (NULL != dataFramePtr)
			{
				if ((0 != dataFramePtr->dataFrame.timeout) && ((--dataFramePtr->dataFrame.timeout) == 0))
				{
					DataConf_callback_t callback = dataFramePtr->dataFrame.confCallback;
					if (NULL != callback && 1 != dataFramePtr->dataFrame.broadcast)
					{
						callback(dataFramePtr->dataFrame.msghandle, TRANSACTION_EXPIRED, dataFramePtr->dataFrame.msg);
					}
					MiMem_Free((uint8_t *)dataFramePtr);
				}
				else
				{
					miQueueAppend(&indirectFrameQueue, (miQueueBuffer_t *)dataFramePtr);
				}
			}
		}
	}
#endif
}

#if defined(PROTOCOL_STAR)
static void dataTimerHandler(SYS_Timer_t *timer)
{
	uint8_t loopIndex;
	P2PStarDataFrame_t *dataFramePtr = NULL;
	for (loopIndex = 0; loopIndex < appAckWaitDataQueue.size; loopIndex++)
	{
		dataFramePtr = (P2PStarDataFrame_t *) miQueueRemove(&appAckWaitDataQueue, NULL);

		if (NULL == dataFramePtr)
		{
			return;
		}
		
		if((0 != dataFramePtr->dataFrame.timeout) && (--dataFramePtr->dataFrame.timeout) == 0)
		{
			DataConf_callback_t callback = dataFramePtr->dataFrame.confCallback;
			if (NULL != callback)
			{
				callback(dataFramePtr->dataFrame.msghandle, NO_ACK, (uint8_t*)&(dataFramePtr->dataFrame.msg));
			}
			MiMem_Free((uint8_t *)dataFramePtr);
		}
		else
		{
			miQueueAppend(&appAckWaitDataQueue, (miQueueBuffer_t *)dataFramePtr);
		}
	}
	if (0 == appAckWaitDataQueue.size)
	{
		SYS_TimerStop(&dataTimer);
	}
}

/************************************************************************************
* Function:
*      void MiApp_BroadcastConnectionTable(void)
*
* Summary:
*      This function is  used for command type packet TX (Only used by PAN CO)
*
* Description:
*      This function is used by only PAN CO in a Star network and is a cmd
*      type packet. PAN CO in Star Network , holds the responsibility to Share
*      peer end devices connection table.
*
* PreCondition:
*      Protocol initialization has been done.
*
* Returns:
*      None.
*
* Remarks:
*      None
*
*****************************************************************************************/
static void MiApp_BroadcastConnectionTable(void)
{
    uint8_t i,j , k , count;
    // Based on Connection Size in Network broadcast the connection details Multiple Times
    // so that all the END_DEVICES in Star Network Receive the packet
    uint8_t broadcast_count = 0;
    uint8_t* dataPtr = NULL;
    uint8_t dataLen = 0;

    if ((conn_size  * 4 ) + 4 < TX_BUFFER_SIZE)
    {
        broadcast_count = 1;
    }
    else
    {
        broadcast_count = ((conn_size * 4) + 4 )/ TX_BUFFER_SIZE;
        if ((conn_size *4) + 4 % TX_BUFFER_SIZE != 0)
        {
            broadcast_count = broadcast_count + ((conn_size *4) + 4 )% TX_BUFFER_SIZE;
        }

    }

    for (i = 0 ; i < broadcast_count ; i++)
    {
        dataPtr = MiMem_Alloc(TX_BUFFER_SIZE);
        if (NULL == dataPtr)
            return;
        dataPtr[dataLen++] = CMD_SHARE_CONNECTION_TABLE;
        dataPtr[dataLen++] = conn_size; // No of end devices in network
        dataPtr[dataLen++] = (((TX_BUFFER_SIZE-4)/4)*i);
        dataPtr[dataLen++] = (((TX_BUFFER_SIZE-4)/4)*(i+1));
        count = 4;
        for (j= ((TX_BUFFER_SIZE-4)/4)*i ;j<((TX_BUFFER_SIZE-4)/4)*(i+1);j++)
        {
            if (j < conn_size)
            {
                if (miwiDefaultRomOrRamParams->ConnectionTable[j].status.bits.isValid)
                {
                    dataPtr[dataLen++] = (miwiDefaultRomOrRamParams->ConnectionTable[j].Address[0]);
                    dataPtr[dataLen++] = (miwiDefaultRomOrRamParams->ConnectionTable[j].Address[1]);
                    dataPtr[dataLen++] = (miwiDefaultRomOrRamParams->ConnectionTable[j].Address[2]);
                    dataPtr[dataLen++] = j;
                }
                else
                {
                    dataPtr[dataLen++] = 0xff;
                    dataPtr[dataLen++] = 0xff;
                    dataPtr[dataLen++] = 0xff;
                    dataPtr[dataLen++] = j;
                }
                count = count + 4;
            }
        }
        // Fill the remaining buffer with garbage value
        for (k=count;k<TX_BUFFER_SIZE;k++)
        {
            dataPtr[dataLen++] = 0xFF;   // Garbage Value
        }
        frameTransmit(true, myPANID, NULL, true, false, dataLen, dataPtr,0, true, CommandConfCallback);
    }
}
#endif

#if defined(PROTOCOL_STAR)
/* All connections (FFD || RFD)are stored in Connection Table of PAN Coordinator
    Each Connection is identified by its index no. In case of Data TX , EDx --> PAN CO --> EDy
    PAN Coordinator will forward the data to EDy , In order to know */
static uint8_t Find_Index (uint8_t *DestAddr)
{
    uint8_t i;
    uint8_t return_val;
    for (i = 0;i < conn_size; i++)
    {
        if (miwiDefaultRomOrRamParams->ConnectionTable[i].status.bits.isValid)
        {
            if (DestAddr[0] == miwiDefaultRomOrRamParams->ConnectionTable[i].Address[0] && DestAddr[1] == miwiDefaultRomOrRamParams->ConnectionTable[i].Address[1] && DestAddr[2] == miwiDefaultRomOrRamParams->ConnectionTable[i].Address[2] )
            {
                return_val = i;
                break;
            }

        }
    }
    if (i == conn_size)
    {
        return_val = 0xff;
    }
    return return_val;
}

bool MiApp_SubscribeLinkFailureCallback(LinkFailureCallback_t callback)
{
    if (NULL != callback)
    {
        linkFailureCallback = callback;
        return true;
    }
    return false;
}

static void handleLostConnection(void)
{
    uint8_t i ;
    bool stat = false;
    if (END_DEVICE == role)
    {
        for (i = 0; i < end_nodes; i++)
        {
            if (myLongAddress[0] == END_DEVICES_Short_Address[i].Address[0] && myLongAddress[1] == END_DEVICES_Short_Address[i].Address[1])
            {
                stat = true;
            }
        }
        if (!stat)
        {
            /* Stop Timers */
            linkStatusTimeInterval = 0;
#ifdef ENABLE_SLEEP_FEATURE
            dataRequestInterval = 0;
#endif
            if ((NULL != linkFailureCallback) && (p2pStarCurrentState != DISCONNECTED))
            {
                linkFailureCallback();
            }
            p2pStarCurrentState = DISCONNECTED;
        }
    }
}

/* Function to store the Connection Table Information which is Broadcasted by PAN Coordinator
   Used by END_DEVICES (FFD || RFD) only */
static void store_connection_tb(uint8_t *payload)
{
    uint8_t i , j ;
    for (i = 4; i < RX_BUFFER_SIZE; i+=4)
    {
        j = payload[i+3];
        if (0xFF != j)
        {
            END_DEVICES_Short_Address[j].connection_slot = j;
            END_DEVICES_Short_Address[j].Address[0] = payload[i];
            END_DEVICES_Short_Address[j].Address[1] = payload[i+1];
            END_DEVICES_Short_Address[j].Address[2] = payload[i+2];
        }
    }
    handleLostConnection();
}
#endif

#if defined(ENABLE_NETWORK_FREEZER)
bool MiApp_SubscribeReConnectionCallback(ReconnectionCallback_t callback)
{
    if (NULL != callback)
    {
        reconnectionCallback = callback;
        return true;
    }
    return false;
}
#endif

void macAckOnlyDataCallback(uint8_t handle, miwi_status_t status, uint8_t* msgPointer)
{
	P2PStarDataFrame_t *dataFramePtr = NULL;
	dataFramePtr = (P2PStarDataFrame_t *) miQueueRemove(&macAckOnlyFrameQueue, NULL);

	if (NULL != dataFramePtr)
	{
		DataConf_callback_t callback = dataFramePtr->dataFrame.confCallback;
		if (NULL != callback && 1 != dataFramePtr->dataFrame.broadcast)
		{
			callback(handle, status, msgPointer);
		}
#if defined(PROTOCOL_STAR)
		if (dataFramePtr->dataFrame.fromEDToED)
		{
			uint8_t ed_index = Find_Index(dataFramePtr->dataFrame.msg);
			if (0xFF != ed_index)
			{
				uint8_t* dataPtr;
				dataPtr = MiMem_Alloc(PACKETLEN_CMD_DATA_TO_ENDDEV_SUCCESS);
				if (NULL == dataPtr)
				return;
				dataPtr[0] = CMD_DATA_TO_ENDDEV_SUCCESS;
				frameTransmit(false, myPANID, miwiDefaultRomOrRamParams->ConnectionTable[ed_index].Address, true, true, 1, dataPtr, 0, true, CommandConfCallback);
			}
		}
#endif
		MiMem_Free((uint8_t *)dataFramePtr);
	}
}

#if defined(PROTOCOL_STAR)
void appAckWaitDataCallback(uint8_t handle, miwi_status_t status, uint8_t* msgPointer)
{
	if (PAN_COORD == role)
	{
		uint8_t loopIndex;
		P2PStarDataFrame_t *dataFramePtr = NULL;
		for (loopIndex = 0; loopIndex < appAckWaitDataQueue.size; loopIndex++)
		{
			dataFramePtr = (P2PStarDataFrame_t *) miQueueRemove(&appAckWaitDataQueue, NULL);

			if (NULL == dataFramePtr)
			{
				return;
			}
			
			if(msgPointer == (uint8_t*)&(dataFramePtr->dataFrame.msg))
			{
				if (SUCCESS == status)
				{
					uint8_t* dataPtr = NULL;
					uint8_t ed_index = Find_Index(dataFramePtr->dataFrame.msg);
					if (0xFF != ed_index)
					{
						dataPtr = MiMem_Alloc(PACKETLEN_CMD_DATA_TO_ENDDEV_SUCCESS);
						if (NULL == dataPtr)
							return;
						dataPtr[0] = CMD_DATA_TO_ENDDEV_SUCCESS;
						frameTransmit(false, myPANID, miwiDefaultRomOrRamParams->ConnectionTable[ed_index].Address, true, true, 1, dataPtr, 0, true, CommandConfCallback);
					}
				}
				MiMem_Free(dataFramePtr);
			}
			else
			{
				miQueueAppend(&appAckWaitDataQueue, (miQueueBuffer_t *)dataFramePtr);
			}
		}
	}
	else if (SUCCESS != status)
	{
		uint8_t loopIndex;
		P2PStarDataFrame_t *dataFramePtr = NULL;
		for (loopIndex = 0; loopIndex < appAckWaitDataQueue.size; loopIndex++)
		{
			dataFramePtr = (P2PStarDataFrame_t *) miQueueRemove(&appAckWaitDataQueue, NULL);

			if (NULL == dataFramePtr)
			{
				return;
			}
		
			if(handle == dataFramePtr->dataFrame.msghandle && msgPointer == (uint8_t*)&(dataFramePtr->dataFrame.msg))
			{
				DataConf_callback_t callback = dataFramePtr->dataFrame.confCallback;
				if (NULL != callback)
				{
					callback(handle, status, msgPointer);
				}
				MiMem_Free((uint8_t *)dataFramePtr);
			}
			else
			{
				miQueueAppend(&appAckWaitDataQueue, (miQueueBuffer_t *)dataFramePtr);
			}
		}
	}
}
#endif

#ifdef ENABLE_SLEEP_FEATURE
static void rfdDataWaitTimerExpired(struct SYS_Timer_t *timer)
{
    P2PStatus.bits.DataRequesting = 0;
    dataRequestInterval = RFD_WAKEUP_INTERVAL;
}
#endif

#ifdef ENABLE_FREQUENCY_AGILITY
static void channelHopCmdCallback(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
    MiMem_Free(msgPointer);

    ++freqAgilityRetries;

    /* If all the broadcasts are done, then set the optimal channel */
    if (freqAgilityRetries >= FA_BROADCAST_TIME)
    {
        MiApp_Set(CHANNEL, &optimalChannel);
        channelChangeInProgress = false;
        return;
    }

    /* Start the timer for broadcast retries */
    freqAgilityBroadcastInterval = (miwi_scan_duration_ticks(9)/1000000)+1;
}

/*********************************************************************
* static void StartChannelHopping(uint8_t OptimalChannel)
*
* Overview:        This function broadcast the channel hopping command
*                  and after that, change operating channel to the 
*                  input optimal channel     
*
* PreCondition:    Transceiver has been initialized
*
* Input:           None
*                  
* Output: 
*          None
*
* Side Effects:    The operating channel for current device will change
*                  to the specified channel
*
********************************************************************/
static void StartChannelHopping(void)
{
    uint8_t* dataPtr = NULL;
    uint8_t dataLen = 0;

    /* Prepare the Channel Hopping Message */
    dataPtr = MiMem_Alloc(PACKETLEN_CMD_CHANNEL_HOPPING);
    if (NULL == dataPtr)
       return;

    /* Prepare channel hop command */
    dataPtr[dataLen++] = CMD_CHANNEL_HOPPING;
    dataPtr[dataLen++] = currentChannel;
    dataPtr[dataLen++] = optimalChannel;

    /* Initiate the transmission */
    frameTransmit(true, myPANID, NULL, true, false, dataLen, dataPtr,0, true, channelHopCmdCallback);
}

/*******************************************************************************************
    * Function:
    *      BOOL MiApp_InitChannelHopping(uint32_t ChannelMap)
    *
    * Summary:
    *      
    *      This function tries to start a channel hopping (frequency agility) procedure
    *
    * Description:        
    *      This is the primary user interface function for the application to do energy 
    *      scan to locate the channel with least noise. If the channel is not current 
    *      operating channel, process of channel hopping will be started.
    *
    * PreCondition:    
    *      Transceiver has been initialized
    *
    * Parameters:      
    *      uint32_t ChannelMap -  The bit map of the candicate channels
    *                          which can be hopped to
    *                  
    * Returns: 
    *                  a boolean to indicate if channel hopping is initiated
    *
    * Example:
    *      <code>
    *      // if condition meets, scan all possible channels and hop 
    *      // to the one with least noise
    *      MiApp_InitChannelHopping(0xFFFFFFFF);
    *      </code>
    *
    * Remark:          The operating channel will change to the optimal 
    *                  channel with least noise
    *
    ******************************************************************************************/
bool MiApp_InitChannelHopping( INPUT uint32_t ChannelMap)
{
    uint8_t RSSIValue;
    /* Backup the channel and connection mode */
    uint8_t backupChannell = currentChannel;
    uint8_t backupConnMode = ConnMode;

    /* Disable allowing connections before NoiseDetectioScan*/
    MiApp_ConnectionMode(DISABLE_ALL_CONN);

    /* Find the optimal channel to switch using noise detection scan */
    optimalChannel = MiApp_NoiseDetection(ChannelMap, 3, NOISE_DETECT_ENERGY, &RSSIValue);

    /* Restore back the connection mode*/
    MiApp_ConnectionMode(backupConnMode);

    /* Set the original channel so that channel hop command is transmitted */
    MiApp_Set(CHANNEL, &backupChannell);

    /* If same channel, skip transmitting channel hop */
    if( optimalChannel == backupChannell )
    {
        return false;
    }

    /* Start the channel Hopping process */
    freqAgilityRetries = 0;
    channelChangeInProgress = true;
    StartChannelHopping();

    return true;
}
#endif

#ifdef ENABLE_SLEEP_FEATURE
/************************************************************************************
* Function:
* bool MiApp_ReadyToSleep(uint32_t* sleepTime)
*
* Summary:
*      This function informs whether the device can go sleep or not and how much time
*      it can sleep
*
* Description:
*      This is used to understand the stack is ready to sleep and how much time stack
*      allows to sleep if it is ready.
*
* Parameters:
*      uint32_t* sleepTime - Pointer to sleep time which specifies the sleepable time
*                            when stack is ready to sleep
*
* Returns:
*      A boolean to indicates that stack is ready to sleep or not
*
*****************************************************************************************/
bool MiApp_ReadyToSleep(uint32_t* sleepTime)
{
    if((p2pStarCurrentState == IN_NETWORK_STATE) && !(P2PStatus.bits.DataRequesting || P2PStatus.bits.RxHasUserData || (frameTxQueue.size) || (!txCallbackReceived)))
    {
        *sleepTime = dataRequestInterval * 1000;
        return true;
    }
    return false;
}
#endif

#if defined(ENABLE_NETWORK_FREEZER)
/************************************************************************************
* Function:
* bool MiApp_ResetToFactoryNew(void)
*
* Summary:
*      This function makes the device to factory new device
*
* Description:
*      This is used to erase all the persistent items in the non-volatile memory and resets the system.
*
* Returns:
*      A boolean to indicate the operation is success or not
*
*****************************************************************************************/
bool MiApp_ResetToFactoryNew(void)
{
    if (PDS_DeleteAll(true))
    {
        NVIC_SystemReset();
        return true;
    }
    else
    {
        return false;
    }
}
#endif
