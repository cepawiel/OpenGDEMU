/**
* \file  miwi_p2p_star.h
*
* \brief MiWi P2P Star header defintions.
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

#ifndef __MIWI_P2P_STAR_H_
#define __MIWI_P2P_STAR_H_

#define INPUT
#define OUTPUT
#define IOPUT

#include "system.h"
#include "miwi_config.h"          //MiWi Application layer configuration file
#include "miwi_config_p2p.h"      //MiWi Protocol layer configuration file

/************************ HEADERS **********************************/
#include "sysTimer.h"
#include "miwi_api.h"


/************************ DEFINITIONS ******************************/

#define PAYLOAD_START                           0

#define STATUS_SUCCESS                          0x00
#define STATUS_EXISTS                           0x01
#define STATUS_ACTIVE_SCAN                      0x02
#define STATUS_ENTRY_NOT_EXIST                  0xF0
#define STATUS_NOT_ENOUGH_SPACE                 0xF1
#define STATUS_NOT_SAME_PAN                     0xF2
#define STATUS_NOT_PERMITTED                    0xF3

#define CMD_P2P_CONNECTION_REQUEST              0x81
#define CMD_P2P_CONNECTION_REMOVAL_REQUEST      0x82
#define CMD_DATA_REQUEST                        0x83
#define CMD_CHANNEL_HOPPING                     0x84
#define CMD_TIME_SYNCHRONIZATION_REQUEST        0x85
#define CMD_TIME_SYNCHRONIZATION_NOTIFICATION   0x86
#define CMD_P2P_ACTIVE_SCAN_REQUEST             0x87

#define CMD_TIME_SYNC_DATA_PACKET               0x8A
#define CMD_TIME_SYNC_COMMAND_PACKET            0x8B

#define CMD_P2P_CONNECTION_RESPONSE             0x91
#define CMD_P2P_CONNECTION_REMOVAL_RESPONSE     0x92
#define CMD_P2P_ACTIVE_SCAN_RESPONSE            0x97

#define CMD_MAC_DATA_REQUEST                    0x04

#define PACKETLEN_P2P_ACTIVE_SCAN_RESPONSE             (2 + ADDITIONAL_NODE_ID_SIZE)
#define PACKETLEN_P2P_CONNECTION_REMOVAL_RESPONSE       2
#define PACKETLEN_TIME_SYNC_DATA_PACKET                 TX_BUFFER_SIZE
#define PACKETLEN_MAC_DATA_REQUEST                      TX_BUFFER_SIZE
#define PACKETLEN_P2P_CONNECTION_REMOVAL_REQUEST        1
#define PACKETLEN_CMD_IAM_ALIVE                         1
#define PACKETLEN_CMD_DATA_TO_ENDDEV_SUCCESS            1
#define PACKETLEN_P2P_CONNECTION_REQUEST               (4 + ADDITIONAL_NODE_ID_SIZE)
#define PACKETLEN_P2P_ACTIVE_SCAN_REQUEST               2
#define PACKETLEN_CMD_CHANNEL_HOPPING                   3

#if defined (PROTOCOL_STAR)
// END_device uses this command to denote PAN COR
// that the data enclosed in packet is to be forwarded
// to another END_Device in network
#define CMD_FORWRD_PACKET 0xCC
// PAN COR will send this command to denote Packet Forward Success ,
// SW generated ACK
# define CMD_DATA_TO_ENDDEV_SUCCESS 0xDA
// Used by END Devices to Send Link Status
#define CMD_IAM_ALIVE  0x7A
// Used by END Devices  to qualify them as permanent forever in Network Table
#define CMD_MAKE_CONNECTION_ENTRY_PERMENANT  0x3A
// Used by PAN COR to Share Connection Table Information with Peer END Devices
#define CMD_SHARE_CONNECTION_TABLE              0x77
#endif

#if defined(ENABLE_ED_SCAN) && defined(ENABLE_FREQUENCY_AGILITY)
#define FREQUENCY_AGILITY_STARTER
#endif

#define MICRO_SECOND_PER_COUNTER_TICK   (1000000ul / ( COUNTER_CRYSTAL_FREQ / 8 ))

/******************************************************************
 * Overview: structure to indicate the status of P2P stack
 *****************************************************************/
typedef union
{
    uint8_t Val;                               // The value of the P2P status flags
    struct 
    {
        uint8_t Sleeping               :1;     // indicate if the device in sleeping state
        uint8_t SaveConnection         :1;     // indicate if it is time for Network Freezer module to
                                            // save valid connections. Important because there may be
                                            // more than one response coming back and NVM operation is 
                                            // blocking
        uint8_t DataRequesting         :1;     // indicate that device is in the process of data request
                                            // from its parent. Only effective if device enables sleeping
        uint8_t RxHasUserData          :1;     // indicate if the received frame needs processing from
                                            // the application layer   
        uint8_t SearchConnection       :1;     // indicate if the stack is currently in the process of
                                            // looking for new connection
        uint8_t Resync                 :1;     // indicate if the stack is currently in the process of
                                            // resynchronizing connection with the peer device
        uint8_t Enhanced_DR_SecEn      :1;
    }bits;                                  // bit map of the P2P status
} P2P_STATUS;                               

// Number of octets added by the PHY: 4 sync octets + SFD octet
#define PHY_OVERHEAD                    (5)

//Number of octets added by MAC- 2 FrameCoontrol, 1-SeqNo,2- SrcPanId,
// 2 -DstPanId 8-src address, 8- dst address, */
#define MAC_OVERHEAD                    (23)

// The maximum size of an MPDU, in octets, that can be followed by a SIFS period
#define aMaxSIFSFrameSize               (18)

// Default value for PIB macMinLIFSPeriod
#define macMinLIFSPeriod_def            (40)

// Default value for PIB macMinSIFSPeriod
#define macMinSIFSPeriod_def            (12)

// Maximum turnaround Time of the radio to switch from Rx to Tx or Tx to Rx in symbols
#define aTurnaroundTime                 (12)

// The number of symbols forming the basic time period used by the CSMA-CA algorithm
#define aUnitBackoffPeriod              (20)

#define MAX_SCAN_DURATION           14

#if defined(PROTOCOL_STAR)
#define SHARE_PEER_DEVICE_INFO_TIMEOUT      15
#define LINK_STATUS_TIMEOUT                 15
#define SW_ACK_TIMEOUT                      2
// every 1 minute / 60 seconds the stack will evaluate the inactive end nodes
#define FIND_INACTIVE_DEVICE_TIMEOUT        60
#define END_DEVICES_DISPLAY_TIMEOUT         1000*15

#endif

#define MAX_PAYLOAD                 76

/************************ Type Definitions **************************************/
/* State of the P2PSTAR stack */
typedef enum meshState_
{
    /* State during boot condition */
    INITIAL_STATE,
    /* State where Mesh protocol Init is completed */
    INIT_STATE,
    /* State where Network Start operation is in progress */
    STARTING_NETWORK,
    /* State where device is searching for network to join */
    SEARCHING_NETWORK,
    /* State where device is connecting with a device/parent */
    ESTABLISHING_NETWORK,
    /* State where search operation is complete */
    SEARCH_COMPLETE,
    /* PAN Coordinator successfully started in a network */
    PANC_IN_NETWORK_STATE,
    /* Device successfully started or joined into a network */
    IN_NETWORK_STATE,
    /* Device is disconnected from network - applicable only for End devices*/
    DISCONNECTED,
    /* Device is resynching */
    RESYNC_IN_PROGRESS
}p2pStarState_t;


typedef struct _DataFrame_t
{
	DataConf_callback_t confCallback;
	uint8_t destAddress[MY_ADDRESS_LENGTH];
	uint8_t timeout;
	uint8_t ackReq;
	uint8_t broadcast;
	uint8_t fromEDToED;
	uint8_t msghandle;
	uint8_t msgLength;
	uint8_t msg[MAX_PAYLOAD + 4]; // +4 to support packet forward header
} DataFrame_t;

typedef struct _P2PStarDataFrame_t
{
	DataFrame_t *nextFrame;
	DataFrame_t dataFrame;
} P2PStarDataFrame_t;

/************************ FUNCTION PROTOTYPES **********************/
bool    isSameAddress(INPUT uint8_t *Address1, INPUT uint8_t *Address2);

#endif //__MIWI_P2P_STAR_H_