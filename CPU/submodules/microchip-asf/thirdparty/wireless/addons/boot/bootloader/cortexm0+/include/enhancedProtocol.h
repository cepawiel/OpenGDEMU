/**
* \file  enhancedProtocol.h
*
* \brief Interface of Enhanced Protocol
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

#ifndef OTAU_CLIENT_H
#define OTAU_CLIENT_H

#include "gpio.h"

// Image download status
#define INVALID_MSG	            0x41
#define MSG_IN_PROGRESS	        0x42
#define IMAGE_CRC_SUCCESS       0x00
#define IMAGE_CRC_ERROR	        0x81
#define IMAGE_IMPROPER_ADDRESS	0x82
#define INVALID_CURRENT_KEY	    0x83
 

#define SECURITY_INDEX  (1)

#define FIRMWARE_VERSION_INDEX  (2)
#define FIRMWARE_VERSION        ("X.X.X")
#define FIRMWARE_VERSION_SIZE   (5)

#define FIRWARE_NAME_INDEX      (FIRMWARE_VERSION_INDEX + FIRMWARE_VERSION_SIZE + 1)
#define FIRMWARE_NAME	        ("No Valid Application")
#define FIRWARE_NAME_SIZE       (sizeof(FIRMWARE_NAME))

// Message/Command IDs for OTA server-client communication
enum proto_msg_code {
	PROTO_UPGRADE_REQUEST = 0,
  PROTO_UPGRADE_REQUEST_ACK,
	PROTO_IMAGE_REQUEST,
	PROTO_IMAGE_CONFIRM,
  PROTO_IMAGE_CONFIRM_ACK,
	PROTO_IMAGE_DATA,
	PROTO_UPGRADE_RESPONSE,
	PROTO_SET_KEY_REQUEST,
	PROTO_SET_KEY_CONFIRM,
	PROTO_ERROR_MSG
};

// Bootloader state machine during OTA image download
typedef enum proto_state_tag {
	PROTO_IDLE_STATE = 0,
	PROTO_HANDSHAKE_CONF_STATE,
	PROTO_IMG_REQUEST_STATE,
  PROTO_IMG_CONFIRM_ACK_STATE,
	PROTO_IMG_ACK_STATE,
	PROTO_SET_KEY_STATE
} proto_state_t;

// Image parameters shared by the server during handshake
typedef struct
{
  uint32_t              address;
  uint32_t              size;
  uint8_t               rcvd_crc;
  uint8_t               img_dest;
  uint8_t               img_type;
  uint8_t               app_index;
  uint8_t               is_default;
} ImageResponse_t;

// Image parameters supported by the client node
typedef struct
{
  uint8_t              startAddressSupported;
  uint8_t              destTypeSupported;
  uint8_t              securitySupported;
  uint8_t              imageSizeSupported;
} ImageSupport_t;


extern proto_state_t proto_state;

void enhancedProtocol (void);


#endif /* OTAU_CLIENT_H */