/**
* \file  rf_server.h
*
* \brief RF server OTAU interfaces
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
#ifndef OTAU_SERVER_H
#define OTAU_SERVER_H


#include "uartSerializer.h"

#define LEGACY_HANDSHAKE_REQ             0x4b65a5b2
#define LEGACY_HANDSHAKE_CONF            0x26d2d369
#define PROTO_HANDSHAKE_REQ              0x5c76b6c3 
#define PROTO_HANDSHAKE_CONF             0x37e3e47a 

/**
 * A UART state that expects a \ref SOT to be received as the next character.
 */
#define UART_RX_STATE_SOT               (1)

/**
 * A UART state that expects the length to be received as the next character.
 */
#define UART_RX_STATE_LENGTH            (2)

/**
 * A UART state that expects the next data character to be received.
 */
#define UART_RX_STATE_DATA              (3)

/**
 * A UART state that expects a \ref EOT to be received as the next character.
 */
#define UART_RX_STATE_EOT               (4)

/* UART communication framing */

/** The start of transmission delimiter. */
#define SOT                             (1)

/** The end of transmission delimiter. */
#define EOT                             (4)

#define PROTOCOL_ID						(0x0A)

#define UPGADE_MODE						(0x00)

#define IMAGE_DATA_RETRY_COUNT			(0x05)

// #define INTER_PKT_DELAY_US  10
#define EXT_FLASH_INTER_PKT_DELAY_US  100
#define INT_FLASH_INTER_PKT_DELAY_US   50 

#define INVALID_MSG	            0x41
#define MSG_IN_PROGRESS	        0x42
#define IMAGE_CRC_ERROR	        0x81
#define IMAGE_IMPROPER_ADDRESS	0x82
#define INVALID_CURRENT_KEY	    0x83
#define VERIFY_FLASH_FAILED	    0x84  

enum serial_msg_code {
	UPGRADE_REQUEST = 0,
	UPGRADE_REQUEST_ACK,
	IMAGE_INDICATION,
	IMAGE_RESPONSE,
	IMAGE_STATUS_INDICATION,
  IMAGE_SEND_START,
  IMAGE_ACK,
  IMAGE_NACK,
  UPGRADE_CONFIRM,
	SWITCH_IMAGE_REQUEST,
	SWITCH_IMAGE_CONFIRM,
	FORCE_IMAGE_REQUEST,
	FORCE_IMAGE_CONFIRM,
  SET_SERVER_IEEE_ADDRESS,
  SET_SERVER_IEEE_ADDRESS_ACK,
	SET_SECURITY_KEY_REQUEST,
	SET_SECURITY_KEY_CONFIRM,
	SET_KEY_REQUEST,
	SET_KEY_CONFIRM,
  IDENTIFY_BOARD_REQUEST,
	IDENTIFY_BOARD_CONFIRM,
	UPDATE_IMAGE_DETAILS_REQUEST,
	UPDATE_IMAGE_DETAILS_CONFIRM,
	ERROR_MSG
};

typedef enum otau_server_state {
	OTAU_IDLE = 0,
  OTAU_SET_ADDRESS,
	OTAU_HANDSHAKE_REQ,
	OTAU_HANDSHAKE_ACK,
	OTAU_IMG_RSP,
	OTAU_UPGRADE,
	OTAU_SWITCH_IMG,
	OTAU_FORCE_IMG,
	OTAU_SET_KEY,
	OTAU_IDENTIFY,
}otau_server_state_t;

typedef enum otau_server_upgrade_state {
	OTAU_UPGRADE_IDLE,
	OTAU_SEND_IMAGE_DATA,
	OTAU_IMAGE_DATA_RETRY,
	OTAU_UPGRADE_DONE
}otau_server_upgrade_state_t;

enum error_status{
	INVALID_CMD = 0x80,
	INVALID_MODE,
	CMD_IN_PROGRESS
};


enum rf_server_msg_code {
	OTAU_UPGRADE_REQUEST = 0,
	OTAU_UPGRADE_REQUEST_ACK,
	OTAU_IMAGE_REQUEST,
	OTAU_IMAGE_CONFIRM,
  OTAU_IMAGE_CONFIRM_ACK,
  OTAU_IMAGE_DATA,
	OTAU_UPGRADE_RESPONSE,
	OTAU_SET_KEY_REQUEST,
	OTAU_SET_KEY_CONFIRM,
	OTAU_IDENTIFY_BOARD,
	OTAU_FORCE_IMAGE,
	OTAU_BLOCK_IMAGE_REQUEST,
	OTAU_BLOCK_IMAGE_CONFIRM,
	OTAU_ERROR_MSG
};

void rf_server_init(void);
void rf_server_task(void);
void rf_server_rcvd_frame(uint8_t *src_addr, uint8_t length, uint8_t *payload);
void rf_server_sent_frame(uint8_t status);
void transmit_frame(uint8_t dst_addr_mode, uint8_t payload_length, uint8_t *payload, bool ack_req, bool secured);


#endif /* OTAU_SERVER_H */
