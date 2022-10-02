/**
* \file  rfInterface.h
*
* \brief RF OTAU interfaces
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
#ifndef OTAU_H
#define OTAU_H

#if (USE_RF == 1)
#include <stdint.h>
#include <stdbool.h>
#include "bmm.h"
#include "qmm.h"

// Default 15.4 MAC address for the bootloader client if it not available in user-page
#define CLIENT_IEEE_ADDRESS     (0x8877665544332211)   

#define CHANNEL_NUMBER     13    // Build-time defined channel for the bootloader 
#define CHANNEL_PAGE       0     // Build-time defined page for the bootloader 

/**
 * The mask for the ACK request bit of the FCF
 */
#define FCF_ACK_REQUEST                 (1 << 5)

/**
 * The mask for the PAN ID compression bit of the FCF
 */
#define FCF_PAN_ID_COMPRESSION          (1 << 6)

/**
 * Address Mode: NO ADDRESS
 */
#define FCF_NO_ADDR                     (0x00)

/**
 * Address Mode: RESERVED
 */
#define FCF_RESERVED_ADDR               (0x01)

/**
 * Address Mode: SHORT
 */
#define FCF_SHORT_ADDR                  (0x02)

/**
 * Address Mode: LONG
 */
#define FCF_LONG_ADDR                   (0x03)

/**
 * Defines the offset of the destination address
 */
#define FCF_DEST_ADDR_OFFSET            (10)

/**
 * Defines the offset of the source address
 */
#define FCF_SOURCE_ADDR_OFFSET          (14)

/**
 * Macro to set the source address mode
 */
#define FCF_SET_SOURCE_ADDR_MODE(x)     ((unsigned int)((x) << \
	FCF_SOURCE_ADDR_OFFSET))

/**
 * Macro to set the destination address mode
 */
#define FCF_SET_DEST_ADDR_MODE(x)       ((unsigned int)((x) << \
	FCF_DEST_ADDR_OFFSET))

/**
 * Defines a mask for the frame type. (Table 65 IEEE 802.15.4 Specification)
 */
#define FCF_FRAMETYPE_MASK              (0x07)

/**
 * A macro to set the frame type.
 */
#define FCF_SET_FRAMETYPE(x)            (x)

/**
 * The mask for the security enable bit of the FCF.
 */
#define FCF_SECURITY_ENABLED            (1 << 3)

/**
 * Macro to get the frame type.
 */
#define FCF_GET_FRAMETYPE(x)            ((x) & FCF_FRAMETYPE_MASK)

/*
 * Defines the mask for the FCF address mode
 */
#define FCF_ADDR_MASK                   (3)

/*
 * Macro to get the source address mode.
 */
#define FCF_GET_SOURCE_ADDR_MODE(x) \
	(((x) >> FCF_SOURCE_ADDR_OFFSET) & FCF_ADDR_MASK)

/*
 * Macro to get the destination address mode.
 */
#define FCF_GET_DEST_ADDR_MODE(x) \
	(((x) >> FCF_DEST_ADDR_OFFSET) & FCF_ADDR_MASK)
/**
 * Length (in octets) of FCF
 */
#define FCF_LEN                             (2)

/**
 * Length (in octets) of FCS
 */
#define FCS_LEN                             (2)

/**
 * Length of the sequence number field
 */
#define SEQ_NUM_LEN                         (1)

/**
 * Length (in octets) of extended address
 */
#define EXT_ADDR_LEN                        (8)

/**
 * Length (in octets) of short address
 */
#define SHORT_ADDR_LEN                      (2)

/**
 * Length (in octets) of PAN ID
 */
#define PAN_ID_LEN                          (2)

/**
 * Default value for PIB macShortAddress
 */

#define macShortAddress_def             (0xFFFF)

#define FRAME_OVERHEAD    (3)

#define BUFFER_SIZE (140)    
    
#pragma pack (1)
typedef struct
{
  uint16_t  fcf;
  uint8_t   seq;
  uint16_t  dst_panid;
  uint64_t  dst_addr;
  uint64_t  src_addr;
  uint8_t   payload[100];  
  uint8_t   len;
} frame_t;
#pragma pack ()
    
// RF interface parameters
extern  uint64_t server_ieee_addr;
extern  uint64_t client_ieee_addr;
extern  uint64_t rx_ieee_addr;
extern  uint8_t  curr_channel;
extern  uint8_t  curr_channel_page;
extern  uint16_t pan_id;
extern  uint8_t seq_no; 
extern uint8_t   curr_channel;
extern uint8_t   curr_channel_page;
extern uint64_t  server_ieee_addr;
extern uint64_t  client_ieee_addr;

#if !RF_SERVER_MODE
  extern queue_t data_from_server_q;
#endif
       

static uint16_t convert_byte_array_to_16_bit(uint8_t *data)
{
    return (data[0] | ((uint16_t)data[1] << 8));
}

static void convert_16_bit_to_byte_array(uint16_t value, uint8_t *data)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
}

static void convert_64_bit_to_byte_array(uint64_t value, uint8_t *data)
{
    uint8_t index = 0;

    while (index < 8)
    {
        data[index++] = value & 0xFF;
        value = value >> 8;
    }
}


#if !RF_SERVER_MODE
void rf_client_init(void);
void rf_client_sent_frame(uint8_t status);
void rf_client_rcvd_frame(uint8_t *src_addr, uint8_t length, uint8_t *payload);
#endif

void txDataRf (uint16_t len, uint8_t *p);
bool rxDataRf (buffer_t **p);



void reverse_memcpy(uint8_t *dst, uint8_t *src, uint8_t len);

#endif

#endif /* OTAU_H */
