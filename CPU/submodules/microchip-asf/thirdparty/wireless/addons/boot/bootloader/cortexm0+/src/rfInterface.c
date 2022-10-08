/**
* \file  rfInterface.c
*
* \brief Implementation of RF OTAU API
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

#if (USE_RF == 1)
#include "string.h"
#include "rfInterface.h"
#include "phy.h"
#include "at86rf233.h"
#include "abstractSerializer.h"
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
#include <atsamr21.h>
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#include <atsamr30.h>
#endif
#include "extMemReader.h"
#include "atomic.h"
#include "flashLoader.h"
#include "appEntry.h"
#include "timerInterval.h"
#include "configuration.h"
#if RF_SERVER_MODE
#include "rf_server.h"
#endif


/*Buffer for Transmit Payload*/ 
// static uint8_t storage_buffer[BUFFER_SIZE];

uint8_t  curr_channel = CHANNEL_NUMBER;
uint8_t  curr_channel_page = CHANNEL_PAGE;

uint64_t server_ieee_addr;
uint64_t client_ieee_addr = CLIENT_IEEE_ADDRESS;

uint64_t rx_ieee_addr;
uint16_t pan_id = 0xBABE;
uint8_t seq_no = 0;

#if !RF_SERVER_MODE
  queue_t data_from_server_q;
#endif

uint32_t img_req_delay_us = 9000;
extern app_info_t checkapp_info;

void reverse_memcpy(uint8_t *dst, uint8_t *src, uint8_t len)
{
	uint8_t i;
	for (i=0; i < len; ++i)
	{
		dst[len-1-i] = src[i];
	}
}

void PHY_DataConf(uint8_t status)
{
#if RF_SERVER_MODE
  rf_server_sent_frame(status);
#else  
  rf_client_sent_frame(status);
#endif
}


void PHY_DataInd(PHY_DataInd_t *ind)
{
	uint8_t *frame_ptr = ind->data;
	uint8_t src_addr_mode;
	uint8_t dst_addr_mode;
	bool intra_pan;
	uint8_t addr_field_len = FRAME_OVERHEAD;
	uint16_t fcf;
	uint16_t frame_type;
	uint8_t *src_addr;

	fcf = convert_byte_array_to_16_bit(frame_ptr);
	
	src_addr_mode = FCF_GET_SOURCE_ADDR_MODE(fcf);
	dst_addr_mode = FCF_GET_DEST_ADDR_MODE(fcf);
	intra_pan = fcf & FCF_PAN_ID_COMPRESSION;

	frame_type = FCF_GET_FRAMETYPE(fcf);

	if(0x0007 == frame_type)
	{
		if (0 != dst_addr_mode)
		{
			addr_field_len += PAN_ID_LEN;
			if(FCF_SHORT_ADDR == dst_addr_mode)
			{
				addr_field_len += SHORT_ADDR_LEN;
			}
			else if (FCF_LONG_ADDR == dst_addr_mode)
			{
				addr_field_len += EXT_ADDR_LEN;
			}
		}
		if (0 != src_addr_mode) {
			if (!intra_pan) {
				addr_field_len += PAN_ID_LEN;
				} else {
			}
			src_addr = frame_ptr + addr_field_len;
			if (FCF_SHORT_ADDR == src_addr_mode) {
				addr_field_len += SHORT_ADDR_LEN;
				} else if (FCF_LONG_ADDR == src_addr_mode) {
				addr_field_len += EXT_ADDR_LEN;
			}
		}
		else
		{
			src_addr = NULL;
		}
#if RF_SERVER_MODE
		 rf_server_rcvd_frame(src_addr, ind->size - addr_field_len, frame_ptr + addr_field_len);
#else
     rf_client_rcvd_frame(src_addr, ind->size - addr_field_len, frame_ptr + addr_field_len);
#endif    
	}
}


#if !RF_SERVER_MODE
void txDataRf (uint16_t len, uint8_t *p)
{
  
   frame_t frame_ptr;
	 frame_ptr.seq =seq_no++;

   frame_ptr.dst_panid =  pan_id;
   frame_ptr.dst_addr = server_ieee_addr;
   frame_ptr.src_addr = client_ieee_addr;
   
   frame_ptr.fcf = 0xcc67;
   memcpy(frame_ptr.payload, p, len);
   frame_ptr.len=21+len;
   PHY_TaskHandler();      
	 PHY_DataReq((uint8_t *)&(frame_ptr), frame_ptr.len);
   
}


bool rxDataRf (buffer_t **p)
{
  PHY_TaskHandler();
   

  *p = qmm_queue_remove (&data_from_server_q); 
    
  
  if (*p != NULL)
    return true;
  else
    return false;

}



void rf_client_init(void)
{
  
  uint8_t xtal_trim_val;
    
  
  memcpy((uint8_t *)&client_ieee_addr,(uint8_t *)(USER_PAGE_ADDRESS + IEEE_ADDR_OFFSET), 8);
  if(0x0000000000000000 == client_ieee_addr || 0xFFFFFFFFFFFFFFFF == client_ieee_addr)
  {
    client_ieee_addr = CLIENT_IEEE_ADDRESS;
  }
  
 // hwStopWdt();
  halPhyInit();
  
  PHY_Init();
  reverse_memcpy((uint8_t *)&xtal_trim_val,(uint8_t *)(USER_PAGE_ADDRESS + XTAL_TRIM_OFFSET), 1);
  PHY_SetXtaltrim(xtal_trim_val);
  PHY_SetChannel(curr_channel);

  //PHY_SetChannelPage(curr_channel_page);

  PHY_SetIEEEAddr((uint8_t *)&client_ieee_addr);

  PHY_SetPanId(pan_id);

  PHY_SetRxState(true);

  PHY_EnableReservedFrameRx();

  qmm_queue_init(&data_from_server_q);
    
  
}

void rf_client_rcvd_frame(uint8_t *src_addr, uint8_t length, uint8_t *payload)
{
  buffer_t *temp_buff = bmm_buffer_alloc();
  if (NULL != temp_buff)
  {
    memcpy(&rx_ieee_addr, src_addr, 8);
    memcpy(BMM_BUFFER_POINTER(temp_buff), payload, length);
    qmm_queue_append(&data_from_server_q, temp_buff);
  }
}

void rf_client_sent_frame(uint8_t status)
{
  buffer_t *buffer;
  uint8_t len;
  uint8_t *buffer_body;
   
  if (PROTO_IMG_REQUEST_STATE == proto_state) 
  {
    if(PHY_STATUS_SUCCESS != status)
    {  
      halDelay (img_req_delay_us);
      proto_state = PROTO_HANDSHAKE_CONF_STATE;
      status = PHY_STATUS_SUCCESS;
    }
  }
    
  if (PROTO_HANDSHAKE_CONF_STATE == proto_state) 
  {
    buffer = bmm_buffer_alloc();
    buffer_body = BMM_BUFFER_POINTER((buffer_t *)buffer);
    
    if(PHY_STATUS_SUCCESS == status)
		{
      // As soon as handshake response is successfully sent,
      // transmit the image request
      
      if (NULL != buffer)
      {        
        // Send image_request with app_info
        *buffer_body = PROTO_IMAGE_REQUEST;
        len=1;
        // Append boot_info and app_info_common
        *(buffer_body+len) = 1;  // boot_info valid
        len++;  
        memcpy(buffer_body+len, (uint8_t*) &bInfo, sizeof(bInfo)); 
        len += sizeof(bInfo);
        *(buffer_body+len) = isCrcMatched;  // app_info valid
        
        if (isCrcMatched)
        {
          memcpy(buffer_body+len, (uint8_t*) &checkapp_info, 5); 
          len += 5;
        }
          
        buffer_body = BMM_BUFFER_POINTER((buffer_t *)buffer);
        halDelay (img_req_delay_us);
        bootSetChar(len, buffer_body);
                
        bmm_buffer_free(buffer);
      }
      
      proto_state = PROTO_IMG_REQUEST_STATE;
    }  // if(PHY_STATUS_SUCCESS == status)
  } // if (PROTO_HANDSHAKE_CONF_STATE == proto_state) 
 
}
#endif  // if !RF_SERVER_MODE

#endif // USE_RF == 1
