/**
* \file  rf_server.c
*
* \brief Implementation of RF OTAU server
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

#include "rf_server.h"

// #include "string.h"

#include "phy.h"
#include "bmm.h"
#include "qmm.h"
#include "rfInterface.h"
#include "uartSerializer.h"
#include <timerInterval.h>
// #include "at86rf233.h"

#define IMAGE_PAYLOAD_SIZE (64)

uint32_t  image_start = 0x00000000;
uint32_t  image_size  = 0x00000000;
uint32_t  image_end   = 0x00000000;
uint8_t   image_crc   = 0x00;
uint8_t   image_dest  = 0x00;
uint8_t   image_type  = 0x00;
uint8_t   app_index   = 0x00;
uint8_t   is_default   = 0x00;
uint8_t   interPktDelay = 0;
uint32_t  current_index;
uint8_t   retry_count = 0;
uint32_t  pktCount = 0; 

bool      serialRxSpinState = false; 
bool      isScanStarted = false;
uint16_t  scanInterval_ms = 100;

/**
 * This is the receive buffer of the UART.
 */
static uint8_t sio_rx_buf[BUFFER_SIZE];

/**
 * This pointer points to the next free element inside the
 * receive buffer of the UART.
 */
// static uint8_t *sio_rx_ptr;

/**
 * This is the receiver state of the UART. (\ref UART_RX_STATE_SOT,
 *   \ref UART_RX_STATE_LENGTH, \ref UART_RX_STATE_DATA, or
 *   \ref UART_RX_STATE_EOT)
 */
static volatile uint8_t sio_rx_state = UART_RX_STATE_SOT;

/**
 * This is the length of the message should be received.
 */
 static uint8_t sio_rx_length;

static uint8_t rx_index = 0;

uint8_t security_enable = false;

uint8_t SECURITY_KEY[AES_BLOCK_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// uint8_t IV[] = {0x00 , 0x01 , 0x02 , 0x03 , 0x04 , 0x05 , 0x06 , 0x07 , 0x08 , 0x09 , 0x10 , 0x11 , 0x12 , 0x13 , 0x14 , 0x15 , 0x16 , 0x17 , 0x18 , 0x19 , 0x20 , 0x21 , 0x22 , 0x23 , 0x24 , 0x25 , 0x26 , 0x27 , 0x28 , 0x29 , 0x30 , 0x31 , 0x32 , 0x33 , 0x34 , 0x35 , 0x36 , 0x37 , 0x38 , 0x39 , 0x40 , 0x41 , 0x42 , 0x43 , 0x44 , 0x45 , 0x46 , 0x47 , 0x48 , 0x49 , 0x50 , 0x51 , 0x52 , 0x53 , 0x54 , 0x55 , 0x56 , 0x57 , 0x58 , 0x59 , 0x60 , 0x61 , 0x62 , 0x63};

uint8_t otau_sent_data_index = 0;

otau_server_state_t otau_state = OTAU_IDLE;

otau_server_upgrade_state_t otau_upgrade_state = OTAU_UPGRADE_IDLE;

uint8_t otau_buff[IMAGE_PAYLOAD_SIZE+5] = {OTAU_IMAGE_DATA};

uint8_t *temp_ptr;



#ifdef _DEBUG_
uint8_t ser_cmd_buf[50] = {0};
uint8_t ota_cmd_buf[100] = {0};
uint8_t ser_cmd_index = 0;
uint8_t ota_cmd_index = 0;
#endif


// uint16_t count = 0;


static void handle_incoming_msg(void);
static void serial_data_handler(bool spinFlag);
// static void process_incoming_sio_data(void);
void reverse_memcpy(uint8_t *dst, uint8_t *src, uint8_t len);
void send_pc_data(uint8_t msg_id, uint8_t *msg, uint8_t len);
void send_image_data(uint32_t index);


void transmit_frame(uint8_t dst_addr_mode, uint8_t payload_length, uint8_t *payload, 
	bool ack_req, bool secured)
{
	uint8_t i;
	uint8_t *frame_ptr;
	uint16_t fcf = 0;
	uint16_t temp_value;
  uint8_t  tx_buf[BUFFER_SIZE];
  
	/* Get length of current frame. */
	frame_ptr = &tx_buf[0] + FCF_LEN;

	/* Set DSN. */
	*frame_ptr = seq_no++;
	frame_ptr++;

	/* Destination address */
	if (FCF_LONG_ADDR == dst_addr_mode) {
		/* Destination PAN-Id */
		temp_value = pan_id;
		convert_16_bit_to_byte_array(temp_value, frame_ptr);
		frame_ptr += PAN_ID_LEN;

		convert_64_bit_to_byte_array(client_ieee_addr, frame_ptr);
		frame_ptr += EXT_ADDR_LEN;

		fcf |= FCF_SET_DEST_ADDR_MODE(FCF_LONG_ADDR);

		/* Source address */
		convert_64_bit_to_byte_array(server_ieee_addr, frame_ptr);
		frame_ptr += EXT_ADDR_LEN;

		fcf |= FCF_SET_SOURCE_ADDR_MODE(FCF_LONG_ADDR);
	}

	else
	{
		/* Destination PAN-Id */
		temp_value = macShortAddress_def;
		convert_16_bit_to_byte_array(temp_value, frame_ptr);
		frame_ptr += PAN_ID_LEN;

		convert_16_bit_to_byte_array(macShortAddress_def,
		frame_ptr);
		frame_ptr += SHORT_ADDR_LEN;

		fcf |= FCF_SET_DEST_ADDR_MODE(FCF_SHORT_ADDR);

		/* Source address */
		convert_64_bit_to_byte_array(server_ieee_addr, frame_ptr);
		frame_ptr += EXT_ADDR_LEN;

		fcf |= FCF_SET_SOURCE_ADDR_MODE(FCF_LONG_ADDR);
	}
  
	/*
	* Payload is stored to the end of the buffer avoiding payload
	* copying by TAL.
	*/
	for (i = 0; i < payload_length; i++) {
		*frame_ptr++ = *(payload + i);
	}

	/* No source PAN-Id included, but FCF updated. */
	fcf |= FCF_PAN_ID_COMPRESSION;

	/* Set the FCF. */
	fcf |= FCF_SET_FRAMETYPE(0x07);
	
  if (ack_req) {
		fcf |= FCF_ACK_REQUEST;
	}
	
  if(security_enable & secured)
	{
		fcf |= FCF_SECURITY_ENABLED;
	}
	
  convert_16_bit_to_byte_array(fcf, tx_buf);
  PHY_TaskHandler(); 
	PHY_DataReq(tx_buf, frame_ptr - tx_buf);
    
}


void rf_server_init(void)
{
  
#if USE_USART0  
  hwInitUsart0();
#elif USE_USART1
  hwInitUsart1();
#endif  
  
	halPhyInit();
  
  PHY_Init();

  PHY_SetChannel(curr_channel);

  //PHY_SetChannelPage(curr_channel_page);

  PHY_SetIEEEAddr((uint8_t *)&server_ieee_addr);

  PHY_SetPanId(pan_id);

  PHY_SetRxState(true);

  PHY_EnableReservedFrameRx();
	
  otau_state = OTAU_IDLE;   
  serialRxSpinState = true;
  isScanStarted = false;
     
}


void rf_server_task(void)
{

	serial_data_handler(serialRxSpinState);
  serialRxSpinState = false;
  
  if (isScanStarted && (otau_state == OTAU_HANDSHAKE_REQ)) {
    // Check for scan timer expiry 
    // and send a new handshake broadcast 
    if (timerReadIntervalState()) {
      uint8_t temp[5] = {OTAU_UPGRADE_REQUEST};
			uint32_t handshake_req = PROTO_HANDSHAKE_REQ; 	
			memcpy((uint8_t*)&temp+1, &handshake_req, sizeof(uint32_t));	   
			transmit_frame(FCF_NO_ADDR, 5, (uint8_t*) temp, 0, 0);
    }
  }  
   
  PHY_TaskHandler ();
   
}


void rf_server_rcvd_frame(uint8_t *src_addr, uint8_t length, uint8_t *payload)
{

#ifdef _DEBUG_  
  ota_cmd_buf[ota_cmd_index++] = *payload;
#endif
  
  if(OTAU_UPGRADE_REQUEST_ACK == *payload)
  {
    uint32_t  handshake_conf;
    
  	memcpy(&handshake_conf, payload+1, sizeof(uint32_t));

    if (handshake_conf == PROTO_HANDSHAKE_CONF) 
    {
      buffer_t *temp_buff = bmm_buffer_alloc();
      if (NULL != temp_buff)
      {
        uint8_t *temp_ptr = BMM_BUFFER_POINTER(temp_buff);
        reverse_memcpy(temp_ptr, src_addr, 8);
        temp_ptr += 8;
        memcpy(temp_ptr, payload + 5, length - 5);

        send_pc_data(UPGRADE_REQUEST_ACK, BMM_BUFFER_POINTER(temp_buff), length + 8 - 1);
        serialRxSpinState = false;
        
        bmm_buffer_free(temp_buff);
        otau_state = OTAU_HANDSHAKE_ACK;
        
        // clear the scan client timer
        timerIntervalStop();
       // timerIntervalUnInit(); 
        isScanStarted = false;

      }
         
    }
  }

	else if(OTAU_IMAGE_REQUEST == *payload)
	{
		buffer_t *temp_buff = bmm_buffer_alloc();
		if (NULL != temp_buff)
		{
			uint8_t *temp_ptr = BMM_BUFFER_POINTER(temp_buff);
			reverse_memcpy(temp_ptr, src_addr, 8);
			temp_ptr += 8;
			memcpy(temp_ptr, payload + 1, length - 1);
			send_pc_data(IMAGE_INDICATION, BMM_BUFFER_POINTER(temp_buff), length + 8 - 1);
      serialRxSpinState = true;
      
			bmm_buffer_free(temp_buff);
		}
	}

  else if(OTAU_IMAGE_CONFIRM_ACK == *payload)
  {
    buffer_t *temp_buff = bmm_buffer_alloc();
    if (NULL != temp_buff)
    {
      uint8_t *temp_ptr = BMM_BUFFER_POINTER(temp_buff);
      reverse_memcpy(temp_ptr, src_addr, 8);
      temp_ptr += 8;
      memcpy(temp_ptr, payload + 1, length - 1);
      send_pc_data(IMAGE_STATUS_INDICATION, BMM_BUFFER_POINTER(temp_buff), length + 8 - 1);
      serialRxSpinState = true;
      
      bmm_buffer_free(temp_buff);
    }
  }

	else if(OTAU_UPGRADE_RESPONSE == *payload && OTAU_IDLE == otau_state)
	{
    uint8_t temp[5];
		if (VERIFY_FLASH_FAILED == *(payload+1)) {
		  memcpy(&current_index, payload+2, 4);
		  retry_count = 0;
		  otau_upgrade_state = OTAU_IMAGE_DATA_RETRY;
		  send_image_data(current_index);
		} else {
        temp[0] = payload[1];
        memcpy(temp+1, &pktCount, 4);
			  send_pc_data(UPGRADE_CONFIRM, temp, 5);
        
        // Finsihed transaction with this client 
        // Wait for next command from PC script
        serialRxSpinState = true;
		}
	}
}


void rf_server_sent_frame(uint8_t status)
{
//  PHY_SetRxState(true);
  
  if(OTAU_UPGRADE == otau_state)
	{
		if(PHY_STATUS_SUCCESS == status)
		{
			current_index += 64;
			if(current_index < image_end)
			{
				otau_upgrade_state = OTAU_SEND_IMAGE_DATA;
				send_image_data(current_index);	
			}
			else
			{
        // Finished sending all image packets over-the-air
				current_index = image_start;
				otau_state = OTAU_IDLE;
			}
			retry_count = 0;
		} 
		else // (PHY_STATUS_SUCCESS == status)
		{
			if(OTAU_IMAGE_DATA_RETRY != otau_upgrade_state)
			{
				retry_count = 0;
				otau_upgrade_state = OTAU_IMAGE_DATA_RETRY;
				halDelay (interPktDelay * 10);
				send_image_data(current_index);
				
			}
			else
			{
				if (retry_count == IMAGE_DATA_RETRY_COUNT)
				{
					current_index = image_start;
					send_pc_data(UPGRADE_CONFIRM, &status, 1);
					otau_state = OTAU_IDLE;
          
				}
				else
				{
					retry_count++;
					halDelay (interPktDelay * (10 << retry_count));
					send_image_data(current_index);
				}
			}
		}  // (PHY_STATUS_SUCCESS == status)
	} // if(OTAU_UPGRADE == otau_state)
	else if (OTAU_IMG_RSP == otau_state)
	{
      if(PHY_STATUS_SUCCESS != status)
      {
        retry_count++;
        if (retry_count == IMAGE_DATA_RETRY_COUNT) {
          otau_state = OTAU_IDLE;
        } else {
			    uint8_t temp[25] = {OTAU_IMAGE_CONFIRM};
        
          memcpy (&temp[1], &image_start, 4);
          memcpy (&temp[5], &image_size, 4);
          memcpy (&temp[9], &image_crc, 1);
          memcpy (&temp[10], &image_dest, 1);
          memcpy (&temp[11], &image_type, 1);
          memcpy (&temp[12], &app_index, 1);
          memcpy (&temp[13], &is_default, 1);

          transmit_frame(FCF_LONG_ADDR, 14, temp, 1, 1);

        }

      }
	}  // if (OTAU_IMG_RSP == otau_state)
	  	
	else if(OTAU_FORCE_IMG == otau_state)
	{
		send_pc_data(FORCE_IMAGE_CONFIRM, &status, 1);
		otau_state = OTAU_IDLE;
	}
	else if(OTAU_SET_KEY == otau_state)
	{
		send_pc_data(SET_SECURITY_KEY_CONFIRM, &status, 1);
		otau_state = OTAU_IDLE;
	}
	else if(OTAU_IDENTIFY == otau_state)
	{
		send_pc_data(IDENTIFY_BOARD_CONFIRM, &status, 1);
		otau_state = OTAU_IDLE;
	}

}


///**
// * @brief Process data received from SIO
// */
//static void process_incoming_sio_data(void)
//{
//	switch (sio_rx_state) {
//	case UART_RX_STATE_SOT:
//		sio_rx_ptr = sio_rx_buf;
//		if (SOT == data[rx_index]) {
//			sio_rx_state = UART_RX_STATE_LENGTH;
//		}
//
//		break;
//
//	case UART_RX_STATE_LENGTH:
//		sio_rx_length = data[rx_index];
//		if (sio_rx_length) {
//			sio_rx_state = UART_RX_STATE_DATA;
//			*sio_rx_ptr = sio_rx_length;
//			sio_rx_ptr++;
//		} else {
//			/* NULL message */
//			sio_rx_ptr = sio_rx_buf;
//			sio_rx_state = UART_RX_STATE_SOT;
//		}
//
//		break;
//
//	case UART_RX_STATE_DATA:
//		*sio_rx_ptr = data[rx_index];
//		sio_rx_ptr++;
//		sio_rx_length--;
//		if (!sio_rx_length) {
//			sio_rx_state = UART_RX_STATE_EOT;
//		}
//
//		break;
//
//	case UART_RX_STATE_EOT:
//		if (EOT == data[rx_index]) {
//			/* Message received successfully */
//			handle_incoming_msg();
//		}
//
//		/* Make rx buffer ready for next reception before handling
//		 *received data. */
//		sio_rx_ptr = sio_rx_buf;
//		sio_rx_state = UART_RX_STATE_SOT;
//		break;
//
//	default:
//		sio_rx_ptr = sio_rx_buf;
//		sio_rx_state = UART_RX_STATE_SOT;
//		break;
//	}
//}

/*
 * @brief Function to handle the state machine serial data exchange.
 */
static void serial_data_handler(bool spinFlag)
{    
  bool     result = false;
  uint8_t  rx_byte;
  uint32_t   maxPollingCount;
  bool     exitNow = false;
  
  sio_rx_state = UART_RX_STATE_SOT;
  rx_index = 0;

  do {
    if (sio_rx_state == UART_RX_STATE_SOT)
      // Checking if we got a new serial packet
      maxPollingCount = 5;
    else
      // Waiting for the next byte within the same packet
      maxPollingCount = 50;
    
    
    do {
      result = getByteUsart(&rx_byte);
      maxPollingCount--;
    } while (!result && (spinFlag || maxPollingCount));  
      
    if (result) {
      switch (sio_rx_state) {
        case UART_RX_STATE_SOT:
          if (SOT == rx_byte) {
            sio_rx_state = UART_RX_STATE_LENGTH;
          }
          else {
            // Some byte other than SOT being received first
            // Continue polling for the next repetition of the frame, 
            // if we have "spinFlag" set
            if (!spinFlag) 
              exitNow = true;
          }  
          break;

        case UART_RX_STATE_LENGTH:
          sio_rx_length = rx_byte;
          if (sio_rx_length) {
            sio_rx_state = UART_RX_STATE_DATA;
            sio_rx_buf[rx_index] = sio_rx_length;
            rx_index++;
          } else {
            exitNow = true;
          }
          break;

        case UART_RX_STATE_DATA:
          sio_rx_buf[rx_index++] = rx_byte;
          sio_rx_length--;
          if (!sio_rx_length) {
            sio_rx_state = UART_RX_STATE_EOT;
          }
          break;

        case UART_RX_STATE_EOT:
          if (EOT == rx_byte) {
            /* Message received successfully */
            handle_incoming_msg();
            exitNow = true;
          }

          /* Make rx buffer ready for next reception before handling
           *received data. */
          result = false;
          sio_rx_state = UART_RX_STATE_SOT;
          return;
          break;

        default:
          sio_rx_state = UART_RX_STATE_SOT;
          break;          
      
      }  // switch (sio_rx_state)
    } //  if (result)
    else {
      // No serial input
      exitNow = true;
    }  
      
  } while (!exitNow);  
  		  
}


/**
 * \brief Parses the Received Data in the Buffer and Process the Commands
 *accordingly.
 */
static void handle_incoming_msg(void)
{
	uint8_t temp8;
  uint16_t temp16;
	uint8_t serial_input_cmd;
	uint64_t local_ieee_addr;
	uint8_t rx_len;

/*
	if(OTAU_IDLE != otau_state)
	{
		temp = CMD_IN_PROGRESS;
		send_pc_data(ERROR_MSG, &temp, 1);
	}
*/
  
  rx_len = sio_rx_buf[0];

	if (PROTOCOL_ID != sio_rx_buf[1])
	{
		temp8 = INVALID_CMD;
		send_pc_data(ERROR_MSG, &temp8, 1);
	}
	else
	{
    serial_input_cmd = sio_rx_buf[2];
#ifdef _DEBUG_    
    ser_cmd_buf[ser_cmd_index++] = serial_input_cmd;
#endif
    
		switch (serial_input_cmd)
		{ /* message type */
                      
			case UPGRADE_REQUEST:
			{
				if (otau_state == OTAU_HANDSHAKE_ACK) {
					// Handshake already received and ACK-ed by the client
					break;
				} else {
					otau_state = OTAU_HANDSHAKE_REQ;

          // Check channel number in scan request
          if (curr_channel != sio_rx_buf[3]) {
            curr_channel = sio_rx_buf[3];
					  PHY_SetChannel(curr_channel);
          }

          // Check channel page in scan request
          if (curr_channel_page != sio_rx_buf[4]) {
            curr_channel_page = sio_rx_buf[4];
            //PHY_SetChannelPage(curr_channel_page);
          }
          
          // Check channel page in scan request
          temp16 = (sio_rx_buf[5] << 8) | sio_rx_buf[6];  
          if (scanInterval_ms != temp16) {
            scanInterval_ms = temp16;
          }

          if (rx_len > 6) {
            // Check server address in scan request
            reverse_memcpy((uint8_t*) &local_ieee_addr, sio_rx_buf+7, 8);
            if (server_ieee_addr != local_ieee_addr) {
              server_ieee_addr = local_ieee_addr;
              PHY_SetIEEEAddr ((uint8_t*)&server_ieee_addr);
            }

              send_pc_data(SET_SERVER_IEEE_ADDRESS_ACK, NULL, 0);
          }        

//					uint8_t temp[5] = {OTAU_UPGRADE_REQUEST};
//					uint32_t handshake_req = PROTO_HANDSHAKE_REQ; 	
//					memcpy((uint8_t*)&temp+1, &handshake_req, sizeof(uint32_t));	   
//					transmit_frame(FCF_NO_ADDR, 5, (uint8_t*) temp, 0, 0);
          
          if (isScanStarted == false) {
            isScanStarted = true;
            timerIntervalStop();
            //timerIntervalUnInit();  
            timerIntervalInit(((F_CPU/1000ul) / DIV_1024) * scanInterval_ms, RF_TIMER_FREQUENCY_PRESCALER);
            timerIntervalStart();
          }  
           
        } // if-else otau_state

				break;
			}
			case IMAGE_RESPONSE:
			{
				uint8_t temp[25] = {OTAU_IMAGE_CONFIRM};
				uint8_t value = 0;
				uint32_t index = 0;
				image_crc = 0;
				reverse_memcpy((uint8_t*) &client_ieee_addr, &sio_rx_buf[3], 8);
				memcpy(&image_start, &sio_rx_buf[11], 4);
				memcpy(&image_size, &sio_rx_buf[15], 4);
				memcpy(&image_dest, &sio_rx_buf[19], 1);
				memcpy(&image_type, &sio_rx_buf[20], 1);
				memcpy(&app_index, &sio_rx_buf[21], 1);
				memcpy(&is_default, &sio_rx_buf[22], 1);

				image_end = image_start + image_size;

				if (image_dest)
					interPktDelay = EXT_FLASH_INTER_PKT_DELAY_US;
				else
					interPktDelay = INT_FLASH_INTER_PKT_DELAY_US;


				for(index = image_start; index < image_end; index ++)
				{
          value = *(uint8_t*)index;				
					image_crc ^= value;
				}

				otau_state = OTAU_IMG_RSP;
				retry_count = 0;
        				
				memcpy (&temp[1], &image_start, 4);
				memcpy (&temp[5], &image_size, 4);
				memcpy (&temp[9], &image_crc, 1);
				memcpy (&temp[10], &image_dest, 1);
				memcpy (&temp[11], &image_type, 1);
				memcpy (&temp[12], &app_index, 1);
				memcpy (&temp[13], &is_default, 1);

				transmit_frame(FCF_LONG_ADDR, 14, temp, 1, 1);
				break;
			}

			case IMAGE_SEND_START:
			{
				
				reverse_memcpy((uint8_t*) &local_ieee_addr, &sio_rx_buf[3], 8);

				if (local_ieee_addr != client_ieee_addr)
				{
				  break;
				}

				current_index = image_start;
        pktCount = 0;
				if(0 != current_index)
				{
					otau_upgrade_state = OTAU_SEND_IMAGE_DATA;
					send_image_data(current_index);
					
				}
				break;
			}
			
			case FORCE_IMAGE_REQUEST:
			{
				otau_state = OTAU_FORCE_IMG;
				uint8_t temp = OTAU_FORCE_IMAGE;
				transmit_frame(FCF_NO_ADDR, 1, &temp, 0, 1);
				break;
			}
			case SET_SECURITY_KEY_REQUEST:
			{
				otau_state = OTAU_SET_KEY;
				reverse_memcpy((uint8_t*) &local_ieee_addr, &sio_rx_buf[3], 8);
				uint8_t temp = OTAU_SET_KEY_REQUEST;
				transmit_frame(FCF_LONG_ADDR, 1, &temp, 1, 1);
				break;
			}
			case IDENTIFY_BOARD_REQUEST:
			{
				otau_state = OTAU_IDENTIFY;
				reverse_memcpy((uint8_t*)&local_ieee_addr, &sio_rx_buf[3], 8);
				uint8_t temp = OTAU_IDENTIFY_BOARD;
				transmit_frame(FCF_LONG_ADDR, 1, &temp, 1, 1);
				break;
			}
			case SET_KEY_REQUEST:
			{
				uint8_t status = PHY_STATUS_SUCCESS;
				if(sio_rx_buf[3])
				{
					memcpy(SECURITY_KEY, &sio_rx_buf[4], 16);
					security_enable = true;
				}
				else
				{
					security_enable = false;
				}
				send_pc_data(SET_KEY_CONFIRM, &status, 1);
				break;
			}
			case UPDATE_IMAGE_DETAILS_REQUEST:
			{
				uint8_t status = PHY_STATUS_SUCCESS;
				uint8_t value = 0;
				uint32_t index = 0;
				image_crc = 0;
				memcpy(&image_start, &sio_rx_buf[3], 4);
				memcpy(&image_size, &sio_rx_buf[7], 4);
				image_end = image_start + image_size;
				
				for(index = image_start; index < image_end; index ++)
				{
          value = *(uint8_t*)index;	          
					image_crc ^= value;
				}
				
				send_pc_data(UPDATE_IMAGE_DETAILS_CONFIRM, &status, 1);
			}
			default:
			{
				/* Do nothing...*/
			}
		}
	}
}

void send_pc_data(uint8_t msg_id, uint8_t *msg, uint8_t len)
{
  uint8_t  data;
  data = SOT;
	setByteUsart(1, &data);
  data = len + 2;
	setByteUsart(1, &data);
  data = PROTOCOL_ID;
	setByteUsart(1, &data);
  data = msg_id;
	setByteUsart(1, &data);
	setByteUsart(len, msg);
  data = EOT;
	setByteUsart(1, &data);
}

void send_image_data(uint32_t index)
{
  otau_state = OTAU_UPGRADE;
	otau_buff[0] = OTAU_IMAGE_DATA;
	memcpy(&otau_buff[1], (uint8_t *)&index, 4);
  memcpy(&otau_buff[5], (uint8_t *) index, IMAGE_PAYLOAD_SIZE);
	pktCount++;
  halDelay (interPktDelay);
	transmit_frame(FCF_LONG_ADDR, IMAGE_PAYLOAD_SIZE+5, otau_buff, 1, 1);
}


