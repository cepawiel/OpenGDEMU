/**
* \file  enhancedProtocol.c
*
* \brief Implementation of ehnanced protocol
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

#include <stdint.h>
#include "string.h"
#include "enhancedProtocol.h"
#include "qmm.h"
#if USE_RF == 1
#include "phy.h"
#endif
#ifdef EXT_MEMORY
  #include "extMemReader.h"
#endif
#include "atomic.h"
#include "flashLoader.h"
// #include "appEntry.h"
// #include "timerInterval.h"
#include "configuration.h"
#include "abstractSerializer.h"
#include "spiMemInterface.h"

#ifdef SAM0
#include "app_info.h"
#include <boot_version.h>
#endif


// Size of the bootloader-OTA image payload
#define  IMAGE_PAYLOAD_SIZE  64


uint32_t image_start;
uint32_t image_size;
uint32_t image_end;
uint8_t  image_crc;
uint8_t  image_dest;
uint8_t  image_type;
uint8_t  app_index;
uint8_t  is_default;

uint32_t write_address;
proto_state_t proto_state = PROTO_IDLE_STATE;
uint16_t word_buffer[ROW_SIZE << 1];

#ifdef EXT_MEMORY
  static uint8_t extMemSupported = 1;
#else
  static uint8_t extMemSupported = 0;
#endif
  
// cached copy of the APP_INFO in RAM
extern app_info_t checkapp_info;

#if (USE_IMAGE_SECURITY == 1)  
  bool imgSecSupported = true;
#else
  bool imgSecSupported = false;    
#endif  

/**************************************************************************//**
\brief Task-handler for protocol-based image download
******************************************************************************/  
static bool proto_task(void)
{
  buffer_t *event;
  uint8_t *buffer_body;
  
//  hwRestartWdt();
     
  if (serialInterface[currentInterface].getPacket != NULL)
    serialInterface[currentInterface].getPacket(&event);
    
  if (NULL != event)
  {
    buffer_body = BMM_BUFFER_POINTER((buffer_t *)event);
    
    if(PROTO_IMAGE_DATA == *buffer_body)
    {
      buffer_body++;
      memcpy(&write_address, buffer_body, 4);
      buffer_body += 4;
      memcpy(word_buffer, buffer_body, IMAGE_PAYLOAD_SIZE);
       
      if (image_dest || image_type) { // Ext mem
#ifdef EXT_MEMORY        
        memWriteData (write_address, (uint8_t *)word_buffer, IMAGE_PAYLOAD_SIZE);
#endif      
      } else {  // Int mem
        halFlashWrite (write_address, (uint8_t *)word_buffer, IMAGE_PAYLOAD_SIZE);
      }
      // Check if this is last page
      if (write_address + 64 >= image_end)
      {
        // Read from mem and calculate crc
        uint8_t crc = 0;
        uint8_t value= 0;
        uint32_t address = image_start;
        uint8_t* readBuf = (uint8_t*) word_buffer;
        uint8_t readSize = 64;
        uint8_t index = 0;
        uint32_t count = 0;
                       
        for(; address < image_end; address += 64)
        {
          // Read 64-byte chunks to word_buffer
          if (address + 64 >= image_end) 
            readSize = image_end - address;
          else
            readSize = 64;
            
          if (image_dest || image_type) {
#ifdef EXT_MEMORY            
            memReadData (address, readBuf, readSize);
#endif            
          } else { 
            memcpy (readBuf, (uint8_t*) address, readSize);
          }
          
          // Calculate running crc on the buffer
          for (index = 0; index<readSize; index++) {
            value = readBuf[index];
            crc ^= value;

            count++;
          }  
        }
       
        buffer_body = BMM_BUFFER_POINTER((buffer_t *)event);
        *buffer_body = PROTO_UPGRADE_RESPONSE;
        proto_state = PROTO_IDLE_STATE;
        
        if (image_dest || image_type) { 
#if defined(EXT_MEMORY)
          //spiMemUnInit(); 
#endif
        }

        if (image_crc == crc)
        {
          // Image successfully downloaded 
          *(buffer_body + 1) = IMAGE_CRC_SUCCESS;
          bootSetChar (2, buffer_body);
                    
          // Update the default address if image destination is int mem and it is specified as the default app
          if (is_default && !image_dest) {
            appStartAddr = image_start;
            flashUpdateAppStartAddr(); 
          }
          
          
          // Need to manually program the corresponding app structure for
          // apps downlaoded to ext mem (includes secured images)
          if (image_dest || image_type) { 
            if ((app_index != 1) || (app_index != 0))
              app_index = 0;
            
            if (!isCrcMatched) {
              // Zero-ize the app_info struct to avoid unwanted effects.
              memset (&checkapp_info, 0, sizeof (app_info_t)); 
              
              // Valid app count needed for app_info to be considered valid
              checkapp_info.app_cnt = 2; 
              
              // enabled_features mask is initialized to what is supported (boot_info)
              checkapp_info.required_features = bInfo.supported_features; 
              
            }

            checkapp_info.appTable[app_index].app_start_addr = image_start;
            checkapp_info.appTable[app_index].app_size = image_size;
            checkapp_info.appTable[app_index].existingImageInfo.memtype = image_dest;

            if (image_type) {
              // Create an Update action plan for secured image 
              checkapp_info.appTable[app_index].bootInfo.update_action = 1;
              checkapp_info.appTable[app_index].bootInfo.src_memtype = 1;
              checkapp_info.appTable[app_index].bootInfo.dest_memtype = image_dest;
              checkapp_info.appTable[app_index].bootInfo.img_type = image_type;
              checkapp_info.appTable[app_index].src_addr = image_start;  
              checkapp_info.appTable[app_index].dest_addr = image_start;
              checkapp_info.appTable[app_index].img_size = image_size;  
#if (USE_IMAGE_SECURITY == 1)
             // memset(&checkapp_info.appTable[app_index].security_key,0,AES_BLOCK_SIZE);
#endif
 }
            if (is_default) {
              checkapp_info.default_app = app_index;
            }
            writeapp_infotable ();
          }            
        }
        else
        { 
          *(buffer_body + 1) = IMAGE_CRC_ERROR;
          bootSetChar (2, buffer_body);
          
        } // (image_crc == crc)
        
        return false; 
      }
/*
      else {
        buffer_body = BMM_BUFFER_POINTER((buffer_t *)event);
        *buffer_body = PROTO_IMAGE_ACK;
        write_address += 64;
        memcpy(buffer_body + 1, (void const *) &write_address, 4);
                
        proto_state = PROTO_IMG_ACK_STATE;   
        bootSetChar (5, buffer_body);
        
      }
*/

    }
    else if(PROTO_IMAGE_CONFIRM == *buffer_body)
    {
      ImageResponse_t temp;
      ImageSupport_t  tempAck;
      
      
      buffer_body++;
      memcpy(&temp, buffer_body, sizeof(ImageResponse_t));
      image_start = temp.address;
      image_size = temp.size;
      image_crc = temp.rcvd_crc;
      image_dest = temp.img_dest;
      image_type = temp.img_type;
      app_index  = temp.app_index;
      is_default = temp.is_default;
      
      image_end = image_start + image_size;

      buffer_body = BMM_BUFFER_POINTER((buffer_t *)event);
      *buffer_body = PROTO_IMAGE_CONFIRM_ACK;
      proto_state = PROTO_IMG_CONFIRM_ACK_STATE;
      memset (&tempAck, 1, sizeof(ImageSupport_t));

      // Image_start supported
      if (image_start < (1<<bInfo.bprot_size+10)) { 
        tempAck.startAddressSupported = 0;
      }

      // Image dest_type supported
      if (!extMemSupported && image_dest) { 
        tempAck.destTypeSupported = 0;
      }  

      // Image dest_type supported
      if (!extMemSupported && image_dest) { 
        tempAck.destTypeSupported = 0;
      } 

      // Image_size supported
      if (image_end > (1<<bInfo.mcu_flash_size+10)) {
        tempAck.imageSizeSupported = 0;
      } 

      memcpy(buffer_body+1, &tempAck, sizeof(ImageSupport_t));

#ifdef EXT_MEMORY
      if (image_dest || image_type) { // Ext mem
        // SPI interface for ext mem
        spiMemInit(); 
        memPreEraseImageArea (image_start, image_size);
      }
#endif

      bootSetChar (1+sizeof(ImageSupport_t), buffer_body);

    }  //  else if(PROTO_IMAGE_CONFIRM == *buffer_body)
    
    bmm_buffer_free(event);
    
  }  // if (NULL != event)
    
  return true;
}

void enhancedProtocol (void)
{
  while (proto_task());
}

// eof enhancedProtocol.c