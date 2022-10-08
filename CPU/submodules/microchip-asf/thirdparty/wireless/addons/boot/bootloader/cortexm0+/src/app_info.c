/**
* \file  app_info.c
*
* \brief Implementation of app_info table handling .
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
/******************************************************************************
                   Includes section
******************************************************************************/

#include <types.h>
#include <app_info.h>
#include <bootStructure.h>
#include <appEntry.h>
#include <extMemReader.h>
#include <types.h>
#include <bootStructure.h>
#include <phy.h>
#include <imgSecurity.h>
#include <eepromLoader.h>
#include <spiMemInterface.h>
#include <flashLoader.h>
#include <at86rf233.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
#define APP_INFO_START_ADDRESS 0x3ff00

void extReadDataForFlashArea(uint32_t ext_addr,uint32_t int_addr,uint32_t size,
                              uint8_t img_type,uint8_t *key,uint8_t app_index);

/******************************************************************************
                   External global variables section
******************************************************************************/
uint16_t otauHdrImgType;
uint32_t otauFileLen;
uint32_t otauFileHdrLen;
uint32_t payloadOffset;
uint32_t payloadLen;
uint32_t otauFileId;
uint32_t alignedPayloadLen;
uint16_t remainderData;
#if (USE_IMAGE_SECURITY==1)
uint8_t  data[AES_BLOCKSIZE];                 // Payload to the AES engine
extern uint8_t dataMinusOne[AES_BLOCKSIZE];   // AES Key
#endif
extern BootBuffer_t dataBuffer;
#ifdef EXT_MEMORY
skipStatus_t skipStatus;
static uint32_t skipStartAddress;
static uint32_t skipEndAddress;
#endif
#if (USE_IMAGE_SECURITY==1)

#endif

app_info_t checkapp_info;

// Note: isCrcMatched is used to check if app_info table (read into "checkapp_info") is valid. 
// It is considered invalid if the app_cnt field is other than 1 or 2
bool isCrcMatched = false;
/******************************************************************************
                   Implementations section
******************************************************************************/

/**************************************************************************\\**
\brief Read the app_info and process it for any pending action
******************************************************************************/
void app_infoAction(void)
{
  isCrcMatched = readapp_infotable();
  if(isCrcMatched )
  {
    update_action();
  }
}

/*************************************************************************************\\**
\brief Routine to read the app_info from flash and validate the CRC and the app_count
**************************************************************************************/
bool readapp_infotable(void)
{ 
  uint32_t app_info_addr = APP_INFO_START_ADDRESS;
   uint8_t calculated_crc=0;
   memcpy(&checkapp_info,(uint32_t *)app_info_addr,sizeof(app_info_t));
   if ((checkapp_info.app_cnt <= 2) && (checkapp_info.app_cnt !=0))
   {
     calculated_crc = MemcalcCrc(calculated_crc,(uint8_t *)&(checkapp_info.default_app),
                                                                sizeof(app_info_t)-1);
     if(checkapp_info.crc == calculated_crc)
      return true;
   }
   return false;

}

/******************************************************************************************\\**
\brief Routine to commit the given contents of app_info to flash with updated CRC; 
       Also reads it back in RAM-cache
*******************************************************************************************/
void writeapp_infotable(void)
{
  uint32_t app_info_addr = APP_INFO_START_ADDRESS;
  uint8_t calculated_crc;
  calculated_crc = MemcalcCrc(0,(uint8_t *)&(checkapp_info.default_app),sizeof(checkapp_info)-1);  
  checkapp_info.crc = calculated_crc;  
  halFlashWrite (app_info_addr, (uint8_t *)&checkapp_info, sizeof(checkapp_info));
  isCrcMatched=readapp_infotable();
  return;
}

/******************************************************************************************\\**
\brief Routine to check for pending action for image update
*******************************************************************************************/
void  update_action(void)
{
  /*if crc valid call update action.. */   
  for (uint8_t i=0;i<checkapp_info.app_cnt;i++)
  {
    if(checkapp_info.appTable[i].bootInfo.update_action==1)
    {
      memcopy(checkapp_info.appTable[i].bootInfo.dest_memtype,
              checkapp_info.appTable[i].dest_addr,
              checkapp_info.appTable[i].bootInfo.src_memtype,
              checkapp_info.appTable[i].src_addr,
              checkapp_info.appTable[i].bootInfo.img_type,
              checkapp_info.appTable[i].img_size,i);
      clear_updateaction(i);
      break;
    }
  }
}

/******************************************************************************************\\**
\brief Routine for post-update initialization and cleanup  
*******************************************************************************************/
void clear_updateaction(uint8_t app_index)
{ if(checkapp_info.appTable[app_index].bootInfo.dest_memtype == TYPE_INTERNAL)
  {
  
    checkapp_info.appTable[app_index].app_start_addr=checkapp_info.appTable[app_index].dest_addr;
    checkapp_info.appTable[app_index].existingImageInfo.memtype = checkapp_info.appTable[app_index].bootInfo.dest_memtype;
    checkapp_info.appTable[app_index].app_size= checkapp_info.appTable[app_index].img_size;
    checkapp_info.appTable[app_index].bootInfo.update_action=0;  
    writeapp_infotable();
    
    if (checkapp_info.default_app == app_index) {
      appStartAddr = checkapp_info.appTable[app_index].app_start_addr;
      flashUpdateAppStartAddr();   
    }
  }
   
}

/******************************************************************************************\\**
\brief Routine for mem-transfer across internal and external flashes during update   
*******************************************************************************************/
void memcopy(uint8_t dst_mem,uint32_t dst_addr,uint8_t src_mem,uint32_t src_addr,uint8_t img_type,uint32_t img_size,uint8_t app_index)
{
#ifdef EXT_MEMORY
  if(src_mem== TYPE_EXTERNAL && dst_mem == TYPE_INTERNAL)
  {
     spiMemInit();
     if (false == memCheckMem())
     {
       return;
     }
     extReadDataForFlashArea(src_addr,dst_addr,img_size,img_type,(uint8_t *)&checkapp_info.appTable[app_index].security_key,app_index);
  }
  else if(src_mem==TYPE_INTERNAL && dst_mem==TYPE_INTERNAL)
#endif
  /* Unsecured image*/
  {
    if(0 != checkapp_info.appTable[app_index].skip_pds_start && 0 != checkapp_info.appTable[app_index].skip_pds_end)
    {
      uint32_t index;
      uint16_t temp_buffer[ROW_SIZE_IN_HALF_WORDS];
      uint32_t numOfRows = (checkapp_info.appTable[app_index].skip_pds_start - checkapp_info.appTable[app_index].app_start_addr) / ROW_SIZE;
      for(index = 0; index < numOfRows; index++)
      {
        memcpy(&temp_buffer, (uint8_t *)(src_addr + (index * ROW_SIZE)), ROW_SIZE);
        hwEraseFlashRow(dst_addr + (index * ROW_SIZE));
        hwWriteFlashPage(dst_addr + (index * ROW_SIZE), (uint16_t *)&temp_buffer);
      }

      dst_addr = checkapp_info.appTable[app_index].skip_pds_end + 1;

      src_addr = src_addr + (checkapp_info.appTable[app_index].skip_pds_end + 1 - checkapp_info.appTable[app_index].app_start_addr);
      numOfRows = (img_size - ((checkapp_info.appTable[app_index].skip_pds_end + 1)
                                  - checkapp_info.appTable[app_index].app_start_addr)) / ROW_SIZE;
      for(index = 0; index <= numOfRows; index++)
      {
        memcpy(&temp_buffer, (uint8_t *)(src_addr + (index * ROW_SIZE)), ROW_SIZE);
        hwEraseFlashRow(dst_addr + (index * ROW_SIZE));
        hwWriteFlashPage(dst_addr + (index * ROW_SIZE), (uint16_t *)&temp_buffer);
      }
    }
    else
    {
      uint32_t index;
      uint16_t temp_buffer[ROW_SIZE_IN_HALF_WORDS];
      for(index = 0; index < (img_size + ROW_SIZE); index += ROW_SIZE)
      {
        memcpy(&temp_buffer, (uint8_t *)(src_addr + index), ROW_SIZE);
        hwEraseFlashRow(dst_addr + index);
        hwWriteFlashPage(dst_addr + index, (uint16_t *)&temp_buffer);
      }
    }
  }
  /*unsecured image*/
}

/**************************************************************************//**
\brief Read EEPROM index 39 to 46, and populate skip offsets
******************************************************************************/
#ifdef EXT_MEMORY
void configureSkipRegionForFlashCopy(uint8_t app_index)
{

  memcpy(&skipStartAddress,&(checkapp_info.appTable[app_index].skip_pds_start),4);
  memcpy(&skipEndAddress,&(checkapp_info.appTable[app_index].skip_pds_end),4);
  // ceil the addresses to the nearest page boundary(256bytes)
  if (skipStartAddress & REMAINDER_FOR_256_MASK)
    skipStartAddress += (uint32_t)ADD_256_FOR_ROUND_MASK;
  
  if (skipEndAddress & REMAINDER_FOR_256_MASK)
    skipEndAddress += (uint32_t)ADD_256_FOR_ROUND_MASK;  
  
  skipStartAddress &= ROUND_TO_256_INT_MASK;
  skipEndAddress &= ROUND_TO_256_INT_MASK;
  
  skipStatus = SKIP_NOT_STARTED;
}

/**************************************************************************//**
\brief Skips writing internal flash for the PDS region.Supports one skip region.
       Populates dataBuffer.address and dataBuffer.dataSize based on skip 
       start and end addresses. These addresses are passed by application
       through EEPROM
\return true - skip the call to int flash write, false - continue with flash write
******************************************************************************/
bool skipRegionForFlashCopy(void)
{
  bool retSts = false;
  
  if (skipStartAddress &&
      (SKIP_DONE != skipStatus))
  { // valid skip address
    if (SKIP_NOT_STARTED == skipStatus)
    { // first skip page
      // validate skip start and end addresses
      if ( (skipStartAddress >= skipEndAddress) || 
           (skipStartAddress >= IMAGE_SIZE)     ||
           (skipEndAddress >= IMAGE_SIZE) )
        skipStatus = SKIP_DONE;

      if (IS_SKIP_ADDRESS_WITHIN_BUFFER(dataBuffer.address, SREC_DATA_LENGTH, skipStartAddress))
      { // skipStartAddress within page
        if (IS_SKIP_ADDRESS_WITHIN_BUFFER(dataBuffer.address, SREC_DATA_LENGTH, skipEndAddress))
        { // hole within the page. Call flashPacketToPage() twice for either sides of the hole
          dataBuffer.dataSize = skipStartAddress - dataBuffer.address;
          flashPacketToPage();

          // partial write
          dataBuffer.address  = skipEndAddress;
          dataBuffer.dataSize = (dataBuffer.address+SREC_DATA_LENGTH) - skipEndAddress;
          skipStatus = SKIP_DONE;
        }
        else
        { // hole spans across pages. so, partial write of the current page
          skipStatus = SKIP_IN_PROCESS;
          dataBuffer.dataSize = skipStartAddress - dataBuffer.address;
        }
      }
    }
    else 
    { // multi-page skip. in the middle of skip process
      if (IS_SKIP_ADDRESS_WITHIN_BUFFER(dataBuffer.address, SREC_DATA_LENGTH, skipEndAddress))
      { // last page of the hole. Partial write
        dataBuffer.dataSize = (dataBuffer.address+SREC_DATA_LENGTH) - skipEndAddress;
        dataBuffer.address = skipEndAddress;
        skipStatus = SKIP_DONE;
      }
      else
      { // skip complete page
        retSts = true;
      }
    }
  }
  return retSts;
}
#endif

/**************************************************************************//**
\brief Reads data from external flash and loads to internal flash.
******************************************************************************/
#ifdef EXT_MEMORY
void extReadDataForFlashArea(uint32_t ext_addr,uint32_t int_addr,uint32_t size,uint8_t img_type,uint8_t *key,uint8_t app_index)
{
  uint32_t address = 0ul;
#if (USE_IMAGE_SECURITY == 1)  
  uint32_t intWriteAddr = 0;
   halPhyInit();
#endif
  configureSkipRegionForFlashCopy(app_index);
  while (address < size)
  {
    memReadData(address+ext_addr, dataBuffer.data, SREC_DATA_LENGTH);
    dataBuffer.dataSize = SREC_DATA_LENGTH;
   
#if (USE_IMAGE_SECURITY == 1)
    if (img_type == TYPE_SECURED)
    {
       dataBuffer.address = intWriteAddr +ext_addr;
       PHY_DecryptReq(key,dataBuffer.data, SREC_DATA_LENGTH/AES_BLOCK_SIZE);
    }
    else
#endif /* (USE_IMAGE_SECURITY == 1) */
      dataBuffer.address = address + ext_addr;

    if(!skipRegionForFlashCopy())
    {
      halFlashWrite (address+int_addr, dataBuffer.data,SREC_DATA_LENGTH);
    }
    address += SREC_DATA_LENGTH;
#if (USE_IMAGE_SECURITY == 1)
    intWriteAddr += SREC_DATA_LENGTH;
    if (intWriteAddr >= MCU_FLASH_SIZE)
      break;
#endif
  }
}
#endif

/**************************************************************************//**
\brief Counts crc given  memory block. CRC-8. Polynom 0x31    x^8 + x^5 + x^4 + 1.

\param[in]
  crc - first crc state
\param[in]
  pcBlock - pointer to the memory for crc counting
\param[in]
  length - memory size

\return
  current area crc
******************************************************************************/
uint8_t MemcalcCrc(uint8_t crc, uint8_t *pcBlock, uint8_t length)
{
  uint8_t i;
  while (length--)
  {
    crc ^= *pcBlock++;
    for (i = 0; i < 8; i++)
      crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
  }
  return crc;
}
/******************************************************************************************\\**
\brief Routine to Update BootInfo to  with updated CRC; 
     
*******************************************************************************************/
void update_boot_info(void)
{
  uint8_t calculated_crc;
  boot_info_t newbInfo;
  memcpy(&newbInfo,(uint32_t *)&bInfo,sizeof(newbInfo));
 
  calculated_crc = MemcalcCrc(0,(uint8_t *)&(newbInfo.majorRev),
                         sizeof(boot_info_t)- 
                           ((uint8_t *)&newbInfo.majorRev - ((uint8_t *)&newbInfo.crc)));
  newbInfo.crc = calculated_crc;  
  halFlashWrite (BOOT_INFO_START_ADDRESS, (uint8_t *)&newbInfo, sizeof(boot_info_t));
}

// eof app_info.c
