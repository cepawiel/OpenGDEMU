/**
* \file  extMemReader.c
*
* \brief Look through action, check image crc and read data from external memory.
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
///******************************************************************************
//                   Includes section
//******************************************************************************/

#ifdef EXT_MEMORY

#include <extMemReader.h>
#include <spiMemInterface.h>
#include <bootStructure.h>
#include <eepromLoader.h>
#include <flashLoader.h>
#include <abstractSerializer.h>
#include <moveIntVector.h>
#include <appEntry.h>
#if (USE_IMAGE_SECURITY == 1)
#include <imgSecurity.h>
#endif
/******************************************************************************
                   External global variables section
******************************************************************************/
extern const uint32_t imageStartAddress[POSITION_MAX];
extern BootBuffer_t dataBuffer;
#if (USE_IMAGE_SECURITY == 1)
extern uint32_t payloadOffset;
extern uint32_t payloadLen;
extern uint16_t otauHdrImgType;
#endif

/******************************************************************************
                   Global variables section
******************************************************************************/
uint8_t readCrc;
uint8_t countedCrc = CRC_INITIALIZATION_VALUE;
ImagePosition_t imageNumber;
uint8_t eepromServiceArea[SERVICE_INFO_SIZE];
bool isImgCrcChecked = false;
skipStatus_t skipStatus;
static uint32_t skipStartAddress;
static uint32_t skipEndAddress;

/******************************************************************************
                     Prototypes section
******************************************************************************/

void extCleanServiceEepromArea(void);
static inline bool extCheckAction(void);
static inline bool extCheckCrc(void);
static inline void extReadDataForFlashArea(void);
static inline void extReadDataForEepromArea(void);
#if (USE_IMAGE_SECURITY == 1)
static inline bool extCheckHdrAndApplyIntegrity(void);
extern bool otauCheckHdrAndIntegrity(uint32_t src_addr);
#endif

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Counts crc current memory area. CRC-8. Polynom 0x31    x^8 + x^5 + x^4 + 1.

\param[in]
  crc - first crc state
\param[in]
  pcBlock - pointer to the memory for crc counting
\param[in]
  length - memory size

\return
  current area crc
******************************************************************************/
uint8_t extMemCrc(uint8_t crc, uint8_t *pcBlock, uint8_t length)
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

/**************************************************************************//**
\brief Clears eeprom service area.
******************************************************************************/
void extCleanServiceEepromArea(void)
{
  memset(dataBuffer.data, EMPTY_EEPROM_VALUE, SERVICE_INFO_SIZE);
  dataBuffer.address = EEPROM_START_ADDRESS;
  dataBuffer.dataSize = SERVICE_INFO_SIZE;
  eepromPacketToPage();
  eepromFlushPage();
}

/**************************************************************************//**
\brief Checks bootloader action. Clears service area if incorrect action.

\return
  true - correct action, \n
  false - other
******************************************************************************/
static inline bool extCheckAction(void)
{
  uint8_t i;

  for (i = 0; i < SERVICE_INFO_SIZE; i++)
    eepromServiceArea[i] = eepromReadData(i);

  imageNumber = (ImagePosition_t)eepromServiceArea[ACTION_POSITION];
  // check eeprom service area
  if (!(imageNumber < POSITION_MAX))
  {
    if (EMPTY_EEPROM_VALUE != eepromServiceArea[ACTION_POSITION])
    { // clear service eeprom area
      extCleanServiceEepromArea();
    }
    return false;
  }

  readCrc = eepromServiceArea[imageNumber+1];
  return true;
}

/**************************************************************************//**
\brief Checks image crc. Clears service area if incorrect crc.

\return
  true - correct crc, \n
  false - other
******************************************************************************/
static inline bool extCheckCrc(void)
{
  uint32_t address = 0ul;

  while (address < IMAGE_SIZE)
  {
    memReadData(address+imageStartAddress[imageNumber], dataBuffer.data, SREC_DATA_LENGTH);
    countedCrc = extMemCrc(countedCrc, dataBuffer.data, SREC_DATA_LENGTH);
    address += SREC_DATA_LENGTH;
  }

  if (countedCrc == readCrc)
  {
    return true;
  }
  else
  { // clear service eeprom area
    extCleanServiceEepromArea();
    return false;
  }
}

/**************************************************************************//**
\brief Read EEPROM index 39 to 46, and populate skip offsets
******************************************************************************/
void configureSkipRegionForFlashCopy(void)
{
  for (uint8_t i = 0; i < sizeof(uint32_t); i++)
  { // little endian
    skipStartAddress |= (eepromReadData(i+SKIP_START_ADDRESS_EEPROM_OFFSET) << (8*i));
    skipEndAddress   |= (eepromReadData(i+SKIP_END_ADDRESS_EEPROM_OFFSET) << (8*i));
  }
  
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

/**************************************************************************//**
\brief Reads data from external flash and loads to internal flash.
******************************************************************************/
static inline void extReadDataForFlashArea(void)
{
#if (USE_IMAGE_SECURITY == 1)
  uint32_t address = payloadOffset;
  uint32_t size    = payloadLen;
  uint32_t intWriteAddr = 0;
#else
  uint32_t address = 0ul;
  uint32_t size    = MCU_FLASH_SIZE;
#endif

  configureSkipRegionForFlashCopy();

  while (address < size)
  {
    memReadData(address+imageStartAddress[imageNumber], dataBuffer.data, SREC_DATA_LENGTH);

    dataBuffer.dataSize = SREC_DATA_LENGTH;
#if (USE_IMAGE_SECURITY == 1)
    dataBuffer.address = intWriteAddr + imageStartAddress[imageNumber];
    processBlock(dataBuffer.data, SREC_DATA_LENGTH/BLOCK_SIZE);
#else
    dataBuffer.address = address + imageStartAddress[imageNumber];
#endif /* (USE_IMAGE_SECURITY == 1) */

    if(!skipRegionForFlashCopy())
      flashPacketToPage();

    address += SREC_DATA_LENGTH;
#if (USE_IMAGE_SECURITY == 1)
    intWriteAddr += SREC_DATA_LENGTH;

    if (intWriteAddr >= MCU_FLASH_SIZE)
      break;
#endif
  }

  flashFlushPage();
}

/**************************************************************************//**
\brief Reads data from external flash and loads to internal eeprom.
******************************************************************************/
static inline void extReadDataForEepromArea(void)
{
  uint32_t address = MCU_FLASH_SIZE;
  uint32_t eepromAddres = EEPROM_START_ADDRESS;
  uint8_t eepromAction;

  eepromAction = eepromReadData(EEPROM_ACTION_POSITION);
  while (address < IMAGE_SIZE)
  {
    memReadData(address+imageStartAddress[imageNumber], dataBuffer.data, SREC_DATA_LENGTH);
    dataBuffer.address = eepromAddres;
    dataBuffer.dataSize = SREC_DATA_LENGTH;
    if (EEPROM_START_ADDRESS == eepromAddres)
    { // delete bootloader action
      eepromServiceArea[ACTION_POSITION] = EMPTY_EEPROM_VALUE;
      memcpy(dataBuffer.data, eepromServiceArea, SERVICE_INFO_SIZE);
      // Retaining EEPROM region allocated for secured OTAU & PDS restore parameters
      for (uint8_t i = SERVICE_INFO_SIZE;i < EEPROM_USER_REGION_START_ADDRESS; i++)
        dataBuffer.data[i] = eepromReadData(i);
    }
    if (!eepromAction)
    {
      dataBuffer.dataSize = SERVICE_INFO_SIZE;
      eepromPacketToPage();
      break;
    }
    eepromPacketToPage();
    eepromAddres += SREC_DATA_LENGTH;
    address += SREC_DATA_LENGTH;
  }

  eepromFlushPage();
}

/**************************************************************************//**
\brief write imgType to EEPROM for new image's use. Write zero for plain image
\brief write imgType to EEPROM for new image's use. Write zero for plain image
       For secure image bootloader:
       1. read eeprom imgtype
       2. if it is not a valid value, write SECURED_DEFAULT_IMGTYPE_VALUE
       3. if it already has valid value, do no over-write
       4. Always write file's image type after update
       For plain image bootloader:
       always write 0 as image type

\param[in] flag that indicates whether the call was before honoring the image or
           after
******************************************************************************/
void writeImgTypeToEeprom(bool isAfterFlashUpdate)
{
  bool write = false;
#if (USE_IMAGE_SECURITY == 1)
  uint16_t eepromImgType;
  uint16_t tmp;
  
  eepromImgType = eepromReadData(EEPROM_IMGTYPE_OFFSET);
  tmp           = eepromReadData(EEPROM_IMGTYPE_OFFSET+1);
  eepromImgType|= (tmp << 8);

  tmp = SECURED_DEFAULT_IMGTYPE_VALUE;
  if (true == isAfterFlashUpdate)
  {
    tmp = otauHdrImgType;
    write = true;
  }
  else 
  {
    if (!(eepromImgType & OTAUHDR_IMGTYPE_IMGENC_MASK) ||
         (eepromImgType == INVALID_IMGTYPE_VALUE))
      write = true;
  }

  memcpy(dataBuffer.data, &tmp, EEPROM_IMGTYPE_LENGTH);
#else
  memset(dataBuffer.data, DEFAULT_IMGTYPE_VALUE, EEPROM_IMGTYPE_LENGTH);
  write = true;
#endif
  if (write)
  {
    dataBuffer.address = EEPROM_START_ADDRESS+EEPROM_IMGTYPE_OFFSET;
    dataBuffer.dataSize = EEPROM_IMGTYPE_LENGTH;
    eepromPacketToPage();
  }
  (void)isAfterFlashUpdate;
}

/**************************************************************************//**
\brief Checks the following:
  1. Check if the in-file is OTAU or raw file
  2. Check if the payload or CRC is encrypted or not
  3. Calculate CRC for the entire file - excluding the CRC(last 4 bytes)
  4. Save control info and strip OTAU header
  5. Decrypt CRC field if present. Compare with calculated CRC. strip CRC.

\return
  true - integrity is fine \n
  false - integrity check failed
******************************************************************************/
static inline bool extCheckHdrAndApplyIntegrity(void)
{
#if (USE_IMAGE_SECURITY == 1)
  return otauCheckHdrAndIntegrity();
#else
  return true;
#endif
}

/**************************************************************************//**
\brief Starts external flash loader. Makes all tests and loads image.
******************************************************************************/
void extLoadDataFromExtMem(void)
{
  // init serial interface
  spiMemInit();

  // write default ImgType to EEPROM
  writeImgTypeToEeprom(false);
  
  // check availability of the external flash
  if (false == memCheckMem())
  {
    spiMemUnInit();
    return;
  }
  

  // checks action
  if (false == extCheckAction())
  {
    spiMemUnInit();
    return;
  }

  // checks for file integrity, if present in OTAU file
  if (false == extCheckHdrAndApplyIntegrity())
  {
    spiMemUnInit();
    return;
  } 

  // checks crc
  if (false == isImgCrcChecked)
  {
    if (false == extCheckCrc())
    {
      spiMemUnInit();
      return;
    }
    isImgCrcChecked = true;
  }
  
  // load internal flash
  extReadDataForFlashArea();
  // load internal eeprom
  extReadDataForEepromArea();

  // write updated ImgType to EEPROM for image's use
  writeImgTypeToEeprom(true);

  // clear spi interface
  spiMemUnInit();
  // clear low level initialization
  
  // Change interrupt vector to apps section
  moveIntVectToAppSection();
  jumpToApplication();
 
  
}

#endif //EXT_MEMORY

// eof extMemReader.c
