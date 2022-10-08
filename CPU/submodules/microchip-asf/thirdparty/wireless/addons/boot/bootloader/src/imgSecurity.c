/**
* \file  imgSecurity.c
*
* \brief Image encryption/decryption, file integrity check related
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
#if (USE_IMAGE_SECURITY == 1)

/******************************************************************************
                   Security module call hierarchy
                        -------------------

                  extMemReader.c/hardwareInit.c
                              |
                        ImgSecurity.c
                              |
                          ----------
                          |        |
                        phy.c     phy.c
                      (256RFR2)  (SAMR21)
******************************************************************************/

/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
#include <extMemReader.h>
#include <bootStructure.h>
#include <phy.h>
#include <imgSecurity.h>
#include <eepromLoader.h>

/******************************************************************************
                   Global variables section
******************************************************************************/
uint16_t otauHdrImgType;
uint32_t otauFileLen;
uint32_t otauFileHdrLen;
uint32_t payloadOffset;
uint32_t payloadLen;
uint32_t otauFileId;
uint32_t alignedPayloadLen;
uint16_t remainderData;
uint8_t  inKey[AES_BLOCKSIZE];
uint8_t  inIv[AES_BLOCKSIZE];
uint8_t  data[AES_BLOCKSIZE];
uint8_t  localMic[AES_BLOCKSIZE] = {0};
uint32_t fileId = OTAUHDR_FILEID;
uint32_t crcSize = 0;
uint16_t imgType;
endecryptReq_t req;

/******************************************************************************
                   External global variables section
******************************************************************************/
extern const uint32_t imageStartAddress[POSITION_MAX];
extern BootBuffer_t dataBuffer;
extern uint8_t readCrc;
extern uint8_t countedCrc;
extern ImagePosition_t imageNumber;
extern uint8_t dataMinusOne[AES_BLOCKSIZE];
extern bool isImgCrcChecked;

/******************************************************************************
                     Prototypes section
******************************************************************************/
extern uint8_t extMemCrc(uint8_t crc, uint8_t *pcBlock, uint8_t length);
extern void extCleanServiceEepromArea(void);

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief setup en/decrypt key and IV, if security was enabled

                          EEPROM
                       ---------------
           index 4    |Is key updated?|
                       ---------------
           index 5-20 |    New key    |
                       ---------------
           index 21-36|       IV      |
                       ---------------
           index 37,38|     ImgType   |
                       ---------------
******************************************************************************/
void setupKeyAndIv(void)
{
  // config user key or hard-coded key based on the request
  if (EEPROM_CHANGEKEYREQUESTED_VAL == eepromReadData(EEPROM_ISKEYCHANGED_OFFSET))
  {
    uint8_t i = EEPROM_NEWKEY_LENGTH;
    uint8_t j = 0;

    while (i)
    {
      inKey[j] = eepromReadData(EEPROM_NEWKEY_START_OFFSET+j);
      inIv[j]  = eepromReadData(EEPROM_NEWIV_START_OFFSET+j);
      i--;
      j++;
    }
  }
  else
  {
    IMGSEC_GET_DEFAULT_KEY(inKey);
    IMGSEC_GET_DEFAULT_IV(inIv);
  }

  phySetKeyAndIv(inKey, inIv);
}

/**************************************************************************//**
\brief setup the following:
  1. initialize Phy/AES
  2. setup decrypt key, if security was enabled
  Mode and direction will be set independently by the caller
******************************************************************************/
void setupKeyAndPhyAccess(void)
{
  phyClearHistory();
  setupKeyAndIv();
}

/**************************************************************************//**
\brief Checks the following:
  1. Check if the in-file is OTAU or raw file. return failure
  2. Check if the payload or CRC is encrypted or not
  3. Calculate CRC for the entire file - excluding the CRC(last 4 bytes)
  4. Save control info and strip OTAU header
  5. Decrypt CRC field if present. Compare with calculated CRC. strip CRC.

File integrity check is used the following way:

OTA source side:
          OTA-hdr-added-encrypted-file
                      |
  calculate MAC/CRC for the entire file including header
                      |
            Encrypt the MAC/CRC(same key)
                      |
    Append the encrypted MAC/CRC at the end of the file
                      |
    Update the OTA hdr bit for the presence of MAC/CRC

OTA destination side:
          OTA-hdr-added-encrypted-file
                      |
  Read OTA hdr and see if MAC/CRC is present at the end
                      |
         Decrypt the MAC/CRC,if encrypted
                      |
  Except the MAC/CRC, calculate the MAC/CRC for the rest of the file
                      |
      Match with the value read from the file

\return header check and integration check status
  true  - integrity is fine
  false - failure. can be due to one of :
          (1) file is raw
          (2) raw content within OTA file
          (3) file length incorrect - too less or unaligned
          (4) decryption key mismatch
          (5) CRC/MAC mismatch(file possibly corrupted)
******************************************************************************/
bool otauCheckHdrAndIntegrity(void)
{
  uint32_t address = 0ul;

  memReadData(address+imageStartAddress[imageNumber],
    (uint8_t *)(&otauFileId), OTAUHDR_FILEID_OCTETLEN);

  if (0 != memcmp((uint8_t *)&fileId, (uint8_t *)&otauFileId, OTAUHDR_FILEID_OCTETLEN))
  {
    // raw file, skip integrity check and return failure
    return false;
  }

  memReadData(OTAUHDR_IMGTYPE_OFFSET+imageStartAddress[imageNumber],
    (uint8_t *)(&otauHdrImgType), OTAUHDR_IMGTYPE_OCTETLEN);
  memReadData(OTAUHDR_HDRLEN_OFFSET+imageStartAddress[imageNumber],
    (uint8_t *)(&otauFileHdrLen), OTAUHDR_HDRLEN_OCTETLEN);
  memReadData(OTAUHDR_TOTALFILESIZE_OFFSET+imageStartAddress[imageNumber],
    (uint8_t *)(&otauFileLen), OTAUHDR_TOTALFILESIZE_OCTETLEN);

  if (!OTAUHDR_IMGTYPE_ISIMGSEC_ENABLED(otauHdrImgType))
  {
    // raw file inside OTA format file, skip integrity check and return failure
    return false;
  }

  payloadOffset = otauFileHdrLen;
  payloadLen    = otauFileLen;

  // do file integrity check
  if (OTAUHDR_IMGTYPE_ISINTCHKPRESENT(otauHdrImgType))
  {
    // read MAC/CRC at the end of the file
    if (otauFileLen > OTAUHDR_INTEGRITYCODE_OCTETLEN)
    {
      memReadData((otauFileLen-OTAUHDR_INTEGRITYCODE_OCTETLEN)+imageStartAddress[imageNumber],
                   localMic, OTAUHDR_INTEGRITYCODE_OCTETLEN);
      payloadLen -= OTAUHDR_INTEGRITYCODE_OCTETLEN;
    }
    else
    {
      // wrong file parameters
      return false;
    }

    alignedPayloadLen = payloadLen - (payloadLen % SREC_DATA_LENGTH);
    remainderData = payloadLen-alignedPayloadLen;

    // is CRC/MAC encrypted?
    if (OTAUHDR_IMGTYPE_ISINTCHKENCRYPTED(otauHdrImgType))
    {
      setupKeyAndPhyAccess();
      phySetModeAndDir(AES_MODE_CBC, AES_DIR_DECRYPT);

      // decrypt the integrity code
      processBlock((uint8_t *)localMic, OTAU_MIC_BLOCK_COUNT);
    }

    // file length for MAC check can not be unaligned to 128-bits
    if (payloadLen % BLOCK_SIZE)
      return false;

    // setup AES for MAC calculation
    if (OTAUHDR_IMGTYPE_ISINTCHKOTHERTHANCRC(otauHdrImgType))
    {
      setupKeyAndPhyAccess();
      phySetModeAndDir(AES_MODE_CBC, AES_DIR_ENCRYPT);
    }

    while (address < alignedPayloadLen)
    {
      memReadData(address+imageStartAddress[imageNumber], dataBuffer.data, SREC_DATA_LENGTH);
      if (OTAUHDR_IMGTYPE_ISINTCHKOTHERTHANCRC(otauHdrImgType))
        processBlock(dataBuffer.data, SREC_DATA_LENGTH/BLOCK_SIZE);
      else
        countedCrc = extMemCrc(countedCrc, dataBuffer.data, SREC_DATA_LENGTH);
      address += SREC_DATA_LENGTH;
    }

    // process the remainder
    if (remainderData)
    {
      memReadData(address+imageStartAddress[imageNumber], dataBuffer.data, remainderData);
      if (OTAUHDR_IMGTYPE_ISINTCHKOTHERTHANCRC(otauHdrImgType))
        processBlock(dataBuffer.data, remainderData/BLOCK_SIZE);
      else
        countedCrc = extMemCrc(countedCrc, dataBuffer.data, remainderData);
    }

    if (OTAUHDR_IMGTYPE_ISINTCHKOTHERTHANCRC(otauHdrImgType))
    {
      // read the MAC
      phyReadLastEncData(dataMinusOne);

      // compare with the file MAC. can fail due to wrong key or corrupted MAC
      if (memcmp(dataMinusOne, localMic, BLOCK_SIZE))
        return false;
    }
    else
    {
      // wrong key or corrupted CRC
      if (countedCrc != localMic[OTAU_MIC_CRCOFFSET])
        return false;
      else
        isImgCrcChecked = true;
    }
  }

  // either CRC match or no CRC. setup phy for CBC decrypt
  setupKeyAndPhyAccess();
  phySetModeAndDir(AES_MODE_CBC, AES_DIR_DECRYPT);

  return true;
}

/**************************************************************************//**
\brief Encrypt/Decrypt a block using AES engine

\param[in]
  cipher - start pointer of cipher text
  numBlocks - number of blocks(of 128 bytes) of cipher

\return
  true - decrypt is fine \n
  false - decrypt failed
******************************************************************************/
bool processBlock(uint8_t *cipher, uint8_t numBlocks)
{
  req.text       = cipher;
  req.blockCount = numBlocks;
  return phyEnDecrypt();
}
#endif //(USE_IMAGE_SECURITY == 1)

// eof imgSecurity.c
