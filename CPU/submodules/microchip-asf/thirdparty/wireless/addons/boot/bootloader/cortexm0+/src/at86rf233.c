/**
* \file  phy.c
*
* \brief AT86RF233 PHY implementation
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

/*- Includes ---------------------------------------------------------------*/
#include <stdbool.h>
#include "phy.h"
#include "at86rf233.h"
#include "app_info.h"
/*- Definitions ------------------------------------------------------------*/
#define PHY_CRC_SIZE    2

/*- Types ------------------------------------------------------------------*/
typedef enum
{
  PHY_STATE_INITIAL,
  PHY_STATE_IDLE,
  PHY_STATE_SLEEP,
  PHY_STATE_TX_WAIT_END,
} PhyState_t;

#define AES_ECB   (0)
#define AES_CBC   (2)

/*- Prototypes -------------------------------------------------------------*/
static void phyWriteRegister(uint8_t reg, uint8_t value);
static uint8_t phyReadRegister(uint8_t reg);
static void phyWaitState(uint8_t state);
static void phyTrxSetState(uint8_t state);
static void phySetRxState(void);
uint8_t default_key[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/*- Variables --------------------------------------------------------------*/

static PhyState_t phyState = PHY_STATE_INITIAL;
static uint8_t phyRxBuffer[128];
static bool phyRxState;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/

void PHY_Init(void)
{
  halPhyReset();
  phyRxState = false;
  phyState = PHY_STATE_IDLE;

  phyWriteRegister(TRX_STATE_REG, TRX_CMD_TRX_OFF);
  phyWaitState(TRX_STATUS_TRX_OFF);

  phyWriteRegister(TRX_CTRL_1_REG, (1<<TX_AUTO_CRC_ON) | (3<<SPI_CMD_MODE) |
      (1<<IRQ_MASK_MODE));

  phyWriteRegister(TRX_CTRL_2_REG, (1<<RX_SAFE_MODE) | (1<<OQPSK_SCRAM_EN));
}

/*************************************************************************//**
*****************************************************************************/
void PHY_SetRxState(bool rx)
{
  phyRxState = rx;
  phySetRxState();
}

/*************************************************************************//**
*****************************************************************************/
void PHY_SetChannel(uint8_t channel)
{
  uint8_t reg;

  reg = phyReadRegister(PHY_CC_CCA_REG) & ~0x1f;
  phyWriteRegister(PHY_CC_CCA_REG, reg | channel);
}

/*************************************************************************//**
*****************************************************************************/

void PHY_SetXtaltrim(uint8_t xtal_trim)
{
  uint8_t reg;

  reg = phyReadRegister(XOSC_CTRL_REG) & ~0x0F;
  phyWriteRegister(PHY_CC_CCA_REG, reg | xtal_trim);
}
/*************************************************************************//**
*****************************************************************************/
void PHY_SetPanId(uint16_t panId)
{
  uint8_t *d = (uint8_t *)&panId;

  phyWriteRegister(PAN_ID_0_REG, d[0]);
  phyWriteRegister(PAN_ID_1_REG, d[1]);
}

/*************************************************************************//**
*****************************************************************************/
void PHY_SetShortAddr(uint16_t addr)
{
  uint8_t *d = (uint8_t *)&addr;

  phyWriteRegister(SHORT_ADDR_0_REG, d[0]);
  phyWriteRegister(SHORT_ADDR_1_REG, d[1]);
  phyWriteRegister(CSMA_SEED_0_REG, d[0] + d[1]);
}

/*************************************************************************//**
*****************************************************************************/
void PHY_SetTxPower(uint8_t txPower)
{
  uint8_t reg;

  reg = phyReadRegister(PHY_TX_PWR_REG) & ~0x0f;
  phyWriteRegister(PHY_TX_PWR_REG, reg | txPower);
}

/*************************************************************************//**
*****************************************************************************/
void PHY_Sleep(void)
{
  phyTrxSetState(TRX_CMD_TRX_OFF);
  halPhySlpTrSet();
  phyState = PHY_STATE_SLEEP;
}

/*************************************************************************//**
*****************************************************************************/
void PHY_Wakeup(void)
{
  halPhySlpTrClear();
  phySetRxState();
  phyState = PHY_STATE_IDLE;
}

/*************************************************************************//**
*****************************************************************************/
void PHY_DataReq(uint8_t *data, uint8_t size)
{
  phyTrxSetState(TRX_CMD_TX_ARET_ON);

  phyReadRegister(IRQ_STATUS_REG);

  halPhySpiSelect();
  halPhySpiWriteByte(RF_CMD_FRAME_W);
  halPhySpiWriteByte(size + PHY_CRC_SIZE);
  
  for (uint8_t i = 0; i < size; i++)
    halPhySpiWriteByte(data[i]);
  halPhySpiDeselect();

  phyState = PHY_STATE_TX_WAIT_END;
  halPhySlpTrSet();
  halPhySlpTrClear();
}

#ifdef PHY_ENABLE_RANDOM_NUMBER_GENERATOR
/*************************************************************************//**
*****************************************************************************/
uint16_t PHY_RandomReq(void)
{
  uint16_t rnd = 0;
  uint8_t rndValue;

  phyTrxSetState(TRX_CMD_RX_ON);

  for (uint8_t i = 0; i < 16; i += 2)
  {
    halDelay(RANDOM_NUMBER_UPDATE_INTERVAL);
    rndValue = (phyReadRegister(PHY_RSSI_REG) >> RND_VALUE) & 3;
    rnd |= rndValue << i;
  }

  phySetRxState();

  return rnd;
}
#endif

//uint8_t AES_status = 0;


/*************************************************************************//**
*****************************************************************************/


void PHY_DecryptReq(uint8_t *key, uint8_t *dec_text, uint8_t no_of_blocks)
{
  uint8_t index;static uint8_t f=1;static uint8_t fla=1;
  uint8_t Key[AES_BLOCK_SIZE];static uint8_t tempm2[AES_BLOCK_SIZE];uint8_t tempm1[AES_BLOCK_SIZE];
  if(fla==1)
  {
  for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
      tempm2[i] =dec_text[i];
  fla=0;
  }
  for(index = 0; index < no_of_blocks; index++)
  {
    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
      tempm1[i] =dec_text[i];
    PHY_SetAESKey(key);
    halPhySpiSelect();
    halPhySpiWriteByte(RF_CMD_SRAM_W);
    halPhySpiWriteByte(AES_CTRL_REG);
    halPhySpiWriteByte(0x00);
    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
      halPhySpiWriteByte(0);
    halPhySpiWriteByte(0x80);
    halPhySpiDeselect();

    halDelay(AES_CORE_CYCLE_TIME);

    halPhySpiSelect();
    halPhySpiWriteByte(RF_CMD_SRAM_W);
    halPhySpiWriteByte(AES_CTRL_REG);
    halPhySpiWriteByte((1<<AES_CTRL_MODE) | (0<<AES_CTRL_DIR));
    halPhySpiWriteByte(0);
    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
      Key[i] = halPhySpiWriteByte(0);
    halPhySpiDeselect();

    halPhySpiSelect();
    halPhySpiWriteByte(RF_CMD_SRAM_W);
    halPhySpiWriteByte(AES_CTRL_REG);
    halPhySpiWriteByte((1<<AES_CTRL_MODE) | (0<<AES_CTRL_DIR));
    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
      halPhySpiWriteByte(Key[i]);
    halPhySpiDeselect();

    halPhySpiSelect();
    halPhySpiWriteByte(RF_CMD_SRAM_W);
    halPhySpiWriteByte(AES_CTRL_REG);
    halPhySpiWriteByte(0x00);//ecb mode 
     //halPhySpiWriteByte(0x10);//ecb mod
    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
      halPhySpiWriteByte(dec_text[i]);
    halPhySpiWriteByte(0x88);
    halPhySpiDeselect();

    halDelay(AES_CORE_CYCLE_TIME);

    halPhySpiSelect();
    halPhySpiWriteByte(RF_CMD_SRAM_R);
    halPhySpiWriteByte(AES_STATE_REG);
    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    {
      dec_text[i] = halPhySpiWriteByte(0);
      
    }
    halPhySpiDeselect();
    if(f!=1)
    {
      for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        dec_text[i]^= tempm2[i];
      for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
        tempm2[i] =tempm1[i];
       
    }    
    else
      f=0;
    
     
     dec_text += AES_BLOCK_SIZE;
    
  }
}

void PHY_SetAESKey(uint8_t *key)
{
  halPhySpiSelect();
  halPhySpiWriteByte(RF_CMD_SRAM_W);
  halPhySpiWriteByte(AES_CTRL_REG);
  if(key[0]==0xFF && key[1]==0xFF)
  {
    key = (uint8_t *)&default_key;
  }
  halPhySpiWriteByte((1<<AES_CTRL_MODE) | (0<<AES_CTRL_DIR));
  for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    halPhySpiWriteByte(key[i]);
  halPhySpiDeselect();
}


/*************************************************************************//**
*****************************************************************************/
static void phyWriteRegister(uint8_t reg, uint8_t value)
{
  halPhySpiSelect();
  halPhySpiWriteByteInline(RF_CMD_REG_W | reg);
  halPhySpiWriteByteInline(value);
  halPhySpiDeselect();
}

/*************************************************************************//**
*****************************************************************************/
static uint8_t phyReadRegister(uint8_t reg)
{
  uint8_t value;

  halPhySpiSelect();
  halPhySpiWriteByteInline(RF_CMD_REG_R | reg);
  value = halPhySpiWriteByteInline(0);
  halPhySpiDeselect();

  return value;
}

/*************************************************************************//**
*****************************************************************************/
static void phyWaitState(uint8_t state)
{
  while (state != (phyReadRegister(TRX_STATUS_REG) & TRX_STATUS_MASK));
}

/*************************************************************************//**
*****************************************************************************/
static void phySetRxState(void)
{
  phyTrxSetState(TRX_CMD_TRX_OFF);

  phyReadRegister(IRQ_STATUS_REG);

  if (phyRxState)
    phyTrxSetState(TRX_CMD_RX_AACK_ON);
}

/*************************************************************************//**
*****************************************************************************/
static void phyTrxSetState(uint8_t state)
{
  phyWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
  phyWaitState(TRX_STATUS_TRX_OFF);

  phyWriteRegister(TRX_STATE_REG, state);
  phyWaitState(state);
}

/*************************************************************************//**
*****************************************************************************/
void PHY_TaskHandler(void)
{
//  if (PHY_STATE_SLEEP == phyState)
//    return;

  if (phyReadRegister(IRQ_STATUS_REG) & (1<<TRX_END))
  {
    
    if (PHY_STATE_TX_WAIT_END == phyState)
    {
      uint8_t status = (phyReadRegister(TRX_STATE_REG) >> TRAC_STATUS) & 7;

      if (TRAC_STATUS_SUCCESS == status)
        status = PHY_STATUS_SUCCESS;
//      else if (TRAC_STATUS_CHANNEL_ACCESS_FAILURE == status)
//        status = PHY_STATUS_CHANNEL_ACCESS_FAILURE;
//      else if (TRAC_STATUS_NO_ACK == status)
//        status = PHY_STATUS_NO_ACK;
      else
        status = PHY_STATUS_ERROR;

      phySetRxState();
      phyState = PHY_STATE_IDLE;

      PHY_DataConf(status);
    }
    else if (PHY_STATE_IDLE == phyState)
    {
      PHY_DataInd_t ind;
      uint8_t size;
      int8_t rssi;

      rssi = (int8_t)phyReadRegister(PHY_ED_LEVEL_REG);

      halPhySpiSelect();
      halPhySpiWriteByte(RF_CMD_FRAME_R);
      size = halPhySpiWriteByte(0);
      for (uint8_t i = 0; i < size + 1/*lqi*/; i++)
        phyRxBuffer[i] = halPhySpiWriteByte(0);
      halPhySpiDeselect();

      ind.data = phyRxBuffer;
      ind.size = size - PHY_CRC_SIZE;
      ind.lqi  = phyRxBuffer[size];
      ind.rssi = rssi + PHY_RSSI_BASE_VAL;
      PHY_DataInd(&ind);

      phyWaitState(TRX_STATUS_RX_AACK_ON);
    }
  }
}

//void PHY_SetChannelPage(uint8_t page)
//{
//	//valid values of page = 0,1,2,3 (250,500,1000,2000)
//	uint8_t reg;
//	reg = phyReadRegister(TRX_CTRL_2_REG) & ~0x07;
//	phyWriteRegister(TRX_CTRL_2_REG, reg | page);
//	reg = phyReadRegister(XAH_CTRL_1_REG) & ~0x04;
//	if(page)
//		phyWriteRegister(XAH_CTRL_1_REG, reg | 0X04);
//	else
//		phyWriteRegister(XAH_CTRL_1_REG, reg);
//}

void PHY_SetIEEEAddr(uint8_t *ieee_addr)
{
  phyWriteRegister(IEEE_ADDR_0_REG, ieee_addr[0]);
	phyWriteRegister(IEEE_ADDR_1_REG, ieee_addr[1]);
	phyWriteRegister(IEEE_ADDR_2_REG, ieee_addr[2]);
	phyWriteRegister(IEEE_ADDR_3_REG, ieee_addr[3]);
	phyWriteRegister(IEEE_ADDR_4_REG, ieee_addr[4]);
	phyWriteRegister(IEEE_ADDR_5_REG, ieee_addr[5]);
	phyWriteRegister(IEEE_ADDR_6_REG, ieee_addr[6]);
	phyWriteRegister(IEEE_ADDR_7_REG, ieee_addr[7]);
}

void PHY_EnableReservedFrameRx(void)
{
	uint8_t reg;
	reg = phyReadRegister(XAH_CTRL_1_REG) & ~0x07;
	reg = reg | (1 << AACK_FLTR_RES_FT) | (1 << AACK_UPLD_RES_FT);
	phyWriteRegister(XAH_CTRL_1_REG, reg);
}


