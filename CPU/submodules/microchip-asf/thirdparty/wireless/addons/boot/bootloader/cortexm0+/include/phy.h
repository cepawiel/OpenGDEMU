/**
* \file  phy.h
*
* \brief Phy access APIs for TRX and AES en/decryption - SAMR21
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

#ifndef _PHY_H_
#define _PHY_H_

#include <stdint.h>
#include <stdbool.h>
#include <gpio.h>
#include <component_sercom.h>
#include <fcpu.h>
#if (defined(ATSAMR21E18A) || defined(ATSAMR21G18A) || defined(ATSAMR21E19A))
#include <atsamr21.h>
#endif
#if (defined(ATSAMR30E18A) || defined(ATSAMR30G18A))
#include <atsamr30.h>
#endif

#define AES_BLOCK_SIZE                 16

#define PHY_RSSI_BASE_VAL                       (-91)
#define PHY_HAS_RANDOM_NUMBER_GENERATOR
#define PHY_HAS_AES_MODULE

#define GPIO_PORTA                                0
#define GPIO_PORTB                                1
#define GPIO_PORTC                                2

#define HAL_RESET_DELAY                           10
#define CLKCTRL_SERCOM_ID                         0x18
#define CLKCTRL_GENERTR_VAL                       0
#define CLKCTRL_GENERTR_CLKEN                     1
#define SERCOM4_SPI_CTRLA_MODE_VAL                3
#define SERCOM4_SPI_CTRLA_DOPO_VAL                1
#define PORTB_PMUX15_PMUXE_VALUE                  5
#define PORTC_PMUX9_PMUXE_VALUE                   5
#define PORTC_PMUX9_PMUXO_VALUE                   5
#define SPI_CLOCK_RATE_2000                     ((F_CPU / (2 * 2000000ul)) - 1)

/**************** Phy access APIs ********************************************/
#define PHY_SPI_SELECT()          GPIO_PHY_CS_clr()
#define PHY_SPI_WRITE_BYTE(value) halPhySpiWriteByteInline((value))
#define PHY_SPI_DESELECT()        GPIO_PHY_CS_set()


/*- Types ------------------------------------------------------------------*/
typedef struct PHY_DataInd_t
{
  uint8_t    *data;
  uint8_t    size;
  uint8_t    lqi;
  int8_t     rssi;
} PHY_DataInd_t;

enum
{
  PHY_STATUS_SUCCESS                = 0,
  PHY_STATUS_CHANNEL_ACCESS_FAILURE = 1,
  PHY_STATUS_NO_ACK                 = 2,
  PHY_STATUS_ERROR                  = 3,
};

/*- Prototypes -------------------------------------------------------------*/
void PHY_Init(void);
void PHY_SetRxState(bool rx);
void PHY_SetChannel(uint8_t channel);
void PHY_SetXtaltrim(uint8_t xtal_trim);
void PHY_SetPanId(uint16_t panId);
void PHY_SetShortAddr(uint16_t addr);
void PHY_SetTxPower(uint8_t txPower);
void PHY_Sleep(void);
void PHY_Wakeup(void);
void PHY_DataReq(uint8_t *data, uint8_t size);
void PHY_DataConf(uint8_t status);
void PHY_DataInd(PHY_DataInd_t *ind);
void PHY_TaskHandler(void);
//void PHY_SetChannelPage(uint8_t page);
void PHY_SetIEEEAddr(uint8_t *ieee_addr);
void PHY_EnableReservedFrameRx(void);
uint8_t halPhySpiWriteByte(uint8_t value);
void halPhyReset(void);
void halPhyInit(void);
void halPhyUnInit(void);
static void halDelay(uint64_t us);

#ifdef PHY_ENABLE_RANDOM_NUMBER_GENERATOR
uint16_t PHY_RandomReq(void);
#endif
void PHY_EncryptReq(uint8_t *key, uint8_t *text, uint8_t no_of_blocks);
void PHY_DecryptReq(uint8_t *key, uint8_t *dec_text, uint8_t no_of_blocks);
void PHY_SetAESKey(uint8_t *key);
#ifdef PHY_ENABLE_ENERGY_DETECTION
int8_t PHY_EdReq(void);
#endif


/**************************************************************************//**
 \brief Delay(busy wait) in micro-seconds

 \param[in] micro-seconds delay intended
**************************************************************************/
static void halDelay(uint64_t us)
{
  uint64_t i;

  for(i = 0; i < us; i++)
  {
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
  }
}

/**************************************************************************//**
 \brief Write a value to SPI

 \param[in] value to be written
**************************************************************************/
static uint8_t halPhySpiWriteByteInline(uint8_t value)
{
  SERCOM4->SPI.DATA.reg = value;
  while (!SERCOM4->SPI.INTFLAG.bit.RXC);
  return SERCOM4->SPI.DATA.reg;
}

/*****************************************************************************
 \brief function to check the SPI CTRL B BUSY status

 \return true - sync busy, false - not busy
*****************************************************************************/
static inline bool halSpiCtrlBSyncing()
{
  return (SC4_SPI_SYNCBUSY & SC0_SPI_SYNCBUSY_CTRLB);
}

/*****************************************************************************
 \brief function to check the SPI ENABLE BUSY status

 \return true - sync busy enabled, false - not enabled
*****************************************************************************/
static inline bool halSpiEnableSyncing()
{
  return (SC4_SPI_SYNCBUSY & SC0_SPI_SYNCBUSY_ENABLE);
}

/*****************************************************************************
 \brief function to check the SPI ENABLE BUSY status

 \return true - sync busy set, false - sync busy reset
*****************************************************************************/
static inline bool halSpiSwrstSyncing()
{
  return (SC4_SPI_SYNCBUSY & SC0_SPI_SYNCBUSY_SWRST);
}

/**************************************************************************//**
 \brief Phy access(through GPIO) static functions

  These macros define set, clear, toggle, in, out, pullup, read and few
  other reg operations
**************************************************************************/
HW_ASSIGN_PIN(PHY_RST,    B, 15);
HW_ASSIGN_PIN(PHY_IRQ,    B, 0);
HW_ASSIGN_PIN(PHY_SLP_TR, A, 20);
HW_ASSIGN_PIN(PHY_CS,     B, 31);
HW_ASSIGN_PIN(PHY_MISO,   C, 19);
HW_ASSIGN_PIN(PHY_MOSI,   B, 30);
HW_ASSIGN_PIN(PHY_SCK,    C, 18);

/*************************************************************************//**
*****************************************************************************/
static inline void halPhySpiSelect(void)
{
  GPIO_PHY_CS_clr();
}

/*************************************************************************//**
*****************************************************************************/
static inline void halPhySpiDeselect(void)
{
  GPIO_PHY_CS_set();
}

/*************************************************************************//**
*****************************************************************************/
static inline void halPhySlpTrSet(void)
{
  GPIO_PHY_SLP_TR_set();
}

/*************************************************************************//**
*****************************************************************************/
static inline void halPhySlpTrClear(void)
{
  GPIO_PHY_SLP_TR_clr();
}



#if (USE_IMAGE_SECURITY == 1)
/******************************************************************************
                   Type Definitions section
******************************************************************************/
/** AES modes */
typedef enum {
  AES_MODE_ECB = (0),
  AES_MODE_CBC = (2),
  AES_MODE_UNKNOWN
} aesModeType_t;

/** AES direction */
typedef enum {
  AES_DIR_ENCRYPT = (0),
  AES_DIR_DECRYPT = (1),
  AES_DIR_UNKNOWN
} aesDirectionType_t;

/** En/Decryption request structure */
typedef struct
{
  //! Pointer to input data for AES command.
  uint8_t *text;

  //! Size of input data in blocks(128-bit)
  uint8_t blockCount;
} endecryptReq_t;

/******************************************************************************
                   Implementation section
******************************************************************************/

/******************************************************************************
                   constants section
******************************************************************************/

/** Key length */
#define KEY_SIZE                                (16U)
#define AES_BLOCKSIZE                           KEY_SIZE


/******************************************************************************

AES_CTRL:

position:      7      6     5     4     3     2     1     0
            -------------------------------------------------
meaning:    | REQ |  M2 |  M1 |  M0 | DIR |  R  |  R  |  R  |
            -------------------------------------------------

M2 M1 M0
0  0  0  - ECB
0  0  1  - key operation
0  1  0  - CBC

DIR
0  - Encryption
1  - Decryption

REQ
0  - idle
1  - start operation

-------------------------------------------------------------------------------

AES_STATUS:

position:      7      6     5     4     3     2     1     0
            -------------------------------------------------
meaning:    | ERR |  R  |  R  |  R  |  R  |  R  |  R  | DON |
            -------------------------------------------------

ERR
0  - no error
1  - error

DON
0  - operation not completed
1  - operation completed

******************************************************************************/
#define MODEBYTE_FOR_KEY_OP                       (0x10)
#define MODEBYTE_FOR_ECBDECRYPT                   (0x08)
#define MODEBYTE_FOR_ECBENCRYPT                   (0x00)
#define MODEBYTE_FOR_START_OP                     (0x80)


#define AES_CORE_CYCLE_TIME_US                    (24)


/**************** AES engine access recipes **********************************/
#define WRITE_AES_KEY(key)                                      \
  PHY_SPI_SELECT();                                             \
  PHY_SPI_WRITE_BYTE(RF_CMD_SRAM_W);                            \
  PHY_SPI_WRITE_BYTE(AES_CTRL_REG);                             \
  PHY_SPI_WRITE_BYTE(MODEBYTE_FOR_KEY_OP);                      \
  for (uint8_t i = 0; i < AES_BLOCKSIZE; i++)                   \
    PHY_SPI_WRITE_BYTE(key[i]);                                 \
  PHY_SPI_DESELECT();

#define WRITE_AES_ZERO_KEY()                                    \
  PHY_SPI_SELECT();                                             \
  PHY_SPI_WRITE_BYTE(RF_CMD_SRAM_W);                            \
  PHY_SPI_WRITE_BYTE(AES_CTRL_REG);                             \
  PHY_SPI_WRITE_BYTE(MODEBYTE_FOR_KEY_OP);                      \
  for (uint8_t i = 0; i < AES_BLOCKSIZE; i++)                   \
    PHY_SPI_WRITE_BYTE(0);                                      \
  PHY_SPI_DESELECT();

#define DO_ECB_DECRYPT(data)                                    \
  PHY_SPI_SELECT();                                             \
  PHY_SPI_WRITE_BYTE(RF_CMD_SRAM_W);                            \
  PHY_SPI_WRITE_BYTE(AES_CTRL_REG);                             \
  PHY_SPI_WRITE_BYTE(MODEBYTE_FOR_ECBDECRYPT);                  \
  for (uint8_t i = 0; i < AES_BLOCKSIZE; i++)                   \
    PHY_SPI_WRITE_BYTE(data[i]);                                \
  PHY_SPI_WRITE_BYTE(MODEBYTE_FOR_START_OP|MODEBYTE_FOR_ECBDECRYPT); \
  PHY_SPI_DESELECT();

#define READ_AES_KEY(key)                                       \
  PHY_SPI_SELECT();                                             \
  PHY_SPI_WRITE_BYTE(RF_CMD_SRAM_W);                            \
  PHY_SPI_WRITE_BYTE(AES_CTRL_REG);                             \
  PHY_SPI_WRITE_BYTE(MODEBYTE_FOR_KEY_OP);                      \
  PHY_SPI_DESELECT();                                           \
                                                                \
  PHY_SPI_SELECT();                                             \
  PHY_SPI_WRITE_BYTE(RF_CMD_SRAM_R);                            \
  PHY_SPI_WRITE_BYTE(AES_STATE_REG);                            \
  for (uint8_t i = 0; i < AES_BLOCKSIZE; i++)                   \
    dataMinusOne[i] = PHY_SPI_WRITE_BYTE(0);                    \
  PHY_SPI_DESELECT();

#define READ_AES_DATA(data)                                     \
  PHY_SPI_SELECT();                                             \
  PHY_SPI_WRITE_BYTE(RF_CMD_SRAM_R);                            \
  PHY_SPI_WRITE_BYTE(AES_STATE_REG);                            \
  for (uint8_t i = 0; i < AES_BLOCKSIZE; i++)                   \
    data[i] = PHY_SPI_WRITE_BYTE(0);                            \
  PHY_SPI_DESELECT();

#define DO_ECB_ENCRYPT(data)                                    \
  PHY_SPI_SELECT();                                             \
  PHY_SPI_WRITE_BYTE(RF_CMD_SRAM_W);                            \
  PHY_SPI_WRITE_BYTE(AES_CTRL_REG);                             \
  PHY_SPI_WRITE_BYTE(MODEBYTE_FOR_ECBENCRYPT);                  \
  for (uint8_t i = 0; i < AES_BLOCKSIZE; i++)                   \
    PHY_SPI_WRITE_BYTE(data[i]);                                \
  PHY_SPI_WRITE_BYTE(MODEBYTE_FOR_START_OP|MODEBYTE_FOR_ECBENCRYPT); \
  PHY_SPI_DESELECT();

#define DO_ECB_ZERO_ENCRYPT()                                   \
  PHY_SPI_SELECT();                                             \
  PHY_SPI_WRITE_BYTE(RF_CMD_SRAM_W);                            \
  PHY_SPI_WRITE_BYTE(AES_CTRL_REG);                             \
  PHY_SPI_WRITE_BYTE(MODEBYTE_FOR_ECBENCRYPT);                  \
  for (uint8_t i = 0; i < AES_BLOCKSIZE; i++)                   \
    PHY_SPI_WRITE_BYTE(0);                                      \
  PHY_SPI_WRITE_BYTE(MODEBYTE_FOR_START_OP|MODEBYTE_FOR_ECBENCRYPT); \
  PHY_SPI_DESELECT();

#define WAIT_FOR_AES_OP_END()                                   \
  halDelay(AES_CORE_CYCLE_TIME_US)

/******************************************************************************
                   extern variables section
******************************************************************************/
/* payload for the API call */
extern endecryptReq_t req;

/******************************************************************************
                   Prototypes section
******************************************************************************/

/**************************************************************************//**
 \brief Setup AES unit

  This function performs the following tasks as part of the setup of the
  AES unit: key initialization, set encryption direction and encryption mode.

 \return  False if some parameter was illegal or operation failed, true else
**************************************************************************/
bool aesSetup(void);

/**************************************************************************//**
 \brief Reads the result of previous AES en/decryption
        This function returns the result of the previous AES operation

 \param[out] data - result of previous operation
**************************************************************************/
void aesRead(uint8_t *data);

/**************************************************************************//**
\brief En/decrypt one AES block.
       The function returns after the AES operation is finished.

       AES CBC decryption flow :

       key cipher1----              key cipher2----
        |     |       |              |     |
       ---------      |             ---------
       |decrypt|      |             |decrypt|
       ---------      |             ---------
           |          |                 |               .......
  iv ---- XOR         ---------------- XOR
           |                            |
      Plain text 1                  plain text 2

\param[in/out]  data - AES block to be en/decrypted
******************************************************************************/
void aesExec(uint8_t *data);

/****************** external functions ************************************/

/**************************************************************************//**
\brief Reads the last or previous 128-bit data processed

\return
  true - succesfull read operation
  false - failed to read
***************************************************************************/
bool phyReadLastEncData(uint8_t *data);

/**************************************************************************//**
\brief Clear any carry-forward data/feedback

\return
  true - succesfull operation
  false - failed
***************************************************************************/
bool phyClearHistory(void);

/**************************************************************************//**
\brief Phy call to encrypt/decrypt a cipher block

\param
  [in] cipher block
  [in] number of blocks of 16bytes

\return
  true - succesful en/decryption
  false - failed operation
***************************************************************************/
bool phyEnDecrypt(void);

/**************************************************************************//**
\brief Phy call to set mode for en/decryption

\param
  [in] AES mode
  [in] AES direction

\return
  true - succesfully set
  false - failed to set
***************************************************************************/
bool phySetModeAndDir(aesModeType_t mode, aesDirectionType_t dir);

/**************************************************************************//**
\brief Phy call to plumb key and IV for en/decryption

\param
  [in] 128-bit key
  [in] 128-bit IV

\return
  true - succesfully set
  false - failed to set
***************************************************************************/
bool phySetKeyAndIv(uint8_t *inKey, uint8_t *inIv);

#endif /* #if (USE_IMAGE_SECURITY == 1) */



#endif /* _PHY_H_ */

//eof phy.h
