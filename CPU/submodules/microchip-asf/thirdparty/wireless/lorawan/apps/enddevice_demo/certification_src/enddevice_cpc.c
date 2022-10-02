/**
* \file enddevice_cpc.c
*
* \brief Certification Protocol Commands implementation
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
#if (CERT_APP == 1)
/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include "asf.h"
#include "lorawan.h"
#include "radio_interface.h"
#include "radio_driver_hal.h"
#include "enddevice_cpc.h"
#include "enddevice_cert.h"
#include "conf_certification.h"
#include "sio2host.h"
#include "sw_timer.h"
#if (ENABLE_PDS == 1)
#include "pds_interface.h"
#endif

/************************************************************************/
/* TYPE DEFINES                                                         */
/************************************************************************/

/************************************************************************/
/* STATIC FUNCTION PROTOTYPES                                           */
/************************************************************************/
static void enddevice_txcw_callback(void);
static void enddevice_cpc_fresh_join(void);

/************************************************************************/
/* EXTERN VARTIABLES                                                    */
/************************************************************************/
extern uint8_t testMode;
extern uint8_t sendData[100];
extern uint16_t sendDataLen;
extern uint16_t downlinkCtr;
extern bool bTxCnf;
extern uint8_t uplinkTestNoResp;
extern bool pktRxd;
extern uint8_t certAppTimerId;
extern uint32_t certAppTimeout;
extern TransmissionType_t txType;
extern LorawanSendReq_t lorawanSendReq;
extern IsmBand_t current_band;
extern bool clear_linkcheck;
extern uint8_t tx_cw_timer;
//extern bool reset_test_mode;
extern uint8_t devEui[8];
extern uint8_t joinEui[8];
extern uint8_t appKey[16];
extern uint8_t bandTable[];
extern uint8_t sendFport;

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/

/************************************************************************/
/* FUNCTION DEFINITIONS                                                 */
/************************************************************************/
void enddevice_cpc_execute_command(uint8_t *buffer, uint8_t buflen)
{
  StackRetStatus_t status;
  //printf("\r\n<-- "); 
  switch (buffer[0])
  {
    case CPC_PACKAGE_VERSION_REQ:
    {
      //printf("PackageVersionReq");
      sendData[0] = CPC_PACKAGE_VERSION_ANS;
      sendData[1] = CERT_PROTOCOL_PACKAGE_IDENTIFIER;
      sendData[2] = CERT_PROTOCOL_PACKAGE_VERSION;
      sendDataLen = 3;
      sendFport = TEST_PORT_NB;
    }
    break;
    
    case CPC_DUT_RESET_REQ:
    {
      //printf("DutResetReq");
      NVIC_SystemReset();
    }
    break;
    
    case CPC_DUT_JOIN_REQ:
    {
      //printf("DutJoinReq");
      enddevice_cpc_fresh_join();
    }
    break;
    
    case CPC_SWITCH_CLASS_REQ:
    {
      EdClass_t target_class = (1 << buffer[1]);
      //printf("SwitchClassReq");
      printf("\nSwitch to class %c:%sOK\n\r",
        (1 == target_class) ? 'A' : (2 == target_class) ? 'B' : 'C',
        (LORAWAN_SUCCESS == LORAWAN_SetAttr(EDCLASS, &target_class)) ? 
            " " : "NOT");
    }
    break;
    
    case CPC_ADR_BIT_CHANGE_REQ:
    {
      bool new_adr;
      //printf("AdrBitChangeReq");
      LORAWAN_SetAttr(ADR, &buffer[1]);
      LORAWAN_GetAttr(ADR, NULL, &new_adr);
      //printf("\r\nADR bit: %d", (uint8_t)new_adr);
    }
    break;
    
    case CPC_REGIONAL_DUTYCYCLE_CTRL_REQ:
    {
      bool new_regdcycle;
      //printf("RegionalDutyCycleCtrlReq");
      LORAWAN_SetAttr(REGIONAL_DUTY_CYCLE, &buffer[1]);
      LORAWAN_GetAttr(REGIONAL_DUTY_CYCLE, NULL, &new_regdcycle);
      //printf("\r\nRegional DutyCycle status: %d", (uint8_t)new_regdcycle);
    }
    break;
    
    case CPC_TX_PERIODICITY_CHANGE_REQ:
    {
      uint16_t periodicity[11] = {CERT_APP_TIMEOUT, 5, 10, 20, 30,
                                    40, 50, 60, 120, 240, 480};
      //printf("TxPeriodicityChangeReq");
      if (10 >= buffer[1])
      {
        certAppTimeout = 1000 * periodicity[buffer[1]];
        if (SwTimerIsRunning(certAppTimerId))
        {
          SwTimerStop(certAppTimerId);
        }
        SwTimerStart(certAppTimerId, MS_TO_US(certAppTimeout),
            SW_TIMEOUT_RELATIVE, cert_app_timer_callback, NULL);
        //printf("\r\nNew periodicity: %ds", periodicity[buffer[1]]);
      }
    }
    break;
    
    case CPC_TX_FRAMES_CTRL_REQ:
    {
      //printf("TxFramesCtrlReq");
      switch (buffer[1])
      {
        case 0:
        {
          bTxCnf = bTxCnf;
          //printf("\r\nNo change in FrameType");
        }
        break;
        case 1:
        {
          bTxCnf = false;
          //printf("\r\nChanged to UNCNF frames");
        }
        break;
        case 2:
        {
          bTxCnf = true;
          //printf("\r\nChanged to CNF frames");
        }
        break;
        default:
        {
          // keep compiler happy
          //printf("\r\nError: RFU values in FrameType field");
        }
        break;
      }
    }
    break;
    
    case CPC_ECHO_INC_PAYLOAD_REQ:
    {
      uint8_t tx_size;
      //printf("EchoIncPayloadReq\r\nRxPayload: ");
      //for (uint8_t i = 1; i < buflen; i++)
      //{
      //    printf("%02X", buffer[i]);
      //}
      
      LORAWAN_GetAttr(NEXT_PAYLOAD_SIZE, NULL, &tx_size);
      tx_size = (tx_size <= (buflen - 1)) ? tx_size : (buflen - 1);
      sendData[0] = CPC_ECHO_INC_PAYLOAD_ANS;
      for (uint8_t i = 0; i < tx_size; i++)
      {
          sendData[i+1] = (1 + buffer[i+1]) % UINT8_MAX;
      }
      sendDataLen = tx_size + 1;
      //printf("\r\n--> EchoIncPayloadAns\r\nTxPayload: ");
      //for (uint8_t i = 0; i < sendDataLen; i++)
      //{
      //    printf("%02X", sendData[i]);
      //}
      sendFport = TEST_PORT_NB;
  }
  break;
  
  case CPC_RX_APP_CNT_REQ:
  {
      //printf("RxAppCntReq");
      sendData[0] = CPC_RX_APP_CNT_ANS;
      sendData[1] = (uint8_t) (downlinkCtr >> 8);
      sendData[2] = (uint8_t) (downlinkCtr);
      sendDataLen = 3;
      //printf("\r\n--> RxAppCntAns\r\nTxPayload: ");
      //for (uint8_t i = 0; i < sendDataLen; i++)
      //{
      //    printf("%02X", sendData[i]);
      //}
      sendFport = TEST_PORT_NB;
    }
    break;
    
    case CPC_RX_APP_CNT_RESET_REQ:
    {
      //printf("RxAppCntRstReq");
      downlinkCtr = 0;
    }
    break;
    
    case CPC_LINK_CHECK_REQ:
    {
      //printf("CpcLinkCheckReq");
      LORAWAN_SetAttr(SEND_LINK_CHECK_CMD, NULL);
    }
    break;
    
    case CPC_DEVICE_TIME_REQ:
    {
      //printf("CpcDeviceTimeReq");
      LORAWAN_SetAttr(SEND_DEVICE_TIME_CMD, NULL);
    }
    break;
    
    case CPC_PING_SLOT_INFO_REQ:
    {
      // TODO: Pull class-B patch
    }
    break;
    
    case CPC_TX_CW_REQ:
    {
      uint8_t txpower = 0;
      uint16_t timeout = 0;
      uint32_t freq = 0;
      
      //printf("TxCwReq");
      
      timeout = buffer[2];
      timeout |= ((uint16_t)buffer[1]) << 8;
      
      freq = buffer[5];
      freq |= ((uint32_t)buffer[4]) << 8;
      freq |= ((uint32_t)buffer[3]) << 16;
      freq *= 100U;
      
      txpower = buffer[6];
      
      RADIO_SetAttr(CHANNEL_FREQUENCY, (void *)&freq);
      RADIO_SetAttr(OUTPUT_POWER, (void *)&txpower);
      
      if (LORAWAN_SUCCESS == SwTimerStart(tx_cw_timer, MS_TO_US(timeout * 1000U),
        SW_TIMEOUT_RELATIVE, enddevice_txcw_callback, NULL))
      {
          if (ERR_NONE == RADIO_TransmitCW())
          {
              //printf("\r\nRadio CW on");
          }
          else
          {
              //printf("\r\nError: Radio failed to start CW TX");
          }
      }
      else
      {
          //printf("\r\nError: Unable to start CW TX timer");
      }
    }
    break;
    
    case CPC_DUT_FPORT224_DISABLE_REQ:
    {
      bool enable = false;
      //printf("DutFport224DisableReq\r\n");
      LORAWAN_SetAttr(TEST_MODE_ENABLE, &enable);
#if (ENABLE_PDS == 1)
      PDS_DeleteAll();
#endif
      delay_ms(1000);
      NVIC_SystemReset();
    }
    break;
    
    case CPC_DUT_VERSIONS_REQ:
    {
      //printf("DutVersionsReq");
      sendData[0] = CPC_DUT_VERSIONS_ANS;
      // FwVersion
      sendData[1] = 1; // 1
      sendData[2] = 0; // 0
      sendData[3] = 4; // P_4
      // LrwanVersion
      sendData[4] = 1;
      sendData[5] = 0;
      sendData[6] = 4;
      // LrwanRpVersion
      sendData[7] = 1;
      sendData[8] = 0;
      sendData[9] = 4;
      
      sendDataLen = 10;
      sendFport = TEST_PORT_NB;
    }
    break;
  }
  (void)status;
}

bool enddevice_cpc_is_payload_valid(uint8_t *buffer, uint8_t len)
{
  bool valid = false;
  switch (*buffer)
  {
    case CPC_TX_FRAMES_CTRL_REQ:
    case CPC_ECHO_INC_PAYLOAD_REQ:
    {
      valid = (len >= 1);
    }
    break;
    
    case CPC_PACKAGE_VERSION_REQ:
    case CPC_DUT_RESET_REQ:
    case CPC_DUT_JOIN_REQ:
    case CPC_RX_APP_CNT_REQ:
    case CPC_RX_APP_CNT_RESET_REQ:
    case CPC_LINK_CHECK_REQ:
    case CPC_DEVICE_TIME_REQ:
    case CPC_DUT_FPORT224_DISABLE_REQ:
    case CPC_DUT_VERSIONS_REQ:
    {
      valid = (1 == len); 
    }
    break;
    
    case CPC_SWITCH_CLASS_REQ:
    case CPC_ADR_BIT_CHANGE_REQ:
    case CPC_REGIONAL_DUTYCYCLE_CTRL_REQ:
    case CPC_TX_PERIODICITY_CHANGE_REQ:
    case CPC_PING_SLOT_INFO_REQ:
    {
      valid = (2 == len);
    }
    break;
    
    case CPC_TX_CW_REQ:
    {
      valid = (7 == len);
    }
    break;
  }
  
  return valid;
}

static void enddevice_txcw_callback(void)
{
    if (ERR_NONE == RADIO_StopCW())
    {
        //printf("\r\nTX CW stopped");
    }
    else
    {
        //printf("\r\nError in stopping CW, pls check");
    }
}

static void enddevice_cpc_fresh_join(void)
{
    StackRetStatus_t status;
    bool trueval = true;
#ifdef CRYPTO_DEV_ENABLED
    LORAWAN_SetAttr(CRYPTODEVICE_ENABLED, &trueval );
#endif /* #ifdef CRYPTO_DEV_ENABLED */
    SwTimerStop(certAppTimerId);
    LORAWAN_Reset(bandTable[current_band]);
#if (ENABLE_PDS == 1)
    PDS_RestoreAll();
#endif
    LORAWAN_SetAttr(TEST_MODE_ENABLE, &trueval);
    LORAWAN_SetAttr(DEV_EUI, devEui);
    LORAWAN_SetAttr(JOIN_EUI, joinEui);
    LORAWAN_SetAttr(APP_KEY, appKey);
    LORAWAN_SetAttr(ADR, &trueval);
    status = LORAWAN_Join(LORAWAN_OTAA);
    printf("\nOTAA Join Request Sent\n\r");
    (void)status;
}

#endif /* #if (CERT_APP == 1) */
/* eof enddevice_cpc.c */
