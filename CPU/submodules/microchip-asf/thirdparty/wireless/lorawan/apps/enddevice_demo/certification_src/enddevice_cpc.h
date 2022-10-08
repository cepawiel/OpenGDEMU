/**
* \file enddevice_cpc.h
*
* \brief Certification Protocol Commands header
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

#ifndef ENDDEVICE_CPC_H
#define ENDDEVICE_CPC_H

#if (CERT_APP == 1)
/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
#define CERT_PROTOCOL_PACKAGE_IDENTIFIER    (6)
#define CERT_PROTOCOL_PACKAGE_VERSION       (1)

/*
* NOTE: command length is minimum valid length,
* inclusive of its identifer
*/

/* command identifiers */
#define CPC_PACKAGE_VERSION_REQ_LEN         (1)
#define CPC_PACKAGE_VERSION_REQ             (0x00)
#define CPC_PACKAGE_VERSION_ANS             CPC_PACKAGE_VERSION_REQ

#define CPC_DUT_RESET_REQ_LEN               (1)
#define CPC_DUT_RESET_REQ                   (0x01)
#define CPC_DUT_RESET_ANS                   CPC_DUT_RESET_REQ

#define CPC_DUT_JOIN_REQ_LEN                (1)
#define CPC_DUT_JOIN_REQ                    (0x02)

#define CPC_SWITCH_CLASS_REQ_LEN            (2)
#define CPC_SWITCH_CLASS_REQ                (0x03)

#define CPC_ADR_BIT_CHANGE_REQ_LEN          (2)
#define CPC_ADR_BIT_CHANGE_REQ              (0x04)

#define CPC_REGIONAL_DUTYCYCLE_CTRL_REQ_LEN (2)
#define CPC_REGIONAL_DUTYCYCLE_CTRL_REQ     (0x05)

#define CPC_TX_PERIODICITY_CHANGE_REQ_LEN   (2)
#define CPC_TX_PERIODICITY_CHANGE_REQ       (0x06)

//** can contain 0..N variable payload
#define CPC_TX_FRAMES_CTRL_REQ_LEN          (2)
#define CPC_TX_FRAMES_CTRL_REQ              (0x07)

//** can contain 0..N variable payload
#define CPC_ECHO_INC_PAYLOAD_REQ_LEN        (1)
#define CPC_ECHO_INC_PAYLOAD_REQ            (0x08)
#define CPC_ECHO_INC_PAYLOAD_ANS            CPC_ECHO_INC_PAYLOAD_REQ

#define CPC_RX_APP_CNT_REQ_LEN              (1)
#define CPC_RX_APP_CNT_REQ                  (0x09)
#define CPC_RX_APP_CNT_ANS                  CPC_RX_APP_CNT_REQ

#define CPC_RX_APP_CNT_RESET_REQ_LEN        (1)
#define CPC_RX_APP_CNT_RESET_REQ            (0x0A)

// ones that trigger MAC commands
#define CPC_LINK_CHECK_REQ_LEN              (1)
#define CPC_LINK_CHECK_REQ                  (0x20)

#define CPC_DEVICE_TIME_REQ_LEN             (1)
#define CPC_DEVICE_TIME_REQ                 (0x21)

#define CPC_PING_SLOT_INFO_REQ_LEN          (2)
#define CPC_PING_SLOT_INFO_REQ              (0x22)

// miscellaneous
#define CPC_TX_CW_REQ_LEN                   (7)
#define CPC_TX_CW_REQ                       (0x7D)

#define CPC_DUT_FPORT224_DISABLE_REQ_LEN    (1)
#define CPC_DUT_FPORT224_DISABLE_REQ        (0x7E)

#define CPC_DUT_VERSIONS_REQ_LEN            (1)
#define CPC_DUT_VERSIONS_REQ                (0x7F)
#define CPC_DUT_VERSIONS_ANS                CPC_DUT_VERSIONS_REQ

/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/
void enddevice_cpc_execute_command(uint8_t *buffer, uint8_t buflen);
bool enddevice_cpc_is_payload_valid(uint8_t *buffer, uint8_t len);

#endif /* #if (CERT_APP == 1) */
#endif /* #ifndef ENDDEVICE_CPC_H */
/* eof enddevice_cpc.h */
