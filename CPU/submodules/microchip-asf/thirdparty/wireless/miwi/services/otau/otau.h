/**
* \file otau.h
*
* \brief OTAU interface
*
* Copyright (c) 2018 - 2019 Microchip Technology Inc. and its subsidiaries.
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

#ifndef OTAU_H
#define OTAU_H

#include "compiler.h"

/*- Definitions ------------------------------------------------------------*/

/************************************************************************/
/* Over-the-air Configuration parameters                                */
/************************************************************************/
#define MODE_MASK     0x03

#define APP_MODE_SUPPORTED_BIT 0x01

#define SUPPORTED_MODE (APP_MODE_SUPPORTED_BIT)

/* Broadcast Address size */
#define BROADCAST_ADDR_SIZE 2
/* Native Address size */
#define NATIVE_ADDR_SIZE    2
/* Extended address size */
#define EXTENDED_ADDR_SIZE  8

#define APP_MAX_PAYLOAD_SIZE  (64)

#define PHY_MAX_PAYLOAD_SIZE  (80)

#if SAM0
/* Address for User page to retrieve IEEE and Short address */
#define NVM_APP_SHORT_ADDR            (0x80400EL)
#define NVM_APP_IEEE_ADDR             (0x804008L)
#endif
/* Address of end of flash for storing device information */
#if SAM0
#define NVM_DEVICE_INFO_LOCATION      ((FLASH_ADDR + FLASH_SIZE) - 64UL)
#endif
/* Address of mid of Flash page with 64byte at end for storing device information */
#if SAM0
#define OFFSET_ADDRESS    (0x1EFE0UL)
#endif

#define DATA_HANDLE_TABLE_SIZE   20

typedef enum{
	ENTRY = 0,
	EXIT
}SHORTENUM error_code_t;

typedef enum{
	TRACE_ENTRY = 0,
	TRACE_EXIT
}SHORTENUM trace_type_t;

typedef enum{
	LOG_INFO = 0x01,
	LOG_WARNING = 0x02,
	LOG_ERROR = 0x04,
	LOG_ASSERT = 0x08
}SHORTENUM log_type_t;

typedef enum {
	MIWI_NWK,
	MIWI_PHY,
	MIWI_SYS,
	MIWI_APP
}SHORTENUM module_id_t;

/**
 * The mask for the ACK request bit of the FCF
 */
#define FCF_ACK_REQUEST                 (1 << 5)

/**
 * The mask for the PAN ID compression bit of the FCF
 */
#define FCF_PAN_ID_COMPRESSION          (1 << 6)

/**
 * Address Mode: NO ADDRESS
 */
#define FCF_NO_ADDR                     (0x00)

/**
 * Address Mode: RESERVED
 */
#define FCF_RESERVED_ADDR               (0x01)

/**
 * Address Mode: SHORT
 */
#define FCF_SHORT_ADDR                  (0x02)

/**
 * Address Mode: LONG
 */
#define FCF_LONG_ADDR                   (0x03)

/**
 * Defines the offset of the destination address
 */
#define FCF_DEST_ADDR_OFFSET            (10)

/**
 * Defines the offset of the source address
 */
#define FCF_SOURCE_ADDR_OFFSET          (14)

/**
 * Macro to set the source address mode
 */
#define FCF_SET_SOURCE_ADDR_MODE(x)     ((unsigned int)((x) << \
	FCF_SOURCE_ADDR_OFFSET))

/**
 * Macro to set the destination address mode
 */
#define FCF_SET_DEST_ADDR_MODE(x)       ((unsigned int)((x) << \
	FCF_DEST_ADDR_OFFSET))

/**
 * Defines a mask for the frame type. (Table 65 IEEE 802.15.4 Specification)
 */
#define FCF_FRAMETYPE_MASK              (0x07)

/**
 * A macro to set the frame type.
 */
#define FCF_SET_FRAMETYPE(x)            (x)

/**
 * The mask for the security enable bit of the FCF.
 */
#define FCF_SECURITY_ENABLED            (1 << 3)

/**
 * Macro to get the frame type.
 */
#define FCF_GET_FRAMETYPE(x)            ((x) & FCF_FRAMETYPE_MASK)

/*
 * Defines the mask for the FCF address mode
 */
#define FCF_ADDR_MASK                   (3)

/*
 * Macro to get the source address mode.
 */
#define FCF_GET_SOURCE_ADDR_MODE(x) \
	(((x) >> FCF_SOURCE_ADDR_OFFSET) & FCF_ADDR_MASK)

/*
 * Macro to get the destination address mode.
 */
#define FCF_GET_DEST_ADDR_MODE(x) \
	(((x) >> FCF_DEST_ADDR_OFFSET) & FCF_ADDR_MASK)
/**
 * Length (in octets) of FCF
 */
#define FCF_LEN                             (2)

/**
 * Length (in octets) of FCS
 */
#define FCS_LEN                             (2)

/**
 * Length of the sequence number field
 */
#define SEQ_NUM_LEN                         (1)

/**
 * Length (in octets) of extended address
 */
#define EXT_ADDR_LEN                        (8)


/**
 * Length (in octets) of PAN ID
 */
#define PAN_ID_LEN                          (2)

/**
 * Default value for PIB macShortAddress
 */
#define macShortAddress_def             (0xFFFF)

#define FRAME_OVERHEAD                      (3)


typedef enum {
	OTAU_SUCCESS = 0x00,
	OTAU_CCA_FAILURE,
	OTAU_NO_ACK,
	OTAU_ERROR,
	OTAU_INVALID_MSG = 0x80,
	OTAU_MSG_IN_PROGRESS
}SHORTENUM error_status_t;

typedef enum {
	TIMER_MODE_SINGLE,
	TIMER_MODE_PERIODIC,
}SHORTENUM otau_timer_mode_t;

typedef enum {
	BROADCAST_MODE = 0,
	NATIVE_ADDR_MODE,
	EXTENDED_ADDR_MODE,
}SHORTENUM addr_mode_t;

typedef enum {
	DOMAIN_OTAU_NOTIFY = 0x80,
	DOMAIN_OTAU_UPGRADE,
	DOMAIN_OTAU_DEBUG,
}SHORTENUM otau_domain_t;

typedef enum {
	OTAU_LED_ON,
	OTAU_LED_OFF,
	OTAU_LED_TOGGLE
}SHORTENUM otau_led_t;

COMPILER_PACK_SET(1)
typedef struct DataHandleTable_
{
	uint16_t nativeAddr;
	uint8_t addrMode;
	uint8_t dataHandle;
	uint8_t domainId;
	uint8_t messageId;
}DataHandleTable_t;

typedef struct {
	uint16_t domain_len;
	uint8_t domainId;
	uint8_t domain_msg;
}otau_domain_msg_t;

typedef struct {
	uint8_t *src_addr;
	uint8_t *frame_payload;
	uint16_t frame_length;
	addr_mode_t addr_mode;
}otau_rcvd_frame_t;
COMPILER_PACK_RESET()

typedef struct {
	uint8_t extended_addr[8];
	uint16_t native_addr;
}node_address_t;

extern node_address_t serverAddress;

void otauInit(void);
void otauTask(void);
void otauHandleMsg(otau_domain_msg_t *otau_domain_msg);
void otauRcvdFrame(otau_rcvd_frame_t *rcvd_frame);
void otau_sent_frame(uint8_t addr_mode, uint8_t *addr, uint8_t domainId, uint8_t status);
void otauDataSend(addr_mode_t addr_mode, uint8_t *addr, void *payload, uint16_t len);
void otauTimerStart(otau_domain_t domain_code, uint32_t interval, otau_timer_mode_t mode);
void otauTimerStop(otau_domain_t domain_code);
uint8_t *get_node_address(addr_mode_t addr_mode);
void otauLed(otau_led_t led_state);
void otauResetDevice(void);
void reverseMemcpy(uint8_t *dst, uint8_t *src, uint8_t len);
#ifndef OTAU_SERVER
void otauSetServerDetails(addr_mode_t addr_mode,uint8_t *addr);
void otauGetServerDetails(addr_mode_t addr_mode,uint8_t *addr);
#endif
void otau_log(uint8_t log_type, module_id_t module_id, error_code_t error_code, uint8_t len, uint8_t* user_log);
void otau_trace(trace_type_t trace_type);
#endif /* OTAU_H */

