/*******************************************************************************
Copyright (c) RivieraWaves 2009-2014
Copyright (c) 2017 released Microchip Technology Inc. All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

#ifndef __INTERFACE_H__
#define __INTERFACE_H__

#include "event.h"

#define INTERFACE_HDR_LENGTH        9

#define INTERFACE_API_PKT_ID            0x05

#define INTERFACE_SEND_BUF_MAX  600
#define INTERFACE_RCV_BUFF_LEN 500

extern uint8_t interface_send_msg[INTERFACE_SEND_BUF_MAX];

#define INTERFACE_MSG_INIT(msg_id, dest_id) \
do{\
    uint8_t* __ptr = interface_send_msg;\
    /*uint16_t __len;*/\
    *__ptr++ = (INTERFACE_API_PKT_ID);\
    *__ptr++ = ((msg_id) & 0x00FF );\
    *__ptr++ = (((msg_id)>>8) & 0x00FF );\
    *__ptr++ = ((dest_id) & 0x00FF );\
    *__ptr++ = (((dest_id)>>8) & 0x00FF );\
    *__ptr++ = ((TASK_EXTERN) & 0x00FF );\
    *__ptr++ = (((TASK_EXTERN)>>8) & 0x00FF );\
    __ptr += 2

#define INTERFACE_PACK_ARG_UINT8(arg)\
    *__ptr++ = (arg)

#define INTERFACE_PACK_ARG_UINT16(arg)\
    *__ptr++ = ((arg) & 0x00FF);\
    *__ptr++ = (((arg) >> 8) & 0x00FF)

#define INTERFACE_PACK_ARG_UINT32(arg) \
    *__ptr++ = (uint8_t)((arg) & 0x00FF );\
    *__ptr++ = (uint8_t)(( (arg) >> 8) & 0x00FF) ;\
    *__ptr++ = (uint8_t)(( (arg) >> 16) & 0x00FF);\
    *__ptr++ = (uint8_t)(( (arg) >> 24) & 0x00FF)

#define INTERFACE_PACK_ARG_BLOCK(ptr,len)\
    memcpy(__ptr, ptr, len);\
    __ptr += len

#define INTERFACE_PACK_ARG_DUMMY(len)\
    __ptr += len

#define INTERFACE_PACK_LEN()\
    __len = __ptr - &interface_send_msg[INTERFACE_HDR_LENGTH];\
    interface_send_msg[7] = ((__len) & 0x00FF );\
    interface_send_msg[8] = (((__len)>>8) & 0x00FF);\
    __len += INTERFACE_HDR_LENGTH;
/*
#define INTERFACE_SEND_NO_WAIT()\
    INTERFACE_PACK_LEN();\
    interface_send(interface_send_msg, __len)
*/
#define INTERFACE_SEND_NO_WAIT()\
    interface_send(interface_send_msg, (__ptr - &interface_send_msg[INTERFACE_HDR_LENGTH]))
/*
#define INTERFACE_SEND_WAIT(msg, src)\
    watched_event.msg_id = msg;\
    watched_event.src_id = src;\
    INTERFACE_PACK_LEN();\
    if (interface_send(interface_send_msg, __len) != AT_BLE_SUCCESS){return AT_BLE_FAILURE;};\
    if(platform_cmd_cmpl_wait()){return AT_BLE_FAILURE;}\
    __ptr = watched_event.params;\
*/
#define INTERFACE_SEND_WAIT(msg_id, src_id)\
	interface_send_wait(interface_send_msg, (__ptr - &interface_send_msg[INTERFACE_HDR_LENGTH]), msg_id, src_id, &__ptr)

#define INTERFACE_MSG_DONE()\
}while(0)

#define INTERFACE_UNPACK_INIT(ptr)\
do{\
    uint8_t* __ptr = (uint8_t*)(ptr);\

#define INTERFACE_UNPACK_UINT8(ptr)\
    *ptr = *__ptr++

#define INTERFACE_UNPACK_UINT16(ptr)\
    *ptr = (uint16_t)__ptr[0]\
        | ((uint16_t)__ptr[1] << 8);\
    __ptr += 2

#define INTERFACE_UNPACK_UINT32(ptr)\
    *ptr = (uint32_t)__ptr[0] \
        | ((uint32_t)__ptr[1] << 8) \
        | ((uint32_t)__ptr[2] << 16)\
        | ((uint32_t)__ptr[3] << 24);\
    __ptr += 4

#define INTERFACE_UNPACK_BLOCK(ptr, len)\
    memcpy(ptr, __ptr, len);\
    __ptr += len

#define INTERFACE_UNPACK_SKIP(len)\
    __ptr += (len)

#define INTERFACE_UNPACK_DONE()\
}while(0)

at_ble_status_t interface_send(uint8_t* msg, uint16_t u16TxLen);
at_ble_status_t interface_send_wait(uint8_t* msg, uint16_t u16TxLen, uint16_t msg_id, uint16_t src_id, uint8_t**rsp);

#endif /* HCI_H_ */