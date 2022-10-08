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
/*
 * dbg_task.h
 *
 */
#ifndef ERROR_H_
#define ERROR_H_

#include "cmn_defs.h"

at_ble_status_t at_ble_att_error(uint8_t att_error_code);

at_ble_status_t at_ble_gap_error(uint8_t gap_error_code);


#define IS_ERR(__status__) (__status__ != AT_BLE_SUCCESS)

#define ERRORCHECK(__status__) do{\
    if(IS_ERR(__status__))\
    {\
        printf("ERRORCHECK(%d)\n", __status__);\
        goto ERRORHANDLER;\
    }\
}while(0)

#define ERRORREPORT(__status__, __err__) do{\
    printf("NMI_ERRORREPORT(%d)\n", __err__);\
    __status__ = __err__;\
    goto ERRORHANDLER;\
}while(0)

#define  NULLCHECK(__status__, __ptr__) do{\
    if(__ptr__ == NULL)\
    {\
        ERRORREPORT(__status__, AT_BLE_FAILURE);\
    }\
}while(0)

#define CATCH(__status__) \
ERRORHANDLER :\
if(IS_ERR(__status__)) \

#endif

