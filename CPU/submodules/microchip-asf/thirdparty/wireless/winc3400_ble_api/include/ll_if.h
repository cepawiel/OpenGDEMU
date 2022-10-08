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
 * ll_if.h
 *
 */

#ifndef LL_IF_H_
#define LL_IF_H_

#define BD_ADDR_LEN         0x06
#define ADV_DATA_LEN        0x1F
#define SCAN_RSP_DATA_LEN   0x1F

/// Minimal authorized MTU value (defined by Bluetooth SIG)
#define L2C_MIN_LE_MTUSIG                   (23)

///Advertising HCI Type
enum
{
    ///Connectable Undirected advertising
    ADV_CONN_UNDIR                = 0x00,
    ///Connectable directed advertising
    ADV_CONN_DIR,
    ///Discoverable undirected advertising
    ADV_DISC_UNDIR,
    ///Non-connectable undirected advertising
    ADV_NONCONN_UNDIR,
    ///Enumeration end value for advertising type value check
    ADV_END
};

///Scanning HCI Type
enum
{
    ///Scan request
    SCAN_REQ,
    ///Scan response
    SCAN_RSP,
    ///Enumeration end value for scanning type value check
    SCAN_LEN
};

///BD address type
enum
{
    ///Public BD address
    ADDR_PUBLIC                   = 0x00,
    ///Random BD Address
    ADDR_RAND,
    ///Enumeration end value for BD address type value check
    ADDR_END
};

///Advertising channels enables
enum
{
    ///Byte value for advertising channel map for channel 37 enable
    ADV_CHNL_37_EN                = 0x01,
    ///Byte value for advertising channel map for channel 38 enable
    ADV_CHNL_38_EN,
    ///Byte value for advertising channel map for channel 39 enable
    ADV_CHNL_39_EN                = 0x04,
    ///Byte value for advertising channel map for channel 37, 38 and 39 enable
    ADV_ALL_CHNLS_EN              = 0x07,
    ///Enumeration end value for advertising channels enable value check
    ADV_CHNL_END
};


///Advertising enables
enum
{
    ///Disable advertising
    ADV_DIS                       = 0x00,
    ///Enable advertising
    ADV_EN,
    ///Enumeration end value for advertising enable value check
    ADV_EN_END
};

///LE Scan type
enum
{
    ///Passive scan
    SCAN_PASSIVE                  = 0x00,
    ///Active scan
    SCAN_ACTIVE,
    ///Enumeration end value for scan type value check
    SCAN_END
};

///Scan filter policy
enum
{
    ///Allow advertising packets from anyone
    SCAN_ALLOW_ADV_ALL            = 0x00,
    ///Allow advertising packets from White List devices only
    SCAN_ALLOW_ADV_WLST,
    ///Enumeration end value for scan filter policy value check
    SCAN_ALLOW_ADV_END
};

///Le Scan enables
enum
{
    ///Disable scan
    SCAN_DIS                      = 0x00,
    ///Enable scan
    SCAN_EN,
    ///Enumeration end value for scan enable value check
    SCAN_EN_END
};

///BD Address structure
struct bd_addr
{
    ///6-byte array address value
    uint8_t     addr[BD_ADDR_LEN];
};

///Advertising data structure
struct adv_data
{
    ///Maximum length data bytes array
    uint8_t data[ADV_DATA_LEN];
};

///Scan response data structure
struct scan_rsp_data
{
    ///Maximum length data bytes array
    uint8_t data[SCAN_RSP_DATA_LEN];
};


///LLM LE Set Advertising Parameters Command parameters structure
struct llm_le_set_adv_param_cmd
{
    ///Minimum interval for advertising
    uint16_t       adv_intv_min;
    ///Maximum interval for advertising
    uint16_t       adv_intv_max;
    ///Advertising type
    uint8_t        adv_type;
    ///Own address type: public=0x00 /random = 0x01
    uint8_t        own_addr_type;
    ///Direct address type: public=0x00 /random = 0x01
    uint8_t        direct_addr_type;
    ///Direct Bluetooth device address
    struct bd_addr direct_addr;
    ///Advertising channel map
    uint8_t        adv_chnl_map;
    ///Advertising filter policy
    uint8_t        adv_filt_policy;
};

///LLM LE Set Advertising Data Command parameters structure
struct llm_le_set_adv_data_cmd
{
    ///Advertising data length
    uint8_t         adv_data_len;
    ///Advertising data - maximum 31 bytes
    struct adv_data data;
};

///LLM LE Set Scan Response Data Command parameters structure
struct llm_le_set_scan_rsp_data_cmd
{
    ///Scan response data length
    uint8_t              scan_rsp_data_len;
    ///Scan response data - maximum 31 bytes
    struct scan_rsp_data data;
};

///LLM LE Set Scan Parameters Command parameters structure
struct llm_le_set_scan_param_cmd
{
    ///Scan type - 0=passive / 1=active
    uint8_t        scan_type;
    ///Scan interval
    uint16_t       scan_intv;
    ///Scan window size
    uint16_t       scan_window;
    ///Own address type - 0=public, 1=random
    uint8_t        own_addr_type;
    ///Scan filter policy
    uint8_t        scan_filt_policy;
};
#endif /* LL_IF_H_ */
