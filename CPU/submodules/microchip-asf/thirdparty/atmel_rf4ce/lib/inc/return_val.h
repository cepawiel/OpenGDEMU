/**
 * @file return_val.h
 *
 * @brief Return values of APIs
 *
 * This header file has enumeration of return values.
 *
 * $Id: return_val.h 26685 2011-05-16 11:38:33Z sschneid $
 *
 * @author    Microchip Technology Inc: http://www.microchip.com
 * @author    Support: https://www.microchip.com/support/
 */
/*
 * Copyright (c) 2009-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * 
 */

/* Prevent double inclusion */
#ifndef RETURN_VAL_H
#define RETURN_VAL_H

/* === Includes ============================================================ */


/* === Externals =========================================================== */


/* === Types =============================================================== */

/**
 * These are the return values of the PAL API.
 */
typedef enum
#if !defined(DOXYGEN)
retval_tag
#endif
{
    MAC_SUCCESS                 = 0x00, /**< Success defined by 802.15.4 */

#ifdef ENABLE_RTB
    RTB_SUCCESS                 = 0x10, /**< Success of ranging procedure */
    RTB_RANGING_IN_PROGRESS     = 0x11, /**< Ranging procedure is already in progress */
    RTB_REJECT                  = 0x12, /**< Ranging procedure is rejected */
    RTB_UNSUPPORTED_ATTRIBUTE   = 0x13, /**< Unsupported attribute for RTB */
    RTB_INVALID_PARAMETER       = 0x14, /**< Unsupported parameter for RTB */
    RTB_OUT_OF_BUFFERS          = 0x15, /**< No further buffers available for RTB ranging measurement */
    RTB_UNSUPPORTED_RANGING     = 0x16, /**< Ranging is currently not supported */
    RTB_UNSUPPORTED_METHOD      = 0x17, /**< Requested Ranging method is currently not supported at reflector */
    RTB_TIMEOUT                 = 0x18, /**< Timeout since requested Ranging response frame is not received */
#endif

    TAL_TRX_ASLEEP              = 0x81, /**< Transceiver is currently sleeping */
    TAL_TRX_AWAKE               = 0x82, /**< Transceiver is currently awake */
    FAILURE                     = 0x85, /**< General failure condition */
    TAL_BUSY                    = 0x86, /**< TAL busy condition */
    TAL_FRAME_PENDING           = 0x87, /**< Frame pending at TAL */
    PAL_TMR_ALREADY_RUNNING     = 0x88, /**< Timer is already running */
    PAL_TMR_NOT_RUNNING         = 0x89, /**< Timer is not running */
    PAL_TMR_INVALID_ID          = 0x8A, /**< Invalid timer ID*/
    PAL_TMR_INVALID_TIMEOUT     = 0x8B, /**< Requested Timeout is out of range or too short */
    QUEUE_FULL                  = 0x8C, /**< Requested queue is full */

    MAC_COUNTER_ERROR           = 0xDB, /**< Invalid Frame counter value defined by 802.15.4 */
    MAC_IMPROPER_KEY_TYPE       = 0xDC, /**< Unallowed key in received frame defined by 802.15.4 */
    MAC_IMPROPER_SECURITY_LEVEL = 0xDD, /**< Required security level not applied within received frame defined by 802.15.4 */
    MAC_UNSUPPORTED_LEGACY      = 0xDE, /**< Received frame was secured using 802.15.4-2003 security */
    MAC_UNSUPPORTED_SECURITY    = 0xDF, /**< Not supported applied security defined by 802.15.4 */
    MAC_BEACON_LOSS             = 0xE0, /**< Loss of beacons defined by 802.15.4 */
    MAC_CHANNEL_ACCESS_FAILURE  = 0xE1, /**< Channel access failure defined by 802.15.4 */
    MAC_DISABLE_TRX_FAILURE     = 0xE3, /**< Disabling of TRX failed defined by 802.15.4 */
    MAC_SECURITY_ERROR          = 0xE4, /**< Failed cryptographic processing of the received secured frame defined by 802.15.4 */
    MAC_FRAME_TOO_LONG          = 0xE5, /**< Current frame is too long defined by 802.15.4 */
    MAC_INVALID_GTS             = 0xE6, /**< Invalid GTS defined by 802.15.4 */
    MAC_INVALID_HANDLE          = 0xE7, /**< Invalid handle defined by 802.15.4 */
    MAC_INVALID_PARAMETER       = 0xE8, /**< Invalid Parameter defined by 802.15.4 */
    MAC_NO_ACK                  = 0xE9, /**< No ACK defined by 802.15.4 */
    MAC_NO_BEACON               = 0xEA, /**< No beacon defined by 802.15.4 */
    MAC_NO_DATA                 = 0xEB, /**< NO data defined by 802.15.4 */
    MAC_NO_SHORT_ADDRESS        = 0xEC, /**< No valid short address defined by 802.15.4 */
    MAC_OUT_OF_CAP              = 0xED, /**< Out of CA defined by 802.15.4-2003, deprecated in 802.15.4-2006 */
    MAC_PAN_ID_CONFLICT         = 0xEE, /**< PAN ID conflict defined by 802.15.4 */
    MAC_REALIGNMENT             = 0xEF, /**< Realignment defined by 802.15.4 */
    MAC_TRANSACTION_EXPIRED     = 0xF0, /**< Transaction expired defined by 802.15.4 */
    MAC_TRANSACTION_OVERFLOW    = 0xF1, /**< Transaction overflow defined by 802.15.4 */
    MAC_TX_ACTIVE               = 0xF2, /**< Tx active defined by 802.15.4 */
    MAC_UNAVAILABLE_KEY         = 0xF3, /**< Unavailable key or blacklisted device defined by 802.15.4 */
    MAC_UNSUPPORTED_ATTRIBUTE   = 0xF4, /**< Unsupported attribute defined by 802.15.4 */
    MAC_INVALID_ADDRESS         = 0xF5, /**< Invalid address defined by 802.15.4 */
    MAC_PAST_TIME               = 0xF7, /**< Receiver could not be enabled as defined by 802.15.4 */
    MAC_INVALID_INDEX           = 0xF9, /**< Invalid index in table of MAC PIB attribute defined by 802.15.4 */
    MAC_LIMIT_REACHED           = 0xFA, /**< Scan operation terminated prematurely defined by 802.15.4 */
    MAC_READ_ONLY               = 0xFB, /**< SET request issued for read only attribute defined by 802.15.4 */
    MAC_SCAN_IN_PROGRESS        = 0xFC  /**< Scan operation failed because of ongoing scan defined by 802.15.4 */
} SHORTENUM retval_t;

/* === Macros ============================================================== */


/* === Prototypes ========================================================== */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RETURN_VAL_H */
/* EOF */
