/**
 * @file stack_config.h
 *
 * @brief Stack configuration parameters
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
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
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
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
 * Copyright (c) 2013-2018, Microchip Technology Inc All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef STACK_CONFIG_H
#define STACK_CONFIG_H

#include "compiler.h"
#include "tal_types.h"
#include "ieee_const.h"
#if (TAL_TYPE == AT86RF215)
#include "ieee_154g.h"
#endif

/**
 * \ingroup group_inc
 * \defgroup group_stack Common Stack Definitions
 * Includes Stack Definitions
 * @{
 *
 */

/* Highest stack layer definitions up to MAC
 *
 * Do NOT change order here! The numbering needs to start with the lowest layer
 * increasing to the higher layers.
 */
#define PAL                                 (1)
#define TAL                                 (2)
#define RTB                                 (3)
#define MAC                                 (4)

/*
 * Layers above MAC are defined further below, since the enumeration
 * depends on the usage of the switch VENDOR_STACK_CONFIG.
 */

#ifdef VENDOR_STACK_CONFIG
#include "vendor_stack_config.h"
#else   /* Use standard stack layer configurations as defined below. */

/* Highest stack layer definitions above MAC */
#define RF4CE                               (5)

#if (!defined HIGHEST_STACK_LAYER)
#define HIGHEST_STACK_LAYER     MAC
#endif /* (!defined HIGHEST_STACK_LAYER) */

#if (HIGHEST_STACK_LAYER == PAL)

/*
 * +----------+
 +          +
 +   PAL    +
 +          +
 +++++----------+
 */
/* Reduce the header file dependency by using hard-coded values */
#define LARGE_BUFFER_SIZE               (160)
#define SMALL_BUFFER_SIZE               (68)

#elif (HIGHEST_STACK_LAYER == TAL)

/*
 * +----------+
 +          +
 +   TAL    +
 +          +
 +++++----------+
 +          +
 +   PAL    +
 +          +
 +++++----------+
 */

/**
 * The following macros hold the size of a large buffer.
 * Additional octets for the length of the frame, the LQI
 * and the ED value are required.
 */

#if (MEGA || XMEGA)

/*
 * Size of frame_info_t + max number of payload octets +
 * 1 octet LQI  + 1 octet ED value.
 */
#if (TAL_TYPE == AT86RF215)
#define LARGE_BUFFER_SIZE                   (sizeof(frame_info_t) + \
	aMaxPHYPacketSize_4g + \
	LQI_LEN + ED_VAL_LEN)
#else
#define LARGE_BUFFER_SIZE                   (sizeof(frame_info_t) + \
	aMaxPHYPacketSize + \
	LENGTH_FIELD_LEN + LQI_LEN + ED_VAL_LEN)
#endif

/**
 * The following macro holds the size of a small buffer.
 * Additional octets for the length of the frame, the LQI
 * and the ED value are required.
 */
#define SMALL_BUFFER_SIZE                   (sizeof(frame_info_t) + \
	MAX_MGMT_FRAME_LENGTH +	\
	LENGTH_FIELD_LEN + LQI_LEN + ED_VAL_LEN)
#elif (UC3 || SAM)

/*
 * Size of frame_info_t + max number of payload octets +
 * 1 octet LQI  + 1 octet ED value.
 * The buffer size has to be a DWORD.
 */
#if (TAL_TYPE == AT86RF215)
#define LARGE_BUFFER_SIZE                   (sizeof(frame_info_t) + \
	aMaxPHYPacketSize_4g + \
	LQI_LEN + ED_VAL_LEN)
#else
#define LARGE_BUFFER_SIZE                   (((sizeof(frame_info_t) + \
	aMaxPHYPacketSize + \
	LENGTH_FIELD_LEN + LQI_LEN + ED_VAL_LEN) / 4 + 1) * 4)
#endif

/**
 * The following macro holds the size of a small buffer.
 * Additional octets for the length of the frame, the LQI
 * and the ED value are required.
 */
#define SMALL_BUFFER_SIZE                   (((sizeof(frame_info_t) + \
	MAX_MGMT_FRAME_LENGTH +	\
	LENGTH_FIELD_LEN + LQI_LEN + ED_VAL_LEN) / 4 + 1) * 4)
#else
#error "Unknown Device family for buffer calculation"
#endif

#elif ((HIGHEST_STACK_LAYER == MAC) || (HIGHEST_STACK_LAYER == RTB))

/*
 * +----------+    *----------+
 +          +    +          +
 +   MAC    +    +   RTB    +
 +          +    +          +
 +++++----------+    +----------+
 +          +    +          +
 +   TAL    + or +   TAL    +
 +          +    +          +
 +++++----------+    +----------+
 +          +    +          +
 +   PAL    +    +   PAL    +
 +          +    +          +
 +++++----------+    +----------+
 */

/**
 * The following macros hold the size of a large buffer.
 * Additional octets for the length of the frame, the LQI
 * and the ED value are required.
 */
#include "mac_msg_type_data_ind.h"
#define MCPS_DATA_IND_SIZE                  (sizeof(mcps_data_ind_t))

#if (MEGA || XMEGA)

/*
 * Size of mcps_data_ind_t + max number of payload octets +
 * 1 octet LQI  + 1 octet ED value.
 */
#define LARGE_BUFFER_SIZE                   (MCPS_DATA_IND_SIZE + \
	aMaxMACPayloadSize + \
	FCS_LEN + \
	LQI_LEN + ED_VAL_LEN)

/**
 * The following macro holds the size of a small buffer.
 * Additional octets for the length of the frame, the LQI
 * and the ED value are required.
 */
#define SMALL_BUFFER_SIZE                   (sizeof(frame_info_t) + \
	MAX_MGMT_FRAME_LENGTH +	\
	LENGTH_FIELD_LEN + LQI_LEN + ED_VAL_LEN)
#elif (UC3 || SAM)

/*
 * Size of mcps_data_ind_t + max number of payload octets +
 * 1 octet LQI  + 1 octet ED value.
 * The buffer size has to be a DWORD.
 */
#define LARGE_BUFFER_SIZE                   (((MCPS_DATA_IND_SIZE + \
	aMaxMACPayloadSize + \
	LQI_LEN + ED_VAL_LEN) / 4 + 1) * 4)

/**
 * The following macro holds the size of a small buffer.
 * Additional octets for the length of the frame, the LQI
 * and the ED value are required.
 */
#define SMALL_BUFFER_SIZE                   (((sizeof(frame_info_t) + \
	MAX_MGMT_FRAME_LENGTH +	\
	LENGTH_FIELD_LEN + LQI_LEN + ED_VAL_LEN) / 4 + 1) * 4)
#else
#error "Unknown Device family for buffer calculation"
#endif

#elif (HIGHEST_STACK_LAYER == RF4CE)
#include "nwk_config.h"

#else
#error "Unknown HIGHEST_STACK_LAYER for buffer calculation"
#endif  /* #if (HIGHEST_STACK_LAYER == ...) */

/* Configuration if PAL is the highest stack layer */
#if (HIGHEST_STACK_LAYER == PAL)
#define NUMBER_OF_TOTAL_STACK_TIMERS        (0)
#define NUMBER_OF_LARGE_STACK_BUFS          (0)
#define NUMBER_OF_SMALL_STACK_BUFS          (0)
#endif  /* (HIGHEST_STACK_LAYER == PAL) */

/* Configuration if TAL is the highest stack layer */
#if (HIGHEST_STACK_LAYER == TAL)
#include "tal_build_config.h"
#include "tal_config.h"
#define NUMBER_OF_TOTAL_STACK_TIMERS        (NUMBER_OF_TAL_TIMERS)
#define NUMBER_OF_LARGE_STACK_BUFS          (3)
#define NUMBER_OF_SMALL_STACK_BUFS          (0)
#endif  /* (HIGHEST_STACK_LAYER == TAL) */

#ifdef ENABLE_RTB
/* Configuration if RTB is the highest stack layer */
#if (HIGHEST_STACK_LAYER == RTB)
#include "rtb_config.h"
#define NUMBER_OF_TOTAL_STACK_TIMERS        (NUMBER_OF_TAL_TIMERS + \
	NUMBER_OF_RTB_TIMERS)
/* Minimum number of large buffers is 3, but one more is given. */
#define NUMBER_OF_LARGE_STACK_BUFS          (4)
#define NUMBER_OF_SMALL_STACK_BUFS          (0)
#else

/*
 * In case RTB is enabled for a stack using higher layers,
 * extra buffers are required for proper RTB operation.
 */
#define EXTRA_RTB_BUFFER                    (3)
#endif
#else
#define EXTRA_RTB_BUFFER                        (0)
#define NUMBER_OF_RTB_TIMERS                    (0)
#endif  /* ENABLE_RTB */

/* Configuration if MAC is the highest stack layer */
#if (HIGHEST_STACK_LAYER == MAC)
#include "mac_config.h"
#define NUMBER_OF_TOTAL_STACK_TIMERS        (NUMBER_OF_TAL_TIMERS + \
	NUMBER_OF_RTB_TIMERS + NUMBER_OF_MAC_TIMERS)
#ifdef GTS_SUPPORT
#if (MAC_INDIRECT_DATA_FFD == 1)
#define NUMBER_OF_LARGE_STACK_BUFS      (6 + 7 + EXTRA_RTB_BUFFER)
#else
#define NUMBER_OF_LARGE_STACK_BUFS      (4 + 2 + EXTRA_RTB_BUFFER)
#endif  /* (MAC_INDIRECT_DATA_FFD == 1) */
#else /* GTS_SUPPORT */
#if (MAC_INDIRECT_DATA_FFD == 1)
#define NUMBER_OF_LARGE_STACK_BUFS      (6 + EXTRA_RTB_BUFFER)
#else
#define NUMBER_OF_LARGE_STACK_BUFS      (4 + EXTRA_RTB_BUFFER)
#endif  /* (MAC_INDIRECT_DATA_FFD == 1) */
#endif  /* GTS_SUPPORT */
#define NUMBER_OF_SMALL_STACK_BUFS          (0)
#endif  /* (HIGHEST_STACK_LAYER == MAC) */

#endif  /* #ifdef VENDOR_STACK_CONFIG */
/* ! @} */
#endif /* STACK_CONFIG_H */
/* EOF */
