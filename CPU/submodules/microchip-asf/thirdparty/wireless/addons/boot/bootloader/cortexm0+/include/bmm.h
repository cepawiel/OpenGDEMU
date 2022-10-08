/**
* \file  bmm.h
*
* \brief This file contains the Buffer Management Module definitions.
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

/* Prevent double inclusion */
#ifndef BMM_INTERFACE_H
#define BMM_INTERFACE_H

/* === Includes ============================================================ */

#include <stdint.h>
#include <stdbool.h>

#define LARGE_BUFFER_SIZE    (140)


#if (SAMD20) || (SAM4L)
#define __PACK__RST_DATA__  COMPILER_PACK_RESET()
#define __PACK__DATA__		COMPILER_PACK_SET(1)
#else
#define __PACK__RST_DATA__
#define __PACK__DATA__
#endif

/* This macro saves the global interrupt status */
#define ENTER_CRITICAL_REGION()              {uint8_t flags = cpu_irq_save();

	/* This macro restores the global interrupt status */
#define LEAVE_CRITICAL_REGION()              cpu_irq_restore(flags);}

/**
 * \defgroup group_resources  Resource Management
 * The Resource Management provides access to resources  to the stack or the
 * application.
 *  @{
 */

/**
 * \defgroup group_bmm  Buffer Management
 * Buffer Management (large and small buffers): provides services for
 *dynamically
 * allocating and freeing memory buffers.
 *  @{
 */

/* === Macros ============================================================== */

/**
 * This macro provides the pointer to the corresponding body of the supplied
 *buffer header.
 */

#define BMM_BUFFER_POINTER(buf)  ((buf)->body) 

/* === Types =============================================================== */

/**
 * @brief Buffer structure holding information of each buffer.
 *
 */
__PACK__DATA__
typedef struct
#if !defined(__DOXYGEN__)
		buffer_tag
#endif
{
	/** Pointer to the buffer body */
	uint8_t *body;
	
	/** Pointer to next free buffer */
	struct buffer_tag *next;
} buffer_t;
__PACK__RST_DATA__
/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the buffer module.
 *
 * This function initializes the buffer module.
 * This function should be called before using any other functionality
 * of buffer module.
 *
 */
void bmm_buffer_init(void);

/**
 * @brief Allocates a buffer
 *
 * This function allocates a buffer and returns a pointer to the buffer.
 * The same pointer should be used while freeing the buffer.User should
 * call BMM_BUFFER_POINTER(buf) to get the pointer to buffer user area.
 *
 * @param size size of buffer to be allocated.
 *
 * @return pointer to the buffer allocated,
 *  NULL if buffer not available.
 *
 */
buffer_t *bmm_buffer_alloc(void);

/**
 * @brief Frees up a buffer.
 *
 * This function frees up a buffer. The pointer passed to this function
 * should be the pointer returned during buffer allocation. The result is
 * unpredictable if an incorrect pointer is passed.
 *
 * @param pbuffer Pointer to buffer that has to be freed.
 *
 */
void bmm_buffer_free(buffer_t *pbuffer);

/* ! @} */
#endif /* BMM_INTERFACE_H */

/* EOF */
