/**
* \file  qmm.h
*
* \brief This file contains the Queue Management Module definitions.
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
#ifndef QMM_INTERFACE_H
#define QMM_INTERFACE_H

/* === Includes ============================================================ */

#include "bmm.h"

/**
 * \ingroup group_resources
 * \defgroup group_qmm  Queue Management
 * Queue Management: provides services for creating and maintaining the queues.
 *  @{
 */

/* === Macros ============================================================== */

/* === Types =============================================================== */

/**
 * @brief Structure to search for a buffer to be removed from a queue
 */
__PACK__DATA__

/**
 * @brief Queue structure
 *
 * This structure defines the queue structure.
 * The application should declare the queue of type queue_t
 * and call qmm_queue_init before invoking any other functionality of qmm.
 *
 */
typedef struct
#if !defined(__DOXYGEN__)
		queue_tag
#endif
{
	/** Pointer to head of queue */
	buffer_t *head;
	/** Pointer to tail of queue */
	buffer_t *tail;

	/**
	 * Number of buffers present in the current queue
	 */
	uint8_t size;
} queue_t;
__PACK__RST_DATA__
/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Initializes the queue.
 *
 * This function initializes the queue. Note that this function
 * should be called before invoking any other functionality of QMM.
 *
 * @param q The queue which should be initialized.
 *
 */

void qmm_queue_init(queue_t *q);

/**
 * @brief Appends a buffer into the queue.
 *
 * This function appends a buffer into the queue.
 *
 * @param q Queue into which buffer should be appended
 *
 * @param buf Pointer to the buffer that should be appended into the queue.
 * Note that this pointer should be same as the
 * pointer returned by bmm_buffer_alloc.
 *
 */

void qmm_queue_append(queue_t *q, buffer_t *buf);


/**
 * @brief Removes a buffer from queue.
 *
 * This function removes a buffer from queue
 *
 * @param q Queue from which buffer should be removed
 *
 * @param search Search criteria. If this parameter is NULL, first buffer in the
 * queue will be removed. Otherwise buffer matching the criteria will be
 * removed.
 *
 * @return Pointer to the buffer header, if the buffer is
 * successfully removed, NULL otherwise.
 *
 */
buffer_t *qmm_queue_remove(queue_t *q);




#ifdef __cplusplus
} /* extern "C" */
#endif

/* ! @} */
#endif /* QMM_INTERFACE_H */

/* EOF */
