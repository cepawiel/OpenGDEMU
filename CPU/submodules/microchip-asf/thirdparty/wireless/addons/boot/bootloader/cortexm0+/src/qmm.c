/**
* \file  qmm.c
*
* \brief This file implements the  functions for initializing the queues,
*  appending a buffer into the queue, removing a buffer from the queue and
*  reading a buffer from the queue as per the search criteria.
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
/* === Includes ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "bmm.h"
#include "qmm.h"
#include "bmm_config.h"
#include "atomic.h"

#if (TOTAL_NUMBER_OF_BUFS > 0)

/* === Types =============================================================== */

/*
 * Specifies whether the buffer needs to be read from the queue or to be
 * removed from the queue.
 */
typedef enum buffer_mode_tag {
	REMOVE_MODE,
	READ_MODE
} buffer_mode_t;

/* === Macros ============================================================== */

/* === Prototypes ========================================================== */


/* === Implementation ====================================================== */

/**
 * @brief Initializes the queue.
 *
 * This function initializes the queue. Note that this function
 * should be called before invoking any other functionality of QMM.
 *
 * @param q The queue which should be initialized.
 */
void qmm_queue_init(queue_t *q)
{
	q->head = NULL;
	q->tail = NULL;
	q->size = 0;
}

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
 */
void qmm_queue_append(queue_t *q, buffer_t *buf)

{

	ATOMIC_SECTION_ENTER

		/* Check whether queue is empty */
		if (q->size == 0) {
			/* Add the buffer at the head */
			q->head = buf;
		} else {
			/* Add the buffer at the end */
			q->tail->next = buf;
		}

		/* Update the list */
		q->tail = buf;

		/* Terminate the list */
		buf->next = NULL;

		/* Update size */
		q->size++;


	ATOMIC_SECTION_LEAVE


  
} /* qmm_queue_append */

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
 */
buffer_t *qmm_queue_remove(queue_t *q)
{
	buffer_t *buffer_current = NULL;
	buffer_t *buffer_previous;

	ATOMIC_SECTION_ENTER
	/* Check whether queue is empty */
	if (q->size != 0) {
		buffer_current = q->head;
		buffer_previous = q->head;

		
		/* Buffer matching with search criteria found */
		if (NULL != buffer_current) {
			/* Remove buffer from the queue */
			
				/* Update head if buffer removed is first node
				 **/
				if (buffer_current == q->head) {
					q->head = buffer_current->next;
				} else {
					/* Update the link by removing the
					 *buffer */
					buffer_previous->next
						= buffer_current->next;
				}

				/* Update tail if buffer removed is last node */
				if (buffer_current == q->tail) {
					q->tail = buffer_previous;
				}

				/* Update size */
				q->size--;

				if (NULL == q->head) {
					q->tail = NULL;
				}
			
			
		}
	} /* q->size != 0 */

	ATOMIC_SECTION_LEAVE

	/* Return the buffer. note that pointer to header of buffer is returned
	 **/
	return (buffer_current);
} /* queue_read_or_remove */





#endif  /* (TOTAL_NUMBER_OF_BUFS > 0) */

/* EOF */
