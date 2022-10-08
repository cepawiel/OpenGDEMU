/**
* \file  bmm.c
*
* \brief This file implements the functions for initializing buffer module,
 *  allocating and freeing up buffers.
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
#include <stdlib.h>
#include <stdbool.h>
#include "bmm.h"
#include "qmm.h"
#include "bmm_config.h"
   
#if (TOTAL_NUMBER_OF_BUFS > 0)

/*
 * Check if the buffer configuration does not exceed the queue limits.
 * Note: The queue's size parameter is a 8bit value.
 */
#if (TOTAL_NUMBER_OF_BUFS > 255)
#error "Number of buffer exceeds its limit"
#endif

/* === Types =============================================================== */

/* === Macros ============================================================== */



/* === Globals ============================================================= */
/**
 * Common Buffer pool holding the buffer user area
 */

static uint8_t buf_pool[((TOTAL_NUMBER_OF_LARGE_BUFS * LARGE_BUFFER_SIZE))];


/*
 * Array of buffer headers
 */
static buffer_t buf_header[TOTAL_NUMBER_OF_LARGE_BUFS];

/*
 * Queue of free large buffers
 */
#if (TOTAL_NUMBER_OF_LARGE_BUFS > 0)
static queue_t free_large_buffer_q;
#endif



/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

/**
 * @brief Initializes the buffer module.
 *
 * This function initializes the buffer module.
 * This function should be called before using any other functionality
 * of buffer module.
 */
void bmm_buffer_init(void)
{
	uint8_t index;

	/* Initialize free buffer queue for large buffers */
#if (TOTAL_NUMBER_OF_LARGE_BUFS > 0)
   	qmm_queue_init(&free_large_buffer_q);
#endif



#if (TOTAL_NUMBER_OF_LARGE_BUFS > 0)
	for (index = 0; index < TOTAL_NUMBER_OF_LARGE_BUFS; index++) {
		/*
		 * Initialize the buffer body pointer with address of the
		 * buffer body
		 */
		buf_header[index].body = buf_pool + (index * LARGE_BUFFER_SIZE);

		/* Append the buffer to free large buffer queue */
		qmm_queue_append(&free_large_buffer_q, &buf_header[index]);
	}
#endif

}

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
 */
buffer_t *bmm_buffer_alloc(void)
{
	buffer_t *pfree_buffer = NULL;

	/* Allocate buffer from free large buffer queue */
	pfree_buffer = qmm_queue_remove(&free_large_buffer_q);

	return pfree_buffer;
}

/**
 * @brief Frees up a buffer.
 *
 * This function frees up a buffer. The pointer passed to this function
 * should be the pointer returned during buffer allocation. The result is
 * unpredictable if an incorrect pointer is passed.
 *
 * @param pbuffer Pointer to buffer that has to be freed.
 */
void bmm_buffer_free(buffer_t *pbuffer)
{
	if (NULL == pbuffer) {
		/* If the buffer pointer is NULL abort free operation */
		return;
	}

	/* Append the buffer into free large buffer queue */
	qmm_queue_append(&free_large_buffer_q, pbuffer);

}

#endif /* (TOTAL_NUMBER_OF_BUFS > 0) */
/* EOF */
