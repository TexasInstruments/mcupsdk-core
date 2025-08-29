/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** 
 * @file   dwc_queue.c
 *
 * @brief  DWC queue implementation
 * @addtogroup misc_api_grp Miscellaneous API Routines
 * @{
 */

#include "dwc_queue.h"
#include <string.h>
#include <kernel/dpl/HwiP.h>


/**
 * @brief This routine initializes the queue
 * 
 * @param queue			Pointer to the instance of queue
 */
void dwc_queueInit(dwc_queue_t *queue)
{
	memset(queue->buffer, 0, sizeof(uint32_t) * DWC_QUEUE_SIZE);
	queue->front = 0;
	queue->rear = 0;
}

/**
 * @brief This routine adds an element to the queue
 * 
 * @param queue	Pointer to the instance of queue
 * @param data	Data to be inserted in the queue
 * 
 * @return true if the element was successfully added, false if the queue is full
 */
bool dwc_queuePut(dwc_queue_t *queue, uint32_t data)
{
	bool retval = false;

	uintptr_t  key = HwiP_disable();

	if ((queue->rear + 1) % (DWC_QUEUE_SIZE) != queue->front)
	{
		queue->buffer[queue->rear] = data;
        queue->rear = (queue->rear + 1) % DWC_QUEUE_SIZE;
		retval = true;
	}

	HwiP_restore(key);
	
	return retval;
}

/**
 * @brief This routine retrieves an elements from the queue
 * 
 * @param queue	Pointer to the instance of queue
 * @param data	Pointer to a variable where the data will be stored
 * 
 * @return true if an element was successfully retrieved, false if the queue is empty
 */
bool dwc_queueGet(dwc_queue_t *queue, uint32_t *data)
{
	bool retval = false;

	uintptr_t  key = HwiP_disable();

	if((data != NULL) && (queue->front != queue->rear) )
	{
		*data = queue->buffer[queue->front];
		queue->front = (queue->front + 1) % DWC_QUEUE_SIZE;

		retval = true;
	}

	HwiP_restore(key);

	return retval;
}
/** @} */