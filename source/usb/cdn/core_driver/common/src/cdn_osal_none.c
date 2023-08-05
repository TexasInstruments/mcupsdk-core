/**
 * @file   cdn_osal_none.c
 *
 * @brief  Cadence No os implementation
 */
/*
 * Copyright (c) 2019 - 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================*/

/* Includes */ 

#include "cdn_osal_none.h"

/********************************************************************************
 * Public Functions.
 ****************************************************************************** */


OsalNoRtosReturn_t OsalNoRtosQueue_init(
               OsalNoRtosQueue_t* queue,
               uint8_t* buffer,
               uint16_t bufferSize,
			   uint16_t maxDepth,
			   uint16_t eleSize )
{
	OsalNoRtosReturn_t status = OSAL_NO_RTOS_RETURN_SUCCESS ; 
	/* parameters sanity check */ 
	if ((queue != NULL) && (buffer != NULL) &&  \
			   (maxDepth != 0) && (eleSize != 0) && (bufferSize >= maxDepth*eleSize))
	{
		memset((void *) queue,0,sizeof(OsalNoRtosQueue_t));
		
		queue->fifo = buffer; 
		queue->maxDepth = maxDepth ; 
		queue->eleSize = eleSize ; 
	}
	else 
	{
		status = OSAL_NO_RTOS_RETURN_FAILURE ; 
	}

	return status; 
}

OsalNoRtosReturn_t OsalNoRtosQueue_enqueue(
            OsalNoRtosQueue_t* queue,
            const uint8_t* dataBuff,
            uint16_t size )
{
	OsalNoRtosReturn_t status = OSAL_NO_RTOS_RETURN_FAILURE; 
    uintptr_t key;
	volatile uint32_t rdIdx ;
    volatile uint32_t wrIdx ;
	key = HwiP_disable();
	rdIdx = queue->rdIdx; 
    wrIdx = queue->wrIdx;
	/* redundant size check with the global message size 
	                        to protect against buffer-overflow */ 
    if((rdIdx < queue->maxDepth) && (wrIdx < queue->maxDepth) \
	                                  && (size == queue->eleSize))
    {
        if( queue->currCount < queue->maxDepth )
        {
            /* There is some space in the FIFO write data to queue*/
            memcpy((queue->fifo + (queue->eleSize*wrIdx)),\
			                               dataBuff,queue->eleSize);
			
			queue->currCount += 1 ; 

            wrIdx = (wrIdx+1)%queue->maxDepth;

            queue->wrIdx = wrIdx;
            /* read back to ensure the update has reached the memory */
            wrIdx = queue->wrIdx; 

            status = OSAL_NO_RTOS_RETURN_SUCCESS;
        }
    }
	HwiP_restore(key);

    return status;
}


OsalNoRtosReturn_t OsalNoRtosQueue_dequeue(
            OsalNoRtosQueue_t* queue,
			uint8_t* dataBuff,
			uint16_t size )
{

	OsalNoRtosReturn_t status = OSAL_NO_RTOS_RETURN_FAILURE; 
    uintptr_t key;

	volatile uint32_t rdIdx; 
    volatile uint32_t wrIdx;
	
	key = HwiP_disable();
	rdIdx = queue->rdIdx; 
    wrIdx = queue->wrIdx;

	/* redundant size check with the global element size to 
	                          protect against buffer-overflow */ 
    if((rdIdx < queue->maxDepth) && (wrIdx < queue->maxDepth) && \
	                                    (size == queue->eleSize))
	{
		/* If there is someting to read in the fifo */ 
        if(queue->currCount > 0)
        {
            /* Copy EleSize bytes from Queue memory to the buffer */
            memcpy(dataBuff, (queue->fifo + (queue->eleSize*rdIdx)), \
			                                         queue->eleSize);

			queue->currCount -= 1 ; 

            rdIdx = (rdIdx+1)%queue->maxDepth;

            queue->rdIdx = rdIdx;
            /* read back to ensure the update has reached the memory */
            rdIdx = queue->rdIdx; 
			
            status = OSAL_NO_RTOS_RETURN_SUCCESS;
        }
    }
	HwiP_restore(key);

    return status;
}

OsalNoRtosReturn_t OsalNoRtosQueue_isEmpty(OsalNoRtosQueue_t* queue)
{
	if(queue->currCount == 0 )
	{
		return OSAL_NO_RTOS_RETURN_SUCCESS;
	}
	else
	{
		return OSAL_NO_RTOS_RETURN_FAILURE;
	}
}

uint16_t OsalNoRtosQueue_getMaxDepth(OsalNoRtosQueue_t* queue)
{
	return queue->maxDepth ;

}

uint16_t OsalNoRtosQueue_getCurCount(OsalNoRtosQueue_t* queue)
{
	return queue->currCount ; 
}