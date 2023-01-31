/**
 * @file   cdn_osal_none.h
 *
 * @brief  Cadence no os implementation
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

#ifndef CDN_OSAL_NORTOS_H_
#define CDN_OSAL_NORTOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cdn_stdtypes.h"
#include <kernel/dpl/HwiP.h>

#define OSAL_NO_RTOS_RETURN_SUCCESS (1)
#define OSAL_NO_RTOS_RETURN_FAILURE (0)
typedef struct OsalNoRtosQueue_t_
{
    uint32_t rdIdx; /**<queue element will be read from this index.*/
    uint32_t wrIdx; /**<queue element will be written to this index.*/
    uint16_t eleSize ; /**<Element size in bytes, this will be a fixed parameter */
    uint16_t maxDepth ; /**< max number of elements in a queue*/
    uint16_t currCount ; /**< total number of messages in a queue */
    uint8_t *fifo; /**Pointer to the FIFO queue queue element */
} OsalNoRtosQueue_t;

typedef struct
{
  uint16_t depth;  /**<queue depth .*/
  uint16_t item_sz; /**<size of each queue element .*/
  void*    buf; /**<queue buffer pointer*/

  OsalNoRtosQueue_t *queue; /**<Noos queue type*/
}osal_queue_def_t;

typedef osal_queue_def_t* osal_queue_t;
typedef int32_t OsalNoRtosReturn_t;

/* role device/host is used by OS NONE for mutex (disable usb isr) only*/
#define OSAL_QUEUE_DEF(_role, _name, _depth, _type) \
  _type _name##_##buf[_depth];\
  OsalNoRtosQueue_t _name##_##queue; \
  osal_queue_def_t _name = { .depth = _depth, .item_sz = sizeof(_type), .buf = _name##_##buf, .queue =  &_name##_##queue };

/**
 * @brief
 * This function is used for initializing the OsalNoRtosQueue instance.
 * It expects buffer pointer and the size of buffer, thus buffer can be
 * static or dynamic. If it is dynamic then it is application
 *  responsibility to free it.
 * @param queue - [In] OsalNoRtosQueue_t instance
 * @param buffer - [In] Pointer to memory buffer
 * @param bufferSize - [In] size of the buffer
 * @param maxDepth - [In] max possible element that this queue can store
 * @param eleSize - [In] size of a single queue element.
 *
 * @return - OSAL_NO_RTOS_RETURN_SUCCESS if bufferSize >= maxDepth*eleSize and buffer is
 *          not a null pointer else return OSAL_NO_RTOS_RETURN_FAILURE
 */
OsalNoRtosReturn_t OsalNoRtosQueue_init(OsalNoRtosQueue_t* queue,uint8_t* buffer,uint16_t bufferSize,
                           uint16_t maxDepth,uint16_t eleSize);

/**
 * @brief
 * This function is used to add a new element to the queue
 *
 * @param queue - [In] OsalNoRtosQueue_t instance
 * @param dataBuff - [In] pointer to data which will be enqueued
 * @param size - [In] size of data which will be enqueued
 *
 * @return - OSAL_NO_RTOS_RETURN_FAILURE if dataBuff is a null pointer
 *           or queue size is full or data size not equal to
 *            queue->eleSize else return OSAL_NO_RTOS_RETURN_SUCCESS
 */
OsalNoRtosReturn_t OsalNoRtosQueue_enqueue(OsalNoRtosQueue_t* queue,\
                                     const uint8_t* dataBuff,uint16_t size);

/**
 * @brief
 * This function is used to remove an element from the queue
 *
 * @param queue -  [In] OsalNoRtosQueue_t instance
 * @param buffer - [Out] pointer to buffer which contain read value
 * @param size - [In] size of data which will be dequeued, This is a redundant parameter
 *                    to protect against buffer overflow.
 *
 * @return - OSAL_NO_RTOS_RETURN_FAILURE if dataBuff is a null
 *           pointer or queue size is zero else return
 *            OSAL_NO_RTOS_RETURN_SUCCESS
 */
OsalNoRtosReturn_t OsalNoRtosQueue_dequeue(OsalNoRtosQueue_t* queue,\
                                     uint8_t* dataBuff,uint16_t size);

/**
 * @brief
 * check if queue is empty or not
 *
 * @param queue - [In] OsalNoRtosQueue_t instance
 *
 * @return - OSAL_NO_RTOS_RETURN_SUCCESS if queue is empty
 *           else return OSAL_NO_RTOS_RETURN_FAILURE
 *
 */
OsalNoRtosReturn_t OsalNoRtosQueue_isEmpty(OsalNoRtosQueue_t* queue);

/**
 * @brief
 * get maxDepth parameter of queue.
 *
 * @param queue - [In] OsalNoRtosQueue_t instance
 *
 * @return - Max number of elements that this queue can hold
 *
 */
uint16_t OsalNoRtosQueue_getMaxDepth(OsalNoRtosQueue_t* queue);

/**
 * @brief
 * get current count of number of elements
 * present in the queue
 *
 * @param queue - [In] OsalNoRtosQueue_t instance
 *
 * @return - number of elements that are there in this
 *           queue.
 */
uint16_t OsalNoRtosQueue_getCurCount(OsalNoRtosQueue_t* queue);

/**
 * @brief
 * Wrapper around  OsalNoRtosQueue_enqueue for OSAL
 *
 * @param queue - [In] osal_queue_t Queue handle
 *                [In] data data to be stored in Queue
 *                [In] in_isr Dummy Param
 *
 * @return - status of enqueue operation.
 */
static inline bool osal_queue_send(osal_queue_t qhdl, \
                             void const * data, bool in_isr)
{
  return OsalNoRtosQueue_enqueue(qhdl->queue, \
                       (const uint8_t*)data, qhdl->queue->eleSize);
}
/**
 * @brief
 * Wrapper around  OsalNoRtosQueue_dequeue for OSAL
 *
 * @param queue - [In] osal_queue_t Queue handle
 *                [In] data retrieved data
 *
 * @return - Getting Queue element status.
 */

static inline bool osal_queue_receive(osal_queue_t qhdl, void* data)
{
    return OsalNoRtosQueue_dequeue(qhdl->queue, \
                        (uint8_t *)data, qhdl->queue->eleSize);
}
/**
 * @brief
 * Wrapper around  OsalNoRtosQueue_init for OSAL
 *
 * @param queue - [In] osal_queue_def_t Queue definition
 *
 * @return - Queue handle.
 */

static inline osal_queue_t osal_queue_create(osal_queue_def_t* qdef)
{
   if (OSAL_NO_RTOS_RETURN_SUCCESS == \
       OsalNoRtosQueue_init(qdef->queue,\
             (uint8_t *)qdef->buf, (qdef->depth * qdef->item_sz), \
                                       qdef->depth, qdef->item_sz))
   {
       return qdef;
   }
   else
   {
       return NULL;
   }
}
/**
 * @brief
 * Wrapper around  OsalNoRtosQueue_isEmpty for OSAL
 *
 * @param queue - [In] osal_queue_t Queue handle
 *
 * @return - status of Queue if its empty.
 */

static inline bool osal_queue_empty(osal_queue_t qhdl)
{
  return (OsalNoRtosQueue_isEmpty((OsalNoRtosQueue_t*)&(qhdl->queue)) == OSAL_NO_RTOS_RETURN_SUCCESS);
}


/* Dummy Definitions; Will be implemented on need bases.*/
/*--------------------------------------------------------------------+
* MUTEX API (priority inheritance)
*--------------------------------------------------------------------*/

typedef uint32_t osal_mutex_t  ;
typedef uint32_t osal_mutex_def_t ;
static inline osal_mutex_t osal_mutex_create(osal_mutex_def_t* mdef)
{
  return 0;
}

static inline bool osal_mutex_lock (osal_mutex_t mutex_hdl, uint32_t msec)
{
  return 0;
}

static inline bool osal_mutex_unlock(osal_mutex_t mutex_hdl)
{
  return 0;
}
typedef uint32_t osal_semaphore_t  ;
typedef uint32_t osal_semaphore_def_t ;

static inline osal_semaphore_t osal_semaphore_create(osal_semaphore_def_t* semdef)
{
    return 0;
}

static inline bool osal_semaphore_post(osal_semaphore_t sem_hdl, bool in_isr)
{
    return 0;
}

static inline bool osal_semaphore_wait (osal_semaphore_t sem_hdl, uint32_t msec)
{
    return 0;
}

static inline void osal_semaphore_reset(osal_semaphore_t const sem_hdl)
{
    return;
}

#ifdef __cplusplus
}
#endif

#endif /*CDN_OSAL_NORTOS_H_*/
