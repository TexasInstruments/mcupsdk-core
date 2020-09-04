/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#ifndef QUEUEP_H
#define QUEUEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

/**
 * \defgroup KERNEL_DPL_QUEUE APIs for Queue
 * \ingroup KERNEL_DPL
 *
 * For more details and example usage, see \ref KERNEL_DPL_QUEUE_PAGE
 *
 * @{
 */

/**
 *  \anchor QUEUE_State
 *  \name State codes for current queue state
 *
 *  @{
 */
#define QueueP_NOTEMPTY               (0U)
#define QueueP_EMPTY                  (1U)
/** @} */

/*!
 *  @brief    Opaque client reference to an instance of a QueueP
 *
 *  A QueueP_Handle returned from the ::QueueP_create represents that
 *  instance and  is used in the other instance based functions
 */
typedef  void *QueueP_Handle;

/*!
 *  @brief  Opaque QueueP element
 *
 *  Structure that defines a single queue element and/or a list of queue elements.
 *  A field of this type is placed at the head of client structs.
 *
 */
typedef struct QueueP_Elem_s{
    /*! Pointer to the next queue element */
    struct QueueP_Elem_s *next;

    /*! Pointer to the previous queue element */
    struct QueueP_Elem_s *prev;
} QueueP_Elem;

/**
 * \brief Max size of task object across all OS's
 */
#define QueueP_OBJECT_SIZE_MAX       (8u)
/**
 * \brief Opaque task object used with the task APIs
 */
typedef struct QueueP_Object_ {

    /* uintptr_t translates to uint64_t for A53 and uint32_t for R5 and M4 */
    /* This accounts for the 64bit pointer in A53 and 32bit pointer in R5 and M4 */
    uintptr_t rsv[QueueP_OBJECT_SIZE_MAX/sizeof(uint32_t)]; /**< reserved, should NOT be modified by end users */

} QueueP_Object;

/*!
 *  @brief  Function to create a queue.
 *
 *  @param  obj  [in] Pointer to QueueP_Object.
 *
 *  @return A QueueP_Handle on success or a NULL on an error
 */
QueueP_Handle QueueP_create(QueueP_Object *obj);

/*!
 *  @brief  Function to delete a queue.
 *
 *  @param  handle  [in] A QueueP_Handle returned from QueueP_create
 *
 *  @return Status of the functions
 *    - QueueP_OK: Deleted the queue instance
 *    - QueueP_FAILURE: Failed to delete the queue instance
 */
int32_t QueueP_delete(QueueP_Handle handle);

/*!
 *  @brief  Function to Get the element at the front of the queue.
            This function removes an element from the front of a queue and returns it.
 *
 *  @param  handle  [in] A QueueP_Handle returned from QueueP_create
 *
 *  @return pointer to the element or
 *          pointer to queue itself incase of empty queue
 */
void * QueueP_get(QueueP_Handle handle);

/*!
 *  @brief  Function to Put an element at end of queue.
 *
 *  @param  handle  [in] A QueueP_Handle returned from QueueP_create
 *
 *  @param  elem [in] Pointer to new queue element
 *
 *  @return Status of the functions
 *    - QueueP_OK: Put the element at end of queue
 *    - QueueP_FAILURE: Failed to Put the element at end of queue
 */
int32_t QueueP_put(QueueP_Handle handle,
                   void *elem);

/*!
 *  @brief  Function to perform queue empty check
 *
 *  @param  handle  [in] A QueueP_Handle returned from QueueP_create
 *
 *  @return Current state of the Queue
 *    - QueueP_NOTEMPTY: queue is not empty
 *    - QueueP_EMPTY: queue is empty
 */
uint32_t QueueP_isEmpty(QueueP_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* ti_osal_QueueP__include */

/** @} */
