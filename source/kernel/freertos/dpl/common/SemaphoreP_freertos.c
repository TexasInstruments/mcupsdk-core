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
#include <string.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <FreeRTOS.h>
#include <semphr.h>

typedef struct SemaphoreP_Struct_ {
    StaticSemaphore_t semObj;
    SemaphoreHandle_t semHndl;
    uint32_t isRecursiveMutex;
} SemaphoreP_Struct;

int32_t SemaphoreP_constructBinary(SemaphoreP_Object *obj, uint32_t initCount)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    int32_t status;

    DebugP_assert(sizeof(SemaphoreP_Struct) <= sizeof(SemaphoreP_Object) );

    pSemaphore->isRecursiveMutex = 0;
    pSemaphore->semHndl = xSemaphoreCreateBinaryStatic(&pSemaphore->semObj);
    if( pSemaphore->semHndl == NULL )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        vQueueAddToRegistry(pSemaphore->semHndl, "Binary Sem (DPL)");

        if(initCount == 1)
        {
            /* post a semaphore to increment initial count to 1 */
            xSemaphoreGive(pSemaphore->semHndl);
        }
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t SemaphoreP_constructCounting(SemaphoreP_Object *obj, uint32_t initCount, uint32_t maxCount)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    int32_t status;

    DebugP_assert(sizeof(SemaphoreP_Struct) <= sizeof(SemaphoreP_Object) );

    pSemaphore->isRecursiveMutex = 0;
    pSemaphore->semHndl = xSemaphoreCreateCountingStatic(
                                maxCount,
                                initCount,
                                &pSemaphore->semObj);
    if( pSemaphore->semHndl == NULL )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        vQueueAddToRegistry(pSemaphore->semHndl, "Counting Sem (DPL)");
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t SemaphoreP_constructMutex(SemaphoreP_Object *obj)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    int32_t status;

    DebugP_assert(sizeof(SemaphoreP_Struct) <= sizeof(SemaphoreP_Object) );

    pSemaphore->isRecursiveMutex = 1;
    pSemaphore->semHndl = xSemaphoreCreateRecursiveMutexStatic(&pSemaphore->semObj);
    if( pSemaphore->semHndl == NULL )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        vQueueAddToRegistry(pSemaphore->semHndl, "Mutex (DPL)");
        status = SystemP_SUCCESS;
    }

    return status;
}

void SemaphoreP_destruct(SemaphoreP_Object *obj)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;

    vSemaphoreDelete(pSemaphore->semHndl);
}


int32_t SemaphoreP_pend(SemaphoreP_Object *obj, uint32_t timeout)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    uint32_t isSemTaken = 0;
    int32_t status;

    if(pSemaphore->isRecursiveMutex)
    {
        if( ! HwiP_inISR() )
        {
            isSemTaken = xSemaphoreTakeRecursive(pSemaphore->semHndl, timeout);
        }
        else
        {
            /* NOT allowed to use mutex in ISR */
            DebugP_assertNoLog(0);
        }
    }
    else
    {
        if( HwiP_inISR() )
        {
            BaseType_t xHigherPriorityTaskWoken = 0;

            /* timeout is ignored when in ISR mode */
            isSemTaken = xSemaphoreTakeFromISR(pSemaphore->semHndl, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        else
        {
            isSemTaken = xSemaphoreTake(pSemaphore->semHndl, timeout);
        }
    }
    if(isSemTaken)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_TIMEOUT;
    }

    return status;
}

void SemaphoreP_post(SemaphoreP_Object *obj)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;

    if(pSemaphore->isRecursiveMutex)
    {
        if( ! HwiP_inISR() )
        {
            xSemaphoreGiveRecursive(pSemaphore->semHndl);
        }
        else
        {
            /* NOT allowed to use mutex in ISR */
            DebugP_assertNoLog(0);
        }
    }
    else
    {
        if( HwiP_inISR() )
        {
            BaseType_t xHigherPriorityTaskWoken = 0;

            xSemaphoreGiveFromISR(pSemaphore->semHndl, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        else
        {
            xSemaphoreGive(pSemaphore->semHndl);
        }
    }
}

/* IMPORTANT:
   This structure is copied from source\kernel\freertos\FreeRTOS-Kernel\queue.c
   and is used for ROV based views in CCS

   Due to presence of union in xQUEUE, ROV does not fetch the data correctly.
   Below is identical structure except that there is no union and
   instead the element having maximum size in the union is put as a field in below structure.
   Rest of the fields are same as xQUEUE.

   This is not used anywhere in the run-time application and has effect only when
   ROV is used
 */

typedef struct SemaphoreData
{
    TaskHandle_t xMutexHolder;        /*< The handle of the task that holds the mutex. */
    UBaseType_t uxRecursiveCallCount; /*< Maintains a count of the number of times a recursive mutex has been recursively 'taken' when the structure is used as a mutex. */
} SemaphoreData_t;

typedef struct /* The old naming convention is used to prevent breaking kernel aware debuggers. */
{
    int8_t * pcHead;           /*< Points to the beginning of the queue storage area. */
    int8_t * pcWriteTo;        /*< Points to the free next place in the storage area. */

    SemaphoreData_t xSemaphore; /*< Data required exclusively when this structure is used as a semaphore. */

    List_t xTasksWaitingToSend;             /*< List of tasks that are blocked waiting to post onto this queue.  Stored in priority order. */
    List_t xTasksWaitingToReceive;          /*< List of tasks that are blocked waiting to read from this queue.  Stored in priority order. */

    volatile UBaseType_t uxMessagesWaiting; /*< The number of items currently in the queue. */
    UBaseType_t uxLength;                   /*< The length of the queue defined as the number of items it will hold, not the number of bytes. */
    UBaseType_t uxItemSize;                 /*< The size of each items that the queue will hold. */

    volatile int8_t cRxLock;                /*< Stores the number of items received from the queue (removed from the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */
    volatile int8_t cTxLock;                /*< Stores the number of items transmitted to the queue (added to the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */

    #if ( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
        uint8_t ucStaticallyAllocated; /*< Set to pdTRUE if the memory used by the queue was statically allocated to ensure no attempt is made to free the memory. */
    #endif

    #if ( configUSE_QUEUE_SETS == 1 )
        struct QueueDefinition * pxQueueSetContainer;
    #endif

    #if ( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t uxQueueNumber;
        uint8_t ucQueueType;
    #endif
} xQUEUE_ROV;

/* This is defined so that the symbol is retained in the final executable */
xQUEUE_ROV gRovDummyQueue __attribute__((used));



