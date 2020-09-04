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
#include <SafeRTOS.h>
#include <queue.h>
#include <semaphore.h>
#include <mutex.h>

typedef struct SemaphoreP_Struct_ {
    uint64_t         semObj[(safertosapiQUEUE_OVERHEAD_BYTES/sizeof(uint64_t) + 1)];
    xSemaphoreHandle semHndl;
    uint32_t         isRecursiveMutex;
} SemaphoreP_Struct;

int32_t SemaphoreP_constructBinary(SemaphoreP_Object *obj, uint32_t initCount)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    int32_t status;
    portBaseType xResult;

    DebugP_assert(sizeof(SemaphoreP_Struct) <= sizeof(SemaphoreP_Object) );

    memset(pSemaphore, 0U, sizeof(SemaphoreP_Struct));
    pSemaphore->isRecursiveMutex = 0;
    xResult = xSemaphoreCreateBinary((portInt8Type *)&pSemaphore->semObj[0], &pSemaphore->semHndl);
    if((xResult != pdPASS) || (pSemaphore->semHndl == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        /* Not supported in SafeRTOS */
        //vQueueAddToRegistry(pSemaphore->semHndl, "Binary Sem (DPL)");

        if(initCount == 0)
        {
            uint32_t            isSemTaken;
            DebugP_assert(xPortInIsrContext( ) == pdFALSE);
            /* SafeRTOS on BinarySemaphore create initializes semaphore with count of 1.
             * So we need to take semaphore to make count 0, if we are creating a binary semaphore with init count of 0.
             */
            isSemTaken = xSemaphoreTake( pSemaphore->semHndl, safertosapiMAX_DELAY);
            DebugP_assert(isSemTaken == pdTRUE);
        }
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t SemaphoreP_constructCounting(SemaphoreP_Object *obj, uint32_t initCount, uint32_t maxCount)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    int32_t status;
    portBaseType xResult;

    DebugP_assert(sizeof(SemaphoreP_Struct) <= sizeof(SemaphoreP_Object) );

    memset(pSemaphore, 0U, sizeof(SemaphoreP_Struct));
    pSemaphore->isRecursiveMutex = 0;
    xResult = xSemaphoreCreateCounting(
                  maxCount,
                  initCount,
                  (portInt8Type *)&pSemaphore->semObj[0],
                  &pSemaphore->semHndl);
    if((xResult != pdPASS) || (pSemaphore->semHndl == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        /* Not supported in SafeRTOS */
        //vQueueAddToRegistry(pSemaphore->semHndl, "Counting Sem (DPL)");
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t SemaphoreP_constructMutex(SemaphoreP_Object *obj)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    int32_t status;
    portBaseType xResult;

    DebugP_assert(sizeof(SemaphoreP_Struct) <= sizeof(SemaphoreP_Object) );

    memset(pSemaphore, 0U, sizeof(SemaphoreP_Struct));
    pSemaphore->isRecursiveMutex = 1;
    xResult = xMutexCreate((portInt8Type *)&pSemaphore->semObj[0], &pSemaphore->semHndl);
    if((xResult != pdPASS) || (pSemaphore->semHndl == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        /* Not supported in SafeRTOS */
        //vQueueAddToRegistry(pSemaphore->semHndl, "Mutex (DPL)");
        status = SystemP_SUCCESS;
    }

    return status;
}

void SemaphoreP_destruct(SemaphoreP_Object *obj)
{
    /* Not implemented in SafeRTOS */
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
            isSemTaken = xMutexTake(pSemaphore->semHndl, timeout);
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
            /* timeout is ignored when in ISR mode */
            isSemTaken = xSemaphoreTakeFromISR(pSemaphore->semHndl);
            safertosapiYIELD_FROM_ISR();
        }
        else
        {
            isSemTaken = xSemaphoreTake(pSemaphore->semHndl, timeout);
        }
    }
    if(pdPASS == isSemTaken)
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
            xMutexGive(pSemaphore->semHndl);
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
            xSemaphoreGiveFromISR(pSemaphore->semHndl);
            safertosapiYIELD_FROM_ISR();
        }
        else
        {
            xSemaphoreGive(pSemaphore->semHndl);
        }
    }
}
