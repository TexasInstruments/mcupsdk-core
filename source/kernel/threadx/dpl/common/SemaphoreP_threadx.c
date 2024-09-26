/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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
#include "tx_api.h"

typedef struct SemaphoreP_Struct_ {
    TX_SEMAPHORE semObj;
    TX_MUTEX mutexObj;
    uint32_t isRecursiveMutex;
    uint32_t isMutex;
    uint32_t isCounting;
} SemaphoreP_Struct;

int32_t SemaphoreP_constructBinary(SemaphoreP_Object *obj, uint32_t initCount)
{
    SemaphoreP_Struct *pSemaphore = NULL;
    int32_t status = SystemP_FAILURE;
    UINT tx_ret;

    DebugP_assert(sizeof(SemaphoreP_Struct) <= sizeof(SemaphoreP_Object) );

    if(obj != NULL)
    {
        pSemaphore = (SemaphoreP_Struct *)obj;
        status = SystemP_SUCCESS;
    }

    if (SystemP_SUCCESS == status)
    {
        pSemaphore->isRecursiveMutex = 0;
        pSemaphore->isMutex = 0;
        pSemaphore->isCounting = 1;

        tx_ret = tx_semaphore_create(&pSemaphore->semObj, "Binary Sem (DPL)", initCount);
        if(tx_ret != TX_SUCCESS) {
            status = SystemP_FAILURE;
        }
        else
        {
            status = SystemP_SUCCESS;
        }
    }

    return status;
}

int32_t SemaphoreP_constructCounting(SemaphoreP_Object *obj, uint32_t initCount, uint32_t maxCount)
{
    SemaphoreP_Struct *pSemaphore = NULL;
    int32_t status = SystemP_FAILURE;
    UINT tx_ret;

    DebugP_assert(sizeof(SemaphoreP_Struct) <= sizeof(SemaphoreP_Object) );

    if(obj != NULL)
    {
        pSemaphore = (SemaphoreP_Struct *)obj;
        status = SystemP_SUCCESS;
    }

    if (SystemP_SUCCESS == status)
    {
        pSemaphore->isRecursiveMutex = 0;
        pSemaphore->isMutex = 0;
        pSemaphore->isCounting = 0;

        tx_ret = tx_semaphore_create(&pSemaphore->semObj, "Binary Sem (DPL)", initCount);
        if(tx_ret != TX_SUCCESS) {
            status = SystemP_FAILURE;
        }
        else
        {
            status = SystemP_SUCCESS;
        }
    }

    return status;
}

int32_t SemaphoreP_constructMutex(SemaphoreP_Object *obj)
{
    SemaphoreP_Struct *pSemaphore = NULL;
    int32_t status = SystemP_FAILURE;
    UINT tx_ret;

    DebugP_assert(sizeof(SemaphoreP_Struct) <= sizeof(SemaphoreP_Object) );

    if(obj != NULL)
    {
        pSemaphore = (SemaphoreP_Struct *)obj;
        status = SystemP_SUCCESS;
    }

    if (SystemP_SUCCESS == status)
    {
        pSemaphore->isRecursiveMutex = 1;
        pSemaphore->isMutex = 1;
        pSemaphore->isCounting = 0;

        tx_ret = tx_mutex_create(&pSemaphore->mutexObj, "Mutex (DPL)", TX_TRUE);
        if(tx_ret != TX_SUCCESS) {
            status = SystemP_FAILURE;
        }
        else
        {
            status = SystemP_SUCCESS;
        }
    }

    return status;
}

void SemaphoreP_destruct(SemaphoreP_Object *obj)
{
    SemaphoreP_Struct *pSemaphore = NULL;

    if(obj != NULL)
    {
        pSemaphore = (SemaphoreP_Struct *)obj;

        if(pSemaphore->isMutex == 1) {
            (void)_tx_mutex_delete(&pSemaphore->mutexObj);
        }
        else
        {
            (void)_tx_semaphore_delete(&pSemaphore->semObj);
        }

    }
}


int32_t SemaphoreP_pend(SemaphoreP_Object *obj, uint32_t timeout)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    int32_t status;
    UINT tx_ret;

    if(pSemaphore->isMutex == 1U)
    {
        tx_ret = tx_mutex_get(&pSemaphore->mutexObj, timeout);
    } else {
        tx_ret = tx_semaphore_get(&pSemaphore->semObj, timeout);
    }

    if(tx_ret != TX_SUCCESS)
    {
        if(tx_ret == TX_NO_INSTANCE)
        {
            status = SystemP_TIMEOUT;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_SUCCESS;
    }

    return status;
}

void SemaphoreP_post(SemaphoreP_Object *obj)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    UINT tx_ret;

    if(pSemaphore->isMutex == 1U)
    {
        tx_ret = tx_mutex_put(&pSemaphore->mutexObj);
    }
    else
    {
        if(pSemaphore->isCounting == 1U)
        {
            tx_ret = tx_semaphore_ceiling_put(&pSemaphore->semObj, 1U);
        }
        else
        {
            tx_ret = tx_semaphore_put(&pSemaphore->semObj);
        }
    }

    (void)tx_ret; // Can't return an error.
}

/*
 *  ======== SemaphoreP_getCount ========
 */
int32_t SemaphoreP_getCount(SemaphoreP_Object *obj)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    ULONG current_value;
    uint32_t count;

    if(pSemaphore->isMutex == 1U)
    {
        (void)tx_mutex_info_get(&pSemaphore->mutexObj, NULL, &current_value, NULL, NULL, NULL, NULL);
    }
    else
    {
        (void)tx_semaphore_info_get(&pSemaphore->semObj, NULL, &current_value, NULL, NULL, NULL);
    }

    count = current_value;

    return count;
}

