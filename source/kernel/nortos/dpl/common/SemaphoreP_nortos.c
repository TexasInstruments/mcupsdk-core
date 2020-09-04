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

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>

/**
 * \brief Semaphore type
 */
typedef enum SemaphoreP_Type_ {
    SemaphoreP_TYPE_BINARY, /**< Binary semaphore */
    SemaphoreP_TYPE_COUNTING, /**< Counting semaphore */
    SemaphoreP_TYPE_MUTEX /**< Mutual exclusion semaphore */
} SemaphoreP_Type;

/**
 * \brief Parameters passed during \ref SemaphoreP_construct
 */
typedef struct SemaphoreP_Params_ {

    uint32_t type; /**< see \ref SemaphoreP_Type */
    uint32_t initCount; /**< initial value of the semaphore. \n
        Only valid for \ref SemaphoreP_TYPE_COUNTING and \ref SemaphoreP_TYPE_BINARY. \n
        MUST be 0 or 1 for \ref SemaphoreP_TYPE_BINARY \n
        Ignored for \ref SemaphoreP_TYPE_MUTEX */
    uint32_t maxCount; /**< max value the semaphore can take. \n
        Only valid for \ref SemaphoreP_TYPE_COUNTING. \n
        MUST be >= SemaphoreP_Params.initCount \n
        Ignored for \ref SemaphoreP_TYPE_MUTEX and \ref SemaphoreP_TYPE_BINARY
        */

} SemaphoreP_Params;

typedef struct SemaphoreP_Struct_ {
    uint32_t type;
    uint32_t maxCount;
    volatile uint32_t count;
    volatile uint32_t nestCount;
} SemaphoreP_Struct;

void SemaphoreP_Params_init(SemaphoreP_Params *params)
{
    params->type = SemaphoreP_TYPE_BINARY;
    params->initCount = 0;
    params->maxCount = 1;
}

int32_t SemaphoreP_construct(SemaphoreP_Object *obj, SemaphoreP_Params *params)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;

    DebugP_assert(sizeof(SemaphoreP_Object) >= sizeof(SemaphoreP_Struct));

    pSemaphore->type = params->type;
    pSemaphore->count = params->initCount;
    pSemaphore->maxCount = params->maxCount;
    pSemaphore->nestCount = 0;

    return SystemP_SUCCESS;
}

int32_t SemaphoreP_constructBinary(SemaphoreP_Object *obj, uint32_t initCount)
{
    SemaphoreP_Params params;

    SemaphoreP_Params_init(&params);
    params.type = SemaphoreP_TYPE_BINARY;
    params.initCount = (initCount & 0x1);
    params.maxCount = 1;

    return (SemaphoreP_construct(obj, &params));
}

int32_t SemaphoreP_constructCounting(SemaphoreP_Object *obj, uint32_t initCount, uint32_t maxCount)
{
    SemaphoreP_Params params;

    SemaphoreP_Params_init(&params);
    params.type = SemaphoreP_TYPE_COUNTING;
    if(maxCount == 0)
        maxCount = 1;
    if(initCount > maxCount)
        initCount = maxCount;
    params.initCount = initCount;
    params.maxCount = maxCount;

    return (SemaphoreP_construct(obj, &params));
}

int32_t SemaphoreP_constructMutex(SemaphoreP_Object *obj)
{
    SemaphoreP_Params params;

    SemaphoreP_Params_init(&params);
    params.type = SemaphoreP_TYPE_MUTEX;
    params.initCount = 1;
    params.maxCount = 1;

    return (SemaphoreP_construct(obj, &params));
}

void SemaphoreP_destruct(SemaphoreP_Object *obj)
{
    /* nothing to do */
}


int32_t SemaphoreP_pend(SemaphoreP_Object *obj, uint32_t timeout)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    ClockP_Params      clockParams;
    ClockP_Object      clockObj;
    uintptr_t          key;
    int32_t            status = SystemP_SUCCESS;

    /*
     * Always add Clock (but don't start) so that ClockP_isActive() below
     * is valid.  It's OK to add a Clock even when timeout is 0 or forever
     * (but it is not OK to start it).
     */
    ClockP_Params_init(&clockParams);
    clockParams.timeout = timeout;
    ClockP_construct(&clockObj, &clockParams);

    if ((timeout != 0) && (timeout != SystemP_WAIT_FOREVER)) {
        ClockP_start(&clockObj);
    }

    key = HwiP_disable();

    while ((pSemaphore->count == 0) && (pSemaphore->nestCount == 0) &&
           ((timeout == SystemP_WAIT_FOREVER) ||
            ClockP_isActive(&clockObj))) {

        HwiP_restore(key);

        key = HwiP_disable();
    }

    if (pSemaphore->count > 0) {
        (pSemaphore->count)--;
        if(pSemaphore->type==SemaphoreP_TYPE_MUTEX)
        {
            pSemaphore->nestCount++;
        }
        status = SystemP_SUCCESS;
    }
    else {
        if(pSemaphore->type==SemaphoreP_TYPE_MUTEX)
        {
            pSemaphore->nestCount++;
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_TIMEOUT;
        }
    }

    HwiP_restore(key);

    ClockP_destruct(&clockObj);

    return (status);
}

void SemaphoreP_post(SemaphoreP_Object *obj)
{
    SemaphoreP_Struct *pSemaphore = (SemaphoreP_Struct *)obj;
    uintptr_t       key;

    key = HwiP_disable();

    if (pSemaphore->count < pSemaphore->maxCount) {

        if(pSemaphore->type==SemaphoreP_TYPE_MUTEX)
        {
            if(pSemaphore->nestCount>0)
                pSemaphore->nestCount--;
        }
        if(pSemaphore->nestCount==0)
        {
            (pSemaphore->count)++;
        }
    }

    HwiP_restore(key);
}

