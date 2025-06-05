/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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


#include <stdlib.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/common/HeapP_internal.h>
#include <FreeRTOS.h>
#include <task.h>

int32_t HeapP_construct( HeapP_Object *heapObj, void *heapAddr, size_t heapSize )
{
    int32_t status = SystemP_FAILURE;

    DebugP_assert(heapObj != NULL);
    DebugP_assert(heapAddr != NULL);
    DebugP_assert(heapSize > 0U);
    
    heapObj->heapMutexHndl = xSemaphoreCreateRecursiveMutexStatic(&heapObj->heapMutexObj);

    if(heapObj->heapMutexHndl != NULL)
    {
        vQueueAddToRegistry(heapObj->heapMutexHndl, "HeapP Mutex");

        vHeapCreateStatic(&heapObj->heapHndl, heapAddr, heapSize);

        status = SystemP_SUCCESS;
    }
    
    return status;
}

void HeapP_destruct(HeapP_Object *heapObj)
{
    DebugP_assert(heapObj != NULL);

    vTaskSuspendAll();

    vHeapDelete(&heapObj->heapHndl);

    vQueueUnregisterQueue(heapObj->heapMutexHndl);

    vSemaphoreDelete(heapObj->heapMutexHndl);

    (void)xTaskResumeAll();
}

void* HeapP_alloc( HeapP_Object *heapObj, size_t allocSize )
{
    void *ptr = NULL;

    DebugP_assert(heapObj != NULL);
    DebugP_assert(allocSize > 0U);

    if(xSemaphoreTakeRecursive(heapObj->heapMutexHndl, portMAX_DELAY) == pdTRUE) 
    {
        ptr = pvHeapMalloc(&heapObj->heapHndl, allocSize);

        xSemaphoreGiveRecursive(heapObj->heapMutexHndl);
    };

    return ptr;
}

int32_t HeapP_free( HeapP_Object *heapObj, void * ptr )
{
    int32_t status = SystemP_FAILURE;

    DebugP_assert(heapObj != NULL);
    DebugP_assert(ptr != NULL);

    if(xSemaphoreTakeRecursive(heapObj->heapMutexHndl, portMAX_DELAY) == pdTRUE) 
    {
        vHeapFree(&heapObj->heapHndl, ptr);

        xSemaphoreGiveRecursive(heapObj->heapMutexHndl);

        status = SystemP_SUCCESS;
    };

    return status;
}

size_t HeapP_getFreeHeapSize( HeapP_Object *heapObj )
{
    DebugP_assert(heapObj != NULL);

    return xHeapGetFreeHeapSize(&heapObj->heapHndl);
}

size_t HeapP_getMinimumEverFreeHeapSize( HeapP_Object *heapObj )
{
    DebugP_assert(heapObj != NULL);

    return xHeapGetMinimumEverFreeHeapSize(&heapObj->heapHndl);
}

int32_t HeapP_getHeapStats( HeapP_Object *heapObj, HeapP_MemStats * pHeapStats )
{
    int32_t status = SystemP_FAILURE;

    DebugP_assert(heapObj != NULL);
    DebugP_assert(pHeapStats != NULL);

    memset(pHeapStats, 0, sizeof(HeapP_MemStats));

    if(xSemaphoreTakeRecursive(heapObj->heapMutexHndl, portMAX_DELAY) == pdTRUE) 
    {
        vHeapGetHeapStats(&heapObj->heapHndl, pHeapStats);

        xSemaphoreGiveRecursive(heapObj->heapMutexHndl);
        
        status = SystemP_SUCCESS;
    };

    return status;
}

