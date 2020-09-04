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


#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/common/HeapP_internal.h>
#include <SafeRTOS.h>
#include <task.h>

void   HeapP_construct( HeapP_Object *heap, void *heapAddr, size_t heapSize )
{
    DebugP_assert( sizeof(StaticHeap_t) < sizeof(HeapP_Object) );

    vHeapCreateStatic((StaticHeap_t*)heap, heapAddr, heapSize);
}

void   HeapP_destruct(HeapP_Object *heap)
{
    vTaskSuspendScheduler();
    vHeapDelete((StaticHeap_t*)heap);
    xTaskResumeScheduler();
}

void  *HeapP_alloc( HeapP_Object *heap, size_t allocSize )
{
    void *ptr;

    vTaskSuspendScheduler();
    ptr = pvHeapMalloc((StaticHeap_t*)heap, allocSize);
    xTaskResumeScheduler();

    return ptr;
}

void   HeapP_free( HeapP_Object *heap, void * ptr )
{
    vTaskSuspendScheduler();
    vHeapFree((StaticHeap_t*)heap, ptr);
    xTaskResumeScheduler();
}

size_t HeapP_getFreeHeapSize( HeapP_Object *heap )
{
    return xHeapGetFreeHeapSize((StaticHeap_t*)heap);
}

size_t HeapP_getMinimumEverFreeHeapSize( HeapP_Object *heap )
{
    return xHeapGetMinimumEverFreeHeapSize((StaticHeap_t*)heap);
}

void   HeapP_getHeapStats( HeapP_Object *heap, HeapP_MemStats * pHeapStats )
{
    vTaskSuspendScheduler();
    vHeapGetHeapStats((StaticHeap_t*)heap, pHeapStats);
    xTaskResumeScheduler();
}

