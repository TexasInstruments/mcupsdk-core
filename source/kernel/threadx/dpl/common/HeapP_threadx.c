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


#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/common/HeapP_internal.h>
#include "tx_api.h"

void   HeapP_construct( HeapP_Object *heap, void *heapAddr, size_t heapSize )
{
    TX_BYTE_POOL *p_pool;

    DebugP_assert( sizeof(StaticHeap_t) < sizeof(HeapP_Object) );

    p_pool = (TX_BYTE_POOL *)heap;

    (void)tx_byte_pool_create(p_pool, "dpl pool", heapAddr, heapSize);

}

void   HeapP_destruct(HeapP_Object *heap)
{
    TX_BYTE_POOL *p_pool;

    p_pool = (TX_BYTE_POOL *)heap;

    _tx_byte_pool_delete(p_pool);
}

void  *HeapP_alloc( HeapP_Object *heap, size_t allocSize )
{
    void *ptr = NULL;
    TX_BYTE_POOL *p_pool;

    p_pool = (TX_BYTE_POOL *)heap;

    (void)tx_byte_allocate(p_pool, &ptr, allocSize, TX_NO_WAIT);

    return ptr;
}

void   HeapP_free( HeapP_Object *heap, void * ptr )
{
    (void)tx_byte_release(ptr);
}

size_t HeapP_getFreeHeapSize( HeapP_Object *heap )
{
    size_t rem_size = 0u;
    TX_BYTE_POOL *p_pool;
    ULONG available_bytes = 0u;

    p_pool = (TX_BYTE_POOL *)heap;

    tx_byte_pool_info_get(p_pool, NULL, &available_bytes, NULL, NULL, NULL, NULL);

    rem_size = available_bytes;

    return rem_size;
}

size_t HeapP_getMinimumEverFreeHeapSize( HeapP_Object *heap )
{
    return 0;
}

void   HeapP_getHeapStats( HeapP_Object *heap, HeapP_MemStats * pHeapStats )
{
    (void)heap;
    (void)pHeapStats;
}

