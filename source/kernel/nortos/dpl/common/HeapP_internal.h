/*
 * FreeRTOS Kernel V10.4.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */
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

#ifndef HEAPP_INTERNAL_H
#define HEAPP_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/HeapP.h>

/* Define the linked list structure.  This is used to link free blocks in order
 * of their memory address. */
typedef struct HEAP_BLOCK_LINK
{
    struct HEAP_BLOCK_LINK * pxNextFreeBlock; /* The next free block in the list. */
    size_t xBlockSize;                     /* The size of the free block. */
} HeapBlockLink_t;

/* Static heap instance structure */
typedef struct StaticHeap_ {

    /* Create a couple of list links to mark the start and end of the list. */
    HeapBlockLink_t xStart;
    HeapBlockLink_t *pxEnd;

    /* Keeps track of the number of calls to allocate and free memory as well as the
    * number of free bytes remaining, but says nothing about fragmentation. */
    size_t xFreeBytesRemaining;
    size_t xMinimumEverFreeBytesRemaining;
    size_t xNumberOfSuccessfulAllocations;
    size_t xNumberOfSuccessfulFrees;

    /* Gets set to the top bit of an size_t type.  When this bit in the xBlockSize
    * member of an HeapBlockLink_t structure is set then the block belongs to the
    * application.  When the bit is free the block is still part of the free heap
    * space. */
    size_t xBlockAllocatedBit;

    /* base address and size of heap */
    void *pvHeap;
    size_t xTotalHeapSize;

} StaticHeap_t;

/* Creates a heap from user provided heap address and size */
void vHeapCreateStatic( StaticHeap_t *heap, void *pvHeap, size_t xTotalHeapSize );

/* Delete a heap */
void vHeapDelete(StaticHeap_t *heap);

/* Allocate from user specified heap */
void * pvHeapMalloc( StaticHeap_t *heap, size_t xWantedSize );

/* Free from user specified heap */
void vHeapFree( StaticHeap_t *heap, void * pv );

/* Get free heap size from user specified heap */
size_t xHeapGetFreeHeapSize( StaticHeap_t *heap );

/* Get lowest ever heap size from user specified heap */
size_t xHeapGetMinimumEverFreeHeapSize( StaticHeap_t *heap );

/* Get heap stats from user specified heap */
void vHeapGetHeapStats( StaticHeap_t *heap, HeapP_MemStats * pxHeapStats );

#ifdef __cplusplus
}
#endif

#endif /* HEAPP_INTERNAL_H */
