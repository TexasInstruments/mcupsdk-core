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

#ifndef HEAPP_H
#define HEAPP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <sys/types.h>
#include <kernel/dpl/SystemP.h>

/**
 * \defgroup KERNEL_DPL_HEAP APIs for Heap management
 * \ingroup KERNEL_DPL
 *
 * For more details and example usage, see \ref KERNEL_DPL_HEAP_PAGE
 *
 * @{
 */

/** \brief Minimum alignment for heap allocations, in units of bytes */
#define HeapP_BYTE_ALIGNMENT        (64u)

/**
 * \brief Structure used to pass information about the heap out of HeapP_getHeapStats().
 */
typedef struct HeapP_MemStats_
{
    size_t availableHeapSpaceInBytes;          /**< The total heap size, in bytes, currently available - this is the sum of all the free blocks, not the largest block that can be allocated. */
    size_t sizeOfLargestFreeBlockInBytes;      /**< The maximum size, in bytes, of all the free blocks within the heap at the time vPortGetHeapStats() is called. */
    size_t sizeOfSmallestFreeBlockInBytes;     /**< The minimum size, in bytes, of all the free blocks within the heap at the time vPortGetHeapStats() is called. */
    size_t numberOfFreeBlocks;                 /**< The number of free memory blocks within the heap at the time vPortGetHeapStats() is called. */
    size_t minimumEverFreeBytesRemaining;      /**< The minimum amount of total free memory, in bytes, (sum of all free blocks) there has been in the heap since the system booted. */
    size_t numberOfSuccessfulAllocations;      /**< The number of calls to HeapP_alloc() that have returned a valid memory block. */
    size_t numberOfSuccessfulFrees;            /**< The number of calls to HeapP_free() that has successfully freed a block of memory. */
} HeapP_MemStats;

/**
 * \brief The linked list structure is used to link free blocks in order of their memory address.
 */
typedef struct HEAP_BLOCK_LINK
{
    struct HEAP_BLOCK_LINK * pxNextFreeBlock; /* The next free block in the list. */
    size_t xBlockSize;                     /* The size of the free block. */
} HeapBlockLink_t;

/**
 * \brief Static heap instance structure.
 */
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

/**
 * \brief Opaque heap object used with the heap APIs
 */
 
#if defined (OS_FREERTOS) || defined (OS_FREERTOS_SMP) || defined (OS_FREERTOS_MPU)

#include <FreeRTOS.h>
#include <semphr.h>

typedef struct HeapP_Object_ {
    
    StaticHeap_t        heapHndl;
    StaticSemaphore_t   heapMutexObj;
    SemaphoreHandle_t   heapMutexHndl;

} HeapP_Object;

#else

typedef struct HeapP_Object_ {

    StaticHeap_t heapHndl;

} HeapP_Object;

#endif


/**
 * \brief Create a user defined heap
 *
 * The actual heap start address and size will be adjusted to satisfy
 * \ref HeapP_BYTE_ALIGNMENT.
 *
 * \param heapObj   [out] Initalized heap object to be used for subsequent API calls
 * \param heapAddr  [in] Base address of memory to be used as heap
 * \param heapSize  [in] Size of memory block that is to be used as heap
 * 
 * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
 */
int32_t HeapP_construct( HeapP_Object *heapObj, void *heapAddr, size_t heapSize );

/**
 * \brief Delete the user defined heap
 *
 * \param heapObj   [in] Heap object
 */
void HeapP_destruct( HeapP_Object *heapObj);

/**
 * \brief Alloc memory from user defined heap
 *
 * \param heapObj   [in] Heap object
 * \param allocSize [in] Size of memory to allocate
 *
 * \return pointer to allcoated memory
 * \return NULL memory could not be allocated since a free block of required size could not be found
 */
void* HeapP_alloc( HeapP_Object *heapObj, size_t allocSize );

/**
 * \brief Free memory from user defined heap
 *
 * \param heapObj   [in] Heap object
 * \param ptr       [in] Pointer to memory allocated using \ref HeapP_alloc
 *
 * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
 */
int32_t HeapP_free( HeapP_Object *heapObj, void * ptr );

/**
 * \brief Get free heap size, in bytes
 *
 * \param heapObj   [in] Heap object
 *
 * \return Free memory size in this heap, in bytes
 */
size_t HeapP_getFreeHeapSize( HeapP_Object *heapObj );

/**
 * \brief Get lowest ever free heap size, in bytes
 *
 * \param heapObj   [in] Heap object
 *
 * \return Lowest ever free heap size, in bytes
 */
size_t HeapP_getMinimumEverFreeHeapSize( HeapP_Object *heapObj );

/**
 * \brief Get detailed heap statistics
 *
 * \param heapObj    [in] Heap object
 * \param pHeapStats [out] Returned heap statistics
 * 
 * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
 */
int32_t HeapP_getHeapStats( HeapP_Object *heapObj, HeapP_MemStats * pHeapStats );

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* HEAPP_H */
