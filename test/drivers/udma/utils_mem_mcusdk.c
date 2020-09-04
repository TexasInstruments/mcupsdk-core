/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file utils_mem.c
 *
 *  \brief Memory allocator API.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HeapP.h>
#include <drivers/udma.h>
#include "utils_mem.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Memory pool handle */
static HeapP_Object gUtilsHeapObject[UTILS_MEM_HEAP_NUM];

static uint8_t gUtilsHeapMemMsmc[UTILS_MEM_HEAP_SIZE_MSMC] __attribute__((aligned(128)));
//static uint8_t gUtilsHeapMemDdr[UTILS_MEM_HEAP_SIZE_DDR] __attribute__(( aligned(128), section(".udma_buffer_ddr") ));
//static uint8_t gUtilsHeapMemInternal[UTILS_MEM_HEAP_SIZE_INTERNAL] __attribute__(( aligned(128), section(".udma_buffer_internal") ));
//static uint8_t gUtilsHeapMemOspi[UTILS_MEM_HEAP_SIZE_OSPI] __attribute__(( aligned(128), section(".udma_buffer_ospi") ));

static uint32_t gUtilsMemClearBuf = TRUE;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Utils_memInit(void)
{
    /* create memory pool heap */
    HeapP_construct(&gUtilsHeapObject[UTILS_MEM_HEAP_ID_MSMC], gUtilsHeapMemMsmc, sizeof (gUtilsHeapMemMsmc));
    //HeapP_construct(&gUtilsHeapObject[UTILS_MEM_HEAP_ID_DDR], gUtilsHeapMemDdr, sizeof (gUtilsHeapMemDdr));
    //HeapP_construct(&gUtilsHeapObject[UTILS_MEM_HEAP_ID_INTERNAL], gUtilsHeapMemInternal, sizeof (gUtilsHeapMemInternal));
    //HeapP_construct(&gUtilsHeapObject[UTILS_MEM_HEAP_ID_OSPI], gUtilsHeapMemOspi, sizeof (gUtilsHeapMemOspi));

    return (UDMA_SOK);
}

int32_t Utils_memDeInit(void)
{
    /* delete memory pool heap  */
    HeapP_destruct(&gUtilsHeapObject[UTILS_MEM_HEAP_ID_MSMC]);
    //HeapP_destruct(&gUtilsHeapObject[UTILS_MEM_HEAP_ID_DDR]);
    //HeapP_destruct(&gUtilsHeapObject[UTILS_MEM_HEAP_ID_INTERNAL]);
    //HeapP_destruct(&gUtilsHeapObject[UTILS_MEM_HEAP_ID_OSPI]);

    return (UDMA_SOK);
}

void *Utils_memAlloc(uint32_t heapId, uint32_t size, uint32_t align)
{
    void *addr;

    DebugP_assert(heapId < UTILS_MEM_HEAP_NUM);

    /* Heap alloc need some minimum allocation size */
    if(size < UDMA_CACHELINE_ALIGNMENT)
    {
        size = UDMA_CACHELINE_ALIGNMENT;
    }

    /* allocate memory  */
    addr = HeapP_alloc(&gUtilsHeapObject[heapId], size);
    if((addr != NULL) && (TRUE == gUtilsMemClearBuf))
    {
        memset(addr, 0U, size);
        /* Flush and invalidate the CPU write */
        CacheP_wbInv(addr, size, CacheP_TYPE_ALLD);
    }

    return (addr);
}

int32_t Utils_memFree(uint32_t heapId, void *addr, uint32_t size)
{
    DebugP_assert(heapId < UTILS_MEM_HEAP_NUM);

    /* Heap alloc need some minimum allocation size */
    if(size < UDMA_CACHELINE_ALIGNMENT)
    {
        size = UDMA_CACHELINE_ALIGNMENT;
    }
    /* free previously allocated memory  */
    HeapP_free(&gUtilsHeapObject[heapId], addr);
    return (UDMA_SOK);
}

int32_t Utils_memClearOnAlloc(Bool enable)
{
    gUtilsMemClearBuf = enable;

    return (UDMA_SOK);
}

void Utils_memGetHeapStat(Utils_MemHeapStatus *heapStat)
{
    uint32_t    idx;

    /* NULL pointer check */
    DebugP_assert(NULL != heapStat);

    heapStat->freeSysHeapSize = 0U;
    for(idx = 0U; idx < UTILS_MEM_HEAP_NUM; idx++)
    {
        heapStat->freeBufHeapSize[idx] = Utils_memGetBufferHeapFreeSpace(idx);
    }

    return;
}

int32_t Utils_memCheckHeapStat(const Utils_MemHeapStatus *heapStat)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            idx;
    Utils_MemHeapStatus curStat;

    /* NULL pointer check */
    DebugP_assert(NULL != heapStat);

    Utils_memGetHeapStat(&curStat);

    if(heapStat->freeSysHeapSize != curStat.freeSysHeapSize)
    {
        DebugP_logError(
                  "Warning: Memory leak (%d bytes) in System Heap!!\r\n",
                  (heapStat->freeSysHeapSize - curStat.freeSysHeapSize));
        retVal = UDMA_EFAIL;
    }
    for(idx = 0U; idx < UTILS_MEM_HEAP_NUM; idx++)
    {
        if(heapStat->freeBufHeapSize[idx] != curStat.freeBufHeapSize[idx])
        {
            DebugP_logError(
                      "Warning: Memory leak (%d bytes) in Buffer Heap!!\r\n",
                      (heapStat->freeBufHeapSize[idx] - curStat.freeBufHeapSize[idx]));
            retVal = UDMA_EFAIL;
        }
    }

    return (retVal);
}

uint32_t Utils_memGetBufferHeapFreeSpace(uint32_t heapId)
{
    uint32_t    totalFreeSize;

    DebugP_assert(heapId < UTILS_MEM_HEAP_NUM);
    totalFreeSize = HeapP_getFreeHeapSize(&gUtilsHeapObject[heapId]);

    return (totalFreeSize);
}
