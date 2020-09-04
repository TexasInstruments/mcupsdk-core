/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
#include "udma_test.h"

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
static HeapP_Handle gUtilsHeapMemHandle[UTILS_MEM_HEAP_NUM] = {NULL, NULL};

static uint8_t gUtilsHeapMemMsmc[UTILS_MEM_HEAP_SIZE_MSMC] __attribute__(( aligned(128), section(".udma_buffer_msmc") ));
static uint8_t gUtilsHeapMemDdr[UTILS_MEM_HEAP_SIZE_DDR] __attribute__(( aligned(128), section(".udma_buffer_ddr") ));
static uint8_t gUtilsHeapMemInternal[UTILS_MEM_HEAP_SIZE_INTERNAL] __attribute__(( aligned(128), section(".udma_buffer_internal") ));
static uint8_t gUtilsHeapMemOspi[UTILS_MEM_HEAP_SIZE_OSPI] __attribute__(( aligned(128), section(".udma_buffer_ospi") ));

static uint32_t gUtilsMemClearBuf = FALSE;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Utils_memInit(void)
{
    HeapP_Params heapMemPrm;

    /* create memory pool heap - Msmc */
    HeapP_Params_init(&heapMemPrm);
    heapMemPrm.buf  = gUtilsHeapMemMsmc;
    heapMemPrm.size = sizeof (gUtilsHeapMemMsmc);
    gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_MSMC] = HeapP_create(&heapMemPrm);
    GT_assert(UdmaUtTrace, gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_MSMC] != NULL);

    /* create memory pool heap - DDR */
    HeapP_Params_init(&heapMemPrm);
    heapMemPrm.buf  = gUtilsHeapMemDdr;
    heapMemPrm.size = sizeof (gUtilsHeapMemDdr);
    gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_DDR] = HeapP_create(&heapMemPrm);
    GT_assert(UdmaUtTrace, gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_DDR] != NULL);

    /* create memory pool heap - internal */
    HeapP_Params_init(&heapMemPrm);
    heapMemPrm.buf  = gUtilsHeapMemInternal;
    heapMemPrm.size = sizeof (gUtilsHeapMemInternal);
    gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_INTERNAL] = HeapP_create(&heapMemPrm);
    GT_assert(UdmaUtTrace, gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_INTERNAL] != NULL);

    /* create memory pool heap - OSPI */
    HeapP_Params_init(&heapMemPrm);
    heapMemPrm.buf  = gUtilsHeapMemOspi;
    heapMemPrm.size = sizeof (gUtilsHeapMemOspi);
    gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_OSPI] = HeapP_create(&heapMemPrm);
    GT_assert(UdmaUtTrace, gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_OSPI] != NULL);

    gUtilsMemClearBuf = TRUE;

    return (UDMA_SOK);
}

int32_t Utils_memDeInit(void)
{
    /* delete memory pool heap  */
    HeapP_delete(&gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_MSMC]);
    gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_MSMC] = NULL;
    HeapP_delete(&gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_DDR]);
    gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_DDR] = NULL;
    HeapP_delete(&gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_INTERNAL]);
    gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_INTERNAL] = NULL;
    HeapP_delete(&gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_OSPI]);
    gUtilsHeapMemHandle[UTILS_MEM_HEAP_ID_OSPI] = NULL;
    return (UDMA_SOK);
}

void *Utils_memAlloc(uint32_t heapId, uint32_t size, uint32_t align)
{
    void *addr;

    GT_assert(UdmaUtTrace, heapId < UTILS_MEM_HEAP_NUM);
    GT_assert(UdmaUtTrace, gUtilsHeapMemHandle[heapId] != NULL);

    /* Heap alloc need some minimum allocation size */
    if(size < UDMA_CACHELINE_ALIGNMENT)
    {
        size = UDMA_CACHELINE_ALIGNMENT;
    }

    /* allocate memory  */
        addr = HeapP_alloc(gUtilsHeapMemHandle[heapId], size);
    if((addr != NULL) && (TRUE == gUtilsMemClearBuf))
    {
        memset(addr, 0U, size);
        /* Flush and invalidate the CPU write */
        Udma_appUtilsCacheWbInv(addr, size);
    }

    return (addr);
}

int32_t Utils_memFree(uint32_t heapId, void *addr, uint32_t size)
{
    GT_assert(UdmaUtTrace, heapId < UTILS_MEM_HEAP_NUM);
    GT_assert(UdmaUtTrace, gUtilsHeapMemHandle[heapId] != NULL);

    /* Heap alloc need some minimum allocation size */
    if(size < UDMA_CACHELINE_ALIGNMENT)
    {
        size = UDMA_CACHELINE_ALIGNMENT;
    }
    /* free previously allocated memory  */
    HeapP_free(gUtilsHeapMemHandle[heapId], addr, size);
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
    GT_assert(UdmaUtTrace, NULL != heapStat);

    heapStat->freeSysHeapSize = Utils_memGetSystemHeapFreeSpace();
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
    GT_assert(UdmaUtTrace, NULL != heapStat);

    Utils_memGetHeapStat(&curStat);

    if(heapStat->freeSysHeapSize != curStat.freeSysHeapSize)
    {
        GT_1trace(UdmaUtTrace, GT_CRIT,
                  "Warning: Memory leak (%d bytes) in System Heap!!\r\n",
                  (heapStat->freeSysHeapSize - curStat.freeSysHeapSize));
        retVal = UDMA_EFAIL;
    }
    for(idx = 0U; idx < UTILS_MEM_HEAP_NUM; idx++)
    {
        if(heapStat->freeBufHeapSize[idx] != curStat.freeBufHeapSize[idx])
        {
            GT_1trace(UdmaUtTrace, GT_CRIT,
                      "Warning: Memory leak (%d bytes) in Buffer Heap!!\r\n",
                      (heapStat->freeBufHeapSize[idx] - curStat.freeBufHeapSize[idx]));
            retVal = UDMA_EFAIL;
        }
    }

    return (retVal);
}

uint32_t Utils_memGetSystemHeapFreeSpace(void)
{
    MemoryP_Stats stats;

    MemoryP_getStats(&stats);

    return ((uint32_t) (stats.totalFreeSize));
}

uint32_t Utils_memGetBufferHeapFreeSpace(uint32_t heapId)
{
    uint32_t        totalFreeSize = 0U;
    HeapP_MemStats  stats;

    GT_assert(UdmaUtTrace, heapId < UTILS_MEM_HEAP_NUM);
    if(NULL != gUtilsHeapMemHandle[heapId])
    {
        HeapP_getHeapStats(gUtilsHeapMemHandle[heapId], &stats);
        totalFreeSize = (uint32_t) stats.totalFreeSize;
    }
    return (totalFreeSize);
}
