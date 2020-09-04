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
#include <stdlib.h>
#include "udma_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Macro to align x to y */
#define align(x,y)   ((x + y - 1) & (~(y - 1)))

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

static uint8_t gUtilsHeapMemMsmc[UTILS_MEM_HEAP_SIZE_MSMC] __attribute__(( aligned(128), section(".udma_buffer_msmc") ));
static uint8_t gUtilsHeapMemDdr[UTILS_MEM_HEAP_SIZE_DDR] __attribute__(( aligned(128), section(".udma_buffer_ddr") ));
static uint8_t gUtilsHeapMemInternal[UTILS_MEM_HEAP_SIZE_INTERNAL] __attribute__(( aligned(128), section(".udma_buffer_internal") ));
static uint8_t gUtilsHeapMemOspi[UTILS_MEM_HEAP_SIZE_OSPI] __attribute__(( aligned(128), section(".udma_buffer_ospi") ));

uint8_t *fw_mem_start[UTILS_MEM_HEAP_NUM];
uint8_t *fw_mem_end[UTILS_MEM_HEAP_NUM];
uint8_t *fw_mem_alloc_ptr[UTILS_MEM_HEAP_NUM];
uint8_t  fw_mem_wraparound[UTILS_MEM_HEAP_NUM];

static uint32_t gUtilsMemClearBuf = FALSE;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Utils_memInit(void)
{
    fw_mem_start[0]     = (uint8_t*)  gUtilsHeapMemMsmc;
    fw_mem_end[0]       = (uint8_t*)(&gUtilsHeapMemMsmc[UTILS_MEM_HEAP_SIZE_MSMC-1U]);
    fw_mem_alloc_ptr[0] = (uint8_t*)  gUtilsHeapMemMsmc;
    fw_mem_wraparound[0]= FALSE;
    fw_mem_start[1]     = (uint8_t*)  gUtilsHeapMemDdr;
    fw_mem_end[1]       = (uint8_t*)(&gUtilsHeapMemDdr[UTILS_MEM_HEAP_SIZE_DDR-1U]);
    fw_mem_alloc_ptr[1] = (uint8_t*)  gUtilsHeapMemDdr;
    fw_mem_wraparound[1]= FALSE;
    fw_mem_start[2]     = (uint8_t*)  gUtilsHeapMemInternal;
    fw_mem_end[2]       = (uint8_t*)(&gUtilsHeapMemInternal[UTILS_MEM_HEAP_SIZE_INTERNAL-1U]);
    fw_mem_alloc_ptr[2] = (uint8_t*)  gUtilsHeapMemInternal;
    fw_mem_wraparound[2]= FALSE;
    fw_mem_start[3]     = (uint8_t*)  gUtilsHeapMemOspi;
    fw_mem_end[3]       = (uint8_t*)(&gUtilsHeapMemOspi[UTILS_MEM_HEAP_SIZE_OSPI-1U]);
    fw_mem_alloc_ptr[3] = (uint8_t*)  gUtilsHeapMemOspi;
    fw_mem_wraparound[3]= FALSE;

    gUtilsMemClearBuf = TRUE;

    return (UDMA_SOK);
}

int32_t Utils_memDeInit(void)
{
    return (UDMA_SOK);
}

void *Utils_alignedMalloc(uint32_t heapId, uint32_t size, uint32_t alignment)
{
  uint8_t *alloc_ptr;
  void    *p_block = (void *) NULL;

  alloc_ptr = (uint8_t*)align((uintptr_t)fw_mem_alloc_ptr[heapId], alignment);

  if ((alloc_ptr + size) < fw_mem_end[heapId])
  {
    p_block =(void *)alloc_ptr;
    fw_mem_alloc_ptr[heapId] = alloc_ptr + size;
  }
  else
  {
      /* wraparound happened. */
      fw_mem_wraparound[heapId] = TRUE;
      fw_mem_alloc_ptr[heapId] = fw_mem_start[heapId];
      alloc_ptr = (uint8_t*)align((uintptr_t)fw_mem_alloc_ptr[heapId], alignment);
      p_block =(void *)alloc_ptr;
      fw_mem_alloc_ptr[heapId] = alloc_ptr + size;
  }

    return p_block;
}

void Utils_alignedFree(void *p, uint32_t size )
{
  /* Nothing to be done here */
}

void *Utils_memAlloc(uint32_t heapId, uint32_t size, uint32_t align)
{
    void *addr;

    GT_assert(UdmaUtTrace, heapId < UTILS_MEM_HEAP_NUM);

    /* Heap alloc need some minimum allocation size */
    if(size < UDMA_CACHELINE_ALIGNMENT)
    {
        size = UDMA_CACHELINE_ALIGNMENT;
    }

    /* allocate memory */
    addr = Utils_alignedMalloc(heapId, size, align);
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
    GT_assert(UdmaUtTrace, heapId < UTILS_MEM_HEAP_NUM);

    /* Heap alloc need some minimum allocation size */
    if(size < UDMA_CACHELINE_ALIGNMENT)
    {
        size = UDMA_CACHELINE_ALIGNMENT;
    }
    /* free previously allocated memory  */
    Utils_alignedFree(addr, size);
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
    return ((UInt32) 1U);
}

uint32_t Utils_memGetBufferHeapFreeSpace(uint32_t heapId)
{
    return ((UInt32) 0U);
}
