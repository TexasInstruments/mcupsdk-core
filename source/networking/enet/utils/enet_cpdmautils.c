/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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

/*!
 * \file  enet_cpdmautils.c
 *
 * \brief This file contains the implementation of the Enet CPDMA utils functions.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/per/cpsw.h>

#include <include/core/enet_dma.h>
#include <include/core/enet_utils.h>
#include "include/enet_appmemutils.h"
#include "include/enet_appmemutils_cfg.h"
#include "include/enet_apputils.h"
#include "include/enet_appsoc.h"
#include "include/enet_apprm.h"


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

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetAppUtils_freePktInfoQ(EnetDma_PktQ *pPktInfoQ)
{
    EnetDma_Pkt *pktInfo;
    uint32_t pktCnt, i;

    pktCnt = EnetQueue_getQCount(pPktInfoQ);
    /* Free all retrieved packets from DMA */
    for (i = 0U; i < pktCnt; i++)
    {
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(pPktInfoQ);
        EnetMem_freeEthPkt(pktInfo);
    }
}


#if ((__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R'))
#include <kernel/dpl/MpuP_armv7.h>
/* these are defined as part of SysConfig */

#define MPU_SIZE_KB (1024U)
#define MPU_SIZE_MB (MPU_SIZE_KB * MPU_SIZE_KB)
#define MPU_SIZE_GB (MPU_SIZE_KB * MPU_SIZE_MB)

extern MpuP_Config gMpuConfig;
extern MpuP_RegionConfig gMpuRegionConfig[];
const uint32_t gMpuRegionSizeInBytes[] =
{
    [MpuP_RegionSize_32]   =  32,
    [MpuP_RegionSize_64]   =  64,
    [MpuP_RegionSize_128]  = 128,
    [MpuP_RegionSize_256]  = 256,
    [MpuP_RegionSize_512]  = 512,
    [MpuP_RegionSize_1K]   = (1   * MPU_SIZE_KB),
    [MpuP_RegionSize_2K]   = (2   * MPU_SIZE_KB),
    [MpuP_RegionSize_4K]   = (4   * MPU_SIZE_KB),
    [MpuP_RegionSize_8K]   = (8   * MPU_SIZE_KB),
    [MpuP_RegionSize_16K]  = (16  * MPU_SIZE_KB),
    [MpuP_RegionSize_32K]  = (32  * MPU_SIZE_KB),
    [MpuP_RegionSize_64K]  = (64  * MPU_SIZE_KB),
    [MpuP_RegionSize_128K] = (128 * MPU_SIZE_KB),
    [MpuP_RegionSize_256K] = (256 * MPU_SIZE_KB),
    [MpuP_RegionSize_512K] = (512 * MPU_SIZE_KB),
    [MpuP_RegionSize_1M] =   (1   * MPU_SIZE_MB),
    [MpuP_RegionSize_2M] =   (2   * MPU_SIZE_MB),
    [MpuP_RegionSize_4M] =   (4   * MPU_SIZE_MB),
    [MpuP_RegionSize_8M] =   (8   * MPU_SIZE_MB),
    [MpuP_RegionSize_16M] =  (16  * MPU_SIZE_MB),
    [MpuP_RegionSize_32M] =  (32  * MPU_SIZE_MB),
    [MpuP_RegionSize_64M] =  (64  * MPU_SIZE_MB),
    [MpuP_RegionSize_128M] = (128 * MPU_SIZE_MB),
    [MpuP_RegionSize_256M] = (256 * MPU_SIZE_MB),
    [MpuP_RegionSize_512M] = (512 * MPU_SIZE_MB),
    [MpuP_RegionSize_1G] =   (1   * MPU_SIZE_GB),
    [MpuP_RegionSize_2G] =   (2   * MPU_SIZE_GB),
};


bool EnetAppUtils_isDescCached(void)
{
    int32_t i;
    Enet_Type enetType;
    uintptr_t descStartAddr, descEndAddr;
    uint32_t descSize;

#if defined(SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)
    enetType = ENET_CPSW_3G;
#elif (defined(SOC_AM273X) || defined(SOC_AWR294X))
    enetType = ENET_CPSW_2G;
#else
#error "Unknown SOC"
#endif

    EnetSoc_getCppiDescInfo(enetType,0, &descStartAddr, &descSize);
    descEndAddr   = (uintptr_t) (descStartAddr + descSize);
    for (i = (gMpuConfig.numRegions - 1); i >= 0; i--)
    {

        if (gMpuRegionConfig[i].size != MpuP_RegionSize_4G)
        {
            EnetAppUtils_assert(gMpuRegionConfig[i].size >= MpuP_RegionSize_32);
            EnetAppUtils_assert(gMpuRegionConfig[i].size <= ENET_ARRAYSIZE(gMpuRegionSizeInBytes));
            if ((descStartAddr >= gMpuRegionConfig[i].baseAddr)
                &&
                (descEndAddr <= gMpuRegionConfig[i].baseAddr + gMpuRegionSizeInBytes[gMpuRegionConfig[i].size]))
            {
                break;
            }
        }
    }
    EnetAppUtils_assert(i >= 0);
    return (gMpuRegionConfig[i].attrs.isCacheable);
}

#endif
