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
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/hw_include/csl_types.h>

#define RAT_CTRL(baseAddr, i)       (volatile uint32_t*)(baseAddr + 0x20 + 0x10*(i))
#define RAT_BASE(baseAddr, i)       (volatile uint32_t*)(baseAddr + 0x24 + 0x10*(i))
#define RAT_TRANS_L(baseAddr, i)    (volatile uint32_t*)(baseAddr + 0x28 + 0x10*(i))
#define RAT_TRANS_H(baseAddr, i)    (volatile uint32_t*)(baseAddr + 0x2C + 0x10*(i))

AddrTranslateP_Params gAddrTranslateConfig = {
    .numRegions = 0,
    .ratBaseAddr = 0,
    .regionConfig = NULL,
};

static void AddrTranslateP_setRegion(uint32_t ratBaseAddr, uint16_t regionNum,
        uint64_t systemAddr, uint32_t localAddr,
        uint32_t size, uint32_t enable)
{
    uint32_t systemAddrL, systemAddrH;
    if(size > AddrTranslateP_RegionSize_4G)
    {
        size = AddrTranslateP_RegionSize_4G;
    }
    systemAddrL = (uint32_t)(systemAddr & ~( (uint32_t)( ((uint64_t)1 << size) - 1) ));
    systemAddrH = (uint32_t)((systemAddr >> 32 ) & 0xFFFF);
    localAddr = localAddr   & ~( (uint32_t)( ((uint64_t)1 << size) - 1) );

    /* disable RAT region first */
    *RAT_CTRL(ratBaseAddr, regionNum) = 0;
    *RAT_BASE(ratBaseAddr, regionNum) = localAddr;
    *RAT_TRANS_L(ratBaseAddr, regionNum) = systemAddrL;
    *RAT_TRANS_H(ratBaseAddr, regionNum) = systemAddrH;
    /* set size and enable the region */
    *RAT_CTRL(ratBaseAddr, regionNum) = ((enable & 0x1) << 31u) | (size & 0x3F);
}

void AddrTranslateP_Params_init(AddrTranslateP_Params *params)
{
    params->numRegions = 0;
    params->ratBaseAddr = 0;
    params->regionConfig = NULL;
}

void AddrTranslateP_init(AddrTranslateP_Params *params)
{
    uint32_t i;

    if(params!=NULL)
    {
        gAddrTranslateConfig = *params;
    }

    DebugP_assertNoLog(gAddrTranslateConfig.numRegions<AddrTranslateP_MAX_REGIONS);

    for(i=0; i<gAddrTranslateConfig.numRegions; i++)
    {
        DebugP_assertNoLog(gAddrTranslateConfig.ratBaseAddr!=0);
        DebugP_assertNoLog(gAddrTranslateConfig.regionConfig!=NULL);

        /* enable regions setup by user */
        AddrTranslateP_setRegion(
            gAddrTranslateConfig.ratBaseAddr,
            i,
            gAddrTranslateConfig.regionConfig[i].systemAddr,
            gAddrTranslateConfig.regionConfig[i].localAddr,
            gAddrTranslateConfig.regionConfig[i].size,
            1
            );
    }
}

void *AddrTranslateP_getLocalAddr(uint64_t systemAddr)
{
    uint32_t found, regionId;
    void *localAddr;

    DebugP_assertNoLog(gAddrTranslateConfig.numRegions<AddrTranslateP_MAX_REGIONS);

    found = 0;
    for(regionId=0; regionId<gAddrTranslateConfig.numRegions; regionId++)
    {
        uint64_t startAddr, endAddr;
        uint32_t sizeMask;

        /* we assume gAddrTranslateConfig.regionConfig[] address and size is aligned */
        sizeMask = ( (uint32_t)( ((uint64_t)1 << gAddrTranslateConfig.regionConfig[regionId].size) - 1) );

        startAddr = gAddrTranslateConfig.regionConfig[regionId].systemAddr;

        /* calculate end address */
        endAddr = startAddr + sizeMask;

        /* see if input address falls in this region */
        if(systemAddr >= startAddr && systemAddr <= endAddr)
        {
            /* yes, input address falls in this region, break from loop */
            found = 1;
            break;
        }
    }
    if(found)
    {
        /* translate input address to output address */
        uint32_t offset = systemAddr - gAddrTranslateConfig.regionConfig[regionId].systemAddr;

        localAddr = (void *) (gAddrTranslateConfig.regionConfig[regionId].localAddr + offset);
    }
    else
    {
        /* no mapping found, set output = input with 32b truncation */
        localAddr = (void *) systemAddr;
    }
    return localAddr;
}

