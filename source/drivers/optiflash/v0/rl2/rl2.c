/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include <string.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/am263px/cslr_soc_baseaddress.h>
#include <drivers/hw_include/am263px/cslr_soc_defines.h>
#include "rl2.h"

RL2_API_STS_t RL2_configure(RL2_Params *config)
{
    RL2_API_STS_t retStatus = RL2_API_STS_SUCCESS;
    if(config == NULL)
    {
        retStatus = RL2_API_STS_CANNOT_CONFIGURE;
    }
    else
    {
        uint32_t requried_cache_size = 0;
        switch (config->cacheSize)
        {
        case RL2_CACHESIZE_8K:
        case RL2_CACHESIZE_16K:
        case RL2_CACHESIZE_32K:
        case RL2_CACHESIZE_64K:
        case RL2_CACHESIZE_128K:
        case RL2_CACHESIZE_256K:
            requried_cache_size = 1;
            break;
        default:
            requried_cache_size = 0;
            break;
        }

        /* check if region size is given something else which is not known */
        if(requried_cache_size == (uint32_t)0)
        {
            retStatus = (RL2_API_STS_CANNOT_CONFIGURE | RL2_API_STS_UNKNOWN_CACHE_SIZE);
        }
        else
        {
            /* check if the sum of length of all regions is equal to the size of rl2 cache required */
            if((config->l2Sram0Len + config->l2Sram1Len + config->l2Sram2Len) != requried_cache_size)
            {
                retStatus = (RL2_API_STS_CANNOT_CONFIGURE | RL2_API_STS_WRONG_REGION_CALCULATION);
            }
            else
            {
                /* check if user is writing to the reserved bits of the registors */
                if(
                    (uint32_t)(config->l2Sram0Base & (CSL_RL2_OF_R5FSS0_CORE0_REM_ADR_ADR_LSW_MASK)) != (uint32_t)0 ||
                    (uint32_t)(config->l2Sram1Base & (CSL_RL2_OF_R5FSS0_CORE0_REM_ADR_ADR_LSW_MASK)) != (uint32_t)0 ||
                    (uint32_t)(config->l2Sram2Base & (CSL_RL2_OF_R5FSS0_CORE0_REM_ADR_ADR_LSW_MASK)) != (uint32_t)0 ||
                    (uint32_t)(config->rangeStart & (CSL_RL2_OF_R5FSS0_CORE0_L2_LO_ADDRESS_LO_LSW_MASK)) != (uint32_t)0 ||
                    (uint32_t)(config->rangeEnd & (CSL_RL2_OF_R5FSS0_CORE0_L2_HI_ADDRESS_HI_LSW_MASK)) != (uint32_t)0 ||
                    (uint32_t)(config->l2Sram0Len & (CSL_RL2_OF_R5FSS0_CORE0_REM_LEN_LEN_MASK)) != (uint32_t)0 ||
                    (uint32_t)(config->l2Sram1Len & (CSL_RL2_OF_R5FSS0_CORE0_REM_LEN_LEN_MASK)) != (uint32_t)0 ||
                    (uint32_t)(config->l2Sram2Len & (CSL_RL2_OF_R5FSS0_CORE0_REM_LEN_LEN_MASK)) != (uint32_t)0
                )
                {
                    retStatus = (RL2_API_STS_CANNOT_CONFIGURE | RL2_API_STS_RESERVED_BIT_WRITE);
                }
                else
                {
                    /* check if regions are overlapping */
                    {
                        uint32_t r1start = config->l2Sram0Base, r2start = config->l2Sram1Base, r3start = config->l2Sram0Base;
                        uint32_t r1end = r1start + config->l2Sram0Len - (uint32_t)1, r2end = r2start + config->l2Sram1Len - (uint32_t)1, r3end = r3start + config->l2Sram2Len - (uint32_t)1;
                        uint32_t ranges[][2] = {{r1start, r1end}, {r2start, r2end}, {r3start, r3end}};
                        for(int i=0;i < (int)(sizeof(ranges)/sizeof(*ranges));i++)
                        {
                            for(int j=0;j < (int)(sizeof(ranges)/sizeof(*ranges));j++)
                            {
                                if(i==j)
                                {
                                    continue;
                                }
                                if(ranges[i][0] >= ranges[j][0] && ranges[i][0] <= ranges[j][1])
                                {
                                    retStatus = (RL2_API_STS_CANNOT_CONFIGURE | RL2_API_STS_REGIONS_OVERLAPPING);
                                    break;
                                }
                                if(ranges[i][1] >= ranges[j][0] && ranges[i][1] <= ranges[j][1])
                                {
                                    retStatus = (RL2_API_STS_CANNOT_CONFIGURE | RL2_API_STS_REGIONS_OVERLAPPING);
                                    break;
                                }
                            }
                        }
                    }

                    if(retStatus == RL2_API_STS_SUCCESS)
                    {
                        CSL_rl2_of_r5fss0_core0Regs *regs;
                        regs = (CSL_rl2_of_r5fss0_core0Regs*)config->baseAddress;
                        // set the regions
                        regs->REM[0].ADR = config->l2Sram0Base;
                        regs->REM[0].LEN = config->l2Sram0Len;
                        regs->REM[1].ADR = config->l2Sram1Base;
                        regs->REM[1].LEN = config->l2Sram1Len;
                        regs->REM[2].ADR = config->l2Sram2Base;
                        regs->REM[2].LEN = config->l2Sram2Len;
                        regs->L2_LO = config->rangeStart;
                        regs->L2_HI = config->rangeEnd;
                        regs->L2_CTRL |= CSL_RL2_OF_R5FSS0_CORE0_L2_CTRL_ENABLE_MASK | ((((uint32_t)(config->cacheSize)) << CSL_RL2_OF_R5FSS0_CORE0_L2_CTRL_SIZE_SHIFT) & CSL_RL2_OF_R5FSS0_CORE0_L2_CTRL_SIZE_MASK);
                        while(regs->L2_STS != CSL_RL2_OF_R5FSS0_CORE0_L2_STS_OK_TO_GO_MASK);
                    }
                }
            }
        }
    }
    return retStatus;
}

RL2_API_STS_t RL2_initparams(RL2_Params * config)
{
    RL2_API_STS_t retStatus = RL2_API_STS_SUCCESS;
    if(config == NULL)
    {
        retStatus = RL2_API_STS_CANNOT_CONFIGURE;
    }
    else
    {
        (void)memset(config, 0, sizeof(RL2_Params));
        config->cacheSize = RL2_CACHESIZE_128K;
    }
    return retStatus;
}

RL2_API_STS_t RL2_setInterrupt(RL2_Params * config, RL2_Interrupt intr)
{
    RL2_API_STS_t retStatus = RL2_API_STS_SUCCESS;
    if(config == NULL)
    {
        retStatus = RL2_API_STS_CANNOT_CONFIGURE;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)config->baseAddress;
        switch(intr)
        {
            case RL2_INTERRUPT_WRITE_HIT:
                regs->IRQENABLE_SET |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_WR_HIT_MASK;
                break;
            case RL2_INTERRUPT_WRITE_ERROR:
                regs->IRQENABLE_SET |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_FLC_WRERR_MASK;
                break;
            default:
                retStatus = RL2_API_STS_UNKNOWN_INTERRUPT;
                break;
        }
    }
    return retStatus;
}

RL2_API_STS_t RL2_clearInterrupt(RL2_Params * config, RL2_Interrupt intr)
{
    RL2_API_STS_t retStatus = RL2_API_STS_SUCCESS;
    if(config == NULL)
    {
        retStatus = RL2_API_STS_CANNOT_CONFIGURE;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)config->baseAddress;
        switch(intr)
        {
            case RL2_INTERRUPT_WRITE_HIT:
                regs->IRQENABLE_CLR |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_WR_HIT_MASK;
                break;
            case RL2_INTERRUPT_WRITE_ERROR:
                regs->IRQENABLE_CLR |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_FLC_WRERR_MASK;
                break;
            default:
                retStatus = RL2_API_STS_UNKNOWN_INTERRUPT;
                break;
        }
    }
    return retStatus;
}

RL2_API_STS_t RL2_readIRQMask(RL2_Params * config, uint32_t * status)
{
    RL2_API_STS_t retStatus = RL2_API_STS_SUCCESS;
    if(config == NULL || status == NULL)
    {
        retStatus = RL2_API_STS_CANNOT_CONFIGURE;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)config->baseAddress;
        *status = regs->IRQSTATUS_MSK;
    }
    return retStatus;
}

RL2_API_STS_t RL2_readIRQStatus(RL2_Params * config, uint32_t * status)
{
    RL2_API_STS_t retStatus = RL2_API_STS_SUCCESS;
    if(config == NULL || status == NULL)
    {
        retStatus = RL2_API_STS_CANNOT_CONFIGURE;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)config->baseAddress;
        *status = regs->IRQSTATUS_RAW;
    }
    return retStatus;
}

RL2_API_STS_t RL2_getCacheMiss(RL2_Params * config, uint32_t * miss)
{
    RL2_API_STS_t ret = RL2_API_STS_SUCCESS;
    if(NULL == config || NULL == miss)
    {
        ret = RL2_API_STS_CANNOT_CONFIGURE;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)config->baseAddress;
        *miss = regs->L2MC;
    }
    return ret;
}

RL2_API_STS_t RL2_getCacheHits(RL2_Params * config, uint32_t * hits)
{
    RL2_API_STS_t ret = RL2_API_STS_SUCCESS;
    if(NULL == config || NULL == hits)
    {
        ret = RL2_API_STS_CANNOT_CONFIGURE;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)config->baseAddress;
        *hits = regs->L2HC;
    }
    return ret;
}