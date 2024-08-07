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
#include "flc.h"

FLC_API_STS_t FLC_configureRegion(FLC_RegionInfo * const regionInfo)
{
    FLC_API_STS_t status = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo*)NULL == regionInfo)
    {
        status = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        if(regionInfo->regionId >= (FLC_regionId)FLC_MAX_REGION)
        {
            status = FLC_API_STS_ERROR_ILLEGAL_REGION_ID;
        }
        else
        {
            CSL_rl2_of_r5fss0_core0Regs *regs;
            regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
            // first clear feanble bit as if it is left set, it won't let new FLC config to be written in the registers
            regs->FLC[regionInfo->regionId].CTL &= ~(CSL_RL2_OF_R5FSS0_CORE0_FLC_CTL_FENABLE_MASK);
            // configure this FLC
            regs->FLC[regionInfo->regionId].LO = regionInfo->sourceStartAddress;
            regs->FLC[regionInfo->regionId].HI = regionInfo->sourceEndAddress;
            regs->FLC[regionInfo->regionId].RA = regionInfo->destinationStartAddress;
        }
    }
    return status;
}

FLC_API_STS_t FLC_startRegion(FLC_RegionInfo * const regionInfo)
{
    FLC_API_STS_t status = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo * const)NULL == regionInfo)
    {
        status = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        CSL_rl2_of_r5fss0_core0Regs_flc *flcRegs = &(regs->FLC[regionInfo->regionId]);
        flcRegs->CTL |= CSL_RL2_OF_R5FSS0_CORE0_FLC_CTL_FENABLE_MASK;
    }
    return status;
}

FLC_API_STS_t FLC_isRegionDone(FLC_RegionInfo * const regionInfo, uint32_t *status)
{
    FLC_API_STS_t retStatus = FLC_API_STS_SUCCESS;
    if(((FLC_RegionInfo * const)NULL == regionInfo) || (NULL == status))
    {
        retStatus = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        *status = regs->FLC_STS;
    }
    return retStatus;
}

FLC_API_STS_t FLC_wasReadError(FLC_RegionInfo * const regionInfo, uint32_t *status)
{
    FLC_API_STS_t retStatus = FLC_API_STS_SUCCESS;
    if(((FLC_RegionInfo * const)NULL == regionInfo) || (NULL == status))
    {
        retStatus = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        *status = (regs->IRQSTATUS_RAW & CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_FLC_RDERR_MASK) >> CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_FLC_RDERR_SHIFT;
    }
    return retStatus;
}

FLC_API_STS_t FLC_wasWriteError(FLC_RegionInfo * const regionInfo, uint32_t *status)
{
    FLC_API_STS_t retStatus = FLC_API_STS_SUCCESS;
    if(((FLC_RegionInfo * const)NULL == regionInfo) || (NULL == status))
    {
        retStatus = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        *status = (regs->IRQSTATUS_RAW & CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_WR_ERR_MASK) >> CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_WR_HIT_SHIFT;
    }
    return retStatus;
}

FLC_API_STS_t FLC_clearWriteError(FLC_RegionInfo * const regionInfo)
{
    FLC_API_STS_t status = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo * const)NULL == regionInfo)
    {
        status = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        regs->IRQSTATUS_MSK |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_WRERR_MASK;
    }
    return status;
}

FLC_API_STS_t FLC_clearReadError(FLC_RegionInfo * const regionInfo)
{
    FLC_API_STS_t status = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo * const)NULL == regionInfo)
    {
        status = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        regs->IRQSTATUS_MSK |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_RDERR_MASK;
    }
    return status;
}

FLC_API_STS_t FLC_readIRQMask(FLC_RegionInfo * const regionInfo, uint32_t *status)
{
    FLC_API_STS_t retStatus = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo * const)NULL == regionInfo)
    {
        retStatus = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        *status = regs->IRQSTATUS_MSK;
    }
    return retStatus;
}

FLC_API_STS_t FLC_readIRQStatus(FLC_RegionInfo * const regionInfo, uint32_t *status)
{
    FLC_API_STS_t retStatus = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo * const)NULL == regionInfo)
    {
        retStatus = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        *status = regs->IRQSTATUS_RAW;
    }
    return retStatus;
}

FLC_API_STS_t FLC_enableInterrupt(FLC_RegionInfo * const regionInfo, FLC_Interrupt intr)
{
    FLC_API_STS_t status = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo * const)NULL == regionInfo)
    {
        status = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        switch(intr)
        {
            case FLC_INTERRUPT_DONE:
                regs->IRQENABLE_SET |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_DON_MASK;
                break;
            case FLC_INTERRUPT_WRITE_ERROR:
                regs->IRQENABLE_SET |= CSL_RL2_OF_R5FSS0_CORE0_IRQENABLE_SET_EN_FLC_WRERR_MASK;
                break;
            case FLC_INTERRUPT_READ_ERROR:
                regs->IRQENABLE_SET |= CSL_RL2_OF_R5FSS0_CORE0_IRQENABLE_SET_EN_FLC_RDERR_MASK;
                break;
            default:
                status = FLC_API_STS_ERROR_UNKNOWN_INTERRUPT;
                break;
        }
    }
    return status;
}

FLC_API_STS_t FLC_clearInterrupt(FLC_RegionInfo * const regionInfo, FLC_Interrupt intr)
{
    FLC_API_STS_t status = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo * const)NULL == regionInfo)
    {
        status = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        switch(intr)
        {
            case FLC_INTERRUPT_DONE:
                regs->IRQSTATUS_MSK |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_DON_MASK;
                break;
            case FLC_INTERRUPT_WRITE_ERROR:
                regs->IRQSTATUS_MSK |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_WRERR_MASK;
                break;
            case FLC_INTERRUPT_READ_ERROR:
                regs->IRQSTATUS_MSK |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_RDERR_MASK;
                break;
            default:
                status = FLC_API_STS_ERROR_UNKNOWN_INTERRUPT;
                break;
        }
    }
    return status;
}

FLC_API_STS_t FLC_disableInterrupt(FLC_RegionInfo * const regionInfo, FLC_Interrupt intr)
{
    FLC_API_STS_t status = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo * const)NULL == regionInfo)
    {
        status = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        switch(intr)
        {
            case FLC_INTERRUPT_DONE:
                regs->IRQENABLE_CLR |= CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_DON_MASK;
                break;
            case FLC_INTERRUPT_WRITE_ERROR:
                regs->IRQENABLE_CLR |= CSL_RL2_OF_R5FSS0_CORE0_IRQENABLE_CLR_EN_FLC_WRERR_MASK;
                break;
            case FLC_INTERRUPT_READ_ERROR:
                regs->IRQENABLE_CLR |= CSL_RL2_OF_R5FSS0_CORE0_IRQENABLE_CLR_EN_FLC_RDERR_MASK;
                break;
            default:
                status = FLC_API_STS_ERROR_UNKNOWN_INTERRUPT;
                break;
        }
    }
    return status;
}

FLC_API_STS_t FLC_disable(FLC_RegionInfo * const regionInfo)
{
    FLC_API_STS_t status = FLC_API_STS_SUCCESS;
    if((FLC_RegionInfo * const)NULL == regionInfo)
    {
        status = FLC_API_STS_ERROR_NULL_PTR;
    }
    else
    {
        CSL_rl2_of_r5fss0_core0Regs *regs;
        regs = (CSL_rl2_of_r5fss0_core0Regs*)regionInfo->baseAddress;
        regs->FLC[regionInfo->regionId].CTL &= ~(CSL_RL2_OF_R5FSS0_CORE0_FLC_CTL_FENABLE_MASK);
    }
    return status;
}
