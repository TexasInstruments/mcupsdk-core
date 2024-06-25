/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 *  \file gpmc_priv_v0.c
 *
 *  \brief File containing private GPMC Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/gpmc.h>
#include <drivers/elm.h>
#include <drivers/gpmc/v0/dma/gpmc_dma.h>
#include "gpmc_priv_v0.h"

void GPMC_disableInterupt(uint32_t baseAddr, uint32_t interupt)
{
    /* Disable GPMC interupt. */
    switch (interupt)
    {
        case GPMC_FIFOEVENT_INT:
            CSL_REG32_FINS(baseAddr + CSL_GPMC_IRQENABLE,
                          GPMC_IRQENABLE_FIFOEVENTENABLE,
                          CSL_GPMC_IRQENABLE_FIFOEVENTENABLE_FIFOMASKED);
            break;
        case GPMC_TERMINALCOUNT_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE,
                CSL_GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE_TCMASKED);
            break;
        case GPMC_WAIT0EDGEDETECTION_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE,
                CSL_GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE_W0MASKED);
            break;
        case GPMC_WAIT1EDGEDETECTION_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE,
                CSL_GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE_W1MASKED);
            break;

        default:
            break;
    }
}

void GPMC_enableInterupt(uint32_t baseAddr, uint32_t interupt)
{
    /* Enable GPMC interupt. */
    switch (interupt)
    {
        case GPMC_FIFOEVENT_INT:
            CSL_REG32_FINS(baseAddr + CSL_GPMC_IRQENABLE,
                          GPMC_IRQENABLE_FIFOEVENTENABLE,
                          CSL_GPMC_IRQENABLE_FIFOEVENTENABLE_FIFOENABLED);
            break;
        case GPMC_TERMINALCOUNT_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE,
                CSL_GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE_TCENABLED);
            break;
        case GPMC_WAIT0EDGEDETECTION_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE,
                CSL_GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE_W0ENABLED);
            break;
        case GPMC_WAIT1EDGEDETECTION_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE,
                CSL_GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE_W1ENABLED);
            break;

        default:
            break;
    }
}

void GPMC_interuptStatusClear(uint32_t baseAddr, uint32_t interupt)
{
    /* Clear GPMC interupt status. */
    switch (interupt)
    {
        case GPMC_FIFOEVENT_STATUS:
            CSL_REG32_FINS(baseAddr + CSL_GPMC_IRQSTATUS,
                          GPMC_IRQSTATUS_FIFOEVENTSTATUS,
                          CSL_GPMC_IRQSTATUS_FIFOEVENTSTATUS_FIFOSTAT1_W);
            break;
        case GPMC_TERMINALCOUNT_STATUS:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQSTATUS, GPMC_IRQSTATUS_TERMINALCOUNTSTATUS,
                CSL_GPMC_IRQSTATUS_TERMINALCOUNTSTATUS_TCSTAT1_W);
            break;
        case GPMC_WAIT0EDGEDETECTION_STATUS:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQSTATUS,
                GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS,
                CSL_GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS_W0DET1_W);
            break;
        case GPMC_WAIT1EDGEDETECTION_STATUS:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQSTATUS,
                GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS,
                CSL_GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS_W1DET1_W);
            break;

        default:
            break;
    }
}

uint32_t  GPMC_interuptStatusGet(uint32_t baseAddr, uint32_t interupt)
{
    uint32_t retVal;

    retVal = 0;
    /* Get GPMC interupt status. */
    switch (interupt)
    {
        case GPMC_FIFOEVENT_STATUS:
            retVal = CSL_REG32_FEXT(baseAddr + CSL_GPMC_IRQSTATUS,
                                   GPMC_IRQSTATUS_FIFOEVENTSTATUS);
            break;
        case GPMC_TERMINALCOUNT_STATUS:
            retVal = CSL_REG32_FEXT(baseAddr + CSL_GPMC_IRQSTATUS,
                                   GPMC_IRQSTATUS_TERMINALCOUNTSTATUS);
            break;
        case GPMC_WAIT0EDGEDETECTION_STATUS:
            retVal = CSL_REG32_FEXT(baseAddr + CSL_GPMC_IRQSTATUS,
                                   GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS);
            break;
        case GPMC_WAIT1EDGEDETECTION_STATUS:
            retVal = CSL_REG32_FEXT(baseAddr + CSL_GPMC_IRQSTATUS,
                                   GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS);
            break;

        default:
            break;
    }

    return (retVal);
}

int32_t GPMC_disableFlashWriteProtect(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;
        /* Disable write protect. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_CONFIG, GPMC_CONFIG_WRITEPROTECT, \
                        CSL_GPMC_CONFIG_WRITEPROTECT_WPHIGH);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_enableFlashWriteProtect(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;

        /* Enable Write protect. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_CONFIG, GPMC_CONFIG_WRITEPROTECT, \
                        CSL_GPMC_CONFIG_WRITEPROTECT_WPHIGH);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_waitPinInteruptStatusReadyWaitTimeout(GPMC_Handle handle, uint32_t timeOut)
{
    int32_t status  =   SystemP_SUCCESS;
    uint32_t waitPinInterupt = 0;
    uint64_t curTime = 0;

    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;

        if(hwAttrs->waitPinNum == CSL_GPMC_CONFIG1_WAITPINSELECT_W0)
        {
            waitPinInterupt = GPMC_WAIT0EDGEDETECTION_STATUS;
        }
        else if(hwAttrs->waitPinNum == CSL_GPMC_CONFIG1_WAITPINSELECT_W1)
        {
            waitPinInterupt = GPMC_WAIT1EDGEDETECTION_STATUS;
        }

        if(timeOut != 0)
        {

            curTime = ClockP_getTimeUsec();
            /* Check WAIT PIN interupt status with timeout.*/
            while((GPMC_interuptStatusGet(hwAttrs->gpmcBaseAddr,waitPinInterupt) == 0) \
            && ((ClockP_getTimeUsec() - curTime) < timeOut))
            {
                /* Do nothing.*/
            }


            if(GPMC_interuptStatusGet(hwAttrs->gpmcBaseAddr,waitPinInterupt) == 1)
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }

        }
        else
        {
            if(GPMC_interuptStatusGet(hwAttrs->gpmcBaseAddr,hwAttrs->waitPinNum) == 1)
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;

 }

int32_t GPMC_waitPinStatusReadyWaitTimeout(GPMC_Handle handle, uint32_t timeOut)
{

    int32_t status  =   SystemP_SUCCESS;
    const GPMC_HwAttrs *hwAttrs = NULL;
    uint64_t curTime= 0;

    if(handle != NULL)
    {
        hwAttrs = ((GPMC_Config*)handle)->attrs;

        if(timeOut != 0)
        {
            curTime = ClockP_getTimeUsec();
            /* Check WAIT PIN status with timeout. */
            while((GPMC_waitPinStatusGet(hwAttrs->gpmcBaseAddr,hwAttrs->waitPinNum) == \
            CSL_GPMC_STATUS_WAIT0STATUS_W0ACTIVEL) && ((ClockP_getTimeUsec() - curTime) < timeOut))
            {
                /* Do nothing. */
            }

            if((GPMC_waitPinStatusGet(hwAttrs->gpmcBaseAddr,hwAttrs->waitPinNum) == \
            CSL_GPMC_STATUS_WAIT0STATUS_W0ACTIVEH))
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            if((GPMC_waitPinStatusGet(hwAttrs->gpmcBaseAddr,hwAttrs->waitPinNum) == \
            CSL_GPMC_STATUS_WAIT0STATUS_W0ACTIVEH))
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;

 }

int32_t GPMC_isDmaRestrictedRegion(GPMC_Handle handle, uint32_t addr)
{
    int32_t isRestricted = FALSE;
    const GPMC_HwAttrs *attrs = ((GPMC_Config *)handle)->attrs;

    if(NULL != attrs->dmaRestrictedRegions)
    {
        const GPMC_AddrRegion *addrRegions = attrs->dmaRestrictedRegions;
        uint32_t i = 0;
        uint32_t start;
        uint32_t size;

        /* Check for DMA restricted regions. */
        while(addrRegions[i].regionStartAddr != 0xFFFFFFFF)
        {
            start = addrRegions[i].regionStartAddr;
            size = addrRegions[i].regionSize;

            if((addr >= start) && (addr < (start + size)))
            {
                isRestricted = TRUE;
                break;
            }
            i++;
        }
    }

    return isRestricted;
}

uint32_t GPMC_waitPinStatusGet(uint32_t baseAddr, uint32_t pin)
{
    uint32_t pinStatus;

    pinStatus = 0;
    /* Check WAIT PIN status. */
    if (pin == CSL_GPMC_CONFIG1_WAITPINSELECT_W0)
    {
        pinStatus = CSL_REG32_FEXT(baseAddr + CSL_GPMC_STATUS,
                                  GPMC_STATUS_WAIT0STATUS);
    }
    else if (pin == CSL_GPMC_CONFIG1_WAITPINSELECT_W1)
    {
        pinStatus = CSL_REG32_FEXT(baseAddr + CSL_GPMC_STATUS,
                                  GPMC_STATUS_WAIT1STATUS);
    }
    else
    {
        /*
         * Do nothing. Error will be generated by the hardware
         */
    }

    return (pinStatus);
}