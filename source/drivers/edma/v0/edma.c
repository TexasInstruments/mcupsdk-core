/*
 * Copyright (C) 2022-23 Texas Instruments Incorporated
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
 *  \file   edma.c
 *
 *  \brief  This file contains device abstraction layer APIs for the EDMA device.
 *          There are APIs here to enable the EDMA instance, set the required
 *          configurations for communication, transmit or receive data.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stdint.h>
#include <drivers/edma.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                        Static Function Declaration                         */
/* ========================================================================== */
static int32_t EDMA_initialize (uint32_t baseAddr, const EDMA_InitParams *initParam);
static int32_t EDMA_deinitialize (uint32_t baseAddr, const EDMA_InitParams *initParam);
static void    EDMA_transferCompletionMasterIsrFxn(void *args);
static int32_t Alloc_resource(const EDMA_Attrs *attrs, EDMA_Object *object, uint32_t *resId, uint32_t resType);
static uint32_t EDMA_isDmaChannelAllocated(EDMA_Handle handle, const uint32_t *dmaCh);
static uint32_t EDMA_isTccAllocated(EDMA_Handle handle, const uint32_t *tcc);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void EDMA_initParamsInit (EDMA_InitParams *initParam)
{
    uint32_t i;

    if (initParam != NULL)
    {
        initParam->regionId     = 0U;
        initParam->queNum       = 0U;
        initParam->ownResource.qdmaCh      = 0xFFU;
        initParam->initParamSet = FALSE;
        for (i = 0U; i < (SOC_EDMA_NUM_DMACH/32U); i++)
        {
            initParam->ownResource.dmaCh[i]      = 0xFFFFFFFFU;
            initParam->reservedDmaCh[i] = 0U;
        }
        for (i = 0U; i < (EDMA_NUM_TCC/32U); i++)
        {
            initParam->ownResource.tcc[i]      = 0xFFFFFFFFU;
        }
        for (i = 0U; i < (SOC_EDMA_NUM_PARAMSETS/32U); i++)
        {
            initParam->ownResource.paramSet[i]      = 0xFFFFFFFFU;
        }
    }
}

uint32_t EDMA_isInitialized(EDMA_Handle handle)
{
    EDMA_Config    *config;
    EDMA_Object    *object;
    uint32_t        edmaStatus = FALSE;

    if (handle != NULL)
    {
        config = (EDMA_Config *) handle;
        object = config->object;
        edmaStatus = object->isOpen;
    }

    return edmaStatus;
}

static int32_t EDMA_initialize (uint32_t baseAddr, const EDMA_InitParams *initParam)
{
    int32_t retVal = SystemP_SUCCESS;
    EDMACCPaRAMEntry paramSet;
    uint32_t count = 0;
    uint32_t i     = 0;
    uint32_t qnumValue;

    /* Clear the Event miss Registers */
    HW_WR_REG32(baseAddr + EDMA_TPCC_EMCR, initParam->ownResource.dmaCh[0]);
#if SOC_EDMA_NUM_DMACH > 32
    HW_WR_REG32(baseAddr + EDMA_TPCC_EMCRH, initParam->ownResource.dmaCh[1]);
#endif
    HW_WR_REG32(baseAddr + EDMA_TPCC_QEMCR, initParam->ownResource.qdmaCh);

    /* Clear CCERR register */
    HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR, EDMA_SET_ALL_BITS);

    /* Disable and clear DMA events for all own dma channels */
    for (i = 0; i < SOC_EDMA_NUM_DMACH; i++)
    {
        if (((1U << (i%32U)) & initParam->ownResource.dmaCh[i/32U]) != 0U)
        {
            EDMA_disableDmaEvtRegion(baseAddr, initParam->regionId, i);
            EDMA_clrEvtRegion(baseAddr, initParam->regionId, i);
            EDMA_clrMissEvtRegion(baseAddr, initParam->regionId, i);
        }
    }

    /* Disable and clear channel interrupts for all own dma channels */
    for (i = 0; i < EDMA_NUM_TCC; i++)
    {
        if (((1U << (i%32U)) & initParam->ownResource.tcc[i/32U]) != 0U)
        {
            EDMA_disableEvtIntrRegion(baseAddr, initParam->regionId, i);
            EDMA_clrIntrRegion(baseAddr, initParam->regionId, i);
        }
    }

    /* Disable and clear channel interrupts for all own qdma channels */
    for (i = 0; i < SOC_EDMA_NUM_QDMACH; i++)
    {
        if (((1U << i) & initParam->ownResource.qdmaCh) != 0U)
        {
            EDMA_disableQdmaEvtRegion(baseAddr, initParam->regionId, i);
            EDMA_qdmaClrMissEvtRegion(baseAddr, initParam->regionId, i);
        }
    }
    /* FOR TYPE EDMA*/
    /* Enable the own DMA (0 - 64) channels in the DRAE and DRAEH register */
    HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(initParam->regionId), initParam->ownResource.dmaCh[0]);
#if SOC_EDMA_NUM_DMACH > 32
    HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(initParam->regionId), initParam->ownResource.dmaCh[1]);
#endif
    /* Enable the own TCCs also for the region in DRAE and DRAEH register */
    HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(initParam->regionId), initParam->ownResource.tcc[0]);
#if SOC_EDMA_NUM_DMACH > 32
    HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(initParam->regionId), initParam->ownResource.tcc[1]);
#endif
    if( SOC_EDMA_CHMAPEXIST != 0U)
    {
        for (i = 0U; i < SOC_EDMA_NUM_DMACH; i++)
        {
            if (((1U << (i%32U)) & initParam->ownResource.dmaCh[i/32U]) != 0U)
            {
                /* All channels are one to one mapped with the params */
                HW_WR_REG32(baseAddr + EDMA_TPCC_DCHMAPN(i), i << 5);
            }
        }
    }

    /* Initialize the DMA Queue Number Registers */
    for (count = 0U; count < SOC_EDMA_NUM_DMACH; count++)
    {
        if (((1U << (count%32U)) & initParam->ownResource.dmaCh[count/32U]) != 0U)
        {
            qnumValue  = HW_RD_REG32(baseAddr + (EDMA_TPCC_DMAQNUMN((count >> 3U))));
            qnumValue &= EDMACC_DMAQNUM_CLR(count);
            qnumValue |= EDMACC_DMAQNUM_SET(count, initParam->queNum);
            HW_WR_REG32(baseAddr + (EDMA_TPCC_DMAQNUMN((count >> 3U))), qnumValue);
        }
    }

    /* FOR TYPE QDMA */
    /* Enable the QDMA (0 - 64) channels in the DRAE register */
    HW_WR_REG32(baseAddr + EDMA_TPCC_QRAEN(initParam->regionId), initParam->ownResource.qdmaCh);

    /* Initialize the QDMA Queue Number Registers */
    for (count = 0U; count < SOC_EDMA_NUM_QDMACH; count++)
    {
        if (((1U << count) & initParam->ownResource.qdmaCh) != 0U)
        {
            qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QDMAQNUM);
            qnumValue &= EDMACC_QDMAQNUM_CLR(count);
            qnumValue |= EDMACC_QDMAQNUM_SET(count, initParam->queNum);
            HW_WR_REG32(baseAddr + EDMA_TPCC_QDMAQNUM, qnumValue);
        }
    }

    if (initParam->initParamSet == TRUE)
    {
        EDMA_ccPaRAMEntry_init(&paramSet);
        /* cleanup Params, note h/w reset state is all 0s, must be done after
        disabling/clearning channel events (in particular QDMA) */
        for (count = 0; count < SOC_EDMA_NUM_PARAMSETS; count++)
        {
            if (((1U << (count%32U)) & initParam->ownResource.paramSet[count/32U]) != 0U)
            {
                EDMA_setPaRAM(baseAddr, count, &paramSet);
            }
        }
    }
    return retVal;
}

void EDMA_ccPaRAMEntry_init(EDMACCPaRAMEntry *paramEntry)
{
    if (paramEntry != NULL)
    {
        /* Initialize all PaRAM entries as 0 */
        paramEntry->opt = 0;
        paramEntry->srcAddr = 0;
        paramEntry->aCnt = 0;
        paramEntry->bCnt = 0;
        paramEntry->destAddr = 0;
        paramEntry->srcBIdx = 0;
        paramEntry->destBIdx = 0;
        paramEntry->linkAddr = 0;
        paramEntry->bCntReload = 0;
        paramEntry->srcCIdx = 0;
        paramEntry->destCIdx = 0;
        paramEntry->cCnt = 0;
        paramEntry->srcBIdxExt = 0;
        paramEntry->destBIdxExt = 0;
    }
}

uint32_t EDMA_peripheralIdGet(uint32_t baseAddr)
{
    return (HW_RD_REG32(baseAddr + EDMA_TPCC_PID));
}

void EDMA_enableChInShadowRegRegion(uint32_t baseAddr,
                                    uint32_t regionId,
                                    uint32_t chType,
                                    uint32_t chNum)
{
    uint32_t draeValue;
    /* Allocate the DMA/QDMA channel */
    if (EDMA_CHANNEL_TYPE_DMA == chType)
    {
        /* FOR TYPE EDMA*/
        if (chNum < 32U)
        {
            draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId));
            /* Enable the DMA channel in the DRAE registers */
            draeValue |= (uint32_t) 0x01 << chNum;
            HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId), draeValue);
        }
        else
        {
            draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId));

            /* Enable the DMA channel in the DRAEH registers */
            draeValue |= (uint32_t) 0x01 << (chNum - 32U);
            HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId), draeValue);
        }
    }
    else if (EDMA_CHANNEL_TYPE_QDMA == chType)
    {
        /* FOR TYPE QDMA */
        /* Enable the QDMA channel in the DRAE/DRAEH registers */
        draeValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QRAEN(regionId));
        draeValue |= (uint32_t) 0x01 << chNum;
        HW_WR_REG32(baseAddr + EDMA_TPCC_QRAEN(regionId), draeValue);
    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

void EDMA_disableChInShadowRegRegion(uint32_t baseAddr,
                                     uint32_t regionId,
                                     uint32_t chType,
                                     uint32_t chNum)
{
    uint32_t draeValue;
    /* Allocate the DMA/QDMA channel */
    if (EDMA_CHANNEL_TYPE_DMA == chType)
    {
        /* FOR TYPE EDMA*/
        if (chNum < 32U)
        {
            draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId));
            /* Disable the DMA channel in the DRAE registers */
            draeValue &= ~((uint32_t) 0x01 << chNum);
            HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId), draeValue);
        }
        else
        {
            draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId));
            /* Disable the DMA channel in the DRAEH registers */
            draeValue &= ~((uint32_t) 0x01 << (chNum - 32U));
            HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId), draeValue);
        }
    }
    else if (EDMA_CHANNEL_TYPE_QDMA == chType)
    {
        /* FOR TYPE QDMA */
        draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_QRAEN(regionId));
        /* Disable the QDMA channel in the DRAE/DRAEH registers */
        draeValue &= ~((uint32_t) 0x01) << chNum;
        HW_WR_REG32(baseAddr + EDMA_TPCC_QRAEN(regionId), draeValue);
    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

static void EDMA_enableTccInShadowRegRegion(uint32_t baseAddr,
                                    uint32_t regionId,
                                    uint32_t tccNum)
{
    uint32_t draeValue;

    if (tccNum < 32U)
    {
        draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId));
        /* Enable the DMA channel in the DRAE registers */
        draeValue |= (uint32_t) 0x01 << tccNum;
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId), draeValue);
    }
    else
    {
        draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId));

        /* Enable the DMA channel in the DRAEH registers */
        draeValue |= (uint32_t) 0x01 << (tccNum - 32U);
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId), draeValue);
    }
}

static void EDMA_disableTccInShadowRegRegion(uint32_t baseAddr,
                                     uint32_t regionId,
                                     uint32_t tccNum)
{
    uint32_t draeValue;

    if (tccNum < 32U)
    {
        draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId));
        /* Enable the DMA channel in the DRAE registers */
        draeValue &= ~((uint32_t) 0x01 << tccNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId), draeValue);
    }
    else
    {
        draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId));
        /* Enable the DMA channel in the DRAEH registers */
        draeValue &= ~((uint32_t) 0x01 << (tccNum - 32U));
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId), draeValue);
    }
}

void EDMA_channelToParamMap(uint32_t baseAddr,
                            uint32_t channel,
                            uint32_t paramSet)

{
    if( SOC_EDMA_CHMAPEXIST != 0U)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_DCHMAPN(channel), paramSet << 5U);
    }
}

void EDMA_mapChToEvtQ(uint32_t baseAddr,
                      uint32_t chType,
                      uint32_t chNum,
                      uint32_t evtQNum)
{
    uint32_t qnumValue;
    if (EDMA_CHANNEL_TYPE_DMA == chType)
    {
        /* Associate DMA Channel to Event Queue                             */
        qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_DMAQNUMN(chNum >> 3U));
        qnumValue &= EDMACC_DMAQNUM_CLR(chNum);
        qnumValue |= EDMACC_DMAQNUM_SET(chNum, evtQNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_DMAQNUMN(chNum >> 3U), qnumValue);
    }
    else if (EDMA_CHANNEL_TYPE_QDMA == chType)
    {
        /* Associate QDMA Channel to Event Queue                            */
        qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QDMAQNUM);
        qnumValue &= EDMACC_QDMAQNUM_CLR(chNum);
        qnumValue |= EDMACC_QDMAQNUM_SET(chNum, evtQNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_QDMAQNUM, qnumValue);
    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

void EDMA_unmapChToEvtQ(uint32_t baseAddr,
                        uint32_t chType,
                        uint32_t chNum)
{
    uint32_t qnumValue;
    if (EDMA_CHANNEL_TYPE_DMA == chType)
    {
        /* Unmap DMA Channel to Event Queue                                */
        qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_DMAQNUMN(chNum >> 3U));
        qnumValue &= EDMACC_DMAQNUM_CLR(chNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_DMAQNUMN(chNum >> 3U), qnumValue);
    }
    else if (EDMA_CHANNEL_TYPE_QDMA == chType)
    {
        /* Unmap QDMA Channel to Event Queue                               */
        qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QDMAQNUM);
        qnumValue &= EDMACC_QDMAQNUM_CLR(chNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_QDMAQNUM, qnumValue);
    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

void EDMA_mapQdmaChToPaRAM(uint32_t        baseAddr,
                           uint32_t        chNum,
                           const uint32_t *paRAMId)
{
    uint32_t qchmapValue;
    /* Map Parameter RAM Set Number for specified channelId             */
    qchmapValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QCHMAPN(chNum));
    qchmapValue &= EDMACC_QCHMAP_PAENTRY_CLR;
    qchmapValue |= (uint32_t) EDMACC_QCHMAP_PAENTRY_SET(*paRAMId);
    HW_WR_REG32(baseAddr + EDMA_TPCC_QCHMAPN(chNum), qchmapValue);
}

uint32_t EDMA_getMappedPaRAM(uint32_t baseAddr,
                             uint32_t chNum,
                             uint32_t chType,
                             uint32_t *paramId)
{
    uint32_t retVal = FALSE;
    if ((EDMA_CHANNEL_TYPE_DMA == chType) &&
        (chNum <= SOC_EDMA_NUM_DMACH))
    {

        /* Bug Fix - Changed to == */
        if( SOC_EDMA_CHMAPEXIST == 0U)
        {
            *paramId = chNum;
        }
        else
        {
            *paramId = HW_RD_FIELD32(baseAddr + EDMA_TPCC_DCHMAPN(chNum),
                                    EDMA_TPCC_DCHMAPN_PAENTRY);
        }
        retVal = TRUE;
    }
    else if ((EDMA_CHANNEL_TYPE_QDMA == chType) &&
             (chNum <= SOC_EDMA_NUM_QDMACH))
    {
        *paramId = HW_RD_FIELD32(baseAddr + EDMA_TPCC_QCHMAPN(chNum),
                                EDMA_TPCC_QCHMAPN_PAENTRY);
        retVal = TRUE;
    }
    else
    {
        /*An error will be generated automatically.*/
    }
    return retVal;
}

void EDMA_setQdmaTrigWord(uint32_t baseAddr,
                          uint32_t chNum,
                          uint32_t trigWord)
{
    uint32_t qchmapValue;
    qchmapValue = HW_RD_REG32(baseAddr + EDMA_TPCC_QCHMAPN(chNum));
    /* Clear QDMA Trigger word value */
    qchmapValue &= EDMACC_QCHMAP_TRWORD_CLR;
    /* Set the Trigger Word */
    qchmapValue |= EDMACC_QCHMAP_TRWORD_SET(trigWord);
    HW_WR_REG32(baseAddr + EDMA_TPCC_QCHMAPN(chNum), qchmapValue);
}

void EDMA_clrMissEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /*clear SECR to clean any previous NULL request */
        HW_WR_REG32(baseAddr + EDMA_TPCC_SECR_RN(
                        regionId), (uint32_t) 0x01 << chNum);

        /*clear EMCR to clean any previous NULL request */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EMCR, (uint32_t) 0x01 << chNum);
    }
    else
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_SECRH_RN(regionId),
                    (uint32_t) 0x01 << (chNum - 32U));
        /*clear EMCRH to clean any previous NULL request */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EMCRH, (uint32_t) 0x01 << (chNum - 32U));
    }
}

void EDMA_qdmaClrMissEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    /*clear SECR to clean any previous NULL request  */
    HW_WR_REG32(baseAddr + EDMA_TPCC_QSECR_RN(
                    regionId), (uint32_t) 0x01 << chNum);

    /*clear EMCR to clean any previous NULL request  */
    HW_WR_REG32(baseAddr + EDMA_TPCC_QEMCR, (uint32_t) 0x01 << chNum);
}

void EDMA_clrCCErr(uint32_t baseAddr, uint32_t flags)
{
    /* (CCERRCLR) - clear channel controller error register */
    HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR, flags);
}

void EDMA_setEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* (ESR) - set corresponding bit to set a event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_ESR_RN(
                        regionId), (uint32_t) 0x01 << chNum);
    }
    else
    {
        /* (ESRH) - set corresponding bit to set a event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_ESRH_RN(regionId),
                    (uint32_t) 0x01 << (chNum - 32U));
    }
}

void EDMA_clrEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* (ECR) - set corresponding bit to clear a event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_ECR_RN(
                        regionId), (uint32_t) 0x01 << chNum);
    }
    else
    {
        /* (ECRH) - set corresponding bit to clear a event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_ECRH_RN(regionId),
                    (uint32_t) 0x01 << (chNum - 32U));
    }
}

void EDMA_enableDmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* (EESR) - set corresponding bit to enable DMA event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EESR_RN(
                        regionId), (uint32_t) 0x01 << chNum);
    }
    else
    {
        /* (EESRH) - set corresponding bit to enable DMA event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EESRH_RN(regionId),
                    (uint32_t) 0x01 << (chNum - 32U));
    }
}

void EDMA_disableDmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* (EECR) - set corresponding bit to disable event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EECR_RN(
                        regionId), (uint32_t) 0x01 << chNum);
    }
    else
    {
        /* (EECRH) - set corresponding bit to disable event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EECRH_RN(
                        regionId), (uint32_t) 0x01 << (chNum - 32U));
    }
}

void EDMA_enableQdmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    /* (QEESR) - set corresponding bit to enable QDMA event */
    HW_WR_REG32(baseAddr + EDMA_TPCC_QEESR_RN(
                    regionId), (uint32_t) 0x01 << chNum);
}

void EDMA_disableQdmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    /* (QEESR) - set corresponding bit to disable QDMA event */
    HW_WR_REG32(baseAddr + EDMA_TPCC_QEECR_RN(
                    regionId), (uint32_t) 0x01 << chNum);
}

uint32_t EDMA_getCCErrStatus(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_CCERR);

    return intrStatusVal;
}

uint32_t EDMA_getIntrStatusRegion(uint32_t baseAddr, uint32_t regionId)
{
    uint32_t intrStatusVal = 0;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_IPR_RN(regionId));

    return intrStatusVal;
}

uint32_t EDMA_intrStatusHighGetRegion(uint32_t baseAddr, uint32_t regionId)
{
    uint32_t intrStatusVal = 0;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_IPRH_RN(regionId));

    return intrStatusVal;
}

uint32_t EDMA_readIntrStatusRegion(uint32_t baseAddr, uint32_t regionId, uint32_t tccNum)
{
    uint32_t intrStatus = 0;

    if(tccNum < 32U)
    {
        if ((EDMA_getIntrStatusRegion(baseAddr, regionId) & ((uint32_t)0x1 << tccNum)) ==
            ((uint32_t)0x1 << tccNum))
        {
            intrStatus = 1;
        }
    }
    else
    {
        if ((EDMA_intrStatusHighGetRegion(baseAddr, regionId) &
               ((uint32_t)0x1 << (tccNum - 32U))) ==
               ((uint32_t)0x1 << (tccNum - 32U)))
        {
            intrStatus = 1;
        }
    }
    return intrStatus;
}

uint32_t EDMA_getEventStatus(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_ER);

    return intrStatusVal;
}

uint32_t EDMA_getEventStatusHigh(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_ERH);

    return intrStatusVal;
}

uint32_t EDMA_readEventStatusRegion(uint32_t baseAddr, uint32_t chNum)
{
    uint32_t eventStatus = 0;

    if(chNum < 32U)
    {
        if ((EDMA_getEventStatus(baseAddr) & ((uint32_t)0x1 << chNum)) ==
            ((uint32_t)0x1 << chNum))
        {
            eventStatus = 1;
        }
    }
    else
    {
        if ((EDMA_getEventStatusHigh(baseAddr) & ((uint32_t)0x1 << (chNum - 32U))) ==
            ((uint32_t)0x1 << (chNum - 32U)))
        {
            eventStatus = 1;
        }
    }
    return eventStatus;
}

uint32_t EDMA_getErrIntrStatus(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_EMR);

    return intrStatusVal;
}

uint32_t EDMA_errIntrHighStatusGet(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_EMRH);

    return intrStatusVal;
}

uint32_t EDMA_qdmaGetErrIntrStatus(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0;
    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_QEMR);

    return intrStatusVal;
}

void EDMA_enableEvtIntrRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /*  Interrupt Enable Set Register (IESR) */
        HW_WR_REG32(baseAddr + EDMA_TPCC_IESR_RN(
                        regionId), (uint32_t) 0x01 << chNum);
    }
    else
    {
        /*  Interrupt Enable Set Register (IESRH) */
        HW_WR_REG32(baseAddr + EDMA_TPCC_IESRH_RN(regionId),
                    (uint32_t) 0x01 << (chNum - 32U));
    }
}

void EDMA_disableEvtIntrRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* Interrupt Enable Clear Register (IECR) */
        HW_WR_REG32(baseAddr + EDMA_TPCC_IECR_RN(
                        regionId), (uint32_t) 0x01 << chNum);
    }
    else
    {
        /* Interrupt Enable Clear Register (IECRH) */
        HW_WR_REG32(baseAddr + EDMA_TPCC_IECRH_RN(regionId),
                    (uint32_t) 0x01 << (chNum - 32U));
    }
}

void EDMA_clrIntrRegion(uint32_t baseAddr, uint32_t regionId, uint32_t value)
{
    if (value < 32U)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_ICR_RN(
                        regionId), (uint32_t) 1 << value);
    }
    else
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_ICRH_RN(regionId), (uint32_t) 1 <<
                    (value - 32U));
    }
}

uint32_t EDMA_getEnabledIntrRegion(uint32_t baseAddr, uint32_t regionId)
{
    uint32_t intrEnableVal = 0;

    intrEnableVal = HW_RD_REG32(baseAddr + EDMA_TPCC_IER_RN(regionId));

    return intrEnableVal;
}

uint32_t EDMA_getEnabledIntrHighRegion(uint32_t baseAddr, uint32_t regionId)
{
    uint32_t intrEnableVal = 0;

    intrEnableVal = HW_RD_REG32(baseAddr + EDMA_TPCC_IERH_RN(regionId));

    return intrEnableVal;
}

void EDMA_getPaRAM(uint32_t           baseAddr,
                   uint32_t           paRAMId,
                   EDMACCPaRAMEntry *currPaRAM)
{
    uint32_t  i = 0;
    uint32_t sr;
    uint32_t *ds = (uint32_t *) currPaRAM;

    sr = baseAddr + EDMA_TPCC_OPT(paRAMId);

    for (i = 0; i < EDMACC_PARAM_ENTRY_FIELDS; i++)
    {
        *ds = HW_RD_REG32(sr);
        ds++;
        sr+= (uint32_t)sizeof(uint32_t);
    }
}

void EDMA_qdmaGetPaRAM(uint32_t           baseAddr,
                       uint32_t           paRAMId,
                       EDMACCPaRAMEntry *currPaRAM)
{
    uint32_t  i = 0;
    uint32_t *ds     = (uint32_t *) currPaRAM;
    uint32_t  sr     = baseAddr + EDMA_TPCC_OPT(paRAMId);

    for (i = 0; i < EDMACC_PARAM_ENTRY_FIELDS; i++)
    {
        *ds = HW_RD_REG32(sr);
        ds++;
        sr+= (uint32_t)sizeof(uint32_t);
    }
}

void EDMA_setPaRAM(uint32_t           baseAddr,
                   uint32_t           paRAMId,
                   const EDMACCPaRAMEntry *newPaRAM)
{
    uint32_t           i  = 0;
    uint32_t          *sr = (uint32_t *) newPaRAM;
    volatile uint32_t  ds;
    uint32_t           dsAddr =baseAddr + EDMA_TPCC_OPT(paRAMId);

    ds = (uint32_t ) (dsAddr);

    for (i = 0; i < EDMACC_PARAM_ENTRY_FIELDS; i++)
    {
        HW_WR_REG32(ds, *sr);
        ds+= (uint32_t)sizeof(uint32_t);
        sr++;
    }
}

void EDMA_qdmaSetPaRAM(uint32_t           baseAddr,
                       uint32_t           paRAMId,
                       const EDMACCPaRAMEntry *newPaRAM)
{
    uint32_t  i  = 0;
    uint32_t *sr = (uint32_t *) newPaRAM;
    uint32_t  ds;
    uint32_t  dsAddr =baseAddr + EDMA_TPCC_OPT(paRAMId);

    ds = (uint32_t ) (dsAddr);

    for (i = 0; i < EDMACC_PARAM_ENTRY_FIELDS; i++)
    {
        HW_WR_REG32(ds, *sr);
        ds+= (uint32_t)sizeof(uint32_t);
        sr++;
    }
}

void EDMA_qdmaSetPaRAMEntry(uint32_t baseAddr,
                            uint32_t paRAMId,
                            uint32_t paRAMEntry,
                            uint32_t newPaRAMEntryVal)
{
    EDMA_dmaSetPaRAMEntry(baseAddr, paRAMId,
                          paRAMEntry, newPaRAMEntryVal);
}

uint32_t EDMA_qdmaGetPaRAMEntry(uint32_t baseAddr,
                                uint32_t paRAMId,
                                uint32_t paRAMEntry)
{
    return EDMA_dmaGetPaRAMEntry(baseAddr, paRAMId,
                          paRAMEntry);
}

void EDMA_dmaSetPaRAMEntry(uint32_t baseAddr,
                            uint32_t paRAMId,
                            uint32_t paRAMEntry,
                            uint32_t newPaRAMEntryVal)
{
    if (paRAMEntry <= EDMACC_PARAM_ENTRY_CCNT)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_OPT(paRAMId) +
                    (paRAMEntry * 0x04U), newPaRAMEntryVal);
    }
}

uint32_t EDMA_dmaGetPaRAMEntry(uint32_t baseAddr,
                                uint32_t paRAMId,
                                uint32_t paRAMEntry)
{
    uint32_t paRAMEntryVal = 0;
    if (paRAMEntry <= EDMACC_PARAM_ENTRY_CCNT)
    {
        paRAMEntryVal = HW_RD_REG32(baseAddr + EDMA_TPCC_OPT(paRAMId) +
                                    (paRAMEntry * 0x04U));
    }
    return (paRAMEntryVal);
}

uint32_t EDMA_configureChannelRegion(uint32_t baseAddr,
                                     uint32_t regionId,
                                     uint32_t chType,
                                     uint32_t chNum,
                                     uint32_t tccNum,
                                     uint32_t paramId,
                                     uint32_t evtQNum)
{
    uint32_t optValue;
    uint32_t retVal = FALSE;
    if (((EDMA_CHANNEL_TYPE_DMA == chType) && (chNum < SOC_EDMA_NUM_DMACH)) ||
        ((EDMA_CHANNEL_TYPE_QDMA == chType) && (chNum < SOC_EDMA_NUM_QDMACH)))
    {
        /* Enable the DMA channel and the TCC in the shadow region
         * specific register
         */
        EDMA_enableChInShadowRegRegion(baseAddr, regionId, chType, chNum);
        EDMA_enableTccInShadowRegRegion(baseAddr, regionId, tccNum);

        EDMA_mapChToEvtQ(baseAddr, chType, chNum, evtQNum);

        if (EDMA_CHANNEL_TYPE_DMA == chType)
        {
            EDMA_channelToParamMap(baseAddr, chNum, paramId);
        }
        else
        {
            EDMA_mapQdmaChToPaRAM(baseAddr, chNum, &paramId);
        }

        optValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_OPT(paramId));
        optValue &= EDMACC_OPT_TCC_CLR;
        optValue |= EDMACC_OPT_TCC_SET(tccNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_OPT(paramId), optValue);

        retVal = (uint32_t) TRUE;
    }
    return retVal;
}

uint32_t EDMA_freeChannelRegion(uint32_t baseAddr,
                                uint32_t regionId,
                                uint32_t chType,
                                uint32_t chNum,
                                uint32_t trigMode,
                                uint32_t tccNum,
                                uint32_t evtQNum)
{
    uint32_t    cnt;
    uint32_t    retVal = FALSE;
    uint32_t    chCheckNum = tccNum;
    uint32_t    tccCheckNum = chNum;
    EDMA_Handle handle = NULL;

    if ((chNum < SOC_EDMA_NUM_DMACH) &&
        ((EDMA_CHANNEL_TYPE_DMA == chType) || (EDMA_CHANNEL_TYPE_QDMA == chType)))
    {
        (void) EDMA_disableTransferRegion(baseAddr, regionId, chNum, trigMode);

        /*
         * Loop throught the list of EDMA handles to find the one with current
         * base address.
         */
        for (cnt = 0U; cnt < gEdmaConfigNum; cnt++)
        {
            if((gEdmaConfig[cnt].attrs->baseAddr) == baseAddr)
            {
                handle = (EDMA_Handle)&gEdmaConfig[cnt];
                break;
            }
        }
        if(NULL != handle)
        {
            /*
             * Disable the DMA channel in the shadow region specific register if a TCC
             * with the same number is not already allocated.
             */
            if(EDMA_isTccAllocated(handle, &tccCheckNum) == FALSE)
            {
                EDMA_disableChInShadowRegRegion(baseAddr, regionId, chType, chNum);
            }
            /*
             * Disable the TCC in the shadow region specific register if a DMA channel
             * with the same number is not already allocated.
             */
            if(EDMA_isDmaChannelAllocated(handle, &chCheckNum) == FALSE)
            {
                EDMA_disableTccInShadowRegRegion(baseAddr, regionId, tccNum);
            }

            EDMA_unmapChToEvtQ(baseAddr, chType, chNum);
        }
        retVal = (uint32_t) TRUE;
    }
    return retVal;
}

uint32_t EDMA_enableTransferRegion(uint32_t baseAddr,
                                   uint32_t regionId,
                                   uint32_t chNum,
                                   uint32_t trigMode)
{
    uint32_t retVal = FALSE;
    switch (trigMode)
    {
        case EDMA_TRIG_MODE_MANUAL:
            if (chNum < SOC_EDMA_NUM_DMACH)
            {
                EDMA_setEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        case EDMA_TRIG_MODE_QDMA:
            if (chNum < SOC_EDMA_NUM_QDMACH)
            {
                EDMA_enableQdmaEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        case EDMA_TRIG_MODE_EVENT:
            if (chNum < SOC_EDMA_NUM_DMACH)
            {
                /*clear SECR & EMCR to clean any previous NULL request */
                EDMA_clrMissEvtRegion(baseAddr, regionId, chNum);

                /* Set EESR to enable event */
                EDMA_enableDmaEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        default:
            retVal = (uint32_t) FALSE;
            break;
    }
    return retVal;
}

uint32_t EDMA_disableTransferRegion(uint32_t baseAddr,
                                    uint32_t regionId,
                                    uint32_t chNum,
                                    uint32_t trigMode)
{
    uint32_t retVal = FALSE;
    switch (trigMode)
    {
        case EDMA_TRIG_MODE_MANUAL:
            if (chNum < SOC_EDMA_NUM_DMACH)
            {
                EDMA_clrEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        case EDMA_TRIG_MODE_QDMA:
            if (chNum < SOC_EDMA_NUM_QDMACH)
            {
                EDMA_disableQdmaEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        case EDMA_TRIG_MODE_EVENT:
            if (chNum < SOC_EDMA_NUM_DMACH)
            {
                /*clear SECR & EMCR to clean any previous NULL request */
                EDMA_clrMissEvtRegion(baseAddr, regionId, chNum);

                /* Set EESR to enable event */
                EDMA_disableDmaEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        default:
            retVal = (uint32_t) FALSE;
            break;
    }
    return retVal;
}

void EDMA_clearErrorBitsRegion(uint32_t baseAddr,
                               uint32_t regionId,
                               uint32_t chNum,
                               uint32_t evtQNum)
{
    if (chNum < SOC_EDMA_NUM_DMACH)
    {
        if (chNum < 32U)
        {
            HW_WR_REG32(baseAddr + EDMA_TPCC_EECR_RN(
                            regionId),(uint32_t) 0x01 << chNum);
            /* Write to EMCR to clear the corresponding EMR bit */
            HW_WR_REG32(baseAddr + EDMA_TPCC_EMCR, (uint32_t) 0x01 << chNum);
            /* Clears the SER */
            HW_WR_REG32(baseAddr + EDMA_TPCC_SECR_RN(
                            regionId), (uint32_t) 0x01 << chNum);
        }
        else
        {
            HW_WR_REG32(baseAddr +
                        EDMA_TPCC_EECRH_RN(regionId), (uint32_t) 0x01 <<
                        (chNum - 32U));
            /* Write to EMCR to clear the corresponding EMR bit */
            HW_WR_REG32(baseAddr + EDMA_TPCC_EMCRH, (uint32_t) 0x01 <<
                        (chNum - 32U));
            /* Clears the SER */
            HW_WR_REG32(baseAddr +
                        EDMA_TPCC_SECRH_RN(regionId), (uint32_t) 0x01 <<
                        (chNum - 32U));
        }
    }

    /* Clear the global CC Error Register */
    if (0U == evtQNum)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR,
                    EDMA_TPCC_CCERRCLR_QTHRXCD0_MASK |
                    EDMA_TPCC_CCERRCLR_TCERR_MASK);
    }
    else if (1U == evtQNum)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR,
                    EDMA_TPCC_CCERRCLR_QTHRXCD1_MASK |
                    EDMA_TPCC_CCERRCLR_TCERR_MASK);
    }
#if SOC_EDMA_NUM_EVQUE > 2
    else if (2U == evtQNum)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR,
                    EDMA_TPCC_CCERRCLR_QTHRXCD2_MASK |
                    EDMA_TPCC_CCERRCLR_TCERR_MASK);
    }
#if SOC_EDMA_NUM_EVQUE > 3
    else if (3U == evtQNum)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR,
                    EDMA_TPCC_CCERRCLR_QTHRXCD3_MASK |
                    EDMA_TPCC_CCERRCLR_TCERR_MASK);
    }
#endif
#endif
    else
    {
        /*Error will be generated automatically*/
    }
}

static int32_t EDMA_deinitialize (uint32_t baseAddr, const EDMA_InitParams *initParam)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t count = 0;
    uint32_t qnumValue;

    /* Disable the DMA (0 - 62) channels in the DRAE register */
    HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(
                    initParam->regionId), EDMA_CLR_ALL_BITS);
    HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(
                    initParam->regionId), EDMA_CLR_ALL_BITS);

    EDMA_clrCCErr(baseAddr, EDMACC_CLR_TCCERR);

    /* Clear the Event miss Registers */
    HW_WR_REG32(baseAddr + EDMA_TPCC_EMCR, initParam->ownResource.dmaCh[0]);
#if SOC_EDMA_NUM_DMACH > 32
    HW_WR_REG32(baseAddr + EDMA_TPCC_EMCRH, initParam->ownResource.dmaCh[1]);
#endif
    /* Clear CCERR register */
    HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR, initParam->ownResource.qdmaCh);

    /* Disable and clear channel interrupts for all dma channels */
    for (count = 0; count < EDMA_NUM_TCC; count++)
    {
        if (((1U << (count%32U)) & initParam->ownResource.tcc[count/32U]) != 0U)
        {
            EDMA_disableEvtIntrRegion(baseAddr, initParam->regionId, count);
            EDMA_clrIntrRegion(baseAddr, initParam->regionId, count);
        }
    }
    /* Disable and clear channel interrupts for all qdma channels */
    for (count = 0; count < SOC_EDMA_NUM_QDMACH; count++)
    {
        if (((1U << count) & initParam->ownResource.qdmaCh) != 0U)
        {
            EDMA_disableQdmaEvtRegion(baseAddr, initParam->regionId, count);
            EDMA_qdmaClrMissEvtRegion(baseAddr, initParam->regionId, count);
        }
    }

    /* Deinitialize the Queue Number Registers */
    for (count = 0; count < SOC_EDMA_NUM_DMACH; count++)
    {
        if (((1U << (count%32U)) & initParam->ownResource.dmaCh[count/32U]) != 0U)
        {
            qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_DMAQNUMN((count >> 3U)));
            qnumValue &= EDMACC_DMAQNUM_CLR(count);
            HW_WR_REG32(baseAddr + EDMA_TPCC_DMAQNUMN((count >> 3U)), qnumValue);
        }
    }

    for (count = 0; count < SOC_EDMA_NUM_QDMACH; count++)
    {
        if (((1U << count) & initParam->ownResource.qdmaCh) != 0U)
        {
            qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QDMAQNUM);
            qnumValue &= EDMACC_QDMAQNUM_CLR(count);
            HW_WR_REG32(baseAddr + EDMA_TPCC_QDMAQNUM, qnumValue);
        }
    }
    return retVal;
}

void EDMA_chainChannel(uint32_t baseAddr,
                       uint32_t paRAMId1,
                       uint32_t chId2,
                       uint32_t chainOptions)
{
    EDMACCPaRAMEntry *currPaRAM     = NULL;
    uint32_t           currPaRAMAddr = baseAddr + EDMA_TPCC_OPT(paRAMId1);
    uint32_t           optVal;
    uintptr_t          optAddr;

    /* Get param set for the channel Id passed*/
    currPaRAM = (EDMACCPaRAMEntry *) (currPaRAMAddr);

    optAddr    = (uintptr_t) &currPaRAM->opt;
    optVal = HW_RD_REG32((uint32_t) optAddr);
    optVal &= ~(EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
               EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK);
    optVal |= chainOptions;
    optVal &= ~EDMA_TPCC_OPT_TCC_MASK;
    optVal |= (chId2 << EDMA_TPCC_OPT_TCC_SHIFT) & EDMA_TPCC_OPT_TCC_MASK;
    HW_WR_REG32((uint32_t) optAddr, optVal);
}

void EDMA_linkChannel(uint32_t baseAddr,
                      uint32_t paRAMId1,
                      uint32_t paRAMId2)
{
    EDMACCPaRAMEntry *currPaRAM1;
    EDMACCPaRAMEntry *currPaRAM2;
    uint32_t           optVal1, optVal2;
    uint32_t           currPaRAMAddr1 = baseAddr + EDMA_TPCC_OPT(paRAMId1);
    uint32_t           currPaRAMAddr2 = baseAddr + EDMA_TPCC_OPT(paRAMId2);
    uintptr_t          lnkAddr;

    /* Get param set for the paRAMId1 passed*/
    currPaRAM1 = (EDMACCPaRAMEntry *) (currPaRAMAddr1);

    /* Update the Link field with lch2 PaRAM set */
    lnkAddr = (uintptr_t) &currPaRAM1->linkAddr;
    HW_WR_REG16(lnkAddr,
        (uint16_t) ((baseAddr + EDMA_TPCC_OPT(paRAMId2)) & (uint16_t) 0x0FFFF));

    /* Get param set for the paRAMId2 passed */
    currPaRAM2 = (EDMACCPaRAMEntry *) (currPaRAMAddr2);

    /*Updated TCC value of param2 with that of param1 */
    optVal1 = HW_RD_REG32((uint32_t) &currPaRAM1->opt);
    optVal2 = HW_RD_REG32((uint32_t) &currPaRAM2->opt);
    optVal2 &= ~EDMA_TPCC_OPT_TCC_MASK;
    optVal2 |= optVal1 & EDMA_TPCC_OPT_TCC_MASK;
    HW_WR_REG32((uint32_t) &currPaRAM2->opt, optVal2);
}

void EDMA_init(void)
{
    uint32_t        cnt;
    EDMA_Object    *object;
    uint32_t        baseAddr;

    /* Init each driver instance object */
    for (cnt = 0U; cnt < gEdmaConfigNum; cnt++)
    {
        /* initialize object varibles */
        object = gEdmaConfig[cnt].object;
        DebugP_assert(NULL != object);
        (void) memset(object, 0, sizeof(EDMA_Object));
        /* Get the edma base address. */
        baseAddr = gEdmaConfig[cnt].attrs->baseAddr;
        /* Initialize the EDMA hardware. */
        (void) EDMA_initialize(baseAddr, &gEdmaConfig[cnt].attrs->initPrms);
    }

    return;
}

void EDMA_deinit(void)
{
    uint32_t        cnt;
    EDMA_Object    *object;
    uint32_t        baseAddr;

    /* Init each driver instance object */
    for (cnt = 0U; cnt < gEdmaConfigNum; cnt++)
    {
        /* initialize object varibles */
        object = gEdmaConfig[cnt].object;
        if(NULL != object)
        {
            (void) memset(object, 0, sizeof(EDMA_Object));
            /* Get the edma base address. */
            baseAddr = gEdmaConfig[cnt].attrs->baseAddr;
            /* Initialize the EDMA hardware. */
            (void) EDMA_deinitialize(baseAddr, &gEdmaConfig[cnt].attrs->initPrms);
        }
    }
    return;
}

EDMA_Handle EDMA_open(uint32_t index, const EDMA_Params *prms)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Handle         handle = NULL;
    EDMA_Config        *config = NULL;
    EDMA_Object        *object    = NULL;
    HwiP_Params         hwiPrms;

    /* Check index */
    if(index >= gEdmaConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gEdmaConfig[index];
    }

    if(SystemP_SUCCESS == status)
    {
       if((NULL != config->object) && (NULL != config->attrs))
       {
            object = config->object;
            if(TRUE == object->isOpen)
            {
                /* Handle is already opened */
                status = SystemP_FAILURE;
            }
       }
       else
       {
           status = SystemP_FAILURE;
       }
    }

    if(SystemP_SUCCESS == status)
    {
        /* Init state */
        object->handle = (EDMA_Handle) config;
        if(NULL != prms)
        {
            /* Store the open params in driver object. */
            (void) memcpy(&object->openPrms, prms, sizeof(EDMA_Params));

            if (prms->intrEnable == TRUE)
            {
                /* Register the master ISR. */
                /* Enable the aggregated interrupt. */
                HW_WR_REG32(config->attrs->intrAggEnableAddr, config->attrs->intrAggEnableMask);

                /* Register interrupt */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum   = config->attrs->compIntrNumber;
                hwiPrms.callback = &EDMA_transferCompletionMasterIsrFxn;
                hwiPrms.args     = object->handle;
                hwiPrms.isPulse     = 1;
                status = HwiP_construct(&object->hwiObj, &hwiPrms);
                DebugP_assert(status == SystemP_SUCCESS);
                object->hwiHandle = &object->hwiObj;
            }
            object->firstIntr = NULL;
            object->isOpen = TRUE;
            handle = (EDMA_Handle) config;
        }
    }
    return handle;
}

void EDMA_close(EDMA_Handle handle)
{
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    config = (EDMA_Config *) handle;

    if((NULL != config)&&
       (config->object != NULL)&&
       (config->object->isOpen != (uint32_t)FALSE))
    {
        object = config->object;
        attrs = config->attrs;

        DebugP_assert(NULL != object);
        DebugP_assert(NULL != attrs);

        if(NULL != object->hwiHandle)
        {
            HwiP_destruct(&object->hwiObj);
            object->hwiHandle = NULL;
        }

        object->isOpen = FALSE;
    }
}

EDMA_Handle EDMA_getHandle(uint32_t index)
{
    EDMA_Handle         handle = NULL;
    /* Check index */
    if(index < gEdmaConfigNum)
    {
        EDMA_Object *object;

        object = gEdmaConfig[index].object;
        if((object != NULL) && (TRUE == object->isOpen))
        {
            /* valid handle */
            handle = (EDMA_Handle)&(gEdmaConfig[index]);
        }
    }

    return handle;
}

uint32_t EDMA_isInterruptEnabled(EDMA_Handle handle)
{
    EDMA_Object    *object = ((EDMA_Config *)handle)->object;
    EDMA_Params    *openParams = &(object->openPrms);
    return (openParams->intrEnable);
}

static int32_t EDMA_validateIntrObject(Edma_IntrObject *intrObj)
{
    int32_t             status = SystemP_SUCCESS;
    if (intrObj->cbFxn == NULL)
    {
        /* Callback function cant be NULL for registering interrupt. */
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        /* TODO: validate tccnum */
    }
    return status;
}

int32_t EDMA_registerIntr(EDMA_Handle handle, Edma_IntrObject *intrObj)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    uintptr_t           intrState;

    if ((handle == NULL) || (intrObj == NULL))
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        status = EDMA_validateIntrObject(intrObj);
    }
    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if((config->object != NULL) &&
        (config->object->isOpen != (uint32_t)FALSE) &&
        (config->object->openPrms.intrEnable != FALSE))
        {
            object = config->object;
            attrs = config->attrs;

            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);

            intrState = HwiP_disable();
            if (object->firstIntr == NULL)
            {
                object->firstIntr = intrObj;
                intrObj->nextIntr = NULL;
                intrObj->prevIntr = NULL;
            }
            else
            {
                /* Insert the intrObj at the end of the list. */
                Edma_IntrObject *tempObj;
                tempObj = object->firstIntr;
                while (tempObj->nextIntr != NULL)
                {
                    tempObj = tempObj->nextIntr;
                }
                tempObj->nextIntr = intrObj;
                intrObj->nextIntr = NULL;
                intrObj->prevIntr = tempObj;
            }
            HwiP_restore (intrState);

            /* Enable the tcc interrupt bit. */
            EDMA_enableEvtIntrRegion(attrs->baseAddr, attrs->initPrms.regionId, intrObj->tccNum);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    return status;
}

int32_t EDMA_unregisterIntr(EDMA_Handle handle, Edma_IntrObject *intrObj)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    uintptr_t           intrState;

    if ((handle == NULL) || (intrObj == NULL))
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if((config->object != NULL) &&
        (config->object->isOpen != (uint32_t)FALSE))
        {
            Edma_IntrObject *tempObj;
            uint32_t                  elementFound = (uint32_t)FALSE;
            object = config->object;
            attrs = config->attrs;

            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);

            intrState = HwiP_disable();
            /* Find the intrObj in the list. */
            if (object->firstIntr == intrObj)
            {
                tempObj = object->firstIntr;
                object->firstIntr = tempObj->nextIntr;
                elementFound = TRUE;
            }
            else
            {
                tempObj = object->firstIntr;
                while ((tempObj != NULL) && (tempObj != intrObj))
                {
                    tempObj = tempObj->nextIntr;
                }
                if (tempObj == intrObj)
                {
                    Edma_IntrObject *prevObj;
                    /* Element is found. Remove current object from list. */
                    prevObj = tempObj->prevIntr;
                    prevObj->nextIntr = tempObj->nextIntr;
                    elementFound = TRUE;
                }
            }
            HwiP_restore (intrState);

            if (elementFound == TRUE)
            {
                intrObj->prevIntr = NULL;
                intrObj->nextIntr = NULL;
                /* Disable the tcc interrupt bit. */
                EDMA_disableEvtIntrRegion(attrs->baseAddr, attrs->initPrms.regionId, intrObj->tccNum);
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }
    return status;
}

uint32_t EDMA_getBaseAddr(EDMA_Handle handle)
{
    EDMA_Config        *config;
    const EDMA_Attrs   *attrs;
    uint32_t            baseAddr = 0;

    if (handle != NULL)
    {
        config = (EDMA_Config *) handle;

        if((config->object != NULL) &&
           (config->object->isOpen != FALSE))
        {
            attrs = config->attrs;
            DebugP_assert(NULL != attrs);
            baseAddr = attrs->baseAddr;
        }
    }
    return baseAddr;
}

uint32_t EDMA_getRegionId(EDMA_Handle handle)
{
    EDMA_Config        *config;
    uint32_t            regionId = SOC_EDMA_NUM_REGIONS;

    if (handle != NULL)
    {
        config = (EDMA_Config *) handle;

        if((config->object != NULL) &&
           (config->object->isOpen != (uint32_t)FALSE))
        {
            regionId = config->attrs->initPrms.regionId;
        }
    }
    return regionId;
}

int32_t EDMA_allocDmaChannel(EDMA_Handle handle, uint32_t *dmaCh)
{
    int32_t            status = SystemP_FAILURE;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;

    if ((handle != NULL) && (dmaCh != NULL))
    {
        config = (EDMA_Config *) handle;

        if(config->object != NULL)
        {
            if(config->object->isOpen != (uint32_t)FALSE)
            {
                object = config->object;
                attrs = config->attrs;
                DebugP_assert(NULL != attrs);
                status = Alloc_resource(attrs, object, dmaCh, EDMA_RESOURCE_TYPE_DMA);
            }
        }
    }
    return status;
}

int32_t EDMA_allocQdmaChannel(EDMA_Handle handle, uint32_t *qdmaCh)
{
    int32_t            status = SystemP_FAILURE;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;

    if ((handle != NULL) && (qdmaCh != NULL))
    {
        config = (EDMA_Config *) handle;

        if(config->object != NULL)
        {
            if(config->object->isOpen != (uint32_t)FALSE)
            {
                object = config->object;
                attrs = config->attrs;
                DebugP_assert(NULL != attrs);
                status = Alloc_resource(attrs, object, qdmaCh, EDMA_RESOURCE_TYPE_QDMA);
            }
        }
    }
    return status;
}

int32_t EDMA_allocTcc(EDMA_Handle handle, uint32_t *tcc)
{
    int32_t            status = SystemP_FAILURE;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;

    if ((handle != NULL) && (tcc != NULL))
    {
        config = (EDMA_Config *) handle;

        if(config->object != NULL)
        {
            if(config->object->isOpen != (uint32_t)FALSE)
            {
                object = config->object;
                attrs = config->attrs;
                DebugP_assert(NULL != attrs);
                status = Alloc_resource(attrs, object, tcc, EDMA_RESOURCE_TYPE_TCC);
            }
        }
    }
    return status;
}

int32_t EDMA_allocParam(EDMA_Handle handle, uint32_t *param)
{
    int32_t             status = SystemP_FAILURE;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;

    if ((handle != NULL) && (param != NULL))
    {
        config = (EDMA_Config *) handle;

        if(config->object != NULL)
        {
            if(config->object->isOpen != (uint32_t)FALSE)
            {
                object = config->object;
                attrs = config->attrs;
                DebugP_assert(NULL != attrs);
                status = Alloc_resource(attrs, object, param, EDMA_RESOURCE_TYPE_PARAM);
            }
        }
    }
    return status;
}

static int32_t Alloc_resource(const EDMA_Attrs *attrs, EDMA_Object *object, uint32_t *resId, uint32_t resType)
{
    uint32_t    i,j;
    uintptr_t   intrState;
    int32_t     status = SystemP_SUCCESS;
    uint32_t    *allocPtr;
    const uint32_t *ownPtr, *reservedPtr;
    uint32_t    resPtrLen, maxRes;
    uint32_t resAllocated = 0U;
    switch (resType)
    {
        case EDMA_RESOURCE_TYPE_DMA:
            allocPtr = &object->allocResource.dmaCh[0];
            ownPtr = &attrs->initPrms.ownResource.dmaCh[0];
            reservedPtr = &attrs->initPrms.reservedDmaCh[0];
            resPtrLen = SOC_EDMA_NUM_DMACH/32U;
            maxRes = SOC_EDMA_NUM_DMACH;
            break;
        case EDMA_RESOURCE_TYPE_QDMA:
            allocPtr = &object->allocResource.qdmaCh;
            ownPtr = &attrs->initPrms.ownResource.qdmaCh;
            resPtrLen = 1;
            maxRes = SOC_EDMA_NUM_QDMACH;
            break;
        case EDMA_RESOURCE_TYPE_TCC:
            allocPtr = &object->allocResource.tcc[0];
            ownPtr = &attrs->initPrms.ownResource.tcc[0];
            resPtrLen = SOC_EDMA_NUM_DMACH/32U;
            maxRes = SOC_EDMA_NUM_DMACH;
            break;
        case EDMA_RESOURCE_TYPE_PARAM:
            allocPtr = &object->allocResource.paramSet[0];
            ownPtr = &attrs->initPrms.ownResource.paramSet[0];
            resPtrLen = SOC_EDMA_NUM_PARAMSETS/32U;
            maxRes = SOC_EDMA_NUM_PARAMSETS;
            break;
        default:
            status = SystemP_FAILURE;
            break;
    }

    if (status == SystemP_SUCCESS)
    {
        /* set the status to failure.
           If allocation is successful status will be updated. */
        status = SystemP_FAILURE;
        intrState = HwiP_disable();
        if (*resId == EDMA_RESOURCE_ALLOC_ANY)
        {
            /* Find available resource. */
            for (i=0; i < resPtrLen ; i++)
            {
                for (j=0; ((j<32U)&&(resAllocated==0U)); j++)
                {
                    if (resType == EDMA_RESOURCE_TYPE_DMA)
                    {
                        /* Check if the dma channel is owned and available and not reserved. */
                        if (((ownPtr[i] & (1U << j)) != 0U) &&
                            ((reservedPtr[i] & (1U << j)) == 0U) &&
                            ((allocPtr[i] & (1U << j)) == 0U))
                        {
                            *resId = (i * 32U) + j;
                            allocPtr[i] |= (uint32_t)1 << (j%32U);
                            status = SystemP_SUCCESS;
                            resAllocated = 1u;
                        }
                    }
                    else
                    {
                        /* Check if the resource is owned and available. */
                        if (((ownPtr[i] & (1U << j)) != 0U) &&
                            ((allocPtr[i] & (1U << j)) == 0U))
                        {
                            *resId = (i * 32U) + j;
                            allocPtr[i] |= (uint32_t)1 << (j%32U);
                            status = SystemP_SUCCESS;
                            resAllocated = 1u;
                        }
                    }
                }
                if (*resId != EDMA_RESOURCE_ALLOC_ANY)
                {
                    break;
                }
            }
        }
        else
        {
            /* Check if the resource is already allocated. */
            if ((*resId < maxRes) &&
                ((ownPtr[*resId/32U] & (1U << (*resId%32U))) != 0U) &&
                ((allocPtr[*resId/32U] & (1U << (*resId%32U))) == 0U))
            {
                allocPtr[*resId/32U] |= (uint32_t)1 << (*resId%32U);
                status = SystemP_SUCCESS;
            }
        }
        HwiP_restore(intrState);
    }
    return status;
}

static uint32_t EDMA_isDmaChannelAllocated(EDMA_Handle handle, const uint32_t *dmaCh)
{
    EDMA_Config         *config;
    EDMA_Object         *object;
    const EDMA_Attrs    *attrs;
    uint32_t            *allocPtr;
    const uint32_t      *ownPtr;
    uint32_t            maxRes;
    uint32_t            isAllocated;

    DebugP_assert(NULL != handle);
    DebugP_assert(NULL != dmaCh);

    config = (EDMA_Config *) handle;
    maxRes = 0;

    object = config->object;
    attrs = config->attrs;

    DebugP_assert(NULL != object);
    DebugP_assert(NULL != attrs);
    DebugP_assert(object->isOpen != (uint32_t)FALSE);

    allocPtr = &object->allocResource.dmaCh[0];
    ownPtr = &attrs->initPrms.ownResource.dmaCh[0];
    maxRes = SOC_EDMA_NUM_DMACH;

    isAllocated = TRUE;
    /* Check if the resource is already allocated. */
    if ((*dmaCh < maxRes) &&
        ((ownPtr[*dmaCh/32U] & ((uint32_t)1 << (*dmaCh%32U))) != 0U) &&
        ((allocPtr[*dmaCh/32U] & ((uint32_t)1 << (*dmaCh%32U))) == 0U))
    {
        isAllocated = FALSE;
    }

    return isAllocated;
}

static uint32_t EDMA_isTccAllocated(EDMA_Handle handle, const uint32_t *tcc)
{
    EDMA_Config         *config;
    EDMA_Object         *object;
    const EDMA_Attrs    *attrs;
    uint32_t            *allocPtr;
    const uint32_t      *ownPtr;
    uint32_t            maxRes;
    uint32_t            isAllocated;

    DebugP_assert(NULL != handle);
    DebugP_assert(NULL != tcc);

    config = (EDMA_Config *) handle;
    maxRes = 0;

    object = config->object;
    attrs = config->attrs;

    DebugP_assert(NULL != object);
    DebugP_assert(NULL != attrs);

    allocPtr = &object->allocResource.tcc[0];
    ownPtr = &attrs->initPrms.ownResource.tcc[0];
    maxRes = SOC_EDMA_NUM_DMACH;

    isAllocated = TRUE;
    /* Check if the resource is already allocated. */
    if ((*tcc < maxRes) &&
        ((ownPtr[*tcc/32U] & ((uint32_t)1 << (*tcc%32U))) != 0U) &&
        ((allocPtr[*tcc/32U] & ((uint32_t)1 << (*tcc%32U))) == 0U))
    {
        isAllocated = FALSE;
    }

    return isAllocated;
}

int32_t EDMA_freeDmaChannel(EDMA_Handle handle, uint32_t *dmaCh)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    uintptr_t           intrState = 0;

    if ((handle == NULL) || (dmaCh == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        if(*dmaCh >= SOC_EDMA_NUM_DMACH )
            {
                status = SystemP_FAILURE;
            }
    }

    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if(config->object->isOpen != (uint32_t)FALSE)
        {
            object = config->object;
            attrs = config->attrs;
            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);
            intrState = HwiP_disable();
            object->allocResource.dmaCh[*dmaCh/32U] &= ~(1U << (*dmaCh%32U));
            HwiP_restore(intrState);
        }
    }
    return status;
}

int32_t EDMA_freeQdmaChannel(EDMA_Handle handle, uint32_t *qdmaCh)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    uintptr_t           intrState = 0;

    if ((handle == NULL) || (qdmaCh == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        if(*qdmaCh >= SOC_EDMA_NUM_QDMACH)
        {
            status = SystemP_FAILURE;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if(config->object->isOpen != (uint32_t)FALSE)
        {
            object = config->object;
            attrs = config->attrs;
            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);
            intrState = HwiP_disable();
            object->allocResource.qdmaCh &= ~(1U << (*qdmaCh%32U));
            HwiP_restore(intrState);
        }
    }
    return status;
}

int32_t EDMA_freeTcc(EDMA_Handle handle, uint32_t *tcc)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    uintptr_t           intrState = 0;

    if ((handle == NULL) || (tcc == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        if (*tcc >= SOC_EDMA_NUM_DMACH )
            {
                status = SystemP_FAILURE;
            }
    }

    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if(config->object->isOpen != (uint32_t)FALSE)
        {
            object = config->object;
            attrs = config->attrs;
            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);
            intrState = HwiP_disable();
            object->allocResource.tcc[*tcc/32U] &= ~(1U << (*tcc%32U));
            HwiP_restore(intrState);
        }
    }
    return status;
}

int32_t EDMA_freeParam(EDMA_Handle handle, uint32_t *param)
{
   int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    uintptr_t           intrState = 0;

    if ((handle == NULL) || (param == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        if(*param >= SOC_EDMA_NUM_PARAMSETS)
            {
                status = SystemP_FAILURE;
            }
    }

    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if(config->object->isOpen != (uint32_t)FALSE)
        {
            object = config->object;
            attrs = config->attrs;
            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);
            intrState = HwiP_disable();
            object->allocResource.paramSet[*param/32U] &= ~(1U << (*param%32U));
            HwiP_restore(intrState);
        }
    }
    return status;
}

static void EDMA_transferCompletionMasterIsrFxn(void *args)
{

    EDMA_Handle         handle = (EDMA_Handle) args;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    Edma_IntrObject *intrObj;
    uint32_t baseAddr, regionId;
    uint32_t intrLow, intrHigh;

    DebugP_assert(NULL != handle);
    config = (EDMA_Config *) handle;

    object = config->object;
    attrs = config->attrs;
    DebugP_assert(NULL != object);
    DebugP_assert(NULL != attrs);

    baseAddr = attrs->baseAddr;
    regionId = attrs->initPrms.regionId;

    intrLow = EDMA_getIntrStatusRegion(baseAddr, regionId);
    intrHigh = EDMA_intrStatusHighGetRegion(baseAddr, regionId);

    intrObj = object->firstIntr;

    while ((intrObj != NULL) && ((intrLow != 0U) || (intrHigh != 0U)))
    {
        if ((intrObj->tccNum < 32U) && ((intrLow & (1U << intrObj->tccNum)) != 0U))
        {
            EDMA_clrIntrRegion(baseAddr, regionId, intrObj->tccNum);
            intrLow &= ~(1U << intrObj->tccNum);
            intrObj->cbFxn(intrObj, intrObj->appData);
        }
        if ((intrObj->tccNum >= 32U) && ((intrHigh & (1U << (intrObj->tccNum - 32U))) != 0U))
        {
            EDMA_clrIntrRegion(baseAddr, regionId, intrObj->tccNum);
            intrHigh &= ~(1U << (intrObj->tccNum - 32U));
            intrObj->cbFxn(intrObj, intrObj->appData);
        }
        /* Get next intr Obj. */
        intrObj = intrObj->nextIntr;
    }

    /* Clear the aggregator interrupt */
    HW_WR_REG32(attrs->intrAggStatusAddr, attrs->intrAggClearMask);
    /* re evaluate the edma interrupt. */
    HW_WR_FIELD32(baseAddr + EDMA_TPCC_IEVAL_RN(regionId), EDMA_TPCC_IEVAL_RN_EVAL, 1);
}
