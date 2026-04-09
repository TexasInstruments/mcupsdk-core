/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
 *  \file  v1/i2c_dma.c
 *  \brief Application-level EDMA utility for I2C DMA transfers.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HwiP.h>
#include "i2c_dma.h"

/* ========================================================================== */
/*                         Internal ISR Callbacks                             */
/* ========================================================================== */

/**
 *  \brief  EDMA TX completion ISR.
 *
 *  Posts the TX completion semaphore so the application can proceed after
 *  the transmit DMA finishes.
 *
 *  \param intrHandle   EDMA interrupt handle (unused)
 *  \param args         Pointer to the I2C_DmaChConfig for this channel
 */
static void I2C_dmaTxIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    I2C_DmaChConfig *cfg = (I2C_DmaChConfig *)args;

    SemaphoreP_post(&cfg->txDoneSem);
}

/**
 *  \brief  EDMA RX completion ISR.
 *
 *  Invalidates the receive buffer so the CPU sees fresh DMA-written data,
 *  then posts the RX completion semaphore.
 *
 *  \param intrHandle   EDMA interrupt handle (unused)
 *  \param args         Pointer to the I2C_DmaChConfig for this channel
 */
static void I2C_dmaRxIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    I2C_DmaChConfig *cfg = (I2C_DmaChConfig *)args;

    /* Invalidate the receive buffer so the CPU sees fresh DMA-written data.
     * The DSB ensures the cache maintenance completes before the semaphore
     * post makes the data visible to the pending thread. */
    CacheP_inv((void *)cfg->rxBuf, cfg->rxBufSize, CacheP_TYPE_ALL);
    __asm__ volatile("dsb" ::: "memory");

    SemaphoreP_post(&cfg->rxDoneSem);
}

/* ========================================================================== */
/*                         Public API Implementation                          */
/* ========================================================================== */

int32_t I2C_dmaOpen(I2C_DmaChConfig *cfg)
{
    int32_t  status = SystemP_SUCCESS;
    uint32_t ch, tcc, param;

    DebugP_assert(cfg != NULL);
    DebugP_assert(cfg->edmaHandle != NULL);

    /* Get EDMA base address and region — shared by TX and RX */
    cfg->baseAddr = EDMA_getBaseAddr(cfg->edmaHandle);
    DebugP_assert(cfg->baseAddr != 0U);

    cfg->regionId = EDMA_getRegionId(cfg->edmaHandle);

    /* ------------------------------------------------------------------ */
    /* TX channel: allocate, configure channel region, register interrupt  */
    /* ------------------------------------------------------------------ */

    ch     = cfg->edmaTxChNum;
    status = EDMA_allocDmaChannel(cfg->edmaHandle, &ch);
    DebugP_assert(status == SystemP_SUCCESS);
    cfg->txCh = ch;

    tcc    = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(cfg->edmaHandle, &tcc);
    DebugP_assert(status == SystemP_SUCCESS);
    cfg->txTcc = tcc;

    param  = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(cfg->edmaHandle, &param);
    DebugP_assert(status == SystemP_SUCCESS);
    cfg->txParam = param;

    EDMA_configureChannelRegion(cfg->baseAddr, cfg->regionId,
                                EDMA_CHANNEL_TYPE_DMA,
                                cfg->txCh, cfg->txTcc,
                                cfg->txParam, I2C_EDMA_EVT_QUEUE_NO);

    status = SemaphoreP_constructBinary(&cfg->txDoneSem, 0U);
    DebugP_assert(status == SystemP_SUCCESS);

    memset(&cfg->txIntrObj, 0, sizeof(cfg->txIntrObj));
    cfg->txIntrObj.tccNum  = cfg->txTcc;
    cfg->txIntrObj.cbFxn   = I2C_dmaTxIsrFxn;
    cfg->txIntrObj.appData = (void *)cfg;
    status = EDMA_registerIntr(cfg->edmaHandle, &cfg->txIntrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    /* ------------------------------------------------------------------ */
    /* RX channel: allocate, configure channel region, register interrupt  */
    /* ------------------------------------------------------------------ */

    ch     = cfg->edmaRxChNum;
    status = EDMA_allocDmaChannel(cfg->edmaHandle, &ch);
    DebugP_assert(status == SystemP_SUCCESS);
    cfg->rxCh = ch;

    tcc    = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(cfg->edmaHandle, &tcc);
    DebugP_assert(status == SystemP_SUCCESS);
    cfg->rxTcc = tcc;

    param  = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(cfg->edmaHandle, &param);
    DebugP_assert(status == SystemP_SUCCESS);
    cfg->rxParam = param;

    EDMA_configureChannelRegion(cfg->baseAddr, cfg->regionId,
                                EDMA_CHANNEL_TYPE_DMA,
                                cfg->rxCh, cfg->rxTcc,
                                cfg->rxParam, I2C_EDMA_EVT_QUEUE_NO);

    status = SemaphoreP_constructBinary(&cfg->rxDoneSem, 0U);
    DebugP_assert(status == SystemP_SUCCESS);

    memset(&cfg->rxIntrObj, 0, sizeof(cfg->rxIntrObj));
    cfg->rxIntrObj.tccNum  = cfg->rxTcc;
    cfg->rxIntrObj.cbFxn   = I2C_dmaRxIsrFxn;
    cfg->rxIntrObj.appData = (void *)cfg;
    status = EDMA_registerIntr(cfg->edmaHandle, &cfg->rxIntrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    return status;
}

int32_t I2C_dmaConfigTx(I2C_DmaChConfig *cfg, uint8_t *buf, uint32_t nBytes)
{
    int32_t          status = SystemP_SUCCESS;
    uint32_t         edmaStatus;
    EDMACCPaRAMEntry edmaParam;

    DebugP_assert(cfg != NULL);
    DebugP_assert(buf != NULL);
    DebugP_assert(nBytes > 0U);

    /* Write-back the TX buffer from cache to memory before DMA reads it */
    CacheP_wb((void *)buf, nBytes, CacheP_TYPE_ALL);

    /* Build PaRAM:
     *   A-sync mode: one byte transferred per event (I2C TX-ready trigger)
     *   Source: user TX buffer (increments each byte)
     *   Destination: I2C TX data register (fixed) */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr    = (uint32_t)(uintptr_t)buf;
    edmaParam.destAddr   = cfg->i2cTxRegAddr;
    edmaParam.aCnt       = (uint16_t)1U;
    edmaParam.bCnt       = (uint16_t)nBytes;
    edmaParam.cCnt       = (uint16_t)1U;
    edmaParam.srcBIdx    = (int16_t)1;   /* source increments by 1 byte */
    edmaParam.destBIdx   = (int16_t)0;   /* destination is fixed register */
    edmaParam.srcCIdx    = (int16_t)0;
    edmaParam.destCIdx   = (int16_t)0;
    edmaParam.bCntReload = (uint16_t)0U;
    edmaParam.linkAddr   = (uint16_t)0xFFFFU; /* no reload */
    edmaParam.opt        = 0U;
    /* TCINTEN only — fires once when bCnt reaches 0 (all bytes transferred).
     * ITCINTEN would fire after every A-array (every byte in A-sync mode),
     * causing txDoneSem to post after the first byte, not the last. */
    edmaParam.opt |= (EDMA_OPT_TCINTEN_MASK |
                      ((cfg->txTcc << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(cfg->baseAddr, cfg->txParam, &edmaParam);

    /* Arm channel for event-triggered transfer */
    edmaStatus = EDMA_enableTransferRegion(cfg->baseAddr, cfg->regionId,
                                           cfg->txCh, EDMA_TRIG_MODE_EVENT);
    if (edmaStatus == TRUE)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    DebugP_assert(status == SystemP_SUCCESS);

    return status;
}

int32_t I2C_dmaConfigRx(I2C_DmaChConfig *cfg, uint8_t *buf, uint32_t nBytes)
{
    int32_t          status = SystemP_SUCCESS;
    uint32_t         edmaStatus;
    EDMACCPaRAMEntry edmaParam;

    DebugP_assert(cfg != NULL);
    DebugP_assert(buf != NULL);
    DebugP_assert(nBytes > 0U);

    /* Store buffer info so the RX ISR can invalidate the correct region */
    cfg->rxBuf     = buf;
    cfg->rxBufSize = nBytes;

    /* Build PaRAM:
     *   A-sync mode: one byte transferred per event (I2C RX-ready trigger)
     *   Source: I2C RX data register (fixed)
     *   Destination: user RX buffer (increments each byte) */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr    = cfg->i2cRxRegAddr;
    edmaParam.destAddr   = (uint32_t)(uintptr_t)buf;
    edmaParam.aCnt       = (uint16_t)1U;
    edmaParam.bCnt       = (uint16_t)nBytes;
    edmaParam.cCnt       = (uint16_t)1U;
    edmaParam.srcBIdx    = (int16_t)0;   /* source is fixed register */
    edmaParam.destBIdx   = (int16_t)1;   /* destination increments by 1 byte */
    edmaParam.srcCIdx    = (int16_t)0;
    edmaParam.destCIdx   = (int16_t)0;
    edmaParam.bCntReload = (uint16_t)0U;
    edmaParam.linkAddr   = (uint16_t)0xFFFFU; /* no reload */
    edmaParam.opt        = 0U;
    /* TCINTEN only — fires once when bCnt reaches 0 (all bytes transferred).
     * ITCINTEN would fire after every A-array (every byte in A-sync mode),
     * causing rxDoneSem to post after the first byte, not the last. */
    edmaParam.opt |= (EDMA_OPT_TCINTEN_MASK |
                      ((cfg->rxTcc << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(cfg->baseAddr, cfg->rxParam, &edmaParam);

    /* Arm channel for event-triggered transfer */
    edmaStatus = EDMA_enableTransferRegion(cfg->baseAddr, cfg->regionId,
                                           cfg->rxCh, EDMA_TRIG_MODE_EVENT);
    if (edmaStatus == TRUE)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    DebugP_assert(status == SystemP_SUCCESS);

    return status;
}

void I2C_dmaClose(I2C_DmaChConfig *cfg)
{
    int32_t  status;
    uint32_t edmaStatus;

    if (cfg == NULL)
    {
        return;
    }

    /* Unregister TX interrupt and free TX resources */
    status = EDMA_unregisterIntr(cfg->edmaHandle, &cfg->txIntrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    edmaStatus = EDMA_freeChannelRegion(cfg->baseAddr, cfg->regionId,
                                        EDMA_CHANNEL_TYPE_DMA,
                                        cfg->txCh, EDMA_TRIG_MODE_EVENT,
                                        cfg->txTcc, I2C_EDMA_EVT_QUEUE_NO);
    DebugP_assert(edmaStatus == TRUE);

    SemaphoreP_destruct(&cfg->txDoneSem);

    status = EDMA_freeDmaChannel(cfg->edmaHandle, &cfg->txCh);
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeTcc(cfg->edmaHandle, &cfg->txTcc);
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeParam(cfg->edmaHandle, &cfg->txParam);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Unregister RX interrupt and free RX resources */
    status = EDMA_unregisterIntr(cfg->edmaHandle, &cfg->rxIntrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    edmaStatus = EDMA_freeChannelRegion(cfg->baseAddr, cfg->regionId,
                                        EDMA_CHANNEL_TYPE_DMA,
                                        cfg->rxCh, EDMA_TRIG_MODE_EVENT,
                                        cfg->rxTcc, I2C_EDMA_EVT_QUEUE_NO);
    DebugP_assert(edmaStatus == TRUE);

    SemaphoreP_destruct(&cfg->rxDoneSem);

    status = EDMA_freeDmaChannel(cfg->edmaHandle, &cfg->rxCh);
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeTcc(cfg->edmaHandle, &cfg->rxTcc);
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_freeParam(cfg->edmaHandle, &cfg->rxParam);
    DebugP_assert(status == SystemP_SUCCESS);
}
