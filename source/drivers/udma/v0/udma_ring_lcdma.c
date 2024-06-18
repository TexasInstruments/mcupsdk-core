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

/**
 *  \file udma_ring.c
 *
 *  \brief File containing the UDMA driver ring related APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/udma/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

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

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Udma_ringSetCfgLcdma(Udma_DrvHandleInt drvHandle,
                          Udma_RingHandleInt ringHandle,
                          const Udma_RingPrms *ringPrms)
{
    uint32_t            addrHi, addrLo /*, elemSize*/;
    CSL_LcdmaRingaccRingCfg *lcdmaRingCfg;

    /* Configure ring object */
    DebugP_assert(drvHandle->lcdmaRaRegs.pRingCfgRegs != NULL_PTR);
    DebugP_assert(drvHandle->lcdmaRaRegs.pRingRtRegs != NULL_PTR);
    DebugP_assert(ringHandle->ringNum < 303U);//from AM64x DMSS Spec, need to be updated once CSL changes.
    ringHandle->pLcdmaCfgRegs =
        &drvHandle->lcdmaRaRegs.pRingCfgRegs->RING[ringHandle->ringNum];
    ringHandle->pLcdmaRtRegs  =
        &drvHandle->lcdmaRaRegs.pRingRtRegs->RING[ringHandle->ringNum];

    lcdmaRingCfg = &ringHandle->lcdmaCfg;
    if(NULL_PTR != ringPrms)
    {
        lcdmaRingCfg->virtBase       = (void *) ringPrms->ringMem;
        lcdmaRingCfg->physBase       =
            Udma_virtToPhyFxn(ringPrms->ringMem, drvHandle, (Udma_ChHandleInt) NULL_PTR);
        lcdmaRingCfg->mode           = ringPrms->mode;
        lcdmaRingCfg->elCnt          = ringPrms->elemCnt;
        /* CSL expects ring size in bytes */
        lcdmaRingCfg->elSz           = ((uint32_t) 1U << (ringPrms->elemSize + 2U));
        lcdmaRingCfg->asel           = ringPrms->asel;
    }
    else
    {
        /* Init CSL ring object */
        lcdmaRingCfg = &ringHandle->lcdmaCfg;
        addrHi = CSL_REG32_FEXT(&ringHandle->pLcdmaCfgRegs->BA_HI, LCDMA_RINGACC_RING_CFG_RING_BA_HI_ADDR_HI);
        addrLo = CSL_REG32_FEXT(&ringHandle->pLcdmaCfgRegs->BA_LO, LCDMA_RINGACC_RING_CFG_RING_BA_LO_ADDR_LO);
        lcdmaRingCfg->physBase       = (uint64_t)((((uint64_t) addrHi) << 32UL) |
                                            ((uint64_t) addrLo));
        lcdmaRingCfg->virtBase       = Udma_phyToVirtFxn(lcdmaRingCfg->physBase, drvHandle, (Udma_ChHandleInt) NULL_PTR);
        lcdmaRingCfg->mode           = CSL_REG32_FEXT(&ringHandle->pLcdmaCfgRegs->SIZE, LCDMA_RINGACC_RING_CFG_RING_SIZE_QMODE);
        lcdmaRingCfg->elCnt          = CSL_REG32_FEXT(&ringHandle->pLcdmaCfgRegs->SIZE, LCDMA_RINGACC_RING_CFG_RING_SIZE_ELCNT);
        /* CSL expects ring size in bytes; ring_elsize for AM64x is hardcoded as 1=8bytes*/
        lcdmaRingCfg->elSz           = (uint32_t) 8U;
    }

    lcdmaRingCfg->credChkSecure  = 0U;
    lcdmaRingCfg->credSecure     = 0U;
    lcdmaRingCfg->credPriv       = 0U;
    lcdmaRingCfg->credPrivId     = CSL_LCDMA_RINGACC_CRED_PASSTHRU;
    lcdmaRingCfg->credVirtId     = CSL_LCDMA_RINGACC_CRED_PASSTHRU;
    CSL_lcdma_ringaccInitRingObj(ringHandle->ringNum, lcdmaRingCfg);

    return;
}

void Udma_ringHandleClearRegsLcdma(Udma_RingHandleInt ringHandle)
{
    ringHandle->pLcdmaCfgRegs   = (volatile CSL_lcdma_ringacc_ring_cfgRegs_RING *) NULL_PTR;
    ringHandle->pLcdmaRtRegs    = (volatile CSL_lcdma_ringacc_ringrtRegs_ring *) NULL_PTR;
}

int32_t Udma_ringQueueRawLcdma(Udma_DrvHandleInt  drvHandle, Udma_RingHandleInt ringHandle, uint64_t phyDescMem)
{
    int32_t         retVal = UDMA_SOK;

    retVal = CSL_lcdma_ringaccPush64(
        &ringHandle->drvHandle->lcdmaRaRegs,
        &ringHandle->lcdmaCfg,
        (uint64_t) phyDescMem,
        &Udma_lcdmaRingaccMemOps);

    return (retVal);
}

int32_t Udma_ringDequeueRawLcdma(Udma_DrvHandleInt  drvHandle, Udma_RingHandleInt ringHandle, uint64_t *phyDescMem)
{
    int32_t         retVal = UDMA_SOK, cslRetVal;

    cslRetVal = CSL_lcdma_ringaccPop64(
            &ringHandle->drvHandle->lcdmaRaRegs,
            &ringHandle->lcdmaCfg,
            phyDescMem,
            &Udma_lcdmaRingaccMemOps);
    if(0 != cslRetVal)
    {
        retVal = UDMA_ETIMEOUT;
    }

    return (retVal);
}

int32_t Udma_ringFlushRawLcdma(Udma_DrvHandleInt drvHandle, Udma_RingHandleInt ringHandle, uint64_t *phyDescMem)
{
    int32_t         retVal = UDMA_SOK, cslRetVal;
    uint32_t        addrlo;
    struct tisci_msg_rm_ring_cfg_req    rmRingReq;
    struct tisci_msg_rm_ring_cfg_resp   rmRingResp;

    cslRetVal = CSL_lcdma_ringaccDequeue(
            &ringHandle->drvHandle->lcdmaRaRegs,
            &ringHandle->lcdmaCfg,
            phyDescMem);
    if(0 != cslRetVal)
    {
        /*---------------------------------------------------------------------------------------
         * In case of LCDMA Rings, #CSL_lcdma_ringaccDequeue returns -1
         * when ring is empty (there are no more unprocessed descriptors in the ring to dequeue).
         * At this stage, resetting the ring is strongly suggested as part of the procedure
         * for returning a ring to a known state.
        -----------------------------------------------------------------------------------------*/
        /* Reset the ring by writing to ring's BA_LO, BA_HI, or SIZE register */
        addrlo = CSL_REG32_FEXT(&ringHandle->pLcdmaCfgRegs->BA_LO, LCDMA_RINGACC_RING_CFG_RING_BA_LO_ADDR_LO);
        rmRingReq.valid_params  = TISCI_MSG_VALUE_RM_RING_MODE_VALID |
                                  TISCI_MSG_VALUE_RM_RING_ADDR_LO_VALID;
        rmRingReq.nav_id        = drvHandle->devIdRing;
        rmRingReq.index         = ringHandle->ringNum;
        rmRingReq.mode          = TISCI_MSG_VALUE_RM_RING_MODE_RING;
        rmRingReq.addr_lo       = addrlo;
        /* Not used */
        rmRingReq.addr_hi       = 0U;
        rmRingReq.size          = 0U;
        rmRingReq.count         = 0U;
        rmRingReq.order_id      = UDMA_DEFAULT_RING_ORDER_ID;
        retVal = Sciclient_rmRingCfg(
                        &rmRingReq, &rmRingResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] Ring reset failed!!!\r\n");
            retVal = UDMA_EFAIL;
        }
        else
        {
            /* Initialize state fields */
            CSL_lcdma_ringaccInitRingObj(ringHandle->ringNum, &ringHandle->lcdmaCfg);
            /* ringFlush API returns UDMA_ETIMEOUT when there are no more
             * unprocessed descriptors to be dequeued and the ring reset
             * was successfull for returning a ring to a known state. */
            retVal = UDMA_ETIMEOUT;
        }
    }
    return (retVal);
}

void Udma_ringPrimeLcdma(Udma_RingHandleInt ringHandle, uint64_t phyDescMem)
{
    volatile uint64_t        *ringPtr;
    CSL_LcdmaRingaccRingCfg  *pLcdmaRing;
    uintptr_t                 tempPtr;

    pLcdmaRing = &ringHandle->lcdmaCfg;
    tempPtr = (uintptr_t)(pLcdmaRing->wrIdx * pLcdmaRing->elSz) +
              (uintptr_t)pLcdmaRing->virtBase;
    ringPtr = (volatile uint64_t *)(tempPtr);
    *ringPtr = phyDescMem;

    /* Book keeping */
    pLcdmaRing->wrIdx++;
    if(pLcdmaRing->wrIdx >= pLcdmaRing->elCnt)
    {
        pLcdmaRing->wrIdx = 0U;
    }
    pLcdmaRing->wrOcc++;

    return;
}

void Udma_ringPrimeReadLcdma(Udma_RingHandleInt ringHandle, uint64_t *phyDescMem)
{
    volatile uint64_t        *ringPtr;
    CSL_LcdmaRingaccRingCfg  *pLcdmaRing;
    uintptr_t                 tempPtr;

    pLcdmaRing = &ringHandle->lcdmaCfg;
    tempPtr = (uintptr_t)(pLcdmaRing->rdIdx * pLcdmaRing->elSz) +
              (uintptr_t)pLcdmaRing->virtBase;
    ringPtr = (volatile uint64_t *)(tempPtr);
    *phyDescMem = *ringPtr;

    if (*phyDescMem != 0U)
    {
        /* Book keeping */
        pLcdmaRing->rdIdx++;
        if(pLcdmaRing->rdIdx >= pLcdmaRing->elCnt)
        {
            pLcdmaRing->rdIdx = 0U;
        }
        pLcdmaRing->rdOcc--;
        pLcdmaRing->wrOcc--;
    }
}

void Udma_ringSetDoorBellLcdma(Udma_RingHandleInt ringHandle, int32_t count)
{
    uint32_t    regVal;
    int32_t     thisDbRingCnt, maxDbRingCnt;

    /* count will be positive when ring elements are queued into the ring */
    if (count >= 0)
    {
        /*-------------------------------------------------------------------------
        * Set maxDbRingCnt to the largest positive value that can be written to
        * the forward doorbell field (a two's compliment value).
        *-----------------------------------------------------------------------*/
        maxDbRingCnt = (int32_t)((((uint32_t)CSL_LCDMA_RINGACC_RINGRT_RING_FDB_CNT_MAX + 1U) >> 1) - 1U);

        while(count != 0)
        {
            if(count < maxDbRingCnt)
            {
                thisDbRingCnt = count;
                regVal = CSL_FMK(LCDMA_RINGACC_RINGRT_RING_FDB_CNT, (uint32_t)thisDbRingCnt);
            }
            else
            {
                thisDbRingCnt = maxDbRingCnt;
                regVal = CSL_FMK(LCDMA_RINGACC_RINGRT_RING_FDB_CNT, (uint32_t)thisDbRingCnt);
            }
            CSL_REG32_WR(&ringHandle->pLcdmaRtRegs->FDB, regVal);
            count -= thisDbRingCnt;
        }
    }

    /* count will be negative when ring elements are dequeued from the ring */
    else
    {
        /*-------------------------------------------------------------------------
        * Set maxDbRingCnt to the largest positive value that can be written to
        * the reverse doorbell field (a two's compliment value).
        *-----------------------------------------------------------------------*/
        maxDbRingCnt = (int32_t)((((uint32_t)CSL_LCDMA_RINGACC_RINGRT_RING_RDB_CNT_MAX + 1U) >> 1) - 1U);

        while(count != 0)
        {
            if(count > (-1 * (int32_t)maxDbRingCnt))
            {
                thisDbRingCnt = count;
                regVal = CSL_FMK(LCDMA_RINGACC_RINGRT_RING_RDB_CNT, (uint32_t)thisDbRingCnt);
            }
            else
            {
                thisDbRingCnt = (-1 * (int32_t)maxDbRingCnt);
                regVal = CSL_FMK(LCDMA_RINGACC_RINGRT_RING_RDB_CNT, (uint32_t)thisDbRingCnt);
            }
            CSL_REG32_WR(&ringHandle->pLcdmaRtRegs->RDB, regVal);
            count -= thisDbRingCnt;
        }
    }

    return;
}

void *Udma_ringGetMemPtrLcdma(Udma_RingHandleInt ringHandle)
{
    void   *ringMem = NULL_PTR;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        ringMem = ringHandle->lcdmaCfg.virtBase;
    }

    return (ringMem);
}

uint32_t Udma_ringGetModeLcdma(Udma_RingHandleInt ringHandle)
{
    uint32_t ringMode = CSL_LCDMA_RINGACC_RING_MODE_INVALID;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        ringMode = ringHandle->lcdmaCfg.mode;
    }

    return (ringMode);
}

uint32_t Udma_ringGetElementCntLcdma(Udma_RingHandleInt ringHandle)
{
    uint32_t size = 0U;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        size = ringHandle->lcdmaCfg.elCnt;
    }

    return (size);
}

uint32_t Udma_ringGetForwardRingOccLcdma(Udma_RingHandleInt ringHandle)
{
    uint32_t occ = 0U;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        occ = CSL_lcdma_ringaccGetForwardRingOcc(&ringHandle->drvHandle->lcdmaRaRegs,
                                                 ringHandle->ringNum,
                                                 ringHandle->lcdmaCfg.mode);
        /* Update lcdmaCfg->wrOcc */
        ringHandle->lcdmaCfg.wrOcc = occ;
    }

    return (occ);
}

uint32_t Udma_ringGetReverseRingOccLcdma(Udma_RingHandleInt ringHandle)
{
    uint32_t occ = 0U;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        occ = CSL_lcdma_ringaccGetReverseRingOcc(&ringHandle->drvHandle->lcdmaRaRegs,
                                                 ringHandle->ringNum,
                                                 ringHandle->lcdmaCfg.mode);
        /* Update lcdmaCfg->rdOcc */
        ringHandle->lcdmaCfg.rdOcc = occ;
    }

    return (occ);
}

uint32_t Udma_ringGetWrIdxLcdma(Udma_RingHandleInt ringHandle)
{
    uint32_t idx = 0U;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        idx = ringHandle->lcdmaCfg.wrIdx;
    }

    return (idx);
}

uint32_t Udma_ringGetRdIdxLcdma(Udma_RingHandleInt ringHandle)
{
    uint32_t idx = 0U;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        idx = ringHandle->lcdmaCfg.rdIdx;
    }

    return (idx);
}

void Udma_lcdmaRingaccMemOps(void *pVirtAddr, uint32_t size, uint32_t opsType)
{
    uint32_t    isCacheCoherent = Udma_isCacheCoherent();

    if(isCacheCoherent != TRUE)
    {
        if(CSL_LCDMA_RINGACC_MEM_OPS_TYPE_WR == opsType)
        {
            CacheP_wb(pVirtAddr, size, CacheP_TYPE_ALL);
        }

        if(CSL_LCDMA_RINGACC_MEM_OPS_TYPE_RD == opsType)
        {
            CacheP_inv(pVirtAddr, size, CacheP_TYPE_ALL);
        }
    }

    return;
}
