/*
 *  Copyright (c) 2024 Texas Instruments Incorporated
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

/** \brief Max number of door bell ring that can be performed at one go */
#define UDMA_RING_MAX_DB_RING_CNT       (127U)

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

void Udma_ringSetCfgNormal(Udma_DrvHandleInt drvHandle,
                           Udma_RingHandleInt ringHandle,
                           const Udma_RingPrms *ringPrms)
{
    uint32_t            addrHi, addrLo, elemSize;
    CSL_RingAccRingCfg *ringCfg;

    /* Configure ring object */
    Udma_assert(drvHandle, drvHandle->raRegs.pCfgRegs != NULL_PTR);
    Udma_assert(drvHandle, drvHandle->raRegs.pRtRegs != NULL_PTR);
    Udma_assert(drvHandle, ringHandle->ringNum < 1024U);
    ringHandle->pCfgRegs =
        &drvHandle->raRegs.pCfgRegs->RING[ringHandle->ringNum];
    ringHandle->pRtRegs         =
        &drvHandle->raRegs.pRtRegs->RINGRT[ringHandle->ringNum];

    ringCfg = &ringHandle->cfg;
    if(NULL_PTR != ringPrms)
    {
        ringCfg->physBase    =
            Udma_virtToPhyFxn(ringPrms->ringMem, drvHandle, (Udma_ChHandleInt) NULL_PTR);
        ringCfg->virtBase    = (void *) ringPrms->ringMem;
        ringCfg->mode        = ringPrms->mode;
        ringCfg->elCnt       = ringPrms->elemCnt;
        /* CSL expects ring size in bytes */
        ringCfg->elSz        = ((uint32_t) 1U << (ringPrms->elemSize + 2U));
    }
    else
    {
        /* Init CSL ring object */
        addrHi = CSL_REG32_FEXT(&ringHandle->pCfgRegs->BA_HI, RINGACC_CFG_RING_BA_HI_ADDR_HI);
        addrLo = CSL_REG32_FEXT(&ringHandle->pCfgRegs->BA_LO, RINGACC_CFG_RING_BA_LO_ADDR_LO);
        ringCfg->physBase    = (uint64_t)((((uint64_t) addrHi) << 32UL) |
                                            ((uint64_t) addrLo));
        ringCfg->virtBase    = Udma_phyToVirtFxn(ringCfg->physBase, drvHandle, (Udma_ChHandleInt) NULL_PTR);
        ringCfg->mode        = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_QMODE);
        ringCfg->elCnt       = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_ELCNT);
        elemSize             = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_ELSIZE);
        /* CSL expects ring size in bytes */
        ringCfg->elSz        = ((uint32_t) 1U << (elemSize + 2U));
    }

    ringCfg->evtNum      = UDMA_EVENT_INVALID;
    ringCfg->credSecure  = 0U;
    ringCfg->credPriv    = 0U;
    ringCfg->credPrivId  = CSL_RINGACC_CRED_PASSTHRU;
    ringCfg->credVirtId  = CSL_RINGACC_CRED_PASSTHRU;
    CSL_ringaccInitRingObj(ringHandle->ringNum, ringCfg);

    ringHandle->proxyAddr =
        CSL_proxyGetDataAddr(
            &drvHandle->proxyCfg,
            drvHandle->proxyTargetNumRing,
            drvHandle->rmInitPrms.proxyThreadNum,
            ringCfg->elSz);

    return;
}

void Udma_ringHandleClearRegsNormal(Udma_RingHandleInt ringHandle)
{
    ringHandle->pCfgRegs        = (volatile CSL_ringacc_cfgRegs_RING *) NULL_PTR;
    ringHandle->pRtRegs         = (volatile CSL_ringacc_rtRegs_RINGRT *) NULL_PTR;
}

int32_t Udma_ringQueueRawNormal(Udma_DrvHandleInt drvHandle, Udma_RingHandleInt ringHandle, uint64_t phyDescMem)
{
    int32_t         retVal = UDMA_SOK;

    if(TISCI_MSG_VALUE_RM_RING_MODE_RING == ringHandle->cfg.mode)
    {
        /* Use direct memory access for RING mode */
        retVal = CSL_ringaccPush64(
            &ringHandle->drvHandle->raRegs,
            &ringHandle->cfg,
            (uint64_t) phyDescMem,
            &Udma_ringaccMemOps);
    }
    else
    {
        /* Use proxy for other modes */
        retVal = Udma_ringProxyQueueRaw(ringHandle, drvHandle, phyDescMem);
    }

    return (retVal);
}

int32_t Udma_ringDequeueRawNormal(Udma_DrvHandleInt drvHandle, Udma_RingHandleInt ringHandle, uint64_t *phyDescMem)
{
    int32_t         retVal = UDMA_SOK, cslRetVal;

    if(TISCI_MSG_VALUE_RM_RING_MODE_RING == ringHandle->cfg.mode)
    {
        /* Use direct memory access for RING mode */
        cslRetVal = CSL_ringaccPop64(
            &ringHandle->drvHandle->raRegs,
            &ringHandle->cfg,
            phyDescMem,
            &Udma_ringaccMemOps);
        if(0 != cslRetVal)
        {
            retVal = UDMA_ETIMEOUT;
        }
    }
    else
    {
        /* Use proxy for other modes */
        retVal = Udma_ringProxyDequeueRaw(ringHandle, drvHandle, phyDescMem);
    }

    return (retVal);
}

int32_t Udma_ringFlushRawNormal(Udma_DrvHandleInt drvHandle, Udma_RingHandleInt ringHandle, uint64_t *phyDescMem)
{
    int32_t         retVal = UDMA_SOK;

    /* Same as proxy dequeue API as proxy can be used in all modes of ring */
    retVal = Udma_ringProxyDequeueRaw(ringHandle, drvHandle, phyDescMem);

    return (retVal);
}

void Udma_ringPrimeNormal(Udma_RingHandleInt ringHandle, uint64_t phyDescMem)
{
    volatile uint64_t        *ringPtr;
    CSL_RingAccRingCfg       *pRing;
    uintptr_t                 tempPtr;

    pRing = &ringHandle->cfg;
    tempPtr = (uintptr_t)(pRing->rwIdx * pRing->elSz) +
              (uintptr_t)pRing->virtBase;
    ringPtr = (volatile uint64_t *)(tempPtr);
    *ringPtr = phyDescMem;

    /* Book keeping */
    pRing->waiting++;
    pRing->rwIdx++;
    if(pRing->rwIdx >= pRing->elCnt)
    {
        pRing->rwIdx = 0U;
    }
    pRing->occ++;

    return;
}

void Udma_ringPrimeReadNormal(Udma_RingHandleInt ringHandle, uint64_t *phyDescMem)
{
    volatile uint64_t        *ringPtr;
    CSL_RingAccRingCfg       *pRing;
    uintptr_t                 tempPtr;

    pRing = &ringHandle->cfg;
    tempPtr = (uintptr_t)(pRing->rwIdx * pRing->elSz) +
              (uintptr_t)pRing->virtBase;
    ringPtr = (volatile uint64_t *)(tempPtr);
    *phyDescMem = *ringPtr;

    if (*phyDescMem != 0U)
    {
        /* Book keeping */
        pRing->waiting++;
        pRing->rwIdx++;
        if(pRing->rwIdx >= pRing->elCnt)
        {
            pRing->rwIdx = 0U;
        }
        pRing->occ--;
    }
}

void Udma_ringSetDoorBellNormal(Udma_RingHandleInt ringHandle, int32_t count)
{
    uint32_t    regVal;
    int32_t     thisDbRingCnt;
    CSL_RingAccRingCfg       *pRing;

    pRing = &ringHandle->cfg;

    /* count will be positive when ring elements are queued into the ring */
    if (count >= 0)
    {
        while(count != 0)
        {
            if(count < UDMA_RING_MAX_DB_RING_CNT)
            {
                thisDbRingCnt = count;
                regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
            }
            else
            {
                thisDbRingCnt = UDMA_RING_MAX_DB_RING_CNT;
                regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
            }
            CSL_REG32_WR(&ringHandle->pRtRegs->DB, regVal);
            pRing->waiting -= thisDbRingCnt;
            count -= thisDbRingCnt;
        }
    }

    /* count will be negative when ring elements are dequeued from the ring */
    else
    {
        while(count != 0)
        {
            if(count > (-1 * (int32_t)UDMA_RING_MAX_DB_RING_CNT))
            {
                thisDbRingCnt = count;
                regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
            }
            else
            {
                thisDbRingCnt = (-1 * (int32_t)UDMA_RING_MAX_DB_RING_CNT);
                regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
            }
            CSL_REG32_WR(&ringHandle->pRtRegs->DB, regVal);
            pRing->waiting += thisDbRingCnt;
            count -= thisDbRingCnt;
        }
    }
}

void *Udma_ringGetMemPtrNormal(Udma_RingHandleInt ringHandle)
{
    void   *ringMem = NULL_PTR;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        ringMem = ringHandle->cfg.virtBase;
    }

    return (ringMem);
}
uint32_t Udma_ringGetModeNormal(Udma_RingHandleInt ringHandle)
{
    uint32_t ringMode = CSL_RINGACC_RING_MODE_INVALID;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        ringMode = ringHandle->cfg.mode;
    }

    return (ringMode);
}

uint32_t Udma_ringGetElementCntNormal(Udma_RingHandleInt ringHandle)
{
    uint32_t size = 0U;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        size = ringHandle->cfg.elCnt;
    }

    return (size);
}

uint32_t Udma_ringGetRingOccNormal(Udma_RingHandleInt ringHandle)
{
    uint32_t occ = 0U;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        occ = CSL_ringaccGetRingHwOcc(&ringHandle->drvHandle->raRegs,
                                      ringHandle->ringNum);
        /* Update cfg->occ */
        ringHandle->cfg.occ = occ;
    }

    return (occ);
}

uint32_t Udma_ringGetWrIdxNormal(Udma_RingHandleInt ringHandle)
{
    uint32_t idx = 0U;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        idx = ringHandle->cfg.rwIdx;
    }

    return (idx);
}

uint32_t Udma_ringGetRdIdxNormal(Udma_RingHandleInt ringHandle)
{
    uint32_t idx = 0U;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        idx = ringHandle->cfg.rwIdx;
    }

    return (idx);
}

void Udma_ringaccMemOps(void *pVirtAddr, uint32_t size, uint32_t opsType)
{
    uint32_t    isCacheCoherent = Udma_isCacheCoherent();

    if(isCacheCoherent != TRUE)
    {
        if(CSL_RINGACC_MEM_OPS_TYPE_WR == opsType)
        {
            CacheP_wb(pVirtAddr, size, CacheP_TYPE_ALL);
        }

        if(CSL_RINGACC_MEM_OPS_TYPE_RD == opsType)
        {
            CacheP_inv(pVirtAddr, size, CacheP_TYPE_ALL);
        }
    }

    return;
}