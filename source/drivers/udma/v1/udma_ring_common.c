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

static int32_t Udma_ringCheckParams(Udma_DrvHandleInt drvHandle,
                                    const Udma_RingPrms *ringPrms);
static inline void Udma_ringAssertFnPointers(Udma_DrvHandleInt drvHandle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_ringAlloc(Udma_DrvHandle drvHandle,
                       Udma_RingHandle ringHandle,
                       uint16_t ringNum,
                       const Udma_RingPrms *ringPrms)
{
    int32_t             retVal = UDMA_SOK;
    uint64_t            physBase;
    uint32_t            allocDone = (uint32_t) FALSE;
    Udma_DrvHandleInt   drvHandleInt = (Udma_DrvHandleInt) drvHandle;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    struct tisci_msg_rm_ring_cfg_req    rmRingReq;
    struct tisci_msg_rm_ring_cfg_resp   rmRingResp;

    /* Error check */
    if((NULL_PTR == drvHandleInt) ||
       (NULL_PTR == ringHandleInt) ||
       (NULL_PTR == ringPrms))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(drvHandleInt->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        retVal = Udma_ringCheckParams(drvHandleInt, ringPrms);
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_RING_ANY == ringNum)
        {
            /* Alloc free ring */
            if(UDMA_MAPPED_GROUP_INVALID == ringPrms->mappedRingGrp)
            {
                ringHandleInt->ringNum = Udma_rmAllocFreeRing(drvHandleInt);
            }

            if(UDMA_RING_INVALID == ringHandleInt->ringNum)
            {
                retVal = UDMA_EALLOC;
            }
            else
            {
                allocDone = (uint32_t) TRUE;
            }
        }
        else
        {
            if(ringNum >= drvHandleInt->maxRings)
            {
                DebugP_logError("[UDMA] Out of range ring index!!!\r\n");
                retVal = UDMA_EINVALID_PARAMS;
            }
            else
            {
                ringHandleInt->ringNum = ringNum;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        Udma_ringAssertFnPointers(drvHandleInt);
        ringHandleInt->drvHandle = drvHandleInt;
        /* Set the mapped group in ringHandleInt, since only ringHandleInt is passed to rmFreeMappedRing() and
         * the mapped group parameter is required to reset the appropriate flag */
        ringHandleInt->mappedRingGrp   = ringPrms->mappedRingGrp;
        ringHandleInt->mappedChNum     = ringPrms->mappedChNum;
        drvHandleInt->ringSetCfg(drvHandleInt, ringHandleInt, ringPrms);
        /* Perform ring reset */
        retVal = Udma_ringReset(drvHandleInt, ringHandleInt);
        if(UDMA_SOK != retVal)
        {
            DebugP_logError("[UDMA] Ring reset failed!!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Configure ring */
        rmRingReq.valid_params  = TISCI_MSG_VALUE_RM_RING_ADDR_LO_VALID |
                                  TISCI_MSG_VALUE_RM_RING_ADDR_HI_VALID |
                                  TISCI_MSG_VALUE_RM_RING_COUNT_VALID |
                                  TISCI_MSG_VALUE_RM_RING_MODE_VALID |
                                  TISCI_MSG_VALUE_RM_RING_SIZE_VALID |
                                  TISCI_MSG_VALUE_RM_RING_ORDER_ID_VALID |
                                  TISCI_MSG_VALUE_RM_RING_ASEL_VALID;
        rmRingReq.nav_id        = drvHandleInt->devIdRing;
        rmRingReq.index         = ringHandleInt->ringNum;
        physBase = Udma_virtToPhyFxn(ringPrms->ringMem, drvHandleInt, (Udma_ChHandleInt) NULL_PTR);
        rmRingReq.addr_lo       = (uint32_t)physBase;
        rmRingReq.addr_hi       = (uint32_t)(physBase >> 32UL);
        rmRingReq.count         = ringPrms->elemCnt;
        rmRingReq.mode          = ringPrms->mode;
        rmRingReq.size          = ringPrms->elemSize;
        rmRingReq.order_id      = ringPrms->orderId;
        rmRingReq.asel          = ringPrms->asel;

        if(UDMA_RING_VIRTID_INVALID != ringPrms->virtId)
        {
            rmRingReq.valid_params |= TISCI_MSG_VALUE_RM_RING_VIRTID_VALID;
            rmRingReq.virtid        = ringPrms->virtId;
        }

        retVal = Sciclient_rmRingCfg(
                     &rmRingReq, &rmRingResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] Ring config failed!!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        ringHandleInt->ringInitDone = UDMA_INIT_DONE;
    }
    else
    {
        /* Error. Free-up resource if allocated */
        if(((uint32_t) TRUE) == allocDone)
        {
            if(UDMA_MAPPED_GROUP_INVALID == ringPrms->mappedRingGrp)
            {
                Udma_rmFreeFreeRing(ringHandleInt->ringNum, drvHandleInt);
            }
        }
    }

    return (retVal);
}

int32_t Udma_ringFree(Udma_RingHandle ringHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;

    /* Error check */
    if(NULL_PTR == ringHandleInt)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(ringHandleInt->ringInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Free-up event resources */
        DebugP_assert(ringHandleInt->ringNum != UDMA_RING_INVALID);
        if(UDMA_MAPPED_GROUP_INVALID == ringHandleInt->mappedRingGrp)
        {
            Udma_rmFreeFreeRing(ringHandleInt->ringNum, drvHandle);
        }

        ringHandleInt->ringNum         = UDMA_RING_INVALID;
        ringHandleInt->ringInitDone    = UDMA_DEINIT_DONE;
        drvHandle->ringHandleClearRegs(ringHandleInt);
        ringHandleInt->drvHandle       = (Udma_DrvHandleInt) NULL_PTR;
    }

    return (retVal);
}

int32_t Udma_ringAttach(Udma_DrvHandle drvHandle,
                        Udma_RingHandle ringHandle,
                        uint16_t ringNum)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandleInt = (Udma_DrvHandleInt) drvHandle;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;

    /* Error check */
    if((NULL_PTR == drvHandleInt) || (NULL_PTR == ringHandleInt))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(drvHandleInt->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        if(ringNum >= drvHandleInt->maxRings)
        {
            DebugP_logError("[UDMA] Out of range ring index!!!\r\n");
            retVal = UDMA_EINVALID_PARAMS;
        }
    }

    if(UDMA_SOK == retVal)
    {
        ringHandleInt->ringNum = ringNum;
        ringHandleInt->drvHandle = drvHandleInt;
        drvHandleInt->ringSetCfg(drvHandleInt, ringHandleInt, (Udma_RingPrms *) NULL_PTR);
        ringHandleInt->ringInitDone = UDMA_INIT_DONE;
    }

    return (retVal);
}

int32_t Udma_ringDetach(Udma_RingHandle ringHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;

    /* Error check */
    if(NULL_PTR == ringHandleInt)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(ringHandleInt->ringInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) ||
           (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Clear handle object */
        DebugP_assert(ringHandleInt->ringNum != UDMA_RING_INVALID);
        ringHandleInt->ringInitDone    = UDMA_DEINIT_DONE;
        drvHandle->ringHandleClearRegs(ringHandleInt);
        ringHandleInt->drvHandle       = (Udma_DrvHandleInt) NULL_PTR;

    }

    return (retVal);
}

int32_t Udma_ringQueueRaw(Udma_RingHandle ringHandle, uint64_t phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    uintptr_t           cookie;
    Udma_DrvHandleInt   drvHandle;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;

    /* Error check */
    if((NULL_PTR == ringHandleInt) ||
       (ringHandleInt->ringInitDone != UDMA_INIT_DONE) ||
       (ringHandleInt->ringNum == UDMA_RING_INVALID))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) ||
           (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        cookie = HwiP_disable();

        retVal = drvHandle->ringQueueRaw(drvHandle, ringHandleInt, phyDescMem);

        HwiP_restore(cookie);
    }

    return (retVal);
}

int32_t Udma_ringDequeueRaw(Udma_RingHandle ringHandle, uint64_t *phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    uintptr_t           cookie;
    Udma_DrvHandleInt   drvHandle;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;

    /* Error check */
    if((NULL_PTR == ringHandleInt) ||
       (ringHandleInt->ringInitDone != UDMA_INIT_DONE) ||
       (ringHandleInt->ringNum == UDMA_RING_INVALID))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) ||
           (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        cookie = HwiP_disable();

        retVal = drvHandle->ringDequeueRaw(drvHandle, ringHandleInt, phyDescMem);

        HwiP_restore(cookie);
    }

    return (retVal);
}

int32_t Udma_ringFlushRaw(Udma_RingHandle ringHandle, uint64_t *phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;

    /* Error check */
    if((NULL_PTR == ringHandleInt) ||
       (ringHandleInt->ringInitDone != UDMA_INIT_DONE) ||
       (ringHandleInt->ringNum == UDMA_RING_INVALID))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) ||
           (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        retVal = drvHandle->ringFlushRaw(drvHandle, ringHandleInt, phyDescMem);
    }

    return (retVal);
}

void Udma_ringPrime(Udma_RingHandle ringHandle, uint64_t phyDescMem)
{
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    drvHandle->ringPrime(ringHandleInt, phyDescMem);

    return;
}

void Udma_ringPrimeRead(Udma_RingHandle ringHandle, uint64_t *phyDescMem)
{
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    drvHandle->ringPrimeRead(ringHandleInt, phyDescMem);

    return;
}

void Udma_ringSetDoorBell(Udma_RingHandle ringHandle, int32_t count)
{
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    drvHandle->ringSetDoorBell(ringHandleInt, count);

    return;
}

uint16_t Udma_ringGetNum(Udma_RingHandle ringHandle)
{
    uint16_t            ringNum = UDMA_RING_INVALID;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;

    if((NULL_PTR != ringHandleInt) &&
       (UDMA_INIT_DONE == ringHandleInt->ringInitDone))
    {
        ringNum = ringHandleInt->ringNum;
    }

    return (ringNum);
}

void *Udma_ringGetMemPtr(Udma_RingHandle ringHandle)
{
    void               *ringMem = NULL_PTR;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    ringMem = drvHandle->ringGetMemPtr(ringHandleInt);

    return (ringMem);
}

uint32_t Udma_ringGetMode(Udma_RingHandle ringHandle)
{
    uint32_t            ringMode;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    ringMode = drvHandle->ringGetMode(ringHandleInt);

    return (ringMode);
}

uint32_t Udma_ringGetElementCnt(Udma_RingHandle ringHandle)
{
    uint32_t            size = 0U;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    size = drvHandle->ringGetElementCnt(ringHandleInt);

    return (size);
}

uint32_t Udma_ringGetForwardRingOcc(Udma_RingHandle ringHandle)
{
    uint32_t            occ = 0U;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    occ = drvHandle->ringGetForwardRingOcc(ringHandleInt);

    return (occ);
}

uint32_t Udma_ringGetReverseRingOcc(Udma_RingHandle ringHandle)
{
    uint32_t            occ = 0U;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    occ = drvHandle->ringGetReverseRingOcc(ringHandleInt);

    return (occ);
}

uint32_t Udma_ringGetWrIdx(Udma_RingHandle ringHandle)
{
    uint32_t            idx = 0U;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    idx = drvHandle->ringGetWrIdx(ringHandleInt);

    return (idx);
}

uint32_t Udma_ringGetRdIdx(Udma_RingHandle ringHandle)
{
    uint32_t            idx = 0U;
    Udma_RingHandleInt  ringHandleInt = (Udma_RingHandleInt) ringHandle;
    Udma_DrvHandleInt   drvHandle = ringHandleInt->drvHandle;

    idx = drvHandle->ringGetRdIdx(ringHandleInt);

    return (idx);
}

void UdmaRingPrms_init(Udma_RingPrms *ringPrms)
{
    if(NULL_PTR != ringPrms)
    {
        ringPrms->ringMem       = NULL_PTR;
        ringPrms->ringMemSize   = UDMA_RING_SIZE_CHECK_SKIP;
        ringPrms->mode          = TISCI_MSG_VALUE_RM_RING_MODE_RING;
        ringPrms->virtId        = UDMA_RING_VIRTID_INVALID;
        ringPrms->elemCnt       = 0U;
        ringPrms->elemSize      = UDMA_RING_ES_8BYTES;
        ringPrms->orderId       = UDMA_DEFAULT_RING_ORDER_ID;
        ringPrms->asel          = UDMA_RINGACC_ASEL_ENDPOINT_PHYSADDR;
        ringPrms->mappedRingGrp = UDMA_MAPPED_GROUP_INVALID;
        ringPrms->mappedChNum   = UDMA_DMA_CH_INVALID;
    }

    return;
}

static int32_t Udma_ringCheckParams(Udma_DrvHandleInt drvHandle,
                                    const Udma_RingPrms *ringPrms)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    ringMemSize;

    DebugP_assert(ringPrms != NULL_PTR);

    if(NULL_PTR == ringPrms->ringMem)
    {
        retVal = UDMA_EINVALID_PARAMS;
        DebugP_logError("[UDMA] Ring memory should not be NULL_PTR!!!\r\n");
    }
    else
    {
        /* Check 128 byte alignment */
        if(((uintptr_t)ringPrms->ringMem & (UDMA_CACHELINE_ALIGNMENT - 1U)) != 0U)
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Ring memory not aligned!!!\r\n");
        }
    }

    if(0U == ringPrms->elemCnt)
    {
        retVal = UDMA_EINVALID_PARAMS;
        DebugP_logError("[UDMA] Ring element count should not be zero!!!\r\n");
    }

    if(UDMA_INST_TYPE_NORMAL != drvHandle->instType)
    {
        if(TISCI_MSG_VALUE_RM_RING_MODE_RING != ringPrms->mode)
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Invalid Ring Mode for LCDMA!!!\r\n");
        }
    }

    if(UDMA_RING_SIZE_CHECK_SKIP != ringPrms->ringMemSize)
    {
        /* Get ring memory size */
        ringMemSize = UdmaUtils_getRingMemSize(
                          ringPrms->mode,
                          ringPrms->elemCnt,
                          ringPrms->elemSize);
        if(ringPrms->ringMemSize < ringMemSize)
        {
            retVal = UDMA_EALLOC;
            DebugP_logError("[UDMA] Ring memory not sufficient!!!\r\n");
        }
    }

    if (UDMA_RING_ORDERID_MAX < ringPrms->orderId)
    {
        retVal = UDMA_EINVALID_PARAMS;
        DebugP_logError("[UDMA] Ring orderId out of range (%u)!!!\r\n", ringPrms->orderId);
    }

    return (retVal);
}

static inline void Udma_ringAssertFnPointers(Udma_DrvHandleInt drvHandle)
{
    DebugP_assert(drvHandle->ringDequeueRaw        != (Udma_ringDequeueRawFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringQueueRaw          != (Udma_ringQueueRawFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringFlushRaw          != (Udma_ringFlushRawFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringGetElementCnt     != (Udma_ringGetElementCntFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringGetMemPtr         != (Udma_ringGetMemPtrFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringGetMode           != (Udma_ringGetModeFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringGetForwardRingOcc != (Udma_ringGetForwardRingOccFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringGetReverseRingOcc != (Udma_ringGetReverseRingOccFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringGetWrIdx          != (Udma_ringGetWrIdxFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringGetRdIdx          != (Udma_ringGetRdIdxFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringPrime             != (Udma_ringPrimeFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringPrimeRead         != (Udma_ringPrimeReadFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringSetDoorBell       != (Udma_ringSetDoorBellFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringSetCfg            != (Udma_ringSetCfgFxn) NULL_PTR);
    DebugP_assert(drvHandle->ringHandleClearRegs   != (Udma_ringHandleClearRegsFxn) NULL_PTR);

    return;
}

int32_t Udma_ringProxyQueueRaw(Udma_RingHandleInt ringHandle,
                               Udma_DrvHandleInt drvHandle,
                               uint64_t phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            ringHwOcc;
    CSL_ProxyThreadCfg  threadCfg;

    /* Get ring occupancy. We should write to FIFO (through proxy) only
     * when there is room available. Otherwise ring will overflow */
    ringHwOcc = CSL_ringaccGetRingHwOcc(
                    &ringHandle->drvHandle->raRegs, ringHandle->ringNum);
    if(ringHwOcc >= ringHandle->cfg.elCnt)
    {
        /* Ring full */
        retVal = UDMA_EFAIL;
    }
    else
    {
        threadCfg.mode      = CSL_PROXY_QUEUE_ACCESS_MODE_TAIL;
        threadCfg.elSz      = (uint32_t)sizeof(uint64_t);
        threadCfg.queueNum  = ringHandle->ringNum;
        threadCfg.errEvtNum = UDMA_EVENT_INVALID;
        retVal = CSL_proxyCfgThread(
                     &drvHandle->proxyCfg,
                     drvHandle->proxyTargetNumRing,
                     drvHandle->rmInitPrms.proxyThreadNum,
                     &threadCfg);
        if(UDMA_SOK == retVal)
        {
            Udma_proxyWrite64(ringHandle->proxyAddr, phyDescMem);
        }
    }

    return (retVal);
}

int32_t Udma_ringProxyDequeueRaw(Udma_RingHandleInt ringHandle,
                                        Udma_DrvHandleInt drvHandle,
                                        uint64_t *phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            ringHwOcc;
    CSL_ProxyThreadCfg  threadCfg;

    /* Get ring occupancy. We should read from FIFO (through proxy) only
     * when something is available. Otherwise ring will underflow */
    ringHwOcc = CSL_ringaccGetRingHwOcc(
                    &ringHandle->drvHandle->raRegs, ringHandle->ringNum);
    if(0U == ringHwOcc)
    {
        /* Nothing to flush */
        retVal = UDMA_ETIMEOUT;
    }
    else
    {
        threadCfg.mode      = CSL_PROXY_QUEUE_ACCESS_MODE_HEAD;
        threadCfg.elSz      = (uint32_t)sizeof(uint64_t);
        threadCfg.queueNum  = ringHandle->ringNum;
        threadCfg.errEvtNum = UDMA_EVENT_INVALID;
        retVal = CSL_proxyCfgThread(
                     &drvHandle->proxyCfg,
                     drvHandle->proxyTargetNumRing,
                     drvHandle->rmInitPrms.proxyThreadNum,
                     &threadCfg);
        if(UDMA_SOK == retVal)
        {
            Udma_proxyRead64(ringHandle->proxyAddr, phyDescMem);
        }
    }

    return (retVal);
}

int32_t Udma_ringReset(Udma_DrvHandleInt drvHandle,
                              Udma_RingHandleInt ringHandle)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    regVal;
    uint32_t    ringOcc, mode, elemSize;
    uint32_t    applyWorkaround = 0U, freeRingStart;
    uint32_t    dbRingCnt, thisDbRingCnt;
    struct tisci_msg_rm_ring_cfg_req    rmRingReq;
    struct tisci_msg_rm_ring_cfg_resp   rmRingResp;

    Udma_assert(drvHandle, drvHandle->raRegs.pGlbRegs != NULL_PTR);
    Udma_assert(drvHandle, ringHandle->pCfgRegs != NULL_PTR);
    Udma_assert(drvHandle, ringHandle->pRtRegs != NULL_PTR);

    /*-------------------------------------------------------------------------
     *  ringacc revision 1.0.19 and earlier has an issue where the UDMAP ring
     *  state (ring occupancy) will be out of sync with the ringacc ring state
     *  when a ring is reset.
     *
     *  Perform the software workaround to correct this.
     *-----------------------------------------------------------------------*/
    regVal = CSL_REG32_RD(&drvHandle->raRegs.pGlbRegs->REVISION);
    if(!((CSL_FEXT(regVal, RINGACC_GCFG_REVISION_REVMAJ) > 1U) ||
         (CSL_FEXT(regVal, RINGACC_GCFG_REVISION_REVMIN) > 0U) ||
         (CSL_FEXT(regVal, RINGACC_GCFG_REVISION_REVRTL) > 19U)))
    {
        /* Workaround required only for FQ ring */
        freeRingStart = drvHandle->udmapRegs.txChanCnt +
                        drvHandle->udmapRegs.txExtUtcChanCnt +
                        drvHandle->udmapRegs.rxChanCnt;
        if(ringHandle->ringNum < freeRingStart)
        {
            applyWorkaround = 1U;
        }
    }

    if(1U == applyWorkaround)
    {
        /* Call SCICLIENT with dummy cfg to get read access to the ring RT
         * By default this is is firewalled even for read access */
        rmRingReq.valid_params  = 0U;
        rmRingReq.nav_id        = drvHandle->devIdRing;
        rmRingReq.index         = ringHandle->ringNum;
        /* Not used */
        rmRingReq.mode          = 0U;
        rmRingReq.size          = 0U;
        rmRingReq.addr_lo       = 0U;
        rmRingReq.addr_hi       = 0U;
        rmRingReq.count         = 0U;
        rmRingReq.order_id      = UDMA_DEFAULT_RING_ORDER_ID;
        retVal = Sciclient_rmRingCfg(
                     &rmRingReq, &rmRingResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] Ring config for read access failed!!!\n");
        }
    }

    if((1U == applyWorkaround) && (UDMA_SOK == retVal))
    {
        /*-------------------------------------------------------------------------
         *  1. Read the ring occupancy.
         *
         *  In the case of a credentials mode or qm mode ring, each ring write
         *  results in the ring occupancy increasing by 2 elements (one entry for
         *  the credentials, one entry for the data). However, the internal UDMAP
         *  ring state occupancy counter only records the number of writes, so we
         *  divide the ring occupancy by 2 to get back to the number of doorbell
         *  rings needed.
         *
         *  If the ring occupancy is not 0, then we need to execute the workaround.
         *-----------------------------------------------------------------------*/
        ringOcc = CSL_REG32_FEXT(&ringHandle->pRtRegs->OCC, RINGACC_RT_RINGRT_OCC_CNT);
        mode = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_QMODE);
        elemSize = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_ELSIZE);
        if((CSL_RINGACC_RING_MODE_CREDENTIALS == mode) ||
           (CSL_RINGACC_RING_MODE_QM == mode))
        {
            ringOcc >>= 1U;
        }
        if(ringOcc != 0U)
        {
            /*---------------------------------------------------------------------
             *  2. Reset the ring by writing to size register
             *  3. At the same time setup the ring in ring/doorbell mode
             *-------------------------------------------------------------------*/
            rmRingReq.valid_params  = TISCI_MSG_VALUE_RM_RING_MODE_VALID |
                                      TISCI_MSG_VALUE_RM_RING_SIZE_VALID;
            rmRingReq.nav_id        = drvHandle->devIdRing;
            rmRingReq.index         = ringHandle->ringNum;
            rmRingReq.mode          = TISCI_MSG_VALUE_RM_RING_MODE_RING;
            rmRingReq.size          = elemSize;
            /* Not used */
            rmRingReq.addr_lo       = 0U;
            rmRingReq.addr_hi       = 0U;
            rmRingReq.count         = 0U;
            rmRingReq.order_id      = UDMA_DEFAULT_RING_ORDER_ID;
            retVal = Sciclient_rmRingCfg(
                         &rmRingReq, &rmRingResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] Ring reset failed!!!\n");
            }
            else
            {
                /*--------------------------------------------------------------
                 *  4. Ring the doorbell 2**22 - ringOcc times. This will wrap
                 *     the internal UDMAP ring state occupancy counter
                 *     (which is 21-bits wide) to 0.
                 *------------------------------------------------------------*/
                dbRingCnt = ((uint32_t)1U << 22) - ringOcc;
                thisDbRingCnt = UDMA_RING_MAX_DB_RING_CNT;
                regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
                while(dbRingCnt != 0U)
                {
                    /*----------------------------------------------------------
                     *  Ring the doorbell with the maximum count each iteration
                     *  if possible to minimize the total # of writes
                     *--------------------------------------------------------*/
                    if(dbRingCnt < UDMA_RING_MAX_DB_RING_CNT)
                    {
                        thisDbRingCnt = dbRingCnt;
                        regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
                    }
                    CSL_REG32_WR(&ringHandle->pRtRegs->DB, regVal);
                    dbRingCnt -= thisDbRingCnt;
                }

                /*--------------------------------------------------------------
                 *  5. Restore the original ring mode (if not ring mode)
                 *------------------------------------------------------------*/
                /* Not done as we will reconfigure the ring after reset step */
            }
        }
    }

    return (retVal);
}