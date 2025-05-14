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

/** \brief Max number of door bell ring that can be performed at one go */
#define UDMA_RING_MAX_DB_RING_CNT       (127U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t Udma_ringCheckParams(Udma_DrvHandle drvHandle,
                                    const Udma_RingPrms *ringPrms);

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
    Udma_DrvHandle   drvHandleInt = (Udma_DrvHandle) drvHandle;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;
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
#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
            else
            {
                ringHandleInt->ringNum = Udma_rmAllocMappedRing(drvHandleInt, ringPrms->mappedRingGrp, ringPrms->mappedChNum);
            }
#endif
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
        ringHandleInt->drvHandle = drvHandleInt;
        /* Set the mapped group in ringHandleInt, since only ringHandleInt is passed to rmFreeMappedRing() and
         * the mapped group parameter is required to reset the appropriate flag */
        ringHandleInt->mappedRingGrp   = ringPrms->mappedRingGrp;
        ringHandleInt->mappedChNum     = ringPrms->mappedChNum;
        Udma_ringSetCfgLcdma(drvHandleInt, ringHandleInt, ringPrms);
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
        physBase = Udma_virtToPhyFxn(ringPrms->ringMem, drvHandleInt, (Udma_ChHandle) NULL_PTR);
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
#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
            else
            {
                Udma_rmFreeMappedRing(ringHandleInt->ringNum, drvHandleInt, ringHandleInt->mappedRingGrp, ringHandleInt->mappedChNum);
            }
#endif
        }
    }

    return (retVal);
}

int32_t Udma_ringFree(Udma_RingHandle ringHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandle   drvHandle;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

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
#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
        else
        {
            Udma_rmFreeMappedRing(
                ringHandleInt->ringNum,
                drvHandle,
                ringHandleInt->mappedRingGrp,
                ringHandleInt->mappedChNum);
        }
#endif
        ringHandleInt->ringNum         = UDMA_RING_INVALID;
        ringHandleInt->ringInitDone    = UDMA_DEINIT_DONE;
        Udma_ringHandleClearRegsLcdma(ringHandleInt);
        ringHandleInt->drvHandle       = (Udma_DrvHandle) NULL_PTR;
    }

    return (retVal);
}

int32_t Udma_ringAttach(Udma_DrvHandle drvHandle,
                        Udma_RingHandle ringHandle,
                        uint16_t ringNum)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandle   drvHandleInt = (Udma_DrvHandle) drvHandle;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

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
        Udma_ringSetCfgLcdma(drvHandleInt, ringHandleInt, (Udma_RingPrms *) NULL_PTR);
        ringHandleInt->ringInitDone = UDMA_INIT_DONE;
    }

    return (retVal);
}

int32_t Udma_ringDetach(Udma_RingHandle ringHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandle   drvHandle;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

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
        Udma_ringHandleClearRegsLcdma(ringHandleInt);
        ringHandleInt->drvHandle       = (Udma_DrvHandle) NULL_PTR;

    }

    return (retVal);
}

int32_t Udma_ringQueueRaw(Udma_RingHandle ringHandle, uint64_t phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    uintptr_t           cookie;
    Udma_DrvHandle   drvHandle;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

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

        retVal = Udma_ringQueueRawLcdma(drvHandle, ringHandleInt, phyDescMem);

        HwiP_restore(cookie);
    }

    return (retVal);
}

int32_t Udma_ringDequeueRaw(Udma_RingHandle ringHandle, uint64_t *phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    uintptr_t           cookie;
    Udma_DrvHandle   drvHandle;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

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

        retVal = Udma_ringDequeueRawLcdma(drvHandle, ringHandleInt, phyDescMem);

        HwiP_restore(cookie);
    }

    return (retVal);
}

int32_t Udma_ringFlushRaw(Udma_RingHandle ringHandle, uint64_t *phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandle   drvHandle;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

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
        retVal = Udma_ringFlushRawLcdma(drvHandle, ringHandleInt, phyDescMem);
    }

    return (retVal);
}

void Udma_ringPrime(Udma_RingHandle ringHandle, uint64_t phyDescMem)
{
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    Udma_ringPrimeLcdma(ringHandleInt, phyDescMem);

    return;
}

void Udma_ringPrimeRead(Udma_RingHandle ringHandle, uint64_t *phyDescMem)
{
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    Udma_ringPrimeReadLcdma(ringHandleInt, phyDescMem);

    return;
}

void Udma_ringSetDoorBell(Udma_RingHandle ringHandle, int32_t count)
{
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    Udma_ringSetDoorBellLcdma(ringHandleInt, count);

    return;
}

uint16_t Udma_ringGetNum(Udma_RingHandle ringHandle)
{
    uint16_t            ringNum = UDMA_RING_INVALID;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    if((NULL_PTR != ringHandleInt) &&
       (UDMA_INIT_DONE == ringHandleInt->ringInitDone))
    {
        ringNum = ringHandleInt->ringNum;
    }

    return (ringNum);
}

uint8_t *Udma_ringGetMemPtr(Udma_RingHandle ringHandle)
{
    uint8_t             *ringMem = NULL_PTR;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    ringMem = Udma_ringGetMemPtrLcdma(ringHandleInt);

    return (ringMem);
}

uint32_t Udma_ringGetMode(Udma_RingHandle ringHandle)
{
    uint32_t            ringMode;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    ringMode = Udma_ringGetModeLcdma(ringHandleInt);

    return (ringMode);
}

uint32_t Udma_ringGetElementCnt(Udma_RingHandle ringHandle)
{
    uint32_t            size = 0U;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    size = Udma_ringGetElementCntLcdma(ringHandleInt);

    return (size);
}

uint32_t Udma_ringGetForwardRingOcc(Udma_RingHandle ringHandle)
{
    uint32_t            occ = 0U;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    occ = Udma_ringGetForwardRingOccLcdma(ringHandleInt);

    return (occ);
}

uint32_t Udma_ringGetReverseRingOcc(Udma_RingHandle ringHandle)
{
    uint32_t            occ = 0U;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    occ = Udma_ringGetReverseRingOccLcdma(ringHandleInt);

    return (occ);
}

uint32_t Udma_ringGetWrIdx(Udma_RingHandle ringHandle)
{
    uint32_t            idx = 0U;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    idx = Udma_ringGetWrIdxLcdma(ringHandleInt);

    return (idx);
}

uint32_t Udma_ringGetRdIdx(Udma_RingHandle ringHandle)
{
    uint32_t            idx = 0U;
    Udma_RingHandle  ringHandleInt = (Udma_RingHandle) ringHandle;

    idx = Udma_ringGetRdIdxLcdma(ringHandleInt);

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

static int32_t Udma_ringCheckParams(Udma_DrvHandle drvHandle,
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

#if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
    if((UDMA_MAPPED_GROUP_INVALID != ringPrms->mappedRingGrp) &&
       (ringPrms->mappedRingGrp >= (UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP)))
    {
        retVal = UDMA_EINVALID_PARAMS;
        DebugP_logError("[UDMA] Incorrect Mapped Ring Group!!!\r\n");
    }
#endif

    return (retVal);
}
