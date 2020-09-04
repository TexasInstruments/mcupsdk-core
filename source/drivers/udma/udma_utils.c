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
 *  \file udma_utils.c
 *
 *  \brief File containing the UDMA driver utility functions.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/udma/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    uint32_t trType;
    uint32_t trSize;
    uint32_t trSizeEncoded;
} UdmaUtilsTrSizeTable;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* \brief TR size table */
static UdmaUtilsTrSizeTable gUdmaUtilsTrSizeTable[] =
{
    {UDMA_TR_TYPE_0,  16U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_16B},
    {UDMA_TR_TYPE_1,  32U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_32B},
    {UDMA_TR_TYPE_2,  32U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_32B},
    {UDMA_TR_TYPE_3,  32U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_32B},
    {UDMA_TR_TYPE_4,  64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_5,  64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_8,  64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_9,  64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_10, 64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_11, 64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_15, 64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B}
};

#define UDMA_UTILS_NUM_TR_TYPE          ((sizeof (gUdmaUtilsTrSizeTable)) / \
                                         (sizeof (UdmaUtilsTrSizeTable)))

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

uint32_t UdmaUtils_getRingMemSize(uint8_t mode,
                                  uint32_t elemCnt,
                                  uint8_t elemSize)
{
    uint32_t    ringMemSize;

    ringMemSize = ((uint32_t) 1U << (elemSize + 2U));   /* Element size in bytes */
    ringMemSize *= elemCnt;
    /* In the case of a credentials mode or qm mode, each ring write
     * results in the ring occupancy increasing by 2 elements (one entry for
     * the credentials, one entry for the data) */
    if((TISCI_MSG_VALUE_RM_RING_MODE_CREDENTIALS == mode) ||
       (TISCI_MSG_VALUE_RM_RING_MODE_QM == mode))
    {
        ringMemSize <<= 1U;
    }

    return (ringMemSize);
}

void UdmaUtils_makeTrpdTr15(uint8_t *trpdMem, uint32_t trCnt, uint32_t cqRingNum)
{
    uint32_t    i;
    uint32_t   *pTrResp;

    UdmaUtils_makeTrpd(trpdMem, UDMA_TR_TYPE_15, trCnt, cqRingNum);

    /* Clear TR response memory */
    pTrResp = (uint32_t *) (trpdMem + (sizeof(CSL_UdmapTR15) + (sizeof(CSL_UdmapTR15) * trCnt)));
    for(i = 0U; i < trCnt; i++)
    {
        *pTrResp = 0xFFFFFFFFU;
        pTrResp++;
    }

    return;
}

void UdmaUtils_makeTrpd(uint8_t *trpdMem,
                        uint32_t trType,
                        uint32_t trCnt,
                        uint32_t cqRingNum)
{
    uint32_t descType = CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR;
    uint32_t trSizeEncoded = UdmaUtils_getTrSizeEncoded(trType);

    /* Setup descriptor */
    CSL_udmapCppi5SetDescType(trpdMem, descType);
    CSL_udmapCppi5TrSetReload((CSL_UdmapCppi5TRPD *)trpdMem, 0U, 0U);
    CSL_udmapCppi5SetPktLen(trpdMem, descType, trCnt);
    CSL_udmapCppi5SetIds(trpdMem, descType, 0U, UDMA_DEFAULT_FLOW_ID); /* Flow ID and Packet ID */
    CSL_udmapCppi5SetSrcTag(trpdMem, 0x0000);
    CSL_udmapCppi5SetDstTag(trpdMem, 0x0000);
    CSL_udmapCppi5TrSetEntryStride((CSL_UdmapCppi5TRPD *)trpdMem, trSizeEncoded);
    /* Return Policy descriptors are reserved in case of AM243X/Am64X */
    CSL_udmapCppi5SetReturnPolicy(
        trpdMem,
        descType,
        0U,
        0U,
        0U,
        0U);

    return;
}

uint32_t UdmaUtils_getTrSizeEncoded(uint32_t trType)
{
    uint32_t i, trSizeEncoded = CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B;

    for(i=0; i<UDMA_UTILS_NUM_TR_TYPE; i++)
    {
        if(gUdmaUtilsTrSizeTable[i].trType == trType)
        {
            trSizeEncoded = gUdmaUtilsTrSizeTable[i].trSizeEncoded;
            break;
        }
    }

    return (trSizeEncoded);
}

uint32_t UdmaUtils_getTrSizeBytes(uint32_t trType)
{
    uint32_t i, trSize = 64U;

    for(i=0; i<UDMA_UTILS_NUM_TR_TYPE; i++)
    {
        if(gUdmaUtilsTrSizeTable[i].trType == trType)
        {
            trSize = gUdmaUtilsTrSizeTable[i].trSize;
            break;
        }
    }

    return (trSize);
}

uint64_t Udma_virtToPhyFxn(const void *virtAddr,
                           Udma_DrvHandleInt drvHandle,
                           Udma_ChHandleInt chHandle)
{
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    void               *appData = NULL_PTR;
    uint64_t            phyAddr;

    if(NULL_PTR != chHandle)
    {
        chNum   = chHandle->chPrms.chNum;
        appData = chHandle->chPrms.appData;
    }

    if((Udma_VirtToPhyFxn) NULL_PTR != drvHandle->initPrms.virtToPhyFxn)
    {
        phyAddr = drvHandle->initPrms.virtToPhyFxn(virtAddr, chNum, appData);
    }
    else
    {
        phyAddr = Udma_defaultVirtToPhyFxn(virtAddr, chNum, appData);
    }

    return (phyAddr);
}

void *Udma_phyToVirtFxn(uint64_t phyAddr,
                        Udma_DrvHandleInt drvHandle,
                        Udma_ChHandleInt chHandle)
{
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    void               *appData = NULL_PTR;
    void               *virtAddr;

    if(NULL_PTR != chHandle)
    {
        chNum   = chHandle->chPrms.chNum;
        appData = chHandle->chPrms.appData;
    }

    if((Udma_VirtToPhyFxn) NULL_PTR != drvHandle->initPrms.virtToPhyFxn)
    {
        virtAddr = drvHandle->initPrms.phyToVirtFxn(phyAddr, chNum, appData);
    }
    else
    {
        virtAddr = Udma_defaultPhyToVirtFxn(phyAddr, chNum, appData);
    }

    return (virtAddr);
}

uint64_t Udma_defaultVirtToPhyFxn(const void *virtAddr,
                                  uint32_t chNum,
                                  void *appData)
{
    return ((uint64_t) virtAddr);
}

void *Udma_defaultPhyToVirtFxn(uint64_t phyAddr,
                               uint32_t chNum,
                               void *appData)
{
#if defined (__aarch64__)
    uint64_t temp = phyAddr;
#else
    /* R5 is 32-bit machine, need to truncate to avoid void * typecast error */
    uint32_t temp = (uint32_t) phyAddr;
#endif

    return ((void *) temp);
}
