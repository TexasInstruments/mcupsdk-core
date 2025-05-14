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
 *  \file udma_priv.h
 *
 *  \brief UDMA private header file.
 */

#ifndef UDMA_PRIV_H_
#define UDMA_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* Disable logs and assert - make it 1 and rebuild to enable */
#ifdef DebugP_LOG_ENABLED
#undef DebugP_LOG_ENABLED
#endif /* DebugP_LOG_ENABLED */

#define DebugP_LOG_ENABLED 0

#ifdef DebugP_ASSERT_ENABLED
#undef DebugP_ASSERT_ENABLED
#endif /* DebugP_ASSERT_ENABLED */

#define DebugP_ASSERT_ENABLED 0

/* This is needed for memset/memcpy */
#include <string.h>

#include <drivers/udma.h>

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>

#include <drivers/sciclient.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/**
 *  \anchor Udma_InstanceType
 *  \name UDMA Instance Type
 *
 *  UDMA instance Type - UDMA/LCDMA_BCDMA/LCDMA_PKTDMA
 *
 *  @{
 */
/** \brief UDMA instance type*/
#define UDMA_INST_TYPE_NORMAL           (0U)
/** \brief LCDMA - Block Copy DMA instance type*/
#define UDMA_INST_TYPE_LCDMA_BCDMA      (1U)
/** \brief LCDMA - Packet DMA instance type*/
#define UDMA_INST_TYPE_LCDMA_PKTDMA     (2U)
/** \brief Maximum number of UDMA instance types */
#define UDMA_INST_TYPE_MAX              (3U)
/* @} */

/**
 *  \anchor Udma_RingAccType
 *  \name UDMA Ring Accelerator Type
 *
 *  UDMA ring accelerator Type - Normal RA/Lcdma RA
 *
 *  @{
 */
/** \brief Normal RA type*/
#define UDMA_RA_TYPE_NORMAL             (0U)
/** \brief Lcdma RA type*/
#define UDMA_RA_TYPE_LCDMA              (1U)
/** \brief Maximum number of RA types */
#define UDMA_RA_TYPE_MAX                (2U)
/* @} */

#define Udma_assert(drvHandle, cond)

/** \brief Macro used to specify that the thread ID is invalid. */
#define UDMA_THREAD_ID_INVALID          ((uint32_t) 0xFFFF0004U)

/** \brief Macro used to specify that the Sciclient RM resource assignment type is invalid. */
#define UDMA_RM_SCI_REQ_TYPE_INVALID       ((uint16_t) 0xFFFFU)

/** \brief Macro used to specify that the Sciclient RM resource assignment subtype is invalid. */
#define UDMA_RM_SCI_REQ_SUBTYPE_INVALID    ((uint16_t) 0xFFFFU)

/** \brief Macro used to specify shift value for RX flow threshold before passing to SysFw */
#define UDMA_RFLOW_RX_SIZE_THRESH_VAL_SHIFT      ((uint32_t) 0x00000005U)

/** \brief Default ring order ID */
#define UDMA_DEFAULT_RING_ORDER_ID      (0U)

/** \brief Default TX channel DMA priority */
#define UDMA_DEFAULT_TX_CH_DMA_PRIORITY                                     \
                                    (TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_MEDHIGH)
/** \brief Default RX channel DMA priority */
#define UDMA_DEFAULT_RX_CH_DMA_PRIORITY                                     \
                                    (TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_MEDHIGH)

/** \brief Default TX channel bus priority */
#define UDMA_DEFAULT_TX_CH_BUS_PRIORITY (4U)
/** \brief Default RX channel bus priority */
#define UDMA_DEFAULT_RX_CH_BUS_PRIORITY (4U)

/** \brief Default TX channel bus QOS */
#define UDMA_DEFAULT_TX_CH_BUS_QOS      (4U)
/** \brief Default RX channel bus QOS */
#define UDMA_DEFAULT_RX_CH_BUS_QOS      (4U)

/** \brief Default TX channel bus order ID */
#define UDMA_DEFAULT_TX_CH_BUS_ORDERID  (0U)
/** \brief Default RX channel bus order ID */
#define UDMA_DEFAULT_RX_CH_BUS_ORDERID  (0U)

/** \brief SCICLIENT API timeout */
#define UDMA_SCICLIENT_TIMEOUT          (SystemP_WAIT_FOREVER)

/** \brief Macro used to specify that init is performed for an object. */
#define UDMA_INIT_DONE                  (0xABDCABCDU)
/** \brief Macro used to specify that deinit is performed for an object. */
#define UDMA_DEINIT_DONE                (0x00000000U)

/** \brief Macro used to specify that channel teardown status is not available */
#define UDMA_EVENT_CH_TEARDOWN_STATUS_NA           ((uint32_t) 0x0001U)
/** \brief Macro used to specify that channel teardown is complete */
#define UDMA_EVENT_CH_TEARDOWN_STATUS_COMPLETE     ((uint32_t) 0x0002U)
/** \brief Macro used to specify that channel teardown is not complete */
#define UDMA_EVENT_CH_TEARDOWN_STATUS_INCOMPLETE   ((uint32_t) 0x0003U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */



/**
 *  \brief UDMA Sciclient Default BoardCfg RM parameters.
 */
typedef struct
{
    uint32_t                resId;
    /**< UDMA Resource Id \ref Udma_RmResId macros. */
    uint16_t                sciclientReqType;
    /**< Sciclient RM resource assignment type */
    uint16_t                sciclientReqSubtype;
    /**< Sciclient RM resource assignment subtype */
    uint8_t                 sciclientSecHost;
    /**< Sciclient Secondary host */
} Udma_RmDefBoardCfgPrms;

/**
 *  \brief UDMA Sciclient Default BoardCfg RM Response.
 */
typedef struct
{
    uint32_t                resId;
    /**< UDMA Resource Id \ref Udma_RmResId macros. */
    uint16_t                rangeStart;
    /**< Sciclient RM resource reservation start */
    uint16_t                rangeNum;
    /**< Sciclient RM no. of resources reserverd */
    uint16_t                rangeStartSec;
    /**< Sciclient RM resource secondary reservation start */
    uint16_t                rangeNumSec;
    /**< Sciclient RM no. of resources reserverd secondary*/
} Udma_RmDefBoardCfgResp;

/**
 *  \brief UDMA ring monitor object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
struct Udma_RingMonObj
{
    Udma_DrvHandle              drvHandle;
    /**< Pointer to global driver handle. */

    uint16_t                    ringMonNum;
    /**< Ring number */

    uint32_t                    ringMonInitDone;
    /**< Flag to set the ring monitor object is init. */
};

/* ========================================================================== */
/*                         Global Variables                                   */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* SOC APIs */
void Udma_initDrvHandle(Udma_DrvHandle drvHandle);
int32_t UdmaRmInitPrms_init(uint32_t instId, Udma_RmInitPrms *rmInitPrms);
const Udma_RmDefBoardCfgPrms *Udma_rmGetDefBoardCfgPrms(uint32_t instId);
/**
 *  \brief Default Normal RA memory fence API used for CSL-FL to perform cache ops
 *
 *  \param pVirtAddr        [IN]    The virtual memory address written to
 *  \param size             [IN]    Number of bytes to writeback
 *  \param opsType          [IN]    \ref CSL_RingAccMemoryOpsType
 */
void Udma_ringaccMemOps(void *pVirtAddr, uint32_t size, uint32_t opsType);
/* Private APIs */
int32_t Udma_ringReset(Udma_DrvHandle drvHandle,
                       Udma_RingHandle ringHandle);
/* Normal RA APIs*/
void Udma_ringHandleClearRegsNormal(Udma_RingHandle ringHandle);
void Udma_ringSetDoorBellNormal(Udma_RingHandle ringHandle, int32_t count);
void Udma_ringPrimeNormal(Udma_RingHandle ringHandle, uint64_t phyDescMem);
void Udma_ringPrimeReadNormal(Udma_RingHandle ringHandle, uint64_t *phyDescMem);
uint8_t *Udma_ringGetMemPtrNormal(Udma_RingHandle ringHandle);
uint32_t Udma_ringGetModeNormal(Udma_RingHandle ringHandle);
uint32_t Udma_ringGetElementCntNormal(Udma_RingHandle ringHandle);
uint32_t Udma_ringGetRingOccNormal(Udma_RingHandle ringHandle);
uint32_t Udma_ringGetWrIdxNormal(Udma_RingHandle ringHandle);
uint32_t Udma_ringGetRdIdxNormal(Udma_RingHandle ringHandle);
int32_t Udma_ringDequeueRawNormal(Udma_DrvHandle  drvHandle,
                                  Udma_RingHandle ringHandle,
                                  uint64_t *phyDescMem);
int32_t Udma_ringQueueRawNormal(Udma_DrvHandle  drvHandle,
                                Udma_RingHandle ringHandle,
                                uint64_t phyDescMem);
int32_t Udma_ringFlushRawNormal(Udma_DrvHandle  drvHandle,
                                Udma_RingHandle ringHandle,
                                uint64_t *phyDescMem);
void Udma_ringSetCfgNormal(Udma_DrvHandle drvHandle,
                           Udma_RingHandle ringHandle,
                           const Udma_RingPrms *ringPrms);

int32_t Udma_ringProxyQueueRaw(Udma_RingHandle ringHandle,
                               Udma_DrvHandle drvHandle,
                               uint64_t phyDescMem);
int32_t Udma_ringProxyDequeueRaw(Udma_RingHandle ringHandle,
                                 Udma_DrvHandle drvHandle,
                                 uint64_t *phyDescMem);
/*
 * RM APIs
 */
void Udma_rmInit(Udma_DrvHandle drvHandle);
int32_t Udma_rmDeinit(Udma_DrvHandle drvHandle);

/* Channel RM APIs */
uint32_t Udma_rmAllocBlkCopyCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeBlkCopyCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocBlkCopyHcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeBlkCopyHcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocBlkCopyUhcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeBlkCopyUhcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocTxCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeTxCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocRxCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeRxCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocTxHcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeTxHcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocRxHcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeRxHcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocTxUhcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeTxUhcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocRxUhcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeRxUhcCh(uint32_t chNum, Udma_DrvHandle drvHandle);

uint16_t Udma_rmAllocFreeRing(Udma_DrvHandle drvHandle);
void Udma_rmFreeFreeRing(uint16_t ringNum, Udma_DrvHandle drvHandle);

/* Event RM APIs */
uint32_t Udma_rmAllocEvent(Udma_DrvHandle drvHandle);
void Udma_rmFreeEvent(uint32_t globalEvent, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocVintr(Udma_DrvHandle drvHandle);
void Udma_rmFreeVintr(uint32_t vintrNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocVintrBit(Udma_EventHandle eventHandle);
void Udma_rmFreeVintrBit(uint32_t vintrBitNum,
                         Udma_DrvHandle drvHandle,
                         Udma_EventHandle eventHandle);
uint32_t Udma_rmAllocIrIntr(uint32_t preferredIrIntrNum,
                              Udma_DrvHandle drvHandle);
void Udma_rmFreeIrIntr(uint32_t irIntrNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmTranslateIrOutput(Udma_DrvHandle drvHandle, uint32_t irIntrNum);
uint32_t Udma_rmTranslateCoreIntrInput(Udma_DrvHandle drvHandle, uint32_t coreIntrNum);
void Udma_rmFreeCoreIntr(uint32_t coreIntrNum, Udma_DrvHandle drvHandle);

/* Query Sciclient_DefaultBoardCfg_rm API */
int32_t Udma_rmGetSciclientDefaultBoardCfgRmRange(const Udma_RmDefBoardCfgPrms *rmDefBoardCfgPrms,
                                                  Udma_RmDefBoardCfgResp *rmDefBoardCfgResp,
                                                  uint32_t *splitResFlag);
/* Set Shared Resource rmInitPrms API */
int32_t Udma_rmSetSharedResRmInitPrms(const Udma_RmSharedResPrms *rmSharedResPrms,
                                      uint32_t instId,
                                      uint32_t rangeStart,
                                      uint32_t rangeTotalNum,
                                      uint32_t *start,
                                      uint32_t *num);

/* Utils APIs */
uint64_t Udma_virtToPhyFxn(const void *virtAddr,
                           Udma_DrvHandle drvHandle,
                           Udma_ChHandle chHandle);
void *Udma_phyToVirtFxn(uint64_t phyAddr,
                        Udma_DrvHandle drvHandle,
                        Udma_ChHandle chHandle);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_PRIV_H_ */
