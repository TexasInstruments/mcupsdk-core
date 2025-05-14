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

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/SemaphoreP.h>

#if (UDMA_SOC_CFG_RA_LCDMA_PRESENT == 1)
#include <drivers/udma/hw_include/csl_lcdma_ringacc.h>
#endif
#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
#include <drivers/udma/hw_include/csl_bcdma.h>
#include <drivers/udma/hw_include/csl_pktdma.h>
#endif
#include <drivers/udma/hw_include/csl_intaggr.h>

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

/** \brief Macro used to specify that the thread ID is invalid. */
#define UDMA_THREAD_ID_INVALID          ((uint32_t) 0xFFFF0004U)

/** \brief Macro used to specify that the Sciclient RM resource assignment type is invalid. */
#define UDMA_RM_SCI_REQ_TYPE_INVALID       ((uint16_t) 0xFFFFU)

/** \brief Macro used to specify that the Sciclient RM resource assignment subtype is invalid. */
#define UDMA_RM_SCI_REQ_SUBTYPE_INVALID    ((uint16_t) 0xFFFFU)

/** \brief Macro used to specify shift value for RX flow threshold before passing to SysFw */
#define UDMA_RFLOW_RX_SIZE_THRESH_VAL_SHIFT      ((uint32_t) 0x00000005U)

/** \brief UDMA driver handle */
typedef struct Udma_DrvObjectInt_t     *Udma_DrvHandleInt;
/** \brief UDMA channel handle */
typedef struct Udma_ChObjectInt_t      *Udma_ChHandleInt;
/** \brief UDMA event handle */
typedef struct Udma_EventObjectInt_t   *Udma_EventHandleInt;
/** \brief UDMA ring handle */
typedef struct Udma_RingObjectInt_t    *Udma_RingHandleInt;
/** \brief UDMA flow handle */
typedef struct Udma_FlowObjectInt_t    *Udma_FlowHandleInt;

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

#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
/**
 *  \brief UDMA mapped channel ring attributes.
 */
typedef struct
{
    uint32_t    defaultRing;
    /**< Default ring number of a particular mapped channel. */
    uint32_t    startFreeRing;
    /**< Start free ring number of a particular mapped channel. */
    uint32_t    numFreeRing;
    /**< Number of free rings for a particular mapped channel. */
} Udma_MappedChRingAttributes;
#endif

/* ========================================================================== */
/*                         Global Variables                                   */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* SOC APIs */
void Udma_initDrvHandle(Udma_DrvHandleInt drvHandle);
int32_t UdmaRmInitPrms_init(uint32_t instId, Udma_RmInitPrms *rmInitPrms);
const Udma_RmDefBoardCfgPrms *Udma_rmGetDefBoardCfgPrms(uint32_t instId);
#if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
int32_t Udma_getMappedChRingAttributes(Udma_DrvHandleInt drvHandle,
                                       uint32_t mappedGrp,
                                       uint32_t chNum,
                                       Udma_MappedChRingAttributes *chAttr);
#endif

#if (UDMA_SOC_CFG_RA_LCDMA_PRESENT == 1)
/* LCDMA RA APIs*/
void Udma_lcdmaRingaccMemOps(void *pVirtAddr, uint32_t size, uint32_t opsType);
void Udma_ringHandleClearRegsLcdma(Udma_RingHandleInt ringHandle);
void Udma_ringSetDoorBellLcdma(Udma_RingHandleInt ringHandle, int32_t count);
void Udma_ringPrimeLcdma(Udma_RingHandleInt ringHandle, uint64_t phyDescMem);
void Udma_ringPrimeReadLcdma(Udma_RingHandleInt ringHandle, uint64_t *phyDescMem);
void *Udma_ringGetMemPtrLcdma(Udma_RingHandleInt ringHandle);
uint32_t Udma_ringGetModeLcdma(Udma_RingHandleInt ringHandle);
uint32_t Udma_ringGetElementCntLcdma(Udma_RingHandleInt ringHandle);
uint32_t Udma_ringGetForwardRingOccLcdma(Udma_RingHandleInt ringHandle);
uint32_t Udma_ringGetReverseRingOccLcdma(Udma_RingHandleInt ringHandle);
uint32_t Udma_ringGetWrIdxLcdma(Udma_RingHandleInt ringHandle);
uint32_t Udma_ringGetRdIdxLcdma(Udma_RingHandleInt ringHandle);
int32_t Udma_ringDequeueRawLcdma(Udma_DrvHandleInt  drvHandle,
                                 Udma_RingHandleInt ringHandle,
                                 uint64_t *phyDescMem);
int32_t Udma_ringQueueRawLcdma(Udma_DrvHandleInt  drvHandle,
                               Udma_RingHandleInt ringHandle,
                               uint64_t phyDescMem);
int32_t Udma_ringFlushRawLcdma(Udma_DrvHandleInt  drvHandle,
                               Udma_RingHandleInt ringHandle,
                               uint64_t *phyDescMem);
void Udma_ringSetCfgLcdma(Udma_DrvHandleInt drvHandle,
                          Udma_RingHandleInt ringHandle,
                          const Udma_RingPrms *ringPrms);
#endif

/*
 * RM APIs
 */
void Udma_rmInit(Udma_DrvHandleInt drvHandle);
int32_t Udma_rmDeinit(Udma_DrvHandleInt drvHandle);

/* Channel RM APIs */
uint32_t Udma_rmAllocBlkCopyCh(uint32_t preferredChNum, Udma_DrvHandleInt drvHandle);
void Udma_rmFreeBlkCopyCh(uint32_t chNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocBlkCopyHcCh(uint32_t preferredChNum, Udma_DrvHandleInt drvHandle);
void Udma_rmFreeBlkCopyHcCh(uint32_t chNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocBlkCopyUhcCh(uint32_t preferredChNum, Udma_DrvHandleInt drvHandle);
void Udma_rmFreeBlkCopyUhcCh(uint32_t chNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocTxCh(uint32_t preferredChNum, Udma_DrvHandleInt drvHandle);
void Udma_rmFreeTxCh(uint32_t chNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocRxCh(uint32_t preferredChNum, Udma_DrvHandleInt drvHandle);
void Udma_rmFreeRxCh(uint32_t chNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocTxHcCh(uint32_t preferredChNum, Udma_DrvHandleInt drvHandle);
void Udma_rmFreeTxHcCh(uint32_t chNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocRxHcCh(uint32_t preferredChNum, Udma_DrvHandleInt drvHandle);
void Udma_rmFreeRxHcCh(uint32_t chNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocTxUhcCh(uint32_t preferredChNum, Udma_DrvHandleInt drvHandle);
void Udma_rmFreeTxUhcCh(uint32_t chNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocRxUhcCh(uint32_t preferredChNum, Udma_DrvHandleInt drvHandle);
void Udma_rmFreeRxUhcCh(uint32_t chNum, Udma_DrvHandleInt drvHandle);
#if (UDMA_NUM_MAPPED_TX_GROUP > 0)
uint32_t Udma_rmAllocMappedTxCh(uint32_t preferredChNum,
                                Udma_DrvHandleInt drvHandle,
                                const uint32_t mappedChGrp);
void Udma_rmFreeMappedTxCh(uint32_t chNum,
                           Udma_DrvHandleInt drvHandle,
                           const uint32_t mappedChGrp);
#endif
#if (UDMA_NUM_MAPPED_RX_GROUP > 0)
uint32_t Udma_rmAllocMappedRxCh(uint32_t preferredChNum,
                                Udma_DrvHandleInt drvHandle,
                                const uint32_t mappedChGrp);
void Udma_rmFreeMappedRxCh(uint32_t chNum,
                           Udma_DrvHandleInt drvHandle,
                           const uint32_t mappedChGrp);
#endif

/* Ring RM APIs */
#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
uint32_t Udma_rmAllocMappedRing(Udma_DrvHandleInt drvHandle,
                                const uint32_t mappedRingGrp,
                                const uint32_t mappedChNum);
void Udma_rmFreeMappedRing(uint32_t ringNum,
                           Udma_DrvHandleInt drvHandle,
                           const uint32_t mappedRingGrp,
                           const uint32_t mappedChNum);
#endif
uint16_t Udma_rmAllocFreeRing(Udma_DrvHandleInt drvHandle);
void Udma_rmFreeFreeRing(uint16_t ringNum, Udma_DrvHandleInt drvHandle);

/* Event RM APIs */
uint32_t Udma_rmAllocEvent(Udma_DrvHandleInt drvHandle);
void Udma_rmFreeEvent(uint32_t globalEvent, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocVintr(Udma_DrvHandleInt drvHandle);
void Udma_rmFreeVintr(uint32_t vintrNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmAllocVintrBit(Udma_EventHandleInt eventHandle);
void Udma_rmFreeVintrBit(uint32_t vintrBitNum,
                         Udma_DrvHandleInt drvHandle,
                         Udma_EventHandleInt eventHandle);
uint32_t Udma_rmAllocIrIntr(uint32_t preferredIrIntrNum,
                              Udma_DrvHandleInt drvHandle);
void Udma_rmFreeIrIntr(uint32_t irIntrNum, Udma_DrvHandleInt drvHandle);
uint32_t Udma_rmTranslateIrOutput(Udma_DrvHandleInt drvHandle, uint32_t irIntrNum);
uint32_t Udma_rmTranslateCoreIntrInput(Udma_DrvHandleInt drvHandle, uint32_t coreIntrNum);
void Udma_rmFreeCoreIntr(uint32_t coreIntrNum, Udma_DrvHandleInt drvHandle);

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
                           Udma_DrvHandleInt drvHandle,
                           Udma_ChHandleInt chHandle);
void *Udma_phyToVirtFxn(uint64_t phyAddr,
                        Udma_DrvHandleInt drvHandle,
                        Udma_ChHandleInt chHandle);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_PRIV_H_ */
