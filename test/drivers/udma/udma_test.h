/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 *  \file udma_test.h
 *
 *  \brief This file contains all the structures, macros, enums
 *  used by the UDMA test application.
 *
 */

#ifndef UDMA_TEST_H_
#define UDMA_TEST_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <drivers/udma.h>
#include <drivers/udma/udma_priv.h>

#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>

#include "utils_mem.h"
#include "utils_prf.h"
#include "utils_trace.h"
#include "udma_test_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Log enable for example application. */
#define UdmaUtTrace                     (GT_INFO1 | GT_TraceState_Enable)

/* UART read timeout in msec */
#define UDMA_TEST_UART_TIMEOUT_MSEC     (10000U)

/* Application name string used in print statements */
#define APP_NAME                        "UDMA_TEST"

/* Default values */
#define UDMA_TEST_USE_DEF               (0xFAFAU)
#define UDMA_TEST_DEF_LOOP_CNT          (100U)
#define UDMA_TEST_DEF_QDEPTH            (1U)
#define UDMA_TEST_PERF_LOOP_CNT         (10000U)
#define UDMA_TEST_PERF_QDEPTH           (1U)

#define UDMA_TEST_DEF_ICNT0             (1000U)
#define UDMA_TEST_DEF_DICNT0            (1000U)

#define UDMA_TEST_PERF_ICNT0            (10000U)
#define UDMA_TEST_PERF_DICNT0           (10000U)

#define UDMA_TEST_PRINT_BUFSIZE         (0x4000U)

#define UDMA_TEST_MAX_TASKS             (10U)
#define UDMA_TEST_MAX_CH                (15U)
#define UDMA_TEST_MAX_QDEPTH            (500U)

#define UDMA_TEST_MAX_ICNT              (4U)
#define UDMA_TEST_MAX_DIM               (3U)

/* Defined the following so that it is easy to understand a particular config */
#define USE_DEF_LP_CNT                  (UDMA_TEST_USE_DEF)
#define USE_DEF_QDEPTH                  (UDMA_TEST_USE_DEF)
#define DEF_HEAP_ID                     (UTILS_MEM_HEAP_ID_DDR)
#define TEST_ENABLE                     (TRUE)
#define TEST_DISABLE                    (FALSE)
#define PRF_ENABLE                      (TRUE)
#define PRF_DISABLE                     (FALSE)
#define PRINT_ENABLE                    (TRUE)
#define PRINT_DISABLE                   (FALSE)
#define TIMEST_ENABLE                   (TRUE)
#define TIMEST_DISABLE                  (FALSE)
#define DATA_CHECK_ENABLE               (TRUE)
#define DATA_CHECK_DISABLE              (FALSE)
#define PACING_NONE                     (0U)
#define DEF_PACING                      (20U)

#define UDMA_TEST_RING_ACC_DIRECTION_FORWARD    (0U)
#define UDMA_TEST_RING_ACC_DIRECTION_REVERSE    (1U)

#define UDMA_TEST_INST_ID_MAIN_0        (UDMA_INST_ID_0)
#define UDMA_TEST_INST_ID_MCU_0         (UDMA_INST_ID_1)
#define UDMA_TEST_INST_ID_BCDMA_0       (UDMA_INST_ID_2)
#define UDMA_TEST_INST_ID_PKTDMA_0      (UDMA_INST_ID_3)

#define UDMA_TEST_RF_SOC_AM65XX         ((uint64_t) 0x00000001U)
#define UDMA_TEST_RF_SOC_J721E          ((uint64_t) 0x00000002U)
#define UDMA_TEST_RF_SOC_J7200          ((uint64_t) 0x00000004U)
#define UDMA_TEST_RF_SOC_AM64X          ((uint64_t) 0x00000008U)
#define UDMA_TEST_RF_SOC_ALL            ((uint64_t) 0xFFFFFFFFU)

#define UDMA_TEST_RF_CORE_MPU1_0        ((uint64_t)(((uint64_t) 0x0001U) << 32U))
#define UDMA_TEST_RF_CORE_MCU1_0        ((uint64_t)(((uint64_t) 0x0002U) << 32U))
#define UDMA_TEST_RF_CORE_MCU1_1        ((uint64_t)(((uint64_t) 0x0004U) << 32U))
#define UDMA_TEST_RF_CORE_MCU2_0        ((uint64_t)(((uint64_t) 0x0008U) << 32U))
#define UDMA_TEST_RF_CORE_MCU2_1        ((uint64_t)(((uint64_t) 0x0010U) << 32U))
#define UDMA_TEST_RF_CORE_MCU3_0        ((uint64_t)(((uint64_t) 0x0020U) << 32U))
#define UDMA_TEST_RF_CORE_MCU3_1        ((uint64_t)(((uint64_t) 0x0040U) << 32U))
#define UDMA_TEST_RF_CORE_C7X_1         ((uint64_t)(((uint64_t) 0x0080U) << 32U))
#define UDMA_TEST_RF_CORE_C66X_1        ((uint64_t)(((uint64_t) 0x0100U) << 32U))
#define UDMA_TEST_RF_CORE_C66X_2        ((uint64_t)(((uint64_t) 0x0200U) << 32U))
#define UDMA_TEST_RF_CORE_M4F_0         ((uint64_t)(((uint64_t) 0x0400U) << 32U))
#define UDMA_TEST_RF_CORE_ALL           ((uint64_t)(((uint64_t) 0xFFFFU) << 32U))
#define UDMA_TEST_RF_CORE_MCU_ALL       (UDMA_TEST_RF_CORE_MCU1_0 | UDMA_TEST_RF_CORE_MCU1_1 | \
                                         UDMA_TEST_RF_CORE_MCU2_0 | UDMA_TEST_RF_CORE_MCU2_1 | \
                                         UDMA_TEST_RF_CORE_MCU3_0 | UDMA_TEST_RF_CORE_MCU3_1)

/* For future when we have dynamic coverage testcases */
#define UDMA_TEST_RF_CFG_DEF            ((uint64_t)(((uint64_t) 0x00001U) << 48U))
#define UDMA_TEST_RF_CFG_DYN            ((uint64_t)(((uint64_t) 0x00002U) << 48U))

/**
 *  \brief Test types - based on this the different application flow will be
 *  determined.
 */
typedef enum
{
    UDMA_TT_BLK_CPY,
    UDMA_TT_DRU_DIRECT,
    UDMA_TT_DRU_INDIRECT,
    UDMA_TT_PDMA_UART,
    UDMA_TT_PDMA_OSPI,
    UDMA_TT_PDMA_MCSPI,
    UDMA_TT_PDMA_MCASP,
    UDMA_TT_PDMA_CRC,
    UDMA_TT_PDMA_ADC,
    UDMA_TT_MISC,
    UDMA_TT_CUSTOM
} UdmaTestType;

/**
 *  \brief Testcase types.
 */
typedef enum
{
    /* Category */
    UDMA_TCT_SANITY     = 0x01U,
    UDMA_TCT_REGRESSION = 0x02U,
    UDMA_TCT_FULL       = 0x04U,
    /* Adequacy */
    UDMA_TCT_FUNCTIONAL  = 0x08U,
    UDMA_TCT_STRESS      = 0x10U,
    UDMA_TCT_NEGATIVE    = 0x20U,
    UDMA_TCT_PERFORMANCE = 0x40U,
    UDMA_TCT_MISC        = 0x80U,
    UDMA_TCT_API         = 0x100U,
    /* Used for Test parser dont use in test case */
    UDMA_TCT_ALL = 0x1FFU
} UdmaTestCaseType;

/**
 *  \brief Event types.
 */
typedef enum
{
    UDMA_TEST_EVENT_NONE,
    /**< No events are used - use direct dequeue API to poll for completion */
    UDMA_TEST_EVENT_INTR,
    /**< Events with interrupt callback - use sempahore post for completion */
    UDMA_TEST_EVENT_POLLED,
    /**< Events with polled mode at IA - use IA status poll for completion */
} UdmaTestEventMode;

/**
 *  UDMA UT TX param IDs.
 */
typedef enum
{
    UDMA_TEST_TXCH_PRMID_DEF,
    UDMA_TEST_TXCH_PRMID_DMA_PRIORITY_HIGH,
    UDMA_TEST_TXCH_PRMID_INVALID,
} UdmaTestTxChPrmId;

/**
 *  UDMA UT RX parameter IDs.
 */
typedef enum
{
    UDMA_TEST_RXCH_PRMID_DEF,
    UDMA_TEST_RXCH_PRMID_DMA_PRIORITY_HIGH,
    UDMA_TEST_RXCH_PRMID_INVALID,
} UdmaTestRxChPrmId;

/**
 *  UDMA UT UTC parameter IDs.
 */
typedef enum
{
    UDMA_TEST_UTCCH_PRMID_DEF,
    UDMA_TEST_UTCCH_PRMID_DMA_PRIORITY_HIGH,
    UDMA_TEST_UTCCH_PRMID_INVALID,
} UdmaTestUtcChPrmId;

/**
 *  UDMA UT PDMA parameter IDs.
 */
typedef enum
{
    UDMA_TEST_PDMACH_PRMID_DEF,
    UDMA_TEST_PDMACH_PRMID_ES_16BITS,
    UDMA_TEST_PDMACH_PRMID_INVALID,
} UdmaTestPdmaChPrmId;

/**
 *  UDMA UT channel param IDs.
 */
typedef enum
{
    UDMA_TEST_CH_PRMID_DEF,
    UDMA_TEST_CH_PRMID_INTR_DEF,
    UDMA_TEST_CH_PRMID_TRIGGER_GLOBAL0,
    UDMA_TEST_CH_PRMID_TRIGGER_GLOBAL0_INTR,
    UDMA_TEST_CH_PRMID_EVENTSIZE_ICNT1,
    UDMA_TEST_CH_PRMID_EVENTSIZE_ICNT2,
    UDMA_TEST_CH_PRMID_EVENTSIZE_ICNT3,
    UDMA_TEST_CH_PRMID_DRU_DEF,
    UDMA_TEST_CH_PRMID_DRU_INTR_DEF,
    UDMA_TEST_CH_PRMID_DRU_TRIGGER_GLOBAL0,
    UDMA_TEST_CH_PRMID_DRU_TRIGGER_GLOBAL0_INTR,
    UDMA_TEST_CH_PRMID_DRU_EVENTSIZE_ICNT1,
    UDMA_TEST_CH_PRMID_DRU_EVENTSIZE_ICNT2,
    UDMA_TEST_CH_PRMID_DRU_EVENTSIZE_ICNT3,
    UDMA_TEST_CH_PRMID_BLKCPY_HC_DEF,
    UDMA_TEST_CH_PRMID_BLKCPY_HC_INTR_DEF,
    UDMA_TEST_CH_PRMID_BLKCPY_UHC_DEF,
    UDMA_TEST_CH_PRMID_BLKCPY_UHC_INTR_DEF,
    UDMA_TEST_CH_PRMID_INVALID,
} UdmaTestChPrmId;

/**
 *  UDMA UT ring param IDs.
 */
typedef enum
{
    UDMA_TEST_RING_PRMID_EVENT_NONE,
    UDMA_TEST_RING_PRMID_EVENT_INTR,
    UDMA_TEST_RING_PRMID_EVENT_POLLED,
    UDMA_TEST_RING_PRMID_INVALID,
} UdmaTestRingPrmId;

/**
 *  UDMA UT PKTDMA channel param IDs.
 */
typedef enum
{
    UDMA_TEST_PKTDMA_CH_PRMID_UNMAPPED_TX,
    UDMA_TEST_PKTDMA_CH_PRMID_CPSW_TX,
    UDMA_TEST_PKTDMA_CH_PRMID_SAUL_TX,
    UDMA_TEST_PKTDMA_CH_PRMID_ICSSG_0_TX,
    UDMA_TEST_PKTDMA_CH_PRMID_ICSSG_1_TX,
    UDMA_TEST_PKTDMA_CH_PRMID_UNMAPPED_RX,
    UDMA_TEST_PKTDMA_CH_PRMID_CPSW_RX,
    UDMA_TEST_PKTDMA_CH_PRMID_SAUL_RX,
    UDMA_TEST_PKTDMA_CH_PRMID_ICSSG_0_RX,
    UDMA_TEST_PKTDMA_CH_PRMID_ICSSG_1_RX,
} UdmaTestPktdmaChPrmId;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Forward declaration. */
typedef struct UdmaTestSystemCtrl_t UdmaTestSystemCtrl;
typedef struct UdmaTestParams_t     UdmaTestParams;
typedef struct UdmaTestTaskObj_t    UdmaTestTaskObj;
typedef struct UdmaTestObj_t        UdmaTestObj;

/** \brief Typedef for test case type function pointer */
typedef int32_t (*UdmaTestFxnPtr)(UdmaTestTaskObj *taskObj);

/**
 *  \brief Test parameters for a TX channel.
 */
typedef struct
{
    UdmaTestTxChPrmId       txChPrmId;
    /**< TX Channel parameter ID. */
    Udma_ChTxPrms           txPrms;
    /**< TX channel parameter */
} UdmaTestTxChPrm;

/**
 *  \brief Test parameters for a RX channel.
 */
typedef struct
{
    UdmaTestRxChPrmId       rxChPrmId;
    /**< RX Channel parameter ID. */
    Udma_ChRxPrms           rxPrms;
    /**< RX channel parameter */
} UdmaTestRxChPrm;

/**
 *  \brief Test parameters for a PDMA channel.
 */
typedef struct
{
    UdmaTestPdmaChPrmId     pdmaChPrmId;
    /**< PDMA Channel parameter ID. */
    Udma_ChPdmaPrms         pdmaPrms;
    /**< PDMA channel parameter */
} UdmaTestPdmaChPrm;

/**
 *  \brief Test parameters for a channel.
 */
typedef struct
{
    UdmaTestChPrmId         chPrmId;
    /**< Channel parameter ID. */
    uint32_t                chType;
    /**< UDMA channel type. */
    uint32_t                eventMode;
    /**< Use no event, event with interrupt or event with polled mode.
     *   Refer #UdmaTestEventMode */
    uint32_t                trigger;
    /**< Global0 or Global 1 Trigger - refer \ref CSL_UdmapTrFlagsTrigger. */
    uint32_t                eventSize;
    /**< Refer \ref CSL_UdmapTrFlagsEventSize. */
    uint32_t                triggerType;
    /**< Refer \ref CSL_UdmapTrFlagsTriggerType. */
    UdmaTestTxChPrmId       txPrmId;
    /**< TX Channel parameter ID. */
    UdmaTestRxChPrmId       rxPrmId;
    /**< RX Channel parameter ID. */
    UdmaTestUtcChPrmId      utcPrmId;
    /**< UTC Channel parameter ID. */
    UdmaTestPdmaChPrmId     pdmaPrmId;
    /**< PDMA Channel parameter ID. */
} UdmaTestChPrm;

/**
 *  \brief Test parameters for a ring.
 */
typedef struct
{
    UdmaTestRingPrmId       ringPrmId;
    /**< Ring parameter ID. */
    uint32_t                eventMode;
    /**< Use no event, event with interrupt or event with polled mode.
     *   Refer #UdmaTestEventMode */
} UdmaTestRingPrm;

/**
 *  \brief Test parameters for a PKTDMA Channel.
 */
typedef struct
{
    UdmaTestPktdmaChPrmId   pktdmachPrmId;
    /**< PKTDMA Channel parameter ID. */
    uint32_t                chType;
    /**< UDMA channel type. */
    uint32_t                mappedChGrp;
    /**< UDMA mapped channel group. */
    uint32_t                peerChNum;
    /**< UDMA peer channel to link the #chNum using PSILCFG.*/
} UdmaTestPktdmaChPrm;

/**
 *  \brief Test case parameter structure.
 */
struct UdmaTestParams_t
{
    bool                    enableTest;
    /**< Whether test case should be executed or not. */
    uint32_t                tcId;
    /**< Test case ID. */
    char                   *tcName;
    /**< Test case name. */
    char                   *disableInfo;
    /**< Reason string for disabling a test case. */
    bool                    printEnable;
    /**< Enable/disable print statements, used for stress testing. */
    bool                    prfEnable;
    /**< Enable performance prints. */
    uint32_t                tcType;
    /**< Type of testcase  - like BFT, stress etc... */
    uint32_t                dcEnable;
    /**< Enable/disable data check - used for performance. */
    uint32_t                loopCnt;
    /**< Loop count for test. */

    uint32_t                numTasks;
    /**< Number of tasks to test. */
    uint32_t                testType[UDMA_TEST_MAX_TASKS];
    /**< Type of test */
    UdmaTestFxnPtr          testFxnPtr[UDMA_TEST_MAX_TASKS];
    /**< Type of test case to run. */
    uint32_t                pacingTime[UDMA_TEST_MAX_TASKS];
    /**< Pacing Interval in ms. */
    uint32_t                numCh[UDMA_TEST_MAX_TASKS];
    /**< Number of channels per task to test. */
    uint32_t                instId[UDMA_TEST_MAX_CH];
    /**< NAVSS/DMSS instance ID. */
    UdmaTestChPrmId         chPrmId[UDMA_TEST_MAX_CH];
    /**< Channel config IDs for all the tasks.
     *   Task 0 channel configs will be first (0 to (numCh[0] - 1)),
     *   (numCh[0] to (numCh[1] - 1)) and so on. */
    uint32_t                qdepth[UDMA_TEST_MAX_CH];
    /**< Queue depth. */
    uint32_t                icnt[UDMA_TEST_MAX_CH][UDMA_TEST_MAX_ICNT];
    /**< Source counts. */
    uint32_t                dicnt[UDMA_TEST_MAX_CH][UDMA_TEST_MAX_ICNT];
    /**< Destination counts. */
    uint32_t                dim[UDMA_TEST_MAX_CH][UDMA_TEST_MAX_DIM];
    /**< Source dims. */
    uint32_t                ddim[UDMA_TEST_MAX_CH][UDMA_TEST_MAX_DIM];
    /**< Destination dims. */
    uint32_t                heapIdSrc[UDMA_TEST_MAX_CH];
    /**< Heap ID to allocate source buffer. */
    uint32_t                heapIdDest[UDMA_TEST_MAX_CH];
    /**< Heap ID to allocate destination buffer. */
    uint32_t                srcBufSize[UDMA_TEST_MAX_CH];
    /**< Source buffer size to allocate. */
    uint32_t                destBufSize[UDMA_TEST_MAX_CH];
    /**< Destination buffer size to allocate. */
    uint64_t                runFlag;
    /**< Flag to indicate whether the test needs to be run on a particular
     *   SOC, CORE and other configurations. */
    UdmaTestRingPrmId       ringPrmId;
    /**< Ring config ID. Applicable only for ring testcases */

    /*
     * Below variables are initialized in code and not in table!!
     */
    bool                    isRun;
    /**< Flag to indicate whether the test case is run or not. */
    int32_t                 testResult;
    /**< Test result. */
};

/**
 *  \brief Structure used for UDMA UT control parameters.
 */
struct UdmaTestSystemCtrl_t
{
    uint32_t    loopCnt;
    /**< Default loop count. */
    uint32_t    qdepth;
    /**< Default queue depth for queue and dequeue operation. */
    uint32_t    rtPrintEnable;
    /**< Enables runtime remote prints like VENC underflow status etc. */
};

typedef struct UdmaTestChObj_t
{
    int32_t                 chIdx;

    Udma_ChObject           drvChObj;
    Udma_EventObject        cqEventObj;
    Udma_EventObject        trEventObj;

    Udma_ChHandle           chHandle;
    Udma_EventHandle        cqEventHandle;
    Udma_EventHandle        tdCqEventHandle;
    Udma_EventHandle        trEventHandle;

    Udma_EventPrms          trEventPrms;

    Udma_DrvHandle          drvHandle;
    uint32_t                instId;
    SemaphoreP_Object       transferDoneSem;
    /**< Semaphore to indicate transfer completion */

    uint32_t                queueCnt;
    uint32_t                dequeueCnt;

    uint8_t                 *fqRingMem;
    uint8_t                 *cqRingMem;
    uint8_t                 *tdCqRingMem;
    uint8_t                 *trpdMem[UDMA_TEST_MAX_QDEPTH];

    uint8_t                 *srcBuf;
    uint8_t                 *destBuf[UDMA_TEST_MAX_QDEPTH];

    const UdmaTestChPrm     *chPrms;
    const Udma_ChTxPrms     *txPrms;
    const Udma_ChRxPrms     *rxPrms;
    const Udma_ChPdmaPrms   *pdmaPrms;
    uint32_t                qdepth;
    uint32_t                icnt[UDMA_TEST_MAX_ICNT];
    uint32_t                dicnt[UDMA_TEST_MAX_ICNT];
    uint32_t                dim[UDMA_TEST_MAX_DIM];
    uint32_t                ddim[UDMA_TEST_MAX_DIM];
    uint32_t                heapIdSrc;
    uint32_t                heapIdDest;
    uint32_t                srcBufSize;
    uint32_t                destBufSize;

    uint32_t                ringMemSize;
    uint32_t                trpdSize;
} UdmaTestChObj;

/**
 *  \brief Structure used for UDMA UT task object parameters.
 */
struct UdmaTestTaskObj_t
{
    UdmaTestParams         *testPrms;
    /**< Pointer to test params for reference. */
    uint32_t                testType;
    UdmaTestFxnPtr          testFxnPtr;
    uint32_t                pacingTime;
    /**< Pacing Interval in ms. */
    uint32_t                numCh;
    UdmaTestChObj          *chObj[UDMA_TEST_MAX_CH];
    uint32_t                traceMask;
    int32_t                 testResult;
    uint64_t                totalTransfer;
    /**< Total bytes transferred */
    uint32_t                durationMs;
    /**< Time taken in ms */
    uint64_t                mps;
    /**< Mega bytes per second */
    UdmaTestObj            *testObj;
    uint32_t                taskId;
    TaskP_Object            taskHandle;
    Utils_PrfTsHndl        *prfTsHandle;
    char                    prfTsName[15];
    uint32_t                loopCnt;
    const UdmaTestRingPrm  *ringPrms;
};

/**
 *  \brief Structure used for UDMA test object common for all the tasks.
 */
struct UdmaTestObj_t
{
    UdmaTestSystemCtrl  sysCtrl;
    /**< System control information. */
    UdmaTestParams     *testPrms;
    /**< Pointer to test params for reference. */
    uint32_t            skipCount;
    /**< Number of test cases skipped because of platform/user settings. */
    uint32_t            disableCount;
    /**< Number of test cases disabled because of any bug etc. */
    uint32_t            traceMask;
    /**< Masks for the debug prints. */
    uint64_t            runFlag;
    /**< Current run flag for a SOC, CORE and other configurations. */

    Udma_DrvObject      drvObj[UDMA_INST_ID_MAX + 1U];
    /**< Driver Object for all applicable instances. (max+1 since instance index starts from 0) */

    SemaphoreP_Object   taskCompleteSem;
    SemaphoreP_Object   lockSem;

    UdmaTestTaskObj     taskObj[UDMA_TEST_MAX_TASKS];
    UdmaTestChObj       chObjs[UDMA_TEST_MAX_CH];
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*
 * Parser functions
 */
int32_t udmaTestParser(void);
void udmaDrvPrint(const char *str);

/*
 * UDMA block copy test case functions
 */
int32_t udmaTestBlkcpyTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestBlkcpyPauseResumeTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestBlkcpyChainingTc(UdmaTestTaskObj *taskObj);
/*
 * UDMA ring test functions
 */
int32_t udmaTestRingProxyTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingFlushTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingEventTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingParamCheckTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingUtilsMemSizeTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingMemPtrTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingAttachTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingResetTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingPrimeTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingPrimeLcdmaTc(UdmaTestTaskObj *taskObj);
/*
 * UDMA ring monitor test functions
 */
int32_t udmaTestRingMonPushPopTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingMonLowThresholdTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestRingMonHighThresholdTc(UdmaTestTaskObj *taskObj);
/*
 * UDMA proxy test functions
 */
int32_t udmaTestProxyPerformanceTc(UdmaTestTaskObj *taskObj);
/*
 * UDMA flow test functions
 */
int32_t udmaTestFlowAttachMappedTc(UdmaTestTaskObj *taskObj);
/*
 * UDMA misc test functions
 */
int32_t udmaTestPsilMacroTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestTrMakeTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestStructSizeTc(UdmaTestTaskObj *taskObj);
/*
 * UDMA bug test functions
 */
int32_t udmaTestBugTcPDK_4654(UdmaTestTaskObj *taskObj);
/*
 * UDMA channel test functions
 */
int32_t udmaTestChPktdmaParamCheckTc(UdmaTestTaskObj *taskObj);
int32_t udmaTestChPktdmaChApiTc(UdmaTestTaskObj *taskObj);

/*
 * UDMA Negative test case functions
 */

/*
 * UDMA SOC specific functions
 */
int32_t udmaTestPrintPsilMacro(UdmaTestTaskObj *taskObj);
int32_t udmaTestPrintPdmaMacro(UdmaTestTaskObj *taskObj);

/*
 * UDMA common functions
 */
int32_t udmaTestInitDriver(UdmaTestObj *testObj);
int32_t udmaTestDeinitDriver(UdmaTestObj *testObj);
int32_t udmaTestInitVariables(const UdmaTestObj *testObj);
void udmaTestFreeVariables(const UdmaTestObj *testObj);
void udmaTestLogTestResult(const UdmaTestObj *testObj,
                           int32_t            testResult,
                           uint32_t           tcId,
                           char              *tcInfo);
void udmaTestPrintTestResult(const UdmaTestObj *testObj,
                             uint32_t           skipCount,
                             uint32_t           disableCount);
void udmaTestResetTestResult(void);
void udmaTestCalcPerformance(UdmaTestTaskObj *taskObj, uint32_t durationMs);
void udmaTestCalcTotalPerformance(UdmaTestObj *testObj, uint32_t durationMs);
int32_t udmaTestCompareRingHwOccDriver(Udma_RingHandle ringHandle, uint32_t cnt, uint32_t direction);
uint32_t udmaTestGetRingHwOccDriver(Udma_RingHandle ringHandle, uint32_t direction);
int32_t udmaTestBlkCpyRingPrimeLcdmaTest(UdmaTestTaskObj *taskObj);

char AppUtils_getChar(void);
int32_t AppUtils_getCharTimeout(char *ch, uint32_t msec);
int32_t AppUtils_getNum(void);

uint32_t AppUtils_getCurTimeInMsec(void);
uint32_t AppUtils_getElapsedTimeInMsec(uint32_t startTime);

/* ========================================================================== */
/*      Internal Function Declarations (Needed for other static inlines)      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_TEST_H_ */
