/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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
 *  \file mcrc_auto mode.c
 *
 *  \brief Common across use-cases using MCRC Auto mode.
 *
 */

/*===========================================================================*/
/*                         Include Files                                     */
/*===========================================================================*/
#include "main.h"
#include <stdio.h>
#include <kernel/dpl/DebugP.h>

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define MCRC_APP_USER_DATA_SIZE              ((uint32_t)1000U)
#define MCRC_APP_CRC_SIGN_SIZE               ((uint32_t)8)
#define MCRC_APP_CRC_PATTERN_SIZE            (4U)
#define MCRC_APP_CRC_PATTERN_CNT             ((uint32_t)(MCRC_APP_USER_DATA_SIZE / MCRC_APP_CRC_PATTERN_SIZE))
#define MCRC_APP_CRC_SECT_CNT                (1U)

/** \brief Number of times to perform the MCRC operation */
#define LOOP_COUNT          (20U)

/*
 * Ring parameters
 */
/** \brief Number of ring entries - we can prime this much MCRC operations */
#define UDMA_APP_RING_ENTRIES      (1U)
/** \brief Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define UDMA_APP_RING_ENTRY_SIZE   (sizeof(uint64_t))
/** \brief Total ring memory */
#define UDMA_APP_RING_MEM_SIZE     (UDMA_APP_RING_ENTRIES * UDMA_APP_RING_ENTRY_SIZE)
/**
 *  \brief UDMA TR packet descriptor memory.
 *  This contains the CSL_UdmapCppi5TRPD + Padding to sizeof(CSL_UdmapTR15) +
 *  one Type_15 TR (CSL_UdmapTR15) + one TR response of 4 bytes.
 *  Since CSL_UdmapCppi5TRPD is less than CSL_UdmapTR15, size is just two times
 *  CSL_UdmapTR15 for alignment.
 */
#define UDMA_APP_TRPD_SIZE         ((sizeof(CSL_UdmapTR15) * 2U) + 4U)
#define MCRC_USECASES	(2U)
#define MCRC_EXPECTED_HIGH_VALUE   ((uint32_t)0x474B7CF0)
#define MCRC_EXPECTED_LOW_VALUE    ((uint32_t)0x03D4145D)

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */
static uint8_t gMCRCSrcBuffer[MCRC_APP_USER_DATA_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
static uint8_t gTxRingMem[UDMA_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

static uint8_t gTxCompRingMem[UDMA_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gTxTdCompRingMem[UDMA_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Buffer to store predefined CRC value */
static uint8_t gUdmaTprdMem[UDMA_APP_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gUdmaAppDoneSem;

Udma_ChObject       gUdmaChObj;
Udma_EventObject    gUdmaCqEventObj;
Udma_EventObject    gUdmaTdCqEventObj;
uint32_t            testCase = 0;

/** Defines the various MCRC use cases. */
static    SDL_MCRC_ConfigParams_t testparams[MCRC_USECASES] =
{
  {

#if defined(SOC_AM64X) || defined(SOC_AM243X)
	 MCU_MCRC64_0,
#endif

      (uint32_t) SDL_MCRC_CHANNEL_1,
      (uint32_t) SDL_MCRC_OPERATION_MODE_AUTO,
      4U,
      MCRC_APP_CRC_PATTERN_CNT,
      MCRC_APP_CRC_SECT_CNT,
      MCRC_DEF_WATCHDOG_PRELOAD,
      MCRC_DEF_BLOCK_PRELOAD,
      MCRC_EXPECTED_HIGH_VALUE,
      MCRC_EXPECTED_LOW_VALUE,
      MCRC_APP_USER_DATA_SIZE,
      (uint32_t) &gMCRCSrcBuffer[0],
  },
  {

#if defined(SOC_AM64X) || defined(SOC_AM243X)
	 MCU_MCRC64_0,
#endif

      (uint32_t) SDL_MCRC_CHANNEL_2,
      (uint32_t) SDL_MCRC_OPERATION_MODE_AUTO,
      4U,
      MCRC_APP_CRC_PATTERN_CNT,
      MCRC_APP_CRC_SECT_CNT,
      MCRC_DEF_WATCHDOG_PRELOAD,
      MCRC_DEF_BLOCK_PRELOAD,
      MCRC_EXPECTED_HIGH_VALUE,
      MCRC_EXPECTED_LOW_VALUE,
      MCRC_APP_USER_DATA_SIZE,
      (uint32_t) &gMCRCSrcBuffer[0],
  },
};

/*===========================================================================*/
/*                   Function definitions                              */
/*===========================================================================*/

static void mcrcAutoCPU_udmaEventCb(Udma_EventHandle eventHandle,
                            uint32_t eventType,
                            void *appData);

static void mcrcAutoCPU_udmaTrpdInit(Udma_ChHandle chHandle,
                                       uint8_t *pTrpdMem,
                                       const void *srcBuf,
                                       const void *destBuf,
                                       uint32_t length,
                                       uint32_t patternSize);

int32_t mcrcAutoCPU_main(void)
{
    int32_t                     retVal = SDL_PASS;
    uint32_t                    intrStatus,loopCnt = 0;
    SDL_MCRC_SignatureRegAddr_t psaSignRegAddr;
    uint32_t                    patternCnt, sectCnt;
    SDL_MCRC_InstType           instance;
    SDL_MCRC_Channel_t          mcrcChannel;
    uint32_t                    IntrMask = 0x1U;
    Udma_ChHandle               chHandle = &gUdmaChObj;
    uint64_t                    cpuModeTime;
    SDL_MCRC_Signature_t        psaSignRegVal,refSignVal;
    uint32_t                    chType = 0;
    Udma_ChPrms                 chPrms;
    Udma_DrvHandle              drvHandle = gUdmaDrvObj;
    uint8_t                     *tprdMem = &gUdmaTprdMem[0U];
    uint32_t                    testStatus = SystemP_SUCCESS;
    Udma_ChTxPrms               txPrms;
    Udma_ChRxPrms               rxPrms;
    Udma_EventHandle            eventHandle;
    Udma_EventPrms              eventPrms;
    uint64_t                    trpdMemPhy;

    DebugP_log("\r\n\r\nMCRC Auto_MODE mode : starting\r\n\r\n");
    testStatus = SemaphoreP_constructBinary(&gUdmaAppDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == testStatus);

    if(SDL_PASS == retVal)
    {
        /* Init channel parameters */
        chType = UDMA_CH_TYPE_TR_BLK_COPY;
        UdmaChPrms_init(&chPrms, chType);
        chPrms.fqRingPrms.ringMem   = &gTxRingMem[0U];
        chPrms.cqRingPrms.ringMem   = &gTxCompRingMem[0U];
        chPrms.tdCqRingPrms.ringMem = &gTxTdCompRingMem[0U];
        chPrms.fqRingPrms.ringMemSize   = UDMA_APP_RING_MEM_SIZE;
        chPrms.cqRingPrms.ringMemSize   = UDMA_APP_RING_MEM_SIZE;
        chPrms.tdCqRingPrms.ringMemSize = UDMA_APP_RING_MEM_SIZE;
        chPrms.fqRingPrms.elemCnt   = UDMA_APP_RING_ENTRIES;
        chPrms.cqRingPrms.elemCnt   = UDMA_APP_RING_ENTRIES;
        chPrms.tdCqRingPrms.elemCnt = UDMA_APP_RING_ENTRIES;
    }
    /* Open channel for block copy */
    retVal = Udma_chOpen(drvHandle, chHandle, chType, &chPrms);
    if(SDL_PASS != retVal)
    {
        DebugP_log("[Error] UDMA channel open failed!!\r\n");
    }
    if(SDL_PASS == retVal)
    {
        /* Config TX channel */
        UdmaChTxPrms_init(&txPrms, chType);
        retVal = Udma_chConfigTx(chHandle, &txPrms);
        if(SDL_PASS != retVal)
        {
            DebugP_log("[Error] UDMA TX channel config failed!!\r\n");
        }
    }

    if(SDL_PASS == retVal)
    {
        /* Config RX channel - which is implicitly paired to TX channel in
         * block copy mode */
        UdmaChRxPrms_init(&rxPrms, chType);
        retVal = Udma_chConfigRx(chHandle, &rxPrms);
        if(SDL_PASS != retVal)
        {
            DebugP_log("[Error] UDMA RX channel config failed!!\r\n");
        }
    }

    if(SDL_PASS == retVal)
    {
        /* Register ring completion callback */
        eventHandle = &gUdmaCqEventObj;
        UdmaEventPrms_init(&eventPrms);
        eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
        eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
        eventPrms.chHandle          = chHandle;
        eventPrms.eventCb           = &mcrcAutoCPU_udmaEventCb;
        retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
        if(SDL_PASS != retVal)
        {
            DebugP_log("[Error] UDMA CQ event register failed!!\r\n");
        }
    }

    for(testCase=0;testCase<MCRC_USECASES;testCase++)
    {
        DebugP_log("\r\nMCRC AUTO mode on Channel %d: Transfer Test Started...\r\n\r\n", testCase+1);

        for (loopCnt = 0U; loopCnt < MCRC_APP_USER_DATA_SIZE; loopCnt++)
        {
            gMCRCSrcBuffer[loopCnt] = (uint8_t)loopCnt;
        }
        CacheP_wb((void *)gMCRCSrcBuffer, MCRC_APP_USER_DATA_SIZE, CacheP_TYPE_ALL);

        /* Get the reference crc sign value. The reference crc sign value is retrieved
        by performing full cpu mode CRC on the same set of data used for Auto mode.
        This is done in the test app to get the reference value for the test, but is
        not required for using Auto mode. */
        if ((0U == testparams[testCase].mcrcSignHigh) &&
            (0U == testparams[testCase].mcrcSignLow))
        {
            DebugP_log("\r\nCalculating Reference MCRC signature Value.");
            instance = testparams[testCase].instance;
            mcrcChannel  = testparams[testCase].mcrcChannelNumber;
            patternCnt  = testparams[testCase].mcrcPatternSize;
            sectCnt     = testparams[testCase].mcrcSectorCount;
            uint32_t *srcBufferPtr;

            /* Reset the CRC channel*/
            SDL_MCRC_channelReset(instance, mcrcChannel);
            /* Get CRC PSA signature register address */
            SDL_MCRC_getPSASigRegAddr(instance, mcrcChannel, &psaSignRegAddr);

            SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, SDL_MCRC_OPERATION_MODE_FULLCPU);

            /* Get CRC PSA signature register address */
            SDL_MCRC_getPSASigRegAddr(instance, mcrcChannel, &psaSignRegAddr);

            srcBufferPtr = (uint32_t *)gMCRCSrcBuffer;
            cpuModeTime = ClockP_getTimeUsec();

            /* compute the MCRC by writing the data buffer on which MCRC computation is needed */
            for (loopCnt = 0; loopCnt < MCRC_APP_CRC_PATTERN_CNT; loopCnt++)
            {
                HW_WR_REG32(psaSignRegAddr.regL, srcBufferPtr[loopCnt]);
            }
            cpuModeTime = ClockP_getTimeUsec() - cpuModeTime;

            /* Fetch MCRC signature value */
            SDL_MCRC_getPSASig(instance, mcrcChannel, &refSignVal);
            DebugP_log("\r\n MCRC signature value : 0x%x%xU",
                    refSignVal.regH,
                    refSignVal.regL);
            DebugP_log("\r\nMCRC Full Mode Computation Time: %dus\r\n\r\n", cpuModeTime);
        }
        else
        {
            DebugP_log("\r\nUsing Pre-Defined Reference MCRC signature Value.\r\n");
            refSignVal.regH = testparams[testCase].mcrcSignHigh;
            refSignVal.regL = testparams[testCase].mcrcSignLow;
            DebugP_log("\r\nPre-defined MCRC signature value : 0x%x%xU\r\n",
                        refSignVal.regH,
                        refSignVal.regL);
        }
        if(retVal == SDL_PASS)
        {
            patternCnt  = testparams[testCase].dataSize / testparams[testCase].mcrcPatternSize;
            sectCnt     = testparams[testCase].mcrcSectorCount;
            instance    = testparams[testCase].instance;
            mcrcChannel  = testparams[testCase].mcrcChannelNumber;

            /* Reset the CRC channel*/
            SDL_MCRC_channelReset(instance, mcrcChannel);
            SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, testparams[testCase].mcrcMode);

            SDL_MCRC_getPSASigRegAddr(instance, mcrcChannel, &psaSignRegAddr);

            /* Initialize and Configure MCRC channel */
            retVal = SDL_MCRC_init(testparams[testCase].instance,
                        testparams[testCase].mcrcChannelNumber,
                        testparams[testCase].mcrcWatchdogPreload,
                        testparams[testCase].mcrcBlockPreload);
            if(SDL_PASS != retVal)
            {
                DebugP_log("[Error] mcrcAutoCPU channel intialization failed!!\r\n");
            }

            retVal = SDL_MCRC_enableIntr(instance, mcrcChannel,IntrMask);
            retVal |= SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, testparams[testCase].mcrcMode);

            retVal |= Udma_chEnable(chHandle);
            trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(tprdMem, 0U, NULL);
            /* Update TR packet descriptor */
            mcrcAutoCPU_udmaTrpdInit(chHandle, tprdMem,
                                (void *) testparams[testCase].sourceMemory,
                                (void *)(uintptr_t) psaSignRegAddr.regL,
                                testparams[testCase].dataSize,
                                testparams[testCase].mcrcPatternSize);

            retVal |= Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle),trpdMemPhy);
            if(SDL_PASS != retVal)
            {
                DebugP_log("[Error] Channel queue failed!!\r\n");
            }
            retVal |= SemaphoreP_pend(&gUdmaAppDoneSem, SystemP_WAIT_FOREVER);

            if(SDL_PASS != retVal)
            {
                DebugP_log("[Error] No descriptor after callback!!\r\n");
                retVal = SDL_EFAIL;
            }

            if (SDL_PASS == retVal)
            {
                retVal = SDL_MCRC_getPSASectorSig(instance, mcrcChannel, &psaSignRegVal);
                SDL_MCRC_getIntrStatus(instance, mcrcChannel, &intrStatus);
            }

            if(((refSignVal.regH == psaSignRegVal.regH) &&
            (refSignVal.regL == psaSignRegVal.regL)))
            {
                DebugP_log("\r\nSector signature matches - Passed");
                DebugP_log("\r\nCalculated MCRC signature value : 0x%08x%08xU\r\n",
                                    psaSignRegVal.regH,
                                    psaSignRegVal.regL);
                DebugP_log("\r\nUDMA Data transfer completed !!\r\n\r\n");
                DebugP_log("MCRC Auto Mode Computation Time: %dus\r\n\r\n", cpuModeTime);
                retVal = SDL_PASS;
            }
            else
            {
                retVal = SDL_EFAIL;
                DebugP_log("\r\nSector signature does not match.");
                DebugP_log("\r\nSome tests have failed!!\r\n\r\n");
                DebugP_log("\r\nExpected MCRC signature value : 0x%x%xU\r\n",
                    refSignVal.regH,
                    refSignVal.regL);
                DebugP_log("\r\nCalculated MCRC signature value : 0x%08x%08xU\r\n",
                    psaSignRegVal.regH,
                    psaSignRegVal.regL);
            }
            SDL_MCRC_clearIntr(instance, mcrcChannel, SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL);
        }
    }

    /* Channel disable */
    retVal = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == retVal);

    /* UnRegister Event */
    retVal = Udma_eventUnRegister(eventHandle);
    DebugP_assert(UDMA_SOK == retVal);

    /* Flush any pending request from the free queue */
    while(1)
    {
        uint64_t pDesc;
        int32_t  tempRetVal;

        tempRetVal = Udma_ringFlushRaw(
                         Udma_chGetFqRingHandle(chHandle), &pDesc);
        if(UDMA_ETIMEOUT == tempRetVal)
        {
            break;
        }
    }

    retVal = Udma_chClose(chHandle);
    DebugP_assert(UDMA_SOK == retVal);

    SemaphoreP_destruct(&gUdmaAppDoneSem);
    return (retVal);
}

static void mcrcAutoCPU_udmaEventCb(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    uint64_t pDesc;

    Udma_ringDequeueRaw(Udma_chGetCqRingHandle(&gUdmaChObj), &pDesc);

    SemaphoreP_post(&gUdmaAppDoneSem);
}

static void mcrcAutoCPU_udmaTrpdInit(Udma_ChHandle chHandle,
                                       uint8_t *pTrpdMem,
                                       const void *srcBuf,
                                       const void *destBuf,
                                       uint32_t length,
                                       uint32_t patternSize)
{
    CSL_UdmapCppi5TRPD *pTrpd = (CSL_UdmapCppi5TRPD *) pTrpdMem;
    CSL_UdmapTR15 *pTr = (CSL_UdmapTR15 *)(pTrpdMem + sizeof(CSL_UdmapTR15));
    uint32_t *pTrResp = (uint32_t *) (pTrpdMem + (sizeof(CSL_UdmapTR15) * 2U));
    uint32_t cqRingNum = Udma_chGetCqRingNum(chHandle);
    uint32_t cCnt;

    /* Make TRPD */
    UdmaUtils_makeTrpd((uint8_t *)pTrpd, UDMA_TR_TYPE_15, 1U, cqRingNum);

    /* Setup TR */
    cCnt = 1;
    while ((length / cCnt) > 0x7FFFU)
    {
        cCnt = cCnt * 2;
    }
    pTr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, 15)                                            |
                    CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U)                                          |
                    CSL_FMK(UDMAP_TR_FLAGS_EOL, 0U)                                             |   /* NA */
                    CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION)|
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE)           |
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL)  |
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE)           |
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL)  |
                    CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U)                                       |   /* This will come back in TR response */
                    CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U)                                     |
                    CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U)                                     |
                    CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);
    pTr->icnt0    = patternSize;
    pTr->icnt1    = (length / patternSize) / cCnt;
    pTr->icnt2    = cCnt;
    pTr->icnt3    = 1U;
    pTr->dim1     = pTr->icnt0;
    pTr->dim2     = (pTr->icnt0 * pTr->icnt1);
    pTr->dim3     = (pTr->icnt0 * pTr->icnt1 * pTr->icnt2);
    pTr->addr     = (uint64_t) srcBuf;
    pTr->fmtflags = 0x00000000U;        /* Linear addressing, 1 byte per elem.
                                           Replace with CSL-FL API */
    pTr->dicnt0   = patternSize;
    pTr->dicnt1   = (length / pTr->dicnt0) / cCnt;
    pTr->dicnt2   = cCnt;
    pTr->dicnt3   = 1U;
    pTr->ddim1    = 0U;
    pTr->ddim2    = 0U;
    pTr->ddim3    = 0U;
    pTr->daddr    = (uint64_t) destBuf;

    /* Clear TR response memory */
    *pTrResp = 0xFFFFFFFFU;

    /* Writeback cache */
    CacheP_wb(pTrpdMem, UDMA_APP_TRPD_SIZE, CacheP_TYPE_ALL);
    return;
}
