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
 *  \brief Common across use-cases using MCRC Auto-CPU mode.
 *
 */

/*===========================================================================*/
/*                         Include Files                                     */
/*===========================================================================*/
#include "main.h"
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

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

#define MCRC_256KB_BYTES    (uint32_t)(1024 * 256)
#define MCRC_128KB_BYTES    (uint32_t)(1024 * 128)
#define MCRC_1KB_BYTES      (uint32_t)(1024 * 1)

/* Pre calculated CRC values for profiling datasets */
#define MCRC_256KB_HI       (0xA51F5565)
#define MCRC_256KB_LO       (0xECA7D261)
#define MCRC_128KB_HI       (0x4EB4CABB)
#define MCRC_128KB_LO       (0x432911AF)
#define MCRC_1KB_HI         (0x958B7A02)
#define MCRC_1KB_LO         (0x1871EC9A)

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */
static uint8_t gMCRCSrcBuffer[MCRC_APP_USER_DATA_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
static uint8_t gTxRingMem[UDMA_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint32_t profMCRCSrcBuffer[MCRC_256KB_BYTES/4] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

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
      (uint32_t) SDL_MCRC_OPERATION_MODE_FULLCPU,
      4U,
      MCRC_APP_CRC_PATTERN_CNT,
      MCRC_APP_CRC_SECT_CNT,
      MCRC_DEF_WATCHDOG_PRELOAD,
      MCRC_DEF_BLOCK_PRELOAD,
      0x474B7CF0,
      0x03D4145D,
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
      0x474B7CF0,
      0x03D4145D,
      MCRC_APP_USER_DATA_SIZE,
      (uint32_t) &gMCRCSrcBuffer[0],
  },
};

static SDL_MCRC_ConfigParams_t profParams =
{
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    MCU_MCRC64_0,
#endif
    (uint32_t) SDL_MCRC_CHANNEL_2,
    (uint32_t) SDL_MCRC_OPERATION_MODE_AUTO,
    4U,
    0,
    MCRC_APP_CRC_SECT_CNT,
    MCRC_DEF_WATCHDOG_PRELOAD,
    MCRC_DEF_BLOCK_PRELOAD,
    0x00000000,
    0x00000000,
    0,
    (uint32_t) &profMCRCSrcBuffer[0],
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void mcrcAutoMode_udmaEventCb(Udma_EventHandle eventHandle,
                            uint32_t eventType,
                            void *appData);

static void mcrcAutoMode_udmaTrpdInit(Udma_ChHandle chHandle,
                                       uint8_t *pTrpdMem,
                                       const void *srcBuf,
                                       const void *destBuf,
                                       uint32_t length,
                                       uint32_t patternSize);

static int32_t mcrcAutoProfile(SDL_MCRC_ConfigParams_t mcrcParams, Udma_ChHandle chHandle, SDL_MCRC_Signature_t *crc,
                               uint64_t trpdMemPhy, SDL_MCRC_SignatureRegAddr_t psaSignRegAddr, uint64_t *profTime);

/*===========================================================================*/
/*                   Function definitions                              */
/*===========================================================================*/

static int32_t mcrcAutoProfile(SDL_MCRC_ConfigParams_t mcrcParams, Udma_ChHandle chHandle, SDL_MCRC_Signature_t *crc,
                               uint64_t trpdMemPhy, SDL_MCRC_SignatureRegAddr_t psaSignRegAddr, uint64_t *profTime)
{
    uint64_t profStartTime, profEndTime;
    uint32_t patternCnt = mcrcParams.dataSize / mcrcParams.mcrcPatternSize;
    uint32_t sectCnt = profParams.mcrcSectorCount;
    SDL_MCRC_InstType instance = profParams.instance;
    SDL_MCRC_Channel_t mcrcChannel = profParams.mcrcChannelNumber;
    uint32_t intrMask = 0x1U;
    int32_t retVal = SDL_PASS;
    uint16_t timeout = 0xFFFF;
    uint32_t intrStatus;

    /* Reset the CRC channel*/
    SDL_MCRC_channelReset(instance, mcrcChannel);
    SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, mcrcParams.mcrcMode);

    retVal = SDL_MCRC_enableIntr(instance, mcrcChannel,intrMask);
    retVal = SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, mcrcParams.mcrcMode);

    retVal = Udma_chEnable(chHandle);
    /* Update TR packet descriptor */
    mcrcAutoMode_udmaTrpdInit(chHandle,(uint8_t *)trpdMemPhy,
                              (void *) mcrcParams.sourceMemory,
                              (void *)(uintptr_t) psaSignRegAddr.regL,
                              mcrcParams.dataSize,
                              mcrcParams.mcrcPatternSize);
    profStartTime = ClockP_getTimeUsec();
    retVal = Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle),trpdMemPhy);
    if(SDL_PASS != retVal)
    {
        DebugP_log(" [Error] Channel queue failed during profiling test!!\r\n");
    }
    retVal = SemaphoreP_pend(&gUdmaAppDoneSem, SystemP_WAIT_FOREVER);

    if(SDL_PASS != retVal)
    {
        DebugP_log(" [Error] No descriptor after callback during profiling test!!\r\n");
        retVal = SDL_EFAIL;
    }
    intrStatus=0u;
    while (intrStatus == 0x0U && timeout>0)
    {
        /* complete is set. */
        SDL_MCRC_getIntrStatus(instance, mcrcChannel, &intrStatus);
        timeout--;
    }
    profEndTime = ClockP_getTimeUsec();
    if (timeout == 0)
    {
        DebugP_log(" [Error] MCRC timed out during profiling test!!\r\n");
        retVal = SDL_EFAIL;
        /* To set profTime to 0 because of the failure */
        profEndTime = profStartTime;
    }
    else
    {
        /*
         * Return value is discarded because the only fail condition is bad
         * args, which is not possible at this stage due to previous checks
         */
        SDL_MCRC_getPSASectorSig(instance, mcrcChannel, crc);
        DebugP_log(" Calculated CRC value is 0x%08x%08x\r\n", crc->regH, crc->regL);
    }
    *profTime = profEndTime - profStartTime;
    return retVal;
}

int32_t mcrcAutoCPU_main(void)
{
    int32_t                     retVal = SDL_PASS;
    uint32_t                    intrStatus,loopCnt = 0;
    SDL_MCRC_SignatureRegAddr_t psaSignRegAddr;
    uint32_t                    patternCnt, sectCnt;
    SDL_MCRC_InstType           instance;
    SDL_MCRC_Channel_t          mcrcChannel;
    uint32_t                    intrMask = 0x1U;
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
    uint64_t                    trpdMemPhy, profTime;

    DebugP_log("\r\r\nMCRC Auto_MODE mode : starting\r\r\n");
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
        eventPrms.eventCb           = &mcrcAutoMode_udmaEventCb;
        retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
        if(SDL_PASS != retVal)
        {
            DebugP_log("[Error] UDMA CQ event register failed!!\r\n");
        }
    }

    /* First, perform profiling tests for 3 data size values */
    DebugP_log("MCRC Profiling Tests: \r\n\r\n");
    for (loopCnt = 0U; loopCnt < MCRC_256KB_BYTES/profParams.mcrcPatternSize; loopCnt++)
    {
        profMCRCSrcBuffer[loopCnt] = (uint32_t) loopCnt;
    }

    CacheP_wb((void *)profMCRCSrcBuffer, MCRC_256KB_BYTES, CacheP_TYPE_ALL);
    instance = profParams.instance;
    mcrcChannel = profParams.mcrcChannelNumber;
    trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(tprdMem, 0U, NULL);
    SDL_MCRC_getPSASigRegAddr(instance, mcrcChannel, &psaSignRegAddr);
    /* Initialize and Configure MCRC channel */
    retVal = SDL_MCRC_init(profParams.instance,
                           profParams.mcrcChannelNumber,
                           profParams.mcrcWatchdogPreload,
                           profParams.mcrcBlockPreload);
    if(SDL_PASS != retVal)
    {
        DebugP_log("[Error] mcrcAutoCPU channel intialization failed!!\r\n");
    }

    /* For 256KB data size */
    if (retVal == SDL_PASS)
    {
        DebugP_log("Profiling for 256KB dataset\r\n");
        profParams.dataSize = MCRC_256KB_BYTES;
        retVal = mcrcAutoProfile(profParams, chHandle, &psaSignRegVal, trpdMemPhy, psaSignRegAddr, &profTime);
        if (retVal == SDL_PASS)
        {
            if (psaSignRegVal.regH != MCRC_256KB_HI || psaSignRegVal.regL != MCRC_256KB_LO)
            {
                DebugP_log(" Error: MCRC value does not match for 256KB \r\n");
                retVal = SDL_EFAIL;
            }
            else
            {
                DebugP_log(" Calculated and Expected CRC values match for 256KB \r\n");
            }
        }
        if (retVal == SDL_PASS)
        {
            DebugP_log(" MCRC Profiling result: 256KB ~ %dus \r\n", profTime);
        }
        else
        {
            DebugP_log(" Error in MCRC Profiling run for 256KB \r\n");
        }
        DebugP_log("\r\n");
    }

    /* For 128KB data size */
    if (retVal == SDL_PASS)
    {
        DebugP_log("Profiling for 128KB dataset\r\n");
        profParams.dataSize = MCRC_128KB_BYTES;
        retVal = mcrcAutoProfile(profParams, chHandle, &psaSignRegVal, trpdMemPhy, psaSignRegAddr, &profTime);
        if (retVal == SDL_PASS)
        {
            if (psaSignRegVal.regH != MCRC_128KB_HI || psaSignRegVal.regL != MCRC_128KB_LO)
            {
                DebugP_log(" Error: MCRC value does not match for 128KB \r\n");
                retVal = SDL_EFAIL;
            }
            else
            {
                DebugP_log(" Calculated and Expected CRC values match for 128KB \r\n");
            }
        }
        if (retVal == SDL_PASS)
        {
            DebugP_log(" MCRC Profiling result: 128KB ~ %dus \r\n", profTime);
        }
        else
        {
            DebugP_log(" Error in MCRC Profiling run for 128KB \r\n");
        }
        DebugP_log("\r\n");
    }

    /* For 1KB data size */
    if (retVal == SDL_PASS)
    {
        DebugP_log("Profiling for 1KB dataset\r\n");
        profParams.dataSize = MCRC_1KB_BYTES;
        retVal = mcrcAutoProfile(profParams, chHandle, &psaSignRegVal, trpdMemPhy, psaSignRegAddr, &profTime);
        if (retVal == SDL_PASS)
        {
            if (psaSignRegVal.regH != MCRC_1KB_HI || psaSignRegVal.regL != MCRC_1KB_LO)
            {
                DebugP_log(" Error: MCRC value does not match for 1KB \r\n");
                retVal = SDL_EFAIL;
            }
            else
            {
                DebugP_log(" Calculated and Expected CRC values match for 1KB \r\n");
            }
        }
        if (retVal == SDL_PASS)
        {
            DebugP_log(" MCRC Profiling result: 1KB ~ %dus \r\n", profTime);
        }
        else
        {
            DebugP_log(" Error in MCRC Profiling run for 1KB \r\n");
        }
        DebugP_log("\r\n");
    }

    /* Continue with regular MCRC Automode testcases */
    for(testCase=0;testCase<MCRC_USECASES;testCase++)
    {
        DebugP_log("\r\nMCRC AUTO CPU mode on Channel %d: Transfer Test Started...\r\n", testCase+1);

        for (loopCnt = 0U; loopCnt < MCRC_APP_USER_DATA_SIZE; loopCnt++)
        {
            gMCRCSrcBuffer[loopCnt] = (uint8_t)loopCnt;
        }
        CacheP_wb((void *)gMCRCSrcBuffer, MCRC_APP_USER_DATA_SIZE, CacheP_TYPE_ALL);

        /* Get the reference crc sign value. The reference crc sign value is retrieved
        by performing full cpu mode CRC on the same set of data used for Auto mode.
        This is done in the test app to get the reference value for the test, but is
        not required for using Auto-CPU mode. */
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

            /* Fetch MCRC signature value */
            SDL_MCRC_getPSASig(instance, mcrcChannel, &refSignVal);
            cpuModeTime = ClockP_getTimeUsec() - cpuModeTime;
            DebugP_log("\r\n MCRC signature value : 0x%x%xU",
                       refSignVal.regH,
                       refSignVal.regL);
            DebugP_log("\r\nMCRC Full Mode Computation Time: %dus\r\n", cpuModeTime);
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
                DebugP_log("[Error] mcrcAutoMode channel intialization failed!!\r\n");
            }

            retVal = SDL_MCRC_enableIntr(instance, mcrcChannel,intrMask);
            retVal |= SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, testparams[testCase].mcrcMode);

            retVal |= Udma_chEnable(chHandle);
            trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(tprdMem, 0U, NULL);
            /* Update TR packet descriptor */
            mcrcAutoMode_udmaTrpdInit(chHandle, tprdMem,
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
                DebugP_log("\r\nUDMA Data transfer completed !!\r\n");
                DebugP_log("MCRC Auto Mode Computation Time: %dus\r\n", cpuModeTime);
                retVal = SDL_PASS;
            }
            else
            {
                retVal = SDL_EFAIL;
                DebugP_log("\r\nSector signature does not match.");
                DebugP_log("\r\nSome tests have failed!!\r\n");
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

static void mcrcAutoMode_udmaEventCb(Udma_EventHandle eventHandle,
                                 uint32_t eventType,
                                 void *args)
{
    uint64_t pDesc;

    Udma_ringDequeueRaw(Udma_chGetCqRingHandle(&gUdmaChObj), &pDesc);

    SemaphoreP_post(&gUdmaAppDoneSem);
}

static void mcrcAutoMode_udmaTrpdInit(Udma_ChHandle chHandle,
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
