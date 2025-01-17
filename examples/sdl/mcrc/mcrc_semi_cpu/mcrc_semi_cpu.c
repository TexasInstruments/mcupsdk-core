/*
 *  Copyright (C) Texas Instruments Incorporated 2022-2024
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
 *  \file mcrc_semiCPU.c
 *
 *  \brief Common across use-cases using MCRC Semi-CPU mode.
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
/*Macros for instances*/
#if defined (R5F_INPUTS)
#define MCRC_INSTANCE  MSS_MCRC
#endif
#if defined (C66_INPUTS)
#define MCRC_INSTANCE  DSS_MCRC
#endif
/* Value for A count*/
#define EDMA_TEST_A_COUNT           (8U)
/* Value for B count */
#define EDMA_TEST_B_COUNT           (1U)
/* Value for C count */
#define EDMA_TEST_C_COUNT           (1U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)

#define MCRC_APP_USER_DATA_SIZE              ((uint32_t)1000U)
#define MCRC_APP_CRC_SIGN_SIZE               ((uint32_t)8)
#define MCRC_APP_CRC_PATTERN_SIZE            (4U)
#define MCRC_APP_CRC_PATTERN_CNT             ((uint32_t)(MCRC_APP_USER_DATA_SIZE / MCRC_APP_CRC_PATTERN_SIZE))
#define MCRC_APP_CRC_SECT_CNT                (1U)

#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
#define MCRC_USECASES   (4U)
#elif defined(SOC_AM273X)||defined(SOC_AWR294X)
#define MCRC_USECASES   (2U)
#endif
/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gEdmaTransferDoneSem;
static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */
static uint8_t gMCRCSrcBuffer[MCRC_APP_USER_DATA_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
/* Buffer to store predefined CRC value */
static uint8_t gTestSignBuff[MCRC_APP_CRC_SIGN_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

uint64_t semiModeTime, cpuModeTime;

/** Defines the various MCRC use cases. */
static    SDL_MCRC_ConfigParams_t params[MCRC_USECASES] =
{
    {
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
        MCRC0,
#endif
#if defined(SOC_AM273X)||defined(SOC_AWR294X)
        MCRC_INSTANCE,
#endif
        (uint32_t) SDL_MCRC_CHANNEL_1,
        (uint32_t) SDL_MCRC_OPERATION_MODE_SEMICPU,
        4U,
        MCRC_APP_CRC_PATTERN_CNT,
        MCRC_APP_CRC_SECT_CNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x00000000,
        0x00000000,
        MCRC_APP_USER_DATA_SIZE,
        (uint32_t) &gMCRCSrcBuffer[0],
    },
    {
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
        MCRC0,
#endif
#if defined(SOC_AM273X)||defined(SOC_AWR294X)
        MCRC_INSTANCE,
#endif
        (uint32_t) SDL_MCRC_CHANNEL_2,
        (uint32_t) SDL_MCRC_OPERATION_MODE_SEMICPU,
        4U,
        MCRC_APP_CRC_PATTERN_CNT,
        MCRC_APP_CRC_SECT_CNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x01C133AD,
        0xAB4DD50F,
        MCRC_APP_USER_DATA_SIZE,
        (uint32_t) &gMCRCSrcBuffer[0],
    },
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
    {
        MCRC0,
        (uint32_t) SDL_MCRC_CHANNEL_3,
        (uint32_t) SDL_MCRC_OPERATION_MODE_SEMICPU,
        4U,
        MCRC_APP_CRC_PATTERN_CNT,
        MCRC_APP_CRC_SECT_CNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x01C133AD,
        0xAB4DD50F,
        MCRC_APP_USER_DATA_SIZE,
        (uint32_t) &gMCRCSrcBuffer[0],
    },
    {
        MCRC0,
        (uint32_t) SDL_MCRC_CHANNEL_4,
        (uint32_t) SDL_MCRC_OPERATION_MODE_SEMICPU,
        4U,
        MCRC_APP_CRC_PATTERN_CNT,
        MCRC_APP_CRC_SECT_CNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x01C133AD,
        0xAB4DD50F,
        MCRC_APP_USER_DATA_SIZE,
        (uint32_t) &gMCRCSrcBuffer[0],
    },
#endif
};

 static SDL_MCRC_Config_t SDL_MCRC_Config[MCRC_USECASES] =
 {
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_32BIT,
          SDL_MCRC_DATA_32_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_ENABLE
      },
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_32BIT,
          SDL_MCRC_DATA_32_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_ENABLE
      },
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_32BIT,
          SDL_MCRC_DATA_32_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_ENABLE
      },
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_32BIT,
          SDL_MCRC_DATA_32_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_ENABLE
      },
#endif
 };

/*===========================================================================*/
/*                   Function definitions                              */
/*===========================================================================*/
void EDMA_mcrcInit(void)
{
    Drivers_open();
    Board_driversOpen();
}

void EDMA_mcrcDeinit(void)
{
    Board_driversClose();
    Drivers_close();
}

int32_t mcrcSemiCPU_main(void)
{
    int32_t                     retVal = SDL_PASS;
    int32_t                     retVal1 = SDL_PASS;
    uint32_t                    testCase = 0;
    SDL_MCRC_SignatureRegAddr_t psaSignRegAddr;
    uint32_t                    patternCnt, sectCnt;
    SDL_MCRC_InstType           instance;
    SDL_MCRC_Channel_t          mcrcChannel;
    uint8_t                     *srcBuffPtr;
    uint32_t                    *signBuffPtr;
    uint32_t                    IntrMask = 0x1U;
    /* EDMA driver init */
    uint32_t                    baseAddr, regionId;
    int32_t                     testStatus = SystemP_SUCCESS;
    uint32_t                    intrStatus,loopCnt = 0;
    EDMACCPaRAMEntry            edmaParam1;
    uint32_t                    dmaCh1=0, tcc1=0, param1=0;
    Edma_IntrObject             intrObj;
    SDL_MCRC_Signature_t        psaSignRegVal,refSignVal;

    EDMA_mcrcInit();

    for(testCase=0;testCase<MCRC_USECASES;testCase++)
    {
        DebugP_log("\nMCRC SEMI CPU mode on Channel %d: Transfer Test Started...\r\n", testCase+1);

        for (loopCnt = 0U; loopCnt < MCRC_APP_USER_DATA_SIZE; loopCnt++)
        {
            gMCRCSrcBuffer[loopCnt] = (uint8_t)loopCnt;
        }
        CacheP_wb((void *)gMCRCSrcBuffer, MCRC_APP_USER_DATA_SIZE, CacheP_TYPE_ALL);

        /* Get the reference crc sign value. The reference crc sign value is retrieved
        by performing full cpu mode CRC on the same set of data used for semiCPU mode.
        This is done in the test app to get the reference value for the test, but is
        not required for using semi-CPU mode. */
        if ((0U == params[testCase].mcrcSignHigh) &&
            (0U == params[testCase].mcrcSignLow))
        {
            DebugP_log("\nCalculating Reference MCRC signature Value.");
            instance = params[testCase].instance;
            mcrcChannel  = params[testCase].mcrcChannelNumber;
            patternCnt  = params[testCase].mcrcPatternSize;
            sectCnt     = params[testCase].mcrcSectorCount;
            uint32_t *srcBufferPtr;

            /* Reset the CRC channel*/
            SDL_MCRC_channelReset(instance, mcrcChannel);
            /* Get CRC PSA signature register address */
            SDL_MCRC_getPSASigRegAddr(instance, mcrcChannel, &psaSignRegAddr);

            /* Configure CRC channel */
            SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, SDL_MCRC_OPERATION_MODE_FULLCPU);
            SDL_MCRC_addConfig(params[testCase].instance,params[testCase].mcrcChannelNumber,&SDL_MCRC_Config[testCase]);

            /* Get CRC PSA signature register address */
            SDL_MCRC_getPSASigRegAddr(instance, mcrcChannel, &psaSignRegAddr);

            srcBufferPtr = (uint32_t *)gMCRCSrcBuffer;
            signBuffPtr = (uint32_t *)gTestSignBuff;

            cpuModeTime = ClockP_getTimeUsec();

            /* compute the MCRC by writing the data buffer on which MCRC computation is needed */
            for (loopCnt = 0; loopCnt < MCRC_APP_CRC_PATTERN_CNT; loopCnt++)
            {
                HW_WR_REG32(psaSignRegAddr.regL, srcBufferPtr[loopCnt]);
            }

            /* Fetch MCRC signature value */
            SDL_MCRC_getPSASig(instance, mcrcChannel, &refSignVal);
            DebugP_log("\r\n MCRC signature value : 0x%x%xU",
                    refSignVal.regH,
                    refSignVal.regL);
            CacheP_wb((void *)signBuffPtr, params[testCase].mcrcPatternSize, CacheP_TYPE_ALL);
            cpuModeTime = ClockP_getTimeUsec() - cpuModeTime;
            DebugP_log("\r\nMCRC Full Mode Computation Time: %dus\r\n", cpuModeTime);
        }
        else{
            DebugP_log("\r\nUsing Pre-Defined Reference MCRC signature Value.\n");
            refSignVal.regH = params[testCase].mcrcSignHigh;
            refSignVal.regL = params[testCase].mcrcSignLow;
            DebugP_log("\r\nPre-defined MCRC signature value : 0x%x%xU\n",
                        refSignVal.regH,
                        refSignVal.regL);
        }
        if(retVal == SDL_PASS){
            patternCnt  = params[testCase].dataSize / params[testCase].mcrcPatternSize;
            sectCnt     = params[testCase].mcrcSectorCount;
            instance    = params[testCase].instance;
            mcrcChannel  = params[testCase].mcrcChannelNumber;

            IntrMask = 0x2U;

            /* Reset the CRC channel*/
            SDL_MCRC_channelReset(instance, mcrcChannel);
            SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, params[testCase].mcrcMode);
            SDL_MCRC_addConfig(params[testCase].instance,params[testCase].mcrcChannelNumber,&SDL_MCRC_Config[testCase]);

            SDL_MCRC_getPSASigRegAddr(instance, mcrcChannel, &psaSignRegAddr);

            gTestSignBuff[0]=params[testCase].mcrcSignLow;
            gTestSignBuff[1]=params[testCase].mcrcSignHigh;

            /* Initialize and Configure MCRC channel */
            retVal = SDL_MCRC_init(params[testCase].instance,
                        params[testCase].mcrcChannelNumber,
                        params[testCase].mcrcWatchdogPreload,
                        params[testCase].mcrcBlockPreload);
            if(SDL_PASS != retVal)
            {
                DebugP_log("[Error] mcrcSemiCPU channel intialization failed!!\n");
            }

            SDL_MCRC_enableIntr(instance, mcrcChannel,IntrMask);
            retVal = SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, params[testCase].mcrcMode);

            baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
            DebugP_assert(baseAddr != 0);

            regionId = EDMA_getRegionId(gEdmaHandle[0]);
            DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

            dmaCh1 = EDMA_RESOURCE_ALLOC_ANY;
            testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh1);
            DebugP_assert(testStatus == SystemP_SUCCESS);

            tcc1 = EDMA_RESOURCE_ALLOC_ANY;
            testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc1);
            DebugP_assert(testStatus == SystemP_SUCCESS);

            param1 = EDMA_RESOURCE_ALLOC_ANY;
            testStatus = EDMA_allocParam(gEdmaHandle[0], &param1);
            DebugP_assert(testStatus == SystemP_SUCCESS);

            srcBuffPtr = (uint8_t *)gMCRCSrcBuffer;

            signBuffPtr[0] = (uint32_t)refSignVal.regL;
            signBuffPtr[1] = (uint32_t)refSignVal.regH;

            CacheP_wb((void *)signBuffPtr, MCRC_APP_CRC_PATTERN_SIZE, CacheP_TYPE_ALL);

            EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                        dmaCh1, tcc1, param1, EDMA_TEST_EVT_QUEUE_NO);

            /* Disable the interrupt for the channel*/
            EDMA_enableEvtIntrRegion(baseAddr, regionId, dmaCh1);

            /* Program Param Set */
            EDMA_ccPaRAMEntry_init(&edmaParam1);
            edmaParam1.srcAddr      = (uint32_t)SOC_virtToPhy(srcBuffPtr);
            edmaParam1.destAddr     = (uint32_t)(psaSignRegAddr.regL);
            edmaParam1.aCnt         = (uint16_t)(params[testCase].mcrcPatternSize);
            edmaParam1.bCnt         = (uint16_t)MCRC_APP_CRC_PATTERN_CNT;
            edmaParam1.cCnt         = (uint16_t)MCRC_APP_CRC_SECT_CNT;
            edmaParam1.bCntReload   = (uint16_t)MCRC_APP_CRC_PATTERN_CNT;
            edmaParam1.srcBIdx      = (int16_t)EDMA_PARAM_BIDX(params[testCase].mcrcPatternSize);
            edmaParam1.destBIdx     = (int16_t)0;
            edmaParam1.srcCIdx      = (int16_t)0;
            edmaParam1.destCIdx     = (int16_t)0;
            edmaParam1.linkAddr     = 0xFFFFU;
            edmaParam1.srcBIdxExt   = (int8_t)EDMA_PARAM_BIDX_EXT(params[testCase].mcrcPatternSize);
            edmaParam1.destBIdxExt  = 0;
            edmaParam1.opt |=
                (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
                ((((uint32_t)tcc1) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
            /* config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA_ADDRESSING_MODE_LINEAR; */
            edmaParam1.opt |= (EDMA_ADDRESSING_MODE_LINEAR << EDMA_TPCC_OPT_SAM_SHIFT) & EDMA_TPCC_OPT_SAM_MASK;
            /* config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA_ADDRESSING_MODE_FIFO_WRAP; */
            edmaParam1.opt |= (EDMA_ADDRESSING_MODE_FIFO_WRAP << EDMA_TPCC_OPT_DAM_SHIFT) & EDMA_TPCC_OPT_DAM_MASK;
            edmaParam1.opt |= (EDMA_FIFO_WIDTH_32BIT << EDMA_TPCC_OPT_FWID_SHIFT) & EDMA_TPCC_OPT_FWID_MASK;

            EDMA_setPaRAM(baseAddr, param1, &edmaParam1);

            testStatus = SemaphoreP_constructBinary(&gEdmaTransferDoneSem, 0);
            DebugP_assert(SystemP_SUCCESS == testStatus);

            /* Register interrupt */
            intrObj.tccNum = tcc1;
            intrObj.cbFxn = &EDMA_regionIsrFxn;
            intrObj.appData = (void *)&gEdmaTransferDoneSem;
            testStatus = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
            DebugP_assert(testStatus == SystemP_SUCCESS);

            semiModeTime = ClockP_getTimeUsec();
            EDMA_enableTransferRegion(baseAddr, regionId, dmaCh1,
                              EDMA_TRIG_MODE_EVENT);

            /* Trigger channel dma2 using SW */
            for (loopCnt = 0; loopCnt < (MCRC_APP_CRC_SECT_CNT); loopCnt++)
            {
                EDMA_enableTransferRegion(baseAddr, regionId, dmaCh1, EDMA_TRIG_MODE_MANUAL);
                SemaphoreP_pend(&gEdmaTransferDoneSem, SystemP_WAIT_FOREVER);
            }

            semiModeTime = ClockP_getTimeUsec() - semiModeTime;

            testStatus = EDMA_unregisterIntr(gEdmaHandle[0], &intrObj);
            SemaphoreP_destruct(&gEdmaTransferDoneSem);
            DebugP_assert(testStatus == SystemP_SUCCESS);

            /* Free channel */
            EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                    dmaCh1, EDMA_TRIG_MODE_MANUAL, tcc1, EDMA_TEST_EVT_QUEUE_NO);

            /* Free the EDMA resources managed by driver. */
            testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh1);
            DebugP_assert(testStatus == SystemP_SUCCESS);
            testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc1);
            DebugP_assert(testStatus == SystemP_SUCCESS);
            testStatus = EDMA_freeParam(gEdmaHandle[0], &param1);
            DebugP_assert(testStatus == SystemP_SUCCESS);

            intrStatus=0u;

            while (intrStatus == 0x0U)
            {
                /* Wait here till MCRC compression complete is set. */
                SDL_MCRC_getIntrStatus(instance, mcrcChannel, &intrStatus);
                DebugP_log("\r\nMCRC Semi CPU intrrupt status : %x\r\n", intrStatus);
            }

            /* Get MCRC PSA signature register address */
            SDL_MCRC_getPSASectorSig(instance, mcrcChannel, &psaSignRegVal);

            if(((refSignVal.regH == psaSignRegVal.regH) &&
            (refSignVal.regL == psaSignRegVal.regL)))
            {
                DebugP_log("\r\nSector signature matches - Passed");
                DebugP_log("\r\nCalculated MCRC signature value : 0x%08x%08xU\n",
                                    psaSignRegVal.regH,
                                    psaSignRegVal.regL);
                DebugP_log("\r\nEDMA Data transfer completed !!\r\n");
                DebugP_log("MCRC Semi Mode Computation Time: %dus\r\n", semiModeTime);
                retVal = SDL_PASS;
            }
            else
            {
                retVal = SDL_EFAIL;
                DebugP_log("\nSector signature does not match.");
                DebugP_log("\nSome tests have failed!!\r\n");
                DebugP_log("\nExpected MCRC signature value : 0x%x%xU\n",
                    refSignVal.regH,
                    refSignVal.regL);
                DebugP_log("\nCalculated MCRC signature value : 0x%08x%08xU\n",
                    psaSignRegVal.regH,
                    psaSignRegVal.regL);
            }
            SDL_MCRC_clearIntr(instance, mcrcChannel, SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL);
        }
    }
    if (retVal == SDL_PASS)
    {
        DebugP_log("\n All tests have passed. \n");
    }
    EDMA_mcrcDeinit();
    return (retVal + retVal1);
}
static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);
}

