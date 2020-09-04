/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \file mcrc_autoCPU.c
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
#define EDMA_TEST_A_COUNT           (4U)
/* Value for B count */
#define EDMA_TEST_B_COUNT           (2U)
/* Value for C count */
#define EDMA_TEST_C_COUNT           (1U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)

#define CONFIG_EDMA0                (0U)
#define MCRC_BUF_MAX_SIZE           (10000U)

#define EDMA_TEST_BUFFER_SIZE       (EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT)
/** \brief Number of times to perform the MCRC operation */
#define LOOP_COUNT          		(1U)
/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */
static uint8_t gMCRCSrcBuffer[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(128), section(".bss:extMemCache:ramdisk"))) = {1U};
static uint8_t gMCRCDestBuffer[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(128), section(".bss:extMemCache:ramdisk"))) = {1U};

/** Defines the various MCRC use cases. */
static    SDL_MCRC_ConfigParams_t params[2] =
{
    {
#if defined(SOC_AM263X)
     MCRC0,
#endif
#if defined(SOC_AM273X)||(SOC_AWR294X)
     MCRC_INSTANCE,
#endif
        (uint32_t) SDL_MCRC_CHANNEL_1,
        (uint32_t) SDL_MCRC_OPERATION_MODE_AUTO,
        4U,
        MCRC_DEF_PATTERN_COUNT,
        MCRC_DEF_SECTOR_COUNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x65e09d48U,
		0x6837b1e1U,
        MCRC_BUF_MAX_SIZE,
		(uint32_t) &gMCRCSrcBuffer[0],
    },

    {
#if defined(SOC_AM263X)
     MCRC0,
#endif
#if defined(SOC_AM273X)||(SOC_AWR294X)
     MCRC_INSTANCE,
#endif
        (uint32_t) SDL_MCRC_CHANNEL_1,
        (uint32_t) SDL_MCRC_OPERATION_MODE_AUTO,
        4U,
        MCRC_DEF_PATTERN_COUNT,
        MCRC_DEF_SECTOR_COUNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x65e09d48U,
		0x6837b1e1U,
        MCRC_BUF_MAX_SIZE,
		(uint32_t) &gMCRCSrcBuffer[0],
    },
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

int32_t mcrcAutoCPU_main(void)
{
	int32_t         			retVal = SDL_PASS;
    int32_t         			retVal1 = SDL_PASS;
	uint32_t 					testCase = 0;
    SDL_MCRC_Signature_t        sectSignVal, refSignVal;
    SDL_MCRC_SignatureRegAddr_t psaSignRegAddr, refSignRegAddr;
    uint32_t              		patternCnt, sectCnt;
    SDL_MCRC_InstType 			instance;
    SDL_MCRC_Channel_t          mcrcChannel;
    uint32_t 					IntrMask = 0x1U;
    uint32_t    				i;
    uint32_t    				dataSize;
    uint32_t   					*mcrcSourceMemory, *mcrcDestMemory;
	/* EDMA driver init */
    uint32_t            		baseAddr, regionId;
    int32_t             		testStatus = SystemP_SUCCESS;
    uint32_t            		loopCnt = 0;
    EDMACCPaRAMEntry   			edmaParam1, edmaParam2;
    uint32_t            		dmaCh0, tcc0, param0, dmaCh1, tcc1, param1;

    EDMA_mcrcInit();
    DebugP_log("MCRC AUTO CPU mode: Transfer Test Started...\r\n");

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh0 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc0 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param0 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param0);
    DebugP_assert(testStatus == SystemP_SUCCESS);
	
    dmaCh1 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc1 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param1 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

	for(testCase=0; testCase <= 1; testCase++)
	{
		/* Configure MCRC channel */
        retVal = SDL_MCRC_init(params[testCase].instance,
                    params[testCase].mcrcChannelNumber,
                    params[testCase].mcrcWatchdogPreload,
                    params[testCase].mcrcBlockPreload);
        if(SDL_PASS != retVal)
        {
             DebugP_log("[Error] mcrcAutoCPU channel intialization failed!!\n");
        }

		sectSignVal.regL = 0U;
		sectSignVal.regH = 0U;
		patternCnt  = params[testCase].dataSize / params[testCase].mcrcPatternSize;
		sectCnt     = params[testCase].mcrcSectorCount;
		instance = params[testCase].instance;
		mcrcChannel  = params[testCase].mcrcChannelNumber;

		/* Get MCRC PSA signature register address */
		SDL_MCRC_getPSASigRegAddr(instance, mcrcChannel, &psaSignRegAddr);
		SDL_MCRC_getPSASigRegAddr(instance, mcrcChannel, &refSignRegAddr);
		retVal = SDL_MCRC_config(instance, mcrcChannel, patternCnt, sectCnt, params[testCase].mcrcMode);

		if (retVal == SDL_PASS)
		{
			retVal = SDL_MCRC_enableIntr(instance, mcrcChannel,IntrMask);
		}
		
		/* Init buffers */
		mcrcSourceMemory = (uint32_t *) params[testCase].sourceMemory; 
		dataSize = (params[testCase].dataSize);
		mcrcDestMemory = (uint32_t *)gMCRCDestBuffer;
		for (i = 0; i < (EDMA_TEST_BUFFER_SIZE/2); i++)
		{
			mcrcSourceMemory[i] = (uint32_t)i;
			mcrcDestMemory[i]= 0U;
		}
		mcrcDestMemory[0] = params[testCase].mcrcSignLow;
		mcrcDestMemory[1] = params[testCase].mcrcSignHigh;
		DebugP_log("\nCopied reference data into memory.");
		CacheP_wb((void *)mcrcSourceMemory, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
		CacheP_wb((void *)mcrcDestMemory, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

		/* Get the reference crc sign value. The reference crc sign value is retrieved
		by performing auto cpu mode CRC on the same set of data used for autoCPU mode.
		This is done in the test app to get the reference value for the test, but is
		not required for using auto-CPU mode. */
		if ((0U == params[testCase].mcrcSignHigh) &&
			(0U == params[testCase].mcrcSignLow))
		{
			SDL_MCRC_InstType instance = params[testCase].instance;
			SDL_MCRC_Channel_t mcrcChannelNumber = params[testCase].mcrcChannelNumber;
			uint32_t mcrcPatternSize       = params[testCase].mcrcPatternSize;
			uint32_t mcrcPatterCount       = (params[testCase].dataSize) / mcrcPatternSize;
			uint32_t mcrcSectorCount       = params[testCase].mcrcSectorCount;
			SDL_MCRC_SignatureRegAddr_t mcrcPSASignatureRegAddress;
			uint32_t forLoopCount;

			/* Configure the MCRC module in Auto CPU mode and get reference MCRC. */
			DebugP_log("\nCalculating Reference MCRC signature Value.");
			/* MCRC channel RESET before initialization/configuration. */
			retVal = SDL_MCRC_channelReset(instance, mcrcChannelNumber);
			
			if (retVal == SDL_PASS)
			{
				/* Initialize MCRC channel */
				retVal = SDL_MCRC_config(instance,
									mcrcChannelNumber,
									mcrcPatterCount,
									mcrcSectorCount,
									SDL_MCRC_OPERATION_MODE_AUTO);
			}

			if (retVal == SDL_PASS)
			{
				retVal = SDL_MCRC_enableIntr(instance, mcrcChannelNumber,IntrMask);
			}

			if (retVal == SDL_PASS)
			{
				/* Get MCRC PSA signature register address */
				retVal = SDL_MCRC_getPSASigRegAddr(instance,
											mcrcChannelNumber,
											&mcrcPSASignatureRegAddress);
			}

			if (retVal == SDL_PASS)
			{
				for (forLoopCount = 0;
					forLoopCount < (dataSize / mcrcPatternSize);
					forLoopCount++)
				{
					HW_WR_REG32(mcrcPSASignatureRegAddress.regL,
								mcrcSourceMemory[forLoopCount]);
					
					
				}
				/* Fetch MCRC signature value       */
				retVal = SDL_MCRC_getPSASectorSig(instance,
											mcrcChannelNumber,
											&refSignVal);
			}
			if (retVal == SDL_PASS)
			{
				DebugP_log("\n MCRC signature value : 0x%x%xU",
							refSignVal.regH,
							refSignVal.regL);
			}
		}
		else
		{
			DebugP_log("\nUsing Pre-Defined Reference MCRC signature Value.");
			refSignVal.regH = params[testCase].mcrcSignHigh;
			refSignVal.regL = params[testCase].mcrcSignLow;
		}

		if (retVal == SDL_PASS)
		{
			loopCnt=0;
			while(loopCnt < LOOP_COUNT)
			{
				/* Perform EDMA MCRC */
				/* Request channel */
				EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
						dmaCh0, tcc0, param0, EDMA_TEST_EVT_QUEUE_NO);
				EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
						dmaCh1, tcc1, param1, EDMA_TEST_EVT_QUEUE_NO);
	
				/* Disable the interrupt for the channel*/
				EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh0);
				EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh1);
				
				/* Program Param Set */
				EDMA_ccPaRAMEntry_init(&edmaParam1);
				edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy(mcrcSourceMemory);
				edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy((uint32_t *)psaSignRegAddr.regL);
				edmaParam1.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
				edmaParam1.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
				edmaParam1.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
				edmaParam1.bCntReload    = 0;
				edmaParam1.srcBIdx       = (int16_t) EDMA_TEST_A_COUNT;
				edmaParam1.destBIdx      = (int16_t) EDMA_TEST_A_COUNT;
				edmaParam1.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
				edmaParam1.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
				edmaParam1.linkAddr      = 0xFFFFU;
				edmaParam1.opt          |=
						(EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
						((((uint32_t)tcc0) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
						
				EDMA_ccPaRAMEntry_init(&edmaParam2);
				edmaParam2.srcAddr       = (uint32_t) SOC_virtToPhy(mcrcDestMemory);
				edmaParam2.destAddr      = (uint32_t) SOC_virtToPhy((uint32_t *)refSignRegAddr.regL);
				edmaParam2.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
				edmaParam2.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
				edmaParam2.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
				edmaParam2.bCntReload    = 0;
				edmaParam2.srcBIdx       = (int16_t) EDMA_TEST_A_COUNT;
				edmaParam2.destBIdx      = (int16_t) EDMA_TEST_A_COUNT;
				edmaParam2.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
				edmaParam2.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
				edmaParam2.linkAddr      = 0xFFFFU;
				edmaParam2.opt          |=
						(EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
						((((uint32_t)tcc1) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
						
				EDMA_setPaRAM(baseAddr, param0, &edmaParam1);
				EDMA_setPaRAM(baseAddr, param1, &edmaParam2);
				/*
				* Transfer is done in AB sync mode
				* Number of triggers required is C_COUNT
				*/
				for(loopCnt = 0; loopCnt < (EDMA_TEST_C_COUNT); loopCnt++)
				{
					EDMA_enableTransferRegion(baseAddr, regionId, dmaCh0,
						EDMA_TRIG_MODE_MANUAL);
					EDMA_enableTransferRegion(baseAddr, regionId, dmaCh1,
						EDMA_TRIG_MODE_MANUAL);

					while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc0) != 1);
					while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc1) != 1);

					EDMA_clrIntrRegion(baseAddr, regionId, tcc0);
					EDMA_clrIntrRegion(baseAddr, regionId, tcc1);
				}
				
				/* Free channel */
				EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
					dmaCh0, EDMA_TRIG_MODE_MANUAL, tcc0, EDMA_TEST_EVT_QUEUE_NO);
				EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
					dmaCh1, EDMA_TRIG_MODE_MANUAL, tcc1, EDMA_TEST_EVT_QUEUE_NO);

				/* Free the EDMA resources managed by driver. */
				testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh0);
				DebugP_assert(testStatus == SystemP_SUCCESS);
				testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc0);
				DebugP_assert(testStatus == SystemP_SUCCESS);
				testStatus = EDMA_freeParam(gEdmaHandle[0], &param0);
				DebugP_assert(testStatus == SystemP_SUCCESS);
				testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh1);
				DebugP_assert(testStatus == SystemP_SUCCESS);
				testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc1);
				DebugP_assert(testStatus == SystemP_SUCCESS);
				testStatus = EDMA_freeParam(gEdmaHandle[0], &param1);
				DebugP_assert(testStatus == SystemP_SUCCESS);

				if(testStatus != SystemP_FAILURE)
				{
					DebugP_log("\nTest is completed");
				}
				if (SDL_PASS == retVal)
				{
					retVal = SDL_MCRC_getPSASectorSig(instance, mcrcChannel, &sectSignVal);
					retVal = SDL_MCRC_getPSASectorSig(instance, mcrcChannel, &refSignVal);
				}
				if (SDL_PASS == retVal)
				{
				/* Compare MCRC signature value against reference MCRC signature */
					if((sectSignVal.regH == refSignVal.regH) &&
					(sectSignVal.regL == refSignVal.regL))
					{
						DebugP_log("\nSector signature matches - Passed");
						DebugP_log("\nEDMA Data transfer completed !!\r\n");
						DebugP_log("All tests have passed!!\r\n");
					}
					else
					{
						retVal = SDL_EFAIL;
						DebugP_log("\nSector signature does not match.");
						DebugP_log("\nSome tests have failed!!\r\n");
						DebugP_log("\nExpected MCRC signature value : 0x%x%xU",
							refSignVal.regH,
							refSignVal.regL);
						DebugP_log("\nCalculated MCRC signature value : 0x%08x%08xU",
							sectSignVal.regH,
							sectSignVal.regL);
					}
					retVal = SDL_MCRC_clearIntr(instance, mcrcChannel, SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL);
				}
				else
				{
					DebugP_log("\nMCRC signature verification failed.");
					retVal = SDL_EFAIL;
				}
				if(SDL_PASS != retVal)
				{
					break;
				}

				loopCnt++;
			}
		}
	}
    EDMA_mcrcDeinit();
	return (retVal + retVal1);
}
