/*
 *   Copyright (c) Texas Instruments Incorporated 2023
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
 *
 */

 /**
 *  \file     ecc_trigger.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the Error Correcting Code (ECC) Module application.
 *
 *  \details  ECC Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/include/am263px/sdlr_soc_ecc_aggr.h>
#include <sdl/sdl_ecc.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include "ecc_main.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* Value for A count*/
#define EDMA_TEST_A_COUNT           	(16U)
/* Value for B count */
#define EDMA_TEST_B_COUNT           	(4U)
/* Value for C count */
#define EDMA_TEST_C_COUNT           	(2U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      	(0U)

/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */
#define EDMA_TEST_BUFFER_SIZE     	  	(EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT)

#define SDL_MSS_MAX_MEM_SECTIONS        (1u)
#define SDL_ECC_SEC						(1U)
#define SDL_ECC_DED						(2U)

#define SDL_EXAMPLE_ECC_AGGR            SDL_SOC_ECC_AGGR
/*
*	To test other RAMID, then change the SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_ID macro according to the test RAMIDs
*  		Refer sdlr_soc_ecc_aggr.h file
*/
#define SDL_EXAMPLE_ECC_RAM_ID          SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_ID

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* The source buffer used for transfer */
static uint8_t gEdmaTestSrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
/* The destination buffer used for transfer */
static uint8_t gEdmaTestDstBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gEdmaTestDoneSem;

static SDL_ECC_MemSubType ECC_Test_MSS_subMemTypeList[SDL_MSS_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MSS_ECCInitConfig =
{
    .numRams = SDL_MSS_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MSS_subMemTypeList[0]),
    /**< Sub type list  */
};

static uint32_t arg;

SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00180000u, 0x00000000u, 0x00000000u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    /**< All events enable: except clkstop events for unused clocks
     *   and PCIE events */
    /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
    .priorityBitmap = {0x00180000u, 0x000000000u, 0x00000000u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00180000u, 0x00000000u, 0x00000000u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);
static int32_t edma_interrupt_transfer(uint32_t edmaConfigNum, uint32_t injectType, uint32_t queueType, uint32_t channelEvent);
int32_t ECC_sdlFuncTest(void);

static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*********************************************************************
* @fn      ECC_Example_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t ECC_Example_init (void)
{
	int32_t retValue=0;
    SDL_ErrType_t result;
	void *ptr = (void *)&arg;

	if (retValue == 0)
	{
		result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
		if (retValue == SDL_PASS)
		{
			DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
		}
		else {
			DebugP_log("\r\nECC_Example_init: Error initializing ESM: result = %d\r\n", result);
			retValue = -1;
		}
		esmError = false;
	}

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_MSS_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Test_init: Error initializing MSS ECC AGGR: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MSS ECC AGGR initialization is completed \n");
        }
    }
    return retValue;
}/* End of ECC_Example_init() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_TPTC_A0_1Bit_InjectTest
 *
 * @brief   Execute ECC MSS_TPTC_A0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_TPTC_A0_1Bit_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\nMSS TPTC_A0 Single bit error inject: test starting");

    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    /* Run one shot test for MSS TPTC_A0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        DebugP_log("\r\nMSS TPTC_A0 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\r\nMSS TPTC_A0 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}/* End of ECC_Test_run_MSS_TPTC_A0_1Bit_InjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_TPTC_A0_2Bit_InjectTest
 *
 * @brief   Execute ECC MSS_TPTC_A0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_TPTC_A0_2Bit_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\nMSS TPTC_A0 Double bit error inject: starting");

    /* Run one shot test for MSS TPTC_A0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        DebugP_log("\r\nMSS TPTC_A0 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\r\nMSS TPTC_A0 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_TPTC_A0_2Bit_InjectTest() */

/*********************************************************************
 * @fn      edma_interrupt_transfer
 *
 * @brief   Execute EDMA interrupt transfer and ECC inject test
 *
 * @param   edmaConfig  : EDMA channel
 * @param   injectType  : SEC or DED inject
 * @param   queueType   : queue number
 * @param   channelEvent: Channel event number
 *
 * @return  SDL_PASS or SDL_EFAIL
 ********************************************************************/
static int32_t edma_interrupt_transfer(uint32_t edmaConfigNum, uint32_t injectType, uint32_t queueType, uint32_t channelEvent)
{
    uint32_t            baseAddr, regionId;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            loopCnt = 0;
    uint8_t            *srcBuffPtr, *dstBuffPtr;
    EDMACCPaRAMEntry   edmaParam;
    Edma_IntrObject     intrObj;
    uint32_t            dmaCh, tcc, param;
	int32_t             result = SDL_PASS;

    DebugP_log("\r\n[EDMA] Interrupt Transfer Test Started...\r\n");
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[edmaConfigNum]);

    regionId = EDMA_getRegionId(gEdmaHandle[edmaConfigNum]);

    dmaCh = channelEvent;
    status = EDMA_allocDmaChannel(gEdmaHandle[edmaConfigNum], &dmaCh);

    tcc = channelEvent;
    status = EDMA_allocTcc(gEdmaHandle[edmaConfigNum], &tcc);

    param = channelEvent;
    status = EDMA_allocParam(gEdmaHandle[edmaConfigNum], &param);

    /*ECC Init*/
    ECC_Example_init();

	if(injectType == SDL_ECC_SEC)
	{
		ECC_Test_run_MSS_TPTC_A0_1Bit_InjectTest();
	}
	else if(injectType == SDL_ECC_DED)
	{
		ECC_Test_run_MSS_TPTC_A0_2Bit_InjectTest();
	}
	else{
		/* No ECC inject */
	}

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;
    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
        dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param, queueType);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam.bCntReload    = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSem, 0);

    /* Register interrupt */
    intrObj.tccNum = tcc;
    intrObj.cbFxn  = &EDMA_regionIsrFxn;
    intrObj.appData = (void *) &gEdmaTestDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[edmaConfigNum], &intrObj);

    /*
     * Transfer is done in A sync mode
     * Number of triggers required are B_COUNT * C_COUNT
     */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMA_enableTransferRegion(
            baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            DebugP_log("Error matching value at src and dst address %d\r\n", loopCnt);
            status = SystemP_FAILURE;
			result = SDL_EFAIL;
            break;
        }
    }

    status = EDMA_unregisterIntr(gEdmaHandle[edmaConfigNum], &intrObj);
    SemaphoreP_destruct(&gEdmaTestDoneSem);

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[edmaConfigNum], &dmaCh);
    status = EDMA_freeTcc(gEdmaHandle[edmaConfigNum], &tcc);
    status = EDMA_freeParam(gEdmaHandle[edmaConfigNum], &param);

	if(status == SystemP_SUCCESS)
    {
        DebugP_log("\r\n[EDMA] Interrupt Transfer Test Completed!!\r\n");
        if(esmError == TRUE)
        {
            DebugP_log("\r\nAll tests have passed!!\r\n");
            esmError = false;
        }
        else
        {
            result = SDL_EFAIL;
            DebugP_log("\r\nESM interrupt is not occurred.... Test is failed!!\r\n");
        }
    }
    else
    {
        result = SDL_EFAIL;
        DebugP_log("\r\nSome tests have failed!!\r\n");
    }
    return(result);
}

static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    SemaphoreP_post(semObjPtr);
}


/*********************************************************************
 * @fn      sdlApp_dplInit
 *
 * @brief   Initialization of DPL
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("\r\nError: Init Failed\r\n");
    }

    return ret;
}

/* ECC Function module test */
int32_t ECC_funcTest(void)
{
	int32_t testResult = SDL_PASS;
    /*Initializing the DPL*/
    sdlApp_dplInit();

	if(testResult == SDL_PASS)
    {
		/* EDMA transfer with ECC single bit injection*/
		testResult = edma_interrupt_transfer(CONFIG_EDMA0, SDL_ECC_SEC, 0u, EDMA_RESOURCE_ALLOC_ANY);
	}

    return (testResult);
}

/* Nothing past this point */
