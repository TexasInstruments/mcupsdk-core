/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 * This example performs CBUFF streaming test using sw trigger mode.
 *
 *
 */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include <drivers/cbuff.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <string.h>

/**
 * @brief
 *  The DCA1000EVM FPGA needs a minimum delay of 12ms between Bit clock starts and
 *  actual LVDS Data start to lock the LVDS PLL IP. This is documented in the DCA UG
 */
#define HSI_DCA_MIN_DELAY_MSEC                  (12U * 1000U)

/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */
#define CBUFF_TEST_BUFFER_SIZE                  (4 * 1024U)

/**
 * @brief   Number of EDMA Channels which have been allocated and can be used
 * by the CBUFF Driver.
 */
#define TEST_APP_MAX_EDMA_TABLE_ENTRIES             (3U)

/* The source buffer used for transfer */
static uint8_t gUserBuffer[CBUFF_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/**
 * @brief   Global variable to track the SW trigger Frame Done interrupt count.
 */
 volatile uint32_t  gCBUFFSwTriggerFrameDoneCounter = 0;

  /**
  * @brief   Global variable which tracks the EDMA Channels which have been allocated to the
  * CBUFF.
  */
 volatile uint8_t  gCBUFFEDMAChannelResourceCounter = 0;

/**
 * @brief   Global CBUFF EDMA Channel Resource Table:
 */
static CBUFF_EDMAChannelCfg gCBUFFEDMAChannelResource[TEST_APP_MAX_EDMA_TABLE_ENTRIES] =
{
    /* EDMA Channel Identifier, Shadow Link Channel Identifier */
    {  EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ0,  65 },
    {  EDMA_DSS_TPCC_A_EVT_FREE_0,   66 },
    {  EDMA_DSS_TPCC_A_EVT_FREE_1,   67 }
};

/**
 *  @b Description
 *  @n
 *      This is the registered function which is hooked up with the
 *      CBUFF driver to indicate frame done interrupt.
 *
 *  @retval
 *      Not applicable
 */
static void Test_SwTriggerFrameDone (CBUFF_SessionHandle sessionHandle)
{
    /* Increment stats*/
    gCBUFFSwTriggerFrameDoneCounter++;
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the registered free function which is hooked up with the
 *      CBUFF driver to free the allocated EDMA Channels
 *
 *  @retval
 *      Not applicable
 */
void Test_EDMAFreeCBUFFChannel(CBUFF_EDMAChannelCfg* ptrEDMACfg)
{
    /* Debug Message: */
    gCBUFFEDMAChannelResourceCounter--;
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the registered allocation function which is hooked up with the
 *      CBUFF driver to allocate EDMA Channels
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Test_EDMAAllocateCBUFFChannel (CBUFF_EDMAInfo* ptrEDMAInfo, CBUFF_EDMAChannelCfg* ptrEDMAChannelCfg)
{
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    int32_t             retVal = SystemP_SUCCESS;

    /* NO: Have we allocated all the EDMA channels? */
    if (gCBUFFEDMAChannelResourceCounter >= TEST_APP_MAX_EDMA_TABLE_ENTRIES)
    {
        /* Error: Exceeded the allocated table. Failure */
        DebugP_assert (0);
    }

    /* Special case handling: First EDMA channel which is being allocated */
    if (ptrEDMAInfo->isFirstEDMAChannel)
    {
        ptrEDMAChannelCfg->chainChannelsId      = gCBUFFEDMAChannelResource[0].chainChannelsId;

        testStatus = EDMA_allocDmaChannel(gEdmaHandle[CONFIG_EDMA0], &ptrEDMAChannelCfg->chainChannelsId );
        DebugP_assert(testStatus == SystemP_SUCCESS);

        testStatus = EDMA_allocTcc(gEdmaHandle[CONFIG_EDMA0], &ptrEDMAChannelCfg->chainChannelsId );
        DebugP_assert(testStatus == SystemP_SUCCESS);

        testStatus = EDMA_allocParam(gEdmaHandle[CONFIG_EDMA0], &ptrEDMAChannelCfg->chainChannelsId );
        DebugP_assert(testStatus == SystemP_SUCCESS);

        baseAddr = EDMA_getBaseAddr(gEdmaHandle[CONFIG_EDMA0]);
        DebugP_assert(baseAddr != 0);

        regionId = EDMA_getRegionId(gEdmaHandle[CONFIG_EDMA0]);
        DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

        /* Request channel */
        EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            ptrEDMAChannelCfg->chainChannelsId, ptrEDMAChannelCfg->chainChannelsId, ptrEDMAChannelCfg->chainChannelsId , 0);


        /* Allocate Shadow EDMA channel. */
        ptrEDMAChannelCfg->shadowLinkChannelsId = gCBUFFEDMAChannelResource[0].shadowLinkChannelsId;;

        testStatus = EDMA_allocParam(gEdmaHandle[CONFIG_EDMA0], &ptrEDMAChannelCfg->shadowLinkChannelsId );
        DebugP_assert(testStatus == SystemP_SUCCESS);
    }
    else
    {
        /* Copy over the allocated EDMA configuration. */
        memcpy ((void *)ptrEDMAChannelCfg, (void*)&gCBUFFEDMAChannelResource[gCBUFFEDMAChannelResourceCounter],
                sizeof(CBUFF_EDMAChannelCfg));
    }

    /* Increment the number of EDMA Channels which have been allocated */
    gCBUFFEDMAChannelResourceCounter++;

    return retVal;
}


void cbuff_sw_trigger(void *args)
{
    CBUFF_InitCfg           initCfg;
    int32_t                 errCode;
    int32_t                 testStatus = SystemP_SUCCESS;
    CBUFF_Handle            cbuffHandle;
    CBUFF_SessionCfg        sessionCfg;
    CBUFF_SessionHandle     sessionHandle;
    uint32_t                index;

    /* Configure HSI interface Clock */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL, 0x222);
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_HSI_DIV_VAL, 0x333);

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

	DebugP_log("[CBUFF] SW Trigger Test Started...\r\n");

    /*************************************************************************************
     * Open the CBUFF Driver:
     *************************************************************************************/
    memset ((void *)&initCfg, 0, sizeof(CBUFF_InitCfg));

    /* Populate the configuration: */
    initCfg.enableECC                 = 0U;
    initCfg.crcEnable                 = 1U;
    /* Up to 1 SW session + 1 HW session can be configured for each frame. Therefore max session is 2. */
    initCfg.maxSessions               = 2U;
    initCfg.enableDebugMode           = false;
    initCfg.interface                 = CBUFF_Interface_LVDS;
    initCfg.outputDataFmt             = CBUFF_OutputDataFmt_16bit;
    initCfg.lvdsCfg.crcEnable         = 0U;
    initCfg.lvdsCfg.msbFirst          = 1U;
    /* Enable all lanes available on the platform*/
    initCfg.lvdsCfg.lvdsLaneEnable    = 0x3U;
    initCfg.lvdsCfg.ddrClockMode      = 1U;
    initCfg.lvdsCfg.ddrClockModeMux   = 1U;

    /* Initialize the CBUFF Driver: */
    cbuffHandle = CBUFF_open (&initCfg, &errCode);
    if (cbuffHandle == NULL)
    {
        /* Error: Unable to initialize the CBUFF Driver */
        DebugP_log("Error: CBUFF_open failed with [Error=%d]\n", errCode);
    }

    /*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
      This is needed because the DCA1000EVM FPGA needs the delay to lock to the
      bit clock before they can start capturing the data correctly. */
    ClockP_usleep(HSI_DCA_MIN_DELAY_MSEC);

    /* Initialize the configuration */
    memset ((void*)&sessionCfg, 0, sizeof(CBUFF_SessionCfg));

    /* Initialize the configuration: */
    for(index = 0; index < CBUFF_TEST_BUFFER_SIZE; index++)
    {
    	gUserBuffer[index] = index;
    }
    CacheP_wb((void *)gUserBuffer, CBUFF_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Populate the configuration: */
    // sessionCfg.executionMode                      = CBUFF_SessionExecuteMode_SW;
    sessionCfg.edmaHandle                         = gEdmaHandle[CONFIG_EDMA0];
    sessionCfg.allocateEDMAChannelFxn             = Test_EDMAAllocateCBUFFChannel;
    sessionCfg.freeEDMAChannelFxn                 = Test_EDMAFreeCBUFFChannel;
    sessionCfg.frameDoneCallbackFxn               = Test_SwTriggerFrameDone;
    sessionCfg.dataType                           = CBUFF_DataType_REAL;
    sessionCfg.swCfg.userBufferInfo[0].size       = sizeof(gUserBuffer)/2;
    sessionCfg.swCfg.userBufferInfo[0].address    = (uint32_t)&gUserBuffer[0];

    /* Create the Session: */
    sessionHandle = CBUFF_createSession (cbuffHandle, &sessionCfg, &errCode);
    if (sessionHandle == NULL)
    {
        DebugP_log ("Error: Unable to create the session [Error code %d]\r\n", errCode);
    }

    /* Activate the session: */
    if (CBUFF_activateSession (sessionHandle, &errCode) < 0)
    {
        DebugP_log ("Error: Unable to activate the session [Error code %d]\r\n", errCode);
    }

    while (gCBUFFSwTriggerFrameDoneCounter != 1)
    {
        DebugP_log("waiting for frameDone interrupt : %d\r\n", gCBUFFSwTriggerFrameDoneCounter);
        ClockP_usleep(1 * 1000);
    }

    DebugP_log("Received frameDone interrupt : %d\r\n", gCBUFFSwTriggerFrameDoneCounter);

    DebugP_log("Data is transmitted over LVDS successfully.\r\n");

    /* Deactivate the session: */
    if (CBUFF_deactivateSession (sessionHandle, &errCode) < 0)
    {
        DebugP_log ("Error: Unable to deactivate the session [Error code %d]\r\n", errCode);
    }

    /* Delete the session: */
    if (CBUFF_close (sessionHandle, &errCode) < 0)
    {
        DebugP_log ("Error: Unable to delete the session [Error code %d]\r\n", errCode);
    }

    if(testStatus == SystemP_SUCCESS)
    {
        DebugP_log("[CBUFF] SW Trigger Test Completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

	Board_driversClose();
    Drivers_close();

    return;
}
