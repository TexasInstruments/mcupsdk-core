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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example performs FSI TX to FSI RX internal loopback in DMA mode.
 * The application configures an instance of FSI TX and FSI RX module with below configuration
 *
 * - Single lane
 * - TX clock at 50 MHz
 * - 16 words per frame (transfer). Tag and user data is 1 word each per frame
 * - Registers both FSI TX DMA and FSI RX DMA edma interrupts
 * - Set up two DMA channels to be triggered by the same FSI transmitter and DMA trigger.
 * - Configure one channel to fill the transmit buffer.
 * - Configure the other channel to set the frame tag and user data fields
 * - Similar configuration for RX to receive data and frame tag.
 * - 4 DMA Channels required to transmit and receive.
 *
 * With above configuration, the application transfers 100 frames of data from FSI TX,
 * waits for data to be received by FSI RX. EDMA completion interrupt is configured
 * once all the data, tag and user data is transmitted/received.
 *
 * Once the transfer completes, it compares the source and destination buffers for any data mismatch.
 */

/* FSI TXCLK - 50 MHz */
#define FSI_APP_TXCLK_FREQ              (50 * 1000 * 1000)
/* FSI module input clock - 500 MHz */
#define FSI_APP_CLK_FREQ                (CONFIG_FSI_TX0_CLK)
/* FSI TX prescaler value for TXCLKIN of 100 MHz. / 2 is provided as TXCLK = TXCLKIN/2 */
#define FSI_APP_TX_PRESCALER_VAL        (FSI_APP_CLK_FREQ / FSI_APP_TXCLK_FREQ / 2U)

#define FSI_APP_LOOP_COUNT              (100U)
/* User data to be sent with Data frame */
#define FSI_APP_TX_USER_DATA            (0x07U)
/* Configuring Frame - can be between 1-16U */
#define FSI_APP_FRAME_DATA_WORD_COUNT   (16U)
/* 0x0U for 1 lane and 0x1U for two lane */
#define FSI_APP_N_LANES                 (0x0U)
#define FSI_APP_TX_DATA_FRAME_TAG       (0x1U)

/* Event queue to be used for EDMA transfer */
#define EDMA_TEST_EVT_QUEUE_NO          (0U)

/* DMA channel number */
#define FSI_TX_EDMA_CHANNEL0       (DMA_TRIG_XBAR_EDMA_MODULE_0)
#define FSI_TX_EDMA_CHANNEL1       (DMA_TRIG_XBAR_EDMA_MODULE_1)
#define FSI_RX_EDMA_CHANNEL0       (DMA_TRIG_XBAR_EDMA_MODULE_2)
#define FSI_RX_EDMA_CHANNEL1       (DMA_TRIG_XBAR_EDMA_MODULE_3)

/* Index of FSI TX/RX buffer, gBudIdx + FSI_APP_FRAME_DATA_WORD_COUNT should be <= 16 */
uint16_t gRxBufData[FSI_APP_FRAME_DATA_WORD_COUNT * FSI_APP_LOOP_COUNT] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
/* Tag and User data is per frame */
uint16_t gRxBufTagAndUserData[FSI_APP_LOOP_COUNT] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint16_t gTxBufData[FSI_APP_FRAME_DATA_WORD_COUNT * FSI_APP_LOOP_COUNT] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint16_t gTxBufTagAndUserData[FSI_APP_LOOP_COUNT] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

static SemaphoreP_Object gFsiDmaTxSemObject, gFsiDmaRxSemObject;
/* EDMA interrupt objects should be always declared global */
Edma_IntrObject     gTxIntrObj;
Edma_IntrObject     gRxIntrObj;

static int32_t  Fsi_appDmaTxConfig(uint32_t txBaseAddr);
static int32_t  Fsi_appDmaRxConfig(uint32_t rxBaseAddr);
static int32_t  Fsi_appDmaRxIntrInit(uint32_t tccAlloc);
static int32_t  Fsi_appDmaTxIntrInit(uint32_t tccAlloc);
static void     Fsi_appDmaIntrDeInit(void);
static void     Fsi_appDmaTxCallback(Edma_IntrHandle intrHandle, void *args);
static void     Fsi_appDmaRxCallback(Edma_IntrHandle intrHandle, void *args);
static int32_t  Fsi_appCompareData(uint16_t *txBufPtr, uint16_t *rxBufPtr);
static uint16_t Fsi_appDmaConfigure(EDMA_Handle dma_handle, uint32_t *dmaCh,
                void *src, void *dst, uint32_t *tcc, uint32_t *param, uint32_t regionId,
                uint32_t aCnt, uint32_t bCnt, uint32_t cCnt, uint32_t srcBIdx, uint32_t destBIdx,
                uint32_t srcCIdx, uint32_t destCIdx, uint32_t triggerMode);

void *fsi_loopback_dma_main(void *args)
{
    int32_t     status;
    uint32_t    rxBaseAddr, txBaseAddr, regionId, edmaBaseAddr;
    uint16_t    numWords, bufIdx, edmaStatus;
    uint32_t    appLoopCnt, tccAlloc0, tccAlloc1;
    uint32_t    param0, param1, param2, param3;
    uint32_t    dmaCh0, dmaCh1, dmaCh2, dmaCh3;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[FSI] Loopback Dma application started ...\r\n");

    /* Test parameters */
    rxBaseAddr = CONFIG_FSI_RX0_BASE_ADDR;
    txBaseAddr = CONFIG_FSI_TX0_BASE_ADDR;
    numWords   = FSI_APP_FRAME_DATA_WORD_COUNT;
    appLoopCnt = FSI_APP_LOOP_COUNT;
    bufIdx     = 0U;

    /* Memset TX buffer with new data for every loop */
    for(uint32_t i = 0; i < (numWords * appLoopCnt); i++)
    {
        gTxBufData[i] = i + 1U;
        gRxBufData[i] = 0U;
    }
    /* Configure Frame Tag and User Data */
    for(uint32_t i = 0; i < appLoopCnt; i++)
    {
        gTxBufTagAndUserData[i] = (((FSI_APP_TX_DATA_FRAME_TAG << CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_FRAME_TAG_SHIFT) & CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_FRAME_TAG_MASK) |
                             ((FSI_APP_TX_USER_DATA << CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_USER_DATA_SHIFT) & CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_USER_DATA_MASK));
        gRxBufTagAndUserData[i] = 0U;
    }

    /* Perform a cache write back to the result buffers */
    CacheP_wb((void *)gTxBufData, ((FSI_APP_FRAME_DATA_WORD_COUNT * FSI_APP_LOOP_COUNT) * sizeof(uint16_t)), CacheP_TYPE_ALL);
    CacheP_wb((void *)&gTxBufTagAndUserData, (FSI_APP_LOOP_COUNT * sizeof(uint16_t)) , CacheP_TYPE_ALL);
    CacheP_wb((void *)gRxBufData, ((FSI_APP_FRAME_DATA_WORD_COUNT * FSI_APP_LOOP_COUNT) * sizeof(uint16_t)), CacheP_TYPE_ALL);
    CacheP_wb((void *)&gRxBufTagAndUserData, (FSI_APP_LOOP_COUNT * sizeof(uint16_t)), CacheP_TYPE_ALL);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* EDMA Params Description
     * tcc  - The tcc number on which the completion interrupt is generated
     * aCnt - Number of bytes in each word
     * bCnt - Number of words in each frame
     * cCnt - Number of frames
     * srcBIdx  - Index between consecutive words of a Source Buffer
     * destBIdx - Index between consecutive words of a Destination Buffer
     * srcCIdx  - Index between consecutive words of a Source Block
     * destCIdx - Index between consecutive words of a Destination Block
     * Fsi_appDmaConfigure(EDMA_Handle dma_handle, uint32_t dma_ch,
                void *src, void *dst, uint32_t *tccAlloc,
                uint32_t aCnt, uint32_t bCnt, uint32_t cCnt,
                uint32_t srcBIdx, uint32_t destBIdx, uint32_t srcCIdx,
                uint32_t destCIdx, uint32_t triggerMode) */

    dmaCh0 = FSI_TX_EDMA_CHANNEL0;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh0);
    DebugP_assert(status == SystemP_SUCCESS);

    param0 = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param0);
    DebugP_assert(status == SystemP_SUCCESS);

    /* In FSI, as soon as the DMA TX trigger is started, whatever data is available in the
     * internal FSI buffer is transferred immediately before the DMA actually transfers from
     * external buffer to FSI internal buffer. Filed a bug MCUSDK-4059 to investigate further.
     * Please refer release notes */
    /* As a workaround, first iteration we are manually copying onto internal buffer
     * before FSI DMA event is enabled and for the remaining iterations it is done based on FSI Event */
    Fsi_appDmaConfigure(gEdmaHandle[0], &dmaCh0, (void *)gTxBufData,
                       ((void *)CONFIG_FSI_TX0_BASE_ADDR + CSL_FSI_TX_CFG_TX_BUF_BASE(bufIdx)),
                         NULL, &param0, regionId, sizeof(uint16_t), FSI_APP_FRAME_DATA_WORD_COUNT, 1U,
                         sizeof(uint16_t), sizeof(uint16_t), 0U, 0U, EDMA_TRIG_MODE_MANUAL);

    dmaCh1 = FSI_TX_EDMA_CHANNEL1;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh1);
    DebugP_assert(status == SystemP_SUCCESS);

    param1 = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param1);
    DebugP_assert(status == SystemP_SUCCESS);

    tccAlloc0 = EDMA_RESOURCE_ALLOC_ANY;
    edmaStatus = EDMA_allocTcc(gEdmaHandle[0], &tccAlloc0);
    DebugP_assert(edmaStatus == SystemP_SUCCESS);

    /* In case of Frame Tag and User Data, bCnt is 1 as its a 16 bit field */   
    Fsi_appDmaConfigure(gEdmaHandle[0], &dmaCh1, (void *)&gTxBufTagAndUserData,
                        (void *)(CONFIG_FSI_TX0_BASE_ADDR + CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA),
                        NULL, &param1, regionId, sizeof(uint16_t), 1U, 1U, 0U, 0U, sizeof(uint16_t), 0U, EDMA_TRIG_MODE_MANUAL);

    /* Copy rest of the data with event based */
    Fsi_appDmaConfigure(gEdmaHandle[0], &dmaCh0, (void *)(gTxBufData + FSI_APP_FRAME_DATA_WORD_COUNT),
                       ((void *)CONFIG_FSI_TX0_BASE_ADDR + CSL_FSI_TX_CFG_TX_BUF_BASE(bufIdx)),
                         NULL, &param0, regionId, sizeof(uint16_t), FSI_APP_FRAME_DATA_WORD_COUNT, FSI_APP_LOOP_COUNT - 1U,
                         sizeof(uint16_t), sizeof(uint16_t), sizeof(uint16_t) * FSI_APP_FRAME_DATA_WORD_COUNT, 0U, EDMA_TRIG_MODE_EVENT);
    Fsi_appDmaConfigure(gEdmaHandle[0], &dmaCh1, (void *)&gTxBufTagAndUserData[1U],
                        (void *)(CONFIG_FSI_TX0_BASE_ADDR + CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA),
                        &tccAlloc0, &param1, regionId, sizeof(uint16_t), 1U, FSI_APP_LOOP_COUNT - 1U, 0U, 0U, sizeof(uint16_t), 0U, EDMA_TRIG_MODE_EVENT);


    dmaCh2 = FSI_RX_EDMA_CHANNEL0;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh2);
    DebugP_assert(status == SystemP_SUCCESS);

    param2 = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param2);
    DebugP_assert(status == SystemP_SUCCESS);

    Fsi_appDmaConfigure(gEdmaHandle[0], &dmaCh2, (void *)(CONFIG_FSI_RX0_BASE_ADDR + CSL_FSI_RX_CFG_RX_BUF_BASE(bufIdx)),
                       (void *)gRxBufData, NULL, &param2, regionId, sizeof(uint16_t), FSI_APP_FRAME_DATA_WORD_COUNT, FSI_APP_LOOP_COUNT,
                         sizeof(uint16_t), sizeof(uint16_t), 0U, sizeof(uint16_t) * FSI_APP_FRAME_DATA_WORD_COUNT, EDMA_TRIG_MODE_EVENT);

    dmaCh3 = FSI_RX_EDMA_CHANNEL1;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh3);
    DebugP_assert(status == SystemP_SUCCESS);

    param3 = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param3);
    DebugP_assert(status == SystemP_SUCCESS);

    tccAlloc1 = EDMA_RESOURCE_ALLOC_ANY;
    edmaStatus = EDMA_allocTcc(gEdmaHandle[0], &tccAlloc1);
    DebugP_assert(edmaStatus == SystemP_SUCCESS);

    /* In case of Frame Tag and User Data, bCnt is 1 as its a 16 bit field */   
    Fsi_appDmaConfigure(gEdmaHandle[0], &dmaCh3, (void *)(CONFIG_FSI_RX0_BASE_ADDR + CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA),
                       (void *)&gRxBufTagAndUserData, &tccAlloc1, &param3, regionId, sizeof(uint16_t), 1U, FSI_APP_LOOP_COUNT,
                        0U, 0U, 0U, sizeof(uint16_t), EDMA_TRIG_MODE_EVENT);

    /* Create a semaphore to signal EDMA transfer completion */
    status = SemaphoreP_constructBinary(&gFsiDmaTxSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gFsiDmaRxSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* EDMA Register Interrupt */
    status = Fsi_appDmaTxIntrInit(tccAlloc0);
    DebugP_assert(status == SystemP_SUCCESS);

    status = Fsi_appDmaRxIntrInit(tccAlloc1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* FSI configuration */
    status  = Fsi_appDmaTxConfig(txBaseAddr);
    status += Fsi_appDmaRxConfig(rxBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Enable loopback */
    status = FSI_enableRxInternalLoopback(rxBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Send Flush Sequence to sync, after every rx soft reset */
    status = FSI_executeTxFlushSequence(txBaseAddr, FSI_APP_TX_PRESCALER_VAL);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Transmit data */
    status = FSI_setTxStartMode(txBaseAddr, FSI_TX_START_FRAME_CTRL_OR_UDATA_TAG);
    FSI_enableRxDMAEvent(rxBaseAddr);
    FSI_enableTxDMAEvent(txBaseAddr);
    status += FSI_startTxTransmit(txBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Wait for TX and RX completion */
    SemaphoreP_pend(&gFsiDmaTxSemObject, SystemP_WAIT_FOREVER);
    SemaphoreP_pend(&gFsiDmaRxSemObject, SystemP_WAIT_FOREVER);

    /* Perform a cache invalidate of result buffers */
    CacheP_inv((void *)gTxBufData, ((FSI_APP_FRAME_DATA_WORD_COUNT * FSI_APP_LOOP_COUNT) * sizeof(uint16_t)), CacheP_TYPE_ALL);
    CacheP_inv((void *)&gTxBufTagAndUserData, (FSI_APP_LOOP_COUNT * sizeof(uint16_t)), CacheP_TYPE_ALL);
    CacheP_inv((void *)gRxBufData, ((FSI_APP_FRAME_DATA_WORD_COUNT * FSI_APP_LOOP_COUNT) * sizeof(uint16_t)), CacheP_TYPE_ALL);
    CacheP_inv((void *)&gRxBufTagAndUserData, (FSI_APP_LOOP_COUNT * sizeof(uint16_t)), CacheP_TYPE_ALL);

    status = Fsi_appCompareData(gTxBufData, gRxBufData);
    DebugP_assert(status == SystemP_SUCCESS);

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("[FSI] %d frames successfully received!!!\r\n", FSI_APP_LOOP_COUNT);
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    FSI_disableRxDMAEvent(rxBaseAddr);
    FSI_disableTxDMAEvent(txBaseAddr);
    Fsi_appDmaIntrDeInit();

    edmaBaseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(edmaBaseAddr != 0);

    /* Free channel */
    EDMA_freeChannelRegion(edmaBaseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh1, EDMA_TRIG_MODE_EVENT, tccAlloc0, EDMA_TEST_EVT_QUEUE_NO);
    status = EDMA_freeTcc(gEdmaHandle[0], &tccAlloc0);
    DebugP_assert(status == SystemP_SUCCESS);

    EDMA_freeChannelRegion(edmaBaseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh3, EDMA_TRIG_MODE_EVENT, tccAlloc1, EDMA_TEST_EVT_QUEUE_NO);
    status = EDMA_freeTcc(gEdmaHandle[0], &tccAlloc1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh0);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &param0);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh1);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &param1);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh2);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &param2);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh3);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &param3);
    DebugP_assert(status == SystemP_SUCCESS);

    Board_driversClose();
    Drivers_close();

    return NULL;
}

static int32_t Fsi_appDmaTxConfig(uint32_t txBaseAddr)
{
    int32_t     status;

    /* TX init and reset */
    status = FSI_performTxInitialization(txBaseAddr, FSI_APP_TX_PRESCALER_VAL);
    status += FSI_resetTxModule(txBaseAddr, FSI_TX_MASTER_CORE_RESET);
    FSI_clearTxModuleReset(txBaseAddr, FSI_TX_MASTER_CORE_RESET);

    /* Setting for requested transfer params */
    status += FSI_setTxSoftwareFrameSize(txBaseAddr, FSI_APP_FRAME_DATA_WORD_COUNT);
    status += FSI_setTxDataWidth(txBaseAddr, FSI_APP_N_LANES);
    status += FSI_setTxFrameType(txBaseAddr, FSI_FRAME_TYPE_NWORD_DATA);

    return status;
}

static int32_t Fsi_appDmaRxConfig(uint32_t rxBaseAddr)
{
    int32_t     status;

    /* RX init and reset */
    status  = FSI_performRxInitialization(rxBaseAddr);
    status += FSI_resetRxModule(rxBaseAddr, FSI_RX_MASTER_CORE_RESET);
    FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_MASTER_CORE_RESET);

    /* Setting for requested transfer params */
    status += FSI_setRxSoftwareFrameSize(rxBaseAddr, FSI_APP_FRAME_DATA_WORD_COUNT);
    status += FSI_setRxDataWidth(rxBaseAddr, FSI_APP_N_LANES);

    return status;
}

static int32_t Fsi_appDmaTxIntrInit(uint32_t tccAlloc)
{
    int32_t     status;

    /* Register TX DMA Completion interrupt */
    gTxIntrObj.tccNum = tccAlloc;
    gTxIntrObj.cbFxn  = &Fsi_appDmaTxCallback;
    gTxIntrObj.appData = (void *) &gFsiDmaTxSemObject;
    status = EDMA_registerIntr(gEdmaHandle[0], &gTxIntrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    return status;
}

static int32_t Fsi_appDmaRxIntrInit(uint32_t tccAlloc)
{
    int32_t     status;

    /* Register RX DMA Completion interrupt */
    gRxIntrObj.tccNum = tccAlloc;
    gRxIntrObj.cbFxn  = &Fsi_appDmaRxCallback;
    gRxIntrObj.appData = (void *) &gFsiDmaRxSemObject;
    status = EDMA_registerIntr(gEdmaHandle[0], &gRxIntrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    return status;
}

static void Fsi_appDmaIntrDeInit(void)
{
    int32_t status;

    SemaphoreP_destruct(&gFsiDmaTxSemObject);
    SemaphoreP_destruct(&gFsiDmaRxSemObject);
    status  = EDMA_unregisterIntr(gEdmaHandle[0], &gTxIntrObj);
    status += EDMA_unregisterIntr(gEdmaHandle[0], &gRxIntrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    return;
}

static void Fsi_appDmaTxCallback(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);

    return;
}

static void Fsi_appDmaRxCallback(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);

    return;
}

static int32_t Fsi_appCompareData(uint16_t *txBufPtr, uint16_t *rxBufPtr)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    i;

    for(i = 0; i < (FSI_APP_FRAME_DATA_WORD_COUNT * FSI_APP_LOOP_COUNT); i++)
    {
        if(*rxBufPtr++ != *txBufPtr++)
        {
            status = SystemP_FAILURE;
            break;
        }
    }

    for(i = 0; i < FSI_APP_LOOP_COUNT; i++)
    {
        /* Verify Tag And UserData */
        uint16_t tagData, userData;
        tagData = (gRxBufTagAndUserData[i] & CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_FRAME_TAG_MASK) >>
                     CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_FRAME_TAG_SHIFT;
        if(FSI_APP_TX_DATA_FRAME_TAG != tagData)
        {
            status |= SystemP_FAILURE;
            break;
        }
        userData = (gRxBufTagAndUserData[i] & CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_USER_DATA_MASK) >>
                     CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_USER_DATA_SHIFT;
        if(FSI_APP_TX_USER_DATA != userData)
        {
            status |= SystemP_FAILURE;
            break;
        }
    }

    return status;
}

static uint16_t Fsi_appDmaConfigure(EDMA_Handle dma_handle, uint32_t *dmaCh,
                void *src, void *dst, uint32_t *tcc, uint32_t *param, uint32_t regionId,
                uint32_t aCnt, uint32_t bCnt, uint32_t cCnt, uint32_t srcBIdx, uint32_t destBIdx,
                uint32_t srcCIdx, uint32_t destCIdx, uint32_t triggerMode)
{
    uint32_t            baseAddr;
    EDMACCPaRAMEntry    edmaParam;
    int32_t             status = SystemP_SUCCESS;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         *dmaCh, *tcc, *param, EDMA_TEST_EVT_QUEUE_NO);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)src);
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)dst);
    edmaParam.aCnt          = (uint16_t) aCnt;
    edmaParam.bCnt          = (uint16_t) bCnt;
    edmaParam.cCnt          = (uint16_t) cCnt;
    edmaParam.bCntReload    = bCnt;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(srcBIdx);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(destBIdx);
    edmaParam.srcCIdx       = (int16_t) srcCIdx;
    edmaParam.destCIdx      = (int16_t) destCIdx;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(srcBIdx);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(destBIdx);
    /* Transfer Completion Interrupt is enabled only after Frame Tag And User
     * data is transmitted/received */
    if (tcc == NULL)
    {
        edmaParam.opt       = EDMA_OPT_SYNCDIM_MASK;
    }
    else
    {
        edmaParam.opt       = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
                              ((((uint32_t)*tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    }

    EDMA_setPaRAM(baseAddr, *param, &edmaParam);
    EDMA_enableTransferRegion(baseAddr, regionId, *dmaCh,
                              triggerMode);

    return status;
}
