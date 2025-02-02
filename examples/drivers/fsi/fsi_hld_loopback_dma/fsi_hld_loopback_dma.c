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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/SemaphoreP.h>

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

/* Index of FSI TX/RX buffer, gBudIdx + FSI_APP_FRAME_DATA_WORD_COUNT should be <= 16 */
uint16_t gRxBufData[FSI_APP_FRAME_DATA_WORD_COUNT * FSI_APP_LOOP_COUNT] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
/* Tag and User data is per frame */
uint16_t gRxBufTagAndUserData[FSI_APP_LOOP_COUNT] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint16_t gTxBufData[FSI_APP_FRAME_DATA_WORD_COUNT * FSI_APP_LOOP_COUNT] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint16_t gTxBufTagAndUserData[FSI_APP_LOOP_COUNT] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

static int32_t  Fsi_appCompareData(uint16_t *txBufPtr, uint16_t *rxBufPtr);

void *fsi_hld_loopback_dma_main(void *args)
{
    int32_t     status;
    uint32_t    rxBaseAddr;
    uint16_t    numWords, bufIdx;
    uint32_t    appLoopCnt;
    uint32_t    txBaseAddr;

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

    /* Enable loopback */
    status = FSI_enableRxInternalLoopback(rxBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Transmit data */
    status = FSI_Tx_hld(gFsiTxHandle[CONFIG_FSI_TX0], gTxBufData, gTxBufTagAndUserData, 0, bufIdx);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Receive data */
    status = FSI_Rx_hld(gFsiRxHandle[CONFIG_FSI_RX0], gRxBufData, gRxBufTagAndUserData, 0, bufIdx);
    DebugP_assert(status == SystemP_SUCCESS);

    status += FSI_startTxTransmit(txBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    FSI_Tx_pendDmaCompletion();
    FSI_Rx_pendDmaCompletion();

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

    Board_driversClose();
    Drivers_close();

    return NULL;
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
