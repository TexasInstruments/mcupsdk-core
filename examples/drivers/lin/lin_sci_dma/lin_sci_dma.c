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

/*
 *  This example configures the LIN module in SCI mode for internal loopback
 *  with the use of the DMA. The LIN module performs as SCI with a set
 *  character and frame length in multi-buffer mode. When the transmit buffers
 *  in the LINTD0 and LINTD1 registers have enough space, the DMA will transfer
 *  data from global variable sData into those transmit registers. Once the
 *  received buffers in the LINRD0 and LINRD1 registers contain data,the DMA
 *  will transfer the data into the global variable rdata.
 *
 *  When all data has been placed into rData, a check of the validity of the
 *  data will be performed in one of the DMA channels' ISRs.
 *
 *  External Connections :
 *   - None.
 *
 *  Watch Variables :
 *   - sData - Data to send
 *   - rData - Received data
 *
 */


/* Included Files */
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/lin.h>
#include <ti_drivers_config.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Defines */
#define CHAR_LENGTH         (0x8)
#define FRAME_LENGTH        (0x8)

#define APP_LIN_BASE_ADDR   (CONFIG_LIN1_BASE_ADDR)

const uint32_t PRESCALER    = 0x000001A;
const uint16_t DIVIDER      = 0x0002;

/* Globals */
uint16_t sData[128] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint16_t rData[128] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

uint32_t            regionId;
uint32_t            dmaCh_rx, tcc_rx, param_rx;
uint32_t            dmaCh_tx, tcc_tx, param_tx;
Edma_IntrObject     intrObj_rx, intrObj_tx;

/* Flag to set when all data transfered */
volatile uint16_t error = 0;
volatile uint16_t tx_int, rx_int = 0;

static void initDMA(void);
static void deinitDMA(void);
static void configureSCIMode(void);

static void dmaRx(Edma_IntrHandle intrHandle, void *args);
static void dmaTx(Edma_IntrHandle intrHandle, void *args);

/* lin_sci_loopback_main */
void lin_sci_dma_main(void)
{
    uint8_t i = 0;
    uint32_t            baseAddr;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[LIN] SCI DMA mode, application started ...\r\n");

    /* Configure the LIN module to operate in SCI mode */
    configureSCIMode();

    initDMA();

    /* Get BaseAddr of EDMA */
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    /* Initialize the data buffers */
    for(i = 0; i < 128; i++)
    {
       sData[i]= i;
       rData[i]= 0;
    }

    /* Writeback buffer */
    CacheP_wb(&sData[0U], sizeof(sData), CacheP_TYPE_ALL);
    CacheP_wb(&rData[0U], sizeof(rData), CacheP_TYPE_ALL);

    /* Wait for the SCI receiver to be idle */
    while(!LIN_isSCIReceiverIdle(APP_LIN_BASE_ADDR));

    /* Wait until space is available in the transmit buffer. */
    while(!LIN_isSCISpaceAvailable(APP_LIN_BASE_ADDR));

    /* Register interrupt */
    intrObj_rx.tccNum = tcc_rx;
    intrObj_rx.cbFxn  = &dmaRx;
    intrObj_rx.appData = NULL;
    EDMA_registerIntr(gEdmaHandle[0], &intrObj_rx);

    /* Register interrupt */
    intrObj_tx.tccNum = tcc_tx;
    intrObj_tx.cbFxn  = &dmaTx;
    intrObj_tx.appData = NULL;
    EDMA_registerIntr(gEdmaHandle[0], &intrObj_tx);

    /* Start the DMA receive channel */
    EDMA_enableTransferRegion(
        baseAddr, regionId, dmaCh_rx, EDMA_TRIG_MODE_EVENT);

    /* Start the DMA transmit channel */
    EDMA_enableTransferRegion(
        baseAddr, regionId, dmaCh_tx, EDMA_TRIG_MODE_EVENT);

    /* Enable the DMA to receive */
    LIN_enableSCIInterrupt(APP_LIN_BASE_ADDR, LIN_SCI_INT_RX_DMA);

    /* Enable the DMA for transmission */
    LIN_enableSCIInterrupt(APP_LIN_BASE_ADDR, LIN_SCI_INT_TX_DMA);

    /* Wait until all the DMA event trigger */
    while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc_tx) != 1);
    while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc_rx) != 1);
    while(tx_int != rx_int);

    /* Invalidate cache */
    CacheP_inv(&rData[0U], sizeof(rData), CacheP_TYPE_ALL);

    /* Check if any data errors occurred */
    for(i=0U; i<128U; i++)
    {
        if(rData[i] != i)
        {
           DebugP_log("Test FAILED!!\r\n");
           error = 1;
           break;
        }
    }

    if(error != 1)
    {
        DebugP_log("All tests have passed!!\r\n");
    }

    deinitDMA();

    Board_driversClose();
    Drivers_close();
}

/*
 * Configure SCI Mode - This function configures the LIN module to operate as
 * an SCI with the specified settings.
*/
void
configureSCIMode(void)
{
    /* Enter LIN reset state to perform configurations */
    LIN_enterSoftwareReset(APP_LIN_BASE_ADDR);

    /* Switch LIN into SCI mode */
    LIN_enableSCIMode(APP_LIN_BASE_ADDR);

    /* Set the SCI communication mode to idle line */
    LIN_setSCICommMode(APP_LIN_BASE_ADDR, LIN_COMM_SCI_IDLELINE);

    /* Set SCI to transmit one stop bit */
    LIN_setSCIStopBits(APP_LIN_BASE_ADDR, LIN_SCI_STOP_ONE);

    /* Disable parity check */
    LIN_disableSCIParity(APP_LIN_BASE_ADDR);

    /* Enable multi-buffer mode */
    LIN_enableMultibufferMode(APP_LIN_BASE_ADDR);

    /* Module set to complete operations when halted by debugger */
    LIN_setDebugSuspendMode(APP_LIN_BASE_ADDR, LIN_DEBUG_COMPLETE);

    /* Set character length as 8-bits */
    LIN_setSCICharLength(APP_LIN_BASE_ADDR, CHAR_LENGTH);

    /* Set to 1 character in response field */
    LIN_setSCIFrameLength(APP_LIN_BASE_ADDR, FRAME_LENGTH);

    /* Set the Baud Rate */
    LIN_setBaudRatePrescaler(APP_LIN_BASE_ADDR, PRESCALER, DIVIDER);

    /* Enable Internal Loopback mode */
    LIN_enableIntLoopback(APP_LIN_BASE_ADDR);

    /* Exit LIN reset state */
    LIN_exitSoftwareReset(APP_LIN_BASE_ADDR);
}

static void initDMA(void)
{
    uint32_t            baseAddr;
    uint8_t            *srcBuffPtr, *dstBuffPtr;
    EDMACCPaRAMEntry    edmaParam_Rx, edmaParam_Tx;
    int32_t             status = SystemP_SUCCESS;

    DebugP_log("[LIN] Initialization of DMA started ...\r\n");

    /**************************** LIN RX ***********************************/
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* Allocate the resources dma channel, tcc and param */
    dmaCh_rx = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh_rx);
    DebugP_assert(status == SystemP_SUCCESS);

    tcc_rx = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &tcc_rx);
    DebugP_assert(status == SystemP_SUCCESS);

    param_rx = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param_rx);
    DebugP_assert(status == SystemP_SUCCESS);

    srcBuffPtr = (uint8_t *) (APP_LIN_BASE_ADDR + CSL_LIN_LINRD0);
    dstBuffPtr = (uint8_t *) &rData[0U];

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh_rx, tcc_rx, param_rx, 0U);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam_Rx);
    edmaParam_Rx.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam_Rx.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam_Rx.aCnt          = (uint16_t) 8U;
    edmaParam_Rx.bCnt          = (uint16_t) 32U;
    edmaParam_Rx.cCnt          = (uint16_t) 1U;
    edmaParam_Rx.bCntReload    = (uint16_t) 32U;
    edmaParam_Rx.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0U);
    edmaParam_Rx.destBIdx      = (int16_t) EDMA_PARAM_BIDX(8U);
    edmaParam_Rx.srcCIdx       = (int16_t) 0U;
    edmaParam_Rx.destCIdx      = (int16_t) 0U;
    edmaParam_Rx.linkAddr      = 0xFFFFU;
    edmaParam_Rx.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0U);
    edmaParam_Rx.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(8U);
    edmaParam_Rx.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc_rx) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param_rx, &edmaParam_Rx);

    /**************************** LIN TX ***********************************/
    /* Allocate the resources dma channel, tcc and param */
    dmaCh_tx = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh_tx);
    DebugP_assert(status == SystemP_SUCCESS);

    tcc_tx = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &tcc_tx);
    DebugP_assert(status == SystemP_SUCCESS);

    param_tx = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param_tx);
    DebugP_assert(status == SystemP_SUCCESS);

    srcBuffPtr = (uint8_t *) &sData[0U];
    dstBuffPtr = (uint8_t *) (APP_LIN_BASE_ADDR + CSL_LIN_LINTD0);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh_tx, tcc_tx, param_tx, 1U);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam_Tx);
    edmaParam_Tx.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam_Tx.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam_Tx.aCnt          = (uint16_t) 8U;
    edmaParam_Tx.bCnt          = (uint16_t) 32U;
    edmaParam_Tx.cCnt          = (uint16_t) 1U;
    edmaParam_Tx.bCntReload    = (uint16_t) 32U;
    edmaParam_Tx.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(8U);
    edmaParam_Tx.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0U);
    edmaParam_Tx.srcCIdx       = (int16_t) 0U;
    edmaParam_Tx.destCIdx      = (int16_t) 0U;
    edmaParam_Tx.linkAddr      = 0xFFFFU;
    edmaParam_Tx.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(8U);
    edmaParam_Tx.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0U);
    edmaParam_Tx.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc_tx) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param_tx, &edmaParam_Tx);
}

static void deinitDMA(void)
{
    uint32_t            baseAddr;
    int32_t             status = SystemP_SUCCESS;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    /* Clearing the Interrupts in EDMA */
    EDMA_clrIntrRegion(baseAddr, regionId, tcc_tx);
    EDMA_clrIntrRegion(baseAddr, regionId, tcc_rx);

    /* Clearing the Interrupts */
    EDMA_unregisterIntr(gEdmaHandle[0], &intrObj_tx);
    EDMA_unregisterIntr(gEdmaHandle[0], &intrObj_rx);

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh_rx, EDMA_TRIG_MODE_MANUAL, tcc_rx, 0U);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh_rx);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeTcc(gEdmaHandle[0], &tcc_rx);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &param_rx);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh_tx, EDMA_TRIG_MODE_MANUAL, tcc_tx, 1U);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh_tx);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeTcc(gEdmaHandle[0], &tcc_tx);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &param_tx);
    DebugP_assert(status == SystemP_SUCCESS);
}

static void dmaRx(Edma_IntrHandle intrHandle, void *args)
{
    rx_int++;
}

static void dmaTx(Edma_IntrHandle intrHandle, void *args)
{
    tx_int++;
}
