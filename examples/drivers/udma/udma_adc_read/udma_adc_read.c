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
 *  This example performs PDMA RX data capture from ADC.
 *
 * ADC is configured in single shot mode and captures APP_ADC_NUM_CH channel
 * of ADC data. The FIFO is configured to generate a DMA trigger after all
 * channel data is captured.
 *
 * The application opens and configures a Packet DMA (PKTDMA) channel.
 * It configures the PDMA parameter for transfer from ADC. The PDMA element count
 * is set to the number of ADC samples - APP_ADC_NUM_CH.
 *
 * This uses Host Packet Descriptor (HPD) to receive data from ADC PDMA channel
 * into the destination buffer.
 *
 * The ADC is configured to tag the channel/step ID as part of ADC data using \ref ADCStepIdTagEnable API.
 * The application uses this to compare that the DMA read data is in proper
 * sequence and prints pass/fail accordingly.
 */

#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * Application test parameters
 */
#define APP_ADC_FIFO                    (ADC_FIFO_NUM_0)
#define APP_ADC_MODULE                  (CSL_ADC0_BASE)
#define APP_ADC_RX_PDMA_CH              (UDMA_PDMA_CH_MAIN1_ADC0_CH0_RX)
#define APP_ADC_NUM_CH                  (8U)

/* Reference voltage for ADC - should be given in mV */
#define APP_ADC_REF_VOLTAGE             (1800U)

/*
 * Number of bytes transmitted by PDMA per RX event sent by ADC.
 * In ADC, this should be equal to the DMA trigger.
 * Testing single shot mode - so should be same as number of channels being
 * captured
 */
#define RX_BYTES_PER_EVENT              (APP_ADC_NUM_CH)

/** \brief Number of times to perform the ADC operation */
#define UDMA_TEST_LOOP_CNT              (10U)

/*
 * Ring parameters
 */
/** \brief Number of ring entries - we can prime this much ADC operations */
#define UDMA_TEST_RING_ENTRIES          (1U)
/** \brief Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define UDMA_TEST_RING_ENTRY_SIZE       (sizeof(uint64_t))
/** \brief Total ring memory */
#define UDMA_TEST_RING_MEM_SIZE         (UDMA_TEST_RING_ENTRIES * UDMA_TEST_RING_ENTRY_SIZE)
/** \brief UDMA host mode buffer descriptor memory size. */
#define UDMA_TEST_DESC_SIZE             (sizeof(CSL_UdmapCppi5HMPD))

static void App_adcTest(Udma_ChHandle rxChHandle);
static void App_udmaAdcRx(Udma_ChHandle rxChHandle, uint32_t *destBuf);

static void App_udmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData);
static void App_create(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle);
static void App_delete(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle);

static void App_udmaRxHpdInit(Udma_ChHandle rxChHandle,
                              uint8_t *pHpdMem,
                              const uint32_t *destBuf,
                              uint32_t length);

static void App_adcInit(void);
static void App_adcConfig(void);
static void App_adcStart(void);
static void App_adcStop(void);
static void App_adcDeInit(void);

/*
 * UDMA driver objects
 */
Udma_ChObject       gUdmaRxChObj;
Udma_EventObject    gUdmaCqEventObj;

/*
 * UDMA Memories
 */
static uint8_t gRxFqRingMem[UDMA_ALIGN_SIZE(UDMA_TEST_RING_MEM_SIZE)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaRxHpdMem[UDMA_ALIGN_SIZE(UDMA_TEST_DESC_SIZE)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/*
 * Application Buffers
 */
uint32_t gAdcDestBuf[APP_ADC_NUM_CH] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gUdmaTestDoneSem;

void *udma_adc_read_main(void *args)
{
    Udma_DrvHandle  drvHandle = &gUdmaDrvObj[CONFIG_UDMA0];
    Udma_ChHandle   rxChHandle = &gUdmaRxChObj;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    App_adcInit();
    DebugP_log("[UDMA] ADC read application started ...\r\n");

    App_create(drvHandle, rxChHandle);

    App_adcTest(rxChHandle);

    App_delete(drvHandle, rxChHandle);

    DebugP_log("All tests have passed!!\r\n");
    Board_driversClose();
    App_adcDeInit();
    Drivers_close();

    return NULL;
}

static void App_adcTest(Udma_ChHandle rxChHandle)
{
    uint32_t    loopCnt = 0U;
    uint32_t   *destBuf = &gAdcDestBuf[0U];
    uint32_t    i;

    /* Init buffers */
    for(i = 0U; i < APP_ADC_NUM_CH; i++)
    {
        destBuf[i] = 0U;
    }
    /* Writeback buffer */
    CacheP_wb(&gAdcDestBuf[0U], sizeof(gAdcDestBuf), CacheP_TYPE_ALLD);

    while(loopCnt < UDMA_TEST_LOOP_CNT)
    {
        /* Perform UDMA ADC RX */
        App_udmaAdcRx(rxChHandle, destBuf);
        loopCnt++;
        DebugP_log("Loop Count: %d completed!!\n\r\n", loopCnt);
    }

    return;
}

static void App_udmaAdcRx(Udma_ChHandle rxChHandle, uint32_t *destBuf)
{
    int32_t         retVal;
    uint32_t        loopCnt, fifoData, adcData, stepId, chCnt, voltageLvl;
    uint64_t        pDesc = 0;
    uint8_t        *pHpdMem = &gUdmaRxHpdMem[0U];

    App_adcConfig();

    /* Update host packet descriptor */
    App_udmaRxHpdInit(rxChHandle, pHpdMem, destBuf, APP_ADC_NUM_CH * 4U);

    /* Submit HPD to channel */
    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(rxChHandle),
                 (uint64_t) Udma_defaultVirtToPhyFxn(pHpdMem, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);

    App_adcStart();

    /* Wait for return descriptor in completion ring - this marks the
     * transfer completion */
    SemaphoreP_pend(&gUdmaTestDoneSem, SystemP_WAIT_FOREVER);

    /* Response received in completion queue */
    retVal = Udma_ringDequeueRaw(
                 Udma_chGetCqRingHandle(rxChHandle), &pDesc);
    DebugP_assert(UDMA_SOK == retVal);

    /* Invalidate cache */
    CacheP_inv(pHpdMem, UDMA_TEST_DESC_SIZE, CacheP_TYPE_ALLD);
    CacheP_inv(&gAdcDestBuf[0U], sizeof(gAdcDestBuf), CacheP_TYPE_ALLD);
    for (loopCnt = 0U; loopCnt < APP_ADC_NUM_CH; loopCnt++)
    {
        chCnt = loopCnt % APP_ADC_NUM_CH;
        fifoData = destBuf[loopCnt];
        stepId   = ((fifoData & ADC_FIFODATA_ADCCHNLID_MASK) >>
                    ADC_FIFODATA_ADCCHNLID_SHIFT);
        adcData = ((fifoData & ADC_FIFODATA_ADCDATA_MASK) >>
                    ADC_FIFODATA_ADCDATA_SHIFT);
        if(stepId != chCnt)     /* Both channel and step are 1:1 mapped */
        {
            DebugP_logError("Step ID Error: %d\r\n", stepId);
            DebugP_assert(FALSE);
        }
        voltageLvl  = adcData * (uint32_t) APP_ADC_REF_VOLTAGE;
        voltageLvl /= (uint32_t) ADC_GET_RANGE(CONFIG_ADC0_NUM_BITS);
        DebugP_log("CH %d ", chCnt);
        DebugP_log("ADC Voltage: %d mV\r\n", voltageLvl);
    }

    App_adcStop();

    return;
}

static void App_udmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData)
{
    if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventType)
    {
        SemaphoreP_post(&gUdmaTestDoneSem);
    }

    return;
}

static void App_create(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle)
{
    int32_t             retVal;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_ChPdmaPrms     pdmaPrms;

    retVal = SemaphoreP_constructBinary(&gUdmaTestDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == retVal);

    /* Init channel parameters */
    chType = UDMA_CH_TYPE_PDMA_RX;
    UdmaChPrms_init(&chPrms, chType);
    chPrms.peerChNum            = APP_ADC_RX_PDMA_CH;
    chPrms.fqRingPrms.ringMem   = &gRxFqRingMem[0U];
    chPrms.fqRingPrms.ringMemSize   = UDMA_TEST_RING_MEM_SIZE;
    chPrms.fqRingPrms.elemCnt   = UDMA_TEST_RING_ENTRIES;

    /* Open channel for block copy */
    retVal = Udma_chOpen(drvHandle, rxChHandle, chType, &chPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config RX channel */
    UdmaChRxPrms_init(&rxPrms, UDMA_CH_TYPE_PDMA_RX);
    retVal = Udma_chConfigRx(rxChHandle, &rxPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Register ring completion callback */
    eventHandle = &gUdmaCqEventObj;
    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = rxChHandle;
    eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
    eventPrms.eventCb           = &App_udmaEventDmaCb;
    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config PDMA channel */
    UdmaChPdmaPrms_init(&pdmaPrms);
    pdmaPrms.elemSize   = UDMA_PDMA_ES_32BITS;
    pdmaPrms.elemCnt    = RX_BYTES_PER_EVENT;
    pdmaPrms.fifoCnt    = (APP_ADC_NUM_CH / RX_BYTES_PER_EVENT);
    retVal = Udma_chConfigPdma(rxChHandle, &pdmaPrms);
    DebugP_assert(UDMA_SOK == retVal);

    retVal = Udma_chEnable(rxChHandle);
    DebugP_assert(UDMA_SOK == retVal);

    return;
}

static void App_delete(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle)
{
    int32_t             retVal;
    Udma_EventHandle    eventHandle;

    retVal = Udma_chDisable(rxChHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == retVal);

    /* Unregister all events */
    eventHandle = &gUdmaCqEventObj;
    retVal = Udma_eventUnRegister(eventHandle);
    DebugP_assert(UDMA_SOK == retVal);

    retVal = Udma_chClose(rxChHandle);
    DebugP_assert(UDMA_SOK == retVal);

    SemaphoreP_destruct(&gUdmaTestDoneSem);

    return;
}

static void App_udmaRxHpdInit(Udma_ChHandle rxChHandle,
                              uint8_t *pHpdMem,
                              const uint32_t *destBuf,
                              uint32_t length)
{
    CSL_UdmapCppi5HMPD *pHpd = (CSL_UdmapCppi5HMPD *) pHpdMem;
    uint32_t descType = (uint32_t)CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST;

    /* Setup descriptor */
    CSL_udmapCppi5SetDescType(pHpd, descType);
    CSL_udmapCppi5SetEpiDataPresent(pHpd, FALSE);
    CSL_udmapCppi5SetPsDataLoc(pHpd, 0U);
    CSL_udmapCppi5SetPsDataLen(pHpd, 0U);
    CSL_udmapCppi5SetPktLen(pHpd, descType, length);
    CSL_udmapCppi5SetPsFlags(pHpd, 0U);
    CSL_udmapCppi5SetIds(pHpd, descType, 0x321, UDMA_DEFAULT_FLOW_ID);
    CSL_udmapCppi5SetSrcTag(pHpd, 0x0000);     /* Not used */
    CSL_udmapCppi5SetDstTag(pHpd, 0x0000);     /* Not used */
    /* Return Policy descriptors are reserved in case of AM243X/Am64X */
    CSL_udmapCppi5SetReturnPolicy(
        pHpd,
        descType,
        0U,
        0U,
        0U,
        0U);
    CSL_udmapCppi5LinkDesc(pHpd, 0U);
    CSL_udmapCppi5SetBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetBufferLen(pHpd, length);
    CSL_udmapCppi5SetOrgBufferAddr(pHpd, (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL));
    CSL_udmapCppi5SetOrgBufferLen(pHpd, length);

    /* Writeback cache */
    CacheP_wb(pHpdMem, UDMA_TEST_DESC_SIZE, CacheP_TYPE_ALLD);

    return;
}

static void App_adcInit(void)
{
    /* Clear All interrupt status */
    ADCClearIntrStatus(APP_ADC_MODULE, ADC_INTR_STATUS_ALL);

    /* Power up AFE */
    ADCPowerUp(APP_ADC_MODULE, TRUE);
    /* Wait for 4us at least */
    ClockP_usleep(10);

    /* Do the internal calibration */
    ADCInit(APP_ADC_MODULE, FALSE, 0U, 0U);

    return;
}

static void App_adcConfig(void)
{
    uint32_t        chCnt;
    adcStepConfig_t adcConfig;

    /* Initialize ADC configuration params */
    adcConfig.mode             = ADC_OPERATION_MODE_SINGLE_SHOT;
    adcConfig.openDelay        = 0x1U;
    adcConfig.sampleDelay      = 0U;
    adcConfig.rangeCheckEnable = 0U;
    adcConfig.averaging        = ADC_AVERAGING_16_SAMPLES;
    adcConfig.fifoNum          = APP_ADC_FIFO;
    for(chCnt = 0U; chCnt < APP_ADC_NUM_CH; chCnt++)
    {
        /* Step configuration */
        adcConfig.channel = ADC_CHANNEL_1 + chCnt;
        ADCSetStepParams(APP_ADC_MODULE, ADC_STEP_1 + chCnt, &adcConfig);
        /* step enable */
        ADCStepEnable(APP_ADC_MODULE, ADC_STEP_1 + chCnt, TRUE);
    }

    ADCStepIdTagEnable(APP_ADC_MODULE, TRUE);
    ADCSetDMAFIFOThresholdLevel(APP_ADC_MODULE, APP_ADC_FIFO, APP_ADC_NUM_CH);

    return;
}

static void App_adcStart(void)
{
    adcSequencerStatus_t status;

    /* Enable DMA */
    ADCFIFODMAAccessEnable(APP_ADC_MODULE, APP_ADC_FIFO, TRUE);

    /* Check if FSM is idle */
    ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    while((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    }

    /* Start ADC conversion */
    ADCStart(APP_ADC_MODULE, TRUE);

    return;
}

static void App_adcStop(void)
{
    uint32_t                chCnt;
    adcSequencerStatus_t    status;

    /* Disable DMA */
    ADCFIFODMAAccessEnable(APP_ADC_MODULE, APP_ADC_FIFO, FALSE);

    /* Disable all/enabled steps */
    for(chCnt = 0U; chCnt < APP_ADC_NUM_CH; chCnt++)
    {
        ADCStepEnable(APP_ADC_MODULE, ADC_STEP_1 + chCnt, FALSE);
    }

    /* Wait for FSM to go IDLE */
    ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    while((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    }

    /* Stop ADC */
    ADCStart(APP_ADC_MODULE, FALSE);

    /* Wait for FSM to go IDLE */
    ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    while((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    }

    return;
}

static void App_adcDeInit(void)
{
    /* Power down ADC */
    ADCPowerUp(APP_ADC_MODULE, FALSE);

    return;
}
