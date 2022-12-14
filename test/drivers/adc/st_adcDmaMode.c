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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "st_adc.h"
#include <drivers/udma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * Application test parameters
 */
#define APP_ADC_RX_PDMA_CH_FIFO0        (UDMA_PDMA_CH_MAIN1_ADC0_CH0_RX)
#define APP_ADC_RX_PDMA_CH_FIFO1        (UDMA_PDMA_CH_MAIN1_ADC0_CH1_RX)
#define APP_ADC_NUM_MAX_CH              (16U)

/*
 * Ring parameters
 */
/* Number of ring entries - we can prime this much ADC operations */
#define UDMA_TEST_APP_RING_ENTRIES      (1U)
/* Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define UDMA_TEST_APP_RING_ENTRY_SIZE   (sizeof(uint64_t))
/* Total ring memory */
#define UDMA_TEST_APP_RING_MEM_SIZE     (UDMA_TEST_APP_RING_ENTRIES * UDMA_TEST_APP_RING_ENTRY_SIZE)
/* UDMA host mode buffer descriptor memory size. */
#define UDMA_TEST_APP_DESC_SIZE         (sizeof(CSL_UdmapCppi5HMPD) + (128U - sizeof(CSL_UdmapCppi5HMPD)))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t App_adcTest(st_ADCTestcaseParams_t *testParams,
                           Udma_ChHandle rxChHandle);
static void App_udmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData);

static int32_t App_create(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle,
                          st_ADCTestcaseParams_t *testParams);
static int32_t App_delete(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle);

static void App_udmaRxHpdInit(Udma_ChHandle rxChHandle,
                              uint8_t *pHpdMem,
                              const uint32_t *destBuf,
                              uint32_t length);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static uint32_t     gAdcModule;
Udma_ChObject       gUdmaRxChObj;
Udma_EventObject    gUdmaCqEventObj;
Udma_EventObject    gUdmaTdCqEventObj;

/*
 * UDMA Memories
 */
static uint8_t gRxFqRingMem[UDMA_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaRxHpdMem[UDMA_TEST_APP_DESC_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/*
 * Application Buffers
 */
uint32_t gAdcDestBuf[APP_ADC_NUM_MAX_CH] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gUdmaAppDoneSem;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t st_adcDmaMode_main(st_ADCTestcaseParams_t *testParams)
{
    int32_t         status;
    Udma_ChHandle   rxChHandle = &gUdmaRxChObj;
    Udma_DrvHandle  drvHandle = &gUdmaDrvObj[CONFIG_UDMA0];

    gAdcModule = testParams->adcConfigParams.adcModule;
    st_adcModuleInit(gAdcModule);
    App_create(drvHandle, rxChHandle, testParams);

    status = App_adcTest(testParams, rxChHandle);
    DebugP_assert(UDMA_SOK == status);

    App_delete(drvHandle, rxChHandle);
    ADCPowerUp(gAdcModule, FALSE);

    testParams->testResult = status;

    return 0;
}

static int32_t App_adcTest(st_ADCTestcaseParams_t *testParams, Udma_ChHandle rxChHandle)
{
    int32_t         retVal = UDMA_SOK;
    uint64_t        pDesc = 0;
    uint32_t       *destBuf = &gAdcDestBuf[0U], i;
    uint8_t        *pHpdMem = &gUdmaRxHpdMem[0U];

    retVal = st_adcStepConfig(testParams);
    DebugP_assert(SystemP_SUCCESS == retVal);

    /* Init buffers */
    for(i = 0U; i < APP_ADC_NUM_MAX_CH; i++)
    {
        destBuf[i] = 0U;
    }

    /* Writeback buffer */
    CacheP_wb(&gAdcDestBuf[0U], sizeof(gAdcDestBuf), CacheP_TYPE_ALLD);

    /* Enable ADC DMA */
    ADCSetDMAFIFOThresholdLevel(gAdcModule,
                                testParams->adcConfigParams.fifoNum,
                                testParams->adcConfigParams.numSteps);

    ADCFIFODMAAccessEnable(gAdcModule, testParams->adcConfigParams.fifoNum, TRUE);

    /* Update host packet descriptor */
    App_udmaRxHpdInit(rxChHandle, pHpdMem, destBuf, testParams->adcConfigParams.numSteps * 4U);

    /* Submit HPD to channel */
    retVal = Udma_ringQueueRaw(
                    Udma_chGetFqRingHandle(rxChHandle),
                    (uint64_t) Udma_defaultVirtToPhyFxn(pHpdMem, 0U, NULL));
    DebugP_assert(UDMA_SOK == retVal);

    /* Start ADC conversion */
    st_adcModuleStart(gAdcModule);

    /* Wait for return descriptor in completion ring - this marks the
        * transfer completion */
    SemaphoreP_pend(&gUdmaAppDoneSem, SystemP_WAIT_FOREVER);

    /* Response received in completion queue */
    retVal = Udma_ringDequeueRaw(
                    Udma_chGetCqRingHandle(rxChHandle), &pDesc);
    DebugP_assert(UDMA_SOK == retVal);

    /* Invalidate cache */
    CacheP_inv(pHpdMem, UDMA_TEST_APP_DESC_SIZE, CacheP_TYPE_ALLD);
    CacheP_inv(&gAdcDestBuf[0U], sizeof(gAdcDestBuf), CacheP_TYPE_ALLD);
    retVal =  st_adcValidateFifoData(testParams, &destBuf[0], testParams->adcConfigParams.numSteps);

    /* Disable DMA */
    ADCFIFODMAAccessEnable(gAdcModule, testParams->adcConfigParams.fifoNum, FALSE);

    /* Stop ADC */
    st_adcModuleStop(testParams);

    return (retVal);
}

static void App_udmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData)
{
    if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventType)
    {
        SemaphoreP_post(&gUdmaAppDoneSem);
    }
    return;
}

static int32_t App_create(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle, st_ADCTestcaseParams_t *testParams)
{
    int32_t             retVal;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    Udma_ChPdmaPrms     pdmaPrms;

    retVal = SemaphoreP_constructBinary(&gUdmaAppDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == retVal);

    /* Init channel parameters */
    chType = UDMA_CH_TYPE_PDMA_RX;
    UdmaChPrms_init(&chPrms, chType);
    if(ADC_FIFO_NUM_0 == testParams->adcConfigParams.fifoNum)
    {
        chPrms.peerChNum            = APP_ADC_RX_PDMA_CH_FIFO0;
    }
    else
    {
        chPrms.peerChNum            = APP_ADC_RX_PDMA_CH_FIFO1;
    }
    chPrms.fqRingPrms.ringMem   = &gRxFqRingMem[0U];
    chPrms.fqRingPrms.ringMemSize   = UDMA_TEST_APP_RING_MEM_SIZE;
    chPrms.fqRingPrms.elemCnt   = UDMA_TEST_APP_RING_ENTRIES;

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
    eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(drvHandle);
    eventPrms.eventCb           = &App_udmaEventDmaCb;
    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

    /* Config PDMA channel */
    UdmaChPdmaPrms_init(&pdmaPrms);
    pdmaPrms.elemSize   = UDMA_PDMA_ES_32BITS;
    pdmaPrms.elemCnt    = testParams->adcConfigParams.numSteps;
    pdmaPrms.fifoCnt    = 1U;
    retVal = Udma_chConfigPdma(rxChHandle, &pdmaPrms);
    DebugP_assert(UDMA_SOK == retVal);

    retVal = Udma_chEnable(rxChHandle);
    DebugP_assert(UDMA_SOK == retVal);

    return (retVal);
}

static int32_t App_delete(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle)
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

    SemaphoreP_destruct(&gUdmaAppDoneSem);

    return (retVal);
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
    CSL_udmapCppi5SetReturnPolicy(pHpd, descType,
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
    CacheP_wb(pHpdMem, UDMA_TEST_APP_DESC_SIZE, CacheP_TYPE_ALLD);

    return;
}
