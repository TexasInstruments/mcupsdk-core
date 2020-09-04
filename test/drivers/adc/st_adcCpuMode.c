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

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define APP_ADC_MAX_SAMPLES             (300U)
#define APP_ADC_DIV                     (8U)
#define ADC_VOLTAGE_ERR_OFFSET_MV       (100U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static uint32_t             gAdcModule;
static uint64_t             gTimeStampBeforeADC, gTimeStampAfterADC;
static uint32_t             gOutOfRange;
static HwiP_Object          gHwiHandleDone;
static SemaphoreP_Object    gAdcSyncSemObject;

/* Application Buffers */
uint32_t gAdcCpuDestBuf[APP_ADC_MAX_SAMPLES];

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static void AppADCIntrISR(void *handle);
static uint32_t AppADCExtractFifoData(uint32_t fifoNum, uint32_t *dstBuff);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t st_adcCpuMode_main(st_ADCTestcaseParams_t *testParams)
{
    int32_t         status;
    uint32_t        loopCnt, numElems, numIter;
    HwiP_Params     hwiPrms;

    /* Construct Semaphore */
    status = SemaphoreP_constructCounting(&gAdcSyncSemObject, 0, 1);

    /* Number of iterations on configured steps */
    numIter = testParams->adcConfigParams.numIterations;
    gOutOfRange = 0U;

    /* Initialize buffer */
    for(loopCnt = 0U; loopCnt < APP_ADC_MAX_SAMPLES; loopCnt++)
    {
        gAdcCpuDestBuf[loopCnt] = 0U;
    }

    /* Initialize ADC module. */
    gAdcModule = testParams->adcConfigParams.adcModule;
    st_adcModuleInit(gAdcModule);

    /* Register ADC interrupts */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = CONFIG_ADC0_INTR;
    hwiPrms.callback = &AppADCIntrISR;
    hwiPrms.priority = 1U;
    status = HwiP_construct(&gHwiHandleDone, &hwiPrms);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* Configure ADC step */
    status = st_adcStepConfig(testParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* Enable ADC FIFO */
    if(testParams->adcConfigParams.fifoEnable == TRUE)
    {
        status = ADCSetCPUFIFOThresholdLevel(gAdcModule,
                  testParams->adcConfigParams.fifoNum,
                  testParams->adcConfigParams.fifoThreshold);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }

    gTimeStampBeforeADC = ClockP_getTimeUsec();
    ADCSetRange(gAdcModule,
                testParams->adcConfigParams.highRange,
                testParams->adcConfigParams.lowRange);

    /* Clear All interrupts */
    ADCClearIntrStatus(gAdcModule, ADC_INTR_ALL);
    /* Enable interrupts */
    ADCEnableIntr(gAdcModule, testParams->adcConfigParams.intrEnable);

    /* Start ADC conversion */
    st_adcModuleStart(gAdcModule);

    /* wait for adc completion */
    while(numIter > 0)
    {
        SemaphoreP_pend(&gAdcSyncSemObject, SystemP_WAIT_FOREVER);
        numIter--;
    }

    /*Get FIFO data and Validate */
    numElems = AppADCExtractFifoData(testParams->adcConfigParams.fifoNum, &gAdcCpuDestBuf[0]);
    if(numElems != 0U)
    {
        status =  st_adcValidateFifoData(testParams, &gAdcCpuDestBuf[0], numElems);
    }

    /* Disable ADC interrupts */
    ADCDisableIntr(gAdcModule, testParams->adcConfigParams.intrEnable);

    /* Clear any enabled steps */
    ADCClearAllSteps(gAdcModule);

    /* Stop ADC */
    st_adcModuleStop(testParams);

    /* Power down the ADC */
    ADCPowerUp(gAdcModule, FALSE);

    /* Disable Hwi and Semaphore */
    HwiP_destruct(&gHwiHandleDone);
    SemaphoreP_destruct(&gAdcSyncSemObject);

    if(gOutOfRange == 1U)
    {
        DebugP_log("The converted values were out of range of the specified range.\n\r");
    }

    DebugP_log("Time taken for conversion is %d us.\n\r",
        (uint32_t) (gTimeStampAfterADC - gTimeStampBeforeADC));

    testParams->testResult = status;

    return 0;
}

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

static uint32_t AppADCExtractFifoData(uint32_t fifoNum, uint32_t *dstBuff)
{
    uint32_t loopCnt, numElems;
    numElems = ADCGetFIFOWordCount(gAdcModule, fifoNum);

    for(loopCnt = 0U; loopCnt < numElems; loopCnt++)
    {
        *dstBuff = ADCGetFIFOData(gAdcModule, fifoNum);
        dstBuff++;
    }
    return numElems;
}

static void AppADCIntrISR(void *handle)
{
    uint32_t status;

    gTimeStampAfterADC = ClockP_getTimeUsec();
    status = ADCGetIntrStatus(gAdcModule);
    if((status & ADC_INTR_SRC_OUT_OF_RANGE) > 0)
    {
        gOutOfRange = 1U;
    }
    ADCClearIntrStatus(gAdcModule, ADCGetIntrRawStatus(gAdcModule));
    ADCWriteEOI(gAdcModule);
    SemaphoreP_post(&gAdcSyncSemObject);
}
