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

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static uint32_t             gAdcModule;
static uint64_t             gTimeStampBeforeADC, gTimeStampAfterADC;
/* Application Buffers */
uint32_t gAdcCpuDestBuf[APP_ADC_MAX_SAMPLES];

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static uint32_t AppADCExtractFifoData(uint32_t fifoNum, uint32_t *dstBuff);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t st_adcPollingMode_main(st_ADCTestcaseParams_t *testParams)
{
    int32_t         status;
    uint32_t        loopCnt, numElems;

    /* Initialize ADC configuration params */
    gAdcModule = testParams->adcConfigParams.adcModule;

    /* Get and print revision ID */
    adcRevisionId_t adcRevision;
    ADCGetRevisionId(gAdcModule, &adcRevision);
    DebugP_log("ADC Scheme : %d \r\n", adcRevision.scheme);
    DebugP_log("ADC Functional Release number : %d \r\n", adcRevision.func);
    DebugP_log("ADC Design Release number : %d \r\n", adcRevision.rtlRev);
    DebugP_log("ADC Major Revision number : %d \r\n", adcRevision.major);
    DebugP_log("ADC Custom Version number : %d \r\n", adcRevision.custom);
    DebugP_log("ADC Minor Version number : %d \r\n", adcRevision.minor);

    for(loopCnt = 0U; loopCnt < APP_ADC_MAX_SAMPLES; loopCnt++)
    {
        gAdcCpuDestBuf[loopCnt] = 0U;
    }

    /* Initialize ADC module. */
    st_adcModuleInit(gAdcModule);

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

    /* Start ADC conversion */
    st_adcModuleStart(gAdcModule);

    /* Poll for adc completion */
    adcSequencerStatus_t adcStatus;
    ADCGetSequencerStatus(gAdcModule, &adcStatus);
    while(adcStatus.fsmBusy == 1)
    {
        ADCGetSequencerStatus(gAdcModule, &adcStatus);
    }

    gTimeStampAfterADC = ClockP_getTimeUsec();

    /*Get FIFO data and Validate */
    numElems = AppADCExtractFifoData(testParams->adcConfigParams.fifoNum,
                                     &gAdcCpuDestBuf[0]);
    if(numElems != 0U)
    {
        status =  st_adcValidateFifoData(testParams, &gAdcCpuDestBuf[0], numElems);
    }

    /* Clear any enabled steps */
    ADCClearAllSteps(gAdcModule);

    /* Stop ADC */
    st_adcModuleStop(testParams);

    /* Power down the ADC */
    ADCPowerUp(gAdcModule, FALSE);

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
