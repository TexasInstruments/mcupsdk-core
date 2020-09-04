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
#include "st_adcTestCases.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */
void adcRunTestcase(void *args);
void st_adcTCResultInit(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static st_ADCTestcaseParams_t *gTestParams;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    int32_t  testcaseIdx;

    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    /* Initialization for tests */
    st_adcTCResultInit();

    /* Run all tests */
    for(testcaseIdx = 0; testcaseIdx < ADC_NUM_TESTCASES; testcaseIdx++)
    {
        gTestParams = &gADCTestcaseParams[testcaseIdx];
        RUN_TEST(adcRunTestcase, gTestParams->testcaseId, NULL);
    }

    UNITY_END();

    /* Close drivers */
    Board_driversClose();
    Drivers_close();

    return;
}

/* Unity framework required information */
void setUp(void)
{
}

void tearDown(void)
{
}

void adcRunTestcase(void *args)
{
    DebugP_log("\r\nTest ID : MCU-SDK %u \r\n", gTestParams->testcaseId);
    DebugP_log("Test Description : %s \r\n", gTestParams->testCaseName);

    if(gTestParams->adcConfigParams.testMode == ADC_TEST_MODE_CPU)
    {
        st_adcCpuMode_main(gTestParams);
    }
    else if(gTestParams->adcConfigParams.testMode == ADC_TEST_MODE_DMA)
    {
        st_adcDmaMode_main(gTestParams);
    }
    else
    {
        st_adcPollingMode_main(gTestParams);
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, gTestParams->testResult);
}

void st_adcTCResultInit(void)
{
    uint32_t loopCnt;
    st_ADCTestcaseParams_t * testParams;

    for(loopCnt = 0 ; loopCnt < ADC_NUM_TESTCASES; loopCnt++)
    {
        testParams              = &gADCTestcaseParams[loopCnt];
        testParams->testResult  = SystemP_FAILURE;
    }
}
