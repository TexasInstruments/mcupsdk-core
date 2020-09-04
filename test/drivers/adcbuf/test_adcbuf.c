/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <drivers/adcbuf.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Test definition specific to the device */
#define TEST_ADCBUF_TOTAL_TEST_CASES        (4U)
#define TEST_ADCBUF_NUM_SAMPLES             (1024U)
#define TEST_ADCBUF_DMADATA_BASE_ADDRESS    (CSL_DSS_L3_U_BASE + 0x8000U)

#if defined(_TMS320C6X)
#define RSS_ADC_CAPTURE_COMPLETE_IRQ        (57U)
#else
#define RSS_ADC_CAPTURE_COMPLETE_IRQ        (142U)
#endif

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct ADCBufTestParams_t
{
    /* Number of ADC samples */
    uint32_t    numSamples;
    /* Device specific ADCBUF memory address */
    uint32_t    memAddr;
    /* Memory address to store ADC samples */
    uint32_t    dstMemAddr;
    /* Chirp Interrupt Number */
    uint32_t    chirpIntNumber;
} ADCBufTestParams;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void test_api(void *args);
static void test_cmdParamsCheck(void *args);
static void test_continuousMode(void *args);

/* ISR */
static void test_intrCallBackFunc(void *arg);

/* Helper functions */
static uint32_t test_verifyDataPattern(void *memAddr, ADCBuf_dataFormat *ptrDataFormat);
static void test_contDataPathWithTestPattern(ADCBuf_Handle handle, ADCBuf_dataFormat *ptrDataFormat);
static void test_contModeConfig(ADCBuf_Handle handle, ADCBuf_dataFormat *ptrDataFormat);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Device specific test parameters */
static ADCBufTestParams gADCBufTestParam =
{
    TEST_ADCBUF_NUM_SAMPLES,
    CSL_RSS_ADCBUF_READ_U_BASE,
    TEST_ADCBUF_DMADATA_BASE_ADDRESS,
    RSS_ADC_CAPTURE_COMPLETE_IRQ
};

/* Test cases with different data format */
static ADCBuf_dataFormat gADCBufDataFmtTestCase[TEST_ADCBUF_TOTAL_TEST_CASES] =
{
    {0, 0, 0},      /* Complex, Q+I, non-interleaved */
    {0, 1, 0},      /* Complex, I+Q, non-interleaved */
    {1, 0, 0},      /* Real, I, non-interleaved */
    {1, 1, 0},      /* Real, Q, non-interleaved */
};

/* Test Control variables */
static volatile uint32_t  gADCBufTestRun = 1U;
static SemaphoreP_Object  gADCBufIntSemObj;

/* Test statistics */
static volatile uint32_t  gADCBufIntCounter = 0U;
static volatile uint32_t  gADCBufVerifyFailCount = 0U;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    CycleCounterP_reset();

    RUN_TEST(test_api,            1782, NULL);
    RUN_TEST(test_cmdParamsCheck, 1783, NULL);
    RUN_TEST(test_continuousMode, 1784, NULL);

    UNITY_END();
    Drivers_close();

    return;
}

/*
 * Unity framework required functions
 */
void setUp(void)
{
}

void tearDown(void)
{
}

/*
 * Testcases
 */
static void test_api(void *args)
{
    ADCBuf_Params        params;
    ADCBuf_Handle        handle;

    /* Open the first ADCBUF Instance */
    ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
    if(handle == NULL)
    {
        DebugP_log("Error: Unable to open the ADCBUF Instance\r\n");
        TEST_ASSERT_NOT_NULL(handle);
    }

    DebugP_log("ADCBUF Instance has been opened successfully\r\n");
    ADCBuf_close(handle);
    DebugP_log("ADCBUF Instance has been closed successfully\r\n");

    return;
}

static void test_cmdParamsCheck(void *args)
{
    ADCBuf_Params       params;
    ADCBuf_Handle       handle;
    uint32_t            arg;
    int32_t             retVal;
    ADCBufMMWave_CMD    command;
    ADCBuf_dataFormat   dataFormat;
    ADCBuf_RxChanConf   rxChanConf;
    ADCBuf_CQConf       cqConf;
    int32_t             testResult = SystemP_SUCCESS;

    /* Open the first ADCBUF Instance */
    ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
    if(handle == NULL)
    {
        DebugP_log("Error: Unable to open the ADCBUF Instance\r\n");
        TEST_ASSERT_NOT_NULL(handle);
    }
    DebugP_log("ADCBUF Instance(0) %p has been opened successfully\r\n", handle);

    /* Params check Test */
    command = ADCBufMMWave_CMD_SET_SRC;
    arg = 0x5;
    if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with arg=%d\r\n", command, arg);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with arg=%d\r\n", command, arg);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_SET_CONTINUOUS_MODE;
    arg = 0x8;
    if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with arg=%d\r\n", command, arg);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with arg=%d\r\n", command, arg);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_CONF_DATA_FORMAT;
    memset((void *)&dataFormat, 0, sizeof(dataFormat));
    dataFormat.adcOutFormat = 2;
    if((retVal = ADCBuf_control(handle, command, (void *)&dataFormat)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with adcOutFormat=%d \r\n", command, dataFormat.adcOutFormat);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with adcOutFormat=%d\r\n", command, dataFormat.adcOutFormat);
        testResult = SystemP_FAILURE;
    }

    memset((void *)&dataFormat, 0, sizeof(dataFormat));
    dataFormat.sampleInterleave = 2;
    if((retVal = ADCBuf_control(handle, command, (void *)&dataFormat)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with sampleInterleave=%d\r\n", command, dataFormat.sampleInterleave);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with sampleInterleave=%d\r\n", command, dataFormat.sampleInterleave);
        testResult = SystemP_FAILURE;
    }

    memset((void *)&dataFormat, 0, sizeof(dataFormat));
    dataFormat.channelInterleave = 2;
    if((retVal = ADCBuf_control(handle, command, (void *)&dataFormat)) < 0)
    {
        DebugP_log("Passed the param check for command=%d channelInterleave=%d\r\n", command, dataFormat.channelInterleave);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with channelInterleave=%d\r\n", command, dataFormat.channelInterleave);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_CONF_CQ;
    memset((void *)&cqConf, 0, sizeof(cqConf));
    cqConf.cqDataWidth= 5;
    if((retVal = ADCBuf_control(handle, command, (void *)&cqConf)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with cqDataWidth=%d \r\n", command, cqConf.cqDataWidth);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with cqDataWidth=%d\r\n", command, cqConf.cqDataWidth);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_CONF_CQ;
    memset((void *)&cqConf, 0, sizeof(cqConf));
    cqConf.cq96BitPackEn = 4;
    if((retVal = ADCBuf_control(handle, command, (void *)&cqConf)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with cq96BitPackEn=%d \r\n", command, cqConf.cqDataWidth);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with cq96BitPackEn=%d\r\n", command, cqConf.cq96BitPackEn);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_START_CONTINUOUS_MODE;
    arg = 0x1U<<16;
    if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
    {
        DebugP_log("Passed the param check for command=%d \r\n", command);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with arg=%d\r\n", command, arg);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_CHANNEL_ENABLE;
    memset((void *)&rxChanConf, 0, sizeof(rxChanConf));
    rxChanConf.channel = 6;
    if((retVal = ADCBuf_control(handle, command, (void *)&rxChanConf)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with channel = %d\r\n", command, rxChanConf.channel);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with channel=%d\r\n", command, rxChanConf.channel);
        testResult = SystemP_FAILURE;
    }

    memset((void *)&rxChanConf, 0, sizeof(rxChanConf));
    rxChanConf.channel = 2;
    rxChanConf.offset = 0x1U << 15U;
    if((retVal = ADCBuf_control(handle, command, (void *)&rxChanConf)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with offset = %d\r\n", command, rxChanConf.offset);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with offset=%d\r\n", command, rxChanConf.offset);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_SET_CHIRP_THRESHHOLD;
    arg = 35;
    if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with arg = %d , retVal=%d\r\n", command, arg, retVal);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD;
    arg = 35;
    if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD;
    arg = 35;
    if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_CHANNEL_DISABLE;
    arg = 0xc;
    if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
    {
        DebugP_log("Error: Failed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
        testResult = SystemP_FAILURE;
    }
    else
    {
        DebugP_log("Passed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
    }

    command = ADCBufMMWave_CMD_CHANNEL_DISABLE;
    arg = 0x1c;
    if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
        testResult = SystemP_FAILURE;
    }

    command = ADCBufMMWave_CMD_CHANNEL_DISABLE;
    arg = 0x0;
    if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
    {
        DebugP_log("Passed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
    }
    else
    {
        DebugP_log("Error: Failed the param check for command=%d with arg = %d, retVal=%d\r\n", command, arg, retVal);
        testResult = SystemP_FAILURE;
    }

    /* Close ADCbuf driver */
    ADCBuf_close(handle);
    DebugP_log("ADCBUF Instance has been closed successfully\r\n");
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testResult);

    return;
}

static inline CSL_dss_ctrlRegs* CSL_DSS_CTRL_getBaseAddress (void)
{
    return (CSL_dss_ctrlRegs *) CSL_DSS_CTRL_U_BASE;
}

static void test_continuousMode(void *args)
{
    HwiP_Params         hwiPrms;
    HwiP_Object         hwiObject;
    ADCBuf_Params       params;
    ADCBuf_Handle       handle;
    ADCBuf_dataFormat   dataFormat;
    int32_t             retVal;
    uint8_t             index;
    volatile uint32_t   srcMemAddr;
    volatile uint32_t   dstMemAddr;
    uint32_t            numSamples;
    ADCBuf_dataFormat  *ptrDataFormat = &dataFormat;
    CSL_dss_ctrlRegs *dssCtrl = CSL_DSS_CTRL_getBaseAddress();

    /* Initialize L3 memory banks. */
    CSL_REG_WR((uint32_t*)(CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_START), 0x1);

    while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE) != 1);

    /* Clear L3 memory initialization Status. */
    CSL_REG_WR((uint32_t*)(CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE), 0x1U);

    /* Configure delay. Applicable in HIL mode. */
    CSL_REG_WR((uint32_t*)(CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_ADCBUFCFG1_EXTD), 0x0U);

    /* Create a binary semaphore which is used to handle interrupt handling. */
    retVal = SemaphoreP_constructBinary(&gADCBufIntSemObj, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    /* Setup the default ADCBUF Parameters */
    ADCBuf_Params_init(&params);
    params.continousMode = 1;

    /* Go through different data format test cases */
    for(index = 0; index < TEST_ADCBUF_TOTAL_TEST_CASES; index++)
    {
        handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
        if(handle == NULL)
        {
            DebugP_log("Error: Unable to open the ADCBUF Instance\r\n");
            TEST_ASSERT_NOT_NULL(handle);
        }
        DebugP_log("ADCBUF Instance has been reopened successfully\r\n");

        /* Register chirp interrupt */
        HwiP_Params_init(&hwiPrms);
        hwiPrms.args            = handle;
        hwiPrms.callback        = &test_intrCallBackFunc;
        hwiPrms.intNum          = gADCBufTestParam.chirpIntNumber;
        hwiPrms.isPulse         = TRUE;
        /* Register interrupts */
        retVal = HwiP_construct(&hwiObject, &hwiPrms);
        if(SystemP_SUCCESS != retVal)
        {
            DebugP_log("Error Could not register ISR !!!\n");
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
        }

        /* Initialize the data format for the test */
        dataFormat = gADCBufDataFmtTestCase[index];

        /* Print out the test params */
        DebugP_log("ADCBUF Test Pattern with dataFormat=%d, sampleSwap=%d, interleave=%d \r\n",
                       dataFormat.adcOutFormat, dataFormat.sampleInterleave,
                       dataFormat.channelInterleave);

        /* Number of loops that test run for current data Format
            Data is validated for every 10 ping/pong buffer swap.
          */
        gADCBufTestRun = 20;

        /* ADCBuf Test Pattern Test, this function will be blocked until gADCBufTestRun is set to 0 */
        test_contDataPathWithTestPattern(handle, &dataFormat);

        /* Assign memory base address to save the ADC samples */
        dstMemAddr = (uint32_t) SOC_phyToVirt(gADCBufTestParam.dstMemAddr);
        srcMemAddr = (uint32_t) SOC_phyToVirt(gADCBufTestParam.memAddr);
        numSamples = gADCBufTestParam.numSamples;
        while(gADCBufTestRun)
        {
            /* Wait for the Semaphore */
            SemaphoreP_pend(&gADCBufIntSemObj, SystemP_WAIT_FOREVER);

            /* Copy the adcbuf data to memory */
            memcpy((void *)dstMemAddr,  (void *)srcMemAddr, (16 * numSamples));
            CacheP_wbInv((void *)dstMemAddr, (16 * numSamples), CacheP_TYPE_ALL);
            CacheP_inv((void *)srcMemAddr, (16 * numSamples), CacheP_TYPE_ALL);
            CacheP_inv((void *)dstMemAddr, (16 * numSamples), CacheP_TYPE_ALL);

            /* Verify data integrity */
            if(test_verifyDataPattern((void *)dstMemAddr, ptrDataFormat) !=0)
            {
                gADCBufVerifyFailCount++;
            }

            gADCBufTestRun--;

            /* Re-Start Test Pattern generation if test is not stopped */
            if(gADCBufTestRun)
            {
                uint32_t  rxChanMask;

                /* Disable all channels */
                rxChanMask = 0xF;
                if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask)) < 0)
                {
                    DebugP_log("Error: ADCBufMMWave_CMD_CHANNEL_DISABLE failed with [Error=%d]\r\n", retVal);
                    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
                }

                /* ADCBuf Test Pattern Test, this function will be blocked until gADCBufTestRun is set to 0 */
                test_contDataPathWithTestPattern(handle, &dataFormat);
            }
        }

        /* StopTest Pattern generation */
        if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_STOP_TEST_PATTERN, NULL)) < 0)
        {
            DebugP_log("Error: ADCBufMMWave_CMD_STOP_TEST_PATTERN failed with [Error=%d]\r\n", retVal);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
        }

        /* Close  ADCBuf driver */
        ADCBuf_close(handle);
        DebugP_log("ADCBUF Instance has been closed successfully\r\n");

        /* Deregister chirp interrupt listener */
        HwiP_destruct(&hwiObject);

        /* Send test results to logger */
        if(gADCBufVerifyFailCount > 0)
        {
            DebugP_log("ADCBUF Test Pattern failed\r\n");
            DebugP_log("gADCBufVerifyFailCount = %d\r\n", gADCBufVerifyFailCount);
        }
        else
        {
            DebugP_log("ADCBUF Test Pattern passed\r\n");
        }
        TEST_ASSERT_EQUAL_UINT32(0, gADCBufVerifyFailCount);

        /* Clear the counter */
        gADCBufIntCounter = 0;
        gADCBufVerifyFailCount = 0;
    }

    /* Delete semaphore */
    SemaphoreP_destruct(&gADCBufIntSemObj);

    return;
}

/*
 * ISR
 */
static void test_intrCallBackFunc(void *arg)
{
    ADCBuf_Handle     handle;

    handle = (ADCBuf_Handle)arg;

    /* Increment interrupt counter for debugging purpose */
    gADCBufIntCounter++;

    /* Check the buffer every 10 chirp avail interrupt */
    if(gADCBufIntCounter % 10 == 0)
    {
        /* Stop Continuous mode */
        ADCBuf_control(handle, ADCBufMMWave_CMD_STOP_CONTINUOUS_MODE, NULL);

        /* Stop Test Pattern generation */
        ADCBuf_control(handle, ADCBufMMWave_CMD_STOP_TEST_PATTERN, NULL);

        /* Semaphore post for test thread to copy samples and verify */
        SemaphoreP_post(&gADCBufIntSemObj);
    }

    return;
}

/*
 * Helper functions
 */
static uint32_t test_verifyDataPattern(void *memAddr, ADCBuf_dataFormat *ptrDataFormat)
{
    volatile uint32_t*   ptrDataAddress;
    uint8_t              channel;
    uint32_t             sampleIdx;
    volatile uint32_t    addrOffset;
    uint32_t             data, firstSample;
    uint16_t             idata, qdata;
    uint32_t             incr = 2;
    uint32_t             failFlag = 0;
    uint32_t             dataIncr;
    uint32_t             numSamples;

    ptrDataAddress = (volatile uint32_t *)memAddr;
    DebugP_log("First 4 32 bit words 0x%x     0x%x    0x%x    0x%x\r\n", ptrDataAddress[0],
                        ptrDataAddress[1], ptrDataAddress[2], ptrDataAddress[3]);

    /* Set the address offset */
    addrOffset = 0;

    /* Get number of samples from test params */
    numSamples = gADCBufTestParam.numSamples;

    /* Get the first sample */
    firstSample = *ptrDataAddress;

    /* Channel interleaved-mode */
    if(ptrDataFormat->channelInterleave == 0)
    {
        /* Test 10 samples for all channels */
        for(sampleIdx=0; sampleIdx < numSamples; sampleIdx++)
        {
            for(channel = 0; channel < 4; channel++)
            {
                addrOffset = sampleIdx * 4;
                if(ptrDataFormat->adcOutFormat == 0)
                {
                    data = ptrDataAddress[addrOffset + channel];

                    /* Complex data format, get I and Q data */
                    if(ptrDataFormat->sampleInterleave == 0)
                    {
                        qdata = (data & 0xffff0000) >> 16;
                        idata  = (data & 0xffff);
                    }
                    else
                    {
                        idata = (data & 0xffff0000) >> 16;
                        qdata  = (data & 0xffff);
                    }

                    /* Verify data 1: increment = 2, */
                    dataIncr =( ( sampleIdx * incr)<<16) + (incr * sampleIdx);
                    if((data - firstSample) != dataIncr)
                    {
                         failFlag = 1;
                    }

                    /* Verify data 2: Q = I + 0x1000 */
                    if( (idata + 0x1000) != qdata)
                    {
                         failFlag = 1;
                    }
                }
                else
                {
                    uint16_t *ptrRealData = (uint16_t *)ptrDataAddress;
                    uint16_t  realData;
                    uint16_t  firstRealSample;

                    /* Address offset in terms of 16bits real data */
                    addrOffset = sampleIdx * 4;
                    realData = ptrRealData[addrOffset + channel];
                    firstRealSample = ptrRealData[channel];

                    /* Verify data 1: increment = 2, */
                    dataIncr =incr * sampleIdx;
                    if((realData - firstRealSample) != dataIncr)
                    {
                         failFlag = 1;
                    }
                }
            }
        }
    }
    else
    {
        for(channel = 0; channel < 4; channel++)
        {
            addrOffset = (channel * numSamples) ;
            firstSample = ptrDataAddress[addrOffset];

            for(sampleIdx=0; sampleIdx<numSamples; sampleIdx++)
            {
                data = ptrDataAddress[addrOffset + sampleIdx];

                if(ptrDataFormat->adcOutFormat == 0)
                {
                    /* Complex data format, get I and Q data */
                    if(ptrDataFormat->sampleInterleave == 0)
                    {
                        qdata = (data & 0xffff0000) >> 16;
                        idata  = (data & 0xffff);
                    }
                    else
                    {
                        idata = (data & 0xffff0000) >> 16;
                        qdata  = (data & 0xffff);
                    }

                    /* Verify data 1: increment = 2, */
                    dataIncr =( ( sampleIdx * incr)<<16) + (incr * sampleIdx);
                    if((data - firstSample) != dataIncr)
                    {
                         failFlag = 1;
                    }

                    /* Verify data 2: Q = I + 0x1000 */
                    if( (idata + 0x1000) != qdata)
                    {
                         failFlag = 1;
                    }
                }
                else
                {
                    volatile uint16_t *ptrRealData = (uint16_t *)ptrDataAddress;
                    uint16_t  realData;
                    uint16_t  firstRealSample;

                    /* Address offset in terms of 16bits real data */
                    addrOffset = (channel * numSamples * 2) ;
                    realData = ptrRealData[addrOffset + sampleIdx];
                    firstRealSample = ptrRealData[addrOffset];

                    /* Verify data 1: increment = 2, */
                    dataIncr =incr * sampleIdx;
                    if((realData - firstRealSample) != dataIncr)
                    {
                         failFlag = 1;
                    }
                }
            }
        }
    }

    return ((failFlag == 0) ? 0 : 1);
}

static void test_contDataPathWithTestPattern(ADCBuf_Handle handle, ADCBuf_dataFormat *ptrDataFormat)
{
    int32_t     retVal = 0;
    uint32_t    numOfClks = 0x32;

    /* Configure test pattern */
    test_contModeConfig(handle, ptrDataFormat);

    /* Start Test Pattern generation */
    if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_START_TEST_PATTERN, (void *)&numOfClks)) < 0)
    {
        DebugP_log("Error: ADCBufMMWave_CMD_START_TEST_PATTERN failed with [Error=%d]\r\n", retVal);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    }

    return;
}

static void test_contModeConfig(ADCBuf_Handle handle, ADCBuf_dataFormat *ptrDataFormat)
{
    uint32_t                 arg;
    ADCBuf_RxChanConf        rxChanConf;
    ADCBuf_TestPatternConf   testPatternConf;
    uint8_t                  channel;
    uint16_t                 offset = 0;
    uint32_t                 numSamples;
    int32_t                  retVal = 0;

    /* Configure ADC buffer data format */
    if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, (void *)ptrDataFormat)) < 0)
    {
        DebugP_log("Error: ADCBufMMWave_CMD_CONF_DATA_FORMAT failed with [Error=%d]\r\n", retVal);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    }

    /* Enable all 4  Rx Channel */
    for(channel=0; channel < SOC_ADCBUF_NUM_RX_CHANNEL; channel++)
    {
        rxChanConf.channel = channel;
        rxChanConf.offset    = offset;
        if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf)) < 0)
        {
            DebugP_log("Error: ADCBufMMWave_CMD_CHANNEL_ENABLE failed for channel %d offset =0x%x with [Error = %d]\r\n",
                                     channel, offset, retVal);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
        }
        /* Test purpose only : Verify channel address */
        {
            uint32_t channelAddr = 0;

            if((channelAddr = ADCBuf_getChanBufAddr(handle, channel, &retVal)) != 0)
            {
                channelAddr -= gADCBufTestParam.memAddr;
                if(channelAddr != offset)
                {
                    DebugP_log("Error: ADCBuf_getChanBufAddr() return mismatched channel(%d) buffer address [%x: %x]\r\n",
                                 channel, offset, channelAddr);

                    retVal = -1;
                    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
                }
            }
            else
            {
                DebugP_log("Error: ADCBuf_getChanBufAddr failed for channel %d with [Error = %d]\r\n",
                                         channel, retVal);
                TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
            }
        }

        offset += gADCBufTestParam.numSamples * 4;
    }

    /* Configure ADC buffer in continuous mode */
    arg = 1;
    if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_SET_CONTINUOUS_MODE, (void *)&arg)) < 0)
    {
        DebugP_log("Error: ADCBufMMWave_CMD_SET_CONTINUOUS_MODE failed with [Error=%d]\r\n", retVal);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    }

    /* Start the continuous streaming mode in ADCBUFF */
    numSamples = gADCBufTestParam.numSamples;
    if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_START_CONTINUOUS_MODE, (void *)&numSamples)) < 0)
    {
        DebugP_log("Error: ADCBufMMWave_CMD_START_CONTINUOUS_MODE failed with [Error=%d]\r\n", retVal);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    }

    /* Configure Test Pattern generation */
    testPatternConf.period = 255;
    testPatternConf.numSamples = gADCBufTestParam.numSamples;
    for(channel=0; channel < 4; channel++)
    {
        testPatternConf.rxConfig[channel].rxIOffset   = 0x0000;
        testPatternConf.rxConfig[channel].rxIInc      = 2;
        testPatternConf.rxConfig[channel].rxQOffset   = 0x1000 ;
        testPatternConf.rxConfig[channel].rxQInc      = 2;
    }

    /* Send control command to driver */
    if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_CONF_TEST_PATTERN, (void *)&testPatternConf)) < 0)
    {
        DebugP_log("Error: ADCBufMMWave_CMD_CONF_TEST_PATTERN failed with [Error=%d]\r\n", retVal);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    }

    return;
}
