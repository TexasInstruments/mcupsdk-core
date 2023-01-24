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
#include <drivers/hwa.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

/* input vector */
#include "hwa_testvector_input.h"
#include "hwa_testvector_output.h"
#include "fft_window.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DRV_API_OPEN                    (0U)
#define DRV_API_CLOSE                   (1U)
#define DRV_API_ENABLE                  (2U)
#define DRV_API_SOFTWARETRIG            (3U)
#define DRV_API_DMAMANUALTRIG           (4U)
#define DRV_API_SETSRCADDR              (5U)
#define DRV_API_MAX                     (6U)

/* ADC data charateristics */
#define HWA_TEST_NUM_SAMPLES            (225)
#define HWA_TEST_1DFFT_SIZE             (256)
#define HWA_TEST_NUM_RX_ANT             (4)
#define HWA_TEST_COMPLEX_16BIT_SIZE     (4)
#define HWA_TEST_COMPLEX_32BIT_SIZE     (8)
/* Input data characteristics */
#define HWA_TEST_IS_INPUT_REAL          (0)
#define HWA_TEST_IS_INPUT_SIGNED        (1)
/* window RAM */
#define HWA_TEST_1DFFT_WINDOW_START     (0)
/* DMA channel */
#define HWA_TEST_SRC_TRIGGER_DMACH0     (0)
#define HWA_TEST_SRC_TRIGGER_DMACH1     (1)

#define HWA_MEMn_SIZE                   (CSL_DSS_HWA_BANK_SIZE) //16K size

/* benchmark collect flag, if defined, benchmark is collected after functional test pass, if un-defined, no benchmark is collcted after the funtional test
 * default is HWA_TEST_COLLECTBENCHMARK defined
 */
//#define HWA_TEST_COLLECTBENCHMARK

/* hwa memory cache is disabeld by default in the test */
#define SOC_HWA_MEM0                    (CSL_DSS_HWA_DMA0_RAM_BANK0_BASE)
#define SOC_HWA_MEM1                    (CSL_DSS_HWA_DMA0_RAM_BANK1_BASE)
#define SOC_HWA_MEM2                    (CSL_DSS_HWA_DMA0_RAM_BANK2_BASE)
#define SOC_HWA_MEM3                    (CSL_DSS_HWA_DMA0_RAM_BANK3_BASE)
#define SOC_HWA_MEM4                    (CSL_DSS_HWA_DMA0_RAM_BANK4_BASE)
#define SOC_HWA_MEM5                    (CSL_DSS_HWA_DMA0_RAM_BANK5_BASE)
#define SOC_HWA_MEM6                    (CSL_DSS_HWA_DMA0_RAM_BANK6_BASE)

#define TEST_PARAMSET_INTERRUPT

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void test_fftwithPreproc(void *args);
static void test_complexmultiply(void *args);
static void test_azimfft(void *args);
static void test_histogram(void *args);
static void test_compress(void *args);
static void test_2dfft(void *args);
static void test_fft4k(void *args);
static void test_contextswitch(void *args);
#if defined (SOC_AWR294X)
static void test_SW_restartLoop(void *args);
#endif

/* ISR */
static void HWA_Test_ParamSetISR_Callback(uint32_t intrIdx, uint32_t paramSet, void *arg);
static void HWA_contextswitch_Test_ParamSetISR_Callback(uint32_t intrIdx, uint32_t paramSet, void *arg);
static void HWA_Test_DoneISR_Callback(uint32_t threadIdx, void *arg);
static void HWA_Test_ALT_DoneISR_Callback(uint32_t threadIdx, void *arg);
#if defined (SOC_AWR294X)
static void HWA_swRestartLoop_Test_ParamSetISR_Callback(uint32_t intrIdx, uint32_t paramSet, void *arg);
#endif

/* Helper functions */
static uint32_t log2Approx(uint32_t x);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

HWA_Handle   gHwaHandle;
uint32_t     drvApiCycles[DRV_API_MAX];

/* using this as global to facilitate debugging */
HWA_ParamConfig gHWATestParamConfig[16];
HWA_CommonConfig gCommonConfig;

uint32_t gHWATestParamSetISR = 0;
volatile uint32_t gHWATestDoneISR;
volatile uint32_t gHWAcontextswitchTestParamSetIdx[90 + 7];
volatile uint8_t gHWAcontextswitchTestParamIdx = 0;
volatile uint8_t gHWAOneLoopDone = 0;
volatile uint8_t gHWAALTThreadDone = 0; //ALT thread done interrupt flag,
volatile uint8_t gHWAcontextswitchParamDone = 0; //flag for every paramset done interrupt
volatile uint8_t intr1ParamDoneCount = 0;
volatile uint8_t intr2ParamDoneCount = 0;
#if defined (SOC_AWR294X)
volatile uint8_t SW_RestartLoop = 0;
volatile uint8_t intr1ParamDoneCount_swRestartLoop = 0;
#endif

cmplx32ImRe_t     DCEstResult[2 * HWA_TEST_NUM_RX_ANT];
uint32_t          magEstResult[HWA_TEST_NUM_RX_ANT];
uint32_t          magDiffEstResult[HWA_TEST_NUM_RX_ANT];
cmplx64ImRe_t     DCAccResult[HWA_TEST_NUM_RX_ANT];
uint64_t          magAccResult[HWA_TEST_NUM_RX_ANT];
uint64_t          magDiffAccResult[HWA_TEST_NUM_RX_ANT];
uint16_t          shuffleIdx[HWA_TEST_NUM_SAMPLES];
uint32_t          fft_input_32bits[256];
uint32_t          vectorArray[HWA_TEST_NUM_SAMPLES * 2];
HWA_Stats         statsResults[4];
HWA_CdfThreshold  cdfThresholdResults[16];

#ifdef HWA_TEST_COLLECTBENCHMARK
#pragma DATA_SECTION(benchmarkData, ".benchmarkL2");
uint8_t           benchmarkData[16384];   //for benchmark data transfer from hwa to L2.
#ifdef BUILD_DSP_1
#pragma DATA_SECTION(benchmarkL1Data, ".benchmarkL1");
uint8_t           benchmarkL1Data[16384];
#endif
#ifdef BUILD_MCU
#pragma DATA_SECTION(benchmarkL1Data, ".benchmarkL1");
uint8_t           benchmarkL1Data[8192];
#endif
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    int32_t                 errCode;
    volatile uint32_t       startTime;
    volatile uint32_t       endTime;
#if defined (SOC_AWR294X)
    HWA_Object              *ptrHWADriver;
#endif


    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    CycleCounterP_reset();

    /* Driver Open */
    startTime = CycleCounterP_getCount32();
    gHwaHandle = HWA_open(0, NULL, &errCode);
    endTime = CycleCounterP_getCount32();
    drvApiCycles[DRV_API_OPEN] = endTime - startTime;
    if(gHwaHandle == NULL)
    {
        DebugP_log("Error: Unable to open the HWA Instance err: %d\n", errCode);
        DebugP_log("Feature: API HWA_open() FAIL\n");
        TEST_ASSERT_NOT_NULL(gHwaHandle);
    }
    DebugP_log("HWA Instance has been opened successfully\n");

    RUN_TEST(test_fftwithPreproc,   1270, NULL);
    RUN_TEST(test_complexmultiply,  1271, NULL);
    RUN_TEST(test_azimfft,          1272, NULL);
    RUN_TEST(test_histogram,        1273, NULL);
    RUN_TEST(test_compress,         1274, NULL);
    RUN_TEST(test_2dfft,            1275, NULL);
    RUN_TEST(test_fft4k,            1276, NULL);
    RUN_TEST(test_contextswitch,    1277, NULL);

#if defined (SOC_AWR294X)
    ptrHWADriver = (HWA_Object *)gHwaHandle;

    if(ptrHWADriver->isES2P0Device == true)
    {
        RUN_TEST(test_SW_restartLoop,   1278, NULL);
    }
#endif

    HWA_controlPeripheralSuspendMode(gHwaHandle, 0);

    /* Driver Close */
    startTime = CycleCounterP_getCount32();
    errCode = HWA_close(gHwaHandle);
    endTime = CycleCounterP_getCount32();
    drvApiCycles[DRV_API_CLOSE] = endTime - startTime;
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("HWA Instance has been closed successfully\n");

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
static void test_fftwithPreproc(void *args)
{
    HWA_Handle              handle = gHwaHandle;
#if defined (SOC_AWR294X)
    HWA_Object              *ptrHWADriver = (HWA_Object*)gHwaHandle;
#endif
    int32_t                 errCode, compareCode1, compareCode2;
    uint8_t                 paramsetIdx = 0;
    uint8_t                 pingParamSetIdx, pongParamSetIdx;
    uint8_t                *srcAddr = (uint8_t*)SOC_HWA_MEM0;
    uint8_t                *dstAddr = (uint8_t*)SOC_HWA_MEM1;
#ifdef TEST_PARAMSET_INTERRUPT
    HWA_InterruptConfig     paramISRConfig;
    SemaphoreP_Object       paramSetSem;
#endif
    SemaphoreP_Object       doneSem;
    int32_t                 status;
    volatile uint32_t       startTime;
    volatile uint32_t       endTime;

    /* Test: DMA triggered mode */
    DebugP_log("\r\n------- HWA FFT with pre-processing test ------- \r\n\r\n");
    /* Do dma trigger mode test - simple FFT - one pass */
    paramsetIdx = 0;
    memset(gHWATestParamConfig, 0, sizeof(gHWATestParamConfig));
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    /*
     * ping dummy + FFT enable with DC and interference statistics estimation
     * pong dummy + FFT enable intereference localization + mitigation (all pass + all zeroout)
     * FFT with shuffle addressing
     */
    /* Step 1 - config the paramset */
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA; //Software triggered  - in demo this will be HWA_TRIG_MODE_DMA
    gHWATestParamConfig[paramsetIdx].triggerSrc = HWA_TEST_SRC_TRIGGER_DMACH0; //in demo this will be first EDMA Src channel id
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_NONE; //dummy
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (DMA Trigger paramset for ping buffer, AccelMode None) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [DMA Trigger paramset for ping buffer, AccelMode None] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : DMA Trigger AccelMode None\r\n", paramsetIdx);
    }
    paramsetIdx++;
    pingParamSetIdx = paramsetIdx;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; //Immediate following first - in demo this should be HWA_TRIG_MODE_DFE
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; //do FFT
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(srcAddr);// (uint32_t)((uint32_t)srcAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = HWA_TEST_NUM_SAMPLES - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = HWA_TEST_NUM_RX_ANT - 1; //no iterations here
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 8;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    /* M1*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(dstAddr); //(uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = HWA_TEST_1DFFT_SIZE - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSize = log2Approx(HWA_TEST_1DFFT_SIZE);
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x3; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_ENABLE; //enabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowStart = HWA_TEST_1DFFT_WINDOW_START; //start of window RAM
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.winSymm = HWA_FFT_WINDOW_NONSYMMETRIC; //non-symmetric - in demo do we make this symmetric

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    /* add DC and interference statistics processing*/
    /* DC estimation */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET;
    /* interference statistics */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET;

    /* add some non-zero DC subtraction value, not enable the DC sub */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.dcSubSelect = HWA_DCSUB_SELECT_DCEST;
    /* add some non-zero value, not enable the feature  */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramScaleSelect = 8;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramSizeSelect = 4;

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [Immediate Triggered, Ping buffer FFT processing with DC and Interference Statistics] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : Immediate Trigger AccelMode FFT with DC, interference estimation enabled \r\n", paramsetIdx);
    }
    paramsetIdx++;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA; //Immediate triggered  - in demo this will be HWA_TRIG_MODE_DMA
    gHWATestParamConfig[paramsetIdx].triggerSrc = HWA_TEST_SRC_TRIGGER_DMACH1; //in demo this will be second EDMA Src channel id
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_NONE; //dummy
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [DMA Trigger paramset for pong buffer, AccelMode None] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : DMA Trigger AccelMode None\r\n", paramsetIdx);
    }
    paramsetIdx++;
    pongParamSetIdx = paramsetIdx;
    gHWATestParamConfig[paramsetIdx] = gHWATestParamConfig[pingParamSetIdx];

    /* M2*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA ((uint32_t) dstAddr + HWA_MEMn_SIZE);

    /*disable the DC estimation */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE;
    /* disable interference statistics */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE;

    /* enable interference localization, but threshold is set to maximum*/
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdEnable = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdMode = HWA_INTERFTHRESH_MODE_MAG_AND_MAGDIFF;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdSelect = HWA_INTERFTHRESH_SELECT_SW;

    /* enabled interference mitigation */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.enable = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.countThreshold = 4;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.leftHystOrder = 3;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.rightHystOrder = 3;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.pathSelect = HWA_INTERFMITIGATION_PATH_ZEROOUT;

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() (Immediate Triggered, Pong buffer FFT processing with interference localization/mitigation enabled (all pass)) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : Immediate Trigger AccelMode FFT, with intereference mitigation enabled \r\n", paramsetIdx);
    }

    /* this paramset also enable the threshold mitigation, by zero out all the samples to zero*/
    paramsetIdx++;
    gHWATestParamConfig[paramsetIdx] = gHWATestParamConfig[pongParamSetIdx];
    /*M3*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA ( (uint32_t ) dstAddr + (2 * HWA_MEMn_SIZE));

    /* enable interference localization*/
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdEnable = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdMode = HWA_INTERFTHRESH_MODE_MAGDIFF;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdSelect = HWA_INTERFTHRESH_SELECT_SW;

    /* enabled interference mitigation */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.enable = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.countThreshold = 0;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.leftHystOrder = 3;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.rightHystOrder = 3;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfMitigation.pathSelect = HWA_INTERFMITIGATION_PATH_ZEROOUT;

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() (Immediate Triggered, FFT processing with localization/interference mitigation(all zero out)) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : Immediate Triggered, AccelMode FFT, interference mitigation with threshold 0\r\n", paramsetIdx);
    }


    /*this paramset enable the fft not use linear addressing, instead of using shuffle addressing*/
    paramsetIdx++;
    gHWATestParamConfig[paramsetIdx] = gHWATestParamConfig[pingParamSetIdx];

    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_COMPLEX_16BIT_SIZE;
    gHWATestParamConfig[paramsetIdx].source.shuffleMode = HWA_SRC_SHUFFLE_AB_MODE_ADIM;  //A dimension shuffle
    gHWATestParamConfig[paramsetIdx].source.wrapComb = HWA_TEST_NUM_SAMPLES * HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowStart = HWA_TEST_NUM_SAMPLES; //start of window RAM in terms of word

    /* DC estimation */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE;
    /* interference statistics */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE;

    /* add some non-zero DC subtraction value, not enable the DC sub */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.dcSubSelect = HWA_DCSUB_SELECT_DCSW;
    /* add some non-zero value, not enable the feature  */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramScaleSelect = 0;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramSizeSelect = 0;

    /* and enable the channel combine, but not combine any data*/
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.chanCombEn = HWA_FEATURE_BIT_ENABLE;
    /* insert a zero*/
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.zeroInsertEn = HWA_FEATURE_BIT_ENABLE;

    /* mem4*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA ( (uint32_t) dstAddr + 3 * HWA_MEMn_SIZE);
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() (Immediate Triggered, FFT enabled with shuffle addressing(wraparound) + channel combine + zeroinsert) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
       DebugP_log("Debug: paramset %d : Immediate Trigger AccelMode FFT with shuffle addressing \r\n", paramsetIdx);
    }
    /* config shuffle ram */
    {
        uint32_t ii;
        for(ii = 0; ii < HWA_TEST_NUM_SAMPLES; ii++)
        {
            shuffleIdx[ii] = ii * HWA_TEST_NUM_RX_ANT + HWA_TEST_NUM_SAMPLES * HWA_TEST_NUM_RX_ANT;
        }
    }

    errCode = HWA_configRam(handle, HWA_RAM_TYPE_SHUFFLE_RAM, (uint8_t*)shuffleIdx, HWA_TEST_NUM_SAMPLES * sizeof(uint16_t), 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configRam(HWA_RAM_TYPE_SHUFFLE_RAM) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_configRam(HWA_RAM_TYPE_SHUFFLE_RAM) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* Step 2 - load window coefficients */
    /* if windowing is enabled, load the window coefficients in RAM */
    errCode = HWA_configRam(handle, HWA_RAM_TYPE_WINDOW_RAM, (uint8_t *)win_data225, sizeof(win_data225), HWA_TEST_1DFFT_WINDOW_START);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* if windowing is enabled, load the window coefficients in RAM for shuffling address */
    errCode = HWA_configRam(handle, HWA_RAM_TYPE_WINDOW_RAM, (uint8_t *)win_data225, sizeof(win_data225), sizeof(win_data225));  //in terms of byte
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM with offset not 0) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM with offset not 0) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

#ifdef TEST_PARAMSET_INTERRUPT
    /* Enable Paramset Interrupt for CPU and create semaphore*/
    errCode = SemaphoreP_constructBinary(&paramSetSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    paramISRConfig.cpu.callbackFn = HWA_Test_ParamSetISR_Callback;
    paramISRConfig.cpu.callbackArg = &paramSetSem;
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableParamSetInterrupt returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_enableParamSetInterrupt FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
#endif

    /* Step 3 - Enable Interrupts */
    /* Enable Done Interrut */
    errCode = SemaphoreP_constructBinary(&doneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 0, HWA_Test_DoneISR_Callback, &doneSem);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_enableDoneInterrupt() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* Step 4 - Config Common Registers */
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG | HWA_COMMONCONFIG_MASK_INTERFMAG_THRESHOLD |
        HWA_COMMONCONFIG_MASK_DCEST_SCALESHIFT |
        HWA_COMMONCONFIG_MASK_INTERFSUM_MAG |
        HWA_COMMONCONFIG_MASK_INTERFSUM_MAGDIFF |
        HWA_COMMONCONFIG_MASK_CHANCOMB_VEC_SIZE |
        HWA_COMMONCONFIG_MASK_ZEROINSERT_NUM_MASK;
    /* mag threshold */
    gCommonConfig.interfConfig.thresholdMagSw[0] = 0xffffff;
    gCommonConfig.interfConfig.thresholdMagSw[1] = 0xffffff;
    gCommonConfig.interfConfig.thresholdMagSw[2] = 0xffffff;
    gCommonConfig.interfConfig.thresholdMagSw[3] = 0xffffff;
    /* DC estimate scale, shift*/
    gCommonConfig.dcEstimateConfig.scale = 256;
    gCommonConfig.dcEstimateConfig.shift = 2;
    /* interference estimate scale, shift */
    gCommonConfig.interfConfig.sumMagScale = 8;
    gCommonConfig.interfConfig.sumMagShift = 2;
    gCommonConfig.interfConfig.sumMagDiffScale = 8;
    gCommonConfig.interfConfig.sumMagDiffShift = 2;

    /* channel combine */
    gCommonConfig.chanCombConfig.size = HWA_TEST_NUM_SAMPLES;
    gCommonConfig.chanCombConfig.vector[0] = 0xaaaaaaaa;
    gCommonConfig.chanCombConfig.vector[1] = 0xaaaaaaaa;
    gCommonConfig.chanCombConfig.vector[2] = 0xaaaaaaaa;
    gCommonConfig.chanCombConfig.vector[3] = 0xaaaaaaaa;
    gCommonConfig.chanCombConfig.vector[4] = 0xaaaaaaaa;
    gCommonConfig.chanCombConfig.vector[5] = 0xaaaaaaaa;
    gCommonConfig.chanCombConfig.vector[6] = 0xaaaaaaaa;
    gCommonConfig.chanCombConfig.vector[7] = 0x0;

#if defined (SOC_AWR294X)
    //Supported only in HWA2.0
    if(ptrHWADriver->isES2P0Device == true)
    {
        gCommonConfig.zeroInsertConfig.number = 0;

        /* COnfigure LUT RAM for Zero insertion in AWR294x ES2.0 devices only. */
        uint32_t arr_zeroInsermask[8] = {0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000001};
        HWA_configRam(handle, HWA_RAM_TYPE_LUT_FREQ_DEROTATE_RAM, (uint8_t *)arr_zeroInsermask, 32, 0);

    }
    else
    {
        /* zero insert, at the end */
        gCommonConfig.zeroInsertConfig.mask[0] = 0xffffffff;
        gCommonConfig.zeroInsertConfig.mask[1] = 0xffffffff;
        gCommonConfig.zeroInsertConfig.mask[2] = 0xffffffff;
        gCommonConfig.zeroInsertConfig.mask[3] = 0xffffffff;
        gCommonConfig.zeroInsertConfig.mask[4] = 0xffffffff;
        gCommonConfig.zeroInsertConfig.mask[5] = 0xffffffff;
        gCommonConfig.zeroInsertConfig.mask[6] = 0xffffffff;
        gCommonConfig.zeroInsertConfig.mask[7] = 0x00000001;
        gCommonConfig.zeroInsertConfig.number = 1;
    }
#else
    /* zero insert, at the end */
    gCommonConfig.zeroInsertConfig.mask[0] = 0xffffffff;
    gCommonConfig.zeroInsertConfig.mask[1] = 0xffffffff;
    gCommonConfig.zeroInsertConfig.mask[2] = 0xffffffff;
    gCommonConfig.zeroInsertConfig.mask[3] = 0xffffffff;
    gCommonConfig.zeroInsertConfig.mask[4] = 0xffffffff;
    gCommonConfig.zeroInsertConfig.mask[5] = 0xffffffff;
    gCommonConfig.zeroInsertConfig.mask[6] = 0xffffffff;
    gCommonConfig.zeroInsertConfig.mask[7] = 0x00000001;
    gCommonConfig.zeroInsertConfig.number = 1;
#endif


    gCommonConfig.paramStartIdx = 0;
    gCommonConfig.paramStopIdx = paramsetIdx;
    gCommonConfig.numLoops = 1; //do only one iteration

    errCode = HWA_configCommon(handle, &gCommonConfig);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configCommon returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_enableDoneInterrupt() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* Step 5 - Enable the HWA */
    startTime = CycleCounterP_getCount32();
    errCode = HWA_enable(handle, 1);
    endTime = CycleCounterP_getCount32();
    drvApiCycles[DRV_API_ENABLE] = endTime - startTime;
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enable(1) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_enable() to enable HWA FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* Step 6 - Reset the HWA state machine */
    errCode = HWA_reset(handle);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_reset returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_reset( ) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* Prepare the input data for test vector based testing */
    /* memcopy the data to MEM0 */
    memcpy(srcAddr, (uint8_t *)gHWATest_1DFFT_input,
        HWA_TEST_NUM_SAMPLES*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    /* m1 fft result */
    memset((uint8_t *)dstAddr, 0, HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    /* m2 fft result */
    memset((uint8_t *)((uint32_t)dstAddr + HWA_MEMn_SIZE), 0, HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    /* m3 fft result all zero*/
    memset((uint8_t *)((uint32_t)dstAddr + 2 * HWA_MEMn_SIZE), 0xffffffff, HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    /* m3 fft with shuffle address*/
    memset((uint8_t *)((uint32_t)dstAddr + 3 * HWA_MEMn_SIZE), 0, HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);

    /* Step 7: trigger HWA */
    /* trigger the HWA using HWA_setDMA2ACCManualTrig if triggerMode is set to DMA */
    startTime = CycleCounterP_getCount32();
    errCode = HWA_setDMA2ACCManualTrig(handle, HWA_TEST_SRC_TRIGGER_DMACH0);
    endTime = CycleCounterP_getCount32();
    drvApiCycles[DRV_API_DMAMANUALTRIG] = endTime - startTime;
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_setDMA2ACCManualTrig(0) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_setDMA2ACCManualTrig(ch:0) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    errCode = HWA_setDMA2ACCManualTrig(handle, HWA_TEST_SRC_TRIGGER_DMACH1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_setDMA2ACCManualTrig(1) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_setDMA2ACCManualTrig(ch:1) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* Step 8: Wait for completion */

    /* now wait for the interrupts */
#ifdef TEST_PARAMSET_INTERRUPT
    /* first wait for the paramSet done interrupt */
    status = SemaphoreP_pend(&paramSetSem, SystemP_WAIT_FOREVER);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Error: SemaphoreP_pend returned %d\r\n", status);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }
#endif
    /* then wait for the all paramSets done interrupt */
    status = SemaphoreP_pend(&doneSem, SystemP_WAIT_FOREVER);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Error: Wait for all paramsets done failed %d\r\n", status);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        return;
    }

    /* check the DC and interference statistics result */
    errCode = HWA_readDCEstimateReg(handle, DCEstResult, 0, HWA_TEST_NUM_RX_ANT);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_readDCEstimateReg returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_readDCEstimateReg() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* mag results */
    errCode = HWA_readInterfThreshReg(handle, magEstResult, 0, HWA_TEST_NUM_RX_ANT, HWA_INTERFERENCE_THRESHOLD_TYPE_MAG);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_readInterfThreshReg(MAG) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_readInterfThreshReg(MAG) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* magdiff threshold reg check */
    errCode = HWA_readInterfThreshReg(handle, magDiffEstResult, 0, HWA_TEST_NUM_RX_ANT, HWA_INTERFERENCE_THRESHOLD_TYPE_MAGDIFF);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_readInterfThreshReg(MAGDIFF) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_readInterfThreshReg(MAGDIFF) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    errCode = HWA_readDCAccReg(handle, DCAccResult, 0, HWA_TEST_NUM_RX_ANT);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_readDCAccReg returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_readDCAccReg() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    errCode = HWA_readIntfAccReg(handle, magAccResult, HWA_INTERFERENCE_THRESHOLD_TYPE_MAG, 0, HWA_TEST_NUM_RX_ANT);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_readIntfAccReg(MAG) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_readIntfAccReg(MAG) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    errCode = HWA_readIntfAccReg(handle, magDiffAccResult, HWA_INTERFERENCE_THRESHOLD_TYPE_MAGDIFF, 0, HWA_TEST_NUM_RX_ANT);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_readIntfAccReg(MAGDIFF) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_readIntfAccReg(MAGDIFF) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    DebugP_log("\r\nDebug: HWA FFT with pre-processing test result check\r\n");
    DebugP_log(" \r\n");

    /* check the results */
    compareCode2 = 0;
    compareCode1 = memcmp((uint8_t *)DCEstResult, (uint8_t*)gHWATest_dcest_output, 2 * HWA_TEST_NUM_RX_ANT * 4);
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA DC ESTIMATION found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA DC ESTIMATION found correct\r\n");
    }
    compareCode2 += compareCode1;

    compareCode1 = memcmp((uint8_t *)magEstResult, (uint8_t*)gHWATest_intefMag_output, HWA_TEST_NUM_RX_ANT * 4);
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA Interference Statistics Magnitude found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA Interference Statistics Magnitude found correct\r\n");
    }
    compareCode2 += compareCode1;

    compareCode1 = memcmp((uint8_t *)magDiffEstResult, (uint8_t*)gHWATest_intefMagDiff_output, HWA_TEST_NUM_RX_ANT * 4);
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA Interference Statistics Magnitude DIFF found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA Interference Statistics Magnitude DIFF found correct\r\n");
    }
    compareCode2 += compareCode1;

    compareCode1 = memcmp((uint8_t *)DCAccResult, (uint8_t*)HWA_dcAccumulator_output, 2 * HWA_TEST_NUM_RX_ANT * 8);
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA DC Accumulator found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA DC Accumulator found correct\r\n");
    }
    compareCode2 += compareCode1;

    compareCode1 = memcmp((uint8_t *)magAccResult, (uint8_t*)HWA_magAccumulator_output, HWA_TEST_NUM_RX_ANT * 8);
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA Interference Mag Accumulator found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA Interference Mag Accumulator found correct\r\n");
    }

    compareCode2 += compareCode1;

    compareCode1 = memcmp((uint8_t *)magDiffAccResult, (uint8_t*)HWA_magDiffAccumulator_output, HWA_TEST_NUM_RX_ANT * 8);
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA  Interference Magdiff Accumulator found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA  Interference Magdiff Accumulator found correct\r\n");
    }
    compareCode2 += compareCode1;

    /* Check output data */
    /* output data is ready - compare, PING fft + DC/interference  */
    compareCode1 = memcmp(dstAddr, (uint8_t *)gHWATest_1DFFT_output,
        HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    compareCode2 += compareCode1;
    if(compareCode1 != 0)
    {
        DebugP_log("Error: fft output produced by HWA in MEM1 found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: fft output produced by HWA (paramset 1) found correct\r\n");
    }

    /* pong fft + interference all pass through*/
    compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr + HWA_MEMn_SIZE), (uint8_t *)gHWATest_1DFFT_output,
        HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    compareCode2 += compareCode1;
    if(compareCode1 != 0)
    {
        DebugP_log("Error: fft output produced by HWA in MEM2 found incorrect: error %d\r\n", compareCode2);
    }
    else
    {
        DebugP_log("Debug: fft output produced by HWA (paramset 3) found correct\r\n");
    }

     /* fft interference all zero-out */
    if((*((uint8_t *)((uint32_t)dstAddr + HWA_MEMn_SIZE * 2))) == 0) //first one is 0
    {
        //the rest are all 0
        compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr + HWA_MEMn_SIZE * 2), (uint8_t *)((uint32_t)dstAddr + HWA_MEMn_SIZE * 2 + 1),
            HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE - 1);
    }
    else
    {
        compareCode1 = 1;
    }
    if(compareCode1 != 0)
    {
        DebugP_log("Error: fft output produced by HWA in MEM3 found incorrect: error %d\r\n", compareCode2);
    }
    else
    {
        DebugP_log("Debug: fft output produced by HWA (paramset 4) found correct\r\n");
    }
    compareCode2 += compareCode1;

    /* fft + shuffle addressing output */
    compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr + HWA_MEMn_SIZE * 3), (uint8_t *)gHWATest_1DFFT_output,
        HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    compareCode2 += compareCode1;
    if(compareCode1 != 0)
    {
        DebugP_log("Error: fft output produced by HWA in MEM4 found incorrect: error %d\r\n", compareCode2);
    }
    else
    {
        DebugP_log("Debug: fft output produced by HWA (paramset 5) found correct\r\n");
    }

    /* Cleanup */
    errCode = HWA_disableParamSetInterrupt(handle, paramsetIdx, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_disableParamSetInterrupt returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_readIntfAccReg(MAGDIFF) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_disableDoneInterrupt(handle, 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_disableDoneInterrupt returned %d\r\n", errCode);
        DebugP_log("Feature :  API HWA_disableDoneInterrupt() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    /* disable HWA */
    errCode = HWA_enable(handle, 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enable(0) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_enable() to disable HWA FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

#ifdef TEST_PARAMSET_INTERRUPT
    /* cleanup semaphores */
    SemaphoreP_destruct(&paramSetSem);
#endif
    SemaphoreP_destruct(&doneSem);

    DebugP_log(" \r\n");

    /* final FFT test results*/
    if(compareCode2 != 0)
    {
        DebugP_log("Feature : HWA FFT with pre-processing Tests FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode2);
    }
    else
    {
        DebugP_log("Feature : HWA FFT with pre-processing Tests PASS\r\n");
    }

    DebugP_log("\r\nHWA FFT with pre-processing test completed \r\n");
    return;
}

static void test_complexmultiply(void *args)
{
    HWA_Handle              handle = gHwaHandle;
    int32_t                 errCode, compareCode1, compareCode2;
    uint8_t                 paramsetIdx = 0;
    uint8_t                *srcAddr = (uint8_t*)SOC_HWA_MEM0;
    uint8_t                *dstAddr = (uint8_t*)SOC_HWA_MEM1;
    uint16_t                startParamIdx = 31;
    uint32_t                ii;
    uint8_t                 paramsetCheckArray[16];
    uint8_t                 numCheckParamSets;

    errCode = HWA_reset(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    DebugP_log("\n------- HWA complex multiply Tests -------\r\n\r\n");

    paramsetIdx = 0;
    memset(gHWATestParamConfig, 0, sizeof(gHWATestParamConfig));  //init to zero
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

     /* this paramset disable the FFT, vector multiplication mode 1 using data from internal ram */
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(srcAddr); //(uint32_t)((uint32_t)srcAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = HWA_TEST_NUM_SAMPLES - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = HWA_TEST_NUM_RX_ANT - 1;
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 8;

    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(dstAddr); //(uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = HWA_TEST_NUM_SAMPLES - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_DISABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSize = 0;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowEn = 0; //disable
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowStart = 0; //start of window RAM
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.winSymm = 0;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.vecMultiMode1RamAddrOffset = 5;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.cmultScaleEn = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)((uint32_t)dstAddr  - SOC_HWA_MEM0); // address is relative to start of MEM0
    errCode = HWA_configParamSet(handle, paramsetIdx + startParamIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", paramsetIdx + startParamIdx, errCode);
        DebugP_log("Feature : API HWA_configParamSet() (Immediate Triggered, AccelMode FFT, complex multiply: vector multiplication) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d :Immediate Triggered, AccelMode FFT, complex multiply: vector multiplication\r\n", paramsetIdx + startParamIdx);
    }


    /* this paramset disable the FFT, scale multiply set to magnitude squared mode */
    paramsetIdx++;
    gHWATestParamConfig[paramsetIdx] = gHWATestParamConfig[0];
    gHWATestParamConfig[paramsetIdx].source.srcScale = 0; //left shift 8 bits from 16 bits to 24 bits
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 8;   //no shift from 24 bits to 32 bits

    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED;


    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_MAG_SQUARED;

     /* mem2 */
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA ( (uint32_t) dstAddr + 0x4000);
    errCode = HWA_configParamSet(handle, paramsetIdx + startParamIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", errCode, paramsetIdx + startParamIdx);
        DebugP_log("Feature : API HWA_configParamSet() (Immediate Triggered, magnitude mode with sum statistics output) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d :Immediate Triggered, AccelMode FFT, magnitude mode with sum statistics output\r\n", paramsetIdx + startParamIdx);
    }

    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG;
    gCommonConfig.paramStartIdx = 0 + startParamIdx;
    gCommonConfig.paramStopIdx = paramsetIdx + startParamIdx;
    gCommonConfig.numLoops = 1; //do only one iteration

    errCode = HWA_configCommon(handle, &gCommonConfig);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configCommon returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* config the vector multiply internal ram */
    for(ii = 0; ii < HWA_TEST_NUM_SAMPLES; ii++)
    {
        vectorArray[2 * ii] = 0xfffff; //Q20 1
        vectorArray[2 * ii + 1] = 0x0; //imag
    }

    errCode = HWA_configRam(handle, HWA_RAM_TYPE_VECTORMULTIPLY_RAM, (uint8_t *)vectorArray,
                  HWA_TEST_NUM_SAMPLES * 8,  //complex coefficients, with 21 bits I and 21 bits Q
                  gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.vecMultiMode1RamAddrOffset * 8);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configRam returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_configRam(HWA_RAM_TYPE_VECTORMULTIPLY_RAM) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* Prepare the input data for test vector based testing */
    /* memcopy the data to MEM0 */
    memcpy(srcAddr, (uint8_t *)gHWATest_1DFFT_input,
        HWA_TEST_NUM_SAMPLES*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    /* m1 vector multiply results */
    memset((uint8_t *)dstAddr, 0, HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    /* m2 vector multiply results */
    memset((uint8_t *)((uint32_t) dstAddr + HWA_MEMn_SIZE), 0, HWA_TEST_NUM_RX_ANT*4*2); //for magquare, 2 words

    /* Step 5 - Enable the HWA */
    errCode = HWA_enable(handle, 1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enable(1) returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* wait for hwa to finish */
    numCheckParamSets = 2;
    paramsetCheckArray[0] = startParamIdx + 0;
    paramsetCheckArray[1] = startParamIdx + 1;
    errCode = HWA_paramSetDonePolling(handle, numCheckParamSets, paramsetCheckArray);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_paramSetDonePolling() returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_paramSetDonePolling() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    DebugP_log("\r\nDebug: HWA complex multiply test result check \r\n");
    DebugP_log(" \r\n");

    /* statistics block output check, I_SUM1_LSB */
    errCode = HWA_readStatsReg(handle, statsResults, 4);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_readStatsReg() returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_readStatsReg() FAIL \r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* check the vector multiplication results */
    compareCode2 = 0;
    compareCode1 = memcmp((uint8_t *)gHWATest_1DFFT_input, (uint8_t*)((uint32_t)dstAddr),
        HWA_TEST_NUM_SAMPLES*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    if(compareCode1 != 0)
    {
        DebugP_log("Error: output produced by HWA in MEM1 found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: vector multiply output produced by HWA (paramset 31) found correct\r\n");
    }
    compareCode2 += compareCode1;

    compareCode1 = 0;
    for(ii = 0; ii < HWA_TEST_NUM_RX_ANT; ii++)
    {
        if((gHWATest_magSquareSum_output[2 * ii + 1] != statsResults[3 - ii].iSumLSB) || (0 != statsResults[3 - ii].iSumMSB))
        {
            compareCode1++;
        }
    }

    if(compareCode1 != 0)
    {
        DebugP_log("Error: magnitude square sum outputs in common registers produced by HWA : error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: magnitude square sum outputs in common registers produced by HWA (paramset 32) found correct\r\n");
    }
    compareCode2 += compareCode1;

    compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr + HWA_MEMn_SIZE), (uint8_t *)gHWATest_magSquareSum_output, sizeof(uint32_t) * 2 * HWA_TEST_NUM_RX_ANT);
    if(compareCode1 != 0)
    {
        DebugP_log("Error: magnitude square sum outputs in memory produced by HWA : error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: magnitude square sum outputs in data memory produced by HWA (paramset 32) found correct\r\n");
    }
    compareCode2 += compareCode1;

    /* disable HWA */
    errCode = HWA_enable(handle, 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enable(0) returned %d\r\n", errCode);
        return;
    }

    DebugP_log(" \r\n");
    /* final results */
    if(compareCode2 != 0)
    {
        DebugP_log("Feature : Immediate triggered complex multiply tests FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode2);
    }
    else
    {
        DebugP_log("Feature : Immediate triggered complex multiply tests PASS\r\n");
    }
    DebugP_log("\r\nHWA complex multiple test completed\r\n");

    return;
}

static void test_azimfft(void *args)
{
    HWA_Handle              handle = gHwaHandle;
#if defined (SOC_AWR294X)
    HWA_Object              *ptrHWADriver = (HWA_Object*)gHwaHandle;
#endif
    int32_t                 errCode, compareCode1, compareCode2;
    uint8_t                 paramsetIdx = 0;
    uint8_t                *srcAddr = (uint8_t*)SOC_HWA_MEM0;
    uint8_t                *dstAddr = (uint8_t*)SOC_HWA_MEM1;
    uint8_t                 paramsetCheckArray[16];
    uint8_t                 numCheckParamSets;
    uint8_t                 numIterations = 2;
    uint8_t                 numInputSymbols = 12;
    uint8_t                 fftSize = 16;
    uint32_t                ii, jj;
    uint8_t                 cdfCntBitNum[16];
    uint16_t                cdfCntCdfValue[16];
    uint16_t                cdfCntHistValue[16];
    uint32_t                ramMemAddress;
    volatile uint32_t       startTime;
    volatile uint32_t       endTime;

    errCode = HWA_reset(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    DebugP_log("\r\n------- HWA Azim FFT calculation test ------- \r\n\r\n");

    paramsetIdx = 0;
    memset(gHWATestParamConfig, 0, sizeof(gHWATestParamConfig));  //init to zero
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    /* add complex multiply with HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT mode, with 32 bits complex input */
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(srcAddr);

    gHWATestParamConfig[paramsetIdx].source.srcAcnt = numInputSymbols - 1;
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_COMPLEX_32BIT_SIZE; //32 bits complex
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = numIterations - 1;
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = HWA_TEST_COMPLEX_32BIT_SIZE * HWA_TEST_NUM_RX_ANT;
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_32BIT; //32-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 0;
    gHWATestParamConfig[paramsetIdx].source.shuffleMode = HWA_SRC_SHUFFLE_AB_MODE_ADIM;  //A dimension shuffle
    gHWATestParamConfig[paramsetIdx].source.wrapComb = 0;

    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = fftSize - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = HWA_TEST_COMPLEX_32BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = HWA_TEST_COMPLEX_32BIT_SIZE * fftSize; //should be dont care
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 8;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSize = log2Approx(fftSize);
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x0; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_DISABLE; //disable

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.bpmEnable = HWA_FEATURE_BIT_DISABLE;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples

    /* channel combine */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.chanCombEn = HWA_FEATURE_BIT_ENABLE;
    /* insert a zero */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.zeroInsertEn = HWA_FEATURE_BIT_ENABLE;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.cmultScaleEn = HWA_FEATURE_BIT_ENABLE;

    /* mem1 */
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(dstAddr);
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() (Software Triggered, AccelMode FFT, with channel combine, and zero insert) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d Software Triggered, AccelMode FFT, with channel combine, and zero insert\r\n", paramsetIdx);
    }

    /* configure shuffle index */
    shuffleIdx[0] = 0;
    shuffleIdx[1] = 1;
    shuffleIdx[2] = 2;
    shuffleIdx[3] = 4 * 8;
    shuffleIdx[4] = 4 * 8 + 1;
    shuffleIdx[5] = 3;
    shuffleIdx[6] = 4 * 8 + 2;
    shuffleIdx[7] = 4 * 8 + 3;
    shuffleIdx[8] = 4 * 16;
    shuffleIdx[9] = 4 * 16 + 1;
    shuffleIdx[10] = 4 * 16 + 2;
    shuffleIdx[11] = 4 * 16 + 3;
    errCode = HWA_configRam(handle, HWA_RAM_TYPE_SHUFFLE_RAM, (uint8_t*)shuffleIdx, numInputSymbols * sizeof(uint16_t), 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configRam(HWA_RAM_TYPE_SHUFFLE_RAM) returned %d\r\n", errCode);
        DebugP_log("Feature: API HWA_configRam(HWA_RAM_TYPE_SHUFFLE_RAM) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* random pick some input data */
    for(jj = 0; jj < numIterations; jj++)
    {
        for(ii = 0; ii < numInputSymbols; ii++)
        {
            fft_input_32bits[2 * shuffleIdx[ii] + jj * HWA_TEST_NUM_RX_ANT * 2] = (int32_t)((int16_t)gHWATest_1DFFT_input[2 * ii]);       //imag
            fft_input_32bits[2 * shuffleIdx[ii] + 1 + jj * HWA_TEST_NUM_RX_ANT * 2] = (int32_t)((int16_t)gHWATest_1DFFT_input[2 * ii + 1]);   //real
        }
    }

    /* copy input data */
    memcpy(srcAddr, (uint8_t *)fft_input_32bits, sizeof(fft_input_32bits));

    /* second histogram calculation */
    paramsetIdx++;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    //M2
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA((uint32_t) dstAddr + 0x4000);

    gHWATestParamConfig[paramsetIdx].source.srcAcnt = 16 - 1;
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = 4;
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = 256 - 1;
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = 16 * 4;
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //real data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //un-signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 8; //0

    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = 16 - 1;
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = 2;
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = 16 * 2;
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED;
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_DISABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn =  HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

    /* enable the 2D maximum */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.max2Denable = HWA_FEATURE_BIT_ENABLE;

    /* histogram with cdf threshold */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramMode = HWA_HISTOGRAM_MODE_CDF_THRESHOLD;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramScaleSelect = 11;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramSizeSelect = 6;
    /* M3*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA((uint32_t) dstAddr + 2 * 0x4000); //(uint32_t)((uint32_t)dstAddr + 2 * HWA_MEMn_SIZE - SOC_HWA_MEM0);

    /* memcopy the data to MEM2 */
    memcpy((uint8_t *)((uint32_t)dstAddr + HWA_MEMn_SIZE), (uint8_t *)gHWATest_1DFFT_input, 256 * 16 * 4);
    /* clear M3*/
    memset((uint8_t *)((uint32_t)dstAddr + 2 * HWA_MEMn_SIZE), 0, 256 * 16 * 4);

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() (Immediate Triggered,AccelMode FFT with CDF calculation, and 2D maximum enabled) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d Immediate Triggered,AccelMode FFT with CDF calculation, and 2D maximum enabled\r\n", paramsetIdx);
    }

    /* configure common register */
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG |
        HWA_COMMONCONFIG_MASK_CHANCOMB_VEC_SIZE |
        HWA_COMMONCONFIG_MASK_ZEROINSERT_NUM_MASK |
        HWA_COMMONCONFIG_MASK_COMPLEXMULT_SCALEARRAY |
        HWA_COMMONCONFIG_MASK_CDFCNT_THRESHOLD |
        HWA_COMMONCONFIG_MASK_MAX2D_OFFSETBOTHDIM |
        HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
        HWA_COMMONCONFIG_MASK_LFSRSEED;

    gCommonConfig.fftConfig.twidDitherEnable = 1;
    gCommonConfig.fftConfig.lfsrSeed = 0x1234567;

    /* 2D maximum value offset, visually check the values written into the register */
    gCommonConfig.advStatConfig.max2DoffsetDim1 = -1;
    gCommonConfig.advStatConfig.max2DoffsetDim2 = 1;

    /* histogram calculation */
    gCommonConfig.advStatConfig.cdfCntThresh = 110;

    /* channel combine */
    gCommonConfig.chanCombConfig.size = 9;
    gCommonConfig.chanCombConfig.vector[0] = 0xA6D;
    gCommonConfig.chanCombConfig.vector[1] = 0x0;
    gCommonConfig.chanCombConfig.vector[2] = 0x0;
    gCommonConfig.chanCombConfig.vector[3] = 0x0;
    gCommonConfig.chanCombConfig.vector[4] = 0x0;
    gCommonConfig.chanCombConfig.vector[5] = 0x0;
    gCommonConfig.chanCombConfig.vector[6] = 0x0;
    gCommonConfig.chanCombConfig.vector[7] = 0x0;

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        /* Initialize LUT RAM for Zero insertion in AWR294x ES2.0 device only. */
        uint32_t arr_zeroInsermask[8] = {0x5DF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
        HWA_configRam(handle, HWA_RAM_TYPE_LUT_FREQ_DEROTATE_RAM, (uint8_t *)arr_zeroInsermask, 32, 0);
    }
    else
    {
        /* zero insert, at the end */
        gCommonConfig.zeroInsertConfig.mask[0] = 0x5DF;
        gCommonConfig.zeroInsertConfig.mask[1] = 0x0;
        gCommonConfig.zeroInsertConfig.mask[2] = 0x0;
        gCommonConfig.zeroInsertConfig.mask[3] = 0x0;
        gCommonConfig.zeroInsertConfig.mask[4] = 0x0;
        gCommonConfig.zeroInsertConfig.mask[5] = 0x0;
        gCommonConfig.zeroInsertConfig.mask[6] = 0x0;
        gCommonConfig.zeroInsertConfig.mask[7] = 0x0;
    }
#else
    /* zero insert, at the end */
    gCommonConfig.zeroInsertConfig.mask[0] = 0x5DF;
    gCommonConfig.zeroInsertConfig.mask[1] = 0x0;
    gCommonConfig.zeroInsertConfig.mask[2] = 0x0;
    gCommonConfig.zeroInsertConfig.mask[3] = 0x0;
    gCommonConfig.zeroInsertConfig.mask[4] = 0x0;
    gCommonConfig.zeroInsertConfig.mask[5] = 0x0;
    gCommonConfig.zeroInsertConfig.mask[6] = 0x0;
    gCommonConfig.zeroInsertConfig.mask[7] = 0x0;
#endif

    gCommonConfig.zeroInsertConfig.number = 2;

    /* vector multiplication vector */
    for(ii = 0; ii < 12; ii++)
    {
        gCommonConfig.complexMultiplyConfig.Iscale[ii] = 0xFFFFF; //Q20 format
        gCommonConfig.complexMultiplyConfig.Qscale[ii] = 0;
    }

    gCommonConfig.paramStartIdx = 0;
    gCommonConfig.paramStopIdx = paramsetIdx;
    gCommonConfig.numLoops = 1;

    errCode = HWA_configCommon(handle, &gCommonConfig);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configCommon returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* clear the histgoram */
    errCode = HWA_initializeRAM(handle, (uint32_t)(HWA_APP_MEMINIT_HIST_EVEN_RAM | HWA_APP_MEMINIT_HIST_ODD_RAM));
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_clearHistgoramRAM(1) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_clearHistgoramRAM() to clear Histogram RAM FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* Step 5 - Enable the HWA */
    errCode = HWA_enable(handle, 1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enable(1) returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* trigger the HWA manually using HWA_setSoftwareTrigger if triggerMode is set to Software */
    startTime = CycleCounterP_getCount32();
    errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);  //through DSP software trigger
    endTime = CycleCounterP_getCount32();
    drvApiCycles[DRV_API_SOFTWARETRIG] = endTime - startTime;
    if(errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        DebugP_log("Error: HWA_setSoftwareTrigger returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_setSoftwareTrigger() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* wait for hwa to finish */
    numCheckParamSets = 2;
    paramsetCheckArray[0] = 0;  //azim fft
    paramsetCheckArray[1] = 1; //histogram
    errCode = HWA_paramSetDonePolling(handle, numCheckParamSets, paramsetCheckArray);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_paramSetDonePolling() returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_paramSetDonePolling() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    DebugP_log("\r\nDebug:  HWA Azim FFT test result check\r\n");
    DebugP_log(" \r\n");
    /* compare the results */
    compareCode2 = 0;
    for(ii = 0; ii < numIterations; ii++)
    {
        compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr + ii * HWA_TEST_COMPLEX_32BIT_SIZE * fftSize), (uint8_t*)gHWATest_azimfft_output, HWA_TEST_COMPLEX_32BIT_SIZE * fftSize);
        if(compareCode1 != 0)
        {
            DebugP_log("Error: output produced by HWA Azim %d Iteration found incorrect: error %d\r\n", ii, compareCode1);
        }
        else
        {
            DebugP_log("Debug: HWA Azim FFT %d Iteration found correct (paramset 0)\r\n", ii);
        }
        compareCode2 += compareCode1;
    }
    ramMemAddress = HWA_getRamAddress(HWA_RAM_TYPE_HISTOGRAM_RAM);

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t*)gHWATest_histogram_output_2p1, 64 * 16);
    }
    else
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t*)gHWATest_histogram_output, 64 * 16);
    }
#else
    compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t*)gHWATest_histogram_output, 64 * 16);
#endif

    if(compareCode1 != 0)
    {
        DebugP_log("Error: output produced by HWA Histogram found incorrect: error %d\r\n", compareCode1);

    }
    else
    {
        DebugP_log("Debug: output produced by HWA Histogram found correct (paramset 1)\r\n");

    }
    compareCode2 += compareCode1;

    /* check the cdf threshold value*/
    errCode = HWA_readHistThresholdRam(handle, cdfThresholdResults, 16, 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_readHistThresholdRam() returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_readHistThresholdRam() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /*verify the structure definition*/
    for(ii = 0; ii < 16; ii++)
    {
        cdfCntBitNum[ii] = cdfThresholdResults[ii].binNumber;
        cdfCntCdfValue[ii] = cdfThresholdResults[ii].cdfValue;
        cdfCntHistValue[ii] = cdfThresholdResults[ii].pdfValue;
    }

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        compareCode1 = memcmp((uint8_t*)cdfCntBitNum, (uint8_t*)HWA_cdfBitNum_output_2p1, 16);
    }
    else
    {
        compareCode1 = memcmp((uint8_t*)cdfCntBitNum, (uint8_t*)HWA_cdfBitNum_output, 16);
    }
#else
    compareCode1 = memcmp((uint8_t*)cdfCntBitNum, (uint8_t*)HWA_cdfBitNum_output, 16);
#endif

    if(compareCode1 != 0)
    {
        DebugP_log("Error: CDF threshold bit number returned %d\r\n", compareCode1);
        DebugP_log("Feature: CDF threshold bit number FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode1);
        return;
    }
    else
    {
        DebugP_log("Debug: CDF threshold bit number output by HWA found correct (paramset 1)\r\n");
    }
    compareCode2 += compareCode1;

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        compareCode1 = memcmp((uint8_t*)cdfCntCdfValue, (uint8_t*)HWA_cdfCdfValue_output_2p1, 16 * 2);
    }
    else
    {
        compareCode1 = memcmp((uint8_t*)cdfCntCdfValue, (uint8_t*)HWA_cdfCdfValue_output, 16 * 2);
    }
#else
    compareCode1 = memcmp((uint8_t*)cdfCntCdfValue, (uint8_t*)HWA_cdfCdfValue_output, 16 * 2);
#endif

    if(compareCode1 != 0)
    {
        DebugP_log("Error: CDF threshold CDF value returned %d\r\n", compareCode1);
        DebugP_log("Feature : CDF threshold CDF value FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode1);
    }
    else
    {
        DebugP_log("Debug: CDF threshold CDF value output by HWA found correct (paramset 1)\r\n");
    }
    compareCode2 += compareCode1;

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        compareCode1 = memcmp((uint8_t*)cdfCntHistValue, (uint8_t*)HWA_cdfPdfValue_output_2p1, 16 * 2);
    }
    else
    {
        compareCode1 = memcmp((uint8_t*)cdfCntHistValue, (uint8_t*)HWA_cdfPdfValue_output, 16 * 2);
    }
#else
    compareCode1 = memcmp((uint8_t*)cdfCntHistValue, (uint8_t*)HWA_cdfPdfValue_output, 16 * 2);
#endif

    if(compareCode1 != 0)
    {
        DebugP_log("Error: CDF threshold PDF value returned %d\r\n", compareCode1);
        DebugP_log("Feature : CDF threshold PDF value FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode1);
        return;
    }
    else
    {
        DebugP_log("Debug: CDF threshold PDF value output by HWA found correct (paramset 1)\r\n");
    }
    compareCode2 += compareCode1;

    ramMemAddress = HWA_getRamAddress(HWA_RAM_TYPE_2DSTAT_ITER_IDX);

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim1_maxIdx_output_2p1, sizeof(uint16_t) * 256);
    }
    else
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim1_maxIdx_output, sizeof(uint16_t) * 256);
    }
#else
    compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim1_maxIdx_output, sizeof(uint16_t) * 256);
#endif

    if(compareCode1 != 0)
    {
        DebugP_log("Error: 2D maximum Iteration max location idx returned %d\r\n", compareCode1);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode1);
    }
    else
    {
        DebugP_log("Debug: 2D maximum Iteration max location idx output by HWA found correct (paramset 1)\r\n");
    }
    compareCode2 += compareCode1;

    ramMemAddress = HWA_getRamAddress(HWA_RAM_TYPE_2DSTAT_ITER_VAL);

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim1_maxValue_output_2p1, sizeof(uint32_t) * 256);
    }
    else
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim1_maxValue_output, sizeof(uint32_t) * 256);
    }
#else
    compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim1_maxValue_output, sizeof(uint32_t) * 256);
#endif

    if(compareCode1 != 0)
    {
        DebugP_log("Error: 2D maximum Iteration max value returned %d\r\n", compareCode1);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode1);
    }
    else
    {
        DebugP_log("Debug: 2D maximum Iteration max value output by HWA found correct (paramset 1)\r\n");
    }
    compareCode2 += compareCode1;

    ramMemAddress = HWA_getRamAddress(HWA_RAM_TYPE_2DSTAT_SAMPLE_IDX);

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim2_maxIdx_output_2p1, sizeof(uint16_t) * 16);
    }
    else
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim2_maxIdx_output, sizeof(uint16_t) * 16);
    }
#else
    compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim2_maxIdx_output, sizeof(uint16_t) * 16);
#endif

    if(compareCode1 != 0)
    {
        DebugP_log("Error: 2D maximum sample max location idx returned %d\r\n", compareCode1);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode1);
    }
    else
    {
        DebugP_log("Debug: 2D maximum sample max location idx value output by HWA found correct (paramset 1)\r\n");
    }
    compareCode2 += compareCode1;

    ramMemAddress = HWA_getRamAddress(HWA_RAM_TYPE_2DSTAT_SAMPLE_VAL);

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim2_maxValue_output_2p1, sizeof(uint32_t) * 16);
    }
    else
    {
        compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim2_maxValue_output, sizeof(uint32_t) * 16);
    }
#else
    compareCode1 = memcmp((uint8_t *)ramMemAddress, (uint8_t *)HWA_2Dmax_dim2_maxValue_output, sizeof(uint32_t) * 16);
#endif

    if(compareCode1 != 0)
    {
        DebugP_log("Error: 2D maximum sample max Value returned %d\r\n", compareCode1);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode1);
    }
    else
    {
        DebugP_log("Debug: 2D maximum sample max value output by HWA found correct (paramset 1)\r\n");
    }
    compareCode2 += compareCode1;

    /* disable HWA */
    errCode = HWA_enable(handle, 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enable(0) returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    DebugP_log(" \r\n");
    /* final results */
    if(compareCode2 != 0)
    {
        DebugP_log("Feature : Software triggered azim FFT test FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode2);
    }
    else
    {
        DebugP_log("Feature : Software triggered azim FFT test PASS\r\n");
    }

    DebugP_log("\r\nHWA azim FFT test completed\r\n");

    return;
}

static void test_histogram(void *args)
{
    HWA_Handle              handle = gHwaHandle;
    int32_t                 errCode, compareCode1, compareCode2;
    uint8_t                 paramsetIdx = 0;
    uint8_t                *srcAddr = (uint8_t*)SOC_HWA_MEM0;
    uint8_t                *dstAddr = (uint8_t*)SOC_HWA_MEM1;

    errCode = HWA_reset(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("\r\n------- HWA Histogram test ------- \r\n\r\n");

    paramsetIdx = 0;
    memset(gHWATestParamConfig, 0, sizeof(gHWATestParamConfig));  //init to zero
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    /* histogram calculation */
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA (srcAddr);

    gHWATestParamConfig[paramsetIdx].source.srcAcnt = 5 - 1;
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = 2;
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = 256 - 1;
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = 16 * 2;
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL; //real data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED; //un-signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 8; //0

    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = 5 - 1;
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = 2;
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = 16 * 2;
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED;
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_DISABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED; //disabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

    /* enable the 2D maximum */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.max2Denable = HWA_FEATURE_BIT_ENABLE;

    /* histogram with cdf threshold */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramMode = HWA_HISTOGRAM_MODE_CDF_THRESHOLD;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramScaleSelect = 11;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.histogramSizeSelect = 6;

    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA (dstAddr);

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() (Immediate Triggered, AccelMode FFT, Histogram calculation) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d Immediate Triggered, AccelMode FFT, Histogram calculation\r\n", paramsetIdx);
    }

    /* configure common register */
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG | HWA_COMMONCONFIG_MASK_CDFCNT_THRESHOLD;
    gCommonConfig.advStatConfig.cdfCntThresh = 140;

    gCommonConfig.paramStartIdx = 0;
    gCommonConfig.paramStopIdx = paramsetIdx;
    gCommonConfig.numLoops = 1;

    errCode = HWA_configCommon(handle, &gCommonConfig);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configCommon returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* memcopy the data to MEM0 */
    memcpy(srcAddr, (uint8_t *)gHWATest_1DFFT_input,  256*16*2 );
    memset((uint8_t *)dstAddr, 0, 256 * 16 * 2);

    /* clear the histgoram */
    errCode = HWA_initializeRAM(handle, (uint32_t)(HWA_APP_MEMINIT_HIST_EVEN_RAM | HWA_APP_MEMINIT_HIST_ODD_RAM));
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_clearHistgoramRAM(1) returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* Step 5 - Enable the HWA */
    errCode = HWA_enable(handle, 1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enable(1) returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* trigger the HWA manually using HWA_setSoftwareTrigger if triggerMode is set to Software */
    errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);  //through DSP software trigger
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_setSoftwareTrigger returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* wait for hwa to finish */
    errCode = HWA_singleParamSetDonePolling(handle, paramsetIdx);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_paramSetDonePolling() returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* HWA stuck for this parameters, all the functionality result check are done in HWA_azimfft_test
     * so for this test, not check the output, just check if hwa can run to finish
     */
    compareCode2 = 0;
    compareCode1 = 0;

    compareCode2 += compareCode1;

    DebugP_log("\r\nHWA run to complete, paramdone is generated \r\n");

    /* disable HWA */
    errCode = HWA_enable(handle, 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enable(0) returned %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    DebugP_log(" \r\n");
    /* final results */
    if(compareCode2 != 0)
    {
        DebugP_log("Feature : Software triggered Histogram test FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode2);
    }
    else
    {
        DebugP_log("Feature : Software triggered Histogram test PASS\r\n");
    }

    DebugP_log("\r\nHWA Histogram test completed\r\n");
    return;
}

static void test_compress(void *args)
{
    HWA_Handle              handle = gHwaHandle;
#if defined (SOC_AWR294X)
    HWA_Object              *ptrHWADriver = (HWA_Object *)gHwaHandle;
#endif
    int32_t                 errCode, compareCode1, compareCode2;
    uint8_t                 paramsetIdx = 0;
    uint8_t                *srcAddr = (uint8_t*)SOC_HWA_MEM0;
    uint8_t                *dstAddr = (uint8_t*)SOC_HWA_MEM1;
    uint16_t                numBFPInputBlock = 128;
    uint8_t                 numEGEInputData = 128;
    uint8_t                *dstAddr2 = (uint8_t*)SOC_HWA_MEM2;
    uint8_t                 numEGEBlockSize = 16; //number of complex data
    uint8_t                 numEGECompressedWord;

    errCode = HWA_reset(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    compareCode2 = 0;
    DebugP_log("\r\n\r\n------- HWA compress/decompress tests ------- \r\n\r\n");
    /* Do dma trigger mode test - simple FFT - one pass */
    paramsetIdx = 0;
    memset(gHWATestParamConfig, 0, sizeof(gHWATestParamConfig));  //init to zero
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_COMPRESS;

    gHWATestParamConfig[paramsetIdx].source.srcAddr = (uint32_t)((uint32_t)srcAddr - SOC_HWA_MEM0);
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = 4 - 1;
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_COMPLEX_16BIT_SIZE;
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = numBFPInputBlock - 1;
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = 4*4;
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 0;

    gHWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = 2 - 1;
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = 4;
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = 2 * 4;
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED;
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    /* two path BFP */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_DISABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.compressDecompress = HWA_CMP_DCMP_COMPRESS;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.method = HWA_COMPRESS_METHOD_BFP;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = 4; //log2(sample bits)
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.BFPMantissaBW = 7;

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        //Below registers only work for ES2.0 samples
        gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.cmpRoundEn = 1;
        gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.decrImagBitw = 1;
        gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.selLfsr = 0;
        gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.ditherEnable = 1;
    }
#endif

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if (errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (Immediate triggered, AccelMode compress) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [Immediate triggered, AccelMode compress] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : Immediate Trigger AccelMode Compression(BFP)\r\n", paramsetIdx);
    }

    paramsetIdx++; //decompress
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_COMPRESS;

    /*source the previous param dst*/
    gHWATestParamConfig[paramsetIdx].source.srcAddr = (uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0 ); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = 2 - 1;
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = 4; // 1 word
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = numBFPInputBlock - 1;
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = 2 * 4; //2 words
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED;
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 0;

    /* M2*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)((uint32_t)dstAddr2  - SOC_HWA_MEM0 ); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = 4 - 1;
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = 4;
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = 4*4;
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED;
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    /* two path BFP */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_DISABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.compressDecompress = HWA_CMP_DCMP_DECOMPRESS;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.method = HWA_COMPRESS_METHOD_BFP;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = 4;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.BFPMantissaBW = 7;

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        //Below registers only work for ES2.0 samples
        gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.cmpRoundEn = 1;
        gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.decrImagBitw = 1;
        gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.selLfsr = 0;
        gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.ditherEnable = 1;
    }
#endif

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if (errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (software triggered, AccelMode decompress) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [software triggered, AccelMode decompress] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : software Trigger AccelMode deCompression(BFP)\r\n", paramsetIdx);
    }

    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));
    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG;
#if defined (SOC_AWR294X)
    gCommonConfig.configMask |= HWA_COMMONCONFIG_MASK_CMP_LFSRSEED0;
    gCommonConfig.compressConfig.cmpLfsrSeed0 = 123;
#endif

    gCommonConfig.paramStartIdx = 0 ;
    gCommonConfig.paramStopIdx = paramsetIdx ;
    gCommonConfig.numLoops = 1;

    errCode = HWA_configCommon(handle, &gCommonConfig);

#if defined (SOC_AWR294X)
    /* copy the input data */
    if(ptrHWADriver->isES2P0Device == true)
    {
        memcpy(srcAddr, (uint8_t*)gHWATest_compressBFP_input_2p1, sizeof(uint32_t) * 4 * numBFPInputBlock);
    }
    else
    {
        memcpy(srcAddr, (uint8_t*)gHWATest_compressBFP_input, sizeof(uint32_t) * 4 * numBFPInputBlock);
    }
#else
    memcpy(srcAddr, (uint8_t*)gHWATest_compressBFP_input, sizeof(uint32_t) * 4 * numBFPInputBlock);
#endif

    errCode = HWA_enable(handle, 1); // set 1 to enable

    errCode = HWA_singleParamSetDonePolling(handle, 0);

    errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);

    errCode = HWA_singleParamSetDonePolling(handle, paramsetIdx);

    DebugP_log("\r\n");

#if defined (SOC_AWR294X)
    /* checked the decompressed data in M2*/
    if(ptrHWADriver->isES2P0Device == true)
    {
        compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr2), (uint8_t*)HWA_BFPcompdecomp_output_2p1, 4 * 4 * numBFPInputBlock);
    }
    else
    {
        compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr2), (uint8_t*)HWA_BFPcompdecomp_output, 4 * 4 * numBFPInputBlock);
    }
#else
    compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr2), (uint8_t*)HWA_BFPcompdecomp_output, 4 * 4 * numBFPInputBlock);
#endif

    if (compareCode1 != 0)
    {
        DebugP_log("Error: HCompression/decompression BFP output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: Compression/decompression BFP output generated by HWA found correct\r\n");
    }
    compareCode2 += compareCode1;

    errCode = HWA_enable(handle, 0);

    /* EGE algorithm */
    numEGECompressedWord = numEGEBlockSize >> 1; //50%
    paramsetIdx++;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_COMPRESS;

    /* compress 16 range bins, per chirp, per rx antenna, needs to check the data format */
    gHWATestParamConfig[paramsetIdx].source.srcAddr = (uint32_t)((uint32_t)srcAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = numEGEBlockSize - 1; // 16 complex values per block
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_COMPLEX_16BIT_SIZE;
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = numEGEInputData/ numEGEBlockSize - 1;
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = HWA_TEST_COMPLEX_16BIT_SIZE * numEGEBlockSize; //next block
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 0;

    gHWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = numEGECompressedWord - 1; //16 * 4 bytes 50% -> 8 words
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = 4;     // 1 word size
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = 4 * numEGECompressedWord; // 8 words per block
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED;
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_DISABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.compressDecompress = HWA_CMP_DCMP_COMPRESS;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.method = HWA_COMPRESS_METHOD_EGE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = 4; //log2(sample bits)
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.EGEKarrayLength = 3; //log2(8)

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (software triggered, AccelMode compress) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [software triggered, AccelMode compress] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("\r\nDebug: paramset %d : software Trigger AccelMode Compression(EGE)\r\n", paramsetIdx);
    }

    paramsetIdx++;
    /* decompression */
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_COMPRESS;

    gHWATestParamConfig[paramsetIdx].source.srcAddr = (uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = numEGECompressedWord - 1; // 8 words
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = 4; // word size
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = numEGEInputData/ numEGEBlockSize - 1;
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = 4 * numEGECompressedWord; //block
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_32BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 0;

    gHWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)((uint32_t)dstAddr + HWA_MEMn_SIZE  - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = numEGEBlockSize - 1; //16 * 4 bytes 50% -> 8 words
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = HWA_TEST_COMPLEX_16BIT_SIZE;
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = HWA_TEST_COMPLEX_16BIT_SIZE * numEGEBlockSize;
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED;
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_DISABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.compressDecompress = HWA_CMP_DCMP_DECOMPRESS;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.method = HWA_COMPRESS_METHOD_EGE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = 4; //log2(sample bits)
    gHWATestParamConfig[paramsetIdx].accelModeArgs.compressMode.EGEKarrayLength = 3; //log2(8)

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (immediate triggered, AccelMode decompress) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [software triggered, AccelMode compress] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : immediate Trigger AccelMode deCompression(EGE)\r\n", paramsetIdx);
    }

    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));
    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG | HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM;

    gCommonConfig.paramStartIdx = paramsetIdx - 1; //compression
    gCommonConfig.paramStopIdx = paramsetIdx; //decompression
    gCommonConfig.numLoops = 1; //do only one iteration


    gCommonConfig.compressConfig.EGEKparam[0] = 0;
    gCommonConfig.compressConfig.EGEKparam[1] = 2;
    gCommonConfig.compressConfig.EGEKparam[2] = 4;
    gCommonConfig.compressConfig.EGEKparam[3] = 6;
    gCommonConfig.compressConfig.EGEKparam[4] = 8;
    gCommonConfig.compressConfig.EGEKparam[5] = 10;
    gCommonConfig.compressConfig.EGEKparam[6] = 12;
    gCommonConfig.compressConfig.EGEKparam[7] = 15;
    errCode = HWA_configCommon(handle, &gCommonConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* copy the input data */
    memcpy(srcAddr, (uint8_t*)gHWATest_compressEGE_input, sizeof(uint32_t) * numEGEInputData);

    errCode = HWA_enable(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_singleParamSetDonePolling(handle, paramsetIdx);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    DebugP_log(" \r\n");

    /* checked the decompressed data in M2*/
    compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr + HWA_MEMn_SIZE), (uint8_t*)HWA_EGEcompdecomp_output, 4 * numEGEInputData);
    if(compareCode1 != 0)
    {
        DebugP_log("Error: Compression/decompression EGE output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: Compression/decompression EGE output generated by HWA found correct\r\n");
    }
    compareCode2 += compareCode1;

    errCode = HWA_enable(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    DebugP_log(" \r\n");
    if(compareCode2 != 0)
    {
        DebugP_log("Feature : HWA compression/decompression Tests FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode2);
    }
    else
    {
        DebugP_log("Feature : HWA compression/decompression Tests PASS\r\n");
    }

    DebugP_log("\r\nHWA compression/decompression test completed \r\n");

    return;
}

static void test_2dfft(void *args)
{
    HWA_Handle              handle = gHwaHandle;
    int32_t                 errCode, compareCode2;
    uint8_t                 paramsetIdx = 0;
    uint8_t                *srcAddr = (uint8_t*)SOC_HWA_MEM0;
    uint8_t                *dstAddr = (uint8_t*)SOC_HWA_MEM1;
    uint16_t                fftSize1 = 8, fftSize2 = 16;
    uint16_t                numInputSamples = 96;

    errCode = HWA_reset(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    compareCode2 = 0;
    DebugP_log("\r\n\r\n------- HWA 2D FFT tests ------- \r\n\r\n");

    /* 2D fft test  */
    paramsetIdx = 0;
    memset(gHWATestParamConfig, 0, sizeof(gHWATestParamConfig));  //init to zero
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;

    gHWATestParamConfig[paramsetIdx].source.srcAddr = (uint32_t)((uint32_t)srcAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = numInputSamples - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = 1 - 1; //no iterations here
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = HWA_TEST_COMPLEX_16BIT_SIZE * numInputSamples; //should be dont care
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 8;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    /* M1*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = fftSize2 * fftSize1 - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = HWA_TEST_COMPLEX_16BIT_SIZE * fftSize2 * fftSize1; //should be dont care
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSize = log2Approx(fftSize2 * fftSize1);
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSizeDim2 = log2Approx(fftSize2);
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x1ff; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_ENABLE; //enabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowStart = 0; //start of window RAM
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.winSymm = HWA_FFT_WINDOW_NONSYMMETRIC; //non-symmetric - in demo do we make this symmetric

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    /* add DC and interference statistics processing*/
    /* DC estimation */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE;
    /* interference statistics */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE;

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (Immediate triggered, AccelMode FFT - 2DFFT) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [Immediate triggered, AccelMode FFT - 2DFFT] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : Immediate Trigger AccelMode FFT - 2DFFT\r\n", paramsetIdx);
    }

    /* configure the common register */
    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG;

    gCommonConfig.paramStartIdx = 0;
    gCommonConfig.paramStopIdx = paramsetIdx;
    gCommonConfig.numLoops = 1;

    errCode = HWA_configCommon(handle, &gCommonConfig);

    /* copy the input data */
    memcpy(srcAddr, (uint8_t*)gHWATest_2DFFT_input, sizeof(uint32_t) *numInputSamples);

    /* configure the window */
    errCode = HWA_configRam(handle, HWA_RAM_TYPE_WINDOW_RAM, (uint8_t *)win_fft2d, sizeof(win_fft2d), gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowStart);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* enable HWA */
    errCode = HWA_enable(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* wait for hwa is done */
    errCode = HWA_singleParamSetDonePolling(handle, paramsetIdx);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* disable  HWA */
    errCode = HWA_enable(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* check the results */
    DebugP_log(" \r\n");

    compareCode2 = memcmp(dstAddr, (uint8_t*)gHWATest_2DFFT_output, 4 * fftSize1 * fftSize2);
    if(compareCode2 != 0)
    {
        DebugP_log("Feature : HWA 2DFFT Tests FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode2);
    }
    else
    {
        DebugP_log("Feature : HWA 2DFFT Tests PASS\r\n");
    }

    DebugP_log("\r\nHWA 2DFFT test completed \r\n\r\n");

    return;
}

static void test_fft4k(void *args)
{
    HWA_Handle              handle = gHwaHandle;
    int32_t                 errCode, compareCode2;
    uint8_t                 paramsetIdx = 0;
    uint8_t                *srcAddr = (uint8_t*)SOC_HWA_MEM0;
    uint8_t                *dstAddr = (uint8_t*)SOC_HWA_MEM1;
    uint16_t                fftSize = 4096;

    errCode = HWA_reset(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    compareCode2 = 0;
    DebugP_log("\r\n\r\n------- HWA FFT 4K tests ------- \r\n\r\n");

    /* FFT size 4096 */
    paramsetIdx = 0;
    memset(gHWATestParamConfig, 0, sizeof(gHWATestParamConfig));  //init to zero
    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;

    gHWATestParamConfig[paramsetIdx].source.srcAddr = (uint32_t)((uint32_t)srcAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = 2048 - 1;
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_COMPLEX_16BIT_SIZE * 2; // 16 bytes
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = 2 - 1; //2 fft
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 0;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    /* M1*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = 2048 - 1;
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = 16;
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = 8; //should be dont care
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 8;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSize = 11;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x7ff; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_ENABLE; //enabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowStart = 0; //start of window RAM
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.winSymm = HWA_FFT_WINDOW_NONSYMMETRIC; //non-symmetric - in demo do we make this symmetric

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples
    /* first paramset, only set the window interpolation configuration */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.FFTstitching.winInterpolateMode = HWA_FFT_WINDOW_INTERPOLATE_MODE_4K;

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (Immediate trigged, AccelMode FFT - 4K) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature :  API HWA_configParamSet() [Immediate trigged, AccelMode FFT - 4K] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : Immediate Trigger AccelMode FFT - 4K - first paramset \r\n", paramsetIdx);
    }

    /* second paramset */
    paramsetIdx++;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;

    gHWATestParamConfig[paramsetIdx].source.srcAddr = (uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = 2 - 1;
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = 8; // 16 bytes
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = 2048 - 1; //2 fft
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = 16; //should be dont care
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_32BIT; //32-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 0;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    /* M1*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = (uint32_t)((uint32_t)dstAddr + 2 * HWA_MEMn_SIZE - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = 2 - 1;
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = 2048*8;
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = 8; //should be dont care
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 8;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSize = 1;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x3ff; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_DISABLE; //disable

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples
    /* second paramset, enable the fft stitching */
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_FFT_STITCHING;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.FFTstitching.twiddlePattern = HWA_FFT_STITCHING_TWID_PATTERN_4K;

    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configParamSet(%d) (Immediate triggered, AccelMode FFT - 4K) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [Immediate triggered, AccelMode FFT - 4K] FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : Immediate Trigger AccelMode FFT - 4K - second paramset \r\n", paramsetIdx);
    }

    /* common configuration */
    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG |
                    HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                    HWA_COMMONCONFIG_MASK_LFSRSEED;
    gCommonConfig.fftConfig.twidDitherEnable = 1;
    gCommonConfig.fftConfig.lfsrSeed = 0x1234567;
    gCommonConfig.paramStartIdx = 0;
    gCommonConfig.paramStopIdx = paramsetIdx;
    gCommonConfig.numLoops = 1;

    errCode = HWA_configCommon(handle, &gCommonConfig);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configCommon(%d) (Immediate triggered, AccelMode FFT - 4K) returned %d\r\n", errCode, paramsetIdx);
        DebugP_log("Feature : API HWA_configParamSet() [Immediate triggered, AccelMode FFT - 4 FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    else
    {
        DebugP_log("Debug: paramset %d : HWA_configCommon FFT - 4K \r\n", paramsetIdx);
    }

    /* copy the input data */
    memcpy(srcAddr, (uint8_t*)gHWATest_4KFFT_input, sizeof(uint32_t) *fftSize);

    /* configure the window */
    errCode = HWA_configRam(handle, HWA_RAM_TYPE_WINDOW_RAM, (uint8_t *)win_fft4k, sizeof(win_fft4k), gHWATestParamConfig[0].accelModeArgs.fftMode.windowStart);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    /* enable HWA */
    errCode = HWA_enable(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* wait for hwa is done, both paramsets */
    errCode = HWA_singleParamSetDonePolling(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_singleParamSetDonePolling(handle, paramsetIdx);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* disable  HWA */
    errCode = HWA_enable(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* check the results */
    DebugP_log(" \r\n");

    compareCode2 = memcmp((uint8_t *)((uint32_t)dstAddr + 2 * HWA_MEMn_SIZE) , (uint8_t*)gHWATest_FFT4K_output, 4 * 2 * fftSize); //output is 32bits I/32 bitsQ
    if(compareCode2 != 0)
    {
        DebugP_log("Feature : HWA 4KFFT Tests FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode2);
    }
    else
    {
        DebugP_log("Feature : HWA 4KFFT Tests PASS\r\n");
    }

    DebugP_log("\r\nHWA 4KFFT test completed \r\n\r\n");

    return;
}

#if defined (SOC_AWR294X)
static void test_SW_restartLoop(void *args)
{
    HWA_Handle              handle = gHwaHandle;
    int32_t                 errCode, compareCode1, compareCode2, compareCode3;
    uint8_t                 paramsetIdx;
    HWA_InterruptConfig     paramISRConfig;
    uint8_t                *srcAddr = (uint8_t*)SOC_HWA_MEM0;
    uint8_t                *dstAddr = (uint8_t*)SOC_HWA_MEM1;
    uint8_t                *dstAddr2 = (uint8_t*)SOC_HWA_MEM2;
    uint8_t                *dstAddr3 = (uint8_t*)SOC_HWA_MEM3;
    SemaphoreP_Object       doneSem;
    SemaphoreP_Object       doneSemALT;
    int32_t                 status;
    SW_RestartLoop =1;

    errCode = HWA_reset(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("\r\n\r\n------- HWA SW restart Loop test ------- \r\n\r\n");

    /*configure paramset used for the context switch tests */
    memset(gHWATestParamConfig, 0, sizeof(gHWATestParamConfig));  //init to zero
    /* gHWATestParamConfig[0] is used in 1st background thread */

    paramsetIdx = 0;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; //Immediate following first - in demo this should be HWA_TRIG_MODE_DFE
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; //do FFT
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(srcAddr);// (uint32_t)((uint32_t)srcAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = HWA_TEST_NUM_SAMPLES - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = HWA_TEST_NUM_RX_ANT - 1; //no iterations here
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 8;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    /* M1*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(dstAddr); //(uint32_t)((uint32_t)dstAddr - SOC_HWA_MEM0); // address is relative to start of MEM0
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = HWA_TEST_1DFFT_SIZE - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSize = log2Approx(HWA_TEST_1DFFT_SIZE);
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x3; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_ENABLE; //enabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowStart = HWA_TEST_1DFFT_WINDOW_START; //start of window RAM
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.winSymm = HWA_FFT_WINDOW_NONSYMMETRIC; //non-symmetric - in demo do we make this symmetric

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    /* if windowing is enabled, load the window coefficients in RAM */
    errCode = HWA_configRam(handle, HWA_RAM_TYPE_WINDOW_RAM, (uint8_t *)win_data225, sizeof(win_data225), HWA_TEST_1DFFT_WINDOW_START);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_configRam(HWA_RAM_TYPE_WINDOW_RAM) FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }
    paramsetIdx = 1;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_NONE;
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(srcAddr);
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(dstAddr);
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);

    //2nd background context thread
    paramsetIdx = 12;
    gHWATestParamConfig[paramsetIdx] = gHWATestParamConfig[0];
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(srcAddr);
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(dstAddr3);
    gHWATestParamConfig[paramsetIdx].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[paramsetIdx], NULL);
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    paramISRConfig.cpu.callbackFn = HWA_swRestartLoop_Test_ParamSetISR_Callback;
    paramISRConfig.cpu.callbackArg = NULL;
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    //High priority context
    paramsetIdx = 10;
    gHWATestParamConfig[paramsetIdx] = gHWATestParamConfig[0];
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(srcAddr);
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(dstAddr2);
    gHWATestParamConfig[paramsetIdx].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[10], NULL);

    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG | HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG_ALT | HWA_COMMONCONFIG_CONTEXTSWITCH_TRIG_CFG;

    gCommonConfig.paramStartIdx = 0;
    gCommonConfig.paramStopIdx = 1;
    gCommonConfig.numLoops = 1;

    gCommonConfig.paramStartIdxALT = 10;
    gCommonConfig.paramStopIdxALT = 10;
    gCommonConfig.numLoopsALT = 1;
    gCommonConfig.contextswitchTriggerMode = HWA_CONTEXTSWITCH_TRIG_MODE_SOFTWARE;

    errCode = HWA_configCommon(handle, &gCommonConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* param all done interrupt for background thread */
    errCode = SemaphoreP_constructBinary(&doneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 0, HWA_Test_DoneISR_Callback, &doneSem);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    /* param all done interrupt for ALT thread  */
    errCode = SemaphoreP_constructBinary(&doneSemALT, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 1, HWA_Test_ALT_DoneISR_Callback, &doneSemALT);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    /* Prepare the input data for test vector based testing */
    /* memcopy the data to MEM0 */
    memcpy(srcAddr, (uint8_t *)gHWATest_1DFFT_input,
        HWA_TEST_NUM_SAMPLES*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);

    /* Enable context switching */
    HWA_enableContextSwitch(handle, 1);
    /* hwa enable */
    errCode = HWA_enable(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* Give trigger for context switching */
    HWA_setContextswitchSoftwareTrigger(handle);

    while (!gHWAALTThreadDone)
    {

    }
    gHWAALTThreadDone = 0;

    /* then wait for ALT thread done interrupt */
    status = SemaphoreP_pend(&doneSemALT, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* then wait for background thread done interrupt */
    status = SemaphoreP_pend(&doneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    /* fft all pass through*/
    compareCode1 = memcmp((uint8_t *)((uint32_t)dstAddr), (uint8_t *)gHWATest_1DFFT_output,
        HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    compareCode2 = memcmp((uint8_t *)((uint32_t)dstAddr2), (uint8_t *)gHWATest_1DFFT_output,
        HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    compareCode3 = memcmp((uint8_t *)((uint32_t)dstAddr3), (uint8_t *)gHWATest_1DFFT_output,
        HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    if((compareCode2 + compareCode1 + compareCode3) != 0)
    {
        DebugP_log("Feature : HWA swResartLoop Tests FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode2);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode1);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode3);
    }
    else
    {
        DebugP_log("Feature : HWA swRestartLoop Test PASS\r\n");
    }

    DebugP_log("\r\nHWA swRestartLoop test completed \r\n\r\n");

    errCode = HWA_disableDoneInterrupt(handle, 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_disableDoneInterrupt returned %d\r\n", errCode);
        DebugP_log("Feature :  API HWA_disableDoneInterrupt() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_disableDoneInterrupt(handle, 1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_disableDoneInterrupt returned %d\r\n", errCode);
        DebugP_log("Feature :  API HWA_disableDoneInterrupt() FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    /* disable HWA */
    errCode = HWA_enable(handle, 0);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enable(0) returned %d\r\n", errCode);
        DebugP_log("Feature : API HWA_enable() to disable HWA FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        return;
    }

    SemaphoreP_destruct(&doneSem);
    SemaphoreP_destruct(&doneSemALT);
}
#endif

static void test_contextswitch(void *args)
{
    HWA_Handle              handle = gHwaHandle;
#if defined (SOC_AWR294X)
    HWA_Object              *ptrHWADriver = (HWA_Object*)gHwaHandle;
#endif
    int32_t                 errCode, compareCode1, compareCode2;
    uint8_t                 paramsetIdx;
    uint8_t                 ii;
    HWA_InterruptConfig     paramISRConfig;
    uint8_t                *srcAddr = (uint8_t*)SOC_HWA_MEM0;
    uint8_t                *dstAddr = (uint8_t*)SOC_HWA_MEM1;
    SemaphoreP_Object       doneSem;
    SemaphoreP_Object       doneSemALT;
    int32_t                 status;
    uint32_t                numLoops;

    errCode = HWA_reset(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("\r\n\r\n------- HWA context switch tests ------- \r\n\r\n");

    /*configure paramset used for the context switch tests */
    memset(gHWATestParamConfig, 0, sizeof(gHWATestParamConfig));  //init to zero
    /* gHWATestParamConfig[0] is used in background thread */
    paramsetIdx = 0;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(srcAddr);
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = HWA_TEST_NUM_SAMPLES - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = HWA_TEST_NUM_RX_ANT - 1;
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 8;

    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(dstAddr);
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = HWA_TEST_NUM_SAMPLES - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_DISABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSize = 0;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowEn = 0; //disable
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowStart = 0; //start of window RAM
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.winSymm = 0;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.vecMultiMode1RamAddrOffset = 5;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.cmultScaleEn = 0;

    /* gHWATestParamConfig[1] is the ALT thread paramset */
    paramsetIdx = 1;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    gHWATestParamConfig[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA ( (uint32_t) srcAddr + (2*0x4000));
    gHWATestParamConfig[paramsetIdx].source.srcAcnt = HWA_TEST_NUM_SAMPLES - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].source.srcAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].source.srcBcnt = HWA_TEST_NUM_RX_ANT - 1; //no iterations here
    gHWATestParamConfig[paramsetIdx].source.srcBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    gHWATestParamConfig[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    gHWATestParamConfig[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    gHWATestParamConfig[paramsetIdx].source.srcConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].source.srcScale = 8;

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    /* M1*/
    gHWATestParamConfig[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA ((uint32_t) dstAddr + (2 * 0x4000));
    gHWATestParamConfig[paramsetIdx].dest.dstAcnt = HWA_TEST_1DFFT_SIZE - 1; //this is samples - 1
    gHWATestParamConfig[paramsetIdx].dest.dstAIdx = HWA_TEST_NUM_RX_ANT * HWA_TEST_COMPLEX_16BIT_SIZE; // 16 bytes
    gHWATestParamConfig[paramsetIdx].dest.dstBIdx = HWA_TEST_COMPLEX_16BIT_SIZE; //should be dont care
    gHWATestParamConfig[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
    gHWATestParamConfig[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    gHWATestParamConfig[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    gHWATestParamConfig[paramsetIdx].dest.dstConjugate = HWA_FEATURE_BIT_DISABLE; //no conjugate
    gHWATestParamConfig[paramsetIdx].dest.dstScale = 0;
    gHWATestParamConfig[paramsetIdx].dest.dstSkipInit = 0; // no skipping
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.fftSize = log2Approx(HWA_TEST_1DFFT_SIZE);
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x3; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_ENABLE; //enabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.windowStart = HWA_TEST_1DFFT_WINDOW_START; //start of window RAM
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.winSymm = HWA_FFT_WINDOW_NONSYMMETRIC; //non-symmetric - in demo do we make this symmetric

    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; //disabled
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples
    gHWATestParamConfig[paramsetIdx].accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    /*gHWATestParamConfig[2] is a dummy paramset used in background thread */
    paramsetIdx = 2;
    gHWATestParamConfig[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[paramsetIdx].accelMode = HWA_ACCELMODE_NONE; //dummy

    /* configure the paramsetdone interrupt */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    paramISRConfig.cpu.callbackFn = HWA_contextswitch_Test_ParamSetISR_Callback;
    paramISRConfig.cpu.callbackArg = NULL;

    errCode = HWA_reset(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* enable the context switch */
    errCode = HWA_enableContextSwitch(handle, 1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableContextSwitch return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    /* set up the data */
    for(ii = 0; ii < HWA_TEST_NUM_SAMPLES; ii++)
    {
        vectorArray[2 * ii] = 0xfffff; //Q20 1
        vectorArray[2 * ii + 1] = 0x0; //imag
    }
    errCode = HWA_configRam(handle, HWA_RAM_TYPE_VECTORMULTIPLY_RAM, (uint8_t *)vectorArray,
        HWA_TEST_NUM_SAMPLES * 8,  //complex coefficients, with 21 bits I and 21 bits Q
        gHWATestParamConfig[0].accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.vecMultiMode1RamAddrOffset * 8);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* Prepare the input data for test vector based testing */
    /* memcopy the data to MEM0 */
    memcpy(srcAddr, (uint8_t *)gHWATest_1DFFT_input,
        HWA_TEST_NUM_SAMPLES*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    memcpy((uint8_t*)((uint32_t)srcAddr + 2 * 0x4000), (uint8_t *)gHWATest_1DFFT_input,
        HWA_TEST_NUM_SAMPLES*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);
    /* m1 vector multiply results */
    memset((uint8_t *)dstAddr, 0, HWA_TEST_1DFFT_SIZE*HWA_TEST_NUM_RX_ANT*HWA_TEST_COMPLEX_16BIT_SIZE);

    compareCode2 = 0;

    /* force context switch tests */
    DebugP_log("\r\nDebug: HWA context switch (force switch) : test starts \r\n");

    /* set up the background thread, 6 paramsets, loop 3, all paramsetDone interrupt enabled
       paramset 0: software trigger, INTR2
       paramset 1: immediate trigger, INTR2
       paramset 2: immediate trigger + force context switch, INTR2
       paramset 3: software trigger, will triggered by ALT allloopDone interrupt, INTR2
       paramset 4: immediate trigger, INTR2
       paramset 5: immediate trigger, INTR2, in the paramsetDone interrupt, will trigger the paramset 0 in the next loop
    */
    paramsetIdx = 0;  //paramset 0
    gHWATestParamConfig[0].triggerMode =  HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);

    paramsetIdx++;  //pareamset 1
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);

    paramsetIdx++; //paramset 2
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_FORCE_ENABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);

    paramsetIdx++; //paramset 3 set as software trigger
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);

    paramsetIdx++; //paramset 4
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);

    paramsetIdx++; //paramset 5
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);

    /* set up the interrupt for background thread paramsets*/
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2;
    for(ii = 0; ii < 6; ii++)
    {
        errCode = HWA_enableParamSetInterrupt(handle, ii, &paramISRConfig);
    }
    DebugP_log("Debug: background thread %d paramsets configuration done\r\n", paramsetIdx + 1);

    /* set up the ALT thread paramsets, 2 paramsets, loop 2, all paramsetDone interrupt enabled
       paramset 10: immediate triggered, INTR1
       paramset 11: immediate triggered, INTR1
       ALT all loop done interrupt is alse enabled, to trigger paramset 3 in the background thread.
    */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;

    paramsetIdx = 10;
    gHWATestParamConfig[1].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[1].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[1], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("Debug: ALT (high priority) thread paramset %d, ", paramsetIdx);

    paramsetIdx++; //paramset 11
    gHWATestParamConfig[1].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[1].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[1], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("paramset %d configuration done\r\n", paramsetIdx);

    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG | HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG_ALT;

    gCommonConfig.paramStartIdx = 0;
    gCommonConfig.paramStopIdx = 5;
    gCommonConfig.numLoops = 3;

    gCommonConfig.paramStartIdxALT = 10;
    gCommonConfig.paramStopIdxALT = 11;
    gCommonConfig.numLoopsALT = 2;

    errCode = HWA_configCommon(handle, &gCommonConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* param all done interrupt for background thread */
    errCode = SemaphoreP_constructBinary(&doneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 0, HWA_Test_DoneISR_Callback, &doneSem);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    /* param all done interrupt for ALT thread  */
    errCode = SemaphoreP_constructBinary(&doneSemALT, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 1, HWA_Test_ALT_DoneISR_Callback, &doneSemALT);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    numLoops = gCommonConfig.numLoops;

    /* hwa enable */
    errCode = HWA_enable(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* trigger background thread */
    errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    numLoops--;
    while (!gHWAALTThreadDone)
    {

    }

    /* wait for ALT treahd  done, trigger paramset 3 in the background thread */
    gHWAALTThreadDone = 0;
    errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    while (numLoops)
    {
        while (!gHWAOneLoopDone) //wait for background thread loop is done
        {

        }
        gHWAOneLoopDone = 0;
        numLoops--;
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //trigger next loop for background thread
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

        while (!gHWAALTThreadDone)
        {

        }
        gHWAALTThreadDone = 0;
        //wait for ALT thread done, trigger paramset 3
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    while (!gHWAOneLoopDone) //wait for last paramset ISR in background thread is finish
    {

    }
    gHWAOneLoopDone = 0;
    /* then wait for ALT thread done interrupt */
    status = SemaphoreP_pend(&doneSemALT, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* then wait for background thread done interrupt */
    status = SemaphoreP_pend(&doneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* disable interrupt */
    for(ii = 0; ii < 6; ii++)
    {
        errCode = HWA_disableParamSetInterrupt(handle, ii, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_disableParamSetInterrupt(handle, 10, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableParamSetInterrupt(handle, 11, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableDoneInterrupt(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableDoneInterrupt(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    DebugP_log("\r\nDebug:  HWA completed, check the results \r\n\r\n");
    /* check the results in gHWAcontextswitchTestParamSetIdx */
    compareCode1 = memcmp((uint8_t *)gHWAcontextswitchTestParamSetIdx, (uint8_t*)gHWATest_contextswitchParamIdx_output, 10 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (force switch) : first loop output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (force switch) : first loop  generated by HWA found correct\r\n");
    }
    compareCode2 += compareCode1;
    compareCode1 = memcmp((uint8_t *) (gHWAcontextswitchTestParamSetIdx + 10), (uint8_t*)gHWATest_contextswitchParamIdx_output, 10 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (force switch) : second loop output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (force switch) : second loop  generated by HWA found correct\r\n");
    }
    compareCode2 += compareCode1;

    compareCode1 = memcmp((uint8_t *)(gHWAcontextswitchTestParamSetIdx + 20), (uint8_t*)gHWATest_contextswitchParamIdx_output, 10 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (force switch) : third loop output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (force switch) : third loop  generated by HWA found correct\r\n");
    }

    /* check the interrupt counts from two threads*/
    if((intr2ParamDoneCount == 18) && (intr1ParamDoneCount==12))
    {
        DebugP_log("Debug: HWA context switch (force switch) : INTR1/INTR2 paramsetDone interrupts generated by HWA found correct\r\n");
        compareCode1 = 0;
    }
    else
    {
        DebugP_log("Debug: HWA context switch (force switch) : INTR1/INTR2 paramsetDone interrupts generated by HWA found incorrect : %d, %d\r\n", intr2ParamDoneCount, intr1ParamDoneCount);
        compareCode1 = 1;
    }

    compareCode2 += compareCode1;
    if(compareCode2 == 0)
    {
        DebugP_log("\r\nDebug: HWA context switch (force switch) : test passes \r\n");
    }
    /* disable HWA */
    errCode = HWA_enable(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    /*  delete SemaphoreP */
    SemaphoreP_destruct(&doneSem);
    SemaphoreP_destruct(&doneSemALT);

    /* software trigger context switch */
    DebugP_log(" \r\n");

    /* 6 background thread paramsets same as forced context switch test, except all paramsets are software triggered,
       and in paramset 2, context switch is triggered by software
       background thread, paramset 0,1,2, INTR2, paramset 3,4,5 INTR1
       ALT thread, paramset 10, INTR1,pramset 11, INTR2
    */
    DebugP_log("\r\nDebug: HWA context switch (software trigger switch) : test starts \r\n");
    paramsetIdx = 0;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_NONFORCE_ENABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* set up the interrupt for background thread paramsets*/
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2;
    for(ii = 0; ii < 3; ii++)
    {
        errCode = HWA_enableParamSetInterrupt(handle, ii, &paramISRConfig);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    for(ii = 3; ii < 6; ii++)
    {
        errCode = HWA_enableParamSetInterrupt(handle, ii, &paramISRConfig);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }
    DebugP_log("Debug: background thread %d paramsets configuration done\r\n", paramsetIdx + 1);

    /* 2 paramsets in ALT thread, same as foced context switch test, except all software triggered */
    paramsetIdx = 10;
    gHWATestParamConfig[1].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[1].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[1], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("Debug: high-priority thread paramset %d, ", 10);

    paramsetIdx++;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[1], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2;
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("paramset %d configurations done\r\n", 11);

    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG | HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG_ALT | HWA_COMMONCONFIG_CONTEXTSWITCH_TRIG_CFG;
    gCommonConfig.paramStartIdx = 0;
    gCommonConfig.paramStopIdx = 5;
    gCommonConfig.numLoops = 3;
    gCommonConfig.paramStartIdxALT = 10;
    gCommonConfig.paramStopIdxALT = 11;
    gCommonConfig.numLoopsALT = 2;
    gCommonConfig.contextswitchTriggerMode = HWA_CONTEXTSWITCH_TRIG_MODE_SOFTWARE;
    errCode = HWA_configCommon(handle, &gCommonConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* param all done interrupt for background thread */
    errCode = SemaphoreP_constructBinary(&doneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 0, HWA_Test_DoneISR_Callback, &doneSem);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    /* param all done interrupt for ALT thread */
    errCode = SemaphoreP_constructBinary(&doneSemALT, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 1, HWA_Test_ALT_DoneISR_Callback, &doneSemALT);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    numLoops = gCommonConfig.numLoops;

    errCode = HWA_enableContextSwitch(handle, 1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableContextSwitch return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_enable(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    for(ii = 0; ii < numLoops; ii++)
    {
        /* trigger backgroud thread */
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 0
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 1
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
#if defined (SOC_AWR294X)
        /* Applicable for only AWR294x ES1.0.*/
        if(ptrHWADriver->isES2P0Device == false)
        {
            /* trigger high-priority ALT thread */
            errCode = HWA_setContextswitchSoftwareTrigger(handle);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        }
#else
        /* Applicable for only AM273x devices.*/
        /* trigger high-priority ALT thread */
        errCode = HWA_setContextswitchSoftwareTrigger(handle);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

#endif
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 2
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
#if defined (SOC_AWR294X)
        /* Applicable for only AWR294x ES2.0 devices.*/
        if(ptrHWADriver->isES2P0Device == true)
        {
            /* trigger high-priority ALT thread */
            errCode = HWA_setContextswitchSoftwareTrigger(handle);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        }
#endif
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 10
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 11
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 10
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 11
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 3
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 4
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 5
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        while (!gHWAcontextswitchParamDone)
        {

        }
        gHWAcontextswitchParamDone = 0;
    }

    /* then wait for ALT thread done interrupt */
    status = SemaphoreP_pend(&doneSemALT, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* then wait for background thread done interrupt */
    status = SemaphoreP_pend(&doneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    for(ii = 0; ii < 3; ii++)
    {
        errCode = HWA_disableParamSetInterrupt(handle, ii, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }
    for(ii = 3; ii < 6; ii++)
    {
        errCode = HWA_disableParamSetInterrupt(handle, ii, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_disableParamSetInterrupt(handle, 10, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableParamSetInterrupt(handle, 11, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    errCode = HWA_disableDoneInterrupt(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableDoneInterrupt(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    DebugP_log("\r\nDebug:  HWA completed, check the results \r\n\r\n");
    /* check the results in gHWAcontextswitchTestParamSetIdx */
    compareCode1 = memcmp((uint8_t *) (gHWAcontextswitchTestParamSetIdx + 30), (uint8_t*)gHWATest_contextswitchParamIdx_output, 10 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (software trigger switch) : first loop output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (software trigger switch) : first loop  generated by HWA found correct\r\n");
    }
    compareCode2 += compareCode1;
    compareCode1 = memcmp((uint8_t *)(gHWAcontextswitchTestParamSetIdx + 40), (uint8_t*)gHWATest_contextswitchParamIdx_output, 10 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (software trigger switch) : second loop output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (software trigger switch) : second loop  generated by HWA found correct\r\n");
    }
    compareCode2 += compareCode1;

    compareCode1 = memcmp((uint8_t *)(gHWAcontextswitchTestParamSetIdx + 50), (uint8_t*)gHWATest_contextswitchParamIdx_output, 10 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (software trigger switch) : third loop output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (software trigger switch) : third loop  generated by HWA found correct\r\n");
    }

    /* check the interrupt counts from two threads*/
    if((intr2ParamDoneCount == 18  + 15) && (intr1ParamDoneCount == 12 + 15))
    {
        DebugP_log("Debug: HWA context switch (software trigger switch) : INTR1/INTR2 paramsetDone interrupts generated by HWA found correct\r\n");
        compareCode1 = 0;
    }
    else
    {
        DebugP_log("Debug: HWA context switch (software trigger switch) : INTR1/INTR2 paramsetDone interrupts generated by HWA found incorrect : %d, %d\r\n", intr2ParamDoneCount, intr1ParamDoneCount);
        compareCode1 = 1;
    }

    if(compareCode2 == 0)
    {
        DebugP_log("\r\nDebug: HWA context switch (software trigger switch) : test passes \r\n");
    }

    errCode = HWA_enable(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    /*  delete SemaphoreP */
    SemaphoreP_destruct(&doneSem);
    SemaphoreP_destruct(&doneSemALT);

    /* DMA trigger context switch */
    DebugP_log(" \r\n");

    /* 6 paramsets in background thread are same as force context switch test, except
       paramset 0, 1 are immediate trigger,
       paramset 2 is software trigger, and also set the DMA trigger context switch
       paramset 3 software trigger, triggered by ALT thread allloopDone interrupt
       paramset 4, 5 all immediate trigger.
       and all use INTR1
   */
    DebugP_log("\r\nDebug: HWA context switch (DMA trigger switch) : test starts \r\n");
    paramsetIdx = 0;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_NONFORCE_ENABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* set up the interrupt for background thread paramsets*/
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    for(ii = 0; ii < 6; ii++)
    {
        errCode = HWA_enableParamSetInterrupt(handle, ii, &paramISRConfig);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    DebugP_log("Debug: background thread %d paramsets configuration done\r\n", paramsetIdx + 1);

    /* 2 paramsets in ALT thread are same as forced context switch, except all immediate trigger */
    paramsetIdx = 10;
    gHWATestParamConfig[1].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[1], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("Debug: high-priority thread paramset %d, ", 10);

    paramsetIdx++;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[1], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("paramset %d configurations done\r\n", 11);

    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG | HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG_ALT | HWA_COMMONCONFIG_CONTEXTSWITCH_TRIG_CFG;
    gCommonConfig.paramStartIdx = 0;
    gCommonConfig.paramStopIdx = 5;
    gCommonConfig.numLoops = 3;
    gCommonConfig.paramStartIdxALT = 10;
    gCommonConfig.paramStopIdxALT = 11;
    gCommonConfig.numLoopsALT = 2;
    gCommonConfig.contextswitchTriggerMode = HWA_CONTEXTSWITCH_TRIG_MODE_DMA;
    gCommonConfig.contextswitchTriggerSrc = 4; //DMA channel number
    errCode = HWA_configCommon(handle, &gCommonConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* param all done interrupt for background thread */
    errCode = SemaphoreP_constructBinary(&doneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 0, HWA_Test_DoneISR_Callback, &doneSem);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    /* param all done interrupt for ALT thread */
    errCode = SemaphoreP_constructBinary(&doneSemALT, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 1, HWA_Test_ALT_DoneISR_Callback, &doneSemALT);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_enableContextSwitch(handle, 1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableContextSwitch return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_enable(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

#if defined (SOC_AWR294X)
    if(ptrHWADriver->isES2P0Device == true)
    {
        /* This is applicable for only AWR294x ES2.0 devices. */
        for(ii = 0; ii < numLoops; ii++)
        {
            gHWAOneLoopDone = 0;

            /* trigger backgroud thread, paramset 2*/
            errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 2
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
            //This is required for the 1st iteration to synchronize the HWA context -switching to what is expected else context switching in ES2.0 works as expected
            if (ii==0)
            {
                while(intr1ParamDoneCount!=30)
                {

                }
            }

           if (ii==1)
           {
                while(intr1ParamDoneCount!=40)
                {

                }
            }

           if (ii==2)
           {
                while(intr1ParamDoneCount!=50)
                {

                }
            }

            //For ES2.0 samples below lines of code must be active
            errCode = HWA_setContextswitchDMAManualTrigger(handle, gCommonConfig.contextswitchTriggerSrc); //manually triger the DMA trigger for context switch
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

            /* then wait for ALT thread done interrupt */
            status = SemaphoreP_pend(&doneSemALT, SystemP_WAIT_FOREVER);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

            //when ALT thread is done, trigger param3
            errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 3
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

            while(gHWAOneLoopDone!=1)
            {

            }
        }

    }
    else
    {
        for(ii = 0; ii < numLoops; ii++)
        {
            errCode = HWA_setContextswitchDMAManualTrigger(handle, gCommonConfig.contextswitchTriggerSrc); //manually triger the DMA trigger for context switch
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

            /* trigger backgroud thread, paramset 2*/
            errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 2
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

            /* then wait for ALT thread done interrupt */
            status = SemaphoreP_pend(&doneSemALT, SystemP_WAIT_FOREVER);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

            //when ALT thread is done, trigger param3
            errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 3
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
        }
    }
#else
    for(ii = 0; ii < numLoops; ii++)
    {
        errCode = HWA_setContextswitchDMAManualTrigger(handle, gCommonConfig.contextswitchTriggerSrc); //manually triger the DMA trigger for context switch
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

        /* trigger backgroud thread, paramset 2*/
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 2
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

        /* then wait for ALT thread done interrupt */
        status = SemaphoreP_pend(&doneSemALT, SystemP_WAIT_FOREVER);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

        //when ALT thread is done, trigger param3
        errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //param 3
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }
#endif

    /* then wait for background done interrupt */
    status = SemaphoreP_pend(&doneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    for(ii = 0; ii < 6; ii++)
    {
        errCode = HWA_disableParamSetInterrupt(handle, ii, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_disableParamSetInterrupt(handle, 10, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableParamSetInterrupt(handle, 11, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    errCode = HWA_disableDoneInterrupt(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableDoneInterrupt(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    DebugP_log("\r\nDebug:  HWA completed, check the results \r\n\r\n");
    /* check the results in gHWAcontextswitchTestParamSetIdx */
    compareCode1 = memcmp((uint8_t *)(gHWAcontextswitchTestParamSetIdx + 60), (uint8_t*)gHWATest_contextswitchParamIdx_output, 10 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (DMA trigger switch) : first loop output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (DMA trigger switch) : first loop  generated by HWA found correct\r\n");
    }
    compareCode2 += compareCode1;
    compareCode1 = memcmp((uint8_t *)(gHWAcontextswitchTestParamSetIdx + 70), (uint8_t*)gHWATest_contextswitchParamIdx_output, 10 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (DMA trigger switch) : second loop output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (DMA trigger switch) : second loop  generated by HWA found correct\r\n");
    }
    compareCode2 += compareCode1;

    compareCode1 = memcmp((uint8_t *)(gHWAcontextswitchTestParamSetIdx + 80), (uint8_t*)gHWATest_contextswitchParamIdx_output, 10 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (DMA trigger switch) : third loop output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (DMA trigger switch) : third loop  generated by HWA found correct\r\n");
    }


    /* check the interrupt counts from two threads*/
    if((intr2ParamDoneCount == (18 + 15)) && (intr1ParamDoneCount == (12 + 15 + 30)))
    {
        DebugP_log("Debug: HWA context switch (DMA trigger switch) : INTR1/INTR2 paramsetDone interrupts generated by HWA found correct\r\n");
        compareCode1 = 0;
    }
    else
    {
        DebugP_log("Debug: HWA context switch (DMA trigger switch) : INTR1/INTR2 paramsetDone interrupts generated by HWA found incorrect : %d, %d\r\n", intr2ParamDoneCount, intr1ParamDoneCount);
        compareCode1 = 1;
    }

    if(compareCode2 == 0)
    {
        DebugP_log("\r\nDebug: HWA context switch (DMA trigger switch) : test passes \r\n");
    }

    errCode = HWA_enable(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /*  delete SemaphoreP */
    SemaphoreP_destruct(&doneSem);
    SemaphoreP_destruct(&doneSemALT);

    /* multiple context switches in background switch with dummy paramsets,
       context switch is enabled by either force, or software trigger,
       all use INTR2 */
    DebugP_log(" \r\n");

    /* set up paramsets for background thread */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2;

    paramsetIdx = 32;
    gHWATestParamConfig[2].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[2].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_FORCE_ENABLE; //first one is force context swith
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[2], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    gHWATestParamConfig[2].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[2].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_NONFORCE_ENABLE; //second one is software triggerd context switch;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[2], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    paramsetIdx++;
    gHWATestParamConfig[0].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    gHWATestParamConfig[0].contextswitchCfg = HWA_PARAMSET_CONTEXTSWITCH_DISABLE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[0], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* 2 paramsets in ALT thread are same as forced context switch, except all immediate trigger */
    paramsetIdx = 10;
    gHWATestParamConfig[1].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[1], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("Debug: high-priority thread paramset %d, ", 10);

    paramsetIdx++;
    errCode = HWA_configParamSet(handle, paramsetIdx, &gHWATestParamConfig[1], NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableParamSetInterrupt(handle, paramsetIdx, &paramISRConfig);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    DebugP_log("paramset %d configurations done\r\n", 11);

    memset((void *)&gCommonConfig, 0, sizeof(HWA_CommonConfig));

    gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG | HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG_ALT | HWA_COMMONCONFIG_CONTEXTSWITCH_TRIG_CFG;
    gCommonConfig.paramStartIdx = 32;
    gCommonConfig.paramStopIdx = 34;
    gCommonConfig.numLoops = 1;
    gCommonConfig.paramStartIdxALT = 10;
    gCommonConfig.paramStopIdxALT = 11;
    gCommonConfig.numLoopsALT = 1;
    gCommonConfig.contextswitchTriggerMode = HWA_CONTEXTSWITCH_TRIG_MODE_SOFTWARE;
    errCode = HWA_configCommon(handle, &gCommonConfig);

    /* param all done interrupt for background thread */
    errCode = SemaphoreP_constructBinary(&doneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 0, HWA_Test_DoneISR_Callback, &doneSem);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    /* param all done interrupt for ALT thread */
    errCode = SemaphoreP_constructBinary(&doneSemALT, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_enableDoneInterrupt(handle, 1, HWA_Test_ALT_DoneISR_Callback, &doneSemALT);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableDoneInterrupt return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_enableContextSwitch(handle, 1);
    if(errCode != 0)
    {
        DebugP_log("Error: HWA_enableContextSwitch return %d\r\n", errCode);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }

    errCode = HWA_enable(handle, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //trigger paramset 32
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* force context switch, then wait for ALT thread done interrupt */
    status = SemaphoreP_pend(&doneSemALT, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

#if defined (SOC_AWR294X)
    /* Applicable for only AWR294x ES1.0. */
    if(ptrHWADriver->isES2P0Device == false)
    {
        /* trigger high-priority ALT thread */
        errCode = HWA_setContextswitchSoftwareTrigger(handle);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }
#else
    /* Applicable for only AM273X. */
    /* trigger high-priority ALT thread */
    errCode = HWA_setContextswitchSoftwareTrigger(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
#endif

    errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //trigger paramset 33
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

#if defined (SOC_AWR294X)
    /* Applicable for only AWR294x ES2.0 devices. */
    if(ptrHWADriver->isES2P0Device == true)
    {
        /* trigger high-priority ALT thread */
        errCode = HWA_setContextswitchSoftwareTrigger(handle);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    }
#endif

    /* force context switch, then wait for ALT thread done interrupt */
    status = SemaphoreP_pend(&doneSemALT, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    errCode = HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE); //trigger paramset 34
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /* wait for background thread done */
    status = SemaphoreP_pend(&doneSem, SystemP_WAIT_FOREVER);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    errCode = HWA_disableParamSetInterrupt(handle, 32, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1 | HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableParamSetInterrupt(handle, 33, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1 | HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableParamSetInterrupt(handle, 34, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1 | HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    errCode = HWA_disableParamSetInterrupt(handle, 10, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1 | HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);
    errCode = HWA_disableParamSetInterrupt(handle, 11, HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1 | HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    errCode = HWA_enable(handle, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, errCode);

    /*  delete SemaphoreP */
    SemaphoreP_destruct(&doneSem);
    SemaphoreP_destruct(&doneSemALT);

    DebugP_log("\r\nDebug:  HWA completed, check the results \r\n\r\n");

    compareCode1 = memcmp((uint8_t *)(gHWAcontextswitchTestParamSetIdx + 90), (uint8_t*) (gHWATest_contextswitchParamIdx_output + 10), 7 * sizeof(uint32_t));
    if(compareCode1 != 0)
    {
        DebugP_log("Error: HWA context switch (multiple switches from dummy paramset) : output found incorrect: error %d\r\n", compareCode1);
    }
    else
    {
        DebugP_log("Debug: HWA context switch (multiple switches from dummy paramset) : output generated by HWA found correct\r\n");
    }
    compareCode2 += compareCode1;

    /* check the interrupt counts from two threads*/
    if((intr2ParamDoneCount == (18 + 15 + 7)) && (intr1ParamDoneCount == (12 + 15 + 30)))
    {
        DebugP_log("Debug: HWA context switch (multiple switches from dummy paramset) : INTR1/INTR2 paramsetDone interrupts generated by HWA found correct\r\n");
        compareCode1 = 0;
    }
    else
    {
        DebugP_log("Debug: HWA context switch (multiple switches from dummy paramset) : INTR1/INTR2 paramsetDone interrupts generated by HWA found incorrect : %d, %d\r\n", intr2ParamDoneCount, intr1ParamDoneCount);
        compareCode1 = 1;
    }
    compareCode2 += compareCode1;

    if(compareCode2 == 0)
    {
        DebugP_log("\r\nDebug: HWA context switch (multiple switches from dummy paramset) : test passes \r\n");
    }

    DebugP_log(" \r\n");
    if(compareCode2 != 0)
    {
        DebugP_log("Feature :  HWA context switch Tests FAIL\r\n");
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, compareCode2);
    }
    else
    {
        DebugP_log("Feature :  HWA context switch Tests PASS\r\n");
    }

    DebugP_log("\r\nHWA context switch tests complelted \r\n\r\n");

    return;
}

/* ISR */
static void HWA_Test_ParamSetISR_Callback(uint32_t intrIdx, uint32_t paramSet, void *arg)
{
    SemaphoreP_Object *semHandle;

    gHWATestParamSetISR++;
    if(arg != NULL)
    {
        semHandle = (SemaphoreP_Object *)arg;
        SemaphoreP_post(semHandle);
    }
}

static void HWA_contextswitch_Test_ParamSetISR_Callback(uint32_t intrIdx, uint32_t paramSet, void *arg)
{
    gHWAcontextswitchTestParamSetIdx[gHWAcontextswitchTestParamIdx] = paramSet;
    gHWAcontextswitchTestParamIdx++;
    /* paramset 5 is the last paramset in background thread, once it is done, trigger the next loop in background thread */
    if(paramSet == 5)
    {
        gHWAOneLoopDone = 1;
    }

    if(intrIdx == HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1)
    {
        intr1ParamDoneCount++;
    }
    else
    {
        intr2ParamDoneCount++;
    }

    /* for software triggered switch, every paramset is software triggered, set this flag to trigger next paramset */
    gHWAcontextswitchParamDone = 1;
}

#if defined (SOC_AWR294X)
static void HWA_swRestartLoop_Test_ParamSetISR_Callback(uint32_t intrIdx, uint32_t paramSet, void *arg)
{
    if(intrIdx == HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1 && paramSet == 12)
    {
        intr1ParamDoneCount_swRestartLoop++;
    }
}
#endif

static void HWA_Test_DoneISR_Callback(uint32_t threadIdx, void *arg)
{
    SemaphoreP_Object *semHandle;

    gHWATestDoneISR = threadIdx + 1;
#if defined (SOC_AWR294X)
    if(SW_RestartLoop)
    {
        gCommonConfig.configMask = HWA_COMMONCONFIG_MASK_SW_RESTART_LOOP | HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG;
        gCommonConfig.numLoops = 3;
        gCommonConfig.paramStartIdx = 12;
        gCommonConfig.paramStopIdx = 12;
        SW_RestartLoop = 0;
        gCommonConfig.swRestartLoop = 1;
        HWA_configCommon(gHwaHandle, &gCommonConfig);
    }
#endif
    if(arg != NULL)
    {
        semHandle = (SemaphoreP_Object *) arg;
        SemaphoreP_post(semHandle);
    }
}

static void HWA_Test_ALT_DoneISR_Callback(uint32_t threadIdx, void *arg)
{
    SemaphoreP_Object *semHandle;

    gHWATestDoneISR = threadIdx + 2;
    gHWAALTThreadDone = 1;
    if(arg != NULL)
    {
        semHandle = (SemaphoreP_Object *) arg;
        SemaphoreP_post(semHandle);
    }
}

/* Helper functions */
static uint32_t log2Approx(uint32_t x)
{
    uint32_t idx,detectFlag=0;

    if( x < 2)
    {
        return (0);
    }

    idx=32;
    while((detectFlag==0)||(idx==0))
    {
        if(x&0x80000000)
        {
            detectFlag=1;
        }
        x<<=1;
        idx--;
    }

    return(idx);
}
