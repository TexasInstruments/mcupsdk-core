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

/*
 *  This example demonstrates eQEP capture test.
 *  Example configures the eQEP and captures the quadrature input signal
 *  at index event. Example also configures the eQEP to calculate frequency
 *  using unit timeout event. Based on the position count values,
 *  it calculates the frequency of the input signal.
 *  EQEP signal is generated using GPIO pin toggling.
 *  GPIO pins need to be looped back to the EQEP pins that are available in the
 *  IO expansion board.
 */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/TaskP.h>
#include <unity.h>
#include <drivers/eqep.h>
#include <drivers/hw_include/csl_types.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "test_eqep_pattern_gen.h"

typedef struct EQEP_TestParams_s {

    uint32_t eqepDir;
    uint32_t eqepFreqMeas;
    uint32_t eqepCntrRstIdxEvt;
    uint32_t eqepIntrEvt;
    uint32_t eqepWdgEvtTest;
    uint32_t eqepIntrNum;
    uint32_t eqepBaseAddr;
} EQEP_TestParams;


/* EQEP Interrupt Sources */
#define EQEP_INT_ALL                        (EQEP_INT_GLOBAL            | \
                                             EQEP_INT_POS_CNT_ERROR     | \
                                             EQEP_INT_PHASE_ERROR       | \
                                             EQEP_INT_DIR_CHANGE        | \
                                             EQEP_INT_WATCHDOG          | \
                                             EQEP_INT_UNDERFLOW         | \
                                             EQEP_INT_OVERFLOW          | \
                                             EQEP_INT_POS_COMP_READY    | \
                                             EQEP_INT_POS_COMP_MATCH    | \
                                             EQEP_INT_STROBE_EVNT_LATCH | \
                                             EQEP_INT_INDEX_EVNT_LATCH  | \
                                             EQEP_INT_UNIT_TIME_OUT     | \
                                             EQEP_INT_QMA_ERROR)
/* Number of EQEP EVENTS */
#define EQEP_EVENT_CNT                      (10U)

/* Frequency of EQEP signal for testing unit timeout event in Hz. */
#define EQEP_SIGNAL_TEST_FREQ               (500U)

/* Frequence of unit timeout event in Hz.
 * Should be less than the EQEP signal frequency to capture multiple clocks at
 * Timeout event. So configuring it to 1/10th the EQEP signal frequency. */
#define EQEP_UNIT_TIMEOUT_FREQ              (EQEP_SIGNAL_TEST_FREQ / EQEP_EVENT_CNT)

/* Number of times the EQEP pattern is generated. */
#define EQEP_PATTERN_GEN_LOOP_COUNT         (EQEP_SIGNAL_TEST_FREQ / EQEP_EVENT_CNT)

/* By default IP holds LOW/HIGH signal in EQEP pins.
 * First position count can contain old value.
 * Setting variance to 4 as we capture 4 edges per cycle. */
#define EQEP_POS_CNT_VARIANCE               (4U)

/* Global variables and objects */
static HwiP_Object           gEqepHwiObject;
/* Variable to hold base address of EQEP/GPIO that is used */
uint32_t                     gEqepBaseAddr;
/* EQEP interrupt handle. */
static SemaphoreP_Object     gEqepSyncSem;

/* Isr count at different events. */
volatile uint32_t            gEqepIsrCnt = 0U;
/* Pos Count capture at different events. */
uint32_t                     gEqepPosCnt[EQEP_EVENT_CNT];
uint32_t                     gEqepCapPrd[EQEP_EVENT_CNT];
uint32_t                     gEqepCapTmr[EQEP_EVENT_CNT];
uint32_t                     gEqepCapPrdLatch[EQEP_EVENT_CNT];
uint32_t                     gEqepCapTmrLatch[EQEP_EVENT_CNT];
uint32_t                     gEqepAPin, gEqepBPin, gEqepIPin, gEqepSPin;

/* Static Function declarations */
static void App_eqepIntrISR(void *arg);
static void App_eqepInitQuadratureWave(EQEP_TestParams *testParams);
static void App_eqepInitFrequencyCalculation(EQEP_TestParams *testParams);
static void App_eqepInitPattern(EqepAppPatternParams *eqepPattern);
static void App_eqepComparePosCnt(int32_t expCnt);
static void App_eqepTestClockwiseDirection(EQEP_TestParams *testParams);
static void App_eqepTestAntiClockwiseDirection(EQEP_TestParams *testParams);
static void App_eqepTestFrequency(EQEP_TestParams *testParams);
static uint32_t App_eqepCalculateFrequencyUnitTimeout(void);
static void test_eqep_init_test_params(EQEP_TestParams *testParams,
                                           uint32_t testCaseId);
static void eqep_capture_main(void *args);
static void eqep_phase_error_wdg_test(void *args);
static void eqep_code_coverage_enhancement_test(void *args);
                                           
void test_main(void *args)
{
    EQEP_TestParams testParams;
    /* Open drivers */
    Drivers_open();

    UNITY_BEGIN();

    DebugP_log("Please refer EXAMPLES_DRIVERS_EQEP_CAPTURE example user \
guide for the test setup details.\r\n");

    /* Run tests */
    test_eqep_init_test_params(&testParams, 1546);
    RUN_TEST(eqep_capture_main, 1546, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1547);
    RUN_TEST(eqep_capture_main, 1547, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1548);
    RUN_TEST(eqep_capture_main, 1548, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1549);
    RUN_TEST(eqep_capture_main, 1549, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1550);
    RUN_TEST(eqep_capture_main, 1550, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1551);
    RUN_TEST(eqep_capture_main, 1551, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1553);
    RUN_TEST(eqep_capture_main, 1553, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1554);
    RUN_TEST(eqep_phase_error_wdg_test, 1554, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1555);
    RUN_TEST(eqep_phase_error_wdg_test, 1555, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1556);
    RUN_TEST(eqep_phase_error_wdg_test, 1556, (void*)&testParams);
    test_eqep_init_test_params(&testParams, 1738);
    RUN_TEST(eqep_code_coverage_enhancement_test, 1738, (void*)&testParams);

    UNITY_END();

    /* Close drivers */
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

static void eqep_capture_main(void *args)
{
    int32_t              status;
    HwiP_Params          hwiPrms;
    EQEP_TestParams *testParams = (EQEP_TestParams *)args;

    status = SemaphoreP_constructBinary(&gEqepSyncSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = testParams->eqepIntrNum;
    hwiPrms.callback    = &App_eqepIntrISR;
    hwiPrms.isPulse     = CONFIG_EQEP0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEqepHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Address translate */
    gEqepBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(testParams->eqepBaseAddr);
    /* Clear Interrupts */
    EQEP_disableInterrupt(gEqepBaseAddr, EQEP_INT_ALL);
    EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_ALL);

    DebugP_log("Sending quadrature wave for 50 cycles in clockwise direction.\
With index event in between, Captures 4 edges per cycle\r\n");

    /* Test Clockwise Direction */
    App_eqepTestClockwiseDirection(testParams);

    DebugP_log("Quadrature input capture test clockwise direction passed\r\n");

    DebugP_log("Sending quadrature wave for 50 cycles in anticlockwise direction.\
With index event in between, Captures 4 edges per cycle\r\n");

    /* Test Anti Clockwise Direction */
    App_eqepTestAntiClockwiseDirection(testParams);

    DebugP_log("Quadrature input capture test anti clockwise direction passed\r\n");

    if (testParams->eqepFreqMeas == TRUE)
    {
        DebugP_log("Starting Frequency calculation test\r\n");

        /* Frequency Calculation Test */
        App_eqepTestFrequency(testParams);

        DebugP_log("Frequency calculation test passed\r\n");
    }

    EQEP_disableModule(gEqepBaseAddr);
    EQEP_disableCapture(gEqepBaseAddr);
    EQEP_disableUnitTimer(gEqepBaseAddr);

    HwiP_destruct(&gEqepHwiObject);
    SemaphoreP_destruct(&gEqepSyncSem);

}

static void eqep_phase_error_wdg_test(void *args)
{
    int32_t              status;
    HwiP_Params          hwiPrms;
    EqepAppPatternParams eqepPattern = {0U};
    EQEP_TestParams *testParams = (EQEP_TestParams *)args;

    status = SemaphoreP_constructBinary(&gEqepSyncSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = testParams->eqepIntrNum;
    hwiPrms.callback    = &App_eqepIntrISR;
    hwiPrms.isPulse     = CONFIG_EQEP0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEqepHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Address translate */
    gEqepBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(testParams->eqepBaseAddr);
    /* Clear Interrupts */
    EQEP_disableInterrupt(gEqepBaseAddr, EQEP_INT_ALL);
    EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_ALL);

    /* Enable capture from quadrature wave with index event. */
    App_eqepInitQuadratureWave(testParams);

    /* Reset ISR Count */
    gEqepIsrCnt = 0U;

    /* Init default EQEP pattern params */
    App_eqepInitPattern(&eqepPattern);

    /* Enable capture from quadrature wave with index event. */
    App_eqepInitQuadratureWave(testParams);

    /* Start EQEP Pattern Generation. */
    eqepPattern.direction        = testParams->eqepDir;
    eqepPattern.generateIdxPulse = TRUE;
    App_eqepGeneratePattern(&eqepPattern);

    if (testParams->eqepWdgEvtTest == TRUE)
    {
        EQEP_disableWatchdog(gEqepBaseAddr);
    }

    HwiP_destruct(&gEqepHwiObject);
    SemaphoreP_destruct(&gEqepSyncSem);

}

static void eqep_code_coverage_enhancement_test(void *args)
{
    int32_t              status;
    EqepAppPatternParams eqepPattern = {0U};
    EQEP_TestParams *testParams = (EQEP_TestParams *)args;
    uint16_t             regVal, intStatus;
    uint32_t             captVal, tmpEqepAPin, tmpEqepBPin, tmpEqepIPin, tmpEqepSPin;

    /* Address translate */
    gEqepBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(testParams->eqepBaseAddr);
    /* Clear Interrupts */
    EQEP_disableInterrupt(gEqepBaseAddr, EQEP_INT_ALL);
    EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_ALL);

    /* Init default EQEP pattern params */
    App_eqepInitPattern(&eqepPattern);

    /* Enable capture from quadrature wave with index event. */
    App_eqepInitQuadratureWave(testParams);

    /* Start EQEP Pattern Generation. */
    eqepPattern.direction        = testParams->eqepDir;
    eqepPattern.generateIdxPulse = TRUE;
    App_eqepGeneratePattern(&eqepPattern);

    App_eqepReadPinValue(&gEqepAPin, &gEqepBPin, &gEqepIPin, &gEqepSPin);
    tmpEqepAPin = gEqepAPin;
    tmpEqepBPin = gEqepBPin;
    tmpEqepIPin = gEqepIPin;
    tmpEqepSPin = gEqepSPin;
    EQEP_setInputPolarity(gEqepBaseAddr, FALSE, FALSE, FALSE, FALSE);
    App_eqepReadPinValue(&gEqepAPin, &gEqepBPin, &gEqepIPin, &gEqepSPin);
    DebugP_assert((tmpEqepAPin | tmpEqepBPin | tmpEqepIPin | tmpEqepSPin) ==
                   (gEqepAPin | gEqepBPin | gEqepIPin | gEqepSPin));
    EQEP_setInputPolarity(gEqepBaseAddr, TRUE, TRUE, TRUE, TRUE);

    status = EQEP_isErrorSet(gEqepBaseAddr);
    DebugP_assert(FALSE == status);

    EQEP_forceInterrupt(gEqepBaseAddr, EQEP_INT_PHASE_ERROR);
    intStatus = EQEP_getInterruptStatus(gEqepBaseAddr);
    DebugP_assert((intStatus & EQEP_INT_PHASE_ERROR) == EQEP_INT_PHASE_ERROR);
    DebugP_assert(TRUE == EQEP_isErrorSet(gEqepBaseAddr));

    EQEP_enableCompare(gEqepBaseAddr);
    regVal = HW_RD_REG16(gEqepBaseAddr + CSL_EQEP_QPOSCTL);
    DebugP_assert((regVal & CSL_EQEP_QPOSCTL_PCE_MASK) == CSL_EQEP_QPOSCTL_PCE_MASK);

    EQEP_disableCompare(gEqepBaseAddr);
    regVal = HW_RD_REG16(gEqepBaseAddr + CSL_EQEP_QPOSCTL);
    DebugP_assert((regVal & CSL_EQEP_QPOSCTL_PCE_MASK) == 0U);

    status = EQEP_setComparePulseWidth(gEqepBaseAddr, CSL_EQEP_QPOSCTL_PCSPW_MASK + 2U);
    DebugP_assert(status == CSL_EBADARGS);    

    status = EQEP_setComparePulseWidth(gEqepBaseAddr, CSL_EQEP_QPOSCTL_PCSPW_MASK);
    regVal = HW_RD_REG16(gEqepBaseAddr + CSL_EQEP_QPOSCTL);
    DebugP_assert((regVal & CSL_EQEP_QPOSCTL_PCSPW_MASK) == CSL_EQEP_QPOSCTL_PCSPW_MASK - 1U); 

    EQEP_setPositionInitMode(gEqepBaseAddr, EQEP_INIT_FALLING_INDEX);
    regVal = HW_RD_REG16(gEqepBaseAddr + CSL_EQEP_QEPCTL);
    DebugP_assert((regVal & CSL_EQEP_QEPCTL_IEI_MASK) == EQEP_INIT_FALLING_INDEX);

    EQEP_setSWPositionInit(gEqepBaseAddr, TRUE);
    regVal = HW_RD_REG16(gEqepBaseAddr + CSL_EQEP_QEPCTL);
    DebugP_assert((regVal & CSL_EQEP_QEPCTL_SWI_MASK) == CSL_EQEP_QEPCTL_SWI_MASK);

    EQEP_setSWPositionInit(gEqepBaseAddr, FALSE);
    regVal = HW_RD_REG16(gEqepBaseAddr + CSL_EQEP_QEPCTL);
    DebugP_assert((regVal & CSL_EQEP_QEPCTL_SWI_MASK) == 0U);

    captVal = EQEP_getStrobePositionLatch(gEqepBaseAddr);
    DebugP_assert(captVal == HW_RD_REG32(gEqepBaseAddr + CSL_EQEP_QPOSSLAT));

    EQEP_setQMAModuleMode(gEqepBaseAddr, EQEP_QMA_MODE_1);
    captVal = HW_RD_REG32(gEqepBaseAddr + CSL_EQEP_QMACTRL);
    DebugP_assert((captVal & CSL_EQEP_QMACTRL_MODE_MASK) == EQEP_QMA_MODE_1);

    EQEP_setStrobeSource(gEqepBaseAddr, EQEP_STROBE_OR_ADCSOCB);
    captVal = HW_RD_REG32(gEqepBaseAddr + CSL_EQEP_QEPSTROBESEL);
    DebugP_assert((captVal & CSL_EQEP_QEPSTROBESEL_STROBESEL_MASK) == EQEP_STROBE_OR_ADCSOCB);

    EQEP_setCompareConfig(gEqepBaseAddr, EQEP_COMPARE_IDX_SYNC_OUT | EQEP_COMPARE_LOAD_ON_MATCH,
                          CSL_EQEP_QPOSCTL_PCSPW_MASK, CSL_EQEP_QPOSCTL_PCSPW_MASK);
    captVal = HW_RD_REG32(gEqepBaseAddr + CSL_EQEP_QPOSCMP);
    regVal = HW_RD_REG16(gEqepBaseAddr + CSL_EQEP_QPOSCTL);
    DebugP_assert((regVal & EQEP_COMPARE_LOAD_ON_MATCH) == EQEP_COMPARE_LOAD_ON_MATCH);
    regVal = HW_RD_REG16(gEqepBaseAddr + CSL_EQEP_QDECCTL_TYPE2);
    DebugP_assert((regVal & EQEP_COMPARE_IDX_SYNC_OUT) == EQEP_COMPARE_IDX_SYNC_OUT);
    regVal = HW_RD_REG16(gEqepBaseAddr + CSL_EQEP_QDECCTL_TYPE2);
    DebugP_assert((regVal & CSL_EQEP_QDECCTL_TYPE2_SPSEL_MASK) == 0U);
}

static void App_eqepIntrISR(void *arg)
{
    uint16_t intEnabled, intStatus;
    uint32_t eqepStatus;

    intStatus = EQEP_getInterruptStatus(gEqepBaseAddr);
    intEnabled = EQEP_getEnabledInterrupt(gEqepBaseAddr);
    if (((intStatus & EQEP_INT_INDEX_EVNT_LATCH) != 0) &&
        ((intEnabled & EQEP_INT_INDEX_EVNT_LATCH) != 0))
    {
        /* Get the position latch count */
        if (gEqepIsrCnt < EQEP_EVENT_CNT)
        {
            gEqepPosCnt[gEqepIsrCnt] = EQEP_getIndexPositionLatch(gEqepBaseAddr);
        }
        if (gEqepIsrCnt == (EQEP_EVENT_CNT - 1))
        {
            eqepStatus = EQEP_getStatus(gEqepBaseAddr);
            //DebugP_assert((eqepStatus & CSL_EQEP_QEPSTS_TYPE1_PCEF_MASK) == 0U);
            EQEP_clearStatus(gEqepBaseAddr, eqepStatus);
            EQEP_disableInterrupt(gEqepBaseAddr, EQEP_INT_ALL);
            SemaphoreP_post(&gEqepSyncSem);
        }
        gEqepIsrCnt++;
    }
    if (((intStatus & EQEP_INT_UNIT_TIME_OUT) != 0) &&
        ((intEnabled & EQEP_INT_UNIT_TIME_OUT) != 0))
    {
        /* Get the position latch count */
        if (gEqepIsrCnt < EQEP_EVENT_CNT)
        {
            gEqepPosCnt[gEqepIsrCnt] = EQEP_getPositionLatch(gEqepBaseAddr);
            gEqepCapPrdLatch[gEqepIsrCnt] = EQEP_getCapturePeriodLatch(gEqepBaseAddr);
            gEqepCapTmrLatch[gEqepIsrCnt] = EQEP_getCaptureTimerLatch(gEqepBaseAddr);
            gEqepCapPrd[gEqepIsrCnt] = EQEP_getCapturePeriod(gEqepBaseAddr);
            gEqepCapTmr[gEqepIsrCnt] = EQEP_getCaptureTimer(gEqepBaseAddr);
        }
        if (gEqepIsrCnt == (EQEP_EVENT_CNT - 1))
        {
            EQEP_disableInterrupt(gEqepBaseAddr, EQEP_INT_ALL);
            SemaphoreP_post(&gEqepSyncSem);
        }
        gEqepIsrCnt++;
    }
    if (((intStatus & EQEP_INT_PHASE_ERROR) != 0) &&
        ((intEnabled & EQEP_INT_PHASE_ERROR) != 0))
    {
        SemaphoreP_post(&gEqepSyncSem);
    }
    if (((intStatus & EQEP_INT_WATCHDOG) != 0) &&
        ((intEnabled & EQEP_INT_WATCHDOG) != 0))
    {
        EQEP_getWatchdogTimerValue(gEqepBaseAddr);
        SemaphoreP_post(&gEqepSyncSem);
    }

    /* Clear EQEP Interrupt. */
    EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_ALL);
}

static void App_eqepInitQuadratureWave(EQEP_TestParams *testParams)
{
    uint32_t getPosCnt;

    EQEP_setInitialPosition(gEqepBaseAddr, 0U);
    EQEP_setPosition(gEqepBaseAddr, 0U);
    getPosCnt = EQEP_getPosition(gEqepBaseAddr);
    DebugP_assert(getPosCnt == 0U);

    /* Configure the decoder for quadrature mode, counting rising edge
      (that is, 1x resolution) */
    EQEP_setDecoderConfig(gEqepBaseAddr, (EQEP_CONFIG_1X_RESOLUTION |
                                        EQEP_CONFIG_QUADRATURE |
                                        EQEP_CONFIG_NO_SWAP));
    EQEP_setEmulationMode(gEqepBaseAddr, EQEP_EMULATIONMODE_RUNFREE);

    /* Configure the position counter to reset on an index event */
    EQEP_setPositionCounterConfig(gEqepBaseAddr, testParams->eqepCntrRstIdxEvt,
                                  CSL_EQEP_QPOSCNT_QPOSCNT_MAX);

    /* Configure the position counter to be latched on rising edge index */
    EQEP_setLatchMode(gEqepBaseAddr, EQEP_LATCH_RISING_INDEX);

    if (testParams->eqepWdgEvtTest == TRUE)
    {
        EQEP_setWatchdogTimerValue(gEqepBaseAddr, 0U);
        EQEP_enableWatchdog(gEqepBaseAddr, CSL_EQEP_QWDPRD_QWDPRD_MAX / 2);
    }

    /* Enable EQEP Module and interrupt */
    EQEP_enableModule(gEqepBaseAddr);
    EQEP_enableInterrupt(gEqepBaseAddr, testParams->eqepIntrEvt);
}

static void App_eqepInitFrequencyCalculation(EQEP_TestParams *testParams)
{
    EQEP_setInitialPosition(gEqepBaseAddr, 0U);
    EQEP_setPosition(gEqepBaseAddr, 0U);

    /* Configure the decoder for up-count mode, counting both rising and
       falling edges (that is, 2x resolution) */
    EQEP_setDecoderConfig(gEqepBaseAddr, (EQEP_CONFIG_2X_RESOLUTION |
                                          EQEP_CONFIG_UP_COUNT |
                                          EQEP_CONFIG_NO_SWAP));
    EQEP_setEmulationMode(gEqepBaseAddr, EQEP_EMULATIONMODE_RUNFREE);

    /* Configure the position counter to reset on an unit timeout event */
    EQEP_setPositionCounterConfig(gEqepBaseAddr,
                                  EQEP_POSITION_RESET_UNIT_TIME_OUT,
                                  CSL_EQEP_QPOSCNT_QPOSCNT_MAX);

    /* Enable the unit timer, setting the frequency to EQEP_UNIT_TIMEOUT_FREQ */
    EQEP_enableUnitTimer(gEqepBaseAddr, (CONFIG_EQEP0_FCLK / (EQEP_UNIT_TIMEOUT_FREQ)));

    /* Configure the position counter to be latched on a unit time out */
    EQEP_setLatchMode(gEqepBaseAddr, EQEP_LATCH_UNIT_TIME_OUT);

    /* Enable the EQEP module */
    EQEP_enableModule(gEqepBaseAddr);

    /* Configure and enable the edge-capture unit. The capture clock divider is
       SYSCLKOUT/128. The unit-position event divider is QCLK/8. */
    EQEP_setCaptureConfig(gEqepBaseAddr, EQEP_CAPTURE_CLK_DIV_128,
                          EQEP_UNIT_POS_EVNT_DIV_8);
    EQEP_enableCapture(gEqepBaseAddr);

    /* Enable unit timeout interrupt. */
    EQEP_enableInterrupt(gEqepBaseAddr, EQEP_INT_UNIT_TIME_OUT);
}

/* This function calculates the average frequency. based on the latched
 * pos values at the unit timeout event. It assumes that the
 * array gEqepPosCnt is filled. */
static uint32_t App_eqepCalculateFrequencyUnitTimeout(void)
{
    uint32_t i;
    uint32_t posCnt;
    uint32_t freq, avgFreq = 0U;

    for (i = 1U; i < EQEP_EVENT_CNT; i++)
    {
        /* posCnt is reset upon the unit timeout event.
           So each value is the diff from prev pos Cnt at unit time out */
        posCnt = gEqepPosCnt[i];

        /* Unit timeout is configured as EQEP_UNIT_TIMEOUT_FREQ Hz.
         * and position count is calculating on both edges. */
        freq = posCnt * (EQEP_UNIT_TIMEOUT_FREQ) / 2;
        /* Note: this is a simplified equation. Boundary condition not taken care. */
        avgFreq += freq;
    }
    /* Ignoring first timeout event position for frequency calculation,
     * as it could contain older value. */
    avgFreq /= (EQEP_EVENT_CNT - 1);

    return avgFreq;
}

static void App_eqepComparePosCnt(int32_t expCnt)
{
    uint32_t i;

    for (i = 0U; i < EQEP_EVENT_CNT; i++)
    {
        if (((gEqepPosCnt[i] - (expCnt - EQEP_POS_CNT_VARIANCE)) *
            (gEqepPosCnt[i] - (expCnt + EQEP_POS_CNT_VARIANCE))) < 0U)
        {
            DebugP_log("Quadrature capture count does not match\r\n");
            DebugP_assert(FALSE);
        }
    }

    return;
}

static void App_eqepInitPattern(EqepAppPatternParams *eqepPattern)
{
    eqepPattern->eqepClockFreq    = EQEP_SIGNAL_TEST_FREQ;
    eqepPattern->direction        = EQEP_DIR_CLOCKWISE;
    eqepPattern->idxEvtCnt        = EQEP_EVENT_CNT;
    eqepPattern->loopCnt          = EQEP_PATTERN_GEN_LOOP_COUNT;
    eqepPattern->generateIdxPulse = TRUE;
}

static void App_eqepTestClockwiseDirection(EQEP_TestParams *testParams)
{
    int32_t              expCnt, eqepDirection;
    EqepAppPatternParams eqepPattern = {0U};

    /* Reset ISR Count */
    gEqepIsrCnt = 0U;
    /* The count values expected is 199
     * Count starts from 0 and counts 4 edges per cycle. */
    expCnt = 199U;

    /* Init default EQEP pattern params */
    App_eqepInitPattern(&eqepPattern);

    /* Enable capture from quadrature wave with index event. */
    App_eqepInitQuadratureWave(testParams);

    /* Start EQEP Pattern Generation. */
    eqepPattern.direction        = testParams->eqepDir;
    eqepPattern.generateIdxPulse = TRUE;
    App_eqepGeneratePattern(&eqepPattern);

    /* Wait for the EQEP_EVENT_CNT number of EQEP Index Latch interrupt. */
    SemaphoreP_pend (&gEqepSyncSem, SystemP_WAIT_FOREVER);

    eqepDirection = EQEP_getDirection(gEqepBaseAddr);
    DebugP_assert(eqepDirection == eqepDirection);

    /* Check position count. */
    App_eqepComparePosCnt(expCnt);
}

static void App_eqepTestAntiClockwiseDirection(EQEP_TestParams *testParams)
{
    int32_t              expCnt, eqepDirection;
    EqepAppPatternParams eqepPattern = {0U};

    /* Reset ISR Count */
    gEqepIsrCnt = 0U;
    /* The count values expected is -199
     * Count starts from 0 and counts 4 edges per cycle. */
    expCnt = -199U;

    /* Init default EQEP pattern params */
    App_eqepInitPattern(&eqepPattern);

    /* Enable capture from quadrature wave with index event. */
    App_eqepInitQuadratureWave(testParams);

    /* Start EQEP Pattern Generation. */
    eqepPattern.direction        = testParams->eqepDir;
    eqepPattern.generateIdxPulse = TRUE;
    App_eqepGeneratePattern(&eqepPattern);

    /* Wait for the EQEP_EVENT_CNT number of EQEP Index Latch interrupt. */
    SemaphoreP_pend (&gEqepSyncSem, SystemP_WAIT_FOREVER);

    eqepDirection = EQEP_getDirection(gEqepBaseAddr);
    DebugP_assert(eqepDirection == eqepDirection);

    /* Check position count. */
    App_eqepComparePosCnt(expCnt);
}

static void App_eqepTestFrequency(EQEP_TestParams *testParams)
{
    uint32_t             avgFreq = 0U;
    EqepAppPatternParams eqepPattern = {0U};

    /* Reset ISR Count */
    gEqepIsrCnt = 0U;

    /* Init default EQEP pattern params */
    App_eqepInitPattern(&eqepPattern);

    /* Enable Frequency Calculation. */
    App_eqepInitFrequencyCalculation(testParams);

    /* Start EQEP Pattern Generation. */
    eqepPattern.direction        = testParams->eqepDir;
    eqepPattern.generateIdxPulse = FALSE;
    App_eqepGeneratePattern(&eqepPattern);

    /* Wait for the Unit Timeout interrupt. */
    SemaphoreP_pend(&gEqepSyncSem, SystemP_WAIT_FOREVER);

    avgFreq = App_eqepCalculateFrequencyUnitTimeout();
    DebugP_assert(avgFreq == EQEP_SIGNAL_TEST_FREQ);

    DebugP_log("Expected Frequency is %d Hz\r\n", EQEP_SIGNAL_TEST_FREQ);
    DebugP_log("Average frequency is %d Hz\r\n", avgFreq);
}

static void test_eqep_init_test_params(EQEP_TestParams *testParams,
                                           uint32_t testCaseId)
{
    /* Default Initialization */
    testParams->eqepDir = EQEP_DIR_CLOCKWISE;
    testParams->eqepFreqMeas = FALSE;
    testParams->eqepCntrRstIdxEvt = EQEP_POSITION_RESET_IDX;
    testParams->eqepIntrEvt = EQEP_INT_INDEX_EVNT_LATCH;
    testParams->eqepWdgEvtTest = FALSE;
    testParams->eqepIntrNum = CONFIG_EQEP0_INTR;
    testParams->eqepBaseAddr = CONFIG_EQEP0_BASE_ADDR;

    switch (testCaseId)
    {
        case 1548:
            testParams->eqepDir = EQEP_DIR_ANTI_CLOCKWISE;
            break;
        case 1549:
            testParams->eqepFreqMeas = TRUE;
            break;
        case 1550:
            testParams->eqepCntrRstIdxEvt = EQEP_POSITION_RESET_1ST_IDX;
            break;
        case 1551:
            testParams->eqepCntrRstIdxEvt = EQEP_POSITION_RESET_MAX_POS;
            break;
        case 1554:
            testParams->eqepIntrEvt = EQEP_INT_PHASE_ERROR;
            break;
        case 1555:
            testParams->eqepIntrEvt = EQEP_INT_WATCHDOG;
            testParams->eqepWdgEvtTest = TRUE;
            testParams->eqepIntrNum = CONFIG_EQEP1_INTR;
            testParams->eqepBaseAddr = CONFIG_EQEP1_BASE_ADDR;
            break;
        case 1556:
            testParams->eqepIntrEvt = EQEP_INT_WATCHDOG;
            testParams->eqepWdgEvtTest = TRUE;
            testParams->eqepIntrNum = CONFIG_EQEP2_INTR;
            testParams->eqepBaseAddr = CONFIG_EQEP2_BASE_ADDR;
            break;
    }
}
