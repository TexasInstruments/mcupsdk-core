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
#include <drivers/eqep.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "eqep_pattern_gen.h"

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

/* Static Function declarations */
static void App_eqepIntrISR(void *arg);
static void App_eqepInitQuadratureWave(void);
static void App_eqepInitFrequencyCalculation(void);
static void App_eqepInitPattern(EqepAppPatternParams *eqepPattern);
static void App_eqepComparePosCnt(int32_t expCnt);
static void App_eqepTestClockwiseDirection(void);
static void App_eqepTestAntiClockwiseDirection(void);
static void App_eqepTestFrequency(void);
static uint32_t App_eqepCalculateFrequencyUnitTimeout(void);

void eqep_capture_main(void *args)
{
    int32_t              status;
    HwiP_Params          hwiPrms;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EQEP Capture application started...\r\n");
    DebugP_log("Please refer EXAMPLES_DRIVERS_EQEP_CAPTURE example user \
guide for the test setup details.\r\n");

    status = SemaphoreP_constructBinary(&gEqepSyncSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_EQEP0_INTR;
    hwiPrms.callback    = &App_eqepIntrISR;
    hwiPrms.isPulse     = CONFIG_EQEP0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEqepHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Address translate */
    gEqepBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EQEP0_BASE_ADDR);
    /* Clear Interrupts */
    EQEP_disableInterrupt(gEqepBaseAddr, EQEP_INT_ALL);
    EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_ALL);

    DebugP_log("Sending quadrature wave for 50 cycles in clockwise direction.\
With index event in between, Captures 4 edges per cycle\r\n");

    /* Test Clockwise Direction */
    App_eqepTestClockwiseDirection();

    DebugP_log("Quadrature input capture test clockwise direction passed\r\n");

    DebugP_log("Sending quadrature wave for 50 cycles in anticlockwise direction.\
With index event in between, Captures 4 edges per cycle\r\n");

    /* Test Anti Clockwise Direction */
    App_eqepTestAntiClockwiseDirection();

    DebugP_log("Quadrature input capture test anti clockwise direction passed\r\n");

    DebugP_log("Starting Frequency calculation test\r\n");

    /* Frequency Calculation Test */
    App_eqepTestFrequency();

    DebugP_log("Frequency calculation test passed\r\n");

    HwiP_destruct(&gEqepHwiObject);
    SemaphoreP_destruct(&gEqepSyncSem);

    DebugP_log("All tests have passed.\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_eqepIntrISR(void *arg)
{
    uint16_t intEnabled, intStatus;

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
        }
        if (gEqepIsrCnt == (EQEP_EVENT_CNT - 1))
        {
            EQEP_disableInterrupt(gEqepBaseAddr, EQEP_INT_ALL);
            SemaphoreP_post(&gEqepSyncSem);
        }
        gEqepIsrCnt++;
    }

    /* Clear EQEP Interrupt. */
    EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_ALL);
}

static void App_eqepInitQuadratureWave(void)
{
    EQEP_setInitialPosition(gEqepBaseAddr, 0U);
    EQEP_setPosition(gEqepBaseAddr, 0U);

    /* Configure the decoder for quadrature mode, counting rising edge
      (that is, 1x resolution) */
    EQEP_setDecoderConfig(gEqepBaseAddr, (EQEP_CONFIG_1X_RESOLUTION |
                                        EQEP_CONFIG_QUADRATURE |
                                        EQEP_CONFIG_NO_SWAP));
    EQEP_setEmulationMode(gEqepBaseAddr, EQEP_EMULATIONMODE_RUNFREE);

    /* Configure the position counter to reset on an index event */
    EQEP_setPositionCounterConfig(gEqepBaseAddr, EQEP_POSITION_RESET_IDX,
                                  CSL_EQEP_QPOSCNT_QPOSCNT_MAX);

    /* Configure the position counter to be latched on rising edge index */
    EQEP_setLatchMode(gEqepBaseAddr, EQEP_LATCH_RISING_INDEX);

    /* Enable EQEP Module and interrupt */
    EQEP_enableModule(gEqepBaseAddr);
    EQEP_enableInterrupt(gEqepBaseAddr, EQEP_INT_INDEX_EVNT_LATCH);
}

static void App_eqepInitFrequencyCalculation(void)
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
            (gEqepPosCnt[i] - (expCnt + EQEP_POS_CNT_VARIANCE))) <= 0U)
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

static void App_eqepTestClockwiseDirection(void)
{
    int32_t              expCnt;
    EqepAppPatternParams eqepPattern = {0U};

    /* Reset ISR Count */
    gEqepIsrCnt = 0U;
    /* The count values expected is 199
     * Count starts from 0 and counts 4 edges per cycle. */
    expCnt = 199U;

    /* Init default EQEP pattern params */
    App_eqepInitPattern(&eqepPattern);

    /* Enable capture from quadrature wave with index event. */
    App_eqepInitQuadratureWave();

    /* Start EQEP Pattern Generation. */
    eqepPattern.direction        = EQEP_DIR_CLOCKWISE;
    eqepPattern.generateIdxPulse = TRUE;
    App_eqepGeneratePattern(&eqepPattern);

    /* Wait for the EQEP_EVENT_CNT number of EQEP Index Latch interrupt. */
    SemaphoreP_pend (&gEqepSyncSem, SystemP_WAIT_FOREVER);

    /* Check position count. */
    App_eqepComparePosCnt(expCnt);
}

static void App_eqepTestAntiClockwiseDirection(void)
{
    int32_t              expCnt;
    EqepAppPatternParams eqepPattern = {0U};

    /* Reset ISR Count */
    gEqepIsrCnt = 0U;
    /* The count values expected is -199
     * Count starts from 0 and counts 4 edges per cycle. */
    expCnt = -199U;

    /* Init default EQEP pattern params */
    App_eqepInitPattern(&eqepPattern);

    /* Enable capture from quadrature wave with index event. */
    App_eqepInitQuadratureWave();

    /* Start EQEP Pattern Generation. */
    eqepPattern.direction        = EQEP_DIR_ANTI_CLOCKWISE;
    eqepPattern.generateIdxPulse = TRUE;
    App_eqepGeneratePattern(&eqepPattern);

    /* Wait for the EQEP_EVENT_CNT number of EQEP Index Latch interrupt. */
    SemaphoreP_pend (&gEqepSyncSem, SystemP_WAIT_FOREVER);

    /* Check position count. */
    App_eqepComparePosCnt(expCnt);
}

static void App_eqepTestFrequency(void)
{
    uint32_t             avgFreq = 0U;
    EqepAppPatternParams eqepPattern = {0U};

    /* Reset ISR Count */
    gEqepIsrCnt = 0U;

    /* Init default EQEP pattern params */
    App_eqepInitPattern(&eqepPattern);

    /* Enable Frequency Calculation. */
    App_eqepInitFrequencyCalculation();

    /* Start EQEP Pattern Generation. */
    eqepPattern.direction        = EQEP_DIR_CLOCKWISE;
    eqepPattern.generateIdxPulse = FALSE;
    App_eqepGeneratePattern(&eqepPattern);

    /* Wait for the Unit Timeout interrupt. */
    SemaphoreP_pend(&gEqepSyncSem, SystemP_WAIT_FOREVER);

    avgFreq = App_eqepCalculateFrequencyUnitTimeout();
    DebugP_assert(avgFreq == EQEP_SIGNAL_TEST_FREQ);

    DebugP_log("Expected Frequency is %d Hz\r\n", EQEP_SIGNAL_TEST_FREQ);
    DebugP_log("Average frequency is %d Hz\r\n", avgFreq);
}
