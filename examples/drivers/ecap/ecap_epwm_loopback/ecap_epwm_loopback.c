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
 *  This example demonstrates ePWM to eCAP loopback test.
 *  The ecap module is configured in the capture mode and the ecap device pin is
 *  configured as input pin. A square wave needs to be fed to the ecap pin
 *  externally. Based on the internal counter the count values for each of the
 *  edge is latched in register. 4th edge will generate the interrupt. Based on
 *  the latched counter values calculates the input signal frequency and the
 *  duty cycle based on the input functional clock frequency to ecap module.
 *  ePWM is configured to generate a square wave with 25% duty cycle.
 *  Connect the ePWM output to eCAP input externally on the board.
 */

#include <math.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/ecap.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ECAP Interrupt Sources */
#define ECAP_INT_ALL                    (ECAP_CEVT1_INT  | \
                                         ECAP_CEVT2_INT  | \
                                         ECAP_CEVT3_INT  | \
                                         ECAP_CEVT4_INT  | \
                                         ECAP_CNTOVF_INT | \
                                         ECAP_PRDEQ_INT  | \
                                         ECAP_CMPEQ_INT)
/* ECAP Frequency MHz */
#define ECAP_INPUT_FREQ_MHZ             (CONFIG_ECAP0_FCLK / (1000U * 1000U))
/* Output channel - A or B */
#define APP_EPWM_OUTPUT_CH              (EPWM_OUTPUT_CH_A)
/* Duty Cycle of PWM output signal in % - give value from 0 to 100 */
#define APP_EPWM_DUTY_CYCLE             (25U)
/* Frequency of PWM output signal in Hz - 1 KHz is selected */
#define APP_EPWM_OUTPUT_FREQ            (1U * 1000U)

/* TB frequency in Hz - so that /4 divider is used */
#define APP_EPWM_TB_FREQ                (CONFIG_EPWM0_FCLK / 4U)
/*
 *  PRD value - this determines the period
 *  PRD = (TBCLK/PWM FREQ) / 2
 *  /2 is added becasue up&down counter is selected. So period is 2 times
 */
#define APP_EPWM_PRD_VAL                ((APP_EPWM_TB_FREQ / \
                                          APP_EPWM_OUTPUT_FREQ) / 2U)
/*
 *  COMPA value - this determines the duty cycle
 *  COMPA = (PRD - ((dutycycle * PRD) / 100)
 */
#define APP_EPWM_COMPA_VAL              (APP_EPWM_PRD_VAL - \
                                        ((APP_EPWM_DUTY_CYCLE * \
                                          APP_EPWM_PRD_VAL) / 100U))
/* Capture iteration count */
#define APP_ECAP_CAPTURE_LOOP_COUNT     (5U)

/* Global variables and objects */
static HwiP_Object       gEcapHwiObject;
static SemaphoreP_Object gEcapSyncSemObject;
/* Variable to hold base address of EPWM/ECAP that is used */
uint32_t gEcapBaseAddr, gEpwmBaseAddr;

/* Static Function declarations */
static void App_ecapIntrISR(void *arg);
static void App_epwmInit(void);
static void App_ecapInit(void);
static void App_ecapCompareOutput(double dutyCycle, double actualOpFreq);

void ecap_epwm_loopback_main(void *args)
{
    int32_t             status;
    HwiP_Params         hwiPrms;
    uint32_t            loopCnt = APP_ECAP_CAPTURE_LOOP_COUNT;
    uint32_t            cap1Count, cap2Count, cap3Count, cap4Count;
    double              highTime, lowTime, dutyCycle, actualOpFreq;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM to ECAP loopback application started...\r\n");
    DebugP_log("Please refer EXAMPLES_DRIVERS_ECAP_EPWM_LOOPBACK example user \
guide for the test setup details. \r\n");

    /* Address translate */
    gEcapBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_ECAP0_BASE_ADDR);
    gEpwmBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);

    status = SemaphoreP_constructCounting(&gEcapSyncSemObject, 0, loopCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_ECAP0_INTR;
    hwiPrms.callback    = &App_ecapIntrISR;
    hwiPrms.isPulse     = CONFIG_ECAP0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEcapHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Initialize ECAP and EPWM */
    App_epwmInit();
    App_ecapInit();

    /* Start Capture for APP_ECAP_CAPTURE_LOOP_COUNT iterations */
    while(loopCnt > 0)
    {
        ECAP_oneShotReArm(gEcapBaseAddr);
        SemaphoreP_pend(&gEcapSyncSemObject, SystemP_WAIT_FOREVER);
        loopCnt--;
    }

    /* Clear any pending interrupts if any */
    EPWM_etIntrDisable(gEpwmBaseAddr);
    EPWM_etIntrClear(gEpwmBaseAddr);
    ECAP_intrDisable(gEcapBaseAddr, ECAP_INT_ALL);

    /* Read Counter values and print for last iteration. */
    cap1Count = ECAP_timeStampRead(gEcapBaseAddr, ECAP_CAPTURE_EVENT_1);
    cap2Count = ECAP_timeStampRead(gEcapBaseAddr, ECAP_CAPTURE_EVENT_2);
    cap3Count = ECAP_timeStampRead(gEcapBaseAddr, ECAP_CAPTURE_EVENT_3);
    cap4Count = ECAP_timeStampRead(gEcapBaseAddr, ECAP_CAPTURE_EVENT_4);

    DebugP_log("Count1 = %u, Count2 = %u, Count3 = %u, Count4 = %u\r\n",
               cap1Count, cap2Count, cap3Count, cap4Count);

    highTime = (double)cap2Count / (double)ECAP_INPUT_FREQ_MHZ;
    lowTime  = (double)cap3Count / (double)ECAP_INPUT_FREQ_MHZ;
    dutyCycle = ((double)cap2Count * (double)100) /
                    ((double)cap2Count + (double)cap3Count);
    actualOpFreq = ((1000 * 1000) / (highTime + lowTime));

    DebugP_log("Hight time is %.lf us, Low time is %.lf us\r\n",
    trunc(round(highTime)), trunc(round(lowTime)));

    DebugP_log("Expected DutyCycle %u%%, Actual DutyCycle %.lf%%\r\n",
    APP_EPWM_DUTY_CYCLE, trunc(round(dutyCycle)));
    DebugP_log("Expected Output Frequency %uKHz, Actual Output Frequency %.lfKHz\r\n",
    APP_EPWM_OUTPUT_FREQ / 1000, trunc(actualOpFreq / 1000));

    HwiP_destruct(&gEcapHwiObject);
    SemaphoreP_destruct(&gEcapSyncSemObject);

    DebugP_log("All tests have passed.\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_ecapIntrISR(void *arg)
{
    /* Clear Ecap Interrupt. */
    ECAP_intrStatusClear(gEcapBaseAddr, ECAP_INT_ALL);
    /* Clear Global Interrupt Flag. */
    ECAP_globalIntrClear(gEcapBaseAddr);

    SemaphoreP_post(&gEcapSyncSemObject);
}

static void App_ecapInit(void)
{
    /* Disable and Clear Interrupts */
    ECAP_intrDisable(gEcapBaseAddr, ECAP_INT_ALL);
    ECAP_intrStatusClear(gEcapBaseAddr, ECAP_INT_ALL);
    
#if defined(SOC_AM273X)   
    /* Capture input source select */
    ECAP_captureInputSourceSelect(gEcapBaseAddr,ECAP_CAPTURE_INPUT_SOURCE_SELECT_0);
#endif
    /* Disable CAP1-CAP4 register loads */
    ECAP_captureLoadingDisable(gEcapBaseAddr);

    /* Configure eCAP */
    ECAP_counterControl(gEcapBaseAddr, ECAP_COUNTER_STOP);
    /* Enable capture mode */
    ECAP_operatingModeSelect(gEcapBaseAddr, ECAP_CAPTURE_MODE);

    /* One shot mode, stop capture at event 4 */
    ECAP_oneShotModeConfig(gEcapBaseAddr, ECAP_CAPTURE_EVENT4_STOP);

    /* Set polarity of the events to rising, falling, rising, falling edge */
    ECAP_captureEvtPolarityConfig(gEcapBaseAddr,
                                 ECAP_CAPTURE_EVENT_RISING,
                                 ECAP_CAPTURE_EVENT_FALLING,
                                 ECAP_CAPTURE_EVENT_RISING,
                                 ECAP_CAPTURE_EVENT_FALLING);

    /* Set capture in time difference mode */
    ECAP_captureEvtCntrRstConfig(gEcapBaseAddr,
                                ECAP_CAPTURE_EVENT_RESET_COUNTER_RESET,
                                ECAP_CAPTURE_EVENT_RESET_COUNTER_RESET,
                                ECAP_CAPTURE_EVENT_RESET_COUNTER_RESET,
                                ECAP_CAPTURE_EVENT_RESET_COUNTER_RESET);

    ECAP_syncInOutSelect(gEcapBaseAddr, ECAP_ENABLE_COUNTER, ECAP_SYNC_IN);
    ECAP_counterControl(gEcapBaseAddr, ECAP_COUNTER_FREE_RUNNING);

    /* Enable eCAP module */
    ECAP_captureLoadingEnable(gEcapBaseAddr);
    /* Enable interrupt */
    ECAP_intrEnable(gEcapBaseAddr, ECAP_CEVT4_INT);
}

/* Configure EPWM0 Output Channel A to 1KHZ and 25% DutyCycle */
static void App_epwmInit(void)
{
    EPWM_AqActionCfg  aqConfig;

    /* Configure Time base submodule */
    EPWM_tbTimebaseClkCfg(gEpwmBaseAddr, APP_EPWM_TB_FREQ, CONFIG_EPWM0_FCLK);
    EPWM_tbPwmFreqCfg(gEpwmBaseAddr, APP_EPWM_TB_FREQ, APP_EPWM_OUTPUT_FREQ,
                      EPWM_TB_COUNTER_DIR_UP_DOWN, EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(gEpwmBaseAddr);
    EPWM_tbSetSyncOutMode(gEpwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_SYNCIN);
    EPWM_tbSetEmulationMode(gEpwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(gEpwmBaseAddr, EPWM_CC_CMP_A,
            APP_EPWM_COMPA_VAL, EPWM_SHADOW_REG_CTRL_ENABLE,
            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
    EPWM_counterComparatorCfg(gEpwmBaseAddr, EPWM_CC_CMP_B,
            APP_EPWM_COMPA_VAL, EPWM_SHADOW_REG_CTRL_ENABLE,
            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);

    /* Configure Action Qualifier Submodule */
    aqConfig.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.prdAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    aqConfig.cmpADownAction = EPWM_AQ_ACTION_LOW;
    aqConfig.cmpBUpAction = EPWM_AQ_ACTION_HIGH;
    aqConfig.cmpBDownAction = EPWM_AQ_ACTION_LOW;
    EPWM_aqActionOnOutputCfg(gEpwmBaseAddr, APP_EPWM_OUTPUT_CH, &aqConfig);

    /* Configure Dead Band Submodule */
    EPWM_deadbandBypass(gEpwmBaseAddr);

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(gEpwmBaseAddr, FALSE);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(gEpwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(gEpwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    /* Configure event trigger Submodule */
    EPWM_etIntrCfg(gEpwmBaseAddr, EPWM_ET_INTR_EVT_CNT_EQ_ZRO,
        EPWM_ET_INTR_PERIOD_FIRST_EVT);
    EPWM_etIntrEnable(gEpwmBaseAddr);
}
