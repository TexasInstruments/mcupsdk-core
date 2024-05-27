/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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

//
// Included Files
//
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 *
 *
 * ECAP Signal Monitoring
 * ----------------------
 * Each ECAP instance is equipped with 2 signal monitoring units.
 * These signal monitoring units can be configured in one of the 4 following
 * modes.
 *      1. High Pulse Monitoring Mode -> Rising to Falling
 *      2. Low Pulse Monitoring Mode  -> Falling to Rising
 *      3. Period Monitoring, Rise edge to Rise edge -> Rising to Rising
 *      4. Period Monitoring, Fall edge to Fall edge -> Falling to Falling.
 *      5. Edge Monitoring ( Posedge ) -> Sync in is mandatory for this mode
 *      5. Edge Monitoring ( Negedge ) -> Sync in is mandatory for this mode
 *
 * For each mode, the CAPEVTs and Stop/Wrap are configured respectively.
 * The Input signal is captured and monitored between Min and Max registers
 * against the Counter value captured with the CAPEVT for the respective mode.
 * Also, these Min and Max registers are 32bit values contain Active-Shadow set.
 * The Shadow to Active loading can be selected to be triggered by either...
 *      - a Sync event or
 *      - a Global Load Strobe event.
 *
 * Debug Mode is present, for when enabled, the events occuring in
 * the min-max window ranges, the minimum and maximum values within
 * the noted min-max window are captured and stored per monitoring unit.
 *
 * Note
 * ----
 *      1. The ECAP should be in Absolute mode where no CAPEVT reset the counters
 *         for the signal monitoring to be working.
 *
 *      2. For Edge Monitoring, there needs to be a Sync pulse from an EPWM to
 *         restart the monitoring.
 *
 *      3. For Pulse or Peroid Monitoring there should not be a Sync in enabled.
 *
 *      4. The error1 and error2 are used for the min window violation error and
 *         the max window violation error in the pulse/ period monitoring.
 *         Once these are set, in order to restart the monitoring, one needs to
 *         re-enable the monitoring unit and start timestamping on ecap.
 *
 *      5. On the Edge monitoring modes, the error1 is set for min or max window
 *         violation. the error2 is set if there is never an edge between the two
 *         sync events. For every sync pulse, the edge monitoring restarts.
 *
 *
 * Example Description
 * -------------------
 *      This example configures 6 ECAP moudule, with a total of 6 signal
 * monitoring modules enabled, each with one configuration that are supported.
 * All ECAPs are enabled with Interrupts for the Errors from the Signal Monitoring
 * Modules. An EPWM is configured to provide the input, routed to ECAP via
 * GPI and Inputxbar. This input is varied in a way to cross min and max values from
 * the siganl monitoring modules and the application checks for the interrupt flags
 * are well as the debug range values.
 *
 * External connections
 * --------------------
 *      This example doesn't need any external connections, as it uses internal
 * GPI to read the PWM signal generated. If user wants to use an external signal,
 * please configure a GPI and route it via INPUTxBar to the ECAP module.
 *
*/


/* Defines */
#define TEST_STATE_PULSE (0)
#define TEST_STATE_PERIOD (1)
#define TEST_STATE_EDGE (2)
#define TEST_STATE_COMPLETE (3)

#define PWM_INIT_ON_EDGE_TIME (4999U)
#define PWM_INIT_OFF_EDGE_TIME (14999U)
#define PWM_INIT_PERIOD (19999U)

#define PWM_INIT_ON_TIME (PWM_INIT_OFF_EDGE_TIME - PWM_INIT_ON_EDGE_TIME)
#define PWM_INIT_OFF_TIME (PWM_INIT_PERIOD - PWM_INIT_ON_TIME)

#define MON_PULSE_HIGH_MIN_BOUND (PWM_INIT_ON_TIME - 50U)
#define MON_PULSE_HIGH_MAX_BOUND (PWM_INIT_ON_TIME + 50U)

#define MON_PULSE_LOW_MIN_BOUND (PWM_INIT_OFF_TIME - 50U)
#define MON_PULSE_LOW_MAX_BOUND (PWM_INIT_OFF_TIME + 50U)

#define MON_PERIOD_MIN_BOUND (PWM_INIT_PERIOD - 50U)
#define MON_PERIOD_MAX_BOUND (PWM_INIT_PERIOD + 50U)

#define MON_POSEDGE_MIN_BOUND (PWM_INIT_ON_EDGE_TIME - 100U)
#define MON_POSEDGE_MAX_BOUND (PWM_INIT_ON_EDGE_TIME + 100U)

#define MON_NEGEDGE_MIN_BOUND (PWM_INIT_OFF_EDGE_TIME - 100U)
#define MON_NEGEDGE_MAX_BOUND (PWM_INIT_OFF_EDGE_TIME + 100U)

/* Global Variables */
volatile uint16_t gPwm_onEdgeTime = PWM_INIT_ON_EDGE_TIME;
volatile uint16_t gPwm_offEdgeTime = PWM_INIT_OFF_EDGE_TIME;
volatile uint16_t gPwm_period = PWM_INIT_PERIOD;

uint8_t gPwm_updateFlag = FALSE;
uint8_t testCase = TEST_STATE_PULSE;

static SemaphoreP_Object  gEpwmSyncSemObject;

int32_t             status;

static HwiP_Object  gEpwmHwiObject_1;
static HwiP_Object  gEcapHwiObject_1;

volatile uint32_t gHighPulseMonErrorStatus   = 0;
volatile uint32_t gLowPulseMonErrorStatus    = 0;
volatile uint32_t gHighPeriodMonErrorStatus  = 0;
volatile uint32_t gLowPeriodMonErrorStatus   = 0;
volatile uint32_t gPosedgeMonErrorStatus     = 0;
volatile uint32_t gNegedgeMonErrorStatus     = 0;

volatile uint16_t current_on_edge_time = PWM_INIT_ON_EDGE_TIME;
volatile uint16_t current_off_edge_time = PWM_INIT_OFF_EDGE_TIME;
volatile uint16_t current_period = PWM_INIT_PERIOD;
/*
ECAP base addresses :
    MON_PULSE_HIGH_BASE_ADDR
    MON_PULSE_LOW_BASE_ADDR
    MON_PERIOD_HIGH_BASE_ADDR
    MON_PERIOD_LOW_BASE_ADDR
    MON_POSEDGE_BASE_ADDR
    MON_NEGEDGE_BASE_ADDR
 */


/* Function Prototypes */
/* Common ISR for the Signal Monitoring errors from All the ECAPs  */

/* ISR to update the EPWM Signal controllably */
void App_registerInterrupt(void);
static void App_ecapIsr(void* args);

/* Function to update the input by steps */
void App_updateEpwmInput(uint16_t onTime, uint16_t offTime, uint16_t period);
static void App_epwmUpdateIsr(void* args);

/*  */
void App_restartMonitoring(uint32_t base);


void ecap_signal_monitoring_main(void *args)
{

    Drivers_open();
    Board_driversOpen();

    uint8_t testErrors = 0;
    status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);

    App_registerInterrupt();

    DebugP_log("ECAP Signal Monitoring Test Test Started ...\r\n");
    DebugP_log("Currently the Input waveform is having all the properties in expected state\r\n\r\n");

    EPWM_clearEventTriggerInterruptFlag(INPUT_EPWM_BASE_ADDR);

    while(testCase != TEST_STATE_COMPLETE)
    {
        gPwm_updateFlag = FALSE;
        switch(testCase)
        {
            case TEST_STATE_PULSE :
            {
                /*
                Increasing the High Pulse Width more than Max bounds.
                this should trigger
                    1. the High Pulse width High error and
                    2. the Low Pulse Width Low error
                */

                /* Pulse or Period monitoring, once passed an error, will be disabled immediately. If we want to restart the monitoring, please uncomment the following */
                // App_restartMonitoring(MON_PULSE_HIGH_BASE_ADDR);
                // App_restartMonitoring(MON_PULSE_LOW_BASE_ADDR);

                /* Update the input to cross bounds */
                App_updateEpwmInput(PWM_INIT_ON_EDGE_TIME - 50, PWM_INIT_OFF_EDGE_TIME + 50, PWM_INIT_PERIOD);

                DebugP_log("Pulse Width is modified to exceed bounds\r\n");
                DebugP_log("----------------------------------------\r\n");
                DebugP_log("\tHIGH PULSE Min Bound : %u\tHIGH PULSE Max Bound : %u\tConfigured High Pulse Width : %u\r\n\tObserved Min Width : %u\tObserved Max Width : %u\r\n\r\n",
                            MON_PULSE_HIGH_MIN_BOUND, MON_PULSE_HIGH_MAX_BOUND, gPwm_offEdgeTime - gPwm_onEdgeTime,
                            ECAP_observedMinValue(MON_PULSE_HIGH_BASE_ADDR, ECAP_MONITORING_UNIT_1),
                            ECAP_observedMaxValue(MON_PULSE_HIGH_BASE_ADDR, ECAP_MONITORING_UNIT_1));

                DebugP_log("\tLOW PULSE Min Bound  : %u\tLOW PULSE Max Bound  : %u\tConfigured low Pulse Width  : %u\r\n\tObserved Min Width : %u\tObserved Max Width : %u\r\n\r\n",
                            MON_PULSE_HIGH_MIN_BOUND, MON_PULSE_HIGH_MAX_BOUND, gPwm_period - (gPwm_offEdgeTime - gPwm_onEdgeTime),
                            ECAP_observedMinValue(MON_PULSE_LOW_BASE_ADDR, ECAP_MONITORING_UNIT_1),
                            ECAP_observedMaxValue(MON_PULSE_LOW_BASE_ADDR, ECAP_MONITORING_UNIT_1));

                /* Checking for the error flags expected for High Pulse and Low Pulse monitors */
                if( ((gHighPulseMonErrorStatus & ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2) != ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2)
                    ||
                    ((gLowPulseMonErrorStatus & ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1) != ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1))
                {
                    /* test fail scenario */
                    testErrors++;
                    testCase = TEST_STATE_COMPLETE;
                }
                else
                {
                    testCase = TEST_STATE_EDGE;
                }

                /* Return to normal PWM wave */
                App_updateEpwmInput(PWM_INIT_ON_EDGE_TIME, PWM_INIT_OFF_EDGE_TIME, PWM_INIT_PERIOD);

                break;
            }
            case TEST_STATE_EDGE :
            {
                /*
                Increasing the Posedge and Negedge more than Max bounds.
                this should trigger
                    1. the Posedge error and
                    2. the Negedge error
                Note:
                    The error1 occurs when the edge doesn't within Min-Max window.
                    The error2 occurs if there is no edge between 2 sync events.
                    Sync event restarts the Monitoring.
                */

                /* Edge Monitoring will automatically restart if there is a sync in coming. */

                App_updateEpwmInput(PWM_INIT_ON_EDGE_TIME - 150, PWM_INIT_OFF_EDGE_TIME + 150, PWM_INIT_PERIOD);

                DebugP_log("Edge Position is modified to exceed bounds\r\n");
                DebugP_log("------------------------------------------\r\n");
                DebugP_log("\tPosedge Min Bound : %u\tPosedge Max Bound : %u\tConfigured Posedge Position : %u\r\n\tObserved Min Value : %u\tObserved Max Value : %u\r\n\r\n",
                            MON_POSEDGE_MIN_BOUND, MON_POSEDGE_MAX_BOUND, gPwm_onEdgeTime,
                            ECAP_observedMinValue(MON_POSEDGE_BASE_ADDR, ECAP_MONITORING_UNIT_1),
                            ECAP_observedMaxValue(MON_POSEDGE_BASE_ADDR, ECAP_MONITORING_UNIT_1));

                DebugP_log("\tNegedge Min Bound : %u\tNegedge Max Bound : %u\tConfigured Negedge Position : %u\r\n\tObserved Min Value : %u\tObserved Max Value : %d\r\n\r\n",
                            MON_NEGEDGE_MIN_BOUND, MON_NEGEDGE_MAX_BOUND, gPwm_offEdgeTime,
                            ECAP_observedMinValue(MON_NEGEDGE_BASE_ADDR, ECAP_MONITORING_UNIT_1),
                            ECAP_observedMaxValue(MON_NEGEDGE_BASE_ADDR, ECAP_MONITORING_UNIT_1));

                /* Checking for the error flags expected for High Pulse and Low Pulse monitors */
                if( ((gPosedgeMonErrorStatus & ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1) != ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1)
                    ||
                    ((gNegedgeMonErrorStatus & ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1) != ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1))
                {
                    /* test fail scenario */
                    testErrors++;
                    testCase = TEST_STATE_COMPLETE;
                }
                else
                {
                    testCase = TEST_STATE_PERIOD;
                }

                /* Return to normal PWM wave */
                App_updateEpwmInput(PWM_INIT_ON_EDGE_TIME, PWM_INIT_OFF_EDGE_TIME, PWM_INIT_PERIOD);

                break;
            }
            case TEST_STATE_PERIOD :
            {
                /*
                Increasing the Period more than Max bounds.
                this should trigger Both Period monitors High error
                */

                /* Pulse or Period monitoring, once passed an error, will be disabled immediately. If we want to restart the monitoring, please uncomment the following */
                // App_restartMonitoring(MON_PERIOD_HIGH_BASE_ADDR);
                // App_restartMonitoring(MON_PERIOD_LOW_BASE_ADDR);

                App_updateEpwmInput(PWM_INIT_ON_EDGE_TIME, PWM_INIT_OFF_EDGE_TIME, PWM_INIT_PERIOD + 150);

                DebugP_log("Period is modified to exceed bounds\r\n");
                DebugP_log("-----------------------------------\r\n");
                DebugP_log("\tPeriod Min Bound : %u\tPeriod Max Bound : %u\tConfigured Period : %u\r\n\tObserved Min Value on High Period Monitor : %u\tObserved Max Value on High Period Monitor : %u\r\n\tObserved Min Value on Low Period Monitor : %u\tObserved Max Value on Low Period Monitor : %u\r\n",
                            MON_PERIOD_MIN_BOUND, MON_PERIOD_MAX_BOUND, gPwm_period,
                            ECAP_observedMinValue(MON_PERIOD_HIGH_BASE_ADDR, ECAP_MONITORING_UNIT_1),
                            ECAP_observedMaxValue(MON_PERIOD_HIGH_BASE_ADDR, ECAP_MONITORING_UNIT_1),
                            ECAP_observedMinValue(MON_PERIOD_LOW_BASE_ADDR, ECAP_MONITORING_UNIT_1),
                            ECAP_observedMaxValue(MON_PERIOD_LOW_BASE_ADDR, ECAP_MONITORING_UNIT_1));

                /* Checking for the error flags expected for High Pulse and Low Pulse monitors */
                if( ((gHighPeriodMonErrorStatus & ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2) != ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2)
                    ||
                    ((gLowPeriodMonErrorStatus & ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2) != ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2))
                {
                    /* test fail scenario */
                    testErrors++;
                }
                testCase = TEST_STATE_COMPLETE;

                /* Return to normal PWM wave */
                App_updateEpwmInput(PWM_INIT_ON_EDGE_TIME, PWM_INIT_OFF_EDGE_TIME, PWM_INIT_PERIOD);
                break;
            }
            default :
            {
                /* TEST_STATE_COMPLETE */
                break;
            }
        }
    }

    if(testErrors > 0)
    {
        DebugP_log("some tests have failed\r\n");
    }
    else
    {
        DebugP_log("ECAP Signal Monitoring Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    /* Clean exit by deregistering the interrupts, clearing and disabling the peripheral interrupts */

    HwiP_destruct(&gEpwmHwiObject_1);
    HwiP_destruct(&gEcapHwiObject_1);

    EPWM_setTimeBaseCounterMode(INPUT_EPWM_BASE_ADDR, EPWM_COUNTER_MODE_STOP_FREEZE);
    EPWM_disableInterrupt(INPUT_EPWM_BASE_ADDR);
    EPWM_clearEventTriggerInterruptFlag(INPUT_EPWM_BASE_ADDR);

    ECAP_disableInterrupt(MON_PULSE_HIGH_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
    ECAP_clearInterrupt(MON_PULSE_HIGH_BASE_ADDR, ECAP_ISR_SOURCE_ALL);

    ECAP_disableInterrupt(MON_PULSE_LOW_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
    ECAP_clearInterrupt(MON_PULSE_LOW_BASE_ADDR, ECAP_ISR_SOURCE_ALL);

    ECAP_disableInterrupt(MON_PERIOD_HIGH_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
    ECAP_clearInterrupt(MON_PERIOD_HIGH_BASE_ADDR, ECAP_ISR_SOURCE_ALL);

    ECAP_disableInterrupt(MON_PERIOD_LOW_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
    ECAP_clearInterrupt(MON_PERIOD_LOW_BASE_ADDR, ECAP_ISR_SOURCE_ALL);

    ECAP_disableInterrupt(MON_POSEDGE_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
    ECAP_clearInterrupt(MON_POSEDGE_BASE_ADDR, ECAP_ISR_SOURCE_ALL);

    ECAP_disableInterrupt(MON_NEGEDGE_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
    ECAP_clearInterrupt(MON_NEGEDGE_BASE_ADDR, ECAP_ISR_SOURCE_ALL);

    Board_driversClose();
    Drivers_close();
}

static void App_epwmUpdateIsr(void* args)
{

    if(gPwm_updateFlag == TRUE)
    {
        if((current_on_edge_time == gPwm_onEdgeTime) && (current_off_edge_time == gPwm_offEdgeTime) && (gPwm_period == current_period))
        {
            SemaphoreP_post(&gEpwmSyncSemObject);
            gPwm_updateFlag = FALSE;
        }
        else
        {
            if(current_on_edge_time != gPwm_onEdgeTime)
            {
                current_on_edge_time = (current_on_edge_time > gPwm_onEdgeTime)? current_on_edge_time - 1 : current_on_edge_time + 1;
            }
            if(current_off_edge_time != gPwm_offEdgeTime)
            {
                current_off_edge_time = (current_off_edge_time > gPwm_offEdgeTime)? current_off_edge_time - 1 : current_off_edge_time + 1;
            }
            if(current_period != gPwm_period)
            {
                current_period = (current_period > gPwm_period)? current_period - 1 : current_period + 1;
            }

            /* update the input waveform from EPWM slowly */
            EPWM_setTimeBasePeriod(INPUT_EPWM_BASE_ADDR, current_period);
            EPWM_setCounterCompareValue(INPUT_EPWM_BASE_ADDR, EPWM_COUNTER_COMPARE_A, current_on_edge_time);
            EPWM_setCounterCompareValue(INPUT_EPWM_BASE_ADDR, EPWM_COUNTER_COMPARE_B, current_off_edge_time);
        }
    }
    /* Clearing interrupts */
    EPWM_clearEventTriggerInterruptFlag(INPUT_EPWM_BASE_ADDR);
}

static void App_ecapIsr(void* args)
{
    if(ECAP_getGlobalInterruptStatus(MON_PULSE_HIGH_BASE_ADDR) == TRUE)
    {
        gHighPulseMonErrorStatus = ECAP_getInterruptSource(MON_PULSE_HIGH_BASE_ADDR);
        ECAP_clearInterrupt(MON_PULSE_HIGH_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
        ECAP_clearGlobalInterrupt(MON_PULSE_HIGH_BASE_ADDR);
    }
    if(ECAP_getGlobalInterruptStatus(MON_PULSE_LOW_BASE_ADDR) == TRUE)
    {
        gLowPulseMonErrorStatus = ECAP_getInterruptSource(MON_PULSE_LOW_BASE_ADDR);
        ECAP_clearInterrupt(MON_PULSE_LOW_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
        ECAP_clearGlobalInterrupt(MON_PULSE_LOW_BASE_ADDR);
    }
    if(ECAP_getGlobalInterruptStatus(MON_PERIOD_HIGH_BASE_ADDR) == TRUE)
    {
        gHighPeriodMonErrorStatus = ECAP_getInterruptSource(MON_PERIOD_HIGH_BASE_ADDR);
        ECAP_clearInterrupt(MON_PERIOD_HIGH_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
        ECAP_clearGlobalInterrupt(MON_PERIOD_HIGH_BASE_ADDR);
    }
    if(ECAP_getGlobalInterruptStatus(MON_PERIOD_LOW_BASE_ADDR) == TRUE)
    {
        gLowPeriodMonErrorStatus = ECAP_getInterruptSource(MON_PERIOD_LOW_BASE_ADDR);
        ECAP_clearInterrupt(MON_PERIOD_LOW_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
        ECAP_clearGlobalInterrupt(MON_PERIOD_LOW_BASE_ADDR);
    }
    if(ECAP_getGlobalInterruptStatus(MON_POSEDGE_BASE_ADDR) == TRUE)
    {
        /* preserve read state because the sync in will restart the monitoring. */
        if(testCase == TEST_STATE_EDGE)
        {
            gPosedgeMonErrorStatus = ECAP_getInterruptSource(MON_POSEDGE_BASE_ADDR);
        }
        else
        {
            /*  */
        }
        ECAP_clearInterrupt(MON_POSEDGE_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
        ECAP_clearGlobalInterrupt(MON_POSEDGE_BASE_ADDR);
    }
    if(ECAP_getGlobalInterruptStatus(MON_NEGEDGE_BASE_ADDR) == TRUE)
    {
        /* preserve read state because the sync in will restart the monitoring. */
        if(testCase == TEST_STATE_EDGE)
        {
            gNegedgeMonErrorStatus = ECAP_getInterruptSource(MON_NEGEDGE_BASE_ADDR);
        }
        else
        {
            /*  */
        }
        ECAP_clearInterrupt(MON_NEGEDGE_BASE_ADDR, ECAP_ISR_SOURCE_ALL);
        ECAP_clearGlobalInterrupt(MON_NEGEDGE_BASE_ADDR);
    }
}

void App_registerInterrupt(void)
{
    HwiP_Params         hwiPrms_1;
    HwiP_Params         hwiPrms_2;
    /* Registering Interrupt for the EPWM and ECAP */
    HwiP_Params_init(&hwiPrms_1);
    /* Integrate with Syscfg */
    hwiPrms_1.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms_1.isPulse     = 1;
    hwiPrms_1.priority    = 0;
    hwiPrms_1.callback    = &App_epwmUpdateIsr;
    status                = HwiP_construct(&gEpwmHwiObject_1, &hwiPrms_1);
    DebugP_assert(status == SystemP_SUCCESS);

    HwiP_Params_init(&hwiPrms_2);
    /* Integrate with Syscfg */
    hwiPrms_2.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms_2.isPulse     = 1;
    hwiPrms_2.priority    = 0;
    hwiPrms_2.callback    = &App_ecapIsr;
    status                = HwiP_construct(&gEcapHwiObject_1, &hwiPrms_2);
    DebugP_assert(status == SystemP_SUCCESS);
}

void App_updateEpwmInput(uint16_t onTime, uint16_t offTime, uint16_t period)
{
    gPwm_onEdgeTime = onTime;
    gPwm_offEdgeTime = offTime;
    gPwm_period = period;
    gPwm_updateFlag = TRUE;
    SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);

    return;
}

void App_restartMonitoring(uint32_t base)
{
    /* Once the Edge Monitoring is  */
    ECAP_enableSignalMonitoringUnit(base, ECAP_MONITORING_UNIT_1);
    ECAP_enableTimeStampCapture(base);

    return;
}

