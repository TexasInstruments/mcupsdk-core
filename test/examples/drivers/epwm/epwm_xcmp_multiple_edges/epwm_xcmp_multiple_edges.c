
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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example uses the ePWM module to generate multiple edges in a pwm cycle.
 * Three instances of epwm have been used to demonstrate the use of XCMP feature.
 *
 * For EPWMx, XCMP feature is disabled and it generates waves with duty cycle of 37.5%
 *
 * For EPWMy, XCMP feature is enabled and the only the ACTIVE set registers are used, duty cycle is 1%.
 *
 * For EPWMz, XCMP feature is enabled and the ACTIVE as well as the three SHADOW set registers are used.
 * Here using shadow set 3, waves with duty cycle 50% is generated for 5 cycles for both output channels.
 * Using shadow set 2, waves with duty cycle 0% is generated for 8 cycles for both output channels.
 * Using shadow set 1, waves with duty cycle 20% for channel A and 40% for channel B is generated and this continues.
 */

/* Frequency of PWM output signal in Hz - 1 KHz is selected */
#define APP_EPWM_OUTPUT_FREQ    (1U * 1000U)
/* APP run time in seconds */
#define APP_EPWM_RUN_TIME    (10U)
/* FIXME : To be removed after syscfg integration */
#define APP_INT_IS_PULSE    (1U)

/* Global variables and objects */
static HwiP_Object  gEpwmHwiObject0;
static HwiP_Object  gEpwmHwiObject1;
static HwiP_Object  gEpwmHwiObject2;
static SemaphoreP_Object  gEpwmSyncSemObject;

/* Function Prototypes */
static void App_epwmIntrISR0(void *handle);
static void App_epwmIntrISR1(void *handle);
static void App_epwmIntrISR2(void *handle);

/* variable to hold base address of EPWM that is used */
uint32_t gEpwmBaseAddr0;
uint32_t gEpwmBaseAddr1;
uint32_t gEpwmBaseAddr2;


uint32_t  numIsrCnt;

void wait_for_events(uint32_t count)
{
    while (count--)
    {
        numIsrCnt = (APP_EPWM_RUN_TIME * APP_EPWM_OUTPUT_FREQ);
        while(numIsrCnt > 0)
        {
            SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
            numIsrCnt--;
        }
    }
}

void epwm_xcmp_multiple_edges_main(void *args)
{
    int32_t  status;
    HwiP_Params  hwiPrms0;
    HwiP_Params  hwiPrms1;
    HwiP_Params  hwiPrms2;

    numIsrCnt = (APP_EPWM_RUN_TIME * APP_EPWM_OUTPUT_FREQ);

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM XCMP multiple edges Test Started ...\r\n");

    /* Get Address of ePWM */
    gEpwmBaseAddr0 = CONFIG_EPWM0_BASE_ADDR;
    gEpwmBaseAddr1 = CONFIG_EPWM1_BASE_ADDR;
    gEpwmBaseAddr2 = CONFIG_EPWM2_BASE_ADDR;

    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* For EPWM0 */
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms0);
    hwiPrms0.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms0.callback    = &App_epwmIntrISR0;
    hwiPrms0.isPulse     = APP_INT_IS_PULSE;
    status               = HwiP_construct(&gEpwmHwiObject0, &hwiPrms0);
    DebugP_assert(status == SystemP_SUCCESS);

    /* For EPWM1 */
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms1);
    hwiPrms1.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms1.callback    = &App_epwmIntrISR1;
    hwiPrms1.isPulse     = APP_INT_IS_PULSE;
    status               = HwiP_construct(&gEpwmHwiObject1, &hwiPrms1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* For EPWM2 */
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms2);
    hwiPrms2.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_2;
    hwiPrms2.callback    = &App_epwmIntrISR2;
    hwiPrms2.isPulse     = APP_INT_IS_PULSE;
    status               = HwiP_construct(&gEpwmHwiObject2, &hwiPrms2);
    DebugP_assert(status == SystemP_SUCCESS);

    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr0);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr1);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr2);

    EPWM_enableXLoad(gEpwmBaseAddr2);
    //wait_for_events(1);

    EPWM_disableInterrupt(gEpwmBaseAddr0);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr0);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject0);

    EPWM_disableInterrupt(gEpwmBaseAddr1);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr1);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject1);

    EPWM_disableInterrupt(gEpwmBaseAddr2);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr2);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject2);

    SemaphoreP_destruct(&gEpwmSyncSemObject);

    DebugP_log("EPWM XCMP multiple edges Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_epwmIntrISR0(void *handle)
{
    volatile bool status;

    status = EPWM_getEventTriggerInterruptStatus(gEpwmBaseAddr0);
    if(status == true)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr0);
    }

    return;
}

static void App_epwmIntrISR1(void *handle)
{
    volatile bool status;

    status = EPWM_getEventTriggerInterruptStatus(gEpwmBaseAddr1);
    if(status == true)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr1);
    }

    return;
}

static void App_epwmIntrISR2(void *handle)
{
    volatile bool status;

    status = EPWM_getEventTriggerInterruptStatus(gEpwmBaseAddr2);
    if(status == true)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr2);
    }

    return;
}
