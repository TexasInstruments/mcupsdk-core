/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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
#include <drivers/ecap.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example uses the ECAP module in APWM mode to generate a signal.
 *
 * User can specify paramters like frequency and duty cycle for the
 * generated wave. Run time can also be specified for the example.
 *
 * This example also showcases how to configure and use the ECAP module.
 */

/* Input frequency to the ECAP module (SYSCLK/4) i.e 125MHz (in Hz) */
#define ECAP_INPUT_CLK_FREQ             (CONFIG_ECAP0_FCLK)
/* Output frequency of the PWM wave (in Hz) */
#define APP_ECAP_APWM_OUT_FREQ          (1000U)
/* Duty cycle of the PWM wave (in %) */
#define APP_ECAP_APWM_OUT_DUTY_CYCLE    (50U)
/* App run time (in seconds) */
#define APP_ECAP_RUN_TIME               (10U)
/* App Output pin initial state */
#define APP_ECAP_APWM_OUT_POLARITY      (ECAP_APWM_ACTIVE_LOW)

#define ECAP_INT_ALL                                   (ECAP_CEVT1_INT  | \
                                                        ECAP_CEVT2_INT  | \
                                                        ECAP_CEVT3_INT  | \
                                                        ECAP_CEVT4_INT  | \
                                                        ECAP_CNTOVF_INT | \
                                                        ECAP_PRDEQ_INT  | \
                                                        ECAP_CMPEQ_INT)

/* Global variables and objects */
static HwiP_Object  gEcapHwiObject;
static SemaphoreP_Object  gEcapSyncSemObject;

/* Function Prototypes */
static void App_ecapIntrISR(void *handle);

/* variable to hold base address of ECAP that is used */
uint32_t gEcapBaseAddr;

void ecap_apwm_mode_main(void *args)
{

    int32_t status = SystemP_SUCCESS;
    HwiP_Params hwiPrms;
    uint64_t periodVal, compareVal = 0;
    uint32_t numIsrCnt = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ECAP APWM Mode Test Started ...\r\n");

    /* Get ECAP address */
    gEcapBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_ECAP0_BASE_ADDR);
    numIsrCnt = APP_ECAP_RUN_TIME * APP_ECAP_APWM_OUT_FREQ;

    status = SemaphoreP_constructCounting(&gEcapSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CONFIG_ECAP0_INTR;
    hwiPrms.callback    = &App_ecapIntrISR;
    hwiPrms.isPulse     = CONFIG_ECAP0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEcapHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Condition to check PWM output Hz is 0 or duty cycle > 100 */
    if((APP_ECAP_APWM_OUT_FREQ == 0) || (APP_ECAP_APWM_OUT_DUTY_CYCLE > 100))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        /* Disable ECAP Interrupt and Stop Counter */
        ECAP_intrDisable(gEcapBaseAddr, ECAP_INT_ALL);
        ECAP_intrStatusClear(gEcapBaseAddr, ECAP_INT_ALL);
        ECAP_captureLoadingDisable(gEcapBaseAddr);
        ECAP_counterControl(gEcapBaseAddr, ECAP_COUNTER_STOP);

        /* Enable period equal interrupt */
        ECAP_intrEnable(gEcapBaseAddr, ECAP_PRDEQ_INT);
        /* Config ECAP Operating mode, Output pulse initial state, Sync IN,OUT */
        ECAP_operatingModeSelect(gEcapBaseAddr, ECAP_APWM_MODE);
        ECAP_APWM_polarityConfig(gEcapBaseAddr, APP_ECAP_APWM_OUT_POLARITY);
        ECAP_syncInOutSelect(gEcapBaseAddr, ECAP_SYNC_IN_DISABLE, ECAP_SYNC_OUT_DISABLE);

        periodVal = ECAP_INPUT_CLK_FREQ/APP_ECAP_APWM_OUT_FREQ;
        /* if periodVal > 32b then we cannot give accurate timing */
        DebugP_assert(periodVal < 0xFFFFFFFFU);
        /* Calculate capture value based on the duty cycle */
        compareVal = (periodVal * APP_ECAP_APWM_OUT_DUTY_CYCLE )/ 100;

        ECAP_APWM_captureConfig(gEcapBaseAddr, compareVal, periodVal);
        ECAP_APWM_shadowCaptureConfig(gEcapBaseAddr, compareVal, periodVal);
        /* Start the counter */
        ECAP_counterControl(gEcapBaseAddr, ECAP_COUNTER_FREE_RUNNING);

        /* Wait for specified time */
        while(numIsrCnt > 0)
        {
            SemaphoreP_pend(&gEcapSyncSemObject, SystemP_WAIT_FOREVER);
            numIsrCnt--;
        }

        /* Stop the counter */
        ECAP_counterControl(gEcapBaseAddr, ECAP_COUNTER_STOP);
        ECAP_intrStatusClear(gEcapBaseAddr, ECAP_INT_ALL);

        DebugP_log("ECAP APWM Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

static void App_ecapIntrISR(void *handle)
{
    uint32_t status = 0;
    status = ECAP_getIntrStatus(gEcapBaseAddr, ECAP_PRDEQ_INT);
    if(status & ECAP_PRDEQ_INT)
    {
        SemaphoreP_post(&gEcapSyncSemObject);
        ECAP_intrStatusClear(gEcapBaseAddr, ECAP_INT_ALL);
        ECAP_globalIntrClear(gEcapBaseAddr);
    }
    return;
}


