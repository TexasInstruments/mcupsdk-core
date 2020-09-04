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
#include <drivers/ecap.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example uses the ECAP module in APWM mode to generate a signal.
 *
 * In this example ECAP0 is used, the user can also select a
 * different one through syscfg.
 *
 * User can specify paramters like frequency and duty cycle for the
 * generated wave. Run time can also be specified for the example.
 *
 * This example also showcases how to configure and use the ECAP module.
 */

/* Output frequency of the PWM wave (in Hz) */
#define APP_ECAP_APWM_OUT_FREQ  (1000U)
/* App run time (in seconds) */
#define APP_ECAP_RUN_TIME  (10U)
/* FIXME : To be removed after syscfg integration */
#define APP_INT_IS_PULSE  (1U)

/* Global variables and objects */
static HwiP_Object  gEcapHwiObject;
static SemaphoreP_Object  gEcapSyncSemObject;

/* Function Prototypes */
static void App_ecapIntrISR(void *handle);

/* variable to hold base address of ECAP that is used */
uint32_t gEcapBaseAddr;

void ecap_apwm_mode_main(void *args)
{
    int32_t status;
    uint32_t numIsrCnt = (APP_ECAP_RUN_TIME * APP_ECAP_APWM_OUT_FREQ);
    HwiP_Params hwiPrms;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ECAP APWM Mode Test Started ...\r\n");

    /* Get ECAP address */
    gEcapBaseAddr = CONFIG_ECAP0_BASE_ADDR;

    status = SemaphoreP_constructCounting(&gEcapSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_22;
    hwiPrms.callback    = &App_ecapIntrISR;
    /* Integrate with Syscfg */
    hwiPrms.isPulse     = APP_INT_IS_PULSE;
    status              = HwiP_construct(&gEcapHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    ECAP_clearInterrupt(gEcapBaseAddr, ECAP_ISR_SOURCE_COUNTER_PERIOD);

    /* Wait for specified time */
    while(numIsrCnt > 0)
    {
        SemaphoreP_pend(&gEcapSyncSemObject, SystemP_WAIT_FOREVER);
        numIsrCnt--;
    }

    /* Stop ECAP Counter */
    ECAP_stopCounter(CONFIG_ECAP0_BASE_ADDR);

    /* Disable and clear interrupts */
    ECAP_disableInterrupt(gEcapBaseAddr, ECAP_ISR_SOURCE_ALL);
    ECAP_clearInterrupt(gEcapBaseAddr, ECAP_ISR_SOURCE_ALL);
    HwiP_destruct(&gEcapHwiObject);
    SemaphoreP_destruct(&gEcapSyncSemObject);

    DebugP_log("ECAP APWM Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_ecapIntrISR(void *handle)
{
    volatile uint16_t status;

    status = ECAP_getInterruptSource(gEcapBaseAddr);
    if(status & ECAP_ISR_SOURCE_COUNTER_PERIOD)
    {
        SemaphoreP_post(&gEcapSyncSemObject);
        ECAP_clearInterrupt(gEcapBaseAddr, ECAP_ISR_SOURCE_COUNTER_PERIOD);
    }

    return;
}
