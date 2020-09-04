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

//
// Included Files
//
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/*
This example configures ePWMx and ePWMy as follows

ePWMx has TZ1 as one shot trip source
ePWMy has TZ1 as cycle by cycle trip source
Initially tie TZ1 high. During the test, monitor ePWM1 or ePWM2 outputs on a scope. Pull TZ1 low to see the effect.

ePWMxA is on GPIO_A
ePWMyA is on GPIO_B
This example also makes use of the Input X-BAR. The external trigger pin is routed to the input X-BAR, from which it is routed to TZ1.

The TZ-Event is defined such that ePWMxA will undergo a One-Shot Trip and ePWMyA will undergo a Cycle-By-Cycle Trip.
 */


//
// Globals
//
uint32_t  gEpwm1TZIntCount;
uint32_t  gEpwm2TZIntCount;

/* FIXME : To be removed after syscfg integration */
#define APP_INT_IS_PULSE    (1U)

/* Global variables and objects */
static HwiP_Object  gEpwmHwiObject_1;
static HwiP_Object  gEpwmHwiObject_2;
static SemaphoreP_Object  gEpwmSyncSemObject;

/* Function Prototypes */
static void App_epwmIntrISR_1(void *handle);
static void App_epwmIntrISR_2(void *handle);

/* Get Address of ePWM */
uint32_t gEpwm1BaseAddr;
uint32_t gEpwm2BaseAddr;


void epwm_trip_zone_main(void *args)
{
    int32_t  status;
    HwiP_Params  hwiPrms_1;
    HwiP_Params  hwiPrms_2;

    GPIO_setDirMode(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN, 0);
    GPIO_pinWriteHigh(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN);

    Drivers_open();
    Board_driversOpen();

    /* Get Address of ePWM */
    gEpwm1BaseAddr = CONFIG_EPWMx_BASE_ADDR;
    gEpwm2BaseAddr = CONFIG_EPWMy_BASE_ADDR;
    EPWM_clearTripZoneFlag(gEpwm1BaseAddr,EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(gEpwm2BaseAddr,EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_CBC);

    DebugP_log("EPWM Trip Zone Test Started ...\r\n");


    status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* For EPWMx  */

    /* Register & enable interrupt */
     HwiP_Params_init(&hwiPrms_1);
    /* Integrate with Syscfg */
    hwiPrms_1.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms_1.isPulse     = APP_INT_IS_PULSE;
    hwiPrms_1.callback    = &App_epwmIntrISR_1;
    status              =   HwiP_construct(&gEpwmHwiObject_1, &hwiPrms_1);
    DebugP_assert(status == SystemP_SUCCESS);


    /* For EPWMy  */

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms_2);
    /* Integrate with Syscfg */
    hwiPrms_2.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms_2.isPulse     = APP_INT_IS_PULSE;
    hwiPrms_2.callback    = &App_epwmIntrISR_2;
    status              = HwiP_construct(&gEpwmHwiObject_2, &hwiPrms_2);
    DebugP_assert(status == SystemP_SUCCESS);


    gEpwm1TZIntCount=0;
    EPWM_clearTripZoneFlag(gEpwm1BaseAddr,EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);

    while( gEpwm1TZIntCount < 10 )
    {
        GPIO_pinWriteLow(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN);

        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);

        DebugP_log("TZ OST interrupt hit %d times!!\r\n", gEpwm1TZIntCount);

        GPIO_pinWriteHigh(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN);
        EPWM_clearTripZoneFlag(gEpwm1BaseAddr,EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
       //Introducing some delay for the epwm to recover
        ClockP_usleep(1920*10);  //time period = 1920 Us
    }

    gEpwm2TZIntCount=0;
    EPWM_clearTripZoneFlag(gEpwm2BaseAddr,EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_CBC);
    while( gEpwm2TZIntCount < 10 )
    {

        GPIO_pinWriteLow(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN);

        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);

        DebugP_log("TZ CBC interrupt hit %d times!!\r\n", gEpwm2TZIntCount);

        GPIO_pinWriteHigh(CSL_GPIO0_U_BASE, GPIO_OUTPUT_PIN);
        EPWM_clearTripZoneFlag(gEpwm2BaseAddr,EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_CBC);
        //Introducing some delay for the epwm to recover
        ClockP_usleep(960*10);  //time period = 960 Us
    }

    EPWM_disableInterrupt(gEpwm1BaseAddr);
    EPWM_clearEventTriggerInterruptFlag(gEpwm1BaseAddr);     /* Clear any pending interrupts if any */

    EPWM_disableInterrupt(gEpwm2BaseAddr);
    EPWM_clearEventTriggerInterruptFlag(gEpwm2BaseAddr);     /* Clear any pending interrupts if any */

    HwiP_destruct(&gEpwmHwiObject_1);
    HwiP_destruct(&gEpwmHwiObject_2);
    SemaphoreP_destruct(&gEpwmSyncSemObject);

    DebugP_log("EPWM Trip Zone Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

}


static void App_epwmIntrISR_1(void *handle)
{

    volatile uint16_t status;
    gEpwm1TZIntCount++;
    status = EPWM_getTripZoneFlagStatus(gEpwm1BaseAddr);

    if(status & EPWM_TZ_INTERRUPT)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
    }

    return;
}

static void App_epwmIntrISR_2(void *handle)
{
    volatile uint16_t status;
    gEpwm2TZIntCount++;
    status = EPWM_getTripZoneFlagStatus(gEpwm2BaseAddr);

    if(status & EPWM_TZ_INTERRUPT)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
    }

    return;
}