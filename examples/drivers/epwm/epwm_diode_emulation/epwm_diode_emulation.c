
/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#define APP_INT_IS_PULSE    (1U)

/* Global variables and objects */
static HwiP_Object  gEpwmHwiObject_1;
static HwiP_Object  gEpwmHwiObject_2;
static HwiP_Object  gEpwmHwiObject_3;

/* Function Prototypes */
static void App_epwmIntrISR_1(void *handle);
static void App_epwmIntrISR_2(void *handle);
static void App_epwmIntrISR_3(void *handle);

/* Get Address of ePWM */
uint32_t gEpwm1BaseAddr;
uint32_t gEpwm2BaseAddr;
uint32_t gEpwm3BaseAddr;
bool toggle_config_tripH = false;
bool toggle_config_tripL = false;

void epwm_diode_emulation_main(void *args)
{
    int32_t  status;
    HwiP_Params  hwiPrms_1;
    HwiP_Params  hwiPrms_2;
    HwiP_Params  hwiPrms_3;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM Diode Emulation Logic Test Started ...\r\n");
    DebugP_log("App will wait for 60 seconds\r\n");

    /* Get Address of ePWM */
    gEpwm1BaseAddr = TripH_source_BASE_ADDR;
    gEpwm2BaseAddr = TripL_source_BASE_ADDR;
    gEpwm3BaseAddr = CONFIG_EPWM6_BASE_ADDR;

    /* For EPWM 0 */

    /* Register & enable interrupt */
     HwiP_Params_init(&hwiPrms_1);
    /* Integrate with Syscfg */
    hwiPrms_1.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms_1.isPulse     = APP_INT_IS_PULSE;
    hwiPrms_1.callback    = &App_epwmIntrISR_1;
    status              =   HwiP_construct(&gEpwmHwiObject_1, &hwiPrms_1);
    DebugP_assert(status == SystemP_SUCCESS);


    /* For EPWM 1 */

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms_2);
    /* Integrate with Syscfg */
    hwiPrms_2.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms_2.isPulse     = APP_INT_IS_PULSE;
    hwiPrms_2.callback    = &App_epwmIntrISR_2;
    status              = HwiP_construct(&gEpwmHwiObject_2, &hwiPrms_2);
    DebugP_assert(status == SystemP_SUCCESS);

    /* For EPWM 6 */

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms_3);
    /* Integrate with Syscfg */
    hwiPrms_3.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_2;
    hwiPrms_3.isPulse     = APP_INT_IS_PULSE;
    hwiPrms_3.callback    = &App_epwmIntrISR_3;
    status              = HwiP_construct(&gEpwmHwiObject_3, &hwiPrms_3);
    DebugP_assert(status == SystemP_SUCCESS);

    EPWM_clearEventTriggerInterruptFlag(gEpwm1BaseAddr);
    EPWM_clearEventTriggerInterruptFlag(gEpwm2BaseAddr);
    EPWM_clearTripZoneFlag(gEpwm3BaseAddr, (EPWM_TZ_INTERRUPT| EPWM_TZ_FLAG_DCAEVT1));

    ClockP_sleep(60);

    DebugP_log("EPWM Diode Emulation Logic Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_epwmIntrISR_1(void *handle)
{

    volatile uint16_t status;

    status = EPWM_getEventTriggerInterruptStatus(gEpwm1BaseAddr);

    if(status)
    {
        if(!toggle_config_tripH)
        {
            EPWM_setActionQualifierAction(TripH_source_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
            EPWM_setActionQualifierAction(TripH_source_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
        }
        else
        {
            EPWM_setActionQualifierAction(TripH_source_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
        }
        toggle_config_tripH = !toggle_config_tripH;
        EPWM_clearEventTriggerInterruptFlag(gEpwm1BaseAddr);
    }

    return;
}

static void App_epwmIntrISR_2(void *handle)
{
    volatile uint16_t status;

    status = EPWM_getEventTriggerInterruptStatus(gEpwm2BaseAddr);

    if(status)
    {
        if(!toggle_config_tripL)
        {
        EPWM_setActionQualifierAction(TripL_source_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
        EPWM_setActionQualifierAction(TripL_source_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
        }
        else
        {
            EPWM_setActionQualifierAction(TripL_source_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
        }
        toggle_config_tripL = !toggle_config_tripL;
        EPWM_clearEventTriggerInterruptFlag(gEpwm2BaseAddr);
    }

    return;
}

static void App_epwmIntrISR_3(void *handle)
{
    volatile uint16_t status;

    /* Adding some delay to position DE_Trip in TripHorTripL = low */
    volatile uint32_t ctr = 1000;
    volatile uint32_t *ptr= &ctr;

    while(ctr--)
    {
        HW_RD_REG32(ptr);
    }


    status = EPWM_getTripZoneFlagStatus(gEpwm3BaseAddr);

    if(status)
    {
        EPWM_clearDiodeEmulationActive(gEpwm3BaseAddr);
        EPWM_clearTripZoneFlag(gEpwm3BaseAddr,EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_DCAEVT1);
    }

    return;
}
