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
    This example configures ePWM9A for:
     Up count mode
     Period starts at 500 and goes up to 8000
     Toggle output on PRD

     eCAP1 is configured to capture the time between rising
     and falling edge of the ePWM9A output.

     ecap1PassCount - Successful captures.
     ecap1IntCount - Interrupt counts.
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

/* variable to hold base address of ECAP and EPWM that is used */
uint32_t gEcapBaseAddr;
uint32_t gEpwm9BaseAddr;

//
// Defines
//
#define PWM3_TIMER_MIN     500U
#define PWM3_TIMER_MAX     8000U
#define EPWM_TIMER_UP      1U
#define EPWM_TIMER_DOWN    0U

//
// Globals
//
uint32_t ecap1IntCount;
uint32_t ecap1PassCount;
uint32_t epwm9TimerDirection;
volatile uint16_t cap2Count;
volatile uint16_t cap3Count;
volatile uint16_t cap4Count;
volatile uint16_t epwm9PeriodCount;
volatile uint16_t epwm_tick_time;
volatile uint16_t ecap_tick_time;


void ecap_capture_pwm_main(void *args)
{

    int32_t status;
    uint32_t numIsrCnt = 10;
    HwiP_Params hwiPrms;

    Drivers_open();
    Board_driversOpen();

    epwm9TimerDirection = EPWM_TIMER_UP;
    //
    // Initialize counters:
    //
    cap2Count = 0U;
    cap3Count = 0U;
    cap4Count = 0U;
    ecap1IntCount = 0U;
    ecap1PassCount = 0U;
    epwm9PeriodCount = 0U;

    /* In EPWM_setClockPrescaler configuration,
       EPWM_ClockDivider is EPWM_CLOCK_DIVIDER_1
       EPWM_HSClockDivider is EPWM_HSCLOCK_DIVIDER_2
       TBCLK = SYSCLK/(1*2)
       1 clock pulse time = 1/TBCLK
       Eg: SYSCLK = 200 MHZ => TBCLK = 100 MHz
       1 tick time = 1/100MHz = 10 ns
    */
    epwm_tick_time = 10;

    /* The ECAP gets SYSCLK
       1 tick time = 1/SYSCLK = 5 ns
    */
    ecap_tick_time = 5;

    DebugP_log("ECAP Capture Pwm Test Started ...\r\n");

     /* Get ECAP address */
    gEcapBaseAddr = CONFIG_ECAP0_BASE_ADDR;

     /* Get Address of ePWM */
    gEpwm9BaseAddr = CONFIG_EPWM9_BASE_ADDR;

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


    /* Wait for specified time */
    while(numIsrCnt > 0)
    {
        //
        // Clear interrupt flags for upcoming interrupts.
        //
        ECAP_clearInterrupt(gEcapBaseAddr, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
        ECAP_clearGlobalInterrupt(gEcapBaseAddr);

        //
        // Start eCAP
        //
        ECAP_reArm(gEcapBaseAddr);

        SemaphoreP_pend(&gEcapSyncSemObject, SystemP_WAIT_FOREVER);
        numIsrCnt--;
    }

    DebugP_log("Interrupt No.: %u and Pass Count: %u\n",ecap1IntCount,ecap1PassCount);

    /* Stop ECAP Counter */
    ECAP_stopCounter(CONFIG_ECAP0_BASE_ADDR);

    /* Disable and clear interrupts */
    ECAP_disableInterrupt(gEcapBaseAddr, ECAP_ISR_SOURCE_ALL);
    ECAP_clearInterrupt(gEcapBaseAddr, ECAP_ISR_SOURCE_ALL);
    HwiP_destruct(&gEcapHwiObject);
    SemaphoreP_destruct(&gEcapSyncSemObject);


    DebugP_log("ECAP Capture Pwm Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_ecapIntrISR(void *handle)
{
    volatile uint16_t status;

    status = ECAP_getInterruptSource(gEcapBaseAddr);

    if(status & ECAP_ISR_SOURCE_CAPTURE_EVENT_4)
    {
        SemaphoreP_post(&gEcapSyncSemObject);


        // Get the capture counts. Each capture should be 4x the ePWM count
        // because of the ePWM clock divider.
        //
        cap2Count = ECAP_getEventTimeStamp(gEcapBaseAddr, ECAP_EVENT_2);
        cap3Count = ECAP_getEventTimeStamp(gEcapBaseAddr, ECAP_EVENT_3);
        cap4Count = ECAP_getEventTimeStamp(gEcapBaseAddr, ECAP_EVENT_4);

        //
        // Compare the period value with the captured count
        //
        epwm9PeriodCount = EPWM_getTimeBasePeriod(gEpwm9BaseAddr);

        if(cap2Count * ecap_tick_time > ((epwm9PeriodCount *epwm_tick_time) + 10) ||
        cap2Count * ecap_tick_time < ((epwm9PeriodCount *epwm_tick_time) - 10))
        {
            DebugP_assert(0);
        }

        if(cap3Count * ecap_tick_time > ((epwm9PeriodCount *epwm_tick_time) + 10) ||
        cap3Count * ecap_tick_time < ((epwm9PeriodCount *epwm_tick_time) - 10))
        {
            DebugP_assert(0);
        }

        if(cap4Count * ecap_tick_time > ((epwm9PeriodCount *epwm_tick_time) + 10) ||
        cap4Count * ecap_tick_time < ((epwm9PeriodCount *epwm_tick_time) - 10))
        {
            DebugP_assert(0);
        }

        ecap1IntCount++;

        //
        // Keep track of the ePWM direction and adjust period accordingly to
        // generate a variable frequency PWM.
        //
        if(epwm9TimerDirection == EPWM_TIMER_UP)
        {
            if(epwm9PeriodCount < PWM3_TIMER_MAX)
            {
            EPWM_setTimeBasePeriod(gEpwm9BaseAddr, ++epwm9PeriodCount);
            }
            else
            {
            epwm9TimerDirection = EPWM_TIMER_DOWN;
            EPWM_setTimeBasePeriod(gEpwm9BaseAddr, ++epwm9PeriodCount);
            }
        }
        else
        {
            if(epwm9PeriodCount > PWM3_TIMER_MIN)
            {
                EPWM_setTimeBasePeriod(gEpwm9BaseAddr, --epwm9PeriodCount);
            }
            else
            {
            epwm9TimerDirection = EPWM_TIMER_UP;
            EPWM_setTimeBasePeriod(gEpwm9BaseAddr, ++epwm9PeriodCount);
            }
        }

        //
        // Count correct captures
        //
        ecap1PassCount++;



    }

    return;
}