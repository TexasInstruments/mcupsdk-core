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
 * This examples demonstrates the prioritization of interrupts in VIM
 * through CPU Timer Interrupts of different priority which are configured at
 * same tick period(500ms).
 * Nesting of interrupts is shown with the help of trace buffer which captures
 * the start and end of each Timer ISR.
 * This example configures Timer 0, 1, and 2 interrupt priority in SysCfg
 * with timer 0 priority being highest and timer 2 being lowest and prints
 * a trace for the order of execution.
 * Timer2 is started first followed by Timer1 and then Timer0.
 * Timer2 Interrupts gets nested by Timer1 interrupt which in turn gets
 * nested by Timer0 interrupt.
 * The trace macros used are -
 * 0xA0 - Timer0 ISR start
 * 0xB0 - Timer0 ISR end
 * 0xA1 - Timer1 ISR start
 * 0xB1 - Timer1 ISR end
 * 0xA2 - Timer2 ISR start
 * 0xB2 - Timer2 ISR end
 *
 * The expected order of trace is -
 * A2 - A1 - A0 - B0 - B1 - B2
 *
 * For configuring the priority of interrupts which are not supported by SysCfg,
 * use the element "HwiP_Params.priority" in HwiP.
 * Priority can be specified from a level of 0 to 15, where 0 denotes highest
 * priority.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Number of trace elements to be stored*/
#define TRACE_SIZE  24
/* Counter to delay the Timere ISR */
#define LOOP_COUNT  1000 * 1000
/* The delay in uS between start of timers */
#define TIMER_START_INTERVAL  50
/*
 * Delay in seconds during which the Timer ISR's execute and
 * trace is captured
 */
#define APP_DELAY   2

/*
 * This array will be used as a trace to check the order
 * that the interrupts were serviced
 */
uint8_t  gTraceISR[TRACE_SIZE];
/* Index to update an element in the trace buffer */
volatile uint16_t  gTraceISRIndex = 0;
/* Loop variables used in ISR to simulate delay */
volatile uint32_t gLoopVar0;
volatile uint32_t gLoopVar1;
volatile uint32_t gLoopVar2;
volatile int32_t  status = SystemP_SUCCESS;

void interrupt_prioritization(void *args)
{
    uint32_t loopVar = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

	DebugP_log("[DPL] Interrupt prioritization Test Started...\r\n");

    /* Start the Lowest priority timer */
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER2]);
    /* Start next timer after a small delay */
    ClockP_usleep(TIMER_START_INTERVAL);
    /* Start the Medium priority timer */
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER1]);
    /* Start next timer after a small delay */
    ClockP_usleep(TIMER_START_INTERVAL);
    /* Start the Highest priority timer */
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    /* Wait until few ISR's are triggered and trace is captured */
    ClockP_sleep(APP_DELAY);

    /* Stop the timers */
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER2]);
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER1]);
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

    DebugP_log("[DPL] All timers stopped\r\n");
    DebugP_log("[DPL] Printing Trace - \r\n");

    /* Print the contents of trace buffer */
    for(loopVar = 0; loopVar < TRACE_SIZE; loopVar++)
    {
        DebugP_log("%X - ", gTraceISR[loopVar]);
    }

    DebugP_log("\r\n");

    /* Test passed if all the Timer ISR's were executed in correct order */
    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[DPL] Interrupt prioritization Test Completed!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

	Board_driversClose();
    Drivers_close();

    return;
}

/* Highest proiority ISR */
void timer0_ISR(void)
{
    /* Check if the previous trace element is as expected */
    if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xA1)
    {
        status = SystemP_FAILURE;
    }

    /* Add Timer 0 ISR start to trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xA0;
    gTraceISRIndex++;

    gLoopVar0 = LOOP_COUNT;
    while(gLoopVar0 > 0)
    {
        gLoopVar0--;
    }

    /* Check if the previous trace element is as expected */
    if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xA0)
    {
        status = SystemP_FAILURE;
    }

    /* Add Timer 0 ISR end to trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xB0;
    gTraceISRIndex++;
}

/* Medium proiority ISR */
void timer1_ISR(void)
{
    /* Check if the previous trace element is as expected */
    if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xA2)
    {
        status = SystemP_FAILURE;
    }

    /* Add Timer 1 ISR start to trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xA1;
    gTraceISRIndex++;

    gLoopVar1 = LOOP_COUNT;
    while(gLoopVar1 > 0)
    {
        gLoopVar1--;
    }

    /* Check if the previous trace element is as expected */
    if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xB0)
    {
        status = SystemP_FAILURE;
    }

    /* Add Timer 1 ISR end to trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xB1;
    gTraceISRIndex++;
}

/* Lowest proiority ISR */
void timer2_ISR(void)
{
    /*
     * Since this is the ISR which gets executed first, we make sure that this is not
     * its initial execution and then check if the previous trace element is as expected.
     */
    if((gTraceISRIndex != 0) && (gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xB2))
    {
        status = SystemP_FAILURE;
    }

    /* Add Timer 2 ISR start to trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xA2;
    gTraceISRIndex++;

    gLoopVar2 = LOOP_COUNT;
    while(gLoopVar2 > 0)
    {
        gLoopVar2--;
    }

    /* Check if the previous trace element is as expected */
    if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xB1)
    {
        status = SystemP_FAILURE;
    }

    /* Add Timer 2 ISR end to trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xB2;
    gTraceISRIndex++;
}