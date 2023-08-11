/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/HwiP.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example demonstrates profiling of interrupts (just for analysis)
 *
 * Here we create two tasks i.e., ping and pong
 * ping task - configures three interrupts with different
 *             priority levels, causing nesting. Later
 *             after servicing these, it configures another
 *             interrupt to wake-up the pong task.
 *
 * pong task - initially waits for signal from ping task.
 *             Once its received, it configures an ISR to
 *             wake-up ping task.
 *
 * In this process, the ISR being serviced will update the
 * trace buffer and compares it with the expected behaviour
 * of interrupts. Test case is marked as pass, if the behaviour
 * is as expected else its considered as fail.
 *
 * The behaviour of ISR is as expected -
 * A2-A1-A0-B0-B1-B2-A2-A1-A0-B0-B1-B2-A2-A1-A0-B0-B1-B2-A2-
 * A1-A0-B0-B1-B2-A3-B3-A4-B4
 *
 * Finally, the total ISR load and CPU load is displayed.
 *
 * User can get INTR load at any point of time by
 * - providing a call to "ProfileP_CycCount()" and dividing it
 *   by core frequency
 * - providing a call to "ProfileP_time()"
 *
 * NOTE - Make sure "#define INTR_PROF" is present in
 *       "/source/drivers/hw_include/xxx/soc_config.h",
 *       where "xxx" = am64x_am243x or am263x or am273x or awr294x
 *       to enable capturing of interrupt trace.
 */

/* Allocate stack size to the task */
#define PONG_TASK_SIZE (1024*4u)
#define PING_TASK_SIZE (1024*4u)

/* Interrupt and task priorities */
#define PING_INT_NUM           (20u)
#define PONG_INT_NUM           (21u)
#define PING_TASK_PRI          (2u)
#define PONG_TASK_PRI          (3u)

/* Number of trace elements to be stored*/
#define TRACE_SIZE  30

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
 * This array will be used as stack for the respective
 * ping and pong tasks
 */
uint8_t gPingTaskStack[PING_TASK_SIZE] __attribute__((aligned(32)));
uint8_t gPongTaskStack[PONG_TASK_SIZE] __attribute__((aligned(32)));

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
uint64_t pingIsrLoad = 0, pongIsrLoad = 0, cpuClockRate = 0;

/* Task, Semaphore and Hwi objects */
TaskP_Object gPingTaskObj;
StaticSemaphore_t gPingSemObj;
SemaphoreHandle_t gPingSem;
HwiP_Object gPingHwiObj;

TaskP_Object gPongTaskObj;
StaticSemaphore_t gPongSemObj;
SemaphoreHandle_t gPongSem;
HwiP_Object gPongHwiObj;

/* ping ISR */
static void ping_isr(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    /* Check if the previous trace element is as expected */
    if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xB2)
    {
        if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xB1)
        {
            status = SystemP_FAILURE;
        }
    }

    /* Add Timer 0 ISR start to trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xA3;
    gTraceISRIndex++;

    gLoopVar0 = LOOP_COUNT;
    while(gLoopVar0 > 0)
    {
        gLoopVar0--;
    }

    /* Check if the previous trace element is as expected */
    if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xA3)
    {
        status = SystemP_FAILURE;
    }

    /* Add ping ISR to the trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xB3;
    gTraceISRIndex++;

    xSemaphoreGiveFromISR( gPongSem, &doTaskSwitch); /* wake up pong task */
    portYIELD_FROM_ISR( doTaskSwitch );
    pingIsrLoad = (ProfileP_CycCount()*100)/cpuClockRate;
}

/* pong ISR */
static void pong_isr(void *arg)
{
    BaseType_t doTaskSwitch = 0;

    /* Check if the previous trace element is as expected */
    if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xB3)
    {
        status = SystemP_FAILURE;
    }

    /* Add Timer 0 ISR start to trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xA4;
    gTraceISRIndex++;

    gLoopVar0 = LOOP_COUNT;
    while(gLoopVar0 > 0)
    {
        gLoopVar0--;
    }

    /* Check if the previous trace element is as expected */
    if(gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xA4)
    {
        status = SystemP_FAILURE;
    }

    /* Add pong ISR to the trace */
    gTraceISR[gTraceISRIndex % TRACE_SIZE] = 0xB4;
    gTraceISRIndex++;

    xSemaphoreGiveFromISR( gPingSem, &doTaskSwitch); /* wake up ping task */
    portYIELD_FROM_ISR( doTaskSwitch );
    pongIsrLoad = ((ProfileP_CycCount()*100)/cpuClockRate) - pingIsrLoad;
}

void ping_main(void *args)
{
    uint32_t cpuLoad = 0;
    uint64_t isrload = 0;
    HwiP_Params hwiParams;
    TaskP_Load taskLoad;
    cpuClockRate = SOC_getSelfCpuClk();

    DebugP_log("\r\n");
    DebugP_log("\r\n[FreeRTOS] profiling ... start !!!\r\n");

    /* Start the Lowest priority timer */
    #if defined SOC_AM64X || SOC_AM243X || SOC_AM263X
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER2]);
    ClockP_usleep(TIMER_START_INTERVAL);
    #endif

    /* Start the Medium priority timer */
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER1]);
    ClockP_usleep(TIMER_START_INTERVAL);

    /* Start the Highest priority timer */
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
    /* Wait until few ISR's are triggered and trace is captured */
    ClockP_sleep(APP_DELAY);

    /* Stop the timers */
    #if defined SOC_AM64X || SOC_AM243X || SOC_AM263X
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER2]);
    #endif
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER1]);
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

    /* switch between ping and pong tasks using direct-to-task notifications */
    xTaskNotifyGive(TaskP_getHndl(&gPongTaskObj)); /* wake up pong task */
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY); /* wait for pong to signal */

    /* switch from ping task to ISR to pong task and back to ping task using semaphores */
    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PING_INT_NUM;
    hwiParams.callback = ping_isr;
    HwiP_construct(&gPingHwiObj, &hwiParams);
    HwiP_post(PING_INT_NUM);
    xSemaphoreTake( gPingSem, portMAX_DELAY); /* wait for ISR to signal */
    HwiP_destruct(&gPingHwiObj);

    DebugP_log("[FreeRTOS] profiling ... done !!!\r\n");
    DebugP_log("[FreeRTOS] profiling ... completed !!!\r\n");
    DebugP_log("\r\n");

    isrload = (ProfileP_CycCount()*100)/cpuClockRate;
    DebugP_log("LOAD: ISR  = %2llu.%2llu %%\r\n", isrload/100, isrload%100 );

    cpuLoad = TaskP_loadGetTotalCpuLoad() - isrload;
    DebugP_log("LOAD: CPU = %2d.%2d %%\r\n", cpuLoad/100, cpuLoad%100 );

    TaskP_loadGet(&gPingTaskObj, &taskLoad);
    DebugP_log("LOAD: %s = %0.2f %%\r\n", taskLoad.name, ((float)taskLoad.cpuLoad/100)-((float)pingIsrLoad/100) );

    TaskP_loadGet(&gPongTaskObj, &taskLoad);
    DebugP_log("LOAD: %s = %0.2f %%\r\n", taskLoad.name, ((float)taskLoad.cpuLoad/100)-((float)pongIsrLoad/100) );
    DebugP_log("\r\n");

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    /* One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete */
    vTaskDelete(NULL);
}

void pong_main(void *args)
{
    HwiP_Params hwiParams;

    ulTaskNotifyTake( pdTRUE, portMAX_DELAY); /* wait for ping to signal */
    xTaskNotifyGive(TaskP_getHndl(&gPingTaskObj)); /* wake up ping task */

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PONG_INT_NUM;
    hwiParams.callback = pong_isr;
    HwiP_construct(&gPongHwiObj, &hwiParams);

    xSemaphoreTake( gPongSem, portMAX_DELAY); /* wait for ISR to signal */
    HwiP_post(PONG_INT_NUM);
    HwiP_destruct(&gPongHwiObj);

    /* One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete */
    vTaskDelete(NULL);
}

/* Highest proiority ping ISR */
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

/* Medium proiority ping ISR */
void timer1_ISR(void)
{
    /* Check if the previous trace element is as expected */
    #if defined SOC_AM64X || SOC_AM243X || SOC_AM263X
    if((gTraceISRIndex != 0) && (gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xA2))
    {
        status = SystemP_FAILURE;
    }
    #else
    if((gTraceISRIndex != 0) && (gTraceISR[(gTraceISRIndex-1) % TRACE_SIZE] != 0xB1))
    {
        status = SystemP_FAILURE;
    }
    #endif

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

/* Lowest proiority ping ISR */
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

void interrupt_profiling_main(void *args)
{
    int32_t status;
    TaskP_Params taskParams;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* First create the semaphores */
    gPingSem = xSemaphoreCreateBinaryStatic(&gPingSemObj);
    configASSERT(gPingSem != NULL);
    vQueueAddToRegistry(gPingSem, "Ping Sem"); /* This makes the semaphore visible in ROV within CCS IDE */

    gPongSem = xSemaphoreCreateBinaryStatic(&gPongSemObj);
    configASSERT(gPongSem != NULL);
    vQueueAddToRegistry(gPongSem, "Pong Sem"); /* This makes the semaphore visible in ROV within CCS IDE */

    /* Then create the tasks, order of task creation does not matter for this example */
    TaskP_Params_init(&taskParams);
    taskParams.name = "pong";
    taskParams.stackSize = PONG_TASK_SIZE;
    taskParams.stack = gPongTaskStack;
    taskParams.priority = PONG_TASK_PRI;
    taskParams.args = NULL;
    taskParams.taskMain = pong_main;
    status = TaskP_construct(&gPongTaskObj, &taskParams);
    DebugP_assert(status==SystemP_SUCCESS);

    TaskP_Params_init(&taskParams);
    taskParams.name = "ping";
    taskParams.stackSize = PING_TASK_SIZE;
    taskParams.stack = gPingTaskStack;
    taskParams.priority = PING_TASK_PRI;
    taskParams.args = NULL;
    taskParams.taskMain = ping_main;
    status = TaskP_construct(&gPingTaskObj, &taskParams);
    DebugP_assert(status==SystemP_SUCCESS);

    Board_driversClose();
    /* Dont close drivers to keep the UART driver open for console */
    /* Drivers_close(); */
}
