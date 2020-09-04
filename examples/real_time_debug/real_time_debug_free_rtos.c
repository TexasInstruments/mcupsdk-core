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

#include <FreeRTOS.h>
#include <task.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_dpl_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "Serial_Cmd_HAL.h"

/*
 * This example is used to demonstrate the Real time debug functionality.
 *
 * The value of gLoopTicker is continuously updated and it can be viewed from CCS
 * during run time by enabling real time debug functionality as shown in \ref REAL_TIME_DEBUG_SUPPORT_GUIDE.
 *
 */

/* Serial monitor task related properties, like priority, stack size, stack memory, task object handles */
#define MONITOR_TASK_PRI (2u)
#define MONITOR_TASK_SIZE (16*1024/sizeof(StackType_t))
StackType_t  gMonitorTaskStack[MONITOR_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gMonitorTaskObj;
TaskHandle_t gMonitorTask;

/* Watch variable to be viewedin CCS */
uint32_t gLoopTicker = 0;

/* Serial monitor Task which communicates with CCS */
void serialMonitorTask(void *args)
{
    /* Initialize the serial monitor */
    SerialCmd_init();

    while(1)
    {
        /* Run serial monitor */
        SerialCmd_read();
    }
}

void real_time_debug_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Console won't work along with Serial monitor */
    DebugP_log("Real time debug Test Started ...\r\n");
    DebugP_log("Real time debug Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    /* Low Priority Task is created to run the Real time debug loop */
    gMonitorTask = xTaskCreateStatic(serialMonitorTask,
                                  "serialMonitor_task",
                                  MONITOR_TASK_SIZE,
                                  NULL,
                                  MONITOR_TASK_PRI,
                                  gMonitorTaskStack,
                                  &gMonitorTaskObj);
    configASSERT(gMonitorTask != NULL);

    while (1){
        /* Increment the watch variable continuously */
        gLoopTicker+=1;

        /*
         * Delay of 500 ms. This makes sure that the low priority Serial
         * monitor task gets to run.
         */
        ClockP_usleep(500000);

        /* Reset the watch variable */
        if (gLoopTicker>=100)
        {
            gLoopTicker=0;
        }
    }

    Board_driversClose();
    Drivers_close();
}