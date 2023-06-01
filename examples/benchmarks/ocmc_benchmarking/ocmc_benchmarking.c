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
#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/pmu.h>
#include <kernel/dpl/CacheP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ocmc_benchmarking.h"


uint32_t            hrs, mins, secs, durationInSecs, usecs;
uint32_t            startTime, elapsedTime;


// Types_FreqHz freq1;

/* task_calls is the number of random calls to the slave tasks for different
 * test cases. This can be used to controll the runtime of the code
*/
uint32_t task_calls = 500;

/* For each test case the  size of the memcpy buffer can be defined
 * individually. Lesser the value more is the number of cache misses per
 * second but reduced run time as the number of instructions executed
 * reduces and the memcpy instructions still remain in the cache */
uint32_t memcopy_size_arr[NUM_TEST] = {0, 100, 200, 400, 700, 1000, 1500, 2250, 2500, BUF_SIZE};

/* Variable to pick up value from the memcopy_size_arr for each test*/
uint32_t memcopy_size = 0;

/* Number of times each slave task must repeat the operation.
 * This can be used the control the execution time of the code without much
 * impact on the cache misses
*/
uint32_t iter = 1;

/* Counter for the number of sysbios task switches that occur during the
 * execution of the code
*/
uint32_t num_switches = 0;

QueueP_Handle myQ[NUM_TASK];
QueueP_Object myQObj[NUM_TASK];
TaskP_Object main_task[NUM_TASK+1];

/* Array to hold all the semaphores */
SemaphoreP_Object gSemaphorePHandle[NUM_TASK];

/* Array to hold the address of all the source buffers*/
uint32_t *buf[NUM_TASK];

PMU_EventCfg gPmuEventCfg[3] =
{
    {
        .name = "ICache Miss",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_MISS
    },
    {
        .name = "Instruction Executed",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_I_X
    },
    {
        .name = "ICache Access",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_ACCESS
    }
};

PMU_Config gPmuConfig =
{
    .bCycleCounter = TRUE,
    .numEventCounters = 3U,
    .eventCounters = gPmuEventCfg,
};

/* Source buffers for all the memcpy operations. They can lie in the OCMC
 * or the same memory as the code like flash. The location can be changed
 * from the linker file
*/

uint32_t buf_0[BUF_SIZE] __attribute__((section(".buf_0"))) ;
uint32_t buf_1[BUF_SIZE] __attribute__((section(".buf_1"))) ;
uint32_t buf_2[BUF_SIZE] __attribute__((section(".buf_2"))) ;
uint32_t buf_3[BUF_SIZE] __attribute__((section(".buf_3"))) ;
uint32_t buf_4[BUF_SIZE] __attribute__((section(".buf_4"))) ;
uint32_t buf_5[BUF_SIZE] __attribute__((section(".buf_5"))) ;
uint32_t buf_6[BUF_SIZE] __attribute__((section(".buf_6"))) ;
uint32_t buf_7[BUF_SIZE] __attribute__((section(".buf_7"))) ;
uint32_t buf_8[BUF_SIZE] __attribute__((section(".buf_8"))) ;
uint32_t buf_9[BUF_SIZE] __attribute__((section(".buf_9"))) ;
uint32_t buf_10[BUF_SIZE] __attribute__((section(".buf_10"))) ;
uint32_t buf_11[BUF_SIZE] __attribute__((section(".buf_11"))) ;
uint32_t buf_12[BUF_SIZE] __attribute__((section(".buf_12"))) ;
uint32_t buf_13[BUF_SIZE] __attribute__((section(".buf_13"))) ;
uint32_t buf_14[BUF_SIZE] __attribute__((section(".buf_14"))) ;
uint32_t buf_15[BUF_SIZE] __attribute__((section(".buf_15"))) ;

/* The target buffer for all the memcpy operations */
uint32_t buf_ocmc[BUF_SIZE] __attribute__((section(".buf_cpy")));

/* Slave Task Function Definition. All the tasks are same
 * functionally. The only differnce is their location in the memory. They
 * are placed in the memory such that all of them occupy the same cache entry
 * in the 4-way cahce. The sections are defined in the linker cmd file*/

void SlaveTaskFxn_0(void * a)  __attribute__((section(".task_0")));
void SlaveTaskFxn_1(void * a)  __attribute__((section(".task_1")));
void SlaveTaskFxn_2(void * a)  __attribute__((section(".task_2")));
void SlaveTaskFxn_3(void * a)  __attribute__((section(".task_3")));
void SlaveTaskFxn_4(void * a)  __attribute__((section(".task_4")));
void SlaveTaskFxn_5(void * a)  __attribute__((section(".task_5")));
void SlaveTaskFxn_6(void * a)  __attribute__((section(".task_6")));
void SlaveTaskFxn_7(void * a)  __attribute__((section(".task_7")));
void SlaveTaskFxn_8(void * a)  __attribute__((section(".task_8")));
void SlaveTaskFxn_9(void * a)  __attribute__((section(".task_9")));
void SlaveTaskFxn_10(void * a)  __attribute__((section(".task_10")));
void SlaveTaskFxn_11(void * a)  __attribute__((section(".task_11")));
void SlaveTaskFxn_12(void * a)  __attribute__((section(".task_12")));
void SlaveTaskFxn_13(void * a)  __attribute__((section(".task_13")));
void SlaveTaskFxn_14(void * a)  __attribute__((section(".task_14")));
void SlaveTaskFxn_15(void * a)  __attribute__((section(".task_15")));


static uint8_t MainApp_TaskStack[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_0[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_1[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_2[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_3[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_4[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_5[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_6[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_7[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_8[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_9[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_10[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_11[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_12[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_13[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_14[TASK_STACK_SIZE] __attribute__((aligned(32)));
static uint8_t SlaveTaskStack_15[TASK_STACK_SIZE] __attribute__((aligned(32)));



uint32_t get_rand()
{
    return (rand()%NUM_TASK);
}

void MasterTask(void * a)
{
    DebugP_log("\r\n\rmaster_task\r\n\r");

    uint32_t icm, ica, icnt;
    msg r[NUM_TASK];
    int i, j;


    for (i = 0; i < NUM_TASK; ++i)
    {
        myQ[i] = QueueP_create(&myQObj[i]);
    }

    DebugP_log("\r\nmaster_task -- start sending\r\n");

    uint32_t count = 0;
    /* this loop is for NUM_TEST with the specified task calls and memcpy_size*/
    for (i = 0; i < NUM_TEST; ++i)
    {
        /* counter to track the task calls already made*/
        count = 0;
        /*reset the value of task switch for that test*/
        num_switches = 0;
        /* number of times each task should repeat its operation*/
        iter = 1;

        /* size of the memcpy to be performaed by each task*/
        memcopy_size = memcopy_size_arr[i];

        /*invalidate all the cache to get fresh and relaible data*/
        CacheP_wbInvAll(CacheP_TYPE_ALL);

        /* reset the PMU counters to get relevant data */
        PMU_profileStart("Program");
        startTime = ClockP_getTimeUsec();

        /*start sending signals and messages to tasks*/
        while(count < task_calls)
        {
            /* Get a random task number*/
            j = get_rand();
            r[j].task_call_number = count++;
            do
            {
                QueueP_get(myQ[j]);
            }
            while(QueueP_EMPTY != QueueP_isEmpty(myQ[j]));
            /* Add the message to the queue of the task */
            QueueP_put(myQ[j], &(r[j].elem));
            /* Signal the task to start executing */
            SemaphoreP_post(&gSemaphorePHandle[j]);
            /* Yield the CPU for the other task to execute*/
            TaskP_yield();
            /* Wait for the task to complete */
            SemaphoreP_pend(&gSemaphorePHandle[j], SystemP_WAIT_FOREVER);
        }

        elapsedTime = ClockP_getTimeUsec() - startTime;
        PMU_profileEnd("Program");
        durationInSecs = ((elapsedTime) / 1000U);
        hrs  = durationInSecs / (60U * 60U);
        mins = (durationInSecs / 60U) - (hrs * 60U);
        secs = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
        usecs = elapsedTime - (((hrs * 60U * 60U) + (mins * 60U) + secs) * 1000000U);
        /* Read the value of the PMU counteres and print the result */
        icm = CSL_armR5PmuReadCntr(0);
        ica = CSL_armR5PmuReadCntr(2);
        icnt = CSL_armR5PmuReadCntr(1);
        DebugP_log("\r\nMem Cpy Size    => %d\r\n", (unsigned int) memcopy_size);
        DebugP_log("Start Time in Usec => %d\r\n", (unsigned int) startTime);
        DebugP_log("Exec Time in Usec => %d\r\n", (unsigned int) elapsedTime);
        DebugP_log("Iter            => %d\r\n", (unsigned int) iter);
        DebugP_log("Task Calls      => %d\r\n",  task_calls);
        DebugP_log("Inst Cache Miss => %d\r\n",  icm);
        DebugP_log("Inst Cache Acc  => %d\r\n",  ica);
        DebugP_log("Num Instr Exec  => %d\r\n",  icnt);
        DebugP_log("ICM/sec         => %u\r\n", (unsigned int) (1000000 * ((float) (icm*1.0)/(elapsedTime*1.0))));
        DebugP_log("INST/sec        => %u\r\n", (unsigned int) (1000000 * ((float) (icnt*1.0)/(elapsedTime*1.0))));
    }

    DebugP_log("\r\nAll tests have passed\r\n");
    vTaskDelete(NULL);
}


void ocmc_benchmarking_main(void *args)
{
    int i,j;

    DebugP_log("\r\nOCMC benchmarking:: Board_init success\r\n");

    PMU_init(&gPmuConfig);

    TaskP_FxnMain tasks[NUM_TASK];
    tasks[0] = SlaveTaskFxn_0;
    tasks[1] = SlaveTaskFxn_1;
    tasks[2] = SlaveTaskFxn_2;
    tasks[3] = SlaveTaskFxn_3;
    tasks[4] = SlaveTaskFxn_4;
    tasks[5] = SlaveTaskFxn_5;
    tasks[6] = SlaveTaskFxn_6;
    tasks[7] = SlaveTaskFxn_7;
    tasks[8] = SlaveTaskFxn_8;
    tasks[9] = SlaveTaskFxn_9;
    tasks[10] = SlaveTaskFxn_10;
    tasks[11] = SlaveTaskFxn_11;
    tasks[12] = SlaveTaskFxn_12;
    tasks[13] = SlaveTaskFxn_13;
    tasks[14] = SlaveTaskFxn_14;
    tasks[15] = SlaveTaskFxn_15;

    void * taskStacks[NUM_TASK];
    taskStacks[0]  = SlaveTaskStack_0;
    taskStacks[1]  = SlaveTaskStack_1;
    taskStacks[2]  = SlaveTaskStack_2;
    taskStacks[3]  = SlaveTaskStack_3;
    taskStacks[4]  = SlaveTaskStack_4;
    taskStacks[5]  = SlaveTaskStack_5;
    taskStacks[6]  = SlaveTaskStack_6;
    taskStacks[7]  = SlaveTaskStack_7;
    taskStacks[8]  = SlaveTaskStack_8;
    taskStacks[9]  = SlaveTaskStack_9;
    taskStacks[10] = SlaveTaskStack_10;
    taskStacks[11] = SlaveTaskStack_11;
    taskStacks[12] = SlaveTaskStack_12;
    taskStacks[13] = SlaveTaskStack_13;
    taskStacks[14] = SlaveTaskStack_14;
    taskStacks[15] = SlaveTaskStack_15;


    buf[0] = buf_0;
    buf[1] = buf_1;
    buf[2] = buf_2;
    buf[3] = buf_3;
    buf[4] = buf_4;
    buf[5] = buf_5;
    buf[6] = buf_6;
    buf[7] = buf_7;
    buf[8] = buf_8;
    buf[9] = buf_9;
    buf[10] = buf_10;
    buf[11] = buf_11;
    buf[12] = buf_12;
    buf[13] = buf_13;
    buf[14] = buf_14;
    buf[15] = buf_15;


    DebugP_log("Filling up the buffers\r\n");
    for (i = 0; i < NUM_TASK; ++i)
    {
        for (j = 0; j < BUF_SIZE; ++j)
        {
            buf[i][j] = j;
        }
    }

    {
        for (i = 0; i < NUM_TASK; ++i)
        {
            SemaphoreP_constructBinary(&gSemaphorePHandle[i], 0);
        }
    }

    {

        TaskP_Params taskParams;
        //TaskP_Object tobj;

        TaskP_Params_init(&taskParams);

        taskParams.priority = 1;
        taskParams.stack = MainApp_TaskStack;
        taskParams.stackSize = sizeof(MainApp_TaskStack);
        taskParams.taskMain = MasterTask;
        TaskP_construct(&main_task[0], &taskParams);


        for (i = 0; i <  NUM_TASK; ++i)
        {
            taskParams.taskMain = (TaskP_FxnMain)tasks[i];
            taskParams.args = (void *) i;
            taskParams.stack = taskStacks[i];
            taskParams.stackSize = TASK_STACK_SIZE;
            TaskP_construct(&main_task[i+1], &taskParams);
        }



    }
}

void ocmc_benchmarking_entry()
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
}

void ocmc_benchmarking_exit()
{
    Board_driversClose();
    Drivers_close();
}


void SlaveTaskFxn_0(void * a) {TSKFN};

void SlaveTaskFxn_1(void * a) {TSKFN};

void SlaveTaskFxn_2(void * a) {TSKFN};

void SlaveTaskFxn_3(void * a) {TSKFN};

void SlaveTaskFxn_4(void * a) {TSKFN};

void SlaveTaskFxn_5(void * a) {TSKFN};

void SlaveTaskFxn_6(void * a) {TSKFN};

void SlaveTaskFxn_7(void * a) {TSKFN};

void SlaveTaskFxn_8(void * a) {TSKFN};

void SlaveTaskFxn_9(void * a) {TSKFN};

void SlaveTaskFxn_10(void * a)  {TSKFN};

void SlaveTaskFxn_11(void * a)  {TSKFN};

void SlaveTaskFxn_12(void * a)  {TSKFN};

void SlaveTaskFxn_13(void * a)  {TSKFN};

void SlaveTaskFxn_14(void * a)  {TSKFN};

void SlaveTaskFxn_15(void * a)  {TSKFN};
