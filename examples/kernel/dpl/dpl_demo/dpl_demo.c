/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/CycleCounterP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* buffer used for cache operations, MUST be cache line aligned */
#define MY_BUF_SIZE    (1024u)
static uint8_t gMyBuf[MY_BUF_SIZE] __attribute__((aligned(32)));

/* User defined heap memory and handle */
#define MY_HEAP_MEM_SIZE  (2*1024u)
static uint8_t gMyHeapMem[MY_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

static HeapP_Object gMyHeapObj;

/* user defined ISR and semaphore to signal from ISR to main thread */
static SemaphoreP_Object gMyISRDoneSem;

static void myISR(void *arg)
{
    SemaphoreP_post(&gMyISRDoneSem);
}

void dpl_demo_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* eample usage of Hwi and Sempahore APIs */
    {
        HwiP_Params hwiParams;
        HwiP_Object hwiObj;
        int32_t status;

        SemaphoreP_constructBinary(&gMyISRDoneSem, 0);

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = 20;
        hwiParams.callback = myISR;
        HwiP_construct(&hwiObj, &hwiParams);

        DebugP_log("[DPL] Hwi post ...\r\n");

        HwiP_post(hwiParams.intNum);

        status = SemaphoreP_pend(&gMyISRDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(status==SystemP_SUCCESS);

        DebugP_log("[DPL] Hwi post ... DONE !!!\r\n");

        HwiP_destruct(&hwiObj);
        SemaphoreP_destruct(&gMyISRDoneSem);
    }

    /* example usage of Clock and time measurement APIs */
    {
        uint32_t cycleCount, sleepTimeInMs = 100;
        uint64_t curTime;

        DebugP_log("[DPL] Sleep for %d msecs ... \r\n", sleepTimeInMs);

        CycleCounterP_reset(); /* enable and reset CPU cycle counter */

        curTime = ClockP_getTimeUsec(); /* get time as measured by timer associated with ClockP module */
        cycleCount = CycleCounterP_getCount32(); /* get CPU cycle count */

        ClockP_usleep(sleepTimeInMs*1000); /* sleep for few usecs */

        cycleCount = CycleCounterP_getCount32() - cycleCount; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */
        curTime = ClockP_getTimeUsec() - curTime; /* get time and calculate diff, ClockP returns 64b value so there wont be overflow here */

        DebugP_log("[DPL] Sleep ... DONE (Measured time = %d usecs, CPU cycles = %d ) !!!\r\n",
            (uint32_t)curTime, cycleCount);
        DebugP_log("Note: In case of FREERTOS CPU Cycles will not match with the measured time\r\n"
                   "\tAs the \"WFI\" instruction is called from idle task, which suspends the PMU counter used to measure CPU cycles.");
    }

    /* example usage of Cache APIs */
    {
        void *ptr = &gMyBuf[0] ;

        DebugP_log("[DPL] Running cache operations ...\r\n");

        CacheP_inv(ptr, MY_BUF_SIZE, CacheP_TYPE_ALL);
        memset(ptr, 0xFF, MY_BUF_SIZE);
        CacheP_wb(ptr, MY_BUF_SIZE, CacheP_TYPE_ALL);

        DebugP_log("[DPL] Running cache operations ... DONE !!!\r\n");
    }

    /* example usage of Heap APIs */
    {
        void *ptr1;
        uint32_t size1 = 1023u;

        /* create heap */
        HeapP_construct(&gMyHeapObj, gMyHeapMem, MY_HEAP_MEM_SIZE);

        DebugP_log("[DPL] Heap free size = %d bytes\r\n",
            (uint32_t)HeapP_getFreeHeapSize(&gMyHeapObj)
            );

        /* allocate memory from heap */
        ptr1 = HeapP_alloc(&gMyHeapObj, size1);
        DebugP_assert(ptr1!=NULL);

        /* use the memory */
        memset(ptr1, 0xAA, size1);
        DebugP_log("[DPL] Allocated %d bytes @ 0x%08x, heap free size = %d bytes\r\n",
            size1, ptr1,
            (uint32_t)HeapP_getFreeHeapSize(&gMyHeapObj)
            );

        /* free the memory back to the same heap */
        HeapP_free(&gMyHeapObj, ptr1);
        DebugP_log("[DPL] Free'ed %d bytes @ 0x%08x, heap free size = %d bytes\r\n",
            size1, ptr1,
            (uint32_t)HeapP_getFreeHeapSize(&gMyHeapObj)
            );

        HeapP_destruct(&gMyHeapObj);
    }

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

