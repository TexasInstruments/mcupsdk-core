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
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/spinlock.h>
#include <drivers/ipc_notify.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example shows shared memory access between multiple cores in an atomic way using spinlock.
 *
 * Shared memory is allocated across two cores - using gUserSharedMem variable.
 * In this example, this memory is just a counter for illustration purpose.
 * Each core does a read-modify write to this variable in a continuous manner.
 *
 * Spinlocks are used to protect this shared memory access. This ensures
 * that the access is atomic and the shared variable is incremented properly.
 * If proper protection is done, then at the end of the iteration the counter
 * value will be N times loop count where N is number of cores acting on the
 * variable.
 *
 * When iteration count reaches SPINLOCK_TEST_LOOPCNT, the example is completed.
 *
 */

#define SPINLOCK_BASE_ADDR      (CSL_SPINLOCK0_BASE)
#define SPINLOCK_LOCK_NUM       (0U)
#define SPINLOCK_TEST_LOOPCNT   (100000U)
#define SPINLOCK_TEST_NUM_CORES (2U)

volatile uint32_t gUserSharedMem __attribute__((aligned(128), section(".bss.user_shared_mem")));

static void app_ipc_sharemem_lock(SemaphoreP_Object *mutexObj,
                                  uint32_t spinlockBaseAddr,
                                  uint32_t lockNum)
{
    int32_t  status;

    /* Take local mutex to protect against multi-thread from same core */
    SemaphoreP_pend(mutexObj, SystemP_WAIT_FOREVER);

    /* Spin till lock is acquired */
    while(1U)
    {
        status = Spinlock_lock(spinlockBaseAddr, lockNum);
        if(status == SPINLOCK_LOCK_STATUS_FREE)
        {
            break;  /* Free and taken */
        }
        /* Note: Customers can implement timeout instead of wait forever */
    }

    return;
}

static void app_ipc_sharemem_unlock(SemaphoreP_Object *mutexObj,
                                    uint32_t spinlockBaseAddr,
                                    uint32_t lockNum)
{
    /* Free the aquired lock and release local mutex */
    Spinlock_unlock(spinlockBaseAddr, lockNum);
    SemaphoreP_post(mutexObj);

    return;
}

void ipc_spinlock_sharedmem_main(void *args)
{
    uint32_t            spinlockBaseAddr;
    int32_t             status;
    uint32_t            lockNum, loopCnt;
    SemaphoreP_Object   mutexObj;

    lockNum = SPINLOCK_LOCK_NUM;
    gUserSharedMem = 0;     /* All cores will init this to zero before start */

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[IPC Spinlock Sharedmem] Example started ...\r\n");
    DebugP_log("Waiting for all cores to start ...\r\n");

    /* Wait for all cores to start */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Get address after translation translate */
    spinlockBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(SPINLOCK_BASE_ADDR);

    /* Mutex for local thread safe operation on spinlock */
    status = SemaphoreP_constructMutex(&mutexObj);
    DebugP_assert(status==SystemP_SUCCESS);

    for(loopCnt = 0; loopCnt < SPINLOCK_TEST_LOOPCNT; loopCnt++)
    {
        app_ipc_sharemem_lock(&mutexObj, spinlockBaseAddr, lockNum);

        /*
         * Critical section goes here!!
         */
        /* Read/modify shared memory */
        gUserSharedMem++;

        app_ipc_sharemem_unlock(&mutexObj, spinlockBaseAddr, lockNum);
    }

    SemaphoreP_destruct(&mutexObj);

    /* Wait for all cores before checking results */
    DebugP_log("Waiting for all cores to complete ...\r\n");
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    if(gUserSharedMem == (SPINLOCK_TEST_LOOPCNT * SPINLOCK_TEST_NUM_CORES))
    {
        /* Shared mem count incremented as expected */
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Counter mismatch. Shared mem access was not atomic!!\r\n");
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    /* We dont close drivers to let the UART driver remain open and flush any pending messages to console */
    //Drivers_close();

    return;
}
