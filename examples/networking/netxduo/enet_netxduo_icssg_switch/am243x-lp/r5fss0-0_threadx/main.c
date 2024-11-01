/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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

#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include <tx_api.h>


#define MAIN_TASK_PRI  (1)

#define MAIN_TASK_STACK_SIZE (8192U)

uint8_t main_thread_stack[MAIN_TASK_STACK_SIZE] __attribute__((aligned(32)));

TX_THREAD main_thread;

void netxduo_icssg_main(ULONG arg);

void threadx_main(ULONG arg)
{
    netxduo_icssg_main(arg);
}


int main(void)
{
    /* init SOC specific modules */
    System_init();
    Board_init();

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

void tx_application_define(void *first_unused_memory)
{
    UINT status;

    status = tx_thread_create(&main_thread,           /* Pointer to the main thread object. */ 
                              "main_thread",          /* Name of the task for debugging purposes. */
                              threadx_main,           /* Entry function for the main thread. */
                              0,                      /* Arguments passed to the entry function. */  
                              main_thread_stack,      /* Main thread stack. */
                              MAIN_TASK_STACK_SIZE,   /* Main thread stack size in bytes. */
                              MAIN_TASK_PRI,          /* Main task priority. */
                              MAIN_TASK_PRI,          /* Highest priority level of disabled preemption. */
                              TX_NO_TIME_SLICE,       /* No time slice. */
                              TX_AUTO_START);         /* Start immediately. */

    DebugP_assertNoLog(status == TX_SUCCESS);
}


