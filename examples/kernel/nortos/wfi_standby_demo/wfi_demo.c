/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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
#include <inttypes.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CpuIdP.h>
#include <drivers/ipc_notify.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#if defined(SOC_AM64X) || defined (SOC_AM243X)
#define TIMER_INTERRUPT (CSLR_R5FSS0_CORE0_INTR_TIMER10_INTR_PEND_0)
#endif

#if defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define TIMER_INTERRUPT (CSLR_R5FSS1_CORE0_INTR_RTI2_INTR_0)
#endif
/* client ID that is used to send and receive messages */
uint32_t gClientId = 4u;

#if defined(SOC_AM64X) || defined (SOC_AM243X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core */
uint32_t gRemoteCoreId = CSL_CORE_ID_R5FSS1_0;
#endif

#if defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core */
uint32_t gRemoteCoreId = CSL_CORE_ID_R5FSS1_0;
#endif

/* semaphore used to indicate a remote core */
SemaphoreP_Object gRemoteDoneSem;

void wfe_demo_handler_remote_core(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    HwiP_enableInt(TIMER_INTERRUPT);
    SemaphoreP_post(&gRemoteDoneSem);
}


void wfi_standby_demo_main(void *args)
{
    int32_t status;
    uint32_t msgValue = 0;

    Drivers_open();
    Board_driversOpen();

    SemaphoreP_constructBinary(&gRemoteDoneSem, 0);

    /* register a handler to receive messages */
    status = IpcNotify_registerClient(gClientId, wfe_demo_handler_remote_core, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* wait for all messages to be echo'ed back */
    SemaphoreP_pend(&gRemoteDoneSem, SystemP_WAIT_FOREVER);
    /* Disable the any pending interrupt before executing wfi call */
    HwiP_disableInt(TIMER_INTERRUPT);
    status = IpcNotify_sendMsg(gMainCoreId, gClientId, msgValue, 1);
    CSL_armR5SetWFIMode();
    /* wait for all messages to be echo'ed back */
    SemaphoreP_pend(&gRemoteDoneSem, SystemP_WAIT_FOREVER);

    Board_driversClose();
    Drivers_close();
}
