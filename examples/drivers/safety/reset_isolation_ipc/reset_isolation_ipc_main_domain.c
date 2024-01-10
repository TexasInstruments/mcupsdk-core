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

#include <stdio.h>
#include <string.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/soc.h>
#include <drivers/ipc_rpmsg.h>
#include <inttypes.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "FreeRTOS.h"
#include "task.h"

#define HEARTBEAT_TASK_PRI  (configMAX_PRIORITIES - 5)
#define HEARTBEAT_TASK_SIZE (8192/sizeof(configSTACK_DEPTH_TYPE))

#define IPC_TASK_PRI  (configMAX_PRIORITIES - 3)
#define IPC_TASK_SIZE (8192/sizeof(configSTACK_DEPTH_TYPE))

#define MAX_MSG_SIZE        (64u)

StackType_t gHeartBeatTaskStack[HEARTBEAT_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gHeartBeatTaskObj;
TaskHandle_t gHeartBeatTask;

StackType_t gIPCTaskStack[IPC_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gIPCTaskObj;
TaskHandle_t gIPCTask;

/* Remote core service end point */
uint16_t gRpmsgEndPointId = 7;
RPMessage_Object gAckReplyMsgObject;

uint32_t            gGpioBaseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;
HwiP_Object         gGpioHwiObject;

extern void Board_gpioInit(void);
extern void Board_gpioDeinit(void);
extern uint32_t Board_getGpioButtonIntrNum(void);
extern uint32_t Board_getGpioButtonSwitchNum(void);

static void GPIO_bankIsrFxn(void *args)
{
    uint32_t    pinNum = (uint32_t) args;
    uint32_t    bankNum =  GPIO_GET_BANK_INDEX(pinNum);
    uint32_t    intrStatus, pinMask = GPIO_GET_BANK_BIT_MASK(pinNum);
    /* Get and clear bank interrupt status */
    intrStatus = GPIO_getBankIntrStatus(gGpioBaseAddr, bankNum);
    GPIO_clearBankIntrStatus(gGpioBaseAddr, bankNum, intrStatus);

    /* Send IPC Rpmsg to MCU domain core to initiate a reset */
    if(intrStatus & pinMask)
    {
        char msgBuf[MAX_MSG_SIZE];
        uint16_t msgSize;
        snprintf(msgBuf, MAX_MSG_SIZE-1, "KILL");
        msgBuf[MAX_MSG_SIZE-1] = 0;
        msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */

        RPMessage_send(msgBuf, msgSize,
                CSL_CORE_ID_M4FSS0_0, gRpmsgEndPointId,
                RPMessage_getLocalEndPt(&gAckReplyMsgObject),
                SystemP_WAIT_FOREVER);
    }
}

void shutdownFunc (void)
{
    /* Disable all interrupts */
    HwiP_disable();

    Board_gpioDeinit();
    Board_driversClose();
    Drivers_close();

    Board_deinit();
    System_deinit();

    __asm__ __volatile__ ("wfi" "\n\t": : : "memory");
}

void heartBeatTask(void *args)
{
    uint32_t count = 0;

    volatile int x = 1;
    while(x)
    {
        ClockP_sleep(1);
        DebugP_log("I am running (R5) !!:- %d\r\n", count++);
    }
}

void ipcTask(void *args)
{
    volatile int x = 1;
    int32_t status = 0;
    char msgBuf[MAX_MSG_SIZE];
    uint16_t msgSize, remoteCoreId, remoteCoreEndPt;
    uint32_t count = 0;

    while (x)
    {
        msgSize = sizeof(msgBuf);
        status = RPMessage_recv(&gAckReplyMsgObject,
                msgBuf, &msgSize,
                &remoteCoreId, &remoteCoreEndPt,
                SystemP_WAIT_FOREVER);
        DebugP_assert (status == SystemP_SUCCESS);

        if(strncmp(msgBuf, "KILL", 4) == 0)
        {
            shutdownFunc();
        }
        else
        {
            ClockP_usleep(100);
            sscanf (msgBuf, "ALIVE %" PRIu32, &count);

            if ((count % 10000) == 0)
            {
                DebugP_log ("R5F: R5 <--> M4 is communicating --> %d\r\n", count/10000);
            }

            snprintf(msgBuf, MAX_MSG_SIZE-1, "ALIVE %" PRIu32, count);
            msgBuf[MAX_MSG_SIZE-1] = 0;
            msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */
            status = RPMessage_send(
                    msgBuf, msgSize,
                    CSL_CORE_ID_M4FSS0_0, gRpmsgEndPointId,
                    RPMessage_getLocalEndPt(&gAckReplyMsgObject),
                    SystemP_WAIT_FOREVER);
            DebugP_assert (status == SystemP_SUCCESS);
        }
    }

}

void reset_isolation_main(void *args)
{
    int32_t         retVal;
    uint32_t        pinNum, intrNum;
    uint32_t        bankNum;
    HwiP_Params     hwiPrms;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    RPMessage_CreateParams createParams;
    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = gRpmsgEndPointId;
    int32_t status = RPMessage_construct(&gAckReplyMsgObject, &createParams);
    DebugP_assert (status == SystemP_SUCCESS);

    Board_gpioInit();

    pinNum          = GPIO_PUSH_BUTTON_PIN;
    intrNum         = Board_getGpioButtonIntrNum();
    bankNum         = GPIO_GET_BANK_INDEX(pinNum);

    /* Address translate */
    gGpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gGpioBaseAddr);

    /* Setup GPIO for interrupt generation */
    GPIO_setDirMode(gGpioBaseAddr, pinNum, GPIO_PUSH_BUTTON_DIR);
    GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_PUSH_BUTTON_TRIG_TYPE);
    GPIO_bankIntrEnable(gGpioBaseAddr, bankNum);

    /* Register pin interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = &GPIO_bankIsrFxn;
    hwiPrms.args     = (void *) pinNum;
    retVal = HwiP_construct(&gGpioHwiObject, &hwiPrms);
    DebugP_assert(retVal == SystemP_SUCCESS );

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log ("Starting R5 \r\n");
    DebugP_log("Press and release SW%d button on EVM to trigger warm reset from SW...\r\n", Board_getGpioButtonSwitchNum());
    DebugP_log("Press and release SW4 button on EVM to trigger a warm reset from HW..\r\n");

    DebugP_log("\r\n");

    /* Start Heartbeat Task */
    {
        /* This task is created at highest priority, it should create more tasks and then delete itself */
        gHeartBeatTask = xTaskCreateStatic( heartBeatTask,
                                    "HeartBeat Task",
                                    HEARTBEAT_TASK_SIZE,
                                    NULL,
                                    HEARTBEAT_TASK_PRI,
                                    gHeartBeatTaskStack,
                                    &gHeartBeatTaskObj );
        configASSERT(gHeartBeatTask != NULL);
    }

    /* Initiate transaction */
    {
        uint16_t msgSize;
        char msgBuf[MAX_MSG_SIZE];

        snprintf(msgBuf, MAX_MSG_SIZE-1, "ALIVE %" PRIu32, (uint32_t)0);
        msgBuf[MAX_MSG_SIZE-1] = 0;
        msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */

        status = RPMessage_send(
                    msgBuf, msgSize,
                    CSL_CORE_ID_M4FSS0_0, gRpmsgEndPointId,
                    RPMessage_getLocalEndPt(&gAckReplyMsgObject),
                    SystemP_WAIT_FOREVER);
        DebugP_assert (status == SystemP_SUCCESS);
    }

    /* Start IPC task */
    {
        gIPCTask = xTaskCreateStatic( ipcTask,
                                    "IPC Task",
                                    IPC_TASK_SIZE,
                                    NULL,
                                    IPC_TASK_PRI,
                                    gIPCTaskStack,
                                    &gIPCTaskObj );
        configASSERT(gIPCTask != NULL);
    }

    /* We are not closing the drivers as the drivers will be used by tasks */
    /* Board_gpioDeinit(); */
    /* Board_driversClose(); */
    /* Drivers_close(); */
}
