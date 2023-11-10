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
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "stdbool.h"
#include <drivers/soc.h>
#include <drivers/ipc_rpmsg.h>
#include "FreeRTOS.h"
#include "task.h"

#define MAX_MSG_SIZE            (64u)

#define HEARTBEAT_TASK_PRI      (configMAX_PRIORITIES - 5)
#define HEARTBEAT_TASK_SIZE     (8192/sizeof(configSTACK_DEPTH_TYPE))

#define IPC_LOG_TASK_PRI        (configMAX_PRIORITIES - 3)
#define IPC_LOG_TASK_SIZE       (8192/sizeof(configSTACK_DEPTH_TYPE))

#define IPC_RESTART_TASK_PRI       (configMAX_PRIORITIES - 4)
#define IPC_RESTART_TASK_SIZE      (8192/sizeof(configSTACK_DEPTH_TYPE))

#define IPC_RPMESSAGE_NUM_VRING_BUF       (8U)
#define IPC_RPMESSAGE_MAX_VRING_BUF_SIZE  (128U)
#define IPC_RPMESSAGE_NUM_CORES           (2U)
#define IPC_RPMESSAGE_NUM_VRINGS          (IPC_RPMESSAGE_NUM_CORES*(IPC_RPMESSAGE_NUM_CORES-1))
#define IPC_RPMESSAGE_VRING_SIZE          RPMESSAGE_VRING_SIZE(IPC_RPMESSAGE_NUM_VRING_BUF, IPC_RPMESSAGE_MAX_VRING_BUF_SIZE)

uint32_t ipcCount = 0;

SemaphoreP_Object gIPCRecvSem;
SemaphoreP_Object gIPCRestartSem;

StackType_t  gIPCLogTaskStack[IPC_LOG_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gIPCLogTaskObj;
TaskHandle_t gIPCLogTask;

StackType_t  gHeartBeatTaskStack[HEARTBEAT_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gHeartBeatTaskObj;
TaskHandle_t gHeartBeatTask;

StackType_t  gIPCRestartTaskStack[IPC_RESTART_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gIPCRestartTaskObj;
TaskHandle_t gIPCRestartTask;

/* Remote core service end point */
uint16_t gRpmsgEndPointId = 7;
RPMessage_Object gAckReplyMsgObject;

extern uint8_t gIpcSharedMem[];

void resetReqIsr(void *args)
{
    int32_t status;
    uint32_t resetSrc;
    uint32_t pscDomainState;
    uint32_t pscModuleStateMain2MCU = 0;
    uint32_t pscModuleStateMCU2Main = 0;

    {
        char msgBuf[MAX_MSG_SIZE];
        uint16_t msgSize;
        snprintf(msgBuf, MAX_MSG_SIZE-1, "KILL");
        msgBuf[MAX_MSG_SIZE-1] = 0;
        msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */

        status = RPMessage_send(
                msgBuf, msgSize,
                CSL_CORE_ID_R5FSS0_0, gRpmsgEndPointId,
                RPMessage_getLocalEndPt(&gAckReplyMsgObject),
                SystemP_WAIT_FOREVER);
        DebugP_assert (status == SystemP_SUCCESS);
    }

    /* Disable LPSC Main2MCU */
    status = SOC_getPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU,
            CSL_MCU_LPSC_MAIN2MCU, &pscDomainState, &pscModuleStateMain2MCU);
    if ((status == SystemP_SUCCESS) && (pscDomainState == SOC_PSC_DOMAIN_ON)
        && (pscModuleStateMain2MCU != SOC_PSC_DISABLE))
    {
        status = SOC_setPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU, \
                        CSL_MCU_LPSC_MAIN2MCU, SOC_PSC_DISABLE);
    }

    if (status == SystemP_SUCCESS)
    {
        /* Disable LPSC MCU2Main */
        status = SOC_getPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU,
                CSL_MCU_LPSC_MCU2MAIN, &pscDomainState, &pscModuleStateMCU2Main);
        if ((status == SystemP_SUCCESS) && (pscDomainState == SOC_PSC_DOMAIN_ON)
            && (pscModuleStateMCU2Main != SOC_PSC_DISABLE))
        {
            status = SOC_setPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU, \
                            CSL_MCU_LPSC_MCU2MAIN, SOC_PSC_DISABLE);
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Clear reset source */
        resetSrc = SOC_getWarmResetCauseMcuDomain();
        SOC_clearResetCauseMainMcuDomain (resetSrc);

        /* Allow main domain reset to propogate */
        SOC_setMCUResetIsolationDone(0);

        SOC_waitMainDomainReset();

        /* Enable back reset isolation */
        SOC_setMCUResetIsolationDone(1);
    }

    /* Restore back previous state of LSPC Main2MCU */
    if ((status == SystemP_SUCCESS) && (pscModuleStateMain2MCU != SOC_PSC_DISABLE))
    {
        SOC_setPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU, \
                        CSL_MCU_LPSC_MAIN2MCU, pscModuleStateMain2MCU);
    }

    /* Restore back previous state of LSPC MCU2Main */
    if (((status == SystemP_SUCCESS)) && (pscModuleStateMCU2Main != SOC_PSC_DISABLE))
    {
        SOC_setPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU, \
                        CSL_MCU_LPSC_MCU2MAIN, pscModuleStateMCU2Main);
    }

    RPMessage_deInit();
    IpcNotify_deInit();

    SemaphoreP_post(&gIPCRestartSem);
}

void ipcRecvMessage(RPMessage_Object *obj, void *args, void *data, uint16_t dataLen, int32_t crcStatus, \
                            uint16_t remoteCoreId, uint16_t remoteEndPt)
{
    int32_t status;
    uint16_t msgSize;
    char msgBuf[MAX_MSG_SIZE];

    if (strncmp((char*)data, "ALIVE", strlen("ALIVE")) == 0)
    {
        sscanf((char *)data, "ALIVE %d", &ipcCount);
        if ((ipcCount % 10000) == 0)
        {
            SemaphoreP_post(&gIPCRecvSem);
        }
        ipcCount++;

        snprintf(msgBuf, MAX_MSG_SIZE-1, "ALIVE %d", ipcCount);
        msgBuf[MAX_MSG_SIZE-1] = 0;
        msgSize = strlen(msgBuf) + 1; /* count the terminating char as well */

        status = RPMessage_send(
                    msgBuf, msgSize,
                    CSL_CORE_ID_R5FSS0_0, gRpmsgEndPointId,
                    RPMessage_getLocalEndPt(&gAckReplyMsgObject),
                    SystemP_WAIT_FOREVER);
        DebugP_assert (status == SystemP_SUCCESS);
    }
    else if (strncmp((char *)data, "KILL", strlen("KILL")) == 0)
    {
        SOC_generateSwWarmResetMainDomainFromMcuDomain ();
    }
}

void ipcLogTask(void* arg)
{
    volatile int x = 1;

    while (x)
    {
        SemaphoreP_pend(&gIPCRecvSem, SystemP_WAIT_FOREVER);

        DebugP_log("M4F: R5 <--> M4 is communicating --> %d\r\n", ipcCount/10000);
    }
}

void heartBeatTask(void *args)
{
    uint32_t count = 0;

    volatile int x = 1;
    while(x)
    {
        ClockP_sleep(1);
        DebugP_log("I am running (M4) !!:- %d\r\n", count++);
    }
}

void ipcRestartTask(void *args)
{
    volatile int x = 1;

    while (x == 1)
    {
        SemaphoreP_pend(&gIPCRestartSem, SystemP_WAIT_FOREVER);

        /* Wait for firewall unlock from SBL */
        SOC_waitForFwlUnlock();

        /* IPC Notify */
        {
            IpcNotify_Params notifyParams;
            int32_t status;

            /* initialize parameters to default */
            IpcNotify_Params_init(&notifyParams);

            /* specify the core on which this API is called */
            notifyParams.selfCoreId = CSL_CORE_ID_M4FSS0_0;

            /* list the cores that will do IPC Notify with this core
            * Make sure to NOT list 'self' core in the list below
            */
            notifyParams.numCores = 1;
            notifyParams.coreIdList[0] = CSL_CORE_ID_R5FSS0_0;

            /* initialize the IPC Notify module */
            status = IpcNotify_init(&notifyParams);
            DebugP_assert(status==SystemP_SUCCESS);

        }

        /* IPC RPMessage */
        {
            RPMessage_Params rpmsgParams;
            int32_t status;

            /* initialize parameters to default */
            RPMessage_Params_init(&rpmsgParams);

            /* VRING mapping from source core to destination core, '-1' means NO VRING,
                r5fss0_0 => {"r5fss0_0":-1,"m4fss0_0":0}
                m4fss0_0 => {"r5fss0_0":1,"m4fss0_0":-1}
            */
            /* TX VRINGs */
            rpmsgParams.vringTxBaseAddr[CSL_CORE_ID_R5FSS0_0] = (uintptr_t)(&gIpcSharedMem[1312]);
            /* RX VRINGs */
            rpmsgParams.vringRxBaseAddr[CSL_CORE_ID_R5FSS0_0] = (uintptr_t)(&gIpcSharedMem[0]);
            /* Other VRING properties */
            rpmsgParams.vringSize = IPC_RPMESSAGE_VRING_SIZE;
            rpmsgParams.vringNumBuf = IPC_RPMESSAGE_NUM_VRING_BUF;
            rpmsgParams.vringMsgSize = IPC_RPMESSAGE_MAX_VRING_BUF_SIZE;

            /* initialize the IPC RP Message module */
            status = RPMessage_init(&rpmsgParams);
            DebugP_assert(status==SystemP_SUCCESS);
        }

        RPMessage_CreateParams createParams;
        RPMessage_CreateParams_init(&createParams);
        createParams.localEndPt = gRpmsgEndPointId;
        createParams.recvCallback = &ipcRecvMessage;
        int32_t status = RPMessage_construct(&gAckReplyMsgObject, &createParams);
        DebugP_assert (status == SystemP_SUCCESS);

        IpcNotify_syncAll(SystemP_WAIT_FOREVER);
    }
}

void reset_isolation_main (void * args)
{
    int32_t status = SystemP_FAILURE;
    uint32_t pscMain2MCUDisable, pscMCU2MainDisable, debugIsolationEnable;
    RPMessage_CreateParams createParams;

    Drivers_open();
    Board_driversOpen();

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = gRpmsgEndPointId;
    createParams.recvCallback = &ipcRecvMessage;
    status = RPMessage_construct(&gAckReplyMsgObject, &createParams);
    DebugP_assert (status == SystemP_SUCCESS);

    status = SemaphoreP_constructBinary(&gIPCRecvSem, 0);
    DebugP_assert(status == SystemP_SUCCESS);

    status = SemaphoreP_constructBinary(&gIPCRestartSem, 0);
    DebugP_assert(status == SystemP_SUCCESS);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Disabling Main2MCU PSC. This would restrict the main domain from accessing
    MCU domain peripherals/registers. Care must be taken no Main domain cores access
    MCU domain registers after this */
    pscMain2MCUDisable = 1;
    /* Disabling MCU2Main PSC. This would restrict the MCU domain from accessing
    Main domain peripherals/registers. Care must be taken no MCU domain cores access
    Main domain registers after this */
    pscMCU2MainDisable = 0;
    /* Enabling debug isolation will restrict the JTAG access to MCU domain */
    debugIsolationEnable = 1;

    status = SOC_enableResetIsolation (pscMain2MCUDisable, pscMCU2MainDisable,
                                            debugIsolationEnable);
    DebugP_assert (status == SystemP_SUCCESS);

    {
        HwiP_Params resetHwiParams;
        HwiP_Object resetObject;

        HwiP_Params_init(&resetHwiParams);
        resetHwiParams.intNum =         \
        CSLR_MCU_M4FSS0_CORE0_NVIC_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_RESETZ_SYNC_STRETCH_0 + 16;
        resetHwiParams.callback = resetReqIsr;
        resetHwiParams.isPulse = 0;
        HwiP_construct(&resetObject, &resetHwiParams);
    }

    /* Start IPC Log task */
    {
        gIPCLogTask = xTaskCreateStatic( ipcLogTask,
                                    "IPC Log Task",
                                    IPC_LOG_TASK_SIZE,
                                    NULL,
                                    IPC_LOG_TASK_PRI,
                                    gIPCLogTaskStack,
                                    &gIPCLogTaskObj );
        configASSERT(gIPCLogTask != NULL);
    }

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


    /* Start Task to restart IPC */
    {
        gIPCRestartTask = xTaskCreateStatic( ipcRestartTask,
                                    "IPC Restart Task",
                                    IPC_RESTART_TASK_SIZE,
                                    NULL,
                                    IPC_RESTART_TASK_PRI,
                                    gIPCRestartTaskStack,
                                    &gIPCRestartTaskObj );
        configASSERT(gIPCRestartTask != NULL);
    }

    /* Board_driversClose(); */
    /* Drivers_close(); */
}
