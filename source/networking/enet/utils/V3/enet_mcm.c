/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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

/*!
 * \file enet_mcm.c
 *
 * \brief This file contains Multi-client Manager related functionality.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/* This is needed for memset/memcpy */
#include <string.h>

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>

#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_appmemutils.h>
#include <networking/enet/utils/include/enet_appsoc.h>
#include <networking/enet/utils/include/enet_mcm.h>

#include <include/core/enet_soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENETMCM_TSK_STACK_MAIN              (3 * 1024)
#define ENETMCM_PERIODIC_TICK_TSK_STACK     (3 * 1024)
#define ENETMCM_ASYNC_IOCTL_TSK_STACK       (2 * 1024)

#define ENETMCM_MBOX_MSG_COUNT              (10U)
#define ENETMCM_TSK_PRIORITY                (2U)
#define ENETMCM_PERIODICTSK_PRIORITY        (7U)
#define ENETMCM_ASYNCIOCTLTASK_PRIORITY     (9U)

#ifdef NULL_PTR
#undef NULL_PTR
#endif

#define NULL_PTR                            ((void *)NULL)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/*!
 * \brief CPSW MCM handle
 *
 * CPSW MCM opaque handle.
 */
typedef struct EnetMcm_Obj_s *EnetMcm_Handle;

typedef struct EnetMcm_CoreAttachTableEntry_s
{
    uint32_t coreId;
    EnetPer_AttachCoreOutArgs coreInfo;
}  EnetMcm_CoreAttachTableEntry;

typedef struct EnetMcm_CoreAttachTable_s
{
    uint32_t numCoresAttached;
    EnetMcm_CoreAttachTableEntry entry[ENET_CFG_REMOTE_CLIENT_CORES_MAX];
}  EnetMcm_CoreAttachTable;

typedef struct EnetMcm_Obj_s
{
    bool isInitDone;

    uint8_t refCnt;

    Enet_Type enetType;

    uint32_t instId;

    Enet_Handle hEnet;

    Udma_DrvHandle hUdmaDrv;

    uint32_t selfCoreId;

    TaskP_Object taskObj;

    QueueHandle_t hMboxCmd;

    QueueHandle_t hMboxResponse;

    ClockP_Object timerObj;

    TaskP_Object task_periodicTickObj;

    SemaphoreP_Object timerSemObj;

    volatile bool timerTaskShutDownFlag;

    SemaphoreP_Object mutexObj;

    SemaphoreP_Object * hMutex;

    uint32_t periodicTaskPeriod;

    EnetMcm_CoreAttachTable coreAttachTable;

    Enet_Print print;

    /* Async IOCTL handling */

    /*! Whether an ICSSG async IOCTL is complete or not */
    volatile bool asyncIoctlDone;

    /*! Flag to indicate that async IOCTL task should be shut down */
    bool asyncIoctlTaskShutdown;

    /*! Async IOCTL task that pends on asyncIoctlSem and then indicates
     *  IOCTL completion */
    TaskP_Object asyncIoctlTaskObj;

    /*! Semaphore used to wait for async IOCTL completion. It's posted from
     *  async IOCTL callback which is called from Enet_poll() */
    SemaphoreP_Object asyncIoctlSemObj;

    /* Disable Enet_periodic tick function */
    bool disablePeriodicFxn;

    uint8_t mcmMainTaskStack[ENETMCM_TSK_STACK_MAIN] __attribute__ ((aligned(32)));

    uint8_t mcmPrdTaskStack[ENETMCM_PERIODIC_TICK_TSK_STACK] __attribute__ ((aligned(32)));

    uint8_t asyncIoctlTaskStack[ENETMCM_ASYNC_IOCTL_TSK_STACK] __attribute__ ((aligned(32)));

}EnetMcm_Obj;

typedef enum EnetMcm_Command_e
{
    /*! GET CPSW & UDMA HANDLE */
    MCM_GET_HANDLE,

    /*! RELEASE HANDLE */
    MCM_RELEASE_HANDLE,

    /*! ATTACH core */
    MCM_CORE_ATTACH,

    /*! DETACH core */
    MCM_CORE_DETACH,

    /*! Run IOCTL. Mainly useful for asynchronous IOCTLs */
    MCM_IOCTL,

    /*! SHUTDOWN MCM */
    MCM_SHUTDOWN,

    /*! Response to MCM_GET_HANDLE command */
    MCM_RESPONSE_GET_HANDLE,

    /*! Response to MCM_RELEASE_HANDLE command */
    MCM_RESPONSE_RELEASE_HANDLE,

    /*! Response to MCM_CORE_ATTACH command */
    MCM_RESPONSE_CORE_ATTACH,

    /*! Response to MCM_CORE_DETACH command */
    MCM_RESPONSE_CORE_DETACH,

    /*! Response to MCM_IOCL command */
    MCM_RESPONSE_IOCTL,

    /*! Response to SHUTDOWN MCM */
    MCM_RESPONSE_SHUTDOWN,
}
EnetMcm_Command;

typedef struct EnetMcm_mailboxObj_s
{
    EnetMcm_Command cmd;

    struct EnetMcm_mailboxMsgs_s
    {
        uint32_t coreId;
        uint32_t coreKey;
        uint32_t ioctlStatus;
        uint32_t ioctlCmd;
        Enet_IoctlPrms *ioctlPrms;
        EnetMcm_HandleInfo handleInfo;
        EnetPer_AttachCoreOutArgs attachInfo;
    } msgBody;
}EnetMcm_mailboxObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t  EnetMcm_open(EnetMcm_Handle hMcm);

static void     EnetMcm_close(EnetMcm_Handle hMcm);

static void     EnetMcm_serverTask(void * hMcm);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static EnetMcm_Obj gMcmObj[] =
{
    [0U] =
    {
        .isInitDone             = false,
        .timerTaskShutDownFlag  = false,
        .hMutex                 = (SemaphoreP_Object *) NULL_PTR,
        .hMboxCmd               = (QueueHandle_t) NULL_PTR,
        .hMboxResponse          = (QueueHandle_t) NULL_PTR,
        .asyncIoctlDone         = false,
        .asyncIoctlTaskShutdown = false,
    },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#if (ENET_ENABLE_PER_ICSSG == 1)
static void EnetMcm_evtCb(Enet_Event evt,
                          uint32_t evtNum,
                          void *evtCbArgs,
                          void *arg1,
                          void *arg2)
{
    if (ENET_EVT_ASYNC_CMD_RESP == evt)
    {
        SemaphoreP_Object * asyncIoctlSem = (SemaphoreP_Object *)evtCbArgs;
        SemaphoreP_post(asyncIoctlSem);
    }
    else
    {
        EnetAppUtils_print("Unhandled Event: %d\n", evtNum);
    }
}
#endif

static void EnetMcm_asyncIoctlTask(void * arg)
{
    EnetMcm_Handle hMcm = (EnetMcm_Handle)arg;
    SemaphoreP_Object * asyncIoctlSem = &hMcm->asyncIoctlSemObj;

    while (!hMcm->asyncIoctlTaskShutdown)
    {
        SemaphoreP_pend(asyncIoctlSem, SystemP_WAIT_FOREVER);
        hMcm->asyncIoctlDone = true;
    }
}

static void EnetMcm_initAttachTable(EnetMcm_CoreAttachTable *attachTbl)
{
    uint32_t i;

    attachTbl->numCoresAttached = 0;
    for (i = 0; i < ENET_ARRAYSIZE(attachTbl->entry); i++)
    {
        attachTbl->entry[i].coreId = ENET_RM_INVALIDCORE;
    }
}

static EnetMcm_CoreAttachTableEntry *EnetMcm_getCoreAttachEntry(EnetMcm_CoreAttachTable *attachTbl,
                                                                uint32_t coreId)
{
    uint32_t i;
    EnetMcm_CoreAttachTableEntry *entry;

    entry = (EnetMcm_CoreAttachTableEntry *) NULL_PTR;
    for (i = 0; i < attachTbl->numCoresAttached; i++)
    {
        if (attachTbl->entry[i].coreId == coreId)
        {
            entry = &attachTbl->entry[i];
            break;
        }
    }

    return entry;
}

static void EnetMcm_addCoreAttachEntry(EnetMcm_CoreAttachTable *attachTbl,
                                       uint32_t coreId,
                                       EnetPer_AttachCoreOutArgs *coreInfo)
{
    EnetMcm_CoreAttachTableEntry *entry;

    entry = EnetMcm_getCoreAttachEntry(attachTbl, coreId);
    if (entry == NULL_PTR)
    {
        EnetAppUtils_assert(attachTbl->numCoresAttached < ENET_ARRAYSIZE(attachTbl->entry));
        attachTbl->entry[attachTbl->numCoresAttached].coreId   = coreId;
        attachTbl->entry[attachTbl->numCoresAttached].coreInfo = *coreInfo;
        attachTbl->numCoresAttached++;
    }
    else
    {
        uint32_t i;

        EnetAppUtils_assert(entry->coreId == coreId);
        EnetAppUtils_assert(entry->coreInfo.coreKey == coreInfo->coreKey);
        EnetAppUtils_assert(entry->coreInfo.rxMtu == coreInfo->rxMtu);
        for (i = 0; i < ENET_ARRAYSIZE(entry->coreInfo.txMtu); i++)
        {
            EnetAppUtils_assert(entry->coreInfo.txMtu[i] == coreInfo->txMtu[i]);
        }
    }
}

static void EnetMcm_delCoreAttachEntry(EnetMcm_CoreAttachTable *attachTbl,
                                       uint32_t coreId,
                                       uint32_t coreKey)
{
    uint32_t i;
    EnetMcm_CoreAttachTableEntry *entry;

    entry = EnetMcm_getCoreAttachEntry(attachTbl, coreId);
    if (entry != NULL_PTR)
    {
        EnetAppUtils_assert(attachTbl->numCoresAttached > 0);
        for (i = 0; i < attachTbl->numCoresAttached; i++)
        {
            if (attachTbl->entry[i].coreId == coreId)
            {
                break;
            }
        }

        EnetAppUtils_assert(i < attachTbl->numCoresAttached);
        EnetAppUtils_assert(attachTbl->entry[i].coreId == coreId);
        EnetAppUtils_assert(attachTbl->entry[i].coreInfo.coreKey == coreKey);
        /* Move the last entry to the freed entry */
        attachTbl->numCoresAttached--;
        attachTbl->entry[i] = attachTbl->entry[attachTbl->numCoresAttached];
    }
}

static void EnetMcm_initMbox(EnetMcm_Handle hMcm)
{
    TaskP_Params tskParams;
    int32_t status;


    hMcm->hMboxCmd = xQueueCreate(ENETMCM_MBOX_MSG_COUNT,sizeof(EnetMcm_mailboxObj));
    EnetAppUtils_assert(hMcm->hMboxCmd != NULL_PTR);

    hMcm->hMboxResponse = xQueueCreate(ENETMCM_MBOX_MSG_COUNT, sizeof(EnetMcm_mailboxObj));
    EnetAppUtils_assert(hMcm->hMboxResponse != NULL_PTR);

    TaskP_Params_init(&tskParams);

    tskParams.priority  = ENETMCM_TSK_PRIORITY;
    tskParams.stack     = &hMcm->mcmMainTaskStack[0];
    tskParams.stackSize = ENETMCM_TSK_STACK_MAIN;
    tskParams.args      = (void *)hMcm;

    switch (hMcm->enetType)
    {
        case ENET_CPSW_3G:
            tskParams.name = "MCM3G_Task";
            break;

#if (ENET_ENABLE_PER_ICSSG == 1)
        case ENET_ICSSG_DUALMAC:
            tskParams.name = "MCMICSSG_Task";
            break;
        case ENET_ICSSG_SWITCH:
            tskParams.name = "MCMICSSG_SWITCH_Task";
            break;
#endif

        default:
            EnetAppUtils_assert(false);
    }

    tskParams.taskMain = &EnetMcm_serverTask;
    status = TaskP_construct(&hMcm->taskObj, &tskParams);

    EnetAppUtils_assert(status == SystemP_SUCCESS);
}

static int32_t EnetMcm_validateInitCfg(const EnetMcm_InitConfig *pMcmInitCfg)
{
    int32_t status = ENET_SOK;

    if (pMcmInitCfg != NULL_PTR)
    {
        if (pMcmInitCfg->hUdmaDrv == NULL_PTR)
        {
            status = ENET_EINVALIDPARAMS;
        }
    }
    else
    {
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

int32_t  EnetMcm_init(const EnetMcm_InitConfig *pMcmInitCfg)
{
    int32_t status = ENET_SOK;
    uintptr_t key;
    Enet_Type enetType  = pMcmInitCfg->enetType;
    EnetMcm_Handle hMcm;

    key = HwiP_disable();

    hMcm = &gMcmObj[0U];
    if (hMcm->hMutex == NULL_PTR)
    {
        int32_t mutexStatus = SemaphoreP_constructMutex(&hMcm->mutexObj);
        DebugP_assert(mutexStatus == SystemP_SUCCESS);
        hMcm->hMutex = &hMcm->mutexObj;
    }

    HwiP_restore(key);
    EnetAppUtils_assert(hMcm->hMutex != NULL_PTR);
    SemaphoreP_pend(hMcm->hMutex, SystemP_WAIT_FOREVER);

    if (hMcm->isInitDone == false)
    {
        status = EnetMcm_validateInitCfg(pMcmInitCfg);

        if (status == ENET_SOK)
        {
            hMcm->selfCoreId = pMcmInitCfg->selfCoreId;
            hMcm->refCnt             = 0U;
            hMcm->enetType           = enetType;
            hMcm->instId             = pMcmInitCfg->instId;
            hMcm->hEnet              = (Enet_Handle)NULL_PTR;
            hMcm->hUdmaDrv           = pMcmInitCfg->hUdmaDrv;
            hMcm->periodicTaskPeriod = pMcmInitCfg->periodicTaskPeriod;
            hMcm->disablePeriodicFxn = pMcmInitCfg->disablePeriodicFxn;

            hMcm->print = pMcmInitCfg->print;
            if (hMcm->print == NULL_PTR)
            {
                hMcm->print = EnetAppUtils_print;
            }

            EnetMcm_initAttachTable(&hMcm->coreAttachTable);

            EnetMcm_initMbox(hMcm);

            hMcm->isInitDone = true;
        }
    }

    SemaphoreP_post(hMcm->hMutex);

    return status;
}

void  EnetMcm_getCmdIf(Enet_Type enetType,
                       EnetMcm_CmdIf *hMcmCmdIf)
{
    EnetMcm_Handle hMcm = &gMcmObj[0U];

    EnetAppUtils_assert(hMcmCmdIf != NULL_PTR);
    EnetAppUtils_assert(hMcm->hMutex != NULL_PTR);

    SemaphoreP_pend(hMcm->hMutex, SystemP_WAIT_FOREVER);
    EnetAppUtils_assert(hMcm->isInitDone == true);

    hMcmCmdIf->hMboxCmd      = hMcm->hMboxCmd;
    hMcmCmdIf->hMboxResponse = hMcm->hMboxResponse;

    SemaphoreP_post(hMcm->hMutex);

    EnetAppUtils_assert(hMcmCmdIf->hMboxCmd != NULL_PTR);
    EnetAppUtils_assert(hMcmCmdIf->hMboxResponse != NULL_PTR);
}

void EnetMcm_deInit(Enet_Type enetType)
{
    EnetMcm_Handle hMcm = &gMcmObj[0U];
    UBaseType_t numPendingmsgs;

    EnetAppUtils_assert(hMcm->hMutex != NULL_PTR);

    SemaphoreP_pend(hMcm->hMutex, SystemP_WAIT_FOREVER);

    if (hMcm->isInitDone == true)
    {
        BaseType_t qStatus;
        EnetMcm_mailboxObj msg;

        msg.cmd = MCM_SHUTDOWN;

        qStatus = xQueueSendToBack(hMcm->hMboxCmd, &msg, portMAX_DELAY);
        EnetAppUtils_assert(qStatus == pdPASS);
        qStatus = xQueueReceive(hMcm->hMboxResponse, &msg, portMAX_DELAY);
        EnetAppUtils_assert(qStatus == pdPASS);
        EnetAppUtils_assert(msg.cmd == MCM_RESPONSE_SHUTDOWN);

        TaskP_destruct(&hMcm->taskObj);

        numPendingmsgs = uxQueueMessagesWaiting(hMcm->hMboxCmd);
        while (numPendingmsgs != 0U)
        {
            qStatus = xQueueReceive(hMcm->hMboxCmd, &msg, portMAX_DELAY);
            EnetAppUtils_assert(qStatus == pdPASS);
            numPendingmsgs = uxQueueMessagesWaiting(hMcm->hMboxCmd);
        }

        vQueueDelete(hMcm->hMboxCmd);

        numPendingmsgs = uxQueueMessagesWaiting(hMcm->hMboxResponse);
        while (numPendingmsgs != 0U)
        {
            qStatus = xQueueReceive(hMcm->hMboxResponse, &msg, portMAX_DELAY);
            EnetAppUtils_assert(qStatus == pdPASS);
            numPendingmsgs = uxQueueMessagesWaiting(hMcm->hMboxResponse);
        }

        vQueueDelete(hMcm->hMboxResponse);

        EnetMcm_initAttachTable(&hMcm->coreAttachTable);
        hMcm->isInitDone = false;
    }

    SemaphoreP_post(hMcm->hMutex);
}

static void EnetMcm_timerCb(ClockP_Object *clkInst, void * arg)
{
    SemaphoreP_Object * timerSem = (SemaphoreP_Object *)arg;

    /* Tick! */
    SemaphoreP_post(timerSem);
}

static void EnetMcm_periodicTick(void * mcmHandle)
{
    SemaphoreP_Object * timerSem;
    EnetMcm_Handle hMcm       = (EnetMcm_Handle)mcmHandle;

    timerSem = &hMcm->timerSemObj;
    while (hMcm->timerTaskShutDownFlag != true)
    {
        SemaphoreP_pend(timerSem, SystemP_WAIT_FOREVER);
        /* Enet_periodicTick should be called from only task context */
        Enet_periodicTick(hMcm->hEnet);
    }
}

static void EnetMcm_createClock(EnetMcm_Handle hMcm)
{
    TaskP_Params tskParams;
    ClockP_Params clkParams;
    int32_t status;

    status = SemaphoreP_constructCounting(&hMcm->timerSemObj, 0, 128);
    EnetAppUtils_assert(status == SystemP_SUCCESS);

    ClockP_Params_init(&clkParams);
    clkParams.start = FALSE;
    clkParams.timeout   = ClockP_usecToTicks(hMcm->periodicTaskPeriod*1000U);
    clkParams.period    = ClockP_usecToTicks(hMcm->periodicTaskPeriod*1000U);
    clkParams.args      = &hMcm->timerSemObj;
    clkParams.callback  = &EnetMcm_timerCb;

    /* Creating timer and setting timer callback function*/
    status = ClockP_construct(&hMcm->timerObj ,
                              &clkParams);
    if (status == SystemP_SUCCESS)
    {
        /* Set timer expiry time in OS ticks */
        ClockP_setTimeout(&hMcm->timerObj, hMcm->periodicTaskPeriod);
        ClockP_start(&hMcm->timerObj);
        hMcm->timerTaskShutDownFlag = false;
    }
    else
    {
        hMcm->print("EnetMcm_createClock() failed to create clock\r\n");
    }

    /* Initialize the taskperiodicTick params. Set the task priority higher than the
     * default priority (1) */
    TaskP_Params_init(&tskParams);
    tskParams.priority       = ENETMCM_PERIODICTSK_PRIORITY;
    tskParams.stack          = &hMcm->mcmPrdTaskStack[0];
    tskParams.stackSize      = sizeof(hMcm->mcmPrdTaskStack);
    tskParams.args           = hMcm;
    tskParams.name           = "Enet_PeriodicTickTask";
    tskParams.taskMain       =  &EnetMcm_periodicTick;

    status = TaskP_construct(&hMcm->task_periodicTickObj, &tskParams);
    EnetAppUtils_assert(status == SystemP_SUCCESS);
}

static int32_t EnetMcm_open(EnetMcm_Handle hMcm)
{
    int32_t status = ENET_SOK;

    TaskP_Params tskParams;


    hMcm->hEnet = Enet_getHandle(hMcm->enetType, hMcm->instId);
    EnetAppUtils_assert(NULL != hMcm->hEnet);

    status = SemaphoreP_constructCounting(&hMcm->asyncIoctlSemObj, 0, 128);
    EnetAppUtils_assert(SystemP_SUCCESS == status);

    /* Initialize the taskperiodicTick params. Set the task priority higher than the
     * default priority (1) */
    TaskP_Params_init(&tskParams);
    tskParams.priority       = ENETMCM_ASYNCIOCTLTASK_PRIORITY;
    tskParams.args           = (void *)hMcm;
    tskParams.name           = "EnetMcm_AsyncIoctlTask";
    tskParams.taskMain       =  EnetMcm_asyncIoctlTask;
    tskParams.stack          =  &hMcm->asyncIoctlTaskStack[0];
    tskParams.stackSize      =  sizeof(hMcm->asyncIoctlTaskStack);

    hMcm->asyncIoctlTaskShutdown = false;

    status = TaskP_construct(&hMcm->asyncIoctlTaskObj, &tskParams);
    EnetAppUtils_assert(status == SystemP_SUCCESS);

#if (ENET_ENABLE_PER_ICSSG == 1)
    if (Enet_isIcssFamily(hMcm->enetType))
    {
        Enet_registerEventCb(hMcm->hEnet,
                             ENET_EVT_ASYNC_CMD_RESP,
                             0U,
                             EnetMcm_evtCb,
                             (void *) &hMcm->asyncIoctlSemObj);
    }
#endif

    if (hMcm->disablePeriodicFxn != true)
    {
        EnetMcm_createClock(hMcm);
    }

    return status;
}

static void EnetMcm_coreAttachHandler(EnetMcm_Handle hMcm,
                                      uint32_t coreId,
                                      EnetPer_AttachCoreOutArgs *pAttachCoreOutArgs)
{
    Enet_IoctlPrms prms;
    EnetMcm_CoreAttachTableEntry *entry;
    int32_t status;

    if (NULL_PTR != pAttachCoreOutArgs)
    {
        entry = EnetMcm_getCoreAttachEntry(&hMcm->coreAttachTable, coreId);
        if (entry == NULL_PTR)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &coreId, pAttachCoreOutArgs);
            ENET_IOCTL(hMcm->hEnet,
                        coreId,
                        ENET_PER_IOCTL_ATTACH_CORE,
                        &prms,
                        status);
            if (status != ENET_SOK)
            {
                hMcm->print("EnetMcm_open failed ENET_PER_IOCTL_ATTACH_CORE: %d\r\n", status);
                EnetAppUtils_assert(false);
            }

            EnetMcm_addCoreAttachEntry(&hMcm->coreAttachTable, coreId, pAttachCoreOutArgs);
            entry = EnetMcm_getCoreAttachEntry(&hMcm->coreAttachTable, coreId);
        }

        EnetAppUtils_assert((entry != NULL_PTR));
        if (entry != NULL_PTR)
        {
            *pAttachCoreOutArgs = entry->coreInfo;
        }
    }
    else
    {
        EnetAppUtils_assert(false);
    }
}

static void EnetMcm_coreDetachHandler(EnetMcm_Handle hMcm,
                                      uint32_t coreId,
                                      uint32_t coreKey)
{
    Enet_IoctlPrms prms;
    EnetMcm_CoreAttachTableEntry *entry;

    entry = EnetMcm_getCoreAttachEntry(&hMcm->coreAttachTable, coreId);

    if (entry != NULL_PTR)
    {
        int32_t status;

        ENET_IOCTL_SET_IN_ARGS(&prms, &coreKey);
        ENET_IOCTL(hMcm->hEnet,
                   coreId,
                   ENET_PER_IOCTL_DETACH_CORE,
                   &prms,
                   status);
        if (status != ENET_SOK)
        {
            hMcm->print("close() failed ENET_PER_IOCTL_DETACH_CORE: %d\r\n", status);
            EnetAppUtils_assert(false);
        }

        EnetMcm_delCoreAttachEntry(&hMcm->coreAttachTable, coreId, coreKey);
    }
}

static int32_t EnetMcm_ioctlHandler(EnetMcm_Handle hMcm,
                                    uint32_t cmd,
                                    Enet_IoctlPrms *prms)
{
    int32_t status;

    if (prms != NULL)
    {
        /* Cannot register IOCTL and invoke Ioctl as done for other Ioctl
         * cmds as the ioctl cmd is not known in MCM. So have to directly invoke the
         * ioctl. The IOCTL register must be done by application code that invokes the
         * mcm IOCTL command
         */
        extern int32_t Enet_ioctl(Enet_Handle enetHandle,
           uint32_t ioctlCoreId,
           uint32_t cmd,
           Enet_IoctlPrms *ioctlPrms);

        status = Enet_ioctl(hMcm->hEnet, hMcm->selfCoreId, cmd, prms);

        if (status == ENET_SINPROGRESS)
        {
            /* Wait for asyc ioctl to complete */
            hMcm->asyncIoctlDone = false;
            while (!hMcm->asyncIoctlDone)
            {
                Enet_poll(hMcm->hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);

                ClockP_usleep(1000U);
            }

            status = ENET_SOK;
        }
        else if (status != ENET_SOK)
        {
            hMcm->print("EnetMcm_asyncIoctlHandler failed: %d\n", status);
        }
    }
    else
    {
        status = ENET_EBADARGS;
        EnetAppUtils_assert(false);
    }

    return status;
}

static void EnetMcm_serverTask(void * McmHandle)
{
    volatile bool isShutdownMcm = false;
    EnetMcm_Handle hMcm         = (EnetMcm_Handle)McmHandle;

    while (!isShutdownMcm)
    {
        int32_t status = ENET_SOK;
        EnetMcm_mailboxObj msg;
        BaseType_t qStatus;

        qStatus = xQueueReceive(hMcm->hMboxCmd, &msg, portMAX_DELAY);
        EnetAppUtils_assert(qStatus == pdPASS);

        switch (msg.cmd)
        {
            case MCM_GET_HANDLE:
                if (hMcm->refCnt == 0)
                {
                    status = EnetMcm_open(hMcm);
                    EnetAppUtils_assert(ENET_SOK == status);
                }

                if (status == ENET_SOK)
                {
                    hMcm->refCnt++;
                }

                msg.cmd                         = MCM_RESPONSE_GET_HANDLE;
                msg.msgBody.handleInfo.hEnet    = hMcm->hEnet;
                msg.msgBody.handleInfo.hUdmaDrv = hMcm->hUdmaDrv;
                break;

            case MCM_RELEASE_HANDLE:
                EnetAppUtils_assert(hMcm->refCnt > 0);
                hMcm->refCnt--;

                if (hMcm->refCnt == 0)
                {
                    EnetMcm_close(hMcm);
                }

                msg.cmd = MCM_RESPONSE_RELEASE_HANDLE;
                break;

            case MCM_CORE_ATTACH:
                EnetAppUtils_assert(hMcm->hEnet != NULL_PTR);
                EnetMcm_coreAttachHandler(hMcm, msg.msgBody.coreId, &msg.msgBody.attachInfo);
                msg.cmd = MCM_RESPONSE_CORE_ATTACH;
                break;

            case MCM_CORE_DETACH:
                EnetAppUtils_assert(hMcm->hEnet != NULL_PTR);
                EnetMcm_coreDetachHandler(hMcm, msg.msgBody.coreId, msg.msgBody.coreKey);
                msg.cmd = MCM_RESPONSE_CORE_DETACH;
                break;

            case MCM_IOCTL:
                EnetAppUtils_assert(hMcm->hEnet != NULL);
                msg.msgBody.ioctlStatus = EnetMcm_ioctlHandler(hMcm, msg.msgBody.ioctlCmd, msg.msgBody.ioctlPrms);
                msg.cmd = MCM_RESPONSE_IOCTL;
                break;

            case MCM_SHUTDOWN:
                isShutdownMcm = true;
                msg.cmd       = MCM_RESPONSE_SHUTDOWN;
                break;

            default:
                /* Unahandle MCM command */
                EnetAppUtils_assert(false);
        }

        qStatus = xQueueSendToBack(hMcm->hMboxResponse, &msg, portMAX_DELAY);
        EnetAppUtils_assert(qStatus == pdPASS);
    }
}

static void EnetMcm_close(EnetMcm_Handle hMcm)
{
    Enet_IoctlPrms prms;
    int32_t status;
    uint32_t i;
    Enet_MacPort macPortList[ENET_MAC_PORT_NUM];
    uint8_t numMacPorts;

    EnetApp_getEnetInstMacInfo(hMcm->enetType, hMcm->instId, macPortList, &numMacPorts);

    if (Enet_isCpswFamily(hMcm->enetType))
    {
        /* Disable host port */
        ENET_IOCTL_SET_NO_ARGS(&prms);
        ENET_IOCTL(hMcm->hEnet,
                   hMcm->selfCoreId,
                   ENET_HOSTPORT_IOCTL_DISABLE,
                   &prms,
                   status);
        if (status != ENET_SOK)
        {
            hMcm->print("Failed to disable host port: %d\r\n", status);
        }
    }

    if (Enet_isIcssFamily(hMcm->enetType))
    {
        EnetAppUtils_print("Unregister async IOCTL callback\r\n");
        Enet_unregisterEventCb(hMcm->hEnet,
                               ENET_EVT_ASYNC_CMD_RESP,
                               0U);

        EnetAppUtils_print("Unregister TX timestamp callback\r\n");
        Enet_unregisterEventCb(hMcm->hEnet,
                               ENET_EVT_TIMESTAMP_TX,
                               0U);
    }

    for (i = 0U; i < numMacPorts; i++)
    {
        Enet_MacPort macPort = macPortList[i];

        ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
        ENET_IOCTL(hMcm->hEnet,
                   hMcm->selfCoreId,
                   ENET_PER_IOCTL_CLOSE_PORT_LINK,
                   &prms,
                   status);
        if (status != ENET_SOK)
        {
            hMcm->print("close() failed to close MAC port: %d\r\n", status);
        }
    }

    if (hMcm->disablePeriodicFxn != true)
    {
        hMcm->timerTaskShutDownFlag = true;

        ClockP_stop(&hMcm->timerObj);
        ClockP_destruct(&hMcm->timerObj);

        /* Post Timer Sem once to get the Periodic Tick task terminated */
        SemaphoreP_post(&hMcm->timerSemObj);

        TaskP_destruct(&hMcm->task_periodicTickObj);

        SemaphoreP_destruct(&hMcm->timerSemObj);
    }

    /* Post semaphore once to get the async IOCTL task terminated */
    hMcm->asyncIoctlTaskShutdown = true;
    SemaphoreP_post(&hMcm->asyncIoctlSemObj);

    TaskP_destruct(&hMcm->asyncIoctlTaskObj);

    SemaphoreP_destruct(&hMcm->asyncIoctlSemObj);

    Enet_close(hMcm->hEnet);

    EnetMem_deInit();

    Enet_deinit();

    hMcm->hEnet = (Enet_Handle) NULL_PTR;
    /* Clear disablePeriodicFxn flag on deInit */
    hMcm->disablePeriodicFxn = false;
}

void EnetMcm_acquireHandleInfo(const EnetMcm_CmdIf *hMcmCmdIf,
                               EnetMcm_HandleInfo *handleInfo)
{
    EnetMcm_mailboxObj msg;

    memset(&msg, 0, sizeof(msg));
    msg.cmd = MCM_GET_HANDLE;
    if ((hMcmCmdIf != NULL_PTR) && (handleInfo != NULL_PTR))
    {
        BaseType_t qStatus;

        qStatus = xQueueSendToBack(hMcmCmdIf->hMboxCmd, &msg, portMAX_DELAY);
        EnetAppUtils_assert(pdPASS == qStatus);
        qStatus = xQueueReceive(hMcmCmdIf->hMboxResponse, &msg, portMAX_DELAY);
        EnetAppUtils_assert(pdPASS == qStatus);
        EnetAppUtils_assert(msg.cmd == MCM_RESPONSE_GET_HANDLE);

        *handleInfo = msg.msgBody.handleInfo;
    }
    else
    {
        EnetAppUtils_assert(false);
    }
}

void EnetMcm_releaseHandleInfo(const EnetMcm_CmdIf *hMcmCmdIf)
{
    EnetMcm_mailboxObj msg;
    BaseType_t  qStatus;

    memset(&msg, 0, sizeof(msg));
    msg.cmd = MCM_RELEASE_HANDLE;
    EnetAppUtils_assert(hMcmCmdIf != NULL_PTR);

    qStatus = xQueueSendToBack(hMcmCmdIf->hMboxCmd, &msg, portMAX_DELAY);
    EnetAppUtils_assert(pdPASS == qStatus);

    qStatus = xQueueReceive(hMcmCmdIf->hMboxResponse, &msg, portMAX_DELAY);
    EnetAppUtils_assert(pdPASS == qStatus);
    EnetAppUtils_assert(msg.cmd == MCM_RESPONSE_RELEASE_HANDLE);
}

void EnetMcm_coreAttach(const EnetMcm_CmdIf *hMcmCmdIf,
                        uint32_t coreId,
                        EnetPer_AttachCoreOutArgs *attachInfo)
{
    EnetMcm_mailboxObj msg;
    BaseType_t  qStatus;

    memset(&msg, 0, sizeof(msg));
    if ((hMcmCmdIf != NULL_PTR) && (attachInfo != NULL_PTR))
    {
        msg.cmd            = MCM_CORE_ATTACH;
        msg.msgBody.coreId = coreId;

        qStatus = xQueueSendToBack(hMcmCmdIf->hMboxCmd, &msg, portMAX_DELAY);
        EnetAppUtils_assert(pdPASS == qStatus);

        qStatus = xQueueReceive(hMcmCmdIf->hMboxResponse, &msg, portMAX_DELAY);
        EnetAppUtils_assert(pdPASS == qStatus);

        EnetAppUtils_assert(msg.cmd == MCM_RESPONSE_CORE_ATTACH);
        *attachInfo = msg.msgBody.attachInfo;
    }
    else
    {
        EnetAppUtils_assert(false);
    }
}

void     EnetMcm_coreDetach(const EnetMcm_CmdIf *hMcmCmdIf,
                            uint32_t coreId,
                            uint32_t coreKey)
{
    EnetMcm_mailboxObj msg;
    BaseType_t  qStatus;

    memset(&msg, 0, sizeof(msg));
    EnetAppUtils_assert(hMcmCmdIf != NULL_PTR);
    msg.cmd             = MCM_CORE_DETACH;
    msg.msgBody.coreId  = coreId;
    msg.msgBody.coreKey = coreKey;

    qStatus = xQueueSendToBack(hMcmCmdIf->hMboxCmd, &msg, portMAX_DELAY);
    EnetAppUtils_assert(pdPASS == qStatus);

    qStatus = xQueueReceive(hMcmCmdIf->hMboxResponse, &msg, portMAX_DELAY);
    EnetAppUtils_assert(pdPASS == qStatus);

    EnetAppUtils_assert(msg.cmd == MCM_RESPONSE_CORE_DETACH);
}

int32_t EnetMcm_ioctl(const EnetMcm_CmdIf *hMcmCmdIf,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms)
{
    EnetMcm_mailboxObj msg;
    int32_t status;
    BaseType_t  qStatus;

    memset(&msg, 0, sizeof(msg));

    if ((hMcmCmdIf != NULL) && (prms != NULL))
    {
        msg.cmd = MCM_IOCTL;
        msg.msgBody.ioctlCmd  = cmd;
        msg.msgBody.ioctlPrms = prms;

        qStatus = xQueueSendToBack(hMcmCmdIf->hMboxCmd, &msg, portMAX_DELAY);
        EnetAppUtils_assert(pdPASS == qStatus);

        qStatus = xQueueReceive(hMcmCmdIf->hMboxResponse, &msg, portMAX_DELAY);
        EnetAppUtils_assert(pdPASS == qStatus);
        EnetAppUtils_assert(msg.cmd == MCM_RESPONSE_IOCTL);
        status = msg.msgBody.ioctlStatus;
    }
    else
    {
        status = ENET_EBADARGS;
        EnetAppUtils_assert(false);
    }

    return status;
}
