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

#include <drivers/ipc_notify/v0/soc/ipc_notify_soc.h>
#include <drivers/ipc_notify/v0/ipc_notify_v0.h>
#include <drivers/ipc_notify/v0/ipc_notify_v0_mailbox.h>

/* This shift value MUST be in sync with IPC_NOTIFY_CLIENT_ID_MAX
 * and IPC_NOTIFY_MSG_VALUE_MAX
 */
#define IPC_NOTIFY_CLIENT_ID_SHIFT      (28U)

/* This shift value MUST be in sync with IPC_NOTIFY_CLIENT_ID_MAX
 * and IPC_NOTIFY_CRC_MSG_VALUE_MAX
 */
#define IPC_NOTIFY_CRC_SHIFT      (20U)

/*
 * global internal module state
 */
typedef struct
{
    uint32_t                selfCoreId; /* core ID on which this module is running */
    IpcNotify_FxnCallback   callback[IPC_NOTIFY_CLIENT_ID_MAX]; /* user registered callback's */
    void*                   callbackArgs[IPC_NOTIFY_CLIENT_ID_MAX]; /* arguments to user registered callback's */
    uint8_t                 isCoreEnabled[CSL_CORE_ID_MAX]; /* flags to indicate if a core is enabled for IPC */
    IpcNotify_InterruptConfig *interruptConfig; /* interrupt config for this core,
                                                 * this is a array with one element per interrupt that is setup to receive messages
                                                 */
    uint32_t                interruptConfigNum; /* number of interrupts to setup, i.e number of element in interruptConfig array */
    uint16_t                syncMsgPend[CSL_CORE_ID_MAX]; /* Number of sync messages pending */
    uint32_t                linuxCoreId; /* core ID of core running linux */
    uint8_t                 isCrcEnabled; /* CRC Enable/Disable flag */
    IpcNotify_CrcHookFxn    crcHookFxn; /* Hook Function to be provided by application for CRC calculation. */
} IpcNotify_Ctrl;

IpcNotify_Ctrl gIpcNotifyCtrl;

/**
 * \brief Callback to call when interrupt is received
 */
void IpcNotify_isr(void *args);

/**
 * \brief Callback that is invoked during initialization for a given client ID
 *
 * \param remoteCoreId  [in] Remote core that has sent the message
 * \param localClientId [in] Local client ID to which the message is sent
 * \param msgValue      [in] Message value that is sent
 * \param crcStatus     [in] CRC Check status. SystemP_SUCCESS on success, else SystemP_FAILURE.
 * \param args          [in] Argument pointer passed by user when \ref IpcNotify_registerClient is called
 */
void IpcNotify_syncCallback(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args);

static inline void IpcNotify_getWriteMailbox(uint32_t remoteCoreId, uint32_t *mailboxBaseAddr, uint32_t *hwFifoId)
{
    IpcNotify_MailboxConfig *pMailboxConfig;

    pMailboxConfig = &gIpcNotifyMailboxConfig[gIpcNotifyCtrl.selfCoreId][remoteCoreId];

    *mailboxBaseAddr = gIpcNotifyMailboxBaseAddr[ pMailboxConfig->mailboxId ];
    *hwFifoId = pMailboxConfig->hwFifoId;

}

static inline void IpcNotify_getReadMailbox(uint32_t remoteCoreId, uint32_t *mailboxBaseAddr, uint32_t *hwFifoId, uint32_t *userId)
{
    IpcNotify_MailboxConfig *pMailboxConfig;

    pMailboxConfig = &gIpcNotifyMailboxConfig[remoteCoreId][gIpcNotifyCtrl.selfCoreId];

    *mailboxBaseAddr = gIpcNotifyMailboxBaseAddr[ pMailboxConfig->mailboxId ];
    *hwFifoId = pMailboxConfig->hwFifoId;
    *userId = pMailboxConfig->userId;
}

static uint32_t IpcNotify_makeMsg(uint16_t clientId, uint32_t msgValue)
{
    uint32_t msg;
    uint8_t crc = 0;
    uint32_t crcData;

    if(gIpcNotifyCtrl.isCrcEnabled)
    {
        crcData = msgValue & (IPC_NOTIFY_CRC_MSG_VALUE_MAX-1U);

        gIpcNotifyCtrl.crcHookFxn(((uint8_t *)(&crcData)), IPC_NOTIFY_CRC_DATASIZE, IPC_NOTIFY_CRC_SIZE, &crc);

        msg = ((clientId & (IPC_NOTIFY_CLIENT_ID_MAX-1U)) << IPC_NOTIFY_CLIENT_ID_SHIFT) |
            (crc << IPC_NOTIFY_CRC_SHIFT) |
            (msgValue & (IPC_NOTIFY_CRC_MSG_VALUE_MAX-1U));
    }
    else
    {
        msg = ((clientId & (IPC_NOTIFY_CLIENT_ID_MAX-1U)) << IPC_NOTIFY_CLIENT_ID_SHIFT) |
            (msgValue & (IPC_NOTIFY_MSG_VALUE_MAX-1U));
    }

    return msg;
}

void IpcNotify_isr(void *args)
{
    IpcNotify_InterruptConfig *pInterruptConfig = (IpcNotify_InterruptConfig *) args;
    uint32_t mailboxBaseAddr, hwFifoId, userId;
    uint32_t core, msg, value, numMsgs;
    uint16_t clientId;
    uint8_t inputCrc, calcCrc;
    uint32_t crcData;
    int32_t crcStatus = SystemP_SUCCESS;

    for(core=0; core<pInterruptConfig->numCores; core++)
    {
        IpcNotify_getReadMailbox(pInterruptConfig->coreIdList[core], &mailboxBaseAddr, &hwFifoId, &userId);
        DebugP_assertNoLog(mailboxBaseAddr!=0U);

        IpcNotify_mailboxClearInt(mailboxBaseAddr, hwFifoId, userId);
        numMsgs = IpcNotify_mailboxGetNumMsg(mailboxBaseAddr, hwFifoId);

        for(msg=0; msg<numMsgs; msg++)
        {
            value = IpcNotify_mailboxRead(mailboxBaseAddr, hwFifoId);
            clientId = (value >> IPC_NOTIFY_CLIENT_ID_SHIFT) & (IPC_NOTIFY_CLIENT_ID_MAX-1U);

            if(gIpcNotifyCtrl.isCrcEnabled)
            {
                crcData = (value & (IPC_NOTIFY_CRC_MSG_VALUE_MAX-1U));
                crcStatus = SystemP_FAILURE;

                inputCrc = (uint8_t)((value >> IPC_NOTIFY_CRC_SHIFT) & (IPC_NOTIFY_CRC_MAX - 1U));
                crcStatus = gIpcNotifyCtrl.crcHookFxn((uint8_t *)(&crcData), IPC_NOTIFY_CRC_DATASIZE, IPC_NOTIFY_CRC_SIZE, &calcCrc);

                if((crcStatus == SystemP_SUCCESS) && (inputCrc == calcCrc))
                {
                    crcStatus = SystemP_SUCCESS;
                }
            }

            if(gIpcNotifyCtrl.callback[clientId]!=NULL)
            {
                if(gIpcNotifyCtrl.isCrcEnabled)
                {
                    gIpcNotifyCtrl.callback[clientId](
                            pInterruptConfig->coreIdList[core],
                            clientId,
                            (value & (IPC_NOTIFY_CRC_MSG_VALUE_MAX-1U)),
                            crcStatus,
                            gIpcNotifyCtrl.callbackArgs[clientId]
                            );
                }
                else
                {
                    gIpcNotifyCtrl.callback[clientId](
                            pInterruptConfig->coreIdList[core],
                            clientId,
                            (value & (IPC_NOTIFY_MSG_VALUE_MAX-1U)),
                            crcStatus,
                            gIpcNotifyCtrl.callbackArgs[clientId]
                            );
                }
            }
        }
    }
}

int32_t IpcNotify_sendMsg(uint32_t remoteCoreId, uint16_t remoteClientId, uint32_t msgValue, uint32_t waitForFifoNotFull)
{
    uint32_t oldIntState, isFull;
    uint32_t mailboxBaseAddr, hwFifoId;
    int32_t status = SystemP_FAILURE;

    if((remoteCoreId < CSL_CORE_ID_MAX) && (gIpcNotifyCtrl.isCoreEnabled[remoteCoreId] != 0U))
    {
        IpcNotify_getWriteMailbox(remoteCoreId, &mailboxBaseAddr, &hwFifoId);
        DebugP_assert(mailboxBaseAddr!= 0U);

        oldIntState = HwiP_disable();
        do
        {
            isFull = IpcNotify_mailboxIsFull(mailboxBaseAddr, hwFifoId);
            if((isFull != (uint32_t)SystemP_SUCCESS) && (waitForFifoNotFull != 0U))
            {
                /* allow interrupt enable and check again */
                HwiP_restore(oldIntState);
                oldIntState = HwiP_disable();
            }
        } while((isFull != (uint32_t)SystemP_SUCCESS) && (waitForFifoNotFull != 0U));

        if(isFull == 0U)
        {
            uint32_t value = IpcNotify_makeMsg(remoteClientId, msgValue);
            IpcNotify_mailboxWrite(mailboxBaseAddr, hwFifoId, value);
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_TIMEOUT;
        }
        HwiP_restore(oldIntState);
    }
    return status;
}

int32_t IpcNotify_registerClient(uint16_t localClientId, IpcNotify_FxnCallback msgCallback, void *args)
{
    int32_t status = SystemP_FAILURE;
    uint32_t oldIntState;

    DebugP_assert(msgCallback != NULL);

    if(localClientId < IPC_NOTIFY_CLIENT_ID_MAX)
    {
        oldIntState = HwiP_disable();
        if(gIpcNotifyCtrl.callback[localClientId] == NULL )
        {
            gIpcNotifyCtrl.callback[localClientId] = msgCallback;
            gIpcNotifyCtrl.callbackArgs[localClientId] = args;
            status = SystemP_SUCCESS;
        }
        HwiP_restore(oldIntState);
    }
    return status;
}

int32_t IpcNotify_unregisterClient(uint16_t localClientId)
{
    uint32_t oldIntState;

    oldIntState = HwiP_disable();
    if(localClientId < IPC_NOTIFY_CLIENT_ID_MAX)
    {
        gIpcNotifyCtrl.callback[localClientId] = NULL;
        gIpcNotifyCtrl.callbackArgs[localClientId] = NULL;
    }
    HwiP_restore(oldIntState);

    return SystemP_SUCCESS;
}

void IpcNotify_syncCallback(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    if(remoteCoreId < CSL_CORE_ID_MAX)
    {
        gIpcNotifyCtrl.syncMsgPend[remoteCoreId]++;
    }
}

void IpcNotify_Params_init(IpcNotify_Params *params)
{
    uint32_t i;

    params->intrPriority = IPC_NOTIFY_DEFAULT_INTR_PRIORITY;
    params->numCores = 0;
    for(i=0; i<CSL_CORE_ID_MAX; i++)
    {
        params->coreIdList[i] = CSL_CORE_ID_MAX;
    }
    params->selfCoreId = CSL_CORE_ID_MAX;
    params->linuxCoreId = CSL_CORE_ID_MAX;
    params->isCrcEnabled = 0;
    params->crcHookFxn = NULL;
    params->isIPCIntrRouter = 0;
}

int32_t IpcNotify_init(const IpcNotify_Params *params)
{
    uint32_t i, core, oldIntState;
    int32_t status = SystemP_SUCCESS;
    uint32_t mailboxBaseAddr, hwFifoId, userId;

    gIpcNotifyCtrl.linuxCoreId = params->linuxCoreId;

    IpcNotify_getConfig(&gIpcNotifyCtrl.interruptConfig, &gIpcNotifyCtrl.interruptConfigNum);

    /* translate mailbox address to local CPU addresses */
    for(i=0; gIpcNotifyMailboxBaseAddr[i]!=0U; i++)
    {
        gIpcNotifyMailboxBaseAddr[i] = (uint32_t) AddrTranslateP_getLocalAddr(gIpcNotifyMailboxBaseAddr[i]);
    }
    uint32_t selfCoreId_Check = (params->selfCoreId < CSL_CORE_ID_MAX)?1U:0U;
    DebugP_assert(selfCoreId_Check != 0U);
    gIpcNotifyCtrl.selfCoreId = params->selfCoreId;
    gIpcNotifyCtrl.isCrcEnabled = params->isCrcEnabled;
    gIpcNotifyCtrl.crcHookFxn = params->crcHookFxn;
    for(i=0; i<IPC_NOTIFY_CLIENT_ID_MAX; i++)
    {
        IpcNotify_unregisterClient(i);
    }
    for(core=0; core<CSL_CORE_ID_MAX; core++)
    {
        gIpcNotifyCtrl.isCoreEnabled[core] = 0;
        gIpcNotifyCtrl.syncMsgPend[core] = 0;
    }

    /* check parameters and config and assert if invalid */
    DebugP_assert(params->numCores > 0U);
    for(core=0; core<params->numCores; core++)
    {
        uint32_t coreIDlist_Check = (params->coreIdList[core] < CSL_CORE_ID_MAX)?1U:0U;
        DebugP_assert(coreIDlist_Check!=0U);
        DebugP_assert(params->coreIdList[core] != params->selfCoreId);
        /* mark core as enabled for IPC */
        gIpcNotifyCtrl.isCoreEnabled[params->coreIdList[core]] = 1;
    }
    for(i=0; i<gIpcNotifyCtrl.interruptConfigNum; i++)
    {
        IpcNotify_InterruptConfig *pInterruptConfig;

        pInterruptConfig = &gIpcNotifyCtrl.interruptConfig[i];

        DebugP_assert(pInterruptConfig->numCores > 0U);
        for(core=0; core<pInterruptConfig->numCores; core++)
        {
            uint32_t coreIDlist_InterruptCheck = (pInterruptConfig->coreIdList[core] < CSL_CORE_ID_MAX)?1U:0U;
            DebugP_assert(coreIDlist_InterruptCheck!=0U);
            DebugP_assert(pInterruptConfig->coreIdList[core] != gIpcNotifyCtrl.selfCoreId);
            /* check if mailbox info is valid for this core */
            IpcNotify_getReadMailbox(pInterruptConfig->coreIdList[core], &mailboxBaseAddr, &hwFifoId, &userId);
            DebugP_assert(mailboxBaseAddr!=0U);
        }
    }

    IpcNotify_registerClient(IPC_NOTIFY_CLIENT_ID_SYNC, IpcNotify_syncCallback, NULL);

    oldIntState = HwiP_disable();

    for(i=0; i<gIpcNotifyCtrl.interruptConfigNum; i++)
    {
        HwiP_Params hwiParams;
        IpcNotify_InterruptConfig *pInterruptConfig;

        pInterruptConfig = &gIpcNotifyCtrl.interruptConfig[i];

        for(core=0; core<pInterruptConfig->numCores; core++)
        {
            if(gIpcNotifyCtrl.isCoreEnabled[pInterruptConfig->coreIdList[core]] != 0U)
            {
                IpcNotify_getReadMailbox(pInterruptConfig->coreIdList[core], &mailboxBaseAddr, &hwFifoId, &userId);
		if( params->isIPCIntrRouter == IPC_INTR_ROUTER_ENABLE){
                    IpcNotify_setIntrRtr( gIpcNotifyCtrl.selfCoreId, pInterruptConfig->coreIdList[core], mailboxBaseAddr, hwFifoId, userId);
		}else{
                    IpcNotify_mailboxClearInt(mailboxBaseAddr, hwFifoId, userId);
                    IpcNotify_mailboxEnableInt(mailboxBaseAddr, hwFifoId, userId);
		}
            }
        }
        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = pInterruptConfig->intNum;
        hwiParams.priority = params->intrPriority;
        hwiParams.callback = IpcNotify_isr;
        hwiParams.args = (void*)pInterruptConfig;
        hwiParams.eventId = pInterruptConfig->eventId;
        hwiParams.isPulse = 0; /* mailbox is level interrupt */

        status
        += HwiP_construct(
            &pInterruptConfig->hwiObj,
            &hwiParams);
    }

    HwiP_restore(oldIntState);

    return status;
}

void IpcNotify_deInit(void)
{
    uint32_t i, core;
    uint32_t oldIntState;
    uint32_t mailboxBaseAddr, hwFifoId, userId;

    for(i=0; i<IPC_NOTIFY_CLIENT_ID_MAX; i++)
    {
        IpcNotify_unregisterClient(i);
    }

    oldIntState = HwiP_disable();

    for(i=0; i<gIpcNotifyCtrl.interruptConfigNum; i++)
    {
        IpcNotify_InterruptConfig *pInterruptConfig;

        pInterruptConfig = &gIpcNotifyCtrl.interruptConfig[i];

        for(core=0; core<pInterruptConfig->numCores; core++)
        {
            if(gIpcNotifyCtrl.isCoreEnabled[pInterruptConfig->coreIdList[core]] != 0U)
            {
                IpcNotify_getReadMailbox(pInterruptConfig->coreIdList[core],
                                            &mailboxBaseAddr, &hwFifoId, &userId);
                IpcNotify_mailboxClearInt(mailboxBaseAddr, hwFifoId, userId);
                IpcNotify_mailboxDisableInt(mailboxBaseAddr, hwFifoId, userId);
            }
        }
        HwiP_destruct(&pInterruptConfig->hwiObj);
    }

    HwiP_restore(oldIntState);
}

uint32_t IpcNotify_getSelfCoreId(void)
{
    return gIpcNotifyCtrl.selfCoreId;
}

uint32_t IpcNotify_isCoreEnabled(uint32_t coreId)
{
    uint32_t isEnabled = 0;

    if(coreId < CSL_CORE_ID_MAX)
    {
        isEnabled = gIpcNotifyCtrl.isCoreEnabled[coreId];
    }
    return isEnabled;
}

int32_t IpcNotify_sendSync(uint32_t remoteCoreId)
{
    return IpcNotify_sendMsg(remoteCoreId,
                IPC_NOTIFY_CLIENT_ID_SYNC,
                0xFF, /* message value is dont care */
                1 /* wait for messahe to be put in the HwFifo */
                );
}

int32_t IpcNotify_waitSync(uint32_t remoteCoreId, uint32_t timeout)
{
    int32_t status = SystemP_FAILURE;
    uint32_t startTicks, eslapedTicks, isDone;

    if((remoteCoreId < CSL_CORE_ID_MAX) && (gIpcNotifyCtrl.isCoreEnabled[remoteCoreId] != 0U))
    {
        startTicks = ClockP_getTicks();
        isDone = 0;
        while(isDone == 0U)
        {
            if(gIpcNotifyCtrl.syncMsgPend[remoteCoreId] ==  0U)
            {
                eslapedTicks = ClockP_getTicks() - startTicks;
                if(eslapedTicks>=timeout)
                {
                    status = SystemP_TIMEOUT;
                    isDone = 1;
                }
                else
                {
                    /* check again after 1 tick */
                    ClockP_usleep(ClockP_ticksToUsec(1));
                }
            }
            else
            {
                uint32_t oldIntState;

                oldIntState = HwiP_disable();
                gIpcNotifyCtrl.syncMsgPend[remoteCoreId]--;
                HwiP_restore(oldIntState);

                status = SystemP_SUCCESS;
                isDone = 1;
            }
        }
    }
    return status;
}

int32_t IpcNotify_syncAll(uint32_t timeout)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t remoteCoreId;

    for(remoteCoreId=0; remoteCoreId<CSL_CORE_ID_MAX; remoteCoreId++)
    {
        if((gIpcNotifyCtrl.isCoreEnabled[remoteCoreId] != 0U)
            &&
            (remoteCoreId != gIpcNotifyCtrl.linuxCoreId) /* sync not supported with Linux */
            )
        {
            /* no need to check return status, this will always pass */
            IpcNotify_sendSync(remoteCoreId);
        }
    }
    for(remoteCoreId=0; remoteCoreId<CSL_CORE_ID_MAX; remoteCoreId++)
    {
        if((gIpcNotifyCtrl.isCoreEnabled[remoteCoreId] != 0U)
            &&
            (remoteCoreId != gIpcNotifyCtrl.linuxCoreId) /* sync not supported with Linux */
            )
        {
            status = IpcNotify_waitSync(remoteCoreId, timeout);
            if(status != SystemP_SUCCESS)
            {
                break;
            }
        }
    }
    return status;
}
