/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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

#include <drivers/ipc_notify/v1/ipc_notify_v1.h>


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

    uint32_t                nonNotifyNumCores; /* Number of core's not participating in IPC Notify */
    uint32_t                nonNotifyCoreList[CSL_CORE_ID_MAX]; /* Core ID of cores not participating in IPC Notify */
    IpcNotify_NonNotifyCallback  nonNotifyCallback; /* Function to call when interrupt is received from a non-notify core */
    uint8_t                 isCrcEnabled; /* CRC Enable/Disable flag */
    IpcNotify_CrcHookFxn    crcHookFxn; /* Hook Function to be provided by application for CRC calculation. */
} IpcNotify_Ctrl;

IpcNotify_Ctrl gIpcNotifyCtrl;

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

/**
 * \brief Callback to call when interrupt is received
 */
void IpcNotify_isr(void *args);

static inline void IpcNotify_getWriteMailbox(uint32_t remoteCoreId, uint32_t *mailboxBaseAddr, uint32_t *intrBitPos, IpcNotify_SwQueue **swQ)
{
    IpcNotify_MailboxConfig *pMailboxConfig;

    pMailboxConfig = &gIpcNotifyMailboxConfig[gIpcNotifyCtrl.selfCoreId][remoteCoreId];

    *mailboxBaseAddr = pMailboxConfig->writeDoneMailboxBaseAddr;
    *intrBitPos = pMailboxConfig->intrBitPos;
    *swQ = pMailboxConfig->swQ;

}

static inline void IpcNotify_getReadMailbox(uint32_t *mailboxBaseAddr)
{
    IpcNotify_MailboxConfig *pMailboxConfig;

    pMailboxConfig = &gIpcNotifyMailboxConfig[gIpcNotifyCtrl.selfCoreId][gIpcNotifyCtrl.selfCoreId];

    *mailboxBaseAddr = pMailboxConfig->readReqMailboxBaseAddr;

}

static inline void IpcNotify_getReadSwQ(uint32_t remoteCoreId, IpcNotify_SwQueue **swQ)
{
    IpcNotify_MailboxConfig *pMailboxConfig;

    pMailboxConfig = &gIpcNotifyMailboxConfig[remoteCoreId][gIpcNotifyCtrl.selfCoreId];

    *swQ = pMailboxConfig->swQ;
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
    uint32_t mailboxBaseAddr;
    IpcNotify_SwQueue *swQ;
    uint32_t core, value;
    uint16_t clientId;
    uint32_t remoteCoreId;
    int32_t status;
    uint32_t pendingIntr;
    uint8_t inputCrc, calcCrc;
    uint32_t crcData;
    int32_t crcStatus = SystemP_SUCCESS;

    IpcNotify_getReadMailbox(&mailboxBaseAddr);
    DebugP_assertNoLog(mailboxBaseAddr!=0U);

    pendingIntr = IpcNotify_mailboxGetPendingIntr(mailboxBaseAddr);
    do
    {
        /*
         * Processor sending will trigger read request multiple times and ensure
         * that read request is reached to receiving processor. The delay implemented
         * here is not to clear the interrupt while sending processor is reading back
         * and verifying the interrupt is triggered at receving Processor.
         *
         * NOTE: This workaround is currently implemented only for AWR294x SOC.
         */
        IpcNotify_wait();

        /* We clear pending interrupt unconditional here, and read all the SW queues later */
        IpcNotify_mailboxClearPendingIntr(mailboxBaseAddr, pendingIntr);

        if(gIpcNotifyCtrl.nonNotifyCallback!=NULL)
        {
            /* handle non notify interrupts, if any */
            for(core=0; core<gIpcNotifyCtrl.nonNotifyNumCores; core++)
            {
                if( (IpcNotify_mailboxIsPendingIntr(pendingIntr, gIpcNotifyCtrl.nonNotifyCoreList[core])) != 0U)
                {
                    gIpcNotifyCtrl.nonNotifyCallback(gIpcNotifyCtrl.nonNotifyCoreList[core]);
                }
            }
        }

        for(core=0; core<pInterruptConfig->numCores; core++)
        {
            remoteCoreId = pInterruptConfig->coreIdList[core];

            if(gIpcNotifyCtrl.isCoreEnabled[remoteCoreId] != 0U)
            {
                IpcNotify_getReadSwQ(remoteCoreId, &swQ);
                DebugP_assertNoLog(swQ!=NULL);
                do
                {
                    status = IpcNotify_mailboxReadSwQ(swQ, &value);
                    if(status == SystemP_SUCCESS)
                    {
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
                } while(status == SystemP_SUCCESS);
            }
        }

        /* we need to keeping doing this until all status bits are 0, else we dont get new interrupt at R5F */
        pendingIntr = IpcNotify_mailboxGetPendingIntr(mailboxBaseAddr);
    } while ( pendingIntr != 0U );

}

int32_t IpcNotify_sendMsg(uint32_t remoteCoreId, uint16_t remoteClientId, uint32_t msgValue, uint32_t waitForFifoNotFull)
{
    uint32_t oldIntState;
    uint32_t mailboxBaseAddr, intrBitPos;
    IpcNotify_SwQueue *swQ;
    int32_t status = SystemP_FAILURE;


    if((remoteCoreId < CSL_CORE_ID_MAX) && (gIpcNotifyCtrl.isCoreEnabled[remoteCoreId] != 0U))
    {
        uint32_t value = IpcNotify_makeMsg(remoteClientId, msgValue);

        IpcNotify_getWriteMailbox(remoteCoreId, &mailboxBaseAddr, &intrBitPos, &swQ);
        DebugP_assert(mailboxBaseAddr!=0U);
        DebugP_assert(swQ!=NULL);

        oldIntState = HwiP_disable();
        do
        {
#if !(defined(SOC_AM273X))
            status = IpcNotify_mailboxWrite(mailboxBaseAddr, intrBitPos, swQ, value);
#else
            status = IpcNotify_mailboxWrite(gIpcNotifyCtrl.selfCoreId, remoteCoreId, mailboxBaseAddr, intrBitPos, swQ, value);
#endif
            if((status != SystemP_SUCCESS) && (waitForFifoNotFull != 0U))
            {
                /* allow interrupt enable and check again */
                HwiP_restore(oldIntState);
                oldIntState = HwiP_disable();
            }
        } while((status != SystemP_SUCCESS)  && (waitForFifoNotFull != 0U));

        HwiP_restore(oldIntState);

        if(status != SystemP_SUCCESS)
        {
            status = SystemP_TIMEOUT;
        }
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
    int32_t status = SystemP_FAILURE;

    oldIntState = HwiP_disable();
    if(localClientId < IPC_NOTIFY_CLIENT_ID_MAX)
    {
        gIpcNotifyCtrl.callback[localClientId] = NULL;
        gIpcNotifyCtrl.callbackArgs[localClientId] = NULL;
        status = SystemP_SUCCESS;
    }
    HwiP_restore(oldIntState);

    return status;
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
}

int32_t IpcNotify_init(const IpcNotify_Params *params)
{
    uint32_t i, core, oldIntState;
    int32_t status = SystemP_SUCCESS;
    uint32_t mailboxBaseAddr;
    uint32_t coreIDlist_InterruptCheck;
    uint32_t coreID_Check = (params->selfCoreId < CSL_CORE_ID_MAX)?1U:0U;

    IpcNotify_allocSwQueue(&gIpcNotifyMailboxConfig[0][0]);
    IpcNotify_getConfig(&gIpcNotifyCtrl.interruptConfig, &gIpcNotifyCtrl.interruptConfigNum);

    DebugP_assert(coreID_Check!=0U);

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
        gIpcNotifyCtrl.nonNotifyCoreList[core] = CSL_CORE_ID_MAX;
    }
    gIpcNotifyCtrl.nonNotifyCallback = NULL;
    gIpcNotifyCtrl.nonNotifyNumCores = 0;

    /* check parameters and config and assert if invalid */
    for(core=0; core<params->numCores; core++)
    {
        uint32_t coreIDlist_Check = (params->coreIdList[core] < CSL_CORE_ID_MAX)?1U:0U;
        DebugP_assert(coreIDlist_Check!=0U);
        DebugP_assert(params->coreIdList[core] != params->selfCoreId);
        /* mark core as enabled for IPC */
        gIpcNotifyCtrl.isCoreEnabled[params->coreIdList[core]] = 1;
    }
    /* fill list of non notify cores */
    for(core=0; core<CSL_CORE_ID_MAX; core++)
    {
        if((gIpcNotifyCtrl.isCoreEnabled[core]==0U) && (core != gIpcNotifyCtrl.selfCoreId))
        {
            gIpcNotifyCtrl.nonNotifyCoreList[gIpcNotifyCtrl.nonNotifyNumCores] = core;
            gIpcNotifyCtrl.nonNotifyNumCores++;
        }
    }
    for(i=0; i<gIpcNotifyCtrl.interruptConfigNum; i++)
    {
        IpcNotify_InterruptConfig *pInterruptConfig;

        pInterruptConfig = &gIpcNotifyCtrl.interruptConfig[i];

        DebugP_assert(pInterruptConfig->numCores > 0U );
        for(core=0; core<pInterruptConfig->numCores; core++)
        {
            coreIDlist_InterruptCheck = (pInterruptConfig->coreIdList[core] < CSL_CORE_ID_MAX)?1U:0U;
            DebugP_assert(coreIDlist_InterruptCheck!=0U);
            DebugP_assert(pInterruptConfig->coreIdList[core] != gIpcNotifyCtrl.selfCoreId);
        }
        /* check if mailbox info is valid for this core */
        IpcNotify_getReadMailbox(&mailboxBaseAddr);
        DebugP_assert(mailboxBaseAddr!=0U);
    }

    IpcNotify_registerClient(IPC_NOTIFY_CLIENT_ID_SYNC, IpcNotify_syncCallback, NULL);

    oldIntState = HwiP_disable();

    for(i=0; i<gIpcNotifyCtrl.interruptConfigNum; i++)
    {
        HwiP_Params hwiParams;
        IpcNotify_InterruptConfig *pInterruptConfig;

        pInterruptConfig = &gIpcNotifyCtrl.interruptConfig[i];

        IpcNotify_getReadMailbox(&mailboxBaseAddr);

        if((pInterruptConfig->clearIntOnInit) != 0U)
        {
            IpcNotify_mailboxClearAllInt(mailboxBaseAddr);
        }

        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = pInterruptConfig->intNum;
        hwiParams.priority = params->intrPriority;
        hwiParams.callback = IpcNotify_isr;
        hwiParams.args = (void*)pInterruptConfig;
        hwiParams.eventId = pInterruptConfig->eventId;
        hwiParams.isPulse = 0;

        status += HwiP_construct(
            &pInterruptConfig->hwiObj,
            &hwiParams);
    }

    HwiP_restore(oldIntState);

    return status;
}

void IpcNotify_deInit(void)
{
    uint32_t i;
    uint32_t oldIntState;
    uint32_t mailboxBaseAddr;

    for(i=0; i<IPC_NOTIFY_CLIENT_ID_MAX; i++)
    {
        IpcNotify_unregisterClient(i);
    }

    oldIntState = HwiP_disable();

    for(i=0; i<gIpcNotifyCtrl.interruptConfigNum; i++)
    {
        IpcNotify_InterruptConfig *pInterruptConfig;

        pInterruptConfig = &gIpcNotifyCtrl.interruptConfig[i];

        IpcNotify_getReadMailbox(&mailboxBaseAddr);
        IpcNotify_mailboxClearAllInt(mailboxBaseAddr);

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
        if((gIpcNotifyCtrl.isCoreEnabled[remoteCoreId]) != 0U)
        {
            /* no need to check return status, this will always pass */
            IpcNotify_sendSync(remoteCoreId);
        }
    }
    for(remoteCoreId=0; remoteCoreId<CSL_CORE_ID_MAX; remoteCoreId++)
    {
        if((gIpcNotifyCtrl.isCoreEnabled[remoteCoreId]) != 0U)
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

void IpcNotify_registerNonNotifyCallback(IpcNotify_NonNotifyCallback callback)
{
    gIpcNotifyCtrl.nonNotifyCallback = callback;
}
