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

#include <drivers/ipc_rpmsg/ipc_rpmsg_priv.h>

IpcRpmsg_Ctrl gIpcRpmsgCtrl;

RPMessage_LocalMsg *RPMessage_allocEndPtMsg(uint32_t remoteCoreId)
{
    RPMessage_Core *coreObj = &gIpcRpmsgCtrl.coreObj[remoteCoreId];
    RPMessage_LocalMsg *pMsg;
    uint32_t oldIntState;

    oldIntState = HwiP_disable();
    pMsg = (RPMessage_LocalMsg*)RPMessage_queueGet(&coreObj->freeQ);
    if(pMsg == NULL)
    {
        coreObj->freeQAllocPending = 1;
    }
    else
    {
        coreObj->freeQAllocPending = 0;
    }
    HwiP_restore(oldIntState);

    return pMsg;
}

uint32_t RPMessage_freeEndPtMsg(uint16_t remoteCoreId, RPMessage_LocalMsg *pMsg)
{
    RPMessage_Core *coreObj = &gIpcRpmsgCtrl.coreObj[remoteCoreId];
    uint32_t oldIntState, isAllocPending;

    oldIntState = HwiP_disable();
    isAllocPending = coreObj->freeQAllocPending;
    RPMessage_queuePut(&coreObj->freeQ, &pMsg->elem);
    HwiP_restore(oldIntState);

    return isAllocPending;
}

void RPMessage_putEndPtMsg(RPMessage_Struct *obj, RPMessage_LocalMsg *pMsg)
{
    uint32_t oldIntState;

    oldIntState = HwiP_disable();
    RPMessage_queuePut(&obj->endPtQ, &pMsg->elem);
    HwiP_restore(oldIntState);

    SemaphoreP_post(&obj->newEndPtMsgSem);
}

int32_t RPMessage_getEndPtMsg(RPMessage_Struct *obj, RPMessage_LocalMsg **pMsg, uint32_t timeout)
{
    uint32_t oldIntState, done;
    int32_t status = SystemP_TIMEOUT;

    done = 0;
    do {
        oldIntState = HwiP_disable();
        *pMsg = (RPMessage_LocalMsg*)RPMessage_queueGet(&obj->endPtQ);
        HwiP_restore(oldIntState);

        if(*pMsg==NULL)
        {
            status = SemaphoreP_pend(&obj->newEndPtMsgSem, timeout);
            if(status == SystemP_TIMEOUT)
            {
                done = 1;
            }
            if((status == SystemP_SUCCESS) && ((obj->doRecvUnblock)!=0U))
            {
                status = SystemP_TIMEOUT;
                done = 1;
            }
        }
        else
        {
            status = SystemP_SUCCESS;
            done = 1;
        }
    } while(done!=1U);

    return status;
}

/* handle one new received message from vring */
void RPMessage_recvHandler(uint32_t remoteCoreId)
{
    uint16_t vringBufId;
    int32_t status;
    RPMessage_LocalMsg *pMsg;

    /* get a free message pointer to hold vring buf info
     * if no free message pointer then dont remove from vring
     */
    pMsg = RPMessage_allocEndPtMsg(remoteCoreId);
    if(pMsg!=NULL)
    {
        status = RPMessage_vringGetFullRxBuf(remoteCoreId, &vringBufId);
        if(status == SystemP_SUCCESS)
        {
            /* message in vring, extract it and copy info to message pointer and put in end point Q */
            uint8_t *vringBufAddr = RPMessage_vringGetRxBufAddr(remoteCoreId, vringBufId);
            RPMessage_Header *header = (RPMessage_Header *)vringBufAddr;
            uint16_t localEndPt = header->dstEndPt;

            status = SystemP_FAILURE;
            if(localEndPt < RPMESSAGE_MAX_LOCAL_ENDPT)
            {
                RPMessage_Struct *obj = gIpcRpmsgCtrl.localEndPtObj[localEndPt];
                if(obj!=NULL)
                {
                    if(obj->recvCallback != NULL)
                    {
                        uint16_t crc;
                        int32_t crcStatus = SystemP_SUCCESS;

                        if(gIpcRpmsgCtrl.isCrcEnabled)
                        {
                            crcStatus = gIpcRpmsgCtrl.crcHookFxn((uint8_t *)(&vringBufAddr[sizeof(RPMessage_Header)]), header->dataLen, RPMESSAGE_CRC_SIZE, &crc);
                            
                            if(header->flags != crc)
                            {
                                crcStatus = SystemP_FAILURE;
                            }
                        }
                        /* recv messages handled in callback mode */
                        obj->recvCallback( (RPMessage_Object*)obj,
                            obj->recvCallbackArgs,
                            &vringBufAddr[sizeof(RPMessage_Header)],
                            header->dataLen,
                            crcStatus,
                            remoteCoreId,
                            header->srcEndPt
                            );
                        status = SystemP_SUCCESS;

                        /* pMsg is not used, free it */
                        (void)RPMessage_freeEndPtMsg(remoteCoreId, pMsg);
                        /* done using vring buf, free it */
                        RPMessage_vringPutEmptyRxBuf(remoteCoreId, vringBufId);
                    }
                    else
                    {
                        /* recv messages handled in non-callback mode */
                        pMsg->remoteCoreId = remoteCoreId;
                        pMsg->vringBufId = vringBufId;
                        RPMessage_putEndPtMsg(obj, pMsg);
                        status = SystemP_SUCCESS;

                        if(obj->recvNotifyCallback!=NULL)
                        {
                            obj->recvNotifyCallback((RPMessage_Object*)obj, obj->recvNotifyCallbackArgs);
                        }
                    }
                }
            }
            if(status!=SystemP_SUCCESS)
            {
                /* invalid vring message header or invalid endpt
                * or no object registered for local end pt, so no need handle the message pointer,
                * free it
                */
                RPMessage_vringPutEmptyRxBuf(remoteCoreId, vringBufId);
            }
        }
        if(status!=SystemP_SUCCESS)
        {
            /* no message in vring or invalid vring message header or invalid endpt
            * or no object registered for local end pt, so no need handle the message pointer,
            * free it
            */
            (void)RPMessage_freeEndPtMsg(remoteCoreId, pMsg);
        }
    }
}

void RPMessage_notifyCallback(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    if((gIpcRpmsgCtrl.isCoreEnable[remoteCoreId]!=0U) && (gIpcRpmsgCtrl.isCoreInitialized[remoteCoreId]!=0U))
    {
        uint16_t rxMsgValue = RPMESSAGE_MSG_VRING_NEW_FULL;

        if((RPMessage_isLinuxCore(remoteCoreId)!=0U))
        {
            rxMsgValue = RPMESSAGE_LINUX_RX_VRING_ID; /* In linux, we get RX VRING ID, which is 1 in linux */
        }
        if(msgValue == rxMsgValue)
        {   /* check full ring */
            while((RPMessage_vringIsFullRxBuf(remoteCoreId)!=0U))
            {
                RPMessage_recvHandler(remoteCoreId);
            }
        }
        else
        {   /* check empty ring */

            /* check if there is any new empty buf, if yes then post semaphore to wake up any waiting threads */
            RPMessage_vringCheckEmptyTxBuf(remoteCoreId);
        }
    }
}

int32_t RPMessage_send( void*    data,
                        uint16_t dataLen,
                        uint16_t remoteCoreId,
                        uint16_t remoteEndPt,
                        uint16_t localEndPt,
                        uint32_t timeout
                      )
{
    uint16_t dataLength = dataLen;
    int32_t status = SystemP_FAILURE;

    if(((remoteCoreId < CSL_CORE_ID_MAX)) && ((gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] != 0U))
        && ((data != NULL)) && ((dataLength != 0U)) && (localEndPt < RPMESSAGE_MAX_LOCAL_ENDPT)
        )
    {
        uint16_t vringBufId;
        uint16_t crc;

        status = RPMessage_vringGetEmptyTxBuf(remoteCoreId, &vringBufId, timeout);
        if(status == SystemP_SUCCESS)
        {
            uint8_t *vringBufAddr = RPMessage_vringGetTxBufAddr(remoteCoreId, vringBufId);
            uint16_t vringBufLen = RPMessage_vringGetTxBufLen(remoteCoreId, vringBufId);
            RPMessage_Header *header = (RPMessage_Header *)vringBufAddr;

            if(dataLength > (vringBufLen - sizeof(RPMessage_Header)) )
            {
                dataLength = vringBufLen - sizeof(RPMessage_Header);

                DebugP_logWarn("[IPC RPMSG] Message send to remote core %d @ %d end point truncated due to lack of space in vring buffer !!!\r\n",
                    remoteCoreId, remoteEndPt);
            }

            header->srcEndPt = localEndPt;
            header->dstEndPt = remoteEndPt;
            header->srcCoreId = gIpcRpmsgCtrl.selfCoreId;
            if(gIpcRpmsgCtrl.isCrcEnabled)
            {
                status = gIpcRpmsgCtrl.crcHookFxn((uint8_t *)data, dataLen, RPMESSAGE_CRC_SIZE, &crc);
                header->flags = crc;
            }
            else
            {
                header->flags = 0;
            }
            header->dataLen = dataLength;

            memcpy((void *) &vringBufAddr[sizeof(RPMessage_Header)], (const void *) data, (size_t)dataLength);

            status = RPMessage_vringPutFullTxBuf(remoteCoreId, vringBufId, dataLength + sizeof(RPMessage_Header), timeout);

            if(status != SystemP_SUCCESS)
            {
                DebugP_logWarn("[IPC RPMSG] Message send to remote core %d @ %d end point failed due to lack of space in Notify Queue !!!\r\n",
                remoteCoreId, remoteEndPt);
            }
        }
        else
        {
            DebugP_logWarn("[IPC RPMSG] Message send to remote core %d @ %d end point failed due to lack of space in vring !!!\r\n",
                remoteCoreId, remoteEndPt);
        }
    }
    else
    {
        DebugP_logError("[IPC RPMSG] Message send to remote core %d @ %d end point failed due to invalid parameters !!!\r\n",
            remoteCoreId, remoteEndPt
            );
    }
    return status;
}

int32_t RPMessage_recv(RPMessage_Object *handle, void* data, uint16_t *dataLen,
                      uint16_t *remoteCoreId, uint16_t *remoteEndPt, uint32_t timeout)
{
    int32_t status = SystemP_FAILURE;
    RPMessage_Struct *obj = (RPMessage_Struct *)handle;

   if( (data != NULL) && (dataLen != NULL) && (remoteCoreId != NULL) && (remoteEndPt != NULL)
        && (obj->recvCallback == NULL) /* i.e non-callback mode */
      )
    {
        RPMessage_LocalMsg *pMsg;

        status = RPMessage_getEndPtMsg(obj, &pMsg, timeout);
        if((status == SystemP_SUCCESS) && (pMsg != NULL))
        {
            uint16_t crc;
            uint32_t isAllocPending = 0;
            uint16_t vringBufId = pMsg->vringBufId;
            uint8_t *vringBufAddr = RPMessage_vringGetRxBufAddr(pMsg->remoteCoreId, vringBufId);
            RPMessage_Header *header = (RPMessage_Header *)vringBufAddr;

            *remoteCoreId = pMsg->remoteCoreId;
            *remoteEndPt =  header->srcEndPt;

            if( *dataLen < header->dataLen )
            {
                DebugP_logWarn("[IPC RPMSG] Message recv @ %d local end point truncated due to insufficient user buffer size !!!\r\n",
                    obj->localEndPt
                    );
            }
            else
            {
                *dataLen = header->dataLen;
            }

            memcpy((void *) data,(const void *) &vringBufAddr[sizeof(RPMessage_Header)],(size_t) *dataLen);

            if(gIpcRpmsgCtrl.isCrcEnabled)
            {
                status = gIpcRpmsgCtrl.crcHookFxn((uint8_t *)data, *dataLen, RPMESSAGE_CRC_SIZE, &crc);
                
                if(header->flags != crc)
                {
                    status = SystemP_FAILURE;
                }
            }

            RPMessage_vringPutEmptyRxBuf(*remoteCoreId, vringBufId);
            isAllocPending = RPMessage_freeEndPtMsg(*remoteCoreId, pMsg);
            if(isAllocPending!=0U)
            {   /* if any messages are pending message pointer due to free Q being empty,
                 * now there will be atleast one element to handle any pending vring requests.
                 * So check vring and handle pending messages if any
                 */
                RPMessage_notifyCallback(*remoteCoreId,
                    IPC_NOTIFY_CLIENT_ID_RPMSG,
                    RPMESSAGE_MSG_VRING_NEW_FULL,
                    SystemP_SUCCESS,
                    NULL);
            }
        }
        else
        {
            DebugP_logError("[IPC RPMSG] Message recv @ %d local end point failed due to invalid end point Q !!!\r\n",
                    obj->localEndPt
                    );
        }
    }
    else
    {
        DebugP_logError("[IPC RPMSG] Message recv @ %d local end point failed due to invalid parameters !!!\r\n",
            obj->localEndPt
            );
    }
    return status;
}

void RPMessage_unblock(RPMessage_Object *handle)
{
    RPMessage_Struct *obj;

    if(handle != NULL)
    {
    obj = (RPMessage_Struct *)handle;
    obj->doRecvUnblock = 1;
    SemaphoreP_post(&obj->newEndPtMsgSem);
    }
}

uint16_t RPMessage_getLocalEndPt(const RPMessage_Object *handle)
{
    RPMessage_Struct *obj;

    DebugP_assert(handle != NULL);

    obj = (RPMessage_Struct *)handle;
    return obj->localEndPt;

}

int32_t RPMessage_construct(RPMessage_Object *handle, const RPMessage_CreateParams *createParams)
{
    RPMessage_Struct *obj;
    int32_t status = SystemP_FAILURE;

    DebugP_assert(sizeof(RPMessage_Object) >= sizeof(RPMessage_Struct));

    if((handle != NULL) && (createParams != NULL))
    {
    obj = (RPMessage_Struct *)handle;
    if((createParams->localEndPt < RPMESSAGE_MAX_LOCAL_ENDPT)
        && (gIpcRpmsgCtrl.localEndPtObj[createParams->localEndPt] == NULL))
    {
        obj->localEndPt = createParams->localEndPt;
        obj->recvCallback = createParams->recvCallback;
        obj->recvCallbackArgs = createParams->recvCallbackArgs;
        obj->recvNotifyCallback = createParams->recvNotifyCallback;
        obj->recvNotifyCallbackArgs = createParams->recvNotifyCallbackArgs;
        obj->doRecvUnblock = 0;
        RPMessage_queueReset(&obj->endPtQ);
        SemaphoreP_constructBinary(&obj->newEndPtMsgSem, 0);

        gIpcRpmsgCtrl.localEndPtObj[createParams->localEndPt] = obj;

        status = SystemP_SUCCESS;
    }
    }
    return status;
}

void RPMessage_destruct(RPMessage_Object *handle)
{
    RPMessage_Struct *obj;

    if(handle != NULL)
    {
    obj = (RPMessage_Struct *)handle;
    if((obj->localEndPt < RPMESSAGE_MAX_LOCAL_ENDPT) &&
        (gIpcRpmsgCtrl.localEndPtObj[obj->localEndPt] != NULL))
    {
        gIpcRpmsgCtrl.localEndPtObj[obj->localEndPt] = NULL;

        obj->localEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
        obj->recvCallback = NULL;
        obj->recvCallbackArgs = NULL;
        obj->doRecvUnblock = 0;
        RPMessage_queueReset(&obj->endPtQ);
        SemaphoreP_destruct(&obj->newEndPtMsgSem);
    }
    }
}

void RPMessage_CreateParams_init(RPMessage_CreateParams *params)
{
    params->localEndPt = RPMESSAGE_MAX_LOCAL_ENDPT;
    params->recvCallback = NULL;
    params->recvCallbackArgs = NULL;
    params->recvNotifyCallback = NULL;
    params->recvNotifyCallbackArgs = NULL;
}

void RPMessage_Params_init(RPMessage_Params *params)
{
    uint16_t coreId;

    if(params != NULL)
    {
        memset(params, 0, sizeof(RPMessage_Params));

        for(coreId=0; coreId<CSL_CORE_ID_MAX; coreId++)
        {
            params->vringTxBaseAddr[coreId] = RPMESSAGE_VRING_ADDR_INVALID;
            params->vringRxBaseAddr[coreId] = RPMESSAGE_VRING_ADDR_INVALID;
        }
        params->vringNumBuf = 8;
        params->vringMsgSize = 128;
        params->vringSize = RPMESSAGE_VRING_SIZE((uint32_t)params->vringNumBuf, params->vringMsgSize);
        params->linuxCoreId = CSL_CORE_ID_MAX;
        params->linuxResourceTable = NULL;
        params->isCrcEnabled = 0;
        params->crcHookFxn = NULL;
    }
}

int32_t  RPMessage_coreInit(uint16_t remoteCoreId, const RPMessage_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    RPMessage_Core *coreObj = &gIpcRpmsgCtrl.coreObj[remoteCoreId];
    uint16_t elemId;

    SemaphoreP_constructBinary(&coreObj->newEmptyVringBufSem, 0);
    coreObj->freeQAllocPending = 0;
    RPMessage_queueReset(&coreObj->freeQ);
    for(elemId=0; elemId<RPMESSAGE_MAX_LOCAL_MSG_OBJ; elemId++)
    {
        RPMessage_queuePut(&coreObj->freeQ, &coreObj->localMsgObj[elemId].elem);
    }
    /* Linux VRINGs we will init later inside RPMessage_waitForLinuxReady() */
    if((gIpcRpmsgCtrl.isCoreEnable[remoteCoreId] != 0U) && (RPMessage_isLinuxCore(remoteCoreId) == 0U))
    {
        /* reset RX ring */
        RPMessage_vringReset(remoteCoreId, 0, params);
        /* reset TX ring */
        RPMessage_vringReset(remoteCoreId, 1, params);

        /* mark core data structure as initialized, now we can handle interrupts */
        gIpcRpmsgCtrl.isCoreInitialized[remoteCoreId] = 1;
    }
    return status;
}

void RPMessage_coreDeInit(uint16_t remoteCoreId)
{
    RPMessage_Core *coreObj = &gIpcRpmsgCtrl.coreObj[remoteCoreId];

    SemaphoreP_destruct(&coreObj->newEmptyVringBufSem);
    coreObj->freeQAllocPending = 0;
    RPMessage_queueReset(&coreObj->freeQ);
}

void RPMessage_forceRecvMsgHandlers(void)
{
    uint16_t coreId;

    for(coreId=0; coreId<CSL_CORE_ID_MAX; coreId++)
    {
        RPMessage_notifyCallback(coreId,
            IPC_NOTIFY_CLIENT_ID_RPMSG,
            RPMESSAGE_MSG_VRING_NEW_FULL,
            SystemP_SUCCESS,
            NULL);
    }
}

void RPMessage_controlEndPtHandler(RPMessage_Object *obj, void *arg,
        void *data, uint16_t dataLen, int32_t crcStatus,
        uint16_t remoteCoreId, uint16_t remoteEndPt)
{
    if(gIpcRpmsgCtrl.controlEndPtCallback!=NULL)
    {
        /* check if message is of correct size */
        if(dataLen == sizeof(RPMessage_AnnounceMsg))
        {
            /* invoke user callback */
            RPMessage_AnnounceMsg *pMsg = (RPMessage_AnnounceMsg*)data;

            gIpcRpmsgCtrl.controlEndPtCallback(
                gIpcRpmsgCtrl.controlEndPtCallbackArgs,
                remoteCoreId,
                pMsg->remoteEndPt,
                pMsg->name
                );
        }
    }
}

int32_t RPMessage_controlEndPtInit(void)
{
    RPMessage_CreateParams createPrms;
    int32_t status;

    RPMessage_CreateParams_init(&createPrms);
    createPrms.localEndPt = RPMESSAGE_CTRL_ENDPOINT_ID;
    createPrms.recvCallback = RPMessage_controlEndPtHandler;
    status = RPMessage_construct(&gIpcRpmsgCtrl.controlEndPtObj, &createPrms);

    return status;
}

void RPMessage_controlEndPtDeInit(void)
{
    RPMessage_destruct(&gIpcRpmsgCtrl.controlEndPtObj);
}

int32_t  RPMessage_announce(uint16_t remoteCoreId, uint16_t localEndPt, const char* name)
{
    int32_t status = SystemP_FAILURE;
    RPMessage_AnnounceMsg msg;

    if((remoteCoreId < CSL_CORE_ID_MAX) && (localEndPt < RPMESSAGE_MAX_LOCAL_ENDPT) && (name != NULL))
    {
    msg.type = 0;
    msg.remoteEndPt = localEndPt; /* local end point will be remote end point for the other side */
    strncpy(msg.name, name, RPMESSAGE_ANNOUNCE_SERVICENAME_LEN-1U);
    msg.name[RPMESSAGE_ANNOUNCE_SERVICENAME_LEN-1U] = '\0';

    status = RPMessage_send(
                &msg,
                sizeof(RPMessage_AnnounceMsg),
                remoteCoreId,
                RPMESSAGE_CTRL_ENDPOINT_ID, /* control end point on remote side */
                RPMESSAGE_CTRL_ENDPOINT_ID, /* reply or local end point, set also to control end point */
                SystemP_WAIT_FOREVER /* wait until message is put in VRING */
    );
    }
    return status;
}

void RPMessage_controlEndPtCallback(RPMessage_ControlEndPtCallback controlEndPtCallback,
    void  *controlEndPtCallbackArgs)
{
    uint32_t oldIntState;

    if(controlEndPtCallback != NULL)
    {
        oldIntState = HwiP_disable();
        gIpcRpmsgCtrl.controlEndPtCallback = controlEndPtCallback;
        gIpcRpmsgCtrl.controlEndPtCallbackArgs = controlEndPtCallbackArgs;

        HwiP_restore(oldIntState);
    }
}

uint32_t RPMessage_isLinuxCore(uint16_t coreId)
{
    uint32_t isLinuxCore = 0;

    if((coreId == gIpcRpmsgCtrl.linuxCoreId) && (gIpcRpmsgCtrl.linuxResourceTable))
    {
        isLinuxCore = 1;
    }
    return isLinuxCore;
}

int32_t  RPMessage_waitForLinuxReady(uint32_t timeout)
{
    int32_t status = SystemP_FAILURE;
    volatile RPMessage_ResourceTable *rscTable = (RPMessage_ResourceTable *)gIpcRpmsgCtrl.linuxResourceTable;

    if(rscTable!=NULL)
    {
        uint32_t elaspedTicks, startTicks = ClockP_getTicks();
        do
        {
            CacheP_inv((void*)rscTable, sizeof(RPMessage_ResourceTable), CacheP_TYPE_ALL);
            if(rscTable->vdev.status == 0x7U)
            {
                /* linux has initialized the resource table, break out */
                status = SystemP_SUCCESS;
            }
            if(status != SystemP_SUCCESS)
            {
                elaspedTicks = ClockP_getTicks() - startTicks;
                if( elaspedTicks >= timeout)
                {
                    /* timeout, linux did not init the resource table in user specific timeout time */
                    status = SystemP_TIMEOUT;
                }
                if(status != SystemP_TIMEOUT)
                {
                    /* sleep one tick */
                    ClockP_usleep(ClockP_ticksToUsec(1));
                }
            }
        } while(status == SystemP_FAILURE);

        if(status == SystemP_SUCCESS)
        {
            /* init virtio on linux side */

            /* initialize RX VRING */
            RPMessage_vringResetLinux(
                    gIpcRpmsgCtrl.linuxCoreId,
                    0,
                    gIpcRpmsgCtrl.linuxResourceTable);
            /* initialize TX VRING */
            RPMessage_vringResetLinux(
                    gIpcRpmsgCtrl.linuxCoreId,
                    1,
                    gIpcRpmsgCtrl.linuxResourceTable);

            /* mark core data structure as initialized, now we can handle interrupts */
            gIpcRpmsgCtrl.isCoreInitialized[gIpcRpmsgCtrl.linuxCoreId] = 1;
        }
    }

    return status;
}

int32_t  RPMessage_init(const RPMessage_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t coreId, localEndPtId;

    if(params != NULL)
    {
    gIpcRpmsgCtrl.selfCoreId = IpcNotify_getSelfCoreId();
    gIpcRpmsgCtrl.controlEndPtCallback = NULL;
    gIpcRpmsgCtrl.controlEndPtCallbackArgs = NULL;
    gIpcRpmsgCtrl.linuxResourceTable = params->linuxResourceTable;
    gIpcRpmsgCtrl.linuxCoreId = params->linuxCoreId;
    gIpcRpmsgCtrl.isCrcEnabled = params->isCrcEnabled;
    gIpcRpmsgCtrl.crcHookFxn = params->crcHookFxn;
    for(localEndPtId = 0; localEndPtId < RPMESSAGE_MAX_LOCAL_ENDPT; localEndPtId++)
    {
        gIpcRpmsgCtrl.localEndPtObj[localEndPtId] = NULL;
    }
    for(coreId=0; coreId<CSL_CORE_ID_MAX; coreId++)
    {
        /* enable a core for RPMessage only when below is satisifed
         * - valid vring ID is set
         * - not self core ID
         * - IPC Notify with that core is enabled
         */
        gIpcRpmsgCtrl.isCoreEnable[coreId] = 0;
        gIpcRpmsgCtrl.isCoreInitialized[coreId] = 0;
       if((params->vringTxBaseAddr[coreId] != RPMESSAGE_VRING_ADDR_INVALID)
            &&
            (params->vringRxBaseAddr[coreId] != RPMESSAGE_VRING_ADDR_INVALID)
            &&
            (coreId != gIpcRpmsgCtrl.selfCoreId)
            &&
            ((IpcNotify_isCoreEnabled(coreId))!=0U)
          )
        {
            gIpcRpmsgCtrl.isCoreEnable[coreId] = 1;
        }
        if((RPMessage_isLinuxCore(coreId)!=0U)
            && (IpcNotify_isCoreEnabled(coreId)!=0U)
            )
        {
            gIpcRpmsgCtrl.isCoreEnable[coreId] = 1;
        }

    }
    for(coreId=0; coreId<CSL_CORE_ID_MAX; coreId++)
    {
        status += RPMessage_coreInit(coreId, params);
    }

    /* create control end point */
    status += RPMessage_controlEndPtInit();

    IpcNotify_registerClient(IPC_NOTIFY_CLIENT_ID_RPMSG,
        RPMessage_notifyCallback, NULL
        );

    }
    return status;
}

void  RPMessage_deInit(void)
{
    uint16_t coreId;

    IpcNotify_unregisterClient(IPC_NOTIFY_CLIENT_ID_RPMSG);

    RPMessage_controlEndPtDeInit();

    for(coreId=0; coreId<CSL_CORE_ID_MAX; coreId++)
    {
        RPMessage_coreDeInit(coreId);
    }
}

