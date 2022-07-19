/*
 *  Copyright (C) 2020-2023 Texas Instruments Incorporated
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
 *
 */

#include "mailbox_priv.h"

#if defined(__aarch64__) || defined(__arm__)
#define ASM_DSP  __asm__( "dsb sy")
#define ASM_ISB  __asm__( "isb")
#endif

/* global state of mailbox communication */
Mailbox_Ctrl gMailbox_ctrl;

static inline uint32_t Mailbox_isWriteAckIntr(const Mailbox_RemoteCoreObj *obj)
{
    volatile uint32_t *addr = (uint32_t*)obj->writeAckIntrRegAddr;

    return (uint32_t) (((uint32_t) *addr) & ((uint32_t) (1UL << (obj->writeAckIntrBitPos))));
}

static inline void Mailbox_clearWriteAckIntr(const Mailbox_RemoteCoreObj *obj)
{
    volatile uint32_t *addr = (uint32_t*)obj->writeAckIntrRegAddr;

    *addr =(uint32_t) (1UL << obj->writeAckIntrBitPos);
}

static inline void Mailbox_sendWriteIntr(const Mailbox_RemoteCoreObj *obj)
{
    volatile uint32_t *addr = (uint32_t*)obj->writeIntrRegAddr;

    *addr =(uint32_t) (1UL << obj->writeIntrBitPos);
}

static inline void Mailbox_sendReadAckIntr(const Mailbox_RemoteCoreObj *obj)
{
    volatile uint32_t *addr = (uint32_t*)obj->readAckIntrRegAddr;

    *addr = (uint32_t)(1UL << obj->readAckIntrBitPos);
}

static int32_t Mailbox_waitWriteAckIntr(const Mailbox_RemoteCoreObj *obj, uint32_t timeToWaitInTicks)
{
    int32_t status = SystemP_FAILURE;
    uint32_t done = 0, isAck;
    uint32_t curTicks = ClockP_getTicks(), elaspedTicks;

    do {
        isAck = Mailbox_isWriteAckIntr(obj);
        if(isAck != 0U)
        {
            Mailbox_clearWriteAckIntr(obj);
            status = SystemP_SUCCESS;
            done = 1;
        }
        if(done == 0U)
        {
            elaspedTicks =  ClockP_getTicks() - curTicks;

            if(elaspedTicks >= timeToWaitInTicks)
            {
                status = SystemP_TIMEOUT;
                done = 1;
            }
        }
    } while(done == 0U);

    return status;
}

void Mailbox_readCallback(uint32_t remoteCoreId)
{
    if( Mailbox_isCoreEnabled(remoteCoreId) != 0U )
    {
        Mailbox_RemoteCoreObj *obj = gMailbox_ctrl.pRemoteCoreObj[remoteCoreId];

        SemaphoreP_post( &obj->readSem );

        if( gMailbox_ctrl.readCallback != NULL)
        {
            gMailbox_ctrl.readCallback(remoteCoreId, gMailbox_ctrl.readCallbackArgs);
        }
    }
}

void Mailbox_Params_init(Mailbox_Params *params)
{
    params->rsv = 0U;
}

/* we assume IpcNotify_init() is called before calling mailbox init */
int32_t Mailbox_init(Mailbox_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t core, numCores = 0U;

    gMailbox_ctrl.readCallback = NULL;
    gMailbox_ctrl.readCallbackArgs = NULL;

    for(core=0; core<CSL_CORE_ID_MAX; core++)
    {
        gMailbox_ctrl.pRemoteCoreObj[core] = NULL;

        if( (IpcNotify_isCoreEnabled(core)==0U) && (core != IpcNotify_getSelfCoreId()))
        {
            Mailbox_RemoteCoreObj *obj;

            /* This function is defined per SOC since this information it returned is per SOC */
            obj = Mailbox_getRemoteCoreObj(
                        IpcNotify_getSelfCoreId(),
                        core
                        );
            if(obj != NULL)
            {
                numCores++;

                /* we then initialize the SOC independant state */
                status = SemaphoreP_constructBinary( &obj->readSem, 0U );
                DebugP_assert(status == SystemP_SUCCESS);

                obj->curReadSize = 0U;

                gMailbox_ctrl.pRemoteCoreObj[core] = obj;
            }
        }
    }

    if(numCores > 0U)
    {
        IpcNotify_registerNonNotifyCallback(Mailbox_readCallback);
    }

    return status;
}

void Mailbox_setReadCallback(Mailbox_ReadCallback readCallback, void *readCallbackArgs)
{
    gMailbox_ctrl.readCallbackArgs = readCallbackArgs;
    gMailbox_ctrl.readCallback = readCallback;
}

uint32_t Mailbox_isCoreEnabled(uint32_t coreId)
{
    uint32_t isEnabled = 0U;

    if((coreId < CSL_CORE_ID_MAX) && (gMailbox_ctrl.pRemoteCoreObj[coreId] != NULL))
    {
        isEnabled = 1U;
    }
    return isEnabled;
}

int32_t Mailbox_write(uint32_t remoteCoreId, const uint8_t *buffer, uint32_t size, uint32_t timeToWaitInTicks)
{
    int32_t status = SystemP_FAILURE;

    if(    (buffer != NULL)
        && (size > 0U)
        && (Mailbox_isCoreEnabled(remoteCoreId)!= 0U) )
    {
        Mailbox_RemoteCoreObj *obj = gMailbox_ctrl.pRemoteCoreObj[remoteCoreId];

        if(size < obj->maxBufferSize)
        {
            (void)memcpy(obj->writeShmBuffer, buffer, size);

            #if defined(__aarch64__) || defined(__arm__)
            __asm__ __volatile__ ( "dsb sy"  "\n\t": : : "memory");
            __asm__ __volatile__ ( "isb sy"     "\n\t": : : "memory");
            #endif
            #if defined(_TMS320C6X)
            _mfence();
            _mfence();
            #endif

            Mailbox_clearWriteAckIntr(obj);
            Mailbox_sendWriteIntr(obj);

            status = SystemP_SUCCESS;

            if(timeToWaitInTicks > 0U )
            {
                status = Mailbox_waitWriteAckIntr(obj, timeToWaitInTicks);
            }
        }
    }
    return status;
}

int32_t Mailbox_read(uint32_t remoteCoreId, uint8_t *buffer, uint32_t size, uint32_t timeToWaitInTicks)
{
    int32_t status = SystemP_FAILURE;
    uint32_t maxSize = size;
    if(    (buffer !=  NULL)
        && (maxSize > 0U)
        && (Mailbox_isCoreEnabled(remoteCoreId) != 0U) )
    {
        Mailbox_RemoteCoreObj *obj = gMailbox_ctrl.pRemoteCoreObj[remoteCoreId];

        if( (obj->curReadSize + maxSize) > obj->maxBufferSize)
        {
            maxSize = obj->maxBufferSize - obj->curReadSize;
        }
        if(maxSize > 0U)
        {
            status = SystemP_SUCCESS;

            if(obj->curReadSize == 0U)
            {
                status = SemaphoreP_pend(&obj->readSem, timeToWaitInTicks);
            }
            if(status == SystemP_SUCCESS)
            {
                (void)memcpy(buffer, &obj->readShmBuffer[obj->curReadSize], maxSize);

                obj->curReadSize += maxSize;
            }
        }
    }
    return status;
}

int32_t Mailbox_readDone(uint32_t remoteCoreId)
{
    int32_t status = SystemP_FAILURE;

    if(  Mailbox_isCoreEnabled(remoteCoreId) != 0U )
    {
        Mailbox_RemoteCoreObj *obj = gMailbox_ctrl.pRemoteCoreObj[remoteCoreId];

        obj->curReadSize = 0U;

        Mailbox_sendReadAckIntr(obj);

        status = SystemP_SUCCESS;
    }
    return status;
}
