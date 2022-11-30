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
#ifndef MAILBOX_V0_PRIV_H_
#define MAILBOX_V0_PRIV_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/ipc_notify.h>
#include <drivers/mailbox.h>

/* object representing information of remote core for mailbox communication */
typedef struct {

    SemaphoreP_Object readSem;  /* semaphore used to wait on read interrupt */
    uint32_t curReadSize; /* current size of data read from shared memory */

    uint32_t maxBufferSize; /* max buffer size for read and write */
    uint8_t *writeShmBuffer; /* shared memory used to send data to remote core */
    uint8_t *readShmBuffer; /* shared memory used to read data from remote core */


    /* register address to trigger write interrupt on remote core */
    uint32_t writeIntrRegAddr;
    /* register address to check write ACK interrupt from remote core */
    uint32_t writeAckIntrRegAddr;
    /* register address to trigger read ACK interrupt on remote core */
    uint32_t readAckIntrRegAddr;

    /* bit position for this remote core for various registers */
    uint32_t writeIntrBitPos;
    uint32_t writeAckIntrBitPos;
    uint32_t readAckIntrBitPos;

} Mailbox_RemoteCoreObj;

/* mailbox driver global state */
typedef struct {

    /* remote core mailbox communication info and state, when NULL, mailbox communication with that core is not supported */
    Mailbox_RemoteCoreObj *pRemoteCoreObj[CSL_CORE_ID_MAX];

    /* user callback to call when a read interrupt is received */
    Mailbox_ReadCallback readCallback;
    /* user callback arguments */
    void *readCallbackArgs;

} Mailbox_Ctrl;

/* SOC specific function, implemented in soc specific file mailbox/soc/{soc}/mailbox_cfg.c

   Returns non NULL object when mailbox communication is supported between self core and remote core
 */
Mailbox_RemoteCoreObj *Mailbox_getRemoteCoreObj(uint32_t selfCoreId, uint32_t remoteCoreId);


#ifdef __cplusplus
}
#endif

#endif
