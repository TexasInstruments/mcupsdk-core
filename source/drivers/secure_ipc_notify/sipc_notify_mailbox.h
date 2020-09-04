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

#ifndef SIPC_NOTIFY_MAILBOX_H_
#define SIPC_NOTIFY_MAILBOX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* this file has define's and inline function's to program the HW mailbox registers and SW queue structure */
#define MAILBOX_MAX_SW_QUEUE_STRUCT_SIZE      (sizeof(SIPC_SwQueue))

/* The HW mailbox only allows to trigger a interrupt on another core,
 * the SIPC Notify needs ability to pass x byte message along with a interrupt
 *
 * Basically this mimics the functionality of HW mailbox with HW FIFO in AM243x SOC
 *
 * This needs to be in sync with the addresses being set for SW queue memory in soc/{soc}/sipc_notify_cfg.c
 *
 * The new queue has two more parameters i.e EleSize = Size of 1 queue element in words
 * Qlength = total length of this Queue */

/**
 * @brief
 * @ingroup DRV_SIPC_NOTIFY_MODULE
 *  SIPC swQ structure which holds the data pointer to a fifo Queue in HSM MBOX memory.
 */
typedef struct SIPC_SwQueue_
{
    uint32_t rdIdx; /**<queue element will be read from this index.*/
    uint32_t wrIdx; /**<queue element will be written to this index.*/
    uint16_t EleSize ; /**<Element size in words this will be a fixed parameter */
    uint16_t Qlength ; /**<total number of elements */
    uint8_t *Qfifo; /**Pointer to the FIFO queue in HSM MBOX memory */
} SIPC_SwQueue;

/* Read from SW fifo within a mailbox  */
static inline int32_t SIPC_mailboxRead(SIPC_SwQueue *swQ, uint8_t *Buff)
{
    int32_t status = SystemP_FAILURE;

    volatile uint32_t rdIdx = swQ->rdIdx;
    volatile uint32_t wrIdx = swQ->wrIdx;

    if(rdIdx < swQ->Qlength && wrIdx < swQ->Qlength)
    {
        /* If this condition meets then it means there is something in the fifo*/
        if( rdIdx != wrIdx)
        {
            /* Copy EleSize bytes from Queue memory to the buffer */
            memcpy(Buff, (swQ->Qfifo + (swQ->EleSize*rdIdx)),swQ->EleSize);

            rdIdx = (rdIdx+1)%swQ->Qlength;

            swQ->rdIdx = rdIdx;

            rdIdx = swQ->rdIdx; /* read back to ensure the update has reached the memory */

            #if defined(__aarch64__) || defined(__arm__)
            __asm__ __volatile__( "dsb sy" "\n\t": : : "memory");
            __asm__ __volatile__( "isb" "\n\t": : : "memory");
            #endif

            status = SystemP_SUCCESS;
        }
    }

    return status;
}

/* Write to SW fifo and trigger HW interrupt using HW mailbox */
static inline int32_t SIPC_mailboxWrite(uint32_t mailboxBaseAddr, uint32_t wrIntrBitPos, SIPC_SwQueue *swQ, uint8_t *Buff)
{
    int32_t status = SystemP_FAILURE;

    volatile uint32_t rdIdx = swQ->rdIdx;
    volatile uint32_t wrIdx = swQ->wrIdx;

    if(rdIdx < swQ->Qlength && wrIdx < swQ->Qlength)
    {
        if( ( (wrIdx+1)%swQ->Qlength ) != rdIdx )
        {
            volatile uint32_t *addr = (uint32_t *)mailboxBaseAddr;

            /* There is some space in the FIFO */

            memcpy((swQ->Qfifo + (swQ->EleSize*wrIdx)),Buff,swQ->EleSize);

            wrIdx = (wrIdx+1)%swQ->Qlength;

            swQ->wrIdx = wrIdx;

            wrIdx = swQ->wrIdx; /* read back to ensure the update has reached the memory */

            #if defined(__aarch64__) || defined(__arm__)
            __asm__( "dsb sy" "\n\t": : : "memory");
            __asm__( "isb"    "\n\t": : : "memory");
            #endif

            /* Trigger interrupt to other core */
            *addr = (1U << (wrIntrBitPos));

            status = SystemP_SUCCESS;
        }
    }
    return status;
}

static inline void SIPC_mailboxClearAllInt(uint32_t mailboxBaseAddr)
{
    volatile uint32_t *addr = (uint32_t *)mailboxBaseAddr;
    *addr = 0x1111111;
}

static inline uint32_t SIPC_mailboxGetPendingIntr(uint32_t mailboxBaseAddr)
{
    volatile uint32_t *addr = (uint32_t *)mailboxBaseAddr;

    return *addr;
}

static inline void SIPC_mailboxClearPendingIntr(uint32_t mailboxBaseAddr, uint32_t pendingIntr)
{
    volatile uint32_t *addr = ( uint32_t *)mailboxBaseAddr;
    *addr = pendingIntr;
}

static inline uint32_t SIPC_mailboxIsPendingIntr(uint32_t pendingIntr, uint32_t coreId)
{
    extern uint32_t gSIPCCoreIntrBitPos[];

    uint32_t isPending = 0;
    isPending = pendingIntr & (1 << gSIPCCoreIntrBitPos[coreId]);
    return isPending;
}

#ifdef __cplusplus
}
#endif

#endif /*SIPC_NOTIFY_MAILBOX_H_*/

