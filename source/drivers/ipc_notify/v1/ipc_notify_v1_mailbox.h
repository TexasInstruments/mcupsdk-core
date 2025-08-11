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

#ifndef IPC_NOTIFY_V1_MAILBOX_H_
#define IPC_NOTIFY_V1_MAILBOX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* this file has define's and inline function's to program the HW mailbox registers and SW queue structure */

#define MAILBOX_MAX_MSGS_IN_SW_FIFO          ( 4u)
#define MAILBOX_MAX_SW_QUEUE_SIZE            (32u)

/* The HW mailbox only allows to trigger a interrupt on another core,
 * the IPC Notify needs ability to pass 32b value along with a interrupt
 * This 32b value is kept a MAILBOX_MAX_MSGS_IN_FIFO deep SW queue
 *
 * Basically this mimics the functionality of HW mailbox with HW FIFO in AM243x SOC
 *
 * Note, recommend to not modify MAILBOX_MAX_MSGS_IN_SW_FIFO and MAILBOX_MAX_SW_QUEUE_SIZE
 * and IpcNotify_SwQueue struct.
 *
 * This needs to be in sync with the addresses being set for SW queue memory in soc/{soc}/ipc_notify_v1_cfg.c
 */
typedef struct {

    uint32_t rdIdx;
    uint32_t wrIdx;
    uint32_t fifo[MAILBOX_MAX_MSGS_IN_SW_FIFO];

} IpcNotify_SwQueue;

#if !(defined(SOC_AM273X))
extern void IpcNotify_trigInterrupt(uint32_t mailboxBaseAddr, uint32_t intrBitPos);
#else
extern int32_t IpcNotify_trigInterrupt(uint32_t selfCoreId, uint32_t remoteCoreId, uint32_t mailboxBaseAddr, uint32_t intrBitPos);
#endif

/* Function to set Instruction Synchronization Barrier (ISB) and Data Synchronization Barrier on arm core (Cortex-R/Cortex-M).
 * This ensure that all instructions and memory transactions, including cache operations, are completed. This is needed
 * to avoid any coherency issue especially while doing read/write operation on a shared memory.
 * For C66 core, _mefence() can be used for this purpose.
 */
#if defined(__aarch64__) || defined(__arm__)
static inline void IpcNotify_dataAndInstructionBarrier(void)
{
    __asm__ __volatile__ ( "dsb sy"  "\n\t": : : "memory");
    __asm__ __volatile__ ( "isb"     "\n\t": : : "memory");
}
#endif

/* read from SW fifo within a mailbox  */
static inline int32_t IpcNotify_mailboxReadSwQ(IpcNotify_SwQueue *swQ, uint32_t *value)
{
    int32_t status = SystemP_FAILURE;

    uint32_t rdIdx = swQ->rdIdx;
    uint32_t wrIdx = swQ->wrIdx;

    if((rdIdx < MAILBOX_MAX_MSGS_IN_SW_FIFO) && (wrIdx < MAILBOX_MAX_MSGS_IN_SW_FIFO))
    {
        if( rdIdx != wrIdx)
        {
            /* there is something in the FIFO */
            *value = swQ->fifo[rdIdx];

            rdIdx = (rdIdx+1U)%MAILBOX_MAX_MSGS_IN_SW_FIFO;

            swQ->rdIdx = rdIdx;

            rdIdx = swQ->rdIdx; /* read back to ensure the update has reached the memory */

            /* ensure that all instructions and memory transactions, including cache operations, are completed
            * this is required to avoid any multi-core coherency issue since shared memory is being written/accessed
            */
            #if defined(__aarch64__) || defined(__arm__)
            IpcNotify_dataAndInstructionBarrier();
            #endif
            #if defined(_TMS320C6X)
            _mfence();
            _mfence();
            #endif

            status = SystemP_SUCCESS;
        }
    }

    return status;
}

/* write to SW fifo and trigger HW interrupt using HW mailbox */
#if !(defined(SOC_AM273X))
static inline int32_t IpcNotify_mailboxWrite(uint32_t mailboxBaseAddr, uint32_t intrBitPos, IpcNotify_SwQueue *swQ, uint32_t value)
#else
static inline int32_t IpcNotify_mailboxWrite(uint32_t selfCoreId, uint32_t remoteCoreId, uint32_t mailboxBaseAddr, uint32_t intrBitPos, IpcNotify_SwQueue *swQ, uint32_t value)
#endif
{
    int32_t status = SystemP_FAILURE;

    uint32_t rdIdx = swQ->rdIdx;
    uint32_t wrIdx = swQ->wrIdx;

    if((rdIdx < MAILBOX_MAX_MSGS_IN_SW_FIFO) && (wrIdx < MAILBOX_MAX_MSGS_IN_SW_FIFO))
    {
        if( ( (wrIdx+1U)%MAILBOX_MAX_MSGS_IN_SW_FIFO ) != rdIdx )
        {
            /* there is some space in the FIFO */
            swQ->fifo[wrIdx] = value;

            wrIdx = (wrIdx+1U)%MAILBOX_MAX_MSGS_IN_SW_FIFO;

            swQ->wrIdx = wrIdx;

            wrIdx = swQ->wrIdx; /* read back to ensure the update has reached the memory */

            /* ensure that all instructions and memory transactions, including cache operations, are completed
            * this is required to avoid any multi-core coherency issue since shared memory is being written/accessed
            */
            #if defined(__aarch64__) || defined(__arm__)
            IpcNotify_dataAndInstructionBarrier();
            #endif
            #if defined(_TMS320C6X)
            _mfence();
            _mfence();
            #endif

            /* trigger interrupt to other core */
#if !(defined(SOC_AM273X))
            IpcNotify_trigInterrupt(mailboxBaseAddr, intrBitPos);
            
            status = SystemP_SUCCESS;
#else
            status = IpcNotify_trigInterrupt(selfCoreId, remoteCoreId, mailboxBaseAddr, intrBitPos);
#endif
        }
    }
    return status;
}

static inline void IpcNotify_mailboxClearAllInt(uint32_t mailboxBaseAddr)
{
    volatile uint32_t *addr = (uint32_t *)mailboxBaseAddr;
    *addr = 0x1111111;
}

static inline uint32_t IpcNotify_mailboxGetPendingIntr(uint32_t mailboxBaseAddr)
{
    volatile uint32_t *addr = (uint32_t *)mailboxBaseAddr;

    return *addr;
}

static inline void IpcNotify_mailboxClearPendingIntr(uint32_t mailboxBaseAddr, uint32_t pendingIntr)
{
    volatile uint32_t *addr = (uint32_t *)mailboxBaseAddr;

    *addr = pendingIntr;
}

static inline uint32_t IpcNotify_mailboxIsPendingIntr(uint32_t pendingIntr, uint32_t coreId)
{
    extern uint32_t gIpcNotifyCoreIntrBitPos[];

    uint32_t isPending = 0;
    if(coreId<CSL_CORE_ID_MAX)
    {
        isPending = pendingIntr & ((uint32_t)1 << gIpcNotifyCoreIntrBitPos[coreId]);
    }
    return isPending;
}

#ifdef __cplusplus
}
#endif

#endif /* IPC_NOTIFY_V1_MAILBOX_H_ */

