/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#ifndef IPC_NOTIFY_V0_MAILBOX_H_
#define IPC_NOTIFY_V0_MAILBOX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* this file has define's and inline function's to program the HW mailbox registers */

/* max limits for HW mailbox's */
#define MAILBOX_MAX_MSGS_IN_FIFO                ( 8u)
#define MAILBOX_MAX_FIFO                        (16u)
#define MAILBOX_MAX_USER                        ( 4u)

/* HW mailbox register address, parameterized via base addr, hw fifo num, user id that is used */
#define MAILBOX_MESSAGE(base, fifo)             (volatile uint32_t*)((base) + 0x040u + 0x04u*((fifo) & (MAILBOX_MAX_FIFO-1)))
#define MAILBOX_FIFO_STATUS(base, fifo)         (volatile uint32_t*)((base) + 0x080u + 0x04u*((fifo) & (MAILBOX_MAX_FIFO-1)))
#define MAILBOX_MSG_STATUS(base, fifo)          (volatile uint32_t*)((base) + 0x0C0u + 0x04u*((fifo) & (MAILBOX_MAX_FIFO-1)))    
#define MAILBOX_CLEAR_INT(base, user)           (volatile uint32_t*)((base) + 0x104u + 0x10u*((user) & (MAILBOX_MAX_USER-1)))
#define MAILBOX_ENABLE_INT(base, user)          (volatile uint32_t*)((base) + 0x108u + 0x10u*((user) & (MAILBOX_MAX_USER-1)))
#define MAILBOX_DISABLE_INT(base, user)         (volatile uint32_t*)((base) + 0x10Cu + 0x10u*((user) & (MAILBOX_MAX_USER-1)))
#define MAILBOX_EOI_INT(base)                   (volatile uint32_t*)((base) + 0x140u)

/* value to construct to enable/disable/clear mew message interrupt for a given HW fifo */
#define MAILBOX_NEW_MSG_INT(fifo)               (uint32_t)(1 << (((fifo) & (MAILBOX_MAX_FIFO-1))*2))

/* return number of messages pending in the HW fifo to be read within a mailbox */
static inline uint32_t IpcNotify_mailboxGetNumMsg(uint32_t mailboxBaseAddr, uint32_t hwFifoNum)
{
    volatile uint32_t *addr = MAILBOX_MSG_STATUS(mailboxBaseAddr, hwFifoNum);
    return *addr & (MAILBOX_MAX_MSGS_IN_FIFO-1);
}

/* check if HW fifo is full within a mailbox */
static inline uint32_t IpcNotify_mailboxIsFull(uint32_t mailboxBaseAddr, uint32_t hwFifoNum)
{
    volatile uint32_t *addr = MAILBOX_FIFO_STATUS(mailboxBaseAddr, hwFifoNum);
    return *addr & 0x1;
}

/* read from HW fifo within a mailbox  */
static inline uint32_t IpcNotify_mailboxRead(uint32_t mailboxBaseAddr, uint32_t hwFifoNum)
{
    volatile uint32_t *addr = MAILBOX_MESSAGE(mailboxBaseAddr, hwFifoNum);
    return *addr;
}

/* write to HW fifo within a mailbox */
static inline void IpcNotify_mailboxWrite(uint32_t mailboxBaseAddr, uint32_t hwFifoNum, uint32_t value)
{
    volatile uint32_t *addr = MAILBOX_MESSAGE(mailboxBaseAddr, hwFifoNum);
    *addr = value;
}

/* clear new message interrupt for a  HW fifo within a mailbox */
static inline void IpcNotify_mailboxClearInt(uint32_t mailboxBaseAddr, uint32_t hwFifoNum, uint32_t userId)
{
    volatile uint32_t *addr = MAILBOX_CLEAR_INT(mailboxBaseAddr, userId);
    *addr = MAILBOX_NEW_MSG_INT(hwFifoNum);
}

/* enable new message interrupt for a HW fifo within a mailbox */
static inline void IpcNotify_mailboxEnableInt(uint32_t mailboxBaseAddr, uint32_t hwFifoNum, uint32_t userId)
{
    volatile uint32_t *addr = MAILBOX_ENABLE_INT(mailboxBaseAddr, userId);
    *addr = MAILBOX_NEW_MSG_INT(hwFifoNum);
}

/* disable new message interrupt for a HW fifo within a mailbox */
static inline void IpcNotify_mailboxDisableInt(uint32_t mailboxBaseAddr, uint32_t hwFifoNum, uint32_t userId)
{
    volatile uint32_t *addr = MAILBOX_DISABLE_INT(mailboxBaseAddr, userId);
    *addr = MAILBOX_NEW_MSG_INT(hwFifoNum);
}

#ifdef __cplusplus
}
#endif

#endif /* IPC_NOTIFY_V0_MAILBOX_H_ */

