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

/* mailbox registers */
#define R5FSS0_0_MBOX_WRITE_DONE    (CSL_MSS_CTRL_U_BASE + 0x5FCU)
#define R5FSS0_0_MBOX_READ_REQ      (CSL_MSS_CTRL_U_BASE + 0x600U)
#define R5FSS0_0_MBOX_READ_DONE     (CSL_MSS_CTRL_U_BASE + 0x604U)
#define R5FSS0_0_MBOX_READ_DONE_ACK (CSL_MSS_CTRL_U_BASE + 0xFF0U)
#define R5FSS0_1_MBOX_WRITE_DONE    (CSL_MSS_CTRL_U_BASE + 0x608U)
#define R5FSS0_1_MBOX_READ_REQ      (CSL_MSS_CTRL_U_BASE + 0x60CU)
#define R5FSS0_1_MBOX_READ_DONE     (CSL_MSS_CTRL_U_BASE + 0x610U)
#define R5FSS0_1_MBOX_READ_DONE_ACK (CSL_MSS_CTRL_U_BASE + 0xFF0U)
#define C66SS0_MBOX_WRITE_DONE      (CSL_DSS_CTRL_U_BASE + 0x56CU)
#define C66SS0_MBOX_READ_REQ        (CSL_DSS_CTRL_U_BASE + 0x570U)
#define C66SS0_MBOX_READ_DONE       (CSL_DSS_CTRL_U_BASE + 0x574U)
#define C66SS0_MBOX_READ_DONE_ACK   (CSL_DSS_CTRL_U_BASE + 0xFF0U)

/* CPU bit positions within the mailbox registers */
#define R5FSS0_0_MBOX_PROC_BIT_POS  ( 0U)
#define R5FSS0_1_MBOX_PROC_BIT_POS  ( 4U)
#define C66SS0_MBOX_PROC_BIT_POS    (16U)

/* mailbox interrupts */
#define R5FSS0_0_MBOX_READ_REQ_INTR ( 77U)
#define R5FSS0_0_MBOX_READ_ACK_INTR ( 78U)
#define R5FSS0_1_MBOX_READ_REQ_INTR ( 79U)
#define R5FSS0_1_MBOX_READ_ACK_INTR ( 80U)
#define C66SS0_MBOX_READ_REQ_INTR   ( 94U)
#define C66SS0_MBOX_READ_ACK_INTR   ( 95U)

/* A delay of 60-70 clock cycles is recommended before clear pending read request from remote core
 * This delay is implemented as a loop and is profiled to be approximately 80 clock cycles
 */
 #define IPC_NOTIFY_WAIT_CYCLES           (5U)

 /* A counter that restricts the loop to pend in isr forever and also avoid any race around condtion between the cores */
 #define IPC_NOTIFY_LOOP_COUNTER_MAX           (1000U)

/* shift to apply in mailbox addr to get to core specific status */
uint32_t gIpcNotifyCoreIntrBitPos[] =
{
    R5FSS0_0_MBOX_PROC_BIT_POS,
    R5FSS0_1_MBOX_PROC_BIT_POS,
    C66SS0_MBOX_PROC_BIT_POS,
};

/* Pre-defined mailbox config to allow any CPU to send and receive messages from any CPU on this SOC
 *
 * These assignments need to be in sync with gIpcNotifyInterruptConfig* done later below,
 *
 * This is a 2D array
 * - 1st indexed by self core ID
 * - then indexed by remote core ID
 */
IpcNotify_MailboxConfig gIpcNotifyMailboxConfig[CSL_CORE_ID_MAX][CSL_CORE_ID_MAX] =
{
    /* R5FSS0-0 */
    {
        { /* with R5FSS0_0 */
            .writeDoneMailboxBaseAddr = R5FSS0_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_0_MBOX_READ_REQ,
            .readDoneMailboxBaseAddr = R5FSS0_0_MBOX_READ_DONE,
            .readDoneAckMailboxBaseAddr = R5FSS0_0_MBOX_READ_DONE_ACK,
            .intrBitPos = R5FSS0_0_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
        { /* with R5FSS0_1 */
            .writeDoneMailboxBaseAddr = R5FSS0_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_0_MBOX_READ_REQ,
            .readDoneMailboxBaseAddr = R5FSS0_0_MBOX_READ_DONE,
            .readDoneAckMailboxBaseAddr = R5FSS0_0_MBOX_READ_DONE_ACK,
            .intrBitPos = R5FSS0_1_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
        { /* with C66SS0 */
            .writeDoneMailboxBaseAddr = R5FSS0_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_0_MBOX_READ_REQ,
            .readDoneMailboxBaseAddr = R5FSS0_0_MBOX_READ_DONE,
            .readDoneAckMailboxBaseAddr = R5FSS0_0_MBOX_READ_DONE_ACK,
            .intrBitPos = C66SS0_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
    },
    /* R5FSS0-1 */
    {
        { /* with R5FSS0_0 */
            .writeDoneMailboxBaseAddr = R5FSS0_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_1_MBOX_READ_REQ,
            .readDoneMailboxBaseAddr = R5FSS0_1_MBOX_READ_DONE,
            .readDoneAckMailboxBaseAddr = R5FSS0_1_MBOX_READ_DONE_ACK,
            .intrBitPos = R5FSS0_0_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
        { /* with R5FSS0_1 */
            .writeDoneMailboxBaseAddr = R5FSS0_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_1_MBOX_READ_REQ,
            .readDoneMailboxBaseAddr = R5FSS0_1_MBOX_READ_DONE,
            .readDoneAckMailboxBaseAddr = R5FSS0_1_MBOX_READ_DONE_ACK,
            .intrBitPos = R5FSS0_1_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
        { /* with C66SS0 */
            .writeDoneMailboxBaseAddr = R5FSS0_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_1_MBOX_READ_REQ,
            .readDoneMailboxBaseAddr = R5FSS0_1_MBOX_READ_DONE,
            .readDoneAckMailboxBaseAddr = R5FSS0_1_MBOX_READ_DONE_ACK,
            .intrBitPos = C66SS0_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
    },
    /* C66SS0 */
    {
        { /* with R5FSS0_0 */
            .writeDoneMailboxBaseAddr = C66SS0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = C66SS0_MBOX_READ_REQ,
            .readDoneMailboxBaseAddr = C66SS0_MBOX_READ_DONE,
            .readDoneAckMailboxBaseAddr = C66SS0_MBOX_READ_DONE_ACK,
            .intrBitPos = R5FSS0_0_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
        { /* with R5FSS0_1 */
            .writeDoneMailboxBaseAddr = C66SS0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = C66SS0_MBOX_READ_REQ,
            .readDoneMailboxBaseAddr = C66SS0_MBOX_READ_DONE,
            .readDoneAckMailboxBaseAddr = C66SS0_MBOX_READ_DONE_ACK,
            .intrBitPos = R5FSS0_1_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
        { /* with C66SS0 */
            .writeDoneMailboxBaseAddr = C66SS0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = C66SS0_MBOX_READ_REQ,
            .readDoneMailboxBaseAddr = C66SS0_MBOX_READ_DONE,
            .readDoneAckMailboxBaseAddr = C66SS0_MBOX_READ_DONE_ACK,
            .intrBitPos = C66SS0_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
    },
};

extern void IpcNotify_isr(void *args);
extern void IpcNotify_readAckCallback(void *args);

/* Interrupt config for R5FSS0-0 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM   (2u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_0[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM] = {
    {
        .intNum = R5FSS0_0_MBOX_READ_REQ_INTR,   /* interrupt line on R5FSS0-0 */
        .eventId = 0U,   /* not used */
        .numCores = 2U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_1,
            CSL_CORE_ID_C66SS0,
			CSL_CORE_ID_MAX,
			CSL_CORE_ID_MAX,
        },
	.clearIntOnInit = 0,
    .callback = IpcNotify_isr
    },
    {
        .intNum = R5FSS0_0_MBOX_READ_ACK_INTR,   /* interrupt line on R5FSS0-0 */
        .eventId = 0U,   /* not used */
        .numCores = 2U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_1,
            CSL_CORE_ID_C66SS0,
			CSL_CORE_ID_MAX,
			CSL_CORE_ID_MAX,
        },
	.clearIntOnInit = 0,
    .callback = IpcNotify_readAckCallback
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss0_0 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM;

/* Interrupt config for R5FSS0-1 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM   (2u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_1[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM] = {
    {
        .intNum = R5FSS0_1_MBOX_READ_REQ_INTR,   /* interrupt line on R5FSS0-1 */
        .eventId = 0U,   /* not used */
        .numCores = 2U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_C66SS0,
		    CSL_CORE_ID_MAX,
			CSL_CORE_ID_MAX,
        },
	.clearIntOnInit = 0,
    .callback = IpcNotify_isr
    },
    {
        .intNum = R5FSS0_1_MBOX_READ_ACK_INTR,   /* interrupt line on R5FSS0-1 */
        .eventId = 0U,   /* not used */
        .numCores = 2U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_C66SS0,
		    CSL_CORE_ID_MAX,
			CSL_CORE_ID_MAX,
        },
	.clearIntOnInit = 0,
    .callback = IpcNotify_readAckCallback
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss0_1 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM;

/* Interrupt config for C66SS0 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_C66SS0_NUM   (2u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_c66ss0[IPC_NOFTIY_INTERRUPT_CONFIG_C66SS0_NUM] = {
    {
        .intNum = C66SS0_MBOX_READ_REQ_INTR,   /* interrupt line on R5FSS0-1 */
        .eventId = 0U,   /* not used */
        .numCores = 2U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS0_1,
			CSL_CORE_ID_MAX,
			CSL_CORE_ID_MAX,
        },
	.clearIntOnInit = 1,
    .callback = IpcNotify_isr
    },
    {
        .intNum = C66SS0_MBOX_READ_ACK_INTR,   /* interrupt line on R5FSS0-1 */
        .eventId = 0U,   /* not used */
        .numCores = 2U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS0_1,
			CSL_CORE_ID_MAX,
			CSL_CORE_ID_MAX,
        },
	.clearIntOnInit = 1,
    .callback = IpcNotify_readAckCallback
    }
};
uint32_t gIpcNotifyInterruptConfigNum_c66ss0 = IPC_NOFTIY_INTERRUPT_CONFIG_C66SS0_NUM;

void IpcNotify_trigInterrupt(uint32_t selfCoreId, uint32_t remoteCoreId, uint32_t mailboxBaseAddr, uint32_t intrBitPos)
 {
    uint32_t pendingIntr, counter = 0;
    volatile uint32_t *addr = (uint32_t *)mailboxBaseAddr;
    /* Keep polling for READ_REQ register bit of Receiver */
    IpcNotify_MailboxConfig *pReceiverMailboxConfig;
    pReceiverMailboxConfig = &gIpcNotifyMailboxConfig[remoteCoreId][selfCoreId];
    do
    {
        counter++;
        /* trigger interrupt to other core */
        *addr = ((uint32_t)1U << intrBitPos);
        pendingIntr = IpcNotify_mailboxGetPendingIntr(pReceiverMailboxConfig->readReqMailboxBaseAddr);
        pendingIntr = (pendingIntr >> (pReceiverMailboxConfig->intrBitPos)) & (0x1U); /* Get the READ_REQ reg. value w.r.t Core bit pos. */
    }while ((pendingIntr!=1U) && counter < IPC_NOTIFY_LOOP_COUNTER_MAX);
}

void IpcNotify_trigInterrupt_ack(uint32_t selfCoreId, uint32_t remoteCoreId, uint32_t mailboxBaseAddr, uint32_t intrBitPos)
{
    uint32_t pendingIntr, counter = 0;
    volatile uint32_t *addr = (uint32_t *)mailboxBaseAddr;
    /* Keep polling for READ_DONE register bit of Sender */
    IpcNotify_MailboxConfig *pSenderMailboxConfig;
    pSenderMailboxConfig = &gIpcNotifyMailboxConfig[remoteCoreId][selfCoreId];
    do
    {
        counter++;
        /* trigger interrupt to other core */
        if(selfCoreId == CSL_CORE_ID_R5FSS0_1){
            *addr = (((uint32_t)1U << (intrBitPos/4)) << 8);
        }
        else{
            *addr = ((uint32_t)1U << (intrBitPos/4));
        }
        pendingIntr = IpcNotify_mailboxGetPendingIntr(pSenderMailboxConfig->readDoneMailboxBaseAddr);
        pendingIntr = (pendingIntr >> (pSenderMailboxConfig->intrBitPos)) & (0x1U); /* Get the READ_DONE reg. value w.r.t Core bit pos. */ 
    }while ((pendingIntr!=1U) && counter < IPC_NOTIFY_LOOP_COUNTER_MAX);
 }


void IpcNotify_wait(void)
{
    volatile uint32_t loopCounter = 0U;

    /* Processor sending will trigger read request multiple times and ensure
    * that read request is reached to receiving processor. The delay implemented
    * here is not to clear the interrupt while sending processor is reading back
    * and verifying the interrupt is triggered at receving Processor
    */
    for(loopCounter = 0; loopCounter < IPC_NOTIFY_WAIT_CYCLES; loopCounter+=1U)
	{
		;
	}
    return;
}
