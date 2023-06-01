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
#define R5FSS0_0_MBOX_WRITE_DONE    (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE)
#define R5FSS0_0_MBOX_READ_REQ      (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ)
#define R5FSS0_1_MBOX_WRITE_DONE    (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE)
#define R5FSS0_1_MBOX_READ_REQ      (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ)
#define R5FSS1_0_MBOX_WRITE_DONE    (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS1_CORE0_MBOX_WRITE_DONE)
#define R5FSS1_0_MBOX_READ_REQ      (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS1_CORE0_MBOX_READ_REQ)
#define R5FSS1_1_MBOX_WRITE_DONE    (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS1_CORE1_MBOX_WRITE_DONE)
#define R5FSS1_1_MBOX_READ_REQ      (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_R5SS1_CORE1_MBOX_READ_REQ)

/* CPU bit positions within the mailbox registers */
#define R5FSS0_0_MBOX_PROC_BIT_POS  ( 0U)
#define R5FSS0_1_MBOX_PROC_BIT_POS  ( 4U)
#define R5FSS1_0_MBOX_PROC_BIT_POS  ( 8U)
#define R5FSS1_1_MBOX_PROC_BIT_POS  (12U)

/* mailbox interrupts */
#define R5FSS0_0_MBOX_READ_REQ_INTR ( 136U)
#define R5FSS0_1_MBOX_READ_REQ_INTR ( 136U)
#define R5FSS1_0_MBOX_READ_REQ_INTR ( 136U)
#define R5FSS1_1_MBOX_READ_REQ_INTR ( 136U)

/* dedicated mailbox memories address and size */
#define MSS_MBOX_MEM                (CSL_MBOX_SRAM_U_BASE)
#define MSS_MBOX_MEM_SIZE           (16U*1024U)

/*
 * SW queue between each pair of CPUs
 *
 * place SW queues at the bottom of the dedicated mailbox memories.
 * Driver assume this memory is init to zero in bootloader as it's ECC protected and
 * needs to be intialized only once and to ensure that only one core has done the
 * mailbox ram initialization before ipc_init. If SBL is not used then Gel does the initialization.
 * We need 4*3 SW Q's for the 4x R5F to send messages to each other, i.e 384 B.
 *
 * Rest of the mailbox memory cna be used for ipc_rpmessage or custom message passing.
 */
#define R5FSS0_0_TO_R5FSS0_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*12U))
#define R5FSS0_0_TO_R5FSS1_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*11U))
#define R5FSS0_0_TO_R5FSS1_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*10U))
#define R5FSS0_1_TO_R5FSS0_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*9U))
#define R5FSS0_1_TO_R5FSS1_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*8U))
#define R5FSS0_1_TO_R5FSS1_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*7U))
#define R5FSS1_0_TO_R5FSS0_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*6U))
#define R5FSS1_0_TO_R5FSS0_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*5U))
#define R5FSS1_0_TO_R5FSS1_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*4U))
#define R5FSS1_1_TO_R5FSS0_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*3U))
#define R5FSS1_1_TO_R5FSS0_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*2U))
#define R5FSS1_1_TO_R5FSS1_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*1U))

/* shift to apply in mailbox addr to get to core specific status */
uint32_t gIpcNotifyCoreIntrBitPos[] =
{
    R5FSS0_0_MBOX_PROC_BIT_POS,
    R5FSS0_1_MBOX_PROC_BIT_POS,
    R5FSS1_0_MBOX_PROC_BIT_POS,
    R5FSS1_1_MBOX_PROC_BIT_POS
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
            .intrBitPos = R5FSS0_0_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
        { /* with R5FSS0_1 */
            .writeDoneMailboxBaseAddr = R5FSS0_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_0_MBOX_READ_REQ,
            .intrBitPos = R5FSS0_1_MBOX_PROC_BIT_POS,
            .swQ = R5FSS0_0_TO_R5FSS0_1_SW_QUEUE,
        },
        { /* with R5FSS1_0 */
            .writeDoneMailboxBaseAddr = R5FSS0_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_0_MBOX_READ_REQ,
            .intrBitPos = R5FSS1_0_MBOX_PROC_BIT_POS,
            .swQ = R5FSS0_0_TO_R5FSS1_0_SW_QUEUE,
        },
        { /* with R5FSS1_1 */
            .writeDoneMailboxBaseAddr = R5FSS0_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_0_MBOX_READ_REQ,
            .intrBitPos = R5FSS1_1_MBOX_PROC_BIT_POS,
            .swQ = R5FSS0_0_TO_R5FSS1_1_SW_QUEUE,
        },
    },
    /* R5FSS0-1 */
    {
        { /* with R5FSS0_0 */
            .writeDoneMailboxBaseAddr = R5FSS0_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_1_MBOX_READ_REQ,
            .intrBitPos = R5FSS0_0_MBOX_PROC_BIT_POS,
            .swQ = R5FSS0_1_TO_R5FSS0_0_SW_QUEUE,
        },
        { /* with R5FSS0_1 */
            .writeDoneMailboxBaseAddr = R5FSS0_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_1_MBOX_READ_REQ,
            .intrBitPos = R5FSS0_1_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
        { /* with R5FSS1_0 */
            .writeDoneMailboxBaseAddr = R5FSS0_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_1_MBOX_READ_REQ,
            .intrBitPos = R5FSS1_0_MBOX_PROC_BIT_POS,
            .swQ = R5FSS0_1_TO_R5FSS1_0_SW_QUEUE,
        },
        { /* with R5FSS1_1 */
            .writeDoneMailboxBaseAddr = R5FSS0_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS0_1_MBOX_READ_REQ,
            .intrBitPos = R5FSS1_1_MBOX_PROC_BIT_POS,
            .swQ = R5FSS0_1_TO_R5FSS1_1_SW_QUEUE,
        },
    },
        /* R5FSS1-0 */
    {
        { /* with R5FSS0_0 */
            .writeDoneMailboxBaseAddr = R5FSS1_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS1_0_MBOX_READ_REQ,
            .intrBitPos = R5FSS0_0_MBOX_PROC_BIT_POS,
            .swQ = R5FSS1_0_TO_R5FSS0_0_SW_QUEUE,
        },
        { /* with R5FSS0_1 */
            .writeDoneMailboxBaseAddr = R5FSS1_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS1_0_MBOX_READ_REQ,
            .intrBitPos = R5FSS0_1_MBOX_PROC_BIT_POS,
            .swQ = R5FSS1_0_TO_R5FSS0_1_SW_QUEUE,
        },
        { /* with R5FSS1_0 */
            .writeDoneMailboxBaseAddr = R5FSS1_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS1_0_MBOX_READ_REQ,
            .intrBitPos = R5FSS1_0_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
        { /* with R5FSS1_1 */
            .writeDoneMailboxBaseAddr = R5FSS1_0_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS1_0_MBOX_READ_REQ,
            .intrBitPos = R5FSS1_1_MBOX_PROC_BIT_POS,
            .swQ = R5FSS1_0_TO_R5FSS1_1_SW_QUEUE,
        },
    },
        /* R5FSS1-1 */
    {
        { /* with R5FSS0_0 */
            .writeDoneMailboxBaseAddr = R5FSS1_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS1_1_MBOX_READ_REQ,
            .intrBitPos = R5FSS0_0_MBOX_PROC_BIT_POS,
            .swQ = R5FSS1_1_TO_R5FSS0_0_SW_QUEUE,
        },
        { /* with R5FSS0_1 */
            .writeDoneMailboxBaseAddr = R5FSS1_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS1_1_MBOX_READ_REQ,
            .intrBitPos = R5FSS0_1_MBOX_PROC_BIT_POS,
            .swQ = R5FSS1_1_TO_R5FSS0_1_SW_QUEUE,
        },
        { /* with R5FSS1_0 */
            .writeDoneMailboxBaseAddr = R5FSS1_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS1_1_MBOX_READ_REQ,
            .intrBitPos = R5FSS1_0_MBOX_PROC_BIT_POS,
            .swQ = R5FSS1_1_TO_R5FSS1_0_SW_QUEUE,
        },
        { /* with R5FSS1_1 */
            .writeDoneMailboxBaseAddr = R5FSS1_1_MBOX_WRITE_DONE,
            .readReqMailboxBaseAddr = R5FSS1_1_MBOX_READ_REQ,
            .intrBitPos = R5FSS1_1_MBOX_PROC_BIT_POS,
            .swQ = NULL,
        },
    },
};

/* Interrupt config for R5FSS0-0 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM   (1u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_0[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM] = {
    {
        .intNum = R5FSS0_0_MBOX_READ_REQ_INTR,   /* interrupt line on R5FSS0-0 */
        .eventId = 0U,   /* not used */
        .numCores = 3U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_1,
            CSL_CORE_ID_R5FSS1_0,
            CSL_CORE_ID_R5FSS1_1,
            CSL_CORE_ID_MAX,
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss0_0 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM;

/* Interrupt config for R5FSS0-1 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM   (1u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_1[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM] = {
    {
        .intNum = R5FSS0_1_MBOX_READ_REQ_INTR,   /* interrupt line on R5FSS0-1 */
        .eventId = 0U,   /* not used */
        .numCores = 3U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS1_0,
            CSL_CORE_ID_R5FSS1_1,
            CSL_CORE_ID_MAX,
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss0_1 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM;

/* Interrupt config for R5FSS1-0 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_0_NUM   (1u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss1_0[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_0_NUM] = {
    {
        .intNum = R5FSS1_0_MBOX_READ_REQ_INTR,   /* interrupt line on R5FSS1-0 */
        .eventId = 0U,   /* not used */
        .numCores = 3U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS0_1,
            CSL_CORE_ID_R5FSS1_1,
            CSL_CORE_ID_MAX,
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss1_0 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_0_NUM;

/* Interrupt config for R5FSS1-1 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_1_NUM   (1u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss1_1[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_1_NUM] = {
    {
        .intNum = R5FSS1_1_MBOX_READ_REQ_INTR,   /* interrupt line on R5FSS1-1 */
        .eventId = 0U,   /* not used */
        .numCores = 3U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS0_1,
            CSL_CORE_ID_R5FSS1_0,
            CSL_CORE_ID_MAX,
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss1_1 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_1_NUM;

void IpcNotify_trigInterrupt(uint32_t mailboxBaseAddr, uint32_t intrBitPos)
{
    volatile uint32_t *addr = (uint32_t *)mailboxBaseAddr;

    /* trigger interrupt to other core */
    *addr = (1U << intrBitPos);
}

void IpcNotify_wait(void)
{
}
