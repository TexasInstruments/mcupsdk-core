/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions arootnd the following disclaimer.
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

#include <drivers/ipc_notify/v0/ipc_notify_v0.h>
#include <drivers/ipc_notify/v0/soc/am65x/ipc_notify_soc.h>

/* values to use when a mailbox config is not used in gIpcNotifyMailboxConfig */
#define MAILBOX_UNUSED      0U, 0U, 0U

/* All mailbox base addresses */
uint32_t gIpcNotifyMailboxBaseAddr[IPC_NOTIFY_MAILBOX_MAX_INSTANCES+1] = {
    (0x31F80000U),
    (0x31F81000U),
    (0x31F82000U),
    (0x31F83000U),
    (0x31F84000U),
    (0x31F85000U),
    (0x31F86000U),
    (0x31F87000U),
    (0x31F88000U),
    (0x31F89000U),
    (0x31F8A000U),
    (0x31F8B000U),
    (0x0), /* MUST be terminated by 0x0 */
};

/* Pre-defined mailbox config to allow any CPU to send and receive messages from any CPU on this SOC
 *
 * Each element consists of [maibox base addr, hw fifo, user id]
 * These assignments need to be in sync with gIpcNotifyInterruptConfig* done later below,
 *
 * This is a 2D array
 * - 1st indexed by source core ID
 * - then indexed by destination core ID
 */
IpcNotify_MailboxConfig gIpcNotifyMailboxConfig[CSL_CORE_ID_MAX][CSL_CORE_ID_MAX] =
{
    /* from R5FSS0-0 */
    {
        { /* to R5FSS0-0 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS0-1 */
            2U, 1U, 1U
        },
        { /* to A53SS0_0 */
            0U, 0U, 1U
        },
        { /* to A53SS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1-1 */
            MAILBOX_UNUSED
        },
    },
    /* from R5FSS0-1 */
    {
        { /* to R5FSS0-0 */
            2U, 0U, 0U
        },
        { /* to R5FSS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0_0 */
            1U, 0U, 1U
        },
        { /* to A53SS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1-1 */
            MAILBOX_UNUSED
        },
    },
    /* from A53SS0_0 */
    {
        { /* to R5FSS0-0 */
            0U, 1U, 1U
        },
        { /* to R5FSS0-1 */
            1U, 1U, 1U
        },
        { /* to A53SS0_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1-1 */
            MAILBOX_UNUSED
        },
    },
    /* from A53SS0_1 */
    {
        { /* to R5FSS0-0 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1-1 */
            MAILBOX_UNUSED
        },
    },
    /* from A53SS1_0 */
    {
        { /* to R5FSS0-0 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1-1 */
            MAILBOX_UNUSED
        },
    },
    /* from A53SS1_1 */
    {
        { /* to R5FSS0-0 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS1-1 */
            MAILBOX_UNUSED
        },
    },
};

/* Interrupt config for R5FSS0_0 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM   (2u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_0[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM] = {
    {
        .intNum = 187U,   /* interrupt line on R5FSS0_0 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 1U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_A53SS0_0,
        },
    },
    {
        .intNum = 188,   /* interrupt line on R5FSS0_0 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 1U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_1,
        },
    },
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss0_0 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM;

/* Interrupt config for R5FSS0_1 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM   (2u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_1[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM] = {
    {
        .intNum = 219,   /* interrupt line on R5FSS0_1 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 1U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_A53SS0_0,
        },
    },
    {
        .intNum = 220,   /* interrupt line on R5FSS0_1 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 1U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
        },
    },
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss0_1 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM;

/* Interrupt config for A53SS0_0 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_A53SS0_0_NUM   (1u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_a53ss0_0[IPC_NOFTIY_INTERRUPT_CONFIG_A53SS0_0_NUM] = {
    {
        .intNum = 189U,   /* interrupt line on A53SS0_0 CPU */
        .eventId = 0U,    /* not used */
        .numCores = 2U,   /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = {   /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS0_1
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_a53ss0_0 = IPC_NOFTIY_INTERRUPT_CONFIG_A53SS0_0_NUM;
