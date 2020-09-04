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

#include <drivers/ipc_notify/v0/ipc_notify_v0.h>

/* values to use when a mailbox config is not used in gIpcNotifyMailboxConfig */
#define MAILBOX_UNUSED      0U, 0U, 0U

/* All mailbox base addresses */
#define IPC_NOTIFY_MAILBOX_MAX_INSTANCES    (8U)
uint32_t gIpcNotifyMailboxBaseAddr[IPC_NOTIFY_MAILBOX_MAX_INSTANCES+1] = {
    0x29000000U,
    0x29010000U,
    0x29020000U,
    0x29030000U,
    0x29040000U,
    0x29050000U,
    0x29060000U,
    0x29070000U,
    0x0, /* MUST be terminated by 0x0 */
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
    /* from M4FSS */
    {
        { /* to M4FSS */
            MAILBOX_UNUSED
        },
        { /* to R5FSS0-0 */
            0U, 0U, 0U
        },
        { /* to R5FSS0-1 */
            0U, 4U, 1U
        },
        { /* to R5FSS1-0 */
            0U, 8U, 2U
        },
        { /* to R5FSS1-1 */
            0U, 12U, 3U
        },
        { /* to A53SS0_0 */
            6U, 0U, 2U
        },
        { /* to A53SS0_1 */
            MAILBOX_UNUSED
        },
    },
    /* from R5FSS0-0 */
    {
        { /* to M4FSS */
            6U, 2U, 3U
        },
        { /* to R5FSS0-0 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS0-1 */
            0U, 5U, 1U
        },
        { /* to R5FSS1-0 */
            0U, 9U, 2U
        },
        { /* to R5FSS1-1 */
            0U, 13U, 3U
        },
        { /* to A53SS0_0 */
            2U, 0U, 2U
        },
        { /* to A53SS0_1 */
            MAILBOX_UNUSED
        },
    },
    /* from R5FSS0-1 */
    {
        { /* to M4FSS */
            6U, 3U, 3U
        },
        { /* to R5FSS0-0 */
            0U, 1U, 0U
        },
        { /* to R5FSS0-1 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS1-0 */
            0U, 10U, 2U
        },
        { /* to R5FSS1-1 */
            0U, 14U, 3U
        },
        { /* to A53SS0_0 */
            2U, 2U, 2U
        },
        { /* to A53SS0_1 */
            MAILBOX_UNUSED
        },
    },
    /* from R5FSS1-0 */
    {
        { /* to M4FSS */
            6U, 4U, 3U
        },
        { /* to R5FSS0-0 */
            0U, 2U, 0U
        },
        { /* to R5FSS0-1 */
            0U, 6U, 1U
        },
        { /* to R5FSS1-0 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS1-1 */
            0U, 15U, 3U
        },
        { /* to A53SS0_0 */
            4U, 0U, 2U
        },
        { /* to A53SS0_1 */
            MAILBOX_UNUSED
        },
    },
    /* from R5FSS1-1 */
    {
        { /* to M4FSS */
            6U, 5U, 3U
        },
        { /* to R5FSS0-0 */
            0U, 3U, 0U
        },
        { /* to R5FSS0-1 */
            0U, 7U, 1U
        },
        { /* to R5FSS1-0 */
            0U, 11U, 2U
        },
        { /* to R5FSS1-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0_0 */
            4U, 2U, 2U
        },
        { /* to A53SS0_1 */
            MAILBOX_UNUSED
        },
    },
    /* from A53SS0_0 */
    {
        { /* to M4FSS */
            6U, 1U, 3U
        },
        { /* to R5FSS0-0 */
            2U, 1U, 0U
        },
        { /* to R5FSS0-1 */
            2U, 3U, 1U
        },
        { /* to R5FSS1-0 */
            4U, 1U, 0U
        },
        { /* to R5FSS1-1 */
            4U, 3U, 1U
        },
        { /* to A53SS0_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0_1 */
            MAILBOX_UNUSED
        },
    },
    /* from A53SS0_1 */
    {
        { /* to M4FSS */
            MAILBOX_UNUSED
        },
        { /* to R5FSS0-0 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS0-1 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS1-0 */
            MAILBOX_UNUSED
        },
        { /* to R5FSS1-1 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0_0 */
            MAILBOX_UNUSED
        },
        { /* to A53SS0_1 */
            MAILBOX_UNUSED
        },
    },
};

/* Interrupt config for M4FSS */
#define IPC_NOFTIY_INTERRUPT_CONFIG_M4FSS_NUM   (1u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_m4fss0_0[IPC_NOFTIY_INTERRUPT_CONFIG_M4FSS_NUM] = {
    {
        .intNum = 16U + 56U,   /* interrupt line on M4FSS CPU, +16 offset to account for M4 internal interrupts */
        .eventId = 0U,   /* not used */
        .numCores = 5U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS0_1,
            CSL_CORE_ID_R5FSS1_0,
            CSL_CORE_ID_R5FSS1_1,
            CSL_CORE_ID_A53SS0_0,
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_m4fss0_0 = IPC_NOFTIY_INTERRUPT_CONFIG_M4FSS_NUM;

/* Interrupt config for R5FSS0_0 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM   (2u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_0[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM] = {
    {
        .intNum = 96U,   /* interrupt line on R5FSS0_0 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 4U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_1,
            CSL_CORE_ID_R5FSS1_0,
            CSL_CORE_ID_R5FSS1_1,
            CSL_CORE_ID_M4FSS0_0,
        },
    },
    {
        .intNum = 98U,   /* interrupt line on R5FSS0_0 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 1U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_A53SS0_0,
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss0_0 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_0_NUM;

/* Interrupt config for R5FSS0_1 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM   (2u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_1[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM] = {
    {
        .intNum = 96U,   /* interrupt line on R5FSS0_1 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 4U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS1_0,
            CSL_CORE_ID_R5FSS1_1,
            CSL_CORE_ID_M4FSS0_0,
        },
    },
    {
        .intNum = 98U,   /* interrupt line on R5FSS0_0 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 1U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_A53SS0_0,
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss0_1 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS0_1_NUM;

/* Interrupt config for R5FSS1_0 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_0_NUM   (2u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss1_0[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_0_NUM] = {
    {
        .intNum = 96U,   /* interrupt line on R5FSS1_0 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 4U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS0_1,
            CSL_CORE_ID_R5FSS1_1,
            CSL_CORE_ID_M4FSS0_0,
        },
    },
    {
        .intNum = 98U,   /* interrupt line on R5FSS0_0 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 1U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_A53SS0_0,
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss1_0 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_0_NUM;

/* Interrupt config for R5FSS1_1 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_1_NUM   (2u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss1_1[IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_1_NUM] = {
    {
        .intNum = 96U,   /* interrupt line on R5FSS1_1 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 4U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS0_1,
            CSL_CORE_ID_R5FSS1_0,
            CSL_CORE_ID_M4FSS0_0,
        },
    },
    {
        .intNum = 98U,   /* interrupt line on R5FSS0_0 CPU */
        .eventId = 0U,   /* not used */
        .numCores = 1U,  /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = { /* core ID's tied to this interrupt line */
            CSL_CORE_ID_A53SS0_0,
        },
    }
};
uint32_t gIpcNotifyInterruptConfigNum_r5fss1_1 = IPC_NOFTIY_INTERRUPT_CONFIG_R5FSS1_1_NUM;

/* Interrupt config for A53SS0_0 */
#define IPC_NOFTIY_INTERRUPT_CONFIG_A53SS0_0_NUM   (3u)
IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_a53ss0_0[IPC_NOFTIY_INTERRUPT_CONFIG_A53SS0_0_NUM] = {
    {
        .intNum = 108U,   /* interrupt line on A53SS0_0 CPU */
        .eventId = 0U,    /* not used */
        .numCores = 1U,   /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = {   /* core ID's tied to this interrupt line */
            CSL_CORE_ID_M4FSS0_0
        },
    },
    {
        .intNum = 112U,   /* interrupt line on A53SS0_0 CPU */
        .eventId = 0U,    /* not used */
        .numCores = 2U,   /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = {   /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS0_0,
            CSL_CORE_ID_R5FSS0_1
        },
    },
    {
        .intNum = 116U,   /* interrupt line on A53SS0_0 CPU */
        .eventId = 0U,    /* not used */
        .numCores = 2U,   /* number of cores that send messages which tied to this interrupt line */
        .coreIdList = {   /* core ID's tied to this interrupt line */
            CSL_CORE_ID_R5FSS1_0,
            CSL_CORE_ID_R5FSS1_1
        },
    },
};
uint32_t gIpcNotifyInterruptConfigNum_a53ss0_0 = IPC_NOFTIY_INTERRUPT_CONFIG_A53SS0_0_NUM;