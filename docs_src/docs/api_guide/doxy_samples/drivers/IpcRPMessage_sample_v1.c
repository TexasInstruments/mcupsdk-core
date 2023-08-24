
//! [include]
#include <stdio.h>
#include <string.h>
#include <drivers/ipc_notify.h>
#include <drivers/ipc_notify/v1/ipc_notify_v1.h>
#include <drivers/ipc_rpmsg.h>
#include <kernel/dpl/DebugP.h>
//! [include]

//! [shared mem]

/* Below code is for reference, recommened to use SysCfg to generate this code */

/* Below is common code for all the CPUs participating in this IPC */

#define IPC_NOTIFY_NUM_CORES              (2U)
#define IPC_NOTIFY_MEMORY_SIZE            (((IPC_NOTIFY_NUM_CORES) * (IPC_NOTIFY_NUM_CORES - 1)) * (MAILBOX_MAX_SW_QUEUE_SIZE))
#define IPC_RPMESSAGE_NUM_CORES           (2U)
#define IPC_RPMESSAGE_NUM_VRINGS          (IPC_RPMESSAGE_NUM_CORES * (IPC_RPMESSAGE_NUM_CORES - 1))
#define IPC_RPMESSAGE_NUM_VRING_BUF       (8U)
#define IPC_RPMESSAGE_MAX_VRING_BUF_SIZE  (128U)
#define IPC_RPMESSAGE_VRING_SIZE          (RPMESSAGE_VRING_SIZE(IPC_RPMESSAGE_NUM_VRING_BUF, IPC_RPMESSAGE_MAX_VRING_BUF_SIZE))
/* Total memory size required for RPMSG VRINGS */
#define IPC_RPMESSAGE_MEMORY_SIZE          (IPC_RPMESSAGE_VRING_SIZE * IPC_RPMESSAGE_NUM_VRINGS)

/* Total Shared memory size used for IPC */
#define IPC_SHARED_MEM_SIZE               (IPC_NOTIFY_MEMORY_SIZE + IPC_RPMESSAGE_MEMORY_SIZE)

/* Shared memory used for IPC
 *
 * IMPORTANT: Make sure of below,
 * - The section defined below should be placed at the exact same location in memory for all the CPUs
 * - The memory should be marked as non-cached for all the CPUs
 * - The section should be marked as NOLOAD in all the CPUs linker command file
 */
uint8_t gIpcSharedMem[IPC_SHARED_MEM_SIZE] __attribute__((aligned(128), section(".bss.ipc_shared_mem")));

/*
 * Driver assume this memory is init to zero in bootloader as it's ECC protected and
 * needs to be intialized only once and to ensure that only one core has done the
 * mailbox ram initialization before ipc_init. If SBL is not used then Gel does the initialization.
 */
#define IPC_NOTIFY_SW_QUEUE_R5FSS0_0_R5FSS0_1      (IpcNotify_SwQueue*)(&gIpcSharedMem[(IPC_SHARED_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*1U)])
#define IPC_RPMSG_VRING_R5FSS0_0_R5FSS0_1          (uintptr_t)(&gIpcSharedMem[(IPC_SHARED_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*1U) - (IPC_RPMESSAGE_VRING_SIZE*1U)])
#define IPC_NOTIFY_SW_QUEUE_R5FSS0_1_R5FSS0_0      (IpcNotify_SwQueue*)(&gIpcSharedMem[(IPC_SHARED_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*2U) - (IPC_RPMESSAGE_VRING_SIZE*1U)])
#define IPC_RPMSG_VRING_R5FSS0_1_R5FSS0_0          (uintptr_t)(&gIpcSharedMem[(IPC_SHARED_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*2U) - (IPC_RPMESSAGE_VRING_SIZE*2U)])

//! [shared mem]

//! [driver api]

/* This function is called within IpcNotify_init, this function returns core specific IPC config */
void IpcNotify_getConfig(IpcNotify_InterruptConfig **interruptConfig, uint32_t *interruptConfigNum)
{
    /* extern globals that are specific to this core */
    extern IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_0[];
    extern uint32_t gIpcNotifyInterruptConfigNum_r5fss0_0;

    *interruptConfig = &gIpcNotifyInterruptConfig_r5fss0_0[0];
    *interruptConfigNum = gIpcNotifyInterruptConfigNum_r5fss0_0;
}

/* This function is called within IpcNotify_init, this function allocates SW queue */
void IpcNotify_allocSwQueue(IpcNotify_MailboxConfig *mailboxConfig)
{
    IpcNotify_MailboxConfig (*mailboxConfigPtr)[CSL_CORE_ID_MAX] = (void *)mailboxConfig;

    mailboxConfigPtr[CSL_CORE_ID_R5FSS0_0][CSL_CORE_ID_R5FSS0_1].swQ = IPC_NOTIFY_SW_QUEUE_R5FSS0_0_R5FSS0_1;
    mailboxConfigPtr[CSL_CORE_ID_R5FSS0_1][CSL_CORE_ID_R5FSS0_0].swQ = IPC_NOTIFY_SW_QUEUE_R5FSS0_1_R5FSS0_0;
}

//! [driver api]

//! [core0 obj]
/* IMPORTANT: 
 * - Below code is for Core 0
 * - All RPMessage_Object MUST be global 
 */
RPMessage_Object gAckReplyMsgObject;
//! [core0 obj]

//! [core1 obj]
/* IMPORTANT: 
 * - Below code is for Core 1
 * - All RPMessage_Object MUST be global 
 */
RPMessage_Object gRecvMsgObject;
//! [core1 obj]

void ipc_rpmessage_sample()
{
{
//! [core0 init]
/* Below code is for reference, recommened to use SysCfg to generate this code */

    /* IMPORTANT: 
     * - Below code is for Core 0
     * - Make sure IPC Notify is enabled before enabling IPC RPMessage
     */
    int32_t status;
    RPMessage_Params initParams;

    RPMessage_Params_init(&initParams);
    initParams.vringTxBaseAddr[CSL_CORE_ID_R5FSS0_1] = IPC_RPMSG_VRING_R5FSS0_0_R5FSS0_1;
    initParams.vringRxBaseAddr[CSL_CORE_ID_R5FSS0_1] = IPC_RPMSG_VRING_R5FSS0_1_R5FSS0_0;
    initParams.vringNumBuf = IPC_RPMESSAGE_NUM_VRING_BUF;
    initParams.vringMsgSize = IPC_RPMESSAGE_MAX_VRING_BUF_SIZE;
    initParams.vringSize = IPC_RPMESSAGE_VRING_SIZE;
    status = RPMessage_init(&initParams);
    DebugP_assert(status==SystemP_SUCCESS);
//! [core0 init]
}
{
//! [core1 init]
/* Below code is for reference, recommened to use SysCfg to generate this code */

    /* IMPORTANT: 
     * - Below code is for Core 1
     * - Make sure IPC Notify is enabled before enabling IPC RPMessage
     */
    int32_t status;
    RPMessage_Params initParams;

    RPMessage_Params_init(&initParams);
    initParams.vringTxBaseAddr[CSL_CORE_ID_R5FSS0_0] = IPC_RPMSG_VRING_R5FSS0_1_R5FSS0_0;
    initParams.vringRxBaseAddr[CSL_CORE_ID_R5FSS0_0] = IPC_RPMSG_VRING_R5FSS0_0_R5FSS0_1;
    initParams.vringNumBuf = IPC_RPMESSAGE_NUM_VRING_BUF;
    initParams.vringMsgSize = IPC_RPMESSAGE_MAX_VRING_BUF_SIZE;
    initParams.vringSize = IPC_RPMESSAGE_VRING_SIZE;
    status = RPMessage_init(&initParams);
    DebugP_assert(status==SystemP_SUCCESS);
//! [core1 init]
}
{
//! [core0 create]
    /* IMPORTANT: 
     * - Below code is for Core 0
     */ 
    RPMessage_CreateParams createParams;
    
    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = 12; /* pick any unique value on that core between 0..RPMESSAGE_MAX_LOCAL_ENDPT-1 
                                   * the value need not be unique across cores 
                                   */
    RPMessage_construct(&gAckReplyMsgObject, &createParams);
//! [core0 create]
}
{
//! [core1 create]
    /* IMPORTANT: 
     * - Below code is for Core 1
     */ 
    RPMessage_CreateParams createParams;
    
    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = 13; /* pick any unique value on that core between 0..RPMESSAGE_MAX_LOCAL_ENDPT-1 
                                   * the value need not be unique across cores 
                                   */
    RPMessage_construct(&gRecvMsgObject, &createParams);
//! [core1 create]
}
{
//! [send]
    /* IMPORTANT: 
     * - Below code is for Core 0
     */ 

    char sendMsg[64] = "hello, world!!!";
    char replyMsg[64];
    uint16_t replyMsgSize, remoteCoreId, remoteCoreEndPt;

    RPMessage_send( 
        sendMsg, strlen(sendMsg), 
        CSL_CORE_ID_R5FSS0_1, 13, 
        RPMessage_getLocalEndPt(&gAckReplyMsgObject),
        SystemP_WAIT_FOREVER);

    /* set 'replyMsgSize' to size of recv buffer, 
     * after return `replyMsgSize` contains actual size of valid data in recv buffer 
     */
    replyMsgSize = sizeof(replyMsg); /*  */
    RPMessage_recv(&gAckReplyMsgObject, 
        replyMsg, &replyMsgSize, 
        &remoteCoreId, &remoteCoreEndPt, 
        SystemP_WAIT_FOREVER);
//! [send]        
}
{
//! [recv]
    /* IMPORTANT: 
     * - Below code is for Core 1
     */ 
    while(1)
    {
        char recvMsg[64];
        char replyMsg[64];
        uint16_t recvMsgSize, remoteCoreId, remoteCoreEndPt;

        /* wait for messages forever in a loop */

        /* set 'recvMsgSize' to size of recv buffer, 
        * after return `recvMsgSize` contains actual size of valid data in recv buffer 
        */
        recvMsgSize = sizeof(recvMsg);
        RPMessage_recv(&gRecvMsgObject, 
            recvMsg, &recvMsgSize, 
            &remoteCoreId, &remoteCoreEndPt, 
            SystemP_WAIT_FOREVER);

        /* echo the message string as reply, we know this is null terminating string
         * so strcpy is safe to use.
         */
        strcpy(replyMsg, recvMsg);

        /* send ack to sender CPU at the sender end point */
        RPMessage_send( 
            replyMsg, strlen(replyMsg), 
            remoteCoreId, remoteCoreEndPt, 
            RPMessage_getLocalEndPt(&gRecvMsgObject),
            SystemP_WAIT_FOREVER);
    }
//! [recv]
}
}