
//! [include]
#include <stdio.h>
#include <string.h>
#include <drivers/ipc_rpmsg.h>
#include <kernel/dpl/DebugP.h>
//! [include]

//! [defines]
/* This is used to run the echo test with user space kernel */
#define IPC_RPMESSAGE_SERVICE_CHRDEV      "rpmsg_chrdev"
#define IPC_RPMESSAGE_ENDPT_CHRDEV_PING   (14U)

/* maximum size that message can have in this example */
#define IPC_RPMESSAGE_MAX_MSG_SIZE        (96u)

//! [defines]

//! [resource tab]
/* Buffer used for trace, address and size of this buffer is put in the resource table so that Linux can read it */
char gDebugMemLog[DebugP_MEM_LOG_SIZE] __attribute__ ((section (".bss.debug_mem_trace_buf"), aligned (128)));
uint32_t gDebugMemLogSize = DebugP_MEM_LOG_SIZE;

const RPMessage_ResourceTable gRPMessage_linuxResourceTable __attribute__ ((section (".resource_table"), aligned (4096))) =
{
    {
        1U,         /* we're the first version that implements this */
        2U,         /* number of entries, MUST be 2 */
        { 0U, 0U, } /* reserved, must be zero */
    },
    /* offsets to the entries */
    {
        offsetof(RPMessage_ResourceTable, vdev),
        offsetof(RPMessage_ResourceTable, trace),
    },
    /* vdev entry */
    {
        RPMESSAGE_RSC_TYPE_VDEV, RPMESSAGE_RSC_VIRTIO_ID_RPMSG,
        0U, 1U, 0U, 0U, 0U, 2U, { 0U, 0U },
    },
    /* the two vrings */
    { RPMESSAGE_RSC_VRING_ADDR_ANY, 4096U, 256U, 1U, 0U },
    { RPMESSAGE_RSC_VRING_ADDR_ANY, 4096U, 256U, 2U, 0U },
    {
        (RPMESSAGE_RSC_TRACE_INTS_VER0 | RPMESSAGE_RSC_TYPE_TRACE),
        (uint32_t)gDebugMemLog, DebugP_MEM_LOG_SIZE,
        0, "trace:m4fss0_0",
    },
};
//! [resource tab]

//! [obj]
/* IMPORTANT:
 * - Below code is for Core 1
 * - All RPMessage_Object MUST be global
 */
RPMessage_Object gRecvMsgObject;
//! [obj]

void ipc_rpmessage_sample()
{
{
//! [init]
/* Below code is for reference, recommened to use SysCfg to generate this code */

    /* IMPORTANT:
     * - Make sure IPC Notify is enabled before enabling IPC RPMessage
     */

    RPMessage_Params rpmsgParams;
    int32_t status;

    /* initialize parameters to default */
    RPMessage_Params_init(&rpmsgParams);

    rpmsgParams.linuxResourceTable = &gRPMessage_linuxResourceTable;
    rpmsgParams.linuxCoreId = CSL_CORE_ID_A53SS0_0;

    /* initialize the IPC RP Message module */
    status = RPMessage_init(&rpmsgParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* This API MUST be called by applications when its ready to talk to Linux */
    status = RPMessage_waitForLinuxReady(SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);

//! [init]
}
{
//! [create]
    /* IMPORTANT:
     * - Below code is for Core 0
     */
    RPMessage_CreateParams createParams;
    int32_t status;

    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = IPC_RPMESSAGE_ENDPT_CHRDEV_PING;
    status = RPMessage_construct(&gRecvMsgObject, &createParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* We need to "announce" to Linux client else Linux does not know a service exists on this CPU
     * This is not mandatory to do for RTOS clients
     */
    status = RPMessage_announce(CSL_CORE_ID_A53SS0_0, IPC_RPMESSAGE_ENDPT_CHRDEV_PING, IPC_RPMESSAGE_SERVICE_CHRDEV);
    DebugP_assert(status==SystemP_SUCCESS);
//! [create]
}
{
//! [recv]
    /* IMPORTANT:
     * - Below code is for Core 1
     */
    while(1)
    {
        char recvMsg[IPC_RPMESSAGE_MAX_MSG_SIZE + 1];
        char replyMsg[IPC_RPMESSAGE_MAX_MSG_SIZE + 1];
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