
//! [include]
#include <stdio.h>
#include <string.h>
#include <drivers/ipc_rpmsg.h>
#include <kernel/dpl/DebugP.h>
//! [include]

//! [shared mem]

/* Below code is for reference, recommened to use SysCfg to generate this code */

/* Below is common code for all the CPUs participating in this IPC */

#define NUM_CPUS            (2u) /* R5FSS0-0 and R5FSS0-1 in this example */
#define VRING_NUM           (NUM_CPUS*(NUM_CPUS-1))
#define VRING_NUM_BUF       (8u)
#define VRING_MAX_MSG_SIZE  (128u)   

/* Size of each VRING is
 *     number of buffers x ( size of each buffer + space for data structures of one buffer (32B) ) 
 */ 
#define VRING_SIZE          RPMESSAGE_VRING_SIZE(VRING_NUM_BUF, VRING_MAX_MSG_SIZE) 

/* Make sure of below,  
 * - The section defined below should be placed at the exact same location in memory for all the CPUs
 * - the memory should be marked as non-cached for all the CPUs
 * - the section should be marked as NOLOAD in all the CPUs linker command file
 */ 
uint8_t gVringMem[VRING_NUM][VRING_SIZE] __attribute__((aligned(128), section(".bss.sharedmemory")));

//! [shared mem]

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
    initParams.vringTxBaseAddr[CSL_CORE_ID_R5FSS0_1] = (uintptr_t)gVringMem[0];
    initParams.vringRxBaseAddr[CSL_CORE_ID_R5FSS0_1] = (uintptr_t)gVringMem[1];
    initParams.vringNumBuf = VRING_NUM_BUF;
    initParams.vringMsgSize = VRING_MAX_MSG_SIZE;
    initParams.vringSize = VRING_SIZE;
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
    initParams.vringTxBaseAddr[CSL_CORE_ID_R5FSS0_0] = (uintptr_t)gVringMem[1];
    initParams.vringRxBaseAddr[CSL_CORE_ID_R5FSS0_0] = (uintptr_t)gVringMem[0];
    initParams.vringNumBuf = VRING_NUM_BUF;
    initParams.vringMsgSize = VRING_MAX_MSG_SIZE;
    initParams.vringSize = VRING_SIZE;
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