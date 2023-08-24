
//! [include]
#include <stdio.h>
#include <drivers/ipc_notify.h>
#include <drivers/ipc_notify/v1/ipc_notify_v1.h>
#include <kernel/dpl/DebugP.h>
//! [include]

//! [shared mem]

/* Below code is for reference, recommened to use SysCfg to generate this code */

/* Below is common code for all the CPUs participating in this IPC */

#define IPC_NOTIFY_NUM_CORES              (2U)
#define IPC_NOTIFY_MEMORY_SIZE            (((IPC_NOTIFY_NUM_CORES) * (IPC_NOTIFY_NUM_CORES - 1)) * (MAILBOX_MAX_SW_QUEUE_SIZE))

/* Total Shared memory size used for IPC */
#define IPC_SHARED_MEM_SIZE               (IPC_NOTIFY_MEMORY_SIZE)

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
#define IPC_NOTIFY_SW_QUEUE_R5FSS0_1_R5FSS0_0      (IpcNotify_SwQueue*)(&gIpcSharedMem[(IPC_SHARED_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*2U)])

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

/* dummy definitions to allow compile to pass */
typedef struct {
    uint32_t rsv;
} MyQueue_Obj;
void MyQueue_create(MyQueue_Obj *obj);
int32_t MyQueue_put(MyQueue_Obj *obj, uint32_t value);
int32_t MyQueue_wait(MyQueue_Obj *obj, uint32_t *value);

//! [recv]

/* NOTE: local queue implementation not shown, this is a standard FIFO like SW queue,
 *       which is thread and interrupt safe and can block until there is a element to dequeue
 */

/* local Q to hold received messages */
MyQueue_Obj gMyLocalQ;

void MyMsg_handler(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args)
{
    MyQueue_Obj *myLocalQ = (MyQueue_Obj*)args;

    /* message received from remote core `remoteCoreId`, for client ID `localClientId` on this core */

    /* instead of handling the messages in callback which is called within ISR, queue this into a larger SW queue.
     * Handle to the SW queue is passed via args in this example.
     * SW queue could be one per remote core, one per client ID or a common Q for all remote cores and so on.
     * Passing queue handle as argument allows the handler to remain
     * common across multiple remote cores and client ID's
     */
    MyQueue_put(myLocalQ, msgValue);

    /* NOTE: THis is a sample handler, actually application can have different design based on its
     *       specific requirements
     */
}

/* Message handler task */
void MyTask_main(void *args)
{
    while(1)
    {
        uint32_t msgValue;

        /* block until there is a element to dequeue from this Q */
        MyQueue_wait(&gMyLocalQ, &msgValue);

        if(msgValue == 0x08765432)
        {
            /* handle message value.
             * typically message value will be a command to execute
             * OR
             * it will point (offset or index within a known shared memory base address or array)
             * to command and parameters to execute
             */
        }
    }
}
//! [recv]

void ipc_notify_init()
{
    {
    //! [init]
    int32_t status;
    IpcNotify_Params notifyParams;

    /* initialize parameters to default */
    IpcNotify_Params_init(&notifyParams);

    /* specify the core on which this API is called */
    notifyParams.selfCoreId = CSL_CORE_ID_R5FSS0_0;

    /* list the cores that will do IPC Notify with this core
     * Make sure to NOT list `self` core in the list below
     */
    notifyParams.numCores = 1;
    notifyParams.coreIdList[0] = CSL_CORE_ID_R5FSS0_1;

    status = IpcNotify_init(&notifyParams);
    DebugP_assert(status==SystemP_SUCCESS);
    //! [init]
    }
    {
    //! [register]
    int32_t status;
    /* client ID to register against, make sure messages are sent to this client ID */
    uint16_t clientId = 4;

    /* create a local queue to hold the emssage */
    MyQueue_create(&gMyLocalQ);

    /* register a handler to receive messages */
    status = IpcNotify_registerClient(clientId, MyMsg_handler, &gMyLocalQ);
    DebugP_assert(status==SystemP_SUCCESS);
    //! [register]
    }
    {
    //! [send]
    /* send `msgValue` to `clientId` of core CSL_CORE_ID_R5FSS0_1 */
    int32_t status;
    /* client ID for which this message is intended,
     * make sure a handler is registered for this client ID
     */
    uint16_t clientId = 4;
    /* message value to send, amke sure the
     * registered handler handles this message
     */
    uint32_t msgValue = 0x08765432;

    /* no error checks done inside IpcNotify_sendMsg(), so doing here just to show the constraints */
    DebugP_assert(msgValue < IPC_NOTIFY_MSG_VALUE_MAX);
    DebugP_assert(clientId < IPC_NOTIFY_CLIENT_ID_MAX);

    /* wait until msg is put into internal HW/SW FIFO */
    status = IpcNotify_sendMsg(CSL_CORE_ID_R5FSS0_0, clientId, msgValue, 1);
    DebugP_assert(status==SystemP_SUCCESS);
    //! [send]
    }
}