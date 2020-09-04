
//! [include]
#include <stdio.h>
#include <drivers/ipc_notify.h>
#include <kernel/dpl/DebugP.h>
//! [include]

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

void ipc_notofy_init()
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