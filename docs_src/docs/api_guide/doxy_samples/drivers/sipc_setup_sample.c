//![include]
#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include <drivers/sipc_notify.h>
#include <drivers/secure_ipc_notify/sipc_notify_src.h>
//![include]

//![setup]
/* Sysconfig generated parameter QUEUE LENGTH */
#define SIPC_QUEUE_LENGTH                 (32u)

/* Sysconfig generated parameter TOTAL R5 CORES [1,2] */
#define SIPC_NUM_R5_CORES                  (2u)

/* Memory assigned for each R5x <-> HSM channel */
uint8_t gQueue_R5ToHsm[SIPC_NUM_R5_CORES][SIPC_QUEUE_LENGTH*SIPC_MSG_SIZE] __attribute__((aligned(8),section(".bss.sipc_hsm_queue_mem")));
uint8_t gQueue_HsmToR5[SIPC_NUM_R5_CORES][SIPC_QUEUE_LENGTH*SIPC_MSG_SIZE] __attribute__((aligned(8),section(".bss.sipc_r5f_queue_mem")));
//![setup]

void secure_ipc_demo_main(void *args);


// ![r5conf]
/* Following SIPC init will be done by HSM client running on R5*/
void SIPC_R5F_secure_host_config()
{
    SIPC_Params notifyParams;
    int32_t status;

    /* Initialize parameters to default */
    SIPC_Params_init(&notifyParams);

    /* List the cores that will do IPC Notify with this core
    * Make sure to NOT list 'self' core in the list below
    */
    notifyParams.numCores = 1;
    notifyParams.coreIdList[0] = CORE_INDEX_HSM;
    notifyParams.ipcQueue_eleSize_inBytes = SIPC_MSG_SIZE;
    notifyParams.ipcQueue_length = SIPC_QUEUE_LENGTH ;
    /* R5 -> HSM Queue
     * HSM -> R5 queue will be configured by HSM core */
    notifyParams.tx_SipcQueues[CORE_INDEX_HSM] = (uintptr_t)gQueue_R5ToHsm[0] ;
    notifyParams.rx_SipcQueues[CORE_INDEX_HSM] = (uintptr_t)gQueue_HsmToR5[0] ;

    notifyParams.secHostCoreId[CORE_INDEX_SEC_MASTER_0] = CORE_ID_R5FSS0_0 ;
    notifyParams.secHostCoreId[CORE_INDEX_SEC_MASTER_1] = CORE_ID_R5FSS0_1 ;

    /* Initialize the IPC Notify module */
    status = SIPC_init(&notifyParams);
    DebugP_assert(status==SystemP_SUCCESS);
}
// ![r5conf]

//![hsmconf]
/* Following SIPC init will be done by HSM server */
void SIPC_HSM_config(void)
{
    SIPC_Params notifyParams;
    int32_t status;

    /* Initialize parameters to default */
    SIPC_Params_init(&notifyParams);

    /* List the cores that will do IPC Notify with this core
    * Make sure to NOT list 'self' core in the list below
    */
    notifyParams.numCores = 2;
    notifyParams.coreIdList[0] = CORE_INDEX_SEC_MASTER_0 ;
    notifyParams.coreIdList[1] = CORE_INDEX_SEC_MASTER_1 ;
    notifyParams.ipcQueue_eleSize_inBytes = SIPC_MSG_SIZE;
    notifyParams.ipcQueue_length = SIPC_QUEUE_LENGTH ;
    /* This is HSM -> R50 queue */
    notifyParams.tx_SipcQueues[CORE_INDEX_SEC_MASTER_0] = (uintptr_t)gQueue_HsmToR5[0] ;
    /* This is HSM -> R51 queue */
    notifyParams.tx_SipcQueues[CORE_INDEX_SEC_MASTER_1] = (uintptr_t)gQueue_HsmToR5[1] ;

    notifyParams.rx_SipcQueues[CORE_INDEX_SEC_MASTER_0] = (uintptr_t)gQueue_R5ToHsm[0] ;
    /* This is R5 -> HSM queue */
    notifyParams.rx_SipcQueues[CORE_INDEX_SEC_MASTER_1] = (uintptr_t)gQueue_R5ToHsm[1] ;

    notifyParams.secHostCoreId[CORE_INDEX_SEC_MASTER_0] = CORE_ID_R5FSS0_0 ;
    notifyParams.secHostCoreId[CORE_INDEX_SEC_MASTER_1] = CORE_ID_R5FSS0_1 ;

    /* Initialize the SIPC Notify module */
    status = SIPC_init(&notifyParams);
    DebugP_assert(status==SystemP_SUCCESS);
}
//![hsmconf]

//![linker]

SECTIONS
{
    /*memory sections configurations*/

    /* this is used only when Secure IPC is enabled */
    .bss.sipc_hsm_queue_mem   (NOLOAD) : {} > MAILBOX_HSM
    .bss.sipc_r5f_queue_mem   (NOLOAD) : {} > MAILBOX_R5F
}
MEMORY
{
    /* definitions of memory sections */

    /* MSS mailbox memory is used as shared memory, we dont use bottom 32*12 bytes, since its used as SW queue by ipc_notify */
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0x72000000, LENGTH = 0x3E80
    MAILBOX_HSM:    ORIGIN = <HSM MBOX START ADDR> , LENGTH = 0x000003CE
    MAILBOX_R5F:    ORIGIN = <HSM MBOX START ADDR + 0x000003CE> , LENGTH = 0x000003CE
}
//![linker]

// ![exusage]

/* client ID that is used to send and receive messages */
uint32_t gSrcClientId0 = 3u;
uint32_t gSrcClientId1 = 4u;

/* R5 cores will send a message to HSM and it supports only 2 client IDs i.e
 * ClientId = 0 for BootNotify message.
 * ClientId = 1 for general message passing */
uint32_t gDestClientId = 1u;

/* Sec Host core that starts the message exchange */
uint32_t gSecMasterId0 = CORE_INDEX_SEC_MASTER_0 ;
uint32_t gSecMasterId1 = CORE_INDEX_SEC_MASTER_1 ;
/* Remote cores that echo messages from main core,
 * Make sure to NOT list main core in this list */
uint32_t gRemoteCoreId = CORE_INDEX_HSM;

/* Semaphore's used to indicate a main core has finished all message exchanges */
SemaphoreP_Object gMainDoneSem;

void sipc_sec_host_msg_handler(uint8_t remoteCoreId, uint8_t localClientId, uint8_t remoteClientId, uint8_t* msgValue, void *args)
{
    /* Handle the received message */

     /* post the semaphore */
    SemaphoreP_post(@gMainDoneSem);
}
void sipc_sec_host_start(void)
{
    int32_t status;
    /* Create completion semaphore for this core */
    SemaphoreP_constructBinary(&gMainDoneSem, 0);
    /* Register a handler to receive messages */
    status = SIPC_registerClient(gSrcClientId1, sipc_sec_host_msg_handler, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    uint8_t msgValue[SIPC_MSG_SIZE] = { 0x00 , 0x00 ,0x02,0x03,0x04,0x05,0x06,0x07};

    status = SIPC_sendMsg(gRemoteCoreId, gDestClientId, gSrcClientId1 ,msgValue, 1);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Wait for message to be echoed back by HSM  */
    SemaphoreP_pend(&gMainDoneSem, SystemP_WAIT_FOREVER);
}
void main()
{
    /* sipc initialization which will be done via sysconfig. */
    SIPC_R5F_secure_host_config();
    sipc_sec_host_start();
}
// ![exusage]