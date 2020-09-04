#include <stddef.h>
#include <string.h>

//! [icss_emac_include]
#include <networking/icss_emac/icss_emac.h>
//! [icss_emac_include]

#define CONFIG_PRU_ICSS1 (0U)
#define CONFIG_ICSS_EMAC1 (0U)

void icss_emac_open(void)
{
//! [icss_emac_open]

    PRUICSS_Handle      pruicssHandle;
    ICSS_EMAC_Params    icssEmacParams;
    ICSS_EMAC_Handle    icssEmacHandle;

    pruicssHandle = PRUICSS_open(CONFIG_PRU_ICSS1);
    DebugP_assert(pruicssHandle != NULL);

    ICSS_EMAC_Params_init(&icssEmacParams);

    /* Fill the icssEmacParams as needed */

    /*Use CONFIG_ICSS_EMAC1 (as configured in SysConfig) macro as parameter to ICSS_EMAC_open */
    icssEmacHandle = ICSS_EMAC_open(CONFIG_ICSS_EMAC1, &icssEmacParams);
    DebugP_assert(icssEmacHandle != NULL);

//! [icss_emac_open]
}

void icss_emac_tx(void)
{
    ICSS_EMAC_Handle        icssEmacHandle = NULL;
    uint8_t                 testPacketArray[ICSS_EMAC_MAXMTU];
//! [icss_emac_tx]
    ICSS_EMAC_TxArgument    txArgs;
    int32_t                 status;

    txArgs.icssEmacHandle = icssEmacHandle;
    txArgs.srcAddress = &testPacketArray[0];
    txArgs.portNumber = ICSS_EMAC_PORT_1;
    txArgs.queuePriority = ICSS_EMAC_QUEUE4;
    txArgs.lengthOfPacket = sizeof(testPacketArray);

    status = ICSS_EMAC_txPacket(&txArgs, NULL);
    DebugP_assert(status == SystemP_SUCCESS);
//! [icss_emac_tx]
}

//! [icss_emac_rx]

/*
Assume that following callback was registered while passing ICSS_EMAC_Params for ICSS_EMAC_open call
icssEmacParams.callBackObject.rxNRTCallBack.callBack = (ICSS_EMAC_CallBack)nrtCallbackRx;
*/
void nrtCallbackRx(void *icssEmacHandleVoidPtr, void *queueNum, void *userArg)
{
    ICSS_EMAC_Handle        icssEmacHandle = (ICSS_EMAC_Handle)icssEmacHandleVoidPtr;
    ICSS_EMAC_RxArgument    rxArgs;
    int32_t                 packetLength;
    uint8_t                 testPacketArray[ICSS_EMAC_MAXMTU];

    rxArgs.icssEmacHandle = icssEmacHandle;
    rxArgs.queueNumber = ((uint32_t)(queueNum));
    rxArgs.more = 0; /* Returns more which is set to 1 if there are more frames in the queue */
    rxArgs.port = 0; /* Returns port number on which frame was received */
    rxArgs.destAddress =  (uint32_t)(&testPacketArray[0]);

    memset(testPacketArray, 0, ICSS_EMAC_MAXMTU);

    packetLength = ICSS_EMAC_rxPktGet(&rxArgs, NULL);
    /* Typically packetLength would be returned to caller. In example typecast to void to kill warning 
     * regarding variable set but not used 
     */
    (void)packetLength;
}
//! [icss_emac_rx]

void icss_emac_ioctl(void)
{
    ICSS_EMAC_Handle    icssEmacHandle = NULL;
//! [icss_emac_ioctl]
    ICSS_EMAC_IoctlCmd  ioctlParams;
    uint8_t             ioctlvalue = 0;

    ioctlvalue = ICSS_EMAC_IOCTL_PORT_CTRL_DISABLE;
    ioctlParams.ioctlVal = &ioctlvalue;
    ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, (uint8_t)ICSS_EMAC_PORT_1, (void*)&ioctlParams);
//! [icss_emac_ioctl]
}
