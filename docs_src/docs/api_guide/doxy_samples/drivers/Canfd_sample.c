//! [include]
#include <string.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/mcan/v0/canfd.h>
#include <drivers/mcan/v0/mcan.h>
#include <drivers/mcan.h>
//! [include]

/** \brief Number of messages sent */
#define MCAN_APP_TEST_MESSAGE_COUNT         100U
/** \brief Data size for each transfer */
#define MCAN_APP_TEST_DATA_SIZE             64U
#define CONFIG_MCAN0                       (0U)

CANFD_Handle                gCanfdHandle;
CANFD_Config                gCanfdConfig[CONFIG_MCAN0];
CANFD_OpenParams            openParams[CONFIG_MCAN0];
uint8_t                     rxData[MCAN_APP_TEST_DATA_SIZE] = {0};
uint8_t                     txData[128U] =
    {0xA1, 0x1A, 0xFF, 0xFF, 0xC1, 0x1C, 0xB1, 0x1B,
    0xA2, 0x2A, 0xFF, 0xFF, 0xC2, 0x2C, 0xB2, 0x2B,
    0xA3, 0x3A, 0xFF, 0xFF, 0xC3, 0x3C, 0xB3, 0x3B,
    0xA4, 0x4A, 0xFF, 0xFF, 0xC4, 0x4C, 0xB4, 0x4B,
    0xA5, 0x5A, 0xFF, 0xFF, 0xC5, 0x5C, 0xB5, 0x5B,
    0xA6, 0x6A, 0xFF, 0xFF, 0xC6, 0x6C, 0xB6, 0x6B,
    0xA7, 0x7A, 0xFF, 0xFF, 0xC7, 0x7C, 0xB7, 0x7B,
    0xA8, 0x8A, 0xFF, 0xFF, 0xC8, 0x8C, 0xB8, 0x8B,
    0xA1, 0x1A, 0xFF, 0xFF, 0xC1, 0x1C, 0xB1, 0x1B,
    0xA2, 0x2A, 0xFF, 0xFF, 0xC2, 0x2C, 0xB2, 0x2B,
    0xA3, 0x3A, 0xFF, 0xFF, 0xC3, 0x3C, 0xB3, 0x3B,
    0xA4, 0x4A, 0xFF, 0xFF, 0xC4, 0x4C, 0xB4, 0x4B,
    0xA5, 0x5A, 0xFF, 0xFF, 0xC5, 0x5C, 0xB5, 0x5B,
    0xA6, 0x6A, 0xFF, 0xFF, 0xC6, 0x6C, 0xB6, 0x6B,
    0xA7, 0x7A, 0xFF, 0xFF, 0xC7, 0x7C, 0xB7, 0x7B,
    0xA8, 0x8A, 0xFF, 0xFF, 0xC8, 0x8C, 0xB8, 0x8B
    };

//! [open]
void open(void)
{
    /* Open CANFD instances */  
    gCanfdHandle = CANFD_open(CONFIG_MCAN0, &openParams[CONFIG_MCAN0]);
    if(NULL == gCanfdHandle)
    {
        DebugP_logError("CANFD open failed!!\r\n");
    }

    return;
}
//! [open]

//! [close]
void close(void)
{
    /* Close CANFD instances that is open */
    if(gCanfdHandle != NULL)
    {
        CANFD_close(gCanfdHandle);
        gCanfdHandle = NULL;
    }

    return;
}
//! [close]

void transfer_blocking(void *args)
{
//! [transfer_blocking]
    CANFD_MessageObject         txMsgObject;
    CANFD_MessageObject         rxMsgObject;
    CANFD_MsgObjHandle          txMsgObjHandle;
    CANFD_MsgObjHandle          rxMsgObjHandle;
    uint32_t                    iterationCount = 0U;
    int32_t                     retVal = SystemP_SUCCESS;

    /* Setup the transmit message object */
    txMsgObject.direction = CANFD_Direction_TX;
    txMsgObject.msgIdType = CANFD_MCANXidType_29_BIT;
    txMsgObject.startMsgId = 0x29E;
    txMsgObject.endMsgId   = 0x29E;
    txMsgObject.txMemType  = MCAN_MEM_TYPE_BUF;
    txMsgObject.dataLength = MCAN_APP_TEST_DATA_SIZE;
    txMsgObject.args       = NULL;
    retVal = CANFD_createMsgObject (gCanfdHandle, &txMsgObject);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\n");
        return;
    }
    txMsgObjHandle = &txMsgObject;

    /* Setup the receive message object */
    rxMsgObject.direction = CANFD_Direction_RX;
    rxMsgObject.msgIdType = CANFD_MCANXidType_29_BIT;
    rxMsgObject.startMsgId = 0x29E;
    rxMsgObject.endMsgId   = 0x29E;
    rxMsgObject.args       = (uint8_t*) rxData;
    rxMsgObject.rxMemType  = MCAN_MEM_TYPE_BUF;
    rxMsgObject.dataLength = MCAN_APP_TEST_DATA_SIZE;
    retVal = CANFD_createMsgObject (gCanfdHandle, &rxMsgObject);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\n");
        return;
    }
    rxMsgObjHandle = &rxMsgObject;

    while (iterationCount != MCAN_APP_TEST_MESSAGE_COUNT)
    {
        /* Send data over Tx message object */
        retVal += CANFD_write (txMsgObjHandle,
                               txMsgObject.startMsgId,
                               CANFD_MCANFrameType_FD,
                               0,
                               &txData[0]);

        /* Compare data */
        for(int32_t i = 0U; i < MCAN_APP_TEST_DATA_SIZE; i++)
        {
            if(txData[i] != rxData[i])
            {
                retVal = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
        }

        if (retVal != SystemP_SUCCESS)
        {
            DebugP_log ("Error: CANFD transmit data for iteration %d failed\n", iterationCount);
            return;
        }

        iterationCount++;
    }

    // Delete Message Object
    retVal = CANFD_deleteMsgObject(txMsgObjHandle);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed\n");
        return;
    }

    retVal = CANFD_deleteMsgObject(rxMsgObjHandle);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed\n");
        return;
    }

    if (retVal == SystemP_SUCCESS)
    {
        DebugP_log("[MCAN] Internal loopback testing for %d iterations Passed\n", iterationCount);
        DebugP_log("All tests have passed\n");
    }
    else
    {
        DebugP_log("[MCAN] Internal loopback testing for %d iterations Failed\n", iterationCount);
        DebugP_log("Some tests have Failed\n");
    }

//! [transfer_blocking]
    return;
}