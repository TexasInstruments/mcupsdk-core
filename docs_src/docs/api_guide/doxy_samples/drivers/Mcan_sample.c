
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr_soc.h>
//! [include]
#include <drivers/mcan.h>
//! [include]

#define CONFIG_MCAN0_BASE_ADDR                  (CSL_MCAN0_MSGMEM_RAM_BASE)
#define CONFIG_MCAN0_INTR                       (187U)
#define CONFIG_MCAN_NUM_INSTANCES               (1U)

#define APP_MCAN_BASE_ADDR                       (CONFIG_MCAN0_BASE_ADDR)
#define APP_MCAN_INTR_NUM                        (CONFIG_MCAN0_INTR)
#define APP_MCAN_MSG_LOOP_COUNT                  (10U)

/* Allocate Message RAM memory section to filter elements, buffers, FIFO */
/* Maximum STD Filter Element can be configured is 128 */
#define APP_MCAN_STD_ID_FILTER_CNT               (1U)
/* Maximum EXT Filter Element can be configured is 64 */
#define APP_MCAN_EXT_ID_FILTER_CNT               (0U)
/* Maximum TX Buffer + TX FIFO, combined can be configured is 32 */
#define APP_MCAN_TX_BUFF_CNT                     (1U)
#define APP_MCAN_TX_FIFO_CNT                     (0U)
/* Maximum TX Event FIFO can be configured is 32 */
#define APP_MCAN_TX_EVENT_FIFO_CNT               (0U)
/* Maximum RX FIFO 0 can be configured is 64 */
#define APP_MCAN_FIFO_0_CNT                      (0U)
/* Maximum RX FIFO 1 can be configured is 64 and
 * rest of the memory is allocated to RX buffer which is again of max size 64 */
#define APP_MCAN_FIFO_1_CNT                      (0U)

/* Standard Id configured in this app */
#define APP_MCAN_STD_ID                          (0xC0U)
#define APP_MCAN_STD_ID_MASK                     (0x7FFU)
#define APP_MCAN_STD_ID_SHIFT                    (18U)

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gMcanTxDoneSem, gMcanRxDoneSem;
static HwiP_Object       gMcanHwiObject;
static uint32_t          gMcanBaseAddr = APP_MCAN_BASE_ADDR;

/* Static Function Declarations */
static void    App_mcanIntrISR(void *arg);

static int32_t App_mcanIntrReg()
{
    //! [App_mcanIntrReg]
    int32_t                 status = SystemP_SUCCESS;
    HwiP_Params             hwiPrms;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = APP_MCAN_INTR_NUM;
    hwiPrms.callback    = &App_mcanIntrISR;
    status              = HwiP_construct(&gMcanHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    //! [App_mcanIntrReg]

    return status;
}

static void App_mcanInitMsgRamConfigParams(MCAN_MsgRAMConfigParams
                                           *msgRAMConfigParams)
{
    int32_t status;
    //! [App_mcanInitMsgRamConfigParams]
    MCAN_initMsgRamConfigParams(msgRAMConfigParams);

    /* Configure the user required msg ram params */
    msgRAMConfigParams->lss = APP_MCAN_STD_ID_FILTER_CNT;
    msgRAMConfigParams->lse = APP_MCAN_EXT_ID_FILTER_CNT;
    msgRAMConfigParams->txBufCnt = APP_MCAN_TX_BUFF_CNT;
    msgRAMConfigParams->txFIFOCnt = APP_MCAN_TX_FIFO_CNT;
    /* Buffer/FIFO mode is selected */
    msgRAMConfigParams->txBufMode = MCAN_TX_MEM_TYPE_BUF;
    msgRAMConfigParams->txEventFIFOCnt = APP_MCAN_TX_EVENT_FIFO_CNT;
    msgRAMConfigParams->rxFIFO0Cnt = APP_MCAN_FIFO_0_CNT;
    msgRAMConfigParams->rxFIFO1Cnt = APP_MCAN_FIFO_1_CNT;
    /* FIFO blocking mode is selected */
    msgRAMConfigParams->rxFIFO0OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;
    msgRAMConfigParams->rxFIFO1OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;

    status = MCAN_calcMsgRamParamsStartAddr(msgRAMConfigParams);
    DebugP_assert(status == CSL_PASS);
    //! [App_mcanInitMsgRamConfigParams]
    return;
}

static void App_mcanInitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem,
                                            uint32_t bufNum)
{
    //! [App_mcanInitStdFilterElemParams]
    /* sfid1 defines the ID of the standard message to be stored. */
    stdFiltElem->sfid1 = APP_MCAN_STD_ID;
    /* As buffer mode is selected, sfid2 should be bufNum[0 - 63] */
    stdFiltElem->sfid2 = bufNum;
    /* Store message in buffer */
    stdFiltElem->sfec  = MCAN_STD_FILT_ELEM_BUFFER;
    /* Below configuration is ignored if message is stored in buffer */
    stdFiltElem->sft   = MCAN_STD_FILT_TYPE_RANGE;
    //! [App_mcanInitStdFilterElemParams]
    return;
}

static void App_mcanConfigTxMsg(MCAN_TxBufElement *txMsg)
{
    uint32_t i;
    //! [App_mcanConfigTxMsg]
    /* Initialize message to transmit */
    MCAN_initTxBufElement(txMsg);
    /* Standard message identifier 11 bit, stored into ID[28-18] */
    txMsg->id  = ((APP_MCAN_STD_ID & MCAN_STD_ID_MASK) << MCAN_STD_ID_SHIFT);
    txMsg->dlc = MCAN_DATA_SIZE_64BYTES; /* Payload size is 64 bytes */
    txMsg->fdf = TRUE; /* CAN FD Frame Format */
    txMsg->xtd = FALSE; /* Extended id not configured */
    for (i = 0U; i < MCAN_MAX_PAYLOAD_BYTES; i++)
    {
        txMsg->data[i] = i;
    }
    //! [App_mcanConfigTxMsg]
    return;
}

void App_mcanTxRxMsg(void)
{
    int32_t                 status;
    MCAN_TxBufElement       txMsg;
    MCAN_ProtocolStatus     protStatus;
    MCAN_RxBufElement       rxMsg;
    MCAN_RxNewDataStatus    newDataStatus;
    MCAN_ErrCntStatus       errCounter;
    uint32_t                i, bufNum, fifoNum, bitPos = 0U;

    //! [App_mcanTxRxMsg]
    /* Configure Tx Msg to transmit */
    App_mcanConfigTxMsg(&txMsg);

    /* Select buffer number, 32 buffers available */
    bufNum = 0U;
    /* Enable Transmission interrupt for the selected buf num,
     * If FIFO is used, then need to send FIFO start index until FIFO count */
    status = MCAN_txBufTransIntrEnable(gMcanBaseAddr, bufNum, (uint32_t)TRUE);
    DebugP_assert(status == CSL_PASS);

    /* Write message to Msg RAM */
    MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_BUF, bufNum, &txMsg);

    /* Add request for transmission, This function will trigger transmission */
    status = MCAN_txBufAddReq(gMcanBaseAddr, bufNum);
    DebugP_assert(status == CSL_PASS);

    /* Wait for Tx completion */
    SemaphoreP_pend(&gMcanTxDoneSem, SystemP_WAIT_FOREVER);

    MCAN_getProtocolStatus(gMcanBaseAddr, &protStatus);
    /* Checking for Tx Errors */
    if (((MCAN_ERR_CODE_NO_ERROR != protStatus.lastErrCode) ||
         (MCAN_ERR_CODE_NO_CHANGE != protStatus.lastErrCode)) &&
        ((MCAN_ERR_CODE_NO_ERROR != protStatus.dlec) ||
         (MCAN_ERR_CODE_NO_CHANGE != protStatus.dlec)) &&
        (0U != protStatus.pxe))
    {
         DebugP_assert(FALSE);
    }

    /* Wait for Rx completion */
    SemaphoreP_pend(&gMcanRxDoneSem, SystemP_WAIT_FOREVER);

    /* Checking for Rx Errors */
    MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
    DebugP_assert((0U == errCounter.recErrCnt) &&
                  (0U == errCounter.canErrLogCnt));

    /* Get the new data staus, indicates buffer num which received message */
    MCAN_getNewDataStatus(gMcanBaseAddr, &newDataStatus);
    MCAN_clearNewDataStatus(gMcanBaseAddr, &newDataStatus);

    /* Select buffer and fifo number, Buffer is used in this app */
    bufNum = 0U;
    fifoNum = MCAN_RX_FIFO_NUM_0;

    bitPos = (1U << bufNum);
    if (bitPos == (newDataStatus.statusLow & bitPos))
    {
        MCAN_readMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_BUF, bufNum, fifoNum, &rxMsg);
    }
    else
    {
        DebugP_assert(FALSE);
    }

    /* Compare Tx/Rx data */
    if (((txMsg.id >> APP_MCAN_STD_ID_SHIFT) & APP_MCAN_STD_ID_MASK) ==
            ((rxMsg.id >> APP_MCAN_STD_ID_SHIFT) & APP_MCAN_STD_ID_MASK))
    {
        for (i = 0U; i < MCAN_MAX_PAYLOAD_BYTES; i++)
        {
            if (txMsg.data[i] != rxMsg.data[i])
            {
                DebugP_logError("Data mismatch !!!\r\n");
                DebugP_assert(FALSE);
            }
        }
    }
    else
    {
        DebugP_logError("Message ID mismatch !!!\r\n");
        DebugP_assert(FALSE);
    }
    //! [App_mcanTxRxMsg]

    return;
}

static void App_mcanIntrISR(void *arg)
{
    uint32_t intrStatus;
    //! [App_mcanIntrISR]
    intrStatus = MCAN_getIntrStatus(gMcanBaseAddr);
    MCAN_clearIntrStatus(gMcanBaseAddr, intrStatus);

    if (MCAN_INTR_SRC_TRANS_COMPLETE ==
        (intrStatus & MCAN_INTR_SRC_TRANS_COMPLETE))
    {
        SemaphoreP_post(&gMcanTxDoneSem);
    }

    /* If FIFO0/FIFO1 is used, then MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG macro
     * needs to be replaced by MCAN_INTR_SRC_RX_FIFO0_NEW_MSG/
     * MCAN_INTR_SRC_RX_FIFO1_NEW_MSG respectively */
    if (MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG ==
        (intrStatus & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG))
    {
        SemaphoreP_post(&gMcanRxDoneSem);
    }
    //! [App_mcanIntrISR]
    return;
}

static void App_mcanIntrDeReg()
{
    //! [App_mcanIntrDeReg]
    HwiP_destruct(&gMcanHwiObject);
    //! [App_mcanIntrDeReg]
}
