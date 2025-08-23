/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/**
 *   @file  canfd.c
 *
 *   @brief
 *      The file implements the Controller Area Network Driver Flexible data.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/mcan/v0/canfd.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>

/* ========================================================================== */
/*                             Defines                                        */
/* ========================================================================== */

/* Timeout value for OPMODE update */
#define MCAN_OPMODE_TIMEOUT_US        (500000)
#define MCAN_OPMODE_TIMEOUT           (ClockP_usecToTicks(MCAN_OPMODE_TIMEOUT_US))
/* Timeout value for MEMINIT operation */
#define MCAN_MEMINIT_TIMEOUT_US       (500000)
#define MCAN_MEMINIT_TIMEOUT          (ClockP_usecToTicks(MCAN_MEMINIT_TIMEOUT_US))

/*  MCAN power down timeout in micro seconds  */
#define MCAN_POWER_DOWN_TIMEOUT_IN_US     (500U)
/*  MCAN wakeup timeout in micro seconds  */
#define MCAN_WAKEUP_TIMEOUT_IN_US         (500U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void CANFD_processFIFOElements(CANFD_Handle handle, MCAN_RxFIFONum fifoNum);
static int32_t CANFD_configMessageRAM(uint32_t baseAddr, 
                                      const CANFD_MCANMsgRAMCfgParams* configParams);
static int32_t CANFD_updateOpMode(uint32_t baseAddr, uint32_t mode);
static void CANFD_transCompleteInterrupt(CANFD_Handle handle);
static void CANFD_receiveBufferInterrupt(CANFD_Handle handle);
static void CANFD_transferCancelInterrupt(CANFD_Handle handle);
static int32_t MCAN_configParamsLss(uint32_t lss);
static int32_t MCAN_configParamsLse(uint32_t lse);
static int32_t MCAN_configParamsTxBuffNum(uint32_t txBufNum);
static int32_t MCAN_configParamsTxFifoSize(uint32_t txFIFOSize);
static int32_t MCAN_configParamsTxEventFifoSize(uint32_t txEventFIFOSize);
static int32_t MCAN_configParamsRxFifoSize(uint32_t rxFIFOSize);
static int32_t MCAN_configParamsFifoOpMode(uint32_t fifoOpMode);
static int32_t MCAN_CheckRegBaseAddr(uint32_t baseAddr);
static int32_t CANFD_writePoll(CANFD_MsgObjHandle handle,
                                uint32_t id,
                                CANFD_MCANFrameType frameType,
                                const uint8_t* data);

static int32_t CANFD_writeIntr(CANFD_MsgObjHandle handle,
                                uint32_t id,
                                CANFD_MCANFrameType frameType,
                                const uint8_t* data);

static int32_t CANFD_readPoll(CANFD_MsgObjHandle handle, uint8_t* data);

static int32_t CANFD_readIntr(CANFD_MsgObjHandle handle, uint8_t* data);

static int32_t CANFD_readDma(CANFD_MsgObjHandle handle,
                              uint32_t numMsgs,
                              const uint8_t* data);

static int32_t CANFD_readPollProcessFIFO(CANFD_MessageObject* ptrCanMsgObj,
                                         uint32_t fifoNum);

static int32_t CANFD_readPollProcessBuff(CANFD_MessageObject *ptrCanMsgObj, uint8_t* data);

static int32_t CANFD_writeIntrProcessBuff(CANFD_MsgObjHandle handle,
                                          uint32_t id,
                                          CANFD_MCANFrameType frameType,
                                          const uint8_t* data);
static int32_t CANFD_writePollProcessBuff(CANFD_MsgObjHandle handle,
                                          uint32_t id,
                                          CANFD_MCANFrameType frameType,
                                          const uint8_t* data);
static int32_t CANFD_writePollProcessFIFO(CANFD_MsgObjHandle handle,
                                          uint32_t id,
                                          CANFD_MCANFrameType frameType,
                                          const uint8_t* data);
static int32_t CANFD_writeIntrProcessFIFO(CANFD_MsgObjHandle msgObjHandle,
                                          uint32_t id,
                                          CANFD_MCANFrameType frameType,
                                          const uint8_t* data);
static int32_t CANFD_configInstance(CANFD_Handle hCanfd);
static int32_t CANFD_deConfigInstance(CANFD_Handle handle);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct
{
    void                   *lock;
    /**< Driver lock - to protect across open/close */
    SemaphoreP_Object       lockObj;
    /**< Driver lock object */
} CANFD_DrvObj;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/** \brief Driver object */
static CANFD_DrvObj     gCANFDDrvObj =
{
    .lock           = NULL,
};

extern uint32_t gCANFDConfigNum;
extern CANFD_Config gCanfdConfig[];
extern CANFD_DmaHandle gCanfdDmaHandle[];
extern CANFD_DmaChConfig gCanfdDmaChCfg[];
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  @b Description
 *  @n
 *      Function processes the Rx FIFO and calls application callback function to receive data.
 *
 *  @param[in]  ptrCanFdMCB
 *      Pointer to the CANFD driver MCB
 *  @param[in]  fifoNum
 *      Rx FiFO number
 *
 *  @retval
 *      Success -   Handle to the CAN Driver
 *  @retval
 *      Error   -   NULL
 */
static void CANFD_processFIFOElements(CANFD_Handle handle, MCAN_RxFIFONum fifoNum)
{
    MCAN_RxFIFOStatus       fifoStatus = {0};
    uint32_t                fillLevel;
    uint32_t                index;
    CANFD_MessageObject*    ptrCanMsgObj = NULL;
    uint32_t                baseAddr;
    CANFD_Config           *config = NULL;
    CANFD_Object           *ptrCanFdObj = NULL;

    if(handle != NULL)
    {
        config      = (CANFD_Config*) handle;
        ptrCanFdObj = config->object;
        baseAddr    = ptrCanFdObj->regBaseAddress;

        /* Get the FIFO status */
        fifoStatus.num = (uint32_t)fifoNum;
        MCAN_getRxFIFOStatus(baseAddr, &fifoStatus);
        fillLevel = fifoStatus.fillLvl;

        for(index = (uint32_t)0U; index < fillLevel; index++)
        {
            /* Get the message object pointer */
            ptrCanMsgObj = ptrCanFdObj->rxMapping[fifoStatus.getIdx];
            if(ptrCanMsgObj != NULL)
            {
                MCAN_readMsgRam(baseAddr, MCAN_MEM_TYPE_FIFO, fifoStatus.getIdx, 
                                (uint32_t)fifoNum, &ptrCanFdObj->rxBuffElem);

                /* Copy the data */
                (void)memcpy (ptrCanMsgObj->args, 
                            (void*)&ptrCanFdObj->rxBuffElem.data, 
                            ptrCanMsgObj->dataLength);

                /* Increment the number of interrupts received */
                ptrCanMsgObj->interruptsRxed++;

                /* Call the registered callback. */
                CANFD_transferCallBack((CANFD_MsgObjHandle)ptrCanMsgObj, CANFD_Reason_RX);

                /* Acknowledge the data read */
                (void)MCAN_writeRxFIFOAck(baseAddr, (uint32_t)fifoNum, fifoStatus.getIdx);

                MCAN_getRxFIFOStatus(baseAddr, &fifoStatus);
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is the registered interrupt 0 ISR for the CANFD Driver.
 *
 *  @param[in]  arg
 *      Argument which is registered with the OS while registering the ISR
 *
 *  \ingroup CANFD_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void CANFD_int0Isr (void * args)
{
    uint32_t                baseAddr;
    uint32_t                intrStatus = 0U, errStatus = false;
    CANFD_Object           *ptrCanFdObj = NULL;
    CANFD_Handle            canFdHandle = NULL;
    CANFD_Config           *config = NULL;

    if(args != NULL)
    {
        config      = (CANFD_Config*) args;
        canFdHandle = (CANFD_Config*) config;
        ptrCanFdObj = config->object;
        baseAddr    = ptrCanFdObj->regBaseAddress;
        /* Increment the number of interrupts received */
        ptrCanFdObj->interrupts++;

        if(baseAddr != (uint32_t)0U)
        {
            intrStatus = MCAN_getIntrStatus(baseAddr);
            MCAN_clearIntrStatus(baseAddr, intrStatus);
        }

        /* Process Bus-Off condition */
        if ((intrStatus & MCAN_INTR_SRC_BUS_OFF_STATUS) == MCAN_INTR_SRC_BUS_OFF_STATUS)
        {
            /* Increment the number of interrupts received */
            ptrCanFdObj->busOffInterrupts++;

            ptrCanFdObj->state = CANFD_DRIVER_STATE_STOPPED;

            /* Call the registered callback. */
            CANFD_errStatusCallBack(canFdHandle, CANFD_Reason_BUSOFF, NULL);
        }

        /* Process Protocol error in data phase condition */
        if ((intrStatus & MCAN_INTR_SRC_PROTOCOL_ERR_DATA) == MCAN_INTR_SRC_PROTOCOL_ERR_DATA)
        {
            /* Increment the number of interrupts received */
            ptrCanFdObj->protoDataErrInterrupts++;

            /* Call the registered callback. */
            CANFD_errStatusCallBack(canFdHandle, CANFD_Reason_PROTOCOL_ERR_DATA_PHASE, NULL);
        }

        /* Process Protocol error in arbitration phase condition */
        if ((intrStatus & MCAN_INTR_SRC_PROTOCOL_ERR_ARB) == MCAN_INTR_SRC_PROTOCOL_ERR_ARB)
        {
            /* Increment the number of interrupts received */
            ptrCanFdObj->protoArbErrInterrupts++;

            /* Call the registered callback. */
            CANFD_errStatusCallBack(canFdHandle, CANFD_Reason_PROTOCOL_ERR_ARB_PHASE, NULL);
        }
    
        /* Process message lost interrupt */
        if (((intrStatus & MCAN_INTR_SRC_RX_FIFO0_MSG_LOST) == MCAN_INTR_SRC_RX_FIFO0_MSG_LOST) ||
            ((intrStatus & MCAN_INTR_SRC_RX_FIFO1_MSG_LOST) == MCAN_INTR_SRC_RX_FIFO1_MSG_LOST))
        {
            ptrCanFdObj->state = CANFD_DRIVER_STATE_STOPPED;
            errStatus = true;
            /* Call the registered error callback. */
            if((intrStatus & MCAN_INTR_SRC_RX_FIFO0_MSG_LOST) == MCAN_INTR_SRC_RX_FIFO0_MSG_LOST)
            {
                CANFD_errStatusCallBack(canFdHandle, CANFD_Reason_SRC_RX_FIFO0_MSG_LOST, NULL);
            }
            else
            {
                CANFD_errStatusCallBack(canFdHandle, CANFD_Reason_SRC_RX_FIFO1_MSG_LOST, NULL);
            }
        }

        /* Process Transmit complete interrrupt */
        if (((intrStatus & MCAN_INTR_SRC_TRANS_COMPLETE) == MCAN_INTR_SRC_TRANS_COMPLETE) && 
            ((errStatus != (uint32_t)true)))
        {
            CANFD_transCompleteInterrupt(canFdHandle);
        }

        /* Process Receive buffer interrupt */
        if ((intrStatus & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) == MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG)
        {
            CANFD_receiveBufferInterrupt(canFdHandle);
        }

        /* Process Receive FIFO interrupts */
        if ((intrStatus & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) == MCAN_INTR_SRC_RX_FIFO0_NEW_MSG)
        {
            /* Process FIFO 0 */
            CANFD_processFIFOElements(canFdHandle, MCAN_RX_FIFO_NUM_0);
        }

        if ((intrStatus & MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) == MCAN_INTR_SRC_RX_FIFO1_NEW_MSG)
        {
            /* Process FIFO 1 */
            CANFD_processFIFOElements(canFdHandle, MCAN_RX_FIFO_NUM_1);
        }

        /* Process Transmit cancel interrrupt */
        if ((intrStatus & MCAN_INTR_SRC_TRANS_CANCEL_FINISH) == MCAN_INTR_SRC_TRANS_CANCEL_FINISH)
        {
            CANFD_transferCancelInterrupt(canFdHandle);
        }
    }

    return;
}

static void CANFD_transCompleteInterrupt(CANFD_Handle handle)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    uint32_t                baseAddr;
    uint32_t                index, status, buffIndex;
    CANFD_Config           *config;
    CANFD_Object           *ptrCanFdObj = NULL;

    if(handle != NULL)
    {
        config      = (CANFD_Config*)handle;
        ptrCanFdObj = (CANFD_Object*)config->object;
        baseAddr    = ptrCanFdObj->regBaseAddress;
        status      = MCAN_getTxBufTransmissionStatus(baseAddr);

        /* Process all 32 Tx buffers */
        for(index = (uint32_t)0U; index < MCAN_MAX_TX_BUFFERS; index++)
        {
            buffIndex = ((uint32_t)1U << index);
            if(buffIndex == (status & buffIndex))
            {
                /* Get the message object pointer */
                ptrCanMsgObj = ptrCanFdObj->txMapping[index];

                if (ptrCanMsgObj != NULL)
                {
                    /* Increment the number of interrupts received */
                    ptrCanMsgObj->interruptsRxed++;

                    if(ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] == (uint8_t)1)
                    {
                        ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = (uint8_t)0;
                        if (config->attrs->operMode != CANFD_OPER_MODE_DMA)
                        {
                            /* Call the registered callback. */
                            CANFD_transferCallBack((CANFD_MsgObjHandle)ptrCanMsgObj, 
                                                    CANFD_Reason_TX_COMPLETION);
                        }
                    }
                }
            }
            status = (status & ~buffIndex);
            if (status == (uint32_t)0U)
            {
                break;
            }
        }
    }
}

static void CANFD_receiveBufferInterrupt(CANFD_Handle handle)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    uint32_t                baseAddr;
    uint32_t                index, status;
    MCAN_RxNewDataStatus    newDataStatus = {0};
    CANFD_Config           *config = NULL;
    CANFD_Object           *ptrCanFdObj = NULL;

    if(handle != NULL)
    {
        config      = (CANFD_Config*) handle;
        DebugP_assert(NULL_PTR != config->object);
        ptrCanFdObj = config->object;
        baseAddr    = ptrCanFdObj->regBaseAddress;

        /* Get the new data status */
        MCAN_getNewDataStatus(baseAddr, &newDataStatus);

        /* Clear NewData status to accept new messages */
        MCAN_clearNewDataStatus(baseAddr, &newDataStatus);

        /* Process the low 32 buffers */
        status = newDataStatus.statusLow;
        index = (uint32_t)0U;
        while (status != (uint32_t)0U)
        {
            if ((status & (uint32_t)1U) == (uint32_t)1U)
            {
                /* Get the message object pointer */
                ptrCanMsgObj = ptrCanFdObj->rxMapping[index];

                if(ptrCanMsgObj != NULL)
                {
                    /* Increment the number of interrupts received */
                    ptrCanMsgObj->interruptsRxed++;

                    /*  In non dma mode read the message and give callback.
                     *  In dma mode the msgram is read using dma. Only clear 
                     *  the new data status here. 
                     */
                    if (config->attrs->operMode != CANFD_OPER_MODE_DMA)
                    {
                        /* Read the pending data */
                        MCAN_readMsgRam(baseAddr, ptrCanMsgObj->rxMemType, 
                                        ptrCanMsgObj->rxElement, 0, 
                                        &ptrCanFdObj->rxBuffElem);

                        /* Copy the data */
                        (void)memcpy (ptrCanMsgObj->args, 
                                      (void*)&ptrCanFdObj->rxBuffElem.data, 
                                      ptrCanMsgObj->dataLength);

                        /* Call the registered callback. */
                        CANFD_transferCallBack((CANFD_MsgObjHandle)ptrCanMsgObj, 
                                                CANFD_Reason_RX);
                    }
                }
            }
            index++;
            status = (status >> (uint32_t)1U);
        }
    }
}

static void CANFD_transferCancelInterrupt(CANFD_Handle handle)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    uint32_t                baseAddr;
    uint32_t                index, status, buffIndex;
    CANFD_Config           *config = NULL;
    CANFD_Object           *ptrCanFdObj = NULL;

    if(handle != NULL)
    {
        config      = (CANFD_Config*) handle;
        DebugP_assert(NULL_PTR != config->object);
        ptrCanFdObj = config->object;
        if(ptrCanFdObj != NULL)
        {
            baseAddr    = ptrCanFdObj->regBaseAddress;
            status = MCAN_txBufCancellationStatus(baseAddr);

            /* Process all 32 Tx buffers */
            for(index = (uint32_t)0; index < MCAN_MAX_TX_BUFFERS; index++)
            {
                buffIndex = ((uint32_t)1U << index);
                if(buffIndex == (status & buffIndex))
                {
                    /* Get the message object pointer */
                    ptrCanMsgObj = ptrCanFdObj->txMapping[index];

                    if(ptrCanMsgObj != NULL)
                    {
                        /* Increment the number of interrupts received */
                        ptrCanMsgObj->interruptsRxed++;

                        if(ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] == (uint8_t)1)
                        {
                            ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = (uint8_t)0;

                            /* Call the registered callback. */
                            CANFD_transferCallBack((CANFD_MsgObjHandle)ptrCanMsgObj, 
                                                    CANFD_Reason_TX_CANCELED);
                        }
                    }
                }
                status = (status & ~buffIndex);
                if (status == (uint32_t)0U)
                {
                    break;
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is the registered interrupt 1 ISR for the CANFD Driver.
 *
 *  @param[in]  arg
 *      Argument which is registered with the OS while registering the ISR
 *
 *  \ingroup CANFD_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void CANFD_int1Isr (void* args)
{
    uint32_t            baseAddr;
    MCAN_ECCErrStatus   errStatusResp = {0};
    CANFD_ErrStatusResp errStatusInfo;
    CANFD_Config       *config = NULL;
    CANFD_Object       *ptrCanFdObj = NULL;
    CANFD_Handle        canFdHandle = NULL;

    if(args != NULL)
    {
        config = (CANFD_Config*) args;
        DebugP_assert(NULL_PTR != config->object);
        ptrCanFdObj = config->object;
        canFdHandle = config;

        /* Get the pointer to the CAN Driver Block */
        baseAddr = ptrCanFdObj->regBaseAddress;

        /* Increment the number of interrupts received */
        ptrCanFdObj->eccInterrupts++;

        /* Read ECC error status */
        MCAN_eccGetErrorStatus(baseAddr, &errStatusResp);
        if (errStatusResp.secErr == (uint32_t)1U)
        {
            MCAN_eccClearErrorStatus(baseAddr, CANFD_MCAN_ECC_ERR_TYPE_SEC);
        }
        if (errStatusResp.dedErr == (uint32_t)1U)
        {
            MCAN_eccClearErrorStatus(baseAddr, CANFD_MCAN_ECC_ERR_TYPE_DED);
        }

        /* Call the registered callback. */
        errStatusInfo.eccErrStatus.bit1 = errStatusResp.bit1;
        errStatusInfo.eccErrStatus.bit2 = errStatusResp.bit2;
        errStatusInfo.eccErrStatus.dedErr = errStatusResp.dedErr;
        errStatusInfo.eccErrStatus.row    = errStatusResp.row;
        errStatusInfo.eccErrStatus.secErr = errStatusResp.secErr;
        CANFD_errStatusCallBack(canFdHandle, CANFD_Reason_ECC_ERROR, &errStatusInfo);

        MCAN_eccWriteEOI(baseAddr, CANFD_MCAN_ECC_ERR_TYPE_SEC);
        MCAN_eccWriteEOI(baseAddr, CANFD_MCAN_ECC_ERR_TYPE_DED);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      Function initializes the CANFD driver instance with the specified hardware attributes.
 *      It resets and configures the MCAN module, sets up the Message RAM and ECC Aggregator.
 *      It configures the CANFD driver with the control parameters.
 *
 *  @param[in]  ptrCanFdMCB
 *      Pointer to the CANFD driver MCB
 *  @param[in]  configParams
 *      CAN module configuration parameters
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Handle to the CAN Driver
 *  @retval
 *      Error   -   NULL
 */

static int32_t CANFD_configMessageRAM(uint32_t baseAddr, 
                                      const CANFD_MCANMsgRAMCfgParams* configParams)
{
    MCAN_MsgRAMConfigParams msgRAMConfigParams;
    uint32_t                startAddr;
    int32_t                 status = SystemP_SUCCESS;

    /* Compute the start Address and populate the Message RAM configuration parameters */
    startAddr = (uint32_t)0U;

    if((configParams != NULL) && (baseAddr != (uint32_t)0U))
    {
        status = MCAN_configParamsLss(configParams->lss);
        status += MCAN_configParamsLse(configParams->lse);
        status += MCAN_configParamsTxBuffNum(configParams->txBufNum);
        status += MCAN_configParamsTxFifoSize(configParams->txFIFOSize);
        status += MCAN_configParamsTxEventFifoSize(configParams->txEventFIFOSize);
        status += MCAN_configParamsRxFifoSize(configParams->rxFIFO0size);
        status += MCAN_configParamsRxFifoSize(configParams->rxFIFO1size);
        status += MCAN_configParamsFifoOpMode(configParams->rxFIFO0OpMode);
        status += MCAN_configParamsFifoOpMode(configParams->rxFIFO1OpMode);

        /* Tx buffer configuration */
        msgRAMConfigParams.txStartAddr = startAddr;
        msgRAMConfigParams.txBufCnt  = configParams->txBufNum;
        msgRAMConfigParams.txFIFOCnt = configParams->txFIFOSize;
        msgRAMConfigParams.txBufMode = configParams->txBufMode;
        msgRAMConfigParams.txBufElemSize = (uint32_t)CANFD_MCANElemSize_64BYTES;

        /* Rx Buffer configuration */
        startAddr += (configParams->txBufNum * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
        startAddr += (configParams->txFIFOSize * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
        msgRAMConfigParams.rxBufStartAddr = startAddr;
        msgRAMConfigParams.rxBufElemSize = CANFD_MCANElemSize_64BYTES;

        /* 11-bit filter configuration */
        startAddr += (64U * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
        msgRAMConfigParams.flssa = startAddr;
        msgRAMConfigParams.lss = configParams->lss;

        /* 29-bit filter configuration */
        startAddr += ((configParams->lss + (uint32_t)1U) * MCAN_MSG_RAM_STD_ELEM_SIZE * (uint32_t)4U);
        msgRAMConfigParams.flesa = startAddr;
        msgRAMConfigParams.lse = configParams->lse;

        /* Rx FIFO 0 configuration */
        startAddr += ((configParams->lse + (uint32_t)1U) * MCAN_MSG_RAM_EXT_ELEM_SIZE * (uint32_t)4U);
        msgRAMConfigParams.rxFIFO0StartAddr = startAddr;
        msgRAMConfigParams.rxFIFO0Cnt = configParams->rxFIFO0size;
        msgRAMConfigParams.rxFIFO0WaterMark = configParams->rxFIFO0waterMark;
        msgRAMConfigParams.rxFIFO0ElemSize = CANFD_MCANElemSize_64BYTES;
        msgRAMConfigParams.rxFIFO0OpMode = configParams->rxFIFO0OpMode;

        /* Rx FIFO 1 configuration */
        startAddr += ((configParams->rxFIFO0size + (uint32_t)1U) * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * (uint32_t)4U);
        msgRAMConfigParams.rxFIFO1StartAddr = startAddr;
        msgRAMConfigParams.rxFIFO1Cnt = configParams->rxFIFO1size;
        msgRAMConfigParams.rxFIFO1WaterMark = configParams->rxFIFO0waterMark;
        msgRAMConfigParams.rxFIFO1ElemSize = CANFD_MCANElemSize_64BYTES;
        msgRAMConfigParams.rxFIFO1OpMode = configParams->rxFIFO1OpMode;

        /* Tx Event FIFO configuration */
        startAddr += ((configParams->rxFIFO1size + (uint32_t)1U) * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * (uint32_t)4U);
        msgRAMConfigParams.txEventFIFOStartAddr = startAddr;
        msgRAMConfigParams.txEventFIFOCnt = configParams->txFIFOSize;
        msgRAMConfigParams.txEventFIFOWaterMark = configParams->txEventFIFOWaterMark;

        /* Configure Message RAM */
        status += MCAN_msgRAMConfig(baseAddr, &msgRAMConfigParams);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static void CANFD_initCslMcanInitParams(const CANFD_OpenParams* configParams, 
                                        MCAN_InitParams *cslMcanInitPrms)
{
    if((configParams != NULL_PTR) && (cslMcanInitPrms != NULL_PTR))
    {
        cslMcanInitPrms->autoWkupEnable  = configParams->autoWkupEnable;
        cslMcanInitPrms->brsEnable       = configParams->brsEnable;
        cslMcanInitPrms->clkStopFAck     = configParams->clkStopFAck;
        cslMcanInitPrms->darEnable       = configParams->darEnable;
        cslMcanInitPrms->efbi            = configParams->efbi;
        cslMcanInitPrms->emulationEnable = configParams->emulationEnable;
        cslMcanInitPrms->emulationFAck   = configParams->emulationFAck;
        cslMcanInitPrms->fdMode          = configParams->fdMode;
        cslMcanInitPrms->pxhddisable     = configParams->pxhddisable;
        cslMcanInitPrms->tdcEnable       = configParams->tdcEnable;
        cslMcanInitPrms->txpEnable       = configParams->txpEnable;
        cslMcanInitPrms->wdcPreload      = configParams->wdcPreload;
        cslMcanInitPrms->wkupReqEnable   = configParams->wkupReqEnable;
        cslMcanInitPrms->tdcConfig.tdcf  = configParams->tdcConfig.tdcf;
        cslMcanInitPrms->tdcConfig.tdco  = configParams->tdcConfig.tdco;
    }
}

static void CANFD_initCslMcanConfigParams(const CANFD_OpenParams* configParams, 
                                          MCAN_ConfigParams *cslMcanConfigPrms)
{
    if((configParams != NULL_PTR) && (cslMcanConfigPrms != NULL_PTR))
    {
        cslMcanConfigPrms->asmEnable         = configParams->asmEnable;
        cslMcanConfigPrms->monEnable         = configParams->monEnable;
        cslMcanConfigPrms->timeoutCntEnable  = configParams->timeoutCntEnable;
        cslMcanConfigPrms->timeoutPreload    = configParams->timeoutPreload;
        cslMcanConfigPrms->timeoutSelect     = (uint32_t)configParams->timeoutSelect;
        cslMcanConfigPrms->tsPrescalar       = configParams->tsPrescalar;
        cslMcanConfigPrms->tsSelect          = configParams->tsSelect;
        cslMcanConfigPrms->filterConfig.anfe = configParams->filterConfig.anfe;
        cslMcanConfigPrms->filterConfig.anfs = configParams->filterConfig.anfs;
        cslMcanConfigPrms->filterConfig.rrfe = configParams->filterConfig.rrfe;
        cslMcanConfigPrms->filterConfig.rrfs = configParams->filterConfig.rrfs;
    }
}

static void CANFD_initCslMcanECCConfigParams(const CANFD_OpenParams* configParams, 
                                             MCAN_ECCConfigParams *cslMcanEccConfigPrms)
{
    if((configParams != NULL_PTR) && (cslMcanEccConfigPrms != NULL_PTR))
    {
        cslMcanEccConfigPrms->enable = configParams->eccConfig.enable;
        cslMcanEccConfigPrms->enableChk = configParams->eccConfig.enableChk;
        cslMcanEccConfigPrms->enableRdModWr = configParams->eccConfig.enableRdModWr;
    }
}

CANFD_Handle CANFD_open(uint32_t index, CANFD_OpenParams *openPrms)
{
    int32_t              status = SystemP_SUCCESS;
    CANFD_Config        *config = NULL;
    CANFD_Object        *obj    = NULL;
    const CANFD_Attrs   *attrs;
    HwiP_Params          hwiPrms0, hwiPrms1;
    CANFD_Handle         handle = NULL;

    /* Check index */
    if((index >= gCANFDConfigNum) && (openPrms == NULL))
    {
        status = SystemP_FAILURE;
        DebugP_assert(NULL_PTR != openPrms);
    }
    else
    {
        config = &gCanfdConfig[index];
    }

    DebugP_assert(NULL_PTR != gCANFDDrvObj.lock);
    status += SemaphoreP_pend(&gCANFDDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if((SystemP_SUCCESS == status) && (config != NULL_PTR))
    {
        obj = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        attrs = config->attrs;
        obj->openParams = openPrms;
        obj->regBaseAddress = attrs->baseAddr;
        if(attrs->operMode == CANFD_OPER_MODE_DMA)
        {
            obj->canfdDmaHandle = gCanfdDmaHandle[index];
            obj->canfdDmaChCfg  = gCanfdDmaChCfg[index];
        }
        obj->handle = (CANFD_Handle) config;
        handle = obj->handle;
    
        status += CANFD_configInstance(handle);
        if(status == SystemP_SUCCESS)
        {
            /* Create read transfer sync semaphore */
            status += SemaphoreP_constructBinary(&obj->readTransferSemObj, 0U);
            obj->readTransferSem = &obj->readTransferSemObj;

            /* Create write transfer sync semaphore */
            status += SemaphoreP_constructBinary(&obj->writeTransferSemObj, 0U);
            obj->writeTransferSem = &obj->writeTransferSemObj;

            /* Register interrupt */
            if(CANFD_OPER_MODE_INTERRUPT == attrs->operMode)
            {
                HwiP_Params_init(&hwiPrms0);
                hwiPrms0.intNum      = attrs->intrNum0;
                hwiPrms0.priority    = attrs->intrPriority;
                hwiPrms0.callback    = &CANFD_int0Isr;
                hwiPrms0.args        = (CANFD_Handle) handle;
                status += HwiP_construct(&obj->hwiObj0, &hwiPrms0);

                HwiP_Params_init(&hwiPrms1);
                hwiPrms1.intNum      = attrs->intrNum1;
                hwiPrms1.priority    = attrs->intrPriority;
                hwiPrms1.callback    = &CANFD_int1Isr;
                hwiPrms1.args        = (CANFD_Handle) handle;
                status += HwiP_construct(&obj->hwiObj1, &hwiPrms1);
            }
        }

        SemaphoreP_post(&gCANFDDrvObj.lockObj);

        /* Free-up resources in case of error */
        if (SystemP_SUCCESS != status)
        {
            CANFD_close((CANFD_Handle) config);
        }
    }

    return (handle);
}

void CANFD_close(CANFD_Handle handle)
{
    CANFD_Config        *config;
    CANFD_Object        *obj;
    int32_t              status = SystemP_FAILURE;

    if(NULL != handle)
    {
        config = (CANFD_Config *)handle;
        obj    = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        DebugP_assert(NULL_PTR != gCANFDDrvObj.lock);

        status = SemaphoreP_pend(&gCANFDDrvObj.lockObj, SystemP_WAIT_FOREVER);
        status += CANFD_deConfigInstance(handle);
        DebugP_assert(SystemP_SUCCESS == status);

        if(NULL != obj->readTransferSem)
        {
            SemaphoreP_destruct(&obj->readTransferSemObj);
            obj->readTransferSem = NULL;
        }
        if(NULL != obj->writeTransferSem)
        {
            SemaphoreP_destruct(&obj->writeTransferSemObj);
            obj->writeTransferSem = NULL;
        }
        if(NULL != obj->hwiHandle)
        {
            HwiP_destruct(&obj->hwiObj0);
            HwiP_destruct(&obj->hwiObj1);
            obj->hwiHandle = NULL;
        }

        SemaphoreP_post(&gCANFDDrvObj.lockObj);
    }

    return;
}

void CANFD_init(void)
{
    uint32_t       cnt;
    CANFD_Object  *obj;
    int32_t        status = SystemP_SUCCESS;

    /* Init each driver instance object */
    for (cnt = (uint32_t)0U; cnt < gCANFDConfigNum; cnt++)
    {
        /* initialize object varibles */
        obj = gCanfdConfig[cnt].object;
        DebugP_assert(NULL_PTR != obj);
        (void)memset(obj, 0, sizeof(CANFD_Object));
    }

    /* Create driver lock */
    status = SemaphoreP_constructMutex(&gCANFDDrvObj.lockObj);
    if(status == SystemP_SUCCESS)
    {
        gCANFDDrvObj.lock = &gCANFDDrvObj.lockObj;
    }

    return;
}

void CANFD_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gCANFDDrvObj.lock)
    {
        SemaphoreP_destruct(&gCANFDDrvObj.lockObj);
        gCANFDDrvObj.lock = NULL;
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      Function initializes the CANFD driver instance with the specified hardware attributes.
 *      It resets and configures the MCAN module, sets up the Message RAM and ECC Aggregator.
 *      It configures the CANFD driver with the control parameters.
 *
 *  @param[in]  handle
 *      CANFD handle.
 *  @retval
 *      Success  -   SystemP_SUCCESS
 *  @retval
 *      Failure  -   SystemP_FAILURE
 */

static int32_t CANFD_configInstance(CANFD_Handle handle)
{
    uint32_t                baseAddr, elapsedTicks, startTicks;
    MCAN_RxNewDataStatus    newDataStatus;
    int32_t                 status = SystemP_SUCCESS;
    MCAN_InitParams         cslMcanInitPrms;
    MCAN_ConfigParams       cslMcanConfigPrms;
    MCAN_ECCConfigParams    cslMcanEccConfigPrms;
    CANFD_Object           *ptrCanFdObj = NULL;
    CANFD_Config           *config;
    CANFD_MCANBitTimingParams  *bitTimingParams;
    CANFD_OptionTLV             optionTLV;

    if(handle == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = (CANFD_Config*) handle;
        ptrCanFdObj = (CANFD_Object *) config->object;
        bitTimingParams = (&config->attrs->CANFDMcanBitTimingParams);
        optionTLV.type  = (CANFD_Option)config->attrs->OptionTLVtype;
        status = MCAN_CheckRegBaseAddr(ptrCanFdObj->regBaseAddress);
        if (status == SystemP_SUCCESS)
        {
            baseAddr = ptrCanFdObj->regBaseAddress;

            /* Check if Message RAM initialization is done */
            startTicks = ClockP_getTicks();
            while((MCAN_isMemInitDone(baseAddr) != (uint32_t)TRUE) && 
                  (status == SystemP_SUCCESS))
            {
                elapsedTicks = ClockP_getTicks() - startTicks;
                if(elapsedTicks > MCAN_MEMINIT_TIMEOUT)
                {
                    status = SystemP_TIMEOUT;
                }
            }
            /* Set MCAN in SW initialization mode */
            status += CANFD_updateOpMode(baseAddr, MCAN_OPERATION_MODE_SW_INIT);

            /* Initialize the MCAN module */
            if (status == SystemP_SUCCESS)
            {
                CANFD_initCslMcanInitParams(ptrCanFdObj->openParams, &cslMcanInitPrms);
                status  = MCAN_init (baseAddr, &cslMcanInitPrms);
            }

            /* Configure the MCAN module */
            if (status == SystemP_SUCCESS)
            {
                CANFD_initCslMcanConfigParams(ptrCanFdObj->openParams, &cslMcanConfigPrms);
                status  = MCAN_config (baseAddr, &cslMcanConfigPrms);
            }

            /* Configure the Message RAM */
            if (status == SystemP_SUCCESS)
            {
                status  = CANFD_configMessageRAM(baseAddr, 
                                    &ptrCanFdObj->openParams->msgRAMConfig);
            }

            /* Initialize dma mode if the dma handle is not NULL */
            if (status == SystemP_SUCCESS)
            {
                if (ptrCanFdObj->canfdDmaHandle != NULL)
                {
                    status  = CANFD_dmaOpen(handle, ptrCanFdObj->canfdDmaChCfg);
                }
            }

            if (status == SystemP_SUCCESS)
            {
                /* Configure the ECC Aggregator */
                CANFD_initCslMcanECCConfigParams(ptrCanFdObj->openParams, 
                                                 &cslMcanEccConfigPrms);
                MCAN_eccConfig(baseAddr, &cslMcanEccConfigPrms);

                /* Clear all pending error flags and status */
                MCAN_clearIntrStatus(baseAddr, MCAN_INTR_MASK);
                newDataStatus.statusLow  = MCAN_NDAT1_CLEAR;
                newDataStatus.statusHigh = MCAN_NDAT2_CLEAR;
                MCAN_clearNewDataStatus(baseAddr, &newDataStatus);

                /* Set MCAN in opertional mode */
                status = CANFD_updateOpMode(baseAddr, MCAN_OPERATION_MODE_NORMAL);

                /* Initialize the datalength to DLC mapping */
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_0BYTES] = (uint8_t)0;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_1BYTES] = (uint8_t)1U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_2BYTES] = (uint8_t)2U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_3BYTES] = (uint8_t)3U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_4BYTES] = (uint8_t)4U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_5BYTES] = (uint8_t)5U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_6BYTES] = (uint8_t)6U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_7BYTES] = (uint8_t)7U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_8BYTES] = (uint8_t)8U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_12BYTES] = (uint8_t)12U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_16BYTES] = (uint8_t)16U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_20BYTES] = (uint8_t)20U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_24BYTES] = (uint8_t)24U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_32BYTES] = (uint8_t)32U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_48BYTES] = (uint8_t)48U;
                ptrCanFdObj->mcanDataSize[MCAN_DATA_SIZE_64BYTES] = (uint8_t)64U;

                /* Initialize the CAN driver state */
                ptrCanFdObj->state = CANFD_DRIVER_STATE_STARTED;

                /* Configure the Bit timing */
                if (status == SystemP_SUCCESS)
                {
                    status = CANFD_configBitTime (handle, bitTimingParams);
                }
                /* 
                * Configure the driver options. Option info in TLV format 
                * which is configures the driver 
                */
                if (status == SystemP_SUCCESS)
                {
                    optionTLV.length = sizeof(CANFD_MCANLoopbackCfgParams);
                    optionTLV.value  = (void*) (&config->attrs->CANFDMcanloopbackParams);
                    status = CANFD_setOptions(handle, &optionTLV);
                }
            }
        }
    }
    return status;
}

/**
 *  @b Description
 *  @n
 *      Function closes the CANFD driver instance and cleanups all the memory allocated by the CANFD driver.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t CANFD_deConfigInstance(CANFD_Handle handle)
{
    uint32_t            index, baseAddr;
    int32_t             retVal = SystemP_SUCCESS;
    CANFD_Object       *ptrCanFdObj = NULL;
    CANFD_Config       *config;

    if(handle != NULL)
    {
        config = (CANFD_Config*) handle;
        ptrCanFdObj = (CANFD_Object *) config->object;
        baseAddr = ptrCanFdObj->regBaseAddress;

        /* Set MCAN in SW initialization mode */
        retVal = CANFD_updateOpMode(baseAddr, MCAN_OPERATION_MODE_SW_INIT);

        /* Disable the MCAN interrupts */
        MCAN_enableIntr(baseAddr, MCAN_INTR_MASK, 0U);
        MCAN_enableIntrLine(baseAddr, MCAN_INTR_LINE_NUM_0, 0U);

        /* Update the driver state */
        ptrCanFdObj->state = CANFD_DRIVER_STATE_STOPPED;

        /* Delete the message objects */
        for (index = (uint32_t)0; index < MCAN_MAX_MSG_OBJECTS; index++)
        {
            if (ptrCanFdObj->msgObjectHandle[index] != NULL)
            {
                ptrCanFdObj->msgObjectHandle[index] = NULL;
            }
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function configures the bit time parameters for the CANFD module.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[in]  bitTimeParams
 *      Bit time configuration parameters
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - SystemP_SUCCESS.
 *  @retval
 *      Error       - SystemP_FAILURE.
 */
int32_t CANFD_configBitTime(CANFD_Handle handle, const CANFD_MCANBitTimingParams* bitTimeParams)
{
    MCAN_BitTimingParams    mcanBitTimingParams;
    uint32_t                baseAddr;
    int32_t                 retVal = SystemP_SUCCESS;
    CANFD_Config           *config;
    CANFD_Object           *ptrCanFdObj = NULL;

    if((handle == NULL) || (bitTimeParams == NULL))
    {
        retVal = SystemP_FAILURE;
    }
    if(retVal == SystemP_SUCCESS)
    {
        config = (CANFD_Config*) handle;
        ptrCanFdObj = (CANFD_Object *) config->object;

        baseAddr = ptrCanFdObj->regBaseAddress;

        /* Set MCAN in SW initialization mode */
        retVal = CANFD_updateOpMode(baseAddr, MCAN_OPERATION_MODE_SW_INIT);

        /* Calculate the MCAN bit timing parameters */
        mcanBitTimingParams.nomRatePrescalar   = bitTimeParams->nomBrp;
        mcanBitTimingParams.nomTimeSeg1        = bitTimeParams->nomPropSeg + bitTimeParams->nomPseg1;
        mcanBitTimingParams.nomTimeSeg2        = bitTimeParams->nomPseg2;
        mcanBitTimingParams.nomSynchJumpWidth  = bitTimeParams->nomSjw;

        mcanBitTimingParams.dataRatePrescalar  = bitTimeParams->dataBrp;
        mcanBitTimingParams.dataTimeSeg1       = bitTimeParams->dataPropSeg + bitTimeParams->dataPseg1;
        mcanBitTimingParams.dataTimeSeg2       = bitTimeParams->dataPseg2;
        mcanBitTimingParams.dataSynchJumpWidth = bitTimeParams->dataSjw;

        /* Set the bit timing values */
        retVal += MCAN_setBitTime(baseAddr, &mcanBitTimingParams);

        /* Set MCAN in opertional mode */
        retVal += CANFD_updateOpMode(baseAddr, MCAN_OPERATION_MODE_NORMAL);
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function configures the receive or transmit message object.
 *      It also enables Tx completion and Tx cancelation interrupts.
 *      The callback function will be invoked on data transmit complete 
 *      for transmit message objects OR upon receiving data for receive
 *      message objects. The application MUST then call CANFD_read() 
 *      API to process the received data.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[in]  msgObjectParams
 *      Message Object configuration parameters
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Handle to the message object.
 *  @retval
 *      Error   -   SystemP_FAILURE.
 */
int32_t CANFD_createMsgObject(CANFD_Handle handle, 
                              CANFD_MessageObject* ptrCanMsgObj)
{
    uint32_t                    baseAddr;
    MCAN_StdMsgIDFilterElement  stdMsgIdFilter;
    MCAN_ExtMsgIDFilterElement  extMsgIdFilter;
    uint32_t                    i = 0U;
    int32_t                     retVal = SystemP_SUCCESS;
    CANFD_Config               *config;
    CANFD_Object               *ptrCanFdObj = NULL;

    if ((handle != NULL) && (ptrCanMsgObj != NULL))
    {
        config = (CANFD_Config*) handle;
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object *) config->object;
        baseAddr    = ptrCanFdObj->regBaseAddress;

        /* Save the specified parameters */
        ptrCanMsgObj->canfdHandle = (CANFD_Config*) handle;
        /* initialize the below varible to zero */
        ptrCanMsgObj->messageProcessed = 0U;
        ptrCanMsgObj->interruptsRxed = 0U;
        ptrCanMsgObj->dmaEventNo = 0U;
        if(ptrCanMsgObj->msgIdType == CANFD_MCANXidType_29_BIT)
        {
            ptrCanMsgObj->filterConfig = (MCAN_ExtMsgIDFilterElement*)config->attrs->filterConfig;
        }
        else
        {
            ptrCanMsgObj->filterConfig = (MCAN_StdMsgIDFilterElement*)config->attrs->filterConfig;
        }

        /* Loop to find a free message object handle */
        for (i = (uint32_t)0U; i < MCAN_MAX_MSG_OBJECTS; i++)
        {
            if (ptrCanFdObj->msgObjectHandle[i] == NULL)
            {
                break;
            }
        }
        if (i == MCAN_MAX_MSG_OBJECTS)
        {
            /* Error: Unable to allocate the memory */
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Store the message object handle for book keeping */
            ptrCanMsgObj->messageObjNum = i;
            ptrCanFdObj->msgObjectHandle[i] = ptrCanMsgObj;
        }

        /* Configure the Tx Message Id */
        if ((retVal == SystemP_SUCCESS) && (ptrCanMsgObj->direction == CANFD_Direction_TX))
        {
            uint32_t txElemStart, txElemEnd;
            if (ptrCanFdObj->canfdDmaHandle != NULL)
            {
                /* dma mode enabled in can driver, tx elements 0 to 
                   MCAN_MAX_TX_DMA_BUFFERS are reserved for dma mode. 
                 */
                if (config->attrs->operMode == CANFD_OPER_MODE_DMA)
                {
                    /* msg obj created for dma mode, allocate from 0 
                       to MCAN_MAX_TX_DMA_BUFFERS 
                     */
                    txElemStart = (uint32_t)0;
                    txElemEnd = MCAN_MAX_TX_DMA_BUFFERS;
                }
                else
                {
                    /* msg obj created for non dma mode, 
                       allocate after MCAN_MAX_TX_DMA_BUFFERS 
                     */
                    txElemStart = MCAN_MAX_TX_DMA_BUFFERS;
                    txElemEnd = MCAN_MAX_TX_MSG_OBJECTS;
                }
            }
            else
            {
                /* dma mode not enabled in can driver,
                   any available tx buffer elements can be allocated. 
                 */
                txElemStart = (uint32_t)0;
                txElemEnd = MCAN_MAX_TX_MSG_OBJECTS;
            }

            /* Get a free Tx element */
            for (i = txElemStart; i < txElemEnd; i++)
            {
                if (ptrCanFdObj->txMapping[i] == NULL)
                {
                    break;
                }
            }
            if (i == MCAN_MAX_TX_MSG_OBJECTS)
            {
                /* Error: Unable to allocate the memory */
                retVal = SystemP_FAILURE;
                ptrCanFdObj->msgObjectHandle[ptrCanMsgObj->messageObjNum] = NULL;
            }
            else
            {
                /* Store the tx to message object handle mapping */
                ptrCanFdObj->txMapping[i] = ptrCanMsgObj;
                ptrCanMsgObj->txElement = i;

                if (config->attrs->operMode == CANFD_OPER_MODE_DMA)
                {
                    /* Initialize DMA channels for the TX message object */
                    retVal = CANFD_createDmaTxMsgObject(ptrCanFdObj, ptrCanMsgObj);
                }

                if(retVal == SystemP_SUCCESS)
                {
                    /* Enable TX Buffer interrupt */
                    retVal = MCAN_txBufTransIntrEnable(baseAddr, i, 1U);
                }
                if(retVal == SystemP_SUCCESS)
                {
                    /* Enable Tx cancelation interrupt*/
                    retVal = MCAN_txBufCancellationIntrEnable(baseAddr, i, 1U);
                }
            }
        }

        /* Configure the Rx Message Id */
        if ((retVal == SystemP_SUCCESS) && (ptrCanMsgObj->direction == CANFD_Direction_RX))
        {
            /* Get a free Rx element */
            for (i = (uint32_t)0U; i < MCAN_MAX_RX_MSG_OBJECTS; i++)
            {
                if (ptrCanFdObj->rxMapping[i] == NULL)
                {
                    break;
                }
            }
            if (i == MCAN_MAX_RX_MSG_OBJECTS)
            {
                /* Error: Unable to allocate the memory */
                retVal = SystemP_FAILURE;
                ptrCanFdObj->msgObjectHandle[ptrCanMsgObj->messageObjNum] = NULL;
            }
            else
            {
                /* Store the rx to message object handle mapping */
                ptrCanFdObj->rxMapping[i] = ptrCanMsgObj;
                ptrCanMsgObj->rxElement = i;

                if (config->attrs->operMode == CANFD_OPER_MODE_DMA)
                {
                    /* Initialize DMA channels for the RX message object */
                    retVal = CANFD_createDmaRxMsgObject(ptrCanFdObj, ptrCanMsgObj);
                }

                if(retVal != SystemP_SUCCESS)
                {
                    retVal = SystemP_FAILURE;
                }

                /* Add the filter to message RAM */
                if (ptrCanMsgObj->msgIdType == CANFD_MCANXidType_11_BIT)
                {
                    stdMsgIdFilter.sfid1 = ptrCanMsgObj->startMsgId & STD_MSGID_MASK;
                    stdMsgIdFilter.sfid2 = i;
                    if (config->attrs->operMode == CANFD_OPER_MODE_DMA)
                    {
                        /* Configure the filter element number to be used for this msgId.
                        sfid2[8:6] has the dma event number configuration. */
                        stdMsgIdFilter.sfid2 |= CANFD_getFilterEventConfig(ptrCanMsgObj->dmaEventNo);
                    }

                    /* Store the message in rx buffer */
                    stdMsgIdFilter.sfec = ((MCAN_StdMsgIDFilterElement*)(ptrCanMsgObj->filterConfig))->sfec;
                    stdMsgIdFilter.sft  = ((MCAN_StdMsgIDFilterElement*)(ptrCanMsgObj->filterConfig))->sft;
                    MCAN_addStdMsgIDFilter(baseAddr, i, &stdMsgIdFilter);
                }
                else
                {
                    extMsgIdFilter.efid1 = ptrCanMsgObj->startMsgId & XTD_MSGID_MASK;
                    extMsgIdFilter.efid2 = i;
                    if (config->attrs->operMode == CANFD_OPER_MODE_DMA)
                    {
                        /* Configure the filter element number to be used for this msgId.
                        efid2[8:6] has the dma event number configuration. */
                        extMsgIdFilter.efid2 |= CANFD_getFilterEventConfig(ptrCanMsgObj->dmaEventNo);
                    }

                    /* Store the message in rx buffer */
                    extMsgIdFilter.efec = ((MCAN_ExtMsgIDFilterElement*)(ptrCanMsgObj->filterConfig))->efec;
                    extMsgIdFilter.eft  = ((MCAN_ExtMsgIDFilterElement*)(ptrCanMsgObj->filterConfig))->eft;
                    MCAN_addExtMsgIDFilter(baseAddr, i, &extMsgIdFilter);
                }
            }
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function configures a receive message objects for a range of message 
 *      Identifiersmessage object. It also enables Rx interrupts.
 *      The callback function will be invoked upon receiving data for receive
 *      message objects. The application MUST then call CANFD_read()
 *      API to process the received data.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[in]  msgObjectParams
 *      Message Object configuration parameters
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Handle to the message object.
 *  @retval
 *      Error   -   SystemP_FAILURE.
 */
int32_t CANFD_createRxRangeMsgObject(CANFD_Handle handle, 
                                     CANFD_MessageObject* ptrCanMsgObj)
{
    uint32_t                    baseAddr, i = 0U;
    MCAN_StdMsgIDFilterElement  stdMsgIdFilter;
    MCAN_ExtMsgIDFilterElement  extMsgIdFilter;
    int32_t                     retVal = SystemP_SUCCESS;
    CANFD_Config               *config = NULL;
    CANFD_Object               *ptrCanFdObj = NULL;

    if ((handle == NULL) || (ptrCanMsgObj == NULL)  || 
        (ptrCanMsgObj->startMsgId > ptrCanMsgObj->endMsgId))
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        config      = (CANFD_Config *) handle;
        ptrCanFdObj = config->object;
        baseAddr    = ptrCanFdObj->regBaseAddress;

        /* Save the specified parameters */
        ptrCanMsgObj->canfdHandle->object = ptrCanFdObj;
        {
            /* Save the specified parameters */
            ptrCanMsgObj->canfdHandle->object = ptrCanFdObj;
            ptrCanMsgObj->direction = CANFD_Direction_RX;

            for (i = (uint32_t)0U; i < MCAN_MAX_MSG_OBJECTS; i++)
            {
                if (ptrCanFdObj->msgObjectHandle[i] == NULL)
                {
                    break;
                }
            }
            if (i == MCAN_MAX_MSG_OBJECTS)
            {
                /* Error: Unable to allocate the memory */
                retVal = SystemP_FAILURE;
            }
            else
            {
                /* Store the message object handle for book keeping */
                ptrCanMsgObj->messageObjNum = i;
                ptrCanFdObj->msgObjectHandle[i] = ptrCanMsgObj;
            }

            /* Configure the Rx Message Id */
            if (retVal == SystemP_SUCCESS)
            {
                /* Get a free Rx element */
                for (i = (uint32_t)0U; i < MCAN_MAX_RX_MSG_OBJECTS; i++)
                {
                    if (ptrCanFdObj->rxMapping[i] == NULL)
                    {
                        break;
                    }
                }
                if (i == MCAN_MAX_RX_MSG_OBJECTS)
                {
                    /* Error: Unable to allocate the memory */
                    retVal = SystemP_FAILURE;
                    ptrCanFdObj->msgObjectHandle[ptrCanMsgObj->messageObjNum] = NULL;
                }
                else
                {
                    /* Store the rx to message object handle mapping */
                    ptrCanFdObj->rxMapping[i] = ptrCanMsgObj;
                    ptrCanMsgObj->rxElement = i;

                    /* Add the filter to message RAM */
                    if (ptrCanMsgObj->msgIdType == CANFD_MCANXidType_11_BIT)
                    {
                        stdMsgIdFilter.sfid1 = ptrCanMsgObj->startMsgId & STD_MSGID_MASK;
                        stdMsgIdFilter.sfid2 = ptrCanMsgObj->endMsgId & STD_MSGID_MASK;

                        /* Store the message in FIFO */
                        stdMsgIdFilter.sfec = (ptrCanFdObj->useFifoNum + (uint32_t)1U);
                        stdMsgIdFilter.sft = (uint32_t)0;
                        MCAN_addStdMsgIDFilter(baseAddr, i, &stdMsgIdFilter);
                    }
                    else
                    {
                        extMsgIdFilter.efid1 = ptrCanMsgObj->startMsgId & XTD_MSGID_MASK;
                        extMsgIdFilter.efid2 = ptrCanMsgObj->endMsgId & XTD_MSGID_MASK;

                        /* Store the message in FIFO */
                        extMsgIdFilter.efec = (ptrCanFdObj->useFifoNum + (uint32_t)1U);
                        extMsgIdFilter.eft = (uint32_t)0U;
                        MCAN_addExtMsgIDFilter(baseAddr, i, &extMsgIdFilter);
                    }

                    /* Toggle the FIFO number for the next message Id */
                    ptrCanFdObj->useFifoNum = (uint32_t)1U - ptrCanFdObj->useFifoNum;
                }
            }
        }
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function deletes a message object.
 *
 *  @param[in]  handle
 *      Handle to the message object
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - SystemP_SUCCESS.
 *  @retval
 *      Error       - SystemP_FAILURE.
 */
int32_t CANFD_deleteMsgObject(CANFD_MsgObjHandle handle)
{
    CANFD_MessageObject*  ptrCanMsgObj;
    CANFD_Object*         ptrCanFdObj;
    int32_t               retVal = SystemP_SUCCESS;
    CANFD_Config         *config;
    uint32_t              index = 0U;

    if(handle != NULL)
    {
        ptrCanMsgObj = (CANFD_MessageObject*)handle;
        config       = ptrCanMsgObj->canfdHandle;
        ptrCanFdObj  = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;

        if (ptrCanFdObj == NULL)
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            ptrCanFdObj->msgObjectHandle[ptrCanMsgObj->messageObjNum] = NULL;

            if ((ptrCanMsgObj->direction == CANFD_Direction_TX) && 
                (MCAN_MAX_TX_MSG_OBJECTS > ptrCanMsgObj->txElement))
            {
                for (index = (uint32_t)0; index < MCAN_MAX_TX_MSG_OBJECTS; index++)
                {
                    /* Clear the tx to message object handle mapping */
                    ptrCanFdObj->txMapping[index] = NULL;
                }

                if (config->attrs->operMode == CANFD_OPER_MODE_DMA)
                {
                    /* De-Initialize DMA channels for the TX message object */
                    retVal = CANFD_deleteDmaTxMsgObject(ptrCanFdObj, ptrCanMsgObj);
                }
            }

            if ((ptrCanMsgObj->direction == CANFD_Direction_RX) && 
                (MCAN_MAX_RX_MSG_OBJECTS > ptrCanMsgObj->rxElement))
            {
                for (index = (uint32_t)0; index < MCAN_MAX_RX_MSG_OBJECTS; index++)
                {
                    /* Clear the rx to message object handle mapping */
                    ptrCanFdObj->rxMapping[index] = NULL;
                }

                if (config->attrs->operMode == CANFD_OPER_MODE_DMA)
                {
                    /* De-Initialize DMA channels for the RX message object */
                    retVal = CANFD_deleteDmaRxMsgObject(ptrCanFdObj, ptrCanMsgObj);
                }
            }
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function used by the application to transmit data.
 *
 *  @param[in]  handle
 *      Handle to the message object
 *  @param[in]  id
 *      Message Identifier
 *  @param[in]  frameType
 *      Frame type - Classic or FD
 *  @param[in]  data
 *      Data to be transmitted
 *  @param[in]  numMsgs
 *      Number of msgs to be transmitted Only applicale in DMA Mode for other pass 0.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - SystemP_SUCCESS.
 *  @retval
 *      Error       - SystemP_FAILURE.
 */
int32_t CANFD_write(CANFD_MsgObjHandle txMsgHandle, uint32_t id, 
                    CANFD_MCANFrameType frameType, uint32_t numMsgs, 
                    const uint8_t* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object           *ptrCanFdObj;
    int32_t                 retVal = SystemP_FAILURE;
    const CANFD_Attrs      *attrs;
    CANFD_Config           *config = NULL;
    uint32_t                dataLength;

    if(NULL != txMsgHandle)
    {
        /* Get the message object pointer */
        ptrCanMsgObj = (CANFD_MessageObject*)txMsgHandle;
        DebugP_assert(NULL_PTR != ptrCanMsgObj);
        dataLength = ptrCanMsgObj->dataLength;
        if ((data == NULL_PTR) || (dataLength < (uint32_t)1U) || 
            (dataLength > (uint32_t)64U))
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Get the pointer to the CAN Driver Block */
            config = (CANFD_Config*)ptrCanMsgObj->canfdHandle;
            ptrCanFdObj = (CANFD_Object*)config->object;
            attrs = config->attrs;
            if(ptrCanFdObj != NULL)
            {
                if((CANFD_OPER_MODE_INTERRUPT == attrs->operMode) ||
                    (CANFD_OPER_MODE_DMA == attrs->operMode))
                {
                    if(CANFD_OPER_MODE_INTERRUPT == attrs->operMode)
                    {
                        retVal = CANFD_writeIntr(ptrCanMsgObj, id, frameType, (uint8_t*)data);
                    }
                    else
                    {
                        retVal = CANFD_writeDma(ptrCanMsgObj, id, frameType, numMsgs, data);
                    }
                    if (retVal == SystemP_SUCCESS)
                    {
                        if (ptrCanFdObj->openParams->transferMode == CANFD_TRANSFER_MODE_BLOCKING)
                        {
                            /* Block on transferSem till the transfer completion. */
                            DebugP_assert(NULL_PTR != ptrCanFdObj->writeTransferSem);
                            retVal = SemaphoreP_pend(&ptrCanFdObj->writeTransferSemObj, SystemP_WAIT_FOREVER);
                            if (retVal != SystemP_SUCCESS)
                            {
                                retVal = CANFD_deConfigInstance((CANFD_Handle)config);
                            }
                        }
                    }
                }
                else
                {
                    retVal = CANFD_writePoll(ptrCanMsgObj, id, frameType, (uint8_t*)data);
                }
            }
        }
    }
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function used by the application to cancel a pending data transmit.
 *
 *  @param[in]  handle
 *      Handle to the message object
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - SystemP_SUCCESS.
 *  @retval
 *      Error       - SystemP_FAILURE.
 */
int32_t CANFD_writeCancel(CANFD_MsgObjHandle handle)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;
    uint32_t                baseAddr;
    CANFD_Config           *config;

    /* Get the message object pointer */
    if(handle != NULL)
    {
        ptrCanMsgObj = (CANFD_MessageObject*)handle;
        config      = ptrCanMsgObj->canfdHandle;
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;

        if (ptrCanFdObj == NULL)
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            baseAddr = ptrCanFdObj->regBaseAddress;

            /* Cancel the pending transmit */
            retVal = MCAN_txBufCancellationReq(baseAddr, ptrCanMsgObj->txElement);

            if ((retVal == SystemP_SUCCESS) && (config->attrs->operMode == CANFD_OPER_MODE_DMA))
            {
                retVal = CANFD_cancelDmaTx(ptrCanFdObj, ptrCanMsgObj);
            }
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function is used by the application to get the CAN message from message RAM
 *      using a receive message object. 
 *      NOTE: This API must ONLY be called from the callback context.
 *
 *  @param[in]  rxMsgHandle
 *      Handle to the message object
 *  @param[out]  numMsgs
 *      Number of message count. Applicale only for DMA Mode, for other modes, provide 0.
 *  @param[out]  data
 *      Pointer to the receiver's buffer, where read data will be stored.
 *  @param[out]  errCode
 *      Error code populated on error
 *  @retval
 *      Success     - SystemP_SUCCESS.
 *  @retval
 *      Error       - SystemP_FAILURE.
 */
int32_t CANFD_read(CANFD_MsgObjHandle rxMsgHandle, uint32_t numMsgs, uint8_t* data)
{
    CANFD_MessageObject    *ptrCanMsgObj;
    CANFD_Object           *ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;
    const CANFD_Attrs      *attrs;
    CANFD_Config           *config = NULL;
    uint32_t                dataLength;

    if(NULL != rxMsgHandle)
    {
        /* Get the message object pointer */
        ptrCanMsgObj = (CANFD_MessageObject*)rxMsgHandle;
        dataLength = ptrCanMsgObj->dataLength;

        if (((dataLength < (uint32_t)1U) || (dataLength > (uint32_t)64U)) || (data == NULL))
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            config = (CANFD_Config*)ptrCanMsgObj->canfdHandle;
            DebugP_assert(NULL != config);
            ptrCanFdObj = (CANFD_Object*)config->object;
            attrs = config->attrs;
            if((ptrCanFdObj != NULL) && (attrs != NULL))
            {
                if((CANFD_OPER_MODE_INTERRUPT == attrs->operMode) ||
                   (CANFD_OPER_MODE_DMA == attrs->operMode))
                {
                    if(CANFD_OPER_MODE_INTERRUPT == attrs->operMode)
                    {
                        retVal = CANFD_readIntr(ptrCanMsgObj, data);
                    }
                    else
                    {
                        retVal = CANFD_readDma(ptrCanMsgObj, numMsgs, data);
                    }
                    if (retVal == SystemP_SUCCESS)
                    {
                        if (ptrCanFdObj->openParams->transferMode == CANFD_TRANSFER_MODE_BLOCKING)
                        {
                            /* Block on transferSem till the read completion. */
                            DebugP_assert(NULL_PTR != ptrCanFdObj->readTransferSem);
                            retVal += SemaphoreP_pend(&ptrCanFdObj->readTransferSemObj, SystemP_WAIT_FOREVER);
                            if (retVal != SystemP_SUCCESS)
                            {
                                retVal = CANFD_deConfigInstance((CANFD_Handle)config);
                            }
                        }
                    }
                }
                else
                {
                    retVal = CANFD_readPoll(ptrCanMsgObj, data);
                }
            }
        }
    }
    
    return retVal;
}

static int32_t CANFD_readIntr(CANFD_MsgObjHandle rxMsgHandle, uint8_t* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;
    uint32_t                dataLength, baseAddr;
    CANFD_Attrs            *attrs = NULL;
    CANFD_Config           *config = NULL;
    uint32_t                index;

    if(rxMsgHandle != NULL)
    {
        ptrCanMsgObj = (CANFD_MessageObject*)rxMsgHandle;
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
        baseAddr = ptrCanFdObj->regBaseAddress;
        config = ptrCanMsgObj->canfdHandle;
        attrs  = config->attrs;
        DebugP_assert(NULL_PTR != attrs);

        MCAN_enableIntr(baseAddr, MCAN_INTR_MASK_RX, (uint32_t)TRUE);
        MCAN_enableIntr(baseAddr, MCAN_INTR_SRC_RES_ADDR_ACCESS, (uint32_t)FALSE);
        /* Select Interrupt Line 0 */
        MCAN_selectIntrLine(baseAddr, MCAN_INTR_MASK_RX, MCAN_INTR_LINE_NUM_0);
        /* Enable Interrupt Line */
        MCAN_enableIntrLine(baseAddr, MCAN_INTR_LINE_NUM_0, (uint32_t)TRUE);

        if(attrs->CANFDMcanloopbackParams.mode == CANFD_MCANLoopBackMode_EXTERNAL)
        {
            /* Compute the DLC value */
            for(index = (uint32_t)0U; index < (uint32_t)16U; index++)
            {
                if((uint8_t)ptrCanMsgObj->dataLength <= ptrCanFdObj->mcanDataSize[index])
                {
                    ptrCanFdObj->rxBuffElem.dlc = index;
                    break;
                }
            }
            
            /* Get the data length from DLC */
            dataLength = (uint32_t)ptrCanFdObj->mcanDataSize[ptrCanFdObj->rxBuffElem.dlc];
            retVal = CANFD_isDataSizeValid(ptrCanMsgObj->dataLength);
            /*
            * Check if the size of buffer is large enough to hold the received data.
            * If yes, store the data in buffer and update ptrDataLength to the actual
            * size of received data or else, return error. This is to prevent buffer
            * overflow if the buffer size is not sufficient.
            */
            if((ptrCanMsgObj->dataLength >= dataLength) && (retVal != SystemP_FAILURE))
            {
                ptrCanMsgObj->dataLength = dataLength;
                /* Copy the data */
                (void)memcpy ((void *)data, 
                              ptrCanFdObj->rxBuffElem.data, 
                              ptrCanMsgObj->dataLength);

                /* Increment the stats */
                ptrCanMsgObj->messageProcessed++;
            }
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

static int32_t CANFD_readPollProcessBuff(CANFD_MessageObject *ptrCanfdMsgObj, 
                                         uint8_t *data)
{
    uint32_t                baseAddr;
    uint32_t                index, status;
    int32_t                 retVal = SystemP_SUCCESS;
    MCAN_RxNewDataStatus    newDataStatus = {0};
    CANFD_Object*           ptrCanFdObj;
    CANFD_Config           *config;
    CANFD_MessageObject    *ptrCanMsgObj = ptrCanfdMsgObj;

    if(ptrCanMsgObj != NULL)
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
        config      = ptrCanMsgObj->canfdHandle;
        baseAddr    = ptrCanFdObj->regBaseAddress;
        /* Get the new data status */
        MCAN_getNewDataStatus(baseAddr, &newDataStatus);
        /* Clear NewData status to accept new messages */
        MCAN_clearNewDataStatus(baseAddr, &newDataStatus);

        /* Process the low 32 buffers */
        status = newDataStatus.statusLow;
        index = (uint32_t)0U;
        while (status != (uint32_t)0U)
        {
            if ((status & (uint32_t)1U) == (uint32_t)1U)
            {
                /* Get the message object pointer */
                ptrCanMsgObj = ptrCanFdObj->rxMapping[index];

                /* Increment the number of interrupts received */
                ptrCanMsgObj->interruptsRxed++;

                /* In non dma mode read the message and give callback.
                In dma mode the msgram is read using dma. Only clear the new data status here. */
                if (config->attrs->operMode != CANFD_OPER_MODE_DMA)
                {
                    /* Read the pending data */
                    MCAN_readMsgRam(baseAddr, ptrCanMsgObj->rxMemType, 
                                    ptrCanMsgObj->rxElement, 0, 
                                    &ptrCanFdObj->rxBuffElem);

                    /* Copy the data */
                    (void)memcpy (ptrCanMsgObj->args, 
                                  (void*)&ptrCanFdObj->rxBuffElem.data, 
                                  ptrCanMsgObj->dataLength);
                }
            }
            index++;
            status = (status >> (uint32_t)1U);
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

static int32_t CANFD_readPollProcessFIFO(CANFD_MessageObject* ptrCanfdMsgObj, 
                                         uint32_t fifoNum)
{
    int32_t retVal =        SystemP_SUCCESS;
    uint32_t                baseAddr;
    MCAN_RxFIFOStatus       fifoStatus = {0};
    CANFD_Object*           ptrCanFdObj = NULL;
    CANFD_MessageObject    *ptrCanMsgObj = ptrCanfdMsgObj;

    if(ptrCanMsgObj != NULL)
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
        baseAddr    = ptrCanFdObj->regBaseAddress;
        /* Get the FIFO status */
        fifoStatus.num = (uint32_t)fifoNum;
        MCAN_getRxFIFOStatus(baseAddr, &fifoStatus);

        MCAN_readMsgRam(baseAddr, MCAN_MEM_TYPE_FIFO, 
                        fifoStatus.getIdx, (uint32_t)fifoNum, 
                        &ptrCanFdObj->rxBuffElem);

        /* Copy the data */
        (void)memcpy (ptrCanMsgObj->args, 
                    (void*)&ptrCanFdObj->rxBuffElem.data, 
                    ptrCanMsgObj->dataLength);

        if(ptrCanFdObj->rxBuffElem.fidx < MCAN_MAX_RX_MSG_OBJECTS)
        {
            /* Get the message object pointer */
            ptrCanMsgObj = ptrCanFdObj->rxMapping[ptrCanFdObj->rxBuffElem.fidx];

            if(ptrCanMsgObj != NULL)
            {
                /* Increment the number of interrupts received */
                ptrCanMsgObj->interruptsRxed++;

                /* Acknowledge the data read */
                retVal += MCAN_writeRxFIFOAck(baseAddr, 
                                            (uint32_t)fifoNum, 
                                            fifoStatus.getIdx);

                MCAN_getRxFIFOStatus(baseAddr, &fifoStatus);
            }
            else
            {
                retVal = SystemP_FAILURE;
            }
        }
        else
        {
            retVal = SystemP_FAILURE;
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

static int32_t CANFD_readPoll(CANFD_MsgObjHandle rxMsgHandle, uint8_t* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    int32_t                 retVal = SystemP_SUCCESS;

    if(rxMsgHandle != NULL_PTR)
    {
        ptrCanMsgObj = (CANFD_MessageObject*)rxMsgHandle;

        if(ptrCanMsgObj->rxMemType == MCAN_MEM_TYPE_BUF)
        {
            retVal = CANFD_readPollProcessBuff(ptrCanMsgObj, data);
        }
        if(ptrCanMsgObj->rxMemType == MCAN_MEM_TYPE_FIFO)
        {
            retVal = CANFD_readPollProcessFIFO(ptrCanMsgObj, 
                                               ptrCanMsgObj->rxFifoNum);
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function is used by the application to get the error and 
 *      status information from the driver.
 *
 *  @param[in]  handle
 *      Handle to the CAN Driver
 *  @param[out] ptrOptInfo
 *      Option info in TLV format which is populated with the requested information
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CANFD_getOptions(CANFD_Handle handle, const CANFD_OptionTLV* ptrOptInfo)
{
    CANFD_Object*               ptrCanFdObj;
    CANFD_MessageObject*        ptrCanMsgObj;
    CANFD_MCANErrCntStatus*     ptrErrCounter;
    CANFD_MCANProtocolStatus*   ptrProtoStatus;
    CANFD_MCANMsgObjectStats*   ptrMsgObjectStats;
    int32_t                     retVal = SystemP_SUCCESS;
    MCAN_ErrCntStatus           cslErrStatus = {0};
    MCAN_ProtocolStatus         cslProtocolStatus;
    CANFD_Config               *config;

    if(NULL != handle)
    {
        config = (CANFD_Config *)handle;
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)config->object;
        if ((ptrCanFdObj == NULL) || (ptrOptInfo == NULL))
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Process the supported options */
            switch (ptrOptInfo->type)
            {
                case CANFD_Option_MCAN_ERROR_COUNTER:
                {
                    ptrErrCounter = (CANFD_MCANErrCntStatus*) ptrOptInfo->value;
                    /* Sanity Check: Validate the arguments. */
                    if ((ptrOptInfo->length != (int32_t)sizeof(CANFD_MCANErrCntStatus)) || 
                        (ptrErrCounter == NULL))
                    {
                        retVal = SystemP_FAILURE;
                    }
                    else
                    {
                        /* Populate the stats */
                        MCAN_getErrCounters (ptrCanFdObj->regBaseAddress, &cslErrStatus);
                        ptrErrCounter->canErrLogCnt = cslErrStatus.canErrLogCnt;
                        ptrErrCounter->recErrCnt    = cslErrStatus.recErrCnt;
                        ptrErrCounter->rpStatus     = cslErrStatus.rpStatus;
                        ptrErrCounter->transErrLogCnt = cslErrStatus.transErrLogCnt;
                    }
                    break;
                }
                case CANFD_Option_MCAN_PROTOCOL_STATUS:
                {
                    ptrProtoStatus = (CANFD_MCANProtocolStatus*) ptrOptInfo->value;
                    /* Sanity Check: Validate the arguments. */
                    if ((ptrOptInfo->length != (int32_t)sizeof(CANFD_MCANProtocolStatus)) || 
                        (ptrProtoStatus == NULL))
                    {
                        retVal = SystemP_FAILURE;
                    }
                    else
                    {
                        /* Populate the stats */
                        MCAN_getProtocolStatus (ptrCanFdObj->regBaseAddress, &cslProtocolStatus);
                        ptrProtoStatus->act           = cslProtocolStatus.act;
                        ptrProtoStatus->busOffStatus  = cslProtocolStatus.busOffStatus;
                        ptrProtoStatus->dlec          = cslProtocolStatus.dlec;
                        ptrProtoStatus->errPassive    = cslProtocolStatus.errPassive;
                        ptrProtoStatus->lastErrCode   = cslProtocolStatus.lastErrCode;
                        ptrProtoStatus->pxe           = cslProtocolStatus.pxe;
                        ptrProtoStatus->rbrs          = cslProtocolStatus.rbrs;
                        ptrProtoStatus->resi          = cslProtocolStatus.resi;
                        ptrProtoStatus->rfdf          = cslProtocolStatus.rfdf;
                        ptrProtoStatus->tdcv          = cslProtocolStatus.tdcv;
                        ptrProtoStatus->warningStatus = cslProtocolStatus.warningStatus;
                    }

                    break;
                }
                case CANFD_Option_MCAN_MSG_OBJECT_STATS:
                {
                    ptrMsgObjectStats = (CANFD_MCANMsgObjectStats*) ptrOptInfo->value;
                    /* Sanity Check: Validate the arguments. */
                    if ((ptrOptInfo->length != (int32_t)sizeof(CANFD_MCANMsgObjectStats)) ||
                        (ptrMsgObjectStats == NULL) )
                    {
                        retVal = SystemP_FAILURE;
                    }
                    else
                    {
                        ptrCanMsgObj = (CANFD_MessageObject*) ptrMsgObjectStats->handle;
                        if(ptrCanMsgObj == NULL)
                        {
                            retVal = SystemP_FAILURE;
                        }
                        else
                        {
                            /* Populate the stats */
                            ptrMsgObjectStats->startMsgIdentifier = ptrCanMsgObj->startMsgId;
                            ptrMsgObjectStats->endMsgIdentifier = ptrCanMsgObj->endMsgId;
                            ptrMsgObjectStats->direction = (uint32_t)ptrCanMsgObj->direction;
                            ptrMsgObjectStats->interruptsRxed = ptrCanMsgObj->interruptsRxed;
                            ptrMsgObjectStats->messageProcessed = ptrCanMsgObj->messageProcessed;
                        }
                    }

                    break;
                }
                default:
                {
                    /* Option is NOT supported */
                    retVal = SystemP_FAILURE;
                    break;
                }
            }
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function is used by the application to configure the driver options.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[in] ptrOptInfo
 *      Option info in TLV format which is used to configure the driver
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - SystemP_SUCCESS
 *  @retval
 *      Error       - SystemP_FAILURE
 */
int32_t CANFD_setOptions(CANFD_Handle handle, const CANFD_OptionTLV* ptrOptInfo)
{
    CANFD_Object*                  ptrCanFdObj;
    uint32_t                       baseAddr;
    CANFD_MCANLoopbackCfgParams*   ptrMcanLoopBackCfg;
    int32_t                        retVal = SystemP_SUCCESS;
    uint32_t                       startTicks, elapsedTicksPowerDown;
    uint32_t                       elapsedTicksWakeUp;
    CANFD_Config                  *config;

    if(handle != NULL)
    {
        config = (CANFD_Config *)handle;
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)config->object;
        if ((ptrCanFdObj == NULL) || (ptrOptInfo == NULL))
        {
            retVal = SystemP_FAILURE;
        }

        if(retVal == SystemP_SUCCESS)
        {
            /* Process the supported options */
            switch (ptrOptInfo->type)
            {
                case CANFD_Option_MCAN_MODE:
                {
                    /* Sanity Check: Validate the arguments. */
                    if (ptrOptInfo->length != (int32_t)sizeof(uint8_t))
                    {
                        retVal = SystemP_FAILURE;
                    }

                    if(retVal == SystemP_SUCCESS)
                    {
                        baseAddr = ptrCanFdObj->regBaseAddress;

                        if ((*(uint8_t*)ptrOptInfo->value == (uint8_t)1U) || 
                            (*(uint8_t*)ptrOptInfo->value == (uint8_t)0U))
                        {
                            /* Set MCAN in specified mode */
                            retVal = CANFD_updateOpMode(baseAddr, 
                                      (uint32_t)(*(uint8_t*)ptrOptInfo->value));
                        }
                        else
                        {
                            retVal = SystemP_FAILURE;
                        }
                    }
                    break;
                }
                case CANFD_Option_MCAN_LOOPBACK:
                {
                    /* Sanity Check: Validate the arguments. */
                    if (ptrOptInfo->length != (int32_t)sizeof(CANFD_MCANLoopbackCfgParams))
                    {
                        retVal = SystemP_FAILURE;
                    }
                    if(retVal == SystemP_SUCCESS)
                    {
                        baseAddr = ptrCanFdObj->regBaseAddress;

                        ptrMcanLoopBackCfg = (CANFD_MCANLoopbackCfgParams*) ptrOptInfo->value;

                        /* Set MCAN in SW initialization mode */
                        retVal += CANFD_updateOpMode(baseAddr, 
                                                MCAN_OPERATION_MODE_SW_INIT);

                        /* Disable loopback mode */
                        if (ptrMcanLoopBackCfg->enable == (uint32_t)0U)
                        {
                            MCAN_lpbkModeEnable(baseAddr, 
                                                ptrMcanLoopBackCfg->mode, 
                                                0U);
                        }
                        else
                        {
                            /* Enable loopback mode */
                            MCAN_lpbkModeEnable(baseAddr, 
                                                ptrMcanLoopBackCfg->mode, 
                                                1U);
                        }
                        /* Set MCAN in opertional mode */
                        retVal += CANFD_updateOpMode(baseAddr, 
                                                MCAN_OPERATION_MODE_NORMAL);
                    }
                    break;
                }
                case CANFD_Option_MCAN_POWER_DOWN:
                {
                    /* Sanity Check: Validate the arguments. */
                    if (ptrOptInfo->length != (int32_t)sizeof(uint8_t))
                    {
                        retVal = SystemP_FAILURE;
                    }

                    if(retVal == SystemP_SUCCESS)
                    {
                        baseAddr = ptrCanFdObj->regBaseAddress;

                        startTicks = ClockP_getTicks();

                        if (*(uint8_t*)ptrOptInfo->value == (uint8_t)1U)
                        {
                            /* Request a clock stop to enter local power down */
                            MCAN_addClockStopRequest(baseAddr, 1U);

                            /* Wait for power down to be successful */
                            do
                            {
                                elapsedTicksPowerDown = ClockP_getTicks() - startTicks;
                            } while ((MCAN_getClkStopAck(baseAddr) != MCAN_CCCR_CSA_ACK) && 
                                     (elapsedTicksPowerDown < ClockP_usecToTicks(MCAN_POWER_DOWN_TIMEOUT_IN_US)));

                            /* Set timeout error if timeout occurred */
                            if (elapsedTicksPowerDown >= ClockP_usecToTicks(MCAN_POWER_DOWN_TIMEOUT_IN_US))
                            {
                                retVal = SystemP_TIMEOUT;
                            }
                            else
                            {
                                /* Update the state information */
                                ptrCanFdObj->state = CCANFD_DRIVER_STATE_SLEEP;
                            }
                        }
                        else if (*(uint8_t*)ptrOptInfo->value == (uint8_t)0U)
                        {
                            /* Turn on the local clocks to wakeup from local power down */
                            MCAN_addClockStopRequest(baseAddr, 0);

                            /* Wait for wake up to be successful */
                            do
                            {
                                elapsedTicksWakeUp = ClockP_getTicks() - startTicks;
                            } while ((MCAN_getClkStopAck(baseAddr) != MCAN_CCCR_CSA_NO_ACK) && 
                                     (elapsedTicksWakeUp < ClockP_usecToTicks(MCAN_WAKEUP_TIMEOUT_IN_US)));

                            /* Set timeout error if timeout occurred */
                            if (elapsedTicksWakeUp >= ClockP_usecToTicks(MCAN_WAKEUP_TIMEOUT_IN_US))
                            {
                                retVal = SystemP_TIMEOUT;
                            }
                            else
                            {
                                /* Set MCAN in operational mode */
                                retVal = CANFD_updateOpMode(baseAddr, MCAN_OPERATION_MODE_NORMAL);

                                /* Update the state information */
                                ptrCanFdObj->state = CANFD_DRIVER_STATE_STARTED;
                            }
                        }
                        else
                        {
                            retVal = SystemP_FAILURE;
                        }
                    }
                    break;
                }
                default:
                {
                    /* Option is NOT supported */
                    retVal = SystemP_FAILURE;
                    break;
                }
            }
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

static int32_t CANFD_updateOpMode(uint32_t baseAddr, uint32_t mode)
{
    int32_t  status = SystemP_SUCCESS;
    uint32_t opMode, elapsedTicks, startTicks;

    MCAN_setOpMode(baseAddr, mode);

    /* Wait while MCAN mode is updated */
    startTicks = ClockP_getTicks();
    do
    {
        opMode = MCAN_getOpMode(baseAddr);
        elapsedTicks = ClockP_getTicks() - startTicks;

        if(elapsedTicks > MCAN_OPMODE_TIMEOUT)
        {
            status = SystemP_TIMEOUT;
        }
    } while((opMode != mode) && (status == SystemP_SUCCESS));

    return status;
}

void CANFD_transferCallBack(void* args, CANFD_Reason reason)
{
    CANFD_Object*           ptrCanFdObj;
    CANFD_MsgObjHandle      ptrCanMsgObj = (CANFD_MsgObjHandle)args;

    if(args != NULL)
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;

        if ((reason == CANFD_Reason_TX_COMPLETION) || 
            (reason == CANFD_Reason_TX_CANCELED))
        {
            if((ptrCanFdObj->openParams->transferMode) == CANFD_TRANSFER_MODE_CALLBACK)
            {
                ptrCanFdObj->openParams->transferCallbackFxn(ptrCanMsgObj, reason);
            }
            else
            {
                SemaphoreP_post((SemaphoreP_Object *)&ptrCanFdObj->writeTransferSemObj);
            }
        }
        if (reason == CANFD_Reason_RX)
        {
            if((ptrCanFdObj->openParams->transferMode) == CANFD_TRANSFER_MODE_CALLBACK)
            {
                ptrCanFdObj->openParams->transferCallbackFxn(ptrCanMsgObj, reason);
            }
            else
            {
                SemaphoreP_post((SemaphoreP_Object *)&ptrCanFdObj->readTransferSemObj);
            }
        }
    }

    return;
}

void CANFD_errStatusCallBack(CANFD_Handle handle, CANFD_Reason reason, 
                             CANFD_ErrStatusResp* errStatusResp)
{
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;

    if(handle != NULL)
    {
        config      = (CANFD_Config*) handle;
        ptrCanFdObj = (CANFD_Object*) config->object;

        if ((reason == CANFD_Reason_ECC_ERROR) ||
            (reason == CANFD_Reason_BUSOFF) ||
            (reason == CANFD_Reason_PROTOCOL_ERR_DATA_PHASE) ||
            (reason == CANFD_Reason_PROTOCOL_ERR_ARB_PHASE) ||
            (reason == CANFD_Reason_SRC_RX_FIFO0_MSG_LOST) ||
            (reason == CANFD_Reason_SRC_RX_FIFO1_MSG_LOST))
        {
            if((ptrCanFdObj->openParams->transferMode) == CANFD_TRANSFER_MODE_CALLBACK)
            {
                ptrCanFdObj->openParams->errorCallbackFxn(handle, reason, errStatusResp);
            }
            else
            {
                SemaphoreP_post((SemaphoreP_Object *)&ptrCanFdObj->readTransferSemObj);
                SemaphoreP_post((SemaphoreP_Object *)&ptrCanFdObj->writeTransferSemObj);
            }
        }
    }

    return;
}

static int32_t MCAN_configParamsLss(uint32_t lss)
{
    int32_t status = SystemP_FAILURE;

    if(lss <= (uint32_t)128U)
    {
        status = MCAN_STATUS_SUCCESS;
    }

    return status;
}

static int32_t MCAN_configParamsLse(uint32_t lse)
{
    int32_t status = SystemP_FAILURE;

    if(lse <= (uint32_t)64U)
    {
        status = MCAN_STATUS_SUCCESS;
    }

    return status;
}

static int32_t MCAN_configParamsTxBuffNum(uint32_t txBufNum)
{
    int32_t status = SystemP_FAILURE;

    if(txBufNum <= (uint32_t)32U)
    {
        status = MCAN_STATUS_SUCCESS;
    }

    return status;
}

static int32_t MCAN_configParamsTxFifoSize(uint32_t txFIFOSize)
{
    int32_t status = SystemP_FAILURE;

    if(txFIFOSize <= (uint32_t)32U)
    {
        status = MCAN_STATUS_SUCCESS;
    }

    return status;
}

static int32_t MCAN_configParamsTxEventFifoSize(uint32_t txEventFIFOSize)
{
    int32_t status = SystemP_FAILURE;

    if(txEventFIFOSize <= (uint32_t)32U)
    {
        status = MCAN_STATUS_SUCCESS;
    }

    return status;
}

static int32_t MCAN_configParamsRxFifoSize(uint32_t rxFIFOSize)
{
    int32_t status = SystemP_FAILURE;

    if(rxFIFOSize <= (uint32_t)64U)
    {
        status = MCAN_STATUS_SUCCESS;
    }

    return status;
}

static int32_t MCAN_configParamsFifoOpMode(uint32_t fifoOpMode)
{
    int32_t status = SystemP_FAILURE;

    if((fifoOpMode == MCAN_RX_FIFO_OPERATION_MODE_BLOCKING) ||
       (fifoOpMode == MCAN_RX_FIFO_OPERATION_MODE_OVERWRITE))
    {
        status = MCAN_STATUS_SUCCESS;
    }

    return status;
}

static int32_t MCAN_CheckRegBaseAddr(uint32_t baseAddr)
{
    int32_t status = SystemP_FAILURE;

    if(baseAddr != (uint32_t)0U)
    {
        status = MCAN_STATUS_SUCCESS;
    }

    return status;
}

static int32_t CANFD_writePollProcessBuff(CANFD_MsgObjHandle handle,
                                          uint32_t id,
                                          CANFD_MCANFrameType frameType,
                                          const uint8_t* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;
    MCAN_TxBufElement       txBuffElem;
    uint32_t                index, txStatus;
    uint8_t                 padSize = 0U;
    uint32_t                baseAddr, bitPos = 0U;

    ptrCanMsgObj = (CANFD_MessageObject*)handle;
    /* Get the pointer to the CAN Driver Block */
    ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
    baseAddr    = ptrCanFdObj->regBaseAddress;

    /* Check for pending messages */
    index = (uint32_t)1U << ptrCanMsgObj->txElement;
    if (index == (MCAN_getTxBufReqPend(baseAddr) & index))
    {
        retVal = MCAN_STATUS_BUSY;
    }
    else
    {
        /* populate the Tx buffer message element */
        txBuffElem.rtr = (uint32_t)0U;
        txBuffElem.esi = (uint32_t)0U;
        txBuffElem.efc = (uint32_t)0U;
        txBuffElem.mm =  (uint32_t)0U;

        /* Update fields based on Frame type - Classic CAN or CAN FD */
        if(frameType == CANFD_MCANFrameType_CLASSIC)
        {
            txBuffElem.brs = (uint32_t)0U;
            txBuffElem.fdf = (uint32_t)0U;
        }
        else
        {
            txBuffElem.brs = (uint32_t)1U;
            txBuffElem.fdf = (uint32_t)1U;
        }
        /* Populate the Id */
        if (ptrCanMsgObj->msgIdType == CANFD_MCANXidType_11_BIT)
        {
            txBuffElem.xtd = (uint32_t)CANFD_MCANXidType_11_BIT;
            txBuffElem.id = (id & STD_MSGID_MASK) << STD_MSGID_SHIFT;
        }
        else
        {
            txBuffElem.xtd = (uint32_t)CANFD_MCANXidType_29_BIT;
            txBuffElem.id = id & XTD_MSGID_MASK;
        }

        /* Copy the data */
        (void)memcpy ((void*)&txBuffElem.data, data, ptrCanMsgObj->dataLength);
                
        txBuffElem.dlc = 0U;
        /* Compute the DLC value */
        for(index = (uint32_t)0U ; index < (uint32_t)16U ; index++)
        {
            if((uint8_t)ptrCanMsgObj->dataLength <= ptrCanFdObj->mcanDataSize[index])
            {
                txBuffElem.dlc = index;
                padSize = ptrCanFdObj->mcanDataSize[index] - (uint8_t)ptrCanMsgObj->dataLength;
                break;
            }
        }
        if (index == (uint32_t)CANFD_MAX_DLC_MAPPING)
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Pad the unused data in payload */
            index = ptrCanMsgObj->dataLength;
            while (padSize != (uint8_t)0U)
            {
                txBuffElem.data[index] = (uint8_t)0xCCU;
                index++;
                padSize--;
            }
        }

        MCAN_writeMsgRam(baseAddr, MCAN_MEM_TYPE_BUF, ptrCanMsgObj->txElement, &txBuffElem);
        /* Add request for transmission, This function will trigger transmission */
        retVal += MCAN_txBufAddReq(baseAddr, ptrCanMsgObj->txElement);
        bitPos = ((uint32_t)1U << ptrCanMsgObj->txElement);
        /* Poll for Tx completion */
        do
        {
            txStatus = MCAN_getTxBufTransmissionStatus(baseAddr);
        }while((txStatus & bitPos) != bitPos);

        ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = (uint8_t)1;

        /* Increment the stats */
        ptrCanMsgObj->messageProcessed++;
    }
    return retVal;
 }

static int32_t CANFD_writePollProcessFIFO(CANFD_MsgObjHandle msgObjHandle,
                                          uint32_t id,
                                          CANFD_MCANFrameType frameType,
                                          const uint8_t* data)
 {
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;
    MCAN_TxBufElement       txBuffElem;
    uint32_t                index, txStatus;
    uint8_t                 padSize = 0U;
    uint32_t                baseAddr, bitPos = 0U;
    MCAN_TxFIFOStatus       fifoStatus = {0};

    if(msgObjHandle != NULL)
    {
        ptrCanMsgObj = (CANFD_MessageObject*)msgObjHandle;
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
        baseAddr    = ptrCanFdObj->regBaseAddress;

        MCAN_getTxFIFOQueStatus(baseAddr, &fifoStatus);

        /* populate the Tx buffer message element */
        txBuffElem.rtr = (uint32_t)0U;
        txBuffElem.esi = (uint32_t)0U;
        txBuffElem.efc = (uint32_t)0U;
        txBuffElem.mm =  (uint32_t)0U;

        /* Update fields based on Frame type - Classic CAN or CAN FD */
        if(frameType == CANFD_MCANFrameType_CLASSIC)
        {
            txBuffElem.brs = (uint32_t)0U;
            txBuffElem.fdf = (uint32_t)0U;
        }
        else
        {
            txBuffElem.brs = (uint32_t)1U;
            txBuffElem.fdf = (uint32_t)1U;
        }
        /* Populate the Id */
        if (ptrCanMsgObj->msgIdType == CANFD_MCANXidType_11_BIT)
        {
            txBuffElem.xtd = (uint32_t)CANFD_MCANXidType_11_BIT;
            txBuffElem.id = (id & STD_MSGID_MASK) << STD_MSGID_SHIFT;
        }
        else
        {
            txBuffElem.xtd = (uint32_t)CANFD_MCANXidType_29_BIT;
            txBuffElem.id = id & XTD_MSGID_MASK;
        }
        /* Copy the data */
        (void)memcpy ((void*)&txBuffElem.data, data, ptrCanMsgObj->dataLength);
        
        txBuffElem.dlc = 0U;
        /* Compute the DLC value */
        for(index = (uint32_t)0U ; index < (uint32_t)16U ; index++)
        {
            if((uint8_t)ptrCanMsgObj->dataLength <= ptrCanFdObj->mcanDataSize[index])
            {
                txBuffElem.dlc = index;
                padSize = ptrCanFdObj->mcanDataSize[index] - (uint8_t)ptrCanMsgObj->dataLength;
                break;
            }
        }
        if (index == (uint32_t)CANFD_MAX_DLC_MAPPING)
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Pad the unused data in payload */
            index = ptrCanMsgObj->dataLength;
            while (padSize != (uint8_t)0U)
            {
                txBuffElem.data[index] = (uint8_t)0xCCU;
                index++;
                padSize--;
            }

            MCAN_writeMsgRam(baseAddr, MCAN_MEM_TYPE_FIFO, 
                            ptrCanMsgObj->txElement, 
                            &txBuffElem);
            /* Add request for transmission, This function will trigger transmission */
            retVal += MCAN_txBufAddReq(baseAddr, ptrCanMsgObj->txElement);
            bitPos = ((uint32_t)1U << ptrCanMsgObj->txElement);
            /* Poll for Tx completion */
            do
            {
                txStatus = MCAN_getTxBufTransmissionStatus(baseAddr);
            }while((txStatus & bitPos) != bitPos);

            ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = (uint8_t)1;

            /* Increment the stats */
            ptrCanMsgObj->messageProcessed++;
            /* Acknowledge the writen data */
            (void)MCAN_getTxFIFOQueStatus(baseAddr, &fifoStatus);
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
 }

static int32_t CANFD_writePoll(CANFD_MsgObjHandle handle,
                                uint32_t id,
                                CANFD_MCANFrameType frameType,
                                const uint8_t* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    int32_t                 retVal = SystemP_SUCCESS;

    ptrCanMsgObj = (CANFD_MessageObject*)handle;

    if(ptrCanMsgObj->txMemType == MCAN_MEM_TYPE_BUF)
    {
        retVal = CANFD_writePollProcessBuff(ptrCanMsgObj, id, frameType, data);
    }
    if(ptrCanMsgObj->txMemType == MCAN_MEM_TYPE_FIFO)
    {
        retVal = CANFD_writePollProcessFIFO(ptrCanMsgObj, id, frameType, data);
    }

    return retVal;
}

static int32_t CANFD_writeIntr(CANFD_MsgObjHandle txMsgHandle,
                                uint32_t id,
                                CANFD_MCANFrameType frameType,
                                const uint8_t* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t retVal =        SystemP_SUCCESS;
    uint32_t                baseAddr;

    ptrCanMsgObj = (CANFD_MessageObject*)txMsgHandle;
    /* Get the pointer to the CAN Driver Block */
    ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
    baseAddr = ptrCanFdObj->regBaseAddress;

    if(baseAddr != (uint32_t)0U)
    {
        if (ptrCanFdObj->openParams->eccConfig.enable == (uint32_t)1U)
        {
            /* Enable ECC Interrupt */
            MCAN_eccEnableIntr(baseAddr, CANFD_MCAN_ECC_ERR_TYPE_SEC, 1U);
            MCAN_eccEnableIntr(baseAddr, CANFD_MCAN_ECC_ERR_TYPE_DED, 1U);
        }

        MCAN_enableIntr(baseAddr, MCAN_INTR_MASK_TX, (uint32_t)TRUE);
        MCAN_enableIntr(baseAddr, MCAN_INTR_SRC_RES_ADDR_ACCESS, (uint32_t)FALSE);
        /* Select Interrupt Line 0 */
        MCAN_selectIntrLine(baseAddr, MCAN_INTR_MASK_TX, MCAN_INTR_LINE_NUM_0);
        /* Enable Interrupt Line */
        MCAN_enableIntrLine(baseAddr, MCAN_INTR_LINE_NUM_0, (uint32_t)TRUE);
        /* Select Interrupt Line 1 */
        // MCAN_selectIntrLine(baseAddr, MCAN_INTR_MASK_ALL, MCAN_INTR_LINE_NUM_1);
        // /* Enable Interrupt Line 1 */
        // MCAN_enableIntrLine(baseAddr, MCAN_INTR_LINE_NUM_1, (uint32_t)TRUE);

        if(ptrCanMsgObj->txMemType == MCAN_MEM_TYPE_BUF)
        {
            retVal = CANFD_writeIntrProcessBuff(ptrCanMsgObj, id, frameType, data);
        }
        if(ptrCanMsgObj->txMemType == MCAN_MEM_TYPE_FIFO)
        {
            retVal = CANFD_writeIntrProcessFIFO(ptrCanMsgObj, id, frameType, data);
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

static int32_t CANFD_writeIntrProcessFIFO(CANFD_MsgObjHandle msgObjHandle,
                                          uint32_t id,
                                          CANFD_MCANFrameType frameType,
                                          const uint8_t* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;
    MCAN_TxBufElement       txBuffElem;
    uint32_t                index;
    uint8_t                 padSize = 0U;
    uint32_t                baseAddr;
    MCAN_TxFIFOStatus       fifoStatus = {0};

    if(msgObjHandle != NULL)
    {
        ptrCanMsgObj = (CANFD_MessageObject*)msgObjHandle;
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
        baseAddr    = ptrCanFdObj->regBaseAddress;

        /* Get the FIFO status */
        MCAN_getTxFIFOQueStatus(baseAddr, &fifoStatus);

        /* populate the Tx buffer message element */
        txBuffElem.rtr = (uint32_t)0U;
        txBuffElem.esi = (uint32_t)0U;
        txBuffElem.efc = (uint32_t)0U;
        txBuffElem.mm =  (uint32_t)0U;

        /* Update fields based on Frame type - Classic CAN or CAN FD */
        if(frameType == CANFD_MCANFrameType_CLASSIC)
        {
            txBuffElem.brs = (uint32_t)0U;
            txBuffElem.fdf = (uint32_t)0U;
        }
        else
        {
            txBuffElem.brs = (uint32_t)1U;
            txBuffElem.fdf = (uint32_t)1U;
        }
        /* Populate the Id */
        if (ptrCanMsgObj->msgIdType == CANFD_MCANXidType_11_BIT)
        {
            txBuffElem.xtd = (uint32_t)CANFD_MCANXidType_11_BIT;
            txBuffElem.id = (id & STD_MSGID_MASK) << STD_MSGID_SHIFT;
        }
        else
        {
            txBuffElem.xtd = (uint32_t)CANFD_MCANXidType_29_BIT;
            txBuffElem.id = id & XTD_MSGID_MASK;
        }
        /* Copy the data */
        (void)memcpy ((void*)&txBuffElem.data, data, ptrCanMsgObj->dataLength);
        
        txBuffElem.dlc = 0U;
        /* Compute the DLC value */
        for(index = (uint32_t)0U ; index < (uint32_t)16U ; index++)
        {
            if((uint8_t)ptrCanMsgObj->dataLength <= ptrCanFdObj->mcanDataSize[index])
            {
                txBuffElem.dlc = index;
                padSize = ptrCanFdObj->mcanDataSize[index] - (uint8_t)ptrCanMsgObj->dataLength;
                break;
            }
        }
        if (index == (uint32_t)CANFD_MAX_DLC_MAPPING)
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Pad the unused data in payload */
            index = ptrCanMsgObj->dataLength;
            while (padSize != (uint8_t)0U)
            {
                txBuffElem.data[index] = (uint8_t)0xCCU;
                index++;
                padSize--;
            }

            MCAN_writeMsgRam(baseAddr, MCAN_MEM_TYPE_FIFO, 
                         ptrCanMsgObj->txElement, 
                         &txBuffElem);
            /* Add request for transmission, This function will trigger transmission */
            retVal += MCAN_txBufAddReq(baseAddr, ptrCanMsgObj->txElement);
            ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = (uint8_t)1;
            /* Increment the stats */
            ptrCanMsgObj->messageProcessed++;
            /* Acknowledge the writen data */
            (void)MCAN_getTxFIFOQueStatus(baseAddr, &fifoStatus);
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

static int32_t CANFD_writeIntrProcessBuff(CANFD_MsgObjHandle txMsgHandle,
                                            uint32_t id,
                                            CANFD_MCANFrameType frameType,
                                            const uint8_t* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;
    MCAN_TxBufElement       txBuffElem;
    uint32_t                index;
    uint8_t                 padSize = 0U;
    uint32_t                baseAddr;

    ptrCanMsgObj = (CANFD_MessageObject*)txMsgHandle;
    /* Get the pointer to the CAN Driver Block */
    ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
    baseAddr    = ptrCanFdObj->regBaseAddress;

    /* Check for pending messages */
    index = (uint32_t)1U << ptrCanMsgObj->txElement;
    if (index == (MCAN_getTxBufReqPend(baseAddr) & index))
    {
        retVal = MCAN_STATUS_BUSY;
    }
    else
    {
        /* populate the Tx buffer message element */
        txBuffElem.rtr = (uint32_t)0U;
        txBuffElem.esi = (uint32_t)0U;
        txBuffElem.efc = (uint32_t)0U;
        txBuffElem.mm =  (uint32_t)0U;

        /* Update fields based on Frame type - Classic CAN or CAN FD */
        if(frameType == CANFD_MCANFrameType_CLASSIC)
        {
            txBuffElem.brs = (uint32_t)0U;
            txBuffElem.fdf = (uint32_t)0U;
        }
        else
        {
            txBuffElem.brs = (uint32_t)1U;
            txBuffElem.fdf = (uint32_t)1U;
        }
        /* Populate the Id */
        if (ptrCanMsgObj->msgIdType == CANFD_MCANXidType_11_BIT)
        {
            txBuffElem.xtd = (uint32_t)CANFD_MCANXidType_11_BIT;
            txBuffElem.id = (id & STD_MSGID_MASK) << STD_MSGID_SHIFT;
        }
        else
        {
            txBuffElem.xtd = (uint32_t)CANFD_MCANXidType_29_BIT;
            txBuffElem.id = id & XTD_MSGID_MASK;
        }

        /* Copy the data */
        (void)memcpy ((void*)&txBuffElem.data, data, ptrCanMsgObj->dataLength);
        
        txBuffElem.dlc = 0U;
        /* Compute the DLC value */
        for(index = (uint32_t)0U ; index < (uint32_t)16U ; index++)
        {
            if((uint8_t)ptrCanMsgObj->dataLength <= ptrCanFdObj->mcanDataSize[index])
            {
                txBuffElem.dlc = index;
                padSize = ptrCanFdObj->mcanDataSize[index] - (uint8_t)ptrCanMsgObj->dataLength;
                break;
            }
        }
        if (index == (uint32_t)CANFD_MAX_DLC_MAPPING)
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Pad the unused data in payload */
            index = ptrCanMsgObj->dataLength;
            while (padSize != (uint8_t)0U)
            {
                txBuffElem.data[index] = (uint8_t)0xCCU;
                index++;
                padSize--;
            }
        }

        MCAN_writeMsgRam(baseAddr, ptrCanMsgObj->txMemType, 
                         ptrCanMsgObj->txElement, &txBuffElem);
        /* Add request for transmission, This function will trigger transmission */
        retVal += MCAN_txBufAddReq(baseAddr, ptrCanMsgObj->txElement);

        ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = (uint8_t)1;
        /* Increment the stats */
        ptrCanMsgObj->messageProcessed++;
    }

    return retVal;
}

static int32_t CANFD_readDma(CANFD_MsgObjHandle txMsgHandle,
                              uint32_t numMsgs,
                              const uint8_t* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;

    if(txMsgHandle != NULL)
    {
        /* Get the message object pointer */
        ptrCanMsgObj = (CANFD_MessageObject*)txMsgHandle;

        if (data == NULL)
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Get the pointer to the CAN Driver Block */
            ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;

            if (ptrCanFdObj == NULL)
            {
                retVal = SystemP_FAILURE;
            }
            else
            {
                retVal = CANFD_configureDmaRx(ptrCanFdObj, ptrCanMsgObj, 
                                              txMsgHandle->dataLength, 
                                              numMsgs, data);
            }
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t CANFD_isDataSizeValid(uint32_t dataSize)
{
    switch (dataSize)
    {
        case 0U:
        case 1U:
        case 2U:
        case 3U:
        case 4U:
        case 5U:
        case 6U:
        case 7U:
        case 8U:
        case 12U:
        case 16U:
        case 20U:
        case 24U:
        case 32U:
        case 48U:
        case 64U:
            return SystemP_SUCCESS;
        
        default:
            return SystemP_FAILURE;
    }
}

void CANFD_dmaTxCompletionCallback(CANFD_MessageObject* ptrCanMsgObj, 
                                   const void *data, 
                                   uint32_t completionType)
{
    CANFD_Object*           ptrCanFdObj;

    if(ptrCanMsgObj != NULL)
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
        /* Send next data over Tx message object */
        (void) CANFD_writeDmaTriggerNext(ptrCanMsgObj);
        
        if(completionType == CANFD_DMA_TX_COMPLETION_FINAL)
        {
            if((ptrCanFdObj->openParams->transferMode) == CANFD_TRANSFER_MODE_CALLBACK)
            {
                ptrCanFdObj->openParams->transferCallbackFxn(ptrCanMsgObj, CANFD_Reason_TX_COMPLETION);
            }
            else
            {
                SemaphoreP_post((SemaphoreP_Object *)&ptrCanFdObj->writeTransferSemObj);
            }
        }
    }

    return;
}

void CANFD_dmaRxCompletionCallback(CANFD_MessageObject* ptrCanMsgObj, 
                                   const void *data, 
                                   uint32_t completionType)
{
    CANFD_Object*           ptrCanFdObj;

    if(ptrCanMsgObj != NULL)
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
        if(completionType == CANFD_DMA_RX_COMPLETION_FINAL)
        {
            if((ptrCanFdObj->openParams->transferMode) == CANFD_TRANSFER_MODE_CALLBACK)
            {
                ptrCanFdObj->openParams->transferCallbackFxn(ptrCanMsgObj, CANFD_Reason_RX);
            }
            else
            {
                SemaphoreP_post(&ptrCanMsgObj->canfdHandle->object->readTransferSemObj);
            }
        }
    }
    
    return;
}

/* This function return endianness value of MCAN module. */
int32_t CANFD_getEndianVal(CANFD_Handle canfdHandle)
{
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;
    int32_t       endVal = 0;

    if(canfdHandle != NULL)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;

        endVal = MCAN_getEndianVal(baseAddr);
    }

    return endVal;
}

/* This API is used get the MCAN revision ID. */
void CANFD_getRevisionId(CANFD_Handle canfdHandle)
{
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;

    DebugP_assert(NULL_PTR != canfdHandle);
    config      = (CANFD_Config*) canfdHandle;
    ptrCanFdObj = (CANFD_Object*) config->object;
    DebugP_assert(NULL_PTR != ptrCanFdObj);
    baseAddr    = ptrCanFdObj->regBaseAddress;

    MCAN_getRevisionId(baseAddr, &ptrCanFdObj->revId);

    return;
}

/*  This API add clock stop request for MCAN module to put it
 *  in power down mode. 
 */
void CANFD_addClockStopRequest(CANFD_Handle canfdHandle, uint32_t enable)
{
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;

    if(NULL_PTR != canfdHandle)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;

        MCAN_addClockStopRequest(baseAddr, enable);
    }

    return;
}

/*  This API stops the clock for MCAN module.
 *  It return whether MCAN is power down mode or not. 
 */
uint32_t CANFD_clockStopReq(CANFD_Handle canfdHandle)
{
    int32_t       clkAck = SystemP_SUCCESS;
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;

    if(NULL_PTR != canfdHandle)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;
        /* Request for clk stop request */
        clkAck = MCAN_getClockStopAck(baseAddr);
    }

    return clkAck;
}

/*  This API will return clock stop acknowledgement for MCAN module. */
uint32_t CANFD_getClkStopAck(CANFD_Handle canfdHandle)
{
    int32_t       clkAck = SystemP_SUCCESS;
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;

    if(NULL_PTR != canfdHandle)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;

        clkAck = MCAN_getClkStopAck(baseAddr);
    }

    return clkAck;
}

/* This API will return Rx pin state of MCAN module. */
uint32_t CANFD_getRxPinState(CANFD_Handle canfdHandle)
{
    uint32_t       pinState = 0U;
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;

    if(NULL_PTR != canfdHandle)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;

        pinState = MCAN_getRxPinState(baseAddr);
    }

    return pinState;
}

/* This API will set Tx pin state of MCAN module. */
void CANFD_setTxPinState(CANFD_Handle canfdHandle, uint32_t state)
{
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;

    if(NULL_PTR != canfdHandle)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;

        MCAN_setTxPinState(baseAddr, state);
    }
}

/* This API will return Tx pin state of MCAN module. */
uint32_t CANFD_getTxPinState(CANFD_Handle canfdHandle)
{
    uint32_t      pinState = 0U;
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;

    if(NULL_PTR != canfdHandle)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;

        pinState = MCAN_getTxPinState(baseAddr);
    }

    return pinState;
}

/* This API will get the configured bit timings for MCAN module. */
void CANFD_getBitTime(CANFD_Handle canfdHandle)
{
    CANFD_Config* config = NULL;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;
    CANFD_MCANBitTimingParams  *bitTimingParams;
    MCAN_BitTimingParams configParams;

    if(NULL_PTR != canfdHandle)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;
        bitTimingParams = &config->attrs->CANFDMcanBitTimingParams;

        configParams.nomSynchJumpWidth = HW_RD_FIELD32(MCAN_calcCfgAddr(baseAddr) + MCAN_NBTP,
                                                    MCAN_NBTP_NSJW);
        configParams.nomTimeSeg2 = HW_RD_FIELD32(MCAN_calcCfgAddr(baseAddr) + MCAN_NBTP,
                                                MCAN_NBTP_NTSEG2);
        configParams.nomTimeSeg1 = HW_RD_FIELD32(MCAN_calcCfgAddr(baseAddr) + MCAN_NBTP,
                                                MCAN_NBTP_NTSEG1);
        configParams.nomRatePrescalar = HW_RD_FIELD32(MCAN_calcCfgAddr(baseAddr) + MCAN_NBTP,
                                                    MCAN_NBTP_NBRP);

        configParams.dataSynchJumpWidth = HW_RD_FIELD32(MCAN_calcCfgAddr(baseAddr) + MCAN_DBTP,
                                                        MCAN_DBTP_DSJW);
        configParams.dataTimeSeg2 = HW_RD_FIELD32(MCAN_calcCfgAddr(baseAddr) + MCAN_DBTP,
                                                MCAN_DBTP_DTSEG2);
        configParams.dataTimeSeg1 = HW_RD_FIELD32(MCAN_calcCfgAddr(baseAddr) + MCAN_DBTP,
                                                MCAN_DBTP_DTSEG1);
        configParams.dataRatePrescalar = HW_RD_FIELD32(MCAN_calcCfgAddr(baseAddr) + MCAN_DBTP,
                                                    MCAN_DBTP_DBRP);

        if((configParams.nomTimeSeg1 % 2) == 0U)
        {
            bitTimingParams->nomPropSeg = configParams.nomTimeSeg1 / 2U;
            bitTimingParams->nomPseg1 = configParams.nomTimeSeg1 / 2U;
        }
        else
        {
            bitTimingParams->nomPropSeg = (configParams.nomTimeSeg1 / 2U) + 1U;
            bitTimingParams->nomPseg1 = configParams.nomTimeSeg1 - bitTimingParams->nomPropSeg;

        }
        bitTimingParams->nomBrp = configParams.nomRatePrescalar;
        bitTimingParams->nomPseg2 = configParams.nomTimeSeg2;
        bitTimingParams->nomSjw = configParams.nomSynchJumpWidth + 1U;

        if((configParams.dataTimeSeg1 % 2U) == 0U)
        {
            bitTimingParams->dataPropSeg = configParams.dataTimeSeg1 / 2U;
            bitTimingParams->dataPseg1 = configParams.dataTimeSeg1 / 2U;
        }
        else
        {
            bitTimingParams->nomPropSeg = (configParams.nomTimeSeg1 / 2U) - 1U;
            bitTimingParams->nomPseg1 = configParams.nomTimeSeg1 - bitTimingParams->nomPropSeg;

        }
        bitTimingParams->dataBrp = configParams.dataRatePrescalar;
        bitTimingParams->dataPseg2 = configParams.dataTimeSeg2;
        bitTimingParams->dataSjw =  configParams.dataSynchJumpWidth;
    }

    return;
}

/* This API will return current timestamp counter value. */
uint32_t CANFD_getTSCounterVal(CANFD_Handle canfdHandle)
{
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr, timeStampVal = 0U;

    if(NULL_PTR != canfdHandle)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;

        timeStampVal = MCAN_getTSCounterVal(baseAddr);
    }

    return timeStampVal;
}

/* This API will reset timestamp counter value. */
void CANFD_resetTSCounter(CANFD_Handle canfdHandle)
{
    CANFD_Config* config;
    CANFD_Object* ptrCanFdObj;
    uint32_t      baseAddr;

    if(NULL_PTR != canfdHandle)
    {
        config      = (CANFD_Config*) canfdHandle;
        ptrCanFdObj = (CANFD_Object*) config->object;
        DebugP_assert(NULL_PTR != ptrCanFdObj);
        baseAddr    = ptrCanFdObj->regBaseAddress;

        MCAN_resetTSCounter(baseAddr);
    }

    return;
}
