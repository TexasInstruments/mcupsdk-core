/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_fsi_tx.h>
#include <drivers/fsi.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "test_fsi_common.h"
#include "test_fsi_tx_task.h"

/* ========================================================================== */
/*                         Global Varialbe Declarations                       */
/* ========================================================================== */

uint16_t gTxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];
static HwiP_Object gFsiTxHwiObject;
static SemaphoreP_Object gFsiTxSemObject;
extern SemaphoreP_Object gFsiTxRx_SyncSemaphoreObj;

/* ========================================================================== */
/*                         Static Function Declarations                       */
/* ========================================================================== */
static int32_t Fsi_appTxConfig(uint32_t txBaseAddr, FSI_TxTestParams  *txTestParams);
static int32_t Fsi_appTxIntrInit(uint32_t txBaseAddr, FSI_TxTestParams  *txTestParams);
static void Fsi_appTxIntrDeInit(uint32_t txBaseAddr, FSI_TxTestParams  *txTestParams);
static void Fsi_appTxCallback(void *args);

/* ========================================================================== */
/*                         Function Definitions                                 */
/* ========================================================================== */

void fsi_tx_main(void *args)
{
    int32_t status;
    uint32_t txBaseAddr, txBufAddr;
    uint16_t dataSize;
    uint16_t bufIdx;
    SemaphoreP_Object *p_taskDoneSemaphoreObj;
    FSI_TxTestParams  *txTestParams = (FSI_TxTestParams  *)args;
    uint16_t bufPtrLoc, wordCnt;
    uint16_t *tmpBufAddr;

    /* Test parameters */
    txBaseAddr = txTestParams->baseAddr;
    dataSize = txTestParams->frameDataSize;
    bufIdx = 0U;
    p_taskDoneSemaphoreObj = &txTestParams->taskDoneSemaphoreObj;

    DebugP_log("[FSI] Loopback Tx application started ...\r\n");

    /* FSI configuration */
    status = Fsi_appTxConfig(txBaseAddr, txTestParams);
    status += Fsi_appTxIntrInit(txBaseAddr, txTestParams);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Send Flush Sequence to sync, after every rx soft reset */
    status += FSI_executeTxFlushSequence(txBaseAddr, txTestParams->prescaleVal);
    DebugP_assert(status == SystemP_SUCCESS);

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    if (txTestParams->delayLineCtrlTest == TRUE)
    {
        FSI_configTxDelayLine(txBaseAddr, FSI_TX_DELAY_CLK, 5U);
        FSI_configTxDelayLine(txBaseAddr, FSI_TX_DELAY_D0, 5U);
        FSI_configTxDelayLine(txBaseAddr, FSI_TX_DELAY_D1, 5U);
    }
#endif

    /* sync between rx and tx */
    status = SemaphoreP_pend(&gFsiTxRx_SyncSemaphoreObj,SystemP_WAIT_FOREVER);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Start transfer */
    /* Memset TX buffer with new data for every loop */
    for (uint32_t i = 0; i < dataSize; i++)
    {
        gTxBufData[i] = dataSize + i;
    }

    if (txTestParams->userDefinedCrc == TRUE)
    {
        FSI_enableTxUserCRC(txBaseAddr, FSI_APP_TX_PATTERN_USER_CRC_VALUE);
    }

    if (txTestParams->testEccFlag == TRUE)
    {
        /* ECC computation for 2 words */
        uint16_t getEccVal = 0U;
        uint32_t txData = gTxBufData[0U] | gTxBufData[1U] << 16U;

        FSI_setTxECCComputeWidth(txBaseAddr, FSI_16BIT_ECC_COMPUTE);
        FSI_setTxECCdata(txBaseAddr, txData);
        FSI_getTxECCValue(txBaseAddr, &getEccVal);
        FSI_setTxUserDefinedData(txBaseAddr, getEccVal);
    }

    /* Transmit data */
    status = FSI_setTxBufferPtr(txBaseAddr, bufIdx);
    status += FSI_getTxBufferPtr(txBaseAddr, &bufPtrLoc);
    DebugP_assert(bufPtrLoc == bufIdx);
    status += FSI_writeTxBuffer(txBaseAddr, gTxBufData, dataSize, bufIdx);
    DebugP_assert(status == SystemP_SUCCESS);

    if (txTestParams->rxFrameWDTest != TRUE)
    {
        FSI_getTxBufferAddress(txBaseAddr, &txBufAddr);
        tmpBufAddr = (uint16_t *)txBufAddr;
        for (uint32_t i = 0; i < dataSize; i++)
        {
            DebugP_assert(gTxBufData[i] == tmpBufAddr[i]);
        }
        status = FSI_startTxTransmit(txBaseAddr);
        DebugP_assert(status == SystemP_SUCCESS);

        FSI_getTxWordCount(txBaseAddr, &wordCnt);
        /* Wait for TX and RX completion */
        SemaphoreP_pend(&gFsiTxSemObject, SystemP_WAIT_FOREVER);
    }
    else
    {
        status = FSI_startTxTransmit(txBaseAddr);
        DebugP_assert(status == SystemP_SUCCESS);
        FSI_disableTxClock(txBaseAddr);
    }

    Fsi_appTxIntrDeInit(txBaseAddr, txTestParams);
    FSI_disableTxClock(txBaseAddr);
    if (txTestParams->userDefinedCrc == TRUE)
    {
        /* CRC Value is calculated based on the TX pattern */
        FSI_disableTxUserCRC(txBaseAddr);
    }

    SemaphoreP_post(p_taskDoneSemaphoreObj);
    TaskP_exit();
}

void fsi_tx_hwPingTest(void *args)
{
    int32_t status;
    uint32_t txBaseAddr;
    uint32_t pingTimeCounter = 0U;
    SemaphoreP_Object *p_taskDoneSemaphoreObj;
    FSI_TxTestParams  *txTestParams = (FSI_TxTestParams  *)args;

    /* Test parameters */
    txBaseAddr = txTestParams->baseAddr;
    p_taskDoneSemaphoreObj = &txTestParams->taskDoneSemaphoreObj;

    DebugP_log("[FSI] Loopback Tx application started ...\r\n");

    /* FSI configuration */
    status = Fsi_appTxConfig(txBaseAddr, txTestParams);
    status += FSI_resetTxModule(txBaseAddr, FSI_TX_PING_TIMEOUT_CNT_RESET);
    status += FSI_clearTxModuleReset(txBaseAddr, FSI_TX_PING_TIMEOUT_CNT_RESET);
    status += Fsi_appTxIntrInit(txBaseAddr, txTestParams);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Send Flush Sequence to sync, after every rx soft reset */
    status += FSI_executeTxFlushSequence(txBaseAddr, txTestParams->prescaleVal);
    DebugP_assert(status == SystemP_SUCCESS);

    /* sync between rx and tx */
    status = SemaphoreP_pend(&gFsiTxRx_SyncSemaphoreObj,SystemP_WAIT_FOREVER);
    DebugP_assert(status == SystemP_SUCCESS);

    if (txTestParams->rxPingWDTest != TRUE)
    {
        FSI_setTxPingTimeoutMode(txBaseAddr, FSI_PINGTIMEOUT_ON_HWINIT_PING_FRAME);
        FSI_setTxPingTag(txBaseAddr, FSI_FRAME_TAG10);
        FSI_enableTxPingTimer(txBaseAddr, FSI_APP_TXCLK_FREQ / 2, FSI_FRAME_TAG10);

        FSI_getTxCurrentPingTimeoutCounter(txBaseAddr, &pingTimeCounter);
        /* Wait for TX and RX completion */
        SemaphoreP_pend(&gFsiTxSemObject, SystemP_WAIT_FOREVER);
    }
    else
    {
        FSI_setTxPingTimeoutMode(txBaseAddr, FSI_PINGTIMEOUT_ON_HWINIT_PING_FRAME);
        FSI_setTxPingTag(txBaseAddr, FSI_FRAME_TAG10);
        FSI_enableTxPingTimer(txBaseAddr, FSI_APP_TXCLK_FREQ / 2, FSI_FRAME_TAG10);
        FSI_disableTxClock(txBaseAddr);
    }

    Fsi_appTxIntrDeInit(txBaseAddr, txTestParams);
    FSI_disableTxPingTimer(txBaseAddr);

    SemaphoreP_post(p_taskDoneSemaphoreObj);
    TaskP_exit();
}


void fsi_tx_negativeTest(void *args)
{
    int32_t status;
    uint32_t txBaseAddr;
    SemaphoreP_Object *p_taskDoneSemaphoreObj;
    FSI_TxTestParams  *txTestParams = (FSI_TxTestParams  *)args;
    const uint16_t txBuf;
    uint16_t regVal;

    /* Test parameters */
    txBaseAddr = txTestParams->baseAddr;
    p_taskDoneSemaphoreObj = &txTestParams->taskDoneSemaphoreObj;

    status = FSI_selectTxPLLClock(txBaseAddr, FSI_TX_CLK_SEL1 + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_enableTxClock(txBaseAddr, 0xFF + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxDataWidth(txBaseAddr, FSI_DATA_WIDTH_2_LANE + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxStartMode(txBaseAddr, FSI_TX_START_FRAME_CTRL_OR_UDATA_TAG + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxPingTimeoutMode(txBaseAddr, FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxExtFrameTrigger(txBaseAddr, FSI_TX_MAX_NUM_EXT_TRIGGERS);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxECCComputeWidth(txBaseAddr, FSI_16BIT_ECC_COMPUTE + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxFrameType(txBaseAddr, FSI_FRAME_TYPE_ERROR + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxSoftwareFrameSize(txBaseAddr, FSI_MAX_LEN_NWORDS_DATA + 2);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxSoftwareFrameSize(txBaseAddr, 0);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxFrameTag(txBaseAddr, FSI_FRAME_TAG15 + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxUserDefinedData(txBaseAddr, FSI_MAX_VALUE_USERDATA + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxBufferPtr(txBaseAddr, FSI_MAX_VALUE_BUF_PTR_OFF + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getTxBufferPtr(txBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getTxWordCount(txBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_enableTxPingTimer(txBaseAddr, FSI_FRAME_TAG15, FSI_FRAME_TAG15 + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setTxPingTag(txBaseAddr, FSI_FRAME_TAG15 + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_enableTxExtPingTrigger(txBaseAddr, FSI_TX_MAX_NUM_EXT_TRIGGERS);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getTxCurrentPingTimeoutCounter(txBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getTxEventStatus(txBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_forceTxEvents(txBaseAddr, FSI_TX_EVTMASK + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_clearTxEvents(txBaseAddr, FSI_TX_EVTMASK + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getTxECCValue(txBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_enableTxInterrupt(txBaseAddr, FSI_INT2, FSI_TX_EVTMASK + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_enableTxInterrupt(txBaseAddr, FSI_INT2 + 1, FSI_TX_EVTMASK);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_disableTxInterrupt(txBaseAddr, FSI_INT2, FSI_TX_EVTMASK + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_disableTxInterrupt(txBaseAddr, FSI_INT2 + 1, FSI_TX_EVTMASK);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getTxBufferAddress(txBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_resetTxModule(txBaseAddr, FSI_TX_PING_TIMEOUT_CNT_RESET + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_clearTxModuleReset(txBaseAddr, FSI_TX_PING_TIMEOUT_CNT_RESET + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_writeTxBuffer(txBaseAddr, NULL_PTR,
                                (FSI_MAX_VALUE_BUF_PTR_OFF + (uint16_t)1U),
                                 FSI_MAX_VALUE_BUF_PTR_OFF);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_writeTxBuffer(txBaseAddr, &txBuf,
                                (FSI_MAX_VALUE_BUF_PTR_OFF + (uint16_t)2U),
                                 FSI_MAX_VALUE_BUF_PTR_OFF);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_writeTxBuffer(txBaseAddr, &txBuf,
                                (FSI_MAX_VALUE_BUF_PTR_OFF + (uint16_t)1U),
                                 FSI_MAX_VALUE_BUF_PTR_OFF + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_executeTxFlushSequence(txBaseAddr, 0xFF + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_resetTxModule(txBaseAddr, FSI_TX_PING_TIMEOUT_CNT_RESET + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_clearTxModuleReset(txBaseAddr, FSI_TX_PING_TIMEOUT_CNT_RESET + 1);
    DebugP_assert(status == CSL_EBADARGS);

    FSI_enableTxSPIMode(txBaseAddr);
    regVal = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SPI_MODE_MASK) ==
                CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SPI_MODE_MASK);
    FSI_disableTxSPIMode(txBaseAddr);
    regVal = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SPI_MODE_MASK) ==
                0U);
    FSI_setTxPingTimeoutMode(txBaseAddr, FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME);
    regVal = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__PING_TO_MODE_MASK) ==
                CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__PING_TO_MODE_MASK);
    FSI_setTxECCComputeWidth(txBaseAddr, FSI_32BIT_ECC_COMPUTE);
    regVal = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__ECC_SEL_MASK) ==
                0U);
    FSI_setTxExtFrameTrigger(txBaseAddr, 0U);
    regVal = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__EXT_TRIG_SEL_MASK) ==
                0U);
    FSI_enableTxExtPingTrigger(txBaseAddr, 0U);
    regVal = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_SEL_MASK) ==
                0U);
    regVal = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_EN_MASK) ==
                CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_EN_MASK);
    FSI_disableTxExtPingTrigger(txBaseAddr);
    regVal = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_EN_MASK) ==
                0U);
    FSI_setTxStartMode(txBaseAddr, FSI_TX_START_FRAME_CTRL_OR_UDATA_TAG);
    regVal = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
    DebugP_assert(((regVal & CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__START_MODE_MASK) >>
                    CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__START_MODE_SHIFT) ==
                FSI_TX_START_FRAME_CTRL_OR_UDATA_TAG);
    FSI_enableTxCRCForceError(txBaseAddr);
    regVal  = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__FORCE_ERR_MASK) ==
                CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__FORCE_ERR_MASK);
    FSI_disableTxCRCForceError(txBaseAddr);
    regVal  = HW_RD_REG16(txBaseAddr + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_);
    DebugP_assert((regVal & CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__FORCE_ERR_MASK) ==
                0U);
    FSI_forceTxEvents(txBaseAddr, FSI_TX_EVT_BUF_OVERRUN);
    FSI_getTxEventStatus(txBaseAddr, &regVal);
    DebugP_assert((regVal & FSI_TX_EVT_BUF_OVERRUN) == FSI_TX_EVT_BUF_OVERRUN);
    FSI_clearTxEvents(txBaseAddr, FSI_TX_EVT_BUF_OVERRUN);
    FSI_getTxEventStatus(txBaseAddr, &regVal);
    DebugP_assert((regVal & FSI_TX_EVT_BUF_OVERRUN) == 0U);

    SemaphoreP_post(p_taskDoneSemaphoreObj);
    TaskP_exit();
}

static int32_t Fsi_appTxConfig(uint32_t txBaseAddr,  FSI_TxTestParams *txTestParams)
{
    int32_t status;

    /* TX init and reset */
    status = FSI_performTxInitialization(txBaseAddr, txTestParams->prescaleVal);
    status += FSI_resetTxModule(txBaseAddr, FSI_TX_MAIN_CORE_RESET);
    FSI_clearTxModuleReset(txBaseAddr, FSI_TX_MAIN_CORE_RESET);

    /* Setting for requested transfer params */
    status += FSI_setTxSoftwareFrameSize(txBaseAddr, txTestParams->frameDataSize);
    status += FSI_setTxDataWidth(txBaseAddr, txTestParams->numLane);

    /* Setting frame config */
 #if defined (SOC_AM263X)
    if (txTestParams->udataFilterTest == TRUE)
    {
        status += FSI_setTxUserDefinedData(txBaseAddr, FSI_APP_TX_USER_DATA_FILTER_VALUE);
    }
    if (txTestParams->rxTriggerTest == TRUE)
    {
        /* RX TRIGGER 0*/
        status += FSI_setTxExtFrameTrigger(txBaseAddr, FSI_APP_RX_TRIGGER_0_VALUE);
    }
#else
        status += FSI_setTxUserDefinedData(txBaseAddr, txTestParams->userData);
#endif
    status += FSI_setTxFrameTag(txBaseAddr, txTestParams->frameDataTag);
    status += FSI_setTxFrameType(txBaseAddr, txTestParams->frameType);

    return status;
}

static int32_t Fsi_appTxIntrInit(uint32_t txBaseAddr, FSI_TxTestParams  *txTestParams)
{
    int32_t status;
    uint32_t txIntrNum;
    HwiP_Params txHwiPrms;

    /*
     * TX interrupt config and registration
     */
    txIntrNum = txTestParams->intrLine;
    status = SemaphoreP_constructBinary(&gFsiTxSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    HwiP_Params_init(&txHwiPrms);
    txHwiPrms.intNum = txIntrNum;
    txHwiPrms.callback = Fsi_appTxCallback;
    txHwiPrms.args = (void *)txBaseAddr;
    status += HwiP_construct(&gFsiTxHwiObject, &txHwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    /* Enable TX frame done interrupt */
    status += FSI_enableTxInterrupt(txBaseAddr, txTestParams->intrNum, txTestParams->intrEvt);

    return status;
}

static void Fsi_appTxIntrDeInit(uint32_t txBaseAddr, FSI_TxTestParams  *txTestParams)
{
    /* TX interrupt deinit */
    FSI_disableTxInterrupt(txBaseAddr, txTestParams->intrNum, FSI_TX_EVTMASK);
    FSI_clearTxEvents(txBaseAddr, FSI_TX_EVTMASK);
    HwiP_destruct(&gFsiTxHwiObject);
    SemaphoreP_destruct(&gFsiTxSemObject);

    return;
}

static void Fsi_appTxCallback(void *args)
{
    uint32_t txBaseAddr = (uint32_t)args;
    uint16_t intrStatus;

    FSI_getTxEventStatus(txBaseAddr, &intrStatus);
    if ((intrStatus & FSI_TX_EVT_FRAME_DONE) == FSI_TX_EVT_FRAME_DONE)
    {
        FSI_clearTxEvents(txBaseAddr, FSI_TX_EVT_FRAME_DONE);
        SemaphoreP_post(&gFsiTxSemObject);
    }
    if ((intrStatus & FSI_TX_EVT_PING_HW_TRIG) == FSI_TX_EVT_PING_HW_TRIG)
    {
        FSI_clearTxEvents(txBaseAddr, FSI_TX_EVT_PING_HW_TRIG);
        SemaphoreP_post(&gFsiTxSemObject);
    }

    return;
}
