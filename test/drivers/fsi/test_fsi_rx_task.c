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
#include <drivers/hw_include/cslr_fsi_rx.h>
#include <drivers/fsi.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "test_fsi_common.h"
#include "test_fsi_rx_task.h"

/* ========================================================================== */
/*                         Global Varialbe Declarations                       */
/* ========================================================================== */

uint16_t gRxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];
static HwiP_Object gFsiRxHwiObject;
static SemaphoreP_Object gFsiRxSemObject;
extern SemaphoreP_Object gFsiTxRx_SyncSemaphoreObj;
volatile uint32_t gRxFrameWdTest = FALSE;
volatile uint32_t gRxPingWdTest = FALSE;

/* ========================================================================== */
/*                         Static Function Declarations                       */
/* ========================================================================== */
static int32_t Fsi_appRxConfig(uint32_t rxBaseAddr, FSI_RxTestParams *rxTestParams);
static int32_t Fsi_appRxIntrInit(uint32_t rxBaseAddr, FSI_RxTestParams  *rxTestParams);
static void Fsi_appRxIntrDeInit(uint32_t rxBaseAddr, FSI_RxTestParams  *rxTestParams);
static void Fsi_appRxCallback(void *args);
/* ========================================================================== */
/*                         Function Definitions                                 */
/* ========================================================================== */

void fsi_rx_main(void *args)
{
    int32_t status;
    uint32_t rxBaseAddr, rxBufAddr;
    uint16_t dataSize, frameTag, wordCnt;
    uint16_t bufIdx, rxHwComputedCrc, bufPtrLoc;
    uint16_t *tmpBufAddr;
    SemaphoreP_Object *p_taskDoneSemaphoreObj;
    FSI_RxTestParams  *rxTestParams = (FSI_RxTestParams  *)args;

    /* Test parameters */
    rxBaseAddr = rxTestParams->baseAddr;;
    dataSize = rxTestParams->frameDataSize;
    bufIdx = 0U;
    status = SystemP_SUCCESS;
    p_taskDoneSemaphoreObj = &rxTestParams->taskDoneSemaphoreObj;

    DebugP_log("[FSI] Loopback RX application test started ...\r\n");

    /* RX init and reset */
    status += FSI_resetRxModule(rxBaseAddr, FSI_RX_MAIN_CORE_RESET);
    FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_MAIN_CORE_RESET);
    status = FSI_performRxInitialization(rxBaseAddr);

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    if(rxTestParams->udataFilterTest == TRUE)
    {
        FSI_enableRxDataFilter(rxBaseAddr);
        FSI_setRxUserDataRefValue(rxBaseAddr, rxTestParams->userData);
        FSI_setRxUserDataBitMask(rxBaseAddr, FSI_APP_RX_USER_DATA_BIT_MASK);
    }
    if(rxTestParams->rxTriggerTest == TRUE)
    {
        FSI_enableAndConfigRxTrigCtrl(rxBaseAddr, FSI_RX_TRIG_CTRL_REG_0,
                                      FSI_RX_TRIGGER_CTRL_SEL_DATA_PACKET,
                                      FSI_APP_RX_TRIGGER_DELAY_IN_CYCLES);
    }
#endif

    /* Setting for requested transfer params */
    status += FSI_setRxSoftwareFrameSize(rxBaseAddr, dataSize);
    status += FSI_setRxDataWidth(rxBaseAddr, rxTestParams->numLane);
    status += FSI_setRxBufferPtr(rxBaseAddr, bufIdx);
    status += FSI_getRxBufferPtr(rxBaseAddr, &bufPtrLoc);
    DebugP_assert(bufPtrLoc == bufIdx);

    status += FSI_enableRxInternalLoopback(rxBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);
    /* Enable loopback */
    if (rxTestParams->rxFrameWDTest != TRUE)
    {
        /* Interrupt Init */
        status += Fsi_appRxIntrInit(rxBaseAddr, rxTestParams);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    else
    {
        /* Performing a reset on frame WD */
        FSI_resetRxModule(rxBaseAddr, FSI_RX_FRAME_WD_CNT_RESET);
        ClockP_usleep(1U);
        FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_FRAME_WD_CNT_RESET);
        /* Interrupt Init */
        status += Fsi_appRxIntrInit(rxBaseAddr, rxTestParams);
        DebugP_assert(status == SystemP_SUCCESS);
        /* Value to generate a Frame WD timeout interrupt event */
        FSI_enableRxFrameWatchdog(rxBaseAddr, 0x10000);
    }

    if (rxTestParams->delayLineCtrlTest == TRUE)
    {
        FSI_configRxDelayLine(rxBaseAddr, FSI_RX_DELAY_CLK, 5U);
        FSI_configRxDelayLine(rxBaseAddr, FSI_RX_DELAY_D0, 5U);
        FSI_configRxDelayLine(rxBaseAddr, FSI_RX_DELAY_D1, 5U);
    }

    /* post semaphore to sync with tx */
    SemaphoreP_post(&gFsiTxRx_SyncSemaphoreObj);

    /* Memset RX buffer with 0 for every loop */
    for (uint32_t i = 0; i < dataSize; i++)
    {
        gRxBufData[i] = 0;
    }

    FSI_getRxWordCount(rxBaseAddr, &wordCnt);

    SemaphoreP_pend(&gFsiRxSemObject, SystemP_WAIT_FOREVER);

    if (rxTestParams->rxFrameWDTest != TRUE)
    {
        /* Read data */
        status = FSI_readRxBuffer(rxBaseAddr, gRxBufData, dataSize, bufIdx);
        DebugP_assert(status == SystemP_SUCCESS);
        status = FSI_getRxFrameTag(rxBaseAddr, &frameTag);
        DebugP_assert(status == SystemP_SUCCESS);

        FSI_getRxBufferAddress(rxBaseAddr, &rxBufAddr);
        tmpBufAddr = (uint16_t *)rxBufAddr;
        for (uint32_t i = 0; i < dataSize; i++)
        {
            DebugP_assert(gRxBufData[i] == tmpBufAddr[i]);
        }

        FSI_getRxComputedCRC(rxBaseAddr, &rxHwComputedCrc);

        if (rxTestParams->userDefinedCrc == TRUE)
        {
            uint16_t receivedCrcVal;
            FSI_getRxReceivedCRC(rxBaseAddr, &receivedCrcVal);
            DebugP_assert(receivedCrcVal == FSI_APP_TX_PATTERN_USER_CRC_VALUE);
        }

        if (rxTestParams->testEccFlag == TRUE)
        {
            /* ECC computation for 2 words */
            uint16_t getEccVal = 0U, rxEccLog;
            uint32_t rxData = gRxBufData[0U] | gRxBufData[1U] << 16U;

            FSI_getRxUserDefinedData(rxBaseAddr, &getEccVal);
            FSI_setRxECCComputeWidth(rxBaseAddr, FSI_16BIT_ECC_COMPUTE);
            FSI_setRxECCData(rxBaseAddr, rxData);
            FSI_setRxReceivedECCValue(rxBaseAddr, getEccVal);
            FSI_getRxECCLog(rxBaseAddr, &rxEccLog);
            DebugP_assert(rxEccLog == 0U);
        }
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
        if(rxTestParams->udataFilterTest == TRUE)
        {
            uint16_t userData;
            FSI_getRxUserDefinedData(rxBaseAddr, &userData);
            if ((userData & FSI_APP_RX_USER_DATA_BIT_MASK) !=
                (rxTestParams->userData & FSI_APP_RX_USER_DATA_BIT_MASK))
                {
                    DebugP_assert(FALSE);
                }
        }
#endif
    }
    else
    {
        DebugP_assert(gRxFrameWdTest == TRUE);
        gRxFrameWdTest = FALSE;
    }

    Fsi_appRxIntrDeInit(rxBaseAddr, rxTestParams);
    FSI_disableRxInternalLoopback(rxBaseAddr);
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    if(rxTestParams->udataFilterTest == TRUE)
    {
        FSI_disableRxDataFilter(rxBaseAddr);
    }
#endif

    SemaphoreP_post(p_taskDoneSemaphoreObj);
    TaskP_exit();
}

void fsi_rx_hwPingTest(void *args)
{
    int32_t status;
    uint32_t rxBaseAddr;
    uint16_t rxPingTag = 0U;
    FSI_FrameType frameType;
    SemaphoreP_Object *p_taskDoneSemaphoreObj;
    FSI_RxTestParams  *rxTestParams = (FSI_RxTestParams  *)args;

    /* Test parameters */
    rxBaseAddr = rxTestParams->baseAddr;
    status = SystemP_SUCCESS;
    p_taskDoneSemaphoreObj = &rxTestParams->taskDoneSemaphoreObj;

    DebugP_log("[FSI] Loopback RX application test started ...\r\n");

    /* RX init and reset */
    status += FSI_resetRxModule(rxBaseAddr, FSI_RX_MAIN_CORE_RESET);
    FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_MAIN_CORE_RESET);
    status = FSI_performRxInitialization(rxBaseAddr);

    status += FSI_setRxDataWidth(rxBaseAddr, rxTestParams->numLane);
    status += FSI_setRxPingTimeoutMode(rxBaseAddr, FSI_PINGTIMEOUT_ON_HWINIT_PING_FRAME);

    /* Enable loopback */
    status += FSI_enableRxInternalLoopback(rxBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    if (rxTestParams->rxPingWDTest != TRUE)
    {
        /* Interrupt Init */
        status += Fsi_appRxIntrInit(rxBaseAddr, rxTestParams);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    else
    {
        /* Performing a reset on frame WD */
        FSI_resetRxModule(rxBaseAddr, FSI_RX_PING_WD_CNT_RESET);
        ClockP_usleep(1U);
        FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_PING_WD_CNT_RESET);
        /* Interrupt Init */
        status += Fsi_appRxIntrInit(rxBaseAddr, rxTestParams);
        DebugP_assert(status == SystemP_SUCCESS);
        /* Value to generate a Frame WD timeout interrupt event */
        FSI_enableRxPingWatchdog(rxBaseAddr, 0x10000);
    }

    /* post semaphore to sync with tx */
    SemaphoreP_post(&gFsiTxRx_SyncSemaphoreObj);

    SemaphoreP_pend(&gFsiRxSemObject, SystemP_WAIT_FOREVER);

    if (rxTestParams->rxPingWDTest != TRUE)
    {
        /* Read PinTag which should be same as TXPingTag */
        FSI_getRxFrameType(rxBaseAddr, &frameType);
        DebugP_assert(frameType == FSI_FRAME_TYPE_PING);
        FSI_getRxPingTag(rxBaseAddr, &rxPingTag);
        DebugP_assert(rxPingTag == FSI_FRAME_TAG10);
    }
    else
    {
        DebugP_assert(gRxPingWdTest == TRUE);
        gRxPingWdTest = FALSE;
    }

    Fsi_appRxIntrDeInit(rxBaseAddr, rxTestParams);
    FSI_disableRxInternalLoopback(rxBaseAddr);

    SemaphoreP_post(p_taskDoneSemaphoreObj);
    TaskP_exit();
}

void fsi_rx_negativeTest(void *args)
{
    SemaphoreP_Object *p_taskDoneSemaphoreObj;
    FSI_RxTestParams  *rxTestParams = (FSI_RxTestParams  *)args;
    uint32_t rxBaseAddr, wdgCntr, eccCorrectedData;
    int32_t  status;
    uint16_t rxBuf, regVal;

    /* Test parameters */
    rxBaseAddr = rxTestParams->baseAddr;
    p_taskDoneSemaphoreObj = &rxTestParams->taskDoneSemaphoreObj;

    status = FSI_setRxSoftwareFrameSize(rxBaseAddr, 0);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setRxSoftwareFrameSize(rxBaseAddr, FSI_MAX_LEN_NWORDS_DATA + 2);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setRxECCComputeWidth(rxBaseAddr, FSI_16BIT_ECC_COMPUTE + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setRxPingTimeoutMode(rxBaseAddr, FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxFrameType(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxFrameTag(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxUserDefinedData(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxEventStatus(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxEventStatus(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_forceRxEvents(rxBaseAddr, FSI_RX_EVTMASK + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVTMASK + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxReceivedCRC(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxComputedCRC(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setRxBufferPtr(rxBaseAddr, FSI_MAX_VALUE_BUF_PTR_OFF + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxBufferPtr(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxWordCount(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxFrameWatchdogCounter(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxPingWatchdogCounter(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxPingTag(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_setRxReceivedECCValue(rxBaseAddr, FSI_MAX_VALUE_USERDATA + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxECCCorrectedData(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxECCLog(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_enableRxInterrupt(rxBaseAddr, FSI_INT2, FSI_RX_EVTMASK + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_enableRxInterrupt(rxBaseAddr, FSI_INT2 + 1, FSI_RX_EVTMASK);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_disableRxInterrupt(rxBaseAddr, FSI_INT2, FSI_RX_EVTMASK + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_disableRxInterrupt(rxBaseAddr, FSI_INT2 + 1, FSI_RX_EVTMASK);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_getRxBufferAddress(rxBaseAddr, NULL_PTR);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_resetRxModule(rxBaseAddr, FSI_RX_PING_WD_CNT_RESET + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_PING_WD_CNT_RESET + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_readRxBuffer(rxBaseAddr, NULL_PTR,
                                (FSI_MAX_VALUE_BUF_PTR_OFF + (uint16_t)1U),
                                 FSI_MAX_VALUE_BUF_PTR_OFF);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_readRxBuffer(rxBaseAddr, &rxBuf,
                                (FSI_MAX_VALUE_BUF_PTR_OFF + (uint16_t)2U),
                                 FSI_MAX_VALUE_BUF_PTR_OFF);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_readRxBuffer(rxBaseAddr, &rxBuf,
                                (FSI_MAX_VALUE_BUF_PTR_OFF + (uint16_t)1U),
                                 FSI_MAX_VALUE_BUF_PTR_OFF + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_configRxDelayLine(rxBaseAddr, FSI_RX_DELAY_CLK, FSI_RX_MAX_DELAY_LINE_VAL + 1);
    DebugP_assert(status == CSL_EBADARGS);
    status = FSI_configRxDelayLine(rxBaseAddr, FSI_RX_DELAY_D1 + 1, FSI_RX_MAX_DELAY_LINE_VAL);
    DebugP_assert(status == CSL_EBADARGS);

    FSI_enableRxSPIPairing(rxBaseAddr);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__SPI_PAIRING_MASK) ==
                CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__SPI_PAIRING_MASK);
    FSI_disableRxSPIPairing(rxBaseAddr);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__SPI_PAIRING_MASK) ==
                0U);
    FSI_enableRxSPIMode(rxBaseAddr);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_OPER_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_OPER_CTRL_SPI_MODE_MASK) ==
                CSL_FSI_RX_CFG_RX_OPER_CTRL_SPI_MODE_MASK);
    FSI_disableRxSPIMode(rxBaseAddr);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_OPER_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_OPER_CTRL_SPI_MODE_MASK) ==
                0U);
    FSI_setRxECCComputeWidth(rxBaseAddr, FSI_32BIT_ECC_COMPUTE);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_OPER_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_OPER_CTRL_ECC_SEL_MASK) ==
                0U);
    FSI_setRxPingTimeoutMode(rxBaseAddr, FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_OPER_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_OPER_CTRL_PING_WD_RST_MODE_MASK) ==
                CSL_FSI_RX_CFG_RX_OPER_CTRL_PING_WD_RST_MODE_MASK);
    FSI_enableRxFrameWatchdog(rxBaseAddr, CONFIG_FSI_TX0_CLK / 50);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_EN_MASK) ==
                CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_EN_MASK);
    FSI_getRxFrameWatchdogCounter(rxBaseAddr, &wdgCntr);
    FSI_disableRxFrameWatchdog(rxBaseAddr);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_EN_MASK) ==
                0U);
    FSI_enableRxPingWatchdog(rxBaseAddr, CONFIG_FSI_TX0_CLK / 50);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_PING_WD_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_EN_MASK) ==
                CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_EN_MASK);
    FSI_getRxPingWatchdogCounter(rxBaseAddr, &wdgCntr);
    FSI_disableRxPingWatchdog(rxBaseAddr);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_PING_WD_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_EN_MASK) ==
                0U);
    FSI_getRxECCCorrectedData(rxBaseAddr, &eccCorrectedData);
    FSI_resetRxModule(rxBaseAddr, FSI_RX_FRAME_WD_CNT_RESET);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST_MASK) ==
                CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST_MASK);
    FSI_resetRxModule(rxBaseAddr, FSI_RX_PING_WD_CNT_RESET);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_PING_WD_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_RST_MASK) ==
                CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_RST_MASK);
    FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_FRAME_WD_CNT_RESET);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST_MASK) ==
                0U);
    FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_PING_WD_CNT_RESET);
    regVal = HW_RD_REG16(rxBaseAddr + CSL_FSI_RX_CFG_RX_PING_WD_CTRL);
    DebugP_assert((regVal & CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_RST_MASK) ==
                0U);
    FSI_forceRxEvents(rxBaseAddr, FSI_RX_EVT_TYPE_ERR);
    FSI_getRxEventStatus(rxBaseAddr, &regVal);
    DebugP_assert((regVal & FSI_RX_EVT_TYPE_ERR) == FSI_RX_EVT_TYPE_ERR);
    FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVT_TYPE_ERR);
    FSI_getRxEventStatus(rxBaseAddr, &regVal);
    DebugP_assert((regVal & FSI_RX_EVT_TYPE_ERR) == 0U);

    SemaphoreP_post(p_taskDoneSemaphoreObj);
    TaskP_exit();
}

static int32_t Fsi_appRxConfig(uint32_t rxBaseAddr, FSI_RxTestParams *rxTestParams)
{
    int32_t status;

    /* RX init and reset */
    status = FSI_performRxInitialization(rxBaseAddr);
    status += FSI_resetRxModule(rxBaseAddr, FSI_RX_MAIN_CORE_RESET);
    FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_MAIN_CORE_RESET);

    /* Setting for requested transfer params */
    status += FSI_setRxSoftwareFrameSize(rxBaseAddr, rxTestParams->frameDataSize);
    status += FSI_setRxDataWidth(rxBaseAddr, rxTestParams->numLane);
    status += FSI_setRxBufferPtr(rxBaseAddr, 0U);

    return status;
}

static int32_t Fsi_appRxIntrInit(uint32_t rxBaseAddr, FSI_RxTestParams *rxTestParams)
{
    int32_t status;
    uint32_t rxIntrNum;
    HwiP_Params rxHwiPrms;

    /*
     * RX interrupt config and registration
     */
    rxIntrNum = rxTestParams->intrLine;
    status = SemaphoreP_constructBinary(&gFsiRxSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    HwiP_Params_init(&rxHwiPrms);
    rxHwiPrms.intNum = rxIntrNum;
    rxHwiPrms.callback = Fsi_appRxCallback;
    rxHwiPrms.args = (void *)rxBaseAddr;
    HwiP_construct(&gFsiRxHwiObject, &rxHwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    /* Enable RX frame done interrupt */
    status += FSI_enableRxInterrupt(rxBaseAddr, rxTestParams->intrNum, rxTestParams->intrEvt);

    return status;
}

static void Fsi_appRxIntrDeInit(uint32_t rxBaseAddr, FSI_RxTestParams  *rxTestParams)
{
    /* RX interrupt deinit */
    FSI_disableRxInterrupt(rxBaseAddr, rxTestParams->intrNum, FSI_RX_EVTMASK);
    FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVTMASK);
    HwiP_destruct(&gFsiRxHwiObject);
    SemaphoreP_destruct(&gFsiRxSemObject);

    return;
}

static void Fsi_appRxCallback(void *args)
{
    uint32_t rxBaseAddr = (uint32_t)args;
    uint16_t intrStatus;

    FSI_getRxEventStatus(rxBaseAddr, &intrStatus);
    if ((intrStatus & FSI_RX_EVT_FRAME_WD_TIMEOUT) == FSI_RX_EVT_FRAME_WD_TIMEOUT)
    {
        gRxFrameWdTest = TRUE;
        FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVT_FRAME_WD_TIMEOUT);
        SemaphoreP_post(&gFsiRxSemObject);
    }
    if ((intrStatus & FSI_RX_EVT_PING_WD_TIMEOUT) == FSI_RX_EVT_PING_WD_TIMEOUT)
    {
        gRxPingWdTest = TRUE;
        FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVT_PING_WD_TIMEOUT);
        SemaphoreP_post(&gFsiRxSemObject);
    }
    if ((intrStatus & (FSI_RX_EVT_DATA_FRAME | FSI_RX_EVT_FRAME_DONE)) ==
                      (FSI_RX_EVT_DATA_FRAME | FSI_RX_EVT_FRAME_DONE))
    {
        FSI_clearRxEvents(rxBaseAddr,
                          (FSI_RX_EVT_DATA_FRAME | FSI_RX_EVT_FRAME_DONE));
        SemaphoreP_post(&gFsiRxSemObject);
    }
    if ((intrStatus & FSI_RX_EVT_PING_FRAME) == FSI_RX_EVT_PING_FRAME)
    {
        FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVT_PING_FRAME);
        SemaphoreP_post(&gFsiRxSemObject);
    }

    return;
}
