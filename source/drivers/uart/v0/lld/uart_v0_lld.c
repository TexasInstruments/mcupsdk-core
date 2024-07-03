/*
 * Copyright (C) 2021-2023 Texas Instruments Incorporated
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
 *  \file   uart_v0_lld.c
 *
 *  \brief  This file contains the implementation of UART driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/uart/v0/lld/uart_lld.h>
#include <drivers/uart/v0/lld/dma/uart_dma.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define UART_FIFO_CONFIG(txGra, rxGra, txTrig, rxTrig, txClr, rxClr, dmaEnPath, \
                         dmaMode)                                               \
    (((uint32_t) (txGra & 0xFU) << (uint32_t)26U)    |                       \
     ((uint32_t) (rxGra & 0xFU) << (uint32_t)22U)    |                       \
     ((uint32_t) (txTrig & 0xFFU) << (uint32_t)14U)  |                       \
     ((uint32_t) (rxTrig & 0xFFU) << (uint32_t)6U)   |                       \
     ((uint32_t) (txClr & 0x1U) << (uint32_t)5U)     |                       \
     ((uint32_t) (rxClr & 0x1U) << (uint32_t)4U)     |                       \
     ((uint32_t) (dmaEnPath & 0x1U) << (uint32_t)3U) |                       \
     (uint32_t) (dmaMode & 0x7U))

#define UART_FIFO_CONFIG_TXGRA          ((uint32_t) 0xFU << 26)
#define UART_FIFO_CONFIG_RXGRA          ((uint32_t) 0xFU << 22)
#define UART_FIFO_CONFIG_TXTRIG         ((uint32_t) 0xFFU << 14)
#define UART_FIFO_CONFIG_RXTRIG         ((uint32_t) 0xFFU << 6)
#define UART_FIFO_CONFIG_TXCLR          ((uint32_t) 0x1U << 5)
#define UART_FIFO_CONFIG_RXCLR          ((uint32_t) 0x1U << 4)
#define UART_FIFO_CONFIG_DMAENPATH      ((uint32_t) 0x1U << 3)
#define UART_FIFO_CONFIG_DMAMODE        ((uint32_t) 0x7U << 0)

#define UART_TRIG_LVL_GRANULARITY_4     ((uint32_t) 0x0000U)
#define UART_TRIG_LVL_GRANULARITY_1     ((uint32_t) 0x0001U)

#define UART_DMA_EN_PATH_FCR            (UART_SCR_DMA_MODE_CTL_DMA_MODE_CTL_VALUE_0)
#define UART_DMA_EN_PATH_SCR            (UART_SCR_DMA_MODE_CTL_DMA_MODE_CTL_VALUE_1)

#define UART_INT2_RX_EMPTY              (UART_IER2_EN_RXFIFO_EMPTY_MASK)
#define UART_INT2_TX_EMPTY              (UART_IER2_EN_TXFIFO_EMPTY_MASK)

#define UART_DMA_MODE_0_ENABLE          (UART_SCR_DMA_MODE_2_DMA_MODE_2_VALUE_0)
#define UART_DMA_MODE_1_ENABLE          (UART_SCR_DMA_MODE_2_DMA_MODE_2_VALUE_1)
#define UART_DMA_MODE_2_ENABLE          (UART_SCR_DMA_MODE_2_DMA_MODE_2_VALUE_2)
#define UART_DMA_MODE_3_ENABLE          (UART_SCR_DMA_MODE_2_DMA_MODE_2_VALUE_3)

#define UART_MIR_OVERSAMPLING_RATE_41   ((uint32_t) 41U)
#define UART_MIR_OVERSAMPLING_RATE_42   ((uint32_t) 42U)

#define UART_BREAK_COND_DISABLE         (UART_LCR_BREAK_EN_BREAK_EN_VALUE_0 \
                                            << UART_LCR_BREAK_EN_SHIFT)
#define UART_BREAK_COND_ENABLE          (UART_LCR_BREAK_EN_BREAK_EN_VALUE_1 \
                                            << UART_LCR_BREAK_EN_SHIFT)
#define UART_NO_HARDWARE_FLOW_CONTROL    (UART_EFR_HW_NO_FLOW_CONTROL_VALUE)
#define UART_RTS_ENABLE                  (UART_EFR_HW_ENABLE_RTS_VALUE)
#define UART_CTS_ENABLE                  (UART_EFR_HW_ENALE_CTS_VALUE)
#define UART_RTS_CTS_ENABLE              (UART_EFR_HW_ENABLE_RTS_CTS_FLOW_CONTROL_VALUE)

#define UART_TIMEOUTL                       (0x98U)
#define UART_TIMEOUTH                       (0x9CU)

#define UART_EFR2                             (0x8CU)
#define UART_EFR2_TIMEOUT_BEHAVE_SHIFT        (0x6U)
#define UART_EFR2_TIMEOUT_BEHAVE_MASK         (0x6U)

/* UART read line status timeout in micro seconds  */
#define UART_READ_LINE_STATUS_TIMEOUT_IN_US    (1U*1000U*1000U)
/* UART read timeout in micro seconds  */
#define UART_READ_TIMEOUT_IN_US                (500U)
/* UART module reset timeout in micro seconds  */
#define UART_MODULE_RESET_TIMEOUT_IN_US        (500U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static void UART_resetModule(UARTLLD_Handle hUart);
static void UART_writeDataPolling(UARTLLD_Handle hUart);
static uint32_t UART_fifoWrite(UARTLLD_Handle hUart,
                               const uint8_t    *buffer,
                               uint32_t          writeSizeRemaining);
/* Low level HW functions */
static uint32_t UART_enhanFuncEnable(uint32_t baseAddr);
static void UART_regConfModeRestore(uint32_t baseAddr, uint32_t lcrRegValue);
static void UART_modemControlReset(uint32_t baseAddr);
static void UART_moduleReset(UARTLLD_Handle hUart);
static uint32_t UART_subConfigTCRTLRModeEn(uint32_t baseAddr);
static void UART_enhanFuncBitValRestore(uint32_t baseAddr, uint32_t enhanFnBitVal);
static uint32_t UART_divisorLatchWrite(uint32_t baseAddr, uint32_t divisorValue);
static void UART_fifoRegisterWrite(uint32_t baseAddr, uint32_t fcrValue);
static void UART_tcrTlrBitValRestore(uint32_t baseAddr, uint32_t tcrTlrBitVal);
static uint32_t UART_fifoConfig(uint32_t baseAddr, uint32_t fifoConfig);
static inline uint32_t UART_divideRoundCloset(uint32_t divident, uint32_t divisor);
static uint32_t UART_divisorValCompute(uint32_t moduleClk,
                                      uint32_t baudRate,
                                      uint32_t modeFlag,
                                      uint32_t mirOverSampRate);
static void UART_lineCharConfig(uint32_t baseAddr,
                                 uint32_t wLenStbFlag,
                                 uint32_t parityFlag);
static void UART_divisorLatchDisable(uint32_t baseAddr);
static void UART_breakCtl(uint32_t baseAddr, uint32_t breakState);
static void UART_hardwareFlowCtrlOptSet(uint32_t baseAddr, uint32_t hwFlowCtrl);
static void UART_flowCtrlTrigLvlConfig(uint32_t baseAddr,
                               uint32_t rtsHaltFlag,
                               uint32_t rtsStartFlag);
static uint32_t UART_getRxError(uint32_t baseAddr);
static uint32_t UART_regConfigModeEnable(uint32_t baseAddr, uint32_t modeFlag);
static int32_t UART_readInterrupt(UARTLLD_Handle handle, UART_Transaction *trans);
static Bool UART_statusIsDataReady(UARTLLD_Handle handle);
static uint32_t UART_fifoRead(UARTLLD_Handle handle, uint8_t *buffer, uint32_t readSizeRemaining);
static void UART_readDataPolling(UARTLLD_Handle handle);
static int32_t UART_readPolling(UARTLLD_Handle handle, UART_Transaction *trans);
static int32_t UART_readPollingWithCounter(UARTLLD_Handle handle, UART_Transaction *trans);
static void UART_i2310WA(uint32_t baseAddr);

static void UART_configInstance(UARTLLD_Handle handle);
static int32_t UART_checkTransaction(UART_Transaction *trans);
static int32_t UART_writePolling(UARTLLD_Handle handle, UART_Transaction *trans);
static int32_t UART_writeInterrupt(UARTLLD_Handle handle);
static inline uint32_t UART_writeData(UARTLLD_Handle handle, uint32_t size);
static inline uint32_t UART_readData(UARTLLD_Handle handle, uint32_t size);
static uint8_t UART_readByte(UARTLLD_Handle handle);
static uint8_t UART_fifoCharGet(uint32_t baseAddr);
static void UART_timeGuardConfig(uint32_t baseAddr, uint32_t timeGuardVal);

/** \brief API to check if the Input Paramter is valid */
static inline int32_t UART_IsParameter(uint32_t InuptParameter);
/** \brief API to check if the Data length is valid */
static inline int32_t UART_IsDataLengthValid(uint32_t dataLength);
/** \brief API to check if the Number of stop bits is valid */
static inline int32_t UART_IsStopBitsValid(uint32_t stopBits);
/** \brief API to check if the Parity type is valid */
static inline int32_t UART_IsParityTypeValid(uint32_t parityType);
/** \brief API to check if the Parity type is valid */
static inline int32_t UART_IsHWFlowCtrlValid(uint32_t hwFlowControlThr);
/** \brief API to check if Operation mode is valid */
static inline int32_t UART_OperModeValid(uint32_t operMode);
/** \brief API to check if the RX Trigger level is valid */
static inline int32_t UART_IsRxTrigLvlValid(uint32_t rxTrigLvl);
/** \brief API to check if the TX Trigger level is valid */
static inline int32_t UART_IsTxTrigLvlValid(uint32_t txTrigLvl);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static inline int32_t UART_IsParameter(uint32_t InuptParameter)
{
    int32_t status = UART_STATUS_SUCCESS;
    if(InuptParameter == 0U)
    {
        status = UART_INVALID_PARAM;
    }
    return status;
}

static inline int32_t UART_IsDataLengthValid(uint32_t dataLength)
{
    int32_t status = UART_INVALID_PARAM;
    if(((dataLength == UART_LEN_5) ||
        (dataLength == UART_LEN_6) ||
        (dataLength == UART_LEN_7) ||
        (dataLength == UART_LEN_8)))
    {
        status = UART_STATUS_SUCCESS;
    }
    return status;
}

static inline int32_t UART_IsStopBitsValid(uint32_t stopBits)
{
    int32_t status = UART_INVALID_PARAM;
    if((stopBits == UART_STOPBITS_1) ||
        (stopBits == UART_STOPBITS_2))
    {
        status = UART_STATUS_SUCCESS;
    }
    return status;
}

static inline int32_t UART_IsParityTypeValid(uint32_t parityType)
{
    int32_t status = UART_INVALID_PARAM;
    if(((parityType == UART_PARITY_NONE)    ||
        (parityType == UART_PARITY_ODD)     ||
        (parityType == UART_PARITY_EVEN)    ||
        (parityType == UART_PARITY_FORCED0) ||
        (parityType == UART_PARITY_FORCED1)))
    {
        status = UART_STATUS_SUCCESS;
    }
    return status;
}

static inline int32_t UART_IsHWFlowCtrlValid(uint32_t hwFlowControlThr)
{
    int32_t status = UART_INVALID_PARAM;
    if(((hwFlowControlThr == UART_RXTRIGLVL_1)  ||
        (hwFlowControlThr == UART_RXTRIGLVL_8)  ||
        (hwFlowControlThr == UART_RXTRIGLVL_16) ||
        (hwFlowControlThr == UART_RXTRIGLVL_56) ||
        (hwFlowControlThr == UART_RXTRIGLVL_60)))
    {
        status = UART_STATUS_SUCCESS;
    }
    return status;
}

static inline int32_t UART_OperModeValid(uint32_t operMode)
{
    int32_t status = UART_INVALID_PARAM;
    if(((operMode == UART_OPER_MODE_16X) ||
        (operMode == UART_OPER_MODE_SIR) ||
        (operMode == UART_OPER_MODE_16X_AUTO_BAUD) ||
        (operMode == UART_OPER_MODE_13X) ||
        (operMode == UART_OPER_MODE_MIR) ||
        (operMode == UART_OPER_MODE_FIR) ||
        (operMode == UART_OPER_MODE_CIR) ||
        (operMode == UART_OPER_MODE_DISABLED)))
    {
        status = UART_STATUS_SUCCESS;
    }
    return status;
}

static inline int32_t UART_IsRxTrigLvlValid(uint32_t rxTrigLvl)
{
    int32_t status = UART_INVALID_PARAM;
    if(((rxTrigLvl == UART_RXTRIGLVL_1)  ||
        (rxTrigLvl == UART_RXTRIGLVL_8)  ||
        (rxTrigLvl == UART_RXTRIGLVL_16) ||
        (rxTrigLvl == UART_RXTRIGLVL_56) ||
        (rxTrigLvl == UART_RXTRIGLVL_60)))
    {
        status = UART_STATUS_SUCCESS;
    }
    return status;
}

static inline int32_t UART_IsTxTrigLvlValid(uint32_t txTrigLvl)
{
    int32_t status = UART_INVALID_PARAM;
    if(((txTrigLvl == UART_TXTRIGLVL_1)  ||
        (txTrigLvl == UART_TXTRIGLVL_8)  ||
        (txTrigLvl == UART_TXTRIGLVL_16) ||
        (txTrigLvl == UART_TXTRIGLVL_32) ||
        (txTrigLvl == UART_TXTRIGLVL_56)))
    {
        status = UART_STATUS_SUCCESS;
    }
    return status;
}

void UART_configInstance(UARTLLD_Handle hUart)
{
    uint32_t                baseAddr;
    uint32_t                regVal, divisorVal, wLenStbFlag, parityFlag;
    UARTLLD_InitHandle         hUartInit;

    baseAddr = hUart->baseAddr;
    hUartInit = hUart->hUartInit;

    /* Reset module */
    UART_resetModule(hUart);

    /* Set up the TX and RX FIFO Trigger levels. */
    if(UART_CONFIG_MODE_DMA == hUartInit->transferMode)
    {
        regVal = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_1,
                                UART_TRIG_LVL_GRANULARITY_1,
                                hUartInit->txTrigLvl,
                                hUartInit->rxTrigLvl,
                                1U,
                                1U,
                                UART_DMA_EN_PATH_FCR,
                                UART_DMA_MODE_1_ENABLE);
    }
    else
    {
        regVal = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_1,
                                UART_TRIG_LVL_GRANULARITY_1,
                                hUartInit->txTrigLvl,
                                hUartInit->rxTrigLvl,
                                1U,
                                1U,
                                UART_DMA_EN_PATH_FCR,
                                UART_DMA_MODE_0_ENABLE);
    }

    /* Configuring the FIFO settings. */
    (void)UART_fifoConfig(baseAddr, regVal);

    /* Configure the TIMEGUARD settings */
    UART_timeGuardConfig(baseAddr, hUartInit->timeGuardVal);

    /* Computing the Divisor Value for params.baudRate */
    divisorVal = UART_divisorValCompute(hUartInit->inputClkFreq,
                                    hUartInit->baudRate,
                                    hUartInit->operMode,
                                    UART_MIR_OVERSAMPLING_RATE_42);
    /* Configuring the Baud Rate settings. */
    (void)UART_divisorLatchWrite(baseAddr, divisorVal);

    /* Switching to Configuration Mode B. */
    (void)UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics */
    wLenStbFlag = (hUartInit->dataLength << UART_LCR_CHAR_LENGTH_SHIFT);
    wLenStbFlag |= (hUartInit->stopBits << UART_LCR_NB_STOP_SHIFT);
    parityFlag = (hUartInit->parityType << UART_LCR_PARITY_EN_SHIFT);
    UART_lineCharConfig(baseAddr, wLenStbFlag, parityFlag);

    /* Disable write access to Divisor Latches. */
    UART_divisorLatchDisable(baseAddr);

    /* Disabling Break Control. */
    UART_breakCtl(baseAddr, UART_BREAK_COND_DISABLE);

    /* Set UART operating mode */
    (void)UART_operatingModeSelect(baseAddr, hUartInit->operMode);

    if (hUartInit->hwFlowControl == (uint32_t)TRUE)
    {

        UART_hardwareFlowCtrlOptSet(baseAddr, UART_RTS_CTS_ENABLE);
        /* In case of HW flow control, the programmer must ensure that the
        trigger level to halt transmission is greater than or equal to the
        RX FIFO trigger level */
        if (hUartInit->hwFlowControlThr >= hUartInit->rxTrigLvl)
        {
            UART_flowCtrlTrigLvlConfig(baseAddr,
                                    hUartInit->hwFlowControlThr,
                                    hUartInit->rxTrigLvl);
        }
    }
    else
    {
        UART_hardwareFlowCtrlOptSet(baseAddr, UART_NO_HARDWARE_FLOW_CONTROL);
    }

    return;
}

int32_t UART_checkTransaction(UART_Transaction *trans)
{
    int32_t     status = UART_TRANSFER_STATUS_SUCCESS;

    if(0U == trans->count)
    {
        /* Transfer count should be positive */
        trans->status = UART_TRANSFER_STATUS_ERROR_OTH;
        status = UART_TRANSFER_STATUS_FAILURE;
    }
    if(NULL == trans->buf)
    {
        status = UART_TRANSFER_STATUS_FAILURE;
    }

    return (status);
}

static void UART_timeGuardConfig(uint32_t baseAddr, uint32_t timeGuardVal)
{
    /* Programming the TIMEGUARD Register */
    HW_WR_FIELD32(baseAddr + UART_TIMEGUARD, UART_TIMEGUARD_TIMEGUARD,
                  timeGuardVal >> UART_TIMEGUARD_TIMEGUARD_SHIFT);
}

uint32_t UART_operatingModeSelect(uint32_t baseAddr, uint32_t modeFlag)
{
    uint32_t operMode;

    operMode = HW_RD_REG32(baseAddr + UART_MDR1) & UART_MDR1_MODE_SELECT_MASK;

    /* Programming the MODESELECT field in MDR1. */
    HW_WR_FIELD32(baseAddr + UART_MDR1, UART_MDR1_MODE_SELECT,
                  modeFlag >> UART_MDR1_MODE_SELECT_SHIFT);

    return operMode;
}

static void UART_resetModule(UARTLLD_Handle hUart)
{
    /* Switch to mode B to access EFR */
    /* Set the ENHANCEDEN Bit Field to Enable access to the MCR & IER reg
     * Setting the EFR[4] bit to 1 */
    (void)UART_enhanFuncEnable(hUart->baseAddr);
    /* Force LCR[6] to zero, to avoid UART breaks and LCR[7] to zero to access
     * MCR reg */
    UART_regConfModeRestore(hUart->baseAddr, 0x00U);
    /* RESET MCR Reg */
    UART_modemControlReset(hUart->baseAddr);

    /* Disable all interrupts */
    UART_intrDisable(hUart->baseAddr, 0xFFU);
    UART_intr2Disable(hUart->baseAddr, UART_INT2_TX_EMPTY);

    /* Put the module in Disable State */
    (void)UART_operatingModeSelect(hUart->baseAddr, UART_OPER_MODE_DISABLED);

    /* Reset Uart and setup hardware params */
    UART_moduleReset(hUart);

    return;
}

static uint32_t UART_enhanFuncEnable(uint32_t baseAddr)
{
    uint32_t enhanFnBitVal;
    uint32_t lcrRegValue;

    /* Enabling Configuration Mode B of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of ENHANCEDEN bit of EFR. */
    enhanFnBitVal = HW_RD_REG32(baseAddr + UART_EFR) & UART_EFR_ENHANCED_EN_MASK;

    /* Setting the ENHANCEDEN bit in EFR register. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

    /* Programming LCR with the collected value. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return enhanFnBitVal;
}

static void UART_regConfModeRestore(uint32_t baseAddr, uint32_t lcrRegValue)
{
    /* Programming the Line Control Register(LCR). */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static void UART_modemControlReset(uint32_t baseAddr)
{
    uint32_t mcrResetVal = 0U;
    /* Resetting bits of MCR. */
    HW_WR_REG32(baseAddr + UART_MCR, mcrResetVal);
}

static void UART_moduleReset(UARTLLD_Handle hUart)
{
    uint32_t startTicks, elapsedTicks = 0;
    UARTLLD_InitHandle        hUartInit;

    hUartInit = hUart->hUartInit;

    /* Performing Software Reset of the module. */
    HW_WR_FIELD32(hUart->baseAddr + UART_SYSC, UART_SYSC_SOFTRESET,
                  UART_SYSC_SOFTRESET_SOFTRESET_VALUE_1);

    startTicks = hUartInit->clockP_get();
    /* Wait until the process of Module Reset is complete. */
    while ((0U == HW_RD_FIELD32(hUart->baseAddr + UART_SYSS, UART_SYSS_RESETDONE)) && (elapsedTicks < hUartInit->clockP_usecToTick(UART_MODULE_RESET_TIMEOUT_IN_US)))
    {
        elapsedTicks = hUartInit->clockP_get() - startTicks;
    }
}

static uint32_t UART_subConfigTCRTLRModeEn(uint32_t baseAddr)
{
    uint32_t enhanFnBitVal;
    uint32_t tcrTlrValue;
    uint32_t lcrRegValue;

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of EFR[4] and later setting it. */
    enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode A. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Collecting the bit value of MCR[6]. */
    tcrTlrValue = HW_RD_REG32(baseAddr + UART_MCR) & UART_MCR_TCR_TLR_MASK;

    /* Setting the TCRTLR bit in Modem Control Register(MCR). */
    HW_WR_FIELD32(baseAddr + UART_MCR, UART_MCR_TCR_TLR,
                  UART_MCR_TCR_TLR_TCR_TLR_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Restoring the value of EFR[4] to its original value. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return tcrTlrValue;
}

static void UART_enhanFuncBitValRestore(uint32_t baseAddr, uint32_t enhanFnBitVal)
{
    uint32_t lcrRegValue;

    /* Enabling Configuration Mode B of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Restoring the value of EFR[4]. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  enhanFnBitVal >> UART_EFR_ENHANCED_EN_SHIFT);

    /* Programming LCR with the collected value. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static uint32_t UART_divisorLatchWrite(uint32_t baseAddr, uint32_t divisorValue)
{
    volatile uint32_t enhanFnBitVal;
    volatile uint32_t sleepMdBitVal;
    volatile uint32_t lcrRegValue;
    volatile uint32_t operMode;
    uint32_t          divRegVal;

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of EFR[4] and later setting it. */
    enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Operational Mode. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_OPERATIONAL_MODE);

    /*
    ** Collecting the current value of IER[4](SLEEPMODE bit) and later
    ** clearing it.
    */
    sleepMdBitVal = HW_RD_FIELD32(baseAddr + UART_IER, UART_IER_SLEEP_MODE);

    HW_WR_FIELD32(baseAddr + UART_IER, UART_IER_SLEEP_MODE, 0U);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of Divisor Latch Registers. */
    divRegVal  = HW_RD_REG32(baseAddr + UART_DLL) & 0xFFU;
    divRegVal |= (HW_RD_REG32(baseAddr + UART_DLH) & 0x3FU) << 8;

    /* Switch the UART instance to Disabled state. */
    operMode = UART_operatingModeSelect(baseAddr,
                                       (uint32_t) UART_MDR1_MODE_SELECT_MASK);

    /* Writing to Divisor Latch Low(DLL) register. */
    HW_WR_REG32(baseAddr + UART_DLL, divisorValue & 0x00FFU);

    /* Writing to Divisor Latch High(DLH) register. */
    HW_WR_REG32(baseAddr + UART_DLH, (divisorValue & 0x3F00U) >> 8);

    /* Restoring the Operating Mode of UART. */
    (void) UART_operatingModeSelect(baseAddr, operMode);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Operational Mode. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_OPERATIONAL_MODE);

    /* Restoring the value of IER[4] to its original value. */
    HW_WR_FIELD32(baseAddr + UART_IER, UART_IER_SLEEP_MODE, sleepMdBitVal);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Restoring the value of EFR[4] to its original value. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);
    /* Restoring the value of LCR Register. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return divRegVal;
}

static void UART_fifoRegisterWrite(uint32_t baseAddr, uint32_t fcrValue)
{
    uint32_t divLatchRegVal;
    uint32_t enhanFnBitVal;
    uint32_t lcrRegValue;
    uint32_t isTxRxFifoEmpty = FALSE;

    /* Switching to Register Configuration Mode A of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Clearing the contents of Divisor Latch Registers. */
    divLatchRegVal = UART_divisorLatchWrite(baseAddr, 0x0000U);

    /* Set the EFR[4] bit to 1. */
    enhanFnBitVal = UART_enhanFuncEnable(baseAddr);

    /* Writing the 'fcrValue' to the FCR register. */
    HW_WR_REG32(baseAddr + UART_FCR, fcrValue);

    while(isTxRxFifoEmpty == FALSE)
    {
        isTxRxFifoEmpty = UART_IsTxRxFifoEmpty(baseAddr);
    }

    /* Restoring the value of EFR[4] to its original value. */
    UART_enhanFuncBitValRestore(baseAddr, enhanFnBitVal);

    /* Programming the Divisor Latch Registers with the collected value. */
    (void) UART_divisorLatchWrite(baseAddr, divLatchRegVal);

    /* Reinstating LCR with its original value. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static void UART_tcrTlrBitValRestore(uint32_t baseAddr, uint32_t tcrTlrBitVal)
{
    uint32_t enhanFnBitVal;
    uint32_t lcrRegValue;

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Collecting the current value of EFR[4] and later setting it. */
    enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                  UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Configuration Mode A of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Programming MCR[6] with the corresponding bit value in 'tcrTlrBitVal'. */
    HW_WR_FIELD32(baseAddr + UART_MCR, UART_MCR_TCR_TLR, tcrTlrBitVal);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switching to Register Configuration Mode B. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Restoring the value of EFR[4] to its original value. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static uint32_t UART_fifoConfig(uint32_t baseAddr, uint32_t fifoConfig)
{
    uint32_t enhanFnBitVal;
    uint32_t tcrTlrBitVal;
    uint32_t tlrValue;
    uint32_t fcrValue = 0U;
    uint32_t txGra = (fifoConfig & UART_FIFO_CONFIG_TXGRA) >> 26;
    uint32_t rxGra = (fifoConfig & UART_FIFO_CONFIG_RXGRA) >> 22;
    uint32_t txTrig = (fifoConfig & UART_FIFO_CONFIG_TXTRIG) >> 14;
    uint32_t rxTrig = (fifoConfig & UART_FIFO_CONFIG_RXTRIG) >> 6;
    uint32_t txClr = (fifoConfig & UART_FIFO_CONFIG_TXCLR) >> 5;
    uint32_t rxClr = (fifoConfig & UART_FIFO_CONFIG_RXCLR) >> 4;

    uint32_t dmaEnPath = (fifoConfig & UART_FIFO_CONFIG_DMAENPATH) >> 3;
    uint32_t dmaMode   = (fifoConfig & UART_FIFO_CONFIG_DMAMODE);

    /* Setting the EFR[4] bit to 1. */
    enhanFnBitVal = UART_enhanFuncEnable(baseAddr);

    tcrTlrBitVal = UART_subConfigTCRTLRModeEn(baseAddr);

    /* Enable FIFO */
    fcrValue |= UART_FCR_FIFO_EN_MASK;

    /* Setting the Receiver FIFO trigger level. */
    if(UART_TRIG_LVL_GRANULARITY_1 != rxGra)
    {
        /* Clearing the RXTRIGGRANU1 bit in SCR. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_RX_TRIG_GRANU1,
                      UART_SCR_RX_TRIG_GRANU1_RX_TRIG_GRANU1_VALUE_0);

        /* Clearing the RX_FIFO_TRIG_DMA field of TLR register. */
        HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_RX_FIFO_TRIG_DMA,
                      0U);

        fcrValue &= ~((uint32_t) UART_FCR_RX_FIFO_TRIG_MASK);

        /*
        ** Checking if 'rxTrig' matches with the RX Trigger level values
        ** in FCR.
        */
        if((UART_RXTRIGLVL_8 == rxTrig) ||
           (UART_RXTRIGLVL_16 == rxTrig) ||
           (UART_RXTRIGLVL_56 == rxTrig) ||
           (UART_RXTRIGLVL_60 == rxTrig))
        {
            fcrValue |= rxTrig & UART_FCR_RX_FIFO_TRIG_MASK;
        }
        else
        {
            /* RX Trigger level will be a multiple of 4. */
            /* Programming the RX_FIFO_TRIG_DMA field of TLR register. */
            HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_RX_FIFO_TRIG_DMA,
                          rxTrig);
        }
    }
    else
    {
        /* 'rxTrig' now has the 6-bit RX Trigger level value. */

        rxTrig &= 0x003FU;

        /* Collecting the bits rxTrig[5:2]. */
        tlrValue = (rxTrig & 0x003CU) >> 2;

        /* Collecting the bits rxTrig[1:0] and writing to 'fcrValue'. */
        fcrValue |= (rxTrig & 0x0003U) << UART_FCR_RX_FIFO_TRIG_SHIFT;

        /* Setting the RXTRIGGRANU1 bit of SCR register. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_RX_TRIG_GRANU1,
                      UART_SCR_RX_TRIG_GRANU1_RX_TRIG_GRANU1_VALUE_1);

        /* Programming the RX_FIFO_TRIG_DMA field of TLR register. */
        HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_RX_FIFO_TRIG_DMA, tlrValue);
    }

    /* Setting the Transmitter FIFO trigger level. */
    if(UART_TRIG_LVL_GRANULARITY_1 != txGra)
    {
        /* Clearing the TXTRIGGRANU1 bit in SCR. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_TX_TRIG_GRANU1,
                      UART_SCR_TX_TRIG_GRANU1_TX_TRIG_GRANU1_VALUE_0);

        /* Clearing the TX_FIFO_TRIG_DMA field of TLR register. */
        HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_TX_FIFO_TRIG_DMA,
                      0U);

        fcrValue &= ~((uint32_t) UART_FCR_TX_FIFO_TRIG_MASK);

        /*
        ** Checking if 'txTrig' matches with the TX Trigger level values
        ** in FCR.
        */
        if((UART_TXTRIGLVL_8 == (txTrig)) ||
           (UART_TXTRIGLVL_16 == (txTrig)) ||
           (UART_TXTRIGLVL_32 == (txTrig)) ||
           (UART_TXTRIGLVL_56 == (txTrig)))
        {
            fcrValue |= txTrig & UART_FCR_TX_FIFO_TRIG_MASK;
        }
        else
        {
            /* TX Trigger level will be a multiple of 4. */
            /* Programming the TX_FIFO_TRIG_DMA field of TLR register. */
            HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_TX_FIFO_TRIG_DMA,
                          txTrig);
        }
    }
    else
    {
        /* 'txTrig' now has the 6-bit TX Trigger level value. */

        txTrig &= 0x003FU;

        /* Collecting the bits txTrig[5:2]. */
        tlrValue = (txTrig & 0x003CU) >> 2;

        /* Collecting the bits txTrig[1:0] and writing to 'fcrValue'. */
        fcrValue |= (txTrig & 0x0003U) << UART_FCR_TX_FIFO_TRIG_SHIFT;

        /* Setting the TXTRIGGRANU1 bit of SCR register. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_TX_TRIG_GRANU1,
                      UART_SCR_TX_TRIG_GRANU1_TX_TRIG_GRANU1_VALUE_1);

        /* Programming the TX_FIFO_TRIG_DMA field of TLR register. */
        HW_WR_FIELD32(baseAddr + UART_TLR, UART_TLR_TX_FIFO_TRIG_DMA, tlrValue);
    }

    if(UART_DMA_EN_PATH_FCR == dmaEnPath)
    {
        /* Configuring the UART DMA Mode through FCR register. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_DMA_MODE_CTL,
                      UART_SCR_DMA_MODE_CTL_DMA_MODE_CTL_VALUE_0);

        dmaMode &= 0x1U;

        /* Clearing the bit corresponding to the DMA_MODE in 'fcrValue'. */
        fcrValue &= ~((uint32_t) UART_FCR_DMA_MODE_MASK);

        /* Setting the DMA Mode of operation. */
        fcrValue |= dmaMode << UART_FCR_DMA_MODE_SHIFT;
    }
    else
    {
        dmaMode &= 0x3U;

        /* Configuring the UART DMA Mode through SCR register. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_DMA_MODE_CTL,
                      UART_SCR_DMA_MODE_CTL_DMA_MODE_CTL_VALUE_1);

        /* Programming the DMAMODE2 field in SCR. */
        HW_WR_FIELD32(baseAddr + UART_SCR, UART_SCR_DMA_MODE_2, dmaMode);
    }

    /* Programming the bits which clear the RX and TX FIFOs. */
    fcrValue |= rxClr << UART_FCR_RX_FIFO_CLEAR_SHIFT;
    fcrValue |= txClr << UART_FCR_TX_FIFO_CLEAR_SHIFT;

    /* Writing 'fcrValue' to the FIFO Control Register(FCR). */
    UART_fifoRegisterWrite(baseAddr, fcrValue);

    /* Restoring the value of TCRTLR bit in MCR. */
    UART_tcrTlrBitValRestore(baseAddr, tcrTlrBitVal);

    /* Restoring the value of EFR[4] to the original value. */
    UART_enhanFuncBitValRestore(baseAddr, enhanFnBitVal);

    return fcrValue;
}

static inline uint32_t UART_divideRoundCloset(uint32_t divident, uint32_t divisor)
{
    return ((divident + (divisor/2U))/divisor);
}

static uint32_t UART_divisorValCompute(uint32_t moduleClk,
                                      uint32_t baudRate,
                                      uint32_t modeFlag,
                                      uint32_t mirOverSampRate)
{
    uint32_t divisorValue = 0U;
    uint32_t tempModeFlag = modeFlag & UART_MDR1_MODE_SELECT_MASK;

    switch (tempModeFlag)
    {
        case UART_OPER_MODE_16X:
        case UART_OPER_MODE_SIR:
            divisorValue = UART_divideRoundCloset(moduleClk, 16U * baudRate);
            break;

        case UART_OPER_MODE_13X:
            divisorValue = UART_divideRoundCloset(moduleClk, 13U * baudRate);
            break;

        case UART_OPER_MODE_MIR:
            divisorValue = UART_divideRoundCloset(moduleClk, mirOverSampRate * baudRate);
            break;

        case UART_OPER_MODE_FIR:
            divisorValue = 0U;
            break;

        default:
            break;
    }

    return divisorValue;
}

static void UART_lineCharConfig(uint32_t baseAddr,
                                 uint32_t wLenStbFlag,
                                 uint32_t parityFlag)
{
    uint32_t lcrRegValue;

    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
    /* Clearing the CHAR_LENGTH and NB_STOP fields in LCR.*/
    lcrRegValue &= ~((uint32_t) UART_LCR_NB_STOP_MASK | (uint32_t) UART_LCR_CHAR_LENGTH_MASK);

    /* Programming the CHAR_LENGTH and NB_STOP fields in LCR. */
    lcrRegValue |= wLenStbFlag & (UART_LCR_NB_STOP_MASK |
                                  UART_LCR_CHAR_LENGTH_MASK);

    /* Clearing the PARITY_EN, PARITY_TYPE1 and PARITY_TYPE2 fields in LCR. */
    lcrRegValue &= ~((uint32_t) UART_LCR_PARITY_TYPE2_MASK |
                     (uint32_t) UART_LCR_PARITY_TYPE1_MASK |
                     (uint32_t) UART_LCR_PARITY_EN_MASK);

    /* Programming the PARITY_EN, PARITY_TYPE1 and PARITY_TYPE2 fields in LCR.*/
    lcrRegValue |= parityFlag & (UART_LCR_PARITY_TYPE2_MASK |
                                 UART_LCR_PARITY_TYPE1_MASK |
                                 UART_LCR_PARITY_EN_MASK);
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static void UART_divisorLatchDisable(uint32_t baseAddr)
{
    /* Disabling access to Divisor Latch registers by clearing LCR[7] bit. */
    HW_WR_FIELD32(baseAddr + UART_LCR, UART_LCR_DIV_EN,
                  UART_LCR_DIV_EN_DIV_EN_VALUE_0);
}

static void UART_breakCtl(uint32_t baseAddr, uint32_t breakState)
{
    /* Programming the BREAK_EN bit in LCR. */
    HW_WR_FIELD32(baseAddr + UART_LCR, UART_LCR_BREAK_EN,
                  breakState >> UART_LCR_BREAK_EN_SHIFT);
}

uint8_t UART_fifoCharGet(uint32_t baseAddr)
{
    uint32_t tempRetVal = 0U;
    tempRetVal = HW_RD_REG32(baseAddr + UART_RHR);
    return ((uint8_t) tempRetVal);
}

static void UART_hardwareFlowCtrlOptSet(uint32_t baseAddr, uint32_t hwFlowCtrl)
{
    uint32_t lcrRegValue = 0;

    /* Switching to Configuration Mode B of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_B);

    /* Configuring the HWFLOWCONTROL field in EFR. */
    HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_HW_FLOW_CONTROL, hwFlowCtrl);

    /* Restoring LCR with the collected value. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
}

static void UART_flowCtrlTrigLvlConfig(uint32_t baseAddr,
                               uint32_t rtsHaltFlag,
                               uint32_t rtsStartFlag)
{
    uint32_t tcrValue = 0;

    tcrValue = rtsHaltFlag & UART_TCR_RX_FIFO_TRIG_HALT_MASK;

    tcrValue |= (rtsStartFlag <<
                 UART_TCR_RX_FIFO_TRIG_START_SHIFT) &
                UART_TCR_RX_FIFO_TRIG_START_MASK;

    /* Writing to TCR register. */
    HW_WR_REG32(baseAddr + UART_TCR, tcrValue);
}

uint32_t UART_spaceAvail(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0;
    uint32_t retVal      = FALSE;

    /* Switching to Register Operational Mode of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_OPERATIONAL_MODE);

    /*
    ** Checking if either TXFIFOE or TXSRE bits of Line Status Register(LSR)
    ** are set. TXFIFOE bit is set if TX FIFO(or THR in non-FIFO mode) is
    ** empty. TXSRE is set if both the TX FIFO(or THR in non-FIFO mode) and
    ** the transmitter shift register are empty.
    */
    if ((UART_LSR_TX_SR_E_MASK | UART_LSR_TX_FIFO_E_MASK) ==
        (HW_RD_REG32(baseAddr + UART_LSR) &
            (UART_LSR_TX_SR_E_MASK | UART_LSR_TX_FIFO_E_MASK)))
    {
        retVal = (uint32_t) TRUE;
    }

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

uint32_t UART_IsTxRxFifoEmpty(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0;
    uint32_t retVal      = FALSE;

    /* Switching to Register Operational Mode of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_OPERATIONAL_MODE);

    /*
    ** Checking if either TXFIFOE and RXFIFOE bits of Line Status Register(LSR)
    ** are set/unset. TXFIFOE bit is set if TX FIFO(or THR in non-FIFO mode) is
    ** empty. RXFIFOE bit is unset if the RX FIFO(or RHR in non-FIFO mode) is
    ** empty.
    */

    if (((UART_LSR_TX_FIFO_E_TX_FIFO_E_VALUE_1 << UART_LSR_TX_FIFO_E_SHIFT) |
         (UART_LSR_RX_FIFO_E_RX_FIFO_E_VALUE_0 << UART_LSR_RX_FIFO_E_SHIFT)) ==
        (HW_RD_REG32(baseAddr + UART_LSR) &
            ((UART_LSR_TX_FIFO_E_TX_FIFO_E_VALUE_1 << UART_LSR_TX_FIFO_E_SHIFT) |
            (UART_LSR_RX_FIFO_E_RX_FIFO_E_VALUE_0 << UART_LSR_RX_FIFO_E_SHIFT))))
    {
        retVal = (uint32_t) TRUE;
    }

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

uint8_t UART_readByte(UARTLLD_Handle hUart)
{
    uint8_t           readByte = 0;
    volatile uint32_t waitCount = UART_ERROR_COUNT;
    uint32_t          errorVal;

    errorVal = UART_getRxError(hUart->baseAddr);
    /* Read and throw Erroneous bytes from RxFIFO */
    while ((UART_LSR_RX_FIFO_STS_MASK |
            UART_LSR_RX_BI_MASK |
            UART_LSR_RX_FE_MASK |
            UART_LSR_RX_PE_MASK |
            UART_LSR_RX_OE_MASK) == errorVal)
    {
        (void)UART_fifoCharGet(hUart->baseAddr);
        hUart->readErrorCnt++;
        waitCount = waitCount - 1U;

        errorVal = UART_getRxError(hUart->baseAddr);
        if (0U == waitCount)
        {
            break;
        }
    }
    /* Read non-erroneous byte from RxFIFO */
    readByte = UART_fifoCharGet(hUart->baseAddr);

    return readByte;
}

static uint32_t UART_getRxError(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0;
    uint32_t retVal      = 0;

    /* Switching to Register Operational Mode of operation. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_OPERATIONAL_MODE);

    retVal = HW_RD_REG32(baseAddr + UART_LSR) &
             (UART_LSR_RX_FIFO_STS_MASK |
              UART_LSR_RX_BI_MASK |
              UART_LSR_RX_FE_MASK |
              UART_LSR_RX_PE_MASK |
              UART_LSR_RX_OE_MASK);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

static uint32_t UART_regConfigModeEnable(uint32_t baseAddr, uint32_t modeFlag)
{
    uint32_t lcrRegValue;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    switch (modeFlag)
    {
        case UART_REG_CONFIG_MODE_A:
        case UART_REG_CONFIG_MODE_B:
            HW_WR_REG32(baseAddr + UART_LCR, modeFlag & 0xFFU);
            break;

        case UART_REG_OPERATIONAL_MODE:
            HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                        & 0x7FU);
            break;

        default:
            break;
    }

    return lcrRegValue;
}

int32_t UART_writeInterrupt(UARTLLD_Handle hUart)
{
    int32_t     status = UART_STATUS_SUCCESS;
    uint32_t    baseAddr;

    baseAddr = hUart->baseAddr;

    /* Enable the transmit interrupt. */
    UART_intrEnable(baseAddr, UART_INTR_THR);

    return status;
}

int32_t UART_writePolling(UARTLLD_Handle hUart, UART_Transaction *trans)
{
    uint32_t            startTicks, elapsedTicks;
    int32_t             retVal          = UART_TRANSFER_STATUS_SUCCESS;
    uint32_t            timeoutElapsed  = FALSE;
    uint32_t            baseAddr        = hUart->baseAddr;
    uint32_t            lineStatus      = 0U;

    UARTLLD_InitHandle        hUartInit;
    hUartInit = hUart->hUartInit;

    hUart->writeSizeRemaining = trans->count;
    /* Update current tick value to perform timeout operation */
    startTicks = hUartInit->clockP_get();
    while ((FALSE == timeoutElapsed)
           && (0U != hUart->writeSizeRemaining))
    {
        /* Transfer DATA */
        UART_writeDataPolling(hUart);
        /* Check whether timeout happened or not */
        elapsedTicks = hUartInit->clockP_get() - startTicks;
        if (elapsedTicks >= trans->timeout)
        {
            /* timeout occured */
            timeoutElapsed = TRUE;
        }
    }

    if (0U == hUart->writeSizeRemaining)
    {
        do
        {
            lineStatus = UART_readLineStatus(baseAddr);
            elapsedTicks = hUartInit->clockP_get() - startTicks;
        }
        while (((uint32_t) (UART_LSR_TX_FIFO_E_MASK |
                         UART_LSR_TX_SR_E_MASK) !=
               (lineStatus & (uint32_t) (UART_LSR_TX_FIFO_E_MASK |
                                       UART_LSR_TX_SR_E_MASK)))
                && (elapsedTicks < hUart->lineStatusTimeout));

        if(elapsedTicks >= hUart->lineStatusTimeout)
        {
            retVal             = UART_TRANSFER_TIMEOUT;
            trans->status      = UART_TRANSFER_STATUS_TIMEOUT;
            trans->count       = hUart->writeCount;
            UART_lld_Transaction_deInit(&hUart->writeTrans);
        }
        else
        {
            retVal             = UART_TRANSFER_STATUS_SUCCESS;
            trans->status      = UART_TRANSFER_STATUS_SUCCESS;
            UART_lld_Transaction_deInit(&hUart->writeTrans);
        }
    }
    else
    {
        /* Return UART_TRANSFER_TIMEOUT so that application gets whatever bytes are
         * transmitted. Set the trans status to timeout so that
         * application can handle the timeout. */
        retVal             = UART_TRANSFER_TIMEOUT;
        trans->status      = UART_TRANSFER_STATUS_TIMEOUT;
        trans->count       = hUart->writeCount;
        UART_lld_Transaction_deInit(&hUart->writeTrans);
    }

    return (retVal);
}

static void UART_writeDataPolling(UARTLLD_Handle hUart)
{
    uint32_t numBytesWritten = 0U;

    numBytesWritten = UART_fifoWrite(hUart,
                                     (const uint8_t *) hUart->writeBuf,
                                     hUart->writeSizeRemaining);

    hUart->writeSizeRemaining -= numBytesWritten;
    hUart->writeBuf           = (const void *)((uint8_t *)hUart->writeBuf + numBytesWritten);
    hUart->writeCount         += numBytesWritten;

    return;
}

static uint32_t UART_fifoWrite(UARTLLD_Handle hUart,
                               const uint8_t    *buffer,
                               uint32_t          writeSizeRemaining)
{
    uint32_t tempWriteSizeRemaining = writeSizeRemaining;
    uint32_t size                  = tempWriteSizeRemaining;
    const uint8_t *tempBuffer = buffer;
    uint32_t lineStatus            = 0U;
    uint32_t tempChunksize         = 0U;
    int32_t  maxTrialCount         = (int32_t) UART_TRANSMITEMPTY_TRIALCOUNT;

    /* Load the fifo size  */
    tempChunksize = UART_FIFO_SIZE;

    /* Before we could write no of bytes, we should have
     * no of free buffers. Hence, we check for shiftregister
     * empty (ensure the FIFO is empty) to write num of bytes */
    do
    {
        lineStatus = (uint32_t) UART_readLineStatus(hUart->baseAddr);
        maxTrialCount--;
    }
    while (((uint32_t) (UART_LSR_TX_SR_E_MASK | UART_LSR_TX_FIFO_E_MASK) !=
            ((uint32_t) (UART_LSR_TX_SR_E_MASK |
                       UART_LSR_TX_FIFO_E_MASK) & lineStatus))
           && (0 < maxTrialCount));

    if (maxTrialCount > 0)
    {
        while ((tempChunksize > 0U) && (tempWriteSizeRemaining > 0U))
        {
            /* Writing to the H/w */
            UART_putChar(hUart->baseAddr, (*tempBuffer));
            tempBuffer++;
            tempWriteSizeRemaining--;
            tempChunksize--;
        }
    }

    /* Returns the size actually written */
    return (size - tempWriteSizeRemaining);
}

/* ========================================================================== */
/*                       Advanced Function Definitions                        */
/* ========================================================================== */
uint32_t UART_getBaseAddr_lld(UARTLLD_Handle hUart)
{
    uint32_t           baseAddr;

    /* Check parameters */
    if (NULL_PTR == hUart)
    {
        baseAddr = 0U;
    }
    else
    {
        baseAddr = hUart->baseAddr;
    }

    return baseAddr;
}

void UART_enableLoopbackMode(uint32_t baseAddr)
{
    uint32_t lcrRegValue;

    /* Switching to Register Configuration Mode A. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Enable Loopback Mode. */
    HW_WR_FIELD32(baseAddr + UART_MCR, UART_MCR_LOOPBACK_EN,
                  UART_MCR_LOOPBACK_EN_LOOPBACK_EN_VALUE_1);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return;
}

void UART_disableLoopbackMode(uint32_t baseAddr)
{
    uint32_t lcrRegValue;

    /* Switching to Register Configuration Mode A. */
    lcrRegValue = UART_regConfigModeEnable(baseAddr, UART_REG_CONFIG_MODE_A);

    /* Enable Loopback Mode. */
    HW_WR_FIELD32(baseAddr + UART_MCR, UART_MCR_LOOPBACK_EN,
                  UART_MCR_LOOPBACK_EN_LOOPBACK_EN_VALUE_0);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return;
}

int32_t UART_lld_init(UARTLLD_Handle hUart)
{
    int32_t                status = UART_STATUS_SUCCESS;
    UARTLLD_InitHandle        hUartInit;

    if((hUart != NULL_PTR) && (hUart->hUartInit != NULL_PTR))
    {
        if(hUart->state != UART_STATE_RESET)
        {
            status = UART_TRANSFER_INVALID_STATE;
        }
    }
    else
    {
        status = UART_INVALID_PARAM;
    }

    if(UART_STATUS_SUCCESS == status)
    {
        hUart->state = UART_STATE_BUSY;
        hUartInit = hUart->hUartInit;
        hUart->lineStatusTimeout = hUartInit->clockP_usecToTick(UART_READ_LINE_STATUS_TIMEOUT_IN_US);

        /* Check the UART input parameters */
        status += UART_IsBaseAddrValid(hUart->baseAddr);
        status += UART_IsParameter(hUartInit->inputClkFreq);
        status += UART_IsParameter(hUartInit->baudRate);
        status += UART_IsDataLengthValid(hUartInit->dataLength);
        status += UART_IsStopBitsValid(hUartInit->stopBits);
        status += UART_IsParityTypeValid(hUartInit->parityType);
        status += UART_IsHWFlowCtrlValid(hUartInit->hwFlowControlThr);
        status += UART_OperModeValid(hUartInit->operMode);
        status += UART_IsRxTrigLvlValid(hUartInit->rxTrigLvl);
        status += UART_IsTxTrigLvlValid(hUartInit->txTrigLvl);

        if(UART_STATUS_SUCCESS == status)
        {
            /* Configure the UART instance parameters */
            UART_configInstance(hUart);
            hUart->state = UART_STATE_READY;
        }
        else
        {
            /* Free-up resources in case of error */
            status += UART_lld_deInit(hUart);
        }
    }

    return status;
}

int32_t UART_lld_initDma(UARTLLD_Handle hUart)
{
    int32_t                status = UART_STATUS_SUCCESS;
    UARTLLD_InitHandle        hUartInit;

    if((hUart != NULL_PTR) && (hUart->hUartInit != NULL_PTR))
    {
        if(hUart->state != UART_STATE_RESET)
        {
            status = UART_TRANSFER_INVALID_STATE;
        }
    }
    else
    {
        status = UART_INVALID_PARAM;
    }

    if(UART_STATUS_SUCCESS == status)
    {
        hUart->state = UART_STATE_BUSY;
        hUartInit = hUart->hUartInit;
        hUart->lineStatusTimeout = hUartInit->clockP_usecToTick(UART_READ_LINE_STATUS_TIMEOUT_IN_US);

        /* Check the UART input parameters */
        status += UART_IsBaseAddrValid(hUart->baseAddr);
        status += UART_IsParameter(hUartInit->inputClkFreq);
        status += UART_IsParameter(hUartInit->baudRate);
        status += UART_IsDataLengthValid(hUartInit->dataLength);
        status += UART_IsStopBitsValid(hUartInit->stopBits);
        status += UART_IsParityTypeValid(hUartInit->parityType);
        status += UART_IsHWFlowCtrlValid(hUartInit->hwFlowControlThr);
        status += UART_OperModeValid(hUartInit->operMode);
        status += UART_IsRxTrigLvlValid(hUartInit->rxTrigLvl);
        status += UART_IsTxTrigLvlValid(hUartInit->txTrigLvl);

        /* NULL check for Dma handle */
        if((hUartInit->uartDmaHandle == NULL_PTR)
            || (hUartInit->dmaChCfg == NULL_PTR))
        {
            status += UART_INVALID_PARAM;
        }
    }

    if(UART_STATUS_SUCCESS == status)
    {
        /* Configure the UART instance parameters */
        UART_configInstance(hUart);

        status = UART_lld_dmaInit(hUart, hUart->hUartInit->dmaChCfg);
    }

    if(UART_STATUS_SUCCESS == status)
    {
        hUart->state = UART_STATE_READY;
    }
    else
    {
        /* Free-up resources in case of error */
       status = UART_lld_deInitDma(hUart);
    }

    return status;
}

int32_t UART_lld_deInit(UARTLLD_Handle hUart)
{
    int32_t             status = UART_STATUS_SUCCESS;

    if (NULL_PTR != hUart)
    {
        hUart->state = UART_STATE_BUSY;

        /* Flush TX FIFO */
       status = UART_lld_flushTxFifo(hUart);

        if(status == UART_STATUS_SUCCESS)
        {
            /* Disable UART and interrupts. */
            UART_intrDisable(hUart->baseAddr,
                        UART_INTR_RHR_CTI | UART_INTR_THR | UART_INTR_LINE_STAT);
            UART_intr2Disable(hUart->baseAddr, UART_INT2_TX_EMPTY);
            (void)UART_operatingModeSelect(hUart->baseAddr, UART_OPER_MODE_DISABLED);
        }
        hUart->state = UART_STATE_RESET;
    }
    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

int32_t UART_lld_deInitDma(UARTLLD_Handle hUart)
{
    int32_t             status = UART_STATUS_SUCCESS;

    if (NULL_PTR != hUart)
    {
        hUart->state = UART_STATE_BUSY;

        /* Flush TX FIFO */
       status = UART_lld_flushTxFifo(hUart);

        if(status == UART_STATUS_SUCCESS)
        {
            /* Disable UART and interrupts. */
            UART_intrDisable(hUart->baseAddr,
                        UART_INTR_RHR_CTI | UART_INTR_THR | UART_INTR_LINE_STAT);
            UART_intr2Disable(hUart->baseAddr, UART_INT2_TX_EMPTY);
            (void)UART_operatingModeSelect(hUart->baseAddr, UART_OPER_MODE_DISABLED);
            hUart->state = UART_STATE_RESET;
        }
        status = UART_lld_dmaDeInit(hUart);
    }
    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

int32_t UART_writeCancelNoCB(UARTLLD_Handle hUart)
{
    uintptr_t           key;
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UARTLLD_InitHandle         hUartInit;
    hUartInit = hUart->hUartInit;

    UART_intrDisable(hUart->baseAddr, UART_INTR_THR);

    /* Disable interrupts to avoid writing data while changing state. */
    key = HwiP_disable();

    /* Return if there is no write. */
    if ((hUart->writeSizeRemaining) == 0U)
    {
        status = UART_TRANSFER_STATUS_FAILURE;
    }
    else
    {
        if (hUartInit->transferMode == UART_CONFIG_MODE_DMA)
        {
            /* Disable DMA TX channel */
            UART_lld_dmaDisableChannel(hUart, (Bool)TRUE);
            if (hUart->writeTrans.buf != NULL)
            {
                hUart->writeTrans.count = 0;
            }
            else
            {
                hUart->writeCount = 0;
            }
        }
        else
        {
            /* Reset the write buffer so we can pass it back */
            hUart->writeBuf = (const uint8_t *)hUart->writeBuf - hUart->writeCount;
            if (hUart->writeTrans.buf != NULL)
            {
                hUart->writeTrans.count = (uint32_t)(hUart->writeCount);
            }

            /* Set size = 0 to prevent writing and restore interrupts. */
            hUart->writeSizeRemaining = 0;
        }
    }

    HwiP_restore(key);

    return (status);
}

int32_t UART_readCancelNoCB(UARTLLD_Handle hUart)
{
    uintptr_t           key;
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UARTLLD_InitHandle         hUartInit;
    hUartInit = hUart->hUartInit;

    UART_intrDisable(hUart->baseAddr,
                   UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);

    /* Disable interrupts to avoid reading data while changing state. */
    key = HwiP_disable();
    if (hUart->readSizeRemaining == 0U)
    {
        status = UART_TRANSFER_STATUS_FAILURE;
    }
    else
    {
        if (hUartInit->transferMode == UART_CONFIG_MODE_DMA)
        {
            /* Disable DMA TX channel */
            UART_lld_dmaDisableChannel(hUart, (Bool)FALSE);
            if (hUart->readTrans.buf != NULL)
            {
                hUart->readTrans.count = 0;
            }
            else
            {
                hUart->readCount = 0;
            }
        }
        else
        {
            /* Reset the read buffer so we can pass it back */
            hUart->readBuf = (uint8_t *)hUart->readBuf - hUart->readCount;
            if (hUart->readTrans.buf != NULL)
            {
                hUart->readTrans.count = hUart->readCount;
            }

            /* Set size = 0 to prevent reading and restore interrupts. */
            hUart->readSizeRemaining = 0;

        }
    }

    HwiP_restore(key);
    return (status);
}

int32_t UART_lld_write(UARTLLD_Handle hUart, void * txBuf, uint32_t size, uint32_t timeout,
                       const UART_ExtendedParams *extendedParams)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_Transaction    *trans;

    if(NULL_PTR != hUart)
    {
         trans = &hUart->writeTrans;
        /* Check if any transaction is in progress */
        if(NULL_PTR != trans->buf)
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_INUSE;
            status = UART_TRANSFER_STATUS_FAILURE;
        }
        else
        {
            /* Initialize transaction params */
            UART_lld_Transaction_init(trans);

            if(extendedParams != NULL)
            {
                trans->args   = extendedParams->args;
            }
            else
            {
                trans->args = NULL;
            }
            trans->buf = (void *) txBuf;
            trans->count = size;
            trans->timeout = timeout;

            if(hUart->state == UART_STATE_READY)
            {
                status = UART_checkTransaction(trans);
            }
            else
            {
                status = UART_TRANSFER_BUSY;
            }

            if(UART_TRANSFER_STATUS_SUCCESS == status)
            {

                /* Initialize transaction params */
                hUart->writeBuf                = trans->buf;
                hUart->writeTrans.timeout      = trans->timeout;
                hUart->writeCount              = 0U;
                hUart->writeSizeRemaining      = trans->count;
            }

            hUart->state = UART_STATE_BUSY;
            /* Polled mode */
            status = UART_writePolling(hUart, trans);
            hUart->state = UART_STATE_READY;

        }

    }
    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

int32_t UART_lld_writeIntr(UARTLLD_Handle hUart, void * txBuf, uint32_t size,
                           const UART_ExtendedParams *extendedParams)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_Transaction    *trans;

    if(NULL_PTR != hUart)
    {
        trans = &hUart->writeTrans;
        /* Check if any transaction is in progress */
        if(NULL_PTR != trans->buf)
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_INUSE;
            status = UART_TRANSFER_STATUS_FAILURE;
        }
        else
        {
            /* Initialize the input parameters */
            UART_lld_Transaction_init(trans);
            if(extendedParams != NULL)
            {
                trans->args   = extendedParams->args;
            }
            else
            {
                trans->args = NULL;
            }

            trans->buf = (void *) txBuf;
            trans->count = size;

            /* Check parameters */
            if(hUart->state == UART_STATE_READY)
            {
                status = UART_checkTransaction(trans);
            }
            else
            {
                status = UART_TRANSFER_BUSY;
            }

            if(UART_TRANSFER_STATUS_SUCCESS == status)
            {
               /* Initialize transaction params */
                hUart->writeBuf                = trans->buf;
                hUart->writeTrans.timeout      = trans->timeout;
                hUart->writeCount              = 0U;
                hUart->writeSizeRemaining      = trans->count;
                hUart->state = UART_STATE_BUSY;

                /* Interrupt mode */
                status = UART_writeInterrupt(hUart);
                hUart->state = UART_STATE_READY;
            }
        }

    }
    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

int32_t UART_lld_writeDma(UARTLLD_Handle hUart, void * txBuf, uint32_t size,
                         const UART_ExtendedParams *extendedParams)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_Transaction    *trans;

    if(NULL_PTR != hUart)
    {
        trans = &hUart->writeTrans;
        /* Check if any transaction is in progress */
        if(NULL_PTR != trans->buf)
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_INUSE;
            status = UART_TRANSFER_STATUS_FAILURE;
        }
        else
        {
            /* Initialize the input parameters */
            UART_lld_Transaction_init(trans);

            if(extendedParams != NULL)
            {
                trans->args   = extendedParams->args;
            }
            else
            {
                trans->args = NULL;
            }

            trans->buf = (void *) txBuf;
            trans->count = size;

            if(hUart->state == UART_STATE_READY)
            {
                status = UART_checkTransaction(trans);
            }
            else
            {
                status = UART_TRANSFER_BUSY;
            }

            if(UART_TRANSFER_STATUS_SUCCESS == status)
            {

                /* Initialize transaction params */
                hUart->writeBuf                = trans->buf;
                hUart->writeTrans.timeout      = trans->timeout;
                hUart->writeCount              = 0U;
                hUart->writeSizeRemaining      = trans->count;
                hUart->state = UART_STATE_BUSY;

                /* DMA mode */
                status = UART_lld_dmaWrite(hUart, trans);
                hUart->state = UART_STATE_READY;
            }
        }

    }

    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

int32_t UART_lld_read(UARTLLD_Handle hUart, void * rxBuf, uint32_t size, uint32_t timeout,
                     const UART_ExtendedParams *extendedParams)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_Transaction    *trans;

    if(NULL_PTR != hUart)
    {
        trans = &hUart->readTrans;
        /* Check if any transaction is in progress */
        if(NULL_PTR != trans->buf)
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_INUSE;
            status = UART_TRANSFER_STATUS_FAILURE;
        }
        else
        {
             /* Initialize the input parameters */
            UART_lld_Transaction_init(trans);
            if(extendedParams != NULL)
            {
                trans->args   = extendedParams->args;
            }
            else
            {
                trans->args = NULL;
            }

            trans->buf = (void *) rxBuf;
            trans->count = size;
            trans->timeout = timeout;

            if(hUart->state == UART_STATE_READY)
            {
                status = UART_checkTransaction(trans);
            }
            else
            {
                status = UART_TRANSFER_BUSY;
            }

            if(UART_TRANSFER_STATUS_SUCCESS == status)
            {
                /* Initialize transaction params */
                hUart->readBuf             = trans->buf;
                hUart->readTrans.timeout   = trans->timeout;
                hUart->readCount           = 0U;
                hUart->rxTimeoutCnt        = 0U;
                hUart->readErrorCnt        = 0U;
                hUart->state = UART_STATE_BUSY;

                /* Polled mode */
                status = UART_readPolling(hUart, trans);
                hUart->state = UART_STATE_READY;
            }
        }
    }

    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

int32_t UART_lld_readWithCounter(UARTLLD_Handle hUart, void * rxBuf, uint32_t size, uint32_t timeout,
                     const UART_ExtendedParams *extendedParams)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_Transaction    *trans;

    if(NULL_PTR != hUart)
    {
        trans = &hUart->readTrans;
        /* Check if any transaction is in progress */
        if(NULL_PTR != trans->buf)
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_INUSE;
            status = UART_TRANSFER_STATUS_FAILURE;
        }
        else
        {
             /* Initialize the input parameters */
            UART_lld_Transaction_init(trans);
            if(extendedParams != NULL)
            {
                trans->args   = extendedParams->args;
            }
            else
            {
                trans->args = NULL;
            }

            trans->buf = (void *) rxBuf;
            trans->count = size;
            trans->timeout = timeout;

            if(hUart->state == UART_STATE_READY)
            {
                status = UART_checkTransaction(trans);
            }
            else
            {
                status = UART_TRANSFER_BUSY;
            }

            if(UART_TRANSFER_STATUS_SUCCESS == status)
            {
                /* Initialize transaction params */
                hUart->readBuf             = trans->buf;
                hUart->readTrans.timeout   = trans->timeout;
                hUart->readCount           = 0U;
                hUart->rxTimeoutCnt        = 0U;
                hUart->readErrorCnt        = 0U;
                hUart->state = UART_STATE_BUSY;

                /* Polled mode */
                status = UART_readPollingWithCounter(hUart, trans);
                hUart->state = UART_STATE_READY;
            }
        }
    }

    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

int32_t UART_lld_readIntr(UARTLLD_Handle hUart, void * rxBuf, uint32_t size,
                          const UART_ExtendedParams *extendedParams)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_Transaction    *trans;

    if(NULL_PTR != hUart)
    {
        trans = &hUart->readTrans;
         /* Check if any transaction is in progress */
        if(NULL_PTR != trans->buf)
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_INUSE;
            status = UART_TRANSFER_STATUS_FAILURE;
        }
        else
        {
            /* Initialize the input parameters */
            UART_lld_Transaction_init(trans);
            if(extendedParams != NULL)
            {
                trans->args   = extendedParams->args;
            }
            else
            {
                trans->args = NULL;
            }

            trans->buf = (void *) rxBuf;
            trans->count = size;

            if(hUart->state == UART_STATE_READY)
            {
                status = UART_checkTransaction(trans);
            }
            else
            {
                status = UART_TRANSFER_BUSY;
            }

            if(UART_TRANSFER_STATUS_SUCCESS == status)
            {
                /* Initialize transaction params */
                hUart->readBuf             = trans->buf;
                hUart->readTrans.timeout   = trans->timeout;
                hUart->readCount           = 0U;
                hUart->rxTimeoutCnt        = 0U;
                hUart->readErrorCnt        = 0U;
                hUart->state = UART_STATE_BUSY;

                /* Interrupt mode */
                status = UART_readInterrupt(hUart, trans);
                hUart->state = UART_STATE_READY;
            }
        }
    }
    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

int32_t UART_lld_readDma(UARTLLD_Handle hUart, void * rxBuf, uint32_t size,
                        const UART_ExtendedParams *extendedParams)
{
    int32_t status = UART_TRANSFER_STATUS_SUCCESS;
    UART_Transaction    *trans;

    if(NULL_PTR != hUart)
    {
        trans = &hUart->readTrans;
        /* Check if any transaction is in progress */
        if(NULL_PTR != trans->buf)
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_INUSE;
            status = UART_TRANSFER_STATUS_FAILURE;
        }
        else
        {
            /* Initialize the input parameters */
            UART_lld_Transaction_init(trans);
            if(extendedParams != NULL)
            {
                trans->args   = extendedParams->args;
            }
            else
            {
                trans->args   = NULL;
            }

            trans->buf = (void *) rxBuf;
            trans->count = size;

            if(hUart->state == UART_STATE_READY)
            {
                status = UART_checkTransaction(trans);
            }
            else
            {
                status = UART_TRANSFER_BUSY;
            }

            if(UART_TRANSFER_STATUS_SUCCESS == status)
            {
                /* Initialize transaction params */
                hUart->readBuf             = trans->buf;
                hUart->readTrans.timeout   = trans->timeout;
                hUart->readCount           = 0U;
                hUart->rxTimeoutCnt        = 0U;
                hUart->readErrorCnt        = 0U;
                hUart->state = UART_STATE_BUSY;

                /* DMA mode */
                status = UART_lld_dmaRead(hUart, trans);
                hUart->state = UART_STATE_READY;
            }

        }

    }
    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

int32_t UART_lld_writeCancel(UARTLLD_Handle hUart, UART_Transaction *trans)
{
    int32_t             status = UART_STATUS_SUCCESS;

    /* Check parameters */
    if((NULL_PTR == hUart) || (NULL == trans))
    {
        status = UART_INVALID_PARAM;
    }

    if(UART_STATUS_SUCCESS == status)
    {
        if(hUart->state == UART_STATE_READY)
        {
            status = UART_checkTransaction(trans);
        }
        else
        {
            status = UART_TRANSFER_BUSY;
        }
    }

    if(UART_STATUS_SUCCESS == status)
    {
        if (UART_writeCancelNoCB(hUart) == UART_STATUS_SUCCESS)
        {
            hUart->writeTrans.status = UART_TRANSFER_STATUS_CANCELLED;
            hUart->state = UART_STATE_READY;
            hUart->hUartInit->writeCompleteCallbackFxn(hUart);
            UART_lld_Transaction_deInit(&hUart->writeTrans);
        }
        else
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_OTH;
            status = UART_TRANSFER_STATUS_FAILURE;
        }
    }

    return (status);
}

int32_t UART_lld_readCancel(UARTLLD_Handle hUart, UART_Transaction *trans)
{
    int32_t             status = UART_STATUS_SUCCESS;

    /* Check parameters */
    if((NULL_PTR == hUart) || (NULL == trans))
    {
        status = UART_INVALID_PARAM;
    }

    if(UART_STATUS_SUCCESS == status)
    {
        if (UART_readCancelNoCB(hUart) == UART_STATUS_SUCCESS)
        {
            hUart->readTrans.status = UART_TRANSFER_STATUS_CANCELLED;
            hUart->state = UART_STATE_READY;
            hUart->hUartInit->readCompleteCallbackFxn(hUart);
            UART_lld_Transaction_deInit(&hUart->readTrans);
        }
        else
        {
            trans->status = UART_TRANSFER_STATUS_ERROR_OTH;
            status = UART_TRANSFER_STATUS_FAILURE;
        }
    }

    return (status);
}

int32_t UART_lld_setRxState(UARTLLD_Handle hUart, uint32_t state)
{
    int32_t            status = UART_STATUS_SUCCESS;
    uint32_t           intrFlag = 0U;
    uint32_t           currIntMask;
    uint32_t           baseAddr;
    uint32_t           enhanFnBitVal = 0U;
    uint32_t           lcrRegValue   = 0U;

    if(hUart != NULL_PTR)
    {
        baseAddr = hUart->baseAddr;

        intrFlag = UART_INTR_RHR_CTI | UART_INTR_THR | UART_INTR_LINE_STAT;

        if ( state == UART_STATE_RX_DISABLED )
        {
            /* Preserving the current value of LCR. */
            lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
            /* Switching to Register Configuration Mode B. */
            HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

            /* Collecting the current value of EFR[4] and later setting it. */
            enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

            HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                        UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

            /* Restoring the value of LCR. */
            HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

            /* Preserving the current value of LCR. */
            lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

            /* Switching to Register Operational Mode of operation. */
            HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                                    & 0x7FU);

            currIntMask = HW_RD_REG32(baseAddr + UART_IER);
            HW_WR_REG32(baseAddr + UART_IER, currIntMask & ~(intrFlag & 0xFFU));
            hUart->currIntMask = currIntMask;

            /* Restoring the value of LCR. */
            HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

            /* Switch to mode B only when the upper 4 bits of IER needs to be changed */
            /* Preserving the current value of LCR. */
            lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
            /* Switching to Register Configuration Mode B. */
            HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

            /* Restoring the value of EFR[4] to its original value. */
            HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

            /* Restoring the value of LCR. */
            HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
        }
        else
        {
            intrFlag = hUart->currIntMask;

            /* Preserving the current value of LCR. */
            lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
            /* Switching to Register Configuration Mode B. */
            HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

            /* Collecting the current value of EFR[4] and later setting it. */
            enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

            HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                        UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

            /* Restoring the value of LCR. */
            HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

            /* Preserving the current value of LCR. */
            lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

            /* Switching to Register Operational Mode of operation. */
            HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                                    & 0x7FU);

            /*
            ** It is suggested that the System Interrupts for UART in the
            ** Interrupt Controller are enabled after enabling the peripheral
            ** interrupts of the UART using this API. If done otherwise, there
            ** is a risk of LCR value not getting restored and illicit characters
            ** transmitted or received from/to the UART. The situation is explained
            ** below.
            ** The scene is that the system interrupt for UART is already enabled
            ** and the current API is invoked. On enabling the interrupts
            ** corresponding to IER[7:4] bits below, if any of those interrupt
            ** conditions already existed, there is a possibility that the control
            ** goes to Interrupt Service Routine (ISR) without executing the
            ** remaining statements in this API. Executing the remaining statements
            ** is critical in that the LCR value is restored in them.
            ** However, there seems to be no risk in this API for enabling
            ** interrupts corresponding to IER[3:0] because it is done at the end
            ** and no statements follow that.
            */

            /************* ATOMIC STATEMENTS START *************************/

            /* Programming the bits IER[7:4]. */
            HW_WR_REG32(baseAddr + UART_IER, intrFlag & 0xF0U);

            /* Restoring the value of LCR. */
            HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

            /* Preserving the current value of LCR. */
            lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
            /* Switching to Register Configuration Mode B. */
            HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

            /* Restoring the value of EFR[4] to its original value. */
            HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

            /* Restoring the value of LCR. */
            HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

            /************** ATOMIC STATEMENTS END *************************/
            /* Programming the bits IER[3:0]. */
            HW_WR_REG32(baseAddr + UART_IER, HW_RD_REG32(baseAddr + UART_IER) |
                        (intrFlag & 0x0FU));
        }
    }

    return status;
}

int32_t UART_lld_flushTxFifo(UARTLLD_Handle hUart)
{
    int32_t             status = UART_STATUS_SUCCESS;
    uint32_t            isTxFifoEmpty, startTicks, elapsedTicks;
    uint32_t            timeout = UART_TRANSMITEMPTY_TRIALCOUNT;
    uint32_t            timeoutElapsed  = FALSE;
    UARTLLD_InitHandle        hUartInit;

    if (NULL_PTR != hUart)
    {
        hUartInit = hUart->hUartInit;
        /* Update current tick value to perform timeout operation */
        startTicks = hUartInit->clockP_get();
        while (FALSE == timeoutElapsed)
        {
            /* Get TX FIFO status */
            isTxFifoEmpty = UART_spaceAvail(hUart->baseAddr);
            if ((uint32_t) TRUE == isTxFifoEmpty)
            {
                /* FIFO and Shift register is empty */
                break;
            }

            /* Check whether timeout happened or not */
            elapsedTicks = hUartInit->clockP_get() - startTicks;
            if (elapsedTicks >= timeout)
            {
                /* timeout occured */
                timeoutElapsed = TRUE;
            }
            else
            {
                TaskP_yield();
            }
        }

        if(TRUE == timeoutElapsed)
        {
            status = UART_TRANSFER_TIMEOUT;
        }
    }
    else
    {
        status = UART_INVALID_PARAM;
    }

    return status;
}

/*
 *  ======== UART_controllerIsr ========
 *  Hwi function that processes UART interrupts.
 *
 *  In non-DMA mode, three UART interrupts are enabled:
 *    1. transmit FIFO is below the TX FIFO trigger level (THR)
 *    2. receive FIFO is above RX FIFO trigger level (RHR)
 *    3. line status rx error
 *
 *  ISR checks the three interrupts, to ensure that all
 *  the pending interrupts are handled.
 *
 *  If line status rx error is detected, ISR clears the last read error, update
 *  the actual number of bytes transferred and transfer status in readTrans and
 *  calls back to application in the callback mode or post the read semaphone
 *  in the blocking mode. ISR
 *
 *  If RHR interrupt is received, ISR calls the in-lined function readData to
 *  read the data out from the RX FIFO. When all the data are received, ISR updates
 *  the actual number of bytes transferred and transfer status in readTrans and
 *  calls back to the application in the callback mode or post the read semaphone
 *  in the blocking mode. if RX timeout is detected, ISR will log the timeout
 *  count.
 *
 *  If THR interrupt is received, ISR calls the in-lined function writeData,
 *  to write the data to the TX FIFO. After all the data are sent, TX FIFO empty
 *  interrupt is enabled, ISR will update the actual number of bytes transferred
 *  and transfer status in writeTrans and calls back to application in the
 *  callback mode or post the read semaphone in the blocking mode.
 *
 *
 *  @param(arg)         The UARTLLD_Handle for this Hwi.
 */
void UART_lld_controllerIsr(void* args)
{
    uint32_t            intType;
    uint8_t             rdData;
    UARTLLD_Handle         hUart;
    uint32_t            retVal = TRUE;

    if(NULL != args)
    {
        hUart = (UARTLLD_Handle)args;

        while (retVal == TRUE)
        {
            intType = UART_getIntrIdentityStatus(hUart->baseAddr);

            if ((intType & UART_INTID_RX_THRES_REACH) == UART_INTID_RX_THRES_REACH)
            {
                if ((intType & UART_INTID_RX_LINE_STAT_ERROR) ==
                    UART_INTID_RX_LINE_STAT_ERROR)
                {
                    /* RX line status error */
                    (void)UART_procLineStatusErr(hUart);
                }
                else
                {
                    if ((intType & UART_INTID_CHAR_TIMEOUT) == UART_INTID_CHAR_TIMEOUT)
                    {
                        /* Disable Interrupt first, to avoid further RX timeout */
                        UART_intrDisable(hUart->baseAddr, UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);
                        /* RX timeout, log the RX timeout errors */
                        hUart->rxTimeoutCnt++;
                    }
                    /* RX FIFO threshold reached */
                    if (hUart->readSizeRemaining > 0U)
                    {
                        hUart->readSizeRemaining = UART_readData(hUart, hUart->readSizeRemaining);
                        if ((hUart->readSizeRemaining == 0U) ||
                            (hUart->hUartInit->readReturnMode == UART_READ_RETURN_MODE_PARTIAL))
                        {
                            /* transfer completed */
                            UART_intrDisable(hUart->baseAddr, UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);

                            /* Update the driver internal status. */
                            /* Reset the read buffer so we can pass it back */
                            hUart->readBuf = (uint8_t *)hUart->readBuf - hUart->readCount;
                            if (hUart->readTrans.buf != NULL)
                            {
                                hUart->readTrans.count = (uint32_t)(hUart->readCount);
                                hUart->readTrans.status = UART_TRANSFER_STATUS_SUCCESS;
                            }
                            /*
                            * Post transfer Sem in case of bloacking transfer.
                            * Call the callback function in case of Callback mode.
                            */
                            hUart->hUartInit->readCompleteCallbackFxn(hUart);
                            UART_lld_Transaction_deInit(&hUart->readTrans);
                        }
                        else
                        {
                            /* Enable Rx interrupt again in case of UART_READ_RETURN_MODE_FULL */
                            UART_intrEnable(hUart->baseAddr, (uint32_t) UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);
                        }
                    }
                    else
                    {
                        /* Disable the receive interrupt again. */
                        (void)UART_getChar(hUart->baseAddr, &rdData);
                        UART_intrDisable(hUart->baseAddr, (uint32_t) UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);
                    }
                }
            }
            else if ((intType & UART_INTID_TX_THRES_REACH) == UART_INTID_TX_THRES_REACH)
            {
                /* TX FIFO threshold reached */
                if (hUart->writeSizeRemaining > 0U)
                {
                    hUart->writeSizeRemaining = (uint32_t)UART_writeData(hUart, (hUart->writeSizeRemaining));
                    if ((hUart->writeSizeRemaining) == 0U)
                    {
                        UART_intrDisable(hUart->baseAddr, UART_INTR_THR);

                        /* Reset the write buffer so we can pass it back */
                        hUart->writeBuf = (const void *)((uint8_t *)hUart->writeBuf - hUart->writeCount);
                        if (hUart->writeTrans.buf != NULL)
                        {
                            hUart->writeTrans.count = (uint32_t)(hUart->writeCount);
                            hUart->writeTrans.status = UART_TRANSFER_STATUS_SUCCESS;
                        }
                        /*
                        * Post transfer Sem in case of bloacking transfer.
                        * Call the callback function in case of Callback mode.
                        */
                        hUart->hUartInit->writeCompleteCallbackFxn(hUart);
                        UART_lld_Transaction_deInit(&hUart->writeTrans);
                    }
                }
                else
                {
                    UART_intrDisable(hUart->baseAddr, UART_INTR_THR);
                }
            }
            else if ((intType & UART_INTID_CHAR_TIMEOUT) == UART_INTID_CHAR_TIMEOUT)
            {
                /* Work around for errata i2310 */
                if (FALSE == UART_checkCharsAvailInFifo(hUart->baseAddr))
                {
                    UART_i2310WA(hUart->baseAddr);
                }
            }
            else
            {
                retVal = FALSE;
            }
        } /* while(TRUE) */
    }
    else
    {
        UART_NOT_IN_USE(args);
    }
}

/* Work around for errata i2310
 *
 * Fixes Erroneous clear/trigger of timeout interrupt
 *  - If timeout interrupt is erroneously set, and the FIFO is empty
 *      - Set a high value of timeout counter in TIMEOUTH and TIMEOUTL registers
 *      - Set EFR2 bit 6 to 1 to change timeout mode to periodic
 *      - Read the IIR register to clear the interrupt
 *      - Set EFR2 bit 6 back to 0 to change timeout mode back to the original mode
 *
 * Errata document : https://www.ti.com/lit/pdf/sprz457
 */
static void UART_i2310WA(uint32_t baseAddr)
{
    HW_WR_REG32(baseAddr + UART_TIMEOUTL, 0xFF);

    HW_WR_REG32(baseAddr + UART_TIMEOUTH, 0xFF);

    HW_WR_FIELD32(baseAddr + UART_EFR2, UART_EFR2_TIMEOUT_BEHAVE, 1);

    HW_WR_FIELD32(baseAddr + UART_EFR2, UART_EFR2_TIMEOUT_BEHAVE, 0);
}

static int32_t UART_readPolling(UARTLLD_Handle hUart, UART_Transaction *trans)
{
    uint32_t            timeout, startTicks, elapsedTicks;
    int32_t             retVal          = UART_TRANSFER_STATUS_SUCCESS;
    uint32_t            timeoutElapsed  = FALSE;
    UARTLLD_InitHandle        hUartInit;

    timeout = trans->timeout;
    hUart->readSizeRemaining = trans->count;
    hUartInit = hUart->hUartInit;
    /* Update current tick value to perform timeout operation */
    startTicks =  hUartInit->clockP_get();
    while ((FALSE == timeoutElapsed)
           && (0U != hUart->readSizeRemaining))
    {
        /* Transfer DATA */
        UART_readDataPolling(hUart);
        /* Check whether timeout happened or not */
        elapsedTicks = hUartInit->clockP_get() - startTicks;
        if (elapsedTicks >= timeout)
        {
            /* timeout occured */
            timeoutElapsed = TRUE;
        }
    }

    if ((hUart->readSizeRemaining == 0U) &&
        (hUart->readErrorCnt == 0U) && (hUart->rxTimeoutCnt == 0U))
    {
        retVal             = UART_TRANSFER_STATUS_SUCCESS;
        trans->status      = UART_TRANSFER_STATUS_SUCCESS;
        UART_lld_Transaction_deInit(&hUart->readTrans);
    }
    else
    {
        /* Return UART_TRANSFER_TIMEOUT so that application gets whatever bytes are
         * transmitted. Set the trans status to timeout so that
         * application can handle the timeout. */
        retVal             = UART_TRANSFER_TIMEOUT;
        trans->status      = UART_TRANSFER_STATUS_TIMEOUT;
        trans->count       = hUart->readCount;
        UART_lld_Transaction_deInit(&hUart->readTrans);
    }

    return (retVal);
}

static int32_t UART_readPollingWithCounter(UARTLLD_Handle hUart, UART_Transaction *trans)
{
    int32_t             retVal  = UART_TRANSFER_STATUS_SUCCESS;
    uint32_t            timeout = 0x6ffff;
    hUart->readSizeRemaining = trans->count;

    /* timeout operation */
    while (( timeout > (uint32_t) 0U) && (0U != hUart->readSizeRemaining))
    {
        /* Transfer DATA */
        UART_readDataPolling(hUart);
        /* Check whether timeout happened or not */
        if ( timeout > (uint32_t) 0U)
        {
            timeout--;
        }
    }

    if ((hUart->readSizeRemaining == 0U) &&
        (hUart->readErrorCnt == 0U) && (hUart->rxTimeoutCnt == 0U))
    {
        retVal             = UART_TRANSFER_STATUS_SUCCESS;
        trans->status      = UART_TRANSFER_STATUS_SUCCESS;
        UART_lld_Transaction_deInit(&hUart->readTrans);
    }
    else
    {
        /* Return UART_TRANSFER_TIMEOUT so that application gets whatever bytes are
         * transmitted. Set the trans status to timeout so that
         * application can handle the timeout. */
        retVal             = UART_TRANSFER_TIMEOUT;
        trans->status      = UART_TRANSFER_STATUS_TIMEOUT;
        trans->count       = hUart->readCount;
        UART_lld_Transaction_deInit(&hUart->readTrans);
    }

    return (retVal);
}

static void UART_readDataPolling(UARTLLD_Handle hUart)
{
    uint32_t numBytesRead = 0U;

    numBytesRead = UART_fifoRead(hUart, (uint8_t *) hUart->readBuf,
                                 hUart->readSizeRemaining);

    hUart->readSizeRemaining -= numBytesRead;
    hUart->readBuf           = (void *)((uint8_t *)hUart->readBuf + numBytesRead);
    hUart->readCount         += numBytesRead;

    return;
}

static uint32_t UART_fifoRead(UARTLLD_Handle hUart, uint8_t *buffer,
                              uint32_t readSizeRemaining)
{
    uint32_t tempReadSizeRemaining = readSizeRemaining;
    uint32_t size    = tempReadSizeRemaining;
    Bool isRxReady = FALSE;
    uint8_t *tempBuffer = buffer;

    isRxReady = UART_statusIsDataReady(hUart);

    while (((Bool)TRUE == isRxReady) && (0U != readSizeRemaining))
    {
        /* once the H/w is ready  reading from the H/w                        */
        *tempBuffer = (UInt8) UART_readByte(hUart);
        tempBuffer++;
        --tempReadSizeRemaining;

        isRxReady = UART_statusIsDataReady(hUart);
    }

    return (size - tempReadSizeRemaining);
}

int32_t UART_procLineStatusErr(UARTLLD_Handle hUart)
{
    int32_t            status = UART_STATUS_SUCCESS;
    uint32_t           lineStatus, iteration = 0U;

    if(NULL_PTR == hUart)
    {
        status = UART_INVALID_PARAM;
    }

    if(UART_STATUS_SUCCESS == status)
    {
        lineStatus = UART_readLineStatus(hUart->baseAddr);

        if(((lineStatus & UART_FIFO_PE_FE_BI_DETECTED) == UART_FIFO_PE_FE_BI_DETECTED)
                || ((lineStatus & UART_OVERRUN_ERROR) == UART_OVERRUN_ERROR))
        {
            /* empty the RX FIFO which contains data with errors */
            if (hUart->readTrans.buf != NULL)
            {
                hUart->readTrans.count = (uint32_t)(hUart->readCount);
            }

            /* Clearing Receive Errors(FE,BI,PE)by reading erroneous data from RX FIFO */
            /* Iteration count: Worst case = FIFO size */
            iteration = UART_FIFO_SIZE;
            do
            {
                /* Read and throw error byte */
                /* Till Line status int is pending */
                (void)UART_fifoCharGet(hUart->baseAddr);

                iteration--;

                lineStatus = (uint32_t) UART_readLineStatus(hUart->baseAddr);
                lineStatus &= (UART_LSR_RX_FIFO_STS_MASK |
                        UART_LSR_RX_BI_MASK |
                        UART_LSR_RX_FE_MASK |
                        UART_LSR_RX_PE_MASK |
                        UART_LSR_RX_OE_MASK |
                        UART_LSR_RX_FIFO_E_MASK);
            }
            while ((lineStatus != 0U) && (iteration != 0U));

            UART_intrDisable(hUart->baseAddr, UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);

            /* Reset the read buffer and read count so we can pass it back */
            hUart->readBuf = (void *)((uint8_t *)hUart->readBuf - hUart->readCount);

            if (hUart->readTrans.buf != NULL)
            {
                if ((lineStatus & UART_BREAK_DETECTED_ERROR) != 0U)
                {
                    hUart->readTrans.status = UART_TRANSFER_STATUS_ERROR_BI;
                    hUart->readErrorCnt++;
                }
                else if ((lineStatus & UART_FRAMING_ERROR) != 0U)
                {
                    hUart->readTrans.status = UART_TRANSFER_STATUS_ERROR_FE;
                    hUart->readErrorCnt++;
                }
                else if ((lineStatus & UART_PARITY_ERROR) != 0U)
                {
                    hUart->readTrans.status = UART_TRANSFER_STATUS_ERROR_PE;
                    hUart->readErrorCnt++;
                }
                else
                {
                    hUart->readTrans.status = UART_TRANSFER_STATUS_ERROR_OE;
                    hUart->readErrorCnt++;
                }
            }

            hUart->hUartInit->errorCallbackFxn(hUart);
            UART_lld_Transaction_deInit(&hUart->readTrans);
        }
    }

    return status;
}

static Bool UART_statusIsDataReady(UARTLLD_Handle hUart)
{
    uint32_t status = 0;
    Bool retVal   = FALSE;

    status = UART_readLineStatus(hUart->baseAddr);

    /* Added for error checks */
    if ((uint32_t) UART_LSR_RX_FIFO_STS_MASK ==
        (status & (uint32_t) UART_LSR_RX_FIFO_STS_MASK))
    {
        (void)UART_procLineStatusErr(hUart);
    }
    /* Caution: This should be under if else of error check since
     * the RX error handler clears the FIFO. Hence the status we have read
     * before this call will become stale. Hence the data will not be
     * ready in FIFO. Otherwise we will read the FIFO register which has
     * a infinite loop for data ready and the code hangs there till user
     * gives any character!! */
    else if ((uint32_t) UART_LSR_RX_FIFO_E_MASK ==
             (status & (uint32_t) UART_LSR_RX_FIFO_E_MASK))
    {
        retVal = TRUE;
    }
    else
    {
        /* Do nothing */
    }

    return retVal;
}

static int32_t UART_readInterrupt(UARTLLD_Handle hUart, UART_Transaction *trans)
{
    int32_t             status = UART_TRANSFER_STATUS_SUCCESS;
    uint32_t            baseAddr;

    if(NULL_PTR != hUart)
    {
        baseAddr = hUart->baseAddr;
        /* Disable RX threshold and line status error interrupt */
        UART_intrDisable(baseAddr, UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);

        hUart->readSizeRemaining = UART_readData(hUart, trans->count);
        if ((hUart->readSizeRemaining) == 0U)
        {
            /* Update the actual read count */
            trans->count = (uint32_t)(hUart->readCount);
            trans->status = UART_TRANSFER_STATUS_SUCCESS;

            hUart->hUartInit->readCompleteCallbackFxn(hUart);

            status = UART_TRANSFER_STATUS_SUCCESS;
            UART_lld_Transaction_deInit(&hUart->readTrans);
        }
        else
        {
            /* Enable Rx, read time-out and RX Line Error interrupt */
            UART_intrEnable(baseAddr, (uint32_t) UART_INTR_RHR_CTI | UART_INTR_LINE_STAT);
        }
    }

    return status;
}

void UART_putChar(uint32_t baseAddr, uint8_t byteTx)
{
    /* Write the byte to the Transmit Holding Register(or TX FIFO). */
    HW_WR_REG32(baseAddr + UART_THR, (uint32_t) byteTx);
}

uint32_t UART_getChar(uint32_t baseAddr, uint8_t *pChar)
{
    uint32_t lcrRegValue = 0U;
    uint32_t retVal      = FALSE;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    /* Checking if the RX FIFO(or RHR) has atleast one byte of data. */
    if ((uint32_t) UART_LSR_RX_FIFO_E_RX_FIFO_E_VALUE_0 !=
        (HW_RD_REG32(baseAddr + UART_LSR) &
         UART_LSR_RX_FIFO_E_MASK))
    {
        uint32_t tempRetVal = HW_RD_REG32(baseAddr + UART_RHR);
        *pChar = (uint8_t)tempRetVal;
        retVal = TRUE;
    }

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

static inline uint32_t UART_readData(UARTLLD_Handle hUart, uint32_t size)
{
    uint8_t             readIn = 0;
    uint32_t            readSuccess;
    uint32_t             rdSize = size;

    readSuccess = UART_getChar(hUart->baseAddr, &readIn);

    /* Receive chars until empty or done. */
    while ((rdSize != 0U) && (readSuccess != 0U))
    {
        *(uint8_t *)hUart->readBuf = readIn;
        hUart->readBuf = (uint8_t *)hUart->readBuf + 1U;
        hUart->readCount++;
        rdSize--;

        /* If read returnMode is UART_RETURN_FULL, avoids missing input character
         * of next read
         */
        if (rdSize != 0U)
        {
            readSuccess = UART_getChar(hUart->baseAddr, &readIn);
        }
    }

    return (rdSize);
}

static inline uint32_t UART_writeData(UARTLLD_Handle hUart, uint32_t writeSizeRemaining)
{
    uint32_t numBytesToTransfer, numBytesToTransferred;
    UARTLLD_InitHandle hUartInit = hUart->hUartInit;

    /* In interrupt mode write only threshold level of data with FIFO enabled */
    numBytesToTransfer = writeSizeRemaining;
    if (numBytesToTransfer >= hUartInit->txTrigLvl)
    {
        numBytesToTransfer = hUartInit->txTrigLvl;
    }

    numBytesToTransferred = numBytesToTransfer;
    /* Send characters until FIFO threshold level or done. */
    while (numBytesToTransfer != 0U)
    {
        UART_putChar(hUart->baseAddr, *(const uint8_t *)hUart->writeBuf);
        hUart->writeBuf = (const uint8_t *)hUart->writeBuf + 1U;

        numBytesToTransfer--;
        hUart->writeCount++;
    }

    return (writeSizeRemaining - numBytesToTransferred);
}

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* ========================================================================== */
/*                       Advanced Function Definitions                        */
/* ========================================================================== */

void UART_intrEnable(uint32_t baseAddr, uint32_t intrFlag)
{
    uint32_t enhanFnBitVal = 0U;
    uint32_t lcrRegValue   = 0U;

    /* Switch to mode B only when the upper 4 bits of IER needs to be changed */
    if ((intrFlag & 0xF0U) > 0U)
    {
        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
        /* Switching to Register Configuration Mode B. */
        HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

        /* Collecting the current value of EFR[4] and later setting it. */
        enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

        HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                      UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

        /* Switching to Register Operational Mode of operation. */
        HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                                & 0x7FU);

        /*
        ** It is suggested that the System Interrupts for UART in the
        ** Interrupt Controller are enabled after enabling the peripheral
        ** interrupts of the UART using this API. If done otherwise, there
        ** is a risk of LCR value not getting restored and illicit characters
        ** transmitted or received from/to the UART. The situation is explained
        ** below.
        ** The scene is that the system interrupt for UART is already enabled
        ** and the current API is invoked. On enabling the interrupts
        ** corresponding to IER[7:4] bits below, if any of those interrupt
        ** conditions already existed, there is a possibility that the control
        ** goes to Interrupt Service Routine (ISR) without executing the
        ** remaining statements in this API. Executing the remaining statements
        ** is critical in that the LCR value is restored in them.
        ** However, there seems to be no risk in this API for enabling
        ** interrupts corresponding to IER[3:0] because it is done at the end
        ** and no statements follow that.
        */

        /************* ATOMIC STATEMENTS START *************************/

        /* Programming the bits IER[7:4]. */
        HW_WR_REG32(baseAddr + UART_IER, intrFlag & 0xF0U);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
        /* Switching to Register Configuration Mode B. */
        HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

        /* Restoring the value of EFR[4] to its original value. */
        HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

        /************** ATOMIC STATEMENTS END *************************/
    }

    /* Programming the bits IER[3:0]. */
    HW_WR_REG32(baseAddr + UART_IER, HW_RD_REG32(baseAddr + UART_IER) |
                (intrFlag & 0x0FU));
}

void UART_intrDisable(uint32_t baseAddr, uint32_t intrFlag)
{
    uint32_t enhanFnBitVal;
    uint32_t lcrRegValue;

    /* Switch to mode B only when the upper 4 bits of IER needs to be changed */
    if((intrFlag & 0xF0U) > 0U)
    {
        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
        /* Switching to Register Configuration Mode B. */
        HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

        /* Collecting the current value of EFR[4] and later setting it. */
        enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

        HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                      UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
    }

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    HW_WR_REG32(baseAddr + UART_IER, HW_RD_REG32(baseAddr + UART_IER) &
                ~(intrFlag & 0xFFU));

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switch to mode B only when the upper 4 bits of IER needs to be changed */
    if((intrFlag & 0xF0U) > 0U)
    {
        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
        /* Switching to Register Configuration Mode B. */
        HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

        /* Restoring the value of EFR[4] to its original value. */
        HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
    }
}

void UART_intr2Enable(uint32_t baseAddr, uint32_t intrFlag)
{
    /* Programming the bits IER2[1:0]. */
    HW_WR_REG32(baseAddr + UART_IER2, HW_RD_REG32(baseAddr + UART_IER2) |
                (intrFlag & 0x03U));
}

void UART_intr2Disable(uint32_t baseAddr, uint32_t intrFlag)
{
    HW_WR_REG32(baseAddr + UART_IER2, HW_RD_REG32(baseAddr + UART_IER2) &
                ~(intrFlag & 0x3U));
}

uint32_t UART_getIntrIdentityStatus(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0U;
    uint32_t retVal      = 0U;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    retVal = HW_RD_REG32(baseAddr + UART_IIR) & UART_IIR_IT_TYPE_MASK;

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

uint32_t UART_getIntr2Status(uint32_t baseAddr)
{
    uint32_t retVal = 0U;

    retVal = HW_RD_REG32(baseAddr + UART_ISR2) &
        (UART_IER2_EN_RXFIFO_EMPTY_MASK | UART_IER2_EN_TXFIFO_EMPTY_MASK);

    return retVal;
}

uint32_t UART_checkCharsAvailInFifo(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0;
    uint32_t retVal      = FALSE;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    /* Checking if the RHR(or RX FIFO) has atleast one byte to be read. */
    if ((uint32_t) UART_LSR_RX_FIFO_E_RX_FIFO_E_VALUE_0 !=
        (HW_RD_REG32(baseAddr + UART_LSR) &
         UART_LSR_RX_FIFO_E_MASK))
    {
        retVal = (uint32_t) TRUE;
    }

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

uint32_t UART_readLineStatus(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0U;
    uint32_t retVal      = 0U;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    retVal = HW_RD_REG32(baseAddr + UART_LSR);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

uint8_t UART_getCharFifo(uint32_t baseAddr, uint8_t *readBuf)
{
    uint8_t           readByte = 0;
    uint32_t          waitCount = UART_ERROR_COUNT;
    uint32_t          errorVal;
    uint32_t          lcrRegValue = 0;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    /* Read Rx Error Status */
    errorVal = HW_RD_REG32(baseAddr + UART_LSR) &
             (UART_LSR_RX_FIFO_STS_MASK |
              UART_LSR_RX_BI_MASK |
              UART_LSR_RX_FE_MASK |
              UART_LSR_RX_PE_MASK |
              UART_LSR_RX_OE_MASK);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Read and throw Erroneous bytes from RxFIFO */
    while ((UART_LSR_RX_FIFO_STS_MASK |
            UART_LSR_RX_BI_MASK |
            UART_LSR_RX_FE_MASK |
            UART_LSR_RX_PE_MASK |
            UART_LSR_RX_OE_MASK) == errorVal)
    {
        readByte = (uint8_t)HW_RD_REG32(baseAddr + UART_RHR);
        waitCount--;
        if (0U == waitCount)
        {
            break;
        }

        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

        /* Switching to Register Operational Mode of operation. */
        HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                                & 0x7FU);

        /* Read Rx Error Status */
        errorVal = HW_RD_REG32(baseAddr + UART_LSR) &
                 (UART_LSR_RX_FIFO_STS_MASK |
                  UART_LSR_RX_BI_MASK |
                  UART_LSR_RX_FE_MASK |
                  UART_LSR_RX_PE_MASK |
                  UART_LSR_RX_OE_MASK);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
    }

    /* Read non-erroneous byte from RxFIFO */
    readByte = (uint8_t)HW_RD_REG32(baseAddr + UART_RHR);

    return readByte;
}

void UART_lld_Transaction_deInit(UART_Transaction *trans)
{
    trans->buf              = NULL;
    trans->count            = 0U;
    trans->timeout          = 0U;
    trans->status           = UART_STATUS_SUCCESS;
    trans->args             = NULL;
}

void UART_lld_Transaction_init(UART_Transaction *trans)
{
    if(trans != NULL)
    {
        trans->buf              = NULL;
        trans->count            = 0U;
        trans->timeout          = UART_WAIT_FOREVER;
        trans->status           = UART_STATUS_SUCCESS;
        trans->args             = NULL;
    }
}
