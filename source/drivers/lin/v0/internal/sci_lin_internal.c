/*
 * Copyright (C) 2025 Texas Instruments Incorporated
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
 *  \file   sci_lin_internal.c
 *
 *  \brief  File containing LIN Driver APIs implementation for V0.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <drivers/lin/v0/internal/sci_lin_internal.h>

/* ========================================================================== */
/*                      Internal Function Definitions                         */
/* ========================================================================== */

/* Register Access Related Internal Functions */
void LIN_HLD_EnableTxRxPin(uint32_t baseAddr)
{
    /* Disable TX and RX pin control functionality. */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIPIO0),
                        CSL_LIN_SCIPIO0_RXFUNC_MASK,
                        CSL_LIN_SCIPIO0_RXFUNC_SHIFT, CSL_TRUE);
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIPIO0),
                        CSL_LIN_SCIPIO0_TXFUNC_MASK,
                        CSL_LIN_SCIPIO0_TXFUNC_SHIFT, CSL_TRUE);
}

void LIN_HLD_DisableTxRxPin(uint32_t baseAddr)
{
    /* Enable TX and RX pin control functionality. */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIPIO0),
                        CSL_LIN_SCIPIO0_RXFUNC_MASK,
                        CSL_LIN_SCIPIO0_RXFUNC_SHIFT, CSL_TRUE);
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIPIO0),
                        CSL_LIN_SCIPIO0_TXFUNC_MASK,
                        CSL_LIN_SCIPIO0_TXFUNC_SHIFT, CSL_TRUE);
}

void LIN_HLD_ModuleResetEnter(uint32_t baseAddr)
{
    /* Set reset bit. Module is held in Reset state */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR0),
                        CSL_LIN_SCIGCR0_RESET_MASK,
                        CSL_LIN_SCIGCR0_RESET_SHIFT, CSL_FALSE);
}

void LIN_HLD_ModuleResetExit(uint32_t baseAddr)
{
    /* Set reset bit. Module is out of Reset */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR0),
                        CSL_LIN_SCIGCR0_RESET_MASK,
                        CSL_LIN_SCIGCR0_RESET_SHIFT, CSL_TRUE);
}

void LIN_HLD_ModuleEnterSoftReset(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_SWNRST_MASK,
                        CSL_LIN_SCIGCR1_SWNRST_SHIFT, CSL_FALSE);
}

void LIN_HLD_ModuleExitSoftReset(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_SWNRST_MASK,
                        CSL_LIN_SCIGCR1_SWNRST_SHIFT, CSL_TRUE);
}

void LIN_HLD_setModuleMode(uint32_t baseAddr, LIN_HLD_ModuleMode mode)
{
    if (mode == LIN_MODULE_OP_MODE_LIN)
    {
        HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                            CSL_LIN_SCIGCR1_TIMINGMODE_MASK,
                            CSL_LIN_SCIGCR1_TIMINGMODE_SHIFT, CSL_FALSE);

        HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                            CSL_LIN_SCIGCR1_LINMODE_MASK,
                            CSL_LIN_SCIGCR1_LINMODE_SHIFT, CSL_TRUE);
    }
    else
    {

        HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                            CSL_LIN_SCIGCR1_LINMODE_MASK,
                            CSL_LIN_SCIGCR1_LINMODE_SHIFT, CSL_FALSE);

        HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                            CSL_LIN_SCIGCR1_CLK_MASTER_MASK,
                            CSL_LIN_SCIGCR1_CLK_MASTER_SHIFT, CSL_TRUE);

        HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                            CSL_LIN_SCIGCR1_TIMINGMODE_MASK,
                            CSL_LIN_SCIGCR1_TIMINGMODE_SHIFT, CSL_TRUE);
    }
}

void LIN_HLD_setLinOpMode(uint32_t baseAddr, LIN_HLD_LinMode mode)
{
    /* Program LIN Mode */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_CLK_MASTER_MASK,
                        CSL_LIN_SCIGCR1_CLK_MASTER_SHIFT, mode);
}

void LIN_HLD_enableAutoBaud(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_ADAPT_MASK,
                        CSL_LIN_SCIGCR1_ADAPT_SHIFT, CSL_TRUE);
}

void LIN_HLD_disableAutoBaud(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_ADAPT_MASK,
                        CSL_LIN_SCIGCR1_ADAPT_SHIFT, CSL_FALSE);
}

void LIN_HLD_setCommMode(uint32_t baseAddr, LIN_HLD_CommMode mode)
{
    /* Write Communication Mode Selection to the Appropriate bit. */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_COMMMODE_MASK,
                        CSL_LIN_SCIGCR1_COMMMODE_SHIFT, mode);
}

void LIN_HLD_setDebugMode(uint32_t baseAddr, LIN_HLD_DebugMode mode)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_CONT_MASK,
                        CSL_LIN_SCIGCR1_CONT_SHIFT, mode);
}

void LIN_HLD_setChecksumType(uint32_t baseAddr, LIN_HLD_ChecksumType type)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_CTYPE_MASK,
                        CSL_LIN_SCIGCR1_CTYPE_SHIFT, type);
}

void LIN_HLD_setMaskFilterType(uint32_t baseAddr, LIN_HLD_MaskFilterType type)
{
    /* Sets the message filtering type */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_HGENCTRL_MASK,
                        CSL_LIN_SCIGCR1_HGENCTRL_SHIFT, type);
}

void LIN_HLD_enableInternalLoopback(uint32_t baseAddr)
{
    /* Set loopback Type */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_LOOPBACK_MASK,
                        CSL_LIN_SCIGCR1_LOOPBACK_SHIFT, CSL_TRUE);
}

void LIN_HLD_disableInternalLoopback(uint32_t baseAddr)
{
    /* Set loopback Type */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_LOOPBACK_MASK,
                        CSL_LIN_SCIGCR1_LOOPBACK_SHIFT, CSL_FALSE);
}

void LIN_HLD_enableMultiBufferMode(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_MBUFMODE_MASK,
                        CSL_LIN_SCIGCR1_MBUFMODE_SHIFT, CSL_TRUE);
}

void LIN_HLD_disableMultiBufferMode(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_MBUFMODE_MASK,
                        CSL_LIN_SCIGCR1_MBUFMODE_SHIFT, CSL_FALSE);
}

void LIN_HLD_enableParity(uint32_t baseAddr)
{
    /* Enable Parity */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_PARITYENA_MASK,
                        CSL_LIN_SCIGCR1_PARITYENA_SHIFT, CSL_TRUE);
}

void LIN_HLD_disableParity(uint32_t baseAddr)
{
    /* Disable Parity */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_PARITYENA_MASK,
                        CSL_LIN_SCIGCR1_PARITYENA_SHIFT, CSL_FALSE);
}

void LIN_HLD_setParityType(uint32_t baseAddr, LIN_HLD_SCIParityType type)
{
    /* Set Parity Type */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_PARITY_MASK,
                        CSL_LIN_SCIGCR1_PARITY_SHIFT, type);
}

void LIN_HLD_setStopBits(uint32_t baseAddr, LIN_HLD_SCIStopBits bits)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_STOP_MASK,
                        CSL_LIN_SCIGCR1_STOP_SHIFT, bits);
}

void LIN_HLD_enableDataTx(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_TXENA_MASK,
                        CSL_LIN_SCIGCR1_TXENA_SHIFT, CSL_TRUE);
}

void LIN_HLD_disableDataTx(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_TXENA_MASK,
                        CSL_LIN_SCIGCR1_TXENA_SHIFT, CSL_FALSE);
}

void LIN_HLD_enableDataRx(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_RXENA_MASK,
                        CSL_LIN_SCIGCR1_RXENA_SHIFT, CSL_TRUE);
}

void LIN_HLD_disableDataRx(uint32_t baseAddr)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR1),
                        CSL_LIN_SCIGCR1_RXENA_MASK,
                        CSL_LIN_SCIGCR1_RXENA_SHIFT, CSL_FALSE);
}

void LIN_HLD_trigChecksumCompare(uint32_t baseAddr)
{
    /* Comparing the Check Sum */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIGCR2),
                        CSL_LIN_SCIGCR2_CC_MASK,
                        CSL_LIN_SCIGCR2_CC_SHIFT, CSL_TRUE);
}

void LIN_HLD_setBaudParams(uint32_t baseAddr, LIN_BaudConfigParams *params)
{
    /* Set baud rate Pre-scaler, Fractional & Super-Fractional divider */
    HW_WR_REG32_RAW((baseAddr + CSL_LIN_BRSR),
                    (   (params->preScaler) |
                        ((uint32_t)(params->fracDivSel_M) << CSL_LIN_BRSR_M_SHIFT) |
                        ((uint32_t)(params->supFracDivSel_U) << CSL_LIN_BRSR_U_SHIFT)));
}

void LIN_HLD_setMaxBaudRate(uint32_t baseAddr, uint32_t linClk,
                            uint32_t maxBaud)
{
    /* Calculate maximum baud rate Pre-scaler 
     * MBR = (0.9 * clockVal / baudrate)
     * where:
     * - `clockVal` is the clock frequency driving the LIN module.
     * - `baudrate` is the desired communication baud rate.
    */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_MBRSR), CSL_LIN_MBRSR_MBR_MASK,
                        CSL_LIN_MBRSR_MBR_SHIFT, (uint32_t)(0.9 * linClk / maxBaud));
}

void LIN_HLD_setFrameLength(uint32_t baseAddr, uint8_t length)
{
    /* Clear and Set frame length value */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIFORMAT),
                        CSL_LIN_SCIFORMAT_LENGTH_MASK,
                        CSL_LIN_SCIFORMAT_LENGTH_SHIFT,
                        ((uint32_t)length - (uint32_t)1U));
}

uint32_t LIN_HLD_getFrameLength(uint32_t baseAddr)
{
    uint32_t frameLen = 0U;

    frameLen = HW_RD_REG32_RAW( (baseAddr + CSL_LIN_SCIFORMAT) &
                                CSL_LIN_SCIFORMAT_LENGTH_MASK);
    frameLen = frameLen >> CSL_LIN_SCIFORMAT_LENGTH_SHIFT;

    return frameLen;
}

void LIN_HLD_setCharLength(uint32_t baseAddr, uint8_t length)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCIFORMAT),
                        CSL_LIN_SCIFORMAT_CHAR_MASK,
                        CSL_LIN_SCIFORMAT_CHAR_SHIFT,
                        ((uint32_t)length - (uint32_t)1U));
}

uint32_t LIN_HLD_getCharLength(uint32_t baseAddr)
{
    uint32_t charLen = 0U;

    charLen = HW_RD_REG32_RAW( (baseAddr + CSL_LIN_SCIFORMAT) &
                                CSL_LIN_SCIFORMAT_CHAR_MASK);
    charLen = charLen >> CSL_LIN_SCIFORMAT_CHAR_SHIFT;

    return charLen;
}

void LIN_HLD_setSyncFields( uint32_t baseAddr, uint32_t syncBrk,
                            uint32_t syncDelimiter)
{
    /* Clear Sync values and set new values */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_LINCOMP),
                        CSL_LIN_LINCOMP_SDEL_MASK,
                        CSL_LIN_LINCOMP_SDEL_SHIFT, (syncDelimiter));
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_LINCOMP),
                        CSL_LIN_LINCOMP_SBREAK_MASK,
                        CSL_LIN_LINCOMP_SBREAK_SHIFT, syncBrk);
}

void LIN_HLD_setTxMask(uint32_t baseAddr, uint32_t mask)
{
    /* Clear previous Mask value and set new Mask */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_LINMASK),
                        CSL_LIN_LINMASK_TXIDMASK_MASK,
                        CSL_LIN_LINMASK_TXIDMASK_SHIFT, mask);
}

void LIN_HLD_setRxMask(uint32_t baseAddr, uint32_t mask)
{
    /* Clear previous Mask value and set new Mask */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_LINMASK),
                        CSL_LIN_LINMASK_RXIDMASK_MASK,
                        CSL_LIN_LINMASK_RXIDMASK_SHIFT, mask);
}

void LIN_HLD_disableExternalLoopback(uint32_t baseAddr)
{
    /* Enable the IO DFT Enable Key */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_IODFTCTRL),
                        CSL_LIN_IODFTCTRL_IODFTENA_MASK,
                        CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, LIN_IODFTENA_KEY);

    /* Disable Analog Loopback */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_IODFTCTRL),
                        CSL_LIN_IODFTCTRL_LPBENA_MASK,
                        CSL_LIN_IODFTCTRL_LPBENA_SHIFT, CSL_FALSE);

    /* Module Analog Loopback through Receive Pin Enable */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_IODFTCTRL),
                        CSL_LIN_IODFTCTRL_RXPENA_MASK,
                        CSL_LIN_IODFTCTRL_RXPENA_SHIFT, CSL_FALSE);

    /* Disable the IO DFT Enable Key */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_IODFTCTRL),
                        CSL_LIN_IODFTCTRL_IODFTENA_MASK,
                        CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, CSL_FALSE);
}

void LIN_HLD_enableExternalLoopback(uint32_t baseAddr,
                                    LIN_HLD_LoopbackType type,
                                    LIN_HLD_LoopbackPath path)
{
    /* Enable the IO DFT Enable Key */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_IODFTCTRL),
                        CSL_LIN_IODFTCTRL_IODFTENA_MASK,
                        CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, LIN_IODFTENA_KEY);
    /* Set loopback Type */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_IODFTCTRL),
                        CSL_LIN_IODFTCTRL_LPBENA_MASK,
                        CSL_LIN_IODFTCTRL_LPBENA_SHIFT, type);
    /* Set Analog Loopback Path */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_IODFTCTRL),
                        CSL_LIN_IODFTCTRL_RXPENA_MASK,
                        CSL_LIN_IODFTCTRL_RXPENA_SHIFT, path);

    /* Disable the IO DFT Enable Key */
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_IODFTCTRL),
                        CSL_LIN_IODFTCTRL_IODFTENA_MASK,
                        CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, CSL_FALSE);
}

void LIN_HLD_readLinRxBuffer(uint32_t baseAddr, uint8_t *data,
                                           uint8_t length)
{
    uint8_t i = 0;

    /* Read each 8-bit piece of data. */
    for(i = 0U; i <= length; i++)
    {
        *data = HW_RD_REG8_RAW(baseAddr + CSL_LIN_LINRD0 + (uint32_t)i ^ 3U);
        data++;
    }
}

void LIN_HLD_writeLinTxBuffer(uint32_t baseAddr, uint8_t *data, uint8_t length)
{
    int8_t i = 0;
    uint8_t *pData;

    pData = data + length;

    for(i = (int8_t)length; i >= 0; i--)
    {
        HW_WR_REG8_RAW( (baseAddr + CSL_LIN_LINTD0 + (uint32_t)i ^ 3U),
                        (uint8_t)*pData);
        pData--;
    }
}

void LIN_HLD_writeSciTxReg(uint32_t baseAddr, uint8_t *data)
{
    /* Write to the TX data Reg */
    uint8_t txData = *data;
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_SCITD),
                        CSL_LIN_SCITD_TD_MASK, CSL_LIN_SCITD_TD_SHIFT, txData);
}

void LIN_HLD_readSciRxReg(uint32_t baseAddr, uint8_t *data, uint8_t charLength)
{
    uint32_t value = 0U;
    value = (HW_RD_REG32_RAW(baseAddr + CSL_LIN_SCIRD) & CSL_LIN_SCIRD_RD_MASK);
    *data = (uint8_t)(value >> (charLength - 8U));
}

void LIN_HLD_setIDByte(uint32_t baseAddr, uint8_t identifier)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_LINID),
                        CSL_LIN_LINID_IDBYTE_MASK,
                        CSL_LIN_LINID_IDBYTE_SHIFT, identifier);
}

uint8_t LIN_HLD_getReceivedID(uint32_t baseAddr)
{
    return (uint8_t)HW_RD_FIELD32_RAW(  (baseAddr + CSL_LIN_LINID),
                                        CSL_LIN_LINID_RECEIVEDID_MASK,
                                        CSL_LIN_LINID_RECEIVEDID_SHIFT);
}

void LIN_HLD_setIDSlaveTaskByte(uint32_t baseAddr, uint8_t slaveTaskByteID)
{
    HW_WR_FIELD32_RAW(  (baseAddr + CSL_LIN_LINID),
                        CSL_LIN_LINID_IDSLAVETASKBYTE_MASK,
                        CSL_LIN_LINID_IDSLAVETASKBYTE_SHIFT, slaveTaskByteID);
}

/* Interrupt Related Internal Functions */
void LIN_HLD_enableInterrupt(uint32_t baseAddr, uint32_t intrMask)
{
    /* Enable Interrupts */
    HW_WR_REG32_RAW(    (baseAddr + CSL_LIN_SCISETINT),
                        (HW_RD_REG32_RAW(baseAddr + CSL_LIN_SCISETINT) |
                        intrMask));
}

void LIN_HLD_disableInterrupt(uint32_t baseAddr,
                                            uint32_t intrMask)
{
    /* Disable Interrupts */
    HW_WR_REG32_RAW((baseAddr + CSL_LIN_SCICLEARINT), intrMask);
}

void LIN_HLD_clearInterrupt(uint32_t baseAddr, uint32_t intrMask)
{
    /* Clear Status Flags */
    HW_WR_REG32_RAW((baseAddr + CSL_LIN_SCIFLR), intrMask);
}

void LIN_HLD_setInterruptLevel0(uint32_t baseAddr, uint32_t intrMask)
{
    /* Clear Status Flags */
    HW_WR_REG32_RAW((baseAddr + CSL_LIN_SCICLEARINTLVL), intrMask);
}

void LIN_HLD_setInterruptLevel1(uint32_t baseAddr, uint32_t intrMask)
{
    /* Clear Status Flags */
    HW_WR_REG32_RAW((baseAddr + CSL_LIN_SCISETINTLVL), intrMask);
}

void LIN_HLD_enableGlobalInterrupt0(uint32_t baseAddr)
{
    uint32_t regVal = HW_RD_REG32_RAW(baseAddr + CSL_LIN_LIN_GLB_INT_EN);

    HW_WR_REG32_RAW((baseAddr + CSL_LIN_LIN_GLB_INT_EN),
                    (regVal | CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK));
}

void LIN_HLD_disableGlobalInterrupt0(uint32_t baseAddr)
{
    uint32_t regVal = HW_RD_REG32_RAW(baseAddr + CSL_LIN_LIN_GLB_INT_EN);

    HW_WR_REG32_RAW((baseAddr + CSL_LIN_LIN_GLB_INT_EN),
                    (regVal & (~CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK)));
}

void LIN_HLD_enableGlobalInterrupt1(uint32_t baseAddr)
{
    uint32_t regVal = HW_RD_REG32_RAW(baseAddr + CSL_LIN_LIN_GLB_INT_EN);

    HW_WR_REG32_RAW((baseAddr + CSL_LIN_LIN_GLB_INT_EN),
                    (regVal | CSL_LIN_LIN_GLB_INT_FLG_INT1_FLG_MASK));
}

void LIN_HLD_disableGlobalInterrupt1(uint32_t baseAddr)
{
    uint32_t regVal = HW_RD_REG32_RAW(baseAddr + CSL_LIN_LIN_GLB_INT_EN);

    HW_WR_REG32_RAW((baseAddr + CSL_LIN_LIN_GLB_INT_EN),
                    (regVal & (~CSL_LIN_LIN_GLB_INT_FLG_INT1_FLG_MASK)));
}

void LIN_HLD_clearGlobalInterrupt0(uint32_t baseAddr)
{
    HW_WR_REG32_RAW((baseAddr + CSL_LIN_LIN_GLB_INT_CLR),
                    (CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK));
}

void LIN_HLD_clearGlobalInterrupt1(uint32_t baseAddr)
{
    HW_WR_REG32_RAW((baseAddr + CSL_LIN_LIN_GLB_INT_CLR),
                    (CSL_LIN_LIN_GLB_INT_FLG_INT1_FLG_MASK));
}

uint32_t LIN_HLD_getIntVect0ffset(uint32_t baseAddr)
{
    /* Read and return the flag register */
    return( HW_RD_REG32_RAW(baseAddr + CSL_LIN_SCIINTVECT0) &
            CSL_LIN_SCIINTVECT0_INTVECT0_MASK);
}

uint32_t LIN_HLD_getIntVect1ffset(uint32_t baseAddr)
{
    /* Read and return the flag register */
    return( HW_RD_REG32_RAW(baseAddr + CSL_LIN_SCIINTVECT1) &
            CSL_LIN_SCIINTVECT1_INTVECT1_MASK);
}

bool LIN_HLD_isTxBufferEmpty(uint32_t baseAddr)
{
    bool retval = false;
    if( (HW_RD_REG32_RAW(baseAddr + CSL_LIN_SCIFLR) &
        CSL_LIN_SCIFLR_TXEMPTY_MASK) == CSL_LIN_SCIFLR_TXEMPTY_MASK)
    {
        retval = true;
    }

    return retval;
}

bool LIN_HLD_isRxIdMatch(uint32_t baseAddr)
{
    bool retval = false;
    if( (HW_RD_REG32_RAW(baseAddr + CSL_LIN_SCIFLR) &
        CSL_LIN_SCIFLR_IDRXFLAG_MASK) == CSL_LIN_SCIFLR_IDRXFLAG_MASK)
    {
        retval = true;
    }

    return retval;
}

uint32_t LIN_HLD_getInterruptStatusSCIFLR(uint32_t baseAddr, uint32_t mask)
{
    return (HW_RD_REG32_RAW(baseAddr + CSL_LIN_SCIFLR) & mask);
}

void LIN_HLD_clearInterruptStatusSCIFLR(uint32_t baseAddr, uint32_t mask)
{
    HW_WR_REG32_RAW((baseAddr + CSL_LIN_SCIFLR), mask);
}

uint8_t LIN_HLD_gen_ParityID(uint8_t identifier)
{
    uint8_t p0, p1, parityIdentifier;

    /* Calculate parity bits and generate updated identifier */
    p0 = (  (identifier & LIN_ID0_BIT_MASK) ^
            ((identifier & LIN_ID1_BIT_MASK) >> 1U) ^
            ((identifier & LIN_ID2_BIT_MASK) >> 2U) ^
            ((identifier & LIN_ID4_BIT_MASK) >> 4U));

    p1 = !( ((identifier & LIN_ID1_BIT_MASK) >> 1U) ^
            ((identifier & LIN_ID3_BIT_MASK) >> 3U) ^
            ((identifier & LIN_ID4_BIT_MASK) >> 4U) ^
            ((identifier & LIN_ID5_BIT_MASK) >> 5U));

    parityIdentifier = identifier | ((p0 << 6U) | (p1 << 7U));

    return(parityIdentifier);
}
