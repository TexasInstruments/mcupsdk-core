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

/**
 *  \file     v1/fsi_tx.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of FSI_TX.
 *            This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/fsi.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Provides delay based on requested count. Needed while performing reset and
 * flush sequence, sufficient delay ensures reliability in operation.
 */
static void FSI_delayWait(uint32_t n);
static void FSI_delayWait(uint32_t n)
{
    volatile uint32_t temp = n;

    while (temp != 0U)
    {
        temp -= 1U;
    }
}

int32_t FSI_sendTxFlush(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_MASTER_CTRL);
    regVal |= (uint16_t)CSL_FSI_TX_CFG_TX_MASTER_CTRL_FLUSH_MASK |
              (FSI_CTRL_REG_KEY <<
               (uint16_t)CSL_FSI_TX_CFG_TX_MASTER_CTRL_KEY_SHIFT);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_MASTER_CTRL, regVal);

    return (retVal);
}

int32_t FSI_stopTxFlush(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_MASTER_CTRL);
    regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_MASTER_CTRL_FLUSH_MASK);
    regVal |= FSI_CTRL_REG_KEY << (uint16_t)CSL_FSI_TX_CFG_TX_MASTER_CTRL_KEY_SHIFT;
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_MASTER_CTRL, regVal);

    return (retVal);
}

int32_t FSI_selectTxPLLClock(uint32_t base, FSI_TxClkSel clkSel)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (clkSel > FSI_TX_CLK_SEL1)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        /* Set PLLCLK as source for clock divider */
        regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
        regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_PLLCLK_MASK);
        regVal |= (clkSel << CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_PLLCLK_SHIFT);
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal);
    }

    return (retVal);
}

int32_t FSI_enableTxClock(uint32_t base, uint16_t preScaleValue)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (preScaleValue > (uint16_t)0xFFU)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        /* Set prescalar value */
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL);
        regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_CLK_CTRL_PRESCALE_VAL_MASK)) |
                 (preScaleValue << CSL_FSI_TX_CFG_TX_CLK_CTRL_PRESCALE_VAL_SHIFT);
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL, regVal);

        /* Enable Tx clock */
        regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL);
        regVal |= CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_EN_MASK;
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL, regVal);
    }

    return (retVal);
}

int32_t FSI_disableTxClock(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL);
    regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_EN_MASK);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL, regVal);

    return (retVal);
}

int32_t FSI_setTxDataWidth(uint32_t base, FSI_DataWidth dataWidth)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (dataWidth > FSI_DATA_WIDTH_2_LANE)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
        regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__DATA_WIDTH_MASK)) |
                 (uint16_t)dataWidth;
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal);
    }

    return (retVal);
}

int32_t FSI_enableTxSPIMode(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
    regVal |= CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SPI_MODE_MASK;
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal);

    return (retVal);
}

int32_t FSI_disableTxSPIMode(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
    regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SPI_MODE_MASK);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal);

    return (retVal);
}

int32_t FSI_setTxStartMode(uint32_t base, FSI_TxStartMode txStartMode)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (txStartMode > FSI_TX_START_FRAME_CTRL_OR_UDATA_TAG)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
        regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__START_MODE_MASK)) |
                 ((uint16_t)txStartMode << CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__START_MODE_SHIFT);
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal);
    }

    return (retVal);
}

int32_t FSI_setTxPingTimeoutMode(uint32_t base, FSI_PingTimeoutMode pingTimeoutMode)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (pingTimeoutMode > FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
        if(pingTimeoutMode == FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME)
        {
            regVal |= CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__PING_TO_MODE_MASK;
        }
        else
        {
            regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__PING_TO_MODE_MASK);
        }
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal);
    }

    return (retVal);
}

int32_t FSI_setTxExtFrameTrigger(uint32_t base, uint16_t extInputNum)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (extInputNum >= FSI_TX_MAX_NUM_EXT_TRIGGERS)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_);
        regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__EXT_TRIG_SEL_MASK)) |
                 (extInputNum << CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__EXT_TRIG_SEL_SHIFT);
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_, regVal);
    }

    return (retVal);
}

int32_t FSI_enableTxCRCForceError(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_);
    regVal |= CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__FORCE_ERR_MASK;
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_, regVal);

    return (retVal);
}

int32_t FSI_disableTxCRCForceError(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_);
    regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__FORCE_ERR_MASK);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_, regVal);

    return (retVal);
}

int32_t FSI_setTxECCComputeWidth(uint32_t base, FSI_ECCComputeWidth eccComputeWidth)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (eccComputeWidth > FSI_16BIT_ECC_COMPUTE)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_);
        if(eccComputeWidth == FSI_16BIT_ECC_COMPUTE)
        {
            regVal |= CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__ECC_SEL_MASK;
        }
        else
        {
            regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__ECC_SEL_MASK);
        }
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_, regVal);
    }

    return (retVal);
}

int32_t FSI_setTxFrameType(uint32_t base, FSI_FrameType frameType)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (frameType > FSI_FRAME_TYPE_ERROR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_CTRL);
        regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_FRAME_CTRL_FRAME_TYPE_MASK)) |
                 (uint16_t)frameType;
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_CTRL, regVal);
    }

    return (retVal);
}

int32_t FSI_setTxSoftwareFrameSize(uint32_t base, uint16_t nWords)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if ((nWords == (uint16_t)0U)       ||
        (nWords > (uint16_t)(FSI_MAX_LEN_NWORDS_DATA + (uint16_t)1U)))
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_CTRL);
        regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_FRAME_CTRL_N_WORDS_MASK)) |
                 ((nWords - (uint16_t)1U) << CSL_FSI_TX_CFG_TX_FRAME_CTRL_N_WORDS_SHIFT);
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_CTRL, regVal);
    }

    return (retVal);
}

int32_t FSI_startTxTransmit(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_CTRL);
    regVal |= CSL_FSI_TX_CFG_TX_FRAME_CTRL_START_MASK;
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_CTRL, regVal);

    return (retVal);
}

int32_t FSI_setTxFrameTag(uint32_t base, FSI_FrameTag frameTag)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (frameTag > FSI_FRAME_TAG15)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA);
        regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_FRAME_TAG_MASK)) |
                 (uint16_t)frameTag;
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA, regVal);
    }

    return (retVal);
}

int32_t FSI_setTxUserDefinedData(uint32_t base, uint16_t userDefData)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (userDefData > FSI_MAX_VALUE_USERDATA)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA);
        regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_USER_DATA_MASK)) |
                 (userDefData << CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_USER_DATA_SHIFT);
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA, regVal);
    }

    return (retVal);
}

int32_t FSI_setTxBufferPtr(uint32_t base, uint16_t bufPtrOff)
{
    int32_t  retVal = CSL_PASS;

    /* Check the arguments */
    if (bufPtrOff > FSI_MAX_VALUE_BUF_PTR_OFF)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_BUF_PTR_LOAD, bufPtrOff);
    }

    return (retVal);
}

int32_t FSI_getTxBufferPtr(uint32_t base, uint16_t *pBufPtrLoc)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (pBufPtrLoc == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_BUF_PTR_STS);
        *pBufPtrLoc = regVal & CSL_FSI_TX_CFG_TX_BUF_PTR_STS_CURR_BUF_PTR_MASK;
    }

    return (retVal);
}

int32_t FSI_getTxWordCount(uint32_t base, uint16_t *pWordCnt)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (pWordCnt == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_BUF_PTR_STS);
        *pWordCnt = (regVal & CSL_FSI_TX_CFG_TX_BUF_PTR_STS_CURR_WORD_CNT_MASK) >>
                    CSL_FSI_TX_CFG_TX_BUF_PTR_STS_CURR_WORD_CNT_SHIFT;
    }

    return (retVal);
}

int32_t FSI_enableTxPingTimer(uint32_t     base,
                              uint32_t     refValue,
                              FSI_FrameTag pingFrameTag)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (pingFrameTag > FSI_FRAME_TAG15)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_PING_TAG, (uint16_t)pingFrameTag);
        HW_WR_REG32(base + CSL_FSI_TX_CFG_TX_PING_TO_REF, refValue);
        regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
        regVal |= CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__TIMER_EN_MASK;
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_, regVal);
    }

    return (retVal);
}

int32_t FSI_setTxPingTag(uint32_t base, FSI_FrameTag frameTag)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (frameTag > FSI_FRAME_TAG15)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_PING_TAG, (uint16_t)frameTag);
    }

    return (retVal);
}

int32_t FSI_disableTxPingTimer(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
    regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__TIMER_EN_MASK);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_, regVal);

    return (retVal);
}

int32_t FSI_enableTxExtPingTrigger(uint32_t base, uint16_t extTrigSel)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (extTrigSel >= FSI_TX_MAX_NUM_EXT_TRIGGERS)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        /* Select external input trigger */
        regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
        regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_SEL_MASK)) |
                 (extTrigSel << CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_SEL_SHIFT);
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_, regVal);

        /* Enable ping frame transmission through external trigger */
        regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
        regVal |= CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_EN_MASK;
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_, regVal);
    }

    return (retVal);
}

int32_t FSI_disableTxExtPingTrigger(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Select external input trigger */
    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
    regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_EN_MASK);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_, regVal);

    return (retVal);
}

int32_t FSI_getTxCurrentPingTimeoutCounter(uint32_t  base, uint32_t *pPingToCnt)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pPingToCnt == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pPingToCnt = HW_RD_REG32(base + CSL_FSI_TX_CFG_TX_PING_TO_CNT);
    }

    return (retVal);
}

int32_t FSI_lockTxCtrl(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal = (uint16_t)CSL_FSI_TX_CFG_TX_LOCK_CTRL_LOCK_MASK |
             (FSI_CTRL_REG_KEY << CSL_FSI_TX_CFG_TX_LOCK_CTRL_KEY_SHIFT);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_LOCK_CTRL, regVal);

    return (retVal);
}

int32_t FSI_getTxEventStatus(uint32_t base, uint16_t *pEvtFlags)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pEvtFlags == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pEvtFlags = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_EVT_STS) & FSI_TX_EVTMASK;
    }

    return (retVal);
}

int32_t FSI_forceTxEvents(uint32_t base, uint16_t evtFlags)
{
    int32_t  retVal = CSL_PASS;
    uint16_t flags = evtFlags & FSI_TX_EVTMASK;

    /* Check the arguments */
    if (flags != evtFlags)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_EVT_FRC, evtFlags);
    }

    return (retVal);
}

int32_t FSI_clearTxEvents(uint32_t base, uint16_t evtFlags)
{
    int32_t  retVal = CSL_PASS;
    uint16_t flags = evtFlags & FSI_TX_EVTMASK;

    /* Check the arguments */
    if (flags != evtFlags)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_EVT_CLR, evtFlags);
    }

    return (retVal);
}

int32_t FSI_enableTxUserCRC(uint32_t base, uint16_t userCRCValue)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
    regVal |= CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SW_CRC_MASK;
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal);

    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_USER_CRC, userCRCValue);

    return (retVal);
}

int32_t FSI_disableTxUserCRC(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
    regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SW_CRC_MASK);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal);

    return (retVal);
}

int32_t FSI_setTxECCdata(uint32_t base, uint32_t data)
{
    int32_t retVal = CSL_PASS;

    HW_WR_REG32(base + CSL_FSI_TX_CFG_TX_ECC_DATA, data);

    return (retVal);
}

int32_t FSI_getTxECCValue(uint32_t base, uint16_t *pEccVal)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pEccVal == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pEccVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_ECC_VAL) & CSL_FSI_TX_CFG_TX_ECC_VAL_ECC_VAL_MASK;
    }

    return (retVal);
}

int32_t FSI_enableTxInterrupt(uint32_t         base,
                              FSI_InterruptNum intNum,
                              uint16_t         intFlags)
{
    int32_t  retVal = CSL_PASS;
    uint16_t flags  = intFlags & FSI_TX_EVTMASK;
    uint16_t regVal;

    /* Check the arguments */
    if ((intNum > FSI_INT2)        ||
        (flags != intFlags))
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_INT_CTRL);
        if(intNum == FSI_INT1)
        {
            regVal |= flags;
        }
        else
        {
            regVal |= (flags << FSI_TX_INT2_CTRL_S);
        }
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_INT_CTRL, regVal);
    }

    return (retVal);
}

int32_t FSI_disableTxInterrupt(uint32_t         base,
                               FSI_InterruptNum intNum,
                               uint16_t         intFlags)
{
    int32_t  retVal = CSL_PASS;
    uint16_t flags  = intFlags & FSI_TX_EVTMASK;
    uint16_t regVal;

    /* Check the arguments */
    if ((intNum > FSI_INT2)        ||
        (flags != intFlags))
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_INT_CTRL);
        if(intNum == FSI_INT1)
        {
            regVal &= ~flags;
        }
        else
        {
            regVal &= (~flags << FSI_TX_INT2_CTRL_S) | (uint16_t)0xFFU;
        }
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_INT_CTRL, regVal);
    }

    return (retVal);
}

int32_t FSI_getTxBufferAddress(uint32_t base, uint32_t *pBufAddr)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pBufAddr == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pBufAddr = base + CSL_FSI_TX_CFG_TX_BUF_BASE(0U);
    }

    return (retVal);
}

int32_t FSI_resetTxModule(uint32_t base, FSI_TxSubmoduleInReset submodule)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    switch(submodule)
    {
        case FSI_TX_MASTER_CORE_RESET:
            regVal = (uint16_t)CSL_FSI_TX_CFG_TX_MASTER_CTRL_CORE_RST_MASK |
                     (FSI_CTRL_REG_KEY << CSL_FSI_TX_CFG_TX_MASTER_CTRL_KEY_SHIFT);
            HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_MASTER_CTRL, regVal);
            break;

        case FSI_TX_CLOCK_RESET:
            regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL);
            regVal |= CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_RST_MASK;
            HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL, regVal);
            break;

        case FSI_TX_PING_TIMEOUT_CNT_RESET:
            regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
            regVal |= CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__CNT_RST_MASK;
            HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_, regVal);
            break;

        default:
            retVal = CSL_EBADARGS;
            break;
    }

    return (retVal);
}

int32_t FSI_clearTxModuleReset(uint32_t base, FSI_TxSubmoduleInReset submodule)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    switch(submodule)
    {
        /*
         * Key value must be written into master control register and ensure
         * that FLUSH pattern is not sent while doing Tx core reset
         */
        case FSI_TX_MASTER_CORE_RESET:
            regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_MASTER_CTRL);
            regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_MASTER_CTRL_CORE_RST_MASK)) |
                     (FSI_CTRL_REG_KEY << CSL_FSI_TX_CFG_TX_MASTER_CTRL_KEY_SHIFT);
            HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_MASTER_CTRL, regVal);
            break;

        case FSI_TX_CLOCK_RESET:
            regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL);
            regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_RST_MASK);
            HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_CLK_CTRL, regVal);
            break;

        case FSI_TX_PING_TIMEOUT_CNT_RESET:
            regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_);
            regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__CNT_RST_MASK);
            HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_, regVal);
            break;

        default:
            retVal = CSL_EBADARGS;
            break;
    }

    return (retVal);
}

int32_t FSI_writeTxBuffer(uint32_t        base,
                          const uint16_t *pArray,
                          uint16_t        length,
                          uint16_t        bufOffset)
{
    int32_t  retVal = CSL_PASS;

    /* Check the arguments */
    if ((pArray == NULL_PTR)                                  ||
        (length > (FSI_MAX_VALUE_BUF_PTR_OFF + (uint16_t)1U)) ||
        (bufOffset > FSI_MAX_VALUE_BUF_PTR_OFF))
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        volatile uint16_t *pSrc16, *pDst16;
        uint16_t len = length;
        uint16_t offset = bufOffset;
        pSrc16 = (volatile uint16_t *)pArray;
        pDst16 = (volatile uint16_t *)(base + CSL_FSI_TX_CFG_TX_BUF_BASE(offset));
        while (len > 0U)
        {
            *pDst16 = *pSrc16;
            pSrc16++;
            pDst16++;
            offset++;
            /*
             * Check for wrap around in case more words to be written in
             * buffer when last write happened at maximum offset in Tx buffer
             */
            if (offset > FSI_MAX_VALUE_BUF_PTR_OFF)
            {
                offset = 0U;
                pDst16 = (volatile uint16_t *)(base + CSL_FSI_TX_CFG_TX_BUF_BASE(offset));
            }
            len--;
        }
    }

    return (retVal);
}

int32_t FSI_configTxDelayLine(uint32_t           base,
                              FSI_TxDelayTapType delayTapType,
                              uint16_t           tapValue)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (tapValue > FSI_TX_MAX_DELAY_LINE_VAL)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        switch(delayTapType)
        {
            case FSI_TX_DELAY_CLK:
                regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_DLYLINE_CTRL);
                regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXCLK_DLY_MASK)) |
                         (tapValue << CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXCLK_DLY_SHIFT);
                HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_DLYLINE_CTRL, regVal);
                break;

            case FSI_TX_DELAY_D0:
                regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_DLYLINE_CTRL);
                regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD0_DLY_MASK)) |
                         (tapValue << CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD0_DLY_SHIFT);
                HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_DLYLINE_CTRL, regVal);
                break;

            case FSI_TX_DELAY_D1:
                regVal = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_DLYLINE_CTRL);
                regVal = (regVal & (uint16_t)(~CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD1_DLY_MASK)) |
                         (tapValue << CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD1_DLY_SHIFT);
                HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_DLYLINE_CTRL, regVal);
                break;

            default:
                retVal = CSL_EBADARGS;
                break;
        }
    }

    return (retVal);
}

int32_t FSI_performTxInitialization(uint32_t base, uint16_t prescalar)
{
    int32_t retVal;

    /*
     * Sequence is :: Reset clock divider -> Wait -> Release reset ->
     * Set prescalar value and then enable clock
     */

    /* Select PLL clock as source for clock dividers */
    retVal = FSI_selectTxPLLClock(base, FSI_TX_CLK_SEL1);
    if (retVal == CSL_PASS)
    {
        retVal = FSI_resetTxModule(base, FSI_TX_CLOCK_RESET);
    }

    if (retVal == CSL_PASS)
    {
        FSI_delayWait(1U);
        retVal = FSI_clearTxModuleReset(base, FSI_TX_CLOCK_RESET);
    }

    if (retVal == CSL_PASS)
    {
        retVal = FSI_enableTxClock(base, prescalar);
    }

    if (retVal == CSL_PASS)
    {
        /*
         * Now reset Tx core and wait sufficient time before releasing it
         */
        retVal = FSI_resetTxModule(base, FSI_TX_MASTER_CORE_RESET);
    }

    if (retVal == CSL_PASS)
    {
        FSI_delayWait(prescalar);

        retVal = FSI_clearTxEvents(base, FSI_TX_EVTMASK);
    }

    if (retVal == CSL_PASS)
    {
        retVal = FSI_clearTxModuleReset(base, FSI_TX_MASTER_CORE_RESET);
    }

    return (retVal);
}

int32_t FSI_executeTxFlushSequence(uint32_t base, uint16_t prescalar)
{
    int32_t retVal;

    /* Check the arguments */
    if (prescalar > 0xFFU)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        (void)FSI_sendTxFlush(base);
        /*
         * Inserting delay as recommended in TRM, It ensures that the transmit
         * module core sees sufficient TXCLK cycles before stopping FLUSH pattern.
         */
        FSI_delayWait(2U * (uint32_t)prescalar);

        retVal = FSI_stopTxFlush(base);
    }

    return (retVal);
}

void FSI_enableTxDMAEvent(uint32_t base)
{
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_DMA_CTRL);
    regVal |= CSL_FSI_TX_CFG_TX_DMA_CTRL_DMA_EVT_EN_MASK;
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_DMA_CTRL, regVal);

    return;
}

void FSI_disableTxDMAEvent(uint32_t base)
{
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_DMA_CTRL);
    regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_DMA_CTRL_DMA_EVT_EN_MASK);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_DMA_CTRL, regVal);

    return;
}

int32_t FSI_enableAndConfigTxTDMMode(uint32_t base, uint8_t tdmInputSrc)
{
    uint16_t regVal;
    int32_t  retVal = CSL_PASS;

    /* Check the arguments */
    if (tdmInputSrc > FSI_TX_INPUT_TDM_PORT_FROM_RX_MODULE)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
        regVal |= CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__TDM_ENABLE_MASK;
        if (tdmInputSrc == FSI_TX_INPUT_TDM_PORT_FROM_RX_MODULE)
        {
            regVal |= CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_TDM_IN_MASK;
        }
        else
        {
            regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_TDM_IN_MASK);
        }
        HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal); 
    }

    return retVal;
}

void FSI_disableTxTDMMode(uint32_t base)
{
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_);
    regVal &= (uint16_t)(~CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__TDM_ENABLE_MASK);
    HW_WR_REG16(base + CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_, regVal);

    return;
}
