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
 *  \file     v0/fsi_rx.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of FSI_RX.
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

int32_t FSI_enableRxInternalLoopback(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_);
    regVal |= (uint16_t)CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__INT_LOOPBACK_MASK |
              (FSI_CTRL_REG_KEY << CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__KEY_SHIFT);
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_, regVal);

    return (retVal);
}

int32_t FSI_disableRxInternalLoopback(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_);
    regVal = (regVal & (uint16_t)(~CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__INT_LOOPBACK_MASK)) |
             (FSI_CTRL_REG_KEY << CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__KEY_SHIFT);
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_, regVal);

    return (retVal);
}

int32_t FSI_enableRxSPIPairing(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_);
    regVal |= (uint16_t)CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__SPI_PAIRING_MASK |
              (FSI_CTRL_REG_KEY << CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__KEY_SHIFT);
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_, regVal);

    return (retVal);
}

int32_t FSI_disableRxSPIPairing(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_);
    regVal = (regVal & (uint16_t)(~CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__SPI_PAIRING_MASK)) |
             (FSI_CTRL_REG_KEY << CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__KEY_SHIFT);
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_, regVal);

    return (retVal);
}

int32_t FSI_setRxDataWidth(uint32_t base, FSI_DataWidth dataWidth)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL);
    regVal = (regVal & (uint16_t)(~CSL_FSI_RX_CFG_RX_OPER_CTRL_DATA_WIDTH_MASK)) |
             (uint16_t)dataWidth;
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL, regVal);

    return (retVal);
}

int32_t FSI_enableRxSPIMode(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL);
    regVal |= CSL_FSI_RX_CFG_RX_OPER_CTRL_SPI_MODE_MASK;
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL, regVal);

    return (retVal);
}

int32_t FSI_disableRxSPIMode(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL);
    regVal &= (uint16_t)(~CSL_FSI_RX_CFG_RX_OPER_CTRL_SPI_MODE_MASK);
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL, regVal);

    return (retVal);
}

int32_t FSI_setRxSoftwareFrameSize(uint32_t base, uint16_t nWords)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if ((nWords == (uint16_t)0U)   ||
        (nWords > (uint16_t)(FSI_MAX_LEN_NWORDS_DATA + 1U)))
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL);
        regVal = (regVal & (uint16_t)(~CSL_FSI_RX_CFG_RX_OPER_CTRL_N_WORDS_MASK)) |
                 ((nWords - (uint16_t)1U) << CSL_FSI_RX_CFG_RX_OPER_CTRL_N_WORDS_SHIFT);
        HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL, regVal);
    }

    return (retVal);
}

int32_t FSI_setRxECCComputeWidth(uint32_t base, FSI_ECCComputeWidth eccComputeWidth)
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
        regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL);
        if (eccComputeWidth == FSI_16BIT_ECC_COMPUTE)
        {
            regVal |= (uint16_t)CSL_FSI_RX_CFG_RX_OPER_CTRL_ECC_SEL_MASK;
        }
        else
        {
            regVal &= (uint16_t)(~CSL_FSI_RX_CFG_RX_OPER_CTRL_ECC_SEL_MASK);
        }
        HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL, regVal);
    }

    return (retVal);
}

int32_t FSI_setRxPingTimeoutMode(uint32_t base, FSI_PingTimeoutMode pingTimeoutMode)
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
        regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL);
        if(pingTimeoutMode == FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME)
        {
            regVal |= CSL_FSI_RX_CFG_RX_OPER_CTRL_PING_WD_RST_MODE_MASK;
        }
        else
        {
            regVal &= (uint16_t)(~CSL_FSI_RX_CFG_RX_OPER_CTRL_PING_WD_RST_MODE_MASK);
        }
        HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_OPER_CTRL, regVal);
    }

    return (retVal);
}

int32_t FSI_getRxFrameType(uint32_t base, FSI_FrameType *pFrameType)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (pFrameType == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_INFO);
        *pFrameType =(FSI_FrameType)((uint32_t)regVal & CSL_FSI_RX_CFG_RX_FRAME_INFO_FRAME_TYPE_MASK);
    }

    return (retVal);
}

int32_t FSI_getRxFrameTag(uint32_t base, uint16_t *pFrameTag)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (pFrameTag == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA);
        *pFrameTag = (regVal & CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_FRAME_TAG_MASK) >>
                     CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_FRAME_TAG_SHIFT;
    }

    return (retVal);
}

int32_t FSI_getRxUserDefinedData(uint32_t base, uint16_t *pUserData)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (pUserData == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA);
        *pUserData = (regVal & CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_USER_DATA_MASK) >>
                     CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_USER_DATA_SHIFT;
    }

    return (retVal);
}

int32_t FSI_getRxEventStatus(uint32_t base, uint16_t *pEvtFlags)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pEvtFlags == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pEvtFlags = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_EVT_STS_ALT1_) & FSI_RX_EVTMASK;
    }

    return (retVal);
}

int32_t FSI_forceRxEvents(uint32_t base, uint16_t evtFlags)
{
    int32_t  retVal = CSL_PASS;
    uint16_t flags  = evtFlags & FSI_RX_EVTMASK;

    /* Check the arguments */
    if (flags != evtFlags)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1_, flags);
    }

    return (retVal);
}

int32_t FSI_clearRxEvents(uint32_t base, uint16_t evtFlags)
{
    int32_t  retVal = CSL_PASS;
    uint16_t flags  = evtFlags & FSI_RX_EVTMASK;

    /* Check the arguments */
    if (flags != evtFlags)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1_, evtFlags & FSI_RX_EVTMASK);
    }

    return (retVal);
}

int32_t FSI_getRxReceivedCRC(uint32_t base, uint16_t *pCrcVal)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pCrcVal == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pCrcVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_CRC_INFO) &
                   CSL_FSI_RX_CFG_RX_CRC_INFO_RX_CRC_MASK;
    }

    return (retVal);
}

int32_t FSI_getRxComputedCRC(uint32_t base, uint16_t *pCrcVal)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (pCrcVal == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {

        regVal   = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_CRC_INFO);
        *pCrcVal = (regVal & CSL_FSI_RX_CFG_RX_CRC_INFO_CALC_CRC_MASK) >>
                   CSL_FSI_RX_CFG_RX_CRC_INFO_CALC_CRC_SHIFT;
    }

    return (retVal);
}

int32_t FSI_setRxBufferPtr(uint32_t base, uint16_t bufPtrOff)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (bufPtrOff > FSI_MAX_VALUE_BUF_PTR_OFF)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_BUF_PTR_LOAD, bufPtrOff);
    }

    return (retVal);
}

int32_t FSI_getRxBufferPtr(uint32_t base, uint16_t *pBufPtrLoc)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pBufPtrLoc == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pBufPtrLoc = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_BUF_PTR_STS) &
                      CSL_FSI_RX_CFG_RX_BUF_PTR_STS_CURR_BUF_PTR_MASK;
    }

    return (retVal);
}

int32_t FSI_getRxWordCount(uint32_t base, uint16_t *pWordCnt)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pWordCnt == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pWordCnt = (HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_BUF_PTR_STS) &
                     CSL_FSI_RX_CFG_RX_BUF_PTR_STS_CURR_WORD_CNT_MASK) >>
                     CSL_FSI_RX_CFG_RX_BUF_PTR_STS_CURR_WORD_CNT_SHIFT;
    }

    return (retVal);
}

int32_t FSI_enableRxFrameWatchdog(uint32_t base, uint32_t wdRef)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    HW_WR_REG32(base + CSL_FSI_RX_CFG_RX_FRAME_WD_REF, wdRef);
    regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL);
    regVal |= CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_EN_MASK;
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL, regVal);

    return (retVal);
}

int32_t FSI_disableRxFrameWatchdog(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL);
    regVal &= (uint16_t)(~CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_EN_MASK);
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL, regVal);

    return (retVal);
}

int32_t FSI_getRxFrameWatchdogCounter(uint32_t base, uint32_t *pWdCnt)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pWdCnt == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pWdCnt = HW_RD_REG32(base + CSL_FSI_RX_CFG_RX_FRAME_WD_CNT);
    }

    return (retVal);
}

int32_t FSI_enableRxPingWatchdog(uint32_t base, uint32_t wdRef)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    HW_WR_REG32(base + CSL_FSI_RX_CFG_RX_PING_WD_REF, wdRef);
    regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_PING_WD_CTRL);
    regVal |= CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_EN_MASK;
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_PING_WD_CTRL, regVal);

    return (retVal);
}

int32_t FSI_disableRxPingWatchdog(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_PING_WD_CTRL);
    regVal &= (uint16_t)(~CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_EN_MASK);
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_PING_WD_CTRL, regVal);

    return (retVal);
}

int32_t FSI_getRxPingWatchdogCounter(uint32_t base, uint32_t *pWdCnt)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pWdCnt == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pWdCnt = HW_RD_REG32(base + CSL_FSI_RX_CFG_RX_PING_WD_CNT);
    }

    return (retVal);
}

int32_t FSI_getRxPingTag(uint32_t base, uint16_t *pPingTag)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (pPingTag == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        regVal    = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_PING_TAG);
        *pPingTag = (regVal & CSL_FSI_RX_CFG_RX_PING_TAG_PING_TAG_MASK) >>
                    CSL_FSI_RX_CFG_RX_PING_TAG_PING_TAG_SHIFT;
    }

    return (retVal);
}

int32_t FSI_lockRxCtrl(uint32_t base)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    regVal  = (uint16_t)CSL_FSI_RX_CFG_RX_LOCK_CTRL_LOCK_MASK |
              (FSI_CTRL_REG_KEY << CSL_FSI_RX_CFG_RX_LOCK_CTRL_KEY_SHIFT);
    HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_LOCK_CTRL, regVal);

    return (retVal);
}

int32_t FSI_setRxECCData(uint32_t base, uint32_t rxECCdata)
{
    int32_t retVal = CSL_PASS;

    HW_WR_REG32(base + CSL_FSI_RX_CFG_RX_ECC_DATA, rxECCdata);

    return (retVal);
}

int32_t FSI_setRxReceivedECCValue(uint32_t base, uint16_t rxECCvalue)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (rxECCvalue > FSI_MAX_VALUE_USERDATA)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        /* ECC value can be passed as 8 bit value in USERDATA field in a frame */
        HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_ECC_VAL, rxECCvalue);
    }

    return (retVal);
}

int32_t FSI_getRxECCCorrectedData(uint32_t base, uint32_t *pEccData)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pEccData == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pEccData = HW_RD_REG32(base + CSL_FSI_RX_CFG_RX_ECC_SEC_DATA);
    }

    return (retVal);
}

int32_t FSI_getRxECCLog(uint32_t base, uint16_t *pEccLog)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pEccLog == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pEccLog = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_ECC_LOG) &
                   (CSL_FSI_RX_CFG_RX_ECC_LOG_SBE_MASK | CSL_FSI_RX_CFG_RX_ECC_LOG_MBE_MASK);
    }

    return (retVal);
}

int32_t FSI_enableRxInterrupt(uint32_t         base,
                              FSI_InterruptNum intNum,
                              uint16_t         intFlags)
{
    int32_t  retVal = CSL_PASS;
    uint16_t flags  = intFlags & FSI_RX_EVTMASK;
    uint16_t regVal;
    uint32_t baseAddr;

    /* Check the arguments */
    if ((intNum > FSI_INT2)        ||
        (flags != intFlags))
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        if(intNum == FSI_INT1)
        {
            baseAddr = base + CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1_;
        }
        else
        {
            baseAddr = base + CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1_;
        }
        regVal = HW_RD_REG16(baseAddr);
        regVal |= flags;
        HW_WR_REG16(baseAddr, regVal);
    }

    return (retVal);
}

int32_t FSI_disableRxInterrupt(uint32_t         base,
                               FSI_InterruptNum intNum,
                               uint16_t         intFlags)
{
    int32_t  retVal = CSL_PASS;
    uint16_t flags  = intFlags & FSI_RX_EVTMASK;
    uint16_t regVal;
    uint32_t baseAddr;

    /* Check the arguments */
    if ((intNum > FSI_INT2)        ||
        (flags != intFlags))
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        if(intNum == FSI_INT1)
        {
            baseAddr = base + CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1_;
        }
        else
        {
            baseAddr = base + CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1_;
        }
        regVal = HW_RD_REG16(baseAddr);
        regVal &= ~flags;
        HW_WR_REG16(baseAddr, regVal);
    }

    return (retVal);
}

int32_t FSI_getRxBufferAddress(uint32_t base, uint32_t *pAddr)
{
    int32_t retVal = CSL_PASS;

    /* Check the arguments */
    if (pAddr == NULL_PTR)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        *pAddr = (base + CSL_FSI_RX_CFG_RX_BUF_BASE(0U));
    }

    return (retVal);
}

int32_t FSI_resetRxModule(uint32_t base, FSI_RxSubmoduleInReset submodule)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    switch(submodule)
    {
        case FSI_RX_MASTER_CORE_RESET:
            regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_);
            regVal |= (uint16_t)CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__CORE_RST_MASK |
                      (FSI_CTRL_REG_KEY << CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__KEY_SHIFT);
            HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_, regVal);
            break;

        case FSI_RX_FRAME_WD_CNT_RESET:
            regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL);
            regVal |= CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST_MASK;
            HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL, regVal);
            break;

        case FSI_RX_PING_WD_CNT_RESET:
            regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_PING_WD_CTRL);
            regVal |= CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_RST_MASK;
            HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_PING_WD_CTRL, regVal);
            break;

        default:
            retVal = CSL_EBADARGS;
            break;
    }

    return (retVal);
}

int32_t FSI_clearRxModuleReset(uint32_t base, FSI_RxSubmoduleInReset submodule)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    switch(submodule)
    {
        case FSI_RX_MASTER_CORE_RESET:
            regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_);
            regVal = (regVal & (uint16_t)(~CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__CORE_RST_MASK)) |
                     (FSI_CTRL_REG_KEY << CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__KEY_SHIFT);
            HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_, regVal);
            break;

        case FSI_RX_FRAME_WD_CNT_RESET:
            regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL);
            regVal &= (uint16_t)(~CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST_MASK);
            HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL, regVal);
            break;

        case FSI_RX_PING_WD_CNT_RESET:
            regVal  = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_PING_WD_CTRL);
            regVal &= (uint16_t)(~CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_RST_MASK);
            HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_PING_WD_CTRL, regVal);
            break;

        default:
            retVal = CSL_EBADARGS;
            break;
    }

    return (retVal);
}

int32_t FSI_readRxBuffer(uint32_t  base,
                         uint16_t *pArray,
                         uint16_t  length,
                         uint16_t  bufOffset)
{
    int32_t  retVal = CSL_PASS;

    /* Check the arguments */
    if ((pArray == NULL_PTR)                                    ||
        (length > (FSI_MAX_VALUE_BUF_PTR_OFF + (uint16_t)1U))   ||
        (bufOffset > FSI_MAX_VALUE_BUF_PTR_OFF))
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        volatile uint16_t *pSrc16, *pDst16;
        uint16_t len = length;
        uint16_t offset = bufOffset;
        pSrc16 = (volatile uint16_t *)(base + CSL_FSI_RX_CFG_RX_BUF_BASE(offset));
        pDst16 = (volatile uint16_t *)pArray;
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
                pSrc16 = (volatile uint16_t *)(base + CSL_FSI_RX_CFG_RX_BUF_BASE(offset));
            }
            len--;
        }
    }

    return (retVal);
}

int32_t FSI_configRxDelayLine(uint32_t           base,
                              FSI_RxDelayTapType delayTapType,
                              uint16_t           tapValue)
{
    int32_t  retVal = CSL_PASS;
    uint16_t regVal;

    /* Check the arguments */
    if (tapValue > FSI_RX_MAX_DELAY_LINE_VAL)
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        switch(delayTapType)
        {
            case FSI_RX_DELAY_CLK:
                regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_DLYLINE_CTRL);
                regVal = (regVal & (uint16_t)(~CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXCLK_DLY_MASK)) |
                         (tapValue << CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXCLK_DLY_SHIFT);
                HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_DLYLINE_CTRL, regVal);
                break;

            case FSI_RX_DELAY_D0:
                regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_DLYLINE_CTRL);
                regVal = (regVal & (uint16_t)(~CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD0_DLY_MASK)) |
                         (tapValue << CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD0_DLY_SHIFT);
                HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_DLYLINE_CTRL, regVal);
                break;

            case FSI_RX_DELAY_D1:
                regVal = HW_RD_REG16(base + CSL_FSI_RX_CFG_RX_DLYLINE_CTRL);
                regVal = (regVal & (uint16_t)(~CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD1_DLY_MASK)) |
                         (tapValue << CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD1_DLY_SHIFT);
                HW_WR_REG16(base + CSL_FSI_RX_CFG_RX_DLYLINE_CTRL, regVal);
                break;

            default:
                retVal = CSL_EBADARGS;
                break;
        }
    }

    return (retVal);
}

int32_t FSI_performRxInitialization(uint32_t base)
{
    int32_t  retVal;

    retVal = FSI_resetRxModule(base, FSI_RX_MASTER_CORE_RESET);

    if (retVal == CSL_PASS)
    {
        retVal = FSI_clearRxEvents(base, FSI_RX_EVTMASK);
    }

    if (retVal == CSL_PASS)
    {
        /*
         * It is important to ensure that there is no active clocks on RXCLK before
         * releasing the reset. This can be accomplished by handshaking or by using
         * the chip level connectivity options to ensure that no receive clocks are
         * seen till the reset is released.
         * FSI Rx module will come out of reset only when the receive clock is sent
         * in or by sending a Flush pattern from FSI RX.
         */
        retVal = FSI_clearRxModuleReset(base, FSI_RX_MASTER_CORE_RESET);
    }

    return (retVal);
}
