/**
 * @file  csl_cpts.c
 *
 * @brief
 *  C implementation file for Ethernet Time Synchronization module CSL.
 *
 *  Contains the different control command and status query functions definations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014-2019, Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include <stdint.h>
#include <csl_cpts.h>

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#define CSL_CPTS_NUM_TS_GENF    (1U)
#define CSL_CPTS_NUM_TS_ESTF    (1U)
#elif defined(SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)
#define CSL_CPTS_NUM_TS_GENF    (1U)
#define CSL_CPTS_NUM_TS_ESTF    (2U)
#else
#define CSL_CPTS_NUM_TS_GENF    (sizeof(pCptsRegs->TS_GENF) / sizeof(CSL_cptsRegs_TS_GENF))
#define CSL_CPTS_NUM_TS_ESTF    (sizeof(pCptsRegs->TS_ESTF) / sizeof(CSL_cptsRegs_TS_ESTF))
#endif

void CSL_CPTS_getCptsVersionInfo (
    const CSL_cptsRegs  *pCptsRegs,
    CSL_CPTS_VERSION*   pVersionInfo
)
{
    uint32_t value = CSL_REG32_RD( &pCptsRegs->IDVER_REG );

    pVersionInfo->minorVer  =   CSL_FEXT (value, CPTS_IDVER_REG_MINOR_VER);
    pVersionInfo->majorVer  =   CSL_FEXT (value, CPTS_IDVER_REG_MAJOR_VER);
    pVersionInfo->rtlVer    =   CSL_FEXT (value, CPTS_IDVER_REG_RTL_VER);
    pVersionInfo->id        =   CSL_FEXT (value, CPTS_IDVER_REG_TX_IDENT);
    return;
}

uint32_t CSL_CPTS_isCptsEnabled (
    const CSL_cptsRegs  *pCptsRegs
)
{
    return CSL_REG32_FEXT (&pCptsRegs->CONTROL_REG, CPTS_CONTROL_REG_CPTS_EN);
}

void CSL_CPTS_enableCpts (
    CSL_cptsRegs    *pCptsRegs
)
{
    CSL_REG32_FINS (&pCptsRegs->CONTROL_REG, CPTS_CONTROL_REG_CPTS_EN, 1);
    return;
}

void CSL_CPTS_disableCpts (
    CSL_cptsRegs    *pCptsRegs
)
{
    CSL_REG32_FINS (&pCptsRegs->CONTROL_REG, CPTS_CONTROL_REG_CPTS_EN, 0);
    return;
}

void CSL_CPTS_getCntlReg (
    const CSL_cptsRegs  *pCptsRegs,
    CSL_CPTS_CONTROL*   pCntlCfg
)
{
    uint32_t value = CSL_REG32_RD( &pCptsRegs->CONTROL_REG );

    pCntlCfg->cptsEn         = CSL_FEXT (value,  CPTS_CONTROL_REG_CPTS_EN);
    pCntlCfg->intTest        = CSL_FEXT (value,  CPTS_CONTROL_REG_INT_TEST);
    pCntlCfg->tsCompPolarity = CSL_FEXT (value,  CPTS_CONTROL_REG_TS_COMP_POLARITY);
    pCntlCfg->tstampEn       = CSL_FEXT (value,  CPTS_CONTROL_REG_TSTAMP_EN);
    pCntlCfg->seqEn          = CSL_FEXT (value,  CPTS_CONTROL_REG_SEQUENCE_EN);
    pCntlCfg->ts64bMode      = CSL_FEXT (value,  CPTS_CONTROL_REG_MODE);
    pCntlCfg->tsCompToggle   = CSL_FEXT (value,  CPTS_CONTROL_REG_TS_COMP_TOG);
    pCntlCfg->tsHwPushEn[0]  = CSL_FEXT (value,  CPTS_CONTROL_REG_HW1_TS_PUSH_EN);
    pCntlCfg->tsHwPushEn[1]  = CSL_FEXT (value,  CPTS_CONTROL_REG_HW2_TS_PUSH_EN);
    pCntlCfg->tsHwPushEn[2]  = CSL_FEXT (value,  CPTS_CONTROL_REG_HW3_TS_PUSH_EN);
    pCntlCfg->tsHwPushEn[3]  = CSL_FEXT (value,  CPTS_CONTROL_REG_HW4_TS_PUSH_EN);
    pCntlCfg->tsHwPushEn[4]  = CSL_FEXT (value,  CPTS_CONTROL_REG_HW5_TS_PUSH_EN);
    pCntlCfg->tsHwPushEn[5]  = CSL_FEXT (value,  CPTS_CONTROL_REG_HW6_TS_PUSH_EN);
    pCntlCfg->tsHwPushEn[6]  = CSL_FEXT (value,  CPTS_CONTROL_REG_HW7_TS_PUSH_EN);
    pCntlCfg->tsHwPushEn[7]  = CSL_FEXT (value,  CPTS_CONTROL_REG_HW8_TS_PUSH_EN);
    pCntlCfg->tsOutputBitSel = (CSL_CPTS_TS_OUTPUT_BIT) CSL_FEXT (value,  CPTS_CONTROL_REG_TS_SYNC_SEL);
#ifdef CSL_CPTS_CONTROL_REG_TS_RX_NO_EVENT_MASK
    pCntlCfg->tsDisableRxEvents = CSL_FEXT (value,  CPTS_CONTROL_REG_TS_RX_NO_EVENT);
#else
    pCntlCfg->tsDisableRxEvents = 0U;
#endif
#ifdef CSL_CPTS_CONTROL_REG_TS_GENF_CLR_EN_MASK
    pCntlCfg->tsGenfClrEn = CSL_FEXT (value,  CPTS_CONTROL_REG_TS_GENF_CLR_EN);
#else
    pCntlCfg->tsGenfClrEn = 0U;
#endif

    return;
}

void CSL_CPTS_setCntlReg (
    CSL_cptsRegs        *pCptsRegs,
    const CSL_CPTS_CONTROL*   pCntlCfg
)
{
    uint32_t value = 0;

    CSL_FINS (value,  CPTS_CONTROL_REG_CPTS_EN, pCntlCfg->cptsEn);
    CSL_FINS (value,  CPTS_CONTROL_REG_INT_TEST, pCntlCfg->intTest);
    CSL_FINS (value,  CPTS_CONTROL_REG_TS_COMP_POLARITY, pCntlCfg->tsCompPolarity);
    CSL_FINS (value,  CPTS_CONTROL_REG_TSTAMP_EN, pCntlCfg->tstampEn);
    CSL_FINS (value,  CPTS_CONTROL_REG_SEQUENCE_EN, pCntlCfg->seqEn);
    CSL_FINS (value,  CPTS_CONTROL_REG_MODE, pCntlCfg->ts64bMode);
    CSL_FINS (value,  CPTS_CONTROL_REG_TS_COMP_TOG, pCntlCfg->tsCompToggle);
    CSL_FINS (value,  CPTS_CONTROL_REG_HW1_TS_PUSH_EN, pCntlCfg->tsHwPushEn[0]);
    CSL_FINS (value,  CPTS_CONTROL_REG_HW2_TS_PUSH_EN, pCntlCfg->tsHwPushEn[1]);
    CSL_FINS (value,  CPTS_CONTROL_REG_HW3_TS_PUSH_EN, pCntlCfg->tsHwPushEn[2]);
    CSL_FINS (value,  CPTS_CONTROL_REG_HW4_TS_PUSH_EN, pCntlCfg->tsHwPushEn[3]);
    CSL_FINS (value,  CPTS_CONTROL_REG_HW5_TS_PUSH_EN, pCntlCfg->tsHwPushEn[4]);
    CSL_FINS (value,  CPTS_CONTROL_REG_HW6_TS_PUSH_EN, pCntlCfg->tsHwPushEn[5]);
    CSL_FINS (value,  CPTS_CONTROL_REG_HW7_TS_PUSH_EN, pCntlCfg->tsHwPushEn[6]);
    CSL_FINS (value,  CPTS_CONTROL_REG_HW8_TS_PUSH_EN, pCntlCfg->tsHwPushEn[7]);
    CSL_FINS (value,  CPTS_CONTROL_REG_TS_SYNC_SEL, (uint32_t)pCntlCfg->tsOutputBitSel);
#ifdef CSL_CPTS_CONTROL_REG_TS_RX_NO_EVENT_MASK
    CSL_FINS (value,  CPTS_CONTROL_REG_TS_RX_NO_EVENT, pCntlCfg->tsDisableRxEvents);
#endif
#ifdef CSL_CPTS_CONTROL_REG_TS_GENF_CLR_EN_MASK
    CSL_FINS (value,  CPTS_CONTROL_REG_TS_GENF_CLR_EN, pCntlCfg->tsGenfClrEn);
#endif
    CSL_REG32_WR( &pCptsRegs->CONTROL_REG, value );
    return;
}

void CSL_CPTS_getRFTCLKSelectReg (
    const CSL_cptsRegs  *pCptsRegs,
    uint32_t*           pRefClockSelect
)
{
    *pRefClockSelect    =   CSL_REG32_FEXT (&pCptsRegs->RFTCLK_SEL_REG, CPTS_RFTCLK_SEL_REG_RFTCLK_SEL);
    return;
}

void CSL_CPTS_setRFTCLKSelectReg (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            refClockSetVal
)
{
    CSL_REG32_FINS (&pCptsRegs->RFTCLK_SEL_REG, CPTS_RFTCLK_SEL_REG_RFTCLK_SEL, refClockSetVal);
    return;
}

void CSL_CPTS_TSEventPush (
    CSL_cptsRegs        *pCptsRegs
)
{
    CSL_REG32_FINS (&pCptsRegs->TS_PUSH_REG, CPTS_TS_PUSH_REG_TS_PUSH, 1);
    return;
}

void CSL_CPTS_getTSLoadValReg (
    const CSL_cptsRegs  *pCptsRegs,
    uint32_t*           pTSLoadVal
)
{
    pTSLoadVal[0]     =   CSL_REG32_FEXT (&pCptsRegs->TS_LOAD_VAL_REG, CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL);
    pTSLoadVal[1]     =   CSL_REG32_FEXT (&pCptsRegs->TS_LOAD_HIGH_VAL_REG, CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL);
    return;
}

void CSL_CPTS_setTSLoadValReg (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsLoadValLo,
    uint32_t            tsLoadValHi
)
{
    CSL_REG32_FINS (&pCptsRegs->TS_LOAD_VAL_REG, CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL, tsLoadValLo);
    CSL_REG32_FINS (&pCptsRegs->TS_LOAD_HIGH_VAL_REG, CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL, tsLoadValHi);
    return;
}

void CSL_CPTS_setTSVal (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsValLo,
    uint32_t            tsValHi
)
{
    CSL_REG32_FINS (&pCptsRegs->TS_LOAD_VAL_REG, CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL, tsValLo);
    CSL_REG32_FINS (&pCptsRegs->TS_LOAD_HIGH_VAL_REG, CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL, tsValHi);
    CSL_REG32_FINS (&pCptsRegs->TS_LOAD_EN_REG, CPTS_TS_LOAD_EN_REG_TS_LOAD_EN, 1);
    return;
}

void CSL_CPTS_setTSCompVal (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsCompValLo,
    uint32_t            tsCompValHi,
    uint32_t            tsCompLen
)
{
    CSL_REG32_FINS (&pCptsRegs->TS_COMP_LEN_REG, CPTS_TS_COMP_LEN_REG_TS_COMP_LENGTH, 0);
    CSL_REG32_FINS (&pCptsRegs->TS_COMP_HIGH_VAL_REG, CPTS_TS_COMP_HIGH_VAL_REG_TS_COMP_HIGH_VAL, tsCompValHi);
    CSL_REG32_FINS (&pCptsRegs->TS_COMP_VAL_REG, CPTS_TS_COMP_VAL_REG_TS_COMP_VAL, tsCompValLo);
    CSL_REG32_FINS (&pCptsRegs->TS_COMP_LEN_REG, CPTS_TS_COMP_LEN_REG_TS_COMP_LENGTH, tsCompLen);
    return;
}

void CSL_CPTS_setTSCompNudge (
    CSL_cptsRegs        *pCptsRegs,
    int32_t             tsCompNudge
)
{
    CSL_REG32_FINS (&pCptsRegs->TS_COMP_NUDGE_REG, CPTS_TS_COMP_NUDGE_REG_NUDGE, tsCompNudge);
    return;
}

uint32_t CSL_CPTS_getTSAddVal (
    CSL_cptsRegs        *pCptsRegs
)
{
    uint32_t tsAddVal = 0U;

    if( CSL_REG32_FEXT (&pCptsRegs->CONTROL_REG, CPTS_CONTROL_REG_MODE) == 1U )
    {
        tsAddVal = CSL_REG32_FEXT (&pCptsRegs->TS_ADD_VAL_REG, CPTS_TS_ADD_VAL_REG_ADD_VAL);
    }
    else
    {
        tsAddVal = 0U;
    }
    return tsAddVal;
}

void CSL_CPTS_setTSAddVal (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsAddVal
)
{
    if( CSL_REG32_FEXT (&pCptsRegs->CONTROL_REG, CPTS_CONTROL_REG_MODE) == 1U )
    {
        CSL_REG32_FINS (&pCptsRegs->TS_ADD_VAL_REG, CPTS_TS_ADD_VAL_REG_ADD_VAL, tsAddVal);
    }
    else
    {
        CSL_REG32_FINS (&pCptsRegs->TS_ADD_VAL_REG, CPTS_TS_ADD_VAL_REG_ADD_VAL, 0);
    }
    return;
}

void CSL_CPTS_setTSNudge (
    CSL_cptsRegs        *pCptsRegs,
    int32_t             tsNudge
)
{
    if( CSL_REG32_FEXT (&pCptsRegs->CONTROL_REG, CPTS_CONTROL_REG_MODE) == 1U )
    {
        CSL_REG32_FINS (&pCptsRegs->TS_NUDGE_VAL_REG, CPTS_TS_NUDGE_VAL_REG_TS_NUDGE_VAL, tsNudge);
    }
    return;
}

void CSL_CPTS_getTSPpm (
    const CSL_cptsRegs  *pCptsRegs,
    uint32_t            tsPpmVal[2]
)
{
    tsPpmVal[0]     =   CSL_REG32_FEXT (&pCptsRegs->TS_PPM_LOW_VAL_REG, CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL);
    tsPpmVal[1]     =   CSL_REG32_FEXT (&pCptsRegs->TS_PPM_HIGH_VAL_REG, CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL);
    return;
}

void CSL_CPTS_setTSPpm (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsPpmValLo,
    uint32_t            tsPpmValHi,
    CSL_CPTS_TS_PPM_DIR tsPpmDir
)
{
    CSL_REG32_FINS (&pCptsRegs->CONTROL_REG, CPTS_CONTROL_REG_TS_PPM_DIR, tsPpmDir);
    /* HIGH_VAL must be written first */
    CSL_REG32_FINS (&pCptsRegs->TS_PPM_HIGH_VAL_REG, CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL, tsPpmValHi);
    CSL_REG32_FINS (&pCptsRegs->TS_PPM_LOW_VAL_REG, CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL, tsPpmValLo);
    return;
}

uint32_t CSL_CPTS_isRawInterruptStatusBitSet (
    const CSL_cptsRegs  *pCptsRegs
)
{
    return CSL_REG32_FEXT (&pCptsRegs->INTSTAT_RAW_REG, CPTS_INTSTAT_RAW_REG_TS_PEND_RAW);
}

uint32_t CSL_CPTS_isMaskedInterruptStatusBitSet (
    const CSL_cptsRegs  *pCptsRegs
)
{
    return CSL_REG32_FEXT (&pCptsRegs->INTSTAT_MASKED_REG, CPTS_INTSTAT_MASKED_REG_TS_PEND);
}

uint32_t CSL_CPTS_isInterruptEnabled (
    const CSL_cptsRegs  *pCptsRegs
)
{
    return CSL_REG32_FEXT (&pCptsRegs->INT_ENABLE_REG, CPTS_INT_ENABLE_REG_TS_PEND_EN);
}

void CSL_CPTS_enableInterrupt (
    CSL_cptsRegs    *pCptsRegs
)
{
    CSL_REG32_FINS (&pCptsRegs->INT_ENABLE_REG, CPTS_INT_ENABLE_REG_TS_PEND_EN, 1);
    return;
}

void CSL_CPTS_disableInterrupt (
    CSL_cptsRegs    *pCptsRegs
)
{
    CSL_REG32_FINS (&pCptsRegs->INT_ENABLE_REG, CPTS_INT_ENABLE_REG_TS_PEND_EN, 0);
    return;
}

void CSL_CPTS_popEvent (
    CSL_cptsRegs    *pCptsRegs
)
{
    CSL_REG32_FINS (&pCptsRegs->EVENT_POP_REG, CPTS_EVENT_POP_REG_EVENT_POP, 1);
    return;
}

void CSL_CPTS_getEventInfo (
    const CSL_cptsRegs  *pCptsRegs,
    CSL_CPTS_EVENTINFO* pEventInfo
)
{
    uint32_t value = CSL_REG32_RD( &pCptsRegs->EVENT_1_REG );

    pEventInfo->timeStamp   =   CSL_REG32_FEXT (&pCptsRegs->EVENT_0_REG, CPTS_EVENT_0_REG_TIME_STAMP);
    pEventInfo->seqId       =   CSL_FEXT (value, CPTS_EVENT_1_REG_SEQUENCE_ID);
    pEventInfo->msgType     =   CSL_FEXT (value, CPTS_EVENT_1_REG_MESSAGE_TYPE);
    pEventInfo->eventType   =   CSL_FEXT (value, CPTS_EVENT_1_REG_EVENT_TYPE);
    pEventInfo->portNo      =   CSL_FEXT (value, CPTS_EVENT_1_REG_PORT_NUMBER);
    pEventInfo->domain      =   CSL_REG32_FEXT (&pCptsRegs->EVENT_2_REG, CPTS_EVENT_2_REG_DOMAIN);
    pEventInfo->timeStampHi =   CSL_REG32_FEXT (&pCptsRegs->EVENT_3_REG, CPTS_EVENT_3_REG_TIME_STAMP);
    return;
}

int32_t CSL_CPTS_getGENFnLength (
    CSL_cptsRegs    *pCptsRegs,
    uint32_t        genfIndex,
    uint32_t*       pGenfLength
)
{
    int32_t retVal = -1;

    if( genfIndex < CSL_CPTS_NUM_TS_GENF )
    {
        *pGenfLength = CSL_REG32_FEXT (&pCptsRegs->TS_GENF[genfIndex].LENGTH_REG, CPTS_TS_GENF_LENGTH_REG_LENGTH);
        retVal = 0;
    }
    return retVal;
}

int32_t CSL_CPTS_setupGENFn(
    CSL_cptsRegs *pCptsRegs,
    uint32_t genfIndex,
    uint32_t length,
    uint64_t compare,
    uint32_t polarityInv,
    uint64_t ppmAdjust,
    CSL_CPTS_GENF_PPM_DIR ppmDir
)
{
    int32_t retVal = -1;

    if( genfIndex < CSL_CPTS_NUM_TS_GENF )
    {
        uint32_t regVal;

        /* TS_GENFn_Length must be zero during configuration */
        CSL_REG32_WR( &pCptsRegs->TS_GENF[genfIndex].LENGTH_REG, CSL_FMK(CPTS_TS_GENF_LENGTH_REG_LENGTH, (uint32_t)0U) );

        /* write Comp_low after Comp_high */
        regVal = (uint32_t)(compare >> 32);
        CSL_REG32_WR( &pCptsRegs->TS_GENF[genfIndex].COMP_HIGH_REG, CSL_FMK( CPTS_TS_GENF_COMP_HIGH_REG_COMP_HIGH, regVal) );
        regVal = (uint32_t)(compare & (uint64_t)0xFFFFFFFFUL);
        CSL_REG32_WR( &pCptsRegs->TS_GENF[genfIndex].COMP_LOW_REG,  CSL_FMK( CPTS_TS_GENF_COMP_LOW_REG_COMP_LOW, regVal) );

        /* write PPM_low after PPM_high */
        regVal = (uint32_t)(ppmAdjust >> 32);
        CSL_REG32_WR( &pCptsRegs->TS_GENF[genfIndex].PPM_HIGH_REG, CSL_FMK( CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH, regVal) );
        regVal = (uint32_t)(ppmAdjust & (uint64_t)0xFFFFFFFFUL);
        CSL_REG32_WR( &pCptsRegs->TS_GENF[genfIndex].PPM_LOW_REG,  CSL_FMK( CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW, regVal) );

        /* write TS_GENFn_Control */
        CSL_REG32_WR( &pCptsRegs->TS_GENF[genfIndex].CONTROL_REG,
            CSL_FMK( CPTS_TS_GENF_CONTROL_REG_PPM_DIR, ppmDir)  |
            CSL_FMK( CPTS_TS_GENF_CONTROL_REG_POLARITY_INV, polarityInv) );

        /* TS_GENFn_Length - write to enable operations */
        CSL_REG32_WR( &pCptsRegs->TS_GENF[genfIndex].LENGTH_REG, CSL_FMK(CPTS_TS_GENF_LENGTH_REG_LENGTH, length) );

        retVal = 0;
    }
    return retVal;
}

void CSL_CPTS_setGENFnPpm (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            genfIndex,
    uint32_t            ppmValLo,
    uint32_t            ppmValHi,
    CSL_CPTS_GENF_PPM_DIR ppmDir
)
{
    CSL_REG32_FINS (&pCptsRegs->TS_GENF[genfIndex].CONTROL_REG, CPTS_TS_GENF_CONTROL_REG_PPM_DIR, ppmDir);
    /* HIGH_VAL must be written first */
    CSL_REG32_FINS (&pCptsRegs->TS_GENF[genfIndex].PPM_HIGH_REG, CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH, ppmValHi);
    CSL_REG32_FINS (&pCptsRegs->TS_GENF[genfIndex].PPM_LOW_REG, CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW, ppmValLo);
    return;
}

int32_t CSL_CPTS_setGENFnNudge (
    CSL_cptsRegs    *pCptsRegs,
    uint32_t        genfIndex,
    int32_t         tsNudge
)
{
    int32_t retVal = -1;

    if( genfIndex < CSL_CPTS_NUM_TS_GENF )
    {
        uint32_t tsNudge2sCompliment = (uint32_t)tsNudge;
        CSL_REG32_WR( &pCptsRegs->TS_GENF[genfIndex].NUDGE_REG, CSL_FMK(CPTS_TS_GENF_NUDGE_REG_NUDGE, tsNudge2sCompliment) );
        retVal = 0;
    }
    return retVal;
}

int32_t CSL_CPTS_getESTFnLength (
    CSL_cptsRegs    *pCptsRegs,
    uint32_t        estfIndex,
    uint32_t*       pEstfLength
)
{
    int32_t retVal = -1;

    if( estfIndex < CSL_CPTS_NUM_TS_ESTF )
    {
        *pEstfLength = CSL_REG32_FEXT (&pCptsRegs->TS_ESTF[estfIndex].LENGTH_REG, CPTS_TS_ESTF_LENGTH_REG_LENGTH);
        retVal = 0;
    }
    return retVal;
}

int32_t CSL_CPTS_setupESTFn(
    CSL_cptsRegs *pCptsRegs,
    uint32_t estfIndex,
    uint32_t length,
    uint64_t compare,
    uint32_t polarityInv,
    uint64_t ppmAdjust,
    CSL_CPTS_ESTF_PPM_DIR ppmDir
)
{
    int32_t retVal = -1;

    if( estfIndex < CSL_CPTS_NUM_TS_ESTF )
    {
        uint32_t regVal;

        /* TS_ESTFn_Length must be zero during configuration */
        CSL_REG32_WR( &pCptsRegs->TS_ESTF[estfIndex].LENGTH_REG, CSL_FMK(CPTS_TS_ESTF_LENGTH_REG_LENGTH, (uint32_t)0U) );

        /* write Comp_low after Comp_high */
        regVal = (uint32_t)(compare >> 32);
        CSL_REG32_WR( &pCptsRegs->TS_ESTF[estfIndex].COMP_HIGH_REG, CSL_FMK( CPTS_TS_ESTF_COMP_HIGH_REG_COMP_HIGH, regVal) );
        regVal = (uint32_t)(compare & (uint64_t)0xFFFFFFFFUL);
        CSL_REG32_WR( &pCptsRegs->TS_ESTF[estfIndex].COMP_LOW_REG,  CSL_FMK( CPTS_TS_ESTF_COMP_LOW_REG_COMP_LOW, regVal) );

        /* write PPM_low after PPM_high */
        regVal = (uint32_t)(ppmAdjust >> 32);
        CSL_REG32_WR( &pCptsRegs->TS_ESTF[estfIndex].PPM_HIGH_REG, CSL_FMK( CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH, regVal) );
        regVal = (uint32_t)(ppmAdjust & (uint64_t)0xFFFFFFFFUL);
        CSL_REG32_WR( &pCptsRegs->TS_ESTF[estfIndex].PPM_LOW_REG,  CSL_FMK( CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW, regVal) );

        /* write TS_ESTFn_Control */
        CSL_REG32_WR( &pCptsRegs->TS_ESTF[estfIndex].CONTROL_REG,
            CSL_FMK( CPTS_TS_ESTF_CONTROL_REG_PPM_DIR, ppmDir)  |
            CSL_FMK( CPTS_TS_ESTF_CONTROL_REG_POLARITY_INV, polarityInv) );

        /* TS_ESTFn_Length - write to enable operations */
        CSL_REG32_WR( &pCptsRegs->TS_ESTF[estfIndex].LENGTH_REG, CSL_FMK(CPTS_TS_ESTF_LENGTH_REG_LENGTH, length) );

        retVal = 0;
    }
    return retVal;
}

int32_t CSL_CPTS_setESTFnNudge (
    CSL_cptsRegs    *pCptsRegs,
    uint32_t        estfIndex,
    int32_t         tsNudge
)
{
    int32_t retVal = -1;

    if( estfIndex < CSL_CPTS_NUM_TS_ESTF )
    {
        uint32_t tsNudge2sCompliment = (uint32_t)tsNudge;
        CSL_REG32_WR( &pCptsRegs->TS_ESTF[estfIndex].NUDGE_REG, CSL_FMK(CPTS_TS_ESTF_NUDGE_REG_NUDGE, tsNudge2sCompliment) );
        retVal = 0;
    }
    return retVal;
}

void CSL_CPTS_setESTFnPpm (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            estfIndex,
    uint32_t            ppmValLo,
    uint32_t            ppmValHi,
    CSL_CPTS_ESTF_PPM_DIR ppmDir
)
{
    CSL_REG32_FINS (&pCptsRegs->TS_ESTF[estfIndex].CONTROL_REG, CPTS_TS_ESTF_CONTROL_REG_PPM_DIR, ppmDir);
    /* HIGH_VAL must be written first */
    CSL_REG32_FINS (&pCptsRegs->TS_ESTF[estfIndex].PPM_HIGH_REG, CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH, ppmValHi);
    CSL_REG32_FINS (&pCptsRegs->TS_ESTF[estfIndex].PPM_LOW_REG, CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW, ppmValLo);
    return;
}

