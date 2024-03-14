/**
 * @file  csl_cpdma.c
 *
 * @brief
 *  C implementation file for Ethernet CPDMA module CSL.
 *
 *  Contains the different control command and status query functions definations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2020, Texas Instruments, Inc.
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

#include <csl_cpdma.h>

void CSL_CPSW_getCpdmaTxVersionInfo (CSL_CpdmaRegs           *hCpdmaRegs,
                                     CSL_CPSW_CPDMA_VERSION  *pVersionInfo)
{
    Uint32 value = CSL_REG32_RD(hCpdmaRegs->TX_IDVER);

    pVersionInfo->minorVer  =   CSL_FEXT (value, CPDMA_TX_IDVER_TX_MINOR_VER);
    pVersionInfo->majorVer  =   CSL_FEXT (value, CPDMA_TX_IDVER_TX_MAJOR_VER);
    pVersionInfo->rtlVer    =   0U;
    pVersionInfo->id        =   CSL_FEXT (value, CPDMA_TX_IDVER_TX_IDENT);

    return;
}


void CSL_CPSW_getCpdmaRxVersionInfo (CSL_CpdmaRegs           *hCpdmaRegs,
                                     CSL_CPSW_CPDMA_VERSION  *pVersionInfo)
{
    Uint32 value = CSL_REG32_RD(hCpdmaRegs->RX_IDVER);

    pVersionInfo->minorVer  =   CSL_FEXT (value, CPDMA_RX_IDVER_RX_MINOR_VER);
    pVersionInfo->majorVer  =   CSL_FEXT (value, CPDMA_RX_IDVER_RX_MAJOR_VER);
    pVersionInfo->rtlVer    =   0U;
    pVersionInfo->id        =   CSL_FEXT (value, CPDMA_RX_IDVER_RX_IDENT);

    return;
}

void CSL_CPSW_configCpdma
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    CSL_CPSW_CPDMA_CONFIG   *pConfig
)
{
    uint32_t  value = 0U;

    CSL_FINS (value,  CPDMA_DMACONTROL_TX_PTYPE, pConfig->txPtype);
    CSL_FINS (value,  CPDMA_DMACONTROL_RX_OWNERSHIP, pConfig->rxOnwBit);
    CSL_FINS (value,  CPDMA_DMACONTROL_RX_OFFLEN_BLOCK, pConfig->rxOffLenBlockEn);
    CSL_FINS (value,  CPDMA_DMACONTROL_CMD_IDLE, pConfig->idleCmd);
    CSL_FINS (value,  CPDMA_DMACONTROL_RX_CEF, pConfig->rxCEFEn);
    CSL_FINS (value,  CPDMA_DMACONTROL_RX_VLAN_ENCAP, pConfig->rxVLANEn);
    CSL_FINS (value,  CPDMA_DMACONTROL_RX_TS_ENCAP, pConfig->rxTSEn);
    CSL_FINS (value,  CPDMA_DMACONTROL_TX_RLIM, pConfig->txRlimType);
    hCpdmaRegs->DMACONTROL = value;
}

void CSL_CPSW_getCpdmaStatus
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    CSL_CPSW_CPDMA_STATUS   *pStatusInfo
)
{
    Uint32 value = CSL_REG32_RD(hCpdmaRegs->DMASTATUS);

    pStatusInfo->rxErrCh    = CSL_FEXT (value, CPDMA_DMASTATUS_RX_ERR_CH);
    pStatusInfo->rxErrCode  = CSL_FEXT (value, CPDMA_DMASTATUS_RX_HOST_ERR_CODE);
    pStatusInfo->txErrCh    = CSL_FEXT (value, CPDMA_DMASTATUS_TX_ERR_CH);
    pStatusInfo->txErrCode  = CSL_FEXT (value, CPDMA_DMASTATUS_TX_HOST_ERR_CODE);
    pStatusInfo->idle       = CSL_FEXT (value, CPDMA_DMASTATUS_IDLE);
}

Uint32 CSL_CPSW_isCpdmaResetDone
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    Uint32 ret_val = 0U;
    if (CSL_FEXT (hCpdmaRegs->SOFT_RESET, CPDMA_SOFT_RESET_SOFT_RESET) == 0)
    {
        ret_val = (Uint32)TRUE;
    }
    else
    {
        ret_val = (Uint32)FALSE;
    }
    return ret_val;
}

void CSL_CPSW_resetCpdma
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->SOFT_RESET, CPDMA_SOFT_RESET_SOFT_RESET, (uint32_t)1U);

    return;
}

Uint32 CSL_CPSW_isCpdmaTxTeardownReady
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (CSL_FEXT (hCpdmaRegs->TX_TEARDOWN, CPDMA_TX_TEARDOWN_TX_TDN_RDY));
}

void CSL_CPSW_cpdmaTxTeardown
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
)
{
    CSL_FINS (hCpdmaRegs->TX_TEARDOWN, CPDMA_TX_TEARDOWN_TX_TDN_CH, channel);
}

Uint32 CSL_CPSW_isCpdmaRxTeardownReady
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (CSL_FEXT (hCpdmaRegs->RX_TEARDOWN, CPDMA_RX_TEARDOWN_RX_TDN_RDY));
}

void CSL_CPSW_cpdmaRxTeardown
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
)
{
    CSL_FINS (hCpdmaRegs->RX_TEARDOWN, CPDMA_RX_TEARDOWN_RX_TDN_CH, channel);
}

void CSL_CPSW_enableCpdmaTxInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
)
{
    CSL_FINSR (hCpdmaRegs->INTERRUPT.TX_INTMASK_SET, channel, channel, (uint32_t)1U);
}

void CSL_CPSW_enableCpdmaRxInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
)
{
    CSL_FINSR (hCpdmaRegs->INTERRUPT.RX_INTMASK_SET, channel, channel, (uint32_t)1U);
}

void CSL_CPSW_enableCpdmaDmaInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 bitMask
)
{
    hCpdmaRegs->INTERRUPT.DMA_INTMASK_SET = bitMask;
}

void CSL_CPSW_enableCpdmaStatsInt
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->INTERRUPT.DMA_INTMASK_SET, CPDMA_DMA_INTMASK_SET_STAT_INT_MASK, (uint32_t)1U);
}

void CSL_CPSW_enableCpdmaHostErrInt
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->INTERRUPT.DMA_INTMASK_SET, CPDMA_DMA_INTMASK_SET_HOST_ERR_INT_MASK, (uint32_t)1U);
}

void CSL_CPSW_disableCpdmaTxInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
)
{
    CSL_FINSR (hCpdmaRegs->INTERRUPT.TX_INTMASK_CLEAR, channel, channel, (uint32_t)1U);
}

void CSL_CPSW_disableCpdmaRxInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
)
{
    CSL_FINSR (hCpdmaRegs->INTERRUPT.RX_INTMASK_CLEAR, channel, channel, (uint32_t)1U);
}

void CSL_CPSW_disableCpdmaDmaInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 bitMask
)
{
    hCpdmaRegs->INTERRUPT.DMA_INTMASK_CLEAR = bitMask;
}

void CSL_CPSW_disableCpdmaStatsInt
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->INTERRUPT.DMA_INTMASK_CLEAR, CPDMA_DMA_INTMASK_CLEAR_STAT_INT_MASK, (uint32_t)1U);
}

void CSL_CPSW_disableCpdmaHostErrInt
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->INTERRUPT.DMA_INTMASK_CLEAR, CPDMA_DMA_INTMASK_CLEAR_HOST_ERR_INT_MASK, (uint32_t)1U);
}

void CSL_CPSW_enableCpdmaTx
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->TX_CONTROL, CPDMA_TX_CONTROL_TX_EN, (uint32_t)1U);
}

void CSL_CPSW_enableCpdmaRx
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->RX_CONTROL, CPDMA_RX_CONTROL_RX_EN, (uint32_t)1U);
}

void CSL_CPSW_enableCpdmaThostTsEncap
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->DMACONTROL, CPDMA_DMACONTROL_RX_TS_ENCAP, (uint32_t)1U);
}

#if ENET_CFG_IS_ON(CPDMA_CH_OVERRIDE)
void CSL_CPSW_enableCpdmaChOverride
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->DMACONTROL, CPDMA_DMACONTROL_TH_CH_OVERRIDE, (uint32_t)1U);
}
#endif

void CSL_CPSW_setCpdmaTxHdrDescPtr
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   descHdr,
    Uint32                   channel
)
{
    hCpdmaRegs->SRAM.TX_HDP[channel] = descHdr;
}

void CSL_CPSW_setCpdmaNumFreeBuf
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   channel,
    Uint32                   nBuf
)
{
    hCpdmaRegs->INTERRUPT.RX_FREEBUFFER[channel] = nBuf;
}

void CSL_CPSW_enableCpdmaIdle
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->DMACONTROL, CPDMA_DMACONTROL_CMD_IDLE, (uint32_t)1U);
}

void CSL_CPSW_disableCpdmaCmdIdle
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    CSL_FINS (hCpdmaRegs->DMACONTROL, CPDMA_DMACONTROL_CMD_IDLE, (uint32_t)0);
}

Uint32 CSL_CPSW_isCpdmaIdle
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return CSL_FEXT (hCpdmaRegs->DMASTATUS, CPDMA_DMASTATUS_IDLE);
}

void CSL_CPSW_setCpdmaRxBufOffset
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   bufOff
)
{
    hCpdmaRegs->RX_BUFFER_OFFSET = bufOff;
}

Uint32 CSL_CPSW_getCpdmaTxIntStatRaw
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (CSL_FEXT(hCpdmaRegs->INTERRUPT.TX_INTSTAT_RAW, CPDMA_TX_INTSTAT_RAW_TX_PEND));
}

Uint32 CSL_CPSW_getCpdmaTxIntStatMasked
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (CSL_FEXT(hCpdmaRegs->INTERRUPT.TX_INTSTAT_MASKED, CPDMA_TX_INTSTAT_MASKED_TX_PEND));
}

Uint32 CSL_CPSW_getCpdmaRxIntStatRaw
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (CSL_FEXT(hCpdmaRegs->INTERRUPT.RX_INTSTAT_RAW, CPDMA_RX_INTSTAT_RAW_RX_PEND));
}

Uint32 CSL_CPSW_getCpdmaRxIntStatMasked
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (CSL_FEXT(hCpdmaRegs->INTERRUPT.RX_INTSTAT_MASKED, CPDMA_RX_INTSTAT_MASKED_RX_PEND));
}

Uint32 CSL_CPSW_getCpdmaRxThreshIntStatRaw
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (CSL_FEXT(hCpdmaRegs->INTERRUPT.RX_INTSTAT_RAW, CPDMA_RX_INTSTAT_RAW_RX_THRESH_PEND));
}

Uint32 CSL_CPSW_getCpdmaRxThreshIntStatMasked
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (CSL_FEXT(hCpdmaRegs->INTERRUPT.RX_INTSTAT_MASKED, CPDMA_RX_INTSTAT_MASKED_RX_THRESH_PEND));
}

Uint32 CSL_CPSW_getCpdmaDmaIntStatRaw
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (hCpdmaRegs->INTERRUPT.DMA_INTSTAT_RAW);
}

Uint32 CSL_CPSW_getCpdmaDmaIntStatMasked
(
    CSL_CpdmaRegs           *hCpdmaRegs
)
{
    return (hCpdmaRegs->INTERRUPT.RX_INTSTAT_MASKED);
}
