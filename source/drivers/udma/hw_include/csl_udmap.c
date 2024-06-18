/**
 *  ============================================================================
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 * @file  csl_udmap.c
 *
 * @brief
 *  C implementation file for the UDMAP module CSL-FL.
 *
 *  Contains the different control command and status query functions definitions
 *
 *  \par
*/
#include <string.h>
#include <stdbool.h>
#include <drivers/udma/hw_include/csl_udmap.h>

/*=============================================================================
 *  Internal definitions and functions
 *===========================================================================*/

#define CSL_UDMAP_CHAN_PEER_REG_OFFSET_ENABLE   ((uint32_t) 8U)

#define CSL_UDMAP_FETCH_WORD_SIZE_16            (16U)
#define CSL_UDMAP_NUM_PEER_REGS                 (16U)
#define CSL_UDMAP_RXCRT_CHAN_PEER(CHAN,REG)     ((uint32_t)CSL_UDMAP_RXCRT_CHAN_PEER0(CHAN)+((uint32_t)(REG)*0x4U))
#define CSL_UDMAP_TXCRT_CHAN_PEER(CHAN,REG)     ((uint32_t)CSL_UDMAP_TXCRT_CHAN_PEER0(CHAN)+((uint32_t)(REG)*0x4U))

#define CSL_UDMAP_UNPAUSE_CHAN                  (0)
#define CSL_UDMAP_PAUSE_CHAN                    (1U)

static bool CSL_udmapIsChanEnabled( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir );
static int32_t CSL_udmapSetChanEnable( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, bool bEnable );
static int32_t CSL_udmapTeardownChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, bool bForce, bool bWait );
static int32_t CSL_udmapPauseChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, uint32_t pauseVal );
static int32_t CSL_udmapTriggerChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir );
static void CSL_udmapClearChanError( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir );
static int32_t CSL_udmapAccessChanPeerReg( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, uint32_t regIdx, uint32_t *pVal, CSL_UdmapChanDir chanDir, bool bRdAccess );
static bool bIsModuleRevAtLeast( const CSL_UdmapCfg *pCfg, uint32_t majorRev, uint32_t minorRev, uint32_t rtlRev );

static bool bIsModuleRevAtLeast( const CSL_UdmapCfg *pCfg, uint32_t majorRev, uint32_t minorRev, uint32_t rtlRev )
{
    bool bRetVal = (bool)false;
    uint32_t pid, encodedPid, encodedRevVals;

    pid = CSL_udmapGetRevision( pCfg );
    encodedPid      = (((pid & 0x0700U) << 3U)  | ((pid & 0x003FU) << 5U) | ((pid & 0xF800U) >> 11U));
    encodedRevVals  = (((majorRev & 0x0007U) << 11U) | ((minorRev & 0x003FU) << 5U) | ((rtlRev & 0x001FU) >> 0));
    if( encodedPid >= encodedRevVals )
    {
        bRetVal = (bool)true;
    }
    return bRetVal;
}

static bool CSL_udmapIsChanEnabled( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir )
{
    uint32_t regVal;

    if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
    {
        regVal = CSL_REG32_FEXT( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, UDMAP_TXCRT_CHAN_CTL_EN );
    }
    else
    {
        regVal = CSL_REG32_FEXT( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, UDMAP_RXCRT_CHAN_CTL_EN );
    }
    return ((regVal == 1U) ? (bool)true : (bool)false);
}

static int32_t CSL_udmapSetChanEnable( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, bool bEnable )
{
    if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
    {
        CSL_REG32_WR(&pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, CSL_FMK(UDMAP_TXCRT_CHAN_CTL_EN,(bEnable ? (uint32_t)1 : (uint32_t)0)) );
    }
    else
    {
        CSL_REG32_WR(&pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, CSL_FMK(UDMAP_RXCRT_CHAN_CTL_EN,(bEnable ? (uint32_t)1 : (uint32_t)0)) );
    }
    return 0;
}

static int32_t CSL_udmapTeardownChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, bool bForce, bool bWait )
{
    int32_t  retVal = 0;
    uint32_t regVal;

    /* Channel can be torn down only when it is enabled */
    if( CSL_udmapIsChanEnabled( pCfg, chanIdx, chanDir ) == (bool)true )
    {
        if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
        {
            regVal = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL );
            CSL_FINS( regVal, UDMAP_TXCRT_CHAN_CTL_TDOWN, (uint32_t)1U );
            CSL_FINS( regVal, UDMAP_TXCRT_CHAN_CTL_FTDOWN, (bForce==(bool)false) ? (uint32_t)0U : (uint32_t)1U  );
            CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, regVal );
        }
        else
        {
            regVal = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL );
            CSL_FINS( regVal, UDMAP_RXCRT_CHAN_CTL_TDOWN, (uint32_t)1U );
            CSL_FINS( regVal, UDMAP_RXCRT_CHAN_CTL_FTDOWN, (bForce==(bool)false) ? (uint32_t)0U : (uint32_t)1U );
            CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, regVal );
        }
        if( bWait == (bool)true )
        {
            /* The channel's enable bit is cleared once teardown is complete */
            while( CSL_udmapIsChanEnabled( pCfg, chanIdx, chanDir ) == (bool)true ) {};
        }
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

static int32_t CSL_udmapPauseChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, uint32_t pauseVal )
{
    int32_t     retVal = 0;

    /* Channel can be paused/un-paused only when it is enabled */
    if( CSL_udmapIsChanEnabled( pCfg, chanIdx, chanDir ) == (bool)true )
    {
        if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
        {
            CSL_REG32_FINS( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, UDMAP_TXCRT_CHAN_CTL_PAUSE, pauseVal );
        }
        else
        {
            CSL_REG32_FINS( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, UDMAP_RXCRT_CHAN_CTL_PAUSE, pauseVal );
        }
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

static int32_t CSL_udmapTriggerChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir )
{
    if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
    {
        CSL_REG32_WR(&pCfg->pTxChanRtRegs->CHAN[chanIdx].SWTRIG, CSL_FMK(UDMAP_TXCRT_CHAN_SWTRIG_TRIGGER, 1U));
    }
    else
    {
        CSL_REG32_WR(&pCfg->pRxChanRtRegs->CHAN[chanIdx].SWTRIG, CSL_FMK(UDMAP_RXCRT_CHAN_SWTRIG_TRIGGER, 1U));
    }
    return 0;
}

static void CSL_udmapClearChanError( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir )
{
    if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
    {
        CSL_REG32_FINS(&pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, UDMAP_TXCRT_CHAN_CTL_ERROR, 0U);
    }
    else
    {
        CSL_REG32_FINS(&pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, UDMAP_RXCRT_CHAN_CTL_ERROR, 0U);
    }
}

static int32_t CSL_udmapAccessChanPeerReg( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, uint32_t regIdx, uint32_t *pVal, CSL_UdmapChanDir chanDir, bool bRdAccess )
{
    int32_t     retVal;
    uint32_t    *pPeerReg;

    if( regIdx < CSL_UDMAP_NUM_PEER_REGS )
    {
        if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
        {
            uint8_t  *pPeerReg8 = (uint8_t *)pCfg->pTxChanRtRegs;
            uint32_t byteOffset = (uint32_t)CSL_UDMAP_TXCRT_CHAN_PEER0(chanIdx) + (regIdx * (uint32_t)0x4U);
            pPeerReg8 += byteOffset;
            pPeerReg = (uint32_t *)pPeerReg8;
        }
        else
        {
            uint8_t  *pPeerReg8 = (uint8_t *)pCfg->pRxChanRtRegs;
            uint32_t byteOffset = (uint32_t)CSL_UDMAP_RXCRT_CHAN_PEER0(chanIdx) + (regIdx * (uint32_t)0x4U);
            pPeerReg8 += byteOffset;
            pPeerReg = (uint32_t *)pPeerReg8;
        }
        if( bRdAccess == (bool)true )
        {
            *pVal = CSL_REG32_RD( pPeerReg );
        }
        else
        {
            CSL_REG32_WR( pPeerReg, *pVal );
        }
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

/*=============================================================================
 *  UDMAP API functions
 *===========================================================================*/
uint32_t CSL_udmapGetRevision( const CSL_UdmapCfg *pCfg )
{
    return CSL_REG32_RD( &pCfg->pGenCfgRegs->REVISION );
}

int32_t CSL_udmapGetRevisionInfo( const CSL_UdmapCfg *pCfg, CSL_UdmapRevision *pRev )
{
    uint32_t val;

    val = CSL_REG32_RD( &pCfg->pGenCfgRegs->REVISION );

    pRev->modId     = CSL_FEXT( val, UDMAP_GCFG_REVISION_MODID );
    pRev->revRtl    = CSL_FEXT( val, UDMAP_GCFG_REVISION_REVRTL );
    pRev->revMajor  = CSL_FEXT( val, UDMAP_GCFG_REVISION_REVMAJ );
    pRev->custom    = CSL_FEXT( val, UDMAP_GCFG_REVISION_CUSTOM );
    pRev->revMinor  = CSL_FEXT( val, UDMAP_GCFG_REVISION_REVMIN );

    return 0;
}

void CSL_udmapGetCfg( CSL_UdmapCfg *pCfg )
{
    uint32_t regVal;

    pCfg->cap0 = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP0 );
    pCfg->cap1 = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP1 );
    regVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP2 );
    pCfg->txChanCnt       = CSL_FEXT( regVal, UDMAP_GCFG_CAP2_TCHAN_CNT );
    pCfg->rxChanCnt       = CSL_FEXT( regVal, UDMAP_GCFG_CAP2_RCHAN_CNT );
    pCfg->txExtUtcChanCnt = CSL_FEXT( regVal, UDMAP_GCFG_CAP2_ECHAN_CNT );
    regVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP3 );
    pCfg->rxFlowCnt       = CSL_FEXT( regVal, UDMAP_GCFG_CAP3_RFLOW_CNT );
#ifdef CSL_UDMAP_GCFG_CAP3_HCHAN_CNT_MASK
    pCfg->txHighCapacityChanCnt = CSL_FEXT( regVal, UDMAP_GCFG_CAP3_HCHAN_CNT );
#else
    pCfg->txHighCapacityChanCnt = 0;
#endif
#ifdef CSL_UDMAP_GCFG_CAP3_UCHAN_CNT_MASK
    pCfg->txUltraHighCapacityChanCnt = CSL_FEXT( regVal, UDMAP_GCFG_CAP3_UCHAN_CNT );
#else
    pCfg->txUltraHighCapacityChanCnt = 0;
#endif
}

void CSL_udmapInitTxChanCfg( CSL_UdmapTxChanCfg *pTxChanCfg )
{
    /*-------------------------------------------------------------------------
     *  Start by initializing all structure members to 0
     *-----------------------------------------------------------------------*/
    memset( (void *)pTxChanCfg, 0, sizeof(CSL_UdmapTxChanCfg) );
    /*-------------------------------------------------------------------------
     *  Now initialize non-zero structure members
     *-----------------------------------------------------------------------*/
    pTxChanCfg->chanType        = CSL_UDMAP_CHAN_TYPE_REF_PKT_RING;
    pTxChanCfg->fetchWordSize   = CSL_UDMAP_FETCH_WORD_SIZE_16;
    pTxChanCfg->trEventNum      = CSL_UDMAP_NO_EVENT;
    pTxChanCfg->errEventNum     = CSL_UDMAP_NO_EVENT;
}

void CSL_udmapInitRxChanCfg( CSL_UdmapRxChanCfg *pRxChanCfg )
{
    /*-------------------------------------------------------------------------
     *  Start by initializing all structure members to 0
     *-----------------------------------------------------------------------*/
    memset( (void *)pRxChanCfg, 0, sizeof(CSL_UdmapRxChanCfg) );
    /*-------------------------------------------------------------------------
     *  Now initialize non-zero structure members
     *-----------------------------------------------------------------------*/
    pRxChanCfg->chanType            = CSL_UDMAP_CHAN_TYPE_REF_PKT_RING;
    pRxChanCfg->fetchWordSize       = CSL_UDMAP_FETCH_WORD_SIZE_16;
    pRxChanCfg->trEventNum          = CSL_UDMAP_NO_EVENT;
    pRxChanCfg->errEventNum         = CSL_UDMAP_NO_EVENT;
    pRxChanCfg->flowIdFwRangeCnt    = 0x00004000U;
}

void CSL_udmapInitRxFlowCfg( CSL_UdmapRxFlowCfg *pFlow )
{
    /*-------------------------------------------------------------------------
     *  Start by initializing all structure members to 0
     *-----------------------------------------------------------------------*/
    memset( (void *)pFlow, 0, sizeof(CSL_UdmapRxFlowCfg) );
    /*-------------------------------------------------------------------------
     *  Now initialize non-zero structure members
     *-----------------------------------------------------------------------*/
}

void CSL_udmapSetPerfCtrl( CSL_UdmapCfg *pCfg, uint32_t rxRetryTimeoutCnt )
{
    uint32_t regVal;

    regVal = CSL_FMK( UDMAP_GCFG_PERF_CTRL_TIMEOUT_CNT, rxRetryTimeoutCnt );
    CSL_REG32_WR( &pCfg->pGenCfgRegs->PERF_CTRL, regVal );
}

void CSL_udmapSetUtcCtrl( CSL_UdmapCfg *pCfg, uint32_t startingThreadNum )
{
    uint32_t regVal;

    regVal = CSL_FMK( UDMAP_GCFG_UTC_CTRL_UTC_CHAN_START, startingThreadNum );
    CSL_REG32_WR( &pCfg->pGenCfgRegs->UTC_CTRL, regVal );
}

int32_t CSL_udmapRxFlowCfg(CSL_UdmapCfg *pCfg, uint32_t flow, const CSL_UdmapRxFlowCfg *pFlow )
{
    uint32_t cfgThresh, i, queue0;

    cfgThresh = 0;
    for(i=0; i<CSL_UDMAP_RXFDQ_THRESH_CNT; i++)
    {
        if( (pFlow->fdqThresh[i].fEnable) != 0U )
        {
            cfgThresh |= ((uint32_t)1U<<i);
        }
    }

    queue0 = pFlow->fdq[0];
    if( (pFlow->fdqThresh[0].fEnable != 0U) && (pFlow->fdqThresh[0].queue != queue0) )
    {
        queue0 = pFlow->fdqThresh[0].queue;
    }

    CSL_REG32_WR(&pCfg->pRxFlowCfgRegs->FLOW[flow].RFA,
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFA_EINFO,          pFlow->einfoPresent) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFA_PSINFO,         pFlow->psInfoPresent) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFA_ERR_HANDLING,   pFlow->errorHandling) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFA_DESC_TYPE,      pFlow->descType) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFA_PS_LOC,         pFlow->psLocation) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFA_SOP_OFF,        pFlow->sopOffset) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFA_DEST_QNUM,      pFlow->defaultRxCQ) );

    CSL_REG32_WR(&pCfg->pRxFlowCfgRegs->FLOW[flow].RFB,
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFB_SRCTAG_HI,      ((uint32_t)pFlow->srcTag.hiVal)) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFB_SRCTAG_LO,      ((uint32_t)pFlow->srcTag.loVal)) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFB_DSTTAG_HI,      ((uint32_t)pFlow->dstTag.hiVal)) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFB_DSTTAG_LO,      ((uint32_t)pFlow->dstTag.loVal)) );

    CSL_REG32_WR(&pCfg->pRxFlowCfgRegs->FLOW[flow].RFC,
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFC_SRCTAG_HI_SEL,  pFlow->srcTag.hiSel) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFC_SRCTAG_LO_SEL,  pFlow->srcTag.loSel) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFC_DSTTAG_HI_SEL,  pFlow->dstTag.hiSel) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFC_DSTTAG_LO_SEL,  pFlow->dstTag.loSel) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFC_SIZE_THRESH_EN, cfgThresh) );

    CSL_REG32_WR(&pCfg->pRxFlowCfgRegs->FLOW[flow].RFD,
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFD_FDQ0_SZ0_QNUM,  queue0) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFD_FDQ1_QNUM,      pFlow->fdq[1]) );

    CSL_REG32_WR(&pCfg->pRxFlowCfgRegs->FLOW[flow].RFE,
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFE_FDQ2_QNUM,      pFlow->fdq[2]) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFE_FDQ3_QNUM,      pFlow->fdq[3]) );

    CSL_REG32_WR(&pCfg->pRxFlowCfgRegs->FLOW[flow].RFF,
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFF_SIZE_THRESH0,   pFlow->fdqThresh[0].pktSize) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFF_SIZE_THRESH1,   pFlow->fdqThresh[1].pktSize) );

    CSL_REG32_WR(&pCfg->pRxFlowCfgRegs->FLOW[flow].RFG,
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFG_SIZE_THRESH2,   pFlow->fdqThresh[2].pktSize) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFG_FDQ0_SZ1_QNUM,  pFlow->fdqThresh[1].queue) );

    CSL_REG32_WR(&pCfg->pRxFlowCfgRegs->FLOW[flow].RFH,
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFH_FDQ0_SZ2_QNUM,  pFlow->fdqThresh[2].queue) |
                    CSL_FMK(UDMAP_RXFCFG_FLOW_RFH_FDQ0_SZ3_QNUM,  pFlow->fdqThresh[3].queue) );

    return 0;
}

int32_t CSL_udmapRxChanSetTrEvent( CSL_UdmapCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum )
{
    CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].ROES[0],
                 CSL_FMK(UDMAP_RXCCFG_CHAN_ROES_EVT_NUM, trEventNum));
    return 0;
}

int32_t CSL_udmapRxChanSetBurstSize( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanBurstSize burstSize )
{
    int32_t retVal = -1;
#ifdef CSL_UDMAP_RXCCFG_CHAN_RCFG_BURST_SIZE_MASK
    if( (burstSize >= CSL_UDMAP_CHAN_BURST_SIZE_64_BYTES) && (burstSize <= CSL_UDMAP_CHAN_BURST_SIZE_256_BYTES) )
    {
        CSL_REG32_FINS( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCFG, UDMAP_RXCCFG_CHAN_RCFG_BURST_SIZE, burstSize );
        retVal = 0;
    }
#endif
    return retVal;
}

int32_t CSL_udmapRxChanCfg( CSL_UdmapCfg *pCfg, uint32_t chanIdx, const CSL_UdmapRxChanCfg *pRxChanCfg )
{
    uint32_t regVal;

    regVal = CSL_REG32_RD( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCFG );
    CSL_FINS( regVal, UDMAP_RXCCFG_CHAN_RCFG_PAUSE_ON_ERR,  pRxChanCfg->pauseOnError );
    CSL_FINS( regVal, UDMAP_RXCCFG_CHAN_RCFG_ATYPE,         pRxChanCfg->addrType );
    CSL_FINS( regVal, UDMAP_RXCCFG_CHAN_RCFG_CHAN_TYPE,     pRxChanCfg->chanType );
    CSL_FINS( regVal, UDMAP_RXCCFG_CHAN_RCFG_IGNORE_SHORT,  (pRxChanCfg->bIgnoreShortPkts) ? (uint32_t)1U : (uint32_t)0U );
    CSL_FINS( regVal, UDMAP_RXCCFG_CHAN_RCFG_IGNORE_LONG,   (pRxChanCfg->bIgnoreLongPkts) ? (uint32_t)1U : (uint32_t)0U );
    CSL_FINS( regVal, UDMAP_RXCCFG_CHAN_RCFG_FETCH_SIZE,    pRxChanCfg->fetchWordSize );
    CSL_REG32_WR( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCFG, regVal );

    CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].ROES[0],
            CSL_FMK(UDMAP_RXCCFG_CHAN_ROES_EVT_NUM, pRxChanCfg->trEventNum) );

    CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].REOES[0],
            CSL_FMK(UDMAP_RXCCFG_CHAN_REOES_EVT_NUM, pRxChanCfg->errEventNum ) );

    CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].RPRI_CTRL,
            CSL_FMK(UDMAP_RXCCFG_CHAN_RPRI_CTRL_PRIORITY,   pRxChanCfg->busPriority) |
            CSL_FMK(UDMAP_RXCCFG_CHAN_RPRI_CTRL_QOS,        pRxChanCfg->busQos)      |
            CSL_FMK(UDMAP_RXCCFG_CHAN_RPRI_CTRL_ORDERID,    pRxChanCfg->busOrderId ) );

    CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCQ,
            CSL_FMK( UDMAP_RXCCFG_CHAN_RCQ_RXCQ_QNUM, pRxChanCfg->rxTrCQ ) );

    CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].THREAD,
            CSL_FMK( UDMAP_RXCCFG_CHAN_THREAD_ID, pRxChanCfg->rxThread ) );

    CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].RST_SCHED,
                  CSL_FMK(UDMAP_RXCCFG_CHAN_RST_SCHED_PRIORITY, pRxChanCfg->dmaPriority) );

    CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].RFLOW_RNG,
            CSL_FMK(UDMAP_RXCCFG_CHAN_RFLOW_RNG_FLOWID_START, pRxChanCfg->flowIdFwRangeStart) |
            CSL_FMK(UDMAP_RXCCFG_CHAN_RFLOW_RNG_FLOWID_CNT, pRxChanCfg->flowIdFwRangeCnt  ));

    return 0;
}

int32_t CSL_udmapTxChanSetTrEvent( CSL_UdmapCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum )
{
    CSL_REG32_WR(&pCfg->pTxChanCfgRegs->CHAN[chanIdx].TOES[0],
                 CSL_FMK(UDMAP_TXCCFG_CHAN_TOES_EVT_NUM, trEventNum));
    return 0;
}

int32_t CSL_udmapTxChanSetBurstSize( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanBurstSize burstSize )
{
    int32_t retVal = -1;
#ifdef CSL_UDMAP_TXCCFG_CHAN_TCFG_BURST_SIZE_MASK
    if( (burstSize >= CSL_UDMAP_CHAN_BURST_SIZE_64_BYTES) && (burstSize <= CSL_UDMAP_CHAN_BURST_SIZE_256_BYTES) )
    {
        CSL_REG32_FINS( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCFG, UDMAP_TXCCFG_CHAN_TCFG_BURST_SIZE, burstSize );
        retVal = 0;
    }
#endif
    return retVal;
}

int32_t CSL_udmapTxChanCfg( CSL_UdmapCfg *pCfg, uint32_t chanIdx, const CSL_UdmapTxChanCfg *pTxChanCfg )
{
    uint32_t regVal;

    regVal = CSL_REG32_RD( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCFG );
    CSL_FINS( regVal, UDMAP_TXCCFG_CHAN_TCFG_PAUSE_ON_ERR,    pTxChanCfg->pauseOnError );
    CSL_FINS( regVal, UDMAP_TXCCFG_CHAN_TCFG_FILT_EINFO,      pTxChanCfg->filterEinfo );
    CSL_FINS( regVal, UDMAP_TXCCFG_CHAN_TCFG_FILT_PSWORDS,    pTxChanCfg->filterPsWords );
    CSL_FINS( regVal, UDMAP_TXCCFG_CHAN_TCFG_ATYPE,           pTxChanCfg->addrType );
    CSL_FINS( regVal, UDMAP_TXCCFG_CHAN_TCFG_NOTDPKT,         ((pTxChanCfg->bNoTeardownCompletePkt) ? (uint32_t)1U : (uint32_t)0U) );
    CSL_FINS( regVal, UDMAP_TXCCFG_CHAN_TCFG_CHAN_TYPE,       pTxChanCfg->chanType );
    CSL_FINS( regVal, UDMAP_TXCCFG_CHAN_TCFG_FETCH_SIZE,      pTxChanCfg->fetchWordSize );
#ifdef CSL_UDMAP_TXCCFG_CHAN_TCFG_TDTYPE_MASK
    /* Teardown-type support valid in udmap version 2.0.0 and later */
    CSL_FINS( regVal, UDMAP_TXCCFG_CHAN_TCFG_TDTYPE,          pTxChanCfg->tdType );
#endif
    CSL_REG32_WR( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCFG, regVal );

    CSL_REG32_WR( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TOES[0],
                    CSL_FMK(UDMAP_TXCCFG_CHAN_TOES_EVT_NUM,         pTxChanCfg->trEventNum ) );

    CSL_REG32_WR( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TEOES[0],
                    CSL_FMK(UDMAP_TXCCFG_CHAN_TEOES_EVT_NUM,        pTxChanCfg->errEventNum ) );

    CSL_REG32_WR(&pCfg->pTxChanCfgRegs->CHAN[chanIdx].TPRI_CTRL,
                    CSL_FMK(UDMAP_TXCCFG_CHAN_TPRI_CTRL_PRIORITY, pTxChanCfg->busPriority) |
                    CSL_FMK(UDMAP_TXCCFG_CHAN_TPRI_CTRL_QOS,      pTxChanCfg->busQos)      |
                    CSL_FMK(UDMAP_TXCCFG_CHAN_TPRI_CTRL_ORDERID,  pTxChanCfg->busOrderId ) );

    CSL_REG32_WR(&pCfg->pTxChanCfgRegs->CHAN[chanIdx].TST_SCHED,
                  CSL_FMK(UDMAP_TXCCFG_CHAN_TST_SCHED_PRIORITY, pTxChanCfg->dmaPriority) );

    CSL_REG32_WR(&pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCREDIT, pTxChanCfg->txCredit);

    CSL_REG32_WR(&pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCQ, pTxChanCfg->txTrCQ);

    CSL_REG32_WR(&pCfg->pTxChanCfgRegs->CHAN[chanIdx].THREAD,
            CSL_FMK( UDMAP_TXCCFG_CHAN_THREAD_ID, pTxChanCfg->txThread ) );
    return 0;
}

int32_t CSL_udmapGetRxRT( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapRT *pRT )
{
    uint32_t val;

    val = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL );

    pRT->enable         = CSL_FEXT( val, UDMAP_RXCRT_CHAN_CTL_EN );
    pRT->teardown       = CSL_FEXT( val, UDMAP_RXCRT_CHAN_CTL_TDOWN );
    pRT->forcedTeardown = CSL_FEXT( val, UDMAP_RXCRT_CHAN_CTL_FTDOWN );
    pRT->pause          = CSL_FEXT( val, UDMAP_RXCRT_CHAN_CTL_PAUSE );
    pRT->error          = CSL_FEXT( val, UDMAP_RXCRT_CHAN_CTL_ERROR );

    return 0;
}

int32_t CSL_udmapSetRxRT( CSL_UdmapCfg *pCfg, uint32_t chanIdx, const CSL_UdmapRT *pRT )
{
    CSL_REG32_WR(&pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL,
                    CSL_FMK(UDMAP_RXCRT_CHAN_CTL_EN,           pRT->enable)         |
                    CSL_FMK(UDMAP_RXCRT_CHAN_CTL_TDOWN,        pRT->teardown)       |
                    CSL_FMK(UDMAP_RXCRT_CHAN_CTL_FTDOWN,       pRT->forcedTeardown) |
                    CSL_FMK(UDMAP_RXCRT_CHAN_CTL_PAUSE,        pRT->pause) );

    return 0;
}

int32_t CSL_udmapGetTxRT( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapRT *pRT )
{
    uint32_t val;

    val = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL );

    pRT->enable         = CSL_FEXT( val, UDMAP_TXCRT_CHAN_CTL_EN );
    pRT->teardown       = CSL_FEXT( val, UDMAP_TXCRT_CHAN_CTL_TDOWN );
    pRT->forcedTeardown = CSL_FEXT( val, UDMAP_TXCRT_CHAN_CTL_FTDOWN );
    pRT->pause          = CSL_FEXT( val, UDMAP_TXCRT_CHAN_CTL_PAUSE );
    pRT->error          = CSL_FEXT( val, UDMAP_TXCRT_CHAN_CTL_ERROR );

    return 0;
}

int32_t CSL_udmapSetTxRT( CSL_UdmapCfg *pCfg, uint32_t chanIdx, const CSL_UdmapRT *pRT )
{
    CSL_REG32_WR(&pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL,
                    CSL_FMK(UDMAP_TXCRT_CHAN_CTL_EN,           pRT->enable)         |
                    CSL_FMK(UDMAP_TXCRT_CHAN_CTL_TDOWN,        pRT->teardown)       |
                    CSL_FMK(UDMAP_TXCRT_CHAN_CTL_FTDOWN,       pRT->forcedTeardown) |
                    CSL_FMK(UDMAP_TXCRT_CHAN_CTL_PAUSE,        pRT->pause) );

    return 0;
}

int32_t CSL_udmapEnableTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapSetChanEnable( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_TX, (bool)true );
}

int32_t CSL_udmapDisableTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapSetChanEnable( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_TX, (bool)false );
}

int32_t CSL_udmapTeardownTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait )
{
    return CSL_udmapTeardownChan( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_TX, bForce, bWait );
}

int32_t CSL_udmapPauseTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapPauseChan( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_TX, CSL_UDMAP_PAUSE_CHAN );
}

int32_t CSL_udmapUnpauseTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapPauseChan( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_TX, CSL_UDMAP_UNPAUSE_CHAN );
}

int32_t CSL_udmapTriggerTxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapTriggerChan( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_TX );
}

void CSL_udmapClearTxChanError( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    CSL_udmapClearChanError( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_TX );
}

int32_t CSL_udmapEnableRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapSetChanEnable( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_RX, (bool)true );
}

int32_t CSL_udmapDisableRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapSetChanEnable( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_RX, (bool)false );
}

int32_t CSL_udmapTeardownRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait )
{
    return CSL_udmapTeardownChan( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_RX, bForce, bWait );
}

int32_t CSL_udmapPauseRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapPauseChan( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_RX, CSL_UDMAP_PAUSE_CHAN );
}

int32_t CSL_udmapUnpauseRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapPauseChan( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_RX, CSL_UDMAP_UNPAUSE_CHAN );
}

int32_t CSL_udmapTriggerRxChan( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    return CSL_udmapTriggerChan( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_RX );
}

void CSL_udmapClearRxChanError( CSL_UdmapCfg *pCfg, uint32_t chanIdx )
{
    CSL_udmapClearChanError( pCfg, chanIdx, CSL_UDMAP_CHAN_DIR_RX );
}

void CSL_udmapCfgRxFlowIdFirewall( CSL_UdmapCfg *pCfg, uint32_t outEvtNum )
{
    CSL_REG32_WR(&pCfg->pGenCfgRegs->RFLOWFWOES,
            CSL_FMK( UDMAP_GCFG_RFLOWFWOES_EVT_NUM, outEvtNum ) );
}

bool CSL_udmapGetRxFlowIdFirewallStatus( CSL_UdmapCfg *pCfg, CSL_UdmapRxFlowIdFirewallStatus *pRxFlowIdFwStatus )
{
    uint32_t regVal;
    bool retVal = (bool)false;

    regVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->RFLOWFWSTAT );

    if( CSL_FEXT( regVal, UDMAP_GCFG_RFLOWFWSTAT_PEND ) != 0U )
    {
        pRxFlowIdFwStatus->flowId = CSL_FEXT( regVal, UDMAP_GCFG_RFLOWFWSTAT_FLOWID );
        pRxFlowIdFwStatus->chnIdx = CSL_FEXT( regVal, UDMAP_GCFG_RFLOWFWSTAT_CHANNEL );
        /* Clear pend bit to allow another exception to be captured */
        CSL_REG32_WR( &pCfg->pGenCfgRegs->RFLOWFWSTAT, 0 );
        retVal = (bool)true;
    }
    return retVal;
}

void CSL_udmapGetChanStats( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, CSL_UdmapChanStats *pChanStats )
{
    if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
    {
        pChanStats->packetCnt        = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].PCNT );
        pChanStats->completedByteCnt = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].BCNT );
        pChanStats->startedByteCnt   = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].SBCNT );
    }
    else
    {
        pChanStats->packetCnt        = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].PCNT );
        pChanStats->completedByteCnt = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].BCNT );
        pChanStats->startedByteCnt   = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].SBCNT );
    }
}

void CSL_udmapDecChanStats( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, const CSL_UdmapChanStats *pChanStats )
{
    if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
    {
        CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].PCNT,  pChanStats->packetCnt );
        CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].BCNT,  pChanStats->completedByteCnt );
        CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].SBCNT, pChanStats->startedByteCnt );
    }
    else
    {
        CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].PCNT,  pChanStats->packetCnt );
        CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].BCNT,  pChanStats->completedByteCnt );
        CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].SBCNT, pChanStats->startedByteCnt );
    }
}

int32_t CSL_udmapGetChanPeerReg( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, uint32_t regIdx, uint32_t *pVal )
{
    return CSL_udmapAccessChanPeerReg( pCfg, chanIdx, regIdx, pVal, chanDir, (bool)true );
}

int32_t CSL_udmapSetChanPeerReg( const CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir, uint32_t regIdx, uint32_t *pVal )
{
    return CSL_udmapAccessChanPeerReg( pCfg, chanIdx, regIdx, pVal, chanDir, (bool)false );
}

int32_t CSL_udmapEnableLink( CSL_UdmapCfg *pCfg, uint32_t chanIdx, CSL_UdmapChanDir chanDir )
{
    int32_t retVal = -1;
    uint32_t peerEnableRegVal;

    peerEnableRegVal = (uint32_t)1U << 31;
    if( chanDir == CSL_UDMAP_CHAN_DIR_TX )
    {
        /* a. Set UDMAP peer real-time enable by calling the CSL_udmapSetChanPeerReg() function */
        if( CSL_udmapSetChanPeerReg( pCfg, chanIdx, chanDir, CSL_UDMAP_CHAN_PEER_REG_OFFSET_ENABLE, &peerEnableRegVal ) == 0 )
        {
            /* b. Enable the UDMAP tx channel by calling the CSL_udmapEnableTxChan() function */
            if( CSL_udmapEnableTxChan( pCfg, chanIdx ) == 0 )
            {
                retVal = 0;
            }
        }
    }
    if( chanDir == CSL_UDMAP_CHAN_DIR_RX )
    {
        /* a. Enable the UDMAP rx channel by calling the CSL_udmapEnableRxChan() function */
        if( CSL_udmapEnableRxChan( pCfg, chanIdx ) == 0 )
        {
            /* b. Set UDMAP peer real-time enable by calling the CSL_udmapSetChanPeerReg() function */
            if( CSL_udmapSetChanPeerReg( pCfg, chanIdx, chanDir, CSL_UDMAP_CHAN_PEER_REG_OFFSET_ENABLE, &peerEnableRegVal ) == 0 )
            {
                retVal = 0;
            }
        }
    }
    return retVal;
}

int32_t CSL_udmapSetAutoClockGatingEnable( CSL_UdmapCfg *pCfg, CSL_UdmapAutoClkgateBlock blockIds, bool bEnable )
{
    int32_t retVal = (int32_t) -1;

#ifdef CSL_UDMAP_GCFG_PM0_NOGATE_LO_MASK
    /*
     *  UDMAP revision 2.2.0.0 and later provides automatic dynamic clock gating
     *  support for internal sub-blocks based on real-time monitoring of activity.
     */
   if( bIsModuleRevAtLeast( pCfg, 2U, 2U, 0U) == (bool)true )
   {
        uint32_t regVal, blockIdMask;

        blockIdMask = (uint32_t)(blockIds & ((CSL_UdmapAutoClkgateBlock)0xFFFFFFFFU));
        if( blockIdMask != 0U )
        {
            regVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->PM0 );
            if( bEnable == (bool)true )
            {
                /* Clear bits (enable) in UDMAP_GCFG_PM0_NOGATE_LO corresponding to 1 bits in blockIdMask */
                regVal &= ~blockIdMask;
            }
            else
            {
                /* Set bits (disable) in UDMAP_GCFG_PM0_NOGATE_LO corresponding to 1 bits in blockIdMask */
                regVal |= blockIdMask;
            }
            CSL_REG32_WR( &pCfg->pGenCfgRegs->PM0, regVal );
        }
        blockIdMask = (uint32_t)(blockIds >> 32);
        if( blockIdMask != 0U )
        {
            regVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->PM1 );
            if( bEnable == (bool)true )
            {
                /* Clear bits (enable) in UDMAP_GCFG_PM1_NOGATE_HI corresponding to 1 bits in blockIdMask */
                regVal &= ~blockIdMask;
            }
            else
            {
                /* Set bits (disable) in UDMAP_GCFG_PM1_NOGATE_HI corresponding to 1 bits in blockIdMask */
                regVal |= blockIdMask;
            }
            CSL_REG32_WR( &pCfg->pGenCfgRegs->PM1, regVal );
        }
        retVal = 0;
    }
#endif
    return retVal;
}

int32_t CSL_udmapSetCommandThrottleThreshold( CSL_UdmapCfg *pCfg, CSL_UdmapMasterInterface interfaceId, uint32_t readCountThresh, uint32_t writeCountThresh )
{
    int32_t retVal = (int32_t) -1;

#ifdef CSL_UDMAP_GCFG_PERF0_VRD_THRESH0_MASK
    /* UDMAP revision 2.1.32.0 and later supports this feature */
    if( bIsModuleRevAtLeast( pCfg, 2U, 1U, 32U) == (bool)true )
    {
        if( ((readCountThresh == 0U) && (interfaceId != CSL_UDMAP_MASTER_INTERFACE_UTC_WRITE))   ||
            ((writeCountThresh == 0U) && (interfaceId != CSL_UDMAP_MASTER_INTERFACE_UTC_READ)) )
        {
            retVal = (int32_t) -2;      /* Invalid read and/or write count threshold */
        }
        else
        {
            retVal = 0;
            switch( interfaceId )
            {
                case CSL_UDMAP_MASTER_INTERFACE_PKTDMA_0:
                    CSL_REG32_WR( &pCfg->pGenCfgRegs->PERF0,
                        CSL_FMK(UDMAP_GCFG_PERF0_VRD_THRESH0, readCountThresh) |
                        CSL_FMK(UDMAP_GCFG_PERF0_VWR_THRESH0, writeCountThresh) );
                    break;
                case CSL_UDMAP_MASTER_INTERFACE_PKTDMA_1:
                    CSL_REG32_WR( &pCfg->pGenCfgRegs->PERF1,
                        CSL_FMK(UDMAP_GCFG_PERF1_VRD_THRESH1, readCountThresh) |
                        CSL_FMK(UDMAP_GCFG_PERF1_VWR_THRESH1, writeCountThresh) );
                    break;
                case CSL_UDMAP_MASTER_INTERFACE_UTC_READ:
                    CSL_REG32_WR( &pCfg->pGenCfgRegs->PERF2, CSL_FMK(UDMAP_GCFG_PERF2_VRD_THRESH2, readCountThresh) );
                    break;
                case CSL_UDMAP_MASTER_INTERFACE_UTC_WRITE:
                    CSL_REG32_WR( &pCfg->pGenCfgRegs->PERF3, CSL_FMK(UDMAP_GCFG_PERF3_VWR_THRESH3, writeCountThresh) );
                    break;
                default:
                    retVal = (int32_t) -2;      /* Invalid interface ID */
                    break;
            }
        }
    }
#endif
    return retVal;
}
