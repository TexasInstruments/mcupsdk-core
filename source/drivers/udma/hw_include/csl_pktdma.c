/**
 * @file  csl_pktdma.c
 *
 * @brief
 *  C implementation file for the PKTDMA module CSL-FL.
 *
 *  Contains the different control command and status query functions definitions
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
#include <string.h>
#include <stdbool.h>
#include <drivers/udma/hw_include/csl_pktdma.h>

/*=============================================================================
 *  Internal definitions and functions
 *===========================================================================*/

#define CSL_PKTDMA_CHAN_PEER_REG_OFFSET_ENABLE      ((uint32_t) 8U)

#define CSL_PKTDMA_FETCH_WORD_SIZE_16               ((uint32_t) 16U)
#define CSL_PKTDMA_NUM_PEER_REGS                    ((uint32_t) 16U)
#define CSL_PKTDMA_RXCRT_CHAN_PEER(CHAN,REG)        (uint32_t)(CSL_PKTDMA_RXCRT_CHAN_PEER0(CHAN)+((uint32_t)(REG)*0x4U))
#define CSL_PKTDMA_TXCRT_CHAN_PEER(CHAN,REG)        (uint32_t)(CSL_PKTDMA_TXCRT_CHAN_PEER0(CHAN)+((uint32_t)(REG)*0x4U))

#define CSL_PKTDMA_UNPAUSE_CHAN                     ((uint32_t) 0U)
#define CSL_PKTDMA_PAUSE_CHAN                       ((uint32_t) 1U)

#define CSL_PKTDMA_TEARDOWN_COMPLETE_WAIT_MAX_CNT   ((uint32_t) 128U)

static bool CSL_pktdmaIsChanEnabled( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir );
static bool CSL_pktdmaIsValidChanIdx( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir );
static int32_t CSL_pktdmaSetChanEnable( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, bool bEnable );
static int32_t CSL_pktdmaTeardownChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, bool bForce, bool bWait );
static int32_t CSL_pktdmaPauseChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, uint32_t pauseVal );
static int32_t CSL_pktdmaTriggerChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir );
static void CSL_pktdmaClearChanError( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir );
static int32_t CSL_pktdmaAccessChanPeerReg( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, uint32_t regIdx, uint32_t *pVal, CSL_PktdmaChanDir chanDir, bool bRdAccess );

static bool CSL_pktdmaIsChanEnabled( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir )
{
    uint32_t regVal;

    if( chanDir == CSL_PKTDMA_CHAN_DIR_TX )
    {
        regVal = CSL_REG32_FEXT( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, PKTDMA_TXCRT_CHAN_CTL_EN );
    }
    else
    {
        regVal = CSL_REG32_FEXT( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, PKTDMA_RXCRT_CHAN_CTL_EN );
    }
    return ((regVal == 1U) ? (bool)true : (bool)false);
}

static bool CSL_pktdmaIsValidChanIdx( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir )
{
    bool bRetVal;

    if( (chanDir == CSL_PKTDMA_CHAN_DIR_TX) && (chanIdx < pCfg->txChanCnt) )
    {
        bRetVal = (bool)true;
    }
    else if( (chanDir == CSL_PKTDMA_CHAN_DIR_RX) && (chanIdx < pCfg->rxChanCnt) )
    {
        bRetVal = (bool)true;
    }
    else
    {
        bRetVal = (bool)false;
    }
    return bRetVal;
}

static int32_t CSL_pktdmaSetChanEnable( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, bool bEnable )
{
    int32_t retVal = CSL_PASS;

    if( (pCfg == NULL) || (CSL_pktdmaIsValidChanIdx( pCfg, chanIdx, chanDir) == (bool)false) )
    {
        retVal = CSL_EFAIL;
    }
    else
    {
        if( chanDir == CSL_PKTDMA_CHAN_DIR_TX )
        {
            CSL_REG32_WR(&pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, CSL_FMK(PKTDMA_TXCRT_CHAN_CTL_EN,(bEnable ? (uint32_t)1 : (uint32_t)0)) );
        }
        else
        {
            CSL_REG32_WR(&pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, CSL_FMK(PKTDMA_RXCRT_CHAN_CTL_EN,(bEnable ? (uint32_t)1 : (uint32_t)0)) );
        }
    }
    return retVal;
}

static int32_t CSL_pktdmaTeardownChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, bool bForce, bool bWait )
{
    int32_t  retVal = CSL_PASS;

    if( (pCfg == NULL) || (CSL_pktdmaIsValidChanIdx( pCfg, chanIdx, chanDir) == (bool)false) )
    {
        retVal = CSL_EFAIL;
    }
    else
    {
        /* Channel can be torn down only when it is enabled */
        if( CSL_pktdmaIsChanEnabled( pCfg, chanIdx, chanDir ) == (bool)true )
        {
            uint32_t regVal;
            if( chanDir == CSL_PKTDMA_CHAN_DIR_TX )
            {
                regVal = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL );
                CSL_FINS( regVal, PKTDMA_TXCRT_CHAN_CTL_TDOWN, (uint32_t)1U );
                CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, regVal );
            }
            else
            {
                regVal = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL );
                CSL_FINS( regVal, PKTDMA_RXCRT_CHAN_CTL_TDOWN, (uint32_t)1U );
                CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, regVal );
            }
            if( bWait == (bool)true )
            {
                uint32_t retryCnt = CSL_PKTDMA_TEARDOWN_COMPLETE_WAIT_MAX_CNT;
                /* The channel's enable bit is cleared once teardown is complete */
                while( (CSL_pktdmaIsChanEnabled( pCfg, chanIdx, chanDir ) == (bool)true) && (retryCnt != (uint32_t)0U) )
                {
                    retryCnt--;
                }
                if( retryCnt == (uint32_t)0U ) {
                    retVal = CSL_EFAIL;
                }
            }
        }
        else
        {
            retVal = CSL_EFAIL;
        }
    }
    return retVal;
}

static int32_t CSL_pktdmaPauseChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, uint32_t pauseVal )
{
    int32_t retVal = CSL_PASS;

    if( (pCfg == NULL) || (CSL_pktdmaIsValidChanIdx( pCfg, chanIdx, chanDir) == (bool)false) )
    {
        retVal = CSL_EFAIL;
    }
    else
    {
         /* Channel can be paused/un-paused only when it is enabled */
        if( CSL_pktdmaIsChanEnabled( pCfg, chanIdx, chanDir ) == (bool)true )
        {
            if( chanDir == CSL_PKTDMA_CHAN_DIR_TX )
            {
                CSL_REG32_FINS( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, PKTDMA_TXCRT_CHAN_CTL_PAUSE, pauseVal );
            }
            else
            {
                CSL_REG32_FINS( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, PKTDMA_RXCRT_CHAN_CTL_PAUSE, pauseVal );
            }
        }
        else
        {
            retVal = CSL_EFAIL;
        }
   }
   return retVal;
}

static int32_t CSL_pktdmaTriggerChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir )
{
    return CSL_EUNSUPPORTED_CMD;
}

static void CSL_pktdmaClearChanError( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir )
{
    if( chanDir == CSL_PKTDMA_CHAN_DIR_TX )
    {
        CSL_REG32_FINS(&pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, PKTDMA_TXCRT_CHAN_CTL_ERROR, 0U);
    }
    else
    {
        CSL_REG32_FINS(&pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, PKTDMA_RXCRT_CHAN_CTL_ERROR, 0U);
    }
}

static int32_t CSL_pktdmaAccessChanPeerReg( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, uint32_t regIdx, uint32_t *pVal, CSL_PktdmaChanDir chanDir, bool bRdAccess )
{
    int32_t retVal = CSL_PASS;

    if( (pCfg == NULL) || (pVal == NULL) || (CSL_pktdmaIsValidChanIdx( pCfg, chanIdx, chanDir) == (bool)false) )
    {
        retVal = CSL_EFAIL;
    }
    else
    {
        uint32_t    *pPeerReg;

        if( regIdx < CSL_PKTDMA_NUM_PEER_REGS )
        {
            if( chanDir == CSL_PKTDMA_CHAN_DIR_TX )
            {
                pPeerReg = (uint32_t *)(((uintptr_t)pCfg->pTxChanRtRegs) + (uintptr_t)CSL_PKTDMA_TXCRT_CHAN_PEER0(chanIdx) + ((uintptr_t)regIdx * 0x4U));
            }
            else
            {
                pPeerReg = (uint32_t *)(((uintptr_t)pCfg->pRxChanRtRegs) + (uintptr_t)CSL_PKTDMA_RXCRT_CHAN_PEER0(chanIdx) + ((uintptr_t)regIdx * 0x4U));
            }
            if( bRdAccess == (bool)true )
            {
                *pVal = CSL_REG32_RD( pPeerReg );
            }
            else
            {
                CSL_REG32_WR( pPeerReg, *pVal );
            }
            retVal = CSL_PASS;
        }
        else
        {
            retVal = CSL_EFAIL;
        }
    }
    return retVal;
}

/*=============================================================================
 *  PKTDMA API functions
 *===========================================================================*/
uint32_t CSL_pktdmaGetRevision( const CSL_PktdmaCfg *pCfg )
{
    return CSL_REG32_RD( &pCfg->pGenCfgRegs->REVISION );
}

int32_t CSL_pktdmaGetRevisionInfo( const CSL_PktdmaCfg *pCfg, CSL_PktdmaRevision *pRev )
{
    uint32_t val;

    val = CSL_REG32_RD( &pCfg->pGenCfgRegs->REVISION );

    pRev->modId     = CSL_FEXT( val, PKTDMA_GCFG_REVISION_MODID );
    pRev->revRtl    = CSL_FEXT( val, PKTDMA_GCFG_REVISION_REVRTL );
    pRev->revMajor  = CSL_FEXT( val, PKTDMA_GCFG_REVISION_REVMAJ );
    pRev->custom    = CSL_FEXT( val, PKTDMA_GCFG_REVISION_CUSTOM );
    pRev->revMinor  = CSL_FEXT( val, PKTDMA_GCFG_REVISION_REVMIN );

    return CSL_PASS;
}

void CSL_pktdmaInitCfg( CSL_PktdmaCfg *pCfg )
{
    if( pCfg != NULL )
    {
        memset( (void *)pCfg, 0, sizeof(CSL_PktdmaCfg) );
    }
}

void CSL_pktdmaGetCfg( CSL_PktdmaCfg *pCfg )
{
    if( pCfg != NULL )
    {
        uint32_t regVal;

        pCfg->cap0 = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP0 );
        pCfg->cap1 = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP1 );
        regVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP2 );
        pCfg->txChanCnt       = CSL_FEXT( regVal, PKTDMA_GCFG_CAP2_TCHAN_CNT );
        pCfg->rxChanCnt       = CSL_FEXT( regVal, PKTDMA_GCFG_CAP2_RCHAN_CNT );
        pCfg->txExtUtcChanCnt = (uint32_t)0U;
        regVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP3 );
        pCfg->rxFlowCnt       = CSL_FEXT( regVal, PKTDMA_GCFG_CAP3_RFLOW_CNT );
        pCfg->txHighCapacityChanCnt = CSL_FEXT( regVal, PKTDMA_GCFG_CAP3_HCHAN_CNT );
        pCfg->txUltraHighCapacityChanCnt = CSL_FEXT( regVal, PKTDMA_GCFG_CAP3_UCHAN_CNT );
    }
}

void CSL_pktdmaInitTxChanCfg( CSL_PktdmaTxChanCfg *pTxChanCfg )
{
    if( pTxChanCfg != NULL )
    {
        /*-------------------------------------------------------------------------
         *  Start by initializing all structure members to 0
         *-----------------------------------------------------------------------*/
        memset( (void *)pTxChanCfg, 0, sizeof(CSL_PktdmaTxChanCfg) );
        /*-------------------------------------------------------------------------
         *  Now initialize non-zero structure members
         *-----------------------------------------------------------------------*/
        pTxChanCfg->chanType        = CSL_PKTDMA_CHAN_TYPE_NORMAL;
        pTxChanCfg->fetchWordSize   = CSL_PKTDMA_FETCH_WORD_SIZE_16;
        pTxChanCfg->trEventNum      = CSL_PKTDMA_NO_EVENT;
        pTxChanCfg->errEventNum     = CSL_PKTDMA_NO_EVENT;
    }
}

void CSL_pktdmaInitRxChanCfg( CSL_PktdmaRxChanCfg *pRxChanCfg )
{
    if( pRxChanCfg != NULL )
    {
        /*-------------------------------------------------------------------------
         *  Start by initializing all structure members to 0
         *-----------------------------------------------------------------------*/
        memset( (void *)pRxChanCfg, 0, sizeof(CSL_PktdmaRxChanCfg) );
        /*-------------------------------------------------------------------------
         *  Now initialize non-zero structure members
         *-----------------------------------------------------------------------*/
        pRxChanCfg->chanType            = CSL_PKTDMA_CHAN_TYPE_NORMAL;
        pRxChanCfg->fetchWordSize       = CSL_PKTDMA_FETCH_WORD_SIZE_16;
        pRxChanCfg->trEventNum          = CSL_PKTDMA_NO_EVENT;
        pRxChanCfg->errEventNum         = CSL_PKTDMA_NO_EVENT;
        pRxChanCfg->flowIdFwRangeCnt    = 0x00004000U;
    }
}

void CSL_pktdmaInitRxFlowCfg( CSL_PktdmaRxFlowCfg *pFlow )
{
    if( pFlow != NULL )
    {
        /*-------------------------------------------------------------------------
         *  Start by initializing all structure members to 0
         *-----------------------------------------------------------------------*/
        memset( (void *)pFlow, 0, sizeof(CSL_PktdmaRxFlowCfg) );
    }
}

void CSL_pktdmaSetPerfCtrl( CSL_PktdmaCfg *pCfg, uint32_t rxRetryTimeoutCnt )
{
    uint32_t regVal;

    regVal = CSL_FMK( PKTDMA_GCFG_PERF_CTRL_TIMEOUT_CNT, rxRetryTimeoutCnt );
    CSL_REG32_WR( &pCfg->pGenCfgRegs->PERF_CTRL, regVal );
}

void CSL_pktdmaSetUtcCtrl( CSL_PktdmaCfg *pCfg, uint32_t startingThreadNum )
{
}

int32_t CSL_pktdmaRxFlowCfg(CSL_PktdmaCfg *pCfg, uint32_t flow, const CSL_PktdmaRxFlowCfg *pFlow )
{
    int32_t retVal;

    if( (pCfg == NULL) || (pFlow == NULL) || (flow >= pCfg->rxFlowCnt) )
    {
        retVal = CSL_EFAIL;
    }
    else
    {
        CSL_REG32_WR(&pCfg->pRxFlowCfgRegs->FLOW[flow].RFA,
                        CSL_FMK(PKTDMA_RXFCFG_FLOW_RFA_EINFO,       pFlow->einfoPresent)  |
                        CSL_FMK(PKTDMA_RXFCFG_FLOW_RFA_PSINFO,      pFlow->psInfoPresent) |
                        CSL_FMK(PKTDMA_RXFCFG_FLOW_RFA_EHANDLING,   pFlow->errorHandling) |
                        CSL_FMK(PKTDMA_RXFCFG_FLOW_RFA_SOP_OFF,     pFlow->sopOffset) );
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_pktdmaRxChanSetTrEvent( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum )
{
    return CSL_EUNSUPPORTED_CMD;
}

int32_t CSL_pktdmaRxChanSetBurstSize( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanBurstSize burstSize )
{
    int32_t retVal = CSL_EFAIL;

    if( (burstSize >= CSL_PKTDMA_CHAN_BURST_SIZE_64_BYTES) && (burstSize <= CSL_PKTDMA_CHAN_BURST_SIZE_256_BYTES) )
    {
        CSL_REG32_FINS( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCFG, PKTDMA_RXCCFG_CHAN_RCFG_BURST_SIZE, burstSize );
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_pktdmaRxChanCfg( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, const CSL_PktdmaRxChanCfg *pRxChanCfg )
{
    int32_t retVal;

    if( (pCfg == NULL) || (pRxChanCfg == NULL) || (chanIdx >= pCfg->rxChanCnt) )
    {
        retVal = CSL_EFAIL;
    }
    else
    {
        uint32_t regVal;

        regVal = CSL_REG32_RD( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCFG );
        CSL_FINS( regVal, PKTDMA_RXCCFG_CHAN_RCFG_PAUSE_ON_ERR,  pRxChanCfg->pauseOnError );
        CSL_FINS( regVal, PKTDMA_RXCCFG_CHAN_RCFG_CHAN_TYPE,     pRxChanCfg->chanType );
        CSL_REG32_WR( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCFG, regVal );

        CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].RPRI_CTRL,
                CSL_FMK(PKTDMA_RXCCFG_CHAN_RPRI_CTRL_PRIORITY,   pRxChanCfg->busPriority) |
                CSL_FMK(PKTDMA_RXCCFG_CHAN_RPRI_CTRL_ORDERID,    pRxChanCfg->busOrderId ) );

        CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].THREAD,
                CSL_FMK( PKTDMA_RXCCFG_CHAN_THREAD_ID, pRxChanCfg->rxThread ) );

        CSL_REG32_WR(&pCfg->pRxChanCfgRegs->CHAN[chanIdx].RST_SCHED,
                      CSL_FMK(PKTDMA_RXCCFG_CHAN_RST_SCHED_PRIORITY, pRxChanCfg->dmaPriority) );
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_pktdmaTxChanSetTrEvent( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum )
{
    return CSL_EUNSUPPORTED_CMD;
}

int32_t CSL_pktdmaTxChanSetBurstSize( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanBurstSize burstSize )
{
    int32_t retVal = CSL_EFAIL;

    if( (burstSize >= CSL_PKTDMA_CHAN_BURST_SIZE_64_BYTES) && (burstSize <= CSL_PKTDMA_CHAN_BURST_SIZE_256_BYTES) )
    {
        CSL_REG32_FINS( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCFG, PKTDMA_TXCCFG_CHAN_TCFG_BURST_SIZE, burstSize );
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_pktdmaTxChanCfg( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, const CSL_PktdmaTxChanCfg *pTxChanCfg )
{
    int32_t retVal;

    if( (pCfg == NULL) || (pTxChanCfg == NULL) || (chanIdx >= pCfg->txChanCnt) )
    {
        retVal = CSL_EFAIL;
    }
    else
    {
        uint32_t regVal;

        regVal = CSL_REG32_RD( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCFG );
        CSL_FINS( regVal, PKTDMA_TXCCFG_CHAN_TCFG_PAUSE_ON_ERR,    pTxChanCfg->pauseOnError );
        CSL_FINS( regVal, PKTDMA_TXCCFG_CHAN_TCFG_FILT_EINFO,      pTxChanCfg->filterEinfo );
        CSL_FINS( regVal, PKTDMA_TXCCFG_CHAN_TCFG_FILT_PSWORDS,    pTxChanCfg->filterPsWords );
        CSL_FINS( regVal, PKTDMA_TXCCFG_CHAN_TCFG_NOTDPKT,         ((pTxChanCfg->bNoTeardownCompletePkt) ? (uint32_t)1U : (uint32_t)0U) );
        CSL_FINS( regVal, PKTDMA_TXCCFG_CHAN_TCFG_CHAN_TYPE,       pTxChanCfg->chanType );
        CSL_FINS( regVal, PKTDMA_TXCCFG_CHAN_TCFG_TDTYPE,          pTxChanCfg->tdType );
        CSL_REG32_WR( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCFG, regVal );

        CSL_REG32_WR(&pCfg->pTxChanCfgRegs->CHAN[chanIdx].TPRI_CTRL,
                        CSL_FMK(PKTDMA_TXCCFG_CHAN_TPRI_CTRL_PRIORITY, pTxChanCfg->busPriority) |
                        CSL_FMK(PKTDMA_TXCCFG_CHAN_TPRI_CTRL_ORDERID,  pTxChanCfg->busOrderId ) );

        CSL_REG32_WR(&pCfg->pTxChanCfgRegs->CHAN[chanIdx].TST_SCHED,
                      CSL_FMK(PKTDMA_TXCCFG_CHAN_TST_SCHED_PRIORITY, pTxChanCfg->dmaPriority) );

        CSL_REG32_WR(&pCfg->pTxChanCfgRegs->CHAN[chanIdx].THREAD,
                CSL_FMK( PKTDMA_TXCCFG_CHAN_THREAD_ID, pTxChanCfg->txThread ) );
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_pktdmaGetRxRT( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaRT *pRT )
{
    uint32_t val;

    val = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL );

    pRT->enable         = CSL_FEXT( val, PKTDMA_RXCRT_CHAN_CTL_EN );
    pRT->teardown       = CSL_FEXT( val, PKTDMA_RXCRT_CHAN_CTL_TDOWN );
    pRT->forcedTeardown = (uint32_t)0U;
    pRT->pause          = CSL_FEXT( val, PKTDMA_RXCRT_CHAN_CTL_PAUSE );
    pRT->error          = CSL_FEXT( val, PKTDMA_RXCRT_CHAN_CTL_ERROR );

    return CSL_PASS;
}

int32_t CSL_pktdmaSetRxRT( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, const CSL_PktdmaRT *pRT )
{
    CSL_REG32_WR(&pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL,
                    CSL_FMK(PKTDMA_RXCRT_CHAN_CTL_EN,           pRT->enable)         |
                    CSL_FMK(PKTDMA_RXCRT_CHAN_CTL_TDOWN,        pRT->teardown)       |
                    CSL_FMK(PKTDMA_RXCRT_CHAN_CTL_PAUSE,        pRT->pause) );

    return CSL_PASS;
}

int32_t CSL_pktdmaGetTxRT( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaRT *pRT )
{
    uint32_t val;

    val = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL );

    pRT->enable         = CSL_FEXT( val, PKTDMA_TXCRT_CHAN_CTL_EN );
    pRT->teardown       = CSL_FEXT( val, PKTDMA_TXCRT_CHAN_CTL_TDOWN );
    pRT->forcedTeardown = (uint32_t)0U;
    pRT->pause          = CSL_FEXT( val, PKTDMA_TXCRT_CHAN_CTL_PAUSE );
    pRT->error          = CSL_FEXT( val, PKTDMA_TXCRT_CHAN_CTL_ERROR );

    return CSL_PASS;
}

int32_t CSL_pktdmaSetTxRT( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, const CSL_PktdmaRT *pRT )
{
    CSL_REG32_WR(&pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL,
                    CSL_FMK(PKTDMA_TXCRT_CHAN_CTL_EN,           pRT->enable)         |
                    CSL_FMK(PKTDMA_TXCRT_CHAN_CTL_TDOWN,        pRT->teardown)       |
                    CSL_FMK(PKTDMA_TXCRT_CHAN_CTL_PAUSE,        pRT->pause) );

    return CSL_PASS;
}

int32_t CSL_pktdmaEnableTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaSetChanEnable( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_TX, (bool)true );
}

int32_t CSL_pktdmaDisableTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaSetChanEnable( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_TX, (bool)false );
}

int32_t CSL_pktdmaTeardownTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait )
{
    return CSL_pktdmaTeardownChan( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_TX, bForce, bWait );
}

int32_t CSL_pktdmaPauseTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaPauseChan( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_TX, CSL_PKTDMA_PAUSE_CHAN );
}

int32_t CSL_pktdmaUnpauseTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaPauseChan( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_TX, CSL_PKTDMA_UNPAUSE_CHAN );
}

int32_t CSL_pktdmaTriggerTxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaTriggerChan( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_TX );
}

void CSL_pktdmaClearTxChanError( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    CSL_pktdmaClearChanError( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_TX );
}

int32_t CSL_pktdmaEnableRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaSetChanEnable( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_RX, (bool)true );
}

int32_t CSL_pktdmaDisableRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaSetChanEnable( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_RX, (bool)false );
}

int32_t CSL_pktdmaTeardownRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait )
{
    return CSL_pktdmaTeardownChan( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_RX, bForce, bWait );
}

int32_t CSL_pktdmaPauseRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaPauseChan( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_RX, CSL_PKTDMA_PAUSE_CHAN );
}

int32_t CSL_pktdmaUnpauseRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaPauseChan( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_RX, CSL_PKTDMA_UNPAUSE_CHAN );
}

int32_t CSL_pktdmaTriggerRxChan( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    return CSL_pktdmaTriggerChan( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_RX );
}

void CSL_pktdmaClearRxChanError( CSL_PktdmaCfg *pCfg, uint32_t chanIdx )
{
    CSL_pktdmaClearChanError( pCfg, chanIdx, CSL_PKTDMA_CHAN_DIR_RX );
}

void CSL_pktdmaCfgRxFlowIdFirewall( CSL_PktdmaCfg *pCfg, uint32_t outEvtNum )
{
}

bool CSL_pktdmaGetRxFlowIdFirewallStatus( CSL_PktdmaCfg *pCfg, CSL_PktdmaRxFlowIdFirewallStatus *pRxFlowIdFwStatus )
{
    bool bRetVal = (bool)false;
#ifdef CSL_PKTDMA_GCFG_RFLOWFWSTAT_PEND_MASK
    uint32_t regVal;

    regVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->RFLOWFWSTAT );
    if( CSL_FEXT( regVal, PKTDMA_GCFG_RFLOWFWSTAT_PEND ) != (uint32_t)0U )
    {
        pRxFlowIdFwStatus->flowId = CSL_FEXT( regVal, PKTDMA_GCFG_RFLOWFWSTAT_FLOWID );
        pRxFlowIdFwStatus->chnIdx = CSL_FEXT( regVal, PKTDMA_GCFG_RFLOWFWSTAT_CHANNEL );
        /* Clear pending bit to ready flow id firewall to capture next error */
        CSL_FINS( regVal, PKTDMA_GCFG_RFLOWFWSTAT_PEND, (uint32_t)0U );
        CSL_REG32_WR( &pCfg->pGenCfgRegs->RFLOWFWSTAT, regVal );
        bRetVal = (bool)true;
    }
#endif
    return bRetVal;
}

void CSL_pktdmaGetChanStats( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, CSL_PktdmaChanStats *pChanStats )
{
    if( chanDir == CSL_PKTDMA_CHAN_DIR_TX )
    {
        pChanStats->packetCnt        = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].PCNT );
        pChanStats->completedByteCnt = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].BCNT );
        pChanStats->startedByteCnt   = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].SBCNT );
        pChanStats->droppedPacketCnt = (uint32_t)0U;
    }
    else
    {
        pChanStats->packetCnt        = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].PCNT );
        pChanStats->completedByteCnt = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].BCNT );
        pChanStats->startedByteCnt   = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].SBCNT );
        pChanStats->droppedPacketCnt = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].DCNT );
    }
}

void CSL_pktdmaDecChanStats( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, const CSL_PktdmaChanStats *pChanStats )
{
    if( chanDir == CSL_PKTDMA_CHAN_DIR_TX )
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
        CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].DCNT,  pChanStats->droppedPacketCnt );
    }
}

int32_t CSL_pktdmaGetChanPeerReg( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, uint32_t regIdx, uint32_t *pVal )
{
    return CSL_pktdmaAccessChanPeerReg( pCfg, chanIdx, regIdx, pVal, chanDir, (bool)true );
}

int32_t CSL_pktdmaSetChanPeerReg( const CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir, uint32_t regIdx, uint32_t *pVal )
{
    return CSL_pktdmaAccessChanPeerReg( pCfg, chanIdx, regIdx, pVal, chanDir, (bool)false );
}

int32_t CSL_pktdmaEnableLink( CSL_PktdmaCfg *pCfg, uint32_t chanIdx, CSL_PktdmaChanDir chanDir )
{
    int32_t  retVal = CSL_PASS;

    if( (pCfg == NULL) || (CSL_pktdmaIsValidChanIdx( pCfg, chanIdx, chanDir) == (bool)false) )
    {
        retVal = CSL_EFAIL;
    }
    else
    {
        uint32_t peerEnableRegVal;

        peerEnableRegVal = (uint32_t)1U << 31;
        if( chanDir == CSL_PKTDMA_CHAN_DIR_TX )
        {
            /* a. Set PKTDMA peer real-time enable by calling the CSL_pktdmaSetChanPeerReg() function */
            retVal = CSL_pktdmaSetChanPeerReg( pCfg, chanIdx, chanDir, CSL_PKTDMA_CHAN_PEER_REG_OFFSET_ENABLE, &peerEnableRegVal );
            if( retVal == CSL_PASS )
            {
                /* b. Enable the PKTDMA tx channel by calling the CSL_pktdmaEnableTxChan() function */
                retVal = CSL_pktdmaEnableTxChan( pCfg, chanIdx );
            }
        }
        else
        {
            /* a. Enable the PKTDMA rx channel by calling the CSL_pktdmaEnableRxChan() function */
            retVal = CSL_pktdmaEnableRxChan( pCfg, chanIdx );
            if( retVal == CSL_PASS )
            {
                /* b. Set PKTDMA peer real-time enable by calling the CSL_pktdmaSetChanPeerReg() function */
                retVal = CSL_pktdmaSetChanPeerReg( pCfg, chanIdx, chanDir, CSL_PKTDMA_CHAN_PEER_REG_OFFSET_ENABLE, &peerEnableRegVal );
            }
        }
    }
    return retVal;
}
