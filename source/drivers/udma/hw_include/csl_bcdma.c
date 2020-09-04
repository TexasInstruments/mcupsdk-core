/**
 * @file  csl_bcdma.c
 *
 * @brief
 *  C implementation file for the BCDMA module CSL-FL.
 *
 *  Contains the different control command and status query functions definitions
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2019-2020, Texas Instruments, Inc.
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
#include <stdbool.h>
#include <string.h>
#include <drivers/udma/hw_include/csl_bcdma.h>

#define CSL_BCDMA_INVALID_CHANNEL_INDEX             ((uint32_t) -1)
#define CSL_BCDMA_CHAN_PEER_REG_OFFSET_ENABLE       ((uint32_t) 8U)
#define CSL_BCDMA_TEARDOWN_COMPLETE_WAIT_MAX_CNT    ((uint32_t) 128U)

/* ----------------------------------------------------------------------------
 *  Static internal functions
 * ----------------------------------------------------------------------------
 */
static uint32_t CSL_bcdmaMapChanIdx( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanType *chanType );
static int32_t CSL_bcdmaDoChanOp( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanOp chanOp, uint32_t chanIdx, void *pOpData );
static bool    CSL_bcdmaChanOpIsValidChanIdx( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx );
static bool    CSL_bcdmaChanOpIsChanEnabled( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx );
static int32_t CSL_bcdmaChanOpCfgChan( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData );
static int32_t CSL_bcdmaChanOpSetChanEnable( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, bool bEnable );
static int32_t CSL_bcdmaChanOpSetChanPause( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, bool bPause );
static int32_t CSL_bcdmaChanOpTeardownChan( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData );
static int32_t CSL_bcdmaChanOpTriggerChan( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx );
static int32_t CSL_bcdmaChanOpGetChanRT( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData );
static int32_t CSL_bcdmaChanOpSetChanRT( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData );
static int32_t CSL_bcdmaChanOpGetChanStats( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData );
static int32_t CSL_bcdmaChanOpDecChanStats( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData );
static int32_t CSL_bcdmaChanOpAccessRemotePeerReg( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData, bool bRead );
static int32_t CSL_bcdmaChanOpSetBurstSize( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData );
static int32_t CSL_bcdmaChanOpClearError( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx );

static uint32_t CSL_bcdmaMapChanIdx( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanType *chanType )
{
    uint32_t base0chanIdx;

    if( chanIdx < pCfg->bcChanCnt )
    {
        *chanType = CSL_BCDMA_CHAN_TYPE_BLOCK_COPY;
        base0chanIdx = chanIdx;
    }
    else if( chanIdx < (pCfg->bcChanCnt + pCfg->splitTxChanCnt) )
    {
        *chanType = CSL_BCDMA_CHAN_TYPE_SPLIT_TX;
        base0chanIdx = chanIdx - pCfg->bcChanCnt;
    }
    else if( chanIdx < (pCfg->bcChanCnt + pCfg->splitTxChanCnt + pCfg->splitRxChanCnt) )
    {
        *chanType = CSL_BCDMA_CHAN_TYPE_SPLIT_RX;
        base0chanIdx = chanIdx - pCfg->bcChanCnt - pCfg->splitTxChanCnt;
    }
    else
    {
        base0chanIdx = CSL_BCDMA_INVALID_CHANNEL_INDEX;
    }
    return base0chanIdx;
}

static int32_t CSL_bcdmaDoChanOp( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanOp chanOp, uint32_t chanIdx, void *pOpData )
{
    int32_t retVal = CSL_EFAIL;

    if( pCfg == NULL )
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        uint32_t base0chanIdx;
        CSL_BcdmaChanType chanType;
        /*
         * Call CSL_bcdmaGetCfg to populate the bcdma cfg structure if it appears
         * the caller has not yet done so.
         */
        if( (pCfg->bcChanCnt == (uint32_t)0U) || (pCfg->splitTxChanCnt == (uint32_t)0U) || (pCfg->splitRxChanCnt == (uint32_t)0U) ) {
            CSL_bcdmaGetCfg( pCfg );
        }
        base0chanIdx = CSL_bcdmaMapChanIdx( pCfg, chanIdx, &chanType );
        if( base0chanIdx != CSL_BCDMA_INVALID_CHANNEL_INDEX )
        {
            retVal = CSL_bcdmaChanOp( pCfg, chanOp, chanType, base0chanIdx, pOpData );
        }
    }
    return retVal;
}

static bool CSL_bcdmaChanOpIsValidChanIdx( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx )
{
    bool retVal = (bool)true;

    if( chanType == CSL_BCDMA_CHAN_TYPE_BLOCK_COPY )
    {
        if( chanIdx > pCfg->bcChanCnt )
        {
            retVal = (bool)false;
        }
    }
    else if( chanType == CSL_BCDMA_CHAN_TYPE_SPLIT_RX )
    {
        if( chanIdx > pCfg->splitRxChanCnt )
        {
            retVal = (bool)false;
        }
    }
    else if( chanType == CSL_BCDMA_CHAN_TYPE_SPLIT_TX )
    {
        if( chanIdx > pCfg->splitTxChanCnt )
        {
            retVal = (bool)false;
        }
    }
    else
    {
        retVal = (bool)false;
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpCfgChan( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData )
{
    int32_t retVal = CSL_PASS;

    if( pOpData == NULL )
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        uint32_t regVal;
        switch( chanType )
        {
            case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
                {
                    CSL_BcdmaTxChanCfg *pChanCfg = (CSL_BcdmaTxChanCfg *)pOpData;
                    if( (pChanCfg->burstSize > CSL_BCDMA_CHAN_BURST_SIZE_128_BYTES)   ||    /* Block-copy supports 32, 64, and 128-byte bursts */
                        (pChanCfg->busPriority > ((uint32_t)7U) )                     ||
                        (pChanCfg->dmaPriority > ((uint32_t)3U) )
                      )
                    {
                        retVal = CSL_EINVALID_PARAMS;
                    }
                    else
                    {
                        /* CFG */
                        regVal = CSL_REG32_RD( &pCfg->pBcChanCfgRegs->CHAN[chanIdx].CFG );
                        CSL_FINS( regVal, BCDMA_BCCFG_CHAN_CFG_PAUSE_ON_ERR, pChanCfg->pauseOnError );
                        CSL_FINS( regVal, BCDMA_BCCFG_CHAN_CFG_BURST_SIZE, pChanCfg->burstSize );
                        CSL_REG32_WR( &pCfg->pBcChanCfgRegs->CHAN[chanIdx].CFG, regVal );
                        /* PRI_CTRL */
                        regVal = CSL_FMK( BCDMA_BCCFG_CHAN_PRI_CTRL_PRIORITY, pChanCfg->busPriority )    |
                                 CSL_FMK( BCDMA_BCCFG_CHAN_PRI_CTRL_ORDERID, pChanCfg->busOrderId );
                        CSL_REG32_WR( &pCfg->pBcChanCfgRegs->CHAN[chanIdx].PRI_CTRL, regVal );
                        /* TST_SCHED */
                        CSL_REG32_WR( &pCfg->pBcChanCfgRegs->CHAN[chanIdx].TST_SCHED, CSL_FMK(BCDMA_BCCFG_CHAN_TST_SCHED_PRIORITY, pChanCfg->dmaPriority) );
                    }
                }
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
                {
                    CSL_BcdmaTxChanCfg *pChanCfg = (CSL_BcdmaTxChanCfg *)pOpData;
                    if( (pChanCfg->burstSize > CSL_BCDMA_CHAN_BURST_SIZE_64_BYTES)    ||    /* Split-tx supports 32, and 64-byte bursts */
                        (pChanCfg->busPriority > ((uint32_t)7U) )                     ||
                        (pChanCfg->dmaPriority > ((uint32_t)3U) )
                      )
                    {
                        retVal = CSL_EINVALID_PARAMS;
                    }
                    else
                    {
                        /* TCFG */
                        regVal = CSL_REG32_RD( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCFG );
                        CSL_FINS( regVal, BCDMA_TXCCFG_CHAN_TCFG_PAUSE_ON_ERR, pChanCfg->pauseOnError);
                        CSL_FINS( regVal, BCDMA_TXCCFG_CHAN_TCFG_BURST_SIZE, pChanCfg->burstSize );
                        CSL_FINS( regVal, BCDMA_TXCCFG_CHAN_TCFG_TDTYPE, pChanCfg->tdType );
                        CSL_FINS( regVal, BCDMA_TXCCFG_CHAN_TCFG_NOTDPKT, pChanCfg->bNoTeardownCompletePkt );
                        CSL_REG32_WR( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCFG, regVal );
                        /* TPRI_CTRL */
                        regVal = CSL_FMK( BCDMA_TXCCFG_CHAN_TPRI_CTRL_PRIORITY, pChanCfg->busPriority )    |
                                 CSL_FMK( BCDMA_TXCCFG_CHAN_TPRI_CTRL_ORDERID, pChanCfg->busOrderId );
                        CSL_REG32_WR( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TPRI_CTRL, regVal );
                        /* THREAD */
                        CSL_REG32_WR( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].THREAD, CSL_FMK(BCDMA_TXCCFG_CHAN_THREAD_ID, pChanCfg->threadId) );
                        /* TST_SCHED */
                        CSL_REG32_WR( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TST_SCHED, CSL_FMK(BCDMA_TXCCFG_CHAN_TST_SCHED_PRIORITY, pChanCfg->dmaPriority) );
                    }
                }
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
                {
                    CSL_BcdmaRxChanCfg *pChanCfg = (CSL_BcdmaRxChanCfg *)pOpData;
                    if( (pChanCfg->burstSize > CSL_BCDMA_CHAN_BURST_SIZE_64_BYTES)    ||     /* Split-rx supports 32, and 64-byte bursts */
                        (pChanCfg->busPriority > ((uint32_t)7U) )                     ||
                        (pChanCfg->dmaPriority > ((uint32_t)3U) )
                      )
                    {
                        retVal = CSL_EINVALID_PARAMS;
                    }
                    else
                    {
                        /* RCFG */
                        regVal = CSL_REG32_RD( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCFG );
                        CSL_FINS( regVal, BCDMA_RXCCFG_CHAN_RCFG_PAUSE_ON_ERR, pChanCfg->pauseOnError);
                        CSL_FINS( regVal, BCDMA_RXCCFG_CHAN_RCFG_BURST_SIZE, pChanCfg->burstSize );
#ifdef CSL_BCDMA_RXCCFG_CHAN_RCFG_IGNORE_LONG_MASK
                        CSL_FINS( regVal, BCDMA_RXCCFG_CHAN_RCFG_IGNORE_LONG, pChanCfg->bIgnoreLongPkts ? (uint32_t)1U : (uint32_t)0U );
#endif
                        CSL_REG32_WR( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCFG, regVal );
                        /* RPRI_CTRL */
                        regVal = CSL_FMK( BCDMA_RXCCFG_CHAN_RPRI_CTRL_PRIORITY, pChanCfg->busPriority )    |
                                 CSL_FMK( BCDMA_RXCCFG_CHAN_RPRI_CTRL_ORDERID, pChanCfg->busOrderId );
                        CSL_REG32_WR( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RPRI_CTRL, regVal );
                        /* THREAD */
                        CSL_REG32_WR( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].THREAD, CSL_FMK(BCDMA_RXCCFG_CHAN_THREAD_ID, pChanCfg->threadId) );
                        /* RST_SCHED */
                        CSL_REG32_WR( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RST_SCHED, CSL_FMK(BCDMA_RXCCFG_CHAN_RST_SCHED_PRIORITY, pChanCfg->dmaPriority) );
                    }
                }
                break;
            default:
                retVal = CSL_EBADARGS;
                break;
        }
    }
    return retVal;
}

static bool CSL_bcdmaChanOpIsChanEnabled( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx )
{
    uint32_t regVal;

    switch( chanType )
    {
        case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
            regVal = CSL_REG32_FEXT( &pCfg->pBcChanRtRegs->CHAN[chanIdx].CTL, BCDMA_BCRT_CHAN_CTL_EN );
            break;
        case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
            regVal = CSL_REG32_FEXT( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, BCDMA_TXCRT_CHAN_CTL_EN );
            break;
        case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
            regVal = CSL_REG32_FEXT( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, BCDMA_RXCRT_CHAN_CTL_EN );
            break;
        default:
            regVal = 0U;
            break;
    }
    return ((regVal == 1U) ? (bool)true : (bool)false);
}

static int32_t CSL_bcdmaChanOpSetChanEnable( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, bool bEnable )
{
    int32_t retVal = CSL_PASS;

    switch( chanType )
    {
        case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
            CSL_REG32_WR(&pCfg->pBcChanRtRegs->CHAN[chanIdx].CTL, CSL_FMK(BCDMA_BCRT_CHAN_CTL_EN,(bEnable ? (uint32_t)1U : (uint32_t)0U)) );
            break;
        case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
            CSL_REG32_WR(&pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, CSL_FMK(BCDMA_TXCRT_CHAN_CTL_EN,(bEnable ? (uint32_t)1U : (uint32_t)0U)) );
            break;
        case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
            CSL_REG32_WR(&pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, CSL_FMK(BCDMA_RXCRT_CHAN_CTL_EN,(bEnable ? (uint32_t)1U : (uint32_t)0U)) );
            break;
        default:
            retVal = CSL_EBADARGS;
            break;
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpSetChanPause( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, bool bPause )
{
    int32_t retVal = CSL_PASS;

    if( CSL_bcdmaChanOpIsChanEnabled( pCfg, chanType, chanIdx ) == (bool)false )
    {
        /* Channel must be enabled in order to be paused or resumed */
        retVal = CSL_EFAIL;
    }
    else
    {
        switch( chanType )
        {
            case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
                CSL_REG32_FINS( &pCfg->pBcChanRtRegs->CHAN[chanIdx].CTL, BCDMA_BCRT_CHAN_CTL_PAUSE, (bPause==(bool)false) ? (uint32_t)0U : (uint32_t)1U );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
                CSL_REG32_FINS( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, BCDMA_TXCRT_CHAN_CTL_PAUSE, (bPause==(bool)false) ? (uint32_t)0U : (uint32_t)1U );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
                CSL_REG32_FINS( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, BCDMA_RXCRT_CHAN_CTL_PAUSE, (bPause==(bool)false) ? (uint32_t)0U : (uint32_t)1U );
                break;
            default:
                retVal = CSL_EBADARGS;
                break;
        }
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpTeardownChan( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData )
{
    int32_t  retVal = CSL_PASS;

    if( CSL_bcdmaChanOpIsChanEnabled( pCfg, chanType, chanIdx ) == (bool)false )
    {
        /* Channel can be torn down only when it is enabled */
        retVal = CSL_EFAIL;
    }
    else
    {
        uint32_t regVal;
        uint32_t force = (uint32_t)0U, wait = (uint32_t)0U;

        if( pOpData != NULL )
        {
            CSL_BcdmaTeardownOpts *pTdOpts = (CSL_BcdmaTeardownOpts *)pOpData;
            force = pTdOpts->force;
            wait  = pTdOpts->wait;
        }
        switch( chanType )
        {
            default:
            case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
                regVal = CSL_REG32_RD( &pCfg->pBcChanRtRegs->CHAN[chanIdx].CTL );
                CSL_FINS( regVal, BCDMA_BCRT_CHAN_CTL_TDOWN, (uint32_t)1U );
                CSL_FINS( regVal, BCDMA_BCRT_CHAN_CTL_FTDOWN, (force==0U) ? (uint32_t)0U : (uint32_t)1U  );
                CSL_REG32_WR( &pCfg->pBcChanRtRegs->CHAN[chanIdx].CTL, regVal );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
                regVal = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL );
                CSL_FINS( regVal, BCDMA_TXCRT_CHAN_CTL_TDOWN, (uint32_t)1U );
                CSL_FINS( regVal, BCDMA_TXCRT_CHAN_CTL_FTDOWN, (force==0U) ? (uint32_t)0U : (uint32_t)1U  );
                CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, regVal );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
                regVal = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL );
                CSL_FINS( regVal, BCDMA_RXCRT_CHAN_CTL_TDOWN, (uint32_t)1U );
                CSL_FINS( regVal, BCDMA_RXCRT_CHAN_CTL_FTDOWN, (force==0U) ? (uint32_t)0U : (uint32_t)1U  );
                CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, regVal );
                break;
        }
        if(wait != 0U)
        {
            uint32_t retryCnt = CSL_BCDMA_TEARDOWN_COMPLETE_WAIT_MAX_CNT;
            /* The channel's enable bit is cleared once teardown is complete */
            while( (CSL_bcdmaChanOpIsChanEnabled( pCfg, chanType, chanIdx ) == (bool)true) && (retryCnt != (uint32_t)0U) )
            {
                retryCnt--;
            }
            if( retryCnt == (uint32_t)0U ) {
                retVal = CSL_EFAIL;
            }
        }
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpTriggerChan( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx )
{
    int32_t retVal = CSL_PASS;

    if( chanType == CSL_BCDMA_CHAN_TYPE_BLOCK_COPY )
    {
        CSL_REG32_WR(&pCfg->pBcChanRtRegs->CHAN[chanIdx].SWTRIG, CSL_FMK(BCDMA_BCRT_CHAN_SWTRIG_TRIGGER, (uint32_t)1U));
    }
    else if( chanType == CSL_BCDMA_CHAN_TYPE_SPLIT_TX )
    {
#ifdef CSL_BCDMA_TXCRT_CHAN_SWTRIG_TRIGGER_MASK
        CSL_REG32_WR(&pCfg->pTxChanRtRegs->CHAN[chanIdx].SWTRIG, CSL_FMK(BCDMA_TXCRT_CHAN_SWTRIG_TRIGGER, (uint32_t)1U));
#else
        retVal = CSL_EFAIL;
#endif
    }
    else if( chanType == CSL_BCDMA_CHAN_TYPE_SPLIT_RX )
    {
#ifdef CSL_BCDMA_RXCRT_CHAN_SWTRIG_TRIGGER_MASK
        CSL_REG32_WR(&pCfg->pRxChanRtRegs->CHAN[chanIdx].SWTRIG, CSL_FMK(BCDMA_RXCRT_CHAN_SWTRIG_TRIGGER, (uint32_t)1U));
#else
        retVal = CSL_EFAIL;
#endif
    }
    else
    {
        retVal = CSL_EFAIL;
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpGetChanRT( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData )
{
    int32_t retVal = CSL_PASS;

    if( pOpData == NULL )
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        uint32_t val;
        switch( chanType )
        {
            case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
                val = CSL_REG32_RD( &pCfg->pBcChanRtRegs->CHAN[chanIdx].CTL );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
                val = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
                val = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL );
                break;
            default:
                retVal = CSL_EBADARGS;
                break;
        }
        if( retVal == CSL_PASS )
        {
            CSL_BcdmaRT *pRT = (CSL_BcdmaRT *)pOpData;

            pRT->enable         = CSL_FEXT( val, BCDMA_TXCRT_CHAN_CTL_EN );
            pRT->teardown       = CSL_FEXT( val, BCDMA_TXCRT_CHAN_CTL_TDOWN );
            pRT->forcedTeardown = CSL_FEXT( val, BCDMA_TXCRT_CHAN_CTL_FTDOWN );
            pRT->pause          = CSL_FEXT( val, BCDMA_TXCRT_CHAN_CTL_PAUSE );
            pRT->error          = CSL_FEXT( val, BCDMA_TXCRT_CHAN_CTL_ERROR );
#ifdef CSL_BCDMA_RXCRT_CHAN_CTL_STARVATION_MASK
            if( chanType == CSL_BCDMA_CHAN_TYPE_SPLIT_RX )
            {
                pRT->starvation = CSL_FEXT( val, BCDMA_RXCRT_CHAN_CTL_STARVATION );
            }
            else
#endif
            {
                pRT->starvation = (uint32_t)0U;
            }
        }
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpSetChanRT( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData )
{
    int32_t retVal = CSL_PASS;

    if( pOpData == NULL )
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        uint32_t val;
        CSL_BcdmaRT *pRT = (CSL_BcdmaRT *)pOpData;

        val =   CSL_FMK(BCDMA_TXCRT_CHAN_CTL_EN,           pRT->enable)         |
                CSL_FMK(BCDMA_TXCRT_CHAN_CTL_TDOWN,        pRT->teardown)       |
                CSL_FMK(BCDMA_TXCRT_CHAN_CTL_FTDOWN,       pRT->forcedTeardown) |
                CSL_FMK(BCDMA_TXCRT_CHAN_CTL_PAUSE,        pRT->pause);
        switch( chanType )
        {
            case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
                CSL_REG32_WR( &pCfg->pBcChanRtRegs->CHAN[chanIdx].CTL, val );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
                CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, val );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
                CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, val );
                break;
            default:
                retVal = CSL_EBADARGS;
                break;
        }
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpGetChanStats( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData )
{
    int32_t retVal = CSL_PASS;

    if( pOpData == NULL )
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        CSL_BcdmaChanStats *pChanStats = (CSL_BcdmaChanStats *)pOpData;

        switch( chanType )
        {
            case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
                pChanStats->packetCnt        = CSL_REG32_RD( &pCfg->pBcChanRtRegs->CHAN[chanIdx].PCNT );
                pChanStats->txPayloadByteCnt = CSL_REG32_RD( &pCfg->pBcChanRtRegs->CHAN[chanIdx].BCNT );
                pChanStats->txStartedByteCnt = CSL_REG32_RD( &pCfg->pBcChanRtRegs->CHAN[chanIdx].SBCNT );
                pChanStats->rxPayloadByteCnt = (uint32_t)0U;
                pChanStats->rxStartedByteCnt = (uint32_t)0U;
                pChanStats->completedByteCnt = pChanStats->txPayloadByteCnt;
                pChanStats->startedByteCnt   = pChanStats->txStartedByteCnt;
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
                pChanStats->packetCnt        = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].PCNT );
                pChanStats->txPayloadByteCnt = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].BCNT );
                pChanStats->txStartedByteCnt = CSL_REG32_RD( &pCfg->pTxChanRtRegs->CHAN[chanIdx].SBCNT );
                pChanStats->rxPayloadByteCnt = (uint32_t)0U;
                pChanStats->rxStartedByteCnt = (uint32_t)0U;
                pChanStats->completedByteCnt = pChanStats->txPayloadByteCnt;
                pChanStats->startedByteCnt   = pChanStats->txStartedByteCnt;
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
                pChanStats->packetCnt        = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].PCNT );
                pChanStats->txPayloadByteCnt = (uint32_t)0U;
                pChanStats->txStartedByteCnt = (uint32_t)0U;
                pChanStats->rxPayloadByteCnt = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].BCNT );
                pChanStats->rxStartedByteCnt = CSL_REG32_RD( &pCfg->pRxChanRtRegs->CHAN[chanIdx].SBCNT );
                pChanStats->completedByteCnt = pChanStats->rxPayloadByteCnt;
                pChanStats->startedByteCnt   = pChanStats->rxStartedByteCnt;
                break;
            default:
                retVal = CSL_EBADARGS;
                break;
        }
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpDecChanStats( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData )
{
    int32_t retVal = CSL_PASS;

    if( pOpData == NULL )
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        CSL_BcdmaChanStats *pChanStats = (CSL_BcdmaChanStats *)pOpData;

        switch( chanType )
        {
            case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
                CSL_REG32_WR( &pCfg->pBcChanRtRegs->CHAN[chanIdx].PCNT,   pChanStats->packetCnt );
                CSL_REG32_WR( &pCfg->pBcChanRtRegs->CHAN[chanIdx].BCNT,   pChanStats->txPayloadByteCnt );
                CSL_REG32_WR( &pCfg->pBcChanRtRegs->CHAN[chanIdx].SBCNT,  pChanStats->txStartedByteCnt );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
                CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].PCNT,   pChanStats->packetCnt );
                CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].BCNT,   pChanStats->txPayloadByteCnt );
                CSL_REG32_WR( &pCfg->pTxChanRtRegs->CHAN[chanIdx].SBCNT,  pChanStats->txStartedByteCnt );
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
                CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].PCNT,   pChanStats->packetCnt );
                CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].BCNT,   pChanStats->rxPayloadByteCnt );
                CSL_REG32_WR( &pCfg->pRxChanRtRegs->CHAN[chanIdx].SBCNT,  pChanStats->rxStartedByteCnt );
                break;
            default:
                retVal = CSL_EBADARGS;
                break;
        }
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpAccessRemotePeerReg( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData, bool bRead )
{
    int32_t retVal = CSL_PASS;

    if( pOpData == NULL )
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        uint32_t *pRemotePeerReg = NULL;
        switch( chanType )
        {
            case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
                pRemotePeerReg = (uint32_t *)&pCfg->pTxChanRtRegs->CHAN[chanIdx].PEER0;
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
                pRemotePeerReg = (uint32_t *)&pCfg->pRxChanRtRegs->CHAN[chanIdx].PEER0;
                break;
            default:
                retVal = CSL_EBADARGS;
                break;
        }
        if( pRemotePeerReg != NULL )
        {
            CSL_BcdmaRemotePeerOpts *pPeerOpts = (CSL_BcdmaRemotePeerOpts *)pOpData;
            if( pPeerOpts->regIdx >= (uint32_t)16u )
            {
                retVal = CSL_EINVALID_PARAMS;
            }
            else
            {
                pRemotePeerReg += pPeerOpts->regIdx;    /* Increment to specified peer register */
                if( bRead == (bool)true )
                {
                    pPeerOpts->regVal = CSL_REG32_RD( pRemotePeerReg );
                }
                else
                {
                    CSL_REG32_WR( pRemotePeerReg, pPeerOpts->regVal );
                }
            }
        }
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpSetBurstSize( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData )
{
    int32_t retVal = CSL_PASS;

    if( pOpData == NULL )
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        CSL_BcdmaChanBurstSize burstSize = *(CSL_BcdmaChanBurstSize *)pOpData;
        switch( chanType )
        {
            case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
                if( burstSize > CSL_BCDMA_CHAN_BURST_SIZE_128_BYTES )   /* Block-copy supports 32, 64, and 128-byte bursts */
                {
                    retVal = CSL_EINVALID_PARAMS;
                }
                else
                {
                    CSL_REG32_FINS( &pCfg->pBcChanCfgRegs->CHAN[chanIdx].CFG, BCDMA_BCCFG_CHAN_CFG_BURST_SIZE, burstSize );
                }
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
                if( burstSize > CSL_BCDMA_CHAN_BURST_SIZE_64_BYTES )    /* Split-tx supports 32, and 64-byte bursts */
                {
                    retVal = CSL_EINVALID_PARAMS;
                }
                else
                {
                    CSL_REG32_FINS( &pCfg->pTxChanCfgRegs->CHAN[chanIdx].TCFG, BCDMA_TXCCFG_CHAN_TCFG_BURST_SIZE, burstSize );
                }
                break;
            case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
                if( burstSize > CSL_BCDMA_CHAN_BURST_SIZE_64_BYTES )    /* Split-rx supports 32, and 64-byte bursts */
                {
                    retVal = CSL_EINVALID_PARAMS;
                }
                else
                {
                    CSL_REG32_FINS( &pCfg->pRxChanCfgRegs->CHAN[chanIdx].RCFG, BCDMA_RXCCFG_CHAN_RCFG_BURST_SIZE, burstSize );
                }
                break;
            default:
                retVal = CSL_EBADARGS;
                break;
        }
    }
    return retVal;
}

static int32_t CSL_bcdmaChanOpClearError( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanType chanType, uint32_t chanIdx )
{
    int32_t retVal = CSL_PASS;

    switch( chanType )
    {
        case CSL_BCDMA_CHAN_TYPE_BLOCK_COPY:
            CSL_REG32_FINS( &pCfg->pBcChanRtRegs->CHAN[chanIdx].CTL, BCDMA_BCRT_CHAN_CTL_ERROR, (uint32_t)0U );
            break;
        case CSL_BCDMA_CHAN_TYPE_SPLIT_TX:
            CSL_REG32_FINS( &pCfg->pTxChanRtRegs->CHAN[chanIdx].CTL, BCDMA_TXCRT_CHAN_CTL_ERROR, (uint32_t)0U );
            break;
        case CSL_BCDMA_CHAN_TYPE_SPLIT_RX:
            CSL_REG32_FINS( &pCfg->pRxChanRtRegs->CHAN[chanIdx].CTL, BCDMA_RXCRT_CHAN_CTL_ERROR, (uint32_t)0U );
            break;
        default:
            retVal = CSL_EBADARGS;
            break;
    }
    return retVal;
}

/* ----------------------------------------------------------------------------
 *  Global API functions
 * ----------------------------------------------------------------------------
 */
int32_t CSL_bcdmaChanOp( CSL_BcdmaCfg *pCfg, CSL_BcdmaChanOp chanOp, CSL_BcdmaChanType chanType, uint32_t chanIdx, void *pOpData )
{
    int32_t retVal = CSL_PASS;

    if( ( pCfg == NULL )                                    ||
        ( chanType > CSL_BCDMA_CHAN_TYPE_SPLIT_RX )         ||
        ( !CSL_bcdmaChanOpIsValidChanIdx( pCfg, chanType, chanIdx ) )
      )
    {
        retVal = CSL_EBADARGS;
    }
    else
    {
        switch(chanOp )
        {
            case CSL_BCDMA_CHAN_OP_CONFIG:
                retVal = CSL_bcdmaChanOpCfgChan( pCfg, chanType, chanIdx, pOpData );
                break;
            case CSL_BCDMA_CHAN_OP_ENABLE:
                retVal = CSL_bcdmaChanOpSetChanEnable( pCfg, chanType, chanIdx, (bool)true );
                break;
            case CSL_BCDMA_CHAN_OP_DISABLE:
                retVal = CSL_bcdmaChanOpSetChanEnable( pCfg, chanType, chanIdx, (bool)false );
                break;
            case CSL_BCDMA_CHAN_OP_PAUSE:
                retVal = CSL_bcdmaChanOpSetChanPause( pCfg, chanType, chanIdx, (bool)true );
                break;
            case CSL_BCDMA_CHAN_OP_RESUME:
                retVal = CSL_bcdmaChanOpSetChanPause( pCfg, chanType, chanIdx, (bool)false );
                break;
            case CSL_BCDMA_CHAN_OP_TEARDOWN:
                retVal = CSL_bcdmaChanOpTeardownChan( pCfg, chanType, chanIdx, pOpData );
                break;
            case CSL_BCDMA_CHAN_OP_TRIGGER:
                retVal = CSL_bcdmaChanOpTriggerChan( pCfg, chanType, chanIdx );
                break;
            case CSL_BCDMA_CHAN_OP_GET_RT:
                retVal = CSL_bcdmaChanOpGetChanRT( pCfg, chanType, chanIdx, pOpData );
                break;
            case CSL_BCDMA_CHAN_OP_SET_RT:
                retVal = CSL_bcdmaChanOpSetChanRT( pCfg, chanType, chanIdx, pOpData );
                break;
            case CSL_BCDMA_CHAN_OP_GET_STATS:
                retVal = CSL_bcdmaChanOpGetChanStats( pCfg, chanType, chanIdx, pOpData );
                break;
            case CSL_BCDMA_CHAN_OP_DEC_STATS:
                retVal = CSL_bcdmaChanOpDecChanStats( pCfg, chanType, chanIdx, pOpData );
                break;
            case CSL_BCDMA_CHAN_OP_GET_REMOTE_PEER_REG:
                retVal = CSL_bcdmaChanOpAccessRemotePeerReg( pCfg, chanType, chanIdx, pOpData, (bool)true );
                break;
            case CSL_BCDMA_CHAN_OP_SET_REMOTE_PEER_REG:
                retVal = CSL_bcdmaChanOpAccessRemotePeerReg( pCfg, chanType, chanIdx, pOpData, (bool)false );
                break;
            case CSL_BCDMA_CHAN_OP_SET_BURST_SIZE:
                retVal = CSL_bcdmaChanOpSetBurstSize( pCfg, chanType, chanIdx, pOpData );
                break;
            case CSL_BCDMA_CHAN_OP_CLEAR_ERROR:
                retVal = CSL_bcdmaChanOpClearError( pCfg, chanType, chanIdx );
                break;
            default:
                retVal = CSL_EBADARGS;
                break;
        }
    }
    return retVal;
}

uint32_t CSL_bcdmaGetRevision( const CSL_BcdmaCfg *pCfg )
{
    uint32_t retVal = 0U;

    if( (pCfg != NULL) && (pCfg->pGenCfgRegs != NULL) )
    {
       retVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->REVISION );
    }
    return retVal;
}

int32_t CSL_bcdmaGetRevisionInfo( const CSL_BcdmaCfg *pCfg, CSL_BcdmaRevision *pRev )
{
    int32_t retVal = CSL_PASS;

    if( (pCfg == NULL) || (pCfg->pGenCfgRegs == NULL) || (pRev == NULL) )
    {
        retVal = CSL_EFAIL;
    }
    else
    {
        uint32_t val;

        val = CSL_REG32_RD( &pCfg->pGenCfgRegs->REVISION );
        pRev->modId     = CSL_FEXT( val, BCDMA_GCFG_REVISION_MODID );
        pRev->revRtl    = CSL_FEXT( val, BCDMA_GCFG_REVISION_REVRTL );
        pRev->revMajor  = CSL_FEXT( val, BCDMA_GCFG_REVISION_REVMAJ );
        pRev->custom    = CSL_FEXT( val, BCDMA_GCFG_REVISION_CUSTOM );
        pRev->revMinor  = CSL_FEXT( val, BCDMA_GCFG_REVISION_REVMIN );
    }
    return retVal;
}

void CSL_bcdmaInitCfg( CSL_BcdmaCfg *pCfg )
{
    if( pCfg != NULL )
    {
        memset( (void *)pCfg, 0, sizeof(CSL_BcdmaCfg) );
    }
}

void CSL_bcdmaGetCfg( CSL_BcdmaCfg *pCfg )
{
    if( ! ((pCfg == NULL) || (pCfg->pGenCfgRegs == NULL)) )
    {
        uint32_t regVal;

        pCfg->cap0 = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP0 );
        pCfg->cap1 = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP1 );

        regVal = CSL_REG32_RD( &pCfg->pGenCfgRegs->CAP2 );
        pCfg->bcChanCnt = CSL_FEXT( regVal, BCDMA_GCFG_CAP2_CHAN_CNT );
        pCfg->splitTxChanCnt = CSL_FEXT( regVal, BCDMA_GCFG_CAP2_TCHAN_CNT );
        pCfg->splitRxChanCnt = CSL_FEXT( regVal, BCDMA_GCFG_CAP2_RCHAN_CNT );
        pCfg->flowCnt = (uint32_t)0U;

        /* Assign values to UDMAP elements for backwards-compatibility */
        pCfg->txChanCnt                     = pCfg->bcChanCnt + pCfg->splitTxChanCnt;
        pCfg->rxChanCnt                     = pCfg->splitRxChanCnt;
        pCfg->rxFlowCnt                     = pCfg->flowCnt;
        pCfg->txExtUtcChanCnt               = (uint32_t)0U;
        pCfg->txHighCapacityChanCnt         = (uint32_t)0U;
        pCfg->txUltraHighCapacityChanCnt    = (uint32_t)0U;
    }
}

void CSL_bcdmaInitTxChanCfg( CSL_BcdmaTxChanCfg *pTxChanCfg )
{
    /*-------------------------------------------------------------------------
     *  Start by initializing all structure members to 0
     *-----------------------------------------------------------------------*/
    memset( (void *)pTxChanCfg, 0, sizeof(CSL_BcdmaTxChanCfg) );
    /*-------------------------------------------------------------------------
     *  Now initialize non-zero structure members
     *-----------------------------------------------------------------------*/
    pTxChanCfg->chanType        = CSL_BCDMA_CHAN_TYPE_REF_PKT_RING;
    pTxChanCfg->fetchWordSize   = CSL_BCDMA_FETCH_WORD_SIZE_16;
    pTxChanCfg->trEventNum      = CSL_BCDMA_NO_EVENT;
    pTxChanCfg->errEventNum     = CSL_BCDMA_NO_EVENT;
}

void CSL_bcdmaInitRxChanCfg( CSL_BcdmaRxChanCfg *pRxChanCfg )
{
    /*-------------------------------------------------------------------------
     *  Start by initializing all structure members to 0
     *-----------------------------------------------------------------------*/
    memset( (void *)pRxChanCfg, 0, sizeof(CSL_BcdmaRxChanCfg) );
    /*-------------------------------------------------------------------------
     *  Now initialize non-zero structure members
     *-----------------------------------------------------------------------*/
    pRxChanCfg->chanType            = CSL_BCDMA_CHAN_TYPE_REF_PKT_RING;
    pRxChanCfg->fetchWordSize       = CSL_BCDMA_FETCH_WORD_SIZE_16;
    pRxChanCfg->trEventNum          = CSL_BCDMA_NO_EVENT;
    pRxChanCfg->errEventNum         = CSL_BCDMA_NO_EVENT;
}

int32_t CSL_bcdmaTxChanCfg( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, const CSL_BcdmaTxChanCfg *pTxChanCfg )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_CONFIG, chanIdx, (void *)pTxChanCfg );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaRxChanCfg( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, const CSL_BcdmaRxChanCfg *pRxChanCfg )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_CONFIG, chanIdx, (void *)pRxChanCfg );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaEnableTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_ENABLE, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaEnableRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_ENABLE, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaGetTxRT( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaRT *pRT )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_GET_RT, chanIdx, (void *)pRT );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaGetRxRT( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaRT *pRT )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_GET_RT, chanIdx, (void *)pRT );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaSetTxRT( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, const CSL_BcdmaRT *pRT )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_SET_RT, chanIdx, (void *)pRT );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaSetRxRT( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, const CSL_BcdmaRT *pRT )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_SET_RT, chanIdx, (void *)pRT );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaDisableTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_DISABLE, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaDisableRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_DISABLE, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaTeardownTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait )
{
    int32_t retVal;
    CSL_BcdmaTeardownOpts   teardownOpts;

    teardownOpts.force  = (bForce == (bool)false) ? (uint32_t)0U : (uint32_t)1U;
    teardownOpts.wait   = (bWait  == (bool)false) ? (uint32_t)0U : (uint32_t)1U;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_TEARDOWN, chanIdx, (void *)&teardownOpts );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaTeardownRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, bool bForce, bool bWait )
{
    int32_t retVal;
    CSL_BcdmaTeardownOpts   teardownOpts;

    teardownOpts.force  = (bForce == (bool)false) ? (uint32_t)0U : (uint32_t)1U;
    teardownOpts.wait   = (bWait  == (bool)false) ? (uint32_t)0U : (uint32_t)1U;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_TEARDOWN, chanIdx, (void *)&teardownOpts );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaPauseTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_PAUSE, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaPauseRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_PAUSE, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaUnpauseTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_RESUME, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaUnpauseRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_RESUME, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaTriggerTxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_TRIGGER, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaTriggerRxChan( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    int32_t retVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_TRIGGER, chanIdx, NULL );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

void CSL_bcdmaGetChanStats( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir, CSL_BcdmaChanStats *pChanStats )
{
    CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_GET_STATS, chanIdx, (void *)pChanStats );
}

void CSL_bcdmaDecChanStats( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir, const CSL_BcdmaChanStats *pChanStats )
{
    CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_DEC_STATS, chanIdx, (void *)pChanStats );
}

int32_t CSL_bcdmaGetChanPeerReg( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir, uint32_t regIdx, uint32_t *pVal )
{
    int32_t retVal;
    CSL_BcdmaRemotePeerOpts remotePeerOpts;

    remotePeerOpts.regIdx = regIdx;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_GET_REMOTE_PEER_REG, chanIdx, (void *)&remotePeerOpts );
    if( retVal == CSL_PASS )
    {
        *pVal = remotePeerOpts.regVal;
    }
    else
    {
        *pVal = (uint32_t)0U;
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaSetChanPeerReg( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir, uint32_t regIdx, uint32_t *pVal )
{
    int32_t retVal;
    CSL_BcdmaRemotePeerOpts remotePeerOpts;

    remotePeerOpts.regIdx = regIdx;
    remotePeerOpts.regVal = *pVal;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_SET_REMOTE_PEER_REG, chanIdx, (void *)&remotePeerOpts );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaTxChanSetBurstSize( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanBurstSize burstSize )
{
    int32_t retVal;
    CSL_BcdmaChanBurstSize parm = burstSize;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_SET_BURST_SIZE, chanIdx, (void *)&parm );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

int32_t CSL_bcdmaRxChanSetBurstSize( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanBurstSize burstSize )
{
    int32_t retVal;
    CSL_BcdmaChanBurstSize parm = burstSize;
    retVal = CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_SET_BURST_SIZE, chanIdx, (void *)&parm );
    if( retVal != CSL_PASS )
    {
        retVal = CSL_EFAIL;     /* API returns CSL_EFAIL on failure for backwards compatibility with udmap API */
    }
    return retVal;
}

void CSL_bcdmaClearTxChanError( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_CLEAR_ERROR, chanIdx, NULL );
}

void CSL_bcdmaClearRxChanError( CSL_BcdmaCfg *pCfg, uint32_t chanIdx )
{
    CSL_bcdmaDoChanOp( pCfg, CSL_BCDMA_CHAN_OP_CLEAR_ERROR, chanIdx, NULL );
}

void CSL_bcdmaInitRxFlowCfg( CSL_BcdmaRxFlowCfg *pFlow )
{
    if( pFlow != NULL )
    {
        memset( (void *)pFlow, 0, sizeof(CSL_BcdmaRxFlowCfg) );
    }
}

void CSL_bcdmaSetPerfCtrl( CSL_BcdmaCfg *pCfg, uint32_t rxRetryTimeoutCnt )
{
    uint32_t regVal;

    regVal = CSL_FMK( BCDMA_GCFG_PERF_CTRL_TIMEOUT_CNT, rxRetryTimeoutCnt );
    CSL_REG32_WR( &pCfg->pGenCfgRegs->PERF_CTRL, regVal );
}

void CSL_bcdmaSetUtcCtrl( CSL_BcdmaCfg *pCfg, uint32_t startingThreadNum )
{
}

int32_t CSL_bcdmaRxFlowCfg(CSL_BcdmaCfg *pCfg, uint32_t flow, const CSL_BcdmaRxFlowCfg *pFlow )
{
    return CSL_EFAIL;
}

int32_t CSL_bcdmaRxChanSetTrEvent( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum )
{
    return CSL_EFAIL;
}

int32_t CSL_bcdmaTxChanSetTrEvent( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, uint32_t trEventNum )
{
    return CSL_EFAIL;
}

void CSL_bcdmaCfgRxFlowIdFirewall( CSL_BcdmaCfg *pCfg, uint32_t outEvtNum )
{
}

bool CSL_bcdmaGetRxFlowIdFirewallStatus( CSL_BcdmaCfg *pCfg, CSL_BcdmaRxFlowIdFirewallStatus *pRxFlowIdFwStatus )
{
    return (bool)false;
}

int32_t CSL_bcdmaEnableLink( CSL_BcdmaCfg *pCfg, uint32_t chanIdx, CSL_BcdmaChanDir chanDir )
{
    int32_t retVal = CSL_EFAIL;
    uint32_t peerEnableRegVal;

    peerEnableRegVal = (uint32_t)1U << 31;
    if( chanDir == CSL_BCDMA_CHAN_DIR_TX )
    {
        /* a. Set BCDMA peer real-time enable by calling the CSL_bcdmaSetChanPeerReg() function */
        if( CSL_bcdmaSetChanPeerReg( pCfg, chanIdx, chanDir, CSL_BCDMA_CHAN_PEER_REG_OFFSET_ENABLE, &peerEnableRegVal ) == 0 )
        {
            /* b. Enable the BCDMA tx channel by calling the CSL_bcdmaEnableTxChan() function */
            if( CSL_bcdmaEnableTxChan( pCfg, chanIdx ) == 0 )
            {
                retVal = 0;
            }
        }
    }
    if( chanDir == CSL_BCDMA_CHAN_DIR_RX )
    {
        /* a. Enable the BCDMA rx channel by calling the CSL_bcdmaEnableRxChan() function */
        if( CSL_bcdmaEnableRxChan( pCfg, chanIdx ) == 0 )
        {
            /* b. Set BCDMA peer real-time enable by calling the CSL_bcdmaSetChanPeerReg() function */
            if( CSL_bcdmaSetChanPeerReg( pCfg, chanIdx, chanDir, CSL_BCDMA_CHAN_PEER_REG_OFFSET_ENABLE, &peerEnableRegVal ) == 0 )
            {
                retVal = 0;
            }
        }
    }
    return retVal;
}
