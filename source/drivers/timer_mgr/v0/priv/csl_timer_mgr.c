/**
 * @file  csl_timer_mgr.c
 *
 * @brief
 *  Implementation file for the timer_mgr module CSL.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2017-2018, Texas Instruments, Inc.
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
 *    Neither the name of Texas Instruments Incorpotimermgred nor the names of
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
#include <drivers/timer_mgr.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>

static bool CSL_timermgrIsEnabled( const CSL_TimermgrCfg *pCfg );
static inline uint32_t CSL_timermgrGetPageNum(uint32_t tnum);
static inline uint32_t CSL_timermgrGetEntryNum(uint32_t tnum);

/*=============================================================================
 *  Inline functions
 *===========================================================================*/
static inline uint32_t CSL_timermgrGetPageNum(uint32_t tnum)
{
    return ((tnum)>>4U);
}

static inline uint32_t CSL_timermgrGetEntryNum(uint32_t tnum)
{
    return ((tnum)&0xFU);
}

/*=============================================================================
 *  Internal functions
 *===========================================================================*/
static bool CSL_timermgrIsEnabled( const CSL_TimermgrCfg *pCfg )
{
    bool retVal;
    if( CSL_REG32_FEXT( &pCfg->pCfgRegs->CNTL, TIMER_MGR_CNTL_ENABLE ) == 0U )
    {
        retVal = (bool)false;
    }
    else
    {
        retVal = (bool)true;
    }
    return retVal;
}

/*=============================================================================
 *  CSL-FL functions
 *===========================================================================*/
uint32_t CSL_timermgrGetRevision( const CSL_TimermgrCfg *pCfg )
{
    return CSL_REG32_RD( &pCfg->pCfgRegs->PID );
}

uint32_t CSL_timermgrGetMaxTimers( const CSL_TimermgrCfg *pCfg )
{
    return CSL_REG32_FEXT( &pCfg->pCfgRegs->CNTL, TIMER_MGR_CNTL_MAX_TIMER ) + 1U;
}

int32_t CSL_timermgrSetMaxTimers( CSL_TimermgrCfg *pCfg, uint32_t maxTimers )
{
    int32_t retVal;

    if( (!CSL_timermgrIsEnabled(pCfg)) && (maxTimers <= CSL_timermgrGetMaxTimers(pCfg)) )
    {
        CSL_REG32_FINS( &pCfg->pCfgRegs->CNTL, TIMER_MGR_CNTL_MAX_TIMER, (maxTimers-1U) );
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

int32_t CSL_timermgrSetTimerCnt( CSL_TimermgrCfg *pCfg, uint32_t timerNum, uint32_t timerCnt )
{
    int32_t retVal;

    if( timerNum < CSL_timermgrGetMaxTimers(pCfg) )
    {
        CSL_REG32_WR( &pCfg->pTimerRegs->PAGE[CSL_timermgrGetPageNum(timerNum)].ENTRY[CSL_timermgrGetEntryNum(timerNum)].SETUP, timerCnt );
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

int32_t CSL_timermgrSetTimerEnable( CSL_TimermgrCfg *pCfg, uint32_t timerNum, bool bEnable )
{
    int32_t retVal;

    if( timerNum < CSL_timermgrGetMaxTimers(pCfg) )
    {
        CSL_REG32_FINS( &pCfg->pTimerRegs->PAGE[CSL_timermgrGetPageNum(timerNum)].ENTRY[CSL_timermgrGetEntryNum(timerNum)].CONTROL, TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_ENABLE, (bEnable ? 1U : 0U) );
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

int32_t CSL_timermgrEnableAllTimers( CSL_TimermgrCfg *pCfg )
{
    int32_t retVal;

    if( !CSL_timermgrIsEnabled(pCfg) )
    {
        CSL_REG32_FINS( &pCfg->pCfgRegs->CNTL, TIMER_MGR_CNTL_MASS_ENABLE, 1 );
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

void CSL_timermgrSetEnable( CSL_TimermgrCfg *pCfg, bool bEnable )
{
   CSL_REG32_FINS( &pCfg->pCfgRegs->CNTL, TIMER_MGR_CNTL_ENABLE, (bEnable ? 1U : 0U) );
}

int32_t CSL_timermgrIsTimerExpired( CSL_TimermgrCfg *pCfg, uint32_t timerNum )
{
    int32_t retVal;

    if( (timerNum < CSL_timermgrGetMaxTimers(pCfg)) &&
        ((timerNum/32U) < (sizeof(pCfg->pCfgRegs->STATUS)/sizeof(uint32_t))) )
    {
        if( ( CSL_REG32_RD( &pCfg->pCfgRegs->STATUS[timerNum/32U] ) & ((uint32_t)1U << (timerNum % 32U)) ) == 0U )
        {
            retVal = 0;
        }
        else
        {
            retVal = 1;
        }
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

uint32_t CSL_timermgrGetExpiredTimers( CSL_TimermgrCfg *pCfg, uint32_t *pExpiredTimerNums )
{
    uint32_t numExpiredTimers;

    numExpiredTimers = CSL_REG32_FEXT( &pCfg->pCfgRegs->TIMEOUT_STATUS0, TIMER_MGR_TIMEOUT_STATUS0_NUM_EXPIRED_TIMERS );
    if( numExpiredTimers > 0U )
    {
        uint32_t regVal, bank, i, expiredTimerNumIdx = 0;

        regVal = CSL_REG32_RD( &pCfg->pCfgRegs->TIMEOUT_STATUS_BANK0 );
        for( bank=0; bank<32U; bank++ )
        {
            if( (regVal & ((uint32_t)1U<<bank)) != 0U )
            {
                regVal = CSL_REG32_RD( &pCfg->pCfgRegs->STATUS[bank] );
                for( i=0; i<32U; i++ )
                {
                    if( (regVal & ((uint32_t)1U<<i)) != 0U )
                    {
                        pExpiredTimerNums[expiredTimerNumIdx] = (bank * 32U) + i;
                        expiredTimerNumIdx++;
                    }
                }
            }
        }
        if( CSL_timermgrGetMaxTimers(pCfg) > (1024U) )
        {
            regVal = CSL_REG32_RD( &pCfg->pCfgRegs->TIMEOUT_STATUS_BANK1 );
            for( bank=0; bank<32U; bank++ )
            {
                if( (regVal & ((uint32_t)1U<<bank))!= 0U )
                {
                    regVal = CSL_REG32_RD( &pCfg->pCfgRegs->STATUS[bank+32U] );
                    for( i=0; i<32U; i++ )
                    {
                        if( (regVal & ((uint32_t)1U<<i)) != 0U )
                        {
                            pExpiredTimerNums[expiredTimerNumIdx] = ((bank+32U) * 32U) + i;
                            expiredTimerNumIdx++;
                        }
                    }
                }
            }
        }
    }
    return numExpiredTimers;
}

int32_t CSL_timermgrTouchTimer( CSL_TimermgrCfg *pCfg, uint32_t timerNum )
{
    int32_t retVal;

    if( timerNum < CSL_timermgrGetMaxTimers(pCfg) )
    {
        CSL_REG32_WR( &pCfg->pTimerRegs->PAGE[CSL_timermgrGetPageNum(timerNum)].ENTRY[CSL_timermgrGetEntryNum(timerNum)].CONTROL,
            CSL_FMK( TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_SET, 1U )   |
            CSL_FMK( TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_ENABLE, 1U ) );
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

uint32_t CSL_timermgrGetCurrCnt( const CSL_TimermgrCfg *pCfg )
{
    return CSL_REG32_RD( &pCfg->pCfgRegs->COUNTER );
}

int32_t CSL_timermgrSetEventIdx( CSL_TimermgrCfg *pCfg, uint32_t timerNum, uint32_t eventIdx )
{
    int32_t retVal;

    if( timerNum < CSL_timermgrGetMaxTimers(pCfg) )
    {
        CSL_REG32_WR( &pCfg->pOesRegs->EVENTIDX[timerNum],
            CSL_FMK( TIMER_MGR_OES_EVENTIDX_VAL, eventIdx ) );
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

int32_t CSL_timermgrGetEventIdx( CSL_TimermgrCfg *pCfg, uint32_t timerNum )
{
    int32_t retVal;

    if( timerNum < CSL_timermgrGetMaxTimers(pCfg) )
    {
        retVal = (int32_t)CSL_REG32_FEXT( &pCfg->pOesRegs->EVENTIDX[timerNum], TIMER_MGR_OES_EVENTIDX_VAL );
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

int32_t CSL_timermgrEnableAutoReset( CSL_TimermgrCfg *pCfg, uint32_t timerNum, bool bEnable )
{
    int32_t retVal = -2;    /* Default is to assume this timermgr feature is not available */

#ifdef CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_AUTORESET_MASK
    if( timerNum < CSL_timermgrGetMaxTimers(pCfg) )
    {
        CSL_REG32_FINS( &pCfg->pTimerRegs->PAGE[CSL_timermgrGetPageNum(timerNum)].ENTRY[CSL_timermgrGetEntryNum(timerNum)].CONTROL, TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_AUTORESET, (bEnable ? 1U : 0U) );
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
#endif
    return retVal;
}
