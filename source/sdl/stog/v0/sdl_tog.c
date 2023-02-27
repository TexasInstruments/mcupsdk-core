/**
 * @file  sdl_tog.c
 *
 * @brief
 *  Implementation file for the VBUSM Timeout Gasket module SDL.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2021-2023, Texas Instruments, Inc.
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
#include "sdlr_tog.h"
#include <sdl/include/sdl_types.h>

 /**
 * Design: PROC_SDL-1168
 */
int32_t SDL_TOG_init(SDL_TOG_Inst instance, const SDL_TOG_config *pConfig)
{
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if((instance <= SDL_TOG_MAX_INSTANCE) && (pConfig != NULL_PTR))
    {
        sdlResult = SDL_PASS;

        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        if((pConfig->cfgCtrl & SDL_TOG_CFG_TIMEOUT) != 0U)
        {
            sdlResult = SDL_TOG_setTimeoutVal(baseAddr, pConfig->timeoutVal );
        }
        if((pConfig->cfgCtrl & SDL_TOG_CFG_INTR_PENDING) != 0U)
        {
            sdlResult = SDL_TOG_setIntrPending(baseAddr, pConfig->intrSrcs );
        }
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1158,PROC_SDL-1159
 */
int32_t SDL_TOG_verifyConfig(SDL_TOG_Inst instance, const SDL_TOG_config *pConfig)
{
    SDL_TOG_config cfg;
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( (instance <= SDL_TOG_MAX_INSTANCE) && (pConfig != NULL_PTR))
    {
        sdlResult = SDL_PASS;
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        if(( pConfig->cfgCtrl & SDL_TOG_CFG_TIMEOUT) != 0U)
        {
            cfg.timeoutVal = SDL_REG32_RD( baseAddr + SDL_TOG_TIMEOUT );
            cfg.timeoutVal = SDL_FEXT(cfg.timeoutVal, TOG_TIMEOUT_TO);
            if(pConfig->timeoutVal != cfg.timeoutVal)
            {
                sdlResult = SDL_EFAIL;
            }
        }
        if(( pConfig->cfgCtrl & SDL_TOG_CFG_INTR_PENDING) != 0U)
        {
            cfg.intrSrcs   = SDL_REG32_RD( baseAddr + SDL_TOG_ERR );
            if(pConfig->intrSrcs != cfg.intrSrcs)
            {
                sdlResult = SDL_EFAIL;
            }
        }
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1168
 */
int32_t SDL_TOG_setIntrEnable( SDL_TOG_Inst instance, SDL_TOG_IntrSrc intrSrcs, bool enable )
{
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( (instance <= SDL_TOG_MAX_INSTANCE) && (intrSrcs > 0U) && (intrSrcs <= SDL_TOG_INTRSRC_ALL) )
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        if( enable == (bool)true )
        {
            SDL_REG32_WR( baseAddr + SDL_TOG_ERR_MSK_SET, intrSrcs );
        }
        else
        {
            SDL_REG32_WR( baseAddr + SDL_TOG_ERR_MSK_CLR, intrSrcs );
        }
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1506
 */
int32_t SDL_TOG_clrIntrPending  (SDL_TOG_Inst instance, SDL_TOG_IntrSrc intrSrc)
{
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( (instance <= SDL_TOG_MAX_INSTANCE) && (intrSrc > 0U) && (intrSrc <= SDL_TOG_INTRSRC_ALL) )
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        SDL_REG32_WR( baseAddr + SDL_TOG_ERR, intrSrc );
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1171
 */
int32_t SDL_TOG_getIntrPending  (SDL_TOG_Inst instance, SDL_TOG_IntrSrc *pPendInts)
{
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( (instance <= SDL_TOG_MAX_INSTANCE) && (pPendInts != NULL_PTR) )
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        *pPendInts = SDL_REG32_RD( baseAddr + SDL_TOG_ERR );
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1505
 */
int32_t SDL_TOG_ackIntr(SDL_TOG_Inst instance, SDL_TOG_IntrSrc intrSrc, uint32_t ackCnt)
{
    int32_t  sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( (instance   <=   SDL_TOG_MAX_INSTANCE)   &&
        (ackCnt     >    0U)                     &&
        (intrSrc    <=  SDL_TOG_INTRSRC_ALL) )
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        uint32_t pendingIntrCnt = 0U;

        if( intrSrc == SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT )
        {
            (void)SDL_TOG_getIntrCountInternal( baseAddr, intrSrc, &pendingIntrCnt );
            if( ackCnt <= pendingIntrCnt )
            {
                SDL_REG32_WR( baseAddr + SDL_TOG_ERR_TM_INFO, SDL_FMK( TOG_ERR_TM_INFO_CNT, ackCnt ) );
                sdlResult = SDL_PASS;
            }
        }
        else if( intrSrc == SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE )
        {
            (void)SDL_TOG_getIntrCountInternal( baseAddr, intrSrc, &pendingIntrCnt );
            if( ackCnt <= pendingIntrCnt )
            {
                SDL_REG32_WR( baseAddr + SDL_TOG_ERR_UN_INFO, SDL_FMK( TOG_ERR_UN_INFO_CNT, ackCnt ) );
                sdlResult = SDL_PASS;
            }
        }
        else
        {
            sdlResult = SDL_EFAIL;
        }
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1507
 */
int32_t SDL_TOG_start(SDL_TOG_Inst instance)
{
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( instance <= SDL_TOG_MAX_INSTANCE )
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        SDL_REG32_WR( (baseAddr + SDL_TOG_ENABLE), SDL_FMK( TOG_ENABLE_EN, SDL_TOG_ENABLE_KEY ) );
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1508
 */
int32_t SDL_TOG_stop(SDL_TOG_Inst instance)
{
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( instance <= SDL_TOG_MAX_INSTANCE )
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        SDL_REG32_WR( (baseAddr + SDL_TOG_ENABLE), SDL_FMK( TOG_ENABLE_EN, 0U ) );
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

/**
 * Design: PROC_SDL-1509
 */
int32_t SDL_TOG_reset(SDL_TOG_Inst instance)
{
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( instance <= SDL_TOG_MAX_INSTANCE )
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        /* Stop timer */
        SDL_REG32_WR( (baseAddr + SDL_TOG_ENABLE), SDL_FMK( TOG_ENABLE_EN, 0U ) );
        /* Reset timer counter and eon to 0 */
        SDL_REG32_FINS( (baseAddr + SDL_TOG_TIMER), TOG_TIMER_CNTR, 0U );
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1169
 */
int32_t SDL_TOG_getErrInfo(SDL_TOG_Inst instance, SDL_TOG_errInfo *pErrInfo)
{
    int32_t  sdlResult = SDL_EFAIL;
    uint32_t baseAddr;

    if( (instance > SDL_TOG_MAX_INSTANCE) || (pErrInfo == NULL_PTR) )
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        uint32_t regVal = SDL_REG32_RD( baseAddr + SDL_TOG_ERR_VAL );

        if( SDL_FEXT( regVal, TOG_ERR_VAL_VAL ) == 1U )
        {
            uint32_t regVal2;

            pErrInfo->routeId   = SDL_FEXT( regVal, TOG_ERR_VAL_RID );
            pErrInfo->orderId   = SDL_FEXT( regVal, TOG_ERR_VAL_OID );
            pErrInfo->dir   = SDL_FEXT( regVal, TOG_ERR_VAL_DIR );
            pErrInfo->type  = SDL_FEXT( regVal, TOG_ERR_VAL_TYP );

            regVal = SDL_REG32_RD( baseAddr + SDL_TOG_ERR_TAG );
            pErrInfo->tag   = SDL_FEXT( regVal, TOG_ERR_TAG_TAG );
            pErrInfo->commandId   = SDL_FEXT( regVal, TOG_ERR_TAG_CID );

            regVal = SDL_REG32_RD( baseAddr + SDL_TOG_ERR_BYT );
            pErrInfo->orgByteCnt    = SDL_FEXT( regVal, TOG_ERR_BYT_OBYTECNT );
            pErrInfo->currByteCnt   = SDL_FEXT( regVal, TOG_ERR_BYT_CBYTECNT );

            regVal  = SDL_REG32_RD( baseAddr + SDL_TOG_ERR_ADDR_L );
            regVal2 = SDL_REG32_RD( baseAddr + SDL_TOG_ERR_ADDR_U );
            pErrInfo->address = (((uint64_t)regVal2) << 32U) | ((uint64_t)regVal);
            sdlResult = SDL_PASS;
        }
    }
    return sdlResult;

}

/**
 * Design: PROC_SDL-1170
 */
int32_t SDL_TOG_getStaticRegisters  (SDL_TOG_Inst instance, SDL_TOG_staticRegs  *pStaticRegs)
{
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( (instance <= SDL_TOG_MAX_INSTANCE) && (pStaticRegs != NULL_PTR) )
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        pStaticRegs->PID        =   SDL_REG32_RD(baseAddr + SDL_TOG_PID);
        pStaticRegs->CFG        =   SDL_REG32_RD(baseAddr + SDL_TOG_CFG);
        pStaticRegs->ENABLE     =   SDL_REG32_RD(baseAddr + SDL_TOG_ENABLE);
        pStaticRegs->FLUSH      =   SDL_REG32_RD(baseAddr + SDL_TOG_FLUSH);
        pStaticRegs->TIMEOUT    =   SDL_REG32_RD(baseAddr + SDL_TOG_TIMEOUT);
        pStaticRegs->ERR        =   SDL_REG32_RD(baseAddr + SDL_TOG_ERR);

        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1510
 */
int32_t SDL_TOG_setFlushMode (SDL_TOG_Inst instance, bool enable)
{
    int32_t sdlResult = SDL_EBADARGS;
    uint32_t baseAddr;

    if( instance <= SDL_TOG_MAX_INSTANCE )
    {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        if( enable == (bool)true )
        {
            SDL_REG32_WR(  baseAddr + SDL_TOG_FLUSH, SDL_FMK( TOG_FLUSH_FL, SDL_TOG_FLUSH_MODE_KEY ) );
        }
        else
        {
            SDL_REG32_WR(  baseAddr + SDL_TOG_FLUSH, SDL_FMK( TOG_FLUSH_FL, 0U ) );
        }
        sdlResult = SDL_PASS;
    }
    return sdlResult;
}

 /**
 * Design: PROC_SDL-1511
 */
int32_t SDL_TOG_getIntrCount(SDL_TOG_Inst instance, SDL_TOG_IntrSrc intrSrc, uint32_t *pIntrCnt )
{
   int32_t  sdlResult = SDL_EFAIL;
   uint32_t baseAddr;

   if( (instance > SDL_TOG_MAX_INSTANCE) ||
       (pIntrCnt == NULL_PTR)            ||
       ((intrSrc != SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE)
            && (intrSrc != SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT)))

   {
       sdlResult = SDL_EBADARGS;
   }
   else
   {
        /* Getting base address */
        (void)SDL_TOG_getBaseaddr(instance, &baseAddr);

        (void)SDL_TOG_getIntrCountInternal( baseAddr, intrSrc, pIntrCnt );
        sdlResult = SDL_PASS;
    }
   return sdlResult;
}

/* nothing past this point */

