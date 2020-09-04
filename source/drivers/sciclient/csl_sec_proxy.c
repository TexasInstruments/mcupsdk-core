/**
 * @file  csl_sec_proxy.c
 *
 * @brief
 *  Implementation file for the sec_proxy module CSL.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2017-2019, Texas Instruments, Inc.
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
 *    Neither the name of Texas Instruments Incorposec_proxyed nor the names of
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

#include <drivers/sciclient/csl_sec_proxy.h>

#define CSL_SEC_PROXY_TARGET_CHAN_SIZE  (0x1000U)

static uint32_t CSL_secProxyIpfeatEncodePidVal( uint32_t pidVal );
static uint32_t CSL_secProxyIpfeatEncodeRevVals( uint32_t majRev, uint32_t minRev, uint32_t rtlRev );

static uint32_t CSL_secProxyIpfeatEncodePidVal( uint32_t pidVal )
{
    uint32_t encodedVal;
    
    encodedVal =    ((pidVal & (uint32_t)0x0700U) << 3U)    |
                    ((pidVal & (uint32_t)0x003FU) << 5U)    |
                    ((pidVal & (uint32_t)0xF800U) >> 11U);
    return encodedVal;
}

static uint32_t CSL_secProxyIpfeatEncodeRevVals( uint32_t majRev, uint32_t minRev, uint32_t rtlRev )
{
    uint32_t encodedVal;
    
    encodedVal =    ((majRev & (uint32_t)0x0007U) << 11U)   |
                    ((minRev & (uint32_t)0x003FU) << 5U)    |
                    ((rtlRev & (uint32_t)0x001FU) >> 0U);
    return encodedVal;
}

/*=============================================================================
 *  CSL-FL functions
 *===========================================================================*/

uint32_t CSL_secProxyGetRevision( const CSL_SecProxyCfg *pSecProxyCfg )
{
    return CSL_REG32_RD(&pSecProxyCfg->pSecProxyRegs->PID);
}

uint32_t CSL_secProxyGetNumThreads( const CSL_SecProxyCfg *pSecProxyCfg )
{
    return (uint32_t)CSL_REG32_FEXT( &pSecProxyCfg->pSecProxyRegs->CONFIG, SEC_PROXY_CONFIG_THREADS );
}

int32_t CSL_secProxyCfgGlobalErrEvtNum( const CSL_SecProxyCfg *pSecProxyCfg, uint32_t globalErrEvtNum )
{
    int32_t retVal;
#ifdef CSL_SEC_PROXY_GLB_EVT_ERR_EVENT_MASK
    CSL_REG32_FINS( &pSecProxyCfg->pSecProxyRegs->GLB_EVT, SEC_PROXY_GLB_EVT_ERR_EVENT, globalErrEvtNum );
    retVal = 0;
#else
    retVal = -1;
#endif
    return retVal;
}

void CSL_secProxySetBufferAccessOrderId( CSL_SecProxyCfg *pSecProxyCfg, uint32_t orderId )
{
    uint32_t regVal;

    regVal = CSL_FMK( SEC_PROXY_SCFG_ORDERID_ORDERID, orderId ) |
             CSL_FMK( SEC_PROXY_SCFG_ORDERID_REPLACE, (uint32_t)1U );
    CSL_REG32_WR( &pSecProxyCfg->pSecProxyScfgRegs->ORDERID, regVal );
}

void CSL_secProxyCfg( CSL_SecProxyCfg *pSecProxyCfg, uint64_t targetAddr, uint64_t extBufferAddr )
{
    uint32_t regVal;

    regVal = (uint32_t)(extBufferAddr & 0xFFFFFFFFUL);
    CSL_REG32_WR( &pSecProxyCfg->pSecProxyScfgRegs->BUFFER_L, CSL_FMK( SEC_PROXY_SCFG_BUFFER_L_BASE_L, regVal ) );
    regVal = (uint32_t)((extBufferAddr >> 32UL) & 0xFFFFFFFFUL);
    CSL_REG32_WR( &pSecProxyCfg->pSecProxyScfgRegs->BUFFER_H, CSL_FMK( SEC_PROXY_SCFG_BUFFER_H_BASE_H, regVal ) );
    if( targetAddr != (uint64_t)0UL )
    {
        regVal = (uint32_t)(targetAddr & 0xFFFFFFFFUL);
        CSL_REG32_WR( &pSecProxyCfg->pSecProxyScfgRegs->TARGET_L, CSL_FMK( SEC_PROXY_SCFG_TARGET_L_BASE_L, regVal ) );
        regVal = (uint32_t)((targetAddr >> 32UL) & 0xFFFFFFFFUL);
        CSL_REG32_WR( &pSecProxyCfg->pSecProxyScfgRegs->TARGET_H, CSL_FMK( SEC_PROXY_SCFG_TARGET_H_BASE_H, regVal ) );
    }
}

int32_t CSL_secProxyCfgThread( CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum, const CSL_SecProxyThreadCfg *pThreadCfg )
{
    int32_t retVal = 0;
    uint32_t regVal;

    if( threadNum < CSL_secProxyGetNumThreads(pSecProxyCfg) )
    {
        regVal = CSL_FMK( SEC_PROXY_SCFG_THREAD_CTL_DIR,     pThreadCfg->dir )                  |
                 CSL_FMK( SEC_PROXY_SCFG_THREAD_CTL_MAX_CNT, pThreadCfg->outboundMaxMsgCnt )    |
                 CSL_FMK( SEC_PROXY_SCFG_THREAD_CTL_QUEUE,   pThreadCfg->queueNum );
        CSL_REG32_WR( &pSecProxyCfg->pSecProxyScfgRegs->THREAD[threadNum].CTL, regVal );

        regVal = CSL_FMK( SEC_PROXY_SCFG_THREAD_EVT_MAP_ERR_EVT, pThreadCfg->errEvtNum )        |
                 CSL_FMK( SEC_PROXY_SCFG_THREAD_EVT_MAP_THR_EVT, pThreadCfg->threshEvtNum );
        CSL_REG32_WR( &pSecProxyCfg->pSecProxyScfgRegs->THREAD[threadNum].EVT_MAP, regVal );

        regVal = CSL_FMK( SEC_PROXY_SCFG_THREAD_DST_THREAD, pThreadCfg->outboundDstThread );
        CSL_REG32_WR( &pSecProxyCfg->pSecProxyScfgRegs->THREAD[threadNum].DST, regVal );

        regVal = CSL_FMK( SEC_PROXY_RT_THREAD_THR_THR_CNT,  pThreadCfg->threshCnt );
        CSL_REG32_WR( &pSecProxyCfg->pSecProxyRtRegs->THREAD[threadNum].THR, regVal );
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

uintptr_t CSL_secProxyGetDataAddr( const CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum, uint32_t numBytes )
{
    uintptr_t dataAddr;

    /*-------------------------------------------------------------------------
     *  Data address is 4 bytes past the start of the buffer for this thread
     *  (the proxy uses the first word of the buffer for private tracking
     *  purposes)
     *-----------------------------------------------------------------------*/
    dataAddr = (uintptr_t)pSecProxyCfg->proxyTargetAddr + ((uintptr_t)threadNum * CSL_SEC_PROXY_TARGET_CHAN_SIZE) + (uintptr_t)CSL_SEC_PROXY_RSVD_MSG_BYTES;
    return dataAddr;
}

void CSL_secProxyAccessTarget( CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum, uint8_t *pData, uint32_t numBytes, CSL_SecProxyMemAccessCbFxnPtr fpMemAccess )
{
    uintptr_t chanDataAddr;

    /*-------------------------------------------------------------------------
     *  Get the pointer to the start of the proxy data buffer (just past the
     *  reserved bytes) and write/read data to/from it.
     *-----------------------------------------------------------------------*/
    chanDataAddr = CSL_secProxyGetDataAddr( pSecProxyCfg, threadNum, numBytes );
    fpMemAccess( chanDataAddr, pData, 1, numBytes );
    /*-------------------------------------------------------------------------
     *  Access the last byte of the proxy data buffer to complete the write or
     *  read access.
     *-----------------------------------------------------------------------*/
    if( numBytes < CSL_secProxyGetMsgSize(pSecProxyCfg) )
    {
        uint8_t tmp = 0;

        chanDataAddr += (uintptr_t)CSL_secProxyGetMsgSize(pSecProxyCfg) - (uintptr_t)1U;
        fpMemAccess( chanDataAddr, &tmp, 1, 1 );
    }
}

int32_t CSL_secProxyGetThreadStatus( const CSL_SecProxyCfg *pSecProxyCfg, uint32_t threadNum, CSL_SecProxyThreadStatus *pThreadStatus )
{
    int32_t retVal = 0;

    if( threadNum < CSL_secProxyGetNumThreads(pSecProxyCfg) )
    {
        uint32_t regVal;

        regVal = CSL_REG32_RD( &pSecProxyCfg->pSecProxyRtRegs->THREAD[threadNum].STATUS );
        pThreadStatus->error     = CSL_FEXT( regVal, SEC_PROXY_RT_THREAD_STATUS_ERROR );
        pThreadStatus->curMsgCnt = CSL_FEXT( regVal, SEC_PROXY_RT_THREAD_STATUS_CUR_CNT );
        pThreadStatus->dir       = CSL_SEC_PROXY_THREAD_STATUS_UNAVAILABLE;
        pThreadStatus->maxMsgCnt = CSL_SEC_PROXY_THREAD_STATUS_UNAVAILABLE;
        /*---------------------------------------------------------------------
         *  Thread status direction and max count features are only available
         *  in secure proxy IP rev 1.0.7.x and later. Also check to make sure
         *  the CSL-RL in use contains the corresponding field macros.
         *-------------------------------------------------------------------*/
#if defined(CSL_SEC_PROXY_RT_THREAD_STATUS_DIR_MASK) && defined(CSL_SEC_PROXY_RT_THREAD_STATUS_MAX_CNT_MASK)
        if( CSL_secProxyIpfeatEncodePidVal( CSL_secProxyGetRevision(pSecProxyCfg) ) >= CSL_secProxyIpfeatEncodeRevVals( (uint32_t)1U, (uint32_t)0U, (uint32_t)7U) )
        {
            pThreadStatus->dir = CSL_FEXT( regVal, SEC_PROXY_RT_THREAD_STATUS_DIR );
            if( pThreadStatus->dir == (uint32_t)0U )
            {
                /* maxMsgCnt is valid only when dir == 0 */
                pThreadStatus->maxMsgCnt = CSL_FEXT( regVal, SEC_PROXY_RT_THREAD_STATUS_MAX_CNT );
            }
            else
            {
                pThreadStatus->maxMsgCnt = 0;
            }
        }
#endif
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}
