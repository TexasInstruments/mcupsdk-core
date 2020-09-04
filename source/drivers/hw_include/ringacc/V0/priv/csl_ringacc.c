/**
 * @file  csl_ringacc.c
 *
 * @brief
 *  This is the C implementation file for the Ring Accelerator (RINGACC) CSL-FL.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2016-2019, Texas Instruments, Inc.
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
#include <drivers/hw_include/ringacc/V0/csl_ringacc.h>

#define CSL_RINGACC_FIFO_WINDOW_SIZE_WORDS  ((uint32_t) 128U)   /* Size of each ring FIFO window in 32-bit words */

static uint32_t ringaccEncodeElementSize( uint32_t elSz );
static bool bIsPowerOfTwo(uint32_t x);
static bool bIsPhysBaseOk( const CSL_RingAccRingCfg *pRing );
static void *CSL_ringaccGetRingElementAddr( const CSL_RingAccRingCfg *pRing );
static CSL_RingAccMonitorType ringaccGetMonitorType( const CSL_ringacc_monitorRegs *pRegs, uint32_t monNum );
static void CSL_ringaccRefreshRingOcc( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing );
static bool CSL_ringaccIsRingEmpty( const CSL_RingAccRingCfg *pRing );
static bool CSL_ringaccIsRingFull( const CSL_RingAccRingCfg *pRing );
static void *CSL_ringaccGetRingDataPtr( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing );
static void CSL_ringaccResetRingWorkaround( CSL_RingAccCfg *pCfg, const CSL_RingAccRingCfg *pRing );
static inline void CSL_archMemoryFence(void);

/*=============================================================================
 * Encode the ring element size as follows:
 *  0 = 4 bytes
 *  1 = 8 bytes
 *  2 = 16 bytes
 *  3 = 32 bytes
 *  4 = 64 bytes
 *  5 = 128 bytes
 *  6 = 256 bytes
 *
 *  0 is returned if the element size != 4, 8, 16, 32, 64, 128, or 256.
 *===========================================================================*/
static inline void CSL_archMemoryFence(void)
{
#if defined (__aarch64__)
    Armv8_dsbSy();
#endif
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') /* R5F */
    __asm__ __volatile__ ( "dsb sy"  "\n\t": : : "memory");
#endif
}

static uint32_t ringaccEncodeElementSize( uint32_t elSz )
{
    uint32_t i;

    for(i=6u; (i>0u) && (elSz != (1u<<(i+2U))); i--) {}
    return i;
}

static bool bIsPowerOfTwo(uint32_t x)
{
  return ((x==0U) || ((x & (x - 1U))==0U)) ? (bool)true : (bool)false;
}

static bool bIsPhysBaseOk( const CSL_RingAccRingCfg *pRing )
{
    bool     bRetVal = (bool)true;
    uint64_t physBase = pRing->physBase;

    /*-------------------------------------------------------------------------
     * Base address cannot be 0 and must be aligned to the element size of the
     * ring (which has already been verified to be a power-of-2)
     *-----------------------------------------------------------------------*/
    if( (physBase == 0UL)   ||
        ((physBase & (((uint64_t)pRing->elSz - 1UL))) != (uint64_t)0UL) )
    {
        bRetVal = (bool)false;
    }
    return bRetVal;
}

static void *CSL_ringaccGetRingElementAddr( const CSL_RingAccRingCfg *pRing )
{
    return (void *)(((uintptr_t)pRing->rwIdx * pRing->elSz) + (uintptr_t)pRing->virtBase);
}

static CSL_RingAccMonitorType ringaccGetMonitorType( const CSL_ringacc_monitorRegs *pRegs, uint32_t monNum )
{
    return (CSL_RingAccMonitorType)CSL_REG32_FEXT( &pRegs->MON[monNum].CONTROL, RINGACC_MONITOR_MON_CONTROL_MODE );
}

static void CSL_ringaccRefreshRingOcc( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing )
{
    pRing->occ = CSL_ringaccGetRingOcc( pCfg, pRing->ringNum );
}

static bool CSL_ringaccIsRingEmpty( const CSL_RingAccRingCfg *pRing )
{
    bool bEmpty = (pRing->occ == 0U) ? (bool)true : (bool)false;
    return bEmpty;
}

static bool CSL_ringaccIsRingFull( const CSL_RingAccRingCfg *pRing )
{
    bool bFull = (pRing->occ == pRing->elCnt) ? (bool)true : (bool)false;
    return bFull;
}

/*=============================================================================
 *  This function returns a pointer to the specified ring element only if
 *  the element contains data. If the ring element is empty, NULL is returned.
 *===========================================================================*/
static void *CSL_ringaccGetRingDataPtr( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing )
{
    void *ptr = NULL;

    /* If this ring appears empty, then update the occupancy */
    if( CSL_ringaccIsRingEmpty( pRing ) == (bool)true )
    {
        CSL_ringaccRefreshRingOcc( pCfg, pRing );
    }
    /* Return pointer if not empty */
    if( CSL_ringaccIsRingEmpty( pRing ) == (bool)false )
    {
        ptr = (void *)CSL_ringaccGetRingElementAddr( pRing );
    }
    return ptr;
}

/*=============================================================================
 *  ringacc revision 1.0.19 and earlier has an issue where the UDMAP ring
 *  state (ring occupancy) will be out of sync with the ringacc ring state
 *  when a ring is reset.
 *
 *  This function performs a software workaround to correct this situation.
 *===========================================================================*/
#define MAX_DB_RING_CNT     (127U)

static void CSL_ringaccResetRingWorkaround( CSL_RingAccCfg *pCfg, const CSL_RingAccRingCfg *pRing )
{
    uint32_t regVal, ringOcc, dbRingCnt, thisDbRingCnt;
    uint32_t ringNum = pRing->ringNum;

    /*-------------------------------------------------------------------------
     *  1. Read the ring occupancy.
     *
     *  In the case of a credentials mode or qm mode ring, each ring write
     *  results in the ring occupancy increasing by 2 elements (one entry for
     *  the credentials, one entry for the data). However, the internal UDMAP
     *  ring state occupancy counter only records the number of writes, so we
     *  divide the ring occupancy by 2 to get back to the number of doorbell
     *  rings needed.
     *
     *  If the ring occupancy is not 0, then we need to execute the workaround.
     *-----------------------------------------------------------------------*/
    ringOcc = CSL_ringaccGetRingOcc( pCfg, ringNum );
    if( (pRing->mode == CSL_RINGACC_RING_MODE_CREDENTIALS) || (pRing->mode == CSL_RINGACC_RING_MODE_QM) )
    {
        ringOcc >>= 1;
    }
    if( ringOcc != 0U )
    {
        /*---------------------------------------------------------------------
         *  2. Reset the ring
         *-------------------------------------------------------------------*/
        regVal = CSL_REG32_RD( &pCfg->pCfgRegs->RING[ringNum].SIZE );
        CSL_REG32_WR( &pCfg->pCfgRegs->RING[ringNum].SIZE, regVal );
        /*---------------------------------------------------------------------
         *  3. Setup the ring in ring/doorbell mode (if not already in this
         *     mode)
         *-------------------------------------------------------------------*/
        if( pRing->mode != CSL_RINGACC_RING_MODE_RING )
        {
            CSL_REG32_FINS( &pCfg->pCfgRegs->RING[ringNum].SIZE, RINGACC_CFG_RING_SIZE_QMODE, CSL_RINGACC_RING_MODE_RING );
        }
        /*---------------------------------------------------------------------
         *  4. Ring the doorbell 2**22 - ringOcc times. This will wrap the
         *     internal UDMAP ring state occupancy counter (which is 21-bits
         *     wide) to 0.
         *-------------------------------------------------------------------*/
        dbRingCnt = ((uint32_t)1U << 22) - ringOcc;
        while( dbRingCnt != 0U )
        {
            /*-----------------------------------------------------------------
             *  Ring the doorbell with the maximum count each iteration if
             *  possible to minimize the total # of writes
             *---------------------------------------------------------------*/
            if( dbRingCnt > MAX_DB_RING_CNT )
            {
                thisDbRingCnt = MAX_DB_RING_CNT;
            }
            else
            {
                thisDbRingCnt = dbRingCnt;
            }
            CSL_REG32_WR( &pCfg->pRtRegs->RINGRT[ringNum].DB, CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt) );
            dbRingCnt -= thisDbRingCnt;
        }
        /*---------------------------------------------------------------------
         *  5. Restore the original ring mode (if not ring mode)
         *-------------------------------------------------------------------*/
        if( pRing->mode != CSL_RINGACC_RING_MODE_RING )
        {
            CSL_REG32_FINS( &pCfg->pCfgRegs->RING[ringNum].SIZE, RINGACC_CFG_RING_SIZE_QMODE, pRing->mode );
        }
    }
}

uint32_t CSL_ringaccGetRevision( const CSL_RingAccCfg *pCfg )
{
    return( CSL_REG32_RD(&pCfg->pGlbRegs->REVISION) );
}

void CSL_ringaccInitRingCfg( CSL_RingAccRingCfg *pRingCfg )
{
    /*-------------------------------------------------------------------------
     *  Start by initializing all structure members to 0
     *-----------------------------------------------------------------------*/
    memset( (void *)pRingCfg, 0, sizeof(CSL_RingAccRingCfg) );
    /*-------------------------------------------------------------------------
     *  Now initialize non-zero structure members
     *-----------------------------------------------------------------------*/
    pRingCfg->evtNum = CSL_RINGACC_RING_EVENT_DISABLE;
}

void CSL_ringaccInitRingObj( uint32_t ringNum,
                             CSL_RingAccRingCfg *pRing )
{
    /* Initialize the supplied and state fields */
    pRing->ringNum  = ringNum;
    pRing->rwIdx    = 0;
    pRing->waiting  = 0;
    pRing->occ      = 0;

    return;
}

int32_t CSL_ringaccInitRing( CSL_RingAccCfg *pCfg,
                             uint32_t ringNum,
                             CSL_RingAccRingCfg *pRing )
{
    int32_t retVal = 0;

    if( (pCfg == NULL)                                          ||
        (pCfg->pGlbRegs == NULL)                                ||
        (pCfg->pCfgRegs == NULL)                                ||
        (pCfg->pRtRegs == NULL)                                 ||
        (pCfg->maxRings > CSL_RINGACC_MAX_RINGS)                ||
        (pCfg->maxMonitors > CSL_RINGACC_MAX_MONITORS)          ||
        (pRing == NULL)                                         ||
        (ringNum >= pCfg->maxRings)                             ||
        (pRing->elCnt == 0U)                                    ||
        (pRing->virtBase == NULL)                               ||
        (pRing->mode >= CSL_RINGACC_RING_MODE_INVALID)          ||
        (pRing->elSz < 4U)                                      ||
        (pRing->elSz > 256U)                                    ||
        (!bIsPowerOfTwo(pRing->elSz))                           ||
        (!bIsPhysBaseOk(pRing)) )
    {
        retVal = -1;
    }
    else
    {
        /* Initialize the supplied fields */
        CSL_ringaccInitRingObj(ringNum, pRing);

        /* Write the register configuration */

        /* Ring base address */
        CSL_REG32_WR(&pCfg->pCfgRegs->RING[pRing->ringNum].BA_LO,
                      CSL_FMK(RINGACC_CFG_RING_BA_LO_ADDR_LO, (uint32_t)pRing->physBase ) );
        CSL_REG32_WR(&pCfg->pCfgRegs->RING[pRing->ringNum].BA_HI,
                      CSL_FMK(RINGACC_CFG_RING_BA_HI_ADDR_HI, (uint32_t)(pRing->physBase >> 32UL)) );

        /* Encoded ring size and element count */
        CSL_REG32_WR(&pCfg->pCfgRegs->RING[pRing->ringNum].SIZE,
            CSL_FMK(RINGACC_CFG_RING_SIZE_QMODE, pRing->mode)                               |
            CSL_FMK(RINGACC_CFG_RING_SIZE_ELSIZE, ringaccEncodeElementSize(pRing->elSz))    |
            CSL_FMK(RINGACC_CFG_RING_SIZE_ELCNT, pRing->elCnt) );

        /* Event number */
        CSL_REG32_WR(&pCfg->pCfgRegs->RING[pRing->ringNum].EVENT,
                      CSL_FMK(RINGACC_CFG_RING_EVENT_EVENT, pRing->evtNum ) );
    }
    return retVal;
}

int32_t CSL_ringaccSetEvent( CSL_RingAccCfg *pCfg,
                             uint32_t ringNum,
                             uint32_t evtNum )
{
    int32_t retVal = 0;

    if( (pCfg == NULL)                                          ||
        (pCfg->pCfgRegs == NULL)                                ||
        (pCfg->maxRings > CSL_RINGACC_MAX_RINGS)                ||
        (ringNum >= pCfg->maxRings))
    {
        retVal = -1;
    }
    else
    {
        /* Set Event number */
        CSL_REG32_WR(&pCfg->pCfgRegs->RING[ringNum].EVENT,
                      CSL_FMK(RINGACC_CFG_RING_EVENT_EVENT, evtNum ) );
    }
    return retVal;
}

uint32_t CSL_ringaccGetRingNum( const CSL_RingAccRingCfg *pRing )
{
    return pRing->ringNum;
}

void CSL_ringaccSetRingOrderId( CSL_RingAccCfg *pCfg, const CSL_RingAccRingCfg *pRing, uint32_t orderId )
{
    uint32_t regVal;

    if( orderId == CSL_RINGACC_ORDERID_BYPASS )
    {
        regVal = 0;
    }
    else
    {
        regVal = CSL_FMK( RINGACC_CFG_RING_ORDERID_ORDERID, orderId ) |
                 CSL_FMK( RINGACC_CFG_RING_ORDERID_REPLACE, (uint32_t)1U );
    }
    CSL_REG32_WR( &pCfg->pCfgRegs->RING[pRing->ringNum].ORDERID, regVal );
}

void CSL_ringaccCfgRingCred( CSL_RingAccCfg *pCfg, const CSL_RingAccRingCfg *pRing, bool bEnable, bool bLock )
{
    uint32_t regVal;

    /* Configure ISC CONTROL2 register */
    if( pRing->credVirtId == CSL_RINGACC_CRED_PASSTHRU )
    {
        regVal = CSL_FMK( RINGACC_ISC_EP_CONTROL2_PASS_V, (uint32_t)1U );
    }
    else
    {
        regVal = CSL_FMK( RINGACC_ISC_EP_CONTROL2_VIRTID, pRing->credVirtId );
    }
    CSL_REG32_WR( &pCfg->pIscRegs->EP[pRing->ringNum].CONTROL2, regVal );

    /* Configure ISC CONTROL register */
    regVal = 0;
    if( bEnable == (bool)true )
    {
        regVal |= CSL_FMK( RINGACC_ISC_EP_CONTROL_ENABLE, (uint32_t)0xAU );
    }
    if( bLock == (bool)true )
    {
        regVal |= CSL_FMK( RINGACC_ISC_EP_CONTROL_LOCK, (uint32_t)1U );
    }
    if( pRing->credPrivId == CSL_RINGACC_CRED_PASSTHRU )
    {
        regVal |= CSL_FMK( RINGACC_ISC_EP_CONTROL_PASS, (uint32_t)1U );
    }
    else
    {
        regVal |= CSL_FMK( RINGACC_ISC_EP_CONTROL_PRIV_ID, pRing->credPrivId );
    }
    regVal |= CSL_FMK( RINGACC_ISC_EP_CONTROL_PRIV, pRing->credPriv );
    if( pRing->credSecure == 0U)
    {
        regVal |= CSL_FMK( RINGACC_ISC_EP_CONTROL_NONSEC, (uint32_t)1U );
    }
    else
    {
         regVal |= CSL_FMK( RINGACC_ISC_EP_CONTROL_SEC, (uint32_t)0xAU );
   }
    CSL_REG32_WR( &pCfg->pIscRegs->EP[pRing->ringNum].CONTROL, regVal );
}

void CSL_ringaccResetRing( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing )
{
    uint32_t regVal;

    /*-------------------------------------------------------------------------
     *  ringacc revision 1.0.19 and earlier has an issue where the UDMAP ring
     *  state (ring occupancy) will be out of sync with the ringacc ring state
     *  when a ring is reset.
     *
     *  Perform the software workaround to correct this.
     *-----------------------------------------------------------------------*/
    regVal = CSL_ringaccGetRevision( pCfg );
    if( !( (CSL_FEXT( regVal, RINGACC_GCFG_REVISION_REVMAJ ) > 1U)  ||
           (CSL_FEXT( regVal, RINGACC_GCFG_REVISION_REVMIN ) > 0U)  ||
           (CSL_FEXT( regVal, RINGACC_GCFG_REVISION_REVRTL ) > 19U) ) )
    {
        CSL_ringaccResetRingWorkaround( pCfg, pRing );
    }

    /* To reset a ring, write to the ring's BA_LO, BA_HI, or SIZE register */
    regVal = CSL_REG32_RD( &pCfg->pCfgRegs->RING[pRing->ringNum].SIZE );
    CSL_REG32_WR( &pCfg->pCfgRegs->RING[pRing->ringNum].SIZE, regVal );

    /* Initialize state fields */
    pRing->rwIdx    = 0;
    pRing->waiting  = 0;
    pRing->occ      = 0;
}

void *CSL_ringaccGetCmdRingPtr( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing )
{
    void *ptr = NULL;

    /* If this ring appears full, then update the occupancy */
    if( CSL_ringaccIsRingFull( pRing ) == (bool)true )
    {
        CSL_ringaccRefreshRingOcc( pCfg, pRing );
    }
    /* Return pointer if not full */
    if( !CSL_ringaccIsRingFull( pRing ) )
    {
        ptr = (void *)CSL_ringaccGetRingElementAddr(pRing);
        pRing->waiting++;
        pRing->rwIdx = (pRing->rwIdx + 1U) % pRing->elCnt;
        /* Track ring occ count */
        pRing->occ++;
    }
    return ptr;
}

void *CSL_ringaccGetRspRingPtr( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing )
{
    void *ptr = NULL;

    /* If this ring appears empty, then update the occupancy */
    if( CSL_ringaccIsRingEmpty( pRing ) == (bool)true )
    {
        CSL_ringaccRefreshRingOcc( pCfg, pRing );
    }
    /* Return pointer if not empty */
    if( !CSL_ringaccIsRingEmpty( pRing ) )
    {
        ptr = (void *)CSL_ringaccGetRingElementAddr(pRing);
        pRing->waiting++;
        pRing->rwIdx = (pRing->rwIdx + 1U) % pRing->elCnt;
        /* Track ring occ count */
        pRing->occ--;
    }
    return ptr;
}

void CSL_ringaccAckRspRing( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, int32_t cnt )
{
    int32_t cntLocal = cnt;

    if( cntLocal==0 )
    {
        cntLocal = pRing->waiting;
    }
    if( cntLocal > 0 )
    {
        if( CSL_ringaccSetRingDoorbell( pCfg, pRing->ringNum, 0-cntLocal ) == (int32_t)0 )
        {
            pRing->waiting -= cntLocal;
        }
    }
}

uint32_t CSL_ringaccGetRingIdx( const CSL_RingAccCfg *pCfg, uint32_t ringNum )
{
    uint32_t retVal = 0;

    if( ringNum < pCfg->maxRings )
    {
        retVal = CSL_REG32_FEXT(&pCfg->pRtRegs->RINGRT[ringNum].INDX, RINGACC_RT_RINGRT_INDX_IDX);
    }
    return retVal;
}

uint32_t CSL_ringaccGetRingHwIdx( const CSL_RingAccCfg *pCfg, uint32_t ringNum )
{
    uint32_t retVal = 0;

    if( ringNum < pCfg->maxRings )
    {
        retVal = CSL_REG32_FEXT(&pCfg->pRtRegs->RINGRT[ringNum].HWINDX, RINGACC_RT_RINGRT_HWINDX_IDX);
    }
    return retVal;
}

uint32_t CSL_ringaccGetRingOcc( const CSL_RingAccCfg *pCfg, uint32_t ringNum )
{
    uint32_t retVal = 0;

    if( ringNum < pCfg->maxRings )
    {
        retVal = CSL_REG32_FEXT(&pCfg->pRtRegs->RINGRT[ringNum].OCC, RINGACC_RT_RINGRT_OCC_CNT);
    }
    return retVal;
}

uint32_t CSL_ringaccGetRingHwOcc( const CSL_RingAccCfg *pCfg, uint32_t ringNum )
{
    uint32_t retVal = 0;

    if( ringNum < pCfg->maxRings )
    {
        retVal = CSL_REG32_FEXT(&pCfg->pRtRegs->RINGRT[ringNum].HWOCC, RINGACC_RT_RINGRT_HWOCC_CNT);
    }
    return retVal;
}

int32_t CSL_ringaccSetTraceEnable( CSL_RingAccCfg *pCfg, bool bEnable )
{
    int32_t retVal = -1;
    if( pCfg->bTraceSupported == (bool)true )
    {
        uint32_t traceEnable = (bEnable==(bool)true) ? (uint32_t)1U : (uint32_t)0U;
        CSL_REG32_FINS(&pCfg->pGlbRegs->TRACE_CTL, RINGACC_GCFG_TRACE_CTL_EN, traceEnable);
        retVal = 0;
    }
    return retVal;
}

int32_t CSL_ringaccEnableTrace( CSL_RingAccCfg *pCfg )
{
    return CSL_ringaccSetTraceEnable(pCfg, (bool)true);
}

int32_t CSL_ringaccDisableTrace( CSL_RingAccCfg *pCfg )
{
    return CSL_ringaccSetTraceEnable(pCfg, (bool)false);
}

int32_t CSL_ringaccCfgTrace( CSL_RingAccCfg *pCfg, bool bTraceAll, bool bIncMsgData, uint32_t ringNum )
{
    int32_t retVal = -1;
    uint32_t ringNumLocal = ringNum;

    /* Ignore ring number for all trace enable */
    if( bTraceAll == (bool)true )
    {
        ringNumLocal = 0U;
    }
    if( pCfg->bTraceSupported && (ringNumLocal < pCfg->maxRings) )
    {
        CSL_REG32_WR( &pCfg->pGlbRegs->TRACE_CTL,
            CSL_FMK(RINGACC_GCFG_TRACE_CTL_ALL_QUEUES, ((bTraceAll==(bool)true) ? (uint32_t)1U: (uint32_t)0) )   |
            CSL_FMK(RINGACC_GCFG_TRACE_CTL_MSG, ((bIncMsgData==(bool)true) ? (uint32_t)1U : (uint32_t)0) )        |
            CSL_FMK(RINGACC_GCFG_TRACE_CTL_QUEUE, ringNumLocal) );
        retVal = 0;
    }
    return retVal;
}

int32_t CSL_ringaccCfgRingMonitor( CSL_RingAccCfg *pCfg,
                            uint32_t monNum,
                            CSL_RingAccMonitorType monType,
                            uint32_t ringNum,
                            uint32_t eventNum,
                            CSL_RingAccMonitorDataSrc dataSrc,
                            uint32_t data0Val,
                            uint32_t data1Val )
{
    uint32_t data0Val_local = data0Val, data1Val_local = data1Val;
    int32_t retVal = -1;

    if( (pCfg->maxMonitors > 0U)                         &&
        (monType < CSL_RINGACC_MONITOR_TYPE_INVALID)    &&
        (monNum < pCfg->maxMonitors)                    &&
        (ringNum < pCfg->maxRings)                      &&
        (dataSrc < CSL_RINGACC_MONITOR_DATA_SRC_INVALID) )
    {
        uint32_t regVal;

        if( monType == CSL_RINGACC_MONITOR_TYPE_THRESHOLD )
        {
            if( data0Val_local > data1Val_local )
            {
                uint32_t swapTmp = data0Val_local;
                data0Val_local   = data1Val_local;
                data1Val_local   = swapTmp;
            }
            CSL_REG32_WR( &pCfg->pMonRegs->MON[monNum].DATA0, data0Val_local );
            CSL_REG32_WR( &pCfg->pMonRegs->MON[monNum].DATA1, data1Val_local );
        }
        regVal =    CSL_FMK( RINGACC_MONITOR_MON_QUEUE_VAL, ringNum );
        CSL_REG32_WR( &pCfg->pMonRegs->MON[monNum].QUEUE, regVal );
        regVal =    CSL_FMK( RINGACC_MONITOR_MON_CONTROL_EVT, eventNum )                |
                    CSL_FMK( RINGACC_MONITOR_MON_CONTROL_SOURCE, ((uint32_t)dataSrc) )    |
                    CSL_FMK( RINGACC_MONITOR_MON_CONTROL_MODE, monType );
        CSL_REG32_WR( &pCfg->pMonRegs->MON[monNum].CONTROL, regVal );
        retVal = 0;
    }
    return retVal;
}

int32_t CSL_ringaccReadRingMonitor( const CSL_RingAccCfg *pCfg, uint32_t monNum, uint32_t *pData0, uint32_t *pData1 )
{
    int32_t retVal = -1;

    if( (pCfg->maxMonitors > 0U)        &&
        (monNum < pCfg->maxMonitors)    &&
        (ringaccGetMonitorType( pCfg->pMonRegs, monNum ) != CSL_RINGACC_MONITOR_TYPE_DISABLED) )
    {
        if( pData0 != NULL )
        {
            *pData0 = CSL_REG32_RD( &pCfg->pMonRegs->MON[monNum].DATA0 );
        }
        if( pData1 != NULL )
        {
            *pData1 = CSL_REG32_RD( &pCfg->pMonRegs->MON[monNum].DATA1 );
        }
        retVal = 0;
    }
    return retVal;
}

int32_t CSL_ringaccPush32( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint32_t val, CSL_ringaccMemOpsFxnPtr pfMemOps)
{
    int32_t retVal = 0;

    if(pRing->mode == CSL_RINGACC_RING_MODE_RING)
    {
        uint32_t *pRingEntry;

        pRingEntry = (uint32_t *)CSL_ringaccGetCmdRingPtr( pCfg,  pRing );
        if( pRingEntry != NULL )
        {
            *pRingEntry = val;
            CSL_archMemoryFence();
            if( pfMemOps != NULL )
            {
                (*pfMemOps)((void *)pRingEntry, sizeof(uint32_t), CSL_RINGACC_MEM_OPS_TYPE_WR);
            }
            CSL_ringaccCommitToCmdRing( pCfg, pRing, 1 );
        }
        else
        {
            retVal = -1;    /* Ring is full */
        }
    }
    else
    {
        CSL_REG32_WR( &pCfg->pFifoRegs->FIFO[pRing->ringNum].RINGTAILDATA[CSL_RINGACC_FIFO_WINDOW_SIZE_WORDS-1U], val );
    }
    return retVal;
}

int32_t CSL_ringaccPop32( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint32_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if(pRing->mode == CSL_RINGACC_RING_MODE_RING)
    {
        uint32_t *pRingEntry;

        pRingEntry = (uint32_t *)CSL_ringaccGetRspRingPtr( pCfg, pRing );
        if( pRingEntry != NULL )
        {
            if( pfMemOps != NULL )
            {
                (*pfMemOps)((void *)pRingEntry, sizeof(uint32_t), CSL_RINGACC_MEM_OPS_TYPE_RD);
            }

            *pVal = *pRingEntry;
            CSL_ringaccAckRspRing( pCfg, pRing, 1 );
        }
        else
        {
            *pVal = 0;
            retVal = -1;    /* Ring is empty */
        }
    }
    else
    {
        *pVal = CSL_REG32_RD( &pCfg->pFifoRegs->FIFO[pRing->ringNum].RINGHEADDATA[CSL_RINGACC_FIFO_WINDOW_SIZE_WORDS-1U] );
    }
    return retVal;
}

int32_t CSL_ringaccHwPop32( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint32_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if( CSL_ringaccGetRingHwOcc( pCfg, pRing->ringNum ) == (uint32_t)0U )
    {
        retVal = -1;        /* Ring is empty */
    }
    else
    {
        uint32_t *pRingHeadFifoAddr;

        pRingHeadFifoAddr = (uint32_t *)&pCfg->pFifoRegs->FIFO[pRing->ringNum].RINGHEADDATA[CSL_RINGACC_FIFO_WINDOW_SIZE_WORDS-1U];
        if( pfMemOps != NULL )
        {
            (*pfMemOps)((void *)pRingHeadFifoAddr, sizeof(uint32_t), CSL_RINGACC_MEM_OPS_TYPE_RD);
        }
        *pVal = CSL_REG32_RD( pRingHeadFifoAddr );
    }
    return retVal;
}

int32_t CSL_ringaccPeek32( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint32_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if(pRing->mode == CSL_RINGACC_RING_MODE_RING)
    {
        uint32_t *pRingEntry;

        pRingEntry = (uint32_t *)CSL_ringaccGetRingDataPtr( pCfg, pRing );
        if( pRingEntry != NULL )
        {
            if( pfMemOps != NULL )
            {
                (*pfMemOps)((void *)pRingEntry, sizeof(uint32_t), CSL_RINGACC_MEM_OPS_TYPE_RD);
            }
            *pVal = *pRingEntry;
        }
        else
        {
            *pVal = 0;
            retVal = -1;    /* Ring is empty */
        }
    }
    else
    {
        *pVal = CSL_REG32_RD( &pCfg->pFifoRegs->FIFO[pRing->ringNum].PEEKHEADDATA[CSL_RINGACC_FIFO_WINDOW_SIZE_WORDS-1U] );
    }
    return retVal;
}

int32_t CSL_ringaccPush64( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint64_t val, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if( pRing->elSz < sizeof(uint64_t) )
    {
        retVal = -2;        /* Requested access size is greater than ring element size */
    }
    else
    {
        if(pRing->mode == CSL_RINGACC_RING_MODE_RING)
        {
            uint64_t *pRingEntry;

            pRingEntry = (uint64_t *)CSL_ringaccGetCmdRingPtr( pCfg,  pRing );
            if( pRingEntry != NULL )
            {
                *pRingEntry = val;
                CSL_archMemoryFence();
                if( pfMemOps != NULL )
                {
                    (*pfMemOps)((void *)pRingEntry, sizeof(uint64_t), CSL_RINGACC_MEM_OPS_TYPE_WR);
                }
                CSL_ringaccCommitToCmdRing( pCfg, pRing, 1 );
            }
            else
            {
                retVal = -1;    /* Ring is full */
            }
        }
        else
        {
            CSL_REG64_WR( (uint64_t *)&pCfg->pFifoRegs->FIFO[pRing->ringNum].RINGTAILDATA[CSL_RINGACC_FIFO_WINDOW_SIZE_WORDS-2U], val );
        }
    }
    return retVal;
}

int32_t CSL_ringaccPop64( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint64_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if( pRing->elSz < sizeof(uint64_t) )
    {
        retVal = -2;        /* Requested access size is greater than ring element size */
    }
    else
    {
        if(pRing->mode == CSL_RINGACC_RING_MODE_RING)
        {
            uint64_t *pRingEntry;

            pRingEntry = (uint64_t *)CSL_ringaccGetRspRingPtr( pCfg, pRing );
            if( pRingEntry != NULL )
            {
                if( pfMemOps != NULL )
                {
                    (*pfMemOps)((void *)pRingEntry, sizeof(uint64_t), CSL_RINGACC_MEM_OPS_TYPE_RD);
                }
                *pVal = *pRingEntry;
                CSL_ringaccAckRspRing( pCfg, pRing, 1 );
            }
            else
            {
                *pVal = 0;
                retVal = -1;    /* Ring is empty */
            }
        }
        else
        {
            *pVal = CSL_REG64_RD( (uint64_t *)&pCfg->pFifoRegs->FIFO[pRing->ringNum].RINGHEADDATA[CSL_RINGACC_FIFO_WINDOW_SIZE_WORDS-2U] );
        }
    }
    return retVal;
}

int32_t CSL_ringaccHwPop64( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint64_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if( CSL_ringaccGetRingHwOcc( pCfg, pRing->ringNum ) == (uint32_t)0U )
    {
        retVal = -1;        /* Ring is empty */
    }
    else if( pRing->elSz < sizeof(uint64_t) )
    {
        retVal = -2;        /* Requested access size is greater than ring element size */
    }
    else
    {
        uint64_t *pRingHeadFifoAddr;

        pRingHeadFifoAddr = (uint64_t *)&pCfg->pFifoRegs->FIFO[pRing->ringNum].RINGHEADDATA[CSL_RINGACC_FIFO_WINDOW_SIZE_WORDS-2U];
        if( pfMemOps != NULL )
        {
            (*pfMemOps)((void *)pRingHeadFifoAddr, sizeof(uint64_t), CSL_RINGACC_MEM_OPS_TYPE_RD);
        }
        *pVal = CSL_REG64_RD( pRingHeadFifoAddr );
    }
    return retVal;
}

int32_t CSL_ringaccPeek64( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint64_t *pVal, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if( pRing->elSz < sizeof(uint64_t) )
    {
        retVal = -2;        /* Requested access size is greater than ring element size */
    }
    else
    {
        if(pRing->mode == CSL_RINGACC_RING_MODE_RING)
        {
            uint64_t *pRingEntry;

            pRingEntry = (uint64_t *)CSL_ringaccGetRingDataPtr( pCfg, pRing );
            if( pRingEntry != NULL )
            {
                if( pfMemOps != NULL )
                {
                    (*pfMemOps)((void *)pRingEntry, sizeof(uint64_t), CSL_RINGACC_MEM_OPS_TYPE_RD);
                }
                *pVal = *pRingEntry;
            }
            else
            {
                *pVal = 0;
                retVal = -1;    /* Ring is empty */
            }
        }
        else
        {
            *pVal = CSL_REG64_RD( (uint64_t *)&pCfg->pFifoRegs->FIFO[pRing->ringNum].PEEKHEADDATA[CSL_RINGACC_FIFO_WINDOW_SIZE_WORDS-2U] );
        }
    }
    return retVal;
}

int32_t CSL_ringaccWrData( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;
    uint32_t numBytes_local = numBytes;
    uint8_t *pData_local = pData;


    if( pRing->mode == CSL_RINGACC_RING_MODE_RING )
    {
        if( numBytes_local <= pRing->elSz )
        {
            uint8_t *pRingEntry;

            pRingEntry = (uint8_t *)CSL_ringaccGetCmdRingPtr( pCfg,  pRing );
            if( pRingEntry != NULL )
            {
                uint8_t *pRingEntrySave = pRingEntry;

                while( numBytes_local != 0U )
                {
                    numBytes_local--;
                    *pRingEntry = *pData_local;
                    pRingEntry++;
                    pData_local++;
                }
                CSL_archMemoryFence();
                if( pfMemOps != NULL )
                {
                    (*pfMemOps)((void *)pRingEntrySave, numBytes, CSL_RINGACC_MEM_OPS_TYPE_WR);
                }
                CSL_ringaccCommitToCmdRing( pCfg, pRing, 1 );
            }
            else
            {
                retVal = -1;    /* Ring is full */
            }
        }
        else
        {
            retVal = -2;    /* Requested access size is greater than ring element size */
        }
    }
    else
    {
        retVal = -3;        /* ring is configured in wrong mode (must be a ring mode ring) */
    }
    return retVal;
}

int32_t CSL_ringaccRdData( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;
    uint32_t numBytes_local = numBytes;
    uint8_t *pData_local = pData;

    if( pRing->mode == CSL_RINGACC_RING_MODE_RING )
    {
        if( numBytes_local <= pRing->elSz )
        {
            uint8_t *pRingEntry;

            pRingEntry = (uint8_t *)CSL_ringaccGetRspRingPtr( pCfg,  pRing );
            if( pRingEntry != NULL )
            {
                if( pfMemOps != NULL )
                {
                    (*pfMemOps)((void *)pRingEntry, numBytes_local, CSL_RINGACC_MEM_OPS_TYPE_RD);
                }
                while( numBytes_local != 0U )
                {
                    numBytes_local--;
                    *pData_local = *pRingEntry;
                    pData_local++;
                    pRingEntry++;
                }
                CSL_ringaccAckRspRing( pCfg, pRing, 1 );
            }
            else
            {
                retVal = -1;    /* Ring is empty */
            }
        }
        else
        {
            retVal = -2;    /* Requested access size is greater than ring element size */
        }
    }
    else
    {
        retVal = -3;        /* ring is configured in wrong mode (must be a ring mode ring) */
    }
    return retVal;
}

int32_t CSL_ringaccPeekData( CSL_RingAccCfg *pCfg, CSL_RingAccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;
    uint32_t numBytes_local = numBytes;
    uint8_t *pData_local = pData;


    if( pRing->mode == CSL_RINGACC_RING_MODE_RING )
    {
        if( numBytes_local <= pRing->elSz )
        {
            uint8_t *pRingEntry;

            pRingEntry = (uint8_t *)CSL_ringaccGetRingElementAddr(pRing);
            if (pRingEntry != NULL )
            {
                if( pfMemOps != NULL )
                {
                    (*pfMemOps)((void *)pRingEntry, numBytes_local, CSL_RINGACC_MEM_OPS_TYPE_RD);
                }
                while( numBytes_local != 0U )
                {
                    numBytes_local--;
                    *pData_local = *pRingEntry;
                    pData_local++;
                    pRingEntry++;
                }
            }
        }
        else
        {
            retVal = -2;    /* Requested access size is greater than ring element size */
        }
    }
    else
    {
        retVal = -3;        /* ring is configured in wrong mode (must be a ring mode ring) */
    }
    return retVal;
}
