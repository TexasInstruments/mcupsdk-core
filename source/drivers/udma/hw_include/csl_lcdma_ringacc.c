/**
 * @file  csl_lcdma_ringacc.c
 *
 * @brief
 *  This is the C implementation file for the low cost DMA Ring Accelerator
 *  (RINGACC) CSL-FL.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2016-2020, Texas Instruments, Inc.
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
#include <drivers/udma/hw_include/csl_lcdma_ringacc.h>
#ifdef __aarch64__
#include <kernel/nortos/dpl/a53/common_armv8.h>
#endif

static bool bIsPhysBaseOk( const CSL_LcdmaRingaccRingCfg *pRing );
static void *CSL_lcdma_ringaccGetRingRdElementAddr( const CSL_LcdmaRingaccRingCfg *pRing );
static void *CSL_lcdma_ringaccGetRingWrElementAddr( const CSL_LcdmaRingaccRingCfg *pRing );
static void CSL_lcdma_ringaccGetNewElCnt( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing );
static bool CSL_lcdma_ringaccIsRingEmpty( const CSL_LcdmaRingaccRingCfg *pRing );
static bool CSL_lcdma_ringaccIsRingFull( const CSL_LcdmaRingaccRingCfg *pRing );
static void *CSL_lcdma_ringaccGetRingDataPtr( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing );
static int32_t CSL_lcdma_ringaccPush64MultiAccess( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVals, uint32_t numValues, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );
static int32_t CSL_lcdma_ringaccPop64MultiAccess( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVals, uint32_t numValues, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );
static int32_t CSL_lcdma_ringaccPeek64Access( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps );

static inline void CSL_archMemoryFence(void);
static inline void CSL_archMemoryFence(void)
{
#if defined (__aarch64__)
    Armv8_dsbSy();
#endif
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') /* R5F */
    //CSL_armR5Dsb(); //TODO
#endif
}

static bool bIsPhysBaseOk( const CSL_LcdmaRingaccRingCfg *pRing )
{
    bool     bRetVal = (bool)true;
    uint64_t physBase;

    /*-------------------------------------------------------------------------
     * Remove asel from physBase
     *-----------------------------------------------------------------------*/
    physBase = CSL_lcdma_ringaccClrAselInAddr( pRing->physBase );
    /*-------------------------------------------------------------------------
     * Base address cannot be 0 and must be aligned to the element size of the
     * ring (which has already been verified to be a power-of-2)
     *-----------------------------------------------------------------------*/
    if( (physBase == 0UL)   ||
        ((physBase & (((uint64_t)CSL_LCDMA_RINGACC_RING_EL_SIZE_BYTES - 1UL))) != (uint64_t)0UL) )
    {
        bRetVal = (bool)false;
    }
    return bRetVal;
}

static void *CSL_lcdma_ringaccGetRingRdElementAddr( const CSL_LcdmaRingaccRingCfg *pRing )
{
    return (void *)(((uintptr_t)pRing->rdIdx * pRing->elSz) + (uintptr_t)pRing->virtBase);
}

static void *CSL_lcdma_ringaccGetRingWrElementAddr( const CSL_LcdmaRingaccRingCfg *pRing )
{
    return (void *)(((uintptr_t)pRing->wrIdx * pRing->elSz) + (uintptr_t)pRing->virtBase);
}

static void CSL_lcdma_ringaccGetNewElCnt( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing )
{
    pRing->rdOcc = CSL_lcdma_ringaccGetReverseRingOcc( pCfg, pRing->ringNum, pRing->mode );
}

static bool CSL_lcdma_ringaccIsRingEmpty( const CSL_LcdmaRingaccRingCfg *pRing )
{
    bool bEmpty = (pRing->rdOcc == 0U) ? (bool)true : (bool)false;
    return bEmpty;
}

static bool CSL_lcdma_ringaccIsRingFull( const CSL_LcdmaRingaccRingCfg *pRing )
{
    bool bFull = (pRing->wrOcc == pRing->elCnt) ? (bool)true : (bool)false;
    return bFull;
}

/*=============================================================================
 *  This function returns a pointer to the specified ring element only if
 *  the element contains data. If the ring element is empty, NULL is returned.
 *===========================================================================*/
static void *CSL_lcdma_ringaccGetRingDataPtr( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing )
{
    void *ptr = NULL;

    /* If this ring appears empty, then update the occupancy */
    if( CSL_lcdma_ringaccIsRingEmpty( pRing ) == (bool)true )
    {
        CSL_lcdma_ringaccGetNewElCnt( pCfg, pRing );
    }
    /* Return pointer if not empty */
    if( CSL_lcdma_ringaccIsRingEmpty( pRing ) == (bool)false )
    {
        ptr = (void *)CSL_lcdma_ringaccGetRingRdElementAddr( pRing );
    }
    return ptr;
}

static int32_t CSL_lcdma_ringaccPush64MultiAccess( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVals, uint32_t numValues, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if( pRing->elSz < sizeof(uint64_t) )
    {
        retVal = -2;        /* Requested access size is greater than ring element size */
    }
    else if( numValues == 0U )
    {
        retVal = 0;
    }
    else
    {
        uint32_t numValuesWritten = pRing->elCnt - pRing->wrOcc;    /* initially set to the maximum # of writes to fill ring */
        if( numValuesWritten == 0U )
        {
            retVal = -1;    /* Ring is full */
        }
        else
        {
            uint32_t i, localWrIdx = pRing->wrIdx;
            uint64_t *pValsLocal = pVals;
            void     *pRingEntry;

            if( numValuesWritten > numValues )
            {
                numValuesWritten = numValues;
            }
            for( i=0U; i<numValuesWritten; i++, pValsLocal++ )
            {
                pRingEntry = (void *)(((uintptr_t)localWrIdx * pRing->elSz) + (uintptr_t)pRing->virtBase);
                *(uint64_t *)pRingEntry = *pValsLocal;
                localWrIdx++;
                localWrIdx = localWrIdx % pRing->elCnt;
            }
            CSL_archMemoryFence();
            /*-----------------------------------------------------------------
             * Perform the specified memory operation only if the ring's
             * physical base address's asel value == 0 (not cache coherent)
             *---------------------------------------------------------------*/
            if( (pfMemOps != NULL) && (pRing->asel == (uint32_t)0U) )
            {
                (*pfMemOps)((void *)pRingEntry, numValuesWritten * sizeof(uint64_t), CSL_LCDMA_RINGACC_MEM_OPS_TYPE_WR);
            }
            CSL_lcdma_ringaccCommitToForwardRing( pCfg, pRing, (int32_t)numValuesWritten );    /* This call will update wrOcc and wrIdx elements in pRing */
            if( numValues != numValuesWritten )
            {
                retVal = (int32_t)numValuesWritten;
            }
        }
    }
    return retVal;
}

static int32_t CSL_lcdma_ringaccPop64MultiAccess( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVals, uint32_t numValues, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if( pRing->elSz < sizeof(uint64_t) )
    {
        retVal = -2;        /* Requested access size is greater than ring element size */
    }
    else
    {
        uint32_t numValuesRead;

        CSL_lcdma_ringaccGetNewElCnt( pCfg, pRing );    /* Update pRing->rdOcc */
        numValuesRead = pRing->rdOcc;                   /* initially set to the maximum # of reads to empty the ring */
        if( numValuesRead == 0U )
        {
            retVal = -1;    /* Ring is empty */
        }
        else
        {
            uint32_t i, localRdIdx = pRing->rdIdx;
            uint64_t *pValsLocal = pVals;
            void     *pRingEntry;

            if( (numValues != 0U) && (numValuesRead > numValues) )
            {
                numValuesRead = numValues;
            }
            /*-----------------------------------------------------------------
             * Perform the specified memory operation only if the ring's
             * physical base address's asel value == 0 (not cache coherent)
             *---------------------------------------------------------------*/
            if( (pfMemOps != NULL) && (pRing->asel == (uint32_t)0U) )
            {
                pRingEntry = (void *)(((uintptr_t)localRdIdx * pRing->elSz) + (uintptr_t)pRing->virtBase);
                (*pfMemOps)((void *)pRingEntry, numValuesRead * sizeof(uint64_t), CSL_LCDMA_RINGACC_MEM_OPS_TYPE_RD);
            }
            for( i=0U; i<numValuesRead; i++, pValsLocal++ )
            {
                pRingEntry = (void *)(((uintptr_t)localRdIdx * pRing->elSz) + (uintptr_t)pRing->virtBase);
                *pValsLocal = *(uint64_t *)pRingEntry;
                localRdIdx++;
                localRdIdx = localRdIdx % pRing->elCnt;
            }
            CSL_lcdma_ringaccAckReverseRing( pCfg, pRing, (int32_t)numValuesRead );    /* This call will update rdOcc and rdIdx elements in pRing */
            if( numValues != numValuesRead )
            {
                retVal = (int32_t)numValuesRead;
            }
        }
    }
    return retVal;
}

static int32_t CSL_lcdma_ringaccPeek64Access( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;

    if( pRing->elSz < sizeof(uint64_t) )
    {
        retVal = -2;        /* Requested access size is greater than ring element size */
    }
    else
    {
        void *pRingEntry;

        pRingEntry = (void *)CSL_lcdma_ringaccGetRingDataPtr( pCfg, pRing );
        if( pRingEntry != NULL )
        {
            /*-----------------------------------------------------------------
             * Perform the specified memory operation only if the ring's
             * physical base address's asel value == 0 (not cache coherent)
             *---------------------------------------------------------------*/
            if( (pfMemOps != NULL) && (pRing->asel == (uint32_t)0U) )
            {
                (*pfMemOps)((void *)pRingEntry, sizeof(uint64_t), CSL_LCDMA_RINGACC_MEM_OPS_TYPE_RD);
            }
            *pVal = *(uint64_t *)pRingEntry;
        }
        else
        {
            *pVal = 0;
            retVal = -1;    /* Ring is empty */
        }
    }
    return retVal;
}

uint32_t CSL_lcdma_ringaccGetRevision( const CSL_LcdmaRingaccCfg *pCfg )
{
    return( 0U );
}

void CSL_lcdma_ringaccInitCfg( CSL_LcdmaRingaccCfg *pCfg )
{
    memset( (void *)pCfg, 0, sizeof(CSL_LcdmaRingaccCfg) );
    pCfg->maxRings = sizeof(CSL_lcdma_ringacc_ringrtRegs) / sizeof(CSL_lcdma_ringacc_ringrtRegs_ring);
}

void CSL_lcdma_ringaccInitRingCfg( CSL_LcdmaRingaccRingCfg *pRingCfg )
{
    /*-------------------------------------------------------------------------
     *  Start by initializing all structure members to 0
     *-----------------------------------------------------------------------*/
    memset( (void *)pRingCfg, 0, sizeof(CSL_LcdmaRingaccRingCfg) );
    pRingCfg->mode = CSL_LCDMA_RINGACC_RING_MODE_TX_RING;
}

void CSL_lcdma_ringaccInitRingObj( uint32_t ringNum,
                             CSL_LcdmaRingaccRingCfg *pRing )
{
    /* Initialize the supplied and state fields */
    pRing->ringNum  = ringNum;
    pRing->wrIdx    = 0;
    pRing->rdIdx    = 0;
    pRing->rdOcc    = 0;
    pRing->wrOcc    = 0;

    return;
}

int32_t CSL_lcdma_ringaccInitRing( CSL_LcdmaRingaccCfg *pCfg,
                             uint32_t ringNum,
                             CSL_LcdmaRingaccRingCfg *pRing )
{
    int32_t retVal = 0;

    if( (pCfg == NULL)                                          ||
        (pRing == NULL)                                         ||
        (pRing->mode == 0U)                                     ||
        (pRing->elCnt == 0U)                                    ||
        (pRing->virtBase == NULL)                               ||
        (!bIsPhysBaseOk(pRing)) )
    {
        retVal = -1;
    }
    else
    {
        uint32_t physBaseHi;

        CSL_lcdma_ringaccInitRingObj(ringNum, pRing);
        /* Ring base address */
        CSL_REG32_WR( &pCfg->pRingCfgRegs->RING[ringNum].BA_LO,
            CSL_FMK(LCDMA_RINGACC_RING_CFG_RING_BA_LO_ADDR_LO, (uint32_t)pRing->physBase ) );
        physBaseHi = (uint32_t)(pRing->physBase >> 32UL);
        pRing->asel = CSL_FEXT( physBaseHi, LCDMA_RINGACC_RING_CFG_RING_BA_HI_ASEL) ;
        CSL_REG32_WR( &pCfg->pRingCfgRegs->RING[ringNum].BA_HI, physBaseHi );
        /* Ring element count */
        CSL_REG32_FINS( &pCfg->pRingCfgRegs->RING[ringNum].SIZE, LCDMA_RINGACC_RING_CFG_RING_SIZE_ELCNT, pRing->elCnt );
        /* Ring credentials */
        CSL_REG32_WR( &pCfg->pCredRegs->RING[ringNum].CRED,
            CSL_FMK( LCDMA_RINGACC_CRED_RING_CRED_CHK_SECURE,   (uint32_t)pRing->credChkSecure )    |
            CSL_FMK( LCDMA_RINGACC_CRED_RING_CRED_SECURE,       (uint32_t)pRing->credSecure )       |
            CSL_FMK( LCDMA_RINGACC_CRED_RING_CRED_PRIV,         (uint32_t)pRing->credPriv   )       |
            CSL_FMK( LCDMA_RINGACC_CRED_RING_CRED_PRIVID,       (uint32_t)pRing->credPrivId ) );
    }
    return retVal;
}

int32_t CSL_lcdma_ringaccSetEvent( CSL_LcdmaRingaccCfg *pCfg,
                             uint32_t ringNum,
                             uint32_t evtNum )
{
    return -1;
}

uint32_t CSL_lcdma_ringaccGetRingNum( const CSL_LcdmaRingaccRingCfg *pRing )
{
    return pRing->ringNum;
}

void CSL_lcdma_ringaccSetRingOrderId( CSL_LcdmaRingaccCfg *pCfg, const CSL_LcdmaRingaccRingCfg *pRing, uint32_t orderId )
{
    return;
}

void CSL_lcdma_ringaccCfgRingCred( CSL_LcdmaRingaccCfg *pCfg, const CSL_LcdmaRingaccRingCfg *pRing, bool bEnable, bool bLock )
{
    return;
}

void CSL_lcdma_ringaccResetRing( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing )
{
    uint32_t regVal;

    /* To reset a ring, write to the ring's BA_LO, BA_HI, or SIZE register */
    regVal = CSL_REG32_RD( &pCfg->pRingCfgRegs->RING[pRing->ringNum].SIZE );
    CSL_REG32_WR( &pCfg->pRingCfgRegs->RING[pRing->ringNum].SIZE, regVal );
    /* Initialize state fields */
    CSL_lcdma_ringaccInitRingObj( pRing->ringNum, pRing );
}

void *CSL_lcdma_ringaccGetForwardRingPtr( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing )
{
    void *ptr = NULL;

    if( CSL_lcdma_ringaccIsRingFull( pRing ) == (bool)true )
    {
        CSL_lcdma_ringaccGetNewElCnt( pCfg, pRing );
    }
    /* Return pointer if not full */
    if( CSL_lcdma_ringaccIsRingFull( pRing ) == (bool)false )
    {
        ptr = (void *)CSL_lcdma_ringaccGetRingWrElementAddr(pRing);
    }
    return ptr;
}

void *CSL_lcdma_ringaccGetReverseRingPtr( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing )
{
    void *ptr = NULL;

    /* If this ring appears empty, then update the occupancy */
    if( CSL_lcdma_ringaccIsRingEmpty( pRing ) == (bool)true )
    {
        CSL_lcdma_ringaccGetNewElCnt( pCfg, pRing );
    }
    /* Return pointer if not empty */
    if( CSL_lcdma_ringaccIsRingEmpty( pRing ) == (bool)false )
    {
        ptr = (void *)CSL_lcdma_ringaccGetRingRdElementAddr(pRing);
    }
    return ptr;
}

uint32_t CSL_lcdma_ringaccGetForwardRingIdx( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum )
{
    return 0U;
}

uint32_t CSL_lcdma_ringaccGetReverseRingIdx( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum )
{
    return 0U;
}

uint32_t CSL_lcdma_ringaccGetForwardRingOcc( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum, CSL_LcdmaRingaccRingMode mode )
{
    uint32_t retVal;

    retVal = CSL_REG32_FEXT( &pCfg->pRingRtRegs->RING[ringNum].FOCC, LCDMA_RINGACC_RINGRT_RING_FOCC_CNT);
    return retVal;
}

uint32_t CSL_lcdma_ringaccGetReverseRingOcc( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum, CSL_LcdmaRingaccRingMode mode )
{
    uint32_t retVal;

    retVal = CSL_REG32_FEXT( &pCfg->pRingRtRegs->RING[ringNum].ROCC, LCDMA_RINGACC_RINGRT_RING_ROCC_CNT);
    return retVal;
}

int32_t CSL_lcdma_ringaccSetTraceEnable( CSL_LcdmaRingaccCfg *pCfg, bool bEnable )
{
    return -1;
}

int32_t CSL_lcdma_ringaccEnableTrace( CSL_LcdmaRingaccCfg *pCfg )
{
    return -1;
}

int32_t CSL_lcdma_ringaccDisableTrace( CSL_LcdmaRingaccCfg *pCfg )
{
    return -1;
}

int32_t CSL_lcdma_ringaccCfgTrace( CSL_LcdmaRingaccCfg *pCfg, bool bTraceAll, bool bIncMsgData, uint32_t ringNum )
{
    return -1;
}

int32_t CSL_lcdma_ringaccCfgRingMonitor( CSL_LcdmaRingaccCfg *pCfg,
                            uint32_t monNum,
                            CSL_LcdmaRingAccMonitorType monType,
                            uint32_t ringNum,
                            uint32_t eventNum,
                            CSL_LcdmaRingAccMonitorDataSrc dataSrc,
                            uint32_t data0Val,
                            uint32_t data1Val )
{
    return -1;
}

int32_t CSL_lcdma_ringaccReadRingMonitor( const CSL_LcdmaRingaccCfg *pCfg, uint32_t monNum, uint32_t *pData0, uint32_t *pData1 )
{
    return -1;
}

int32_t CSL_lcdma_ringaccPush32( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint32_t val, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps)
{
    int32_t retVal = 0;
    uint32_t *pRingEntry;

    pRingEntry = (uint32_t *)CSL_lcdma_ringaccGetForwardRingPtr( pCfg,  pRing );
    if( pRingEntry != NULL )
    {
        *pRingEntry = val;
        CSL_archMemoryFence();
        /*-----------------------------------------------------------------
         * Perform the specified memory operation only if the ring's
         * physical base address's asel value == 0 (not cache coherent)
         *---------------------------------------------------------------*/
        if( (pfMemOps != NULL) && (pRing->asel == (uint32_t)0U) )
        {
            (*pfMemOps)((void *)pRingEntry, sizeof(uint32_t), CSL_LCDMA_RINGACC_MEM_OPS_TYPE_WR);
        }
        CSL_lcdma_ringaccCommitToForwardRing( pCfg, pRing, 1 );
    }
    else
    {
        retVal = -1;    /* Ring is full */
    }
    return retVal;
}

int32_t CSL_lcdma_ringaccPop32( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint32_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;
    uint32_t *pRingEntry;

    pRingEntry = (uint32_t *)CSL_lcdma_ringaccGetReverseRingPtr( pCfg, pRing );
    if( pRingEntry != NULL )
    {
        /*-----------------------------------------------------------------
         * Perform the specified memory operation only if the ring's
         * physical base address's asel value == 0 (not cache coherent)
         *---------------------------------------------------------------*/
        if( (pfMemOps != NULL) && (pRing->asel == (uint32_t)0U) )
        {
            (*pfMemOps)((void *)pRingEntry, sizeof(uint32_t), CSL_LCDMA_RINGACC_MEM_OPS_TYPE_RD);
        }
        *pVal = *pRingEntry;
        CSL_lcdma_ringaccAckReverseRing( pCfg, pRing, 1 );
    }
    else
    {
        *pVal = 0;
        retVal = -1;    /* Ring is empty */
    }
    return retVal;
}

int32_t CSL_lcdma_ringaccHwPop32( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint32_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    return 0;
}

int32_t CSL_lcdma_ringaccPeek32( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint32_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;
    uint32_t *pRingEntry;

    pRingEntry = (uint32_t *)CSL_lcdma_ringaccGetRingDataPtr( pCfg, pRing );
    if( pRingEntry != NULL )
    {
        /*-----------------------------------------------------------------
         * Perform the specified memory operation only if the ring's
         * physical base address's asel value == 0 (not cache coherent)
         *---------------------------------------------------------------*/
        if( (pfMemOps != NULL) && (pRing->asel == (uint32_t)0U) )
        {
            (*pfMemOps)((void *)pRingEntry, sizeof(uint32_t), CSL_LCDMA_RINGACC_MEM_OPS_TYPE_RD);
        }
        *pVal = *pRingEntry;
    }
    else
    {
        *pVal = 0;
        retVal = -1;    /* Ring is empty */
    }
    return retVal;
}

int32_t CSL_lcdma_ringaccPush64( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t val, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    uint64_t *pVal = &val;
    int32_t retVal;

    retVal = CSL_lcdma_ringaccPush64MultiAccess( pCfg, pRing, pVal, (uint32_t)1U, pfMemOps );
    return retVal;
}

int32_t CSL_lcdma_ringaccPush64Multi( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVals, uint32_t numValues, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal;

    retVal = CSL_lcdma_ringaccPush64MultiAccess( pCfg, pRing, pVals, numValues, pfMemOps );
    return retVal;
}

int32_t CSL_lcdma_ringaccPop64( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal;

    retVal = CSL_lcdma_ringaccPop64MultiAccess( pCfg, pRing, pVal, 1U, pfMemOps );
    return retVal;
}

int32_t CSL_lcdma_ringaccPop64Multi( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVals, uint32_t numValues, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal;

    retVal = CSL_lcdma_ringaccPop64MultiAccess( pCfg, pRing, pVals, numValues, pfMemOps );
    return retVal;
}

int32_t CSL_lcdma_ringaccHwPop64( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    return 0;
}

int32_t CSL_lcdma_ringaccPeek64( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal;

    retVal = CSL_lcdma_ringaccPeek64Access( pCfg, pRing, pVal, pfMemOps );
    return retVal;
}

int32_t CSL_lcdma_ringaccWrData( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;
    uint32_t numBytes_local = numBytes;
    uint8_t *pData_local = pData;

    if( numBytes_local <= pRing->elSz )
    {
        uint8_t *pRingEntry;

        pRingEntry = (uint8_t *)CSL_lcdma_ringaccGetForwardRingPtr( pCfg,  pRing );
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
            /*-----------------------------------------------------------------
             * Perform the specified memory operation only if the ring's
             * physical base address's asel value == 0 (not cache coherent)
             *---------------------------------------------------------------*/
            if( (pfMemOps != NULL) && (pRing->asel == (uint32_t)0U) )
            {
                (*pfMemOps)((void *)pRingEntrySave, numBytes, CSL_LCDMA_RINGACC_MEM_OPS_TYPE_WR);
            }
            CSL_lcdma_ringaccCommitToForwardRing( pCfg, pRing, 1 );
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
    return retVal;
}

int32_t CSL_lcdma_ringaccRdData( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;
    uint32_t numBytes_local = numBytes;
    uint8_t *pData_local = pData;

    if( numBytes_local <= pRing->elSz )
    {
        uint8_t *pRingEntry;

        pRingEntry = (uint8_t *)CSL_lcdma_ringaccGetReverseRingPtr( pCfg,  pRing );
        if( pRingEntry != NULL )
        {
            /*-----------------------------------------------------------------
             * Perform the specified memory operation only if the ring's
             * physical base address's asel value == 0 (not cache coherent)
             *---------------------------------------------------------------*/
            if( (pfMemOps != NULL) && (pRing->asel == (uint32_t)0U) )
            {
                (*pfMemOps)((void *)pRingEntry, numBytes_local, CSL_LCDMA_RINGACC_MEM_OPS_TYPE_RD);
            }
            while( numBytes_local != 0U )
            {
                numBytes_local--;
                *pData_local = *pRingEntry;
                pData_local++;
                pRingEntry++;
            }
            CSL_lcdma_ringaccAckReverseRing( pCfg, pRing, 1 );
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
    return retVal;
}

int32_t CSL_lcdma_ringaccPeekData( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint8_t *pData, uint32_t numBytes, CSL_lcdma_ringaccMemOpsFxnPtr pfMemOps )
{
    int32_t retVal = 0;
    uint32_t numBytes_local = numBytes;
    uint8_t *pData_local = pData;

    if( numBytes_local <= pRing->elSz )
    {
        uint8_t *pRingEntry;

        pRingEntry = (uint8_t *)CSL_lcdma_ringaccGetRingRdElementAddr(pRing);
        if (pRingEntry != NULL )
        {
            /*-----------------------------------------------------------------
             * Perform the specified memory operation only if the ring's
             * physical base address's asel value == 0 (not cache coherent)
             *---------------------------------------------------------------*/
            if( (pfMemOps != NULL) && (pRing->asel == (uint32_t)0U) )
            {
                (*pfMemOps)((void *)pRingEntry, numBytes_local, CSL_LCDMA_RINGACC_MEM_OPS_TYPE_RD);
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
    return retVal;
}

uint64_t CSL_lcdma_ringaccClrAselInAddr( uint64_t addr )
{
    uint64_t retAddr = addr;
    retAddr &= ~(((uint64_t)CSL_LCDMA_RINGACC_RING_CFG_RING_BA_HI_ASEL_MASK) << 32U);
    return retAddr;
}

uint64_t CSL_lcdma_ringaccSetAselInAddr( uint64_t addr, CSL_LcdmaRingAccAselEndpoint asel )
{
    uint64_t retAddr;

    retAddr = CSL_lcdma_ringaccClrAselInAddr( addr );
    retAddr |= (((uint64_t)CSL_FMK( LCDMA_RINGACC_RING_CFG_RING_BA_HI_ASEL, asel )) << 32U);
    return retAddr;
}

bool CSL_lcdma_ringaccIsTeardownComplete( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum )
{
    bool bRetVal = (bool)false;

#ifdef CSL_LCDMA_RINGACC_RINGRT_RING_ROCC_COMP_MASK
    if( CSL_REG32_FEXT( &pCfg->pRingRtRegs->RING[ringNum].ROCC, LCDMA_RINGACC_RINGRT_RING_ROCC_COMP ) != (uint32_t)0U )
    {
        bRetVal = (bool)true;
    }
#endif
    return bRetVal;
}

void CSL_lcdma_ringaccAckTeardown( const CSL_LcdmaRingaccCfg *pCfg, uint32_t ringNum )
{
#ifdef CSL_LCDMA_RINGACC_RINGRT_RING_RDB_ACK_MASK
    CSL_REG32_WR( &pCfg->pRingRtRegs->RING[ringNum].RDB, CSL_FMK( LCDMA_RINGACC_RINGRT_RING_RDB_ACK, (uint32_t)1U ) );
#endif
}

int32_t CSL_lcdma_ringaccDequeue( CSL_LcdmaRingaccCfg *pCfg, CSL_LcdmaRingaccRingCfg *pRing, uint64_t *pVal )
{
    int32_t retVal;

    if( pRing->wrOcc )
    {
        uint32_t fifoIdx;

        /*---------------------------------------------------------------------
         * Determine FIFO ring value index and return value at this index
         *-------------------------------------------------------------------*/
        if( pRing->wrIdx < pRing->wrOcc )
        {
            fifoIdx = pRing->elCnt - (pRing->wrOcc - pRing->wrIdx);
        }
        else
        {
            fifoIdx = pRing->wrIdx - pRing->wrOcc;
        }
        *pVal = *(uint64_t *)(((uintptr_t)fifoIdx * pRing->elSz) + (uintptr_t)pRing->virtBase);
        /*---------------------------------------------------------------------
         * Shift all ring elements down one index
         *-------------------------------------------------------------------*/
        if( pRing->wrOcc > 1U )
        {
            uint32_t i, dstIdx, srcIdx;
            uint64_t *pDst, *pSrc;

            dstIdx = fifoIdx;
            srcIdx = dstIdx + 1U;
            if( srcIdx == pRing->elCnt ) { srcIdx = 0U; }
            for( i=0U; i<(pRing->wrOcc-1U); i++ )
            {
                pDst = (uint64_t *)(((uintptr_t)dstIdx * pRing->elSz) + (uintptr_t)pRing->virtBase);
                pSrc = (uint64_t *)(((uintptr_t)srcIdx * pRing->elSz) + (uintptr_t)pRing->virtBase);
                *pDst = *pSrc;
                dstIdx++;
                if( dstIdx == pRing->elCnt ) { dstIdx = 0U; }
                srcIdx++;
                if( srcIdx == pRing->elCnt ) { srcIdx = 0U; }
            }
        }
        /*---------------------------------------------------------------------
         * Adjust FOCC and local wrOcc and wrIdx states
         *-------------------------------------------------------------------*/
        if( CSL_lcdma_ringaccGetForwardRingOcc( pCfg, pRing->ringNum, 0U ) > (uint32_t)0U )
        {
            CSL_lcdma_ringaccSetForwardDoorbell( pCfg, pRing->ringNum, 0U, -1 );
        }
        pRing->wrOcc--;
        if( pRing->wrIdx == 0U ) { pRing->wrIdx = pRing->elCnt - 1U; } else { pRing->wrIdx--; }
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}
