/**
 * @file  csl_intaggr.c
 *
 * @brief
 *  C implementation file for the INTAGGR module CSL-FL.
 *
 *  Contains the different control command and status query functions definitions
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
#include <stddef.h>
#include <drivers/udma/hw_include/csl_intaggr.h>

/* Static functions */
static bool CSL_intaggrIsIntrModeStatusBitNum( uint32_t statusBitNum );
static bool CSL_intaggrIsValidStatusBitNum( const CSL_IntaggrCfg *pCfg, uint32_t statusBitNum );

static bool CSL_intaggrIsIntrModeStatusBitNum( uint32_t statusBitNum )
{
    bool bRetVal;

    if( statusBitNum & CSL_INTAGGR_INTR_MODE_FLAG )
    {
        bRetVal = (bool)true;
    }
    else
    {
        bRetVal = (bool)false;
    }
    return bRetVal;
}

static bool CSL_intaggrIsValidStatusBitNum( const CSL_IntaggrCfg *pCfg, uint32_t statusBitNum )
{
    bool bRetVal;
    uint32_t localStatusBitNum = statusBitNum;

    localStatusBitNum &= ~CSL_INTAGGR_INTR_MODE_FLAG;   /* Remove intr mode flag */
    if( localStatusBitNum < (pCfg->virtIntrCnt << 6U) )
    {
        bRetVal = (bool)true;
    }
    else
    {
        bRetVal = (bool)false;
    }
    return bRetVal;
}


#ifdef CSL_INTAGGR_CFG_AUXCAP_UNMAP_CNT_MASK
static bool CSL_intaggrIsModuleRevAtLeast( const CSL_IntaggrCfg *pCfg, uint32_t majorRev, uint32_t minorRev, uint32_t rtlRev );
static uint32_t CSL_intaggrBuildMapIdxValue( uint32_t statusBitNum );

static bool CSL_intaggrIsModuleRevAtLeast( const CSL_IntaggrCfg *pCfg, uint32_t majorRev, uint32_t minorRev, uint32_t rtlRev )
{
    bool bRetVal = (bool)false;
    uint32_t encodedRevVals;
    static uint32_t encodedPid = 0U;

    if( encodedPid == 0U )
    {
        uint32_t pid;

        pid = CSL_intaggrGetRevision( pCfg );
        encodedPid = (((pid & 0x0700U) << 3U)  | ((pid & 0x003FU) << 5U) | ((pid & 0xF800U) >> 11U));
    }
    encodedRevVals  = (((majorRev & 0x0007U) << 11U) | ((minorRev & 0x003FU) << 5U) | ((rtlRev & 0x001FU) >> 0));
    if( encodedPid >= encodedRevVals )
    {
        bRetVal = (bool)true;
    }
    return bRetVal;
}

static uint32_t CSL_intaggrBuildMapIdxValue( uint32_t statusBitNum )
{
    uint32_t mapIdxVal;

    /* statusBitNum is already in the correct format: bitnum in 5:0 and regnum in 14:6
       We simply need to limit value to bits [14:0] */
    mapIdxVal = statusBitNum & (uint32_t)0x7FFFU;
    return mapIdxVal;
}
#endif

/* API functions */

void CSL_intaggrGetCfg( CSL_IntaggrCfg *pCfg )
{
    uint64_t regVal;

    regVal = CSL_REG64_RD( &pCfg->pCfgRegs->INTCAP );
    pCfg->srcEventCnt       = (uint32_t)CSL_FEXT( regVal, INTAGGR_CFG_INTCAP_SEVT_CNT );
    pCfg->virtIntrCnt       = (uint32_t)CSL_FEXT( regVal, INTAGGR_CFG_INTCAP_VINTR_CNT );
    regVal = CSL_REG64_RD( &pCfg->pCfgRegs->AUXCAP );
    pCfg->localEventCnt     = (uint32_t)CSL_FEXT( regVal, INTAGGR_CFG_AUXCAP_LEVI_CNT );
    pCfg->globalEventCnt    = (uint32_t)CSL_FEXT( regVal, INTAGGR_CFG_AUXCAP_GEVI_CNT );
    pCfg->mcastEventCnt     = (uint32_t)CSL_FEXT( regVal, INTAGGR_CFG_AUXCAP_MEVI_CNT );
#ifdef CSL_INTAGGR_CFG_AUXCAP_UNMAP_CNT_MASK
    pCfg->unmapEventCnt     = (uint32_t)CSL_FEXT( regVal, INTAGGR_CFG_AUXCAP_UNMAP_CNT );
#else
    pCfg->unmapEventCnt     = 0U;
#endif
}

uint32_t CSL_intaggrGetRevision( const CSL_IntaggrCfg *pCfg )
{
    return (uint32_t)CSL_REG64_RD( &pCfg->pCfgRegs->REVISION );
}

int32_t CSL_intaggrMapEventIntr( CSL_IntaggrCfg *pCfg, uint32_t globalEventIdx, uint32_t statusBitNum )
{
    int32_t  retVal = CSL_EBADARGS;
    uint64_t regVal;
    uint32_t regNum, bitNum;

    if( (globalEventIdx < pCfg->srcEventCnt ) && CSL_intaggrIsValidStatusBitNum(pCfg, statusBitNum) )
    {
        regNum = statusBitNum >> 6U;
        bitNum = statusBitNum & (uint32_t)0x003FU;
        regVal = CSL_FMK( INTAGGR_IMAP_GEVI_IMAP_REGNUM, (uint64_t)regNum )   |
                 CSL_FMK( INTAGGR_IMAP_GEVI_IMAP_BITNUM, (uint64_t)bitNum );
        CSL_REG64_WR( &pCfg->pImapRegs->GEVI[globalEventIdx].IMAP, regVal );
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_intaggrMapEventToLocalEvent( CSL_IntaggrCfg *pCfg, uint32_t globalEventIdx, uint32_t localEventIdx, CSL_IntaggrEventDetectMode localEventDetectMode )
{
    int32_t  retVal;
    uint64_t regVal;

    if( (pCfg->pL2gRegs == NULL) || (pCfg->localEventCnt == 0U) )
    {
        retVal = CSL_EFAIL;    /* INTAGGR does not support this feature */
    }
    else if( (localEventIdx >= pCfg->localEventCnt)     ||
             (localEventDetectMode >= CSL_INTAGGR_EVT_DETECT_MODE_INVALID) )
    {
        retVal = CSL_EBADARGS;    /* Invalid localEventIdx or localEventDetectMode parameter */
    }
    else
    {
        regVal = CSL_FMK( INTAGGR_L2G_LEVI_MAP_MODE, (uint64_t)localEventDetectMode )   |
                 CSL_FMK( INTAGGR_L2G_LEVI_MAP_GEVIDX, (uint64_t)globalEventIdx );
        CSL_REG64_WR( &pCfg->pL2gRegs->LEVI[localEventIdx].MAP, regVal );
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_intaggrMapEventRxCntEvent( CSL_IntaggrCfg *pCfg, uint32_t globalEventIdx, uint32_t globalEventOutIdx )
{
    int32_t  retVal;

    if( (pCfg->pGcntCfgRegs == NULL) || (pCfg->globalEventCnt == 0U) )
    {
        retVal = CSL_EFAIL;    /* INTAGGR does not support this feature */
    }
    else if( globalEventIdx >= pCfg->globalEventCnt )
    {
        retVal = CSL_EBADARGS;    /* Invalid globalEventIdx parameter */
    }
    else
    {
        uint64_t regVal;

        if( CSL_intaggrIsIntrModeStatusBitNum( globalEventOutIdx ) )
        {
#ifdef CSL_INTAGGR_GCNTCFG_GEVI_MAP_IRQMODE_MASK
            if( !CSL_intaggrIsModuleRevAtLeast(pCfg, 1U, 1U, 0U) )
            {
                retVal = CSL_EFAIL;    /* INTAGGR does not support this feature */
            }
            else
            {
                if( CSL_intaggrIsValidStatusBitNum(pCfg, globalEventOutIdx) )
                {
                    regVal = CSL_FMK( INTAGGR_GCNTCFG_GEVI_MAP_IRQMODE, (uint64_t)1UL )      |
                             CSL_FMK( INTAGGR_GCNTCFG_GEVI_MAP_GEVIDX,  (uint64_t)CSL_intaggrBuildMapIdxValue(globalEventOutIdx) );
                    CSL_REG64_WR( &pCfg->pGcntCfgRegs->GEVI[globalEventIdx].MAP, regVal );
                    retVal = CSL_PASS;
                }
                else
                {
                    retVal = CSL_EBADARGS;    /* Invalid globalEventOutIdx parameter */
                }
            }
#else
            retVal = CSL_EFAIL;        /* INTAGGR CSL-RL does not support this feature */
#endif
        }
        else
        {
            regVal = CSL_FMK( INTAGGR_GCNTCFG_GEVI_MAP_GEVIDX, (uint64_t)globalEventOutIdx );
            CSL_REG64_WR( &pCfg->pGcntCfgRegs->GEVI[globalEventIdx].MAP, regVal );
            retVal = CSL_PASS;
        }
    }
    return retVal;
}

int32_t CSL_intaggrRdEventRxCnt( const CSL_IntaggrCfg *pCfg, uint32_t globalEventIdx, uint32_t *pCnt )
{
    int32_t  retVal;

    if( (pCfg->pGcntCfgRegs == NULL) || (pCfg->globalEventCnt == 0U) )
    {
        retVal = CSL_EFAIL;    /* INTAGGR does not support this feature */
    }
    else if( globalEventIdx >= pCfg->globalEventCnt )
    {
        retVal = CSL_EBADARGS;    /* Invalid globalEventIdx parameter */
    }
    else
    {
        uint64_t regVal;

        regVal = CSL_REG64_RD( &pCfg->pGcntRtiRegs->GEVI[globalEventIdx].COUNT );
        *pCnt  = (uint32_t)CSL_FEXT( regVal, INTAGGR_GCNTRTI_GEVI_COUNT_CCNT );
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_intaggrWrEventRxCnt( CSL_IntaggrCfg *pCfg, uint32_t globalEventIdx, uint32_t cnt )
{
    int32_t  retVal;

    if( (pCfg->pGcntCfgRegs == NULL) || (pCfg->globalEventCnt == 0U) )
    {
        retVal = CSL_EFAIL;    /* INTAGGR does not support this feature */
    }
    else if( globalEventIdx >= pCfg->globalEventCnt )
    {
        retVal = CSL_EBADARGS;    /* Invalid globalEventIdx parameter */
    }
    else
    {
        uint64_t regVal;

        regVal = CSL_FMK( INTAGGR_GCNTRTI_GEVI_COUNT_CCNT, (uint64_t)cnt );
        CSL_REG64_WR( &pCfg->pGcntRtiRegs->GEVI[globalEventIdx].COUNT, regVal );
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_intaggrEnableEventMulticast( CSL_IntaggrCfg *pCfg, uint32_t globalEventIdx, uint32_t globalEventOutIdx0, uint32_t globalEventOutIdx1 )
{
    int32_t  retVal;

    if( (pCfg->pMcastRegs == NULL) || (pCfg->mcastEventCnt == 0U) )
    {
        retVal = CSL_EFAIL;    /* INTAGGR does not support this feature */
    }
    else if( globalEventIdx >= pCfg->mcastEventCnt )
    {
        retVal = CSL_EBADARGS;    /* Invalid globalEventIdx parameter */
    }
    else
    {
        uint64_t regVal;

        if( CSL_intaggrIsIntrModeStatusBitNum(globalEventOutIdx0) || CSL_intaggrIsIntrModeStatusBitNum(globalEventOutIdx1) )
        {
#ifdef CSL_INTAGGR_GCNTCFG_GEVI_MAP_IRQMODE_MASK
            uint64_t idx0Val    = (uint64_t)globalEventOutIdx0;
            uint64_t mode0Val   = 0UL;
            uint64_t idx1Val    = (uint64_t)globalEventOutIdx1;
            uint64_t mode1Val   = 0UL;

            if( !CSL_intaggrIsModuleRevAtLeast(pCfg, 1U, 1U, 0U) )
            {
                retVal = CSL_EFAIL;    /* INTAGGR does not support this feature */
            }
            else
            {
                retVal = CSL_PASS;      /* Assume OK at this point */
                if( CSL_intaggrIsIntrModeStatusBitNum(globalEventOutIdx0) )
                {
                    if( CSL_intaggrIsValidStatusBitNum(pCfg, globalEventOutIdx0) )
                    {
                        idx0Val  = (uint64_t)CSL_intaggrBuildMapIdxValue(globalEventOutIdx0);
                        mode0Val = 1UL;
                    }
                    else
                    {
                        retVal = CSL_EBADARGS;    /* Invalid globalEventOutIdx0 parameter */
                    }
                }
                if( CSL_intaggrIsIntrModeStatusBitNum(globalEventOutIdx1) )
                {
                    if( CSL_intaggrIsValidStatusBitNum(pCfg, globalEventOutIdx1) )
                    {
                        idx1Val  = (uint64_t)CSL_intaggrBuildMapIdxValue(globalEventOutIdx1);
                        mode1Val = 1UL;
                    }
                    else
                    {
                        retVal = CSL_EBADARGS;    /* Invalid globalEventOutIdx1 parameter */
                    }
                }
                if( retVal == CSL_PASS )
                {
                    regVal = CSL_FMK( INTAGGR_MCAST_GEVI_MCMAP_IRQMODE1, mode1Val ) |
                             CSL_FMK( INTAGGR_MCAST_GEVI_MCMAP_GEVIDX1,  idx1Val  ) |
                             CSL_FMK( INTAGGR_MCAST_GEVI_MCMAP_IRQMODE0, mode0Val ) |
                             CSL_FMK( INTAGGR_MCAST_GEVI_MCMAP_GEVIDX0,  idx0Val  );
                    CSL_REG64_WR( &pCfg->pGcntCfgRegs->GEVI[globalEventIdx].MAP, regVal );
                }
            }
#else
            retVal = CSL_EFAIL;        /* INTAGGR CSL-RL does not support this feature */
#endif
        }
        else
        {
            regVal = CSL_FMK( INTAGGR_MCAST_GEVI_MCMAP_GEVIDX0, (uint64_t)globalEventOutIdx0 )   |
                     CSL_FMK( INTAGGR_MCAST_GEVI_MCMAP_GEVIDX1, (uint64_t)globalEventOutIdx1 );
            CSL_REG64_WR( &pCfg->pMcastRegs->GEVI[globalEventIdx].MCMAP, regVal );
            retVal = CSL_PASS;
        }
    }
    return retVal;
}

int32_t CSL_intaggrMapUnmappedEventToEvent( CSL_IntaggrCfg *pCfg, uint32_t unmappedEventIdx, uint32_t globalEventOutIdx )
{
    int32_t  retVal;

#ifdef CSL_INTAGGR_UNMAP_UNMAP_MAP_MAPIDX_MASK
    if( (!CSL_intaggrIsModuleRevAtLeast(pCfg, 1U, 1U, 0U)) || (pCfg->pUnmapRegs == NULL) || (pCfg->unmapEventCnt == 0U) )
    {
        retVal = CSL_EFAIL;    /* INTAGGR does not support this feature */
    }
    else if( unmappedEventIdx >= pCfg->unmapEventCnt )
    {
        retVal = CSL_EBADARGS;    /* Invalid unmappedEventIdx parameter */
    }
    else
    {
        uint64_t regVal;

        if( CSL_intaggrIsIntrModeStatusBitNum(globalEventOutIdx) )
        {
            if( CSL_intaggrIsValidStatusBitNum(pCfg, globalEventOutIdx) )
            {
                regVal = CSL_FMK( INTAGGR_UNMAP_UNMAP_MAP_IRQMODE, (uint64_t)1UL )      |
                         CSL_FMK( INTAGGR_UNMAP_UNMAP_MAP_MAPIDX, (uint64_t)CSL_intaggrBuildMapIdxValue(globalEventOutIdx) );
                CSL_REG64_WR( &pCfg->pUnmapRegs->UNMAP[unmappedEventIdx].MAP, regVal );
                retVal = CSL_PASS;
            }
            else
            {
                retVal = CSL_EBADARGS;    /* Invalid globalEventOutIdx parameter */
            }
        }
        else
        {
            regVal = CSL_FMK( INTAGGR_UNMAP_UNMAP_MAP_MAPIDX, (uint64_t)globalEventOutIdx );
            CSL_REG64_WR( &pCfg->pUnmapRegs->UNMAP[unmappedEventIdx].MAP, regVal );
            retVal = CSL_PASS;
        }
    }
#else
    retVal = CSL_EFAIL;        /* INTAGGR CSL-RL does not support this feature */
#endif
    return retVal;
}

int32_t CSL_intaggrSetIntrEnable( CSL_IntaggrCfg *pCfg, uint32_t statusBitNum, bool bEnable )
{
    int32_t  retVal = CSL_EFAIL;
    uint64_t regVal;
    uint32_t regNum, bitNum;

    if( CSL_intaggrIsValidStatusBitNum(pCfg, statusBitNum) )
    {
        regNum = statusBitNum >> 6U;
        bitNum = statusBitNum & (uint32_t)0x003FU;
        if( bEnable == (bool)true )
        {
            regVal = CSL_REG64_RD( &pCfg->pIntrRegs->VINT[regNum].ENABLE_SET );
            regVal |= (((uint64_t)1U) << bitNum);
            CSL_REG64_WR( &pCfg->pIntrRegs->VINT[regNum].ENABLE_SET, regVal );
        }
        else
        {
            regVal = (((uint64_t)1U) << bitNum);
            CSL_REG64_WR( &pCfg->pIntrRegs->VINT[regNum].ENABLE_CLEAR, regVal );
        }
        retVal = CSL_PASS;
    }
    return retVal;
}

int32_t CSL_intaggrSetIntrPending( CSL_IntaggrCfg *pCfg, uint32_t statusBitNum )
{
    int32_t  retVal = CSL_EFAIL;
    uint64_t regVal;
    uint32_t regNum, bitNum;

    if( CSL_intaggrIsValidStatusBitNum(pCfg, statusBitNum) )
    {
        regNum = statusBitNum >> 6U;
        bitNum = statusBitNum & (uint32_t)0x003FU;
        regVal = (((uint64_t)1U) << bitNum);
#ifdef CSL_INTAGGR_INTR_VINT_STATUS_SET_STATUS_MASK
        CSL_REG64_WR( &pCfg->pIntrRegs->VINT[regNum].STATUS_SET, regVal );
#else
        CSL_REG64_WR( &pCfg->pIntrRegs->VINT[regNum].STATUS, regVal );
#endif
        retVal = CSL_PASS;
    }
    return retVal;
}

bool CSL_intaggrIsIntrPending( const CSL_IntaggrCfg *pCfg, uint32_t statusBitNum, bool bMaskedStatus )
{
    bool     retVal = (bool)false;
    uint64_t regVal;
    uint32_t regNum, bitNum;

    if( CSL_intaggrIsValidStatusBitNum(pCfg, statusBitNum) )
    {
        regNum = statusBitNum >> 6U;
        bitNum = statusBitNum & (uint32_t)0x003FU;
        if( bMaskedStatus == (bool)true )
        {
            regVal = CSL_REG64_RD( &pCfg->pIntrRegs->VINT[regNum].STATUSM );
        }
        else
        {
#ifdef CSL_INTAGGR_INTR_VINT_STATUS_SET_STATUS_MASK
            regVal = CSL_REG64_RD( &pCfg->pIntrRegs->VINT[regNum].STATUS_SET );
#else
            regVal = CSL_REG64_RD( &pCfg->pIntrRegs->VINT[regNum].STATUS );
#endif
        }
        if( (regVal & (((uint64_t)1U) << bitNum)) == 0U )
        {
            retVal = (bool)false; /* Interrupt is not pending */
        }
        else
        {
            retVal = (bool)true;  /* Interrupt is pending */
        }
    }
    return retVal;
}

int32_t CSL_intaggrClrIntr( CSL_IntaggrCfg *pCfg, uint32_t statusBitNum )
{
    int32_t  retVal = CSL_EFAIL;
    uint64_t regVal;
    uint32_t regNum, bitNum;

    if( CSL_intaggrIsValidStatusBitNum(pCfg, statusBitNum) )
    {
        regNum = statusBitNum >> 6U;
        bitNum = statusBitNum & (uint32_t)0x003FU;
        regVal = (((uint64_t)1U) << bitNum);
#ifdef CSL_INTAGGR_INTR_VINT_STATUS_CLEAR_STATUS_MASK
        CSL_REG64_WR( &pCfg->pIntrRegs->VINT[regNum].STATUS_CLEAR, regVal );
#else
        CSL_REG64_WR( &pCfg->pIntrRegs->VINT[regNum].STATUS, regVal );
#endif
        retVal = CSL_PASS;
    }
    return retVal;
}
