/*
 * @file  sdl_ecc_aggr.c
 *
 * @brief
 *  C implementation file for the ECC Aggregator module SDL.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2015-2024, Texas Instruments, Inc.
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
#include <sdl/ecc/V0/sdl_ip_ecc.h>
#include <sdl/ecc/V0/sdlr_edc_ctl.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/soc_config.h>

#define     SDL_TWO_POWER_TWO_VALUE          (4U)

/*===========================================================================*/
/*  Internal Functions                                                       */
/*===========================================================================*/
static bool SDL_ecc_aggrIsSVBUSRegReadDone(const SDL_ecc_aggrRegs *pEccAggrRegs);
static bool SDL_ecc_aggrIsValidRamId(const SDL_ecc_aggrRegs *pEccAggrRegs,
                                     uint32_t ramId);
static bool SDL_ecc_aggrIsValidEccRamRegOffset(uint32_t regOffset);
static bool SDL_ecc_aggrIsValidEDCInterconnectRegOffset(uint32_t regOffset);
static bool SDL_ecc_aggrIsValidIntrSrc(uint32_t intrSrc);
static bool SDL_ecc_aggrIsValidInstSel(uint32_t instSelect, uint32_t maxInst);
static bool SDL_ecc_aggrToggleEccRamIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
                                                uint32_t ramId,
                                                SDL_Ecc_AggrIntrSrc intrSrc,
                                                uint32_t numEvents, bool bSet);
static bool SDL_ecc_aggrToggleEDCInterconnectIntrPending(
    SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc,
    SDL_Ecc_AggrEDCErrorSubType subType, uint32_t numEvents, bool bSet);
static bool SDL_ecc_aggrToggleIntrEnable(const SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, bool bEnable);
static bool SDL_ecc_aggrToggleIntrsEnable(const SDL_ecc_aggrRegs *pEccAggrRegs,
    SDL_Ecc_AggrIntrSrc intrSrc, bool bEnable);
static uintptr_t SDL_ecc_aggrStatusRegAddr(uintptr_t base, uint32_t n, uint32_t et);
static uintptr_t SDL_ecc_aggrSecStatusRegAddr(uintptr_t base, uint32_t n);
static uintptr_t SDL_ecc_aggrDedStatusRegAddr(uintptr_t base, uint32_t n);
static uintptr_t SDL_ecc_aggrDedEnableSetRegAddr(uintptr_t base, uint32_t n);
static uintptr_t SDL_ecc_aggrSecEnableSetRegAddr(uintptr_t base, uint32_t n);
static uintptr_t SDL_ecc_aggrDedEnableClrRegAddr(uintptr_t base, uint32_t n);
static uintptr_t SDL_ecc_aggrSecEnableClrRegAddr(uintptr_t base, uint32_t n);
static uintptr_t SDL_ecc_aggrEnableSetRegAddr(uintptr_t base, uint32_t n, uint32_t et);
static uintptr_t SDL_ecc_aggrEnableClrRegAddr(uintptr_t base, uint32_t n, uint32_t et);
static void SDL_ecc_aggrReadSVBUSReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t regOffset, uint32_t *pRegVal );
static void SDL_ecc_aggrWriteSVBUSReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t regOffset, uint32_t val );


static uintptr_t SDL_ecc_aggrDedEnableClrRegAddr(uintptr_t base, uint32_t n)
{
    return ((base)+SDL_ECC_AGGR_DED_ENABLE_CLR_REG0+ (uint32_t)((n)*((uint32_t)4U)));
}

static uintptr_t SDL_ecc_aggrSecEnableClrRegAddr(uintptr_t base, uint32_t n)
{
    return ((base)+SDL_ECC_AGGR_SEC_ENABLE_CLR_REG0+ (uint32_t)((n)*((uint32_t)4U)));
}

static uintptr_t SDL_ecc_aggrDedEnableSetRegAddr(uintptr_t base, uint32_t n)
{
    return ((base)+SDL_ECC_AGGR_DED_ENABLE_SET_REG0+ (uint32_t)((n)*((uint32_t)4U)));
}

static uintptr_t SDL_ecc_aggrSecEnableSetRegAddr(uintptr_t base, uint32_t n)
{
    return ((base)+SDL_ECC_AGGR_SEC_ENABLE_SET_REG0+ (uint32_t)((n)*((uint32_t)4U)));
}

static uintptr_t SDL_ecc_aggrEnableSetRegAddr(uintptr_t base, uint32_t n, uint32_t et)
{
    uintptr_t addr;

    if (et == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT)
    {
        addr = SDL_ecc_aggrSecEnableSetRegAddr(base,n);
    }
    else
    {
        addr = SDL_ecc_aggrDedEnableSetRegAddr(base,n);
    }
    return (addr);
}

static uintptr_t SDL_ecc_aggrEnableClrRegAddr(uintptr_t base, uint32_t n, uint32_t et)
{
    uintptr_t addr;

    if (et == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT)
    {
        addr = SDL_ecc_aggrSecEnableClrRegAddr(base,n);
    }
    else
    {
        addr = SDL_ecc_aggrDedEnableClrRegAddr(base,n);
    }
    return (addr);
}

static uintptr_t  SDL_ecc_aggrSecStatusRegAddr(uintptr_t base, uint32_t n)
{
    return ((base)+SDL_ECC_AGGR_SEC_STATUS_REG0+(uint32_t)((n)*((uint32_t)4U)));
}

static uintptr_t  SDL_ecc_aggrDedStatusRegAddr(uintptr_t base, uint32_t n)
{
    return ((base)+SDL_ECC_AGGR_DED_STATUS_REG0+(uint32_t)((n)*((uint32_t)4U)));
}

static uintptr_t SDL_ecc_aggrStatusRegAddr(uintptr_t base, uint32_t n, uint32_t et)
{
    return (((et)==SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT)? \
                SDL_ecc_aggrSecStatusRegAddr(base,n): \
                SDL_ecc_aggrDedStatusRegAddr(base,n));
}

static bool SDL_ecc_aggrIsSVBUSRegReadDone(const SDL_ecc_aggrRegs *pEccAggrRegs)
{
    bool retVal = (bool)false;

    if( SDL_REG32_FEXT(&pEccAggrRegs->VECTOR, ECC_AGGR_VECTOR_RD_SVBUS_DONE) == (uint32_t)1U )
    {
        retVal = (bool)true;
    }
    return retVal;
}

static bool SDL_ecc_aggrIsValidRamId(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId)
{
    bool       retVal = (bool)false;
    uint32_t   numRams;
    int32_t    sdlRet;

    /* read number of Rams*/
    sdlRet = SDL_ecc_aggrGetNumRams(pEccAggrRegs, &numRams);

    if (sdlRet == SDL_PASS)
    {
        if( ramId < numRams )
        {
            retVal = (bool)true;
        }
    }
    return retVal;
}

static bool SDL_ecc_aggrIsValidEccRamRegOffset(uint32_t regOffset)
{
    bool retVal = (bool)true;

    /* Check bounds of register offset for ECC wrapper */
    if( (regOffset < SDL_ECC_RAM_WRAP_REV) ||
        (regOffset > SDL_ECC_RAM_ERR_STAT3) )
    {
        retVal = (bool)false;
    }
    return retVal;
}

static bool SDL_ecc_aggrIsValidEDCInterconnectRegOffset(uint32_t regOffset)
{
    bool retVal = (bool)true;

    /* Check bounds of register offset for EDC Interconnect*/
    if( (regOffset < SDL_EDC_CTL_REVISION) ||
        (regOffset > SDL_EDC_CTL_ERR_STATUS2) )
    {
        retVal = (bool)false;
    }
    return retVal;
}

static bool SDL_ecc_aggrIsValidIntrSrc(uint32_t intrSrc)
{
    bool retVal = (bool)true;

    if( (intrSrc < SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT)      ||
        (intrSrc > SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT) )
    {
        retVal = (bool)false;
    }

    return retVal;
}

static bool SDL_ecc_aggrIsValidInstSel(uint32_t instSelect, uint32_t maxInst)
{
    bool retVal = (bool)true;

    if( instSelect >= maxInst )
    {
        retVal = (bool)false;
    }
    return retVal;
}

static bool SDL_ecc_aggrToggleEccRamIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, uint32_t numEvents, bool bSet)
{
    bool retVal = (bool)false;

    if( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true )
    {
        uint32_t regVal;

        retVal = (bool)true;

        /* Set the appropriate bits to set or unset selected event bits */
        if( intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT )
        {
            if( bSet == (bool)true )
            {
                regVal = SDL_FMK(ECC_RAM_ERR_STAT1_ECC_SEC, (uint32_t)numEvents);
            }
            else
            {
                regVal = SDL_FMK(ECC_RAM_ERR_STAT1_CLR_ECC_SEC, (uint32_t)numEvents);
            }
        }
        else if( intrSrc == SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT )
        {
            if( bSet == (bool)true )
            {
                regVal = SDL_FMK(ECC_RAM_ERR_STAT1_ECC_DED, (uint32_t)numEvents);
            }
            else
            {
                regVal = SDL_FMK(ECC_RAM_ERR_STAT1_CLR_ECC_DED, (uint32_t)numEvents);
            }
        }
        else if( intrSrc == SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS )
        {
            if( bSet == (bool)true )
            {
                regVal = SDL_FMK(ECC_RAM_ERR_STAT1_ECC_OTHER, (uint32_t)numEvents);
            }
            else
            {
                regVal = SDL_FMK(ECC_RAM_ERR_STAT1_CLR_ECC_OTHER, (uint32_t)numEvents);
            }
        }
        else
        {
            retVal = (bool)false;
        }
        if( retVal == (bool)true )
        {
            /* Write the bitmap to the Status register */
            SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_ECC_RAM_ERR_STAT1, regVal);

            /* Readback to make sure write is complete */
            SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_ECC_RAM_ERR_STAT1, &regVal);
        }
    }

    return retVal;
}

static bool SDL_ecc_aggrToggleEDCInterconnectIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc,
    SDL_Ecc_AggrEDCErrorSubType subType, uint32_t numEvents, bool bSet)
{
    bool retVal = (bool)false;

    /* Check for valid RAM Id */
    if( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true )
    {
        uint32_t regVal;

        retVal = (bool)true;

        /* Set the appropriate bits to set or unset selected event bits */
        if( intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT )
        {
            if( bSet == (bool)true )
            {
                if (subType == SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT)
                {
                    regVal = SDL_FMK(EDC_CTL_ERR_STATUS1_INJ_COR_PEND, (uint32_t)numEvents);
                }
                else
                {
                    regVal = SDL_FMK(EDC_CTL_ERR_STATUS1_COR_PEND, (uint32_t)numEvents);
                }
            }
            else
            {
                if (subType == SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT)
                {
                    regVal = SDL_FMK(EDC_CTL_ERR_STATUS1_INJ_COR_PEND_CLR, (uint32_t)numEvents);
                }
                else
                {
                    regVal = SDL_FMK(EDC_CTL_ERR_STATUS1_COR_PEND_CLR, (uint32_t)numEvents);
                }
            }
        }
        else if( intrSrc == SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT )
        {
            if( bSet == (bool)true )
            {
                if (subType == SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT)
                {
                    regVal = SDL_FMK(EDC_CTL_ERR_STATUS1_INJ_UNC_PEND, (uint32_t)numEvents);
                }
                else
                {
                    regVal = SDL_FMK(EDC_CTL_ERR_STATUS1_UNC_PEND, (uint32_t)numEvents);
                }
            }
            else
            {
                if (subType == SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT)
                {
                    regVal = SDL_FMK(EDC_CTL_ERR_STATUS1_INJ_UNC_PEND_CLR, (uint32_t)numEvents);
                }
                else
                {
                    regVal = SDL_FMK(EDC_CTL_ERR_STATUS1_UNC_PEND_CLR, (uint32_t)numEvents);
                }
            }
        }
        else
        {
            retVal = (bool)false;
        }

        if( retVal == (bool)true )
        {
            /* Write the bitmap to the Status register */
            SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_ERR_STATUS1, regVal);
            /* Readback to make sure write is complete */
            SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_ERR_STATUS1, &regVal);
        }
    }

    return retVal;
}

static bool SDL_ecc_aggrToggleIntrEnable(const SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, bool bEnable)
{
    bool retVal = (bool)true;

    if( (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool) true ) &&
        (SDL_ecc_aggrIsValidIntrSrc(intrSrc)           == (bool) true ) )
    {
        if(bEnable == (bool)true )
        {
            SDL_REG32_WR( SDL_ecc_aggrEnableSetRegAddr( (uintptr_t)pEccAggrRegs,(ramId >> (uint32_t)5U), intrSrc ),
                          ((uint32_t)1U << (ramId & (uint32_t)0x1FU)) );
        }
        else
        {
            SDL_REG32_WR( SDL_ecc_aggrEnableClrRegAddr( (uintptr_t)pEccAggrRegs,(ramId >> (uint32_t)5U), intrSrc ),
                          ((uint32_t)1U << (ramId & (uint32_t)0x1FU)) );
        }
    }
    else
    {
        retVal = (bool)false;
    }
    return retVal;
}

static bool SDL_ecc_aggrToggleIntrsEnable(const SDL_ecc_aggrRegs *pEccAggrRegs,
    SDL_Ecc_AggrIntrSrc intrSrc, bool bEnable)
{
    bool retVal = (bool)true;
    int32_t      sdlRet = SDL_EBADARGS;

    if( SDL_ecc_aggrIsValidIntrSrc(intrSrc) == (bool)true )
    {
        uint32_t ramId;
        uint32_t numRams;

        sdlRet = SDL_ecc_aggrGetNumRams(pEccAggrRegs, &numRams);
        if (sdlRet == SDL_PASS)
        {
            for( ramId=((uint32_t)(0u)); ramId<numRams; ramId++ )
            {
                if( bEnable == (bool)true)
                {
                    sdlRet = SDL_ecc_aggrEnableIntr( pEccAggrRegs, ramId, intrSrc );
                }
                else
                {
                    sdlRet = SDL_ecc_aggrDisableIntr( pEccAggrRegs, ramId, intrSrc );
                }
            }
        }
    }

    if (sdlRet == SDL_PASS)
    {
        retVal = (bool) true;
    }
    else
    {
        retVal = (bool) false;
    }
    return retVal;
}

/*===========================================================================*/
/*  External SDL-FL Functions                                                */
/*===========================================================================*/

/**
 * Design: PROC_SDL-1184,PROC_SDL-1185
 */
int32_t SDL_ecc_aggrGetRevision(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t *pRev)
{
    int32_t    retVal = SDL_EBADARGS;

    if ( pEccAggrRegs != NULL_PTR )
    {
        if (pRev  != NULL_PTR)
        {
            *pRev = SDL_REG32_RD(&pEccAggrRegs->REV);
            retVal = SDL_PASS;
        }
    }

    /* Return the API success/fail with value in the address provided by caller */
    return (retVal);
}

/**
 * Design: PROC_SDL-1186,PROC_SDL-1187
 */
int32_t SDL_ecc_aggrGetNumRams(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t *pNumRams)
{
    int32_t    retVal = SDL_EBADARGS;

    if ( pEccAggrRegs != NULL_PTR )
    {
        if (pNumRams  != NULL_PTR)
        {
            *pNumRams = (uint32_t)SDL_REG32_FEXT(&pEccAggrRegs->STAT, ECC_AGGR_STAT_NUM_RAMS);
             retVal   = SDL_PASS;
        }
    }
    /* Return the API success/fail with value in the address provided by caller */
    return (retVal);
}

/**
 * Design: PROC_SDL-1188,PROC_SDL-1189
 */
int32_t SDL_ecc_aggrReadEccRamReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t regOffset, uint32_t *pRegVal )
{
    int32_t    retVal = SDL_EBADARGS;

    if( (pEccAggrRegs                                   != NULL_PTR) &&
        (SDL_ecc_aggrIsValidEccRamRegOffset(regOffset)  == (bool) true )              &&
        (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId)  == (bool) true) )
    {
        if (pRegVal  != NULL_PTR)
        {
           SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, regOffset, pRegVal);
           retVal = SDL_PASS;
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1190,PROC_SDL-1191
 */
int32_t SDL_ecc_aggrReadEccRamWrapRevReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t *pRegVal)
{
    return SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, ramId, SDL_ECC_RAM_WRAP_REV, pRegVal);
}

/**
 * Design: PROC_SDL-1192,PROC_SDL-1193
 */
int32_t SDL_ecc_aggrReadEccRamCtrlReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t *pRegVal)
{
    return SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, ramId, SDL_ECC_RAM_CTRL, pRegVal);
}

/**
 * Design: PROC_SDL-1194,PROC_SDL-1195
 */
int32_t SDL_ecc_aggrReadEccRamErrCtrlReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t instSelect, uint32_t *pRegVal)
{
    int32_t    retVal = SDL_EBADARGS;

    if( SDL_ecc_aggrIsValidInstSel(instSelect, SDL_ECC_AGGR_MAX_NUM_RAM_ERR_CTRL)== (bool)true )
    {
        retVal = SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, ramId,
                     SDL_ECC_RAM_ERR_CTRL1+((instSelect)*(uint32_t)4U), pRegVal);
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1196,PROC_SDL-1197
 */
int32_t SDL_ecc_aggrReadEccRamErrStatReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t instSelect, uint32_t *pRegVal)
{
    int32_t    retVal = SDL_EBADARGS;

    if( SDL_ecc_aggrIsValidInstSel(instSelect, SDL_ECC_AGGR_MAX_NUM_RAM_ERR_STAT)==(bool)true )
    {
        retVal = SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, ramId,
                     SDL_ECC_RAM_ERR_STAT1+((instSelect)*(uint32_t)4U), pRegVal);
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1198,PROC_SDL-1199
 */
int32_t SDL_ecc_aggrWriteEccRamReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t regOffset, uint32_t val )
{
    int32_t    retVal = SDL_EBADARGS;

    if( (pEccAggrRegs                                   != NULL_PTR) &&
        (SDL_ecc_aggrIsValidEccRamRegOffset(regOffset)  == (bool) true ) &&
        (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId)  == (bool) true) )
    {
        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, regOffset, val);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1200,PROC_SDL-1201
 */
int32_t SDL_ecc_aggrWriteEccRamCtrlReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t val)
{
    return SDL_ecc_aggrWriteEccRamReg(pEccAggrRegs, ramId, SDL_ECC_RAM_CTRL, val);
}

/**
 * Design: PROC_SDL-1202,PROC_SDL-1203
 */
int32_t SDL_ecc_aggrWriteEccRamErrCtrlReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t instSelect, uint32_t val)
{
    int32_t    retVal = SDL_EBADARGS;

    if( SDL_ecc_aggrIsValidInstSel(instSelect, SDL_ECC_AGGR_MAX_NUM_RAM_ERR_CTRL) ==(bool)true )
    {
        retVal = SDL_ecc_aggrWriteEccRamReg(pEccAggrRegs, ramId,
                     SDL_ECC_RAM_ERR_CTRL1+((instSelect)*(uint32_t)4U), val);
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1204,PROC_SDL-1205
 */
int32_t SDL_ecc_aggrWriteEccRamErrStatReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t instSelect, uint32_t val)
{
    int32_t    retVal = SDL_EBADARGS;

    if( SDL_ecc_aggrIsValidInstSel(instSelect, SDL_ECC_AGGR_MAX_NUM_RAM_ERR_STAT) == (bool)true )
    {
        retVal = SDL_ecc_aggrWriteEccRamReg(pEccAggrRegs, ramId,
                     SDL_ECC_RAM_ERR_STAT1+((instSelect)*(uint32_t)4U), val);
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1206,PROC_SDL-1207
 */
int32_t SDL_ecc_aggrConfigEccRam(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, bool bEnable, bool bEccCheck, bool bEnableRMW)
{
    int32_t    retVal = SDL_EBADARGS;

    if ( (pEccAggrRegs                                  != NULL_PTR)  &&
         (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true) )
    {
        uint32_t val;

        retVal = SDL_ecc_aggrReadEccRamCtrlReg(pEccAggrRegs, ramId, &val);
        if (retVal == SDL_PASS)
        {
            SDL_FINS(val, ECC_RAM_CTRL_ENABLE_RMW, (bEnableRMW ? (uint32_t)1U : (uint32_t)0) );
            SDL_FINS(val, ECC_RAM_CTRL_ECC_CHECK, (bEccCheck ? (uint32_t)1U : (uint32_t)0) );
            SDL_FINS(val, ECC_RAM_CTRL_ECC_ENABLE,(bEnable ? (uint32_t)1U : (uint32_t)0) );
            SDL_FINS(val, ECC_RAM_CTRL_CHECK_SVBUS_TIMEOUT,(bEnable ? (uint32_t)1U : (uint32_t)0) );
            retVal = SDL_ecc_aggrWriteEccRamCtrlReg(pEccAggrRegs, ramId, val);
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1208,PROC_SDL-1209
 */
int32_t SDL_ecc_aggrVerifyConfigEccRam(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, bool bEnable, bool bEccCheck, bool bEnableRMW)
{
    int32_t    retVal = SDL_EBADARGS;

    if ( (pEccAggrRegs != NULL_PTR)       &&
         (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true) )
    {
        uint32_t val, valExp = 0U;
        uint32_t mask = (SDL_ECC_RAM_CTRL_ECC_ENABLE_MASK |
                         SDL_ECC_RAM_CTRL_ECC_CHECK_MASK  |
                         SDL_ECC_RAM_CTRL_ENABLE_RMW_MASK);

        retVal = SDL_ecc_aggrReadEccRamCtrlReg(pEccAggrRegs, ramId, &val);
        if (retVal == SDL_PASS)
        {
            SDL_FINS(valExp, ECC_RAM_CTRL_ENABLE_RMW, (bEnableRMW ? (uint32_t)1U : (uint32_t)0) );
            SDL_FINS(valExp, ECC_RAM_CTRL_ECC_CHECK, (bEccCheck ? (uint32_t)1U : (uint32_t)0) );
            SDL_FINS(valExp, ECC_RAM_CTRL_ECC_ENABLE,(bEnable ? (uint32_t)1U : (uint32_t)0) );
            /* Get the bit fields for the expected values */
            valExp &= mask;
            val    &= mask;
            if (val == valExp)
            {
                retVal = SDL_PASS;
            }
            else
            {
                retVal = SDL_EFAIL;
            }
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1210,PROC_SDL-1211
 */
int32_t SDL_ecc_aggrGetEccRamErrorStatus(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrEccRamErrorStatusInfo *pEccErrorStatus)
{
    int32_t    retVal = SDL_EBADARGS;

    if ( (pEccAggrRegs != NULL_PTR)       &&
         (pEccErrorStatus != NULL_PTR)       &&
         ( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true ) )
    {
        uint32_t  errorStatus1, errorStatus2, errorStatus3;

        /* Read error status register 1 */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_ECC_RAM_ERR_STAT1, &errorStatus1);

        /* Read error status register 2 */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_ECC_RAM_ERR_STAT2, &errorStatus2);

        /* Read error status register 3 */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_ECC_RAM_ERR_STAT3, &errorStatus3);

        /* Populate the status structure with the appropriate bits */
        pEccErrorStatus->controlRegErr    =  ((SDL_FEXT(errorStatus1, ECC_RAM_ERR_STAT1_CTR_REG_ERR) != 0U) ? TRUE : FALSE);
        pEccErrorStatus->successiveSingleBitErr =  ((SDL_FEXT(errorStatus1, ECC_RAM_ERR_STAT1_CLR_ECC_OTHER) != 0U) ? TRUE : FALSE);
        pEccErrorStatus->parityErrorCount = SDL_FEXT(errorStatus1, ECC_RAM_ERR_STAT1_PARITY_ERR);
        pEccErrorStatus->singleBitErrorCount = SDL_FEXT(errorStatus1, ECC_RAM_ERR_STAT1_ECC_SEC);
        pEccErrorStatus->doubleBitErrorCount = SDL_FEXT(errorStatus1, ECC_RAM_ERR_STAT1_ECC_DED);
        pEccErrorStatus->eccRow    =  SDL_FEXT(errorStatus2, ECC_RAM_ERR_STAT2_ECC_ROW);
        pEccErrorStatus->eccBit1   = SDL_FEXT(errorStatus1, ECC_RAM_ERR_STAT1_ECC_BIT1);
        pEccErrorStatus->sVBUSTimeoutErr = ((SDL_FEXT(errorStatus3, ECC_RAM_ERR_STAT3_SVBUS_TIMEOUT_ERR) != 0U) ? TRUE : FALSE);
        pEccErrorStatus->writebackPend   = ((SDL_FEXT(errorStatus3, ECC_RAM_ERR_STAT3_WB_PEND) != 0U) ? TRUE : FALSE);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1212,PROC_SDL-1213
 */
int32_t SDL_ecc_aggrForceEccRamError(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, const SDL_Ecc_AggrErrorInfo *pEccForceError)
{
    int32_t    retVal = SDL_EBADARGS;

    if ( (pEccAggrRegs != NULL_PTR)       &&
         (pEccForceError != NULL_PTR)       &&
         ( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true ) )
    {
        uint32_t regVal;

        /* Set first the bits to insert error */
        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_ECC_RAM_ERR_CTRL2,
            SDL_FMK(ECC_RAM_ERR_CTRL2_ECC_BIT1, pEccForceError->eccBit1)     |
            SDL_FMK(ECC_RAM_ERR_CTRL2_ECC_BIT2, pEccForceError->eccBit2) );

        /* Configure the row to insert error */
        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_ECC_RAM_ERR_CTRL1,
            SDL_FMK(ECC_RAM_ERR_CTRL1_ECC_ROW, pEccForceError->eccRow) );

        /* Read content of Control register */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_ECC_RAM_CTRL, &regVal);

        /* Update the register value with the required error force settings */
        if(pEccForceError->intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT)
        {
            SDL_FINS(regVal, ECC_RAM_CTRL_FORCE_SEC, (uint32_t)1U);
            SDL_FINS(regVal, ECC_RAM_CTRL_FORCE_DED, (uint32_t)0);
        }
        else if(pEccForceError->intrSrc == SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT)
        {
            SDL_FINS(regVal, ECC_RAM_CTRL_FORCE_SEC, (uint32_t)0);
            SDL_FINS(regVal, ECC_RAM_CTRL_FORCE_DED, (uint32_t)1U);
        }
        else
        {
            SDL_FINS(regVal, ECC_RAM_CTRL_FORCE_SEC, (uint32_t)0);
            SDL_FINS(regVal, ECC_RAM_CTRL_FORCE_DED, (uint32_t)0);
        }
        SDL_FINS(regVal, ECC_RAM_CTRL_FORCE_N_ROW,
            ((pEccForceError->bNextRow == TRUE) ? (uint32_t)1U : (uint32_t)0) );
        SDL_FINS(regVal, ECC_RAM_CTRL_ERROR_ONCE,
            ((pEccForceError->bOneShotMode == TRUE) ? (uint32_t)1U : (uint32_t)0) );

        /* Write back the control register to inject the error */
        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_ECC_RAM_CTRL, regVal);
        retVal = SDL_PASS;
    }
    return retVal;
}

static void SDL_ecc_aggrReadSVBUSReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t regOffset, uint32_t *pRegVal )
{
    /* Write to vector register the RAM ID and offset to read */
    SDL_REG32_WR( &pEccAggrRegs->VECTOR,
        SDL_FMK(ECC_AGGR_VECTOR_ECC_VECTOR, ramId)              |
        SDL_FMK(ECC_AGGR_VECTOR_RD_SVBUS_ADDRESS, regOffset)    |
        SDL_FMK(ECC_AGGR_VECTOR_RD_SVBUS, (uint32_t)1U) );

    /* Wait till read operation is complete */
    while( (SDL_ecc_aggrIsSVBUSRegReadDone(pEccAggrRegs)) == (bool)false ) { }

    /* Now read the read value */
    *pRegVal = SDL_REG32_RD((volatile uint32_t *)((uint32_t)pEccAggrRegs+regOffset));
}

static void SDL_ecc_aggrWriteSVBUSReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t regOffset, uint32_t val )
{
    /* Write to vector register the RAM Id register to map */
    SDL_REG32_WR(&pEccAggrRegs->VECTOR, SDL_FMK(ECC_AGGR_VECTOR_ECC_VECTOR, ramId));

    /* Write the value to the register */
    SDL_REG32_WR(((uint32_t)pEccAggrRegs)+ ((uint32_t)regOffset), val);
}

/**
 * Design: PROC_SDL-1214,PROC_SDL-1215
 */
int32_t SDL_ecc_aggrReadEDCInterconnectReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t regOffset, uint32_t *pRegVal )
{
    int32_t    retVal = SDL_EBADARGS;

    /* Do parameter checks */
    if( (pEccAggrRegs                                   != NULL_PTR) &&
        (SDL_ecc_aggrIsValidEDCInterconnectRegOffset(regOffset)  == (bool) true ) &&
        (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId)  == (bool) true) )
    {
        if (pRegVal  != NULL_PTR)
        {
            /* Call routine to read wrapper register */
            SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, regOffset, pRegVal);
            retVal = SDL_PASS;
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1216,PROC_SDL-1217
 */
int32_t SDL_ecc_aggrWriteEDCInterconnectReg(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, uint32_t regOffset, uint32_t val )
{
    int32_t    retVal = SDL_EBADARGS;

    /* Do parameter checks */
    if( (pEccAggrRegs                                   != NULL_PTR) &&
        (SDL_ecc_aggrIsValidEDCInterconnectRegOffset(regOffset)  == (bool) true ) &&
        (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId)  == (bool) true) )
    {
        /* Call routine to write wrapper register */
        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, regOffset, val);
        retVal = SDL_PASS;
    }
    return retVal;
}
/**
 * Design: PROC_SDL-1218,PROC_SDL-1219
 */
int32_t SDL_ecc_aggrConfigEDCInterconnect(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, bool bEccCheck)
{
    int32_t    retVal = SDL_EBADARGS;

    /* Do parameter checks */
    if ( (pEccAggrRegs                                  != NULL_PTR)  &&
         (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true) )
    {
        uint32_t val;

        /* Read the EDC control register */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_CONTROL, &val);

        /* Update fields to configure */
        SDL_FINS(val, EDC_CTL_CONTROL_ECC_CHECK, (bEccCheck ? (uint32_t)1U : (uint32_t)0) );

        /* Write back to EDC control register */
        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_CONTROL, val);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1220,PROC_SDL-1221
 */
int32_t SDL_ecc_aggrVerifyConfigEDCInterconnect(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, bool bEccCheck)
{
    int32_t    retVal = SDL_EBADARGS;

    /* Do parameter checks */
    if ( (pEccAggrRegs != NULL_PTR)       &&
         (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true) )
    {
        uint32_t val, valExp = 0U;
        uint32_t mask = (SDL_ECC_RAM_CTRL_ECC_CHECK_MASK);

        /* Read the EDC control register */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_CONTROL, &val);

        /* set expected configuration fields */
        SDL_FINS(valExp, ECC_RAM_CTRL_ECC_CHECK, (bEccCheck ? (uint32_t)1U : (uint32_t)0) );

        /* Compare the masked values */
        if ((val & mask) == (valExp & mask))
        {
            retVal = SDL_PASS;
        }
        else
        {
            retVal = SDL_EFAIL;
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1222,PROC_SDL-1223
 */
int32_t SDL_ecc_aggrGetEDCInterconnectErrorStatus(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrEDCInterconnectErrorStatusInfo *pEccErrorStatus)
{
    int32_t    retVal = SDL_EBADARGS;

    /* Do parameter checks */
    if ( (pEccAggrRegs != NULL_PTR)       &&
         (pEccErrorStatus != NULL_PTR)       &&
         ( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true ) )
    {
        uint32_t  errorStatus1, errorStatus2;

        /* Read error status register 1 */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_ERR_STATUS1, &errorStatus1);

        /* Read error status register 2 */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_ERR_STATUS2, &errorStatus2);

        /* Populate the status structure with the appropriate bits */
        pEccErrorStatus->singleBitErrorCount = SDL_FEXT(errorStatus1, EDC_CTL_ERR_STATUS1_COR_PEND);
        pEccErrorStatus->doubleBitErrorCount = SDL_FEXT(errorStatus1, EDC_CTL_ERR_STATUS1_UNC_PEND);
        pEccErrorStatus->injectSingleBitErrorCount = SDL_FEXT(errorStatus1, EDC_CTL_ERR_STATUS1_INJ_COR_PEND);
        pEccErrorStatus->injectDoubleBitErrorCount = SDL_FEXT(errorStatus1, EDC_CTL_ERR_STATUS1_INJ_UNC_PEND);
        pEccErrorStatus->eccGroup    =  SDL_FEXT(errorStatus1, EDC_CTL_ERR_STATUS1_ERR_GRP);
        pEccErrorStatus->eccBit1   = SDL_FEXT(errorStatus2, EDC_CTL_ERR_STATUS2_ERR_BIT);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1224,PROC_SDL-1225
 */
int32_t SDL_ecc_aggrForceEDCInterconnectError(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, const SDL_Ecc_AggrEDCInterconnectErrorInfo *pEccForceError)
{
    int32_t    retVal = SDL_PASS;

    /* Do parameter checks */
    if ( (pEccAggrRegs == NULL_PTR) ||
         (pEccForceError == NULL_PTR) ||
         ( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)false) )
    {
        retVal = SDL_EBADARGS;
    }

    if ( retVal == SDL_PASS)
    {
        if ( pEccForceError->eccPattern > SDL_ECC_EGGR_INJECT_PATTERN_MAX )
        {
            retVal = SDL_EBADARGS;
        }
    }

    if ( retVal == SDL_PASS )
    {
        uint32_t regVal;

        /* Clear any pending error injection before writing */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_CONTROL, &regVal);
        SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_SE, (uint32_t)0);
        SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_DE, (uint32_t)0);
        SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_N_BIT, (uint32_t)0);
        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_CONTROL, regVal);

        /* Configure Group, Bits for error injection */
        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_ERR_INJECT1,
            SDL_FMK(EDC_CTL_ERR_INJECT1_ECC_BIT1, pEccForceError->eccBit1)     |
            SDL_FMK(EDC_CTL_ERR_INJECT1_ECC_GRP, pEccForceError->eccGroup) );

        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_ERR_INJECT2,
            SDL_FMK(EDC_CTL_ERR_INJECT2_ECC_BIT2, pEccForceError->eccBit2) );

        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_CONTROL, &regVal);

        /* Configure single bit or double bit */
        if(pEccForceError->intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT)
        {
            SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_SE, (uint32_t)1U);
            SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_DE, (uint32_t)0);
        }
        else if(pEccForceError->intrSrc == SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT)
        {
            SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_SE, (uint32_t)0);
            SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_DE, (uint32_t)1U);
        }
        else
        {
            SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_SE, (uint32_t)0);
            SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_DE, (uint32_t)0);
        }

        /* Configure force n bit */
        SDL_FINS(regVal, EDC_CTL_CONTROL_FORCE_N_BIT,
            ((pEccForceError->bNextBit == TRUE) ? (uint32_t)1U : (uint32_t)0) );

        /* Configure ECC pattern */
        SDL_FINS(regVal, EDC_CTL_CONTROL_ECC_PATTERN, (((uint32_t)pEccForceError->eccPattern) & (uint32_t)0x3U));

        /* Write the wrapper register to inject the error */
        SDL_ecc_aggrWriteSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_CONTROL, regVal);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1226,PROC_SDL-1227
 */
int32_t SDL_ecc_aggrAckIntr(SDL_ecc_aggrRegs *pEccAggrRegs, SDL_Ecc_AggrIntrSrc intrSrc)
{
    int32_t    retVal = SDL_PASS;

    if ( pEccAggrRegs == NULL_PTR)
    {
         retVal = SDL_EBADARGS;
    }
    else
    {
        if( intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT )
        {
            SDL_REG32_WR( &pEccAggrRegs->SEC_EOI_REG, SDL_FMK(ECC_AGGR_SEC_EOI_REG_EOI_WR,(uint32_t)1U) );
        }
        else if( intrSrc == SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT )
        {
            SDL_REG32_WR( &pEccAggrRegs->DED_EOI_REG, SDL_FMK(ECC_AGGR_DED_EOI_REG_EOI_WR,(uint32_t)1U) );
        }
        else
        {
            retVal = SDL_EBADARGS;
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1228,PROC_SDL-1229
 */
int32_t SDL_ecc_aggrIsEccRamIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, bool *pIsPend)
{
    bool        pend  = (bool)false;
    int32_t     retVal = SDL_EBADARGS;

    if ( (pEccAggrRegs != NULL_PTR)       &&
         ( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true ) )
    {
        uint32_t regVal;
        /* No need to check the return value from SDL as arguments are already checked above */
        retVal = SDL_ecc_aggrReadEccRamErrStatReg(pEccAggrRegs, ramId, SDL_ECC_AGGR_SELECT_ERR_STAT1, &regVal);
        if (retVal == SDL_PASS)
        {
            if( ((intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) &&
                 (SDL_FEXT(regVal,ECC_RAM_ERR_STAT1_ECC_SEC) != 0U) ) ||
                ((intrSrc == SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT) &&
                 (SDL_FEXT(regVal,ECC_RAM_ERR_STAT1_ECC_DED) != 0U) ) ||
                ((intrSrc == SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS) &&
                 (SDL_FEXT(regVal,ECC_RAM_ERR_STAT1_ECC_OTHER) != 0U) ) )
            {
                pend = (bool)true;
            }
        }
    }
    if (pIsPend != NULL_PTR)
    {
        *pIsPend = pend;
    }
    else
    {
        retVal = SDL_EBADARGS;
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1230,PROC_SDL-1231
 */
int32_t SDL_ecc_aggrSetEccRamIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc)
{
    bool       ret = (bool) false;
    int32_t    retVal = SDL_EBADARGS;

    /* RAM ID and Interrupt Source is validated internally in below function */
    if (pEccAggrRegs != NULL_PTR)
    {
        ret = SDL_ecc_aggrToggleEccRamIntrPending(pEccAggrRegs, ramId, intrSrc, 1U, (bool)true);
    }

    if (ret == (bool) true)
    {
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1232,PROC_SDL-1233
 */
int32_t SDL_ecc_aggrSetEccRamNIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, uint32_t numEvents)
{
    bool       ret = (bool) false;
    int32_t    retVal = SDL_EBADARGS;
    void       *pChkPtr = (void *) pEccAggrRegs;

    /* RAM ID and Interrupt Source is validated internally in below function */
    if ( (pChkPtr != NULL_PTR) && (numEvents != (uint32_t)0U) &&
         (numEvents <= (uint32_t)3U) )
    {
        ret = SDL_ecc_aggrToggleEccRamIntrPending(pEccAggrRegs, ramId,
                  intrSrc, numEvents, (bool)true);
    }

    if (ret == (bool) true)
    {
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1234,PROC_SDL-1235
 */
int32_t SDL_ecc_aggrClrEccRamIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc)
{
    bool       ret = (bool) false;
    int32_t    retVal = SDL_EBADARGS;

    /* RAM ID and Interrupt Source is validated internally in below function */
    if  (pEccAggrRegs != NULL_PTR)
    {
        ret = SDL_ecc_aggrToggleEccRamIntrPending(pEccAggrRegs, ramId,
                  intrSrc, 1U, (bool)false);
    }

    if (ret == (bool) true)
    {
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1236,PROC_SDL-1237
 */
int32_t SDL_ecc_aggrClrEccRamNIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, uint32_t numEvents)
{
    bool       ret = (bool) false;
    int32_t    retVal = SDL_EBADARGS;

    /* RAM ID and Interrupt Source is validated internally in below function */
    if ( (pEccAggrRegs != NULL_PTR) &&
         (numEvents != (uint32_t)0U) &&
         (numEvents <= (uint32_t)3U) )

    {
        ret = SDL_ecc_aggrToggleEccRamIntrPending(pEccAggrRegs, ramId,
                  intrSrc, numEvents, (bool)false);
    }

    if (ret == (bool) true)
    {
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1238,PROC_SDL-1239
 */
int32_t SDL_ecc_aggrIsEDCInterconnectIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, bool *pIsPend)
{
    bool        pend  = (bool)false;
    int32_t     retVal = SDL_EBADARGS;

    /* Do parameter checks */
    if ( (pEccAggrRegs != NULL_PTR) &&
         ( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true ) &&
         (SDL_ecc_aggrIsValidIntrSrc(intrSrc)  == (bool) true ) &&
         (pIsPend != NULL_PTR) )
    {
        uint32_t regVal;

        /* Read Error Status register 1 */
        SDL_ecc_aggrReadSVBUSReg(pEccAggrRegs, ramId, SDL_EDC_CTL_ERR_STATUS1, &regVal);

        /* Check event pending */
        if( ((intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) &&
             (SDL_FEXT(regVal, EDC_CTL_ERR_STATUS1_COR_PEND) != 0U) ) ||
            ((intrSrc == SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT) &&
             (SDL_FEXT(regVal, EDC_CTL_ERR_STATUS1_UNC_PEND) != 0U) ) ||
            ((intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) &&
             (SDL_FEXT(regVal, EDC_CTL_ERR_STATUS1_INJ_COR_PEND) != 0U) ) ||
            ((intrSrc == SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT) &&
             (SDL_FEXT(regVal, EDC_CTL_ERR_STATUS1_INJ_UNC_PEND) != 0U) ) )
        {
            pend = (bool)true;
        }

        /* Return is pending flag */
        *pIsPend = pend;
        retVal = SDL_PASS;
    }

    return retVal;
}

/**
 * Design: PROC_SDL-1240,PROC_SDL-1241
 */
int32_t SDL_ecc_aggrSetEDCInterconnectNIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
                                                   uint32_t ramId,
                                                   SDL_Ecc_AggrIntrSrc intrSrc,
                                                   SDL_Ecc_AggrEDCErrorSubType subType,
                                                   uint32_t numEvents)
{
    bool       ret = (bool) false;
    int32_t    retVal = SDL_EBADARGS;
    void       *pChkPtr = (void *) pEccAggrRegs;

    /* RAM ID and Interrupt Source is validated internally in below function */
    if ( (pChkPtr != NULL_PTR) &&
         (numEvents != (uint32_t)0U) &&
         (numEvents <= (uint32_t)3U) )
    {
        ret = SDL_ecc_aggrToggleEDCInterconnectIntrPending(pEccAggrRegs, ramId,
                  intrSrc, subType, numEvents, (bool)true);
    }

    if (ret == (bool) true)
    {
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1242,PROC_SDL-1243
 */
int32_t SDL_ecc_aggrClrEDCInterconnectNIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs,
                                                   uint32_t ramId,
                                                   SDL_Ecc_AggrIntrSrc intrSrc,
                                                   SDL_Ecc_AggrEDCErrorSubType subType,
                                                   uint32_t numEvents)
{
    bool       ret = (bool) false;
    int32_t    retVal = SDL_EBADARGS;

    /* RAM ID and Interrupt Source is validated internally in below function */
    if ( (pEccAggrRegs != NULL_PTR) &&
         (numEvents <= (uint32_t)3U) && (numEvents != (uint32_t)0U) )
    {
        ret = SDL_ecc_aggrToggleEDCInterconnectIntrPending(pEccAggrRegs,
                  ramId, intrSrc, subType, numEvents, (bool)false);
    }

    if (ret == (bool) true)
    {
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1244,PROC_SDL-1245
 */
int32_t SDL_ecc_aggrIsIntrPending(const SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, bool *pIsPend)
{
    bool       pend = (bool) false;
    int32_t    retVal = SDL_EBADARGS;

    if ( (pEccAggrRegs != NULL_PTR) )
    {
        if( (SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true) &&
            (SDL_ecc_aggrIsValidIntrSrc(intrSrc) == (bool)true) )
        {
            uint32_t    regVal;
            uintptr_t   regAddr;

            retVal  = SDL_PASS;
            regAddr = (uintptr_t)SDL_ecc_aggrStatusRegAddr( (uintptr_t)pEccAggrRegs,
                          (ramId >> (uint32_t)5U), intrSrc );
            regVal = SDL_REG32_RD( regAddr );
            regVal >>= (ramId & (uint32_t)0x1FU);                    /* Shift bit corresponding to the ramId into bit 0 */
            if( (regVal & (uint32_t)1U) != 0U )
            {
                pend = (bool)true;
            }
        }
    }

    if (pIsPend != NULL_PTR)
    {
        *pIsPend = pend;
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return retVal;
}

/**
 * Design: PROC_SDL-1246,PROC_SDL-1247
 */
int32_t SDL_ecc_aggrIsAnyIntrPending(const SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, bool *pIsPend)
{
    bool       pend = (bool) false;
    bool       sBitPend, dBitPend;
    int32_t    retVal;

    /* Argument verification is done at below APIs, hence do not need to check here */
    retVal = SDL_ecc_aggrIsIntrPending(pEccAggrRegs, ramId,
                 SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, &sBitPend);
    if (retVal == SDL_PASS)
    {
        retVal = SDL_ecc_aggrIsIntrPending(pEccAggrRegs, ramId,
                     SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, &dBitPend);
    }
    if (retVal == SDL_PASS)
    {
        if ( sBitPend || dBitPend )
        {
           pend = (bool)true;
        }
    }

    if (pIsPend != NULL_PTR)
    {
        *pIsPend = pend;
    }
    else
    {
        retVal = SDL_EBADARGS;
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1248,PROC_SDL-1249
 */
int32_t SDL_ecc_aggrEnableIntr(const SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc)
{
    int32_t retVal = SDL_EBADARGS;
    bool    operation = (bool) false;

    if ( (pEccAggrRegs != NULL_PTR) )
    {
        /* other arguments are verified in below function */
        operation = SDL_ecc_aggrToggleIntrEnable(pEccAggrRegs, ramId,
                        intrSrc, (bool)true);
    }

    if (operation == (bool) true)
    {
        retVal = SDL_PASS;
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);
}

/**
 * Design: PROC_SDL-1250,PROC_SDL-1251
 */
int32_t SDL_ecc_aggrDisableIntr(const SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc)
{
    int32_t retVal = SDL_EBADARGS;
    bool    operation = (bool) false;

    if ( (pEccAggrRegs != NULL_PTR) )
    {
        operation = SDL_ecc_aggrToggleIntrEnable(pEccAggrRegs, ramId,
                        intrSrc, (bool)false);
    }

    if (operation == (bool) true)
    {
        retVal = SDL_PASS;
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);
}

/**
 * Design: PROC_SDL-1252,PROC_SDL-1253
 */
int32_t SDL_ecc_aggrEnableAllIntr(const SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId)
{
    int32_t retVal = SDL_EBADARGS;

    if ( (pEccAggrRegs != NULL_PTR) )
    {
        if( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true )
        {
            /* Other Argument verification is done in below APIs */
            retVal = SDL_ecc_aggrEnableIntr(pEccAggrRegs, ramId,
                         SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT);
            if (retVal == SDL_PASS)
            {
                retVal = SDL_ecc_aggrEnableIntr(pEccAggrRegs, ramId,
                             SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT);
            }
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1254,PROC_SDL-1255
 */
int32_t SDL_ecc_aggrDisableAllIntr(const SDL_ecc_aggrRegs *pEccAggrRegs,
    uint32_t ramId)
{
    int32_t retVal = SDL_EBADARGS;

    if ( (pEccAggrRegs != NULL_PTR) )
    {
        if( SDL_ecc_aggrIsValidRamId(pEccAggrRegs, ramId) == (bool)true )
        {
            /* Other Argument verification is done in below APIs */
            retVal = SDL_ecc_aggrDisableIntr(pEccAggrRegs, ramId,
                         SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT);
            if (retVal == SDL_PASS)
            {
                retVal = SDL_ecc_aggrDisableIntr(pEccAggrRegs, ramId,
                             SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT);
            }
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1256,PROC_SDL-1257
 */
int32_t SDL_ecc_aggrEnableIntrs(const SDL_ecc_aggrRegs *pEccAggrRegs,
    SDL_Ecc_AggrIntrSrc intrSrc)
{
    int32_t retVal = SDL_EBADARGS;
    bool    operation = (bool) false;

    /* Other argument checks are done internal to below API */
    if ( (pEccAggrRegs != NULL_PTR) )
    {
        operation = SDL_ecc_aggrToggleIntrsEnable(pEccAggrRegs, intrSrc,
                       (bool)true);
    }

    if (operation == (bool) true)
    {
        retVal = SDL_PASS;
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);
}

/**
 * Design: PROC_SDL-1258,PROC_SDL-1259
 */
int32_t SDL_ecc_aggrDisableIntrs(const SDL_ecc_aggrRegs *pEccAggrRegs,
    SDL_Ecc_AggrIntrSrc intrSrc)
{
    int32_t retVal = SDL_EBADARGS;
    bool    operation = (bool) false;

    /* Other argument checks are done internal to below API */
    if ( (pEccAggrRegs != NULL_PTR) )
    {
        operation = SDL_ecc_aggrToggleIntrsEnable(pEccAggrRegs, intrSrc, (bool)false);
    }

    if (operation == (bool) true)
    {
        retVal = SDL_PASS;
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);
}

/**
 * Design: PROC_SDL-1260,PROC_SDL-1261
 */
int32_t SDL_ecc_aggrEnableAllIntrs(const SDL_ecc_aggrRegs *pEccAggrRegs)
{
    int32_t retVal = SDL_EBADARGS;

    /* Other argument checks are done internal to below API */
    if ( (pEccAggrRegs != NULL_PTR) )
    {
        retVal = SDL_ecc_aggrEnableIntrs(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT);
        if (retVal == SDL_PASS)
        {
            retVal = SDL_ecc_aggrEnableIntrs(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT);
        }
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1262,PROC_SDL-1263
 */
int32_t SDL_ecc_aggrDisableAllIntrs(const SDL_ecc_aggrRegs *pEccAggrRegs)
{
    int32_t retVal = SDL_EBADARGS;

    /* Other argument checks are done internal to below API */
    if ( (pEccAggrRegs != NULL_PTR) )
    {
        retVal = SDL_ecc_aggrDisableIntrs(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT);
        if (retVal == SDL_PASS)
        {
            retVal = SDL_ecc_aggrDisableIntrs(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT);
        }
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1264,PROC_SDL-1265
 */
int32_t SDL_ecc_aggrReadStaticRegs(SDL_ecc_aggrRegs *pEccAggrRegs,
    SDL_ECC_staticRegs *pEccAggrStaticRegs)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regOffset, *pRegPtr;
    uint32_t i;

    /* Read below static registers */
    retVal = SDL_ecc_aggrGetRevision((const SDL_ecc_aggrRegs *) pEccAggrRegs,
                 &pEccAggrStaticRegs->REV);

    /* ECC Control register */
    if (retVal == SDL_PASS)
    {
        regOffset = SDL_ECC_RAM_CTRL;
        retVal = SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, 0, regOffset,
                     &pEccAggrStaticRegs->ECC_CTRL);
    }

    /* ECC Err Control1 register */
    if (retVal == SDL_PASS)
    {
        regOffset = SDL_ECC_RAM_ERR_CTRL1;
        retVal = SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, 0, regOffset,
                     &pEccAggrStaticRegs->ECC_ERR_CTRL1);
    }

    /* ECC Err Control2 register */
    if (retVal == SDL_PASS)
    {
        regOffset = SDL_ECC_RAM_ERR_CTRL2;
        retVal = SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, 0, regOffset,
                     &pEccAggrStaticRegs->ECC_ERR_CTRL2);
    }

    /* Other argument checks are done internal to below API */
    if (retVal == SDL_PASS)
    {
        for ( i = ((uint32_t) (0u)); i < (SDL_ECC_AGGR_NUM_ENABLE_REGISTERS); i++)
        {
            /* ECC_SEC_ENABLE_SET_REG registers */
            pRegPtr = (uint32_t *) SDL_ecc_aggrSecEnableSetRegAddr ((uintptr_t) pEccAggrRegs, i);
            pEccAggrStaticRegs->ECC_SEC_ENABLE_SET_REG[i] = SDL_REG_RD(pRegPtr);

            /* ECC_SEC_ENABLE_CLR_REG registers */
            pRegPtr = (uint32_t *) SDL_ecc_aggrSecEnableClrRegAddr ((uintptr_t) pEccAggrRegs, i);
            pEccAggrStaticRegs->ECC_SEC_ENABLE_CLR_REG[i] = SDL_REG_RD(pRegPtr);

            /* ECC_DED_ENABLE_SET_REG registers */
            pRegPtr = (uint32_t *) SDL_ecc_aggrDedEnableSetRegAddr ((uintptr_t) pEccAggrRegs, i);
            pEccAggrStaticRegs->ECC_DED_ENABLE_SET_REG[i] = SDL_REG_RD(pRegPtr);

            /* ECC_DED_ENABLE_CLR_REG registers */
            pRegPtr = (uint32_t *) SDL_ecc_aggrDedEnableClrRegAddr ((uintptr_t) pEccAggrRegs, i);
            pEccAggrStaticRegs->ECC_DED_ENABLE_CLR_REG[i] = SDL_REG_RD(pRegPtr);
        }
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1266,PROC_SDL-1267
 */

int32_t SDL_ecc_aggrIntrEnableCtrl(SDL_ecc_aggrRegs *pEccAggrRegs,
    const SDL_ecc_aggrEnableCtrl *pEnableCtrl)
{
    int32_t retVal = SDL_EBADARGS;

    if ((pEccAggrRegs   != NULL_PTR) &&
        (pEnableCtrl    != NULL_PTR))
    {
        if (((uint32_t)pEnableCtrl->validCfg & SDL_ECC_AGGR_VALID_TIMEOUT_ERR) == \
             SDL_ECC_AGGR_VALID_TIMEOUT_ERR)
        {
            if((pEnableCtrl->intrEnableTimeoutErr) == TRUE)
            {
                SDL_REG32_FINS(&pEccAggrRegs->AGGR_ENABLE_SET,      \
                               ECC_AGGR_AGGR_ENABLE_SET_TIMEOUT,    \
                               1u);
            }
            else
            {
                SDL_REG32_FINS(&pEccAggrRegs->AGGR_ENABLE_CLR,      \
                               ECC_AGGR_AGGR_ENABLE_CLR_TIMEOUT,    \
                               1u);
            }
        }
        if (((uint32_t)pEnableCtrl->validCfg & SDL_ECC_AGGR_VALID_PARITY_ERR) == \
             SDL_ECC_AGGR_VALID_PARITY_ERR)
        {
            if((pEnableCtrl->intrEnableParityErr) == TRUE)
            {
                SDL_REG32_FINS(&pEccAggrRegs->AGGR_ENABLE_SET,      \
                               ECC_AGGR_AGGR_ENABLE_SET_PARITY,     \
                               1u);
            }
            else
            {
                SDL_REG32_FINS(&pEccAggrRegs->AGGR_ENABLE_CLR,      \
                               ECC_AGGR_AGGR_ENABLE_CLR_PARITY,     \
                               1u);
            }
        }

        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1268,PROC_SDL-1269
 */
int32_t SDL_ecc_aggrIntrStatusCtrl(SDL_ecc_aggrRegs *pEccAggrRegs,
    const SDL_ecc_aggrStatusCtrl *pStatusCtrl)
{
    int32_t retVal = SDL_EBADARGS;
    uint32_t tCnt, pCnt;

    if ((pEccAggrRegs   != NULL_PTR) &&
        (pStatusCtrl    != NULL_PTR) &&
        (pStatusCtrl->timeOutCnt < SDL_TWO_POWER_TWO_VALUE) &&
        (pStatusCtrl->parityCnt  < SDL_TWO_POWER_TWO_VALUE))
    {
        if (((uint32_t)pStatusCtrl->validCfg & SDL_ECC_AGGR_VALID_TIMEOUT_ERR) == \
             SDL_ECC_AGGR_VALID_TIMEOUT_ERR)
        {
            tCnt    = (uint32_t)pStatusCtrl->timeOutCnt;
            if((pStatusCtrl->intrStatusSetTimeoutErr) == TRUE)
            {
                SDL_REG32_FINS(&pEccAggrRegs->AGGR_STATUS_SET,      \
                               ECC_AGGR_AGGR_STATUS_SET_TIMEOUT,     \
                               tCnt);
            }
            else
            {
                SDL_REG32_FINS(&pEccAggrRegs->AGGR_STATUS_CLR,      \
                               ECC_AGGR_AGGR_STATUS_CLR_TIMEOUT,     \
                               tCnt);
            }
        }
        if (((uint32_t)pStatusCtrl->validCfg & SDL_ECC_AGGR_VALID_PARITY_ERR) == \
             SDL_ECC_AGGR_VALID_PARITY_ERR)
        {
            pCnt    = (uint32_t)pStatusCtrl->parityCnt;
            if((pStatusCtrl->intrStatusSetParityErr) == TRUE)
            {
                SDL_REG32_FINS(&pEccAggrRegs->AGGR_STATUS_SET,      \
                               ECC_AGGR_AGGR_STATUS_SET_PARITY,     \
                               pCnt);
            }
            else
            {
                SDL_REG32_FINS(&pEccAggrRegs->AGGR_STATUS_CLR,      \
                               ECC_AGGR_AGGR_STATUS_CLR_PARITY,     \
                               pCnt);
            }
            retVal = SDL_PASS;
        }
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1270,PROC_SDL-1271
 */
int32_t SDL_ecc_aggrIntrGetStatus(const SDL_ecc_aggrRegs       *pEccAggrRegs,
                                      SDL_ecc_aggrStatusCtrl *pStatusCtrl)
{
    int32_t retVal = SDL_EBADARGS;


    if ((pEccAggrRegs   != NULL_PTR) &&
        (pStatusCtrl    != NULL_PTR))
    {
        if (((uint32_t)pStatusCtrl->validCfg & SDL_ECC_AGGR_VALID_TIMEOUT_ERR) == \
             SDL_ECC_AGGR_VALID_TIMEOUT_ERR)
        {
            if((pStatusCtrl->intrStatusSetTimeoutErr)== TRUE)
            {
                pStatusCtrl->timeOutCnt = \
                SDL_REG32_FEXT(&pEccAggrRegs->AGGR_STATUS_SET,      \
                               ECC_AGGR_AGGR_STATUS_SET_TIMEOUT);
            }
            else
            {
                pStatusCtrl->timeOutCnt = \
                SDL_REG32_FEXT(&pEccAggrRegs->AGGR_STATUS_CLR,      \
                               ECC_AGGR_AGGR_STATUS_CLR_TIMEOUT);
            }
        }
        if (((uint32_t)pStatusCtrl->validCfg & SDL_ECC_AGGR_VALID_PARITY_ERR) == \
             SDL_ECC_AGGR_VALID_PARITY_ERR)
        {
            if((pStatusCtrl->intrStatusSetParityErr) == TRUE)
            {
                pStatusCtrl->parityCnt = \
                SDL_REG32_FEXT(&pEccAggrRegs->AGGR_STATUS_SET,      \
                               ECC_AGGR_AGGR_STATUS_SET_PARITY);
            }
            else
            {
                pStatusCtrl->parityCnt = \
                SDL_REG32_FEXT(&pEccAggrRegs->AGGR_STATUS_CLR,      \
                               ECC_AGGR_AGGR_STATUS_CLR_PARITY);
            }
            retVal = SDL_PASS;
        }
    }
    return (retVal);
}
/* Nothing past this point */
