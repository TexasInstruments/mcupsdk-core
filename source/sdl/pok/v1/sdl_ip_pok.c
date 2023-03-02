/*
 * @file  sdl_ip_pok.c
 *
 * @brief
 *  C implementation file for the POK module SDL-FL.
 *
 *  Translates POK ID to POK Address. This is a SOC specific source file
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2023, Texas Instruments, Inc.
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
#include <sdl/pok/v1/sdl_ip_pok.h>
#include <sdl/pok/v1/sdl_pok.h>
#include <sdl/pok/v1/sdl_pok_def.h>

/*=============================================================================
 *  Internal definitions and functions
 *===========================================================================*/

/*=============================================================================
 *  internal macros
 *===========================================================================*/
/* None */

/*=============================================================================
 *  static global variables
 *===========================================================================*/
/* None */

/*=============================================================================
 *  Non Interface function - internal use only
 *===========================================================================*/


static int32_t  SDL_pokSetOperation(SDL_mcuCtrlRegsBase_t           *pBaseAddress,
                             const SDL_POK_config               *pPokCfg,
                             SDL_POK_Inst                        instance)
{
    int32_t                          retVal = SDL_EBADARGS;
    SDL_pokShiftsAndMasks_t          shiftsNMasks;


    retVal =  SDL_pok_GetShiftsAndMasks(pBaseAddress, instance, &shiftsNMasks);

    if (retVal == SDL_PASS)
    {
        if (pPokCfg->hystCtrl != SDL_PWRSS_HYSTERESIS_NO_ACTION)
        {
            SDL_REG32_FINS_RAW(shiftsNMasks.pokAddr, shiftsNMasks.hystMask, shiftsNMasks.hystShift, pPokCfg->hystCtrl);
        }

        if (pPokCfg->hystCtrlOV != SDL_PWRSS_HYSTERESIS_NO_ACTION)
        {
            SDL_REG32_FINS_RAW(shiftsNMasks.pokOVAddr, shiftsNMasks.hystOVMask, shiftsNMasks.hystOVShift, pPokCfg->hystCtrlOV);
        }

        if (pPokCfg->voltDetMode != SDL_PWRSS_VOLTAGE_DET_NO_ACTION)
        {
            if ((pPokCfg->voltDetMode == SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE) || (pPokCfg->voltDetMode == SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE))
            {
                SDL_REG32_FINS_RAW(shiftsNMasks.pokAddr, shiftsNMasks.vdDetMask, shiftsNMasks.vdDetShift, SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE);
                if (shiftsNMasks.ovSelMask != 0x0u)
                {
                    SDL_REG32_FINS_RAW(shiftsNMasks.pokDetAddr, shiftsNMasks.ovSelMask, shiftsNMasks.ovSelShift, SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE);
                }
                if (pPokCfg->voltDetMode == SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE)
                {
                    SDL_REG32_FINS_RAW(shiftsNMasks.pokEnPPAddr, shiftsNMasks.pokEnPPMask, shiftsNMasks.pokEnPPShift, SDL_PWRSS_PP_MODE_ENABLE);
                }
            }
            if ((pPokCfg->voltDetMode == SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE) || (pPokCfg->voltDetMode == SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE))
            {
                SDL_REG32_FINS_RAW(shiftsNMasks.pokOVAddr, shiftsNMasks.vdDetOVMask, shiftsNMasks.vdDetOVShift, SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE);
                if (pPokCfg->voltDetMode == SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE)
                {
                    if (shiftsNMasks.ovSelMask != 0x0u)
                    {
                        SDL_REG32_FINS_RAW(shiftsNMasks.pokDetAddr, shiftsNMasks.ovSelMask, shiftsNMasks.ovSelShift, SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE);
                    }
                    SDL_REG32_FINS_RAW(shiftsNMasks.pokEnPPAddr, shiftsNMasks.pokEnPPMask, shiftsNMasks.pokEnPPShift, SDL_PWRSS_PP_MODE_DISABLE);
                }
            }
        }

        if ((pPokCfg->trim <= SDL_PWRSS_MAX_TRIM_VALUE) && (shiftsNMasks.trimMask != 0x0u))
        {
            SDL_REG32_FINS_RAW(shiftsNMasks.pokAddr, shiftsNMasks.trimMask, shiftsNMasks.trimShift, pPokCfg->trim);
        }

        if ((pPokCfg->trimOV <= SDL_PWRSS_MAX_TRIM_VALUE) && (shiftsNMasks.trimOVMask != 0x0u))
        {
            SDL_REG32_FINS_RAW(shiftsNMasks.pokOVAddr, shiftsNMasks.trimOVMask, shiftsNMasks.trimOVShift, pPokCfg->trimOV);
        }

        if (pPokCfg->detectionCtrl == SDL_POK_DETECTION_ENABLE)
        {
            SDL_REG32_FINS_RAW(shiftsNMasks.pokDetAddr, shiftsNMasks.detEnMask, shiftsNMasks.detEnShift, SDL_POK_DETECTION_ENABLE);
        }

        if (pPokCfg->detectionCtrl == SDL_POK_DETECTION_DISABLE)
        {
            SDL_REG32_FINS_RAW(shiftsNMasks.pokDetAddr, shiftsNMasks.detEnMask, shiftsNMasks.detEnShift, SDL_POK_DETECTION_DISABLE);
        }

        if (pPokCfg->deglitch != SDL_PWRSS_DEGLITCH_NO_ACTION)
        {
            SDL_REG32_FINS_RAW(shiftsNMasks.deglitchSelAddr, shiftsNMasks.deglitchSelMask, shiftsNMasks.deglitchSelShift, pPokCfg->deglitch);
        }

        if (pPokCfg->pokEnSelSrcCtrl == SDL_POK_ENSEL_HWTIEOFFS)
        {
            SDL_REG32_FINS_RAW(shiftsNMasks.pokEnSelAddr, shiftsNMasks.pokEnSelMask, shiftsNMasks.pokEnSelShift, SDL_POK_ENSEL_HWTIEOFFS);
        }

        if (pPokCfg->pokEnSelSrcCtrl == SDL_POK_ENSEL_PRG_CTRL)
        {
            SDL_REG32_FINS_RAW(shiftsNMasks.pokEnSelAddr, shiftsNMasks.pokEnSelMask, shiftsNMasks.pokEnSelShift, SDL_POK_ENSEL_PRG_CTRL);
        }
    }

    return (retVal);

}

/**
 * Design: PROC_SDL-1357,PROC_SDL-1358
 */

int32_t SDL_porSetControl (SDL_mcuCtrlRegsBase_t           *pBaseAddress,
                           const SDL_pokPorCfg_t            *pPorCfg)
{
    int32_t     retVal = SDL_PASS;

    SDL_mcu_ctrl_mmr_cfg0Regs      *pCtrlMMRCfgRegs = \
                                    (SDL_mcu_ctrl_mmr_cfg0Regs *)pBaseAddress;

    if ( (pBaseAddress == NULL_PTR) ||
         (pPorCfg      == NULL_PTR) ||
         (pPorCfg->trim_select > SDL_POR_TRIM_SELECTION_GET_VALUE) )
    {
        retVal = SDL_EBADARGS;
    }


    if (retVal == SDL_PASS)
    {
        /* Mask HHV output when applying new TRIM values */
        if (pPorCfg->maskHHVOutputEnable == FALSE)
        {
            SDL_REG32_FINS(&pCtrlMMRCfgRegs->POR_CTRL, MCU_CTRL_MMR_CFG0_POR_CTRL_MASK_HHV, 0U);
        }
        else
        {
            SDL_REG32_FINS(&pCtrlMMRCfgRegs->POR_CTRL, MCU_CTRL_MMR_CFG0_POR_CTRL_MASK_HHV, 1U);
        }

        if (pPorCfg->trim_select == SDL_POR_TRIM_SELECTION_FROM_HHV_DEFAULT)
        {
            SDL_REG32_FINS(&pCtrlMMRCfgRegs->POR_CTRL, MCU_CTRL_MMR_CFG0_POR_CTRL_TRIM_SEL, 0U);
        }
        else if (pPorCfg->trim_select == SDL_POR_TRIM_SELECTION_FROM_CTRL_REGS)
        {
            SDL_REG32_FINS(&pCtrlMMRCfgRegs->POR_CTRL, MCU_CTRL_MMR_CFG0_POR_CTRL_TRIM_SEL, 1U);
        }
        else
        {
            /* No Action */
        }
    }

    return (retVal);
}


static int32_t  SDL_pokGetOperation(SDL_mcuCtrlRegsBase_t           *pBaseAddress,
                             const SDL_POK_config               *pPokCfg,
                             SDL_pokVal_t                     *pPokVal,
                             SDL_POK_Inst                        instance)

{
    int32_t                          retVal;
    SDL_pokShiftsAndMasks_t          shiftsNMasks;

    retVal = SDL_pok_GetShiftsAndMasks(pBaseAddress, instance, &shiftsNMasks);

    if (retVal == SDL_PASS)
    {
        if ((pPokCfg->hystCtrl == SDL_PWRSS_GET_HYSTERESIS_VALUE) && (shiftsNMasks.pokAddr != NULL))
        {
            pPokVal->hystCtrl = (SDL_pwrss_hysteresis)SDL_REG32_FEXT_RAW(shiftsNMasks.pokAddr, shiftsNMasks.hystMask, shiftsNMasks.hystShift);
        }

        if ((pPokCfg->hystCtrlOV == SDL_PWRSS_GET_HYSTERESIS_VALUE) && (shiftsNMasks.pokOVAddr != NULL))
        {
            pPokVal->hystCtrlOV = (SDL_pwrss_hysteresis)SDL_REG32_FEXT_RAW(shiftsNMasks.pokOVAddr, shiftsNMasks.hystOVMask, shiftsNMasks.hystOVShift);
        }

        if (pPokCfg->voltDetMode == SDL_PWRSS_GET_VOLTAGE_DET_MODE)
        {
            if (shiftsNMasks.pokEnPPAddr != 0x0)
            {
                /* If PP is supported, then we can check the PRG for the detection type */
                if (SDL_REG32_FEXT_RAW(shiftsNMasks.pokEnPPAddr, shiftsNMasks.pokEnPPMask, shiftsNMasks.pokEnPPShift) == SDL_PWRSS_PP_MODE_ENABLE)
                {
                    pPokVal->voltDetMode = SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE;
                }
                else
                {
                    pPokVal->voltDetMode = (SDL_pwrss_vd_mode)SDL_REG32_FEXT_RAW(shiftsNMasks.pokDetAddr, shiftsNMasks.ovSelMask, shiftsNMasks.ovSelShift);
                }
            }
            else
            {
                if (shiftsNMasks.pokAddr != NULL)
                {
                    pPokVal->voltDetMode = (SDL_pwrss_vd_mode)SDL_REG32_FEXT_RAW(shiftsNMasks.pokAddr, shiftsNMasks.vdDetMask, shiftsNMasks.vdDetShift);
                }
                else
                {
                    pPokVal->voltDetMode = (SDL_pwrss_vd_mode)SDL_REG32_FEXT_RAW(shiftsNMasks.pokOVAddr, shiftsNMasks.vdDetOVMask, shiftsNMasks.vdDetOVShift);
                }
            }
        }


        if ((pPokCfg->trim == SDL_PWRSS_GET_TRIM_VALUE) && (shiftsNMasks.pokAddr != NULL) && (shiftsNMasks.trimMask != 0x0u))
        {
            pPokVal->trim = (SDL_pwrss_trim)SDL_REG32_FEXT_RAW(shiftsNMasks.pokAddr, shiftsNMasks.trimMask, shiftsNMasks.trimShift);
        }

        if ((pPokCfg->trimOV == SDL_PWRSS_GET_TRIM_VALUE) && (shiftsNMasks.pokOVAddr != NULL))
        {
            pPokVal->trimOV = (SDL_pwrss_trim)SDL_REG32_FEXT_RAW(shiftsNMasks.pokOVAddr, shiftsNMasks.trimOVMask, shiftsNMasks.trimOVShift);
        }

        if (pPokCfg->detectionCtrl == SDL_POK_GET_DETECTION_VALUE)
        {
            pPokVal->detectionStatus = (SDL_POK_detection_status)SDL_REG32_FEXT_RAW(shiftsNMasks.pokDetAddr, shiftsNMasks.detEnMask, shiftsNMasks.detEnShift);
        }

        if (pPokCfg->pokEnSelSrcCtrl == SDL_POK_GET_ENSEL_VALUE)
        {
            pPokVal->pokEnSelSrcCtrl = (SDL_POK_enSelSrc)SDL_REG32_FEXT_RAW(shiftsNMasks.pokEnSelAddr, shiftsNMasks.pokEnSelMask, shiftsNMasks.pokEnSelShift);
        }

        if (pPokCfg->deglitch == SDL_PWRSS_DEGLITCH_GET_VALUE)
        {
            pPokVal->deglitch = (SDL_pwrss_deglitch)SDL_REG32_FEXT_RAW(shiftsNMasks.deglitchSelAddr, shiftsNMasks.deglitchSelMask, shiftsNMasks.deglitchSelShift);
        }

    }

    return (retVal);

}

/**
 * Design: PROC_SDL-1355,PROC_SDL-1356
 */

int32_t SDL_pokGetControl (SDL_mcuCtrlRegsBase_t           *pBaseAddress,
                             const SDL_POK_config               *pPokCfg,
                             SDL_pokVal_t                     *pPokVal,
                             SDL_POK_Inst                      instance)
{
    int32_t     retVal = SDL_PASS;

    if ( (pBaseAddress          == NULL_PTR) ||
         (pPokCfg               == NULL_PTR)  ||
         (pPokVal               == NULL_PTR) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        if ((pPokCfg->hystCtrl     == SDL_PWRSS_GET_HYSTERESIS_VALUE) ||
            (pPokCfg->hystCtrlOV   == SDL_PWRSS_GET_HYSTERESIS_VALUE) ||
            (pPokCfg->voltDetMode  == SDL_PWRSS_GET_VOLTAGE_DET_MODE) ||
            (pPokCfg->trim         == SDL_PWRSS_GET_TRIM_VALUE)       ||
            (pPokCfg->trimOV       == SDL_PWRSS_GET_TRIM_VALUE)       ||
            (pPokCfg->detectionCtrl == SDL_POK_GET_DETECTION_VALUE)   ||
            (pPokCfg->pokEnSelSrcCtrl  == SDL_POK_GET_ENSEL_VALUE)    ||
            (pPokCfg->deglitch     == SDL_PWRSS_DEGLITCH_GET_VALUE))
        {
            retVal = SDL_pokGetOperation(pBaseAddress,
                                         pPokCfg,
                                         pPokVal,
                                         instance);
        }
        else
        {
            retVal = SDL_EBADARGS;
        }
    }

    return (retVal);
}

/**
 * Design: PROC_SDL-1357,PROC_SDL-1358
 */

int32_t SDL_pokSetControl (SDL_mcuCtrlRegsBase_t           *pBaseAddress,
                             const SDL_POK_config              *pPokCfg,
                             SDL_POK_Inst                      instance)
{
    int32_t     retVal;

    if ((pBaseAddress          == NULL_PTR) ||
        (pPokCfg               == NULL_PTR) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        if ((pPokCfg->hystCtrl     == SDL_PWRSS_GET_HYSTERESIS_VALUE) ||
            (pPokCfg->hystCtrlOV   == SDL_PWRSS_GET_HYSTERESIS_VALUE) ||
            (pPokCfg->voltDetMode  == SDL_PWRSS_GET_VOLTAGE_DET_MODE) ||
            (pPokCfg->trim         == SDL_PWRSS_GET_TRIM_VALUE) ||
            (pPokCfg->trimOV       == SDL_PWRSS_GET_TRIM_VALUE) ||
            (pPokCfg->detectionCtrl == SDL_POK_GET_DETECTION_VALUE) ||
            (pPokCfg->pokEnSelSrcCtrl == SDL_POK_GET_ENSEL_VALUE) ||
            (pPokCfg->deglitch     == SDL_PWRSS_DEGLITCH_GET_VALUE))
        {
            retVal = SDL_EBADARGS;
        }
        else if ((pPokCfg->hystCtrl        <= SDL_PWRSS_HYSTERESIS_NO_ACTION) &&
                 (pPokCfg->hystCtrlOV      <= SDL_PWRSS_HYSTERESIS_NO_ACTION) &&
                 (pPokCfg->voltDetMode     <= SDL_PWRSS_VOLTAGE_DET_NO_ACTION) &&
                 (pPokCfg->trim            <= SDL_PWRSS_TRIM_NO_ACTION) &&
                 (pPokCfg->trimOV          <= SDL_PWRSS_TRIM_NO_ACTION) &&
                 (pPokCfg->detectionCtrl   <  SDL_POK_GET_DETECTION_VALUE) &&
                 (pPokCfg->pokEnSelSrcCtrl <  SDL_POK_GET_ENSEL_VALUE) &&
                 (pPokCfg->deglitch        <= SDL_PWRSS_DEGLITCH_NO_ACTION))
        {
            retVal = SDL_pokSetOperation(pBaseAddress,
                                         pPokCfg,
                                         instance);
        }
        else
        {
            retVal = SDL_EBADARGS;
        }
    }
    return (retVal);}
	
	