/*
 * @file  sdl_pok_ip_defs.c
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
#include "sdl_ip_pok.h"
#include <sdl/pok/v1/sdl_pok.h>
#include <sdl/pok/v1/sdl_pok_def.h>
#include <sdl/include/am64x_am243x/sdlr_mcu_ctrl_mmr.h>
#include <sdl/include/am64x_am243x/sdlr_intr_mcu_esm0.h>

#if defined (SOC_AM64X)
#include <sdl/pok/v1/soc/am64x/sdl_soc_pok.h>
#endif

#if defined (SOC_AM243X)
#if defined (R5F_CORE)
#include <sdl/pok/v1/soc/am243x/sdl_soc_pok.h>
#endif
#endif


/*=============================================================================
 *  Internal definitions and functions
 *===========================================================================*/

/**
 * Design: PROC_SDL-1351
 */
/*function is used to set error signal wrt given POKID*/
void sdlGetErrSig(uint32_t id, SDL_POK_Inst *instance, uint32_t *esm_err_sig_uv, uint32_t *esm_err_sig_ov, bool *usePorCfgFlag)
{
    switch (id)
    {
        case SDL_POK_VDDA_PMIC_IN_ID:
            *instance = SDL_POK_VDDA_PMIC_IN_ID;
            *usePorCfgFlag = FALSE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VDDA_PMIC_IN_UV;
            *esm_err_sig_ov = (uint32_t)(-1);
            break;
        case SDL_POK_VDDS_DDRIO_ID:
            *instance = SDL_POK_VDDS_DDRIO_ID;
            *usePorCfgFlag = FALSE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VDDS_DDRIO_UV;
            *esm_err_sig_ov = MCU_ESM_ERR_SIG_VDDS_DDRIO_OV;
            break;
        case SDL_POK_VDDR_CORE_ID:
            *instance = SDL_POK_VDDR_CORE_ID;
            *usePorCfgFlag = FALSE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VDDR_CORE_UV;
            *esm_err_sig_ov = MCU_ESM_ERR_SIG_VDDR_CORE_OV;
            break;
        case SDL_POK_VDDSHV_MCU_3P3_ID:
            *instance = SDL_POK_VDDSHV_MCU_3P3_ID;
            *usePorCfgFlag = FALSE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VDDSHV_MCU_3P3_UV;
            *esm_err_sig_ov = MCU_ESM_ERR_SIG_VDDSHV_MCU_3P3_OV;
            break;
        case SDL_POK_VDDSHV_MCU_1P8_ID:
            *instance = SDL_POK_VDDSHV_MCU_1P8_ID;
            *usePorCfgFlag = FALSE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VDDSHV_MCU_1P8_UV;
            *esm_err_sig_ov = MCU_ESM_ERR_SIG_VDDSHV_MCU_1P8_OV;
            break;
        case SDL_POK_VMON_CAP_MCU_GENERAL_ID:
            *instance = SDL_POK_VMON_CAP_MCU_GENERAL_ID;
            *usePorCfgFlag = FALSE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_UV;
            *esm_err_sig_ov = MCU_ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_OV;
            break;
        case SDL_POK_VDDSHV_MAIN_1P8_ID:
            *instance = SDL_POK_VDDSHV_MAIN_1P8_ID;
            *usePorCfgFlag = FALSE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VDDSHV_MAIN_1P8_UV;
            *esm_err_sig_ov = MCU_ESM_ERR_SIG_VDDSHV_MAIN_1P8_OV;
            break;
        case SDL_POK_VDDSHV_MAIN_3P3_ID:
            *instance = SDL_POK_VDDSHV_MAIN_3P3_ID;
            *usePorCfgFlag = FALSE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VDDSHV_MAIN_3P3_UV;
            *esm_err_sig_ov = MCU_ESM_ERR_SIG_VDDSHV_MAIN_3P3_OV;
            break;
		 case SDL_POK_VDD_MCU_OV_ID:
            *instance = SDL_POK_VDD_MCU_OV_ID;
            *usePorCfgFlag = FALSE;
            *esm_err_sig_uv = (uint32_t)(-1);
            *esm_err_sig_ov = MCU_ESM_ERR_SIG_VDD_MCU_OV;
            break;
        case SDL_POR_VDDA_MCU_UV_ID:
            *instance = SDL_POR_VDDA_MCU_UV_ID;
            *usePorCfgFlag = TRUE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VDDA_MCU_UV;
            *esm_err_sig_ov = (uint32_t)(-1);
            break;

		case SDL_POR_VDD_MCU_UV_ID:
            *instance = SDL_POR_VDD_MCU_UV_ID;
            *usePorCfgFlag = TRUE;
            *esm_err_sig_uv = MCU_ESM_ERR_SIG_VDD_MCU_UV;
            *esm_err_sig_ov = (uint32_t)(-1);
            break;

		case SDL_POR_VDDA_MCU_OV_ID:
            *instance = SDL_POR_VDDA_MCU_OV_ID;
            *usePorCfgFlag = TRUE;
            *esm_err_sig_uv = (uint32_t)(-1);
            *esm_err_sig_ov = MCU_ESM_ERR_SIG_VDDA_MCU_OV;
            break;



        default:
           *instance = (SDL_POK_Inst)(-1);
            break;
    }
    return;
}

/**
 * Design: PROC_SDL-3289
 */

int32_t SDL_pok_getPRGInfo(SDL_mcuCtrlRegsBase_t     *pBaseAddress,
                   SDL_PRG_Inst instance,
               SDL_pokPRGInfo_t *pPRGInfo)
{
    SDL_mcu_ctrl_mmr_cfg0Regs      *pCtrlMMRCfgRegs = \
                                    (SDL_mcu_ctrl_mmr_cfg0Regs *)pBaseAddress;
    int32_t                         retVal = SDL_PASS;

    switch (instance)
    {
         case SDL_POK_PRG_PP_0_ID:
             pPRGInfo->addr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pPRGInfo->pokEnPPMask = 0x0u;
             pPRGInfo->pokEnPPShift = 0x0u;
             break;
         case SDL_POK_PRG_PP_1_ID:
             pPRGInfo->addr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pPRGInfo->pokEnPPMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_MASK;
             pPRGInfo->pokEnPPShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_SHIFT;
             break;
         default:
             pPRGInfo->addr = NULL;
             retVal = SDL_EFAIL;
             break;
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1353,PROC_SDL-1354
 */

int32_t SDL_pok_GetShiftsAndMasks(SDL_mcuCtrlRegsBase_t     *pBaseAddress,
                                     SDL_POK_Inst  instance,
                                     SDL_pokShiftsAndMasks_t *pShMasks)
{
    SDL_mcu_ctrl_mmr_cfg0Regs      *pCtrlMMRCfgRegs = \
                                    (SDL_mcu_ctrl_mmr_cfg0Regs *)pBaseAddress;
    int32_t                         retVal = SDL_PASS;

    switch (instance)
    {
         /* PMIC POK ID */
        case    SDL_POK_VDDA_PMIC_IN_ID:
             pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VDDA_PMIC_IN_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_PMIC_IN_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_PMIC_IN_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_PMIC_IN_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_PMIC_IN_CTRL_OVER_VOLT_DET_SHIFT;
             /* Note: PMIC POK does not have a trim setting */
             pShMasks->trimMask    = 0x0;

	     pShMasks->pokOVAddr   = NULL;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDDA_PMIC_IN_UV_EN_MASK;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDDA_PMIC_IN_UV_EN_SHIFT;
             /* Note: PMIC POK does not have an OV Sel setting */
             pShMasks->ovSelMask   = 0x0;

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_MASK;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_SHIFT;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_MASK ;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_SHIFT;

             /* POK Enable PP - No PP support for this POK */
             pShMasks->pokEnPPAddr = NULL;
             break;

	    case     SDL_POK_VDDS_DDRIO_ID:

             pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VDDS_DDRIO_UV_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_UV_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_UV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_UV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_UV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_UV_CTRL_POK_TRIM_MASK;
             pShMasks->trimShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_UV_CTRL_POK_TRIM_SHIFT;

             pShMasks->pokOVAddr     = &pCtrlMMRCfgRegs->POK_VDDS_DDRIO_OV_CTRL;
             pShMasks->hystOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_OV_CTRL_HYST_EN_MASK;
             pShMasks->hystOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_OV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetOVMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_OV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetOVShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_OV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_OV_CTRL_POK_TRIM_MASK;
             pShMasks->trimOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDS_DDRIO_OV_CTRL_POK_TRIM_SHIFT;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDS_DDRIO_EN_MASK;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDS_DDRIO_EN_SHIFT;
             pShMasks->ovSelMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDS_DDRIO_OV_SEL_MASK;
             pShMasks->ovSelShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDS_DDRIO_OV_SEL_SHIFT ;

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_MASK ;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_SHIFT ;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_MASK ;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_SHIFT ;

             /* POK Enable PP */
             pShMasks->pokEnPPAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnPPMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_MASK ;
             pShMasks->pokEnPPShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_SHIFT ;
	     break;

        case     SDL_POK_VDDR_CORE_ID:

             pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VDDR_CORE_UV_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_UV_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_UV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_UV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_UV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_UV_CTRL_POK_TRIM_MASK;
             pShMasks->trimShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_UV_CTRL_POK_TRIM_SHIFT;

             pShMasks->pokOVAddr     = &pCtrlMMRCfgRegs->POK_VDDR_CORE_OV_CTRL;
             pShMasks->hystOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_OV_CTRL_HYST_EN_MASK;
             pShMasks->hystOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_OV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetOVMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_OV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetOVShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_OV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_OV_CTRL_POK_TRIM_MASK;
             pShMasks->trimOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDR_CORE_OV_CTRL_POK_TRIM_SHIFT;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDR_CORE_EN_MASK;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDR_CORE_EN_SHIFT;
             pShMasks->ovSelMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDR_CORE_OV_SEL_MASK;
             pShMasks->ovSelShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDR_CORE_OV_SEL_SHIFT;

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_MASK ;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_SHIFT;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_MASK;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_SHIFT;

             /* POK Enable PP */
             pShMasks->pokEnPPAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnPPMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_MASK ;
             pShMasks->pokEnPPShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_SHIFT;
	     break;
     case     SDL_POK_VDDSHV_MCU_3P3_ID:

             pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VDDSHV_MCU_3P3_UV_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_UV_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_UV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_UV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_UV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_UV_CTRL_POK_TRIM_MASK;
             pShMasks->trimShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_UV_CTRL_POK_TRIM_SHIFT;

             pShMasks->pokOVAddr     = &pCtrlMMRCfgRegs->POK_VDDSHV_MCU_3P3_OV_CTRL;
             pShMasks->hystOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_OV_CTRL_HYST_EN_MASK;
             pShMasks->hystOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_OV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetOVMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_OV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetOVShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_OV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_OV_CTRL_POK_TRIM_MASK;
             pShMasks->trimOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_3P3_OV_CTRL_POK_TRIM_SHIFT;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MCU_3P3_EN_MASK;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MCU_3P3_EN_SHIFT;
             pShMasks->ovSelMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MCU_3P3_OV_SEL_MASK;
             pShMasks->ovSelShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MCU_3P3_OV_SEL_SHIFT;

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_MASK;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_SHIFT;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_MASK;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_SHIFT;

             /* POK Enable PP */
             pShMasks->pokEnPPAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnPPMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_MASK;
             pShMasks->pokEnPPShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_SHIFT;
	     break;


	 case     SDL_POK_VMON_CAP_MCU_GENERAL_ID:
	         pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VMON_CAP_MCU_GENERAL_UV_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_UV_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_UV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_UV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_UV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_UV_CTRL_POK_TRIM_MASK;
             pShMasks->trimShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_UV_CTRL_POK_TRIM_SHIFT;

             pShMasks->pokOVAddr     = &pCtrlMMRCfgRegs->POK_VMON_CAP_MCU_GENERAL_OV_CTRL;
             pShMasks->hystOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_OV_CTRL_HYST_EN_MASK;
             pShMasks->hystOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_OV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetOVMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_OV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetOVShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_OV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_OV_CTRL_POK_TRIM_MASK;
             pShMasks->trimOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VMON_CAP_MCU_GENERAL_OV_CTRL_POK_TRIM_SHIFT;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VMON_CAP_MCU_GENERAL_EN_MASK ;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VMON_CAP_MCU_GENERAL_EN_SHIFT;
             pShMasks->ovSelMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VMON_CAP_MCU_GENERAL_OV_SEL_MASK;
             pShMasks->ovSelShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VMON_CAP_MCU_GENERAL_OV_SEL_SHIFT;

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_MASK;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_SHIFT;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_MASK;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_SHIFT;

             /* POK Enable PP */
             pShMasks->pokEnPPAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnPPMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_MASK;
             pShMasks->pokEnPPShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_SHIFT;
	     break;

	 case     SDL_POK_VDDSHV_MAIN_1P8_ID:
             pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VDDSHV_MAIN_1P8_UV_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_UV_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_UV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_UV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_UV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_UV_CTRL_POK_TRIM_MASK;
             pShMasks->trimShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_UV_CTRL_POK_TRIM_SHIFT;

             pShMasks->pokOVAddr     = &pCtrlMMRCfgRegs->POK_VDDSHV_MAIN_1P8_OV_CTRL;
             pShMasks->hystOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_OV_CTRL_HYST_EN_MASK;
             pShMasks->hystOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_OV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetOVMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_OV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetOVShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_OV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_OV_CTRL_POK_TRIM_MASK;
             pShMasks->trimOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_1P8_OV_CTRL_POK_TRIM_SHIFT;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MAIN_1P8_EN_MASK ;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MAIN_1P8_EN_SHIFT;
             pShMasks->ovSelMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MAIN_1P8_OV_SEL_MASK;
             pShMasks->ovSelShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MAIN_1P8_OV_SEL_SHIFT;

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_MASK ;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_SHIFT ;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_MASK;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_SHIFT;

             /* POK Enable PP */
             pShMasks->pokEnPPAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnPPMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_MASK ;
             pShMasks->pokEnPPShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_SHIFT;
	     break;

	 case     SDL_POK_VDDSHV_MAIN_3P3_ID:
             pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VDDSHV_MAIN_3P3_UV_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_UV_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_UV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_UV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_UV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_UV_CTRL_POK_TRIM_MASK;
             pShMasks->trimShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_UV_CTRL_POK_TRIM_SHIFT;

             pShMasks->pokOVAddr     = &pCtrlMMRCfgRegs->POK_VDDSHV_MAIN_3P3_OV_CTRL;
             pShMasks->hystOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_OV_CTRL_HYST_EN_MASK;
             pShMasks->hystOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_OV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetOVMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_OV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetOVShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_OV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_OV_CTRL_POK_TRIM_MASK;
             pShMasks->trimOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MAIN_3P3_OV_CTRL_POK_TRIM_SHIFT;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MAIN_3P3_EN_MASK;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MAIN_3P3_EN_SHIFT;
             pShMasks->ovSelMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MAIN_3P3_OV_SEL_MASK;
             pShMasks->ovSelShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MAIN_3P3_OV_SEL_SHIFT;

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_MASK;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_SHIFT;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_MASK;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_SHIFT;

             /* POK Enable PP */
             pShMasks->pokEnPPAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnPPMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_MASK ;
             pShMasks->pokEnPPShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_SHIFT;

	     break;
		 case     SDL_POK_VDDSHV_MCU_1P8_ID:
	         pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VDDSHV_MCU_1P8_UV_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_UV_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_UV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_UV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_UV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_UV_CTRL_POK_TRIM_MASK;
             pShMasks->trimShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_UV_CTRL_POK_TRIM_SHIFT;

             pShMasks->pokOVAddr     = &pCtrlMMRCfgRegs->POK_VDDSHV_MCU_1P8_OV_CTRL;
             pShMasks->hystOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_OV_CTRL_HYST_EN_MASK;
             pShMasks->hystOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_OV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetOVMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_OV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetOVShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_OV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_OV_CTRL_POK_TRIM_MASK;
             pShMasks->trimOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDSHV_MCU_1P8_OV_CTRL_POK_TRIM_SHIFT;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MCU_1P8_EN_MASK;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MCU_1P8_EN_SHIFT;
             pShMasks->ovSelMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MCU_1P8_OV_SEL_MASK;
             pShMasks->ovSelShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_VDDSHV_MCU_1P8_OV_SEL_SHIFT;

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_MASK ;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_EN_SEL_SHIFT;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_MASK ;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_DEGLITCH_SEL_SHIFT ;

             /* POK Enable PP */
             pShMasks->pokEnPPAddr = &pCtrlMMRCfgRegs->PRG_PP_1_CTRL;
             pShMasks->pokEnPPMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_MASK;
             pShMasks->pokEnPPShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_1_CTRL_POK_PP_EN_SHIFT;
	     break;


	 case     SDL_POK_VDD_MCU_OV_ID:
              pShMasks->pokAddr     = NULL;



             pShMasks->pokOVAddr     = &pCtrlMMRCfgRegs->POK_VDD_CORE_OV_CTRL;
             pShMasks->hystOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_OV_CTRL_HYST_EN_MASK;
             pShMasks->hystOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_OV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetOVMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_OV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetOVShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_OV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_OV_CTRL_POK_TRIM_MASK;
             pShMasks->trimOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_OV_CTRL_POK_TRIM_SHIFT;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDD_MCU_OV_EN_MASK;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDD_MCU_OV_EN_SHIFT;
             pShMasks->ovSelMask   = 0x0;

              /* POK Enable Selection SRC */
              pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
              pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_MASK;
              pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_SHIFT;

             // /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_MASK;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_SHIFT ;

             /* POK Enable PP */
             pShMasks->pokEnPPAddr = NULL;
		break;

	 case     SDL_POR_VDDA_MCU_UV_ID:
             pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VDDA_MCU_UV_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_UV_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_UV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_UV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_UV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_UV_CTRL_POK_TRIM_MASK;
             pShMasks->trimShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_UV_CTRL_POK_TRIM_SHIFT;

             /* This POR is only for UV control */
             pShMasks->pokOVAddr     = NULL;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDDA_MCU_UV_EN_MASK;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDDA_MCU_UV_EN_SHIFT;
             pShMasks->ovSelMask   = 0x0u; // ov sel not supportted for PRG_PP_POR

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_MASK;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_SHIFT;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_MASK;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_SHIFT;

             /* POK Enable PP - Not supported for PRG_PP_POR_CTRL */
             pShMasks->pokEnPPAddr = NULL;

	     break;
	 case     SDL_POR_VDD_MCU_UV_ID:
             pShMasks->pokAddr     = &pCtrlMMRCfgRegs->POK_VDD_CORE_UV_CTRL;
             pShMasks->hystMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_UV_CTRL_HYST_EN_MASK;
             pShMasks->hystShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_UV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_UV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_UV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_UV_CTRL_POK_TRIM_MASK;
             pShMasks->trimShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDD_CORE_UV_CTRL_POK_TRIM_SHIFT;

	     // /* This POR is only for UV control */
              pShMasks->pokOVAddr     = NULL;

              /* POK Detection enable/disable control */
              pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
              pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDD_MCU_UV_EN_MASK;
              pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDD_MCU_UV_EN_SHIFT;
              pShMasks->ovSelMask   = 0x0u; // ov sel not supported for PRG_PP_POR

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_MASK;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_SHIFT;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_MASK;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_SHIFT;

             /* POK Enable PP - Not supported for PRG_PP_POR_CTRL */
             pShMasks->pokEnPPAddr = NULL;

	     break;

	 case     SDL_POR_VDDA_MCU_OV_ID:
	        pShMasks->pokAddr     = NULL;

             /* This POR is only for OV control */
	         pShMasks->pokOVAddr     = &pCtrlMMRCfgRegs->POK_VDDA_MCU_OV_CTRL;
             pShMasks->hystOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_OV_CTRL_HYST_EN_MASK;
             pShMasks->hystOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_OV_CTRL_HYST_EN_SHIFT;
             pShMasks->vdDetOVMask   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_OV_CTRL_OVER_VOLT_DET_MASK;
             pShMasks->vdDetOVShift  = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_OV_CTRL_OVER_VOLT_DET_SHIFT;
             pShMasks->trimOVMask    = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_OV_CTRL_POK_TRIM_MASK;
             pShMasks->trimOVShift   = SDL_MCU_CTRL_MMR_CFG0_POK_VDDA_MCU_OV_CTRL_POK_TRIM_SHIFT;

             /* POK Detection enable/disable control */
             pShMasks->pokDetAddr  = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->detEnMask   = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDDA_MCU_OV_EN_MASK;
             pShMasks->detEnShift  = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_VDDA_MCU_OV_EN_SHIFT;
             pShMasks->ovSelMask   = 0x0u; // ov sel not supportted for PRG_PP_POR

             /* POK Enable Selection SRC */
             pShMasks->pokEnSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->pokEnSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_MASK;
             pShMasks->pokEnSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_POK_EN_SEL_SHIFT;

             /* POK Deglitch Selection */
             pShMasks->deglitchSelAddr = &pCtrlMMRCfgRegs->PRG_PP_0_CTRL;
             pShMasks->deglitchSelMask = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_MASK ;
             pShMasks->deglitchSelShift = SDL_MCU_CTRL_MMR_CFG0_PRG_PP_0_CTRL_DEGLITCH_SEL_SHIFT ;

             /* POK Enable PP - Not supported for PRG_PP_POR_CTRL */
             pShMasks->pokEnPPAddr = NULL;


	     break;

         default:
             pShMasks->pokAddr      = ( uint32_t * ) 0;
             pShMasks->pokDetAddr   = ( uint32_t * ) 0;
             pShMasks->pokEnSelAddr = ( uint32_t * ) 0;
             retVal  = SDL_EBADARGS;
             break;
     }

     return(retVal);

}

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

