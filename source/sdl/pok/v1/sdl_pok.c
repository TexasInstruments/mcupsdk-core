/*
 * @file  sdl_pok.c
 *
 * @brief
 *  C implementation file for the POK module SDL-FL.
 *
 *  Contains the different control command and status query functions definitions
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
#include <sdl/pok/v1/sdl_pok_def.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/esm/v0/v0_0/sdl_esm_priv.h>
#include <sdl/pok/v1/sdl_ip_pok.h>
#include <sdl/pok/v1/sdl_pok.h>
#include <sdl/include/sdlr.h>

#if defined (SOC_AM64X)
#if defined (M4F_CORE)  || defined (R5F_CORE)
#include <sdl/pok/v1/soc/am64x/sdl_soc_pok.h>
#endif
#endif

#if defined (SOC_AM243X)
#if defined (R5F_CORE)
#include <sdl/pok/v1/soc/am243x/sdl_soc_pok.h>
#endif
#endif

/* delay for 1us*/
#define DELAY 0

/**
 * Design: PROC_SDL-1349,PROC_SDL-1162
 */
/*function reads the values of static registers such as hysteresis control,voltage detect mode, trim ,
PORGAP and module status */
static bool POK_firstTime = true;
int32_t SDL_POK_getStaticRegisters(SDL_POK_Inst instance,SDL_POK_staticRegs *pStaticRegs)
{
    int32_t    retVal = SDL_EBADARGS;
    SDL_pokShiftsAndMasks_t    shiftsNMasks;
    uint32_t pbaseAddress;
    SDL_POK_getBaseaddr(SDL_POK_MCU_CTRL_MMR0, &pbaseAddress);
    SDL_mcuCtrlRegsBase_t    *pBaseAddr = (SDL_mcuCtrlRegsBase_t *) pbaseAddress;

    if((pStaticRegs != NULL) && (instance <= SDL_LAST_POK_ID) \
        && (instance >= SDL_FIRST_POK_ID))
    {
        const    SDL_mcu_ctrl_mmr_cfg0Regs    *pCtrlMMRCfgRegs = (const SDL_mcu_ctrl_mmr_cfg0Regs *)pBaseAddr;
        retVal = SDL_pok_GetShiftsAndMasks(pBaseAddr, instance, &shiftsNMasks);

    if (shiftsNMasks.pokAddr != 0U)
    {
        pStaticRegs->hystCtrl = (SDL_pwrss_hysteresis)SDL_REG32_FEXT_RAW(shiftsNMasks.pokAddr, shiftsNMasks.hystMask, shiftsNMasks.hystShift);
    }
    else
    {
        pStaticRegs->hystCtrl = (SDL_pwrss_hysteresis)0x0u;
    }

    if (shiftsNMasks.pokOVAddr != 0U)
    {
        pStaticRegs->hystCtrlOV = (SDL_pwrss_hysteresis)SDL_REG32_FEXT_RAW(shiftsNMasks.pokOVAddr, shiftsNMasks.hystMask, shiftsNMasks.hystShift);
    }
    else
    {
        pStaticRegs->hystCtrlOV = (SDL_pwrss_hysteresis)0x0u;
    }

	if (shiftsNMasks.pokAddr != 0U)
    {
        pStaticRegs->voltDetMode = (SDL_pwrss_vd_mode)SDL_REG32_FEXT_RAW(shiftsNMasks.pokAddr, shiftsNMasks.vdDetMask, shiftsNMasks.vdDetShift);
    }
	else
    {
        pStaticRegs->voltDetMode = (SDL_pwrss_hysteresis)0x0u;
    }

    if ((shiftsNMasks.pokAddr != 0U) && (shiftsNMasks.trimMask != 0U))
    {
        pStaticRegs->trim = (SDL_pwrss_trim)SDL_REG32_FEXT_RAW(shiftsNMasks.pokAddr, shiftsNMasks.trimMask, shiftsNMasks.trimShift);
    }
    else
    {
        pStaticRegs->trim = (SDL_pwrss_trim)0x0u;
    }

    if (shiftsNMasks.pokOVAddr != 0U)
    {
        pStaticRegs->trimOV = (SDL_pwrss_trim)SDL_REG32_FEXT_RAW(shiftsNMasks.pokOVAddr, shiftsNMasks.trimOVMask, shiftsNMasks.trimOVShift);
    }
    else
    {
        pStaticRegs->trimOV = (SDL_pwrss_trim)0x0u;
    }
        pStaticRegs->porBGapOK = (bool) ((SDL_REG32_FEXT(&pCtrlMMRCfgRegs->POR_STAT,MCU_CTRL_MMR_CFG0_POR_STAT_BGOK) == 0x0u)?false:true);

        pStaticRegs->porModuleStatus = (SDL_por_module_status)SDL_REG32_FEXT(&pCtrlMMRCfgRegs->POR_STAT,MCU_CTRL_MMR_CFG0_POR_STAT_SOC_POR);

    }
    return retVal;
}

static void SDL_pokDisableAll(void)
{
    SDL_POK_config             pokCfg;
    uint32_t pbaseAddress;
    SDL_POK_getBaseaddr(SDL_POK_MCU_CTRL_MMR0, &pbaseAddress);
    SDL_mcuCtrlRegsBase_t    *pBaseAddr = (SDL_mcuCtrlRegsBase_t *) pbaseAddress;
    SDL_POK_Inst instance;
    int8_t i = 0U;

    for (i = 0U; i <= (int8_t)SDL_LAST_POK_ID; i++)
    {
        instance = i;
        /* Disable the detection control */
        pokCfg.hystCtrl      = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pokCfg.hystCtrlOV    = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pokCfg.voltDetMode   = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
        pokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
        pokCfg.trim      = SDL_PWRSS_TRIM_NO_ACTION;
        pokCfg.trimOV    = SDL_PWRSS_TRIM_NO_ACTION;
        pokCfg.detectionCtrl = SDL_POK_DETECTION_DISABLE;
        pokCfg.deglitch      = SDL_PWRSS_DEGLITCH_NO_ACTION;
        (void)SDL_pokSetControl(pBaseAddr,&pokCfg,instance);
    }
}

static bool SDL_pokIsPPEnabled(SDL_POK_Inst instance)
{
    SDL_pokShiftsAndMasks_t    shiftsNMasks;
    uint32_t pbaseAddress;
    SDL_POK_getBaseaddr(SDL_POK_MCU_CTRL_MMR0, &pbaseAddress);
    SDL_mcuCtrlRegsBase_t    *pBaseAddr = (SDL_mcuCtrlRegsBase_t *) pbaseAddress;
    bool isPPEnabled = (bool)FALSE;

    if((instance <= SDL_LAST_POK_ID) \
        && (instance >= SDL_FIRST_POK_ID))
    {
        (void)SDL_pok_GetShiftsAndMasks(pBaseAddr, instance, &shiftsNMasks);

        if ((shiftsNMasks.pokEnPPAddr != 0x0u) &&
            (SDL_REG32_FEXT_RAW(shiftsNMasks.pokEnPPAddr, shiftsNMasks.pokEnPPMask, shiftsNMasks.pokEnPPShift) == SDL_PWRSS_PP_MODE_ENABLE))
        {
           isPPEnabled = (bool)TRUE;
        }
    }

    return isPPEnabled;
}

/**
 * Design: PROC_SDL-1163
 */
static int32_t SDL_POK_Thres_config_seq(SDL_POK_Inst instance, SDL_POK_config *pInitCfg, uint32_t esm_err_sig_uv, uint32_t esm_err_sig_ov)
{
    SDL_POK_config          pokCfg;
    SDL_pokVal_t            pokVal;
    int32_t                 retVal = SDL_EFAIL;
    uint32_t pbaseAddress;
    SDL_POK_getBaseaddr(SDL_POK_MCU_CTRL_MMR0, &pbaseAddress);
    SDL_mcuCtrlRegsBase_t    *pBaseAddr = (SDL_mcuCtrlRegsBase_t *) pbaseAddress;
	uint32_t     esmBaseAddr;
	SDL_ESM_getBaseAddr(SDL_ESM_INST_MCU_ESM0,&esmBaseAddr);
    uint32_t                influenceUV = 0U;
    uint32_t                influenceOV = 0U;

    /* Check if this PRG is set for PP */
    if ((SDL_pokIsPPEnabled(instance) == (bool)true) && (pInitCfg->voltDetMode != SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE))
    {
        retVal = SDL_EFAIL;
    }
    else
    {

        /* POK configuration */
        /* Step 1: Mask POK event propogation by programming ESM_INTR_EN_CLR reg */
        (void)SDL_ESM_disableIntr(ESM_INSTANCE, esm_err_sig_uv);
        (void)SDL_ESM_getInfluenceOnErrPin(esmBaseAddr, esm_err_sig_uv, &influenceUV);
        if (influenceUV == 1U)
        {
            /* Disable the error pin temporarily */
            SDL_ESM_setInfluenceOnErrPin(esmBaseAddr, esm_err_sig_uv, (bool)false);
        }
        (void)SDL_ESM_disableIntr(ESM_INSTANCE, esm_err_sig_ov);
        (void)SDL_ESM_getInfluenceOnErrPin(esmBaseAddr, esm_err_sig_ov, &influenceOV);
        if (influenceOV == 1U)
        {
            /* Disable the error pin temporarily */
            SDL_ESM_setInfluenceOnErrPin(esmBaseAddr, esm_err_sig_ov, (bool)false);
        }

        /* Step 2: Is POK Disabled, if not disable it */
        pokCfg.hystCtrl      = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pokCfg.hystCtrlOV    = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pokCfg.voltDetMode   = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
        pokCfg.pokEnSelSrcCtrl = SDL_POK_GET_ENSEL_VALUE;
        pokCfg.trim      = SDL_PWRSS_TRIM_NO_ACTION;
        pokCfg.trimOV    = SDL_PWRSS_TRIM_NO_ACTION;
        pokCfg.detectionCtrl = SDL_POK_GET_DETECTION_VALUE;
        pokCfg.deglitch  = SDL_PWRSS_DEGLITCH_NO_ACTION;
        retVal = SDL_pokGetControl (pBaseAddr, &pokCfg, &pokVal, instance);

        if (pokVal.detectionStatus == SDL_POK_DETECTION_ENABLED)
        {
            /* Disable the detection control */
            pokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
            pokCfg.detectionCtrl = SDL_POK_DETECTION_DISABLE;
            retVal = SDL_pokSetControl(pBaseAddr,&pokCfg,instance);
        }

        if (POK_firstTime == (bool)true)
        {
            SDL_pokDisableAll();
            POK_firstTime = false;
        }

        /* Set to MMR control for POK */
        pokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
        pokCfg.detectionCtrl = SDL_POK_DETECTION_NO_ACTION;
        retVal = SDL_pokSetControl(pBaseAddr, &pokCfg, instance);

        /* Step 3: Program the appropriate threshold settings and hyst in POK_CTRL reg */
        pokCfg.hystCtrl      = pInitCfg->hystCtrl;
        pokCfg.hystCtrlOV    = pInitCfg->hystCtrlOV;
        pokCfg.voltDetMode   = pInitCfg->voltDetMode;
        pokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
        pokCfg.trim      = pInitCfg->trim;
        pokCfg.trimOV    = pInitCfg->trimOV;
        pokCfg.deglitch  = pInitCfg->deglitch;
        retVal = SDL_pokSetControl(pBaseAddr,&pokCfg,instance);

        /* Step 4: Enable the desired POK */
        pokCfg.hystCtrl      = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pokCfg.hystCtrlOV    = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pokCfg.voltDetMode   = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
        pokCfg.trim      = SDL_PWRSS_TRIM_NO_ACTION;
        pokCfg.trimOV    = SDL_PWRSS_TRIM_NO_ACTION;
        pokCfg.detectionCtrl = SDL_POK_DETECTION_ENABLE;
        retVal = SDL_pokSetControl(pBaseAddr,&pokCfg,instance);

        /* Step 5: Wait for 100 us for the POK to settle */
       SDL_DPL_delay(DELAY);

        /* Step 7: If Power Good == Yes, unmask POK to ESM event propagation by
                    programming ESM_INTR_EN_SET register(s) else report to MCU_ESM0*/
        if (retVal == SDL_PASS)
        {
            SDL_ESM_clearIntrStatus(esmBaseAddr, esm_err_sig_ov);
            SDL_ESM_clearIntrStatus(esmBaseAddr, esm_err_sig_uv);

            if (influenceOV == 1U)
            {
                /* Re-enable the error pin */
                (void)SDL_ESM_setInfluenceOnErrPin(esmBaseAddr, esm_err_sig_ov, (bool)true);
            }
            if (influenceUV == 1U)
            {
                (void)SDL_ESM_setInfluenceOnErrPin(esmBaseAddr, esm_err_sig_uv, (bool)true);
            }

            (void)SDL_ESM_enableIntr(esmBaseAddr, esm_err_sig_ov);
            (void)SDL_ESM_enableIntr(esmBaseAddr, esm_err_sig_uv);
        }
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-1163
 */
static int32_t SDL_POR_Thres_config_seq(SDL_POK_Inst instance, SDL_POK_config *pInitCfg, uint32_t esm_err_sig)
{
   uint32_t pbaseAddress;
    SDL_POK_getBaseaddr(SDL_POK_MCU_CTRL_MMR0, &pbaseAddress);
    SDL_mcuCtrlRegsBase_t    *pBaseAddr = (SDL_mcuCtrlRegsBase_t *) pbaseAddress;
	uint32_t     esmBaseAddr;
	SDL_ESM_getBaseAddr(SDL_ESM_INST_MCU_ESM0,&esmBaseAddr);
    SDL_pokPorCfg_t              porCfg;
    SDL_pokVal_t                 pokVal;
    SDL_POK_config               pokCfg;
    int32_t                      retVal = SDL_EFAIL;
    uint32_t                     influence = 0U;

    /* POR configuration */
    /* Step 1: MASKHHV set to 1 */
    porCfg.maskHHVOutputEnable                 = TRUE;

    /* Step 2: TRIM Mux selection is set to 0 */
    porCfg.trim_select                         = SDL_POR_TRIM_SELECTION_FROM_CTRL_REGS;

    retVal = SDL_porSetControl(pBaseAddr,&porCfg);

    /* Step 3: Is POK Disabled, if not disable it */
    pokCfg.hystCtrl      = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pokCfg.hystCtrlOV    = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pokCfg.voltDetMode   = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
    pokCfg.pokEnSelSrcCtrl = SDL_POK_GET_ENSEL_VALUE;
    pokCfg.trim      = SDL_PWRSS_TRIM_NO_ACTION;
    pokCfg.trimOV    = SDL_PWRSS_TRIM_NO_ACTION;
    pokCfg.detectionCtrl = SDL_POK_GET_DETECTION_VALUE;
    pokCfg.deglitch  = SDL_PWRSS_DEGLITCH_NO_ACTION;
    retVal = SDL_pokGetControl (pBaseAddr, &pokCfg, &pokVal, instance);


    if (pokVal.detectionStatus == SDL_POK_DETECTION_ENABLED)
    {
        /* Disable the detection control */
        pokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
        pokCfg.detectionCtrl = SDL_POK_DETECTION_DISABLE;
        retVal = SDL_pokSetControl(pBaseAddr,&pokCfg,instance);
    }

    if (POK_firstTime == (bool)true)
    {
        SDL_pokDisableAll();
        POK_firstTime = false;
    }

    /* Set to MMR control for POK */
    pokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
    pokCfg.detectionCtrl = SDL_POK_DETECTION_NO_ACTION;
    retVal = SDL_pokSetControl(pBaseAddr, &pokCfg, instance);

    /* Step 4: Pending: Mask POK to ESM event propagation by
                    programming ESM_INTR_EN_CLR register(s)
     */
    (void)SDL_ESM_disableIntr(ESM_INSTANCE, esm_err_sig);
    (void)SDL_ESM_getInfluenceOnErrPin(esmBaseAddr, esm_err_sig, &influence);
    if (influence == 1U)
    {
        /* Disable the error pin temporarily */
        SDL_ESM_setInfluenceOnErrPin(esmBaseAddr, esm_err_sig, (bool)false);
    }

    /* Step 5: Program the appropriate threshold settings in POK_CTRL reg */
    pokCfg.hystCtrl      = pInitCfg->hystCtrl;
    pokCfg.hystCtrlOV    = pInitCfg->hystCtrlOV;
    pokCfg.voltDetMode   = pInitCfg->voltDetMode;
    pokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
    pokCfg.trim      = pInitCfg->trim;
    pokCfg.trimOV    = pInitCfg->trimOV;
    pokCfg.deglitch  = pInitCfg->deglitch;
    retVal = SDL_pokSetControl(pBaseAddr,&pokCfg,instance);

    /* Step 6: Enable the desired POK */
    if (retVal == SDL_PASS)
    {
        pokCfg.voltDetMode   = pInitCfg->voltDetMode;
        pokCfg.trim      = SDL_PWRSS_TRIM_NO_ACTION;
        pokCfg.trimOV    = SDL_PWRSS_TRIM_NO_ACTION;
        pokCfg.detectionCtrl = SDL_POK_DETECTION_ENABLE;
        retVal = SDL_pokSetControl(pBaseAddr,&pokCfg,instance);
    }

    /* Step 7: Wait for 100 us for the POK to settle */
    SDL_DPL_delay(DELAY); /* has 1000 micro seconds delay, more than needed */

    /* Step 8: If Power Good == Yes, unmask POK to ESM event propagation by
                    programming ESM_INTR_EN_SET register(s) else report to MCU_ESM0*/
    if (retVal == SDL_PASS)
    {
        SDL_ESM_clearIntrStatus(esmBaseAddr, esm_err_sig);
        if (influence == 1U)
        {
            /* Re-enable the error pin */
            (void)SDL_ESM_setInfluenceOnErrPin(esmBaseAddr, esm_err_sig, (bool)true);
        }
        (void)SDL_ESM_enableIntr(esmBaseAddr, esm_err_sig);
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-3290
 */

int32_t SDL_POK_enablePP(SDL_PRG_Inst instance, bool enable)
{
    int32_t retVal = SDL_PASS;
    /* Check PRG range */
    uint32_t pbaseAddress;
    SDL_POK_getBaseaddr(SDL_POK_MCU_CTRL_MMR0, &pbaseAddress);
    SDL_mcuCtrlRegsBase_t    *pBaseAddr = (SDL_mcuCtrlRegsBase_t *) pbaseAddress;
    SDL_pokPRGInfo_t prgInfo;

    if((instance <= SDL_POK_PRG_LAST_ID) \
            && (instance >= SDL_POK_PRG_FIRST_ID))
    {
        retVal = SDL_pok_getPRGInfo(pBaseAddr, instance, &prgInfo);

        if ((retVal == SDL_PASS) && (prgInfo.pokEnPPMask != 0x0u))
        {
                /* Allows enable/disable of PP mode for a particular PRG */
                if (enable == (bool)TRUE)
                {
                    SDL_REG32_FINS_RAW(prgInfo.addr, prgInfo.pokEnPPMask, prgInfo.pokEnPPShift, SDL_PWRSS_PP_MODE_ENABLE);
                }
                else
                {
                    SDL_REG32_FINS_RAW(prgInfo.addr, prgInfo.pokEnPPMask, prgInfo.pokEnPPShift, SDL_PWRSS_PP_MODE_DISABLE);
                }
        }
        else
        {
            retVal = SDL_EFAIL;
        }
    }
    else
    {
        retVal = SDL_EBADARGS;
    }
    return retVal;
}


/**
 * Design: PROC_SDL-1344,PROC_SDL-1345,PROC_SDL-1346
 */
/*set POK and POR_POK module configurations*/
int32_t SDL_POK_init(SDL_POK_Inst instance,  SDL_POK_config *pPokCfg)
{
    int32_t                 retVal = SDL_PASS;
    SDL_POK_Inst            esm_inst;
    bool                    usePorCfgFlag;
    uint32_t                esm_err_sig_ov;
    uint32_t                esm_err_sig_uv;

    if (pPokCfg == NULL)
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        sdlGetErrSig(instance, &esm_inst, &esm_err_sig_uv, &esm_err_sig_ov, &usePorCfgFlag);
        if (instance != esm_inst)
        {
            retVal = SDL_EBADARGS;
        }
        else if ((pPokCfg->trim > SDL_PWRSS_TRIM_NO_ACTION) ||
                 (pPokCfg->trimOV > SDL_PWRSS_TRIM_NO_ACTION) ||
                 (pPokCfg->deglitch == SDL_PWRSS_DEGLITCH_GET_VALUE) ||
                 (pPokCfg->deglitch > SDL_PWRSS_DEGLITCH_NO_ACTION) ||
                 (pPokCfg->hystCtrl == SDL_PWRSS_GET_HYSTERESIS_VALUE) ||
                 (pPokCfg->hystCtrlOV > SDL_PWRSS_HYSTERESIS_NO_ACTION) ||
                 (pPokCfg->voltDetMode == SDL_PWRSS_GET_VOLTAGE_DET_MODE) ||
                 (pPokCfg->voltDetMode > SDL_PWRSS_VOLTAGE_DET_NO_ACTION) ||
                 (pPokCfg->detectionCtrl > SDL_POK_DETECTION_NO_ACTION) ||
                 (pPokCfg->pokEnSelSrcCtrl > SDL_POK_ENSEL_NO_ACTION))
        {
            retVal = SDL_EBADARGS;
        }
        else if (usePorCfgFlag == (bool)false)
        {
            retVal = SDL_POK_Thres_config_seq(instance, pPokCfg, esm_err_sig_uv, esm_err_sig_ov);
        }
        else
        {
            if (esm_err_sig_uv != (uint32_t)(-1))
            {
                retVal = SDL_POR_Thres_config_seq(instance, pPokCfg, esm_err_sig_uv);
            }
            else
            {
                retVal = SDL_POR_Thres_config_seq(instance, pPokCfg, esm_err_sig_ov);
            }
        }
    }

    return retVal;
}

/**
 * Design: PROC_SDL-1348,PROC_SDL-1161
 */

/*Verify POK module configurations*/
int32_t SDL_POK_verifyConfig(SDL_POK_Inst instance, SDL_POK_config *pPokCfg )
{
    int32_t              retVal= SDL_EFAIL;
    SDL_POK_config       pokCfg;
    SDL_pokVal_t         pPokVal;
    bool                 compare_hyst = false;
    bool                 compare_voltmode = false;
    bool                 compare_ensel = false;
    bool                 compare_trim = false;
    bool                 compare_detctrl = false;
    bool                 compare_hystOV = false;
    bool                 compare_trimOV = false;
    bool                 compare_deglitch = false;

    uint32_t pbaseAddress;
    SDL_POK_getBaseaddr(SDL_POK_MCU_CTRL_MMR0, &pbaseAddress);
    SDL_mcuCtrlRegsBase_t    *pBaseAddr = (SDL_mcuCtrlRegsBase_t *) pbaseAddress;

    if (pPokCfg == NULL_PTR)
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        if ((pPokCfg->hystCtrl != SDL_PWRSS_HYSTERESIS_NO_ACTION) &&
            (pPokCfg->hystCtrl != SDL_PWRSS_GET_HYSTERESIS_VALUE))
        {
            pokCfg.hystCtrl = SDL_PWRSS_GET_HYSTERESIS_VALUE;
            compare_hyst = true;
        }
        else
        {
            pokCfg.hystCtrl = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        }

        if ((pPokCfg->hystCtrlOV != SDL_PWRSS_HYSTERESIS_NO_ACTION) &&
            (pPokCfg->hystCtrlOV != SDL_PWRSS_GET_HYSTERESIS_VALUE))
        {
            pokCfg.hystCtrlOV = SDL_PWRSS_GET_HYSTERESIS_VALUE;
            compare_hystOV = true;
        }
        else
        {
            pokCfg.hystCtrlOV = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        }

        if ((pPokCfg->voltDetMode != SDL_PWRSS_VOLTAGE_DET_NO_ACTION) &&
            (pPokCfg->voltDetMode != SDL_PWRSS_GET_VOLTAGE_DET_MODE))
        {
            pokCfg.voltDetMode = SDL_PWRSS_GET_VOLTAGE_DET_MODE;
            compare_voltmode = true;
        }
        else
        {
            pokCfg.voltDetMode = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
        }

        if ((pPokCfg->pokEnSelSrcCtrl != SDL_POK_ENSEL_NO_ACTION) &&
            (pPokCfg->pokEnSelSrcCtrl != SDL_POK_GET_ENSEL_VALUE))
        {
            pokCfg.pokEnSelSrcCtrl = SDL_POK_GET_ENSEL_VALUE;
            compare_ensel = true;
        }
        else
        {
            pokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
        }
        if ((pPokCfg->trim != SDL_PWRSS_TRIM_NO_ACTION) &&
            (pPokCfg->trim != SDL_PWRSS_GET_TRIM_VALUE))
        {
            pokCfg.trim = SDL_PWRSS_GET_TRIM_VALUE;
            compare_trim = true;
        }
        else
        {
            pokCfg.trim = SDL_PWRSS_TRIM_NO_ACTION;
        }

        if ((pPokCfg->trimOV != SDL_PWRSS_TRIM_NO_ACTION) &&
            (pPokCfg->trimOV != SDL_PWRSS_GET_TRIM_VALUE))
        {
            pokCfg.trimOV = SDL_PWRSS_GET_TRIM_VALUE;
            compare_trimOV = true;
        }
        else
        {
            pokCfg.trimOV = SDL_PWRSS_TRIM_NO_ACTION;
        }

        if ((pPokCfg->detectionCtrl != SDL_POK_DETECTION_NO_ACTION) &&
            (pPokCfg->detectionCtrl != SDL_POK_GET_DETECTION_VALUE))
        {
            pokCfg.detectionCtrl = SDL_POK_GET_DETECTION_VALUE;
            compare_detctrl = true;
        }
        else
        {
            pokCfg.detectionCtrl = SDL_POK_DETECTION_NO_ACTION;
        }

        if ((pPokCfg->deglitch != SDL_PWRSS_DEGLITCH_NO_ACTION) &&
            (pPokCfg->deglitch != SDL_PWRSS_DEGLITCH_GET_VALUE))
        {
            pokCfg.deglitch = SDL_PWRSS_DEGLITCH_GET_VALUE;
            compare_deglitch = true;
        }
        else
        {
            pokCfg.deglitch = SDL_PWRSS_DEGLITCH_NO_ACTION;
        }

        retVal = SDL_pokGetControl (pBaseAddr, &pokCfg, &pPokVal, instance);
        if (retVal == SDL_PASS)
        {
            if (((compare_hyst == (bool)true) && (pPokVal.hystCtrl != pPokCfg->hystCtrl)) ||
                ((compare_hystOV == (bool)true) && (pPokVal.hystCtrlOV != pPokCfg->hystCtrlOV)) ||
                ((compare_voltmode == (bool)true) && (pPokVal.voltDetMode != pPokCfg->voltDetMode)) ||
                ((compare_trim == (bool)true) && (pPokVal.trim != pPokCfg->trim)) ||
                ((compare_trimOV == (bool)true) && (pPokVal.trimOV != pPokCfg->trimOV)) ||
                ((compare_detctrl == (bool)true) && (pPokVal.detectionStatus != pPokCfg->detectionCtrl)) ||
                ((compare_ensel == (bool)true) && (pPokVal.pokEnSelSrcCtrl != pPokCfg->pokEnSelSrcCtrl)) ||
                ((compare_deglitch == (bool)true) && (pPokVal.deglitch != pPokCfg->deglitch)))
            {
                retVal = SDL_EFAIL;
            }
            else
            {
                retVal = SDL_PASS;
            }
        }
    }
    return (retVal);
}
