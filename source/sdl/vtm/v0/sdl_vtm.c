/**
 * @file  sdl_vtm.c
 *
 * @brief
 *  C implementation file for the VTM module SDL-FL.
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

#include <stdint.h>
#include <sdl/sdl_vtm.h>
#include <sdl/include/sdl_types.h>

/*=============================================================================
 *  internal macros
 *===========================================================================*/
#define SDL_VTM_VALUES_ARE_UNINITIALIZED    (-1)
/* Delay for Reg Reads */
#define SDL_VTM_DOUT_REG_READ_DELAY         (100)

/*=============================================================================
 *  global variables
 *===========================================================================*/
extern int32_t gNumTempSensors;
extern int32_t gNumCoreVoltageDomains;

/*=============================================================================
 *  Internal functions
 *===========================================================================*/

 /**
 * Design: PROC_SDL-1164,PROC_SDL-1298,PROC_SDL-1299
 */

int32_t SDL_VTM_initVd(SDL_VTM_InstVd instance, const SDL_VTM_configVd *pConfig)
{
    const SDL_VTM_cfg1Regs           *p_cfg1;
    const SDL_VTM_cfg2Regs           *p_cfg2;
	uint32_t baseAddr,baseAddr1;

	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_2, &baseAddr1);
    p_cfg2 = (SDL_VTM_cfg2Regs *) baseAddr1;
    SDL_VTM_vid_opp         vid_opp;
    uint8_t                 vid_opp_val;
    SDL_VTM_vdEvtSel_set   vd_temp_evts;
    SDL_VTM_tsGlobal_cfg    p_tsGlobal_cfg;
    SDL_VTM_configVdCtrl    cfgVdCtrl;

    int32_t                    sdlResult = SDL_PASS;

        /* Argument check for VD, temperature sensor */
    if (gNumCoreVoltageDomains == SDL_VTM_VALUES_ARE_UNINITIALIZED)
    {
        SDL_VTM_getSensorVDCount(p_cfg1);
    }

    /* argument checks */
    if( (pConfig   == NULL_PTR)        ||
        ((int32_t)instance  >= gNumCoreVoltageDomains))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        cfgVdCtrl       =       pConfig->configVdCtrl;
        vid_opp         =       pConfig->vid_opp;
        vid_opp_val     =       pConfig->vid_opp_val;
        vd_temp_evts    =       pConfig->vd_temp_evts;
        p_tsGlobal_cfg  =       pConfig->tsGlobal_cfg;

        if((cfgVdCtrl & SDL_VTM_VD_CONFIG_CTRL_VID_OPP) != 0U)
        {
            /*Set op mode and voltage*/
            (void)SDL_VTM_vdSetOppVid (p_cfg1, instance, vid_opp, vid_opp_val);
        }

        if((cfgVdCtrl & SDL_VTM_VD_CONFIG_CTRL_EVT_SEL) != 0U)
        {
            /*Enable temperature moniture for Voltage domain*/
            (void)SDL_VTM_vdEvtSelSet (p_cfg1, instance, vd_temp_evts);
        }

        if((cfgVdCtrl & SDL_VTM_VD_CONFIG_CTRL_GLB_CFG) != 0U)
        {
            /*Set global configuration for VTM*/
            (void)SDL_VTM_tsSetGlobalCfg (p_cfg2, &p_tsGlobal_cfg);
        }

    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1165,PROC_SDL-1302,PROC_SDL-1300
 */
int32_t SDL_VTM_initTs(SDL_VTM_InstTs instance, const SDL_VTM_configTs *pConfig)
{
    int32_t                        	sdlResult = SDL_PASS;
    SDL_VTM_Ctrlcfg           		tsCtrl_cfg;
    int32_t                        	high_temp_in_mdc;
    int32_t                     	low_temp_in_mdc;
    SDL_VTM_tsThrVal               	thr_val;
    SDL_VTM_configTsCtrl        	cfgTsCtrl;
    const SDL_VTM_cfg1Regs               *p_cfg1;
    const SDL_VTM_cfg2Regs               *p_cfg2;
	uint32_t baseAddr,baseAddr1;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_2, &baseAddr1);
    p_cfg2 = (SDL_VTM_cfg2Regs *) baseAddr1;

    /* Argument check for temperature sensor */
    if (gNumTempSensors == SDL_VTM_VALUES_ARE_UNINITIALIZED)
    {
        SDL_VTM_getSensorVDCount(p_cfg1);
    }

    if( (pConfig   == NULL_PTR)        ||
        ((int32_t)instance  >= gNumTempSensors))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        cfgTsCtrl           =       pConfig->configTsCtrl;
        tsCtrl_cfg          =       pConfig->tsCtrl_cfg;
        high_temp_in_mdc    =       pConfig->high_temp_in_milli_degree_celsius;
        low_temp_in_mdc     =       pConfig->low_temp_in_milli_degree_celsius;
        thr_val             =       pConfig->thr_val;

        if((cfgTsCtrl & (SDL_VTM_configTsCtrl)SDL_VTM_VD_CONFIG_CTRL_SET_CTL) != 0U)
        {
            /*ADC mode and set Outer Range Alert*/
            (void)SDL_VTM_tsSetCtrl (p_cfg2, instance, &tsCtrl_cfg);
        }

        if((cfgTsCtrl & (SDL_VTM_configTsCtrl)SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT) != 0U)
        {
            /*thermal shutdown range*/
            (void)SDL_VTM_tsSetMaxTOutRgAlertThr(p_cfg2, instance, high_temp_in_mdc, low_temp_in_mdc);
        }
        if(((cfgTsCtrl & (SDL_VTM_configTsCtrl)SDL_VTM_VD_CONFIG_CTRL_SET_THR) != 0U))
        {
            /*Threshold and Interrupt*/
            (void)SDL_VTM_tsSetThresholds (p_cfg1, instance, &thr_val);
        }
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1165,PROC_SDL-1303,PROC_SDL-1304
 */
int32_t SDL_VTM_getTemp(SDL_VTM_InstTs instance, uint32_t *pTempVal)
{
    const SDL_VTM_cfg1Regs      *p_cfg1;
	uint32_t baseAddr;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;

    const SDL_VTM_cfg1Regs_TMPSENS  *p_sensor;
    SDL_VTM_adc_code           	adc_code;
    int32_t                     p_milli_degree_temp_val;
    int32_t sdlResult     =     SDL_PASS;
    p_sensor              =     &p_cfg1->TMPSENS[instance];

    if( (pTempVal   == NULL_PTR)        ||
        ((uint32_t)instance  >= SDL_VTM_TS_MAX_NUM))
    {
        sdlResult = SDL_EBADARGS;
    }
    if(sdlResult == SDL_PASS)
    {
        adc_code = SDL_VTM_getAdcCode(p_sensor);
        sdlResult = SDL_VTM_tsConvADCToTemp (adc_code, instance, &p_milli_degree_temp_val);
        *pTempVal = (uint32_t) p_milli_degree_temp_val;
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1164,PROC_SDL-1305,PROC_SDL-1306
 */
int32_t SDL_VTM_getSensorStatus(SDL_VTM_InstTs instance, const SDL_VTM_Stat_read_ctrl *pCtrl, \
                                SDL_VTM_Stat_val *pStat_val)
{
    const SDL_VTM_cfg1Regs               *p_cfg1;
	uint32_t baseAddr;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;

    SDL_VTM_Stat_read_ctrl                    ctrl;
    const SDL_VTM_cfg1Regs_TMPSENS          *p_sensor;
    int32_t                                 sdlResult = SDL_EFAIL;

    /* argument checks */
    if( (pCtrl       == NULL_PTR)        ||
        (pStat_val   == NULL_PTR)        ||
        ((uint32_t)instance  >= SDL_VTM_TS_MAX_NUM))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        p_sensor = &p_cfg1->TMPSENS[instance];
        ctrl     = *pCtrl;
        if ((ctrl & SDL_VTM_TS_READ_VD_MAP_VAL) != 0u)
        {
            pStat_val->vd_map = (SDL_VTM_ts_stat_vd_map) \
                                    SDL_REG32_FEXT(&p_sensor->STAT, \
                                                   VTM_CFG1_TMPSENS_STAT_VD_MAP);
			sdlResult = SDL_PASS;
        }

        if ((ctrl & SDL_VTM_TS_READ_ALL_THRESHOLD_ALERTS) != 0u)
        {
            pStat_val->lt_th0_alert = (uint8_t) SDL_REG32_FEXT(&p_sensor->STAT, \
                VTM_CFG1_TMPSENS_STAT_LT_TH0_ALERT);
            pStat_val->gt_th1_alert = (uint8_t) SDL_REG32_FEXT(&p_sensor->STAT, \
                VTM_CFG1_TMPSENS_STAT_GT_TH1_ALERT);
            pStat_val->gt_th2_alert = (uint8_t) SDL_REG32_FEXT(&p_sensor->STAT, \
                VTM_CFG1_TMPSENS_STAT_GT_TH2_ALERT);
            pStat_val->maxt_outrg_alert = (uint8_t) SDL_REG32_FEXT(&p_sensor->STAT, \
                VTM_CFG1_TMPSENS_STAT_MAXT_OUTRG_ALERT);
			sdlResult = SDL_PASS;
		}

        if ((ctrl & SDL_VTM_TS_READ_FIRST_TIME_EOC_BIT) != 0u)
        {
            pStat_val->soc_fc_update = (uint8_t) SDL_REG32_FEXT(&p_sensor->STAT, \
                VTM_CFG1_TMPSENS_STAT_EOC_FC_UPDATE);
			sdlResult = SDL_PASS;
		}

        if ((ctrl & SDL_VTM_TS_READ_DATA_VALID_BIT) != 0u)
        {
            pStat_val->data_valid = (uint8_t) SDL_REG32_FEXT(&p_sensor->STAT, \
                VTM_CFG1_TMPSENS_STAT_DATA_VALID);
			sdlResult = SDL_PASS;
		}

        if ((ctrl & SDL_VTM_TS_READ_DATA_OUT_VAL) != 0u)
        {
            pStat_val->data_out = SDL_VTM_getAdcCode(p_sensor);
			sdlResult = SDL_PASS;
		}
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1164,PROC_SDL-1307,PROC_SDL-1308
 */
int32_t SDL_VTM_intrCntrl(SDL_VTM_InstVd instance, SDL_VTM_intrCtrl intrCtrl)
{
    const SDL_VTM_cfg1Regs               *p_cfg1;
	uint32_t baseAddr;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;

    int32_t    sdlResult = SDL_PASS;
    uint32_t   vd;

    /* argument checks */
    if((int32_t)instance  >= gNumCoreVoltageDomains)
    {
        sdlResult = SDL_EBADARGS;
    }

    /* Arg check for the control, it can't have orthogonal settings */
    if ((sdlResult                                   == SDL_PASS) &&
        ((intrCtrl & SDL_VTM_VD_LT_THR0_INTR_RAW_SET) == SDL_VTM_VD_LT_THR0_INTR_RAW_SET) &&
        ((intrCtrl & SDL_VTM_VD_LT_THR0_INTR_RAW_CLR) == SDL_VTM_VD_LT_THR0_INTR_RAW_CLR))
    {
        sdlResult = SDL_EBADARGS;
    }

    /* Arg check for the control, it can't have orthogonal settings */
    if ((sdlResult                                   == SDL_PASS) &&
        ((intrCtrl & SDL_VTM_VD_GT_THR1_INTR_RAW_SET) == SDL_VTM_VD_GT_THR1_INTR_RAW_SET) &&
        ((intrCtrl & SDL_VTM_VD_GT_THR1_INTR_RAW_CLR) == SDL_VTM_VD_GT_THR1_INTR_RAW_CLR))
    {
        sdlResult = SDL_EBADARGS;
    }

    /* Arg check for the control, it can't have orthogonal settings */
    if ((sdlResult                                   == SDL_PASS) &&
        ((intrCtrl & SDL_VTM_VD_GT_THR2_INTR_RAW_SET) == SDL_VTM_VD_GT_THR2_INTR_RAW_SET) &&
        ((intrCtrl & SDL_VTM_VD_GT_THR2_INTR_RAW_CLR) == SDL_VTM_VD_GT_THR2_INTR_RAW_CLR))
    {
        sdlResult = SDL_EBADARGS;
    }

    /* Arg check for the control, it can't have orthogonal settings */
    if ((sdlResult                                  == SDL_PASS) &&
        ((intrCtrl & SDL_VTM_VD_LT_THR0_INTR_EN_SET) == SDL_VTM_VD_LT_THR0_INTR_EN_SET) &&
        ((intrCtrl & SDL_VTM_VD_LT_THR0_INTR_EN_CLR) == SDL_VTM_VD_LT_THR0_INTR_EN_CLR))
    {
        sdlResult = SDL_EBADARGS;
    }

    /* Arg check for the control, it can't have orthogonal settings */
    if ((sdlResult                                  == SDL_PASS) &&
        ((intrCtrl & SDL_VTM_VD_GT_THR1_INTR_EN_SET) == SDL_VTM_VD_GT_THR1_INTR_EN_SET) &&
        ((intrCtrl & SDL_VTM_VD_GT_THR1_INTR_EN_CLR) == SDL_VTM_VD_GT_THR1_INTR_EN_CLR))
    {
        sdlResult = SDL_EBADARGS;
    }

    /* Arg check for the control, it can't have orthogonal settings */
    if ((sdlResult                                  == SDL_PASS) &&
        ((intrCtrl & SDL_VTM_VD_GT_THR2_INTR_EN_SET) == SDL_VTM_VD_GT_THR2_INTR_EN_SET) &&
        ((intrCtrl & SDL_VTM_VD_GT_THR2_INTR_EN_CLR) == SDL_VTM_VD_GT_THR2_INTR_EN_CLR))
    {
        sdlResult = SDL_EBADARGS;
    }

    if (sdlResult == SDL_PASS)
    {
        vd = (uint32_t)((uint32_t)1u << (uint32_t)instance);
        if ((intrCtrl & SDL_VTM_VD_LT_THR0_INTR_RAW_SET) == SDL_VTM_VD_LT_THR0_INTR_RAW_SET)
        {
            SDL_REG32_FINS(&p_cfg1->LT_TH0_INT_RAW_STAT_SET,     \
               VTM_CFG1_LT_TH0_INT_RAW_STAT_SET_INT_VD, vd);
        }
        if ((intrCtrl & SDL_VTM_VD_GT_THR1_INTR_RAW_SET) == SDL_VTM_VD_GT_THR1_INTR_RAW_SET)
        {
            SDL_REG32_FINS(&p_cfg1->GT_TH1_INT_RAW_STAT_SET,     \
               VTM_CFG1_GT_TH1_INT_RAW_STAT_SET_INT_VD, vd);
        }

        if ((intrCtrl & SDL_VTM_VD_GT_THR2_INTR_RAW_SET) == SDL_VTM_VD_GT_THR2_INTR_RAW_SET)
        {
            SDL_REG32_FINS(&p_cfg1->GT_TH2_INT_RAW_STAT_SET,     \
               VTM_CFG1_GT_TH2_INT_RAW_STAT_SET_INT_VD, vd);
        }

        if ((intrCtrl & SDL_VTM_VD_LT_THR0_INTR_RAW_CLR) == SDL_VTM_VD_LT_THR0_INTR_RAW_CLR)
        {
            SDL_REG32_FINS(&p_cfg1->LT_TH0_INT_EN_STAT_CLR,     \
               VTM_CFG1_LT_TH0_INT_EN_STAT_CLR_INT_VD, vd);
        }
        if ((intrCtrl & SDL_VTM_VD_GT_THR1_INTR_RAW_CLR) == SDL_VTM_VD_GT_THR1_INTR_RAW_CLR)
        {
            SDL_REG32_FINS(&p_cfg1->GT_TH1_INT_EN_STAT_CLR,     \
               VTM_CFG1_GT_TH1_INT_EN_STAT_CLR_INT_VD, vd);
        }

        if ((intrCtrl & SDL_VTM_VD_GT_THR2_INTR_RAW_CLR) == SDL_VTM_VD_GT_THR2_INTR_RAW_CLR)
        {
            SDL_REG32_FINS(&p_cfg1->GT_TH2_INT_EN_STAT_CLR,     \
               VTM_CFG1_GT_TH2_INT_EN_STAT_CLR_INT_VD, vd);
        }

        if ((intrCtrl & SDL_VTM_VD_LT_THR0_INTR_EN_SET) == SDL_VTM_VD_LT_THR0_INTR_EN_SET)
        {
            SDL_REG32_FINS(&p_cfg1->LT_TH0_INT_EN_SET,     \
               VTM_CFG1_LT_TH0_INT_EN_SET_INT_VD, vd);
        }
        if ((intrCtrl & SDL_VTM_VD_GT_THR1_INTR_EN_SET) == SDL_VTM_VD_GT_THR1_INTR_EN_SET)
        {
            SDL_REG32_FINS(&p_cfg1->GT_TH1_INT_EN_SET,     \
               VTM_CFG1_GT_TH1_INT_EN_SET_INT_VD, vd);
        }

        if ((intrCtrl & SDL_VTM_VD_GT_THR2_INTR_EN_SET) == SDL_VTM_VD_GT_THR2_INTR_EN_SET)
        {
            SDL_REG32_FINS(&p_cfg1->GT_TH2_INT_EN_SET,     \
               VTM_CFG1_GT_TH2_INT_EN_SET_INT_VD, vd);
        }

        if ((intrCtrl & SDL_VTM_VD_LT_THR0_INTR_EN_CLR) == SDL_VTM_VD_LT_THR0_INTR_EN_CLR)
        {
            SDL_REG32_FINS(&p_cfg1->LT_TH0_INT_EN_CLR,     \
               VTM_CFG1_LT_TH0_INT_EN_CLR_INT_VD, vd);
        }
        if ((intrCtrl & SDL_VTM_VD_GT_THR1_INTR_EN_CLR) == SDL_VTM_VD_GT_THR1_INTR_EN_CLR)
        {
            SDL_REG32_FINS(&p_cfg1->GT_TH1_INT_EN_CLR,     \
               VTM_CFG1_GT_TH1_INT_EN_CLR_INT_VD, vd);
        }

        if ((intrCtrl & SDL_VTM_VD_GT_THR2_INTR_EN_CLR) == SDL_VTM_VD_GT_THR2_INTR_EN_CLR)
        {
            SDL_REG32_FINS(&p_cfg1->GT_TH2_INT_EN_CLR,     \
               VTM_CFG1_GT_TH2_INT_EN_CLR_INT_VD, vd);
        }
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1175,PROC_SDL-1309
 */
int32_t SDL_VTM_getStaticRegistersVd(SDL_VTM_InstVd instance, SDL_VTM_staticRegsVd *pStaticRegs)
{
    const SDL_VTM_cfg1Regs               *p_cfg1;
    const SDL_VTM_cfg2Regs               *p_cfg2;
	uint32_t baseAddr,baseAddr1;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_2, &baseAddr1);
    p_cfg2 = (SDL_VTM_cfg2Regs *) baseAddr1;

    int32_t               sdlResult = SDL_EBADARGS;

    /* arg checked */
    if (((int32_t)instance < gNumCoreVoltageDomains) &&
         (pStaticRegs  != NULL_PTR))
    {
        sdlResult = SDL_PASS;
    }

    /* if good args are passed */
    if(sdlResult == SDL_PASS)
    {
        /* Read all elements */
        pStaticRegs->vtm_global_cfg.validMap =  \
                                 (SDL_VTM_TSGLOBAL_CLK_SEL_VALID                 |
                                  SDL_VTM_TSGLOBAL_CLK_DIV_VALID                 |
                                  SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_EN_VALID |
                                  SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR0_VALID   |
                                  SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR_VALID    |
                                  SDL_VTM_TSGLOBAL_SAMPLES_PER_CNT_VALID );

        /* did not check p_cfg2 arg as it would be checked in below call */
        sdlResult = SDL_VTM_tsGetGlobalCfg(p_cfg2, &pStaticRegs->vtm_global_cfg);
    }

    /* proceed if previous operation is good */
    if (sdlResult == SDL_PASS)
    {
        /* Read the voltage domain event selection and opp vid values */
        const SDL_VTM_cfg1Regs_VD * vtmVDRegs = &(p_cfg1->VD[instance]);
        pStaticRegs->vtm_vd_evt_sel_ctrl = SDL_REG32_RD(&vtmVDRegs->EVT_SEL_SET);
        pStaticRegs->vtm_vd_opp_vid      = SDL_REG32_RD(&vtmVDRegs->OPPVID);
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1175,PROC_SDL-1310
 */
int32_t SDL_VTM_getStaticRegistersTs(SDL_VTM_InstTs instance, SDL_VTM_staticRegsTs *pStaticRegs)
{
    const SDL_VTM_cfg1Regs               *p_cfg1;
    const SDL_VTM_cfg2Regs               *p_cfg2;
	uint32_t baseAddr,baseAddr1;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_2, &baseAddr1);
    p_cfg2 = (SDL_VTM_cfg2Regs *) baseAddr1;
    int32_t               sdlResult = SDL_EBADARGS;

    /* arg checked */
    if (((uint32_t)instance < SDL_VTM_TS_MAX_NUM) && (pStaticRegs  != NULL_PTR))
    {
        sdlResult = SDL_PASS;
    }

    /* if good args are passed */
    if(sdlResult == SDL_PASS)
    {
        const SDL_VTM_cfg1Regs_TMPSENS *pSensor = &(p_cfg1->TMPSENS[instance]);
        const SDL_VTM_cfg2Regs_TMPSENS *pSensor2 = &(p_cfg2->TMPSENS[instance]);
        /* Read the temperature sensor control values */
        pStaticRegs->vtm_ts_ctrl2 = SDL_REG32_RD(&pSensor2->CTRL);
        pStaticRegs->vtm_ts_ctrl  = SDL_REG32_RD(&pSensor->CTRL);
        pStaticRegs->vtm_ts_th    = SDL_REG32_RD(&pSensor->TH);
        pStaticRegs->vtm_ts_th2   = SDL_REG32_RD(&pSensor->TH2);
    }
    return (sdlResult);

}

 /**
 * Design: PROC_SDL-1183,PROC_SDL-1311
 */
int32_t SDL_VTM_verifyConfigVd(SDL_VTM_InstVd instance, const SDL_VTM_configVd *pConfig)
{
    uint8_t                 			vid_opp_val;
    uint32_t                			vd_temp_evts;
    const SDL_VTM_cfg1Regs               *p_cfg1;
    const SDL_VTM_cfg2Regs               *p_cfg2;
	uint32_t baseAddr,baseAddr1;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_2, &baseAddr1);
    p_cfg2 = (SDL_VTM_cfg2Regs *) baseAddr1;
    SDL_VTM_tsGlobal_cfg               tsGlobal_cfg;
    SDL_VTM_tsGlobal_cfg               cfg_tsGlobal_cfg;
    SDL_VTM_tsGlobal_ctrl_valid_map validMap;
    SDL_VTM_configVdCtrl               cfgVdCtrl;
    int32_t    sdlResult =     SDL_PASS;


    /* argument checks */
    if(((int32_t)instance  >= gNumCoreVoltageDomains)  ||
       (pConfig       == NULL_PTR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        cfgVdCtrl = pConfig->configVdCtrl;
    }

    if((sdlResult == SDL_PASS) &&
       ((cfgVdCtrl & (SDL_VTM_configVdCtrl)SDL_VTM_VD_CONFIG_CTRL_VID_OPP) != 0U))
    {
        (void)SDL_VTM_vdGetOppVid (p_cfg1, instance, pConfig->vid_opp, &vid_opp_val);
        if(vid_opp_val != pConfig->vid_opp_val)
        {
            sdlResult = SDL_EFAIL;
        }
    }

    if((sdlResult == SDL_PASS) &&
       ((cfgVdCtrl & (SDL_VTM_configVdCtrl)SDL_VTM_VD_CONFIG_CTRL_EVT_SEL) != 0U))
    {
        const SDL_VTM_cfg1Regs_VD * vtmVDRegs = &(p_cfg1->VD[instance]);
        vd_temp_evts = SDL_REG32_FEXT(&vtmVDRegs->EVT_SEL_SET, \
                                                            VTM_CFG1_EVT_SET_TSENS_EVT_SEL);
        if(vd_temp_evts != (uint32_t)(pConfig->vd_temp_evts))
        {
            sdlResult = SDL_EFAIL;
        }
    }
    if((sdlResult == SDL_PASS) &&
       ((cfgVdCtrl & (SDL_VTM_configVdCtrl)SDL_VTM_VD_CONFIG_CTRL_GLB_CFG) != 0U))
    {
        cfg_tsGlobal_cfg = pConfig->tsGlobal_cfg;
        tsGlobal_cfg.validMap = cfg_tsGlobal_cfg.validMap;

        tsGlobal_cfg.clkSel = 0x0;
        tsGlobal_cfg.clkDiv = 0xFF;
        tsGlobal_cfg.any_maxt_outrg_alert_en = 0x0;
        tsGlobal_cfg.maxt_outrg_alert_thr0 = 0x0;
        tsGlobal_cfg.maxt_outrg_alert_thr = 0x0;
        tsGlobal_cfg.samplesPerCnt = 0x0;
        (void)SDL_VTM_tsGetGlobalCfg (p_cfg2, &tsGlobal_cfg);
            validMap = tsGlobal_cfg.validMap;

            if(((validMap & SDL_VTM_TSGLOBAL_CLK_SEL_VALID) != 0U) &&
               (tsGlobal_cfg.clkSel != cfg_tsGlobal_cfg.clkSel))
            {
                sdlResult = SDL_EFAIL;
            }
            if(((validMap & SDL_VTM_TSGLOBAL_CLK_DIV_VALID) != 0U) &&
               (tsGlobal_cfg.clkDiv != cfg_tsGlobal_cfg.clkDiv) &&
               (sdlResult == SDL_PASS))
            {
                sdlResult = SDL_EFAIL;
            }
            if(((validMap & SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_EN_VALID) != 0U) &&
               (tsGlobal_cfg.any_maxt_outrg_alert_en != cfg_tsGlobal_cfg.any_maxt_outrg_alert_en) &&
               (sdlResult == SDL_PASS))
            {
                sdlResult = SDL_EFAIL;
            }
            if(((validMap & SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR0_VALID) != 0U) &&
               (tsGlobal_cfg.maxt_outrg_alert_thr0 != cfg_tsGlobal_cfg.maxt_outrg_alert_thr0) &&
               (sdlResult == SDL_PASS))
            {
                sdlResult = SDL_EFAIL;
            }
            if(((validMap & SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR_VALID) != 0U) &&
               (tsGlobal_cfg.maxt_outrg_alert_thr != cfg_tsGlobal_cfg.maxt_outrg_alert_thr) &&
               (sdlResult == SDL_PASS))
            {
                sdlResult = SDL_EFAIL;
            }
            if(((validMap & SDL_VTM_TSGLOBAL_SAMPLES_PER_CNT_VALID) != 0U) &&
               (tsGlobal_cfg.samplesPerCnt != cfg_tsGlobal_cfg.samplesPerCnt) &&
               (sdlResult == SDL_PASS))
            {
                sdlResult = SDL_EFAIL;
            }
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1183,PROC_SDL-1312
 */
int32_t SDL_VTM_verifyConfigTs(SDL_VTM_InstTs instance, const SDL_VTM_configTs *pConfig)
{
    int32_t    sdlResult = SDL_PASS;
    SDL_VTM_Ctrlcfg           tsCtrl_cfg;
    SDL_VTM_Ctrlcfg           cfg_tsCtrl_cfg;
    SDL_VTM_tsThrVal               thr_val;
    SDL_VTM_tsThrVal               cfg_thr_val;
    const SDL_VTM_cfg1Regs               *p_cfg1;
    const SDL_VTM_cfg2Regs               *p_cfg2;
	uint32_t baseAddr,baseAddr1;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_2, &baseAddr1);
    p_cfg2 = (SDL_VTM_cfg2Regs *) baseAddr1;
    uint32_t                adc_code_h, adc_code_l;
    SDL_VTM_adc_code        config_adc_h, config_adc_l;
    SDL_VTM_configTsCtrl    configTsCtrl;

    /* argument checks */
    if(((uint32_t)instance  >= SDL_VTM_TS_MAX_NUM)  ||
       (pConfig       == NULL_PTR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        configTsCtrl = pConfig->configTsCtrl;
    }

    if((sdlResult == SDL_PASS) &&
       ((configTsCtrl & (SDL_VTM_configTsCtrl)SDL_VTM_VD_CONFIG_CTRL_SET_CTL) != 0U))
    {
        cfg_tsCtrl_cfg = pConfig->tsCtrl_cfg;
        tsCtrl_cfg.valid_map = cfg_tsCtrl_cfg.valid_map;
        tsCtrl_cfg.maxt_outrg_alert_en = 0x0;
        tsCtrl_cfg.tsReset = 0x0;
        tsCtrl_cfg.adc_stat = 0x0;
        tsCtrl_cfg.mode = 0x0;
        (void)SDL_VTM_tsGetCtrl (p_cfg2, instance, &tsCtrl_cfg);


        if(((cfg_tsCtrl_cfg.valid_map & \
                (SDL_VTM_tsCtrl_valid_map)SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID) != 0U) &&    \
            (tsCtrl_cfg.maxt_outrg_alert_en     !=  cfg_tsCtrl_cfg.maxt_outrg_alert_en))
        {
            sdlResult = SDL_EFAIL;
        }

        if(((cfg_tsCtrl_cfg.valid_map & (SDL_VTM_tsCtrl_valid_map)SDL_VTM_TS_CTRL_RESET_CTRL_VALID) != 0U) &&
                (tsCtrl_cfg.tsReset     !=  cfg_tsCtrl_cfg.tsReset) &&
                (sdlResult == SDL_PASS))
        {
            sdlResult = SDL_EFAIL;
        }

        if(((cfg_tsCtrl_cfg.valid_map & (SDL_VTM_tsCtrl_valid_map)SDL_VTM_TS_CTRL_SOC_VALID) != 0U) &&
                (tsCtrl_cfg.adc_stat     !=  cfg_tsCtrl_cfg.adc_stat) &&
                (sdlResult == SDL_PASS))
        {
            sdlResult = SDL_EFAIL;
        }

        if(((cfg_tsCtrl_cfg.valid_map & (SDL_VTM_tsCtrl_valid_map)SDL_VTM_TS_CTRL_MODE_VALID) != 0U) &&
                (tsCtrl_cfg.mode     !=  cfg_tsCtrl_cfg.mode) &&
                (sdlResult == SDL_PASS))
        {
            sdlResult = SDL_EFAIL;
        }
    }

    if((sdlResult == SDL_PASS) && ((configTsCtrl & (SDL_VTM_configTsCtrl)SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT) != 0U))
    {
        adc_code_l = SDL_REG32_FEXT(&p_cfg2->MISC_CTRL2, VTM_CFG2_MISC_CTRL2_MAXT_OUTRG_ALERT_THR0);
        adc_code_h = SDL_REG32_FEXT(&p_cfg2->MISC_CTRL2, VTM_CFG2_MISC_CTRL2_MAXT_OUTRG_ALERT_THR);

        (void)SDL_VTM_tsConvTempToAdc(pConfig->low_temp_in_milli_degree_celsius, instance, &config_adc_l);
        (void)SDL_VTM_tsConvTempToAdc(pConfig->high_temp_in_milli_degree_celsius, instance, &config_adc_h);

        if((adc_code_l != (uint32_t)config_adc_l) ||
           (adc_code_h != (uint32_t)config_adc_h))
        {
            sdlResult = SDL_EFAIL;
        }
    }

    if((sdlResult == SDL_PASS) && ((configTsCtrl & (SDL_VTM_configTsCtrl)SDL_VTM_VD_CONFIG_CTRL_SET_THR) != 0U))
    {
        cfg_thr_val = pConfig->thr_val;
        thr_val.thrValidMap =     cfg_thr_val.thrValidMap;

	/* Set defaults for MISRA Compliance */
        thr_val.ltTh0 = (SDL_VTM_adc_code)(-1);
        thr_val.ltTh0En = FALSE;
        thr_val.gtTh1 = (SDL_VTM_adc_code)(-1);
        thr_val.gtTh1En = FALSE;
        thr_val.gtTh2 = (SDL_VTM_adc_code)(-1);
        thr_val.gtTh2En = FALSE;

        (void)SDL_VTM_tsGetThresholds (p_cfg1, instance, &thr_val);

        if(((cfg_thr_val.thrValidMap & (SDL_VTM_thr_valid_map)SDL_VTM_GT_TH1_VALID) != 0U) &&
                ((thr_val.gtTh1     !=  cfg_thr_val.gtTh1)       ||
                (thr_val.gtTh1En != cfg_thr_val.gtTh1En)))
        {
            sdlResult = SDL_EFAIL;
        }

        if(((cfg_thr_val.thrValidMap & (SDL_VTM_thr_valid_map)SDL_VTM_GT_TH2_VALID) != 0U) &&
                ((thr_val.gtTh2     !=  cfg_thr_val.gtTh2) ||
                 (thr_val.gtTh2En != cfg_thr_val.gtTh2En)) &&
                (sdlResult == SDL_PASS))
        {
            sdlResult = SDL_EFAIL;
        }

        if(((cfg_thr_val.thrValidMap & (SDL_VTM_thr_valid_map)SDL_VTM_LT_TH0_VALID) != 0U) &&
                ((thr_val.ltTh0     !=  cfg_thr_val.ltTh0) ||
                 (thr_val.ltTh0En != cfg_thr_val.ltTh0En)) &&
                (sdlResult == SDL_PASS))
        {
            sdlResult = SDL_EFAIL;
        }
    }
    return (sdlResult);
}

/* Nothing past this point */
