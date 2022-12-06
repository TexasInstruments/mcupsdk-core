/**
 * @file  sdl_ip_vtm.c
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
#include <sdl/vtm/v0/sdlr_vtm.h>
#include "sdl_ip_vtm.h"
#include <sdl/include/sdl_types.h>
/*=============================================================================
 *   functions
 *===========================================================================*/
/*=============================================================================
 *   macros
 *===========================================================================*/
#define SDL_VTM_VALUES_ARE_UNINITIALIZED    (-1)
/* Delay for Reg Reads */
#define SDL_VTM_DOUT_REG_READ_DELAY         (100)


/*=============================================================================
 *  global variables
 *===========================================================================*/
/* Uninitialized number of temperature sensors, will be initialized later */
int32_t gNumTempSensors        = SDL_VTM_VALUES_ARE_UNINITIALIZED;
/* Uninitialized number of core voltage domain, will be initialized later */
int32_t gNumCoreVoltageDomains = SDL_VTM_VALUES_ARE_UNINITIALIZED;

/*=============================================================================
 *  Internal functions
 *===========================================================================*/
/**
 * Design: PROC_SDL-1313,PROC_SDL-1314
 */
static SDL_VTM_adc_code SDL_VTM_abs(SDL_VTM_adc_code val);
static SDL_VTM_adc_code SDL_VTM_abs(SDL_VTM_adc_code val)
{
    SDL_VTM_adc_code sdlResult;
    if (val < 0)
    {
        sdlResult = -val;
    }
    else
    {
        sdlResult = val;
    }
    return (sdlResult);
}

/*=============================================================================
 *  SDL IP functions
 *===========================================================================*/

 /**
 * Design: PROC_SDL-1316,PROC_SDL-1315
 */
void SDL_VTM_getSensorVDCount(const SDL_VTM_cfg1Regs       *p_cfg1)
{
    gNumTempSensors         = (int32_t)SDL_REG32_FEXT(&p_cfg1->DEVINFO_PWR0, \
                                     VTM_CFG1_DEVINFO_PWR0_TMPSENS_CT);
    gNumCoreVoltageDomains  = (int32_t)SDL_REG32_FEXT(&p_cfg1->DEVINFO_PWR0, \
                                     VTM_CFG1_DEVINFO_PWR0_CVD_CT);
}

 /**
 * Design: PROC_SDL-1317,PROC_SDL-1318
 */
SDL_VTM_adc_code SDL_VTM_getBestValue(SDL_VTM_adc_code c0,
                                            SDL_VTM_adc_code c1,
                                            SDL_VTM_adc_code c2)
{
   SDL_VTM_adc_code d01 = SDL_VTM_abs(c0 - c1);
   SDL_VTM_adc_code d02 = SDL_VTM_abs(c0 - c2);
   SDL_VTM_adc_code d12 = SDL_VTM_abs(c1 - c2);
   SDL_VTM_adc_code result;

   /* if delta 01 is least, take 0 and 1 */
   if ((d01 <= d02) && (d01 <=d12))
   {
           result = (c0+c1)/2;
   }
   /* if delta 02 is least, take 0 and 2 */
   else if ((d02 <= d01) && (d02 <=d12))
   {
           result = (c0+c2)/2;
   }
   else
   {
       /* in all other cases, take 1 and 2 */
       result = (c1+c2)/2;
   }
   return (result);
}

 /**
 * Design: PROC_SDL-1319
 */
SDL_VTM_adc_code SDL_VTM_getAdcCode(const SDL_VTM_cfg1Regs_TMPSENS    *p_sensor)
{
    SDL_VTM_adc_code s0,s1,s2;
    volatile        int32_t  i;

    /* have some delay before read */
    for (i = 0; i < SDL_VTM_DOUT_REG_READ_DELAY;)
    {
        i = i + 1;
    }
    s0 = (SDL_VTM_adc_code)SDL_REG32_FEXT(&p_sensor->STAT, \
                VTM_CFG1_TMPSENS_STAT_DATA_OUT);

    /* have some delay before read */
    for (i = 0; i < SDL_VTM_DOUT_REG_READ_DELAY;)
    {
        i = i + 1;
    }
    s1 = (SDL_VTM_adc_code)SDL_REG32_FEXT(&p_sensor->STAT, \
                VTM_CFG1_TMPSENS_STAT_DATA_OUT);

    /* have some delay before read */
    for (i = 0; i < SDL_VTM_DOUT_REG_READ_DELAY;)
    {
        i = i + 1;
    }
    s2 = (SDL_VTM_adc_code)SDL_REG32_FEXT(&p_sensor->STAT, \
                VTM_CFG1_TMPSENS_STAT_DATA_OUT);

    /* Return the best of 3 values */
    return SDL_VTM_getBestValue(s0,s1,s2);
}

 /**
 * Design: PROC_SDL-1324,PROC_SDL-1325
 */
int32_t SDL_VTM_vdSetOppVid (const SDL_VTM_cfg1Regs  *p_cfg1,
                                    SDL_VTM_InstVd             instance,
                                    SDL_VTM_vid_opp         vid_opp,
                                    uint8_t                 vid_opp_val)
{
    int32_t  retVal = SDL_PASS;

    /* argument checks */
    if(((int32_t)instance  >= gNumCoreVoltageDomains)  ||

       (p_cfg1             == NULL_PTR))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        const SDL_VTM_cfg1Regs_VD * vtmVDRegs = &(p_cfg1->VD[instance]);

        switch (vid_opp)
        {
            case SDL_VTM_VID_OPP_0_CODE:
                SDL_REG32_FINS(&vtmVDRegs->OPPVID,              \
                               VTM_CFG1_VTM_VD_OPPVID_OPP_0,    \
                               vid_opp_val);
                break;
            case SDL_VTM_VID_OPP_1_CODE:
                SDL_REG32_FINS(&vtmVDRegs->OPPVID,              \
                               VTM_CFG1_VTM_VD_OPPVID_OPP_1,    \
                               vid_opp_val);
                break;
            case SDL_VTM_VID_OPP_2_CODE:
                SDL_REG32_FINS(&vtmVDRegs->OPPVID,              \
                               VTM_CFG1_VTM_VD_OPPVID_OPP_2,    \
                               vid_opp_val);
                break;
            case SDL_VTM_VID_OPP_3_CODE:
                SDL_REG32_FINS(&vtmVDRegs->OPPVID,              \
                               VTM_CFG1_VTM_VD_OPPVID_OPP_3,    \
                               vid_opp_val);
                break;
            default:
                retVal = SDL_EBADARGS;
                break;
        }

    }
    return (retVal);
}

 /**
 * Design: PROC_SDL-1326,PROC_SDL-1327
 */
int32_t SDL_VTM_vdGetOppVid (const SDL_VTM_cfg1Regs  *p_cfg1,
                            SDL_VTM_InstVd instance,
                            SDL_VTM_vid_opp         vid_opp,
                            uint8_t                 *p_vid_opp_val)

{
    int32_t     retVal = SDL_PASS;
    uint32_t    vid_opp_val = 0;

    /* argument checks */
    if(((int32_t)instance >= gNumCoreVoltageDomains)  ||

       (p_cfg1            == NULL_PTR)                ||
       (p_vid_opp_val     == NULL_PTR))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        const SDL_VTM_cfg1Regs_VD * vtmVDRegs = &(p_cfg1->VD[instance]);

        switch (vid_opp)
        {
            case SDL_VTM_VID_OPP_0_CODE:
                vid_opp_val = SDL_REG32_FEXT(&vtmVDRegs->OPPVID, \
                               VTM_CFG1_VTM_VD_OPPVID_OPP_0);
                break;
            case SDL_VTM_VID_OPP_1_CODE:
                vid_opp_val = SDL_REG32_FEXT(&vtmVDRegs->OPPVID, \
                               VTM_CFG1_VTM_VD_OPPVID_OPP_1);
                break;
            case SDL_VTM_VID_OPP_2_CODE:
                vid_opp_val = SDL_REG32_FEXT(&vtmVDRegs->OPPVID, \
                               VTM_CFG1_VTM_VD_OPPVID_OPP_2);
                break;
            case SDL_VTM_VID_OPP_3_CODE:
                vid_opp_val = SDL_REG32_FEXT(&vtmVDRegs->OPPVID, \
                               VTM_CFG1_VTM_VD_OPPVID_OPP_3);
                break;
            default:
                retVal = SDL_EBADARGS;
                break;
        }
        *p_vid_opp_val = (uint8_t) vid_opp_val;
    }
    return (retVal);
}

 /**
 * Design: PROC_SDL-1328,PROC_SDL-1329
 */
int32_t SDL_VTM_tsSetCtrl (const SDL_VTM_cfg2Regs          *p_cfg2,
                            SDL_VTM_InstTs                 instance,
                            const SDL_VTM_Ctrlcfg   *p_tsCtrl_cfg)

{
    int32_t sdlResult = SDL_PASS;
    SDL_VTM_tsCtrl_valid_map                valid_map;
    SDL_VTM_tsCtrl_max_outrg_alert          maxt_outrg_alert_en;
    SDL_VTM_tsCtrl_resetCtrl                tsReset;
    SDL_VTM_tsCtrl_singleshot_conv_stat     adc_trigger;
    SDL_VTM_tsCtrl_mode                     mode;
    const SDL_VTM_cfg2Regs_TMPSENS         *p_sensor;

    /* argument checks */
    if( (p_tsCtrl_cfg    == NULL_PTR)    ||
        (p_cfg2          == NULL_PTR)	 ||
        ((uint32_t)instance  >=  SDL_VTM_TS_MAX_NUM))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        valid_map = p_tsCtrl_cfg->valid_map;
        p_sensor  = &p_cfg2->TMPSENS[instance];

        /* get the maxt_outrg_en filed */
        if (p_tsCtrl_cfg->maxt_outrg_alert_en == SDL_VTM_TS_CTRL_MAXT_OUTRG_GEN_ALERT)
        {
            maxt_outrg_alert_en = SDL_VTM_TS_CTRL_MAXT_OUTRG_GEN_ALERT;
        }
        else
        {
            maxt_outrg_alert_en = SDL_VTM_TS_CTRL_MAXT_OUTRG_NO_ALERT;
        }

        /* Get the clrz filed */
        if (p_tsCtrl_cfg->tsReset == SDL_VTM_TS_CTRL_SENSOR_RESET)
        {
            tsReset   = SDL_VTM_TS_CTRL_SENSOR_RESET;
        }
        else
        {
            tsReset  = SDL_VTM_TS_CTRL_SENSOR_NORM_OP;
        }

        /* Get the start of conversion trigger field */
        if (p_tsCtrl_cfg->adc_stat == SDL_VTM_TS_CTRL_SINGLESHOT_ADC_CONV_IN_PROGRESS)
        {
            adc_trigger = SDL_VTM_TS_CTRL_SINGLESHOT_ADC_CONV_IN_PROGRESS;
        }
        else
        {
            adc_trigger = SDL_VTM_TS_CTRL_SINGLESHOT_ADC_CONV_COMPLETE;
        }

        /* Get the temperature sensor mode of operation */
        if (p_tsCtrl_cfg->mode == SDL_VTM_TS_CTRL_CONTINUOUS_MODE)
        {
            mode = SDL_VTM_TS_CTRL_CONTINUOUS_MODE;
        }
        else
        {
            mode = SDL_VTM_TS_CTRL_SINGLESHOT_MODE;
        }

        /* update the fileds that are valid */
        if ((valid_map & SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_sensor->CTRL, VTM_CFG2_TMPSENS_CTRL_MAXT_OUTRG_EN, maxt_outrg_alert_en);
        }

        if ((valid_map & SDL_VTM_TS_CTRL_RESET_CTRL_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_sensor->CTRL, VTM_CFG2_TMPSENS_CTRL_CLRZ, tsReset);
        }

        if ((valid_map & SDL_VTM_TS_CTRL_SOC_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_sensor->CTRL, VTM_CFG2_TMPSENS_CTRL_SOC, adc_trigger);
        }

        if ((valid_map & SDL_VTM_TS_CTRL_MODE_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_sensor->CTRL, VTM_CFG2_TMPSENS_CTRL_CONT, mode);
        }
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1330,PROC_SDL-1331
 */
int32_t SDL_VTM_tsGetCtrl (const SDL_VTM_cfg2Regs      *p_cfg2,
                            SDL_VTM_InstTs                 instance,
                          SDL_VTM_Ctrlcfg         *p_tsCtrl_cfg)

{
    int32_t sdlResult = SDL_PASS;
    SDL_VTM_tsCtrl_valid_map           valid_map;
    const SDL_VTM_cfg2Regs_TMPSENS     *p_sensor;

    /* argument checks */
    if( (p_tsCtrl_cfg    == NULL_PTR)   ||
        (p_cfg2          == NULL_PTR)   ||
        ((uint32_t)instance  >=  SDL_VTM_TS_MAX_NUM))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        valid_map = p_tsCtrl_cfg->valid_map;
        p_sensor  = &p_cfg2->TMPSENS[instance];

        /* Get the fileds that are valid */
        if ((valid_map & SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID) !=0u)
        {
            p_tsCtrl_cfg->maxt_outrg_alert_en = (SDL_VTM_tsCtrl_max_outrg_alert) \
                    SDL_REG32_FEXT(&p_sensor->CTRL, VTM_CFG2_TMPSENS_CTRL_MAXT_OUTRG_EN);
        }

        if ((valid_map & SDL_VTM_TS_CTRL_RESET_CTRL_VALID) !=0u)
        {
             p_tsCtrl_cfg->tsReset = (SDL_VTM_tsCtrl_resetCtrl) \
                    SDL_REG32_FEXT(&p_sensor->CTRL, VTM_CFG2_TMPSENS_CTRL_CLRZ);
        }

        if ((valid_map & SDL_VTM_TS_CTRL_SOC_VALID) !=0u)
        {
             p_tsCtrl_cfg->adc_stat = (SDL_VTM_tsCtrl_singleshot_conv_stat) \
                    SDL_REG32_FEXT(&p_sensor->CTRL, VTM_CFG2_TMPSENS_CTRL_SOC);
        }

        if ((valid_map & SDL_VTM_TS_CTRL_MODE_VALID) !=0u)
        {
             p_tsCtrl_cfg->mode = (SDL_VTM_tsCtrl_mode) \
                    SDL_REG32_FEXT(&p_sensor->CTRL, VTM_CFG2_TMPSENS_CTRL_CONT);
        }
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1164,PROC_SDL-1332,PROC_SDL-1333
 */
int32_t SDL_VTM_tsSetThresholds (const SDL_VTM_cfg1Regs        *p_cfg1,
                                SDL_VTM_InstTs                 instance,
                                const SDL_VTM_tsThrVal       *p_thr_val)
{
    int32_t                             sdlResult = SDL_PASS;
    uint32_t                            enable;
    const SDL_VTM_cfg1Regs_TMPSENS      *pVtmTSRegs;

    /* argument checks */
    if(((uint32_t)instance  >= SDL_VTM_TS_MAX_NUM)      || \
       (p_thr_val       == NULL_PTR)                 || \
       (p_cfg1           == NULL_PTR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        pVtmTSRegs = &p_cfg1->TMPSENS[instance];

        if ((p_thr_val->thrValidMap & SDL_VTM_LT_TH0_VALID) != 0u)
        {
            SDL_REG32_FINS(&pVtmTSRegs->TH, VTM_CFG1_TMPSENS_TH_TH0_VAL, p_thr_val->ltTh0);
            if (p_thr_val->ltTh0En == FALSE)
            {
                enable = 0u;
            }
            else
            {
                enable = 1u;
            }
            SDL_REG32_FINS(&pVtmTSRegs->CTRL, VTM_CFG1_TMPSENS_CTRL_LT_TH0_EN, enable);
        }
        if ((p_thr_val->thrValidMap & SDL_VTM_GT_TH1_VALID) != 0u)
        {
            SDL_REG32_FINS(&pVtmTSRegs->TH, VTM_CFG1_TMPSENS_TH_TH1_VAL, p_thr_val->gtTh1);

            if (p_thr_val->gtTh1En == FALSE)
            {
                enable = 0u;
            }
            else
            {
                enable = 1u;
            }

            SDL_REG32_FINS(&pVtmTSRegs->CTRL, VTM_CFG1_TMPSENS_CTRL_GT_TH1_EN, enable);
        }
        if ((p_thr_val->thrValidMap & SDL_VTM_GT_TH2_VALID) != 0u)
        {
            SDL_REG32_FINS(&pVtmTSRegs->TH2, VTM_CFG1_TMPSENS_TH2_TH2_VAL, p_thr_val->gtTh2);

            if (p_thr_val->gtTh2En == FALSE)
            {
                enable = 0u;
            }
            else
            {
                enable = 1u;
            }

            SDL_REG32_FINS(&pVtmTSRegs->CTRL, VTM_CFG1_TMPSENS_CTRL_GT_TH2_EN, enable);
        }
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1164,PROC_SDL-1334,PROC_SDL-1335
 */
int32_t SDL_VTM_tsGetThresholds (const SDL_VTM_cfg1Regs   *p_cfg,
                                SDL_VTM_InstTs                 instance,
                                SDL_VTM_tsThrVal         *p_thr_val)
{
    int32_t sdlResult = SDL_PASS;
    const SDL_VTM_cfg1Regs_TMPSENS    *pVtmTSRegs;
    uint32_t reg_val;

    /* argument checks */
    if(((uint32_t)instance >= SDL_VTM_TS_MAX_NUM)   ||
        (p_thr_val      == NULL_PTR)             ||
        (p_cfg          == NULL_PTR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        pVtmTSRegs = &p_cfg->TMPSENS[instance];

        /* Set defaults for MISRA Compliance */
        p_thr_val->ltTh0 = (SDL_VTM_adc_code)(-1);
        p_thr_val->ltTh0En = FALSE;
        p_thr_val->gtTh1 = (SDL_VTM_adc_code)(-1);
        p_thr_val->gtTh1En = FALSE;
        p_thr_val->gtTh2 = (SDL_VTM_adc_code)(-1);
        p_thr_val->gtTh2En = FALSE;

        if ((p_thr_val->thrValidMap & SDL_VTM_LT_TH0_VALID) == SDL_VTM_LT_TH0_VALID)
        {
            p_thr_val->ltTh0   = (SDL_VTM_adc_code)SDL_REG32_FEXT(&pVtmTSRegs->TH, VTM_CFG1_TMPSENS_TH_TH0_VAL);
            reg_val = SDL_REG32_FEXT(&pVtmTSRegs->CTRL, VTM_CFG1_TMPSENS_CTRL_LT_TH0_EN);
            if (reg_val == 0x1u)
            {
                p_thr_val->ltTh0En = TRUE;
            }
            else
            {
                p_thr_val->ltTh0En = FALSE;
            }
        }
        if ((p_thr_val->thrValidMap & SDL_VTM_GT_TH1_VALID) == SDL_VTM_GT_TH1_VALID)
        {
            p_thr_val->gtTh1   = (SDL_VTM_adc_code) SDL_REG32_FEXT(&pVtmTSRegs->TH, VTM_CFG1_TMPSENS_TH_TH1_VAL);
            reg_val = SDL_REG32_FEXT(&pVtmTSRegs->CTRL, VTM_CFG1_TMPSENS_CTRL_GT_TH1_EN);
            if (reg_val == 0x1u)
            {
                p_thr_val->gtTh1En = TRUE;
            }
            else
            {
                p_thr_val->gtTh1En = FALSE;
            }
        }
        if ((p_thr_val->thrValidMap & SDL_VTM_GT_TH2_VALID) == SDL_VTM_GT_TH2_VALID)
        {
            p_thr_val->gtTh2   = (SDL_VTM_adc_code)SDL_REG32_FEXT(&pVtmTSRegs->TH2, VTM_CFG1_TMPSENS_TH2_TH2_VAL);
            reg_val = SDL_REG32_FEXT(&pVtmTSRegs->CTRL, VTM_CFG1_TMPSENS_CTRL_GT_TH2_EN);
            if (reg_val == 0x1u)
            {
                p_thr_val->gtTh2En = TRUE;
            }
            else
            {
                p_thr_val->gtTh2En = FALSE;
            }
        }
   }

   return (sdlResult);
}

 /**
 * Design: PROC_SDL-1336,PROC_SDL-1337
 */
int32_t SDL_VTM_vdEvtSelSet (const SDL_VTM_cfg1Regs  *p_cfg1,
                            SDL_VTM_InstVd instance,
                            SDL_VTM_vdEvtSel_set   vd_temp_evts)

{
    int32_t sdlRetVal = SDL_PASS;

    /* argument checks */
    if(((int32_t)instance >= gNumCoreVoltageDomains)      ||
       (p_cfg1           == NULL_PTR))
    {
        sdlRetVal = SDL_EBADARGS;
    }
    else
    {
        const SDL_VTM_cfg1Regs_VD * vtmVDRegs = &(p_cfg1->VD[instance]);
        SDL_REG32_FINS(&vtmVDRegs->EVT_SEL_SET,              \
               VTM_CFG1_EVT_SET_TSENS_EVT_SEL,               \
               vd_temp_evts);
    }

    return (sdlRetVal);
}

 /**
 * Design: PROC_SDL-1340,PROC_SDL-1341
 */
int32_t SDL_VTM_tsSetGlobalCfg (const SDL_VTM_cfg2Regs       *p_cfg2,
                               const SDL_VTM_tsGlobal_cfg   *p_tsGlobal_cfg)
{
    int32_t                             retVal = SDL_PASS;
    SDL_VTM_tsGlobal_ctrl_valid_map     valid_map;

    /* argument checks */
    if( (p_tsGlobal_cfg  == NULL_PTR)    ||
        (p_cfg2          ==  NULL_PTR))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        valid_map = p_tsGlobal_cfg->validMap;
        if ((valid_map & SDL_VTM_TSGLOBAL_CLK_SEL_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_cfg2->CLK_CTRL, VTM_CFG2_CLK_CTRL_TSENS_CLK_SEL, \
                p_tsGlobal_cfg->clkSel);
        }
        if ((valid_map & SDL_VTM_TSGLOBAL_CLK_DIV_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_cfg2->CLK_CTRL, VTM_CFG2_CLK_CTRL_TSENS_CLK_DIV, \
                p_tsGlobal_cfg->clkDiv);
        }

        if ((valid_map & SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_EN_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_cfg2->MISC_CTRL, VTM_CFG2_MISC_CTRL_ANY_MAXT_OUTRG_ALERT_EN, \
                p_tsGlobal_cfg->any_maxt_outrg_alert_en);
        }

        if ((valid_map & SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_cfg2->MISC_CTRL2, VTM_CFG2_MISC_CTRL2_MAXT_OUTRG_ALERT_THR, \
                p_tsGlobal_cfg->maxt_outrg_alert_thr);
        }

        if ((valid_map & SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR0_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_cfg2->MISC_CTRL2, VTM_CFG2_MISC_CTRL2_MAXT_OUTRG_ALERT_THR0, \
                p_tsGlobal_cfg->maxt_outrg_alert_thr0);
        }

        if ((valid_map & SDL_VTM_TSGLOBAL_SAMPLES_PER_CNT_VALID) !=0u)
        {
            SDL_REG32_FINS(&p_cfg2->SAMPLE_CTRL, VTM_CFG2_SAMPLE_CTRL_SAMPLE_PER_CNT, \
                p_tsGlobal_cfg->samplesPerCnt);
        }
    }

    return (retVal);
}

 /**
 * Design: PROC_SDL-1342,PROC_SDL-1343
 */
int32_t SDL_VTM_tsGetGlobalCfg (const SDL_VTM_cfg2Regs       *p_cfg2,
                               SDL_VTM_tsGlobal_cfg         *p_tsGlobal_cfg)

{
    int32_t sdlResult = SDL_PASS;
    SDL_VTM_tsGlobal_ctrl_valid_map valid_map;

    /* argument checks */
    if( (p_tsGlobal_cfg  == NULL_PTR)    ||
        (p_cfg2          == NULL_PTR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        valid_map = p_tsGlobal_cfg->validMap;
        if ((valid_map & SDL_VTM_TSGLOBAL_CLK_SEL_VALID) !=0u)
        {
            p_tsGlobal_cfg->clkSel = (SDL_VTM_tsGlobal_clkSel) \
                SDL_REG32_FEXT(&p_cfg2->CLK_CTRL, VTM_CFG2_CLK_CTRL_TSENS_CLK_SEL);
        }
        if ((valid_map & SDL_VTM_TSGLOBAL_CLK_DIV_VALID) !=0u)
        {
            p_tsGlobal_cfg->clkDiv = (SDL_VTM_tsGlobal_clkDiv) \
            SDL_REG32_FEXT(&p_cfg2->CLK_CTRL, VTM_CFG2_CLK_CTRL_TSENS_CLK_DIV);
        }

        if ((valid_map & SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_EN_VALID) !=0u)
        {
            p_tsGlobal_cfg->any_maxt_outrg_alert_en = (SDL_VTM_tsGlobal_any_maxt_outrg_alert_en) \
            SDL_REG32_FEXT(&p_cfg2->MISC_CTRL, VTM_CFG2_MISC_CTRL_ANY_MAXT_OUTRG_ALERT_EN);
        }

        if ((valid_map & SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR_VALID) !=0u)
        {
            p_tsGlobal_cfg->maxt_outrg_alert_thr = (SDL_VTM_adc_code) \
            SDL_REG32_FEXT(&p_cfg2->MISC_CTRL2, VTM_CFG2_MISC_CTRL2_MAXT_OUTRG_ALERT_THR);
        }

        if ((valid_map & SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR0_VALID) !=0u)
        {
            p_tsGlobal_cfg->maxt_outrg_alert_thr0 = (SDL_VTM_adc_code) \
            SDL_REG32_FEXT(&p_cfg2->MISC_CTRL2, VTM_CFG2_MISC_CTRL2_MAXT_OUTRG_ALERT_THR0);
        }

        if ((valid_map & SDL_VTM_TSGLOBAL_SAMPLES_PER_CNT_VALID) !=0u)
        {
            p_tsGlobal_cfg->samplesPerCnt = (SDL_VTM_tsGlobal_samples_per_count)\
            SDL_REG32_FEXT(&p_cfg2->SAMPLE_CTRL, VTM_CFG2_SAMPLE_CTRL_SAMPLE_PER_CNT);
        }
    }
    return (sdlResult);
}


/* Nothing past this point */
