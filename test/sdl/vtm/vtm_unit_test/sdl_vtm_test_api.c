/* Copyright (c) 2024 Texas Instruments Incorporated
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

 /**
 *  \file     vtm_test_api.c
 *
 *  \brief    This file contains vtm test code.
 *
 *  \details  VTM API tests
 **/

#include "sdl_vtm_test_main.h"
#define LT_THR0_DEFAULT          (95000)
#define GT_THR1_DEFAULT          (105000)
#define GT_THR2_DEFAULT          (115000)

#if defined (SOC_AM64X) || defined (SOC_AM243X)
extern int32_t gNumTempSensors;
extern int32_t gNumCoreVoltageDomains;
#endif

#if defined (SOC_AM263PX)
SDL_VTM_configTs SDL_VTM_configTempSense =
{
    1U,     /* TS0 Shut*/
    1U,     /* TS0 Alert*/
    72000,//58000,  /* TS0 Hot*/
    8000,//52000,  /* TS0 Cold*/
    58000,  /* TSA0 Alert Hot*/
    52000,  /* TS0 Alert Cold*/
    1U,
    1U,
    58000,
    52000,
    58000,
    52000
};
#endif

int32_t sdlVTM_apiTest(void)
{
#if defined (SOC_AM64X) || defined (SOC_AM243X)
    uint32_t                         tempVal;
    int32_t                            i, sdlResult;
    SDL_VTM_configTs                 tsConfig;
    SDL_VTM_configVd                 vdConfig;
    SDL_VTM_tsGlobal_cfg               tsGlobal_cfg;
    SDL_VTM_InstVd                     vdIns;
    int32_t                         testResult = SDL_APP_TEST_PASS;
    SDL_VTM_staticRegsVd             staticVdRegs;
    SDL_VTM_staticRegsTs             staticTsRegs;
    uint8_t                         vid_opp_val[4];
    SDL_VTM_Ctrlcfg                 ctrlCfg;
    SDL_VTM_tsThrVal                 thrVal;
    SDL_VTM_Stat_read_ctrl             pCtrl;
    SDL_VTM_Stat_val                 statVal;
    const SDL_VTM_cfg1Regs            *p_cfg1;
    const SDL_VTM_cfg2Regs             *p_cfg2;
    SDL_VTM_InstTs                      tsIns;
    vdConfig.configVdCtrl = (SDL_VTM_VD_CONFIG_CTRL_EVT_SEL        |    \
                                SDL_VTM_VD_CONFIG_CTRL_GLB_CFG);
    tsConfig.configTsCtrl = (SDL_VTM_VD_CONFIG_CTRL_SET_CTL     |    \
                             SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT    |    \
                             SDL_VTM_VD_CONFIG_CTRL_SET_THR);
    vdConfig.vd_temp_evts     = SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_0;
    tsGlobal_cfg.validMap = (SDL_VTM_TSGLOBAL_CLK_SEL_VALID |
                            SDL_VTM_TSGLOBAL_CLK_DIV_VALID |
                            SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_EN_VALID |
                            SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR0_VALID |
                            SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR_VALID |
                            SDL_VTM_TSGLOBAL_SAMPLES_PER_CNT_VALID);
    tsGlobal_cfg.clkSel     = SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_SEL_FIX_REF_CLK;
    tsGlobal_cfg.clkDiv = SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_1;
    tsGlobal_cfg.any_maxt_outrg_alert_en = SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_ENABLE;
    tsGlobal_cfg.maxt_outrg_alert_thr0 = (SDL_VTM_adc_code)186;
    tsGlobal_cfg.maxt_outrg_alert_thr  = (SDL_VTM_adc_code)716;
    tsGlobal_cfg.samplesPerCnt  = 1;
    vdConfig.tsGlobal_cfg     = tsGlobal_cfg;

    ctrlCfg.valid_map    =  (SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID  |
                            SDL_VTM_TS_CTRL_RESET_CTRL_VALID       |
                            SDL_VTM_TS_CTRL_SOC_VALID              |
                            SDL_VTM_TS_CTRL_MODE_VALID);
    ctrlCfg.maxt_outrg_alert_en        =    SDL_VTM_TS_CTRL_MAXT_OUTRG_GEN_ALERT;
    ctrlCfg.tsReset                    =    SDL_VTM_TS_CTRL_SENSOR_NORM_OP;
    ctrlCfg.adc_stat                =   SDL_VTM_TS_CTRL_SINGLESHOT_ADC_CONV_IN_PROGRESS;
    ctrlCfg.mode                    =    SDL_VTM_TS_CTRL_SINGLESHOT_MODE;
    tsConfig.tsCtrl_cfg                            =    ctrlCfg;
    tsConfig.high_temp_in_milli_degree_celsius    =    68000;
    tsConfig.low_temp_in_milli_degree_celsius    =    64000;

	uint32_t baseAddr,baseAddr1;

	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;

	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_2, &baseAddr1);
    p_cfg2 = (SDL_VTM_cfg2Regs *) baseAddr1;

	SDL_VTM_getBaseAddr(3, &baseAddr1);
    p_cfg2 = (SDL_VTM_cfg2Regs *) baseAddr1;

    thrVal.thrValidMap = (SDL_VTM_GT_TH1_VALID | \
                          SDL_VTM_GT_TH2_VALID | \
                          SDL_VTM_LT_TH0_VALID);
    thrVal.ltTh0En = TRUE;
    thrVal.gtTh1En = TRUE;
    thrVal.gtTh2En = TRUE;
    thrVal.ltTh0 = 300;
    thrVal.gtTh2 = 500;
    thrVal.gtTh1 = 400;

        pCtrl =     (SDL_VTM_TS_READ_VD_MAP_VAL            |    \
                 SDL_VTM_TS_READ_ALL_THRESHOLD_ALERTS    |    \
                 SDL_VTM_TS_READ_FIRST_TIME_EOC_BIT        |    \
                 SDL_VTM_TS_READ_DATA_VALID_BIT            |    \
                 SDL_VTM_TS_READ_DATA_OUT_VAL);
    tsConfig.thr_val    =    thrVal;

    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_initVd(vdIns, &vdConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
        for (vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_1; vdIns < gNumCoreVoltageDomains; vdIns++ )
        {
            sdlResult = SDL_VTM_initVd(vdIns, &vdConfig);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_initVd API test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

    if(testResult == 0)
    {
        SDL_VTM_vdGetOppVid (p_cfg1, SDL_VTM_INSTANCE_VD_DOMAIN_0, \
                                SDL_VTM_VID_OPP_1_CODE, &vid_opp_val[0]);
        vdConfig.vid_opp_val     = vid_opp_val[0];
        vdConfig.vid_opp         = SDL_VTM_VID_OPP_1_CODE;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_VID_OPP;
        sdlResult = SDL_VTM_initVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

	if(testResult == 0)
    {
        SDL_VTM_vdGetOppVid (p_cfg1, SDL_VTM_INSTANCE_VD_DOMAIN_0, \
                                SDL_VTM_VID_OPP_0_CODE, &vid_opp_val[0]);
        vdConfig.vid_opp_val     = vid_opp_val[0];
        vdConfig.vid_opp         = SDL_VTM_VID_OPP_0_CODE;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_VID_OPP;
        sdlResult = SDL_VTM_initVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

	if(testResult == 0)
    {
        SDL_VTM_vdGetOppVid (p_cfg1, SDL_VTM_INSTANCE_VD_DOMAIN_0, \
                                SDL_VTM_VID_OPP_2_CODE, &vid_opp_val[0]);
        vdConfig.vid_opp_val     = vid_opp_val[0];
        vdConfig.vid_opp         = SDL_VTM_VID_OPP_2_CODE;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_VID_OPP;
        sdlResult = SDL_VTM_initVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

	if(testResult == 0)
    {
        SDL_VTM_vdGetOppVid (p_cfg1, SDL_VTM_INSTANCE_VD_DOMAIN_0, \
                                SDL_VTM_VID_OPP_3_CODE , &vid_opp_val[0]);
        vdConfig.vid_opp_val     = vid_opp_val[0];
        vdConfig.vid_opp         = SDL_VTM_VID_OPP_3_CODE ;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_VID_OPP;
        sdlResult = SDL_VTM_initVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {

        vdConfig.configVdCtrl = (SDL_VTM_VD_CONFIG_CTRL_EVT_SEL |   \
                         SDL_VTM_VD_CONFIG_CTRL_GLB_CFG);
        for (vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0; vdIns < gNumCoreVoltageDomains; vdIns++ )
        {
            sdlResult = SDL_VTM_verifyConfigVd(vdIns, &vdConfig);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

    if(testResult == 0)
    {
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_VID_OPP;
        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        /* To get EFAIL, added actual value with 1  */
        vdConfig.vid_opp_val    = vid_opp_val[0]+1;
        vdConfig.vid_opp        = SDL_VTM_VID_OPP_1_CODE;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_VID_OPP;

        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {
        /* To get EFAIL, added actual value with 1  */
        vdConfig.vd_temp_evts   = SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_0+1;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_EVT_SEL;

        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        /* To get EFAIL, added actual value with 1  */
        tsGlobal_cfg.validMap = SDL_VTM_TSGLOBAL_CLK_SEL_VALID;
        tsGlobal_cfg.clkSel = SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_SEL_FIX_REF_CLK+1;
        vdConfig.tsGlobal_cfg   = tsGlobal_cfg;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_GLB_CFG;

        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        /* To get EFAIL, added actual value with 1  */
        tsGlobal_cfg.validMap = SDL_VTM_TSGLOBAL_CLK_DIV_VALID;
        tsGlobal_cfg.clkDiv = SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_1+1;
        vdConfig.tsGlobal_cfg   = tsGlobal_cfg;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_GLB_CFG;

        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {
        /* To get EFAIL, added actual value with 1  */
        tsGlobal_cfg.validMap = SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_EN_VALID;
        tsGlobal_cfg.any_maxt_outrg_alert_en = SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_DISABLE;
        vdConfig.tsGlobal_cfg   = tsGlobal_cfg;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_GLB_CFG;

        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        /* To get EFAIL, added actual value with 1  */
        tsGlobal_cfg.validMap = SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR0_VALID;
        tsGlobal_cfg.maxt_outrg_alert_thr0 = (SDL_VTM_adc_code)186+1;
        vdConfig.tsGlobal_cfg   = tsGlobal_cfg;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_GLB_CFG;

        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {
        /* To get EFAIL, added actual value with 1  */
        tsGlobal_cfg.validMap = SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR_VALID ;
        tsGlobal_cfg.maxt_outrg_alert_thr  = (SDL_VTM_adc_code)716+1;
        vdConfig.tsGlobal_cfg   = tsGlobal_cfg;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_GLB_CFG;

        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        /* To get EFAIL, added actual value with 1  */
        tsGlobal_cfg.validMap = SDL_VTM_TSGLOBAL_SAMPLES_PER_CNT_VALID;
        tsGlobal_cfg.samplesPerCnt  = 1+1;
        vdConfig.tsGlobal_cfg   = tsGlobal_cfg;
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_GLB_CFG;

        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        /* To get EFAIL, not actual bad Argument  */
        vdConfig.tsGlobal_cfg.clkSel   = 10;
        vdConfig.tsGlobal_cfg.validMap    =    (SDL_VTM_TSGLOBAL_CLK_SEL_VALID    |    \
                                    SDL_VTM_TSGLOBAL_CLK_DIV_VALID    |    \
                                    SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_EN_VALID    |    \
                                    SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR0_VALID    |    \
                                    SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR_VALID        |    \
                                    SDL_VTM_TSGLOBAL_SAMPLES_PER_CNT_VALID);
        vdConfig.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_GLB_CFG;
        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &vdConfig);
        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_getTemp((SDL_VTM_InstTs)i, &tempVal);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_getTemp API test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_initTs((SDL_VTM_InstTs)i, &tsConfig);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_initTs API test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

    if(testResult == 0)
    {
        tsConfig.configTsCtrl = (SDL_VTM_VD_CONFIG_CTRL_SET_CTL     |    \
                                SDL_VTM_VD_CONFIG_CTRL_SET_THR        |    \
                                SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT)        ;
        sdlResult = SDL_VTM_initTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
            tsConfig.configTsCtrl = 0;
        sdlResult = SDL_VTM_initTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        gNumTempSensors = (-1);
        sdlResult = SDL_VTM_initTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        ctrlCfg.valid_map        =  (SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID  |
                                    SDL_VTM_TS_CTRL_RESET_CTRL_VALID       |
                                    SDL_VTM_TS_CTRL_MODE_VALID);
        tsConfig.tsCtrl_cfg                         =   ctrlCfg;
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)i, &tsConfig);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

    if(testResult == 0)
    {
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        ctrlCfg.valid_map               =  (SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID    |    \
                                            SDL_VTM_TS_CTRL_RESET_CTRL_VALID);
        ctrlCfg.maxt_outrg_alert_en     =   SDL_VTM_TS_CTRL_MAXT_OUTRG_NO_ALERT;
        tsConfig.tsCtrl_cfg                         =   ctrlCfg;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {
        ctrlCfg.valid_map               =  SDL_VTM_TS_CTRL_RESET_CTRL_VALID;
        ctrlCfg.tsReset                 =   SDL_VTM_TS_CTRL_SENSOR_NORM_OP;
        tsConfig.tsCtrl_cfg                         =   ctrlCfg;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
        if(testResult == 0)
    {
        ctrlCfg.valid_map               =  (SDL_VTM_TS_CTRL_RESET_CTRL_VALID    |    \
                                            SDL_VTM_TS_CTRL_SOC_VALID);
        ctrlCfg.tsReset                 =   SDL_VTM_TS_CTRL_SENSOR_RESET;
        tsConfig.tsCtrl_cfg                         =   ctrlCfg;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

            if(testResult == 0)
    {
        ctrlCfg.valid_map               =  (SDL_VTM_TS_CTRL_SOC_VALID    |    \
                                            SDL_VTM_TS_CTRL_MODE_VALID);
        ctrlCfg.adc_stat                =   SDL_VTM_TS_CTRL_SINGLESHOT_ADC_CONV_IN_PROGRESS;
        tsConfig.tsCtrl_cfg                         =   ctrlCfg;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {
        ctrlCfg.valid_map               =  SDL_VTM_TS_CTRL_SOC_VALID;
        ctrlCfg.adc_stat                =   SDL_VTM_TS_CTRL_SINGLESHOT_ADC_CONV_COMPLETE;
        tsConfig.tsCtrl_cfg                         =   ctrlCfg;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

            if(testResult == 0)
    {
        ctrlCfg.valid_map               =  SDL_VTM_TS_CTRL_MODE_VALID;
        ctrlCfg.mode                    =   SDL_VTM_TS_CTRL_SINGLESHOT_MODE;
        tsConfig.tsCtrl_cfg                         =   ctrlCfg;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {
        ctrlCfg.valid_map               =  SDL_VTM_TS_CTRL_MODE_VALID;
        ctrlCfg.mode                    =   SDL_VTM_TS_CTRL_CONTINUOUS_MODE;
        tsConfig.tsCtrl_cfg                         =   ctrlCfg;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {

        tsConfig.high_temp_in_milli_degree_celsius  =   68000;
        tsConfig.low_temp_in_milli_degree_celsius   =   64000;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {

        tsConfig.high_temp_in_milli_degree_celsius  =   68000;
        tsConfig.low_temp_in_milli_degree_celsius   =   0;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {

        tsConfig.high_temp_in_milli_degree_celsius  =   0;
        tsConfig.low_temp_in_milli_degree_celsius   =   64000;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {

        tsConfig.high_temp_in_milli_degree_celsius  =   68000+5000;
        tsConfig.low_temp_in_milli_degree_celsius   =   64000;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {
       tsConfig.high_temp_in_milli_degree_celsius  =   68000;
        tsConfig.low_temp_in_milli_degree_celsius   =   64000-5000;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT;

        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_GT_TH1_VALID;
        thrVal.gtTh1En = TRUE;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_GT_TH2_VALID;
        thrVal.gtTh2En = TRUE;
        thrVal.gtTh2 = 500;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = (SDL_VTM_GT_TH1_VALID    |    SDL_VTM_GT_TH2_VALID);
        thrVal.gtTh2En = FALSE;
        thrVal.gtTh1En = FALSE;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = (SDL_VTM_GT_TH1_VALID    |    SDL_VTM_GT_TH2_VALID);
        thrVal.gtTh2En = TRUE;
        thrVal.gtTh2 = 500-100;
        thrVal.gtTh1En = FALSE;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_GT_TH2_VALID;
        thrVal.gtTh2En = TRUE;
        thrVal.gtTh2 = 500-100;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_GT_TH1_VALID;
        thrVal.gtTh1En = TRUE;
        thrVal.gtTh1 = 400-100;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }


    if(testResult == 0)
    {
        thrVal.thrValidMap = (SDL_VTM_GT_TH2_VALID    |    SDL_VTM_LT_TH0_VALID);
        thrVal.gtTh2En = FALSE;
        thrVal.ltTh0En = FALSE;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_GT_TH2_VALID;
        thrVal.gtTh2En = FALSE;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_GT_TH2_VALID;
        thrVal.gtTh2En = TRUE;
        thrVal.gtTh2 = 500-100;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_GT_TH2_VALID;
        thrVal.gtTh2En = FALSE;
        thrVal.gtTh2 = 500;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_GT_TH2_VALID;
        thrVal.gtTh2En = TRUE;
        thrVal.gtTh2 = 0;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_LT_TH0_VALID;
        thrVal.ltTh0En = TRUE;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_LT_TH0_VALID;
        thrVal.ltTh0En = FALSE;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = SDL_VTM_LT_TH0_VALID;
        thrVal.ltTh0En = TRUE;
        thrVal.ltTh0 = 300-100;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

        if(testResult == 0)
    {
        thrVal.thrValidMap = (SDL_VTM_LT_TH0_VALID    |    SDL_VTM_GT_TH2_VALID);
        thrVal.ltTh0En = TRUE;
        thrVal.gtTh2En = FALSE;
        thrVal.ltTh0 = 300-100;
        tsConfig.thr_val    =   thrVal;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        tsConfig.tsCtrl_cfg.maxt_outrg_alert_en     =   SDL_VTM_TS_CTRL_MAXT_OUTRG_NO_ALERT;
        tsConfig.tsCtrl_cfg.valid_map   =   (SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID    |    \
                                             SDL_VTM_TS_CTRL_RESET_CTRL_VALID        |    \
                                             SDL_VTM_TS_CTRL_SOC_VALID                |    \
                                             SDL_VTM_TS_CTRL_MODE_VALID);
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        tsConfig.tsCtrl_cfg.maxt_outrg_alert_en     =   SDL_VTM_TS_CTRL_MAXT_OUTRG_GEN_ALERT;
        tsConfig.tsCtrl_cfg.valid_map   =   SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        tsConfig.tsCtrl_cfg.maxt_outrg_alert_en     =   SDL_VTM_TS_CTRL_MAXT_OUTRG_GEN_ALERT;
        tsConfig.tsCtrl_cfg.valid_map   =   SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;
        sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_1, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        tsConfig.high_temp_in_milli_degree_celsius    =    86844;
        tsConfig.low_temp_in_milli_degree_celsius    =    64000;
        tsConfig.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT;
        sdlResult = SDL_VTM_initTs((SDL_VTM_InstTs)SDL_VTM_INSTANCE_TS_0, &tsConfig);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initTs API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &tempVal);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_getTemp API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_getSensorStatus((SDL_VTM_InstTs)i, &pCtrl, &statVal);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("SDL_VTM_getSensorStatus API test failed on line no. %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

        if (testResult == 0)
    {
        pCtrl = 0;
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_getSensorStatus((SDL_VTM_InstTs)i, &pCtrl, &statVal);

            if (sdlResult != SDL_EFAIL)
            {
                DebugP_log("SDL_VTM_getSensorStatus API test failed on line no. %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_LT_THR0_INTR_RAW_SET);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_GT_THR1_INTR_RAW_SET);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_GT_THR2_INTR_RAW_SET);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_LT_THR0_INTR_RAW_CLR);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }
    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_GT_THR1_INTR_RAW_CLR);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_GT_THR2_INTR_RAW_CLR);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }
    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_LT_THR0_INTR_EN_SET);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_GT_THR1_INTR_EN_SET);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }
    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_GT_THR2_INTR_EN_SET);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_LT_THR0_INTR_EN_CLR);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }
    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_GT_THR1_INTR_EN_CLR);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_intrCntrl((SDL_VTM_InstVd)i, SDL_VTM_VD_GT_THR2_INTR_EN_CLR);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_intrCntrl API test failed on line no: %d \n", __LINE__);
                testResult = -1;
            }
        }
    }


    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_getStaticRegistersVd((SDL_VTM_InstVd)i, &staticVdRegs);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_getStaticRegistersVd API test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_getStaticRegistersTs((SDL_VTM_InstTs)i, &staticTsRegs);

            if (sdlResult != SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_getStaticRegistersTs API test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

        if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        thrVal.thrValidMap = SDL_VTM_LT_TH0_VALID;
        thrVal.ltTh0En = FALSE;
        sdlResult = SDL_VTM_tsSetThresholds(p_cfg1, tsIns, &thrVal);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetThresholds API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

            if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        thrVal.thrValidMap = SDL_VTM_GT_TH1_VALID;
        thrVal.gtTh1En = FALSE;
        sdlResult = SDL_VTM_tsSetThresholds(p_cfg1, tsIns, &thrVal);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetThresholds API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
            if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        thrVal.thrValidMap = SDL_VTM_GT_TH2_VALID;
        thrVal.gtTh2En = FALSE;
        sdlResult = SDL_VTM_tsSetThresholds(p_cfg1, tsIns, &thrVal);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetThresholds API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        thrVal.thrValidMap = (SDL_VTM_LT_TH0_VALID | SDL_VTM_GT_TH1_VALID | SDL_VTM_GT_TH2_VALID);
        sdlResult = SDL_VTM_tsGetThresholds(p_cfg1, SDL_VTM_INSTANCE_TS_0, &thrVal);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsGetThresholds API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        ctrlCfg.maxt_outrg_alert_en = SDL_VTM_TS_CTRL_MAXT_OUTRG_NO_ALERT;
        ctrlCfg.valid_map = SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID;
        sdlResult = SDL_VTM_tsSetCtrl(p_cfg2, tsIns, &ctrlCfg);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetCtrl API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        ctrlCfg.tsReset = SDL_VTM_TS_CTRL_SENSOR_RESET;
        ctrlCfg.valid_map = SDL_VTM_TS_CTRL_RESET_CTRL_VALID;
        sdlResult = SDL_VTM_tsSetCtrl(p_cfg2, tsIns, &ctrlCfg);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetCtrl API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        ctrlCfg.mode = SDL_VTM_TS_CTRL_CONTINUOUS_MODE;
        ctrlCfg.valid_map = SDL_VTM_TS_CTRL_MODE_VALID;
        sdlResult = SDL_VTM_tsSetCtrl(p_cfg2, tsIns, &ctrlCfg);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetCtrl API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }


    if (testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdSetOppVid(p_cfg1, vdIns, 1U, vid_opp_val[0]);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdSetOppVid API test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
   }

       if (testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdSetOppVid(p_cfg1, vdIns, 0U, vid_opp_val[0]);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdSetOppVid API test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
   }

    if (testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdSetOppVid(p_cfg1, vdIns, 2U, vid_opp_val[0]);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdSetOppVid API test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

	if (testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdSetOppVid(p_cfg1, vdIns, 3U, vid_opp_val[0]);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdSetOppVid API test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

       if(testResult == 0)
    {
        tsGlobal_cfg.validMap = 0;
        sdlResult = SDL_VTM_tsSetGlobalCfg(p_cfg2, &tsGlobal_cfg);

        if (sdlResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetGlobalCfg API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        (void)SDL_VTM_getBestValue(100, 200, 300);
        (void)SDL_VTM_getBestValue(200, 100, 300);
        (void)SDL_VTM_getBestValue(200, 300, 100);
        (void)SDL_VTM_getBestValue(300, 200, 100);
        (void)SDL_VTM_getBestValue(300, 100, 200);
        (void)SDL_VTM_getBestValue(191, 146, 142);
        (void)SDL_VTM_getBestValue(63,    25,  25);
        (void)SDL_VTM_getBestValue(62,    60,  50);
    }

    return (testResult);
#endif
#if defined (SOC_AM263PX)
    int32_t                          	testResult = SDL_APP_TEST_PASS;
    int32_t                          	sdlResult;
    uint32_t pTempVal;
    SDL_VTM_adc_code adccode;
    uint32_t ptsenseCTRL;
    SDL_VTM_Stat_val pStat_val;
    SDL_VTM_staticRegsTs pStaticRegs;

    sdlResult = SDL_VTM_initTs(&SDL_VTM_configTempSense);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_initTs API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    SDL_VTM_enableTc();
    SDL_VTM_enableTs(SDL_VTM_SENSOR_SEL0, 0);
    SDL_VTM_enableTs(SDL_VTM_SENSOR_SEL1, 0);
    sdlResult = SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &pTempVal);
    if (sdlResult == SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_getTemp API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }
    sdlResult = SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_1, &pTempVal);
    if (sdlResult == SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_getTemp API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }
    sdlResult = SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_2, &pTempVal);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_getTemp API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }
    sdlResult = SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_3, &pTempVal);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_getTemp API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_setAlertTemp(SDL_VTM_INSTANCE_TS_0, 0, 66000);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_setAlertTemp API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_setAlertTemp(SDL_VTM_INSTANCE_TS_1, 0, 66000);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_setAlertTemp API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_setTShutTemp(SDL_VTM_INSTANCE_TS_0, 66000, 74000);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_setTShutTemp API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_setTShutTemp(SDL_VTM_INSTANCE_TS_1, 66000, 74000);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_setTShutTemp API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }


    sdlResult = SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, SDL_VTM_MASK_HOT, SDL_VTM_MASK_COLD, SDL_VTM_MASK_LOW_TH);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_setClearInterrupts API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_1, SDL_VTM_MASK_HOT, SDL_VTM_MASK_COLD, SDL_VTM_MASK_LOW_TH);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_setClearInterrupts API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_getSensorStatus(&pStat_val);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_getSensorStatus API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_getStaticRegistersTs(&pStaticRegs);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_getStaticRegistersTs API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_enableESMWarmReset(SDL_VTM_INSTANCE_TS_0);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_enableESMWarmReset API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_enableESMWarmReset(SDL_VTM_INSTANCE_TS_1);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_enableESMWarmReset API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    adccode = SDL_VTM_getAdcCode(SDL_VTM_INSTANCE_TS_0);
    if (adccode != 0xFF)
    {
        DebugP_log("\n  SDL_VTM_getAdcCode API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }
    adccode = SDL_VTM_getAdcCode(SDL_VTM_INSTANCE_TS_1);
    if (adccode != 0xFF)
    {
        DebugP_log("\n  SDL_VTM_getAdcCode API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }
    adccode = SDL_VTM_getAdcCode(SDL_VTM_INSTANCE_TS_2);
    if (adccode == 0xFF)
    {
        DebugP_log("\n  SDL_VTM_getAdcCode API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }
    adccode = SDL_VTM_getAdcCode(SDL_VTM_INSTANCE_TS_3);
    if (adccode == 0xFF)
    {
        DebugP_log("\n  SDL_VTM_getAdcCode API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_tsGetCtrl(SDL_VTM_INSTANCE_TS_0, &ptsenseCTRL);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_tsGetCtrl API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    sdlResult = SDL_VTM_tsGetCtrl(SDL_VTM_INSTANCE_TS_1,  &ptsenseCTRL);
    if (sdlResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_tsGetCtrl API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

     return (testResult);

#endif

}
/* Nothing past this point */