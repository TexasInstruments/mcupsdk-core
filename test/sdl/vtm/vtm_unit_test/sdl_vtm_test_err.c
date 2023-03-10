/* Copyright (c) 2023 Texas Instruments Incorporated
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
 *  \file     vtm_test_err.c
 *
 *  \brief    This file contains vtm test code.
 *
 *  \details  VTM negative tests
 **/

#include "sdl_vtm_test_main.h"

extern int32_t gNumTempSensors;
extern int32_t gNumCoreVoltageDomains;

int32_t sdlVTM_errTest(void)
{
    int32_t                             i;
    SDL_VTM_configVd                  	vdConfig;
    SDL_VTM_configTs                  	tsConfig;
    int32_t                          	testResult = SDL_APP_TEST_PASS;
    int32_t                          	sdlResult;
    uint32_t                          	tempVal;
    SDL_VTM_Stat_val                  	statVal;
    SDL_VTM_Stat_read_ctrl              rdCtrl;
    SDL_VTM_staticRegsVd              	vdStaticRegs;
    SDL_VTM_staticRegsTs              	tsStaticRegs;
    SDL_VTM_InstVd                      vdIns;
    SDL_VTM_InstTs                      tsIns;
    SDL_VTM_intrCtrl                 	intrControl;
    SDL_VTM_Ctrlcfg                 	ctrlCfg;
    SDL_VTM_adc_code                	adc_code;
    uint8_t                         	vid_opp_val[4];
    int32_t                         	tempValmDegree;
    const SDL_VTM_cfg1Regs              *p_cfg1;
    const SDL_VTM_cfg2Regs              *p_cfg2;
    uint32_t baseAddr,baseAddr1;
	
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;
	
	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_2, &baseAddr1);
    p_cfg2 = (SDL_VTM_cfg2Regs *) baseAddr1;
    SDL_VTM_tsGlobal_cfg             tsGlobal_cfg;
    SDL_VTM_tsThrVal             thrVal;
    vdConfig.configVdCtrl = (SDL_VTM_VD_CONFIG_CTRL_EVT_SEL     |    \
                             SDL_VTM_VD_CONFIG_CTRL_GLB_CFG);
    tsConfig.configTsCtrl = (SDL_VTM_VD_CONFIG_CTRL_SET_CTL     |    \
                             SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT |    \
                             SDL_VTM_VD_CONFIG_CTRL_SET_THR);

    sdlResult = SDL_VTM_initVd(SDL_VTM_INSTANCE_VD_DOMAIN_CNT, NULL);
    if (sdlResult == SDL_PASS)
    {
        DebugP_log("\n  SDL_VTM_initVd negative test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    if(testResult == 0)
    {
        for (vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0; vdIns < gNumCoreVoltageDomains; vdIns++ )
        {
            sdlResult = SDL_VTM_initVd(vdIns, NULL);

            if (sdlResult == SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_initVd negative test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }
	
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_initVd(SDL_VTM_INSTANCE_VD_DOMAIN_CNT, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initVd negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
	
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_initVd(SDL_VTM_INSTANCE_VD_DOMAIN_CNT, &vdConfig);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initVd negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
	
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_initTs(SDL_VTM_INSTANCE_TS_MAX_NUM, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initTs negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_initTs((SDL_VTM_InstTs)i, NULL);

            if (sdlResult == SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_initTs negative test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }
	
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_initTs(SDL_VTM_INSTANCE_TS_MAX_NUM, &tsConfig);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_initTs negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
	
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_MAX_NUM, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_getTemp negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
	
    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_getTemp((SDL_VTM_InstTs)i, NULL);

            if (sdlResult == SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_getTemp negative test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }
	
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_MAX_NUM, &tempVal);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_getTemp negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getSensorStatus(SDL_VTM_INSTANCE_TS_MAX_NUM, NULL, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_getSensorStatus negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_getSensorStatus((SDL_VTM_InstTs)i, NULL, &statVal);

            if (sdlResult == SDL_PASS)
            {
                DebugP_log("SDL_VTM_getSensorStatus negative test failed on line no. %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getSensorStatus(SDL_VTM_INSTANCE_TS_0, &rdCtrl, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_getSensorStatus negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getSensorStatus(SDL_VTM_INSTANCE_TS_MAX_NUM, NULL, &statVal);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_getSensorStatus negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getSensorStatus(SDL_VTM_INSTANCE_TS_MAX_NUM, &rdCtrl, &statVal);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_getSensorStatus negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_CNT, SDL_VTM_VD_INTR_INVALID);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_intrCntrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        sdlResult = SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, SDL_VTM_VD_INTR_INVALID);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_intrCntrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_CNT, SDL_VTM_VD_LT_THR0_INTR_RAW_SET);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_intrCntrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        intrControl = SDL_VTM_VD_GT_THR1_INTR_RAW_SET | SDL_VTM_VD_GT_THR1_INTR_RAW_CLR;
        sdlResult = SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, intrControl);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_intrCntrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        intrControl = SDL_VTM_VD_GT_THR2_INTR_RAW_SET | SDL_VTM_VD_GT_THR2_INTR_RAW_CLR;
        sdlResult = SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, intrControl);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_intrCntrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        intrControl = SDL_VTM_VD_LT_THR0_INTR_EN_SET | SDL_VTM_VD_LT_THR0_INTR_EN_CLR;
        sdlResult = SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, intrControl);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_intrCntrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        intrControl = SDL_VTM_VD_GT_THR1_INTR_EN_SET | SDL_VTM_VD_GT_THR1_INTR_EN_CLR;
        sdlResult = SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, intrControl);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_intrCntrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        intrControl = SDL_VTM_VD_GT_THR2_INTR_EN_SET | SDL_VTM_VD_GT_THR2_INTR_EN_CLR;
        sdlResult = SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, intrControl);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_intrCntrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_CNT, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_verifyConfigVd((SDL_VTM_InstVd)i, NULL);

            if (sdlResult == SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_verifyConfigVd negative test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_verifyConfigVd(SDL_VTM_INSTANCE_VD_DOMAIN_CNT, &vdConfig);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigVd negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_verifyConfigTs(SDL_VTM_INSTANCE_TS_MAX_NUM, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_verifyConfigTs((SDL_VTM_InstTs)i, NULL);

            if (sdlResult == SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_verifyConfigTs negative test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_verifyConfigTs(SDL_VTM_INSTANCE_TS_MAX_NUM, &tsConfig);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_verifyConfigTs negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getStaticRegistersVd(SDL_VTM_INSTANCE_VD_DOMAIN_CNT, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_getStaticRegistersVd negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_VD_DOMAIN_0; i < gNumCoreVoltageDomains; i++ )
        {
            sdlResult = SDL_VTM_getStaticRegistersVd((SDL_VTM_InstVd)i, NULL);

            if (sdlResult == SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_getStaticRegistersVd negative test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }

    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getStaticRegistersVd(SDL_VTM_INSTANCE_VD_DOMAIN_CNT, &vdStaticRegs);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_getStaticRegistersVd negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getStaticRegistersTs(SDL_VTM_INSTANCE_TS_MAX_NUM, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_getStaticRegistersTs negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        for (i = SDL_VTM_INSTANCE_TS_0; i < gNumTempSensors; i++ )
        {
            sdlResult = SDL_VTM_getStaticRegistersTs((SDL_VTM_InstTs)i, NULL);

            if (sdlResult == SDL_PASS)
            {
                DebugP_log("\n  SDL_VTM_getStaticRegistersTs negative test failed on line no: %d \n", __LINE__);
                testResult = -1;
                break;
            }
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_getStaticRegistersTs(SDL_VTM_INSTANCE_TS_MAX_NUM, &tsStaticRegs);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_getStaticRegistersTs negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsGetGlobalCfg(p_cfg2, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsGetGlobalCfg negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsGetGlobalCfg(NULL, &tsGlobal_cfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsGetGlobalCfg negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsSetGlobalCfg(p_cfg2, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetGlobalCfg negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsSetGlobalCfg(NULL, &tsGlobal_cfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetGlobalCfg negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_3;
        sdlResult = SDL_VTM_vdEvtSelSet(p_cfg1, vdIns, 0U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_vdEvtSelSet negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdEvtSelSet(NULL, vdIns, 0U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_vdEvtSelSet negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        sdlResult = SDL_VTM_tsGetThresholds(NULL, tsIns, &thrVal);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsGetThresholds negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsGetThresholds(p_cfg1, (SDL_VTM_InstTs)(SDL_VTM_TS_MAX_NUM+1), &thrVal);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsGetThresholds negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        sdlResult = SDL_VTM_tsGetThresholds(p_cfg1, tsIns, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsGetThresholds negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        sdlResult = SDL_VTM_tsSetThresholds(NULL, tsIns, &thrVal);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetThresholds negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = (SDL_VTM_InstTs)(SDL_VTM_TS_MAX_NUM+1);
        sdlResult = SDL_VTM_tsSetThresholds(p_cfg1, tsIns, &thrVal);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetThresholds negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        sdlResult = SDL_VTM_tsSetThresholds(p_cfg1, tsIns, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetThresholds negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        sdlResult = SDL_VTM_tsGetCtrl(NULL, tsIns, &ctrlCfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsGetCtrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = (SDL_VTM_InstTs)(SDL_VTM_TS_MAX_NUM+1);
        sdlResult = SDL_VTM_tsGetCtrl(p_cfg2, tsIns, &ctrlCfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsGetCtrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        sdlResult = SDL_VTM_tsGetCtrl(p_cfg2, tsIns, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsGetCtrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        sdlResult = SDL_VTM_tsSetCtrl(NULL, tsIns, &ctrlCfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetCtrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        tsIns = (SDL_VTM_InstTs)(SDL_VTM_TS_MAX_NUM+1);
        sdlResult = SDL_VTM_tsSetCtrl(p_cfg2, tsIns, &ctrlCfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetCtrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        tsIns = SDL_VTM_INSTANCE_TS_0;
        sdlResult = SDL_VTM_tsSetCtrl(p_cfg2, tsIns, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_VTM_tsSetCtrl negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdGetOppVid(NULL, vdIns, 1U, &vid_opp_val[0]);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdGetOppVid negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_5;
        sdlResult = SDL_VTM_vdGetOppVid(p_cfg1, vdIns, 1U, &vid_opp_val[0]);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdGetOppVid negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdGetOppVid(p_cfg1, vdIns, 5U, &vid_opp_val[0]);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdGetOppVid negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdGetOppVid(p_cfg1, vdIns, 1U, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdGetOppVid negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdSetOppVid(NULL, vdIns, 1U, vid_opp_val[0]);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdSetOppVid negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_5;
        sdlResult = SDL_VTM_vdSetOppVid(p_cfg1, vdIns, 1U, vid_opp_val[0]);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdSetOppVid negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdSetOppVid(p_cfg1, vdIns, 5U, vid_opp_val[0]);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdSetOppVid negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        vdIns = SDL_VTM_INSTANCE_VD_DOMAIN_0;
        sdlResult = SDL_VTM_vdSetOppVid(p_cfg1, vdIns, 5U, vid_opp_val[0]);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_vdSetOppVid negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsSetMaxTOutRgAlertThr(NULL, SDL_VTM_INSTANCE_TS_0, 68000, 64000);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_tsSetMaxTOutRgAlertThr negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsSetMaxTOutRgAlertThr(p_cfg2, SDL_VTM_INSTANCE_TS_MAX_NUM, 68000, 64000);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_tsSetMaxTOutRgAlertThr negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsConvTempToAdc(160000, SDL_VTM_INSTANCE_TS_0, &adc_code);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_tsConvTempToAdc negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsConvTempToAdc(-45000, SDL_VTM_INSTANCE_TS_0, &adc_code);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_tsConvTempToAdc negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsConvTempToAdc(156000, SDL_VTM_INSTANCE_TS_0, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_tsConvTempToAdc negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsConvTempToAdc(156000, SDL_VTM_INSTANCE_TS_MAX_NUM, &adc_code);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_tsConvTempToAdc negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        gNumTempSensors = (-1);
        sdlResult = SDL_VTM_tsConvADCToTemp(-20, SDL_VTM_INSTANCE_TS_MAX_NUM, &tempValmDegree);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_tsConvADCToTemp negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        gNumTempSensors = (-1);
        sdlResult = SDL_VTM_tsConvADCToTemp(-20, SDL_VTM_INSTANCE_TS_0, NULL);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_tsConvADCToTemp negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if(testResult == 0)
    {
        sdlResult = SDL_VTM_tsConvADCToTemp(1024, SDL_VTM_INSTANCE_TS_0, &tempValmDegree);

        if (sdlResult == SDL_PASS)
        {
            DebugP_log("SDL_VTM_tsConvADCToTemp negative test failed on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    return (testResult);
}
/* Nothing past this point */
