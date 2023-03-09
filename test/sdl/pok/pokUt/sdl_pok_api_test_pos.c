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
 *  \file     sdl_pok_api_test_pos.c
 *
 *  \brief    This file contains POK API unit test code.
 *
 *  \details  POK unit tests
 **/

#include "test_main.h"
#include <sdl/pok/v1/sdl_pok.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/esm/v0/esm.h>

#if defined (SOC_AM64X)
#include <sdl/esm/soc/am64x/sdl_esm_soc.h>
#include <sdl/pok/v1/soc/am64x/sdl_soc_pok.h>
#endif

#if defined (SOC_AM243X)
#include <sdl/esm/soc/am243x/sdl_esm_soc.h>
#include <sdl/pok/v1/soc/am243x/sdl_soc_pok.h>
#endif

#define SDL_POK_TEST_ID SDL_POK_VDDR_CORE_ID


int32_t sdl_pok_posTest(void)
{
    int32_t             testStatus = SDL_APP_TEST_PASS;
    SDL_POK_Inst        i;
    SDL_POK_staticRegs  pStaticRegs;
    SDL_POK_config      pConfig;

    /** hysteresis control */
    SDL_pwrss_hysteresis            hystCtrl;
    /** Voltage Detection Mode control */
    SDL_pwrss_vd_mode               voltDetMode;
    /** POK Trim bits 7 bits wide */
    SDL_pwrss_trim                  trim;
    /** POK Detection Enable */
    SDL_POK_detection               detectionCtrl;
    /** POK Enable Source control */
    SDL_POK_enSelSrc                pokEnSelSrcCtrl;

    /*test functin POK module configurations*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        for (i = SDL_FIRST_POK_ID; i <= SDL_LAST_POK_ID; i++)
        {


            for (hystCtrl = SDL_PWRSS_SET_HYSTERESIS_DISABLE; \
                 hystCtrl <= SDL_PWRSS_HYSTERESIS_NO_ACTION; \
                 hystCtrl++ )

            {
                for (voltDetMode = SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE; \
                     voltDetMode <= SDL_PWRSS_VOLTAGE_DET_NO_ACTION; \
                     voltDetMode++ )
                {
                    for (trim = SDL_PWRSS_MAX_TRIM_VALUE; \
                         trim <= SDL_PWRSS_GET_TRIM_VALUE; \
                         trim++ )
                    {
                        for (detectionCtrl = SDL_POK_DETECTION_DISABLE; \
                             detectionCtrl <= SDL_POK_GET_DETECTION_VALUE; \
                             detectionCtrl++ )
                        {
                            for (pokEnSelSrcCtrl= SDL_POK_ENSEL_HWTIEOFFS; \
                                 pokEnSelSrcCtrl <= SDL_POK_GET_ENSEL_VALUE; \
                                 pokEnSelSrcCtrl++ )
                            {
                                pConfig.hystCtrl         = hystCtrl;
                                pConfig.voltDetMode      = voltDetMode;
                                pConfig.trim             = trim;
                                pConfig.detectionCtrl    = detectionCtrl;
                                pConfig.pokEnSelSrcCtrl  = pokEnSelSrcCtrl;
                                pConfig.hystCtrlOV       = SDL_PWRSS_HYSTERESIS_NO_ACTION;
                                pConfig.trimOV           = SDL_PWRSS_TRIM_NO_ACTION;
                                pConfig.deglitch         = SDL_PWRSS_DEGLITCH_NO_ACTION;
                                SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, false);



                                if (((SDL_POK_init(i,&pConfig)) != SDL_PASS) /*)||
                                    (SDL_POK_verifyConfig(i, &pConfig ) != SDL_PASS)*/)
                                {
                                    if ( (pokEnSelSrcCtrl == SDL_POK_GET_ENSEL_VALUE) ||
                                         (detectionCtrl   == SDL_POK_GET_DETECTION_VALUE) ||
                                         (trim            == SDL_PWRSS_GET_TRIM_VALUE) ||
                                         (voltDetMode     == SDL_PWRSS_GET_VOLTAGE_DET_MODE) ||
                                         (hystCtrl        == SDL_PWRSS_GET_HYSTERESIS_VALUE))
                                    {
                                        /* no break, expected result */
                                    }
                                    else
                                    {
                                        testStatus = SDL_APP_TEST_FAILED;
                                        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

	 SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, false);
     if (testStatus == SDL_APP_TEST_PASS)
    {
        i = SDL_POK_TEST_ID;
        pConfig.voltDetMode  = 246u;

        if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }

        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
    }
     /* function test reads the values of static registers such as hysteresis
     * control,voltage detect mode, trim, PORGAP and module status */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=SDL_FIRST_POK_ID; i<=SDL_LAST_POK_ID; i++)
        {
            if ((SDL_POK_getStaticRegisters(i,&pStaticRegs)) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        // the first POK ID is PMIC and it does not support a trim setting
        for( i=SDL_FIRST_POK_ID+1; i<=SDL_LAST_POK_ID; i++)
        {
            SDL_POK_Inst            esm_inst;
            bool                    usePorCfgFlag;
            uint32_t                esm_err_sig_ov;
            uint32_t                esm_err_sig_uv;

           sdlGetErrSig(i, &esm_inst, &esm_err_sig_uv, &esm_err_sig_ov, &usePorCfgFlag);
            if (esm_err_sig_uv == (uint32_t)(-1))
            {
                continue;
            }
            pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
            pConfig.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
            pConfig.trim            = 0u;
            pConfig.voltDetMode     = SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE;
            pConfig.detectionCtrl   = SDL_POK_DETECTION_ENABLE;
            pConfig.hystCtrlOV      = SDL_PWRSS_HYSTERESIS_NO_ACTION;
            pConfig.trimOV          = SDL_PWRSS_TRIM_NO_ACTION;
            pConfig.deglitch        = SDL_PWRSS_DEGLITCH_NO_ACTION;

            if (SDL_POK_init(i, &pConfig) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d for instance %d\n", __LINE__, i);
                break;
            }
            if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d for instance %d\n", __LINE__, i);
                break;
            }
        }

    }

    /* test Read threshold status of POK module */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=SDL_FIRST_POK_ID; i<=SDL_LAST_POK_ID; i++)
        {
            SDL_POK_Inst            esm_inst;
            bool                    usePorCfgFlag;
            uint32_t                esm_err_sig_ov;
            uint32_t                esm_err_sig_uv;

            sdlGetErrSig(i, &esm_inst, &esm_err_sig_uv, &esm_err_sig_ov, &usePorCfgFlag);
            if (esm_err_sig_ov == (uint32_t)(-1))
            {
                continue;
            }
            pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
            pConfig.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
            pConfig.trim            = 0u;
            pConfig.voltDetMode     = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
            pConfig.detectionCtrl   = SDL_POK_DETECTION_ENABLE;
            pConfig.hystCtrlOV      = SDL_PWRSS_HYSTERESIS_NO_ACTION;
            pConfig.trimOV          = SDL_PWRSS_TRIM_NO_ACTION;
            pConfig.deglitch        = SDL_PWRSS_DEGLITCH_NO_ACTION;

            if (SDL_POK_init(i, &pConfig) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
                break;
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=SDL_FIRST_POK_ID; i<=SDL_LAST_POK_ID; i++)
        {
            pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
            pConfig.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
            pConfig.trim            = 0u;
            pConfig.voltDetMode     = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
            pConfig.detectionCtrl   = SDL_POK_DETECTION_ENABLE;
            pConfig.hystCtrlOV      = SDL_PWRSS_HYSTERESIS_NO_ACTION;
            pConfig.trimOV          = SDL_PWRSS_TRIM_NO_ACTION;
            pConfig.deglitch        = SDL_PWRSS_DEGLITCH_NO_ACTION;
            if (SDL_POK_init(i, &pConfig) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
                break;
            }
        }

    }
	
    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=SDL_FIRST_POK_ID; i<=SDL_LAST_POK_ID; i++)
        {
            pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
            pConfig.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
            pConfig.trim            = SDL_PWRSS_TRIM_NO_ACTION ;
            pConfig.voltDetMode     = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
            pConfig.detectionCtrl   = SDL_POK_DETECTION_ENABLE;
            pConfig.hystCtrlOV      = SDL_PWRSS_HYSTERESIS_NO_ACTION;
            pConfig.trimOV          = SDL_PWRSS_TRIM_NO_ACTION;
            pConfig.deglitch        = SDL_PWRSS_DEGLITCH_NO_ACTION;

            if (SDL_POK_init(i, &pConfig) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
                break;
            }
            if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
                break;
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=SDL_FIRST_POK_ID; i<=SDL_LAST_POK_ID; i++)
        {
            pConfig.trim            = 45u;
            pConfig.voltDetMode     = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;

            if (SDL_POK_init(i, &pConfig) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
                break;
            }
        }

    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=SDL_FIRST_POK_ID; i<=SDL_LAST_POK_ID; i++)
        {
            pConfig.hystCtrl        = 255u;
            pConfig.pokEnSelSrcCtrl = 255u;
            pConfig.trim            = 255u ;
            pConfig.voltDetMode     = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
            pConfig.detectionCtrl   = 255u;

            if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
                break;
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=SDL_FIRST_POK_ID; i<=SDL_LAST_POK_ID; i++)
        {
            pConfig.hystCtrl        = 255u;
            pConfig.pokEnSelSrcCtrl = 255u;
            pConfig.trim            = 255u ;
            pConfig.voltDetMode     = SDL_PWRSS_GET_VOLTAGE_DET_MODE;
            pConfig.detectionCtrl   = 255u;

            if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
                break;
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        i= SDL_LAST_POK_ID;
        pConfig.hystCtrl      = SDL_PWRSS_SET_HYSTERESIS_ENABLE;
        pConfig.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
        pConfig.trim          = 127;
        pConfig.voltDetMode   = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
        pConfig.detectionCtrl = SDL_POK_DETECTION_NO_ACTION;

        if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        i= SDL_LAST_POK_ID;
        pConfig.hystCtrl      = SDL_PWRSS_SET_HYSTERESIS_ENABLE;
        pConfig.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
        pConfig.trim          = 127;
        pConfig.voltDetMode   = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
        pConfig.detectionCtrl = SDL_POK_GET_DETECTION_VALUE;

        if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        i= SDL_LAST_POK_ID;
        pConfig.hystCtrl      = 246u;
        pConfig.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
        pConfig.trim          = 127;
        pConfig.voltDetMode   = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
        pConfig.detectionCtrl = SDL_POK_GET_DETECTION_VALUE;

        if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        i= SDL_LAST_POK_ID;
        pConfig.hystCtrl      = SDL_PWRSS_SET_HYSTERESIS_ENABLE;
        pConfig.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
        pConfig.trim          = 127;
        pConfig.voltDetMode   = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
        pConfig.detectionCtrl = SDL_POK_GET_DETECTION_VALUE;

        if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        i= SDL_LAST_POK_ID;
        pConfig.hystCtrl      = 246u;
        pConfig.pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
        pConfig.trim          = 127;
        pConfig.voltDetMode   = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
        pConfig.detectionCtrl = SDL_POK_GET_DETECTION_VALUE;

        if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        i= SDL_LAST_POK_ID;
        pConfig.hystCtrl        = 255u;
        pConfig.pokEnSelSrcCtrl = 255u;
        pConfig.trim            = 255u;
        pConfig.voltDetMode     = 255u;
        pConfig.detectionCtrl   = 255u;

        if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
	}

	if (testStatus == SDL_APP_TEST_PASS)
    {
        i=SDL_LAST_POK_ID;
        pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pConfig.pokEnSelSrcCtrl = 255u;
        pConfig.trim            = 255u ;
        pConfig.voltDetMode     = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
        pConfig.detectionCtrl   = 255u;
        uint32_t esm_err_sig = MCU_ESM_ERR_SIG_VDDA_MCU_OV;
		SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true);
		SDL_ESM_setInfluenceOnErrPin(SOC_MCU_ESM_BASE, esm_err_sig, (bool)true);

        if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }

        if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }
	}

	if (testStatus == SDL_APP_TEST_PASS)
    {
        i=SDL_LAST_POK_ID;
        pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pConfig.pokEnSelSrcCtrl = 255u;
        pConfig.trim            = 255u ;
        pConfig.voltDetMode     = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
        pConfig.detectionCtrl   = 255u;
		SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true);

        if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }
	}

	if (testStatus == SDL_APP_TEST_PASS)
    {
        i=SDL_LAST_POK_ID;
        pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pConfig.pokEnSelSrcCtrl = 255u;
        pConfig.trim            = 255u ;
        pConfig.voltDetMode     = SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE;
        pConfig.detectionCtrl   = 255u;
		SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true);
        if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }
	}

	if (testStatus == SDL_APP_TEST_PASS)
    {
        i=SDL_POK_VDDA_PMIC_IN_ID;
        pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pConfig.pokEnSelSrcCtrl = 255u;
        pConfig.trim            = 255u ;
        pConfig.voltDetMode     = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
        pConfig.detectionCtrl   = 255u;

        uint32_t esm_err_sig = MCU_ESM_ERR_SIG_VDDA_MCU_OV;
		SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true);
		SDL_ESM_setInfluenceOnErrPin(SOC_MCU_ESM_BASE, esm_err_sig, (bool)true);
        if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }
        if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        i=SDL_POK_VDDA_PMIC_IN_ID;
        pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pConfig.pokEnSelSrcCtrl = 255u;
        pConfig.trim            = 255u ;
        pConfig.voltDetMode     = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
        pConfig.detectionCtrl   = 255u;
		SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true);
        if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }
	}
	if (testStatus == SDL_APP_TEST_PASS)
    {
        i=SDL_POK_VDDA_PMIC_IN_ID;
        pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pConfig.pokEnSelSrcCtrl = 255u;
        pConfig.trim            = 255u ;
        pConfig.voltDetMode     = SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE;
        pConfig.detectionCtrl   = 255u;
		SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true);
        if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }
	}
	if (testStatus == SDL_APP_TEST_PASS)
    {
        i=SDL_LAST_POK_ID;
        pConfig.hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
        pConfig.pokEnSelSrcCtrl = 255u;
        pConfig.trim            = 255u ;
        pConfig.voltDetMode     = SDL_PWRSS_VOLTAGE_DET_NO_ACTION;
        pConfig.detectionCtrl   = 255u;
        pConfig.trimOV = 45u;
		pConfig.deglitch = SDL_PWRSS_DEGLITCH_5US;
		SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true);


		if (SDL_POK_init(i, &pConfig) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }
        if ((SDL_POK_verifyConfig(i, &pConfig )) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

        }
	}

	pConfig.voltDetMode     = SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE;
	if ((SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true)) != SDL_PASS)
    {
        testStatus = SDL_APP_TEST_FAILED;
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
    };
	pConfig.trim = 127U;
	pConfig.trimOV = 127U;
	pConfig.deglitch = 3U;
	pConfig.hystCtrl =  4U;
	pConfig.hystCtrlOV = 1U;
	pConfig.voltDetMode = 1U;
	pConfig.pokEnSelSrcCtrl = 1U;
	pConfig.detectionCtrl =1U;
	if (SDL_POK_init(SDL_POK_VDDSHV_MCU_3P3_ID, &pConfig) != SDL_EFAIL)
    {
        testStatus = SDL_APP_TEST_FAILED;
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
    }

	if ((SDL_POK_enablePP(SDL_POK_PRG_PP_0_ID, true)) != SDL_EFAIL)
    {
        testStatus = SDL_APP_TEST_FAILED;
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);

    }
	if (SDL_POK_init(SDL_POR_VDD_MCU_UV_ID, &pConfig) != SDL_EBADARGS)
    {
        testStatus = SDL_APP_TEST_FAILED;
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
    }

	pConfig.trim = 127U;
	pConfig.trimOV = 127U;
	pConfig.deglitch = 3U;
	pConfig.hystCtrl =  4U;
	pConfig.hystCtrlOV = 1U;
	pConfig.voltDetMode = 1U;
	if (SDL_POK_init(SDL_POR_VDD_MCU_UV_ID, &pConfig) != SDL_EBADARGS)
    {
        testStatus = SDL_APP_TEST_FAILED;
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
    }


	if ((SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true)) != SDL_PASS)
    {
        testStatus = SDL_APP_TEST_FAILED;
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
	}
	pConfig.trim = 127U;
	pConfig.trimOV = 127U;
	pConfig.deglitch = 3U;
	pConfig.hystCtrl =  4U;
	pConfig.hystCtrlOV = 1U;
	pConfig.detectionCtrl = 1U;
	pConfig.pokEnSelSrcCtrl = 1U;
	pConfig.voltDetMode = SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE;
	if (SDL_POK_init(SDL_POK_VDDR_CORE_ID, &pConfig) != SDL_PASS)
    {
        testStatus = SDL_APP_TEST_FAILED;
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
    }
	pConfig.trim = 127U;
	pConfig.trimOV = 127U;
	pConfig.deglitch = 2U;
	pConfig.hystCtrl =  4U;
	pConfig.hystCtrlOV = 1U;
	pConfig.detectionCtrl = 1U;
	pConfig.pokEnSelSrcCtrl = 1U;
	pConfig.voltDetMode = SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE;
	if (SDL_POK_verifyConfig(SDL_POR_VDD_MCU_UV_ID, &pConfig) != SDL_EFAIL)
    {
        testStatus = SDL_APP_TEST_FAILED;
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
    }

	pConfig.hystCtrl =  4U;
	pConfig.hystCtrlOV = 1U;
	pConfig.voltDetMode = SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE;
	pConfig.trimOV = 45u;
	pConfig.trim = 127U;
	if (SDL_POK_verifyConfig(SDL_POR_VDD_MCU_UV_ID, &pConfig) != SDL_EFAIL)
    {
        testStatus = SDL_APP_TEST_FAILED;
        DebugP_log("SDLPok_api_Pos_Test: failure on line no. %d \n", __LINE__);
    }
    return (testStatus);
}
