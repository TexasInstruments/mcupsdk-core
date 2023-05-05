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
 *  \file     sdl_pok_example.c
 *
 *  \brief    This file contains POK example code.
 *
 *  \details  POK example
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "pok_main.h"
#include <sdl/include/am64x_am243x/sdlr_intr_mcu_esm0.h>
#include <sdl/pok/v1/sdl_ip_pok.h>
#include <sdl/sdl_esm.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/pok/v1/soc/sdl_soc_pok.h>



/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */

/* Global variables */


/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
int32_t        SDL_POK_setConfig(SDL_POK_Inst instance, SDL_POK_config *pPokCfg);
int32_t        sdlPOKInPor_func(void);
int32_t        sdlPOK_func(void);
int32_t        SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);
volatile bool ESM_Error = false;
uint32_t deactivate_trigger(uint32_t *esm_err_sig );
static void sdlGetInstance(SDL_POK_Inst *instance, uint32_t *esm_err_sig);

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg)
{
    int32_t retVal = SDL_PASS;
    DebugP_log("\r\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("  Take action \r\n");
    /* Disable the ESM Interrupt */
    deactivate_trigger(&intSrc);
    SDL_ESM_clrNError(esmInst);
    ESM_Error = true;
    /* Any additional customer specific actions can be added here */

    return retVal;
}

uint32_t deactivate_trigger(uint32_t *esm_err_sig )
{
    SDL_POK_Inst               instance;
    SDL_POK_config             pPokCfg;
    SDL_pokVal_t               pPokVal;
    uint32_t pbaseAddress;
    SDL_POK_getBaseaddr(SDL_POK_MCU_CTRL_MMR0, &pbaseAddress);
    SDL_mcuCtrlRegsBase_t    *pBaseAddr = (SDL_mcuCtrlRegsBase_t *) pbaseAddress;
    int32_t                    sdlRet = SDL_EFAIL;

    sdlGetInstance(&instance, esm_err_sig);

    pPokCfg.hystCtrl = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg.voltDetMode = SDL_PWRSS_GET_VOLTAGE_DET_MODE;
    pPokCfg.trim = SDL_PWRSS_TRIM_NO_ACTION;
    pPokCfg.detectionCtrl = SDL_POK_DETECTION_NO_ACTION;
    pPokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
    pPokCfg.hystCtrlOV = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg.trimOV = SDL_PWRSS_TRIM_NO_ACTION;
    pPokCfg.deglitch = SDL_PWRSS_DEGLITCH_NO_ACTION;


    SDL_pokGetControl (pBaseAddr,&pPokCfg,&pPokVal,instance);
    /* Re-configure to "good" setting */
    if (pPokVal.voltDetMode == SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE)
    {
        pPokCfg.trim = 0;

        pPokCfg.trimOV = SDL_PWRSS_TRIM_NO_ACTION;

    }
     else if(pPokVal.voltDetMode == SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE)
    {

        pPokCfg.trimOV = 45;
        pPokCfg.trim = SDL_PWRSS_TRIM_NO_ACTION;
    }
	else{
        pPokCfg.trim = 0;
    }

    pPokCfg.hystCtrl = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg.voltDetMode = pPokVal.voltDetMode;
    pPokCfg.detectionCtrl = SDL_POK_DETECTION_NO_ACTION;
    pPokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
    pPokCfg.hystCtrlOV = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg.deglitch = SDL_PWRSS_DEGLITCH_NO_ACTION;
    sdlRet = SDL_POK_init(instance,&pPokCfg);

    return sdlRet;
}

int32_t SDL_POK_setConfig(SDL_POK_Inst instance, SDL_POK_config *pPokCfg)
{
    int32_t sdlRet = SDL_EFAIL;
	volatile int32_t i = 0;
    sdlRet = SDL_POK_init(instance, pPokCfg);
    if (sdlRet != SDL_PASS)
    {
        DebugP_log("SDL_POK_init failed! \r\n");
    }
    else
    {
        DebugP_log("Waiting for ESM to report the error \r\n");
        /* Wait for the ESM interrupt to report the error */
        do {
            i++;
            if (i > 0x0FFFFFFF)
            {
                /* Timeout for the wait */
                break;
            }
        } while (ESM_Error == false);

        if (ESM_Error == true)
        {
            DebugP_log(" Got the ESM Error Interrupt \r\n");
            DebugP_log("Action taken \r\n");
            ESM_Error = false;
            if (sdlRet != SDL_PASS)
            {
                DebugP_log("SDL_POK_init failed! \r\n");
            }
        }
        else
        {
            sdlRet = SDL_EFAIL;
        }
    }
    return(sdlRet);
}


int32_t sdlPOKInPor_func(void)
{
    int32_t                      testStatus, sdlRet = SDL_PASS, overallStatus = SDL_APP_TEST_PASS;
    SDL_POK_config               pPokCfg;
    SDL_POK_Inst                 instance;
    instance = SDL_POR_VDDA_MCU_OV_ID;


    DebugP_log("\r\n\r\n POK ID = %d , monitoring set to OV \r\n", instance);
    pPokCfg.voltDetMode = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
    pPokCfg.trim = 0;
    pPokCfg.hystCtrl = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg.detectionCtrl = SDL_POK_DETECTION_NO_ACTION;
    pPokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
    pPokCfg.hystCtrlOV = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg.trimOV = 0;
    pPokCfg.deglitch = SDL_PWRSS_DEGLITCH_NO_ACTION;


    sdlRet = SDL_POK_setConfig(instance, &pPokCfg);
    if (sdlRet == SDL_PASS)
    {
        testStatus = SDL_APP_TEST_PASS;
    }
    else
    {
        testStatus = SDL_APP_TEST_FAILED;
        overallStatus = SDL_APP_TEST_FAILED;
    }
    DebugP_log("Safety software Example UC-2 pok for instance  %d %s\r\n",
                instance, (testStatus == SDL_APP_TEST_PASS) ? "PASSED" : "FAILED");

    return (overallStatus);
}

int32_t sdlPOK_func(void)
{
    int32_t                      testStatus, sdlRet = SDL_PASS, overallStatus = SDL_APP_TEST_PASS;
    SDL_POK_config               pPokCfg;
    SDL_POK_Inst                 instance;
    instance = SDL_POK_VDDS_DDRIO_ID;


    DebugP_log ("\r\n\r\n POK ID = %d , monitoring set to UV \r\n", instance);
    pPokCfg.voltDetMode = SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE;
    pPokCfg.trim = 127;
    pPokCfg.hystCtrl = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg.detectionCtrl = SDL_POK_DETECTION_NO_ACTION;
    pPokCfg.pokEnSelSrcCtrl = SDL_POK_ENSEL_NO_ACTION;
    pPokCfg.hystCtrlOV = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg.trimOV = SDL_PWRSS_TRIM_NO_ACTION;
    pPokCfg.deglitch = SDL_PWRSS_DEGLITCH_NO_ACTION;


    sdlRet = SDL_POK_setConfig(instance, &pPokCfg);
    if (sdlRet == SDL_PASS)
    {
        testStatus = SDL_APP_TEST_PASS;
    }
    else
    {
        testStatus = SDL_APP_TEST_FAILED;
        overallStatus = SDL_APP_TEST_FAILED;
    }
    DebugP_log("Safety software Example UC-1 pok for instance %d %s\r\n",
                instance, (testStatus == SDL_APP_TEST_PASS) ? "PASSED" : "FAILED");

    return (overallStatus);
}

static void sdlGetInstance(SDL_POK_Inst *instance, uint32_t *esm_err_sig)
{
    switch (*esm_err_sig)
    {
	    case MCU_ESM_ERR_SIG_VDDA_PMIC_IN_UV:
            *instance = SDL_POK_VDDA_PMIC_IN_ID;
            break;
        case MCU_ESM_ERR_SIG_VDD_MCU_OV:
             *instance = SDL_POK_VDD_MCU_OV_ID;
             break;
        case MCU_ESM_ERR_SIG_VDDS_DDRIO_UV:
        case MCU_ESM_ERR_SIG_VDDS_DDRIO_OV:
            *instance    = SDL_POK_VDDS_DDRIO_ID;
            break;
        case MCU_ESM_ERR_SIG_VDDR_CORE_UV:
        case MCU_ESM_ERR_SIG_VDDR_CORE_OV:
            *instance    = SDL_POK_VDDR_CORE_ID;
            break;
        case MCU_ESM_ERR_SIG_VDDSHV_MCU_3P3_UV:
        case MCU_ESM_ERR_SIG_VDDSHV_MCU_3P3_OV:
            *instance    = SDL_POK_VDDSHV_MCU_3P3_ID;
            break;
        case MCU_ESM_ERR_SIG_VDDSHV_MCU_1P8_UV:
        case MCU_ESM_ERR_SIG_VDDSHV_MCU_1P8_OV:
            *instance    = SDL_POK_VDDSHV_MCU_1P8_ID;
            break;
        case MCU_ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_UV:
        case MCU_ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_OV:
            *instance = SDL_POK_VMON_CAP_MCU_GENERAL_ID;
            break;
        case MCU_ESM_ERR_SIG_VDDSHV_MAIN_1P8_UV:
        case MCU_ESM_ERR_SIG_VDDSHV_MAIN_1P8_OV:
            *instance = SDL_POK_VDDSHV_MAIN_1P8_ID;
            break;
        case MCU_ESM_ERR_SIG_VDDSHV_MAIN_3P3_UV:
        case MCU_ESM_ERR_SIG_VDDSHV_MAIN_3P3_OV:
            *instance = SDL_POK_VDDSHV_MAIN_3P3_ID;
            break;

		case MCU_ESM_ERR_SIG_VDDA_MCU_UV:
             *instance = SDL_POR_VDDA_MCU_UV_ID;
             break;
        case MCU_ESM_ERR_SIG_VDDA_MCU_OV:
             *instance = SDL_POR_VDDA_MCU_OV_ID;
             break;
		case MCU_ESM_ERR_SIG_VDD_MCU_UV:
             *instance = SDL_POR_VDD_MCU_UV_ID;
             break;
        default:
            *instance = (SDL_POK_Inst)(-1);
            break;
    }
    return;
}
/* Nothing past this point */