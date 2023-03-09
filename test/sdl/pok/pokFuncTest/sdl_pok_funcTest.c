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
 *  \file     sdl_pok_funcTest.c
 *
 *  \brief    This file contains POK API functionality test code.
 *
 *  \details  POK functionality tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "pok_main.h"
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/esm/v0/v0_0/sdl_esm_priv.h>
#include <sdl/pok/v1/sdl_ip_pok.h>
#include <sdl/pok/v1/sdl_pok.h>

#if defined (SOC_AM64X)
#include <sdl/pok/v1/soc/am64x/sdl_soc_pok.h>
#endif
#if defined (SOC_AM243X)
#include <sdl/pok/v1/soc/am243x/sdl_soc_pok.h>
#endif

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */
 
/* Global variables */

volatile Bool ESM_Error = false;
uint32_t deactivate_trigger(uint32_t *esm_err_sig );
static void sdlGetInstance(SDL_POK_Inst *instance, uint32_t *esm_err_sig);


/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
static int32_t sdlPOK_Test(SDL_POK_Inst instance, SDL_POK_config *pPokCfg);
static void    sdlEsmSetupForPOK(uint32_t esm_err_sig);
int32_t        sdlPOKInPor_funcTest(void);
int32_t        sdlPOK_funcTest(void);


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
    DebugP_log("\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("  Take action \n");
    /* Disable the ESM Interrupt */
    deactivate_trigger(&intSrc);
    SDL_ESM_clrNError(SDL_ESM_INST_MCU_ESM0);
	SDL_ESM_disableIntr(SDL_MCU_ESM0_CFG_BASE,intSrc);
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
    else
	{
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

static void sdlPOKGetUVOV(uint32_t id, uint32_t *isOV)
{
    switch (id)
    {   
	     
        case SDL_POR_VDDA_MCU_UV_ID:
        case SDL_POR_VDD_MCU_UV_ID:
            *isOV = 0x0;
	    break;
	case SDL_POK_VDD_MCU_OV_ID:
        case SDL_POR_VDDA_MCU_OV_ID:
            *isOV = 0x1;
            break;
     
        default:
            *isOV = 0x2;
	    break;
    }
    return;
}
	
void sdlEsmSetupForPOK(uint32_t esm_err_sig)
{
    /* ESM Variables */
    esmInfo_t   appEsmInfo;
    uint32_t     esmBaseAddr;
#if defined (SOC_AM64X)
#if defined (M4F_CORE)	
	SDL_ESM_getBaseAddr(SDL_ESM_INST_MCU_ESM0,&esmBaseAddr);
#endif
#endif

#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (R5F_CORE)	
	SDL_ESM_getBaseAddr(SDL_ESM_INST_MAIN_ESM0,&esmBaseAddr);
#endif
#endif

    /* Check INFO register for ESM last reset cause */
    SDL_ESM_getInfo(esmBaseAddr, &appEsmInfo);

    /* The below function can be changed to force an error for diagnostic
     * reasons. */
    /* make sure we're not in force error mode */
    SDL_ESM_setMode(esmBaseAddr, ESM_OPERATION_MODE_NORMAL);

    /* Enable this ESM Error Signal */
    SDL_ESM_enableIntr(esmBaseAddr, esm_err_sig);

    /* Set the output interrupt priority level */
    SDL_ESM_setIntrPriorityLvl(esmBaseAddr, esm_err_sig, ESM_INTR_PRIORITY_LEVEL_HIGH);

    /* Enable Error Pin on this ESM Error Signal */
    SDL_ESM_setInfluenceOnErrPin(esmBaseAddr, esm_err_sig, TRUE);

    /* Enable for all ESM Error Signals */
    SDL_ESM_enableGlobalIntr(esmBaseAddr);
}

static int32_t sdlPOK_Test(SDL_POK_Inst instance, SDL_POK_config *pPokCfg)
{
    int32_t sdlRet;
    SDL_POK_staticRegs pStaticRegs;
    pPokCfg->hystCtrl        = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg->hystCtrlOV      = SDL_PWRSS_HYSTERESIS_NO_ACTION;
    pPokCfg->deglitch        = SDL_PWRSS_DEGLITCH_NO_ACTION;
    pPokCfg->pokEnSelSrcCtrl = SDL_POK_ENSEL_PRG_CTRL;
    pPokCfg->detectionCtrl   = SDL_POK_DETECTION_ENABLE;

    sdlRet = SDL_POK_getStaticRegisters(instance,&pStaticRegs);
    if (sdlRet == SDL_PASS )
    {
        DebugP_log ("\n static trim value is = %d \n", pStaticRegs.trim);
    }
    else{
        DebugP_log ("\n\n SDL_POK_getStaticRegisters Failed \n");
    }
    sdlRet = SDL_POK_init(instance, pPokCfg);

    if (sdlRet != SDL_PASS)
    {
        DebugP_log("SDL_POK_init failed! \n");
    }
    sdlRet = SDL_POK_verifyConfig(instance,pPokCfg );
    if (sdlRet != SDL_PASS)
    {
        DebugP_log("SDL_POK_verifyConfig failed! \n");
    }

   else
    {
        volatile int32_t i = 0;
        DebugP_log("Waiting for ESM to report the error \n");
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
         DebugP_log(" Got the ESM Error Interrupt \n");
            sdlRet  = SDL_PASS;
            ESM_Error = false;
            }
            else
            {
                sdlRet = SDL_EFAIL;
        }
    }
    return (sdlRet);
}

int32_t sdlPOKInPor_funcTest(void)
{
    int32_t                      testStatus, sdlRet = SDL_PASS, overallStatus = SDL_APP_TEST_PASS;
    SDL_POK_config               pPokCfg;
    uint32_t                     esm_err_sig_uv, esm_err_sig_ov;
    SDL_POK_Inst                 instance;
    bool                         usePorCfgFlag;
    uint32_t                     i;
    uint32_t                     isOV;
    SDL_POK_staticRegs           pStaticRegs;
    DebugP_log(" Below are the POK In POR ID values for the test\n");
    DebugP_log("  SDL_POR_VDDA_MCU_UV_ID is:            9 \n");
    DebugP_log("  SDL_POR_VDD_MCU_UV_ID is:             10 \n");
	DebugP_log("  SDL_POR_VDDA_MCU_OV_ID is:            11 \n");

    DebugP_log(" Enter the Voltage Detection (0: UV, 1: OV, 2: PP) for the POK ID to monitor for the test  \n");

    instance = SDL_POR_VDDA_MCU_UV_ID;
    DebugP_log ("\n\nDefault test loops through POK IDs \n", instance);
    pPokCfg.voltDetMode = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
    pPokCfg.trim = 0;

    sdlRet = SDL_POK_getStaticRegisters(instance,&pStaticRegs);
    if (sdlRet == SDL_PASS )
    {
        DebugP_log ("\n\bandGap status %d\n",  pStaticRegs.porBGapOK );
        DebugP_log ("\n\nmodule status %d\n",  pStaticRegs.porModuleStatus);
    }
    else{
        DebugP_log("SDL_POK_getStaticRegisters failed! \n");
    }


    for (i = SDL_POR_VDDA_MCU_UV_ID; i < SDL_POR_VDDA_MCU_OV_ID + 1; i++)
	{
		sdlGetErrSig(i, &instance, &esm_err_sig_uv, &esm_err_sig_ov, &usePorCfgFlag);
        sdlPOKGetUVOV(i, &isOV);

        if (isOV == 0x1)
        {
            pPokCfg.voltDetMode = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;

            pPokCfg.trimOV = 0;
	    pPokCfg.trim = SDL_PWRSS_TRIM_NO_ACTION;
		}
        else
        {
            pPokCfg.voltDetMode = SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE;
            pPokCfg.trim = 127;
	    pPokCfg.trimOV = SDL_PWRSS_TRIM_NO_ACTION;
        }

        /* ESM Setup for POK tests */
		sdlEsmSetupForPOK(esm_err_sig_uv);
		if (sdlRet == SDL_PASS)
        {
            sdlRet = sdlPOK_Test(instance, &pPokCfg);
            /* Re-configure to "good" setting */
            if (isOV == 0x1)
	        {
                pPokCfg.trimOV = 45;
                pPokCfg.trim = SDL_PWRSS_TRIM_NO_ACTION;

            }
            else
            {
                pPokCfg.trim = 0;
                pPokCfg.trimOV = SDL_PWRSS_TRIM_NO_ACTION;
			}
            
            sdlPOK_Test(instance, &pPokCfg);
            /* Un register the Interrupt */

        }

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_PASS;
        }
        else
        {
            testStatus = SDL_APP_TEST_FAILED;
            overallStatus = SDL_APP_TEST_FAILED;
        }
        DebugP_log("Test for instance %d %s\n\n", instance, (testStatus == SDL_APP_TEST_PASS) ? "PASSED" : "FAILED");
    }
    return (overallStatus);
}


int32_t sdlPOK_funcTest(void)
{
    int32_t                      testStatus, sdlRet = SDL_PASS, overallStatus = SDL_APP_TEST_PASS;
    SDL_POK_config               pPokCfg;
    bool                         usePorCfgFlag;

    uint32_t                     esm_err_sig_uv, esm_err_sig_ov;
    SDL_POK_Inst                 instance;
    uint32_t                     i;
    uint32_t                     isOV;
    SDL_POK_staticRegs           pStaticRegs;

    
    DebugP_log(" Below are the POK ID values \n");
    DebugP_log("  SDL_POK_VDDA_PMIC_IN_ID is:            0 \n");
	DebugP_log("  SDL_POK_VDDS_DDRIO_ID is:              1 \n");
    DebugP_log("  SDL_POK_VDDR_CORE_ID is:               2 \n");
    DebugP_log("  SDL_POK_VDDSHV_MCU_3P3_ID is:          3 \n");
    DebugP_log("  SDL_POK_VDDSHV_MCU_1P8_ID is:          4 \n");
    DebugP_log("  SDL_POK_VMON_CAP_MCU_GENERAL_ID is:    5 \n");
    DebugP_log("  SDL_POK_VDDSHV_MAIN_1P8_ID is:         6 \n");
	DebugP_log("  SDL_POK_VDDSHV_MAIN_3P3_ID is:         7 \n");
	DebugP_log("  SDL_POK_VDD_MCU_OV_ID is:              8 \n");
				   
				   

    DebugP_log(" Enter the Voltage Detection (0: UV, 1: OV, 2: PP) for the POK ID to monitor for the test  \n");
	DebugP_log("\n\nDefault test cycles through POKs, monitoring set to OV \n");
    pPokCfg.voltDetMode = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
    pPokCfg.trim = 0;

    for (i = SDL_FIRST_POK_ID + 1; i < SDL_LAST_POK_ID + 1; i++)
	{
        sdlGetErrSig(i, &instance, &esm_err_sig_uv, &esm_err_sig_ov, &usePorCfgFlag);
        sdlPOKGetUVOV(i,&isOV);
        if (isOV == 0x1 )
        {
            pPokCfg.voltDetMode = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
	        pPokCfg.trim = SDL_PWRSS_TRIM_NO_ACTION;
            pPokCfg.trimOV = 0;
        }
        else if (isOV == 0x0)
        {
            pPokCfg.voltDetMode = SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE;
            pPokCfg.trim = 127;
	        pPokCfg.trimOV = SDL_PWRSS_TRIM_NO_ACTION;
		}
	else
	{
           
	    SDL_POK_enablePP(SDL_POK_PRG_PP_1_ID, true);
            /* This is Ping/Pong */
            pPokCfg.voltDetMode = SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE;
            pPokCfg.trim = 127;
            pPokCfg.trimOV = 0;
	}

        /* ESM Setup for POK tests */
	sdlEsmSetupForPOK(esm_err_sig_uv);
	sdlEsmSetupForPOK(esm_err_sig_ov);

        if (sdlRet == SDL_PASS)
        {
            sdlRet = SDL_POK_getStaticRegisters(instance,&pStaticRegs);
            if (sdlRet == SDL_PASS )
            {
                 DebugP_log ("\n static trim value is %d \n",pStaticRegs.trim );
                 DebugP_log ("\n static UV/OV value is %d \n", pStaticRegs.voltDetMode);
            }

            sdlRet = sdlPOK_Test(instance, &pPokCfg);

            if (sdlRet == SDL_PASS)
            {
                testStatus = SDL_APP_TEST_PASS;
            }
            else
            {
                testStatus = SDL_APP_TEST_FAILED;
                overallStatus = SDL_APP_TEST_FAILED;
            }
            /* Re-set the trim value to a good value */
            if (isOV == 0x1)
            {
                pPokCfg.trim = SDL_PWRSS_TRIM_NO_ACTION;
                pPokCfg.trimOV = 45;
            }
            else if (isOV == 0x0)
            {
                pPokCfg.trim = 0;
		        pPokCfg.trimOV = SDL_PWRSS_TRIM_NO_ACTION;
            }
	    else
	    {
                pPokCfg.trim = 0;
		pPokCfg.trimOV = 45;
	    }

        sdlPOK_Test(instance, &pPokCfg);

        }
        DebugP_log("Test for instance %d %s\n\n", instance, (testStatus == SDL_APP_TEST_PASS) ? "PASSED" : "FAILED");
    }
    return (overallStatus);
}

static void sdlGetInstance(SDL_POK_Inst *instance, uint32_t *esm_err_sig)
{
    switch (*esm_err_sig)
    {   
	    case MCU_ESM_ERR_SIG_VDDA_PMIC_IN_UV:
            *instance = SDL_POK_VDDA_PMIC_IN_ID;
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
		case MCU_ESM_ERR_SIG_VDD_MCU_OV:
             *instance = SDL_POK_VDD_MCU_OV_ID;
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