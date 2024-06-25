 /* Copyright (c) 2021 Texas Instruments Incorporated
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
 *  \file     sdl_Esm_posTest.h
 *
 *  \brief    This file contains ESM API positive test code.
 *
 **/

#define DISP_APP_ARGB32                   (3U)

#include "esm_test_main.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>

#define SDTF_NUM_RUNALL_TEST_COMMANDS 3
#define MASK_BIT (1u)
#define STATUS_NUM (1u)
#define SDL_ESM_EN_KEY_ENBALE_VAL (0xFU)

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
SDL_ESM_config ESM_esmInitConfig_MAIN_appcallback =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0xfff00fffu, 0xffffffffu, 0x1ffbff, 0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
	  /* CCM_1_SELFTEST_ERR and _R5FSS1_COMPARE_ERR_PULSE_0 */
    .priorityBitmap = {0xfff00fffu, 0xffffff0fu, 0x1ffbff, 0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0xfff00fffu, 0xffffffffu, 0x1ffbff, 0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};
#endif

extern int32_t SDL_ESM_errorInsert (const SDL_ESM_Inst esmInstType,
                              SDL_ESM_ErrorConfig_t *esmErrorConfig);

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                         SDL_ESM_IntType esmIntType,
                                         uint32_t grpChannel,
                                         uint32_t index,
                                         uint32_t intSrc,
                                         void *arg);


int32_t SDTF_runESMInjectHigh_MAIN(void);
int32_t SDTF_runESMInjectHigh_MAIN1(void);
int32_t SDTF_runESMInjectHigh_MAIN2(void);
int32_t sdl_Esm_posTest(void)
{

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
 	SDL_ESM_Inst         i = SDL_ESM_INST_MAIN_ESM0;
	SDL_ESM_Inst Test_instance = i;
#endif

    int32_t              testStatus = SDL_APP_TEST_PASS, apparg;
    SDL_ESM_staticRegs         staticRegs;
    uint32_t esmBaseAddr, val;
    uint32_t esmMaxNumEvents;
    SDL_ESM_Instance_t *pEsmInstancePtr;
    esmOperationMode_t esmOpMode;
    uint32_t influence;
    uint32_t lowTime;
    uint32_t pinCntrPre;
    uint32_t status;
    esmGroupIntrStatus_t intrstatus;
    esmInfo_t info;
    esmRevisionId_t revId;
    SDL_ESM_config pCofnig;
    uint32_t New_SDL_TEST_ESM_BASE;

    New_SDL_TEST_ESM_BASE = (uint32_t) AddrTranslateP_getLocalAddr(SDL_TEST_ESM_BASE);

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)

   /* ESMSetInfluenceOnErrPin API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setNError(i) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* ESMGetInfluenceOnErrPin API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clrNError(i) != SDL_PASS)
        {
            /* Delay some time to allow pin to clear */
            ClockP_usleep(1000);
            if (SDL_ESM_clrNError(i) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
            }
        }
    }

  /* SDL_ESM_getNErrorStatus API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getNErrorStatus(i,&val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
        }

    }

    /* ESMReadStaticRegs API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_ESM_getStaticRegisters(i, &staticRegs)) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
		}
    }

    /* SDL_ESM_registerECCCallback API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_ESM_registerECCCallback(i, ESM_esmInitConfig_MAIN_appcallback.enableBitmap,
                                         SDL_ESM_applicationCallbackFunction, &apparg)) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
        }

    }

    /* SDL_ESM_init API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_ESM_init(i, &pCofnig, NULL, &apparg)) == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getBaseAddr((SDL_ESM_Inst)i, &esmBaseAddr) != true)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getMaxNumEvents((SDL_ESM_Inst)i, &esmMaxNumEvents) != true)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }

    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_selectEsmInst((SDL_ESM_Inst)i, &pEsmInstancePtr) != true)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

#endif

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setMode(ESM_TEST_BASE, ESM_OPERATION_MODE_NORMAL) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setMode(ESM_TEST_BASE, ESM_OPERATION_MODE_ERROR_FORCE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getPinMode(ESM_TEST_BASE, &esmOpMode) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
#if defined (SOC_AM263X)
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getErrorOutMode(ESM_TEST_BASE, &esmOpMode) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
#endif
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setInfluenceOnErrPin(ESM_TEST_BASE, 5U, true) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getInfluenceOnErrPin(ESM_TEST_BASE, 5U, &influence) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setInfluenceOnErrPin(ESM_TEST_BASE, 5U, false) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getInfluenceOnErrPin(ESM_TEST_BASE, 5U, &influence) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setErrPinLowTimePreload(ESM_TEST_BASE, 0x0000FFFFU) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
#if defined (SOC_AM263X)
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_PWML_setErrPinLowTimePreload(ESM_TEST_BASE, 0x0000FFFFU) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_PWMH_setErrPinHighTimePreload(ESM_TEST_BASE, 0x0000FFFFU) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
#endif
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getErrPinLowTimePreload(ESM_TEST_BASE, &lowTime) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
#if defined (SOC_AM263X)
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_PWMH_getErrPinHighTimePreload(ESM_TEST_BASE, &lowTime) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_PWML_getErrPinLowTimePreload(ESM_TEST_BASE, &lowTime) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
#endif
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getCurrErrPinLowTimeCnt(ESM_TEST_BASE, &pinCntrPre) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
#if defined (SOC_AM263X)
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_PWMH_getCurrErrPinHighTimeCnt(ESM_TEST_BASE, &pinCntrPre) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_PWML_getCurrErrPinLowTimeCnt(ESM_TEST_BASE, &pinCntrPre) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
#endif
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getErrPinStatus(ESM_TEST_BASE, &status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_resetErrPin(ESM_TEST_BASE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_enableIntr(ESM_TEST_BASE, 5U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_isEnableIntr(ESM_TEST_BASE, 5U, &status ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_enableIntr(ESM_TEST_BASE, 12U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

        if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_isEnableIntr(ESM_TEST_BASE, 12U, &status ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_disableIntr(ESM_TEST_BASE, 5U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setIntrPriorityLvl(ESM_TEST_BASE, 5U, ESM_INTR_PRIORITY_LEVEL_LOW) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getIntrPriorityLvl(ESM_TEST_BASE, 5U, &status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setIntrPriorityLvl(ESM_TEST_BASE, 5U, ESM_INTR_PRIORITY_LEVEL_HIGH) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getIntrPriorityLvl(ESM_TEST_BASE, 5U, &status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getIntrStatus(ESM_TEST_BASE, 5U, &status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getGroupIntrStatus(ESM_TEST_BASE, ESM_INTR_PRIORITY_LEVEL_LOW, &intrstatus) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getGroupIntrStatus(ESM_TEST_BASE, ESM_INTR_PRIORITY_LEVEL_HIGH, &intrstatus) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_clearIntrStatus(ESM_TEST_BASE, 5U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setIntrStatusRAW(ESM_TEST_BASE, 5U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatusRAW(ESM_TEST_BASE, 5U, &status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_writeEOI(ESM_TEST_BASE, 5U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getRevisionId(ESM_TEST_BASE, &revId) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getInfo(ESM_TEST_BASE, &info) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getGlobalIntrEnabledStatus(ESM_TEST_BASE, &status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableGlobalIntr(ESM_TEST_BASE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableGlobalIntr(ESM_TEST_BASE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_reset(ESM_TEST_BASE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntNumber(Test_instance, SDL_ESM_INT_TYPE_HI) != HI_INTNO)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntNumber(Test_instance, SDL_ESM_INT_TYPE_CFG) != CFG_INTNO)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntNumber(Test_instance, SDL_ESM_INT_TYPE_LO) != LO_INTNO)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }



    /* SDL_ESM_enableCfgIntr API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableCfgIntr(New_SDL_TEST_ESM_BASE, 0x0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableCfgIntr(New_SDL_TEST_ESM_BASE, 31) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableCfgIntr(New_SDL_TEST_ESM_BASE, 0x2) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    /* SDL_ESM_disableCfgIntr API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableCfgIntr(New_SDL_TEST_ESM_BASE, 0x0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableCfgIntr(New_SDL_TEST_ESM_BASE, 31) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableCfgIntr(New_SDL_TEST_ESM_BASE, 0x2) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    /* SDL_ESM_getCfgIntrStatus API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getCfgIntrStatus(New_SDL_TEST_ESM_BASE, 0x0, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getCfgIntrStatus(New_SDL_TEST_ESM_BASE, 31, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getCfgIntrStatus(New_SDL_TEST_ESM_BASE, 0x2, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    /* SDL_ESM_clearCfgIntrStatus API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearCfgIntrStatus(New_SDL_TEST_ESM_BASE, 0x0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearCfgIntrStatus(New_SDL_TEST_ESM_BASE, 31) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearCfgIntrStatus(New_SDL_TEST_ESM_BASE, 0x2) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    /* SDL_ESM_setCfgIntrStatusRAW API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setCfgIntrStatusRAW(New_SDL_TEST_ESM_BASE, 0x0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setCfgIntrStatusRAW(New_SDL_TEST_ESM_BASE, 31) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setCfgIntrStatusRAW(New_SDL_TEST_ESM_BASE, 0x2) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_apiTest: failure on line no %d \r\n", __LINE__);
        }
    }


    /* SDL_ESM_init API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_ESM_init(Test_instance, &pCofnig, NULL, &apparg)) == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        pCofnig.enableBitmap[1] = 0x00180003;
        pCofnig.priorityBitmap[1] = 0x000000ff;
        pCofnig.errorpinBitmap[1] = 0xffffffff;
        i=Test_instance;
        SDL_ESM_init(i, &pCofnig, NULL, &apparg);
        pCofnig.enableBitmap[1] = 0x00000000;
        pCofnig.priorityBitmap[1] = 0x000000ff;
        pCofnig.errorpinBitmap[1] = 0xffffffff;
        if ((SDL_ESM_verifyConfig(i, &pCofnig)) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        pCofnig.esmErrorConfig.groupNumber = 0;
        pCofnig.esmErrorConfig.bitNumber   = 8;
        pCofnig.enableBitmap[1]   = 0x00180003;
        pCofnig.priorityBitmap[1] = 0x000000ff;
        pCofnig.errorpinBitmap[1] = 0xffffffff;
        i=Test_instance;
        SDL_ESM_init(i, &pCofnig, NULL, &apparg);
        pCofnig.esmErrorConfig.groupNumber = 8;
        pCofnig.esmErrorConfig.bitNumber   = 8;
        pCofnig.enableBitmap[1]   = 0x00000000;
        pCofnig.priorityBitmap[1] = 0x000000ff;
        pCofnig.errorpinBitmap[1] = 0xffffffff;
        if ((SDL_ESM_verifyConfig(i, &pCofnig)) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        pCofnig.enableBitmap[1] = 0x00180003u;
        pCofnig.priorityBitmap[1] = 0x000000ff;
        pCofnig.errorpinBitmap[1] = 0xffffffff;
        i=Test_instance;
        SDL_ESM_init(i, &pCofnig, NULL, &apparg);
        if ((SDL_ESM_clrNError(i)) == SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

#if defined (SOC_AM263X)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-7438 */
        if (SDL_ESM_setPinOutMode(i, SDL_ESM_PWM_PINOUT) != SDL_PASS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-7439 */
        if (SDL_ESM_setPinOutMode(i, SDL_ESM_LVL_PINOUT) != SDL_PASS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
#endif
    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2011 */
        if (SDL_ESM_isEnableCfgIntr(0u, 0x0u, &val) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \r\n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2013 */
        if (SDL_ESM_isEnableCfgIntr(New_SDL_TEST_ESM_BASE, 0x0, NULL) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \r\n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2013 */
        if (SDL_ESM_registerCCMCallback(SDL_ESM_INST_MAIN_ESM0,0, &SDL_ESM_applicationCallbackFunction,NULL) != SDL_PASS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \r\n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2013 */
        if (SDTF_runESMInjectHigh_MAIN() != SDL_PASS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \r\n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2013 */
        if (SDTF_runESMInjectHigh_MAIN1() != SDL_PASS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2013 */
        if (SDL_ESM_setCfgIntrStatusRAW(SDL_TOP_ESM_U_BASE, 1u) != SDL_PASS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2013 */
        if (SDTF_runESMInjectHigh_MAIN2() != SDL_PASS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }


return (testStatus);
}

static uint32_t arg;
void  esm_init_appcb(SDL_ESM_Inst esmType)
{
    void *ptr = (void *)&arg;
    SDL_ErrType_t result;
    result = SDL_ESM_init(esmType, &ESM_esmInitConfig_MAIN_appcallback,SDL_ESM_applicationCallbackFunction,ptr);
    if (result != SDL_PASS) {
        /* print error and quit */
        DebugP_log("ESM_ECC_Example_init: Error initializing MAIN ESM: result = %d\r\n", result);

    } else {
        DebugP_log("\r\nESM_ECC_Example_init: Init MAIN ESM complete \r\n");
    }
}
/*********************************************************************
 * @fn      SDTF_runESMInject
 *
 * @brief   Execute ESM Inject
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
static int32_t SDTF_runESMInjectInstance(SDL_ESM_Inst esmType,
                                         uint32_t groupNumber,
                                         uint32_t bitNumber)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ESM_ErrorConfig_t esmErrorConfig;

    esmErrorConfig.groupNumber = groupNumber;
    esmErrorConfig.bitNumber = bitNumber;

    DebugP_log("\n ESM inject: test starting for Esm instance %d\r\n", esmType);

    /* Run esm test 2*/
    result = SDL_ESM_errorInsert(esmType, &esmErrorConfig);

    if (result != SDL_PASS ) {
        DebugP_log("\n ESM inject test for Esm instance %d failed", esmType);
        retVal = -1;
    } else {
        DebugP_log("\n ESM inject test for Esm instance %d Done\r\n", esmType);

    }
    SDL_ESM_clrNError(esmType);
    return retVal;
}

int32_t SDTF_runESMInjectHigh_MAIN(void)
{
    int32_t retVal=0;
    esm_init_appcb(SDL_ESM_INST_MAIN_ESM0);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MAIN_ESM0, 1, 0);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDTF_runESMInjectHigh_MAIN1(void)
{
    int32_t retVal=0;
    esm_init_appcb(SDL_ESM_INST_MAIN_ESM0);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MAIN_ESM0, 1, 4);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDTF_runESMInjectHigh_MAIN2(void)
{
    int32_t retVal=0;
    esm_init_appcb(SDL_ESM_INST_MAIN_ESM0);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MAIN_ESM0, 1, 8);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}