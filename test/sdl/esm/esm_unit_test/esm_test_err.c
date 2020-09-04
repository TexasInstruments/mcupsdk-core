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
 *  \file     sdl_Esm_negTest.h
 *
 *  \brief    This file contains ESM API negetive test code.
 *
 **/

#include "esm_test_main.h"
#include <sdl/sdl_esm.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#if defined (SOC_AM64X)
#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
#include <sdl/esm/soc/am64x/sdl_esm_core.h>
#define SDL_TEST_ESM_BASE  SDL_MCU_ESM0_CFG_BASE

static SDL_ESM_config ESM_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 0u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
     /**< No events enable */
    .priorityBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                        },
    /**< All events low priority */
    .errorpinBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                      },
};

#endif


extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                         SDL_ESM_IntType esmIntType,
                                         uint32_t grpChannel,
                                         uint32_t index,
                                         uint32_t intSrc,
                                         void *arg);


  /* This file contains ESM Negtive test code.*/
int32_t sdl_Esm_negTest(void)
{
    SDL_ESM_Inst         instance = SDL_ESM_INSTANCE_MAX;
    int32_t              testStatus = SDL_APP_TEST_PASS;
    int32_t apparg;
    uint32_t             val;
    SDL_ESM_staticRegs         staticRegs;
    SDL_ESM_Inst         i;
    uint32_t esmBaseAddr;
    SDL_ESM_Instance_t *pEsmInstancePtr = (SDL_ESM_Instance_t *)NULL;
    uint32_t esmMaxNumEvents;
    uint32_t influence;
    esmOperationMode_t esmOpMode;
    uint32_t lowTime;
    uint32_t pinCntrPre;
    uint32_t status;
    esmGroupIntrStatus_t intrstatus;
    esmRevisionId_t revId;
    esmInfo_t info;
    uint32_t New_SDL_TEST_ESM_BASE;

    New_SDL_TEST_ESM_BASE = (uint32_t) AddrTranslateP_getLocalAddr(SDL_TEST_ESM_BASE);

    SDL_ESM_config pCofnig;

   /* ESMSetInfluenceOnErrPin negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setNError(instance) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setNError((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /* ESMGetInfluenceOnErrPin negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clrNError(instance) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


   /* ESMGetErrPinStatus negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getNErrorStatus(instance,&val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INST_MCU_ESM0;
        if (SDL_ESM_getNErrorStatus(instance, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2011 */
        if (SDL_ESM_isEnableCfgIntr(0x0u, 0x0u, &val) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2012 */
        if (SDL_ESM_isEnableCfgIntr(SDL_TEST_ESM_BASE, 32, &val) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Test case: PROC_SDL-2013 */
        if (SDL_ESM_isEnableCfgIntr(SDL_TEST_ESM_BASE, 0x0, NULL) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableCfgIntr(0x0u, 0x0) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableCfgIntr(SDL_TEST_ESM_BASE, 32) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableCfgIntr(0x0u, 0x0) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableCfgIntr(SDL_TEST_ESM_BASE, 32) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }



    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getCfgIntrStatus(0x0u, 0x0, &val) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getCfgIntrStatus(New_SDL_TEST_ESM_BASE, 32, &val) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getCfgIntrStatus(New_SDL_TEST_ESM_BASE, 0x0, NULL) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearCfgIntrStatus(0x0u, 0x0) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearCfgIntrStatus(SDL_TEST_ESM_BASE, 32) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setCfgIntrStatusRAW(0x0u, 0x0) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setCfgIntrStatusRAW(SDL_TEST_ESM_BASE, 32) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setCfgIntrStatusRAW(0x0u, 0x0) != SDL_EBADARGS)
        {
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_getNErrorStatus(instance, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }



    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_getNErrorStatus(instance, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getNErrorStatus((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    /* SDL_ESM_getStaticRegisters negative test */

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_getStaticRegisters(instance, &staticRegs) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }



    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INST_MCU_ESM0;
        if (SDL_ESM_getStaticRegisters(instance, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }



    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_getStaticRegisters(instance, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getStaticRegisters((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


     /* SDL_ESM_resetErrPin negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_resetErrPin(0x0u) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    /*  Negative test for API SDL_ESM_registerECCCallback  */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if ((SDL_ESM_registerECCCallback(instance, ESM_esmInitConfig_MCU.enableBitmap,
                                             SDL_ESM_applicationCallbackFunction, &apparg) != SDL_EFAIL))
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    /*  Negative test for API SDL_ESM_init  */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INST_MCU_ESM0;
        if (SDL_ESM_init(instance, NULL, NULL, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }



    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if ((SDL_ESM_init(instance, &pCofnig, NULL, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }



    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if ((SDL_ESM_init(instance, NULL, NULL, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }



    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if ((SDL_ESM_init(instance, NULL, NULL, &apparg)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_init((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD, NULL, NULL, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_init((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD, &pCofnig, SDL_ESM_applicationCallbackFunction, &apparg) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_verifyConfig((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD, &pCofnig)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /* SDL_ESM_verifyConfig API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_verifyConfig(instance, &pCofnig) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

/* sdl_esm_core.c APIs start     */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=1;i<=SDL_ESM_INST_MAIN_ESM0; i++)
        {
            if (SDL_ESM_getBaseAddr((SDL_ESM_Inst)i, NULL) != false)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getBaseAddr(SDL_ESM_INSTANCE_MAX, &esmBaseAddr) != false)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=1;i<=SDL_ESM_INST_MAIN_ESM0; i++)
        {
            if (SDL_ESM_getMaxNumEvents((SDL_ESM_Inst)i, NULL) != false)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getMaxNumEvents(SDL_ESM_INSTANCE_MAX, &esmMaxNumEvents) != false)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_selectEsmInst(SDL_ESM_INSTANCE_MAX, &pEsmInstancePtr) != false)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }


/* sdl_esm_core.c APIs end     */

/* sdl_ip_esm.c APIs start     */

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setMode(0U, ESM_OPERATION_MODE_ERROR_FORCE) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getPinMode(0U, &esmOpMode) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getPinMode(SDL_ESM0_CFG_BASE, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setInfluenceOnErrPin(0U, 5U, true) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setInfluenceOnErrPin(SDL_ESM0_CFG_BASE, 1025U, false) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getInfluenceOnErrPin(0U, 5U, &influence) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getInfluenceOnErrPin(SDL_ESM0_CFG_BASE, 1025U, &influence) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getInfluenceOnErrPin(SDL_ESM0_CFG_BASE, 5U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setErrPinLowTimePreload(0U, 0x0000FFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setErrPinLowTimePreload(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getErrPinLowTimePreload(0U, &lowTime) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getErrPinLowTimePreload(SDL_ESM0_CFG_BASE, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getCurrErrPinLowTimeCnt(0U, &pinCntrPre) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getCurrErrPinLowTimeCnt(SDL_ESM0_CFG_BASE, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getErrPinStatus(0U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getErrPinStatus(SDL_ESM0_CFG_BASE, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_resetErrPin(0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_isEnableIntr(0U, 5U, &status ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_isEnableIntr(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU, &status ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_isEnableIntr(SDL_ESM0_CFG_BASE, 5U, NULL ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableIntr(0U, 5U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableIntr(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableIntr(0U, 5U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableIntr(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setIntrPriorityLvl(0U, 5U, ESM_INTR_PRIORITY_LEVEL_LOW) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setIntrPriorityLvl(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU, ESM_INTR_PRIORITY_LEVEL_LOW) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrPriorityLvl(0U, 5U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrPriorityLvl(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrPriorityLvl(SDL_ESM0_CFG_BASE, 5U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatus(0U, 5U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatus(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatus(SDL_ESM0_CFG_BASE, 5U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getGroupIntrStatus(0U, ESM_INTR_PRIORITY_LEVEL_LOW, &intrstatus) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getGroupIntrStatus(SDL_ESM0_CFG_BASE, ESM_INTR_PRIORITY_LEVEL_LOW, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearIntrStatus(0U, 5U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearIntrStatus(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setIntrStatusRAW(0U, 5U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setIntrStatusRAW(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatusRAW(0U, 5U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatusRAW(SDL_ESM0_CFG_BASE, 0xFFFFFFFFU, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatusRAW(SDL_ESM0_CFG_BASE, 5U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_writeEOI(0U, 5U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getRevisionId(0U, &revId) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getInfo(0U, &info) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getGlobalIntrEnabledStatus(0U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getGlobalIntrEnabledStatus(SDL_ESM0_CFG_BASE, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableGlobalIntr(0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableGlobalIntr(0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_reset(0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

/* sdl_ip_esm.c APIs end     */


    return (testStatus);
}
